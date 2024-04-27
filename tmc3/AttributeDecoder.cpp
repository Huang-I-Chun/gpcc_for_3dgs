/* The copyright in this software is being made available under the BSD
 * Licence, included below.  This software may be subject to other third
 * party and contributor rights, including patent rights, and no such
 * rights are granted under this licence.
 *
 * Copyright (c) 2017-2018, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the ISO/IEC nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "AttributeDecoder.h"

#include "AttributeCommon.h"
#include "DualLutCoder.h"
#include "attribute_raw.h"
#include "constants.h"
#include "entropy.h"
#include "hls.h"
#include "io_hls.h"
#include "RAHT.h"
#include "FixedPoint.h"

namespace pcc {

//============================================================================
// An encapsulation of the entropy decoding methods used in attribute coding

class PCCResidualsDecoder : protected AttributeContexts {
public:
  PCCResidualsDecoder(
    const AttributeBrickHeader& abh, const AttributeContexts& ctxtMem);

  EntropyDecoder arithmeticDecoder;

  const AttributeContexts& getCtx() const { return *this; }

  void start(const SequenceParameterSet& sps, const char* buf, int buf_len);
  void stop();

  int decodeRunLength();
  int decodeSymbol(int k1, int k2, int k3);
  void decode(int32_t values[3]);
  int32_t decode();
};

//----------------------------------------------------------------------------

PCCResidualsDecoder::PCCResidualsDecoder(
  const AttributeBrickHeader& abh, const AttributeContexts& ctxtMem)
  : AttributeContexts(ctxtMem)
{}

//----------------------------------------------------------------------------

void
PCCResidualsDecoder::start(
  const SequenceParameterSet& sps, const char* buf, int buf_len)
{
  arithmeticDecoder.setBuffer(buf_len, buf);
  arithmeticDecoder.enableBypassStream(sps.cabac_bypass_stream_enabled_flag);
  arithmeticDecoder.setBypassBinCodingWithoutProbUpdate(
    sps.bypass_bin_coding_without_prob_update);
  arithmeticDecoder.start();
}

//----------------------------------------------------------------------------

void
PCCResidualsDecoder::stop()
{
  arithmeticDecoder.stop();
}

//----------------------------------------------------------------------------

int
PCCResidualsDecoder::decodeRunLength()
{
  int runLength = 0;
  auto* ctx = ctxRunLen;
  for (; runLength < 3; runLength++, ctx++) {
    int bin = arithmeticDecoder.decode(*ctx);
    if (!bin)
      return runLength;
  }

  for (int i = 0; i < 4; i++) {
    int bin = arithmeticDecoder.decode(*ctx);
    if (!bin) {
      runLength += arithmeticDecoder.decode();
      return runLength;
    }
    runLength += 2;
  }

  runLength += arithmeticDecoder.decodeExpGolomb(2, *++ctx);
  return runLength;
}

//----------------------------------------------------------------------------

int
PCCResidualsDecoder::decodeSymbol(int k1, int k2, int k3)
{
  if (!arithmeticDecoder.decode(ctxCoeffGtN[0][k1]))
    return 0;

  if (!arithmeticDecoder.decode(ctxCoeffGtN[1][k2]))
    return 1;

  int coeff_abs_minus2 = arithmeticDecoder.decodeExpGolomb(
    1, ctxCoeffRemPrefix[k3], ctxCoeffRemSuffix[k3]);

  return coeff_abs_minus2 + 2;
}

//----------------------------------------------------------------------------

void
PCCResidualsDecoder::decode(int32_t value[3])
{
  value[1] = decodeSymbol(0, 0, 1);
  int b0 = value[1] == 0;
  int b1 = value[1] <= 1;
  value[2] = decodeSymbol(1 + b0, 1 + b1, 1);
  int b2 = value[2] == 0;
  int b3 = value[2] <= 1;
  value[0] = decodeSymbol(3 + (b0 << 1) + b2, 3 + (b1 << 1) + b3, 0);

  if (b0 && b2)
    value[0] += 1;

  if (value[0] && arithmeticDecoder.decode())
    value[0] = -value[0];
  if (value[1] && arithmeticDecoder.decode())
    value[1] = -value[1];
  if (value[2] && arithmeticDecoder.decode())
    value[2] = -value[2];
}

//----------------------------------------------------------------------------

int32_t
PCCResidualsDecoder::decode()
{
  auto mag = decodeSymbol(0, 0, 0) + 1;
  bool sign = arithmeticDecoder.decode();
  return sign ? -mag : mag;
}

//============================================================================
// AttributeDecoderIntf

AttributeDecoderIntf::~AttributeDecoderIntf() = default;

//============================================================================
// AttributeDecoder factory

std::unique_ptr<AttributeDecoderIntf>
makeAttributeDecoder()
{
  return std::unique_ptr<AttributeDecoder>(new AttributeDecoder());
}

//============================================================================
// AttributeDecoder Members

void
AttributeDecoder::decode(
  const SequenceParameterSet& sps,
  const AttributeDescription& attr_desc,
  const AttributeParameterSet& attr_aps,
  const AttributeBrickHeader& abh,
  int geom_num_points_minus1,
  int minGeomNodeSizeLog2,
  const char* payload,
  size_t payloadLen,
  AttributeContexts& ctxtMem,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  std::cout << "AttributeDecoder::decode start" << std::endl;
  if (attr_aps.attr_encoding == AttributeEncoding::kRaw) {
    AttrRawDecoder::decode(
      attr_desc, attr_aps, abh, payload, payloadLen, pointCloud);
    return;
  }

  QpSet qpSet = deriveQpSet(attr_desc, attr_aps, abh);

  PCCResidualsDecoder decoder(abh, ctxtMem);
  decoder.start(sps, payload, payloadLen);

  // generate LoDs if necessary
  if (attr_aps.lodParametersPresent() && _lods.empty())
    _lods.generate(
      attr_aps, abh, geom_num_points_minus1, minGeomNodeSizeLog2, pointCloud,
      attrInterPredParams);

  std::cout << "attr_desc.attr_num_dimensions_minus1 = "
            << attr_desc.attr_num_dimensions_minus1 << std::endl;

  if (attr_desc.attr_num_dimensions_minus1 == 0) {  // for reflect
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodeReflectancesRaht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;

    case AttributeEncoding::kPredictingTransform:
      decodeReflectancesPred(
        attr_desc, attr_aps, abh, qpSet, decoder, pointCloud,
        attrInterPredParams);
      break;

    case AttributeEncoding::kLiftingTransform:
      decodeReflectancesLift(
        attr_desc, attr_aps, abh, qpSet, geom_num_points_minus1,
        minGeomNodeSizeLog2, decoder, pointCloud, attrInterPredParams);
      break;

    case AttributeEncoding::kRaw:
      // Already handled
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 2) {  // for color
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodeColorsRaht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;

    case AttributeEncoding::kPredictingTransform:
      decodeColorsPred(attr_desc, attr_aps, abh, qpSet, decoder, pointCloud);
      break;

    case AttributeEncoding::kLiftingTransform:
      decodeColorsLift(
        attr_desc, attr_aps, abh, qpSet, geom_num_points_minus1,
        minGeomNodeSizeLog2, decoder, pointCloud);
      break;

    case AttributeEncoding::kRaw:
      // Already handled
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 3) {  // for opacity
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodeOpacitiesRaht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodeOpacitiesRaht for opacity deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 4) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_dc_0Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodef_dc_0Raht for f_dc_0 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 5) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_dc_1Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodef_dc_1Raht for f_dc_1 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 6) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_dc_2Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodef_dc_2Raht for f_dc_2 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 7) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodescale_0Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodescale_0Raht for scale_0 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 8) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodescale_1Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodescale_1Raht for scale_1 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 9) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodescale_2Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodescale_2Raht for scale_2 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 10) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decoderot_0Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decoderot_0Raht for rot_0 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 11) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decoderot_1Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decoderot_1Raht for rot_1 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 12) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decoderot_2Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decoderot_2Raht for rot_2 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 13) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decoderot_3Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decoderot_3Raht for rot_3 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 14) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_0Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodef_rest_0Raht for f_rest_0 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 15) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_1Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodef_rest_1Raht for f_rest_1 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 16) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_2Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodef_rest_2Raht for f_rest_2 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 17) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_3Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodef_rest_3Raht for f_rest_3 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 18) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_4Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodef_rest_4Raht for f_rest_4 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 19) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_5Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodef_rest_5Raht for f_rest_5 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 20) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_6Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodef_rest_6Raht for f_rest_6 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 21) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_7Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodef_rest_7Raht for f_rest_7 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 22) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_8Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodef_rest_8Raht for f_rest_8 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 23) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_9Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout
        << "Currently we only provide decodef_rest_9Raht for f_rest_9 deocode"
        << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 24) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_10Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_10Raht for "
                   "f_rest_10 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 25) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_11Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_11Raht for "
                   "f_rest_11 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 26) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_12Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_12Raht for "
                   "f_rest_12 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 27) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_13Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_13Raht for "
                   "f_rest_13 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 28) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_14Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_14Raht for "
                   "f_rest_14 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 29) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_15Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_15Raht for "
                   "f_rest_15 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 30) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_16Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_16Raht for "
                   "f_rest_16 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 31) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_17Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_17Raht for "
                   "f_rest_17 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 32) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_18Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_18Raht for "
                   "f_rest_18 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 33) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_19Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_19Raht for "
                   "f_rest_19 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 34) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_20Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_20Raht for "
                   "f_rest_20 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 35) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_21Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_21Raht for "
                   "f_rest_21 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 36) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_22Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_22Raht for "
                   "f_rest_22 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 37) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_23Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_23Raht for "
                   "f_rest_23 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 38) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_24Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_24Raht for "
                   "f_rest_24 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 39) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_25Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_25Raht for "
                   "f_rest_25 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 40) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_26Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_26Raht for "
                   "f_rest_26 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 41) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_27Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_27Raht for "
                   "f_rest_27 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 42) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_28Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_28Raht for "
                   "f_rest_28 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 43) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_29Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_29Raht for "
                   "f_rest_29 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 44) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_30Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_30Raht for "
                   "f_rest_30 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 45) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_31Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_31Raht for "
                   "f_rest_31 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 46) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_32Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_32Raht for "
                   "f_rest_32 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 47) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_33Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_33Raht for "
                   "f_rest_33 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 48) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_34Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_34Raht for "
                   "f_rest_34 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 49) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_35Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_35Raht for "
                   "f_rest_35 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 50) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_36Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_36Raht for "
                   "f_rest_36 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 51) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_37Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_37Raht for "
                   "f_rest_37 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 52) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_38Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_38Raht for "
                   "f_rest_38 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 53) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_39Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_39Raht for "
                   "f_rest_39 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 54) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_40Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_40Raht for "
                   "f_rest_40 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 55) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_41Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_41Raht for "
                   "f_rest_41 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 56) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_42Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_42Raht for "
                   "f_rest_42 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 57) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_43Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_43Raht for "
                   "f_rest_43 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else if (attr_desc.attr_num_dimensions_minus1 == 58) {
    QpSet qpSet = deriveQpSet(
      attr_desc, attr_aps, abh, attr_desc.attr_num_dimensions_minus1);
    switch (attr_aps.attr_encoding) {
    case AttributeEncoding::kRAHTransform:
      decodef_rest_44Raht(
        attr_desc, attr_aps, qpSet, decoder, pointCloud, attrInterPredParams);
      break;
    default:
      std::cout << "Currently we only provide decodef_rest_44Raht for "
                   "f_rest_44 deocode"
                << std::endl;
      exit(0);
      break;
    }
  } else {
    assert(
      attr_desc.attr_num_dimensions_minus1 == 0
      || attr_desc.attr_num_dimensions_minus1 == 2
      || attr_desc.attr_num_dimensions_minus1 == 3);
  }

  decoder.stop();

  // save the context state for re-use by a future slice if required
  ctxtMem = decoder.getCtx();
}

//----------------------------------------------------------------------------

bool
AttributeDecoder::isReusable(
  const AttributeParameterSet& aps, const AttributeBrickHeader& abh) const
{
  return _lods.isReusable(aps, abh);
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decodePredModeRefl(
  const AttributeParameterSet& aps, int32_t& coeff, PCCPredictor& predictor)
{
  int coeffAbs = abs(coeff);
  int coeffSign = coeff < 0 ? -1 : 1;
  int mode;

  int maxcand =
    aps.max_num_direct_predictors + !aps.direct_avg_predictor_disabled_flag;
  switch (maxcand) {
  case 4:
    mode = coeffAbs & 3;
    coeff = coeffSign * (coeffAbs >> 2);
    break;

  case 3:
    mode = coeffAbs & 1;
    coeffAbs >>= 1;
    if (mode > 0) {
      mode += coeffAbs & 1;
      coeffAbs >>= 1;
    }
    coeff = coeffSign * coeffAbs;
    break;

  case 2:
    mode = coeffAbs & 1;
    coeff = coeffSign * (coeffAbs >> 1);
    break;

  default: mode = 0;
  }

  predictor.predMode = mode + aps.direct_avg_predictor_disabled_flag;
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decodeReflectancesPred(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const AttributeBrickHeader& abh,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  const AttributeInterPredParams& attrInterPredParams)
{
  const size_t pointCount = pointCloud.getPointCount();
  const int64_t maxReflectance = (1ll << desc.bitdepth) - 1;

  int zeroRunRem = 0;
  int quantLayer = 0;

  std::vector<uint64_t> quantWeights;
  if (!aps.scalable_lifting_enabled_flag) {
    computeQuantizationWeights(
      _lods.predictors, quantWeights, aps.quant_neigh_weight,
      attrInterPredParams.enableAttrInterPred);
  } else {
    computeQuantizationWeightsScalable(
      _lods.predictors, _lods.numPointsInLod, pointCount, 0, quantWeights);
  }

  for (size_t predictorIndex = 0; predictorIndex < pointCount;
       ++predictorIndex) {
    if (predictorIndex == _lods.numPointsInLod[quantLayer]) {
      quantLayer = std::min(int(qpSet.layers.size()) - 1, quantLayer + 1);
    }
    const uint32_t pointIndex = _lods.indexes[predictorIndex];
    auto quant = qpSet.quantizers(pointCloud[pointIndex], quantLayer);
    auto& predictor = _lods.predictors[predictorIndex];
    predictor.predMode = 0;

    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    int32_t attValue0 = 0;
    if (!zeroRunRem)
      attValue0 = decoder.decode();

    if (predModeEligibleRefl(
          desc, aps, pointCloud, _lods.indexes, predictor,
          attrInterPredParams))
      decodePredModeRefl(aps, attValue0, predictor);

    attr_t& reflectance = pointCloud.getReflectance(pointIndex);
    const int64_t quantPredAttValue = predictor.predictReflectance(
      pointCloud, _lods.indexes, attrInterPredParams);

    int64_t qStep = quant[0].stepSize();
    int64_t weight =
      std::min(static_cast<int64_t>(quantWeights[predictorIndex]), qStep)
      >> kFixedPointWeightShift;
    int64_t delta =
      divExp2RoundHalfUp(quant[0].scale(attValue0), kFixedPointAttributeShift);
    delta /= weight;

    const int64_t reconstructedQuantAttValue = quantPredAttValue + delta;
    reflectance =
      attr_t(PCCClip(reconstructedQuantAttValue, int64_t(0), maxReflectance));
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decodePredModeColor(
  const AttributeParameterSet& aps,
  Vec3<int32_t>& coeff,
  PCCPredictor& predictor)
{
  int signk1 = coeff[1] < 0 ? -1 : 1;
  int signk2 = coeff[2] < 0 ? -1 : 1;
  int coeffAbsk1 = abs(coeff[1]);
  int coeffAbsk2 = abs(coeff[2]);

  int mode;
  int maxcand =
    aps.max_num_direct_predictors + !aps.direct_avg_predictor_disabled_flag;
  switch (maxcand) {
    int parityk1, parityk2;
  case 4:
    parityk1 = coeffAbsk1 & 1;
    parityk2 = coeffAbsk2 & 1;
    coeff[1] = signk1 * (coeffAbsk1 >> 1);
    coeff[2] = signk2 * (coeffAbsk2 >> 1);

    mode = (parityk1 << 1) + parityk2;
    break;

  case 3:
    parityk1 = coeffAbsk1 & 1;
    coeff[1] = signk1 * (coeffAbsk1 >> 1);
    mode = parityk1;
    if (parityk1) {
      parityk2 = coeffAbsk2 & 1;
      coeff[2] = signk2 * (coeffAbsk2 >> 1);
      mode += parityk2;
    }
    break;

  case 2:
    parityk1 = coeffAbsk1 & 1;
    coeff[1] = signk1 * (coeffAbsk1 >> 1);
    mode = parityk1;
    break;

  default: assert(maxcand >= 2); mode = 0;
  }

  predictor.predMode = mode + aps.direct_avg_predictor_disabled_flag;
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decodeColorsPred(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const AttributeBrickHeader& abh,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud)
{
  const size_t pointCount = pointCloud.getPointCount();

  int64_t clipMax = (1 << desc.bitdepth) - 1;
  Vec3<int32_t> values;

  bool icpPresent = abh.icpPresent(desc, aps);
  auto icpCoeff = icpPresent ? abh.icpCoeffs[0] : 0;

  int lod = 0;
  int zeroRunRem = 0;
  int quantLayer = 0;

  std::vector<uint64_t> quantWeights;
  if (!aps.scalable_lifting_enabled_flag) {
    computeQuantizationWeights(
      _lods.predictors, quantWeights, aps.quant_neigh_weight);
  } else {
    computeQuantizationWeightsScalable(
      _lods.predictors, _lods.numPointsInLod, pointCount, 0, quantWeights);
  }

  for (size_t predictorIndex = 0; predictorIndex < pointCount;
       ++predictorIndex) {
    if (predictorIndex == _lods.numPointsInLod[quantLayer]) {
      quantLayer = std::min(int(qpSet.layers.size()) - 1, quantLayer + 1);
    }
    const uint32_t pointIndex = _lods.indexes[predictorIndex];
    auto quant = qpSet.quantizers(pointCloud[pointIndex], quantLayer);
    auto& predictor = _lods.predictors[predictorIndex];
    predictor.predMode = 0;

    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    if (zeroRunRem)
      values[0] = values[1] = values[2] = 0;
    else
      decoder.decode(&values[0]);

    if (predModeEligibleColor(desc, aps, pointCloud, _lods.indexes, predictor))
      decodePredModeColor(aps, values, predictor);

    Vec3<attr_t>& color = pointCloud.getColor(pointIndex);
    const Vec3<attr_t> predictedColor =
      predictor.predictColor(pointCloud, _lods.indexes);

    if (icpPresent && predictorIndex == _lods.numPointsInLod[lod])
      icpCoeff = abh.icpCoeffs[++lod];

    int64_t residual0 = 0;
    for (int k = 0; k < 3; ++k) {
      const auto& q = quant[std::min(k, 1)];

      int64_t qStep = q.stepSize();
      int64_t weight =
        std::min(static_cast<int64_t>(quantWeights[predictorIndex]), qStep)
        >> kFixedPointWeightShift;
      int64_t residual =
        divExp2RoundHalfUp(q.scale(values[k]), kFixedPointAttributeShift);
      residual /= weight;

      const int64_t recon =
        predictedColor[k] + residual + ((icpCoeff[k] * residual0 + 2) >> 2);
      color[k] = attr_t(PCCClip(recon, int64_t(0), clipMax));

      if (!k && aps.inter_component_prediction_enabled_flag)
        residual0 = residual;
    }
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decodeReflectancesRaht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  std::cout << "AttributeDecoder::decodeReflectancesRaht start" << std::endl;
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());

  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }

  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);

  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }

  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;

  std::vector<int> attributes(attribCount * voxelCount);

  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }

    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());

    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getReflectance(
          packedVoxel_ref[n].index);
    }
  }

  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);

  std::cout << "desc.bitdepth = " << desc.bitdepth << std::endl;
  std::cout << "maxReflectance = " << (1 << desc.bitdepth) - 1 << std::endl;

  const int64_t maxReflectance = (1 << desc.bitdepth) - 1;
  const int64_t minReflectance = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t reflectance =
      attr_t(PCCClip(val, minReflectance, maxReflectance));
    pointCloud.setReflectance(packedVoxel[n].index, reflectance);
    attributes[attribCount * n] = reflectance;
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decodeOpacitiesRaht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  std::cout << "AttributeDecoder::decodeOpacitiesRaht start" << std::endl;
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());

  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }

  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);

  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }

  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;

  std::vector<int> attributes(attribCount * voxelCount);

  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }

    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());

    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getOpacity(
          packedVoxel_ref[n].index);
    }
  }

  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);

  std::cout << "desc.bitdepth = " << desc.bitdepth << std::endl;
  std::cout << "maxOpacity = " << (1 << desc.bitdepth) - 1 << std::endl;
  std::cout << "pointCloud.opacities.size() = " << pointCloud.opacities.size()
            << std::endl;

  const int64_t maxOpacity = (1 << desc.bitdepth) - 1;
  const int64_t minOpacity = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t opacity = attr_t(PCCClip(val, minOpacity, maxOpacity));
    pointCloud.setOpacity(packedVoxel[n].index, opacity);
    attributes[attribCount * n] = opacity;
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decodef_dc_0Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());

  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }

  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);

  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }

  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;

  std::vector<int> attributes(attribCount * voxelCount);

  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }

    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());

    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_dc_0(
          packedVoxel_ref[n].index);
    }
  }

  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);

  const int64_t maxf_dc_0 = (1 << desc.bitdepth) - 1;
  const int64_t minf_dc_0 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_dc_0 = attr_t(PCCClip(val, minf_dc_0, maxf_dc_0));
    pointCloud.setf_dc_0(packedVoxel[n].index, f_dc_0);
    attributes[attribCount * n] = f_dc_0;
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decodef_dc_1Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());

  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }

  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);

  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }

  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;

  std::vector<int> attributes(attribCount * voxelCount);

  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }

    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());

    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_dc_1(
          packedVoxel_ref[n].index);
    }
  }

  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);

  const int64_t maxf_dc_1 = (1 << desc.bitdepth) - 1;
  const int64_t minf_dc_1 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_dc_1 = attr_t(PCCClip(val, minf_dc_1, maxf_dc_1));
    pointCloud.setf_dc_1(packedVoxel[n].index, f_dc_1);
    attributes[attribCount * n] = f_dc_1;
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decodef_dc_2Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());

  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }

  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);

  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }

  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;

  std::vector<int> attributes(attribCount * voxelCount);

  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }

    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());

    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_dc_2(
          packedVoxel_ref[n].index);
    }
  }

  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);

  const int64_t maxf_dc_2 = (1 << desc.bitdepth) - 1;
  const int64_t minf_dc_2 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_dc_2 = attr_t(PCCClip(val, minf_dc_2, maxf_dc_2));
    pointCloud.setf_dc_2(packedVoxel[n].index, f_dc_2);
    attributes[attribCount * n] = f_dc_2;
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decodescale_0Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());

  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }

  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);

  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }

  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;

  std::vector<int> attributes(attribCount * voxelCount);

  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }

    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());

    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getscale_0(
          packedVoxel_ref[n].index);
    }
  }

  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);

  const int64_t maxscale_0 = (1 << desc.bitdepth) - 1;
  const int64_t minscale_0 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t scale_0 = attr_t(PCCClip(val, minscale_0, maxscale_0));
    pointCloud.setscale_0(packedVoxel[n].index, scale_0);
    attributes[attribCount * n] = scale_0;
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decodescale_1Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());

  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }

  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);

  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }

  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;

  std::vector<int> attributes(attribCount * voxelCount);

  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }

    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());

    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getscale_1(
          packedVoxel_ref[n].index);
    }
  }

  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);

  const int64_t maxscale_1 = (1 << desc.bitdepth) - 1;
  const int64_t minscale_1 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t scale_1 = attr_t(PCCClip(val, minscale_1, maxscale_1));
    pointCloud.setscale_1(packedVoxel[n].index, scale_1);
    attributes[attribCount * n] = scale_1;
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decodescale_2Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());

  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }

  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);

  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }

  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;

  std::vector<int> attributes(attribCount * voxelCount);

  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }

    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());

    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getscale_2(
          packedVoxel_ref[n].index);
    }
  }

  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);

  const int64_t maxscale_2 = (1 << desc.bitdepth) - 1;
  const int64_t minscale_2 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t scale_2 = attr_t(PCCClip(val, minscale_2, maxscale_2));
    pointCloud.setscale_2(packedVoxel[n].index, scale_2);
    attributes[attribCount * n] = scale_2;
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decoderot_0Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());

  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }

  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);

  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }

  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;

  std::vector<int> attributes(attribCount * voxelCount);

  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }

    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());

    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getrot_0(
          packedVoxel_ref[n].index);
    }
  }

  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);

  const int64_t maxrot_0 = (1 << desc.bitdepth) - 1;
  const int64_t minrot_0 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t rot_0 = attr_t(PCCClip(val, minrot_0, maxrot_0));
    pointCloud.setrot_0(packedVoxel[n].index, rot_0);
    attributes[attribCount * n] = rot_0;
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decoderot_1Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());

  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }

  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);

  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }

  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;

  std::vector<int> attributes(attribCount * voxelCount);

  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }

    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());

    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getrot_1(
          packedVoxel_ref[n].index);
    }
  }

  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);

  const int64_t maxrot_1 = (1 << desc.bitdepth) - 1;
  const int64_t minrot_1 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t rot_1 = attr_t(PCCClip(val, minrot_1, maxrot_1));
    pointCloud.setrot_1(packedVoxel[n].index, rot_1);
    attributes[attribCount * n] = rot_1;
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decoderot_2Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());

  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }

  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);

  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }

  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;

  std::vector<int> attributes(attribCount * voxelCount);

  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }

    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());

    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getrot_2(
          packedVoxel_ref[n].index);
    }
  }

  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);

  const int64_t maxrot_2 = (1 << desc.bitdepth) - 1;
  const int64_t minrot_2 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t rot_2 = attr_t(PCCClip(val, minrot_2, maxrot_2));
    pointCloud.setrot_2(packedVoxel[n].index, rot_2);
    attributes[attribCount * n] = rot_2;
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decoderot_3Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());

  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }

  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);

  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }

  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;

  std::vector<int> attributes(attribCount * voxelCount);

  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }

    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());

    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getrot_3(
          packedVoxel_ref[n].index);
    }
  }

  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);

  const int64_t maxrot_3 = (1 << desc.bitdepth) - 1;
  const int64_t minrot_3 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t rot_3 = attr_t(PCCClip(val, minrot_3, maxrot_3));
    pointCloud.setrot_3(packedVoxel[n].index, rot_3);
    attributes[attribCount * n] = rot_3;
  }
}

//----------------------------------------------------------------------------
void
AttributeDecoder::decodef_rest_0Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_0(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_0 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_0 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_0 = attr_t(PCCClip(val, minf_rest_0, maxf_rest_0));
    pointCloud.setf_rest_0(packedVoxel[n].index, f_rest_0);
    attributes[attribCount * n] = f_rest_0;
  }
}
void
AttributeDecoder::decodef_rest_1Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_1(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_1 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_1 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_1 = attr_t(PCCClip(val, minf_rest_1, maxf_rest_1));
    pointCloud.setf_rest_1(packedVoxel[n].index, f_rest_1);
    attributes[attribCount * n] = f_rest_1;
  }
}
void
AttributeDecoder::decodef_rest_2Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_2(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_2 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_2 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_2 = attr_t(PCCClip(val, minf_rest_2, maxf_rest_2));
    pointCloud.setf_rest_2(packedVoxel[n].index, f_rest_2);
    attributes[attribCount * n] = f_rest_2;
  }
}
void
AttributeDecoder::decodef_rest_3Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_3(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_3 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_3 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_3 = attr_t(PCCClip(val, minf_rest_3, maxf_rest_3));
    pointCloud.setf_rest_3(packedVoxel[n].index, f_rest_3);
    attributes[attribCount * n] = f_rest_3;
  }
}
void
AttributeDecoder::decodef_rest_4Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_4(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_4 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_4 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_4 = attr_t(PCCClip(val, minf_rest_4, maxf_rest_4));
    pointCloud.setf_rest_4(packedVoxel[n].index, f_rest_4);
    attributes[attribCount * n] = f_rest_4;
  }
}
void
AttributeDecoder::decodef_rest_5Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_5(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_5 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_5 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_5 = attr_t(PCCClip(val, minf_rest_5, maxf_rest_5));
    pointCloud.setf_rest_5(packedVoxel[n].index, f_rest_5);
    attributes[attribCount * n] = f_rest_5;
  }
}
void
AttributeDecoder::decodef_rest_6Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_6(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_6 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_6 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_6 = attr_t(PCCClip(val, minf_rest_6, maxf_rest_6));
    pointCloud.setf_rest_6(packedVoxel[n].index, f_rest_6);
    attributes[attribCount * n] = f_rest_6;
  }
}
void
AttributeDecoder::decodef_rest_7Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_7(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_7 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_7 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_7 = attr_t(PCCClip(val, minf_rest_7, maxf_rest_7));
    pointCloud.setf_rest_7(packedVoxel[n].index, f_rest_7);
    attributes[attribCount * n] = f_rest_7;
  }
}
void
AttributeDecoder::decodef_rest_8Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_8(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_8 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_8 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_8 = attr_t(PCCClip(val, minf_rest_8, maxf_rest_8));
    pointCloud.setf_rest_8(packedVoxel[n].index, f_rest_8);
    attributes[attribCount * n] = f_rest_8;
  }
}
void
AttributeDecoder::decodef_rest_9Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_9(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_9 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_9 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_9 = attr_t(PCCClip(val, minf_rest_9, maxf_rest_9));
    pointCloud.setf_rest_9(packedVoxel[n].index, f_rest_9);
    attributes[attribCount * n] = f_rest_9;
  }
}
void
AttributeDecoder::decodef_rest_10Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_10(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_10 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_10 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_10 = attr_t(PCCClip(val, minf_rest_10, maxf_rest_10));
    pointCloud.setf_rest_10(packedVoxel[n].index, f_rest_10);
    attributes[attribCount * n] = f_rest_10;
  }
}
void
AttributeDecoder::decodef_rest_11Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_11(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_11 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_11 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_11 = attr_t(PCCClip(val, minf_rest_11, maxf_rest_11));
    pointCloud.setf_rest_11(packedVoxel[n].index, f_rest_11);
    attributes[attribCount * n] = f_rest_11;
  }
}
void
AttributeDecoder::decodef_rest_12Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_12(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_12 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_12 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_12 = attr_t(PCCClip(val, minf_rest_12, maxf_rest_12));
    pointCloud.setf_rest_12(packedVoxel[n].index, f_rest_12);
    attributes[attribCount * n] = f_rest_12;
  }
}
void
AttributeDecoder::decodef_rest_13Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_13(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_13 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_13 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_13 = attr_t(PCCClip(val, minf_rest_13, maxf_rest_13));
    pointCloud.setf_rest_13(packedVoxel[n].index, f_rest_13);
    attributes[attribCount * n] = f_rest_13;
  }
}
void
AttributeDecoder::decodef_rest_14Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_14(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_14 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_14 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_14 = attr_t(PCCClip(val, minf_rest_14, maxf_rest_14));
    pointCloud.setf_rest_14(packedVoxel[n].index, f_rest_14);
    attributes[attribCount * n] = f_rest_14;
  }
}
void
AttributeDecoder::decodef_rest_15Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_15(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_15 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_15 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_15 = attr_t(PCCClip(val, minf_rest_15, maxf_rest_15));
    pointCloud.setf_rest_15(packedVoxel[n].index, f_rest_15);
    attributes[attribCount * n] = f_rest_15;
  }
}
void
AttributeDecoder::decodef_rest_16Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_16(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_16 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_16 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_16 = attr_t(PCCClip(val, minf_rest_16, maxf_rest_16));
    pointCloud.setf_rest_16(packedVoxel[n].index, f_rest_16);
    attributes[attribCount * n] = f_rest_16;
  }
}
void
AttributeDecoder::decodef_rest_17Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_17(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_17 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_17 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_17 = attr_t(PCCClip(val, minf_rest_17, maxf_rest_17));
    pointCloud.setf_rest_17(packedVoxel[n].index, f_rest_17);
    attributes[attribCount * n] = f_rest_17;
  }
}
void
AttributeDecoder::decodef_rest_18Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_18(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_18 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_18 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_18 = attr_t(PCCClip(val, minf_rest_18, maxf_rest_18));
    pointCloud.setf_rest_18(packedVoxel[n].index, f_rest_18);
    attributes[attribCount * n] = f_rest_18;
  }
}
void
AttributeDecoder::decodef_rest_19Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_19(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_19 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_19 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_19 = attr_t(PCCClip(val, minf_rest_19, maxf_rest_19));
    pointCloud.setf_rest_19(packedVoxel[n].index, f_rest_19);
    attributes[attribCount * n] = f_rest_19;
  }
}
void
AttributeDecoder::decodef_rest_20Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_20(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_20 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_20 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_20 = attr_t(PCCClip(val, minf_rest_20, maxf_rest_20));
    pointCloud.setf_rest_20(packedVoxel[n].index, f_rest_20);
    attributes[attribCount * n] = f_rest_20;
  }
}
void
AttributeDecoder::decodef_rest_21Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_21(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_21 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_21 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_21 = attr_t(PCCClip(val, minf_rest_21, maxf_rest_21));
    pointCloud.setf_rest_21(packedVoxel[n].index, f_rest_21);
    attributes[attribCount * n] = f_rest_21;
  }
}
void
AttributeDecoder::decodef_rest_22Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_22(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_22 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_22 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_22 = attr_t(PCCClip(val, minf_rest_22, maxf_rest_22));
    pointCloud.setf_rest_22(packedVoxel[n].index, f_rest_22);
    attributes[attribCount * n] = f_rest_22;
  }
}
void
AttributeDecoder::decodef_rest_23Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_23(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_23 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_23 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_23 = attr_t(PCCClip(val, minf_rest_23, maxf_rest_23));
    pointCloud.setf_rest_23(packedVoxel[n].index, f_rest_23);
    attributes[attribCount * n] = f_rest_23;
  }
}
void
AttributeDecoder::decodef_rest_24Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_24(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_24 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_24 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_24 = attr_t(PCCClip(val, minf_rest_24, maxf_rest_24));
    pointCloud.setf_rest_24(packedVoxel[n].index, f_rest_24);
    attributes[attribCount * n] = f_rest_24;
  }
}
void
AttributeDecoder::decodef_rest_25Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_25(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_25 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_25 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_25 = attr_t(PCCClip(val, minf_rest_25, maxf_rest_25));
    pointCloud.setf_rest_25(packedVoxel[n].index, f_rest_25);
    attributes[attribCount * n] = f_rest_25;
  }
}
void
AttributeDecoder::decodef_rest_26Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_26(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_26 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_26 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_26 = attr_t(PCCClip(val, minf_rest_26, maxf_rest_26));
    pointCloud.setf_rest_26(packedVoxel[n].index, f_rest_26);
    attributes[attribCount * n] = f_rest_26;
  }
}
void
AttributeDecoder::decodef_rest_27Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_27(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_27 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_27 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_27 = attr_t(PCCClip(val, minf_rest_27, maxf_rest_27));
    pointCloud.setf_rest_27(packedVoxel[n].index, f_rest_27);
    attributes[attribCount * n] = f_rest_27;
  }
}
void
AttributeDecoder::decodef_rest_28Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_28(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_28 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_28 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_28 = attr_t(PCCClip(val, minf_rest_28, maxf_rest_28));
    pointCloud.setf_rest_28(packedVoxel[n].index, f_rest_28);
    attributes[attribCount * n] = f_rest_28;
  }
}
void
AttributeDecoder::decodef_rest_29Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_29(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_29 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_29 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_29 = attr_t(PCCClip(val, minf_rest_29, maxf_rest_29));
    pointCloud.setf_rest_29(packedVoxel[n].index, f_rest_29);
    attributes[attribCount * n] = f_rest_29;
  }
}
void
AttributeDecoder::decodef_rest_30Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_30(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_30 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_30 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_30 = attr_t(PCCClip(val, minf_rest_30, maxf_rest_30));
    pointCloud.setf_rest_30(packedVoxel[n].index, f_rest_30);
    attributes[attribCount * n] = f_rest_30;
  }
}
void
AttributeDecoder::decodef_rest_31Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_31(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_31 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_31 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_31 = attr_t(PCCClip(val, minf_rest_31, maxf_rest_31));
    pointCloud.setf_rest_31(packedVoxel[n].index, f_rest_31);
    attributes[attribCount * n] = f_rest_31;
  }
}
void
AttributeDecoder::decodef_rest_32Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_32(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_32 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_32 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_32 = attr_t(PCCClip(val, minf_rest_32, maxf_rest_32));
    pointCloud.setf_rest_32(packedVoxel[n].index, f_rest_32);
    attributes[attribCount * n] = f_rest_32;
  }
}
void
AttributeDecoder::decodef_rest_33Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_33(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_33 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_33 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_33 = attr_t(PCCClip(val, minf_rest_33, maxf_rest_33));
    pointCloud.setf_rest_33(packedVoxel[n].index, f_rest_33);
    attributes[attribCount * n] = f_rest_33;
  }
}
void
AttributeDecoder::decodef_rest_34Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_34(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_34 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_34 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_34 = attr_t(PCCClip(val, minf_rest_34, maxf_rest_34));
    pointCloud.setf_rest_34(packedVoxel[n].index, f_rest_34);
    attributes[attribCount * n] = f_rest_34;
  }
}
void
AttributeDecoder::decodef_rest_35Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_35(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_35 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_35 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_35 = attr_t(PCCClip(val, minf_rest_35, maxf_rest_35));
    pointCloud.setf_rest_35(packedVoxel[n].index, f_rest_35);
    attributes[attribCount * n] = f_rest_35;
  }
}
void
AttributeDecoder::decodef_rest_36Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_36(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_36 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_36 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_36 = attr_t(PCCClip(val, minf_rest_36, maxf_rest_36));
    pointCloud.setf_rest_36(packedVoxel[n].index, f_rest_36);
    attributes[attribCount * n] = f_rest_36;
  }
}
void
AttributeDecoder::decodef_rest_37Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_37(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_37 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_37 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_37 = attr_t(PCCClip(val, minf_rest_37, maxf_rest_37));
    pointCloud.setf_rest_37(packedVoxel[n].index, f_rest_37);
    attributes[attribCount * n] = f_rest_37;
  }
}
void
AttributeDecoder::decodef_rest_38Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_38(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_38 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_38 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_38 = attr_t(PCCClip(val, minf_rest_38, maxf_rest_38));
    pointCloud.setf_rest_38(packedVoxel[n].index, f_rest_38);
    attributes[attribCount * n] = f_rest_38;
  }
}
void
AttributeDecoder::decodef_rest_39Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_39(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_39 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_39 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_39 = attr_t(PCCClip(val, minf_rest_39, maxf_rest_39));
    pointCloud.setf_rest_39(packedVoxel[n].index, f_rest_39);
    attributes[attribCount * n] = f_rest_39;
  }
}
void
AttributeDecoder::decodef_rest_40Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_40(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_40 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_40 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_40 = attr_t(PCCClip(val, minf_rest_40, maxf_rest_40));
    pointCloud.setf_rest_40(packedVoxel[n].index, f_rest_40);
    attributes[attribCount * n] = f_rest_40;
  }
}
void
AttributeDecoder::decodef_rest_41Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_41(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_41 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_41 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_41 = attr_t(PCCClip(val, minf_rest_41, maxf_rest_41));
    pointCloud.setf_rest_41(packedVoxel[n].index, f_rest_41);
    attributes[attribCount * n] = f_rest_41;
  }
}
void
AttributeDecoder::decodef_rest_42Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_42(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_42 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_42 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_42 = attr_t(PCCClip(val, minf_rest_42, maxf_rest_42));
    pointCloud.setf_rest_42(packedVoxel[n].index, f_rest_42);
    attributes[attribCount * n] = f_rest_42;
  }
}
void
AttributeDecoder::decodef_rest_43Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_43(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_43 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_43 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_43 = attr_t(PCCClip(val, minf_rest_43, maxf_rest_43));
    pointCloud.setf_rest_43(packedVoxel[n].index, f_rest_43);
    attributes[attribCount * n] = f_rest_43;
  }
}
void
AttributeDecoder::decodef_rest_44Raht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());
  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }
  // Entropy decode
  const int attribCount = 1;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);
  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();
    uint32_t value = 0;
    if (!zeroRunRem)
      value = decoder.decode();
    coefficients[n] = value;
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }
  std::cout << "attribCount = " << attribCount << std::endl;
  std::cout << "voxelCount = " << voxelCount << std::endl;
  std::vector<int> attributes(attribCount * voxelCount);
  if (attrInterPredParams.enableAttrInterPred) {
    const int voxelCount_ref = int(attrInterPredParams.getPointCount());
    attrInterPredParams.paramsForInterRAHT.voxelCount = voxelCount_ref;
    std::vector<MortonCodeWithIndex> packedVoxel_ref(voxelCount_ref);
    for (int n = 0; n < voxelCount_ref; n++) {
      packedVoxel_ref[n].mortonCode =
        mortonAddr(attrInterPredParams.referencePointCloud[n]);
      packedVoxel_ref[n].index = n;
    }
    sort(packedVoxel_ref.begin(), packedVoxel_ref.end());
    attrInterPredParams.paramsForInterRAHT.mortonCode.resize(voxelCount_ref);
    attrInterPredParams.paramsForInterRAHT.attributes.resize(
      attribCount * voxelCount_ref);
    // Populate input arrays.
    for (int n = 0; n < voxelCount_ref; n++) {
      attrInterPredParams.paramsForInterRAHT.mortonCode[n] =
        packedVoxel_ref[n].mortonCode;
      attrInterPredParams.paramsForInterRAHT.attributes[n] =
        attrInterPredParams.referencePointCloud.getf_rest_44(
          packedVoxel_ref[n].index);
    }
  }
  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);
  const int64_t maxf_rest_44 = (1 << desc.bitdepth) - 1;
  const int64_t minf_rest_44 = 0;
  for (int n = 0; n < voxelCount; n++) {
    int64_t val = attributes[attribCount * n];
    const attr_t f_rest_44 = attr_t(PCCClip(val, minf_rest_44, maxf_rest_44));
    pointCloud.setf_rest_44(packedVoxel[n].index, f_rest_44);
    attributes[attribCount * n] = f_rest_44;
  }
}

void
AttributeDecoder::decodeColorsRaht(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const QpSet& qpSet,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  AttributeInterPredParams& attrInterPredParams)
{
  const int voxelCount = int(pointCloud.getPointCount());
  std::vector<MortonCodeWithIndex> packedVoxel(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    packedVoxel[n].mortonCode = mortonAddr(pointCloud[n]);
    packedVoxel[n].index = n;
  }
  sort(packedVoxel.begin(), packedVoxel.end());

  // Morton codes
  std::vector<int64_t> mortonCode(voxelCount);
  for (int n = 0; n < voxelCount; n++) {
    mortonCode[n] = packedVoxel[n].mortonCode;
  }

  // Entropy decode
  const int attribCount = 3;
  std::vector<int> coefficients(attribCount * voxelCount);
  std::vector<Qps> pointQpOffsets(voxelCount);

  int zeroRunRem = 0;
  for (int n = 0; n < voxelCount; ++n) {
    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    int32_t values[3] = {};
    if (!zeroRunRem)
      decoder.decode(values);

    for (int d = 0; d < attribCount; ++d) {
      coefficients[voxelCount * d + n] = values[d];
    }
    pointQpOffsets[n] = qpSet.regionQpOffset(pointCloud[packedVoxel[n].index]);
  }

  std::vector<int> attributes(attribCount * voxelCount);

  regionAdaptiveHierarchicalInverseTransform(
    aps.rahtPredParams, qpSet, pointQpOffsets.data(), mortonCode.data(),
    attributes.data(), attribCount, voxelCount, coefficients.data(),
    aps.raht_extension, attrInterPredParams);

  int clipMax = (1 << desc.bitdepth) - 1;
  for (int n = 0; n < voxelCount; n++) {
    const int r = attributes[attribCount * n];
    const int g = attributes[attribCount * n + 1];
    const int b = attributes[attribCount * n + 2];
    Vec3<attr_t> color;
    color[0] = attr_t(PCCClip(r, 0, clipMax));
    color[1] = attr_t(PCCClip(g, 0, clipMax));
    color[2] = attr_t(PCCClip(b, 0, clipMax));
    pointCloud.setColor(packedVoxel[n].index, color);
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decodeColorsLift(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const AttributeBrickHeader& abh,
  const QpSet& qpSet,
  int geom_num_points_minus1,
  int minGeomNodeSizeLog2,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud)
{
  const size_t pointCount = pointCloud.getPointCount();
  std::vector<uint64_t> weights;

  if (!aps.scalable_lifting_enabled_flag) {
    PCCComputeQuantizationWeights(_lods.predictors, weights);
  } else {
    computeQuantizationWeightsScalable(
      _lods.predictors, _lods.numPointsInLod, geom_num_points_minus1 + 1,
      minGeomNodeSizeLog2, weights);
  }

  const size_t lodCount = _lods.numPointsInLod.size();
  std::vector<Vec3<int64_t>> colors;
  colors.resize(pointCount);

  // decompress
  // Per level-of-detail coefficients for last component prediction
  int lod = 0;
  int8_t lastCompPredCoeff = 0;
  if (aps.last_component_prediction_enabled_flag)
    lastCompPredCoeff = abh.attrLcpCoeffs[0];

  int zeroRunRem = 0;
  int quantLayer = 0;
  for (size_t predictorIndex = 0; predictorIndex < pointCount;
       ++predictorIndex) {
    if (predictorIndex == _lods.numPointsInLod[quantLayer]) {
      quantLayer = std::min(int(qpSet.layers.size()) - 1, quantLayer + 1);
    }

    if (predictorIndex == _lods.numPointsInLod[lod]) {
      lod++;
      if (aps.last_component_prediction_enabled_flag)
        lastCompPredCoeff = abh.attrLcpCoeffs[lod];
    }

    const uint32_t pointIndex = _lods.indexes[predictorIndex];
    auto quant = qpSet.quantizers(pointCloud[pointIndex], quantLayer);

    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    int32_t values[3] = {};
    if (!zeroRunRem)
      decoder.decode(values);

    const int64_t iQuantWeight = irsqrt(weights[predictorIndex]);
    auto& color = colors[predictorIndex];

    int64_t scaled = quant[0].scale(values[0]);
    color[0] = divExp2RoundHalfInf(scaled * iQuantWeight, 40);

    scaled = quant[1].scale(values[1]);
    color[1] = divExp2RoundHalfInf(scaled * iQuantWeight, 40);

    scaled *= lastCompPredCoeff;
    scaled >>= 2;

    scaled += quant[1].scale(values[2]);
    color[2] = divExp2RoundHalfInf(scaled * iQuantWeight, 40);
  }

  // reconstruct
  for (size_t lodIndex = 1; lodIndex < lodCount; ++lodIndex) {
    const size_t startIndex = _lods.numPointsInLod[lodIndex - 1];
    const size_t endIndex = _lods.numPointsInLod[lodIndex];
    PCCLiftUpdate(
      _lods.predictors, weights, startIndex, endIndex, false, colors);
    PCCLiftPredict(_lods.predictors, startIndex, endIndex, false, colors);
  }

  int64_t clipMax = (1 << desc.bitdepth) - 1;
  for (size_t f = 0; f < pointCount; ++f) {
    const auto color0 =
      divExp2RoundHalfInf(colors[f], kFixedPointAttributeShift);
    Vec3<attr_t> color;
    for (size_t d = 0; d < 3; ++d) {
      color[d] = attr_t(PCCClip(color0[d], int64_t(0), clipMax));
    }
    pointCloud.setColor(_lods.indexes[f], color);
  }
}

//----------------------------------------------------------------------------

void
AttributeDecoder::decodeReflectancesLift(
  const AttributeDescription& desc,
  const AttributeParameterSet& aps,
  const AttributeBrickHeader& abh,
  const QpSet& qpSet,
  int geom_num_points_minus1,
  int minGeomNodeSizeLog2,
  PCCResidualsDecoder& decoder,
  PCCPointSet3& pointCloud,
  const AttributeInterPredParams& attrInterPredParams)
{
  const size_t pointCount = pointCloud.getPointCount();
  std::vector<uint64_t> weights;

  if (!aps.scalable_lifting_enabled_flag) {
    PCCComputeQuantizationWeights(
      _lods.predictors, weights, attrInterPredParams.enableAttrInterPred);
  } else {
    computeQuantizationWeightsScalable(
      _lods.predictors, _lods.numPointsInLod, geom_num_points_minus1 + 1,
      minGeomNodeSizeLog2, weights);
  }

  const size_t lodCount = _lods.numPointsInLod.size();
  std::vector<int64_t> reflectances;
  reflectances.resize(pointCount);

  std::vector<int64_t> reflectancesRef;
  const auto& referencePointCloud = attrInterPredParams.referencePointCloud;
  reflectancesRef.resize(referencePointCloud.getPointCount());

  for (size_t index = 0; index < referencePointCloud.getPointCount();
       ++index) {
    reflectancesRef[index] = int32_t(referencePointCloud.getReflectance(index))
      << kFixedPointAttributeShift;
  }

  // decompress
  int zeroRunRem = 0;
  int quantLayer = 0;
  for (size_t predictorIndex = 0; predictorIndex < pointCount;
       ++predictorIndex) {
    if (predictorIndex == _lods.numPointsInLod[quantLayer]) {
      quantLayer = std::min(int(qpSet.layers.size()) - 1, quantLayer + 1);
    }
    const uint32_t pointIndex = _lods.indexes[predictorIndex];
    auto quant = qpSet.quantizers(pointCloud[pointIndex], quantLayer);

    if (--zeroRunRem < 0)
      zeroRunRem = decoder.decodeRunLength();

    int64_t detail = 0;
    if (!zeroRunRem)
      detail = decoder.decode();

    const int64_t iQuantWeight = irsqrt(weights[predictorIndex]);
    auto& reflectance = reflectances[predictorIndex];
    const int64_t delta = detail;
    const int64_t reconstructedDelta = quant[0].scale(delta);
    reflectance = divExp2RoundHalfInf(reconstructedDelta * iQuantWeight, 40);
  }

  // reconstruct
  for (size_t lodIndex = 1; lodIndex < lodCount; ++lodIndex) {
    const size_t startIndex = _lods.numPointsInLod[lodIndex - 1];
    const size_t endIndex = _lods.numPointsInLod[lodIndex];
    PCCLiftUpdate(
      _lods.predictors, weights, startIndex, endIndex, false, reflectances,
      attrInterPredParams.enableAttrInterPred);
    PCCLiftPredict(
      _lods.predictors, startIndex, endIndex, false, reflectances,
      attrInterPredParams.enableAttrInterPred, reflectancesRef);
  }
  const int64_t maxReflectance = (1 << desc.bitdepth) - 1;
  for (size_t f = 0; f < pointCount; ++f) {
    const auto refl =
      divExp2RoundHalfInf(reflectances[f], kFixedPointAttributeShift);
    pointCloud.setReflectance(
      _lods.indexes[f], attr_t(PCCClip(refl, int64_t(0), maxReflectance)));
  }
}

//============================================================================

} /* namespace pcc */
