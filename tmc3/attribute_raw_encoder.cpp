/* The copyright in this software is being made available under the BSD
 * Licence, included below.  This software may be subject to other third
 * party and contributor rights, including patent rights, and no such
 * rights are granted under this licence.
 *
 * Copyright (c) 2021, ISO/IEC
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

#include "attribute_raw.h"

#include "BitWriter.h"
#include "io_hls.h"

namespace pcc {

//============================================================================

void
AttrRawEncoder::encode(
  const SequenceParameterSet& sps,
  const AttributeDescription& desc,
  const AttributeParameterSet& attr_aps,
  AttributeBrickHeader& abh,
  PCCPointSet3& cloud,
  PayloadBuffer* payload)
{
  write(sps, attr_aps, abh, payload);
  auto bs = makeBitWriter(std::back_inserter(*payload));

  // todo(df): add support for variable length coding
  assert(!attr_aps.raw_attr_variable_len_flag);
  int valueBits = desc.bitdepth;

  // todo(df): update to correctly map attribute types
  if (desc.attr_num_dimensions_minus1 == 0) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getReflectance(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 2) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      auto& value = cloud.getColor(i);
      for (int c = 0; c < 3; c++)
        bs.writeUn(valueBits, value[c]);
    }
  } else if (desc.attr_num_dimensions_minus1 == 3) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getOpacity(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 4) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_dc_0(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 5) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_dc_1(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 6) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_dc_2(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 7) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getscale_0(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 8) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getscale_1(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 9) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getscale_2(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 10) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getrot_0(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 11) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getrot_1(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 12) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getrot_2(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 13) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getrot_3(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 14) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_0(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 15) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_1(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 16) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_2(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 17) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_3(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 18) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_4(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 19) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_5(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 20) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_6(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 21) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_7(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 22) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_8(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 23) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_9(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 24) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_10(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 25) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_11(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 26) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_12(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 27) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_13(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 28) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_14(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 29) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_15(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 30) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_16(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 31) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_17(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 32) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_18(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 33) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_19(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 34) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_20(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 35) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_21(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 36) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_22(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 37) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_23(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 38) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_24(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 39) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_25(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 40) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_26(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 41) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_27(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 42) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_28(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 43) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_29(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 44) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_30(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 45) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_31(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 46) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_32(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 47) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_33(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 48) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_34(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 49) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_35(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 50) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_36(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 51) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_37(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 52) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_38(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 53) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_39(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 54) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_40(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 55) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_41(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 56) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_42(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 57) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_43(i));
    }
  } else if (desc.attr_num_dimensions_minus1 == 58) {
    for (size_t i = 0; i < cloud.getPointCount(); i++) {
      bs.writeUn(valueBits, cloud.getf_rest_44(i));
    }
  } else {
    assert(
      desc.attr_num_dimensions_minus1 == 0
      || desc.attr_num_dimensions_minus1 == 2
      || desc.attr_num_dimensions_minus1 == 3);
  }

  bs.byteAlign();
}

//============================================================================

} /* namespace pcc */
