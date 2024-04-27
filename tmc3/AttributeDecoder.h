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

#pragma once

#include <stdint.h>

#include "Attribute.h"
#include "AttributeCommon.h"
#include "PayloadBuffer.h"
#include "PCCTMC3Common.h"
#include "quantization.h"

namespace pcc {

//============================================================================
// Opaque definitions (Internal detail)

class PCCResidualsDecoder;

//============================================================================

class AttributeDecoder : public AttributeDecoderIntf {
public:
  void decode(
    const SequenceParameterSet& sps,
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const AttributeBrickHeader& abh,
    int geom_num_points_minus1,
    int minGeomNodeSizeLog2,
    const char* payload,
    size_t payloadLen,
    AttributeContexts& ctxtMem,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams) override;

  bool isReusable(
    const AttributeParameterSet& aps,
    const AttributeBrickHeader& abh) const override;

protected:
  // todo(df): consider alternative encapsulation

  void decodeReflectancesLift(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const AttributeBrickHeader& abh,
    const QpSet& qpSet,
    int geom_num_points_minus1,
    int minGeomNodeSizeLog2,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    const AttributeInterPredParams& attrInterPredParams);

  void decodeColorsLift(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const AttributeBrickHeader& abh,
    const QpSet& qpSet,
    int geom_num_points_minus1,
    int minGeomNodeSizeLog2,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud);

  void decodeReflectancesPred(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const AttributeBrickHeader& abh,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    const AttributeInterPredParams& attrInterPredParams);

  void decodeColorsPred(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const AttributeBrickHeader& abh,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud);

  void decodeReflectancesRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);

  void decodeOpacitiesRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);

  void decodef_dc_0Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);

  void decodef_dc_1Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);

  void decodef_dc_2Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);

  void decodescale_0Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);

  void decodescale_1Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);

  void decodescale_2Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);

  void decoderot_0Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);

  void decoderot_1Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);

  void decoderot_2Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);

  void decoderot_3Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);

  void decodef_rest_0Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_1Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_2Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_3Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_4Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_5Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_6Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_7Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_8Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_9Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_10Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_11Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_12Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_13Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_14Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_15Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_16Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_17Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_18Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_19Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_20Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_21Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_22Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_23Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_24Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_25Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_26Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_27Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_28Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_29Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_30Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_31Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_32Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_33Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_34Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_35Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_36Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_37Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_38Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_39Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_40Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_41Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_42Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_43Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);
  void decodef_rest_44Raht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);

  void decodeColorsRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCResidualsDecoder& decoder,
    PCCPointSet3& pointCloud,
    AttributeInterPredParams& attrInterPredParams);

  static void decodePredModeColor(
    const AttributeParameterSet& aps,
    Vec3<int32_t>& coeff,
    PCCPredictor& predictor);

  static void decodePredModeRefl(
    const AttributeParameterSet& aps, int32_t& coeff, PCCPredictor& predictor);

private:
  AttributeLods _lods;
};

//============================================================================

} /* namespace pcc */
