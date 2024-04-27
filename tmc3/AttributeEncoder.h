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
#include <vector>

#include "Attribute.h"
#include "AttributeCommon.h"
#include "PayloadBuffer.h"
#include "PCCTMC3Common.h"
#include "hls.h"
#include "quantization.h"

namespace pcc {

//============================================================================
// Opaque definitions (Internal detail)

class PCCResidualsEncoder;
struct PCCResidualsEntropyEstimator;

//============================================================================

class AttributeEncoder : public AttributeEncoderIntf {
public:
  void encode(
    const SequenceParameterSet& sps,
    const AttributeDescription& desc,
    const AttributeParameterSet& attr_aps,
    AttributeBrickHeader& abh,
    AttributeContexts& ctxtMem,
    PCCPointSet3& pointCloud,
    PayloadBuffer* payload,
    AttributeInterPredParams& attrInterPredParams) override;

  bool isReusable(
    const AttributeParameterSet& aps,
    const AttributeBrickHeader& abh) const override;

protected:
  // todo(df): consider alternative encapsulation

  void encodeReflectancesLift(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encodeColorsLift(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder);

  void encodeReflectancesPred(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encodeColorsPred(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder);

  void encodeReflectancesTransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encodeOpacitiesTransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encodef_dc_0TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encodef_dc_1TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encodef_dc_2TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encodescale_0TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encodescale_1TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encodescale_2TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encoderot_0TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encoderot_1TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encoderot_2TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encoderot_3TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encodef_rest_0TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_1TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_2TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_3TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_4TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_5TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_6TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_7TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_8TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_9TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_10TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_11TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_12TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_13TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_14TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_15TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_16TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_17TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_18TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_19TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_20TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_21TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_22TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_23TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_24TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_25TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_26TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_27TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_28TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_29TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_30TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_31TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_32TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_33TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_34TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_35TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_36TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_37TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_38TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_39TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_40TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_41TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_42TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_43TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);
  void encodef_rest_44TransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  void encodeColorsTransformRaht(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const QpSet& qpSet,
    PCCPointSet3& pointCloud,
    PCCResidualsEncoder& encoder,
    AttributeInterPredParams& attrInterPredParams);

  static Vec3<int64_t> computeColorResiduals(
    const AttributeParameterSet& aps,
    const Vec3<attr_t> color,
    const Vec3<attr_t> predictedColor,
    const Vec3<int8_t> icpCoeff,
    const Quantizers& quant);

  static int computeColorDistortions(
    const AttributeDescription& desc,
    const Vec3<attr_t> color,
    const Vec3<attr_t> predictedColor,
    const Quantizers& quant);

  static void decidePredModeColor(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const PCCPointSet3& pointCloud,
    const std::vector<uint32_t>& indexesLOD,
    const uint32_t predictorIndex,
    PCCPredictor& predictor,
    PCCResidualsEncoder& encoder,
    PCCResidualsEntropyEstimator& context,
    const Vec3<int8_t>& icpCoeff,
    const Quantizers& quant);

  static void encodePredModeColor(
    const AttributeParameterSet& aps, int predMode, Vec3<int32_t>& coeff);

  static int64_t computeReflectanceResidual(
    const uint64_t reflectance,
    const uint64_t predictedReflectance,
    const Quantizer& quant);

  static void decidePredModeRefl(
    const AttributeDescription& desc,
    const AttributeParameterSet& aps,
    const PCCPointSet3& pointCloud,
    const std::vector<uint32_t>& indexesLOD,
    const uint32_t predictorIndex,
    PCCPredictor& predictor,
    PCCResidualsEncoder& encoder,
    PCCResidualsEntropyEstimator& context,
    const Quantizer& quant,
    const AttributeInterPredParams& attrInterPredParams);

  static void encodePredModeRefl(
    const AttributeParameterSet& aps, int predMode, int32_t& coeff);

private:
  std::vector<int8_t> computeLastComponentPredictionCoeff(
    const AttributeParameterSet& aps,
    const std::vector<Vec3<int64_t>>& coeffs);

  std::vector<Vec3<int8_t>> computeInterComponentPredictionCoeffs(
    const AttributeParameterSet& aps, const PCCPointSet3& pointCloud);

private:
  // The current attribute slice header
  AttributeBrickHeader* _abh;

  AttributeLods _lods;
};

//============================================================================

} /* namespace pcc */
