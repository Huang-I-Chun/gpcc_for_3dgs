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

#include "quantization.h"

#include "constants.h"
#include "hls.h"
#include "tables.h"

namespace pcc {

//============================================================================

Quantizer::Quantizer(int qp)
{
  qp = std::max(qp, 4);
  int qpShift = qp / 6;
  _stepSize = kQpStep[qp % 6] << qpShift;
  _stepSizeRecip = kQpStepRecip[qp % 6] >> qpShift;
}

//============================================================================

Qps
deriveQps(
  const AttributeParameterSet& attr_aps,
  const AttributeBrickHeader& abh,
  int qpLayer,
  int attr_num_dimensions_minus1)
{
  // int sliceQpLuma = attr_aps.init_qp_minus4 + 4;
  int sliceQpLuma;

  if (attr_num_dimensions_minus1 == -1 || attr_num_dimensions_minus1 == 0) {
    sliceQpLuma = attr_aps.init_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 3) {
    sliceQpLuma = attr_aps.opacity_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 4) {
    sliceQpLuma = attr_aps.f_dc_0_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 5) {
    sliceQpLuma = attr_aps.f_dc_1_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 6) {
    sliceQpLuma = attr_aps.f_dc_2_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 7) {
    sliceQpLuma = attr_aps.f_rest_0_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 8) {
    sliceQpLuma = attr_aps.scale_0_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 9) {
    sliceQpLuma = attr_aps.scale_1_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 10) {
    sliceQpLuma = attr_aps.scale_2_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 11) {
    sliceQpLuma = attr_aps.rot_0_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 12) {
    sliceQpLuma = attr_aps.rot_1_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 13) {
    sliceQpLuma = attr_aps.rot_2_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 14) {
    sliceQpLuma = attr_aps.rot_3_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 15) {
    sliceQpLuma = attr_aps.f_rest_1_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 16) {
    sliceQpLuma = attr_aps.f_rest_2_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 17) {
    sliceQpLuma = attr_aps.f_rest_3_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 18) {
    sliceQpLuma = attr_aps.f_rest_4_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 19) {
    sliceQpLuma = attr_aps.f_rest_5_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 20) {
    sliceQpLuma = attr_aps.f_rest_6_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 21) {
    sliceQpLuma = attr_aps.f_rest_7_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 22) {
    sliceQpLuma = attr_aps.f_rest_8_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 23) {
    sliceQpLuma = attr_aps.f_rest_9_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 24) {
    sliceQpLuma = attr_aps.f_rest_10_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 25) {
    sliceQpLuma = attr_aps.f_rest_11_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 26) {
    sliceQpLuma = attr_aps.f_rest_12_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 27) {
    sliceQpLuma = attr_aps.f_rest_13_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 28) {
    sliceQpLuma = attr_aps.f_rest_14_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 29) {
    sliceQpLuma = attr_aps.f_rest_15_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 30) {
    sliceQpLuma = attr_aps.f_rest_16_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 31) {
    sliceQpLuma = attr_aps.f_rest_17_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 32) {
    sliceQpLuma = attr_aps.f_rest_18_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 33) {
    sliceQpLuma = attr_aps.f_rest_19_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 34) {
    sliceQpLuma = attr_aps.f_rest_20_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 35) {
    sliceQpLuma = attr_aps.f_rest_21_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 36) {
    sliceQpLuma = attr_aps.f_rest_22_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 37) {
    sliceQpLuma = attr_aps.f_rest_23_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 38) {
    sliceQpLuma = attr_aps.f_rest_24_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 39) {
    sliceQpLuma = attr_aps.f_rest_25_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 40) {
    sliceQpLuma = attr_aps.f_rest_26_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 41) {
    sliceQpLuma = attr_aps.f_rest_27_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 42) {
    sliceQpLuma = attr_aps.f_rest_28_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 43) {
    sliceQpLuma = attr_aps.f_rest_29_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 44) {
    sliceQpLuma = attr_aps.f_rest_30_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 45) {
    sliceQpLuma = attr_aps.f_rest_31_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 46) {
    sliceQpLuma = attr_aps.f_rest_32_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 47) {
    sliceQpLuma = attr_aps.f_rest_33_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 48) {
    sliceQpLuma = attr_aps.f_rest_34_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 49) {
    sliceQpLuma = attr_aps.f_rest_35_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 50) {
    sliceQpLuma = attr_aps.f_rest_36_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 51) {
    sliceQpLuma = attr_aps.f_rest_37_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 52) {
    sliceQpLuma = attr_aps.f_rest_38_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 53) {
    sliceQpLuma = attr_aps.f_rest_39_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 54) {
    sliceQpLuma = attr_aps.f_rest_40_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 55) {
    sliceQpLuma = attr_aps.f_rest_41_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 56) {
    sliceQpLuma = attr_aps.f_rest_42_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 57) {
    sliceQpLuma = attr_aps.f_rest_43_qp_minus4 + 4;
  } else if (attr_num_dimensions_minus1 == 58) {
    sliceQpLuma = attr_aps.f_rest_44_qp_minus4 + 4;
  } else {
    std::cout << "error happen while setting qp" << std::endl;
  }

  // sliceQpLuma = attr_aps.init_qp_minus4 + 4;

  // if (attr_num_dimensions_minus1 >= 15) {
  //   sliceQpLuma = 35;
  // }
  // std::cout << "attr_aps.f_rest_44_qp_minus4: " << attr_aps.f_rest_44_qp_minus4
  //           << std::endl;
  std::cout << "sliceQpLuma: " << sliceQpLuma << std::endl;

  int sliceQpChroma = attr_aps.aps_chroma_qp_offset;

  if (attr_aps.aps_slice_qp_deltas_present_flag) {
    sliceQpLuma += abh.attr_qp_delta_luma;
    sliceQpChroma += abh.attr_qp_delta_chroma;
  }

  if (abh.attr_layer_qp_present_flag()) {
    sliceQpLuma += abh.attr_layer_qp_delta_luma[qpLayer];
    sliceQpChroma += abh.attr_layer_qp_delta_chroma[qpLayer];
  }

  return {sliceQpLuma, sliceQpChroma};
}

//============================================================================

QpLayers
deriveLayerQps(
  const AttributeParameterSet& attr_aps,
  const AttributeBrickHeader& abh,
  int attr_num_dimensions_minus1)
{
  QpLayers layers;

  layers.push_back(deriveQps(attr_aps, abh, 0, attr_num_dimensions_minus1));

  if (abh.attr_layer_qp_present_flag()) {
    int numLayers = abh.attr_num_qp_layers_minus1() + 1;
    for (int layer = 1; layer < numLayers; layer++) {
      layers.push_back(
        deriveQps(attr_aps, abh, layer, attr_num_dimensions_minus1));
    }
  }

  return layers;
}

//============================================================================

QpRegionList
deriveQpRegions(
  const AttributeParameterSet& attr_aps, const AttributeBrickHeader& abh)
{
  QpRegionList regions;
  regions.reserve(abh.qpRegions.size());

  for (int i = 0; i < abh.qpRegions.size(); i++) {
    regions.emplace_back();
    auto& region = regions.back();
    const auto& src = abh.qpRegions[i];

    region.qpOffset = src.attr_region_qp_offset;
    region.region.min = src.regionOrigin;
    region.region.max = src.regionOrigin + src.regionSize;
  }

  return regions;
}

//============================================================================

RahtAcCoeffQpOffset
deriveRahtAcCoeffQpOffsets(
  const AttributeParameterSet& attr_aps, const AttributeBrickHeader& abh)
{
  RahtAcCoeffQpOffset rahtAcCoeffQpOffsets;
  if (abh.attr_raht_ac_coeff_qp_offset_preset()) {
    int numLayers = abh.attr_num_raht_ac_coeff_qp_layers_minus1() + 1;
    rahtAcCoeffQpOffsets.resize(numLayers, std::vector<Qps>(7, {0, 0}));
    for (auto i = 0; i < numLayers; i++) {
      for (auto coeffIdx = 0; coeffIdx < 7; coeffIdx++) {
        rahtAcCoeffQpOffsets[i][coeffIdx] = {
          abh.attr_raht_ac_coeff_qp_delta_luma[i][coeffIdx],
          abh.attr_raht_ac_coeff_qp_delta_chroma[i][coeffIdx]};
      }
    }
  }
  return rahtAcCoeffQpOffsets;
}

//============================================================================

QpSet
deriveQpSet(
  const AttributeDescription& attrDesc,
  const AttributeParameterSet& attr_aps,
  const AttributeBrickHeader& abh,
  int attr_num_dimensions_minus1)
{
  QpSet qpset;
  qpset.layers = deriveLayerQps(attr_aps, abh, attr_num_dimensions_minus1);
  qpset.regions = deriveQpRegions(attr_aps, abh);
  qpset.rahtAcCoeffQps = deriveRahtAcCoeffQpOffsets(attr_aps, abh);

  // The mimimum Qp = 4 is always lossless; the maximum varies according to
  // bitdepth.
  qpset.maxQp = 51 + 6 * (attrDesc.bitdepth - 8);

  // the lifting transform has extra fractional bits that equate to
  // increasing the QP.
  qpset.fixedPointQpOffset = 0;
  if (attr_aps.attr_encoding == AttributeEncoding::kLiftingTransform)
    qpset.fixedPointQpOffset = (kFixedPointWeightShift / 2) * 6;

  return qpset;
}

//============================================================================
// Determines the quantizers at a given layer
Quantizers
QpSet::quantizers(int qpLayer, Qps qpOffset) const
{
  int qp0 = PCCClip(layers[qpLayer][0] + qpOffset[0], 4, maxQp);
  int qp1 = PCCClip(layers[qpLayer][1] + qpOffset[1] + qp0, 4, maxQp);
  qp0 = qp0 + fixedPointQpOffset;
  qp1 = qp1 + fixedPointQpOffset;

  return {Quantizer(qp0), Quantizer(qp1)};
}

//============================================================================
// Determines the quantizers for a point at a given layer
Quantizers
QpSet::quantizers(const Vec3<int32_t>& point, int qpLayer) const
{
  for (const auto& region : regions) {
    if (region.region.contains(point))
      return quantizers(qpLayer, region.qpOffset);
  }

  return quantizers(qpLayer, {0, 0});
}

//============================================================================
//for RAHT region QP Offset
Qps
QpSet::regionQpOffset(const Vec3<int32_t>& point) const
{
  for (const auto& region : regions) {
    if (region.region.contains(point))
      return region.qpOffset;
  }

  return {0, 0};
}

//============================================================================

const int32_t QuantizerGeom::kQpStep[8] = {8, 9, 10, 11, 12, 13, 14, 15};

const int32_t QuantizerGeom::kQpStepRecip[8] = {
  1 << 20, 932068, 838861, 762601, 699051, 645278, 599186, 559241};

//============================================================================

}  // namespace pcc
