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

#include "ply.h"

#include "PCCMisc.h"
#include "PCCPointSet.h"
#include "attrbute_para.h"

#include <fstream>
#include <string>
#include <vector>

namespace pcc {

//============================================================================

static bool
compareSeparators(char aChar, const char* const sep)
{
  int i = 0;
  while (sep[i] != '\0') {
    if (aChar == sep[i])
      return false;
    i++;
  }
  return true;
}

float
reverse_value(float value, float min_value, float max_value, float scale_by)
{
  // std::cout << float(value) << " " << scale_by << " " << max_value << " "
  //           << min_value << " "
  //           << float(value) / scale_by * max_value + min_value << std::endl;
  return float(value) / scale_by * max_value + min_value;
}

std::vector<float>
processVector(std::vector<float> vec, std::string attr_name, float scale_by)
{
  float min_value = *std::min_element(vec.begin(), vec.end());

  // Subtract the minimum value from all elements
  for (float& value : vec) {
    value -= min_value;
  }

  // Normalize the values to range 0-1
  float max_value = *std::max_element(vec.begin(), vec.end());
  if (max_value != 0) {  // Prevent division by zero
    for (float& value : vec) {
      value /= max_value;
    }
  }

  for (float& value : vec) {
    value *= scale_by;
  }

  if (attr_name == "opacity") {
    opacity_min = min_value;
    opacity_max = max_value;
  } else if (attr_name == "f_dc_0") {
    f_dc_0_min = min_value;
    f_dc_0_max = max_value;
  } else if (attr_name == "f_dc_1") {
    f_dc_1_min = min_value;
    f_dc_1_max = max_value;
  } else if (attr_name == "f_dc_2") {
    f_dc_2_min = min_value;
    f_dc_2_max = max_value;
  } else if (attr_name == "f_rest_0") {
    f_rest_0_min = min_value;
    f_rest_0_max = max_value;
  } else if (attr_name == "scale_0") {
    scale_0_min = min_value;
    scale_0_max = max_value;
  } else if (attr_name == "scale_1") {
    scale_1_min = min_value;
    scale_1_max = max_value;
  } else if (attr_name == "scale_2") {
    scale_2_min = min_value;
    scale_2_max = max_value;
  } else if (attr_name == "rot_0") {
    rot_0_min = min_value;
    rot_0_max = max_value;
  } else if (attr_name == "rot_1") {
    rot_1_min = min_value;
    rot_1_max = max_value;
  } else if (attr_name == "rot_2") {
    rot_2_min = min_value;
    rot_2_max = max_value;
  } else if (attr_name == "rot_3") {
    rot_3_min = min_value;
    rot_3_max = max_value;
  } else if (attr_name == "f_rest_1") {
    f_rest_1_min = min_value;
    f_rest_1_max = max_value;
  } else if (attr_name == "f_rest_2") {
    f_rest_2_min = min_value;
    f_rest_2_max = max_value;
  } else if (attr_name == "f_rest_3") {
    f_rest_3_min = min_value;
    f_rest_3_max = max_value;
  } else if (attr_name == "f_rest_4") {
    f_rest_4_min = min_value;
    f_rest_4_max = max_value;
  } else if (attr_name == "f_rest_5") {
    f_rest_5_min = min_value;
    f_rest_5_max = max_value;
  } else if (attr_name == "f_rest_6") {
    f_rest_6_min = min_value;
    f_rest_6_max = max_value;
  } else if (attr_name == "f_rest_7") {
    f_rest_7_min = min_value;
    f_rest_7_max = max_value;
  } else if (attr_name == "f_rest_8") {
    f_rest_8_min = min_value;
    f_rest_8_max = max_value;
  } else if (attr_name == "f_rest_9") {
    f_rest_9_min = min_value;
    f_rest_9_max = max_value;
  } else if (attr_name == "f_rest_10") {
    f_rest_10_min = min_value;
    f_rest_10_max = max_value;
  } else if (attr_name == "f_rest_11") {
    f_rest_11_min = min_value;
    f_rest_11_max = max_value;
  } else if (attr_name == "f_rest_12") {
    f_rest_12_min = min_value;
    f_rest_12_max = max_value;
  } else if (attr_name == "f_rest_13") {
    f_rest_13_min = min_value;
    f_rest_13_max = max_value;
  } else if (attr_name == "f_rest_14") {
    f_rest_14_min = min_value;
    f_rest_14_max = max_value;
  } else if (attr_name == "f_rest_15") {
    f_rest_15_min = min_value;
    f_rest_15_max = max_value;
  } else if (attr_name == "f_rest_16") {
    f_rest_16_min = min_value;
    f_rest_16_max = max_value;
  } else if (attr_name == "f_rest_17") {
    f_rest_17_min = min_value;
    f_rest_17_max = max_value;
  } else if (attr_name == "f_rest_18") {
    f_rest_18_min = min_value;
    f_rest_18_max = max_value;
  } else if (attr_name == "f_rest_19") {
    f_rest_19_min = min_value;
    f_rest_19_max = max_value;
  } else if (attr_name == "f_rest_20") {
    f_rest_20_min = min_value;
    f_rest_20_max = max_value;
  } else if (attr_name == "f_rest_21") {
    f_rest_21_min = min_value;
    f_rest_21_max = max_value;
  } else if (attr_name == "f_rest_22") {
    f_rest_22_min = min_value;
    f_rest_22_max = max_value;
  } else if (attr_name == "f_rest_23") {
    f_rest_23_min = min_value;
    f_rest_23_max = max_value;
  } else if (attr_name == "f_rest_24") {
    f_rest_24_min = min_value;
    f_rest_24_max = max_value;
  } else if (attr_name == "f_rest_25") {
    f_rest_25_min = min_value;
    f_rest_25_max = max_value;
  } else if (attr_name == "f_rest_26") {
    f_rest_26_min = min_value;
    f_rest_26_max = max_value;
  } else if (attr_name == "f_rest_27") {
    f_rest_27_min = min_value;
    f_rest_27_max = max_value;
  } else if (attr_name == "f_rest_28") {
    f_rest_28_min = min_value;
    f_rest_28_max = max_value;
  } else if (attr_name == "f_rest_29") {
    f_rest_29_min = min_value;
    f_rest_29_max = max_value;
  } else if (attr_name == "f_rest_30") {
    f_rest_30_min = min_value;
    f_rest_30_max = max_value;
  } else if (attr_name == "f_rest_31") {
    f_rest_31_min = min_value;
    f_rest_31_max = max_value;
  } else if (attr_name == "f_rest_32") {
    f_rest_32_min = min_value;
    f_rest_32_max = max_value;
  } else if (attr_name == "f_rest_33") {
    f_rest_33_min = min_value;
    f_rest_33_max = max_value;
  } else if (attr_name == "f_rest_34") {
    f_rest_34_min = min_value;
    f_rest_34_max = max_value;
  } else if (attr_name == "f_rest_35") {
    f_rest_35_min = min_value;
    f_rest_35_max = max_value;
  } else if (attr_name == "f_rest_36") {
    f_rest_36_min = min_value;
    f_rest_36_max = max_value;
  } else if (attr_name == "f_rest_37") {
    f_rest_37_min = min_value;
    f_rest_37_max = max_value;
  } else if (attr_name == "f_rest_38") {
    f_rest_38_min = min_value;
    f_rest_38_max = max_value;
  } else if (attr_name == "f_rest_39") {
    f_rest_39_min = min_value;
    f_rest_39_max = max_value;
  } else if (attr_name == "f_rest_40") {
    f_rest_40_min = min_value;
    f_rest_40_max = max_value;
  } else if (attr_name == "f_rest_41") {
    f_rest_41_min = min_value;
    f_rest_41_max = max_value;
  } else if (attr_name == "f_rest_42") {
    f_rest_42_min = min_value;
    f_rest_42_max = max_value;
  } else if (attr_name == "f_rest_43") {
    f_rest_43_min = min_value;
    f_rest_43_max = max_value;
  } else if (attr_name == "f_rest_44") {
    f_rest_44_min = min_value;
    f_rest_44_max = max_value;
  }

  return vec;
}

//============================================================================

static bool
getTokens(
  const char* str, const char* const sep, std::vector<std::string>& tokens)
{
  if (!tokens.empty())
    tokens.clear();
  std::string buf = "";
  size_t i = 0;
  size_t length = ::strlen(str);
  while (i < length) {
    if (compareSeparators(str[i], sep)) {
      buf += str[i];
    } else if (buf.length() > 0) {
      tokens.push_back(buf);
      buf = "";
    }
    i++;
  }
  if (!buf.empty())
    tokens.push_back(buf);
  return !tokens.empty();
}

//============================================================================

bool
ply::write(
  const PCCPointSet3& cloud,
  const PropertyNameMap& attributeNames,
  double positionScale,
  Vec3<double> positionOffset,
  const std::string& fileName,
  bool asAscii)
{
  std::ofstream fout(fileName, std::ofstream::out);
  if (!fout.is_open()) {
    return false;
  }

  const size_t pointCount = cloud.getPointCount();
  fout << "ply" << std::endl;

  if (asAscii) {
    fout << "format ascii 1.0" << std::endl;
  } else {
    PCCEndianness endianess = PCCSystemEndianness();
    if (endianess == PCC_BIG_ENDIAN) {
      fout << "format binary_big_endian 1.0" << std::endl;
    } else {
      fout << "format binary_little_endian 1.0" << std::endl;
    }
  }
  fout << "element vertex " << pointCount << std::endl;
  if (asAscii) {
    fout << "property float " << attributeNames.position[0] << std::endl;
    fout << "property float " << attributeNames.position[1] << std::endl;
    fout << "property float " << attributeNames.position[2] << std::endl;
  } else {
    // fout << "property float64 " << attributeNames.position[0] << std::endl;
    // fout << "property float64 " << attributeNames.position[1] << std::endl;
    // fout << "property float64 " << attributeNames.position[2] << std::endl;
    fout << "property float " << attributeNames.position[0] << std::endl;
    fout << "property float " << attributeNames.position[1] << std::endl;
    fout << "property float " << attributeNames.position[2] << std::endl;
  }

  if (cloud.hasColors()) {
    fout << "property uchar green" << std::endl;
    fout << "property uchar blue" << std::endl;
    fout << "property uchar red" << std::endl;
  }
  if (cloud.hasReflectances()) {
    fout << "property uint16 refc" << std::endl;
  }

  if (true) {
    fout << "property float nx" << std::endl;
    fout << "property float ny" << std::endl;
    fout << "property float nz" << std::endl;
  }

  if (cloud.hasf_dc_0()) {
    fout << "property float f_dc_0" << std::endl;
  }
  if (cloud.hasf_dc_1()) {
    fout << "property float f_dc_1" << std::endl;
  }
  if (cloud.hasf_dc_2()) {
    fout << "property float f_dc_2" << std::endl;
  }
  if (cloud.hasf_rest_0()) {
    fout << "property float f_rest_0" << std::endl;
  }
  if (cloud.hasf_rest_1()) {
    fout << "property float f_rest_1" << std::endl;
  }
  if (cloud.hasf_rest_2()) {
    fout << "property float f_rest_2" << std::endl;
  }
  if (cloud.hasf_rest_3()) {
    fout << "property float f_rest_3" << std::endl;
  }
  if (cloud.hasf_rest_4()) {
    fout << "property float f_rest_4" << std::endl;
  }
  if (cloud.hasf_rest_5()) {
    fout << "property float f_rest_5" << std::endl;
  }
  if (cloud.hasf_rest_6()) {
    fout << "property float f_rest_6" << std::endl;
  }
  if (cloud.hasf_rest_7()) {
    fout << "property float f_rest_7" << std::endl;
  }
  if (cloud.hasf_rest_8()) {
    fout << "property float f_rest_8" << std::endl;
  }
  if (cloud.hasf_rest_9()) {
    fout << "property float f_rest_9" << std::endl;
  }
  if (cloud.hasf_rest_10()) {
    fout << "property float f_rest_10" << std::endl;
  }
  if (cloud.hasf_rest_11()) {
    fout << "property float f_rest_11" << std::endl;
  }
  if (cloud.hasf_rest_12()) {
    fout << "property float f_rest_12" << std::endl;
  }
  if (cloud.hasf_rest_13()) {
    fout << "property float f_rest_13" << std::endl;
  }
  if (cloud.hasf_rest_14()) {
    fout << "property float f_rest_14" << std::endl;
  }
  if (cloud.hasf_rest_15()) {
    fout << "property float f_rest_15" << std::endl;
  }
  if (cloud.hasf_rest_16()) {
    fout << "property float f_rest_16" << std::endl;
  }
  if (cloud.hasf_rest_17()) {
    fout << "property float f_rest_17" << std::endl;
  }
  if (cloud.hasf_rest_18()) {
    fout << "property float f_rest_18" << std::endl;
  }
  if (cloud.hasf_rest_19()) {
    fout << "property float f_rest_19" << std::endl;
  }
  if (cloud.hasf_rest_20()) {
    fout << "property float f_rest_20" << std::endl;
  }
  if (cloud.hasf_rest_21()) {
    fout << "property float f_rest_21" << std::endl;
  }
  if (cloud.hasf_rest_22()) {
    fout << "property float f_rest_22" << std::endl;
  }
  if (cloud.hasf_rest_23()) {
    fout << "property float f_rest_23" << std::endl;
  }
  if (cloud.hasf_rest_24()) {
    fout << "property float f_rest_24" << std::endl;
  }
  if (cloud.hasf_rest_25()) {
    fout << "property float f_rest_25" << std::endl;
  }
  if (cloud.hasf_rest_26()) {
    fout << "property float f_rest_26" << std::endl;
  }
  if (cloud.hasf_rest_27()) {
    fout << "property float f_rest_27" << std::endl;
  }
  if (cloud.hasf_rest_28()) {
    fout << "property float f_rest_28" << std::endl;
  }
  if (cloud.hasf_rest_29()) {
    fout << "property float f_rest_29" << std::endl;
  }
  if (cloud.hasf_rest_30()) {
    fout << "property float f_rest_30" << std::endl;
  }
  if (cloud.hasf_rest_31()) {
    fout << "property float f_rest_31" << std::endl;
  }
  if (cloud.hasf_rest_32()) {
    fout << "property float f_rest_32" << std::endl;
  }
  if (cloud.hasf_rest_33()) {
    fout << "property float f_rest_33" << std::endl;
  }
  if (cloud.hasf_rest_34()) {
    fout << "property float f_rest_34" << std::endl;
  }
  if (cloud.hasf_rest_35()) {
    fout << "property float f_rest_35" << std::endl;
  }
  if (cloud.hasf_rest_36()) {
    fout << "property float f_rest_36" << std::endl;
  }
  if (cloud.hasf_rest_37()) {
    fout << "property float f_rest_37" << std::endl;
  }
  if (cloud.hasf_rest_38()) {
    fout << "property float f_rest_38" << std::endl;
  }
  if (cloud.hasf_rest_39()) {
    fout << "property float f_rest_39" << std::endl;
  }
  if (cloud.hasf_rest_40()) {
    fout << "property float f_rest_40" << std::endl;
  }
  if (cloud.hasf_rest_41()) {
    fout << "property float f_rest_41" << std::endl;
  }
  if (cloud.hasf_rest_42()) {
    fout << "property float f_rest_42" << std::endl;
  }
  if (cloud.hasf_rest_43()) {
    fout << "property float f_rest_43" << std::endl;
  }
  if (cloud.hasf_rest_44()) {
    fout << "property float f_rest_44" << std::endl;
  }
  if (cloud.hasOpacities()) {
    fout << "property float opacity" << std::endl;
  }
  if (cloud.hasscale_0()) {
    fout << "property float scale_0" << std::endl;
  }
  if (cloud.hasscale_1()) {
    fout << "property float scale_1" << std::endl;
  }
  if (cloud.hasscale_2()) {
    fout << "property float scale_2" << std::endl;
  }
  if (cloud.hasrot_0()) {
    fout << "property float rot_0" << std::endl;
  }
  if (cloud.hasrot_1()) {
    fout << "property float rot_1" << std::endl;
  }
  if (cloud.hasrot_2()) {
    fout << "property float rot_2" << std::endl;
  }
  if (cloud.hasrot_3()) {
    fout << "property float rot_3" << std::endl;
  }

  if (cloud.hasFrameIndex()) {
    fout << "property uint8 frameindex" << std::endl;
  }
  // fout << "element face 0" << std::endl;
  // fout << "property list uint8 int32 vertex_index" << std::endl;
  fout << "end_header" << std::endl;
  if (asAscii) {
    //      fout << std::setprecision(std::numeric_limits<double>::max_digits10);
    fout << std::fixed << std::setprecision(5);
    for (size_t i = 0; i < pointCount; ++i) {
      Vec3<double> position = cloud[i] * positionScale + positionOffset;

      fout << float(position.x()) / my_inputScale << " "
           << float(position.y()) / my_inputScale << " "
           << float(position.z()) / my_inputScale;
      if (cloud.hasColors()) {
        const Vec3<attr_t>& color = cloud.getColor(i);
        fout << " " << static_cast<int>(color[0]) << " "
             << static_cast<int>(color[1]) << " "
             << static_cast<int>(color[2]);
      }
      if (cloud.hasReflectances()) {
        fout << " " << static_cast<int>(cloud.getReflectance(i));
      }

      // for nx, ny, nz
      if (true) {
        fout << ' ' << 0;
        fout << ' ' << 0;
        fout << ' ' << 0;
      }

      if (cloud.hasf_dc_0()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_dc_0(i)), f_dc_0_min, f_dc_0_max,
                  f_dc_0_scale));
      }
      if (cloud.hasf_dc_1()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_dc_1(i)), f_dc_1_min, f_dc_1_max,
                  f_dc_1_scale));
      }
      if (cloud.hasf_dc_2()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_dc_2(i)), f_dc_2_min, f_dc_2_max,
                  f_dc_2_scale));
      }
      if (cloud.hasf_rest_0()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_0(i)), f_rest_0_min, f_rest_0_max,
                  f_rest_0_scale));
      }
      if (cloud.hasf_rest_1()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_1(i)), f_rest_1_min, f_rest_1_max,
                  f_rest_1_scale));
      }
      if (cloud.hasf_rest_2()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_2(i)), f_rest_2_min, f_rest_2_max,
                  f_rest_2_scale));
      }
      if (cloud.hasf_rest_3()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_3(i)), f_rest_3_min, f_rest_3_max,
                  f_rest_3_scale));
      }
      if (cloud.hasf_rest_4()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_4(i)), f_rest_4_min, f_rest_4_max,
                  f_rest_4_scale));
      }
      if (cloud.hasf_rest_5()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_5(i)), f_rest_5_min, f_rest_5_max,
                  f_rest_5_scale));
      }
      if (cloud.hasf_rest_6()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_6(i)), f_rest_6_min, f_rest_6_max,
                  f_rest_6_scale));
      }
      if (cloud.hasf_rest_7()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_7(i)), f_rest_7_min, f_rest_7_max,
                  f_rest_7_scale));
      }
      if (cloud.hasf_rest_8()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_8(i)), f_rest_8_min, f_rest_8_max,
                  f_rest_8_scale));
      }
      if (cloud.hasf_rest_9()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_9(i)), f_rest_9_min, f_rest_9_max,
                  f_rest_9_scale));
      }
      if (cloud.hasf_rest_10()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_10(i)), f_rest_10_min, f_rest_10_max,
                  f_rest_10_scale));
      }
      if (cloud.hasf_rest_11()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_11(i)), f_rest_11_min, f_rest_11_max,
                  f_rest_11_scale));
      }
      if (cloud.hasf_rest_12()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_12(i)), f_rest_12_min, f_rest_12_max,
                  f_rest_12_scale));
      }
      if (cloud.hasf_rest_13()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_13(i)), f_rest_13_min, f_rest_13_max,
                  f_rest_13_scale));
      }
      if (cloud.hasf_rest_14()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_14(i)), f_rest_14_min, f_rest_14_max,
                  f_rest_14_scale));
      }
      if (cloud.hasf_rest_15()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_15(i)), f_rest_15_min, f_rest_15_max,
                  f_rest_15_scale));
      }
      if (cloud.hasf_rest_16()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_16(i)), f_rest_16_min, f_rest_16_max,
                  f_rest_16_scale));
      }
      if (cloud.hasf_rest_17()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_17(i)), f_rest_17_min, f_rest_17_max,
                  f_rest_17_scale));
      }
      if (cloud.hasf_rest_18()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_18(i)), f_rest_18_min, f_rest_18_max,
                  f_rest_18_scale));
      }
      if (cloud.hasf_rest_19()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_19(i)), f_rest_19_min, f_rest_19_max,
                  f_rest_19_scale));
      }
      if (cloud.hasf_rest_20()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_20(i)), f_rest_20_min, f_rest_20_max,
                  f_rest_20_scale));
      }
      if (cloud.hasf_rest_21()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_21(i)), f_rest_21_min, f_rest_21_max,
                  f_rest_21_scale));
      }
      if (cloud.hasf_rest_22()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_22(i)), f_rest_22_min, f_rest_22_max,
                  f_rest_22_scale));
      }
      if (cloud.hasf_rest_23()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_23(i)), f_rest_23_min, f_rest_23_max,
                  f_rest_23_scale));
      }
      if (cloud.hasf_rest_24()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_24(i)), f_rest_24_min, f_rest_24_max,
                  f_rest_24_scale));
      }
      if (cloud.hasf_rest_25()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_25(i)), f_rest_25_min, f_rest_25_max,
                  f_rest_25_scale));
      }
      if (cloud.hasf_rest_26()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_26(i)), f_rest_26_min, f_rest_26_max,
                  f_rest_26_scale));
      }
      if (cloud.hasf_rest_27()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_27(i)), f_rest_27_min, f_rest_27_max,
                  f_rest_27_scale));
      }
      if (cloud.hasf_rest_28()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_28(i)), f_rest_28_min, f_rest_28_max,
                  f_rest_28_scale));
      }
      if (cloud.hasf_rest_29()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_29(i)), f_rest_29_min, f_rest_29_max,
                  f_rest_29_scale));
      }
      if (cloud.hasf_rest_30()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_30(i)), f_rest_30_min, f_rest_30_max,
                  f_rest_30_scale));
      }
      if (cloud.hasf_rest_31()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_31(i)), f_rest_31_min, f_rest_31_max,
                  f_rest_31_scale));
      }
      if (cloud.hasf_rest_32()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_32(i)), f_rest_32_min, f_rest_32_max,
                  f_rest_32_scale));
      }
      if (cloud.hasf_rest_33()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_33(i)), f_rest_33_min, f_rest_33_max,
                  f_rest_33_scale));
      }
      if (cloud.hasf_rest_34()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_34(i)), f_rest_34_min, f_rest_34_max,
                  f_rest_34_scale));
      }
      if (cloud.hasf_rest_35()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_35(i)), f_rest_35_min, f_rest_35_max,
                  f_rest_35_scale));
      }
      if (cloud.hasf_rest_36()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_36(i)), f_rest_36_min, f_rest_36_max,
                  f_rest_36_scale));
      }
      if (cloud.hasf_rest_37()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_37(i)), f_rest_37_min, f_rest_37_max,
                  f_rest_37_scale));
      }
      if (cloud.hasf_rest_38()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_38(i)), f_rest_38_min, f_rest_38_max,
                  f_rest_38_scale));
      }
      if (cloud.hasf_rest_39()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_39(i)), f_rest_39_min, f_rest_39_max,
                  f_rest_39_scale));
      }
      if (cloud.hasf_rest_40()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_40(i)), f_rest_40_min, f_rest_40_max,
                  f_rest_40_scale));
      }
      if (cloud.hasf_rest_41()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_41(i)), f_rest_41_min, f_rest_41_max,
                  f_rest_41_scale));
      }
      if (cloud.hasf_rest_42()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_42(i)), f_rest_42_min, f_rest_42_max,
                  f_rest_42_scale));
      }
      if (cloud.hasf_rest_43()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_43(i)), f_rest_43_min, f_rest_43_max,
                  f_rest_43_scale));
      }
      if (cloud.hasf_rest_44()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getf_rest_44(i)), f_rest_44_min, f_rest_44_max,
                  f_rest_44_scale));
      }
      if (cloud.hasOpacities()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getOpacity(i)), opacity_min, opacity_max,
                  opacity_scale));
      }
      if (cloud.hasscale_0()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getscale_0(i)), scale_0_min, scale_0_max,
                  scale_0_scale));
      }
      if (cloud.hasscale_1()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getscale_1(i)), scale_1_min, scale_1_max,
                  scale_1_scale));
      }
      if (cloud.hasscale_2()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getscale_2(i)), scale_2_min, scale_2_max,
                  scale_2_scale));
      }
      if (cloud.hasrot_0()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getrot_0(i)), rot_0_min, rot_0_max,
                  rot_0_scale));
      }
      if (cloud.hasrot_1()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getrot_1(i)), rot_1_min, rot_1_max,
                  rot_1_scale));
      }
      if (cloud.hasrot_2()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getrot_2(i)), rot_2_min, rot_2_max,
                  rot_2_scale));
      }
      if (cloud.hasrot_3()) {
        fout << ' '
             << static_cast<float>(reverse_value(
                  float(cloud.getrot_3(i)), rot_3_min, rot_3_max,
                  rot_3_scale));
      }

      if (cloud.hasFrameIndex()) {
        fout << " " << static_cast<int>(cloud.getFrameIndex(i));
      }
      fout << std::endl;
    }
  } else {
    fout.clear();
    fout.close();
    fout.open(fileName, std::ofstream::binary | std::ofstream::app);
    for (size_t i = 0; i < pointCount; ++i) {
      Vec3<double> position =
        (cloud[i] * positionScale + positionOffset) / my_inputScale;

      const float& position_x = position.x();
      const float& position_y = position.y();
      const float& position_z = position.z();

      fout.write(
        reinterpret_cast<const char* const>(&position_x), sizeof(float));
      fout.write(
        reinterpret_cast<const char* const>(&position_y), sizeof(float));
      fout.write(
        reinterpret_cast<const char* const>(&position_z), sizeof(float));

      if (cloud.hasColors()) {
        const Vec3<attr_t>& c = cloud.getColor(i);
        Vec3<uint8_t> val8b{uint8_t(c[0]), uint8_t(c[1]), uint8_t(c[2])};
        fout.write(reinterpret_cast<const char*>(&val8b), sizeof(uint8_t) * 3);
      }
      if (cloud.hasReflectances()) {
        const attr_t& reflectance = cloud.getReflectance(i);
        fout.write(
          reinterpret_cast<const char*>(&reflectance), sizeof(uint16_t));
      }

      // for nx, ny, nz
      if (true) {
        const float& my_n = 0;
        fout.write(reinterpret_cast<const char*>(&my_n), sizeof(float));
        fout.write(reinterpret_cast<const char*>(&my_n), sizeof(float));
        fout.write(reinterpret_cast<const char*>(&my_n), sizeof(float));
      }

      if (cloud.hasf_dc_0()) {
        const float& reverse_f_dc_0 = reverse_value(
          float(cloud.getf_dc_0(i)), f_dc_0_min, f_dc_0_max, f_dc_0_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_dc_0), sizeof(float));
      }
      if (cloud.hasf_dc_1()) {
        const float& reverse_f_dc_1 = reverse_value(
          float(cloud.getf_dc_1(i)), f_dc_1_min, f_dc_1_max, f_dc_1_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_dc_1), sizeof(float));
      }
      if (cloud.hasf_dc_2()) {
        const float& reverse_f_dc_2 = reverse_value(
          float(cloud.getf_dc_2(i)), f_dc_2_min, f_dc_2_max, f_dc_2_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_dc_2), sizeof(float));
      }
      if (cloud.hasf_rest_0()) {
        const float& reverse_f_rest_0 = reverse_value(
          float(cloud.getf_rest_0(i)), f_rest_0_min, f_rest_0_max,
          f_rest_0_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_0), sizeof(float));
      }
      if (cloud.hasf_rest_1()) {
        const float& reverse_f_rest_1 = reverse_value(
          float(cloud.getf_rest_1(i)), f_rest_1_min, f_rest_1_max,
          f_rest_1_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_1), sizeof(float));
      }
      if (cloud.hasf_rest_2()) {
        const float& reverse_f_rest_2 = reverse_value(
          float(cloud.getf_rest_2(i)), f_rest_2_min, f_rest_2_max,
          f_rest_2_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_2), sizeof(float));
      }
      if (cloud.hasf_rest_3()) {
        const float& reverse_f_rest_3 = reverse_value(
          float(cloud.getf_rest_3(i)), f_rest_3_min, f_rest_3_max,
          f_rest_3_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_3), sizeof(float));
      }
      if (cloud.hasf_rest_4()) {
        const float& reverse_f_rest_4 = reverse_value(
          float(cloud.getf_rest_4(i)), f_rest_4_min, f_rest_4_max,
          f_rest_4_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_4), sizeof(float));
      }
      if (cloud.hasf_rest_5()) {
        const float& reverse_f_rest_5 = reverse_value(
          float(cloud.getf_rest_5(i)), f_rest_5_min, f_rest_5_max,
          f_rest_5_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_5), sizeof(float));
      }
      if (cloud.hasf_rest_6()) {
        const float& reverse_f_rest_6 = reverse_value(
          float(cloud.getf_rest_6(i)), f_rest_6_min, f_rest_6_max,
          f_rest_6_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_6), sizeof(float));
      }
      if (cloud.hasf_rest_7()) {
        const float& reverse_f_rest_7 = reverse_value(
          float(cloud.getf_rest_7(i)), f_rest_7_min, f_rest_7_max,
          f_rest_7_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_7), sizeof(float));
      }
      if (cloud.hasf_rest_8()) {
        const float& reverse_f_rest_8 = reverse_value(
          float(cloud.getf_rest_8(i)), f_rest_8_min, f_rest_8_max,
          f_rest_8_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_8), sizeof(float));
      }
      if (cloud.hasf_rest_9()) {
        const float& reverse_f_rest_9 = reverse_value(
          float(cloud.getf_rest_9(i)), f_rest_9_min, f_rest_9_max,
          f_rest_9_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_9), sizeof(float));
      }
      if (cloud.hasf_rest_10()) {
        const float& reverse_f_rest_10 = reverse_value(
          float(cloud.getf_rest_10(i)), f_rest_10_min, f_rest_10_max,
          f_rest_10_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_10), sizeof(float));
      }
      if (cloud.hasf_rest_11()) {
        const float& reverse_f_rest_11 = reverse_value(
          float(cloud.getf_rest_11(i)), f_rest_11_min, f_rest_11_max,
          f_rest_11_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_11), sizeof(float));
      }
      if (cloud.hasf_rest_12()) {
        const float& reverse_f_rest_12 = reverse_value(
          float(cloud.getf_rest_12(i)), f_rest_12_min, f_rest_12_max,
          f_rest_12_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_12), sizeof(float));
      }
      if (cloud.hasf_rest_13()) {
        const float& reverse_f_rest_13 = reverse_value(
          float(cloud.getf_rest_13(i)), f_rest_13_min, f_rest_13_max,
          f_rest_13_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_13), sizeof(float));
      }
      if (cloud.hasf_rest_14()) {
        const float& reverse_f_rest_14 = reverse_value(
          float(cloud.getf_rest_14(i)), f_rest_14_min, f_rest_14_max,
          f_rest_14_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_14), sizeof(float));
      }
      if (cloud.hasf_rest_15()) {
        const float& reverse_f_rest_15 = reverse_value(
          float(cloud.getf_rest_15(i)), f_rest_15_min, f_rest_15_max,
          f_rest_15_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_15), sizeof(float));
      }
      if (cloud.hasf_rest_16()) {
        const float& reverse_f_rest_16 = reverse_value(
          float(cloud.getf_rest_16(i)), f_rest_16_min, f_rest_16_max,
          f_rest_16_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_16), sizeof(float));
      }
      if (cloud.hasf_rest_17()) {
        const float& reverse_f_rest_17 = reverse_value(
          float(cloud.getf_rest_17(i)), f_rest_17_min, f_rest_17_max,
          f_rest_17_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_17), sizeof(float));
      }
      if (cloud.hasf_rest_18()) {
        const float& reverse_f_rest_18 = reverse_value(
          float(cloud.getf_rest_18(i)), f_rest_18_min, f_rest_18_max,
          f_rest_18_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_18), sizeof(float));
      }
      if (cloud.hasf_rest_19()) {
        const float& reverse_f_rest_19 = reverse_value(
          float(cloud.getf_rest_19(i)), f_rest_19_min, f_rest_19_max,
          f_rest_19_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_19), sizeof(float));
      }
      if (cloud.hasf_rest_20()) {
        const float& reverse_f_rest_20 = reverse_value(
          float(cloud.getf_rest_20(i)), f_rest_20_min, f_rest_20_max,
          f_rest_20_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_20), sizeof(float));
      }
      if (cloud.hasf_rest_21()) {
        const float& reverse_f_rest_21 = reverse_value(
          float(cloud.getf_rest_21(i)), f_rest_21_min, f_rest_21_max,
          f_rest_21_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_21), sizeof(float));
      }
      if (cloud.hasf_rest_22()) {
        const float& reverse_f_rest_22 = reverse_value(
          float(cloud.getf_rest_22(i)), f_rest_22_min, f_rest_22_max,
          f_rest_22_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_22), sizeof(float));
      }
      if (cloud.hasf_rest_23()) {
        const float& reverse_f_rest_23 = reverse_value(
          float(cloud.getf_rest_23(i)), f_rest_23_min, f_rest_23_max,
          f_rest_23_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_23), sizeof(float));
      }
      if (cloud.hasf_rest_24()) {
        const float& reverse_f_rest_24 = reverse_value(
          float(cloud.getf_rest_24(i)), f_rest_24_min, f_rest_24_max,
          f_rest_24_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_24), sizeof(float));
      }
      if (cloud.hasf_rest_25()) {
        const float& reverse_f_rest_25 = reverse_value(
          float(cloud.getf_rest_25(i)), f_rest_25_min, f_rest_25_max,
          f_rest_25_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_25), sizeof(float));
      }
      if (cloud.hasf_rest_26()) {
        const float& reverse_f_rest_26 = reverse_value(
          float(cloud.getf_rest_26(i)), f_rest_26_min, f_rest_26_max,
          f_rest_26_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_26), sizeof(float));
      }
      if (cloud.hasf_rest_27()) {
        const float& reverse_f_rest_27 = reverse_value(
          float(cloud.getf_rest_27(i)), f_rest_27_min, f_rest_27_max,
          f_rest_27_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_27), sizeof(float));
      }
      if (cloud.hasf_rest_28()) {
        const float& reverse_f_rest_28 = reverse_value(
          float(cloud.getf_rest_28(i)), f_rest_28_min, f_rest_28_max,
          f_rest_28_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_28), sizeof(float));
      }
      if (cloud.hasf_rest_29()) {
        const float& reverse_f_rest_29 = reverse_value(
          float(cloud.getf_rest_29(i)), f_rest_29_min, f_rest_29_max,
          f_rest_29_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_29), sizeof(float));
      }
      if (cloud.hasf_rest_30()) {
        const float& reverse_f_rest_30 = reverse_value(
          float(cloud.getf_rest_30(i)), f_rest_30_min, f_rest_30_max,
          f_rest_30_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_30), sizeof(float));
      }
      if (cloud.hasf_rest_31()) {
        const float& reverse_f_rest_31 = reverse_value(
          float(cloud.getf_rest_31(i)), f_rest_31_min, f_rest_31_max,
          f_rest_31_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_31), sizeof(float));
      }
      if (cloud.hasf_rest_32()) {
        const float& reverse_f_rest_32 = reverse_value(
          float(cloud.getf_rest_32(i)), f_rest_32_min, f_rest_32_max,
          f_rest_32_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_32), sizeof(float));
      }
      if (cloud.hasf_rest_33()) {
        const float& reverse_f_rest_33 = reverse_value(
          float(cloud.getf_rest_33(i)), f_rest_33_min, f_rest_33_max,
          f_rest_33_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_33), sizeof(float));
      }
      if (cloud.hasf_rest_34()) {
        const float& reverse_f_rest_34 = reverse_value(
          float(cloud.getf_rest_34(i)), f_rest_34_min, f_rest_34_max,
          f_rest_34_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_34), sizeof(float));
      }
      if (cloud.hasf_rest_35()) {
        const float& reverse_f_rest_35 = reverse_value(
          float(cloud.getf_rest_35(i)), f_rest_35_min, f_rest_35_max,
          f_rest_35_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_35), sizeof(float));
      }
      if (cloud.hasf_rest_36()) {
        const float& reverse_f_rest_36 = reverse_value(
          float(cloud.getf_rest_36(i)), f_rest_36_min, f_rest_36_max,
          f_rest_36_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_36), sizeof(float));
      }
      if (cloud.hasf_rest_37()) {
        const float& reverse_f_rest_37 = reverse_value(
          float(cloud.getf_rest_37(i)), f_rest_37_min, f_rest_37_max,
          f_rest_37_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_37), sizeof(float));
      }
      if (cloud.hasf_rest_38()) {
        const float& reverse_f_rest_38 = reverse_value(
          float(cloud.getf_rest_38(i)), f_rest_38_min, f_rest_38_max,
          f_rest_38_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_38), sizeof(float));
      }
      if (cloud.hasf_rest_39()) {
        const float& reverse_f_rest_39 = reverse_value(
          float(cloud.getf_rest_39(i)), f_rest_39_min, f_rest_39_max,
          f_rest_39_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_39), sizeof(float));
      }
      if (cloud.hasf_rest_40()) {
        const float& reverse_f_rest_40 = reverse_value(
          float(cloud.getf_rest_40(i)), f_rest_40_min, f_rest_40_max,
          f_rest_40_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_40), sizeof(float));
      }
      if (cloud.hasf_rest_41()) {
        const float& reverse_f_rest_41 = reverse_value(
          float(cloud.getf_rest_41(i)), f_rest_41_min, f_rest_41_max,
          f_rest_41_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_41), sizeof(float));
      }
      if (cloud.hasf_rest_42()) {
        const float& reverse_f_rest_42 = reverse_value(
          float(cloud.getf_rest_42(i)), f_rest_42_min, f_rest_42_max,
          f_rest_42_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_42), sizeof(float));
      }
      if (cloud.hasf_rest_43()) {
        const float& reverse_f_rest_43 = reverse_value(
          float(cloud.getf_rest_43(i)), f_rest_43_min, f_rest_43_max,
          f_rest_43_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_43), sizeof(float));
      }
      if (cloud.hasf_rest_44()) {
        const float& reverse_f_rest_44 = reverse_value(
          float(cloud.getf_rest_44(i)), f_rest_44_min, f_rest_44_max,
          f_rest_44_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_f_rest_44), sizeof(float));
      }

      if (cloud.hasOpacities()) {
        const float& reverse_opacity = reverse_value(
          float(cloud.getOpacity(i)), opacity_min, opacity_max, opacity_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_opacity), sizeof(float));
      }
      if (cloud.hasscale_0()) {
        const float& reverse_scale_0 = reverse_value(
          float(cloud.getscale_0(i)), scale_0_min, scale_0_max, scale_0_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_scale_0), sizeof(float));
      }
      if (cloud.hasscale_1()) {
        const float& reverse_scale_1 = reverse_value(
          float(cloud.getscale_1(i)), scale_1_min, scale_1_max, scale_1_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_scale_1), sizeof(float));
      }
      if (cloud.hasscale_2()) {
        const float& reverse_scale_2 = reverse_value(
          float(cloud.getscale_2(i)), scale_2_min, scale_2_max, scale_2_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_scale_2), sizeof(float));
      }
      if (cloud.hasrot_0()) {
        const float& reverse_rot_0 = reverse_value(
          float(cloud.getrot_0(i)), rot_0_min, rot_0_max, rot_0_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_rot_0), sizeof(float));
      }
      if (cloud.hasrot_1()) {
        const float& reverse_rot_1 = reverse_value(
          float(cloud.getrot_1(i)), rot_1_min, rot_1_max, rot_1_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_rot_1), sizeof(float));
      }
      if (cloud.hasrot_2()) {
        const float& reverse_rot_2 = reverse_value(
          float(cloud.getrot_2(i)), rot_2_min, rot_2_max, rot_2_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_rot_2), sizeof(float));
      }
      if (cloud.hasrot_3()) {
        const float& reverse_rot_3 = reverse_value(
          float(cloud.getrot_3(i)), rot_3_min, rot_3_max, rot_3_scale);
        fout.write(
          reinterpret_cast<const char*>(&reverse_rot_3), sizeof(float));
      }

      if (cloud.hasFrameIndex()) {
        const uint16_t& findex = cloud.getFrameIndex(i);
        fout.write(reinterpret_cast<const char*>(&findex), sizeof(uint16_t));
      }
    }
  }
  fout.close();
  return true;
}

//============================================================================

bool
ply::read(
  const std::string& fileName,
  const PropertyNameMap& attributeNames,
  double positionScale,
  PCCPointSet3& cloud)
{
  std::ifstream ifs(fileName, std::ifstream::in | std::ifstream::binary);
  if (!ifs.is_open()) {
    return false;
  }

  enum AttributeType
  {
    ATTRIBUTE_TYPE_FLOAT64 = 0,
    ATTRIBUTE_TYPE_FLOAT32 = 1,
    ATTRIBUTE_TYPE_UINT64 = 2,
    ATTRIBUTE_TYPE_UINT32 = 3,
    ATTRIBUTE_TYPE_UINT16 = 4,
    ATTRIBUTE_TYPE_UINT8 = 5,
    ATTRIBUTE_TYPE_INT64 = 6,
    ATTRIBUTE_TYPE_INT32 = 7,
    ATTRIBUTE_TYPE_INT16 = 8,
    ATTRIBUTE_TYPE_INT8 = 9,
  };
  struct AttributeInfo {
    std::string name;
    AttributeType type;
    size_t byteCount;
  };

  std::vector<AttributeInfo> attributesInfo;
  attributesInfo.reserve(16);
  const size_t MAX_BUFFER_SIZE = 4096;
  char tmp[MAX_BUFFER_SIZE];
  const char* sep = " \t\r";
  std::vector<std::string> tokens;

  ifs.getline(tmp, MAX_BUFFER_SIZE);
  getTokens(tmp, sep, tokens);
  if (tokens.empty() || tokens[0] != "ply") {
    std::cout << "Error: corrupted file!" << std::endl;
    return false;
  }
  bool isAscii = false;
  double version = 1.0;
  size_t pointCount = 0;
  bool isVertexProperty = true;
  while (1) {
    if (ifs.eof()) {
      std::cout << "Error: corrupted header!" << std::endl;
      return false;
    }
    ifs.getline(tmp, MAX_BUFFER_SIZE);
    getTokens(tmp, sep, tokens);
    if (tokens.empty() || tokens[0] == "comment") {
      continue;
    }
    if (tokens[0] == "format") {
      if (tokens.size() != 3) {
        std::cout << "Error: corrupted format info!" << std::endl;
        return false;
      }
      isAscii = tokens[1] == "ascii";
      version = atof(tokens[2].c_str());
    } else if (tokens[0] == "element") {
      if (tokens.size() != 3) {
        std::cout << "Error: corrupted element info!" << std::endl;
        return false;
      }
      if (tokens[1] == "vertex") {
        pointCount = atoi(tokens[2].c_str());
      } else {
        isVertexProperty = false;
      }
    } else if (tokens[0] == "property" && isVertexProperty) {
      if (tokens.size() != 3) {
        std::cout << "Error: corrupted property info!" << std::endl;
        return false;
      }
      const std::string& propertyType = tokens[1];
      const std::string& propertyName = tokens[2];
      const size_t attributeIndex = attributesInfo.size();
      attributesInfo.resize(attributeIndex + 1);
      AttributeInfo& attributeInfo = attributesInfo[attributeIndex];
      attributeInfo.name = propertyName;
      if (propertyType == "float64") {
        attributeInfo.type = ATTRIBUTE_TYPE_FLOAT64;
        attributeInfo.byteCount = 8;
      } else if (propertyType == "float" || propertyType == "float32") {
        attributeInfo.type = ATTRIBUTE_TYPE_FLOAT32;
        attributeInfo.byteCount = 4;
      } else if (propertyType == "uint64") {
        attributeInfo.type = ATTRIBUTE_TYPE_UINT64;
        attributeInfo.byteCount = 8;
      } else if (propertyType == "uint32") {
        attributeInfo.type = ATTRIBUTE_TYPE_UINT32;
        attributeInfo.byteCount = 4;
      } else if (propertyType == "uint16") {
        attributeInfo.type = ATTRIBUTE_TYPE_UINT16;
        attributeInfo.byteCount = 2;
      } else if (propertyType == "uchar" || propertyType == "uint8") {
        attributeInfo.type = ATTRIBUTE_TYPE_UINT8;
        attributeInfo.byteCount = 1;
      } else if (propertyType == "int64") {
        attributeInfo.type = ATTRIBUTE_TYPE_INT64;
        attributeInfo.byteCount = 8;
      } else if (propertyType == "int32") {
        attributeInfo.type = ATTRIBUTE_TYPE_INT32;
        attributeInfo.byteCount = 4;
      } else if (propertyType == "int16") {
        attributeInfo.type = ATTRIBUTE_TYPE_INT16;
        attributeInfo.byteCount = 2;
      } else if (propertyType == "char" || propertyType == "int8") {
        attributeInfo.type = ATTRIBUTE_TYPE_INT8;
        attributeInfo.byteCount = 1;
      }
    } else if (tokens[0] == "end_header") {
      break;
    }
  }
  if (version != 1.0) {
    std::cout << "Error: non-supported version!" << std::endl;
    return false;
  }

  size_t indexX = PCC_UNDEFINED_INDEX;
  size_t indexY = PCC_UNDEFINED_INDEX;
  size_t indexZ = PCC_UNDEFINED_INDEX;
  size_t indexR = PCC_UNDEFINED_INDEX;
  size_t indexG = PCC_UNDEFINED_INDEX;
  size_t indexB = PCC_UNDEFINED_INDEX;
  size_t indexReflectance = PCC_UNDEFINED_INDEX;
  size_t indexFrame = PCC_UNDEFINED_INDEX;
  size_t indexNX = PCC_UNDEFINED_INDEX;
  size_t indexNY = PCC_UNDEFINED_INDEX;
  size_t indexNZ = PCC_UNDEFINED_INDEX;
  size_t indexLaserAngle = PCC_UNDEFINED_INDEX;
  size_t indexf_dc_0 = PCC_UNDEFINED_INDEX;
  size_t indexf_dc_1 = PCC_UNDEFINED_INDEX;
  size_t indexf_dc_2 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_0 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_1 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_2 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_3 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_4 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_5 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_6 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_7 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_8 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_9 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_10 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_11 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_12 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_13 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_14 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_15 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_16 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_17 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_18 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_19 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_20 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_21 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_22 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_23 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_24 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_25 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_26 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_27 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_28 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_29 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_30 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_31 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_32 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_33 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_34 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_35 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_36 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_37 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_38 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_39 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_40 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_41 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_42 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_43 = PCC_UNDEFINED_INDEX;
  size_t indexf_rest_44 = PCC_UNDEFINED_INDEX;
  size_t indexopacity = PCC_UNDEFINED_INDEX;
  size_t indexscale_0 = PCC_UNDEFINED_INDEX;
  size_t indexscale_1 = PCC_UNDEFINED_INDEX;
  size_t indexscale_2 = PCC_UNDEFINED_INDEX;
  size_t indexrot_0 = PCC_UNDEFINED_INDEX;
  size_t indexrot_1 = PCC_UNDEFINED_INDEX;
  size_t indexrot_2 = PCC_UNDEFINED_INDEX;
  size_t indexrot_3 = PCC_UNDEFINED_INDEX;

  const size_t attributeCount = attributesInfo.size();
  for (size_t a = 0; a < attributeCount; ++a) {
    const auto& attributeInfo = attributesInfo[a];
    if (
      attributeInfo.name == attributeNames.position[0]
      && (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4)) {
      indexX = a;
    } else if (
      attributeInfo.name == attributeNames.position[1]
      && (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4)) {
      indexY = a;
    } else if (
      attributeInfo.name == attributeNames.position[2]
      && (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4)) {
      indexZ = a;
    } else if (attributeInfo.name == "red" && attributeInfo.byteCount == 1) {
      indexR = a;
    } else if (attributeInfo.name == "green" && attributeInfo.byteCount == 1) {
      indexG = a;
    } else if (attributeInfo.name == "blue" && attributeInfo.byteCount == 1) {
      indexB = a;
    } else if (
      (attributeInfo.name == "reflectance" || attributeInfo.name == "refc")
      && attributeInfo.byteCount <= 2) {
      indexReflectance = a;
    } else if (
      attributeInfo.name == "frameindex" && attributeInfo.byteCount <= 2) {
      indexFrame = a;
    } else if (
      attributeInfo.name == "nx"
      && (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4)) {
      indexNX = a;
    } else if (
      attributeInfo.name == "ny"
      && (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4)) {
      indexNY = a;
    } else if (
      attributeInfo.name == "nz"
      && (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4)) {
      indexNZ = a;
    } else if (attributeInfo.name == "laserangle") {
      indexLaserAngle = a;
    } else if (attributeInfo.name == "opacity") {
      indexopacity = a;
    } else if (attributeInfo.name == "f_dc_0") {
      indexf_dc_0 = a;
    } else if (attributeInfo.name == "f_dc_1") {
      indexf_dc_1 = a;
    } else if (attributeInfo.name == "f_dc_2") {
      indexf_dc_2 = a;
    } else if (attributeInfo.name == "scale_0") {
      indexscale_0 = a;
    } else if (attributeInfo.name == "scale_1") {
      indexscale_1 = a;
    } else if (attributeInfo.name == "scale_2") {
      indexscale_2 = a;
    } else if (attributeInfo.name == "rot_0") {
      indexrot_0 = a;
    } else if (attributeInfo.name == "rot_1") {
      indexrot_1 = a;
    } else if (attributeInfo.name == "rot_2") {
      indexrot_2 = a;
    } else if (attributeInfo.name == "rot_3") {
      indexrot_3 = a;
    } else if (attributeInfo.name == "f_rest_0") {
      indexf_rest_0 = a;
    } else if (attributeInfo.name == "f_rest_1") {
      indexf_rest_1 = a;
    } else if (attributeInfo.name == "f_rest_2") {
      indexf_rest_2 = a;
    } else if (attributeInfo.name == "f_rest_3") {
      indexf_rest_3 = a;
    } else if (attributeInfo.name == "f_rest_4") {
      indexf_rest_4 = a;
    } else if (attributeInfo.name == "f_rest_5") {
      indexf_rest_5 = a;
    } else if (attributeInfo.name == "f_rest_6") {
      indexf_rest_6 = a;
    } else if (attributeInfo.name == "f_rest_7") {
      indexf_rest_7 = a;
    } else if (attributeInfo.name == "f_rest_8") {
      indexf_rest_8 = a;
    } else if (attributeInfo.name == "f_rest_9") {
      indexf_rest_9 = a;
    } else if (attributeInfo.name == "f_rest_10") {
      indexf_rest_10 = a;
    } else if (attributeInfo.name == "f_rest_11") {
      indexf_rest_11 = a;
    } else if (attributeInfo.name == "f_rest_12") {
      indexf_rest_12 = a;
    } else if (attributeInfo.name == "f_rest_13") {
      indexf_rest_13 = a;
    } else if (attributeInfo.name == "f_rest_14") {
      indexf_rest_14 = a;
    } else if (attributeInfo.name == "f_rest_15") {
      indexf_rest_15 = a;
    } else if (attributeInfo.name == "f_rest_16") {
      indexf_rest_16 = a;
    } else if (attributeInfo.name == "f_rest_17") {
      indexf_rest_17 = a;
    } else if (attributeInfo.name == "f_rest_18") {
      indexf_rest_18 = a;
    } else if (attributeInfo.name == "f_rest_19") {
      indexf_rest_19 = a;
    } else if (attributeInfo.name == "f_rest_20") {
      indexf_rest_20 = a;
    } else if (attributeInfo.name == "f_rest_21") {
      indexf_rest_21 = a;
    } else if (attributeInfo.name == "f_rest_22") {
      indexf_rest_22 = a;
    } else if (attributeInfo.name == "f_rest_23") {
      indexf_rest_23 = a;
    } else if (attributeInfo.name == "f_rest_24") {
      indexf_rest_24 = a;
    } else if (attributeInfo.name == "f_rest_25") {
      indexf_rest_25 = a;
    } else if (attributeInfo.name == "f_rest_26") {
      indexf_rest_26 = a;
    } else if (attributeInfo.name == "f_rest_27") {
      indexf_rest_27 = a;
    } else if (attributeInfo.name == "f_rest_28") {
      indexf_rest_28 = a;
    } else if (attributeInfo.name == "f_rest_29") {
      indexf_rest_29 = a;
    } else if (attributeInfo.name == "f_rest_30") {
      indexf_rest_30 = a;
    } else if (attributeInfo.name == "f_rest_31") {
      indexf_rest_31 = a;
    } else if (attributeInfo.name == "f_rest_32") {
      indexf_rest_32 = a;
    } else if (attributeInfo.name == "f_rest_33") {
      indexf_rest_33 = a;
    } else if (attributeInfo.name == "f_rest_34") {
      indexf_rest_34 = a;
    } else if (attributeInfo.name == "f_rest_35") {
      indexf_rest_35 = a;
    } else if (attributeInfo.name == "f_rest_36") {
      indexf_rest_36 = a;
    } else if (attributeInfo.name == "f_rest_37") {
      indexf_rest_37 = a;
    } else if (attributeInfo.name == "f_rest_38") {
      indexf_rest_38 = a;
    } else if (attributeInfo.name == "f_rest_39") {
      indexf_rest_39 = a;
    } else if (attributeInfo.name == "f_rest_40") {
      indexf_rest_40 = a;
    } else if (attributeInfo.name == "f_rest_41") {
      indexf_rest_41 = a;
    } else if (attributeInfo.name == "f_rest_42") {
      indexf_rest_42 = a;
    } else if (attributeInfo.name == "f_rest_43") {
      indexf_rest_43 = a;
    } else if (attributeInfo.name == "f_rest_44") {
      indexf_rest_44 = a;
    } else {
      std::cout << attributeInfo.name << std::endl;
    }
  }
  if (
    indexX == PCC_UNDEFINED_INDEX || indexY == PCC_UNDEFINED_INDEX
    || indexZ == PCC_UNDEFINED_INDEX) {
    std::cout << "Error: missing coordinates!" << std::endl;
    return false;
  }
  bool withColors = indexR != PCC_UNDEFINED_INDEX
    && indexG != PCC_UNDEFINED_INDEX && indexB != PCC_UNDEFINED_INDEX;
  bool withReflectances = indexReflectance != PCC_UNDEFINED_INDEX;
  bool withOpacities = indexopacity != PCC_UNDEFINED_INDEX;
  bool withf_dc_0 = indexf_dc_0 != PCC_UNDEFINED_INDEX;
  bool withf_dc_1 = indexf_dc_1 != PCC_UNDEFINED_INDEX;
  bool withf_dc_2 = indexf_dc_2 != PCC_UNDEFINED_INDEX;
  bool withscale_0 = indexscale_0 != PCC_UNDEFINED_INDEX;
  bool withscale_1 = indexscale_1 != PCC_UNDEFINED_INDEX;
  bool withscale_2 = indexscale_2 != PCC_UNDEFINED_INDEX;
  bool withrot_0 = indexrot_0 != PCC_UNDEFINED_INDEX;
  bool withrot_1 = indexrot_1 != PCC_UNDEFINED_INDEX;
  bool withrot_2 = indexrot_2 != PCC_UNDEFINED_INDEX;
  bool withrot_3 = indexrot_3 != PCC_UNDEFINED_INDEX;
  bool withf_rest_0 = indexf_rest_0 != PCC_UNDEFINED_INDEX;
  bool withf_rest_1 = indexf_rest_1 != PCC_UNDEFINED_INDEX;
  bool withf_rest_2 = indexf_rest_2 != PCC_UNDEFINED_INDEX;
  bool withf_rest_3 = indexf_rest_3 != PCC_UNDEFINED_INDEX;
  bool withf_rest_4 = indexf_rest_4 != PCC_UNDEFINED_INDEX;
  bool withf_rest_5 = indexf_rest_5 != PCC_UNDEFINED_INDEX;
  bool withf_rest_6 = indexf_rest_6 != PCC_UNDEFINED_INDEX;
  bool withf_rest_7 = indexf_rest_7 != PCC_UNDEFINED_INDEX;
  bool withf_rest_8 = indexf_rest_8 != PCC_UNDEFINED_INDEX;
  bool withf_rest_9 = indexf_rest_9 != PCC_UNDEFINED_INDEX;
  bool withf_rest_10 = indexf_rest_10 != PCC_UNDEFINED_INDEX;
  bool withf_rest_11 = indexf_rest_11 != PCC_UNDEFINED_INDEX;
  bool withf_rest_12 = indexf_rest_12 != PCC_UNDEFINED_INDEX;
  bool withf_rest_13 = indexf_rest_13 != PCC_UNDEFINED_INDEX;
  bool withf_rest_14 = indexf_rest_14 != PCC_UNDEFINED_INDEX;
  bool withf_rest_15 = indexf_rest_15 != PCC_UNDEFINED_INDEX;
  bool withf_rest_16 = indexf_rest_16 != PCC_UNDEFINED_INDEX;
  bool withf_rest_17 = indexf_rest_17 != PCC_UNDEFINED_INDEX;
  bool withf_rest_18 = indexf_rest_18 != PCC_UNDEFINED_INDEX;
  bool withf_rest_19 = indexf_rest_19 != PCC_UNDEFINED_INDEX;
  bool withf_rest_20 = indexf_rest_20 != PCC_UNDEFINED_INDEX;
  bool withf_rest_21 = indexf_rest_21 != PCC_UNDEFINED_INDEX;
  bool withf_rest_22 = indexf_rest_22 != PCC_UNDEFINED_INDEX;
  bool withf_rest_23 = indexf_rest_23 != PCC_UNDEFINED_INDEX;
  bool withf_rest_24 = indexf_rest_24 != PCC_UNDEFINED_INDEX;
  bool withf_rest_25 = indexf_rest_25 != PCC_UNDEFINED_INDEX;
  bool withf_rest_26 = indexf_rest_26 != PCC_UNDEFINED_INDEX;
  bool withf_rest_27 = indexf_rest_27 != PCC_UNDEFINED_INDEX;
  bool withf_rest_28 = indexf_rest_28 != PCC_UNDEFINED_INDEX;
  bool withf_rest_29 = indexf_rest_29 != PCC_UNDEFINED_INDEX;
  bool withf_rest_30 = indexf_rest_30 != PCC_UNDEFINED_INDEX;
  bool withf_rest_31 = indexf_rest_31 != PCC_UNDEFINED_INDEX;
  bool withf_rest_32 = indexf_rest_32 != PCC_UNDEFINED_INDEX;
  bool withf_rest_33 = indexf_rest_33 != PCC_UNDEFINED_INDEX;
  bool withf_rest_34 = indexf_rest_34 != PCC_UNDEFINED_INDEX;
  bool withf_rest_35 = indexf_rest_35 != PCC_UNDEFINED_INDEX;
  bool withf_rest_36 = indexf_rest_36 != PCC_UNDEFINED_INDEX;
  bool withf_rest_37 = indexf_rest_37 != PCC_UNDEFINED_INDEX;
  bool withf_rest_38 = indexf_rest_38 != PCC_UNDEFINED_INDEX;
  bool withf_rest_39 = indexf_rest_39 != PCC_UNDEFINED_INDEX;
  bool withf_rest_40 = indexf_rest_40 != PCC_UNDEFINED_INDEX;
  bool withf_rest_41 = indexf_rest_41 != PCC_UNDEFINED_INDEX;
  bool withf_rest_42 = indexf_rest_42 != PCC_UNDEFINED_INDEX;
  bool withf_rest_43 = indexf_rest_43 != PCC_UNDEFINED_INDEX;
  bool withf_rest_44 = indexf_rest_44 != PCC_UNDEFINED_INDEX;

  bool withFrameIndex = indexFrame != PCC_UNDEFINED_INDEX;
  bool withLaserAngles = indexLaserAngle != PCC_UNDEFINED_INDEX;

  cloud.addRemoveAttributes(withColors, withReflectances);
  cloud.addRemoveOpacities(withOpacities);
  cloud.addRemovef_dc_0(withf_dc_0);
  cloud.addRemovef_dc_1(withf_dc_1);
  cloud.addRemovef_dc_2(withf_dc_2);
  cloud.addRemovescale_0(withscale_0);
  cloud.addRemovescale_1(withscale_1);
  cloud.addRemovescale_2(withscale_2);
  cloud.addRemoverot_0(withrot_0);
  cloud.addRemoverot_1(withrot_1);
  cloud.addRemoverot_2(withrot_2);
  cloud.addRemoverot_3(withrot_3);
  cloud.addRemovef_rest_0(withf_rest_0);
  cloud.addRemovef_rest_1(withf_rest_1);
  cloud.addRemovef_rest_2(withf_rest_2);
  cloud.addRemovef_rest_3(withf_rest_3);
  cloud.addRemovef_rest_4(withf_rest_4);
  cloud.addRemovef_rest_5(withf_rest_5);
  cloud.addRemovef_rest_6(withf_rest_6);
  cloud.addRemovef_rest_7(withf_rest_7);
  cloud.addRemovef_rest_8(withf_rest_8);
  cloud.addRemovef_rest_9(withf_rest_9);
  cloud.addRemovef_rest_10(withf_rest_10);
  cloud.addRemovef_rest_11(withf_rest_11);
  cloud.addRemovef_rest_12(withf_rest_12);
  cloud.addRemovef_rest_13(withf_rest_13);
  cloud.addRemovef_rest_14(withf_rest_14);
  cloud.addRemovef_rest_15(withf_rest_15);
  cloud.addRemovef_rest_16(withf_rest_16);
  cloud.addRemovef_rest_17(withf_rest_17);
  cloud.addRemovef_rest_18(withf_rest_18);
  cloud.addRemovef_rest_19(withf_rest_19);
  cloud.addRemovef_rest_20(withf_rest_20);
  cloud.addRemovef_rest_21(withf_rest_21);
  cloud.addRemovef_rest_22(withf_rest_22);
  cloud.addRemovef_rest_23(withf_rest_23);
  cloud.addRemovef_rest_24(withf_rest_24);
  cloud.addRemovef_rest_25(withf_rest_25);
  cloud.addRemovef_rest_26(withf_rest_26);
  cloud.addRemovef_rest_27(withf_rest_27);
  cloud.addRemovef_rest_28(withf_rest_28);
  cloud.addRemovef_rest_29(withf_rest_29);
  cloud.addRemovef_rest_30(withf_rest_30);
  cloud.addRemovef_rest_31(withf_rest_31);
  cloud.addRemovef_rest_32(withf_rest_32);
  cloud.addRemovef_rest_33(withf_rest_33);
  cloud.addRemovef_rest_34(withf_rest_34);
  cloud.addRemovef_rest_35(withf_rest_35);
  cloud.addRemovef_rest_36(withf_rest_36);
  cloud.addRemovef_rest_37(withf_rest_37);
  cloud.addRemovef_rest_38(withf_rest_38);
  cloud.addRemovef_rest_39(withf_rest_39);
  cloud.addRemovef_rest_40(withf_rest_40);
  cloud.addRemovef_rest_41(withf_rest_41);
  cloud.addRemovef_rest_42(withf_rest_42);
  cloud.addRemovef_rest_43(withf_rest_43);
  cloud.addRemovef_rest_44(withf_rest_44);

  if (withFrameIndex)
    cloud.addFrameIndex();
  else
    cloud.removeFrameIndex();

  if (withLaserAngles)
    cloud.addLaserAngles();
  else
    cloud.removeLaserAngles();

  cloud.resize(pointCount);

  size_t total_point_num;
  if (isAscii) {
    size_t pointCounter = 0;
    while (!ifs.eof() && pointCounter < pointCount) {
      ifs.getline(tmp, MAX_BUFFER_SIZE);
      getTokens(tmp, sep, tokens);
      if (tokens.empty()) {
        continue;
      }
      if (tokens.size() < attributeCount) {
        return false;
      }
      auto& position = cloud[pointCounter];
      position[0] = atof(tokens[indexX].c_str()) * positionScale;
      position[1] = atof(tokens[indexY].c_str()) * positionScale;
      position[2] = atof(tokens[indexZ].c_str()) * positionScale;
      if (cloud.hasColors()) {
        auto& color = cloud.getColor(pointCounter);
        color[0] = atoi(tokens[indexG].c_str());
        color[1] = atoi(tokens[indexB].c_str());
        color[2] = atoi(tokens[indexR].c_str());
      }
      if (cloud.hasReflectances()) {
        cloud.getReflectance(pointCounter) =
          uint16_t(atoi(tokens[indexReflectance].c_str()));
      }
      if (cloud.hasOpacities()) {
        cloud.getOpacity(pointCounter) =
          uint16_t(atoi(tokens[indexopacity].c_str()));
        cloud.temp_opacities.push_back(atof(tokens[indexopacity].c_str()));
      }
      if (cloud.hasf_dc_0()) {
        cloud.getf_dc_0(pointCounter) =
          uint16_t(atoi(tokens[indexf_dc_0].c_str()));
        cloud.temp_f_dc_0.push_back(atof(tokens[indexf_dc_0].c_str()));
      }
      if (cloud.hasf_dc_1()) {
        cloud.getf_dc_1(pointCounter) =
          uint16_t(atoi(tokens[indexf_dc_1].c_str()));
        cloud.temp_f_dc_1.push_back(atof(tokens[indexf_dc_1].c_str()));
      }
      if (cloud.hasf_dc_2()) {
        cloud.getf_dc_2(pointCounter) =
          uint16_t(atoi(tokens[indexf_dc_2].c_str()));
        cloud.temp_f_dc_2.push_back(atof(tokens[indexf_dc_2].c_str()));
      }
      if (cloud.hasf_rest_0()) {
        cloud.getf_rest_0(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_0].c_str()));
        cloud.temp_f_rest_0.push_back(atof(tokens[indexf_rest_0].c_str()));
      }
      if (cloud.hasscale_0()) {
        cloud.getscale_0(pointCounter) =
          uint16_t(atoi(tokens[indexscale_0].c_str()));
        cloud.temp_scale_0.push_back(atof(tokens[indexscale_0].c_str()));
      }
      if (cloud.hasscale_1()) {
        cloud.getscale_1(pointCounter) =
          uint16_t(atoi(tokens[indexscale_1].c_str()));
        cloud.temp_scale_1.push_back(atof(tokens[indexscale_1].c_str()));
      }
      if (cloud.hasscale_2()) {
        cloud.getscale_2(pointCounter) =
          uint16_t(atoi(tokens[indexscale_2].c_str()));
        cloud.temp_scale_2.push_back(atof(tokens[indexscale_2].c_str()));
      }
      if (cloud.hasrot_0()) {
        cloud.getrot_0(pointCounter) =
          uint16_t(atoi(tokens[indexrot_0].c_str()));
        cloud.temp_rot_0.push_back(atof(tokens[indexrot_0].c_str()));
      }
      if (cloud.hasrot_1()) {
        cloud.getrot_1(pointCounter) =
          uint16_t(atoi(tokens[indexrot_1].c_str()));
        cloud.temp_rot_1.push_back(atof(tokens[indexrot_1].c_str()));
      }
      if (cloud.hasrot_2()) {
        cloud.getrot_2(pointCounter) =
          uint16_t(atoi(tokens[indexrot_2].c_str()));
        cloud.temp_rot_2.push_back(atof(tokens[indexrot_2].c_str()));
      }
      if (cloud.hasrot_3()) {
        cloud.getrot_3(pointCounter) =
          uint16_t(atoi(tokens[indexrot_3].c_str()));
        cloud.temp_rot_3.push_back(atof(tokens[indexrot_3].c_str()));
      }
      if (cloud.hasf_rest_1()) {
        cloud.getf_rest_1(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_1].c_str()));
        cloud.temp_f_rest_1.push_back(atof(tokens[indexf_rest_1].c_str()));
      }
      if (cloud.hasf_rest_2()) {
        cloud.getf_rest_2(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_2].c_str()));
        cloud.temp_f_rest_2.push_back(atof(tokens[indexf_rest_2].c_str()));
      }
      if (cloud.hasf_rest_3()) {
        cloud.getf_rest_3(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_3].c_str()));
        cloud.temp_f_rest_3.push_back(atof(tokens[indexf_rest_3].c_str()));
      }
      if (cloud.hasf_rest_4()) {
        cloud.getf_rest_4(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_4].c_str()));
        cloud.temp_f_rest_4.push_back(atof(tokens[indexf_rest_4].c_str()));
      }
      if (cloud.hasf_rest_5()) {
        cloud.getf_rest_5(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_5].c_str()));
        cloud.temp_f_rest_5.push_back(atof(tokens[indexf_rest_5].c_str()));
      }
      if (cloud.hasf_rest_6()) {
        cloud.getf_rest_6(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_6].c_str()));
        cloud.temp_f_rest_6.push_back(atof(tokens[indexf_rest_6].c_str()));
      }
      if (cloud.hasf_rest_7()) {
        cloud.getf_rest_7(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_7].c_str()));
        cloud.temp_f_rest_7.push_back(atof(tokens[indexf_rest_7].c_str()));
      }
      if (cloud.hasf_rest_8()) {
        cloud.getf_rest_8(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_8].c_str()));
        cloud.temp_f_rest_8.push_back(atof(tokens[indexf_rest_8].c_str()));
      }
      if (cloud.hasf_rest_9()) {
        cloud.getf_rest_9(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_9].c_str()));
        cloud.temp_f_rest_9.push_back(atof(tokens[indexf_rest_9].c_str()));
      }
      if (cloud.hasf_rest_10()) {
        cloud.getf_rest_10(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_10].c_str()));
        cloud.temp_f_rest_10.push_back(atof(tokens[indexf_rest_10].c_str()));
      }
      if (cloud.hasf_rest_11()) {
        cloud.getf_rest_11(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_11].c_str()));
        cloud.temp_f_rest_11.push_back(atof(tokens[indexf_rest_11].c_str()));
      }
      if (cloud.hasf_rest_12()) {
        cloud.getf_rest_12(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_12].c_str()));
        cloud.temp_f_rest_12.push_back(atof(tokens[indexf_rest_12].c_str()));
      }
      if (cloud.hasf_rest_13()) {
        cloud.getf_rest_13(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_13].c_str()));
        cloud.temp_f_rest_13.push_back(atof(tokens[indexf_rest_13].c_str()));
      }
      if (cloud.hasf_rest_14()) {
        cloud.getf_rest_14(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_14].c_str()));
        cloud.temp_f_rest_14.push_back(atof(tokens[indexf_rest_14].c_str()));
      }
      if (cloud.hasf_rest_15()) {
        cloud.getf_rest_15(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_15].c_str()));
        cloud.temp_f_rest_15.push_back(atof(tokens[indexf_rest_15].c_str()));
      }
      if (cloud.hasf_rest_16()) {
        cloud.getf_rest_16(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_16].c_str()));
        cloud.temp_f_rest_16.push_back(atof(tokens[indexf_rest_16].c_str()));
      }
      if (cloud.hasf_rest_17()) {
        cloud.getf_rest_17(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_17].c_str()));
        cloud.temp_f_rest_17.push_back(atof(tokens[indexf_rest_17].c_str()));
      }
      if (cloud.hasf_rest_18()) {
        cloud.getf_rest_18(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_18].c_str()));
        cloud.temp_f_rest_18.push_back(atof(tokens[indexf_rest_18].c_str()));
      }
      if (cloud.hasf_rest_19()) {
        cloud.getf_rest_19(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_19].c_str()));
        cloud.temp_f_rest_19.push_back(atof(tokens[indexf_rest_19].c_str()));
      }
      if (cloud.hasf_rest_20()) {
        cloud.getf_rest_20(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_20].c_str()));
        cloud.temp_f_rest_20.push_back(atof(tokens[indexf_rest_20].c_str()));
      }
      if (cloud.hasf_rest_21()) {
        cloud.getf_rest_21(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_21].c_str()));
        cloud.temp_f_rest_21.push_back(atof(tokens[indexf_rest_21].c_str()));
      }
      if (cloud.hasf_rest_22()) {
        cloud.getf_rest_22(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_22].c_str()));
        cloud.temp_f_rest_22.push_back(atof(tokens[indexf_rest_22].c_str()));
      }
      if (cloud.hasf_rest_23()) {
        cloud.getf_rest_23(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_23].c_str()));
        cloud.temp_f_rest_23.push_back(atof(tokens[indexf_rest_23].c_str()));
      }
      if (cloud.hasf_rest_24()) {
        cloud.getf_rest_24(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_24].c_str()));
        cloud.temp_f_rest_24.push_back(atof(tokens[indexf_rest_24].c_str()));
      }
      if (cloud.hasf_rest_25()) {
        cloud.getf_rest_25(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_25].c_str()));
        cloud.temp_f_rest_25.push_back(atof(tokens[indexf_rest_25].c_str()));
      }
      if (cloud.hasf_rest_26()) {
        cloud.getf_rest_26(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_26].c_str()));
        cloud.temp_f_rest_26.push_back(atof(tokens[indexf_rest_26].c_str()));
      }
      if (cloud.hasf_rest_27()) {
        cloud.getf_rest_27(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_27].c_str()));
        cloud.temp_f_rest_27.push_back(atof(tokens[indexf_rest_27].c_str()));
      }
      if (cloud.hasf_rest_28()) {
        cloud.getf_rest_28(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_28].c_str()));
        cloud.temp_f_rest_28.push_back(atof(tokens[indexf_rest_28].c_str()));
      }
      if (cloud.hasf_rest_29()) {
        cloud.getf_rest_29(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_29].c_str()));
        cloud.temp_f_rest_29.push_back(atof(tokens[indexf_rest_29].c_str()));
      }
      if (cloud.hasf_rest_30()) {
        cloud.getf_rest_30(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_30].c_str()));
        cloud.temp_f_rest_30.push_back(atof(tokens[indexf_rest_30].c_str()));
      }
      if (cloud.hasf_rest_31()) {
        cloud.getf_rest_31(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_31].c_str()));
        cloud.temp_f_rest_31.push_back(atof(tokens[indexf_rest_31].c_str()));
      }
      if (cloud.hasf_rest_32()) {
        cloud.getf_rest_32(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_32].c_str()));
        cloud.temp_f_rest_32.push_back(atof(tokens[indexf_rest_32].c_str()));
      }
      if (cloud.hasf_rest_33()) {
        cloud.getf_rest_33(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_33].c_str()));
        cloud.temp_f_rest_33.push_back(atof(tokens[indexf_rest_33].c_str()));
      }
      if (cloud.hasf_rest_34()) {
        cloud.getf_rest_34(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_34].c_str()));
        cloud.temp_f_rest_34.push_back(atof(tokens[indexf_rest_34].c_str()));
      }
      if (cloud.hasf_rest_35()) {
        cloud.getf_rest_35(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_35].c_str()));
        cloud.temp_f_rest_35.push_back(atof(tokens[indexf_rest_35].c_str()));
      }
      if (cloud.hasf_rest_36()) {
        cloud.getf_rest_36(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_36].c_str()));
        cloud.temp_f_rest_36.push_back(atof(tokens[indexf_rest_36].c_str()));
      }
      if (cloud.hasf_rest_37()) {
        cloud.getf_rest_37(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_37].c_str()));
        cloud.temp_f_rest_37.push_back(atof(tokens[indexf_rest_37].c_str()));
      }
      if (cloud.hasf_rest_38()) {
        cloud.getf_rest_38(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_38].c_str()));
        cloud.temp_f_rest_38.push_back(atof(tokens[indexf_rest_38].c_str()));
      }
      if (cloud.hasf_rest_39()) {
        cloud.getf_rest_39(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_39].c_str()));
        cloud.temp_f_rest_39.push_back(atof(tokens[indexf_rest_39].c_str()));
      }
      if (cloud.hasf_rest_40()) {
        cloud.getf_rest_40(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_40].c_str()));
        cloud.temp_f_rest_40.push_back(atof(tokens[indexf_rest_40].c_str()));
      }
      if (cloud.hasf_rest_41()) {
        cloud.getf_rest_41(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_41].c_str()));
        cloud.temp_f_rest_41.push_back(atof(tokens[indexf_rest_41].c_str()));
      }
      if (cloud.hasf_rest_42()) {
        cloud.getf_rest_42(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_42].c_str()));
        cloud.temp_f_rest_42.push_back(atof(tokens[indexf_rest_42].c_str()));
      }
      if (cloud.hasf_rest_43()) {
        cloud.getf_rest_43(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_43].c_str()));
        cloud.temp_f_rest_43.push_back(atof(tokens[indexf_rest_43].c_str()));
      }
      if (cloud.hasf_rest_44()) {
        cloud.getf_rest_44(pointCounter) =
          uint16_t(atoi(tokens[indexf_rest_44].c_str()));
        cloud.temp_f_rest_44.push_back(atof(tokens[indexf_rest_44].c_str()));
      }

      if (cloud.hasFrameIndex()) {
        cloud.getFrameIndex(pointCounter) =
          uint8_t(atoi(tokens[indexFrame].c_str()));
      }
      if (cloud.hasLaserAngles()) {
        cloud.getLaserAngle(pointCounter) =
          std::round(atof(tokens[indexLaserAngle].c_str()));
      }
      ++pointCounter;
      total_point_num = pointCounter;
    }

  } else {  // read binary start
    for (size_t pointCounter = 0; pointCounter < pointCount && !ifs.eof();
         ++pointCounter) {
      auto& position = cloud[pointCounter];
      for (size_t a = 0; a < attributeCount && !ifs.eof(); ++a) {
        const auto& attributeInfo = attributesInfo[a];
        if (a == indexX) {
          if (attributeInfo.byteCount == 4) {
            float x;
            ifs.read(reinterpret_cast<char*>(&x), sizeof(float));
            position[0] = x * positionScale;
          } else {
            double x;
            ifs.read(reinterpret_cast<char*>(&x), sizeof(double));
            position[0] = x * positionScale;
          }
        } else if (a == indexY) {
          if (attributeInfo.byteCount == 4) {
            float y;
            ifs.read(reinterpret_cast<char*>(&y), sizeof(float));
            position[1] = y * positionScale;
          } else {
            double y;
            ifs.read(reinterpret_cast<char*>(&y), sizeof(double));
            position[1] = y * positionScale;
          }
        } else if (a == indexZ) {
          if (attributeInfo.byteCount == 4) {
            float z;
            ifs.read(reinterpret_cast<char*>(&z), sizeof(float));
            position[2] = z * positionScale;
          } else {
            double z;
            ifs.read(reinterpret_cast<char*>(&z), sizeof(double));
            position[2] = z * positionScale;
          }
        } else if (a == indexR && attributeInfo.byteCount == 1) {
          uint8_t val8b;
          ifs.read(reinterpret_cast<char*>(&val8b), sizeof(uint8_t));
          cloud.getColor(pointCounter)[2] = val8b;
        } else if (a == indexG && attributeInfo.byteCount == 1) {
          uint8_t val8b;
          ifs.read(reinterpret_cast<char*>(&val8b), sizeof(uint8_t));
          cloud.getColor(pointCounter)[0] = val8b;
        } else if (a == indexB && attributeInfo.byteCount == 1) {
          uint8_t val8b;
          ifs.read(reinterpret_cast<char*>(&val8b), sizeof(uint8_t));
          cloud.getColor(pointCounter)[1] = val8b;
        } else if (a == indexReflectance && attributeInfo.byteCount <= 2) {
          if (attributeInfo.byteCount == 1) {
            uint8_t reflectance;
            ifs.read(reinterpret_cast<char*>(&reflectance), sizeof(uint8_t));
            cloud.getReflectance(pointCounter) = reflectance;
          } else {
            auto& reflectance = cloud.getReflectance(pointCounter);
            ifs.read(reinterpret_cast<char*>(&reflectance), sizeof(uint16_t));
          }
        } else if (a == indexopacity) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_opacities.push_back(read_value);
        } else if (a == indexf_dc_0) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_dc_0.push_back(read_value);
        } else if (a == indexf_dc_1) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_dc_1.push_back(read_value);
        } else if (a == indexf_dc_2) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_dc_2.push_back(read_value);
        } else if (a == indexf_rest_0) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_0.push_back(read_value);
        } else if (a == indexscale_0) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_scale_0.push_back(read_value);
        } else if (a == indexscale_1) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_scale_1.push_back(read_value);
        } else if (a == indexscale_2) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_scale_2.push_back(read_value);
        } else if (a == indexrot_0) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_rot_0.push_back(read_value);
        } else if (a == indexrot_1) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_rot_1.push_back(read_value);
        } else if (a == indexrot_2) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_rot_2.push_back(read_value);
        } else if (a == indexrot_3) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_rot_3.push_back(read_value);
        } else if (a == indexf_rest_1) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_1.push_back(read_value);
        } else if (a == indexf_rest_2) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_2.push_back(read_value);
        } else if (a == indexf_rest_3) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_3.push_back(read_value);
        } else if (a == indexf_rest_4) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_4.push_back(read_value);
        } else if (a == indexf_rest_5) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_5.push_back(read_value);
        } else if (a == indexf_rest_6) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_6.push_back(read_value);
        } else if (a == indexf_rest_7) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_7.push_back(read_value);
        } else if (a == indexf_rest_8) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_8.push_back(read_value);
        } else if (a == indexf_rest_9) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_9.push_back(read_value);
        } else if (a == indexf_rest_10) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_10.push_back(read_value);
        } else if (a == indexf_rest_11) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_11.push_back(read_value);
        } else if (a == indexf_rest_12) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_12.push_back(read_value);
        } else if (a == indexf_rest_13) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_13.push_back(read_value);
        } else if (a == indexf_rest_14) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_14.push_back(read_value);
        } else if (a == indexf_rest_15) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_15.push_back(read_value);
        } else if (a == indexf_rest_16) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_16.push_back(read_value);
        } else if (a == indexf_rest_17) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_17.push_back(read_value);
        } else if (a == indexf_rest_18) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_18.push_back(read_value);
        } else if (a == indexf_rest_19) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_19.push_back(read_value);
        } else if (a == indexf_rest_20) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_20.push_back(read_value);
        } else if (a == indexf_rest_21) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_21.push_back(read_value);
        } else if (a == indexf_rest_22) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_22.push_back(read_value);
        } else if (a == indexf_rest_23) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_23.push_back(read_value);
        } else if (a == indexf_rest_24) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_24.push_back(read_value);
        } else if (a == indexf_rest_25) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_25.push_back(read_value);
        } else if (a == indexf_rest_26) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_26.push_back(read_value);
        } else if (a == indexf_rest_27) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_27.push_back(read_value);
        } else if (a == indexf_rest_28) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_28.push_back(read_value);
        } else if (a == indexf_rest_29) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_29.push_back(read_value);
        } else if (a == indexf_rest_30) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_30.push_back(read_value);
        } else if (a == indexf_rest_31) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_31.push_back(read_value);
        } else if (a == indexf_rest_32) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_32.push_back(read_value);
        } else if (a == indexf_rest_33) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_33.push_back(read_value);
        } else if (a == indexf_rest_34) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_34.push_back(read_value);
        } else if (a == indexf_rest_35) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_35.push_back(read_value);
        } else if (a == indexf_rest_36) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_36.push_back(read_value);
        } else if (a == indexf_rest_37) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_37.push_back(read_value);
        } else if (a == indexf_rest_38) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_38.push_back(read_value);
        } else if (a == indexf_rest_39) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_39.push_back(read_value);
        } else if (a == indexf_rest_40) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_40.push_back(read_value);
        } else if (a == indexf_rest_41) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_41.push_back(read_value);
        } else if (a == indexf_rest_42) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_42.push_back(read_value);
        } else if (a == indexf_rest_43) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_43.push_back(read_value);
        } else if (a == indexf_rest_44) {
          float read_value;
          ifs.read(reinterpret_cast<char*>(&read_value), sizeof(float));
          cloud.temp_f_rest_44.push_back(read_value);
        } else if (a == indexFrame && attributeInfo.byteCount <= 2) {
          if (attributeInfo.byteCount == 1) {
            auto& findex = cloud.getFrameIndex(pointCounter);
            ifs.read(reinterpret_cast<char*>(&findex), sizeof(uint8_t));
          } else {
            uint16_t findex;
            ifs.read(reinterpret_cast<char*>(&findex), sizeof(uint16_t));
            cloud.getFrameIndex(pointCounter) = uint8_t(findex);
          }
        } else {
          char buffer[128];
          ifs.read(buffer, attributeInfo.byteCount);
        }
      }
      total_point_num = pointCounter;
    }
  }

  // ------------------------------------------------------------------------
  // float ichun_scale = 60000.0;

  if (cloud.hasf_dc_0()) {
    cloud.temp_f_dc_0 =
      processVector(cloud.temp_f_dc_0, "f_dc_0", f_dc_0_scale);
  }
  if (cloud.hasf_dc_1()) {
    cloud.temp_f_dc_1 =
      processVector(cloud.temp_f_dc_1, "f_dc_1", f_dc_1_scale);
  }
  if (cloud.hasf_dc_2()) {
    cloud.temp_f_dc_2 =
      processVector(cloud.temp_f_dc_2, "f_dc_2", f_dc_2_scale);
  }
  if (cloud.hasf_rest_0()) {
    cloud.temp_f_rest_0 =
      processVector(cloud.temp_f_rest_0, "f_rest_0", f_rest_0_scale);
  }
  if (cloud.hasf_rest_1()) {
    cloud.temp_f_rest_1 =
      processVector(cloud.temp_f_rest_1, "f_rest_1", f_rest_1_scale);
  }
  if (cloud.hasf_rest_2()) {
    cloud.temp_f_rest_2 =
      processVector(cloud.temp_f_rest_2, "f_rest_2", f_rest_2_scale);
  }
  if (cloud.hasf_rest_3()) {
    cloud.temp_f_rest_3 =
      processVector(cloud.temp_f_rest_3, "f_rest_3", f_rest_3_scale);
  }
  if (cloud.hasf_rest_4()) {
    cloud.temp_f_rest_4 =
      processVector(cloud.temp_f_rest_4, "f_rest_4", f_rest_4_scale);
  }
  if (cloud.hasf_rest_5()) {
    cloud.temp_f_rest_5 =
      processVector(cloud.temp_f_rest_5, "f_rest_5", f_rest_5_scale);
  }
  if (cloud.hasf_rest_6()) {
    cloud.temp_f_rest_6 =
      processVector(cloud.temp_f_rest_6, "f_rest_6", f_rest_6_scale);
  }
  if (cloud.hasf_rest_7()) {
    cloud.temp_f_rest_7 =
      processVector(cloud.temp_f_rest_7, "f_rest_7", f_rest_7_scale);
  }
  if (cloud.hasf_rest_8()) {
    cloud.temp_f_rest_8 =
      processVector(cloud.temp_f_rest_8, "f_rest_8", f_rest_8_scale);
  }
  if (cloud.hasf_rest_9()) {
    cloud.temp_f_rest_9 =
      processVector(cloud.temp_f_rest_9, "f_rest_9", f_rest_9_scale);
  }
  if (cloud.hasf_rest_10()) {
    cloud.temp_f_rest_10 =
      processVector(cloud.temp_f_rest_10, "f_rest_10", f_rest_10_scale);
  }
  if (cloud.hasf_rest_11()) {
    cloud.temp_f_rest_11 =
      processVector(cloud.temp_f_rest_11, "f_rest_11", f_rest_11_scale);
  }
  if (cloud.hasf_rest_12()) {
    cloud.temp_f_rest_12 =
      processVector(cloud.temp_f_rest_12, "f_rest_12", f_rest_12_scale);
  }
  if (cloud.hasf_rest_13()) {
    cloud.temp_f_rest_13 =
      processVector(cloud.temp_f_rest_13, "f_rest_13", f_rest_13_scale);
  }
  if (cloud.hasf_rest_14()) {
    cloud.temp_f_rest_14 =
      processVector(cloud.temp_f_rest_14, "f_rest_14", f_rest_14_scale);
  }
  if (cloud.hasf_rest_15()) {
    cloud.temp_f_rest_15 =
      processVector(cloud.temp_f_rest_15, "f_rest_15", f_rest_15_scale);
  }
  if (cloud.hasf_rest_16()) {
    cloud.temp_f_rest_16 =
      processVector(cloud.temp_f_rest_16, "f_rest_16", f_rest_16_scale);
  }
  if (cloud.hasf_rest_17()) {
    cloud.temp_f_rest_17 =
      processVector(cloud.temp_f_rest_17, "f_rest_17", f_rest_17_scale);
  }
  if (cloud.hasf_rest_18()) {
    cloud.temp_f_rest_18 =
      processVector(cloud.temp_f_rest_18, "f_rest_18", f_rest_18_scale);
  }
  if (cloud.hasf_rest_19()) {
    cloud.temp_f_rest_19 =
      processVector(cloud.temp_f_rest_19, "f_rest_19", f_rest_19_scale);
  }
  if (cloud.hasf_rest_20()) {
    cloud.temp_f_rest_20 =
      processVector(cloud.temp_f_rest_20, "f_rest_20", f_rest_20_scale);
  }
  if (cloud.hasf_rest_21()) {
    cloud.temp_f_rest_21 =
      processVector(cloud.temp_f_rest_21, "f_rest_21", f_rest_21_scale);
  }
  if (cloud.hasf_rest_22()) {
    cloud.temp_f_rest_22 =
      processVector(cloud.temp_f_rest_22, "f_rest_22", f_rest_22_scale);
  }
  if (cloud.hasf_rest_23()) {
    cloud.temp_f_rest_23 =
      processVector(cloud.temp_f_rest_23, "f_rest_23", f_rest_23_scale);
  }
  if (cloud.hasf_rest_24()) {
    cloud.temp_f_rest_24 =
      processVector(cloud.temp_f_rest_24, "f_rest_24", f_rest_24_scale);
  }
  if (cloud.hasf_rest_25()) {
    cloud.temp_f_rest_25 =
      processVector(cloud.temp_f_rest_25, "f_rest_25", f_rest_25_scale);
  }
  if (cloud.hasf_rest_26()) {
    cloud.temp_f_rest_26 =
      processVector(cloud.temp_f_rest_26, "f_rest_26", f_rest_26_scale);
  }
  if (cloud.hasf_rest_27()) {
    cloud.temp_f_rest_27 =
      processVector(cloud.temp_f_rest_27, "f_rest_27", f_rest_27_scale);
  }
  if (cloud.hasf_rest_28()) {
    cloud.temp_f_rest_28 =
      processVector(cloud.temp_f_rest_28, "f_rest_28", f_rest_28_scale);
  }
  if (cloud.hasf_rest_29()) {
    cloud.temp_f_rest_29 =
      processVector(cloud.temp_f_rest_29, "f_rest_29", f_rest_29_scale);
  }
  if (cloud.hasf_rest_30()) {
    cloud.temp_f_rest_30 =
      processVector(cloud.temp_f_rest_30, "f_rest_30", f_rest_30_scale);
  }
  if (cloud.hasf_rest_31()) {
    cloud.temp_f_rest_31 =
      processVector(cloud.temp_f_rest_31, "f_rest_31", f_rest_31_scale);
  }
  if (cloud.hasf_rest_32()) {
    cloud.temp_f_rest_32 =
      processVector(cloud.temp_f_rest_32, "f_rest_32", f_rest_32_scale);
  }
  if (cloud.hasf_rest_33()) {
    cloud.temp_f_rest_33 =
      processVector(cloud.temp_f_rest_33, "f_rest_33", f_rest_33_scale);
  }
  if (cloud.hasf_rest_34()) {
    cloud.temp_f_rest_34 =
      processVector(cloud.temp_f_rest_34, "f_rest_34", f_rest_34_scale);
  }
  if (cloud.hasf_rest_35()) {
    cloud.temp_f_rest_35 =
      processVector(cloud.temp_f_rest_35, "f_rest_35", f_rest_35_scale);
  }
  if (cloud.hasf_rest_36()) {
    cloud.temp_f_rest_36 =
      processVector(cloud.temp_f_rest_36, "f_rest_36", f_rest_36_scale);
  }
  if (cloud.hasf_rest_37()) {
    cloud.temp_f_rest_37 =
      processVector(cloud.temp_f_rest_37, "f_rest_37", f_rest_37_scale);
  }
  if (cloud.hasf_rest_38()) {
    cloud.temp_f_rest_38 =
      processVector(cloud.temp_f_rest_38, "f_rest_38", f_rest_38_scale);
  }
  if (cloud.hasf_rest_39()) {
    cloud.temp_f_rest_39 =
      processVector(cloud.temp_f_rest_39, "f_rest_39", f_rest_39_scale);
  }
  if (cloud.hasf_rest_40()) {
    cloud.temp_f_rest_40 =
      processVector(cloud.temp_f_rest_40, "f_rest_40", f_rest_40_scale);
  }
  if (cloud.hasf_rest_41()) {
    cloud.temp_f_rest_41 =
      processVector(cloud.temp_f_rest_41, "f_rest_41", f_rest_41_scale);
  }
  if (cloud.hasf_rest_42()) {
    cloud.temp_f_rest_42 =
      processVector(cloud.temp_f_rest_42, "f_rest_42", f_rest_42_scale);
  }
  if (cloud.hasf_rest_43()) {
    cloud.temp_f_rest_43 =
      processVector(cloud.temp_f_rest_43, "f_rest_43", f_rest_43_scale);
  }
  if (cloud.hasf_rest_44()) {
    cloud.temp_f_rest_44 =
      processVector(cloud.temp_f_rest_44, "f_rest_44", f_rest_44_scale);
  }
  if (cloud.hasOpacities()) {
    cloud.temp_opacities =
      processVector(cloud.temp_opacities, "opacity", opacity_scale);
  }
  if (cloud.hasscale_0()) {
    cloud.temp_scale_0 =
      processVector(cloud.temp_scale_0, "scale_0", scale_0_scale);
  }
  if (cloud.hasscale_1()) {
    cloud.temp_scale_1 =
      processVector(cloud.temp_scale_1, "scale_1", scale_1_scale);
  }
  if (cloud.hasscale_2()) {
    cloud.temp_scale_2 =
      processVector(cloud.temp_scale_2, "scale_2", scale_2_scale);
  }
  if (cloud.hasrot_0()) {
    cloud.temp_rot_0 = processVector(cloud.temp_rot_0, "rot_0", rot_0_scale);
  }
  if (cloud.hasrot_1()) {
    cloud.temp_rot_1 = processVector(cloud.temp_rot_1, "rot_1", rot_1_scale);
  }
  if (cloud.hasrot_2()) {
    cloud.temp_rot_2 = processVector(cloud.temp_rot_2, "rot_2", rot_2_scale);
  }
  if (cloud.hasrot_3()) {
    cloud.temp_rot_3 = processVector(cloud.temp_rot_3, "rot_3", rot_3_scale);
  }

  for (size_t ichun_i = 0; ichun_i < total_point_num; ichun_i++) {
    if (cloud.hasOpacities())
      cloud.getOpacity(ichun_i) = uint16_t(cloud.temp_opacities[ichun_i]);
    if (cloud.hasf_dc_0())
      cloud.getf_dc_0(ichun_i) = uint16_t(cloud.temp_f_dc_0[ichun_i]);
    if (cloud.hasf_dc_1())
      cloud.getf_dc_1(ichun_i) = uint16_t(cloud.temp_f_dc_1[ichun_i]);
    if (cloud.hasf_dc_2())
      cloud.getf_dc_2(ichun_i) = uint16_t(cloud.temp_f_dc_2[ichun_i]);
    if (cloud.hasf_rest_0())
      cloud.getf_rest_0(ichun_i) = uint16_t(cloud.temp_f_rest_0[ichun_i]);
    if (cloud.hasscale_0())
      cloud.getscale_0(ichun_i) = uint16_t(cloud.temp_scale_0[ichun_i]);
    if (cloud.hasscale_1())
      cloud.getscale_1(ichun_i) = uint16_t(cloud.temp_scale_1[ichun_i]);
    if (cloud.hasscale_2())
      cloud.getscale_2(ichun_i) = uint16_t(cloud.temp_scale_2[ichun_i]);
    if (cloud.hasrot_0())
      cloud.getrot_0(ichun_i) = uint16_t(cloud.temp_rot_0[ichun_i]);
    if (cloud.hasrot_1())
      cloud.getrot_1(ichun_i) = uint16_t(cloud.temp_rot_1[ichun_i]);
    if (cloud.hasrot_2())
      cloud.getrot_2(ichun_i) = uint16_t(cloud.temp_rot_2[ichun_i]);
    if (cloud.hasrot_3())
      cloud.getrot_3(ichun_i) = uint16_t(cloud.temp_rot_3[ichun_i]);
    if (cloud.hasf_rest_1())
      cloud.getf_rest_1(ichun_i) = uint16_t(cloud.temp_f_rest_1[ichun_i]);
    if (cloud.hasf_rest_2())
      cloud.getf_rest_2(ichun_i) = uint16_t(cloud.temp_f_rest_2[ichun_i]);
    if (cloud.hasf_rest_3())
      cloud.getf_rest_3(ichun_i) = uint16_t(cloud.temp_f_rest_3[ichun_i]);
    if (cloud.hasf_rest_4())
      cloud.getf_rest_4(ichun_i) = uint16_t(cloud.temp_f_rest_4[ichun_i]);
    if (cloud.hasf_rest_5())
      cloud.getf_rest_5(ichun_i) = uint16_t(cloud.temp_f_rest_5[ichun_i]);
    if (cloud.hasf_rest_6())
      cloud.getf_rest_6(ichun_i) = uint16_t(cloud.temp_f_rest_6[ichun_i]);
    if (cloud.hasf_rest_7())
      cloud.getf_rest_7(ichun_i) = uint16_t(cloud.temp_f_rest_7[ichun_i]);
    if (cloud.hasf_rest_8())
      cloud.getf_rest_8(ichun_i) = uint16_t(cloud.temp_f_rest_8[ichun_i]);
    if (cloud.hasf_rest_9())
      cloud.getf_rest_9(ichun_i) = uint16_t(cloud.temp_f_rest_9[ichun_i]);
    if (cloud.hasf_rest_10())
      cloud.getf_rest_10(ichun_i) = uint16_t(cloud.temp_f_rest_10[ichun_i]);
    if (cloud.hasf_rest_11())
      cloud.getf_rest_11(ichun_i) = uint16_t(cloud.temp_f_rest_11[ichun_i]);
    if (cloud.hasf_rest_12())
      cloud.getf_rest_12(ichun_i) = uint16_t(cloud.temp_f_rest_12[ichun_i]);
    if (cloud.hasf_rest_13())
      cloud.getf_rest_13(ichun_i) = uint16_t(cloud.temp_f_rest_13[ichun_i]);
    if (cloud.hasf_rest_14())
      cloud.getf_rest_14(ichun_i) = uint16_t(cloud.temp_f_rest_14[ichun_i]);
    if (cloud.hasf_rest_15())
      cloud.getf_rest_15(ichun_i) = uint16_t(cloud.temp_f_rest_15[ichun_i]);
    if (cloud.hasf_rest_16())
      cloud.getf_rest_16(ichun_i) = uint16_t(cloud.temp_f_rest_16[ichun_i]);
    if (cloud.hasf_rest_17())
      cloud.getf_rest_17(ichun_i) = uint16_t(cloud.temp_f_rest_17[ichun_i]);
    if (cloud.hasf_rest_18())
      cloud.getf_rest_18(ichun_i) = uint16_t(cloud.temp_f_rest_18[ichun_i]);
    if (cloud.hasf_rest_19())
      cloud.getf_rest_19(ichun_i) = uint16_t(cloud.temp_f_rest_19[ichun_i]);
    if (cloud.hasf_rest_20())
      cloud.getf_rest_20(ichun_i) = uint16_t(cloud.temp_f_rest_20[ichun_i]);
    if (cloud.hasf_rest_21())
      cloud.getf_rest_21(ichun_i) = uint16_t(cloud.temp_f_rest_21[ichun_i]);
    if (cloud.hasf_rest_22())
      cloud.getf_rest_22(ichun_i) = uint16_t(cloud.temp_f_rest_22[ichun_i]);
    if (cloud.hasf_rest_23())
      cloud.getf_rest_23(ichun_i) = uint16_t(cloud.temp_f_rest_23[ichun_i]);
    if (cloud.hasf_rest_24())
      cloud.getf_rest_24(ichun_i) = uint16_t(cloud.temp_f_rest_24[ichun_i]);
    if (cloud.hasf_rest_25())
      cloud.getf_rest_25(ichun_i) = uint16_t(cloud.temp_f_rest_25[ichun_i]);
    if (cloud.hasf_rest_26())
      cloud.getf_rest_26(ichun_i) = uint16_t(cloud.temp_f_rest_26[ichun_i]);
    if (cloud.hasf_rest_27())
      cloud.getf_rest_27(ichun_i) = uint16_t(cloud.temp_f_rest_27[ichun_i]);
    if (cloud.hasf_rest_28())
      cloud.getf_rest_28(ichun_i) = uint16_t(cloud.temp_f_rest_28[ichun_i]);
    if (cloud.hasf_rest_29())
      cloud.getf_rest_29(ichun_i) = uint16_t(cloud.temp_f_rest_29[ichun_i]);
    if (cloud.hasf_rest_30())
      cloud.getf_rest_30(ichun_i) = uint16_t(cloud.temp_f_rest_30[ichun_i]);
    if (cloud.hasf_rest_31())
      cloud.getf_rest_31(ichun_i) = uint16_t(cloud.temp_f_rest_31[ichun_i]);
    if (cloud.hasf_rest_32())
      cloud.getf_rest_32(ichun_i) = uint16_t(cloud.temp_f_rest_32[ichun_i]);
    if (cloud.hasf_rest_33())
      cloud.getf_rest_33(ichun_i) = uint16_t(cloud.temp_f_rest_33[ichun_i]);
    if (cloud.hasf_rest_34())
      cloud.getf_rest_34(ichun_i) = uint16_t(cloud.temp_f_rest_34[ichun_i]);
    if (cloud.hasf_rest_35())
      cloud.getf_rest_35(ichun_i) = uint16_t(cloud.temp_f_rest_35[ichun_i]);
    if (cloud.hasf_rest_36())
      cloud.getf_rest_36(ichun_i) = uint16_t(cloud.temp_f_rest_36[ichun_i]);
    if (cloud.hasf_rest_37())
      cloud.getf_rest_37(ichun_i) = uint16_t(cloud.temp_f_rest_37[ichun_i]);
    if (cloud.hasf_rest_38())
      cloud.getf_rest_38(ichun_i) = uint16_t(cloud.temp_f_rest_38[ichun_i]);
    if (cloud.hasf_rest_39())
      cloud.getf_rest_39(ichun_i) = uint16_t(cloud.temp_f_rest_39[ichun_i]);
    if (cloud.hasf_rest_40())
      cloud.getf_rest_40(ichun_i) = uint16_t(cloud.temp_f_rest_40[ichun_i]);
    if (cloud.hasf_rest_41())
      cloud.getf_rest_41(ichun_i) = uint16_t(cloud.temp_f_rest_41[ichun_i]);
    if (cloud.hasf_rest_42())
      cloud.getf_rest_42(ichun_i) = uint16_t(cloud.temp_f_rest_42[ichun_i]);
    if (cloud.hasf_rest_43())
      cloud.getf_rest_43(ichun_i) = uint16_t(cloud.temp_f_rest_43[ichun_i]);
    if (cloud.hasf_rest_44())
      cloud.getf_rest_44(ichun_i) = uint16_t(cloud.temp_f_rest_44[ichun_i]);
  }
  return true;
}

//============================================================================

}  // namespace pcc
