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

#ifndef PCCPointSet_h
#define PCCPointSet_h

#include <assert.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

#include "PCCMath.h"
#include "PCCMisc.h"

namespace pcc {

//============================================================================
// The type used for internally representing attribute data
typedef uint16_t attr_t;

// The type used for internally representing positions
typedef Vec3<int32_t> point_t;

//============================================================================

class PCCPointSet3 {
public:
  typedef point_t PointType;

  //=========================================================================
  // proxy object for use with iterator, allowing handling of PCCPointSet3's
  // structure-of-arrays as a single array.

  class iterator;
  class Proxy {
    friend class iterator;

    PCCPointSet3* parent_;
    size_t idx_;

  public:
    //-----------------------------------------------------------------------

    Proxy() : parent_(nullptr), idx_() {}

    Proxy(PCCPointSet3* parent, size_t idx) : parent_(parent), idx_(idx) {}

    //-----------------------------------------------------------------------

    PointType operator*() const { return (*parent_)[idx_]; }

    PointType& operator*() { return (*parent_)[idx_]; }

    //-----------------------------------------------------------------------
    // Swap the position of the current proxied point (including attributes)
    // with that of @other in the same PointSet.

    void swap(const Proxy& other) const
    {
      assert(parent_ == other.parent_);
      parent_->swapPoints(idx_, other.idx_);
    }

    //-----------------------------------------------------------------------
  };

  //=========================================================================
  // iterator for use with stl algorithms

  class iterator {
  private:
    Proxy p_;

  public:
    typedef std::random_access_iterator_tag iterator_category;
    typedef const Proxy value_type;
    typedef std::ptrdiff_t difference_type;
    typedef const Proxy* pointer;
    typedef const Proxy& reference;

    //-----------------------------------------------------------------------

    iterator() = default;
    iterator(const iterator&) = default;

    //-----------------------------------------------------------------------

    explicit iterator(PCCPointSet3* parent) : p_{parent, 0} {}

    explicit iterator(PCCPointSet3* parent, size_t idx) : p_{parent, idx} {}

    //-----------------------------------------------------------------------
    // :: Iterator

    reference operator*() const { return p_; }

    //-----------------------------------------------------------------------

    iterator& operator++()
    {
      p_.idx_++;
      return *this;
    }

    //-----------------------------------------------------------------------
    // :: ForwardIterator

    iterator operator++(int)
    {
      iterator retval = *this;
      ++(*this);
      return retval;
    }

    //-----------------------------------------------------------------------

    pointer operator->() const { return &p_; }

    //-----------------------------------------------------------------------

    bool operator==(const iterator& other) const
    {
      return p_.idx_ == other.p_.idx_;
    }

    //-----------------------------------------------------------------------

    bool operator!=(const iterator& other) const { return !(*this == other); }

    //-----------------------------------------------------------------------
    // :: BidirectionalIterator

    iterator& operator--()
    {
      p_.idx_--;
      return *this;
    }

    //-----------------------------------------------------------------------

    iterator operator--(int)
    {
      iterator retval = *this;
      --(*this);
      return retval;
    }

    //-----------------------------------------------------------------------
    // :: RandomAccessIterator

    value_type operator[](difference_type n)
    {
      return Proxy{p_.parent_, p_.idx_ + n};
    }

    //-----------------------------------------------------------------------

    iterator& operator+=(difference_type n)
    {
      p_.idx_ += n;
      return *this;
    }

    //-----------------------------------------------------------------------

    iterator operator+(difference_type n) const
    {
      iterator it(*this);
      it += n;
      return it;
    }

    //-----------------------------------------------------------------------

    iterator& operator-=(difference_type n)
    {
      p_.idx_ -= n;
      return *this;
    }

    //-----------------------------------------------------------------------

    iterator operator-(difference_type n) const
    {
      iterator it(*this);
      it -= n;
      return it;
    }

    //-----------------------------------------------------------------------

    difference_type operator-(const iterator& other) const
    {
      return p_.idx_ - other.p_.idx_;
    }

    //-----------------------------------------------------------------------
  };

  //=========================================================================

  PCCPointSet3()
  {
    withColors = false;
    withReflectances = false;
    withOpacity = false;
    withf_dc_0 = false;
    withf_dc_1 = false;
    withf_dc_2 = false;
    withscale_0 = false;
    withscale_1 = false;
    withscale_2 = false;
    withrot_0 = false;
    withrot_1 = false;
    withrot_2 = false;
    withrot_3 = false;
    withf_rest_0 = false;
    withf_rest_1 = false;
    withf_rest_2 = false;
    withf_rest_3 = false;
    withf_rest_4 = false;
    withf_rest_5 = false;
    withf_rest_6 = false;
    withf_rest_7 = false;
    withf_rest_8 = false;
    withf_rest_9 = false;
    withf_rest_10 = false;
    withf_rest_11 = false;
    withf_rest_12 = false;
    withf_rest_13 = false;
    withf_rest_14 = false;
    withf_rest_15 = false;
    withf_rest_16 = false;
    withf_rest_17 = false;
    withf_rest_18 = false;
    withf_rest_19 = false;
    withf_rest_20 = false;
    withf_rest_21 = false;
    withf_rest_22 = false;
    withf_rest_23 = false;
    withf_rest_24 = false;
    withf_rest_25 = false;
    withf_rest_26 = false;
    withf_rest_27 = false;
    withf_rest_28 = false;
    withf_rest_29 = false;
    withf_rest_30 = false;
    withf_rest_31 = false;
    withf_rest_32 = false;
    withf_rest_33 = false;
    withf_rest_34 = false;
    withf_rest_35 = false;
    withf_rest_36 = false;
    withf_rest_37 = false;
    withf_rest_38 = false;
    withf_rest_39 = false;
    withf_rest_40 = false;
    withf_rest_41 = false;
    withf_rest_42 = false;
    withf_rest_43 = false;
    withf_rest_44 = false;
    withFrameIndex = false;
    withLaserAngles = false;
  }
  PCCPointSet3(const PCCPointSet3&) = default;
  PCCPointSet3& operator=(const PCCPointSet3& rhs) = default;
  ~PCCPointSet3() = default;

  void swap(PCCPointSet3& other)
  {
    using std::swap;
    swap(positions, other.positions);
    swap(colors, other.colors);
    swap(reflectances, other.reflectances);
    swap(opacities, other.opacities);
    swap(f_dc_0, other.f_dc_0);
    swap(f_dc_1, other.f_dc_1);
    swap(f_dc_2, other.f_dc_2);
    swap(scale_0, other.scale_0);
    swap(scale_1, other.scale_1);
    swap(scale_2, other.scale_2);
    swap(rot_0, other.rot_0);
    swap(rot_1, other.rot_1);
    swap(rot_2, other.rot_2);
    swap(rot_3, other.rot_3);
    swap(f_rest_0, other.f_rest_0);
    swap(f_rest_1, other.f_rest_1);
    swap(f_rest_2, other.f_rest_2);
    swap(f_rest_3, other.f_rest_3);
    swap(f_rest_4, other.f_rest_4);
    swap(f_rest_5, other.f_rest_5);
    swap(f_rest_6, other.f_rest_6);
    swap(f_rest_7, other.f_rest_7);
    swap(f_rest_8, other.f_rest_8);
    swap(f_rest_9, other.f_rest_9);
    swap(f_rest_10, other.f_rest_10);
    swap(f_rest_11, other.f_rest_11);
    swap(f_rest_12, other.f_rest_12);
    swap(f_rest_13, other.f_rest_13);
    swap(f_rest_14, other.f_rest_14);
    swap(f_rest_15, other.f_rest_15);
    swap(f_rest_16, other.f_rest_16);
    swap(f_rest_17, other.f_rest_17);
    swap(f_rest_18, other.f_rest_18);
    swap(f_rest_19, other.f_rest_19);
    swap(f_rest_20, other.f_rest_20);
    swap(f_rest_21, other.f_rest_21);
    swap(f_rest_22, other.f_rest_22);
    swap(f_rest_23, other.f_rest_23);
    swap(f_rest_24, other.f_rest_24);
    swap(f_rest_25, other.f_rest_25);
    swap(f_rest_26, other.f_rest_26);
    swap(f_rest_27, other.f_rest_27);
    swap(f_rest_28, other.f_rest_28);
    swap(f_rest_29, other.f_rest_29);
    swap(f_rest_30, other.f_rest_30);
    swap(f_rest_31, other.f_rest_31);
    swap(f_rest_32, other.f_rest_32);
    swap(f_rest_33, other.f_rest_33);
    swap(f_rest_34, other.f_rest_34);
    swap(f_rest_35, other.f_rest_35);
    swap(f_rest_36, other.f_rest_36);
    swap(f_rest_37, other.f_rest_37);
    swap(f_rest_38, other.f_rest_38);
    swap(f_rest_39, other.f_rest_39);
    swap(f_rest_40, other.f_rest_40);
    swap(f_rest_41, other.f_rest_41);
    swap(f_rest_42, other.f_rest_42);
    swap(f_rest_43, other.f_rest_43);
    swap(f_rest_44, other.f_rest_44);
    swap(frameidx, other.frameidx);
    swap(laserAngles, other.laserAngles);

    swap(withColors, other.withColors);
    swap(withReflectances, other.withReflectances);
    swap(withOpacity, other.withOpacity);
    swap(withf_dc_0, other.withf_dc_0);
    swap(withf_dc_1, other.withf_dc_1);
    swap(withf_dc_2, other.withf_dc_2);
    swap(withscale_0, other.withscale_0);
    swap(withscale_1, other.withscale_1);
    swap(withscale_2, other.withscale_2);
    swap(withrot_0, other.withrot_0);
    swap(withrot_1, other.withrot_1);
    swap(withrot_2, other.withrot_2);
    swap(withrot_3, other.withrot_3);
    swap(withf_rest_0, other.withf_rest_0);
    swap(withf_rest_1, other.withf_rest_1);
    swap(withf_rest_2, other.withf_rest_2);
    swap(withf_rest_3, other.withf_rest_3);
    swap(withf_rest_4, other.withf_rest_4);
    swap(withf_rest_5, other.withf_rest_5);
    swap(withf_rest_6, other.withf_rest_6);
    swap(withf_rest_7, other.withf_rest_7);
    swap(withf_rest_8, other.withf_rest_8);
    swap(withf_rest_9, other.withf_rest_9);
    swap(withf_rest_10, other.withf_rest_10);
    swap(withf_rest_11, other.withf_rest_11);
    swap(withf_rest_12, other.withf_rest_12);
    swap(withf_rest_13, other.withf_rest_13);
    swap(withf_rest_14, other.withf_rest_14);
    swap(withf_rest_15, other.withf_rest_15);
    swap(withf_rest_16, other.withf_rest_16);
    swap(withf_rest_17, other.withf_rest_17);
    swap(withf_rest_18, other.withf_rest_18);
    swap(withf_rest_19, other.withf_rest_19);
    swap(withf_rest_20, other.withf_rest_20);
    swap(withf_rest_21, other.withf_rest_21);
    swap(withf_rest_22, other.withf_rest_22);
    swap(withf_rest_23, other.withf_rest_23);
    swap(withf_rest_24, other.withf_rest_24);
    swap(withf_rest_25, other.withf_rest_25);
    swap(withf_rest_26, other.withf_rest_26);
    swap(withf_rest_27, other.withf_rest_27);
    swap(withf_rest_28, other.withf_rest_28);
    swap(withf_rest_29, other.withf_rest_29);
    swap(withf_rest_30, other.withf_rest_30);
    swap(withf_rest_31, other.withf_rest_31);
    swap(withf_rest_32, other.withf_rest_32);
    swap(withf_rest_33, other.withf_rest_33);
    swap(withf_rest_34, other.withf_rest_34);
    swap(withf_rest_35, other.withf_rest_35);
    swap(withf_rest_36, other.withf_rest_36);
    swap(withf_rest_37, other.withf_rest_37);
    swap(withf_rest_38, other.withf_rest_38);
    swap(withf_rest_39, other.withf_rest_39);
    swap(withf_rest_40, other.withf_rest_40);
    swap(withf_rest_41, other.withf_rest_41);
    swap(withf_rest_42, other.withf_rest_42);
    swap(withf_rest_43, other.withf_rest_43);
    swap(withf_rest_44, other.withf_rest_44);
    swap(withFrameIndex, other.withFrameIndex);
    swap(withLaserAngles, other.withLaserAngles);
  }

  PointType operator[](const size_t index) const
  {
    assert(index < positions.size());
    return positions[index];
  }
  PointType& operator[](const size_t index)
  {
    assert(index < positions.size());
    return positions[index];
  }

  void swapPoints(std::vector<PointType>& other) { positions.swap(other); }

  Vec3<attr_t> getColor(const size_t index) const
  {
    assert(index < colors.size() && withColors);
    return colors[index];
  }
  Vec3<attr_t>& getColor(const size_t index)
  {
    assert(index < colors.size() && withColors);
    return colors[index];
  }
  void setColor(const size_t index, const Vec3<attr_t> color)
  {
    assert(index < colors.size() && withColors);
    colors[index] = color;
  }
  attr_t getReflectance(const size_t index) const
  {
    assert(index < reflectances.size() && withReflectances);
    return reflectances[index];
  }
  attr_t getOpacity(const size_t index) const
  {
    assert(index < opacities.size() && withOpacity);
    return opacities[index];
  }
  attr_t getf_dc_0(const size_t index) const
  {
    assert(index < f_dc_0.size() && withf_dc_0);
    return f_dc_0[index];
  }
  attr_t getf_dc_1(const size_t index) const
  {
    assert(index < f_dc_1.size() && withf_dc_1);
    return f_dc_1[index];
  }
  attr_t getf_dc_2(const size_t index) const
  {
    assert(index < f_dc_2.size() && withf_dc_2);
    return f_dc_2[index];
  }
  attr_t getscale_0(const size_t index) const
  {
    assert(index < scale_0.size() && withscale_0);
    return scale_0[index];
  }
  attr_t getscale_1(const size_t index) const
  {
    assert(index < scale_1.size() && withscale_1);
    return scale_1[index];
  }
  attr_t getscale_2(const size_t index) const
  {
    assert(index < scale_2.size() && withscale_2);
    return scale_2[index];
  }
  attr_t getrot_0(const size_t index) const
  {
    assert(index < rot_0.size() && withrot_0);
    return rot_0[index];
  }
  attr_t getrot_1(const size_t index) const
  {
    assert(index < rot_1.size() && withrot_1);
    return rot_1[index];
  }
  attr_t getrot_2(const size_t index) const
  {
    assert(index < rot_2.size() && withrot_2);
    return rot_2[index];
  }
  attr_t getrot_3(const size_t index) const
  {
    assert(index < rot_3.size() && withrot_3);
    return rot_3[index];
  }
  attr_t getf_rest_0(const size_t index) const
  {
    assert(index < f_rest_0.size() && withf_rest_0);
    return f_rest_0[index];
  }
  attr_t getf_rest_1(const size_t index) const
  {
    assert(index < f_rest_1.size() && withf_rest_1);
    return f_rest_1[index];
  }
  attr_t getf_rest_2(const size_t index) const
  {
    assert(index < f_rest_2.size() && withf_rest_2);
    return f_rest_2[index];
  }
  attr_t getf_rest_3(const size_t index) const
  {
    assert(index < f_rest_3.size() && withf_rest_3);
    return f_rest_3[index];
  }
  attr_t getf_rest_4(const size_t index) const
  {
    assert(index < f_rest_4.size() && withf_rest_4);
    return f_rest_4[index];
  }
  attr_t getf_rest_5(const size_t index) const
  {
    assert(index < f_rest_5.size() && withf_rest_5);
    return f_rest_5[index];
  }
  attr_t getf_rest_6(const size_t index) const
  {
    assert(index < f_rest_6.size() && withf_rest_6);
    return f_rest_6[index];
  }
  attr_t getf_rest_7(const size_t index) const
  {
    assert(index < f_rest_7.size() && withf_rest_7);
    return f_rest_7[index];
  }
  attr_t getf_rest_8(const size_t index) const
  {
    assert(index < f_rest_8.size() && withf_rest_8);
    return f_rest_8[index];
  }
  attr_t getf_rest_9(const size_t index) const
  {
    assert(index < f_rest_9.size() && withf_rest_9);
    return f_rest_9[index];
  }
  attr_t getf_rest_10(const size_t index) const
  {
    assert(index < f_rest_10.size() && withf_rest_10);
    return f_rest_10[index];
  }
  attr_t getf_rest_11(const size_t index) const
  {
    assert(index < f_rest_11.size() && withf_rest_11);
    return f_rest_11[index];
  }
  attr_t getf_rest_12(const size_t index) const
  {
    assert(index < f_rest_12.size() && withf_rest_12);
    return f_rest_12[index];
  }
  attr_t getf_rest_13(const size_t index) const
  {
    assert(index < f_rest_13.size() && withf_rest_13);
    return f_rest_13[index];
  }
  attr_t getf_rest_14(const size_t index) const
  {
    assert(index < f_rest_14.size() && withf_rest_14);
    return f_rest_14[index];
  }
  attr_t getf_rest_15(const size_t index) const
  {
    assert(index < f_rest_15.size() && withf_rest_15);
    return f_rest_15[index];
  }
  attr_t getf_rest_16(const size_t index) const
  {
    assert(index < f_rest_16.size() && withf_rest_16);
    return f_rest_16[index];
  }
  attr_t getf_rest_17(const size_t index) const
  {
    assert(index < f_rest_17.size() && withf_rest_17);
    return f_rest_17[index];
  }
  attr_t getf_rest_18(const size_t index) const
  {
    assert(index < f_rest_18.size() && withf_rest_18);
    return f_rest_18[index];
  }
  attr_t getf_rest_19(const size_t index) const
  {
    assert(index < f_rest_19.size() && withf_rest_19);
    return f_rest_19[index];
  }
  attr_t getf_rest_20(const size_t index) const
  {
    assert(index < f_rest_20.size() && withf_rest_20);
    return f_rest_20[index];
  }
  attr_t getf_rest_21(const size_t index) const
  {
    assert(index < f_rest_21.size() && withf_rest_21);
    return f_rest_21[index];
  }
  attr_t getf_rest_22(const size_t index) const
  {
    assert(index < f_rest_22.size() && withf_rest_22);
    return f_rest_22[index];
  }
  attr_t getf_rest_23(const size_t index) const
  {
    assert(index < f_rest_23.size() && withf_rest_23);
    return f_rest_23[index];
  }
  attr_t getf_rest_24(const size_t index) const
  {
    assert(index < f_rest_24.size() && withf_rest_24);
    return f_rest_24[index];
  }
  attr_t getf_rest_25(const size_t index) const
  {
    assert(index < f_rest_25.size() && withf_rest_25);
    return f_rest_25[index];
  }
  attr_t getf_rest_26(const size_t index) const
  {
    assert(index < f_rest_26.size() && withf_rest_26);
    return f_rest_26[index];
  }
  attr_t getf_rest_27(const size_t index) const
  {
    assert(index < f_rest_27.size() && withf_rest_27);
    return f_rest_27[index];
  }
  attr_t getf_rest_28(const size_t index) const
  {
    assert(index < f_rest_28.size() && withf_rest_28);
    return f_rest_28[index];
  }
  attr_t getf_rest_29(const size_t index) const
  {
    assert(index < f_rest_29.size() && withf_rest_29);
    return f_rest_29[index];
  }
  attr_t getf_rest_30(const size_t index) const
  {
    assert(index < f_rest_30.size() && withf_rest_30);
    return f_rest_30[index];
  }
  attr_t getf_rest_31(const size_t index) const
  {
    assert(index < f_rest_31.size() && withf_rest_31);
    return f_rest_31[index];
  }
  attr_t getf_rest_32(const size_t index) const
  {
    assert(index < f_rest_32.size() && withf_rest_32);
    return f_rest_32[index];
  }
  attr_t getf_rest_33(const size_t index) const
  {
    assert(index < f_rest_33.size() && withf_rest_33);
    return f_rest_33[index];
  }
  attr_t getf_rest_34(const size_t index) const
  {
    assert(index < f_rest_34.size() && withf_rest_34);
    return f_rest_34[index];
  }
  attr_t getf_rest_35(const size_t index) const
  {
    assert(index < f_rest_35.size() && withf_rest_35);
    return f_rest_35[index];
  }
  attr_t getf_rest_36(const size_t index) const
  {
    assert(index < f_rest_36.size() && withf_rest_36);
    return f_rest_36[index];
  }
  attr_t getf_rest_37(const size_t index) const
  {
    assert(index < f_rest_37.size() && withf_rest_37);
    return f_rest_37[index];
  }
  attr_t getf_rest_38(const size_t index) const
  {
    assert(index < f_rest_38.size() && withf_rest_38);
    return f_rest_38[index];
  }
  attr_t getf_rest_39(const size_t index) const
  {
    assert(index < f_rest_39.size() && withf_rest_39);
    return f_rest_39[index];
  }
  attr_t getf_rest_40(const size_t index) const
  {
    assert(index < f_rest_40.size() && withf_rest_40);
    return f_rest_40[index];
  }
  attr_t getf_rest_41(const size_t index) const
  {
    assert(index < f_rest_41.size() && withf_rest_41);
    return f_rest_41[index];
  }
  attr_t getf_rest_42(const size_t index) const
  {
    assert(index < f_rest_42.size() && withf_rest_42);
    return f_rest_42[index];
  }
  attr_t getf_rest_43(const size_t index) const
  {
    assert(index < f_rest_43.size() && withf_rest_43);
    return f_rest_43[index];
  }
  attr_t getf_rest_44(const size_t index) const
  {
    assert(index < f_rest_44.size() && withf_rest_44);
    return f_rest_44[index];
  }

  attr_t& getReflectance(const size_t index)
  {
    assert(index < reflectances.size() && withReflectances);
    return reflectances[index];
  }
  attr_t& getOpacity(const size_t index)
  {
    assert(index < opacities.size() && withOpacity);
    return opacities[index];
  }
  attr_t& getf_dc_0(const size_t index)
  {
    assert(index < f_dc_0.size() && withf_dc_0);
    return f_dc_0[index];
  }
  attr_t& getf_dc_1(const size_t index)
  {
    assert(index < f_dc_1.size() && withf_dc_1);
    return f_dc_1[index];
  }
  attr_t& getf_dc_2(const size_t index)
  {
    assert(index < f_dc_2.size() && withf_dc_2);
    return f_dc_2[index];
  }
  attr_t& getscale_0(const size_t index)
  {
    assert(index < scale_0.size() && withscale_0);
    return scale_0[index];
  }
  attr_t& getscale_1(const size_t index)
  {
    assert(index < scale_1.size() && withscale_1);
    return scale_1[index];
  }
  attr_t& getscale_2(const size_t index)
  {
    assert(index < scale_2.size() && withscale_2);
    return scale_2[index];
  }
  attr_t& getrot_0(const size_t index)
  {
    assert(index < rot_0.size() && withrot_0);
    return rot_0[index];
  }
  attr_t& getrot_1(const size_t index)
  {
    assert(index < rot_1.size() && withrot_1);
    return rot_1[index];
  }
  attr_t& getrot_2(const size_t index)
  {
    assert(index < rot_2.size() && withrot_2);
    return rot_2[index];
  }
  attr_t& getrot_3(const size_t index)
  {
    assert(index < rot_3.size() && withrot_3);
    return rot_3[index];
  }
  attr_t& getf_rest_0(const size_t index)
  {
    assert(index < f_rest_0.size() && withf_rest_0);
    return f_rest_0[index];
  }
  attr_t& getf_rest_1(const size_t index)
  {
    assert(index < f_rest_1.size() && withf_rest_1);
    return f_rest_1[index];
  }
  attr_t& getf_rest_2(const size_t index)
  {
    assert(index < f_rest_2.size() && withf_rest_2);
    return f_rest_2[index];
  }
  attr_t& getf_rest_3(const size_t index)
  {
    assert(index < f_rest_3.size() && withf_rest_3);
    return f_rest_3[index];
  }
  attr_t& getf_rest_4(const size_t index)
  {
    assert(index < f_rest_4.size() && withf_rest_4);
    return f_rest_4[index];
  }
  attr_t& getf_rest_5(const size_t index)
  {
    assert(index < f_rest_5.size() && withf_rest_5);
    return f_rest_5[index];
  }
  attr_t& getf_rest_6(const size_t index)
  {
    assert(index < f_rest_6.size() && withf_rest_6);
    return f_rest_6[index];
  }
  attr_t& getf_rest_7(const size_t index)
  {
    assert(index < f_rest_7.size() && withf_rest_7);
    return f_rest_7[index];
  }
  attr_t& getf_rest_8(const size_t index)
  {
    assert(index < f_rest_8.size() && withf_rest_8);
    return f_rest_8[index];
  }
  attr_t& getf_rest_9(const size_t index)
  {
    assert(index < f_rest_9.size() && withf_rest_9);
    return f_rest_9[index];
  }
  attr_t& getf_rest_10(const size_t index)
  {
    assert(index < f_rest_10.size() && withf_rest_10);
    return f_rest_10[index];
  }
  attr_t& getf_rest_11(const size_t index)
  {
    assert(index < f_rest_11.size() && withf_rest_11);
    return f_rest_11[index];
  }
  attr_t& getf_rest_12(const size_t index)
  {
    assert(index < f_rest_12.size() && withf_rest_12);
    return f_rest_12[index];
  }
  attr_t& getf_rest_13(const size_t index)
  {
    assert(index < f_rest_13.size() && withf_rest_13);
    return f_rest_13[index];
  }
  attr_t& getf_rest_14(const size_t index)
  {
    assert(index < f_rest_14.size() && withf_rest_14);
    return f_rest_14[index];
  }
  attr_t& getf_rest_15(const size_t index)
  {
    assert(index < f_rest_15.size() && withf_rest_15);
    return f_rest_15[index];
  }
  attr_t& getf_rest_16(const size_t index)
  {
    assert(index < f_rest_16.size() && withf_rest_16);
    return f_rest_16[index];
  }
  attr_t& getf_rest_17(const size_t index)
  {
    assert(index < f_rest_17.size() && withf_rest_17);
    return f_rest_17[index];
  }
  attr_t& getf_rest_18(const size_t index)
  {
    assert(index < f_rest_18.size() && withf_rest_18);
    return f_rest_18[index];
  }
  attr_t& getf_rest_19(const size_t index)
  {
    assert(index < f_rest_19.size() && withf_rest_19);
    return f_rest_19[index];
  }
  attr_t& getf_rest_20(const size_t index)
  {
    assert(index < f_rest_20.size() && withf_rest_20);
    return f_rest_20[index];
  }
  attr_t& getf_rest_21(const size_t index)
  {
    assert(index < f_rest_21.size() && withf_rest_21);
    return f_rest_21[index];
  }
  attr_t& getf_rest_22(const size_t index)
  {
    assert(index < f_rest_22.size() && withf_rest_22);
    return f_rest_22[index];
  }
  attr_t& getf_rest_23(const size_t index)
  {
    assert(index < f_rest_23.size() && withf_rest_23);
    return f_rest_23[index];
  }
  attr_t& getf_rest_24(const size_t index)
  {
    assert(index < f_rest_24.size() && withf_rest_24);
    return f_rest_24[index];
  }
  attr_t& getf_rest_25(const size_t index)
  {
    assert(index < f_rest_25.size() && withf_rest_25);
    return f_rest_25[index];
  }
  attr_t& getf_rest_26(const size_t index)
  {
    assert(index < f_rest_26.size() && withf_rest_26);
    return f_rest_26[index];
  }
  attr_t& getf_rest_27(const size_t index)
  {
    assert(index < f_rest_27.size() && withf_rest_27);
    return f_rest_27[index];
  }
  attr_t& getf_rest_28(const size_t index)
  {
    assert(index < f_rest_28.size() && withf_rest_28);
    return f_rest_28[index];
  }
  attr_t& getf_rest_29(const size_t index)
  {
    assert(index < f_rest_29.size() && withf_rest_29);
    return f_rest_29[index];
  }
  attr_t& getf_rest_30(const size_t index)
  {
    assert(index < f_rest_30.size() && withf_rest_30);
    return f_rest_30[index];
  }
  attr_t& getf_rest_31(const size_t index)
  {
    assert(index < f_rest_31.size() && withf_rest_31);
    return f_rest_31[index];
  }
  attr_t& getf_rest_32(const size_t index)
  {
    assert(index < f_rest_32.size() && withf_rest_32);
    return f_rest_32[index];
  }
  attr_t& getf_rest_33(const size_t index)
  {
    assert(index < f_rest_33.size() && withf_rest_33);
    return f_rest_33[index];
  }
  attr_t& getf_rest_34(const size_t index)
  {
    assert(index < f_rest_34.size() && withf_rest_34);
    return f_rest_34[index];
  }
  attr_t& getf_rest_35(const size_t index)
  {
    assert(index < f_rest_35.size() && withf_rest_35);
    return f_rest_35[index];
  }
  attr_t& getf_rest_36(const size_t index)
  {
    assert(index < f_rest_36.size() && withf_rest_36);
    return f_rest_36[index];
  }
  attr_t& getf_rest_37(const size_t index)
  {
    assert(index < f_rest_37.size() && withf_rest_37);
    return f_rest_37[index];
  }
  attr_t& getf_rest_38(const size_t index)
  {
    assert(index < f_rest_38.size() && withf_rest_38);
    return f_rest_38[index];
  }
  attr_t& getf_rest_39(const size_t index)
  {
    assert(index < f_rest_39.size() && withf_rest_39);
    return f_rest_39[index];
  }
  attr_t& getf_rest_40(const size_t index)
  {
    assert(index < f_rest_40.size() && withf_rest_40);
    return f_rest_40[index];
  }
  attr_t& getf_rest_41(const size_t index)
  {
    assert(index < f_rest_41.size() && withf_rest_41);
    return f_rest_41[index];
  }
  attr_t& getf_rest_42(const size_t index)
  {
    assert(index < f_rest_42.size() && withf_rest_42);
    return f_rest_42[index];
  }
  attr_t& getf_rest_43(const size_t index)
  {
    assert(index < f_rest_43.size() && withf_rest_43);
    return f_rest_43[index];
  }
  attr_t& getf_rest_44(const size_t index)
  {
    assert(index < f_rest_44.size() && withf_rest_44);
    return f_rest_44[index];
  }

  void setReflectance(const size_t index, const attr_t reflectance)
  {
    assert(index < reflectances.size() && withReflectances);
    reflectances[index] = reflectance;
  }
  void setOpacity(const size_t index, const attr_t opacity)
  {
    assert(index < opacities.size() && withOpacity);
    opacities[index] = opacity;
  }
  void setf_dc_0(const size_t index, const attr_t value)
  {
    assert(index < f_dc_0.size() && withf_dc_0);
    f_dc_0[index] = value;
  }
  void setf_dc_1(const size_t index, const attr_t value)
  {
    assert(index < f_dc_1.size() && withf_dc_1);
    f_dc_1[index] = value;
  }
  void setf_dc_2(const size_t index, const attr_t value)
  {
    assert(index < f_dc_2.size() && withf_dc_2);
    f_dc_2[index] = value;
  }
  void setscale_0(const size_t index, const attr_t value)
  {
    assert(index < scale_0.size() && withscale_0);
    scale_0[index] = value;
  }
  void setscale_1(const size_t index, const attr_t value)
  {
    assert(index < scale_1.size() && withscale_1);
    scale_1[index] = value;
  }
  void setscale_2(const size_t index, const attr_t value)
  {
    assert(index < scale_2.size() && withscale_2);
    scale_2[index] = value;
  }
  void setrot_0(const size_t index, const attr_t value)
  {
    assert(index < rot_0.size() && withrot_0);
    rot_0[index] = value;
  }
  void setrot_1(const size_t index, const attr_t value)
  {
    assert(index < rot_1.size() && withrot_1);
    rot_1[index] = value;
  }
  void setrot_2(const size_t index, const attr_t value)
  {
    assert(index < rot_2.size() && withrot_2);
    rot_2[index] = value;
  }
  void setrot_3(const size_t index, const attr_t value)
  {
    assert(index < rot_3.size() && withrot_3);
    rot_3[index] = value;
  }
  void setf_rest_0(const size_t index, const attr_t value)
  {
    assert(index < f_rest_0.size() && withf_rest_0);
    f_rest_0[index] = value;
  }
  void setf_rest_1(const size_t index, const attr_t value)
  {
    assert(index < f_rest_1.size() && withf_rest_1);
    f_rest_1[index] = value;
  }
  void setf_rest_2(const size_t index, const attr_t value)
  {
    assert(index < f_rest_2.size() && withf_rest_2);
    f_rest_2[index] = value;
  }
  void setf_rest_3(const size_t index, const attr_t value)
  {
    assert(index < f_rest_3.size() && withf_rest_3);
    f_rest_3[index] = value;
  }
  void setf_rest_4(const size_t index, const attr_t value)
  {
    assert(index < f_rest_4.size() && withf_rest_4);
    f_rest_4[index] = value;
  }
  void setf_rest_5(const size_t index, const attr_t value)
  {
    assert(index < f_rest_5.size() && withf_rest_5);
    f_rest_5[index] = value;
  }
  void setf_rest_6(const size_t index, const attr_t value)
  {
    assert(index < f_rest_6.size() && withf_rest_6);
    f_rest_6[index] = value;
  }
  void setf_rest_7(const size_t index, const attr_t value)
  {
    assert(index < f_rest_7.size() && withf_rest_7);
    f_rest_7[index] = value;
  }
  void setf_rest_8(const size_t index, const attr_t value)
  {
    assert(index < f_rest_8.size() && withf_rest_8);
    f_rest_8[index] = value;
  }
  void setf_rest_9(const size_t index, const attr_t value)
  {
    assert(index < f_rest_9.size() && withf_rest_9);
    f_rest_9[index] = value;
  }
  void setf_rest_10(const size_t index, const attr_t value)
  {
    assert(index < f_rest_10.size() && withf_rest_10);
    f_rest_10[index] = value;
  }
  void setf_rest_11(const size_t index, const attr_t value)
  {
    assert(index < f_rest_11.size() && withf_rest_11);
    f_rest_11[index] = value;
  }
  void setf_rest_12(const size_t index, const attr_t value)
  {
    assert(index < f_rest_12.size() && withf_rest_12);
    f_rest_12[index] = value;
  }
  void setf_rest_13(const size_t index, const attr_t value)
  {
    assert(index < f_rest_13.size() && withf_rest_13);
    f_rest_13[index] = value;
  }
  void setf_rest_14(const size_t index, const attr_t value)
  {
    assert(index < f_rest_14.size() && withf_rest_14);
    f_rest_14[index] = value;
  }
  void setf_rest_15(const size_t index, const attr_t value)
  {
    assert(index < f_rest_15.size() && withf_rest_15);
    f_rest_15[index] = value;
  }
  void setf_rest_16(const size_t index, const attr_t value)
  {
    assert(index < f_rest_16.size() && withf_rest_16);
    f_rest_16[index] = value;
  }
  void setf_rest_17(const size_t index, const attr_t value)
  {
    assert(index < f_rest_17.size() && withf_rest_17);
    f_rest_17[index] = value;
  }
  void setf_rest_18(const size_t index, const attr_t value)
  {
    assert(index < f_rest_18.size() && withf_rest_18);
    f_rest_18[index] = value;
  }
  void setf_rest_19(const size_t index, const attr_t value)
  {
    assert(index < f_rest_19.size() && withf_rest_19);
    f_rest_19[index] = value;
  }
  void setf_rest_20(const size_t index, const attr_t value)
  {
    assert(index < f_rest_20.size() && withf_rest_20);
    f_rest_20[index] = value;
  }
  void setf_rest_21(const size_t index, const attr_t value)
  {
    assert(index < f_rest_21.size() && withf_rest_21);
    f_rest_21[index] = value;
  }
  void setf_rest_22(const size_t index, const attr_t value)
  {
    assert(index < f_rest_22.size() && withf_rest_22);
    f_rest_22[index] = value;
  }
  void setf_rest_23(const size_t index, const attr_t value)
  {
    assert(index < f_rest_23.size() && withf_rest_23);
    f_rest_23[index] = value;
  }
  void setf_rest_24(const size_t index, const attr_t value)
  {
    assert(index < f_rest_24.size() && withf_rest_24);
    f_rest_24[index] = value;
  }
  void setf_rest_25(const size_t index, const attr_t value)
  {
    assert(index < f_rest_25.size() && withf_rest_25);
    f_rest_25[index] = value;
  }
  void setf_rest_26(const size_t index, const attr_t value)
  {
    assert(index < f_rest_26.size() && withf_rest_26);
    f_rest_26[index] = value;
  }
  void setf_rest_27(const size_t index, const attr_t value)
  {
    assert(index < f_rest_27.size() && withf_rest_27);
    f_rest_27[index] = value;
  }
  void setf_rest_28(const size_t index, const attr_t value)
  {
    assert(index < f_rest_28.size() && withf_rest_28);
    f_rest_28[index] = value;
  }
  void setf_rest_29(const size_t index, const attr_t value)
  {
    assert(index < f_rest_29.size() && withf_rest_29);
    f_rest_29[index] = value;
  }
  void setf_rest_30(const size_t index, const attr_t value)
  {
    assert(index < f_rest_30.size() && withf_rest_30);
    f_rest_30[index] = value;
  }
  void setf_rest_31(const size_t index, const attr_t value)
  {
    assert(index < f_rest_31.size() && withf_rest_31);
    f_rest_31[index] = value;
  }
  void setf_rest_32(const size_t index, const attr_t value)
  {
    assert(index < f_rest_32.size() && withf_rest_32);
    f_rest_32[index] = value;
  }
  void setf_rest_33(const size_t index, const attr_t value)
  {
    assert(index < f_rest_33.size() && withf_rest_33);
    f_rest_33[index] = value;
  }
  void setf_rest_34(const size_t index, const attr_t value)
  {
    assert(index < f_rest_34.size() && withf_rest_34);
    f_rest_34[index] = value;
  }
  void setf_rest_35(const size_t index, const attr_t value)
  {
    assert(index < f_rest_35.size() && withf_rest_35);
    f_rest_35[index] = value;
  }
  void setf_rest_36(const size_t index, const attr_t value)
  {
    assert(index < f_rest_36.size() && withf_rest_36);
    f_rest_36[index] = value;
  }
  void setf_rest_37(const size_t index, const attr_t value)
  {
    assert(index < f_rest_37.size() && withf_rest_37);
    f_rest_37[index] = value;
  }
  void setf_rest_38(const size_t index, const attr_t value)
  {
    assert(index < f_rest_38.size() && withf_rest_38);
    f_rest_38[index] = value;
  }
  void setf_rest_39(const size_t index, const attr_t value)
  {
    assert(index < f_rest_39.size() && withf_rest_39);
    f_rest_39[index] = value;
  }
  void setf_rest_40(const size_t index, const attr_t value)
  {
    assert(index < f_rest_40.size() && withf_rest_40);
    f_rest_40[index] = value;
  }
  void setf_rest_41(const size_t index, const attr_t value)
  {
    assert(index < f_rest_41.size() && withf_rest_41);
    f_rest_41[index] = value;
  }
  void setf_rest_42(const size_t index, const attr_t value)
  {
    assert(index < f_rest_42.size() && withf_rest_42);
    f_rest_42[index] = value;
  }
  void setf_rest_43(const size_t index, const attr_t value)
  {
    assert(index < f_rest_43.size() && withf_rest_43);
    f_rest_43[index] = value;
  }
  void setf_rest_44(const size_t index, const attr_t value)
  {
    assert(index < f_rest_44.size() && withf_rest_44);
    f_rest_44[index] = value;
  }

  bool hasReflectances() const { return withReflectances; }
  bool hasOpacities() const { return withOpacity; }
  bool hasf_dc_0() const { return withf_dc_0; }
  bool hasf_dc_1() const { return withf_dc_1; }
  bool hasf_dc_2() const { return withf_dc_2; }
  bool hasscale_0() const { return withscale_0; }
  bool hasscale_1() const { return withscale_1; }
  bool hasscale_2() const { return withscale_2; }
  bool hasrot_0() const { return withrot_0; }
  bool hasrot_1() const { return withrot_1; }
  bool hasrot_2() const { return withrot_2; }
  bool hasrot_3() const { return withrot_3; }
  bool hasf_rest_0() const { return withf_rest_0; }
  bool hasf_rest_1() const { return withf_rest_1; }
  bool hasf_rest_2() const { return withf_rest_2; }
  bool hasf_rest_3() const { return withf_rest_3; }
  bool hasf_rest_4() const { return withf_rest_4; }
  bool hasf_rest_5() const { return withf_rest_5; }
  bool hasf_rest_6() const { return withf_rest_6; }
  bool hasf_rest_7() const { return withf_rest_7; }
  bool hasf_rest_8() const { return withf_rest_8; }
  bool hasf_rest_9() const { return withf_rest_9; }
  bool hasf_rest_10() const { return withf_rest_10; }
  bool hasf_rest_11() const { return withf_rest_11; }
  bool hasf_rest_12() const { return withf_rest_12; }
  bool hasf_rest_13() const { return withf_rest_13; }
  bool hasf_rest_14() const { return withf_rest_14; }
  bool hasf_rest_15() const { return withf_rest_15; }
  bool hasf_rest_16() const { return withf_rest_16; }
  bool hasf_rest_17() const { return withf_rest_17; }
  bool hasf_rest_18() const { return withf_rest_18; }
  bool hasf_rest_19() const { return withf_rest_19; }
  bool hasf_rest_20() const { return withf_rest_20; }
  bool hasf_rest_21() const { return withf_rest_21; }
  bool hasf_rest_22() const { return withf_rest_22; }
  bool hasf_rest_23() const { return withf_rest_23; }
  bool hasf_rest_24() const { return withf_rest_24; }
  bool hasf_rest_25() const { return withf_rest_25; }
  bool hasf_rest_26() const { return withf_rest_26; }
  bool hasf_rest_27() const { return withf_rest_27; }
  bool hasf_rest_28() const { return withf_rest_28; }
  bool hasf_rest_29() const { return withf_rest_29; }
  bool hasf_rest_30() const { return withf_rest_30; }
  bool hasf_rest_31() const { return withf_rest_31; }
  bool hasf_rest_32() const { return withf_rest_32; }
  bool hasf_rest_33() const { return withf_rest_33; }
  bool hasf_rest_34() const { return withf_rest_34; }
  bool hasf_rest_35() const { return withf_rest_35; }
  bool hasf_rest_36() const { return withf_rest_36; }
  bool hasf_rest_37() const { return withf_rest_37; }
  bool hasf_rest_38() const { return withf_rest_38; }
  bool hasf_rest_39() const { return withf_rest_39; }
  bool hasf_rest_40() const { return withf_rest_40; }
  bool hasf_rest_41() const { return withf_rest_41; }
  bool hasf_rest_42() const { return withf_rest_42; }
  bool hasf_rest_43() const { return withf_rest_43; }
  bool hasf_rest_44() const { return withf_rest_44; }

  void addReflectances()
  {
    withReflectances = true;
    resize(getPointCount());
  }
  void addOpacities()
  {
    withOpacity = true;
    resize(getPointCount());
  }
  void addf_dc_0()
  {
    withf_dc_0 = true;
    resize(getPointCount());
  }
  void addf_dc_1()
  {
    withf_dc_1 = true;
    resize(getPointCount());
  }
  void addf_dc_2()
  {
    withf_dc_2 = true;
    resize(getPointCount());
  }
  void addscale_0()
  {
    withscale_0 = true;
    resize(getPointCount());
  }
  void addscale_1()
  {
    withscale_1 = true;
    resize(getPointCount());
  }
  void addscale_2()
  {
    withscale_2 = true;
    resize(getPointCount());
  }
  void addrot_0()
  {
    withrot_0 = true;
    resize(getPointCount());
  }
  void addrot_1()
  {
    withrot_1 = true;
    resize(getPointCount());
  }
  void addrot_2()
  {
    withrot_2 = true;
    resize(getPointCount());
  }
  void addrot_3()
  {
    withrot_3 = true;
    resize(getPointCount());
  }
  void addf_rest_0()
  {
    withf_rest_0 = true;
    resize(getPointCount());
  }
  void addf_rest_1()
  {
    withf_rest_1 = true;
    resize(getPointCount());
  }
  void addf_rest_2()
  {
    withf_rest_2 = true;
    resize(getPointCount());
  }
  void addf_rest_3()
  {
    withf_rest_3 = true;
    resize(getPointCount());
  }
  void addf_rest_4()
  {
    withf_rest_4 = true;
    resize(getPointCount());
  }
  void addf_rest_5()
  {
    withf_rest_5 = true;
    resize(getPointCount());
  }
  void addf_rest_6()
  {
    withf_rest_6 = true;
    resize(getPointCount());
  }
  void addf_rest_7()
  {
    withf_rest_7 = true;
    resize(getPointCount());
  }
  void addf_rest_8()
  {
    withf_rest_8 = true;
    resize(getPointCount());
  }
  void addf_rest_9()
  {
    withf_rest_9 = true;
    resize(getPointCount());
  }
  void addf_rest_10()
  {
    withf_rest_10 = true;
    resize(getPointCount());
  }
  void addf_rest_11()
  {
    withf_rest_11 = true;
    resize(getPointCount());
  }
  void addf_rest_12()
  {
    withf_rest_12 = true;
    resize(getPointCount());
  }
  void addf_rest_13()
  {
    withf_rest_13 = true;
    resize(getPointCount());
  }
  void addf_rest_14()
  {
    withf_rest_14 = true;
    resize(getPointCount());
  }
  void addf_rest_15()
  {
    withf_rest_15 = true;
    resize(getPointCount());
  }
  void addf_rest_16()
  {
    withf_rest_16 = true;
    resize(getPointCount());
  }
  void addf_rest_17()
  {
    withf_rest_17 = true;
    resize(getPointCount());
  }
  void addf_rest_18()
  {
    withf_rest_18 = true;
    resize(getPointCount());
  }
  void addf_rest_19()
  {
    withf_rest_19 = true;
    resize(getPointCount());
  }
  void addf_rest_20()
  {
    withf_rest_20 = true;
    resize(getPointCount());
  }
  void addf_rest_21()
  {
    withf_rest_21 = true;
    resize(getPointCount());
  }
  void addf_rest_22()
  {
    withf_rest_22 = true;
    resize(getPointCount());
  }
  void addf_rest_23()
  {
    withf_rest_23 = true;
    resize(getPointCount());
  }
  void addf_rest_24()
  {
    withf_rest_24 = true;
    resize(getPointCount());
  }
  void addf_rest_25()
  {
    withf_rest_25 = true;
    resize(getPointCount());
  }
  void addf_rest_26()
  {
    withf_rest_26 = true;
    resize(getPointCount());
  }
  void addf_rest_27()
  {
    withf_rest_27 = true;
    resize(getPointCount());
  }
  void addf_rest_28()
  {
    withf_rest_28 = true;
    resize(getPointCount());
  }
  void addf_rest_29()
  {
    withf_rest_29 = true;
    resize(getPointCount());
  }
  void addf_rest_30()
  {
    withf_rest_30 = true;
    resize(getPointCount());
  }
  void addf_rest_31()
  {
    withf_rest_31 = true;
    resize(getPointCount());
  }
  void addf_rest_32()
  {
    withf_rest_32 = true;
    resize(getPointCount());
  }
  void addf_rest_33()
  {
    withf_rest_33 = true;
    resize(getPointCount());
  }
  void addf_rest_34()
  {
    withf_rest_34 = true;
    resize(getPointCount());
  }
  void addf_rest_35()
  {
    withf_rest_35 = true;
    resize(getPointCount());
  }
  void addf_rest_36()
  {
    withf_rest_36 = true;
    resize(getPointCount());
  }
  void addf_rest_37()
  {
    withf_rest_37 = true;
    resize(getPointCount());
  }
  void addf_rest_38()
  {
    withf_rest_38 = true;
    resize(getPointCount());
  }
  void addf_rest_39()
  {
    withf_rest_39 = true;
    resize(getPointCount());
  }
  void addf_rest_40()
  {
    withf_rest_40 = true;
    resize(getPointCount());
  }
  void addf_rest_41()
  {
    withf_rest_41 = true;
    resize(getPointCount());
  }
  void addf_rest_42()
  {
    withf_rest_42 = true;
    resize(getPointCount());
  }
  void addf_rest_43()
  {
    withf_rest_43 = true;
    resize(getPointCount());
  }
  void addf_rest_44()
  {
    withf_rest_44 = true;
    resize(getPointCount());
  }

  void removeReflectances()
  {
    withReflectances = false;
    reflectances.resize(0);
  }
  void removeOpacities()
  {
    withOpacity = false;
    opacities.resize(0);
  }
  void removef_dc_0()
  {
    withf_dc_0 = false;
    f_dc_0.resize(0);
  }
  void removef_dc_1()
  {
    withf_dc_1 = false;
    f_dc_1.resize(0);
  }
  void removef_dc_2()
  {
    withf_dc_2 = false;
    f_dc_2.resize(0);
  }
  void removescale_0()
  {
    withscale_0 = false;
    scale_0.resize(0);
  }
  void removescale_1()
  {
    withscale_1 = false;
    scale_1.resize(0);
  }
  void removescale_2()
  {
    withscale_2 = false;
    scale_2.resize(0);
  }
  void removerot_0()
  {
    withrot_0 = false;
    rot_0.resize(0);
  }
  void removerot_1()
  {
    withrot_1 = false;
    rot_1.resize(0);
  }
  void removerot_2()
  {
    withrot_2 = false;
    rot_2.resize(0);
  }
  void removerot_3()
  {
    withrot_3 = false;
    rot_3.resize(0);
  }
  void removef_rest_0()
  {
    withf_rest_0 = false;
    f_rest_0.resize(0);
  }
  void removef_rest_1()
  {
    withf_rest_1 = false;
    f_rest_1.resize(0);
  }
  void removef_rest_2()
  {
    withf_rest_2 = false;
    f_rest_2.resize(0);
  }
  void removef_rest_3()
  {
    withf_rest_3 = false;
    f_rest_3.resize(0);
  }
  void removef_rest_4()
  {
    withf_rest_4 = false;
    f_rest_4.resize(0);
  }
  void removef_rest_5()
  {
    withf_rest_5 = false;
    f_rest_5.resize(0);
  }
  void removef_rest_6()
  {
    withf_rest_6 = false;
    f_rest_6.resize(0);
  }
  void removef_rest_7()
  {
    withf_rest_7 = false;
    f_rest_7.resize(0);
  }
  void removef_rest_8()
  {
    withf_rest_8 = false;
    f_rest_8.resize(0);
  }
  void removef_rest_9()
  {
    withf_rest_9 = false;
    f_rest_9.resize(0);
  }
  void removef_rest_10()
  {
    withf_rest_10 = false;
    f_rest_10.resize(0);
  }
  void removef_rest_11()
  {
    withf_rest_11 = false;
    f_rest_11.resize(0);
  }
  void removef_rest_12()
  {
    withf_rest_12 = false;
    f_rest_12.resize(0);
  }
  void removef_rest_13()
  {
    withf_rest_13 = false;
    f_rest_13.resize(0);
  }
  void removef_rest_14()
  {
    withf_rest_14 = false;
    f_rest_14.resize(0);
  }
  void removef_rest_15()
  {
    withf_rest_15 = false;
    f_rest_15.resize(0);
  }
  void removef_rest_16()
  {
    withf_rest_16 = false;
    f_rest_16.resize(0);
  }
  void removef_rest_17()
  {
    withf_rest_17 = false;
    f_rest_17.resize(0);
  }
  void removef_rest_18()
  {
    withf_rest_18 = false;
    f_rest_18.resize(0);
  }
  void removef_rest_19()
  {
    withf_rest_19 = false;
    f_rest_19.resize(0);
  }
  void removef_rest_20()
  {
    withf_rest_20 = false;
    f_rest_20.resize(0);
  }
  void removef_rest_21()
  {
    withf_rest_21 = false;
    f_rest_21.resize(0);
  }
  void removef_rest_22()
  {
    withf_rest_22 = false;
    f_rest_22.resize(0);
  }
  void removef_rest_23()
  {
    withf_rest_23 = false;
    f_rest_23.resize(0);
  }
  void removef_rest_24()
  {
    withf_rest_24 = false;
    f_rest_24.resize(0);
  }
  void removef_rest_25()
  {
    withf_rest_25 = false;
    f_rest_25.resize(0);
  }
  void removef_rest_26()
  {
    withf_rest_26 = false;
    f_rest_26.resize(0);
  }
  void removef_rest_27()
  {
    withf_rest_27 = false;
    f_rest_27.resize(0);
  }
  void removef_rest_28()
  {
    withf_rest_28 = false;
    f_rest_28.resize(0);
  }
  void removef_rest_29()
  {
    withf_rest_29 = false;
    f_rest_29.resize(0);
  }
  void removef_rest_30()
  {
    withf_rest_30 = false;
    f_rest_30.resize(0);
  }
  void removef_rest_31()
  {
    withf_rest_31 = false;
    f_rest_31.resize(0);
  }
  void removef_rest_32()
  {
    withf_rest_32 = false;
    f_rest_32.resize(0);
  }
  void removef_rest_33()
  {
    withf_rest_33 = false;
    f_rest_33.resize(0);
  }
  void removef_rest_34()
  {
    withf_rest_34 = false;
    f_rest_34.resize(0);
  }
  void removef_rest_35()
  {
    withf_rest_35 = false;
    f_rest_35.resize(0);
  }
  void removef_rest_36()
  {
    withf_rest_36 = false;
    f_rest_36.resize(0);
  }
  void removef_rest_37()
  {
    withf_rest_37 = false;
    f_rest_37.resize(0);
  }
  void removef_rest_38()
  {
    withf_rest_38 = false;
    f_rest_38.resize(0);
  }
  void removef_rest_39()
  {
    withf_rest_39 = false;
    f_rest_39.resize(0);
  }
  void removef_rest_40()
  {
    withf_rest_40 = false;
    f_rest_40.resize(0);
  }
  void removef_rest_41()
  {
    withf_rest_41 = false;
    f_rest_41.resize(0);
  }
  void removef_rest_42()
  {
    withf_rest_42 = false;
    f_rest_42.resize(0);
  }
  void removef_rest_43()
  {
    withf_rest_43 = false;
    f_rest_43.resize(0);
  }
  void removef_rest_44()
  {
    withf_rest_44 = false;
    f_rest_44.resize(0);
  }

  uint8_t getFrameIndex(const size_t index) const
  {
    assert(index < frameidx.size() && withFrameIndex);
    return frameidx[index];
  }

  uint8_t& getFrameIndex(const size_t index)
  {
    assert(index < frameidx.size() && withFrameIndex);
    return frameidx[index];
  }

  void setFrameIndex(const size_t index, const uint8_t frameindex)
  {
    assert(index < frameidx.size() && withFrameIndex);
    frameidx[index] = frameindex;
  }

  bool hasFrameIndex() const { return withFrameIndex; }
  void addFrameIndex()
  {
    withFrameIndex = true;
    resize(getPointCount());
  }
  void removeFrameIndex()
  {
    withFrameIndex = false;
    frameidx.resize(0);
  }

  int getLaserAngle(const size_t index) const
  {
    assert(index < laserAngles.size() && withLaserAngles);
    return laserAngles[index];
  }

  int& getLaserAngle(const size_t index)
  {
    assert(index < laserAngles.size() && withLaserAngles);
    return laserAngles[index];
  }

  void setLaserAngle(const size_t index, const int angle)
  {
    assert(index < laserAngles.size() && withLaserAngles);
    laserAngles[index] = angle;
  }

  bool hasLaserAngles() const { return withLaserAngles; }
  void addLaserAngles()
  {
    withLaserAngles = true;
    resize(getPointCount());
  }
  void removeLaserAngles()
  {
    withLaserAngles = false;
    laserAngles.resize(0);
  }

  bool hasColors() const { return withColors; }
  void addColors()
  {
    withColors = true;
    resize(getPointCount());
  }
  void removeColors()
  {
    withColors = false;
    colors.resize(0);
  }

  void addRemoveAttributes(bool withColors, bool withReflectances)
  {
    if (withColors)
      addColors();
    else
      removeColors();

    if (withReflectances)
      addReflectances();
    else
      removeReflectances();
  }

  void addRemoveOpacities(bool withOpacites)
  {
    if (withOpacites)
      addOpacities();
    else
      removeOpacities();
  }

  void addRemovef_dc_0(bool withf_dc_0)
  {
    if (withf_dc_0)
      addf_dc_0();
    else
      removef_dc_0();
  }
  void addRemovef_dc_1(bool withf_dc_1)
  {
    if (withf_dc_1)
      addf_dc_1();
    else
      removef_dc_1();
  }
  void addRemovef_dc_2(bool withf_dc_2)
  {
    if (withf_dc_2)
      addf_dc_2();
    else
      removef_dc_2();
  }

  void addRemovescale_0(bool withscale_0)
  {
    if (withscale_0)
      addscale_0();
    else
      removescale_0();
  }
  void addRemovescale_1(bool withscale_1)
  {
    if (withscale_1)
      addscale_1();
    else
      removescale_1();
  }
  void addRemovescale_2(bool withscale_2)
  {
    if (withscale_2)
      addscale_2();
    else
      removescale_2();
  }
  void addRemoverot_0(bool withrot_0)
  {
    if (withrot_0)
      addrot_0();
    else
      removerot_0();
  }
  void addRemoverot_1(bool withrot_1)
  {
    if (withrot_1)
      addrot_1();
    else
      removerot_1();
  }
  void addRemoverot_2(bool withrot_2)
  {
    if (withrot_2)
      addrot_2();
    else
      removerot_2();
  }
  void addRemoverot_3(bool withrot_3)
  {
    if (withrot_3)
      addrot_3();
    else
      removerot_3();
  }
  void addRemovef_rest_0(bool withf_rest_0)
  {
    if (withf_rest_0)
      addf_rest_0();
    else
      removef_rest_0();
  }
  void addRemovef_rest_1(bool withf_rest_1)
  {
    if (withf_rest_1)
      addf_rest_1();
    else
      removef_rest_1();
  }
  void addRemovef_rest_2(bool withf_rest_2)
  {
    if (withf_rest_2)
      addf_rest_2();
    else
      removef_rest_2();
  }
  void addRemovef_rest_3(bool withf_rest_3)
  {
    if (withf_rest_3)
      addf_rest_3();
    else
      removef_rest_3();
  }
  void addRemovef_rest_4(bool withf_rest_4)
  {
    if (withf_rest_4)
      addf_rest_4();
    else
      removef_rest_4();
  }
  void addRemovef_rest_5(bool withf_rest_5)
  {
    if (withf_rest_5)
      addf_rest_5();
    else
      removef_rest_5();
  }
  void addRemovef_rest_6(bool withf_rest_6)
  {
    if (withf_rest_6)
      addf_rest_6();
    else
      removef_rest_6();
  }
  void addRemovef_rest_7(bool withf_rest_7)
  {
    if (withf_rest_7)
      addf_rest_7();
    else
      removef_rest_7();
  }
  void addRemovef_rest_8(bool withf_rest_8)
  {
    if (withf_rest_8)
      addf_rest_8();
    else
      removef_rest_8();
  }
  void addRemovef_rest_9(bool withf_rest_9)
  {
    if (withf_rest_9)
      addf_rest_9();
    else
      removef_rest_9();
  }
  void addRemovef_rest_10(bool withf_rest_10)
  {
    if (withf_rest_10)
      addf_rest_10();
    else
      removef_rest_10();
  }
  void addRemovef_rest_11(bool withf_rest_11)
  {
    if (withf_rest_11)
      addf_rest_11();
    else
      removef_rest_11();
  }
  void addRemovef_rest_12(bool withf_rest_12)
  {
    if (withf_rest_12)
      addf_rest_12();
    else
      removef_rest_12();
  }
  void addRemovef_rest_13(bool withf_rest_13)
  {
    if (withf_rest_13)
      addf_rest_13();
    else
      removef_rest_13();
  }
  void addRemovef_rest_14(bool withf_rest_14)
  {
    if (withf_rest_14)
      addf_rest_14();
    else
      removef_rest_14();
  }
  void addRemovef_rest_15(bool withf_rest_15)
  {
    if (withf_rest_15)
      addf_rest_15();
    else
      removef_rest_15();
  }
  void addRemovef_rest_16(bool withf_rest_16)
  {
    if (withf_rest_16)
      addf_rest_16();
    else
      removef_rest_16();
  }
  void addRemovef_rest_17(bool withf_rest_17)
  {
    if (withf_rest_17)
      addf_rest_17();
    else
      removef_rest_17();
  }
  void addRemovef_rest_18(bool withf_rest_18)
  {
    if (withf_rest_18)
      addf_rest_18();
    else
      removef_rest_18();
  }
  void addRemovef_rest_19(bool withf_rest_19)
  {
    if (withf_rest_19)
      addf_rest_19();
    else
      removef_rest_19();
  }
  void addRemovef_rest_20(bool withf_rest_20)
  {
    if (withf_rest_20)
      addf_rest_20();
    else
      removef_rest_20();
  }
  void addRemovef_rest_21(bool withf_rest_21)
  {
    if (withf_rest_21)
      addf_rest_21();
    else
      removef_rest_21();
  }
  void addRemovef_rest_22(bool withf_rest_22)
  {
    if (withf_rest_22)
      addf_rest_22();
    else
      removef_rest_22();
  }
  void addRemovef_rest_23(bool withf_rest_23)
  {
    if (withf_rest_23)
      addf_rest_23();
    else
      removef_rest_23();
  }
  void addRemovef_rest_24(bool withf_rest_24)
  {
    if (withf_rest_24)
      addf_rest_24();
    else
      removef_rest_24();
  }
  void addRemovef_rest_25(bool withf_rest_25)
  {
    if (withf_rest_25)
      addf_rest_25();
    else
      removef_rest_25();
  }
  void addRemovef_rest_26(bool withf_rest_26)
  {
    if (withf_rest_26)
      addf_rest_26();
    else
      removef_rest_26();
  }
  void addRemovef_rest_27(bool withf_rest_27)
  {
    if (withf_rest_27)
      addf_rest_27();
    else
      removef_rest_27();
  }
  void addRemovef_rest_28(bool withf_rest_28)
  {
    if (withf_rest_28)
      addf_rest_28();
    else
      removef_rest_28();
  }
  void addRemovef_rest_29(bool withf_rest_29)
  {
    if (withf_rest_29)
      addf_rest_29();
    else
      removef_rest_29();
  }
  void addRemovef_rest_30(bool withf_rest_30)
  {
    if (withf_rest_30)
      addf_rest_30();
    else
      removef_rest_30();
  }
  void addRemovef_rest_31(bool withf_rest_31)
  {
    if (withf_rest_31)
      addf_rest_31();
    else
      removef_rest_31();
  }
  void addRemovef_rest_32(bool withf_rest_32)
  {
    if (withf_rest_32)
      addf_rest_32();
    else
      removef_rest_32();
  }
  void addRemovef_rest_33(bool withf_rest_33)
  {
    if (withf_rest_33)
      addf_rest_33();
    else
      removef_rest_33();
  }
  void addRemovef_rest_34(bool withf_rest_34)
  {
    if (withf_rest_34)
      addf_rest_34();
    else
      removef_rest_34();
  }
  void addRemovef_rest_35(bool withf_rest_35)
  {
    if (withf_rest_35)
      addf_rest_35();
    else
      removef_rest_35();
  }
  void addRemovef_rest_36(bool withf_rest_36)
  {
    if (withf_rest_36)
      addf_rest_36();
    else
      removef_rest_36();
  }
  void addRemovef_rest_37(bool withf_rest_37)
  {
    if (withf_rest_37)
      addf_rest_37();
    else
      removef_rest_37();
  }
  void addRemovef_rest_38(bool withf_rest_38)
  {
    if (withf_rest_38)
      addf_rest_38();
    else
      removef_rest_38();
  }
  void addRemovef_rest_39(bool withf_rest_39)
  {
    if (withf_rest_39)
      addf_rest_39();
    else
      removef_rest_39();
  }
  void addRemovef_rest_40(bool withf_rest_40)
  {
    if (withf_rest_40)
      addf_rest_40();
    else
      removef_rest_40();
  }
  void addRemovef_rest_41(bool withf_rest_41)
  {
    if (withf_rest_41)
      addf_rest_41();
    else
      removef_rest_41();
  }
  void addRemovef_rest_42(bool withf_rest_42)
  {
    if (withf_rest_42)
      addf_rest_42();
    else
      removef_rest_42();
  }
  void addRemovef_rest_43(bool withf_rest_43)
  {
    if (withf_rest_43)
      addf_rest_43();
    else
      removef_rest_43();
  }
  void addRemovef_rest_44(bool withf_rest_44)
  {
    if (withf_rest_44)
      addf_rest_44();
    else
      removef_rest_44();
  }

  void addRemoveAttributes(const PCCPointSet3& ref)
  {
    ref.hasColors() ? addColors() : removeColors();
    ref.hasReflectances() ? addReflectances() : removeReflectances();
    ref.hasOpacities() ? addOpacities() : removeOpacities();
    ref.hasf_dc_0() ? addf_dc_0() : removef_dc_0();
    ref.hasf_dc_1() ? addf_dc_1() : removef_dc_1();
    ref.hasf_dc_2() ? addf_dc_2() : removef_dc_2();
    ref.hasscale_0() ? addscale_0() : removescale_0();
    ref.hasscale_1() ? addscale_1() : removescale_1();
    ref.hasscale_2() ? addscale_2() : removescale_2();
    ref.hasrot_0() ? addrot_0() : removerot_0();
    ref.hasrot_1() ? addrot_1() : removerot_1();
    ref.hasrot_2() ? addrot_2() : removerot_2();
    ref.hasrot_3() ? addrot_3() : removerot_3();
    ref.hasf_rest_0() ? addf_rest_0() : removef_rest_0();
    ref.hasf_rest_1() ? addf_rest_1() : removef_rest_1();
    ref.hasf_rest_2() ? addf_rest_2() : removef_rest_2();
    ref.hasf_rest_3() ? addf_rest_3() : removef_rest_3();
    ref.hasf_rest_4() ? addf_rest_4() : removef_rest_4();
    ref.hasf_rest_5() ? addf_rest_5() : removef_rest_5();
    ref.hasf_rest_6() ? addf_rest_6() : removef_rest_6();
    ref.hasf_rest_7() ? addf_rest_7() : removef_rest_7();
    ref.hasf_rest_8() ? addf_rest_8() : removef_rest_8();
    ref.hasf_rest_9() ? addf_rest_9() : removef_rest_9();
    ref.hasf_rest_10() ? addf_rest_10() : removef_rest_10();
    ref.hasf_rest_11() ? addf_rest_11() : removef_rest_11();
    ref.hasf_rest_12() ? addf_rest_12() : removef_rest_12();
    ref.hasf_rest_13() ? addf_rest_13() : removef_rest_13();
    ref.hasf_rest_14() ? addf_rest_14() : removef_rest_14();
    ref.hasf_rest_15() ? addf_rest_15() : removef_rest_15();
    ref.hasf_rest_16() ? addf_rest_16() : removef_rest_16();
    ref.hasf_rest_17() ? addf_rest_17() : removef_rest_17();
    ref.hasf_rest_18() ? addf_rest_18() : removef_rest_18();
    ref.hasf_rest_19() ? addf_rest_19() : removef_rest_19();
    ref.hasf_rest_20() ? addf_rest_20() : removef_rest_20();
    ref.hasf_rest_21() ? addf_rest_21() : removef_rest_21();
    ref.hasf_rest_22() ? addf_rest_22() : removef_rest_22();
    ref.hasf_rest_23() ? addf_rest_23() : removef_rest_23();
    ref.hasf_rest_24() ? addf_rest_24() : removef_rest_24();
    ref.hasf_rest_25() ? addf_rest_25() : removef_rest_25();
    ref.hasf_rest_26() ? addf_rest_26() : removef_rest_26();
    ref.hasf_rest_27() ? addf_rest_27() : removef_rest_27();
    ref.hasf_rest_28() ? addf_rest_28() : removef_rest_28();
    ref.hasf_rest_29() ? addf_rest_29() : removef_rest_29();
    ref.hasf_rest_30() ? addf_rest_30() : removef_rest_30();
    ref.hasf_rest_31() ? addf_rest_31() : removef_rest_31();
    ref.hasf_rest_32() ? addf_rest_32() : removef_rest_32();
    ref.hasf_rest_33() ? addf_rest_33() : removef_rest_33();
    ref.hasf_rest_34() ? addf_rest_34() : removef_rest_34();
    ref.hasf_rest_35() ? addf_rest_35() : removef_rest_35();
    ref.hasf_rest_36() ? addf_rest_36() : removef_rest_36();
    ref.hasf_rest_37() ? addf_rest_37() : removef_rest_37();
    ref.hasf_rest_38() ? addf_rest_38() : removef_rest_38();
    ref.hasf_rest_39() ? addf_rest_39() : removef_rest_39();
    ref.hasf_rest_40() ? addf_rest_40() : removef_rest_40();
    ref.hasf_rest_41() ? addf_rest_41() : removef_rest_41();
    ref.hasf_rest_42() ? addf_rest_42() : removef_rest_42();
    ref.hasf_rest_43() ? addf_rest_43() : removef_rest_43();
    ref.hasf_rest_44() ? addf_rest_44() : removef_rest_44();

    ref.hasLaserAngles() ? addLaserAngles() : removeLaserAngles();
  }

  size_t getPointCount() const { return positions.size(); }
  void resize(const size_t size)
  {
    positions.resize(size);
    if (hasColors()) {
      colors.resize(size);
    }
    if (hasReflectances()) {
      reflectances.resize(size);
    }
    if (hasOpacities()) {
      opacities.resize(size);
    }
    if (hasf_dc_0()) {
      f_dc_0.resize(size);
    }
    if (hasf_dc_1()) {
      f_dc_1.resize(size);
    }
    if (hasf_dc_2()) {
      f_dc_2.resize(size);
    }
    if (hasscale_0()) {
      scale_0.resize(size);
    }
    if (hasscale_1()) {
      scale_1.resize(size);
    }
    if (hasscale_2()) {
      scale_2.resize(size);
    }
    if (hasrot_0()) {
      rot_0.resize(size);
    }
    if (hasrot_1()) {
      rot_1.resize(size);
    }
    if (hasrot_2()) {
      rot_2.resize(size);
    }
    if (hasrot_3()) {
      rot_3.resize(size);
    }
    if (hasf_rest_0()) {
      f_rest_0.resize(size);
    }
    if (hasf_rest_1()) {
      f_rest_1.resize(size);
    }
    if (hasf_rest_2()) {
      f_rest_2.resize(size);
    }
    if (hasf_rest_3()) {
      f_rest_3.resize(size);
    }
    if (hasf_rest_4()) {
      f_rest_4.resize(size);
    }
    if (hasf_rest_5()) {
      f_rest_5.resize(size);
    }
    if (hasf_rest_6()) {
      f_rest_6.resize(size);
    }
    if (hasf_rest_7()) {
      f_rest_7.resize(size);
    }
    if (hasf_rest_8()) {
      f_rest_8.resize(size);
    }
    if (hasf_rest_9()) {
      f_rest_9.resize(size);
    }
    if (hasf_rest_10()) {
      f_rest_10.resize(size);
    }
    if (hasf_rest_11()) {
      f_rest_11.resize(size);
    }
    if (hasf_rest_12()) {
      f_rest_12.resize(size);
    }
    if (hasf_rest_13()) {
      f_rest_13.resize(size);
    }
    if (hasf_rest_14()) {
      f_rest_14.resize(size);
    }
    if (hasf_rest_15()) {
      f_rest_15.resize(size);
    }
    if (hasf_rest_16()) {
      f_rest_16.resize(size);
    }
    if (hasf_rest_17()) {
      f_rest_17.resize(size);
    }
    if (hasf_rest_18()) {
      f_rest_18.resize(size);
    }
    if (hasf_rest_19()) {
      f_rest_19.resize(size);
    }
    if (hasf_rest_20()) {
      f_rest_20.resize(size);
    }
    if (hasf_rest_21()) {
      f_rest_21.resize(size);
    }
    if (hasf_rest_22()) {
      f_rest_22.resize(size);
    }
    if (hasf_rest_23()) {
      f_rest_23.resize(size);
    }
    if (hasf_rest_24()) {
      f_rest_24.resize(size);
    }
    if (hasf_rest_25()) {
      f_rest_25.resize(size);
    }
    if (hasf_rest_26()) {
      f_rest_26.resize(size);
    }
    if (hasf_rest_27()) {
      f_rest_27.resize(size);
    }
    if (hasf_rest_28()) {
      f_rest_28.resize(size);
    }
    if (hasf_rest_29()) {
      f_rest_29.resize(size);
    }
    if (hasf_rest_30()) {
      f_rest_30.resize(size);
    }
    if (hasf_rest_31()) {
      f_rest_31.resize(size);
    }
    if (hasf_rest_32()) {
      f_rest_32.resize(size);
    }
    if (hasf_rest_33()) {
      f_rest_33.resize(size);
    }
    if (hasf_rest_34()) {
      f_rest_34.resize(size);
    }
    if (hasf_rest_35()) {
      f_rest_35.resize(size);
    }
    if (hasf_rest_36()) {
      f_rest_36.resize(size);
    }
    if (hasf_rest_37()) {
      f_rest_37.resize(size);
    }
    if (hasf_rest_38()) {
      f_rest_38.resize(size);
    }
    if (hasf_rest_39()) {
      f_rest_39.resize(size);
    }
    if (hasf_rest_40()) {
      f_rest_40.resize(size);
    }
    if (hasf_rest_41()) {
      f_rest_41.resize(size);
    }
    if (hasf_rest_42()) {
      f_rest_42.resize(size);
    }
    if (hasf_rest_43()) {
      f_rest_43.resize(size);
    }
    if (hasf_rest_44()) {
      f_rest_44.resize(size);
    }

    if (hasFrameIndex()) {
      frameidx.resize(size);
    }
    if (hasLaserAngles()) {
      laserAngles.resize(size);
    }
  }

  void reserve(const size_t size)
  {
    positions.reserve(size);
    if (hasColors()) {
      colors.reserve(size);
    }
    if (hasReflectances()) {
      reflectances.reserve(size);
    }
    if (hasOpacities()) {
      opacities.reserve(size);
    }
    if (hasf_dc_0()) {
      f_dc_0.reserve(size);
    }
    if (hasf_dc_1()) {
      f_dc_1.reserve(size);
    }
    if (hasf_dc_2()) {
      f_dc_2.reserve(size);
    }
    if (hasscale_0()) {
      scale_0.reserve(size);
    }
    if (hasscale_1()) {
      scale_1.reserve(size);
    }
    if (hasscale_2()) {
      scale_2.reserve(size);
    }
    if (hasrot_0()) {
      rot_0.reserve(size);
    }
    if (hasrot_1()) {
      rot_1.reserve(size);
    }
    if (hasrot_2()) {
      rot_2.reserve(size);
    }
    if (hasrot_3()) {
      rot_3.reserve(size);
    }
    if (hasf_rest_0()) {
      f_rest_0.reserve(size);
    }
    if (hasf_rest_1()) {
      f_rest_1.reserve(size);
    }
    if (hasf_rest_2()) {
      f_rest_2.reserve(size);
    }
    if (hasf_rest_3()) {
      f_rest_3.reserve(size);
    }
    if (hasf_rest_4()) {
      f_rest_4.reserve(size);
    }
    if (hasf_rest_5()) {
      f_rest_5.reserve(size);
    }
    if (hasf_rest_6()) {
      f_rest_6.reserve(size);
    }
    if (hasf_rest_7()) {
      f_rest_7.reserve(size);
    }
    if (hasf_rest_8()) {
      f_rest_8.reserve(size);
    }
    if (hasf_rest_9()) {
      f_rest_9.reserve(size);
    }
    if (hasf_rest_10()) {
      f_rest_10.reserve(size);
    }
    if (hasf_rest_11()) {
      f_rest_11.reserve(size);
    }
    if (hasf_rest_12()) {
      f_rest_12.reserve(size);
    }
    if (hasf_rest_13()) {
      f_rest_13.reserve(size);
    }
    if (hasf_rest_14()) {
      f_rest_14.reserve(size);
    }
    if (hasf_rest_15()) {
      f_rest_15.reserve(size);
    }
    if (hasf_rest_16()) {
      f_rest_16.reserve(size);
    }
    if (hasf_rest_17()) {
      f_rest_17.reserve(size);
    }
    if (hasf_rest_18()) {
      f_rest_18.reserve(size);
    }
    if (hasf_rest_19()) {
      f_rest_19.reserve(size);
    }
    if (hasf_rest_20()) {
      f_rest_20.reserve(size);
    }
    if (hasf_rest_21()) {
      f_rest_21.reserve(size);
    }
    if (hasf_rest_22()) {
      f_rest_22.reserve(size);
    }
    if (hasf_rest_23()) {
      f_rest_23.reserve(size);
    }
    if (hasf_rest_24()) {
      f_rest_24.reserve(size);
    }
    if (hasf_rest_25()) {
      f_rest_25.reserve(size);
    }
    if (hasf_rest_26()) {
      f_rest_26.reserve(size);
    }
    if (hasf_rest_27()) {
      f_rest_27.reserve(size);
    }
    if (hasf_rest_28()) {
      f_rest_28.reserve(size);
    }
    if (hasf_rest_29()) {
      f_rest_29.reserve(size);
    }
    if (hasf_rest_30()) {
      f_rest_30.reserve(size);
    }
    if (hasf_rest_31()) {
      f_rest_31.reserve(size);
    }
    if (hasf_rest_32()) {
      f_rest_32.reserve(size);
    }
    if (hasf_rest_33()) {
      f_rest_33.reserve(size);
    }
    if (hasf_rest_34()) {
      f_rest_34.reserve(size);
    }
    if (hasf_rest_35()) {
      f_rest_35.reserve(size);
    }
    if (hasf_rest_36()) {
      f_rest_36.reserve(size);
    }
    if (hasf_rest_37()) {
      f_rest_37.reserve(size);
    }
    if (hasf_rest_38()) {
      f_rest_38.reserve(size);
    }
    if (hasf_rest_39()) {
      f_rest_39.reserve(size);
    }
    if (hasf_rest_40()) {
      f_rest_40.reserve(size);
    }
    if (hasf_rest_41()) {
      f_rest_41.reserve(size);
    }
    if (hasf_rest_42()) {
      f_rest_42.reserve(size);
    }
    if (hasf_rest_43()) {
      f_rest_43.reserve(size);
    }
    if (hasf_rest_44()) {
      f_rest_44.reserve(size);
    }

    if (hasFrameIndex()) {
      frameidx.reserve(size);
    }
    if (hasLaserAngles()) {
      laserAngles.reserve(size);
    }
  }
  void clear()
  {
    positions.clear();
    colors.clear();
    reflectances.clear();
    opacities.clear();
    f_dc_0.clear();
    f_dc_1.clear();
    f_dc_2.clear();
    scale_0.clear();
    scale_1.clear();
    scale_2.clear();
    rot_0.clear();
    rot_1.clear();
    rot_2.clear();
    rot_3.clear();
    f_rest_0.clear();
    f_rest_1.clear();
    f_rest_2.clear();
    f_rest_3.clear();
    f_rest_4.clear();
    f_rest_5.clear();
    f_rest_6.clear();
    f_rest_7.clear();
    f_rest_8.clear();
    f_rest_9.clear();
    f_rest_10.clear();
    f_rest_11.clear();
    f_rest_12.clear();
    f_rest_13.clear();
    f_rest_14.clear();
    f_rest_15.clear();
    f_rest_16.clear();
    f_rest_17.clear();
    f_rest_18.clear();
    f_rest_19.clear();
    f_rest_20.clear();
    f_rest_21.clear();
    f_rest_22.clear();
    f_rest_23.clear();
    f_rest_24.clear();
    f_rest_25.clear();
    f_rest_26.clear();
    f_rest_27.clear();
    f_rest_28.clear();
    f_rest_29.clear();
    f_rest_30.clear();
    f_rest_31.clear();
    f_rest_32.clear();
    f_rest_33.clear();
    f_rest_34.clear();
    f_rest_35.clear();
    f_rest_36.clear();
    f_rest_37.clear();
    f_rest_38.clear();
    f_rest_39.clear();
    f_rest_40.clear();
    f_rest_41.clear();
    f_rest_42.clear();
    f_rest_43.clear();
    f_rest_44.clear();

    frameidx.clear();
    laserAngles.clear();
  }

  size_t removeDuplicatePointInQuantizedPoint(int minGeomNodeSizeLog2)
  {
    for (int i = 0; i < positions.size(); i++) {
      PointType newPoint = positions[i];
      if (minGeomNodeSizeLog2 > 0) {
        uint32_t mask = ((uint32_t)-1) << minGeomNodeSizeLog2;
        positions[i].x() = ((int32_t)(positions[i].x()) & mask);
        positions[i].y() = ((int32_t)(positions[i].y()) & mask);
        positions[i].z() = ((int32_t)(positions[i].z()) & mask);
      }
    }
    positions.erase(
      std::unique(positions.begin(), positions.end()), positions.end());

    return positions.size();
  }

  void append(const PCCPointSet3& src)
  {
    if (!getPointCount())
      addRemoveAttributes(src);

    int dstEnd = positions.size();
    int srcSize = src.positions.size();
    resize(dstEnd + srcSize);

    std::copy(
      src.positions.begin(), src.positions.end(),
      std::next(positions.begin(), dstEnd));

    if (hasColors() && src.hasColors())
      std::copy(
        src.colors.begin(), src.colors.end(),
        std::next(colors.begin(), dstEnd));

    if (hasReflectances() && src.hasReflectances())
      std::copy(
        src.reflectances.begin(), src.reflectances.end(),
        std::next(reflectances.begin(), dstEnd));

    if (hasOpacities() && src.hasOpacities())
      std::copy(
        src.opacities.begin(), src.opacities.end(),
        std::next(opacities.begin(), dstEnd));

    if (hasf_dc_0() && src.hasf_dc_0())
      std::copy(
        src.f_dc_0.begin(), src.f_dc_0.end(),
        std::next(f_dc_0.begin(), dstEnd));

    if (hasf_dc_1() && src.hasf_dc_1())
      std::copy(
        src.f_dc_1.begin(), src.f_dc_1.end(),
        std::next(f_dc_1.begin(), dstEnd));

    if (hasf_dc_2() && src.hasf_dc_2())
      std::copy(
        src.f_dc_2.begin(), src.f_dc_2.end(),
        std::next(f_dc_2.begin(), dstEnd));

    if (hasscale_0() && src.hasscale_0())
      std::copy(
        src.scale_0.begin(), src.scale_0.end(),
        std::next(scale_0.begin(), dstEnd));

    if (hasscale_1() && src.hasscale_1())
      std::copy(
        src.scale_1.begin(), src.scale_1.end(),
        std::next(scale_1.begin(), dstEnd));

    if (hasscale_2() && src.hasscale_2())
      std::copy(
        src.scale_2.begin(), src.scale_2.end(),
        std::next(scale_2.begin(), dstEnd));

    if (hasrot_0() && src.hasrot_0())
      std::copy(
        src.rot_0.begin(), src.rot_0.end(), std::next(rot_0.begin(), dstEnd));
    if (hasrot_1() && src.hasrot_1())
      std::copy(
        src.rot_1.begin(), src.rot_1.end(), std::next(rot_1.begin(), dstEnd));
    if (hasrot_2() && src.hasrot_2())
      std::copy(
        src.rot_2.begin(), src.rot_2.end(), std::next(rot_2.begin(), dstEnd));
    if (hasrot_3() && src.hasrot_3())
      std::copy(
        src.rot_3.begin(), src.rot_3.end(), std::next(rot_3.begin(), dstEnd));
    if (hasf_rest_0() && src.hasf_rest_0()) {
      std::copy(
        src.f_rest_0.begin(), src.f_rest_0.end(),
        std::next(f_rest_0.begin(), dstEnd));
    }
    if (hasf_rest_1() && src.hasf_rest_1()) {
      std::copy(
        src.f_rest_1.begin(), src.f_rest_1.end(),
        std::next(f_rest_1.begin(), dstEnd));
    }
    if (hasf_rest_2() && src.hasf_rest_2()) {
      std::copy(
        src.f_rest_2.begin(), src.f_rest_2.end(),
        std::next(f_rest_2.begin(), dstEnd));
    }
    if (hasf_rest_3() && src.hasf_rest_3()) {
      std::copy(
        src.f_rest_3.begin(), src.f_rest_3.end(),
        std::next(f_rest_3.begin(), dstEnd));
    }
    if (hasf_rest_4() && src.hasf_rest_4()) {
      std::copy(
        src.f_rest_4.begin(), src.f_rest_4.end(),
        std::next(f_rest_4.begin(), dstEnd));
    }
    if (hasf_rest_5() && src.hasf_rest_5()) {
      std::copy(
        src.f_rest_5.begin(), src.f_rest_5.end(),
        std::next(f_rest_5.begin(), dstEnd));
    }
    if (hasf_rest_6() && src.hasf_rest_6()) {
      std::copy(
        src.f_rest_6.begin(), src.f_rest_6.end(),
        std::next(f_rest_6.begin(), dstEnd));
    }
    if (hasf_rest_7() && src.hasf_rest_7()) {
      std::copy(
        src.f_rest_7.begin(), src.f_rest_7.end(),
        std::next(f_rest_7.begin(), dstEnd));
    }
    if (hasf_rest_8() && src.hasf_rest_8()) {
      std::copy(
        src.f_rest_8.begin(), src.f_rest_8.end(),
        std::next(f_rest_8.begin(), dstEnd));
    }
    if (hasf_rest_9() && src.hasf_rest_9()) {
      std::copy(
        src.f_rest_9.begin(), src.f_rest_9.end(),
        std::next(f_rest_9.begin(), dstEnd));
    }
    if (hasf_rest_10() && src.hasf_rest_10()) {
      std::copy(
        src.f_rest_10.begin(), src.f_rest_10.end(),
        std::next(f_rest_10.begin(), dstEnd));
    }
    if (hasf_rest_11() && src.hasf_rest_11()) {
      std::copy(
        src.f_rest_11.begin(), src.f_rest_11.end(),
        std::next(f_rest_11.begin(), dstEnd));
    }
    if (hasf_rest_12() && src.hasf_rest_12()) {
      std::copy(
        src.f_rest_12.begin(), src.f_rest_12.end(),
        std::next(f_rest_12.begin(), dstEnd));
    }
    if (hasf_rest_13() && src.hasf_rest_13()) {
      std::copy(
        src.f_rest_13.begin(), src.f_rest_13.end(),
        std::next(f_rest_13.begin(), dstEnd));
    }
    if (hasf_rest_14() && src.hasf_rest_14()) {
      std::copy(
        src.f_rest_14.begin(), src.f_rest_14.end(),
        std::next(f_rest_14.begin(), dstEnd));
    }
    if (hasf_rest_15() && src.hasf_rest_15()) {
      std::copy(
        src.f_rest_15.begin(), src.f_rest_15.end(),
        std::next(f_rest_15.begin(), dstEnd));
    }
    if (hasf_rest_16() && src.hasf_rest_16()) {
      std::copy(
        src.f_rest_16.begin(), src.f_rest_16.end(),
        std::next(f_rest_16.begin(), dstEnd));
    }
    if (hasf_rest_17() && src.hasf_rest_17()) {
      std::copy(
        src.f_rest_17.begin(), src.f_rest_17.end(),
        std::next(f_rest_17.begin(), dstEnd));
    }
    if (hasf_rest_18() && src.hasf_rest_18()) {
      std::copy(
        src.f_rest_18.begin(), src.f_rest_18.end(),
        std::next(f_rest_18.begin(), dstEnd));
    }
    if (hasf_rest_19() && src.hasf_rest_19()) {
      std::copy(
        src.f_rest_19.begin(), src.f_rest_19.end(),
        std::next(f_rest_19.begin(), dstEnd));
    }
    if (hasf_rest_20() && src.hasf_rest_20()) {
      std::copy(
        src.f_rest_20.begin(), src.f_rest_20.end(),
        std::next(f_rest_20.begin(), dstEnd));
    }
    if (hasf_rest_21() && src.hasf_rest_21()) {
      std::copy(
        src.f_rest_21.begin(), src.f_rest_21.end(),
        std::next(f_rest_21.begin(), dstEnd));
    }
    if (hasf_rest_22() && src.hasf_rest_22()) {
      std::copy(
        src.f_rest_22.begin(), src.f_rest_22.end(),
        std::next(f_rest_22.begin(), dstEnd));
    }
    if (hasf_rest_23() && src.hasf_rest_23()) {
      std::copy(
        src.f_rest_23.begin(), src.f_rest_23.end(),
        std::next(f_rest_23.begin(), dstEnd));
    }
    if (hasf_rest_24() && src.hasf_rest_24()) {
      std::copy(
        src.f_rest_24.begin(), src.f_rest_24.end(),
        std::next(f_rest_24.begin(), dstEnd));
    }
    if (hasf_rest_25() && src.hasf_rest_25()) {
      std::copy(
        src.f_rest_25.begin(), src.f_rest_25.end(),
        std::next(f_rest_25.begin(), dstEnd));
    }
    if (hasf_rest_26() && src.hasf_rest_26()) {
      std::copy(
        src.f_rest_26.begin(), src.f_rest_26.end(),
        std::next(f_rest_26.begin(), dstEnd));
    }
    if (hasf_rest_27() && src.hasf_rest_27()) {
      std::copy(
        src.f_rest_27.begin(), src.f_rest_27.end(),
        std::next(f_rest_27.begin(), dstEnd));
    }
    if (hasf_rest_28() && src.hasf_rest_28()) {
      std::copy(
        src.f_rest_28.begin(), src.f_rest_28.end(),
        std::next(f_rest_28.begin(), dstEnd));
    }
    if (hasf_rest_29() && src.hasf_rest_29()) {
      std::copy(
        src.f_rest_29.begin(), src.f_rest_29.end(),
        std::next(f_rest_29.begin(), dstEnd));
    }
    if (hasf_rest_30() && src.hasf_rest_30()) {
      std::copy(
        src.f_rest_30.begin(), src.f_rest_30.end(),
        std::next(f_rest_30.begin(), dstEnd));
    }
    if (hasf_rest_31() && src.hasf_rest_31()) {
      std::copy(
        src.f_rest_31.begin(), src.f_rest_31.end(),
        std::next(f_rest_31.begin(), dstEnd));
    }
    if (hasf_rest_32() && src.hasf_rest_32()) {
      std::copy(
        src.f_rest_32.begin(), src.f_rest_32.end(),
        std::next(f_rest_32.begin(), dstEnd));
    }
    if (hasf_rest_33() && src.hasf_rest_33()) {
      std::copy(
        src.f_rest_33.begin(), src.f_rest_33.end(),
        std::next(f_rest_33.begin(), dstEnd));
    }
    if (hasf_rest_34() && src.hasf_rest_34()) {
      std::copy(
        src.f_rest_34.begin(), src.f_rest_34.end(),
        std::next(f_rest_34.begin(), dstEnd));
    }
    if (hasf_rest_35() && src.hasf_rest_35()) {
      std::copy(
        src.f_rest_35.begin(), src.f_rest_35.end(),
        std::next(f_rest_35.begin(), dstEnd));
    }
    if (hasf_rest_36() && src.hasf_rest_36()) {
      std::copy(
        src.f_rest_36.begin(), src.f_rest_36.end(),
        std::next(f_rest_36.begin(), dstEnd));
    }
    if (hasf_rest_37() && src.hasf_rest_37()) {
      std::copy(
        src.f_rest_37.begin(), src.f_rest_37.end(),
        std::next(f_rest_37.begin(), dstEnd));
    }
    if (hasf_rest_38() && src.hasf_rest_38()) {
      std::copy(
        src.f_rest_38.begin(), src.f_rest_38.end(),
        std::next(f_rest_38.begin(), dstEnd));
    }
    if (hasf_rest_39() && src.hasf_rest_39()) {
      std::copy(
        src.f_rest_39.begin(), src.f_rest_39.end(),
        std::next(f_rest_39.begin(), dstEnd));
    }
    if (hasf_rest_40() && src.hasf_rest_40()) {
      std::copy(
        src.f_rest_40.begin(), src.f_rest_40.end(),
        std::next(f_rest_40.begin(), dstEnd));
    }
    if (hasf_rest_41() && src.hasf_rest_41()) {
      std::copy(
        src.f_rest_41.begin(), src.f_rest_41.end(),
        std::next(f_rest_41.begin(), dstEnd));
    }
    if (hasf_rest_42() && src.hasf_rest_42()) {
      std::copy(
        src.f_rest_42.begin(), src.f_rest_42.end(),
        std::next(f_rest_42.begin(), dstEnd));
    }
    if (hasf_rest_43() && src.hasf_rest_43()) {
      std::copy(
        src.f_rest_43.begin(), src.f_rest_43.end(),
        std::next(f_rest_43.begin(), dstEnd));
    }
    if (hasf_rest_44() && src.hasf_rest_44()) {
      std::copy(
        src.f_rest_44.begin(), src.f_rest_44.end(),
        std::next(f_rest_44.begin(), dstEnd));
    }

    if (hasLaserAngles())
      std::copy(
        src.laserAngles.begin(), src.laserAngles.end(),
        std::next(laserAngles.begin(), dstEnd));
  }

  void swapPoints(const size_t index1, const size_t index2)
  {
    assert(index1 < getPointCount());
    assert(index2 < getPointCount());
    std::swap((*this)[index1], (*this)[index2]);
    if (hasColors()) {
      std::swap(getColor(index1), getColor(index2));
    }
    if (hasReflectances()) {
      std::swap(getReflectance(index1), getReflectance(index2));
    }
    if (hasOpacities()) {
      std::swap(getOpacity(index1), getOpacity(index2));
    }
    if (hasf_dc_0()) {
      std::swap(getf_dc_0(index1), getf_dc_0(index2));
    }
    if (hasf_dc_1()) {
      std::swap(getf_dc_1(index1), getf_dc_1(index2));
    }
    if (hasf_dc_2()) {
      std::swap(getf_dc_2(index1), getf_dc_2(index2));
    }
    if (hasscale_0()) {
      std::swap(getscale_0(index1), getscale_0(index2));
    }
    if (hasscale_1()) {
      std::swap(getscale_1(index1), getscale_1(index2));
    }
    if (hasscale_2()) {
      std::swap(getscale_2(index1), getscale_2(index2));
    }
    if (hasrot_0()) {
      std::swap(getrot_0(index1), getrot_0(index2));
    }
    if (hasrot_1()) {
      std::swap(getrot_1(index1), getrot_1(index2));
    }
    if (hasrot_2()) {
      std::swap(getrot_2(index1), getrot_2(index2));
    }
    if (hasrot_3()) {
      std::swap(getrot_3(index1), getrot_3(index2));
    }
    if (hasf_rest_0()) {
      std::swap(getf_rest_0(index1), getf_rest_0(index2));
    }
    if (hasf_rest_1()) {
      std::swap(getf_rest_1(index1), getf_rest_1(index2));
    }
    if (hasf_rest_2()) {
      std::swap(getf_rest_2(index1), getf_rest_2(index2));
    }
    if (hasf_rest_3()) {
      std::swap(getf_rest_3(index1), getf_rest_3(index2));
    }
    if (hasf_rest_4()) {
      std::swap(getf_rest_4(index1), getf_rest_4(index2));
    }
    if (hasf_rest_5()) {
      std::swap(getf_rest_5(index1), getf_rest_5(index2));
    }
    if (hasf_rest_6()) {
      std::swap(getf_rest_6(index1), getf_rest_6(index2));
    }
    if (hasf_rest_7()) {
      std::swap(getf_rest_7(index1), getf_rest_7(index2));
    }
    if (hasf_rest_8()) {
      std::swap(getf_rest_8(index1), getf_rest_8(index2));
    }
    if (hasf_rest_9()) {
      std::swap(getf_rest_9(index1), getf_rest_9(index2));
    }
    if (hasf_rest_10()) {
      std::swap(getf_rest_10(index1), getf_rest_10(index2));
    }
    if (hasf_rest_11()) {
      std::swap(getf_rest_11(index1), getf_rest_11(index2));
    }
    if (hasf_rest_12()) {
      std::swap(getf_rest_12(index1), getf_rest_12(index2));
    }
    if (hasf_rest_13()) {
      std::swap(getf_rest_13(index1), getf_rest_13(index2));
    }
    if (hasf_rest_14()) {
      std::swap(getf_rest_14(index1), getf_rest_14(index2));
    }
    if (hasf_rest_15()) {
      std::swap(getf_rest_15(index1), getf_rest_15(index2));
    }
    if (hasf_rest_16()) {
      std::swap(getf_rest_16(index1), getf_rest_16(index2));
    }
    if (hasf_rest_17()) {
      std::swap(getf_rest_17(index1), getf_rest_17(index2));
    }
    if (hasf_rest_18()) {
      std::swap(getf_rest_18(index1), getf_rest_18(index2));
    }
    if (hasf_rest_19()) {
      std::swap(getf_rest_19(index1), getf_rest_19(index2));
    }
    if (hasf_rest_20()) {
      std::swap(getf_rest_20(index1), getf_rest_20(index2));
    }
    if (hasf_rest_21()) {
      std::swap(getf_rest_21(index1), getf_rest_21(index2));
    }
    if (hasf_rest_22()) {
      std::swap(getf_rest_22(index1), getf_rest_22(index2));
    }
    if (hasf_rest_23()) {
      std::swap(getf_rest_23(index1), getf_rest_23(index2));
    }
    if (hasf_rest_24()) {
      std::swap(getf_rest_24(index1), getf_rest_24(index2));
    }
    if (hasf_rest_25()) {
      std::swap(getf_rest_25(index1), getf_rest_25(index2));
    }
    if (hasf_rest_26()) {
      std::swap(getf_rest_26(index1), getf_rest_26(index2));
    }
    if (hasf_rest_27()) {
      std::swap(getf_rest_27(index1), getf_rest_27(index2));
    }
    if (hasf_rest_28()) {
      std::swap(getf_rest_28(index1), getf_rest_28(index2));
    }
    if (hasf_rest_29()) {
      std::swap(getf_rest_29(index1), getf_rest_29(index2));
    }
    if (hasf_rest_30()) {
      std::swap(getf_rest_30(index1), getf_rest_30(index2));
    }
    if (hasf_rest_31()) {
      std::swap(getf_rest_31(index1), getf_rest_31(index2));
    }
    if (hasf_rest_32()) {
      std::swap(getf_rest_32(index1), getf_rest_32(index2));
    }
    if (hasf_rest_33()) {
      std::swap(getf_rest_33(index1), getf_rest_33(index2));
    }
    if (hasf_rest_34()) {
      std::swap(getf_rest_34(index1), getf_rest_34(index2));
    }
    if (hasf_rest_35()) {
      std::swap(getf_rest_35(index1), getf_rest_35(index2));
    }
    if (hasf_rest_36()) {
      std::swap(getf_rest_36(index1), getf_rest_36(index2));
    }
    if (hasf_rest_37()) {
      std::swap(getf_rest_37(index1), getf_rest_37(index2));
    }
    if (hasf_rest_38()) {
      std::swap(getf_rest_38(index1), getf_rest_38(index2));
    }
    if (hasf_rest_39()) {
      std::swap(getf_rest_39(index1), getf_rest_39(index2));
    }
    if (hasf_rest_40()) {
      std::swap(getf_rest_40(index1), getf_rest_40(index2));
    }
    if (hasf_rest_41()) {
      std::swap(getf_rest_41(index1), getf_rest_41(index2));
    }
    if (hasf_rest_42()) {
      std::swap(getf_rest_42(index1), getf_rest_42(index2));
    }
    if (hasf_rest_43()) {
      std::swap(getf_rest_43(index1), getf_rest_43(index2));
    }
    if (hasf_rest_44()) {
      std::swap(getf_rest_44(index1), getf_rest_44(index2));
    }

    if (hasLaserAngles()) {
      std::swap(getLaserAngle(index1), getLaserAngle(index2));
    }
  }

  Box3<int32_t> computeBoundingBox() const
  {
    Box3<int32_t> bbox(
      std::numeric_limits<int32_t>::max(),
      std::numeric_limits<int32_t>::lowest());
    const size_t pointCount = getPointCount();
    for (size_t i = 0; i < pointCount; ++i) {
      const auto& pt = (*this)[i];
      for (int k = 0; k < 3; ++k) {
        if (pt[k] > bbox.max[k]) {
          bbox.max[k] = pt[k];
        }
        if (pt[k] < bbox.min[k]) {
          bbox.min[k] = pt[k];
        }
      }
    }
    return bbox;
  }

  void shiftPointPositions(const pcc::point_t shiftVal)
  {
    const size_t pointCount = getPointCount();
    for (size_t i = 0; i < pointCount; ++i)
      positions[i] += shiftVal;
  }

  //--------------------------------------------------------------------------
  // Determine the bounding box of the set of points given by the indicies
  // given by iterating over [begin, end)

  template<typename ForwardIt>
  Box3<int32_t> computeBoundingBox(ForwardIt begin, ForwardIt end) const
  {
    Box3<int32_t> bbox(
      std::numeric_limits<int32_t>::max(),
      std::numeric_limits<int32_t>::lowest());

    for (auto it = begin; it != end; ++it) {
      int i = *it;
      const auto& pt = (*this)[i];
      for (int k = 0; k < 3; ++k) {
        if (pt[k] > bbox.max[k]) {
          bbox.max[k] = pt[k];
        }
        if (pt[k] < bbox.min[k]) {
          bbox.min[k] = pt[k];
        }
      }
    }
    return bbox;
  }

  //--------------------------------------------------------------------------

public:
  std::vector<PointType> positions;
  std::vector<Vec3<attr_t>> colors;
  std::vector<attr_t> reflectances;
  std::vector<attr_t> f_dc_0;
  std::vector<attr_t> f_dc_1;
  std::vector<attr_t> f_dc_2;
  std::vector<attr_t> opacities;
  std::vector<attr_t> scale_0;
  std::vector<attr_t> scale_1;
  std::vector<attr_t> scale_2;
  std::vector<attr_t> rot_0;
  std::vector<attr_t> rot_1;
  std::vector<attr_t> rot_2;
  std::vector<attr_t> rot_3;
  std::vector<attr_t> f_rest_0;
  std::vector<attr_t> f_rest_1;
  std::vector<attr_t> f_rest_2;
  std::vector<attr_t> f_rest_3;
  std::vector<attr_t> f_rest_4;
  std::vector<attr_t> f_rest_5;
  std::vector<attr_t> f_rest_6;
  std::vector<attr_t> f_rest_7;
  std::vector<attr_t> f_rest_8;
  std::vector<attr_t> f_rest_9;
  std::vector<attr_t> f_rest_10;
  std::vector<attr_t> f_rest_11;
  std::vector<attr_t> f_rest_12;
  std::vector<attr_t> f_rest_13;
  std::vector<attr_t> f_rest_14;
  std::vector<attr_t> f_rest_15;
  std::vector<attr_t> f_rest_16;
  std::vector<attr_t> f_rest_17;
  std::vector<attr_t> f_rest_18;
  std::vector<attr_t> f_rest_19;
  std::vector<attr_t> f_rest_20;
  std::vector<attr_t> f_rest_21;
  std::vector<attr_t> f_rest_22;
  std::vector<attr_t> f_rest_23;
  std::vector<attr_t> f_rest_24;
  std::vector<attr_t> f_rest_25;
  std::vector<attr_t> f_rest_26;
  std::vector<attr_t> f_rest_27;
  std::vector<attr_t> f_rest_28;
  std::vector<attr_t> f_rest_29;
  std::vector<attr_t> f_rest_30;
  std::vector<attr_t> f_rest_31;
  std::vector<attr_t> f_rest_32;
  std::vector<attr_t> f_rest_33;
  std::vector<attr_t> f_rest_34;
  std::vector<attr_t> f_rest_35;
  std::vector<attr_t> f_rest_36;
  std::vector<attr_t> f_rest_37;
  std::vector<attr_t> f_rest_38;
  std::vector<attr_t> f_rest_39;
  std::vector<attr_t> f_rest_40;
  std::vector<attr_t> f_rest_41;
  std::vector<attr_t> f_rest_42;
  std::vector<attr_t> f_rest_43;
  std::vector<attr_t> f_rest_44;

  std::vector<float> temp_f_dc_0;
  std::vector<float> temp_f_dc_1;
  std::vector<float> temp_f_dc_2;
  std::vector<float> temp_opacities;
  std::vector<float> temp_scale_0;
  std::vector<float> temp_scale_1;
  std::vector<float> temp_scale_2;
  std::vector<float> temp_rot_0;
  std::vector<float> temp_rot_1;
  std::vector<float> temp_rot_2;
  std::vector<float> temp_rot_3;
  std::vector<float> temp_f_rest_0;
  std::vector<float> temp_f_rest_1;
  std::vector<float> temp_f_rest_2;
  std::vector<float> temp_f_rest_3;
  std::vector<float> temp_f_rest_4;
  std::vector<float> temp_f_rest_5;
  std::vector<float> temp_f_rest_6;
  std::vector<float> temp_f_rest_7;
  std::vector<float> temp_f_rest_8;
  std::vector<float> temp_f_rest_9;
  std::vector<float> temp_f_rest_10;
  std::vector<float> temp_f_rest_11;
  std::vector<float> temp_f_rest_12;
  std::vector<float> temp_f_rest_13;
  std::vector<float> temp_f_rest_14;
  std::vector<float> temp_f_rest_15;
  std::vector<float> temp_f_rest_16;
  std::vector<float> temp_f_rest_17;
  std::vector<float> temp_f_rest_18;
  std::vector<float> temp_f_rest_19;
  std::vector<float> temp_f_rest_20;
  std::vector<float> temp_f_rest_21;
  std::vector<float> temp_f_rest_22;
  std::vector<float> temp_f_rest_23;
  std::vector<float> temp_f_rest_24;
  std::vector<float> temp_f_rest_25;
  std::vector<float> temp_f_rest_26;
  std::vector<float> temp_f_rest_27;
  std::vector<float> temp_f_rest_28;
  std::vector<float> temp_f_rest_29;
  std::vector<float> temp_f_rest_30;
  std::vector<float> temp_f_rest_31;
  std::vector<float> temp_f_rest_32;
  std::vector<float> temp_f_rest_33;
  std::vector<float> temp_f_rest_34;
  std::vector<float> temp_f_rest_35;
  std::vector<float> temp_f_rest_36;
  std::vector<float> temp_f_rest_37;
  std::vector<float> temp_f_rest_38;
  std::vector<float> temp_f_rest_39;
  std::vector<float> temp_f_rest_40;
  std::vector<float> temp_f_rest_41;
  std::vector<float> temp_f_rest_42;
  std::vector<float> temp_f_rest_43;
  std::vector<float> temp_f_rest_44;

private:
  std::vector<uint8_t> frameidx;
  bool withColors;
  bool withReflectances;
  bool withOpacity;
  bool withf_dc_0;
  bool withf_dc_1;
  bool withf_dc_2;
  bool withscale_0;
  bool withscale_1;
  bool withscale_2;
  bool withrot_0;
  bool withrot_1;
  bool withrot_2;
  bool withrot_3;
  bool withf_rest_0;
  bool withf_rest_1;
  bool withf_rest_2;
  bool withf_rest_3;
  bool withf_rest_4;
  bool withf_rest_5;
  bool withf_rest_6;
  bool withf_rest_7;
  bool withf_rest_8;
  bool withf_rest_9;
  bool withf_rest_10;
  bool withf_rest_11;
  bool withf_rest_12;
  bool withf_rest_13;
  bool withf_rest_14;
  bool withf_rest_15;
  bool withf_rest_16;
  bool withf_rest_17;
  bool withf_rest_18;
  bool withf_rest_19;
  bool withf_rest_20;
  bool withf_rest_21;
  bool withf_rest_22;
  bool withf_rest_23;
  bool withf_rest_24;
  bool withf_rest_25;
  bool withf_rest_26;
  bool withf_rest_27;
  bool withf_rest_28;
  bool withf_rest_29;
  bool withf_rest_30;
  bool withf_rest_31;
  bool withf_rest_32;
  bool withf_rest_33;
  bool withf_rest_34;
  bool withf_rest_35;
  bool withf_rest_36;
  bool withf_rest_37;
  bool withf_rest_38;
  bool withf_rest_39;
  bool withf_rest_40;
  bool withf_rest_41;
  bool withf_rest_42;
  bool withf_rest_43;
  bool withf_rest_44;

  bool withFrameIndex;
  std::vector<int> laserAngles;
  bool withLaserAngles;
};

//===========================================================================
// Swap the position of two points (including attributes) in the PointSet
// as referenced by the proxies a and b.

inline void
swap(const PCCPointSet3::Proxy& a, const PCCPointSet3::Proxy& b)
{
  a.swap(b);
}

//---------------------------------------------------------------------------
// Swap two point clouds

inline void
swap(PCCPointSet3& a, PCCPointSet3& b)
{
  a.swap(b);
}

//============================================================================

static inline int
findLaserPrecise(
  pcc::point_t point,
  const int* thetaList,
  const int* zList,
  const int numTheta)
{
  if (numTheta == 1)
    return 0;

  int64_t xLidar = int64_t(point[0]) << 8;
  int64_t yLidar = int64_t(point[1]) << 8;
  int64_t rInv = irsqrt(xLidar * xLidar + yLidar * yLidar);

  int lBest = 0;
  int dBest = std::numeric_limits<int>::max();

  for (int l = 0; l < numTheta; l++, thetaList++) {
    int zS3 = (point[2] << 3) + zList[l];
    int theta32 =
      zS3 >= 0 ? (zS3 * rInv) >> (14 + 3) : -((-zS3 * rInv) >> (14 + 3));
    int d = std::abs(theta32 - *thetaList);
    if (d < dBest) {
      dBest = d;
      lBest = l;
    }
  }

  return lBest;
}

//============================================================================

class AzimuthalPhiZi {
public:
  AzimuthalPhiZi(int numLasers, const std::vector<int>& numPhi)
    : _delta(numLasers), _invDelta(numLasers)
  {
    for (int laserIndex = 0; laserIndex < numLasers; laserIndex++) {
      constexpr int k2pi = 6588397;  // 2**20 * 2 * pi
      _delta[laserIndex] = k2pi / numPhi[laserIndex];
      _invDelta[laserIndex] =
        int64_t((int64_t(numPhi[laserIndex]) << 30) / k2pi);
    }
  }

  const int delta(size_t idx) const { return _delta[idx]; }
  const int64_t invDelta(size_t idx) const { return _invDelta[idx]; }

private:
  std::vector<int> _delta;
  std::vector<int64_t> _invDelta;
};

//============================================================================

} /* namespace pcc */

#endif /* PCCPointSet_h */
