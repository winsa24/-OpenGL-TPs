// ----------------------------------------------------------------------------
// Quaternion.hpp
//
//  Created on: 22 Dec 2020
//      Author: Kiwon Um
//        Mail: kiwon.um@telecom-paris.fr
//
// Description: Quaternion implementation (DO NOT DISTRIBUTE!)
//
// Copyright 2020 Kiwon Um
//
// The copyright to the computer program(s) herein is the property of Kiwon Um,
// Telecom Paris, France. The program(s) may be used and/or copied only with
// the written permission of Kiwon Um or in accordance with the terms and
// conditions stipulated in the agreement/contract under which the program(s)
// have been supplied.
// ----------------------------------------------------------------------------

#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include <cerrno>
#include <cmath>

#include "Matrix3x3.hpp"
#include "Vector3.hpp"

class Quaternion {
public:
  explicit Quaternion(
    const Real real=1, const Real qi=0, const Real qj=0, const Real qk=0)
    : r(real), i(qi), j(qj), k(qk) {}
  explicit Quaternion(const Mat3f &m) { initWithRotation(m); }
  Quaternion(const Vec3f &axis, const Real radian)
  {
    initWithRotation(axis, radian);
  }

  Quaternion& initWithRotation(const Vec3f &axis, const Real radian)
  {
    return initWithRotation(axis.x, axis.y, axis.z, radian);
  }
  Quaternion& initWithRotation(
    const Real ax, const Real ay, const Real az, const Real radian)
  {
    Vec3f v(Vec3f(ax, ay, az).normalized());
    const Real h_theta = 0.5 * radian;
    const Real sin_h_theta = std::sin(h_theta);
    r = std::cos(h_theta);
    i = v.x * sin_h_theta;
    j = v.y * sin_h_theta;
    k = v.z * sin_h_theta;
    return *this;
  }
  // refer to
  // http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
  Quaternion& initWithRotation(const Mat3f &m) // m: rotation matrix
  {
    const Real trace = m.v00 + m.v11 + m.v22;
    if(trace>0) {
      const Real s = 0.5/std::sqrt(trace+1.0);
      r = 0.25/s;
      i = (m.v21-m.v12)*s;
      j = (m.v02-m.v20)*s;
      k = (m.v10-m.v01)*s;
    } else {
      if (m.v00>m.v11 && m.v00>m.v22) {
        const Real s = 2.0*std::sqrt(1.0+m.v00-m.v11-m.v22);
        r = (m.v21-m.v12)/s;
        i = 0.25*s;
        j = (m.v01+m.v10)/s;
        k = (m.v02+m.v20)/s;
      } else if (m.v11 > m.v22) {
        const Real s = 2.0*std::sqrt(1.0+m.v11-m.v00-m.v22);
        r = (m.v02-m.v20)/s;
        i = (m.v01+m.v10)/s;
        j = 0.25*s;
        k = (m.v12+m.v21)/s;
      } else {
        const Real s = 2.0*std::sqrt(1.0+m.v22-m.v00-m.v11);
        r = (m.v10-m.v01)/s;
        i = (m.v02+m.v20)/s;
        j = (m.v12+m.v21)/s;
        k = 0.25*s;
      }
    }
    return *this;
  }

  Real rotationAngle() const
  {
    const Real arccos_r = std::acos(r); assert(errno!=EDOM);
    return 2e0*arccos_r;
  }
  Vec3f rotationAxis() const
  {
    if(r==1) return Vec3f(0e0);
    const Real d = std::sqrt(1e0-r*r); assert(errno!=EDOM);
    return Vec3f(i/d, j/d, k/d);}
  Vec3f eulerAngles() const
  {
    return Vec3f(
      std::atan2(2e0*(r*i+j*k), 1e0-2e0*(i*i+j*j)),
      std::asin(2e0*(r*j - k*i)),
      std::atan2(2e0*(r*k+i*j), 1e0-2e0*(j*j+k*k)));
  }

  Mat3f matrix3x3() const
  {
    const Real ii=i*i, jj=j*j, kk=k*k;
    const Real ij=i*j, ik=i*k, ir=i*r, jk=j*k, jr=j*r, kr=k*r;

    return Mat3f(
      1e0-2e0*(jj+kk), 2e0*(ij-kr),     2e0*(ik+jr),
      2e0*(ij+kr),     1e0-2e0*(ii+kk), 2e0*(jk-ir),
      2e0*(ik-jr),     2e0*(jk+ir),     1e0-2e0*(ii+jj));
  }
  // GL's matrix is in column major array
  bool getGLMatrix(Real *pMat16) const
  {
    if(!pMat16) return false;

    const Real ii=i*i, jj=j*j, kk=k*k;
    const Real ij=i*j, ik=i*k, ir=i*r, jk=j*k, jr=j*r, kr=k*r;

    // First col
    pMat16[0] = 1e0 - 2e0 * (jj + kk);
    pMat16[1] = 2e0 * (ij + kr);
    pMat16[2] = 2e0 * (ik - jr);
    pMat16[3] = 0;

    // Second col
    pMat16[4] = 2e0 * (ij - kr);
    pMat16[5] = 1e0 - 2e0 * (ii + kk);
    pMat16[6] = 2e0 * (jk + ir);
    pMat16[7] = 0;

    // Third col
    pMat16[8] = 2e0 * (ik + jr);
    pMat16[9] = 2e0 * (jk - ir);
    pMat16[10] = 1e0 - 2e0 * (ii + jj);
    pMat16[11] = 0;

    // Fourth col
    pMat16[12] = 0;
    pMat16[13] = 0;
    pMat16[14] = 0;
    pMat16[15] = 1e0;

    return true;
  }

  // comparison operators
  bool operator==(const Quaternion &q) const
  {
    return ((r==q.r) && (i==q.i) && (j==q.j) && (k==q.k));
  }
  bool operator!=(const Quaternion &q) const { return !(*this==q); }

  Quaternion conjugate() const { return Quaternion(r, -i, -j, -k); }

  Quaternion normalized() const
  {
    const Real d = std::sqrt(r*r+i*i+j*j+k*k); assert(d!=0);
    return Quaternion(r/d, i/d, j/d, k/d);
  }

  Quaternion operator*(const Quaternion &q) const
  {
    Quaternion qr;
    qr.r = r*q.r - i*q.i - j*q.j - k*q.k;
    qr.i = r*q.i + i*q.r + j*q.k - k*q.j;
    qr.j = r*q.j + j*q.r + k*q.i - i*q.k;
    qr.k = r*q.k + k*q.r + i*q.j - j*q.i;
    return qr;
  }
  Quaternion operator*(const Real s) const
  {
    return Quaternion(r*s, i*s, j*s, k*s);
  }
  Quaternion operator+(const Quaternion &q) const
  {
    return Quaternion(r+q.r, i+q.i, j+q.j, k+q.k);
  }

  // v' = qvq'
  Vec3f rotateVector(const Vec3f &v) const
  {
    const Quaternion vv = (*this)*Quaternion(0, v.x, v.y, v.z)*conjugate();
    return Vec3f(vv.i, vv.j, vv.k);
  }

  // v = q'v'q
  Vec3f inverseRotateVector(const Vec3f &v) const
  {
    const Quaternion vv = conjugate()*Quaternion(0, v.x, v.y, v.z)*(*this);
    return Vec3f(vv.i, vv.j, vv.k);
  }

  Real r, i, j, k;
};

inline Quaternion operator*(const Real s, const Quaternion &q)
{
  return q*s;
}

#endif  /* _QUATERNION_H_ */
