// ----------------------------------------------------------------------------
// RigidSolver.hpp
//
//  Created on: 18 Dec 2020
//      Author: Kiwon Um
//        Mail: kiwon.um@telecom-paris.fr
//
// Description: Simple Rigid Body Solver (DO NOT DISTRIBUTE!)
//
// Copyright 2020 Kiwon Um
//
// The copyright to the computer program(s) herein is the property of Kiwon Um,
// Telecom Paris, France. The program(s) may be used and/or copied only with
// the written permission of Kiwon Um or in accordance with the terms and
// conditions stipulated in the agreement/contract under which the program(s)
// have been supplied.
// ----------------------------------------------------------------------------

#ifndef _RIGIDSOLVER_HPP_
#define _RIGIDSOLVER_HPP_

#define USE_Q                   // use quaternion instead of matrix for rotation

#include <glm/ext/matrix_transform.hpp>

#include "Vector3.hpp"
#include "Matrix3x3.hpp"
#ifdef USE_Q
#include "Quaternion.hpp"
#endif

struct BodyAttributes {
  BodyAttributes() :
    X(0, 0, 0), R(Mat3f::I()), P(0, 0, 0), L(0, 0, 0),
    V(0, 0, 0), omega(0, 0, 0), F(0, 0, 0), tau(0, 0, 0) {}

  Vec3f worldCoordOf(const tIndex vidx) const
  {
    return R*vdata0[vidx%vdata0.size()] + X;
  }

  glm::mat4 worldMat() const
  {
    return glm::mat4(           // column-major
      R(0,0), R(1,0), R(2,0), 0,
      R(0,1), R(1,1), R(2,1), 0,
      R(0,2), R(1,2), R(2,2), 0,
      X[0],   X[1],   X[2],   1);
  }

  Real M;                       // mass
  Mat3f I0, I0inv;              // inertia tensor and its inverse in body space

  // rigid body state
  Vec3f X;                      // position
  Mat3f R;                      // rotation
#ifdef USE_Q
  Quaternion Q;                 // rotation
#endif
  Vec3f P;                      // linear momentum
  Vec3f L;                      // angular momentum

  // auxiliary quantities
  Mat3f Iinv;                   // inverse of inertia tensor
  Vec3f V;                      // linear velocity
  Vec3f omega;                  // angular velocity

  // force and torque
  Vec3f F;                      // force
  Vec3f tau;                    // torque

  // mesh's vertices in body space
  std::vector<Vec3f> vdata0;
};

class Cylinder : public BodyAttributes {
public:
  explicit Cylinder(
    const Real h=2.0, const Real r=0.5,
    const Vec3f v0=Vec3f(0, 0, 0), const Vec3f omega0=Vec3f(0, 0, 0))
    : height(h), radius(r)
  {
    V = v0;
    omega = omega0;
    M = 3.14*r*r*h;
    I0 = Mat3f(
      M*(h*h + 3.0*r*r)/12.0, 0, 0,
      0, M*(h*h + 3.0*r*r)/12.0, 0,
      0, 0, M*r*r/2.0);
    I0inv = Mat3f(
      1.0/I0(0,0), 0, 0,
      0, 1.0/I0(1,1), 0,
      0, 0, 1.0/I0(2,2));
    Iinv = I0inv;

    // vertices data
    for(int i=0; i<10; ++i) {
      const Real rx = r*std::sin((static_cast<Real>(i)+0.5)*2.0*M_PI/10.0);
      const Real ry = r*std::cos((static_cast<Real>(i)+0.5)*2.0*M_PI/10.0);
      vdata0.push_back(Vec3f(rx, ry,  0.5*h));
      vdata0.push_back(Vec3f(rx, ry, -0.5*h));
    }
  }

  Real height, radius;
};


class Box : public BodyAttributes {
public:
  explicit Box(
    const Real w=1.0, const Real h=1.0, const Real d=1.0, const Real dens=10.0,
    const Vec3f v0=Vec3f(0, 0, 0), const Vec3f omega0=Vec3f(0, 0, 0)) :
    width(w), height(h), depth(d)
  {
    V = v0;                     // initial velocity
    omega = omega0;             // initial angular velocity
    M = w*h*d*dens;
    I0 = Mat3f(
      M*(h*h + d*d)/12.0, 0, 0,
      0, M*(w*w + d*d)/12.0, 0,
      0, 0, M*(w*w + h*h)/12.0);
    I0inv = Mat3f(
      1.0/I0(0,0), 0, 0,
      0, 1.0/I0(1,1), 0,
      0, 0, 1.0/I0(2,2));
    Iinv = I0inv;

    // vertices data (8 vertices)
    vdata0.push_back(Vec3f(-0.5*w, -0.5*h, -0.5*d));
    vdata0.push_back(Vec3f( 0.5*w, -0.5*h, -0.5*d));
    vdata0.push_back(Vec3f( 0.5*w,  0.5*h, -0.5*d));
    vdata0.push_back(Vec3f(-0.5*w,  0.5*h, -0.5*d));

    vdata0.push_back(Vec3f(-0.5*w, -0.5*h,  0.5*d));
    vdata0.push_back(Vec3f( 0.5*w, -0.5*h,  0.5*d));
    vdata0.push_back(Vec3f( 0.5*w,  0.5*h,  0.5*d));
    vdata0.push_back(Vec3f(-0.5*w,  0.5*h,  0.5*d));
  }

  // rigid body property
  Real width, height, depth;
};

class RigidSolver {
public:
  explicit RigidSolver(
    BodyAttributes *body0=nullptr, const Vec3f g=Vec3f(0, 0, 0)) :
    body(body0), _g(g), _step(0), _sim_t(0) {}

  void init(BodyAttributes *body0)
  {
    body = body0;
    _step = 0;
    _sim_t = 0;
  }

  void step(const Real dt)
  {
    std::cout << "t=" << _sim_t << " (dt=" << dt << ")" << std::endl;

    computeForceAndTorque();

    const Vec3f dxdt = body->V;
#ifdef USE_Q
    const Quaternion w(0, body->omega.x, body->omega.y, body->omega.z);
    const Quaternion dQdt = 0.5*w*body->Q;
#else
    const Mat3f dRdt = body->omega.crossProductMatrix()*body->R;
#endif
    const Vec3f dPdt = body->F;
    const Vec3f dLdt = body->tau;

    const Vec3f xnew = body->X + dt*dxdt;
#ifdef USE_Q
    const Quaternion Qnew = body->Q + dt*dQdt;
#else
    const Mat3f Rnew = body->R + dt*dRdt;
#endif
    const Vec3f Pnew = body->P + dt*dPdt;
    const Vec3f Lnew = body->L + dt*dLdt;

    body->X = xnew;
#ifdef USE_Q
    body->Q = Qnew.normalized();
    body->R = body->Q.matrix3x3();
#else
    body->R = Rnew;
#endif
    body->P = Pnew;
    body->L = Lnew;

    body->V = body->P/body->M;
    body->Iinv = body->R*body->I0inv*body->R.transposed();
    body->omega = body->Iinv*body->L;

    ++_step;
    _sim_t += dt;
  }

  BodyAttributes *body;

private:
  void computeForceAndTorque()
  {
    body->F = body->M*_g;
    body->tau = Vec3f(0, 0, 0);

    // Arbitrary force at the very first step
    if(_step==1) {
      const Vec3f fi(.15, .25, .03);
      body->F += fi;
      body->tau += (body->worldCoordOf(0) - body->X).crossProduct(fi);
    }
  }

  // simulation parameters
  Vec3f _g;                     // gravity
  tIndex _step;                 // step count
  Real _sim_t;                  // simulation time
};

#endif  /* _RIGIDSOLVER_HPP_ */
