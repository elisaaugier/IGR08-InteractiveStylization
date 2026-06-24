//
// Created by jbcou on 15/06/2026.
//

#include "sdf.h"

#include <iostream>
#include <Eigen/src/Geometry/Homogeneous.h>
#include <Eigen/Geometry>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/read_triangle_mesh.h>

void Primitive::getMesh(Eigen::MatrixXd &V, Eigen::MatrixXi &F) {
  if (changed) {
    changed = false;
    const Eigen::MatrixXd TTot = (T * T0).transpose();
    Vc = (V0.rowwise().homogeneous() * TTot).rowwise().hnormalized();
  }
  F = F0;
  V = Vc;
}

Sphere::Sphere(double r, Eigen::RowVector3d c) {
  radius = r;
  center = c;
  Eigen::Affine3d t, s;
  t = Eigen::Translation3d(center);
  s = Eigen::Scaling(radius);
  T0 = (t * s).matrix();
  TNoScale = t.matrix();
  igl::read_triangle_mesh("../meshes/sphere2.obj", V0, F0);
  const Eigen::MatrixXd TTot = (T * T0).transpose();
  Vc = (V0.rowwise().homogeneous() * TTot).rowwise().hnormalized();
}

void Sphere::eval(Eigen::RowVector3d p, double &d, Eigen::RowVector3d &grad) {
  Eigen::Matrix4d Minv = (T * TNoScale).inverse();          // evaluate the inverse into a real Matrix4d
  p = (p.homogeneous() * Minv.transpose()).eval().hnormalized();
  //double l = (p - center).norm();
  double l = (p).norm();
  l = abs(l) < 1e-6 ? 1e-6 : l;
  d = l - radius;
  grad = (p - center) / l;
  Eigen::Matrix3d R = T.block<3,3>(0,0);
  grad = (R * grad.transpose()).transpose();
  grad.normalize();
}

Box::Box(Eigen::RowVector3d d, Eigen::RowVector3d c) {
  dim = d;
  center = c;
  Eigen::Affine3d t, s;
  t = Eigen::Translation3d(center);
  s = Eigen::Scaling(dim);
  T0 = (t * s).matrix();
  TNoScale = t.matrix();
  igl::read_triangle_mesh("../meshes/cube.obj", V0, F0);
  const Eigen::MatrixXd TTot = (T * T0).transpose();
  TNoScale = t.matrix();
  Vc = (V0.rowwise().homogeneous() * TTot).rowwise().hnormalized();
}

void Box::eval(Eigen::RowVector3d p, double &d, Eigen::RowVector3d &grad) {
  // Care with scaling need to change
  Eigen::Matrix4d Minv = (T * TNoScale).inverse();          // evaluate the inverse into a real Matrix4d
  p = (p.homogeneous() * Minv.transpose()).eval().hnormalized();
  //p = p - center;
  Eigen::RowVector3d s(p(0) < 0 ? -1.0 : 1.0, p(1) < 0 ? -1.0 : 1.0, p(2) < 0 ? -1.0 : 1.0);
  Eigen::RowVector3d w = p.cwiseAbs() - dim;
  double g = std::max(w(0), std::max(w(1), w(2)));
  Eigen::RowVector3d q = w.cwiseMax(0.0);
  double l = q.norm();
  l = std::abs(l) < 1e-6 ? 1e-6 : l;
  d = g > 0.0 ? (l < 1e-6 ? 0 : l) : g;
  Eigen::RowVector3d localGrad = g > 0.0
    ? Eigen::RowVector3d(q / l)
    : Eigen::RowVector3d(w(0) == g ? 1.0 : 0.0, w(1) == g ? 1.0 : 0.0, w(2) == g ? 1.0 : 0.0);
  grad = localGrad.cwiseProduct(s);
  Eigen::Matrix3d R = T.block<3,3>(0,0);
  grad = (R * grad.transpose()).transpose();
  grad.normalize();
}

Cylinder::Cylinder(double h, double r, Eigen::RowVector3d c) {
  height = h;
  radius = r;
  center = c;
  Eigen::Affine3d t = Eigen::Translation3d(center) * Eigen::Scaling(r,h/2,r);
  T0 = t.matrix();
  igl::read_triangle_mesh("../meshes/cylinder.obj", V0, F0);
  const Eigen::MatrixXd TTot = (T * T0).transpose();
  Vc = (V0.rowwise().homogeneous() * TTot).rowwise().hnormalized();
  std::cout << Vc << std::endl;
}

void Cylinder::eval(Eigen::RowVector3d p, double &d, Eigen::RowVector3d &grad) {
  Eigen::Matrix4d Minv = T.inverse();
  p = (p.homogeneous() * Minv.transpose()).eval().hnormalized();
  p = p - center;
  double l = Eigen::Vector2d(p.x(), p.z()).norm();
  Eigen::Vector2d e = Eigen::Vector2d(l-radius,abs(p.y()) - height/2.0);
  Eigen::Vector2d h = e.cwiseMax(0.0);
  double f = h.norm();
  double g = std::max(e.x(), e.y());
  Eigen::RowVector3d du = Eigen::Vector3d(p.x()/l, 0.0, p.z()/l);
  Eigen::RowVector3d dv = Eigen::Vector3d(0.0,p.y()<0.0?-1.0:1.0, 0.0);
  d = g<=0 ? g : f;
  grad = g<=0 ? (e.x() > e.y() ? du : dv) : (h.x() * du + h.y() * dv) / f;
  Eigen::Matrix3d R = T.block<3,3>(0,0);
  grad = (R * grad.transpose()).transpose();
  grad.normalize();
}


SDF::SDF() {
  primitives = std::vector<std::unique_ptr<Primitive>>();
}

void SDF::getMesh(Eigen::MatrixXd &V, Eigen::MatrixXi &F) {
  // I am ashamed of this function, need to rewrite
  size_t n = primitives.size();
  Eigen::MatrixXd Vtot[n];
  Eigen::MatrixXi Ftot[n];

  int offset = 0;
  int totF = 0;
  for (size_t i = 0; i < n; i++) {
    Eigen::MatrixXd Vi;
    Eigen::MatrixXi Fi;
    primitives[i]->getMesh(Vi, Fi);
    totF += Fi.rows();
    Vtot[i] = Vi;
    Ftot[i] = Fi.array() + offset;
    offset += Vi.rows();
  }

  V.resize(offset, 3);
  F.resize(totF, 3);
  int Voffset = 0;
  int Foffset = 0;

  for (size_t i = 0; i < n; i++) {
    V.middleRows(Voffset, Vtot[i].rows()) = Vtot[i];
    F.middleRows(Foffset, Ftot[i].rows()) = Ftot[i];
    Voffset += Vtot[i].rows();
    Foffset += Ftot[i].rows();
  }
}

void SDF::eval(Eigen::RowVector3d p, double &d, Eigen::RowVector3d &grad) {
  d = 1e6;
  grad = Eigen::RowVector3d::Zero();
  for (size_t i = 0; i < primitives.size(); i++) {
    double di;
    Eigen::RowVector3d gradi;
    primitives[i]->eval(p, di, gradi);
    grad = di < d ? gradi : grad;
    d = di < d ? di : d;
  }
}



void SDF::setT(Eigen::MatrixXd Transformation) {
  for (size_t i = 0; i < primitives.size(); i++) {
    primitives[i]->setT(Transformation);
  }
}

void SDF::computeSdfVector (Eigen::MatrixXd &V, Eigen::MatrixXd &Vsdf) {
  Vsdf.resize(V.rows(), V.cols());
  for (int i = 0; i < V.rows(); i++) {
    double d;
    Eigen::RowVector3d grad;
    auto p = V.row(i);
    this->eval(p, d, grad);
    Vsdf.row(i) = p - d * grad;
  }
}

void SDF::computeSdfVector2 (Eigen::MatrixXd &V, Eigen::MatrixXd &Vsdf) {
  Vsdf.resize(V.rows(), V.cols());
  for (int i = 0; i < V.rows(); i++) {
    double d, d2;
    Eigen::RowVector3d grad, grad2;
    auto p = V.row(i);
    this->eval(p, d, grad);
    this->eval(p - d * grad, d2, grad2);
    if (d2 >= 0)
      Vsdf.row(i) = p - d * grad;
    else
      Vsdf.row(i) = p;
  }
}

void sdf(Eigen::RowVector3d p, double &d, Eigen::RowVector3d &grad) {
  Eigen::RowVector3d center = Eigen::RowVector3d::Zero();
  double radius = 0.5;
  double l = (p - center).norm();
  l = abs(l) < 1e-6 ? 1e-6 : l;
  d = l - radius;
  grad = (p - center) / l;
}

void sdf_cube(Eigen::RowVector3d p, double &d, Eigen::RowVector3d &grad) {
  Eigen::RowVector3d center = Eigen::RowVector3d::Zero();
  Eigen::RowVector3d box = 0.7 * Eigen::RowVector3d::Ones();
  Eigen::RowVector3d w = p.cwiseAbs()-(box);
  double g = std::max(w(0),std::max(w(1),w(2)));
  Eigen::RowVector3d q = w.cwiseMax(0.0);
  double l = q.norm();
  l = abs(l) < 1e-6 ? 1e-6 : l;
  d = g > 0.0 ? (l < 1e-6 ? 0 : l) : g;
  grad = g > 0.0 ? Eigen::RowVector3d(q/l) : Eigen::RowVector3d(w(0) == g ? 1.0 : 0.0 ,w(1) == g ? 1.0 : 0.0, w(2) == g ? 1.0 : 0.0) ;
}



void computeSdfVector (Eigen::MatrixXd &V, Eigen::MatrixXd &Vsdf) {
  Vsdf.resize(V.rows(), V.cols());
  for (int i = 0; i < V.rows(); i++) {
    double d;
    Eigen::RowVector3d grad;
    auto p = V.row(i);
    sdf(p, d, grad);
    Vsdf.row(i) = p - d * grad;
  }
}

