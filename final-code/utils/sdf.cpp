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
    Eigen::Vector3d pos = pos_runtime;  // Use absolute position directly
    Eigen::Matrix3d rot = rotation * rot_runtime;
    Eigen::Vector3d scl = scale.cwiseProduct(scale_runtime);
    
    Eigen::Affine3d mesh_transform = Eigen::Translation3d(pos) * 
                                      Eigen::Matrix3d(rot) * 
                                      Eigen::Scaling(scl);
    const Eigen::MatrixXd TTot = mesh_transform.matrix().transpose();
    Vc = (V0.rowwise().homogeneous() * TTot).rowwise().hnormalized();
  }
  F = F0;
  V = Vc;
}

Sphere::Sphere(double r, Eigen::RowVector3d c) {
  radius = r;
  position = c.transpose();
  pos_runtime = position;  // Initialize runtime position to base position
  rotation = Eigen::Matrix3d::Identity();
  scale = Eigen::Vector3d::Ones() * radius;
  igl::read_triangle_mesh("../meshes/sphere2.obj", V0, F0);
}

void Sphere::eval(Eigen::RowVector3d p, double &d, Eigen::RowVector3d &grad) {
  Eigen::Vector3d p_local;
  Eigen::Vector3d grad_local;
  transformToLocal(p.transpose(), p_local);
  evalLocal(p_local, d, grad_local);
  
  Eigen::Vector3d pos = position + pos_runtime;
  Eigen::Matrix3d rot = rotation * rot_runtime;
  Eigen::Vector3d scl = scale.cwiseProduct(scale_runtime);
  
  transformGradientToWorld(grad_local, scl, rot);
  grad = grad_local.transpose();
}

void Sphere::evalLocal(const Eigen::Vector3d& p_local, double& d, Eigen::Vector3d& grad_local) {
  double l = p_local.norm();
  l = abs(l) < 1e-6 ? 1e-6 : l;
  d = l - 1.0;  // radius is already factored into scale
  grad_local = p_local / l;
}

void Sphere::setScale(const Eigen::Vector3d& s) {
  // Check if scale has actually changed (within tolerance)
  double change = (s - last_applied_scale).norm();
  if (change < 1e-6) {
    return;  // No significant change, don't apply scaling
  }
  
  // Compute additive scale delta: (current_scale - last_applied_scale)
  Eigen::Vector3d scale_delta = s - last_applied_scale;
  // Apply uniform scale (use average delta)
  double avg_delta = (scale_delta(0) + scale_delta(1) + scale_delta(2)) / 3.0;
  radius += avg_delta;
  // Clamp to minimum
  radius = std::max(radius, 0.02);
  scale = Eigen::Vector3d::Ones() * radius;
  last_applied_scale = s;
  changed = true;
}

void Sphere::resetScaleTracking() {
  // Sync to current radius
  last_applied_scale = Eigen::Vector3d::Ones() * radius;
}

Box::Box(Eigen::RowVector3d d, Eigen::RowVector3d c) {
  dim = d.transpose();
  position = c.transpose();
  pos_runtime = position;  // Initialize runtime position to base position
  rotation = Eigen::Matrix3d::Identity();
  scale = dim;
  igl::read_triangle_mesh("../meshes/cube2.obj", V0, F0);
}

void Box::eval(Eigen::RowVector3d p, double &d, Eigen::RowVector3d &grad) {
  Eigen::Vector3d p_local;
  Eigen::Vector3d grad_local;
  transformToLocal(p.transpose(), p_local);
  evalLocal(p_local, d, grad_local);
  
  Eigen::Vector3d pos = position + pos_runtime;
  Eigen::Matrix3d rot = rotation * rot_runtime;
  Eigen::Vector3d scl = scale.cwiseProduct(scale_runtime);
  
  transformGradientToWorld(grad_local, scl, rot);
  grad = grad_local.transpose();
}

void Box::evalLocal(const Eigen::Vector3d& p_local, double& d, Eigen::Vector3d& grad_local) {
  // In local space, box extends from -1 to 1 (normalized)
  Eigen::Vector3d s(p_local(0) < 0 ? -1.0 : 1.0, 
                    p_local(1) < 0 ? -1.0 : 1.0, 
                    p_local(2) < 0 ? -1.0 : 1.0);
  Eigen::Vector3d w = p_local.cwiseAbs() - Eigen::Vector3d::Ones();
  double g = std::max(w(0), std::max(w(1), w(2)));
  Eigen::Vector3d q = w.cwiseMax(0.0);
  double l = q.norm();
  l = std::abs(l) < 1e-6 ? 1e-6 : l;
  d = g > 0.0 ? (l < 1e-6 ? 0 : l) : g;
  
  Eigen::Vector3d localGrad = g > 0.0
    ? Eigen::Vector3d(q / l)
    : Eigen::Vector3d(w(0) == g ? 1.0 : 0.0, w(1) == g ? 1.0 : 0.0, w(2) == g ? 1.0 : 0.0);
  grad_local = localGrad.cwiseProduct(s);
}

void Box::setScale(const Eigen::Vector3d& s) {
  // Apply absolute per-axis scale so X/Y/Z controls stay independent.
  Eigen::Vector3d clamped = s.cwiseMax(Eigen::Vector3d::Constant(0.02));
  double change = (clamped - dim).norm();
  if (change < 1e-6) {
    return;  // No significant change, don't apply scaling
  }

  dim = clamped;
  scale = dim;
  last_applied_scale = dim;
  changed = true;
}

void Box::resetScaleTracking() {
  // Sync to current dimensions
  last_applied_scale = dim;
}

Cylinder::Cylinder(double h, double r, Eigen::RowVector3d c) {
  height = h;
  radius = r;
  position = c.transpose();
  rotation = Eigen::Matrix3d::Identity();
  scale = Eigen::Vector3d(r, h / 2.0, r);
  igl::read_triangle_mesh("../meshes/cylinder.obj", V0, F0);
}

void Cylinder::eval(Eigen::RowVector3d p, double &d, Eigen::RowVector3d &grad) {
  Eigen::Vector3d p_local;
  Eigen::Vector3d grad_local;
  transformToLocal(p.transpose(), p_local);
  evalLocal(p_local, d, grad_local);
  
  Eigen::Vector3d pos = position + pos_runtime;
  Eigen::Matrix3d rot = rotation * rot_runtime;
  Eigen::Vector3d scl = scale.cwiseProduct(scale_runtime);
  
  transformGradientToWorld(grad_local, scl, rot);
  grad = grad_local.transpose();
}

void Cylinder::evalLocal(const Eigen::Vector3d& p_local, double& d, Eigen::Vector3d& grad_local) {
  // In local space, cylinder has radius 1, height 1 (normalized)
  double l = Eigen::Vector2d(p_local.x(), p_local.z()).norm();
  l = l < 1e-6 ? 1e-6 : l;
  Eigen::Vector2d e = Eigen::Vector2d(l - 1.0, abs(p_local.y()) - 1.0);
  Eigen::Vector2d h = e.cwiseMax(0.0);
  double f = h.norm();
  double g = std::max(e.x(), e.y());
  
  Eigen::Vector3d du = Eigen::Vector3d(p_local.x() / l, 0.0, p_local.z() / l);
  Eigen::Vector3d dv = Eigen::Vector3d(0.0, p_local.y() < 0.0 ? -1.0 : 1.0, 0.0);
  
  d = g <= 0 ? g : f;
  grad_local = g <= 0 ? (e.x() > e.y() ? du : dv) : (h.x() * du + h.y() * dv) / f;
}


SDF::SDF() {
  primitives = std::vector<std::shared_ptr<Primitive>>();
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



void SDF::setRuntimeTransform(int idx, const Eigen::Vector3d& p, const Eigen::Matrix3d& r, const Eigen::Vector3d& s) {
  if (idx >= 0 && idx < (int)primitives.size()) {
    primitives[idx]->setRuntimeTransform(p, r, s);
  }
}

void SDF::setScale(int idx, const Eigen::Vector3d& s) {
  if (idx >= 0 && idx < (int)primitives.size()) {
    primitives[idx]->setScale(s);
  }
}

void SDF::resetScaleTracking(int idx) {
  if (idx >= 0 && idx < (int)primitives.size()) {
    primitives[idx]->resetScaleTracking();
  }
}

Eigen::Vector3d SDF::getScale(int idx) const {
  if (idx >= 0 && idx < (int)primitives.size()) {
    if (auto sphere = dynamic_cast<Sphere*>(primitives[idx].get())) {
      return sphere->getScale();
    } else if (auto box = dynamic_cast<Box*>(primitives[idx].get())) {
      return box->getScale();
    }
  }
  return Eigen::Vector3d::Ones();
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

void SDF::computeSdfVector3 (Eigen::MatrixXd &V, Eigen::MatrixXd &Vsdf) {
  Vsdf.resize(V.rows(), V.cols());
  double alpha = 0.01;  // Step size along gradient
  for (int i = 0; i < V.rows(); i++) {
    double d;
    Eigen::RowVector3d grad;
    auto p = V.row(i);
    this->eval(p, d, grad);
    grad.normalize();
    Vsdf.row(i) = p - alpha * grad * (d >= 0 ? 1.0 : -1.0);  // Current position + step along gradient
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

