
#ifndef EXAMPLE_SDF_H
#define EXAMPLE_SDF_H

#include <eigen/Eigen>
#include <memory>

enum PrimitiveType {
  SPHERE,
  CUBE
};

class Primitive {
public:
  Primitive() : position(Eigen::Vector3d::Zero()), 
                rotation(Eigen::Matrix3d::Identity()),
                scale(Eigen::Vector3d::Ones()),
                pos_runtime(Eigen::Vector3d::Zero()),
                rot_runtime(Eigen::Matrix3d::Identity()),
                scale_runtime(Eigen::Vector3d::Ones()),
                changed(true) {}
  
  void setRuntimeTransform(const Eigen::Vector3d& p, const Eigen::Matrix3d& r, const Eigen::Vector3d& s) {
    pos_runtime = p;
    rot_runtime = r;
    scale_runtime = s;
    changed = true;
  }
  
  virtual void setScale(const Eigen::Vector3d& s) {
    scale = s;
    changed = true;
  }
  
  virtual void resetScaleTracking() {}

  virtual Eigen::Vector3d getScale() const { return scale; }
  
  void getMesh(Eigen::MatrixXd &V, Eigen::MatrixXi &F);
  virtual void eval(Eigen::RowVector3d p, double &d, Eigen::RowVector3d &grad) = 0;

protected:
  // Base transformation components
  Eigen::Vector3d position;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d scale;
  
  // Runtime updates (for animation/interaction)
  Eigen::Vector3d pos_runtime;
  Eigen::Matrix3d rot_runtime;
  Eigen::Vector3d scale_runtime;
  
  bool changed;
  Eigen::MatrixXd V0, Vc;
  Eigen::MatrixXi F0;
  
  // Helper: transform point to local space
  void transformToLocal(const Eigen::Vector3d& p_world, Eigen::Vector3d& p_local) {
    Eigen::Vector3d pos = pos_runtime;  // Use absolute position directly
    Eigen::Matrix3d rot = rotation * rot_runtime;
    Eigen::Vector3d scl = scale.cwiseProduct(scale_runtime);
    
    // Translate, then apply inverse rotation (transpose), then scale
    p_local = (rot.transpose() * (p_world - pos)).cwiseQuotient(scl);
  }
  
  // Helper: transform gradient from local to world space (handles non-uniform scaling)
  void transformGradientToWorld(Eigen::Vector3d& grad_local,
                                const Eigen::Vector3d& scl_active,
                                const Eigen::Matrix3d& rot_active) {
    // For non-uniform scaling: grad *= (1/scale)
    grad_local = grad_local.cwiseQuotient(scl_active);
    // Rotate back to world space: grad = R * grad
    grad_local = rot_active * grad_local;
    grad_local.normalize();
  }
  
  // Each primitive implements raw SDF in local space
  virtual void evalLocal(const Eigen::Vector3d& p_local, double& d, Eigen::Vector3d& grad_local) = 0;
};

class SDF {
  public:
  SDF();
  void addPrimitive(std::shared_ptr<Primitive> p) {primitives.push_back(std::move(p));}
  void eval(Eigen::RowVector3d p, double &d, Eigen::RowVector3d &grad);
  void setRuntimeTransform(int idx, const Eigen::Vector3d& p, const Eigen::Matrix3d& r, const Eigen::Vector3d& s);
  void setScale(int idx, const Eigen::Vector3d& s);
  void resetScaleTracking(int idx);
  Eigen::Vector3d getScale(int idx) const;
  void getMesh(Eigen::MatrixXd &V, Eigen::MatrixXi &F);
  void computeSdfVector (Eigen::MatrixXd &V, Eigen::MatrixXd &Vsdf);
  void computeSdfVector2 (Eigen::MatrixXd &V, Eigen::MatrixXd &Vsdf);
  void computeSdfVector3 (Eigen::MatrixXd &V, Eigen::MatrixXd &Vsdf);
  void setRuntimeTransform(const Eigen::Vector3d& p, const Eigen::Matrix3d& r, const Eigen::Vector3d& s, size_t i) {
    if (i <primitives.size()) {
      primitives[i]->setRuntimeTransform(p, r, s);
    }
  };
private:
  std::vector<std::shared_ptr<Primitive>> primitives;
};

class Sphere : public Primitive {
  public:
  Sphere(double r, Eigen::RowVector3d c);
  void eval(Eigen::RowVector3d p, double &d, Eigen::RowVector3d &grad) final;
  void evalLocal(const Eigen::Vector3d& p_local, double& d, Eigen::Vector3d& grad_local) final;
  void setScale(const Eigen::Vector3d& s) final;
  void resetScaleTracking() final;
  double radius;
  Eigen::Vector3d getScale() const { return scale; }
  
private:
  Eigen::Vector3d last_applied_scale = Eigen::Vector3d::Ones();
};

class Box : public Primitive {
public:
  Box(Eigen::RowVector3d d, Eigen::RowVector3d c);
  void eval(Eigen::RowVector3d p, double &d, Eigen::RowVector3d &grad) final;
  void evalLocal(const Eigen::Vector3d& p_local, double& d, Eigen::Vector3d& grad_local) final;
  void setScale(const Eigen::Vector3d& s) final;
  void resetScaleTracking() final;
  Eigen::Vector3d dim;
  Eigen::Vector3d getScale() const { return scale; }
  
private:
  Eigen::Vector3d last_applied_scale = Eigen::Vector3d::Ones();
};

class Cylinder : public Primitive {
public:
  Cylinder(double h, double r, Eigen::RowVector3d c);
  void eval(Eigen::RowVector3d p, double &d, Eigen::RowVector3d &grad) final;
  void evalLocal(const Eigen::Vector3d& p_local, double& d, Eigen::Vector3d& grad_local) final;
  double height;
  double radius;
};

#endif //EXAMPLE_SDF_H