#ifndef SKETCH2MESH_BRIDGE_H
#define SKETCH2MESH_BRIDGE_H

#include <Eigen/Core>
#include <igl/opengl/glfw/Viewer.h>

#include <string>
#include <vector>

namespace sketch2mesh_bridge
{

struct AddStyleRequest
{
    bool valid = false;
    bool isBaseMesh = false;  // true = load as base deformable mesh, false = add as style/primitive
    std::string baseName;
    std::string filename;
    Eigen::RowVector3d center = Eigen::RowVector3d::Zero();
    Eigen::RowVector3d primitiveScale = Eigen::RowVector3d::Ones();
    bool hasAnisotropicScale = false;
    double lambda = 1.0;
    double radius = 1.0;
    double falloff = 2.0;
};

class Bridge
{
public:
    void initialize(const std::string& meshDirectory);

    bool isDrawingMode() const;

    void drawUi(igl::opengl::glfw::Viewer& viewer);
    void drawOverlay();

    bool onMouseDown(const igl::opengl::glfw::Viewer& viewer, int button);
    bool onMouseMove(const igl::opengl::glfw::Viewer& viewer);
    bool onMouseUp(const igl::opengl::glfw::Viewer& viewer, int button, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

    bool consumeRequest(AddStyleRequest& outRequest);

private:
    enum class PrimitiveKind
    {
        Cube,
        Sphere,
        Pyramid
    };

    PrimitiveKind classifyStroke() const;
    void queuePrimitiveRequest(const igl::opengl::glfw::Viewer& viewer, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, PrimitiveKind kind);
    void queueSelectedMeshRequest();

    bool drawingMode_ = false;
    bool isDrawing_ = false;
    std::vector<Eigen::Vector2f> strokePoints_;

    std::vector<std::string> meshFiles_;
    int selectedMeshIndex_ = 0;

    AddStyleRequest pendingRequest_;
};

} // namespace sketch2mesh_bridge

#endif
