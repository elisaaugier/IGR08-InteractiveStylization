#include "sketch2mesh_bridge.h"

#include <igl/project.h>
#include <igl/unproject.h>
#include <imgui/imgui.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>

namespace sketch2mesh_bridge
{

namespace
{

double polygon_area(const std::vector<Eigen::Vector2f>& pts)
{
    if (pts.size() < 3)
    {
        return 0.0;
    }

    double area2 = 0.0;
    for (size_t i = 0; i < pts.size(); ++i)
    {
        const size_t j = (i + 1) % pts.size();
        area2 += static_cast<double>(pts[i].x()) * static_cast<double>(pts[j].y()) -
                 static_cast<double>(pts[j].x()) * static_cast<double>(pts[i].y());
    }

    return 0.5 * std::abs(area2);
}

double polyline_perimeter(const std::vector<Eigen::Vector2f>& pts)
{
    if (pts.size() < 2)
    {
        return 0.0;
    }

    double p = 0.0;
    for (size_t i = 1; i < pts.size(); ++i)
    {
        p += (pts[i] - pts[i - 1]).norm();
    }
    p += (pts.front() - pts.back()).norm();
    return p;
}

double cross2d(const Eigen::Vector2d& o, const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
    return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
}

std::vector<Eigen::Vector2d> convex_hull_2d(const std::vector<Eigen::Vector2f>& pts)
{
    std::vector<Eigen::Vector2d> p;
    p.reserve(pts.size());
    for (const auto& q : pts)
    {
        p.emplace_back(static_cast<double>(q.x()), static_cast<double>(q.y()));
    }

    if (p.size() <= 3)
    {
        return p;
    }

    std::sort(p.begin(), p.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b)
    {
        if (a.x() != b.x()) return a.x() < b.x();
        return a.y() < b.y();
    });

    std::vector<Eigen::Vector2d> h;
    h.reserve(p.size() * 2);

    for (size_t i = 0; i < p.size(); ++i)
    {
        while (h.size() >= 2 && cross2d(h[h.size() - 2], h[h.size() - 1], p[i]) <= 0.0)
        {
            h.pop_back();
        }
        h.push_back(p[i]);
    }

    const size_t lower_size = h.size();
    for (int i = static_cast<int>(p.size()) - 2; i >= 0; --i)
    {
        while (h.size() > lower_size && cross2d(h[h.size() - 2], h[h.size() - 1], p[static_cast<size_t>(i)]) <= 0.0)
        {
            h.pop_back();
        }
        h.push_back(p[static_cast<size_t>(i)]);
    }

    if (!h.empty())
    {
        h.pop_back();
    }
    return h;
}

int estimate_corner_count(const std::vector<Eigen::Vector2f>& pts, bool closed)
{
    if (pts.size() < 10)
    {
        return 0;
    }

    const int n = static_cast<int>(pts.size());
    const int start = closed ? 0 : 1;
    const int end = closed ? n : (n - 1);
    const int skip = std::max(2, n / 18);

    int corners = 0;
    int cooldown = 0;
    for (int i = start; i < end; ++i)
    {
        if (cooldown > 0)
        {
            cooldown -= 1;
            continue;
        }

        const int im1 = (i - 1 + n) % n;
        const int ip1 = (i + 1) % n;

        Eigen::Vector2d e1 = (pts[i] - pts[im1]).cast<double>();
        Eigen::Vector2d e2 = (pts[ip1] - pts[i]).cast<double>();
        const double n1 = e1.norm();
        const double n2 = e2.norm();
        if (n1 < 1e-8 || n2 < 1e-8)
        {
            continue;
        }

        e1 /= n1;
        e2 /= n2;
        const double c = std::max(-1.0, std::min(1.0, e1.dot(e2)));
        const double turning = M_PI - std::acos(c);
        if (turning > 0.55)
        {
            corners += 1;
            cooldown = skip;
        }
    }

    return corners;
}

} // namespace

void Bridge::initialize(const std::string& meshDirectory)
{
    meshFiles_.clear();

    std::error_code ec;
    if (!std::filesystem::exists(meshDirectory, ec))
    {
        return;
    }

    for (const auto& entry : std::filesystem::directory_iterator(meshDirectory, ec))
    {
        if (!entry.is_regular_file())
        {
            continue;
        }
        const std::string ext = entry.path().extension().string();
        if (ext == ".obj" || ext == ".OBJ")
        {
            meshFiles_.push_back(entry.path().filename().string());
        }
    }

    std::sort(meshFiles_.begin(), meshFiles_.end());
    if (selectedMeshIndex_ >= static_cast<int>(meshFiles_.size()))
    {
        selectedMeshIndex_ = 0;
    }
}

bool Bridge::isDrawingMode() const
{
    return drawingMode_;
}

void Bridge::drawUi(igl::opengl::glfw::Viewer& viewer)
{
    ImGui::Text("Add from meshes/");
    if (meshFiles_.empty())
    {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "No .obj found in meshes/");
    }
    else
    {
        std::vector<const char*> labels;
        labels.reserve(meshFiles_.size());
        for (const std::string& f : meshFiles_)
        {
            labels.push_back(f.c_str());
        }
        ImGui::Combo("Mesh file", &selectedMeshIndex_, labels.data(), static_cast<int>(labels.size()));
        if (ImGui::Button("Add selected mesh", ImVec2(-1, 0)))
        {
            queueSelectedMeshRequest();
        }
    }

    ImGui::Separator();
    ImGui::TextWrapped("Hold SHIFT and drag to draw on the model. The tool adds one of: cube, sphere, pyramid.");

    (void)viewer;
}

void Bridge::drawOverlay()
{
    if (strokePoints_.size() < 2)
    {
        return;
    }

    ImDrawList* drawList = ImGui::GetBackgroundDrawList();
    for (size_t i = 1; i < strokePoints_.size(); ++i)
    {
        drawList->AddLine(
            ImVec2(strokePoints_[i - 1].x(), strokePoints_[i - 1].y()),
            ImVec2(strokePoints_[i].x(), strokePoints_[i].y()),
            IM_COL32(255, 0, 0, 255),
            2.5f);
    }
}

bool Bridge::onMouseDown(const igl::opengl::glfw::Viewer& viewer, int button)
{
    if (button != 0)
    {
        return false;
    }

    isDrawing_ = true;
    strokePoints_.clear();
    strokePoints_.emplace_back(static_cast<float>(viewer.current_mouse_x), static_cast<float>(viewer.current_mouse_y));
    return true;
}

bool Bridge::onMouseMove(const igl::opengl::glfw::Viewer& viewer)
{
    if (!isDrawing_)
    {
        return false;
    }

    strokePoints_.emplace_back(static_cast<float>(viewer.current_mouse_x), static_cast<float>(viewer.current_mouse_y));
    return true;
}

bool Bridge::onMouseUp(const igl::opengl::glfw::Viewer& viewer, int button, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    if (!isDrawing_ || button != 0)
    {
        return false;
    }

    isDrawing_ = false;
    if (strokePoints_.size() < 6)
    {
        strokePoints_.clear();
        return true;
    }

    const PrimitiveKind kind = classifyStroke();
    queuePrimitiveRequest(viewer, V, F, kind);
    strokePoints_.clear();
    return true;
}

bool Bridge::consumeRequest(AddStyleRequest& outRequest)
{
    if (!pendingRequest_.valid)
    {
        return false;
    }

    outRequest = pendingRequest_;
    pendingRequest_.valid = false;
    return true;
}

Bridge::PrimitiveKind Bridge::classifyStroke() const
{
    if (strokePoints_.size() < 10)
    {
        return PrimitiveKind::Cube;
    }

    Eigen::MatrixXd stroke2d(static_cast<int>(strokePoints_.size()), 2);
    for (int i = 0; i < static_cast<int>(strokePoints_.size()); ++i)
    {
        stroke2d(i, 0) = static_cast<double>(strokePoints_[i].x());
        stroke2d(i, 1) = static_cast<double>(strokePoints_[i].y());
    }

    const Eigen::RowVector2d minP = stroke2d.colwise().minCoeff();
    const Eigen::RowVector2d maxP = stroke2d.colwise().maxCoeff();
    const double w = std::max(1e-8, maxP(0) - minP(0));
    const double h = std::max(1e-8, maxP(1) - minP(1));
    const double aspectRatio = w / h;
    const double diag = std::sqrt(w * w + h * h);
    const bool isClosed = ((stroke2d.row(0) - stroke2d.row(stroke2d.rows() - 1)).norm() < 0.2 * diag);

    double perimeter = 0.0;
    for (int i = 1; i < stroke2d.rows(); ++i)
    {
        perimeter += (stroke2d.row(i) - stroke2d.row(i - 1)).norm();
    }
    if (isClosed)
    {
        perimeter += (stroke2d.row(0) - stroke2d.row(stroke2d.rows() - 1)).norm();
    }

    double area = 0.0;
    if (isClosed)
    {
        for (int i = 0; i < stroke2d.rows(); ++i)
        {
            const int j = (i + 1) % stroke2d.rows();
            area += stroke2d(i, 0) * stroke2d(j, 1) - stroke2d(j, 0) * stroke2d(i, 1);
        }
        area = 0.5 * std::abs(area);
    }
    const double circularity = (perimeter > 1e-8) ? (4.0 * M_PI * area / (perimeter * perimeter)) : 0.0;
    const int cornerCount = estimate_corner_count(strokePoints_, isClosed);

    // Normalize + PCA align (Sketch2Mesh behavior)
    Eigen::MatrixXd norm2d = stroke2d;
    const Eigen::RowVector2d mean2d = norm2d.colwise().mean();
    norm2d.rowwise() -= mean2d;
    const double scale = norm2d.cwiseAbs().maxCoeff();
    if (scale > 1e-10)
    {
        norm2d /= scale;
    }
    if (norm2d.rows() > 1)
    {
        Eigen::Matrix2d cov = (norm2d.transpose() * norm2d) / std::max(1, static_cast<int>(norm2d.rows()) - 1);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
        if (es.info() == Eigen::Success)
        {
            norm2d = norm2d * es.eigenvectors();
        }
    }

    std::vector<Eigen::Vector2f> normPts;
    normPts.reserve(static_cast<size_t>(norm2d.rows()));
    for (int i = 0; i < norm2d.rows(); ++i)
    {
        normPts.emplace_back(static_cast<float>(norm2d(i, 0)), static_cast<float>(norm2d(i, 1)));
    }
    const std::vector<Eigen::Vector2d> hull = convex_hull_2d(normPts);
    const int hullVerts = static_cast<int>(hull.size());
    if (hullVerts < 3)
    {
        return PrimitiveKind::Cube;
    }

    double hullMinX = std::numeric_limits<double>::infinity();
    double hullMaxX = -std::numeric_limits<double>::infinity();
    double hullMinY = std::numeric_limits<double>::infinity();
    double hullMaxY = -std::numeric_limits<double>::infinity();
    for (const auto& p : hull)
    {
        hullMinX = std::min(hullMinX, p.x());
        hullMaxX = std::max(hullMaxX, p.x());
        hullMinY = std::min(hullMinY, p.y());
        hullMaxY = std::max(hullMaxY, p.y());
    }
    const double hullW = std::max(1e-8, hullMaxX - hullMinX);
    const double hullH = std::max(1e-8, hullMaxY - hullMinY);
    const double hullAspect = hullW / hullH;

    double hullPerim = 0.0;
    double hullArea2 = 0.0;
    for (int i = 0; i < hullVerts; ++i)
    {
        const int j = (i + 1) % hullVerts;
        hullPerim += (hull[j] - hull[i]).norm();
        hullArea2 += hull[i].x() * hull[j].y() - hull[j].x() * hull[i].y();
    }
    const double hullArea = 0.5 * std::abs(hullArea2);
    const double hullCircularity = (hullPerim > 1e-8) ? (4.0 * M_PI * hullArea / (hullPerim * hullPerim)) : 0.0;

    const int majorCol = (hullW >= hullH) ? 0 : 1;
    const int minorCol = 1 - majorCol;
    const double majorMin = (majorCol == 0) ? hullMinX : hullMinY;
    const double majorMax = (majorCol == 0) ? hullMaxX : hullMaxY;
    const double majorRange = std::max(1e-8, majorMax - majorMin);
    const double bandFrac = 0.22;

    double lo_min = 1e9, lo_max = -1e9, hi_min = 1e9, hi_max = -1e9;
    int loN = 0, hiN = 0;
    for (int i = 0; i < hullVerts; ++i)
    {
        const double mj = (majorCol == 0) ? hull[i].x() : hull[i].y();
        const double mn = (minorCol == 0) ? hull[i].x() : hull[i].y();
        if (mj - majorMin <= bandFrac * majorRange) { lo_min = std::min(lo_min, mn); lo_max = std::max(lo_max, mn); loN++; }
        if (majorMax - mj <= bandFrac * majorRange) { hi_min = std::min(hi_min, mn); hi_max = std::max(hi_max, mn); hiN++; }
    }
    const double loW = (loN >= 1) ? std::max(1e-8, lo_max - lo_min) : 1e-8;
    const double hiW = (hiN >= 1) ? std::max(1e-8, hi_max - hi_min) : 1e-8;
    const double endWidthRatio = std::min(loW, hiW) / std::max(loW, hiW);

    const double F0 = hullCircularity;
    const double F1 = std::min(1.0, std::max(0.0, (double(hullVerts) - 3.0) / 9.0));
    const double F2 = std::min(1.0, std::abs(hullAspect - 1.0));
    const double F3 = endWidthRatio;

    struct Prototype { const char* name; double p0,p1,p2,p3,w0,w1,w2,w3; };
    const std::vector<Prototype> prototypes = {
        {"sphere",  0.95, 0.7, 0.05, 0.9,  3.0, 1.0, 2.0, 1.0},
        {"cube",    0.78, 0.1, 0.05, 0.9,  2.5, 2.0, 2.5, 1.0},
        {"pyramid", 0.62, 0.0, 0.10, 0.20, 2.8, 2.2, 2.0, 2.4}
    };

    std::vector<std::pair<std::string, double>> scores;
    for (const Prototype& p : prototypes)
    {
        double s = p.w0 * std::abs(F0 - p.p0)
                 + p.w1 * std::abs(F1 - p.p1)
                 + p.w2 * std::abs(F2 - p.p2)
                 + p.w3 * std::abs(F3 - p.p3);

        const bool squareLikeHull = (hullVerts <= 5 && F2 < 0.22 && F3 > 0.75 && F0 > 0.70 && F0 < 0.90);
        const bool parallelogramLikeHull = (hullVerts <= 5 && F0 > 0.68 && F0 < 0.90 && F3 > 0.60 && F2 > 0.08);
        if (squareLikeHull && std::string(p.name) == "cube")
        {
            s -= 0.20;
        }
        if (parallelogramLikeHull && std::string(p.name) == "cube")
        {
            s -= 0.12;
        }

        if (isClosed)
        {
            const bool nearCircular = (F0 > 0.90 && F2 < 0.18 && F3 > 0.75);
            const bool triangularLike = (hullVerts <= 4 && F0 < 0.72 && F3 < 0.55);
            const bool manyCorners = (cornerCount >= 4);
            const bool triangleCorners = (cornerCount == 3);
            const bool taperedPolygonLike = (cornerCount >= 3 && cornerCount <= 5 && F3 < 0.72);

            if (nearCircular && std::string(p.name) == "sphere") s -= 0.12;
            if (!nearCircular && std::string(p.name) == "sphere") s += 0.80;
            if (triangularLike && std::string(p.name) == "pyramid") s -= 0.35;
            if (taperedPolygonLike && std::string(p.name) == "pyramid") s -= 0.22;

            if (manyCorners && std::string(p.name) == "cube") s -= 0.22;
            if (triangleCorners && std::string(p.name) == "pyramid") s -= 0.45;
            if ((manyCorners || triangleCorners) && std::string(p.name) == "sphere") s += 0.50;
        }

        scores.emplace_back(p.name, s);
    }

    std::sort(scores.begin(), scores.end(), [](const std::pair<std::string, double>& a, const std::pair<std::string, double>& b)
    {
        return a.second < b.second;
    });

    const std::string label = scores.front().first;

    if (label == "sphere") return PrimitiveKind::Sphere;
    if (label == "pyramid") return PrimitiveKind::Pyramid;
    return PrimitiveKind::Cube;
}

void Bridge::queuePrimitiveRequest(const igl::opengl::glfw::Viewer& viewer, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, PrimitiveKind kind)
{
    if (V.rows() == 0 || F.rows() == 0 || strokePoints_.empty())
    {
        return;
    }

    float minX = std::numeric_limits<float>::infinity();
    float maxX = -std::numeric_limits<float>::infinity();
    float minY = std::numeric_limits<float>::infinity();
    float maxY = -std::numeric_limits<float>::infinity();

    for (const auto& p : strokePoints_)
    {
        minX = std::min(minX, p.x());
        maxX = std::max(maxX, p.x());
        minY = std::min(minY, p.y());
        maxY = std::max(maxY, p.y());
    }

    const float centerX = 0.5f * (minX + maxX);
    const float centerY = 0.5f * (minY + maxY);
    const Eigen::Vector3f meshCenter = V.colwise().mean().transpose().cast<float>();
    const Eigen::Vector3f meshCenterProj = igl::project(meshCenter,
                                                        viewer.core().view,
                                                        viewer.core().proj,
                                                        viewer.core().viewport);

    const float depth = meshCenterProj(2);
    const float viewH = static_cast<float>(viewer.core().viewport(3));
    const Eigen::Vector3f centerWin(centerX, viewH - centerY, depth);
    const Eigen::Vector3f centerWorldF = igl::unproject(centerWin,
                                                        viewer.core().view,
                                                        viewer.core().proj,
                                                        viewer.core().viewport);
    const Eigen::RowVector3d center = centerWorldF.cast<double>().transpose();

    // Convert stroke screen extents to world extents at the same drawing depth.
    const Eigen::Vector3f xMinWin(minX, viewH - centerY, depth);
    const Eigen::Vector3f xMaxWin(maxX, viewH - centerY, depth);
    const Eigen::Vector3f yMinWin(centerX, viewH - minY, depth);
    const Eigen::Vector3f yMaxWin(centerX, viewH - maxY, depth);

    const Eigen::Vector3f xMinWorld = igl::unproject(xMinWin,
                                                     viewer.core().view,
                                                     viewer.core().proj,
                                                     viewer.core().viewport);
    const Eigen::Vector3f xMaxWorld = igl::unproject(xMaxWin,
                                                     viewer.core().view,
                                                     viewer.core().proj,
                                                     viewer.core().viewport);
    const Eigen::Vector3f yMinWorld = igl::unproject(yMinWin,
                                                     viewer.core().view,
                                                     viewer.core().proj,
                                                     viewer.core().viewport);
    const Eigen::Vector3f yMaxWorld = igl::unproject(yMaxWin,
                                                     viewer.core().view,
                                                     viewer.core().proj,
                                                     viewer.core().viewport);

    const double worldWidth = (xMaxWorld - xMinWorld).norm();
    const double worldHeight = (yMaxWorld - yMinWorld).norm();
    const double sx = std::max(0.02, 0.5 * worldWidth);
    const double sy = std::max(0.02, 0.5 * worldHeight);
    const double sz = std::max(0.02, 0.25 * (worldWidth + worldHeight));
    const double radius = std::max(0.02, std::max(sx, std::max(sy, sz)));

    AddStyleRequest req;
    req.valid = true;
    req.center = center;
    req.primitiveScale = Eigen::RowVector3d(sx, sy, sz);
    req.hasAnisotropicScale = true;
    req.lambda = 1.0;
    req.radius = radius;
    req.falloff = 2.0;

    switch (kind)
    {
        case PrimitiveKind::Sphere:
            req.baseName = "sphere";
            req.filename = "sphere2.obj";
            break;
        case PrimitiveKind::Pyramid:
            req.baseName = "pyramid";
            req.filename = "pyramid.obj";
            break;
        case PrimitiveKind::Cube:
        default:
            req.baseName = "cube";
            req.filename = "cube.obj";
            break;
    }

    pendingRequest_ = req;
}

void Bridge::queueSelectedMeshRequest()
{
    if (meshFiles_.empty() || selectedMeshIndex_ < 0 || selectedMeshIndex_ >= static_cast<int>(meshFiles_.size()))
    {
        return;
    }

    const std::string selected = meshFiles_[selectedMeshIndex_];
    std::string baseName = selected;
    const size_t dot = baseName.find_last_of('.');
    if (dot != std::string::npos)
    {
        baseName = baseName.substr(0, dot);
    }

    AddStyleRequest req;
    req.valid = true;
    req.isBaseMesh = true;  // "Add selected mesh" loads it as the base deformable mesh
    req.baseName = baseName;
    req.filename = selected;
    req.center = Eigen::RowVector3d(0.0, 0.0, 0.0);
    req.lambda = 1.0;
    req.radius = 1.0;
    req.falloff = 2.0;
    pendingRequest_ = req;
}

} // namespace sketch2mesh_bridge
