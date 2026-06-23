#include <igl/readOBJ.h>
#include <igl/project.h>
#include <igl/unproject.h>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <imgui/imgui.h>

#include <normalize_unitbox.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#ifndef MESH_PATH
#define MESH_PATH "../meshes/"
#endif

#ifndef PRIMITIVE_PATH
#define PRIMITIVE_PATH "../primitives/"
#endif

namespace
{
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::RowVector4d;
using Eigen::RowVector3d;
using Eigen::Vector3d;

const RowVector4d& default_primitive_color()
{
    static const RowVector4d color(242.0 / 255.0, 208.0 / 255.0, 73.0 / 255.0, 0.45);
    return color;
}

struct TemplateMesh
{
    std::string name;
    std::string path;
    MatrixXd V;
    MatrixXi F;
    MatrixXd samples;
    std::vector<MatrixXd> samples2dVariants;
    std::vector<double> circularity2dVariants;
};

struct RecognitionResult
{
    std::string label;
    double score = std::numeric_limits<double>::infinity();
    std::vector<std::pair<std::string, double>> ranking;
};

struct StrokeFeatures
{
    int cornerCount = 0;
    double circularity = 0.0;
    double aspectRatio = 1.0;
    double endWidthRatio = 1.0;
    bool isClosed = false;
};

struct PrimitiveInstance
{
    std::string label;
    MatrixXd V;
    MatrixXi F;
    RowVector4d color = default_primitive_color();
    RowVector3d scale = RowVector3d::Ones();
    int pyramidSides = 5;
    double pyramidBaseSize = 1.0;
    double pyramidHeight = 1.0;
    Matrix3d pyramidOrientation = Matrix3d::Identity();
};

struct AppState
{
    MatrixXd meshV;
    MatrixXi meshF;
    MatrixXd strokePoints;
    bool drawing = false;
    bool hasPrediction = false;
    std::string currentPath;
    RecognitionResult prediction;
    std::vector<PrimitiveInstance> primitives;
    std::vector<Eigen::Vector2d> strokeScreenPoints;
    std::array<char, 512> meshInput{};
    int selectedMeshTemplate = 0;
    int selectedPrimitive = -1;
    bool draggingPrimitive = false;
    float dragDepth = 0.5f;
    RowVector3d dragOffset = RowVector3d::Zero();
    int uiSelectionPrimitive = -1;
    RowVector3d uiTranslate = RowVector3d::Zero();
    RowVector3d uiTranslatePrev = RowVector3d::Zero();
    RowVector3d uiRotateDeg = RowVector3d::Zero();
    RowVector3d uiRotateDegPrev = RowVector3d::Zero();
    RowVector3d uiScale = RowVector3d::Ones();
    RowVector3d uiScalePrev = RowVector3d::Ones();
    int uiPyramidSides = 5;
    float uiPyramidBaseSize = 1.0f;
    float uiPyramidHeight = 1.0f;
    int uiPyramidSidesPrev = 5;
    float uiPyramidBaseSizePrev = 1.0f;
    float uiPyramidHeightPrev = 1.0f;
} state;

std::vector<TemplateMesh> meshTemplates;
std::vector<TemplateMesh> primitiveTemplates;

std::vector<std::string> list_obj_files_in_directory(const std::string& directory)
{
    std::vector<std::string> files;
    namespace fs = std::filesystem;

    std::error_code ec;
    const fs::path dirPath(directory);
    if (!fs::exists(dirPath, ec) || !fs::is_directory(dirPath, ec))
    {
        return files;
    }

    for (const auto& entry : fs::directory_iterator(dirPath, ec))
    {
        if (ec || !entry.is_regular_file())
        {
            continue;
        }

        const fs::path p = entry.path();
        std::string ext = p.extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c)
        {
            return static_cast<char>(std::tolower(c));
        });

        if (ext == ".obj")
        {
            files.push_back(p.filename().string());
        }
    }

    std::sort(files.begin(), files.end());
    return files;
}

bool file_exists(const std::string& path)
{
    std::ifstream in(path.c_str());
    return in.good();
}

std::string resolve_existing_directory(std::initializer_list<std::string> candidates)
{
    namespace fs = std::filesystem;

    std::error_code ec;
    for (const std::string& candidate : candidates)
    {
        const fs::path dirPath(candidate);
        if (fs::exists(dirPath, ec) && fs::is_directory(dirPath, ec))
        {
            return dirPath.lexically_normal().string();
        }
    }

    return candidates.begin() != candidates.end() ? *candidates.begin() : std::string();
}

std::string mesh_directory()
{
    static const std::string dir = resolve_existing_directory(
    {
        std::string(MESH_PATH),
        "./meshes",
        "../meshes",
        "./Sketch2Mesh/meshes",
        "../Sketch2Mesh/meshes"
    });
    return dir;
}

std::string primitive_directory()
{
    static const std::string dir = resolve_existing_directory(
    {
        std::string(PRIMITIVE_PATH),
        "./primitives",
        "../primitives",
        "./Sketch2Mesh/primitives",
        "../Sketch2Mesh/primitives"
    });
    return dir;
}

std::string stem_from_path(const std::string& path)
{
    const std::size_t slash = path.find_last_of("/\\");
    const std::string file = (slash == std::string::npos) ? path : path.substr(slash + 1);
    const std::size_t dot = file.find_last_of('.');
    return (dot == std::string::npos) ? file : file.substr(0, dot);
}

std::string resolve_mesh_path(const std::string& input)
{
    if (input.empty())
    {
        return mesh_directory() + "/spot.obj";
    }

    if (file_exists(input))
    {
        return input;
    }

    const std::string fallback = mesh_directory() + "/" + input;
    if (file_exists(fallback))
    {
        return fallback;
    }

    return input;
}

Matrix3d pca_frame(const MatrixXd& V)
{
    MatrixXd centered = V.rowwise() - V.colwise().mean();
    Matrix3d covariance = Matrix3d::Zero();
    if (centered.rows() > 1)
    {
        covariance = (centered.transpose() * centered) / double(centered.rows() - 1);
    }

    Eigen::SelfAdjointEigenSolver<Matrix3d> solver(covariance);
    Matrix3d frame = Matrix3d::Zero();
    std::array<int, 3> order = {0, 1, 2};
    std::sort(order.begin(), order.end(), [&](int a, int b)
    {
        return solver.eigenvalues()(a) > solver.eigenvalues()(b);
    });

    for (int column = 0; column < 3; ++column)
    {
        frame.col(column) = solver.eigenvectors().col(order[column]);
    }

    return frame;
}

MatrixXd normalize_and_center(MatrixXd V)
{
    if (V.rows() == 0)
    {
        return V;
    }

    normalize_unitbox(V);
    V = V.rowwise() - V.colwise().mean();
    return V;
}

MatrixXd align_to_pca(const MatrixXd& V)
{
    if (V.rows() == 0)
    {
        return V;
    }

    const Matrix3d frame = pca_frame(V);
    return V * frame;
}

MatrixXd sample_points(const MatrixXd& V, int maxSamples)
{
    if (V.rows() <= maxSamples)
    {
        return V;
    }

    MatrixXd samples(maxSamples, 3);
    const int last = V.rows() - 1;
    const int denom = std::max(1, maxSamples - 1);
    for (int i = 0; i < maxSamples; ++i)
    {
        const int index = static_cast<int>(std::lround(double(i) * double(last) / double(denom)));
        samples.row(i) = V.row(index);
    }

    return samples;
}

MatrixXd sample_points_2d(const MatrixXd& P, int maxSamples)
{
    if (P.rows() <= maxSamples)
    {
        return P;
    }

    MatrixXd samples(maxSamples, 2);
    const int last = P.rows() - 1;
    const int denom = std::max(1, maxSamples - 1);
    for (int i = 0; i < maxSamples; ++i)
    {
        const int index = static_cast<int>(std::lround(double(i) * double(last) / double(denom)));
        samples.row(i) = P.row(index);
    }

    return samples;
}

double cross2d(const Eigen::Vector2d& O, const Eigen::Vector2d& A, const Eigen::Vector2d& B)
{
    return (A.x() - O.x()) * (B.y() - O.y()) - (A.y() - O.y()) * (B.x() - O.x());
}

MatrixXd convex_hull_2d(const MatrixXd& P)
{
    if (P.rows() <= 2)
    {
        return P;
    }

    std::vector<Eigen::Vector2d> pts;
    pts.reserve(P.rows());
    for (int i = 0; i < P.rows(); ++i)
    {
        pts.emplace_back(P(i, 0), P(i, 1));
    }

    std::sort(pts.begin(), pts.end(), [](const Eigen::Vector2d& a, const Eigen::Vector2d& b)
    {
        if (a.x() == b.x())
        {
            return a.y() < b.y();
        }
        return a.x() < b.x();
    });

    std::vector<Eigen::Vector2d> uniquePts;
    uniquePts.reserve(pts.size());
    for (const auto& p : pts)
    {
        if (uniquePts.empty() || (p - uniquePts.back()).norm() > 1e-12)
        {
            uniquePts.push_back(p);
        }
    }

    if (uniquePts.size() <= 2)
    {
        MatrixXd out(uniquePts.size(), 2);
        for (int i = 0; i < static_cast<int>(uniquePts.size()); ++i)
        {
            out.row(i) = uniquePts[i].transpose();
        }
        return out;
    }

    std::vector<Eigen::Vector2d> H(2 * uniquePts.size());
    int k = 0;

    for (std::size_t i = 0; i < uniquePts.size(); ++i)
    {
        while (k >= 2 && cross2d(H[k - 2], H[k - 1], uniquePts[i]) <= 0.0)
        {
            --k;
        }
        H[k++] = uniquePts[i];
    }

    for (int i = static_cast<int>(uniquePts.size()) - 2, t = k + 1; i >= 0; --i)
    {
        while (k >= t && cross2d(H[k - 2], H[k - 1], uniquePts[i]) <= 0.0)
        {
            --k;
        }
        H[k++] = uniquePts[i];
    }

    if (k > 1)
    {
        --k;
    }

    MatrixXd hull(k, 2);
    for (int i = 0; i < k; ++i)
    {
        hull.row(i) = H[i].transpose();
    }
    return hull;
}

MatrixXd sample_closed_contour_2d(const MatrixXd& contour, int sampleCount)
{
    if (contour.rows() == 0)
    {
        return MatrixXd(0, 2);
    }
    if (contour.rows() == 1)
    {
        MatrixXd out = MatrixXd::Zero(sampleCount, 2);
        for (int i = 0; i < sampleCount; ++i)
        {
            out.row(i) = contour.row(0);
        }
        return out;
    }

    std::vector<double> lengths(contour.rows(), 0.0);
    double total = 0.0;
    for (int i = 0; i < contour.rows(); ++i)
    {
        const int j = (i + 1) % contour.rows();
        lengths[i] = (contour.row(j) - contour.row(i)).norm();
        total += lengths[i];
    }

    if (total <= 1e-12)
    {
        return sample_points_2d(contour, sampleCount);
    }

    MatrixXd samples(sampleCount, 2);
    for (int s = 0; s < sampleCount; ++s)
    {
        const double target = (double(s) / double(sampleCount)) * total;
        double acc = 0.0;
        int edge = 0;
        while (edge < contour.rows() - 1 && acc + lengths[edge] < target)
        {
            acc += lengths[edge];
            ++edge;
        }

        const int next = (edge + 1) % contour.rows();
        const double edgeLen = std::max(lengths[edge], 1e-12);
        const double t = (target - acc) / edgeLen;
        samples.row(s) = (1.0 - t) * contour.row(edge) + t * contour.row(next);
    }

    return samples;
}

Eigen::Matrix2d pca_frame_2d(const MatrixXd& P)
{
    MatrixXd centered = P.rowwise() - P.colwise().mean();
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    if (centered.rows() > 1)
    {
        covariance = (centered.transpose() * centered) / double(centered.rows() - 1);
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    Eigen::Matrix2d frame = Eigen::Matrix2d::Zero();
    std::array<int, 2> order = {0, 1};
    std::sort(order.begin(), order.end(), [&](int a, int b)
    {
        return solver.eigenvalues()(a) > solver.eigenvalues()(b);
    });

    frame.col(0) = solver.eigenvectors().col(order[0]);
    frame.col(1) = solver.eigenvectors().col(order[1]);
    return frame;
}

MatrixXd normalize_and_center_2d(MatrixXd P)
{
    if (P.rows() == 0)
    {
        return P;
    }

    P = P.rowwise() - P.colwise().mean();
    const double scale = P.cwiseAbs().maxCoeff();
    if (scale > 1e-10)
    {
        P /= scale;
    }
    return P;
}

MatrixXd align_to_pca_2d(const MatrixXd& P)
{
    if (P.rows() == 0)
    {
        return P;
    }

    const Eigen::Matrix2d frame = pca_frame_2d(P);
    return P * frame;
}

MatrixXd reflect_axes(const MatrixXd& V, const std::array<int, 3>& signs)
{
    MatrixXd reflected = V;
    for (int axis = 0; axis < 3; ++axis)
    {
        reflected.col(axis) *= double(signs[axis]);
    }
    return reflected;
}

double cloud_distance(const MatrixXd& A, const MatrixXd& B)
{
    if (A.rows() == 0 || B.rows() == 0)
    {
        return std::numeric_limits<double>::infinity();
    }

    double total = 0.0;
    for (int i = 0; i < A.rows(); ++i)
    {
        double best = std::numeric_limits<double>::infinity();
        for (int j = 0; j < B.rows(); ++j)
        {
            const double d = (A.row(i) - B.row(j)).squaredNorm();
            if (d < best)
            {
                best = d;
            }
        }
        total += best;
    }

    return total / double(A.rows());
}

double symmetric_cloud_distance(const MatrixXd& A, const MatrixXd& B)
{
    return 0.5 * (cloud_distance(A, B) + cloud_distance(B, A));
}

RecognitionResult classify_against_templates(const MatrixXd& querySamples, const std::vector<TemplateMesh>& library)
{
    RecognitionResult result;
    if (library.empty() || querySamples.rows() == 0)
    {
        result.label = "unknown";
        return result;
    }

    const std::array<std::array<int, 3>, 8> signPatterns = {{
        std::array<int, 3>{ 1,  1,  1},
        std::array<int, 3>{ 1,  1, -1},
        std::array<int, 3>{ 1, -1,  1},
        std::array<int, 3>{ 1, -1, -1},
        std::array<int, 3>{-1,  1,  1},
        std::array<int, 3>{-1,  1, -1},
        std::array<int, 3>{-1, -1,  1},
        std::array<int, 3>{-1, -1, -1}
    }};

    std::vector<std::pair<std::string, double>> ranking;
    ranking.reserve(library.size());

    for (const TemplateMesh& templateMesh : library)
    {
        double bestScore = std::numeric_limits<double>::infinity();
        for (const auto& signs : signPatterns)
        {
            const MatrixXd reflected = reflect_axes(querySamples, signs);
            const double score = symmetric_cloud_distance(reflected, templateMesh.samples);
            if (score < bestScore)
            {
                bestScore = score;
            }
        }

        ranking.emplace_back(templateMesh.name, bestScore);
    }

    std::sort(ranking.begin(), ranking.end(), [](const std::pair<std::string, double>& a, const std::pair<std::string, double>& b)
    {
        return a.second < b.second;
    });

    result.ranking = ranking;
    result.label = ranking.front().first;
    result.score = ranking.front().second;
    return result;
}

RecognitionResult classify_mesh(const MatrixXd& querySamples)
{
    return classify_against_templates(querySamples, meshTemplates);
}

RecognitionResult classify_primitive(const MatrixXd& querySamples)
{
    return classify_against_templates(querySamples, primitiveTemplates);
}

RecognitionResult classify_primitive_2d(const MatrixXd& querySamples)
{
    RecognitionResult result;
    if (primitiveTemplates.empty() || querySamples.rows() == 0)
    {
        result.label = "unknown";
        return result;
    }

    const std::array<std::array<int, 2>, 4> signPatterns = {{
        std::array<int, 2>{ 1,  1},
        std::array<int, 2>{ 1, -1},
        std::array<int, 2>{-1,  1},
        std::array<int, 2>{-1, -1}
    }};

    std::vector<std::pair<std::string, double>> ranking;
    ranking.reserve(primitiveTemplates.size());

    for (const TemplateMesh& primitive : primitiveTemplates)
    {
        if (primitive.samples2dVariants.empty())
        {
            continue;
        }

        double bestScore = std::numeric_limits<double>::infinity();
        for (const MatrixXd& variant : primitive.samples2dVariants)
        {
            if (variant.rows() == 0)
            {
                continue;
            }

            for (const auto& signs : signPatterns)
            {
                MatrixXd reflected = querySamples;
                reflected.col(0) *= double(signs[0]);
                reflected.col(1) *= double(signs[1]);

                const double score = symmetric_cloud_distance(reflected, variant);
                if (score < bestScore)
                {
                    bestScore = score;
                }
            }
        }

        ranking.emplace_back(primitive.name, bestScore);
    }

    if (ranking.empty())
    {
        result.label = "unknown";
        return result;
    }

    std::sort(ranking.begin(), ranking.end(), [](const std::pair<std::string, double>& a, const std::pair<std::string, double>& b)
    {
        return a.second < b.second;
    });

    result.ranking = ranking;
    result.label = ranking.front().first;
    result.score = ranking.front().second;
    return result;
}

int estimate_corner_count(const MatrixXd& P, bool closed)
{
    if (P.rows() < 10)
    {
        return 0;
    }

    const int n = P.rows();
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
        if (!closed && (im1 < 0 || ip1 >= n))
        {
            continue;
        }

        Eigen::Vector2d e1 = P.row(i) - P.row(im1);
        Eigen::Vector2d e2 = P.row(ip1) - P.row(i);
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

StrokeFeatures extract_stroke_features(const MatrixXd& stroke2d)
{
    StrokeFeatures f;
    if (stroke2d.rows() < 3)
    {
        return f;
    }

    const Eigen::RowVector2d minP = stroke2d.colwise().minCoeff();
    const Eigen::RowVector2d maxP = stroke2d.colwise().maxCoeff();
    const double w = std::max(1e-8, maxP(0) - minP(0));
    const double h = std::max(1e-8, maxP(1) - minP(1));
    f.aspectRatio = w / h;

    const double diag = std::sqrt(w * w + h * h);
    const Eigen::Vector2d first = stroke2d.row(0);
    const Eigen::Vector2d last = stroke2d.row(stroke2d.rows() - 1);
    f.isClosed = ((first - last).norm() < 0.2 * diag);

    double perimeter = 0.0;
    for (int i = 1; i < stroke2d.rows(); ++i)
    {
        perimeter += (stroke2d.row(i) - stroke2d.row(i - 1)).norm();
    }
    if (f.isClosed)
    {
        perimeter += (stroke2d.row(0) - stroke2d.row(stroke2d.rows() - 1)).norm();
    }

    double area = 0.0;
    if (f.isClosed)
    {
        for (int i = 0; i < stroke2d.rows(); ++i)
        {
            const int j = (i + 1) % stroke2d.rows();
            area += stroke2d(i, 0) * stroke2d(j, 1) - stroke2d(j, 0) * stroke2d(i, 1);
        }
        area = 0.5 * std::abs(area);
    }

    if (perimeter > 1e-8)
    {
        f.circularity = 4.0 * M_PI * area / (perimeter * perimeter);
    }

    // Distinguish tapered shapes: cone-like (very small end) vs frustum-like (both ends non-zero).
    const double rangeX = maxP(0) - minP(0);
    const double rangeY = maxP(1) - minP(1);
    const int majorAxis = (rangeX >= rangeY) ? 0 : 1;
    const int minorAxis = 1 - majorAxis;

    const double minMajor = stroke2d.col(majorAxis).minCoeff();
    const double maxMajor = stroke2d.col(majorAxis).maxCoeff();
    const double majorRange = std::max(1e-8, maxMajor - minMajor);
    const double band = 0.20 * majorRange;

    double lowMin = std::numeric_limits<double>::infinity();
    double lowMax = -std::numeric_limits<double>::infinity();
    double highMin = std::numeric_limits<double>::infinity();
    double highMax = -std::numeric_limits<double>::infinity();
    int lowCount = 0;
    int highCount = 0;

    for (int i = 0; i < stroke2d.rows(); ++i)
    {
        const double major = stroke2d(i, majorAxis);
        const double minor = stroke2d(i, minorAxis);

        if (major - minMajor <= band)
        {
            lowMin = std::min(lowMin, minor);
            lowMax = std::max(lowMax, minor);
            lowCount += 1;
        }
        if (maxMajor - major <= band)
        {
            highMin = std::min(highMin, minor);
            highMax = std::max(highMax, minor);
            highCount += 1;
        }
    }

    if (lowCount >= 2 && highCount >= 2)
    {
        const double lowWidth = std::max(1e-8, lowMax - lowMin);
        const double highWidth = std::max(1e-8, highMax - highMin);
        f.endWidthRatio = std::min(lowWidth, highWidth) / std::max(lowWidth, highWidth);
    }

    f.cornerCount = estimate_corner_count(stroke2d, f.isClosed);
    return f;
}

double primitive_score_bias(const std::string& name, const StrokeFeatures& f)
{
    double bias = 0.0;

    // Keep only a very soft bias so classification is data-driven.
    if (f.cornerCount >= 4 && std::abs(f.aspectRatio - 1.0) < 0.35)
    {
        if (name == "cube")
        {
            bias -= 0.04;
        }
    }

    if (f.isClosed && f.circularity > 0.84)
    {
        if (name == "sphere")
        {
            bias -= 0.04;
        }
    }

    if (name == "pyramid" && f.isClosed && f.cornerCount == 3)
    {
        bias -= 0.15; // Donne un fort bonus si 3 coins sont détectés
    }



    return bias;
}

RecognitionResult classify_primitive_2d_with_features(const MatrixXd& querySamples, const StrokeFeatures& features)
{
    RecognitionResult result;
    if (primitiveTemplates.empty() || querySamples.rows() == 0)
    {
        result.label = "unknown";
        return result;
    }

    const std::array<std::array<int, 2>, 4> signPatterns = {{
        std::array<int, 2>{ 1,  1},
        std::array<int, 2>{ 1, -1},
        std::array<int, 2>{-1,  1},
        std::array<int, 2>{-1, -1}
    }};

    std::vector<std::pair<std::string, double>> ranking;
    ranking.reserve(primitiveTemplates.size());

    for (const TemplateMesh& primitive : primitiveTemplates)
    {
        if (primitive.samples2dVariants.empty())
        {
            continue;
        }

        double bestScore = std::numeric_limits<double>::infinity();
        for (std::size_t vid = 0; vid < primitive.samples2dVariants.size(); ++vid)
        {
            const MatrixXd& variant = primitive.samples2dVariants[vid];
            if (variant.rows() == 0)
            {
                continue;
            }
            const double variantCircularity = (vid < primitive.circularity2dVariants.size()) ? primitive.circularity2dVariants[vid] : 0.0;

            for (const auto& signs : signPatterns)
            {
                MatrixXd reflected = querySamples;
                reflected.col(0) *= double(signs[0]);
                reflected.col(1) *= double(signs[1]);

                const double geomScore = symmetric_cloud_distance(reflected, variant);
                const double circScore = std::abs(features.circularity - variantCircularity);
                const double score = geomScore + 0.30 * circScore;
                if (score < bestScore)
                {
                    bestScore = score;
                }
            }
        }

        if (!std::isfinite(bestScore))
        {
            continue;
        }

        bestScore += primitive_score_bias(primitive.name, features);
        ranking.emplace_back(primitive.name, bestScore);
    }

    if (ranking.empty())
    {
        return classify_primitive_2d(querySamples);
    }

    std::sort(ranking.begin(), ranking.end(), [](const std::pair<std::string, double>& a, const std::pair<std::string, double>& b)
    {
        return a.second < b.second;
    });

    result.ranking = ranking;
    result.label = ranking.front().first;
    result.score = ranking.front().second;
    return result;
}

const TemplateMesh* find_primitive_template(const std::string& label)
{
    for (const TemplateMesh& templateMesh : primitiveTemplates)
    {
        if (templateMesh.name == label)
        {
            return &templateMesh;
        }
    }
    return nullptr;
}

bool load_mesh_file(const std::string& path, MatrixXd& V, MatrixXi& F)
{
    if (!igl::readOBJ(path, V, F))
    {
        return false;
    }

    if (V.rows() == 0 || F.rows() == 0)
    {
        return false;
    }

    V = normalize_and_center(V);
    return true;
}

void refresh_prediction();
float draw_plane_depth(const igl::opengl::glfw::Viewer& viewer);
void clear_stroke();

RowVector3d primitive_center(const PrimitiveInstance& primitive)
{
    if (primitive.V.rows() == 0)
    {
        return RowVector3d::Zero();
    }
    return primitive.V.colwise().mean();
}

bool project_mouse_at_depth(const igl::opengl::glfw::Viewer& viewer, int mouseX, int mouseY, float depth, RowVector3d& point)
{
    const Eigen::Vector3f win(
        static_cast<float>(mouseX),
        static_cast<float>(viewer.core().viewport(3) - mouseY),
        depth);
    const Eigen::Vector3f world = igl::unproject(win, viewer.core().view, viewer.core().proj, viewer.core().viewport);
    point = world.cast<double>();
    return true;
}

void draw_axis_gizmo_top_left(const igl::opengl::glfw::Viewer& viewer)
{
    ImDrawList* drawList = ImGui::GetForegroundDrawList();
    if (drawList == nullptr)
    {
        return;
    }

    const ImVec2 displaySize = ImGui::GetIO().DisplaySize;
    const ImVec2 origin(displaySize.x - 70.0f, 70.0f);
    const float radius = 46.0f;
    const float axisLen = 34.0f;

    drawList->AddCircleFilled(origin, radius, IM_COL32(20, 24, 28, 170), 32);
    drawList->AddCircle(origin, radius, IM_COL32(230, 235, 240, 170), 32, 1.0f);

    const Eigen::Matrix3f R = viewer.core().view.block<3, 3>(0, 0);
    const std::array<Vector3d, 3> worldAxes = { Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ() };
    const std::array<ImU32, 3> colors = {
        IM_COL32(235, 70, 70, 255),
        IM_COL32(80, 210, 110, 255),
        IM_COL32(70, 140, 245, 255)
    };
    const std::array<const char*, 3> labels = {"X", "Y", "Z"};

    // Draw farther axes first so nearer ones are on top.
    std::array<int, 3> order = {0, 1, 2};
    std::sort(order.begin(), order.end(), [&](int a, int b)
    {
        const Eigen::Vector3f ca = R * worldAxes[a].cast<float>();
        const Eigen::Vector3f cb = R * worldAxes[b].cast<float>();
        return ca(2) < cb(2);
    });

    for (int idx : order)
    {
        const Eigen::Vector3f camAxis = R * worldAxes[idx].cast<float>();
        const ImVec2 end(
            origin.x + axisLen * camAxis(0),
            origin.y - axisLen * camAxis(1));

        drawList->AddLine(origin, end, colors[idx], 2.5f);
        drawList->AddCircleFilled(end, 4.0f, colors[idx], 12);
        drawList->AddText(ImVec2(end.x + 6.0f, end.y - 8.0f), colors[idx], labels[idx]);
    }
}

bool point_in_triangle_2d(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c)
{
    const double c1 = cross2d(a, b, p);
    const double c2 = cross2d(b, c, p);
    const double c3 = cross2d(c, a, p);
    const bool hasNeg = (c1 < 0.0) || (c2 < 0.0) || (c3 < 0.0);
    const bool hasPos = (c1 > 0.0) || (c2 > 0.0) || (c3 > 0.0);
    return !(hasNeg && hasPos);
}

double point_segment_distance2_2d(const Eigen::Vector2d& p, const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
    const Eigen::Vector2d ab = b - a;
    const double denom = ab.squaredNorm();
    if (denom <= 1e-16)
    {
        return (p - a).squaredNorm();
    }
    const double t = std::max(0.0, std::min(1.0, (p - a).dot(ab) / denom));
    const Eigen::Vector2d proj = a + t * ab;
    return (p - proj).squaredNorm();
}

bool pick_primitive_by_mouse(const igl::opengl::glfw::Viewer& viewer, int mouseX, int mouseY, int& primitiveIndex)
{
    primitiveIndex = -1;
    if (state.primitives.empty())
    {
        return false;
    }

    const double sx = double(mouseX);
    const double sy = double(viewer.core().viewport(3) - mouseY);
    const Eigen::Vector2d mousePt(sx, sy);
    const double maxEdgePickDistance2 = 12.0 * 12.0;
    const double maxVertexPickDistance2 = 24.0 * 24.0;

    bool bestIsFaceHit = false;
    double bestScore = std::numeric_limits<double>::infinity();
    int bestIndex = -1;

    for (int i = 0; i < static_cast<int>(state.primitives.size()); ++i)
    {
        const PrimitiveInstance& primitive = state.primitives[i];
        if (primitive.V.rows() == 0)
        {
            continue;
        }

        bool primitiveFaceHit = false;
        double primitiveBestDepth = std::numeric_limits<double>::infinity();
        double primitiveBestEdgeD2 = std::numeric_limits<double>::infinity();

        for (int f = 0; f < primitive.F.rows(); ++f)
        {
            const int ia = primitive.F(f, 0);
            const int ib = primitive.F(f, 1);
            const int ic = primitive.F(f, 2);
            if (ia < 0 || ib < 0 || ic < 0 || ia >= primitive.V.rows() || ib >= primitive.V.rows() || ic >= primitive.V.rows())
            {
                continue;
            }

            const Eigen::Vector3f a3 = primitive.V.row(ia).transpose().cast<float>();
            const Eigen::Vector3f b3 = primitive.V.row(ib).transpose().cast<float>();
            const Eigen::Vector3f c3 = primitive.V.row(ic).transpose().cast<float>();

            const Eigen::Vector3f ap = igl::project(a3, viewer.core().view, viewer.core().proj, viewer.core().viewport);
            const Eigen::Vector3f bp = igl::project(b3, viewer.core().view, viewer.core().proj, viewer.core().viewport);
            const Eigen::Vector3f cp = igl::project(c3, viewer.core().view, viewer.core().proj, viewer.core().viewport);

            const Eigen::Vector2d a2(ap(0), ap(1));
            const Eigen::Vector2d b2(bp(0), bp(1));
            const Eigen::Vector2d c2(cp(0), cp(1));

            if (point_in_triangle_2d(mousePt, a2, b2, c2))
            {
                primitiveFaceHit = true;
                const double depth = (double(ap(2)) + double(bp(2)) + double(cp(2))) / 3.0;
                if (depth < primitiveBestDepth)
                {
                    primitiveBestDepth = depth;
                }
            }

            const double e0 = point_segment_distance2_2d(mousePt, a2, b2);
            const double e1 = point_segment_distance2_2d(mousePt, b2, c2);
            const double e2 = point_segment_distance2_2d(mousePt, c2, a2);
            primitiveBestEdgeD2 = std::min(primitiveBestEdgeD2, std::min(e0, std::min(e1, e2)));
        }

        if (primitiveFaceHit)
        {
            if (!bestIsFaceHit || primitiveBestDepth < bestScore)
            {
                bestIsFaceHit = true;
                bestScore = primitiveBestDepth;
                bestIndex = i;
            }
            continue;
        }

        if (bestIsFaceHit)
        {
            continue;
        }

        if (primitiveBestEdgeD2 < maxEdgePickDistance2 && primitiveBestEdgeD2 < bestScore)
        {
            bestScore = primitiveBestEdgeD2;
            bestIndex = i;
            continue;
        }

        // Fallback for very sparse meshes.
        for (int v = 0; v < primitive.V.rows(); ++v)
        {
            const Eigen::Vector3f p = primitive.V.row(v).transpose().cast<float>();
            const Eigen::Vector3f pp = igl::project(p, viewer.core().view, viewer.core().proj, viewer.core().viewport);
            const double dx = double(pp(0)) - sx;
            const double dy = double(pp(1)) - sy;
            const double d2 = dx * dx + dy * dy;
            if (d2 < maxVertexPickDistance2 && d2 < bestScore)
            {
                bestScore = d2;
                bestIndex = i;
            }
        }
    }

    if (bestIndex >= 0)
    {
        primitiveIndex = bestIndex;
        return true;
    }
    return false;
}

bool has_selected_primitive()
{
    return state.selectedPrimitive >= 0 && state.selectedPrimitive < static_cast<int>(state.primitives.size());
}

// Generate a pyramid with n-sided base
void generate_pyramid_geometry(int sides, double baseRadius, double height, MatrixXd& V, MatrixXi& F)
{
    if (sides < 3)
    {
        sides = 3;
    }

    V.resize(sides + 1, 3);
    
    // Base vertices
    for (int i = 0; i < sides; ++i)
    {
        const double angle = 2.0 * M_PI * i / sides;
        V(i, 0) = baseRadius * std::cos(angle);
        V(i, 1) = baseRadius * std::sin(angle);
        V(i, 2) = 0.0;
    }
    
    // Apex vertex
    V(sides, 0) = 0.0;
    V(sides, 1) = 0.0;
    V(sides, 2) = height;
    
    // Faces: triangular sides + base
    int triangleCount = sides; // sides faces
    int baseTriangles = std::max(1, sides - 2); // tessellate base as fan
    F.resize(triangleCount + baseTriangles, 3);
    
    // Side faces (triangles from base edges to apex)
    for (int i = 0; i < sides; ++i)
    {
        const int next = (i + 1) % sides;
        F(i, 0) = i;
        F(i, 1) = next;
        F(i, 2) = sides; // apex
    }
    
    // Base faces (fan triangulation)
    for (int i = 0; i < baseTriangles; ++i)
    {
        F(triangleCount + i, 0) = 0;
        F(triangleCount + i, 1) = i + 1;
        F(triangleCount + i, 2) = i + 2;
    }
}

void rotate_selected_primitive(const Vector3d& axis, double angleRadians)
{
    if (!has_selected_primitive())
    {
        return;
    }

    PrimitiveInstance& primitive = state.primitives[state.selectedPrimitive];
    if (primitive.V.rows() == 0)
    {
        return;
    }

    const RowVector3d center = primitive_center(primitive);
    const Eigen::Matrix3d R = Eigen::AngleAxisd(angleRadians, axis.normalized()).toRotationMatrix();
    for (int i = 0; i < primitive.V.rows(); ++i)
    {
        const Vector3d p = primitive.V.row(i).transpose() - center.transpose();
        primitive.V.row(i) = (R * p + center.transpose()).transpose();
    }

    if (primitive.label == "pyramid")
    {
        primitive.pyramidOrientation = R * primitive.pyramidOrientation;
    }
}

void scale_selected_primitive(double scaleFactor)
{
    if (!has_selected_primitive() || scaleFactor <= 1e-8)
    {
        return;
    }

    PrimitiveInstance& primitive = state.primitives[state.selectedPrimitive];
    if (primitive.V.rows() == 0)
    {
        return;
    }

    const RowVector3d center = primitive_center(primitive);
    for (int i = 0; i < primitive.V.rows(); ++i)
    {
        const Vector3d p = primitive.V.row(i).transpose() - center.transpose();
        primitive.V.row(i) = (scaleFactor * p + center.transpose()).transpose();
    }



    if (primitive.label == "sphere" || primitive.label == "cube" || primitive.label == "pyramid")
    {
        primitive.scale *= scaleFactor;
    }
}

void scale_selected_primitive_nonuniform(const RowVector3d& scaleFactors)
{
    if (!has_selected_primitive())
    {
        return;
    }

    PrimitiveInstance& primitive = state.primitives[state.selectedPrimitive];
    if (primitive.V.rows() == 0)
    {
        return;
    }

    const double sx = std::max(1e-8, scaleFactors(0));
    const double sy = std::max(1e-8, scaleFactors(1));
    const double sz = std::max(1e-8, scaleFactors(2));

    const RowVector3d center = primitive_center(primitive);
    for (int i = 0; i < primitive.V.rows(); ++i)
    {
        const Vector3d p = primitive.V.row(i).transpose() - center.transpose();
        Vector3d q = p;
        q(0) *= sx;
        q(1) *= sy;
        q(2) *= sz;
        primitive.V.row(i) = (q + center.transpose()).transpose();
    }

    if (primitive.label == "sphere" || primitive.label == "cube" || primitive.label == "pyramid")
    {
        primitive.scale(0) *= sx;
        primitive.scale(1) *= sy;
        primitive.scale(2) *= sz;
    }
}

void update_selected_pyramid()
{
    if (!has_selected_primitive())
    {
        return;
    }

    PrimitiveInstance& primitive = state.primitives[state.selectedPrimitive];
    if (primitive.label != "pyramid")
    {
        return;
    }

    // Keep pyramid direction fixed using stored orientation (stable under large base scaling).
    Matrix3d orientation = primitive.pyramidOrientation;
    if (!orientation.allFinite() || std::abs(orientation.determinant()) < 1e-10)
    {
        orientation = Matrix3d::Identity();
    }
    else
    {
        Eigen::Quaterniond q(orientation);
        if (q.norm() > 1e-12)
        {
            q.normalize();
            orientation = q.toRotationMatrix();
        }
        else
        {
            orientation = Matrix3d::Identity();
        }
    }

    RowVector3d oldBaseCenter = RowVector3d::Zero();
    const int oldSides = std::min(std::max(3, primitive.pyramidSides), std::max(0, static_cast<int>(primitive.V.rows()) - 1));
    if (oldSides > 0)
    {
        for (int i = 0; i < oldSides; ++i)
        {
            oldBaseCenter += primitive.V.row(i);
        }
        oldBaseCenter /= static_cast<double>(oldSides);
    }
    else
    {
        oldBaseCenter = primitive_center(primitive);
    }

    // Generate new pyramid geometry.
    MatrixXd V;
    MatrixXi F;
    generate_pyramid_geometry(state.uiPyramidSides, state.uiPyramidBaseSize, state.uiPyramidHeight, V, F);

    // Recenter locally, then apply stored orientation.
    const RowVector3d newLocalCenter = V.colwise().mean();
    V = V.rowwise() - newLocalCenter;

    const Matrix3d R = orientation;
    for (int i = 0; i < V.rows(); ++i)
    {
        const Vector3d p = V.row(i).transpose();
        V.row(i) = (R * p).transpose();
    }

    // Keep base center fixed in world space.
    RowVector3d newBaseCenter = RowVector3d::Zero();
    const int newSides = std::max(3, state.uiPyramidSides);
    for (int i = 0; i < newSides; ++i)
    {
        newBaseCenter += V.row(i);
    }
    newBaseCenter /= static_cast<double>(newSides);

    const RowVector3d translation = oldBaseCenter - newBaseCenter;

    // Update the primitive
    primitive.V = V.rowwise() + translation;
    primitive.F = F;
    primitive.pyramidSides = state.uiPyramidSides;
    primitive.pyramidBaseSize = state.uiPyramidBaseSize;
    primitive.pyramidHeight = state.uiPyramidHeight;
    primitive.pyramidOrientation = orientation;

    // Update scale tracking
    state.uiPyramidSidesPrev = state.uiPyramidSides;
    state.uiPyramidBaseSizePrev = state.uiPyramidBaseSize;
    state.uiPyramidHeightPrev = state.uiPyramidHeight;
}

double wrap_degrees_180(double degrees)
{
    while (degrees > 180.0)
    {
        degrees -= 360.0;
    }
    while (degrees < -180.0)
    {
        degrees += 360.0;
    }
    return degrees;
}

void update_ui_rotation_from_keyboard(const Vector3d& deltaDeg)
{
    if (!has_selected_primitive())
    {
        return;
    }

    for (int axis = 0; axis < 3; ++axis)
    {
        const double v = wrap_degrees_180(state.uiRotateDeg(axis) + deltaDeg(axis));
        state.uiRotateDeg(axis) = v;
        state.uiRotateDegPrev(axis) = v;
    }
}

void translate_selected_primitive(const RowVector3d& delta)
{
    if (!has_selected_primitive())
    {
        return;
    }

    PrimitiveInstance& primitive = state.primitives[state.selectedPrimitive];
    if (primitive.V.rows() == 0)
    {
        return;
    }

    primitive.V.rowwise() += delta;
}

RowVector3d mesh_spawn_reference_point()
{
    // Meshes added from the library are currently inserted centered at world origin.
    return RowVector3d::Zero();
}

void sync_transform_ui_with_selection(bool force = false)
{
    if (!force && state.uiSelectionPrimitive == state.selectedPrimitive)
    {
        return;
    }

    state.uiSelectionPrimitive = state.selectedPrimitive;
    state.uiRotateDeg = RowVector3d::Zero();
    state.uiRotateDegPrev = RowVector3d::Zero();
    state.uiScale = RowVector3d::Ones();
    state.uiScalePrev = RowVector3d::Ones();
    state.uiPyramidSides = 5;
    state.uiPyramidSidesPrev = 5;
    state.uiPyramidBaseSize = 1.0f;
    state.uiPyramidBaseSizePrev = 1.0f;
    state.uiPyramidHeight = 1.0f;
    state.uiPyramidHeightPrev = 1.0f;

    if (has_selected_primitive())
    {
        const RowVector3d center = primitive_center(state.primitives[state.selectedPrimitive]);
        const RowVector3d spawnRef = mesh_spawn_reference_point();
        state.uiTranslate = center - spawnRef;
        state.uiTranslatePrev = state.uiTranslate;

        const PrimitiveInstance& selected = state.primitives[state.selectedPrimitive];
        if (selected.label == "sphere" || selected.label == "cube" || selected.label == "pyramid")
        {
            state.uiScale = selected.scale;
            state.uiScalePrev = selected.scale;
        }

        if (selected.label == "pyramid")
        {
            state.uiPyramidSides = selected.pyramidSides;
            state.uiPyramidSidesPrev = selected.pyramidSides;
            state.uiPyramidBaseSize = static_cast<float>(selected.pyramidBaseSize);
            state.uiPyramidBaseSizePrev = static_cast<float>(selected.pyramidBaseSize);
            state.uiPyramidHeight = static_cast<float>(selected.pyramidHeight);
            state.uiPyramidHeightPrev = static_cast<float>(selected.pyramidHeight);
        }
    }
    else
    {
        state.uiTranslate = RowVector3d::Zero();
        state.uiTranslatePrev = RowVector3d::Zero();
    }
}

void add_mesh_from_library(int meshIndex)
{
    if (meshIndex < 0 || meshIndex >= static_cast<int>(meshTemplates.size()))
    {
        return;
    }

    const TemplateMesh& meshTemplate = meshTemplates[meshIndex];
    if (meshTemplate.V.rows() == 0 || meshTemplate.F.rows() == 0)
    {
        return;
    }

    PrimitiveInstance meshInstance;
    meshInstance.label = meshTemplate.name;
    meshInstance.V = meshTemplate.V;
    meshInstance.F = meshTemplate.F;
    meshInstance.color = Eigen::RowVector4d(200.0 / 255.0, 200.0 / 255.0, 200.0 / 255.0, 1.0);

    state.primitives.push_back(meshInstance);
}

void draw_scene(igl::opengl::glfw::Viewer& viewer)
{
    const Eigen::RowVector4d meshColor(200.0 / 255.0, 200.0 / 255.0, 200.0 / 255.0, 1.0);
    const RowVector3d strokeColor(250.0 / 255.0, 114.0 / 255.0, 104.0 / 255.0);

    MatrixXd renderV = state.meshV;
    MatrixXi renderF = state.meshF;
    std::vector<Eigen::RowVector4d> faceColors;
    faceColors.reserve(state.meshF.rows());
    for (int i = 0; i < state.meshF.rows(); ++i)
    {
        faceColors.push_back(meshColor);
    }

    auto append_primitive = [&](int pid)
    {
        const PrimitiveInstance& primitive = state.primitives[pid];
        if (primitive.V.rows() == 0 || primitive.F.rows() == 0)
        {
            return;
        }

        const int offset = renderV.rows();

        const int oldVRows = renderV.rows();
        renderV.conservativeResize(oldVRows + primitive.V.rows(), 3);
        renderV.bottomRows(primitive.V.rows()) = primitive.V;

        const int oldFRows = renderF.rows();
        renderF.conservativeResize(oldFRows + primitive.F.rows(), 3);
        renderF.bottomRows(primitive.F.rows()) = (primitive.F.array() + offset).matrix();

        Eigen::RowVector4d faceColor = primitive.color;
        if (pid == state.selectedPrimitive)
        {
            // Slightly highlight selected object without changing opacity semantics.
            faceColor.head<3>() = (faceColor.head<3>().array() + 0.15).min(1.0);
        }

        for (int i = 0; i < primitive.F.rows(); ++i)
        {
            faceColors.push_back(faceColor);
        }
    };

    // Draw opaque objects first, then transparent ones.
    for (int pid = 0; pid < static_cast<int>(state.primitives.size()); ++pid)
    {
        const double alpha = state.primitives[pid].color(3);
        if (alpha >= 0.999)
        {
            append_primitive(pid);
        }
    }

    for (int pid = 0; pid < static_cast<int>(state.primitives.size()); ++pid)
    {
        const double alpha = state.primitives[pid].color(3);
        if (alpha < 0.999)
        {
            append_primitive(pid);
        }
    }

    viewer.data().clear();
    viewer.data().face_based = true;
    viewer.data().set_mesh(renderV, renderF);
    viewer.data().line_width = 1.0f;
    viewer.data().point_size = 4.0f;

    MatrixXd C(renderF.rows(), 4);
    for (int i = 0; i < renderF.rows(); ++i)
    {
        C.row(i) = faceColors[i];
    }
    viewer.data().set_colors(C);

    if (state.strokePoints.rows() > 0)
    {
        viewer.data().add_points(state.strokePoints, strokeColor);
        for (int i = 1; i < state.strokePoints.rows(); ++i)
        {
            viewer.data().add_edges(state.strokePoints.row(i - 1), state.strokePoints.row(i), strokeColor);
        }
    }
}

float draw_plane_depth(const igl::opengl::glfw::Viewer& viewer)
{
    if (state.meshV.rows() == 0)
    {
        return 0.5f;
    }

    const Eigen::Vector3f center = state.meshV.colwise().mean().transpose().cast<float>();
    const Eigen::Vector3f projected = igl::project(center, viewer.core().view, viewer.core().proj, viewer.core().viewport);
    return projected(2);
}

bool project_mouse_to_draw_plane(igl::opengl::glfw::Viewer& viewer, int mouseX, int mouseY, RowVector3d& point)
{
    const float depth = draw_plane_depth(viewer);
    const Eigen::Vector3f win(
        static_cast<float>(mouseX),
        static_cast<float>(viewer.core().viewport(3) - mouseY),
        depth);

    const Eigen::Vector3f world = igl::unproject(win, viewer.core().view, viewer.core().proj, viewer.core().viewport);
    point = world.cast<double>();
    return true;
}

// Rule-based classifier using directly measurable stroke geometry descriptors.
// More robust than contour cloud matching which biases toward "average" polygons.
RecognitionResult classify_stroke()
{
    if (state.strokeScreenPoints.size() < 12)
    {
        RecognitionResult result;
        result.label = "unknown";
        return result;
    }

    MatrixXd stroke2d(static_cast<int>(state.strokeScreenPoints.size()), 2);
    for (int i = 0; i < static_cast<int>(state.strokeScreenPoints.size()); ++i)
    {
        stroke2d.row(i) = state.strokeScreenPoints[i].transpose();
    }

    const StrokeFeatures rawFeatures = extract_stroke_features(stroke2d);

    MatrixXd norm2d = normalize_and_center_2d(stroke2d);
    norm2d = align_to_pca_2d(norm2d);
    MatrixXd hull2d = convex_hull_2d(norm2d);
    if (hull2d.rows() < 3)
    {
        RecognitionResult result;
        result.label = "unknown";
        return result;
    }

    // Compute hull descriptors.
    const int hullVerts = hull2d.rows();

    const Eigen::RowVector2d hullMin = hull2d.colwise().minCoeff();
    const Eigen::RowVector2d hullMax = hull2d.colwise().maxCoeff();
    const double hullW = std::max(1e-8, hullMax(0) - hullMin(0));
    const double hullH = std::max(1e-8, hullMax(1) - hullMin(1));
    const double hullAspect = hullW / hullH;

    // Hull circularity.
    double hullPerim = 0.0;
    for (int i = 0; i < hull2d.rows(); ++i)
    {
        const int j = (i + 1) % hull2d.rows();
        hullPerim += (hull2d.row(j) - hull2d.row(i)).norm();
    }
    double hullArea2 = 0.0;
    for (int i = 0; i < hull2d.rows(); ++i)
    {
        const int j = (i + 1) % hull2d.rows();
        hullArea2 += hull2d(i, 0) * hull2d(j, 1) - hull2d(j, 0) * hull2d(i, 1);
    }
    const double hullArea = 0.5 * std::abs(hullArea2);
    const double hullCircularity = (hullPerim > 1e-8) ? (4.0 * M_PI * hullArea / (hullPerim * hullPerim)) : 0.0;

    // Compute end-width ratio using hull vertices projected onto minor axis.
    const double majorLen = std::max(hullW, hullH);
    const int majorCol = (hullW >= hullH) ? 0 : 1;
    const int minorCol = 1 - majorCol;
    const double majorMin = hull2d.col(majorCol).minCoeff();
    const double majorMax = hull2d.col(majorCol).maxCoeff();
    const double majorRange = std::max(1e-8, majorMax - majorMin);
    const double bandFrac = 0.22;

    double lo_min = 1e9, lo_max = -1e9, hi_min = 1e9, hi_max = -1e9;
    int loN = 0, hiN = 0;
    for (int i = 0; i < hull2d.rows(); ++i)
    {
        const double mj = hull2d(i, majorCol);
        const double mn = hull2d(i, minorCol);
        if (mj - majorMin <= bandFrac * majorRange) { lo_min = std::min(lo_min, mn); lo_max = std::max(lo_max, mn); loN++; }
        if (majorMax - mj <= bandFrac * majorRange) { hi_min = std::min(hi_min, mn); hi_max = std::max(hi_max, mn); hiN++; }
    }
    const double loW = (loN >= 1) ? std::max(1e-8, lo_max - lo_min) : 1e-8;
    const double hiW = (hiN >= 1) ? std::max(1e-8, hi_max - hi_min) : 1e-8;
    const double endWidthRatio = std::min(loW, hiW) / std::max(loW, hiW);

    // ----- Prototype-distance classification -----
    // Each primitive is a prototype in a 4D feature space:
    //   F0 = hullCircularity  (circle=1, square=0.785, triangle=0.605)
    //   F1 = normalised hull vertex count  (clamped to [0,1] over range [3,12])
    //   F2 = |aspectRatio - 1|  (0=square, large=elongated)
    //   F3 = endWidthRatio      (1=symmetric, 0=very tapered)
    //
    // Score = weighted L1 distance to prototype.  Lower = better match.

    const double F0 = hullCircularity;
    const double F1 = std::min(1.0, std::max(0.0, (double(hullVerts) - 3.0) / 9.0)); // [3..12] -> [0..1]
    const double F2 = std::min(1.0, std::abs(hullAspect - 1.0));
    const double F3 = endWidthRatio;

    // Weights [w0, w1, w2, w3] and prototypes [p0, p1, p2, p3]
    struct Prototype { std::string name; double p0,p1,p2,p3,w0,w1,w2,w3; };
    const std::vector<Prototype> prototypes = {
        // name                            p0     p1    p2    p3     w0    w1    w2    w3
        {"sphere",                         0.95,  0.7,  0.05, 0.9,  3.0,  1.0,  2.0,  1.0},
        {"cube",                           0.78,  0.1,  0.05, 0.9,  2.5,  2.0,  2.5,  1.0},
        {"pyramid",                        0.62,  0.0,  0.10, 0.20, 2.8,  2.2,  2.0,  2.4},
    };

    std::vector<std::pair<std::string, double>> scores;
    for (const Prototype& p : prototypes)
    {
        double s = p.w0 * std::abs(F0 - p.p0)
                 + p.w1 * std::abs(F1 - p.p1)
                 + p.w2 * std::abs(F2 - p.p2)
                 + p.w3 * std::abs(F3 - p.p3);

        const bool closedStroke = rawFeatures.isClosed;
        // Strong prior for polygon-like closed strokes.
        const bool squareLikeHull = (hullVerts <= 5 && F2 < 0.22 && F3 > 0.75 && F0 > 0.70 && F0 < 0.90);
        const bool parallelogramLikeHull = (hullVerts <= 5 && F0 > 0.68 && F0 < 0.90 && F3 > 0.60 && F2 > 0.08);
        if (squareLikeHull)
        {
            if (p.name == "cube")
            {
                s -= 0.20;
            }
        }
        if (parallelogramLikeHull)
        {
            if (p.name == "cube")
            {
                s -= 0.12;
            }
        }

        // Closed strokes can also represent pyramid; keep this as soft priors,
        // not hard decisions, to avoid short-circuiting classes.
        if (closedStroke)
        {
            const bool nearCircular = (F0 > 0.90 && F2 < 0.18 && F3 > 0.75);
            const bool triangularLike = (hullVerts <= 4 && F0 < 0.72 && F3 < 0.55);
            const bool manyCorners = (rawFeatures.cornerCount >= 4);
            const bool triangleCorners = (rawFeatures.cornerCount == 3);
            const bool taperedPolygonLike = (rawFeatures.cornerCount >= 3 && rawFeatures.cornerCount <= 5 && F3 < 0.72);

            if (nearCircular && p.name == "sphere")
            {
                s -= 0.12;
            }
            if (!nearCircular && p.name == "sphere")
            {
                // Keep sphere from dominating non-circular closed strokes.
                s += 0.80;
            }
            if (triangularLike && p.name == "pyramid")
            {
                s -= 0.35;
            }
            if (taperedPolygonLike && p.name == "pyramid")
            {
                s -= 0.22;
            }

            // Corner-aware priors for hand-drawn closed sketches.
            if (manyCorners && p.name == "cube")
            {
                s -= 0.22;
            }
            if (triangleCorners && p.name == "pyramid")
            {
                s -= 0.45;
            }
            if ((manyCorners || triangleCorners) && p.name == "sphere")
            {
                s += 0.50;
            }
        }

        scores.emplace_back(p.name, s);
    }

    std::sort(scores.begin(), scores.end(), [](const std::pair<std::string, double>& a, const std::pair<std::string, double>& b)
    {
        return a.second < b.second;
    });

    RecognitionResult result;
    result.label = scores.front().first;
    result.score = scores.front().second;
    result.ranking = scores;
    return result;
}

void add_primitive_from_stroke(const igl::opengl::glfw::Viewer& viewer, const std::string& primitiveLabel)
{
    const TemplateMesh* primitiveTemplate = find_primitive_template(primitiveLabel);
    if (primitiveTemplate == nullptr || primitiveTemplate->V.rows() == 0 || state.strokePoints.rows() == 0)
    {
        return;
    }

    const RowVector3d minP = state.strokePoints.colwise().minCoeff();
    const RowVector3d maxP = state.strokePoints.colwise().maxCoeff();
    const RowVector3d centerRawMean = state.strokePoints.colwise().mean();
    const RowVector3d centerRawBox = 0.5 * (minP + maxP);

    const RowVector3d centerRaw = centerRawMean;

    const RowVector3d spawnRef = mesh_spawn_reference_point();
    const Eigen::Vector3f centerRawF = centerRaw.transpose().cast<float>();
    const Eigen::Vector3f spawnRefF = spawnRef.transpose().cast<float>();
    const Eigen::Vector3f centerProj = igl::project(centerRawF, viewer.core().view, viewer.core().proj, viewer.core().viewport);
    const Eigen::Vector3f spawnProj = igl::project(spawnRefF, viewer.core().view, viewer.core().proj, viewer.core().viewport);
    double targetScreenX = static_cast<double>(centerProj(0));
    double targetScreenY = static_cast<double>(centerProj(1));
    const Eigen::Vector3f targetWin(static_cast<float>(targetScreenX), static_cast<float>(targetScreenY), spawnProj(2));
    const Eigen::Vector3f centerWorld = igl::unproject(targetWin, viewer.core().view, viewer.core().proj, viewer.core().viewport);
    const RowVector3d center = centerWorld.cast<double>().transpose();

    const Eigen::Matrix4f invView = viewer.core().view.inverse();
    const Vector3d right = invView.block<3, 1>(0, 0).cast<double>().normalized();
    const Vector3d up = invView.block<3, 1>(0, 1).cast<double>().normalized();
    const Vector3d forward = invView.block<3, 1>(0, 2).cast<double>().normalized();

    // Perspective compensation: if we move along view depth, preserve apparent on-screen size.
    double depthScale = 1.0;
    {
        Eigen::Vector4f srcH;
        srcH << static_cast<float>(centerRaw(0)), static_cast<float>(centerRaw(1)), static_cast<float>(centerRaw(2)), 1.0f;
        Eigen::Vector4f dstH;
        dstH << static_cast<float>(center(0)), static_cast<float>(center(1)), static_cast<float>(center(2)), 1.0f;

        const Eigen::Vector4f srcCam = viewer.core().view * srcH;
        const Eigen::Vector4f dstCam = viewer.core().view * dstH;

        const double zSrc = std::abs(static_cast<double>(srcCam(2)));
        const double zDst = std::abs(static_cast<double>(dstCam(2)));
        if (zSrc > 1e-8)
        {
            depthScale = std::max(0.1, std::min(10.0, zDst / zSrc));
        }
    }

    const RowVector3d span = maxP - minP;
    double halfSize = 0.5 * std::max(span(0), std::max(span(1), span(2)));
    if (halfSize < 1e-3)
    {
        const double fallbackScale = (state.meshV.rows() > 0) ? 0.08 * (state.meshV.rowwise().norm().maxCoeff()) : 0.05;
        halfSize = std::max(0.02, fallbackScale);
    }

    MatrixXd normalizedPrimitive = primitiveTemplate->V;
    normalize_unitbox(normalizedPrimitive);
    normalizedPrimitive = normalizedPrimitive.rowwise() - normalizedPrimitive.colwise().mean();

    auto compute_stroke_screen_extents = [&](double& width, double& height)
    {
        double minU = std::numeric_limits<double>::infinity();
        double maxU = -std::numeric_limits<double>::infinity();
        double minV = std::numeric_limits<double>::infinity();
        double maxV = -std::numeric_limits<double>::infinity();

        for (int i = 0; i < state.strokePoints.rows(); ++i)
        {
            const Vector3d d = state.strokePoints.row(i).transpose() - center.transpose();
            const double u = d.dot(right);
            const double v = d.dot(up);
            minU = std::min(minU, u);
            maxU = std::max(maxU, u);
            minV = std::min(minV, v);
            maxV = std::max(maxV, v);
        }

        width = std::max(1e-4, maxU - minU);
        height = std::max(1e-4, maxV - minV);
        if (!std::isfinite(width) || !std::isfinite(height))
        {
            width = 2.0 * halfSize;
            height = 2.0 * halfSize;
        }

        width *= depthScale;
        height *= depthScale;
    };

    auto transform_with_axis_scales = [&](const RowVector3d& scales)
    {
        MatrixXd transformed(normalizedPrimitive.rows(), 3);
        for (int i = 0; i < normalizedPrimitive.rows(); ++i)
        {
            const double lx = normalizedPrimitive(i, 0) * scales(0);
            const double ly = normalizedPrimitive(i, 1) * scales(1);
            const double lz = normalizedPrimitive(i, 2) * scales(2);
            const Vector3d world = center.transpose() + lx * right + ly * up + lz * forward;
            transformed.row(i) = world.transpose();
        }
        normalizedPrimitive = transformed;
    };

    const RowVector3d primMin = normalizedPrimitive.colwise().minCoeff();
    const RowVector3d primMax = normalizedPrimitive.colwise().maxCoeff();
    const RowVector3d primHalf = 0.5 * (primMax - primMin);

    auto closest_pair_axes = [&](int& a, int& b, int& c)
    {
        const double hx = std::max(1e-8, primHalf(0));
        const double hy = std::max(1e-8, primHalf(1));
        const double hz = std::max(1e-8, primHalf(2));
        const double d01 = std::abs(hx - hy);
        const double d02 = std::abs(hx - hz);
        const double d12 = std::abs(hy - hz);

        if (d01 <= d02 && d01 <= d12)
        {
            a = 0;
            b = 1;
            c = 2;
        }
        else if (d02 <= d01 && d02 <= d12)
        {
            a = 0;
            b = 2;
            c = 1;
        }
        else
        {
            a = 1;
            b = 2;
            c = 0;
        }
    };

    Matrix3d createdPyramidOrientation = Matrix3d::Identity();
    bool hasCreatedPyramidOrientation = false;

    if (primitiveLabel == "cube")
    {
        double width = 0.0;
        double height = 0.0;
        compute_stroke_screen_extents(width, height);
        const double depth = 0.5 * (width + height);
        const double hx = std::max(1e-8, primHalf(0));
        const double hy = std::max(1e-8, primHalf(1));
        const double hz = std::max(1e-8, primHalf(2));

        RowVector3d scales;
        scales << (0.5 * width) / hx, (0.5 * height) / hy, (0.5 * depth) / hz;
        transform_with_axis_scales(scales);
    }
    else if (primitiveLabel == "pyramid")
    {
        double width = 0.0;
        double height = 0.0;
        compute_stroke_screen_extents(width, height);

        const double baseHalfWidth = 0.5 * width;
        const double baseHalfDepth = 0.5 * height;
        const double targetHalfHeight = 0.35 * (width + height);

        int baseAxis0 = 0;
        int baseAxis1 = 1;
        int heightAxis = 2;
        closest_pair_axes(baseAxis0, baseAxis1, heightAxis);

        // Robustly detect which template vertex is the apex along the height axis.
        int apexIndexTemplate = 0;
        std::vector<char> isBaseVertex(normalizedPrimitive.rows(), 0);
        {
            const double hMin = normalizedPrimitive.col(heightAxis).minCoeff();
            const double hMax = normalizedPrimitive.col(heightAxis).maxCoeff();
            const double hRange = std::max(1e-8, hMax - hMin);
            const double eps = 0.04 * hRange;

            int countNearMin = 0;
            int countNearMax = 0;
            for (int i = 0; i < normalizedPrimitive.rows(); ++i)
            {
                const double h = normalizedPrimitive(i, heightAxis);
                if (std::abs(h - hMin) <= eps)
                {
                    countNearMin += 1;
                }
                if (std::abs(h - hMax) <= eps)
                {
                    countNearMax += 1;
                }
            }

            const bool apexAtMax = (countNearMax <= countNearMin);
            const double baseLevel = apexAtMax ? hMin : hMax;
            double best = apexAtMax ? -std::numeric_limits<double>::infinity()
                                    : std::numeric_limits<double>::infinity();
            for (int i = 0; i < normalizedPrimitive.rows(); ++i)
            {
                const double h = normalizedPrimitive(i, heightAxis);
                if (apexAtMax)
                {
                    if (h > best)
                    {
                        best = h;
                        apexIndexTemplate = i;
                    }
                }
                else
                {
                    if (h < best)
                    {
                        best = h;
                        apexIndexTemplate = i;
                    }
                }

                if (std::abs(h - baseLevel) <= eps)
                {
                    isBaseVertex[i] = 1;
                }
            }

            // Fallback if base ring is too sparse under epsilon criterion.
            int baseCount = 0;
            for (int i = 0; i < static_cast<int>(isBaseVertex.size()); ++i)
            {
                if (isBaseVertex[i])
                {
                    baseCount += 1;
                }
            }
            if (baseCount < 3)
            {
                std::fill(isBaseVertex.begin(), isBaseVertex.end(), 0);
                for (int i = 0; i < normalizedPrimitive.rows(); ++i)
                {
                    if (i != apexIndexTemplate)
                    {
                        isBaseVertex[i] = 1;
                    }
                }
            }
        }

        RowVector3d scales = RowVector3d::Ones();
        scales(baseAxis0) = baseHalfWidth / std::max(1e-8, primHalf(baseAxis0));
        scales(baseAxis1) = baseHalfDepth / std::max(1e-8, primHalf(baseAxis1));
        scales(heightAxis) = targetHalfHeight / std::max(1e-8, primHalf(heightAxis));
        transform_with_axis_scales(scales);

        // If the user drew a triangle, orient the pyramid so its apex points toward
        // the triangle vertex that is farthest from the other two.
        MatrixXd strokeUV(state.strokePoints.rows(), 2);
        for (int i = 0; i < state.strokePoints.rows(); ++i)
        {
            const Vector3d d = state.strokePoints.row(i).transpose() - center.transpose();
            strokeUV(i, 0) = d.dot(right);
            strokeUV(i, 1) = d.dot(up);
        }

        const MatrixXd strokeHull = convex_hull_2d(strokeUV);
        if (strokeHull.rows() >= 3)
        {
            // Robustly extract 3 representative corners from a possibly dense hull.
            int c0 = 0;
            {
                const Eigen::RowVector2d hCenter = strokeHull.colwise().mean();
                double best = -1.0;
                for (int i = 0; i < strokeHull.rows(); ++i)
                {
                    const double d2 = (strokeHull.row(i) - hCenter).squaredNorm();
                    if (d2 > best)
                    {
                        best = d2;
                        c0 = i;
                    }
                }
            }

            int c1 = c0;
            {
                double best = -1.0;
                for (int i = 0; i < strokeHull.rows(); ++i)
                {
                    if (i == c0)
                    {
                        continue;
                    }
                    const double d2 = (strokeHull.row(i) - strokeHull.row(c0)).squaredNorm();
                    if (d2 > best)
                    {
                        best = d2;
                        c1 = i;
                    }
                }
            }

            int c2 = c0;
            {
                const Eigen::RowVector2d a = strokeHull.row(c0);
                const Eigen::RowVector2d b = strokeHull.row(c1);
                const Eigen::RowVector2d ab = b - a;
                const double ab2 = std::max(1e-12, ab.squaredNorm());
                double best = -1.0;
                for (int i = 0; i < strokeHull.rows(); ++i)
                {
                    if (i == c0 || i == c1)
                    {
                        continue;
                    }
                    const Eigen::RowVector2d p = strokeHull.row(i);
                    const double t = std::max(0.0, std::min(1.0, (p - a).dot(ab) / ab2));
                    const Eigen::RowVector2d proj = a + t * ab;
                    const double d2 = (p - proj).squaredNorm();
                    if (d2 > best)
                    {
                        best = d2;
                        c2 = i;
                    }
                }
            }

            std::array<int, 3> corners = {c0, c1, c2};

            int apex2dLocal = 0;
            double bestScore = -1.0;
            for (int i = 0; i < 3; ++i)
            {
                const int ii = corners[i];
                const int jj = corners[(i + 1) % 3];
                const int kk = corners[(i + 2) % 3];
                const double score = (strokeHull.row(ii) - strokeHull.row(jj)).squaredNorm()
                                   + (strokeHull.row(ii) - strokeHull.row(kk)).squaredNorm();
                if (score > bestScore)
                {
                    bestScore = score;
                    apex2dLocal = i;
                }
            }

            const int apex2d = corners[apex2dLocal];
            const int b0 = corners[(apex2dLocal + 1) % 3];
            const int b1 = corners[(apex2dLocal + 2) % 3];
            const Eigen::RowVector2d baseMid2d = 0.5 * (strokeHull.row(b0) + strokeHull.row(b1));
            const Eigen::RowVector2d dir2d = strokeHull.row(apex2d) - baseMid2d;

            // For triangular strokes, anchor placement to the drawn base midpoint in screen space.
            {
                MatrixXd strokeScreen(static_cast<int>(state.strokeScreenPoints.size()), 2);
                for (int i = 0; i < static_cast<int>(state.strokeScreenPoints.size()); ++i)
                {
                    strokeScreen.row(i) = state.strokeScreenPoints[i].transpose();
                }
                const MatrixXd screenHull = convex_hull_2d(strokeScreen);
                if (screenHull.rows() >= 3)
                {
                    int s0 = 0;
                    {
                        const Eigen::RowVector2d hCenter = screenHull.colwise().mean();
                        double best = -1.0;
                        for (int i = 0; i < screenHull.rows(); ++i)
                        {
                            const double d2 = (screenHull.row(i) - hCenter).squaredNorm();
                            if (d2 > best)
                            {
                                best = d2;
                                s0 = i;
                            }
                        }
                    }

                    int s1 = s0;
                    {
                        double best = -1.0;
                        for (int i = 0; i < screenHull.rows(); ++i)
                        {
                            if (i == s0)
                            {
                                continue;
                            }
                            const double d2 = (screenHull.row(i) - screenHull.row(s0)).squaredNorm();
                            if (d2 > best)
                            {
                                best = d2;
                                s1 = i;
                            }
                        }
                    }

                    int s2 = s0;
                    {
                        const Eigen::RowVector2d a = screenHull.row(s0);
                        const Eigen::RowVector2d b = screenHull.row(s1);
                        const Eigen::RowVector2d ab = b - a;
                        const double ab2 = std::max(1e-12, ab.squaredNorm());
                        double best = -1.0;
                        for (int i = 0; i < screenHull.rows(); ++i)
                        {
                            if (i == s0 || i == s1)
                            {
                                continue;
                            }
                            const Eigen::RowVector2d p = screenHull.row(i);
                            const double t = std::max(0.0, std::min(1.0, (p - a).dot(ab) / ab2));
                            const Eigen::RowVector2d proj = a + t * ab;
                            const double d2 = (p - proj).squaredNorm();
                            if (d2 > best)
                            {
                                best = d2;
                                s2 = i;
                            }
                        }
                    }

                    std::array<int, 3> sc = {s0, s1, s2};
                    int sapexLocal = 0;
                    double sbest = -1.0;
                    for (int i = 0; i < 3; ++i)
                    {
                        const int ii = sc[i];
                        const int jj = sc[(i + 1) % 3];
                        const int kk = sc[(i + 2) % 3];
                        const double score = (screenHull.row(ii) - screenHull.row(jj)).squaredNorm()
                                           + (screenHull.row(ii) - screenHull.row(kk)).squaredNorm();
                        if (score > sbest)
                        {
                            sbest = score;
                            sapexLocal = i;
                        }
                    }
                    const int sb0 = sc[(sapexLocal + 1) % 3];
                    const int sb1 = sc[(sapexLocal + 2) % 3];
                    const Eigen::RowVector2d screenBaseMid = 0.5 * (screenHull.row(sb0) + screenHull.row(sb1));
                    targetScreenX = screenBaseMid(0);
                    targetScreenY = screenBaseMid(1);
                }
            }

            if (dir2d.squaredNorm() > 1e-12)
            {
                Vector3d targetDir = (dir2d(0) * right + dir2d(1) * up).normalized();

                const int apexIndex = apexIndexTemplate;

                RowVector3d baseCenter = RowVector3d::Zero();
                int baseCount = 0;
                for (int i = 0; i < normalizedPrimitive.rows(); ++i)
                {
                    if (isBaseVertex[i])
                    {
                        baseCenter += normalizedPrimitive.row(i);
                        baseCount += 1;
                    }
                }
                baseCenter /= static_cast<double>(std::max(1, baseCount));

                Vector3d currentDir = (normalizedPrimitive.row(apexIndex) - baseCenter).transpose();
                if (currentDir.squaredNorm() > 1e-12)
                {
                    currentDir.normalize();
                    Eigen::Quaterniond q;
                    q.setFromTwoVectors(currentDir, targetDir);
                    const Matrix3d R = q.toRotationMatrix();
                    for (int i = 0; i < normalizedPrimitive.rows(); ++i)
                    {
                        const Vector3d p = normalizedPrimitive.row(i).transpose() - baseCenter.transpose();
                        normalizedPrimitive.row(i) = (R * p + baseCenter.transpose()).transpose();
                    }
                }
            }
        }

        // Anchor insertion on the base center so the drawn base location is preserved.
        const int apexIndex = apexIndexTemplate;

        RowVector3d baseCenter = RowVector3d::Zero();
        int baseCount = 0;
        for (int i = 0; i < normalizedPrimitive.rows(); ++i)
        {
            if (isBaseVertex[i])
            {
                baseCenter += normalizedPrimitive.row(i);
                baseCount += 1;
            }
        }
        if (baseCount > 0)
        {
            baseCenter /= static_cast<double>(baseCount);
        }
        else
        {
            // Safety fallback
            if (normalizedPrimitive.rows() > 1)
            {
                for (int i = 0; i < normalizedPrimitive.rows(); ++i)
                {
                    if (i != apexIndex)
                    {
                        baseCenter += normalizedPrimitive.row(i);
                    }
                }
                baseCenter /= static_cast<double>(normalizedPrimitive.rows() - 1);
            }
            else
            {
                baseCenter = normalizedPrimitive.row(0);
            }
        }

        const RowVector3d baseDelta = center - baseCenter;
        normalizedPrimitive = normalizedPrimitive.rowwise() + baseDelta;

        // Final correction in screen space: force base center to match drawn 2D anchor.
        RowVector3d finalBaseCenter = RowVector3d::Zero();
        int finalBaseCount = 0;
        for (int i = 0; i < normalizedPrimitive.rows(); ++i)
        {
            if (isBaseVertex[i])
            {
                finalBaseCenter += normalizedPrimitive.row(i);
                finalBaseCount += 1;
            }
        }
        if (finalBaseCount > 0)
        {
            finalBaseCenter /= static_cast<double>(finalBaseCount);

            const Eigen::Vector3f baseCenterF = finalBaseCenter.transpose().cast<float>();
            const Eigen::Vector3f baseProj = igl::project(baseCenterF, viewer.core().view, viewer.core().proj, viewer.core().viewport);
            const Eigen::Vector3f targetWin(static_cast<float>(targetScreenX), static_cast<float>(targetScreenY), baseProj(2));
            const Eigen::Vector3f targetWorld = igl::unproject(targetWin, viewer.core().view, viewer.core().proj, viewer.core().viewport);
            const RowVector3d screenAnchorDelta = targetWorld.cast<double>().transpose() - finalBaseCenter;
            normalizedPrimitive = normalizedPrimitive.rowwise() + screenAnchorDelta;
        }

        // Save the actual spawn orientation so later base/height edits keep the same direction.
        {
            RowVector3d orientedBaseCenter = RowVector3d::Zero();
            int orientedBaseCount = 0;
            for (int i = 0; i < normalizedPrimitive.rows(); ++i)
            {
                if (isBaseVertex[i])
                {
                    orientedBaseCenter += normalizedPrimitive.row(i);
                    orientedBaseCount += 1;
                }
            }
            if (orientedBaseCount > 0)
            {
                orientedBaseCenter /= static_cast<double>(orientedBaseCount);
                Vector3d axis = (normalizedPrimitive.row(apexIndexTemplate) - orientedBaseCenter).transpose();
                if (axis.squaredNorm() > 1e-12)
                {
                    axis.normalize();
                    Eigen::Quaterniond q;
                    q.setFromTwoVectors(Vector3d::UnitZ(), axis);
                    createdPyramidOrientation = q.toRotationMatrix();
                    hasCreatedPyramidOrientation = true;
                }
            }
        }
    }
    else
    {
        const double primitiveScale = normalizedPrimitive.cwiseAbs().maxCoeff();
        if (primitiveScale > 1e-10)
        {
            normalizedPrimitive *= ((halfSize * depthScale) / primitiveScale);
        }
        normalizedPrimitive = normalizedPrimitive.rowwise() + center;
    }

    PrimitiveInstance primitive;
    primitive.label = primitiveLabel;
    primitive.V = normalizedPrimitive;
    primitive.F = primitiveTemplate->F;
    primitive.color = default_primitive_color();

    if (primitiveLabel == "pyramid")
    {
        primitive.pyramidSides = 5;
        primitive.pyramidBaseSize = std::max(0.1, 0.5 * (normalizedPrimitive.col(0).maxCoeff() - normalizedPrimitive.col(0).minCoeff()));
        primitive.pyramidHeight = std::max(0.1, normalizedPrimitive.col(2).maxCoeff() - normalizedPrimitive.col(2).minCoeff());
        primitive.pyramidOrientation = hasCreatedPyramidOrientation ? createdPyramidOrientation : Matrix3d::Identity();
    }

    state.primitives.push_back(primitive);
    
    state.prediction.label = primitiveLabel;
    state.prediction.score = 0.0;
    state.prediction.ranking.clear();
    state.prediction.ranking.emplace_back(primitiveLabel, 0.0);
    state.hasPrediction = true;
}

void append_stroke_point(igl::opengl::glfw::Viewer& viewer, int mouseX, int mouseY)
{
    RowVector3d point;
    if (!project_mouse_to_draw_plane(viewer, mouseX, mouseY, point))
    {
        return;
    }

    const Eigen::Vector2d screenPoint{double(mouseX), double(mouseY)};

    if (state.strokePoints.rows() == 0)
    {
        state.strokePoints.resize(1, 3);
        state.strokePoints.row(0) = point;
        state.strokeScreenPoints.push_back(screenPoint);
        return;
    }

    const double minSpacing = 0.003;
    if ((state.strokePoints.row(state.strokePoints.rows() - 1) - point).norm() < minSpacing)
    {
        return;
    }

    state.strokePoints.conservativeResize(state.strokePoints.rows() + 1, 3);
    state.strokePoints.row(state.strokePoints.rows() - 1) = point;
    state.strokeScreenPoints.push_back(screenPoint);
}

void refresh_prediction()
{
    state.prediction = classify_mesh(sample_points(align_to_pca(state.meshV), 600));
    state.hasPrediction = true;
}

void load_current_mesh(const std::string& requestedPath)
{
    const std::string resolvedPath = resolve_mesh_path(requestedPath);
    MatrixXd V;
    MatrixXi F;

    if (!load_mesh_file(resolvedPath, V, F))
    {
        std::cerr << "Failed to load mesh: " << resolvedPath << std::endl;
        state.hasPrediction = false;
        return;
    }

    state.currentPath = resolvedPath;
    state.meshV = V;
    state.meshF = F;
    state.primitives.clear();
    clear_stroke();
    refresh_prediction();
}

void load_templates()
{
    const std::string meshDir = mesh_directory();
    const std::string primitiveDir = primitive_directory();
    const std::vector<std::string> meshTemplateFiles = list_obj_files_in_directory(meshDir);

    meshTemplates.clear();
    if (meshTemplateFiles.empty())
    {
        std::cerr << "No .obj mesh templates found in: " << meshDir << std::endl;
    }

    for (const std::string& fileName : meshTemplateFiles)
    {
        TemplateMesh mesh;
        mesh.name = stem_from_path(fileName);
        mesh.path = meshDir + "/" + fileName;

        if (!load_mesh_file(mesh.path, mesh.V, mesh.F))
        {
            std::cerr << "Skipping template: " << mesh.path << std::endl;
            continue;
        }

        meshTemplates.push_back(mesh);
    }

    const std::vector<std::string> primitiveFiles =
    {
        "cube.obj",
        "sphere.obj",
        "pyramid.obj"
    };

    primitiveTemplates.clear();
    for (const std::string& fileName : primitiveFiles)
    {
        TemplateMesh primitive;
        primitive.name = stem_from_path(fileName);
        primitive.path = primitiveDir + "/" + fileName;

        if (!load_mesh_file(primitive.path, primitive.V, primitive.F))
        {
            std::cerr << "Skipping primitive: " << primitive.path << std::endl;
            continue;
        }

        primitive.samples = sample_points(align_to_pca(primitive.V), 600);
        const MatrixXd aligned = align_to_pca(primitive.V);

        primitive.samples2dVariants.clear();
        primitive.circularity2dVariants.clear();
        std::array<std::pair<int, int>, 3> projections = {
            std::make_pair(0, 1),
            std::make_pair(0, 2),
            std::make_pair(1, 2)
        };

        for (const auto& projection : projections)
        {
            MatrixXd primitive2d(aligned.rows(), 2);
            primitive2d.col(0) = aligned.col(projection.first);
            primitive2d.col(1) = aligned.col(projection.second);

            primitive2d = normalize_and_center_2d(primitive2d);
            primitive2d = align_to_pca_2d(primitive2d);
            const MatrixXd hull2d = convex_hull_2d(primitive2d);
            const MatrixXd contour = sample_closed_contour_2d(hull2d, 300);
            primitive.samples2dVariants.push_back(contour);

            const double perim = [&]()
            {
                if (hull2d.rows() < 2)
                {
                    return 0.0;
                }
                double p = 0.0;
                for (int i = 0; i < hull2d.rows(); ++i)
                {
                    const int j = (i + 1) % hull2d.rows();
                    p += (hull2d.row(j) - hull2d.row(i)).norm();
                }
                return p;
            }();

            const double area = [&]()
            {
                if (hull2d.rows() < 3)
                {
                    return 0.0;
                }
                double a2 = 0.0;
                for (int i = 0; i < hull2d.rows(); ++i)
                {
                    const int j = (i + 1) % hull2d.rows();
                    a2 += hull2d(i, 0) * hull2d(j, 1) - hull2d(j, 0) * hull2d(i, 1);
                }
                return 0.5 * std::abs(a2);
            }();

            const double circularity = (perim > 1e-12 && area > 1e-12) ? (4.0 * M_PI * area / (perim * perim)) : 0.0;
            primitive.circularity2dVariants.push_back(circularity);
        }

        primitiveTemplates.push_back(primitive);
    }
}

void clear_stroke()
{
    state.strokePoints.resize(0, 3);
    state.strokeScreenPoints.clear();
}

} // namespace

int main(int argc, char* argv[])
{
    using namespace Eigen;

    load_templates();

    // Empty-canvas mode: no input mesh, only drawn strokes and primitive overlays.
    state.meshV.resize(0, 3);
    state.meshF.resize(0, 3);
    clear_stroke();
    state.primitives.clear();
    state.currentPath = "(none)";
    state.hasPrediction = false;
    std::fill(state.meshInput.begin(), state.meshInput.end(), '\0');

    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);
    menu.callback_draw_viewer_window = []() {};
    menu.callback_draw_custom_window = [&]()
    {
        draw_axis_gizmo_top_left(viewer);
        sync_transform_ui_with_selection();

        ImGui::SetNextWindowPos(ImVec2(0.f * menu.menu_scaling(), 0), ImGuiSetCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(300, 500), ImGuiSetCond_FirstUseEver);
        ImGui::Begin("Mesh Sketch Recognizer", nullptr, ImGuiWindowFlags_NoSavedSettings);

        ImGui::Text("Instructions");
        ImGui::BulletText("Shift + left click and drag to draw anywhere");
        ImGui::BulletText("Left click + drag on an object to move it");
        ImGui::BulletText("I/K: rotate X, J/L: rotate Y, U/O: rotate Z");
        ImGui::BulletText("+/-: scale selected object up/down");
        ImGui::BulletText("Release: add the closest primitive from primitives/");
        ImGui::BulletText("X: clear all");

        ImGui::Separator();
        ImGui::Text("Add Mesh");

        const bool hasMeshTemplates = !meshTemplates.empty();
        if (hasMeshTemplates)
        {
            if (state.selectedMeshTemplate < 0 || state.selectedMeshTemplate >= static_cast<int>(meshTemplates.size()))
            {
                state.selectedMeshTemplate = 0;
            }

            const char* preview = meshTemplates[state.selectedMeshTemplate].name.c_str();
            if (ImGui::BeginCombo("##mesh_to_add", preview))
            {
                for (int i = 0; i < static_cast<int>(meshTemplates.size()); ++i)
                {
                    const bool selected = (state.selectedMeshTemplate == i);
                    if (ImGui::Selectable(meshTemplates[i].name.c_str(), selected))
                    {
                        state.selectedMeshTemplate = i;
                    }
                    if (selected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
        }
        else
        {
            ImGui::TextDisabled("No mesh found in meshes/");
        }

        if (hasMeshTemplates)
        {
            if (ImGui::Button("add mesh", ImVec2(-1, 0)))
            {
                add_mesh_from_library(state.selectedMeshTemplate);
                draw_scene(viewer);
            }
        }
        else
        {
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.5f * ImGui::GetStyle().Alpha);
            ImGui::Button("add mesh", ImVec2(-1, 0));
            ImGui::PopStyleVar();
        }

        ImGui::Separator();
        ImGui::Text("Objects");
        if (state.primitives.empty())
        {
            ImGui::Text("None");
        }
        else
        {
            for (int i = 0; i < static_cast<int>(state.primitives.size()); ++i)
            {
                std::ostringstream itemLabel;
                itemLabel << i << ": " << state.primitives[i].label;
                const bool isSelected = (state.selectedPrimitive == i);
                if (ImGui::Selectable(itemLabel.str().c_str(), isSelected))
                {
                    state.selectedPrimitive = i;
                    state.draggingPrimitive = false;
                    sync_transform_ui_with_selection(true);
                    draw_scene(viewer);
                }
            }
        }

        ImGui::Separator();
        ImGui::Text("Selected Object");
        if (has_selected_primitive())
        {
            bool sceneDirty = false;
            PrimitiveInstance& selected = state.primitives[state.selectedPrimitive];
            ImGui::Text("Name: %s", selected.label.c_str());

            const RowVector3d spawnRef = mesh_spawn_reference_point();
            state.uiTranslate = primitive_center(selected) - spawnRef;
            state.uiTranslatePrev = state.uiTranslate;

            if (ImGui::Button("delete selected", ImVec2(-1, 0)))
            {
                state.primitives.erase(state.primitives.begin() + state.selectedPrimitive);
                state.selectedPrimitive = -1;
                state.draggingPrimitive = false;
                sync_transform_ui_with_selection(true);
                draw_scene(viewer);
                ImGui::End();
                return;
            }

            float tx = static_cast<float>(state.uiTranslate(0));
            float ty = static_cast<float>(state.uiTranslate(1));
            float tz = static_cast<float>(state.uiTranslate(2));
            ImGui::SliderFloat("Move X", &tx, -3.0f, 3.0f);
            ImGui::SliderFloat("Move Y", &ty, -3.0f, 3.0f);
            ImGui::SliderFloat("Move Z", &tz, -3.0f, 3.0f);

            const RowVector3d newOffset(tx, ty, tz);
            const RowVector3d targetCenter = spawnRef + newOffset;
            const RowVector3d currentCenter = primitive_center(selected);
            const RowVector3d deltaT = targetCenter - currentCenter;
            if (deltaT.norm() > 1e-12)
            {
                translate_selected_primitive(deltaT);
                sceneDirty = true;
            }

            state.uiTranslate = newOffset;
            state.uiTranslatePrev = newOffset;

            float rx = static_cast<float>(state.uiRotateDeg(0));
            float ry = static_cast<float>(state.uiRotateDeg(1));
            float rz = static_cast<float>(state.uiRotateDeg(2));
            ImGui::SliderFloat("Rotate X", &rx, -180.0f, 180.0f);
            ImGui::SliderFloat("Rotate Y", &ry, -180.0f, 180.0f);
            ImGui::SliderFloat("Rotate Z", &rz, -180.0f, 180.0f);

            const RowVector3d newRotate(rx, ry, rz);
            const RowVector3d deltaDeg = newRotate - state.uiRotateDegPrev;
            if (std::abs(deltaDeg(0)) > 1e-8)
            {
                rotate_selected_primitive(Vector3d::UnitX(), deltaDeg(0) * M_PI / 180.0);
                sceneDirty = true;
            }
            if (std::abs(deltaDeg(1)) > 1e-8)
            {
                rotate_selected_primitive(Vector3d::UnitY(), deltaDeg(1) * M_PI / 180.0);
                sceneDirty = true;
            }
            if (std::abs(deltaDeg(2)) > 1e-8)
            {
                rotate_selected_primitive(Vector3d::UnitZ(), deltaDeg(2) * M_PI / 180.0);
                sceneDirty = true;
            }

            state.uiRotateDeg = newRotate;
            state.uiRotateDegPrev = newRotate;

            const bool isScalablePrimitive = (selected.label == "sphere" || selected.label == "cube");
            if (isScalablePrimitive)
            {
                float sx = static_cast<float>(state.uiScale(0));
                float sy = static_cast<float>(state.uiScale(1));
                float sz = static_cast<float>(state.uiScale(2));
                ImGui::SliderFloat("Scale X", &sx, 0.1f, 4.0f);
                ImGui::SliderFloat("Scale Y", &sy, 0.1f, 4.0f);
                ImGui::SliderFloat("Scale Z", &sz, 0.1f, 4.0f);

                const RowVector3d newScale(sx, sy, sz);
                RowVector3d ratio(1.0, 1.0, 1.0);
                ratio(0) = newScale(0) / std::max(1e-8, state.uiScalePrev(0));
                ratio(1) = newScale(1) / std::max(1e-8, state.uiScalePrev(1));
                ratio(2) = newScale(2) / std::max(1e-8, state.uiScalePrev(2));

                if (std::abs(ratio(0) - 1.0) > 1e-8 ||
                    std::abs(ratio(1) - 1.0) > 1e-8 ||
                    std::abs(ratio(2) - 1.0) > 1e-8)
                {
                    scale_selected_primitive_nonuniform(ratio);
                    sceneDirty = true;
                }

                state.uiScale = newScale;
                state.uiScalePrev = newScale;
            }

            const bool isPyramid = (selected.label == "pyramid");
            if (isPyramid)
            {
                ImGui::SliderInt("Base Sides", &state.uiPyramidSides, 3, 12);
                ImGui::SliderFloat("Base Size", &state.uiPyramidBaseSize, 0.1f, 3.0f);
                ImGui::SliderFloat("Height", &state.uiPyramidHeight, 0.1f, 3.0f);

                if (state.uiPyramidSides != state.uiPyramidSidesPrev ||
                    std::abs(state.uiPyramidBaseSize - state.uiPyramidBaseSizePrev) > 1e-8 ||
                    std::abs(state.uiPyramidHeight - state.uiPyramidHeightPrev) > 1e-8)
                {
                    update_selected_pyramid();
                    sceneDirty = true;
                }
            }

            if (sceneDirty)
            {
                draw_scene(viewer);
            }
        }
        else
        {
            ImGui::Text("None");
        }
        ImGui::End();
    };

    const RowVector3d backgroundColor(208.0 / 255.0, 237.0 / 255.0, 227.0 / 255.0);
    viewer.core().background_color = Vector4f(backgroundColor(0), backgroundColor(1), backgroundColor(2), 1.0f);
    viewer.core().is_animating = true;
    viewer.data().show_lines = false;
    viewer.data().face_based = true;
    draw_scene(viewer);

    viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer&, unsigned int key, int)
    {
        const double angleStep = 8.0 * M_PI / 180.0;
        const double scaleUp = 1.08;
        const double scaleDown = 1.0 / scaleUp;

        switch (key)
        {
            case 'I':
            case 'i':
                rotate_selected_primitive(Vector3d::UnitX(), +angleStep);
                update_ui_rotation_from_keyboard(Vector3d(+8.0, 0.0, 0.0));
                draw_scene(viewer);
                return true;
            case 'K':
            case 'k':
                rotate_selected_primitive(Vector3d::UnitX(), -angleStep);
                update_ui_rotation_from_keyboard(Vector3d(-8.0, 0.0, 0.0));
                draw_scene(viewer);
                return true;
            case 'J':
            case 'j':
                rotate_selected_primitive(Vector3d::UnitY(), +angleStep);
                update_ui_rotation_from_keyboard(Vector3d(0.0, +8.0, 0.0));
                draw_scene(viewer);
                return true;
            case 'L':
            case 'l':
                rotate_selected_primitive(Vector3d::UnitY(), -angleStep);
                update_ui_rotation_from_keyboard(Vector3d(0.0, -8.0, 0.0));
                draw_scene(viewer);
                return true;
            case 'U':
            case 'u':
                rotate_selected_primitive(Vector3d::UnitZ(), +angleStep);
                update_ui_rotation_from_keyboard(Vector3d(0.0, 0.0, +8.0));
                draw_scene(viewer);
                return true;
            case 'O':
            case 'o':
                rotate_selected_primitive(Vector3d::UnitZ(), -angleStep);
                update_ui_rotation_from_keyboard(Vector3d(0.0, 0.0, -8.0));
                draw_scene(viewer);
                return true;
            case '+':
            case '=':
                scale_selected_primitive(scaleUp);
                draw_scene(viewer);
                return true;
            case '-':
            case '_':
                scale_selected_primitive(scaleDown);
                draw_scene(viewer);
                return true;
            case 'X':
            case 'x':
                state.primitives.clear();
                draw_scene(viewer);
                return true;
            default:
                return false;
        }
    };

    viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer&, int button, int modifier) -> bool
    {
        if (button == 0 && (modifier & 0x0001))
        {
            state.drawing = true;
            state.draggingPrimitive = false;
            clear_stroke();
            append_stroke_point(viewer, viewer.current_mouse_x, viewer.current_mouse_y);
            draw_scene(viewer);
            return true;
        }

        if (button == 0)
        {
            int picked = -1;
            if (pick_primitive_by_mouse(viewer, viewer.current_mouse_x, viewer.current_mouse_y, picked))
            {
                state.selectedPrimitive = picked;
                state.draggingPrimitive = true;

                const RowVector3d center = primitive_center(state.primitives[picked]);
                const Eigen::Vector3f centerf = center.transpose().cast<float>();
                const Eigen::Vector3f projected = igl::project(centerf, viewer.core().view, viewer.core().proj, viewer.core().viewport);
                state.dragDepth = projected(2);

                RowVector3d mouseWorld;
                project_mouse_at_depth(viewer, viewer.current_mouse_x, viewer.current_mouse_y, state.dragDepth, mouseWorld);
                state.dragOffset = center - mouseWorld;

                draw_scene(viewer);
                return true;
            }

            state.selectedPrimitive = -1;
            state.draggingPrimitive = false;
            draw_scene(viewer);
            return false;
        }

        return false;
    };

    viewer.callback_mouse_move = [&](igl::opengl::glfw::Viewer&, int mouse_x, int mouse_y) -> bool
    {
        if (state.draggingPrimitive && state.selectedPrimitive >= 0 && state.selectedPrimitive < static_cast<int>(state.primitives.size()))
        {
            RowVector3d mouseWorld;
            project_mouse_at_depth(viewer, mouse_x, mouse_y, state.dragDepth, mouseWorld);

            PrimitiveInstance& primitive = state.primitives[state.selectedPrimitive];
            const RowVector3d currentCenter = primitive_center(primitive);
            const RowVector3d targetCenter = mouseWorld + state.dragOffset;
            const RowVector3d delta = targetCenter - currentCenter;
            primitive.V.rowwise() += delta;
            state.uiTranslate = primitive_center(primitive) - mesh_spawn_reference_point();
            state.uiTranslatePrev = state.uiTranslate;

            draw_scene(viewer);
            return true;
        }

        if (!state.drawing)
        {
            return false;
        }

        append_stroke_point(viewer, mouse_x, mouse_y);
        draw_scene(viewer);
        return true;
    };

    viewer.callback_mouse_up = [&](igl::opengl::glfw::Viewer&, int button, int) -> bool
    {
        if (state.draggingPrimitive && button == 0)
        {
            state.draggingPrimitive = false;
            return true;
        }

        if (state.drawing && button == 0)
        {
            state.drawing = false;
            if (state.strokePoints.rows() > 0)
            {
                const RecognitionResult strokePrediction = classify_stroke();
                if (strokePrediction.label != "unknown")
                {
                    add_primitive_from_stroke(viewer, strokePrediction.label);
                    clear_stroke();
                }
                else
                {
                    refresh_prediction();
                }
            }
            draw_scene(viewer);
            return true;
        }
        return false;
    };

    viewer.launch();
    return 0;
}
