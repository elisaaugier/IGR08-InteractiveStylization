#include <igl/read_triangle_mesh.h>
#include <igl/writeOBJ.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/snap_points.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/unique_edge_map.h>
#include <igl/flip_edge.h>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>

#include <ImGuizmo.h>
#include <normalize_unitbox.h>
#include <normal_driven_data.h>
#include <normal_driven_precomputation.h>
#include <normal_driven_prim_single_iteration.h>
#include <get_bounding_box.h>
#include <localAvgMat.h>
#include <sketch2mesh_bridge.h>

#include <ctime>
#include <cmath>
#include <filesystem>
#include <vector>
#include <iostream>
#include <fstream>

#ifndef MESH_PATH
#define MESH_PATH "../meshes/"
#endif

// state of the mode
struct Mode
{
    Eigen::MatrixXd CV; // point constraint
    bool place_constraints = true;
    bool show_base_mesh = false;
    int numCV = 0;
    Eigen::MatrixXi CV_row_col; 

    int hovered_constr_idx = -1;
    int selected_tstyle_idx = -1;

    bool drawing_mode = false;
    bool is_drawing = false;
    std::vector<Eigen::Vector2f> stroke_points;
} state;

// Launch with no CLI argument: ./app.exe
int main(int argc, char *argv[])
{
	using namespace Eigen;
	using namespace std;

    // load mesh
	MatrixXd V, U, VO; //, desiredNormals;
	MatrixXi F;
    string meshName; //, styleMeshName;
    bool useFaceArap = false;

    normal_driven_data data;

    meshName = (argc > 1) ? argv[1] : "cow.obj";

    // read mesh files 

    string file = MESH_PATH + meshName;
    if (!std::filesystem::exists(file))
    {
        cerr << "ERROR: input mesh not found: " << file << endl;
        cerr << "Hint: try running from final-code/build or pass an explicit mesh name." << endl;
        return 1;
    }
    if (!igl::read_triangle_mesh(file, V, F) || V.rows() == 0 || F.rows() == 0)
    {
        cerr << "ERROR: Failed to load input mesh: " << file << endl;
        return 1;
    }
    V = V.rowwise() - V.colwise().minCoeff();
    V /= V.maxCoeff();
    U = V;
    VO = V;

    double spot_reference_diag = 1.0;
    {
        MatrixXd spotV;
        MatrixXi spotF;
        const string spotFile = string(MESH_PATH) + "spot.obj";
        if (igl::read_triangle_mesh(spotFile, spotV, spotF) && spotV.rows() > 0)
        {
            const RowVector3d spotMin = spotV.colwise().minCoeff();
            const RowVector3d spotMax = spotV.colwise().maxCoeff();
            const double diag = (spotMax - spotMin).norm();
            if (diag > 1e-8)
            {
                spot_reference_diag = diag;
            }
        }
    }

    auto normalize_mesh_to_spot = [&] (MatrixXd& meshV) {
        if (meshV.rows() == 0)
        {
            return;
        }

        const RowVector3d vMin = meshV.colwise().minCoeff();
        const RowVector3d vMax = meshV.colwise().maxCoeff();
        const RowVector3d center = 0.5 * (vMin + vMax);
        const double diag = (vMax - vMin).norm();

        meshV = meshV.rowwise() - center;
        if (diag > 1e-8)
        {
            meshV *= (spot_reference_diag / diag);
        }
    };

    auto generate_pyramid_geometry = [&] (int sides, double baseRadius, double height, MatrixXd& Vp, MatrixXi& Fp) {
        if (sides < 3)
        {
            sides = 3;
        }

        Vp.resize(sides + 1, 3);
        for (int i = 0; i < sides; ++i)
        {
            const double angle = 2.0 * std::acos(-1.0) * static_cast<double>(i) / static_cast<double>(sides);
            Vp(i, 0) = baseRadius * std::cos(angle);
            Vp(i, 1) = baseRadius * std::sin(angle);
            Vp(i, 2) = 0.0;
        }
        Vp(sides, 0) = 0.0;
        Vp(sides, 1) = 0.0;
        Vp(sides, 2) = height;

        const int triangleCount = sides;
        const int baseTriangles = std::max(1, sides - 2);
        Fp.resize(triangleCount + baseTriangles, 3);

        for (int i = 0; i < sides; ++i)
        {
            const int next = (i + 1) % sides;
            Fp(i, 0) = i;
            Fp(i, 1) = next;
            Fp(i, 2) = sides;
        }

        for (int i = 0; i < baseTriangles; ++i)
        {
            Fp(triangleCount + i, 0) = 0;
            Fp(triangleCount + i, 1) = i + 1;
            Fp(triangleCount + i, 2) = i + 2;
        }
    };

    auto update_pyramid_style = [&] (target_style& tStyle) {
        if (!tStyle.is_pyramid)
        {
            return;
        }

        Eigen::Matrix3d orientation = tStyle.pyramidOrientation;
        if (!orientation.allFinite() || std::abs(orientation.determinant()) < 1e-10)
        {
            orientation = Eigen::Matrix3d::Identity();
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
                orientation = Eigen::Matrix3d::Identity();
            }
        }

        Eigen::RowVector3d oldBaseCenter = Eigen::RowVector3d::Zero();
        const int oldSides = std::min(std::max(3, tStyle.pyramidSides), std::max(0, static_cast<int>(tStyle.sV.rows()) - 1));
        if (oldSides > 0 && tStyle.sV.rows() >= oldSides)
        {
            for (int i = 0; i < oldSides; ++i)
            {
                oldBaseCenter += tStyle.sV.row(i);
            }
            oldBaseCenter /= static_cast<double>(oldSides);
        }

        Eigen::MatrixXd Vp;
        Eigen::MatrixXi Fp;
        generate_pyramid_geometry(tStyle.pyramidSides, tStyle.pyramidBaseRadius, tStyle.pyramidHeight, Vp, Fp);

        const Eigen::RowVector3d localCenter = Vp.colwise().mean();
        Vp = Vp.rowwise() - localCenter;

        for (int i = 0; i < Vp.rows(); ++i)
        {
            const Eigen::Vector3d p = Vp.row(i).transpose();
            Vp.row(i) = (orientation * p).transpose();
        }

        Eigen::RowVector3d newBaseCenter = Eigen::RowVector3d::Zero();
        const int newSides = std::max(3, tStyle.pyramidSides);
        for (int i = 0; i < newSides; ++i)
        {
            newBaseCenter += Vp.row(i);
        }
        newBaseCenter /= static_cast<double>(newSides);

        const Eigen::RowVector3d translation = oldBaseCenter - newBaseCenter;
        tStyle.sV = Vp.rowwise() + translation;
        tStyle.sF = Fp;
        tStyle.pyramidOrientation = orientation;

        MatrixXd sN;
        MatrixXi useless1, useless2;
        igl::per_face_normals(tStyle.sV, tStyle.sF, sN);
        igl::unique_rows(sN, tStyle.desiredN, useless1, useless2);
    };

    auto load_geometric_style = [&] (const std::string& name, const std::string& filename, RowVector3d center, double lambda, double radius, double falloff, const RowVector3d& primitiveScale)
        -> target_style {
        target_style tStyle;
        tStyle.name = name;
        tStyle.center = center;
        tStyle.lambda = lambda;
        tStyle.radius = radius;
        tStyle.falloff = falloff;

        string styleFile = MESH_PATH + filename;
        MatrixXd sN;
        MatrixXi useless1,useless2; 

        if (filename.find("pyramid") != std::string::npos) {
            tStyle.is_pyramid = true;
            tStyle.pyramidSides = 5;
            tStyle.pyramidBaseRadius = std::max(0.02, std::max(primitiveScale(0), primitiveScale(2)));
            tStyle.pyramidHeight = std::max(0.02, 2.0 * primitiveScale(1));
            tStyle.pyramidOrientation = Eigen::Matrix3d::Identity();
            generate_pyramid_geometry(tStyle.pyramidSides, tStyle.pyramidBaseRadius, tStyle.pyramidHeight, tStyle.sV, tStyle.sF);
            igl::per_face_normals(tStyle.sV, tStyle.sF, sN);
            igl::unique_rows(sN, tStyle.desiredN, useless1, useless2);

            std::shared_ptr<Primitive> prim_ptr = std::make_shared<Box>(Eigen::RowVector3d(tStyle.pyramidBaseRadius, tStyle.pyramidHeight * 0.5, tStyle.pyramidBaseRadius), center);
            tStyle.prim_sdf = prim_ptr;
            data.SDF.addPrimitive(prim_ptr);
        }
        else if (!igl::read_triangle_mesh(styleFile, tStyle.sV,tStyle.sF) || tStyle.sV.rows() == 0 || tStyle.sF.rows() == 0)
        {
            cerr << "ERROR: Failed to load style mesh: " << styleFile << endl;
            return target_style{};
        }
        else
        {
            normalize_mesh_to_spot(tStyle.sV);
            igl::per_face_normals(tStyle.sV, tStyle.sF, sN);
            igl::unique_rows(sN, tStyle.desiredN, useless1, useless2);

            if (filename.find("cube") != std::string::npos) {
            std::shared_ptr<Primitive> prim_ptr = std::make_shared<Box>(primitiveScale, center);
            tStyle.prim_sdf = prim_ptr;
            data.SDF.addPrimitive(prim_ptr);
        }
        else if (filename.find("sphere") != std::string::npos) {
            std::shared_ptr<Primitive> prim_ptr = std::make_shared<Sphere>(radius, center);
            tStyle.prim_sdf = prim_ptr;
            data.SDF.addPrimitive(prim_ptr);
        }
        else if (filename.find("cylinder") != std::string::npos) {
            std::shared_ptr<Primitive> prim_ptr = std::make_shared<Cylinder>(radius * 2.0, radius, center);
            tStyle.prim_sdf = prim_ptr;
            data.SDF.addPrimitive(prim_ptr);
        }
        }

        return tStyle;
    };

    // a supprimer à la fin car utile que si formes dès l'initialiation ?


    cout << "Loaded " << data.styles_list.size() << " geometric styles." << endl;
    cout << "Launching with mesh: " << file << endl;
    
    // get input output name
    // string tmp = styleMeshName;
    // tmp.erase(tmp.end()-4, tmp.end());
    // string outputName = tmp + "_" + meshName;
    // string inputName = "input_" + meshName;
    // cout << outputName << endl;
    // cout << inputName << endl;

    string outputName = "output_" + meshName;
    string inputName = "input_" + meshName;
    cout << "Output file will be: " << outputName << endl;
    cout << "Input file was: " << inputName << endl;

   
    data.V0 = VO;

    // initialize viewer
    igl::opengl::glfw::Viewer viewer;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
	viewer.plugins.push_back(&menu);

    MatrixXd Vsdf;
    MatrixXi Fsdf;

    auto sdfId = viewer.append_mesh();
    data.SDF.getMesh(Vsdf, Fsdf);
    viewer.data(sdfId).clear();
    viewer.data(sdfId).set_mesh(Vsdf, Fsdf);
    viewer.data(sdfId).compute_normals();
    Eigen::MatrixXd C(Fsdf.rows(), 4);
    C.col(0).setConstant(0.8);  // R
    C.col(1).setConstant(0.0);  // G
    C.col(2).setConstant(0.8);  // B
    C.col(3).setConstant(0.3);  // alpha

    viewer.data(sdfId).set_colors(C);
    viewer.data(sdfId).show_faces = false;
    viewer.data(sdfId).show_lines = false;
    ImGuizmo::OPERATION current_operation = ImGuizmo::TRANSLATE;
    sketch2mesh_bridge::Bridge sketchBridge;
    sketchBridge.initialize(MESH_PATH);

    static bool show_sdf_preview = false;

    // rotation parameters
    double theta_x = 10.0; 
	double theta_y = 10.0; 
	double theta_z = 10.0; 
	Matrix3d Rx, iRx, Ry, iRy, Rz, iRz;
    {
        Rx << 
            1., 0., 0.,
            0., cos(theta_x/180.*3.14), -sin(theta_x/180.*3.14),
            0., sin(theta_x/180.*3.14), cos(theta_x/180.*3.14);
        Ry <<
            cos(theta_y/180.*3.14), 0., sin(theta_y/180.*3.14),
            0., 1., 0.,
            -sin(theta_y/180.*3.14), 0, cos(theta_y/180.*3.14);
        Rz <<
            cos(theta_z/180.*3.14), -sin(theta_z/180.*3.14), 0.,
            sin(theta_z/180.*3.14), cos(theta_z/180.*3.14), 0.,
            0., 0., 1;
    }

    // ------------------------------------------------------------------------
    // adding dynamically target styles
    // ------------------------------------------------------------------------

    auto build_style_vertices = [&] (const target_style& tStyle) -> MatrixXd {
        MatrixXd Vstyle = tStyle.sV;
        if (Vstyle.rows() == 0)
        {
            return Vstyle;
        }

        const RowVector3d baseCenter = Vstyle.colwise().mean();
        Vstyle = Vstyle.rowwise() - baseCenter;

        if (tStyle.prim_sdf)
        {
            const Eigen::Vector3d targetHalfExt = tStyle.prim_sdf->getScale();
            const RowVector3d vmin = Vstyle.colwise().minCoeff();
            const RowVector3d vmax = Vstyle.colwise().maxCoeff();
            const RowVector3d baseHalfExt = 0.5 * (vmax - vmin);

            const double fx = targetHalfExt(0) / std::max(1e-8, baseHalfExt(0));
            const double fy = targetHalfExt(1) / std::max(1e-8, baseHalfExt(1));
            const double fz = targetHalfExt(2) / std::max(1e-8, baseHalfExt(2));

            Vstyle.col(0) *= fx;
            Vstyle.col(1) *= fy;
            Vstyle.col(2) *= fz;
        }

        Vstyle = Vstyle.rowwise() + tStyle.center;
        return Vstyle;
    };

    const Eigen::RowVector4d kPrimitiveDefaultColor(0.65, 0.82, 1.00, 0.28);  // pale blue, slightly transparent
    const Eigen::RowVector4d kPrimitiveSelectedColor(1.00, 0.95, 0.55, 0.35); // pale yellow, slightly transparent

    auto apply_style_color = [&] (target_style& tStyle, bool selected) {
        if (tStyle.viewer_id == -1)
        {
            return;
        }
        viewer.data(tStyle.viewer_id).set_colors(selected ? kPrimitiveSelectedColor : kPrimitiveDefaultColor);
    };

    auto render_style = [&] (target_style& tStyle) {
        tStyle.viewer_id = viewer.append_mesh();
        MatrixXd primitive_V = build_style_vertices(tStyle);
        viewer.data(tStyle.viewer_id).set_mesh(primitive_V, tStyle.sF);
        apply_style_color(tStyle, false);
        viewer.data(tStyle.viewer_id).show_lines = tStyle.is_visible; // display edges
        viewer.data(tStyle.viewer_id).show_faces = tStyle.is_visible;
    };

    auto update_style = [&] (target_style& tStyle) {
        viewer.data(tStyle.viewer_id).show_lines = tStyle.is_visible; // display edges
        viewer.data(tStyle.viewer_id).show_faces = tStyle.is_visible;
    };

    auto add_new_style = [&] (const std::string& base_name, const std::string& filename, RowVector3d center, double lambda, double radius, double falloff, const RowVector3d& primitiveScale = RowVector3d::Ones()) {
        // en considérant pr le moment que l'utilisateur peut pas renommer les noms des formes
        int count = 0;
        for (const auto& s : data.styles_list) {
            if (s.name.size() >= base_name.size() && s.name.substr(0, base_name.size()) == base_name) {
                count++;
            } 
        }
        std::string unique_name = base_name + "_" + std::to_string(count);

        target_style new_style = load_geometric_style(
            unique_name, filename, center, lambda, radius, falloff, primitiveScale
        );
        render_style(new_style);
        data.styles_list.push_back(new_style);
    };

    auto refresh_style_mesh = [&] (target_style& tStyle) {
        if (!tStyle.is_pyramid)
        {
            return build_style_vertices(tStyle);
        }

        update_pyramid_style(tStyle);
        return tStyle.sV;
    };

    auto rebuild_sdf_view = [&]() {
        data.SDF = SDF();
        for (auto& tStyle : data.styles_list)
        {
            if (tStyle.prim_sdf)
            {
                data.SDF.addPrimitive(tStyle.prim_sdf);
            }
        }

        data.SDF.getMesh(Vsdf, Fsdf);
        viewer.data(sdfId).clear();
        if (Vsdf.rows() > 0 && Fsdf.rows() > 0)
        {
            viewer.data(sdfId).set_mesh(Vsdf, Fsdf);
            viewer.data(sdfId).compute_normals();
            Eigen::MatrixXd Cnew(Fsdf.rows(), 4);
            Cnew.col(0).setConstant(0.8);
            Cnew.col(1).setConstant(0.0);
            Cnew.col(2).setConstant(0.8);
            Cnew.col(3).setConstant(0.3);
            viewer.data(sdfId).set_colors(Cnew);
        }
        viewer.data(sdfId).show_faces = show_sdf_preview;
        viewer.data(sdfId).show_lines = show_sdf_preview;
    };

    auto delete_selected_style = [&]() -> bool {
        const int idx = state.selected_tstyle_idx;
        if (idx < 0 || idx >= static_cast<int>(data.styles_list.size()))
        {
            return false;
        }

        const int removed_viewer_id = data.styles_list[idx].viewer_id;
        if (removed_viewer_id >= 0 && removed_viewer_id < static_cast<int>(viewer.data_list.size()))
        {
            if (!viewer.erase_mesh(removed_viewer_id))
            {
                viewer.data(removed_viewer_id).clear();
            }
        }

        data.styles_list.erase(data.styles_list.begin() + idx);
        for (auto& tStyle : data.styles_list)
        {
            if (tStyle.viewer_id > removed_viewer_id)
            {
                tStyle.viewer_id -= 1;
            }
        }

        state.selected_tstyle_idx = -1;
        rebuild_sdf_view();
        return true;
    };

    // draw additional windows

	menu.callback_draw_viewer_window = [](){};

    static bool show_all_targets = true;

	menu.callback_draw_custom_window = [&]()
	{
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // ~~~~~~~~~~~~~~  W I N D O W   1  ~~~~~~~~~~~~~~
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

		// Define next window position + size
		{
			ImGui::SetNextWindowPos(ImVec2(0.f * menu.menu_scaling(), 0), ImGuiCond_FirstUseEver);
			ImGui::SetNextWindowSize(ImVec2(300, 400), ImGuiCond_FirstUseEver);
			ImGui::Begin(
				"Normal-Driven Editing", nullptr,
				ImGuiWindowFlags_NoSavedSettings
			);

            ImGuizmo::SetOrthographic(false);
            ImGuizmo::BeginFrame();

            ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
            ImGuizmo::SetDrawlist(draw_list);
            ImGuizmo::SetRect(0, 0, viewer.core().viewport(2), viewer.core().viewport(3));

            Eigen::Matrix4f view = viewer.core().view.cast<float>();
            Eigen::Matrix4f proj = viewer.core().proj.cast<float>();

            for (int i = 0; i < data.styles_list.size(); i++) 
            {
                auto& tStyle = data.styles_list[i];

                if (tStyle.is_visible && i == state.selected_tstyle_idx) 
                {
                    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
                    model.block<3,1>(0,3) = tStyle.center.cast<float>().transpose();   
                    
                    // Track scale vector per primitive (not just radius)
                    static int last_selected_idx = -1;
                    static Eigen::Vector3d gizmo_scale = Eigen::Vector3d::Ones();
                    
                    if (i != last_selected_idx) {
                        data.SDF.resetScaleTracking(i);
                        gizmo_scale = data.SDF.getScale(i);
                        last_selected_idx = i;
                    }
                    
                    model(0,0) = gizmo_scale(0);
                    model(1,1) = gizmo_scale(1);
                    model(2,2) = gizmo_scale(2);

                    //current_operation = ImGuizmo::TRANSLATE;

                    if (ImGuizmo::Manipulate(view.data(), proj.data(), current_operation, ImGuizmo::LOCAL, model.data()))
                    {
                        Eigen::Vector3f new_center = model.block<3,1>(0,3);
                        Eigen::Vector3d new_pos = new_center.cast<double>();
                        tStyle.center = new_pos.transpose();
                        
                        // Extract scale from model matrix
                        Eigen::Vector3d scale(model(0,0), model(1,1), model(2,2));
                        
                        data.SDF.setRuntimeTransform(i, new_pos, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Ones());
                        if (tStyle.prim_sdf) {
                            tStyle.prim_sdf->setRuntimeTransform(new_pos, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Ones());
                        }
                        data.SDF.getMesh(Vsdf, Fsdf);
                        viewer.data(sdfId).set_mesh(Vsdf, Fsdf);
                        viewer.data(sdfId).compute_normals();
                        viewer.data(sdfId).show_faces = show_sdf_preview;
                        viewer.data(sdfId).show_lines = show_sdf_preview;
                        
                        if (current_operation == ImGuizmo::SCALE) {
                            // Update tracked scale and apply to SDF
                            gizmo_scale = scale;
                            data.SDF.setScale(i, scale);
                            // moi en +
                            if (tStyle.prim_sdf) {
                                tStyle.prim_sdf->setScale(scale); 
                            }
                            // data.SDF.getMesh(Vsdf, Fsdf);
                            // viewer.data(sdfId).set_mesh(Vsdf, Fsdf);
                            // viewer.data(sdfId).compute_normals();
                        }

                        data.SDF.getMesh(Vsdf, Fsdf);
                        viewer.data(sdfId).set_mesh(Vsdf, Fsdf);
                        viewer.data(sdfId).compute_normals();
                        viewer.data(sdfId).show_faces = show_sdf_preview;
                        viewer.data(sdfId).show_lines = show_sdf_preview;

                        if (tStyle.viewer_id != -1) {
                            MatrixXd new_V = build_style_vertices(tStyle);
                            viewer.data(tStyle.viewer_id).set_vertices(new_V);
                            // viewer.data(tStyle.viewer_id).clear();
                        }
                    }
                }
            }
		}
        // Expose the same variable directly
        {
            // How to use
            ImGui::Text("Instructions");
            ImGui::BulletText("[click]  place constrained points");
            ImGui::BulletText("[space]  start/stop optimization");
            ImGui::BulletText("R        reset ");
            ImGui::BulletText("</>      decrease/increase lambda");
            ImGui::BulletText("Q/W      rotate x-axis");
            ImGui::BulletText("A/S      rotate y-axis");
            ImGui::BulletText("Z/X      rotate z-axis");
            ImGui::BulletText("L        show edges");
            ImGui::Text(" ");
        }
        {
            ImGui::PushItemWidth(-80);
            if (!data.styles_list.empty())
            {
                ImGui::DragScalar("lambda", ImGuiDataType_Double, &data.styles_list[0].lambda, 5e-1, 0, 0, "%.1e");
            }
            else
            {
                ImGui::TextUnformatted("lambda: add a target style first");
            }
		    ImGui::DragScalar("gamma", ImGuiDataType_Double, &data.gamma, 1e-2, 0, 0, "%.3f");
            ImGui::PopItemWidth();
        }
        {
            // output file name
            // ImGui::InputText(".obj", outputName, IM_ARRAYSIZE(outputName));
        }
        {
            ImGui::Checkbox("use face ARAP", &useFaceArap);
        }
        {
            ImGui::Checkbox("allow local scaling", &data.useScale);
        }
        if (ImGui::Button("save output mesh", ImVec2(-1,0)))
		{
			igl::writeOBJ("../output.obj", U, F);
		}

        ImGui::Separator();
        sketchBridge.drawUi(viewer);

        sketch2mesh_bridge::AddStyleRequest request;
        if (sketchBridge.consumeRequest(request) && request.valid)
        {
            if (request.isBaseMesh)
            {
                // Load as the base deformable mesh (V/F).
                const std::string meshFile = std::string(MESH_PATH) + request.filename;
                Eigen::MatrixXd newV;
                Eigen::MatrixXi newF;
                if (igl::read_triangle_mesh(meshFile, newV, newF) && newV.rows() > 0 && newF.rows() > 0)
                {
                    newV = newV.rowwise() - newV.colwise().minCoeff();
                    newV /= newV.maxCoeff();
                    V = newV;
                    F = newF;
                    U = V;
                    VO = V;
                    data.V0 = V;  // keep V0 in sync so precomputation is valid
                    data.b.resize(0);
                    data.bc.resize(0, 3);
                    state.CV = Eigen::MatrixXd();
                    state.CV_row_col = Eigen::MatrixXi();
                    state.CV_row_col.resize(100, 2);
                    state.numCV = 0;
                    state.place_constraints = true;
                    state.show_base_mesh = true;
                    // draw() called automatically on next frame
                }
            }
            else
            {
                add_new_style(request.baseName,
                              request.filename,
                              request.center,
                              request.lambda,
                              request.radius,
                              request.falloff,
                              request.primitiveScale);
            }
        }
        ImGui::End();


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // ~~~~~~~~~~~~~~  W I N D O W   2  ~~~~~~~~~~~~~~
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        {
            // Anchor the layers window to the top-right corner of the viewport.
            ImGui::SetNextWindowPos(
                ImVec2(static_cast<float>(viewer.core().viewport(2) - 10), 10.0f),
                ImGuiCond_Always,
                ImVec2(1.0f, 0.0f)
            );
			ImGui::SetNextWindowSize(ImVec2(300, 400), ImGuiCond_FirstUseEver);
			ImGui::Begin(
				"Target-Styles Layers", nullptr,
				ImGuiWindowFlags_NoSavedSettings
			);

            if (ImGui::Checkbox("Show all", &show_all_targets)) {
                for (auto& tStyle : data.styles_list) {
                    if (tStyle.viewer_id != -1) {
                        viewer.data(tStyle.viewer_id).show_lines = show_all_targets ? tStyle.is_visible : false;
                        viewer.data(tStyle.viewer_id).show_faces = show_all_targets ? tStyle.is_visible : false;
                    }
                }
            }

            ImGui::Separator();
            ImGui::Text("Test");
            ImGui::Spacing();

            if (state.selected_tstyle_idx >= 0 && state.selected_tstyle_idx < static_cast<int>(data.styles_list.size()))
            {
                ImGui::Text("Selected: %s", data.styles_list[state.selected_tstyle_idx].name.c_str());
                if (ImGui::Button("Delete Selected Primitive", ImVec2(-1, 0)))
                {
                    delete_selected_style();
                    ImGui::End();
                    return;
                }
            }
            else
            {
                ImGui::TextUnformatted("Selected: none");
            }
            ImGui::Separator();

            int pending_delete_idx = -1;

            for (int i=0; i < data.styles_list.size(); i++) {
                auto& tStyle = data.styles_list[i];
                ImGui::PushID(i);

                // pour déplier/replier
                bool is_open = ImGui::TreeNodeEx(tStyle.name.c_str(), ImGuiTreeNodeFlags_Framed | ImGuiTreeNodeFlags_AllowItemOverlap);
                
                // pour visible/invisible
                ImGui::SameLine(ImGui::GetWindowWidth() - 40); // Aligner à droite
                bool prev_visible = tStyle.is_visible;
                if (ImGui::Checkbox("##vis", &tStyle.is_visible)) {
                    if (tStyle.viewer_id != -1 && show_all_targets) {
                        update_style(tStyle);
                    }
                }

                if (is_open) 
                {
                    bool changed = false;

                    if (ImGui::Button("Delete Primitive", ImVec2(-1, 0)))
                    {
                        pending_delete_idx = i;
                    }
                    ImGui::Separator();

                    // Sliders de Position
                    float center_f[3] = { (float)tStyle.center(0), (float)tStyle.center(1), (float)tStyle.center(2) };
                    if (ImGui::SliderFloat3("Position", center_f, -2.0f, 2.0f, "%.3f")) {
                        tStyle.center << center_f[0], center_f[1], center_f[2];
                        changed = true;
                        if (tStyle.prim_sdf) {
                            tStyle.prim_sdf->setRuntimeTransform(tStyle.center.transpose(), Eigen::Matrix3d::Identity(), Eigen::Vector3d::Ones());
                        }
                    }

                    // Sliders de Paramètres
                    // if (ImGui::DragScalar("Rayon", ImGuiDataType_Double, &tStyle.prim_sdf->getScale(), 0.01f, 0, 0, "%.2f")) changed = true;
                    if (tStyle.prim_sdf)
                    {
                        if (auto* sphere = dynamic_cast<Sphere*>(tStyle.prim_sdf.get()))
                        {
                            double radius_value = sphere->getScale()(0);
                            double radius_min = 0.02;
                            double radius_max = 3.0;
                            if (ImGui::SliderScalar("Rayon", ImGuiDataType_Double, &radius_value, &radius_min, &radius_max, "%.2f"))
                            {
                                changed = true;
                                const Eigen::Vector3d new_scale = Eigen::Vector3d::Constant(std::max(0.02, radius_value));
                                data.SDF.setScale(i, new_scale);
                            }
                        }
                        else if (auto* box = dynamic_cast<Box*>(tStyle.prim_sdf.get()))
                        {
                            if (tStyle.is_pyramid)
                            {
                                double base_radius = tStyle.pyramidBaseRadius;
                                double height = tStyle.pyramidHeight;

                                double pyr_radius_min = 0.02;
                                double pyr_radius_max = 3.0;
                                if (ImGui::SliderScalar("Rayon base", ImGuiDataType_Double, &base_radius, &pyr_radius_min, &pyr_radius_max, "%.2f"))
                                {
                                    changed = true;
                                    tStyle.pyramidBaseRadius = std::max(0.02, base_radius);
                                }
                                double pyr_height_min = 0.02;
                                double pyr_height_max = 4.0;
                                if (ImGui::SliderScalar("Hauteur", ImGuiDataType_Double, &height, &pyr_height_min, &pyr_height_max, "%.2f"))
                                {
                                    changed = true;
                                    tStyle.pyramidHeight = std::max(0.02, height);
                                }
                            }
                            else
                            {
                                Eigen::Vector3d current_scale = box->getScale();
                                double length = 2.0 * current_scale(0);
                                double width = 2.0 * current_scale(2);
                                double height = 2.0 * current_scale(1);

                                double dim_min = 0.02;
                                double dim_max = 4.0;
                                if (ImGui::SliderScalar("Longueur", ImGuiDataType_Double, &length, &dim_min, &dim_max, "%.2f"))
                                {
                                    changed = true;
                                    current_scale(0) = std::max(0.02, 0.5 * length);
                                }
                                if (ImGui::SliderScalar("Largeur", ImGuiDataType_Double, &width, &dim_min, &dim_max, "%.2f"))
                                {
                                    changed = true;
                                    current_scale(2) = std::max(0.02, 0.5 * width);
                                }
                                if (ImGui::SliderScalar("Hauteur", ImGuiDataType_Double, &height, &dim_min, &dim_max, "%.2f"))
                                {
                                    changed = true;
                                    current_scale(1) = std::max(0.02, 0.5 * height);
                                }

                                if (changed)
                                {
                                    data.SDF.setScale(i, current_scale);
                                }
                            }

                            if (tStyle.is_pyramid)
                            {
                                if (changed)
                                {
                                    if (tStyle.viewer_id != -1)
                                    {
                                        viewer.data(tStyle.viewer_id).set_mesh(tStyle.sV, tStyle.sF);
                                    }
                                    Eigen::Vector3d current_scale = Eigen::Vector3d(tStyle.pyramidBaseRadius,
                                                                                   0.5 * tStyle.pyramidHeight,
                                                                                   tStyle.pyramidBaseRadius);
                                    data.SDF.setScale(i, current_scale);
                                }
                            }
                        }
                    }
                    double style_lambda_min = 0.0;
                    double style_lambda_max = 20.0;
                    ImGui::SliderScalar("Lambda (Force)", ImGuiDataType_Double, &tStyle.lambda, &style_lambda_min, &style_lambda_max, "%.2f");
                    double falloff_min = 0.0;
                    double falloff_max = 10.0;
                    ImGui::SliderScalar("Falloff", ImGuiDataType_Double, &tStyle.falloff, &falloff_min, &falloff_max, "%.2f");

                    // Mise à jour visuelle du mesh transparent si on bouge un slider (position/rayon)
                    if (changed && tStyle.is_visible && show_all_targets && tStyle.viewer_id != -1) {
                        if (tStyle.is_pyramid)
                        {
                            viewer.data(tStyle.viewer_id).set_mesh(tStyle.sV, tStyle.sF);
                        }
                        else
                        {
                            MatrixXd new_V = build_style_vertices(tStyle);
                            viewer.data(tStyle.viewer_id).set_vertices(new_V);
                        }
                    }

                    ImGui::TreePop(); // Fin de l'accordéon
                }

                ImGui::PopID();

                if (pending_delete_idx != -1)
                {
                    break;
                }

            }

            if (pending_delete_idx != -1)
            {
                state.selected_tstyle_idx = pending_delete_idx;
                delete_selected_style();
            }
            
        }
        ImGui::End();

    };

    // recompute state.CV function
    int maxCV = 100;
    state.CV_row_col.resize(maxCV,2); // assume nor more than 100 constraints
    const auto & resetCV = [&]()
    {
        for (int ii=0; ii<state.numCV; ii++)
        {
            int fid = state.CV_row_col(ii,0);
            int c   = state.CV_row_col(ii,1);
            RowVector3d new_c = V.row(F(fid,c));
            state.CV.row(ii) = new_c;
        }
    };

    // draw function

    // const auto & draw = [&]()
    // {
    //     const Eigen::RowVector3d blue(149.0/255, 217.0/255, 244.0/255);
    //     const Eigen::RowVector3d red(250.0/255, 114.0/255, 104.0/255);
    //     const Eigen::RowVector3d gray(200.0/255, 200.0/255, 200.0/255);

    //     if(state.place_constraints)
    //     {
    //         viewer.data().clear();
    //         viewer.data().face_based = true;
    //         viewer.data().set_mesh(V,F);
    //         viewer.data().set_colors(gray);
    //         viewer.data().set_points(state.CV, red);

    //         // draw bounding box
    //         MatrixXd V_box;
    //         MatrixXi E_box;
    //         get_bounding_box(V, V_box, E_box);
    //         viewer.data().add_points(V_box, red);
    //         for (unsigned i=0;i<E_box.rows(); ++i)
    //             viewer.data().add_edges(V_box.row(E_box(i,0)),V_box.row(E_box(i,1)),red);
    //     }
    //     else
    //     {   
    //         normal_driven_prim_single_iteration(F,U,data);
    //         viewer.data().clear();
    //         viewer.data().face_based = true;
    //         viewer.data().set_mesh(U,F);
    //         viewer.data().set_colors(blue);
    //         viewer.data().set_points(state.CV, red);

    //         // draw bounding box
    //         MatrixXd V_box;
    //         MatrixXi E_box;
    //         get_bounding_box(U, V_box, E_box);
    //         viewer.data().add_points(V_box, red);
    //         for (unsigned i=0;i<E_box.rows(); ++i)
    //             viewer.data().add_edges(V_box.row(E_box(i,0)),V_box.row(E_box(i,1)),red);
    //     }
    // };

    const auto & set_contraints_colors = [&]()
    {
        if (state.CV.rows() > 0)
        {
            Eigen::MatrixXd C_colors(state.CV.rows(), 3);
            
            for (int i = 0; i < state.CV.rows(); ++i) {
                if (i == state.hovered_constr_idx) {
                    C_colors.row(i) << 1.0, 1.0, 0.0;
                } else {
                    C_colors.row(i) << 1.0, 0.0, 0.0; 
                }
            }
              viewer.data(0).set_points(state.CV, C_colors);
        }
    };

    // draw function: multi-tstyles version

    const auto & draw = [&]()
    {
        const Eigen::RowVector3d blue(149.0/255, 217.0/255, 244.0/255);
        const Eigen::RowVector3d red(250.0/255, 114.0/255, 104.0/255);
        const Eigen::RowVector3d gray(200.0/255, 200.0/255, 200.0/255);

        // viewer.data_list[0]: deformed mesh's layer
        viewer.data_list[0].clear();
        viewer.data_list[0].face_based = true;
        viewer.data_list[0].show_lines = false;

        if (!state.show_base_mesh)
        {
            return;
        }

        if(state.place_constraints)
        // IDLE mode (NO deformation)
        {
            viewer.data_list[0].set_mesh(V, F);
            viewer.data_list[0].set_colors(gray);
            set_contraints_colors();
        }
        else
        // OPTIMISATION mode (deformation) — only runs if there are target styles
        {
            if (!data.styles_list.empty())
            {
                normal_driven_prim_single_iteration(F, U, data);
            }
            viewer.data_list[0].set_mesh(U, F);
            viewer.data_list[0].set_colors(blue);
            set_contraints_colors();
        }
    };

    // when key pressed do 
    viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer &, unsigned int key, int mod)
    {
        switch(key)
        {
            case '>':
            case '.':
            {
                for(auto& p : data.styles_list) p.lambda += 0.02;
                break;
            }
            case '<':
            case ',':
            {
                for(auto& p : data.styles_list) p.lambda -= 0.02;
                break;
            }
            case 'R':
            case 'r':
            {
                if (state.place_constraints)
                {
                    state.CV = MatrixXd();
                    state.CV_row_col = MatrixXi();
                    state.CV_row_col.resize(maxCV,2);
                    state.numCV = 0;
                    V = VO;
                }
                break;
            }
            case 'Q':
            case 'q':
            {
                U = (Rx * U.transpose()).transpose();
                if (state.place_constraints)
                {
                    V = (Rx * V.transpose()).transpose();
                    resetCV();
                    draw();
                }
                break;
            }
            case 'W':
            case 'w':
            {
                U = (Rx.transpose() * U.transpose()).transpose();
                if (state.place_constraints)
                {
                    V = (Rx.transpose() * V.transpose()).transpose();
                    resetCV();
                    draw();
                }
                break;
            }
            case 'A':
            case 'a':
            {
                U = (Ry * U.transpose()).transpose();
                if (state.place_constraints)
                {
                    V = (Ry * V.transpose()).transpose();
                    resetCV();
                    draw();
                }
                break;
            }
            case 'S':
            case 's':
            {
                U = (Ry.transpose() * U.transpose()).transpose();
                if (state.place_constraints)
                {
                    V = (Ry.transpose() * V.transpose()).transpose();
                    resetCV();
                    draw();
                }
                break;
            }
            case 'Z':
            case 'z':
            {
                U = (Rz * U.transpose()).transpose();
                if (state.place_constraints)
                {
                    V = (Rz * V.transpose()).transpose();
                    resetCV();
                    draw();
                }
                break;
            }
            case 'X':
            case 'x':
            {
                U = (Rz.transpose() * U.transpose()).transpose();
                if (state.place_constraints)
                {
                    V = (Rz.transpose() * V.transpose()).transpose();
                    resetCV();
                    draw();
                }
                break;
            }
            case 'l':
            case 'L': {
                if (current_operation == ImGuizmo::TRANSLATE) {
                    current_operation = ImGuizmo::SCALE;
                } else {
                    current_operation = ImGuizmo::TRANSLATE;
                }
                break;
            }
            case 'd':
            case 'D':
            case 8:   // ASCII Backspace (some backends)
            case 127: // ASCII Delete (some backends)
            case 261: // GLFW_KEY_DELETE
            case 259: // GLFW_KEY_BACKSPACE
            {
                if (delete_selected_style())
                {
                    draw();
                    return true;
                }
                break;
            }
            case ' ':
            {
                if (!state.show_base_mesh) break; // no mesh loaded yet
                // Only toggle if there are styles to optimize against.
                if (!state.place_constraints)
                {
                    // going back to idle: always allowed
                    state.place_constraints = true;
                    resetCV();
                    draw();
                    break;
                }
                // going into optimization: require at least one style
                if (data.styles_list.empty()) break;
                state.place_constraints = false;
                if (!state.place_constraints && state.CV.rows()>0)
                {
                    // set constrained points
                    igl::snap_points(state.CV, V, data.b);
                    data.bc.setZero(data.b.size(), 3);
                    for (int ii=0; ii<data.b.size(); ii++)
                        data.bc.row(ii) = V.row(data.b(ii));

                    // set cubic normals
                    // data.desiredN.resize(desiredNormals.rows(),desiredNormals.cols());
                    // data.desiredN = desiredNormals;

                    // pre computation
                    U = V;
                    if (useFaceArap)
                        normal_driven_precomputation(V,F,1,data); // use face arap
                    else
                        normal_driven_precomputation(V,F,0,data); // use vertex arap
                }
                else if(!state.place_constraints && state.CV.rows()==0) 
                {
                    // if not constraint points, then set the F(0,0) to be the contrained point
                    state.CV = MatrixXd();
                    state.CV.resize(1,3);
                    RowVector3d new_c = V.row(F(0,0));
                    state.CV.row(0) << new_c;
                    state.CV_row_col = MatrixXi();
                    state.CV_row_col.resize(maxCV,2);
                    state.CV_row_col.row(0) << 0, 0;
                    state.numCV = 1;

                    // set cubic normals
                    // data.desiredN.resize(desiredNormals.rows(),desiredNormals.cols());
                    // data.desiredN = desiredNormals;

                    // set constrained points
                    igl::snap_points(state.CV, V, data.b);
                    data.bc.setZero(data.b.size(), 3);
                    for (int ii=0; ii<data.b.size(); ii++)
                        data.bc.row(ii) = V.row(data.b(ii));

                    // pre computation
                    U = V;
                    if (useFaceArap)
                        normal_driven_precomputation(V,F,1,data); // use face arap
                    else
                        normal_driven_precomputation(V,F,0,data); // use vertex arap
                }
                break;
            }
            default:
                return false;
        }
        draw();
        return true;
    };

    // when click mouse
    viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer&, int button, int modifier)->bool
    {
        double x = viewer.current_mouse_x;
        double y = viewer.core().viewport(3) - viewer.current_mouse_y;

        // if (state.place_constraints)
        // {
        //     // Find closest point on mesh to mouse position
        //     int fid; // face id
        //     Vector3f bary; // barycentric coord
        //     if(igl::unproject_onto_mesh(Vector2f(x,y), viewer.core().view, viewer.core().proj, viewer.core().viewport, V, F, fid, bary))
        //     {
        //         long c;
        //         bary.maxCoeff(&c);
                
        //         state.CV_row_col.row(state.numCV) << fid, c;
        //         state.numCV++;

        //         RowVector3d new_c = V.row(F(fid,c));
        //         if(state.CV.size()==0 || (state.CV.rowwise()-new_c).rowwise().norm().minCoeff() > 0)
        //         {
        //             state.CV.conservativeResize(state.CV.rows()+1,3);
        //             state.CV.row(state.CV.rows()-1) = new_c;
        //             draw();
        //             return true;
        //         }
        //     }
        // }

        // select a target style mesh (disabled when Shift is held for drawing)
        bool isShiftPressed = (modifier & 1); // GLFW_MOD_SHIFT = 1
        if (!isShiftPressed && button==0)
        {
            if (state.selected_tstyle_idx != -1) {
                auto& selected_tstyle = data.styles_list[state.selected_tstyle_idx];
                apply_style_color(selected_tstyle, false);
            }
            int closest_tstyle_idx = -1;
            double min_dist = 1e20;

            // ray casting
            for (int i=0; i<data.styles_list.size(); i++) {
                auto& tStyle = data.styles_list[i];
                if (!tStyle.is_visible || tStyle.viewer_id == -1) continue;
                
                Eigen::MatrixXd tStyle_V = build_style_vertices(tStyle);
                int fid;
                Vector3f bary;

                if (igl::unproject_onto_mesh(Vector2f(x,y), viewer.core().view, viewer.core().proj, viewer.core().viewport, tStyle_V, tStyle.sF, fid, bary))
                {
                    RowVector3d hit_point =
                        bary(0) * tStyle_V.row(tStyle.sF(fid, 0)) + 
                        bary(1) * tStyle_V.row(tStyle.sF(fid, 1)) + 
                        bary(2) * tStyle_V.row(tStyle.sF(fid, 2));

                    Vector3f cam_pos_float = (
                        -viewer.core().view.block<3,3>(0,0).transpose() * viewer.core().view.block<3,1>(0,3)
                    ).eval();
                    Vector3d cam_pos = cam_pos_float.cast<double>();

                    double dist_to_cam = (hit_point - cam_pos.transpose()).norm();

                    if (dist_to_cam < min_dist) {
                        min_dist = dist_to_cam;
                        closest_tstyle_idx = i;
                    }
                }
            }

            if (closest_tstyle_idx != -1) {
                state.selected_tstyle_idx = closest_tstyle_idx;
                auto& selected_tstyle = data.styles_list[closest_tstyle_idx];
                apply_style_color(selected_tstyle, true);
                return true;
            }
            else {
                state.selected_tstyle_idx = -1;
            }
        }

        // add/remove constraints (only in idle mode, not when drawing or selecting primitives)
        if (!sketchBridge.isDrawingMode() && state.place_constraints && button == 0)
        {
            if (state.hovered_constr_idx != -1)
            // constraint removal
            {
                int to_remove_idx = state.hovered_constr_idx;
                MatrixXd new_CV(state.CV.rows()-1, 3);
                new_CV.topRows(to_remove_idx) = state.CV.topRows(to_remove_idx);
                new_CV.bottomRows(state.CV.rows() - to_remove_idx -1) = state.CV.bottomRows(state.CV.rows() - to_remove_idx - 1);
                state.CV = new_CV;

                for (int i=to_remove_idx; i<state.numCV-1; i++) {
                    state.CV_row_col.row(i) = state.CV_row_col.row(i+1);

                }
                state.numCV--;
                state.hovered_constr_idx = -1;
                draw();
            }
            else
            // constraint addition
            {
                int fid;
                Vector3f bary;
                if(igl::unproject_onto_mesh(Vector2f(x,y), viewer.core().view, viewer.core().proj, viewer.core().viewport, V, F, fid, bary))
                {  
                    long c;
                    bary.maxCoeff(&c);
                    RowVector3d new_c = V.row(F(fid,c));

                    state.CV_row_col.row(state.numCV) << fid, c;
                    state.numCV++;

                    state.CV.conservativeResize(state.CV.rows() + 1, 3);
                    state.CV.row(state.CV.rows() - 1) = new_c;

                    draw();
                    return true;
                }
            }
        }

        // draw in 2D (only when Shift is held)
        if (isShiftPressed && sketchBridge.onMouseDown(viewer, button)) {
            return true;
        }

        return false;
    };

    viewer.callback_mouse_move = [&] (igl::opengl::glfw::Viewer&, int mouse_x, int mouse_y) -> bool 
    {
        bool isShiftPressed = ImGui::GetIO().KeyShift;
        double x = viewer.current_mouse_x;
        double y = viewer.core().viewport(3) - viewer.current_mouse_y;

        if (state.place_constraints && !isShiftPressed) {

            int old_hovered = state.hovered_constr_idx;
            state.hovered_constr_idx = -1;
            int fid;
            Vector3f bary;

            if(igl::unproject_onto_mesh(Vector2f(x,y), viewer.core().view, viewer.core().proj, viewer.core().viewport, V, F, fid, bary))
            {
                long c;
                bary.maxCoeff(&c);
                RowVector3d hovered_pt = V.row(F(fid,c)); 
      
                if (state.CV.rows() > 0) 
                {
                    VectorXd dists = (state.CV.rowwise() - hovered_pt).rowwise().norm();
                    int min_row;
                    double min_dist = dists.minCoeff(&min_row);
                    
                    if (min_dist < 1e-5) {
                        state.hovered_constr_idx = min_row;
                    }
                }
            } 
            if (state.hovered_constr_idx != old_hovered) draw();
        }

        if (sketchBridge.onMouseMove(viewer)) {
            return true;
        }
        return false;
    };


    viewer.callback_mouse_up = [&](igl::opengl::glfw::Viewer&, int button, int modifier)->bool
    {
        bool isShiftPressed = ImGui::GetIO().KeyShift;
        if (sketchBridge.onMouseUp(viewer, button, V, F)) {
            sketch2mesh_bridge::AddStyleRequest request;
            if (sketchBridge.consumeRequest(request) && request.valid) {
                add_new_style(request.baseName, request.filename, request.center, request.lambda, request.radius, request.falloff, request.primitiveScale);
            }
            return true;
        }
        return false;
    };



    // default mode: keep drawing the current mesh
    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &)->bool
    {
        ImGuiIO& io = ImGui::GetIO();
        bool deletePressed = false;

        // Newer ImGui key routing (backend-mapped keys)
        deletePressed = deletePressed || ImGui::IsKeyPressed(ImGuiKey_Backspace);
        deletePressed = deletePressed || ImGui::IsKeyPressed(ImGuiKey_Delete);

        // Legacy/raw fallback for GLFW key codes.
        bool deleteDownRaw = false;
        if (259 >= 0 && 259 < IM_ARRAYSIZE(io.KeysDown)) // GLFW_KEY_BACKSPACE
        {
            deleteDownRaw = deleteDownRaw || io.KeysDown[259];
        }
        if (261 >= 0 && 261 < IM_ARRAYSIZE(io.KeysDown)) // GLFW_KEY_DELETE
        {
            deleteDownRaw = deleteDownRaw || io.KeysDown[261];
        }

        static bool deleteWasDownRaw = false;
        const bool deleteEdgeRaw = deleteDownRaw && !deleteWasDownRaw;
        deleteWasDownRaw = deleteDownRaw;

        if ((deletePressed || deleteEdgeRaw) && !io.WantTextInput)
        {
            if (delete_selected_style())
            {
                draw();
            }
        }

        draw();
        sketchBridge.drawOverlay();
        if(viewer.core().is_animating &&!state.place_constraints)
            //draw();
        return false;
    };

    // initialize the scene
    {
        viewer.data().clear();
        viewer.data().show_lines = false;
        viewer.core().is_animating = true;
        viewer.data().face_based = true;
        Vector4f backColor;
        backColor << 208/255., 237/255., 227/255., 1.;
        viewer.core().background_color = backColor;

        // avec formes initialisées mais à enlever à la fin ?
        for (auto& tStyle : data.styles_list) {
                // tStyle.viewer_id = viewer.append_mesh(); 
                // MatrixXd primitive_V = (tStyle.sV * tStyle.radius).rowwise() + tStyle.center;
                // viewer.data(tStyle.viewer_id).set_mesh(primitive_V, tStyle.sF);
                // viewer.data(tStyle.viewer_id).set_colors(Eigen::RowVector4d(1.0, 0.8, 0.2, 0.4)); 
                // viewer.data(tStyle.viewer_id).show_lines = true; // Optionnel: afficher les arêtes de l'octaèdre pour mieux le voir
                // viewer.data(tStyle.viewer_id).show_faces = tStyle.is_visible;

                render_style(tStyle);
        }


        draw();
        viewer.launch();
    }
};
