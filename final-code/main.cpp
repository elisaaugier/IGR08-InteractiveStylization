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

#include <ctime>
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
    int numCV = 0;
    Eigen::MatrixXi CV_row_col; 

    int hovered_constr_idx = -1;
    int selected_tstyle_idx = -1;

    bool drawing_mode = false;
    bool is_drawing = false;
    std::vector<Eigen::Vector2f> stroke_points;
} state;

// to run the code, type "./normalDriven_bin [input obj] [style obj]" (e.g., ./normalDriven_bin bunny.obj cube.obj)
int main(int argc, char *argv[])
{
	using namespace Eigen;
	using namespace std;

    // load mesh
	MatrixXd V, U, VO; //, desiredNormals;
	MatrixXi F;
    string meshName; //, styleMeshName;
    bool useFaceArap = false;
    std::vector<target_style> styles_list;

    // ~~~~~~~~~~~~~~~~~~~~~~ O N E STYLE ~~~~~~~~~~~~~~~~~~~~~~

	// {   
    //     meshName = argv[1];
    //     styleMeshName = argv[2];

    //     // get desired normals
	// 	string styleFile = MESH_PATH + styleMeshName;
    //     MatrixXd sV,sN;
    //     MatrixXi sF,useless1,useless2; 
    //     igl::read_triangle_mesh(styleFile, sV, sF);
    //     igl::per_face_normals(sV,sF,sN);
    //     igl::unique_rows(sN, desiredNormals, useless1, useless2);
    //     cout << "desiredNormals.size: " << desiredNormals.rows() << ", " << desiredNormals.cols() << endl;

    //     // read mesh files
	// 	string file = MESH_PATH + meshName;
	// 	igl::read_triangle_mesh(file, V, F);
    //     U = V;
    //     VO = V;

        

    //     // string output name 
    //     string outputStr = styleMeshName;
    //     outputStr.erase(outputStr.end()-4, outputStr.end());
    //     cout << outputStr << endl;
	// }

    // ~~~~~~~~~~~~~~~~~~~~~~ MULTI-STYLES ~~~~~~~~~~~~~~~~~~~~~~

    meshName = argv[1];

    // read mesh files 

    string file = MESH_PATH + meshName;
    igl::read_triangle_mesh(file, V, F);
    V = V.rowwise() - V.colwise().minCoeff();
    V /= V.maxCoeff();
    U = V;
    VO = V;


    // get desired normals

    // auto load_style_normals = [] (const std::string& filename) -> MatrixXd {
    //     string styleFile = MESH_PATH + filename;
    //     MatrixXd sV, sN, desiredNormals;
    //     MatrixXi sF,useless1,useless2; 
    //     igl::read_triangle_mesh(styleFile, sV, sF);
    //     igl::per_face_normals(sV,sF,sN);
    //     igl::unique_rows(sN, desiredNormals, useless1, useless2);
    //     return desiredNormals;
    // };


    // a supprimer à la fin car utile que si formes dès l'initialiation ?
    auto load_geometric_style = [] (const std::string& name, const std::string& filename, RowVector3d center, double lambda, double radius, double falloff)
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

        igl::read_triangle_mesh(styleFile, tStyle.sV,tStyle.sF);
        igl::per_face_normals(tStyle.sV, tStyle.sF, sN);
        igl::unique_rows(sN, tStyle.desiredN, useless1, useless2);

        return tStyle;
    };


    // Primitive 1 
    // geometric_style prim1;
    // prim1.name = "cube";
    // prim1.desiredN = load_style_normals("octahedron.obj");
    // prim1.lambda = 1.0;
    // prim1.center = RowVector3d(0.0, 0.5, 0.65); 
    // prim1.radius = 0.8;
    // prim1.falloff = 2.0;
    // primitives_list.push_back(prim1);

    target_style style1 = load_geometric_style(
        "cone", "cone.obj", RowVector3d(0.0, 0.5, 0.65), 1.0, 0.5, 2.0
    );


    target_style style2 = load_geometric_style(
        "cube", "cube.obj", RowVector3d(0.0, -0.5, 0.5), 1.0, 0.5, 2.0
    );

    target_style style3 = load_geometric_style(
        "sphere", "sphere2.obj", RowVector3d(0.0, 1.0, 1.0), 1.0, 0.5, 2.0
    );

    styles_list.push_back(style2);
    styles_list.push_back(style3);
    //styles_list.push_back(style1);

    cout << "Loaded " << styles_list.size() << " geometric styles." << endl;
    cout << "Target geometric styles:" << endl;
    for (auto& prim : styles_list) {
        cout << prim.name << endl;
    }
    


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

    // prepare data for stylization
    normal_driven_data data;
    data.V0 = VO;
    // data.lambda = 1e0;
    data.styles_list = styles_list;

    data.SDF.addPrimitive(std::make_unique<Box>(RowVector3d(0.5, 0.5, 0.5), RowVector3d(0.0, -0.5, 0.5)));
    //data.SDF.addPrimitive(std::make_unique<Box>(RowVector3d(0.7 ,0.7 ,0.7), RowVector3d(0.0, -1.5, 1.5)));
    //data.SDF.addPrimitive(std::make_unique<Sphere>(0.7, Eigen::RowVector3d(0.0, 0.0, 0.0)));
    data.SDF.addPrimitive(std::make_unique<Sphere>(0.4, RowVector3d(0.0, 1.0, 1.0)));

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
    ImGuizmo::OPERATION current_operation = ImGuizmo::TRANSLATE;

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

    auto render_style = [&] (target_style& tStyle) {
        tStyle.viewer_id = viewer.append_mesh();
        MatrixXd primitive_V = (tStyle.sV * tStyle.radius).rowwise() + tStyle.center;
        viewer.data(tStyle.viewer_id).set_mesh(primitive_V, tStyle.sF);
        viewer.data(tStyle.viewer_id).set_colors(Eigen::RowVector4d(1.0, 1.0, 1.0, 0.4));
        viewer.data(tStyle.viewer_id).show_lines = tStyle.is_visible; // display edges
        viewer.data(tStyle.viewer_id).show_faces = tStyle.is_visible;
    };

    auto update_style = [&] (target_style& tStyle) {
        viewer.data(tStyle.viewer_id).show_lines = tStyle.is_visible; // display edges
        viewer.data(tStyle.viewer_id).show_faces = tStyle.is_visible;
    };

    auto add_new_style = [&] (const std::string& base_name, const std::string& filename) {

        // en considérant pr le moment que l'utilisateur peut pas renommer les noms des formes
        int count = 0;
        for (const auto& s : data.styles_list) {
            if (s.name.size() >= base_name.size() && s.name.substr(0, base_name.size()) == base_name) {
                count++;
            } 
        }
        std::string unique_name = base_name + "_" + std::to_string(count);

        target_style new_style = load_geometric_style(
            unique_name, filename, RowVector3d(0.0, 0.0, 0.0), 1., 1., 2.
        );
        render_style(new_style);
        data.styles_list.push_back(new_style);
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
                        data.SDF.getMesh(Vsdf, Fsdf);
                        viewer.data(sdfId).set_mesh(Vsdf, Fsdf);
                        viewer.data(sdfId).compute_normals();
                        
                        if (current_operation == ImGuizmo::SCALE) {
                            // Update tracked scale and apply to SDF
                            gizmo_scale = scale;
                            data.SDF.setScale(i, scale);
                            data.SDF.getMesh(Vsdf, Fsdf);
                            viewer.data(sdfId).set_mesh(Vsdf, Fsdf);
                            viewer.data(sdfId).compute_normals();
                        }

                        if (tStyle.viewer_id != -1) {
                            Eigen::MatrixXd new_V = (tStyle.sV * tStyle.radius).rowwise() + tStyle.center;
                            viewer.data(tStyle.viewer_id).set_vertices(new_V);
                            viewer.data(tStyle.viewer_id).clear();
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
            ImGui::DragScalar("lambda", ImGuiDataType_Double, &data.styles_list[0].lambda, 5e-1, 0, 0, "%.1e");
		    ImGui::DragScalar("gamma", ImGuiDataType_Double, &data.gamma, 5e-2, 0, 0, "%.1e");
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
        ImGui::End();


        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // ~~~~~~~~~~~~~~  W I N D O W   2  ~~~~~~~~~~~~~~
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        {
            ImGui::SetNextWindowPos(ImVec2(0.f * menu.menu_scaling(), 520), ImGuiCond_Appearing);
            // ImGui::SetNextWindowPos(ImVec2(viewer.core().viewport(2) - 320, 10), ImGuiCond_FirstUseEver);
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

                    // Sliders de Position
                    float center_f[3] = { (float)tStyle.center(0), (float)tStyle.center(1), (float)tStyle.center(2) };
                    if (ImGui::DragFloat3("Position", center_f, 0.01f)) {
                        tStyle.center << center_f[0], center_f[1], center_f[2];
                        changed = true;
                    }

                    // Sliders de Paramètres
                    if (ImGui::DragScalar("Rayon", ImGuiDataType_Double, &tStyle.radius, 0.01f, 0, 0, "%.2f")) changed = true;
                    ImGui::DragScalar("Lambda (Force)", ImGuiDataType_Double, &tStyle.lambda, 0.05f, 0, 0, "%.2f");
                    ImGui::DragScalar("Falloff", ImGuiDataType_Double, &tStyle.falloff, 0.05f, 0, 0, "%.2f");

                    // Mise à jour visuelle du mesh transparent si on bouge un slider (position/rayon)
                    if (changed && tStyle.is_visible && show_all_targets && tStyle.viewer_id != -1) {
                        MatrixXd new_V = (tStyle.sV * tStyle.radius).rowwise() + tStyle.center;
                        viewer.data(tStyle.viewer_id).set_vertices(new_V);
                    }

                    ImGui::TreePop(); // Fin de l'accordéon
                }

                ImGui::PopID();

            }
            
        }
        ImGui::End();

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // ~~~~~~~~~~~~~~  W I N D O W   3  ~~~~~~~~~~~~~~
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        {
            ImGui::SetNextWindowPos(ImVec2(0.f * menu.menu_scaling(), 1340), ImGuiCond_Appearing);
            // ImGui::SetNextWindowPos(ImVec2(viewer.core().viewport(2) - 320, 420), ImGuiCond_FirstUseEver);
			ImGui::SetNextWindowSize(ImVec2(300, 400), ImGuiCond_FirstUseEver);
			ImGui::Begin(
				"Add a Target-Style", nullptr,
				ImGuiWindowFlags_NoSavedSettings
			);
            ImGui::Text("Add a new target style");
            ImGui::Separator();
            ImGui::Checkbox("activate drawing mode", &state.drawing_mode);
            ImGui::Spacing();
            if (ImGui::Button("+ Ajouter un Cube", ImVec2(-1, 0))) {
                add_new_style("cube", "cube.obj");
            }

            if (state.drawing_mode && state.stroke_points.size() > 1) {
                ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
                for (size_t i = 0; i < state.stroke_points.size() - 1; ++i) {
                    draw_list->AddLine(
                        ImVec2(state.stroke_points[i].x(), state.stroke_points[i].y()),
                        ImVec2(state.stroke_points[i+1].x(), state.stroke_points[i+1].y()),
                        IM_COL32(255, 0, 0, 255), 3.0f 
                    );
                }
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

        if(state.place_constraints)
        // IDLE mode (NO deformation)
        {
            viewer.data_list[0].set_mesh(V, F);
            viewer.data_list[0].set_colors(gray);
            set_contraints_colors();
        }
        else
        // OPTIMISATION mode (deformation)
        { 
            normal_driven_prim_single_iteration(F, U, data);
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
                for(auto& p : styles_list) p.lambda += 0.02;
                break;
            }
            case '<':
            case ',':
            {
                for(auto& p : styles_list) p.lambda -= 0.02;
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
            case ' ':
            {
                state.place_constraints = !state.place_constraints; // switch mode
                if (state.place_constraints)
                {
                    resetCV();
                    draw();
                }
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

        // select a target style mesh
        if (!state.drawing_mode && button==0) 
        {
            if (state.selected_tstyle_idx != -1) {
                auto& selected_tstyle = data.styles_list[state.selected_tstyle_idx];
                viewer.data(selected_tstyle.viewer_id).set_colors(Eigen::RowVector4d(1.0, 1.0, 1.0, 0.4));
            }
            int closest_tstyle_idx = -1;
            double min_dist = 1e20;

            // ray casting
            for (int i=0; i<data.styles_list.size(); i++) {
                auto& tStyle = data.styles_list[i];
                if (!tStyle.is_visible || tStyle.viewer_id == -1) continue;
                
                Eigen::MatrixXd tStyle_V = (tStyle.sV * tStyle.radius).rowwise() + tStyle.center;
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
                viewer.data(selected_tstyle.viewer_id).set_colors(Eigen::RowVector4d(0.0, 0.0, 1.0, 0.4));
                return true;
            }
            else {
                state.selected_tstyle_idx = -1;
            }
        }

        // add/remove constraints (DISABLED by default - hacky fix to prevent blocking primitive selection)
        if (!state.drawing_mode && state.place_constraints && button == 0 && false)
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

        // draw in 2D
        if (state.drawing_mode && button==0) 
        {
            state.is_drawing = true;
            state.stroke_points.clear();
            state.stroke_points.push_back(Eigen::Vector2f(x, viewer.current_mouse_y));
            return true;
        }

        return false;
    };

    viewer.callback_mouse_move = [&] (igl::opengl::glfw::Viewer&, int mouse_x, int mouse_y) -> bool 
    {
        double x = viewer.current_mouse_x;
        double y = viewer.core().viewport(3) - viewer.current_mouse_y;

        if (state.place_constraints && !state.drawing_mode) {

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

        if (state.is_drawing) {
            state.stroke_points.push_back(Eigen::Vector2f(x, viewer.current_mouse_y));
            return true;
        }
        return false;
    };


    viewer.callback_mouse_up = [&](igl::opengl::glfw::Viewer&, int button, int modifier)->bool
    {

        // test pour dessin 2D --> forme 3D (là c'est que pour reconnaissance d'une sphère)
        if (state.is_drawing && button == 0) {
            state.is_drawing = false;
            
            if (state.stroke_points.size() > 5) {
                
                float minX = 1e6, maxX = -1e6, minY = 1e6, maxY = -1e6;
                for (auto& p : state.stroke_points) {
                    if (p.x() < minX) minX = p.x(); if (p.x() > maxX) maxX = p.x();
                    if (p.y() < minY) minY = p.y(); if (p.y() > maxY) maxY = p.y();
                }
                
                float centerX = (minX + maxX) / 2.0f;
                float centerY = (minY + maxY) / 2.0f;
                
                float y_bottom = viewer.core().viewport(3) - centerY;
                
                // ray casting
                int fid; Eigen::Vector3f bary;
                if(igl::unproject_onto_mesh(Eigen::Vector2f(centerX, y_bottom), viewer.core().view, viewer.core().proj, viewer.core().viewport, V, F, fid, bary)) 
                {
                    Eigen::RowVector3d pt3D = 
                        bary(0) * V.row(F(fid, 0)) + 
                        bary(1) * V.row(F(fid, 1)) + 
                        bary(2) * V.row(F(fid, 2));
                        
                    double pixel_width = (maxX - minX);
                    double radius3D = std::max(0.1, pixel_width / 1000.0); // Ratio très basique pour tester
                    
                    target_style new_cube = load_geometric_style(
                        "Dessin_Cube_" + std::to_string(data.styles_list.size()), 
                        "cube.obj", 
                        pt3D,       
                        1.0,      
                        radius3D,  
                        2.0         
                    );
                    
                    new_cube.viewer_id = viewer.append_mesh();
                    if (new_cube.sV.rows() > 0) {
                        MatrixXd primitive_V = (new_cube.sV * new_cube.radius).rowwise() + new_cube.center;
                        viewer.data(new_cube.viewer_id).set_mesh(primitive_V, new_cube.sF);
                        viewer.data(new_cube.viewer_id).set_colors(Eigen::RowVector4d(1.0, 0.2, 0.2, 0.5)); // Cube rouge transparent
                        viewer.data(new_cube.viewer_id).show_lines = true;
                    }
                    
                    data.styles_list.push_back(new_cube);
                    std::cout << "Cube magique cree au point XYZ: " << pt3D << std::endl;
                } else {
                    std::cout << "Echec : vous avez dessine dans le vide, hors du maillage !" << std::endl;
                }
            }
            
            state.stroke_points.clear(); 
            return true;
        }
        return false;
    };



    // default mode: keep drawing the current mesh
    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &)->bool
    {
        draw();
        if(viewer.core().is_animating &&!state.place_constraints)
            //draw();
        return false;
    };

    // initialize the scene
    {
        viewer.data().set_mesh(V,F);
        viewer.data().show_lines = false;
        viewer.core().is_animating = true;
        viewer.data().face_based = true;
        Vector4f backColor;
        backColor << 208/255., 237/255., 227/255., 1.;
        viewer.core().background_color = backColor;

        // avec formes initialisées mais à enlever à la fin ?
        for (auto& tStyle : data.styles_list) {
                // // On crée un calque secondaire pour l'octaèdre
                // tStyle.viewer_id = viewer.append_mesh(); 
                
                // // On calcule sa position (X,Y,Z) et sa taille (Radius)
                // MatrixXd primitive_V = (tStyle.sV * tStyle.radius).rowwise() + tStyle.center;
                
                // // On envoie le maillage de l'OCTAÈDRE sur ce calque
                // viewer.data(tStyle.viewer_id).set_mesh(primitive_V, tStyle.sF);
                
                // // On le peint en Jaune Transparent (R=1.0, G=0.8, B=0.2, Alpha=0.4)
                // viewer.data(tStyle.viewer_id).set_colors(Eigen::RowVector4d(1.0, 0.8, 0.2, 0.4)); 
                
                // viewer.data(tStyle.viewer_id).show_lines = true; // Optionnel: afficher les arêtes de l'octaèdre pour mieux le voir
                // viewer.data(tStyle.viewer_id).show_faces = tStyle.is_visible;

                render_style(tStyle);
        }


        draw();
        viewer.launch();
    }
};
