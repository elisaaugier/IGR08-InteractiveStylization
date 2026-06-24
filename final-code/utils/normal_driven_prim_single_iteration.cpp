#include "normal_driven_prim_single_iteration.h"


/*
void normal_driven_prim_single_iteration(
	const Eigen::MatrixXd & V,
	const Eigen::MatrixXi & F,
    Eigen::MatrixXd & U,
    normal_driven_data & data)
{
    using namespace Eigen;
    using namespace std;

    int nEle = data.A.size(); 
    int nV = U.rows();

    if (data.tarN.rows() != nEle)
    {
        if (data.desiredN.rows() == 0)
        {
            cerr << "desired normal is empty" << endl;
        }
        MatrixXd UN;
        if (nEle == F.rows())
            igl::per_face_normals(U,F,UN);
        else if (nEle == U.rows())
            igl::per_vertex_normals(U,F,UN);
        snap_normals(data.desiredN, UN, data.tarN); 
    }
    
    
    // local step
    MatrixXd RAll;
    {
        PROFC_NODE("local step")
        if (data.useScale)
        {   
            // cout << "use scale \n";
            fit_rotations_normal_scale(F, U, RAll, data);
        }
        else
        {
            // cout << "not use scale \n";
            fit_rotations_normal(F, U, RAll, data);
        }
    }

    // global step
    MatrixXd Upre = U;
    {
        PROFC_NODE("global step")
        MatrixXd RAllT = RAll.transpose();
        Map<const VectorXd> Rcol(RAllT.data(), RAllT.size());
        VectorXd Bcol = data.K.transpose() * Rcol;
        for(int dim=0; dim<U.cols(); dim++)
        {
            VectorXd Uc,Bc,bcc;
            Bc = Bcol.block(dim*nV,0,nV,1);
            bcc = data.bc.col(dim);
            min_quad_with_fixed_solve(
                data.solver_data,Bc,bcc,VectorXd(),Uc);
            U.col(dim) = Uc;
        }
    }

    // recompute desired normal
    {
        PROFC_NODE("snap normals")
        MatrixXd UN;
        if (nEle == F.rows())
            igl::per_face_normals(U,F,UN);
        else if (nEle == U.rows())
            igl::per_vertex_normals(U,F,UN);
        snap_normals(data.desiredN, UN, data.tarN); 
    }

    // print optimization date
    data.reldV = (U-Upre).cwiseAbs().maxCoeff() / (U-V).cwiseAbs().maxCoeff();
    // cout << "reldV:" << scientific << data.reldV << endl;
}
*/


// version multi-cibles 

void normal_driven_prim_single_iteration(
	const Eigen::MatrixXi & F,
    Eigen::MatrixXd & U,
    normal_driven_data & data)
{
    using namespace Eigen;
    using namespace std;

    int nEle = data.A.size();
    int nV = U.rows();

    if (data.tarN.rows() != nEle)
    {
        MatrixXd UN;
        if (nEle == F.rows())
            igl::per_face_normals(U,F,UN);
        else if (nEle == U.rows())
            igl::per_vertex_normals(U,F,UN);
        multi_snap_normals(UN, U, F, nEle, data);
    }
    
    
    // local step
    MatrixXd RAll;
    {
        PROFC_NODE("local step")
        if (data.useScale)
        {   
            // cout << "use scale \n";
            fit_rotations_normal_scale(F, U, RAll, data);
        }
        else
        {
            // cout << "not use scale \n";
            fit_rotations_normal(F, U, RAll, data);
        }
    }

    // global step
    MatrixXd Upre = U;
    {
        PROFC_NODE("global step")
        MatrixXd RAllT = RAll.transpose();
        Map<const VectorXd> Rcol(RAllT.data(), RAllT.size());
        VectorXd Bcol = data.K.transpose() * Rcol;
        Eigen::MatrixXd Vsdf;
        Vsdf.resize(U.rows(), U.cols());
        data.SDF.computeSdfVector3(U, Vsdf);
        MatrixXd P = Vsdf;
        VectorXd VsdfCol;
        VsdfCol.resize(U.rows() * 3, 1);
        for (int i=0; i<U.rows(); i++) {
            VsdfCol(i) = P(i, 0);
            VsdfCol(i + 1 * U.rows()) = P(i, 1);
            VsdfCol(i + 2 * U.rows()) = P(i, 2);
        }
        VectorXd PP;
        PP.resize(U.rows() * 3, 1);
        for (int i=0; i<U.rows(); i++) {
            PP(i) = P(i, 0);
            PP(i + 1 * U.rows()) = U(i, 1);
            PP(i + 2 * U.rows()) = U(i, 2);
        }

        for(int dim=0; dim<U.cols(); dim++)
        {
            VectorXd Uc,Bc,bcc, Vc, Pc;
            Bc = Bcol.block(dim*nV,0,nV,1);
            Vc = VsdfCol.block(dim*U.rows(),0,U.rows(),1);
            Pc = PP.block(dim*U.rows(),0,U.rows(),1);
            bcc = data.bc.col(dim);
            //min_quad_with_fixed_solve(
            //    data.solver_data,Bc,bcc,VectorXd(),Uc);
            min_quad_with_fixed_solve(
                data.solver_data,Bc + data.gamma * Vc,VectorXd(),VectorXd(),Uc);
            U.col(dim) = Uc;
        }
    }
    

    // recompute desired normal
    {
        PROFC_NODE("snap normals")
        MatrixXd UN;
        if (nEle == F.rows()) // Face ARAP (nb d'éléménts = nb de lignes de F, matrice des faces)
            igl::per_face_normals(U,F,UN);
        else if (nEle == U.rows()) // Vertex ARAP (nb d'éléménts = nb de lignes de U, matrice des sommets)
            igl::per_vertex_normals(U,F,UN);
        multi_snap_normals(UN, U, F, nEle, data);
    }

    // print optimization date
    // data.reldV = (U-Upre).cwiseAbs().maxCoeff() / (U-V).cwiseAbs().maxCoeff();
    // cout << "reldV:" << scientific << reldV << ", obj:" << data.objVal << endl;
}