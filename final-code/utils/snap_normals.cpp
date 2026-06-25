#include "snap_normals.h"

void snap_normals(
	const Eigen::MatrixXd & tarN, // target normals
	const Eigen::MatrixXd & N, // input normals
	Eigen::MatrixXd & N_snap) // output normals
{
    // snapping normals to the closest normals in tarN axis
    N_snap.resize(N.rows(), N.cols());
    for (int ii=0; ii<N.rows(); ii++)
    {
        // find the closest normal
        int minIdx = -1;
        double minDist = 1e10;
        for (int jj=0; jj<tarN.rows(); jj++)
        {
          double dist = (tarN.row(jj) - N.row(ii)).norm();
          if (dist < minDist)
          {
            minDist = dist;
            minIdx = jj;
          }
        }
        N_snap.row(ii) = tarN.row(minIdx);
    }
}


void multi_snap_normals(
    const Eigen::MatrixXd & UN, 
    const Eigen::MatrixXd & U, 
    const Eigen::MatrixXi & F, 
    int nEle, 
    normal_driven_data & data)
{
    using namespace Eigen;
    using namespace std;
    
    MatrixXd accumulated_N = MatrixXd::Zero(nEle, 3);
    VectorXd total_weights = VectorXd::Zero(nEle);
    data.lambdaVec.setZero(nEle); 
    
    for (const auto& prim : data.styles_list)
    {
        MatrixXd local_snap;
        if (prim.desiredN.rows() == 0)
            cerr << "primitive " << prim.name << " : desired normal is empty" << endl;
        snap_normals(prim.desiredN, UN, local_snap);
        
        for (int i = 0; i < nEle; i++)
        {
            RowVector3d pos_i;
            if (nEle == F.rows())
                pos_i = (U.row(F(i,0)) + U.row(F(i,1)) + U.row(F(i,2))) / 3.0;
            else if (nEle == U.rows())
                pos_i = U.row(i);

            double d;
            RowVector3d grad;
            prim.prim_sdf->eval(pos_i, d, grad);
            
            // double dist = (pos_i - prim.center).norm();
            
            // if (dist < prim.radius)

            double weight = 0;

            if (d < 0.0)
            {
                // double weight = pow(1.0 - (dist / prim.radius), prim.falloff);

                RowVector3d Q = pos_i - d * grad; // projected point
                double max_internal_dist = (Q - prim.center).norm();
                if (max_internal_dist > 1e-6) {
                    weight = std::abs(d) / max_internal_dist;
                } else {
                    weight = 1.0;
                }
            }

            weight = pow(weight, prim.falloff);

            accumulated_N.row(i) += local_snap.row(i) * weight;
            data.lambdaVec(i) += prim.lambda * weight;
            total_weights(i) += weight;
        }
    } 
    
    data.tarN.resize(nEle, 3);
    for (int i = 0; i < nEle; i++)
    {
        if (total_weights(i) > 0.0)
        {
            data.lambdaVec(i) /= total_weights(i);
            RowVector3d final_normal = accumulated_N.row(i) / total_weights(i);
            final_normal.normalize();
            
            data.tarN.row(i) = final_normal;
        }
        else
        {
            data.tarN.row(i) = UN.row(i);
            data.lambdaVec(i) = 0.0; 
        }
    }
}

