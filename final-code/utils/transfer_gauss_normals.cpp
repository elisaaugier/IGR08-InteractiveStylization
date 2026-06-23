#include "transfer_gauss_normals.h"

void transfer_gauss_normals(
  const Eigen::MatrixXd & gaussPos,
	const Eigen::MatrixXd & tarN,
	const Eigen::MatrixXd & N,
	Eigen::MatrixXd & N_snap)
{
    // snapping normals (N) to the closest point on the Gauss map
    N_snap.resize(N.rows(), N.cols());
    for (int ii=0; ii<N.rows(); ii++)
    {
        // find the closest normal
        int minIdx = -1;
        double minDist = 1e10;
        for (int jj=0; jj<gaussPos.rows(); jj++)
        {
          double dist = (gaussPos.row(jj) - N.row(ii)).norm();
          if (dist < minDist)
          {
            minDist = dist;
            minIdx = jj;
          }
        }
        N_snap.row(ii) = tarN.row(minIdx);
    }
}

void multi_transfer_gauss_normals(
    const Eigen::MatrixXd & UN, 
    const Eigen::MatrixXd & U, 
    const Eigen::MatrixXi & F, 
    int nEle, 
    normal_driven_data & data)
{
    using namespace Eigen;
    
    MatrixXd accumulated_N = MatrixXd::Zero(nEle, 3);
    VectorXd total_weights = VectorXd::Zero(nEle);
    data.lambdaVec.setZero(nEle); 
    
    for (const auto& prim : data.styles_list)
    {
        MatrixXd local_gauss_snap;
        transfer_gauss_normals(prim.gaussPos, prim.desiredN, UN, local_gauss_snap);
        
        for (int i = 0; i < nEle; i++)
        {
            RowVector3d pos_i;
            if (nEle == F.rows())
                pos_i = (U.row(F(i,0)) + U.row(F(i,1)) + U.row(F(i,2))) / 3.0;
            else if (nEle == U.rows())
                pos_i = U.row(i);
            
            double dist = (pos_i - prim.center).norm();
            
            if (dist < prim.radius)
            {
                double weight = pow(1.0 - (dist / prim.radius), prim.falloff);
                
                accumulated_N.row(i) += local_gauss_snap.row(i) * weight;
                data.lambdaVec(i) += prim.lambda * weight;
                total_weights(i) += weight;
            }
        }
    } 
    
    // Normalisation finale (identique à multi_snap_normals)
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