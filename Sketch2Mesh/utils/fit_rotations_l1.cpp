#include "fit_rotations_l1.h"

void fit_rotations_l1(
    const Eigen::MatrixXd & V,
	Eigen::MatrixXd & U,
    Eigen::MatrixXd & RAll,
	cube_style_data & data)
{
    using namespace Eigen;
	using namespace std;

    const RowVector3d centerU = U.colwise().mean();

    data.objValVec.setZero(V.rows());
    igl::parallel_for(
        V.rows(),
        [&data, &RAll, &U, &centerU](const int ii)
        {
            VectorXd n = data.N.row(ii).transpose();
            Matrix3d R;

            // spherical target normal at vertex ii: radial direction from the current shape center
            Vector3d target = (U.row(ii) - centerU).transpose();
            if (target.norm() < 1e-12)
            {
                target = n;
            }
            else
            {
                target.normalize();
            }

            // get energy parameters
            // Note: dVn = [dV n], dUn = [dU z-u]
            MatrixXi hE = data.hEList[ii];
            MatrixXd dU(3,hE.rows()); 
            {
                MatrixXd U_hE0, U_hE1;
                igl::slice(U,hE.col(0),1,U_hE0);
                igl::slice(U,hE.col(1),1,U_hE1);
                dU = (U_hE1 - U_hE0).transpose();
            }

            // Note:
            // We use an L2 spherical alignment term: ||R*n - target||^2.
            // This contributes a linear term in Procrustes via target*n'.
            MatrixXd dV = data.dVList[ii];
            VectorXd WVec = data.WVecList[ii];
            Matrix3d Spre = dV * WVec.asDiagonal() * dU.transpose();
            const double wSphere = data.lambda * data.VA(ii);
            Matrix3d S = Spre + wSphere * (target * n.transpose());

            orthogonal_procrustes(S, R);
            RAll.block(0,3*ii,3,3) = R;

            // save objective
            double objVal = 
                0.5*((R*dV-dU)*WVec.asDiagonal()*(R*dV-dU).transpose()).trace()
                + wSphere * (R*n - target).squaredNorm();
            data.objValVec(ii) = objVal;
        }   
    ,1000);
    data.objVal = data.objValVec.sum();
}
