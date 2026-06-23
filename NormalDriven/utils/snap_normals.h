#ifndef SNAP_NORMALS_H
#define SNAP_NORMALS_H

#include <iostream>
#include <Eigen/Core>
#include <normal_driven_data.h>

// Inputs:
//   tarN     a set of target normals
//   N        a set of input normals
// Outputs: 
//   N_snap   snapped normals
void snap_normals(
	const Eigen::MatrixXd & tarN,
	const Eigen::MatrixXd & N,
	Eigen::MatrixXd & N_snap);

void multi_snap_normals(
    const Eigen::MatrixXd & UN, 
    const Eigen::MatrixXd & U, 
    const Eigen::MatrixXi & F, 
    int nEle, 
    normal_driven_data & data);

#endif
