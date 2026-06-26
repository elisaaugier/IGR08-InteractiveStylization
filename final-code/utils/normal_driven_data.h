#ifndef NORMAL_DRIVEN_DATA_H
#define NORMAL_DRIVEN_DATA_H

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <limits>
#include <igl/min_quad_with_fixed.h>
#include "sdf.h"

struct target_style
{
	std::string name;
	Eigen::MatrixXd desiredN;

	Eigen::MatrixXd sV;
	Eigen::MatrixXi sF;
	int viewer_id = -1; // -1: not displayed on screen
	bool is_visible = true;

	Eigen::MatrixXd gaussPos;
	Eigen::RowVector3d center;
    double radius = 1.0;
	double lambda = 1;
	double falloff = 2;
	bool is_pyramid = false;
	int pyramidSides = 5;
	double pyramidBaseRadius = 1.0;
	double pyramidHeight = 1.0;
	Eigen::Matrix3d pyramidOrientation = Eigen::Matrix3d::Identity();

	std::shared_ptr<Primitive> prim_sdf = nullptr;
	bool isBaseMesh = false;
	Eigen::Vector3d meshScale = Eigen::Vector3d::Ones();
};

struct normal_driven_data
{
	// user should tune these
	// double lambda = 1;
	Eigen::VectorXd lambdaVec;
	double developable_threshold = 0.0;
	double objVal = 0;
	double reldV = std::numeric_limits<float>::max();
	double totalArea = 0;
	bool computeEnergy = false;
	bool useScale = false;

	std::vector<target_style> styles_list;
	// std::vector<std::shared_ptr<Primitive>> primitive_list;
	Eigen::MatrixXd V0;

	std::vector<double> objHis;
	std::vector<Eigen::MatrixXi> E_N;
	std::vector<Eigen::VectorXd> Wvec;
	std::vector<Eigen::MatrixXd> dV, UHis;

	// Eigen::SparseMatrix<double> K1, K2, Q1, Q2;
	Eigen::SparseMatrix<double> K, Q, Q0;
	Eigen::MatrixXd N, tarN; //gaussPos, desiredN;
	Eigen::VectorXd objValVec, A;

	// bool useBc = false;
	Eigen::MatrixXd bc;
	Eigen::VectorXi b;
	Eigen::MatrixXi AF;

	double gamma = 0.05;
	double prevgamma = 0.05;
	double mu = 0.01;
	SDF SDF;

	igl::min_quad_with_fixed_data<double> solver_data;

	void reset()
	{
		// usually these don't need to tune
		objVal = 0;
		totalArea = 0;
		double reldV = std::numeric_limits<float>::max();

		objHis.clear();
		E_N.clear(); 
		dV.clear();
		UHis.clear();
		Wvec.clear();
		// LHS = Eigen::SparseMatrix<double>();
		Q = Eigen::SparseMatrix<double>();
		K = Eigen::SparseMatrix<double>();
		N = Eigen::MatrixXd();
		tarN = Eigen::MatrixXd();
		// gaussPos = Eigen::MatrixXd();
		AF = Eigen::MatrixXi();
		A = Eigen::VectorXd();
		objValVec = Eigen::VectorXd();

		// igl::min_quad_with_fixed_data<double> solver_data;
		// solver_data = igl::min_quad_with_fixed_data<double>();
	}
};

#endif