#pragma once
#include <Eigen/Core>


typedef Matrix<int, -1, -1, Eigen::RowMajor> MatrixRxi;
typedef Matrix<int, 3, 3, Eigen::RowMajor> MatrixR3i;
typedef Matrix<double, 3, 3, Eigen::RowMajor> MatrixR3d;
typedef Matrix<double, -1, -1, Eigen::RowMajor> MatrixRxd;


//
template<typename T>
class DGSolver {
	

public:
	enum ConstraintType { 
						VIRTUAL_VERTEX_FOR_TRIANGLE, 
						POINT_CONSTRAINT 
						};

	

	void add_deformation_gradient(DeformationGradient<T> dg);
	void add_constraint(ConstraintType s, std::vector<int> t);
	void solve(const Eigen::Matrix<T, -1,-1, Eigen::RowMajor>&  coords);

};

