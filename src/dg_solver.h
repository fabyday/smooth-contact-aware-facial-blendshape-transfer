#pragma once
#include <Eigen/Core>

typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> MatrixRxi;
typedef Eigen::Matrix<int, 3, 3, Eigen::RowMajor> MatrixR3i;
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> MatrixR3d;
typedef Eigen::Matrix<double, -1, -1, Eigen::RowMajor> MatrixRxd;


//
template<typename T>
class DGSolver {
	
private:
	bool constraint_flag_;
public:
	enum ConstraintType { 
						VIRTUAL_VERTEX_FOR_TRIANGLE,  // virtual vertex constraint
						POINT_CONSTRAINT //default constraint marker
						};

	template<typename S>
	void add_deformation_gradient(DeformationGradient<T, S> dg);
	void add_constraint(ConstraintType s, std::vector<int> t);

	void reset_all();
	void reset_constraint() {
		constraint_flag_ = false;
	}
	void solve(const Eigen::Matrix<T, -1,-1, Eigen::RowMajor>&  coords);

};

