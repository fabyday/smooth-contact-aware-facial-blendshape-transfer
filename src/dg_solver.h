#pragma once
#include <Eigen/Core>
#include "deformation_trasnfer.h"
typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor> MatrixRxi;
typedef Eigen::Matrix<int, 3, 3, Eigen::RowMajor> MatrixR3i;
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> MatrixR3d;
typedef Eigen::Matrix<double, -1, -1, Eigen::RowMajor> MatrixRxd;


//
template<typename T>
class DGSolver {
	
public:
	using DeformationGradient4 = DeformationGradient<T, struct DGTriangle4<T>>;
	enum PURPOSE{SOURCE=0, TARGET=1};
	enum GENTRI {AUTO_SELECT=0, TRIANGLE_STRIP};
private:
	bool constraint_flag_;


	std::vector<std::tuple<int, int>> marker_idx_;
	std::vector<int> virtual_traingle_idx_; // [tri1 : [t1 t2 t3], tri2 [t4,t5,t6]]

	DeformationGradient4* dg4_src_tgt[2]; // deformation souce and target. accessed by enum PURPOSE

	DeformationTransfer<T> dtrans_;

public:
	
	void add_deformation_gradient(DGSolver::PURPOSE p, DeformationGradient4&  dg);
	void add_marker_index(std::vector<std::tuple<int, int>>& corr_marker);
	void add_virtual_triangle_index(std::vector<int>& tri_index, DGSolver::GENTRI tri = GENTRI::TRIANGLE_STRIP);


	void reset_all();
	void reset_constraint() {
		constraint_flag_ = false;
	}



	void solve(const Eigen::Matrix<T, -1,-1, Eigen::RowMajor>&  coords);
	void solve();
};

#include "dg_solver.cpp"