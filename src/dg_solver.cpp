#include "dg_solver.h"

template<typename T>
void 
DGSolver<T>::solve(const Eigen::Matrix<T, -1, -1, Eigen::RowMajor>& coords)
{

}

template<typename T>
void
DGSolver<T>::solve(){
	dg4_src_tgt[PURPOSE::SOURCE]->compile();
	dtrans_.add_deformation_gradient_data(*dg4_src_tgt[PURPOSE::SOURCE], *dg4_src_tgt[PURPOSE::TARGET]);
	dtrans_.add_marker(marker_idx_);
	dtrans_.compile();



	dtrans_.solve();


}

template<typename T>
void DGSolver<T>::add_marker_index(std::vector<std::tuple<int, int>>& corr_marker){
	marker_idx_ = corr_marker;
}

template<typename T>
void DGSolver<T>::add_deformation_gradient(DGSolver::PURPOSE p, DeformationGradient4& dg)
{
	dg4_src_tgt[p] = &dg;
}


template<typename T>
void DGSolver<T>::add_virtual_triangle_index(std::vector<int>& tri_index, DGSolver::GENTRI tri)
{
	int v_size = dg4_src_tgt[PURPOSE::SOURCE]->get_ref_mesh().verts_size();
	std::for_each(tri_index.begin(), tri_index.end(), [v_size](const int idx) {if (v_size >= idx || idx < 0) std::out_of_bounds() });
	if (tri == GENTRI::TRIANGLE_STRIP) {
		virtual_traingle_idx_ = tri_index;
	}
}