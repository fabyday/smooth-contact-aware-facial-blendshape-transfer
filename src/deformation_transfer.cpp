#include "deformation_trasnfer.h"
template<typename T>
void DeformationTransfer<T>::process_neighbor()
{

	std::shared_ptr<Mesh<T>> src_mesh_ptr = source_->get_ref_mesh();
	Face2Faces;
	Eigen::MatrixXi face = src_mesh_ptr->get_face().rows();
	// edge -> face
	// vertex -> face 
	for (int row_idx = 0; row_idx < face.rows(); row_idx++) {

		f1 = face.row(row_idx)[0]; f2 = face.row(row_idx)[1]; f3 = face.row(row_idx)[2];
		e2f_.edges_[(f1, f2)].push_back(row_idx);
		e2f_.edges_[(f2, f3)].push_back(row_idx);
		e2f_.edges_[(f3, f1)].push_back(row_idx);
		v2f_.vertidx_[f1].push_back(row_idx);
		v2f_.vertidx_[f2].push_back(row_idx);
		v2f_.vertidx_[f3].push_back(row_idx);
	}

	// face -> faces
	for (auto iter = e2f_.edges_.begin(); iter != e2f_.edges_.end(); iter++) {
		std::vector<int>& val = iter->second;
		for (auto iter_f = val.begin(); iter_f != val.end(); iter_f++) {
			int fidx = *iter_f;
			std::copy_if(val.begin(), val.end(),
				std::back_inserter(f2f_.faceidx_[fidx]),
				[fidx](int inserted_fidx) {return fidx != inserted_fidx; });
		}
	}

}

template<typename T>
void add_sparse(Sparse& A, TriangleDeformationGradient<T>& tdg) {
	Eigen::Vector4i idx = tdg.get_ind();
	ROWMAT(T) mat = tdg.get_mat();
	const int row_size = static_cast<int>(A.rows() / 3);
	const int col_size = static_cast<int>(A.cols() / 3);
	const int tri_num = tdg.tri_num_;
	for (int ax = 0; ax < 3; ax++) {
		for (int i = 0; i < 3; i++) {
			A(row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(0, 0)) += (-invV_adj(i, 0) - invV_adj(i, 1) - invV_adj(i, 2));
			A(row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(1, 0)) += (invV_adj(i, 0));
			A(row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(2, 0)) += (invV_adj(i, 1));
			A(row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(3, 0)) += (invV_adj(i, 2);
		}
	}
	
}
template<typename T>
void sub_sparse(Sparse& A, TriangleDeformationGradient<T>& tdg) {
	Eigen::Vector4i idx = tdg.get_ind();
	ROWMAT(T) mat = tdg.get_mat();
	const int row_size = static_cast<int>(A.rows()/3);
	const int col_size = static_cast<int>(A.cols() / 3);
	const int tri_num = tdg.tri_num_;
	for (int ax = 0; ax < 3; ax++) {
		for (int i = 0; i < 3; i++) {
			A( row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(0, 0)) -= (-invV_adj(i, 0) - invV_adj(i, 1) - invV_adj(i, 2));
			A( row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(1, 0)) -= (invV_adj(i, 0));
			A( row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(2, 0)) -= (invV_adj(i, 1));
			A( row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(3, 0)) -= (invV_adj(i, 2);
		}
	}
}


template<typename T>
inline Sparse DeformationTransfer<T>::produce_smoothness()
{
	const int row_size =  f2f_.faceidx_.size();
	const int col_size =  v2f_.vertidx_.size();
	Sparse G(3*row_size, 3*col_size);
	G.setZero();
	Eigen::MatrixXi& face = source_->get_ref_mesh().get_face();
	//Eigen::MatrixXd&source_->get_ref_mesh().get_verts();

	for (int i = 0; i < f2f_.faceidx_.size(); i++) {
		TriangleDeformationGradient<T> tdg_i = source_->get_inv_matrix(i); // Triangle i 
		add_sparse(G, tdg_i);
		
		for (auto iter = f2f_.faceidx_[i].begin(); iter != f2f_.faceidx_[i].end(); iter++) {
			const int adj_face_idx = *iter;
			TriangleDeformationGradient<T>& tdg_adj = source_->get_inv_matrix(adj_face_idx);
			sub_sparse(G, tdg_adj);
		}
	}
	return G;
}

template<typename T>
Eigen::SParse DeformationTransfer<T>::produce_identity()
{
	const int row_size = f2f_.faceidx_.size();
	const int col_size = v2f_.vertidx_.size();
	Sparse G(3 * (3 * row_size), 3 * col_size);

	Eigen::MatrixXi& face = source_->get_ref_mesh().get_face();

	for (int i = 0; i < f2f_.faceidx_.size(); i++) {
		ROWMAT(T) invV_i = source_->get_inv_matrix(i); // Triangle i 

		for (auto iter = f2f_.faceidx_[i].begin(); iter != f2f_.faceidx_[i].end(); iter++) {
			const int adj_face_idx = *iter;
			TriangleDeformationGradient tdg = source_->get_inv_matrix(adj_face_idx);


			ROWMAT(T)& invV_adj = tdg.get_mat();
			Eigen::Vector4i invV_idx = tgd.get_idx();
			int axis = 3;
			for (int a = 0; a < axis; a++) {
				for (int row = 0; row < 3; row++) {
					G(3 * row_size * a + i * 3 + row, 3 * col_size * a + invV_idx(0, 0)) = -(-invV_adj(row, 0) - invV_adj(row, 1) - invV_adj(row, 2));
					G(3 * row_size * a + i * 3 + row, 3 * col_size * a + invV_idx(1, 0)) = -(invV_adj(row, 0));
					G(3 * row_size * a + i * 3 + row, 3 * col_size * a + invV_idx(2, 0)) = -(invV_adj(row, 1));
					G(3 * row_size * a + i * 3 + row, 3 * col_size * a + invV_idx(3, 0)) = -(invV_adj(row, 2);
				}
			}
		}
	}

	return G;


	
}

template<typename T>
Eigen::Matrix<T, -1, -1, Eigen::RowMajor> DeformationTransfer<T>::produce_closest()
{
	const int row_size = f2f_.faceidx_.size();
	const int col_size = v2f_.vertidx_.size();
	Sparse G( 3*(3*row_size), 3*col_size );


	

	return G;
}

template<typename T>
void DeformationTransfer<T>::add_deformation_gradient_data(DeformationGradient<T, struct DGTriangle4<T>>& dg_source, DeformationGradient<T, struct DGTriangle4<T>>& dg_target)
{
	source_ = &dg_source;
	target_ = &dg_target;
}

template<typename T>
void DeformationTransfer<T>::add_constraint(ROWMAT(T) additional_mat)
{

}

template<typename T>
void DeformationTransfer<T>::process_correspondence()
{
	using res_mat = Eigen::Matrix<T, -1, -1, Eigen::RowMajor>;
	process_neighbor();
	

	res_mat s_term = produce_smoothness();
	res_mat i_term = produce_identity();
	res_mat c_term = produce_closest();

}


