#include "deformation_trasnfer.h"
template<typename T>
inline Eigen::Matrix<T, -1, -1, Eigen::RowMajor> DeformationTransfer<T>::produce_smoothness()
{
	return Eigen::Matrix<T, -1, -1, Eigen::RowMajor>();
}

template<typename T>
Eigen::Matrix<T, -1, -1, Eigen::RowMajor> DeformationTransfer<T>::produce_identity()
{
	return Eigen::Matrix<T, -1, -1, Eigen::RowMajor>();
}

template<typename T>
Eigen::Matrix<T, -1, -1, Eigen::RowMajor> DeformationTransfer<T>::produce_closest()
{
	return Eigen::Matrix<T, -1, -1, Eigen::RowMajor>();
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


	std::shared_ptr<Mesh<T>> src_mesh_ptr = source_->get_ref_mesh();
	Face2Faces ;
	Eigen::MatrixXi face = src_mesh_ptr->get_face().rows();
	// edge -> face
	for (int row_idx = 0;  row_idx < face.rows(); row_idx++) {
	
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
	// vertex -> face 
	for ()
}


