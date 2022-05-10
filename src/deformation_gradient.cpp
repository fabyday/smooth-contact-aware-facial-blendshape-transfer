#include "deformation_gradient.h"

template<typename T, typename S>
DeformationGradient<T, S>::DeformationGradient(int targets_num)
{}
	//: targets_(targets_num){}



template<typename T, typename S>
void DeformationGradient<T, S>::add_reference(Mesh<T>& m)
{
	ref_ = &m;
	v_size_ = m.verts_size();
	triangle_size_ = m.face_size();
	
	
}
template<typename T, typename S>
 void DeformationGradient<T,S>::add_target(Mesh<T>& m){
	 targets_.push_back(&m);
}

 template<typename T, typename S>
 void DeformationGradient<T, S>::reserve_all()
 {
	 deformation_gradients_.reserve(1 + targets_.sizse());
	 for (int i = 0; i < deformation_gradients_.size(); i++) {
		 deformation_gradients_[i].
	 }
 }

 template<typename T, typename S>
 void DeformationGradient<T, S>::add_targets(std::vector<Mesh<T>>& target_list){

	for (int i = 0; i < target_list.size(); i++) {
		 targets_.push_back(&target_list[i]); 
	 }
 }

 template<typename T, typename S>
 inline void DeformationGradient<T, S>::compile(){


	 calc_normal_vector();
	 calc_op_g_list();
	 calc_deformation_gradient();
 }

 template<typename T, typename S>
 void DeformationGradient<T, S>::calc_op_g_list() {
	 //using vec3 = Eigen::Matrix<T, 3, 1>;
	 //Mesh<T>::RowmatI& fs = ref_->get_face();
	 //ROWMAT(T)& vs = ref_->get_verts();
	 //for (int i = 0; i < fs.rows(); i++) {
		// int v1_idx = fs(i, 0), v2_idx = fs(i, 1),
		//	 v3_idx = fs(i, 2), v4_idx = fs(i, 3);
		// vec3 v1 = vs.row(v1_idx), v2 = vs.row(v2_idx),
		//	 v3 = vs.row(v3_idx), v4 = vs.row(v3_idx);
		// Eigen::Matrix<T, 3, 3, Eigen::RowMajor> block;
		// block.col(0) = v2 - v1;
		// block.col(1) = v3 - v1;
		// block.col(2) = v4 - v1;
		// block.transposeInPlace();
		// block.inverse(); // T^T = V^T^-1 * V'^T

		// ref_op[i].get_ind() << v1_idx, v2_idx, v3_idx, v4_idx;
		// ref_op[i].get_mat() = block;

	 //}

 }

 template<typename T, typename S>
 void DeformationGradient<T, S>::calc_normal_vector(){

	 if (std::is_same<S, DGTriangle4<T>>::value) {
		 ref_->calc_normal_vector();
		 for (int i = 0; i < targets_.size(); i++) {
			 targets_[i]->calc_normal_vector();
		 }
	 }
 }




 template<typename T, typename S>
 void DeformationGradient<T, S>::calc_deformation_gradient()
 {
	 calc_make_G_mat();
	 Mesh<T>::RowmatI& fs = ref_->get_face();
	 ROWMAT(T)& vs = ref_->get_verts();
	 
	 
 }

 template<typename T, typename S>
 void DeformationGradient<T, S>::calc_make_G_mat()
 {
	 //using Eigen::Vector
	 //std::vector<Eigen::Triplet<T>> triplets;
	 //if (std::is_same<S, DGTriangle4>::value) {
		// Mesh<T>::RowmatI& fs = ref_->get_face();
		// ROWMAT(T)& vs = ref_->get_verts();
		// for (int i = 0; i < fs.rows(); i++) {
		//	 int v1_idx = fs(i,0), v2_idx = fs(i, 1), 
		//		 v3_idx = fs(i,2), v4_idx = fs(i, 3);
		//	 vec3 v1 = vs.row(v1_idx), v2 = vs.row(v2_idx),
		//		 v3 = vs.row(v3_idx), v4 = vs.rows(v3_idx);
		//	 Eigen::Matrix<T, 3, 3, Eigen::RowMajor> block;
		//	 block.col(0) = v2 - v1;
		//	 block.col(1) = v3 - v1;
		//	 block.col(2) = v4 - v1;
		//	 block.trasposeInPlace();
		//	 block.inverse(); // T^T = V^T^-1 * V'^T
		//	
		//	 const int total_axis = 3;
		//	 const int matrix_col = 4;
		//	 for (int axis = 0; axis < total_axis; axis++) {
		//		 for (int row = 0; row < axis; row++) {
		//			const int row_offset = axis * fs_size() + i*3 + row;
		//			const int col_offset = axis * vs.size();
		//			triplets.emplace_back(row_offset, col_offset + v1_idx, -block(row, 0) - block(row, 1) - block(row, 2));
		//			triplets.emplace_back(row_offset, col_offset + v2_idx, block(row, 0));
		//			triplets.emplace_back(row_offset, col_offset + v3_idx, block(row, 1));
		//			triplets.emplace_back(row_offset, col_offset + v4_idx, block(row, 2)); 
		//		 }
		//	 }
		// }
	 //}
	 //else {

	 //}
	 //
	 //op_G_.setFromTriplets(triplets.begin(), triplets.end());
 }