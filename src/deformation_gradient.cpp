#include "deformation_gradient.h"




template<typename T, int S>
void DeformationGradient<T, S>::add_reference(Mesh<T>& m)
{
	ref_ = &m;
	v_size_ = m.verts_size();
	triangle_size_ = m.face_size();
	
	
}
template<typename T, int S>
 void DeformationGradient<T,S>::add_target(Mesh<T>& m){
	 targets_.push_back(&m);
}

 template<typename T, int S>
 void DeformationGradient<T, S>::reserve_all()
 {
	 
	 deformation_gradients_.resize(1 + targets_.sizse());
	 for (int i = 0; i < deformation_gradients_.size(); i++) {
		 deformation_gradients_[i].
	 }
 }

 template<typename T, int S>
 void DeformationGradient<T, S>::add_targets(std::vector<Mesh<T>>& target_list){

	for (int i = 0; i < target_list.size(); i++) {
		 targets_.push_back(&target_list[i]); 
	 }
 }

 template<typename T, int S>
 inline void DeformationGradient<T, S>::compile(){
	 // TODO 
	 if(!is_compiled_){
		 is_compiled_ = true;
		 deformation_gradients_.resize(1 + targets_.size());
		 calc_normal_vector();
		 calc_op_g_list();
		 calc_deformation_gradient();
	 }
 }

 template<typename T, int S>
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

 template<typename T, int S>
 void DeformationGradient<T, S>::calc_normal_vector(){

	 if (S== 4) {
		 ref_->calc_normal_vector();
		 for (int i = 0; i < targets_.size(); i++) {
			 targets_[i]->calc_normal_vector();
		 }
	 }
 }




 template<typename T, int S>
 void DeformationGradient<T, S>::calc_deformation_gradient()
 {
	 using Vec3 = Eigen::Matrix<T, 3, 1>;
	 using Row3mat = Eigen::Matrix<T, 3, 3, Eigen::RowMajor>;
	 calc_make_G_mat();
	 Mesh<T>::RowmatI& fs = ref_->get_face();
	 ROWMAT(T) vs = ref_->get_all_components_verts(); // it contains verts and face normal or verts normal
	 deformation_gradients_[0].resize(fs.rows());
	 DeformationGradientCollection<T, 4>& data = deformation_gradients_[0];
	 for (int i = 0; i < deformation_gradients_[0].size(); i++) {
		Eigen::Map<ROWMAT(T)> mat = data[i].get_mat();
		Eigen::Map<TriangleDeformationGradient<T,4>::VectorSi> idx =  data[i].get_ind();
		

		int v1_idx = fs(i,0), v2_idx = fs(i, 1), 
 			v3_idx = fs(i,2), v4_idx = fs(i, 3);
		
		Vec3 v1 = vs.row(v1_idx), v2 = vs.row(v2_idx),
			 v3 = vs.row(v3_idx), v4 = vs.row(v4_idx);
		
		Row3mat block;
		block.col(0) = v2 - v1;
		block.col(1) = v3 - v1;
		block.col(2) = v4 - v1;
		log_dense(block)
		block.transposeInPlace();
		log_dense(block)
		//block = block.inverse().eval; // T^T = V^T^-1 * V'^T
		//mat = block; // assign block
		mat = block.inverse(); // same as above comments.

		idx(0, 0) = v1_idx; idx(1, 0) = v2_idx; idx(2, 0) = v3_idx;	idx(3, 0) = v4_idx; // assign idx
		data[i].tri_num_ = i; // assign face idx



		PRETTY_LOG_BEGIN("DG_STATE")
		log_msg("==inv T==")
		log_dense(mat)
		ROWMAT(T) test;

		ROWMAT(T) wow_; wow_.resize(4, 3);

		wow_.row(0) = v1.transpose();
		wow_.row(1) = v2.transpose();
		wow_.row(2) = v3.transpose();
		wow_.row(3) = v4.transpose();
		test.resize(3, 4);
		for (int i = 0; i < 3; i++) {
			test(i, 0) =  -(mat(i, 0) + mat(i, 1) + mat(i, 2));
			test(i, 1) = mat(i, 0);
			test(i, 2) = mat(i, 1);
			test(i, 3) = mat(i, 2);
		}
		ROWMAT(T) tmp = test * wow_;
		log_msg("======")
		log_dense(tmp)
		log_msg("======")
		ROWMAT(T) b; b.resize(9, 1);
		b << 1, 0, 0, 0, 1, 0, 0, 0, 1;
		ROWMAT(T) A; A.resize(test.rows() * 3, test.cols() * 3);
		A.setZero();
		A.block<3, 4>(0, 0) = test;
		A.block<3, 4>(3, 4) = test;
		A.block<3, 4>(6, 8) = test;
		log_msg("==A==")
		log_dense(A)

		log_msg("==b==")
		log_dense(b)
		log_msg("==ATA==")
		log_dense((A.transpose().eval() * A))
		Eigen::LLT<ROWMAT(T)> slvr(A.transpose().eval() * A);

		log_msg("===AT===")
		log_dense((A.transpose() ))
		log_msg("===ATb===")
		log_dense((A.transpose() * b))
		log_msg("====sol==")
		log_dense(slvr.solve(A.transpose().eval() * b))
		
		log_msg("====check==")
		ROWMAT(T) sols = slvr.solve(A.transpose().eval() * b);
		ROWMAT(T) res;  res.conservativeResize(4, 3);
		ROWMAT(T) res_real;  res_real.conservativeResize(12,1);
		res.col(0) = sols.block<4, 1>(0, 0);
		res.col(1) = sols.block<4, 1>(4, 0);
		res.col(2) = sols.block<4, 1>(8, 0);
		log_msg("==slvr result==")
		log_dense((res))
		log_msg("==real result==")
		log_dense((wow_))
		log_msg("==A * slvr_result==")
		log_dense((test * res))
		log_msg("==A * real_result==")
		
		res_real.block<4,1>(0,0) = wow_.block<4,1>(0,0);
		res_real.block<4,1>(4,0) = wow_.block<4,1>(0,1);
		res_real.block<4,1>(8,0) = wow_.block<4,1>(0,2);
		log_dense((A * res_real))

		log_msg("== big_A * flat_slvr_result==")
		log_dense((A * sols))


		log_msg("== test simple==")
		ROWMAT(T) mats(3,3);
		mats << 1, 0, 0, 0, 2, 0, 0, 0, 3;
		Eigen::LLT<ROWMAT(T)> slvr2(mats);
		ROWMAT(T) bs(3, 1);
		bs << 3, 2, 3;
		log_dense(slvr2.solve(bs))


		ROWMAT(T) reduced_A(9, 9);
		int cols = static_cast<int>(reduced_A.cols()/3);
		int A_cols = static_cast<int>(A.cols()/3);
		std::set<int> a = { 0 };
		int j = 0; 
		for (int i = 0; i < 4; i++) {
			if (a.count(i))
				continue;
			reduced_A.col(j) = A.col(i);
			reduced_A.col(cols + j) = A.col(1* A_cols+i);
			reduced_A.col(cols*2 + j++) = A.col(2* A_cols+i);
		}
		log_msg("==A==")
		log_dense(A)
		log_msg("==reduced_A==")
		log_dense(reduced_A)
		ROWMAT(T) xx;
		log_msg("xx")
		xx.resize(12, 1); xx << wow_(0, 0), 0, 0, 0, wow_(0, 1), 0, 0, 0, wow_(0, 2), 0, 0, 0;
		log_dense(xx)
		ROWMAT(T) b_bar = (b - A * xx);
		Eigen::LLT<ROWMAT(T)> slvr3(reduced_A.transpose().eval() * reduced_A);
		log_msg("cons result")
		log_dense(slvr3.solve((reduced_A.transpose() * b_bar)))
		PRETTY_LOG_END()
		
	 }
	 
	 
 }

 template<typename T, int S>
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