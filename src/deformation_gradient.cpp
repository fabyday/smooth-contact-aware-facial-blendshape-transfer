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
	 
	 targets_T.resize(targets_.sizse());
	 for (int i = 0; i < targets_T.size(); i++) {
		 //target_T[i]
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
		 //deformation_gradients_;
		 targets_T.resize(1 + targets_.size());
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
		 ref_->calc_face_normal_vector();
		 for (int i = 0; i < targets_.size(); i++) {
			 targets_[i]->calc_face_normal_vector();
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
	 deformation_gradients_.resize(fs.rows());
	 DeformationGradientCollection<T, 4>& data = deformation_gradients_;

	 for (int i = 0; i < deformation_gradients_.size(); i++) {

		Eigen::Map<ROWMAT(T)> mat = data[i].get_mat();
		Eigen::Map<TriangleDeformationGradient<T,4>::VectorSi> idx =  data[i].get_ind();
		
		int v1_idx = fs(i,0), v2_idx = fs(i, 1), 
 			v3_idx = fs(i,2), v4_idx = fs(i, 3);
		
		Vec3 v1 = vs.row(v1_idx), v2 = vs.row(v2_idx),
			 v3 = vs.row(v3_idx), v4 = vs.row(v4_idx);
		
		// current face normal v4 == v4 + v1...
		Row3mat block;
		block.col(0) = v2 - v1;
		block.col(1) = v3 - v1;
		//block.col(2) = v4 - v1;
		//block.col(2) = v4; // v4 is locally normal vector.(Mesh<T> didn't add v1 to v4.)
		Eigen::Matrix<T, 3, 1> vec1, vec2, res_vec;
		vec1 = block.col(0); vec2 = block.col(1);
		vec1.normalize();
		vec2.normalize();
		res_vec = vec1.cross(vec2);
		//block.col(2) = res_vec.eval()/std::sqrt(res_vec.norm()); // v4 is locally normal vector.(Mesh<T> didn't add v1 to v4.)
		block.col(2) = res_vec.eval()/(res_vec.norm()); // v4 is locally normal vector.(Mesh<T> didn't add v1 to v4.)
			
		block.transposeInPlace();
		//block = block.inverse().eval; // T^T = V^T^-1 * V'^T
		//mat = block; // assign block
		mat = block.inverse(); // same as above comments.
		
		idx(0, 0) = v1_idx; idx(1, 0) = v2_idx; idx(2, 0) = v3_idx;	idx(3, 0) = v4_idx; // assign idx
		data[i].tri_num_ = i; // assign face idx
		
	 }
	 
	 
 }

 template<typename T, int S>
 void DeformationGradient<T, S>::calc_make_G_mat()
 {
 }