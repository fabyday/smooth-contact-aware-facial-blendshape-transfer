#include "deformation_gradient.h"

template<typename T, typename S>
DeformationGradient<T,S>::DeformationGradient(int targets_num) : targets(targets_num){
}



template<typename T, typename S>
void DeformationGradient<T, S>::add_reference(Mesh<T>& m)
{
	ref = m;
	v_size_ = m.verts_size();
	triangle_size_ = m.face_size();
	
}
template<typename T, typename S>
 void DeformationGradient<T,S>::add_target(std::shared_ptr<Mesh<T>>& m){
	 targets.push_back(m);
}



 template<typename T, typename S>
 void DeformationGradient<T, S>::add_targets(std::vector<Mesh<T>>& target_list){
	 foreach(target_list : mesh) {
		 targets.emplace_back(mesh)
	 }
 }

 template<typename T, typename S>
 inline void DeformationGradient<T, S>::compile(){



 }
