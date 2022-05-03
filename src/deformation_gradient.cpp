#include "deformation_gradient.h"

template<typename T>
DeformationGradient<T>::DeformationGradient(int targets_num) : targets(targets_num){
}


template<typename T>
void DeformationGradient<T>::add_reference(std::shared_ptr<Mesh<T>>& m){
	ref = m;
	size
}

template<typename T>
 void DeformationGradient<T>::add_target(std::shared_ptr<Mesh<T>>& m){
	 targets.push_back(m);
}

 template<typename T>
 void DeformationGradient<T>::add_targets(std::vector<Mesh>& target_list)
 {
	 foreach(target_list : mesh) {
		 
	 }
 }

 template<typename T>
 inline void DeformationGradient<T>::compile()
 {



 }
