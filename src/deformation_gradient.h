#pragma once

#include "Primitives.h"
#include "Mesh.h"
#include <vector>
#include <xutility>
#include <eigen/core>

template <typename T>
struct DGTriangle3 {
	TriangleV<T> t_;

	void copy_and_process(TriangleV& v) {
		Eigen::Map < Eigen::Matrix<T, 9, 1>> v_vec(v);// x y z x y z x y z;
		Eigen::Map < Eigen::Matrix<T, 9, 1>> result_vec(t_);// x y z x y z x y z;
		result_vec = v_vec;
	}

};


template <typename T>
struct DGTriangle4 : public DGTriangle3{
	Vertex<T> v4_;
	void copy_and_process(const TriangleV& v) {
		using vec3 = Eigen::Matrix<T, 3, 1>;
		DGTriangle3::copy_and_process(v);
		Eigen::Map < Eigen::Matrix<T, 9, 1>> vec(t_);
		Eigen::Map < vec3> vec_v4(v4_);

		vec3 v0 = vec.segment<3>(0);
		vec3 v1 = vec.segment<3>(3);
		vec3 v2 = vec.segment<3>(6);
		
		vec_v4 = Eigen::Cross(v1 - v0, v2 - v0);
	}
};





template<typename T>
typedef struct DeformationGradientData {
	T* ptr;
	int size;
	DeformationGradient(T* ptr, int size) {
		this->ptr = ptr;
		this->size = size;
	}
	T* operator++() {
		return ptr + size;
	}
	T* operator++(void) {
		T* reval = ptr;
		ptr += size;
		return ptr ;
	}
	T* operator+(int i) {
		return ptr + size*i;
	}
	T* operator[]() {

	}

} DeformationGradientDataPTR;



template<typename T, typename>
class DeformationGradient {
public :
	DeformationGradient(int targets_num=10);
	void add_reference(Mesh& m);
	void add_target(Mesh& m);
	void add_targets(std::vector<Mesh> target);
	void calc_deformation_gradient();
	void compile();
	void get_normal_vector();

private :

	inline bool check_same_size(Mesh<T>... a) {
		return a == b;
	}

	std::shared_ptr<Mesh<T>> ref_ = nullptr;
	std::vector<std::shared_ptr<Mesh<T>>> targets_ = nullptr;
	int v_size_;
	std::vector<>;
};

#include "deformation_gradient.cpp"