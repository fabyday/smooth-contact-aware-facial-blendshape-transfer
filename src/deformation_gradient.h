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
struct TriangleDeformationGradient {
	std::array<T, 9> dg_;
};

template<typename T>
struct MeshDGAccessor { // iterator.....?
	TriangleDeformationGradient<T>* ptr_;
	int size_;
	DeformationGradient(T* ptr, int size) {
		this->ptr_ = ptr;
		this->size_ = size;
	}
	T* operator++() {
		return ptr_ + size;
	}
	T* operator++(void) {
		ptr_ += size * i;
		return ptr;
	}
	T* operator+(int i) {
		T* reval = ptr_;
		ptr += size;
		return ptr_;
		
	}
	T* operator[](int i) {
		return ptr;

	}

};



template<typename T, typename S>
class DeformationGradient {
public :
	DeformationGradient(int targets_num=10);
	void add_reference(Mesh<T>& m);
	void add_target(std::shared_ptr<Mesh<T>>& m);
	void add_targets(std::vector<Mesh<T>>& target);
	void calc_deformation_gradient();
	
	void compile();
	void get_normal_vector();

	std::shared_ptr<Mesh<T>> get_ref_mesh();

private :



	std::shared_ptr<Mesh<T>> ref_ = nullptr;
	std::vector<std::shared_ptr<Mesh<T>>> targets_ = nullptr;
	int v_size_;
	int triangle_size_;

	//use MeshDGAccessor, help gradeint access per mesh.
	std::vector<TriangleDeformationGradient<T>> deformation_gradients_; // [mesh1_grads mesh2_grads ... mesh_n grads]
};

#include "deformation_gradient.cpp"