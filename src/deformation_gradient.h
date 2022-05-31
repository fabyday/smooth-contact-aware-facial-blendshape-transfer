#pragma once
#include "pretty_log.h"
#include "Primitives.h"
#include "Mesh.h"
#include <vector>
#include <xutility>
#include <eigen/core>
//
//template <typename T>
//struct DGTriangle3 {
//	TriangleV<T> t_;
//
//	constexpr static int elem_size() { return 3; }
//	
//	void copy_and_process(TriangleV<T>& v) {
//		Eigen::Map < Eigen::Matrix<T, 9, 1>> v_vec(v);// x y z x y z x y z;
//		Eigen::Map < Eigen::Matrix<T, 9, 1>> result_vec(t_);// x y z x y z x y z;
//		result_vec = v_vec;
//	}
//
//};
//
//
//template <typename T>
//struct DGTriangle4 : public DGTriangle3<T>{
//	Vertex<T> v4_;
//	
//	constexpr static int elem_size() { return 4; }
//
//	void copy_and_process(const TriangleV<T>& v) {
//		using vec3 = Eigen::Matrix<T, 3, 1>;
//		DGTriangle3<T>::copy_and_process(v);
//		Eigen::Map < Eigen::Matrix<T, 9, 1>> vec(t_);
//		Eigen::Map < vec3> vec_v4(v4_);
//		
//		vec3 v0 = vec.segment<3>(0);
//		vec3 v1 = vec.segment<3>(3);
//		vec3 v2 = vec.segment<3>(6);
//		
//		vec_v4 = Eigen::Cross(v1 - v0, v2 - v0);	
//	}
//};

template<typename T, int SIZE>
struct TriangleDeformationGradient {
	static_assert(SIZE ==  3 || SIZE == 4);
	using VectorSi = Eigen::Matrix < int, 4, 1>;

	std::array<T, 3*(SIZE -1)> dg_; // v2 - v4 , v3-v1, v4-v1
	std::array<int, SIZE> ind_; // v1, v2, v3, v4 vertex index
	int tri_num_;
	Eigen::Map<ROWMAT(T)> get_mat() {		
		return Eigen::Map<ROWMAT(T)>(dg_.data(), 3, 3);
	}
	Eigen::Map<VectorSi> get_ind() {
		return Eigen::Map< VectorSi>(ind_.data());
	}

};  

template<typename T, int SIZE =4>
struct DeformationGradientCollection { // iterator.....?
	std::vector<TriangleDeformationGradient<T, SIZE>> data;


	void resize(int size) {
		data.resize(size);
	}
	TriangleDeformationGradient<T, SIZE>& operator[](int idx) {
		return data[idx];
	}
	int size() {
		return data.size();
	}
};



template<typename T, int SIZE=4>
class DeformationGradient {
public :
	void add_reference(Mesh<T>& m);
	void add_target(Mesh<T>& m);
	void add_targets(std::vector<Mesh<T>>& target);
	void calc_deformation_gradient();
	void calc_make_G_mat();
	
	void compile();
	void get_normal_vector();
	void calc_normal_vector();


	bool is_compiled_;
	DeformationGradient(void) {
		is_compiled_ = false;
	}
	
	//DeformationGradient(int targets_num=10) {}

	inline Mesh<T>& get_ref_mesh() {
		return *ref_;
	}


	// inverse V matrix.(from reference)
	inline TriangleDeformationGradient<T, SIZE>& get_inv_matrix(int face_idx) {
		return deformation_gradients_[0][face_idx];
	}

	// ref1 -> target(same topology with ref1)
	inline const TriangleDeformationGradient<T, SIZE>& get_deformation_gradient(int to_ref_target_idx, int face_idx) {
		if (to_ref_target_idx < 0)
			throw std::runtime_error("index error. index must be singed.");

		return deformation_gradients_[to_ref_target_idx + 1][face_idx];
	}

	inline const int get_v_size() { return v_size_; };


private :
	void calc_op_g_list();
	void reserve_all();

	Mesh<T>* ref_ = nullptr;
	std::vector<Mesh<T>*> targets_;
	int v_size_;
	int triangle_size_;


	Sparse<T> op_G_;
	std::vector<TriangleDeformationGradient<T, SIZE>> ref_op;

	//use MeshDGAccessor, help gradeint access per mesh.
	std::vector<DeformationGradientCollection<T, SIZE>> deformation_gradients_; // [mesh1_grads mesh2_grads ... mesh_n grads]
};

#include "deformation_gradient.cpp"