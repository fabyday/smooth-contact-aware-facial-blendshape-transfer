#pragma once
#include "Primitives.h"
#include "deformation_gradient.h"
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include<Eigen/SparseLU>	
#include <random>

#define WS_DEFAULT 1
#define WI_DEFAULT 0.001
#define WC_DEFAULT {0, 10, 50, 250, 1000, 2000, 3000, 5000}
template <typename T> 
using A_and_b =  std::pair< Sparse<T>, ROWMAT(T) >;

template <typename T>
class DeformationTransfer {
/**
* add_deformation_gradient_data->add_marker->add_constraint(option)->compile->solve
*/


public:
	constexpr static int SIZE = 4;
	DeformationTransfer(){
		ws_ = WS_DEFAULT;
		wi_ = WI_DEFAULT;
		wc_ = WC_DEFAULT;
		compile_flag_ = true;
	}
	

	//User Input
	void add_deformation_gradient_data(DeformationGradient<T, SIZE>& dg_source,
										DeformationGradient<T, SIZE>& dg_target);
	inline void add_marker(std::vector<std::tuple<int, int>>& marker_index) {
		marker_index_ = marker_index;
	}
	void add_constraint(ROWMAT(T) additional_mat);
	void process_correspondence();

	void compile(T ws, T wi, std::vector<T>& wc);
	void compile();
	void solve();

private:

	std::vector<std::tuple<int,int>> marker_index_;
	DeformationGradient<T, SIZE>* source_;
	DeformationGradient<T, SIZE>* target_;

	//triangle correspondence .. first optimizing problem.
	std::vector<std::vector<int>> corr_tri_tgt_from_src; // corr_tri_tgt_from_src[src_tri_num] = {tgt_tri_num1, ... tgt_tri_num_n}


	Mesh<T> src_copy_; //for preprocessing


	//coeff
	float ws_;
	float wi_;
	std::vector<float> wc_;

	bool compile_flag_;


	void process_neighbor();

	void produce_cloests_point();
	
	A_and_b<T> produce_smoothness();
	A_and_b<T> produce_identity();
	A_and_b<T> produce_closest();


	// sequence
	// add_marker-> phase1 -> phase2 -> recover_marker_point -> make_triangle_correspondence
	A_and_b<T> add_marker_constraint_to_matrix(const A_and_b<T>& ab_pair);
	ROWMAT(T) phase1(A_and_b<T>& S_pair, A_and_b<T>& I_pair);
	ROWMAT(T) phase2(A_and_b<T>& S_pair, A_and_b<T>& I_pair, A_and_b<T>& C_pair);
	ROWMAT(T) recover_marker_points_to_result(const ROWMAT(T)& x); 

	// final call in compile
	void make_triangle_correspondence();
};


#include "deformation_transfer.cpp"
