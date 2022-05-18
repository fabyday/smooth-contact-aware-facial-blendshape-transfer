#pragma once
#include "Primitives.h"
#include "deformation_gradient.h"
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>


#define WS_DEFAULT 1
#define WI_DEFAULT 1
#define WC_DEFAULT {1,2,3}


template <typename T>
class DeformationTransfer {
/**
* add_deformation_gradient_data->add_marker->add_constraint(option)->compile->solve
*/


public:
	DeformationTransfer(){
		ws_ = WS_DEFAULT;
		wi_ = WI_DEFAULT;
		wc_ = WC_DEFAULT;
		compile_flag_ = true;
	}


	//User Input
	void add_deformation_gradient_data(DeformationGradient<T, struct DGTriangle4<T>>& dg_source, 
										DeformationGradient<T, struct DGTriangle4<T>>& dg_target);
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
	DeformationGradient<T, struct DGTriangle4<T>>* source_;
	DeformationGradient<T, struct DGTriangle4<T>>* target_;

	float ws_;
	float wi_;
	std::vector<float> wc_;

	bool compile_flag_;

	// pointXYZ float
	//pcl::KdTreeFLANN<pcl::PointXYZ> src_kdtree_;


	Face2Faces f2f_;
	Edge2Faces e2f_;
	vert2Faces v2f_;

	void process_neighbor();

	void produce_cloests_point();
	Sparse<T> produce_smoothness();
	Sparse<T> produce_identity();
	Sparse<T> produce_closest();


	void phase1();
	void phase2();

};


#include "deformation_transfer.cpp"
