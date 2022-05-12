#pragma once
#include "Primitives.h"
#include "deformation_gradient.h"
#include <kdtree.h>

template <typename T>
class DeformationTransfer {
/**
* add_deformation_gradient_data->add_marker->add_constraint(option)->compile->solve
*/
public:
	DeformationTransfer(){
		ws_ ws;
		wi_ = wi;
		wc_ = wc;
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

	// pointXYZ float
	pcl::KdTree<pcl::PointXYZ> src_kdtree;


	Face2Faces f2f_;
	Edge2Faces e2f_;
	vert2Faces v2f_;

	void process_neighbor();

	void produce_cloests_point();
	Sparse produce_smoothness();
	Sparse produce_identity();
	Sparse produce_closest();


};



#include "deformation_trasnfer.cpp"