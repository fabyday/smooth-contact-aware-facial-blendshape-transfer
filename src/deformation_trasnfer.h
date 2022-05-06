#pragma once
#include "Primitives.h"
#include "deformation_gradient.h"
#include <igl/AABB.h>

#define ROWMAT(T) Eigen::Matrix<T, -1,-1, Eigen::RowMajor>

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
	std::shared_ptr<DeformationGradient<T, struct DGTriangle4<T>>> source_;
	std::shared_ptr<DeformationGradient<T, struct DGTriangle4<T>>> target_;

	float ws_;
	float wi_;
	std::vector<float> wc_;
	
	Face2Faces f2f_;
	Edge2Faces e2f_;
	vert2Faces v2f_;


	ROWMAT(T) produce_smoothness();
	ROWMAT(T) produce_identity();
	ROWMAT(T) produce_closest();


};



