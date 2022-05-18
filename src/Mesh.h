#pragma once
#include <string>
#include <igl/read_triangle_mesh.h>
#include "Primitives.h"
template<typename T>
class Mesh { //lazy evaluation.
public:
	using string = std::string;
	typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor>  RowmatI;
	using vec3 = Eigen::Matrix<T, 1, 3, Eigen::RowMajor>;

private:
	Eigen::Matrix<T, -1, -1, Eigen::RowMajor> V;


	RowmatI F;

	int v_size_;
	int f_size_;
	int fn_size_;//offset, size
	int vn_size_;//offset, size

	bool dirty_flag_; // check vertex info chnaged.
	
	
	Face2Faces f2f_;
	Edge2Faces e2f_;
	vert2Faces v2f_;

public:
	Mesh() {

	}
	explicit public Mesh(const Mesh& mesh):
		V{ mesh.V }, F{ mesh.F }, dirty_flag_(mesh.dirty_flag_),
		v_size_(mesh.v_size_), f_size_(mesh.f_size_), fn_size_(mesh.fn_size_), vn_size_(mesh.vn_size_),
		f2f_(mesh.f2f_), e2f_(mesh.e2f_), v2f_(mesh.v2f_)
	{}

	inline void load_from_file(string name) {
		igl::read_triangle_mesh(name, V, F);
		v_size_ = V.rows();
		f_size_ = F.rows();
		init_geo_struct();



	}
	void init_geo_struct() {
		Mesh<T>& src_mesh_ptr = *this;
		ROWMAT(int) face = src_mesh_ptr.get_face();
		// edge -> face
		// vertex -> face 
		v2f_.resize(v_size_);
		for (int row_idx = 0; row_idx < face.rows(); row_idx++) {

			int f1 = face.row(row_idx)[0], f2 = face.row(row_idx)[1], f3 = face.row(row_idx)[2];
			e2f_.edges_[std::make_pair(f1, f2)].push_back(row_idx);
			e2f_.edges_[std::make_pair(f2, f3)].push_back(row_idx);
			e2f_.edges_[std::make_pair(f3, f1)].push_back(row_idx);
			v2f_.vertidx_[f1].push_back(row_idx);
			v2f_.vertidx_[f2].push_back(row_idx);
			v2f_.vertidx_[f3].push_back(row_idx);
		}

		// face -> faces
		f2f_.resize(f_size_);
		for (auto iter = e2f_.edges_.begin(); iter != e2f_.edges_.end(); iter++) {
			std::vector<int>& val = iter->second;
			for (auto iter_f = val.begin(); iter_f != val.end(); iter_f++) {
				int fidx = *iter_f;
				std::copy_if(val.begin(), val.end(),
					std::back_inserter(f2f_.faceidx_[fidx]),
					[fidx](int inserted_fidx) {return fidx != inserted_fidx; });
			}
		}
	}


	inline ROWMAT(T)& get_verts() {
		return V;
	}
	inline RowmatI& get_face() {
		return F;
	}
	inline int verts_size() {
		return v_size_;
	}
	inline int face_size() {
		return f_size_;
	}
	inline ROWMAT(T)& calc_vertex_normal_vector() {
		bool dirty_flag = dirty_flag_;
		calc_normal_vector();// face normal
		int size = v_size_ + f_size_ + v_size_;
		vn_size_ = size;
		V.conservativeResize(vn_size_, 3);


		
		ROWMAT(T)&& t = V.block(fn_size_, 0, vn_size_, 3);


		return t;
	}





	inline void calc_normal_vector() {
		if (dirty_flag_) {
			using vec3 = Eigen::Matrix<T, 1, 3, Eigen::RowMajor>;
			fn_size_ = v_size_ + f_size_;
			F.conservativeResize(f_size_, 4);
			V.conservativeResize(fn_size_, 3);
		
			for (int i = 0; i < F.rows(); i++) {
				F.row(i)[3] = v_size_ + i;
				int v1 = F.row(i)[0], v2 = F.row(i)[1], v3 = F.row(i)[2];
				vec3 e1 = V.row(v2) - V.row(v1);
				vec3 e2 = V.row(v3) - V.row(v1);
				vec3 e3 = e1.cross(e2);
				e3.normalize();
				V.row(v_size_ + i) = e3 + V.row(v1);
			}
		}

		
	}
};