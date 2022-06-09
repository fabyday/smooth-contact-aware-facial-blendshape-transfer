#pragma once
#include <string>
#include <igl/read_triangle_mesh.h>
#include <igl/write_triangle_mesh.h>
#include "Primitives.h"
#include <algorithm>
#include <cmath>
template<typename T>
class Mesh { //lazy evaluation.
public:
	using string = std::string;
	typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor>  RowmatI;
	using vec3 = Eigen::Matrix<T, 1, 3, Eigen::RowMajor>;

private:
	Eigen::Matrix<T, -1, -1, Eigen::RowMajor> V;


	RowmatI F;

	enum DIRTY_FLAG {D_VERTEX_NORMAL = BIT(0), D_FACE_NORMAL = BIT(1)};


	int v_size_;
	int f_size_;
	int fn_size_;//offset, size
	int vn_size_;//offset, size

	bool dirty_flag_; // check vertex info chnaged.
	//_8BIT_FALG dirty_flag;
	
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

	Mesh& operator=(const Mesh& mesh){
		V = mesh.V;
		F = mesh.F; 
		dirty_flag_ = true;
		v_size_ = mesh.v_size_; 
		f_size_ = mesh.f_size_; 
		fn_size_ = mesh.fn_size_; 
		vn_size_ = mesh.vn_size_;
		f2f_ = mesh.f2f_;
		e2f_ = mesh.e2f_;  
		v2f_ = mesh.v2f_;

		return *this;

	}

	inline void load_from_file(string name) {
		igl::read_triangle_mesh(name, V, F);
		v_size_ = V.rows();
		f_size_ = F.rows();
		init_geo_struct();
	}
	inline void set_mesh(ROWMAT(T)& V, RowmatI& F) {
		this->V = V;
		this->F = F;
		v_size_ = this->V.rows();
		f_size_ = this->F.rows();
		init_geo_struct();
	}
	inline void save_file(string name) {
		igl::write_triangle_mesh(name, V.block(0,0, v_size_, 3), F.block(0,0, f_size_, 3));
	}

	void init_geo_struct() {
		Mesh<T>& src_mesh_ptr = *this;
		ROWMAT(int) face = src_mesh_ptr.get_face();
		// edge -> face
		// vertex -> face 
		v2f_.resize(v_size_);
		for (int row_idx = 0; row_idx < face.rows(); row_idx++) {

			int f1 = face.row(row_idx)[0], f2 = face.row(row_idx)[1], f3 = face.row(row_idx)[2];
			e2f_.edges_[std::minmax(f1, f2)].push_back(row_idx);
			e2f_.edges_[std::minmax(f2, f3)].push_back(row_idx);
			e2f_.edges_[std::minmax(f3, f1)].push_back(row_idx);
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
					[fidx](int inserted_fidx) {	return fidx != inserted_fidx; });
			}
		}
	}

	inline Face2Faces& get_f2f() { return f2f_; }
	inline Edge2Faces& get_e2f() { return e2f_; }
	inline vert2Faces& get_v2f() { return v2f_; }


	inline ROWMAT(T) get_all_components_verts() {
		return V;
	}

	inline Eigen::Block<ROWMAT(T)> get_verts() {
		return V.block(0,0,v_size_ ,3);
	}
	inline Eigen::Block<ROWMAT(T)> get_face_normal_verts() {
		calc_normal_vector();
		return V.block(v_size_, 0, fn_size_, 3);
	}

	inline Eigen::Block<ROWMAT(T)> get_vertex_normal_verts() {
		calc_vertex_normal_vector();
		return V.block(fn_size_, 0, vn_size_, 3);
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
	inline ROWMAT(T) calc_vertex_normal_vector() {
		bool dirty_flag = dirty_flag_;
		
		calc_face_normal_vector();// face normal
		//int size = v_size_ + f_size_ + v_size_;
		//if(dirty_flag){
		if(true){
			dirty_flag_ = false;
			int size = fn_size_ + v_size_;
			vn_size_ = size;
			V.conservativeResize(vn_size_, 3);
			V.block(fn_size_, 0, v_size_, 3).setZero();

			const ROWMAT(T) VFN = V.block(v_size_, 0, f_size_, 3); //TODO

			for (int i = 0; i < v2f_.vertidx_.size(); i++) {
				ROWMAT(T) v_norm_vec = ROWMAT(T)::Zero(1, 3);
				const int neighbor_f_size = v2f_.vertidx_[i].size();
				std::for_each(v2f_.vertidx_[i].begin(), v2f_.vertidx_[i].end(),
					[i, neighbor_f_size, &v_norm_vec, &VFN](int s) { 
						//v_norm_vec.row(0) += VFN.row(s)/ neighbor_f_size ; 
						v_norm_vec.row(0) += VFN.row(s); 
					}
				);
				v_norm_vec /= neighbor_f_size;
				V.row(i + fn_size_) = v_norm_vec;
				V.row(i + fn_size_).normalize();


			}

		// test foreach
		//float test = 0;
		//std::vector as = { 1,2,3,4,5,6,7,8,9,10 };
		//auto ssize = as.size();
		////auto f = [&test, ssize](int s) { std::cout << (float)s/ssize; };
		//auto f = [&test, ssize](int s) { test += s / (float)ssize; };
		//std::for_each(as.begin(), as.end(),
		//	f
		//);
		//std::cout << test;
		}
		ROWMAT(T) t = V.block(fn_size_, 0, v_size_, 3); //hard copy...

		return t;
	}

	inline ROWMAT(T) get_centroid_tri() {
		ROWMAT(T) reval;
		reval.resize(F.rows(), 3);
		for (int i = 0; i < F.rows(); i++) {
			int v1 = F(i, 0), v2 = F(i,1), v3=F(i,2);
			reval.row(i) = (V.row(v1) + V.row(v2) + V.row(v3)) / 3;
		}
		return reval;
	}


	void update_v(const ROWMAT(T)& verts){
		dirty_flag_ = true;
		assert(verts.rows() == v_size_ && "verts_size and v_size_ is diff");
		V.block(0, 0, v_size_, 3) = verts;
	}


	inline ROWMAT(T) calc_face_normal_vector() {
		//if (dirty_flag_) {
		if (true) {
			dirty_flag_ = false;
			using vec3 = Eigen::Matrix<T, 1, 3, Eigen::RowMajor>;
			fn_size_ = v_size_ + f_size_;
			F.conservativeResize(f_size_, 4);
			V.conservativeResize(fn_size_, 3);
		
			for (int i = 0; i < F.rows(); i++) {
				F.row(i)[3] = v_size_ + i;
				int v1 = F.row(i)[0], v2 = F.row(i)[1], v3 = F.row(i)[2];
				vec3 e1 = V.row(v2) - V.row(v1);
				vec3 e2 = V.row(v3) - V.row(v1);
				e1.normalize();
				e2.normalize();
				vec3 e3 = e1.cross(e2);
			//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
				e3.normalize(); ///
				// TODO
				//e3 /= std::sqrt(e3.norm());
			//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
				V.row(v_size_ + i) = e3;

			}
		}
		return V.block(v_size_, 0, f_size_, 3);
	}

	
	//inline ROWMAT(T) calc_normal_vector() {
	//	if (dirty_flag_) {
	//		using vec3 = Eigen::Matrix<T, 1, 3, Eigen::RowMajor>;
	//		fn_size_ = v_size_ + f_size_;
	//		F.conservativeResize(f_size_, 4);
	//		V.conservativeResize(fn_size_, 3);

	//		for (int i = 0; i < F.rows(); i++) {
	//			F.row(i)[3] = v_size_ + i;
	//			int v1 = F.row(i)[0], v2 = F.row(i)[1], v3 = F.row(i)[2];
	//			vec3 e1 = V.row(v2) - V.row(v1);
	//			vec3 e2 = V.row(v3) - V.row(v1);
	//			vec3 e3 = e1.cross(e2);
	//			e3.normalize();
	//			V.row(v_size_ + i) = e3 + V.row(v1);
	//		}
	//	}

	//	return V.block(v_size_, 0, f_size_, 3);
	//}



};