#pragma once
#include <string>
#include <igl/read_triangle_mesh.h>
#include "Primitives.h"
template<typename T>
class Mesh {
public : 
	using string = std::string;
	typedef Eigen::Matrix<int, -1, -1, Eigen::RowMajor>  RowmatI;

private:
	Eigen::Matrix<T, -1,-1, Eigen::RowMajor> V;
	
	
	RowmatI F;

	int v_size_;
	int f_size_;
	int n_size_;

public :

	inline void load_from_file(string name) {
		igl::read_triangle_mesh(name, V, F);
		v_size_ = V.rows();
		f_size_ = F.rows();
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

	inline void calc_normal_vector() {
		using vec3 = Eigen::Matrix<T, 1, 3, Eigen::RowMajor>;
		n_size_ = v_size_ + f_size_;
		F.conservativeResize(f_size_, 4);
		V.conservativeResize(n_size_, 3);
		
		for (int i = 0; i < F.rows(); i++) {
			F.row(i)[3] = v_size_ + i;
			int v1 = F.row(i)[0], v2 = F.row(i)[1], v3 = F.row(i)[2];
			vec3 e1 = V.row(v2) - V.row(v1);
			vec3 e2 = V.row(v3) - V.row(v1);
			std::cout << e1.cross(e2) << std::endl;
			vec3 e3 = e1.cross(e2);
			e3.normalize();
			V.row(v_size_ + i) = e3 + V.row(v1);
		}
	}
};