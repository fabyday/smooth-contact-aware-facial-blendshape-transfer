#pragma once
#include <string>
#include <igl/read_triangle_mesh.h>

template<typename T>
class Mesh {
private:
	Eigen::Matrix<T, -1,-1, Eigen::RowMajor> V;
	Eigen::MatrixXi F;

public :
	using string=std::string;

	inline void load_from_file(string name) {
		igl::read_triangle_mesh(name, V, F);
	}
	inline Eigen::MatrixXd& get_verts() {
		return V;
	}
	inline Eigen::MatrixXi& get_face() {
		return F;
	}
	inline int verts_size() {
		return V.rows();
	}
	inline int face_size() {
		return F.rows();
	}
};