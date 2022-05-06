#pragma once
#include <array>
template<typename T>
struct Vertex {
	T x, y, z;
};


template <typename T>
struct TriangleV {
	Vertex<T> v1, v2, v3;
} ;


typedef 
struct mesh_abs_storage {


	virtual void resize(int n) = 0;
}mesh_abs_storage;

// from face to neighbor face.
typedef 
struct Face2Faces : public mesh_abs_storage {
	using face_idx = int;
	std::vector<std::vector<face_idx>> faceidx_;
	
	void resize(int n) {
		faceidx_.reserve(n);
	}
}Face2Faces;

//from edge to face
typedef
struct Edge2Faces : public mesh_abs_storage {
	using face_idx = int;
	using edge_idx = int;
	std::map<std::pair<edge_idx, edge_idx>, std::vector<face_idx>> edges_;

	void resize(int n) {
		;
	}
}Edge2Faces;


//from vertex to face
typedef
struct vert2Faces : public mesh_abs_storage {
	using face_idx = int;
	std::vector<std::vector<face_idx>> vertidx_;

	void resize(int n) {
		vertidx_.reserve(n);
	}
}vert2Faces;
