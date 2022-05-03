#pragma once
#include <array>
template<typename T>
typedef struct Vertex {
	T x, y, z;
}Vertex;


template <typename T>
typedef struct TriangleV {
	Vertex<T> v1, v2, v3;
} TriangleV;

