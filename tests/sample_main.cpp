#include <iostream>
#include <deformation_gradient.h>
#include <dg_solver.h>
#include <Mesh.h>
#include <filesystem>

#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

int main() {
	DGSolver<float> slvr;
		
	std::vector<Mesh<float>> src_meshes, tgt_meshes; // 0 is reference.
	
	fs::path src_dir(RES"lowpoly/""cat"), tgt_dir(RES"lowpoly/""dog");
	//load src
	for (auto const& f_it : fs::directory_iterator{ src_dir }) {
		Mesh<float> tmp_mesh;
		if (f_it.path().extension() == ".obj") {
			tmp_mesh.load_from_file(f_it.path().string());
			src_meshes.push_back(tmp_mesh);
		}
	}
	//load target
	for (auto const& f_it : fs::directory_iterator{ tgt_dir }) {
		Mesh<float> tmp_mesh;
		if (f_it.path().extension() == ".obj") {
			tmp_mesh.load_from_file(f_it.path().string());
			src_meshes.push_back(tmp_mesh);
		}
	}
	

	DeformationGradient<float, struct DGTriangle4<float>> src_dg4, tgt_dg4; // for deformation transfer
	// [mesh0_ref | mesh_tgt(1) ... mesh_tgt(n)]
	src_dg4.add_reference(src_meshes[0]);
	src_dg4.add_targets(std::vector<Mesh<float>>(src_meshes.begin() + 1, src_meshes.end()));
	
	src_dg4.add_reference(tgt_meshes[0]);
	tgt_dg4.add_targets(std::vector<Mesh<float>>(tgt_meshes.begin() + 1, tgt_meshes.end()));
	


	slvr.add_deformation_gradient(src_dg4);



	slvr.add_constraint(DGSolver<float>::ConstraintType::VIRTUAL_VERTEX_FOR_TRIANGLE, );
	slvr.add_constraint(DGSolver<float>::ConstraintType::POINT_CONSTRAINT,);
	

	return 0;

}