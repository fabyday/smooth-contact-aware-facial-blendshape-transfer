#define LOG_CATEGORY "cst_test"

#include <iostream>
#include <deformation_gradient.h>
#include <dg_solver.h>
#include <Mesh.h>
#include <filesystem>


#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;






int
main() {
	DGSolver<float> slvr;

	Eigen::Matrix<float, -1, -1, Eigen::RowMajor> V;
	Eigen::Matrix<int, -1, -1, Eigen::RowMajor> F;
	V.resize(4, 3);
	F.resize(2, 3);
	V << -0.5, 0, 0,
		0.5, 0, 0,
		0, 0.5, 0,
		0, -0.5, 0
		;
	F << 0, 1, 2,
		0, 3, 1;

	std::vector<Mesh<float>> src_meshes(1), tgt_meshes(1); // 0 is reference.
	std::vector<std::tuple<int, int>> corr_marker;
	//fs::path src_dir(RES"lowpoly/""cat"), tgt_dir(RES"lowpoly/""dog");

	src_meshes[0].set_mesh(V, F);
	tgt_meshes[0].set_mesh(V, F);

	DeformationGradient<float, 4> src_dg4, tgt_dg4; // for deformation transfer

																			// [mesh0_ref | mesh_tgt(1) ... mesh_tgt(n)]
	src_dg4.add_reference(src_meshes[0]);
	std::vector<Mesh<float>> src_tgt(src_meshes.begin() + 1, src_meshes.end());
	src_dg4.add_targets(src_tgt);

	tgt_dg4.add_reference(tgt_meshes[0]);
	std::vector<Mesh<float>> tgt_tgt(tgt_meshes.begin() + 1, tgt_meshes.end());
	tgt_dg4.add_targets(tgt_tgt);



	slvr.add_deformation_gradient(DGSolver<float>::PURPOSE::SOURCE, src_dg4);
	slvr.add_deformation_gradient(DGSolver<float>::PURPOSE::TARGET, tgt_dg4);


	slvr.add_marker_index(corr_marker);
	//slvr.add_virtual_triangle_index();
	//
	//
	slvr.solve();
	return 0;

}