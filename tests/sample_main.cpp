#include <iostream>
#include <deformation_gradient.h>
#include <dg_solver.h>
#include <Mesh.h>
#include <filesystem>


#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;




void 
get_marker(YAML::Node& root_node, std::vector<std::tuple<int,int>>& result) {

	if (root_node["markers"].IsSequence()) {
		for (auto iter = root_node["markers"].begin(); iter != root_node["markers"].end(); iter++) {
			std::string& str = iter->as < std::string>();
			int delim = str.find(':');
			int len = str.length();			
			int src = stoi(str.substr(0, delim));
			int dest = stoi(str.substr(delim+1, len));
			result.push_back(std::make_tuple(src, dest));
		}
	}

}

void get_meshes(YAML::Node& root_node, std::vector<Mesh<float>>& mesh_list, std::string tag) {
	
	//load src
	std::string root_path = RES"/lowpoly/";
	Mesh<float> ref;
	ref.load_from_file(root_path+root_node[tag]["reference"].as<std::string>());
	mesh_list.push_back(ref);

	for (auto iter = root_node[tag]["poses"].begin(); iter != root_node[tag]["poses"].end(); iter++) {
		Mesh<float> tmp_mesh;
		tmp_mesh.load_from_file(root_path+iter->as<std::string>());
		mesh_list.push_back(tmp_mesh);
	}
}

void 
get_virtual_triangle_idx(YAML::Node& root_node, std::vector<int>& result) {
	

}

void printm(const std::vector<std::tuple<int,int>>& s) {
	for each (auto var in s)
	{

		std::cout << std::get<0>(var)<< ":" << std::get<1>(var) << std::endl;
		

	}

}

int 
main() {
	DGSolver<float> slvr;
	
	std::vector<Mesh<float>> src_meshes, tgt_meshes; // 0 is reference.
	std::vector<std::tuple<int, int>> corr_marker;
	//fs::path src_dir(RES"lowpoly/""cat"), tgt_dir(RES"lowpoly/""dog");

	
	YAML::Node yaml_node = YAML::LoadFile(RES"/lowpoly/markers-cat-dog.yml");
	

	get_marker(yaml_node, corr_marker);
	get_meshes(yaml_node, src_meshes, "source");
	get_meshes(yaml_node, tgt_meshes, "target");

	printm(corr_marker);

	
	

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