#include "deformation_trasnfer.h"
#include <cmath>

#include "pretty_log.h"

#include <iostream>

void load_file_as_vector(std::vector<int>& result, std::string filename) {

	std::ifstream openfile(filename);
	if (openfile.is_open()) {
		std::string line;
		while (getline(openfile, line)) {
			std::stringstream ss(line);
			std::string t, tt;
			getline(ss, t, ' ');
			tt = t;
			getline(ss, t, ' ');
			result[atoi(tt.c_str())] = atoi(t.c_str());
		}
	}


}


template<typename T>
void load_row_mat(ROWMAT(T)& res) {

	std::vector<T> raw;

	std::ifstream openfile("D:/lab/2022/mycode/smooth-contact-aware-facial-blendshape-trasnfer/examples/b.txt");
	if (openfile.is_open()) {
		std::string line;
		while (getline(openfile, line)) {
			std::stringstream ss(line);
			std::string t;
			while(getline(ss, t, ' '))
				raw.push_back(atof(t.c_str()));
		}
	}
	int r_size = static_cast<int>(raw.size() / 3);
	Eigen::Map<ROWMAT(T)> rmap(raw.data(), r_size, 3);
	res = rmap;
}



template<typename T>
void DeformationTransfer<T>::process_neighbor()
{

	//Mesh<T>& src_mesh_ptr = source_->get_ref_mesh();
	//ROWMAT(int) face = src_mesh_ptr.get_face();
	//// edge -> face
	//// vertex -> face 
	//for (int row_idx = 0; row_idx < face.rows(); row_idx++) {

	//	int f1 = face.row(row_idx)[0], f2 = face.row(row_idx)[1], f3 = face.row(row_idx)[2];
	//	e2f_.edges_[std::make_pair(f1, f2)].push_back(row_idx);
	//	e2f_.edges_[std::make_pair(f2, f3)].push_back(row_idx);
	//	e2f_.edges_[std::make_pair(f3, f1)].push_back(row_idx);
	//	v2f_.vertidx_[f1].push_back(row_idx);
	//	v2f_.vertidx_[f2].push_back(row_idx);
	//	v2f_.vertidx_[f3].push_back(row_idx);
	//}

	//// face -> faces
	//for (auto iter = e2f_.edges_.begin(); iter != e2f_.edges_.end(); iter++) {
	//	std::vector<int>& val = iter->second;
	//	for (auto iter_f = val.begin(); iter_f != val.end(); iter_f++) {
	//		int fidx = *iter_f;
	//		std::copy_if(val.begin(), val.end(),
	//			std::back_inserter(f2f_.faceidx_[fidx]),
	//			[fidx](int inserted_fidx) {return fidx != inserted_fidx; });
	//	}
	//}

}

//
//template<typename T>
//void operator+=(Sparse<T>& A, TriangleDeformationGradient<T>& tdg) {
//
//}
//
//template<typename T>
//void operator-=(Sparse<T>& A, TriangleDeformationGradient<T>& tdg) {
//
//}

template <typename T>
void add_sparse(Sparse<T>& A, TriangleDeformationGradient<T, 4>& tdg, int tri_idx, int axis_size = 3) {
	Eigen::Vector4i idx = tdg.get_ind();
	ROWMAT(T) invV_adj = tdg.get_mat();
	assert(A.rows() % axis_size == 0 && A.cols() % axis_size == 0);

	const int row_size = static_cast<int>(A.rows() / axis_size);
	const int col_size = static_cast<int>(A.cols() / axis_size);
	for (int ax = 0; ax < axis_size; ax++) {
		for (int i = 0; i < 3; i++) {
			A.coeffRef(row_size * ax + tri_idx * 3 + i, col_size * ax + tdg.get_ind()(0, 0)) += (-invV_adj(i, 0) - invV_adj(i, 1) - invV_adj(i, 2));
			A.coeffRef(row_size * ax + tri_idx * 3 + i, col_size * ax + tdg.get_ind()(1, 0)) += (invV_adj(i, 0));
			A.coeffRef(row_size * ax + tri_idx * 3 + i, col_size * ax + tdg.get_ind()(2, 0)) += (invV_adj(i, 1));
			A.coeffRef(row_size * ax + tri_idx * 3 + i, col_size * ax + tdg.get_ind()(3, 0)) += (invV_adj(i, 2));
		}
	}	
}

template <typename T>
void sub_sparse(Sparse<T>& A, TriangleDeformationGradient<T, 4>& tdg, int tri_idx, int axis_size = 3) {
	Eigen::Vector4i idx = tdg.get_ind();
	ROWMAT(T) invV_adj = tdg.get_mat();
	assert(A.rows() % axis_size == 0 && A.cols() % axis_size == 0);

	const int row_size = static_cast<int>(A.rows() / axis_size);
	const int col_size = static_cast<int>(A.cols() / axis_size);
	for (int ax = 0; ax < axis_size; ax++) {
		for (int i = 0; i < 3; i++) {
			A.coeffRef(row_size * ax + tri_idx * 3 + i, col_size * ax + tdg.get_ind()(0, 0)) -= (-invV_adj(i, 0) - invV_adj(i, 1) - invV_adj(i, 2));
			A.coeffRef(row_size * ax + tri_idx * 3 + i, col_size * ax + tdg.get_ind()(1, 0)) -= (invV_adj(i, 0));
			A.coeffRef(row_size * ax + tri_idx * 3 + i, col_size * ax + tdg.get_ind()(2, 0)) -= (invV_adj(i, 1));
			A.coeffRef(row_size * ax + tri_idx * 3 + i, col_size * ax + tdg.get_ind()(3, 0)) -= (invV_adj(i, 2));
		}
	}
}


template <typename T>
inline A_and_b<T>  DeformationTransfer<T>::produce_smoothness()
{
	//Mesh<T>& mesh_ref = source_->get_ref_mesh();
	//ROWMAT(int)& face = mesh_ref.get_face();
	//ROWMAT(T) verts = mesh_ref.get_verts();

	//Face2Faces& f2f = mesh_ref.get_f2f();

	//const int row_size = mesh_ref.face_size();
	//const int col_size = mesh_ref.verts_size() + mesh_ref.face_size(); // normal_vert_num + vert_num

	//Sparse<T> G(3*(3*row_size), 3*col_size);
	//G.setZero();	

	//for (int i = 0; i < row_size; i++) {
	//	TriangleDeformationGradient<T, SIZE> tdg_i = source_->get_inv_matrix(i); // Triangle i 
	//	for (auto iter = f2f.faceidx_[i].begin(); iter != f2f.faceidx_[i].end(); iter++) {
	//		const int adj_face_idx = *iter;
	//		TriangleDeformationGradient<T, SIZE>& tdg_adj = source_->get_inv_matrix(adj_face_idx);
	//		add_sparse<T>(G, tdg_i, i);
	//		sub_sparse<T>(G, tdg_adj, i);
	//	}
	//}
	Mesh<T>& mesh_ref = source_->get_ref_mesh();
	ROWMAT(int)& face = mesh_ref.get_face();
	ROWMAT(T) verts = mesh_ref.get_verts();

	Face2Faces& f2f = mesh_ref.get_f2f();

	const int row_size = mesh_ref.face_size();
	const int col_size = mesh_ref.verts_size() + mesh_ref.face_size(); // normal_vert_num + vert_num
	
	int neighbor_tri_num = 0;
	std::for_each(f2f.faceidx_.begin(), f2f.faceidx_.end(),
		[&neighbor_tri_num](std::vector<int>& neighbor_list) {
			neighbor_tri_num += neighbor_list.size();
		});

	// axis x n_face x per_face_mat_size x row_size, axis x col_size
	Sparse<T> G(3 * (3 * neighbor_tri_num), 3 * col_size);
	G.setZero();
	

	int tri_pos = 0;
	for (int i = 0; i < f2f.faceidx_.size(); i++) {
		TriangleDeformationGradient<T, SIZE> tdg_i = source_->get_inv_matrix(i); // Triangle i 
		for (int n = 0; n < f2f.faceidx_[i].size(); n++) {
			const int adj_face_idx = f2f.faceidx_[i][n];
			TriangleDeformationGradient<T, SIZE>& tdg_adj = source_->get_inv_matrix(adj_face_idx);

			add_sparse<T>(G, tdg_i, tri_pos);
			sub_sparse<T>(G, tdg_adj, tri_pos);
			tri_pos++;
		}
	}
	assert(tri_pos == neighbor_tri_num && "tri num size... check");
	// return 
	// G , b(zeros like row size of G)
	return std::make_pair(G, ROWMAT(T)::Zero(G.rows(), 1));
}

template <typename T>
A_and_b<T>  DeformationTransfer<T>::produce_identity(){
	Mesh<T>& mesh_ref = source_->get_ref_mesh();
	ROWMAT(int)& face = mesh_ref.get_face();
	ROWMAT(T) verts = mesh_ref.get_verts();


	const int row_size = mesh_ref.face_size();
	const int col_size = mesh_ref.verts_size() + mesh_ref.face_size(); // normal_vert_num + vert_num

	

	Sparse<T> G(3 * (3 * row_size), 3 * col_size);
	G.setZero();
	
	for (int i = 0; i < row_size; i++) {
		TriangleDeformationGradient<T, SIZE>& invV_i = source_->get_inv_matrix(i); // Triangle i 
		add_sparse<T>(G, invV_i, i);
	}
	
	
	ROWMAT(T) b = ROWMAT(T)::Zero(G.rows(), 1);
	ROWMAT(T) eye = ROWMAT(T)::Identity(3, 3);


	//[1, 0, 0]^T
	// ...
	//[0, 1, 0]^T
	// ...
	//[0, 0, 1]^T
	for (int i = 0; i < row_size; i++){
		b.block<3,1>(0 * 3 * row_size + 3 * i, 0) = eye.row(0).transpose();
		b.block<3,1>(1 * 3 * row_size + 3 * i, 0) = eye.row(1).transpose();
		b.block<3,1>(2 * 3 * row_size + 3 * i, 0) = eye.row(2).transpose();
	}
	PRETTY_LOG_BEGIN("block_b")
	log_dense(b)
	PRETTY_LOG_END("block_b")

	return std::make_pair(G, b);
}

std::ostream& operator <<(std::ostream& os, std::vector< std::tuple<int, std::vector<int>>>& ss) {
	
	for (auto ssi = ss.begin(); ssi != ss.end(); ssi++) {
		std::cout << std::get<0>(*ssi) << ":";
		for (auto vi = std::get<1>(*ssi).begin(); vi != std::get<1>(*ssi).end(); vi++) {
			std::cout << *vi<< ", ";
		}
		std::cout << std::endl;

	}
	return os;
}

template <typename T>
A_and_b<T>  DeformationTransfer<T>::produce_closest()
{

	Mesh<T>& mesh_ref = src_copy_;
	ROWMAT(int)& face = mesh_ref.get_face();
	ROWMAT(T) verts = mesh_ref.get_verts();
	Face2Faces& f2f = mesh_ref.get_f2f();
	

	const int row_size = mesh_ref.face_size();
	const int col_size = mesh_ref.verts_size() + mesh_ref.face_size(); // normal_vert_num + vert_num


	std::vector<int> closest;
	closest.resize(src_copy_.verts_size());
	std::fill(closest.begin(), closest.end(), -1);
	
	ROWMAT(T) tgt_v = target_->get_ref_mesh().get_verts();
	ROWMAT(T) tgt_vn = target_->get_ref_mesh().calc_vertex_normal_vector();

	ROWMAT(T) src_v = src_copy_.get_verts();
	ROWMAT(T) src_vn = src_copy_.calc_vertex_normal_vector();

	const int default_k = 200;
	const int K = std::min(default_k, target_->get_v_size());
	

	//search nearest point-map
	for (int i = 0; i < source_->get_v_size(); i++) {
		std::vector<int> knn_idx(K);
		std::vector<float> squared_distances(K);
		pcl::PointXYZ xyz;

		xyz.x = src_v.row(i)[0]; xyz.y = src_v.row(i)[1]; xyz.z = src_v.row(i)[2];
	
		const float max_angle = 90 * M_PI / 180; // to radian
		if (tgt_kdtree_.nearestKSearch(xyz , K, knn_idx, squared_distances) > 0) {
			//find nearest point which has less 8824 than 90 degree.
			auto idx_iter = std::find_if(knn_idx.begin(), knn_idx.end(), [&](int j) {
					float angle = std::acos(tgt_vn.row(j).dot(src_vn.row(i)));
					return (std::abs(angle) < max_angle);
				});

			if (idx_iter != knn_idx.end())
				closest[i] = *idx_iter; // (src_idx, tgt_idx)
		}
	}


		//remove invalid_closest_ points
		const int G_rows = std::count_if(closest.begin(), closest.end(), 
			[&](int idx) {
				return idx != -1;
			});
		
		Sparse<T> G(3 * G_rows, 3 * col_size);
		G.setZero();

		ROWMAT(T) b;
		b.resize(G.rows(), 1);

		
		int row_idx = 0;
		log_dense(tgt_v)
		for (int i = 0; i < closest.size(); i++) {
			if (closest[i] == -1)
				continue;
			G.coeffRef(0 * G_rows + row_idx, 0 * col_size + i) = 1.0;
			G.coeffRef(1 * G_rows + row_idx, 1 * col_size + i) = 1.0;
			G.coeffRef(2 * G_rows + row_idx, 2 * col_size + i) = 1.0;
			b(0 * G_rows + row_idx, 0) = tgt_v.row(closest[i])(0);
			b(1 * G_rows + row_idx, 0) = tgt_v.row(closest[i])(1);
			b(2 * G_rows + row_idx, 0) = tgt_v.row(closest[i])(2);

			log_sparse(G)
			log_dense(b)

			row_idx++;
		}

	return std::make_pair(G, b);
}

template <typename T>
void DeformationTransfer<T>::add_deformation_gradient_data(DeformationGradient<T, SIZE>& dg_source, DeformationGradient<T, SIZE>& dg_target)
{
	source_ = &dg_source;
	target_ = &dg_target;
}

template <typename T>
void DeformationTransfer<T>::add_constraint(ROWMAT(T) additional_mat)
{

}

template <typename T>
void DeformationTransfer<T>::process_correspondence()
{
	using res_mat = Eigen::Matrix<T, -1, -1, Eigen::RowMajor>;
	process_neighbor(); // do nada.

	//copy 
	src_copy_ = source_->get_ref_mesh(); // copy src mesh.



	//setup kdtree for target
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->resize(target_->get_v_size());
	ROWMAT(T) tgt_v = target_->get_ref_mesh().get_verts();
	for (int i = 0; i < cloud->size(); i++) {
		(*cloud)[i].x = tgt_v.row(i)[0];
		(*cloud)[i].y = tgt_v.row(i)[1];
		(*cloud)[i].z = tgt_v.row(i)[2];
	}
	tgt_kdtree_.setInputCloud(cloud);



	// setup matrix( after set once, it doesn't changed.)
	std::pair<Sparse<T>, ROWMAT(T)> S_terms = produce_smoothness();
	std::pair<Sparse<T>, ROWMAT(T)> I_terms = produce_identity();

	//add marker(hard constraint.. it means remove related columns from matrix A.)
	std::pair<Sparse<T>, ROWMAT(T)> reduced_S_terms = add_marker_constraint_to_matrix(S_terms);
	std::pair<Sparse<T>, ROWMAT(T)> reduced_I_terms = add_marker_constraint_to_matrix(I_terms);

	ROWMAT(T) first_estimated_x = phase1(reduced_S_terms, reduced_I_terms); // we use this x coooooooooord to find valid closest points~!
	src_copy_.update_v(recover_marker_points_to_result(first_estimated_x));
	ROWMAT(T) raw_x = phase2(reduced_S_terms, reduced_I_terms); // yeah-a now phase 2. 
	src_copy_.update_v(recover_marker_points_to_result(raw_x));	// attach removed marker from add_marker_constraint_to_matrix() term. :p
	//src_copy_.save_file("phase2-nd.obj");
	make_triangle_correspondence();  // finnally make corrs!! foooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo!!!!!!!!!!
}

template<typename T>
void 
get_closest_triangle(std::vector<int>& result ,
					const ROWMAT(T)& src_fn, const ROWMAT(T)& tgt_fn,
					const ROWMAT(T)& src_fv, const ROWMAT(T)& tgt_fv,
					T max_angle, T radius) {
	
	const int src_fv_size = src_fv.rows();

	const int tgt_fv_size = tgt_fv.rows();

	std::vector<int>& closest = result;
	closest.resize(src_fv_size);
	std::fill(closest.begin(), closest.end(), -1);

	pcl::KdTreeFLANN<pcl::PointXYZ> tgt_kdtree;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->resize(tgt_fv_size);


	for (int i = 0; i < cloud->size(); i++) {
		(*cloud)[i].x = tgt_fv.row(i)[0];
		(*cloud)[i].y = tgt_fv.row(i)[1];
		(*cloud)[i].z = tgt_fv.row(i)[2];
	}
	tgt_kdtree.setInputCloud(cloud);

	const int default_k = 500;
	const int K = std::min(default_k, src_fv_size);

	//search nearest point-map
	for (int i = 0; i < src_fv_size; i++) {
		std::vector<int> knn_idx(K);
		std::vector<float> squared_distances(K);
		pcl::PointXYZ xyz;

		xyz.x = src_fv.row(i)[0]; xyz.y = src_fv.row(i)[1]; xyz.z = src_fv.row(i)[2];

		//if (src_kdtree.nearestKSearch(xyz, K, knn_idx, squared_distances) > 0) {
		if (tgt_kdtree.radiusSearch(xyz, radius, knn_idx, squared_distances) > 0) {
			//find nearest point which has less  than 90 degree.
			auto idx_iter = std::find_if(knn_idx.begin(), knn_idx.end(), [&](int j) {
				//std::cout.precision(10);
				////Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
				//std::cout << "====" << i << "====" << std::endl;
				//std::cout << src_fn.row(i) << std::endl;
				//std::cout << j <<" : " << tgt_fn.row(j)<< std::endl;
				//std::cout << std::acos(src_fn.row(i).dot(tgt_fn.row(j))) << std::endl;
				float angle = std::acos(src_fn.row(i).dot(tgt_fn.row(j)));
				return (std::abs(angle) < max_angle);
				});

			if (idx_iter != knn_idx.end())
				closest[i] = *idx_iter; // (tgt_idx,src_idx)
		}
	}
}

float max_triangle_length() {
	return 10;
}


template<typename T>
void DeformationTransfer<T>::make_triangle_correspondence()
{
	ROWMAT(int)& face = src_copy_.get_face();
	ROWMAT(T) verts = src_copy_.get_centroid_tri();

	const int src_fv_size = src_copy_.face_size();

	const int tgt_fv_size = target_->get_ref_mesh().face_size();

	std::vector<int> closest;
	closest.resize(tgt_fv_size);
	std::fill(closest.begin(), closest.end(), -1);

	ROWMAT(T) tgt_fv = target_->get_ref_mesh().get_centroid_tri();
	ROWMAT(T) tgt_fn = target_->get_ref_mesh().calc_face_normal_vector();

	ROWMAT(T) src_fv = src_copy_.get_centroid_tri();// tri centroid point.
	ROWMAT(T) src_fn = src_copy_.calc_face_normal_vector(); // tri normal vector

	std::vector<int> src_result, tgt_result;
	src_result.resize(src_fv.size()); tgt_result.resize(tgt_fv.size());
	
	const T max_angle = 90 * M_PI / 180; // to radian

	const T factor = 2.0;
	const T radius = 2.4;

	get_closest_triangle<T>(src_result, src_fn, tgt_fn, src_fv, tgt_fv, max_angle, radius);
	get_closest_triangle<T>(tgt_result, tgt_fn, src_fn, tgt_fv, src_fv, max_angle, radius);
	
	//std::vector<int> tmp_vec1(src_result.size()), tmp_vec2(tgt_result.size());
	/*load_file_as_vector(tmp_vec1, "D:/lab/2022/mycode/smooth-contact-aware-facial-blendshape-trasnfer/examples/test1.txt");
	load_file_as_vector(tmp_vec2, "D:/lab/2022/mycode/smooth-contact-aware-facial-blendshape-trasnfer/examples/test2.txt");
	
	
	std::vector < std::tuple<int, int, int>> res1, res2;
	for (int im = 0; im < tmp_vec1.size(); im++) {
		if (src_result[im] != tmp_vec1[im])
			res1.emplace_back(im, src_result[im], tmp_vec1[im]);
	}
	for (int im = 0; im < tmp_vec1.size(); im++) {
		if (tgt_result[im] != tmp_vec2[im])
			res2.emplace_back(im, tgt_result[im], tmp_vec2[im]);
	}
	std::cout << "========" << std::endl;

	for (auto& x : res1) {
		std::cout << std::get<0>(x) << " : " << std::get<1>(x) << " , " << std::get<2>(x) << std::endl;;
	}
	std::cout << "========" << std::endl;
	for (auto& x : res2) {
		std::cout << std::get<0>(x) << " : " << std::get<1>(x) << " , " << std::get<2>(x) << std::endl;
	}*/

	/*src_result = tmp_vec1;
	tgt_result = tmp_vec2;*/



	int idx = 0;
	std::set<std::pair<int, int>> tmp_pair;
	std::for_each(src_result.begin(), src_result.end(), [&idx, &tmp_pair](auto x) {
		//for (auto tgt_id = x.begin(); tgt_id != x.end(); tgt_id++)
			tmp_pair.emplace(idx, x);
		idx++;
		});

	idx = 0;
	std::for_each(tgt_result.begin(), tgt_result.end(), [&idx, &tmp_pair](auto x) {
		//for (auto tgt_id = x.begin(); tgt_id != x.end(); tgt_id++)
			tmp_pair.emplace(x, idx);
		idx++;
		});
	  
	for (int tgt_tri_idx = 0; tgt_tri_idx < closest.size(); tgt_tri_idx++) {
		corr_tri_tgt_from_src.assign(tmp_pair.begin(), tmp_pair.end());
	}
}



template <typename T>
ROWMAT(T) DeformationTransfer<T>::phase1(A_and_b<T>& S_pair, A_and_b<T>& I_pair)
{
	Sparse<T> new_A; 
	ROWMAT(T) new_b;

	////Sparse<T> A = ws * s_term + wi * i_term + wc * c_term;
	concat_v_sparse<T>(new_A, ws_ * S_pair.first, wi_ * I_pair.first);
	concat_v_dense<T>(new_b, ws_ * S_pair.second, wi_ * I_pair.second);

	Sparse<T> AtA =  new_A.transpose().eval()* new_A;
	
	Eigen::Matrix<T, -1, -1> new_col_b = new_A.transpose() * new_b;
	Eigen::SparseLU<Sparse<T>, Eigen::COLAMDOrdering<int>> slvr;
	slvr.compute(AtA);
	
	Eigen::Matrix<T, -1,-1> cx = slvr.solve(new_col_b);
	Eigen::Map<Eigen::Matrix<T, -1, -1>> tmp_map(cx.data(), static_cast<int>(cx.size() / 3), 3);
	ROWMAT(T) reval = tmp_map;

	return reval;
}
template<typename T>
void 
concat_v_sparse(Sparse<T>& stacked, const Sparse<T>& top, const Sparse<T>& bottom)
{
	using StorageIndex = Eigen::Index;
	assert(top.cols() == bottom.cols());
	stacked.resize(top.rows() + bottom.rows(), top.cols());
	stacked.resizeNonZeros(top.nonZeros() + bottom.nonZeros());

	StorageIndex i = 0;

	for (StorageIndex col = 0; col < top.cols(); col++)
	{
		stacked.outerIndexPtr()[col] = i;
		
		for (StorageIndex j = top.outerIndexPtr()[col]; j < top.outerIndexPtr()[col + 1]; j++, i++)
		{
			stacked.innerIndexPtr()[i] = top.innerIndexPtr()[j];
			stacked.valuePtr()[i] = top.valuePtr()[j];
		}

		for (StorageIndex j = bottom.outerIndexPtr()[col]; j < bottom.outerIndexPtr()[col + 1]; j++, i++)
		{
			stacked.innerIndexPtr()[i] = (StorageIndex)top.rows() + bottom.innerIndexPtr()[j];
			stacked.valuePtr()[i] = bottom.valuePtr()[j];
		}
	}
	stacked.outerIndexPtr()[top.cols()] = i;


 }

template<typename T>
void concat_v_dense(ROWMAT(T)& reval, const ROWMAT(T)& A, const ROWMAT(T)& B) {
	assert(A.cols() == B.cols());
	reval.resize(A.rows() + B.rows(), A.cols());
	reval.block(0, 0, A.rows(), A.cols()) = A;
	reval.block(A.rows(), 0, B.rows(), B.cols()) = B;
}


template <typename T>
ROWMAT(T) DeformationTransfer<T>::phase2(A_and_b<T>& S_pair, A_and_b<T>& I_pair)
{
	Sparse<T> cons_A; //
	ROWMAT(T) cons_b;

	concat_v_sparse<T>(cons_A, ws_*S_pair.first, wi_*I_pair.first);
	//Sparse<T> A = ws * s_term + wi * i_term + wc * c_term;
	concat_v_dense<T>(cons_b, ws_ * S_pair.second, wi_ * I_pair.second);
	ROWMAT(T) reval;

	for (int i = 0; i < wc_.size(); i++) {
		A_and_b<T> c_pair = produce_closest();
		A_and_b<T> reduced_c_pair = add_marker_constraint_to_matrix(c_pair);

		Sparse<T> total_A;
		ROWMAT(T) new_b;
		concat_v_sparse<T>(total_A, cons_A, wc_[i] * reduced_c_pair.first);
		concat_v_dense<T>(new_b, cons_b, wc_[i] * reduced_c_pair.second);

		Eigen::SparseLU<Sparse<T>, Eigen::COLAMDOrdering<int>> slvr;
		slvr.compute(total_A.transpose().eval() * total_A);
		Eigen::Matrix<T, -1, -1> cx = slvr.solve(total_A.transpose() * new_b);
		Eigen::Map<Eigen::Matrix<T, -1, -1>> tmp_map(cx.data(), static_cast<int>(cx.size() / 3), 3);
		reval = tmp_map;
		
		src_copy_.update_v(recover_marker_points_to_result(reval));
		src_copy_.save_file("phase2-"+ std::to_string(i) + ".obj");
	}
	return reval;

}



template<typename T>
A_and_b<T> DeformationTransfer<T>::add_marker_constraint_to_matrix(const A_and_b<T>& ab_pair)//, ROWMAT(T)& rhs_constraints)
{
	std::set<int> marker_set;
	std::for_each(marker_index_.begin(), marker_index_.end(), [&marker_set](std::tuple<int,int> tp) {marker_set.insert(std::get<0>(tp)); });

	const Sparse<T>& org_A = ab_pair.first;
	const ROWMAT(T)& org_b = ab_pair.second;
	
	const int marker_num = marker_index_.size();
	const int A_cols_num = org_A.cols()/3;

	// new A and b
	Sparse<T> reduced_A(org_A.rows(), org_A.cols() - marker_num*3);
	reduced_A.setZero();

	ROWMAT(T) hc_x = ROWMAT(T)::Zero(org_A.cols(), 1);
	ROWMAT(T) tgt_V = target_->get_ref_mesh().get_verts();
	
	// layout
	// x
	// y
	// z
	for(int i = 0 ; i < marker_index_.size() ; i++){
		const int src_idx = std::get<0>(marker_index_[i]);
		const int tgt_idx = std::get<1>(marker_index_[i]);
		hc_x(0 * A_cols_num + src_idx, 0) = tgt_V(tgt_idx, 0);
		hc_x(1 * A_cols_num + src_idx, 0) = tgt_V(tgt_idx, 1);
		hc_x(2 * A_cols_num + src_idx, 0) = tgt_V(tgt_idx, 2);

	}

	// A*x = b
	// reducedA*x + A*hc_x = b
	// reducedA*x = b - A*hc_x
	//-org_A.dot(hc_x) + org_b;
	//ROWMAT(T) new_b =  // rhs
	ROWMAT(T) new_b = -org_A * hc_x + org_b;  // rhs
	 
	

	int new_idx=0;
	const int redA_cols = static_cast<int>(reduced_A.cols()/3);
	for (int i = 0; i < A_cols_num; i++) {
		if (marker_set.count(i)) {
			continue;
		}
		else {
			reduced_A.col(0 * redA_cols + new_idx) = org_A.col(0 * A_cols_num + i);
			reduced_A.col(1 * redA_cols + new_idx) = org_A.col(1 * A_cols_num + i);
			reduced_A.col(2 * redA_cols + new_idx) = org_A.col(2 * A_cols_num + i);
			new_idx++;
		}
	}
	
	
	//column_subset.col(j) = m.col(indices[j]);

	

	return std::make_pair(reduced_A, new_b);
}

template<typename T>
ROWMAT(T) DeformationTransfer<T>::recover_marker_points_to_result(const ROWMAT(T)& x)
{
	ROWMAT(T) tgt_V = target_->get_ref_mesh().get_verts();
	ROWMAT(T) src_V = source_->get_ref_mesh().get_verts();
	const int v_size = source_->get_ref_mesh().verts_size();
	std::set<int> marker_set;
	std::for_each(marker_index_.begin(), marker_index_.end(), [&marker_set](std::tuple<int, int> tp) {marker_set.insert(std::get<0>(tp)); });

	const int src_rows = src_V.rows();
	ROWMAT(T) new_x(src_rows, 3);
	
	int j = 0;
	for (int i = 0; i < src_rows; i++) {
		if (marker_set.count(i)) {
			continue;
		}
		else {
			new_x.row(i) = x.row(j++);
		}
	}
	
	for (int i = 0; i < marker_index_.size(); i++) {
		const int src_idx = std::get<0>(marker_index_[i]);
		const int tgt_idx = std::get<1>(marker_index_[i]);
		new_x.row(src_idx) = tgt_V.row(tgt_idx);
	}

	return new_x.block(0, 0, v_size, 3);
}



template <typename T>
void DeformationTransfer<T>::compile()
{
	compile_flag_ = false;
	source_->compile();
	process_correspondence();

}
template <typename T>
void DeformationTransfer<T>::compile(T ws, T wi, std::vector<T>& wc)
{
	ws_ = ws;
	wi_ = wi;
	wc_ = wc;
	compile();
}

template <typename T>
void DeformationTransfer<T>::solve() {
	if (compile_flag_)
		compile();
	
	vert2Faces& v2f = target_->get_ref_mesh().get_v2f();
	int constraint_size = 0;
	std::for_each(v2f.vertidx_.begin(), v2f.vertidx_.end(),
		[&constraint_size](std::vector<int> fidx_list) {
			constraint_size += fidx_list.size();
		});
	
	const int tgt_v_size = target_->get_ref_mesh().verts_size();
	const int tgt_f_size = target_->get_ref_mesh().face_size();
	const int corr_tri_size = corr_tri_tgt_from_src.size();
	const int G_rows = 3 * (corr_tri_size + constraint_size);
	//const int G_rows = 3 *(corr_tri_size );
	const int G_cols = tgt_v_size + tgt_f_size;// vertex + normal size
	
	
	ROWMAT(T) test_b;
	load_row_mat(test_b);
	for (int ss = 0; ss < source_->size(); ss++) { // defored source size.

		Sparse<T> G(G_rows, G_cols);
		Eigen::Matrix<T, -1, -1> b(G_rows, 3);
		b.setZero();
		int tri_idx = 0;
		// build S - T 
		for (int i= 0; i < corr_tri_size; i++) {
			const int src_idx = corr_tri_tgt_from_src[i].first; // target and source inv T per triangle idx.
			const int tgt_idx = corr_tri_tgt_from_src[i].second; // target and source inv T per triangle idx.
			TriangleDeformationGradient<T, SIZE>& tdg_i = target_->get_inv_matrix(tgt_idx);

			
			Eigen::Map<ROWMAT(T)>& S_mat = source_->get_src2tgt_transform(ss, src_idx).get_mat();
			b.block<3, 3>(3*tri_idx, 0) = S_mat;
			//b.block<3, 3>(3*tri_idx, 0) = Eigen::Matrix<T,3,3>::Identity();
			//std::cout << S_mat << std::endl;
			add_sparse<T>(G, tdg_i, tri_idx, 1);
			tri_idx++;
		}
		

		// add constraints
		// T_j*v_i + d_j = T_k*v_i + d_k
		vert2Faces& v2f = target_->get_ref_mesh().get_v2f();
		for (int i = 0; i < v2f.vertidx_.size(); i++) {
			const int size = v2f.vertidx_[i].size();
			for (int j = 0; j < size; j++) {
				const int n_f_idx_prev = v2f.vertidx_[i][j]; // neighbor_face_idx1.
				const int n_f_idx_next = v2f.vertidx_[i][(j+1)%size]; // neighbor_face_idx2.
				TriangleDeformationGradient<T, SIZE>& tdg_i_prev = target_->get_inv_matrix(n_f_idx_prev);
				TriangleDeformationGradient<T, SIZE>& tdg_i_next = target_->get_inv_matrix(n_f_idx_next);
				add_sparse<T>(G, tdg_i_prev, tri_idx, 1);
				sub_sparse<T>(G, tdg_i_next, tri_idx, 1);
				tri_idx++;
			}
		}
		Sparse<T> AtA = G.transpose().eval() * G;
		Eigen::SparseLU<Sparse<T>, Eigen::COLAMDOrdering<int>> slvr;
		slvr.compute(AtA);
		Eigen::Matrix<T,-1,-1> cx = slvr.solve(G.transpose() * b);
		Eigen::Map<Eigen::Matrix<T, -1, -1>> tmp_map(cx.data(), static_cast<int>(cx.size() / 3), 3);
		ROWMAT(T) reval = tmp_map;

		Mesh<T> new_mesh(target_->get_ref_mesh());
		new_mesh.update_v(reval.block(0,0, tgt_v_size,3));
		new_mesh.save_file("transfered_" + std::to_string(ss)+".obj");
	}
}
