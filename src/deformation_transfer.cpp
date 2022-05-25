#include "deformation_trasnfer.h"
#include <cmath>
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
void add_sparse(Sparse<T>& A, TriangleDeformationGradient<T, 4>& tdg) {
	Eigen::Vector4i idx = tdg.get_ind();
	ROWMAT(T) invV_adj = tdg.get_mat();
	const int row_size = static_cast<int>(A.rows() / 3);
	const int col_size = static_cast<int>(A.cols() / 3);
	const int tri_num = tdg.tri_num_;
	//std::cout << "tri num" << tri_num << std::endl;
	//std::cout << "ind :" << tdg.get_ind().transpose() << std::endl;
	for (int ax = 0; ax < 3; ax++) {
		for (int i = 0; i < 3; i++) {
			//std::cout << ax << std::endl;
			//std::cout << "("
			//	<< row_size * ax + tri_num * 3 + i
			//	<< ","
			//	<< col_size * ax + tdg.get_ind()(0, 0)
			//	<< ") / "
			//	<< "("
			//	<< A.rows() << ", " << A.cols() << ")" << std::endl;
			A.coeffRef(row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(0, 0)) += (-invV_adj(i, 0) - invV_adj(i, 1) - invV_adj(i, 2));
			A.coeffRef(row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(1, 0)) += (invV_adj(i, 0));
			A.coeffRef(row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(2, 0)) += (invV_adj(i, 1));
			A.coeffRef(row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(3, 0)) += (invV_adj(i, 2));
		}
	}
	
}
template <typename T>
void sub_sparse(Sparse<T>& A, TriangleDeformationGradient<T, 4>& tdg) {
	Eigen::Vector4i idx = tdg.get_ind();
	ROWMAT(T) invV_adj = tdg.get_mat();
	const int row_size = static_cast<int>(A.rows() / 3);
	const int col_size = static_cast<int>(A.cols() / 3);
	const int tri_num = tdg.tri_num_;
	//std::cout << "tri num" <<  tri_num<<std::endl; 
	for (int ax = 0; ax < 3; ax++) {
		for (int i = 0; i < 3; i++) {
			//std::cout << ax << std::endl;
			//std::cout << "(" 
			//	<< row_size * ax + tri_num * 3 + i 
			//	<< "," 
			//	<< col_size * ax + tdg.get_ind()(0, 0) 
			//	<< ") / " 
			//	<< "(" 
			//	<< A.rows() << ", " << A.cols() << ")" << std::endl;
			A.coeffRef(row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(0, 0)) -= (-invV_adj(i, 0) - invV_adj(i, 1) - invV_adj(i, 2));
			A.coeffRef(row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(1, 0)) -= (invV_adj(i, 0));
			A.coeffRef(row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(2, 0)) -= (invV_adj(i, 1));
			A.coeffRef(row_size * ax + tri_num * 3 + i, col_size * ax + tdg.get_ind()(3, 0)) -= (invV_adj(i, 2));
		}
	}
}


template <typename T>
inline Sparse<T> DeformationTransfer<T>::produce_smoothness()
{
	Mesh<T>& mesh_ref = source_->get_ref_mesh();
	ROWMAT(int)& face = mesh_ref.get_face();
	ROWMAT(T)& verts = mesh_ref.get_verts();
	

	Face2Faces& f2f = mesh_ref.get_f2f();

	const int row_size = mesh_ref.face_size();
	const int col_size = mesh_ref.verts_size() + mesh_ref.face_size(); // normal_vert_num + vert_num



	Sparse<T> G(3*(3*row_size), 3*col_size);
	G.setZero();
	

	for (int i = 0; i < row_size; i++) {
		TriangleDeformationGradient<T, SIZE> tdg_i = source_->get_inv_matrix(i); // Triangle i 
		add_sparse<T>(G, tdg_i);
		
		for (auto iter = f2f.faceidx_[i].begin(); iter != f2f.faceidx_[i].end(); iter++) {
			const int adj_face_idx = *iter;
			TriangleDeformationGradient<T, SIZE>& tdg_adj = source_->get_inv_matrix(adj_face_idx);
			sub_sparse<T>(G, tdg_adj);
		}
	}
	return G;
}

template <typename T>
Sparse<T> DeformationTransfer<T>::produce_identity(){
	Mesh<T>& mesh_ref = source_->get_ref_mesh();
	ROWMAT(int)& face = mesh_ref.get_face();
	ROWMAT(T)& verts = mesh_ref.get_verts();


	Face2Faces& f2f = mesh_ref.get_f2f();

	const int row_size = mesh_ref.face_size();
	const int col_size = mesh_ref.verts_size() + mesh_ref.face_size(); // normal_vert_num + vert_num

	

	Sparse<T> G(3 * (3 * row_size), 3 * col_size);
	G.setZero();
	
	for (int i = 0; i < row_size; i++) {
		TriangleDeformationGradient<T, SIZE>& invV_i = source_->get_inv_matrix(i); // Triangle i 
		add_sparse<T>(G, invV_i);
	}
	
	
	ROWMAT(T) b;
	b.resize(G.rows(), 1);

	return G;
}

template <typename T>
Sparse<T> DeformationTransfer<T>::produce_closest()
{

	Mesh<T>& mesh_ref = source_->get_ref_mesh();
	ROWMAT(int)& face = mesh_ref.get_face();
	ROWMAT(T)& verts = mesh_ref.get_verts();
	Face2Faces& f2f = mesh_ref.get_f2f();

	const int row_size = mesh_ref.face_size();
	const int col_size = mesh_ref.verts_size() + mesh_ref.face_size(); // normal_vert_num + vert_num


	Sparse<T> G(3 * (3 * row_size), 3 * col_size);
	G.setZero();


	//
	std::vector<std::tuple<int, std::vector<int>>> closest;


	//setup kdtree
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	cloud->resize(target_->get_v_size());
	ROWMAT(T)& tgt_v = target_->get_ref_mesh().get_verts();
	ROWMAT(T) tgt_vn = target_->get_ref_mesh().calc_vertex_normal_vector();
	//ROWMAT(T) /*tgt_vn*/(2,3);

	for (int i = 0; i < cloud->size(); i++) {
		(*cloud)[i].x = tgt_v.row(i)[0];
		(*cloud)[i].y = tgt_v.row(i)[1];
		(*cloud)[i].z = tgt_v.row(i)[2];
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> src_kdtree_;
	src_kdtree_.setInputCloud(cloud);

	ROWMAT(T)& src_v = source_->get_ref_mesh().get_verts();
	ROWMAT(T) src_vn = source_->get_ref_mesh().calc_vertex_normal_vector();
	//ROWMAT(T) src_vn(2,3);
	std::cout << src_vn.size() << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr query_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	std::cout << src_vn.size() << std::endl;
   	cloud->resize(source_->get_v_size());
	
	const int default_k = 200;
	const int K = std::min(default_k, target_->get_v_size());
	//search nearest point-map
	for (int i = 0; i < source_->get_v_size(); i++) {
		std::vector<int> knn_idx(K);
		std::vector<float> squared_distances(K);
		pcl::PointXYZ xyz;

		xyz.x = src_v.row(i)[0]; xyz.y = src_v.row(i)[1]; xyz.z = src_v.row(i)[2];
		float angle;
		float max_angle = 90 * M_PI / 180; // to radian
		if (src_kdtree_.nearestKSearch(xyz , K, knn_idx, squared_distances) > 0) {
			for (int j = 0; j < K; j++) {
				angle = std::acos( tgt_vn.row(j).dot(src_vn.row(i)));
				if (std::abs(angle) < max_angle) {
					closest.emplace_back(i, j); // (src_idx, tgt_idx)
				}
			}	
		}
	}

	return G;
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
	process_neighbor();

	Sparse<T> s_term = produce_smoothness();
	Sparse<T> i_term = produce_identity();
	Sparse<T> c_term = produce_closest();

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
void DeformationTransfer<T>::solve()
{
	if (compile_flag_)
		compile();

	phase1();
	for (int i = 0; i < wc_.size(); i++) {
		phase1();
		phase2();
	}
}

template <typename T>
void DeformationTransfer<T>::phase1()
{

}

template <typename T>
void DeformationTransfer<T>::phase2()
{
}