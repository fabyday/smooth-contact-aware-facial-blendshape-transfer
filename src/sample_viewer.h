#include <igl/opengl/glfw/Viewer.h>
#include <Mesh.h>
using PViewer = igl::opengl::glfw::Viewer;
// debugging purpose only
class Viewer {
	

public :
	Viewer() {

	};


	template <typename T>
	void show_normal(Mesh<T>& f){
	
		PViewer v("test");
		v.data().set_mesh(f.get_verts(), f.get_face());
		const RowVector3d black(0, 0, 0);
		v.data().add_edges(f.get_verts(), f.get_normal_verts());


		viewer.data().show_lines = true;
		viewer.launch();
		
		
		
	}
	


};