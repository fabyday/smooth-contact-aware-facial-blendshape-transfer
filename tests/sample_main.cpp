#include <iostream>
#include <deformation_gradient.h>
#include <dg_solver.h>
#include <Mesh.h>


int main() {
	DGSolver slvr;
	
	Mesh s;
	s.load_from_file("");

	slvr.solve();

	return 0;

}