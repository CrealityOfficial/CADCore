#ifndef slic3r_Format_CYLINDER_hpp_
#define slic3r_Format_CYLINDER_hpp_
#include <functional>
#include "trimesh2/TriMesh.h"
#include "../interface.hpp"
#include "ccglobal/tracer.h"

namespace Slic3r {


CADCORE_API trimesh::TriMesh * create_cylinder(double radius, double height, int num_iter,  ccglobal::Tracer* tracer);



}; // namespace Slic3r

#endif /* slic3r_Format_STEP_hpp_ */
