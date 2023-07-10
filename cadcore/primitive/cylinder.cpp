
#include "cylinder.h"
#include <assert.h>
#include "BRepPrimAPI_MakeCylinder.hxx"
#include "cadcore/internal/util.h"


namespace cadcore 
{
    
    trimesh::TriMesh* create_cylinder(double radius, double height, int num_iter)
    {
        BRepPrimAPI_MakeCylinder maker(radius, height);
        maker.Build();

        assert(maker.IsDone());

        const TopoDS_Shape& shape = maker.Shape();
        trimesh::TriMesh* mesh = Shape_Triangulation(shape, 0.1);
    

        return mesh;
    }

}; // namespace Slic3r
