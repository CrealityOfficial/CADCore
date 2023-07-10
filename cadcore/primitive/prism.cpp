#include "prism.h"
#include <assert.h>

#include "cadcore/internal/util.h"

#include "BRepPrimAPI_MakePrism.hxx"
#include "trimesh2/TriMesh_algo.h"

namespace cadcore
{
	trimesh::TriMesh* createPrism(double r)
	{
		BRepPrimAPI_MakePrism maker();
		//maker.Build();

		//assert(maker.IsDone());

		//const TopoDS_Shape& shape = maker.Shape();
		trimesh::TriMesh* mesh = new trimesh::TriMesh();

		if (mesh)
		{
			trimesh::xform xf = trimesh::xform::scale((double)r);
			trimesh::apply_xform(mesh, xf);
		}

		return mesh;
	}
}