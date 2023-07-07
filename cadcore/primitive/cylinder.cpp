#include "fstream"
#include "cylinder.hpp"
#include "tbb/tbb.h"
#include <string>
#include "stl.h"
#include <Eigen/Geometry>
#include "trimesh2/TriMesh.h"
#ifdef _WIN32
#define DIR_SEPARATOR '\\'
#else
#define DIR_SEPARATOR '/'
#endif

#include "TopoDS_Solid.hxx"
#include "TopoDS_Compound.hxx"
#include "TopoDS_Builder.hxx"
#include "TopoDS.hxx"
#include "TopExp_Explorer.hxx"
#include "BRepPrimAPI_MakeCylinder.hxx"
#include "BRepPrimAPI_MakeBox.hxx"
#include "BRepMesh_Context.hxx"
#include "IMeshTools_Parameters.hxx"
#include "BRepMesh_FaceDiscret.hxx"
#include "BRepMesh_DelabellaMeshAlgoFactory.hxx"
#include "BRepMesh_IncrementalMesh.hxx"
#include "IMeshData_Types.hxx"
#include "IMeshData_Face.hxx"
#include "IMeshData_Edge.hxx"
#include "IMeshData_Wire.hxx"
#include "BRepMesh_ShapeTool.hxx"

const double STEP_TRANS_CHORD_ERROR = 0.0025;
const double STEP_TRANS_ANGLE_RES = 0.5;


namespace Slic3r {
    using Vec3f = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;
trimesh::TriMesh* create_cylinder(double radius, double height, int num_iter,  ccglobal::Tracer* tracer)
{
    stl_file  stl;
    TopoDS_Shape t_topo_cylinder = BRepPrimAPI_MakeCylinder(radius, height).Shape();

    gp_Ax2 boxPos;
    boxPos.SetLocation(gp_Pnt(10.0, 10.0, 10.0));

    TopoDS_Shape boxShape = BRepPrimAPI_MakeBox(boxPos, 15, 20, 30).Shape();
    //Handle(AIS_Shape) aCylinder = new AIS_Shape(C.Shape());
    int aNbNodes = 0;
    int aNbTriangles = 0;
   // TopoDS_Vertex Vertex = TopoDS::Vertex(t_topo_cylinder);
    //TopoDS_Face Face = TopoDS::Face(t_topo_cylinder);
    TopExp_Explorer aExp;
    int i = 1;
    for (aExp.Init(boxShape, TopAbs_EDGE); aExp.More(); aExp.Next(), ++i)
    {
        TopLoc_Location     aLoc;
       // const TopoDS_Face& aVertex = TopoDS::Face(aExp.Current());
       // Standard_Boolean aPnt = BRep_Tool::IsGeometric(aVertex);
       // std::cout << "Face " << aPnt << std::endl;
       // TopoDS_Face aFace = TopoDS::Face(aExp.Current());
       // Handle_Poly_Triangulation triFace = BRep_Tool::Triangulation(aFace, aLoc);

       //if (triFace.IsNull()) {
       //   // ++aNbFacesNoTri;
       //    throw std::logic_error{ std::string{"Could not get faces "} };//£¿£¿£¿£¿£¿£¿
       //}
        const TopoDS_Edge& aEdge = TopoDS::Edge(aExp.Current());
        Standard_Boolean aPnt = BRep_Tool::IsGeometric(aEdge);
        Standard_Real aPnt2 = BRep_Tool::Tolerance(aEdge);
        std::cout << "Face " << aPnt << std::endl;
        std::cout << "Face2 " << aPnt2 << std::endl;



       //const TopoDS_Vertex& aVertex = TopoDS::Vertex(aExp.Current());
       //  gp_Pnt aPnt = BRep_Tool:: Pnt(aVertex);
       // std::cout << "vertex " << i << ": (" << aPnt.X() << ", " << aPnt.Y()
       //     << ", " << aPnt.Z() << "), vertex's hash code is "
       //     << aVertex.HashCode(INT_MAX) << std::endl;
        
        //TopLoc_Location     aLoc;
        //const TopoDS_Face& aFace = TopoDS::Face(aExp.Current());
        //Handle(Poly_Triangulation) aTriangulation = BRep_Tool::Triangulation(TopoDS::Face(aFace), aLoc);
        //if (!aTriangulation.IsNull()) {
        //    aNbNodes += aTriangulation->NbNodes();
        //    aNbTriangles += aTriangulation->NbTriangles();
        //}
    }
   


    IMeshTools_Parameters aMeshParams;
    Handle(IMeshTools_Context) aContext = new BRepMesh_Context();
    aContext->SetFaceDiscret(new BRepMesh_FaceDiscret(new BRepMesh_DelabellaMeshAlgoFactory()));
    BRepMesh_IncrementalMesh aMesher;
    aMesher.SetShape(boxShape);
    aMesher.ChangeParameters() = aMeshParams;
    aMesher.Perform(aContext);
    Handle(IMeshData_Model)   theModel =  aContext.get()->GetModel();


    Handle(NCollection_IncAllocator) aTmpAlloc(new NCollection_IncAllocator(IMeshData::MEMORY_BLOCK_SIZE_HUGE));
    NCollection_Map<IMeshData_Face*> aUsedFaces(1, aTmpAlloc);
    for (Standard_Integer aEdgeIt = 0; aEdgeIt < theModel->EdgesNb(); ++aEdgeIt)
    {
        const IMeshData::IEdgeHandle& aDEdge = theModel->GetEdge(aEdgeIt);
        if (aDEdge->IsFree())
        {
            if (aDEdge->IsSet(IMeshData_Outdated))
            {
                TopLoc_Location aLoc;
                BRep_Tool::Polygon3D(aDEdge->GetEdge(), aLoc);
                //BRepMesh_ShapeTool::NullifyEdge(aDEdge->GetEdge(), aLoc);
            }

            continue;
        }

        for (Standard_Integer aPCurveIt = 0; aPCurveIt < aDEdge->PCurvesNb(); ++aPCurveIt)
        {
            // Find adjacent outdated face.
            const IMeshData::IFaceHandle aDFace = aDEdge->GetPCurve(aPCurveIt)->GetFace();
            if (!aUsedFaces.Contains(aDFace.get()))
            {
                aUsedFaces.Add(aDFace.get());
                if (aDFace->IsSet(IMeshData_Outdated))
                {
                    TopLoc_Location aLoc;
                    const Handle(Poly_Triangulation)& aTriangulation =
                        BRep_Tool::Triangulation(aDFace->GetFace(), aLoc);

                    // Clean all edges of oudated face.
                    for (Standard_Integer aWireIt = 0; aWireIt < aDFace->WiresNb(); ++aWireIt)
                    {
                        const IMeshData::IWireHandle& aDWire = aDFace->GetWire(aWireIt);
                        for (Standard_Integer aWireEdgeIt = 0; aWireEdgeIt < aDWire->EdgesNb(); ++aWireEdgeIt)
                        {
                            const IMeshData::IEdgeHandle aTmpDEdge = aDWire->GetEdge(aWireEdgeIt);
                           // BRepMesh_ShapeTool::NullifyEdge(aTmpDEdge->GetEdge(), aTriangulation, aLoc);
                        }
                    }

                   // BRepMesh_ShapeTool::NullifyFace(aDFace->GetFace());
                }
            }
        }
    }

    if (tracer && stl.facet_start.size() > 0)
    {

        tracer->progress(1.0f);
        tracer->success();
    }
    if (tracer && stl.facet_start.size() == 0)
        tracer->failed("Parse File Error.");

    trimesh::TriMesh * tm = new  trimesh::TriMesh();

        for (int i = 0; i < stl.facet_start.size(); i++)
        {
            stl_facet f =  stl.facet_start.at(i);
            stl_normal n = f.normal;
            trimesh::TriMesh::Face tf;  
            for (int j = 0; j < 3; j++)
            {
                trimesh::point p(f.vertex[j].x(), f.vertex[j].y(), f.vertex[j].z());
                tm->vertices.emplace_back(p);
                tf.at(j) = (j + 3 * i);
            }
            tm->faces.emplace_back(tf);
            tm->normals.emplace_back(trimesh::vec(n.x(), n.y(), n.z()));
        }

    

    return tm;
}

}; // namespace Slic3r
