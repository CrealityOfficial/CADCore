#include "fstream"
#include "STEP.hpp"
#include "tbb/tbb.h"
#include <string>
#include <Eigen/Geometry>
#include "trimesh2/TriMesh.h"
#ifdef _WIN32
#define DIR_SEPARATOR '\\'
#else
#define DIR_SEPARATOR '/'
#endif

#include "STEPCAFControl_Reader.hxx"
#include "BRepMesh_IncrementalMesh.hxx"
#include "Interface_Static.hxx"
#include "XCAFDoc_DocumentTool.hxx"
#include "XCAFDoc_ShapeTool.hxx"
#include "XCAFApp_Application.hxx"
#include "TopoDS_Solid.hxx"
#include "TopoDS_Compound.hxx"
#include "TopoDS_Builder.hxx"
#include "TopoDS.hxx"
#include "TDataStd_Name.hxx"
#include "BRepBuilderAPI_Transform.hxx"
#include "TopExp_Explorer.hxx"
#include "TopExp_Explorer.hxx"
#include "BRep_Tool.hxx"

#include "XCAFDoc_ColorType.hxx"
#include "Quantity_Color.hxx"
#include "XCAFDoc_DocumentTool.hxx"
#include "XCAFDoc_ColorTool.hxx"
#include "Quantity_ColorRGBA.hxx"
#include "TopTools_IndexedMapOfShape.hxx"
#include <map>
const double STEP_TRANS_CHORD_ERROR = 0.0025;
const double STEP_TRANS_ANGLE_RES = 0.5;


namespace Slic3r {

    class  Color
    {
    public:
        /**
         * Defines the color as (R,G,B,A) whereas all values are in the range [0,1].
         * \a A defines the transparency.
         */
        explicit Color(float R = 0.0, float G = 0.0, float B = 0.0, float A = 0.0)
            :r(R), g(G), b(B), a(A) {}
        /**
         * Does basically the same as the constructor above unless that (R,G,B,A) is
         * encoded as an unsigned int.
         */
        Color(uint32_t rgba)
        {
            setPackedValue(rgba);
        }
        /** Copy constructor. */
        Color(const Color& c)
            :r(c.r), g(c.g), b(c.b), a(c.a) {}
        /** Returns true if both colors are equal. Therefore all components must be equal. */
        bool operator==(const Color& c) const
        {
            return getPackedValue() == c.getPackedValue();
            //return (c.r==r && c.g==g && c.b==b && c.a==a);
        }
        bool operator!=(const Color& c) const
        {
            return !operator==(c);
        }
        /**
         * Defines the color as (R,G,B,A) whereas all values are in the range [0,1].
         * \a A defines the transparency, 0 means complete opaque and 1 invisible.
         */
        void set(float R, float G, float B, float A = 0.0)
        {
            r = R; g = G; b = B; a = A;
        }
        Color& operator=(const Color& c)
        {
            r = c.r; g = c.g; b = c.b; a = c.a;
            return *this;
        }
        /**
         * Sets the color value as a 32 bit combined red/green/blue/alpha value.
         * Each component is 8 bit wide (i.e. from 0x00 to 0xff), and the red
         * value should be stored leftmost, like this: 0xRRGGBBAA.
         *
         * \sa getPackedValue().
         */
        Color& setPackedValue(uint32_t rgba)
        {
            this->set((rgba >> 24) / 255.0f,
                ((rgba >> 16) & 0xff) / 255.0f,
                ((rgba >> 8) & 0xff) / 255.0f,
                (rgba & 0xff) / 255.0f);
            return *this;
        }
        /**
         * Returns color as a 32 bit packed unsigned int in the form 0xRRGGBBAA.
         *
         *  \sa setPackedValue().
         */
        uint32_t getPackedValue() const
        {
            return ((uint32_t)(r * 255.0f + 0.5f) << 24 |
                (uint32_t)(g * 255.0f + 0.5f) << 16 |
                (uint32_t)(b * 255.0f + 0.5f) << 8 |
                (uint32_t)(a * 255.0f + 0.5f));
        }
        /**
         * creates FC Color from template type, e.g. Qt QColor
         */
        template <typename T>
        void setValue(const T& q)
        {
            set(q.redF(), q.greenF(), q.blueF());
        }
        /**
         * returns a template type e.g. Qt color equivalent to FC color
         *
         */
        template <typename T>
        inline T asValue(void) const {
            return(T(int(r * 255.0), int(g * 255.0), int(b * 255.0)));
        }
        /**
         * returns color as hex color "#RRGGBB"
         *
         */
        std::string asHexString() const {
            std::stringstream ss;
            ss << "#" << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << int(r * 255.0)
                << std::setw(2) << int(g * 255.0)
                << std::setw(2) << int(b * 255.0);
            return ss.str();
        }
        /**
         * \deprecated
         */
        std::string asCSSString() const {
            return asHexString();
        }
        /**
         * gets color from hex color "#RRGGBB"
         *
         */
        bool fromHexString(const std::string& hex) {
            if (hex.size() < 7 || hex[0] != '#')
                return false;
            // #RRGGBB
            if (hex.size() == 7) {
                std::stringstream ss(hex);
                unsigned int rgb;
                char c;

                ss >> c >> std::hex >> rgb;
                int rc = (rgb >> 16) & 0xff;
                int gc = (rgb >> 8) & 0xff;
                int bc = rgb & 0xff;

                r = rc / 255.0f;
                g = gc / 255.0f;
                b = bc / 255.0f;

                return true;
            }
            // #RRGGBBAA
            if (hex.size() == 9) {
                std::stringstream ss(hex);
                unsigned int rgba;
                char c;

                ss >> c >> std::hex >> rgba;
                int rc = (rgba >> 24) & 0xff;
                int gc = (rgba >> 16) & 0xff;
                int bc = (rgba >> 8) & 0xff;
                int ac = rgba & 0xff;

                r = rc / 255.0f;
                g = gc / 255.0f;
                b = bc / 255.0f;
                a = ac / 255.0f;

                return true;
            }

            return false;
        }

        /// color values, public accessible
        float r, g, b, a;
    };

//bool StepPreProcessor::preprocess(const char* path, std::string &output_path)
//{
//    boost::nowide::ifstream infile(path);
//    if (!infile.good()) {
//        throw Slic3r::RuntimeError(std::string("Load step file failed.\nCannot open file for reading.\n"));
//        return false;
//    }
//
//    boost::filesystem::path temp_path(temporary_dir());
//    std::string temp_step_path = temp_path.string() + "/temp.step";
//    boost::nowide::remove(temp_step_path.c_str());
//    boost::nowide::ofstream temp_file(temp_step_path, std::ios::app);
//    std::string temp_line;
//    while (std::getline(infile, temp_line)) {
//        if (m_encode_type == EncodedType::UTF8) {
//            //BBS: continue to judge whether is other type
//            if (isUtf8(temp_line)) {
//                //BBS: do nothing, but must be checked before checking whether is GBK
//            }
//            //BBS: not utf8, then maybe GBK
//            else if (isGBK(temp_line)) {
//                m_encode_type = EncodedType::GBK;
//            }
//            //BBS: not UTF8 and not GBK, then maybe some kind of special encoded type which we can't handle
//            // Load the step as UTF and user will see garbage characters in slicer but we have no solution at the moment
//            else {
//                m_encode_type = EncodedType::OTHER;
//            }
//        }
//        if (m_encode_type == EncodedType::GBK)
//            //BBS: transform to UTF8 format if is GBK
//            //todo: use gbkToUtf8 function to replace
//            temp_file << decode_path(temp_line.c_str()) << std::endl;
//        else
//            temp_file << temp_line.c_str() << std::endl;
//    }
//    temp_file.close();
//    infile.close();
//    if (m_encode_type == EncodedType::GBK) {
//        output_path = temp_step_path;
//    } else {
//        boost::nowide::remove(temp_step_path.c_str());
//        output_path = std::string(path);
//    }
//
//    return true;
//}

bool StepPreProcessor::isUtf8File(const char* path)
{
    std::ifstream infile(path);
    if (!infile.good()) {
        //throw Slic3r::RuntimeError(std::string("Load step file failed.\nCannot open file for reading.\n"));
        return false;
    }

    std::string temp_line;
    while (std::getline(infile, temp_line)) {
        if (!isUtf8(temp_line)) {
            infile.close();
            return false;
        }
    }

    infile.close();
    return true;
}

bool StepPreProcessor::isUtf8(const std::string str)
{
    size_t num = 0;
    int i = 0;
    while (i < str.length()) {
        if ((str[i] & 0x80) == 0x00) {
            i++;
        } else if ((num = preNum(str[i])) > 2) {
            i++;
            for (int j = 0; j < num - 1; j++) {
                if ((str[i] & 0xc0) != 0x80)
                    return false;
                i++;
            }
        } else {
            return false;
        }
    }
    return true;
}

bool StepPreProcessor::isGBK(const std::string str) {
    size_t i = 0;
    while (i < str.length()) {
        if (str[i] <= 0x7f) {
            i++;
            continue;
        } else {
            if (str[i] >= 0x81 &&
                str[i] <= 0xfe &&
                str[i + 1] >= 0x40 &&
                str[i + 1] <= 0xfe &&
                str[i + 1] != 0xf7) {
                i += 2;
                continue;
            }
            else {
                return false;
            }
        }
    }
    return true;
}

int StepPreProcessor::preNum(const unsigned char byte) {
    unsigned char mask = 0x80;
    int num = 0;
    for (int i = 0; i < 8; i++) {
        if ((byte & mask) == mask) {
            mask = mask >> 1;
            num++;
        } else {
            break;
        }
    }
    return num;
}

struct NamedSolid {
    NamedSolid(const TopoDS_Shape& s,
               const std::string& n) : solid{s}, name{n} {}
    const TopoDS_Shape solid;
    const std::string  name;
};

static void getNamedSolids(const TopLoc_Location& location, const std::string& prefix,
                           unsigned int& id, const Handle(XCAFDoc_ShapeTool) shapeTool,
                           const TDF_Label label, std::vector<NamedSolid>& namedSolids) {
    TDF_Label referredLabel{label};
    if (shapeTool->IsReference(label))
        shapeTool->GetReferredShape(label, referredLabel);

    std::string name;
    Handle(TDataStd_Name) shapeName;
    if (referredLabel.FindAttribute(TDataStd_Name::GetID(), shapeName))
        name = TCollection_AsciiString(shapeName->Get()).ToCString();

    if (name == "")
        name = std::to_string(id++);
    std::string fullName{name};

    TopLoc_Location localLocation = location * shapeTool->GetLocation(label);
    TDF_LabelSequence components;
    if (shapeTool->GetComponents(referredLabel, components)) {
        for (Standard_Integer compIndex = 1; compIndex <= components.Length(); ++compIndex) {
            getNamedSolids(localLocation, fullName, id, shapeTool, components.Value(compIndex), namedSolids);
        }
    } else {
        TopoDS_Shape shape;
        shapeTool->GetShape(referredLabel, shape);
        TopAbs_ShapeEnum shape_type = shape.ShapeType();
        BRepBuilderAPI_Transform transform(shape, localLocation, Standard_True);
        switch (shape_type) {
        case TopAbs_COMPOUND:
            namedSolids.emplace_back(TopoDS::Compound(transform.Shape()), fullName);
            break;
        case TopAbs_COMPSOLID:
            namedSolids.emplace_back(TopoDS::CompSolid(transform.Shape()), fullName);
            break;
        case TopAbs_SOLID:
            namedSolids.emplace_back(TopoDS::Solid(transform.Shape()), fullName);
            break;
        default:
            break;
        }
    }
}
using Vec3f = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;
trimesh::TriMesh* load_step(const char *path,  ccglobal::Tracer* tracer)
{
    bool cb_cancel = false;
    //if (stepFn) {
    //    stepFn(LOAD_STEP_STAGE_READ_FILE, 0, 1, cb_cancel);
    //    is_cancel = cb_cancel;
    //    if (cb_cancel) {
    //        return false;
    //    }
    //}

    //if (!StepPreProcessor::isUtf8File(path) && isUtf8Fn)
    //    isUtf8Fn(false);
    std::string file_after_preprocess = std::string(path);

    std::vector<NamedSolid> namedSolids;
    std::vector<TopLoc_Location> namedLables;
    Handle(TDocStd_Document) document;
    Handle(XCAFApp_Application) application = XCAFApp_Application::GetApplication();
    application->NewDocument(file_after_preprocess.c_str(), document);
    STEPCAFControl_Reader reader;
    reader.SetNameMode(true);
    //BBS: Todo, read file is slow which cause the progress_bar no update and gui no response
    IFSelect_ReturnStatus stat = reader.ReadFile(file_after_preprocess.c_str());
    if (stat != IFSelect_RetDone || !reader.Transfer(document)) {
        application->Close(document);
        throw std::logic_error{ std::string{"Could not read '"} + path + "'" };
        
    }
    bool interuptted = false;


    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
    TDF_LabelSequence topLevelShapes;
    shapeTool->GetFreeShapes(topLevelShapes);
    int AXXX = topLevelShapes.Length();
    unsigned int id{1};
    Standard_Integer topShapeLength = topLevelShapes.Length() + 1;
    auto stage_unit2 = topShapeLength / LOAD_STEP_STAGE_UNIT_NUM + 1;

    Handle(XCAFDoc_ColorTool) rootColorTool = XCAFDoc_DocumentTool::ColorTool(document->Main());
    TDF_Label  lab(document->Main().FindChild(0,Standard_True));
    //Quantity_ColorRGBA col();
    //(const Quantity_Color & col, TDF_Label & lab) const;
    Quantity_ColorRGBA colRGBA;
    Quantity_Color col;
    //lab = rootColorTool->FindColor(col);
    //rootColorTool.GetColor();


    TDF_LabelSequence tdfLabels;
    shapeTool->GetShapes(tdfLabels);   //获取装配体和组件对应名称
    int Roots = tdfLabels.Length();
    Color color(0.8f, 0.8f, 0.8f);
    std::map <std::string, Color> colors;
    //std::vector<Color> faceColors;
    for (int i = 1; i <= Roots; i++)
    {
        TDF_Label label = tdfLabels.Value(i);
        bool colorcolor = rootColorTool->IsSet(label, XCAFDoc_ColorGen);
        bool colorcolor1 = rootColorTool->IsSet(label, XCAFDoc_ColorSurf);
        bool colorcolor2 = rootColorTool->IsSet(label, XCAFDoc_ColorCurv);
        //Standard_Boolean GetColor(const TopoDS_Shape & S, const XCAFDoc_ColorType type, TDF_Label & colorL);
        Standard_Boolean xxsss5 = rootColorTool->GetColor(label, XCAFDoc_ColorGen, col);
        Standard_Boolean xxsss6 = rootColorTool->GetColor(label, XCAFDoc_ColorSurf, col);
        Standard_Boolean xxsss7 = rootColorTool->GetColor(label, XCAFDoc_ColorCurv, col);
        color.r = (float)col.Red();
        color.g = (float)col.Green();
        color.b = (float)col.Blue();


        TDF_Label referredLabel{ label };
        if (shapeTool->IsReference(label))
            shapeTool->GetReferredShape(label, referredLabel);

        std::string name;
        Handle(TDataStd_Name) shapeName;
        if (referredLabel.FindAttribute(TDataStd_Name::GetID(), shapeName))
            name = TCollection_AsciiString(shapeName->Get()).ToCString();

        if (name == "")
            name = std::to_string(id++);
        
        colors.emplace (name, color);
    }

    for (Standard_Integer iLabel = 1; iLabel < topShapeLength; ++iLabel) {
        //if (stepFn) {
        //    if ((iLabel % stage_unit2) == 0) {
        //        stepFn(LOAD_STEP_STAGE_GET_SOLID, iLabel, topShapeLength, cb_cancel);
        //        is_cancel = cb_cancel;
        //    }
        //    if (cb_cancel) {
        //        shapeTool.reset(nullptr);
        //        application->Close(document);
        //        return false;
        //    }
        //}
        getNamedSolids(TopLoc_Location{}, "", id, shapeTool, topLevelShapes.Value(iLabel), namedSolids);
    }

    std::vector<Color> color_submodule;
    color_submodule.resize(namedSolids.size());
    if (tracer)
        tracer->progress(0.1f);
    trimesh::TriMesh* tm = new  trimesh::TriMesh();

    //std::vector<stl_file> stl;
    //stl.resize(namedSolids.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, namedSolids.size()), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t i = range.begin(); i < range.end(); i++) {
            if (tracer)
                tracer->formatMessage("range %d", (int)i);
            BRepMesh_IncrementalMesh mesh(namedSolids[i].solid, STEP_TRANS_CHORD_ERROR, false, STEP_TRANS_ANGLE_RES, true);
            // BBS: calculate total number of the nodes and triangles
            int aNbNodes     = 0;
            int aNbTriangles = 0;
            for (TopExp_Explorer anExpSF(namedSolids[i].solid, TopAbs_FACE); anExpSF.More(); anExpSF.Next()) {
                TopLoc_Location aLoc;
                Handle(Poly_Triangulation) aTriangulation = BRep_Tool::Triangulation(TopoDS::Face(anExpSF.Current()), aLoc);
                if (!aTriangulation.IsNull()) {
                    aNbNodes += aTriangulation->NbNodes();
                    aNbTriangles += aTriangulation->NbTriangles();
                }
            }

            if (aNbTriangles == 0 || aNbNodes == 0)
                // BBS: No triangulation on the shape.
                continue;
            std::vector<Vec3f> points;
            points.reserve(aNbNodes);
            // BBS: count faces missing triangulation
            Standard_Integer aNbFacesNoTri = 0;
            // BBS: fill temporary triangulation
            Standard_Integer aNodeOffset    = 0;
            Standard_Integer aTriangleOffet = 0; 
            TopExp_Explorer anExpSF(namedSolids[i].solid, TopAbs_FACE);

        //    Standard_Integer trait =  namedLables.at(i).;
            Color clclclc = colors.at(namedSolids[i].name);
            color_submodule.at(i) = clclclc;
            while (anExpSF.More())
            {
                if (tracer)
                    tracer->progress(0.3f);
   
                const TopoDS_Shape& aFace = anExpSF.Current();
                TopLoc_Location     aLoc;
                Handle(Poly_Triangulation) aTriangulation = BRep_Tool::Triangulation(TopoDS::Face(aFace), aLoc);
                if (aTriangulation.IsNull()) {
                    ++aNbFacesNoTri;
                    continue;
                }
                // BBS: copy nodes
                gp_Trsf aTrsf = aLoc.Transformation();
                for (Standard_Integer aNodeIter = 1; aNodeIter <= aTriangulation->NbNodes(); ++aNodeIter) {
                    gp_Pnt aPnt = aTriangulation->Node(aNodeIter);
                    aPnt.Transform(aTrsf);
                    points.emplace_back(std::move(Vec3f(aPnt.X(), aPnt.Y(), aPnt.Z())));
                }
                // BBS: copy triangles
                const TopAbs_Orientation anOrientation = anExpSF.Current().Orientation();
                Standard_Integer anId[3];
                for (Standard_Integer aTriIter = 1; aTriIter <= aTriangulation->NbTriangles(); ++aTriIter) {

                    if (tracer)
                    {
                        tracer->progress((float)i / (float)aTriIter);
                        if (tracer->interrupt())
                        {
                            interuptted = true;
                            break;
                        }
                        tracer->progress(0.5f);
                    }

                    Poly_Triangle aTri = aTriangulation->Triangle(aTriIter);

                    aTri.Get(anId[0], anId[1], anId[2]);
                    if (anOrientation == TopAbs_REVERSED)
                        std::swap(anId[1], anId[2]);
                    for (int j = 0; j < 3; j++)
                    {
                        trimesh::point p(points[anId[j] + aNodeOffset - 1].x(), points[anId[j] + aNodeOffset - 1].y()
                            , points[anId[j] + aNodeOffset - 1].z());
                        tm->vertices.emplace_back(p);
                    }
                    
                    tm->grid.emplace_back(i);
                }
                aNodeOffset += aTriangulation->NbNodes();
                aTriangleOffet += aTriangulation->NbTriangles();
                anExpSF.Next();
            }
        }
    });

    if (tracer && tm->vertices.size() > 0)
    {

        tracer->progress(1.0f);
        tracer->success();
    }
    if (tracer && tm->vertices.size() == 0)
        tracer->failed("Parse File Error.");

    int fNum = (int)tm->vertices.size() / 3;
    for (int i = 0; i < fNum; ++i)
    {
        trimesh::TriMesh::Face face;
        face.x = 3 * i;
        face.y = 3 * i + 1;
        face.z = 3 * i + 2;
        tm->faces.push_back(face);
        trimesh::Color c(color_submodule.at(tm->grid.at(i)).r, color_submodule.at(tm->grid.at(i)).g, color_submodule.at(tm->grid.at(i)).b);
        tm->colors.emplace_back(c);
    }
    tm->grid.clear();
    return tm;

}

}; // namespace Slic3r
