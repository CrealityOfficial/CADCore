__cc_find(opencascade)
__cc_find(tbb)
__cc_find(eigen)
__cc_find(trimesh2)

__files_group(${CMAKE_CURRENT_SOURCE_DIR}/cadcore/ SRCS)	 

set(LIBS eigen
		 tbb
		 trimesh2
		 #TKBin
		 #TKBinL
		 #TKBinTObj
		 #TKBinXCAF
		 #TKBO
		 #TKBool
		 TKBRep
		 #TKCAF
		 #TKCDF
		 #TKD3DHost
		 #TKD3DHostTest
		 #TKDCAF
		 #TKDraw
		 TKernel
		 #TKExpress
		 #TKFeat
		 #TKFillet
		 #TKG2d
		 #TKG3d
		 #TKGeomAlgo
		 #TKGeomBase
		 #TKHLR
		 #TKIGES
		 #TKIVtk
		 #TKIVtkDraw
		 TKLCAF
		 TKMath
		 TKMesh
		 #TKMeshVS
		 #TKOffset
		 TKPrim
		 #TKQADraw
		 #TKRWMesh
		 #TKService
		 #TKShHealing
		 #TKStd
		 #TKStdL
		 TKSTEP
		 TKSTEP209
		 TKSTEPAttr
		 TKSTEPBase
		 #TKSTL
		 #TKTObj
		 #TKTObjDRAW
		 TKTopAlgo
		 #TKTopTest
		 #TKV3d
		 TKVCAF
		 #TKVRML
		 TKXCAF
		 #TKXDE
		 #TKXDECascade
		 #TKXDEDRAW
		 #TKXDEIGES
		 TKXDESTEP
		 #TKXMesh
		 #TKXml
		 #TKXmlL
		 #TKXmlTObj
		 #TKXmlXCAF
		 #TKXSBase
		 #TKXSDRAW     
		)
set(ILIBS cxnd)
set(DEFS __TBB_NO_IMPLICIT_LINKAGE)

__add_common_library(CADCore)

