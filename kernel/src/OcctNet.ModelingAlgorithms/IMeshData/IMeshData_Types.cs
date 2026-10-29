// Port of: src/ModelingAlgorithms/TKMesh/IMeshData/IMeshData_Types.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// IMeshData_Types.hxx is a header of typedefs. The port writes each type out where it is used (translation-rules
// section 9), with these meanings:
//
//   IEdgeHandle, IWireHandle, IFaceHandle, ICurveHandle, IPCurveHandle   IMeshData_Edge, _Wire, _Face, _Curve, _PCurve
//   IEdgePtr, IFacePtr (raw pointers)                                     the same references (decision 490)
//   ICurveArrayAdaptor(Handle)        IMeshData_ParametersListArrayAdaptor<IMeshData_Curve>
//   BndBox2dTree                      NCollection_EBTree<int, Bnd_Box2d>    (NCollection_Shared<X> is X)
//   BndBox2dTreeFiller                NCollection_UBTreeFiller<int, Bnd_Box2d>
//   VectorOf...                       NCollection_DynamicArray<...>
//   Array1OfVertexOfDelaun            NCollection_Array1<BRepMesh_Vertex>   (unit 4B's types)
//   SequenceOfBndB2d/Integer/Real     NCollection_Sequence<Bnd_B2d / int / double>
//   Model::SequenceOf... (std::deque) List<...> inside BRepMeshData (OCCT's allocator dropped)
//   ListOfInteger/Pnt2d/IPCurves      NCollection_List<int / gp_Pnt2d / IMeshData_PCurve>
//   MapOfInteger                      NCollection_PackedMapOfInteger
//   CircleCellFilter, VertexCellFilter NCollection_CellFilter<BRepMesh_CircleInspector / _VertexInspector, int, gp_XY>
//   DMapOfShapeInteger                NCollection_DataMap<TopoDS_Shape, int>(TopTools_ShapeMapHasher.Instance)
//   DMapOfIFacePtrsListOfInteger      NCollection_DataMap<IMeshData_Face, NCollection_List<int>>(ReferenceEqualityComparer.Instance)
//   MapOfIEdgePtr, MapOfIFacePtr      NCollection_Map<IMeshData_Edge / IMeshData_Face>(ReferenceEqualityComparer.Instance)
//   IDMapOfIFacePtrsListOfIPCurves    NCollection_IndexedDataMap<IMeshData_Face, NCollection_List<IMeshData_PCurve>>(ReferenceEqualityComparer.Instance)
//   DMapOfIFacePtrsMapOfIEdgePtrs     NCollection_DataMap<IMeshData_Face, NCollection_Map<IMeshData_Edge>>(ReferenceEqualityComparer.Instance)
//   MapOfIntegerInteger               NCollection_IntDataMap<bool>: OCCT's iteration order, which Delaun's result
//                                     depends on (decision 541)
//   IDMapOfLink                       BRepMesh_DataStructureOfDelaun's private link map (OCCT's equality, stable nodes)
//   ListOfInteger of a node's links   BRepMesh_DataStructureOfDelaun.LinkedListOfInteger (OCCT's list semantics)
//   MapOfOrientedEdges                NCollection_Map<BRepMesh_OrientedEdge> (its equality is OCCT's)
//   DMapOfIntegerListOfInteger, MapOfReal, IMapOfReal,
//   Array1OfInteger                   the NCollection maps and arrays of the same names' element types
//
// The maps keyed by pointers iterate in address order in OCCT; the port's compare by reference and iterate in
// insertion order (NCollection_Map is a HashSet, whose order doesn't depend on hash codes), which is deterministic
// (decision 490). The allocator arguments (NCollection_IncAllocator, MEMORY_BLOCK_SIZE_HUGE) go, by the rules; the
// preallocation sizes (256 for the vectors) are kept where OCCT passes them.
