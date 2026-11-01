// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Tool.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// SKELETON (unit 7E): the signatures the other BRepOffset files compile against; the porter of BRepOffset_Tool.cxx
// replaces the bodies, keeping these signatures.

namespace Occt;

/// <summary>The offset algorithms' topological and geometric tools.</summary>
public static class BRepOffset_Tool
{
    /// <summary>The vertices of <paramref name="E"/> in the order of its orientation.</summary>
    public static void EdgeVertices(in TopoDS_Edge E, out TopoDS_Vertex V1, out TopoDS_Vertex V2)
    {
        if (E.Orientation() == TopAbs_Orientation.TopAbs_REVERSED)
        {
            TopExp.Vertices(E, out V2, out V1);
        }
        else
        {
            TopExp.Vertices(E, out V1, out V2);
        }
    }

    public static void OrientSection(in TopoDS_Edge E, in TopoDS_Face F1, in TopoDS_Face F2,
                                     out TopAbs_Orientation O1, out TopAbs_Orientation O2) =>
        throw new NotImplementedException();

    public static bool FindCommonShapes(in TopoDS_Face theF1, in TopoDS_Face theF2, NCollection_List<TopoDS_Shape> theLE,
                                        NCollection_List<TopoDS_Shape> theLV) =>
        throw new NotImplementedException();

    public static bool FindCommonShapes(in TopoDS_Shape theS1, in TopoDS_Shape theS2, TopAbs_ShapeEnum theType,
                                        NCollection_List<TopoDS_Shape> theLSC) =>
        throw new NotImplementedException();

    public static void Inter3D(in TopoDS_Face F1, in TopoDS_Face F2, NCollection_List<TopoDS_Shape> LInt1,
                               NCollection_List<TopoDS_Shape> LInt2, TopAbs_State Side, in TopoDS_Edge RefEdge,
                               in TopoDS_Face RefFace1, in TopoDS_Face RefFace2) =>
        throw new NotImplementedException();

    public static bool TryProject(in TopoDS_Face F1, in TopoDS_Face F2, NCollection_List<TopoDS_Shape> Edges,
                                  NCollection_List<TopoDS_Shape> LInt1, NCollection_List<TopoDS_Shape> LInt2,
                                  TopAbs_State Side, double TolConf) =>
        throw new NotImplementedException();

    public static void PipeInter(in TopoDS_Face F1, in TopoDS_Face F2, NCollection_List<TopoDS_Shape> LInt1,
                                 NCollection_List<TopoDS_Shape> LInt2, TopAbs_State Side) =>
        throw new NotImplementedException();

    public static void Inter2d(in TopoDS_Face F, in TopoDS_Edge E1, in TopoDS_Edge E2,
                               NCollection_List<TopoDS_Shape> LV, double Tol) =>
        throw new NotImplementedException();

    public static void InterOrExtent(in TopoDS_Face F1, in TopoDS_Face F2, NCollection_List<TopoDS_Shape> LInt1,
                                     NCollection_List<TopoDS_Shape> LInt2, TopAbs_State Side) =>
        throw new NotImplementedException();

    public static void CheckBounds(in TopoDS_Face F, BRepOffset_Analyse Analyse, out bool enlargeU,
                                   out bool enlargeVfirst, out bool enlargeVlast) =>
        throw new NotImplementedException();

    public static bool EnLargeFace(in TopoDS_Face F, ref TopoDS_Face NF, bool ChangeGeom, bool UpDatePCurve = false,
                                   bool enlargeU = true, bool enlargeVfirst = true, bool enlargeVlast = true,
                                   int theExtensionMode = 1, double theLenBeforeUfirst = -1.0,
                                   double theLenAfterUlast = -1.0, double theLenBeforeVfirst = -1.0,
                                   double theLenAfterVlast = -1.0) =>
        throw new NotImplementedException();

    public static void ExtentFace(in TopoDS_Face F, NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> ConstShapes,
                                  NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> ToBuild, TopAbs_State Side,
                                  double TolConf, ref TopoDS_Face NF) =>
        throw new NotImplementedException();

    public static void BuildNeighbour(in TopoDS_Wire W, in TopoDS_Face F,
                                      NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> NOnV1,
                                      NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> NOnV2) =>
        throw new NotImplementedException();

    public static void MapVertexEdges(in TopoDS_Shape S,
                                      NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MVE) =>
        throw new NotImplementedException();

    public static TopoDS_Shape Deboucle3D(in TopoDS_Shape S, NCollection_Map<TopoDS_Shape> Boundary) =>
        throw new NotImplementedException();

    public static void CorrectOrientation(in TopoDS_Shape SI, NCollection_IndexedMap<TopoDS_Shape> NewEdges,
                                          BRepAlgo_AsDes AsDes, BRepAlgo_Image InitOffset, double Offset) =>
        throw new NotImplementedException();

    public static double Gabarit(Geom_Curve aCurve) => throw new NotImplementedException();

    public static bool CheckPlanesNormals(in TopoDS_Face theFace1, in TopoDS_Face theFace2, double theTolAng = 1.0e-8) =>
        throw new NotImplementedException();
}
