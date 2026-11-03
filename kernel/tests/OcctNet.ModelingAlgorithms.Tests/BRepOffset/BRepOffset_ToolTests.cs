// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Smoke tests of BRepOffset_Tool, the offset algorithms' tools (unit 7E).</summary>
public class BRepOffset_ToolTests
{
    private static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();

    // An unfrozen copy, for the tools that write into the shapes they are given (the offset's own shapes).
    private static TopoDS_Shape WritableBox() => BRepTools.ExactCopyForWrite(Box(), out _);

    private static TopoDS_Face Face(in TopoDS_Shape theS, int theIndex)
    {
        NCollection_IndexedMap<TopoDS_Shape> aMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theS, TopAbs_FACE, aMap);
        return TopoDS.Face(aMap[theIndex]);
    }

    // Two faces of the box sharing an edge, and a face opposite to the first.
    private static void AdjacentFaces(in TopoDS_Shape theS, out TopoDS_Face F1, out TopoDS_Face F2, out TopoDS_Face FOpp)
    {
        F1 = Face(theS, 1);
        F2 = new TopoDS_Face();
        FOpp = new TopoDS_Face();
        NCollection_List<TopoDS_Shape> aLE = new(), aLV = new();
        for (int i = 2; i <= 6; i++)
        {
            TopoDS_Face aF = Face(theS, i);
            if (BRepOffset_Tool.FindCommonShapes(F1, aF, aLE, aLV))
            {
                if (F2.IsNull())
                {
                    F2 = aF;
                }
            }
            else
            {
                FOpp = aF;
            }
        }
    }

    [Fact]
    public void EdgeVertices_FollowTheOrientation()
    {
        TopoDS_Edge E = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        BRepOffset_Tool.EdgeVertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        Assert.Equal(0.0, BRep_Tool.Pnt(V1).X(), 1e-12);
        Assert.Equal(1.0, BRep_Tool.Pnt(V2).X(), 1e-12);

        BRepOffset_Tool.EdgeVertices(TopoDS.Edge(E.Reversed()), out V1, out V2);
        Assert.Equal(1.0, BRep_Tool.Pnt(V1).X(), 1e-12);
        Assert.Equal(0.0, BRep_Tool.Pnt(V2).X(), 1e-12);
    }

    [Fact]
    public void FindCommonShapes_AdjacentAndOppositeFaces()
    {
        TopoDS_Shape aBox = Box();
        AdjacentFaces(aBox, out TopoDS_Face F1, out TopoDS_Face F2, out TopoDS_Face FOpp);
        NCollection_List<TopoDS_Shape> aLE = new(), aLV = new();

        Assert.True(BRepOffset_Tool.FindCommonShapes(F1, F2, aLE, aLV));
        Assert.Equal(1, aLE.Extent());
        Assert.Equal(2, aLV.Extent());

        Assert.False(BRepOffset_Tool.FindCommonShapes(F1, FOpp, aLE, aLV));
        Assert.True(aLE.IsEmpty());
        Assert.True(aLV.IsEmpty());
    }

    [Fact]
    public void OrientSection_OppositeForFacesOfTheSameOrientation()
    {
        TopoDS_Shape aBox = Box();
        AdjacentFaces(aBox, out TopoDS_Face F1, out TopoDS_Face F2, out _);
        NCollection_List<TopoDS_Shape> aLE = new();
        BRepOffset_Tool.FindCommonShapes(F1, F2, TopAbs_EDGE, aLE);
        TopoDS_Edge E = TopoDS.Edge(aLE.First());

        BRepOffset_Tool.OrientSection(E, F1, F2, out TopAbs_Orientation O1, out TopAbs_Orientation O2);
        Assert.Equal(F1.Orientation() != F2.Orientation(), O1 == O2);
    }

    [Fact]
    public void Gabarit_IsTheLargestSideOfTheBox()
    {
        Geom_TrimmedCurve aC = new(new Geom_Line(new Point3D(0, 0, 0), new Direction3D(1, 2, 0)), 0.0, Math.Sqrt(5.0));
        Assert.Equal(2.0, BRepOffset_Tool.Gabarit(aC), 1e-6);
    }

    [Fact]
    public void CheckPlanesNormals_SameAndOppositeFaces()
    {
        TopoDS_Shape aBox = Box();
        AdjacentFaces(aBox, out TopoDS_Face F1, out TopoDS_Face F2, out TopoDS_Face FOpp);
        Assert.True(BRepOffset_Tool.CheckPlanesNormals(F1, F1));
        Assert.False(BRepOffset_Tool.CheckPlanesNormals(F1, F2));
        Assert.False(BRepOffset_Tool.CheckPlanesNormals(F1, FOpp));
    }

    [Fact]
    public void MapVertexEdges_AndBuildNeighbour_OnAFace()
    {
        TopoDS_Face F = Face(Box(), 1);
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MVE = new(TopTools_ShapeMapHasher.Instance);
        BRepOffset_Tool.MapVertexEdges(F, MVE);
        Assert.Equal(4, MVE.Extent());
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aP in MVE)
        {
            Assert.Equal(2, aP.Value.Extent());
        }

        TopoDS_Wire W = TopoDS.Wire(new TopExp_Explorer(F, TopAbs_WIRE).Current());
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> NOnV1 = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> NOnV2 = new(TopTools_ShapeMapHasher.Instance);
        BRepOffset_Tool.BuildNeighbour(W, F, NOnV1, NOnV2);
        for (TopExp_Explorer anExp = new(W, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(anExp.Current());
            Assert.True(NOnV1.IsBound(E));
            Assert.True(NOnV2.IsBound(E));
            TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            Assert.True(TopExp.FirstVertex(TopoDS.Edge(NOnV1[E])).IsSame(V1)
                        || TopExp.LastVertex(TopoDS.Edge(NOnV1[E])).IsSame(V1));
            Assert.True(TopExp.FirstVertex(TopoDS.Edge(NOnV2[E])).IsSame(V2)
                        || TopExp.LastVertex(TopoDS.Edge(NOnV2[E])).IsSame(V2));
        }
    }

    [Fact]
    public void Deboucle3D_KeepsClosedShellsOnly()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Shell aShell = TopoDS.Shell(new TopExp_Explorer(aBox, TopAbs_SHELL).Current());
        NCollection_Map<TopoDS_Shape> aBoundary = new(TopTools_ShapeMapHasher.Instance);
        Assert.True(BRepOffset_Tool.Deboucle3D(aShell, aBoundary).IsSame(aShell));

        // A shell of one face has free edges which aren't in the boundary.
        BRep_Builder B = new();
        TopoDS_Shell anOpen = new();
        B.MakeShell(ref anOpen);
        B.Add(anOpen, Face(aBox, 1));
        TopoDS_Compound aC = new();
        B.MakeCompound(ref aC);
        B.Add(aC, anOpen);
        Assert.True(BRepOffset_Tool.Deboucle3D(anOpen, aBoundary).IsNull());
        Assert.True(BRepOffset_Tool.Deboucle3D(aC, aBoundary).IsNull());

        // With its edges as boundary, it is kept.
        for (TopExp_Explorer anExp = new(anOpen, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            aBoundary.Add(anExp.Current());
        }
        Assert.False(BRepOffset_Tool.Deboucle3D(aC, aBoundary).IsNull());
    }

    [Fact]
    public void CheckBounds_PlanesAreEnlarged()
    {
        TopoDS_Shape aBox = Box();
        BRepOffset_Analyse anAnalyse = new(aBox, 0.01);
        BRepOffset_Tool.CheckBounds(Face(aBox, 1), anAnalyse, out bool aU, out bool aVf, out bool aVl);
        Assert.True(aU && aVf && aVl);
    }

    [Fact]
    public void EnLargeFace_PlaneWithinTheMaximalBounds()
    {
        TopoDS_Face F = Face(Box(), 1);
        TopoDS_Face NF = new();
        bool aChanged = BRepOffset_Tool.EnLargeFace(F, ref NF, true);
        Assert.False(aChanged); // a plane is not changed, only bounded
        Assert.Equal(F.Orientation(), NF.Orientation());
        Assert.Same(BRep_Tool.Surface(F), BRep_Tool.Surface(NF));
        BRepTools.UVBounds(NF, out double U1, out double U2, out double V1, out double V2);
        Assert.Equal(-1.0e7, U1, 1e-3);
        Assert.Equal(1.0e7, U2, 1e-3);
        Assert.Equal(-1.0e7, V1, 1e-3);
        Assert.Equal(1.0e7, V2, 1e-3);
    }

    [Fact]
    public void EnLargeFace_CylinderWithUpdatedPCurves()
    {
        TopoDS_Shape aCyl = BRepTools.ExactCopyForWrite(new BRepPrimAPI_MakeCylinder(5.0, 10.0).Shape(), out _);
        TopoDS_Face F = new();
        for (TopExp_Explorer anExp = new(aCyl, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            if (BRep_Tool.Surface(TopoDS.Face(anExp.Current())) is Geom_CylindricalSurface)
            {
                F = TopoDS.Face(anExp.Current());
            }
        }
        Assert.False(F.IsNull());
        TopoDS_Face NF = new();
        BRepOffset_Tool.EnLargeFace(F, ref NF, true, true);
        BRepTools.UVBounds(NF, out _, out _, out double V1, out double V2);
        Assert.True(V1 < -1.0 && V2 > 11.0);
    }

    [Fact]
    public void EnLargeFace_SurfaceOfRevolutionMovesThePCurves()
    {
        // A skew segment revolved about OZ: a surface of revolution, which EnlargeGeometry extends in V.
        TopoDS_Edge E = new BRepBuilderAPI_MakeEdge(new Point3D(5, 0, 0), new Point3D(5, 3, 10)).Edge();
        TopoDS_Shape aRevol = BRepTools.ExactCopyForWrite(new BRepPrimAPI_MakeRevol(E, Constants.OZ()).Shape(), out _);
        TopoDS_Face F = TopoDS.Face(new TopExp_Explorer(aRevol, TopAbs_FACE).Current());
        Assert.IsType<Geom_SurfaceOfRevolution>(BRep_Tool.Surface(F));
        BRepTools.UVBounds(F, out _, out _, out double FV1, out double FV2);

        TopoDS_Face NF = new();
        Assert.True(BRepOffset_Tool.EnLargeFace(F, ref NF, true, true));
        BRepTools.UVBounds(NF, out _, out _, out double V1, out double V2);
        Assert.True(V1 < FV1 - 1.0e-3 && V2 > FV2 + 1.0e-3);

        // F now lies on the enlarged surface, and its edges have their pcurves on it.
        Assert.Same(BRep_Tool.Surface(NF), BRep_Tool.Surface(F));
        for (TopExp_Explorer anExp = new(F, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.NotNull(BRep_Tool.CurveOnSurface(TopoDS.Edge(anExp.Current()), NF, out _, out _));
        }
    }

    [Fact]
    public void EnLargeFace_LeavesAFrozenFaceAlone()
    {
        TopoDS_Edge E = new BRepBuilderAPI_MakeEdge(new Point3D(5, 0, 0), new Point3D(5, 3, 10)).Edge();
        TopoDS_Shape aRevol = new BRepPrimAPI_MakeRevol(E, Constants.OZ()).Shape();
        TopoDS_Face F = TopoDS.Face(new TopExp_Explorer(aRevol, TopAbs_FACE).Current());
        Geom_Surface? aS = BRep_Tool.Surface(F);
        TopoDS_Face NF = new();
        Assert.True(BRepOffset_Tool.EnLargeFace(F, ref NF, true));
        Assert.Same(aS, BRep_Tool.Surface(F));
        Assert.NotSame(aS, BRep_Tool.Surface(NF));
    }

    [Fact]
    public void Inter3D_InfinitePlanes()
    {
        TopoDS_Face F1 = new BRepLib_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))).InternalFace();
        TopoDS_Face F2 = new BRepLib_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0))).InternalFace();
        NCollection_List<TopoDS_Shape> L1 = new(), L2 = new();
        BRepOffset_Tool.Inter3D(F1, F2, L1, L2, TopAbs_IN, new TopoDS_Edge(), new TopoDS_Face(), new TopoDS_Face());
        Assert.Equal(1, L1.Extent());
        Assert.Equal(1, L2.Extent());
        TopoDS_Edge E = TopoDS.Edge(L1.First());
        Assert.True(E.IsSame(L2.First()));
        BRepAdaptor_Curve aC = new(E);
        Point3D aP = aC.Value(0.5 * (aC.FirstParameter() + aC.LastParameter()));
        Assert.Equal(0.0, aP.X(), 1e-7);
        Assert.Equal(0.0, aP.Z(), 1e-7);
        Assert.NotNull(BRep_Tool.CurveOnSurface(E, F1, out _, out _));
        Assert.NotNull(BRep_Tool.CurveOnSurface(E, F2, out _, out _));
    }

    [Fact]
    public void Inter3D_CrossingFinitePlanes()
    {
        // Finite planar faces: the pave filler's section.
        TopoDS_Face F1 = new BRepLib_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), -5, 5, -5, 5).InternalFace();
        TopoDS_Face F2 = new BRepLib_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), -5, 5, -5, 5).InternalFace();
        NCollection_List<TopoDS_Shape> L1 = new(), L2 = new();
        BRepOffset_Tool.Inter3D(F1, F2, L1, L2, TopAbs_IN, new TopoDS_Edge(), new TopoDS_Face(), new TopoDS_Face());
        Assert.False(L1.IsEmpty());
        Assert.Equal(L1.Extent(), L2.Extent());
        double aLength = 0.0;
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(L1); it.More(); it.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(it.Value());
            BRepAdaptor_Curve aC = new(E);
            Point3D aP = aC.Value(0.5 * (aC.FirstParameter() + aC.LastParameter()));
            Assert.Equal(0.0, aP.X(), 1e-7);
            Assert.Equal(0.0, aP.Z(), 1e-7);
            Assert.NotNull(BRep_Tool.CurveOnSurface(E, F1, out _, out _));
            Assert.NotNull(BRep_Tool.CurveOnSurface(E, F2, out _, out _));
            aLength += GCPnts_AbscissaPoint.Length(aC);
        }
        Assert.Equal(10.0, aLength, 1e-6);
    }

    [Fact]
    public void Inter2d_AdjacentEdgesOfAFace()
    {
        // Inter2d writes into the edges (BRep_Builder.UpdateVertex on them): the offset's own, so unfrozen.
        TopoDS_Face F = Face(WritableBox(), 1);
        TopoDS_Wire W = TopoDS.Wire(new TopExp_Explorer(F, TopAbs_WIRE).Current());
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> NOnV1 = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> NOnV2 = new(TopTools_ShapeMapHasher.Instance);
        BRepOffset_Tool.BuildNeighbour(W, F, NOnV1, NOnV2);
        TopoDS_Edge E1 = TopoDS.Edge(new TopExp_Explorer(W, TopAbs_EDGE).Current());
        TopoDS_Edge E2 = TopoDS.Edge(NOnV2[E1]);
        TopExp.Vertices(E1, out _, out TopoDS_Vertex aV2);

        NCollection_List<TopoDS_Shape> LV = new();
        BRepOffset_Tool.Inter2d(F, E1, E2, LV, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1, LV.Extent());
        Assert.True(BRep_Tool.Pnt(TopoDS.Vertex(LV.First())).Distance(BRep_Tool.Pnt(aV2)) < 1e-7);
    }

    [Fact]
    public void TryProject_TheFaceEdgesOnTheirFace()
    {
        TopoDS_Shape aBox = WritableBox();
        AdjacentFaces(aBox, out TopoDS_Face F1, out TopoDS_Face F2, out TopoDS_Face FOpp);
        NCollection_List<TopoDS_Shape> aEdges = new();
        for (TopExp_Explorer anExp = new(F1, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            aEdges.Append(anExp.Current());
        }
        NCollection_List<TopoDS_Shape> L1 = new(), L2 = new();
        Assert.True(BRepOffset_Tool.TryProject(F1, F2, aEdges, L1, L2, TopAbs_IN, 1e-6));
        Assert.Equal(4, L1.Extent());

        Assert.False(BRepOffset_Tool.TryProject(FOpp, F2, aEdges, L1, L2, TopAbs_IN, 1e-6));
        Assert.True(L1.IsEmpty());
    }
    // A B-spline face's working offset face: BRepOffset_Offset puts it on an offset surface.
    private static TopoDS_Face OffsetCylinderFace(out TopoDS_Face theFace)
    {
        TopoDS_Shape aCyl = new BRepBuilderAPI_NurbsConvert(new BRepPrimAPI_MakeCylinder(5.0, 10.0).Shape()).Shape();
        theFace = Face(aCyl, 1);
        return new BRepOffset_Offset(theFace, -1.0).InternalFace();
    }

    [Fact]
    public void EnLargeFace_PutsTheEnlargedOffsetSurfaceOnAWorkingFace_AsOCCTDoesInPlace()
    {
        // decision 1101: OCCT sets the enlarged basis on the face's own offset surface, so the face lies on it after
        TopoDS_Face anOF = OffsetCylinderFace(out _);
        Geom_Surface aBefore = BRep_Tool.Surface(anOF)!;
        Assert.IsType<Geom_OffsetSurface>(aBefore);
        aBefore.Bounds(out _, out _, out double v1, out double v2);
        string aCheck = new BRepCheck_Analyzer(anOF).IsValid().ToString();
        TopoDS_Face aNF = new();
        Assert.True(BRepOffset_Tool.EnLargeFace(anOF, ref aNF, true));
        Geom_Surface anAfter = BRep_Tool.Surface(anOF)!;
        Assert.NotSame(aBefore, anAfter);
        Assert.Same(BRep_Tool.Surface(aNF), anAfter);
        anAfter.Bounds(out _, out _, out double w1, out double w2);
        Assert.True(w1 < v1 && w2 > v2);
        // the face's edges keep their pcurves, now on the enlarged surface
        for (TopExp_Explorer anExp = new(anOF, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.NotNull(BRep_Tool.CurveOnSurface(TopoDS.Edge(anExp.Current()), anOF, out _, out _));
        }
        Assert.Equal(aCheck, new BRepCheck_Analyzer(anOF).IsValid().ToString());
    }

    [Fact]
    public void EnLargeFace_LeavesAFrozenFacesSurface()
    {
        TopoDS_Face anOF = OffsetCylinderFace(out _);
        anOF.TShape()!.FreezeGraph(); // a caller's face
        Geom_Surface aBefore = BRep_Tool.Surface(anOF)!;
        TopoDS_Face aNF = new();
        Assert.True(BRepOffset_Tool.EnLargeFace(anOF, ref aNF, true));
        Assert.Same(aBefore, BRep_Tool.Surface(anOF));
        Assert.NotSame(aBefore, BRep_Tool.Surface(aNF));
    }
}
