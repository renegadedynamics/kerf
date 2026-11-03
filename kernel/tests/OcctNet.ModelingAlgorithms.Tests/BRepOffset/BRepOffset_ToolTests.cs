// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Smoke tests of BRepOffset_Tool, the offset algorithms' tools (unit 7E).</summary>
public class BRepOffset_ToolTests
{
    private static Shape Box() => new BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();

    // An unfrozen copy, for the tools that write into the shapes they are given (the offset's own shapes).
    private static Shape WritableBox() => BrepTools.ExactCopyForWrite(Box(), out _);

    private static Face Face(in Shape theS, int theIndex)
    {
        NCollection_IndexedMap<Shape> aMap = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        global::Kerf.Engine.Topology.Exploration.MapShapes(theS, global::Kerf.Topology.ShapeType.Face, aMap);
        return global::Kerf.Engine.Topology.ShapeCasts.Face(aMap[theIndex]);
    }

    // Two faces of the box sharing an edge, and a face opposite to the first.
    private static void AdjacentFaces(in Shape theS, out Face F1, out Face F2, out Face FOpp)
    {
        F1 = Face(theS, 1);
        F2 = new Face();
        FOpp = new Face();
        NCollection_List<Shape> aLE = new(), aLV = new();
        for (int i = 2; i <= 6; i++)
        {
            Face aF = Face(theS, i);
            if (BRepOffset_Tool.FindCommonShapes(F1, aF, aLE, aLV))
            {
                if (F2.IsNull)
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
        Edge E = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        BRepOffset_Tool.EdgeVertices(E, out Vertex V1, out Vertex V2);
        Assert.Equal(0.0, Tool.Pnt(V1).X(), 1e-12);
        Assert.Equal(1.0, Tool.Pnt(V2).X(), 1e-12);

        BRepOffset_Tool.EdgeVertices(global::Kerf.Engine.Topology.ShapeCasts.Edge(E.Reversed()), out V1, out V2);
        Assert.Equal(1.0, Tool.Pnt(V1).X(), 1e-12);
        Assert.Equal(0.0, Tool.Pnt(V2).X(), 1e-12);
    }

    [Fact]
    public void FindCommonShapes_AdjacentAndOppositeFaces()
    {
        Shape aBox = Box();
        AdjacentFaces(aBox, out Face F1, out Face F2, out Face FOpp);
        NCollection_List<Shape> aLE = new(), aLV = new();

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
        Shape aBox = Box();
        AdjacentFaces(aBox, out Face F1, out Face F2, out _);
        NCollection_List<Shape> aLE = new();
        BRepOffset_Tool.FindCommonShapes(F1, F2, ShapeType.Edge, aLE);
        Edge E = global::Kerf.Engine.Topology.ShapeCasts.Edge(aLE.First());

        BRepOffset_Tool.OrientSection(E, F1, F2, out Orientation O1, out Orientation O2);
        Assert.Equal(F1.Orientation() != F2.Orientation(), O1 == O2);
    }

    [Fact]
    public void Gabarit_IsTheLargestSideOfTheBox()
    {
        TrimmedCurve3D aC = new(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 2, 0)), 0.0, Math.Sqrt(5.0));
        Assert.Equal(2.0, BRepOffset_Tool.Gabarit(aC), 1e-6);
    }

    [Fact]
    public void CheckPlanesNormals_SameAndOppositeFaces()
    {
        Shape aBox = Box();
        AdjacentFaces(aBox, out Face F1, out Face F2, out Face FOpp);
        Assert.True(BRepOffset_Tool.CheckPlanesNormals(F1, F1));
        Assert.False(BRepOffset_Tool.CheckPlanesNormals(F1, F2));
        Assert.False(BRepOffset_Tool.CheckPlanesNormals(F1, FOpp));
    }

    [Fact]
    public void MapVertexEdges_AndBuildNeighbour_OnAFace()
    {
        Face F = Face(Box(), 1);
        NCollection_DataMap<Shape, NCollection_List<Shape>> MVE = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        BRepOffset_Tool.MapVertexEdges(F, MVE);
        Assert.Equal(4, MVE.Extent());
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> aP in MVE)
        {
            Assert.Equal(2, aP.Value.Extent());
        }

        Wire W = global::Kerf.Engine.Topology.ShapeCasts.Wire(new global::Kerf.Engine.Topology.Explorer(F, ShapeType.Wire).Current());
        NCollection_DataMap<Shape, Shape> NOnV1 = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        NCollection_DataMap<Shape, Shape> NOnV2 = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        BRepOffset_Tool.BuildNeighbour(W, F, NOnV1, NOnV2);
        for (global::Kerf.Engine.Topology.Explorer anExp = new(W, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge E = global::Kerf.Engine.Topology.ShapeCasts.Edge(anExp.Current());
            Assert.True(NOnV1.IsBound(E));
            Assert.True(NOnV2.IsBound(E));
            global::Kerf.Engine.Topology.Exploration.Vertices(E, out Vertex V1, out Vertex V2);
            Assert.True(global::Kerf.Engine.Topology.Exploration.FirstVertex(global::Kerf.Engine.Topology.ShapeCasts.Edge(NOnV1[E])).IsSame(V1)
                        || global::Kerf.Engine.Topology.Exploration.LastVertex(global::Kerf.Engine.Topology.ShapeCasts.Edge(NOnV1[E])).IsSame(V1));
            Assert.True(global::Kerf.Engine.Topology.Exploration.FirstVertex(global::Kerf.Engine.Topology.ShapeCasts.Edge(NOnV2[E])).IsSame(V2)
                        || global::Kerf.Engine.Topology.Exploration.LastVertex(global::Kerf.Engine.Topology.ShapeCasts.Edge(NOnV2[E])).IsSame(V2));
        }
    }

    [Fact]
    public void Deboucle3D_KeepsClosedShellsOnly()
    {
        Shape aBox = Box();
        Shell aShell = global::Kerf.Engine.Topology.ShapeCasts.Shell(new global::Kerf.Engine.Topology.Explorer(aBox, ShapeType.Shell).Current());
        NCollection_Map<Shape> aBoundary = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        Assert.True(BRepOffset_Tool.Deboucle3D(aShell, aBoundary).IsSame(aShell));

        // A shell of one face has free edges which aren't in the boundary.
        BrepBuilder B = new();
        Shell anOpen = new();
        B.MakeShell(ref anOpen);
        B.Add(anOpen, Face(aBox, 1));
        Compound aC = new();
        B.MakeCompound(ref aC);
        B.Add(aC, anOpen);
        Assert.True(BRepOffset_Tool.Deboucle3D(anOpen, aBoundary).IsNull);
        Assert.True(BRepOffset_Tool.Deboucle3D(aC, aBoundary).IsNull);

        // With its edges as boundary, it is kept.
        for (global::Kerf.Engine.Topology.Explorer anExp = new(anOpen, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            aBoundary.Add(anExp.Current());
        }
        Assert.False(BRepOffset_Tool.Deboucle3D(aC, aBoundary).IsNull);
    }

    [Fact]
    public void CheckBounds_PlanesAreEnlarged()
    {
        Shape aBox = Box();
        BRepOffset_Analyse anAnalyse = new(aBox, 0.01);
        BRepOffset_Tool.CheckBounds(Face(aBox, 1), anAnalyse, out bool aU, out bool aVf, out bool aVl);
        Assert.True(aU && aVf && aVl);
    }

    [Fact]
    public void EnLargeFace_PlaneWithinTheMaximalBounds()
    {
        Face F = Face(Box(), 1);
        Face NF = new();
        bool aChanged = BRepOffset_Tool.EnLargeFace(F, ref NF, true);
        Assert.False(aChanged); // a plane is not changed, only bounded
        Assert.Equal(F.Orientation(), NF.Orientation());
        Assert.Same(Tool.Surface(F), Tool.Surface(NF));
        BrepTools.UVBounds(NF, out double U1, out double U2, out double V1, out double V2);
        Assert.Equal(-1.0e7, U1, 1e-3);
        Assert.Equal(1.0e7, U2, 1e-3);
        Assert.Equal(-1.0e7, V1, 1e-3);
        Assert.Equal(1.0e7, V2, 1e-3);
    }

    [Fact]
    public void EnLargeFace_CylinderWithUpdatedPCurves()
    {
        Shape aCyl = BrepTools.ExactCopyForWrite(new BRepPrimAPI_MakeCylinder(5.0, 10.0).Shape(), out _);
        Face F = new();
        for (global::Kerf.Engine.Topology.Explorer anExp = new(aCyl, global::Kerf.Topology.ShapeType.Face); anExp.More(); anExp.Next())
        {
            if (Tool.Surface(global::Kerf.Engine.Topology.ShapeCasts.Face(anExp.Current())) is CylindricalSurface)
            {
                F = global::Kerf.Engine.Topology.ShapeCasts.Face(anExp.Current());
            }
        }
        Assert.False(F.IsNull);
        Face NF = new();
        BRepOffset_Tool.EnLargeFace(F, ref NF, true, true);
        BrepTools.UVBounds(NF, out _, out _, out double V1, out double V2);
        Assert.True(V1 < -1.0 && V2 > 11.0);
    }

    [Fact]
    public void EnLargeFace_SurfaceOfRevolutionMovesThePCurves()
    {
        // A skew segment revolved about OZ: a surface of revolution, which EnlargeGeometry extends in V.
        Edge E = new BRepBuilderAPI_MakeEdge(new Point3D(5, 0, 0), new Point3D(5, 3, 10)).Edge();
        Shape aRevol = BrepTools.ExactCopyForWrite(new BRepPrimAPI_MakeRevol(E, Constants.OZ()).Shape(), out _);
        Face F = global::Kerf.Engine.Topology.ShapeCasts.Face(new global::Kerf.Engine.Topology.Explorer(aRevol, global::Kerf.Topology.ShapeType.Face).Current());
        Assert.IsType<RevolutionSurface>(Tool.Surface(F));
        BrepTools.UVBounds(F, out _, out _, out double FV1, out double FV2);

        Face NF = new();
        Assert.True(BRepOffset_Tool.EnLargeFace(F, ref NF, true, true));
        BrepTools.UVBounds(NF, out _, out _, out double V1, out double V2);
        Assert.True(V1 < FV1 - 1.0e-3 && V2 > FV2 + 1.0e-3);

        // F now lies on the enlarged surface, and its edges have their pcurves on it.
        Assert.Same(Tool.Surface(NF), Tool.Surface(F));
        for (global::Kerf.Engine.Topology.Explorer anExp = new(F, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.NotNull(Tool.CurveOnSurface(global::Kerf.Engine.Topology.ShapeCasts.Edge(anExp.Current()), NF, out _, out _));
        }
    }

    [Fact]
    public void EnLargeFace_LeavesAFrozenFaceAlone()
    {
        Edge E = new BRepBuilderAPI_MakeEdge(new Point3D(5, 0, 0), new Point3D(5, 3, 10)).Edge();
        Shape aRevol = new BRepPrimAPI_MakeRevol(E, Constants.OZ()).Shape();
        Face F = global::Kerf.Engine.Topology.ShapeCasts.Face(new global::Kerf.Engine.Topology.Explorer(aRevol, global::Kerf.Topology.ShapeType.Face).Current());
        Surface? aS = Tool.Surface(F);
        Face NF = new();
        Assert.True(BRepOffset_Tool.EnLargeFace(F, ref NF, true));
        Assert.Same(aS, Tool.Surface(F));
        Assert.NotSame(aS, Tool.Surface(NF));
    }

    [Fact]
    public void Inter3D_InfinitePlanes()
    {
        Face F1 = new BRepLib_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))).InternalFace();
        Face F2 = new BRepLib_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0))).InternalFace();
        NCollection_List<Shape> L1 = new(), L2 = new();
        BRepOffset_Tool.Inter3D(F1, F2, L1, L2, In, new Edge(), new Face(), new Face());
        Assert.Equal(1, L1.Extent());
        Assert.Equal(1, L2.Extent());
        Edge E = global::Kerf.Engine.Topology.ShapeCasts.Edge(L1.First());
        Assert.True(E.IsSame(L2.First()));
        EdgeCurveAdaptor aC = new(E);
        Point3D aP = aC.Value(0.5 * (aC.FirstParameter() + aC.LastParameter()));
        Assert.Equal(0.0, aP.X(), 1e-7);
        Assert.Equal(0.0, aP.Z(), 1e-7);
        Assert.NotNull(Tool.CurveOnSurface(E, F1, out _, out _));
        Assert.NotNull(Tool.CurveOnSurface(E, F2, out _, out _));
    }

    [Fact]
    public void Inter3D_CrossingFinitePlanes()
    {
        // Finite planar faces: the pave filler's section.
        Face F1 = new BRepLib_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), -5, 5, -5, 5).InternalFace();
        Face F2 = new BRepLib_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), -5, 5, -5, 5).InternalFace();
        NCollection_List<Shape> L1 = new(), L2 = new();
        BRepOffset_Tool.Inter3D(F1, F2, L1, L2, In, new Edge(), new Face(), new Face());
        Assert.False(L1.IsEmpty());
        Assert.Equal(L1.Extent(), L2.Extent());
        double aLength = 0.0;
        for (NCollection_List<Shape>.Iterator it = new(L1); it.More(); it.Next())
        {
            Edge E = global::Kerf.Engine.Topology.ShapeCasts.Edge(it.Value());
            EdgeCurveAdaptor aC = new(E);
            Point3D aP = aC.Value(0.5 * (aC.FirstParameter() + aC.LastParameter()));
            Assert.Equal(0.0, aP.X(), 1e-7);
            Assert.Equal(0.0, aP.Z(), 1e-7);
            Assert.NotNull(Tool.CurveOnSurface(E, F1, out _, out _));
            Assert.NotNull(Tool.CurveOnSurface(E, F2, out _, out _));
            aLength += AbscissaPoint.Length(aC);
        }
        Assert.Equal(10.0, aLength, 1e-6);
    }

    [Fact]
    public void Inter2d_AdjacentEdgesOfAFace()
    {
        // Inter2d writes into the edges (BRep_Builder.UpdateVertex on them): the offset's own, so unfrozen.
        Face F = Face(WritableBox(), 1);
        Wire W = global::Kerf.Engine.Topology.ShapeCasts.Wire(new global::Kerf.Engine.Topology.Explorer(F, ShapeType.Wire).Current());
        NCollection_DataMap<Shape, Shape> NOnV1 = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        NCollection_DataMap<Shape, Shape> NOnV2 = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        BRepOffset_Tool.BuildNeighbour(W, F, NOnV1, NOnV2);
        Edge E1 = global::Kerf.Engine.Topology.ShapeCasts.Edge(new global::Kerf.Engine.Topology.Explorer(W, ShapeType.Edge).Current());
        Edge E2 = global::Kerf.Engine.Topology.ShapeCasts.Edge(NOnV2[E1]);
        global::Kerf.Engine.Topology.Exploration.Vertices(E1, out _, out Vertex aV2);

        NCollection_List<Shape> LV = new();
        BRepOffset_Tool.Inter2d(F, E1, E2, LV, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(1, LV.Extent());
        Assert.True(Tool.Pnt(global::Kerf.Engine.Topology.ShapeCasts.Vertex(LV.First())).Distance(Tool.Pnt(aV2)) < 1e-7);
    }

    [Fact]
    public void TryProject_TheFaceEdgesOnTheirFace()
    {
        Shape aBox = WritableBox();
        AdjacentFaces(aBox, out Face F1, out Face F2, out Face FOpp);
        NCollection_List<Shape> aEdges = new();
        for (global::Kerf.Engine.Topology.Explorer anExp = new(F1, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            aEdges.Append(anExp.Current());
        }
        NCollection_List<Shape> L1 = new(), L2 = new();
        Assert.True(BRepOffset_Tool.TryProject(F1, F2, aEdges, L1, L2, In, 1e-6));
        Assert.Equal(4, L1.Extent());

        Assert.False(BRepOffset_Tool.TryProject(FOpp, F2, aEdges, L1, L2, In, 1e-6));
        Assert.True(L1.IsEmpty());
    }
    // A B-spline face's working offset face: BRepOffset_Offset puts it on an offset surface.
    private static Face OffsetCylinderFace(out Face theFace)
    {
        Shape aCyl = new BRepBuilderAPI_NurbsConvert(new BRepPrimAPI_MakeCylinder(5.0, 10.0).Shape()).Shape();
        theFace = Face(aCyl, 1);
        return new BRepOffset_Offset(theFace, -1.0).InternalFace();
    }

    [Fact]
    public void EnLargeFace_PutsTheEnlargedOffsetSurfaceOnAWorkingFace_AsOCCTDoesInPlace()
    {
        // decision 1101: OCCT sets the enlarged basis on the face's own offset surface, so the face lies on it after
        Face anOF = OffsetCylinderFace(out _);
        Surface aBefore = Tool.Surface(anOF)!;
        Assert.IsType<OffsetSurface>(aBefore);
        aBefore.Bounds(out _, out _, out double v1, out double v2);
        string aCheck = new BRepCheck_Analyzer(anOF).IsValid().ToString();
        Face aNF = new();
        Assert.True(BRepOffset_Tool.EnLargeFace(anOF, ref aNF, true));
        Surface anAfter = Tool.Surface(anOF)!;
        Assert.NotSame(aBefore, anAfter);
        Assert.Same(Tool.Surface(aNF), anAfter);
        anAfter.Bounds(out _, out _, out double w1, out double w2);
        Assert.True(w1 < v1 && w2 > v2);
        // the face's edges keep their pcurves, now on the enlarged surface
        for (global::Kerf.Engine.Topology.Explorer anExp = new(anOF, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.NotNull(Tool.CurveOnSurface(global::Kerf.Engine.Topology.ShapeCasts.Edge(anExp.Current()), anOF, out _, out _));
        }
        Assert.Equal(aCheck, new BRepCheck_Analyzer(anOF).IsValid().ToString());
    }

    [Fact]
    public void EnLargeFace_LeavesAFrozenFacesSurface()
    {
        Face anOF = OffsetCylinderFace(out _);
        anOF.TShape()!.FreezeGraph(); // a caller's face
        Surface aBefore = Tool.Surface(anOF)!;
        Face aNF = new();
        Assert.True(BRepOffset_Tool.EnLargeFace(anOF, ref aNF, true));
        Assert.Same(aBefore, Tool.Surface(anOF));
        Assert.NotSame(aBefore, Tool.Surface(aNF));
    }
}
