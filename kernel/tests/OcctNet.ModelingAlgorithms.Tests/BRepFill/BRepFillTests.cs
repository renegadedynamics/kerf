// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.TestHarness;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The BRepFill package statics (unit 5S1): ruled faces and shells, Axe, the reduced curvilinear abscissae and
/// SearchOrigin; frozen inputs copied before the pcurves are written (decision 897). Native parity is in
/// OcctNet.Oracle.Tests' BRepFillOracleTests.
/// </summary>
public class BRepFillTests
{
    private static TopoDS_Edge Segment(gp_Pnt theP1, gp_Pnt theP2) => new BRepBuilderAPI_MakeEdge(theP1, theP2).Edge();

    private static TopoDS_Wire Polygon(bool theClosed, params gp_Pnt[] thePoints)
    {
        BRepBuilderAPI_MakePolygon aP = new();
        foreach (gp_Pnt aQ in thePoints)
        {
            aP.Add(aQ);
        }
        if (theClosed)
        {
            aP.Close();
        }
        return aP.Wire();
    }

    private static TopoDS_Wire Square(double theSide, double theZ) =>
        Polygon(true, new gp_Pnt(0, 0, theZ), new gp_Pnt(theSide, 0, theZ), new gp_Pnt(theSide, theSide, theZ),
                new gp_Pnt(0, theSide, theZ));

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static bool Holds(in TopoDS_Shape theShape, TopoDS_TShape theTShape)
    {
        for (var anExp = new TopExp_Explorer(theShape, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            if (ReferenceEquals(anExp.Current().TShape(), theTShape))
            {
                return true;
            }
        }
        for (var anExp = new TopExp_Explorer(theShape, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            if (ReferenceEquals(anExp.Current().TShape(), theTShape))
            {
                return true;
            }
        }
        return false;
    }

    private static double Area(in TopoDS_Shape theShape)
    {
        GProp_GProps aProps = new();
        BRepGProp.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void Face_ParallelSegments_IsPlanarValidAndFrozen_InputsUnchanged()
    {
        TopoDS_Edge aE1 = Segment(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0));
        TopoDS_Edge aE2 = Segment(new gp_Pnt(0, 5, 0), new gp_Pnt(10, 5, 0));
        string aBefore1 = BRepTextComparison.Write(aE1), aBefore2 = BRepTextComparison.Write(aE2);

        TopoDS_Face aF = BRepFill.Face(aE1, aE2);

        Assert.True(aF.TShape()!.IsFrozen());
        Assert.True(new BRepCheck_Analyzer(aF).IsValid());
        Assert.Equal(50.0, Area(aF), 1e-9);
        Assert.Equal(4, Count(aF, TopAbs_EDGE));
        Geom_Surface aS = BRep_Tool.Surface(aF)!;
        for (double u = 0; u <= 1; u += 0.25)
        {
            for (double v = 0; v <= 1; v += 0.25)
            {
                Assert.Equal(0.0, aS.Value(u, v).Z(), 1e-12);
            }
        }

        // the caller's edges are not written: the face holds copies of them (and of their vertices, which
        // BRepLib::SameParameter rebuilds), so its BRep text is native's byte for byte (decision 897)
        Assert.Equal(aBefore1, BRepTextComparison.Write(aE1));
        Assert.Equal(aBefore2, BRepTextComparison.Write(aE2));
        Assert.False(Holds(aF, aE1.TShape()!));
        Assert.Equal(4, Count(aF, TopAbs_VERTEX));
    }

    [Fact]
    public void Face_SharedVertex_GivesDegeneratedEdge_AndKeepsSharing()
    {
        TopoDS_Vertex aA = new BRepBuilderAPI_MakeVertex(new gp_Pnt(0, 0, 0)).Vertex();
        TopoDS_Vertex aB = new BRepBuilderAPI_MakeVertex(new gp_Pnt(10, 0, 0)).Vertex();
        TopoDS_Vertex aC = new BRepBuilderAPI_MakeVertex(new gp_Pnt(10, 6, 2)).Vertex();
        TopoDS_Edge aE1 = new BRepBuilderAPI_MakeEdge(aA, aB).Edge();
        TopoDS_Edge aE2 = new BRepBuilderAPI_MakeEdge(aA, aC).Edge();

        TopoDS_Face aF = BRepFill.Face(aE1, aE2);

        Assert.True(new BRepCheck_Analyzer(aF).IsValid());
        int aNbDegenerated = 0;
        for (var anExp = new TopExp_Explorer(aF, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            if (BRep_Tool.Degenerated(TopoDS.Edge(anExp.Current())))
            {
                aNbDegenerated++;
            }
        }
        Assert.Equal(1, aNbDegenerated);
        // the shared vertex is one vertex in the face: three vertices in all
        Assert.Equal(3, Count(aF, TopAbs_VERTEX));
    }

    [Fact]
    public void Face_ClosedEdges_GiveSeam()
    {
        TopoDS_Edge aE1 = new BRepBuilderAPI_MakeEdge(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 5)).Edge();
        TopoDS_Edge aE2 = new BRepBuilderAPI_MakeEdge(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 8), new gp_Dir(0, 0, 1)), 3)).Edge();

        TopoDS_Face aF = BRepFill.Face(aE1, aE2);

        Assert.True(new BRepCheck_Analyzer(aF).IsValid());
        Assert.Equal(3, Count(aF, TopAbs_EDGE));
        int aNbSeams = 0;
        for (var anExp = new TopExp_Explorer(aF, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            if (BRep_Tool.IsClosed(TopoDS.Edge(anExp.Current()), aF))
            {
                aNbSeams++;
            }
        }
        Assert.Equal(2, aNbSeams); // the seam appears twice, with both orientations
        // the lateral area of a frustum of radii 5 and 3 and height 8
        Assert.Equal(Math.PI * (5 + 3) * Math.Sqrt(8 * 8 + 2 * 2), Area(aF), 1e-6);
    }

    [Fact]
    public void Shell_TwoSquares_GivesFourFacesTube()
    {
        TopoDS_Wire aW1 = Square(10, 0), aW2 = Square(10, 7);
        string aBefore1 = BRepTextComparison.Write(aW1), aBefore2 = BRepTextComparison.Write(aW2);

        TopoDS_Shell aS = BRepFill.Shell(aW1, aW2);

        Assert.True(aS.TShape()!.IsFrozen());
        Assert.True(new BRepCheck_Analyzer(aS).IsValid());
        Assert.Equal(4, Count(aS, TopAbs_FACE));
        Assert.Equal(12, Count(aS, TopAbs_EDGE));
        Assert.Equal(8, Count(aS, TopAbs_VERTEX));
        Assert.False(aS.Closed());
        Assert.Equal(4 * 10 * 7, Area(aS), 1e-9);
        Assert.Equal(aBefore1, BRepTextComparison.Write(aW1));
        Assert.Equal(aBefore2, BRepTextComparison.Write(aW2));
    }

    [Fact]
    public void Axe_OnSquareSpine()
    {
        TopoDS_Wire aSpine = Square(10, 0);

        // a profile starting at the middle of the edge y = 0, standing up
        TopoDS_Wire aProfile = Polygon(false, new gp_Pnt(4, 0, 0), new gp_Pnt(4, 0, 3), new gp_Pnt(6, 0, 3));
        BRepFill.Axe(aSpine, aProfile, out gp_Ax3 anAx, out bool isOn, 1e-7);
        Assert.True(isOn);
        Assert.True(anAx.Location().IsEqual(new gp_Pnt(4, 0, 0), 1e-12));
        Assert.Equal(1.0, Math.Abs(anAx.Direction().Z()), 1e-12);
        Assert.Equal(1.0, Math.Abs(anAx.XDirection().X()), 1e-12);

        // a profile away from the spine
        TopoDS_Wire aFar = Polygon(false, new gp_Pnt(12, 5, 2), new gp_Pnt(12, 5, 6));
        BRepFill.Axe(aSpine, aFar, out anAx, out isOn, 1e-7);
        Assert.False(isOn);
        Assert.True(anAx.Location().IsEqual(new gp_Pnt(10, 5, 0), 1e-9));
        Assert.Equal(1.0, Math.Abs(anAx.XDirection().Y()), 1e-12);

        // a profile on a spine vertex
        TopoDS_Wire aOnVertex = Polygon(false, new gp_Pnt(10, 10, 0), new gp_Pnt(10, 10, 4), new gp_Pnt(11, 10, 4));
        BRepFill.Axe(aSpine, aOnVertex, out anAx, out isOn, 1e-7);
        Assert.True(isOn);
        Assert.True(anAx.Location().IsEqual(new gp_Pnt(10, 10, 0), 1e-12));
    }

    [Fact]
    public void Axe_NonPlanarFace_Throws()
    {
        TopoDS_Face aCyl = new BRepBuilderAPI_MakeFace(new Geom_CylindricalSurface(new gp_Ax3(), 5), 0, 2, 0, 3, 1e-7).Face();
        TopoDS_Wire aProfile = Polygon(false, new gp_Pnt(5, 0, 0), new gp_Pnt(6, 0, 0));
        Assert.ThrowsAny<Standard_NoSuchObject>(() => BRepFill.Axe(aCyl, aProfile, out _, out _, 1e-7));
    }

    private static TopoDS_Wire Polyline345() =>
        Polygon(false, new gp_Pnt(0, 0, 0), new gp_Pnt(3, 0, 0), new gp_Pnt(3, 4, 0), new gp_Pnt(3, 4, 5));

    [Fact]
    public void ComputeACR_Polyline()
    {
        var aACR = new NCollection_Array1<double>(0, 3);
        BRepFill.ComputeACR(Polyline345(), aACR);
        Assert.Equal(12.0, aACR[0], 1e-12);
        Assert.Equal(3.0 / 12, aACR[1], 1e-12);
        Assert.Equal(7.0 / 12, aACR[2], 1e-12);
        Assert.Equal(1.0, aACR[3], 1e-12);
    }

    [Fact]
    public void InsertACR_CutsTheEdges()
    {
        TopoDS_Wire aW = Polyline345();
        string aBefore = BRepTextComparison.Write(aW);
        var aCuts = new NCollection_Array1<double>(1, 4);
        aCuts[1] = 0.1;          // on the first edge
        aCuts[2] = 0.25;         // the first edge's end: no cut
        aCuts[3] = 0.5;          // on the second edge
        aCuts[4] = 0.9;          // on the third edge

        TopoDS_Wire aRes = BRepFill.InsertACR(aW, aCuts, 1e-6);

        Assert.True(aRes.TShape()!.IsFrozen());
        Assert.Equal(6, Count(aRes, TopAbs_EDGE));
        var aACR = new NCollection_Array1<double>(0, 6);
        BRepFill.ComputeACR(aRes, aACR);
        Assert.Equal(12.0, aACR[0], 1e-9);
        Assert.Equal(0.1, aACR[1], 1e-9);
        Assert.Equal(0.25, aACR[2], 1e-9);
        Assert.Equal(0.5, aACR[3], 1e-9);
        Assert.Equal(7.0 / 12, aACR[4], 1e-9);
        Assert.Equal(0.9, aACR[5], 1e-9);
        Assert.Equal(aBefore, BRepTextComparison.Write(aW));

        // with a precision larger than the distance to the ends, nothing is cut
        var aNear = new NCollection_Array1<double>(1, 1);
        aNear[1] = 0.26;
        Assert.Equal(3, Count(BRepFill.InsertACR(aW, aNear, 0.05), TopAbs_EDGE));
    }

    [Fact]
    public void SearchOrigin_CutsAndOrients()
    {
        TopoDS_Wire aW = Square(10, 0);
        string aBefore = BRepTextComparison.Write(aW);

        // a point in the middle of the edge y = 0: the edge is cut there and the wire starts at the new vertex
        TopoDS_Wire aCut = aW;
        BRepFill.SearchOrigin(ref aCut, new gp_Pnt(5, 0, 0), new gp_Vec(1, 0, 0), 1e-7);
        Assert.Equal(5, Count(aCut, TopAbs_EDGE));
        Assert.True(aCut.Closed());
        var anExp = new BRepTools_WireExplorer(aCut);
        Assert.True(BRep_Tool.Pnt(anExp.CurrentVertex()).IsEqual(new gp_Pnt(5, 0, 0), 1e-9));
        TopoDS_Vertex aV1 = TopExp.LastVertex(anExp.Current(), true);
        Assert.True(BRep_Tool.Pnt(aV1).IsEqual(new gp_Pnt(10, 0, 0), 1e-9)); // along +X

        // a point on a vertex, against the wire's direction: the same four edges, reversed, nothing cut (the
        // reversed wire is rebuilt from the edge starting at the vertex, so it starts at that edge's other end)
        TopoDS_Wire aRev = aW;
        BRepFill.SearchOrigin(ref aRev, new gp_Pnt(10, 10, 0), new gp_Vec(1, 0, 0), 1e-7);
        Assert.Equal(4, Count(aRev, TopAbs_EDGE));
        Assert.Equal(4, Count(aRev, TopAbs_VERTEX));
        Assert.True(aRev.Closed());

        Assert.Equal(aBefore, BRepTextComparison.Write(aW));
        Assert.Throws<Standard_NoSuchObject>(() =>
        {
            TopoDS_Wire anOpen = Polyline345();
            BRepFill.SearchOrigin(ref anOpen, new gp_Pnt(0, 0, 0), new gp_Vec(1, 0, 0), 1e-7);
        });
    }

    [Fact]
    public void InternalFace_UnfrozenInputs_WrittenInPlace()
    {
        // ported code's own unfrozen edges are used and written in place, as in OCCT
        TopoDS_Edge aE1 = new BRepLib_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0)).InternalEdge();
        TopoDS_Edge aE2 = new BRepLib_MakeEdge(new gp_Pnt(0, 5, 3), new gp_Pnt(10, 5, 3)).InternalEdge();
        Assert.False(aE1.TShape()!.IsFrozen());

        TopoDS_Face aF = BRepFill.InternalFace(aE1, aE2);

        Assert.False(aF.TShape()!.IsFrozen());
        Assert.True(Holds(aF, aE1.TShape()!));
        Assert.True(Holds(aF, aE2.TShape()!));
        Assert.NotNull(BRep_Tool.CurveOnSurface(aE1, aF, out _, out _));

        TopoDS_Shell aS = BRepFill.InternalShell(
            TopoDS.Wire(new BRepLib_MakePolygon(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0)).InternalWire()),
            TopoDS.Wire(new BRepLib_MakePolygon(new gp_Pnt(0, 0, 1), new gp_Pnt(1, 0, 1), new gp_Pnt(1, 1, 1)).InternalWire()));
        Assert.False(aS.TShape()!.IsFrozen());
        Assert.Equal(2, Count(aS, TopAbs_FACE));
    }
}
