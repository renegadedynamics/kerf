// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using OcctNet.ModelingData.Tests;
using static Occt.BRepCheck_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepCheck_Analyzer, Vertex, Edge, Face and Solid on frozen BRep_Builder shapes, valid and deliberately broken:
/// vertices off their curves, edges without 3D curve, with bad ranges, wrong flags, wrong or missing pcurves; open,
/// self-intersecting, intersecting and badly oriented wires; faces without surface; reversed and missing faces;
/// solids with several or shared shells and stray vertices. Every shape is checked in the serial and the parallel
/// mode, which must give the same statuses on every sub-shape in every context.
/// </summary>
public class BRepCheckAnalyzerTests
{
    private const double Tol = 1.0e-7;

    private static List<BRepCheck_Status> L(NCollection_List<BRepCheck_Status> theList) => [.. theList];

    // ---------------------------------------------------------------------------------------------------------------
    // Shapes

    private static TopoDS_Face PlaneFace(Geom_Surface? theSurface = null)
    {
        var aB = new BRep_Builder();
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, theSurface ?? new Geom_Plane(new gp_Ax3(new gp_Pnt(), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0))), Tol);
        return aF;
    }

    private static TopoDS_Vertex V(double x, double y) => TestShapes.Vertex(new gp_Pnt(x, y, 0));

    /// <summary>A segment between two vertices of the plane z = 0 with its pcurve on the face (UV = XY).</summary>
    private static TopoDS_Edge Seg(in TopoDS_Face theFace, in TopoDS_Vertex theV1, in TopoDS_Vertex theV2, double theShift = 0)
    {
        var aB = new BRep_Builder();
        gp_Pnt aP1 = BRep_Tool.Pnt(theV1), aP2 = BRep_Tool.Pnt(theV2);
        TopoDS_Edge anE = TestShapes.Edge(theV1, theV2);
        aB.UpdateEdge(anE,
                      new Geom2d_Line(new gp_Pnt2d(aP1.X(), aP1.Y() + theShift), new gp_Dir2d(aP2.X() - aP1.X(), aP2.Y() - aP1.Y())),
                      theFace, Tol);
        return anE;
    }

    /// <summary>A wire of segments through the corners of the plane z = 0, closed unless told otherwise.</summary>
    private static TopoDS_Wire Polygon(in TopoDS_Face theFace, bool theClosed, params (double X, double Y)[] theCorners)
    {
        var aB = new BRep_Builder();
        int n = theCorners.Length;
        var aV = theCorners.Select(c => V(c.X, c.Y)).ToArray();
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        for (int i = 0; i < (theClosed ? n : n - 1); i++)
        {
            aB.Add(aW, Seg(theFace, aV[i], aV[(i + 1) % n]));
        }

        aW.Closed(theClosed);
        return aW;
    }

    private static TopoDS_Face Square(params TopoDS_Wire[] theHoles)
    {
        TopoDS_Face aF = PlaneFace();
        var aB = new BRep_Builder();
        aB.Add(aF, Polygon(aF, true, (0, 0), (10, 0), (10, 10), (0, 10)));
        foreach (TopoDS_Wire aW in theHoles)
        {
            aB.Add(aF, aW);
        }

        return aF;
    }

    /// <summary>The unit square with its first edge built by <paramref name="theEdge"/> (from vertices (0,0), (10,0)).</summary>
    private static (TopoDS_Face Face, TopoDS_Edge Edge) SquareWithEdge(Func<TopoDS_Face, TopoDS_Vertex, TopoDS_Vertex, TopoDS_Edge> theEdge)
    {
        TopoDS_Face aF = PlaneFace();
        var aB = new BRep_Builder();
        TopoDS_Vertex v0 = V(0, 0), v1 = V(10, 0), v2 = V(10, 10), v3 = V(0, 10);
        TopoDS_Edge anE = theEdge(aF, v0, v1);
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aW, Seg(aF, v1, v2));
        aB.Add(aW, Seg(aF, v2, v3));
        aB.Add(aW, Seg(aF, v3, v0));
        aW.Closed(true);
        aB.Add(aF, aW);
        return (TestShapes.Frozen(aF), anE);
    }

    private static TopoDS_Shell FirstShell(in TopoDS_Shape theSolid) => TopoDS.Shell(new TopoDS_Iterator(theSolid).Value());

    private static List<TopoDS_Shape> Faces(in TopoDS_Shape theShape)
    {
        var aFaces = new List<TopoDS_Shape>();
        for (var anIt = new TopoDS_Iterator(FirstShell(theShape)); anIt.More(); anIt.Next())
        {
            aFaces.Add(anIt.Value());
        }

        return aFaces;
    }

    private static TopoDS_Solid SolidOf(params TopoDS_Shape[] theChildren)
    {
        var aB = new BRep_Builder();
        TopoDS_Solid aSolid = new();
        aB.MakeSolid(ref aSolid);
        foreach (TopoDS_Shape aC in theChildren)
        {
            aB.Add(aSolid, aC);
        }

        return aSolid;
    }

    private static TopoDS_Shell ShellOf(IEnumerable<TopoDS_Shape> theFaces, bool theClosed)
    {
        var aB = new BRep_Builder();
        TopoDS_Shell aShell = new();
        aB.MakeShell(ref aShell);
        foreach (TopoDS_Shape aF in theFaces)
        {
            aB.Add(aShell, aF);
        }

        aShell.Closed(theClosed);
        return aShell;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Analysis in both modes

    /// <summary>Every status of every sub-shape (by MapShapes index), on its own and in each context, as text.</summary>
    private static string Dump(BRepCheck_Analyzer theA, in TopoDS_Shape theShape)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, aMap);
        var aText = new StringBuilder();
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            BRepCheck_Result? aR = theA.Result(aMap[i]);
            aText.Append($"{i} {aMap[i].ShapeType()} valid={theA.IsValid(aMap[i])}");
            if (aR is null)
            {
                aText.AppendLine();
                continue;
            }

            aText.Append(" [" + string.Join(",", L(aR.Status())) + "]");
            var aContexts = new SortedDictionary<int, string>();
            for (aR.InitContextIterator(); aR.MoreShapeInContext(); aR.NextShapeInContext())
            {
                aContexts[aMap.FindIndex(aR.ContextualShape())] = string.Join(",", L(aR.StatusOnShape()));
            }

            foreach (KeyValuePair<int, string> aC in aContexts)
            {
                aText.Append($" in {aC.Key}: [{aC.Value}]");
            }

            aText.AppendLine();
        }

        return aText.ToString();
    }

    /// <summary>The serial analysis, checked against the parallel one and its serial fallback, status by status.</summary>
    private static BRepCheck_Analyzer Analyze(in TopoDS_Shape theShape, bool theGeomControls = true, bool theIsExact = false,
                                              bool theMayFail = false)
    {
        var aSerial = new BRepCheck_Analyzer(theShape, theGeomControls, false, theIsExact);
        var aParallel = new BRepCheck_Analyzer(theShape, theGeomControls, true, theIsExact);
        Assert.False(aSerial.IsParallel());
        Assert.True(aParallel.IsParallel());
        string aDump = Dump(aSerial, theShape);
        Assert.Equal(aDump, Dump(aParallel, theShape));
        Assert.Equal(aSerial.IsValid(), aParallel.IsValid());
        // and the parallel analysis run by OSD_Parallel's serial fallback (no threads, as in WebAssembly)
        using (OSD_Parallel.ForceSerialScope())
        {
            var aFallback = new BRepCheck_Analyzer(theShape, theGeomControls, true, theIsExact);
            Assert.Equal(aDump, Dump(aFallback, theShape));
        }
        if (!theMayFail)
        {
            Assert.DoesNotContain(nameof(BRepCheck_CheckFail), aDump);
        }

        return aSerial;
    }

    private static readonly TopAbs_Orientation[] theOrientations = [TopAbs_FORWARD, TopAbs_REVERSED, TopAbs_INTERNAL, TopAbs_EXTERNAL];

    // The analyzer keeps a result per orientation met (OCCT's map compares with IsEqual); the statuses on the shape
    // itself are the same for all of them
    private static List<BRepCheck_Status> StatusOf(BRepCheck_Analyzer theA, TopoDS_Shape theSub)
        => L(theOrientations.Select(o => TryResult(theA, theSub.Oriented(o))).First(r => r is not null)!.Status());

    // ... and the contexts are on the orientation met first
    private static List<BRepCheck_Status> StatusIn(BRepCheck_Analyzer theA, TopoDS_Shape theSub, TopoDS_Shape theContext)
        => L(theOrientations.Select(o => TryResult(theA, theSub.Oriented(o)))
                            .First(r => r is not null && r.IsStatusOnShape(theContext))!
                            .StatusOnShape(theContext));

    private static BRepCheck_Result? TryResult(BRepCheck_Analyzer theA, in TopoDS_Shape theSub)
    {
        try
        {
            return theA.Result(theSub);
        }
        catch (Standard_NoSuchObject)
        {
            return null;
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Valid shapes

    [Fact]
    public void A_box_is_valid_with_every_sub_shape()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(2, 3, 4));
        BRepCheck_Analyzer anA = Analyze(aBox);
        Assert.True(anA.IsValid());
        Assert.IsType<BRepCheck_Solid>(anA.Result(aBox));
        Assert.IsType<BRepCheck_Shell>(anA.Result(FirstShell(aBox)));
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.True(anA.IsValid(anExp.Current()));
            Assert.Equal([BRepCheck_NoError], StatusOf(anA, anExp.Current()));
        }

        Assert.False(anA.IsExactMethod());
        Assert.True(Analyze(aBox, false).IsValid());
        Assert.True(Analyze(aBox, true, true).IsValid());
        // located, reversed and in a compound
        TopoDS_Shape aPlaced = ClassificationTestShapes.RotatedZ(aBox, 0.7).Reversed();
        TopoDS_Compound aC = TopoDS_Builder.MakeCompound(aBox, aPlaced);
        BRepCheck_Analyzer aCA = Analyze(aC);
        Assert.True(aCA.IsValid());
        Assert.Null(aCA.Result(aC));
    }

    [Fact]
    public void Faces_with_holes_seams_and_voids_are_valid()
    {
        Assert.True(Analyze(TestShapes.Frozen(ClassificationTestShapes.HoledSquare())).IsValid());
        TopoDS_Face aCyl = TestShapes.Frozen(TestShapes.CylinderFace(2, 5));
        Assert.True(Analyze(aCyl).IsValid());
        Assert.True(Analyze(aCyl.Reversed()).IsValid());
        TopoDS_Solid aVoid = TestShapes.Frozen(ClassificationTestShapes.BoxWithVoid(out _, out _));
        BRepCheck_Analyzer anA = Analyze(aVoid);
        Assert.True(anA.IsValid());
        Assert.Equal([BRepCheck_NoError], StatusOf(anA, aVoid));
    }

    [Fact]
    public void The_analyzer_checks_a_null_shape_never()
    {
        Assert.Throws<Standard_NullObject>(() => new BRepCheck_Analyzer(new TopoDS_Shape()));
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var anA = new BRepCheck_Analyzer(aBox);
        Assert.ThrowsAny<Standard_Failure>(() => anA.Result(TestShapes.Frozen(TestShapes.Box(1, 1, 1))));
        anA.SetParallel(true);
        anA.SetExactMethod(true);
        anA.Init(aBox, false);
        Assert.True(anA.IsParallel());
        Assert.True(anA.IsExactMethod());
        Assert.True(anA.IsValid());
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Vertices and edges

    [Fact]
    public void A_vertex_off_its_edge_is_an_invalid_point_on_curve()
    {
        // the line goes from (0,0,0) to (10,0,0); the second vertex is 0.1 above its end
        var aLine = new Geom_Line(new gp_Pnt(), new gp_Dir(1, 0, 0));
        TopoDS_Vertex aV1 = TestShapes.Vertex(new gp_Pnt());
        TopoDS_Vertex aV2 = TestShapes.Vertex(new gp_Pnt(10, 0, 0.1));
        TopoDS_Edge anE = TestShapes.Frozen(TestShapes.Edge(aLine, 0, 10, aV1, aV2));
        BRepCheck_Analyzer anA = Analyze(anE);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_NoError], StatusOf(anA, anE));
        Assert.Equal([BRepCheck_InvalidPointOnCurve], StatusIn(anA, aV2, anE));
        Assert.Equal([BRepCheck_NoError], StatusIn(anA, aV1, anE));
        var aCheck = (BRepCheck_Vertex)TryResult(anA, aV2.Oriented(TopAbs_REVERSED))!;
        // a vertex without point representations: its own tolerance, with OCCT's 5% margin on the square
        Assert.Equal(Math.Sqrt(Precision.Confusion() * Precision.Confusion() * 1.05), aCheck.Tolerance(), 1e-20);

        // within a larger tolerance
        var aB = new BRep_Builder();
        TopoDS_Vertex aV3 = new();
        aB.MakeVertex(ref aV3, new gp_Pnt(10, 0, 0.1), 0.2);
        TopoDS_Edge anE2 = TestShapes.Frozen(TestShapes.Edge(aLine, 0, 10, aV1, aV3));
        Assert.True(Analyze(anE2).IsValid());
    }

    [Fact]
    public void Edge_minimum_finds_bad_flags_ranges_and_curves()
    {
        var aB = new BRep_Builder();
        var aLine = new Geom_Line(new gp_Pnt(), new gp_Dir(1, 0, 0));

        // a reversed range
        TopoDS_Edge aRange = TestShapes.Edge(aLine, 0, 10);
        aB.Range(aRange, 5, 1);
        aRange = TestShapes.Frozen(aRange);
        BRepCheck_Analyzer anA = Analyze(aRange);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_InvalidRange], StatusOf(anA, aRange));

        // degenerated with a 3D curve
        // (BRep_Builder.Degenerated removes the 3D curve: put it back after)
        TopoDS_Edge aDeg = TestShapes.Edge(aLine, 0, 10);
        aB.Degenerated(aDeg, true);
        TopoDS_Edge aDegNoCurve = TestShapes.Edge(aLine, 0, 10);
        aB.Degenerated(aDegNoCurve, true);
        aDegNoCurve = TestShapes.Frozen(aDegNoCurve);
        Assert.Equal([BRepCheck_NoError], StatusOf(Analyze(aDegNoCurve), aDegNoCurve));
        aB.UpdateEdge(aDeg, aLine, Tol);
        aDeg = TestShapes.Frozen(aDeg);
        Assert.Equal([BRepCheck_InvalidDegeneratedFlag], StatusOf(Analyze(aDeg), aDeg));

        // same parameter without same range
        TopoDS_Edge aFlags = TestShapes.Edge(aLine, 0, 10);
        aB.SameRange(aFlags, false);
        aFlags = TestShapes.Frozen(aFlags);
        Assert.Equal([BRepCheck_InvalidSameParameterFlag], StatusOf(Analyze(aFlags), aFlags));

        // a range beyond a bounded curve; a trimmed curve is measured against its basis curve, as OCCT does
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 2);
        aPoles[1] = new gp_Pnt();
        aPoles[2] = new gp_Pnt(10, 0, 0);
        TopoDS_Edge aBeyond = TestShapes.Edge(new Geom_BezierCurve(aPoles), 0, 1);
        aB.Range(aBeyond, -1, 1);
        aBeyond = TestShapes.Frozen(aBeyond);
        Assert.Equal([BRepCheck_InvalidRange], StatusOf(Analyze(aBeyond), aBeyond));
        TopoDS_Edge aTrim = TestShapes.Edge(new Geom_TrimmedCurve(aLine, 0, 10), 0, 10);
        aB.Range(aTrim, -1, 10);
        aTrim = TestShapes.Frozen(aTrim);
        Assert.Equal([BRepCheck_NoError], StatusOf(Analyze(aTrim), aTrim));

        // no 3D curve: only a pcurve on a face
        (TopoDS_Face aFace, TopoDS_Edge aNo3D) = SquareWithEdge((f, v1, v2) =>
        {
            var aBB = new BRep_Builder();
            TopoDS_Edge anE = new();
            aBB.MakeEdge(ref anE);
            aBB.UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), f, Tol);
            aBB.Add(anE, v1.Oriented(TopAbs_FORWARD));
            aBB.Add(anE, v2.Oriented(TopAbs_REVERSED));
            aBB.Range(anE, 0, 10);
            return anE;
        });
        anA = Analyze(aFace, theMayFail: true);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_No3DCurve, BRepCheck_CheckFail], StatusOf(anA, aNo3D));
        // the pcurve is the reference; with no other pcurve on the plane, OCCT projects the reference's 3D curve, which
        // it doesn't have: it dereferences a null handle, and the analyzer records a failure
        Assert.Equal([BRepCheck_CheckFail], StatusIn(anA, aNo3D, aFace));
        Assert.Contains(BRepCheck_CheckFail, StatusOf(anA, aFace));
        // without geometric controls there is no projection
        Assert.Equal([BRepCheck_NoError], StatusIn(Analyze(aFace, false), aNo3D, aFace));
    }

    [Fact]
    public void Edges_in_a_face_need_their_flags_and_a_matching_pcurve()
    {
        var aB = new BRep_Builder();
        // SameParameter cleared
        (TopoDS_Face aF1, TopoDS_Edge aE1) = SquareWithEdge((f, v1, v2) =>
        {
            TopoDS_Edge anE = Seg(f, v1, v2);
            aB.SameParameter(anE, false);
            return anE;
        });
        BRepCheck_Analyzer anA = Analyze(aF1);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_NoError], StatusOf(anA, aE1));
        Assert.Equal([BRepCheck_InvalidSameParameterFlag], StatusIn(anA, aE1, aF1));
        Assert.True(anA.IsValid(aE1));
        Assert.False(anA.IsValid(aF1));

        // a pcurve 0.5 away from the 3D curve
        (TopoDS_Face aF2, TopoDS_Edge aE2) = SquareWithEdge((f, v1, v2) => Seg(f, v1, v2, 0.5));
        anA = Analyze(aF2);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_InvalidCurveOnSurface, BRepCheck_InvalidSameParameterFlag], StatusIn(anA, aE2, aF2));
        Assert.Equal([BRepCheck_UnorientableShape], StatusOf(anA, aF2));
        Assert.Equal(0.5 * 1.05, ((BRepCheck_Edge)anA.Result(aE2)!).Tolerance(), 1e-9);
        // the exact method finds it too; without geometric controls only the topology is checked
        Assert.Equal([BRepCheck_InvalidCurveOnSurface, BRepCheck_InvalidSameParameterFlag],
                     StatusIn(Analyze(aF2, true, true), aE2, aF2));
        BRepCheck_Analyzer aTopo = Analyze(aF2, false);
        Assert.True(aTopo.IsValid());
        Assert.False(((BRepCheck_Edge)aTopo.Result(aE2)!).GeometricControls());

        // a pcurve just inside the edge's tolerance
        (TopoDS_Face aF3, _) = SquareWithEdge((f, v1, v2) => Seg(f, v1, v2, 0.5e-7));
        Assert.True(Analyze(aF3).IsValid());
    }

    [Fact]
    public void An_edge_without_pcurve_on_a_curved_face_has_no_curve_on_surface()
    {
        // TestShapes.CylinderFace without the top circle's pcurve
        var aB = new BRep_Builder();
        var aSurf = new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 2);
        double aTwoPi = 2.0 * Math.PI;
        TopoDS_Vertex aV0 = TestShapes.Vertex(new gp_Pnt(2, 0, 0));
        TopoDS_Vertex aV1 = TestShapes.Vertex(new gp_Pnt(2, 0, 3));
        TopoDS_Edge aBottom = TestShapes.Edge(new Geom_Circle(new gp_Ax2(new gp_Pnt(), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 2),
                                              0.0, aTwoPi, aV0, aV0);
        TopoDS_Edge aTop = TestShapes.Edge(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 3), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 2),
                                           0.0, aTwoPi, aV1, aV1);
        TopoDS_Edge aSeam = TestShapes.Edge(new Geom_Line(new gp_Pnt(2, 0, 0), new gp_Dir(0, 0, 1)), 0.0, 3, aV0, aV1);
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aSurf, Tol);
        aB.UpdateEdge(aBottom, new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), aF, Tol);
        aB.UpdateEdge(aSeam, new Geom2d_Line(new gp_Pnt2d(aTwoPi, 0), new gp_Dir2d(0, 1)),
                      new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(0, 1)), aF, Tol);
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, aBottom);
        aB.Add(aW, aSeam);
        aB.Add(aW, aTop.Reversed());
        aB.Add(aW, aSeam.Reversed());
        aW.Closed(true);
        aB.Add(aF, aW);
        aF = TestShapes.Frozen(aF);

        BRepCheck_Analyzer anA = Analyze(aF);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_NoCurveOnSurface], StatusIn(anA, aTop, aF));
        Assert.Equal([BRepCheck_NoError], StatusIn(anA, aSeam, aF));
        Assert.Equal([BRepCheck_UnorientableShape], StatusOf(anA, aF));
    }

    [Fact]
    public void Edges_in_a_solid_bound_two_faces()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        List<TopoDS_Shape> aFaces = Faces(aBox);
        TopoDS_Solid anOpen = TestShapes.Frozen(SolidOf(ShellOf(aFaces.Skip(1), false)));
        var anExp = new TopExp_Explorer(aFaces[0], TopAbs_EDGE);
        TopoDS_Edge aFree = TopoDS.Edge(anExp.Current());
        var aCheck = new BRepCheck_Edge(aFree);
        aCheck.InContext(aBox);
        Assert.Equal([BRepCheck_NoError], L(aCheck.StatusOnShape(aBox)));
        aCheck.InContext(anOpen);
        Assert.Equal([BRepCheck_FreeEdge], L(aCheck.StatusOnShape(anOpen)));
        TopoDS_Solid aRepeated = TestShapes.Frozen(SolidOf(ShellOf(aFaces.Append(aFaces[0]), true)));
        aCheck.InContext(aRepeated);
        Assert.Equal([BRepCheck_InvalidMultiConnexity], L(aCheck.StatusOnShape(aRepeated)));
        TopoDS_Edge anOther = TestShapes.Frozen(TestShapes.Edge(new gp_Pnt(), new gp_Pnt(1, 1, 1)));
        aCheck.InContext(anOther);
        Assert.Equal([BRepCheck_SubshapeNotInShape], L(aCheck.StatusOnShape(anOther)));
        Assert.Equal(BRepCheck_NoError, aCheck.CheckPolygonOnTriangulation(aFree));
        aCheck.Blind();
        Assert.True(aCheck.IsBlind());
        Assert.True(aCheck.IsMinimum());
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Wires and faces

    [Fact]
    public void An_open_wire_makes_its_face_unorientable()
    {
        TopoDS_Face aF = PlaneFace();
        TopoDS_Wire aW = Polygon(aF, false, (0, 0), (10, 0), (10, 10), (0, 10));
        new BRep_Builder().Add(aF, aW);
        aF = TestShapes.Frozen(aF);
        aW = TopoDS.Wire(new TopoDS_Iterator(aF).Value());
        BRepCheck_Analyzer anA = Analyze(aF);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_NotClosed], StatusIn(anA, aW, aF));
        Assert.Equal([BRepCheck_UnorientableShape], StatusOf(anA, aF));
    }

    [Fact]
    public void A_figure_eight_wire_intersects_itself()
    {
        TopoDS_Face aF = PlaneFace();
        TopoDS_Wire aW = Polygon(aF, true, (0, 0), (10, 10), (10, 0), (0, 10));
        new BRep_Builder().Add(aF, aW);
        aF = TestShapes.Frozen(aF);
        aW = TopoDS.Wire(new TopoDS_Iterator(aF).Value());
        BRepCheck_Analyzer anA = Analyze(aF);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_SelfIntersectingWire], StatusIn(anA, aW, aF));
        Assert.Equal([BRepCheck_UnorientableShape], StatusOf(anA, aF));
        // without geometric controls the wire is closed and well oriented
        BRepCheck_Analyzer aTopo = Analyze(aF, false);
        Assert.Equal([BRepCheck_NoError], StatusIn(aTopo, aW, aF));
    }

    [Fact]
    public void A_hole_oriented_as_the_outer_wire_is_a_bad_orientation()
    {
        TopoDS_Face aF = PlaneFace();
        TopoDS_Wire aHole = Polygon(aF, true, (4, 4), (6, 4), (6, 6), (4, 6));
        aF = TestShapes.Frozen(Square(aHole));
        BRepCheck_Analyzer anA = Analyze(aF);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_BadOrientationOfSubshape], StatusOf(anA, aF));
        var aCheck = (BRepCheck_Face)anA.Result(aF)!;
        Assert.True(aCheck.IsUnorientable());
        Assert.Equal(BRepCheck_NoError, aCheck.IntersectWires());
        Assert.Equal(BRepCheck_NoError, aCheck.ClassifyWires());
        Assert.Equal(BRepCheck_BadOrientationOfSubshape, aCheck.OrientationOfWires());

        // the same face with the hole clockwise
        TopoDS_Face aGood = PlaneFace();
        TopoDS_Face aValid = TestShapes.Frozen(Square(Polygon(aGood, true, (4, 4), (4, 6), (6, 6), (6, 4))));
        Assert.True(Analyze(aValid).IsValid());
        Assert.True(Analyze(aValid.Reversed()).IsValid());
    }

    [Fact]
    public void Wires_that_cross_or_lie_apart_are_found()
    {
        TopoDS_Face aF = PlaneFace();
        // a hole across the outer boundary
        TopoDS_Face aCrossing = TestShapes.Frozen(Square(Polygon(aF, true, (8, 4), (8, 6), (12, 6), (12, 4))));
        BRepCheck_Analyzer anA = Analyze(aCrossing);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_IntersectingWires], StatusOf(anA, aCrossing));

        // two outer wires
        TopoDS_Face aTwo = TestShapes.Frozen(Square(Polygon(aF, true, (20, 0), (30, 0), (30, 10), (20, 10))));
        anA = Analyze(aTwo);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_InvalidImbricationOfWires], StatusOf(anA, aTwo));

        // a hole outside the face
        TopoDS_Face anOutside = TestShapes.Frozen(Square(Polygon(aF, true, (20, 0), (20, 10), (30, 10), (30, 0))));
        anA = Analyze(anOutside);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_InvalidImbricationOfWires], StatusOf(anA, anOutside));

        // the same wire twice
        TopoDS_Wire anOuter = Polygon(aF, true, (0, 0), (10, 0), (10, 10), (0, 10));
        TopoDS_Face aTwice = PlaneFace();
        var aB = new BRep_Builder();
        aB.Add(aTwice, anOuter);
        aB.Add(aTwice, anOuter);
        aTwice = TestShapes.Frozen(aTwice);
        Assert.Equal(BRepCheck_RedundantWire, new BRepCheck_Face(aTwice).IntersectWires());
    }

    [Fact]
    public void A_face_without_surface_has_no_surface()
    {
        var aB = new BRep_Builder();
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF);
        aF = TestShapes.Frozen(aF);
        BRepCheck_Analyzer anA = Analyze(aF);
        Assert.False(anA.IsValid());
        // OrientationOfWires(true) finds no wire and records NoError, which BRepCheck.Add appends after NoSurface
        Assert.Equal([BRepCheck_NoSurface, BRepCheck_NoError], StatusOf(anA, aF));
        var aCheck = new BRepCheck_Face(aF);
        Assert.True(aCheck.GeometricControls());
        aCheck.SetUnorientable();
        Assert.Equal([BRepCheck_NoSurface, BRepCheck_UnorientableShape], L(aCheck.Status()));
        aCheck.InContext(TestShapes.Frozen(TestShapes.Box(1, 1, 1)));
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Shells and solids

    [Fact]
    public void Reversed_missing_and_repeated_faces_break_the_shell_in_its_solid()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        List<TopoDS_Shape> aFaces = Faces(aBox);

        List<TopoDS_Shape> aReversed = [.. aFaces];
        aReversed[2] = aReversed[2].Reversed();
        TopoDS_Solid aS1 = TestShapes.Frozen(SolidOf(ShellOf(aReversed, true)));
        BRepCheck_Analyzer anA = Analyze(aS1);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_BadOrientationOfSubshape], StatusIn(anA, FirstShell(aS1), aS1));
        Assert.True(anA.IsValid(FirstShell(aS1)));

        TopoDS_Solid aS2 = TestShapes.Frozen(SolidOf(ShellOf(aFaces.Skip(1), false)));
        anA = Analyze(aS2);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_NotClosed], StatusIn(anA, FirstShell(aS2), aS2));

        // the whole shell reversed: an inside-out solid, valid for the shell check
        TopoDS_Solid aS3 = TestShapes.Frozen(SolidOf(FirstShell(aBox).Reversed()));
        Assert.Equal([BRepCheck_NoError], StatusIn(Analyze(aS3), FirstShell(aS3), aS3));
    }

    [Fact]
    public void Solids_with_several_shells_or_stray_parts_are_found()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Shell aShell = FirstShell(aBox);
        TopoDS_Shape aFar = ClassificationTestShapes.Translated(aShell, new gp_Vec(5, 0, 0));

        // two disjoint closed shells: two growths, and one outside the other
        TopoDS_Solid aTwo = TestShapes.Frozen(SolidOf(aShell, aFar));
        BRepCheck_Analyzer anA = Analyze(aTwo);
        Assert.False(anA.IsValid());
        Assert.Equal([BRepCheck_EnclosedRegion, BRepCheck_SubshapeNotInShape], StatusOf(anA, aTwo));

        // a shell sharing faces with another
        TopoDS_Solid aShared = TestShapes.Frozen(SolidOf(aShell, ShellOf(Faces(aBox).Take(2), false)));
        Assert.Equal([BRepCheck_InvalidImbricationOfShells], StatusOf(Analyze(aShared), aShared));

        // a stray vertex, internal or not
        TopoDS_Vertex aV = TestShapes.Vertex(new gp_Pnt(0.5, 0.5, 0.5));
        TopoDS_Solid aStray = TestShapes.Frozen(SolidOf(aShell, aV));
        Assert.Equal([BRepCheck_BadOrientationOfSubshape], StatusOf(Analyze(aStray), aStray));
        TopoDS_Solid anInternal = TestShapes.Frozen(SolidOf(aShell, aV.Oriented(TopAbs_INTERNAL)));
        Assert.Equal([BRepCheck_NoError], StatusOf(Analyze(anInternal), anInternal));

        // only internal shells
        List<TopoDS_Shape> anInternalFaces = [.. Faces(aBox).Select(f => f.Oriented(TopAbs_INTERNAL))];
        TopoDS_Solid anOnlyInternal = TestShapes.Frozen(SolidOf(ShellOf(anInternalFaces, true)));
        Assert.Contains(BRepCheck_BadOrientationOfSubshape, StatusOf(Analyze(anOnlyInternal), anOnlyInternal));

        // a box with a void is valid; a void outside is not in the shape
        TopoDS_Shape aFarVoid = ClassificationTestShapes.Translated(aShell, new gp_Vec(5, 0, 0)).Reversed();
        TopoDS_Solid aVoidOutside = TestShapes.Frozen(SolidOf(aShell, aFarVoid));
        Assert.Equal([BRepCheck_SubshapeNotInShape], StatusOf(Analyze(aVoidOutside), aVoidOutside));
        var aSolidCheck = new BRepCheck_Solid(aVoidOutside);
        aSolidCheck.InContext(aBox);
        aSolidCheck.Blind();
        Assert.False(aSolidCheck.IsStatusOnShape(aBox));
    }

    [Fact]
    public void The_checked_shapes_are_left_as_they_were()
    {
        // frozen shapes would throw on any write, and a throw would show as CheckFail; the BRep text is unchanged too
        TopoDS_Solid aBox = TestShapes.Frozen(ClassificationTestShapes.BoxWithVoid(out _, out _));
        var aBefore = new StringWriter();
        BRepTools.Write(aBox, aBefore);
        BRepCheck_Analyzer anA = Analyze(aBox);
        Assert.True(anA.IsValid());
        var anAfter = new StringWriter();
        BRepTools.Write(aBox, anAfter);
        Assert.Equal(aBefore.ToString(), anAfter.ToString());
    }
}
