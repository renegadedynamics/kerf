// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text;
using OcctNet.ModelingData.Tests;
using static Kerf.Engine.Healing.Checks.StatusCheck;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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

    private static List<StatusCheck> L(NCollection_List<StatusCheck> theList) => [.. theList];

    // ---------------------------------------------------------------------------------------------------------------
    // Shapes

    private static Face PlaneFace(Surface? theSurface = null)
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, theSurface ?? new PlaneSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))), Tol);
        return aF;
    }

    private static Vertex V(double x, double y) => TestShapes.Vertex(new Point3D(x, y, 0));

    /// <summary>A segment between two vertices of the plane z = 0 with its pcurve on the face (UV = XY).</summary>
    private static Edge Seg(in Face theFace, in Vertex theV1, in Vertex theV2, double theShift = 0)
    {
        var aB = new BrepBuilder();
        Point3D aP1 = Tool.Pnt(theV1), aP2 = Tool.Pnt(theV2);
        Edge anE = TestShapes.Edge(theV1, theV2);
        aB.UpdateEdge(anE,
                      new LineCurve2D(new Point2D(aP1.X(), aP1.Y() + theShift), new Direction2D(aP2.X() - aP1.X(), aP2.Y() - aP1.Y())),
                      theFace, Tol);
        return anE;
    }

    /// <summary>A wire of segments through the corners of the plane z = 0, closed unless told otherwise.</summary>
    private static Wire Polygon(in Face theFace, bool theClosed, params (double X, double Y)[] theCorners)
    {
        var aB = new BrepBuilder();
        int n = theCorners.Length;
        var aV = theCorners.Select(c => V(c.X, c.Y)).ToArray();
        Wire aW = new();
        aB.MakeWire(ref aW);
        for (int i = 0; i < (theClosed ? n : n - 1); i++)
        {
            aB.Add(aW, Seg(theFace, aV[i], aV[(i + 1) % n]));
        }

        aW.Closed(theClosed);
        return aW;
    }

    private static Face Square(params Wire[] theHoles)
    {
        Face aF = PlaneFace();
        var aB = new BrepBuilder();
        aB.Add(aF, Polygon(aF, true, (0, 0), (10, 0), (10, 10), (0, 10)));
        foreach (Wire aW in theHoles)
        {
            aB.Add(aF, aW);
        }

        return aF;
    }

    /// <summary>The unit square with its first edge built by <paramref name="theEdge"/> (from vertices (0,0), (10,0)).</summary>
    private static (Face Face, Edge Edge) SquareWithEdge(Func<Face, Vertex, Vertex, Edge> theEdge)
    {
        Face aF = PlaneFace();
        var aB = new BrepBuilder();
        Vertex v0 = V(0, 0), v1 = V(10, 0), v2 = V(10, 10), v3 = V(0, 10);
        Edge anE = theEdge(aF, v0, v1);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aW, Seg(aF, v1, v2));
        aB.Add(aW, Seg(aF, v2, v3));
        aB.Add(aW, Seg(aF, v3, v0));
        aW.Closed(true);
        aB.Add(aF, aW);
        return (TestShapes.Frozen(aF), anE);
    }

    private static Shell FirstShell(in Shape theSolid) => ShapeCasts.Shell(new Iterator(theSolid).Value());

    private static List<Shape> Faces(in Shape theShape)
    {
        var aFaces = new List<Shape>();
        for (var anIt = new Iterator(FirstShell(theShape)); anIt.More(); anIt.Next())
        {
            aFaces.Add(anIt.Value());
        }

        return aFaces;
    }

    private static Solid SolidOf(params Shape[] theChildren)
    {
        var aB = new BrepBuilder();
        Solid aSolid = new();
        aB.MakeSolid(ref aSolid);
        foreach (Shape aC in theChildren)
        {
            aB.Add(aSolid, aC);
        }

        return aSolid;
    }

    private static Shell ShellOf(IEnumerable<Shape> theFaces, bool theClosed)
    {
        var aB = new BrepBuilder();
        Shell aShell = new();
        aB.MakeShell(ref aShell);
        foreach (Shape aF in theFaces)
        {
            aB.Add(aShell, aF);
        }

        aShell.Closed(theClosed);
        return aShell;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Analysis in both modes

    /// <summary>Every status of every sub-shape (by MapShapes index), on its own and in each context, as text.</summary>
    private static string Dump(Analyzer theA, in Shape theShape)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, aMap);
        var aText = new StringBuilder();
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            ResultCheck? aR = theA.Result(aMap[i]);
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
    private static Analyzer Analyze(in Shape theShape, bool theGeomControls = true, bool theIsExact = false,
                                              bool theMayFail = false)
    {
        var aSerial = new Analyzer(theShape, theGeomControls, false, theIsExact);
        var aParallel = new Analyzer(theShape, theGeomControls, true, theIsExact);
        Assert.False(aSerial.IsParallel);
        Assert.True(aParallel.IsParallel);
        string aDump = Dump(aSerial, theShape);
        Assert.Equal(aDump, Dump(aParallel, theShape));
        Assert.Equal(aSerial.IsValid(), aParallel.IsValid());
        // and the parallel analysis run by OSD_Parallel's serial fallback (no threads, as in WebAssembly)
        using (global::Kerf.Engine.Foundation.OsdParallel.ForceSerialScope())
        {
            var aFallback = new Analyzer(theShape, theGeomControls, true, theIsExact);
            Assert.Equal(aDump, Dump(aFallback, theShape));
        }
        if (!theMayFail)
        {
            Assert.DoesNotContain(nameof(StatusCheck.CheckFail), aDump);
        }

        return aSerial;
    }

    private static readonly Orientation[] theOrientations = [Forward, Reversed, Internal, External];

    // The analyzer keeps a result per orientation met (OCCT's map compares with IsEqual); the statuses on the shape
    // itself are the same for all of them
    private static List<StatusCheck> StatusOf(Analyzer theA, Shape theSub)
        => L(theOrientations.Select(o => TryResult(theA, theSub.Oriented(o))).First(r => r is not null)!.Status());

    // ... and the contexts are on the orientation met first
    private static List<StatusCheck> StatusIn(Analyzer theA, Shape theSub, Shape theContext)
        => L(theOrientations.Select(o => TryResult(theA, theSub.Oriented(o)))
                            .First(r => r is not null && r.IsStatusOnShape(theContext))!
                            .StatusOnShape(theContext));

    private static ResultCheck? TryResult(Analyzer theA, in Shape theSub)
    {
        try
        {
            return theA.Result(theSub);
        }
        catch (global::Kerf.Engine.Foundation.NoSuchObjectException)
        {
            return null;
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Valid shapes

    [Fact]
    public void A_box_is_valid_with_every_sub_shape()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(2, 3, 4));
        Analyzer anA = Analyze(aBox);
        Assert.True(anA.IsValid());
        Assert.IsType<SolidCheck>(anA.Result(aBox));
        Assert.IsType<ShellCheck>(anA.Result(FirstShell(aBox)));
        for (var anExp = new Explorer(aBox, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.True(anA.IsValid(anExp.Current()));
            Assert.Equal([StatusCheck.NoError], StatusOf(anA, anExp.Current()));
        }

        Assert.False(anA.IsExactMethod);
        Assert.True(Analyze(aBox, false).IsValid());
        Assert.True(Analyze(aBox, true, true).IsValid());
        // located, reversed and in a compound
        Shape aPlaced = ClassificationTestShapes.RotatedZ(aBox, 0.7).Reversed();
        Compound aC = Builder.MakeCompound(aBox, aPlaced);
        Analyzer aCA = Analyze(aC);
        Assert.True(aCA.IsValid());
        Assert.Null(aCA.Result(aC));
    }

    [Fact]
    public void Faces_with_holes_seams_and_voids_are_valid()
    {
        Assert.True(Analyze(TestShapes.Frozen(ClassificationTestShapes.HoledSquare())).IsValid());
        Face aCyl = TestShapes.Frozen(TestShapes.CylinderFace(2, 5));
        Assert.True(Analyze(aCyl).IsValid());
        Assert.True(Analyze(aCyl.Reversed()).IsValid());
        Solid aVoid = TestShapes.Frozen(ClassificationTestShapes.BoxWithVoid(out _, out _));
        Analyzer anA = Analyze(aVoid);
        Assert.True(anA.IsValid());
        Assert.Equal([StatusCheck.NoError], StatusOf(anA, aVoid));
    }

    [Fact]
    public void The_analyzer_checks_a_null_shape_never()
    {
        Assert.Throws<global::Kerf.Engine.Foundation.NullObjectException>(() => new Analyzer(new Shape()));
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        var anA = new Analyzer(aBox);
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => anA.Result(TestShapes.Frozen(TestShapes.Box(1, 1, 1))));
        anA.SetParallel(true);
        anA.SetExactMethod(true);
        anA.Init(aBox, false);
        Assert.True(anA.IsParallel);
        Assert.True(anA.IsExactMethod);
        Assert.True(anA.IsValid());
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Vertices and edges

    [Fact]
    public void A_vertex_off_its_edge_is_an_invalid_point_on_curve()
    {
        // the line goes from (0,0,0) to (10,0,0); the second vertex is 0.1 above its end
        var aLine = new LineCurve3D(new Point3D(), new Direction3D(1, 0, 0));
        Vertex aV1 = TestShapes.Vertex(new Point3D());
        Vertex aV2 = TestShapes.Vertex(new Point3D(10, 0, 0.1));
        Edge anE = TestShapes.Frozen(TestShapes.Edge(aLine, 0, 10, aV1, aV2));
        Analyzer anA = Analyze(anE);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.NoError], StatusOf(anA, anE));
        Assert.Equal([StatusCheck.InvalidPointOnCurve], StatusIn(anA, aV2, anE));
        Assert.Equal([StatusCheck.NoError], StatusIn(anA, aV1, anE));
        var aCheck = (VertexCheck)TryResult(anA, aV2.Oriented(Reversed))!;
        // a vertex without point representations: its own tolerance, with OCCT's 5% margin on the square
        Assert.Equal(Math.Sqrt(global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion() * 1.05), aCheck.Tolerance(), 1e-20);

        // within a larger tolerance
        var aB = new BrepBuilder();
        Vertex aV3 = new();
        aB.MakeVertex(ref aV3, new Point3D(10, 0, 0.1), 0.2);
        Edge anE2 = TestShapes.Frozen(TestShapes.Edge(aLine, 0, 10, aV1, aV3));
        Assert.True(Analyze(anE2).IsValid());
    }

    [Fact]
    public void Edge_minimum_finds_bad_flags_ranges_and_curves()
    {
        var aB = new BrepBuilder();
        var aLine = new LineCurve3D(new Point3D(), new Direction3D(1, 0, 0));

        // a reversed range
        Edge aRange = TestShapes.Edge(aLine, 0, 10);
        aB.Range(aRange, 5, 1);
        aRange = TestShapes.Frozen(aRange);
        Analyzer anA = Analyze(aRange);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.InvalidRange], StatusOf(anA, aRange));

        // degenerated with a 3D curve
        // (BRep_Builder.Degenerated removes the 3D curve: put it back after)
        Edge aDeg = TestShapes.Edge(aLine, 0, 10);
        aB.Degenerated(aDeg, true);
        Edge aDegNoCurve = TestShapes.Edge(aLine, 0, 10);
        aB.Degenerated(aDegNoCurve, true);
        aDegNoCurve = TestShapes.Frozen(aDegNoCurve);
        Assert.Equal([StatusCheck.NoError], StatusOf(Analyze(aDegNoCurve), aDegNoCurve));
        aB.UpdateEdge(aDeg, aLine, Tol);
        aDeg = TestShapes.Frozen(aDeg);
        Assert.Equal([StatusCheck.InvalidDegeneratedFlag], StatusOf(Analyze(aDeg), aDeg));

        // same parameter without same range
        Edge aFlags = TestShapes.Edge(aLine, 0, 10);
        aB.SameRange(aFlags, false);
        aFlags = TestShapes.Frozen(aFlags);
        Assert.Equal([StatusCheck.InvalidSameParameterFlag], StatusOf(Analyze(aFlags), aFlags));

        // a range beyond a bounded curve; a trimmed curve is measured against its basis curve, as OCCT does
        var aPoles = new NCollection_Array1<Point3D>(1, 2);
        aPoles[1] = new Point3D();
        aPoles[2] = new Point3D(10, 0, 0);
        Edge aBeyond = TestShapes.Edge(new BezierCurve3D(aPoles), 0, 1);
        aB.Range(aBeyond, -1, 1);
        aBeyond = TestShapes.Frozen(aBeyond);
        Assert.Equal([StatusCheck.InvalidRange], StatusOf(Analyze(aBeyond), aBeyond));
        Edge aTrim = TestShapes.Edge(new TrimmedCurve3D(aLine, 0, 10), 0, 10);
        aB.Range(aTrim, -1, 10);
        aTrim = TestShapes.Frozen(aTrim);
        Assert.Equal([StatusCheck.NoError], StatusOf(Analyze(aTrim), aTrim));

        // no 3D curve: only a pcurve on a face
        (Face aFace, Edge aNo3D) = SquareWithEdge((f, v1, v2) =>
        {
            var aBB = new BrepBuilder();
            Edge anE = new();
            aBB.MakeEdge(ref anE);
            aBB.UpdateEdge(anE, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), f, Tol);
            aBB.Add(anE, v1.Oriented(Forward));
            aBB.Add(anE, v2.Oriented(Reversed));
            aBB.Range(anE, 0, 10);
            return anE;
        });
        anA = Analyze(aFace, theMayFail: true);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.No3DCurve, StatusCheck.CheckFail], StatusOf(anA, aNo3D));
        // the pcurve is the reference; with no other pcurve on the plane, OCCT projects the reference's 3D curve, which
        // it doesn't have: it dereferences a null handle, and the analyzer records a failure
        Assert.Equal([StatusCheck.CheckFail], StatusIn(anA, aNo3D, aFace));
        Assert.Contains(StatusCheck.CheckFail, StatusOf(anA, aFace));
        // without geometric controls there is no projection
        Assert.Equal([StatusCheck.NoError], StatusIn(Analyze(aFace, false), aNo3D, aFace));
    }

    [Fact]
    public void Edges_in_a_face_need_their_flags_and_a_matching_pcurve()
    {
        var aB = new BrepBuilder();
        // SameParameter cleared
        (Face aF1, Edge aE1) = SquareWithEdge((f, v1, v2) =>
        {
            Edge anE = Seg(f, v1, v2);
            aB.SameParameter(anE, false);
            return anE;
        });
        Analyzer anA = Analyze(aF1);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.NoError], StatusOf(anA, aE1));
        Assert.Equal([StatusCheck.InvalidSameParameterFlag], StatusIn(anA, aE1, aF1));
        Assert.True(anA.IsValid(aE1));
        Assert.False(anA.IsValid(aF1));

        // a pcurve 0.5 away from the 3D curve
        (Face aF2, Edge aE2) = SquareWithEdge((f, v1, v2) => Seg(f, v1, v2, 0.5));
        anA = Analyze(aF2);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.InvalidCurveOnSurface, StatusCheck.InvalidSameParameterFlag], StatusIn(anA, aE2, aF2));
        Assert.Equal([StatusCheck.UnorientableShape], StatusOf(anA, aF2));
        Assert.Equal(0.5 * 1.05, ((EdgeCheck)anA.Result(aE2)!).Tolerance(), 1e-9);
        // the exact method finds it too; without geometric controls only the topology is checked
        Assert.Equal([StatusCheck.InvalidCurveOnSurface, StatusCheck.InvalidSameParameterFlag],
                     StatusIn(Analyze(aF2, true, true), aE2, aF2));
        Analyzer aTopo = Analyze(aF2, false);
        Assert.True(aTopo.IsValid());
        Assert.False(((EdgeCheck)aTopo.Result(aE2)!).GeometricControls());

        // a pcurve just inside the edge's tolerance
        (Face aF3, _) = SquareWithEdge((f, v1, v2) => Seg(f, v1, v2, 0.5e-7));
        Assert.True(Analyze(aF3).IsValid());
    }

    [Fact]
    public void An_edge_without_pcurve_on_a_curved_face_has_no_curve_on_surface()
    {
        // TestShapes.CylinderFace without the top circle's pcurve
        var aB = new BrepBuilder();
        var aSurf = new CylindricalSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 2);
        double aTwoPi = 2.0 * Math.PI;
        Vertex aV0 = TestShapes.Vertex(new Point3D(2, 0, 0));
        Vertex aV1 = TestShapes.Vertex(new Point3D(2, 0, 3));
        Edge aBottom = TestShapes.Edge(new CircleCurve3D(new Frame3D(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 2),
                                              0.0, aTwoPi, aV0, aV0);
        Edge aTop = TestShapes.Edge(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 3), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 2),
                                           0.0, aTwoPi, aV1, aV1);
        Edge aSeam = TestShapes.Edge(new LineCurve3D(new Point3D(2, 0, 0), new Direction3D(0, 0, 1)), 0.0, 3, aV0, aV1);
        Face aF = new();
        aB.MakeFace(ref aF, aSurf, Tol);
        aB.UpdateEdge(aBottom, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, Tol);
        aB.UpdateEdge(aSeam, new LineCurve2D(new Point2D(aTwoPi, 0), new Direction2D(0, 1)),
                      new LineCurve2D(new Point2D(0, 0), new Direction2D(0, 1)), aF, Tol);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, aBottom);
        aB.Add(aW, aSeam);
        aB.Add(aW, aTop.Reversed());
        aB.Add(aW, aSeam.Reversed());
        aW.Closed(true);
        aB.Add(aF, aW);
        aF = TestShapes.Frozen(aF);

        Analyzer anA = Analyze(aF);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.NoCurveOnSurface], StatusIn(anA, aTop, aF));
        Assert.Equal([StatusCheck.NoError], StatusIn(anA, aSeam, aF));
        Assert.Equal([StatusCheck.UnorientableShape], StatusOf(anA, aF));
    }

    [Fact]
    public void Edges_in_a_solid_bound_two_faces()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        List<Shape> aFaces = Faces(aBox);
        Solid anOpen = TestShapes.Frozen(SolidOf(ShellOf(aFaces.Skip(1), false)));
        var anExp = new Explorer(aFaces[0], ShapeType.Edge);
        Edge aFree = ShapeCasts.Edge(anExp.Current());
        var aCheck = new EdgeCheck(aFree);
        aCheck.InContext(aBox);
        Assert.Equal([StatusCheck.NoError], L(aCheck.StatusOnShape(aBox)));
        aCheck.InContext(anOpen);
        Assert.Equal([StatusCheck.FreeEdge], L(aCheck.StatusOnShape(anOpen)));
        Solid aRepeated = TestShapes.Frozen(SolidOf(ShellOf(aFaces.Append(aFaces[0]), true)));
        aCheck.InContext(aRepeated);
        Assert.Equal([StatusCheck.InvalidMultiConnexity], L(aCheck.StatusOnShape(aRepeated)));
        Edge anOther = TestShapes.Frozen(TestShapes.Edge(new Point3D(), new Point3D(1, 1, 1)));
        aCheck.InContext(anOther);
        Assert.Equal([StatusCheck.SubshapeNotInShape], L(aCheck.StatusOnShape(anOther)));
        Assert.Equal(StatusCheck.NoError, aCheck.CheckPolygonOnTriangulation(aFree));
        aCheck.Blind();
        Assert.True(aCheck.IsBlind);
        Assert.True(aCheck.IsMinimum);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Wires and faces

    [Fact]
    public void An_open_wire_makes_its_face_unorientable()
    {
        Face aF = PlaneFace();
        Wire aW = Polygon(aF, false, (0, 0), (10, 0), (10, 10), (0, 10));
        new BrepBuilder().Add(aF, aW);
        aF = TestShapes.Frozen(aF);
        aW = ShapeCasts.Wire(new Iterator(aF).Value());
        Analyzer anA = Analyze(aF);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.NotClosed], StatusIn(anA, aW, aF));
        Assert.Equal([StatusCheck.UnorientableShape], StatusOf(anA, aF));
    }

    [Fact]
    public void A_figure_eight_wire_intersects_itself()
    {
        Face aF = PlaneFace();
        Wire aW = Polygon(aF, true, (0, 0), (10, 10), (10, 0), (0, 10));
        new BrepBuilder().Add(aF, aW);
        aF = TestShapes.Frozen(aF);
        aW = ShapeCasts.Wire(new Iterator(aF).Value());
        Analyzer anA = Analyze(aF);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.SelfIntersectingWire], StatusIn(anA, aW, aF));
        Assert.Equal([StatusCheck.UnorientableShape], StatusOf(anA, aF));
        // without geometric controls the wire is closed and well oriented
        Analyzer aTopo = Analyze(aF, false);
        Assert.Equal([StatusCheck.NoError], StatusIn(aTopo, aW, aF));
    }

    [Fact]
    public void A_hole_oriented_as_the_outer_wire_is_a_bad_orientation()
    {
        Face aF = PlaneFace();
        Wire aHole = Polygon(aF, true, (4, 4), (6, 4), (6, 6), (4, 6));
        aF = TestShapes.Frozen(Square(aHole));
        Analyzer anA = Analyze(aF);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.BadOrientationOfSubshape], StatusOf(anA, aF));
        var aCheck = (FaceCheck)anA.Result(aF)!;
        Assert.True(aCheck.IsUnorientable());
        Assert.Equal(StatusCheck.NoError, aCheck.IntersectWires());
        Assert.Equal(StatusCheck.NoError, aCheck.ClassifyWires());
        Assert.Equal(StatusCheck.BadOrientationOfSubshape, aCheck.OrientationOfWires());

        // the same face with the hole clockwise
        Face aGood = PlaneFace();
        Face aValid = TestShapes.Frozen(Square(Polygon(aGood, true, (4, 4), (4, 6), (6, 6), (6, 4))));
        Assert.True(Analyze(aValid).IsValid());
        Assert.True(Analyze(aValid.Reversed()).IsValid());
    }

    [Fact]
    public void Wires_that_cross_or_lie_apart_are_found()
    {
        Face aF = PlaneFace();
        // a hole across the outer boundary
        Face aCrossing = TestShapes.Frozen(Square(Polygon(aF, true, (8, 4), (8, 6), (12, 6), (12, 4))));
        Analyzer anA = Analyze(aCrossing);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.IntersectingWires], StatusOf(anA, aCrossing));

        // two outer wires
        Face aTwo = TestShapes.Frozen(Square(Polygon(aF, true, (20, 0), (30, 0), (30, 10), (20, 10))));
        anA = Analyze(aTwo);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.InvalidImbricationOfWires], StatusOf(anA, aTwo));

        // a hole outside the face
        Face anOutside = TestShapes.Frozen(Square(Polygon(aF, true, (20, 0), (20, 10), (30, 10), (30, 0))));
        anA = Analyze(anOutside);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.InvalidImbricationOfWires], StatusOf(anA, anOutside));

        // the same wire twice
        Wire anOuter = Polygon(aF, true, (0, 0), (10, 0), (10, 10), (0, 10));
        Face aTwice = PlaneFace();
        var aB = new BrepBuilder();
        aB.Add(aTwice, anOuter);
        aB.Add(aTwice, anOuter);
        aTwice = TestShapes.Frozen(aTwice);
        Assert.Equal(StatusCheck.RedundantWire, new FaceCheck(aTwice).IntersectWires());
    }

    [Fact]
    public void A_face_without_surface_has_no_surface()
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF);
        aF = TestShapes.Frozen(aF);
        Analyzer anA = Analyze(aF);
        Assert.False(anA.IsValid());
        // OrientationOfWires(true) finds no wire and records NoError, which BRepCheck.Add appends after NoSurface
        Assert.Equal([StatusCheck.NoSurface, StatusCheck.NoError], StatusOf(anA, aF));
        var aCheck = new FaceCheck(aF);
        Assert.True(aCheck.GeometricControls());
        aCheck.SetUnorientable();
        Assert.Equal([StatusCheck.NoSurface, StatusCheck.UnorientableShape], L(aCheck.Status()));
        aCheck.InContext(TestShapes.Frozen(TestShapes.Box(1, 1, 1)));
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Shells and solids

    [Fact]
    public void Reversed_missing_and_repeated_faces_break_the_shell_in_its_solid()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        List<Shape> aFaces = Faces(aBox);

        List<Shape> aReversed = [.. aFaces];
        aReversed[2] = aReversed[2].Reversed();
        Solid aS1 = TestShapes.Frozen(SolidOf(ShellOf(aReversed, true)));
        Analyzer anA = Analyze(aS1);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.BadOrientationOfSubshape], StatusIn(anA, FirstShell(aS1), aS1));
        Assert.True(anA.IsValid(FirstShell(aS1)));

        Solid aS2 = TestShapes.Frozen(SolidOf(ShellOf(aFaces.Skip(1), false)));
        anA = Analyze(aS2);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.NotClosed], StatusIn(anA, FirstShell(aS2), aS2));

        // the whole shell reversed: an inside-out solid, valid for the shell check
        Solid aS3 = TestShapes.Frozen(SolidOf(FirstShell(aBox).Reversed()));
        Assert.Equal([StatusCheck.NoError], StatusIn(Analyze(aS3), FirstShell(aS3), aS3));
    }

    [Fact]
    public void Solids_with_several_shells_or_stray_parts_are_found()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Shell aShell = FirstShell(aBox);
        Shape aFar = ClassificationTestShapes.Translated(aShell, new Vector3D(5, 0, 0));

        // two disjoint closed shells: two growths, and one outside the other
        Solid aTwo = TestShapes.Frozen(SolidOf(aShell, aFar));
        Analyzer anA = Analyze(aTwo);
        Assert.False(anA.IsValid());
        Assert.Equal([StatusCheck.EnclosedRegion, StatusCheck.SubshapeNotInShape], StatusOf(anA, aTwo));

        // a shell sharing faces with another
        Solid aShared = TestShapes.Frozen(SolidOf(aShell, ShellOf(Faces(aBox).Take(2), false)));
        Assert.Equal([StatusCheck.InvalidImbricationOfShells], StatusOf(Analyze(aShared), aShared));

        // a stray vertex, internal or not
        Vertex aV = TestShapes.Vertex(new Point3D(0.5, 0.5, 0.5));
        Solid aStray = TestShapes.Frozen(SolidOf(aShell, aV));
        Assert.Equal([StatusCheck.BadOrientationOfSubshape], StatusOf(Analyze(aStray), aStray));
        Solid anInternal = TestShapes.Frozen(SolidOf(aShell, aV.Oriented(Internal)));
        Assert.Equal([StatusCheck.NoError], StatusOf(Analyze(anInternal), anInternal));

        // only internal shells
        List<Shape> anInternalFaces = [.. Faces(aBox).Select(f => f.Oriented(Internal))];
        Solid anOnlyInternal = TestShapes.Frozen(SolidOf(ShellOf(anInternalFaces, true)));
        Assert.Contains(StatusCheck.BadOrientationOfSubshape, StatusOf(Analyze(anOnlyInternal), anOnlyInternal));

        // a box with a void is valid; a void outside is not in the shape
        Shape aFarVoid = ClassificationTestShapes.Translated(aShell, new Vector3D(5, 0, 0)).Reversed();
        Solid aVoidOutside = TestShapes.Frozen(SolidOf(aShell, aFarVoid));
        Assert.Equal([StatusCheck.SubshapeNotInShape], StatusOf(Analyze(aVoidOutside), aVoidOutside));
        var aSolidCheck = new SolidCheck(aVoidOutside);
        aSolidCheck.InContext(aBox);
        aSolidCheck.Blind();
        Assert.False(aSolidCheck.IsStatusOnShape(aBox));
    }

    [Fact]
    public void The_checked_shapes_are_left_as_they_were()
    {
        // frozen shapes would throw on any write, and a throw would show as CheckFail; the BRep text is unchanged too
        Solid aBox = TestShapes.Frozen(ClassificationTestShapes.BoxWithVoid(out _, out _));
        var aBefore = new StringWriter();
        BrepTools.Write(aBox, aBefore);
        Analyzer anA = Analyze(aBox);
        Assert.True(anA.IsValid());
        var anAfter = new StringWriter();
        BrepTools.Write(aBox, anAfter);
        Assert.Equal(aBefore.ToString(), anAfter.ToString());
    }
}
