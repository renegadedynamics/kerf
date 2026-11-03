// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of ShapeConstruct's statics, ShapeConstruct_Curve and ShapeConstruct_MakeTriangulation, and of
/// Geom2dAPI_Interpolate and PointsToBSpline; frozen inputs stay unchanged and public results are frozen.
/// </summary>
public class ShapeConstructTests
{
    private const double Tol = 1e-9;

    private static TrimmedCurve3D Segment(Point3D theP1, Point3D theP2) => new MakeSegment(theP1, theP2).Value();

    private static BSplineCurve3D CubicThrough(params Point3D[] thePnts)
    {
        NCollection_Array1<Point3D> aPoles = new(1, thePnts.Length);
        for (int i = 0; i < thePnts.Length; i++)
        {
            aPoles[i + 1] = thePnts[i];
        }
        return new PointsToBSpline(aPoles).Curve();
    }

    [Fact]
    public void ConvertCurveToBSpline_CircleApproximatedAndFrozen()
    {
        CircleCurve3D aCircle = new(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)), 5.0);
        BSplineCurve3D? aBSpl = global::Kerf.Engine.Healing.Construct.ShapeConstruct.ConvertCurveToBSpline(aCircle, 0.0, Math.PI, 1e-6, C1, 100, 9);
        Assert.NotNull(aBSpl);
        Assert.True(aBSpl.IsFrozen);
        Assert.True(aBSpl.Degree() <= 6); // conics are limited to degree 6
        for (int i = 0; i <= 10; i++)
        {
            double t = aBSpl.FirstParameter() + i * (aBSpl.LastParameter() - aBSpl.FirstParameter()) / 10;
            Point3D aP = aBSpl.Value(t);
            Assert.Equal(5.0, aP.Distance(new Point3D(1, 2, aP.Z())), 1e-5);
            Assert.Equal(3.0, aP.Z(), 1e-9);
        }

        // a B-spline is returned as it is
        BSplineCurve3D aCubic = CubicThrough(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0), new Point3D(3, 1, 0));
        Assert.Same(aCubic, global::Kerf.Engine.Healing.Construct.ShapeConstruct.ConvertCurveToBSpline(aCubic, 0.2, 0.8, 1e-6, C1, 100, 9));
        // internal form: a new curve stays unfrozen for ported code
        BSplineCurve3D? anInternal = global::Kerf.Engine.Healing.Construct.ShapeConstruct.InternalConvertCurveToBSpline(aCircle, 0.0, Math.PI, 1e-6, C1, 100, 9);
        Assert.False(anInternal!.IsFrozen);
    }

    [Fact]
    public void ConvertCurveToBSpline2d_ConicAndLine()
    {
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0);
        BSplineCurve2D? aBSpl = global::Kerf.Engine.Healing.Construct.ShapeConstruct.ConvertCurveToBSpline(aCircle, 0.0, Math.PI / 2, 1e-7, C1, 100, 9);
        Assert.NotNull(aBSpl);
        Assert.True(aBSpl.IsFrozen);
        Assert.True(aBSpl.Value(aBSpl.FirstParameter()).IsEqual(new Point2D(2, 0), 1e-6));
        Assert.True(aBSpl.Value(aBSpl.LastParameter()).IsEqual(new Point2D(0, 2), 1e-6));

        TrimmedCurve2D aSeg = new(new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), 0, 4);
        BSplineCurve2D? aLin = global::Kerf.Engine.Healing.Construct.ShapeConstruct.ConvertCurveToBSpline(aSeg, 0, 4, 1e-7, C1, 100, 9);
        Assert.Equal(1, aLin!.Degree());
        Assert.True(aLin.Value(aLin.LastParameter()).IsEqual(new Point2D(4, 0), Tol));
    }

    [Fact]
    public void ConvertSurfaceToBSpline_ElementaryAndExtrusion()
    {
        CylindricalSurface aCyl = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 3.0);
        BSplineSurface? aBS = global::Kerf.Engine.Healing.Construct.ShapeConstruct.ConvertSurfaceToBSpline(aCyl, 0, Math.PI, 0, 5, 1e-6, C1, 100, 9);
        Assert.NotNull(aBS);
        Assert.True(aBS.IsFrozen);
        aBS.Bounds(out double u1, out double u2, out double v1, out double v2);
        for (int i = 0; i <= 4; i++)
        {
            Point3D aP = aBS.Value(u1 + i * (u2 - u1) / 4, 0.5 * (v1 + v2));
            Assert.Equal(3.0, Math.Sqrt(aP.X() * aP.X() + aP.Y() * aP.Y()), 1e-9);
        }

        // an extrusion of a B-spline: two rows of poles shifted along the direction
        BSplineCurve3D aBasis = CubicThrough(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0), new Point3D(3, 1, 0));
        ExtrusionSurface anExtr = new(aBasis, new Direction3D(0, 0, 1));
        BSplineSurface? anExtrBS = global::Kerf.Engine.Healing.Construct.ShapeConstruct.ConvertSurfaceToBSpline(anExtr, aBasis.FirstParameter(), aBasis.LastParameter(),
                                                                               -1, 2, 1e-6, C2, 100, 9);
        Assert.NotNull(anExtrBS);
        Assert.Equal(1, anExtrBS.VDegree());
        Assert.Equal(2, anExtrBS.VPoleCount);
        Assert.True(anExtrBS.Value(aBasis.FirstParameter(), 2).IsEqual(new Point3D(0, 0, 2), Tol));
        Assert.True(anExtrBS.Value(aBasis.LastParameter(), -1).IsEqual(new Point3D(3, 1, -1), Tol));
    }

    [Fact]
    public void JoinCurves3d_SegmentsJoinedAndInputsUnchanged()
    {
        BSplineCurve3D aC1 = CubicThrough(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0), new Point3D(3, 1, 0));
        BSplineCurve3D aC2 = CubicThrough(new Point3D(3, 1, 0), new Point3D(4, 0, 0), new Point3D(5, 1, 0), new Point3D(6, 0, 0));
        aC1.Freeze();
        aC2.Freeze();
        Point3D aPole1 = aC1.Pole(aC1.PoleCount);
        double f1 = aC1.FirstParameter(), l1 = aC1.LastParameter(), f2 = aC2.FirstParameter(), l2 = aC2.LastParameter();
        bool isDone = global::Kerf.Engine.Healing.Construct.ShapeConstruct.JoinCurves(aC1, aC2, Forward, Forward, ref f1, ref l1, ref f2, ref l2,
                                                out Curve3D? aJoined, out bool isRev1, out bool isRev2);
        Assert.True(isDone);
        Assert.False(isRev1);
        Assert.False(isRev2);
        Assert.NotNull(aJoined);
        Assert.True(aJoined.IsFrozen);
        Assert.True(aJoined.Value(aJoined.FirstParameter()).IsEqual(new Point3D(0, 0, 0), 1e-7));
        Assert.True(aJoined.Value(aJoined.LastParameter()).IsEqual(new Point3D(6, 0, 0), 1e-7));
        // the inputs were copied before their poles were moved
        Assert.True(aC1.Pole(aC1.PoleCount).IsEqual(aPole1, 0.0));

        // the second edge reversed: its curve is reversed with its range
        Curve3D aC2Rev = aC2.Reversed();
        double g1 = aC1.FirstParameter(), m1 = aC1.LastParameter(), g2 = aC2Rev.FirstParameter(), m2 = aC2Rev.LastParameter();
        Assert.True(global::Kerf.Engine.Healing.Construct.ShapeConstruct.JoinCurves(aC1, aC2Rev, Forward, Reversed, ref g1, ref m1, ref g2, ref m2,
                                              out Curve3D? aJoined2, out _, out _));
        Assert.True(aJoined2!.Value(aJoined2.LastParameter()).IsEqual(new Point3D(6, 0, 0), 1e-7));
    }

    [Fact]
    public void JoinCurves2d_LinesJoined()
    {
        LineCurve2D aL1 = new(new Point2D(0, 0), new Direction2D(1, 0));
        LineCurve2D aL2 = new(new Point2D(2, 0), new Direction2D(0, 1));
        aL1.Freeze();
        aL2.Freeze();
        double f1 = 0, l1 = 2, f2 = 0, l2 = 3;
        Assert.True(global::Kerf.Engine.Healing.Construct.ShapeConstruct.JoinCurves(aL1, aL2, Forward, Forward, ref f1, ref l1, ref f2, ref l2,
                                              out Curve2D? aJoined, out bool isRev1, out bool isRev2));
        Assert.False(isRev1 || isRev2);
        Assert.True(aJoined!.IsFrozen);
        Assert.True(aJoined.Value(aJoined.FirstParameter()).IsEqual(new Point2D(0, 0), 1e-9));
        Assert.True(aJoined.Value(aJoined.LastParameter()).IsEqual(new Point2D(2, 3), 1e-9));
        Assert.True(aL1.Location().IsEqual(new Point2D(0, 0), 0.0));
    }

    [Fact]
    public void Curve_AdjustInPlaceThrowsOnFrozenAndTwinCopies()
    {
        global::Kerf.Engine.Healing.Construct.ConstructCurve aSCC = new();
        BSplineCurve3D aC = CubicThrough(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0), new Point3D(3, 1, 0));

        BSplineCurve3D aCopy = (BSplineCurve3D)aC.Copy();
        Assert.True(aSCC.AdjustCurve(aCopy, new Point3D(0, 0, 1), new Point3D(3, 1, 1)));
        Assert.True(aCopy.Value(aCopy.FirstParameter()).IsEqual(new Point3D(0, 0, 1), Tol));
        Assert.True(aCopy.Value(aCopy.LastParameter()).IsEqual(new Point3D(3, 1, 1), Tol));

        aC.Freeze();
        Assert.ThrowsAny<ImmutableObjectException>(() => aSCC.AdjustCurve(aC, new Point3D(0, 0, 1), new Point3D(3, 1, 1)));

        Curve3D aTwin = aSCC.CurveAdjusted(aC, new Point3D(0, 0, 1), new Point3D(3, 1, 1), out bool isDone);
        Assert.True(isDone);
        Assert.NotSame(aC, aTwin);
        Assert.True(aTwin.IsFrozen);
        Assert.True(aTwin.Value(aTwin.FirstParameter()).IsEqual(new Point3D(0, 0, 1), Tol));
        Assert.True(aC.Value(aC.FirstParameter()).IsEqual(new Point3D(0, 0, 0), Tol));

        // a line is moved through both points
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Curve3D aMoved = aSCC.CurveAdjusted(aLine, new Point3D(0, 1, 0), new Point3D(1, 2, 0), out isDone);
        Assert.True(isDone);
        Assert.True(((LineCurve3D)aMoved).Lin().Direction().IsEqual(new Direction3D(1, 1, 0), 1e-12));

        // a circle can't be adjusted: the curve itself comes back
        CircleCurve3D aCircle = new(new Frame3D(), 1);
        Assert.Same(aCircle, aSCC.CurveAdjusted(aCircle, new Point3D(), new Point3D(1, 0, 0), out isDone));
        Assert.False(isDone);

        // segment then adjust
        Curve3D aSeg = aSCC.CurveSegmentAdjusted(aC, new Point3D(1, 0, 0), new Point3D(2, 0, 0), 0.25, 0.75, out isDone);
        Assert.True(isDone);
        Assert.Equal(0.25, aSeg.FirstParameter(), Tol);
        Assert.True(aSeg.Value(0.75).IsEqual(new Point3D(2, 0, 0), Tol));

        // 2D
        LineCurve2D aLine2d = new(new Point2D(0, 0), new Direction2D(1, 0));
        aLine2d.Freeze();
        Curve2D aMoved2d = aSCC.Curve2dAdjusted(aLine2d, new Point2D(0, 1), new Point2D(1, 1), out isDone);
        Assert.True(isDone);
        Assert.True(((LineCurve2D)aMoved2d).Location().IsEqual(new Point2D(0, 1), 1e-12));
    }

    [Fact]
    public void Curve_ConvertToBSplineAndFixKnots()
    {
        global::Kerf.Engine.Healing.Construct.ConstructCurve aSCC = new();
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        BSplineCurve3D? aBS = aSCC.ConvertToBSpline(aLine, 1, 3, 1e-7);
        Assert.NotNull(aBS);
        Assert.True(aBS.IsFrozen);
        Assert.True(aBS.Value(aBS.FirstParameter()).IsEqual(new Point3D(1, 0, 0), Tol));
        Assert.True(aBS.Value(aBS.LastParameter()).IsEqual(new Point3D(3, 0, 0), Tol));

        // a B-spline needing a segment is copied; the input keeps its range
        BSplineCurve3D aC = CubicThrough(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0), new Point3D(3, 1, 0));
        aC.Freeze();
        BSplineCurve3D? aSegmented = aSCC.ConvertToBSpline(aC, 0.25, 0.5, 1e-7);
        Assert.NotSame(aC, aSegmented);
        Assert.Equal(0.25, aSegmented!.FirstParameter(), Tol);
        Assert.Equal(0.0, aC.FirstParameter());
        Assert.Same(aC, aSCC.ConvertToBSpline(aC, 0, 1, 1e-7));

        // a circle is approximated
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0);
        BSplineCurve2D? aBS2d = aSCC.ConvertToBSpline(aCircle, 0, 1, 1e-6);
        Assert.NotNull(aBS2d);
        Assert.True(aBS2d.Value(aBS2d.LastParameter()).IsEqual(new Point2D(Math.Cos(1), Math.Sin(1)), 1e-5));

        NCollection_Array1<double> aKnots = new(1, 4);
        aKnots[1] = 0;
        aKnots[2] = 1;
        aKnots[3] = 1;
        aKnots[4] = 2;
        Assert.True(global::Kerf.Engine.Healing.Construct.ConstructCurve.FixKnots(aKnots));
        Assert.True(aKnots[3] > aKnots[2]);
        Assert.False(global::Kerf.Engine.Healing.Construct.ConstructCurve.FixKnots(aKnots));
    }

    [Fact]
    public void ProjectCurveOnSurface_PerformFreezesInternalDoesNot()
    {
        PlaneSurface aPlane = new(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        aPlane.Freeze();
        TrimmedCurve3D aLine = Segment(new Point3D(0, 0, 0), new Point3D(3, 4, 0));
        aLine.Freeze();
        global::Kerf.Engine.Healing.Construct.ProjectCurveOnSurface aProj = new();
        aProj.Init(aPlane, 1e-7);
        Assert.True(aProj.Perform(aLine, aLine.FirstParameter(), aLine.LastParameter(), out Curve2D? aPC));
        Assert.True(aPC!.IsFrozen);
        Assert.True(aProj.InternalPerform(aLine, aLine.FirstParameter(), aLine.LastParameter(), out Curve2D? aPC2));
        Assert.True(aPC2!.Value(aLine.LastParameter()).IsEqual(new Point2D(3, 4), 1e-9));
        Assert.True(aProj.Status(ExtendStatus.DONE1));

        // a non-planar case through the points: a helix-like B-spline on a cylinder (frozen surface and curve)
        CylindricalSurface aCyl = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 2.0);
        aCyl.Freeze();
        NCollection_Array1<Point3D> aPnts = new(1, 7);
        for (int i = 1; i <= 7; i++)
        {
            double a = (i - 1) * 0.4;
            aPnts[i] = new Point3D(2 * Math.Cos(a), 2 * Math.Sin(a), 0.3 * (i - 1));
        }
        BSplineCurve3D aHelix = new PointsToBSpline(aPnts).Curve();
        aProj.Init(aCyl, 1e-4);
        Assert.True(aProj.Perform(aHelix, aHelix.FirstParameter(), aHelix.LastParameter(), out Curve2D? aPCH));
        Assert.True(aProj.Status(ExtendStatus.DONE2));
        Assert.True(aPCH!.IsFrozen);
        Assert.True(aProj.InternalPerform(aHelix, aHelix.FirstParameter(), aHelix.LastParameter(), out Curve2D? aPCH2));
        Assert.False(aPCH2!.IsFrozen); // the interpolated pcurve, unfrozen for ported code
        Point2D aUV = aPCH.Value(aHelix.FirstParameter());
        Assert.True(aCyl.Value(aUV.X(), aUV.Y()).IsEqual(aHelix.Value(aHelix.FirstParameter()), 1e-4));

        // no surface: FAIL1
        global::Kerf.Engine.Healing.Construct.ProjectCurveOnSurface anEmpty = new();
        Assert.False(anEmpty.Perform(aLine, 0, 1, out Curve2D? aNone));
        Assert.Null(aNone);
        Assert.True(anEmpty.Status(ExtendStatus.FAIL1));
    }

    [Fact]
    public void JoinPCurves_TwinOnPlaneKeepsTheEdge()
    {
        Shape aBox = new MakeBox(10, 20, 30).Shape();
        Face aF = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
        var anExp = new Explorer(aF, ShapeType.Edge);
        Edge anE = ShapeCasts.Edge(anExp.Current());
        NCollection_HSequence<Shape> anEdges = new();
        anEdges.Append(anE);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anE);
        Edge aResult = global::Kerf.Engine.Healing.Construct.ShapeConstruct.PCurvesJoined(anEdges, aF, anE, out bool isDone);
        Assert.True(isDone);
        aSnap.AssertUnchanged(anE);
        Assert.True(aResult.IsSame(anE)); // a plane: nothing written, the input itself
        // in place on the frozen edge, on a face where it must write: the frozen edge refuses
        Assert.ThrowsAny<LockedShapeException>(() => global::Kerf.Engine.Healing.Construct.ShapeConstruct.JoinPCurves(anEdges, MakeCylFace(), anE));
    }

    // a cylindrical face (unfrozen) with no edges, for a write that must reach the frozen edge
    private static Face MakeCylFace()
    {
        BrepBuilder aB = new();
        Face aF = new();
        aB.MakeFace(ref aF, new CylindricalSurface(new CoordinateSystem(), 10), 1e-7);
        return aF;
    }

    [Fact]
    public void JoinPCurves_TwinJoinsTwoEdgesOnACylinder()
    {
        // two edges along a circle of a cylinder of radius 10, with pcurves on it; the joined edge gets one pcurve
        CylindricalSurface aCylSurf = new(new CoordinateSystem(), 10);
        BrepBuilder aB = new();
        Face aF = new();
        aB.MakeFace(ref aF, aCylSurf, 1e-7);
        CircleCurve3D aCircle = new(new Frame3D(), 10);
        Edge E1 = new MakeEdge(aCircle, 0, 1).InternalEdge();
        Edge E2 = new MakeEdge(aCircle, 1, 2).InternalEdge();
        aB.UpdateEdge(E1, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, 1e-7);
        aB.UpdateEdge(E2, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, 1e-7);
        Edge anAll = new MakeEdge(aCircle, 0, 2).Edge();
        aF.TShape()!.FreezeGraph();
        E1.TShape()!.FreezeGraph();
        E2.TShape()!.FreezeGraph();

        NCollection_HSequence<Shape> anEdges = new();
        anEdges.Append(E1);
        anEdges.Append(E2);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anAll);
        Edge aJoined = global::Kerf.Engine.Healing.Construct.ShapeConstruct.PCurvesJoined(anEdges, aF, anAll, out bool isDone);
        Assert.False(isDone); // OCCT returns i <= edges->Length(): false once every edge was joined
        aSnap.AssertUnchanged(anAll);
        Assert.False(aJoined.IsSame(anAll));
        Assert.True(aJoined.TShape()!.IsFrozen);
        Curve2D? aPC = Tool.CurveOnSurface(aJoined, aF, out double f, out double l);
        Assert.NotNull(aPC);
        Assert.Equal(0.0, f, 1e-9);
        Assert.Equal(2.0, l, 1e-9);
        Assert.True(aPC.Value(2.0).IsEqual(new Point2D(2, 0), 1e-7));
        Assert.False(Tool.SameParameter(aJoined));
    }

    [Fact]
    public void MakeTriangulation_PlanarAndNonPlanarContours()
    {
        // a planar square: one face
        NCollection_Array1<Point3D> aSquare = new(1, 4);
        aSquare[1] = new Point3D(0, 0, 0);
        aSquare[2] = new Point3D(1, 0, 0);
        aSquare[3] = new Point3D(1, 1, 0);
        aSquare[4] = new Point3D(0, 1, 0);
        global::Kerf.Engine.Healing.Construct.MakeTriangulation aMT = new(aSquare);
        Assert.True(aMT.IsDone);
        Shape aFace = aMT.Shape();
        Assert.Equal(ShapeType.Face, aFace.ShapeType());
        Assert.True(aFace.TShape()!.IsFrozen);

        // a non-planar zigzag hexagon from a frozen wire: a shell of planar facets holding the wire's edges
        MakePolygon aPoly = new();
        aPoly.Add(new Point3D(0, 0, 0));
        aPoly.Add(new Point3D(2, 0, 1));
        aPoly.Add(new Point3D(3, 2, 0));
        aPoly.Add(new Point3D(2, 4, 1));
        aPoly.Add(new Point3D(0, 4, 0));
        aPoly.Add(new Point3D(-1, 2, 1));
        aPoly.Close();
        Wire aWire = aPoly.Wire();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aWire);
        global::Kerf.Engine.Healing.Construct.MakeTriangulation aMT2 = new(aWire);
        Assert.True(aMT2.IsDone);
        Shape aShell = aMT2.Shape();
        aSnap.AssertUnchanged(aWire);
        Assert.Equal(ShapeType.Shell, aShell.ShapeType());
        int aNbFaces = 0;
        for (Explorer anExp = new(aShell, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aNbFaces++;
        }
        Assert.True(aNbFaces >= 2);
        // every edge of the wire is in the result
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aShell, ShapeType.Edge, aMap);
        for (Explorer anExp = new(aWire, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.True(aMap.Contains(anExp.Current()));
        }
    }

    [Fact]
    public void Geom2dAPI_Interpolate_ThroughPointsAndPeriodic()
    {
        NCollection_HArray1<Point2D> aPnts = new(1, 5);
        aPnts.SetValue(1, new Point2D(0, 0));
        aPnts.SetValue(2, new Point2D(1, 2));
        aPnts.SetValue(3, new Point2D(3, 3));
        aPnts.SetValue(4, new Point2D(4, 1));
        aPnts.SetValue(5, new Point2D(6, 0));
        Interpolate2D anInt = new(aPnts, false, 1e-7);
        anInt.Load(new Vector2D(1, 0), new Vector2D(1, -1));
        anInt.Perform();
        Assert.True(anInt.IsDone);
        BSplineCurve2D aC = anInt.Curve();
        Assert.True(aC.IsFrozen);
        Assert.Equal(3, aC.Degree());
        for (int i = 1; i <= 5; i++)
        {
            Assert.True(aC.Value(aC.Knot(i)).IsEqual(aPnts.Value(i), 1e-9));
        }
        aC.D1(aC.FirstParameter(), out _, out Vector2D aT0);
        Assert.True(aT0.IsParallel(new Vector2D(1, 0), 1e-9));

        Interpolate2D aPer = new(aPnts, true, 1e-7);
        aPer.Perform();
        Assert.True(aPer.IsDone);
        BSplineCurve2D aCP = aPer.Curve();
        Assert.True(aCP.IsPeriodic);
        Assert.True(aCP.Value(aCP.FirstParameter()).IsEqual(aCP.Value(aCP.LastParameter()), 1e-9));

        // too close points
        NCollection_HArray1<Point2D> aBad = new(1, 2);
        aBad.SetValue(1, new Point2D(0, 0));
        aBad.SetValue(2, new Point2D(0, 1e-9));
        Assert.ThrowsAny<ConstructionErrorException>(() => new Interpolate2D(aBad, false, 1e-7));
    }

    [Fact]
    public void Geom2dAPI_PointsToBSpline_GraphAndParameters()
    {
        // the graph of y = x^2 sampled at x = 0, 0.5, ..., 3: X(t) = t
        NCollection_Array1<double> aY = new(1, 7);
        for (int i = 1; i <= 7; i++)
        {
            double x = (i - 1) * 0.5;
            aY[i] = x * x;
        }
        PointsToBSpline2D aGraph = new(aY, 0.0, 0.5);
        Assert.True(aGraph.IsDone);
        BSplineCurve2D aC = aGraph.Curve();
        Assert.Equal(0.0, aC.FirstParameter(), 1e-12);
        Assert.Equal(3.0, aC.LastParameter(), 1e-12);
        for (int i = 0; i <= 6; i++)
        {
            double t = i * 0.5;
            Assert.Equal(t, aC.Value(t).X(), 1e-9);
            Assert.Equal(t * t, aC.Value(t).Y(), 1e-5);
        }

        NCollection_Array1<Point2D> aPnts = new(1, 5);
        NCollection_Array1<double> aPar = new(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aPnts[i] = new Point2D(i, Math.Sin(i));
            aPar[i] = 10 + i;
        }
        PointsToBSpline2D anAppr = new(aPnts, aPar);
        Assert.True(anAppr.IsDone);
        BSplineCurve2D aC2 = anAppr.Curve();
        // OCCT's 2D form keeps the knots on [0, 1]
        Assert.Equal(0.0, aC2.FirstParameter(), 1e-12);
        Assert.Equal(1.0, aC2.LastParameter(), 1e-12);
        Assert.True(aC2.Value(0).IsEqual(aPnts[1], 1e-3));

        PointsToBSpline2D aVar = new(aPnts, 1.0, 0.0, 0.0);
        Assert.True(aVar.IsDone);
        Assert.Null((BSplineCurve2D?)new PointsToBSpline2D());
        Assert.Throws<NotDoneException>(() => new PointsToBSpline2D().Curve());
    }
}
