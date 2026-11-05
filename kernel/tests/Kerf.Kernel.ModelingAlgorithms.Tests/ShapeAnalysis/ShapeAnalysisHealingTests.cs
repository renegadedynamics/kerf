// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of ShapeAnalysis's healing-side analysers (unit 6H1): ShapeContents, ShapeTolerance, Geom, WireVertex,
/// CheckSmallFace. Inputs are published (frozen) shapes, checked unchanged.
/// </summary>
public class ShapeAnalysisHealingTests
{
    private static Shape Box() => new MakeBox(10.0, 20.0, 30.0).Shape();

    private static Face PolygonFace(params Point3D[] thePoints)
    {
        var aPoly = new MakePolygon();
        foreach (Point3D aP in thePoints)
        {
            aPoly.Add(aP);
        }
        aPoly.Close();
        var aMF = new MakeFace(aPoly.Wire(), true);
        Assert.True(aMF.IsDone);
        return aMF.Face();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // ShapeContents

    [Fact]
    public void ShapeContents_Box()
    {
        Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aSC = new ShapeContentsAnalysis();
        aSC.Perform(aBox);
        Assert.Equal(1, aSC.SolidCount);
        Assert.Equal(1, aSC.ShellCount);
        Assert.Equal(6, aSC.FaceCount);
        Assert.Equal(6, aSC.WireCount);
        Assert.Equal(24, aSC.EdgeCount);
        Assert.Equal(48, aSC.VertexCount);
        Assert.Equal(1, aSC.SharedSolidCount);
        Assert.Equal(6, aSC.SharedFaceCount);
        Assert.Equal(12, aSC.SharedEdgeCount);
        Assert.Equal(8, aSC.SharedVertexCount);
        Assert.Equal(0, aSC.FreeFaceCount);
        Assert.Equal(0, aSC.FreeEdgeCount);
        Assert.Equal(0, aSC.FreeWireCount);
        Assert.Equal(0, aSC.SolidsWithVoidCount);
        Assert.Equal(0, aSC.WireWitnSeamCount);
        Assert.Equal(0, aSC.BSplibeSurfCount);
        Assert.Equal(0, aSC.C0SurfaceCount);
        Assert.Equal(0, aSC.NoPCurveCount); // the box's planar faces store their pcurves
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void ShapeContents_Cylinder_SeamAndModes()
    {
        Shape aCyl = new ApiMakeCylinder(2.0, 5.0).Shape();
        var aSC = new ShapeContentsAnalysis();
        aSC.ModifyTrimmed3dMode() = true;
        aSC.Perform(aCyl);
        Assert.Equal(3, aSC.FaceCount);
        // the explorer meets the lateral face's seam twice (FORWARD and REVERSED): OCCT counts a wire with several seams
        Assert.Equal(0, aSC.WireWitnSeamCount);
        Assert.Equal(1, aSC.WireWithSevSeamCount);
        Assert.Equal(3, aSC.SharedEdgeCount);
        Assert.Equal(2, aSC.SharedVertexCount);
        Assert.Equal(aSC.TrimmedCurve3dCount, aSC.Trimmed3dSec().Length());

        // a second Perform clears the counts and the sequences
        aSC.ModifyTrimmed3dMode() = false;
        aSC.ClearFlags();
        aSC.Perform(Box());
        Assert.Equal(6, aSC.FaceCount);
        Assert.Equal(0, aSC.Trimmed3dSec().Length());
    }

    [Fact]
    public void ShapeContents_CompoundWithFreeEdgeAndFace()
    {
        var aBB = new BrepBuilder();
        var aComp = new Compound();
        aBB.MakeCompound(ref aComp);
        Edge anEdge = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        Face aFace = PolygonFace(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0));
        aBB.Add(aComp, anEdge);
        aBB.Add(aComp, aFace);
        aBB.Add(aComp, Box());
        aComp.TShape()!.FreezeGraph();

        var aSC = new ShapeContentsAnalysis();
        aSC.Perform(aComp);
        Assert.Equal(7, aSC.FaceCount);
        Assert.Equal(1, aSC.FreeFaceCount);
        Assert.Equal(1, aSC.FreeEdgeCount);
        Assert.Equal(1, aSC.SharedFreeEdgeCount);
        Assert.Equal(1 + 3 + 24, aSC.EdgeCount);
    }

    [Fact]
    public void ShapeContents_IndirectAndBSpline()
    {
        // a face on an indirect plane (left-handed axes), and one on a B-spline surface
        var anAx = new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));
        anAx = anAx.YReversed();
        var aPlane = new PlaneSurface(anAx);
        Face anIndirect = new MakeFace(aPlane, 0, 1, 0, 1, 1e-7).Face();
        BSplineSurface aBs = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(new TrimmedSurface(new PlaneSurface(new Plane3D()), 0, 1, 0, 1));
        Face aBsFace = new MakeFace(aBs, 1e-7).Face();

        var aBB = new BrepBuilder();
        var aComp = new Compound();
        aBB.MakeCompound(ref aComp);
        aBB.Add(aComp, anIndirect);
        aBB.Add(aComp, aBsFace);

        var aSC = new ShapeContentsAnalysis();
        aSC.ModifyIndirectMode() = true;
        aSC.Perform(aComp);
        Assert.Equal(1, aSC.IndirectSurfCount);
        Assert.Equal(1, aSC.IndirectSec().Length());
        Assert.True(aSC.IndirectSec().Value(1).IsSame(anIndirect));
        Assert.Equal(1, aSC.BSplibeSurfCount);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // ShapeTolerance

    [Fact]
    public void ShapeTolerance_Box()
    {
        Shape aBox = Box();
        var aST = new ShapeToleranceAnalysis();
        double aTol = Tool.Tolerance(ShapeCasts.Vertex(new Explorer(aBox, ShapeType.Vertex).Current()));
        Assert.Equal(aTol, aST.Tolerance(aBox, 1));
        Assert.Equal(aTol, aST.Tolerance(aBox, 0));
        Assert.Equal(aTol, aST.Tolerance(aBox, -1));
        Assert.Equal(aTol, aST.Tolerance(aBox, 1, ShapeType.Edge));

        // all 6 faces, 24 edges, 48 vertices (explored with repetition) over 1e-9
        Assert.Equal(6 + 24 + 48, aST.OverTolerance(aBox, 1e-9).Length());
        // over 0 is the range [0, 0]: faces and edges fail "tol <= 0", vertices pass OCCT's "tol >= 0"
        Assert.Equal(48, aST.OverTolerance(aBox, 0.0).Length());
        Assert.Equal(6, aST.OverTolerance(aBox, 1e-9, ShapeType.Face).Length());
        Assert.Equal(0, aST.OverTolerance(aBox, 1e-3).Length());
    }

    [Fact]
    public void ShapeTolerance_CumulatedAndVertexRangeBug()
    {
        var aBB = new BrepBuilder();
        var aV1 = new Vertex();
        var aV2 = new Vertex();
        aBB.MakeVertex(ref aV1, new Point3D(0, 0, 0), 1e-3);
        aBB.MakeVertex(ref aV2, new Point3D(1, 0, 0), 1e-5);
        var aComp = new Compound();
        aBB.MakeCompound(ref aComp);
        aBB.Add(aComp, aV1);
        aBB.Add(aComp, aV2);

        var aST = new ShapeToleranceAnalysis();
        aST.InitTolerance();
        aST.AddTolerance(aComp);
        Assert.Equal(1e-3, aST.GlobalTolerance(1));
        Assert.Equal(1e-5, aST.GlobalTolerance(-1));
        Assert.Equal((1e-3 + 1e-5) / 2, aST.GlobalTolerance(0));
        aST.AddTolerance(Box()); // cumulates
        Assert.Equal(1e-7, aST.GlobalTolerance(-1));

        // OCCT's vertex test is "tol >= valmax" (occt-issues.md): a vertex within [1e-6, 1e-4] isn't found, one over
        // the upper bound is
        NCollection_HSequence<Shape> aRes = aST.InTolerance(aComp, 1e-6, 1e-4, ShapeType.Vertex);
        Assert.Equal(1, aRes.Length());
        Assert.True(aRes.Value(1).IsSame(aV1));
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Geom

    [Fact]
    public void Geom_NearestPlane()
    {
        var aPnts = new NCollection_Array1<Point3D>(1, 5);
        aPnts[1] = new Point3D(0, 0, 1);
        aPnts[2] = new Point3D(4, 0, 1.001);
        aPnts[3] = new Point3D(4, 3, 0.999);
        aPnts[4] = new Point3D(0, 3, 1);
        aPnts[5] = new Point3D(2, 1.5, 1);
        var aPln = new Plane3D();
        Assert.True(GeomAnalysis.NearestPlane(aPnts, ref aPln, out double aDMax));
        Assert.Equal(1.0, Math.Abs(aPln.Axis.Direction.Z), 1e-3);
        Assert.True(aDMax < 2e-3);

        // points spread in 3D: no plane, the plane unchanged
        var aLine = new NCollection_Array1<Point3D>(1, 5);
        aLine[1] = new Point3D(0, 0, 0);
        aLine[2] = new Point3D(1, 0, 0);
        aLine[3] = new Point3D(0, 1, 0);
        aLine[4] = new Point3D(0, 0, 1);
        aLine[5] = new Point3D(1, 1, 1);
        var aKept = new Plane3D(new Point3D(5, 5, 5), new Direction3D(1, 0, 0));
        Assert.False(GeomAnalysis.NearestPlane(aLine, ref aKept, out _));
        Assert.Equal(5.0, aKept.Location.X);
    }

    [Fact]
    public void Geom_PositionTrsf()
    {
        Assert.True(GeomAnalysis.PositionTrsf(null, out Transform3D anId, 1.0, 1e-7));
        Assert.Equal(TransformForm.Identity, anId.Form);

        // a rotation of 90 degrees about Z (columns (0,1,0), (-1,0,0), (0,0,1)), translated by (1, 2, 3), unit 10.
        // OCCT's SetTransformation(axes) maps into the axes, the inverse rotation: (1,0,0) goes to (0,-1,0)
        NCollection_HArray2<double> aCoefs = Matrix(new double[,] { { 0, -1, 0, 1 }, { 1, 0, 0, 2 }, { 0, 0, 1, 3 } });
        Assert.True(GeomAnalysis.PositionTrsf(aCoefs, out Transform3D aT, 10.0, 1e-7));
        Point3D aP = new Point3D(1, 0, 0).Transformed(aT);
        Assert.Equal(10.0, aP.X, 1e-12);
        Assert.Equal(19.0, aP.Y, 1e-12);
        Assert.Equal(30.0, aP.Z, 1e-12);

        // scaled by 2: OCCT's SetScale replaces the rotation (occt-issues.md), so (1,0,0) only scales
        aCoefs = Matrix(new double[,] { { 0, -2, 0, 1 }, { 2, 0, 0, 2 }, { 0, 0, 2, 3 } });
        Assert.True(GeomAnalysis.PositionTrsf(aCoefs, out aT, 10.0, 1e-7));
        aP = new Point3D(1, 0, 0).Transformed(aT);
        Assert.Equal(12.0, aP.X, 1e-12);
        Assert.Equal(20.0, aP.Y, 1e-12);
        Assert.Equal(30.0, aP.Z, 1e-12);

        // not a similarity
        aCoefs[1, 1] = 3.0;
        aCoefs[2, 1] = 0.0;
        aCoefs[1, 2] = 0.0;
        Assert.False(GeomAnalysis.PositionTrsf(aCoefs, out _, 1.0, 1e-7));
    }

    private static NCollection_HArray2<double> Matrix(double[,] theM)
    {
        var aCoefs = new NCollection_HArray2<double>(1, 3, 1, 4);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aCoefs[i, j] = theM[i - 1, j - 1];
            }
        }
        return aCoefs;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // WireVertex

    [Fact]
    public void WireVertex_ClosedPolygon_SameVertices()
    {
        var aPoly = new MakePolygon(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0), true);
        Wire aWire = aPoly.Wire();
        var aWV = new WireVertexAnalysis();
        aWV.Init(aWire, 1e-7);
        Assert.False(aWV.IsDone);
        aWV.Analyze();
        Assert.True(aWV.IsDone);
        Assert.Equal(3, aWV.EdgeCount);
        // OCCT compares the previous edge's last vertex (REVERSED) with the next one's first (FORWARD) with ==, which
        // compares orientations too: the shared vertices give status 1 ("same coords"), never 0 (occt-issues.md)
        for (int i = 1; i <= 3; i++)
        {
            Assert.Equal(1, aWV.Status(i));
        }
        Assert.Equal(1, aWV.NextStatus(1));
        Assert.Equal(0, aWV.NextStatus(0));
        Assert.Equal(2, aWV.NextCriter(3, 1));
        Assert.Equal(0, aWV.NextCriter(-1));
    }

    [Fact]
    public void WireVertex_SeparateVertices_SameCoordsCloseAndEnd()
    {
        // three edges with their own vertices: (0,0)-(1,0); (1,0)-(1,1) starting at the same point; (1,1+1e-5)-(0,0):
        // its start is 1e-5 from the previous end
        Edge aE1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        Edge aE2 = new MakeEdge(new Point3D(1, 0, 0), new Point3D(1, 1, 0)).Edge();
        Edge aE3 = new MakeEdge(new Point3D(1, 1 + 1e-5, 0), new Point3D(0, 0, 0)).Edge();
        var aWD = new ExtendWireData();
        aWD.Add(aE1);
        aWD.Add(aE2);
        aWD.Add(aE3);

        var aWV = new WireVertexAnalysis();
        aWV.SetPrecision(1e-4);
        aWV.Load(aWD); // Load keeps the precision; Init ignores its argument
        aWV.Analyze();
        Assert.Equal(1e-4, aWV.Precision());
        Assert.Equal(1, aWV.Status(1)); // same coords at the vertices' tolerances
        Assert.Equal(2, aWV.Status(2)); // close at the precision
        Assert.Equal(1, aWV.Status(3));
        Assert.Equal(1, aWV.NextCriter(3));
        Assert.Equal(2, aWV.NextCriter(2, 1));

        // with a fine precision, the gap is left to projection: E3's start projects on (the line of) E2, at parameter
        // 1 + 1e-5, and OCCT records it as "start" (4), the parameter on the preceding edge stored as UFollowing
        aWV.SetPrecision(1e-7);
        Assert.False(aWV.IsDone);
        aWV.Analyze();
        int aStat = aWV.Data(2, out Xyz aPos, out double aUPre, out double aUFol);
        Assert.Equal(4, aStat);
        Assert.Equal(0.0, aUPre);
        Assert.Equal(1 + 1e-5, aUFol, 1e-12);
        Assert.Equal(1 + 1e-5, aPos.Y(), 1e-12);

        // the setters
        aWV.SetInters(2, new Xyz(1, 1, 0), 0.5, 0.25);
        Assert.Equal(5, aWV.Data(2, out aPos, out aUPre, out aUFol));
        Assert.Equal(new Xyz(1, 1, 0), aPos);
        Assert.Equal(0.5, aWV.UPrevious(2));
        Assert.Equal(0.25, aWV.UFollowing(2));
        aWV.SetStart(1, new Xyz(2, 0, 0), 0.75);
        Assert.Equal(4, aWV.Status(1));
        Assert.Equal(0.75, aWV.UFollowing(1)); // OCCT stores the preceding edge's parameter as UFollowing
        Assert.Equal(1, aWV.NextCriter(4));
        aWV.SetDisjoined(3);
        Assert.Equal(3, aWV.NextStatus(-1));
    }

    // ---------------------------------------------------------------------------------------------------------------
    // CheckSmallFace

    [Fact]
    public void CheckSmallFace_Spot()
    {
        var aCSF = new CheckSmallFaceAnalysis();
        Assert.Equal(-1.0, aCSF.Tolerance());

        Face aBig = PolygonFace(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0));
        Point3D aSpot = new(7, 7, 7);
        double aSpotTol = 42.0;
        Assert.Equal(0, aCSF.IsSpotFace(aBig, ref aSpot, ref aSpotTol));
        Assert.Equal(7.0, aSpot.X);
        Assert.False(aCSF.CheckSpotFace(aBig));

        // a triangle 1e-4 wide is a spot at tolerance 1e-3
        Face aSmall = PolygonFace(new Point3D(0, 0, 0), new Point3D(1e-4, 0, 0), new Point3D(1e-4, 1e-4, 0));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aSmall);
        Assert.Equal(1, aCSF.IsSpotFace(aSmall, ref aSpot, ref aSpotTol, 1e-3));
        Assert.Equal(0.5e-4, aSpot.X, 1e-15);
        Assert.Equal(0.5e-4, aSpotTol, 1e-15);
        Assert.True(aCSF.CheckSpotFace(aSmall, 1e-3));
        Assert.True(aCSF.StatusSpot(ExtendStatus.DONE1));
        aSnap.AssertUnchanged(aSmall);

        // a face without wires: OCCT answers 1 and leaves the spot alone
        var aBB = new BrepBuilder();
        var anEmpty = new Face();
        aBB.MakeFace(ref anEmpty, new PlaneSurface(new Plane3D()), 1e-7);
        aSpot = new Point3D(7, 7, 7);
        Assert.Equal(1, aCSF.IsSpotFace(anEmpty, ref aSpot, ref aSpotTol));
        Assert.Equal(7.0, aSpot.X);
    }

    [Fact]
    public void CheckSmallFace_Strip()
    {
        // a rectangle 10 long and 1e-5 wide: its short sides are null-length at 1e-4, its long sides confused
        Face aStrip = PolygonFace(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 1e-5, 0), new Point3D(0, 1e-5, 0));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aStrip);
        var aCSF = new CheckSmallFaceAnalysis();
        var aE1 = new Edge();
        var aE2 = new Edge();
        Assert.False(aCSF.CheckSingleStrip(aStrip, ref aE1, ref aE2, 1e-4)); // four vertices
        Assert.True(aCSF.CheckStripFace(aStrip, ref aE1, ref aE2, 1e-4));
        Assert.True(aCSF.StatusStrip(ExtendStatus.DONE3));
        Assert.False(aE1.IsNull);
        Assert.False(aE2.IsNull);
        Assert.True(aCSF.CheckStripEdges(aE1, aE2, 1e-4, out double aDMax));
        Assert.Equal(1e-5, aDMax, 1e-12);
        Assert.False(aCSF.CheckStripEdges(aE1, aE2, 1e-6, out _));
        aSnap.AssertUnchanged(aStrip);

        Face aSquare = PolygonFace(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0), new Point3D(0, 1, 0));
        var aCSF2 = new CheckSmallFaceAnalysis();
        Assert.False(aCSF2.CheckStripFace(aSquare, ref aE1, ref aE2, 1e-4));
        Assert.False(aCSF2.IsStripSupport(aSquare)); // a plane
    }

    [Fact]
    public void CheckSmallFace_StripSupport()
    {
        // a B-spline patch 10 x 1e-9: small in V
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[2, 1] = new Point3D(10, 0, 0);
        aPoles[1, 2] = new Point3D(0, 1e-9, 0);
        aPoles[2, 2] = new Point3D(10, 1e-9, 0);
        var aBz = new BezierSurface(aPoles);
        Face aFace = new MakeFace(aBz, 1e-7).Face();
        var aCSF = new CheckSmallFaceAnalysis();
        Assert.True(aCSF.IsStripSupport(aFace));
        Assert.True(aCSF.StatusStrip(ExtendStatus.DONE2));
    }

    [Fact]
    public void CheckSmallFace_SplittingVertices()
    {
        // a pentagon whose fourth vertex touches the first edge (1e-8 above it)
        Face aFace = PolygonFace(new Point3D(0, 0, 0), new Point3D(2, 0, 0), new Point3D(2, 1, 0), new Point3D(1, 1e-8, 0),
                                        new Point3D(0, 1, 0));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        var aCSF = new CheckSmallFaceAnalysis();
        var aMapEdges = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var aMapParam = new Dictionary<Shape, NCollection_List<double>>(ShapeHasher.Instance);

        // the public form: a null compound gives a new frozen compound
        // (the vertices are explored with repetition: the splitting vertex is met, counted and added twice, as in OCCT)
        var aComp = new Compound();
        Assert.Equal(2, aCSF.CheckSplittingVertices(aFace, aMapEdges, aMapParam, ref aComp));
        Assert.True(aCSF.StatusSplitVert(ExtendStatus.Done));
        Assert.True(aComp.TShape()!.IsFrozen);
        Assert.Equal(2, aComp.ChildCount);
        Assert.Single(aMapEdges);
        Assert.Single(aMapParam);

        // a frozen compound passed in is kept: the result holds its items and the new ones
        Compound aFirst = aComp;
        Assert.Equal(2, aCSF.CheckSplittingVertices(aFace, aMapEdges, aMapParam, ref aComp));
        Assert.Equal(2, aFirst.ChildCount);
        Assert.Equal(4, aComp.ChildCount);
        aSnap.AssertUnchanged(aFace);

        // the internal form adds to the caller's compound in place, and refuses a frozen one
        var aBB = new BrepBuilder();
        var anOwn = new Compound();
        aBB.MakeCompound(ref anOwn);
        Assert.Equal(2, aCSF.InternalCheckSplittingVertices(aFace, aMapEdges, aMapParam, anOwn));
        Assert.Equal(2, anOwn.ChildCount);
        Assert.ThrowsAny<Exception>(() => aCSF.InternalCheckSplittingVertices(aFace, aMapEdges, aMapParam, aFirst));

        Face aSquare = PolygonFace(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0), new Point3D(0, 1, 0));
        var aNone = new Compound();
        Assert.Equal(0, new CheckSmallFaceAnalysis().CheckSplittingVertices(aSquare, aMapEdges, aMapParam, ref aNone));
    }

    [Fact]
    public void CheckSmallFace_PinAndTwisted()
    {
        var aCSF = new CheckSmallFaceAnalysis();
        Face aPlanar = PolygonFace(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0));
        int aRow = -5, aSense = -5;
        Assert.False(aCSF.CheckPin(aPlanar, ref aRow, ref aSense)); // elementary surface: nothing written
        Assert.Equal(-5, aRow);
        double aU = -5, aV = -5;
        Assert.False(aCSF.CheckTwisted(aPlanar, ref aU, ref aV));

        // a Bezier triangle: its first V row of poles collapses to a point, a sharp pin
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[2, 1] = new Point3D(0, 0, 0);
        aPoles[1, 2] = new Point3D(0, 1, 0);
        aPoles[2, 2] = new Point3D(1, 1, 0);
        Face aPinned = new MakeFace(new BezierSurface(aPoles), 1e-7).Face();
        Assert.True(aCSF.CheckPin(aPinned, ref aRow, ref aSense));
        Assert.Equal(2, aSense); // the last bound checked with a non-zero status: V row nbv
        Assert.Equal(2, aRow);
        Assert.True(aCSF.StatusPin(ExtendStatus.DONE2)); // the last bound (V row 2, 1 long) is "sharp"

        // a twisted Bezier patch: the corners turn the normal over
        var aTw = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aTw[1, 1] = new Point3D(0, 0, 0);
        aTw[2, 1] = new Point3D(1, 0, 0);
        aTw[1, 2] = new Point3D(1, 1, 0);
        aTw[2, 2] = new Point3D(0, 1, 0);
        Face aTwisted = new MakeFace(new BezierSurface(aTw), 1e-7).Face();
        Assert.True(aCSF.CheckTwisted(aTwisted, ref aU, ref aV));
        Assert.True(aCSF.StatusTwisted(ExtendStatus.Done));
    }

    [Fact]
    public void CheckSmallFace_PinFace()
    {
        var aCSF = new CheckSmallFaceAnalysis();
        var aMap = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        Face aSquare = PolygonFace(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0), new Point3D(0, 1, 0));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aSquare);
        Assert.False(aCSF.CheckPinFace(aSquare, aMap));
        Assert.Empty(aMap);
        Assert.False(aCSF.StatusPinFace(ExtendStatus.Done));
        aSnap.AssertUnchanged(aSquare);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // CanonicalRecognition on frozen, located shapes

    [Fact]
    public void CanonicalRecognition_LocatedLineEdge_CurveNotTransformedInPlace()
    {
        // an edge on a Geom_Line, moved: the converter hands back the edge's own (frozen) line, which OCCT transforms
        // in place; the port transforms a copy, so repeated calls agree and the edge is unchanged
        var aLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Edge anEdge = new MakeEdge(aLine, 0.0, 2.0).Edge();
        var aT = new Transform3D();
        aT = aT.WithTranslation(new Vector3D(0, 5, 0));
        Edge aMoved = ShapeCasts.Edge(anEdge.Moved(new Placement(aT)));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aMoved);

        var aCR = new CanonicalRecognitionAnalysis(aMoved);
        for (int i = 0; i < 3; i++)
        {
            var aLin = new Line3D();
            Assert.True(aCR.IsLine(1e-7, ref aLin));
            Assert.Equal(5.0, aLin.Location.Y, 1e-12);
            Assert.Equal(0.0, aCR.GetGap());
        }
        aSnap.AssertUnchanged(aMoved);
        Assert.Equal(0.0, aLine.Line.Location.Y);

        // a status other than 0 blocks the operations until ClearStatus
        var aBad = new CanonicalRecognitionAnalysis(Box());
        Assert.Equal(1, aBad.GetStatus());
        var aLin2 = new Line3D();
        Assert.False(aBad.IsLine(1e-7, ref aLin2));
        Assert.Equal(-1, new CanonicalRecognitionAnalysis().GetStatus());
    }
}
