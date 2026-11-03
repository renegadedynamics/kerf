// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Data;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Topology.Orientation;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Known answers of BRepMesh_GeomTool, BRepMesh_DefaultRangeSplitter and BRepMesh_Deflection.</summary>
public class BRepMeshGeomToolTests
{
    private static readonly Xy O = new(0, 0);

    private static GeomTool.IntFlag SegSeg(Xy a1, Xy a2, Xy b1, Xy b2, bool theTouch, bool theOnSeg,
                                                    out Point2D thePnt)
    {
        thePnt = new Point2D(-99, -99);
        return GeomTool.IntSegSeg(a1, a2, b1, b2, theTouch, theOnSeg, ref thePnt);
    }

    [Fact]
    public void IntSegSeg_classifies_every_case()
    {
        Xy a = new(0, 0), b = new(2, 0);

        // a shared end point: touch, or nothing
        Assert.Equal(GeomTool.IntFlag.EndPointTouch, SegSeg(a, b, b, new Xy(3, 1), true, false, out _));
        Assert.Equal(GeomTool.IntFlag.NoIntersection, SegSeg(a, b, b, new Xy(3, 1), false, false, out _));

        // an end point of the second segment inside the first
        Assert.Equal(GeomTool.IntFlag.PointOnSegment,
                     SegSeg(a, b, new Xy(1, 0), new Xy(1, 1), false, true, out Point2D aP));
        // OCCT: the point given is the first segment's start (0, 0), not the second segment's start (1, 0) that lies
        // on it: IntSegSeg pairs the classification of each end point with the wrong point (occt-issues.md, 4A).
        // BRepMesh_Delaun doesn't use the point in this case.
        Assert.True(aP.IsEqual(new Point2D(0, 0), 0.0));
        Assert.Equal(GeomTool.IntFlag.PointOnSegment,
                     SegSeg(new Xy(1, 0), new Xy(1, 1), a, b, false, true, out aP));
        Assert.True(aP.IsEqual(new Point2D(0, 0), 0.0)); // the other way round: the second's start, not the first's (1, 0)
        Assert.Equal(GeomTool.IntFlag.NoIntersection,
                     SegSeg(a, b, new Xy(1, 0), new Xy(1, 1), false, false, out _));

        // overlapping collinear segments
        Assert.Equal(GeomTool.IntFlag.Glued, SegSeg(a, b, new Xy(1, 0), new Xy(3, 0), false, false, out _));

        // collinear, disjoint
        Assert.Equal(GeomTool.IntFlag.NoIntersection,
                     SegSeg(a, b, new Xy(3, 0), new Xy(4, 0), false, false, out _));

        // parallel, apart
        Assert.Equal(GeomTool.IntFlag.NoIntersection,
                     SegSeg(a, b, new Xy(0, 1), new Xy(2, 1), false, false, out _));

        // lines crossing outside the segments
        Assert.Equal(GeomTool.IntFlag.NoIntersection,
                     SegSeg(a, b, new Xy(3, -1), new Xy(3, 1), false, false, out _));

        // a proper cross
        Assert.Equal(GeomTool.IntFlag.Cross, SegSeg(a, b, new Xy(1, -1), new Xy(1, 1), false, false, out aP));
        Assert.True(aP.IsEqual(new Point2D(1, 0), 1e-15));
    }

    [Fact]
    public void IntLinLin_gives_the_parameters_on_both_lines()
    {
        Xy aPnt = new();
        Span<double> aParams = [0, 0];
        Assert.Equal(GeomTool.IntFlag.Cross,
                     GeomTool.IntLinLin(O, new Xy(4, 0), new Xy(1, -1), new Xy(1, 3), ref aPnt, aParams));
        Assert.Equal(0.25, aParams[0]);
        Assert.Equal(0.25, aParams[1]);
        Assert.True(aPnt.IsEqual(new Xy(1, 0), 1e-15));

        Assert.Equal(GeomTool.IntFlag.Same,
                     GeomTool.IntLinLin(O, new Xy(1, 1), new Xy(2, 2), new Xy(3, 3), ref aPnt, aParams));
        Assert.Equal(GeomTool.IntFlag.NoIntersection,
                     GeomTool.IntLinLin(O, new Xy(1, 1), new Xy(0, 1), new Xy(1, 2), ref aPnt, aParams));
    }

    [Fact]
    public void SquareDeflectionOfSegment_is_the_distance_from_the_chord()
    {
        Assert.Equal(4.0, GeomTool.SquareDeflectionOfSegment(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(5, 2, 0)), 1e-12);
        // coincident end points: the distance from the first point
        Assert.Equal(9.0, GeomTool.SquareDeflectionOfSegment(new Point3D(1, 1, 1), new Point3D(1, 1, 1), new Point3D(1, 1, 4)), 1e-12);
    }

    [Fact]
    public void Normal_is_reversed_on_a_reversed_face()
    {
        Face aFace = new MakeFace(new Plane3D(Constants.Origin(), Constants.DZ())).Face();
        Assert.True(GeomTool.Normal(new FaceSurfaceAdaptor(aFace), 1, 2, out Point3D aP, out Direction3D aN));
        Assert.True(aP.IsEqual(new Point3D(1, 2, 0), 0.0));
        Assert.True(aN.IsEqual(new Direction3D(0, 0, 1), 0.0));

        Face aReversed = ShapeCasts.Face(aFace.Reversed());
        Assert.True(GeomTool.Normal(new FaceSurfaceAdaptor(aReversed), 1, 2, out _, out aN));
        Assert.True(aN.IsEqual(new Direction3D(0, 0, -1), 0.0));
    }

    [Fact]
    public void Discretizes_an_edge_and_an_iso_curve_as_GCPnts_does()
    {
        CircleCurve3D aCircle = new(new Frame3D(Constants.Origin(), Constants.DZ()), 10);
        Edge anEdge = new MakeEdge(aCircle, 0.0, Math.PI).Edge();
        EdgeCurveAdaptor anAdaptor = new(anEdge);
        GeomTool aTool = new(anAdaptor, 0.0, Math.PI, 0.1, 0.5, 4, 0.01);
        var aRef = new TangentialDeflection(anAdaptor, 0.0, Math.PI, 0.5, 0.1, 4, global::Kerf.Engine.Foundation.Precision.PConfusion(), 0.01);
        Assert.Equal(aRef.PointCount, aTool.PointCount);

        // On the edge: the parameters, the points and the UV points of the pcurve on a face (here, a plane's).
        Face aFace = new MakeFace(new Plane3D(Constants.Origin(), Constants.DZ()), -20, 20, -20, 20).Face();
        FaceSurfaceAdaptor aSurf = new(aFace);
        Assert.False(aTool.Value(0, aSurf, out _, out _, out _));
        Assert.False(aTool.Value(aTool.PointCount + 1, aSurf, out _, out _, out _));
        for (int i = 1; i <= aTool.PointCount; i++)
        {
            Assert.True(aTool.Value(i, aSurf, out double aParam, out Point3D aPnt, out Point2D aUV));
            Assert.Equal(aRef.Parameter(i), aParam);
            Assert.True(aPnt.IsEqual(aRef.Value(i), 0.0));
            Assert.Equal(aPnt.X(), aUV.X(), 1e-12);
            Assert.Equal(aPnt.Y(), aUV.Y(), 1e-12);
        }

        // AddPoint inserts in parameter order and replaces within PConfusion.
        int aNb = aTool.PointCount;
        aTool.AddPoint(aCircle.Value(0.123), 0.123);
        Assert.Equal(aNb + 1, aTool.PointCount);
        aTool.AddPoint(aCircle.Value(0.123), 0.123);
        Assert.Equal(aNb + 1, aTool.PointCount);

        // On an iso curve: the UV point has the iso parameter as its other coordinate; no pcurve value.
        CylindricalSurface aCyl = new(new CoordinateSystem(Constants.XOY()), 5);
        Face aCylFace = new MakeFace(aCyl, 0, 2 * Math.PI, 0, 10, global::Kerf.Engine.Foundation.Precision.Confusion()).Face();
        FaceSurfaceAdaptor aCylSurf = new(aCylFace);
        GeomTool aV = new(aCylSurf, IsoV, 3.0, 0.0, 2 * Math.PI, 0.1, 0.5);
        Assert.True(aV.PointCount > 4);
        Assert.False(aV.Value(1, aCylSurf, out _, out _, out _));
        for (int i = 1; i <= aV.PointCount; i++)
        {
            Assert.True(aV.Value(i, 3.0, out double aParam, out Point3D aPnt, out Point2D aUV));
            Assert.Equal(aParam, aUV.X());
            Assert.Equal(3.0, aUV.Y());
            Assert.True(aPnt.IsEqual(aCylSurf.Value(aParam, 3.0), 1e-12));
        }

        GeomTool aU = new(aCylSurf, IsoU, 1.0, 0.0, 10.0, 0.1, 0.5);
        Assert.True(aU.Value(1, 1.0, out double aP1, out _, out Point2D aUV1));
        Assert.Equal(1.0, aUV1.X());
        Assert.Equal(aP1, aUV1.Y());
    }

    [Fact]
    public void CellsCount_follows_the_surface_type()
    {
        Face aFace = new MakeFace(new Plane3D(Constants.Origin(), Constants.DZ()), 0, 10, 0, 5).Face();
        FaceData aDFace = new(aFace);
        Assert.Equal((-1, -1), GeomTool.CellsCount(aDFace.GetSurface(), 10, 0.1, null));

        var aSplitter = new DefaultRangeSplitter();
        aSplitter.Reset(aDFace, new ToolParameters());
        aSplitter.AddPoint(new Point2D(0, 0));
        aSplitter.AddPoint(new Point2D(10, 5));
        aSplitter.AdjustRange();
        Assert.True(aSplitter.IsValid);

        // A plane: 2 ^ log10(vertices) in both directions, at least 2.
        int aExpected = Math.Max(2, (int)Math.Ceiling(Math.Pow(2, Math.Log10(1000))));
        Assert.Equal((aExpected, aExpected), GeomTool.CellsCount(aDFace.GetSurface(), 1000, 0.1, aSplitter));
        Assert.Equal((2, 2), GeomTool.CellsCount(aDFace.GetSurface(), 3, 0.1, aSplitter));
    }

    [Fact]
    public void DefaultRangeSplitter_ranges_scale_and_tolerances()
    {
        Face aFace = new MakeFace(new Plane3D(Constants.Origin(), Constants.DZ()), 0, 10, 0, 5).Face();
        FaceData aDFace = new(aFace);
        var aSplitter = new DefaultRangeSplitter();
        aSplitter.Reset(aDFace, new ToolParameters());
        Assert.Equal((1.0e100, -1.0e100), aSplitter.GetRangeU());
        Assert.Equal((1.0, 1.0), aSplitter.GetDelta());
        Assert.Null(aSplitter.GenerateSurfaceNodes(new ToolParameters()));

        aSplitter.AddPoint(new Point2D(2, 1));
        aSplitter.AddPoint(new Point2D(8, 4));
        aSplitter.AddPoint(new Point2D(5, 3));
        aSplitter.AdjustRange();
        Assert.True(aSplitter.IsValid);
        Assert.Equal((2.0, 8.0), aSplitter.GetRangeU());
        Assert.Equal((1.0, 4.0), aSplitter.GetRangeV());
        // On a plane the lengths are the parametric ones: delta 1.
        Assert.Equal(1.0, aSplitter.GetDelta().first, 1e-12);
        Assert.Equal(1.0, aSplitter.GetDelta().second, 1e-12);
        double aTol = Tool.Tolerance(aFace);
        Assert.Equal(Math.Max(Math.Min(1.0e-05, aDFace.GetSurface().UResolution(aTol) * 1.1), 1e-7 * 6), aSplitter.GetToleranceUV().first);
        Assert.Equal(Math.Max(Math.Min(1.0e-05, aDFace.GetSurface().VResolution(aTol) * 1.1), 1e-7 * 3), aSplitter.GetToleranceUV().second);

        Point2D aP = new(3.5, 2.25);
        Point2D aScaled = aSplitter.Scale(aP, true);
        Assert.True(aSplitter.Scale(aScaled, false).IsEqual(aP, 1e-14));
        Assert.True(aSplitter.Point(aP).IsEqual(new Point3D(3.5, 2.25, 0), 0.0));

        // A range collapsed to a line is invalid.
        var aFlat = new DefaultRangeSplitter();
        aFlat.Reset(aDFace, new ToolParameters());
        aFlat.AddPoint(new Point2D(1, 1));
        aFlat.AddPoint(new Point2D(1, 4));
        aFlat.AdjustRange();
        Assert.False(aFlat.IsValid);
    }

    [Fact]
    public void Deflection_IsConsistent_and_ComputeAbsoluteDeflection()
    {
        Assert.True(Deflection.IsConsistent(0.1, 0.1, false));
        Assert.True(Deflection.IsConsistent(0.01, 0.1, false));
        Assert.False(Deflection.IsConsistent(0.01, 0.1, true));
        Assert.False(Deflection.IsConsistent(0.111, 0.1, false));
        Assert.True(Deflection.IsConsistent(0.095, 0.1, true));

        Assert.Equal(0.25, Deflection.ComputeAbsoluteDeflection(new Shape(), 0.25, 1.0));

        // A 10 x 2 x 1 box: shape size 10 (the bounding box is enlarged by the tolerance), coefficient 0.5 to 2.
        Solid aBox = TestShapes.Box(10, 2, 1);
        Box aBnd = new();
        Bounds.Add(aBox, ref aBnd, false);
        double aSize = 0.0;
        ShapeTool.BoxMaxDimension(aBnd, ref aSize);
        Assert.Equal(aSize * 0.5 * 0.01, Deflection.ComputeAbsoluteDeflection(aBox, 0.01, 0.0), 1e-15);
        Assert.Equal(2 * aSize * 0.01, Deflection.ComputeAbsoluteDeflection(aBox, 0.01, 1000.0), 1e-15);
        Assert.Equal(0.5 * aSize * 0.01, Deflection.ComputeAbsoluteDeflection(aBox, 0.01, 0.1), 1e-15);
    }

    [Fact]
    public void Deflection_of_edges_wires_and_faces()
    {
        Solid aBox = TestShapes.Box(10, 2, 1);
        Face aFace = BRepGPropTestShapes.Faces(aBox)[0];
        FaceData aDFace = new(ShapeCasts.Face(aFace.Oriented(Forward)));
        WireModel aDWire = aDFace.AddWire(BrepTools.OuterWire(aFace));

        var aParams = new ToolParameters { Deflection = 0.05, Angle = 0.3, DeflectionInterior = 0.2 };
        double aSum = 0.0;
        int aNb = 0;
        for (var anExp = new Explorer(aFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            EdgeData aDEdge = new(ShapeCasts.Edge(anExp.Current()));
            Deflection.ComputeDeflection(aDEdge, 10.0, aParams);
            Assert.Equal(0.05, aDEdge.GetDeflection());
            Assert.Equal(0.3, aDEdge.GetAngularDeflection());
            aDWire.AddEdge(aDEdge, anExp.Current().Orientation());
            aSum += aDEdge.GetDeflection();
            aNb++;
        }

        Deflection.ComputeDeflection(aDWire, aParams);
        Assert.Equal(aSum / aNb, aDWire.GetDeflection());

        Deflection.ComputeDeflection(aDFace, aParams);
        Assert.Equal(0.2, aDFace.GetDeflection()); // the interior deflection is larger than the wire's

        aParams.DeflectionInterior = 0.01;
        Deflection.ComputeDeflection(aDFace, aParams);
        Assert.Equal(Math.Max(2 * ShapeTool.MaxFaceTolerance(aDFace.GetFace()), aDWire.GetDeflection()),
                     aDFace.GetDeflection());

        aParams.ForceFaceDeflection = true;
        Deflection.ComputeDeflection(aDFace, aParams);
        Assert.Equal(0.01, aDFace.GetDeflection());

        // Relative: the edge's deflection follows its size; an empty wire takes the parameters' deflection.
        aParams = new ToolParameters { Deflection = 0.1, Relative = true };
        EdgeData aLong = new(TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)));
        Deflection.ComputeDeflection(aLong, 10.0, aParams);
        Assert.Equal(Deflection.ComputeAbsoluteDeflection(aLong.GetEdge(), 0.1, 10.0), aLong.GetDeflection());
        WireData anEmpty = new(new Wire(), 0);
        Deflection.ComputeDeflection(anEmpty, aParams);
        Assert.Equal(0.1, anEmpty.GetDeflection());
    }

    [Fact]
    public void Edge_deflection_covers_the_distance_of_its_vertices_from_its_curve()
    {
        // An edge whose end vertex is 0.3 off its curve's end.
        Vertex aV1 = TestShapes.Vertex(new Point3D(0, 0, 0));
        Vertex aV2 = TestShapes.Vertex(new Point3D(10, 0.3, 0));
        Edge anEdge = TestShapes.Edge(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 10, aV1, aV2);
        EdgeData aDEdge = new(anEdge);
        Deflection.ComputeDeflection(aDEdge, 10.0, new ToolParameters { Deflection = 0.1 });
        Assert.Equal(0.3, aDEdge.GetDeflection(), 1e-15);
    }
}
