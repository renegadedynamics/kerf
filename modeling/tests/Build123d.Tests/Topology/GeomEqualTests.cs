// Ported from build123d tests/test_direct_api/test_geom_equal.py.
// Omitted:
// - TestGeomEqualTypeMismatch.test_edge_vs_non_edge, test_wire_vs_non_wire: GeomEqual is typed (Edge with Edge, Wire
//   with Wire), so comparing an edge with a wire doesn't compile.
using Occt;
using Ops = Build123d.Operations;

namespace Build123d.Tests.Topology;

/// <summary>The direct-API equivalents of the sketch and curve objects the Python tests use.</summary>
internal static class GeomEqualObjects
{
    public static Edge CircleEdge(double radius) => new Face(Wire.MakeCircle(radius)).SingleEdge();

    public static Edge EllipseEdge(double xRadius, double yRadius) => new Face(Wire.MakeEllipse(xRadius, yRadius)).SingleEdge();

    public static Wire RectangleWire(double width, double height) => Face.MakeRect(width, height).SingleWire();

    public static Edge Spline(params Vector[] points) => Edge.MakeSpline(points);

    public static Edge Bezier(params Vector[] points) => Edge.MakeBezier(points);

    /// <summary>An edge on an OCCT B-spline built from its poles, knots and multiplicities (and weights).</summary>
    public static Edge BSpline(Vector[] poles, double[] knots, int[] mults, int degree, double[]? weights = null)
    {
        var poleArray = new NCollection_Array1<gp_Pnt>(1, poles.Length);
        for (int i = 0; i < poles.Length; i++)
        {
            var p = poles[i].ToGpPnt();
            poleArray.SetValue(i + 1, p);
        }
        var knotArray = new NCollection_Array1<double>(1, knots.Length);
        for (int i = 0; i < knots.Length; i++)
            knotArray.SetValue(i + 1, knots[i]);
        var multArray = new NCollection_Array1<int>(1, mults.Length);
        for (int i = 0; i < mults.Length; i++)
            multArray.SetValue(i + 1, mults[i]);
        Geom_BSplineCurve curve;
        if (weights is null)
            curve = new Geom_BSplineCurve(poleArray, knotArray, multArray, degree, false);
        else
        {
            var weightArray = new NCollection_Array1<double>(1, weights.Length);
            for (int i = 0; i < weights.Length; i++)
                weightArray.SetValue(i + 1, weights[i]);
            curve = new Geom_BSplineCurve(poleArray, weightArray, knotArray, multArray, degree, false);
        }
        var maker = new BRepBuilderAPI_MakeEdge(curve);
        return new Edge(maker.Edge());
    }
}

public class EdgeGeomEqualLineTests
{
    [Fact]
    public void SameLine()
    {
        var e1 = Edge.MakeLine((0, 0, 0), (1, 1, 1));
        var e2 = Edge.MakeLine((0, 0, 0), (1, 1, 1));
        Assert.Equal(GeomType.Line, e1.GeomType);
        Assert.True(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentLine()
    {
        var e1 = Edge.MakeLine((0, 0, 0), (1, 1, 1));
        var e2 = Edge.MakeLine((0, 0, 0), (1, 1, 2));
        Assert.False(e1.GeomEqual(e2));
    }
}

public class EdgeGeomEqualCircleTests
{
    [Fact]
    public void DifferentCircleFromRevolve()
    {
        // Two circles with same radius/endpoints but different center/axis
        var f1 = Ops.MakeFace(new RadiusArc((5, 0), (-5, 0), 15) + new Line((5, 0), (-5, 0)));
        var p1 = Ops.Revolve(f1, Axis.X, 90);
        var circles = p1.Edges().FilterBy(GeomType.Circle);
        Assert.Equal(2, circles.Count);
        var value1 = circles[0];
        var value2 = circles[1].Reversed();
        // These circles have same endpoints after reversal but different center/axis
        Assert.False(value1.GeomEqual(value2));
    }

    [Fact]
    public void SameCircle()
    {
        var e1 = GeomEqualObjects.CircleEdge(10);
        var e2 = GeomEqualObjects.CircleEdge(10);
        Assert.Equal(GeomType.Circle, e1.GeomType);
        Assert.True(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentRadius() =>
        Assert.False(GeomEqualObjects.CircleEdge(10).GeomEqual(GeomEqualObjects.CircleEdge(11)));

    [Fact]
    public void SameArc()
    {
        var e1 = Edge.MakeCircle(10, startAngle: 0, endAngle: 90);
        var e2 = Edge.MakeCircle(10, startAngle: 0, endAngle: 90);
        Assert.True(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentArcAngle()
    {
        var e1 = Edge.MakeCircle(10, startAngle: 0, endAngle: 90);
        var e2 = Edge.MakeCircle(10, startAngle: 0, endAngle: 180);
        Assert.False(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentLocation()
    {
        // Circles with same radius but different center location
        var e1 = Edge.MakeCircle(10);
        var e2 = Edge.MakeCircle(10).Located(new Location((5, 0, 0)));
        Assert.False(e1.GeomEqual(e2));
    }

    [Fact]
    public void SameLocation()
    {
        var e1 = Edge.MakeCircle(10).Located(new Location((5, 5, 0)));
        var e2 = Edge.MakeCircle(10).Located(new Location((5, 5, 0)));
        Assert.True(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentAxis()
    {
        var e1 = Edge.MakeCircle(10, plane: Plane.XY);
        var e2 = Edge.MakeCircle(10, plane: Plane.XZ);
        Assert.False(e1.GeomEqual(e2));
    }

    [Fact]
    public void SameAxis()
    {
        var e1 = Edge.MakeCircle(10, plane: Plane.YZ);
        var e2 = Edge.MakeCircle(10, plane: Plane.YZ);
        Assert.True(e1.GeomEqual(e2));
    }
}

public class EdgeGeomEqualEllipseTests
{
    [Fact]
    public void SameEllipse()
    {
        var e1 = GeomEqualObjects.EllipseEdge(10, 5);
        var e2 = GeomEqualObjects.EllipseEdge(10, 5);
        Assert.Equal(GeomType.Ellipse, e1.GeomType);
        Assert.True(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentMajorRadius() =>
        Assert.False(GeomEqualObjects.EllipseEdge(10, 5).GeomEqual(GeomEqualObjects.EllipseEdge(11, 5)));

    [Fact]
    public void DifferentMinorRadius() =>
        Assert.False(GeomEqualObjects.EllipseEdge(10, 5).GeomEqual(GeomEqualObjects.EllipseEdge(10, 6)));

    [Fact]
    public void DifferentLocation()
    {
        var e1 = Edge.MakeEllipse(10, 5);
        var e2 = Edge.MakeEllipse(10, 5).Located(new Location((5, 0, 0)));
        Assert.False(e1.GeomEqual(e2));
    }

    [Fact]
    public void SameLocation()
    {
        var e1 = Edge.MakeEllipse(10, 5).Located(new Location((5, 5, 0)));
        var e2 = Edge.MakeEllipse(10, 5).Located(new Location((5, 5, 0)));
        Assert.True(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentAxis()
    {
        var e1 = Edge.MakeEllipse(10, 5, plane: Plane.XY);
        var e2 = Edge.MakeEllipse(10, 5, plane: Plane.XZ);
        Assert.False(e1.GeomEqual(e2));
    }

    [Fact]
    public void SameAxis()
    {
        var e1 = Edge.MakeEllipse(10, 5, plane: Plane.YZ);
        var e2 = Edge.MakeEllipse(10, 5, plane: Plane.YZ);
        Assert.True(e1.GeomEqual(e2));
    }
}

public class EdgeGeomEqualHyperbolaTests
{
    private static Edge Hyperbola(double xRadius, double yRadius, Plane? plane = null) =>
        Edge.MakeHyperbola(xRadius, yRadius, plane: plane, startAngle: -45, endAngle: 45);

    [Fact]
    public void SameHyperbola()
    {
        var e1 = Hyperbola(10, 5);
        var e2 = Hyperbola(10, 5);
        Assert.Equal(GeomType.Hyperbola, e1.GeomType);
        Assert.True(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentXRadius() => Assert.False(Hyperbola(10, 5).GeomEqual(Hyperbola(11, 5)));

    [Fact]
    public void DifferentYRadius() => Assert.False(Hyperbola(10, 5).GeomEqual(Hyperbola(10, 6)));

    [Fact]
    public void DifferentLocation() =>
        Assert.False(Hyperbola(10, 5).GeomEqual(Hyperbola(10, 5).Located(new Location((5, 0, 0)))));

    [Fact]
    public void SameLocation() =>
        Assert.True(Hyperbola(10, 5).Located(new Location((5, 5, 0))).GeomEqual(Hyperbola(10, 5).Located(new Location((5, 5, 0)))));

    [Fact]
    public void DifferentAxis() => Assert.False(Hyperbola(10, 5, Plane.XY).GeomEqual(Hyperbola(10, 5, Plane.XZ)));

    [Fact]
    public void SameAxis() => Assert.True(Hyperbola(10, 5, Plane.YZ).GeomEqual(Hyperbola(10, 5, Plane.YZ)));
}

public class EdgeGeomEqualParabolaTests
{
    private static Edge Parabola(double focalLength, Plane? plane = null) =>
        Edge.MakeParabola(focalLength, plane: plane, startAngle: 0, endAngle: 60);

    [Fact]
    public void SameParabola()
    {
        var e1 = Parabola(5);
        var e2 = Parabola(5);
        Assert.Equal(GeomType.Parabola, e1.GeomType);
        Assert.True(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentFocalLength() => Assert.False(Parabola(5).GeomEqual(Parabola(6)));

    [Fact]
    public void DifferentLocation() =>
        Assert.False(Parabola(5).GeomEqual(Parabola(5).Located(new Location((5, 0, 0)))));

    [Fact]
    public void SameLocation() =>
        Assert.True(Parabola(5).Located(new Location((5, 5, 0))).GeomEqual(Parabola(5).Located(new Location((5, 5, 0)))));

    [Fact]
    public void DifferentAxis() => Assert.False(Parabola(5, Plane.XY).GeomEqual(Parabola(5, Plane.XZ)));

    [Fact]
    public void SameAxis() => Assert.True(Parabola(5, Plane.YZ).GeomEqual(Parabola(5, Plane.YZ)));
}

public class EdgeGeomEqualBezierTests
{
    [Fact]
    public void SameBezier()
    {
        Vector[] pts = [(0, 0), (1, 1), (2, 0)];
        var e1 = GeomEqualObjects.Bezier(pts);
        var e2 = GeomEqualObjects.Bezier(pts);
        Assert.Equal(GeomType.Bezier, e1.GeomType);
        Assert.True(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentBezier()
    {
        var e1 = GeomEqualObjects.Bezier((0, 0), (1, 1), (2, 0));
        var e2 = GeomEqualObjects.Bezier((0, 0), (1, 2), (2, 0));
        Assert.False(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentDegree()
    {
        // Quadratic (degree 2, 3 points)
        var e1 = GeomEqualObjects.Bezier((0, 0), (1, 1), (2, 0));
        // Cubic (degree 3, 4 points) - adjusted to have same endpoints
        var e2 = GeomEqualObjects.Bezier((0, 0), (0.5, 1), (1.5, 1), (2, 0));
        Assert.Equal(GeomType.Bezier, e1.GeomType);
        Assert.Equal(GeomType.Bezier, e2.GeomType);
        Assert.False(e1.GeomEqual(e2));
    }

    [Fact]
    public void RationalBezierDifferentWeights()
    {
        Vector[] pts = [(0, 0, 0), (1, 1, 0), (2, 0, 0)];
        var e1 = Edge.MakeBezier(pts, weights: [1.0, 2.0, 1.0]);
        var e2 = Edge.MakeBezier(pts, weights: [1.0, 3.0, 1.0]);
        Assert.Equal(GeomType.Bezier, e1.GeomType);
        Assert.False(e1.GeomEqual(e2));
    }
}

public class EdgeGeomEqualBSplineTests
{
    [Fact]
    public void SameSpline()
    {
        Vector[] v = [(-2, 0), (-1, 0), (0, 0), (1, 0), (2, 0)];
        var e1 = GeomEqualObjects.Spline(v);
        var e2 = GeomEqualObjects.Spline(v);
        Assert.Equal(GeomType.BSpline, e1.GeomType);
        Assert.True(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentSpline()
    {
        var e1 = GeomEqualObjects.Spline((-2, 0), (-1, 0), (0, 0), (1, 0), (2, 0));
        var e2 = GeomEqualObjects.Spline((-2, 0), (-1, 1), (0, 0), (1, 0), (2, 0));
        Assert.False(e1.GeomEqual(e2));
    }

    [Fact]
    public void ComplexSpline()
    {
        Vector[] v = [(-2, 0), (-1, 0), (0, 0), (1, 0), (2, 0), (3, 0.1), (4, 1), (5, 2.2), (6, 3), (7, 2), (8, -1)];
        Assert.True(GeomEqualObjects.Spline(v).GeomEqual(GeomEqualObjects.Spline(v)));
    }

    [Fact]
    public void DifferentPeriodicity()
    {
        // Same control points, different periodicity
        Vector[] pts = [(0, 0), (1, 1), (2, 0), (1, -1)];
        var e1 = Edge.MakeSpline(pts, periodic: false);
        var e2 = Edge.MakeSpline(pts, periodic: true);
        Assert.Equal(GeomType.BSpline, e1.GeomType);
        Assert.Equal(GeomType.BSpline, e2.GeomType);
        Assert.False(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentPoleCount()
    {
        var e1 = GeomEqualObjects.Spline((0, 0), (1, 1), (2, 0), (3, 1), (4, 0));
        var e2 = GeomEqualObjects.Spline((0, 0), (0.8, 0.8), (1.6, 0.2), (2.4, 0.8), (3.2, 0.2), (4, 0));
        Assert.Equal(GeomType.BSpline, e1.GeomType);
        Assert.Equal(GeomType.BSpline, e2.GeomType);
        Assert.False(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentKnotValues()
    {
        // 5 poles for degree 3 with one internal knot
        Vector[] poles = [(0, 0, 0), (1, 2, 0), (2, 2, 0), (3, 2, 0), (4, 0, 0)];
        int[] mults = [4, 1, 4];
        var e1 = GeomEqualObjects.BSpline(poles, [0.0, 0.5, 1.0], mults, 3);
        // Internal knot at 0.3 - different position changes shape
        var e2 = GeomEqualObjects.BSpline(poles, [0.0, 0.3, 1.0], mults, 3);

        Assert.Equal(GeomType.BSpline, e1.GeomType);
        Assert.True(e1.PositionAt(0.5) != e2.PositionAt(0.5));
        Assert.False(e1.GeomEqual(e2));
    }

    [Fact]
    public void DifferentMultiplicities()
    {
        // Same 7 poles and 4 knots for both curves
        Vector[] poles = [(0, 0, 0), (1, 2, 0), (2, 1, 0), (3, 2, 0), (4, 1, 0), (5, 2, 0), (6, 0, 0)];
        double[] knots = [0.0, 0.33, 0.67, 1.0];
        // Multiplicities [4, 1, 2, 4] - sum = 11 = 7 + 3 + 1
        var e1 = GeomEqualObjects.BSpline(poles, knots, [4, 1, 2, 4], 3);
        // Multiplicities [4, 2, 1, 4] - same sum, swapped internal mults
        var e2 = GeomEqualObjects.BSpline(poles, knots, [4, 2, 1, 4], 3);

        Assert.Equal(GeomType.BSpline, e1.GeomType);
        Assert.Equal(GeomType.BSpline, e2.GeomType);
        Assert.True(e1.PositionAt(0.5) != e2.PositionAt(0.5));
        Assert.False(e1.GeomEqual(e2));
    }

    [Fact]
    public void RationalBSplineDifferentWeights()
    {
        Vector[] poles = [(0, 0, 0), (1, 1, 0), (2, 1, 0), (3, 0, 0)];
        double[] knots = [0.0, 1.0];
        int[] mults = [4, 4];
        var e1 = GeomEqualObjects.BSpline(poles, knots, mults, 3, [1.0, 2.0, 2.0, 1.0]);
        var e2 = GeomEqualObjects.BSpline(poles, knots, mults, 3, [1.0, 3.0, 3.0, 1.0]);
        Assert.Equal(GeomType.BSpline, e1.GeomType);
        Assert.False(e1.GeomEqual(e2));
    }
}

public class EdgeGeomEqualOffsetTests
{
    /// <summary>An edge on an offset curve; Offset2D fits splines to those it makes.</summary>
    private static Edge OffsetEdge(double distance, Vector? direction = null)
    {
        var spline = GeomEqualObjects.Spline((0, 0), (1, 1), (2, 0), (3, 1));
        double first, last;
        var adaptor = spline.GeomAdaptor();
        (first, last) = (adaptor.FirstParameter(), adaptor.LastParameter());
        var topoEdge = TopoDS.Edge(spline.Wrapped);
        double f = 0, l = 0;
        var basis = BRep_Tool.Curve(topoEdge, out f, out l);
        var dir = (direction ?? new Vector(0, 0, 1)).ToGpDir();
        var curve = new Geom_OffsetCurve(basis!, distance, dir);
        var maker = new BRepBuilderAPI_MakeEdge(curve, first, last);
        var edge = new Edge(maker.Edge());
        Assert.Equal(GeomType.Offset, edge.GeomType);
        return edge;
    }

    [Fact]
    public void SameOffset() => Assert.True(OffsetEdge(0.1).GeomEqual(OffsetEdge(0.1)));

    [Fact]
    public void DifferentOffsetValue() => Assert.False(OffsetEdge(0.1).GeomEqual(OffsetEdge(0.2)));

    [Fact]
    public void DifferentOffsetDirection() => Assert.False(OffsetEdge(0.1).GeomEqual(OffsetEdge(0.1, (0, 0, -1))));
}

public class EdgeGeomEqualToleranceTests
{
    [Fact]
    public void CircleRadiusWithinTolerance() =>
        Assert.True(Edge.MakeCircle(10.0).GeomEqual(Edge.MakeCircle(10.0 + 1e-7)));   // within the default 1e-6

    [Fact]
    public void CircleRadiusOutsideTolerance() =>
        Assert.False(Edge.MakeCircle(10.0).GeomEqual(Edge.MakeCircle(10.0 + 1e-5)));

    [Fact]
    public void TolParameterAccepted()
    {
        var e1 = Edge.MakeLine((0, 0), (1, 1));
        var e2 = Edge.MakeLine((0, 0), (1, 1));
        Assert.True(e1.GeomEqual(e2, tolerance: 1e-9));
        Assert.True(e1.GeomEqual(e2, tolerance: 0.1));
    }

    [Fact]
    public void LineEndpointWithinTolerance() =>
        Assert.True(Edge.MakeLine((0, 0, 0), (1, 1, 1)).GeomEqual(Edge.MakeLine((0, 0, 0), (1 + 1e-7, 1, 1))));

    [Fact]
    public void LineEndpointOutsideTolerance() =>
        Assert.False(Edge.MakeLine((0, 0, 0), (1, 1, 1)).GeomEqual(Edge.MakeLine((0, 0, 0), (1.001, 1, 1))));
}

public class EdgeGeomEqualReversedTests
{
    [Fact]
    public void LineReversedNotEqual() =>
        Assert.False(Edge.MakeLine((0, 0), (1, 1)).GeomEqual(Edge.MakeLine((1, 1), (0, 0))));

    [Fact]
    public void ArcReversedNotEqual()
    {
        var e1 = Edge.MakeCircle(10, startAngle: 0, endAngle: 90);
        var e2 = Edge.MakeCircle(10, startAngle: 0, endAngle: 90).Reversed();
        // Reversed edge has swapped start/end points
        Assert.False(e1.GeomEqual(e2));
    }

    [Fact]
    public void SplineReversedNotEqual()
    {
        var e1 = GeomEqualObjects.Spline((0, 0), (1, 1), (2, 0), (3, 1));
        var e2 = e1.Reversed();
        Assert.False(e1.GeomEqual(e2));
    }
}

public class EdgeGeomEqualArcVariationsTests
{
    [Fact]
    public void FullCircleEqual() => Assert.True(Edge.MakeCircle(10).GeomEqual(Edge.MakeCircle(10)));

    [Fact]
    public void ArcDifferentStartSameSweep()
    {
        var e1 = Edge.MakeCircle(10, startAngle: 0, endAngle: 90);
        var e2 = Edge.MakeCircle(10, startAngle: 90, endAngle: 180);
        // Same radius and sweep angle, but different positions
        Assert.False(e1.GeomEqual(e2));
    }

    [Fact]
    public void ArcSameEndpointsDifferentDirection()
    {
        var e1 = Edge.MakeCircle(10, startAngle: 0, endAngle: 90, angularDirection: AngularDirection.CounterClockwise);
        var e2 = Edge.MakeCircle(10, startAngle: 90, endAngle: 0, angularDirection: AngularDirection.Clockwise);
        // These trace different paths (short arc vs long arc)
        Assert.False(e1.GeomEqual(e2));
    }
}

public class EdgeGeomEqualNumericalTests
{
    [Fact]
    public void VerySmallEdge() =>
        Assert.True(Edge.MakeLine((0, 0), (1e-6, 1e-6)).GeomEqual(Edge.MakeLine((0, 0), (1e-6, 1e-6))));

    [Fact]
    public void VeryLargeCoordinates() =>
        Assert.True(Edge.MakeLine((1e6, 1e6), (1e6 + 1, 1e6 + 1)).GeomEqual(Edge.MakeLine((1e6, 1e6), (1e6 + 1, 1e6 + 1))));

    [Fact]
    public void LargeCoordinatesSmallDifference() =>
        // Difference is above tolerance
        Assert.False(Edge.MakeLine((1e6, 1e6), (1e6 + 1, 1e6 + 1)).GeomEqual(Edge.MakeLine((1e6, 1e6), (1e6 + 1 + 1e-5, 1e6 + 1))));
}

public class EdgeGeomEqual3DPositioningTests
{
    [Fact]
    public void SameShapeDifferentZ() =>
        Assert.False(Edge.MakeCircle(10).GeomEqual(Edge.MakeCircle(10).Located(new Location((0, 0, 5)))));

    [Fact]
    public void LineIn3D() => Assert.True(Edge.MakeLine((0, 0, 0), (1, 2, 3)).GeomEqual(Edge.MakeLine((0, 0, 0), (1, 2, 3))));

    [Fact]
    public void LineIn3DDifferent() =>
        Assert.False(Edge.MakeLine((0, 0, 0), (1, 1, 0)).GeomEqual(Edge.MakeLine((0, 0, 0), (1, 1, 1))));
}

public class EdgeGeomEqualSplineVariationsTests
{
    [Fact]
    public void SplineControlPointWithinTolerance() =>
        Assert.True(GeomEqualObjects.Spline((0, 0), (1, 1), (2, 0)).GeomEqual(GeomEqualObjects.Spline((0, 0), (1 + 1e-7, 1), (2, 0))));

    [Fact]
    public void SplineControlPointOutsideTolerance() =>
        Assert.False(GeomEqualObjects.Spline((0, 0), (1, 1), (2, 0)).GeomEqual(GeomEqualObjects.Spline((0, 0), (1.001, 1), (2, 0))));

    [Fact]
    public void SplineDifferentPointCount() =>
        // Different number of poles
        Assert.False(GeomEqualObjects.Spline((0, 0), (1, 1), (2, 0)).GeomEqual(GeomEqualObjects.Spline((0, 0), (0.5, 0.5), (1, 1), (2, 0))));
}

public class EdgeGeomEqualUnknownTypeTests
{
    [Fact]
    public void InterpolationPointsUsed()
    {
        var e1 = Edge.MakeLine((0, 0), (1, 1));
        var e2 = Edge.MakeLine((0, 0), (1, 1));
        // Even with different numInterpolationPoints, these should be equal
        Assert.True(e1.GeomEqual(e2, numInterpolationPoints: 3));
        Assert.True(e1.GeomEqual(e2, numInterpolationPoints: 10));
    }
}

public class WireGeomEqualTests
{
    [Fact]
    public void SameRectangleWire() =>
        Assert.True(GeomEqualObjects.RectangleWire(10, 5).GeomEqual(GeomEqualObjects.RectangleWire(10, 5)));

    [Fact]
    public void DifferentRectangleWire() =>
        Assert.False(GeomEqualObjects.RectangleWire(10, 5).GeomEqual(GeomEqualObjects.RectangleWire(10, 6)));

    [Fact]
    public void SameSplineWire()
    {
        Vector[] v = [(0, 0), (1, 1), (2, 0), (3, 1)];
        var w1 = new Wire([GeomEqualObjects.Spline(v)]);
        var w2 = new Wire([GeomEqualObjects.Spline(v)]);
        Assert.True(w1.GeomEqual(w2));
    }

    [Fact]
    public void DifferentEdgeCount()
    {
        var w1 = GeomEqualObjects.RectangleWire(10, 5);
        var w2 = new Wire([Edge.MakeLine((0, 0), (10, 0))]);
        Assert.False(w1.GeomEqual(w2));
    }

    [Fact]
    public void IdenticalEdgeObjects()
    {
        // Two wires sharing the same edge objects
        var e1 = Edge.MakeLine((0, 0), (1, 0));
        var e2 = Edge.MakeLine((1, 0), (1, 1));
        var e3 = Edge.MakeLine((1, 1), (0, 0));
        var w1 = new Wire([e1, e2, e3]);
        var w2 = new Wire([e1, e2, e3]);
        Assert.True(w1.GeomEqual(w2));
    }

    private static Wire Triangle() =>
        new([Edge.MakeLine((0, 0), (1, 0)), Edge.MakeLine((1, 0), (1, 1)), Edge.MakeLine((1, 1), (0, 0))]);

    [Fact]
    public void GeometricallyEqualEdges() => Assert.True(Triangle().GeomEqual(Triangle()));

    [Fact]
    public void EdgesDifferentStartPoint()
    {
        // Two closed wires with same geometry but different starting vertex are not equal
        var w1 = Triangle();
        var w2 = new Wire([Edge.MakeLine((1, 1), (0, 0)), Edge.MakeLine((0, 0), (1, 0)), Edge.MakeLine((1, 0), (1, 1))]);
        Assert.False(w1.GeomEqual(w2));
    }

    [Fact]
    public void OneEdgeReversed()
    {
        var w1 = Triangle();
        // Middle edge is reversed (direction (1,1) -> (1,0) instead of (1,0) -> (1,1))
        var w2 = new Wire([Edge.MakeLine((0, 0), (1, 0)), Edge.MakeLine((1, 1), (1, 0)), Edge.MakeLine((1, 1), (0, 0))]);
        // OrderEdges should correct the orientation
        Assert.True(w1.GeomEqual(w2));
    }

    [Fact]
    public void ClosedWire()
    {
        static Wire Square() => new(
            [
                Edge.MakeLine((0, 0), (2, 0)),
                Edge.MakeLine((2, 0), (2, 2)),
                Edge.MakeLine((2, 2), (0, 2)),
                Edge.MakeLine((0, 2), (0, 0)),
            ]);
        var w1 = Square();
        var w2 = Square();
        Assert.True(w1.IsClosed);
        Assert.True(w2.IsClosed);
        Assert.True(w1.GeomEqual(w2));
    }

    [Fact]
    public void MixedEdgeTypes()
    {
        // Wire with line + arc + line
        static Wire Make() => new(
            [
                Edge.MakeLine((0, 0), (1, 0)),
                Edge.MakeCircle(0.5, startAngle: 0, endAngle: 180).Located(new Location((1.5, 0, 0))),
                Edge.MakeLine((2, 0), (3, 0)),
            ]);
        Assert.True(Make().GeomEqual(Make()));
    }

    [Fact]
    public void MixedEdgeTypesDifferent()
    {
        // Wire 1: line + arc
        var w1 = new Wire(
            [
                Edge.MakeLine((0, 0), (1, 0)),
                Edge.MakeCircle(0.5, startAngle: 0, endAngle: 180).Located(new Location((1.5, 0, 0))),
            ]);
        // Wire 2: line + different arc (different radius)
        var w2 = new Wire(
            [
                Edge.MakeLine((0, 0), (1, 0)),
                Edge.MakeCircle(0.6, startAngle: 0, endAngle: 180).Located(new Location((1.6, 0, 0))),
            ]);
        Assert.False(w1.GeomEqual(w2));
    }

    [Fact]
    public void AllEdgesReversedNotEqual()
    {
        // Wire 1: (0,0) -> (3,0)
        var w1 = new Wire([Edge.MakeLine((0, 0), (1, 0)), Edge.MakeLine((1, 0), (2, 1)), Edge.MakeLine((2, 1), (3, 0))]);
        // Wire 2: (3,0) -> (0,0) - same path but opposite direction
        var e1b = Edge.MakeLine((1, 0), (0, 0));
        var e2b = Edge.MakeLine((2, 1), (1, 0));
        var e3b = Edge.MakeLine((3, 0), (2, 1));
        var w2 = new Wire([e3b, e2b, e1b]);
        Assert.False(w1.GeomEqual(w2));
    }

    [Fact]
    public void OpenWireDifferentStart()
    {
        var w1 = new Wire([Edge.MakeLine((0, 0), (1, 0)), Edge.MakeLine((1, 0), (2, 1)), Edge.MakeLine((2, 1), (3, 0))]);
        // Different edges entirely (shifted)
        var w2 = new Wire([Edge.MakeLine((1, 0), (2, 0)), Edge.MakeLine((2, 0), (3, 1)), Edge.MakeLine((3, 1), (4, 0))]);
        Assert.False(w1.GeomEqual(w2));
    }

    [Fact]
    public void WireWithSplineEdges()
    {
        Vector[] pts1 = [(0, 0), (1, 1), (2, 0), (3, 1), (4, 0)];
        Vector[] pts2 = [(4, 0), (5, 1), (6, 0)];
        var w1 = new Wire([GeomEqualObjects.Spline(pts1), GeomEqualObjects.Spline(pts2)]);
        var w2 = new Wire([GeomEqualObjects.Spline(pts1), GeomEqualObjects.Spline(pts2)]);
        Assert.True(w1.GeomEqual(w2));
    }

    [Fact]
    public void SingleEdgeWire() =>
        Assert.True(new Wire([Edge.MakeLine((0, 0), (5, 5))]).GeomEqual(new Wire([Edge.MakeLine((0, 0), (5, 5))])));

    [Fact]
    public void SingleEdgeWireReversedNotEqual() =>
        // Opposite direction means not equal
        Assert.False(new Wire([Edge.MakeLine((0, 0), (5, 5))]).GeomEqual(new Wire([Edge.MakeLine((5, 5), (0, 0))])));
}

public class GeomEqualTypeMismatchTests
{
    [Fact]
    public void DifferentGeomTypes()
    {
        var line = Edge.MakeLine((0, 0, 0), (1, 1, 1));
        var circle = GeomEqualObjects.CircleEdge(10);
        Assert.False(line.GeomEqual(circle));
    }
}
