// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using System.Diagnostics.CodeAnalysis;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;

namespace Kerf.Topology;

/// <summary>
/// An edge's curve in world coordinates, over the edge's range: made once (<see cref="Edge.ToCurve"/>), queried many
/// times. OCCT's <c>BRepAdaptor_Curve</c>.
/// </summary>
/// <remarks>
/// An evaluator keeps caches of the curve's spans, so it is not safe to use from several threads at once: make one per
/// thread. Lengths and discretisation are the measurement area's.
/// </remarks>
public sealed class EdgeCurve
{
    private readonly EdgeCurveAdaptor _curve;

    internal EdgeCurve(Edge edge)
    {
        Edge = edge;
        _curve = new EdgeCurveAdaptor(edge);
    }

    /// <summary>The edge.</summary>
    public Edge Edge { get; }

    /// <summary>The edge's parameter range.</summary>
    public Interval Range => new(_curve.FirstParameter(), _curve.LastParameter());

    /// <summary>What kind of curve the edge lies on.</summary>
    public CurveKind Kind => _curve.GetType();

    /// <summary>True if the curve is closed.</summary>
    public bool IsClosed => _curve.IsClosed;

    /// <summary>True if the curve is periodic.</summary>
    public bool IsPeriodic => _curve.IsPeriodic;

    /// <summary>The period of a periodic curve.</summary>
    /// <exception cref="InvalidOperationException">The curve isn't periodic.</exception>
    public double Period => IsPeriodic ? _curve.Period() : throw new InvalidOperationException("The curve is not periodic.");

    /// <summary>The curve's continuity.</summary>
    public Continuity Continuity => _curve.Continuity();

    /// <summary>The point at parameter <paramref name="u"/>.</summary>
    public Point3D Evaluate(double u) => _curve.EvalD0(u);

    /// <summary>The point and the derivatives up to <paramref name="order"/> (0 to 3) at parameter <paramref name="u"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="order"/> is not 0 to 3.</exception>
    public CurveDerivatives Derivatives(double u, int order)
    {
        switch (order)
        {
            case 0:
                return new CurveDerivatives(0, _curve.EvalD0(u));
            case 1:
            {
                Curve3D.ResD1 r = _curve.EvalD1(u);
                return new CurveDerivatives(1, r.Point, r.D1);
            }
            case 2:
            {
                Curve3D.ResD2 r = _curve.EvalD2(u);
                return new CurveDerivatives(2, r.Point, r.D1, r.D2);
            }
            case 3:
            {
                Curve3D.ResD3 r = _curve.EvalD3(u);
                return new CurveDerivatives(3, r.Point, r.D1, r.D2, r.D3);
            }
            default:
                throw new ArgumentOutOfRangeException(nameof(order), order, "The order must be 0 to 3.");
        }
    }

    /// <summary>The first derivative at <paramref name="u"/>: the tangent, not normalised.</summary>
    public Vector3D Tangent(double u) => _curve.EvalD1(u).D1;

    /// <summary>The derivative of order <paramref name="n"/> (1 or more) at <paramref name="u"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="n"/> is less than 1.</exception>
    public Vector3D DerivativeN(double u, int n)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(n, 1);
        return _curve.EvalDN(u, n);
    }

    /// <summary>The length of the edge's curve over its range (OCCT's <c>GCPnts_AbscissaPoint::Length</c>).</summary>
    public double Length => CurveMeasures.Length(_curve);

    /// <summary>The length of the curve between two parameters: negative when <paramref name="u2"/> is before
    /// <paramref name="u1"/>.</summary>
    public double LengthBetween(double u1, double u2) => CurveMeasures.Length(_curve, u1, u2);

    /// <summary>
    /// The parameter <paramref name="length"/> along the curve from the parameter <paramref name="from"/> (backwards for a
    /// negative length; OCCT's <c>GCPnts_AbscissaPoint</c>).
    /// </summary>
    /// <exception cref="InvalidOperationException">The point couldn't be found.</exception>
    public double ParameterAtLength(double length, double from) => CurveMeasures.ParameterAtLength(_curve, length, from);

    /// <summary>
    /// Parameters along the curve, its ends included, such that the polygon through their points stays within
    /// <paramref name="deflection"/> of the curve: OCCT's <c>GCPnts_UniformDeflection</c> for a curve continuous to the
    /// second derivative, else its <c>GCPnts_QuasiUniformDeflection</c>. Empty if it couldn't be computed.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="deflection"/> isn't more than zero.</exception>
    public ImmutableArray<double> Discretize(double deflection) => CurveMeasures.Discretize(_curve, deflection);

    /// <summary>
    /// How smoothly this edge meets <paramref name="other"/> where this one's parameter <paramref name="u"/> and the
    /// other's <paramref name="otherU"/> meet, within the tolerances (OCCT's <c>BRepLProp::Continuity</c>): C0, G1, C1, G2 or C2.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="other"/> is null.</exception>
    public Continuity ContinuityWith(EdgeCurve other, double u, double otherU, double linearTolerance = Tolerances.Confusion,
                                     double angularTolerance = Tolerances.Angular)
    {
        ArgumentNullException.ThrowIfNull(other);
        try
        {
            return Engine.Topology.Properties.BRepLProp.Continuity(_curve, other._curve, u, otherU, linearTolerance, angularTolerance);
        }
        catch (FailureException failure)
        {
            throw Refusal.Argument(failure);
        }
    }

    /// <summary>The engine's adaptor, for the operations of other areas (the frames along a curve).</summary>
    internal EdgeCurveAdaptor Adaptor => _curve;

    /// <summary>The line the edge lies on, if it is one.</summary>
    public bool TryGetLine(out Line3D line)
    {
        bool found = Kind == CurveKind.Line;
        line = found ? _curve.Line() : default;
        return found;
    }

    /// <summary>The circle the edge lies on, if it is one.</summary>
    public bool TryGetCircle(out Circle3D circle)
    {
        bool found = Kind == CurveKind.Circle;
        circle = found ? _curve.Circle() : default;
        return found;
    }

    /// <summary>The ellipse the edge lies on, if it is one.</summary>
    public bool TryGetEllipse(out Ellipse3D ellipse)
    {
        bool found = Kind == CurveKind.Ellipse;
        ellipse = found ? _curve.Ellipse() : default;
        return found;
    }

    /// <summary>The hyperbola the edge lies on, if it is one.</summary>
    public bool TryGetHyperbola(out Hyperbola3D hyperbola)
    {
        bool found = Kind == CurveKind.Hyperbola;
        hyperbola = found ? _curve.Hyperbola() : default;
        return found;
    }

    /// <summary>The parabola the edge lies on, if it is one.</summary>
    public bool TryGetParabola(out Parabola3D parabola)
    {
        bool found = Kind == CurveKind.Parabola;
        parabola = found ? _curve.Parabola() : default;
        return found;
    }

    /// <summary>The B-spline the edge lies on, in world coordinates, if it is one.</summary>
    public bool TryGetBSpline([NotNullWhen(true)] out BSplineCurve3D? curve)
    {
        curve = Kind == CurveKind.BSplineCurve ? _curve.BSpline() : null;
        return curve is not null;
    }

    /// <summary>The Bézier curve the edge lies on, in world coordinates, if it is one.</summary>
    public bool TryGetBezier([NotNullWhen(true)] out BezierCurve3D? curve)
    {
        curve = Kind == CurveKind.BezierCurve ? _curve.Bezier() : null;
        return curve is not null;
    }

    /// <summary>The offset curve the edge lies on, in world coordinates, if it is one.</summary>
    public bool TryGetOffsetCurve([NotNullWhen(true)] out OffsetCurve3D? curve)
    {
        curve = Kind == CurveKind.OffsetCurve ? _curve.OffsetCurve() : null;
        return curve is not null;
    }
}

/// <summary>
/// A wire's curve in world coordinates, its edges joined end to end: made once (<see cref="Wire.ToCurve"/>), queried many
/// times. OCCT's <c>BRepAdaptor_CompCurve</c>, whose parameter runs over each edge's range in turn.
/// </summary>
/// <remarks>
/// An evaluator keeps caches, so it is not safe to use from several threads at once: make one per thread. Beyond its ends
/// it evaluates its first or last edge.
/// </remarks>
public sealed class WireCurve
{
    private readonly WireCurveAdaptor _curve;

    internal WireCurve(Wire wire)
    {
        Wire = wire;
        _curve = new WireCurveAdaptor(wire);
    }

    /// <summary>The wire.</summary>
    public Wire Wire { get; }

    /// <summary>The parameter range: the edges' ranges laid end to end.</summary>
    public Interval Range => new(_curve.FirstParameter(), _curve.LastParameter());

    /// <summary>Whether the wire is closed.</summary>
    public bool IsClosed => _curve.IsClosed;

    /// <summary>The continuity: that of its first edge (OCCT's).</summary>
    public Continuity Continuity => _curve.Continuity();

    /// <summary>The point at <paramref name="u"/>.</summary>
    public Point3D Evaluate(double u) => _curve.EvalD0(u);

    /// <summary>The point and derivatives up to <paramref name="order"/> (0 to 3) at <paramref name="u"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="order"/> is outside 0 to 3.</exception>
    public CurveDerivatives Derivatives(double u, int order)
    {
        switch (order)
        {
            case 0:
                return new CurveDerivatives(0, _curve.EvalD0(u));
            case 1:
            {
                Curve3D.ResD1 r = _curve.EvalD1(u);
                return new CurveDerivatives(1, r.Point, r.D1);
            }
            case 2:
            {
                Curve3D.ResD2 r = _curve.EvalD2(u);
                return new CurveDerivatives(2, r.Point, r.D1, r.D2);
            }
            case 3:
            {
                Curve3D.ResD3 r = _curve.EvalD3(u);
                return new CurveDerivatives(3, r.Point, r.D1, r.D2, r.D3);
            }
            default:
                throw new ArgumentOutOfRangeException(nameof(order), order, "The order must be 0 to 3.");
        }
    }

    /// <summary>The derivative of order <paramref name="n"/> (1 or more) at <paramref name="u"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="n"/> is less than 1.</exception>
    public Vector3D DerivativeN(double u, int n)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(n, 1);
        return _curve.EvalDN(u, n);
    }

    /// <summary>The length of the wire's curve (OCCT's <c>GCPnts_AbscissaPoint::Length</c>).</summary>
    public double Length => CurveMeasures.Length(_curve);

    /// <summary>The length of the curve between two parameters: negative when <paramref name="u2"/> is before
    /// <paramref name="u1"/>.</summary>
    public double LengthBetween(double u1, double u2) => CurveMeasures.Length(_curve, u1, u2);

    /// <summary>The parameter <paramref name="length"/> along the curve from the parameter <paramref name="from"/>.</summary>
    /// <exception cref="InvalidOperationException">The point couldn't be found.</exception>
    public double ParameterAtLength(double length, double from) => CurveMeasures.ParameterAtLength(_curve, length, from);

    /// <summary>
    /// The points of the curve where its distance to <paramref name="point"/> is extreme (OCCT's <c>Extrema_ExtPC</c>), the
    /// nearest first (of equally near ones, the first OCCT found): empty if there are none.
    /// </summary>
    public ImmutableArray<CurvePoint> Project(Point3D point)
    {
        var extrema = new Engine.Extrema.CurvePointExtrema(point, _curve);
        if (!extrema.IsDone || extrema.ExtCount == 0)
        {
            return [];
        }

        var points = new List<CurvePoint>(extrema.ExtCount);
        for (int i = 1; i <= extrema.ExtCount; i++)
        {
            var found = extrema.Point(i);
            points.Add(new CurvePoint(found.Parameter(), found.Value(), Math.Sqrt(extrema.SquareDistance(i))));
        }

        return [.. points.OrderBy(static p => p.Distance)];
    }

    /// <summary>Parameters along the curve within <paramref name="deflection"/>, as <see cref="EdgeCurve.Discretize"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="deflection"/> isn't more than zero.</exception>
    public ImmutableArray<double> Discretize(double deflection) => CurveMeasures.Discretize(_curve, deflection);

    /// <summary>The engine's adaptor, for the operations of other areas (the frames along a curve).</summary>
    internal WireCurveAdaptor Adaptor => _curve;
}

/// <summary>Lengths along curves and their discretisation: OCCT's GCPnts, shared by the curve evaluators.</summary>
internal static class CurveMeasures
{
    public static double Length(Engine.Geometry.Adaptors.CurveAdaptorBase curve) => Engine.Measurement.AbscissaPoint.Length(curve);

    public static double Length(Engine.Geometry.Adaptors.CurveAdaptorBase curve, double u1, double u2) =>
        Engine.Measurement.AbscissaPoint.Length(curve, u1, u2);

    public static double ParameterAtLength(Engine.Geometry.Adaptors.CurveAdaptorBase curve, double length, double from)
    {
        var abscissa = new Engine.Measurement.AbscissaPoint(curve, length, from);
        return abscissa.IsDone ? abscissa.Parameter() : throw new InvalidOperationException("The point at that length couldn't be found.");
    }

    public static ImmutableArray<double> Discretize(Engine.Geometry.Adaptors.CurveAdaptorBase curve, double deflection)
    {
        if (!(deflection > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(deflection), deflection, "The deflection must be more than zero.");
        }

        double first = curve.FirstParameter(), last = curve.LastParameter();
        // GCPnts_UniformDeflection gives the best results but needs C2 continuity
        if (curve.Continuity() is Continuity.C2 or Continuity.C3 or Continuity.CN)
        {
            var uniform = new Engine.Measurement.UniformDeflection();
            uniform.Initialize(curve, deflection, first, last);
            return uniform.IsDone && uniform.PointCount > 0 ? [.. Enumerable.Range(1, uniform.PointCount).Select(uniform.Parameter)] : [];
        }

        var quasi = new Engine.Measurement.QuasiUniformDeflection();
        quasi.Initialize(curve, deflection, first, last);
        return quasi.IsDone && quasi.PointCount > 0 ? [.. Enumerable.Range(1, quasi.PointCount).Select(quasi.Parameter)] : [];
    }
}

/// <summary>A surface's curvature at a point (<see cref="FaceSurface.TryGetCurvature"/>): its principal curvatures and the
/// normal they are measured towards.</summary>
/// <param name="Min">The smaller principal curvature: negative where the surface bends away from <paramref name="Normal"/>.</param>
/// <param name="Max">The larger principal curvature.</param>
/// <param name="Normal">The surface's normal there (the surface's, not the face's orientation).</param>
public readonly record struct SurfaceCurvature(double Min, double Max, Direction3D Normal)
{
    /// <summary>The mean curvature: the principal curvatures' average.</summary>
    public double Mean => (Min + Max) / 2;

    /// <summary>The Gaussian curvature: the principal curvatures' product.</summary>
    public double Gaussian => Min * Max;
}

/// <summary>
/// A face's surface in world coordinates, with the face's orientation: made once (<see cref="Face.ToSurface"/>), queried
/// many times. OCCT's <c>BRepAdaptor_Surface</c>.
/// </summary>
/// <remarks>
/// An evaluator keeps caches of the surface's patches, so it is not safe to use from several threads at once: make one
/// per thread. Curvature, classification and projection are the measurement and healing areas'.
/// </remarks>
public sealed class FaceSurface
{
    private readonly FaceSurfaceAdaptor _surface;

    internal FaceSurface(Face face)
    {
        Face = face;
        _surface = new FaceSurfaceAdaptor(face);
    }

    /// <summary>The face.</summary>
    public Face Face { get; }

    /// <summary>What kind of surface the face lies on.</summary>
    public SurfaceKind Kind => _surface.GetType();

    /// <summary>The face's parameter bounds, from its boundary.</summary>
    public (Interval U, Interval V) Bounds
        => (new Interval(_surface.FirstUParameter(), _surface.LastUParameter()), new Interval(_surface.FirstVParameter(), _surface.LastVParameter()));

    /// <summary>The point at (<paramref name="u"/>, <paramref name="v"/>).</summary>
    public Point3D Evaluate(double u, double v) => _surface.EvalD0(u, v);

    /// <summary>The point and the partial derivatives up to <paramref name="order"/> (0 to 3) at (<paramref name="u"/>,
    /// <paramref name="v"/>).</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="order"/> is not 0 to 3.</exception>
    public SurfaceDerivatives Derivatives(double u, double v, int order)
    {
        switch (order)
        {
            case 0:
                return new SurfaceDerivatives(0, _surface.EvalD0(u, v));
            case 1:
            {
                Surface.ResD1 r = _surface.EvalD1(u, v);
                return new SurfaceDerivatives(1, r.Point, r.D1U, r.D1V);
            }
            case 2:
            {
                Surface.ResD2 r = _surface.EvalD2(u, v);
                return new SurfaceDerivatives(2, r.Point, r.D1U, r.D1V, r.D2U, r.D2V, r.D2UV);
            }
            case 3:
            {
                Surface.ResD3 r = _surface.EvalD3(u, v);
                return new SurfaceDerivatives(3, r.Point, r.D1U, r.D1V, r.D2U, r.D2V, r.D2UV, r.D3U, r.D3V, r.D3UUV, r.D3UVV);
            }
            default:
                throw new ArgumentOutOfRangeException(nameof(order), order, "The order must be 0 to 3.");
        }
    }

    /// <summary>
    /// The face's outward normal at (<paramref name="u"/>, <paramref name="v"/>): the surface's, reversed if the face is.
    /// </summary>
    /// <exception cref="ArgumentException">The normal is undefined there (the surface is singular: a cone's apex, a
    /// sphere's pole).</exception>
    public Direction3D Normal(double u, double v)
    {
        Surface.ResD1 r = _surface.EvalD1(u, v);
        Vector3D normal = r.D1U.Crossed(r.D1V);
        if (normal.Magnitude <= Tolerances.Confusion * Tolerances.Confusion)
        {
            throw new ArgumentException($"The normal is undefined at ({u}, {v}): the surface is singular there.");
        }

        var direction = new Direction3D(normal);
        return Face.Orientation == Orientation.Reversed ? direction.Reversed() : direction;
    }

    /// <summary>The plane the face lies on, if it is one.</summary>
    public bool TryGetPlane(out Plane3D plane)
    {
        bool found = Kind == SurfaceKind.Plane;
        plane = found ? _surface.Plane() : default;
        return found;
    }

    /// <summary>The cylinder the face lies on, if it is one.</summary>
    public bool TryGetCylinder(out Cylinder3D cylinder)
    {
        bool found = Kind == SurfaceKind.Cylinder;
        cylinder = found ? _surface.Cylinder() : default;
        return found;
    }

    /// <summary>The cone the face lies on, if it is one.</summary>
    public bool TryGetCone(out Cone3D cone)
    {
        bool found = Kind == SurfaceKind.Cone;
        cone = found ? _surface.Cone() : default;
        return found;
    }

    /// <summary>The sphere the face lies on, if it is one.</summary>
    public bool TryGetSphere(out Sphere3D sphere)
    {
        bool found = Kind == SurfaceKind.Sphere;
        sphere = found ? _surface.Sphere() : default;
        return found;
    }

    /// <summary>The torus the face lies on, if it is one.</summary>
    public bool TryGetTorus(out Torus3D torus)
    {
        bool found = Kind == SurfaceKind.Torus;
        torus = found ? _surface.Torus() : default;
        return found;
    }

    /// <summary>The partial derivative <paramref name="nu"/> times in u and <paramref name="nv"/> times in v (at least one
    /// in all) at (<paramref name="u"/>, <paramref name="v"/>), ignoring the face's orientation.</summary>
    /// <exception cref="ArgumentOutOfRangeException">An order is negative, or both are zero.</exception>
    public Vector3D DerivativeN(double u, double v, int nu, int nv)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(nu);
        ArgumentOutOfRangeException.ThrowIfNegative(nv);
        if (nu + nv == 0)
        {
            throw new ArgumentOutOfRangeException(nameof(nu), "The orders can't both be zero.");
        }

        return _surface.EvalDN(u, v, nu, nv);
    }

    /// <summary>
    /// The surface's curvature at (<paramref name="u"/>, <paramref name="v"/>), if it is defined there (OCCT's
    /// <c>BRepLProp_SLProps</c>, to second order): false at a singular point, where the normal is undefined, within
    /// <paramref name="tolerance"/>.
    /// </summary>
    public bool TryGetCurvature(double u, double v, double tolerance, out SurfaceCurvature curvature)
    {
        var props = new Engine.Topology.Properties.SLProps(_surface, u, v, 2, tolerance);
        if (!props.IsCurvatureDefined())
        {
            curvature = default;
            return false;
        }

        curvature = new SurfaceCurvature(props.MinCurvature(), props.MaxCurvature(), props.Normal());
        return true;
    }
}
