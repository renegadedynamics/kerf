// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
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
    public bool TryGetCylinder(out Cylinder cylinder)
    {
        bool found = Kind == SurfaceKind.Cylinder;
        cylinder = found ? _surface.Cylinder() : default;
        return found;
    }

    /// <summary>The cone the face lies on, if it is one.</summary>
    public bool TryGetCone(out Cone cone)
    {
        bool found = Kind == SurfaceKind.Cone;
        cone = found ? _surface.Cone() : default;
        return found;
    }

    /// <summary>The sphere the face lies on, if it is one.</summary>
    public bool TryGetSphere(out Sphere sphere)
    {
        bool found = Kind == SurfaceKind.Sphere;
        sphere = found ? _surface.Sphere() : default;
        return found;
    }

    /// <summary>The torus the face lies on, if it is one.</summary>
    public bool TryGetTorus(out Torus torus)
    {
        bool found = Kind == SurfaceKind.Torus;
        torus = found ? _surface.Torus() : default;
        return found;
    }
}
