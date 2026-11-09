// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Unit 10L1: the public evaluation of curves and surfaces (docs/api/geometry.md), in place of OCCT's D0 to DN, Value
// and the out-parameter bounds. The engine keeps those, internal.

namespace Kerf.Geometry;

public abstract partial class Curve3D
{
    /// <summary>The parameter range, <see cref="FirstParameter"/> to <see cref="LastParameter"/>; infinite ends for an
    /// unbounded curve (a line).</summary>
    public Interval Range => new(FirstParameter, LastParameter);

    /// <summary>What kind of curve this is; a trimmed curve is its basis curve's kind.</summary>
    public CurveKind Kind => this switch
    {
        LineCurve3D => CurveKind.Line,
        CircleCurve3D => CurveKind.Circle,
        EllipseCurve3D => CurveKind.Ellipse,
        HyperbolaCurve3D => CurveKind.Hyperbola,
        ParabolaCurve3D => CurveKind.Parabola,
        BezierCurve3D => CurveKind.BezierCurve,
        BSplineCurve3D => CurveKind.BSplineCurve,
        OffsetCurve3D => CurveKind.OffsetCurve,
        TrimmedCurve3D trimmed => trimmed.BasisCurve.Kind,
        _ => CurveKind.OtherCurve,
    };

    /// <summary>The point at parameter <paramref name="u"/>.</summary>
    public Point3D Evaluate(double u) => EvalD0(u);

    /// <summary>The length of the curve between two parameters: never negative, whichever of <paramref name="u1"/> and
    /// <paramref name="u2"/> comes first (OCCT's <c>GCPnts_AbscissaPoint::Length</c>).</summary>
    public double LengthBetween(double u1, double u2) => Topology.CurveMeasures.Length(new Engine.Geometry.Adaptors.CurveAdaptor(this), u1, u2);

    /// <summary>
    /// The parameter <paramref name="length"/> along the curve from the parameter <paramref name="from"/> (backwards for a
    /// negative length; OCCT's <c>GCPnts_AbscissaPoint</c>).
    /// </summary>
    /// <exception cref="InvalidOperationException">The point couldn't be found.</exception>
    public double ParameterAtLength(double length, double from) => Topology.CurveMeasures.ParameterAtLength(new Engine.Geometry.Adaptors.CurveAdaptor(this), length, from);

    /// <summary>The point and the derivatives up to <paramref name="order"/> (0 to 3) at parameter <paramref name="u"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="order"/> is not 0 to 3.</exception>
    public CurveDerivatives Derivatives(double u, int order)
    {
        switch (order)
        {
            case 0:
                return new CurveDerivatives(0, EvalD0(u));
            case 1:
            {
                ResD1 r = EvalD1(u);
                return new CurveDerivatives(1, r.Point, r.D1);
            }
            case 2:
            {
                ResD2 r = EvalD2(u);
                return new CurveDerivatives(2, r.Point, r.D1, r.D2);
            }
            case 3:
            {
                ResD3 r = EvalD3(u);
                return new CurveDerivatives(3, r.Point, r.D1, r.D2, r.D3);
            }
            default:
                throw new KernelArgumentOutOfRangeException(nameof(order), order, "The order must be 0 to 3; use DerivativeN for higher orders.");
        }
    }

    /// <summary>The <paramref name="n"/>th derivative at parameter <paramref name="u"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="n"/> is less than 1.</exception>
    public Vector3D DerivativeN(double u, int n)
    {
        KernelArgumentOutOfRangeException.ThrowIfLessThan(n, 1);
        return EvalDN(u, n);
    }
}

public abstract partial class Curve2D
{
    /// <summary>The parameter range, <see cref="FirstParameter"/> to <see cref="LastParameter"/>; infinite ends for an
    /// unbounded curve (a line).</summary>
    public Interval Range => new(FirstParameter, LastParameter);

    /// <summary>What kind of curve this is; a trimmed curve is its basis curve's kind.</summary>
    public CurveKind Kind => this switch
    {
        LineCurve2D => CurveKind.Line,
        CircleCurve2D => CurveKind.Circle,
        EllipseCurve2D => CurveKind.Ellipse,
        HyperbolaCurve2D => CurveKind.Hyperbola,
        ParabolaCurve2D => CurveKind.Parabola,
        BezierCurve2D => CurveKind.BezierCurve,
        BSplineCurve2D => CurveKind.BSplineCurve,
        OffsetCurve2D => CurveKind.OffsetCurve,
        TrimmedCurve2D trimmed => trimmed.BasisCurve.Kind,
        _ => CurveKind.OtherCurve,
    };

    /// <summary>The point at parameter <paramref name="u"/>.</summary>
    public Point2D Evaluate(double u) => EvalD0(u);

    /// <summary>The point and the derivatives up to <paramref name="order"/> (0 to 3) at parameter <paramref name="u"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="order"/> is not 0 to 3.</exception>
    public CurveDerivatives2D Derivatives(double u, int order)
    {
        switch (order)
        {
            case 0:
                return new CurveDerivatives2D(0, EvalD0(u));
            case 1:
            {
                ResD1 r = EvalD1(u);
                return new CurveDerivatives2D(1, r.Point, r.D1);
            }
            case 2:
            {
                ResD2 r = EvalD2(u);
                return new CurveDerivatives2D(2, r.Point, r.D1, r.D2);
            }
            case 3:
            {
                ResD3 r = EvalD3(u);
                return new CurveDerivatives2D(3, r.Point, r.D1, r.D2, r.D3);
            }
            default:
                throw new KernelArgumentOutOfRangeException(nameof(order), order, "The order must be 0 to 3; use DerivativeN for higher orders.");
        }
    }

    /// <summary>The <paramref name="n"/>th derivative at parameter <paramref name="u"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="n"/> is less than 1.</exception>
    public Vector2D DerivativeN(double u, int n)
    {
        KernelArgumentOutOfRangeException.ThrowIfLessThan(n, 1);
        return EvalDN(u, n);
    }
}

public abstract partial class Surface
{
    /// <summary>The parameter bounds in U and in V; infinite ends where the surface is unbounded (a plane).</summary>
    public (Interval U, Interval V) Bounds
    {
        get
        {
            ParameterBounds(out double u1, out double u2, out double v1, out double v2);
            return (new Interval(u1, u2), new Interval(v1, v2));
        }
    }

    /// <summary>What kind of surface this is; a trimmed surface is its basis surface's kind.</summary>
    public SurfaceKind Kind => this switch
    {
        PlaneSurface => SurfaceKind.Plane,
        CylindricalSurface => SurfaceKind.Cylinder,
        ConicalSurface => SurfaceKind.Cone,
        SphericalSurface => SurfaceKind.Sphere,
        ToroidalSurface => SurfaceKind.Torus,
        BezierSurface => SurfaceKind.BezierSurface,
        BSplineSurface => SurfaceKind.BSplineSurface,
        RevolutionSurface => SurfaceKind.SurfaceOfRevolution,
        ExtrusionSurface => SurfaceKind.SurfaceOfExtrusion,
        OffsetSurface => SurfaceKind.OffsetSurface,
        TrimmedSurface trimmed => trimmed.BasisSurface.Kind,
        _ => SurfaceKind.OtherSurface,
    };

    /// <summary>The point at parameters (<paramref name="u"/>, <paramref name="v"/>).</summary>
    public Point3D Evaluate(double u, double v) => EvalD0(u, v);

    /// <summary>The point and the partial derivatives up to <paramref name="order"/> (0 to 3) at (<paramref name="u"/>,
    /// <paramref name="v"/>).</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="order"/> is not 0 to 3.</exception>
    public SurfaceDerivatives Derivatives(double u, double v, int order)
    {
        switch (order)
        {
            case 0:
                return new SurfaceDerivatives(0, EvalD0(u, v));
            case 1:
            {
                ResD1 r = EvalD1(u, v);
                return new SurfaceDerivatives(1, r.Point, r.D1U, r.D1V);
            }
            case 2:
            {
                ResD2 r = EvalD2(u, v);
                return new SurfaceDerivatives(2, r.Point, r.D1U, r.D1V, r.D2U, r.D2V, r.D2UV);
            }
            case 3:
            {
                ResD3 r = EvalD3(u, v);
                return new SurfaceDerivatives(3, r.Point, r.D1U, r.D1V, r.D2U, r.D2V, r.D2UV, r.D3U, r.D3V, r.D3UUV, r.D3UVV);
            }
            default:
                throw new KernelArgumentOutOfRangeException(nameof(order), order, "The order must be 0 to 3; use DerivativeN for higher orders.");
        }
    }

    /// <summary>The derivative of order <paramref name="uOrder"/> in U and <paramref name="vOrder"/> in V at (<paramref name="u"/>,
    /// <paramref name="v"/>).</summary>
    /// <exception cref="ArgumentOutOfRangeException">An order is negative, or both are zero.</exception>
    public Vector3D DerivativeN(double u, double v, int uOrder, int vOrder)
    {
        KernelArgumentOutOfRangeException.ThrowIfNegative(uOrder);
        KernelArgumentOutOfRangeException.ThrowIfNegative(vOrder);
        if (uOrder + vOrder < 1)
        {
            throw new KernelArgumentOutOfRangeException(nameof(uOrder), "The derivative's order must be 1 or more.");
        }

        return EvalDN(u, v, uOrder, vOrder);
    }
}
