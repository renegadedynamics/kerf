// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_DistanceFunction.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Engine.Extrema.PointCurve;

/// <summary>
/// Distance function for point-curve extrema: F(u) = (C(u) - P) . C'(u), with derivative
/// F'(u) = C'(u) . C'(u) + (C(u) - P) . C''(u). Its roots are the extrema of the squared distance.
/// </summary>
/// <remarks>A struct functor (decision 51) holding the caller's adaptor, as OCCT holds a pointer to it.</remarks>
internal readonly struct DistanceFunction : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction, global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
{
    private readonly CurveAdaptorBase _curve; // Curve adaptor (not owned)
    private readonly Point3D _p;              // Query point

    public DistanceFunction(CurveAdaptorBase curve, in Point3D p)
    {
        _curve = curve;
        _p = p;
    }

    /// <summary>Computes F(u) = (C(u) - P) . C'(u).</summary>
    public bool Value(double u, out double f)
    {
        _curve.D1(u, out Point3D pt, out Vector3D d1);

        Vector3D vec = new(_p, pt);
        f = vec.Dot(d1);
        return true;
    }

    /// <summary>Computes F(u) and F'(u).</summary>
    public bool Values(double u, out double f, out double df)
    {
        _curve.D2(u, out Point3D pt, out Vector3D d1, out Vector3D d2);

        Vector3D vec = new(_p, pt);
        f = vec.Dot(d1);
        df = d1.Dot(d1) + vec.Dot(d2);
        return true;
    }

    /// <summary>Computes the derivative F'(u).</summary>
    public bool Derivative(double u, out double df)
    {
        return Values(u, out _, out df);
    }

    /// <summary>Returns the first parameter of the curve.</summary>
    public double FirstParameter() => _curve.FirstParameter();

    /// <summary>Returns the last parameter of the curve.</summary>
    public double LastParameter() => _curve.LastParameter();

    /// <summary>Returns the query point.</summary>
    public Point3D Point() => _p;

    /// <summary>Returns the curve.</summary>
    public CurveAdaptorBase Curve() => _curve;
}
