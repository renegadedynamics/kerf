// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_DistanceFunction.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Distance function for point-curve extrema: F(u) = (C(u) - P) . C'(u), with derivative
/// F'(u) = C'(u) . C'(u) + (C(u) - P) . C''(u). Its roots are the extrema of the squared distance.
/// </summary>
/// <remarks>A struct functor (decision 51) holding the caller's adaptor, as OCCT holds a pointer to it.</remarks>
public readonly struct ExtremaPC_DistanceFunction : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction, global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
{
    private readonly Adaptor3d_Curve myCurve; // Curve adaptor (not owned)
    private readonly Point3D myP;              // Query point

    public ExtremaPC_DistanceFunction(Adaptor3d_Curve theCurve, in Point3D theP)
    {
        myCurve = theCurve;
        myP = theP;
    }

    /// <summary>Computes F(u) = (C(u) - P) . C'(u).</summary>
    public bool Value(double theU, out double theF)
    {
        myCurve.D1(theU, out Point3D aPt, out Vector3D aD1);

        Vector3D aVec = new(myP, aPt);
        theF = aVec.Dot(aD1);
        return true;
    }

    /// <summary>Computes F(u) and F'(u).</summary>
    public bool Values(double theU, out double theF, out double theDF)
    {
        myCurve.D2(theU, out Point3D aPt, out Vector3D aD1, out Vector3D aD2);

        Vector3D aVec = new(myP, aPt);
        theF = aVec.Dot(aD1);
        theDF = aD1.Dot(aD1) + aVec.Dot(aD2);
        return true;
    }

    /// <summary>Computes the derivative F'(u).</summary>
    public bool Derivative(double theU, out double theDF)
    {
        return Values(theU, out _, out theDF);
    }

    /// <summary>Returns the first parameter of the curve.</summary>
    public double FirstParameter() => myCurve.FirstParameter();

    /// <summary>Returns the last parameter of the curve.</summary>
    public double LastParameter() => myCurve.LastParameter();

    /// <summary>Returns the query point.</summary>
    public Point3D Point() => myP;

    /// <summary>Returns the curve.</summary>
    public Adaptor3d_Curve Curve() => myCurve;
}
