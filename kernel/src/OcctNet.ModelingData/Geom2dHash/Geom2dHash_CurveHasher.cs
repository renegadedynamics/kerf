// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_CurveHasher.hxx
// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_CurveHasher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Polymorphic hasher of Geom2d_Curve, used for geometry deduplication.
/// </summary>
/// <remarks>
/// OCCT's functor is an <see cref="IEqualityComparer{T}"/>: <c>aHasher(x)</c> is <see cref="GetHashCode(Geom2d_Curve)"/>
/// and <c>aHasher(a, b)</c> is <see cref="Equals(Geom2d_Curve, Geom2d_Curve)"/>. As in OCCT, numeric fields are compared within
/// <see cref="CompTolerance"/> and rounded to multiples of <see cref="HashTolerance"/> before hashing (so objects
/// equal within the tolerance can still hash differently across a rounding boundary); Bezier and B-spline objects
/// hash their degrees and counts only. The hash values themselves are not OCCT's.
/// </remarks>
public sealed class Geom2dHash_CurveHasher : IEqualityComparer<Geom2d_Curve?>
{
    /// <summary>The tolerance of the comparison of numeric fields.</summary>
    public readonly double CompTolerance;

    /// <summary>The step to which numeric fields are rounded before hashing.</summary>
    public readonly double HashTolerance;

    /// <summary>A hasher with the given tolerances (OCCT's defaults: Precision::Angular() and Precision::Confusion()).</summary>
    public Geom2dHash_CurveHasher(double theCompTolerance = 1.0e-12, double theHashTolerance = 1.0e-7)
    {
        CompTolerance = theCompTolerance;
        HashTolerance = theHashTolerance;
    }

    /// <summary>Hashes any Geom2d_Curve by dispatching to the hasher of its type; 0 for null.</summary>
    public int GetHashCode(Geom2d_Curve? theCurve) => GeomHash_HashUtils.ToInt32(Hash(theCurve));

    // OCCT's size_t operator()(const handle&)
    internal ulong Hash(Geom2d_Curve? theCurve)
    {
        if (theCurve is null)
        {
            return 0;
        }

        // Dispatch based on actual type
        Type aType = theCurve.DynamicType();
        if (aType == typeof(Geom2d_Line))
        {
            return new Geom2dHash_LineHasher(CompTolerance, HashTolerance).Hash((Geom2d_Line)theCurve);
        }
        else if (aType == typeof(Geom2d_Circle))
        {
            return new Geom2dHash_CircleHasher(CompTolerance, HashTolerance).Hash((Geom2d_Circle)theCurve);
        }
        else if (aType == typeof(Geom2d_Ellipse))
        {
            return new Geom2dHash_EllipseHasher(CompTolerance, HashTolerance).Hash((Geom2d_Ellipse)theCurve);
        }
        else if (aType == typeof(Geom2d_Hyperbola))
        {
            return new Geom2dHash_HyperbolaHasher(CompTolerance, HashTolerance).Hash((Geom2d_Hyperbola)theCurve);
        }
        else if (aType == typeof(Geom2d_Parabola))
        {
            return new Geom2dHash_ParabolaHasher(CompTolerance, HashTolerance).Hash((Geom2d_Parabola)theCurve);
        }
        else if (aType == typeof(Geom2d_BezierCurve))
        {
            return new Geom2dHash_BezierCurveHasher(CompTolerance, HashTolerance).Hash((Geom2d_BezierCurve)theCurve);
        }
        else if (aType == typeof(Geom2d_BSplineCurve))
        {
            return new Geom2dHash_BSplineCurveHasher(CompTolerance, HashTolerance).Hash((Geom2d_BSplineCurve)theCurve);
        }
        else if (aType == typeof(Geom2d_TrimmedCurve))
        {
            return new Geom2dHash_TrimmedCurveHasher(CompTolerance, HashTolerance).Hash((Geom2d_TrimmedCurve)theCurve);
        }
        else if (aType == typeof(Geom2d_OffsetCurve))
        {
            return new Geom2dHash_OffsetCurveHasher(CompTolerance, HashTolerance).Hash((Geom2d_OffsetCurve)theCurve);
        }

        // Unknown type - hash the type name
        return GeomHash_HashUtils.Hash(aType.Name);
    }

    /// <summary>Compares two objects of the same type with the hasher of that type; null equals only null.</summary>
    public bool Equals(Geom2d_Curve? theCurve1, Geom2d_Curve? theCurve2)
    {
        if (theCurve1 is null || theCurve2 is null)
        {
            return theCurve1 is null && theCurve2 is null;
        }

        if (ReferenceEquals(theCurve1, theCurve2))
        {
            return true;
        }

        // Must be same type
        if (theCurve1.DynamicType() != theCurve2.DynamicType())
        {
            return false;
        }

        // Dispatch based on actual type (already confirmed types match)
        Type aType = theCurve1.DynamicType();
        if (aType == typeof(Geom2d_Line))
        {
            return new Geom2dHash_LineHasher(CompTolerance, HashTolerance).IsEqual((Geom2d_Line)theCurve1, (Geom2d_Line)theCurve2);
        }
        else if (aType == typeof(Geom2d_Circle))
        {
            return new Geom2dHash_CircleHasher(CompTolerance, HashTolerance).IsEqual((Geom2d_Circle)theCurve1, (Geom2d_Circle)theCurve2);
        }
        else if (aType == typeof(Geom2d_Ellipse))
        {
            return new Geom2dHash_EllipseHasher(CompTolerance, HashTolerance).IsEqual((Geom2d_Ellipse)theCurve1, (Geom2d_Ellipse)theCurve2);
        }
        else if (aType == typeof(Geom2d_Hyperbola))
        {
            return new Geom2dHash_HyperbolaHasher(CompTolerance, HashTolerance).IsEqual((Geom2d_Hyperbola)theCurve1, (Geom2d_Hyperbola)theCurve2);
        }
        else if (aType == typeof(Geom2d_Parabola))
        {
            return new Geom2dHash_ParabolaHasher(CompTolerance, HashTolerance).IsEqual((Geom2d_Parabola)theCurve1, (Geom2d_Parabola)theCurve2);
        }
        else if (aType == typeof(Geom2d_BezierCurve))
        {
            return new Geom2dHash_BezierCurveHasher(CompTolerance, HashTolerance).IsEqual((Geom2d_BezierCurve)theCurve1, (Geom2d_BezierCurve)theCurve2);
        }
        else if (aType == typeof(Geom2d_BSplineCurve))
        {
            return new Geom2dHash_BSplineCurveHasher(CompTolerance, HashTolerance).IsEqual((Geom2d_BSplineCurve)theCurve1, (Geom2d_BSplineCurve)theCurve2);
        }
        else if (aType == typeof(Geom2d_TrimmedCurve))
        {
            return new Geom2dHash_TrimmedCurveHasher(CompTolerance, HashTolerance).IsEqual((Geom2d_TrimmedCurve)theCurve1, (Geom2d_TrimmedCurve)theCurve2);
        }
        else if (aType == typeof(Geom2d_OffsetCurve))
        {
            return new Geom2dHash_OffsetCurveHasher(CompTolerance, HashTolerance).IsEqual((Geom2d_OffsetCurve)theCurve1, (Geom2d_OffsetCurve)theCurve2);
        }

        // Unknown type - compare by pointer
        return ReferenceEquals(theCurve1, theCurve2);
    }
}
