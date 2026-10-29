// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_CurveHasher.hxx
// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_CurveHasher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Polymorphic hasher of Geom_Curve, used for geometry deduplication.
/// </summary>
/// <remarks>
/// OCCT's functor is an <see cref="IEqualityComparer{T}"/>: <c>aHasher(x)</c> is <see cref="GetHashCode(Geom_Curve)"/>
/// and <c>aHasher(a, b)</c> is <see cref="Equals(Geom_Curve, Geom_Curve)"/>. As in OCCT, numeric fields are compared within
/// <see cref="CompTolerance"/> and rounded to multiples of <see cref="HashTolerance"/> before hashing (so objects
/// equal within the tolerance can still hash differently across a rounding boundary); Bezier and B-spline objects
/// hash their degrees and counts only. The hash values themselves are not OCCT's.
/// </remarks>
public sealed class GeomHash_CurveHasher : IEqualityComparer<Geom_Curve?>
{
    /// <summary>The tolerance of the comparison of numeric fields.</summary>
    public readonly double CompTolerance;

    /// <summary>The step to which numeric fields are rounded before hashing.</summary>
    public readonly double HashTolerance;

    /// <summary>A hasher with the given tolerances (OCCT's defaults: Precision::Angular() and Precision::Confusion()).</summary>
    public GeomHash_CurveHasher(double theCompTolerance = 1.0e-12, double theHashTolerance = 1.0e-7)
    {
        CompTolerance = theCompTolerance;
        HashTolerance = theHashTolerance;
    }

    /// <summary>Hashes any Geom_Curve by dispatching to the hasher of its type; 0 for null.</summary>
    public int GetHashCode(Geom_Curve? theCurve) => GeomHash_HashUtils.ToInt32(Hash(theCurve));

    // OCCT's size_t operator()(const handle&)
    internal ulong Hash(Geom_Curve? theCurve)
    {
        if (theCurve is null)
        {
            return 0;
        }

        // Dispatch based on actual type
        Type aType = theCurve.DynamicType();
        if (aType == typeof(Geom_Line))
        {
            return new GeomHash_LineHasher(CompTolerance, HashTolerance).Hash((Geom_Line)theCurve);
        }
        else if (aType == typeof(Geom_Circle))
        {
            return new GeomHash_CircleHasher(CompTolerance, HashTolerance).Hash((Geom_Circle)theCurve);
        }
        else if (aType == typeof(Geom_Ellipse))
        {
            return new GeomHash_EllipseHasher(CompTolerance, HashTolerance).Hash((Geom_Ellipse)theCurve);
        }
        else if (aType == typeof(Geom_Hyperbola))
        {
            return new GeomHash_HyperbolaHasher(CompTolerance, HashTolerance).Hash((Geom_Hyperbola)theCurve);
        }
        else if (aType == typeof(Geom_Parabola))
        {
            return new GeomHash_ParabolaHasher(CompTolerance, HashTolerance).Hash((Geom_Parabola)theCurve);
        }
        else if (aType == typeof(Geom_BezierCurve))
        {
            return new GeomHash_BezierCurveHasher(CompTolerance, HashTolerance).Hash((Geom_BezierCurve)theCurve);
        }
        else if (aType == typeof(Geom_BSplineCurve))
        {
            return new GeomHash_BSplineCurveHasher(CompTolerance, HashTolerance).Hash((Geom_BSplineCurve)theCurve);
        }
        else if (aType == typeof(Geom_TrimmedCurve))
        {
            return new GeomHash_TrimmedCurveHasher(CompTolerance, HashTolerance).Hash((Geom_TrimmedCurve)theCurve);
        }
        else if (aType == typeof(Geom_OffsetCurve))
        {
            return new GeomHash_OffsetCurveHasher(CompTolerance, HashTolerance).Hash((Geom_OffsetCurve)theCurve);
        }

        // Unknown type - hash the type name
        return GeomHash_HashUtils.Hash(aType.Name);
    }

    /// <summary>Compares two objects of the same type with the hasher of that type; null equals only null.</summary>
    public bool Equals(Geom_Curve? theCurve1, Geom_Curve? theCurve2)
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
        if (aType == typeof(Geom_Line))
        {
            return new GeomHash_LineHasher(CompTolerance, HashTolerance).IsEqual((Geom_Line)theCurve1, (Geom_Line)theCurve2);
        }
        else if (aType == typeof(Geom_Circle))
        {
            return new GeomHash_CircleHasher(CompTolerance, HashTolerance).IsEqual((Geom_Circle)theCurve1, (Geom_Circle)theCurve2);
        }
        else if (aType == typeof(Geom_Ellipse))
        {
            return new GeomHash_EllipseHasher(CompTolerance, HashTolerance).IsEqual((Geom_Ellipse)theCurve1, (Geom_Ellipse)theCurve2);
        }
        else if (aType == typeof(Geom_Hyperbola))
        {
            return new GeomHash_HyperbolaHasher(CompTolerance, HashTolerance).IsEqual((Geom_Hyperbola)theCurve1, (Geom_Hyperbola)theCurve2);
        }
        else if (aType == typeof(Geom_Parabola))
        {
            return new GeomHash_ParabolaHasher(CompTolerance, HashTolerance).IsEqual((Geom_Parabola)theCurve1, (Geom_Parabola)theCurve2);
        }
        else if (aType == typeof(Geom_BezierCurve))
        {
            return new GeomHash_BezierCurveHasher(CompTolerance, HashTolerance).IsEqual((Geom_BezierCurve)theCurve1, (Geom_BezierCurve)theCurve2);
        }
        else if (aType == typeof(Geom_BSplineCurve))
        {
            return new GeomHash_BSplineCurveHasher(CompTolerance, HashTolerance).IsEqual((Geom_BSplineCurve)theCurve1, (Geom_BSplineCurve)theCurve2);
        }
        else if (aType == typeof(Geom_TrimmedCurve))
        {
            return new GeomHash_TrimmedCurveHasher(CompTolerance, HashTolerance).IsEqual((Geom_TrimmedCurve)theCurve1, (Geom_TrimmedCurve)theCurve2);
        }
        else if (aType == typeof(Geom_OffsetCurve))
        {
            return new GeomHash_OffsetCurveHasher(CompTolerance, HashTolerance).IsEqual((Geom_OffsetCurve)theCurve1, (Geom_OffsetCurve)theCurve2);
        }

        // Unknown type - compare by pointer
        return ReferenceEquals(theCurve1, theCurve2);
    }
}
