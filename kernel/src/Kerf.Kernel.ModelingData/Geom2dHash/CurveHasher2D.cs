// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_CurveHasher.hxx
// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_CurveHasher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>
/// Polymorphic hasher of Geom2d_Curve, used for geometry deduplication.
/// </summary>
/// <remarks>
/// OCCT's functor is an <see cref="IEqualityComparer{T}"/>: <c>aHasher(x)</c> is <see cref="GetHashCode(Curve2D)"/>
/// and <c>aHasher(a, b)</c> is <see cref="Equals(Curve2D, Curve2D)"/>. As in OCCT, numeric fields are compared within
/// <see cref="CompTolerance"/> and rounded to multiples of <see cref="HashTolerance"/> before hashing (so objects
/// equal within the tolerance can still hash differently across a rounding boundary); Bezier and B-spline objects
/// hash their degrees and counts only. The hash values themselves are not OCCT's.
/// </remarks>
internal sealed class CurveHasher2D : IEqualityComparer<Curve2D?>
{
    /// <summary>The tolerance of the comparison of numeric fields.</summary>
    public readonly double CompTolerance;

    /// <summary>The step to which numeric fields are rounded before hashing.</summary>
    public readonly double HashTolerance;

    /// <summary>A hasher with the given tolerances (OCCT's defaults: Precision::Angular() and Precision::Confusion()).</summary>
    public CurveHasher2D(double compTolerance = 1.0e-12, double hashTolerance = 1.0e-7)
    {
        CompTolerance = compTolerance;
        HashTolerance = hashTolerance;
    }

    /// <summary>Hashes any Geom2d_Curve by dispatching to the hasher of its type; 0 for null.</summary>
    public int GetHashCode(Curve2D? curve) => HashUtils.ToInt32(Hash(curve));

    // OCCT's size_t operator()(const handle&)
    internal ulong Hash(Curve2D? curve)
    {
        if (curve is null)
        {
            return 0;
        }

        // Dispatch based on actual type
        Type type = curve.DynamicType();
        if (type == typeof(LineCurve2D))
        {
            return new LineHasher2D(CompTolerance, HashTolerance).Hash((LineCurve2D)curve);
        }
        else if (type == typeof(CircleCurve2D))
        {
            return new CircleHasher2D(CompTolerance, HashTolerance).Hash((CircleCurve2D)curve);
        }
        else if (type == typeof(EllipseCurve2D))
        {
            return new EllipseHasher2D(CompTolerance, HashTolerance).Hash((EllipseCurve2D)curve);
        }
        else if (type == typeof(HyperbolaCurve2D))
        {
            return new HyperbolaHasher2D(CompTolerance, HashTolerance).Hash((HyperbolaCurve2D)curve);
        }
        else if (type == typeof(ParabolaCurve2D))
        {
            return new ParabolaHasher2D(CompTolerance, HashTolerance).Hash((ParabolaCurve2D)curve);
        }
        else if (type == typeof(BezierCurve2D))
        {
            return new BezierCurveHasher2D(CompTolerance, HashTolerance).Hash((BezierCurve2D)curve);
        }
        else if (type == typeof(BSplineCurve2D))
        {
            return new BSplineCurveHasher2D(CompTolerance, HashTolerance).Hash((BSplineCurve2D)curve);
        }
        else if (type == typeof(TrimmedCurve2D))
        {
            return new TrimmedCurveHasher2D(CompTolerance, HashTolerance).Hash((TrimmedCurve2D)curve);
        }
        else if (type == typeof(OffsetCurve2D))
        {
            return new OffsetCurveHasher2D(CompTolerance, HashTolerance).Hash((OffsetCurve2D)curve);
        }

        // Unknown type - hash the type name
        return HashUtils.Hash(global::Kerf.Engine.Foundation.OcctTypeNames.Of(type));
    }

    /// <summary>Compares two objects of the same type with the hasher of that type; null equals only null.</summary>
    public bool Equals(Curve2D? curve1, Curve2D? curve2)
    {
        if (curve1 is null || curve2 is null)
        {
            return curve1 is null && curve2 is null;
        }

        if (ReferenceEquals(curve1, curve2))
        {
            return true;
        }

        // Must be same type
        if (curve1.DynamicType() != curve2.DynamicType())
        {
            return false;
        }

        // Dispatch based on actual type (already confirmed types match)
        Type type = curve1.DynamicType();
        if (type == typeof(LineCurve2D))
        {
            return new LineHasher2D(CompTolerance, HashTolerance).IsEqual((LineCurve2D)curve1, (LineCurve2D)curve2);
        }
        else if (type == typeof(CircleCurve2D))
        {
            return new CircleHasher2D(CompTolerance, HashTolerance).IsEqual((CircleCurve2D)curve1, (CircleCurve2D)curve2);
        }
        else if (type == typeof(EllipseCurve2D))
        {
            return new EllipseHasher2D(CompTolerance, HashTolerance).IsEqual((EllipseCurve2D)curve1, (EllipseCurve2D)curve2);
        }
        else if (type == typeof(HyperbolaCurve2D))
        {
            return new HyperbolaHasher2D(CompTolerance, HashTolerance).IsEqual((HyperbolaCurve2D)curve1, (HyperbolaCurve2D)curve2);
        }
        else if (type == typeof(ParabolaCurve2D))
        {
            return new ParabolaHasher2D(CompTolerance, HashTolerance).IsEqual((ParabolaCurve2D)curve1, (ParabolaCurve2D)curve2);
        }
        else if (type == typeof(BezierCurve2D))
        {
            return new BezierCurveHasher2D(CompTolerance, HashTolerance).IsEqual((BezierCurve2D)curve1, (BezierCurve2D)curve2);
        }
        else if (type == typeof(BSplineCurve2D))
        {
            return new BSplineCurveHasher2D(CompTolerance, HashTolerance).IsEqual((BSplineCurve2D)curve1, (BSplineCurve2D)curve2);
        }
        else if (type == typeof(TrimmedCurve2D))
        {
            return new TrimmedCurveHasher2D(CompTolerance, HashTolerance).IsEqual((TrimmedCurve2D)curve1, (TrimmedCurve2D)curve2);
        }
        else if (type == typeof(OffsetCurve2D))
        {
            return new OffsetCurveHasher2D(CompTolerance, HashTolerance).IsEqual((OffsetCurve2D)curve1, (OffsetCurve2D)curve2);
        }

        // Unknown type - compare by pointer
        return ReferenceEquals(curve1, curve2);
    }
}
