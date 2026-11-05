// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_CurveHasher.hxx
// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_CurveHasher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Engine;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>
/// Polymorphic hasher of Geom_Curve, used for geometry deduplication.
/// </summary>
/// <remarks>
/// OCCT's functor is an <see cref="IEqualityComparer{T}"/>: <c>aHasher(x)</c> is <see cref="GetHashCode(Curve3D)"/>
/// and <c>aHasher(a, b)</c> is <see cref="Equals(Curve3D, Curve3D)"/>. As in OCCT, numeric fields are compared within
/// <see cref="CompTolerance"/> and rounded to multiples of <see cref="HashTolerance"/> before hashing (so objects
/// equal within the tolerance can still hash differently across a rounding boundary); Bezier and B-spline objects
/// hash their degrees and counts only. The hash values themselves are not OCCT's.
/// </remarks>
internal sealed class CurveHasher : IEqualityComparer<Curve3D?>
{
    /// <summary>The tolerance of the comparison of numeric fields.</summary>
    public readonly double CompTolerance;

    /// <summary>The step to which numeric fields are rounded before hashing.</summary>
    public readonly double HashTolerance;

    /// <summary>A hasher with the given tolerances (OCCT's defaults: Precision::Angular() and Precision::Confusion()).</summary>
    public CurveHasher(double compTolerance = 1.0e-12, double hashTolerance = 1.0e-7)
    {
        CompTolerance = compTolerance;
        HashTolerance = hashTolerance;
    }

    /// <summary>Hashes any Geom_Curve by dispatching to the hasher of its type; 0 for null.</summary>
    public int GetHashCode(Curve3D? curve) => HashUtils.ToInt32(Hash(curve));

    // OCCT's size_t operator()(const handle&)
    internal ulong Hash(Curve3D? curve)
    {
        if (curve is null)
        {
            return 0;
        }

        // Dispatch based on actual type
        Type type = curve.DynamicType();
        if (type == typeof(LineCurve3D))
        {
            return new LineHasher(CompTolerance, HashTolerance).Hash((LineCurve3D)curve);
        }
        else if (type == typeof(CircleCurve3D))
        {
            return new CircleHasher(CompTolerance, HashTolerance).Hash((CircleCurve3D)curve);
        }
        else if (type == typeof(EllipseCurve3D))
        {
            return new EllipseHasher(CompTolerance, HashTolerance).Hash((EllipseCurve3D)curve);
        }
        else if (type == typeof(HyperbolaCurve3D))
        {
            return new HyperbolaHasher(CompTolerance, HashTolerance).Hash((HyperbolaCurve3D)curve);
        }
        else if (type == typeof(ParabolaCurve3D))
        {
            return new ParabolaHasher(CompTolerance, HashTolerance).Hash((ParabolaCurve3D)curve);
        }
        else if (type == typeof(BezierCurve3D))
        {
            return new BezierCurveHasher(CompTolerance, HashTolerance).Hash((BezierCurve3D)curve);
        }
        else if (type == typeof(BSplineCurve3D))
        {
            return new BSplineCurveHasher(CompTolerance, HashTolerance).Hash((BSplineCurve3D)curve);
        }
        else if (type == typeof(TrimmedCurve3D))
        {
            return new TrimmedCurveHasher(CompTolerance, HashTolerance).Hash((TrimmedCurve3D)curve);
        }
        else if (type == typeof(OffsetCurve3D))
        {
            return new OffsetCurveHasher(CompTolerance, HashTolerance).Hash((OffsetCurve3D)curve);
        }

        // Unknown type - hash the type name
        return HashUtils.Hash(global::Kerf.Engine.Foundation.OcctTypeNames.Of(type));
    }

    /// <summary>Compares two objects of the same type with the hasher of that type; null equals only null.</summary>
    public bool Equals(Curve3D? curve1, Curve3D? curve2)
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
        if (type == typeof(LineCurve3D))
        {
            return new LineHasher(CompTolerance, HashTolerance).IsEqual((LineCurve3D)curve1, (LineCurve3D)curve2);
        }
        else if (type == typeof(CircleCurve3D))
        {
            return new CircleHasher(CompTolerance, HashTolerance).IsEqual((CircleCurve3D)curve1, (CircleCurve3D)curve2);
        }
        else if (type == typeof(EllipseCurve3D))
        {
            return new EllipseHasher(CompTolerance, HashTolerance).IsEqual((EllipseCurve3D)curve1, (EllipseCurve3D)curve2);
        }
        else if (type == typeof(HyperbolaCurve3D))
        {
            return new HyperbolaHasher(CompTolerance, HashTolerance).IsEqual((HyperbolaCurve3D)curve1, (HyperbolaCurve3D)curve2);
        }
        else if (type == typeof(ParabolaCurve3D))
        {
            return new ParabolaHasher(CompTolerance, HashTolerance).IsEqual((ParabolaCurve3D)curve1, (ParabolaCurve3D)curve2);
        }
        else if (type == typeof(BezierCurve3D))
        {
            return new BezierCurveHasher(CompTolerance, HashTolerance).IsEqual((BezierCurve3D)curve1, (BezierCurve3D)curve2);
        }
        else if (type == typeof(BSplineCurve3D))
        {
            return new BSplineCurveHasher(CompTolerance, HashTolerance).IsEqual((BSplineCurve3D)curve1, (BSplineCurve3D)curve2);
        }
        else if (type == typeof(TrimmedCurve3D))
        {
            return new TrimmedCurveHasher(CompTolerance, HashTolerance).IsEqual((TrimmedCurve3D)curve1, (TrimmedCurve3D)curve2);
        }
        else if (type == typeof(OffsetCurve3D))
        {
            return new OffsetCurveHasher(CompTolerance, HashTolerance).IsEqual((OffsetCurve3D)curve1, (OffsetCurve3D)curve2);
        }

        // Unknown type - compare by pointer
        return ReferenceEquals(curve1, curve2);
    }
}
