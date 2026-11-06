// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_SurfaceHasher.hxx
// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_SurfaceHasher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>
/// Polymorphic hasher of Geom_Surface, used for geometry deduplication.
/// </summary>
/// <remarks>
/// OCCT's functor is an <see cref="IEqualityComparer{T}"/>: <c>aHasher(x)</c> is <see cref="GetHashCode(Surface)"/>
/// and <c>aHasher(a, b)</c> is <see cref="Equals(Surface, Surface)"/>. As in OCCT, numeric fields are compared within
/// <see cref="CompTolerance"/> and rounded to multiples of <see cref="HashTolerance"/> before hashing (so objects
/// equal within the tolerance can still hash differently across a rounding boundary); Bezier and B-spline objects
/// hash their degrees and counts only. The hash values themselves are not OCCT's.
/// </remarks>
internal sealed class SurfaceHasher : IEqualityComparer<Surface?>
{
    /// <summary>The tolerance of the comparison of numeric fields.</summary>
    public readonly double CompTolerance;

    /// <summary>The step to which numeric fields are rounded before hashing.</summary>
    public readonly double HashTolerance;

    /// <summary>A hasher with the given tolerances (OCCT's defaults: Precision::Angular() and Precision::Confusion()).</summary>
    public SurfaceHasher(double compTolerance = 1.0e-12, double hashTolerance = 1.0e-7)
    {
        CompTolerance = compTolerance;
        HashTolerance = hashTolerance;
    }

    /// <summary>Hashes any Geom_Surface by dispatching to the hasher of its type; 0 for null.</summary>
    public int GetHashCode(Surface? surface) => HashUtils.ToInt32(Hash(surface));

    // OCCT's size_t operator()(const handle&)
    internal ulong Hash(Surface? surface)
    {
        if (surface is null)
        {
            return 0;
        }

        // Dispatch based on actual type
        Type type = surface.DynamicType();
        if (type == typeof(PlaneSurface))
        {
            return new PlaneHasher(CompTolerance, HashTolerance).Hash((PlaneSurface)surface);
        }
        else if (type == typeof(CylindricalSurface))
        {
            return new CylindricalSurfaceHasher(CompTolerance, HashTolerance).Hash((CylindricalSurface)surface);
        }
        else if (type == typeof(ConicalSurface))
        {
            return new ConicalSurfaceHasher(CompTolerance, HashTolerance).Hash((ConicalSurface)surface);
        }
        else if (type == typeof(SphericalSurface))
        {
            return new SphericalSurfaceHasher(CompTolerance, HashTolerance).Hash((SphericalSurface)surface);
        }
        else if (type == typeof(ToroidalSurface))
        {
            return new ToroidalSurfaceHasher(CompTolerance, HashTolerance).Hash((ToroidalSurface)surface);
        }
        else if (type == typeof(RevolutionSurface))
        {
            return new SurfaceOfRevolutionHasher(CompTolerance, HashTolerance).Hash((RevolutionSurface)surface);
        }
        else if (type == typeof(ExtrusionSurface))
        {
            return new SurfaceOfLinearExtrusionHasher(CompTolerance, HashTolerance).Hash((ExtrusionSurface)surface);
        }
        else if (type == typeof(BezierSurface))
        {
            return new BezierSurfaceHasher(CompTolerance, HashTolerance).Hash((BezierSurface)surface);
        }
        else if (type == typeof(BSplineSurface))
        {
            return new BSplineSurfaceHasher(CompTolerance, HashTolerance).Hash((BSplineSurface)surface);
        }
        else if (type == typeof(TrimmedSurface))
        {
            return new RectangularTrimmedSurfaceHasher(CompTolerance, HashTolerance).Hash((TrimmedSurface)surface);
        }
        else if (type == typeof(OffsetSurface))
        {
            return new OffsetSurfaceHasher(CompTolerance, HashTolerance).Hash((OffsetSurface)surface);
        }

        // Unknown type - hash the type name
        return HashUtils.Hash(global::Kerf.Engine.Foundation.OcctTypeNames.Of(type));
    }

    /// <summary>Compares two objects of the same type with the hasher of that type; null equals only null.</summary>
    public bool Equals(Surface? surface1, Surface? surface2)
    {
        if (surface1 is null || surface2 is null)
        {
            return surface1 is null && surface2 is null;
        }

        if (ReferenceEquals(surface1, surface2))
        {
            return true;
        }

        // Must be same type
        if (surface1.DynamicType() != surface2.DynamicType())
        {
            return false;
        }

        // Dispatch based on actual type (already confirmed types match)
        Type type = surface1.DynamicType();
        if (type == typeof(PlaneSurface))
        {
            return new PlaneHasher(CompTolerance, HashTolerance).IsEqual((PlaneSurface)surface1, (PlaneSurface)surface2);
        }
        else if (type == typeof(CylindricalSurface))
        {
            return new CylindricalSurfaceHasher(CompTolerance, HashTolerance).IsEqual((CylindricalSurface)surface1, (CylindricalSurface)surface2);
        }
        else if (type == typeof(ConicalSurface))
        {
            return new ConicalSurfaceHasher(CompTolerance, HashTolerance).IsEqual((ConicalSurface)surface1, (ConicalSurface)surface2);
        }
        else if (type == typeof(SphericalSurface))
        {
            return new SphericalSurfaceHasher(CompTolerance, HashTolerance).IsEqual((SphericalSurface)surface1, (SphericalSurface)surface2);
        }
        else if (type == typeof(ToroidalSurface))
        {
            return new ToroidalSurfaceHasher(CompTolerance, HashTolerance).IsEqual((ToroidalSurface)surface1, (ToroidalSurface)surface2);
        }
        else if (type == typeof(RevolutionSurface))
        {
            return new SurfaceOfRevolutionHasher(CompTolerance, HashTolerance).IsEqual((RevolutionSurface)surface1, (RevolutionSurface)surface2);
        }
        else if (type == typeof(ExtrusionSurface))
        {
            return new SurfaceOfLinearExtrusionHasher(CompTolerance, HashTolerance).IsEqual((ExtrusionSurface)surface1, (ExtrusionSurface)surface2);
        }
        else if (type == typeof(BezierSurface))
        {
            return new BezierSurfaceHasher(CompTolerance, HashTolerance).IsEqual((BezierSurface)surface1, (BezierSurface)surface2);
        }
        else if (type == typeof(BSplineSurface))
        {
            return new BSplineSurfaceHasher(CompTolerance, HashTolerance).IsEqual((BSplineSurface)surface1, (BSplineSurface)surface2);
        }
        else if (type == typeof(TrimmedSurface))
        {
            return new RectangularTrimmedSurfaceHasher(CompTolerance, HashTolerance).IsEqual((TrimmedSurface)surface1, (TrimmedSurface)surface2);
        }
        else if (type == typeof(OffsetSurface))
        {
            return new OffsetSurfaceHasher(CompTolerance, HashTolerance).IsEqual((OffsetSurface)surface1, (OffsetSurface)surface2);
        }

        // Unknown type - compare by pointer
        return ReferenceEquals(surface1, surface2);
    }
}
