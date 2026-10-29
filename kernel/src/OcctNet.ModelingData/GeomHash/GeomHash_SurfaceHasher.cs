// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_SurfaceHasher.hxx
// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_SurfaceHasher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Polymorphic hasher of Geom_Surface, used for geometry deduplication.
/// </summary>
/// <remarks>
/// OCCT's functor is an <see cref="IEqualityComparer{T}"/>: <c>aHasher(x)</c> is <see cref="GetHashCode(Geom_Surface)"/>
/// and <c>aHasher(a, b)</c> is <see cref="Equals(Geom_Surface, Geom_Surface)"/>. As in OCCT, numeric fields are compared within
/// <see cref="CompTolerance"/> and rounded to multiples of <see cref="HashTolerance"/> before hashing (so objects
/// equal within the tolerance can still hash differently across a rounding boundary); Bezier and B-spline objects
/// hash their degrees and counts only. The hash values themselves are not OCCT's.
/// </remarks>
public sealed class GeomHash_SurfaceHasher : IEqualityComparer<Geom_Surface?>
{
    /// <summary>The tolerance of the comparison of numeric fields.</summary>
    public readonly double CompTolerance;

    /// <summary>The step to which numeric fields are rounded before hashing.</summary>
    public readonly double HashTolerance;

    /// <summary>A hasher with the given tolerances (OCCT's defaults: Precision::Angular() and Precision::Confusion()).</summary>
    public GeomHash_SurfaceHasher(double theCompTolerance = 1.0e-12, double theHashTolerance = 1.0e-7)
    {
        CompTolerance = theCompTolerance;
        HashTolerance = theHashTolerance;
    }

    /// <summary>Hashes any Geom_Surface by dispatching to the hasher of its type; 0 for null.</summary>
    public int GetHashCode(Geom_Surface? theSurface) => GeomHash_HashUtils.ToInt32(Hash(theSurface));

    // OCCT's size_t operator()(const handle&)
    internal ulong Hash(Geom_Surface? theSurface)
    {
        if (theSurface is null)
        {
            return 0;
        }

        // Dispatch based on actual type
        Type aType = theSurface.DynamicType();
        if (aType == typeof(Geom_Plane))
        {
            return new GeomHash_PlaneHasher(CompTolerance, HashTolerance).Hash((Geom_Plane)theSurface);
        }
        else if (aType == typeof(Geom_CylindricalSurface))
        {
            return new GeomHash_CylindricalSurfaceHasher(CompTolerance, HashTolerance).Hash((Geom_CylindricalSurface)theSurface);
        }
        else if (aType == typeof(Geom_ConicalSurface))
        {
            return new GeomHash_ConicalSurfaceHasher(CompTolerance, HashTolerance).Hash((Geom_ConicalSurface)theSurface);
        }
        else if (aType == typeof(Geom_SphericalSurface))
        {
            return new GeomHash_SphericalSurfaceHasher(CompTolerance, HashTolerance).Hash((Geom_SphericalSurface)theSurface);
        }
        else if (aType == typeof(Geom_ToroidalSurface))
        {
            return new GeomHash_ToroidalSurfaceHasher(CompTolerance, HashTolerance).Hash((Geom_ToroidalSurface)theSurface);
        }
        else if (aType == typeof(Geom_SurfaceOfRevolution))
        {
            return new GeomHash_SurfaceOfRevolutionHasher(CompTolerance, HashTolerance).Hash((Geom_SurfaceOfRevolution)theSurface);
        }
        else if (aType == typeof(Geom_SurfaceOfLinearExtrusion))
        {
            return new GeomHash_SurfaceOfLinearExtrusionHasher(CompTolerance, HashTolerance).Hash((Geom_SurfaceOfLinearExtrusion)theSurface);
        }
        else if (aType == typeof(Geom_BezierSurface))
        {
            return new GeomHash_BezierSurfaceHasher(CompTolerance, HashTolerance).Hash((Geom_BezierSurface)theSurface);
        }
        else if (aType == typeof(Geom_BSplineSurface))
        {
            return new GeomHash_BSplineSurfaceHasher(CompTolerance, HashTolerance).Hash((Geom_BSplineSurface)theSurface);
        }
        else if (aType == typeof(Geom_RectangularTrimmedSurface))
        {
            return new GeomHash_RectangularTrimmedSurfaceHasher(CompTolerance, HashTolerance).Hash((Geom_RectangularTrimmedSurface)theSurface);
        }
        else if (aType == typeof(Geom_OffsetSurface))
        {
            return new GeomHash_OffsetSurfaceHasher(CompTolerance, HashTolerance).Hash((Geom_OffsetSurface)theSurface);
        }

        // Unknown type - hash the type name
        return GeomHash_HashUtils.Hash(aType.Name);
    }

    /// <summary>Compares two objects of the same type with the hasher of that type; null equals only null.</summary>
    public bool Equals(Geom_Surface? theSurface1, Geom_Surface? theSurface2)
    {
        if (theSurface1 is null || theSurface2 is null)
        {
            return theSurface1 is null && theSurface2 is null;
        }

        if (ReferenceEquals(theSurface1, theSurface2))
        {
            return true;
        }

        // Must be same type
        if (theSurface1.DynamicType() != theSurface2.DynamicType())
        {
            return false;
        }

        // Dispatch based on actual type (already confirmed types match)
        Type aType = theSurface1.DynamicType();
        if (aType == typeof(Geom_Plane))
        {
            return new GeomHash_PlaneHasher(CompTolerance, HashTolerance).IsEqual((Geom_Plane)theSurface1, (Geom_Plane)theSurface2);
        }
        else if (aType == typeof(Geom_CylindricalSurface))
        {
            return new GeomHash_CylindricalSurfaceHasher(CompTolerance, HashTolerance).IsEqual((Geom_CylindricalSurface)theSurface1, (Geom_CylindricalSurface)theSurface2);
        }
        else if (aType == typeof(Geom_ConicalSurface))
        {
            return new GeomHash_ConicalSurfaceHasher(CompTolerance, HashTolerance).IsEqual((Geom_ConicalSurface)theSurface1, (Geom_ConicalSurface)theSurface2);
        }
        else if (aType == typeof(Geom_SphericalSurface))
        {
            return new GeomHash_SphericalSurfaceHasher(CompTolerance, HashTolerance).IsEqual((Geom_SphericalSurface)theSurface1, (Geom_SphericalSurface)theSurface2);
        }
        else if (aType == typeof(Geom_ToroidalSurface))
        {
            return new GeomHash_ToroidalSurfaceHasher(CompTolerance, HashTolerance).IsEqual((Geom_ToroidalSurface)theSurface1, (Geom_ToroidalSurface)theSurface2);
        }
        else if (aType == typeof(Geom_SurfaceOfRevolution))
        {
            return new GeomHash_SurfaceOfRevolutionHasher(CompTolerance, HashTolerance).IsEqual((Geom_SurfaceOfRevolution)theSurface1, (Geom_SurfaceOfRevolution)theSurface2);
        }
        else if (aType == typeof(Geom_SurfaceOfLinearExtrusion))
        {
            return new GeomHash_SurfaceOfLinearExtrusionHasher(CompTolerance, HashTolerance).IsEqual((Geom_SurfaceOfLinearExtrusion)theSurface1, (Geom_SurfaceOfLinearExtrusion)theSurface2);
        }
        else if (aType == typeof(Geom_BezierSurface))
        {
            return new GeomHash_BezierSurfaceHasher(CompTolerance, HashTolerance).IsEqual((Geom_BezierSurface)theSurface1, (Geom_BezierSurface)theSurface2);
        }
        else if (aType == typeof(Geom_BSplineSurface))
        {
            return new GeomHash_BSplineSurfaceHasher(CompTolerance, HashTolerance).IsEqual((Geom_BSplineSurface)theSurface1, (Geom_BSplineSurface)theSurface2);
        }
        else if (aType == typeof(Geom_RectangularTrimmedSurface))
        {
            return new GeomHash_RectangularTrimmedSurfaceHasher(CompTolerance, HashTolerance).IsEqual((Geom_RectangularTrimmedSurface)theSurface1, (Geom_RectangularTrimmedSurface)theSurface2);
        }
        else if (aType == typeof(Geom_OffsetSurface))
        {
            return new GeomHash_OffsetSurfaceHasher(CompTolerance, HashTolerance).IsEqual((Geom_OffsetSurface)theSurface1, (Geom_OffsetSurface)theSurface2);
        }

        // Unknown type - compare by pointer
        return ReferenceEquals(theSurface1, theSurface2);
    }
}
