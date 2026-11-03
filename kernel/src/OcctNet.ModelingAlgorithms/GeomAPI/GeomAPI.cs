// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The GeomAPI package: an application programming interface for the geometry. Its classes call the algorithms
/// (projections, extrema, intersections, interpolation and approximation) with simple constructors, and keep the
/// low-level tools they use.
/// </summary>
public static partial class GeomAPI
{
    /// <summary>
    /// Builds the 2D curve of the projection of <paramref name="C"/> on the plane <paramref name="P"/>, in the
    /// plane's coordinates. Null when the projection is an offset or "other" curve.
    /// </summary>
    public static Curve2D? To2d(Curve3D C, in Plane3D P)
    {
        Curve2D? result = null;
        var HC = new CurveAdaptor(C);
        var Plane = new PlaneSurface(P);
        var HS = new SurfaceAdaptor(Plane);

        var Proj = new ProjectedCurve(HS, HC);

        if (Proj.GetType() != CurveKind.OffsetCurve && Proj.GetType() != CurveKind.OtherCurve)
        {
            result = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(Proj);
        }

        return result;
    }

    /// <summary>Builds the 3D curve of the 2D curve <paramref name="C"/> placed in the plane <paramref name="P"/>.</summary>
    public static Curve3D To3d(Curve2D C, in Plane3D P)
    {
        CurveAdaptor2D AHC = new(C);

        PlaneSurface ThePlane = new(P);
        SurfaceAdaptor AHS = new(ThePlane);

        CurveOnSurface COS = new(AHC, AHS);
        return global::Kerf.Engine.Geometry.Adaptors.GeomAdaptor.MakeCurve(COS);
    }
}
