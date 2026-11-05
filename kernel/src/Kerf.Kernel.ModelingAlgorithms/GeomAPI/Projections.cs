// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Api;

/// <summary>
/// The GeomAPI package: an application programming interface for the geometry. Its classes call the algorithms
/// (projections, extrema, intersections, interpolation and approximation) with simple constructors, and keep the
/// low-level tools they use.
/// </summary>
internal static partial class Projections
{
    /// <summary>
    /// Builds the 2D curve of the projection of <paramref name="c"/> on the plane <paramref name="p"/>, in the
    /// plane's coordinates. Null when the projection is an offset or "other" curve.
    /// </summary>
    public static Curve2D? To2d(Curve3D c, in Plane3D p)
    {
        Curve2D? result = null;
        var hc = new CurveAdaptor(c);
        var plane = new PlaneSurface(p);
        var hs = new SurfaceAdaptor(plane);

        var proj = new ProjectedCurve(hs, hc);

        if (proj.GetType() != CurveKind.OffsetCurve && proj.GetType() != CurveKind.OtherCurve)
        {
            result = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(proj);
        }

        return result;
    }

    /// <summary>Builds the 3D curve of the 2D curve <paramref name="c"/> placed in the plane <paramref name="p"/>.</summary>
    public static Curve3D To3d(Curve2D c, in Plane3D p)
    {
        CurveAdaptor2D ahc = new(c);

        PlaneSurface thePlane = new(p);
        SurfaceAdaptor ahs = new(thePlane);

        CurveOnSurface cos = new(ahc, ahs);
        return global::Kerf.Engine.Geometry.Adaptors.GeomAdaptor.MakeCurve(cos);
    }
}
