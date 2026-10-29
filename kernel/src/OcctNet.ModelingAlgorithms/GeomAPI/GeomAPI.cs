// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
    public static Geom2d_Curve? To2d(Geom_Curve C, in gp_Pln P)
    {
        // TODO(6A): ProjLib_ProjectedCurve is not ported yet. OCCT:
        //   ProjLib_ProjectedCurve Proj(new GeomAdaptor_Surface(new Geom_Plane(P)), new GeomAdaptor_Curve(C));
        //   if (Proj.GetType() != GeomAbs_OffsetCurve && Proj.GetType() != GeomAbs_OtherCurve)
        //     result = Geom2dAdaptor::MakeCurve(Proj);
        _ = C;
        _ = P;
        throw new Standard_NotImplemented("TODO(6A): GeomAPI::To2d needs ProjLib_ProjectedCurve");
    }

    /// <summary>Builds the 3D curve of the 2D curve <paramref name="C"/> placed in the plane <paramref name="P"/>.</summary>
    public static Geom_Curve To3d(Geom2d_Curve C, in gp_Pln P)
    {
        Geom2dAdaptor_Curve AHC = new(C);

        Geom_Plane ThePlane = new(P);
        GeomAdaptor_Surface AHS = new(ThePlane);

        Adaptor3d_CurveOnSurface COS = new(AHC, AHS);
        return GeomAdaptor.MakeCurve(COS);
    }
}
