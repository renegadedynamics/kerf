// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// Conversions from adaptors back to Geom curves and surfaces.
/// </summary>
public static class GeomAdaptor
{
    /// <summary>
    /// Builds a Geom curve from a curve adaptor: a new line, conic, or a copy of the Bezier or B-spline curve,
    /// trimmed to the adaptor's bounds when they differ from the curve's.
    /// </summary>
    /// <exception cref="Standard_DomainError">The adaptor's type is not one of those.</exception>
    public static Geom_Curve MakeCurve(Adaptor3d_Curve HC)
    {
        Geom_Curve C;

        switch (HC.GetType())
        {
            case GeomAbs_Line:
                C = new Geom_Line(HC.Line());
                break;

            case GeomAbs_Circle:
                C = new Geom_Circle(HC.Circle());
                break;

            case GeomAbs_Ellipse:
                C = new Geom_Ellipse(HC.Ellipse());
                break;

            case GeomAbs_Parabola:
                C = new Geom_Parabola(HC.Parabola());
                break;

            case GeomAbs_Hyperbola:
                C = new Geom_Hyperbola(HC.Hyperbola());
                break;

            case GeomAbs_BezierCurve:
                C = (Geom_BezierCurve)HC.Bezier().Copy();
                break;

            case GeomAbs_BSplineCurve:
                C = (Geom_BSplineCurve)HC.BSpline().Copy();
                break;

            default:
                throw new Standard_DomainError("GeomAdaptor::MakeCurve : OtherCurve");
        }

        // trim the curve if necassary.
        if ((HC.FirstParameter() != C.FirstParameter()) || (HC.LastParameter() != C.LastParameter()))
        {
            C = new Geom_TrimmedCurve(C, HC.FirstParameter(), HC.LastParameter());
        }

        return C;
    }

    /// <summary>
    /// Builds a Geom surface from a surface adaptor: a new elementary, swept or offset surface, or a copy of the
    /// Bezier or B-spline surface, trimmed to the adaptor's bounds when they differ from the surface's (and
    /// theTrimFlag).
    /// </summary>
    /// <exception cref="Standard_DomainError">The adaptor's type is GeomAbs_OtherSurface.</exception>
    public static Geom_Surface MakeSurface(Adaptor3d_Surface HS, bool theTrimFlag = true)
    {
        Geom_Surface S;

        switch (HS.GetType())
        {
            case GeomAbs_Plane:
                S = new Geom_Plane(HS.Plane());
                break;

            case GeomAbs_Cylinder:
                S = new Geom_CylindricalSurface(HS.Cylinder());
                break;

            case GeomAbs_Cone:
                S = new Geom_ConicalSurface(HS.Cone());
                break;

            case GeomAbs_Sphere:
                S = new Geom_SphericalSurface(HS.Sphere());
                break;

            case GeomAbs_Torus:
                S = new Geom_ToroidalSurface(HS.Torus());
                break;

            case GeomAbs_BezierSurface:
                S = (Geom_BezierSurface)HS.Bezier().Copy();
                break;

            case GeomAbs_BSplineSurface:
                S = (Geom_BSplineSurface)HS.BSpline().Copy();
                break;

            case GeomAbs_SurfaceOfRevolution:
                S = new Geom_SurfaceOfRevolution(MakeCurve(HS.BasisCurve()), HS.AxeOfRevolution());
                break;

            case GeomAbs_SurfaceOfExtrusion:
                S = new Geom_SurfaceOfLinearExtrusion(MakeCurve(HS.BasisCurve()), HS.Direction());
                break;

            case GeomAbs_OffsetSurface:
                S = new Geom_OffsetSurface(MakeSurface(HS.BasisSurface()), HS.OffsetValue());
                break;

            case GeomAbs_OtherSurface:
            default:
                throw new Standard_DomainError("GeomAdaptor::MakeSurface : OtherSurface");
        }

        if (!theTrimFlag)
        {
            return S;
        }

        // trim the surface if necassary.
        S.Bounds(out double U1, out double U2, out double V1, out double V2);
        if ((HS.FirstUParameter() != U1) || (HS.LastUParameter() != U2) || (HS.FirstVParameter() != V1)
            || (HS.LastVParameter() != V2))
        {
            S = new Geom_RectangularTrimmedSurface(S,
                                                   HS.FirstUParameter(),
                                                   HS.LastUParameter(),
                                                   HS.FirstVParameter(),
                                                   HS.LastVParameter());
        }

        return S;
    }
}
