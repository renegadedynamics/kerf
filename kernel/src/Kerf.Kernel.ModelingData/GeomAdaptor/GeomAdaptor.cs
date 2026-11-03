// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// Conversions from adaptors back to Geom curves and surfaces.
/// </summary>
public static class GeomAdaptor
{
    /// <summary>
    /// Builds a Geom curve from a curve adaptor: a new line, conic, or a copy of the Bezier or B-spline curve,
    /// trimmed to the adaptor's bounds when they differ from the curve's.
    /// </summary>
    /// <exception cref="DomainErrorException">The adaptor's type is not one of those.</exception>
    public static Curve3D MakeCurve(CurveAdaptorBase hc)
    {
        Curve3D c;

        switch (hc.GetType())
        {
            case Line:
                c = new LineCurve3D(hc.Line());
                break;

            case Circle:
                c = new CircleCurve3D(hc.Circle());
                break;

            case Ellipse:
                c = new EllipseCurve3D(hc.Ellipse());
                break;

            case Parabola:
                c = new ParabolaCurve3D(hc.Parabola());
                break;

            case Hyperbola:
                c = new HyperbolaCurve3D(hc.Hyperbola());
                break;

            case BezierCurve:
                c = (BezierCurve3D)hc.Bezier().Copy();
                break;

            case BSplineCurve:
                c = (BSplineCurve3D)hc.BSpline().Copy();
                break;

            default:
                throw new DomainErrorException("GeomAdaptor::MakeCurve : OtherCurve");
        }

        // trim the curve if necassary.
        if ((hc.FirstParameter() != c.FirstParameter) || (hc.LastParameter() != c.LastParameter))
        {
            c = new TrimmedCurve3D(c, hc.FirstParameter(), hc.LastParameter());
        }

        return c;
    }

    /// <summary>
    /// Builds a Geom surface from a surface adaptor: a new elementary, swept or offset surface, or a copy of the
    /// Bezier or B-spline surface, trimmed to the adaptor's bounds when they differ from the surface's (and
    /// theTrimFlag).
    /// </summary>
    /// <exception cref="DomainErrorException">The adaptor's type is GeomAbs_OtherSurface.</exception>
    public static Surface MakeSurface(SurfaceAdaptorBase hs, bool trimFlag = true)
    {
        Surface s;

        switch (hs.GetType())
        {
            case Plane:
                s = new PlaneSurface(hs.Plane());
                break;

            case SurfaceKind.Cylinder:
                s = new CylindricalSurface(hs.Cylinder());
                break;

            case SurfaceKind.Cone:
                s = new ConicalSurface(hs.Cone());
                break;

            case SurfaceKind.Sphere:
                s = new SphericalSurface(hs.Sphere());
                break;

            case SurfaceKind.Torus:
                s = new ToroidalSurface(hs.Torus());
                break;

            case SurfaceKind.BezierSurface:
                s = (global::Kerf.Geometry.BezierSurface)hs.Bezier().Copy();
                break;

            case SurfaceKind.BSplineSurface:
                s = (global::Kerf.Geometry.BSplineSurface)hs.BSpline().Copy();
                break;

            case SurfaceOfRevolution:
                s = new RevolutionSurface(MakeCurve(hs.BasisCurve()), hs.AxeOfRevolution());
                break;

            case SurfaceOfExtrusion:
                s = new ExtrusionSurface(MakeCurve(hs.BasisCurve()), hs.Direction());
                break;

            case SurfaceKind.OffsetSurface:
                s = new global::Kerf.Geometry.OffsetSurface(MakeSurface(hs.BasisSurface()), hs.OffsetValue());
                break;

            case OtherSurface:
            default:
                throw new DomainErrorException("GeomAdaptor::MakeSurface : OtherSurface");
        }

        if (!trimFlag)
        {
            return s;
        }

        // trim the surface if necassary.
        s.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        if ((hs.FirstUParameter() != u1) || (hs.LastUParameter() != u2) || (hs.FirstVParameter() != v1)
            || (hs.LastVParameter() != v2))
        {
            s = new TrimmedSurface(s,
                                                   hs.FirstUParameter(),
                                                   hs.LastUParameter(),
                                                   hs.FirstVParameter(),
                                                   hs.LastVParameter());
        }

        return s;
    }
}
