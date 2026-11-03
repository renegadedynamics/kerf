// Port of: src/ModelingData/TKGeomBase/GeomProjLib/GeomProjLib.hxx
// Port of: src/ModelingData/TKGeomBase/GeomProjLib/GeomProjLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Created on: 1993-12-24
// Created by: Remi LEQUETTE

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>
/// Projection of a curve on a surface: the 2D curve of a 3D curve lying on a surface, the 3D curve of the normal
/// projection on a surface, and the projection on a plane along a direction.
/// </summary>
/// <remarks>
/// The functions return new curves the caller owns, unfrozen (decision 244), or null when the projection fails. The
/// caller's curves and surfaces are never changed.
/// </remarks>
public static class GeomProjLib
{
    /// <summary>
    /// The 2D curve of a 3D curve lying on a surface (ProjLib_ProjectedCurve), the 3D curve taken between
    /// <paramref name="first"/> and <paramref name="last"/> and the surface between the given bounds.
    /// <paramref name="tolerance"/> is used as input if the projection needs an approximation; the reached tolerance
    /// is set in it as output. Returns null if the projection has failed.
    /// </summary>
    public static Curve2D? Curve2d(Curve3D c,
                                        double first,
                                        double last,
                                        Surface s,
                                        double uDeb,
                                        double uFin,
                                        double vDeb,
                                        double vFin,
                                        ref double tolerance)
    {
        tolerance = Max(global::Kerf.Engine.Foundation.Precision.PConfusion(), tolerance);

        // OCCT builds local adaptors and copies them into handles: one construction each.
        SurfaceAdaptor hs = new(s, uDeb, uFin, vDeb, vFin);
        CurveAdaptor hc = new(c, first, last);

        ProjectedCurve proj = new(hs, hc, tolerance);

        Curve2D? g2dC = null;

        switch (proj.GetType())
        {
            case Line:
                g2dC = new LineCurve2D(proj.Line());
                break;

            case Circle:
                g2dC = new CircleCurve2D(proj.Circle());
                break;

            case Ellipse:
                g2dC = new EllipseCurve2D(proj.Ellipse());
                break;

            case Parabola:
                g2dC = new ParabolaCurve2D(proj.Parabola());
                break;

            case Hyperbola:
                g2dC = new HyperbolaCurve2D(proj.Hyperbola());
                break;

            case BezierCurve:
                // The projection is discarded: its curve is returned unfrozen (decision 244).
                g2dC = proj.InternalBezier();
                break;

            case BSplineCurve:
                g2dC = proj.InternalBSpline();
                break;

            default:
                return g2dC;
        }

        if (g2dC is null)
        {
            tolerance = proj.GetTolerance();
            return g2dC;
        }

        if (c is TrimmedCurve3D cTrim)
        {
            double u1 = cTrim.FirstParameter;
            double u2 = cTrim.LastParameter;
            if (!g2dC.IsPeriodic)
            {
                u1 = Max(u1, g2dC.FirstParameter);
                u2 = Min(u2, g2dC.LastParameter);
            }
            g2dC = new TrimmedCurve2D(g2dC, u1, u2);
        }

        tolerance = proj.GetTolerance();
        return g2dC;
    }

    /// <summary>
    /// The 2D curve of a 3D curve lying on a surface, the 3D curve taken between <paramref name="first"/> and
    /// <paramref name="last"/>. <paramref name="tolerance"/> is used as input if the projection needs an
    /// approximation; the reached tolerance is set in it as output. Returns null if the projection has failed.
    /// </summary>
    public static Curve2D? Curve2d(Curve3D c, double first, double last, Surface s, ref double tolerance)
    {
        double uFirst, uLast, vFirst, vLast;

        s.ParameterBounds(out uFirst, out uLast, out vFirst, out vLast);
        return Curve2d(c, first, last, s, uFirst, uLast, vFirst, vLast, ref tolerance);
    }

    //  Modified by skv - Wed Aug 11 17:26:03 2004 OCC6272 Begin
    //  Add not implemented method.

    /// <summary>
    /// The 2D curve of a 3D curve lying on a surface, the surface between the given bounds (which can expand the
    /// surface's a little). If the projection needs an approximation, Precision::PApproximation() is used. Returns
    /// null if the projection has failed.
    /// </summary>
    public static Curve2D? Curve2d(Curve3D c,
                                        Surface s,
                                        double uDeb,
                                        double uFin,
                                        double vDeb,
                                        double vFin)
    {
        double first = c.FirstParameter;
        double last = c.LastParameter;
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        return global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c, first, last, s, uDeb, uFin, vDeb, vFin, ref tol);
    }

    //  Modified by skv - Wed Aug 11 17:26:03 2004 OCC6272 End

    /// <summary>
    /// The 2D curve of a 3D curve lying on a surface, the surface between the given bounds.
    /// <paramref name="tolerance"/> is used as input if the projection needs an approximation; the reached tolerance
    /// is set in it as output. Returns null if the projection has failed.
    /// </summary>
    public static Curve2D? Curve2d(Curve3D c,
                                        Surface s,
                                        double uDeb,
                                        double uFin,
                                        double vDeb,
                                        double vFin,
                                        ref double tolerance)
    {
        double first = c.FirstParameter;
        double last = c.LastParameter;
        return global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c, first, last, s, uDeb, uFin, vDeb, vFin, ref tolerance);
    }

    /// <summary>
    /// The 2D curve of a 3D curve lying on a surface. If the projection needs an approximation,
    /// Precision::PApproximation() is used. Returns null if the projection has failed.
    /// </summary>
    public static Curve2D? Curve2d(Curve3D c, Surface s)
    {
        double first = c.FirstParameter;
        double last = c.LastParameter;
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        return global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c, first, last, s, ref tol);
    }

    /// <summary>
    /// The 2D curve of a 3D curve lying on a surface, the 3D curve taken between <paramref name="first"/> and
    /// <paramref name="last"/>. If the projection needs an approximation, Precision::PApproximation() is used.
    /// Returns null if the projection has failed.
    /// </summary>
    public static Curve2D? Curve2d(Curve3D c, double first, double last, Surface s)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        return global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c, first, last, s, ref tol);
    }

    /// <summary>
    /// The 3D curve of the normal projection of <paramref name="c"/> on <paramref name="s"/>. Returns null if the
    /// projection has failed.
    /// </summary>
    /// <exception cref="NoSuchObjectException">On a surface other than a plane, the curve doesn't project on the
    /// surface at all (ProjLib_CompProjectedCurve has no part), as in OCCT.</exception>
    public static Curve3D? Project(Curve3D c, Surface s)
    {
        CurveAdaptor ac = new(c);
        SurfaceAdaptor AS = new(s);

        Curve3D? gc = null;

        if (AS.GetType() == Plane)
        {
            ProjectOnPlane proj = new(AS.Plane().Position);
            // OCCT copies AC into a handle: AC itself is used.
            CurveAdaptor hc = ac;
            proj.Load(hc, global::Kerf.Engine.Foundation.Precision.PApproximation());

            switch (proj.GetType())
            {
                case Line:
                    gc = new LineCurve3D(proj.Line());
                    break;

                case Circle:
                    gc = new CircleCurve3D(proj.Circle());
                    break;

                case Ellipse:
                    gc = new EllipseCurve3D(proj.Ellipse());
                    break;

                case Parabola:
                    gc = new ParabolaCurve3D(proj.Parabola());
                    break;

                case Hyperbola:
                    gc = new HyperbolaCurve3D(proj.Hyperbola());
                    break;

                case BezierCurve:
                    // The projector is discarded: its curve is returned unfrozen (decision 244).
                    gc = proj.InternalBezier();
                    break;

                case BSplineCurve:
                    gc = proj.InternalBSpline();
                    break;

                default:
                    return gc;
            }

            if (c is TrimmedCurve3D cTrim)
            {
                double u1 = cTrim.FirstParameter;
                double u2 = cTrim.LastParameter;
                gc = new TrimmedCurve3D(gc, u1, u2);
            }
        }
        else
        {
            // OCCT copies AS and AC into handles: they are used themselves.
            SurfaceAdaptor hs = AS;
            CurveAdaptor hc = ac;
            //    double Tol  = Precision::Approximation();
            //    double TolU = Precision::PApproximation();
            //    double TolV = Precision::PApproximation();
            double tol = 0.0001;
            double tolU = Math.Pow(tol, 2.0 / 3);
            double tolV = Math.Pow(tol, 2.0 / 3);
            CompProjectedCurve proj = new(hs, hc, tolU, tolV, -1.0);

            double f, l;
            proj.Bounds(1, out f, out l);
            Curve2DAdaptorBase hc2d = proj.Trim(f, l, tolU);
            CurveOnSurfaceApproximation approx = new(hc2d, hs, f, l, tol);
            approx.Perform(16, 14, C2, true);

            // ici, on a toujours un type BSpline.
            if (approx.IsDone && approx.HasResult)
            {
                // The approximation is discarded: its curve is returned unfrozen (decision 244).
                gc = approx.InternalCurve3d();
            }
        }

        return gc;
    }

    /// <summary>
    /// The 3D curve of the projection of <paramref name="curve"/> on <paramref name="plane"/> along
    /// <paramref name="dir"/>. If <paramref name="keepParametrization"/> is true, the projected curve keeps the
    /// curve's parametrization: proj(C(u)) = PC(u) for each u; otherwise it may change. Returns null if the
    /// projection has failed.
    /// </summary>
    public static Curve3D? ProjectOnPlane(Curve3D curve, PlaneSurface plane, in Direction3D dir, bool keepParametrization)
    {
        // OCCT copies AC into a handle: AC itself is used.
        CurveAdaptor hc = new(curve);

        ProjectOnPlane proj = new(plane.Position, dir);
        proj.Load(hc, global::Kerf.Engine.Foundation.Precision.Approximation(), keepParametrization);

        Curve3D? gc = null;

        switch (proj.GetType())
        {
            case Line:
                gc = new LineCurve3D(proj.Line());
                break;

            case Circle:
                gc = new CircleCurve3D(proj.Circle());
                break;

            case Ellipse:
                gc = new EllipseCurve3D(proj.Ellipse());
                break;

            case Parabola:
                gc = new ParabolaCurve3D(proj.Parabola());
                break;

            case Hyperbola:
                gc = new HyperbolaCurve3D(proj.Hyperbola());
                break;

            case BezierCurve:
                // The projector is discarded: its curve is returned unfrozen (decision 244).
                gc = proj.InternalBezier();
                break;

            case BSplineCurve:
                gc = proj.InternalBSpline();
                break;
            default:
                return gc;
        }

        if (curve is TrimmedCurve3D)
        {
            gc = new TrimmedCurve3D(gc, proj.FirstParameter(), proj.LastParameter());
        }

        return gc;
    }
}
