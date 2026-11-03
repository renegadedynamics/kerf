// Port of: src/ModelingData/TKGeomBase/GeomProjLib/GeomProjLib.hxx
// Port of: src/ModelingData/TKGeomBase/GeomProjLib/GeomProjLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Created on: 1993-12-24
// Created by: Remi LEQUETTE

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

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
    /// <paramref name="First"/> and <paramref name="Last"/> and the surface between the given bounds.
    /// <paramref name="Tolerance"/> is used as input if the projection needs an approximation; the reached tolerance
    /// is set in it as output. Returns null if the projection has failed.
    /// </summary>
    public static Geom2d_Curve? Curve2d(Geom_Curve C,
                                        double First,
                                        double Last,
                                        Geom_Surface S,
                                        double UDeb,
                                        double UFin,
                                        double VDeb,
                                        double VFin,
                                        ref double Tolerance)
    {
        Tolerance = Max(global::Kerf.Engine.Foundation.Precision.PConfusion(), Tolerance);

        // OCCT builds local adaptors and copies them into handles: one construction each.
        GeomAdaptor_Surface HS = new(S, UDeb, UFin, VDeb, VFin);
        GeomAdaptor_Curve HC = new(C, First, Last);

        ProjLib_ProjectedCurve Proj = new(HS, HC, Tolerance);

        Geom2d_Curve? G2dC = null;

        switch (Proj.GetType())
        {
            case Line:
                G2dC = new Geom2d_Line(Proj.Line());
                break;

            case Circle:
                G2dC = new Geom2d_Circle(Proj.Circle());
                break;

            case Ellipse:
                G2dC = new Geom2d_Ellipse(Proj.Ellipse());
                break;

            case Parabola:
                G2dC = new Geom2d_Parabola(Proj.Parabola());
                break;

            case Hyperbola:
                G2dC = new Geom2d_Hyperbola(Proj.Hyperbola());
                break;

            case BezierCurve:
                // The projection is discarded: its curve is returned unfrozen (decision 244).
                G2dC = Proj.InternalBezier();
                break;

            case BSplineCurve:
                G2dC = Proj.InternalBSpline();
                break;

            default:
                return G2dC;
        }

        if (G2dC is null)
        {
            Tolerance = Proj.GetTolerance();
            return G2dC;
        }

        if (C is Geom_TrimmedCurve CTrim)
        {
            double U1 = CTrim.FirstParameter();
            double U2 = CTrim.LastParameter();
            if (!G2dC.IsPeriodic())
            {
                U1 = Max(U1, G2dC.FirstParameter());
                U2 = Min(U2, G2dC.LastParameter());
            }
            G2dC = new Geom2d_TrimmedCurve(G2dC, U1, U2);
        }

        Tolerance = Proj.GetTolerance();
        return G2dC;
    }

    /// <summary>
    /// The 2D curve of a 3D curve lying on a surface, the 3D curve taken between <paramref name="First"/> and
    /// <paramref name="Last"/>. <paramref name="Tolerance"/> is used as input if the projection needs an
    /// approximation; the reached tolerance is set in it as output. Returns null if the projection has failed.
    /// </summary>
    public static Geom2d_Curve? Curve2d(Geom_Curve C, double First, double Last, Geom_Surface S, ref double Tolerance)
    {
        double UFirst, ULast, VFirst, VLast;

        S.Bounds(out UFirst, out ULast, out VFirst, out VLast);
        return Curve2d(C, First, Last, S, UFirst, ULast, VFirst, VLast, ref Tolerance);
    }

    //  Modified by skv - Wed Aug 11 17:26:03 2004 OCC6272 Begin
    //  Add not implemented method.

    /// <summary>
    /// The 2D curve of a 3D curve lying on a surface, the surface between the given bounds (which can expand the
    /// surface's a little). If the projection needs an approximation, Precision::PApproximation() is used. Returns
    /// null if the projection has failed.
    /// </summary>
    public static Geom2d_Curve? Curve2d(Geom_Curve C,
                                        Geom_Surface S,
                                        double UDeb,
                                        double UFin,
                                        double VDeb,
                                        double VFin)
    {
        double First = C.FirstParameter();
        double Last = C.LastParameter();
        double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        return GeomProjLib.Curve2d(C, First, Last, S, UDeb, UFin, VDeb, VFin, ref Tol);
    }

    //  Modified by skv - Wed Aug 11 17:26:03 2004 OCC6272 End

    /// <summary>
    /// The 2D curve of a 3D curve lying on a surface, the surface between the given bounds.
    /// <paramref name="Tolerance"/> is used as input if the projection needs an approximation; the reached tolerance
    /// is set in it as output. Returns null if the projection has failed.
    /// </summary>
    public static Geom2d_Curve? Curve2d(Geom_Curve C,
                                        Geom_Surface S,
                                        double UDeb,
                                        double UFin,
                                        double VDeb,
                                        double VFin,
                                        ref double Tolerance)
    {
        double First = C.FirstParameter();
        double Last = C.LastParameter();
        return GeomProjLib.Curve2d(C, First, Last, S, UDeb, UFin, VDeb, VFin, ref Tolerance);
    }

    /// <summary>
    /// The 2D curve of a 3D curve lying on a surface. If the projection needs an approximation,
    /// Precision::PApproximation() is used. Returns null if the projection has failed.
    /// </summary>
    public static Geom2d_Curve? Curve2d(Geom_Curve C, Geom_Surface S)
    {
        double First = C.FirstParameter();
        double Last = C.LastParameter();
        double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        return GeomProjLib.Curve2d(C, First, Last, S, ref Tol);
    }

    /// <summary>
    /// The 2D curve of a 3D curve lying on a surface, the 3D curve taken between <paramref name="First"/> and
    /// <paramref name="Last"/>. If the projection needs an approximation, Precision::PApproximation() is used.
    /// Returns null if the projection has failed.
    /// </summary>
    public static Geom2d_Curve? Curve2d(Geom_Curve C, double First, double Last, Geom_Surface S)
    {
        double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        return GeomProjLib.Curve2d(C, First, Last, S, ref Tol);
    }

    /// <summary>
    /// The 3D curve of the normal projection of <paramref name="C"/> on <paramref name="S"/>. Returns null if the
    /// projection has failed.
    /// </summary>
    /// <exception cref="NoSuchObjectException">On a surface other than a plane, the curve doesn't project on the
    /// surface at all (ProjLib_CompProjectedCurve has no part), as in OCCT.</exception>
    public static Geom_Curve? Project(Geom_Curve C, Geom_Surface S)
    {
        GeomAdaptor_Curve AC = new(C);
        GeomAdaptor_Surface AS = new(S);

        Geom_Curve? GC = null;

        if (AS.GetType() == Plane)
        {
            ProjLib_ProjectOnPlane Proj = new(AS.Plane().Position());
            // OCCT copies AC into a handle: AC itself is used.
            GeomAdaptor_Curve HC = AC;
            Proj.Load(HC, global::Kerf.Engine.Foundation.Precision.PApproximation());

            switch (Proj.GetType())
            {
                case Line:
                    GC = new Geom_Line(Proj.Line());
                    break;

                case Circle:
                    GC = new Geom_Circle(Proj.Circle());
                    break;

                case Ellipse:
                    GC = new Geom_Ellipse(Proj.Ellipse());
                    break;

                case Parabola:
                    GC = new Geom_Parabola(Proj.Parabola());
                    break;

                case Hyperbola:
                    GC = new Geom_Hyperbola(Proj.Hyperbola());
                    break;

                case BezierCurve:
                    // The projector is discarded: its curve is returned unfrozen (decision 244).
                    GC = Proj.InternalBezier();
                    break;

                case BSplineCurve:
                    GC = Proj.InternalBSpline();
                    break;

                default:
                    return GC;
            }

            if (C is Geom_TrimmedCurve CTrim)
            {
                double U1 = CTrim.FirstParameter();
                double U2 = CTrim.LastParameter();
                GC = new Geom_TrimmedCurve(GC, U1, U2);
            }
        }
        else
        {
            // OCCT copies AS and AC into handles: they are used themselves.
            GeomAdaptor_Surface HS = AS;
            GeomAdaptor_Curve HC = AC;
            //    double Tol  = Precision::Approximation();
            //    double TolU = Precision::PApproximation();
            //    double TolV = Precision::PApproximation();
            double Tol = 0.0001;
            double TolU = Math.Pow(Tol, 2.0 / 3);
            double TolV = Math.Pow(Tol, 2.0 / 3);
            ProjLib_CompProjectedCurve Proj = new(HS, HC, TolU, TolV, -1.0);

            double f, l;
            Proj.Bounds(1, out f, out l);
            Adaptor2d_Curve2d HC2d = Proj.Trim(f, l, TolU);
            Approx_CurveOnSurface Approx = new(HC2d, HS, f, l, Tol);
            Approx.Perform(16, 14, C2, true);

            // ici, on a toujours un type BSpline.
            if (Approx.IsDone() && Approx.HasResult())
            {
                // The approximation is discarded: its curve is returned unfrozen (decision 244).
                GC = Approx.InternalCurve3d();
            }
        }

        return GC;
    }

    /// <summary>
    /// The 3D curve of the projection of <paramref name="Curve"/> on <paramref name="Plane"/> along
    /// <paramref name="Dir"/>. If <paramref name="KeepParametrization"/> is true, the projected curve keeps the
    /// curve's parametrization: proj(C(u)) = PC(u) for each u; otherwise it may change. Returns null if the
    /// projection has failed.
    /// </summary>
    public static Geom_Curve? ProjectOnPlane(Geom_Curve Curve, Geom_Plane Plane, in Direction3D Dir, bool KeepParametrization)
    {
        // OCCT copies AC into a handle: AC itself is used.
        GeomAdaptor_Curve HC = new(Curve);

        ProjLib_ProjectOnPlane Proj = new(Plane.Position(), Dir);
        Proj.Load(HC, global::Kerf.Engine.Foundation.Precision.Approximation(), KeepParametrization);

        Geom_Curve? GC = null;

        switch (Proj.GetType())
        {
            case Line:
                GC = new Geom_Line(Proj.Line());
                break;

            case Circle:
                GC = new Geom_Circle(Proj.Circle());
                break;

            case Ellipse:
                GC = new Geom_Ellipse(Proj.Ellipse());
                break;

            case Parabola:
                GC = new Geom_Parabola(Proj.Parabola());
                break;

            case Hyperbola:
                GC = new Geom_Hyperbola(Proj.Hyperbola());
                break;

            case BezierCurve:
                // The projector is discarded: its curve is returned unfrozen (decision 244).
                GC = Proj.InternalBezier();
                break;

            case BSplineCurve:
                GC = Proj.InternalBSpline();
                break;
            default:
                return GC;
        }

        if (Curve is Geom_TrimmedCurve)
        {
            GC = new Geom_TrimmedCurve(GC, Proj.FirstParameter(), Proj.LastParameter());
        }

        return GC;
    }
}
