// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheHCurveTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheHCurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// Static access to a curve adaptor's services, as IntCurveSurface_HInter calls them, plus sampling for its
/// polygons.
/// </summary>
/// <remarks>
/// OCCT's static class is a <c>readonly struct</c> with static members, so that it can implement the tool
/// interfaces of the generic algorithms (<see cref="IIntersectionCurveTool{TheCurve}"/>; decision 202's
/// approach). The deprecated <c>SamplePars</c> overload with an output handle is left out.
/// </remarks>
internal readonly struct TheHCurveTool : IIntersectionCurveTool<CurveAdaptorBase>
{
    private const int MinPnts = 5;

    public static double FirstParameter(in CurveAdaptorBase c) => c.FirstParameter();

    public static double LastParameter(in CurveAdaptorBase c) => c.LastParameter();

    public static Continuity Continuity(in CurveAdaptorBase c) => c.Continuity();

    /// <summary>The number of intervals for continuity <paramref name="s"/>; may be one when the curve is <paramref name="s"/>.</summary>
    public static int NbIntervals(in CurveAdaptorBase c, Continuity s) => c.NbIntervals(s);

    /// <summary>
    /// Stores in <paramref name="t"/> the parameters bounding the intervals of continuity <paramref name="s"/>; its
    /// length must be at least <see cref="NbIntervals"/> + 1.
    /// </summary>
    public static void Intervals(in CurveAdaptorBase c, NCollection_Array1<double> t, Continuity s) =>
        c.Intervals(t, s);

    public static bool IsClosed(in CurveAdaptorBase c) => c.IsClosed;

    public static bool IsPeriodic(in CurveAdaptorBase c) => c.IsPeriodic;

    public static double Period(in CurveAdaptorBase c) => c.Period();

    public static Point3D Value(in CurveAdaptorBase c, double u) => c.Value(u);

    public static void D0(in CurveAdaptorBase c, double u, out Point3D p) => c.D0(u, out p);

    public static void D1(in CurveAdaptorBase c, double u, out Point3D p, out Vector3D v) => c.D1(u, out p, out v);

    public static void D2(in CurveAdaptorBase c, double u, out Point3D p, out Vector3D v1, out Vector3D v2) =>
        c.D2(u, out p, out v1, out v2);

    public static void D3(in CurveAdaptorBase c, double u, out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3) =>
        c.D3(u, out p, out v1, out v2, out v3);

    public static Vector3D DN(in CurveAdaptorBase c, double u, int n) => c.DN(u, n);

    /// <summary>The parametric resolution corresponding to the real space resolution <paramref name="r3d"/>.</summary>
    public static double Resolution(in CurveAdaptorBase c, double r3d) => c.Resolution(r3d);

    public static CurveKind GetType(in CurveAdaptorBase c) => c.GetType();

    public static Line3D Line(in CurveAdaptorBase c) => c.Line();

    public static Circle3D Circle(in CurveAdaptorBase c) => c.Circle();

    public static Ellipse3D Ellipse(in CurveAdaptorBase c) => c.Ellipse();

    public static Hyperbola3D Hyperbola(in CurveAdaptorBase c) => c.Hyperbola();

    public static Parabola3D Parabola(in CurveAdaptorBase c) => c.Parabola();

    public static BezierCurve3D Bezier(in CurveAdaptorBase c) => c.Bezier();

    public static BSplineCurve3D BSpline(in CurveAdaptorBase c) => c.BSpline();

    /// <summary>The number of samples of the polygon of the curve between <paramref name="u0"/> and <paramref name="u1"/>.</summary>
    public static int NbSamples(in CurveAdaptorBase c, double u0, double u1)
    {
        CurveKind typC = c.GetType();
        const double nbsOther = 10.0;
        double nbs = nbsOther;

        if (typC == global::Kerf.Geometry.CurveKind.Line)
        {
            nbs = 2;
        }
        else if (typC == BezierCurve)
        {
            nbs = 3 + c.PoleCount;
        }
        else if (typC == BSplineCurve)
        {
            nbs = c.KnotCount;
            nbs *= c.Degree();
            nbs *= c.LastParameter() - c.FirstParameter();
            double range = u1 - u0;
            if (Math.Abs(range) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                nbs /= range;
            }
            if (nbs < 2.0)
            {
                nbs = 2;
            }
        }
        if (nbs > 50)
        {
            nbs = 50;
        }
        return ((int)nbs);
    }

    /// <summary>
    /// The sampling parameters of the curve between <paramref name="u0"/> and <paramref name="u1"/>: uniform for
    /// curves other than B-splines; for B-splines, the knots subdivided by the degree, then thinned to the points
    /// where the polygon deviates from the curve by more than <paramref name="defl"/>.
    /// </summary>
    public static NCollection_HArray1<double> SamplePars(in CurveAdaptorBase c, double u0, double u1, double defl,
                                                         int nbMin)
    {
        NCollection_HArray1<double> Pars;
        CurveKind typC = c.GetType();
        const double nbsOther = 10.0;
        double nbs = nbsOther;

        if (typC == global::Kerf.Geometry.CurveKind.Line)
        {
            nbs = 2;
        }
        else if (typC == BezierCurve)
        {
            nbs = 3 + c.PoleCount;
        }

        if (typC != BSplineCurve)
        {
            if (nbs > 50)
            {
                nbs = 50;
            }
            int nnbs = (int)nbs;

            Pars = new NCollection_HArray1<double>(1, nnbs);
            double du = (u1 - u0) / (nnbs - 1);

            Pars.SetValue(1, u0);
            Pars.SetValue(nnbs, u1);
            int i;
            double u;
            for (i = 2, u = u0 + du; i < nnbs; ++i, u += du)
            {
                Pars.SetValue(i, u);
            }
            return Pars;
        }

        BSplineCurve3D bc = c.BSpline();

        int j, k, nbi;
        double t1, t2, dt;
        int ui1 = bc.FirstUKnotIndex();
        int ui2 = bc.LastUKnotIndex();

        for (int i = ui1; i < ui2; ++i)
        {
            if (u0 >= bc.Knot(i) && u0 < bc.Knot(i + 1))
            {
                ui1 = i;
                break;
            }
        }

        for (int i = ui2; i > ui1; --i)
        {
            if (u1 <= bc.Knot(i) && u1 > bc.Knot(i - 1))
            {
                ui2 = i;
                break;
            }
        }

        int nbsu = ui2 - ui1 + 1;
        nbsu += (nbsu - 1) * (bc.Degree - 1);
        bool bUniform = false;
        if (nbsu < nbMin)
        {
            nbsu = nbMin;
            bUniform = true;
        }

        NCollection_Array1<double> pars = new(1, nbsu);
        NCollection_Array1<bool> flg = new(1, nbsu);
        // Filling of sample parameters
        if (bUniform)
        {
            t1 = u0;
            t2 = u1;
            dt = (t2 - t1) / (nbsu - 1);
            pars[1] = t1;
            flg[1] = false;
            pars[nbsu] = t2;
            flg[nbsu] = false;
            int i;
            for (i = 2, t1 += dt; i < nbsu; ++i, t1 += dt)
            {
                pars[i] = t1;
                flg[i] = false;
            }
        }
        else
        {
            nbi = bc.Degree;
            k = 0;
            t1 = u0;
            for (int i = ui1 + 1; i <= ui2; ++i)
            {
                if (i == ui2)
                {
                    t2 = u1;
                }
                else
                {
                    t2 = bc.Knot(i);
                }
                dt = (t2 - t1) / nbi;
                j = 1;
                do
                {
                    ++k;
                    pars[k] = t1;
                    flg[k] = false;
                    t1 += dt;
                } while (++j <= nbi);
                t1 = t2;
            }
            ++k;
            pars[k] = t1;
        }

        // Analysis of deflection

        double defl2 = Max(defl * defl, 1.0e-9);
        double tol = Max(0.01 * defl2, 1.0e-9);
        int l;

        int nbSamples = 2;
        flg[1] = true;
        flg[nbsu] = true;
        j = 1;
        bool bCont = true;
        while (j < nbsu - 1 && bCont)
        {
            if (flg[j + 1])
            {
                ++j;
                continue;
            }

            t2 = pars[j];
            Point3D p1 = bc.Value(t2);
            for (k = j + 2; k <= nbsu; ++k)
            {
                t2 = pars[k];
                Point3D p2 = bc.Value(t2);

                if (p1.SquareDistanceTo(p2) <= tol)
                {
                    continue;
                }

                MakeLin mkLin = new(p1, p2);
                Line3D lin = mkLin.Value();
                bool ok = true;
                for (l = j + 1; l < k; ++l)
                {
                    if (flg[l])
                    {
                        ok = false;
                        break;
                    }

                    Point3D pp = bc.Value(pars[l]);
                    double d = lin.SquareDistanceTo(pp);

                    if (d <= defl2)
                    {
                        continue;
                    }

                    ok = false;
                    break;
                }

                if (!ok)
                {
                    j = k - 1;
                    flg[j] = true;
                    ++nbSamples;
                    break;
                }

                if (flg[k])
                {
                    j = k;
                    break;
                }
            }

            if (k >= nbsu)
            {
                bCont = false;
            }
        }

        if (nbSamples < MinPnts)
        {
            // uniform distribution
            nbSamples = MinPnts;
            Pars = new NCollection_HArray1<double>(1, nbSamples);
            t1 = u0;
            t2 = u1;
            dt = (t2 - t1) / (nbSamples - 1);
            Pars.SetValue(1, t1);
            Pars.SetValue(nbSamples, t2);
            int i;
            for (i = 2, t1 += dt; i < nbSamples; ++i, t1 += dt)
            {
                Pars.SetValue(i, t1);
            }
            return Pars;
        }

        Pars = new NCollection_HArray1<double>(1, nbSamples);
        j = 0;
        for (int i = 1; i <= nbsu; ++i)
        {
            if (flg[i])
            {
                ++j;
                Pars.SetValue(j, pars[i]);
            }
        }
        return Pars;
    }
}
