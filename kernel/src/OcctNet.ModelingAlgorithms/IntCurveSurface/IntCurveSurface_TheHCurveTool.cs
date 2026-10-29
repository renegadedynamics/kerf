// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheHCurveTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheHCurveTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// Static access to a curve adaptor's services, as IntCurveSurface_HInter calls them, plus sampling for its
/// polygons.
/// </summary>
/// <remarks>
/// OCCT's static class is a <c>readonly struct</c> with static members, so that it can implement the tool
/// interfaces of the generic algorithms (<see cref="IIntCurveSurface_CurveTool{TheCurve}"/>; decision 202's
/// approach). The deprecated <c>SamplePars</c> overload with an output handle is left out.
/// </remarks>
public readonly struct IntCurveSurface_TheHCurveTool : IIntCurveSurface_CurveTool<Adaptor3d_Curve>
{
    private const int myMinPnts = 5;

    public static double FirstParameter(in Adaptor3d_Curve C) => C.FirstParameter();

    public static double LastParameter(in Adaptor3d_Curve C) => C.LastParameter();

    public static GeomAbs_Shape Continuity(in Adaptor3d_Curve C) => C.Continuity();

    /// <summary>The number of intervals for continuity <paramref name="S"/>; may be one when the curve is <paramref name="S"/>.</summary>
    public static int NbIntervals(in Adaptor3d_Curve C, GeomAbs_Shape S) => C.NbIntervals(S);

    /// <summary>
    /// Stores in <paramref name="T"/> the parameters bounding the intervals of continuity <paramref name="S"/>; its
    /// length must be at least <see cref="NbIntervals"/> + 1.
    /// </summary>
    public static void Intervals(in Adaptor3d_Curve C, NCollection_Array1<double> T, GeomAbs_Shape S) =>
        C.Intervals(T, S);

    public static bool IsClosed(in Adaptor3d_Curve C) => C.IsClosed();

    public static bool IsPeriodic(in Adaptor3d_Curve C) => C.IsPeriodic();

    public static double Period(in Adaptor3d_Curve C) => C.Period();

    public static gp_Pnt Value(in Adaptor3d_Curve C, double U) => C.Value(U);

    public static void D0(in Adaptor3d_Curve C, double U, out gp_Pnt P) => C.D0(U, out P);

    public static void D1(in Adaptor3d_Curve C, double U, out gp_Pnt P, out gp_Vec V) => C.D1(U, out P, out V);

    public static void D2(in Adaptor3d_Curve C, double U, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2) =>
        C.D2(U, out P, out V1, out V2);

    public static void D3(in Adaptor3d_Curve C, double U, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2, out gp_Vec V3) =>
        C.D3(U, out P, out V1, out V2, out V3);

    public static gp_Vec DN(in Adaptor3d_Curve C, double U, int N) => C.DN(U, N);

    /// <summary>The parametric resolution corresponding to the real space resolution <paramref name="R3d"/>.</summary>
    public static double Resolution(in Adaptor3d_Curve C, double R3d) => C.Resolution(R3d);

    public static GeomAbs_CurveType GetType(in Adaptor3d_Curve C) => C.GetType();

    public static gp_Lin Line(in Adaptor3d_Curve C) => C.Line();

    public static gp_Circ Circle(in Adaptor3d_Curve C) => C.Circle();

    public static gp_Elips Ellipse(in Adaptor3d_Curve C) => C.Ellipse();

    public static gp_Hypr Hyperbola(in Adaptor3d_Curve C) => C.Hyperbola();

    public static gp_Parab Parabola(in Adaptor3d_Curve C) => C.Parabola();

    public static Geom_BezierCurve Bezier(in Adaptor3d_Curve C) => C.Bezier();

    public static Geom_BSplineCurve BSpline(in Adaptor3d_Curve C) => C.BSpline();

    /// <summary>The number of samples of the polygon of the curve between <paramref name="U0"/> and <paramref name="U1"/>.</summary>
    public static int NbSamples(in Adaptor3d_Curve C, double U0, double U1)
    {
        GeomAbs_CurveType typC = C.GetType();
        const double nbsOther = 10.0;
        double nbs = nbsOther;

        if (typC == GeomAbs_Line)
        {
            nbs = 2;
        }
        else if (typC == GeomAbs_BezierCurve)
        {
            nbs = 3 + C.NbPoles();
        }
        else if (typC == GeomAbs_BSplineCurve)
        {
            nbs = C.NbKnots();
            nbs *= C.Degree();
            nbs *= C.LastParameter() - C.FirstParameter();
            double aRange = U1 - U0;
            if (Math.Abs(aRange) > Precision.PConfusion())
            {
                nbs /= aRange;
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
    /// The sampling parameters of the curve between <paramref name="U0"/> and <paramref name="U1"/>: uniform for
    /// curves other than B-splines; for B-splines, the knots subdivided by the degree, then thinned to the points
    /// where the polygon deviates from the curve by more than <paramref name="Defl"/>.
    /// </summary>
    public static NCollection_HArray1<double> SamplePars(in Adaptor3d_Curve C, double U0, double U1, double Defl,
                                                         int NbMin)
    {
        NCollection_HArray1<double> Pars;
        GeomAbs_CurveType typC = C.GetType();
        const double nbsOther = 10.0;
        double nbs = nbsOther;

        if (typC == GeomAbs_Line)
        {
            nbs = 2;
        }
        else if (typC == GeomAbs_BezierCurve)
        {
            nbs = 3 + C.NbPoles();
        }

        if (typC != GeomAbs_BSplineCurve)
        {
            if (nbs > 50)
            {
                nbs = 50;
            }
            int nnbs = (int)nbs;

            Pars = new NCollection_HArray1<double>(1, nnbs);
            double du = (U1 - U0) / (nnbs - 1);

            Pars.SetValue(1, U0);
            Pars.SetValue(nnbs, U1);
            int i;
            double u;
            for (i = 2, u = U0 + du; i < nnbs; ++i, u += du)
            {
                Pars.SetValue(i, u);
            }
            return Pars;
        }

        Geom_BSplineCurve aBC = C.BSpline();

        int j, k, nbi;
        double t1, t2, dt;
        int ui1 = aBC.FirstUKnotIndex();
        int ui2 = aBC.LastUKnotIndex();

        for (int i = ui1; i < ui2; ++i)
        {
            if (U0 >= aBC.Knot(i) && U0 < aBC.Knot(i + 1))
            {
                ui1 = i;
                break;
            }
        }

        for (int i = ui2; i > ui1; --i)
        {
            if (U1 <= aBC.Knot(i) && U1 > aBC.Knot(i - 1))
            {
                ui2 = i;
                break;
            }
        }

        int nbsu = ui2 - ui1 + 1;
        nbsu += (nbsu - 1) * (aBC.Degree() - 1);
        bool bUniform = false;
        if (nbsu < NbMin)
        {
            nbsu = NbMin;
            bUniform = true;
        }

        NCollection_Array1<double> aPars = new(1, nbsu);
        NCollection_Array1<bool> aFlg = new(1, nbsu);
        // Filling of sample parameters
        if (bUniform)
        {
            t1 = U0;
            t2 = U1;
            dt = (t2 - t1) / (nbsu - 1);
            aPars[1] = t1;
            aFlg[1] = false;
            aPars[nbsu] = t2;
            aFlg[nbsu] = false;
            int i;
            for (i = 2, t1 += dt; i < nbsu; ++i, t1 += dt)
            {
                aPars[i] = t1;
                aFlg[i] = false;
            }
        }
        else
        {
            nbi = aBC.Degree();
            k = 0;
            t1 = U0;
            for (int i = ui1 + 1; i <= ui2; ++i)
            {
                if (i == ui2)
                {
                    t2 = U1;
                }
                else
                {
                    t2 = aBC.Knot(i);
                }
                dt = (t2 - t1) / nbi;
                j = 1;
                do
                {
                    ++k;
                    aPars[k] = t1;
                    aFlg[k] = false;
                    t1 += dt;
                } while (++j <= nbi);
                t1 = t2;
            }
            ++k;
            aPars[k] = t1;
        }

        // Analysis of deflection

        double aDefl2 = Max(Defl * Defl, 1.0e-9);
        double tol = Max(0.01 * aDefl2, 1.0e-9);
        int l;

        int NbSamples = 2;
        aFlg[1] = true;
        aFlg[nbsu] = true;
        j = 1;
        bool bCont = true;
        while (j < nbsu - 1 && bCont)
        {
            if (aFlg[j + 1])
            {
                ++j;
                continue;
            }

            t2 = aPars[j];
            gp_Pnt p1 = aBC.Value(t2);
            for (k = j + 2; k <= nbsu; ++k)
            {
                t2 = aPars[k];
                gp_Pnt p2 = aBC.Value(t2);

                if (p1.SquareDistance(p2) <= tol)
                {
                    continue;
                }

                gce_MakeLin MkLin = new(p1, p2);
                gp_Lin lin = MkLin.Value();
                bool ok = true;
                for (l = j + 1; l < k; ++l)
                {
                    if (aFlg[l])
                    {
                        ok = false;
                        break;
                    }

                    gp_Pnt pp = aBC.Value(aPars[l]);
                    double d = lin.SquareDistance(pp);

                    if (d <= aDefl2)
                    {
                        continue;
                    }

                    ok = false;
                    break;
                }

                if (!ok)
                {
                    j = k - 1;
                    aFlg[j] = true;
                    ++NbSamples;
                    break;
                }

                if (aFlg[k])
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

        if (NbSamples < myMinPnts)
        {
            // uniform distribution
            NbSamples = myMinPnts;
            Pars = new NCollection_HArray1<double>(1, NbSamples);
            t1 = U0;
            t2 = U1;
            dt = (t2 - t1) / (NbSamples - 1);
            Pars.SetValue(1, t1);
            Pars.SetValue(NbSamples, t2);
            int i;
            for (i = 2, t1 += dt; i < NbSamples; ++i, t1 += dt)
            {
                Pars.SetValue(i, t1);
            }
            return Pars;
        }

        Pars = new NCollection_HArray1<double>(1, NbSamples);
        j = 0;
        for (int i = 1; i <= nbsu; ++i)
        {
            if (aFlg[i])
            {
                ++j;
                Pars.SetValue(j, aPars[i]);
            }
        }
        return Pars;
    }
}
