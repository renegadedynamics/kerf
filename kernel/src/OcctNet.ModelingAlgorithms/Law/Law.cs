// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Builds 1D B-spline laws for evolution functions: blends for the boundaries of constrained fillings, scaling
/// laws and reparametrizations by arc length.
/// </summary>
/// <remarks>The B-splines returned are new and unfrozen; <see cref="MixBnd(Law_Linear)"/>'s law freezes its curve.</remarks>
public static class Law
{
    /// <summary>
    /// A cubic B-spline law on the bounds of the linear law Lin, interpolating it at the Schoenberg points with
    /// its first two and last two poles equal (zero derivatives at the ends).
    /// </summary>
    public static Law_BSpFunc MixBnd(Law_Linear Lin)
    {
        Lin.Bounds(out double f, out double l);
        NCollection_Array1<double> Knots = new NCollection_Array1<double>(1, 4);
        NCollection_Array1<int> Mults = new NCollection_Array1<int>(1, 4);
        Knots[1] = f;
        Knots[4] = l;
        Knots[2] = 0.75 * f + 0.25 * l;
        Knots[3] = 0.25 * f + 0.75 * l;
        Mults[1] = Mults[4] = 4;
        Mults[2] = Mults[3] = 1;
        NCollection_HArray1<double> pol = Law.MixBnd(3, Knots, Mults, Lin);
        Law_BSpline bs = new Law_BSpline(pol.Array1(), Knots, Mults, 3);
        Law_BSpFunc bsf = new Law_BSpFunc();
        bsf.SetCurve(bs);
        return bsf;
    }

    /// <summary>
    /// The poles of the B-spline of the given degree, knots and multiplicities interpolating Lin at the Schoenberg
    /// points, with the first two and last two poles equal (when there are at least four).
    /// </summary>
    public static NCollection_HArray1<double> MixBnd(int Degree, NCollection_Array1<double> Knots,
                                                     NCollection_Array1<int> Mults, Law_Linear Lin)
    {
        int nbpol = 0, nbfk = 0, i, j, k = 0;
        for (i = Mults.Lower(); i <= Mults.Upper(); i++)
        {
            nbfk += Mults[i];
        }
        NCollection_Array1<double> fk = new NCollection_Array1<double>(1, nbfk);
        for (i = Mults.Lower(); i <= Mults.Upper(); i++)
        {
            for (j = 1; j <= Mults[i]; j++)
            {
                fk[++k] = Knots[i];
            }
        }
        nbpol = nbfk - Degree - 1;
        NCollection_Array1<double> par = new NCollection_Array1<double>(1, nbpol);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(Degree, fk, par);
        NCollection_HArray1<double> res = new NCollection_HArray1<double>(1, nbpol);
        NCollection_Array1<double> pol = res.ChangeArray1();
        for (i = 1; i <= nbpol; i++)
        {
            pol[i] = Lin.Value(par[i]);
        }
        NCollection_Array1<int> ord = new NCollection_Array1<int>(1, nbpol);
        ord.Init(0);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(Degree, fk, par, ord, 1, pol.AsSpan(1), out i);
        if (nbpol >= 4)
        {
            pol[2] = pol[1];
            pol[nbpol - 1] = pol[nbpol];
        }
        return res;
    }

    private static double eval1(double p, double first, double last, double piv, bool nulr)
    {
        if ((nulr && p >= piv) || (!nulr && p <= piv))
        {
            return 0.0;
        }
        else if (nulr)
        {
            double a = piv - first;
            a *= a;
            a = 1.0 / a;
            double b = p - first;
            a *= b;
            b = piv - p;
            a *= b;
            a *= b;
            return a;
        }
        else
        {
            double a = last - piv;
            a *= a;
            a = 1.0 / a;
            double b = last - p;
            a *= b;
            b = p - piv;
            a *= b;
            a *= b;
            return a;
        }
    }

    /// <summary>
    /// The poles of the B-spline of the given degree, knots and multiplicities interpolating a tangency blend that
    /// is zero on one side of the knot Index (the right if NulOnTheRight).
    /// </summary>
    public static NCollection_HArray1<double> MixTgt(int Degree, NCollection_Array1<double> Knots,
                                                     NCollection_Array1<int> Mults, bool NulOnTheRight, int Index)
    {
        double first = Knots[Knots.Lower()];
        double last = Knots[Knots.Upper()];
        double piv = Knots[Index];
        int nbpol = 0, nbfk = 0, i, j, k = 0;
        for (i = Mults.Lower(); i <= Mults.Upper(); i++)
        {
            nbfk += Mults[i];
        }
        NCollection_Array1<double> fk = new NCollection_Array1<double>(1, nbfk);
        for (i = Mults.Lower(); i <= Mults.Upper(); i++)
        {
            for (j = 1; j <= Mults[i]; j++)
            {
                fk[++k] = Knots[i];
            }
        }
        nbpol = nbfk - Degree - 1;
        NCollection_Array1<double> par = new NCollection_Array1<double>(1, nbpol);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(Degree, fk, par);
        NCollection_HArray1<double> res = new NCollection_HArray1<double>(1, nbpol);
        NCollection_Array1<double> pol = res.ChangeArray1();
        for (i = 1; i <= nbpol; i++)
        {
            pol[i] = eval1(par[i], first, last, piv, NulOnTheRight);
        }
        NCollection_Array1<int> ord = new NCollection_Array1<int>(1, nbpol);
        ord.Init(0);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(Degree, fk, par, ord, 1, pol.AsSpan(1), out i);
        return res;
    }

    /// <summary>
    /// A law reparametrizing Curve by approximate arc length: the law maps [First, Last] onto the curve's
    /// parameters (reversed if Rev), interpolating NbPoints points, with the derivatives DFirst and DLast at the
    /// ends if HasDF and HasDL.
    /// </summary>
    /// <exception cref="FailureException">The interpolation failed.</exception>
    public static Law_BSpline Reparametrize(Adaptor3d_Curve Curve, double First, double Last, bool HasDF, bool HasDL,
                                            double DFirst, double DLast, bool Rev, int NbPoints)
    {
        // On evalue la longeur approximative de la courbe.

        int i;
        double DDFirst = DFirst, DDLast = DLast;
        if (HasDF && Rev)
        {
            DDFirst = -DFirst;
        }
        if (HasDL && Rev)
        {
            DDLast = -DLast;
        }
        NCollection_Array1<double> cumdist = new NCollection_Array1<double>(1, 2 * NbPoints);
        NCollection_Array1<double> ucourbe = new NCollection_Array1<double>(1, 2 * NbPoints);
        Point3D P1, P2;
        double U1 = Curve.FirstParameter();
        double U2 = Curve.LastParameter();
        double U, DU, Length = 0.0;
        if (!Rev)
        {
            P1 = Curve.Value(U1);
            U = U1;
            DU = (U2 - U1) / (2 * NbPoints - 1);
        }
        else
        {
            P1 = Curve.Value(U2);
            U = U2;
            DU = (U1 - U2) / (2 * NbPoints - 1);
        }
        for (i = 1; i <= 2 * NbPoints; i++)
        {
            P2 = Curve.Value(U);
            Length += P2.Distance(P1);
            cumdist[i] = Length;
            ucourbe[i] = U;
            U += DU;
            P1 = P2;
        }
        if (Rev)
        {
            ucourbe[2 * NbPoints] = U1;
        }
        else
        {
            ucourbe[2 * NbPoints] = U2;
        }

        NCollection_HArray1<double> point = new NCollection_HArray1<double>(1, NbPoints);
        NCollection_HArray1<double> param = new NCollection_HArray1<double>(1, NbPoints);

        double DCorde = Length / (NbPoints - 1);
        double Corde = DCorde;
        int Index = 1;
        double Alpha;
        double fac = 1.0 / (NbPoints - 1);

        point.SetValue(1, ucourbe[1]);
        param.SetValue(1, First);
        point.SetValue(NbPoints, ucourbe[2 * NbPoints]);
        param.SetValue(NbPoints, Last);

        for (i = 2; i < NbPoints; i++)
        {
            while (cumdist[Index] < Corde)
            {
                Index++;
            }

            Alpha = (Corde - cumdist[Index - 1]) / (cumdist[Index] - cumdist[Index - 1]);
            U = ucourbe[Index - 1] + Alpha * (ucourbe[Index] - ucourbe[Index - 1]);
            point.SetValue(i, U);
            param.SetValue(i, ((NbPoints - i) * First + (i - 1) * Last) * fac);
            Corde = i * DCorde;
        }
        Law_Interpolate inter = new Law_Interpolate(point, param, false, 1.0e-9);
        if (HasDF || HasDL)
        {
            NCollection_Array1<double> tgt = new NCollection_Array1<double>(1, NbPoints);
            NCollection_HArray1<bool> flag = new NCollection_HArray1<bool>(1, NbPoints);
            flag.ChangeArray1().Init(false);
            if (HasDF)
            {
                flag.SetValue(1, true);
                tgt.SetValue(1, DDFirst);
            }
            if (HasDL)
            {
                flag.SetValue(NbPoints, true);
                tgt.SetValue(NbPoints, DDLast);
            }
            inter.Load(tgt, flag);
        }
        inter.Perform();
        if (!inter.IsDone())
        {
            throw new FailureException("Law::Reparametrize echec interpolation");
        }
        Law_BSpline bs = inter.InternalCurve();
        return bs;
    }

    private static double eval2(double p, double first, double last, double mil, bool hasfirst, bool haslast,
                                Law_BSpline? bs1, Law_BSpline? bs2)
    {
        if (hasfirst && p < mil)
        {
            return bs1!.Value(p);
        }
        else if (haslast && p > mil)
        {
            return bs2!.Value(p);
        }
        else
        {
            return 1.0;
        }
    }

    /// <summary>
    /// A C1 cubic scaling law on [First, Last], equal to 1 in the middle, going to VFirst at First if HasF and to
    /// VLast at Last if HasL, with zero derivatives at the ends.
    /// </summary>
    public static Law_BSpline Scale(double First, double Last, bool HasF, bool HasL, double VFirst, double VLast)
    {
        int i;
        double Milieu = 0.5 * (First + Last);
        NCollection_Array1<double> knot = new NCollection_Array1<double>(1, 3);
        NCollection_Array1<double> fknot = new NCollection_Array1<double>(1, 10);
        NCollection_Array1<int> mult = new NCollection_Array1<int>(1, 3);
        knot[1] = First;
        knot[2] = Milieu;
        knot[3] = Last;
        fknot[1] = fknot[2] = fknot[3] = fknot[4] = First;
        fknot[10] = fknot[9] = fknot[8] = fknot[7] = Last;
        fknot[5] = fknot[6] = Milieu;
        mult[1] = 4;
        mult[3] = 4;
        mult[2] = 2;

        NCollection_Array1<double> pbs = new NCollection_Array1<double>(1, 4);
        NCollection_Array1<double> kbs = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int> mbs = new NCollection_Array1<int>(1, 2);
        mbs[1] = mbs[2] = 4;
        Law_BSpline? bs1 = null, bs2 = null;
        if (HasF)
        {
            pbs[1] = pbs[2] = VFirst;
            pbs[3] = pbs[4] = 1.0;
            kbs[1] = First;
            kbs[2] = Milieu;
            bs1 = new Law_BSpline(pbs, kbs, mbs, 3);
        }
        if (HasL)
        {
            pbs[1] = pbs[2] = 1.0;
            pbs[3] = pbs[4] = VLast;
            kbs[1] = Milieu;
            kbs[2] = Last;
            bs2 = new Law_BSpline(pbs, kbs, mbs, 3);
        }

        NCollection_Array1<double> pol = new NCollection_Array1<double>(1, 6);
        NCollection_Array1<double> par = new NCollection_Array1<double>(1, 6);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(3, fknot, par);
        for (i = 1; i <= 6; i++)
        {
            pol[i] = eval2(par[i], First, Last, Milieu, HasF, HasL, bs1, bs2);
        }
        NCollection_Array1<int> ord = new NCollection_Array1<int>(1, 6);
        ord.Init(0);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(3, fknot, par, ord, 1, pol.AsSpan(1), out i);
        bs1 = new Law_BSpline(pol, knot, mult, 3);
        return bs1;
    }

    /// <summary>
    /// A cubic scaling law on [First, Last] with poles VFirst (or 1), VFirst, 1, VLast, VLast (or 1): its ends are
    /// VFirst if HasF and VLast if HasL, with zero derivatives there.
    /// </summary>
    public static Law_BSpline ScaleCub(double First, double Last, bool HasF, bool HasL, double VFirst, double VLast)
    {
        double Milieu = 0.5 * (First + Last);
        NCollection_Array1<double> pol = new NCollection_Array1<double>(1, 5);
        NCollection_Array1<double> knot = new NCollection_Array1<double>(1, 3);
        NCollection_Array1<int> mult = new NCollection_Array1<int>(1, 3);
        knot[1] = First;
        knot[2] = Milieu;
        knot[3] = Last;
        mult[1] = 4;
        mult[3] = 4;
        mult[2] = 1;

        Law_BSpline bs;
        if (HasF)
        {
            pol[1] = pol[2] = VFirst;
        }
        else
        {
            pol[1] = pol[2] = 1.0;
        }
        if (HasL)
        {
            pol[4] = pol[5] = VLast;
        }
        else
        {
            pol[4] = pol[5] = 1.0;
        }

        pol[3] = 1.0;

        bs = new Law_BSpline(pol, knot, mult, 3);
        return bs;
    }
}
