// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Laws;

/// <summary>
/// Builds 1D B-spline laws for evolution functions: blends for the boundaries of constrained fillings, scaling
/// laws and reparametrizations by arc length.
/// </summary>
/// <remarks>The B-splines returned are new and unfrozen; <see cref="MixBnd(Linear)"/>'s law freezes its curve.</remarks>
internal static class Law
{
    /// <summary>
    /// A cubic B-spline law on the bounds of the linear law Lin, interpolating it at the Schoenberg points with
    /// its first two and last two poles equal (zero derivatives at the ends).
    /// </summary>
    public static BSpFunc MixBnd(Linear lin)
    {
        lin.Bounds(out double f, out double l);
        NCollection_Array1<double> knots = new NCollection_Array1<double>(1, 4);
        NCollection_Array1<int> mults = new NCollection_Array1<int>(1, 4);
        knots[1] = f;
        knots[4] = l;
        knots[2] = 0.75 * f + 0.25 * l;
        knots[3] = 0.25 * f + 0.75 * l;
        mults[1] = mults[4] = 4;
        mults[2] = mults[3] = 1;
        NCollection_HArray1<double> pol = global::Kerf.Engine.Construction.Laws.Law.MixBnd(3, knots, mults, lin);
        BSpline bs = new BSpline(pol.Array1(), knots, mults, 3);
        BSpFunc bsf = new BSpFunc();
        bsf.SetCurve(bs);
        return bsf;
    }

    /// <summary>
    /// The poles of the B-spline of the given degree, knots and multiplicities interpolating Lin at the Schoenberg
    /// points, with the first two and last two poles equal (when there are at least four).
    /// </summary>
    public static NCollection_HArray1<double> MixBnd(int degree, NCollection_Array1<double> knots,
                                                     NCollection_Array1<int> mults, Linear lin)
    {
        int nbpol = 0, nbfk = 0, i, j, k = 0;
        for (i = mults.Lower(); i <= mults.Upper(); i++)
        {
            nbfk += mults[i];
        }
        NCollection_Array1<double> fk = new NCollection_Array1<double>(1, nbfk);
        for (i = mults.Lower(); i <= mults.Upper(); i++)
        {
            for (j = 1; j <= mults[i]; j++)
            {
                fk[++k] = knots[i];
            }
        }
        nbpol = nbfk - degree - 1;
        NCollection_Array1<double> par = new NCollection_Array1<double>(1, nbpol);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(degree, fk, par);
        NCollection_HArray1<double> res = new NCollection_HArray1<double>(1, nbpol);
        NCollection_Array1<double> pol = res.ChangeArray1();
        for (i = 1; i <= nbpol; i++)
        {
            pol[i] = lin.Value(par[i]);
        }
        NCollection_Array1<int> ord = new NCollection_Array1<int>(1, nbpol);
        ord.Init(0);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree, fk, par, ord, 1, pol.AsSpan(1), out i);
        if (nbpol >= 4)
        {
            pol[2] = pol[1];
            pol[nbpol - 1] = pol[nbpol];
        }
        return res;
    }

    private static double Eval1(double p, double first, double last, double piv, bool nulr)
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
    public static NCollection_HArray1<double> MixTgt(int degree, NCollection_Array1<double> knots,
                                                     NCollection_Array1<int> mults, bool nulOnTheRight, int index)
    {
        double first = knots[knots.Lower()];
        double last = knots[knots.Upper()];
        double piv = knots[index];
        int nbpol = 0, nbfk = 0, i, j, k = 0;
        for (i = mults.Lower(); i <= mults.Upper(); i++)
        {
            nbfk += mults[i];
        }
        NCollection_Array1<double> fk = new NCollection_Array1<double>(1, nbfk);
        for (i = mults.Lower(); i <= mults.Upper(); i++)
        {
            for (j = 1; j <= mults[i]; j++)
            {
                fk[++k] = knots[i];
            }
        }
        nbpol = nbfk - degree - 1;
        NCollection_Array1<double> par = new NCollection_Array1<double>(1, nbpol);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(degree, fk, par);
        NCollection_HArray1<double> res = new NCollection_HArray1<double>(1, nbpol);
        NCollection_Array1<double> pol = res.ChangeArray1();
        for (i = 1; i <= nbpol; i++)
        {
            pol[i] = Eval1(par[i], first, last, piv, nulOnTheRight);
        }
        NCollection_Array1<int> ord = new NCollection_Array1<int>(1, nbpol);
        ord.Init(0);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree, fk, par, ord, 1, pol.AsSpan(1), out i);
        return res;
    }

    /// <summary>
    /// A law reparametrizing Curve by approximate arc length: the law maps [First, Last] onto the curve's
    /// parameters (reversed if Rev), interpolating NbPoints points, with the derivatives DFirst and DLast at the
    /// ends if HasDF and HasDL.
    /// </summary>
    /// <exception cref="FailureException">The interpolation failed.</exception>
    public static BSpline Reparametrize(CurveAdaptorBase curve, double first, double last, bool hasDF, bool hasDL,
                                            double dFirst, double dLast, bool rev, int nbPoints)
    {
        // On evalue la longeur approximative de la courbe.

        int i;
        double ddFirst = dFirst, ddLast = dLast;
        if (hasDF && rev)
        {
            ddFirst = -dFirst;
        }
        if (hasDL && rev)
        {
            ddLast = -dLast;
        }
        NCollection_Array1<double> cumdist = new NCollection_Array1<double>(1, 2 * nbPoints);
        NCollection_Array1<double> ucourbe = new NCollection_Array1<double>(1, 2 * nbPoints);
        Point3D p1, p2;
        double u1 = curve.FirstParameter();
        double u2 = curve.LastParameter();
        double u, du, length = 0.0;
        if (!rev)
        {
            p1 = curve.Value(u1);
            u = u1;
            du = (u2 - u1) / (2 * nbPoints - 1);
        }
        else
        {
            p1 = curve.Value(u2);
            u = u2;
            du = (u1 - u2) / (2 * nbPoints - 1);
        }
        for (i = 1; i <= 2 * nbPoints; i++)
        {
            p2 = curve.Value(u);
            length += p2.DistanceTo(p1);
            cumdist[i] = length;
            ucourbe[i] = u;
            u += du;
            p1 = p2;
        }
        if (rev)
        {
            ucourbe[2 * nbPoints] = u1;
        }
        else
        {
            ucourbe[2 * nbPoints] = u2;
        }

        NCollection_HArray1<double> point = new NCollection_HArray1<double>(1, nbPoints);
        NCollection_HArray1<double> param = new NCollection_HArray1<double>(1, nbPoints);

        double dCorde = length / (nbPoints - 1);
        double corde = dCorde;
        int index = 1;
        double alpha;
        double fac = 1.0 / (nbPoints - 1);

        point.SetValue(1, ucourbe[1]);
        param.SetValue(1, first);
        point.SetValue(nbPoints, ucourbe[2 * nbPoints]);
        param.SetValue(nbPoints, last);

        for (i = 2; i < nbPoints; i++)
        {
            while (cumdist[index] < corde)
            {
                index++;
            }

            alpha = (corde - cumdist[index - 1]) / (cumdist[index] - cumdist[index - 1]);
            u = ucourbe[index - 1] + alpha * (ucourbe[index] - ucourbe[index - 1]);
            point.SetValue(i, u);
            param.SetValue(i, ((nbPoints - i) * first + (i - 1) * last) * fac);
            corde = i * dCorde;
        }
        LawsInterpolate inter = new LawsInterpolate(point, param, false, 1.0e-9);
        if (hasDF || hasDL)
        {
            NCollection_Array1<double> tgt = new NCollection_Array1<double>(1, nbPoints);
            NCollection_HArray1<bool> flag = new NCollection_HArray1<bool>(1, nbPoints);
            flag.ChangeArray1().Init(false);
            if (hasDF)
            {
                flag.SetValue(1, true);
                tgt.SetValue(1, ddFirst);
            }
            if (hasDL)
            {
                flag.SetValue(nbPoints, true);
                tgt.SetValue(nbPoints, ddLast);
            }
            inter.Load(tgt, flag);
        }
        inter.Perform();
        if (!inter.IsDone)
        {
            throw new FailureException("Law::Reparametrize echec interpolation");
        }
        BSpline bs = inter.InternalCurve();
        return bs;
    }

    private static double Eval2(double p, double first, double last, double mil, bool hasfirst, bool haslast,
                                BSpline? bs1, BSpline? bs2)
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
    public static BSpline Scale(double first, double last, bool hasF, bool hasL, double vFirst, double vLast)
    {
        int i;
        double milieu = 0.5 * (first + last);
        NCollection_Array1<double> knot = new NCollection_Array1<double>(1, 3);
        NCollection_Array1<double> fknot = new NCollection_Array1<double>(1, 10);
        NCollection_Array1<int> mult = new NCollection_Array1<int>(1, 3);
        knot[1] = first;
        knot[2] = milieu;
        knot[3] = last;
        fknot[1] = fknot[2] = fknot[3] = fknot[4] = first;
        fknot[10] = fknot[9] = fknot[8] = fknot[7] = last;
        fknot[5] = fknot[6] = milieu;
        mult[1] = 4;
        mult[3] = 4;
        mult[2] = 2;

        NCollection_Array1<double> pbs = new NCollection_Array1<double>(1, 4);
        NCollection_Array1<double> kbs = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int> mbs = new NCollection_Array1<int>(1, 2);
        mbs[1] = mbs[2] = 4;
        BSpline? bs1 = null, bs2 = null;
        if (hasF)
        {
            pbs[1] = pbs[2] = vFirst;
            pbs[3] = pbs[4] = 1.0;
            kbs[1] = first;
            kbs[2] = milieu;
            bs1 = new BSpline(pbs, kbs, mbs, 3);
        }
        if (hasL)
        {
            pbs[1] = pbs[2] = 1.0;
            pbs[3] = pbs[4] = vLast;
            kbs[1] = milieu;
            kbs[2] = last;
            bs2 = new BSpline(pbs, kbs, mbs, 3);
        }

        NCollection_Array1<double> pol = new NCollection_Array1<double>(1, 6);
        NCollection_Array1<double> par = new NCollection_Array1<double>(1, 6);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildSchoenbergPoints(3, fknot, par);
        for (i = 1; i <= 6; i++)
        {
            pol[i] = Eval2(par[i], first, last, milieu, hasF, hasL, bs1, bs2);
        }
        NCollection_Array1<int> ord = new NCollection_Array1<int>(1, 6);
        ord.Init(0);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(3, fknot, par, ord, 1, pol.AsSpan(1), out i);
        bs1 = new BSpline(pol, knot, mult, 3);
        return bs1;
    }

    /// <summary>
    /// A cubic scaling law on [First, Last] with poles VFirst (or 1), VFirst, 1, VLast, VLast (or 1): its ends are
    /// VFirst if HasF and VLast if HasL, with zero derivatives there.
    /// </summary>
    public static BSpline ScaleCub(double first, double last, bool hasF, bool hasL, double vFirst, double vLast)
    {
        double milieu = 0.5 * (first + last);
        NCollection_Array1<double> pol = new NCollection_Array1<double>(1, 5);
        NCollection_Array1<double> knot = new NCollection_Array1<double>(1, 3);
        NCollection_Array1<int> mult = new NCollection_Array1<int>(1, 3);
        knot[1] = first;
        knot[2] = milieu;
        knot[3] = last;
        mult[1] = 4;
        mult[3] = 4;
        mult[2] = 1;

        BSpline bs;
        if (hasF)
        {
            pol[1] = pol[2] = vFirst;
        }
        else
        {
            pol[1] = pol[2] = 1.0;
        }
        if (hasL)
        {
            pol[4] = pol[5] = vLast;
        }
        else
        {
            pol[4] = pol[5] = 1.0;
        }

        pol[3] = 1.0;

        bs = new BSpline(pol, knot, mult, 3);
        return bs;
    }
}
