// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_ConstrainedFilling.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_ConstrainedFilling.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A B-spline surface filling three or four boundaries, which serve as path constraints and, when they are
/// <see cref="GeomFill_BoundWithSurf"/>s, as tangency constraints (the Coons style only). Designed for fillets.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21): <c>Init</c>, <see cref="SetDomain"/> and <see cref="ReBuild"/> recompute the
/// surface. As in OCCT, <c>Init</c> sorts the boundaries it is given, reparametrises each of them on [0, 1] (they are
/// working objects, changed in place) and keeps them; the corner states and tangency fields of a previous
/// <c>Init</c> are not reset (occt-issues.md).
/// </para>
/// <para>
/// <see cref="Surface"/> hands out the surface frozen; ported callers use <see cref="InternalSurface"/>. The
/// <c>Check*</c> diagnostics write to a TextWriter (the console by default), as OCCT writes to std::cout.
/// </para>
/// </remarks>
public class GeomFill_ConstrainedFilling
{
    private readonly int degmax;
    private readonly int segmax;
    private GeomFill_CoonsAlgPatch? ptch;
    private readonly GeomFill_TgtField?[] tgalg = new GeomFill_TgtField?[4];
    private readonly double[] mig = new double[4];
    private readonly GeomFill_CornerState[] stcor =
        [new GeomFill_CornerState(), new GeomFill_CornerState(), new GeomFill_CornerState(), new GeomFill_CornerState()];
    private readonly gp_Vec[] v = [new gp_Vec(), new gp_Vec(), new gp_Vec(), new gp_Vec()];
    private bool appdone;
    private readonly int[] degree = new int[2];
    private readonly NCollection_HArray1<gp_Pnt>?[] curvpol = new NCollection_HArray1<gp_Pnt>?[4];
    private readonly NCollection_HArray1<gp_Pnt>?[] tgtepol = new NCollection_HArray1<gp_Pnt>?[4];
    private readonly NCollection_HArray1<int>?[] mults = new NCollection_HArray1<int>?[2];
    private readonly NCollection_HArray1<double>?[] knots = new NCollection_HArray1<double>?[2];
    private readonly NCollection_HArray1<double>?[] ab = new NCollection_HArray1<double>?[4];
    private readonly NCollection_HArray1<double>?[] pq = new NCollection_HArray1<double>?[4];
    private readonly double[] dom = new double[4];
    private readonly NCollection_HArray1<gp_Pnt>?[] ncpol = new NCollection_HArray1<gp_Pnt>?[4];
    private readonly NCollection_HArray1<gp_Pnt>?[] ntpol = new NCollection_HArray1<gp_Pnt>?[4];
    private readonly NCollection_HArray1<int>?[] nm = new NCollection_HArray1<int>?[2];
    private readonly NCollection_HArray1<double>?[] nk = new NCollection_HArray1<double>?[2];
    private readonly int[] ibound = new int[2];
    private readonly int[] ctr = new int[2];
    private int nbd3;
    private NCollection_HArray2<gp_Pnt>? S0;
    private NCollection_HArray2<gp_Pnt>? S1;
    private Geom_BSplineSurface? surf;

    private static int inqadd(double d1, double d2, Span<double> k, Span<int> m, int deg, double tolk)
    {
        int nbadd = 0;
        m[0] = m[1] = deg - 2;
        if (d1 != 1.0 && d2 != 1.0)
        {
            if (Math.Abs(d1 + d2 - 1.0) < tolk)
            {
                k[0] = 0.5 * (d1 + 1.0 - d2);
                nbadd = 1;
            }
            else
            {
                nbadd = 2;
                k[0] = Min(d1, 1.0 - d2);
                k[1] = Max(d1, 1.0 - d2);
            }
        }
        else if (d1 != 1.0)
        {
            k[0] = d1;
            nbadd = 1;
        }
        else if (d2 != 1.0)
        {
            k[0] = d2;
            nbadd = 1;
        }
        return nbadd;
    }

    private static Law_Linear mklin(Law_Function func)
    {
        Law_Linear? fu = func as Law_Linear;
        if (fu is null)
        {
            fu = new Law_Linear();
            double d, f;
            func.Bounds(out d, out f);
            fu.Set(d, func.Value(d), f, func.Value(f));
        }
        return fu;
    }

    private static void sortbounds(int nb, GeomFill_Boundary[] bound, bool[] rev, GeomFill_CornerState[] stat)
    {
        // trier les bords (facon bourinos),
        // flaguer ceux a renverser,
        // flaguer les baillements au coins.
        int i, j;
        GeomFill_Boundary temp;
        rev[0] = false;
        gp_Pnt pf, pl;
        gp_Pnt qf, ql;
        for (i = 0; i < nb - 1; i++)
        {
            if (!rev[i])
            {
                bound[i].Points(out pf, out pl);
            }
            else
            {
                bound[i].Points(out pl, out pf);
            }
            for (j = i + 1; j <= nb - 1; j++)
            {
                bound[j].Points(out qf, out ql);
                //  Modified by skv - Fri Jun 18 12:52:54 2004 OCC6129 Begin
                double df = qf.Distance(pl);
                double dl = ql.Distance(pl);
                if (df < dl)
                {
                    if (df < stat[i + 1].Gap())
                    {
                        temp = bound[i + 1];
                        bound[i + 1] = bound[j];
                        bound[j] = temp;
                        stat[i + 1].Gap(df);
                        rev[i + 1] = false;
                    }
                }
                else
                {
                    if (dl < stat[i + 1].Gap())
                    {
                        temp = bound[i + 1];
                        bound[i + 1] = bound[j];
                        bound[j] = temp;
                        stat[i + 1].Gap(dl);
                        rev[i + 1] = true;
                    }
                }
                //  Modified by skv - Fri Jun 18 12:52:54 2004 OCC6129 End
            }
        }
        if (!rev[nb - 1])
        {
            bound[nb - 1].Points(out pf, out pl);
        }
        else
        {
            bound[nb - 1].Points(out pl, out pf);
        }
        bound[0].Points(out qf, out ql);
        stat[0].Gap(pl.Distance(qf));

        // flaguer les angles entre tangentes au coins et entre les normales au
        // coins pour les bords contraints.
        gp_Pnt pbid;
        gp_Vec tgi, nori, tgn, norn;
        double fi, fn, li, ln;
        for (i = 0; i < nb; i++)
        {
            int next = (i + 1) % nb;
            if (!rev[i])
            {
                bound[i].Bounds(out fi, out li);
            }
            else
            {
                bound[i].Bounds(out li, out fi);
            }
            bound[i].D1(li, out pbid, out tgi);
            if (rev[i])
            {
                tgi.Reverse();
            }
            if (!rev[next])
            {
                bound[next].Bounds(out fn, out ln);
            }
            else
            {
                bound[next].Bounds(out ln, out fn);
            }
            bound[next].D1(fn, out pbid, out tgn);
            if (rev[next])
            {
                tgn.Reverse();
            }
            double ang = M_PI - tgi.Angle(tgn);
            stat[next].TgtAng(ang);
            if (bound[i].HasNormals() && bound[next].HasNormals())
            {
                stat[next].Constraint();
                nori = bound[i].Norm(li);
                norn = bound[next].Norm(fn);
                ang = nori.Angle(norn);
                stat[next].NorAng(ang);
            }
        }
    }

    private static void coonscnd(int nb, GeomFill_Boundary[] bound, bool[] rev, GeomFill_CornerState[] stat,
                                 GeomFill_TgtField?[] tga, double[] mintg)
    {
        double fact_normalization = 100.0;
        int i;
        // Pour chaque coin contraint, on controle les bounds adjascents.
        for (i = 0; i < nb; i++)
        {
            if (stat[i].HasConstraint())
            {
                int ip = (i - 1 + nb) % nb;
                double tolang = Min(bound[ip].Tolang(), bound[i].Tolang());
                double an = stat[i].NorAng();
                bool twist = false;
                if (an >= 0.5 * M_PI)
                {
                    twist = true;
                    an = M_PI - an;
                }
                if (an > tolang)
                {
                    stat[i].DoKill(0.0);
                }
                else
                {
                    double fact = 0.5 * 27.0 / 4;
                    tolang *= (Min(mintg[ip], mintg[i]) * fact * fact_normalization);
                    gp_Vec tgp, dnorp, tgi, dnori, vbid;
                    gp_Pnt pbid;
                    double fp, lp, fi, li;
                    if (!rev[ip])
                    {
                        bound[ip].Bounds(out fp, out lp);
                    }
                    else
                    {
                        bound[ip].Bounds(out lp, out fp);
                    }
                    bound[ip].D1(lp, out pbid, out tgp);
                    bound[ip].D1Norm(lp, out vbid, out dnorp);
                    if (!rev[i])
                    {
                        bound[i].Bounds(out fi, out li);
                    }
                    else
                    {
                        bound[i].Bounds(out li, out fi);
                    }
                    bound[i].D1(fi, out pbid, out tgi);
                    bound[i].D1Norm(fi, out vbid, out dnori);
                    double scal1 = tgp.Dot(dnori);
                    double scal2 = tgi.Dot(dnorp);
                    if (!twist)
                    {
                        scal2 *= -1.0;
                    }
                    scal1 = Math.Abs(scal1 + scal2);
                    if (scal1 > tolang)
                    {
                        double killfactor = tolang / scal1;
                        stat[i].DoKill(killfactor);
                    }
                }
            }
        }
    }

    private static void killcorners(int nb, GeomFill_Boundary[] bound, bool[] rev, bool[] nrev,
                                    GeomFill_CornerState[] stat, GeomFill_TgtField?[] tga)
    {
        int i;
        // Pour chaque  bound, on  controle l etat  des extremites  et on flingue
        // eventuellement le champ tangent et les derivees du bound.
        for (i = 0; i < nb; i++)
        {
            int inext = (i + 1) % nb;
            bool fnul, lnul;
            double fscal, lscal;
            if (!nrev[i])
            {
                fnul = stat[i].IsToKill(out fscal);
                lnul = stat[inext].IsToKill(out lscal);
            }
            else
            {
                lnul = stat[i].IsToKill(out lscal);
                fnul = stat[inext].IsToKill(out fscal);
            }
            if (fnul || lnul)
            {
                bound[i].Reparametrize(0.0, 1.0, fnul, lnul, fscal, lscal, rev[i]);
                if (bound[i].HasNormals() && tga[i]!.IsScalable())
                {
                    Law_BSpline bs = Law.ScaleCub(0.0, 1.0, fnul, lnul, fscal, lscal);
                    tga[i]!.Scale(bs);
                }
            }
        }
    }

    // The evaluator of the curve approximation (OCCT's GeomFill_ConstrainedFilling_Eval).
    private sealed class GeomFill_ConstrainedFilling_Eval(GeomFill_ConstrainedFilling theTool) : AdvApprox_EvaluatorFunction
    {
        private readonly GeomFill_ConstrainedFilling curfil = theTool;

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Parameter,
                                      int DerivativeRequest, Span<double> Result, out int ErrorCode)
        {
            ErrorCode = curfil.Eval(Parameter, DerivativeRequest, Result);
        }
    }

    /// <summary>
    /// An empty framework: the surface will have a degree of at most <paramref name="MaxDeg"/> and at most
    /// <paramref name="MaxSeg"/> segments.
    /// </summary>
    public GeomFill_ConstrainedFilling(int MaxDeg, int MaxSeg)
    {
        degmax = MaxDeg;
        segmax = MaxSeg;
        appdone = false;
        nbd3 = 0;
        dom[0] = dom[1] = dom[2] = dom[3] = 1.0;
    }

    /// <summary>
    /// Fills three boundaries (the fourth side degenerates to a point). With <paramref name="NoCheck"/> the Coons
    /// compatibility conditions at the corners are not checked.
    /// </summary>
    public void Init(GeomFill_Boundary B1, GeomFill_Boundary B2, GeomFill_Boundary B3, bool NoCheck = false)
    {
        bool[] rev = new bool[3];
        rev[0] = rev[1] = rev[2] = false;
        GeomFill_Boundary[] bound = new GeomFill_Boundary[3];
        bound[0] = B1;
        bound[1] = B2;
        bound[2] = B3;
        int i;
        sortbounds(3, bound, rev, stcor);

        // on reoriente.
        rev[2] = !rev[2];

        // on reparamettre tout le monde entre 0. et 1.
        for (i = 0; i <= 2; i++)
        {
            bound[i].Reparametrize(0.0, 1.0, false, false, 1.0, 1.0, rev[i]);
        }

        // On cree le carreau algorithmique (u,(1-u)) et les champs tangents
        // 1er jus.
        // On cree donc le bord manquant.
        gp_Pnt p1 = bound[1].Value(1.0);
        gp_Pnt p2 = bound[2].Value(1.0);
        gp_Pnt ppp = new(0.5 * (p1.XYZ() + p2.XYZ()));
        double t3 = Max(bound[1].Tol3d(), bound[2].Tol3d());
        GeomFill_DegeneratedBound DB = new(ppp, 0.0, 1.0, t3, 10.0);

        ptch = new GeomFill_CoonsAlgPatch(bound[0], bound[1], DB, bound[2]);

        GeomFill_TgtField?[] ttgalg = new GeomFill_TgtField?[3];
        if (bound[0].HasNormals())
        {
            ttgalg[0] = tgalg[0] = new GeomFill_TgtOnCoons(ptch, 0);
        }
        if (bound[1].HasNormals())
        {
            ttgalg[1] = tgalg[1] = new GeomFill_TgtOnCoons(ptch, 1);
        }
        if (bound[2].HasNormals())
        {
            ttgalg[2] = tgalg[3] = new GeomFill_TgtOnCoons(ptch, 3);
        }

        for (i = 0; i <= 3; i++)
        {
            mig[i] = 1.0;
            if (tgalg[i] is not null)
            {
                MinTgte(i);
            }
        }

        if (!NoCheck)
        {
            // On  verifie enfin les conditions  de compatibilites sur les derivees
            // aux coins maintenant qu on a quelque chose a quoi les comparer.
            bool[] nrev = new bool[3];
            nrev[0] = nrev[1] = false;
            nrev[2] = true;
            mig[2] = mig[3];
            coonscnd(3, bound, nrev, stcor, ttgalg, mig);
            killcorners(3, bound, rev, nrev, stcor, ttgalg);
        }
        // on remet les coins en place (on duplique la pointe).
        stcor[3] = stcor[2];

        for (i = 0; i <= 3; i++)
        {
            mig[i] = 1.0;
            if (tgalg[i] is not null)
            {
                if (!CheckTgte(i))
                {
                    Law_Function fu1, fu2;
                    ptch.Func(out fu1, out fu2);
                    Law_Function ffu1 = Law.MixBnd((fu1 as Law_Linear)!);
                    Law_Function ffu2 = Law.MixBnd((fu2 as Law_Linear)!);
                    ptch.SetFunc(ffu1, ffu2);
                    break;
                }
            }
        }

        Build();
    }

    /// <summary>
    /// Fills four boundaries, given in any order: they are sorted and, if need be, reversed and reparametrised. With
    /// <paramref name="NoCheck"/> the Coons compatibility conditions at the corners are not checked.
    /// </summary>
    public void Init(GeomFill_Boundary B1, GeomFill_Boundary B2, GeomFill_Boundary B3, GeomFill_Boundary B4,
                     bool NoCheck = false)
    {
        bool[] rev = new bool[4];
        rev[0] = rev[1] = rev[2] = rev[3] = false;
        GeomFill_Boundary[] bound = new GeomFill_Boundary[4];
        bound[0] = B1;
        bound[1] = B2;
        bound[2] = B3;
        bound[3] = B4;
        int i;
        sortbounds(4, bound, rev, stcor);

        // on reoriente.
        rev[2] = !rev[2];
        rev[3] = !rev[3];

        // on reparamettre tout le monde entre 0. et 1.
        for (i = 0; i <= 3; i++)
        {
            bound[i].Reparametrize(0.0, 1.0, false, false, 1.0, 1.0, rev[i]);
        }

        // On cree le carreau algorithmique (u,(1-u)) et les champs tangents
        // 1er jus.
        ptch = new GeomFill_CoonsAlgPatch(bound[0], bound[1], bound[2], bound[3]);
        for (i = 0; i <= 3; i++)
        {
            if (bound[i].HasNormals())
            {
                tgalg[i] = new GeomFill_TgtOnCoons(ptch, i);
            }
        }
        // on calcule le min de chacun des champs tangents pour l evaluation
        // des tolerances.
        for (i = 0; i <= 3; i++)
        {
            mig[i] = 1.0;
            if (tgalg[i] is not null)
            {
                MinTgte(i);
            }
        }

        if (!NoCheck)
        {
            // On  verifie enfin les conditions  de compatibilites sur les derivees
            // aux coins maintenant qu on a quelque chose a quoi les comparer.
            bool[] nrev = new bool[4];
            nrev[0] = nrev[1] = false;
            nrev[2] = nrev[3] = true;
            coonscnd(4, bound, nrev, stcor, tgalg, mig);
            killcorners(4, bound, rev, nrev, stcor, tgalg);
        }
        // On verifie les champs tangents ne changent pas de direction.
        for (i = 0; i <= 3; i++)
        {
            mig[i] = 1.0;
            if (tgalg[i] is not null)
            {
                if (!CheckTgte(i))
                {
                    Law_Function fu1, fu2;
                    ptch.Func(out fu1, out fu2);
                    Law_Function ffu1 = Law.MixBnd((fu1 as Law_Linear)!);
                    Law_Function ffu2 = Law.MixBnd((fu2 as Law_Linear)!);
                    ptch.SetFunc(ffu1, ffu2);
                    break;
                }
            }
        }

        Build();
    }

    /// <summary>
    /// Sets the relative part <paramref name="l"/> (at most 1, the default) of the parametric range of the
    /// constrained boundary <paramref name="B"/> on which its blending function propagates its tangency field.
    /// Call it after <c>Init</c>, then <see cref="ReBuild"/>.
    /// </summary>
    public void SetDomain(double l, GeomFill_BoundWithSurf B)
    {
        if (ReferenceEquals(B, ptch!.Bound(0)))
        {
            dom[0] = Min(1.0, Math.Abs(l));
        }
        else if (ReferenceEquals(B, ptch.Bound(1)))
        {
            dom[1] = Min(1.0, Math.Abs(l));
        }
        else if (ReferenceEquals(B, ptch.Bound(2)))
        {
            dom[2] = Min(1.0, Math.Abs(l));
        }
        else if (ReferenceEquals(B, ptch.Bound(3)))
        {
            dom[3] = Min(1.0, Math.Abs(l));
        }
    }

    /// <summary>Recomputes the poles with the blending functions set by <see cref="SetDomain"/>.</summary>
    public void ReBuild()
    {
        if (!appdone)
        {
            throw new Standard_Failure("GeomFill_ConstrainedFilling::ReBuild Approx non faite");
        }
        MatchKnots();
        PerformS0();
        PerformS1();
        PerformSurface();
    }

    /// <summary>The boundary <paramref name="I"/> (from 0) after sorting.</summary>
    public GeomFill_Boundary Boundary(int I) => ptch!.Bound(I);

    /// <summary>The surface (null before <c>Init</c>), frozen.</summary>
    public Geom_BSplineSurface? Surface()
    {
        surf?.Freeze();
        return surf;
    }

    /// <summary>The surface, not frozen, for ported callers.</summary>
    internal Geom_BSplineSurface? InternalSurface() => surf;

    private void Build()
    {
        for (int count = 0; count < 2; count++)
        {
            ibound[0] = count;
            ibound[1] = count + 2;
            ctr[0] = ctr[1] = nbd3 = 0;
            int ii;
            for (ii = 0; ii < 2; ii++)
            {
                if (ptch!.Bound(ibound[ii]).HasNormals())
                {
                    ctr[ii] = 2;
                }
                else if (!ptch.Bound(ibound[ii]).IsDegenerated())
                {
                    ctr[ii] = 1;
                }
                nbd3 += ctr[ii];
            }
            if (nbd3 != 0)
            {
                PerformApprox();
            }
        }
        appdone = true;
        MatchKnots();
        PerformS0();
        PerformS1();
        PerformSurface();
    }

    private void PerformApprox()
    {
        int ii;
        NCollection_HArray1<double>? tol3d = null, tol2d = null, tol1d = null;
        if (nbd3 != 0)
        {
            tol3d = new NCollection_HArray1<double>(1, nbd3);
        }
        int i3d = 0;
        for (ii = 0; ii <= 1; ii++)
        {
            if (ctr[ii] != 0)
            {
                tol3d!.SetValue((++i3d), ptch!.Bound(ibound[ii]).Tol3d());
            }
            if (ctr[ii] == 2)
            {
                tol3d!.SetValue(++i3d, 0.5 * mig[ibound[ii]] * ptch!.Bound(ibound[ii]).Tolang());
            }
        }
        double f, l;
        ptch!.Bound(ibound[0]).Bounds(out f, out l);

        GeomFill_ConstrainedFilling_Eval ev = new(this);
        AdvApprox_ApproxAFunction app = new(0, 0, nbd3, tol1d, tol2d, tol3d, f, l, GeomAbs_C1, degmax, segmax, ev);

        if (app.IsDone() || app.HasResult())
        {
            int imk = Min(ibound[0], ibound[1]);
            int nbpol = app.NbPoles();
            degree[imk] = app.Degree();
            mults[imk] = app.Multiplicities();
            knots[imk] = app.Knots();
            i3d = 0;
            for (ii = 0; ii <= 1; ii++)
            {
                curvpol[ibound[ii]] = new NCollection_HArray1<gp_Pnt>(1, nbpol);
                NCollection_Array1<gp_Pnt> cp = curvpol[ibound[ii]]!.ChangeArray1();
                if (ctr[ii] != 0)
                {
                    app.Poles(++i3d, cp);
                }
                else
                {
                    gp_Pnt ppp = ptch.Bound(ibound[ii]).Value(0.5 * (f + l));
                    for (int ij = 1; ij <= nbpol; ij++)
                    {
                        cp[ij] = ppp;
                    }
                }
                if (ctr[ii] == 2)
                {
                    tgtepol[ibound[ii]] = new NCollection_HArray1<gp_Pnt>(1, nbpol);
                    app.Poles(++i3d, tgtepol[ibound[ii]]!.ChangeArray1());
                }
            }
        }
    }

    // OCCT's NCollection_Array1 over the first nbadd values of a C array (kadd, madd).
    private static NCollection_Array1<T> FirstValues<T>(ReadOnlySpan<T> theValues, int theCount)
    {
        NCollection_Array1<T> anArray = new(1, theCount);
        for (int i = 1; i <= theCount; i++)
        {
            anArray[i] = theValues[i - 1];
        }
        return anArray;
    }

    private void MatchKnots()
    {
        // on n insere rien si les domaines valent 1.
        int i, j, l;
        int[] ind = new int[4];
        nm[0] = mults[0];
        nm[1] = mults[1];
        nk[0] = knots[0];
        nk[1] = knots[1];
        ind[0] = nk[1]!.Length();
        ind[2] = 1;
        ind[1] = 1;
        ind[3] = nk[0]!.Length();
        ncpol[0] = curvpol[0];
        ncpol[1] = curvpol[1];
        ncpol[2] = curvpol[2];
        ncpol[3] = curvpol[3];
        ntpol[0] = tgtepol[0];
        ntpol[1] = tgtepol[1];
        ntpol[2] = tgtepol[2];
        ntpol[3] = tgtepol[3];
        Span<double> kadd = stackalloc double[2];
        Span<int> madd = stackalloc int[2];
        double tolk = 1.0 / Max(10, 2 * knots[1]!.Array1().Length());
        int nbadd = inqadd(dom[0], dom[2], kadd, madd, degree[1], tolk);
        if (nbadd != 0)
        {
            NCollection_Array1<double> addk = FirstValues<double>(kadd, nbadd);
            NCollection_Array1<int> addm = FirstValues<int>(madd, nbadd);
            int nbnp, nbnk;
            if (BSplCLib.PrepareInsertKnots(degree[1], false, knots[1]!.Array1(), mults[1]!.Array1(), addk, addm,
                                            out nbnp, out nbnk, tolk, false))
            {
                nm[1] = new NCollection_HArray1<int>(1, nbnk);
                nk[1] = new NCollection_HArray1<double>(1, nbnk);
                ncpol[1] = new NCollection_HArray1<gp_Pnt>(1, nbnp);
                ncpol[3] = new NCollection_HArray1<gp_Pnt>(1, nbnp);
                BSplCLib.InsertKnots(degree[1], false, curvpol[1]!.Array1(), BSplCLib.NoWeights(), knots[1]!.Array1(),
                                     mults[1]!.Array1(), addk, addm, ncpol[1]!.ChangeArray1(), BSplCLib.NoWeights(),
                                     nk[1]!.ChangeArray1(), nm[1]!.ChangeArray1(), tolk, false);

                BSplCLib.InsertKnots(degree[1], false, curvpol[3]!.Array1(), BSplCLib.NoWeights(), knots[1]!.Array1(),
                                     mults[1]!.Array1(), addk, addm, ncpol[3]!.ChangeArray1(), BSplCLib.NoWeights(),
                                     nk[1]!.ChangeArray1(), nm[1]!.ChangeArray1(), tolk, false);
                if (tgtepol[1] is not null)
                {
                    ntpol[1] = new NCollection_HArray1<gp_Pnt>(1, nbnp);
                    BSplCLib.InsertKnots(degree[1], false, tgtepol[1]!.Array1(), BSplCLib.NoWeights(),
                                         knots[1]!.Array1(), mults[1]!.Array1(), addk, addm, ntpol[1]!.ChangeArray1(),
                                         BSplCLib.NoWeights(), nk[1]!.ChangeArray1(), nm[1]!.ChangeArray1(), tolk, false);
                }
                if (tgtepol[3] is not null)
                {
                    ntpol[3] = new NCollection_HArray1<gp_Pnt>(1, nbnp);
                    BSplCLib.InsertKnots(degree[1], false, tgtepol[3]!.Array1(), BSplCLib.NoWeights(),
                                         knots[1]!.Array1(), mults[1]!.Array1(), addk, addm, ntpol[3]!.ChangeArray1(),
                                         BSplCLib.NoWeights(), nk[1]!.ChangeArray1(), nm[1]!.ChangeArray1(), tolk, false);
                }
            }
            if (dom[0] != 1.0)
            {
                for (i = 2; i <= nbnk; i++)
                {
                    // OCCT: compares the domain with the multiplicities, not the knots (occt-issues.md)
                    if (Math.Abs(dom[0] - nm[1]!.Value(i)) < tolk)
                    {
                        ind[0] = i;
                        break;
                    }
                }
            }
            if (dom[2] != 1.0)
            {
                for (i = 1; i < nbnk; i++)
                {
                    // OCCT: the multiplicities, not the knots (occt-issues.md)
                    if (Math.Abs(1.0 - dom[2] - nm[1]!.Value(i)) < tolk)
                    {
                        ind[2] = i;
                        break;
                    }
                }
            }
        }
        tolk = 1.0 / Max(10.0, 2.0 * knots[0]!.Array1().Length());
        nbadd = inqadd(dom[1], dom[3], kadd, madd, degree[0], tolk);
        if (nbadd != 0)
        {
            NCollection_Array1<double> addk = FirstValues<double>(kadd, nbadd);
            NCollection_Array1<int> addm = FirstValues<int>(madd, nbadd);
            int nbnp, nbnk;
            if (BSplCLib.PrepareInsertKnots(degree[0], false, knots[0]!.Array1(), mults[0]!.Array1(), addk, addm,
                                            out nbnp, out nbnk, tolk, false))
            {
                nm[0] = new NCollection_HArray1<int>(1, nbnk);
                nk[0] = new NCollection_HArray1<double>(1, nbnk);
                ncpol[0] = new NCollection_HArray1<gp_Pnt>(1, nbnp);
                ncpol[2] = new NCollection_HArray1<gp_Pnt>(1, nbnp);
                BSplCLib.InsertKnots(degree[0], false, curvpol[0]!.Array1(), BSplCLib.NoWeights(), knots[0]!.Array1(),
                                     mults[0]!.Array1(), addk, addm, ncpol[0]!.ChangeArray1(), BSplCLib.NoWeights(),
                                     nk[0]!.ChangeArray1(), nm[0]!.ChangeArray1(), tolk, false);

                BSplCLib.InsertKnots(degree[0], false, curvpol[2]!.Array1(), BSplCLib.NoWeights(), knots[0]!.Array1(),
                                     mults[0]!.Array1(), addk, addm, ncpol[2]!.ChangeArray1(), BSplCLib.NoWeights(),
                                     nk[0]!.ChangeArray1(), nm[0]!.ChangeArray1(), tolk, false);
                if (tgtepol[0] is not null)
                {
                    ntpol[0] = new NCollection_HArray1<gp_Pnt>(1, nbnp);
                    BSplCLib.InsertKnots(degree[0], false, tgtepol[0]!.Array1(), BSplCLib.NoWeights(),
                                         knots[0]!.Array1(), mults[0]!.Array1(), addk, addm, ntpol[0]!.ChangeArray1(),
                                         BSplCLib.NoWeights(), nk[0]!.ChangeArray1(), nm[0]!.ChangeArray1(), tolk, false);
                }
                if (tgtepol[2] is not null)
                {
                    ntpol[2] = new NCollection_HArray1<gp_Pnt>(1, nbnp);
                    BSplCLib.InsertKnots(degree[0], false, tgtepol[2]!.Array1(), BSplCLib.NoWeights(),
                                         knots[0]!.Array1(), mults[0]!.Array1(), addk, addm, ntpol[2]!.ChangeArray1(),
                                         BSplCLib.NoWeights(), nk[0]!.ChangeArray1(), nm[0]!.ChangeArray1(), tolk, false);
                }
            }
            if (dom[1] != 1.0)
            {
                for (i = 2; i <= nbnk; i++)
                {
                    // OCCT: the multiplicities, not the knots (occt-issues.md)
                    if (Math.Abs(dom[1] - nm[0]!.Value(i)) < tolk)
                    {
                        ind[1] = i;
                        break;
                    }
                }
            }
            if (dom[3] != 1.0)
            {
                for (i = 1; i < nbnk; i++)
                {
                    // OCCT: the multiplicities, not the knots (occt-issues.md)
                    if (Math.Abs(1.0 - dom[3] - nm[0]!.Value(i)) < tolk)
                    {
                        ind[3] = i;
                        break;
                    }
                }
            }
        }
        Law_Linear fu = mklin(ptch!.Func(0));
        ab[0] = Law.MixBnd(degree[1], nk[1]!.Array1(), nm[1]!.Array1(), fu);
        fu = mklin(ptch.Func(1));
        ab[1] = Law.MixBnd(degree[0], nk[0]!.Array1(), nm[0]!.Array1(), fu);

        for (i = 0; i < 2; i++)
        {
            l = ab[i]!.Length();
            ab[i + 2] = new NCollection_HArray1<double>(1, l);
            for (j = 1; j <= l; j++)
            {
                ab[i + 2]!.SetValue(j, 1.0 - ab[i]!.Value(j));
            }
        }
        pq[0] = Law.MixTgt(degree[1], nk[1]!.Array1(), nm[1]!.Array1(), true, ind[0]);
        pq[2] = Law.MixTgt(degree[1], nk[1]!.Array1(), nm[1]!.Array1(), false, ind[2]);

        pq[1] = Law.MixTgt(degree[0], nk[0]!.Array1(), nm[0]!.Array1(), false, ind[1]);
        pq[3] = Law.MixTgt(degree[0], nk[0]!.Array1(), nm[0]!.Array1(), true, ind[3]);
    }

    private void PerformS0()
    {
        // On construit les poles de S0 par combinaison des poles des bords,
        // des poles des fonctions ab, des points c selon la formule :
        // S0(i,j) = ab[0](j)*ncpol[0](i) + ab[1](i)*ncpol[1](j)
        //         + ab[2](j)*ncpol[2](i) + ab[3](i)*ncpol[3](j)
        //         - ab[3](i)*ab[0](j)*c[0] - ab[0](j)*ab[1](i)*c[1]
        //         - ab[1](i)*ab[2](j)*c[2] - ab[2](j)*ab[3](i)*c[3]

        int i, j;
        int ni = ncpol[0]!.Length();
        int nj = ncpol[1]!.Length();
        S0 = new NCollection_HArray2<gp_Pnt>(1, ni, 1, nj);
        NCollection_Array2<gp_Pnt> ss0 = S0.ChangeArray2();
        gp_XYZ c0 = ptch!.Corner(0).Coord();
        gp_XYZ c1 = ptch.Corner(1).Coord();
        gp_XYZ c2 = ptch.Corner(2).Coord();
        gp_XYZ c3 = ptch.Corner(3).Coord();
        for (i = 1; i <= ni; i++)
        {
            double ab1 = ab[1]!.Value(i);
            double ab3 = ab[3]!.Value(i);
            gp_XYZ b0 = ncpol[0]!.Value(i).Coord();
            gp_XYZ b2 = ncpol[2]!.Value(i).Coord();
            for (j = 1; j <= nj; j++)
            {
                double ab0 = ab[0]!.Value(j);
                double ab2 = ab[2]!.Value(j);
                gp_XYZ b1 = ncpol[1]!.Value(j).Coord();
                gp_XYZ b3 = ncpol[3]!.Value(j).Coord();
                gp_XYZ polij = b0.Multiplied(ab0);
                gp_XYZ temp = b1.Multiplied(ab1);
                polij.Add(temp);
                temp = b2.Multiplied(ab2);
                polij.Add(temp);
                temp = b3.Multiplied(ab3);
                polij.Add(temp);
                temp = c0.Multiplied(-ab3 * ab0);
                polij.Add(temp);
                temp = c1.Multiplied(-ab0 * ab1);
                polij.Add(temp);
                temp = c2.Multiplied(-ab1 * ab2);
                polij.Add(temp);
                temp = c3.Multiplied(-ab2 * ab3);
                polij.Add(temp);
                ss0[i, j].SetXYZ(polij);
            }
        }
    }

    private void PerformS1()
    {
        // on construit en temporaire les poles des champs tangents
        // definis par :
        // tgte[ibound](u) - d/dv (S0(u,vbound)) pour ibound = 0 ou 2
        // tgte[ibound](v) - d/du (S0(ubound,v)) pour ibound = 1 ou 3
        // sur les bords ou tgte est defini.
        gp_XYZ[]?[] nt = new gp_XYZ[]?[4];
        NCollection_Array2<gp_Pnt> ss0 = S0!.Array2();
        int l, i, j, k;
        int ni = ss0.ColLength();
        int nj = ss0.RowLength();
        for (i = 0; i <= 3; i++)
        {
            if (ntpol[i] is null)
            {
                nt[i] = null;
            }
            else
            {
                double z = 0;
                int nbp = ntpol[i]!.Length();
                int i1 = 0, i2 = 0, j1 = 0, j2 = 0;
                bool inci = false;
                gp_XYZ[] nti = new gp_XYZ[nbp];
                nt[i] = nti;
                switch (i)
                {
                    case 0:
                        z = -degree[1] / (nk[1]!.Value(2) - nk[1]!.Value(1));
                        inci = true;
                        i1 = 1;
                        i2 = 1;
                        j1 = 1;
                        j2 = 2;
                        break;
                    case 1:
                        l = nk[0]!.Length();
                        z = -degree[0] / (nk[0]!.Value(l) - nk[0]!.Value(l - 1));
                        inci = false;
                        i1 = ni - 1;
                        i2 = ni;
                        j1 = 1;
                        j2 = 1;
                        break;
                    case 2:
                        l = nk[1]!.Length();
                        z = -degree[1] / (nk[1]!.Value(l) - nk[1]!.Value(l - 1));
                        inci = true;
                        i1 = 1;
                        i2 = 1;
                        j1 = nj - 1;
                        j2 = nj;
                        break;
                    case 3:
                        z = -degree[0] / (nk[0]!.Value(2) - nk[0]!.Value(1));
                        inci = false;
                        i1 = 1;
                        i2 = 2;
                        j1 = 1;
                        j2 = 1;
                        break;
                }
                for (k = 0; k < nbp; k++)
                {
                    nti[k] = S0.Value(i1, j1).XYZ();
                    nti[k].Multiply(-1.0);
                    nti[k].Add(S0.Value(i2, j2).XYZ());
                    nti[k].Multiply(z);
                    nti[k].Add(ntpol[i]!.Value(k + 1).XYZ());
                    if (inci)
                    {
                        i1++;
                        i2++;
                    }
                    else
                    {
                        j1++;
                        j2++;
                    }
                }
            }
        }
        // on calcul les termes correctifs pour le melange.
        double coef0 = degree[0] / (nk[0]!.Value(2) - nk[0]!.Value(1));
        double coef1 = degree[1] / (nk[1]!.Value(2) - nk[1]!.Value(1));
        gp_XYZ vtemp, vtemp0, vtemp1;
        if (nt[0] is not null && nt[3] is not null)
        {
            vtemp0 = nt[0]![0].Multiplied(-1.0);
            vtemp0.Add(nt[0]![1]);
            vtemp0.Multiply(coef0);
            vtemp1 = nt[3]![0].Multiplied(-1.0);
            vtemp1.Add(nt[3]![1]);
            vtemp1.Multiply(coef1);
            vtemp = vtemp0.Added(vtemp1);
            vtemp.Multiply(0.5);
            v[0].SetXYZ(vtemp);
        }

        int ln0 = nk[0]!.Length(), lp0 = ncpol[0]!.Length();
        coef0 = degree[0] / (nk[0]!.Value(ln0) - nk[0]!.Value(ln0 - 1));
        coef1 = degree[1] / (nk[1]!.Value(2) - nk[1]!.Value(1));
        if (nt[0] is not null && nt[1] is not null)
        {
            vtemp0 = nt[0]![lp0 - 2].Multiplied(-1.0);
            vtemp0.Add(nt[0]![lp0 - 1]);
            vtemp0.Multiply(coef0);
            vtemp1 = nt[1]![0].Multiplied(-1.0);
            vtemp1.Add(nt[1]![1]);
            vtemp1.Multiply(coef1);
            vtemp = vtemp0.Added(vtemp1);
            vtemp.Multiply(0.5);
            v[1].SetXYZ(vtemp);
        }
        ln0 = nk[0]!.Length();
        lp0 = ncpol[0]!.Length();
        int ln1 = nk[1]!.Length(), lp1 = ncpol[1]!.Length();
        coef0 = degree[0] / (nk[0]!.Value(ln0) - nk[0]!.Value(ln0 - 1));
        coef1 = degree[1] / (nk[1]!.Value(ln1) - nk[1]!.Value(ln1 - 1));
        if (nt[1] is not null && nt[2] is not null)
        {
            vtemp0 = nt[2]![lp0 - 2].Multiplied(-1.0);
            vtemp0.Add(nt[2]![lp0 - 1]);
            vtemp0.Multiply(coef0);
            vtemp1 = nt[1]![lp1 - 2].Multiplied(-1.0);
            vtemp1.Add(nt[1]![lp1 - 1]);
            vtemp1.Multiply(coef1);
            vtemp = vtemp0.Added(vtemp1);
            vtemp.Multiply(0.5);
            v[2].SetXYZ(vtemp);
        }
        ln1 = nk[1]!.Length();
        lp1 = ncpol[1]!.Length();
        coef0 = degree[0] / (nk[0]!.Value(2) - nk[0]!.Value(1));
        coef1 = degree[1] / (nk[1]!.Value(ln1) - nk[1]!.Value(ln1 - 1));
        if (nt[2] is not null && nt[3] is not null)
        {
            vtemp0 = nt[2]![0].Multiplied(-1.0);
            vtemp0.Add(nt[2]![1]);
            vtemp0.Multiply(coef0);
            vtemp1 = nt[3]![lp1 - 2].Multiplied(-1.0);
            vtemp1.Add(nt[3]![lp1 - 1]);
            vtemp1.Multiply(coef1);
            vtemp = vtemp0.Added(vtemp1);
            vtemp.Multiply(0.5);
            v[3].SetXYZ(vtemp);
        }

        // On construit les poles de S1 par combinaison des poles des
        // champs tangents, des poles des fonctions pq, des duv au coins
        // selon la formule :
        // S1(i,j) = pq[0](j)*ntpol[0](i) + pq[1](i)*ntpol[1](j)
        //         + pq[2](j)*ntpol[2](i) + pq[3](i)*ntpol[3](j)
        //         - pq[3](i)*pq[0](j)*v[0] - pq[0](j)*pq[1](i)*v[1]
        //         - pq[1](i)*pq[2](j)*v[2] - pq[2](j)*pq[3](i)*v[3]
        S1 = new NCollection_HArray2<gp_Pnt>(1, ni, 1, nj);
        NCollection_Array2<gp_Pnt> ss1 = S1.ChangeArray2();
        gp_XYZ v0 = v[0].XYZ();
        gp_XYZ v1 = v[1].XYZ();
        gp_XYZ v2 = v[2].XYZ();
        gp_XYZ v3 = v[3].XYZ();

        for (i = 1; i <= ni; i++)
        {
            double pq1 = 0, pq3 = 0;
            if (nt[1] is not null)
            {
                pq1 = -pq[1]!.Value(i);
            }
            if (nt[3] is not null)
            {
                pq3 = pq[3]!.Value(i);
            }
            gp_XYZ t0 = new(), t2 = new();
            if (nt[0] is not null)
            {
                t0 = nt[0]![i - 1];
            }
            if (nt[2] is not null)
            {
                t2 = nt[2]![i - 1];
            }
            for (j = 1; j <= nj; j++)
            {
                double pq0 = 0, pq2 = 0;
                if (nt[0] is not null)
                {
                    pq0 = pq[0]!.Value(j);
                }
                if (nt[2] is not null)
                {
                    pq2 = -pq[2]!.Value(j);
                }
                gp_XYZ t1 = new(), t3 = new();
                if (nt[1] is not null)
                {
                    t1 = nt[1]![j - 1];
                }
                if (nt[3] is not null)
                {
                    t3 = nt[3]![j - 1];
                }

                gp_XYZ tpolij = new(0.0, 0.0, 0.0), temp;
                if (nt[0] is not null)
                {
                    temp = t0.Multiplied(pq0);
                    tpolij.Add(temp);
                }
                if (nt[1] is not null)
                {
                    temp = t1.Multiplied(pq1);
                    tpolij.Add(temp);
                }
                if (nt[2] is not null)
                {
                    temp = t2.Multiplied(pq2);
                    tpolij.Add(temp);
                }
                if (nt[3] is not null)
                {
                    temp = t3.Multiplied(pq3);
                    tpolij.Add(temp);
                }
                if (nt[3] is not null && nt[0] is not null)
                {
                    temp = v0.Multiplied(-pq3 * pq0);
                    tpolij.Add(temp);
                }
                if (nt[0] is not null && nt[1] is not null)
                {
                    temp = v1.Multiplied(-pq0 * pq1);
                    tpolij.Add(temp);
                }
                if (nt[1] is not null && nt[2] is not null)
                {
                    temp = v2.Multiplied(-pq1 * pq2);
                    tpolij.Add(temp);
                }
                if (nt[2] is not null && nt[3] is not null)
                {
                    temp = v3.Multiplied(-pq2 * pq3);
                    tpolij.Add(temp);
                }
                ss1[i, j].SetXYZ(tpolij);
            }
        }
    }

    private void PerformSurface()
    {
        int ni = S0!.ColLength(), nj = S0.RowLength(), i, j;
        NCollection_Array2<gp_Pnt> temp = new(1, ni, 1, nj);
        NCollection_Array2<gp_Pnt> t0 = S0.Array2();
        NCollection_Array2<gp_Pnt> t1 = S1!.Array2();
        for (i = 1; i <= ni; i++)
        {
            for (j = 1; j <= nj; j++)
            {
                temp[i, j].SetXYZ(t0[i, j].XYZ().Added(t1[i, j].XYZ()));
            }
        }
        surf = new Geom_BSplineSurface(temp, nk[0]!.Array1(), nk[1]!.Array1(), nm[0]!.Array1(), nm[1]!.Array1(),
                                       degree[0], degree[1]);
    }

    private bool CheckTgte(int I)
    {
        GeomFill_Boundary bou = ptch!.Bound(I);
        if (!bou.HasNormals())
        {
            return true;
        }
        // On prend 13 points le long du bord et on verifie que le triedre
        // forme par la tangente a la courbe la normale et la tangente du
        // peigne ne change pas d orientation.
        double ll = 1.0 / 12.0, pmix = 0;
        for (int iu = 0; iu < 13; iu++)
        {
            double uu = iu * ll;
            gp_Pnt pbid;
            gp_Vec tgte;
            bou.D1(uu, out pbid, out tgte);
            gp_Vec norm = bou.Norm(uu);
            gp_Vec vfield = tgalg[I]!.Value(uu);
            if (iu == 0)
            {
                pmix = vfield.Dot(tgte.Crossed(norm));
            }
            else
            {
                double pmixcur = vfield.Dot(tgte.Crossed(norm));
                if (pmix * pmixcur < 0.0)
                {
                    return false;
                }
            }
        }
        return true;
    }

    private void MinTgte(int I)
    {
        if (!ptch!.Bound(I).HasNormals())
        {
            return;
        }
        double minmag = RealLast();
        double ll = 0.02;
        for (int iu = 0; iu <= 30; iu++)
        {
            double uu = 0.2 + iu * ll;
            gp_Vec vv = tgalg[I]!.Value(uu);
            double temp = vv.SquareMagnitude();
            if (temp < minmag)
            {
                minmag = temp;
            }
        }
        mig[I] = Math.Sqrt(minmag);
    }

    /// <summary>
    /// The evaluator of the approximation of the two current opposite boundaries and their tangency fields: their
    /// values (Ord 0) or derivatives (Ord 1) at <paramref name="W"/> into <paramref name="Result"/>; returns 0.
    /// </summary>
    public int Eval(double W, int Ord, Span<double> Result)
    {
        Span<double> res = Result;
        int jmp = (3 * ctr[0]);
        switch (Ord)
        {
            case 0:
                if (ctr[0] != 0)
                {
                    ptch!.Bound(ibound[0]).Value(W).Coord(out res[0], out res[1], out res[2]);
                }
                if (ctr[0] == 2)
                {
                    tgalg[ibound[0]]!.Value(W).Coord(out res[3], out res[4], out res[5]);
                }
                if (ctr[1] != 0)
                {
                    ptch!.Bound(ibound[1]).Value(W).Coord(out res[jmp], out res[jmp + 1], out res[jmp + 2]);
                }
                if (ctr[1] == 2)
                {
                    tgalg[ibound[1]]!.Value(W).Coord(out res[jmp + 3], out res[jmp + 4], out res[jmp + 5]);
                }
                break;
            case 1:
                gp_Pnt pt;
                gp_Vec vt;
                if (ctr[0] != 0)
                {
                    ptch!.Bound(ibound[0]).D1(W, out pt, out vt);
                    vt.Coord(out res[0], out res[1], out res[2]);
                }
                if (ctr[0] == 2)
                {
                    tgalg[ibound[0]]!.D1(W).Coord(out res[3], out res[4], out res[5]);
                }
                if (ctr[1] != 0)
                {
                    ptch!.Bound(ibound[1]).D1(W, out pt, out vt);
                    vt.Coord(out res[jmp], out res[jmp + 1], out res[jmp + 2]);
                }
                if (ctr[1] == 2)
                {
                    tgalg[ibound[1]]!.D1(W).Coord(out res[jmp + 3], out res[jmp + 4], out res[jmp + 5]);
                }
                break;
        }
        return 0;
    }

    /// <summary>
    /// Computes the fields of tangents at 31 points along the bound I; they are not the constraint tangents but give
    /// an idea of the algorithmic patch's regularity. As in OCCT, nothing is reported.
    /// </summary>
    public void CheckCoonsAlgPatch(int I)
    {
        int nbp = 30;
        double uu = 0, duu = 0, vv = 0, dvv = 0, ww = 0, dww = 0, u1, u2, v1, v2;
        surf!.Bounds(out u1, out u2, out v1, out v2);
        bool enu = false;
        switch (I)
        {
            case 0:
                uu = ww = u1;
                vv = v1;
                duu = dww = (u2 - u1) / nbp;
                dvv = 0.0;
                break;
            case 1:
                vv = ww = v1;
                uu = u2;
                dvv = dww = (v2 - v1) / nbp;
                duu = 0.0;
                enu = true;
                break;
            case 2:
                uu = ww = u1;
                vv = v2;
                duu = dww = (u2 - u1) / nbp;
                dvv = 0.0;
                break;
            case 3:
                vv = ww = v1;
                uu = u1;
                dvv = dww = (v2 - v1) / nbp;
                duu = 0.0;
                enu = true;
                break;
        }
        gp_Pnt pbound;
        gp_Vec vptch;
        GeomFill_Boundary bou = ptch!.Bound(I);
        for (int k = 0; k <= nbp; k++)
        {
            pbound = bou.Value(ww);
            if (enu)
            {
                vptch = ptch.D1U(uu, vv);
            }
            else
            {
                vptch = ptch.D1V(uu, vv);
            }
            uu += duu;
            vv += dvv;
            ww += dww;
        }
    }

    // std::cout's default formatting of a double (6 significant digits).
    private static string Cout(double theValue) => theValue.ToString("G6", CultureInfo.InvariantCulture);

    /// <summary>
    /// Computes the fields of tangents and normals at 31 points along the bound I and writes the largest deviation
    /// of their angle from a right angle (to <paramref name="theOut"/>, the console by default, as OCCT's std::cout).
    /// </summary>
    public void CheckTgteField(int I, TextWriter? theOut = null)
    {
        TextWriter anOut = theOut ?? Console.Out;
        if (tgalg[I] is null)
        {
            return;
        }
        gp_Pnt p1;
        gp_Vec d1;
        bool caplisse = false;
        double maxang = 0.0, pmix = 0, pmixcur;
        GeomFill_Boundary bou = ptch!.Bound(I);
        for (int iu = 0; iu <= 30; iu++)
        {
            double uu = iu / 30.0;
            bou.D1(uu, out p1, out d1);
            gp_Vec vtg = tgalg[I]!.Value(uu);
            gp_Vec vnor = bou.Norm(uu);
            gp_Vec vcros = d1.Crossed(vnor);
            vcros.Normalize();
            if (iu == 0)
            {
                pmix = vtg.Dot(vcros);
            }
            else
            {
                pmixcur = vtg.Dot(vcros);
                if (pmix * pmixcur < 0.0)
                {
                    caplisse = true;
                }
            }
            if (vnor.Magnitude() > 1.0e-15 && vtg.Magnitude() > 1.0e-15)
            {
                double alpha = Math.Abs(M_PI / 2.0 - Math.Abs(vnor.Angle(vtg)));
                if (Math.Abs(alpha) > maxang)
                {
                    maxang = Math.Abs(alpha);
                }
            }
        }
        anOut.Write("KAlgo angle max sur bord " + I.ToString(CultureInfo.InvariantCulture) + " : " + Cout(maxang) + "\n");
        if (caplisse)
        {
            anOut.Write("sur bord " + I.ToString(CultureInfo.InvariantCulture) + " le champ tangent change de cote!\n");
        }
    }

    /// <summary>
    /// Compares the boundary I and its normals with the approximated curves (boundary and tangency field) at 31
    /// points and writes the largest distance and angle (to <paramref name="theOut"/>, the console by default).
    /// </summary>
    public void CheckApprox(int I, TextWriter? theOut = null)
    {
        TextWriter anOut = theOut ?? Console.Out;
        bool donor = tgalg[I] is not null;
        int nbp = 30;
        double maxang = 0.0, maxdist = 0.0;
        gp_Pnt pbound, papp, pbid;
        gp_Vec vbound, vapp = new();
        GeomFill_Boundary bou = ptch!.Bound(I);
        for (int iu = 0; iu <= nbp; iu++)
        {
            double uu = iu;
            uu /= nbp;
            pbound = bou.Value(uu);
            BSplCLib.D0(uu, 0, degree[I % 2], false, ncpol[I]!.Array1(), BSplCLib.NoWeights(), nk[I % 2]!.Array1(),
                        nm[I % 2]!.Array1(), out papp);
            if (donor)
            {
                BSplCLib.D0(uu, 0, degree[I % 2], false, ntpol[I]!.Array1(), BSplCLib.NoWeights(), nk[I % 2]!.Array1(),
                            nm[I % 2]!.Array1(), out pbid);
                vapp.SetXYZ(pbid.XYZ());
                vbound = bou.Norm(uu);
                if (vapp.Magnitude() > 1.0e-15 && vbound.Magnitude() > 1.0e-15)
                {
                    double alpha = Math.Abs(M_PI / 2.0 - Math.Abs(vbound.Angle(vapp)));
                    if (Math.Abs(alpha) > maxang)
                    {
                        maxang = Math.Abs(alpha);
                    }
                }
            }
            if (papp.Distance(pbound) > maxdist)
            {
                maxdist = papp.Distance(pbound);
            }
        }
        anOut.Write("Controle approx/contrainte sur bord " + I.ToString(CultureInfo.InvariantCulture) + " : \n");
        anOut.Write("Distance max : " + Cout(maxdist) + "\n");
        if (donor)
        {
            maxang = maxang * 180.0 / M_PI;
            anOut.Write("Angle max    : " + Cout(maxang) + " deg\n");
        }
    }

    /// <summary>
    /// Compares the boundary I and its normals with the result surface at 31 points and writes the largest distance
    /// and angle between normals (to <paramref name="theOut"/>, the console by default).
    /// </summary>
    public void CheckResult(int I, TextWriter? theOut = null)
    {
        TextWriter anOut = theOut ?? Console.Out;
        bool donor = tgalg[I] is not null;
        double maxang = 0.0, maxdist = 0.0;
        double uu = 0, duu = 0, vv = 0, dvv = 0, ww = 0, dww = 0, u1, u2, v1, v2;
        surf!.Bounds(out u1, out u2, out v1, out v2);
        switch (I)
        {
            case 0:
                uu = ww = u1;
                vv = v1;
                duu = dww = (u2 - u1) / 30;
                dvv = 0.0;
                break;
            case 1:
                vv = ww = v1;
                uu = u2;
                dvv = dww = (v2 - v1) / 30;
                duu = 0.0;
                break;
            case 2:
                uu = ww = u1;
                vv = v2;
                duu = dww = (u2 - u1) / 30;
                dvv = 0.0;
                break;
            case 3:
                vv = ww = v1;
                uu = u1;
                dvv = dww = (v2 - v1) / 30;
                duu = 0.0;
                break;
        }
        gp_Pnt[] pbound = new gp_Pnt[31], pres = new gp_Pnt[31];
        gp_Vec[] vbound = new gp_Vec[31], vres = new gp_Vec[31];
        GeomFill_Boundary bou = ptch!.Bound(I);
        int k;
        for (k = 0; k <= 30; k++)
        {
            pbound[k] = bou.Value(ww);
            if (!donor)
            {
                surf.D0(uu, vv, out pres[k]);
            }
            else
            {
                vbound[k] = bou.Norm(ww);
                gp_Vec V1, V2;
                surf.D1(uu, vv, out pres[k], out V1, out V2);
                vres[k] = V1.Crossed(V2);
                if (vres[k].Magnitude() > 1.0e-15 && vbound[k].Magnitude() > 1.0e-15)
                {
                    double alpha = Math.Abs(vres[k].Angle(vbound[k]));
                    alpha = Min(alpha, Math.Abs(M_PI - alpha));
                    if (alpha > maxang)
                    {
                        maxang = alpha;
                    }
                }
            }
            if (pres[k].Distance(pbound[k]) > maxdist)
            {
                maxdist = pres[k].Distance(pbound[k]);
            }
            uu += duu;
            vv += dvv;
            ww += dww;
        }
        anOut.Write("Controle resultat/contrainte sur bord " + I.ToString(CultureInfo.InvariantCulture) + " : \n");
        anOut.Write("Distance max : " + Cout(maxdist) + "\n");
        if (donor)
        {
            double angdeg = maxang * 180.0 / M_PI;
            anOut.Write("Angle max    : " + Cout(angdeg) + " deg\n");
        }
    }
}
