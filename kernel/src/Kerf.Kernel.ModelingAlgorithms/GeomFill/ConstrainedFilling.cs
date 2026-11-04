// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_ConstrainedFilling.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_ConstrainedFilling.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using System.Globalization;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A B-spline surface filling three or four boundaries, which serve as path constraints and, when they are
/// <see cref="BoundWithSurf"/>s, as tangency constraints (the Coons style only). Designed for fillets.
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
internal class ConstrainedFilling
{
    private readonly int _degmax;
    private readonly int _segmax;
    private CoonsAlgPatch? _ptch;
    private readonly TgtField?[] _tgalg = new TgtField?[4];
    private readonly double[] _mig = new double[4];
    private readonly CornerState[] _stcor =
        [new CornerState(), new CornerState(), new CornerState(), new CornerState()];
    private readonly Vector3D[] _v = [new Vector3D(), new Vector3D(), new Vector3D(), new Vector3D()];
    private bool _appdone;
    private readonly int[] _degree = new int[2];
    private readonly NCollection_HArray1<Point3D>?[] _curvpol = new NCollection_HArray1<Point3D>?[4];
    private readonly NCollection_HArray1<Point3D>?[] _tgtepol = new NCollection_HArray1<Point3D>?[4];
    private readonly NCollection_HArray1<int>?[] _mults = new NCollection_HArray1<int>?[2];
    private readonly NCollection_HArray1<double>?[] _knots = new NCollection_HArray1<double>?[2];
    private readonly NCollection_HArray1<double>?[] _ab = new NCollection_HArray1<double>?[4];
    private readonly NCollection_HArray1<double>?[] _pq = new NCollection_HArray1<double>?[4];
    private readonly double[] _dom = new double[4];
    private readonly NCollection_HArray1<Point3D>?[] _ncpol = new NCollection_HArray1<Point3D>?[4];
    private readonly NCollection_HArray1<Point3D>?[] _ntpol = new NCollection_HArray1<Point3D>?[4];
    private readonly NCollection_HArray1<int>?[] _nm = new NCollection_HArray1<int>?[2];
    private readonly NCollection_HArray1<double>?[] _nk = new NCollection_HArray1<double>?[2];
    private readonly int[] _ibound = new int[2];
    private readonly int[] _ctr = new int[2];
    private int _nbd3;
    private NCollection_HArray2<Point3D>? _s0;
    private NCollection_HArray2<Point3D>? _s1;
    private BSplineSurface? _surf;

    private static int Inqadd(double d1, double d2, Span<double> k, Span<int> m, int deg, double tolk)
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

    private static Linear Mklin(LawsFunction func)
    {
        Linear? fu = func as Linear;
        if (fu is null)
        {
            fu = new Linear();
            double d, f;
            func.Bounds(out d, out f);
            fu.Set(d, func.Value(d), f, func.Value(f));
        }
        return fu;
    }

    private static void Sortbounds(int nb, Boundary[] bound, bool[] rev, CornerState[] stat)
    {
        // trier les bords (facon bourinos),
        // flaguer ceux a renverser,
        // flaguer les baillements au coins.
        int i, j;
        Boundary temp;
        rev[0] = false;
        Point3D pf, pl;
        Point3D qf, ql;
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
                double df = qf.DistanceTo(pl);
                double dl = ql.DistanceTo(pl);
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
        stat[0].Gap(pl.DistanceTo(qf));

        // flaguer les angles entre tangentes au coins et entre les normales au
        // coins pour les bords contraints.
        Point3D pbid;
        Vector3D tgi, nori, tgn, norn;
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
                tgi = tgi.Reversed();
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
                tgn = tgn.Reversed();
            }
            double ang = M_PI - tgi.AngleTo(tgn);
            stat[next].TgtAng(ang);
            if (bound[i].HasNormals && bound[next].HasNormals)
            {
                stat[next].Constraint();
                nori = bound[i].Norm(li);
                norn = bound[next].Norm(fn);
                ang = nori.AngleTo(norn);
                stat[next].NorAng(ang);
            }
        }
    }

    private static void Coonscnd(int nb, Boundary[] bound, bool[] rev, CornerState[] stat,
                                 TgtField?[] tga, double[] mintg)
    {
        double fact_normalization = 100.0;
        int i;
        // Pour chaque coin contraint, on controle les bounds adjascents.
        for (i = 0; i < nb; i++)
        {
            if (stat[i].HasConstraint)
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
                    Vector3D tgp, dnorp, tgi, dnori, vbid;
                    Point3D pbid;
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

    private static void Killcorners(int nb, Boundary[] bound, bool[] rev, bool[] nrev,
                                    CornerState[] stat, TgtField?[] tga)
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
                if (bound[i].HasNormals && tga[i]!.IsScalable)
                {
                    BSpline bs = global::Kerf.Engine.Construction.Laws.Law.ScaleCub(0.0, 1.0, fnul, lnul, fscal, lscal);
                    tga[i]!.Scale(bs);
                }
            }
        }
    }

    // The evaluator of the curve approximation (OCCT's GeomFill_ConstrainedFilling_Eval).
    private sealed class GeomFill_ConstrainedFilling_Eval(ConstrainedFilling theTool) : EvaluatorFunction
    {
        private readonly ConstrainedFilling _curfil = theTool;

        public override void Evaluate(int dimension, ReadOnlySpan<double> startEnd, double parameter,
                                      int derivativeRequest, Span<double> result, out int errorCode)
        {
            errorCode = _curfil.Eval(parameter, derivativeRequest, result);
        }
    }

    /// <summary>
    /// An empty framework: the surface will have a degree of at most <paramref name="maxDeg"/> and at most
    /// <paramref name="maxSeg"/> segments.
    /// </summary>
    public ConstrainedFilling(int maxDeg, int maxSeg)
    {
        _degmax = maxDeg;
        _segmax = maxSeg;
        _appdone = false;
        _nbd3 = 0;
        _dom[0] = _dom[1] = _dom[2] = _dom[3] = 1.0;
    }

    /// <summary>
    /// Fills three boundaries (the fourth side degenerates to a point). With <paramref name="noCheck"/> the Coons
    /// compatibility conditions at the corners are not checked.
    /// </summary>
    public void Init(Boundary b1, Boundary b2, Boundary b3, bool noCheck = false)
    {
        bool[] rev = new bool[3];
        rev[0] = rev[1] = rev[2] = false;
        Boundary[] bound = new Boundary[3];
        bound[0] = b1;
        bound[1] = b2;
        bound[2] = b3;
        int i;
        Sortbounds(3, bound, rev, _stcor);

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
        Point3D p1 = bound[1].Value(1.0);
        Point3D p2 = bound[2].Value(1.0);
        Point3D ppp = new(0.5 * (p1.XYZ() + p2.XYZ()));
        double t3 = Max(bound[1].Tol3d(), bound[2].Tol3d());
        DegeneratedBound db = new(ppp, 0.0, 1.0, t3, 10.0);

        _ptch = new CoonsAlgPatch(bound[0], bound[1], db, bound[2]);

        TgtField?[] ttgalg = new TgtField?[3];
        if (bound[0].HasNormals)
        {
            ttgalg[0] = _tgalg[0] = new TgtOnCoons(_ptch, 0);
        }
        if (bound[1].HasNormals)
        {
            ttgalg[1] = _tgalg[1] = new TgtOnCoons(_ptch, 1);
        }
        if (bound[2].HasNormals)
        {
            ttgalg[2] = _tgalg[3] = new TgtOnCoons(_ptch, 3);
        }

        for (i = 0; i <= 3; i++)
        {
            _mig[i] = 1.0;
            if (_tgalg[i] is not null)
            {
                MinTgte(i);
            }
        }

        if (!noCheck)
        {
            // On  verifie enfin les conditions  de compatibilites sur les derivees
            // aux coins maintenant qu on a quelque chose a quoi les comparer.
            bool[] nrev = new bool[3];
            nrev[0] = nrev[1] = false;
            nrev[2] = true;
            _mig[2] = _mig[3];
            Coonscnd(3, bound, nrev, _stcor, ttgalg, _mig);
            Killcorners(3, bound, rev, nrev, _stcor, ttgalg);
        }
        // on remet les coins en place (on duplique la pointe).
        _stcor[3] = _stcor[2];

        for (i = 0; i <= 3; i++)
        {
            _mig[i] = 1.0;
            if (_tgalg[i] is not null)
            {
                if (!CheckTgte(i))
                {
                    LawsFunction fu1, fu2;
                    _ptch.Func(out fu1, out fu2);
                    LawsFunction ffu1 = global::Kerf.Engine.Construction.Laws.Law.MixBnd((fu1 as Linear)!);
                    LawsFunction ffu2 = global::Kerf.Engine.Construction.Laws.Law.MixBnd((fu2 as Linear)!);
                    _ptch.SetFunc(ffu1, ffu2);
                    break;
                }
            }
        }

        Build();
    }

    /// <summary>
    /// Fills four boundaries, given in any order: they are sorted and, if need be, reversed and reparametrised. With
    /// <paramref name="noCheck"/> the Coons compatibility conditions at the corners are not checked.
    /// </summary>
    public void Init(Boundary b1, Boundary b2, Boundary b3, Boundary b4,
                     bool noCheck = false)
    {
        bool[] rev = new bool[4];
        rev[0] = rev[1] = rev[2] = rev[3] = false;
        Boundary[] bound = new Boundary[4];
        bound[0] = b1;
        bound[1] = b2;
        bound[2] = b3;
        bound[3] = b4;
        int i;
        Sortbounds(4, bound, rev, _stcor);

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
        _ptch = new CoonsAlgPatch(bound[0], bound[1], bound[2], bound[3]);
        for (i = 0; i <= 3; i++)
        {
            if (bound[i].HasNormals)
            {
                _tgalg[i] = new TgtOnCoons(_ptch, i);
            }
        }
        // on calcule le min de chacun des champs tangents pour l evaluation
        // des tolerances.
        for (i = 0; i <= 3; i++)
        {
            _mig[i] = 1.0;
            if (_tgalg[i] is not null)
            {
                MinTgte(i);
            }
        }

        if (!noCheck)
        {
            // On  verifie enfin les conditions  de compatibilites sur les derivees
            // aux coins maintenant qu on a quelque chose a quoi les comparer.
            bool[] nrev = new bool[4];
            nrev[0] = nrev[1] = false;
            nrev[2] = nrev[3] = true;
            Coonscnd(4, bound, nrev, _stcor, _tgalg, _mig);
            Killcorners(4, bound, rev, nrev, _stcor, _tgalg);
        }
        // On verifie les champs tangents ne changent pas de direction.
        for (i = 0; i <= 3; i++)
        {
            _mig[i] = 1.0;
            if (_tgalg[i] is not null)
            {
                if (!CheckTgte(i))
                {
                    LawsFunction fu1, fu2;
                    _ptch.Func(out fu1, out fu2);
                    LawsFunction ffu1 = global::Kerf.Engine.Construction.Laws.Law.MixBnd((fu1 as Linear)!);
                    LawsFunction ffu2 = global::Kerf.Engine.Construction.Laws.Law.MixBnd((fu2 as Linear)!);
                    _ptch.SetFunc(ffu1, ffu2);
                    break;
                }
            }
        }

        Build();
    }

    /// <summary>
    /// Sets the relative part <paramref name="l"/> (at most 1, the default) of the parametric range of the
    /// constrained boundary <paramref name="b"/> on which its blending function propagates its tangency field.
    /// Call it after <c>Init</c>, then <see cref="ReBuild"/>.
    /// </summary>
    public void SetDomain(double l, BoundWithSurf b)
    {
        if (ReferenceEquals(b, _ptch!.Bound(0)))
        {
            _dom[0] = Min(1.0, Math.Abs(l));
        }
        else if (ReferenceEquals(b, _ptch.Bound(1)))
        {
            _dom[1] = Min(1.0, Math.Abs(l));
        }
        else if (ReferenceEquals(b, _ptch.Bound(2)))
        {
            _dom[2] = Min(1.0, Math.Abs(l));
        }
        else if (ReferenceEquals(b, _ptch.Bound(3)))
        {
            _dom[3] = Min(1.0, Math.Abs(l));
        }
    }

    /// <summary>Recomputes the poles with the blending functions set by <see cref="SetDomain"/>.</summary>
    public void ReBuild()
    {
        if (!_appdone)
        {
            throw new FailureException("GeomFill_ConstrainedFilling::ReBuild Approx non faite");
        }
        MatchKnots();
        PerformS0();
        PerformS1();
        PerformSurface();
    }

    /// <summary>The boundary <paramref name="i"/> (from 0) after sorting.</summary>
    public Boundary Boundary(int i) => _ptch!.Bound(i);

    /// <summary>The surface (null before <c>Init</c>), frozen.</summary>
    public BSplineSurface? Surface()
    {
        _surf?.Freeze();
        return _surf;
    }

    /// <summary>The surface, not frozen, for ported callers.</summary>
    internal BSplineSurface? InternalSurface() => _surf;

    private void Build()
    {
        for (int count = 0; count < 2; count++)
        {
            _ibound[0] = count;
            _ibound[1] = count + 2;
            _ctr[0] = _ctr[1] = _nbd3 = 0;
            int ii;
            for (ii = 0; ii < 2; ii++)
            {
                if (_ptch!.Bound(_ibound[ii]).HasNormals)
                {
                    _ctr[ii] = 2;
                }
                else if (!_ptch.Bound(_ibound[ii]).IsDegenerated)
                {
                    _ctr[ii] = 1;
                }
                _nbd3 += _ctr[ii];
            }
            if (_nbd3 != 0)
            {
                PerformApprox();
            }
        }
        _appdone = true;
        MatchKnots();
        PerformS0();
        PerformS1();
        PerformSurface();
    }

    private void PerformApprox()
    {
        int ii;
        NCollection_HArray1<double>? tol3d = null, tol2d = null, tol1d = null;
        if (_nbd3 != 0)
        {
            tol3d = new NCollection_HArray1<double>(1, _nbd3);
        }
        int i3d = 0;
        for (ii = 0; ii <= 1; ii++)
        {
            if (_ctr[ii] != 0)
            {
                tol3d!.SetValue((++i3d), _ptch!.Bound(_ibound[ii]).Tol3d());
            }
            if (_ctr[ii] == 2)
            {
                tol3d!.SetValue(++i3d, 0.5 * _mig[_ibound[ii]] * _ptch!.Bound(_ibound[ii]).Tolang());
            }
        }
        double f, l;
        _ptch!.Bound(_ibound[0]).Bounds(out f, out l);

        GeomFill_ConstrainedFilling_Eval ev = new(this);
        ApproxAFunction app = new(0, 0, _nbd3, tol1d, tol2d, tol3d, f, l, C1, _degmax, _segmax, ev);

        if (app.IsDone || app.HasResult)
        {
            int imk = Min(_ibound[0], _ibound[1]);
            int nbpol = app.NbPoles();
            _degree[imk] = app.Degree();
            _mults[imk] = app.Multiplicities();
            _knots[imk] = app.Knots();
            i3d = 0;
            for (ii = 0; ii <= 1; ii++)
            {
                _curvpol[_ibound[ii]] = new NCollection_HArray1<Point3D>(1, nbpol);
                NCollection_Array1<Point3D> cp = _curvpol[_ibound[ii]]!.ChangeArray1();
                if (_ctr[ii] != 0)
                {
                    app.Poles(++i3d, cp);
                }
                else
                {
                    Point3D ppp = _ptch.Bound(_ibound[ii]).Value(0.5 * (f + l));
                    for (int ij = 1; ij <= nbpol; ij++)
                    {
                        cp[ij] = ppp;
                    }
                }
                if (_ctr[ii] == 2)
                {
                    _tgtepol[_ibound[ii]] = new NCollection_HArray1<Point3D>(1, nbpol);
                    app.Poles(++i3d, _tgtepol[_ibound[ii]]!.ChangeArray1());
                }
            }
        }
    }

    // OCCT's NCollection_Array1 over the first nbadd values of a C array (kadd, madd).
    private static NCollection_Array1<T> FirstValues<T>(ReadOnlySpan<T> values, int count)
    {
        NCollection_Array1<T> array = new(1, count);
        for (int i = 1; i <= count; i++)
        {
            array[i] = values[i - 1];
        }
        return array;
    }

    private void MatchKnots()
    {
        // on n insere rien si les domaines valent 1.
        int i, j, l;
        int[] ind = new int[4];
        _nm[0] = _mults[0];
        _nm[1] = _mults[1];
        _nk[0] = _knots[0];
        _nk[1] = _knots[1];
        ind[0] = _nk[1]!.Length();
        ind[2] = 1;
        ind[1] = 1;
        ind[3] = _nk[0]!.Length();
        _ncpol[0] = _curvpol[0];
        _ncpol[1] = _curvpol[1];
        _ncpol[2] = _curvpol[2];
        _ncpol[3] = _curvpol[3];
        _ntpol[0] = _tgtepol[0];
        _ntpol[1] = _tgtepol[1];
        _ntpol[2] = _tgtepol[2];
        _ntpol[3] = _tgtepol[3];
        Span<double> kadd = stackalloc double[2];
        Span<int> madd = stackalloc int[2];
        double tolk = 1.0 / Max(10, 2 * _knots[1]!.Array1().Length());
        int nbadd = Inqadd(_dom[0], _dom[2], kadd, madd, _degree[1], tolk);
        if (nbadd != 0)
        {
            NCollection_Array1<double> addk = FirstValues<double>(kadd, nbadd);
            NCollection_Array1<int> addm = FirstValues<int>(madd, nbadd);
            int nbnp, nbnk;
            if (global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareInsertKnots(_degree[1], false, _knots[1]!.Array1(), _mults[1]!.Array1(), addk, addm,
                                            out nbnp, out nbnk, tolk, false))
            {
                _nm[1] = new NCollection_HArray1<int>(1, nbnk);
                _nk[1] = new NCollection_HArray1<double>(1, nbnk);
                _ncpol[1] = new NCollection_HArray1<Point3D>(1, nbnp);
                _ncpol[3] = new NCollection_HArray1<Point3D>(1, nbnp);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(_degree[1], false, _curvpol[1]!.Array1(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _knots[1]!.Array1(),
                                     _mults[1]!.Array1(), addk, addm, _ncpol[1]!.ChangeArray1(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                     _nk[1]!.ChangeArray1(), _nm[1]!.ChangeArray1(), tolk, false);

                global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(_degree[1], false, _curvpol[3]!.Array1(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _knots[1]!.Array1(),
                                     _mults[1]!.Array1(), addk, addm, _ncpol[3]!.ChangeArray1(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                     _nk[1]!.ChangeArray1(), _nm[1]!.ChangeArray1(), tolk, false);
                if (_tgtepol[1] is not null)
                {
                    _ntpol[1] = new NCollection_HArray1<Point3D>(1, nbnp);
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(_degree[1], false, _tgtepol[1]!.Array1(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                         _knots[1]!.Array1(), _mults[1]!.Array1(), addk, addm, _ntpol[1]!.ChangeArray1(),
                                         global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _nk[1]!.ChangeArray1(), _nm[1]!.ChangeArray1(), tolk, false);
                }
                if (_tgtepol[3] is not null)
                {
                    _ntpol[3] = new NCollection_HArray1<Point3D>(1, nbnp);
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(_degree[1], false, _tgtepol[3]!.Array1(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                         _knots[1]!.Array1(), _mults[1]!.Array1(), addk, addm, _ntpol[3]!.ChangeArray1(),
                                         global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _nk[1]!.ChangeArray1(), _nm[1]!.ChangeArray1(), tolk, false);
                }
            }
            if (_dom[0] != 1.0)
            {
                for (i = 2; i <= nbnk; i++)
                {
                    // OCCT: compares the domain with the multiplicities, not the knots (occt-issues.md)
                    if (Math.Abs(_dom[0] - _nm[1]!.Value(i)) < tolk)
                    {
                        ind[0] = i;
                        break;
                    }
                }
            }
            if (_dom[2] != 1.0)
            {
                for (i = 1; i < nbnk; i++)
                {
                    // OCCT: the multiplicities, not the knots (occt-issues.md)
                    if (Math.Abs(1.0 - _dom[2] - _nm[1]!.Value(i)) < tolk)
                    {
                        ind[2] = i;
                        break;
                    }
                }
            }
        }
        tolk = 1.0 / Max(10.0, 2.0 * _knots[0]!.Array1().Length());
        nbadd = Inqadd(_dom[1], _dom[3], kadd, madd, _degree[0], tolk);
        if (nbadd != 0)
        {
            NCollection_Array1<double> addk = FirstValues<double>(kadd, nbadd);
            NCollection_Array1<int> addm = FirstValues<int>(madd, nbadd);
            int nbnp, nbnk;
            if (global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareInsertKnots(_degree[0], false, _knots[0]!.Array1(), _mults[0]!.Array1(), addk, addm,
                                            out nbnp, out nbnk, tolk, false))
            {
                _nm[0] = new NCollection_HArray1<int>(1, nbnk);
                _nk[0] = new NCollection_HArray1<double>(1, nbnk);
                _ncpol[0] = new NCollection_HArray1<Point3D>(1, nbnp);
                _ncpol[2] = new NCollection_HArray1<Point3D>(1, nbnp);
                global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(_degree[0], false, _curvpol[0]!.Array1(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _knots[0]!.Array1(),
                                     _mults[0]!.Array1(), addk, addm, _ncpol[0]!.ChangeArray1(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                     _nk[0]!.ChangeArray1(), _nm[0]!.ChangeArray1(), tolk, false);

                global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(_degree[0], false, _curvpol[2]!.Array1(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _knots[0]!.Array1(),
                                     _mults[0]!.Array1(), addk, addm, _ncpol[2]!.ChangeArray1(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                     _nk[0]!.ChangeArray1(), _nm[0]!.ChangeArray1(), tolk, false);
                if (_tgtepol[0] is not null)
                {
                    _ntpol[0] = new NCollection_HArray1<Point3D>(1, nbnp);
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(_degree[0], false, _tgtepol[0]!.Array1(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                         _knots[0]!.Array1(), _mults[0]!.Array1(), addk, addm, _ntpol[0]!.ChangeArray1(),
                                         global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _nk[0]!.ChangeArray1(), _nm[0]!.ChangeArray1(), tolk, false);
                }
                if (_tgtepol[2] is not null)
                {
                    _ntpol[2] = new NCollection_HArray1<Point3D>(1, nbnp);
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(_degree[0], false, _tgtepol[2]!.Array1(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                         _knots[0]!.Array1(), _mults[0]!.Array1(), addk, addm, _ntpol[2]!.ChangeArray1(),
                                         global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _nk[0]!.ChangeArray1(), _nm[0]!.ChangeArray1(), tolk, false);
                }
            }
            if (_dom[1] != 1.0)
            {
                for (i = 2; i <= nbnk; i++)
                {
                    // OCCT: the multiplicities, not the knots (occt-issues.md)
                    if (Math.Abs(_dom[1] - _nm[0]!.Value(i)) < tolk)
                    {
                        ind[1] = i;
                        break;
                    }
                }
            }
            if (_dom[3] != 1.0)
            {
                for (i = 1; i < nbnk; i++)
                {
                    // OCCT: the multiplicities, not the knots (occt-issues.md)
                    if (Math.Abs(1.0 - _dom[3] - _nm[0]!.Value(i)) < tolk)
                    {
                        ind[3] = i;
                        break;
                    }
                }
            }
        }
        Linear fu = Mklin(_ptch!.Func(0));
        _ab[0] = global::Kerf.Engine.Construction.Laws.Law.MixBnd(_degree[1], _nk[1]!.Array1(), _nm[1]!.Array1(), fu);
        fu = Mklin(_ptch.Func(1));
        _ab[1] = global::Kerf.Engine.Construction.Laws.Law.MixBnd(_degree[0], _nk[0]!.Array1(), _nm[0]!.Array1(), fu);

        for (i = 0; i < 2; i++)
        {
            l = _ab[i]!.Length();
            _ab[i + 2] = new NCollection_HArray1<double>(1, l);
            for (j = 1; j <= l; j++)
            {
                _ab[i + 2]!.SetValue(j, 1.0 - _ab[i]!.Value(j));
            }
        }
        _pq[0] = global::Kerf.Engine.Construction.Laws.Law.MixTgt(_degree[1], _nk[1]!.Array1(), _nm[1]!.Array1(), true, ind[0]);
        _pq[2] = global::Kerf.Engine.Construction.Laws.Law.MixTgt(_degree[1], _nk[1]!.Array1(), _nm[1]!.Array1(), false, ind[2]);

        _pq[1] = global::Kerf.Engine.Construction.Laws.Law.MixTgt(_degree[0], _nk[0]!.Array1(), _nm[0]!.Array1(), false, ind[1]);
        _pq[3] = global::Kerf.Engine.Construction.Laws.Law.MixTgt(_degree[0], _nk[0]!.Array1(), _nm[0]!.Array1(), true, ind[3]);
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
        int ni = _ncpol[0]!.Length();
        int nj = _ncpol[1]!.Length();
        _s0 = new NCollection_HArray2<Point3D>(1, ni, 1, nj);
        NCollection_Array2<Point3D> ss0 = _s0.ChangeArray2();
        Xyz c0 = _ptch!.Corner(0).Coord();
        Xyz c1 = _ptch.Corner(1).Coord();
        Xyz c2 = _ptch.Corner(2).Coord();
        Xyz c3 = _ptch.Corner(3).Coord();
        for (i = 1; i <= ni; i++)
        {
            double ab1 = _ab[1]!.Value(i);
            double ab3 = _ab[3]!.Value(i);
            Xyz b0 = _ncpol[0]!.Value(i).Coord();
            Xyz b2 = _ncpol[2]!.Value(i).Coord();
            for (j = 1; j <= nj; j++)
            {
                double ab0 = _ab[0]!.Value(j);
                double ab2 = _ab[2]!.Value(j);
                Xyz b1 = _ncpol[1]!.Value(j).Coord();
                Xyz b3 = _ncpol[3]!.Value(j).Coord();
                Xyz polij = b0.Multiplied(ab0);
                Xyz temp = b1.Multiplied(ab1);
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
                ss0[i, j] = ss0[i, j].WithXYZ(polij);
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
        Xyz[]?[] nt = new Xyz[]?[4];
        NCollection_Array2<Point3D> ss0 = _s0!.Array2();
        int l, i, j, k;
        int ni = ss0.ColLength();
        int nj = ss0.RowLength();
        for (i = 0; i <= 3; i++)
        {
            if (_ntpol[i] is null)
            {
                nt[i] = null;
            }
            else
            {
                double z = 0;
                int nbp = _ntpol[i]!.Length();
                int i1 = 0, i2 = 0, j1 = 0, j2 = 0;
                bool inci = false;
                Xyz[] nti = new Xyz[nbp];
                nt[i] = nti;
                switch (i)
                {
                    case 0:
                        z = -_degree[1] / (_nk[1]!.Value(2) - _nk[1]!.Value(1));
                        inci = true;
                        i1 = 1;
                        i2 = 1;
                        j1 = 1;
                        j2 = 2;
                        break;
                    case 1:
                        l = _nk[0]!.Length();
                        z = -_degree[0] / (_nk[0]!.Value(l) - _nk[0]!.Value(l - 1));
                        inci = false;
                        i1 = ni - 1;
                        i2 = ni;
                        j1 = 1;
                        j2 = 1;
                        break;
                    case 2:
                        l = _nk[1]!.Length();
                        z = -_degree[1] / (_nk[1]!.Value(l) - _nk[1]!.Value(l - 1));
                        inci = true;
                        i1 = 1;
                        i2 = 1;
                        j1 = nj - 1;
                        j2 = nj;
                        break;
                    case 3:
                        z = -_degree[0] / (_nk[0]!.Value(2) - _nk[0]!.Value(1));
                        inci = false;
                        i1 = 1;
                        i2 = 2;
                        j1 = 1;
                        j2 = 1;
                        break;
                }
                for (k = 0; k < nbp; k++)
                {
                    nti[k] = _s0.Value(i1, j1).XYZ();
                    nti[k].Multiply(-1.0);
                    nti[k].Add(_s0.Value(i2, j2).XYZ());
                    nti[k].Multiply(z);
                    nti[k].Add(_ntpol[i]!.Value(k + 1).XYZ());
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
        double coef0 = _degree[0] / (_nk[0]!.Value(2) - _nk[0]!.Value(1));
        double coef1 = _degree[1] / (_nk[1]!.Value(2) - _nk[1]!.Value(1));
        Xyz vtemp, vtemp0, vtemp1;
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
            _v[0] = _v[0].WithXYZ(vtemp);
        }

        int ln0 = _nk[0]!.Length(), lp0 = _ncpol[0]!.Length();
        coef0 = _degree[0] / (_nk[0]!.Value(ln0) - _nk[0]!.Value(ln0 - 1));
        coef1 = _degree[1] / (_nk[1]!.Value(2) - _nk[1]!.Value(1));
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
            _v[1] = _v[1].WithXYZ(vtemp);
        }
        ln0 = _nk[0]!.Length();
        lp0 = _ncpol[0]!.Length();
        int ln1 = _nk[1]!.Length(), lp1 = _ncpol[1]!.Length();
        coef0 = _degree[0] / (_nk[0]!.Value(ln0) - _nk[0]!.Value(ln0 - 1));
        coef1 = _degree[1] / (_nk[1]!.Value(ln1) - _nk[1]!.Value(ln1 - 1));
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
            _v[2] = _v[2].WithXYZ(vtemp);
        }
        ln1 = _nk[1]!.Length();
        lp1 = _ncpol[1]!.Length();
        coef0 = _degree[0] / (_nk[0]!.Value(2) - _nk[0]!.Value(1));
        coef1 = _degree[1] / (_nk[1]!.Value(ln1) - _nk[1]!.Value(ln1 - 1));
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
            _v[3] = _v[3].WithXYZ(vtemp);
        }

        // On construit les poles de S1 par combinaison des poles des
        // champs tangents, des poles des fonctions pq, des duv au coins
        // selon la formule :
        // S1(i,j) = pq[0](j)*ntpol[0](i) + pq[1](i)*ntpol[1](j)
        //         + pq[2](j)*ntpol[2](i) + pq[3](i)*ntpol[3](j)
        //         - pq[3](i)*pq[0](j)*v[0] - pq[0](j)*pq[1](i)*v[1]
        //         - pq[1](i)*pq[2](j)*v[2] - pq[2](j)*pq[3](i)*v[3]
        _s1 = new NCollection_HArray2<Point3D>(1, ni, 1, nj);
        NCollection_Array2<Point3D> ss1 = _s1.ChangeArray2();
        Xyz v0 = _v[0].XYZ();
        Xyz v1 = _v[1].XYZ();
        Xyz v2 = _v[2].XYZ();
        Xyz v3 = _v[3].XYZ();

        for (i = 1; i <= ni; i++)
        {
            double pq1 = 0, pq3 = 0;
            if (nt[1] is not null)
            {
                pq1 = -_pq[1]!.Value(i);
            }
            if (nt[3] is not null)
            {
                pq3 = _pq[3]!.Value(i);
            }
            Xyz t0 = new(), t2 = new();
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
                    pq0 = _pq[0]!.Value(j);
                }
                if (nt[2] is not null)
                {
                    pq2 = -_pq[2]!.Value(j);
                }
                Xyz t1 = new(), t3 = new();
                if (nt[1] is not null)
                {
                    t1 = nt[1]![j - 1];
                }
                if (nt[3] is not null)
                {
                    t3 = nt[3]![j - 1];
                }

                Xyz tpolij = new(0.0, 0.0, 0.0), temp;
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
                ss1[i, j] = ss1[i, j].WithXYZ(tpolij);
            }
        }
    }

    private void PerformSurface()
    {
        int ni = _s0!.ColLength(), nj = _s0.RowLength(), i, j;
        NCollection_Array2<Point3D> temp = new(1, ni, 1, nj);
        NCollection_Array2<Point3D> t0 = _s0.Array2();
        NCollection_Array2<Point3D> t1 = _s1!.Array2();
        for (i = 1; i <= ni; i++)
        {
            for (j = 1; j <= nj; j++)
            {
                temp[i, j] = temp[i, j].WithXYZ(t0[i, j].XYZ().Added(t1[i, j].XYZ()));
            }
        }
        _surf = new BSplineSurface(temp, _nk[0]!.Array1(), _nk[1]!.Array1(), _nm[0]!.Array1(), _nm[1]!.Array1(),
                                       _degree[0], _degree[1]);
    }

    private bool CheckTgte(int i)
    {
        Boundary bou = _ptch!.Bound(i);
        if (!bou.HasNormals)
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
            Point3D pbid;
            Vector3D tgte;
            bou.D1(uu, out pbid, out tgte);
            Vector3D norm = bou.Norm(uu);
            Vector3D vfield = _tgalg[i]!.Value(uu);
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

    private void MinTgte(int i)
    {
        if (!_ptch!.Bound(i).HasNormals)
        {
            return;
        }
        double minmag = RealLast();
        double ll = 0.02;
        for (int iu = 0; iu <= 30; iu++)
        {
            double uu = 0.2 + iu * ll;
            Vector3D vv = _tgalg[i]!.Value(uu);
            double temp = vv.SquareMagnitude;
            if (temp < minmag)
            {
                minmag = temp;
            }
        }
        _mig[i] = Math.Sqrt(minmag);
    }

    /// <summary>
    /// The evaluator of the approximation of the two current opposite boundaries and their tangency fields: their
    /// values (Ord 0) or derivatives (Ord 1) at <paramref name="w"/> into <paramref name="result"/>; returns 0.
    /// </summary>
    public int Eval(double w, int ord, Span<double> result)
    {
        Span<double> res = result;
        int jmp = (3 * _ctr[0]);
        switch (ord)
        {
            case 0:
                if (_ctr[0] != 0)
                {
                    _ptch!.Bound(_ibound[0]).Value(w).Coord(out res[0], out res[1], out res[2]);
                }
                if (_ctr[0] == 2)
                {
                    _tgalg[_ibound[0]]!.Value(w).Coord(out res[3], out res[4], out res[5]);
                }
                if (_ctr[1] != 0)
                {
                    _ptch!.Bound(_ibound[1]).Value(w).Coord(out res[jmp], out res[jmp + 1], out res[jmp + 2]);
                }
                if (_ctr[1] == 2)
                {
                    _tgalg[_ibound[1]]!.Value(w).Coord(out res[jmp + 3], out res[jmp + 4], out res[jmp + 5]);
                }
                break;
            case 1:
                Point3D pt;
                Vector3D vt;
                if (_ctr[0] != 0)
                {
                    _ptch!.Bound(_ibound[0]).D1(w, out pt, out vt);
                    vt.Coord(out res[0], out res[1], out res[2]);
                }
                if (_ctr[0] == 2)
                {
                    _tgalg[_ibound[0]]!.D1(w).Coord(out res[3], out res[4], out res[5]);
                }
                if (_ctr[1] != 0)
                {
                    _ptch!.Bound(_ibound[1]).D1(w, out pt, out vt);
                    vt.Coord(out res[jmp], out res[jmp + 1], out res[jmp + 2]);
                }
                if (_ctr[1] == 2)
                {
                    _tgalg[_ibound[1]]!.D1(w).Coord(out res[jmp + 3], out res[jmp + 4], out res[jmp + 5]);
                }
                break;
        }
        return 0;
    }

    /// <summary>
    /// Computes the fields of tangents at 31 points along the bound I; they are not the constraint tangents but give
    /// an idea of the algorithmic patch's regularity. As in OCCT, nothing is reported.
    /// </summary>
    public void CheckCoonsAlgPatch(int i)
    {
        int nbp = 30;
        double uu = 0, duu = 0, vv = 0, dvv = 0, ww = 0, dww = 0, u1, u2, v1, v2;
        _surf!.ParameterBounds(out u1, out u2, out v1, out v2);
        bool enu = false;
        switch (i)
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
        Point3D pbound;
        Vector3D vptch;
        Boundary bou = _ptch!.Bound(i);
        for (int k = 0; k <= nbp; k++)
        {
            pbound = bou.Value(ww);
            if (enu)
            {
                vptch = _ptch.D1U(uu, vv);
            }
            else
            {
                vptch = _ptch.D1V(uu, vv);
            }
            uu += duu;
            vv += dvv;
            ww += dww;
        }
    }

    // std::cout's default formatting of a double (6 significant digits).
    private static string Cout(double value) => value.ToString("G6", CultureInfo.InvariantCulture);

    /// <summary>
    /// Computes the fields of tangents and normals at 31 points along the bound I and writes the largest deviation
    /// of their angle from a right angle (to <paramref name="theOut"/>, the console by default, as OCCT's std::cout).
    /// </summary>
    public void CheckTgteField(int i, TextWriter? theOut = null)
    {
        TextWriter anOut = theOut ?? Console.Out;
        if (_tgalg[i] is null)
        {
            return;
        }
        Point3D p1;
        Vector3D d1;
        bool caplisse = false;
        double maxang = 0.0, pmix = 0, pmixcur;
        Boundary bou = _ptch!.Bound(i);
        for (int iu = 0; iu <= 30; iu++)
        {
            double uu = iu / 30.0;
            bou.D1(uu, out p1, out d1);
            Vector3D vtg = _tgalg[i]!.Value(uu);
            Vector3D vnor = bou.Norm(uu);
            Vector3D vcros = d1.Crossed(vnor);
            vcros = vcros.Normalized();
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
            if (vnor.Magnitude > 1.0e-15 && vtg.Magnitude > 1.0e-15)
            {
                double alpha = Math.Abs(M_PI / 2.0 - Math.Abs(vnor.AngleTo(vtg)));
                if (Math.Abs(alpha) > maxang)
                {
                    maxang = Math.Abs(alpha);
                }
            }
        }
        anOut.Write("KAlgo angle max sur bord " + i.ToString(CultureInfo.InvariantCulture) + " : " + Cout(maxang) + "\n");
        if (caplisse)
        {
            anOut.Write("sur bord " + i.ToString(CultureInfo.InvariantCulture) + " le champ tangent change de cote!\n");
        }
    }

    /// <summary>
    /// Compares the boundary I and its normals with the approximated curves (boundary and tangency field) at 31
    /// points and writes the largest distance and angle (to <paramref name="theOut"/>, the console by default).
    /// </summary>
    public void CheckApprox(int i, TextWriter? theOut = null)
    {
        TextWriter anOut = theOut ?? Console.Out;
        bool donor = _tgalg[i] is not null;
        int nbp = 30;
        double maxang = 0.0, maxdist = 0.0;
        Point3D pbound, papp, pbid;
        Vector3D vbound, vapp = new();
        Boundary bou = _ptch!.Bound(i);
        for (int iu = 0; iu <= nbp; iu++)
        {
            double uu = iu;
            uu /= nbp;
            pbound = bou.Value(uu);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(uu, 0, _degree[i % 2], false, _ncpol[i]!.Array1(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _nk[i % 2]!.Array1(),
                        _nm[i % 2]!.Array1(), out papp);
            if (donor)
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(uu, 0, _degree[i % 2], false, _ntpol[i]!.Array1(), global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), _nk[i % 2]!.Array1(),
                            _nm[i % 2]!.Array1(), out pbid);
                vapp = vapp.WithXYZ(pbid.XYZ());
                vbound = bou.Norm(uu);
                if (vapp.Magnitude > 1.0e-15 && vbound.Magnitude > 1.0e-15)
                {
                    double alpha = Math.Abs(M_PI / 2.0 - Math.Abs(vbound.AngleTo(vapp)));
                    if (Math.Abs(alpha) > maxang)
                    {
                        maxang = Math.Abs(alpha);
                    }
                }
            }
            if (papp.DistanceTo(pbound) > maxdist)
            {
                maxdist = papp.DistanceTo(pbound);
            }
        }
        anOut.Write("Controle approx/contrainte sur bord " + i.ToString(CultureInfo.InvariantCulture) + " : \n");
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
    public void CheckResult(int i, TextWriter? theOut = null)
    {
        TextWriter anOut = theOut ?? Console.Out;
        bool donor = _tgalg[i] is not null;
        double maxang = 0.0, maxdist = 0.0;
        double uu = 0, duu = 0, vv = 0, dvv = 0, ww = 0, dww = 0, u1, u2, v1, v2;
        _surf!.ParameterBounds(out u1, out u2, out v1, out v2);
        switch (i)
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
        Point3D[] pbound = new Point3D[31], pres = new Point3D[31];
        Vector3D[] vbound = new Vector3D[31], vres = new Vector3D[31];
        Boundary bou = _ptch!.Bound(i);
        int k;
        for (k = 0; k <= 30; k++)
        {
            pbound[k] = bou.Value(ww);
            if (!donor)
            {
                _surf.D0(uu, vv, out pres[k]);
            }
            else
            {
                vbound[k] = bou.Norm(ww);
                Vector3D V1, V2;
                _surf.D1(uu, vv, out pres[k], out V1, out V2);
                vres[k] = V1.Crossed(V2);
                if (vres[k].Magnitude > 1.0e-15 && vbound[k].Magnitude > 1.0e-15)
                {
                    double alpha = Math.Abs(vres[k].AngleTo(vbound[k]));
                    alpha = Min(alpha, Math.Abs(M_PI - alpha));
                    if (alpha > maxang)
                    {
                        maxang = alpha;
                    }
                }
            }
            if (pres[k].DistanceTo(pbound[k]) > maxdist)
            {
                maxdist = pres[k].DistanceTo(pbound[k]);
            }
            uu += duu;
            vv += dvv;
            ww += dww;
        }
        anOut.Write("Controle resultat/contrainte sur bord " + i.ToString(CultureInfo.InvariantCulture) + " : \n");
        anOut.Write("Distance max : " + Cout(maxdist) + "\n");
        if (donor)
        {
            double angdeg = maxang * 180.0 / M_PI;
            anOut.Write("Angle max    : " + Cout(angdeg) + " deg\n");
        }
    }
}
