// Port of: src/FoundationClasses/TKMath/math/math_FunctionRoots.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionRoots.lxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionRoots.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>All the real roots of a function with derivative within a given range.</summary>
/// <remarks>
/// OCCT compiles only its "new code" (MATH_FUNCTIONROOTS_NEWCODE); the disabled old algorithm
/// (MATH_FUNCTIONROOTS_OLDCODE) and the debug check (MATH_FUNCTIONROOTS_CHECK) are not ported.
/// </remarks>
internal class FunctionRoots
{
    private const int Itmax = 100;
    private const double Epseps = 2e-14;
    private bool _done;
    private bool _allNull;
    private readonly NCollection_Sequence<double> _sol = new();
    private readonly NCollection_Sequence<int> _nbStateSol = new();

    /// <summary>
    /// The derivative of a function, as a function (math_FunctionRoots.cxx's DerivFunction); upstream 71658f6c8f
    /// remembers a derivative that failed or wasn't finite.
    /// </summary>
    private sealed class DerivFunction : Function
    {
        private readonly FunctionWithDerivative _f;

        public DerivFunction(FunctionWithDerivative f)
        {
            _f = f;
        }

        public bool HasFailed { get; private set; }

        public override bool Value(double x, out double fval)
        {
            if (!_f.Derivative(x, out fval) || !double.IsFinite(fval))
            {
                HasFailed = true;
                return false;
            }

            return true;
        }
    }

    // Upstream 71658f6c8f: false when the function fails at the root, whose state it reads
    private static bool AppendRoot(NCollection_Sequence<double> sol, NCollection_Sequence<int> nbStateSol, double x,
                                   FunctionWithDerivative f, double dX)
    {
        int n = sol.Length();
        double t;

        if (n == 0)
        {
            if (!f.Value(x, out t) || !double.IsFinite(t))
            {
                return false;
            }

            sol.Append(x);
            nbStateSol.Append(f.GetStateNumber());
        }
        else
        {
            int i = 1;
            int pl = n + 1;
            while (i <= n)
            {
                t = sol.Value(i);
                if (t >= x)
                {
                    pl = i;
                    i = n;
                }

                if (Math.Abs(x - t) <= dX)
                {
                    pl = 0;
                    i = n;
                }

                i++;
            } //-- while

            if (pl > 0)
            {
                if (!f.Value(x, out t) || !double.IsFinite(t))
                {
                    return false;
                }

                if (pl > n)
                {
                    sol.Append(x);
                    nbStateSol.Append(f.GetStateNumber());
                }
                else
                {
                    sol.InsertBefore(pl, x);
                    nbStateSol.InsertBefore(pl, f.GetStateNumber());
                }
            }
        }

        return true;
    }

    // Upstream 71658f6c8f: false when an evaluation fails or isn't finite, or the search doesn't converge, where 8.0.1 went
    // on as if there were no root
    private static bool Solve(FunctionWithDerivative f, double k, double x1, double y1, double x2, double y2,
                              double tol, double dX, NCollection_Sequence<double> sol, NCollection_Sequence<int> nbStateSol)
    {
        int iter = 0;
        double tols2 = 0.5 * tol;
        double a, b, c, d = 0, e = 0, fa, fb, fc, p, q, r, s, tol1, xm, min1, min2;
        a = x1;
        b = c = x2;
        fa = y1;
        fb = fc = y2;
        for (iter = 1; iter <= Itmax; iter++)
        {
            if ((fb > 0.0 && fc > 0.0) || (fb < 0.0 && fc < 0.0))
            {
                c = a;
                fc = fa;
                e = d = b - a;
            }

            if (Math.Abs(fc) < Math.Abs(fb))
            {
                a = b;
                b = c;
                c = a;
                fa = fb;
                fb = fc;
                fc = fa;
            }

            tol1 = Epseps * Math.Abs(b) + tols2;
            xm = 0.5 * (c - b);
            if (Math.Abs(xm) < tol1 || fb == 0)
            {
                //-- On tente une iteration de newton
                double xp, yp, dp;
                int itern = 5;
                bool ok;
                xp = b;
                do
                {
                    if (!f.Values(xp, out yp, out dp) || !double.IsFinite(yp) || !double.IsFinite(dp))
                    {
                        return false;
                    }

                    ok = false;
                    if (dp > 1e-10 || dp < -1e-10)
                    {
                        xp = xp - (yp - k) / dp;
                    }

                    if (xp <= x2 && xp >= x1)
                    {
                        if (!f.Value(xp, out yp) || !double.IsFinite(yp))
                        {
                            return false;
                        }

                        yp -= k;
                        if (Math.Abs(yp) < Math.Abs(fb))
                        {
                            b = xp;
                            fb = yp;
                            ok = true;
                        }
                    }
                } while (ok && --itern >= 0);

                return AppendRoot(sol, nbStateSol, b, f, dX);
            }

            if (Math.Abs(e) >= tol1 && Math.Abs(fa) > Math.Abs(fb))
            {
                s = fb / fa;
                if (a == c)
                {
                    p = xm * s;
                    p += p;
                    q = 1.0 - s;
                }
                else
                {
                    q = fa / fc;
                    r = fb / fc;
                    p = s * ((xm + xm) * q * (q - r) - (b - a) * (r - 1.0));
                    q = (q - 1.0) * (r - 1.0) * (s - 1.0);
                }

                if (p > 0.0)
                {
                    q = -q;
                }

                p = Math.Abs(p);
                min1 = 3.0 * xm * q - Math.Abs(tol1 * q);
                min2 = Math.Abs(e * q);
                if ((p + p) < ((min1 < min2) ? min1 : min2))
                {
                    e = d;
                    d = p / q;
                }
                else
                {
                    d = xm;
                    e = d;
                }
            }
            else
            {
                d = xm;
                e = d;
            }

            a = b;
            fa = fb;
            if (Math.Abs(d) > tol1)
            {
                b += d;
            }
            else
            {
                if (xm >= 0)
                {
                    b += Math.Abs(tol1);
                }
                else
                {
                    b += -Math.Abs(tol1);
                }
            }

            if (!f.Value(b, out fb) || !double.IsFinite(fb))
            {
                return false;
            }

            fb -= k;
        }

        return false;
    }

    /// <summary>
    /// Computes all the real roots of F - K in the range A..B (in either order). A solution X is found when
    /// abs(Xi - Xi-1) &lt;= EpsX and abs(F(Xi) - K) &lt;= EpsF. The function is considered null between A and B if
    /// abs(F - K) &lt;= EpsNull within the range.
    /// </summary>
    public FunctionRoots(FunctionWithDerivative f, double a, double b, int nbSample,
                              double _EpsX = 0.0, double epsF = 0.0, double epsNull = 0.0, double k = 0.0)
    {
        _sol.Clear();
        _nbStateSol.Clear();
        {
            _done = true;
            double X0 = a;
            double xn = b;
            int n = nbSample;
            //-- ------------------------------------------------------------
            //-- Verifications de bas niveau
            if (b < a)
            {
                X0 = b;
                xn = a;
            }

            n *= 2;
            if (n < 20)
            {
                n = 20;
            }

            //--  On teste si EpsX est trop petit (ie : U+Nn*EpsX == U )
            double epsX = _EpsX;
            double deltaU = Math.Abs(X0) + Math.Abs(xn);
            double nEpsX = 0.0000000001 * deltaU;
            if (epsX < nEpsX)
            {
                epsX = nEpsX;
            }

            //-- recherche d un intervalle ou F(xi) et F(xj) sont de signes differents
            //-- A .............................................................. B
            //-- X0   X1   X2 ........................................  Xn-1      Xn
            int i;
            double x = X0;
            bool ok;
            double dx = (xn - X0) / n;
            // Kerf: on the stack when there are few, as nearly always (every search for an intersection or an extremum does this)
            Span<double> ptrval = n < 256 ? stackalloc double[n + 1] : new double[n + 1];
            int nvalid = -1;
            double aux = 0;
            for (i = 0; i <= n; i++, x += dx)
            {
                if (x > xn)
                {
                    x = xn;
                }

                ok = f.Value(x, out aux) && double.IsFinite(aux);
                if (ok)
                {
                    ptrval[++nvalid] = aux - k;
                }
            }

            //-- Toute la fonction est nulle ?

            if (nvalid < n)
            {
                _done = false;
                return;
            }

            _allNull = true;
            for (i = 0; _allNull && i <= n; i++)
            {
                if (ptrval[i] > epsNull || ptrval[i] < -epsNull)
                {
                    _allNull = false;
                }
            }

            if (_allNull)
            {
                //-- tous les points echantillons sont dans la tolerance
            }
            else
            {
                //-- Il y a des points hors tolerance
                //-- on detecte les changements de signes STRICTS
                int ip1;
                double tol = epsX;
                double X2;
                for (i = 0, ip1 = 1, x = X0; i < n; i++, ip1++, x += dx)
                {
                    X2 = x + dx;
                    if (X2 > xn)
                    {
                        X2 = xn;
                    }

                    if (ptrval[i] < 0.0)
                    {
                        if (ptrval[ip1] > 0.0)
                        {
                            //-- --------------------------------------------------
                            //-- changement de signe dans Xi Xi+1
                            if (!Solve(f, k, x, ptrval[i], X2, ptrval[ip1], tol, nEpsX, _sol, _nbStateSol))
                            {
                                _done = false;
                                return;
                            }
                        }
                    }
                    else
                    {
                        if (ptrval[ip1] < 0.0)
                        {
                            //-- --------------------------------------------------
                            //-- changement de signe dans Xi Xi+1
                            if (!Solve(f, k, x, ptrval[i], X2, ptrval[ip1], tol, nEpsX, _sol, _nbStateSol))
                            {
                                _done = false;
                                return;
                            }
                        }
                    }
                }

                //-- On detecte les cas ou la fct s annule sur des Xi et est
                //-- non nulle au voisinage de Xi
                // Upstream 71658f6c8f (decision 34702): a sample on which F is exactly K is a root unless both its
                // neighbours are too (inside a run of them). 8.0.1 meant to probe either side of the sample, but took
                // both probes at X + dx/2, and kept the sample only where F was off K there. Upstream's search for F's
                // return across K in the neighbouring intervals isn't ported: everywhere Kerf's tests and corpus reach
                // it, it fires only where F is rounding noise, and there it multiplies the roots (decision 34702).
                bool isNative = NativeSampledRoots.IsNative;
                for (i = 0; i <= n; i++)
                {
                    if (isNative && ptrval[i] == 0.0)
                    {
                        // 8.0.1's test, for the native recordings' tests only
                        x = Math.Min(X0 + i * dx, xn);
                        if (!f.Value(Math.Min(x + dx * 0.5, xn), out double aHalfStep) || !double.IsFinite(aHalfStep))
                        {
                            _done = false;
                            return;
                        }

                        if (aHalfStep - k != 0.0 && !AppendRoot(_sol, _nbStateSol, x, f, nEpsX))
                        {
                            _done = false;
                            return;
                        }

                        continue;
                    }

                    if (ptrval[i] != 0.0 || ((i == 0 || ptrval[i - 1] == 0.0) && (i == n || ptrval[i + 1] == 0.0)))
                    {
                        continue;
                    }

                    x = X0 + i * dx;
                    if (x > xn)
                    {
                        x = xn;
                    }

                    if (!AppendRoot(_sol, _nbStateSol, x, f, nEpsX))
                    {
                        _done = false;
                        return;
                    }
                }

                //-- --------------------------------------------------------------------------------
                //-- Il faut traiter differement le cas des points en bout :
                if (ptrval[0] <= epsF && ptrval[0] >= -epsF)
                {
                    if (!AppendRoot(_sol, _nbStateSol, X0, f, nEpsX))
                    {
                        _done = false;
                        return;
                    }
                }

                if (ptrval[n] <= epsF && ptrval[n] >= -epsF)
                {
                    if (!AppendRoot(_sol, _nbStateSol, xn, f, nEpsX))
                    {
                        _done = false;
                        return;
                    }
                }

                //-- --------------------------------------------------------------------------------
                //-- On detecte les zones ou on a sur les points echantillons un minimum avec f(x)>0
                //--                                                          un maximum avec f(x)<0
                //-- On reprend une discretisation plus fine au voisinage de ces extremums
                //--
                //-- Recherche d un minima positif
                double xm, ym, dym, xm1, xp1;
                double majdx = 5.0 * dx;
                bool rediscr;
                int im1 = 0;
                ip1 = 2;
                for (i = 1, xm = X0 + dx; i < n; xm += dx, i++, im1++, ip1++)
                {
                    rediscr = false;
                    if (xm > xn)
                    {
                        xm = xn;
                    }

                    if (ptrval[i] > 0.0)
                    {
                        if ((ptrval[im1] > ptrval[i]) && (ptrval[ip1] > ptrval[i]))
                        {
                            //-- Peut on traverser l axe Ox
                            //-- -------------- Estimation a partir de Xim1
                            xm1 = xm - dx;
                            if (xm1 < X0)
                            {
                                xm1 = X0;
                            }

                            if (!f.Values(xm1, out ym, out dym) || !double.IsFinite(ym) || !double.IsFinite(dym))
                            {
                                _done = false;
                                return;
                            }

                            ym -= k;
                            if (dym < -1e-10 || dym > 1e-10)
                            {                   // normalement dym < 0
                                double t = ym / dym; //-- t=xm-x* = (ym-0)/dym
                                if (t < majdx && t > -majdx)
                                {
                                    rediscr = true;
                                }
                            }

                            //-- -------------- Estimation a partir de Xip1
                            if (!rediscr)
                            {
                                xp1 = xm + dx;
                                if (xp1 > xn)
                                {
                                    xp1 = xn;
                                }

                                if (!f.Values(xp1, out ym, out dym) || !double.IsFinite(ym) || !double.IsFinite(dym))
                                {
                                    _done = false;
                                    return;
                                }

                                ym -= k;
                                if (dym < -1e-10 || dym > 1e-10)
                                {                   // normalement dym > 0
                                    double t = ym / dym; //-- t=xm-x* = (ym-0)/dym
                                    if (t < majdx && t > -majdx)
                                    {
                                        rediscr = true;
                                    }
                                }
                            }
                        }
                    }
                    else if (ptrval[i] < 0.0)
                    {
                        if ((ptrval[im1] < ptrval[i]) && (ptrval[ip1] < ptrval[i]))
                        {
                            //-- Peut on traverser l axe Ox
                            //-- -------------- Estimation a partir de Xim1
                            xm1 = xm - dx;
                            if (xm1 < X0)
                            {
                                xm1 = X0;
                            }

                            if (!f.Values(xm1, out ym, out dym) || !double.IsFinite(ym) || !double.IsFinite(dym))
                            {
                                _done = false;
                                return;
                            }

                            ym -= k;
                            if (dym > 1e-10 || dym < -1e-10)
                            {                   // normalement dym > 0
                                double t = ym / dym; //-- t=xm-x* = (ym-0)/dym
                                if (t < majdx && t > -majdx)
                                {
                                    rediscr = true;
                                }
                            }

                            //-- -------------- Estimation a partir de Xim1
                            if (!rediscr)
                            {
                                // OCCT repeats the estimation from Xi-1 here, where the positive branch above uses
                                // Xi+1 (xp1), so this second test never succeeds where the first failed.
                                xm1 = xm - dx;
                                if (xm1 < X0)
                                {
                                    xm1 = X0;
                                }

                                if (!f.Values(xm1, out ym, out dym) || !double.IsFinite(ym) || !double.IsFinite(dym))
                                {
                                    _done = false;
                                    return;
                                }

                                ym -= k;
                                if (dym > 1e-10 || dym < -1e-10)
                                {                   // normalement dym < 0
                                    double t = ym / dym; //-- t=xm-x* = (ym-0)/dym
                                    if (t < majdx && t > -majdx)
                                    {
                                        rediscr = true;
                                    }
                                }
                            }
                        }
                    }

                    if (rediscr)
                    {
                        double x0 = xm - dx;
                        double x3 = xm + dx;
                        if (x0 < X0)
                        {
                            x0 = X0;
                        }

                        if (x3 > xn)
                        {
                            x3 = xn;
                        }

                        double solX1 = 0.0, solX2 = 0.0;
                        double val1 = 0.0, val2 = 0.0;
                        double der1 = 0.0, der2 = 0.0;
                        bool isSol1 = false;
                        bool isSol2 = false;
                        //-- ----------------------------------------------------
                        //-- Find minimum of the function |F| between x0 and x3
                        //-- by searching for the zero of the function derivative
                        DerivFunction derF = new(f);
                        BracketedRoot br = new(derF, x0, x3, _EpsX);
                        if (derF.HasFailed)
                        {
                            _done = false;
                            return;
                        }

                        if (br.IsDone)
                        {
                            solX1 = br.Root();
                            if (!f.Value(solX1, out val1) || !double.IsFinite(val1))
                            {
                                _done = false;
                                return;
                            }

                            // Upstream 71658f6c8f: the distance from the target, where 8.0.1 took |F| and so found a root
                            // wherever F was near 0 whatever K
                            val1 = Math.Abs(val1 - k);
                            if (val1 < epsF)
                            {
                                isSol1 = true;
                                der1 = br.Value();
                            }
                        }

                        //-- --------------------------------------------------
                        //-- On recherche un extrema entre x0 et x3
                        //-- x1 et x2 sont tels que x0<x1<x2<x3
                        //-- et |f(x0)| > |f(x1)|   et |f(x3)| > |f(x2)|
                        //--
                        //-- En entree : a=xm-dx  b=xm c=xm+dx
                        double x1, x2, f0, f3;
                        double r = 0.61803399;
                        double c = 1.0 - r;
                        double tolCR = nEpsX * 10.0;
                        f0 = ptrval[im1];
                        f3 = ptrval[ip1];
                        bool recherche_minimum = (f0 > 0.0);

                        if (Math.Abs(x3 - xm) > Math.Abs(x0 - xm))
                        {
                            x1 = xm;
                            x2 = xm + c * (x3 - xm);
                        }
                        else
                        {
                            x2 = xm;
                            x1 = xm - c * (xm - x0);
                        }

                        double f1, f2;
                        if (!f.Value(x1, out f1) || !double.IsFinite(f1))
                        {
                            _done = false;
                            return;
                        }

                        f1 -= k;
                        if (!f.Value(x2, out f2) || !double.IsFinite(f2))
                        {
                            _done = false;
                            return;
                        }

                        f2 -= k;
                        double tolX = 0.001 * nEpsX;
                        while (Math.Abs(x3 - x0) > tolCR * (Math.Abs(x1) + Math.Abs(x2)) && (Math.Abs(x1 - x2) > tolX))
                        {
                            if (recherche_minimum)
                            {
                                if (f2 < f1)
                                {
                                    x0 = x1;
                                    x1 = x2;
                                    x2 = r * x1 + c * x3;
                                    f0 = f1;
                                    f1 = f2;
                                    if (!f.Value(x2, out f2) || !double.IsFinite(f2))
                                    {
                                        _done = false;
                                        return;
                                    }

                                    f2 -= k;
                                }
                                else
                                {
                                    x3 = x2;
                                    x2 = x1;
                                    x1 = r * x2 + c * x0;
                                    f3 = f2;
                                    f2 = f1;
                                    if (!f.Value(x1, out f1) || !double.IsFinite(f1))
                                    {
                                        _done = false;
                                        return;
                                    }

                                    f1 -= k;
                                }
                            }
                            else
                            {
                                if (f2 > f1)
                                {
                                    x0 = x1;
                                    x1 = x2;
                                    x2 = r * x1 + c * x3;
                                    f0 = f1;
                                    f1 = f2;
                                    if (!f.Value(x2, out f2) || !double.IsFinite(f2))
                                    {
                                        _done = false;
                                        return;
                                    }

                                    f2 -= k;
                                }
                                else
                                {
                                    x3 = x2;
                                    x2 = x1;
                                    x1 = r * x2 + c * x0;
                                    f3 = f2;
                                    f2 = f1;
                                    if (!f.Value(x1, out f1) || !double.IsFinite(f1))
                                    {
                                        _done = false;
                                        return;
                                    }

                                    f1 -= k;
                                }
                            }

                            //-- On ne fait pas que chercher des extremas. Il faut verifier
                            //-- si on ne tombe pas sur une racine
                            if (f1 * f0 < 0.0 && !Solve(f, k, x0, f0, x1, f1, tol, nEpsX, _sol, _nbStateSol))
                            {
                                _done = false;
                                return;
                            }

                            if (f2 * f3 < 0.0 && !Solve(f, k, x2, f2, x3, f3, tol, nEpsX, _sol, _nbStateSol))
                            {
                                _done = false;
                                return;
                            }
                        }

                        if ((recherche_minimum && f1 < f2) || (!recherche_minimum && f1 > f2))
                        {
                            //-- x1,f(x1) minimum
                            if (Math.Abs(f1) < epsF)
                            {
                                isSol2 = true;
                                solX2 = x1;
                                val2 = Math.Abs(f1);
                            }
                        }
                        else
                        {
                            //-- x2.f(x2) minimum
                            if (Math.Abs(f2) < epsF)
                            {
                                isSol2 = true;
                                solX2 = x2;
                                val2 = Math.Abs(f2);
                            }
                        }

                        // Choose the best solution between aSolX1, aSolX2
                        if (isSol1 && isSol2)
                        {
                            if (val2 - val1 > epsF)
                            {
                                if (!AppendRoot(_sol, _nbStateSol, solX1, f, nEpsX))
                                {
                                    _done = false;
                                    return;
                                }
                            }
                            else if (val1 - val2 > epsF)
                            {
                                if (!AppendRoot(_sol, _nbStateSol, solX2, f, nEpsX))
                                {
                                    _done = false;
                                    return;
                                }
                            }
                            else
                            {
                                der1 = Math.Abs(der1);
                                if (!f.Derivative(solX2, out der2) || !double.IsFinite(der2))
                                {
                                    _done = false;
                                    return;
                                }

                                der2 = Math.Abs(der2);
                                if (der1 < der2)
                                {
                                    if (!AppendRoot(_sol, _nbStateSol, solX1, f, nEpsX))
                                    {
                                        _done = false;
                                        return;
                                    }
                                }
                                else
                                {
                                    if (!AppendRoot(_sol, _nbStateSol, solX2, f, nEpsX))
                                    {
                                        _done = false;
                                        return;
                                    }
                                }
                            }
                        }
                        else if (isSol1)
                        {
                            if (!AppendRoot(_sol, _nbStateSol, solX1, f, nEpsX))
                            {
                                _done = false;
                                return;
                            }
                        }
                        else if (isSol2)
                        {
                            if (!AppendRoot(_sol, _nbStateSol, solX2, f, nEpsX))
                            {
                                _done = false;
                                return;
                            }
                        }
                    } //-- Recherche d un extrema
                } //-- for
            }
        }
    }

    public bool IsDone => _done;

    /// <summary>True if the function is considered null between A and B. Throws <see cref="NotDoneException"/> if the algorithm failed.</summary>
    public bool IsAllNull
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, " ");
            return _allNull;
        }
    }

    /// <summary>The number of solutions. Throws <see cref="NotDoneException"/> if the algorithm failed.</summary>
    public int SolutionCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, " ");
            return _sol.Length();
        }
    }

    /// <summary>The root number <paramref name="nieme"/>. Throws <see cref="NotDoneException"/> if the algorithm failed.</summary>
    public double Value(int nieme)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _sol.Value(nieme);
    }

    /// <summary>The state number of the root number <paramref name="nieme"/>.</summary>
    public int StateNumber(int nieme)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _nbStateSol.Value(nieme);
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_FunctionRoots ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Number of solutions = " + _sol.Length() + '\n');
            for (int i = 1; i <= _sol.Length(); i++)
            {
                o.Write(" Solution Number " + i + "= " + _sol.Value(i).ToString(CultureInfo.InvariantCulture) + '\n');
            }
        }
        else
        {
            o.Write(" Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
