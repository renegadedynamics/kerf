// Port of: src/FoundationClasses/TKMath/math/math_FunctionRoots.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionRoots.lxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionRoots.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>All the real roots of a function with derivative within a given range.</summary>
/// <remarks>
/// OCCT compiles only its "new code" (MATH_FUNCTIONROOTS_NEWCODE); the disabled old algorithm
/// (MATH_FUNCTIONROOTS_OLDCODE) and the debug check (MATH_FUNCTIONROOTS_CHECK) are not ported.
/// </remarks>
public class math_FunctionRoots
{
    private const int ITMAX = 100;
    private const double EPS = 1e-14;
    private const double EPSEPS = 2e-14;
    private const int MAXBIS = 100;

    private bool Done;
    private bool AllNull;
    private readonly NCollection_Sequence<double> Sol = new();
    private readonly NCollection_Sequence<int> NbStateSol = new();

    /// <summary>The derivative of a function, as a function (math_FunctionRoots.cxx's DerivFunction).</summary>
    private sealed class DerivFunction : math_Function
    {
        private readonly math_FunctionWithDerivative myF;

        public DerivFunction(math_FunctionWithDerivative theF)
        {
            myF = theF;
        }

        public override bool Value(double theX, out double theFval) => myF.Derivative(theX, out theFval);
    }

    private static void AppendRoot(NCollection_Sequence<double> Sol, NCollection_Sequence<int> NbStateSol, double X,
                                   math_FunctionWithDerivative F, double K, double dX)
    {
        int n = Sol.Length();
        double t;

        if (n == 0)
        {
            Sol.Append(X);
            F.Value(X, out t);
            NbStateSol.Append(F.GetStateNumber());
        }
        else
        {
            int i = 1;
            int pl = n + 1;
            while (i <= n)
            {
                t = Sol.Value(i);
                if (t >= X)
                {
                    pl = i;
                    i = n;
                }

                if (Math.Abs(X - t) <= dX)
                {
                    pl = 0;
                    i = n;
                }

                i++;
            } //-- while

            if (pl > n)
            {
                Sol.Append(X);
                F.Value(X, out t);
                NbStateSol.Append(F.GetStateNumber());
            }
            else if (pl > 0)
            {
                Sol.InsertBefore(pl, X);
                F.Value(X, out t);
                NbStateSol.InsertBefore(pl, F.GetStateNumber());
            }
        }
    }

    private static void Solve(math_FunctionWithDerivative F, double K, double x1, double y1, double x2, double y2,
                              double tol, double dX, NCollection_Sequence<double> Sol, NCollection_Sequence<int> NbStateSol)
    {
        int iter = 0;
        double tols2 = 0.5 * tol;
        double a, b, c, d = 0, e = 0, fa, fb, fc, p, q, r, s, tol1, xm, min1, min2;
        a = x1;
        b = c = x2;
        fa = y1;
        fb = fc = y2;
        for (iter = 1; iter <= ITMAX; iter++)
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

            tol1 = EPSEPS * Math.Abs(b) + tols2;
            xm = 0.5 * (c - b);
            if (Math.Abs(xm) < tol1 || fb == 0)
            {
                //-- On tente une iteration de newton
                double Xp, Yp, Dp;
                int itern = 5;
                bool Ok;
                Xp = b;
                do
                {
                    Ok = F.Values(Xp, out Yp, out Dp);
                    if (Ok)
                    {
                        Ok = false;
                        if (Dp > 1e-10 || Dp < -1e-10)
                        {
                            Xp = Xp - (Yp - K) / Dp;
                        }

                        if (Xp <= x2 && Xp >= x1)
                        {
                            F.Value(Xp, out Yp);
                            Yp -= K;
                            if (Math.Abs(Yp) < Math.Abs(fb))
                            {
                                b = Xp;
                                fb = Yp;
                                Ok = true;
                            }
                        }
                    }
                } while (Ok && --itern >= 0);

                AppendRoot(Sol, NbStateSol, b, F, K, dX);
                return;
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

            F.Value(b, out fb);
            fb -= K;
        }
    }

    /// <summary>
    /// Computes all the real roots of F - K in the range A..B (in either order). A solution X is found when
    /// abs(Xi - Xi-1) &lt;= EpsX and abs(F(Xi) - K) &lt;= EpsF. The function is considered null between A and B if
    /// abs(F - K) &lt;= EpsNull within the range.
    /// </summary>
    public math_FunctionRoots(math_FunctionWithDerivative F, double A, double B, int NbSample,
                              double _EpsX = 0.0, double EpsF = 0.0, double EpsNull = 0.0, double K = 0.0)
    {
        Sol.Clear();
        NbStateSol.Clear();
        {
            Done = true;
            double X0 = A;
            double XN = B;
            int N = NbSample;
            //-- ------------------------------------------------------------
            //-- Verifications de bas niveau
            if (B < A)
            {
                X0 = B;
                XN = A;
            }

            N *= 2;
            if (N < 20)
            {
                N = 20;
            }

            //--  On teste si EpsX est trop petit (ie : U+Nn*EpsX == U )
            double EpsX = _EpsX;
            double DeltaU = Math.Abs(X0) + Math.Abs(XN);
            double NEpsX = 0.0000000001 * DeltaU;
            if (EpsX < NEpsX)
            {
                EpsX = NEpsX;
            }

            //-- recherche d un intervalle ou F(xi) et F(xj) sont de signes differents
            //-- A .............................................................. B
            //-- X0   X1   X2 ........................................  Xn-1      Xn
            int i;
            double X = X0;
            bool Ok;
            double dx = (XN - X0) / N;
            NCollection_Array1<double> ptrval = new(0, N);
            int Nvalid = -1;
            double aux = 0;
            for (i = 0; i <= N; i++, X += dx)
            {
                if (X > XN)
                {
                    X = XN;
                }

                Ok = F.Value(X, out aux);
                if (Ok)
                {
                    ptrval[++Nvalid] = aux - K;
                }
            }

            //-- Toute la fonction est nulle ?

            if (Nvalid < N)
            {
                Done = false;
                return;
            }

            AllNull = true;
            for (i = 0; AllNull && i <= N; i++)
            {
                if (ptrval[i] > EpsNull || ptrval[i] < -EpsNull)
                {
                    AllNull = false;
                }
            }

            if (AllNull)
            {
                //-- tous les points echantillons sont dans la tolerance
            }
            else
            {
                //-- Il y a des points hors tolerance
                //-- on detecte les changements de signes STRICTS
                int ip1;
                double tol = EpsX;
                double X2;
                for (i = 0, ip1 = 1, X = X0; i < N; i++, ip1++, X += dx)
                {
                    X2 = X + dx;
                    if (X2 > XN)
                    {
                        X2 = XN;
                    }

                    if (ptrval[i] < 0.0)
                    {
                        if (ptrval[ip1] > 0.0)
                        {
                            //-- --------------------------------------------------
                            //-- changement de signe dans Xi Xi+1
                            Solve(F, K, X, ptrval[i], X2, ptrval[ip1], tol, NEpsX, Sol, NbStateSol);
                        }
                    }
                    else
                    {
                        if (ptrval[ip1] < 0.0)
                        {
                            //-- --------------------------------------------------
                            //-- changement de signe dans Xi Xi+1
                            Solve(F, K, X, ptrval[i], X2, ptrval[ip1], tol, NEpsX, Sol, NbStateSol);
                        }
                    }
                }

                //-- On detecte les cas ou la fct s annule sur des Xi et est
                //-- non nulle au voisinage de Xi
                //--
                //-- On prend 2 points u0,u1 au voisinage de Xi
                //-- Si (F(u0)-K)*(F(u1)-K) <0   on lance une recherche
                //-- Sinon si (F(u0)-K)*(F(u1)-K) !=0 on insere le point X
                for (i = 0; i <= N; i++)
                {
                    if (ptrval[i] == 0)
                    {
                        X = X0 + i * dx;
                        if (X > XN)
                        {
                            X = XN;
                        }

                        double u0, u1;
                        u0 = dx * 0.5;
                        u1 = X + u0;
                        u0 += X;
                        if (u0 < X0)
                        {
                            u0 = X0;
                        }

                        if (u0 > XN)
                        {
                            u0 = XN;
                        }

                        if (u1 < X0)
                        {
                            u1 = X0;
                        }

                        if (u1 > XN)
                        {
                            u1 = XN;
                        }

                        double y0, y1;
                        F.Value(u0, out y0);
                        y0 -= K;
                        F.Value(u1, out y1);
                        y1 -= K;
                        if (y0 * y1 < 0.0)
                        {
                            Solve(F, K, u0, y0, u1, y1, tol, NEpsX, Sol, NbStateSol);
                        }
                        else
                        {
                            if (y0 != 0.0 || y1 != 0.0)
                            {
                                AppendRoot(Sol, NbStateSol, X, F, K, NEpsX);
                            }
                        }
                    }
                }

                //-- --------------------------------------------------------------------------------
                //-- Il faut traiter differement le cas des points en bout :
                if (ptrval[0] <= EpsF && ptrval[0] >= -EpsF)
                {
                    AppendRoot(Sol, NbStateSol, X0, F, K, NEpsX);
                }

                if (ptrval[N] <= EpsF && ptrval[N] >= -EpsF)
                {
                    AppendRoot(Sol, NbStateSol, XN, F, K, NEpsX);
                }

                //-- --------------------------------------------------------------------------------
                //-- On detecte les zones ou on a sur les points echantillons un minimum avec f(x)>0
                //--                                                          un maximum avec f(x)<0
                //-- On reprend une discretisation plus fine au voisinage de ces extremums
                //--
                //-- Recherche d un minima positif
                double xm, ym, dym, xm1, xp1;
                double majdx = 5.0 * dx;
                bool Rediscr;
                int im1 = 0;
                ip1 = 2;
                for (i = 1, xm = X0 + dx; i < N; xm += dx, i++, im1++, ip1++)
                {
                    Rediscr = false;
                    if (xm > XN)
                    {
                        xm = XN;
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

                            F.Values(xm1, out ym, out dym);
                            ym -= K;
                            if (dym < -1e-10 || dym > 1e-10)
                            {                   // normalement dym < 0
                                double t = ym / dym; //-- t=xm-x* = (ym-0)/dym
                                if (t < majdx && t > -majdx)
                                {
                                    Rediscr = true;
                                }
                            }

                            //-- -------------- Estimation a partir de Xip1
                            if (!Rediscr)
                            {
                                xp1 = xm + dx;
                                if (xp1 > XN)
                                {
                                    xp1 = XN;
                                }

                                F.Values(xp1, out ym, out dym);
                                ym -= K;
                                if (dym < -1e-10 || dym > 1e-10)
                                {                   // normalement dym > 0
                                    double t = ym / dym; //-- t=xm-x* = (ym-0)/dym
                                    if (t < majdx && t > -majdx)
                                    {
                                        Rediscr = true;
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

                            F.Values(xm1, out ym, out dym);
                            ym -= K;
                            if (dym > 1e-10 || dym < -1e-10)
                            {                   // normalement dym > 0
                                double t = ym / dym; //-- t=xm-x* = (ym-0)/dym
                                if (t < majdx && t > -majdx)
                                {
                                    Rediscr = true;
                                }
                            }

                            //-- -------------- Estimation a partir de Xim1
                            if (!Rediscr)
                            {
                                // OCCT repeats the estimation from Xi-1 here, where the positive branch above uses
                                // Xi+1 (xp1), so this second test never succeeds where the first failed.
                                xm1 = xm - dx;
                                if (xm1 < X0)
                                {
                                    xm1 = X0;
                                }

                                F.Values(xm1, out ym, out dym);
                                ym -= K;
                                if (dym > 1e-10 || dym < -1e-10)
                                {                   // normalement dym < 0
                                    double t = ym / dym; //-- t=xm-x* = (ym-0)/dym
                                    if (t < majdx && t > -majdx)
                                    {
                                        Rediscr = true;
                                    }
                                }
                            }
                        }
                    }

                    if (Rediscr)
                    {
                        double x0 = xm - dx;
                        double x3 = xm + dx;
                        if (x0 < X0)
                        {
                            x0 = X0;
                        }

                        if (x3 > XN)
                        {
                            x3 = XN;
                        }

                        double aSolX1 = 0.0, aSolX2 = 0.0;
                        double aVal1 = 0.0, aVal2 = 0.0;
                        double aDer1 = 0.0, aDer2 = 0.0;
                        bool isSol1 = false;
                        bool isSol2 = false;
                        //-- ----------------------------------------------------
                        //-- Find minimum of the function |F| between x0 and x3
                        //-- by searching for the zero of the function derivative
                        DerivFunction aDerF = new(F);
                        math_BracketedRoot aBR = new(aDerF, x0, x3, _EpsX);
                        if (aBR.IsDone())
                        {
                            aSolX1 = aBR.Root();
                            F.Value(aSolX1, out aVal1);
                            aVal1 = Math.Abs(aVal1);
                            if (aVal1 < EpsF)
                            {
                                isSol1 = true;
                                aDer1 = aBR.Value();
                            }
                        }

                        //-- --------------------------------------------------
                        //-- On recherche un extrema entre x0 et x3
                        //-- x1 et x2 sont tels que x0<x1<x2<x3
                        //-- et |f(x0)| > |f(x1)|   et |f(x3)| > |f(x2)|
                        //--
                        //-- En entree : a=xm-dx  b=xm c=xm+dx
                        double x1, x2, f0, f3;
                        double R = 0.61803399;
                        double C = 1.0 - R;
                        double tolCR = NEpsX * 10.0;
                        f0 = ptrval[im1];
                        f3 = ptrval[ip1];
                        bool recherche_minimum = (f0 > 0.0);

                        if (Math.Abs(x3 - xm) > Math.Abs(x0 - xm))
                        {
                            x1 = xm;
                            x2 = xm + C * (x3 - xm);
                        }
                        else
                        {
                            x2 = xm;
                            x1 = xm - C * (xm - x0);
                        }

                        double f1, f2;
                        F.Value(x1, out f1);
                        f1 -= K;
                        F.Value(x2, out f2);
                        f2 -= K;
                        double tolX = 0.001 * NEpsX;
                        while (Math.Abs(x3 - x0) > tolCR * (Math.Abs(x1) + Math.Abs(x2)) && (Math.Abs(x1 - x2) > tolX))
                        {
                            if (recherche_minimum)
                            {
                                if (f2 < f1)
                                {
                                    x0 = x1;
                                    x1 = x2;
                                    x2 = R * x1 + C * x3;
                                    f0 = f1;
                                    f1 = f2;
                                    F.Value(x2, out f2);
                                    f2 -= K;
                                }
                                else
                                {
                                    x3 = x2;
                                    x2 = x1;
                                    x1 = R * x2 + C * x0;
                                    f3 = f2;
                                    f2 = f1;
                                    F.Value(x1, out f1);
                                    f1 -= K;
                                }
                            }
                            else
                            {
                                if (f2 > f1)
                                {
                                    x0 = x1;
                                    x1 = x2;
                                    x2 = R * x1 + C * x3;
                                    f0 = f1;
                                    f1 = f2;
                                    F.Value(x2, out f2);
                                    f2 -= K;
                                }
                                else
                                {
                                    x3 = x2;
                                    x2 = x1;
                                    x1 = R * x2 + C * x0;
                                    f3 = f2;
                                    f2 = f1;
                                    F.Value(x1, out f1);
                                    f1 -= K;
                                }
                            }

                            //-- On ne fait pas que chercher des extremas. Il faut verifier
                            //-- si on ne tombe pas sur une racine
                            if (f1 * f0 < 0.0)
                            {
                                Solve(F, K, x0, f0, x1, f1, tol, NEpsX, Sol, NbStateSol);
                            }

                            if (f2 * f3 < 0.0)
                            {
                                Solve(F, K, x2, f2, x3, f3, tol, NEpsX, Sol, NbStateSol);
                            }
                        }

                        if ((recherche_minimum && f1 < f2) || (!recherche_minimum && f1 > f2))
                        {
                            //-- x1,f(x1) minimum
                            if (Math.Abs(f1) < EpsF)
                            {
                                isSol2 = true;
                                aSolX2 = x1;
                                aVal2 = Math.Abs(f1);
                            }
                        }
                        else
                        {
                            //-- x2.f(x2) minimum
                            if (Math.Abs(f2) < EpsF)
                            {
                                isSol2 = true;
                                aSolX2 = x2;
                                aVal2 = Math.Abs(f2);
                            }
                        }

                        // Choose the best solution between aSolX1, aSolX2
                        if (isSol1 && isSol2)
                        {
                            if (aVal2 - aVal1 > EpsF)
                            {
                                AppendRoot(Sol, NbStateSol, aSolX1, F, K, NEpsX);
                            }
                            else if (aVal1 - aVal2 > EpsF)
                            {
                                AppendRoot(Sol, NbStateSol, aSolX2, F, K, NEpsX);
                            }
                            else
                            {
                                aDer1 = Math.Abs(aDer1);
                                F.Derivative(aSolX2, out aDer2);
                                aDer2 = Math.Abs(aDer2);
                                if (aDer1 < aDer2)
                                {
                                    AppendRoot(Sol, NbStateSol, aSolX1, F, K, NEpsX);
                                }
                                else
                                {
                                    AppendRoot(Sol, NbStateSol, aSolX2, F, K, NEpsX);
                                }
                            }
                        }
                        else if (isSol1)
                        {
                            AppendRoot(Sol, NbStateSol, aSolX1, F, K, NEpsX);
                        }
                        else if (isSol2)
                        {
                            AppendRoot(Sol, NbStateSol, aSolX2, F, K, NEpsX);
                        }
                    } //-- Recherche d un extrema
                } //-- for
            }
        }
    }

    public bool IsDone() => Done;

    /// <summary>True if the function is considered null between A and B. Throws <see cref="StdFail_NotDone"/> if the algorithm failed.</summary>
    public bool IsAllNull()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return AllNull;
    }

    /// <summary>The number of solutions. Throws <see cref="StdFail_NotDone"/> if the algorithm failed.</summary>
    public int NbSolutions()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Sol.Length();
    }

    /// <summary>The root number <paramref name="Nieme"/>. Throws <see cref="StdFail_NotDone"/> if the algorithm failed.</summary>
    public double Value(int Nieme)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Sol.Value(Nieme);
    }

    /// <summary>The state number of the root number <paramref name="Nieme"/>.</summary>
    public int StateNumber(int Nieme)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return NbStateSol.Value(Nieme);
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_FunctionRoots ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Number of solutions = " + Sol.Length() + '\n');
            for (int i = 1; i <= Sol.Length(); i++)
            {
                o.Write(" Solution Number " + i + "= " + Sol.Value(i).ToString(CultureInfo.InvariantCulture) + '\n');
            }
        }
        else
        {
            o.Write(" Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
