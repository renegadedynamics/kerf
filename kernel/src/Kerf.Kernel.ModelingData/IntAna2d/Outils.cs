// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_Outils.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_Outils.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>IntAna2d's private tools (free functions and a class of IntAna2d_Outils.hxx).</summary>
internal static class Outils
{
    /// <summary>
    /// The real roots of a polynomial of degree 4 or 2, with extra roots from the lower-degree polynomials when the
    /// complete one looks unreliable, sorted by the residual and filtered.
    /// </summary>
    internal sealed class MyDirectPolynomialRoots
    {
        private readonly double[] _sol = new double[16];
        private readonly double[] _val = new double[16];
        private readonly int _nbsol;
        private readonly bool _same;

        public MyDirectPolynomialRoots(double a4, double a3, double a2, double a1, double a0)
        {
            _nbsol = 0;
            _same = false;
            //  Modified by Sergey KHROMOV - Thu Oct 24 13:10:14 2002 Begin
            Span<double> aa = stackalloc double[5];

            aa[0] = Math.Abs(a0);
            aa[1] = Math.Abs(a1);
            aa[2] = Math.Abs(a2);
            aa[3] = Math.Abs(a3);
            aa[4] = Math.Abs(a4);

            if ((aa[0] + aa[1] + aa[2] + aa[3] + aa[4]) < Epsilon(10000.0))
            {
                //  Modified by Sergey KHROMOV - Thu Oct 24 13:10:15 2002 End
                _same = true;
                return;
            }
            int i, j, nbp;
            for (int idx = 0; idx < _val.Length; idx++)
            {
                _val[idx] = RealLast();
                _sol[idx] = RealLast();
            }

            double tol = Epsilon(100.0);
            DirectPolynomialRoots MATH_A43210 = new(a4, a3, a2, a1, a0);
            bool pbPossible = false;
            int nbsolPolyComplet = 0;
            if (MATH_A43210.IsDone)
            {
                nbp = MATH_A43210.SolutionCount;
                nbsolPolyComplet = nbp;
                for (i = 1; i <= nbp; i++)
                {
                    double x = MATH_A43210.Value(i);
                    _val[_nbsol] = a0 + x * (a1 + x * (a2 + x * (a3 + x * a4)));
                    _sol[_nbsol] = x;
                    if (_val[_nbsol] > tol || _val[_nbsol] < -tol)
                    {
                        pbPossible = true;
                    }
                    _nbsol++;
                }
                if ((nbp & 1) != 0)
                {
                    pbPossible = true;
                }
            }
            else
            {
                pbPossible = true;
            }
            //-- Search for the smallest coefficient between A4 and A0
            if (pbPossible)
            {
                //  Modified by Sergey KHROMOV - Thu Oct 24 12:45:35 2002 Begin
                double aMin = RealLast();
                double aMax = -1;
                double eps = RealEpsilon();

                for (i = 0; i < 5; i++)
                {
                    aMin = Min(aMin, Max(aa[i], eps));
                    aMax = Max(aMax, Max(aa[i], eps));
                }

                eps = Min(1.0e-4, Epsilon(1000.0 * aMax / aMin));
                //  Modified by Sergey KHROMOV - Thu Oct 24 15:46:24 2002 End
                DirectPolynomialRoots MATH_A4321 = new(a4, a3, a2, a1);
                if (MATH_A4321.IsDone)
                {
                    nbp = MATH_A4321.SolutionCount;
                    //-- Add the values to the array
                    for (i = 1; i <= nbp; i++)
                    {
                        double x = MATH_A4321.Value(i);
                        bool add = true;
                        for (j = 0; j < _nbsol; j++)
                        {
                            double t = _sol[j] - x;
                            if (Math.Abs(t) < eps)
                            {
                                add = false;
                            }
                        }
                        if (add)
                        {
                            _val[_nbsol] = a0 + x * (a1 + x * (a2 + x * (a3 + x * a4)));
                            _sol[_nbsol] = x;
                            _nbsol++;
                        }
                    }
                }
                DirectPolynomialRoots MATH_A3210 = new(a3, a2, a1, a0);
                if (MATH_A3210.IsDone)
                {
                    nbp = MATH_A3210.SolutionCount;
                    //-- Add the values to the array
                    for (i = 1; i <= nbp; i++)
                    {
                        double x = MATH_A3210.Value(i);
                        bool add = true;
                        for (j = 0; j < _nbsol; j++)
                        {
                            double t = _sol[j] - x;
                            if (Math.Abs(t) < eps)
                            {
                                add = false;
                            }
                        }
                        if (add)
                        {
                            _val[_nbsol] = a0 + x * (a1 + x * (a2 + x * (a3 + x * a4)));
                            _sol[_nbsol] = x;
                            _nbsol++;
                        }
                    }
                }
                // OCCT: named A210 but solves A3 x^2 + A2 x + A1 (probably meant A2, A1, A0)
                DirectPolynomialRoots MATH_A210 = new(a3, a2, a1);
                if (MATH_A210.IsDone)
                {
                    nbp = MATH_A210.SolutionCount;
                    //-- Add the values to the array
                    for (i = 1; i <= nbp; i++)
                    {
                        double x = MATH_A210.Value(i);
                        bool add = true;
                        for (j = 0; j < _nbsol; j++)
                        {
                            double t = _sol[j] - x;
                            if (Math.Abs(t) < eps)
                            {
                                add = false;
                            }
                        }
                        if (add)
                        {
                            _val[_nbsol] = a0 + x * (a1 + x * (a2 + x * (a3 + x * a4)));
                            _sol[_nbsol] = x;
                            _nbsol++;
                        }
                    }
                }
                //------------------------------------------------------------
                //-- Sort values by decreasing order of val
                bool triOK = false;
                do
                {
                    triOK = true;
                    for (i = 1; i < _nbsol; i++)
                    {
                        if (Math.Abs(_val[i]) < Math.Abs(_val[i - 1]))
                        {
                            double t;
                            t = _val[i];
                            _val[i] = _val[i - 1];
                            _val[i - 1] = t;
                            t = _sol[i];
                            _sol[i] = _sol[i - 1];
                            _sol[i - 1] = t;
                            triOK = false;
                        }
                    }
                } while (!triOK);
                //-----------------------------------------------------------
                //-- Keep the first values
                //-- At least as many as the complete polynomial
                //--
                for (_nbsol = 0; _nbsol < nbsolPolyComplet || Math.Abs(_val[_nbsol]) < Epsilon(10000.0); _nbsol++)
                {
                }
            }
            if (_nbsol == 0)
            {
                _nbsol = -1;
            }
            if (_nbsol > 4)
            {
                _same = true;
                _nbsol = 0;
            }
        }

        public MyDirectPolynomialRoots(double a2, double a1, double a0)
        {
            for (int idx = 0; idx < _val.Length; idx++)
            {
                _val[idx] = RealLast();
                _sol[idx] = RealLast();
            }
            _nbsol = 0;
            if ((Math.Abs(a2) + Math.Abs(a1) + Math.Abs(a0)) < Epsilon(10000.0))
            {
                _same = true;
                return;
            }
            DirectPolynomialRoots MATH_A210 = new(a2, a1, a0);
            _same = false;
            if (MATH_A210.IsDone)
            {
                for (int i = 1; i <= MATH_A210.SolutionCount; i++)
                {
                    double x = MATH_A210.Value(i);
                    _val[_nbsol] = a0 + x * (a1 + x * a2);
                    _sol[_nbsol] = x;
                    _nbsol++;
                }
            }
            else
            {
                _nbsol = -1;
            }
        }

        public int SolutionCount => _nbsol;

        public double Value(int i) => _sol[i - 1];

        // OCCT returns this bool as a double
        public bool IsDone => _nbsol > -1;

        public bool InfiniteRoots() => _same;
    }

    internal static bool Points_Confondus(double x1, double y1, double x2, double y2)
    {
        if (Math.Abs(x1 - x2) < Epsilon(x1))
        {
            if (Math.Abs(y1 - y2) < Epsilon(y1))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>Removes the points that are equal to an earlier one, keeping only their parameter on the first curve.</summary>
    internal static void Traitement_Points_Confondus(ref int nb_pts, IntPoint[] pts)
    {
        int i, j;
        for (i = nb_pts; i > 1; i--)
        {
            bool Non_Egalite = true;
            for (j = i - 1; (j > 0) && Non_Egalite; j--)
            {
                //                                        <--- Already Tested --->
                //             | 1  |2  |  | J |  |I-1| I |I+1|          |NPTS|
                //             | 1  |2  |  | J |  |I-1|XXX|I+1|          |NPTS|
                //             | 1  |2  |  | J |  |I-1|I+1|I+2|     |NPTS|
                if (Points_Confondus(pts[i - 1].Value().X,
                                     pts[i - 1].Value().Y,
                                     pts[j - 1].Value().X,
                                     pts[j - 1].Value().Y))
                {
                    int k;
                    Non_Egalite = false;
                    for (k = i; k < nb_pts; k++)
                    {
                        double xk, yk, uk;
                        xk = pts[k].Value().X;
                        yk = pts[k].Value().Y;
                        uk = pts[k].ParamOnFirst();
                        pts[k - 1].SetValue(xk, yk, uk);
                    }
                    nb_pts--;
                }
            }
        }
    }

    /// <summary>
    /// Converts x1, y1, coordinates in the system of <paramref name="dir1"/>, into the "absolute" coordinate system.
    /// </summary>
    internal static void Coord_Ancien_Repere(ref double x1, ref double y1, in Axis2D dir1)
    {
        double t11, t12, t21, t22, t13, t23;
        double x0, y0;

        // x1 and y1 are the coordinates in the reference frame associated with Dir1
        // We return these coordinates in the "absolute" reference frame

        dir1.Direction.Coord(out t11, out t21);
        dir1.Location.Coord(out t13, out t23);

        t22 = t11;
        t12 = -t21;

        x0 = t11 * x1 + t12 * y1 + t13;
        y0 = t21 * x1 + t22 * y1 + t23;

        x1 = x0;
        y1 = y0;
    }
}
