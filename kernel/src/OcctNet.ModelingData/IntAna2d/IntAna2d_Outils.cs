// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_Outils.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_Outils.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>IntAna2d's private tools (free functions and a class of IntAna2d_Outils.hxx).</summary>
internal static class IntAna2d_Outils
{
    /// <summary>
    /// The real roots of a polynomial of degree 4 or 2, with extra roots from the lower-degree polynomials when the
    /// complete one looks unreliable, sorted by the residual and filtered.
    /// </summary>
    internal sealed class MyDirectPolynomialRoots
    {
        private readonly double[] sol = new double[16];
        private readonly double[] val = new double[16];
        private readonly int nbsol;
        private readonly bool same;

        public MyDirectPolynomialRoots(double A4, double A3, double A2, double A1, double A0)
        {
            nbsol = 0;
            same = false;
            //  Modified by Sergey KHROMOV - Thu Oct 24 13:10:14 2002 Begin
            Span<double> anAA = stackalloc double[5];

            anAA[0] = Math.Abs(A0);
            anAA[1] = Math.Abs(A1);
            anAA[2] = Math.Abs(A2);
            anAA[3] = Math.Abs(A3);
            anAA[4] = Math.Abs(A4);

            if ((anAA[0] + anAA[1] + anAA[2] + anAA[3] + anAA[4]) < Epsilon(10000.0))
            {
                //  Modified by Sergey KHROMOV - Thu Oct 24 13:10:15 2002 End
                same = true;
                return;
            }
            int i, j, nbp;
            for (int anIdx = 0; anIdx < val.Length; anIdx++)
            {
                val[anIdx] = RealLast();
                sol[anIdx] = RealLast();
            }

            double tol = Epsilon(100.0);
            math_DirectPolynomialRoots MATH_A43210 = new(A4, A3, A2, A1, A0);
            bool PbPossible = false;
            int NbsolPolyComplet = 0;
            if (MATH_A43210.IsDone())
            {
                nbp = MATH_A43210.NbSolutions();
                NbsolPolyComplet = nbp;
                for (i = 1; i <= nbp; i++)
                {
                    double x = MATH_A43210.Value(i);
                    val[nbsol] = A0 + x * (A1 + x * (A2 + x * (A3 + x * A4)));
                    sol[nbsol] = x;
                    if (val[nbsol] > tol || val[nbsol] < -tol)
                    {
                        PbPossible = true;
                    }
                    nbsol++;
                }
                if ((nbp & 1) != 0)
                {
                    PbPossible = true;
                }
            }
            else
            {
                PbPossible = true;
            }
            //-- Search for the smallest coefficient between A4 and A0
            if (PbPossible)
            {
                //  Modified by Sergey KHROMOV - Thu Oct 24 12:45:35 2002 Begin
                double anAMin = RealLast();
                double anAMax = -1;
                double anEps = RealEpsilon();

                for (i = 0; i < 5; i++)
                {
                    anAMin = Min(anAMin, Max(anAA[i], anEps));
                    anAMax = Max(anAMax, Max(anAA[i], anEps));
                }

                anEps = Min(1.0e-4, Epsilon(1000.0 * anAMax / anAMin));
                //  Modified by Sergey KHROMOV - Thu Oct 24 15:46:24 2002 End
                math_DirectPolynomialRoots MATH_A4321 = new(A4, A3, A2, A1);
                if (MATH_A4321.IsDone())
                {
                    nbp = MATH_A4321.NbSolutions();
                    //-- Add the values to the array
                    for (i = 1; i <= nbp; i++)
                    {
                        double x = MATH_A4321.Value(i);
                        bool Add = true;
                        for (j = 0; j < nbsol; j++)
                        {
                            double t = sol[j] - x;
                            if (Math.Abs(t) < anEps)
                            {
                                Add = false;
                            }
                        }
                        if (Add)
                        {
                            val[nbsol] = A0 + x * (A1 + x * (A2 + x * (A3 + x * A4)));
                            sol[nbsol] = x;
                            nbsol++;
                        }
                    }
                }
                math_DirectPolynomialRoots MATH_A3210 = new(A3, A2, A1, A0);
                if (MATH_A3210.IsDone())
                {
                    nbp = MATH_A3210.NbSolutions();
                    //-- Add the values to the array
                    for (i = 1; i <= nbp; i++)
                    {
                        double x = MATH_A3210.Value(i);
                        bool Add = true;
                        for (j = 0; j < nbsol; j++)
                        {
                            double t = sol[j] - x;
                            if (Math.Abs(t) < anEps)
                            {
                                Add = false;
                            }
                        }
                        if (Add)
                        {
                            val[nbsol] = A0 + x * (A1 + x * (A2 + x * (A3 + x * A4)));
                            sol[nbsol] = x;
                            nbsol++;
                        }
                    }
                }
                // OCCT: named A210 but solves A3 x^2 + A2 x + A1 (probably meant A2, A1, A0)
                math_DirectPolynomialRoots MATH_A210 = new(A3, A2, A1);
                if (MATH_A210.IsDone())
                {
                    nbp = MATH_A210.NbSolutions();
                    //-- Add the values to the array
                    for (i = 1; i <= nbp; i++)
                    {
                        double x = MATH_A210.Value(i);
                        bool Add = true;
                        for (j = 0; j < nbsol; j++)
                        {
                            double t = sol[j] - x;
                            if (Math.Abs(t) < anEps)
                            {
                                Add = false;
                            }
                        }
                        if (Add)
                        {
                            val[nbsol] = A0 + x * (A1 + x * (A2 + x * (A3 + x * A4)));
                            sol[nbsol] = x;
                            nbsol++;
                        }
                    }
                }
                //------------------------------------------------------------
                //-- Sort values by decreasing order of val
                bool TriOK = false;
                do
                {
                    TriOK = true;
                    for (i = 1; i < nbsol; i++)
                    {
                        if (Math.Abs(val[i]) < Math.Abs(val[i - 1]))
                        {
                            double t;
                            t = val[i];
                            val[i] = val[i - 1];
                            val[i - 1] = t;
                            t = sol[i];
                            sol[i] = sol[i - 1];
                            sol[i - 1] = t;
                            TriOK = false;
                        }
                    }
                } while (!TriOK);
                //-----------------------------------------------------------
                //-- Keep the first values
                //-- At least as many as the complete polynomial
                //--
                for (nbsol = 0; nbsol < NbsolPolyComplet || Math.Abs(val[nbsol]) < Epsilon(10000.0); nbsol++)
                {
                }
            }
            if (nbsol == 0)
            {
                nbsol = -1;
            }
            if (nbsol > 4)
            {
                same = true;
                nbsol = 0;
            }
        }

        public MyDirectPolynomialRoots(double A2, double A1, double A0)
        {
            for (int anIdx = 0; anIdx < val.Length; anIdx++)
            {
                val[anIdx] = RealLast();
                sol[anIdx] = RealLast();
            }
            nbsol = 0;
            if ((Math.Abs(A2) + Math.Abs(A1) + Math.Abs(A0)) < Epsilon(10000.0))
            {
                same = true;
                return;
            }
            math_DirectPolynomialRoots MATH_A210 = new(A2, A1, A0);
            same = false;
            if (MATH_A210.IsDone())
            {
                for (int i = 1; i <= MATH_A210.NbSolutions(); i++)
                {
                    double x = MATH_A210.Value(i);
                    val[nbsol] = A0 + x * (A1 + x * A2);
                    sol[nbsol] = x;
                    nbsol++;
                }
            }
            else
            {
                nbsol = -1;
            }
        }

        public int NbSolutions() => nbsol;

        public double Value(int i) => sol[i - 1];

        // OCCT returns this bool as a double
        public bool IsDone() => nbsol > -1;

        public bool InfiniteRoots() => same;
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
    internal static void Traitement_Points_Confondus(ref int nb_pts, IntAna2d_IntPoint[] pts)
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
                if (Points_Confondus(pts[i - 1].Value().X(),
                                     pts[i - 1].Value().Y(),
                                     pts[j - 1].Value().X(),
                                     pts[j - 1].Value().Y()))
                {
                    int k;
                    Non_Egalite = false;
                    for (k = i; k < nb_pts; k++)
                    {
                        double Xk, Yk, Uk;
                        Xk = pts[k].Value().X();
                        Yk = pts[k].Value().Y();
                        Uk = pts[k].ParamOnFirst();
                        pts[k - 1].SetValue(Xk, Yk, Uk);
                    }
                    nb_pts--;
                }
            }
        }
    }

    /// <summary>
    /// Converts x1, y1, coordinates in the system of <paramref name="Dir1"/>, into the "absolute" coordinate system.
    /// </summary>
    internal static void Coord_Ancien_Repere(ref double x1, ref double y1, in gp_Ax2d Dir1)
    {
        double t11, t12, t21, t22, t13, t23;
        double x0, y0;

        // x1 and y1 are the coordinates in the reference frame associated with Dir1
        // We return these coordinates in the "absolute" reference frame

        Dir1.Direction().Coord(out t11, out t21);
        Dir1.Location().Coord(out t13, out t23);

        t22 = t11;
        t12 = -t21;

        x0 = t11 * x1 + t12 * y1 + t13;
        y0 = t21 * x1 + t22 * y1 + t23;

        x1 = x0;
        y1 = y0;
    }
}
