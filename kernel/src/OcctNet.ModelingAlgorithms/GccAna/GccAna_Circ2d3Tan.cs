// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// The 2D circles tangent to three arguments, each a qualified circle, a qualified line or a point (passed through):
/// three circles, two circles and a line, a circle and two lines, three lines, and the cases with one, two or three
/// points. Qualifiers (enclosing, enclosed, outside, unqualified) restrict the solutions; for unqualified arguments
/// WhichQualifier gives the qualifiers the solution actually has.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). The constructor taking three circles solves the
/// eight systems of quadratic equations directly; the others intersect the bisectors of the arguments (GccAna's
/// bisector classes) with IntAna2d.
/// </remarks>
public sealed partial class GccAna_Circ2d3Tan
{
    private bool WellDone;
    private int NbrSol;
    private readonly NCollection_Array1<gp_Circ2d> cirsol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<GccEnt_Position> qualifier2;
    private readonly NCollection_Array1<GccEnt_Position> qualifier3;
    private readonly NCollection_Array1<int> TheSame1;
    private readonly NCollection_Array1<int> TheSame2;
    private readonly NCollection_Array1<int> TheSame3;
    private readonly NCollection_Array1<gp_Pnt2d> pnttg1sol;
    private readonly NCollection_Array1<gp_Pnt2d> pnttg2sol;
    private readonly NCollection_Array1<gp_Pnt2d> pnttg3sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> par2sol;
    private readonly NCollection_Array1<double> par3sol;
    private readonly NCollection_Array1<double> pararg1;
    private readonly NCollection_Array1<double> pararg2;
    private readonly NCollection_Array1<double> pararg3;

    //=========================================================================
    //   Creation of a circle tangent to three circles.                        +
    //=========================================================================
    public GccAna_Circ2d3Tan(in GccEnt_QualifiedCirc Qualified1,
        in GccEnt_QualifiedCirc Qualified2,
        in GccEnt_QualifiedCirc Qualified3,
        double Tolerance)
    {
        cirsol = new NCollection_Array1<gp_Circ2d>(1, 16);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, 16);
        qualifier2 = new NCollection_Array1<GccEnt_Position>(1, 16);
        qualifier3 = new NCollection_Array1<GccEnt_Position>(1, 16);
        TheSame1 = new NCollection_Array1<int>(1, 16);
        TheSame2 = new NCollection_Array1<int>(1, 16);
        TheSame3 = new NCollection_Array1<int>(1, 16);
        pnttg1sol = new NCollection_Array1<gp_Pnt2d>(1, 16);
        pnttg2sol = new NCollection_Array1<gp_Pnt2d>(1, 16);
        pnttg3sol = new NCollection_Array1<gp_Pnt2d>(1, 16);
        par1sol = new NCollection_Array1<double>(1, 16);
        par2sol = new NCollection_Array1<double>(1, 16);
        par3sol = new NCollection_Array1<double>(1, 16);
        pararg1 = new NCollection_Array1<double>(1, 16);
        pararg2 = new NCollection_Array1<double>(1, 16);
        pararg3 = new NCollection_Array1<double>(1, 16);

        gp_Dir2d dirx = new(gp_Dir2d.D.X);
        double Tol = Math.Abs(Tolerance);
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                 || Qualified2.IsUnqualified())
            || !(Qualified3.IsEnclosed() || Qualified3.IsEnclosing() || Qualified3.IsOutside()
                 || Qualified3.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        gp_Circ2d Cir1 = Qualified1.Qualified();
        gp_Circ2d Cir2 = Qualified2.Qualified();
        gp_Circ2d Cir3 = Qualified3.Qualified();
        double R1 = Cir1.Radius();
        double R2 = Cir2.Radius();
        double R3 = Cir3.Radius();
        gp_Pnt2d center1 = Cir1.Location();
        gp_Pnt2d center2 = Cir2.Location();
        gp_Pnt2d center3 = Cir3.Location();

        double X1 = center1.X();
        double X2 = center2.X();
        double X3 = center3.X();

        double Y1 = center1.Y();
        double Y2 = center2.Y();
        double Y3 = center3.Y();

        gp_XY dir2 = center1.XY() - center2.XY();
        gp_XY dir3 = center1.XY() - center3.XY();

        //////////
        if ((Math.Abs(R1 - R2) <= Tolerance && center1.IsEqual(center2, Tolerance))
            || (Math.Abs(R1 - R3) <= Tolerance && center1.IsEqual(center3, Tolerance))
            || (Math.Abs(R2 - R3) <= Tolerance && center2.IsEqual(center3, Tolerance)))
        {
            return;
        }
        else
        {
            if (Math.Abs(dir2 ^ dir3) <= Tolerance)
            {
                double Dist1 = center1.Distance(center2);
                double Dist2 = center1.Distance(center3);
                double Dist3 = center2.Distance(center3);
                if (Math.Abs(Math.Abs(R1 - R2) - Dist1) <= Tolerance)
                {
                    if (Math.Abs(Math.Abs(R1 - R3) - Dist2) <= Tolerance)
                    {
                        if (Math.Abs(Math.Abs(R2 - R3) - Dist3) <= Tolerance)
                        {
                            return;
                        }
                    }
                    else if (Math.Abs(R1 + R3 - Dist2) <= Tolerance)
                    {
                        if (Math.Abs(R2 + R3 - Dist3) <= Tolerance)
                        {
                            return;
                        }
                    }
                }
                else if (Math.Abs(R1 + R2 - Dist1) <= Tolerance)
                {
                    if (Math.Abs(Math.Abs(R1 - R3) - Dist2) <= Tolerance
                        && Math.Abs(R2 + R3 - Dist3) <= Tolerance)
                    {
                    }
                    else
                    {
                        if (Math.Abs(Math.Abs(R2 - R3) - Dist3) <= Tolerance
                            && Math.Abs(R1 + R3 - Dist2) <= Tolerance)
                        {
                            return;
                        }
                    }
                }
            }
        }
        /////////
        NCollection_Array1<double> A2 = new(1, 8);
        NCollection_Array1<double> B2 = new(1, 8);
        NCollection_Array1<double> C2 = new(1, 8);
        NCollection_Array1<double> D2 = new(1, 8);
        NCollection_Array1<double> E2 = new(1, 8);
        NCollection_Array1<double> F2 = new(1, 8);
        NCollection_Array1<double> A3 = new(1, 8);
        NCollection_Array1<double> B3 = new(1, 8);
        NCollection_Array1<double> C3 = new(1, 8);
        NCollection_Array1<double> D3 = new(1, 8);
        NCollection_Array1<double> E3 = new(1, 8);
        NCollection_Array1<double> F3 = new(1, 8);
        NCollection_Array1<double> Beta2 = new(1, 8);
        NCollection_Array1<double> Gamma2 = new(1, 8);
        NCollection_Array1<double> Delta2 = new(1, 8);
        NCollection_Array1<double> Beta3 = new(1, 8);
        NCollection_Array1<double> Gamma3 = new(1, 8);
        NCollection_Array1<double> Delta3 = new(1, 8);
        double a2, b2, c2, d2, e2, f2;
        double a3, b3, c3, d3, e3, f3;
        double A, B, C, D, E;
        bool IsSame;
        bool IsTouch;
        int FirstIndex;

        int i, j, k, l;
        NCollection_Array1<double> xSol = new(1, 64);
        NCollection_Array1<double> ySol = new(1, 64);
        // OCCT: rSol has 16 slots where xSol/ySol have 64; more than 16 triplets would overrun it (OCCT's range check and the port throw)
        NCollection_Array1<double> rSol = new(1, 16);
        NCollection_Array1<int> FirstSol = new(1, 9);
        NCollection_Array1<double> xSol1 = new(1, 32);
        NCollection_Array1<double> ySol1 = new(1, 32);
        NCollection_Array1<double> rSol1 = new(1, 32);
        NCollection_Array1<int> FirstSol1 = new(1, 9);
        double x, y, r;
        double m, n, t, s, v;
        double p, q;
        double Epsilon;

        int CurSol;

        //*********************************************************************************************
        //*********************************************************************************************

        //   Actually we have to find solutions of eight systems of equations:
        //         _                                          _
        //        | (X - X1)2 + (Y - Y1)2 = (R - R1)2        | (X - X1)2 + (Y - Y1)2 = (R + R1)2
        //   1)  <  (X - X2)2 + (Y - Y2)2 = (R - R2)2   2)  <  (X - X2)2 + (Y - Y2)2 = (R - R2)2
        //        \_(X - X3)2 + (Y - Y3)2 = (R - R3)2        \_(X - X3)2 + (Y - Y3)2 = (R - R3)2
        //         _                                          _
        //        | (X - X1)2 + (Y - Y1)2 = (R - R1)2        | (X - X1)2 + (Y - Y1)2 = (R - R1)2
        //   3)  <  (X - X2)2 + (Y - Y2)2 = (R + R2)2   4)  <  (X - X2)2 + (Y - Y2)2 = (R - R2)2
        //        \_(X - X3)2 + (Y - Y3)2 = (R - R3)2        \_(X - X3)2 + (Y - Y3)2 = (R + R3)2
        //         _                                          _
        //        | (X - X1)2 + (Y - Y1)2 = (R + R1)2        | (X - X1)2 + (Y - Y1)2 = (R + R1)2
        //   5)  <  (X - X2)2 + (Y - Y2)2 = (R + R2)2   6)  <  (X - X2)2 + (Y - Y2)2 = (R - R2)2
        //        \_(X - X3)2 + (Y - Y3)2 = (R - R3)2        \_(X - X3)2 + (Y - Y3)2 = (R + R3)2
        //         _                                          _
        //        | (X - X1)2 + (Y - Y1)2 = (R - R1)2        | (X - X1)2 + (Y - Y1)2 = (R + R1)2
        //   7)  <  (X - X2)2 + (Y - Y2)2 = (R + R2)2   8)  <  (X - X2)2 + (Y - Y2)2 = (R + R2)2
        //        \_(X - X3)2 + (Y - Y3)2 = (R + R3)2        \_(X - X3)2 + (Y - Y3)2 = (R + R3)2

        //   each equation (X - Xi)2 + (Y - Yi)2 = (R +- Ri)2 means that the circle (X,Y,R) is tangent
        //   to the circle (Xi,Yi,Ri).

        //   The number of each system is very important.
        //   Further index i shows the number of the system.

        //   Further Beta, Gamma and Delta are coefficients of the equation:
        //                R +- Ri = Beta*X + Gamma*Y + Delta  where i=2 or i=3

        //*********************************************************************************************
        //*********************************************************************************************

        //   Verification do two circles touch each other or not
        //   if at least one circle touches other one IsTouch become Standard_Standard_True

        IsTouch =
          Math.Abs((X1 - X2) * (X1 - X2) + (Y1 - Y2) * (Y1 - Y2) - (R1 - R2) * (R1 - R2)) <= Tolerance
          || Math.Abs((X1 - X2) * (X1 - X2) + (Y1 - Y2) * (Y1 - Y2) - (R1 + R2) * (R1 + R2)) <= Tolerance
          || Math.Abs((X1 - X3) * (X1 - X3) + (Y1 - Y3) * (Y1 - Y3) - (R1 - R3) * (R1 - R3)) <= Tolerance
          || Math.Abs((X1 - X3) * (X1 - X3) + (Y1 - Y3) * (Y1 - Y3) - (R1 + R3) * (R1 + R3)) <= Tolerance
          || Math.Abs((X2 - X3) * (X2 - X3) + (Y2 - Y3) * (Y2 - Y3) - (R2 - R3) * (R2 - R3)) <= Tolerance
          || Math.Abs((X2 - X3) * (X2 - X3) + (Y2 - Y3) * (Y2 - Y3) - (R2 + R3) * (R2 + R3)) <= Tolerance;

        //   First step:
        //     We are searching for Beta, Gamma and Delta coefficients
        //     and also coefficients of the system of second order equations:
        //     _
        //    |  a2*x*x +2*b2*x*y + c2*y*y +2*d2*x + 2*e2*y + f2 = 0
        //   <
        //    \_ a3*x*x +2*b3*x*y + c3*y*y +2*d3*x + 2*e3*y + f3 = 0   ,

        //     obtained by exclusion of R from source systems.

        for (i = 1; i <= 8; i++)
        {
            //    _
            //   | (X - X1)2 + (Y - Y1)2 = (R +- R1)2
            //  <
            //   \_(X - X2)2 + (Y - Y2)2 = (R +- R2)2

            if (i == 1 || i == 4 || i == 5 || i == 8)
            {
                if (Math.Abs(R1 - R2) > Tolerance)
                {
                    Beta2[i] = (X1 - X2) / (R1 - R2);
                    Gamma2[i] = (Y1 - Y2) / (R1 - R2);
                    Delta2[i] =
                      (X2 * X2 - X1 * X1 + Y2 * Y2 - Y1 * Y1 + (R1 - R2) * (R1 - R2)) / (2 * (R1 - R2));
                }
            }
            else
            {
                Beta2[i] = (X1 - X2) / (R1 + R2);
                Gamma2[i] = (Y1 - Y2) / (R1 + R2);
                Delta2[i] = (X2 * X2 - X1 * X1 + Y2 * Y2 - Y1 * Y1 + (R1 + R2) * (R1 + R2)) / (2 * (R1 + R2));
            }
            if ((i == 1 || i == 4 || i == 5 || i == 8) && (Math.Abs(R1 - R2) <= Tolerance))
            {
                //  If R1 = R2
                A2[i] = 0.0;
                B2[i] = 0.0;
                C2[i] = 0.0;
                D2[i] = X2 - X1;
                E2[i] = Y2 - Y1;
                F2[i] = X1 * X1 - X2 * X2 + Y1 * Y1 - Y2 * Y2;
            }
            else
            {
                A2[i] = Beta2[i] * Beta2[i] - 1.0;
                B2[i] = Beta2[i] * Gamma2[i];
                C2[i] = Gamma2[i] * Gamma2[i] - 1.0;
                D2[i] = Beta2[i] * Delta2[i] + X2;
                E2[i] = Gamma2[i] * Delta2[i] + Y2;
                F2[i] = Delta2[i] * Delta2[i] - X2 * X2 - Y2 * Y2;
            }

            //    _
            //   | (X - X1)2 + (Y - Y1)2 = (R +- R1)2
            //  <
            //   \_(X - X3)2 + (Y - Y3)2 = (R +- R3)2

            if (i == 1 || i == 3 || i == 6 || i == 8)
            {
                if (Math.Abs(R1 - R3) > Tolerance)
                {
                    Beta3[i] = (X1 - X3) / (R1 - R3);
                    Gamma3[i] = (Y1 - Y3) / (R1 - R3);
                    Delta3[i] =
                      (X3 * X3 - X1 * X1 + Y3 * Y3 - Y1 * Y1 + (R1 - R3) * (R1 - R3)) / (2 * (R1 - R3));
                }
            }
            else
            {
                Beta3[i] = (X1 - X3) / (R1 + R3);
                Gamma3[i] = (Y1 - Y3) / (R1 + R3);
                Delta3[i] = (X3 * X3 - X1 * X1 + Y3 * Y3 - Y1 * Y1 + (R1 + R3) * (R1 + R3)) / (2 * (R1 + R3));
            }
            if ((i == 1 || i == 3 || i == 6 || i == 8) && (Math.Abs(R1 - R3) <= Tolerance))
            {
                A3[i] = 0.0;
                B3[i] = 0.0;
                C3[i] = 0.0;
                D3[i] = X3 - X1;
                E3[i] = Y3 - Y1;
                F3[i] = X1 * X1 - X3 * X3 + Y1 * Y1 - Y3 * Y3;
            }
            else
            {
                A3[i] = Beta3[i] * Beta3[i] - 1.0;
                B3[i] = Beta3[i] * Gamma3[i];
                C3[i] = Gamma3[i] * Gamma3[i] - 1.0;
                D3[i] = Beta3[i] * Delta3[i] + X3;
                E3[i] = Gamma3[i] * Delta3[i] + Y3;
                F3[i] = Delta3[i] * Delta3[i] - X3 * X3 - Y3 * Y3;
            }
        }

        //   Second step:
        //     We are searching for the couple (X,Y) as a solution  of the system:
        //     _
        //    |  a2*x*x +2*b2*x*y + c2*y*y +2*d2*x + 2*e2*y + f2 = 0
        //   <
        //    \_ a3*x*x +2*b3*x*y + c3*y*y +2*d3*x + 2*e3*y + f3 = 0

        CurSol = 1;
        for (i = 1; i <= 8; i++)
        {
            a2 = A2[i];
            a3 = A3[i];
            b2 = B2[i];
            b3 = B3[i];
            c2 = C2[i];
            c3 = C3[i];
            d2 = D2[i];
            d3 = D3[i];
            e2 = E2[i];
            e3 = E3[i];
            f2 = F2[i];
            f3 = F3[i];

            FirstSol[i] = CurSol;

            //  In some cases we know that some systems have no solution in any case due to qualifiers
            if (((i == 2 || i == 5 || i == 6 || i == 8)
                 && (Qualified1.IsEnclosed() || Qualified1.IsEnclosing()))
                || ((i == 1 || i == 3 || i == 4 || i == 7) && Qualified1.IsOutside()))
            {
                continue;
            }

            if (((i == 3 || i == 5 || i == 7 || i == 8)
                 && (Qualified2.IsEnclosed() || Qualified2.IsEnclosing()))
                || ((i == 1 || i == 2 || i == 4 || i == 6) && Qualified2.IsOutside()))
            {
                continue;
            }

            if (((i == 4 || i == 6 || i == 7 || i == 8)
                 && (Qualified3.IsEnclosed() || Qualified3.IsEnclosing()))
                || ((i == 1 || i == 2 || i == 3 || i == 5) && Qualified3.IsOutside()))
            {
                continue;
            }

            // Check is Cir1 a solution of this system or not
            // In that case equations are equal to each other
            if (Math.Abs(a2 - a3) <= Tolerance && Math.Abs(b2 - b3) <= Tolerance
                && Math.Abs(c2 - c3) <= Tolerance && Math.Abs(d2 - d3) <= Tolerance
                && Math.Abs(e2 - e3) <= Tolerance && Math.Abs(f2 - f3) <= Tolerance)
            {
                xSol[CurSol] = X1;
                ySol[CurSol] = Y1;
                CurSol++;
                continue;
            }
            // 1) a2 = 0
            if (Math.Abs(a2) <= Tolerance)
            {
                // 1.1) b2y + d2 = 0
                //   Searching for solution of the equation Ay2 + By + C = 0
                A = c2;
                B = 2.0 * e2;
                C = f2;
                math_DirectPolynomialRoots yRoots = new(A, B, C);
                if (yRoots.IsDone() && !yRoots.InfiniteRoots())
                {
                    for (k = 1; k <= yRoots.NbSolutions(); k++)
                    {
                        //   for each y solution:
                        y = yRoots.Value(k);
                        //   Searching for solution of the equation Ax2 + Bx + C = 0
                        if ((k != 2 || Math.Abs(y - yRoots.Value(1)) > 10 * Tolerance)
                            && Math.Abs(b2 * y + d2) <= b2 * Tolerance)
                        {
                            A = a3;
                            B = 2 * (b3 * y + d3);
                            C = c3 * (y * y) + 2 * e3 * y + f3;
                            math_DirectPolynomialRoots xRoots = new(A, B, C);
                            if (xRoots.IsDone() && !xRoots.InfiniteRoots())
                            {
                                for (j = 1; j <= xRoots.NbSolutions(); j++)
                                {
                                    x = xRoots.Value(j);
                                    if (j != 2 || Math.Abs(x - xRoots.Value(1)) > 10 * Tolerance)
                                    {
                                        xSol[CurSol] = x;
                                        ySol[CurSol] = y;
                                        CurSol++;
                                    }
                                }
                            }
                        }
                    }
                }

                // 1.2) b2y + d2 != 0
                A = a3 * c2 * c2 - 4 * b2 * (b3 * c2 - b2 * c3);
                B = 4 * a3 * c2 * e2 - 4 * b3 * (c2 * d2 + 2 * b2 * e2)
                    + 4 * b2 * (2 * c3 * d2 - c2 * d3 + 2 * b2 * e3);
                C = 2 * a3 * (c2 * f2 + 2 * e2 * e2) - 4 * b3 * (b2 * f2 + 2 * e2 * d2) + 4 * c3 * d2 * d2
                    - 4 * d3 * (c2 * d2 + 2 * b2 * e2) + 16 * b2 * e3 * d2 + 4 * b2 * b2 * f3;
                D = 4 * a3 * e2 * f2 - 4 * b3 * d2 * f2 - 4 * d3 * (b2 * f2 + 2 * d2 * e2) + 8 * d2 * d2 * e3
                    + 8 * b2 * d2 * f3;
                E = a3 * f2 * f2 - 4 * d2 * d3 * f2 + 4 * d2 * d2 * f3;

                //   Searching for solution of the equation Ay4 + By3 + Cy2 + Dy + E = 0
                // Special case: one circle touches other
                if (IsTouch)
                {
                    // Derivation of the equation Ay4 + By3 + Cy2 + Dy + E
                    // yRootsD is OCCT's yRoots1 (C# forbids reusing the name of the yRoots1 declared below)
                    math_DirectPolynomialRoots yRootsD = new(4 * A, 3 * B, 2 * C, D);
                    if (yRootsD.IsDone() && !yRootsD.InfiniteRoots())
                    {
                        for (k = 1; k <= yRootsD.NbSolutions(); k++)
                        {
                            y = yRootsD.Value(k);
                            // Check if this value is already caught
                            IsSame = false;
                            for (l = 1; l < k; l++)
                            {
                                if (Math.Abs(y - yRootsD.Value(l)) <= 10 * Tolerance)
                                {
                                    IsSame = true;
                                }
                            }

                            Epsilon =
                              (Math.Abs((Math.Abs((Math.Abs(4 * A * y) + Math.Abs(3 * B)) * y) + Math.Abs(2 * C))
                                        * y)
                               + Math.Abs(D));
                            if (Math.Abs((((A * y + B) * y + C) * y + D) * y + E) <= Epsilon * Tolerance)
                            {
                                if (!IsSame && Math.Abs(b2 * y + d2) > b2 * Tolerance)
                                {
                                    x = -(c2 * (y * y) + 2 * e2 * y + f2) / (2 * (b2 * y + d2));
                                    xSol[CurSol] = x;
                                    ySol[CurSol] = y;
                                    CurSol++;
                                }
                            }
                        }
                    }
                }

                math_DirectPolynomialRoots yRoots1 = new(A, B, C, D, E);
                if (yRoots1.IsDone() && !yRoots1.InfiniteRoots())
                {
                    for (k = 1; k <= yRoots1.NbSolutions(); k++)
                    {
                        y = yRoots1.Value(k);
                        // Check if this value is already caught
                        IsSame = false;
                        FirstIndex = (i == 1) ? 1 : FirstSol[i];
                        for (l = FirstIndex; l < CurSol; l++)
                        {
                            if (Math.Abs(y - ySol[l]) <= 10 * Tolerance)
                            {
                                IsSame = true;
                            }
                        }

                        if (!IsSame && Math.Abs(b2 * y + d2) > b2 * Tolerance)
                        {
                            x = -(c2 * (y * y) + 2 * e2 * y + f2) / (2 * (b2 * y + d2));
                            xSol[CurSol] = x;
                            ySol[CurSol] = y;
                            CurSol++;
                        }
                    }
                }
            }
            else
            {
                // 2) a2 != 0
                // Coefficients of the equation     (sy + v)Math.Sqrt(p2 - q) = (my2 + ny + t)
                m = 2 * a3 * b2 * b2 / (a2 * a2) - 2 * b2 * b3 / a2 - a3 * c2 / a2 + c3;
                n = 4 * a3 * b2 * d2 / (a2 * a2) - 2 * b3 * d2 / a2 - 2 * b2 * d3 / a2 - 2 * a3 * e2 / a2
                    + 2 * e3;
                t = 2 * a3 * d2 * d2 / (a2 * a2) - 2 * d2 * d3 / a2 - a3 * f2 / a2 + f3;
                s = 2 * b3 - 2 * a3 * b2 / a2;
                v = 2 * d3 - 2 * d2 * a3 / a2;

                //------------------------------------------
                // If s = v = 0
                if (Math.Abs(s) <= Tolerance && Math.Abs(v) <= Tolerance)
                {
                    math_DirectPolynomialRoots yRoots = new(m, n, t);
                    if (yRoots.IsDone() && !yRoots.InfiniteRoots())
                    {
                        for (k = 1; k <= yRoots.NbSolutions(); k++)
                        {
                            //   for each y solution:
                            y = yRoots.Value(k);

                            p = -(b2 * y + d2) / a2;
                            q = (c2 * (y * y) + 2 * e2 * y + f2) / a2;
                            Epsilon = 2.0
                                      * (Math.Abs((b2 * b2 + Math.Abs(a2 * c2)) * y) + Math.Abs(b2 * d2)
                                         + Math.Abs(a2 * e2))
                                      / (a2 * a2);
                            if ((k != 2 || Math.Abs(y - yRoots.Value(1)) > 10 * Tolerance)
                                && p * p - q >= -Epsilon * Tolerance)
                            {
                                A = a2;
                                B = 2 * (b2 * y + d2);
                                C = c2 * y * y + 2 * e2 * y + f2;
                                math_DirectPolynomialRoots xRoots = new(A, B, C);
                                if (xRoots.IsDone() && !xRoots.InfiniteRoots())
                                {
                                    for (l = 1; l <= xRoots.NbSolutions(); l++)
                                    {
                                        //   for each x solution:
                                        x = xRoots.Value(l);

                                        if (l != 2 || Math.Abs(x - xRoots.Value(1)) > 10 * Tolerance)
                                        {
                                            xSol[CurSol] = x;
                                            ySol[CurSol] = y;
                                            CurSol++;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    //------------------------------------------
                    // If (s*y + v) != 0

                    A = s * s * (b2 * b2 - a2 * c2) - m * m * a2 * a2;
                    B = 2 * s * v * (b2 * b2 - a2 * c2) + 2 * s * s * (b2 * d2 - a2 * e2) - 2 * m * n * a2 * a2;
                    C = v * v * (b2 * b2 - a2 * c2) + 4 * s * v * (b2 * d2 - a2 * e2)
                        + s * s * (d2 * d2 - a2 * f2) - a2 * a2 * (2 * m * t + n * n);
                    D = 2 * v * v * (b2 * d2 - a2 * e2) + 2 * s * v * (d2 * d2 - a2 * f2) - 2 * n * t * a2 * a2;
                    E = v * v * (d2 * d2 - a2 * f2) - t * t * a2 * a2;

                    //   Searching for solution of the equation Ay4 + By3 + Cy2 + Dy + E = 0
                    // Special case: one circle touches other
                    if (IsTouch)
                    {
                        // Derivation of the equation Ay4 + By3 + Cy2 + Dy + E
                        math_DirectPolynomialRoots yRoots1 = new(4 * A, 3 * B, 2 * C, D);
                        if (yRoots1.IsDone() && !yRoots1.InfiniteRoots())
                        {
                            for (k = 1; k <= yRoots1.NbSolutions(); k++)
                            {
                                y = yRoots1.Value(k);

                                p = -(b2 * y + d2) / a2;
                                q = (c2 * (y * y) + 2 * e2 * y + f2) / a2;

                                // Check if this value is already caught
                                IsSame = false;
                                FirstIndex = (i == 1) ? 1 : FirstSol[i];
                                for (l = FirstIndex; l < CurSol; l++)
                                {
                                    if (Math.Abs(y - ySol[l]) <= 10 * Tolerance)
                                    {
                                        IsSame = true;
                                    }
                                }

                                Epsilon =
                                  (Math.Abs((Math.Abs((Math.Abs(4 * A * y) + Math.Abs(3 * B)) * y) + Math.Abs(2 * C))
                                            * y)
                                   + Math.Abs(D));
                                if (Math.Abs((((A * y + B) * y + C) * y + D) * y + E) <= Epsilon * Tolerance)
                                {
                                    Epsilon = 2.0
                                              * (Math.Abs((b2 * b2 + Math.Abs(a2 * c2)) * y) + Math.Abs(b2 * d2)
                                                 + Math.Abs(a2 * e2))
                                              / (a2 * a2);
                                    if (!IsSame && p * p - q >= -Epsilon * Tolerance)
                                    {
                                        A = a2;
                                        B = 2 * (b2 * y + d2);
                                        C = c2 * y * y + 2 * e2 * y + f2;
                                        math_DirectPolynomialRoots xRoots = new(A, B, C);
                                        if (xRoots.IsDone() && !xRoots.InfiniteRoots())
                                        {
                                            for (l = 1; l <= xRoots.NbSolutions(); l++)
                                            {
                                                //   for each x solution:
                                                x = xRoots.Value(l);

                                                if (l != 2 || Math.Abs(x - xRoots.Value(1)) > 10 * Tolerance)
                                                {
                                                    xSol[CurSol] = x;
                                                    ySol[CurSol] = y;
                                                    CurSol++;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    math_DirectPolynomialRoots yRoots = new(A, B, C, D, E);
                    if (yRoots.IsDone() && !yRoots.InfiniteRoots())
                    {
                        for (k = 1; k <= yRoots.NbSolutions(); k++)
                        {
                            //   for each y solution:
                            y = yRoots.Value(k);

                            p = -(b2 * y + d2) / a2;
                            q = (c2 * (y * y) + 2 * e2 * y + f2) / a2;

                            // Check if this value is already caught
                            IsSame = false;
                            for (l = 1; l < k; l++)
                            {
                                if (Math.Abs(y - yRoots.Value(l)) <= 10 * Tolerance)
                                {
                                    IsSame = true;
                                }
                            }

                            Epsilon = 2.0
                                      * (Math.Abs((b2 * b2 + Math.Abs(a2 * c2)) * y) + Math.Abs(b2 * d2)
                                         + Math.Abs(a2 * e2))
                                      / (a2 * a2);
                            if (!IsSame && p * p - q >= -Epsilon * Tolerance)
                            {
                                A = a2;
                                B = 2 * (b2 * y + d2);
                                C = c2 * y * y + 2 * e2 * y + f2;
                                math_DirectPolynomialRoots xRoots = new(A, B, C);
                                if (xRoots.IsDone() && !xRoots.InfiniteRoots())
                                {
                                    for (l = 1; l <= xRoots.NbSolutions(); l++)
                                    {
                                        //   for each x solution:
                                        x = xRoots.Value(l);

                                        if (l != 2 || Math.Abs(x - xRoots.Value(1)) > 10 * Tolerance)
                                        {
                                            xSol[CurSol] = x;
                                            ySol[CurSol] = y;
                                            CurSol++;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        FirstSol[9] = CurSol;

        //  Third step:
        //    Check of couples (X,Y) and searching for R. R must be great than 0
        CurSol = 1;
        for (i = 1; i <= 8; i++)
        {
            FirstSol1[i] = CurSol;
            for (j = FirstSol[i]; j < FirstSol[i + 1]; j++)
            {
                x = xSol[j];
                y = ySol[j];
                // in some cases when R1 = R2 :
                if ((i == 1 || i == 4 || i == 5 || i == 8) && (Math.Abs(R1 - R2) <= Tolerance))
                {
                    if (i == 1 || i == 4)
                    {
                        r = R1 + Math.Sqrt((x - X1) * (x - X1) + (y - Y1) * (y - Y1));
                        Epsilon = 10 * (2 * Math.Abs(r - R2) + Math.Abs(x - X2) + Math.Abs(y - Y2));
                        if (Math.Abs((r - R2) * (r - R2) - (x - X2) * (x - X2) - (y - Y2) * (y - Y2))
                            <= Epsilon * Tolerance)
                        {
                            xSol1[CurSol] = x;
                            ySol1[CurSol] = y;
                            rSol1[CurSol] = r;
                            CurSol++;
                        }
                        r = R1 - Math.Sqrt((x - X1) * (x - X1) + (y - Y1) * (y - Y1));
                        Epsilon = 10 * (2 * Math.Abs(r - R2) + Math.Abs(x - X2) + Math.Abs(y - Y2));
                        if ((r > Tolerance)
                            && (Math.Abs((r - R2) * (r - R2) - (x - X2) * (x - X2) - (y - Y2) * (y - Y2))
                                <= Epsilon * Tolerance))
                        {
                            xSol1[CurSol] = x;
                            ySol1[CurSol] = y;
                            rSol1[CurSol] = r;
                            CurSol++;
                        }
                    }
                    else
                    {
                        //        i == 5 || i == 8
                        r = -R1 + Math.Sqrt((x - X1) * (x - X1) + (y - Y1) * (y - Y1));
                        if (r > Tolerance)
                        {
                            xSol1[CurSol] = x;
                            ySol1[CurSol] = y;
                            rSol1[CurSol] = r;
                            CurSol++;
                        }
                    }
                }
                else
                {
                    // Other cases
                    if (i == 1 || i == 4)
                    {
                        r = R2 + Beta2[i] * x + Gamma2[i] * y + Delta2[i];
                        if (r > Tolerance)
                        {
                            xSol1[CurSol] = x;
                            ySol1[CurSol] = y;
                            rSol1[CurSol] = r;
                            CurSol++;
                        }
                    }
                    if (i == 5 || i == 8)
                    {
                        r = -R2 - Beta2[i] * x - Gamma2[i] * y - Delta2[i];
                        if (r > Tolerance)
                        {
                            xSol1[CurSol] = x;
                            ySol1[CurSol] = y;
                            rSol1[CurSol] = r;
                            CurSol++;
                        }
                    }
                    if (i == 3 || i == 7)
                    {
                        r = -R2 + Beta2[i] * x + Gamma2[i] * y + Delta2[i];
                        if (r > Tolerance)
                        {
                            xSol1[CurSol] = x;
                            ySol1[CurSol] = y;
                            rSol1[CurSol] = r;
                            CurSol++;
                        }
                    }
                    if (i == 2 || i == 6)
                    {
                        r = R2 - Beta2[i] * x - Gamma2[i] * y - Delta2[i];
                        if (r > Tolerance)
                        {
                            xSol1[CurSol] = x;
                            ySol1[CurSol] = y;
                            rSol1[CurSol] = r;
                            CurSol++;
                        }
                    }
                }
            }
        }
        FirstSol1[9] = CurSol;
        //   Fourth step
        //     Check of triplets (X,Y,R).
        CurSol = 1;
        for (i = 1; i <= 8; i++)
        {
            FirstSol[i] = CurSol;
            for (j = FirstSol1[i]; j < FirstSol1[i + 1]; j++)
            {
                x = xSol1[j];
                y = ySol1[j];
                r = rSol1[j];
                // in some cases when R1 = R3 :
                if ((i == 1 || i == 3 || i == 6 || i == 8) && Math.Abs(R1 - R3) <= Tolerance)
                {
                    if (i == 1 || i == 3)
                    {
                        Epsilon = 10 * (2 * Math.Abs(r - R3) + Math.Abs(x - X3) + Math.Abs(y - Y3));
                        if (Math.Abs((r - R3) * (r - R3) - (x - X3) * (x - X3) - (y - Y3) * (y - Y3))
                            <= Epsilon * Tolerance)
                        {
                            xSol[CurSol] = x;
                            ySol[CurSol] = y;
                            rSol[CurSol] = r;
                            CurSol++;
                        }
                    }
                    else
                    {
                        //        i == 6 || i == 8
                        Epsilon = 10 * (2 * (r + R3) + Math.Abs(x - X3) + Math.Abs(y - Y3));
                        if (Math.Abs((r + R3) * (r + R3) - (x - X3) * (x - X3) - (y - Y3) * (y - Y3))
                            <= Epsilon * Tolerance)
                        {
                            xSol[CurSol] = x;
                            ySol[CurSol] = y;
                            rSol[CurSol] = r;
                            CurSol++;
                        }
                    }
                }
                else
                {
                    // Other cases
                    Epsilon = 10 * (Math.Abs(Beta3[i]) + Math.Abs(Gamma3[i]) + 1.0);
                    if (i == 1 || i == 3)
                    {
                        if (Math.Abs(R3 + Beta3[i] * x + Gamma3[i] * y + Delta3[i] - r) <= Epsilon * Tolerance)
                        {
                            xSol[CurSol] = x;
                            ySol[CurSol] = y;
                            rSol[CurSol] = r;
                            CurSol++;
                        }
                    }
                    if (i == 6 || i == 8)
                    {
                        if (Math.Abs(R3 + Beta3[i] * x + Gamma3[i] * y + Delta3[i] + r) <= Epsilon * Tolerance)
                        {
                            xSol[CurSol] = x;
                            ySol[CurSol] = y;
                            rSol[CurSol] = r;
                            CurSol++;
                        }
                    }
                    if (i == 4 || i == 7)
                    {
                        if (Math.Abs(Beta3[i] * x + Gamma3[i] * y + Delta3[i] - r - R3) <= Epsilon * Tolerance)
                        {
                            xSol[CurSol] = x;
                            ySol[CurSol] = y;
                            rSol[CurSol] = r;
                            CurSol++;
                        }
                    }
                    if (i == 2 || i == 5)
                    {
                        if (Math.Abs(r - R3 + Beta3[i] * x + Gamma3[i] * y + Delta3[i]) <= Epsilon * Tolerance)
                        {
                            xSol[CurSol] = x;
                            ySol[CurSol] = y;
                            rSol[CurSol] = r;
                            CurSol++;
                        }
                    }
                }
            }
        }
        FirstSol[9] = CurSol;

        //  Fifth step:
        //    We have found all solutions. We have to calculate some parameters for each one.
        for (i = 1; i <= 8; i++)
        {
            for (j = FirstSol[i]; j < FirstSol[i + 1]; j++)
            {
                if ((Qualified1.IsEnclosed() && rSol[j] > R1) || (Qualified1.IsEnclosing() && rSol[j] < R1))
                {
                    continue;
                }
                if ((Qualified2.IsEnclosed() && rSol[j] > R2) || (Qualified2.IsEnclosing() && rSol[j] < R2))
                {
                    continue;
                }
                if ((Qualified3.IsEnclosed() && rSol[j] > R3) || (Qualified3.IsEnclosing() && rSol[j] < R3))
                {
                    continue;
                }

                NbrSol++;

                // RLE, avoid out of range
                if (NbrSol > cirsol.Upper())
                {
                    NbrSol = cirsol.Upper();
                }

                gp_Pnt2d Center = new gp_Pnt2d(xSol[j], ySol[j]);

                cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), rSol[j]);

                //   ==========================================================
                double distcc1 = Center.Distance(center1);
                if (!Qualified1.IsUnqualified())
                {
                    qualifier1[NbrSol] = Qualified1.Qualifier();
                }
                else if (Math.Abs(distcc1 + rSol[j] - R1) <= Tol)
                {
                    qualifier1[NbrSol] = GccEnt_enclosed;
                }
                else if (Math.Abs(distcc1 - R1 - rSol[j]) <= Tol)
                {
                    qualifier1[NbrSol] = GccEnt_outside;
                }
                else
                {
                    qualifier1[NbrSol] = GccEnt_enclosing;
                }

                // OCCT: measured from center1, not from the circle this qualifier is for
                double distcc2 = Center.Distance(center1);
                if (!Qualified2.IsUnqualified())
                {
                    qualifier2[NbrSol] = Qualified2.Qualifier();
                }
                else if (Math.Abs(distcc2 + rSol[j] - R2) <= Tol)
                {
                    qualifier2[NbrSol] = GccEnt_enclosed;
                }
                else if (Math.Abs(distcc2 - R2 - rSol[j]) <= Tol)
                {
                    qualifier2[NbrSol] = GccEnt_outside;
                }
                else
                {
                    qualifier2[NbrSol] = GccEnt_enclosing;
                }

                // OCCT: measured from center1, not from the circle this qualifier is for
                double distcc3 = Center.Distance(center1);
                if (!Qualified3.IsUnqualified())
                {
                    qualifier3[NbrSol] = Qualified3.Qualifier();
                }
                else if (Math.Abs(distcc3 + rSol[j] - R3) <= Tol)
                {
                    qualifier3[NbrSol] = GccEnt_enclosed;
                }
                else if (Math.Abs(distcc3 - R3 - rSol[j]) <= Tol)
                {
                    qualifier3[NbrSol] = GccEnt_outside;
                }
                else
                {
                    qualifier3[NbrSol] = GccEnt_enclosing;
                }

                //   ==========================================================

                if (Center.Distance(Cir1.Location()) <= Tolerance)
                {
                    TheSame1[NbrSol] = 1;
                }
                else
                {
                    TheSame1[NbrSol] = 0;
                    gp_Dir2d dc = new();
                    if ((i == 2 || i == 5 || i == 6 || i == 8) || rSol[j] > R1)
                    {
                        dc.SetXY(Cir1.Location().XY() - Center.XY());
                    }
                    else
                    {
                        dc.SetXY(Center.XY() - Cir1.Location().XY());
                    }
                    pnttg1sol[NbrSol] = new gp_Pnt2d(Center.XY() + rSol[j] * dc.XY());
                    par1sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                    pararg1[NbrSol] = ElCLib.Parameter(Cir1, pnttg1sol[NbrSol]);
                }

                if (Center.Distance(Cir2.Location()) <= Tolerance)
                {
                    TheSame2[NbrSol] = 1;
                }
                else
                {
                    TheSame2[NbrSol] = 0;
                    gp_Dir2d dc = new();
                    if ((i == 3 || i == 5 || i == 7 || i == 8) || rSol[j] > R2)
                    {
                        dc.SetXY(Cir2.Location().XY() - Center.XY());
                    }
                    else
                    {
                        dc.SetXY(Center.XY() - Cir2.Location().XY());
                    }
                    pnttg2sol[NbrSol] = new gp_Pnt2d(Center.XY() + rSol[j] * dc.XY());
                    par2sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                    pararg2[NbrSol] = ElCLib.Parameter(Cir2, pnttg2sol[NbrSol]);
                }

                if (Center.Distance(Cir3.Location()) <= Tolerance)
                {
                    TheSame3[NbrSol] = 1;
                }
                else
                {
                    TheSame3[NbrSol] = 0;
                    gp_Dir2d dc = new();
                    if ((i == 4 || i == 6 || i == 7 || i == 8) || rSol[j] > R3)
                    {
                        dc.SetXY(Cir3.Location().XY() - Center.XY());
                    }
                    else
                    {
                        dc.SetXY(Center.XY() - Cir3.Location().XY());
                    }
                    pnttg3sol[NbrSol] = new gp_Pnt2d(Center.XY() + rSol[j] * dc.XY());
                    par3sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg3sol[NbrSol]);
                    pararg3[NbrSol] = ElCLib.Parameter(Cir3, pnttg3sol[NbrSol]);
                }
            }
        }
        WellDone = true;
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The number of solutions.</summary>
    public int NbSolutions()
    {
        return NbrSol;
    }

    /// <summary>
    /// The solution number Index. The index only enumerates the solutions of this algorithm object.
    /// </summary>
    /// <exception cref="StdFail_NotDone">The construction failed.</exception>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbSolutions()].</exception>
    public gp_Circ2d ThisSolution(int Index)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }

        if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }

        return cirsol[Index];
    }

    /// <summary>
    /// The qualifiers of the three arguments for the solution number Index: the given ones where qualified, the ones
    /// computed for the solution where unqualified.
    /// </summary>
    public void WhichQualifier(int Index, out GccEnt_Position Qualif1, out GccEnt_Position Qualif2, out GccEnt_Position Qualif3)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }
        else
        {
            Qualif1 = qualifier1[Index];
            Qualif2 = qualifier2[Index];
            Qualif3 = qualifier3[Index];
        }
    }

    /// <summary>
    /// The tangency point of the solution number Index with argument 1: ParSol its parameter on the solution, ParArg
    /// on the argument (0 for a point).
    /// </summary>
    /// <exception cref="StdFail_NotDone">The construction failed, or the solution is the argument itself (IsTheSame1).</exception>
    public void Tangency1(int Index, out double ParSol, out double ParArg, out gp_Pnt2d PntSol)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }
        else
        {
            if (TheSame1[Index] == 0)
            {
                ParSol = par1sol[Index];
                ParArg = pararg1[Index];
                PntSol = pnttg1sol[Index];
            }
            else
            {
                throw new StdFail_NotDone();
            }
        }
    }

    /// <summary>
    /// The tangency point of the solution number Index with argument 2: ParSol its parameter on the solution, ParArg
    /// on the argument (0 for a point).
    /// </summary>
    /// <exception cref="StdFail_NotDone">The construction failed, or the solution is the argument itself (IsTheSame2).</exception>
    public void Tangency2(int Index, out double ParSol, out double ParArg, out gp_Pnt2d PntSol)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }
        else
        {
            if (TheSame2[Index] == 0)
            {
                ParSol = par2sol[Index];
                ParArg = pararg2[Index];
                PntSol = pnttg2sol[Index];
            }
            else
            {
                throw new StdFail_NotDone();
            }
        }
    }

    /// <summary>
    /// The tangency point of the solution number Index with argument 3: ParSol its parameter on the solution, ParArg
    /// on the argument (0 for a point).
    /// </summary>
    /// <exception cref="StdFail_NotDone">The construction failed, or the solution is the argument itself (IsTheSame3).</exception>
    public void Tangency3(int Index, out double ParSol, out double ParArg, out gp_Pnt2d PntSol)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }
        else
        {
            if (TheSame3[Index] == 0)
            {
                ParSol = par3sol[Index];
                ParArg = pararg3[Index];
                PntSol = pnttg3sol[Index];
            }
            else
            {
                throw new StdFail_NotDone();
            }
        }
    }

    /// <summary>True if the solution number Index is argument 1 itself.</summary>
    public bool IsTheSame1(int Index)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }

        if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }

        if (TheSame1[Index] == 0)
        {
            return false;
        }

        return true;
    }

    /// <summary>True if the solution number Index is argument 2 itself.</summary>
    public bool IsTheSame2(int Index)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }

        if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }

        if (TheSame2[Index] == 0)
        {
            return false;
        }

        return true;
    }

    /// <summary>True if the solution number Index is argument 3 itself.</summary>
    public bool IsTheSame3(int Index)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }

        if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }

        if (TheSame3[Index] == 0)
        {
            return false;
        }

        return true;
    }
}
