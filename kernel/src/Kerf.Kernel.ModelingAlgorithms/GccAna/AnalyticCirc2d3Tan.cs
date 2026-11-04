// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

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
internal sealed partial class AnalyticCirc2d3Tan
{
    private bool _wellDone;
    private int _nbrSol;
    private readonly NCollection_Array1<Circle2D> _cirsol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<Qualifier> _qualifier2;
    private readonly NCollection_Array1<Qualifier> _qualifier3;
    private readonly NCollection_Array1<int> _theSame1;
    private readonly NCollection_Array1<int> _theSame2;
    private readonly NCollection_Array1<int> _theSame3;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<Point2D> _pnttg2sol;
    private readonly NCollection_Array1<Point2D> _pnttg3sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _par2sol;
    private readonly NCollection_Array1<double> _par3sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<double> _pararg2;
    private readonly NCollection_Array1<double> _pararg3;

    //=========================================================================
    //   Creation of a circle tangent to three circles.                        +
    //=========================================================================
    public AnalyticCirc2d3Tan(in QualifiedCirc qualified1,
        in QualifiedCirc qualified2,
        in QualifiedCirc qualified3,
        double tolerance)
    {
        _cirsol = new NCollection_Array1<Circle2D>(1, 16);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, 16);
        _qualifier2 = new NCollection_Array1<Qualifier>(1, 16);
        _qualifier3 = new NCollection_Array1<Qualifier>(1, 16);
        _theSame1 = new NCollection_Array1<int>(1, 16);
        _theSame2 = new NCollection_Array1<int>(1, 16);
        _theSame3 = new NCollection_Array1<int>(1, 16);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, 16);
        _pnttg2sol = new NCollection_Array1<Point2D>(1, 16);
        _pnttg3sol = new NCollection_Array1<Point2D>(1, 16);
        _par1sol = new NCollection_Array1<double>(1, 16);
        _par2sol = new NCollection_Array1<double>(1, 16);
        _par3sol = new NCollection_Array1<double>(1, 16);
        _pararg1 = new NCollection_Array1<double>(1, 16);
        _pararg2 = new NCollection_Array1<double>(1, 16);
        _pararg3 = new NCollection_Array1<double>(1, 16);

        Direction2D dirx = new(Direction2D.D.X);
        double tol = Math.Abs(tolerance);
        _wellDone = false;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                 || qualified2.IsUnqualified)
            || !(qualified3.IsEnclosed || qualified3.IsEnclosing || qualified3.IsOutside
                 || qualified3.IsUnqualified))
        {
            throw new BadQualifierException();
        }

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        Circle2D cir1 = qualified1.Qualified();
        Circle2D cir2 = qualified2.Qualified();
        Circle2D cir3 = qualified3.Qualified();
        double r1 = cir1.Radius;
        double r2 = cir2.Radius;
        double r3 = cir3.Radius;
        Point2D center1 = cir1.Location;
        Point2D center2 = cir2.Location;
        Point2D center3 = cir3.Location;

        double x1 = center1.X;
        double x2 = center2.X;
        double x3 = center3.X;

        double y1 = center1.Y;
        double y2 = center2.Y;
        double y3 = center3.Y;

        Xy dir2 = center1.XY() - center2.XY();
        Xy dir3 = center1.XY() - center3.XY();

        //////////
        if ((Math.Abs(r1 - r2) <= tolerance && center1.IsEqual(center2, tolerance))
            || (Math.Abs(r1 - r3) <= tolerance && center1.IsEqual(center3, tolerance))
            || (Math.Abs(r2 - r3) <= tolerance && center2.IsEqual(center3, tolerance)))
        {
            return;
        }
        else
        {
            if (Math.Abs(dir2 ^ dir3) <= tolerance)
            {
                double dist1 = center1.DistanceTo(center2);
                double dist2 = center1.DistanceTo(center3);
                double dist3 = center2.DistanceTo(center3);
                if (Math.Abs(Math.Abs(r1 - r2) - dist1) <= tolerance)
                {
                    if (Math.Abs(Math.Abs(r1 - r3) - dist2) <= tolerance)
                    {
                        if (Math.Abs(Math.Abs(r2 - r3) - dist3) <= tolerance)
                        {
                            return;
                        }
                    }
                    else if (Math.Abs(r1 + r3 - dist2) <= tolerance)
                    {
                        if (Math.Abs(r2 + r3 - dist3) <= tolerance)
                        {
                            return;
                        }
                    }
                }
                else if (Math.Abs(r1 + r2 - dist1) <= tolerance)
                {
                    if (Math.Abs(Math.Abs(r1 - r3) - dist2) <= tolerance
                        && Math.Abs(r2 + r3 - dist3) <= tolerance)
                    {
                    }
                    else
                    {
                        if (Math.Abs(Math.Abs(r2 - r3) - dist3) <= tolerance
                            && Math.Abs(r1 + r3 - dist2) <= tolerance)
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
        NCollection_Array1<double> beta2 = new(1, 8);
        NCollection_Array1<double> gamma2 = new(1, 8);
        NCollection_Array1<double> delta2 = new(1, 8);
        NCollection_Array1<double> beta3 = new(1, 8);
        NCollection_Array1<double> gamma3 = new(1, 8);
        NCollection_Array1<double> delta3 = new(1, 8);
        double a2, b2, c2, d2, e2, f2;
        double a3, b3, c3, d3, e3, f3;
        double a, b, c, d, e;
        bool isSame;
        bool isTouch;
        int firstIndex;

        int i, j, k, l;
        NCollection_Array1<double> xSol = new(1, 64);
        NCollection_Array1<double> ySol = new(1, 64);
        // As many slots as xSol and ySol (OCCT gave rSol 16: more than 16 triplets threw; Kerf decision 1540)
        NCollection_Array1<double> rSol = new(1, 64);
        NCollection_Array1<int> firstSol = new(1, 9);
        NCollection_Array1<double> xSol1 = new(1, 32);
        NCollection_Array1<double> ySol1 = new(1, 32);
        NCollection_Array1<double> rSol1 = new(1, 32);
        NCollection_Array1<int> firstSol1 = new(1, 9);
        double x, y, r;
        double m, n, t, s, v;
        double p, q;
        double epsilon;

        int curSol;

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

        isTouch =
          Math.Abs((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) - (r1 - r2) * (r1 - r2)) <= tolerance
          || Math.Abs((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) - (r1 + r2) * (r1 + r2)) <= tolerance
          || Math.Abs((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3) - (r1 - r3) * (r1 - r3)) <= tolerance
          || Math.Abs((x1 - x3) * (x1 - x3) + (y1 - y3) * (y1 - y3) - (r1 + r3) * (r1 + r3)) <= tolerance
          || Math.Abs((x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3) - (r2 - r3) * (r2 - r3)) <= tolerance
          || Math.Abs((x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3) - (r2 + r3) * (r2 + r3)) <= tolerance;

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
                if (Math.Abs(r1 - r2) > tolerance)
                {
                    beta2[i] = (x1 - x2) / (r1 - r2);
                    gamma2[i] = (y1 - y2) / (r1 - r2);
                    delta2[i] =
                      (x2 * x2 - x1 * x1 + y2 * y2 - y1 * y1 + (r1 - r2) * (r1 - r2)) / (2 * (r1 - r2));
                }
            }
            else
            {
                beta2[i] = (x1 - x2) / (r1 + r2);
                gamma2[i] = (y1 - y2) / (r1 + r2);
                delta2[i] = (x2 * x2 - x1 * x1 + y2 * y2 - y1 * y1 + (r1 + r2) * (r1 + r2)) / (2 * (r1 + r2));
            }
            if ((i == 1 || i == 4 || i == 5 || i == 8) && (Math.Abs(r1 - r2) <= tolerance))
            {
                //  If R1 = R2
                A2[i] = 0.0;
                B2[i] = 0.0;
                C2[i] = 0.0;
                D2[i] = x2 - x1;
                E2[i] = y2 - y1;
                F2[i] = x1 * x1 - x2 * x2 + y1 * y1 - y2 * y2;
            }
            else
            {
                A2[i] = beta2[i] * beta2[i] - 1.0;
                B2[i] = beta2[i] * gamma2[i];
                C2[i] = gamma2[i] * gamma2[i] - 1.0;
                D2[i] = beta2[i] * delta2[i] + x2;
                E2[i] = gamma2[i] * delta2[i] + y2;
                F2[i] = delta2[i] * delta2[i] - x2 * x2 - y2 * y2;
            }

            //    _
            //   | (X - X1)2 + (Y - Y1)2 = (R +- R1)2
            //  <
            //   \_(X - X3)2 + (Y - Y3)2 = (R +- R3)2

            if (i == 1 || i == 3 || i == 6 || i == 8)
            {
                if (Math.Abs(r1 - r3) > tolerance)
                {
                    beta3[i] = (x1 - x3) / (r1 - r3);
                    gamma3[i] = (y1 - y3) / (r1 - r3);
                    delta3[i] =
                      (x3 * x3 - x1 * x1 + y3 * y3 - y1 * y1 + (r1 - r3) * (r1 - r3)) / (2 * (r1 - r3));
                }
            }
            else
            {
                beta3[i] = (x1 - x3) / (r1 + r3);
                gamma3[i] = (y1 - y3) / (r1 + r3);
                delta3[i] = (x3 * x3 - x1 * x1 + y3 * y3 - y1 * y1 + (r1 + r3) * (r1 + r3)) / (2 * (r1 + r3));
            }
            if ((i == 1 || i == 3 || i == 6 || i == 8) && (Math.Abs(r1 - r3) <= tolerance))
            {
                A3[i] = 0.0;
                B3[i] = 0.0;
                C3[i] = 0.0;
                D3[i] = x3 - x1;
                E3[i] = y3 - y1;
                F3[i] = x1 * x1 - x3 * x3 + y1 * y1 - y3 * y3;
            }
            else
            {
                A3[i] = beta3[i] * beta3[i] - 1.0;
                B3[i] = beta3[i] * gamma3[i];
                C3[i] = gamma3[i] * gamma3[i] - 1.0;
                D3[i] = beta3[i] * delta3[i] + x3;
                E3[i] = gamma3[i] * delta3[i] + y3;
                F3[i] = delta3[i] * delta3[i] - x3 * x3 - y3 * y3;
            }
        }

        //   Second step:
        //     We are searching for the couple (X,Y) as a solution  of the system:
        //     _
        //    |  a2*x*x +2*b2*x*y + c2*y*y +2*d2*x + 2*e2*y + f2 = 0
        //   <
        //    \_ a3*x*x +2*b3*x*y + c3*y*y +2*d3*x + 2*e3*y + f3 = 0

        curSol = 1;
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

            firstSol[i] = curSol;

            //  In some cases we know that some systems have no solution in any case due to qualifiers
            if (((i == 2 || i == 5 || i == 6 || i == 8)
                 && (qualified1.IsEnclosed || qualified1.IsEnclosing))
                || ((i == 1 || i == 3 || i == 4 || i == 7) && qualified1.IsOutside))
            {
                continue;
            }

            if (((i == 3 || i == 5 || i == 7 || i == 8)
                 && (qualified2.IsEnclosed || qualified2.IsEnclosing))
                || ((i == 1 || i == 2 || i == 4 || i == 6) && qualified2.IsOutside))
            {
                continue;
            }

            if (((i == 4 || i == 6 || i == 7 || i == 8)
                 && (qualified3.IsEnclosed || qualified3.IsEnclosing))
                || ((i == 1 || i == 2 || i == 3 || i == 5) && qualified3.IsOutside))
            {
                continue;
            }

            // Check is Cir1 a solution of this system or not
            // In that case equations are equal to each other
            if (Math.Abs(a2 - a3) <= tolerance && Math.Abs(b2 - b3) <= tolerance
                && Math.Abs(c2 - c3) <= tolerance && Math.Abs(d2 - d3) <= tolerance
                && Math.Abs(e2 - e3) <= tolerance && Math.Abs(f2 - f3) <= tolerance)
            {
                xSol[curSol] = x1;
                ySol[curSol] = y1;
                curSol++;
                continue;
            }
            // 1) a2 = 0
            if (Math.Abs(a2) <= tolerance)
            {
                // 1.1) b2y + d2 = 0
                //   Searching for solution of the equation Ay2 + By + C = 0
                a = c2;
                b = 2.0 * e2;
                c = f2;
                DirectPolynomialRoots yRoots = new(a, b, c);
                if (yRoots.IsDone && !yRoots.InfiniteRoots())
                {
                    for (k = 1; k <= yRoots.SolutionCount; k++)
                    {
                        //   for each y solution:
                        y = yRoots.Value(k);
                        //   Searching for solution of the equation Ax2 + Bx + C = 0
                        if ((k != 2 || Math.Abs(y - yRoots.Value(1)) > 10 * tolerance)
                            && Math.Abs(b2 * y + d2) <= b2 * tolerance)
                        {
                            a = a3;
                            b = 2 * (b3 * y + d3);
                            c = c3 * (y * y) + 2 * e3 * y + f3;
                            DirectPolynomialRoots xRoots = new(a, b, c);
                            if (xRoots.IsDone && !xRoots.InfiniteRoots())
                            {
                                for (j = 1; j <= xRoots.SolutionCount; j++)
                                {
                                    x = xRoots.Value(j);
                                    if (j != 2 || Math.Abs(x - xRoots.Value(1)) > 10 * tolerance)
                                    {
                                        xSol[curSol] = x;
                                        ySol[curSol] = y;
                                        curSol++;
                                    }
                                }
                            }
                        }
                    }
                }

                // 1.2) b2y + d2 != 0
                a = a3 * c2 * c2 - 4 * b2 * (b3 * c2 - b2 * c3);
                b = 4 * a3 * c2 * e2 - 4 * b3 * (c2 * d2 + 2 * b2 * e2)
                    + 4 * b2 * (2 * c3 * d2 - c2 * d3 + 2 * b2 * e3);
                c = 2 * a3 * (c2 * f2 + 2 * e2 * e2) - 4 * b3 * (b2 * f2 + 2 * e2 * d2) + 4 * c3 * d2 * d2
                    - 4 * d3 * (c2 * d2 + 2 * b2 * e2) + 16 * b2 * e3 * d2 + 4 * b2 * b2 * f3;
                d = 4 * a3 * e2 * f2 - 4 * b3 * d2 * f2 - 4 * d3 * (b2 * f2 + 2 * d2 * e2) + 8 * d2 * d2 * e3
                    + 8 * b2 * d2 * f3;
                e = a3 * f2 * f2 - 4 * d2 * d3 * f2 + 4 * d2 * d2 * f3;

                //   Searching for solution of the equation Ay4 + By3 + Cy2 + Dy + E = 0
                // Special case: one circle touches other
                if (isTouch)
                {
                    // Derivation of the equation Ay4 + By3 + Cy2 + Dy + E
                    // yRootsD is OCCT's yRoots1 (C# forbids reusing the name of the yRoots1 declared below)
                    DirectPolynomialRoots yRootsD = new(4 * a, 3 * b, 2 * c, d);
                    if (yRootsD.IsDone && !yRootsD.InfiniteRoots())
                    {
                        for (k = 1; k <= yRootsD.SolutionCount; k++)
                        {
                            y = yRootsD.Value(k);
                            // Check if this value is already caught
                            isSame = false;
                            for (l = 1; l < k; l++)
                            {
                                if (Math.Abs(y - yRootsD.Value(l)) <= 10 * tolerance)
                                {
                                    isSame = true;
                                }
                            }

                            epsilon =
                              (Math.Abs((Math.Abs((Math.Abs(4 * a * y) + Math.Abs(3 * b)) * y) + Math.Abs(2 * c))
                                        * y)
                               + Math.Abs(d));
                            if (Math.Abs((((a * y + b) * y + c) * y + d) * y + e) <= epsilon * tolerance)
                            {
                                if (!isSame && Math.Abs(b2 * y + d2) > b2 * tolerance)
                                {
                                    x = -(c2 * (y * y) + 2 * e2 * y + f2) / (2 * (b2 * y + d2));
                                    xSol[curSol] = x;
                                    ySol[curSol] = y;
                                    curSol++;
                                }
                            }
                        }
                    }
                }

                DirectPolynomialRoots yRoots1 = new(a, b, c, d, e);
                if (yRoots1.IsDone && !yRoots1.InfiniteRoots())
                {
                    for (k = 1; k <= yRoots1.SolutionCount; k++)
                    {
                        y = yRoots1.Value(k);
                        // Check if this value is already caught
                        isSame = false;
                        firstIndex = (i == 1) ? 1 : firstSol[i];
                        for (l = firstIndex; l < curSol; l++)
                        {
                            if (Math.Abs(y - ySol[l]) <= 10 * tolerance)
                            {
                                isSame = true;
                            }
                        }

                        if (!isSame && Math.Abs(b2 * y + d2) > b2 * tolerance)
                        {
                            x = -(c2 * (y * y) + 2 * e2 * y + f2) / (2 * (b2 * y + d2));
                            xSol[curSol] = x;
                            ySol[curSol] = y;
                            curSol++;
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
                if (Math.Abs(s) <= tolerance && Math.Abs(v) <= tolerance)
                {
                    DirectPolynomialRoots yRoots = new(m, n, t);
                    if (yRoots.IsDone && !yRoots.InfiniteRoots())
                    {
                        for (k = 1; k <= yRoots.SolutionCount; k++)
                        {
                            //   for each y solution:
                            y = yRoots.Value(k);

                            p = -(b2 * y + d2) / a2;
                            q = (c2 * (y * y) + 2 * e2 * y + f2) / a2;
                            epsilon = 2.0
                                      * (Math.Abs((b2 * b2 + Math.Abs(a2 * c2)) * y) + Math.Abs(b2 * d2)
                                         + Math.Abs(a2 * e2))
                                      / (a2 * a2);
                            if ((k != 2 || Math.Abs(y - yRoots.Value(1)) > 10 * tolerance)
                                && p * p - q >= -epsilon * tolerance)
                            {
                                a = a2;
                                b = 2 * (b2 * y + d2);
                                c = c2 * y * y + 2 * e2 * y + f2;
                                DirectPolynomialRoots xRoots = new(a, b, c);
                                if (xRoots.IsDone && !xRoots.InfiniteRoots())
                                {
                                    for (l = 1; l <= xRoots.SolutionCount; l++)
                                    {
                                        //   for each x solution:
                                        x = xRoots.Value(l);

                                        if (l != 2 || Math.Abs(x - xRoots.Value(1)) > 10 * tolerance)
                                        {
                                            xSol[curSol] = x;
                                            ySol[curSol] = y;
                                            curSol++;
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

                    a = s * s * (b2 * b2 - a2 * c2) - m * m * a2 * a2;
                    b = 2 * s * v * (b2 * b2 - a2 * c2) + 2 * s * s * (b2 * d2 - a2 * e2) - 2 * m * n * a2 * a2;
                    c = v * v * (b2 * b2 - a2 * c2) + 4 * s * v * (b2 * d2 - a2 * e2)
                        + s * s * (d2 * d2 - a2 * f2) - a2 * a2 * (2 * m * t + n * n);
                    d = 2 * v * v * (b2 * d2 - a2 * e2) + 2 * s * v * (d2 * d2 - a2 * f2) - 2 * n * t * a2 * a2;
                    e = v * v * (d2 * d2 - a2 * f2) - t * t * a2 * a2;

                    //   Searching for solution of the equation Ay4 + By3 + Cy2 + Dy + E = 0
                    // Special case: one circle touches other
                    if (isTouch)
                    {
                        // Derivation of the equation Ay4 + By3 + Cy2 + Dy + E
                        DirectPolynomialRoots yRoots1 = new(4 * a, 3 * b, 2 * c, d);
                        if (yRoots1.IsDone && !yRoots1.InfiniteRoots())
                        {
                            for (k = 1; k <= yRoots1.SolutionCount; k++)
                            {
                                y = yRoots1.Value(k);

                                p = -(b2 * y + d2) / a2;
                                q = (c2 * (y * y) + 2 * e2 * y + f2) / a2;

                                // Check if this value is already caught
                                isSame = false;
                                firstIndex = (i == 1) ? 1 : firstSol[i];
                                for (l = firstIndex; l < curSol; l++)
                                {
                                    if (Math.Abs(y - ySol[l]) <= 10 * tolerance)
                                    {
                                        isSame = true;
                                    }
                                }

                                epsilon =
                                  (Math.Abs((Math.Abs((Math.Abs(4 * a * y) + Math.Abs(3 * b)) * y) + Math.Abs(2 * c))
                                            * y)
                                   + Math.Abs(d));
                                if (Math.Abs((((a * y + b) * y + c) * y + d) * y + e) <= epsilon * tolerance)
                                {
                                    epsilon = 2.0
                                              * (Math.Abs((b2 * b2 + Math.Abs(a2 * c2)) * y) + Math.Abs(b2 * d2)
                                                 + Math.Abs(a2 * e2))
                                              / (a2 * a2);
                                    if (!isSame && p * p - q >= -epsilon * tolerance)
                                    {
                                        a = a2;
                                        b = 2 * (b2 * y + d2);
                                        c = c2 * y * y + 2 * e2 * y + f2;
                                        DirectPolynomialRoots xRoots = new(a, b, c);
                                        if (xRoots.IsDone && !xRoots.InfiniteRoots())
                                        {
                                            for (l = 1; l <= xRoots.SolutionCount; l++)
                                            {
                                                //   for each x solution:
                                                x = xRoots.Value(l);

                                                if (l != 2 || Math.Abs(x - xRoots.Value(1)) > 10 * tolerance)
                                                {
                                                    xSol[curSol] = x;
                                                    ySol[curSol] = y;
                                                    curSol++;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    DirectPolynomialRoots yRoots = new(a, b, c, d, e);
                    if (yRoots.IsDone && !yRoots.InfiniteRoots())
                    {
                        for (k = 1; k <= yRoots.SolutionCount; k++)
                        {
                            //   for each y solution:
                            y = yRoots.Value(k);

                            p = -(b2 * y + d2) / a2;
                            q = (c2 * (y * y) + 2 * e2 * y + f2) / a2;

                            // Check if this value is already caught
                            isSame = false;
                            for (l = 1; l < k; l++)
                            {
                                if (Math.Abs(y - yRoots.Value(l)) <= 10 * tolerance)
                                {
                                    isSame = true;
                                }
                            }

                            epsilon = 2.0
                                      * (Math.Abs((b2 * b2 + Math.Abs(a2 * c2)) * y) + Math.Abs(b2 * d2)
                                         + Math.Abs(a2 * e2))
                                      / (a2 * a2);
                            if (!isSame && p * p - q >= -epsilon * tolerance)
                            {
                                a = a2;
                                b = 2 * (b2 * y + d2);
                                c = c2 * y * y + 2 * e2 * y + f2;
                                DirectPolynomialRoots xRoots = new(a, b, c);
                                if (xRoots.IsDone && !xRoots.InfiniteRoots())
                                {
                                    for (l = 1; l <= xRoots.SolutionCount; l++)
                                    {
                                        //   for each x solution:
                                        x = xRoots.Value(l);

                                        if (l != 2 || Math.Abs(x - xRoots.Value(1)) > 10 * tolerance)
                                        {
                                            xSol[curSol] = x;
                                            ySol[curSol] = y;
                                            curSol++;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        firstSol[9] = curSol;

        //  Third step:
        //    Check of couples (X,Y) and searching for R. R must be great than 0
        curSol = 1;
        for (i = 1; i <= 8; i++)
        {
            firstSol1[i] = curSol;
            for (j = firstSol[i]; j < firstSol[i + 1]; j++)
            {
                x = xSol[j];
                y = ySol[j];
                // in some cases when R1 = R2 :
                if ((i == 1 || i == 4 || i == 5 || i == 8) && (Math.Abs(r1 - r2) <= tolerance))
                {
                    if (i == 1 || i == 4)
                    {
                        r = r1 + Math.Sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
                        epsilon = 10 * (2 * Math.Abs(r - r2) + Math.Abs(x - x2) + Math.Abs(y - y2));
                        if (Math.Abs((r - r2) * (r - r2) - (x - x2) * (x - x2) - (y - y2) * (y - y2))
                            <= epsilon * tolerance)
                        {
                            xSol1[curSol] = x;
                            ySol1[curSol] = y;
                            rSol1[curSol] = r;
                            curSol++;
                        }
                        r = r1 - Math.Sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
                        epsilon = 10 * (2 * Math.Abs(r - r2) + Math.Abs(x - x2) + Math.Abs(y - y2));
                        if ((r > tolerance)
                            && (Math.Abs((r - r2) * (r - r2) - (x - x2) * (x - x2) - (y - y2) * (y - y2))
                                <= epsilon * tolerance))
                        {
                            xSol1[curSol] = x;
                            ySol1[curSol] = y;
                            rSol1[curSol] = r;
                            curSol++;
                        }
                    }
                    else
                    {
                        //        i == 5 || i == 8
                        r = -r1 + Math.Sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
                        if (r > tolerance)
                        {
                            xSol1[curSol] = x;
                            ySol1[curSol] = y;
                            rSol1[curSol] = r;
                            curSol++;
                        }
                    }
                }
                else
                {
                    // Other cases
                    if (i == 1 || i == 4)
                    {
                        r = r2 + beta2[i] * x + gamma2[i] * y + delta2[i];
                        if (r > tolerance)
                        {
                            xSol1[curSol] = x;
                            ySol1[curSol] = y;
                            rSol1[curSol] = r;
                            curSol++;
                        }
                    }
                    if (i == 5 || i == 8)
                    {
                        r = -r2 - beta2[i] * x - gamma2[i] * y - delta2[i];
                        if (r > tolerance)
                        {
                            xSol1[curSol] = x;
                            ySol1[curSol] = y;
                            rSol1[curSol] = r;
                            curSol++;
                        }
                    }
                    if (i == 3 || i == 7)
                    {
                        r = -r2 + beta2[i] * x + gamma2[i] * y + delta2[i];
                        if (r > tolerance)
                        {
                            xSol1[curSol] = x;
                            ySol1[curSol] = y;
                            rSol1[curSol] = r;
                            curSol++;
                        }
                    }
                    if (i == 2 || i == 6)
                    {
                        r = r2 - beta2[i] * x - gamma2[i] * y - delta2[i];
                        if (r > tolerance)
                        {
                            xSol1[curSol] = x;
                            ySol1[curSol] = y;
                            rSol1[curSol] = r;
                            curSol++;
                        }
                    }
                }
            }
        }
        firstSol1[9] = curSol;
        //   Fourth step
        //     Check of triplets (X,Y,R).
        curSol = 1;
        for (i = 1; i <= 8; i++)
        {
            firstSol[i] = curSol;
            for (j = firstSol1[i]; j < firstSol1[i + 1]; j++)
            {
                x = xSol1[j];
                y = ySol1[j];
                r = rSol1[j];
                // in some cases when R1 = R3 :
                if ((i == 1 || i == 3 || i == 6 || i == 8) && Math.Abs(r1 - r3) <= tolerance)
                {
                    if (i == 1 || i == 3)
                    {
                        epsilon = 10 * (2 * Math.Abs(r - r3) + Math.Abs(x - x3) + Math.Abs(y - y3));
                        if (Math.Abs((r - r3) * (r - r3) - (x - x3) * (x - x3) - (y - y3) * (y - y3))
                            <= epsilon * tolerance)
                        {
                            xSol[curSol] = x;
                            ySol[curSol] = y;
                            rSol[curSol] = r;
                            curSol++;
                        }
                    }
                    else
                    {
                        //        i == 6 || i == 8
                        epsilon = 10 * (2 * (r + r3) + Math.Abs(x - x3) + Math.Abs(y - y3));
                        if (Math.Abs((r + r3) * (r + r3) - (x - x3) * (x - x3) - (y - y3) * (y - y3))
                            <= epsilon * tolerance)
                        {
                            xSol[curSol] = x;
                            ySol[curSol] = y;
                            rSol[curSol] = r;
                            curSol++;
                        }
                    }
                }
                else
                {
                    // Other cases
                    epsilon = 10 * (Math.Abs(beta3[i]) + Math.Abs(gamma3[i]) + 1.0);
                    if (i == 1 || i == 3)
                    {
                        if (Math.Abs(r3 + beta3[i] * x + gamma3[i] * y + delta3[i] - r) <= epsilon * tolerance)
                        {
                            xSol[curSol] = x;
                            ySol[curSol] = y;
                            rSol[curSol] = r;
                            curSol++;
                        }
                    }
                    if (i == 6 || i == 8)
                    {
                        if (Math.Abs(r3 + beta3[i] * x + gamma3[i] * y + delta3[i] + r) <= epsilon * tolerance)
                        {
                            xSol[curSol] = x;
                            ySol[curSol] = y;
                            rSol[curSol] = r;
                            curSol++;
                        }
                    }
                    if (i == 4 || i == 7)
                    {
                        if (Math.Abs(beta3[i] * x + gamma3[i] * y + delta3[i] - r - r3) <= epsilon * tolerance)
                        {
                            xSol[curSol] = x;
                            ySol[curSol] = y;
                            rSol[curSol] = r;
                            curSol++;
                        }
                    }
                    if (i == 2 || i == 5)
                    {
                        if (Math.Abs(r - r3 + beta3[i] * x + gamma3[i] * y + delta3[i]) <= epsilon * tolerance)
                        {
                            xSol[curSol] = x;
                            ySol[curSol] = y;
                            rSol[curSol] = r;
                            curSol++;
                        }
                    }
                }
            }
        }
        firstSol[9] = curSol;

        //  Fifth step:
        //    We have found all solutions. We have to calculate some parameters for each one.
        for (i = 1; i <= 8; i++)
        {
            for (j = firstSol[i]; j < firstSol[i + 1]; j++)
            {
                if ((qualified1.IsEnclosed && rSol[j] > r1) || (qualified1.IsEnclosing && rSol[j] < r1))
                {
                    continue;
                }
                if ((qualified2.IsEnclosed && rSol[j] > r2) || (qualified2.IsEnclosing && rSol[j] < r2))
                {
                    continue;
                }
                if ((qualified3.IsEnclosed && rSol[j] > r3) || (qualified3.IsEnclosing && rSol[j] < r3))
                {
                    continue;
                }

                _nbrSol++;

                // RLE, avoid out of range
                if (_nbrSol > _cirsol.Upper())
                {
                    _nbrSol = _cirsol.Upper();
                }

                Point2D center = new Point2D(xSol[j], ySol[j]);

                _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), rSol[j]);

                //   ==========================================================
                // The system solved says which circles the solution touches from outside (R + Ri), and the
                // radii which of the others it encloses (OCCT compared distances, two of them measured from
                // center1: Kerf decision 1540)
                bool out1 = i == 2 || i == 5 || i == 6 || i == 8;
                bool out2 = i == 3 || i == 5 || i == 7 || i == 8;
                bool out3 = i == 4 || i == 6 || i == 7 || i == 8;
                _qualifier1[_nbrSol] = !qualified1.IsUnqualified ? qualified1.Qualifier()
                                       : out1 ? Qualifier.Outside : rSol[j] > r1 ? Qualifier.Enclosing : Qualifier.Enclosed;
                _qualifier2[_nbrSol] = !qualified2.IsUnqualified ? qualified2.Qualifier()
                                       : out2 ? Qualifier.Outside : rSol[j] > r2 ? Qualifier.Enclosing : Qualifier.Enclosed;
                _qualifier3[_nbrSol] = !qualified3.IsUnqualified ? qualified3.Qualifier()
                                       : out3 ? Qualifier.Outside : rSol[j] > r3 ? Qualifier.Enclosing : Qualifier.Enclosed;

                //   ==========================================================

                if (center.DistanceTo(cir1.Location) <= tolerance)
                {
                    _theSame1[_nbrSol] = 1;
                }
                else
                {
                    _theSame1[_nbrSol] = 0;
                    Direction2D dc = new();
                    if ((i == 2 || i == 5 || i == 6 || i == 8) || rSol[j] > r1)
                    {
                        dc = dc.WithXY(cir1.Location.XY() - center.XY());
                    }
                    else
                    {
                        dc = dc.WithXY(center.XY() - cir1.Location.XY());
                    }
                    _pnttg1sol[_nbrSol] = new Point2D(center.XY() + rSol[j] * dc.XY());
                    _par1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                    _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir1, _pnttg1sol[_nbrSol]);
                }

                if (center.DistanceTo(cir2.Location) <= tolerance)
                {
                    _theSame2[_nbrSol] = 1;
                }
                else
                {
                    _theSame2[_nbrSol] = 0;
                    Direction2D dc = new();
                    if ((i == 3 || i == 5 || i == 7 || i == 8) || rSol[j] > r2)
                    {
                        dc = dc.WithXY(cir2.Location.XY() - center.XY());
                    }
                    else
                    {
                        dc = dc.WithXY(center.XY() - cir2.Location.XY());
                    }
                    _pnttg2sol[_nbrSol] = new Point2D(center.XY() + rSol[j] * dc.XY());
                    _par2sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                    _pararg2[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir2, _pnttg2sol[_nbrSol]);
                }

                if (center.DistanceTo(cir3.Location) <= tolerance)
                {
                    _theSame3[_nbrSol] = 1;
                }
                else
                {
                    _theSame3[_nbrSol] = 0;
                    Direction2D dc = new();
                    if ((i == 4 || i == 6 || i == 7 || i == 8) || rSol[j] > r3)
                    {
                        dc = dc.WithXY(cir3.Location.XY() - center.XY());
                    }
                    else
                    {
                        dc = dc.WithXY(center.XY() - cir3.Location.XY());
                    }
                    _pnttg3sol[_nbrSol] = new Point2D(center.XY() + rSol[j] * dc.XY());
                    _par3sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg3sol[_nbrSol]);
                    _pararg3[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir3, _pnttg3sol[_nbrSol]);
                }
            }
        }
        _wellDone = true;
    }

    /// <summary>
    /// Adds a candidate circle if it touches the three arguments as their qualifiers ask, within the tolerance, and
    /// isn't a solution already: with the qualifiers it has, and the tangency points where it touches (Kerf decision
    /// 17101).
    /// </summary>
    private void AddSolution(in Circle2D sol, in CircleContact a1, in CircleContact a2, in CircleContact a3, double tolerance)
    {
        if (!a1.Touches(sol, tolerance, out Qualifier q1) || !a2.Touches(sol, tolerance, out Qualifier q2)
            || !a3.Touches(sol, tolerance, out Qualifier q3))
        {
            return;
        }
        for (int i = 1; i <= _nbrSol; i++)
        {
            if (_cirsol[i].Location.DistanceTo(sol.Location) <= tolerance && Math.Abs(_cirsol[i].Radius - sol.Radius) <= tolerance)
            {
                return;
            }
        }
        if (_nbrSol == _cirsol.Upper())
        {
            return;
        }
        int n = ++_nbrSol;
        _cirsol[n] = sol;
        _qualifier1[n] = q1;
        _qualifier2[n] = q2;
        _qualifier3[n] = q3;
        _theSame1[n] = a1.Touch(n, sol, q1, tolerance, _pnttg1sol, _par1sol, _pararg1);
        _theSame2[n] = a2.Touch(n, sol, q2, tolerance, _pnttg2sol, _par2sol, _pararg2);
        _theSame3[n] = a3.Touch(n, sol, q3, tolerance, _pnttg3sol, _par3sol, _pararg3);
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>The number of solutions.</summary>
    public int SolutionCount
    {
        get
        {
            return _nbrSol;
        }
    }

    /// <summary>
    /// The solution number Index. The index only enumerates the solutions of this algorithm object.
    /// </summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is outside [1, NbSolutions()].</exception>
    public Circle2D ThisSolution(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }

        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }

        return _cirsol[index];
    }

    /// <summary>
    /// The qualifiers of the three arguments for the solution number Index: the given ones where qualified, the ones
    /// computed for the solution where unqualified.
    /// </summary>
    public void WhichQualifier(int index, out Qualifier qualif1, out Qualifier qualif2, out Qualifier qualif3)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            qualif1 = _qualifier1[index];
            qualif2 = _qualifier2[index];
            qualif3 = _qualifier3[index];
        }
    }

    /// <summary>
    /// The tangency point of the solution number Index with argument 1: ParSol its parameter on the solution, ParArg
    /// on the argument (0 for a point).
    /// </summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the argument itself (IsTheSame1).</exception>
    public void Tangency1(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            if (_theSame1[index] == 0)
            {
                parSol = _par1sol[index];
                parArg = _pararg1[index];
                pntSol = _pnttg1sol[index];
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>
    /// The tangency point of the solution number Index with argument 2: ParSol its parameter on the solution, ParArg
    /// on the argument (0 for a point).
    /// </summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the argument itself (IsTheSame2).</exception>
    public void Tangency2(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            if (_theSame2[index] == 0)
            {
                parSol = _par2sol[index];
                parArg = _pararg2[index];
                pntSol = _pnttg2sol[index];
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>
    /// The tangency point of the solution number Index with argument 3: ParSol its parameter on the solution, ParArg
    /// on the argument (0 for a point).
    /// </summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the argument itself (IsTheSame3).</exception>
    public void Tangency3(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            if (_theSame3[index] == 0)
            {
                parSol = _par3sol[index];
                parArg = _pararg3[index];
                pntSol = _pnttg3sol[index];
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>True if the solution number Index is argument 1 itself.</summary>
    public bool IsTheSame1(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }

        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }

        if (_theSame1[index] == 0)
        {
            return false;
        }

        return true;
    }

    /// <summary>True if the solution number Index is argument 2 itself.</summary>
    public bool IsTheSame2(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }

        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }

        if (_theSame2[index] == 0)
        {
            return false;
        }

        return true;
    }

    /// <summary>True if the solution number Index is argument 3 itself.</summary>
    public bool IsTheSame3(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }

        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }

        if (_theSame3[index] == 0)
        {
            return false;
        }

        return true;
    }
}
