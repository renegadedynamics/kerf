// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2dTanOnRad
{
    //=========================================================================
    //    typedef of handled objects :                                      +
    //=========================================================================

    //=========================================================================
    //   Circle tangent to a point      Point1.                               +
    //          center on straight line OnLine.                               +
    //          radius                  Radius.                               +
    //                                                                        +
    //  Initialize the table of solutions cirsol and all fields.              +
    //  Eliminate cases not being the solution.                     +
    //  Solve the equation of second degree showing that the found center point +
    //  (xc,yc) is at distance Radius from point Point1 and on the straight line OnLine. +
    //  The solutions are represented by circles :                     +
    //                   - of center Pntcen(xc,yc)                            +
    //                   - of radius Radius.                                   +
    //=========================================================================

    /// <summary>Circles of radius Radius passing through a point, centred on a line.</summary>
    /// <exception cref="Standard_NegativeValue">Radius is negative.</exception>
    public GccAna_Circ2dTanOnRad(in gp_Pnt2d Point1, in gp_Lin2d OnLine, double Radius, double Tolerance)
    {
        cirsol = new(1, 2);
        qualifier1 = new(1, 2);
        TheSame1 = new(1, 2);
        pnttg1sol = new(1, 2);
        pntcen3 = new(1, 2);
        par1sol = new(1, 2);
        pararg1 = new(1, 2);
        parcen3 = new(1, 2);


        gp_Dir2d dirx = new(gp_Dir2d.D.X);
        double   Tol  = Math.Abs(Tolerance);
        WellDone      = false;
        NbrSol        = 0;
        double dp1lin = OnLine.Distance(Point1);

        if (Radius < 0.0)
        {
            throw new Standard_NegativeValue();
        }
        else
        {
            // OCCT: WellDone stays false when the point is on OnLine (dp1lin < Tol), which still finds 2 solutions
            if (dp1lin > Radius + Tol)
            {
                WellDone = true;
            }
            double xc;
            double yc;
            double x1    = Point1.X();
            double y1    = Point1.Y();
            double xbid  = 0;
            double xdir  = (OnLine.Direction()).X();
            double ydir  = (OnLine.Direction()).Y();
            double lxloc = (OnLine.Location()).X();
            double lyloc = (OnLine.Location()).Y();
            if (Math.Abs(dp1lin - Radius) < Tol)
            {
                WellDone = true;
                NbrSol   = 1;
                if (-ydir * (x1 - lxloc) + xdir * (y1 - lyloc) < 0.0)
                {
                    gp_Ax2d axe = new(new gp_Pnt2d(x1 - ydir * dp1lin, y1 + xdir * dp1lin), dirx);
                    cirsol[NbrSol] = new gp_Circ2d(axe, Radius);
                    //       ======================================
                    qualifier1[NbrSol] = GccEnt_noqualifier;
                }
                else
                {
                    gp_Ax2d axe = new(new gp_Pnt2d(x1 + ydir * dp1lin, y1 - xdir * dp1lin), dirx);
                    cirsol[NbrSol] = new gp_Circ2d(axe, Radius);
                    //       ======================================
                    qualifier1[NbrSol] = GccEnt_noqualifier;
                }
                TheSame1[NbrSol]  = 0;
                pnttg1sol[NbrSol] = Point1;
                pntcen3[NbrSol]   = cirsol[NbrSol].Location();
                pararg1[NbrSol]   = 0.0;
                par1sol[NbrSol]   = GccAna_ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                parcen3[NbrSol]   = GccAna_ElCLib.Parameter(OnLine, pntcen3[NbrSol]);
            }
            else if (dp1lin < Tol)
            {
                // OCCT: qualifier1 is not set on this path (uninitialised in OCCT, GccEnt_unqualified here)
                pntcen3[1] = new gp_Pnt2d(Point1.X() + Radius * xdir, Point1.Y() + Radius * ydir);
                pntcen3[2] = new gp_Pnt2d(Point1.X() - Radius * xdir, Point1.Y() - Radius * ydir);
                pntcen3[1] = ElCLib.Value(GccAna_ElCLib.Parameter(OnLine, pntcen3[1]), OnLine);
                pntcen3[2] = ElCLib.Value(GccAna_ElCLib.Parameter(OnLine, pntcen3[2]), OnLine);
                gp_Ax2d axe = new(pntcen3[1], OnLine.Direction());
                cirsol[1]    = new gp_Circ2d(axe, Radius);
                axe          = new gp_Ax2d(pntcen3[2], OnLine.Direction());
                cirsol[2]    = new gp_Circ2d(axe, Radius);
                TheSame1[1]  = 0;
                pnttg1sol[1] = Point1;
                pararg1[1]   = 0.0;
                par1sol[1]   = GccAna_ElCLib.Parameter(cirsol[1], pnttg1sol[1]);
                parcen3[1]   = GccAna_ElCLib.Parameter(OnLine, pntcen3[1]);
                TheSame1[2]  = 0;
                pnttg1sol[2] = Point1;
                pararg1[2]   = 0.0;
                par1sol[2]   = GccAna_ElCLib.Parameter(cirsol[2], pnttg1sol[2]);
                parcen3[2]   = GccAna_ElCLib.Parameter(OnLine, pntcen3[2]);
                NbrSol       = 2;
            }
            else
            {
                double A, B, C;
                OnLine.Coefficients(out A, out B, out C);
                double D = A;
                if (A == 0.0)
                {
                    A    = B;
                    B    = D;
                    xbid = x1;
                    x1   = y1;
                    y1   = xbid;
                }
                if (A != 0.0)
                {
                    math_DirectPolynomialRoots Sol = new((B * B + A * A) / (A * A),
                                        2.0 * (B * C / (A * A) + (B / A) * x1 - y1),
                                        x1 * x1 + y1 * y1 + C * C / (A * A) - Radius * Radius + 2.0 * C * x1 / A);
                    if (Sol.IsDone())
                    {
                        for (int i = 1; i <= Sol.NbSolutions(); i++)
                        {
                            if (D != 0.0)
                            {
                                yc = Sol.Value(i);
                                xc = -(B / A) * yc - C / A;
                            }
                            else
                            {
                                xc = Sol.Value(i);
                                yc = -(B / A) * xc - C / A;
                            }
                            NbrSol++;
                            gp_Pnt2d Center = new(xc, yc);
                            cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                            //           =======================================================
                            qualifier1[NbrSol] = GccEnt_noqualifier;
                            TheSame1[NbrSol]   = 0;
                            pnttg1sol[NbrSol]  = Point1;
                            pntcen3[NbrSol]    = cirsol[NbrSol].Location();
                            pararg1[NbrSol]    = 0.0;
                            par1sol[NbrSol]    = GccAna_ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                            parcen3[NbrSol]    = GccAna_ElCLib.Parameter(OnLine, pntcen3[NbrSol]);
                        }
                        WellDone = true;
                    }
                }
            }
        }
    }
}
