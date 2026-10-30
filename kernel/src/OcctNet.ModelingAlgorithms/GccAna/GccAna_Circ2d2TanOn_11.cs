// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_11.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanOn
{
    //=========================================================================
    //   Creation of a circle passing by  : 2 points Point1 and Point2.        +
    //                        Center on   : 1 circle OnCirc.                  +
    //   with a Tolerance of precision    : Tolerance.                        +
    //                                                                        +
    //   Create L1 the straight line of points equidistant to Point1 and Point2.     +
    //   Create then solutions cirsol so that :                      +
    //   cirsol is the set of circles with center in one of inter-    +
    //   sections of L1 with OnCirc and the radius the distance between Point1 and   +
    //   the point calculated below.                                          +
    //=========================================================================

    /// <summary>The circles through two points with their centre on a circle.</summary>
    public GccAna_Circ2d2TanOn(in gp_Pnt2d Point1,
                               in gp_Pnt2d Point2,
                               in gp_Circ2d OnCirc,
                               double Tolerance)
        : this(2)
    {
        TheSame1.Init(0);
        TheSame2.Init(0);
        WellDone = false;
        NbrSol = 0;
        gp_Dir2d dirx = new(gp_Dir2d.D.X);
        double Tol = Math.Abs(Tolerance);
        double dist = Point1.Distance(Point2);
        double dp1cen = Point1.Distance(OnCirc.Location());
        double dp2cen = Point2.Distance(OnCirc.Location());
        double R = OnCirc.Radius();
        gp_Circ2d C1 = OnCirc;
        if (dist < Tol || Math.Abs(dp1cen + 2 * R - dp2cen) < Tol
            || Math.Abs(dp2cen + 2 * R - dp1cen) < Tol)
        {
            WellDone = true;
            return;
        }
        gp_Lin2d L1 = new(new gp_Pnt2d((Point1.XY() + Point2.XY()) / 2.0),
                          new gp_Dir2d(Point1.Y() - Point2.Y(), Point2.X() - Point1.X()));
        // OCCT: never true, since the same conditions returned above (and the inner branches test the same one)
        if (Math.Abs(dp1cen + 2 * R - dp2cen) < Tol || Math.Abs(dp2cen + 2 * R - dp1cen) < Tol)
        {
            if (Math.Abs(dp1cen + 2 * R - dp2cen) < Tol)
            {
                C1 = new gp_Circ2d(new gp_Ax2d(OnCirc.Location(), dirx),
                                   OnCirc.Radius() + Math.Abs(dp2cen - dp1cen - 2.0 * R));
            }
            else if (Math.Abs(dp1cen + 2 * R - dp2cen) < Tol)
            {
                C1 = new gp_Circ2d(new gp_Ax2d(OnCirc.Location(), dirx),
                                   OnCirc.Radius() + Math.Abs(dp2cen - dp1cen - 2.0 * R));
            }
        }
        IntAna2d_AnaIntersection Intp = new(L1, C1);
        if (Intp.IsDone())
        {
            if (!Intp.IsEmpty())
            {
                for (int i = 1; i <= Intp.NbPoints(); i++)
                {
                    NbrSol++;
                    gp_Ax2d axe = new(Intp.Point(i).Value(), dirx);
                    cirsol[NbrSol] = new gp_Circ2d(axe, Point1.Distance(Intp.Point(i).Value()));
                    //      ======================================================================
                    qualifier1[NbrSol] = GccEnt_noqualifier;
                    qualifier2[NbrSol] = GccEnt_noqualifier;
                    pnttg1sol[NbrSol] = Point1;
                    pararg1[NbrSol] = 0.0;
                    par1sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                    pnttg2sol[NbrSol] = Point2;
                    pararg2[NbrSol] = 0.0;
                    par2sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                    pntcen[NbrSol] = cirsol[NbrSol].Location();
                    parcen3[NbrSol] = ElCLib.Parameter(OnCirc, pntcen[NbrSol]);
                }
            }
            WellDone = true;
        }
    }
}
