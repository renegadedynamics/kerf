// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn_5.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanOn
{
    //=========================================================================
    //   Creation of a circle passing by : 2 points Point1 and Point2.        +
    //                        Centered on  : 1 straight line OnLine.                  +
    //   with a Tolerance of precision  : Tolerance.                        +
    //                                                                        +
    //   Create L1 straight line of points equidistant from Point1 and Point2.     +
    //   Then create solutions cirsol such as :                        +
    //   cirsol is all circle with center at the intersections of  +
    //   L1 with OnLine and the radius the distance between Point1 and the point   +
    //   calculated above.                                                   +
    //=========================================================================

    /// <summary>The circles through two points with their centre on a line.</summary>
    public GccAna_Circ2d2TanOn(in gp_Pnt2d Point1,
                               in gp_Pnt2d Point2,
                               in gp_Lin2d OnLine,
                               double Tolerance)
        : this(2)
    {
        TheSame1.Init(0);
        TheSame2.Init(0);
        WellDone = false;
        NbrSol = 0;

        gp_Dir2d dirx = new(gp_Dir2d.D.X);
        double dist = Point1.Distance(Point2);
        if (dist < Math.Abs(Tolerance))
        {
            WellDone = true;
        }
        else
        {
            gp_Lin2d L1 = new(new gp_Pnt2d((Point1.XY() + Point2.XY()) / 2.0),
                              new gp_Dir2d(Point1.Y() - Point2.Y(), Point2.X() - Point1.X()));
            IntAna2d_AnaIntersection Intp = new(L1, OnLine);
            if (Intp.IsDone())
            {
                if (!Intp.IsEmpty())
                {
                    for (int i = 1; i <= Intp.NbPoints(); i++)
                    {
                        NbrSol++;
                        gp_Ax2d axe = new(Intp.Point(i).Value(), dirx);
                        cirsol[NbrSol] = new gp_Circ2d(axe, Point1.Distance(Intp.Point(i).Value()));
                        //        ====================================================================
                        qualifier1[NbrSol] = GccEnt_noqualifier;
                        qualifier2[NbrSol] = GccEnt_noqualifier;
                        pnttg1sol[NbrSol] = Point1;
                        pnttg2sol[NbrSol] = Point2;
                        pntcen[NbrSol] = cirsol[NbrSol].Location();
                        pararg1[NbrSol] = 0.0;
                        pararg2[NbrSol] = 0.0;
                        par1sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                        par2sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                        parcen3[NbrSol] = ElCLib.Parameter(OnLine, pntcen[NbrSol]);
                    }
                }
                WellDone = true;
            }
        }
    }
}
