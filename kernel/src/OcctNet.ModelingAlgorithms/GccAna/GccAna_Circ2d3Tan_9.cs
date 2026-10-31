// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d3Tan_9.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2d3Tan
{
    //=========================================================================
    //   Creation of a circle passing by three points.                       +
    //   Three cases  :                                               +
    //      1/ Three points coincide.                               +
    //      -----------------------------------                               +
    //      The result is the circle with center in Point1 with zero radius.  +
    //      2/ Two of three points coincide.                          +
    //      ----------------------------------------                          +
    //      Create the medium line between two non-coinciding points and      +
    //      the straight line passing by these two points.                               +
    //      The center of the solution is the intersection of two straight lines and the   +
    //      radius is the distance between this center and one of three points.  +
    //      3/ The three points are distinct.                                +
    //      ----------------------------------                                +
    //=========================================================================
    public GccAna_Circ2d3Tan(in gp_Pnt2d Point1,
        in gp_Pnt2d Point2,
        in gp_Pnt2d Point3,
        double Tolerance)
    {
        cirsol = new NCollection_Array1<gp_Circ2d>(1, 1);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, 1);
        qualifier2 = new NCollection_Array1<GccEnt_Position>(1, 1);
        qualifier3 = new NCollection_Array1<GccEnt_Position>(1, 1);
        TheSame1 = new NCollection_Array1<int>(1, 1);
        TheSame2 = new NCollection_Array1<int>(1, 1);
        TheSame3 = new NCollection_Array1<int>(1, 1);
        pnttg1sol = new NCollection_Array1<gp_Pnt2d>(1, 1);
        pnttg2sol = new NCollection_Array1<gp_Pnt2d>(1, 1);
        pnttg3sol = new NCollection_Array1<gp_Pnt2d>(1, 1);
        par1sol = new NCollection_Array1<double>(1, 1);
        par2sol = new NCollection_Array1<double>(1, 1);
        par3sol = new NCollection_Array1<double>(1, 1);
        pararg1 = new NCollection_Array1<double>(1, 1);
        pararg2 = new NCollection_Array1<double>(1, 1);
        pararg3 = new NCollection_Array1<double>(1, 1);

        gp_Dir2d dirx = new(gp_Dir2d.D.X);
        WellDone = false;
        NbrSol = 0;

        //=========================================================================
        //   Processing.                                                          +
        //=========================================================================

        double dist1 = Point1.Distance(Point2);
        double dist2 = Point1.Distance(Point3);
        double dist3 = Point2.Distance(Point3);

        qualifier1[1] = GccEnt_noqualifier;
        qualifier2[1] = GccEnt_noqualifier;
        qualifier3[1] = GccEnt_noqualifier;

        if ((dist1 < Tolerance) && (dist2 < Tolerance) && (dist3 < Tolerance))
        {
            NbrSol++;
            WellDone = true;
            cirsol[1] = new gp_Circ2d(new gp_Ax2d(Point1, dirx), 0.0);
            //   ===============================================
            TheSame1[1] = 0;
            TheSame2[1] = 0;
            TheSame3[1] = 0;
            pnttg1sol[1] = Point1;
            pnttg2sol[1] = Point2;
            pnttg3sol[1] = Point3;
            par1sol[1] = 0.0;
            par2sol[1] = 0.0;
            par3sol[1] = 0.0;
            pararg1[1] = 0.0;
            pararg2[1] = 0.0;
            pararg3[1] = 0.0;
        }
        else
        {
            gp_Lin2d L1 = new();
            gp_Lin2d L2 = new();
            if (dist1 >= Tolerance)
            {
                L1 = new gp_Lin2d(new gp_Pnt2d((Point1.XY() + Point2.XY()) / 2.0),
                              new gp_Dir2d(Point1.Y() - Point2.Y(), Point2.X() - Point1.X()));
            }
            if (dist2 >= Tolerance)
            {
                L2 = new gp_Lin2d(new gp_Pnt2d((Point1.XY() + Point3.XY()) / 2.0),
                              new gp_Dir2d(Point1.Y() - Point3.Y(), Point3.X() - Point1.X()));
            }
            if (dist2 <= Tolerance)
            {
                L2 = new gp_Lin2d(Point1, new gp_Dir2d(Point1.Y() - Point2.Y(), Point2.X() - Point1.X()));
            }
            else if (dist1 <= Tolerance)
            {
                L1 = new gp_Lin2d(Point1, new gp_Dir2d(Point1.Y() - Point3.Y(), Point3.X() - Point1.X()));
            }
            else if (dist3 <= Tolerance)
            {
                L2 = new gp_Lin2d(Point1, new gp_Dir2d(Point1.Y() - Point2.Y(), Point2.X() - Point1.X()));
            }
            IntAna2d_AnaIntersection Intp = new(L1, L2);
            if (Intp.IsDone())
            {
                if (!Intp.IsEmpty())
                {
                    for (int i = 1; i <= Intp.NbPoints(); i++)
                    {
                        NbrSol++;
                        cirsol[NbrSol] =
                          new gp_Circ2d(new gp_Ax2d(Intp.Point(i).Value(), dirx),
                                    //         ===============================================================
                                    Point1.Distance(Intp.Point(i).Value()));
                        //                                    =======================================

                        TheSame1[NbrSol] = 0;
                        TheSame2[NbrSol] = 0;
                        TheSame3[NbrSol] = 0;
                        pnttg1sol[NbrSol] = Point1;
                        pnttg2sol[NbrSol] = Point2;
                        pnttg3sol[NbrSol] = Point3;
                        par1sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                        par2sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                        par3sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg3sol[NbrSol]);
                        pararg1[NbrSol] = 0.0;
                        pararg2[NbrSol] = 0.0;
                        pararg3[NbrSol] = 0.0;
                    }
                }
                WellDone = true;
            }
        }
    }
}
