// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad_5.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanRad
{
    //   Circle passing by two points of given radius.
    //   ==============================================
    //==============================================================================
    //   Solution of the equation of second degree showing that the center of the  +
    //   circle is equidistant from two points.                                    +
    //==============================================================================
    /// <summary>Circles of radius Radius passing through two points.</summary>
    /// <exception cref="Standard_NegativeValue">Radius is negative.</exception>
    public GccAna_Circ2d2TanRad(in gp_Pnt2d Point1, in gp_Pnt2d Point2, double Radius, double Tolerance)
    {
        qualifier1 = new(1, 2);
        qualifier2 = new(1, 2);
        TheSame1 = new(1, 2);
        TheSame2 = new(1, 2);
        cirsol = new(1, 2);
        pnttg1sol = new(1, 2);
        pnttg2sol = new(1, 2);
        par1sol = new(1, 2);
        par2sol = new(1, 2);
        pararg1 = new(1, 2);
        pararg2 = new(1, 2);


        gp_Dir2d dirx = new(gp_Dir2d.D.X);
        double   Tol = Math.Abs(Tolerance);
        NbrSol       = 0;
        WellDone     = false;
        if (Radius < 0.0)
        {
            throw new Standard_NegativeValue();
        }
        else
        {
            if (Point1.Distance(Point2) - Radius * 2.0 > Tol)
            {
                WellDone = true;
            }
            else
            {
                if (Point1.Distance(Point2) - Radius * 2.0 > 0.0)
                {
                    WellDone = true;
                    NbrSol   = 1;
                    gp_Ax2d axe = new(new gp_Pnt2d(0.5 * (Point1.XY() + Point2.XY())), dirx);
                    cirsol[1] = new gp_Circ2d(axe, Radius);
                    //      =================================
                    qualifier1[1] = GccEnt_noqualifier;
                    qualifier2[1] = GccEnt_noqualifier;
                    TheSame1[1]   = 0;
                    TheSame2[1]   = 0;
                    pnttg1sol[1]  = Point1;
                    pnttg2sol[1]  = Point2;
                }
                else
                {
                    gp_Circ2d C1 = new(new gp_Ax2d(Point1, dirx), Radius);
                    gp_Circ2d C2 = new(new gp_Ax2d(Point2, dirx), Radius);
                    IntAna2d_AnaIntersection Intp = new(C1, C2);
                    if (Intp.IsDone())
                    {
                        if (!Intp.IsEmpty())
                        {
                            for (int i = 1; i <= Intp.NbPoints(); i++)
                            {
                                NbrSol++;
                                gp_Pnt2d Center = Intp.Point(i).Value();
                                cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius);
                                //            =======================================================
                                qualifier1[NbrSol] = GccEnt_noqualifier;
                                qualifier2[NbrSol] = GccEnt_noqualifier;
                                TheSame1[NbrSol]   = 0;
                                TheSame2[NbrSol]   = 0;
                                pnttg1sol[NbrSol]  = Point1;
                                pnttg2sol[NbrSol]  = Point2;
                            }
                        }
                        WellDone = true;
                    }
                }
            }
        }
        for (int i = 1; i <= NbrSol; i++)
        {
            pararg1[i] = 0.0;
            pararg2[i] = 0.0;
            par1sol[i] = GccAna_ElCLib.Parameter(cirsol[i], pnttg1sol[i]);
            par2sol[i] = GccAna_ElCLib.Parameter(cirsol[i], pnttg2sol[i]);
        }
    }
}
