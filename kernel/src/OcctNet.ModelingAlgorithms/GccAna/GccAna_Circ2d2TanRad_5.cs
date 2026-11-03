// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad_5.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
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
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public GccAna_Circ2d2TanRad(in Point2D Point1, in Point2D Point2, double Radius, double Tolerance)
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


        Direction2D dirx = new(Direction2D.D.X);
        double   Tol = Math.Abs(Tolerance);
        NbrSol       = 0;
        WellDone     = false;
        if (Radius < 0.0)
        {
            throw new NegativeValueException();
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
                    Axis2D axe = new(new Point2D(0.5 * (Point1.XY() + Point2.XY())), dirx);
                    cirsol[1] = new Circle2D(axe, Radius);
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
                    Circle2D C1 = new(new Axis2D(Point1, dirx), Radius);
                    Circle2D C2 = new(new Axis2D(Point2, dirx), Radius);
                    AnalyticIntersection Intp = new(C1, C2);
                    if (Intp.IsDone)
                    {
                        if (!Intp.IsEmpty)
                        {
                            for (int i = 1; i <= Intp.PointCount; i++)
                            {
                                NbrSol++;
                                Point2D Center = Intp.Point(i).Value();
                                cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius);
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
            par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[i], pnttg1sol[i]);
            par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[i], pnttg2sol[i]);
        }
    }
}
