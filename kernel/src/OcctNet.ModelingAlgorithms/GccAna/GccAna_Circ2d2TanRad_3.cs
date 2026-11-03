// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

public sealed partial class GccAna_Circ2d2TanRad
{
    // Modified:	Thu Jun 18 15:45:00 1998
    //		PRO10310 : cas ou le point est sur la droite


    // circular tangent to a line and a point and a given radius
    //=============================================================
    //========================================================================
    // Initialize WellDone to false.                                         +
    // Return line L1.                                                       +
    // Leave with error if the construction is impossible.                   +
    // Create parallel to L1 in the proper direction.                        +
    // Create the circle with center at Point1 of radius Radius.             +
    // Intersect the parallel and the circle.                                +
    //                              ==> The center point of the  solution.   +
    // Create the solution to be added to already found solutions.           +
    // Fill the fields.                                                      +
    //========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified line and passing through a point.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public GccAna_Circ2d2TanRad(in GccEnt_QualifiedLin Qualified1, in Point2D Point2, double Radius, double Tolerance)
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
        if (!(Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        int                        nbsol  = 0;
        int                        nbcote = 0;
        NCollection_Array1<double> cote = new(1, 2);
        Line2D                   L1     = Qualified1.Qualified();
        double                     displ1 = L1.Distance(Point2);
        double                     xdir   = (L1.Direction()).X();
        double                     ydir   = (L1.Direction()).Y();
        double                     lxloc  = (L1.Location()).X();
        double                     lyloc  = (L1.Location()).Y();
        Point2D origin1 = new(lxloc, lyloc);
        Direction2D normL1 = new(-ydir, xdir);
        double                     cxloc = Point2.X();
        double                     cyloc = Point2.Y();

        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }

        else
        {
            if (displ1 - Radius * 2.0 > Tol)
            {
                WellDone = true;
            }
            else if (Qualified1.IsEnclosed())
            {
                //  =================================
                if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0))
                {
                    WellDone = true;
                }
                else
                {
                    if (displ1 - Radius * 2.0 > 0.0)
                    {
                        nbsol   = 2;
                        NbrSol  = 1;
                        nbcote  = 1;
                        cote[1] = 1.0;
                    }
                    else
                    {
                        nbsol   = 1;
                        nbcote  = 1;
                        cote[1] = 1.0;
                    }
                }
            }
            else if (Qualified1.IsOutside())
            {
                //  ================================
                if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0))
                {
                    WellDone = true;
                }
                else
                {
                    if (displ1 - Radius * 2.0 > 0.0)
                    {
                        nbsol   = 2;
                        nbcote  = 1;
                        cote[1] = -1.0;
                    }
                    else
                    {
                        nbsol   = 1;
                        nbcote  = 1;
                        cote[1] = -1.0;
                    }
                }
            }
            else if (Qualified1.IsUnqualified())
            {
                //  ====================================
                if (displ1 - Radius * 2.0 > 0.0)
                {
                    if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0))
                    {
                        nbsol   = 2;
                        nbcote  = 1;
                        cote[1] = 1.0;
                    }
                    else if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0))
                    {
                        nbsol   = 2;
                        nbcote  = 1;
                        cote[1] = -1.0;
                    }
                }
                else
                {
                    nbsol   = 1;
                    nbcote  = 2;
                    cote[1] = 1.0;
                    cote[2] = -1.0;
                }
            }

            if (nbsol == 1)
            {
                if (displ1 < 1.0e-10)
                {
                    // particular case when Point2 is on the line
                    // construct two solutions directly
                    for (int jcote = 1; jcote <= nbcote; jcote++)
                    {
                        NbrSol++;
                        Point2D Center = new(cxloc - cote[jcote] * ydir * Radius, cyloc + cote[jcote] * xdir * Radius);
                        cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius);
                        //        =======================================================
                        qualifier2[NbrSol] = GccEnt_noqualifier;
                        if (!Qualified1.IsUnqualified())
                        {
                            qualifier1[NbrSol] = Qualified1.Qualifier();
                        }
                        else if (cote[jcote] > 0.0)
                        {
                            qualifier1[NbrSol] = GccEnt_outside;
                        }
                        else
                        {
                            qualifier1[NbrSol] = GccEnt_enclosed;
                        }
                        TheSame1[NbrSol]  = 0;
                        TheSame2[NbrSol]  = 0;
                        pnttg1sol[NbrSol] = Point2;
                        pnttg2sol[NbrSol] = Point2;
                    }
                    WellDone = true;
                }
                else
                {
                    Circle2D cirint = new(new Axis2D(Point2, dirx), Radius);
                    for (int jcote = 1; jcote <= nbcote; jcote++)
                    {
                        Line2D linint = new(
                            new Point2D(lxloc - cote[jcote] * ydir * Radius, lyloc + cote[jcote] * xdir * Radius),
                            L1.Direction());
                        AnalyticIntersection Intp = new(linint, cirint);
                        if (Intp.IsDone)
                        {
                            if (!Intp.IsEmpty)
                            {
                                for (int i = 1; i <= Intp.PointCount && NbrSol < 2; i++)
                                {
                                    NbrSol++;
                                    Point2D Center = Intp.Point(i).Value();
                                    cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius);
                                    //            =======================================================
                                    Direction2D dc1 = new(origin1.XY() - Center.XY());
                                    qualifier2[NbrSol] = GccEnt_noqualifier;
                                    if (!Qualified1.IsUnqualified())
                                    {
                                        qualifier1[NbrSol] = Qualified1.Qualifier();
                                    }
                                    else if (dc1.Dot(normL1) > 0.0)
                                    {
                                        qualifier1[NbrSol] = GccEnt_outside;
                                    }
                                    else
                                    {
                                        qualifier1[NbrSol] = GccEnt_enclosed;
                                    }
                                    TheSame1[NbrSol] = 0;
                                    TheSame2[NbrSol] = 0;
                                    pnttg1sol[NbrSol] =
                                        new Point2D(Center.XY() + cote[jcote] * Radius * new Xy(ydir, -xdir));
                                    pnttg2sol[NbrSol] = Point2;
                                }
                            }
                            WellDone = true;
                        }
                    }
                }
            }

            else if (nbsol == 2)
            {
                Point2D Center = new(Point2.XY() + cote[1] * Radius * new Xy(-ydir, xdir));
                WellDone  = true;
                NbrSol    = 1;
                cirsol[1] = new Circle2D(new Axis2D(Center, dirx), Radius);
                //    ==================================================
                qualifier2[1] = GccEnt_noqualifier;
                TheSame1[1]   = 0;
                TheSame2[1]   = 0;
                pnttg1sol[1]  = new Point2D(Center.XY() + cote[1] * Radius * new Xy(ydir, -xdir));
                pnttg2sol[1]  = Point2;
            }
        }

        for (int i = 1; i <= NbrSol; i++)
        {
            par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[i], pnttg1sol[i]);
            pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L1, pnttg1sol[i]);
            par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[i], pnttg2sol[i]);
            pararg2[i] = 0.0;
        }
    }
}
