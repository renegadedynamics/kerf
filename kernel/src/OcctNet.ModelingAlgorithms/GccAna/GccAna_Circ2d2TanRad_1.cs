// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad_1.cxx
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
    // circular tangent to a circle, a line and a given radius
    //==============================================================
    //========================================================================
    // Initialize WellDone to false.                                         +
    // Return circle C1 and straight line L2.                                +
    // Leave with error if the construction is impossible.                   +
    // Create parallel to C1 in the proper direction.                        +
    // Create parallel to L2 in the proper direction.                        +
    // Intersect parallels ==> center point of the solution.                 +
    // Create the solution and add it to already found ones.                 +
    // Fill the fields.                                                      +
    //========================================================================
    /// <summary>Circles of radius Radius tangent to a qualified circle and a qualified line.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public GccAna_Circ2d2TanRad(in GccEnt_QualifiedCirc Qualified1, in GccEnt_QualifiedLin Qualified2, double Radius, double Tolerance)
    {
        qualifier1 = new(1, 8);
        qualifier2 = new(1, 8);
        TheSame1 = new(1, 8);
        TheSame2 = new(1, 8);
        cirsol = new(1, 8);
        pnttg1sol = new(1, 8);
        pnttg2sol = new(1, 8);
        par1sol = new(1, 8);
        par2sol = new(1, 8);
        pararg1 = new(1, 8);
        pararg2 = new(1, 8);


        double   Tol = Math.Abs(Tolerance);
        Direction2D dirx = new(Direction2D.D.X);
        NbrSol   = 0;
        WellDone = false;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        int i;
        for (i = 1; i <= 8; i++)
        {
            TheSame2[i] = 0;
        }
        int                        ncote1 = 0;
        int                        ncote2 = 0;
        int                        nbsol  = 0;
        double                     cote   = 0.0;
        double                     ccote  = 0.0;
        NCollection_Array1<double> cote1 = new(1, 3);
        NCollection_Array1<double> cote2 = new(1, 2);
        Circle2D                  C1   = Qualified1.Qualified();
        Line2D                   L2   = Qualified2.Qualified();
        double                     xdir = (L2.Direction()).X();
        double                     ydir = (L2.Direction()).Y();
        Direction2D normL2 = new(-ydir, xdir);
        double                     lxloc = (L2.Location()).X();
        double                     lyloc = (L2.Location()).Y();
        double                     cxloc = (C1.Location()).X();
        double                     cyloc = (C1.Location()).Y();
        double                     R1    = C1.Radius();
        Point2D center1 = new(cxloc, cyloc);
        Point2D origin2 = new(lxloc, lyloc);
        double                     dist     = Radius * 2.0 + R1;
        double                     distance = L2.Distance(center1);

        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (distance - dist > Tol)
            {
                WellDone = true;
            }
            else if (Qualified1.IsEnclosed())
            {
                //  =================================
                if (Qualified2.IsEnclosed())
                {
                    //    ============================
                    if (distance - R1 > Tol)
                    {
                        WellDone = true;
                    }
                    else if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                        && (Radius * 2.0 - R1 + distance > Tol))
                                      || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                              && (Radius * 2.0 - R1 - distance > Tol))
                                      || (Radius - R1 > Tol))
                    {
                        WellDone = true;
                    }
                    else
                    {
                        if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                  && (Radius * 2.0 > R1 - distance))
                                || (Math.Abs(distance - R1) < Tol)
                                || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                        && (Radius * 2.0 > (R1 + distance + Tol))))
                        {
                            cote  = 1.0;
                            nbsol = 3;
                            ccote = 1.0;
                        }
                        else
                        {
                            ncote1   = 1;
                            ncote2   = 1;
                            cote1[1] = R1 - Radius;
                            cote2[1] = 1.0;
                            nbsol    = 1;
                        }
                    }
                }
                else if (Qualified2.IsOutside())
                {
                    //    ================================
                    if (distance > R1 + Tol)
                    {
                        WellDone = true;
                    }
                    else if ((((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc)) > 0.0)
                                        && (Radius * 2.0 - R1 + distance > Tol))
                                      || (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc)) < 0.0)
                                              && (Radius * 2.0 - R1 - distance > Tol))
                                      || (Radius - R1 > Tol))
                    {
                        WellDone = true;
                    }
                    else
                    {
                        if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                  && (Radius * 2.0 > R1 - distance))
                                || (Math.Abs(R1 - Radius) < Tol)
                                || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                        && (Radius * 2.0 > (R1 + distance)))
                                || (Math.Abs(distance - R1) < Tol))
                        {
                            cote  = -1.0;
                            nbsol = 3;
                            ccote = 1.0;
                        }
                        else
                        {
                            ncote1   = 1;
                            ncote2   = 1;
                            cote1[1] = R1 - Radius;
                            cote2[1] = -1.0;
                            nbsol    = 1;
                        }
                    }
                }
                else if (Qualified2.IsUnqualified())
                {
                    //    ====================================
                    if ((distance - R1 > Tol) || (Radius * 2.0 - R1 - distance > Tol) || (Radius - R1 > Tol))
                    {
                        WellDone = true;
                    }
                    else if (distance > R1)
                    {
                        if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0))
                        {
                            cote = 1.0;
                        }
                        else
                        {
                            cote  = -1.0;
                            nbsol = 3;
                        }
                        nbsol = 3;
                        ccote = 1;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 2;
                        cote1[1] = R1 - Radius;
                        cote2[1] = 1.0;
                        cote2[2] = -1.0;
                        nbsol    = 1;
                    }
                }
                if (nbsol == 3)
                {
                }
            }
            else if (Qualified1.IsEnclosing())
            {
                //  ==================================
                if (Qualified2.IsEnclosed())
                {
                    //    =================================
                    if ((distance < R1 - Tol) || (Radius < R1 - Tol) || (Radius * 2 < distance + R1 - Tol)
                            || (Tol < R1 - Radius) || (-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0))
                    {
                        WellDone = true;
                    }
                    else if ((distance < R1) || (Radius < R1) || (Radius * 2.0 < distance + R1))
                    {
                        cote  = 1.0;
                        ccote = 1.0;
                        nbsol = 3;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 1;
                        cote1[1] = Radius - R1;
                        cote2[1] = 1.0;
                        nbsol    = 1;
                    }
                }
                else if (Qualified2.IsOutside())
                {
                    //    ================================
                    if ((Tol < R1 - distance) || (Tol < distance + R1 - Radius * 2.0) || (Tol < R1 - Radius)
                            || (-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0))
                    {
                        WellDone = true;
                    }
                    else if ((distance < R1) || (Radius * 2.0 < distance + R1) || (Radius < R1))
                    {
                        cote  = -1.0;
                        ccote = -1.0;
                        nbsol = 3;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 1;
                        cote1[1] = Radius - R1;
                        cote2[1] = -1.0;
                        nbsol    = 1;
                    }
                }
                else if (Qualified2.IsUnqualified())
                {
                    //    ====================================
                    if ((distance < R1 - Tol) || (Radius * 2.0 < distance + R1 - Tol) || (Radius < R1 - Tol))
                    {
                        WellDone = true;
                    }
                    else if ((distance < R1) || (Radius * 2.0 < distance + R1))
                    {
                        if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0))
                        {
                            cote  = 1.0;
                            ccote = 1.0;
                            nbsol = 3;
                        }
                        else
                        {
                            ccote = -1.0;
                            cote  = -1.0;
                            nbsol = 3;
                        }
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 2;
                        cote1[1] = Radius - R1;
                        cote2[1] = 1.0;
                        cote2[2] = -1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((Qualified1.IsOutside()) && (Qualified2.IsEnclosed()))
            {
                //  ===============================================================
                if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0) && (distance > R1 + Tol))
                        || (distance > R1 + Radius * 2.0 + Tol))
                {
                    WellDone = true;
                }
                else
                {
                    if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0) && (distance > R1))
                    {
                        cote  = 1.0;
                        nbsol = 2;
                    }
                    else if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                      && (distance > R1 + 2.0 * Radius))
                    {
                        cote  = -1.0;
                        nbsol = 2;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 1;
                        cote1[1] = Radius + R1;
                        cote2[1] = 1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((Qualified1.IsOutside()) && (Qualified2.IsOutside()))
            {
                //  ==============================================================
                if (((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0) && (distance > R1 + Tol))
                        || (distance > R1 + Radius * 2.0 + Tol))
                {
                    WellDone = true;
                }
                else
                {
                    if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0) && (distance > R1))
                    {
                        cote  = -1.0;
                        nbsol = 2;
                    }
                    else if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                      && (distance > R1 + 2.0 * Radius))
                    {
                        cote  = 1.0;
                        nbsol = 2;
                    }
                    else
                    {
                        ncote1   = 1;
                        ncote2   = 1;
                        cote1[1] = Radius + R1;
                        cote2[1] = -1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((Qualified1.IsOutside()) && (Qualified2.IsUnqualified()))
            {
                //  ==================================================================
                if (distance > Radius * 2.0 + R1 + Tol)
                {
                    WellDone = true;
                }
                else if (distance > Radius * 2.0 + R1)
                {
                    if (-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                    {
                        cote = 1.0;
                    }
                    else
                    {
                        cote = -1.0;
                    }
                    nbsol = 4;
                }
                else
                {
                    ncote1   = 1;
                    ncote2   = 2;
                    cote1[1] = Radius + R1;
                    cote2[1] = 1.0;
                    cote2[2] = -1.0;
                    nbsol    = 1;
                }
            }
            else if ((Qualified1.IsUnqualified()) && (Qualified2.IsEnclosed()))
            {
                //  ===================================================================
                if ((distance > R1 + Radius * 2.0 + Tol)
                        || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0) && (distance > R1 + Tol)))
                {
                    WellDone = true;
                }
                else
                {
                    if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0) && (distance > R1))
                    {
                        cote  = 1.0;
                        nbsol = 2;
                    }
                    else if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0)
                                      && (distance > R1 + 2.0 * Radius))
                    {
                        cote  = -1.0;
                        nbsol = 2;
                    }
                    else
                    {
                        ncote1   = 2;
                        ncote2   = 1;
                        cote1[1] = Math.Abs(Radius - R1);
                        cote1[2] = Radius + R1;
                        cote2[1] = 1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((Qualified1.IsUnqualified()) && (Qualified2.IsOutside()))
            {
                //  ==================================================================
                if ((distance > R1 + Radius * 2.0 + Tol)
                        || ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0) && (distance > R1 + Tol)))
                {
                    WellDone = true;
                }
                else
                {
                    if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) < 0.0) && (distance > R1))
                    {
                        cote  = -1.0;
                        nbsol = 2;
                    }
                    else if ((-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                                      && (distance > R1 + 2.0 * Radius))
                    {
                        cote  = 1.0;
                        nbsol = 2;
                    }
                    else
                    {
                        ncote1   = 2;
                        ncote2   = 1;
                        cote1[1] = Math.Abs(Radius - R1);
                        cote1[2] = Radius + R1;
                        cote2[1] = -1.0;
                        nbsol    = 1;
                    }
                }
            }
            else if ((Qualified1.IsUnqualified()) && (Qualified2.IsUnqualified()))
            {
                //  ======================================================================
                if (distance > R1 + Radius * 2.0 + Tol)
                {
                    WellDone = true;
                }
                else if (distance > R1 + Radius * 2.0)
                {
                    if (-ydir * (cxloc - lxloc) + xdir * (cyloc - lyloc) > 0.0)
                    {
                        cote = -1.0;
                    }
                    else
                    {
                        cote = 1.0;
                    }
                    nbsol = 4;
                }
                else
                {
                    ncote1   = 2;
                    ncote2   = 2;
                    cote1[1] = Math.Abs(Radius - R1);
                    cote1[2] = Radius + R1;
                    cote2[1] = 1.0;
                    cote2[2] = -1.0;
                    nbsol    = 1;
                }
            }
            if (nbsol == 1)
            {
                for (int jcote1 = 1; jcote1 <= ncote1; jcote1++)
                {
                    for (int jcote2 = 1; jcote2 <= ncote2; jcote2++)
                    {
                        if (cote1[jcote1] < Tol)
                        {
                            continue;
                        }
                        Circle2D cirint = new(new Axis2D(center1, dirx), cote1[jcote1]);
                        Line2D linint = new(
                            new Point2D(lxloc - cote2[jcote2] * ydir * Radius, lyloc + cote2[jcote2] * xdir * Radius),
                            L2.Direction());
                        AnalyticIntersection Intp = new(linint, cirint);
                        if (Intp.IsDone)
                        {
                            if (!Intp.IsEmpty)
                            {
                                for (i = 1; i <= Intp.PointCount; i++)
                                {
                                    NbrSol++;
                                    Point2D Center = Intp.Point(i).Value();
                                    Axis2D axe = new(Center, dirx);
                                    cirsol[NbrSol] = new Circle2D(axe, Radius);
    //              ======================================
                                    Direction2D dc2 = new(origin2.XY() - Center.XY());
                                    double   distcc1 = Center.Distance(center1);
                                    if (!Qualified1.IsUnqualified())
                                    {
                                        qualifier1[NbrSol] = Qualified1.Qualifier();
                                    }
                                    else if (Math.Abs(distcc1 + Radius - R1) < Tol)
                                    {
                                        qualifier1[NbrSol] = GccEnt_enclosed;
                                    }
                                    else if (Math.Abs(distcc1 - R1 - Radius) < Tol)
                                    {
                                        qualifier1[NbrSol] = GccEnt_outside;
                                    }
                                    else
                                    {
                                        qualifier1[NbrSol] = GccEnt_enclosing;
                                    }
                                    if (!Qualified2.IsUnqualified())
                                    {
                                        qualifier2[NbrSol] = Qualified2.Qualifier();
                                    }
                                    else if (dc2.Dot(normL2) > 0.0)
                                    {
                                        qualifier2[NbrSol] = GccEnt_outside;
                                    }
                                    else
                                    {
                                        qualifier2[NbrSol] = GccEnt_enclosed;
                                    }
                                    TheSame1[NbrSol] = 0;
                                    if ((Radius < R1) && Center.Distance(center1) <= R1)
                                    {
                                        pnttg1sol[NbrSol] =
                                            new Point2D(Center.XY() + Radius * (new Direction2D(Center.XY() - center1.XY()).XY()));
                                    }
                                    else
                                    {
                                        pnttg1sol[NbrSol] =
                                            new Point2D(Center.XY() - Radius * (new Direction2D(Center.XY() - center1.XY()).XY()));
                                    }
                                    pnttg2sol[NbrSol] =
                                        new Point2D(Center.XY() + cote2[jcote2] * Radius * new Xy(ydir, -xdir));
                                }
                            }
                            WellDone = true;
                        }
                    }
                }
            }
            else if (nbsol == 2)
            {
                Point2D Cen = new(center1.XY() + cote * (R1 + Radius) * new Xy(-ydir, xdir));
                Axis2D axe = new(Cen, dirx);
                cirsol[1] = new Circle2D(axe, Radius);
                //    =================================
                WellDone      = true;
                NbrSol        = 1;
                TheSame1[1]   = 0;
                qualifier1[1] = Qualified1.Qualifier();
                qualifier2[1] = Qualified2.Qualifier();
                pnttg1sol[1]  = new Point2D(Cen.XY() + cote * Radius * new Xy(ydir, -xdir));
                pnttg2sol[1]  = new Point2D(Cen.XY() + Radius * new Xy(ydir, -xdir));
            }
            else if (nbsol == 3)
            {
                WellDone = true;
                NbrSol   = 1;
                Point2D Cen = new(center1.XY() + cote * (R1 - Radius) * new Xy(ydir, -xdir));
                Axis2D axe = new(Cen, dirx);
                cirsol[1] = new Circle2D(axe, Radius);
                //    =================================
                qualifier1[1] = Qualified1.Qualifier();
                qualifier2[1] = Qualified2.Qualifier();
                pnttg2sol[1]  = new Point2D(Cen.XY() + cote * Radius * new Xy(ydir, -xdir));
                if (Math.Abs(R1 - Radius) > 0.0)
                {
                    pnttg1sol[1] = new Point2D(Cen.XY() + ccote * Radius * new Xy(ydir, -xdir));
                }
                else
                {
                    TheSame1[1] = 1;
                }
            }
            else if (nbsol == 4)
            {
                Point2D Cent = new(center1.XY() + cote * (R1 + Radius) * new Xy(-ydir, xdir));
                Axis2D axe = new(Cent, dirx);
                cirsol[1] = new Circle2D(axe, Radius);
                //    =================================
                qualifier1[1] = Qualified1.Qualifier();
                qualifier2[1] = Qualified2.Qualifier();
                WellDone      = true;
                NbrSol        = 1;
                TheSame1[1]   = 0;
                pnttg1sol[1]  = new Point2D(Cent.XY() + cote * Radius * new Xy(ydir, -xdir));
                pnttg2sol[1]  = new Point2D(Cent.XY() + cote * Radius * new Xy(-ydir, xdir));
            }
        }
        for (i = 1; i <= NbrSol; i++)
        {
            par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[i], pnttg1sol[i]);
            if (TheSame1[i] == 0)
            {
                pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[i]);
            }
            par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[i], pnttg2sol[i]);
            pararg2[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(L2, pnttg2sol[i]);
        }
    }
}
