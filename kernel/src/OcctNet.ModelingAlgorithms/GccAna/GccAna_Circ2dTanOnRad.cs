// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2dTanOnRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// 2D circles of a given radius tangent to a qualified circle or line, or passing through a point, with their centre
/// on a line or a circle.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). Up to 4 solutions.</remarks>
public sealed partial class GccAna_Circ2dTanOnRad
{
    private bool WellDone;
    private int NbrSol;
    private readonly NCollection_Array1<Circle2D> cirsol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<int> TheSame1;
    private readonly NCollection_Array1<Point2D> pnttg1sol;
    private readonly NCollection_Array1<Point2D> pntcen3;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> pararg1;
    private readonly NCollection_Array1<double> parcen3;

    // PRO12736 : bug quand OnLine // Ox, JCT 20/03/98
    //========================================================================
    //       circular tangent to element of type :      - Circle.            +
    //                                                  - Line.             +
    //                                                  - Point.             +
    //              center on second element of type :  - Circle.  +
    //                                                  - Line.   +
    //              of given radius : Radius.                             +
    //========================================================================



    //=========================================================================
    //  Circle tangent  :  to circle Qualified1 (C1).                         +
    //         center   :  on straight line OnLine.                           +
    //         of radius :  Radius.                                           +
    //                                                                        +
    //  Initialise the table of solutions cirsol and all fields.              +
    //  Eliminate depending on the qualifier the cases not being solutions.   +
    //  Solve the equation of the second degree indicating that the found center +
    //  point (xc,yc) is at a distance Radius from circle C1 and              +
    //                     on straight line OnLine.                           +
    //  The solutions aret represented by circles :                           +
    //                   - with center Pntcen(xc,yc)                          +
    //                   - with radius Radius.                                +
    //=========================================================================

    /// <summary>Circles of radius Radius tangent to a qualified circle, centred on a line.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public GccAna_Circ2dTanOnRad(in GccEnt_QualifiedCirc Qualified1, in Line2D OnLine, double Radius, double Tolerance)
    {
        cirsol = new(1, 4);
        qualifier1 = new(1, 4);
        TheSame1 = new(1, 4);
        pnttg1sol = new(1, 4);
        pntcen3 = new(1, 4);
        par1sol = new(1, 4);
        pararg1 = new(1, 4);
        parcen3 = new(1, 4);


        TheSame1.Init(0);
        Direction2D dirx = new(Direction2D.D.X);
        double   Tol = Math.Abs(Tolerance);
        WellDone     = false;
        NbrSol       = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        NCollection_Array1<double> Coef = new(1, 2);
        Circle2D                  C1 = Qualified1.Qualified();

        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            int      nbsol = 0;
            int      signe = 0;
            Point2D Center = new();
            double   xc;
            double   yc;
            double   R1    = C1.Radius();
            double   dist  = OnLine.Distance(C1.Location());
            double   xdir  = (OnLine.Direction()).X();
            double   ydir  = (OnLine.Direction()).Y();
            double   lxloc = (OnLine.Location()).X();
            double   lyloc = (OnLine.Location()).Y();
            Point2D center1 = C1.Location();
            double   x1 = center1.X();
            double   y1 = center1.Y();
            if (Qualified1.IsEnclosed())
            {
                //   ============================
                if (Tol < Radius - R1 + dist)
                {
                    WellDone = true;
                }
                else
                {
                    if (Math.Abs(Radius - R1 + dist) < Tol)
                    {
                        WellDone = true;
                        NbrSol   = 1;
                        if (-ydir * (x1 - lxloc) + xdir * (y1 - lyloc) < 0.0)
                        {
                            Center = new Point2D(x1 - ydir * dist, y1 + xdir * dist);
                        }
                        else
                        {
                            Center = new Point2D(x1 + ydir * dist, y1 - xdir * dist);
                        }
                        signe = 1;
                    }
                    else
                    {
                        Coef[1] = (R1 - Radius) * (R1 - Radius);
                        nbsol   = 1;
                    }
                }
            }
            else if (Qualified1.IsEnclosing())
            {
                //   ==================================
                if (R1 + dist - Radius > Tol)
                {
                    WellDone = true;
                }
                else
                {
                    if (R1 + dist - Radius > 0.0)
                    {
                        WellDone = true;
                        NbrSol   = 1;
                        if (-ydir * (x1 - lxloc) + xdir * (y1 - lyloc) < 0.0)
                        {
                            Center = new Point2D(x1 - ydir * dist, y1 + xdir * dist);
                        }
                        else
                        {
                            Center = new Point2D(x1 + ydir * dist, y1 - xdir * dist);
                        }
                        signe = -1;
                    }
                    else
                    {
                        Coef[1] = (Radius - R1) * (Radius - R1);
                        nbsol   = 1;
                    }
                }
            }
            else
            {
                //   ====
                if (dist - R1 - Radius > Tol)
                {
                    WellDone = false;
                }
                else
                {
                    if (Math.Abs(dist - R1 - Radius) < Tol)
                    {
                        WellDone = true;
                        NbrSol   = 1;
                        if (-ydir * (x1 - lxloc) + xdir * (y1 - lyloc) < 0.0)
                        {
                            Center = new Point2D(x1 - ydir * dist, y1 + xdir * dist);
                        }
                        else
                        {
                            Center = new Point2D(x1 + ydir * dist, y1 - xdir * dist);
                        }
                        signe = -1;
                    }
                    else
                    {
                        if (Qualified1.IsOutside())
                        {
                            //         ===========================
                            Coef[1] = (Radius + R1) * (Radius + R1);
                            nbsol   = 1;
                        }
                        else
                        {
                            //         ====
                            Coef[1] = (Radius - R1) * (Radius - R1);
                            Coef[2] = (Radius + R1) * (Radius + R1);
                            nbsol   = 2;
                        }
                    }
                }
            }
            if (signe != 0)
            {
                cirsol[1] = new Circle2D(new Axis2D(Center, dirx), Radius);
                //     ==================================================
                double distcc1 = Center.Distance(center1);
                if (!Qualified1.IsUnqualified())
                {
                    qualifier1[1] = Qualified1.Qualifier();
                }
                else if (Math.Abs(distcc1 + Radius - R1) < Tol)
                {
                    qualifier1[1] = GccEnt_enclosed;
                }
                else if (Math.Abs(distcc1 - R1 - Radius) < Tol)
                {
                    qualifier1[1] = GccEnt_outside;
                }
                else
                {
                    qualifier1[1] = GccEnt_enclosing;
                }
                if (Math.Abs(Radius - R1) <= Tol)
                {
                    TheSame1[1] = 1;
                }
                else
                {
                    Direction2D dir1cen = new(Center.X() - x1, Center.Y() - y1);
                    pnttg1sol[1] = new Point2D(Center.XY() + signe * Radius * dir1cen.XY());
                    par1sol[1]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[1], pnttg1sol[1]);
                    pararg1[1]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[1]);
                }
                pntcen3[1] = cirsol[NbrSol].Location();
                parcen3[1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnLine, pntcen3[1]);
            }
            else if (nbsol > 0)
            {
                for (int j = 1; j <= nbsol; j++)
                {
                    double A, B, C;
                    OnLine.Coefficients(out A, out B, out C);
                    double D = A;
                    double x0, y0;
                    if (Math.Abs(D) <= Tol)
                    {
                        A  = B;
                        B  = D;
                        x0 = y1;
                        y0 = x1;
                    }
                    else
                    {
                        x0 = x1;
                        y0 = y1;
                    }
                    DirectPolynomialRoots Sol = new((B * B + A * A) / (A * A),
                                        2.0 * (B * C / (A * A) + (B / A) * x0 - y0),
                                        x0 * x0 + y0 * y0 + C * C / (A * A) - Coef[j] + 2.0 * C * x0 / A);
                    if (Sol.IsDone)
                    {
                        for (int i = 1; i <= Sol.SolutionCount; i++)
                        {

                            if (Math.Abs(D) > Tol)
                            {
                                yc = Sol.Value(i);
                                xc = -(B / A) * yc - C / A;
                            }
                            else
                            {
                                xc = Sol.Value(i);
                                yc = -(B / A) * xc - C / A;
                            }
                            Center = new Point2D(xc, yc);
                            if (OnLine.Distance(Center) > Tol)
                            {
                                continue;
                            }
                            NbrSol++;
                            cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius);
                            //           =======================================================
                            double distcc1 = Center.Distance(center1);
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
                            Direction2D dir1cen = new(Center.X() - x1, Center.Y() - y1);
                            // OCCT: the signs are swapped: dir1cen points from C1 to the centre, so these are the points
                            // of the solution opposite its tangency point
                            if ((Radius > R1) || (Center.Distance(center1) > R1))
                            {
                                pnttg1sol[NbrSol] = new Point2D(Center.XY() + Radius * dir1cen.XY());
                            }
                            else
                            {
                                pnttg1sol[NbrSol] = new Point2D(Center.XY() - Radius * dir1cen.XY());
                            }
                            pntcen3[NbrSol] = cirsol[NbrSol].Location();
                            par1sol[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                            pararg1[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                            parcen3[NbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(OnLine, pntcen3[NbrSol]);
                        }
                        WellDone = true;
                    }
                }
            }
        }
    }

    /// <summary>True if the construction succeeded (even with no solution).</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The number of solutions.</summary>
    public int NbSolutions()
    {
        return NbrSol;
    }

    /// <summary>The solution number Index.</summary>
    public Circle2D ThisSolution(int Index)
    {
        if (Index > NbrSol || Index <= 0)
        {
            throw new OutOfRangeException();
        }
        return cirsol[Index];
    }

    /// <summary>The qualifiers of the arguments for the solution number Index: those given, or those computed for unqualified arguments (GccEnt_noqualifier for a point).</summary>
    public void WhichQualifier(int Index, out GccEnt_Position Qualif1)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            Qualif1 = qualifier1[Index];
        }
    }

    /// <summary>The tangency point of the solution number Index with the first argument: its parameter on the solution, its parameter on the argument, and the point.</summary>
    public void Tangency1(int Index, out double ParSol, out double ParArg, out Point2D PntSol)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            ParSol = par1sol[Index];
            ParArg = pararg1[Index];
            PntSol = pnttg1sol[Index];
        }
    }

    /// <summary>The centre of the solution number Index on the On-curve: its parameter there, and the point.</summary>
    public void CenterOn3(int Index, out double ParArg, out Point2D PntSol)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            ParArg = parcen3[Index];
            PntSol = pnttg1sol[Index]; // OCCT: the tangency point, not the centre pntcen3
        }
    }

    /// <summary>True if the solution number Index is the first argument.</summary>
    public bool IsTheSame1(int Index)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }

        if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }

        if (TheSame1[Index] == 0)
        {
            return false;
        }

        return true;
    }
}
