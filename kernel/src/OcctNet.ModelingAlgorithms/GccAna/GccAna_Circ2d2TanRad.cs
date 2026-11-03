// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// 2D circles of a given radius tangent to two arguments, each a qualified circle, a qualified line or a point
/// (passed through). Each solution comes with the qualifiers of its arguments (computed for unqualified ones) and its
/// tangency points.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). Up to 8 solutions (two circles), 4 (a circle and a
/// line, two lines) or 2 (with a point).
/// </remarks>
public sealed partial class GccAna_Circ2d2TanRad
{
    private bool WellDone;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<GccEnt_Position> qualifier2;
    private readonly NCollection_Array1<int> TheSame1;
    private readonly NCollection_Array1<int> TheSame2;
    private int NbrSol;
    private readonly NCollection_Array1<Circle2D> cirsol;
    private readonly NCollection_Array1<Point2D> pnttg1sol;
    private readonly NCollection_Array1<Point2D> pnttg2sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> par2sol;
    private readonly NCollection_Array1<double> pararg1;
    private readonly NCollection_Array1<double> pararg2;

    // circular tangent to two cercles and given radius
    //====================================================
    //==================================================================
    // Initialize WellDone to false.                                   +
    // Return circle C1 and circle C2.                                 +
    // Leave with error if the construction is impossible.             +
    // Distinguish boundary cases to process them separately.          +
    // Create parallel to C1 in the proper direction.                  +
    // Create parallel to C2 in the proper direction.                  +
    // Intersect parallels ==> center point of the solution.           +
    // Create the solution to be added to already found solutions.     +
    // Fill the fields.                                                +
    //==================================================================
    /// <summary>Circles of radius Radius tangent to two qualified circles.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public GccAna_Circ2d2TanRad(in GccEnt_QualifiedCirc Qualified1, in GccEnt_QualifiedCirc Qualified2, double Radius, double Tolerance)
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
        WellDone = false;
        NbrSol   = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
                    || Qualified1.IsUnqualified())
                || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                          || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        bool                          invers = false;
        Circle2D                     C1     = Qualified1.Qualified();
        Circle2D                     C2     = Qualified2.Qualified();
        double                        R1     = C1.Radius();
        double                        R2     = C2.Radius();
        double                        rbid   = 0.0;
        int                           signe  = 0;
        double                        R3;
        double                        dist;
        NCollection_Array1<Circle2D> C = new(1, 8);
        C[1]           = C1;
        C[2]           = C2;
        int      nbsol = 0;
        Point2D center1 = C1.Location();
        Point2D center2 = C2.Location();
        Point2D center3 = new();
        if (Radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else if (C[2].Location().IsEqual(C[1].Location(), global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            WellDone = true;
        }
        else
        {
            Direction2D dir1 = new(C[2].Location().XY() - C[1].Location().XY());
            dist = (center2).Distance(center1);
            if ((Qualified1.IsEnclosed()) && Qualified2.IsEnclosed())
            {
                //   =========================================================
                if (Radius * 2.0 - Math.Abs(R1 + R2 - dist) > Tol)
                {
                    WellDone = true;
                }
                else
                {
                    if ((dist - R1 - R2 > Tol) || (Tol < Max(R1, R2) - dist - Min(R1, R2)))
                    {
                        WellDone = true;
                    }
                    else if (Math.Abs(dist - R1 - R2) <= Tol
                                      || Math.Abs(Max(R1, R2) - dist - Min(R1, R2)) <= Tol)
                    {
                        if (Math.Abs(dist - R1 - R2) <= Tol)
                        {
                            rbid  = R2 + (dist - R1 - R2) / 2.0;
                            signe = -1;
                        }
                        else if (Math.Abs(Max(R1, R2) - dist - Min(R1, R2)) <= Tol)
                        {
                            if (R1 > R2)
                            {
                                R3 = R2;
                            }
                            else
                            {
                                C2 = C1; // OCCT: overwrites C2, so pararg2 below is a parameter on C1
                                R3 = R1;
                            }
                            rbid  = -R3 + (Min(R1, R2) + dist - Max(R1, R2)) / 2.0;
                            signe = 1;
                        }
                        Axis2D axe = new(new Point2D(center2.XY() - rbid * dir1.XY()), dirx);
                        cirsol[1] = new Circle2D(axe, Radius);
                        //         =================================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified2.Qualifier();
                        pnttg1sol[1]  = new Point2D(center2.XY() + signe * R2 * dir1.XY());
                        pnttg2sol[1]  = new Point2D(center1.XY() + R1 * dir1.XY());
                        TheSame1[1]   = 0;
                        TheSame2[1]   = 0;
                        WellDone      = true;
                        NbrSol        = 1;
                    }
                    else
                    {
                        C[1]  = new Circle2D(new Axis2D(C[1].Location(), dirx), Math.Abs(Radius - R1));
                        C[2]  = new Circle2D(new Axis2D(C[2].Location(), dirx), Math.Abs(Radius - R2));
                        nbsol = 1;
                    }
                }
            }
            else if (((Qualified1.IsEnclosed()) && Qualified2.IsEnclosing()) ||
                              //   ===================================================================
                              ((Qualified1.IsEnclosing()) && Qualified2.IsEnclosed()))
            {
                //            ========================================================
                if (Qualified1.IsEnclosing())
                {
                    R3      = R1;
                    C[1]    = C2;
                    R1      = R2;
                    C[2]    = C1;
                    R2      = R3;
                    center3 = center1;
                    center1 = center2;
                    center2 = center3;
                    dir1.Reverse();
                    // it is necessary to swap the resulting tangency points
                    invers = true;
                }
                if ((R2 - Radius > Tol) || (Tol < Radius - R1) || (Tol > R1 - dist - R2)
                        || ((Tol < Radius * 2 - R1 - dist - R2) || (Tol < R1 + R2 - dist - Radius * 2.0)))
                {
                    WellDone = true;
                }
                else if ((Math.Abs(R2 - Radius) <= Tol) || (Math.Abs(R1 - Radius) <= Tol))
                {
                    if (Math.Abs(R2 - Radius) <= Tol)
                    {
                        C[2]         = C2;
                        R3           = R2;
                        TheSame2[1]  = 1;
                        TheSame1[1]  = 0;
                        pnttg1sol[1] = new Point2D(C[2].Location().XY() + R3 * dir1.XY());
                    }
                    else if (Math.Abs(Radius - R1) <= Tol)
                    {
                        C[2]         = C1;
                        R3           = R1;
                        TheSame1[1]  = 1;
                        TheSame2[1]  = 0;
                        pnttg2sol[1] = new Point2D(C[2].Location().XY() + R3 * dir1.XY());
                    }
                    WellDone = true;
                    NbrSol   = 1;
                    Axis2D axe = new(C[2].Location(), dirx);
                    cirsol[1] = new Circle2D(axe, Radius);
                    //       =================================
                    qualifier1[1] = Qualified1.Qualifier();
                    qualifier2[1] = Qualified2.Qualifier();
                }
                else
                {
                    if ((Math.Abs(R2 + dist - R1) <= Tol) || (Math.Abs(Radius * 2.0 - R1 - dist - R2) < Tol))
                    {
                        if (Math.Abs(R2 + dist - R1) <= Tol)
                        {
                            signe = 1;
                        }
                        else if (Math.Abs(Radius * 2.0 - R1 - dist - R2) < Tol)
                        {
                            signe = -1;
                        }
                        WellDone = true;
                        NbrSol   = 1;
                        Axis2D axe = new(new Point2D(center1.XY() + signe * (R1 - Radius) * dir1.XY()), dirx);
                        cirsol[1] = new Circle2D(axe, Radius);
                        //         =================================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified2.Qualifier();
                        pnttg1sol[1]  = new Point2D(center1.XY() + signe * R1 * dir1.XY());
                        pnttg2sol[1]  = new Point2D(center2.XY() + R2 * dir1.XY());
                        TheSame1[1]   = 0;
                        TheSame2[1]   = 0;
                    }
                    else
                    {
                        C[1]  = new Circle2D(new Axis2D(C[1].Location(), dirx), Math.Abs(Radius - R1));
                        C[2]  = new Circle2D(new Axis2D(C[2].Location(), dirx), Math.Abs(Radius - R2));
                        nbsol = 1;
                    }
                }
            }
            else if (((Qualified1.IsEnclosed()) && (Qualified2.IsOutside())) ||
                              //   ===================================================================
                              ((Qualified1.IsOutside()) && (Qualified2.IsEnclosed())))
            {
                //            ========================================================
                if (Qualified1.IsOutside())
                {
                    C[2]    = C1;
                    C[1]    = C2;
                    R3      = R1;
                    R1      = R2;
                    R2      = R3;
                    center3 = center1;
                    center1 = center2;
                    center2 = center3;
                    dir1.Reverse();
                    // it is necessary to swap the resulting tangency points
                    invers = true;
                }
                if ((Radius - R1 > Tol) || (dist - R2 - R1 > Tol)
                        || ((R2 - dist - R1 + Radius * 2 > Tol) || (R1 - R2 - dist - Radius * 2 > Tol)))
                {
                    WellDone = true;
                }
                else
                {
                    if (((Radius - R1 > 0.0) && (Math.Abs(dist - R1 - R2) <= Tol))
                            || (Math.Abs(R1 - R2 + dist - Radius * 2.0) <= Tol)
                            || (Math.Abs(R1 - R2 - dist - Radius * 2.0) <= Tol)
                            || (Math.Abs(dist - R1 - R2) <= Tol))
                    {
                        if (Math.Abs(R1 - R2 + dist - Radius * 2.0) <= Tol)
                        {
                            signe = -1;
                        }
                        else
                        {
                            signe = 1;
                            if ((Radius - R1 > 0.0) && (Math.Abs(dist - R1 - R2) <= Tol))
                            {
                                R2 = R1;
                            }
                            else if (Math.Abs(dist - R1 - R2) <= Tol)
                            {
                                R2 = R1;
                                if (Math.Abs(R1 - Radius) <= Tol)
                                {
                                    TheSame1[1] = 1;
                                }
                            }
                        }
                        WellDone = true;
                        NbrSol   = 1;
                        Axis2D axe = new(new Point2D(center1.XY() + signe * (R1 - Radius) * dir1.XY()), dirx);
                        cirsol[1] = new Circle2D(axe, Radius);
                        //         =================================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified2.Qualifier();
                        pnttg1sol[1]  = new Point2D(center1.XY() + signe * R1 * dir1.XY());
                        pnttg2sol[1]  = new Point2D(center2.XY() + signe * R2 * dir1.XY());
                        TheSame1[1]   = 0;
                        TheSame2[1]   = 0;
                    }
                    else
                    {
                        C[1]  = new Circle2D(new Axis2D(C[1].Location(), dirx), Math.Abs(Radius - R1));
                        C[2]  = new Circle2D(new Axis2D(C[2].Location(), dirx), Radius + R2);
                        nbsol = 1;
                    }
                }
            }
            else if (((Qualified1.IsEnclosed()) && (Qualified2.IsUnqualified())) ||
                              //   =======================================================================
                              ((Qualified1.IsUnqualified()) && (Qualified2.IsEnclosed())))
            {
                //            ============================================================
                if (Qualified1.IsUnqualified())
                {
                    C[1]    = C2;
                    C[2]    = C1;
                    R3      = R1;
                    R1      = R2;
                    R2      = R3;
                    center3 = center1;
                    center1 = center2;
                    center2 = center3;
                    dir1.Reverse();
                    // it is necessary to swap the resulting tangency points
                    invers = true;
                }
                if ((Radius - R1 > Tol) || (dist - R2 - R1 > Tol))
                {
                    WellDone = true;
                }
                else
                {
                    if ((Math.Abs(dist - R2 - R1) <= Tol) || (Math.Abs(Radius - R1) <= Tol))
                    {
                        WellDone = true;
                        NbrSol   = 1;
                        Axis2D ax = new(new Point2D(center1.XY() + (R1 - Radius) * dir1.XY()), dirx);
                        cirsol[1] = new Circle2D(ax, Radius);
                        //         ================================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified2.Qualifier();
                        pnttg2sol[1]  = new Point2D(center1.XY() + (dist - R2) * dir1.XY());
                        TheSame2[1]   = 0;
                        if (Math.Abs(Radius - R1) > 0.0)
                        {
                            TheSame1[1] = 1;
                        }
                        else
                        {
                            TheSame1[1]  = 0;
                            pnttg1sol[1] = new Point2D(center1.XY() + R1 * dir1.XY());
                        }
                    }
                    else
                    {
                        C[3]  = new Circle2D(new Axis2D(C[1].Location(), dirx), Math.Abs(Radius - R1));
                        C[4]  = new Circle2D(new Axis2D(C[2].Location(), dirx), Radius + R2);
                        C[1]  = new Circle2D(new Axis2D(C[1].Location(), dirx), Math.Abs(Radius - R1));
                        C[2]  = new Circle2D(new Axis2D(C[2].Location(), dirx), Math.Abs(Radius - R2));
                        nbsol = 2;
                    }
                }
            }
            else if ((Qualified1.IsEnclosing()) && (Qualified2.IsEnclosing()))
            {
                //   ==================================================================
                if ((Tol < Max(R1, R2) - Radius) || (Tol < Max(R1, R2) - dist - Min(R1, R2))
                        || (dist + R1 + R2 - Radius * 2.0 > Tol))
                {
                    WellDone = true;
                }
                else
                {
                    if ((Math.Abs(dist + Min(R1, R2) - Max(R1, R2)) <= Tol)
                            || (Math.Abs(R1 + R2 + dist - 2.0 * Radius) <= Tol))
                    {
                        if (Math.Abs(dist + Min(R1, R2) - Max(R1, R2)) <= Tol)
                        {
                            signe = 1;
                        }
                        else
                        {
                            signe = -1;
                        }
                        WellDone = true;
                        NbrSol   = 1;
                        Axis2D axe = new(new Point2D(center1.XY() + signe * (R1 - Radius) * dir1.XY()), dirx);
                        cirsol[1] = new Circle2D(axe, Radius);
                        //         =================================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified2.Qualifier();
                        pnttg1sol[1]  = new Point2D(center1.XY() + R1 * dir1.XY());
                        pnttg2sol[1]  = new Point2D(center2.XY() + signe * R2 * dir1.XY());
                        TheSame1[1]   = 0;
                        TheSame2[1]   = 0;
                    }
                    else if (Math.Abs(Radius - Max(R1, R2)) <= Tol)
                    {
                        WellDone = true;
                        NbrSol   = 1;
                        if (R1 > R2)
                        {
                            C[1]         = C1;
                            C[2]         = C2;
                            TheSame1[1]  = 1;
                            TheSame2[1]  = 0;
                            pnttg2sol[1] = new Point2D(center1.XY() + (dist + R2) * dir1.XY());
                        }
                        else
                        {
                            C[1]         = C2;
                            C[2]         = C1;
                            TheSame1[1]  = 0;
                            TheSame2[1]  = 1;
                            pnttg1sol[1] = new Point2D(center1.XY() - R1 * dir1.XY());
                        }
                        Axis2D axe = new(C[1].Location(), dirx);
                        cirsol[1] = new Circle2D(axe, Radius);
                        //         =================================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified2.Qualifier();
                    }
                    else
                    {
                        C[1]  = new Circle2D(new Axis2D(C[1].Location(), dirx), Math.Abs(Radius - R1));
                        C[2]  = new Circle2D(new Axis2D(C[2].Location(), dirx), Math.Abs(Radius - R2));
                        nbsol = 1;
                    }
                }
            }
            else if (((Qualified1.IsEnclosing()) && (Qualified2.IsOutside())) ||
                              //   ====================================================================
                              ((Qualified1.IsOutside()) && (Qualified2.IsEnclosing())))
            {
                //            =========================================================
                if (Qualified1.IsOutside())
                {
                    C[1]    = C2;
                    C[2]    = C1;
                    R3      = R1;
                    R1      = R2;
                    R2      = R3;
                    center3 = center1;
                    center1 = center2;
                    center2 = center3;
                    dir1.Reverse();
                    // it is necessary to swap the resulting tangency points
                    invers = true;
                }
                if ((R1 - Radius > Tol) || (Tol < R1 + R2 - dist) || (dist - R2 + R1 - Radius * 2.0 > Tol))
                {
                    WellDone = true;
                }
                else if (((Math.Abs(R1 - Radius) <= Tol) || (Math.Abs(R1 + R2 - dist) <= Tol))
                                  || (Math.Abs(dist - R2 + R1 - Radius * 2.0) <= Tol))
                {
                    WellDone = true;
                    NbrSol   = 1;
                    if ((Math.Abs(R1 - Radius) <= Tol) || (Math.Abs(R1 + R2 - dist) <= Tol))
                    {
                        TheSame1[1] = 1;
                    }
                    else
                    {
                        TheSame1[1] = 0;
                    }
                    TheSame2[1] = 0;
                    Axis2D axe = new(new Point2D(center1.XY() + (R1 - Radius) * dir1.XY()), dirx);
                    cirsol[1] = new Circle2D(axe, Radius);
                    //       =================================
                    qualifier1[1] = Qualified1.Qualifier();
                    qualifier2[1] = Qualified2.Qualifier();
                    pnttg1sol[1]  = new Point2D(center1.XY() - R1 * dir1.XY());
                    pnttg2sol[1]  = new Point2D(center1.XY() + (dist - R2) * dir1.XY());
                }
                else
                {
                    C[1]  = new Circle2D(new Axis2D(C[1].Location(), dirx), Math.Abs(Radius - R1));
                    C[2]  = new Circle2D(new Axis2D(C[2].Location(), dirx), Radius + R2);
                    nbsol = 1;
                }
            }
            else if (((Qualified1.IsEnclosing()) && (Qualified2.IsUnqualified())) ||
                              //   ========================================================================
                              ((Qualified1.IsUnqualified()) && (Qualified2.IsEnclosing())))
            {
                //            =============================================================
                if (Qualified1.IsUnqualified())
                {
                    C[1]    = C2;
                    C[2]    = C1;
                    R3      = R1;
                    R1      = R2;
                    R2      = R3;
                    center3 = center1;
                    center1 = center2;
                    center2 = center3;
                    invers  = true;
                    dir1.Reverse();
                }
                if ((Tol < R1 - dist - R2) || (R1 - Radius > Tol))
                {
                    WellDone = true;
                }
                // OCCT: "> 0.0" holds for every configuration not already rejected, so this branch always returns one
                // circle, centred at center1 + (Radius - R1) dir1, which is not tangent to C2 in general
                else if ((Math.Abs(R1 - Radius) <= Tol) || (Math.Abs(R1 - dist - R2) > 0.0))
                {
                    if (Math.Abs(R1 - Radius) <= Tol)
                    {
                        TheSame1[1] = 1;
                        if ((Math.Abs(Radius - R2) <= Tol) && (center1.Distance(center2) <= Tol))
                        {
                            TheSame2[1] = 1;
                        }
                    }
                    else if (Math.Abs(R1 - dist - R2) > 0.0)
                    {
                        TheSame1[1] = 0;
                        TheSame2[1] = 0;
                    }
                    WellDone = true;
                    NbrSol   = 1;
                    Axis2D axe = new(new Point2D(center1.XY() + (Radius - R1) * dir1.XY()), dirx);
                    cirsol[1] = new Circle2D(axe, Radius);
                    //       =================================
                    qualifier1[1] = Qualified1.Qualifier();
                    qualifier2[1] = Qualified2.Qualifier();
                    pnttg1sol[1]  = new Point2D(center1.XY() + R1 * dir1.XY());
                    pnttg2sol[1]  = new Point2D(center1.XY() + (dist + R2) * dir1.XY());
                }
                else
                {
                    C[3]  = new Circle2D(new Axis2D(C[1].Location(), dirx), Math.Abs(Radius - R1));
                    C[4]  = new Circle2D(new Axis2D(C[2].Location(), dirx), Radius + R2);
                    C[1]  = new Circle2D(new Axis2D(C[1].Location(), dirx), Math.Abs(Radius - R1));
                    C[2]  = new Circle2D(new Axis2D(C[2].Location(), dirx), Math.Abs(Radius - R2));
                    nbsol = 2;
                }
            }
            else if ((Qualified1.IsOutside()) && (Qualified2.IsOutside()))
            {
                //   ==============================================================
                if (Tol < Max(R1, R2) - dist - Min(R1, R2))
                {
                    WellDone = true;
                }
                else if (dist - R1 - R2 - Radius * 2.0 > Tol)
                {
                    WellDone = true;
                }
                else
                {
                    if (Math.Abs(dist + Min(R1, R2) - Max(R1, R2)) <= Tol)
                    {
                        WellDone = true;
                        NbrSol   = 1;
                        if (R1 < R2)
                        {
                            signe = -1;
                        }
                        else
                        {
                            signe = 1;
                        }
                        Axis2D axe = new(new Point2D(center1.XY() + signe * (Radius + R1) * dir1.XY()), dirx);
                        cirsol[1] = new Circle2D(axe, Radius);
                        //         =================================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified2.Qualifier();
                        pnttg1sol[1]  = new Point2D(center1.XY() + signe * R1 * dir1.XY());
                        pnttg2sol[1]  = pnttg1sol[1];
                        TheSame1[1]   = 0;
                        TheSame2[1]   = 0;
                    }
                    else if (Math.Abs(dist - R1 - R2 - Radius * 2.0) <= Tol)
                    {
                        WellDone = true;
                        NbrSol   = 1;
                        Axis2D ax = new(new Point2D(center1.XY() + (R1 + Radius) * dir1.XY()), dirx);
                        cirsol[1] = new Circle2D(ax, Radius);
                        //         ================================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified2.Qualifier();
                        pnttg1sol[1]  = new Point2D(center1.XY() + R1 * dir1.XY());
                        pnttg2sol[1]  = new Point2D(center2.XY() - R2 * dir1.XY());
                        TheSame1[1]   = 0;
                        TheSame2[1]   = 0;
                    }
                    else
                    {
                        C[1]  = new Circle2D(new Axis2D(C[1].Location(), dirx), Radius + R1);
                        C[2]  = new Circle2D(new Axis2D(C[2].Location(), dirx), Radius + R2);
                        nbsol = 1;
                    }
                }
            }
            else if (((Qualified1.IsOutside()) && (Qualified2.IsUnqualified())) ||
                              //   ======================================================================
                              ((Qualified1.IsUnqualified()) && (Qualified2.IsOutside())))
            {
                //            ===========================================================
                if (Qualified1.IsUnqualified())
                {
                    C[1]    = C2;
                    R3      = R1;
                    R1      = R2;
                    C[2]    = C1;
                    R2      = R3;
                    center3 = center1;
                    center1 = center2;
                    center2 = center3;
                    dir1.Reverse();
                    // it is necessary to swap the resulting tangency points
                    invers = true;
                }
                if (Tol < R1 - dist - R2)
                {
                    WellDone = true;
                }
                else if ((Tol < R2 - dist - R1) && (Tol < Radius * 2.0 - R2 - dist + R1))
                {
                    WellDone = true;
                }
                else if ((dist - R1 - R2 > Tol) && (Tol < dist - R1 - R2 - Radius * 2.0))
                {
                    WellDone = true;
                }
                else
                {
                    if ((Math.Abs(R1 - R2 - dist) <= Tol)
                            || ((Math.Abs(dist - R1 - R2) <= Tol)
                                    && (Math.Abs(Radius * 2.0 - dist + R1 + R2) <= Tol))
                            || ((Math.Abs(dist + R1 - R2) <= Tol)
                                    && (Math.Abs(R2 + dist - R1 - Radius * 2.0) <= Tol)))
                    {
                        WellDone = true;
                        NbrSol   = 1;
                        Axis2D axe = new(new Point2D(center1.XY() + (Radius + R1) * dir1.XY()), dirx);
                        cirsol[1] = new Circle2D(axe, Radius);
                        //         =================================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified2.Qualifier();
                        pnttg1sol[1]  = new Point2D(center1.XY() + R1 * dir1.XY());
                        pnttg2sol[1]  = new Point2D(center1.XY() + (dist + R2) * dir1.XY());
                        TheSame1[1]   = 0;
                        TheSame2[1]   = 0;
                    }
                    else
                    {
                        C[3]  = new Circle2D(new Axis2D(C[1].Location(), dirx), Radius + R1);
                        C[4]  = new Circle2D(new Axis2D(C[2].Location(), dirx), Radius + R2);
                        C[1]  = new Circle2D(new Axis2D(C[1].Location(), dirx), Radius + R1);
                        C[2]  = new Circle2D(new Axis2D(C[1].Location(), dirx), Math.Abs(Radius - R2)); // OCCT: centred on C(1), not C(2)
                        nbsol = 2;
                    }
                }
            }
            else if ((Qualified1.IsUnqualified()) && (Qualified2.IsUnqualified()))
            {
                //   ======================================================================
                if ((dist - R1 - R2 > Tol) && (Tol < (dist - R1 - R2 - Radius * 2)))
                {
                    WellDone = true;
                }
                else if ((Max(R1, R2) - dist - Min(R1, R2) > Tol)
                                  && (((Max(R1, R2) - dist - Min(R1, R2)) - Radius * 2.0 > Tol)))
                {
                    WellDone = true;
                }
                else
                {
                    double p3 = Max(R1, R2) - Min(R1, R2) - dist - Radius * 2.0;
                    double p4 = dist - R1 - R2;
                    double p5 = Radius * 2.0 - dist + R1 + R2;
                    if (p3 > 0.0)
                    {
                        dist = Max(R1, R2) - Min(R1, R2) - Radius * 2.0;
                    }
                    else if (p4 > 0.0 && p5 < 0.0)
                    {
                        R1 = dist - R2 - Radius * 2.0;
                    }
                    C[1]  = new Circle2D(new Axis2D(center1, dirx), Math.Abs(Radius - R1));
                    C[2]  = new Circle2D(new Axis2D(center2, dirx), Math.Abs(Radius - R2));
                    C[3]  = new Circle2D(new Axis2D(center1, dirx), Math.Abs(Radius - R1));
                    C[4]  = new Circle2D(new Axis2D(center2, dirx), Radius + R2);
                    C[5]  = new Circle2D(new Axis2D(center1, dirx), Radius + R1);
                    C[6]  = new Circle2D(new Axis2D(center2, dirx), Math.Abs(Radius - R2));
                    C[7]  = new Circle2D(new Axis2D(center1, dirx), Radius + R1);
                    C[8]  = new Circle2D(new Axis2D(center2, dirx), Radius + R2);
                    nbsol = 4;
                }
            }
            if (nbsol > 0)
            {
                for (int j = 1; j <= nbsol; j++)
                {
                    IntAna2d_AnaIntersection Intp = new(C[2 * j - 1], C[2 * j]);
                    if (Intp.IsDone())
                    {
                        if (!Intp.IsEmpty())
                        {
                            for (int i = 1; i <= Intp.NbPoints(); i++)
                            {
                                NbrSol++;
                                Point2D Center = Intp.Point(i).Value();
                                cirsol[NbrSol] = new Circle2D(new Axis2D(Center, dirx), Radius);
                                //             =======================================================
                                dir1 = new Direction2D(Center.XY() - center1.XY());
                                Direction2D dir2 = new(Center.XY() - center2.XY());
                                double   distcc1 = Center.Distance(center1);
                                double   distcc2 = Center.Distance(center2);
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
                                else if (Math.Abs(distcc2 + Radius - R2) < Tol)
                                {
                                    qualifier2[NbrSol] = GccEnt_enclosed;
                                }
                                else if (Math.Abs(distcc2 - R2 - Radius) < Tol)
                                {
                                    qualifier2[NbrSol] = GccEnt_outside;
                                }
                                else
                                {
                                    qualifier2[NbrSol] = GccEnt_enclosing;
                                }
                                if ((Center.Distance(center1) > R1) && (Radius < Center.Distance(center1) + R1))
                                {
                                    pnttg1sol[NbrSol] = new Point2D(Center.XY() - Radius * dir1.XY());
                                }
                                else if ((Center.Distance(center1) < R1) && (Radius < R1))
                                {
                                    pnttg1sol[NbrSol] = new Point2D(Center.XY() + Radius * dir1.XY());
                                }
                                else
                                {
                                    pnttg1sol[NbrSol] = new Point2D(Center.XY() - Radius * dir1.XY());
                                }
                                if ((Center.Distance(center2) > R2) && (Radius < Center.Distance(center2) + R2))
                                {
                                    pnttg2sol[NbrSol] = new Point2D(Center.XY() - Radius * dir2.XY());
                                }
                                else if ((Center.Distance(center2) < R2) && (Radius < R2))
                                {
                                    pnttg2sol[NbrSol] = new Point2D(Center.XY() + Radius * dir2.XY());
                                }
                                else
                                {
                                    pnttg2sol[NbrSol] = new Point2D(Center.XY() - Radius * dir2.XY());
                                }
                                TheSame1[NbrSol] = 0;
                                TheSame2[NbrSol] = 0;
                            }
                        }
                        WellDone = true;
                    }
                }
            }
        }
        // swapping of resulting tangency points if necessary
        if (invers)
        {
            Point2D Psav = new();
            for (int i = 1; i <= NbrSol; i++)
            {
                Psav         = pnttg1sol[i];
                pnttg1sol[i] = pnttg2sol[i];
                pnttg2sol[i] = Psav;
            }
        }
        // calculation of parameters of tangency points
        for (int i = 1; i <= NbrSol; i++)
        {
            par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[i], pnttg1sol[i]);
            if (TheSame1[i] == 0)
            {
                pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C1, pnttg1sol[i]);
            }
            par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(cirsol[i], pnttg2sol[i]);
            if (TheSame2[i] == 0)
            {
                pararg2[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(C2, pnttg2sol[i]);
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
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        return cirsol[Index];
    }

    /// <summary>The qualifiers of the arguments for the solution number Index: those given, or those computed for unqualified arguments (GccEnt_noqualifier for a point).</summary>
    public void WhichQualifier(int Index, out GccEnt_Position Qualif1, out GccEnt_Position Qualif2)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }

        Qualif1 = qualifier1[Index];
        Qualif2 = qualifier2[Index];
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
            if (TheSame1[Index] == 0)
            {
                ParSol = par1sol[Index];
                ParArg = pararg1[Index];
                PntSol = pnttg1sol[Index];
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>The tangency point of the solution number Index with the second argument: its parameter on the solution, its parameter on the argument, and the point.</summary>
    public void Tangency2(int Index, out double ParSol, out double ParArg, out Point2D PntSol)
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
            if (TheSame2[Index] == 0)
            {
                ParSol = par2sol[Index];
                ParArg = pararg2[Index];
                PntSol = pnttg2sol[Index];
            }
            else
            {
                throw new NotDoneException();
            }
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

    /// <summary>True if the solution number Index is the second argument.</summary>
    public bool IsTheSame2(int Index)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }

        if (TheSame2[Index] == 0)
        {
            return false;
        }
        return true;
    }
}
