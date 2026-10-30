// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2d2Tan.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2d2Tan.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// 2D lines tangent to two circles (up to four), tangent to a circle and through a point (up to two), or through two
/// points. A circle may be qualified enclosing, outside or unqualified (not enclosed).
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). As in OCCT, WhichQualifier returns the qualifiers
/// given (GccEnt_unqualified for an unqualified circle, not a computed one), and the second circle's qualifier is
/// mostly the first's.
/// </remarks>
public sealed class GccAna_Lin2d2Tan
{
    private bool WellDone;
    private int NbrSol;
    private readonly NCollection_Array1<gp_Lin2d> linsol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<GccEnt_Position> qualifier2;
    private readonly NCollection_Array1<gp_Pnt2d> pnttg1sol;
    private readonly NCollection_Array1<gp_Pnt2d> pnttg2sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> par2sol;
    private readonly NCollection_Array1<double> pararg1;
    private readonly NCollection_Array1<double> pararg2;

    private GccAna_Lin2d2Tan(int theSize)
    {
        linsol = new NCollection_Array1<gp_Lin2d>(1, theSize);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, theSize);
        qualifier2 = new NCollection_Array1<GccEnt_Position>(1, theSize);
        pnttg1sol = new NCollection_Array1<gp_Pnt2d>(1, theSize);
        pnttg2sol = new NCollection_Array1<gp_Pnt2d>(1, theSize);
        par1sol = new NCollection_Array1<double>(1, theSize);
        par2sol = new NCollection_Array1<double>(1, theSize);
        pararg1 = new NCollection_Array1<double>(1, theSize);
        pararg2 = new NCollection_Array1<double>(1, theSize);
    }

    //=========================================================================
    //   Straight line passing through two points.                                      +
    //   ===============================                                      +
    //=========================================================================

    /// <summary>The line through two points, if they are at least Tolerance apart (else not done).</summary>
    public GccAna_Lin2d2Tan(in gp_Pnt2d ThePoint1, in gp_Pnt2d ThePoint2, double Tolerance) : this(1)
    {
        double Tol = Math.Abs(Tolerance);
        WellDone = false;
        NbrSol = 0;
        double dist = ThePoint1.Distance(ThePoint2);
        qualifier1[1] = GccEnt_noqualifier;
        qualifier2[1] = GccEnt_noqualifier;
        if (dist >= Tol)
        {
            gp_Dir2d dir = new(ThePoint2.X() - ThePoint1.X(), ThePoint2.Y() - ThePoint1.Y());
            linsol[1] = new gp_Lin2d(ThePoint1, dir);
            //   ===================================
            WellDone = true;
            NbrSol = 1;
            pnttg1sol[1] = ThePoint1;
            pnttg2sol[1] = ThePoint2;
            par1sol[NbrSol] = ElCLib.Parameter(linsol[NbrSol], pnttg1sol[NbrSol]);
            par2sol[NbrSol] = ElCLib.Parameter(linsol[NbrSol], pnttg2sol[NbrSol]);
            pararg1[1] = 0.0;
            pararg2[1] = 0.0;
        }
    }

    //=========================================================================
    //   Straight line tangent to a circle passing by a point.                +
    //   =================================================                    +
    //   Basing on the qualifier attached to circle Qualified1 (C1) define    +
    //   the direction of the tangent to be calculated.                       +
    //   This tangent will have connection point P1 (point of tangency with the circle. +
    //   It has angle A (sinus R1/dist or -R1/dist) with straight line        +
    //   passing by the center of the circle and ThePoint.                    +
    //=========================================================================

    /// <summary>The lines tangent to the qualified circle and through ThePoint.</summary>
    /// <exception cref="GccEnt_BadQualifier">The circle is qualified enclosed, or GccEnt_noqualifier.</exception>
    public GccAna_Lin2d2Tan(in GccEnt_QualifiedCirc Qualified1, in gp_Pnt2d ThePoint, double Tolerance) : this(2)
    {
        double Tol = Math.Abs(Tolerance);
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        gp_Circ2d C1 = Qualified1.Qualified();
        double R1 = C1.Radius();

        if (Qualified1.IsEnclosed())
        {
            throw new GccEnt_BadQualifier();
        }
        // ============================
        else if (Tol < R1 - ThePoint.Distance(C1.Location()))
        {
            WellDone = true;
        }
        else if (Math.Abs(ThePoint.Distance(C1.Location()) - R1) <= Tol)
        {
            gp_Dir2d dir = new(new gp_Vec2d(C1.Location(), ThePoint));
            linsol[1] = new gp_Lin2d(ThePoint, new gp_Dir2d(-dir.Y(), dir.X()));
            //   =====================================================================
            qualifier1[1] = Qualified1.Qualifier();
            qualifier2[1] = GccEnt_noqualifier;
            WellDone = true;
            NbrSol = 1;
            pnttg1sol[1] = ThePoint;
            pnttg2sol[1] = ThePoint;
        }
        else
        {
            double signe = 1;
            double dist = ThePoint.Distance(C1.Location());
            double d = dist - Math.Sqrt(dist * dist - R1 * R1);
            if (Qualified1.IsEnclosing())
            {
                //   =============================
                signe = 1;
                NbrSol = 1;
            }
            else if (Qualified1.IsOutside())
            {
                signe = -1;
                NbrSol = 1;
            }
            else if (Qualified1.IsUnqualified())
            {
                //   ====================================
                signe = 1;
                NbrSol = 2;
            }
            for (int i = 1; i <= NbrSol; i++)
            {
                gp_Pnt2d P1 = C1.Location().Rotated(ThePoint, Math.Asin(signe * R1 / dist));
                gp_Dir2d D1 = new(new gp_Vec2d(P1, ThePoint));
                P1 = new gp_Pnt2d(P1.XY() + d * D1.XY());
                linsol[i] = new gp_Lin2d(P1, new gp_Dir2d(new gp_Vec2d(P1, ThePoint)));
                //     ========================================================
                qualifier1[i] = Qualified1.Qualifier();
                qualifier2[i] = GccEnt_noqualifier;
                pnttg1sol[i] = P1;
                pnttg2sol[i] = ThePoint;
                signe = -signe;
            }
            WellDone = true;
        }

        for (int i = 1; i <= NbrSol; i++)
        {
            par1sol[i] = ElCLib.Parameter(linsol[i], pnttg1sol[i]);
            par2sol[i] = ElCLib.Parameter(linsol[i], pnttg2sol[i]);
            // a circle of radius 0 puts the tangency point at its centre
            pararg1[i] = ElCLib.Parameter(C1, pnttg1sol[i]);
            pararg2[i] = 0.0;
        }
    }

    //=========================================================================
    //   Straight line tangent to two circles.                                +
    //   ====================================                                 +
    //   In the boundary cas (two interior circles are tangent to each other) +
    //   take the straight line orthogonal to the straight line connecting    +
    //   two circles.                                                         +
    //   In other cases subject the center of C1 (Qualified1) or of           +
    //   C2 (Qualified2), provided that R1 is greater than R2, to a           +
    //   rotation of angle A with sinus(A) = (R1+R2)/dist or                  +
    //                            sinus(A) = (R1-R2)/dist or                  +
    //                            sinus(A) = (R2-R1)/dist or                  +
    //                            sinus(A) = (-R1-R2)/dist                    +
    //   The point found this way is P1 or P2.                                +
    //   The direction of the straight line to be calculated  should  pass by +
    //   the center of rotation (center of C1 or of C2) and P1 or P2.         +
    //   Then translate the straight line to make it tangent to C1.           +
    //=========================================================================

    /// <summary>The lines tangent to two qualified circles.</summary>
    /// <exception cref="GccEnt_BadQualifier">A circle is qualified enclosed, or GccEnt_noqualifier.</exception>
    public GccAna_Lin2d2Tan(in GccEnt_QualifiedCirc Qualified1, in GccEnt_QualifiedCirc Qualified2, double Tolerance)
        : this(4)
    {
        double Tol = Math.Abs(Tolerance);
        WellDone = false;
        NbrSol = 0;
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                 || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        gp_Circ2d C1 = Qualified1.Qualified();
        gp_Circ2d C2 = Qualified2.Qualified();

        if (Qualified1.IsEnclosed() || Qualified2.IsEnclosed())
        {
            // =======================================================
            throw new GccEnt_BadQualifier();
        }
        else
        {
            double R1 = C1.Radius();
            double R2 = C2.Radius();
            gp_Dir2d D1 = new();
            int signe = 1;
            double dist = C1.Location().Distance(C2.Location());
            if (Tol < Max(R1, R2) - dist - Min(R1, R2))
            {
                WellDone = true;
            }
            else if (!Qualified1.IsUnqualified() || !Qualified2.IsUnqualified())
            {
                //   ====================================================================
                if (Qualified1.IsEnclosing() && Qualified2.IsEnclosing())
                {
                    //     =========================================================
                    if (Math.Abs(dist + Min(R1, R2) - Max(R1, R2)) <= Tol && dist >= Tol)
                    {
                        if (R1 < R2)
                        {
                            D1 = new gp_Dir2d(new gp_Vec2d(C2.Location(), C1.Location()));
                        }
                        else
                        {
                            D1 = new gp_Dir2d(new gp_Vec2d(C1.Location(), C2.Location()));
                        }
                        gp_Pnt2d P1 = new(C1.Location().XY() + R1 * D1.XY());
                        linsol[1] = new gp_Lin2d(P1, new gp_Dir2d(-D1.Y(), D1.X()));
                        //         =================================================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified2.Qualifier();
                        pnttg1sol[1] = P1;
                        pnttg2sol[1] = P1;
                        WellDone = true;
                        NbrSol = 1;
                    }
                    else
                    {
                        gp_Pnt2d P1 = C2.Location().Rotated(C1.Location(), Math.Asin((R1 - R2) / dist));
                        D1 = new gp_Dir2d(new gp_Vec2d(C1.Location(), P1));
                        P1 = new gp_Pnt2d((C1.Location().XY() + new gp_XY(R1 * D1.Y(), -R1 * D1.X())));
                        linsol[1] = new gp_Lin2d(P1, D1);
                        //         ===========================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                        pnttg1sol[1] = P1;
                        pnttg2sol[1] = new gp_Pnt2d(C2.Location().XY() + new gp_XY(R2 * D1.Y(), -R2 * D1.X()));
                        WellDone = true;
                        NbrSol = 1;
                    }
                }
                else if ((Qualified1.IsEnclosing() && Qualified2.IsOutside()) ||
                         //     ================================================================
                         (Qualified2.IsEnclosing() && Qualified1.IsOutside()))
                {
                    //              =====================================================
                    if (Qualified1.IsEnclosing() && Qualified2.IsOutside())
                    {
                        signe = 1;
                    }
                    else
                    {
                        signe = -1;
                    }
                    if (R1 + R2 - dist > Tol)
                    {
                        WellDone = true;
                    }
                    else if (Math.Abs(dist - R1 - R2) < Tol && dist > Tol)
                    {
                        D1 = new gp_Dir2d(new gp_Vec2d(C1.Location(), C2.Location()));
                        gp_Pnt2d P1 = new(C1.Location().XY() + R1 * D1.XY());
                        linsol[1] = new gp_Lin2d(P1, new gp_Dir2d(-D1.Y(), D1.X()));
                        //          =================================================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                        pnttg1sol[1] = P1;
                        pnttg2sol[1] = P1;
                        WellDone = true;
                        NbrSol = 1;
                    }
                    else
                    {
                        gp_Pnt2d P1 = C2.Location().Rotated(C1.Location(), Math.Asin(signe * (R1 + R2) / dist));
                        D1 = new gp_Dir2d(new gp_Vec2d(C1.Location(), P1));
                        P1 = new gp_Pnt2d(C1.Location().XY() + signe * (new gp_XY(R1 * D1.Y(), -R1 * D1.X())));
                        linsol[1] = new gp_Lin2d(P1, D1);
                        //         ===========================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                        pnttg1sol[1] = P1;
                        pnttg2sol[1] = new gp_Pnt2d(C2.Location().XY() + signe * (new gp_XY(-R2 * D1.Y(), R2 * D1.X())));
                        WellDone = true;
                        NbrSol = 1;
                    }
                }
                else if (Qualified1.IsOutside() && Qualified2.IsOutside())
                {
                    //     =========================================================
                    if (Math.Abs(dist + Min(R1, R2) - Max(R1, R2)) < Tol && dist > Tol)
                    {
                        if (R1 < R2)
                        {
                            D1 = new gp_Dir2d(new gp_Vec2d(C2.Location(), C1.Location()));
                        }
                        else
                        {
                            D1 = new gp_Dir2d(new gp_Vec2d(C1.Location(), C2.Location()));
                        }
                        linsol[1] = new gp_Lin2d(new gp_Pnt2d(C1.Location().XY() + R1 * D1.XY()),
                                                 //         =============================================================
                                                 new gp_Dir2d(D1.Y(), -D1.X()));
                        //                              =========================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                        pnttg1sol[1] = new gp_Pnt2d(C1.Location().XY() + R1 * D1.XY());
                        pnttg2sol[1] = pnttg1sol[1];
                        WellDone = true;
                        NbrSol = 1;
                    }
                    else
                    {
                        gp_Pnt2d P1 = C2.Location().Rotated(C1.Location(), Math.Asin((R2 - R1) / dist));
                        D1 = new gp_Dir2d(new gp_Vec2d(C1.Location(), P1));
                        P1 = new gp_Pnt2d(C1.Location().XY() + new gp_XY(-R1 * D1.Y(), R1 * D1.X()));
                        linsol[1] = new gp_Lin2d(P1, D1);
                        //         ===========================
                        qualifier1[1] = Qualified1.Qualifier();
                        qualifier2[1] = Qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                        pnttg1sol[1] = P1;
                        pnttg2sol[1] = new gp_Pnt2d(C2.Location().XY() + (new gp_XY(-R2 * D1.Y(), R2 * D1.X())));
                        WellDone = true;
                        NbrSol = 1;
                    }
                }
                else
                {
                    if ((Qualified1.IsUnqualified() && Qualified2.IsEnclosing()) ||
                        //       ===============================================================
                        (Qualified2.IsUnqualified() && Qualified1.IsEnclosing()))
                    {
                        //           =========================================================
                        if (Qualified2.IsUnqualified())
                        {
                            signe = 1;
                        }
                        else
                        {
                            signe = -1;
                        }
                        if (Math.Abs(dist + Min(R1, R2) - Max(R1, R2)) < Tol && dist > Tol)
                        {
                            if (R1 < R2)
                            {
                                D1 = new gp_Dir2d(new gp_Vec2d(C2.Location(), C1.Location()));
                            }
                            else
                            {
                                D1 = new gp_Dir2d(new gp_Vec2d(C1.Location(), C2.Location()));
                            }
                            linsol[1] = new gp_Lin2d(new gp_Pnt2d(C1.Location().XY() + R1 * D1.XY()),
                                                     //           =============================================================
                                                     new gp_Dir2d(-D1.Y(), D1.X()));
                            //                                =========================
                            qualifier1[1] = Qualified1.Qualifier();
                            qualifier2[1] = Qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                            pnttg1sol[1] = new gp_Pnt2d(C1.Location().XY() + R1 * D1.XY());
                            pnttg2sol[1] = pnttg1sol[1];
                            WellDone = true;
                            NbrSol = 1;
                        }
                        else
                        {
                            gp_Pnt2d P1 = C2.Location().Rotated(C1.Location(), Math.Asin((R1 - R2) / dist));
                            D1 = new gp_Dir2d(new gp_Vec2d(C1.Location(), P1));
                            P1 = new gp_Pnt2d(C1.Location().XY() + new gp_XY(R1 * D1.Y(), -R1 * D1.X()));
                            linsol[1] = new gp_Lin2d(P1, D1);
                            //           ===========================
                            qualifier1[1] = Qualified1.Qualifier();
                            qualifier2[1] = Qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                            pnttg1sol[1] = P1;
                            pnttg2sol[1] =
                                new gp_Pnt2d(C2.Location().XY() + signe * (new gp_XY(R2 * D1.Y(), -R2 * D1.X())));
                            WellDone = true;
                            NbrSol = 1;
                            if (Min(R1, R2) + Max(R1, R2) < dist)
                            {
                                gp_Pnt2d P2 =
                                    C2.Location().Rotated(C1.Location(), Math.Asin(signe * (R1 + R2) / dist));
                                gp_Dir2d D2 = new(new gp_Vec2d(C1.Location(), P2));
                                P2 = new gp_Pnt2d(C1.Location().XY() + signe * (new gp_XY(R1 * D2.Y(), -R1 * D2.X())));
                                linsol[2] = new gp_Lin2d(P2, D2);
                                //             ===========================
                                // OCCT: writes the first solution's qualifiers (the second's stay unset) and gives the
                                // second solution the first's tangency point P1
                                qualifier1[1] = Qualified1.Qualifier();
                                qualifier2[1] = Qualified1.Qualifier();
                                pnttg1sol[2] = P1;
                                pnttg2sol[2] = new gp_Pnt2d(C2.Location().XY() + (new gp_XY(-R2 * D2.Y(), R2 * D2.X())));
                                NbrSol = 2;
                            }
                        }
                    }
                    else if ((Qualified1.IsUnqualified() && Qualified2.IsOutside()) ||
                             //       ==================================================================
                             (Qualified2.IsUnqualified() && Qualified1.IsOutside()))
                    {
                        //           =======================================================
                        if (Qualified2.IsUnqualified())
                        {
                            signe = 1;
                        }
                        else
                        {
                            signe = -1;
                        }
                        if (Math.Abs(dist + Min(R1, R2) - Max(R1, R2)) <= Tol && dist >= Tol)
                        {
                            if (R1 < R2)
                            {
                                D1 = new gp_Dir2d(new gp_Vec2d(C2.Location(), C1.Location()));
                            }
                            else
                            {
                                D1 = new gp_Dir2d(new gp_Vec2d(C1.Location(), C2.Location()));
                            }
                            linsol[1] = new gp_Lin2d(new gp_Pnt2d(C1.Location().XY() + R1 * D1.XY()),
                                                     //           =============================================================
                                                     new gp_Dir2d(D1.Y(), -D1.X()));
                            //                                =========================
                            qualifier1[1] = Qualified1.Qualifier();
                            qualifier2[1] = Qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                            pnttg1sol[1] = new gp_Pnt2d(C1.Location().XY() + R1 * D1.XY());
                            pnttg2sol[1] = pnttg1sol[1];
                            WellDone = true;
                            NbrSol = 1;
                        }
                        else
                        {
                            gp_Pnt2d P1 = C2.Location().Rotated(C1.Location(), Math.Asin(signe * (R2 - R1) / dist));
                            D1 = new gp_Dir2d(new gp_Vec2d(C1.Location(), P1));
                            P1 = new gp_Pnt2d(C1.Location().XY() + new gp_XY(-R1 * D1.Y(), R1 * D1.X()));
                            linsol[1] = new gp_Lin2d(P1, D1);
                            //           ===========================
                            qualifier1[1] = Qualified1.Qualifier();
                            qualifier2[1] = Qualified1.Qualifier(); // OCCT: Qualified1 for the second qualifier
                            pnttg1sol[1] = P1;
                            pnttg2sol[1] =
                                new gp_Pnt2d(C2.Location().XY() + signe * (new gp_XY(-R2 * D1.Y(), R2 * D1.X())));
                            WellDone = true;
                            NbrSol = 1;
                            if (Min(R1, R2) + Max(R1, R2) < dist)
                            {
                                gp_Pnt2d P2 =
                                    C2.Location().Rotated(C1.Location(), Math.Asin(signe * (-R2 - R1) / dist));
                                gp_Dir2d D2 = new(new gp_Vec2d(C1.Location(), P2));
                                P2 = new gp_Pnt2d(C1.Location().XY() + signe * (new gp_XY(-R1 * D2.Y(), R1 * D2.X())));
                                linsol[2] = new gp_Lin2d(P2, D2);
                                //             ===========================
                                // OCCT: writes the first solution's qualifiers (the second's stay unset)
                                qualifier1[1] = Qualified1.Qualifier();
                                qualifier2[1] = Qualified1.Qualifier();
                                pnttg1sol[2] = P2;
                                pnttg2sol[2] =
                                    new gp_Pnt2d(C2.Location().XY() + signe * (new gp_XY(R2 * D2.Y(), -R2 * D2.X())));
                                NbrSol = 2;
                            }
                        }
                    }
                }
            }
            else
            {
                for (int i = 1; i <= 2; i++)
                {
                    signe = -signe;
                    NbrSol++;
                    gp_Pnt2d P1 = C2.Location().Rotated(C1.Location(), Math.Asin(signe * (R2 - R1) / dist));
                    D1 = new gp_Dir2d(new gp_Vec2d(C1.Location(), P1));
                    P1 = new gp_Pnt2d(C1.Location().XY() + signe * new gp_XY(-R1 * D1.Y(), R1 * D1.X()));
                    linsol[NbrSol] = new gp_Lin2d(P1, D1);
                    //       ===========================
                    qualifier1[NbrSol] = Qualified1.Qualifier();
                    qualifier2[NbrSol] = Qualified1.Qualifier();
                    pnttg1sol[NbrSol] = P1;
                    pnttg2sol[NbrSol] =
                        new gp_Pnt2d(C2.Location().XY() + signe * (new gp_XY(-R2 * D1.Y(), R2 * D1.X())));
                    WellDone = true;
                    if (Min(R1, R2) + Max(R1, R2) < dist)
                    {
                        gp_Pnt2d P2 = C2.Location().Rotated(C1.Location(), Math.Asin(signe * (R2 + R1) / dist));
                        gp_Dir2d D2 = new(new gp_Vec2d(C1.Location(), P2));
                        P2 = new gp_Pnt2d(C1.Location().XY() + signe * (new gp_XY(R1 * D2.Y(), -R1 * D2.X())));
                        NbrSol++;
                        linsol[NbrSol] = new gp_Lin2d(P2, D2);
                        //         ================================
                        qualifier1[NbrSol] = Qualified1.Qualifier();
                        qualifier2[NbrSol] = Qualified1.Qualifier();
                        pnttg1sol[NbrSol] = P2;
                        pnttg2sol[NbrSol] =
                            new gp_Pnt2d(C2.Location().XY() + signe * (new gp_XY(-R2 * D2.Y(), R2 * D2.X())));
                    }
                }
            }
        }

        for (int i = 1; i <= NbrSol; i++)
        {
            par1sol[i] = ElCLib.Parameter(linsol[i], pnttg1sol[i]);
            par2sol[i] = ElCLib.Parameter(linsol[i], pnttg2sol[i]);
            // circles of radius 0 put the tangency points at their centres
            pararg1[i] = ElCLib.Parameter(C1, pnttg1sol[i]);
            pararg2[i] = ElCLib.Parameter(C2, pnttg2sol[i]);
        }
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The number of solutions (OCCT doesn't check IsDone here).</summary>
    public int NbSolutions()
    {
        return NbrSol;
    }

    /// <summary>The solution number Index (OCCT doesn't check IsDone here).</summary>
    public gp_Lin2d ThisSolution(int Index)
    {
        if (Index > NbrSol || Index <= 0)
        {
            throw new Standard_OutOfRange();
        }
        return linsol[Index];
    }

    /// <summary>The qualifiers of the arguments for the solution number Index (see the remarks).</summary>
    public void WhichQualifier(int Index, out GccEnt_Position Qualif1, out GccEnt_Position Qualif2)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }
        else
        {
            Qualif1 = qualifier1[Index];
            Qualif2 = qualifier2[Index];
        }
    }

    /// <summary>The tangency point of the solution number Index with the first argument, and its parameters.</summary>
    public void Tangency1(int Index, out double ParSol, out double ParArg, out gp_Pnt2d PntSol)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }
        else
        {
            ParSol = par1sol[Index];
            ParArg = pararg1[Index];
            PntSol = pnttg1sol[Index];
        }
    }

    /// <summary>The tangency point of the solution number Index with the second argument, and its parameters.</summary>
    public void Tangency2(int Index, out double ParSol, out double ParArg, out gp_Pnt2d PntSol)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }
        else
        {
            ParSol = par2sol[Index];
            ParArg = pararg2[Index];
            PntSol = pnttg2sol[Index];
        }
    }
}
