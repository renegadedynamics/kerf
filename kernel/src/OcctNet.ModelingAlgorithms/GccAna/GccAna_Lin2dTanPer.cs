// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dTanPer.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dTanPer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// 2D lines through a point or tangent to a circle, and perpendicular to a line or to a circle. The origin of each
/// solution is its tangency point with the first argument.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). As in OCCT, the qualifier of a point argument, and
/// of a circle in the circle/line case, is never set: WhichQualifier returns GccEnt_unqualified there (uninitialised
/// memory in OCCT).
/// </remarks>
public sealed class GccAna_Lin2dTanPer
{
    private bool WellDone;
    private int NbrSol;
    private readonly NCollection_Array1<gp_Lin2d> linsol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<gp_Pnt2d> pnttg1sol;
    private readonly NCollection_Array1<gp_Pnt2d> pntint2sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> par2sol;
    private readonly NCollection_Array1<double> pararg1;
    private readonly NCollection_Array1<double> pararg2;

    private GccAna_Lin2dTanPer(int theSize)
    {
        linsol = new NCollection_Array1<gp_Lin2d>(1, theSize);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, theSize);
        pnttg1sol = new NCollection_Array1<gp_Pnt2d>(1, theSize);
        pntint2sol = new NCollection_Array1<gp_Pnt2d>(1, theSize);
        par1sol = new NCollection_Array1<double>(1, theSize);
        par2sol = new NCollection_Array1<double>(1, theSize);
        pararg1 = new NCollection_Array1<double>(1, theSize);
        pararg2 = new NCollection_Array1<double>(1, theSize);
    }

    //=========================================================================
    //   Straight line passing by point  : ThePoint and                       +
    //       orthogonal to straight line : TheLin.                            +
    //   Create the straight line of origin     : ThePoint                           +
    //                  and direction  : TheLin.Direction() turned by 90   +
    //=========================================================================

    /// <summary>The line through ThePnt perpendicular to TheLin.</summary>
    public GccAna_Lin2dTanPer(in gp_Pnt2d ThePnt, in gp_Lin2d TheLin) : this(1)
    {
        linsol[1] = new gp_Lin2d(ThePnt,
                                 new gp_Dir2d(-(TheLin.Direction().Y()),
                                              // ===============================================================
                                              TheLin.Direction().X()));
        //                                      ========================
        pnttg1sol[1] = ThePnt;
        IntAna2d_AnaIntersection Intp = new(linsol[1], TheLin);
        if (Intp.IsDone())
        {
            if (!Intp.IsEmpty())
            {
                for (int i = 1; i <= Intp.NbPoints(); i++)
                {
                    pntint2sol[1] = Intp.Point(i).Value();
                }
            }
        }
        par1sol[1] = ElCLib.Parameter(linsol[1], pnttg1sol[1]);
        par2sol[1] = ElCLib.Parameter(linsol[1], pntint2sol[1]);
        pararg1[1] = 0.0;
        pararg2[1] = ElCLib.Parameter(TheLin, pntint2sol[1]);
        NbrSol = 1;
        WellDone = true;
    }

    //=========================================================================
    //   Straight line passing by point      : ThePnt                             +
    //       and orthogonal to circle        : TheCircle.                         +
    //   Create the straight line of origin  : ThePoint                           +
    //                  and direction        : (TheCircle.Location(),ThePnt).     +
    //=========================================================================

    /// <summary>The line through ThePnt and the centre of TheCircle (perpendicular to the circle).</summary>
    public GccAna_Lin2dTanPer(in gp_Pnt2d ThePnt, in gp_Circ2d TheCircle) : this(1)
    {
        linsol[1] = new gp_Lin2d(ThePnt,
                                 // ============================
                                 new gp_Dir2d(TheCircle.Location().XY() - ThePnt.XY()));
        //                      ================================================
        pnttg1sol[1] = ThePnt;
        IntAna2d_AnaIntersection Intp = new(linsol[1], TheCircle);
        if (Intp.IsDone())
        {
            if (!Intp.IsEmpty())
            {
                double maxdist = RealLast();
                for (int i = 1; i <= Intp.NbPoints(); i++)
                {
                    // OCCT: maxdist is never lowered, so this keeps the last intersection point, not the nearest
                    if (Intp.Point(i).Value().Distance(ThePnt) < maxdist)
                    {
                        pntint2sol[1] = Intp.Point(i).Value();
                    }
                }
            }
        }
        par1sol[1] = ElCLib.Parameter(linsol[1], pnttg1sol[1]);
        par2sol[1] = ElCLib.Parameter(linsol[1], pntint2sol[1]);
        pararg1[1] = 0.0;
        // a circle of radius 0 puts the intersection at its centre
        pararg2[1] = ElCLib.Parameter(TheCircle, pntint2sol[1]);
        NbrSol = 1;
        WellDone = true;
    }

    //=========================================================================
    //   Straight line tangent to circle     : Qualified1 (C1)                    +
    //   and orthogonal to straight line     : TheLin.                            +
    //   Create straight line of origin      : P1 (on C1)                         +
    //                  and direction        : TheLin.Direction() turned by 90`   +
    //=========================================================================

    /// <summary>The lines tangent to the qualified circle and perpendicular to TheLin.</summary>
    /// <exception cref="GccEnt_BadQualifier">The circle is qualified enclosed.</exception>
    public GccAna_Lin2dTanPer(in GccEnt_QualifiedCirc Qualified1, in gp_Lin2d TheLin) : this(2)
    {
        WellDone = false;
        int nbsol = 0;
        int signe = 0;
        NbrSol = 0;
        gp_Circ2d C1 = Qualified1.Qualified();

        // OCCT: qualifier1 is never set by this constructor
        if (Qualified1.IsEnclosed())
        {
            // ============================
            throw new GccEnt_BadQualifier();
        }
        else if (Qualified1.IsEnclosing())
        {
            // ==================================
            nbsol = 1;
            signe = -1;
        }
        else if (Qualified1.IsOutside())
        {
            // ================================
            nbsol = 1;
            signe = 1;
        }
        else
        {
            nbsol = 2;
            signe = -1;
        }
        gp_XY xy = C1.Radius() * TheLin.Direction().XY();
        for (int j = 1; j <= nbsol; j++)
        {
            signe = -signe;
            NbrSol++;
            linsol[NbrSol] = new gp_Lin2d(new gp_Pnt2d((C1.Location().XY()).Added(signe * xy)),
                                          //   =======================================================================
                                          new gp_Dir2d(-TheLin.Direction().Y(),
                                                       //                           =================================
                                                       TheLin.Direction().X()));
            //                                    ========================
            pnttg1sol[NbrSol] = new gp_Pnt2d((C1.Location().XY()).Added(signe * xy));
            IntAna2d_AnaIntersection Intp = new(linsol[NbrSol], TheLin);
            if (Intp.IsDone())
            {
                if (!Intp.IsEmpty())
                {
                    for (int i = 1; i <= Intp.NbPoints(); i++)
                    {
                        pntint2sol[NbrSol] = Intp.Point(i).Value();
                    }
                }
            }
            par1sol[NbrSol] = ElCLib.Parameter(linsol[NbrSol], pnttg1sol[NbrSol]);
            par2sol[NbrSol] = ElCLib.Parameter(linsol[NbrSol], pntint2sol[NbrSol]);
            // a circle of radius 0 puts the tangency point at its centre
            pararg1[NbrSol] = ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
            pararg2[NbrSol] = ElCLib.Parameter(TheLin, pntint2sol[NbrSol]);
            WellDone = true;
        }
    }

    //=========================================================================
    //   Straight line tangent to circle     : Qualified1 (C1)                    +
    //       and orthogonal to circle        : TheCircle.                         +
    //   Create straight line of origin      : P1 (on C1)                         +
    //                  and direction        : TheLin.Direction() turned by 90`   +
    //=========================================================================

    /// <summary>
    /// Lines tangent to the qualified circle, at its points on the line of centres, so perpendicular to that line
    /// (as OCCT computes them: they don't pass through the centre of TheCircle).
    /// </summary>
    /// <exception cref="GccEnt_BadQualifier">The circle is qualified enclosed.</exception>
    public GccAna_Lin2dTanPer(in GccEnt_QualifiedCirc Qualified1, in gp_Circ2d TheCircle) : this(2)
    {
        WellDone = false;
        NbrSol = 0;
        int signe = 0;
        gp_Circ2d C1 = Qualified1.Qualified();

        if (Qualified1.IsEnclosed())
        {
            // ============================
            throw new GccEnt_BadQualifier();
        }
        else if (Qualified1.IsEnclosing())
        {
            // ==================================
            signe = -1;
            qualifier1[1] = GccEnt_enclosing;
        }
        else if (Qualified1.IsOutside())
        {
            // ================================
            signe = 1;
            qualifier1[1] = GccEnt_outside;
        }
        else if (Qualified1.IsUnqualified())
        {
            // ====================================
            signe = -1;
            qualifier1[1] = GccEnt_enclosing;
            qualifier1[2] = GccEnt_outside;
        }
        // OCCT: always two solutions, whatever the qualifier (qualifier1(2) is then unset), and the lines are
        // perpendicular to the line of centres instead of passing through TheCircle's centre
        for (int j = 1; j <= 2; j++)
        {
            NbrSol++;
            signe = -signe;
            gp_Dir2d D1 = new(TheCircle.Location().XY() - C1.Location().XY());
            linsol[NbrSol] = new gp_Lin2d(new gp_Pnt2d((C1.Location().XY())
                                                       //   ===================================================
                                                       + signe * (D1.XY() * C1.Radius())),
                                          new gp_Dir2d(-D1.Y(), D1.X()));
            //                     ======================================================
            pnttg1sol[NbrSol] = new gp_Pnt2d((C1.Location().XY()) + signe * (D1.XY() * C1.Radius()));
            IntAna2d_AnaIntersection Intp = new(linsol[NbrSol], TheCircle);
            if (Intp.IsDone())
            {
                if (!Intp.IsEmpty())
                {
                    double maxdist = RealLast();
                    for (int i = 1; i <= Intp.NbPoints(); i++)
                    {
                        // OCCT: maxdist is never lowered, so this keeps the last intersection point
                        if (Intp.Point(i).Value().Distance(pnttg1sol[NbrSol]) < maxdist)
                        {
                            pntint2sol[NbrSol] = Intp.Point(i).Value();
                        }
                    }
                }
            }
            par1sol[NbrSol] = ElCLib.Parameter(linsol[NbrSol], pnttg1sol[NbrSol]);
            par2sol[NbrSol] = ElCLib.Parameter(linsol[NbrSol], pntint2sol[NbrSol]);
            // a circle of radius 0 puts the tangency point at its centre, and a line missing TheCircle leaves the
            // intersection at the origin, which may be TheCircle's centre
            pararg1[NbrSol] = ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
            pararg2[NbrSol] = ElCLib.Parameter(TheCircle, pntint2sol[NbrSol]);
            WellDone = true;
        }
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The number of solutions.</summary>
    public int NbSolutions()
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        return NbrSol;
    }

    /// <summary>
    /// The qualifier of the first argument for the solution number Index (see the remarks for the cases OCCT leaves
    /// unset).
    /// </summary>
    public void WhichQualifier(int Index, out GccEnt_Position Qualif1)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }
        else
        {
            Qualif1 = qualifier1[Index];
        }
    }

    /// <summary>The solution number Index.</summary>
    /// <exception cref="Standard_RangeError">Index is out of range (OCCT throws this base class here).</exception>
    public gp_Lin2d ThisSolution(int Index)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_RangeError();
        }
        return linsol[Index];
    }

    /// <summary>The tangency point of the solution number Index with the first argument, and its parameters.</summary>
    public void Tangency1(int Index, out double ParSol, out double ParArg, out gp_Pnt2d Pnt)
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
            Pnt = pnttg1sol[Index];
        }
    }

    /// <summary>The intersection of the solution number Index with the second argument, and its parameters.</summary>
    public void Intersection2(int Index, out double ParSol, out double ParArg, out gp_Pnt2d PntSol)
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
            PntSol = pntint2sol[Index];
        }
    }
}
