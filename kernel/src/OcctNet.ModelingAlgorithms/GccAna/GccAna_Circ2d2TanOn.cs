// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanOn.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;
using static Occt.GccInt_IType;

namespace Occt;

/// <summary>
/// The 2D circles tangent to two curves, tangent to a curve and through a point, or through two points, with their
/// centre on a third curve. The curves are lines or circles.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). Each constructor fills its arrays as OCCT's does,
/// including OCCT's mistakes (marked <c>// OCCT</c>): <see cref="CenterOn3"/> returns the first tangency point, not
/// the centre, as OCCT's does.
/// </remarks>
public sealed partial class GccAna_Circ2d2TanOn
{
    private bool WellDone;
    private int NbrSol;
    private readonly NCollection_Array1<gp_Circ2d> cirsol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<GccEnt_Position> qualifier2;
    private readonly NCollection_Array1<int> TheSame1;
    private readonly NCollection_Array1<int> TheSame2;
    private readonly NCollection_Array1<gp_Pnt2d> pnttg1sol;
    private readonly NCollection_Array1<gp_Pnt2d> pnttg2sol;
    private readonly NCollection_Array1<gp_Pnt2d> pntcen;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> par2sol;
    private readonly NCollection_Array1<double> pararg1;
    private readonly NCollection_Array1<double> pararg2;
    private readonly NCollection_Array1<double> parcen3;

    // The result arrays, sized as each constructor's initialiser list does in OCCT.
    private GccAna_Circ2d2TanOn(int theSize)
    {
        cirsol = new NCollection_Array1<gp_Circ2d>(1, theSize);
        qualifier1 = new NCollection_Array1<GccEnt_Position>(1, theSize);
        qualifier2 = new NCollection_Array1<GccEnt_Position>(1, theSize);
        TheSame1 = new NCollection_Array1<int>(1, theSize);
        TheSame2 = new NCollection_Array1<int>(1, theSize);
        pnttg1sol = new NCollection_Array1<gp_Pnt2d>(1, theSize);
        pnttg2sol = new NCollection_Array1<gp_Pnt2d>(1, theSize);
        pntcen = new NCollection_Array1<gp_Pnt2d>(1, theSize);
        par1sol = new NCollection_Array1<double>(1, theSize);
        par2sol = new NCollection_Array1<double>(1, theSize);
        pararg1 = new NCollection_Array1<double>(1, theSize);
        pararg2 = new NCollection_Array1<double>(1, theSize);
        parcen3 = new NCollection_Array1<double>(1, theSize);
    }

    //=========================================================================
    //  Circles tangent to two circles C1 and C2 and centered on a straight line.   +
    //  We start by distinguishing various boundary cases that will be processed separately. +
    //  In the general case:                                                  +
    //  ====================                                                  +
    //  We calculate bissectrices to two circles that give us     +
    //  all possible locations of centers of all circles tangent to C1 and C2. + We intersect these
    //  bissectrices with straight line which gives us  + points among which we are going to find
    //  solutions.   + The choices are made basing on Qualifiers of C1 and C2.  +
    //=========================================================================

    /// <summary>The circles tangent to two circles with their centre on a line.</summary>
    public GccAna_Circ2d2TanOn(in GccEnt_QualifiedCirc Qualified1,
                               in GccEnt_QualifiedCirc Qualified2,
                               in gp_Lin2d OnLine,
                               double Tolerance)
        : this(4)
    {
        TheSame1.Init(0);
        TheSame2.Init(0);
        WellDone = false;
        NbrSol = 0;
        int nbsol = 0;
        double Tol = Math.Abs(Tolerance);
        if (!(Qualified1.IsEnclosed() || Qualified1.IsEnclosing() || Qualified1.IsOutside()
              || Qualified1.IsUnqualified())
            || !(Qualified2.IsEnclosed() || Qualified2.IsEnclosing() || Qualified2.IsOutside()
                 || Qualified2.IsUnqualified()))
        {
            throw new GccEnt_BadQualifier();
        }
        gp_Circ2d C1 = Qualified1.Qualified();
        gp_Circ2d C2 = Qualified2.Qualified();
        double R1 = C1.Radius();
        double R2 = C2.Radius();
        gp_Dir2d dirx = new(gp_Dir2d.D.X);
        gp_Pnt2d center1 = C1.Location();
        gp_Pnt2d center2 = C2.Location();
        NCollection_Array1<double> Radius = new(1, 2);

        double dist1 = OnLine.Distance(center1);
        double dist2 = OnLine.Distance(center2);
        double d1 = dist1 + R1;
        double d2 = dist2 + R2;
        double d3 = dist1 - R1;
        double d4 = dist2 - R2;

        //=========================================================================
        //  Processing of boundary cases.                                           +
        //=========================================================================

        // OCCT: up to four of these cases can hold together, but Radius has two elements: OCCT's range check throws
        // Standard_OutOfRange past its end, as the port does (decision 5903).
        if (Math.Abs(d3 - d4) < Tol
            && (Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified())
            && (Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified()))
        {
            nbsol++;
            Radius[nbsol] = Math.Abs(d3);
            WellDone = true;
        }
        if (Math.Abs(d1 - d2) < Tol && (Qualified1.IsEnclosing() || Qualified1.IsUnqualified())
            && (Qualified2.IsEnclosing() || Qualified2.IsUnqualified()))
        {
            nbsol++;
            Radius[nbsol] = Math.Abs(d1);
            WellDone = true;
        }
        if (Math.Abs(d1 - d4) < Tol && (Qualified1.IsEnclosing() || Qualified1.IsUnqualified())
            && (Qualified2.IsEnclosed() || Qualified2.IsOutside() || Qualified2.IsUnqualified()))
        {
            nbsol++;
            Radius[nbsol] = Math.Abs(d1);
            WellDone = true;
        }
        if (Math.Abs(d3 - d2) < Tol && (Qualified2.IsEnclosing() || Qualified2.IsUnqualified())
            && (Qualified1.IsEnclosed() || Qualified1.IsOutside() || Qualified1.IsUnqualified()))
        {
            nbsol++;
            Radius[nbsol] = Math.Abs(d3);
            WellDone = true;
        }
        gp_Lin2d L = new(center1, new gp_Dir2d(center2.XY() - center1.XY()));
        IntAna2d_AnaIntersection Intp = new(OnLine, L);
        if (Intp.IsDone())
        {
            if (!Intp.IsEmpty())
            {
                for (int j = 1; j <= Intp.NbPoints(); j++)
                {
                    gp_Pnt2d Center = Intp.Point(j).Value();
                    for (int i = 1; i <= nbsol; i++)
                    {
                        NbrSol++;
                        cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius[i]);
                        //        ==========================================================
                        WellDone = true;
                        double distcc1 = Center.Distance(center1);
                        double distcc2 = Center.Distance(center2);
                        if (!Qualified1.IsUnqualified())
                        {
                            qualifier1[NbrSol] = Qualified1.Qualifier();
                        }
                        else if (Math.Abs(distcc1 + Radius[i] - R1) < Tol)
                        {
                            qualifier1[NbrSol] = GccEnt_enclosed;
                        }
                        else if (Math.Abs(distcc1 - R1 - Radius[i]) < Tol)
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
                        else if (Math.Abs(distcc2 + Radius[i] - R2) < Tol)
                        {
                            qualifier2[NbrSol] = GccEnt_enclosed;
                        }
                        else if (Math.Abs(distcc2 - R2 - Radius[i]) < Tol)
                        {
                            qualifier2[NbrSol] = GccEnt_outside;
                        }
                        else
                        {
                            qualifier2[NbrSol] = GccEnt_enclosing;
                        }
                        gp_Dir2d dc1 = new(center1.XY() - Center.XY());
                        gp_Dir2d dc2 = new(center2.XY() - Center.XY());
                        pnttg1sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius[i] * dc1.XY());
                        pnttg2sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius[i] * dc2.XY());
                        par1sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                        pararg1[NbrSol] = ElCLib.Parameter(C1, pnttg1sol[NbrSol]);
                        par2sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                        pararg2[NbrSol] = ElCLib.Parameter(C2, pnttg2sol[NbrSol]);
                        pntcen[NbrSol] = cirsol[NbrSol].Location();
                        parcen3[NbrSol] = ElCLib.Parameter(OnLine, pntcen[NbrSol]);
                    }
                }
            }
        }

        //=========================================================================
        //   General case.                                                       +
        //=========================================================================

        if (!WellDone)
        {
            GccAna_Circ2dBisec Bis = new(C1, C2);
            if (Bis.IsDone())
            {
                NCollection_Array1<double> Rbid = new(1, 2);
                NCollection_Array1<double> RBid = new(1, 2);
                int nbsolution = Bis.NbSolutions();
                for (int i = 1; i <= nbsolution; i++)
                {
                    GccInt_Bisec Sol = Bis.ThisSolution(i);
                    GccInt_IType typ = Sol.ArcType();

                    if (typ == GccInt_Cir)
                    {
                        Intp.Perform(OnLine, Sol.Circle());
                    }
                    else if (typ == GccInt_Lin)
                    {
                        Intp.Perform(OnLine, Sol.Line());
                    }
                    else if (typ == GccInt_Hpr)
                    {
                        Intp.Perform(OnLine, new IntAna2d_Conic(Sol.Hyperbola()));
                    }
                    else if (typ == GccInt_Ell)
                    {
                        Intp.Perform(OnLine, new IntAna2d_Conic(Sol.Ellipse()));
                    }
                    if (Intp.IsDone())
                    {
                        if (!Intp.IsEmpty())
                        {
                            for (int j = 1; j <= Intp.NbPoints(); j++)
                            {
                                gp_Pnt2d Center = Intp.Point(j).Value();
                                dist1 = Center.Distance(center1);
                                dist2 = Center.Distance(center2);
                                nbsol = 0;
                                int nsol = 0;
                                int nnsol = 0;
                                R1 = C1.Radius();
                                R2 = C2.Radius();
                                if (Qualified1.IsEnclosed())
                                {
                                    if (dist1 - R1 < Tol)
                                    {
                                        nbsol = 1;
                                        Rbid[1] = Math.Abs(R1 - dist1);
                                    }
                                }
                                else if (Qualified1.IsOutside())
                                {
                                    if (R1 - dist1 < Tol)
                                    {
                                        nbsol = 1;
                                        Rbid[1] = Math.Abs(dist1 - R1);
                                    }
                                }
                                else if (Qualified1.IsEnclosing())
                                {
                                    nbsol = 1;
                                    Rbid[1] = dist1 + R1;
                                }
                                else if (Qualified1.IsUnqualified())
                                {
                                    nbsol = 2;
                                    Rbid[1] = dist1 + R1;
                                    // OCCT: Rbid(1) is written twice and Rbid(2) never (it keeps the value of an earlier
                                    // point, or the array's initial value: uninitialised in C++, 0 in the port)
                                    Rbid[1] = Math.Abs(dist1 - R1);
                                }
                                if (Qualified2.IsEnclosed() && nbsol != 0)
                                {
                                    if (dist2 - R2 < Tol)
                                    {
                                        nsol = 1;
                                        RBid[1] = Math.Abs(R2 - dist2);
                                    }
                                }
                                else if (Qualified2.IsOutside() && nbsol != 0)
                                {
                                    if (R2 - dist2 < Tol)
                                    {
                                        nsol = 1;
                                        RBid[1] = Math.Abs(R2 - dist2);
                                    }
                                }
                                else if (Qualified2.IsEnclosing() && nbsol != 0)
                                {
                                    nsol = 1;
                                    RBid[1] = dist2 + R2;
                                }
                                else if (Qualified2.IsUnqualified() && nbsol != 0)
                                {
                                    nsol = 2;
                                    RBid[1] = dist2 + R2;
                                    RBid[2] = Math.Abs(R2 - dist2);
                                }
                                for (int isol = 1; isol <= nbsol; isol++)
                                {
                                    for (int jsol = 1; jsol <= nsol; jsol++)
                                    {
                                        if (Math.Abs(Rbid[isol] - RBid[jsol]) <= Tol)
                                        {
                                            nnsol++;
                                            // OCCT: up to four matches for two elements (OCCT and the port throw past the end)
                                            Radius[nnsol] = (RBid[jsol] + Rbid[isol]) / 2.0;
                                        }
                                    }
                                }
                                if (nnsol > 0)
                                {
                                    for (int k = 1; k <= nnsol; k++)
                                    {
                                        NbrSol++;
                                        cirsol[NbrSol] = new gp_Circ2d(new gp_Ax2d(Center, dirx), Radius[k]);
                                        //                ==========================================================
                                        double distcc1 = Center.Distance(center1);
                                        double distcc2 = Center.Distance(center2);
                                        // OCCT: the qualifiers of unqualified arguments are computed with Radius(i),
                                        // i the bisector's index, instead of Radius(k); past Radius's two elements
                                        // OCCT's range check throws Standard_OutOfRange, as the port does
                                        if (!Qualified1.IsUnqualified())
                                        {
                                            qualifier1[NbrSol] = Qualified1.Qualifier();
                                        }
                                        else if (Math.Abs(distcc1 + Radius[i] - R1) < Tol)
                                        {
                                            qualifier1[NbrSol] = GccEnt_enclosed;
                                        }
                                        else if (Math.Abs(distcc1 - R1 - Radius[i]) < Tol)
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
                                        else if (Math.Abs(distcc2 + Radius[i] - R2) < Tol)
                                        {
                                            qualifier2[NbrSol] = GccEnt_enclosed;
                                        }
                                        else if (Math.Abs(distcc2 - R2 - Radius[i]) < Tol)
                                        {
                                            qualifier2[NbrSol] = GccEnt_outside;
                                        }
                                        else
                                        {
                                            qualifier2[NbrSol] = GccEnt_enclosing;
                                        }
                                        if (Center.Distance(center1) <= Tol && Math.Abs(Radius[k] - C1.Radius()) <= Tol)
                                        {
                                            TheSame1[NbrSol] = 1;
                                        }
                                        else
                                        {
                                            TheSame1[NbrSol] = 0;
                                            gp_Dir2d dc1 = new(center1.XY() - Center.XY());
                                            pnttg1sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius[k] * dc1.XY());
                                            par1sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg1sol[NbrSol]);
                                            // OCCT: the parameter on C1 is that of the second tangency point
                                            pararg1[NbrSol] = ElCLib.Parameter(C1, pnttg2sol[NbrSol]);
                                        }
                                        if (Center.Distance(center2) <= Tol && Math.Abs(Radius[k] - C2.Radius()) <= Tol)
                                        {
                                            TheSame2[NbrSol] = 1;
                                        }
                                        else
                                        {
                                            TheSame2[NbrSol] = 0;
                                            gp_Dir2d dc2 = new(center2.XY() - Center.XY());
                                            pnttg2sol[NbrSol] = new gp_Pnt2d(Center.XY() + Radius[k] * dc2.XY());
                                            par2sol[NbrSol] = ElCLib.Parameter(cirsol[NbrSol], pnttg2sol[NbrSol]);
                                            pararg2[NbrSol] = ElCLib.Parameter(C2, pnttg2sol[NbrSol]);
                                        }
                                        pntcen[NbrSol] = Center;
                                        parcen3[NbrSol] = ElCLib.Parameter(OnLine, pntcen[NbrSol]);
                                    }
                                }
                            }
                        }
                        WellDone = true;
                    }
                }
            }
        }
    }

    //========================================================================

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

    /// <summary>
    /// The solution number Index. As in OCCT, IsDone and NbSolutions are not checked; an index outside the result
    /// arrays throws Standard_OutOfRange.
    /// </summary>
    public gp_Circ2d ThisSolution(int Index)
    {
        return cirsol[Index];
    }

    /// <summary>
    /// The qualifiers of the arguments for the solution number Index: the ones given, or the ones computed for
    /// unqualified arguments.
    /// </summary>
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

    /// <summary>
    /// The tangency point of the solution number Index with the first argument: its parameter on the solution, on the
    /// argument, and the point. Throws StdFail_NotDone if the solution is the argument itself (IsTheSame1).
    /// </summary>
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
            if (TheSame1[Index] == 0)
            {
                ParSol = par1sol[Index];
                ParArg = pararg1[Index];
                PntSol = pnttg1sol[Index];
            }
            else
            {
                throw new StdFail_NotDone();
            }
        }
    }

    /// <summary>
    /// The tangency point of the solution number Index with the second argument: its parameter on the solution, on the
    /// argument, and the point. Throws StdFail_NotDone if the solution is the argument itself (IsTheSame2).
    /// </summary>
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
            if (TheSame2[Index] == 0)
            {
                ParSol = par2sol[Index];
                ParArg = pararg2[Index];
                PntSol = pnttg2sol[Index];
            }
            else
            {
                throw new StdFail_NotDone();
            }
        }
    }

    /// <summary>
    /// The centre of the solution number Index on the third argument: its parameter there, and a point. As in OCCT,
    /// the point returned is the first tangency point, not the centre.
    /// </summary>
    public void CenterOn3(int Index, out double ParArg, out gp_Pnt2d PntSol)
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
            ParArg = parcen3[Index];
            PntSol = pnttg1sol[Index]; // OCCT: pnttg1sol instead of pntcen
        }
    }

    /// <summary>True if the solution number Index is the first argument.</summary>
    public bool IsTheSame1(int Index)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
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
            throw new StdFail_NotDone();
        }
        else if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }

        if (TheSame2[Index] == 0)
        {
            return false;
        }
        return true;
    }
}
