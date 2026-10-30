// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dBisec.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dBisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The bisecting lines of two 2D lines. Secant lines have two: the inside one, along D1 + D2, and the outside one,
/// along D1 - D2 (D1 and D2 the lines' directions). Parallel lines have one, equidistant from both; coincident lines
/// give the first line.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117).</remarks>
public sealed class GccAna_Lin2dBisec
{
    private bool WellDone;
    private int NbrSol;
    private readonly NCollection_Array1<gp_Lin2d> linsol;
    private readonly NCollection_Array1<gp_Pnt2d> pntint1sol;
    private readonly NCollection_Array1<gp_Pnt2d> pntint2sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> par2sol;
    private readonly NCollection_Array1<double> pararg1;
    private readonly NCollection_Array1<double> pararg2;

    //=========================================================================
    //   The first calculated bissectrice is the interior bisectrice, the     +
    //   second is the exterior bissectrice.                                  +
    //   the direction of the first bissectrice is such that its scalar product +
    //   with direction of Lin1 is always positive.             +
    //   The second bissectrice is turned in the positive direction.             +
    //=========================================================================

    /// <summary>Constructs the bisecting lines of Lin1 and Lin2.</summary>
    public GccAna_Lin2dBisec(in gp_Lin2d Lin1, in gp_Lin2d Lin2)
    {
        linsol = new NCollection_Array1<gp_Lin2d>(1, 2);
        pntint1sol = new NCollection_Array1<gp_Pnt2d>(1, 2);
        pntint2sol = new NCollection_Array1<gp_Pnt2d>(1, 2);
        par1sol = new NCollection_Array1<double>(1, 2);
        par2sol = new NCollection_Array1<double>(1, 2);
        pararg1 = new NCollection_Array1<double>(1, 2);
        pararg2 = new NCollection_Array1<double>(1, 2);

        WellDone = false;
        NbrSol = 0;

        IntAna2d_AnaIntersection Intp = new(Lin1, Lin2);
        if (Intp.IsDone())
        {
            if (Intp.ParallelElements())
            {
                if (Intp.IdenticalElements())
                {
                    NbrSol = 1;
                    WellDone = true;
                    linsol[NbrSol] = Lin1;
                }
                else
                {
                    // Attention : do not use dist = Lin1.Distance(Lin2);
                    // as straight lines can be concurrent for gp_Lin2d
                    // so dist = 0.0 (test of the angle too strict ?)
                    double dist = Lin1.Distance(Lin2.Location()) / 2.0;
                    double cross = new gp_Vec2d(-Lin2.Direction().Y(), Lin2.Direction().X())
                                       .Dot(new gp_Vec2d(Lin2.Location(), Lin1.Location()));
                    if (cross < 0)
                    {
                        dist = -dist;
                    }
                    NbrSol++;
                    WellDone = true;
                    linsol[NbrSol] =
                        new gp_Lin2d(new gp_Pnt2d(Lin2.Location().XY()
                                                  + new gp_XY(-Lin2.Direction().Y() * dist, Lin2.Direction().X() * dist)),
                                     Lin2.Direction());
                }
            }
            else
            {
                if (!Intp.IsEmpty())
                {
                    for (int i = 1; i <= Intp.NbPoints(); i++)
                    {
                        NbrSol++;
                        linsol[NbrSol] = new gp_Lin2d(Intp.Point(i).Value(),
                                                      new gp_Dir2d(Lin1.Direction().XY() + Lin2.Direction().XY()));
                        NbrSol++;
                        linsol[NbrSol] = new gp_Lin2d(Intp.Point(i).Value(),
                                                      new gp_Dir2d(Lin1.Direction().XY() - Lin2.Direction().XY()));
                        if (Lin1.Angle(Lin2) >= 0.0)
                        {
                            linsol[NbrSol].Reverse();
                        }
                        WellDone = true;
                    }
                }
            }
        }

        for (int i = 1; i <= NbrSol; i++)
        {
            pntint1sol[i] = linsol[i].Location();
            pntint2sol[i] = pntint1sol[i];
            par1sol[i] = ElCLib.Parameter(linsol[i], pntint1sol[i]);
            par2sol[i] = ElCLib.Parameter(linsol[i], pntint2sol[i]);
            pararg1[i] = ElCLib.Parameter(Lin1, pntint1sol[i]);
            pararg2[i] = ElCLib.Parameter(Lin2, pntint2sol[i]);
        }
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The number of bisecting lines.</summary>
    public int NbSolutions()
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        return NbrSol;
    }

    /// <summary>
    /// The solution number Index: the first is the inside bisector (along D1 + D2), the second the outside one (along
    /// D1 - D2).
    /// </summary>
    public gp_Lin2d ThisSolution(int Index)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }
        return linsol[Index];
    }

    /// <summary>The intersection of the solution number Index with the first argument.</summary>
    public void Intersection1(int Index, out double ParSol, out double ParArg, out gp_Pnt2d PntSol)
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
            PntSol = pntint1sol[Index];
        }
    }

    /// <summary>The intersection of the solution number Index with the second argument.</summary>
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
