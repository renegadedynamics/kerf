// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2dTanObl.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2dTanObl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// 2D lines tangent to a qualified curve and making a given angle with a line. A circle goes to GccAna_Lin2dTanObl;
/// other curves (lines included) to Geom2dGcc_Lin2dTanOblIter, from 21 sample parameters or from a given one.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). Up to 2 solutions. OCCT declares IsParallel2
/// without defining it (V8_0_1); it is left out.
/// </remarks>
public sealed class Geom2dGcc_Lin2dTanObl
{
    private readonly bool WellDone;
    // OCCT's Paral2 (always false, read only by the undefined IsParallel2) is left out.
    private readonly int NbrSol;
    private readonly NCollection_Array1<gp_Lin2d> linsol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<gp_Pnt2d> pnttg1sol;
    private readonly NCollection_Array1<gp_Pnt2d> pntint2sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> par2sol;
    private readonly NCollection_Array1<double> pararg1;
    private readonly NCollection_Array1<double> pararg2;

    /// <summary>
    /// Lines tangent to the qualified curve at an angle Angle (radians) to TheLin; TolAng is the angular tolerance.
    /// </summary>
    /// <exception cref="Geom2dGcc_IsParallel">The angle is 0 or pi (within TolAng) and the curve isn't a circle.</exception>
    public Geom2dGcc_Lin2dTanObl(in Geom2dGcc_QualifiedCurve Qualified1, in gp_Lin2d TheLine, double TolAng, double Angle)
    {
        linsol     = new(1, 2);
        qualifier1 = new(1, 2);
        pnttg1sol  = new(1, 2);
        pntint2sol = new(1, 2);
        par1sol    = new(1, 2);
        par2sol    = new(1, 2);
        pararg1    = new(1, 2);
        pararg2    = new(1, 2);

        Geom2dAdaptor_Curve C1    = Qualified1.Qualified();
        Geom2d_Curve?       CC1   = C1.Curve();
        GeomAbs_CurveType   Type1 = C1.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        WellDone = false;
        NbrSol   = 0;
        if (Type1 == GeomAbs_Circle)
        {
            Geom2d_Circle        CCC1 = (CC1 as Geom2d_Circle)!;
            gp_Circ2d            c1   = CCC1.Circ2d();
            GccEnt_QualifiedCirc Qc1  = new(c1, Qualified1.Qualifier());
            GccAna_Lin2dTanObl   Lin  = new(Qc1, TheLine, Angle);
            WellDone                  = Lin.IsDone();
            if (WellDone)
            {
                NbrSol = Lin.NbSolutions();
                for (int i = 1; i <= NbrSol; i++)
                {
                    linsol[i] = Lin.ThisSolution(i);
                    Lin.Tangency1(i, out par1sol[i], out pararg1[i], out pnttg1sol[i]);
                    Lin.Intersection2(i, out par2sol[i], out pararg2[i], out pntint2sol[i]);
                    Lin.WhichQualifier(i, out qualifier1[i]);
                }
            }
        }
        else
        {
            Geom2dGcc_QCurve Qc1        = new(C1, Qualified1.Qualifier());
            double           aFirstPar  = Geom2dGcc_CurveTool.FirstParameter(C1);
            double           aLastPar   = Geom2dGcc_CurveTool.LastParameter(C1);
            int              aNbSamples = Geom2dGcc_CurveTool.NbSamples(C1);
            double           aStep      = (aLastPar - aFirstPar) / aNbSamples;
            double           Param1     = aFirstPar;
            int              i;

            for (i = 0; i <= aNbSamples && NbrSol < 2; i++)
            {
                Geom2dGcc_Lin2dTanOblIter Lin = new(Qc1, TheLine, Param1, TolAng, Angle);

                if (Lin.IsDone())
                {
                    if (Add(NbrSol + 1, Lin, TolAng, C1))
                    {
                        NbrSol++;
                    }
                }

                Param1 += aStep;
            }

            WellDone = (NbrSol > 0);
        }
    }

    /// <summary>
    /// The line tangent to the qualified curve at an angle Angle to TheLin, found from the parameter Param1 on the
    /// curve (a circle still goes to GccAna).
    /// </summary>
    public Geom2dGcc_Lin2dTanObl(in Geom2dGcc_QualifiedCurve Qualified1, in gp_Lin2d TheLine, double TolAng, double Param1,
                                 double Angle)
    {
        linsol     = new(1, 2);
        qualifier1 = new(1, 2);
        pnttg1sol  = new(1, 2);
        pntint2sol = new(1, 2);
        par1sol    = new(1, 2);
        par2sol    = new(1, 2);
        pararg1    = new(1, 2);
        pararg2    = new(1, 2);

        Geom2dAdaptor_Curve C1    = Qualified1.Qualified();
        Geom2d_Curve?       CC1   = C1.Curve();
        GeomAbs_CurveType   Type1 = C1.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        WellDone = false;
        NbrSol   = 0;
        if (Type1 == GeomAbs_Circle)
        {
            Geom2d_Circle        CCC1 = (CC1 as Geom2d_Circle)!;
            gp_Circ2d            c1   = CCC1.Circ2d();
            GccEnt_QualifiedCirc Qc1  = new(c1, Qualified1.Qualifier());
            GccAna_Lin2dTanObl   Lin  = new(Qc1, TheLine, Angle);
            WellDone                  = Lin.IsDone();
            if (WellDone)
            {
                NbrSol = Lin.NbSolutions();
                for (int i = 1; i <= NbrSol; i++)
                {
                    linsol[i] = Lin.ThisSolution(i);
                    Lin.Tangency1(i, out par1sol[i], out pararg1[i], out pnttg1sol[i]);
                    Lin.Intersection2(i, out par2sol[i], out pararg2[i], out pntint2sol[i]);
                    Lin.WhichQualifier(i, out qualifier1[i]);
                }
            }
        }
        else
        {
            Geom2dGcc_QCurve Qc1 = new(C1, Qualified1.Qualifier());
            // OCCT: passes TolAng as the start parameter and Param1 as the angular tolerance (the Iter constructor
            // takes Param1 before TolAng)
            Geom2dGcc_Lin2dTanOblIter Lin = new(Qc1, TheLine, TolAng, Param1, Angle);
            WellDone                      = Lin.IsDone();
            if (WellDone)
            {
                linsol[1] = Lin.ThisSolution();
                Lin.Tangency1(out par1sol[1], out pararg1[1], out pnttg1sol[1]);
                Lin.Intersection2(out par2sol[1], out pararg2[1], out pntint2sol[1]);
                Lin.WhichQualifier(out qualifier1[1]);
            }
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
        return NbrSol;
    }

    /// <summary>The solution of index Index.</summary>
    /// <exception cref="Standard_OutOfRange">Index is not in 1..NbSolutions().</exception>
    public gp_Lin2d ThisSolution(int Index)
    {
        if (Index > NbrSol || Index <= 0)
        {
            throw new Standard_OutOfRange();
        }
        return linsol[Index];
    }

    /// <summary>The qualifier of the curve for the solution of index Index.</summary>
    public void WhichQualifier(int Index, out GccEnt_Position Qualif1)
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
        }
    }

    /// <summary>The tangency point between the solution of index Index and the curve, and its parameters.</summary>
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

    /// <summary>The intersection point between the solution of index Index and the line, and its parameters.</summary>
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

    // IsParallel2 is declared in OCCT V8_0_1's header but never defined: left out.

    private bool Add(int theIndex, Geom2dGcc_Lin2dTanOblIter theLin, double theTol, Geom2dAdaptor_Curve theC1)
    {
        int      i;
        gp_Lin2d aLin = theLin.ThisSolution();

        theLin.Tangency1(out double aPar1sol, out double aPar1arg, out gp_Pnt2d aPnt1Sol);
        // Throws Geom2dGcc_IsParallel when the angle is 0 or pi
        theLin.Intersection2(out double aPar2sol, out double aPar2arg, out gp_Pnt2d aPnt2Sol);

        for (i = 1; i < theIndex; i++)
        {
            if (Math.Abs(aPar1arg - pararg1[i]) <= theTol && Math.Abs(aPar2arg - pararg2[i]) <= theTol)
            {
                return false;
            }
        }

        gp_Dir2d aLinDir = aLin.Direction();

        Geom2dGcc_CurveTool.D1(theC1, aPar1arg, out _, out gp_Vec2d aVTan);

        if (Math.Abs(aLinDir.Crossed(new gp_Dir2d(aVTan))) > theTol)
        {
            return false;
        }

        linsol[theIndex]     = aLin;
        par1sol[theIndex]    = aPar1sol;
        pararg1[theIndex]    = aPar1arg;
        pnttg1sol[theIndex]  = aPnt1Sol;
        par2sol[theIndex]    = aPar2sol;
        pararg2[theIndex]    = aPar2arg;
        pntint2sol[theIndex] = aPnt2Sol;

        theLin.WhichQualifier(out qualifier1[theIndex]);

        return true;
    }
}
