// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2d2Tan.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2d2Tan.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// 2D lines tangent to two qualified curves, or tangent to a qualified curve and through a point. Two circles (or a
/// circle and a point) go to GccAna_Lin2d2Tan; other arguments to Geom2dGcc_Lin2d2TanIter, from 21 sample parameters
/// on each curve or from given ones.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). Up to 4 solutions (2 with a point).</remarks>
public sealed class Geom2dGcc_Lin2d2Tan
{
    private readonly bool WellDone;
    private readonly int NbrSol;
    private readonly NCollection_Array1<gp_Lin2d> linsol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<GccEnt_Position> qualifier2;
    private readonly NCollection_Array1<gp_Pnt2d> pnttg1sol;
    private readonly NCollection_Array1<gp_Pnt2d> pnttg2sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> par2sol;
    private readonly NCollection_Array1<double> pararg1;
    private readonly NCollection_Array1<double> pararg2;

    // The fields' initialiser list: arrays of nbSol elements.
    private Geom2dGcc_Lin2d2Tan(int nbSol)
    {
        linsol     = new(1, nbSol);
        qualifier1 = new(1, nbSol);
        qualifier2 = new(1, nbSol);
        pnttg1sol  = new(1, nbSol);
        pnttg2sol  = new(1, nbSol);
        par1sol    = new(1, nbSol);
        par2sol    = new(1, nbSol);
        pararg1    = new(1, nbSol);
        pararg2    = new(1, nbSol);
    }

    /// <summary>Lines tangent to two qualified curves. Tolang is the angular tolerance.</summary>
    public Geom2dGcc_Lin2d2Tan(in Geom2dGcc_QualifiedCurve Qualified1, in Geom2dGcc_QualifiedCurve Qualified2, double Tolang)
        : this(4)
    {
        Geom2dAdaptor_Curve C1    = Qualified1.Qualified();
        Geom2dAdaptor_Curve C2    = Qualified2.Qualified();
        Geom2d_Curve?       CC1   = C1.Curve();
        Geom2d_Curve?       CC2   = C2.Curve();
        GeomAbs_CurveType   Type1 = C1.GetType();
        GeomAbs_CurveType   Type2 = C2.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        NbrSol = 0;
        if (Type1 == GeomAbs_Circle && Type2 == GeomAbs_Circle)
        {
            Geom2d_Circle        CCC1 = (CC1 as Geom2d_Circle)!;
            gp_Circ2d            c1   = CCC1.Circ2d();
            Geom2d_Circle        CCC2 = (CC2 as Geom2d_Circle)!;
            gp_Circ2d            c2   = CCC2.Circ2d();
            GccEnt_QualifiedCirc Qc1  = new(c1, Qualified1.Qualifier());
            GccEnt_QualifiedCirc Qc2  = new(c2, Qualified2.Qualifier());
            GccAna_Lin2d2Tan     Lin  = new(Qc1, Qc2, Tolang);
            WellDone                  = Lin.IsDone();
            if (WellDone)
            {
                NbrSol = Lin.NbSolutions();
                for (int i = 1; i <= NbrSol; i++)
                {
                    linsol[i] = Lin.ThisSolution(i);
                    Lin.Tangency1(i, out par1sol[i], out pararg1[i], out pnttg1sol[i]);
                    Lin.Tangency2(i, out par2sol[i], out pararg2[i], out pnttg2sol[i]);
                    Lin.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                }
            }
        }
        else
        {
            Geom2dGcc_QCurve Qc1         = new(C1, Qualified1.Qualifier());
            double           a1FPar      = Geom2dGcc_CurveTool.FirstParameter(C1);
            double           a1LPar      = Geom2dGcc_CurveTool.LastParameter(C1);
            int              aNbSamples1 = Geom2dGcc_CurveTool.NbSamples(C1);
            double           aStep1      = (a1LPar - a1FPar) / aNbSamples1;
            double           Param1      = a1FPar;
            Geom2dGcc_QCurve Qc2         = new(C2, Qualified2.Qualifier());
            double           a2FPar      = Geom2dGcc_CurveTool.FirstParameter(C2);
            double           a2LPar      = Geom2dGcc_CurveTool.LastParameter(C2);
            int              aNbSamples2 = Geom2dGcc_CurveTool.NbSamples(C2);
            double           aStep2      = (a2LPar - a2FPar) / aNbSamples2;
            double           Param2;
            int              i;
            int              j;

            for (i = 0; i <= aNbSamples1 && NbrSol < 4; i++)
            {
                Param2 = a2FPar;

                for (j = 0; j <= aNbSamples2 && NbrSol < 4; j++)
                {
                    Geom2dGcc_Lin2d2TanIter Lin = new(Qc1, Qc2, Param1, Param2, Tolang);

                    if (Lin.IsDone())
                    {
                        if (Add(NbrSol + 1, Lin, Tolang, C1, C2))
                        {
                            NbrSol++;
                        }
                    }
                    Param2 += aStep2;
                }
                Param1 += aStep1;
            }

            WellDone = (NbrSol > 0);
        }
    }

    /// <summary>Lines tangent to a qualified curve through a point. Tolang is the angular tolerance.</summary>
    public Geom2dGcc_Lin2d2Tan(in Geom2dGcc_QualifiedCurve Qualified1, in gp_Pnt2d ThePoint, double Tolang)
        : this(2)
    {
        Geom2dAdaptor_Curve C1    = Qualified1.Qualified();
        Geom2d_Curve?       CC1   = C1.Curve();
        GeomAbs_CurveType   Type1 = C1.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        NbrSol = 0;
        if (Type1 == GeomAbs_Circle)
        {
            Geom2d_Circle        CCC1 = (CC1 as Geom2d_Circle)!;
            gp_Circ2d            c1   = CCC1.Circ2d();
            GccEnt_QualifiedCirc Qc1  = new(c1, Qualified1.Qualifier());
            GccAna_Lin2d2Tan     Lin  = new(Qc1, ThePoint, Tolang);
            WellDone                  = Lin.IsDone();

            if (WellDone)
            {
                NbrSol = Lin.NbSolutions();
                for (int i = 1; i <= NbrSol; i++)
                {
                    linsol[i] = Lin.ThisSolution(i);
                    Lin.Tangency1(i, out par1sol[i], out pararg1[i], out pnttg1sol[i]);
                    Lin.Tangency2(i, out par2sol[i], out pararg2[i], out pnttg2sol[i]);
                    Lin.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
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
                Geom2dGcc_Lin2d2TanIter Lin = new(Qc1, ThePoint, Param1, Tolang);

                if (Lin.IsDone())
                {
                    if (Add(NbrSol + 1, Lin, Tolang, C1, new Geom2dAdaptor_Curve()))
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
    /// The line tangent to two qualified curves found from the parameters Param1 and Param2 on them (two circles still
    /// go to GccAna).
    /// </summary>
    public Geom2dGcc_Lin2d2Tan(in Geom2dGcc_QualifiedCurve Qualified1, in Geom2dGcc_QualifiedCurve Qualified2, double Tolang,
                               double Param1, double Param2)
        : this(4)
    {
        Geom2dAdaptor_Curve C1    = Qualified1.Qualified();
        Geom2dAdaptor_Curve C2    = Qualified2.Qualified();
        Geom2d_Curve?       CC1   = C1.Curve();
        Geom2d_Curve?       CC2   = C2.Curve();
        GeomAbs_CurveType   Type1 = C1.GetType();
        GeomAbs_CurveType   Type2 = C2.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        NbrSol = 0;
        if (Type1 == GeomAbs_Circle && Type2 == GeomAbs_Circle)
        {
            Geom2d_Circle        CCC1 = (CC1 as Geom2d_Circle)!;
            gp_Circ2d            c1   = CCC1.Circ2d();
            Geom2d_Circle        CCC2 = (CC2 as Geom2d_Circle)!;
            gp_Circ2d            c2   = CCC2.Circ2d();
            GccEnt_QualifiedCirc Qc1  = new(c1, Qualified1.Qualifier());
            GccEnt_QualifiedCirc Qc2  = new(c2, Qualified2.Qualifier());
            GccAna_Lin2d2Tan     Lin  = new(Qc1, Qc2, Tolang);
            WellDone                  = Lin.IsDone();
            if (WellDone)
            {
                NbrSol = Lin.NbSolutions();
                for (int i = 1; i <= NbrSol; i++)
                {
                    linsol[i] = Lin.ThisSolution(i);
                    Lin.Tangency1(i, out par1sol[i], out pararg1[i], out pnttg1sol[i]);
                    Lin.Tangency2(i, out par2sol[i], out pararg2[i], out pnttg2sol[i]);
                    Lin.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                }
            }
        }
        else
        {
            Geom2dGcc_QCurve        Qc1 = new(C1, Qualified1.Qualifier());
            Geom2dGcc_QCurve        Qc2 = new(C2, Qualified2.Qualifier());
            Geom2dGcc_Lin2d2TanIter Lin = new(Qc1, Qc2, Param1, Param2, Tolang);
            WellDone                    = Lin.IsDone();
            if (WellDone)
            {
                NbrSol    = 1;
                linsol[1] = Lin.ThisSolution();
                Lin.Tangency1(out par1sol[1], out pararg1[1], out pnttg1sol[1]);
                Lin.Tangency2(out par2sol[1], out pararg2[1], out pnttg2sol[1]);
                Lin.WhichQualifier(out qualifier1[1], out qualifier2[1]);
            }
        }
    }

    /// <summary>
    /// The line tangent to a qualified curve through a point found from the parameter Param1 on the curve (a circle
    /// still goes to GccAna).
    /// </summary>
    public Geom2dGcc_Lin2d2Tan(in Geom2dGcc_QualifiedCurve Qualified1, in gp_Pnt2d ThePoint, double Tolang, double Param1)
        : this(2)
    {
        Geom2dAdaptor_Curve C1    = Qualified1.Qualified();
        Geom2d_Curve?       CC1   = C1.Curve();
        GeomAbs_CurveType   Type1 = C1.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        NbrSol = 0;
        if (Type1 == GeomAbs_Circle)
        {
            Geom2d_Circle        CCC1 = (CC1 as Geom2d_Circle)!;
            gp_Circ2d            c1   = CCC1.Circ2d();
            GccEnt_QualifiedCirc Qc1  = new(c1, Qualified1.Qualifier());
            GccAna_Lin2d2Tan     Lin  = new(Qc1, ThePoint, Tolang);
            WellDone                  = Lin.IsDone();
            if (WellDone)
            {
                NbrSol = Lin.NbSolutions();
                for (int i = 1; i <= NbrSol; i++)
                {
                    linsol[i] = Lin.ThisSolution(i);
                    Lin.Tangency1(i, out par1sol[i], out pararg1[i], out pnttg1sol[i]);
                    Lin.Tangency2(i, out par2sol[i], out pararg2[i], out pnttg2sol[i]);
                    Lin.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                }
            }
        }
        else
        {
            Geom2dGcc_QCurve        Qc1 = new(C1, Qualified1.Qualifier());
            Geom2dGcc_Lin2d2TanIter Lin = new(Qc1, ThePoint, Param1, Tolang);
            WellDone                    = Lin.IsDone();
            if (WellDone)
            {
                NbrSol    = 1;
                linsol[1] = Lin.ThisSolution();
                Lin.Tangency1(out par1sol[1], out pararg1[1], out pnttg1sol[1]);
                Lin.Tangency2(out par2sol[1], out pararg2[1], out pnttg2sol[1]);
                Lin.WhichQualifier(out qualifier1[1], out qualifier2[1]);
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

    /// <summary>The qualifiers of the arguments for the solution of index Index.</summary>
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

    /// <summary>The tangency point between the solution of index Index and the first argument, and its parameters.</summary>
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

    /// <summary>The tangency point between the solution of index Index and the second argument, and its parameters.</summary>
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

    private bool Add(int theIndex, Geom2dGcc_Lin2d2TanIter theLin, double theTol, Geom2dAdaptor_Curve theC1,
                     Geom2dAdaptor_Curve theC2)
    {
        int      i;
        gp_Lin2d aLin = theLin.ThisSolution();

        theLin.Tangency1(out double aPar1sol, out double aPar1arg, out gp_Pnt2d aPnt1Sol);
        theLin.Tangency2(out double aPar2sol, out double aPar2arg, out gp_Pnt2d aPnt2Sol);

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

        if (theC2.Curve() is not null)
        {
            Geom2dGcc_CurveTool.D1(theC2, aPar2arg, out _, out aVTan);

            if (Math.Abs(aLinDir.Crossed(new gp_Dir2d(aVTan))) > theTol)
            {
                return false;
            }
        }

        linsol[theIndex]    = aLin;
        par1sol[theIndex]   = aPar1sol;
        pararg1[theIndex]   = aPar1arg;
        pnttg1sol[theIndex] = aPnt1Sol;
        par2sol[theIndex]   = aPar2sol;
        pararg2[theIndex]   = aPar2arg;
        pnttg2sol[theIndex] = aPnt2Sol;

        theLin.WhichQualifier(out qualifier1[theIndex], out qualifier2[theIndex]);

        return true;
    }
}
