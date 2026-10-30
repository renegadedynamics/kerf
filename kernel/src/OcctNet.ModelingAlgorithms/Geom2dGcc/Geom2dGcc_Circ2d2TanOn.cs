// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanOn.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanOn.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// 2D circles tangent to two arguments (qualified curves or points) whose centres lie on a curve. Each solution comes
/// with the qualifiers of its arguments, its tangency points and its centre on the curve.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). Lines, circles and points are solved by GccAna_Circ2d2TanOn
/// (OnCurve a line or circle) or Geom2dGcc_Circ2d2TanOnGeo (OnCurve free-form); other arguments by
/// Geom2dGcc_Circ2d2TanOnIter, which gives at most one solution near the given parameters (and, as in OCCT, no centre
/// parameter: CenterOn3 then returns 0 and the origin).
/// </remarks>
public sealed class Geom2dGcc_Circ2d2TanOn
{
    private bool WellDone;
    private readonly NCollection_Array1<gp_Circ2d> cirsol;
    private int NbrSol;
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
    private bool Invert;

    /// <summary>
    /// Circles tangent to two qualified curves whose centres lie on OnCurve. Lines and circles go to GccAna (all
    /// solutions) or Geom2dGcc_Circ2d2TanOnGeo (a free-form OnCurve); other curves to Geom2dGcc_Circ2d2TanOnIter (one
    /// solution, from the parameters Param1, Param2 and ParamOn on the arguments and OnCurve).
    /// </summary>
    public Geom2dGcc_Circ2d2TanOn(in Geom2dGcc_QualifiedCurve Qualified1, in Geom2dGcc_QualifiedCurve Qualified2, Geom2dAdaptor_Curve OnCurve, double Tolerance, double Param1, double Param2, double ParamOn)
    {
        cirsol     = new(1, 8);
        qualifier1 = new(1, 8);
        qualifier2 = new(1, 8);
        TheSame1   = new(1, 8);
        TheSame2   = new(1, 8);
        pnttg1sol  = new(1, 8);
        pnttg2sol  = new(1, 8);
        pntcen     = new(1, 8);
        par1sol    = new(1, 8);
        par2sol    = new(1, 8);
        pararg1    = new(1, 8);
        pararg2    = new(1, 8);
        parcen3    = new(1, 8);
        WellDone   = false;
        Geom2dAdaptor_Curve              C1    = Qualified1.Qualified();
        Geom2dAdaptor_Curve              C2    = Qualified2.Qualified();
        GeomAbs_CurveType                Type1 = C1.GetType();
        GeomAbs_CurveType                Type2 = C2.GetType();
        GeomAbs_CurveType                Type3 = OnCurve.GetType();
        Geom2d_Curve?  CC1   = C1.Curve();
        Geom2d_Curve?  CC2   = C2.Curve();
        Geom2d_Curve?  Con   = OnCurve.Curve();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        Invert = false;
        NbrSol = 0;
        if ((Type1 == GeomAbs_Line || Type1 == GeomAbs_Circle)
                && (Type2 == GeomAbs_Line || Type2 == GeomAbs_Circle))
        {
            if (Type3 == GeomAbs_Line || Type3 == GeomAbs_Circle)
            {
                if (Type1 == GeomAbs_Circle)
                {
                    Geom2d_Circle CCC1 = (CC1 as Geom2d_Circle)!;
                    gp_Circ2d                  c1 = CCC1.Circ2d();
                    GccEnt_QualifiedCirc       Qc1 = new GccEnt_QualifiedCirc(c1, Qualified1.Qualifier());
                    if (Type2 == GeomAbs_Circle)
                    {
                        Geom2d_Circle CCC2 = (CC2 as Geom2d_Circle)!;
                        gp_Circ2d                  c2 = CCC2.Circ2d();
                        if (Type3 == GeomAbs_Circle)
                        {
                            Geom2d_Circle CCon = (Con as Geom2d_Circle)!;
                            GccAna_Circ2d2TanOn        CircAna = new(Qc1,
                                                                                    new GccEnt_QualifiedCirc(c2, Qualified2.Qualifier()),
                                                                                    CCon.Circ2d(),
                                                                                    Tolerance);
                            WellDone = CircAna.IsDone();
                            if (WellDone)
                            {
                                NbrSol = CircAna.NbSolutions();
                                for (int i = 1; i <= NbrSol; i++)
                                {
                                    CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                                }
                                Results(CircAna);
                            }
                        }
                        else
                        {
                            Geom2d_Line LLon = (Con as Geom2d_Line)!;
                            GccAna_Circ2d2TanOn      CircAna = new(Qc1,
                                                                                    new GccEnt_QualifiedCirc(c2, Qualified2.Qualifier()),
                                                                                    LLon.Lin2d(),
                                                                                    Tolerance);
                            WellDone = CircAna.IsDone();
                            if (WellDone)
                            {
                                NbrSol = CircAna.NbSolutions();
                                for (int i = 1; i <= NbrSol; i++)
                                {
                                    CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                                }
                                Results(CircAna);
                            }
                        }
                    }
                    else
                    {
                        Geom2d_Line LL2 = (CC2 as Geom2d_Line)!;
                        gp_Lin2d                 l2 = LL2.Lin2d();
                        if (Type3 == GeomAbs_Circle)
                        {
                            Geom2d_Circle CCon = (Con as Geom2d_Circle)!;
                            GccAna_Circ2d2TanOn        CircAna = new(Qc1,
                                                                                    new GccEnt_QualifiedLin(l2, Qualified2.Qualifier()),
                                                                                    CCon.Circ2d(),
                                                                                    Tolerance);
                            WellDone = CircAna.IsDone();
                            if (WellDone)
                            {
                                NbrSol = CircAna.NbSolutions();
                                for (int i = 1; i <= NbrSol; i++)
                                {
                                    CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                                }
                                Results(CircAna);
                            }
                        }
                        else
                        {
                            Geom2d_Line LLon = (Con as Geom2d_Line)!;
                            GccAna_Circ2d2TanOn      CircAna = new(Qc1,
                                                                                    new GccEnt_QualifiedLin(l2, Qualified2.Qualifier()),
                                                                                    LLon.Lin2d(),
                                                                                    Tolerance);
                            WellDone = CircAna.IsDone();
                            if (WellDone)
                            {
                                NbrSol = CircAna.NbSolutions();
                                for (int i = 1; i <= NbrSol; i++)
                                {
                                    CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                                }
                                Results(CircAna);
                            }
                        }
                    }
                }
                else
                {
                    Geom2d_Line LL1 = (CC1 as Geom2d_Line)!;
                    gp_Lin2d                 l1 = LL1.Lin2d();
                    GccEnt_QualifiedLin      Ql1 = new GccEnt_QualifiedLin(l1, Qualified1.Qualifier());
                    if (Type2 == GeomAbs_Circle)
                    {
                        Geom2d_Circle CCC2 = (CC2 as Geom2d_Circle)!;
                        gp_Circ2d                  c2 = CCC2.Circ2d();
                        if (Type3 == GeomAbs_Circle)
                        {
                            Geom2d_Circle CCon = (Con as Geom2d_Circle)!;
                            GccAna_Circ2d2TanOn        CircAna = new(new GccEnt_QualifiedCirc(c2, Qualified2.Qualifier()),
                                                                                    Ql1,
                                                                                    CCon.Circ2d(),
                                                                                    Tolerance);
                            WellDone = CircAna.IsDone();
                            if (WellDone)
                            {
                                NbrSol = CircAna.NbSolutions();
                                for (int i = 1; i <= NbrSol; i++)
                                {
                                    CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                                }
                                Results(CircAna);
                                Invert = true;
                            }
                        }
                        else
                        {
                            Geom2d_Line LLon = (Con as Geom2d_Line)!;
                            GccAna_Circ2d2TanOn      CircAna = new(new GccEnt_QualifiedCirc(c2, Qualified2.Qualifier()),
                                                                                    Ql1,
                                                                                    LLon.Lin2d(),
                                                                                    Tolerance);
                            WellDone = CircAna.IsDone();
                            if (WellDone)
                            {
                                NbrSol = CircAna.NbSolutions();
                                for (int i = 1; i <= NbrSol; i++)
                                {
                                    CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                                }
                                Results(CircAna);
                                Invert = true;
                            }
                        }
                    }
                    else
                    {
                        Geom2d_Line LL2 = (CC2 as Geom2d_Line)!;
                        gp_Lin2d                 l2 = LL2.Lin2d();
                        if (Type3 == GeomAbs_Circle)
                        {
                            Geom2d_Circle CCon = (Con as Geom2d_Circle)!;
                            GccAna_Circ2d2TanOn        CircAna = new(Ql1,
                                                                                    new GccEnt_QualifiedLin(l2, Qualified2.Qualifier()),
                                                                                    CCon.Circ2d(),
                                                                                    Tolerance);
                            WellDone = CircAna.IsDone();
                            if (WellDone)
                            {
                                NbrSol = CircAna.NbSolutions();
                                for (int i = 1; i <= NbrSol; i++)
                                {
                                    CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                                }
                                Results(CircAna);
                            }
                        }
                        else
                        {
                            Geom2d_Line LLon = (Con as Geom2d_Line)!;
                            GccAna_Circ2d2TanOn      CircAna = new(Ql1,
                                                                                    new GccEnt_QualifiedLin(l2, Qualified2.Qualifier()),
                                                                                    LLon.Lin2d(),
                                                                                    Tolerance);
                            WellDone = CircAna.IsDone();
                            if (WellDone)
                            {
                                NbrSol = CircAna.NbSolutions();
                                for (int i = 1; i <= NbrSol; i++)
                                {
                                    CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                                }
                                Results(CircAna);
                            }
                        }
                    }
                }
            }

            //=============================================================================
            //                            Appel a GccGeo.                                 +
            //=============================================================================

            else
            {
                if (Type1 == GeomAbs_Circle)
                {
                    Geom2d_Circle CCC1 = (CC1 as Geom2d_Circle)!;
                    gp_Circ2d                  c1 = CCC1.Circ2d();
                    GccEnt_QualifiedCirc       Qc1 = new GccEnt_QualifiedCirc(c1, Qualified1.Qualifier());
                    if (Type2 == GeomAbs_Circle)
                    {
                        Geom2d_Circle CCC2 = (CC2 as Geom2d_Circle)!;
                        gp_Circ2d                  c2 = CCC2.Circ2d();
                        GccEnt_QualifiedCirc       Qc2 = new GccEnt_QualifiedCirc(c2, Qualified2.Qualifier());
                        Geom2dGcc_Circ2d2TanOnGeo  CircGeo = new(Qc1, Qc2, OnCurve, Tolerance);
                        WellDone = CircGeo.IsDone();
                        if (WellDone)
                        {
                            NbrSol = CircGeo.NbSolutions();
                            for (int i = 1; i <= NbrSol; i++)
                            {
                                CircGeo.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                            }
                            Results(CircGeo);
                        }
                    }
                    else
                    {
                        Geom2d_Line  LL2 = (CC2 as Geom2d_Line)!;
                        gp_Lin2d                  l2 = LL2.Lin2d();
                        GccEnt_QualifiedLin       Ql2 = new GccEnt_QualifiedLin(l2, Qualified2.Qualifier());
                        Geom2dGcc_Circ2d2TanOnGeo CircGeo = new(Qc1, Ql2, OnCurve, Tolerance);
                        WellDone = CircGeo.IsDone();
                        if (WellDone)
                        {
                            NbrSol = CircGeo.NbSolutions();
                            for (int i = 1; i <= NbrSol; i++)
                            {
                                CircGeo.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                            }
                            Results(CircGeo);
                        }
                    }
                }
                else
                {
                    Geom2d_Line LL1 = (CC1 as Geom2d_Line)!;
                    gp_Lin2d                 l1 = LL1.Lin2d();
                    GccEnt_QualifiedLin      Ql1 = new GccEnt_QualifiedLin(l1, Qualified1.Qualifier());
                    if (Type2 == GeomAbs_Circle)
                    {
                        Geom2d_Circle CCC2 = (CC2 as Geom2d_Circle)!;
                        gp_Circ2d                  c2 = CCC2.Circ2d();
                        GccEnt_QualifiedCirc       Qc2 = new GccEnt_QualifiedCirc(c2, Qualified2.Qualifier());
                        Geom2dGcc_Circ2d2TanOnGeo  CircGeo = new(Qc2, Ql1, OnCurve, Tolerance);
                        WellDone = CircGeo.IsDone();
                        if (WellDone)
                        {
                            NbrSol = CircGeo.NbSolutions();
                            for (int i = 1; i <= NbrSol; i++)
                            {
                                CircGeo.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                            }
                            Results(CircGeo);
                            Invert = true;
                        }
                    }
                    else
                    {
                        Geom2d_Line  LL2 = (CC2 as Geom2d_Line)!;
                        gp_Lin2d                  l2 = LL2.Lin2d();
                        GccEnt_QualifiedLin       Ql2 = new GccEnt_QualifiedLin(l2, Qualified2.Qualifier());
                        Geom2dGcc_Circ2d2TanOnGeo CircGeo = new(Ql1, Ql2, OnCurve, Tolerance);
                        WellDone = CircGeo.IsDone();
                        if (WellDone)
                        {
                            NbrSol = CircGeo.NbSolutions();
                            for (int i = 1; i <= NbrSol; i++)
                            {
                                CircGeo.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                            }
                            Results(CircGeo);
                        }
                    }
                }
            }
        }
        else
        {
            Geom2dGcc_QCurve Qc1 = new(C1, Qualified1.Qualifier());
            Geom2dGcc_QCurve Qc2 = new(C2, Qualified2.Qualifier());
            if ((Type3 == GeomAbs_Circle || Type3 == GeomAbs_Line))
            {
                if (Type3 == GeomAbs_Circle)
                {
                    Geom2d_Circle CCon = (Con as Geom2d_Circle)!;
                    Geom2dGcc_Circ2d2TanOnIter
                        Circ = new(Qc1, Qc2, CCon.Circ2d(), Param1, Param2, ParamOn, Tolerance);
                    WellDone = Circ.IsDone();
                    if (WellDone)
                    {
                        NbrSol    = 1;
                        cirsol[1] = Circ.ThisSolution();
                        if (Circ.IsTheSame1())
                        {
                            TheSame1[1] = 1;
                        }
                        else
                        {
                            TheSame1[1] = 0;
                        }
                        if (Circ.IsTheSame2())
                        {
                            TheSame2[1] = 1;
                        }
                        else
                        {
                            TheSame2[1] = 0;
                        }
                        // OCCT: no WhichQualifier here (the case below overwrites everything anyway)
                        Circ.Tangency1(out par1sol[1], out pararg1[1], out pnttg1sol[1]);
                        Circ.Tangency2(out par2sol[1], out pararg2[1], out pnttg2sol[1]);
                        // OCCT: CenterOn3 is not copied: parcen3(1) stays unset (uninitialised natively, 0 here), pntcen(1) the origin
                    }
                }
                else
                {
                    Geom2d_Line LLon = (Con as Geom2d_Line)!;
                    Geom2dGcc_Circ2d2TanOnIter
                        Circ = new(Qc1, Qc2, LLon.Lin2d(), Param1, Param2, ParamOn, Tolerance);
                    WellDone = Circ.IsDone();
                    if (WellDone)
                    {
                        NbrSol    = 1;
                        cirsol[1] = Circ.ThisSolution();
                        if (Circ.IsTheSame1())
                        {
                            TheSame1[1] = 1;
                        }
                        else
                        {
                            TheSame1[1] = 0;
                        }
                        if (Circ.IsTheSame2())
                        {
                            TheSame2[1] = 1;
                        }
                        else
                        {
                            TheSame2[1] = 0;
                        }
                        Circ.WhichQualifier(out qualifier1[1], out qualifier2[1]);
                        Circ.Tangency1(out par1sol[1], out pararg1[1], out pnttg1sol[1]);
                        Circ.Tangency2(out par2sol[1], out pararg2[1], out pnttg2sol[1]);
                        // OCCT: CenterOn3 is not copied: parcen3(1) stays unset (uninitialised natively, 0 here), pntcen(1) the origin
                    }
                }
            }
            // (Circ in OCCT: renamed, as C# forbids reusing the name of the inner blocks' locals.)
            // OCCT: no "else": this runs after the line and circle cases above too and replaces their result.
            Geom2dGcc_Circ2d2TanOnIter CircCu = new(Qc1, Qc2, OnCurve, Param1, Param2, ParamOn, Tolerance);
            WellDone = CircCu.IsDone();
            if (WellDone)
            {
                NbrSol    = 1;
                cirsol[1] = CircCu.ThisSolution();
                if (CircCu.IsTheSame1())
                {
                    TheSame1[1] = 1;
                }
                else
                {
                    TheSame1[1] = 0;
                }
                if (CircCu.IsTheSame2())
                {
                    TheSame2[1] = 1;
                }
                else
                {
                    TheSame2[1] = 0;
                }
                CircCu.WhichQualifier(out qualifier1[1], out qualifier2[1]);
                CircCu.Tangency1(out par1sol[1], out pararg1[1], out pnttg1sol[1]);
                CircCu.Tangency2(out par2sol[1], out pararg2[1], out pnttg2sol[1]);
                // OCCT: CenterOn3 is not copied: parcen3(1) stays unset (uninitialised natively, 0 here), pntcen(1) the origin
            }
        }
    }

    /// <summary>
    /// Circles tangent to a qualified curve, through a point, whose centres lie on OnCurve (GccAna, Geo or Iter as
    /// for two curves; Param1 and ParamOn start the iteration).
    /// </summary>
    public Geom2dGcc_Circ2d2TanOn(in Geom2dGcc_QualifiedCurve Qualified1, Geom2d_Point Point, Geom2dAdaptor_Curve OnCurve, double Tolerance, double Param1, double ParamOn)
    {
        cirsol     = new(1, 8);
        qualifier1 = new(1, 8);
        qualifier2 = new(1, 8);
        TheSame1   = new(1, 8);
        TheSame2   = new(1, 8);
        pnttg1sol  = new(1, 8);
        pnttg2sol  = new(1, 8);
        pntcen     = new(1, 8);
        par1sol    = new(1, 8);
        par2sol    = new(1, 8);
        pararg1    = new(1, 8);
        pararg2    = new(1, 8);
        parcen3    = new(1, 8);
        WellDone   = false;
        Geom2dAdaptor_Curve              C1    = Qualified1.Qualified();
        GeomAbs_CurveType                Type1 = C1.GetType();
        GeomAbs_CurveType                Type3 = OnCurve.GetType();
        Geom2d_Curve?  CC1   = C1.Curve();
        Geom2d_Curve?  Con   = OnCurve.Curve();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        Invert = false;
        NbrSol = 0;
        if (Type1 == GeomAbs_Line || Type1 == GeomAbs_Circle)
        {
            if (Type3 == GeomAbs_Line || Type3 == GeomAbs_Circle)
            {
                gp_Pnt2d pnt = Point.Pnt2d();
                if (Type1 == GeomAbs_Circle)
                {
                    Geom2d_Circle CCC1 = (CC1 as Geom2d_Circle)!;
                    gp_Circ2d                  c1 = CCC1.Circ2d();
                    GccEnt_QualifiedCirc       Qc1 = new(c1, Qualified1.Qualifier());
                    if (Type3 == GeomAbs_Circle)
                    {
                        Geom2d_Circle CCon = (Con as Geom2d_Circle)!;
                        GccAna_Circ2d2TanOn        CircAna = new(Qc1, pnt, CCon.Circ2d(), Tolerance);
                        WellDone = CircAna.IsDone();
                        if (WellDone)
                        {
                            NbrSol = CircAna.NbSolutions();
                            for (int i = 1; i <= NbrSol; i++)
                            {
                                CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                            }
                            Results(CircAna);
                        }
                    }
                    else if (Type3 == GeomAbs_Line)
                    {
                        Geom2d_Line CCon = (Con as Geom2d_Line)!;
                        GccAna_Circ2d2TanOn      CircAna = new(Qc1, pnt, CCon.Lin2d(), Tolerance);
                        WellDone = CircAna.IsDone();
                        if (WellDone)
                        {
                            NbrSol = CircAna.NbSolutions();
                            for (int i = 1; i <= NbrSol; i++)
                            {
                                CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                            }
                            Results(CircAna);
                        }
                    }
                }
                else
                {
                    Geom2d_Line LLL1 = (CC1 as Geom2d_Line)!;
                    gp_Lin2d                 l1 = LLL1.Lin2d();
                    GccEnt_QualifiedLin      Ql1 = new(l1, Qualified1.Qualifier());
                    if (Type3 == GeomAbs_Circle)
                    {
                        Geom2d_Circle CCon = (Con as Geom2d_Circle)!;
                        GccAna_Circ2d2TanOn        CircAna = new(Ql1, pnt, CCon.Circ2d(), Tolerance);
                        WellDone = CircAna.IsDone();
                        if (WellDone)
                        {
                            NbrSol = CircAna.NbSolutions();
                            for (int i = 1; i <= NbrSol; i++)
                            {
                                CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                            }
                            Results(CircAna);
                        }
                    }
                    else if (Type3 == GeomAbs_Line)
                    {
                        Geom2d_Line LLon = (Con as Geom2d_Line)!;
                        GccAna_Circ2d2TanOn      CircAna = new(Ql1, pnt, LLon.Lin2d(), Tolerance);
                        WellDone = CircAna.IsDone();
                        if (WellDone)
                        {
                            NbrSol = CircAna.NbSolutions();
                            for (int i = 1; i <= NbrSol; i++)
                            {
                                CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                            }
                            Results(CircAna);
                        }
                    }
                }
            }
            //=============================================================================
            //                            Appel a GccGeo.                                 +
            //=============================================================================

            else
            {
                if (Type1 == GeomAbs_Circle)
                {
                    Geom2d_Circle CCC1 = (CC1 as Geom2d_Circle)!;
                    gp_Circ2d                  c1 = CCC1.Circ2d();
                    GccEnt_QualifiedCirc       Qc1 = new(c1, Qualified1.Qualifier());
                    Geom2dGcc_Circ2d2TanOnGeo  CircGeo = new(Qc1, Point.Pnt2d(), OnCurve, Tolerance);
                    WellDone = CircGeo.IsDone();
                    if (WellDone)
                    {
                        NbrSol = CircGeo.NbSolutions();
                        for (int i = 1; i <= NbrSol; i++)
                        {
                            CircGeo.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                        }
                        Results(CircGeo);
                    }
                }
                else
                {
                    Geom2d_Line  LLL1 = (CC1 as Geom2d_Line)!;
                    gp_Lin2d                  l1 = LLL1.Lin2d();
                    GccEnt_QualifiedLin       Ql1 = new(l1, Qualified1.Qualifier());
                    Geom2dGcc_Circ2d2TanOnGeo CircGeo = new(Ql1, Point.Pnt2d(), OnCurve, Tolerance);
                    WellDone = CircGeo.IsDone();
                    if (WellDone)
                    {
                        NbrSol = CircGeo.NbSolutions();
                        for (int i = 1; i <= NbrSol; i++)
                        {
                            CircGeo.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                        }
                        Results(CircGeo);
                    }
                }
            }
        }
        else
        {
            Geom2dGcc_QCurve Qc1 = new(C1, Qualified1.Qualifier());
            if ((Type3 == GeomAbs_Circle || Type3 == GeomAbs_Line))
            {
                if (Type3 == GeomAbs_Circle)
                {
                    Geom2d_Circle CCon = (Con as Geom2d_Circle)!;
                    Geom2dGcc_Circ2d2TanOnIter Circ = new(Qc1,
                                                                                    Point.Pnt2d(),
                                                                                    CCon.Circ2d(),
                                                                                    Param1,
                                                                                    ParamOn,
                                                                                    Tolerance);
                    WellDone = Circ.IsDone();
                    if (WellDone)
                    {
                        NbrSol    = 1;
                        cirsol[1] = Circ.ThisSolution();
                        if (Circ.IsTheSame1())
                        {
                            TheSame1[1] = 1;
                        }
                        else
                        {
                            TheSame1[1] = 0;
                        }
                        // OCCT: TheSame2(1) is not set (uninitialised natively, 0 here); Tangency2 tests it
                        Circ.WhichQualifier(out qualifier1[1], out qualifier2[1]);
                        Circ.Tangency1(out par1sol[1], out pararg1[1], out pnttg1sol[1]);
                        Circ.Tangency2(out par2sol[1], out pararg2[1], out pnttg2sol[1]);
                        // OCCT: CenterOn3 is not copied: parcen3(1) stays unset (uninitialised natively, 0 here), pntcen(1) the origin
                    }
                }
                else
                {
                    Geom2d_Line   LLon = (Con as Geom2d_Line)!;
                    Geom2dGcc_Circ2d2TanOnIter Circ = new(Qc1,
                                                                                    Point.Pnt2d(),
                                                                                    LLon.Lin2d(),
                                                                                    Param1,
                                                                                    ParamOn,
                                                                                    Tolerance);
                    WellDone = Circ.IsDone();
                    if (WellDone)
                    {
                        NbrSol    = 1;
                        cirsol[1] = Circ.ThisSolution();
                        if (Circ.IsTheSame1())
                        {
                            TheSame1[1] = 1;
                        }
                        else
                        {
                            TheSame1[1] = 0;
                        }
                        // OCCT: TheSame2(1) is not set (uninitialised natively, 0 here); Tangency2 tests it
                        Circ.WhichQualifier(out qualifier1[1], out qualifier2[1]);
                        Circ.Tangency1(out par1sol[1], out pararg1[1], out pnttg1sol[1]);
                        Circ.Tangency2(out par2sol[1], out pararg2[1], out pnttg2sol[1]);
                        // OCCT: CenterOn3 is not copied: parcen3(1) stays unset (uninitialised natively, 0 here), pntcen(1) the origin
                    }
                }
            }
            else
            {
                Geom2dGcc_Circ2d2TanOnIter Circ = new(Qc1, Point.Pnt2d(), OnCurve, Param1, ParamOn, Tolerance);
                WellDone = Circ.IsDone();
                if (WellDone)
                {
                    NbrSol    = 1;
                    cirsol[1] = Circ.ThisSolution();
                    if (Circ.IsTheSame1())
                    {
                        TheSame1[1] = 1;
                    }
                    else
                    {
                        TheSame1[1] = 0;
                    }
                    if (Circ.IsTheSame2())
                    {
                        TheSame2[1] = 1;
                    }
                    else
                    {
                        TheSame2[1] = 0;
                    }
                    Circ.WhichQualifier(out qualifier1[1], out qualifier2[1]);
                    Circ.Tangency1(out par1sol[1], out pararg1[1], out pnttg1sol[1]);
                    Circ.Tangency2(out par2sol[1], out pararg2[1], out pnttg2sol[1]);
                    // OCCT: CenterOn3 is not copied: parcen3(1) stays unset (uninitialised natively, 0 here), pntcen(1) the origin
                }
            }
        }
    }

    /// <summary>Circles through two points whose centres lie on OnCurve (GccAna, or Geo for a free-form OnCurve).</summary>
    public Geom2dGcc_Circ2d2TanOn(Geom2d_Point Point1, Geom2d_Point Point2, Geom2dAdaptor_Curve OnCurve, double Tolerance)
    {
        cirsol     = new(1, 8);
        qualifier1 = new(1, 8);
        qualifier2 = new(1, 8);
        TheSame1   = new(1, 8);
        TheSame2   = new(1, 8);
        pnttg1sol  = new(1, 8);
        pnttg2sol  = new(1, 8);
        pntcen     = new(1, 8);
        par1sol    = new(1, 8);
        par2sol    = new(1, 8);
        pararg1    = new(1, 8);
        pararg2    = new(1, 8);
        parcen3    = new(1, 8);
        WellDone   = false;
        GeomAbs_CurveType                Type3 = OnCurve.GetType();
        Geom2d_Curve?  Con   = OnCurve.Curve();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        Invert = false;
        NbrSol = 0;
        if (Type3 == GeomAbs_Line || Type3 == GeomAbs_Circle)
        {
            gp_Pnt2d pnt1 = Point1.Pnt2d();
            gp_Pnt2d pnt2 = Point2.Pnt2d();
            if (Type3 == GeomAbs_Circle)
            {
                Geom2d_Circle CCon = (Con as Geom2d_Circle)!;
                GccAna_Circ2d2TanOn        CircAna = new(pnt1, pnt2, CCon.Circ2d(), Tolerance);
                WellDone = CircAna.IsDone();
                if (WellDone)
                {
                    NbrSol = CircAna.NbSolutions();
                    for (int i = 1; i <= NbrSol; i++)
                    {
                        CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                    }
                    Results(CircAna);
                }
            }
            else
            {
                Geom2d_Line LLon = (Con as Geom2d_Line)!;
                GccAna_Circ2d2TanOn      CircAna = new(pnt1, pnt2, LLon.Lin2d(), Tolerance);
                WellDone = CircAna.IsDone();
                if (WellDone)
                {
                    NbrSol = CircAna.NbSolutions();
                    for (int i = 1; i <= NbrSol; i++)
                    {
                        CircAna.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                    }
                    Results(CircAna);
                }
            }
        }

        //=============================================================================
        //                            Appel a GccGeo.                                 +
        //=============================================================================

        else
        {
            Geom2dGcc_Circ2d2TanOnGeo CircGeo = new(Point1.Pnt2d(), Point2.Pnt2d(), OnCurve, Tolerance);
            WellDone = CircGeo.IsDone();
            if (WellDone)
            {
                NbrSol = CircGeo.NbSolutions();
                for (int i = 1; i <= NbrSol; i++)
                {
                    CircGeo.WhichQualifier(i, out qualifier1[i], out qualifier2[i]);
                }
                Results(CircGeo);
            }
        }
    }

    internal void Results(GccAna_Circ2d2TanOn Circ)
    {
        for (int j = 1; j <= NbrSol; j++)
        {
            cirsol[j] = Circ.ThisSolution(j);
            if (Circ.IsTheSame1(j))
            {
                TheSame1[j] = 1;
            }
            else
            {
                TheSame1[j] = 0;
            }
            if (Circ.IsTheSame2(j))
            {
                TheSame2[j] = 1;
            }
            else
            {
                TheSame2[j] = 0;
            }
            Circ.WhichQualifier(j, out qualifier1[j], out qualifier2[j]);
            Circ.Tangency1(j, out par1sol[j], out pararg1[j], out pnttg1sol[j]);
            Circ.Tangency2(j, out par2sol[j], out pararg2[j], out pnttg2sol[j]);
            Circ.CenterOn3(j, out parcen3[j], out pntcen[j]);
        }
    }

    internal void Results(Geom2dGcc_Circ2d2TanOnGeo Circ)
    {
        for (int j = 1; j <= NbrSol; j++)
        {
            cirsol[j] = Circ.ThisSolution(j);
            if (Circ.IsTheSame1(j))
            {
                TheSame1[j] = 1;
            }
            else
            {
                TheSame1[j] = 0;
            }
            if (Circ.IsTheSame2(j))
            {
                TheSame2[j] = 1;
            }
            else
            {
                TheSame2[j] = 0;
            }
            Circ.WhichQualifier(j, out qualifier1[j], out qualifier2[j]);
            Circ.Tangency1(j, out par1sol[j], out pararg1[j], out pnttg1sol[j]);
            Circ.Tangency2(j, out par2sol[j], out pararg2[j], out pnttg2sol[j]);
            Circ.CenterOn3(j, out parcen3[j], out pntcen[j]);
        }
    }

    /// <summary>True if the construction succeeded (possibly with no solution).</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The number of solutions.</summary>
    public int NbSolutions()
    {
        return NbrSol;
    }

    /// <summary>Solution Index.</summary>
    /// <exception cref="StdFail_NotDone">The construction failed.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbSolutions()].</exception>
    public gp_Circ2d ThisSolution(int Index)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }

        return cirsol[Index];
    }

    /// <summary>The qualifiers of solution Index relative to the two arguments.</summary>
    /// <exception cref="StdFail_NotDone">The construction failed.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbSolutions()].</exception>
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
            if (Invert)
            {
                Qualif1 = qualifier2[Index];
                Qualif2 = qualifier1[Index];
            }
            else
            {
                Qualif1 = qualifier1[Index];
                Qualif2 = qualifier2[Index];
            }
        }
    }

    /// <summary>The tangency point of solution Index on the first argument, with its parameters on both.</summary>
    /// <exception cref="StdFail_NotDone">The construction failed, or the solution is the first argument.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbSolutions()].</exception>
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
            if (Invert)
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
    }

    /// <summary>The tangency point of solution Index on the second argument, with its parameters on both.</summary>
    /// <exception cref="StdFail_NotDone">The construction failed, or the solution is the second argument.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbSolutions()].</exception>
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
            if (!Invert)
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
    }

    /// <summary>The centre of solution Index and its parameter on the centre's curve.</summary>
    /// <exception cref="StdFail_NotDone">The construction failed.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbSolutions()].</exception>
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
            PntSol = pntcen[Index];
        }
    }

    /// <summary>True if solution Index is the first argument.</summary>
    /// <exception cref="StdFail_NotDone">The construction failed.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbSolutions()].</exception>
    public bool IsTheSame1(int Index)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }

        if (Invert)
        {
            return TheSame2[Index] != 0;
        }
        else
        {
            return TheSame1[Index] != 0;
        }
    }

    /// <summary>True if solution Index is the second argument.</summary>
    /// <exception cref="StdFail_NotDone">The construction failed.</exception>
    /// <exception cref="Standard_OutOfRange">Index is not in [1, NbSolutions()].</exception>
    public bool IsTheSame2(int Index)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new Standard_OutOfRange();
        }

        if (!Invert)
        {
            return TheSame2[Index] != 0;
        }
        else
        {
            return TheSame1[Index] != 0;
        }
    }
}
