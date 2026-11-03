// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanOn.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanOn.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

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
    private readonly NCollection_Array1<Circle2D> cirsol;
    private int NbrSol;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<GccEnt_Position> qualifier2;
    private readonly NCollection_Array1<int> TheSame1;
    private readonly NCollection_Array1<int> TheSame2;
    private readonly NCollection_Array1<Point2D> pnttg1sol;
    private readonly NCollection_Array1<Point2D> pnttg2sol;
    private readonly NCollection_Array1<Point2D> pntcen;
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
    public Geom2dGcc_Circ2d2TanOn(in Geom2dGcc_QualifiedCurve Qualified1, in Geom2dGcc_QualifiedCurve Qualified2, CurveAdaptor2D OnCurve, double Tolerance, double Param1, double Param2, double ParamOn)
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
        CurveAdaptor2D              C1    = Qualified1.Qualified();
        CurveAdaptor2D              C2    = Qualified2.Qualified();
        CurveKind                Type1 = C1.GetType();
        CurveKind                Type2 = C2.GetType();
        CurveKind                Type3 = OnCurve.GetType();
        Curve2D?  CC1   = C1.Curve();
        Curve2D?  CC2   = C2.Curve();
        Curve2D?  Con   = OnCurve.Curve();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        Invert = false;
        NbrSol = 0;
        if ((Type1 == Line || Type1 == Circle)
                && (Type2 == Line || Type2 == Circle))
        {
            if (Type3 == Line || Type3 == Circle)
            {
                if (Type1 == Circle)
                {
                    CircleCurve2D CCC1 = (CC1 as CircleCurve2D)!;
                    Circle2D                  c1 = CCC1.Circ2d();
                    GccEnt_QualifiedCirc       Qc1 = new GccEnt_QualifiedCirc(c1, Qualified1.Qualifier());
                    if (Type2 == Circle)
                    {
                        CircleCurve2D CCC2 = (CC2 as CircleCurve2D)!;
                        Circle2D                  c2 = CCC2.Circ2d();
                        if (Type3 == Circle)
                        {
                            CircleCurve2D CCon = (Con as CircleCurve2D)!;
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
                            LineCurve2D LLon = (Con as LineCurve2D)!;
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
                        LineCurve2D LL2 = (CC2 as LineCurve2D)!;
                        Line2D                 l2 = LL2.Lin2d();
                        if (Type3 == Circle)
                        {
                            CircleCurve2D CCon = (Con as CircleCurve2D)!;
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
                            LineCurve2D LLon = (Con as LineCurve2D)!;
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
                    LineCurve2D LL1 = (CC1 as LineCurve2D)!;
                    Line2D                 l1 = LL1.Lin2d();
                    GccEnt_QualifiedLin      Ql1 = new GccEnt_QualifiedLin(l1, Qualified1.Qualifier());
                    if (Type2 == Circle)
                    {
                        CircleCurve2D CCC2 = (CC2 as CircleCurve2D)!;
                        Circle2D                  c2 = CCC2.Circ2d();
                        if (Type3 == Circle)
                        {
                            CircleCurve2D CCon = (Con as CircleCurve2D)!;
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
                            LineCurve2D LLon = (Con as LineCurve2D)!;
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
                        LineCurve2D LL2 = (CC2 as LineCurve2D)!;
                        Line2D                 l2 = LL2.Lin2d();
                        if (Type3 == Circle)
                        {
                            CircleCurve2D CCon = (Con as CircleCurve2D)!;
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
                            LineCurve2D LLon = (Con as LineCurve2D)!;
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
                if (Type1 == Circle)
                {
                    CircleCurve2D CCC1 = (CC1 as CircleCurve2D)!;
                    Circle2D                  c1 = CCC1.Circ2d();
                    GccEnt_QualifiedCirc       Qc1 = new GccEnt_QualifiedCirc(c1, Qualified1.Qualifier());
                    if (Type2 == Circle)
                    {
                        CircleCurve2D CCC2 = (CC2 as CircleCurve2D)!;
                        Circle2D                  c2 = CCC2.Circ2d();
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
                        LineCurve2D  LL2 = (CC2 as LineCurve2D)!;
                        Line2D                  l2 = LL2.Lin2d();
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
                    LineCurve2D LL1 = (CC1 as LineCurve2D)!;
                    Line2D                 l1 = LL1.Lin2d();
                    GccEnt_QualifiedLin      Ql1 = new GccEnt_QualifiedLin(l1, Qualified1.Qualifier());
                    if (Type2 == Circle)
                    {
                        CircleCurve2D CCC2 = (CC2 as CircleCurve2D)!;
                        Circle2D                  c2 = CCC2.Circ2d();
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
                        LineCurve2D  LL2 = (CC2 as LineCurve2D)!;
                        Line2D                  l2 = LL2.Lin2d();
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
            if ((Type3 == Circle || Type3 == Line))
            {
                if (Type3 == Circle)
                {
                    CircleCurve2D CCon = (Con as CircleCurve2D)!;
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
                    LineCurve2D LLon = (Con as LineCurve2D)!;
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
    public Geom2dGcc_Circ2d2TanOn(in Geom2dGcc_QualifiedCurve Qualified1, Point2 Point, CurveAdaptor2D OnCurve, double Tolerance, double Param1, double ParamOn)
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
        CurveAdaptor2D              C1    = Qualified1.Qualified();
        CurveKind                Type1 = C1.GetType();
        CurveKind                Type3 = OnCurve.GetType();
        Curve2D?  CC1   = C1.Curve();
        Curve2D?  Con   = OnCurve.Curve();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        Invert = false;
        NbrSol = 0;
        if (Type1 == Line || Type1 == Circle)
        {
            if (Type3 == Line || Type3 == Circle)
            {
                Point2D pnt = Point.Pnt2d();
                if (Type1 == Circle)
                {
                    CircleCurve2D CCC1 = (CC1 as CircleCurve2D)!;
                    Circle2D                  c1 = CCC1.Circ2d();
                    GccEnt_QualifiedCirc       Qc1 = new(c1, Qualified1.Qualifier());
                    if (Type3 == Circle)
                    {
                        CircleCurve2D CCon = (Con as CircleCurve2D)!;
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
                    else if (Type3 == Line)
                    {
                        LineCurve2D CCon = (Con as LineCurve2D)!;
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
                    LineCurve2D LLL1 = (CC1 as LineCurve2D)!;
                    Line2D                 l1 = LLL1.Lin2d();
                    GccEnt_QualifiedLin      Ql1 = new(l1, Qualified1.Qualifier());
                    if (Type3 == Circle)
                    {
                        CircleCurve2D CCon = (Con as CircleCurve2D)!;
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
                    else if (Type3 == Line)
                    {
                        LineCurve2D LLon = (Con as LineCurve2D)!;
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
                if (Type1 == Circle)
                {
                    CircleCurve2D CCC1 = (CC1 as CircleCurve2D)!;
                    Circle2D                  c1 = CCC1.Circ2d();
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
                    LineCurve2D  LLL1 = (CC1 as LineCurve2D)!;
                    Line2D                  l1 = LLL1.Lin2d();
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
            if ((Type3 == Circle || Type3 == Line))
            {
                if (Type3 == Circle)
                {
                    CircleCurve2D CCon = (Con as CircleCurve2D)!;
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
                    LineCurve2D   LLon = (Con as LineCurve2D)!;
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
    public Geom2dGcc_Circ2d2TanOn(Point2 Point1, Point2 Point2, CurveAdaptor2D OnCurve, double Tolerance)
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
        CurveKind                Type3 = OnCurve.GetType();
        Curve2D?  Con   = OnCurve.Curve();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        Invert = false;
        NbrSol = 0;
        if (Type3 == Line || Type3 == Circle)
        {
            Point2D pnt1 = Point1.Pnt2d();
            Point2D pnt2 = Point2.Pnt2d();
            if (Type3 == Circle)
            {
                CircleCurve2D CCon = (Con as CircleCurve2D)!;
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
                LineCurve2D LLon = (Con as LineCurve2D)!;
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
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
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

    /// <summary>The qualifiers of solution Index relative to the two arguments.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
    public void WhichQualifier(int Index, out GccEnt_Position Qualif1, out GccEnt_Position Qualif2)
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
    /// <exception cref="NotDoneException">The construction failed, or the solution is the first argument.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
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
                    throw new NotDoneException();
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
                    throw new NotDoneException();
                }
            }
        }
    }

    /// <summary>The tangency point of solution Index on the second argument, with its parameters on both.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the second argument.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
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
                    throw new NotDoneException();
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
                    throw new NotDoneException();
                }
            }
        }
    }

    /// <summary>The centre of solution Index and its parameter on the centre's curve.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
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
            PntSol = pntcen[Index];
        }
    }

    /// <summary>True if solution Index is the first argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
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
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
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
