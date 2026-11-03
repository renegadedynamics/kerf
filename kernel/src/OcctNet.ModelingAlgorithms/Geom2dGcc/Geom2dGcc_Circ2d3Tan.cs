// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d3Tan.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d3Tan.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// 2D circles tangent to three arguments, each a qualified curve or a point (passed through). When every curve is a
/// line or a circle, the solutions are GccAna_Circ2d3Tan's; otherwise one solution is found iteratively
/// (Geom2dGcc_Circ2d3TanIter) from the initial parameters Param1 to Param3 on the curves.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). With a free-form argument, NbSolutions is 1 even
/// when the construction fails (IsDone then false), and, as in OCCT, IsTheSame3(1) is true for the iterative solution
/// (Geom2dGcc_Circ2d3TanIter.IsTheSame3), so Tangency3 throws StdFail_NotDone there.
/// </remarks>
public sealed class Geom2dGcc_Circ2d3Tan
{
    private readonly NCollection_Array1<Circle2D> cirsol;
    private double NbrSol;
    private bool WellDone;
    private readonly NCollection_Array1<GccEnt_Position> qualifier1;
    private readonly NCollection_Array1<GccEnt_Position> qualifier2;
    private readonly NCollection_Array1<GccEnt_Position> qualifier3;
    private readonly NCollection_Array1<int> TheSame1;
    private readonly NCollection_Array1<int> TheSame2;
    private readonly NCollection_Array1<int> TheSame3;
    private readonly NCollection_Array1<Point2D> pnttg1sol;
    private readonly NCollection_Array1<Point2D> pnttg2sol;
    private readonly NCollection_Array1<Point2D> pnttg3sol;
    private readonly NCollection_Array1<double> par1sol;
    private readonly NCollection_Array1<double> par2sol;
    private readonly NCollection_Array1<double> par3sol;
    private readonly NCollection_Array1<double> pararg1;
    private readonly NCollection_Array1<double> pararg2;
    private readonly NCollection_Array1<double> pararg3;

    // The initialiser list each of OCCT's constructors writes (arrays of 1 to theNb).
    private Geom2dGcc_Circ2d3Tan(int theNb)
    {
        cirsol     = new(1, theNb);
        qualifier1 = new(1, theNb);
        qualifier2 = new(1, theNb);
        qualifier3 = new(1, theNb);
        TheSame1   = new(1, theNb);
        TheSame2   = new(1, theNb);
        TheSame3   = new(1, theNb);
        pnttg1sol  = new(1, theNb);
        pnttg2sol  = new(1, theNb);
        pnttg3sol  = new(1, theNb);
        par1sol    = new(1, theNb);
        par2sol    = new(1, theNb);
        par3sol    = new(1, theNb);
        pararg1    = new(1, theNb);
        pararg2    = new(1, theNb);
        pararg3    = new(1, theNb);
    }

    public Geom2dGcc_Circ2d3Tan(in Geom2dGcc_QualifiedCurve Qualified1, in Geom2dGcc_QualifiedCurve Qualified2, in Geom2dGcc_QualifiedCurve Qualified3, double Tolerance, double Param1, double Param2, double Param3)
        : this(16)
    {
        CurveAdaptor2D              C1    = Qualified1.Qualified();
        CurveAdaptor2D              C2    = Qualified2.Qualified();
        CurveAdaptor2D              C3    = Qualified3.Qualified();
        Curve2D? CC1   = C1.Curve();
        Curve2D? CC2   = C2.Curve();
        Curve2D? CC3   = C3.Curve();
        CurveKind                Type1 = C1.GetType();
        CurveKind                Type2 = C2.GetType();
        CurveKind                Type3 = C3.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        NbrSol = 0;
        if ((Type1 == Line || Type1 == Circle)
                && (Type2 == Line || Type2 == Circle)
                && (Type3 == Line || Type3 == Circle))
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
                    if (Type3 == Circle)
                    {
                        CircleCurve2D CCC3 = (CC3 as CircleCurve2D)!;
                        Circle2D                  c3 = CCC3.Circ2d();
                        GccEnt_QualifiedCirc       Qc3 = new GccEnt_QualifiedCirc(c3, Qualified3.Qualifier());
                        GccAna_Circ2d3Tan          Circ = new(Qc1, Qc2, Qc3, Tolerance);
                        WellDone = Circ.IsDone();
                        NbrSol   = Circ.NbSolutions();
                        for (int i = 1; i <= NbrSol; i++)
                        {
                            Circ.WhichQualifier(i, out qualifier1[i], out qualifier2[i], out qualifier3[i]);
                        }
                        Results(Circ, 1, 2, 3);
                    }
                    else
                    {
                        LineCurve2D LL3 = (CC3 as LineCurve2D)!;
                        Line2D                 l3 = LL3.Lin2d();
                        GccEnt_QualifiedLin      Ql3 = new GccEnt_QualifiedLin(l3, Qualified3.Qualifier());
                        GccAna_Circ2d3Tan        Circ = new(Qc1, Qc2, Ql3, Tolerance);
                        WellDone = Circ.IsDone();
                        NbrSol   = Circ.NbSolutions();
                        for (int i = 1; i <= NbrSol; i++)
                        {
                            Circ.WhichQualifier(i, out qualifier1[i], out qualifier2[i], out qualifier3[i]);
                        }
                        Results(Circ, 1, 2, 3);
                    }
                }
                else
                {
                    LineCurve2D LL2 = (CC2 as LineCurve2D)!;
                    Line2D                 l2 = LL2.Lin2d();
                    GccEnt_QualifiedLin      Ql2 = new(l2, Qualified2.Qualifier());
                    if (Type3 == Circle)
                    {
                        CircleCurve2D CCC3 = (CC3 as CircleCurve2D)!;
                        Circle2D                  c3 = CCC3.Circ2d();
                        GccEnt_QualifiedCirc       Qc3 = new(c3, Qualified3.Qualifier());
                        GccAna_Circ2d3Tan          Circ = new(Qc1, Qc3, Ql2, Tolerance);
                        WellDone = Circ.IsDone();
                        NbrSol   = Circ.NbSolutions();
                        for (int i = 1; i <= NbrSol; i++)
                        {
                            Circ.WhichQualifier(i, out qualifier1[i], out qualifier3[i], out qualifier2[i]);
                        }
                        Results(Circ, 1, 3, 2);
                    }
                    else
                    {
                        LineCurve2D LL3 = (CC3 as LineCurve2D)!;
                        Line2D                 l3 = LL3.Lin2d();
                        GccEnt_QualifiedLin      Ql3 = new GccEnt_QualifiedLin(l3, Qualified3.Qualifier());
                        GccAna_Circ2d3Tan        Circ = new(Qc1, Ql2, Ql3, Tolerance);
                        WellDone = Circ.IsDone();
                        NbrSol   = Circ.NbSolutions();
                        for (int i = 1; i <= NbrSol; i++)
                        {
                            Circ.WhichQualifier(i, out qualifier1[i], out qualifier2[i], out qualifier3[i]);
                        }
                        Results(Circ, 1, 2, 3);
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
                    if (Type3 == Circle)
                    {
                        CircleCurve2D CCC3 = (CC3 as CircleCurve2D)!;
                        Circle2D                  c3 = CCC3.Circ2d();
                        GccEnt_QualifiedCirc       Qc3 = new GccEnt_QualifiedCirc(c3, Qualified3.Qualifier());
                        GccAna_Circ2d3Tan          Circ = new(Qc2, Qc3, Ql1, Tolerance);
                        WellDone = Circ.IsDone();
                        NbrSol   = Circ.NbSolutions();
                        for (int i = 1; i <= NbrSol; i++)
                        {
                            Circ.WhichQualifier(i, out qualifier3[i], out qualifier1[i], out qualifier2[i]);
                        }
                        Results(Circ, 3, 1, 2);
                    }
                    else
                    {
                        LineCurve2D LL3 = (CC3 as LineCurve2D)!;
                        Line2D                 l3 = LL3.Lin2d();
                        GccEnt_QualifiedLin      Ql3 = new GccEnt_QualifiedLin(l3, Qualified3.Qualifier());
                        GccAna_Circ2d3Tan        Circ = new(Qc2, Ql1, Ql3, Tolerance);
                        WellDone = Circ.IsDone();
                        NbrSol   = Circ.NbSolutions();
                        for (int i = 1; i <= NbrSol; i++)
                        {
                            Circ.WhichQualifier(i, out qualifier2[i], out qualifier1[i], out qualifier3[i]);
                        }
                        Results(Circ, 2, 1, 3);
                    }
                }
                else
                {
                    LineCurve2D LL2 = (CC2 as LineCurve2D)!;
                    Line2D                 l2 = LL2.Lin2d();
                    GccEnt_QualifiedLin      Ql2 = new GccEnt_QualifiedLin(l2, Qualified2.Qualifier());
                    if (Type3 == Circle)
                    {
                        CircleCurve2D CCC3 = (CC3 as CircleCurve2D)!;
                        Circle2D                  c3 = CCC3.Circ2d();
                        GccEnt_QualifiedCirc       Qc3 = new GccEnt_QualifiedCirc(c3, Qualified3.Qualifier());
                        GccAna_Circ2d3Tan          Circ = new(Qc3, Ql2, Ql1, Tolerance);
                        WellDone = Circ.IsDone();
                        NbrSol   = Circ.NbSolutions();
                        for (int i = 1; i <= NbrSol; i++)
                        {
                            Circ.WhichQualifier(i, out qualifier3[i], out qualifier2[i], out qualifier1[i]);
                        }
                        Results(Circ, 3, 2, 1);
                    }
                    else
                    {
                        LineCurve2D LL3 = (CC3 as LineCurve2D)!;
                        Line2D                 l3 = LL3.Lin2d();
                        GccEnt_QualifiedLin      Ql3 = new GccEnt_QualifiedLin(l3, Qualified3.Qualifier());
                        GccAna_Circ2d3Tan        Circ = new(Ql1, Ql2, Ql3, Tolerance);
                        WellDone = Circ.IsDone();
                        NbrSol   = Circ.NbSolutions();
                        for (int i = 1; i <= NbrSol; i++)
                        {
                            Circ.WhichQualifier(i, out qualifier1[i], out qualifier2[i], out qualifier3[i]);
                        }
                        Results(Circ, 1, 2, 3);
                    }
                }
            }
        }
        else
        {
            Geom2dGcc_QCurve         Qc1 = new(C1, Qualified1.Qualifier());
            Geom2dGcc_QCurve         Qc2 = new(C2, Qualified2.Qualifier());
            Geom2dGcc_QCurve         Qc3 = new(C3, Qualified3.Qualifier());
            Geom2dGcc_Circ2d3TanIter Circ = new(Qc1, Qc2, Qc3, Param1, Param2, Param3, Tolerance);
            WellDone = Circ.IsDone();
            NbrSol   = 1;
            if (WellDone)
            {
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
                if (Circ.IsTheSame3())
                {
                    TheSame3[1] = 1;
                }
                else
                {
                    TheSame3[1] = 0;
                }
                Circ.Tangency1(out par1sol[1], out pararg1[1], out pnttg1sol[1]);
                Circ.Tangency2(out par2sol[1], out pararg2[1], out pnttg2sol[1]);
                Circ.Tangency3(out par3sol[1], out pararg3[1], out pnttg3sol[1]);
                Circ.WhichQualifier(out qualifier1[1], out qualifier2[1], out qualifier3[1]);
            }
        }
    }

    public Geom2dGcc_Circ2d3Tan(in Geom2dGcc_QualifiedCurve Qualified1, in Geom2dGcc_QualifiedCurve Qualified2, Point2 Point, double Tolerance, double Param1, double Param2)
        : this(20)
    {
        CurveAdaptor2D              C1    = Qualified1.Qualified();
        CurveAdaptor2D              C2    = Qualified2.Qualified();
        Curve2D? CC1   = C1.Curve();
        Curve2D? CC2   = C2.Curve();
        CurveKind                Type1 = C1.GetType();
        CurveKind                Type2 = C2.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        NbrSol = 0;
        if ((Type1 == Line || Type1 == Circle)
                && (Type2 == Line || Type2 == Circle))
        {
            if (Type1 == Circle)
            {
                CircleCurve2D CCC1 = (CC1 as CircleCurve2D)!;
                Circle2D                  c1 = CCC1.Circ2d();
                GccEnt_QualifiedCirc       Qc1 = new(c1, Qualified1.Qualifier());
                if (Type2 == Circle)
                {
                    CircleCurve2D CCC2 = (CC2 as CircleCurve2D)!;
                    Circle2D                  c2 = CCC2.Circ2d();
                    GccEnt_QualifiedCirc       Qc2 = new(c2, Qualified2.Qualifier());
                    GccAna_Circ2d3Tan          Circ = new(Qc1, Qc2, Point.Pnt2d(), Tolerance);
                    WellDone = Circ.IsDone();
                    NbrSol   = Circ.NbSolutions();
                    for (int i = 1; i <= NbrSol; i++)
                    {
                        Circ.WhichQualifier(i, out qualifier1[i], out qualifier2[i], out qualifier3[i]);
                    }
                    Results(Circ, 1, 2, 3);
                }
                else
                {
                    LineCurve2D LL2 = (CC2 as LineCurve2D)!;
                    Line2D                 l2 = LL2.Lin2d();
                    GccEnt_QualifiedLin      Ql2 = new(l2, Qualified2.Qualifier());
                    GccAna_Circ2d3Tan        Circ = new(Qc1, Ql2, Point.Pnt2d(), Tolerance);
                    WellDone = Circ.IsDone();
                    NbrSol   = Circ.NbSolutions();
                    for (int i = 1; i <= NbrSol; i++)
                    {
                        Circ.WhichQualifier(i, out qualifier1[i], out qualifier2[i], out qualifier3[i]);
                    }
                    Results(Circ, 1, 2, 3);
                }
            }
            else
            {
                LineCurve2D LL1 = (CC1 as LineCurve2D)!;
                Line2D                 l1 = LL1.Lin2d();
                GccEnt_QualifiedLin      Ql1 = new(l1, Qualified1.Qualifier());
                if (Type2 == Circle)
                {
                    CircleCurve2D CCC2 = (CC2 as CircleCurve2D)!;
                    Circle2D                  c2 = CCC2.Circ2d();
                    GccEnt_QualifiedCirc       Qc2 = new(c2, Qualified2.Qualifier());
                    GccAna_Circ2d3Tan          Circ = new(Qc2, Ql1, Point.Pnt2d(), Tolerance);
                    WellDone = Circ.IsDone();
                    NbrSol   = Circ.NbSolutions();
                    for (int i = 1; i <= NbrSol; i++)
                    {
                        Circ.WhichQualifier(i, out qualifier2[i], out qualifier1[i], out qualifier3[i]);
                    }
                    Results(Circ, 2, 1, 3);
                }
                else
                {
                    LineCurve2D LL2 = (CC2 as LineCurve2D)!;
                    Line2D                 l2 = LL2.Lin2d();
                    GccEnt_QualifiedLin      Ql2 = new(l2, Qualified2.Qualifier());
                    GccAna_Circ2d3Tan        Circ = new(Ql1, Ql2, Point.Pnt2d(), Tolerance);
                    WellDone = Circ.IsDone();
                    NbrSol   = Circ.NbSolutions();
                    for (int i = 1; i <= NbrSol; i++)
                    {
                        Circ.WhichQualifier(i, out qualifier1[i], out qualifier2[i], out qualifier3[i]);
                    }
                    Results(Circ, 1, 2, 3);
                }
            }
        }
        else
        {
            Geom2dGcc_QCurve         Qc1 = new(C1, Qualified1.Qualifier());
            Geom2dGcc_QCurve         Qc2 = new(C2, Qualified2.Qualifier());
            Geom2dGcc_Circ2d3TanIter Circ = new(Qc1, Qc2, Point.Pnt2d(), Param1, Param2, Tolerance);
            WellDone = Circ.IsDone();
            NbrSol   = 1;
            if (WellDone)
            {
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
                if (Circ.IsTheSame3())
                {
                    TheSame3[1] = 1;
                }
                else
                {
                    TheSame3[1] = 0;
                }
                Circ.Tangency1(out par1sol[1], out pararg1[1], out pnttg1sol[1]);
                Circ.Tangency2(out par2sol[1], out pararg2[1], out pnttg2sol[1]);
                Circ.Tangency3(out par3sol[1], out pararg3[1], out pnttg3sol[1]);
                Circ.WhichQualifier(out qualifier1[1], out qualifier2[1], out qualifier3[1]);
            }
        }
    }

    public Geom2dGcc_Circ2d3Tan(in Geom2dGcc_QualifiedCurve Qualified1, Point2 Point1, Point2 Point2, double Tolerance, double Param1)
        : this(16)
    {
        CurveAdaptor2D              C1    = Qualified1.Qualified();
        Curve2D? CC1   = C1.Curve();
        CurveKind                Type1 = C1.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        NbrSol = 0;
        if ((Type1 == Line || Type1 == Circle))
        {
            if (Type1 == Circle)
            {
                CircleCurve2D CCC1 = (CC1 as CircleCurve2D)!;
                Circle2D                  c1 = CCC1.Circ2d();
                GccEnt_QualifiedCirc       Qc1 = new(c1, Qualified1.Qualifier());
                GccAna_Circ2d3Tan          Circ = new(Qc1, Point1.Pnt2d(), Point2.Pnt2d(), Tolerance);
                WellDone = Circ.IsDone();
                NbrSol   = Circ.NbSolutions();
                for (int i = 1; i <= NbrSol; i++)
                {
                    Circ.WhichQualifier(i, out qualifier1[i], out qualifier2[i], out qualifier3[i]);
                }
                Results(Circ, 1, 2, 3);
            }
            else
            {
                LineCurve2D LL1 = (CC1 as LineCurve2D)!;
                Line2D                 l1 = LL1.Lin2d();
                GccEnt_QualifiedLin      Ql1 = new(l1, Qualified1.Qualifier());
                GccAna_Circ2d3Tan        Circ = new(Ql1, Point1.Pnt2d(), Point2.Pnt2d(), Tolerance);
                WellDone = Circ.IsDone();
                NbrSol   = Circ.NbSolutions();
                for (int i = 1; i <= NbrSol; i++)
                {
                    Circ.WhichQualifier(i, out qualifier1[i], out qualifier2[i], out qualifier3[i]);
                }
                Results(Circ, 1, 2, 3);
            }
        }
        else
        {
            Geom2dGcc_QCurve         Qc1 = new(C1, Qualified1.Qualifier());
            Geom2dGcc_Circ2d3TanIter Circ = new(Qc1, Point1.Pnt2d(), Point2.Pnt2d(), Param1, Tolerance);
            WellDone = Circ.IsDone();
            NbrSol   = 1;
            if (WellDone)
            {
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
                if (Circ.IsTheSame3())
                {
                    TheSame3[1] = 1;
                }
                else
                {
                    TheSame3[1] = 0;
                }
                Circ.Tangency1(out par1sol[1], out pararg1[1], out pnttg1sol[1]);
                Circ.Tangency2(out par2sol[1], out pararg2[1], out pnttg2sol[1]);
                Circ.Tangency3(out par3sol[1], out pararg3[1], out pnttg3sol[1]);
                Circ.WhichQualifier(out qualifier1[1], out qualifier2[1], out qualifier3[1]);
            }
        }
    }

    public Geom2dGcc_Circ2d3Tan(Point2 Point1, Point2 Point2, Point2 Point3, double Tolerance)
        : this(2)
    {

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        NbrSol = 0;
        GccAna_Circ2d3Tan Circ = new(Point1.Pnt2d(), Point2.Pnt2d(), Point3.Pnt2d(), Tolerance);
        WellDone = Circ.IsDone();
        NbrSol   = Circ.NbSolutions();
        for (int i = 1; i <= NbrSol; i++)
        {
            Circ.WhichQualifier(i, out qualifier1[i], out qualifier2[i], out qualifier3[i]);
        }
        Results(Circ, 1, 2, 3);
    }

    internal void Results(GccAna_Circ2d3Tan Circ, int Rank1, int Rank2, int Rank3)
    {
        for (int j = 1; j <= NbrSol; j++)
        {
            cirsol[j] = Circ.ThisSolution(j);
            int i1 = 0, i2 = 0, i3 = 0;
            if (Circ.IsTheSame1(j))
            {
                i1 = 1;
            }
            if (Circ.IsTheSame2(j))
            {
                i2 = 1;
            }
            if (Circ.IsTheSame3(j))
            {
                i3 = 1;
            }
            if (Rank1 == 1)
            {
                TheSame1[j] = i1;
                if (i1 == 0)
                {
                    Circ.Tangency1(j, out par1sol[j], out pararg1[j], out pnttg1sol[j]);
                }
            }
            else if (Rank1 == 2)
            {
                TheSame1[j] = i2;
                if (i2 == 0)
                {
                    Circ.Tangency2(j, out par1sol[j], out pararg1[j], out pnttg1sol[j]);
                }
            }
            else if (Rank1 == 3)
            {
                TheSame1[j] = i3;
                if (i3 == 0)
                {
                    Circ.Tangency3(j, out par1sol[j], out pararg1[j], out pnttg1sol[j]);
                }
            }
            if (Rank2 == 1)
            {
                TheSame2[j] = i1;
                if (i1 == 0)
                {
                    Circ.Tangency1(j, out par2sol[j], out pararg2[j], out pnttg2sol[j]);
                }
            }
            else if (Rank2 == 2)
            {
                TheSame2[j] = i2;
                if (i2 == 0)
                {
                    Circ.Tangency2(j, out par2sol[j], out pararg2[j], out pnttg2sol[j]);
                }
            }
            else if (Rank2 == 3)
            {
                TheSame2[j] = i3;
                if (i3 == 0)
                {
                    Circ.Tangency3(j, out par2sol[j], out pararg2[j], out pnttg2sol[j]);
                }
            }
            if (Rank3 == 1)
            {
                TheSame3[j] = i1;
                if (i1 == 0)
                {
                    Circ.Tangency1(j, out par3sol[j], out pararg3[j], out pnttg3sol[j]);
                }
            }
            else if (Rank3 == 2)
            {
                TheSame3[j] = i2;
                if (i2 == 0)
                {
                    Circ.Tangency2(j, out par3sol[j], out pararg3[j], out pnttg3sol[j]);
                }
            }
            else if (Rank3 == 3)
            {
                TheSame3[j] = i3;
                if (i3 == 0)
                {
                    Circ.Tangency3(j, out par3sol[j], out pararg3[j], out pnttg3sol[j]);
                }
            }
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
        return (int)NbrSol;
    }

    /// <summary>The solution of rank Index.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1 .. NbSolutions.</exception>
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

    /// <summary>The qualifiers of the three arguments for the solution of rank Index.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1 .. NbSolutions.</exception>
    public void WhichQualifier(int Index, out GccEnt_Position Qualif1, out GccEnt_Position Qualif2, out GccEnt_Position Qualif3)
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
            Qualif2 = qualifier2[Index];
            Qualif3 = qualifier3[Index];
        }
    }

    /// <summary>
    /// The tangency point of the solution of rank Index on argument 1: its parameter on the solution, on the argument,
    /// and the point.
    /// </summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is argument 1.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1 .. NbSolutions.</exception>
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

    /// <summary>
    /// The tangency point of the solution of rank Index on argument 2: its parameter on the solution, on the argument,
    /// and the point.
    /// </summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is argument 2.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1 .. NbSolutions.</exception>
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

    /// <summary>
    /// The tangency point of the solution of rank Index on argument 3: its parameter on the solution, on the argument,
    /// and the point.
    /// </summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is argument 3.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1 .. NbSolutions.</exception>
    public void Tangency3(int Index, out double ParSol, out double ParArg, out Point2D PntSol)
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
            if (TheSame3[Index] == 0)
            {
                ParSol = par3sol[Index];
                ParArg = pararg3[Index];
                PntSol = pnttg3sol[Index];
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>True if the solution of rank Index is argument 1 (then it has no tangency point there).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1 .. NbSolutions.</exception>
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

    /// <summary>True if the solution of rank Index is argument 2 (then it has no tangency point there).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1 .. NbSolutions.</exception>
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

    /// <summary>True if the solution of rank Index is argument 3 (then it has no tangency point there).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1 .. NbSolutions.</exception>
    public bool IsTheSame3(int Index)
    {
        if (!WellDone)
        {
            throw new NotDoneException();
        }
        if (Index <= 0 || Index > NbrSol)
        {
            throw new OutOfRangeException();
        }
        if (TheSame3[Index] == 0)
        {
            return false;
        }
        return true;
    }
}
