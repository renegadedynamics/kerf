// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d3Tan.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d3Tan.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Construction.Constraints;

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
public sealed class Circ2d3Tan
{
    private readonly NCollection_Array1<Circle2D> _cirsol;
    private double _nbrSol;
    private bool _wellDone;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<Qualifier> _qualifier2;
    private readonly NCollection_Array1<Qualifier> _qualifier3;
    private readonly NCollection_Array1<int> _theSame1;
    private readonly NCollection_Array1<int> _theSame2;
    private readonly NCollection_Array1<int> _theSame3;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<Point2D> _pnttg2sol;
    private readonly NCollection_Array1<Point2D> _pnttg3sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _par2sol;
    private readonly NCollection_Array1<double> _par3sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<double> _pararg2;
    private readonly NCollection_Array1<double> _pararg3;

    // The initialiser list each of OCCT's constructors writes (arrays of 1 to theNb).
    private Circ2d3Tan(int nb)
    {
        _cirsol     = new(1, nb);
        _qualifier1 = new(1, nb);
        _qualifier2 = new(1, nb);
        _qualifier3 = new(1, nb);
        _theSame1   = new(1, nb);
        _theSame2   = new(1, nb);
        _theSame3   = new(1, nb);
        _pnttg1sol  = new(1, nb);
        _pnttg2sol  = new(1, nb);
        _pnttg3sol  = new(1, nb);
        _par1sol    = new(1, nb);
        _par2sol    = new(1, nb);
        _par3sol    = new(1, nb);
        _pararg1    = new(1, nb);
        _pararg2    = new(1, nb);
        _pararg3    = new(1, nb);
    }

    public Circ2d3Tan(in QualifiedCurve qualified1, in QualifiedCurve qualified2, in QualifiedCurve qualified3, double tolerance, double param1, double param2, double param3)
        : this(16)
    {
        CurveAdaptor2D              C1    = qualified1.Qualified();
        CurveAdaptor2D              C2    = qualified2.Qualified();
        CurveAdaptor2D              C3    = qualified3.Qualified();
        Curve2D? cc1   = C1.Curve();
        Curve2D? cc2   = C2.Curve();
        Curve2D? cc3   = C3.Curve();
        CurveKind                type1 = C1.GetType();
        CurveKind                type2 = C2.GetType();
        CurveKind                type3 = C3.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        _nbrSol = 0;
        if ((type1 == Line || type1 == Circle)
                && (type2 == Line || type2 == Circle)
                && (type3 == Line || type3 == Circle))
        {
            if (type1 == Circle)
            {
                CircleCurve2D ccc1 = (cc1 as CircleCurve2D)!;
                Circle2D                  c1 = ccc1.Circ2d();
                QualifiedCirc       qc1 = new QualifiedCirc(c1, qualified1.Qualifier());
                if (type2 == Circle)
                {
                    CircleCurve2D ccc2 = (cc2 as CircleCurve2D)!;
                    Circle2D                  c2 = ccc2.Circ2d();
                    QualifiedCirc       qc2 = new QualifiedCirc(c2, qualified2.Qualifier());
                    if (type3 == Circle)
                    {
                        CircleCurve2D ccc3 = (cc3 as CircleCurve2D)!;
                        Circle2D                  c3 = ccc3.Circ2d();
                        QualifiedCirc       qc3 = new QualifiedCirc(c3, qualified3.Qualifier());
                        AnalyticCirc2d3Tan          circ = new(qc1, qc2, qc3, tolerance);
                        _wellDone = circ.IsDone;
                        _nbrSol   = circ.SolutionCount;
                        for (int i = 1; i <= _nbrSol; i++)
                        {
                            circ.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i], out _qualifier3[i]);
                        }
                        Results(circ, 1, 2, 3);
                    }
                    else
                    {
                        LineCurve2D ll3 = (cc3 as LineCurve2D)!;
                        Line2D                 l3 = ll3.Lin2d();
                        QualifiedLin      ql3 = new QualifiedLin(l3, qualified3.Qualifier());
                        AnalyticCirc2d3Tan        circ = new(qc1, qc2, ql3, tolerance);
                        _wellDone = circ.IsDone;
                        _nbrSol   = circ.SolutionCount;
                        for (int i = 1; i <= _nbrSol; i++)
                        {
                            circ.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i], out _qualifier3[i]);
                        }
                        Results(circ, 1, 2, 3);
                    }
                }
                else
                {
                    LineCurve2D ll2 = (cc2 as LineCurve2D)!;
                    Line2D                 l2 = ll2.Lin2d();
                    QualifiedLin      ql2 = new(l2, qualified2.Qualifier());
                    if (type3 == Circle)
                    {
                        CircleCurve2D ccc3 = (cc3 as CircleCurve2D)!;
                        Circle2D                  c3 = ccc3.Circ2d();
                        QualifiedCirc       qc3 = new(c3, qualified3.Qualifier());
                        AnalyticCirc2d3Tan          circ = new(qc1, qc3, ql2, tolerance);
                        _wellDone = circ.IsDone;
                        _nbrSol   = circ.SolutionCount;
                        for (int i = 1; i <= _nbrSol; i++)
                        {
                            circ.WhichQualifier(i, out _qualifier1[i], out _qualifier3[i], out _qualifier2[i]);
                        }
                        Results(circ, 1, 3, 2);
                    }
                    else
                    {
                        LineCurve2D ll3 = (cc3 as LineCurve2D)!;
                        Line2D                 l3 = ll3.Lin2d();
                        QualifiedLin      ql3 = new QualifiedLin(l3, qualified3.Qualifier());
                        AnalyticCirc2d3Tan        circ = new(qc1, ql2, ql3, tolerance);
                        _wellDone = circ.IsDone;
                        _nbrSol   = circ.SolutionCount;
                        for (int i = 1; i <= _nbrSol; i++)
                        {
                            circ.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i], out _qualifier3[i]);
                        }
                        Results(circ, 1, 2, 3);
                    }
                }
            }
            else
            {
                LineCurve2D ll1 = (cc1 as LineCurve2D)!;
                Line2D                 l1 = ll1.Lin2d();
                QualifiedLin      ql1 = new QualifiedLin(l1, qualified1.Qualifier());
                if (type2 == Circle)
                {
                    CircleCurve2D ccc2 = (cc2 as CircleCurve2D)!;
                    Circle2D                  c2 = ccc2.Circ2d();
                    QualifiedCirc       qc2 = new QualifiedCirc(c2, qualified2.Qualifier());
                    if (type3 == Circle)
                    {
                        CircleCurve2D ccc3 = (cc3 as CircleCurve2D)!;
                        Circle2D                  c3 = ccc3.Circ2d();
                        QualifiedCirc       qc3 = new QualifiedCirc(c3, qualified3.Qualifier());
                        AnalyticCirc2d3Tan          circ = new(qc2, qc3, ql1, tolerance);
                        _wellDone = circ.IsDone;
                        _nbrSol   = circ.SolutionCount;
                        for (int i = 1; i <= _nbrSol; i++)
                        {
                            circ.WhichQualifier(i, out _qualifier3[i], out _qualifier1[i], out _qualifier2[i]);
                        }
                        Results(circ, 3, 1, 2);
                    }
                    else
                    {
                        LineCurve2D ll3 = (cc3 as LineCurve2D)!;
                        Line2D                 l3 = ll3.Lin2d();
                        QualifiedLin      ql3 = new QualifiedLin(l3, qualified3.Qualifier());
                        AnalyticCirc2d3Tan        circ = new(qc2, ql1, ql3, tolerance);
                        _wellDone = circ.IsDone;
                        _nbrSol   = circ.SolutionCount;
                        for (int i = 1; i <= _nbrSol; i++)
                        {
                            circ.WhichQualifier(i, out _qualifier2[i], out _qualifier1[i], out _qualifier3[i]);
                        }
                        Results(circ, 2, 1, 3);
                    }
                }
                else
                {
                    LineCurve2D ll2 = (cc2 as LineCurve2D)!;
                    Line2D                 l2 = ll2.Lin2d();
                    QualifiedLin      ql2 = new QualifiedLin(l2, qualified2.Qualifier());
                    if (type3 == Circle)
                    {
                        CircleCurve2D ccc3 = (cc3 as CircleCurve2D)!;
                        Circle2D                  c3 = ccc3.Circ2d();
                        QualifiedCirc       qc3 = new QualifiedCirc(c3, qualified3.Qualifier());
                        AnalyticCirc2d3Tan          circ = new(qc3, ql2, ql1, tolerance);
                        _wellDone = circ.IsDone;
                        _nbrSol   = circ.SolutionCount;
                        for (int i = 1; i <= _nbrSol; i++)
                        {
                            circ.WhichQualifier(i, out _qualifier3[i], out _qualifier2[i], out _qualifier1[i]);
                        }
                        Results(circ, 3, 2, 1);
                    }
                    else
                    {
                        LineCurve2D ll3 = (cc3 as LineCurve2D)!;
                        Line2D                 l3 = ll3.Lin2d();
                        QualifiedLin      ql3 = new QualifiedLin(l3, qualified3.Qualifier());
                        AnalyticCirc2d3Tan        circ = new(ql1, ql2, ql3, tolerance);
                        _wellDone = circ.IsDone;
                        _nbrSol   = circ.SolutionCount;
                        for (int i = 1; i <= _nbrSol; i++)
                        {
                            circ.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i], out _qualifier3[i]);
                        }
                        Results(circ, 1, 2, 3);
                    }
                }
            }
        }
        else
        {
            QCurve         qc1 = new(C1, qualified1.Qualifier());
            QCurve         qc2 = new(C2, qualified2.Qualifier());
            QCurve         qc3 = new(C3, qualified3.Qualifier());
            Circ2d3TanIter circ = new(qc1, qc2, qc3, param1, param2, param3, tolerance);
            _wellDone = circ.IsDone;
            _nbrSol   = 1;
            if (_wellDone)
            {
                _cirsol[1] = circ.ThisSolution();
                if (circ.IsTheSame1())
                {
                    _theSame1[1] = 1;
                }
                else
                {
                    _theSame1[1] = 0;
                }
                if (circ.IsTheSame2())
                {
                    _theSame2[1] = 1;
                }
                else
                {
                    _theSame2[1] = 0;
                }
                if (circ.IsTheSame3)
                {
                    _theSame3[1] = 1;
                }
                else
                {
                    _theSame3[1] = 0;
                }
                circ.Tangency1(out _par1sol[1], out _pararg1[1], out _pnttg1sol[1]);
                circ.Tangency2(out _par2sol[1], out _pararg2[1], out _pnttg2sol[1]);
                circ.Tangency3(out _par3sol[1], out _pararg3[1], out _pnttg3sol[1]);
                circ.WhichQualifier(out _qualifier1[1], out _qualifier2[1], out _qualifier3[1]);
            }
        }
    }

    public Circ2d3Tan(in QualifiedCurve qualified1, in QualifiedCurve qualified2, Point2 point, double tolerance, double param1, double param2)
        : this(20)
    {
        CurveAdaptor2D              C1    = qualified1.Qualified();
        CurveAdaptor2D              C2    = qualified2.Qualified();
        Curve2D? cc1   = C1.Curve();
        Curve2D? cc2   = C2.Curve();
        CurveKind                type1 = C1.GetType();
        CurveKind                type2 = C2.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        _nbrSol = 0;
        if ((type1 == Line || type1 == Circle)
                && (type2 == Line || type2 == Circle))
        {
            if (type1 == Circle)
            {
                CircleCurve2D ccc1 = (cc1 as CircleCurve2D)!;
                Circle2D                  c1 = ccc1.Circ2d();
                QualifiedCirc       qc1 = new(c1, qualified1.Qualifier());
                if (type2 == Circle)
                {
                    CircleCurve2D ccc2 = (cc2 as CircleCurve2D)!;
                    Circle2D                  c2 = ccc2.Circ2d();
                    QualifiedCirc       qc2 = new(c2, qualified2.Qualifier());
                    AnalyticCirc2d3Tan          circ = new(qc1, qc2, point.Pnt2d(), tolerance);
                    _wellDone = circ.IsDone;
                    _nbrSol   = circ.SolutionCount;
                    for (int i = 1; i <= _nbrSol; i++)
                    {
                        circ.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i], out _qualifier3[i]);
                    }
                    Results(circ, 1, 2, 3);
                }
                else
                {
                    LineCurve2D ll2 = (cc2 as LineCurve2D)!;
                    Line2D                 l2 = ll2.Lin2d();
                    QualifiedLin      ql2 = new(l2, qualified2.Qualifier());
                    AnalyticCirc2d3Tan        circ = new(qc1, ql2, point.Pnt2d(), tolerance);
                    _wellDone = circ.IsDone;
                    _nbrSol   = circ.SolutionCount;
                    for (int i = 1; i <= _nbrSol; i++)
                    {
                        circ.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i], out _qualifier3[i]);
                    }
                    Results(circ, 1, 2, 3);
                }
            }
            else
            {
                LineCurve2D ll1 = (cc1 as LineCurve2D)!;
                Line2D                 l1 = ll1.Lin2d();
                QualifiedLin      ql1 = new(l1, qualified1.Qualifier());
                if (type2 == Circle)
                {
                    CircleCurve2D ccc2 = (cc2 as CircleCurve2D)!;
                    Circle2D                  c2 = ccc2.Circ2d();
                    QualifiedCirc       qc2 = new(c2, qualified2.Qualifier());
                    AnalyticCirc2d3Tan          circ = new(qc2, ql1, point.Pnt2d(), tolerance);
                    _wellDone = circ.IsDone;
                    _nbrSol   = circ.SolutionCount;
                    for (int i = 1; i <= _nbrSol; i++)
                    {
                        circ.WhichQualifier(i, out _qualifier2[i], out _qualifier1[i], out _qualifier3[i]);
                    }
                    Results(circ, 2, 1, 3);
                }
                else
                {
                    LineCurve2D ll2 = (cc2 as LineCurve2D)!;
                    Line2D                 l2 = ll2.Lin2d();
                    QualifiedLin      ql2 = new(l2, qualified2.Qualifier());
                    AnalyticCirc2d3Tan        circ = new(ql1, ql2, point.Pnt2d(), tolerance);
                    _wellDone = circ.IsDone;
                    _nbrSol   = circ.SolutionCount;
                    for (int i = 1; i <= _nbrSol; i++)
                    {
                        circ.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i], out _qualifier3[i]);
                    }
                    Results(circ, 1, 2, 3);
                }
            }
        }
        else
        {
            QCurve         qc1 = new(C1, qualified1.Qualifier());
            QCurve         qc2 = new(C2, qualified2.Qualifier());
            Circ2d3TanIter circ = new(qc1, qc2, point.Pnt2d(), param1, param2, tolerance);
            _wellDone = circ.IsDone;
            _nbrSol   = 1;
            if (_wellDone)
            {
                _cirsol[1] = circ.ThisSolution();
                if (circ.IsTheSame1())
                {
                    _theSame1[1] = 1;
                }
                else
                {
                    _theSame1[1] = 0;
                }
                if (circ.IsTheSame2())
                {
                    _theSame2[1] = 1;
                }
                else
                {
                    _theSame2[1] = 0;
                }
                if (circ.IsTheSame3)
                {
                    _theSame3[1] = 1;
                }
                else
                {
                    _theSame3[1] = 0;
                }
                circ.Tangency1(out _par1sol[1], out _pararg1[1], out _pnttg1sol[1]);
                circ.Tangency2(out _par2sol[1], out _pararg2[1], out _pnttg2sol[1]);
                circ.Tangency3(out _par3sol[1], out _pararg3[1], out _pnttg3sol[1]);
                circ.WhichQualifier(out _qualifier1[1], out _qualifier2[1], out _qualifier3[1]);
            }
        }
    }

    public Circ2d3Tan(in QualifiedCurve qualified1, Point2 point1, Point2 point2, double tolerance, double param1)
        : this(16)
    {
        CurveAdaptor2D              C1    = qualified1.Qualified();
        Curve2D? cc1   = C1.Curve();
        CurveKind                type1 = C1.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        _nbrSol = 0;
        if ((type1 == Line || type1 == Circle))
        {
            if (type1 == Circle)
            {
                CircleCurve2D ccc1 = (cc1 as CircleCurve2D)!;
                Circle2D                  c1 = ccc1.Circ2d();
                QualifiedCirc       qc1 = new(c1, qualified1.Qualifier());
                AnalyticCirc2d3Tan          circ = new(qc1, point1.Pnt2d(), point2.Pnt2d(), tolerance);
                _wellDone = circ.IsDone;
                _nbrSol   = circ.SolutionCount;
                for (int i = 1; i <= _nbrSol; i++)
                {
                    circ.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i], out _qualifier3[i]);
                }
                Results(circ, 1, 2, 3);
            }
            else
            {
                LineCurve2D ll1 = (cc1 as LineCurve2D)!;
                Line2D                 l1 = ll1.Lin2d();
                QualifiedLin      ql1 = new(l1, qualified1.Qualifier());
                AnalyticCirc2d3Tan        circ = new(ql1, point1.Pnt2d(), point2.Pnt2d(), tolerance);
                _wellDone = circ.IsDone;
                _nbrSol   = circ.SolutionCount;
                for (int i = 1; i <= _nbrSol; i++)
                {
                    circ.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i], out _qualifier3[i]);
                }
                Results(circ, 1, 2, 3);
            }
        }
        else
        {
            QCurve         qc1 = new(C1, qualified1.Qualifier());
            Circ2d3TanIter circ = new(qc1, point1.Pnt2d(), point2.Pnt2d(), param1, tolerance);
            _wellDone = circ.IsDone;
            _nbrSol   = 1;
            if (_wellDone)
            {
                _cirsol[1] = circ.ThisSolution();
                if (circ.IsTheSame1())
                {
                    _theSame1[1] = 1;
                }
                else
                {
                    _theSame1[1] = 0;
                }
                if (circ.IsTheSame2())
                {
                    _theSame2[1] = 1;
                }
                else
                {
                    _theSame2[1] = 0;
                }
                if (circ.IsTheSame3)
                {
                    _theSame3[1] = 1;
                }
                else
                {
                    _theSame3[1] = 0;
                }
                circ.Tangency1(out _par1sol[1], out _pararg1[1], out _pnttg1sol[1]);
                circ.Tangency2(out _par2sol[1], out _pararg2[1], out _pnttg2sol[1]);
                circ.Tangency3(out _par3sol[1], out _pararg3[1], out _pnttg3sol[1]);
                circ.WhichQualifier(out _qualifier1[1], out _qualifier2[1], out _qualifier3[1]);
            }
        }
    }

    public Circ2d3Tan(Point2 point1, Point2 point2, Point2 point3, double tolerance)
        : this(2)
    {

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        _nbrSol = 0;
        AnalyticCirc2d3Tan circ = new(point1.Pnt2d(), point2.Pnt2d(), point3.Pnt2d(), tolerance);
        _wellDone = circ.IsDone;
        _nbrSol   = circ.SolutionCount;
        for (int i = 1; i <= _nbrSol; i++)
        {
            circ.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i], out _qualifier3[i]);
        }
        Results(circ, 1, 2, 3);
    }

    internal void Results(AnalyticCirc2d3Tan circ, int rank1, int rank2, int rank3)
    {
        for (int j = 1; j <= _nbrSol; j++)
        {
            _cirsol[j] = circ.ThisSolution(j);
            int i1 = 0, i2 = 0, i3 = 0;
            if (circ.IsTheSame1(j))
            {
                i1 = 1;
            }
            if (circ.IsTheSame2(j))
            {
                i2 = 1;
            }
            if (circ.IsTheSame3(j))
            {
                i3 = 1;
            }
            if (rank1 == 1)
            {
                _theSame1[j] = i1;
                if (i1 == 0)
                {
                    circ.Tangency1(j, out _par1sol[j], out _pararg1[j], out _pnttg1sol[j]);
                }
            }
            else if (rank1 == 2)
            {
                _theSame1[j] = i2;
                if (i2 == 0)
                {
                    circ.Tangency2(j, out _par1sol[j], out _pararg1[j], out _pnttg1sol[j]);
                }
            }
            else if (rank1 == 3)
            {
                _theSame1[j] = i3;
                if (i3 == 0)
                {
                    circ.Tangency3(j, out _par1sol[j], out _pararg1[j], out _pnttg1sol[j]);
                }
            }
            if (rank2 == 1)
            {
                _theSame2[j] = i1;
                if (i1 == 0)
                {
                    circ.Tangency1(j, out _par2sol[j], out _pararg2[j], out _pnttg2sol[j]);
                }
            }
            else if (rank2 == 2)
            {
                _theSame2[j] = i2;
                if (i2 == 0)
                {
                    circ.Tangency2(j, out _par2sol[j], out _pararg2[j], out _pnttg2sol[j]);
                }
            }
            else if (rank2 == 3)
            {
                _theSame2[j] = i3;
                if (i3 == 0)
                {
                    circ.Tangency3(j, out _par2sol[j], out _pararg2[j], out _pnttg2sol[j]);
                }
            }
            if (rank3 == 1)
            {
                _theSame3[j] = i1;
                if (i1 == 0)
                {
                    circ.Tangency1(j, out _par3sol[j], out _pararg3[j], out _pnttg3sol[j]);
                }
            }
            else if (rank3 == 2)
            {
                _theSame3[j] = i2;
                if (i2 == 0)
                {
                    circ.Tangency2(j, out _par3sol[j], out _pararg3[j], out _pnttg3sol[j]);
                }
            }
            else if (rank3 == 3)
            {
                _theSame3[j] = i3;
                if (i3 == 0)
                {
                    circ.Tangency3(j, out _par3sol[j], out _pararg3[j], out _pnttg3sol[j]);
                }
            }
        }
    }

    /// <summary>True if the construction succeeded (possibly with no solution).</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>The number of solutions.</summary>
    public int SolutionCount
    {
        get
        {
            return (int)_nbrSol;
        }
    }

    /// <summary>The solution of rank Index.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1 .. NbSolutions.</exception>
    public Circle2D ThisSolution(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        return _cirsol[index];
    }

    /// <summary>The qualifiers of the three arguments for the solution of rank Index.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1 .. NbSolutions.</exception>
    public void WhichQualifier(int index, out Qualifier qualif1, out Qualifier qualif2, out Qualifier qualif3)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            qualif1 = _qualifier1[index];
            qualif2 = _qualifier2[index];
            qualif3 = _qualifier3[index];
        }
    }

    /// <summary>
    /// The tangency point of the solution of rank Index on argument 1: its parameter on the solution, on the argument,
    /// and the point.
    /// </summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is argument 1.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1 .. NbSolutions.</exception>
    public void Tangency1(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            if (_theSame1[index] == 0)
            {
                parSol = _par1sol[index];
                parArg = _pararg1[index];
                pntSol = _pnttg1sol[index];
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
    public void Tangency2(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            if (_theSame2[index] == 0)
            {
                parSol = _par2sol[index];
                parArg = _pararg2[index];
                pntSol = _pnttg2sol[index];
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
    public void Tangency3(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            if (_theSame3[index] == 0)
            {
                parSol = _par3sol[index];
                parArg = _pararg3[index];
                pntSol = _pnttg3sol[index];
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
    public bool IsTheSame1(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        if (_theSame1[index] == 0)
        {
            return false;
        }
        return true;
    }

    /// <summary>True if the solution of rank Index is argument 2 (then it has no tangency point there).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1 .. NbSolutions.</exception>
    public bool IsTheSame2(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        if (_theSame2[index] == 0)
        {
            return false;
        }
        return true;
    }

    /// <summary>True if the solution of rank Index is argument 3 (then it has no tangency point there).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1 .. NbSolutions.</exception>
    public bool IsTheSame3(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        if (_theSame3[index] == 0)
        {
            return false;
        }
        return true;
    }
}
