// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanOn.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanOn.cxx
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
/// 2D circles tangent to two arguments (qualified curves or points) whose centres lie on a curve. Each solution comes
/// with the qualifiers of its arguments, its tangency points and its centre on the curve.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). Lines, circles and points are solved by GccAna_Circ2d2TanOn
/// (OnCurve a line or circle) or Geom2dGcc_Circ2d2TanOnGeo (OnCurve free-form); other arguments by
/// Geom2dGcc_Circ2d2TanOnIter, which gives at most one solution near the given parameters, its centre copied (OCCT left
/// CenterOn3 0 and the origin: Kerf decision 17106).
/// </remarks>
internal sealed class Circ2d2TanOn
{
    private bool _wellDone;
    private readonly NCollection_Array1<Circle2D> _cirsol;
    private int _nbrSol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<Qualifier> _qualifier2;
    private readonly NCollection_Array1<int> _theSame1;
    private readonly NCollection_Array1<int> _theSame2;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<Point2D> _pnttg2sol;
    private readonly NCollection_Array1<Point2D> _pntcen;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _par2sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<double> _pararg2;
    private readonly NCollection_Array1<double> _parcen3;
    private bool _invert;

    /// <summary>
    /// Circles tangent to two qualified curves whose centres lie on OnCurve. Lines and circles go to GccAna (all
    /// solutions) or Geom2dGcc_Circ2d2TanOnGeo (a free-form OnCurve); other curves to Geom2dGcc_Circ2d2TanOnIter (one
    /// solution, from the parameters Param1, Param2 and ParamOn on the arguments and OnCurve).
    /// </summary>
    public Circ2d2TanOn(in QualifiedCurve qualified1, in QualifiedCurve qualified2, CurveAdaptor2D onCurve, double tolerance, double param1, double param2, double paramOn)
    {
        _cirsol     = new(1, 8);
        _qualifier1 = new(1, 8);
        _qualifier2 = new(1, 8);
        _theSame1   = new(1, 8);
        _theSame2   = new(1, 8);
        _pnttg1sol  = new(1, 8);
        _pnttg2sol  = new(1, 8);
        _pntcen     = new(1, 8);
        _par1sol    = new(1, 8);
        _par2sol    = new(1, 8);
        _pararg1    = new(1, 8);
        _pararg2    = new(1, 8);
        _parcen3    = new(1, 8);
        _wellDone   = false;
        CurveAdaptor2D              C1    = qualified1.Qualified();
        CurveAdaptor2D              C2    = qualified2.Qualified();
        CurveKind                type1 = C1.GetType();
        CurveKind                type2 = C2.GetType();
        CurveKind                type3 = onCurve.GetType();
        Curve2D?  cc1   = C1.Curve();
        Curve2D?  cc2   = C2.Curve();
        Curve2D?  con   = onCurve.Curve();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        _invert = false;
        _nbrSol = 0;
        if ((type1 == Line || type1 == Circle)
                && (type2 == Line || type2 == Circle))
        {
            if (type3 == Line || type3 == Circle)
            {
                if (type1 == Circle)
                {
                    CircleCurve2D ccc1 = (cc1 as CircleCurve2D)!;
                    Circle2D                  c1 = ccc1.Circle;
                    QualifiedCirc       qc1 = new QualifiedCirc(c1, qualified1.Qualifier());
                    if (type2 == Circle)
                    {
                        CircleCurve2D ccc2 = (cc2 as CircleCurve2D)!;
                        Circle2D                  c2 = ccc2.Circle;
                        if (type3 == Circle)
                        {
                            CircleCurve2D cCon = (con as CircleCurve2D)!;
                            AnalyticCirc2d2TanOn        circAna = new(qc1,
                                                                                    new QualifiedCirc(c2, qualified2.Qualifier()),
                                                                                    cCon.Circle,
                                                                                    tolerance);
                            _wellDone = circAna.IsDone;
                            if (_wellDone)
                            {
                                _nbrSol = circAna.SolutionCount;
                                for (int i = 1; i <= _nbrSol; i++)
                                {
                                    circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                                }
                                Results(circAna);
                            }
                        }
                        else
                        {
                            LineCurve2D lLon = (con as LineCurve2D)!;
                            AnalyticCirc2d2TanOn      circAna = new(qc1,
                                                                                    new QualifiedCirc(c2, qualified2.Qualifier()),
                                                                                    lLon.Line,
                                                                                    tolerance);
                            _wellDone = circAna.IsDone;
                            if (_wellDone)
                            {
                                _nbrSol = circAna.SolutionCount;
                                for (int i = 1; i <= _nbrSol; i++)
                                {
                                    circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                                }
                                Results(circAna);
                            }
                        }
                    }
                    else
                    {
                        LineCurve2D ll2 = (cc2 as LineCurve2D)!;
                        Line2D                 l2 = ll2.Line;
                        if (type3 == Circle)
                        {
                            CircleCurve2D cCon = (con as CircleCurve2D)!;
                            AnalyticCirc2d2TanOn        circAna = new(qc1,
                                                                                    new QualifiedLin(l2, qualified2.Qualifier()),
                                                                                    cCon.Circle,
                                                                                    tolerance);
                            _wellDone = circAna.IsDone;
                            if (_wellDone)
                            {
                                _nbrSol = circAna.SolutionCount;
                                for (int i = 1; i <= _nbrSol; i++)
                                {
                                    circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                                }
                                Results(circAna);
                            }
                        }
                        else
                        {
                            LineCurve2D lLon = (con as LineCurve2D)!;
                            AnalyticCirc2d2TanOn      circAna = new(qc1,
                                                                                    new QualifiedLin(l2, qualified2.Qualifier()),
                                                                                    lLon.Line,
                                                                                    tolerance);
                            _wellDone = circAna.IsDone;
                            if (_wellDone)
                            {
                                _nbrSol = circAna.SolutionCount;
                                for (int i = 1; i <= _nbrSol; i++)
                                {
                                    circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                                }
                                Results(circAna);
                            }
                        }
                    }
                }
                else
                {
                    LineCurve2D ll1 = (cc1 as LineCurve2D)!;
                    Line2D                 l1 = ll1.Line;
                    QualifiedLin      ql1 = new QualifiedLin(l1, qualified1.Qualifier());
                    if (type2 == Circle)
                    {
                        CircleCurve2D ccc2 = (cc2 as CircleCurve2D)!;
                        Circle2D                  c2 = ccc2.Circle;
                        if (type3 == Circle)
                        {
                            CircleCurve2D cCon = (con as CircleCurve2D)!;
                            AnalyticCirc2d2TanOn        circAna = new(new QualifiedCirc(c2, qualified2.Qualifier()),
                                                                                    ql1,
                                                                                    cCon.Circle,
                                                                                    tolerance);
                            _wellDone = circAna.IsDone;
                            if (_wellDone)
                            {
                                _nbrSol = circAna.SolutionCount;
                                for (int i = 1; i <= _nbrSol; i++)
                                {
                                    circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                                }
                                Results(circAna);
                                _invert = true;
                            }
                        }
                        else
                        {
                            LineCurve2D lLon = (con as LineCurve2D)!;
                            AnalyticCirc2d2TanOn      circAna = new(new QualifiedCirc(c2, qualified2.Qualifier()),
                                                                                    ql1,
                                                                                    lLon.Line,
                                                                                    tolerance);
                            _wellDone = circAna.IsDone;
                            if (_wellDone)
                            {
                                _nbrSol = circAna.SolutionCount;
                                for (int i = 1; i <= _nbrSol; i++)
                                {
                                    circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                                }
                                Results(circAna);
                                _invert = true;
                            }
                        }
                    }
                    else
                    {
                        LineCurve2D ll2 = (cc2 as LineCurve2D)!;
                        Line2D                 l2 = ll2.Line;
                        if (type3 == Circle)
                        {
                            CircleCurve2D cCon = (con as CircleCurve2D)!;
                            AnalyticCirc2d2TanOn        circAna = new(ql1,
                                                                                    new QualifiedLin(l2, qualified2.Qualifier()),
                                                                                    cCon.Circle,
                                                                                    tolerance);
                            _wellDone = circAna.IsDone;
                            if (_wellDone)
                            {
                                _nbrSol = circAna.SolutionCount;
                                for (int i = 1; i <= _nbrSol; i++)
                                {
                                    circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                                }
                                Results(circAna);
                            }
                        }
                        else
                        {
                            LineCurve2D lLon = (con as LineCurve2D)!;
                            AnalyticCirc2d2TanOn      circAna = new(ql1,
                                                                                    new QualifiedLin(l2, qualified2.Qualifier()),
                                                                                    lLon.Line,
                                                                                    tolerance);
                            _wellDone = circAna.IsDone;
                            if (_wellDone)
                            {
                                _nbrSol = circAna.SolutionCount;
                                for (int i = 1; i <= _nbrSol; i++)
                                {
                                    circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                                }
                                Results(circAna);
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
                if (type1 == Circle)
                {
                    CircleCurve2D ccc1 = (cc1 as CircleCurve2D)!;
                    Circle2D                  c1 = ccc1.Circle;
                    QualifiedCirc       qc1 = new QualifiedCirc(c1, qualified1.Qualifier());
                    if (type2 == Circle)
                    {
                        CircleCurve2D ccc2 = (cc2 as CircleCurve2D)!;
                        Circle2D                  c2 = ccc2.Circle;
                        QualifiedCirc       qc2 = new QualifiedCirc(c2, qualified2.Qualifier());
                        Circ2d2TanOnGeo  circGeo = new(qc1, qc2, onCurve, tolerance);
                        _wellDone = circGeo.IsDone;
                        if (_wellDone)
                        {
                            _nbrSol = circGeo.SolutionCount;
                            for (int i = 1; i <= _nbrSol; i++)
                            {
                                circGeo.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                            }
                            Results(circGeo);
                        }
                    }
                    else
                    {
                        LineCurve2D  ll2 = (cc2 as LineCurve2D)!;
                        Line2D                  l2 = ll2.Line;
                        QualifiedLin       ql2 = new QualifiedLin(l2, qualified2.Qualifier());
                        Circ2d2TanOnGeo circGeo = new(qc1, ql2, onCurve, tolerance);
                        _wellDone = circGeo.IsDone;
                        if (_wellDone)
                        {
                            _nbrSol = circGeo.SolutionCount;
                            for (int i = 1; i <= _nbrSol; i++)
                            {
                                circGeo.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                            }
                            Results(circGeo);
                        }
                    }
                }
                else
                {
                    LineCurve2D ll1 = (cc1 as LineCurve2D)!;
                    Line2D                 l1 = ll1.Line;
                    QualifiedLin      ql1 = new QualifiedLin(l1, qualified1.Qualifier());
                    if (type2 == Circle)
                    {
                        CircleCurve2D ccc2 = (cc2 as CircleCurve2D)!;
                        Circle2D                  c2 = ccc2.Circle;
                        QualifiedCirc       qc2 = new QualifiedCirc(c2, qualified2.Qualifier());
                        Circ2d2TanOnGeo  circGeo = new(qc2, ql1, onCurve, tolerance);
                        _wellDone = circGeo.IsDone;
                        if (_wellDone)
                        {
                            _nbrSol = circGeo.SolutionCount;
                            for (int i = 1; i <= _nbrSol; i++)
                            {
                                circGeo.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                            }
                            Results(circGeo);
                            _invert = true;
                        }
                    }
                    else
                    {
                        LineCurve2D  ll2 = (cc2 as LineCurve2D)!;
                        Line2D                  l2 = ll2.Line;
                        QualifiedLin       ql2 = new QualifiedLin(l2, qualified2.Qualifier());
                        Circ2d2TanOnGeo circGeo = new(ql1, ql2, onCurve, tolerance);
                        _wellDone = circGeo.IsDone;
                        if (_wellDone)
                        {
                            _nbrSol = circGeo.SolutionCount;
                            for (int i = 1; i <= _nbrSol; i++)
                            {
                                circGeo.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                            }
                            Results(circGeo);
                        }
                    }
                }
            }
        }
        else
        {
            QCurve qc1 = new(C1, qualified1.Qualifier());
            QCurve qc2 = new(C2, qualified2.Qualifier());
            if ((type3 == Circle || type3 == Line))
            {
                if (type3 == Circle)
                {
                    CircleCurve2D cCon = (con as CircleCurve2D)!;
                    Circ2d2TanOnIter
                        circ = new(qc1, qc2, cCon.Circle, param1, param2, paramOn, tolerance);
                    _wellDone = circ.IsDone;
                    if (_wellDone)
                    {
                        _nbrSol    = 1;
                        _cirsol[1] = circ.ThisSolution();
                        if (circ.IsTheSame1())
                        {
                            _theSame1[1] = 1;
                        }
                        else
                        {
                            _theSame1[1] = 0;
                        }
                        if (circ.IsTheSame2)
                        {
                            _theSame2[1] = 1;
                        }
                        else
                        {
                            _theSame2[1] = 0;
                        }
                        // OCCT: no WhichQualifier here (the case below overwrites everything anyway)
                        circ.Tangency1(out _par1sol[1], out _pararg1[1], out _pnttg1sol[1]);
                        circ.Tangency2(out _par2sol[1], out _pararg2[1], out _pnttg2sol[1]);
                        circ.CenterOn3(out _parcen3[1], out _pntcen[1]);
                    }
                }
                else
                {
                    LineCurve2D lLon = (con as LineCurve2D)!;
                    Circ2d2TanOnIter
                        circ = new(qc1, qc2, lLon.Line, param1, param2, paramOn, tolerance);
                    _wellDone = circ.IsDone;
                    if (_wellDone)
                    {
                        _nbrSol    = 1;
                        _cirsol[1] = circ.ThisSolution();
                        if (circ.IsTheSame1())
                        {
                            _theSame1[1] = 1;
                        }
                        else
                        {
                            _theSame1[1] = 0;
                        }
                        if (circ.IsTheSame2)
                        {
                            _theSame2[1] = 1;
                        }
                        else
                        {
                            _theSame2[1] = 0;
                        }
                        circ.WhichQualifier(out _qualifier1[1], out _qualifier2[1]);
                        circ.Tangency1(out _par1sol[1], out _pararg1[1], out _pnttg1sol[1]);
                        circ.Tangency2(out _par2sol[1], out _pararg2[1], out _pnttg2sol[1]);
                        circ.CenterOn3(out _parcen3[1], out _pntcen[1]);
                    }
                }
            }
            // (Circ in OCCT: renamed, as C# forbids reusing the name of the inner blocks' locals.)
            // OCCT: no "else": this runs after the line and circle cases above too and replaces their result.
            Circ2d2TanOnIter circCu = new(qc1, qc2, onCurve, param1, param2, paramOn, tolerance);
            _wellDone = circCu.IsDone;
            if (_wellDone)
            {
                _nbrSol    = 1;
                _cirsol[1] = circCu.ThisSolution();
                if (circCu.IsTheSame1())
                {
                    _theSame1[1] = 1;
                }
                else
                {
                    _theSame1[1] = 0;
                }
                if (circCu.IsTheSame2)
                {
                    _theSame2[1] = 1;
                }
                else
                {
                    _theSame2[1] = 0;
                }
                circCu.WhichQualifier(out _qualifier1[1], out _qualifier2[1]);
                circCu.Tangency1(out _par1sol[1], out _pararg1[1], out _pnttg1sol[1]);
                circCu.Tangency2(out _par2sol[1], out _pararg2[1], out _pnttg2sol[1]);
                circCu.CenterOn3(out _parcen3[1], out _pntcen[1]);
            }
        }
    }

    /// <summary>
    /// Circles tangent to a qualified curve, through a point, whose centres lie on OnCurve (GccAna, Geo or Iter as
    /// for two curves; Param1 and ParamOn start the iteration).
    /// </summary>
    public Circ2d2TanOn(in QualifiedCurve qualified1, Point2 point, CurveAdaptor2D onCurve, double tolerance, double param1, double paramOn)
    {
        _cirsol     = new(1, 8);
        _qualifier1 = new(1, 8);
        _qualifier2 = new(1, 8);
        _theSame1   = new(1, 8);
        _theSame2   = new(1, 8);
        _pnttg1sol  = new(1, 8);
        _pnttg2sol  = new(1, 8);
        _pntcen     = new(1, 8);
        _par1sol    = new(1, 8);
        _par2sol    = new(1, 8);
        _pararg1    = new(1, 8);
        _pararg2    = new(1, 8);
        _parcen3    = new(1, 8);
        _wellDone   = false;
        CurveAdaptor2D              C1    = qualified1.Qualified();
        CurveKind                type1 = C1.GetType();
        CurveKind                type3 = onCurve.GetType();
        Curve2D?  cc1   = C1.Curve();
        Curve2D?  con   = onCurve.Curve();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        _invert = false;
        _nbrSol = 0;
        if (type1 == Line || type1 == Circle)
        {
            if (type3 == Line || type3 == Circle)
            {
                Point2D pnt = point.Pnt2d();
                if (type1 == Circle)
                {
                    CircleCurve2D ccc1 = (cc1 as CircleCurve2D)!;
                    Circle2D                  c1 = ccc1.Circle;
                    QualifiedCirc       qc1 = new(c1, qualified1.Qualifier());
                    if (type3 == Circle)
                    {
                        CircleCurve2D cCon = (con as CircleCurve2D)!;
                        AnalyticCirc2d2TanOn        circAna = new(qc1, pnt, cCon.Circle, tolerance);
                        _wellDone = circAna.IsDone;
                        if (_wellDone)
                        {
                            _nbrSol = circAna.SolutionCount;
                            for (int i = 1; i <= _nbrSol; i++)
                            {
                                circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                            }
                            Results(circAna);
                        }
                    }
                    else if (type3 == Line)
                    {
                        LineCurve2D cCon = (con as LineCurve2D)!;
                        AnalyticCirc2d2TanOn      circAna = new(qc1, pnt, cCon.Line, tolerance);
                        _wellDone = circAna.IsDone;
                        if (_wellDone)
                        {
                            _nbrSol = circAna.SolutionCount;
                            for (int i = 1; i <= _nbrSol; i++)
                            {
                                circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                            }
                            Results(circAna);
                        }
                    }
                }
                else
                {
                    LineCurve2D lll1 = (cc1 as LineCurve2D)!;
                    Line2D                 l1 = lll1.Line;
                    QualifiedLin      ql1 = new(l1, qualified1.Qualifier());
                    if (type3 == Circle)
                    {
                        CircleCurve2D cCon = (con as CircleCurve2D)!;
                        AnalyticCirc2d2TanOn        circAna = new(ql1, pnt, cCon.Circle, tolerance);
                        _wellDone = circAna.IsDone;
                        if (_wellDone)
                        {
                            _nbrSol = circAna.SolutionCount;
                            for (int i = 1; i <= _nbrSol; i++)
                            {
                                circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                            }
                            Results(circAna);
                        }
                    }
                    else if (type3 == Line)
                    {
                        LineCurve2D lLon = (con as LineCurve2D)!;
                        AnalyticCirc2d2TanOn      circAna = new(ql1, pnt, lLon.Line, tolerance);
                        _wellDone = circAna.IsDone;
                        if (_wellDone)
                        {
                            _nbrSol = circAna.SolutionCount;
                            for (int i = 1; i <= _nbrSol; i++)
                            {
                                circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                            }
                            Results(circAna);
                        }
                    }
                }
            }
            //=============================================================================
            //                            Appel a GccGeo.                                 +
            //=============================================================================

            else
            {
                if (type1 == Circle)
                {
                    CircleCurve2D ccc1 = (cc1 as CircleCurve2D)!;
                    Circle2D                  c1 = ccc1.Circle;
                    QualifiedCirc       qc1 = new(c1, qualified1.Qualifier());
                    Circ2d2TanOnGeo  circGeo = new(qc1, point.Pnt2d(), onCurve, tolerance);
                    _wellDone = circGeo.IsDone;
                    if (_wellDone)
                    {
                        _nbrSol = circGeo.SolutionCount;
                        for (int i = 1; i <= _nbrSol; i++)
                        {
                            circGeo.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                        }
                        Results(circGeo);
                    }
                }
                else
                {
                    LineCurve2D  lll1 = (cc1 as LineCurve2D)!;
                    Line2D                  l1 = lll1.Line;
                    QualifiedLin       ql1 = new(l1, qualified1.Qualifier());
                    Circ2d2TanOnGeo circGeo = new(ql1, point.Pnt2d(), onCurve, tolerance);
                    _wellDone = circGeo.IsDone;
                    if (_wellDone)
                    {
                        _nbrSol = circGeo.SolutionCount;
                        for (int i = 1; i <= _nbrSol; i++)
                        {
                            circGeo.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                        }
                        Results(circGeo);
                    }
                }
            }
        }
        else
        {
            QCurve qc1 = new(C1, qualified1.Qualifier());
            if ((type3 == Circle || type3 == Line))
            {
                if (type3 == Circle)
                {
                    CircleCurve2D cCon = (con as CircleCurve2D)!;
                    Circ2d2TanOnIter circ = new(qc1,
                                                                                    point.Pnt2d(),
                                                                                    cCon.Circle,
                                                                                    param1,
                                                                                    paramOn,
                                                                                    tolerance);
                    _wellDone = circ.IsDone;
                    if (_wellDone)
                    {
                        _nbrSol    = 1;
                        _cirsol[1] = circ.ThisSolution();
                        if (circ.IsTheSame1())
                        {
                            _theSame1[1] = 1;
                        }
                        else
                        {
                            _theSame1[1] = 0;
                        }
                        _theSame2[1] = 0;
                        circ.WhichQualifier(out _qualifier1[1], out _qualifier2[1]);
                        circ.Tangency1(out _par1sol[1], out _pararg1[1], out _pnttg1sol[1]);
                        circ.Tangency2(out _par2sol[1], out _pararg2[1], out _pnttg2sol[1]);
                        circ.CenterOn3(out _parcen3[1], out _pntcen[1]);
                    }
                }
                else
                {
                    LineCurve2D   lLon = (con as LineCurve2D)!;
                    Circ2d2TanOnIter circ = new(qc1,
                                                                                    point.Pnt2d(),
                                                                                    lLon.Line,
                                                                                    param1,
                                                                                    paramOn,
                                                                                    tolerance);
                    _wellDone = circ.IsDone;
                    if (_wellDone)
                    {
                        _nbrSol    = 1;
                        _cirsol[1] = circ.ThisSolution();
                        if (circ.IsTheSame1())
                        {
                            _theSame1[1] = 1;
                        }
                        else
                        {
                            _theSame1[1] = 0;
                        }
                        _theSame2[1] = 0;
                        circ.WhichQualifier(out _qualifier1[1], out _qualifier2[1]);
                        circ.Tangency1(out _par1sol[1], out _pararg1[1], out _pnttg1sol[1]);
                        circ.Tangency2(out _par2sol[1], out _pararg2[1], out _pnttg2sol[1]);
                        circ.CenterOn3(out _parcen3[1], out _pntcen[1]);
                    }
                }
            }
            else
            {
                Circ2d2TanOnIter circ = new(qc1, point.Pnt2d(), onCurve, param1, paramOn, tolerance);
                _wellDone = circ.IsDone;
                if (_wellDone)
                {
                    _nbrSol    = 1;
                    _cirsol[1] = circ.ThisSolution();
                    if (circ.IsTheSame1())
                    {
                        _theSame1[1] = 1;
                    }
                    else
                    {
                        _theSame1[1] = 0;
                    }
                    if (circ.IsTheSame2)
                    {
                        _theSame2[1] = 1;
                    }
                    else
                    {
                        _theSame2[1] = 0;
                    }
                    circ.WhichQualifier(out _qualifier1[1], out _qualifier2[1]);
                    circ.Tangency1(out _par1sol[1], out _pararg1[1], out _pnttg1sol[1]);
                    circ.Tangency2(out _par2sol[1], out _pararg2[1], out _pnttg2sol[1]);
                    circ.CenterOn3(out _parcen3[1], out _pntcen[1]);
                }
            }
        }
    }

    /// <summary>Circles through two points whose centres lie on OnCurve (GccAna, or Geo for a free-form OnCurve).</summary>
    public Circ2d2TanOn(Point2 point1, Point2 point2, CurveAdaptor2D onCurve, double tolerance)
    {
        _cirsol     = new(1, 8);
        _qualifier1 = new(1, 8);
        _qualifier2 = new(1, 8);
        _theSame1   = new(1, 8);
        _theSame2   = new(1, 8);
        _pnttg1sol  = new(1, 8);
        _pnttg2sol  = new(1, 8);
        _pntcen     = new(1, 8);
        _par1sol    = new(1, 8);
        _par2sol    = new(1, 8);
        _pararg1    = new(1, 8);
        _pararg2    = new(1, 8);
        _parcen3    = new(1, 8);
        _wellDone   = false;
        CurveKind                type3 = onCurve.GetType();
        Curve2D?  con   = onCurve.Curve();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        _invert = false;
        _nbrSol = 0;
        if (type3 == Line || type3 == Circle)
        {
            Point2D pnt1 = point1.Pnt2d();
            Point2D pnt2 = point2.Pnt2d();
            if (type3 == Circle)
            {
                CircleCurve2D cCon = (con as CircleCurve2D)!;
                AnalyticCirc2d2TanOn        circAna = new(pnt1, pnt2, cCon.Circle, tolerance);
                _wellDone = circAna.IsDone;
                if (_wellDone)
                {
                    _nbrSol = circAna.SolutionCount;
                    for (int i = 1; i <= _nbrSol; i++)
                    {
                        circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                    }
                    Results(circAna);
                }
            }
            else
            {
                LineCurve2D lLon = (con as LineCurve2D)!;
                AnalyticCirc2d2TanOn      circAna = new(pnt1, pnt2, lLon.Line, tolerance);
                _wellDone = circAna.IsDone;
                if (_wellDone)
                {
                    _nbrSol = circAna.SolutionCount;
                    for (int i = 1; i <= _nbrSol; i++)
                    {
                        circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                    }
                    Results(circAna);
                }
            }
        }

        //=============================================================================
        //                            Appel a GccGeo.                                 +
        //=============================================================================

        else
        {
            Circ2d2TanOnGeo circGeo = new(point1.Pnt2d(), point2.Pnt2d(), onCurve, tolerance);
            _wellDone = circGeo.IsDone;
            if (_wellDone)
            {
                _nbrSol = circGeo.SolutionCount;
                for (int i = 1; i <= _nbrSol; i++)
                {
                    circGeo.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                }
                Results(circGeo);
            }
        }
    }

    internal void Results(AnalyticCirc2d2TanOn circ)
    {
        for (int j = 1; j <= _nbrSol; j++)
        {
            _cirsol[j] = circ.ThisSolution(j);
            if (circ.IsTheSame1(j))
            {
                _theSame1[j] = 1;
            }
            else
            {
                _theSame1[j] = 0;
            }
            if (circ.IsTheSame2(j))
            {
                _theSame2[j] = 1;
            }
            else
            {
                _theSame2[j] = 0;
            }
            circ.WhichQualifier(j, out _qualifier1[j], out _qualifier2[j]);
            circ.Tangency1(j, out _par1sol[j], out _pararg1[j], out _pnttg1sol[j]);
            circ.Tangency2(j, out _par2sol[j], out _pararg2[j], out _pnttg2sol[j]);
            circ.CenterOn3(j, out _parcen3[j], out _pntcen[j]);
        }
    }

    internal void Results(Circ2d2TanOnGeo circ)
    {
        for (int j = 1; j <= _nbrSol; j++)
        {
            _cirsol[j] = circ.ThisSolution(j);
            if (circ.IsTheSame1(j))
            {
                _theSame1[j] = 1;
            }
            else
            {
                _theSame1[j] = 0;
            }
            if (circ.IsTheSame2(j))
            {
                _theSame2[j] = 1;
            }
            else
            {
                _theSame2[j] = 0;
            }
            circ.WhichQualifier(j, out _qualifier1[j], out _qualifier2[j]);
            circ.Tangency1(j, out _par1sol[j], out _pararg1[j], out _pnttg1sol[j]);
            circ.Tangency2(j, out _par2sol[j], out _pararg2[j], out _pnttg2sol[j]);
            circ.CenterOn3(j, out _parcen3[j], out _pntcen[j]);
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
            return _nbrSol;
        }
    }

    /// <summary>Solution Index.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
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

    /// <summary>The qualifiers of solution Index relative to the two arguments.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
    public void WhichQualifier(int index, out Qualifier qualif1, out Qualifier qualif2)
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
            if (_invert)
            {
                qualif1 = _qualifier2[index];
                qualif2 = _qualifier1[index];
            }
            else
            {
                qualif1 = _qualifier1[index];
                qualif2 = _qualifier2[index];
            }
        }
    }

    /// <summary>The tangency point of solution Index on the first argument, with its parameters on both.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the first argument.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
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
            if (_invert)
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
    }

    /// <summary>The tangency point of solution Index on the second argument, with its parameters on both.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the second argument.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
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
            if (!_invert)
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
    }

    /// <summary>The centre of solution Index and its parameter on the centre's curve.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
    public void CenterOn3(int index, out double parArg, out Point2D pntSol)
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
            parArg = _parcen3[index];
            pntSol = _pntcen[index];
        }
    }

    /// <summary>True if solution Index is the first argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
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

        if (_invert)
        {
            return _theSame2[index] != 0;
        }
        else
        {
            return _theSame1[index] != 0;
        }
    }

    /// <summary>True if solution Index is the second argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
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

        if (!_invert)
        {
            return _theSame2[index] != 0;
        }
        else
        {
            return _theSame1[index] != 0;
        }
    }
}
