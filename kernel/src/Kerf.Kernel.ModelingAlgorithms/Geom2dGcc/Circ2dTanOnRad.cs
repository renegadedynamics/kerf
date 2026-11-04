// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanOnRad.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanOnRad.cxx
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
/// 2D circles of a given radius tangent to a qualified curve (or through a point) whose centre lies on a curve. Lines
/// and circles go to GccAna_Circ2dTanOnRad, other curves to Geom2dGcc_Circ2dTanOnRadGeo.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). Up to 8 solutions.</remarks>
internal sealed class Circ2dTanOnRad
{
    private readonly bool _wellDone;
    private readonly int _nbrSol;
    private readonly NCollection_Array1<Circle2D> _cirsol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<int> _theSame1;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<Point2D> _pntcen3;
    private readonly NCollection_Array1<double> _parcen3;

    // The fields' initialiser list.
    private Circ2dTanOnRad(int nbSol)
    {
        _cirsol     = new(1, nbSol);
        _qualifier1 = new(1, nbSol);
        _theSame1   = new(1, nbSol);
        _pnttg1sol  = new(1, nbSol);
        _par1sol    = new(1, nbSol);
        _pararg1    = new(1, nbSol);
        _pntcen3    = new(1, nbSol);
        _parcen3    = new(1, nbSol);
    }

    /// <summary>Circles of radius Radius tangent to the qualified curve, centred on OnCurve.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Circ2dTanOnRad(in QualifiedCurve qualified1, CurveAdaptor2D onCurve, double radius,
                                    double tolerance)
        : this(8)
    {
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            CurveAdaptor2D C1    = qualified1.Qualified();
            CurveKind   type1 = C1.GetType();
            CurveKind   type2 = onCurve.GetType();
            Curve2D?       cc1   = C1.Curve();
            Curve2D?       Con   = onCurve.Curve();

            //=============================================================================
            //                            Appel a GccAna.                                 +
            //=============================================================================

            _nbrSol = 0;
            if ((type1 == Line || type1 == Circle)
                && (type2 == Line || type2 == Circle))
            {
                if (type1 == Circle)
                {
                    CircleCurve2D        ccc1 = (cc1 as CircleCurve2D)!;
                    Circle2D            c1   = ccc1.Circle;
                    QualifiedCirc qc1  = new(c1, qualified1.Qualifier());
                    if (type2 == Circle)
                    {
                        CircleCurve2D         cCon = (Con as CircleCurve2D)!;
                        Circle2D             con  = cCon.Circle;
                        AnalyticCirc2dTanOnRad circ = new(qc1, con, radius, tolerance);
                        _wellDone                   = circ.IsDone;
                        _nbrSol                     = circ.SolutionCount;
                        Results(circ);
                    }
                    else
                    {
                        LineCurve2D           lLon = (Con as LineCurve2D)!;
                        Line2D              lon  = lLon.Line;
                        AnalyticCirc2dTanOnRad circ = new(qc1, lon, radius, tolerance);
                        _wellDone                   = circ.IsDone;
                        _nbrSol                     = circ.SolutionCount;
                        Results(circ);
                    }
                }
                else
                {
                    LineCurve2D         ll1 = (cc1 as LineCurve2D)!;
                    Line2D            l1  = ll1.Line;
                    QualifiedLin ql1 = new(l1, qualified1.Qualifier());
                    if (type2 == Circle)
                    {
                        CircleCurve2D         cCon = (Con as CircleCurve2D)!;
                        Circle2D             con  = cCon.Circle;
                        AnalyticCirc2dTanOnRad circ = new(ql1, con, radius, tolerance);
                        _wellDone                   = circ.IsDone;
                        _nbrSol                     = circ.SolutionCount;
                        Results(circ);
                    }
                    else
                    {
                        LineCurve2D           lLon = (Con as LineCurve2D)!;
                        Line2D              lon  = lLon.Line;
                        AnalyticCirc2dTanOnRad circ = new(ql1, lon, radius, tolerance);
                        _wellDone                   = circ.IsDone;
                        _nbrSol                     = circ.SolutionCount;
                        Results(circ);
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
                    CircleCurve2D               ccc1    = (cc1 as CircleCurve2D)!;
                    Circle2D                   c1      = ccc1.Circle;
                    QualifiedCirc        qc1     = new(c1, qualified1.Qualifier());
                    Circ2dTanOnRadGeo circGeo = new(qc1, onCurve, radius, tolerance);
                    _wellDone                            = circGeo.IsDone;
                    _nbrSol                              = circGeo.SolutionCount;
                    Results(circGeo);
                }
                else if (type1 == Line)
                {
                    LineCurve2D                 ll1     = (cc1 as LineCurve2D)!;
                    Line2D                    l1      = ll1.Line;
                    QualifiedLin         ql1     = new(l1, qualified1.Qualifier());
                    Circ2dTanOnRadGeo circGeo = new(ql1, onCurve, radius, tolerance);
                    _wellDone                            = circGeo.IsDone;
                    _nbrSol                              = circGeo.SolutionCount;
                    Results(circGeo);
                }
                else
                {
                    QCurve            qc1     = new(C1, qualified1.Qualifier());
                    Circ2dTanOnRadGeo circGeo = new(qc1, onCurve, radius, tolerance);
                    _wellDone                            = circGeo.IsDone;
                    _nbrSol                              = circGeo.SolutionCount;
                    Results(circGeo);
                }
            }
        }
    }

    /// <summary>Circles of radius Radius through Point1, centred on OnCurve.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Circ2dTanOnRad(Point2 Point1, CurveAdaptor2D onCurve, double radius, double tolerance)
        : this(8)
    {
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            Point2D          point1 = Point1.Pnt2d();
            CurveKind type2  = onCurve.GetType();
            Curve2D?     Con    = onCurve.Curve();

            //=============================================================================
            //                            Appel a GccAna.                                 +
            //=============================================================================

            _nbrSol = 0;
            if (type2 == Line || type2 == Circle)
            {
                if (type2 == Circle)
                {
                    CircleCurve2D         cCon = (Con as CircleCurve2D)!;
                    Circle2D             con  = cCon.Circle;
                    AnalyticCirc2dTanOnRad circ = new(point1, con, radius, tolerance);
                    _wellDone                   = circ.IsDone;
                    _nbrSol                     = circ.SolutionCount;
                    Results(circ);
                }
                else
                {
                    LineCurve2D           lLon = (Con as LineCurve2D)!;
                    Line2D              lon  = lLon.Line;
                    AnalyticCirc2dTanOnRad circ = new(point1, lon, radius, tolerance);
                    _wellDone                   = circ.IsDone;
                    _nbrSol                     = circ.SolutionCount;
                    Results(circ);
                }
            }

            //=============================================================================
            //                            Appel a GccGeo.                                 +
            //=============================================================================

            else
            {
                Circ2dTanOnRadGeo circGeo = new(point1, onCurve, radius, tolerance);
                _wellDone                            = circGeo.IsDone;
                _nbrSol                              = circGeo.SolutionCount;
                Results(circGeo);
            }
        }
    }

    /// <summary>Copies the analytic solver's solutions (OCCT's public Results; it writes this solver).</summary>
    internal void Results(AnalyticCirc2dTanOnRad circ)
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
            circ.Tangency1(j, out _par1sol[j], out _pararg1[j], out _pnttg1sol[j]);
            circ.CenterOn3(j, out _parcen3[j], out _pntcen3[j]);
            circ.WhichQualifier(j, out _qualifier1[j]);
        }
    }

    /// <summary>Copies the geometric solver's solutions (OCCT's public Results; it writes this solver).</summary>
    internal void Results(Circ2dTanOnRadGeo circ)
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
            circ.Tangency1(j, out _par1sol[j], out _pararg1[j], out _pnttg1sol[j]);
            circ.CenterOn3(j, out _parcen3[j], out _pntcen3[j]);
            circ.WhichQualifier(j, out _qualifier1[j]);
        }
    }

    /// <summary>True if the construction succeeded.</summary>
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

    /// <summary>The solution of index Index.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1..NbSolutions().</exception>
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

    /// <summary>The qualifier of the argument for the solution of index Index.</summary>
    public void WhichQualifier(int index, out Qualifier qualif1)
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
        }
    }

    /// <summary>The tangency point between the solution of index Index and the argument, and its parameters.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the argument itself.</exception>
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

    /// <summary>The centre of the solution of index Index and its parameter on the curve it lies on.</summary>
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
            pntSol = _pntcen3[index];
        }
    }

    /// <summary>True if the solution of index Index is the argument itself.</summary>
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
}
