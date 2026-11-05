// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanRad.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// 2D circles of a given radius tangent to two arguments, each a qualified curve or a point (passed through). Lines
/// and circles go to GccAna_Circ2d2TanRad; a free-form argument to Geom2dGcc_Circ2d2TanRadGeo (the intersections of
/// the arguments' offsets). Each solution comes with the qualifiers of its arguments and its tangency points.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117), up to 16 solutions (2 for two points). Where the
/// second argument is the line or circle and the first is not, the solver is called with the arguments swapped and
/// the accessors swap them back.
/// </remarks>
internal sealed class Circ2d2TanRad
{
    private const int ANbSolMAX = 16;

    private bool _wellDone;
    private readonly NCollection_Array1<Circle2D> _cirsol;
    private int _nbrSol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<Qualifier> _qualifier2;
    private readonly NCollection_Array1<int> _theSame1;
    private readonly NCollection_Array1<int> _theSame2;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<Point2D> _pnttg2sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _par2sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<double> _pararg2;
    private bool _invert;

    // The arrays, sized as each constructor's initialiser list sizes them.
    private Circ2d2TanRad(int nbSol)
    {
        _cirsol     = new(1, nbSol);
        _qualifier1 = new(1, nbSol);
        _qualifier2 = new(1, nbSol);
        _theSame1   = new(1, nbSol);
        _theSame2   = new(1, nbSol);
        _pnttg1sol  = new(1, nbSol);
        _pnttg2sol  = new(1, nbSol);
        _par1sol    = new(1, nbSol);
        _par2sol    = new(1, nbSol);
        _pararg1    = new(1, nbSol);
        _pararg2    = new(1, nbSol);
    }

    // circulaire tangent a deux cercles et de rayon donne
    //====================================================
    //========================================================================
    // On initialise WellDone a false.                                       +
    // On recupere le cercle C1 et le cercle C2.                             +
    // On sort en erreur dans les cas ou la construction est impossible.     +
    // On distingue les cas limites pour les triater separement.             +
    // On fait la parallele a C1 dans le bon sens.                           +
    // On fait la parallele a C2 dans le bon sens.                           +
    // On intersecte les paralleles ==> point de centre de la solution.      +
    // On cree la solution qu on ajoute aux solutions deja trouvees.         +
    // On remplit les champs.                                                +
    //========================================================================

    /// <summary>Circles of radius Radius tangent to two qualified curves.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    /// <exception cref="BadQualifierException">A line is qualified enclosing.</exception>
    public Circ2d2TanRad(in QualifiedCurve qualified1,
                                   in QualifiedCurve qualified2,
                                   double radius,
                                   double tolerance)
        : this(ANbSolMAX)
    {
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            CurveAdaptor2D C1    = qualified1.Qualified();
            CurveAdaptor2D C2    = qualified2.Qualified();
            Curve2D?       cc1   = C1.Curve();
            Curve2D?       cc2   = C2.Curve();
            CurveKind   type1 = C1.GetType();
            CurveKind   type2 = C2.GetType();

            //=============================================================================
            //                            Appel a GccAna.                                 +
            //=============================================================================

            _invert = false;
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
                        CircleCurve2D        ccc2    = (cc2 as CircleCurve2D)!;
                        Circle2D            c2      = ccc2.Circle;
                        AnalyticCirc2d2TanRad circAna = new(qc1, new QualifiedCirc(c2, qualified2.Qualifier()), radius, tolerance);
                        _wellDone = circAna.IsDone;
                        _nbrSol   = circAna.SolutionCount;
                        for (int i = 1; i <= _nbrSol; i++)
                        {
                            circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                        }
                        Results(circAna);
                    }
                    else
                    {
                        LineCurve2D ll2 = (cc2 as LineCurve2D)!;
                        Line2D    l2  = ll2.Line;
                        if (!qualified2.IsEnclosing)
                        {
                            AnalyticCirc2d2TanRad circAna = new(qc1, new QualifiedLin(l2, qualified2.Qualifier()), radius, tolerance);
                            _wellDone = circAna.IsDone;
                            _nbrSol   = circAna.SolutionCount;
                            for (int i = 1; i <= _nbrSol; i++)
                            {
                                circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                            }
                            Results(circAna);
                        }
                        else
                        {
                            _wellDone = false;
                            throw new BadQualifierException();
                        }
                    }
                }
                else
                {
                    LineCurve2D ll1 = (cc1 as LineCurve2D)!;
                    Line2D    l1  = ll1.Line;
                    if (qualified1.IsEnclosing)
                    {
                        _wellDone = false;
                        throw new BadQualifierException();
                    }
                    else
                    {
                        QualifiedLin ql1 = new(l1, qualified1.Qualifier());
                        if (type2 == Circle)
                        {
                            CircleCurve2D ccc2 = (cc2 as CircleCurve2D)!;
                            Circle2D     c2   = ccc2.Circle;
                            _invert             = true;
                            AnalyticCirc2d2TanRad circAna = new(new QualifiedCirc(c2, qualified2.Qualifier()), ql1, radius, tolerance);
                            _wellDone = circAna.IsDone;
                            _nbrSol   = circAna.SolutionCount;
                            for (int i = 1; i <= _nbrSol; i++)
                            {
                                circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                            }
                            Results(circAna);
                        }
                        else
                        {
                            LineCurve2D ll2 = (cc2 as LineCurve2D)!;
                            Line2D    l2  = ll2.Line;
                            if (!qualified2.IsEnclosing)
                            {
                                AnalyticCirc2d2TanRad circAna = new(ql1, new QualifiedLin(l2, qualified2.Qualifier()), radius, tolerance);
                                _wellDone = circAna.IsDone;
                                _nbrSol   = circAna.SolutionCount;
                                for (int i = 1; i <= _nbrSol; i++)
                                {
                                    circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                                }
                                Results(circAna);
                            }
                            else
                            {
                                _wellDone = false;
                                throw new BadQualifierException();
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
                if (type1 == Line)
                {
                    LineCurve2D ll1 = (cc1 as LineCurve2D)!;
                    Line2D    l1  = ll1.Line;
                    if (qualified1.IsEnclosing)
                    {
                        _wellDone = false;
                        throw new BadQualifierException();
                    }
                    else
                    {
                        QualifiedLin        ql1     = new(l1, qualified1.Qualifier());
                        QCurve           qc2     = new(C2, qualified2.Qualifier());
                        Circ2d2TanRadGeo circGeo = new(ql1, qc2, radius, tolerance);
                        _wellDone = circGeo.IsDone;
                        _nbrSol   = circGeo.SolutionCount;
                        for (int i = 1; i <= _nbrSol; i++)
                        {
                            circGeo.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                        }
                        Results(circGeo);
                    }
                }
                else if (type1 == Circle)
                {
                    CircleCurve2D              ccc1    = (cc1 as CircleCurve2D)!;
                    Circle2D                  c1      = ccc1.Circle;
                    QualifiedCirc       qc1     = new(c1, qualified1.Qualifier());
                    QCurve           qc2     = new(C2, qualified2.Qualifier());
                    Circ2d2TanRadGeo circGeo = new(qc1, qc2, radius, tolerance);
                    _wellDone = circGeo.IsDone;
                    _nbrSol   = circGeo.SolutionCount;
                    for (int i = 1; i <= _nbrSol; i++)
                    {
                        circGeo.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                    }
                    Results(circGeo);
                }
                else if (type2 == Line)
                {
                    _invert          = true;
                    LineCurve2D ll2 = (cc2 as LineCurve2D)!;
                    Line2D    l2  = ll2.Line;
                    if (qualified2.IsEnclosing)
                    {
                        _wellDone = false;
                        throw new BadQualifierException();
                    }
                    else
                    {
                        QualifiedLin        ql2     = new(l2, qualified2.Qualifier());
                        QCurve           qc1     = new(C1, qualified1.Qualifier());
                        Circ2d2TanRadGeo circGeo = new(ql2, qc1, radius, tolerance);
                        _wellDone = circGeo.IsDone;
                        _nbrSol   = circGeo.SolutionCount;
                        for (int i = 1; i <= _nbrSol; i++)
                        {
                            circGeo.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                        }
                        Results(circGeo);
                    }
                }
                else if (type2 == Circle)
                {
                    _invert                             = true;
                    CircleCurve2D              ccc2    = (cc2 as CircleCurve2D)!;
                    Circle2D                  c2      = ccc2.Circle;
                    QualifiedCirc       qc2     = new(c2, qualified2.Qualifier());
                    QCurve           qc1     = new(C1, qualified1.Qualifier());
                    Circ2d2TanRadGeo circGeo = new(qc2, qc1, radius, tolerance);
                    _wellDone = circGeo.IsDone;
                    _nbrSol   = circGeo.SolutionCount;
                    for (int i = 1; i <= _nbrSol; i++)
                    {
                        circGeo.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                    }
                    Results(circGeo);
                }
                else
                {
                    QCurve           qc1     = new(C1, qualified1.Qualifier());
                    QCurve           qc2     = new(C2, qualified2.Qualifier());
                    Circ2d2TanRadGeo circGeo = new(qc1, qc2, radius, tolerance);
                    _wellDone = circGeo.IsDone;
                    _nbrSol   = circGeo.SolutionCount;
                    for (int i = 1; i <= _nbrSol; i++)
                    {
                        circGeo.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                    }
                    Results(circGeo);
                }
            }
        }
    }

    /// <summary>Circles of radius Radius tangent to a qualified curve and passing through a point.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Circ2d2TanRad(in QualifiedCurve qualified1, Point2 point, double radius, double tolerance)
        : this(ANbSolMAX)
    {
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            CurveAdaptor2D C1    = qualified1.Qualified();
            Curve2D?       cc1   = C1.Curve();
            CurveKind   type1 = C1.GetType();

            //=============================================================================
            //                            Appel a GccAna.                                 +
            //=============================================================================

            _invert = false;
            _nbrSol = 0;
            if (type1 == Line || type1 == Circle)
            {
                if (type1 == Circle)
                {
                    CircleCurve2D        ccc1    = (cc1 as CircleCurve2D)!;
                    Circle2D            c1      = ccc1.Circle;
                    QualifiedCirc qc1     = new(c1, qualified1.Qualifier());
                    AnalyticCirc2d2TanRad circAna = new(qc1, point.Pnt2d(), radius, tolerance);
                    _wellDone = circAna.IsDone;
                    _nbrSol   = circAna.SolutionCount;
                    for (int i = 1; i <= _nbrSol; i++)
                    {
                        circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                    }
                    Results(circAna);
                }
                else
                {
                    LineCurve2D          lll1    = (cc1 as LineCurve2D)!;
                    Line2D             l1      = lll1.Line;
                    QualifiedLin  ql1     = new(l1, qualified1.Qualifier());
                    AnalyticCirc2d2TanRad circAna = new(ql1, point.Pnt2d(), radius, tolerance);
                    _wellDone = circAna.IsDone;
                    _nbrSol   = circAna.SolutionCount;
                    for (int i = 1; i <= _nbrSol; i++)
                    {
                        circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                    }
                    Results(circAna);
                }
            }
            //=============================================================================
            //                            Appel a GccGeo.                                 +
            //=============================================================================
            else
            {
                QCurve           qc1     = new(C1, qualified1.Qualifier());
                Circ2d2TanRadGeo circGeo = new(qc1, point.Pnt2d(), radius, tolerance);
                _wellDone = circGeo.IsDone;
                _nbrSol   = circGeo.SolutionCount;
                for (int i = 1; i <= _nbrSol; i++)
                {
                    circGeo.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                }
                Results(circGeo);
            }
        }
    }

    /// <summary>Circles of radius Radius passing through two points.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Circ2d2TanRad(Point2 point1, Point2 point2, double radius, double tolerance)
        : this(2)
    {
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {

            //=============================================================================
            //                            Appel a GccAna.                                 +
            //=============================================================================

            _invert = false;
            _nbrSol = 0;
            AnalyticCirc2d2TanRad circAna = new(point1.Pnt2d(), point2.Pnt2d(), radius, tolerance);
            _wellDone = circAna.IsDone;
            _nbrSol   = circAna.SolutionCount;
            for (int i = 1; i <= _nbrSol; i++)
            {
                circAna.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
            }
            Results(circAna);
        }
    }

    /// <summary>Copies the analytic solver's solutions (OCCT's public Results; it writes the solver, so internal).</summary>
    internal void Results(AnalyticCirc2d2TanRad circ)
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
            // a solution that is an argument has no tangency point with it (OCCT asked GccAna for it, which threw
            // StdFail_NotDone out of the constructor: Kerf decision 1545)
            if (_theSame1[j] == 0)
            {
                circ.Tangency1(j, out _par1sol[j], out _pararg1[j], out _pnttg1sol[j]);
            }
            if (_theSame2[j] == 0)
            {
                circ.Tangency2(j, out _par2sol[j], out _pararg2[j], out _pnttg2sol[j]);
            }
        }
    }

    /// <summary>Copies the geometric solver's solutions (OCCT's public Results; it writes the solver, so internal).</summary>
    internal void Results(Circ2d2TanRadGeo circ)
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
            circ.Tangency1(j, out _par1sol[j], out _pararg1[j], out _pnttg1sol[j]);
            circ.Tangency2(j, out _par2sol[j], out _pararg2[j], out _pnttg2sol[j]);
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

    /// <summary>The solution of index Index (1 to NbSolutions).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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

    /// <summary>The qualifiers of the two arguments for the solution of index Index.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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

    /// <summary>The tangency point on the first argument: its parameters on the solution and on the argument.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the first argument.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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

    /// <summary>The tangency point on the second argument: its parameters on the solution and on the argument.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the second argument.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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

    /// <summary>True if the solution of index Index is the first argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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

    /// <summary>True if the solution of index Index is the second argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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
