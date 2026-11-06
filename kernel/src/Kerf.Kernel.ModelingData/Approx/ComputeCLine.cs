// Port of: src/ModelingData/TKGeomBase/Approx/Approx_ComputeCLine.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Approximation;

/// <summary>
/// Approximates a continuous MultiLine (an <see cref="Function"/>) by Bezier MultiCurves, from degree
/// degreemin to degreemax, cutting the parameter range (when cutting is on) until the tolerances are reached.
/// </summary>
/// <remarks>
/// The body of OCCT's <c>Approx_ComputeCLine.gxx</c>. OCCT instantiates it twice, as Approx_FitAndDivide and
/// Approx_FitAndDivide2d, both over AppCont_Function (which needs no tool), so the body is written once here and the
/// two instantiations are sealed subclasses. A working object (decision 21): the setters are public.
/// </remarks>
internal abstract class ComputeCLine
{
    private const int Maxsegm = 1000;

    private readonly NCollection_Sequence<MultiCurve> _multiCurves = new();
    private readonly NCollection_Sequence<double> _myfirstparam = new();
    private readonly NCollection_Sequence<double> _mylastparam = new();
    private MultiCurve _theMultiCurve = new();
    private bool _alldone;
    private bool _tolreached;
    private readonly NCollection_Sequence<double> _tolers3d = new();
    private readonly NCollection_Sequence<double> _tolers2d = new();
    private int _mydegremin;
    private int _mydegremax;
    private double _mytol3d;
    private double _mytol2d;
    private double _currenttol3d;
    private double _currenttol2d;
    private readonly bool _mycut;
    private Constraint _myfirstC;
    private Constraint _mylastC;
    private int _maxSegments;
    private bool _invOrder;
    private bool _hangChecking;

    /// <summary>
    /// Approximates Line until the tolerances are reached, from degreemin to degreemax, cutting the range if
    /// cutting is true.
    /// </summary>
    protected ComputeCLine(Function line,
                                  int degreemin,
                                  int degreemax,
                                  double tolerance3d,
                                  double tolerance2d,
                                  bool cutting,
                                  Constraint firstC,
                                  Constraint lastC)
    {
        _mydegremin = degreemin;
        _mydegremax = degreemax;
        _mytol3d = tolerance3d;
        _mytol2d = tolerance2d;
        _mycut = cutting;
        _myfirstC = firstC;
        _mylastC = lastC;
        _maxSegments = Maxsegm;
        _invOrder = true;
        _hangChecking = true;
        _alldone = false;
        _tolreached = false;
        _currenttol3d = 0.0;
        _currenttol2d = 0.0;
        Perform(line);
    }

    /// <summary>Initializes the fields of the algorithm.</summary>
    protected ComputeCLine(int degreemin,
                                  int degreemax,
                                  double tolerance3d,
                                  double tolerance2d,
                                  bool cutting,
                                  Constraint firstC,
                                  Constraint lastC)
    {
        _alldone = false;
        _mydegremin = degreemin;
        _mydegremax = degreemax;
        _mytol3d = tolerance3d;
        _mytol2d = tolerance2d;
        _mycut = cutting;
        _myfirstC = firstC;
        _mylastC = lastC;
        _maxSegments = Maxsegm;
        _invOrder = true;
        _hangChecking = true;
        _tolreached = false;
        _currenttol3d = 0.0;
        _currenttol2d = 0.0;
    }

    /// <summary>Runs the algorithm after having initialized the fields.</summary>
    public void Perform(Function line)
    {
        double uFirst, uLast;
        bool finish = false, begin = true, ok = false;
        double thetol3d = global::Kerf.Engine.Foundation.Precision.Confusion(), thetol2d = global::Kerf.Engine.Foundation.Precision.Confusion();
        uFirst = line.FirstParameter();
        uLast = line.LastParameter();
        double tolU;
        if (_hangChecking)
        {
            tolU = Max((uLast - uFirst) * 1.0e-03, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        else
        {
            tolU = Max((uLast - uFirst) * 1.0e-05, global::Kerf.Engine.Foundation.Precision.PApproximation());
        }
        double myfirstU = uFirst;
        double mylastU = uLast;
        int maxSegments = 0;
        int maxSegments1 = _maxSegments - 1;
        int nbCut = 0, nbImp = 0, nbComp = 10;

        if (!_mycut)
        {
            _alldone = Compute(line, uFirst, uLast, ref thetol3d, ref thetol2d);
            if (!_alldone)
            {
                _tolreached = false;
                _myfirstparam.Append(uFirst);
                _mylastparam.Append(uLast);
                _multiCurves.Append(new MultiCurve(_theMultiCurve));
                _tolers3d.Append(_currenttol3d);
                _tolers2d.Append(_currenttol2d);
            }
        }
        else
        {
            // previous decision to be taken if we get worse with next cut (eap)
            MultiCurve keptMultiCurve = new();
            double keptUfirst = 0.0, keptUlast = 0.0, keptT3d = RealLast(), keptT2d = 0.0;

            while (!finish)
            {
                // Management of the multiline splitting for approximation:
                if (!begin)
                {
                    if (ok)
                    {
                        // Calcul de la partie a approximer.
                        myfirstU = mylastU;
                        mylastU = uLast;
                        nbCut = 0;
                        nbImp = 0;
                        if (Math.Abs(uLast - myfirstU) <= RealEpsilon() || maxSegments >= _maxSegments)
                        {
                            finish = true;
                            _alldone = true;
                            return;
                        }
                        keptT3d = RealLast();
                        keptT2d = 0;
                        keptUfirst = myfirstU;
                        keptUlast = mylastU;
                    }
                    else
                    {
                        // keep best decision
                        if ((thetol3d + thetol2d) < (keptT3d + keptT2d))
                        {
                            keptMultiCurve = new MultiCurve(_theMultiCurve);
                            keptUfirst = myfirstU;
                            keptUlast = mylastU;
                            keptT3d = thetol3d;
                            keptT2d = thetol2d;
                            nbImp++;
                        }

                        // cut an interval
                        mylastU = (myfirstU + mylastU) / 2;
                        nbCut++;
                    }
                }

                // Calculation of parameters on this new interval.
                ok = Compute(line, myfirstU, mylastU, ref thetol3d, ref thetol2d);
                if (ok)
                {
                    maxSegments++;
                }

                bool stopCutting = false;
                if (_hangChecking && nbCut >= nbComp)
                {
                    if (nbCut > nbImp + 1)
                    {
                        stopCutting = true;
                    }
                    nbCut = 0;
                    nbImp = 0;
                }
                // is new decision better?
                if (!ok
                    && (Math.Abs(myfirstU - mylastU) <= tolU || maxSegments >= maxSegments1
                        || stopCutting))
                {
                    ok = true; // stop interval cutting, approx the rest part

                    if ((thetol3d + thetol2d) < (keptT3d + keptT2d))
                    {
                        keptMultiCurve = new MultiCurve(_theMultiCurve);
                        keptUfirst = myfirstU;
                        keptUlast = mylastU;
                        keptT3d = thetol3d;
                        keptT2d = thetol2d;
                    }

                    mylastU = keptUlast;

                    _tolreached = false; // helas
                    _multiCurves.Append(new MultiCurve(keptMultiCurve));
                    maxSegments++;
                    _tolers3d.Append(keptT3d);
                    _tolers2d.Append(keptT2d);
                    _myfirstparam.Append(keptUfirst);
                    _mylastparam.Append(keptUlast);
                }

                begin = false;
            } // while (!Finish)
        }
    }

    /// <summary>The number of MultiCurves approximating the MultiLine.</summary>
    public int MultiCurveCount => _multiCurves.Length();

    /// <summary>
    /// The approximating MultiCurve of range Index: a copy sharing the poles, as OCCT's by-value return (decision 141).
    /// </summary>
    public MultiCurve Value(int index = 1) => new(_multiCurves.Value(index));

    private bool Compute(Function line,
                         double ufirst,
                         double ulast,
                         ref double theTol3d,
                         ref double theTol2d)
    {
        const int nbPointsMax = 24;
        const double minRatio = 0.05;
        const int maxDeg = 8;
        //
        int deg, nbPoints;
        bool mydone;

        MultiCurve prevCurve;
        double prevTol3d, prevTol2d;
        bool prevIsOk = false;
        bool invOrder = _invOrder;
        if (invOrder && _mydegremax > maxDeg)
        {
            if ((ulast - ufirst) / (line.LastParameter() - line.FirstParameter()) < minRatio)
            {
                invOrder = false;
            }
        }
        if (invOrder)
        {
            prevCurve = new();
            prevTol3d = RealLast();
            prevTol2d = RealLast();
            for (deg = _mydegremax; deg >= _mydegremin; deg--)
            {
                nbPoints = Min(2 * deg + 1, nbPointsMax);
                LeastSquare lSquare = new(line, ufirst, ulast, _myfirstC, _mylastC, deg, nbPoints);
                mydone = lSquare.IsDone;
                if (mydone)
                {
                    lSquare.Error(out _, out theTol3d, out theTol2d);
                    if (theTol3d <= _mytol3d && theTol2d <= _mytol2d)
                    {
                        if (deg == _mydegremin)
                        {
                            // Stockage de la multicurve approximee.
                            _tolreached = true;
                            _multiCurves.Append(lSquare.Value());
                            _myfirstparam.Append(ufirst);
                            _mylastparam.Append(ulast);
                            _tolers3d.Append(theTol3d);
                            _tolers2d.Append(theTol2d);
                            return true;
                        }
                        prevTol3d = theTol3d;
                        prevTol2d = theTol2d;
                        prevCurve = lSquare.Value();
                        prevIsOk = true;
                        continue;
                    }
                    else if (prevIsOk)
                    {
                        // Stockage de la multicurve approximee.
                        _tolreached = true;
                        theTol3d = prevTol3d;
                        theTol2d = prevTol2d;
                        _multiCurves.Append(prevCurve);
                        _myfirstparam.Append(ufirst);
                        _mylastparam.Append(ulast);
                        _tolers3d.Append(prevTol3d);
                        _tolers2d.Append(prevTol2d);
                        return true;
                    }
                }
                else if (prevIsOk)
                {
                    // Stockage de la multicurve approximee.
                    _tolreached = true;
                    theTol3d = prevTol3d;
                    theTol2d = prevTol2d;
                    _multiCurves.Append(prevCurve);
                    _myfirstparam.Append(ufirst);
                    _mylastparam.Append(ulast);
                    _tolers3d.Append(prevTol3d);
                    _tolers2d.Append(prevTol2d);
                    return true;
                }
                if (!prevIsOk && deg == _mydegremax)
                {
                    // OCCT also stores TheMultiCurve and the tolerances in aPrevCurve/aPrevTol*, which are not read again.
                    _theMultiCurve = lSquare.Value();
                    _currenttol3d = theTol3d;
                    _currenttol2d = theTol2d;
                    break;
                }
            }
        }
        else
        {
            for (deg = _mydegremin; deg <= _mydegremax; deg++)
            {
                nbPoints = Min(2 * deg + 1, nbPointsMax);
                LeastSquare lSquare = new(line, ufirst, ulast, _myfirstC, _mylastC, deg, nbPoints);
                mydone = lSquare.IsDone;
                if (mydone)
                {
                    lSquare.Error(out _, out theTol3d, out theTol2d);
                    if (theTol3d <= _mytol3d && theTol2d <= _mytol2d)
                    {
                        // Stockage de la multicurve approximee.
                        _tolreached = true;
                        _multiCurves.Append(lSquare.Value());
                        _myfirstparam.Append(ufirst);
                        _mylastparam.Append(ulast);
                        _tolers3d.Append(theTol3d);
                        _tolers2d.Append(theTol2d);
                        return true;
                    }
                }
                if (deg == _mydegremax)
                {
                    _theMultiCurve = lSquare.Value();
                    _currenttol3d = theTol3d;
                    _currenttol2d = theTol2d;
                }
            }
        }
        return false;
    }

    /// <summary>The first and last parameters of the MultiCurve Index.</summary>
    public void Parameters(int index, out double firstpar, out double lastpar)
    {
        firstpar = _myfirstparam.Value(index);
        lastpar = _mylastparam.Value(index);
    }

    /// <summary>Changes the degrees of the approximation.</summary>
    public void SetDegrees(int degreemin, int degreemax)
    {
        _mydegremin = degreemin;
        _mydegremax = degreemax;
    }

    /// <summary>Changes the tolerances of the approximation.</summary>
    public void SetTolerances(double tolerance3d, double tolerance2d)
    {
        _mytol3d = tolerance3d;
        _mytol2d = tolerance2d;
    }

    /// <summary>Changes the constraints of the approximation.</summary>
    public void SetConstraints(Constraint firstC, Constraint lastC)
    {
        _myfirstC = firstC;
        _mylastC = lastC;
    }

    /// <summary>Changes the maximum number of segments allowed for cutting.</summary>
    public void SetMaxSegments(int maxSegments)
    {
        _maxSegments = maxSegments;
    }

    /// <summary>Sets the order of degree selection: from degreemax down to degreemin when true (the default).</summary>
    public void SetInvOrder(bool invOrder)
    {
        _invOrder = invOrder;
    }

    /// <summary>Sets whether a possible hang of the cutting is checked and stopped (the default).</summary>
    public void SetHangChecking(bool hangChecking)
    {
        _hangChecking = hangChecking;
    }

    /// <summary>False if some part of the line could not be approximated within the tolerances.</summary>
    public bool IsAllApproximated => _alldone;

    /// <summary>Whether the tolerances were reached.</summary>
    public bool IsToleranceReached => _tolreached;

    /// <summary>The 3D and 2D errors of the MultiCurve Index.</summary>
    public void Error(int index, out double tol3d, out double tol2d)
    {
        tol3d = _tolers3d.Value(index);
        tol2d = _tolers2d.Value(index);
    }
}
