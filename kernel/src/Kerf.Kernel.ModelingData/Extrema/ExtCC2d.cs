// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtCC2d.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtCC2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extremum distances between two 2D curves on parameter ranges. A line or circle against a conic (and a few
/// other pairs) are solved analytically (<see cref="ExtElC2d"/>); other pairs by global optimisation
/// (<see cref="Ecc2d"/>).
/// </summary>
/// <remarks>
/// An algorithm object (decision 117): the second curve is set by <c>Initialize</c>, the first by <c>Perform</c>.
/// Keeps a reference to the second curve, as OCCT does. OCCT's <c>std::shared_ptr</c> locals are plain references.
/// </remarks>
internal class ExtCC2d
{
    private bool _isFindSingleSolution; // Default value is false.
    private bool _done;
    private bool _isPar;
    private readonly NCollection_Sequence<POnCurv2d> _mypoints = new();
    private readonly NCollection_Sequence<double> _sqDist = new();
    private int _mynbext;
    private bool _inverse;
    private Curve2DAdaptorBase? _c;
    private double _myv1;
    private double _myv2;
    private double _mytolc1;
    private double _mytolc2;
    private Point2D _p1f = new();
    private Point2D _p1l = new();
    private Point2D _p2f = new();
    private Point2D _p2l = new();
    private double _mydist11;
    private double _mydist12;
    private double _mydist21;
    private double _mydist22;

    public ExtCC2d()
    {
        _isFindSingleSolution = false;
        _done = false;
        _isPar = false;
        _mynbext = 0;
        _inverse = false;
        _c = null;
        _myv1 = 0.0;
        _myv2 = 0.0;
        _mytolc1 = 0.0;
        _mytolc2 = 0.0;
        _mydist11 = 0.0;
        _mydist12 = 0.0;
        _mydist21 = 0.0;
        _mydist22 = 0.0;
    }

    /// <summary>The extremum distances between C1 and C2 on their whole ranges.</summary>
    public ExtCC2d(Curve2DAdaptorBase c1, Curve2DAdaptorBase c2, double tolC1 = 1.0e-10, double tolC2 = 1.0e-10)
    {
        _isFindSingleSolution = false;
        Initialize(c2, Curve2dTool.FirstParameter(c2), Curve2dTool.LastParameter(c2), tolC1, tolC2);
        Perform(c1, Curve2dTool.FirstParameter(c1), Curve2dTool.LastParameter(c1));
    }

    /// <summary>The extremum distances between C1 on [U1, U2] and C2 on [V1, V2].</summary>
    public ExtCC2d(Curve2DAdaptorBase c1, Curve2DAdaptorBase c2, double u1, double u2, double v1, double v2,
        double tolC1 = 1.0e-10, double tolC2 = 1.0e-10)
    {
        _isFindSingleSolution = false;
        Initialize(c2, v1, v2, tolC1, tolC2);
        Perform(c1, u1, u2);
    }

    /// <summary>Sets the second curve C2 on [V1, V2] and the tolerances.</summary>
    public void Initialize(Curve2DAdaptorBase c2, double v1, double v2, double tolC1 = 1.0e-10, double tolC2 = 1.0e-10)
    {
        _c = c2;
        _myv1 = v1;
        _myv2 = v2;
        _mytolc1 = tolC1;
        _mytolc2 = tolC2;
    }

    /// <summary>Computes the extremum distances between C1 on [U1, U2] and the second curve.</summary>
    public void Perform(Curve2DAdaptorBase c1, double u1, double u2)
    {
        _mypoints.Clear();
        _sqDist.Clear();
        CurveKind type1 = Curve2dTool.GetType(c1), type2 = Curve2dTool.GetType(_c!);
        double u11, u12, u21, u22, tol = Min(_mytolc1, _mytolc2);
        //  Extrema_POnCurv2d P1, P2;
        _mynbext = 0;
        _inverse = false;
        _isPar = false;

        u11 = u1;
        u12 = u2;
        u21 = _myv1;
        u22 = _myv2;
        _p1f = Curve2dTool.Value(c1, u11);
        _p1l = Curve2dTool.Value(c1, u12);
        _p2f = Curve2dTool.Value(_c!, u21);
        _p2l = Curve2dTool.Value(_c!, u22);

        ExtElC2d xtream;
        Ecc2d paramSolver;

        switch (type1)
        {
            //
            //  The first curve is a circle:
            //
            case Circle:
            {
                switch (type2)
                {
                    case Line:
                    {
                        _inverse = true;
                        xtream = new ExtElC2d(Curve2dTool.Line(_c!), Curve2dTool.Circle(c1), tol);
                        Results(xtream, u11, u12, u21, u22, 2 * M_PI, 0.0);
                    }
                    break;
                    case Circle:
                    {
                        xtream = new ExtElC2d(Curve2dTool.Circle(c1), Curve2dTool.Circle(_c!));
                        Results(xtream, u11, u12, u21, u22, 2 * M_PI, 2 * M_PI);
                    }
                    break;
                    case Ellipse:
                    {
                        xtream = new ExtElC2d(Curve2dTool.Circle(c1), Curve2dTool.Ellipse(_c!));
                        Results(xtream, u11, u12, u21, u22, 2 * M_PI, 2 * M_PI);
                    }
                    break;
                    case Parabola:
                    {
                        xtream = new ExtElC2d(Curve2dTool.Circle(c1), Curve2dTool.Parabola(_c!));
                        Results(xtream, u11, u12, u21, u22, 2 * M_PI, 0.0);
                    }
                    break;
                    case Hyperbola:
                    {
                        xtream = new ExtElC2d(Curve2dTool.Circle(c1), Curve2dTool.Hyperbola(_c!));
                        Results(xtream, u11, u12, u21, u22, 2 * M_PI, 0.0);
                    }
                    break;
                    default:
                    {
                        paramSolver = new Ecc2d(c1, _c!);
                        paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        paramSolver.Perform();
                        double period2 = 0.0;
                        if (Curve2dTool.IsPeriodic(_c!))
                        {
                            period2 = Curve2dTool.Period(_c!);
                        }
                        Results(paramSolver, u11, u12, u21, u22, 2 * M_PI, period2);
                    }
                    break;
                } // switch(type2)
            }
            break;

            //
            // The first curve is an ellipse:
            //
            case Ellipse:
            {
                switch (type2)
                {
                    case Line:
                    {
                        _inverse = true;
                        xtream = new ExtElC2d(Curve2dTool.Line(_c!), Curve2dTool.Ellipse(c1));
                        Results(xtream, u11, u12, u21, u22, 2 * M_PI, 0.0);
                    }
                    break;
                    case Circle:
                    {
                        _inverse = true;
                        xtream = new ExtElC2d(Curve2dTool.Circle(_c!), Curve2dTool.Ellipse(c1));
                        Results(xtream, u11, u12, u21, u22, 2 * M_PI, 2 * M_PI);
                    }
                    break;
                    case Ellipse:
                    {
                        paramSolver = new Ecc2d(c1, _c!);
                        paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        paramSolver.Perform();
                        Results(paramSolver, u11, u12, u21, u22, 2 * M_PI, 2 * M_PI);
                    }
                    break;
                    case Parabola:
                    {
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Ellipse(C1),
                        // Extrema_Curve2dTool::Parabola(*myC));
                        paramSolver = new Ecc2d(c1, _c!);
                        paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        paramSolver.Perform();
                        Results(paramSolver, u11, u12, u21, u22, 2 * M_PI, 0.0);
                    }
                    break;
                    case Hyperbola:
                    {
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Ellipse(C1),
                        // Extrema_Curve2dTool::Hyperbola(*myC));
                        paramSolver = new Ecc2d(c1, _c!);
                        paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        paramSolver.Perform();
                        Results(paramSolver, u11, u12, u21, u22, 2 * M_PI, 0.0);
                    }
                    break;
                    default:
                    {
                        paramSolver = new Ecc2d(c1, _c!);
                        paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        paramSolver.Perform();
                        double period2 = 0.0;
                        if (Curve2dTool.IsPeriodic(_c!))
                        {
                            period2 = Curve2dTool.Period(_c!);
                        }
                        Results(paramSolver, u11, u12, u21, u22, 2 * M_PI, period2);
                    }
                    break;
                } // switch(type2)
            }
            break;

            //
            // The first curve is a parabola:
            //
            case Parabola:
            {
                switch (type2)
                {
                    case Line:
                    {
                        _inverse = true;
                        xtream = new ExtElC2d(Curve2dTool.Line(_c!), Curve2dTool.Parabola(c1));
                        Results(xtream, u11, u12, u21, u22, 0.0, 0.0);
                    }
                    break;
                    case Circle:
                    {
                        _inverse = true;
                        xtream = new ExtElC2d(Curve2dTool.Circle(_c!), Curve2dTool.Parabola(c1));
                        Results(xtream, u11, u12, u21, u22, 0.0, 2 * M_PI);
                    }
                    break;
                    case Ellipse:
                    {
                        // inverse = true;
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Ellipse(*myC),
                        // Extrema_Curve2dTool::Parabola(C1));
                        paramSolver = new Ecc2d(c1, _c!);
                        paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        paramSolver.Perform();
                        Results(paramSolver, u11, u12, u21, u22, 0.0, 2 * M_PI);
                    }
                    break;
                    case Parabola:
                    {
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Parabola(C1),
                        // Extrema_Curve2dTool::Parabola(*myC));
                        paramSolver = new Ecc2d(c1, _c!);
                        paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        paramSolver.Perform();
                        Results(paramSolver, u11, u12, u21, u22, 0.0, 0.0);
                    }
                    break;
                    case Hyperbola:
                    {
                        // inverse = true;
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Hyperbola(*myC),
                        // Extrema_Curve2dTool::Parabola(C1));
                        paramSolver = new Ecc2d(c1, _c!);
                        paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        paramSolver.Perform();
                        Results(paramSolver, u11, u12, u21, u22, 0.0, 0.0);
                    }
                    break;
                    default:
                    {
                        paramSolver = new Ecc2d(c1, _c!);
                        paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        paramSolver.Perform();
                        double period2 = 0.0;
                        if (Curve2dTool.IsPeriodic(_c!))
                        {
                            period2 = Curve2dTool.Period(_c!);
                        }
                        Results(paramSolver, u11, u12, u21, u22, 0.0, period2);
                    }
                    break;
                } // switch(type2)
            }
            break;

            //
            // The first curve is a hyperbola:
            //
            case Hyperbola:
            {
                switch (type2)
                {
                    case Line:
                    {
                        _inverse = true;
                        xtream = new ExtElC2d(Curve2dTool.Line(_c!), Curve2dTool.Hyperbola(c1));
                        Results(xtream, u11, u12, u21, u22, 0.0, 0.0);
                    }
                    break;
                    case Circle:
                    {
                        _inverse = true;
                        xtream = new ExtElC2d(Curve2dTool.Circle(_c!), Curve2dTool.Hyperbola(c1));
                        Results(xtream, u11, u12, u21, u22, 0.0, 2 * M_PI);
                    }
                    break;
                    case Ellipse:
                    {
                        // inverse = true;
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Ellipse(*myC),
                        // Extrema_Curve2dTool::Hyperbola(C1));
                        paramSolver = new Ecc2d(c1, _c!);
                        paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        paramSolver.Perform();
                        Results(paramSolver, u11, u12, u21, u22, 0.0, 2 * M_PI);
                    }
                    break;
                    case Parabola:
                    {
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Hyperbola(C1),
                        // Extrema_Curve2dTool::Parabola(*myC));
                        paramSolver = new Ecc2d(c1, _c!);
                        paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        paramSolver.Perform();
                        Results(paramSolver, u11, u12, u21, u22, 0.0, 0.0);
                    }
                    break;
                    case Hyperbola:
                    {
                        // Extrema_ExtElC2d Xtrem(Extrema_Curve2dTool::Hyperbola(C1),
                        // Extrema_Curve2dTool::Hyperbola(*myC));
                        paramSolver = new Ecc2d(c1, _c!);
                        paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        paramSolver.Perform();
                        Results(paramSolver, u11, u12, u21, u22, 0.0, 0.0);
                    }
                    break;
                    default:
                    {
                        paramSolver = new Ecc2d(c1, _c!);
                        paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        paramSolver.Perform();
                        double period2 = 0.0;
                        if (Curve2dTool.IsPeriodic(_c!))
                        {
                            period2 = Curve2dTool.Period(_c!);
                        }
                        Results(paramSolver, u11, u12, u21, u22, 0.0, period2);
                    }
                    break;
                } // switch(type2)
            }
            break;

            //
            // The first curve is a line:
            //
            case Line:
            {
                switch (type2)
                {
                    case Line:
                    {
                        xtream = new ExtElC2d(Curve2dTool.Line(c1), Curve2dTool.Line(_c!), tol);
                        Results(xtream, u11, u12, u21, u22, 0.0, 0.0);
                    }
                    break;
                    case Circle:
                    {
                        xtream = new ExtElC2d(Curve2dTool.Line(c1), Curve2dTool.Circle(_c!), tol);
                        Results(xtream, u11, u12, u21, u22, 0.0, 2 * M_PI);
                    }
                    break;
                    case Ellipse:
                    {
                        xtream = new ExtElC2d(Curve2dTool.Line(c1), Curve2dTool.Ellipse(_c!));
                        Results(xtream, u11, u12, u21, u22, 0.0, 2 * M_PI);
                    }
                    break;
                    case Parabola:
                    {
                        xtream = new ExtElC2d(Curve2dTool.Line(c1), Curve2dTool.Parabola(_c!));
                        Results(xtream, u11, u12, u21, u22, 0.0, 0.0);
                    }
                    break;
                    case Hyperbola:
                    {
                        xtream = new ExtElC2d(Curve2dTool.Line(c1), Curve2dTool.Hyperbola(_c!));
                        Results(xtream, u11, u12, u21, u22, 0.0, 0.0);
                    }
                    break;
                    default:
                    {
                        paramSolver = new Ecc2d(c1, _c!);
                        paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                        paramSolver.Perform();
                        double period2 = 0.0;
                        if (Curve2dTool.IsPeriodic(_c!))
                        {
                            period2 = Curve2dTool.Period(_c!);
                        }
                        Results(paramSolver, u11, u12, u21, u22, 0.0, period2);
                    }
                    break;
                } // switch(type2)
            }
            break;

            //
            // The first curve is a BezierCurve or a BSplineCurve:
            //
            default:
            {
                paramSolver = new Ecc2d(c1, _c!);
                paramSolver.SetSingleSolutionFlag(GetSingleSolutionFlag());
                paramSolver.Perform();
                double period1 = 0.0;
                if (Curve2dTool.IsPeriodic(c1))
                {
                    period1 = Curve2dTool.Period(c1);
                }
                double period2 = 0.0;
                if (Curve2dTool.IsPeriodic(_c!))
                {
                    period2 = Curve2dTool.Period(_c!);
                }
                Results(paramSolver, u11, u12, u21, u22, period1, period2);
            }
            break;
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _done;

    /// <summary>The Nth square distance.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="OutOfRangeException">N is not in 1..NbExt().</exception>
    public double SquareDistance(int n = 1)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if ((n <= 0) || (n > _mynbext))
        {
            throw new OutOfRangeException();
        }
        return _sqDist.Value(n);
    }

    /// <summary>The number of extremum distances.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int ExtCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _mynbext;
        }
    }

    /// <summary>The points of the Nth extremum, on the first and on the second curve.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="OutOfRangeException">N is not in 1..NbExt().</exception>
    public void Points(int n, out POnCurv2d p1, out POnCurv2d p2)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if ((n <= 0) || (n > _mynbext))
        {
            throw new OutOfRangeException();
        }
        p1 = _mypoints.Value(2 * n - 1);
        p2 = _mypoints.Value(2 * n);
    }

    /// <summary>The square distances between the ends of the ranges and the end points (set only by a successful
    /// computation).</summary>
    public void TrimmedSquareDistances(out double dist11, out double dist12, out double dist21, out double dist22,
        out Point2D p11, out Point2D p12, out Point2D p21, out Point2D p22)
    {
        dist11 = _mydist11;
        dist12 = _mydist12;
        dist21 = _mydist21;
        dist22 = _mydist22;
        p11 = _p1f;
        p12 = _p1l;
        p21 = _p2f;
        p22 = _p2l;
    }

    protected void Results(ExtElC2d algExt, double ut11, double ut12, double ut21, double ut22,
        double period1 = 0.0, double period2 = 0.0)
    {
        int i, nbExt;
        double val, u, u2;
        POnCurv2d p1, p2;

        _done = algExt.IsDone;
        // OCCT asks IsParallel before checking IsDone: an Extrema_ExtElC2d that is not done throws StdFail_NotDone
        // here (a circle and a concentric circular ellipse) instead of leaving this algorithm not done
        _isPar = algExt.IsParallel;
        if (_done)
        {
            if (!_isPar)
            {
                nbExt = algExt.ExtCount;
                for (i = 1; i <= nbExt; i++)
                {
                    // Verification of the validity of parameters for the trimmed case:
                    algExt.Points(i, out p1, out p2);
                    if (!_inverse)
                    {
                        u = p1.Parameter();
                        if (period1 != 0.0)
                        {
                            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, ut11, ut11 + period1);
                        }
                        u2 = p2.Parameter();
                        if (period2 != 0.0)
                        {
                            u2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u2, ut21, ut21 + period2);
                        }
                    }
                    else
                    {
                        u2 = p1.Parameter();
                        if (period2 != 0.0)
                        {
                            u2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u2, ut21, ut21 + period2);
                        }
                        u = p2.Parameter();
                        if (period1 != 0.0)
                        {
                            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, ut11, ut11 + period1);
                        }
                    }
                    if ((u >= ut11 - global::Kerf.Engine.Foundation.Precision.PConfusion()) && (u <= ut12 + global::Kerf.Engine.Foundation.Precision.PConfusion())
                        && (u2 >= ut21 - global::Kerf.Engine.Foundation.Precision.PConfusion()) && (u2 <= ut22 + global::Kerf.Engine.Foundation.Precision.PConfusion()))
                    {
                        _mynbext++;
                        val = algExt.SquareDistance(i);
                        _sqDist.Append(val);
                        if (!_inverse)
                        {
                            p1.SetValues(u, p1.Value());
                            p2.SetValues(u2, p2.Value());
                            _mypoints.Append(p1);
                            _mypoints.Append(p2);
                        }
                        else
                        {
                            p1.SetValues(u2, p1.Value());
                            p2.SetValues(u, p2.Value());
                            _mypoints.Append(p2);
                            _mypoints.Append(p1);
                        }
                    }
                }
            }

            _mydist11 = _p1f.SquareDistanceTo(_p2f);
            _mydist12 = _p1f.SquareDistanceTo(_p2l);
            _mydist21 = _p1l.SquareDistanceTo(_p2f);
            _mydist22 = _p1l.SquareDistanceTo(_p2l);
        }
    }

    protected void Results(Ecc2d algExt, double ut11, double ut12, double ut21, double ut22,
        double period1 = 0.0, double period2 = 0.0)
    {
        int i, nbExt;
        double val, u, u2;
        POnCurv2d p1, p2;

        _done = algExt.IsDone;
        if (_done)
        {
            _isPar = algExt.IsParallel;
            nbExt = algExt.ExtCount;
            for (i = 1; i <= nbExt; i++)
            {
                // Verification of parameter validity for the trimmed case:
                algExt.Points(i, out p1, out p2);
                u = p1.Parameter();
                if (period1 != 0.0)
                {
                    u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, ut11, ut11 + period1);
                }
                u2 = p2.Parameter();
                if (period2 != 0.0)
                {
                    u2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u2, ut21, ut21 + period2);
                }

                if ((u >= ut11 - global::Kerf.Engine.Foundation.Precision.PConfusion()) && (u <= ut12 + global::Kerf.Engine.Foundation.Precision.PConfusion())
                    && (u2 >= ut21 - global::Kerf.Engine.Foundation.Precision.PConfusion()) && (u2 <= ut22 + global::Kerf.Engine.Foundation.Precision.PConfusion()))
                {
                    _mynbext++;
                    val = algExt.SquareDistance(i);
                    p1.SetValues(u, p1.Value());
                    p2.SetValues(u2, p2.Value());
                    _sqDist.Append(val);
                    _mypoints.Append(p1);
                    _mypoints.Append(p2);
                }
            }

            _mydist11 = _p1f.SquareDistanceTo(_p2f);
            _mydist12 = _p1f.SquareDistanceTo(_p2l);
            _mydist21 = _p1l.SquareDistanceTo(_p2f);
            _mydist22 = _p1l.SquareDistanceTo(_p2l);
        }
    }

    /// <summary>True if the curves are parallel.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public bool IsParallel
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _isPar;
        }
    }

    /// <summary>Asks for one solution only (faster, for the global optimisation).</summary>
    public void SetSingleSolutionFlag(bool flag) => _isFindSingleSolution = flag;

    public bool GetSingleSolutionFlag() => _isFindSingleSolution;
}
