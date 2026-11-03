// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_Variational.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_Variational.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Jeannine PANCIATICI le 06/06/96
// Igor FEOKTISTOV 14/12/98 - correction of Approximate() and Init().
// Approximation of a MultiLine of points described by the tool MLineTool.
// with variational criteria (minimisation of a quadratic criterion and of a "fairness" criterion).

using Kerf.Engine.Approximation.FiniteElements;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics;
using Kerf.Engine.Numerics.Polynomials;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Approximation.Constraint;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Smooths the points of a MultiLine into B-spline curves under passing, tangency and curvature constraints, by
/// minimising a least-squares criterion together with a variational "fair curve" criterion (tension, flexion,
/// jerk).
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21): the setters are public and recompute the criterion as OCCT does;
/// <see cref="Approximate"/> runs the computation. If (MaxDegree - Continuity) * MaxSegment - Nc0 - 2 Nc1 - 3 Nc2
/// is negative (Nc0, Nc1, Nc2 the numbers of passing, tangency and curvature constraints), the problem is
/// over-constrained (<see cref="IsOverConstrained"/>). The MultiLine must have one line (of dimension 2 or 3) or
/// several of the same kind.
/// </para>
/// <para>
/// <see cref="Value"/> returns a copy of the result that shares its poles, as OCCT's by-value return does;
/// <see cref="Parameters"/> and <see cref="Knots"/> return the object's own arrays (shared: don't modify), as OCCT
/// returns its handles (decision 207). <c>Dump</c> is left out (rules, section 6).
/// </para>
/// </remarks>
public class Variational
{
    private readonly MultiLine _ssp;
    private readonly int _nbP3d;
    private readonly int _nbP2d;
    private readonly int _dimension;
    private readonly int _firstPoint;
    private readonly int _lastPoint;
    private readonly int _nbPoints;
    private readonly NCollection_HArray1<double> _tabPoints;
    private NCollection_HArray1<ConstraintCouple> _constraints;
    private int _nbConstraints;
    private NCollection_HArray1<double> _tabConstraints = null!;
    private int _nbPassPoints;
    private int _nbTangPoints;
    private int _nbCurvPoints;
    private NCollection_HArray1<int> _typConstraints = null!;
    private NCollection_HArray1<double> _ttheta = null!;
    private NCollection_HArray1<double> _tfthet = null!;
    private int _maxDegree;
    private int _maxSegment;
    private int _nbIterations;
    private double _tolerance;
    private Continuity _continuity;
    private int _nivCont;
    private bool _withMinMax;
    private bool _withCutting;
    private readonly double[] _percent = new double[3];
    private readonly double[] _criterium = new double[4];
    private SmoothCriterion _smoothCriterion;
    private readonly NCollection_HArray1<double> _parameters;
    private NCollection_HArray1<double> _knots;
    private MultiBSpCurve _mbSpCurve = new();
    private double _maxError;
    private int _maxErrorIndex;
    private double _averageError;
    private bool _isCreated;
    private bool _isDone;
    private bool _isOverConstr;

    /// <summary>
    /// Sets up the smoothing of the points <paramref name="firstPoint"/> to <paramref name="lastPoint"/> of
    /// <paramref name="ssp"/> under the constraints <paramref name="theConstraints"/> (kept, shared).
    /// </summary>
    public Variational(MultiLine ssp, int firstPoint, int lastPoint,
                              NCollection_HArray1<ConstraintCouple> theConstraints, int maxDegree = 14,
                              int maxSegment = 100, Continuity continuity = C2, bool withMinMax = false,
                              bool withCutting = true, double tolerance = 1.0, int nbIterations = 2)
    {
        _ssp = ssp;
        _firstPoint = firstPoint;
        _lastPoint = lastPoint;
        _constraints = theConstraints;
        _maxDegree = maxDegree;
        _maxSegment = maxSegment;
        _nbIterations = nbIterations;
        _tolerance = tolerance;
        _continuity = continuity;
        _withMinMax = withMinMax;
        _withCutting = withCutting;

        // Verifications:
        if (_maxDegree < 1)
        {
            throw new DomainErrorException();
        }
        _maxDegree = Min(30, _maxDegree);
        //
        if (_maxSegment < 1)
        {
            throw new DomainErrorException();
        }
        // OCCT's checks that myWithMinMax and myWithCutting are 0 or 1 always pass for a bool: left out.
        //
        _isOverConstr = false;
        _isCreated = false;
        _isDone = false;
        switch (_continuity)
        {
            case C0:
                _nivCont = 0;
                break;
            case C1:
                _nivCont = 1;
                break;
            case C2:
                _nivCont = 2;
                break;
            default:
                throw new ConstructionErrorException();
        }
        //
        _nbP2d = MyLineTool.NbP2d(ssp);
        _nbP3d = MyLineTool.NbP3d(ssp);
        _dimension = 2 * _nbP2d + 3 * _nbP3d;
        //
        _percent[0] = 0.4;
        _percent[1] = 0.2;
        _percent[2] = 0.4;
        _knots = new NCollection_HArray1<double>(1, 2);
        _knots.SetValue(1, 0.0);
        _knots.SetValue(2, 1.0);

        //  Declaration
        //
        _smoothCriterion = new LinearCriteria(_ssp, _firstPoint, _lastPoint);
        _parameters = new NCollection_HArray1<double>(_firstPoint, _lastPoint);
        _nbPoints = _lastPoint - _firstPoint + 1;
        if (_nbPoints <= 0)
        {
            throw new ConstructionErrorException();
        }
        //
        _tabPoints = new NCollection_HArray1<double>(1, _dimension * _nbPoints);
        //
        //  Table of Points initialization
        //
        int ipoint, jp2d, jp3d, index;
        NCollection_Array1<Point3D> tabP3d = new(1, Max(1, _nbP3d));
        NCollection_Array1<Point2D> tabP2d = new(1, Max(1, _nbP2d));
        Point2D p2d;
        Point3D p3d;
        index = 1;

        for (ipoint = _firstPoint; ipoint <= _lastPoint; ipoint++)
        {
            if (_nbP2d != 0 && _nbP3d == 0)
            {
                MyLineTool.Value(_ssp, ipoint, tabP2d);

                for (jp2d = 1; jp2d <= _nbP2d; jp2d++)
                {
                    p2d = tabP2d.Value(jp2d);

                    _tabPoints.SetValue(index++, p2d.X);
                    _tabPoints.SetValue(index++, p2d.Y);
                }
            }
            if (_nbP3d != 0 && _nbP2d == 0)
            {
                MyLineTool.Value(_ssp, ipoint, tabP3d);

                for (jp3d = 1; jp3d <= _nbP3d; jp3d++)
                {
                    p3d = tabP3d.Value(jp3d);

                    _tabPoints.SetValue(index++, p3d.X);
                    _tabPoints.SetValue(index++, p3d.Y);
                    _tabPoints.SetValue(index++, p3d.Z);
                }
            }
            if (_nbP3d != 0 && _nbP2d != 0)
            {
                MyLineTool.Value(_ssp, ipoint, tabP3d, tabP2d);

                for (jp3d = 1; jp3d <= _nbP3d; jp3d++)
                {
                    p3d = tabP3d.Value(jp3d);

                    _tabPoints.SetValue(index++, p3d.X);
                    _tabPoints.SetValue(index++, p3d.Y);
                    _tabPoints.SetValue(index++, p3d.Z);
                }
                for (jp2d = 1; jp2d <= _nbP2d; jp2d++)
                {
                    p2d = tabP2d.Value(jp2d);

                    _tabPoints.SetValue(index++, p2d.X);
                    _tabPoints.SetValue(index++, p2d.Y);
                }
            }
        }
        Init();
    }

    private void Init()
    {
        int ipoint, jp2d, jp3d, index, jndex;
        int curMultyPoint;
        NCollection_Array1<Vector3D> tabV3d = new(1, Max(1, _nbP3d));
        NCollection_Array1<Vector2D> tabV2d = new(1, Max(1, _nbP2d));
        NCollection_Array1<Vector3D> tabV3dcurv = new(1, Max(1, _nbP3d));
        NCollection_Array1<Vector2D> tabV2dcurv = new(1, Max(1, _nbP2d));

        Vector3D vt3d, vc3d;
        Vector2D vt2d, vc2d;

        _nbConstraints = _constraints.Length();
        if (_nbConstraints < 0)
        {
            throw new ConstructionErrorException();
        }

        _typConstraints = new NCollection_HArray1<int>(1, Max(1, 2 * _nbConstraints));
        _tabConstraints = new NCollection_HArray1<double>(1, Max(1, 2 * _dimension * _nbConstraints));
        _ttheta = new NCollection_HArray1<double>(1, Max(1, (2 * _nbP2d + 6 * _nbP3d) * _nbConstraints));
        _tfthet = new NCollection_HArray1<double>(1, Max(1, (2 * _nbP2d + 6 * _nbP3d) * _nbConstraints));

        //
        // Table of types initialization
        int iconstr;
        index = 1;
        jndex = 1;
        curMultyPoint = 1;
        _nbPassPoints = 0;
        _nbTangPoints = 0;
        _nbCurvPoints = 0;
        Constraint valcontr;

        for (iconstr = _constraints.Lower(); iconstr <= _constraints.Upper(); iconstr++)
        {
            ipoint = (_constraints.Value(iconstr)).Index();

            valcontr = (_constraints.Value(iconstr)).Constraint();
            switch (valcontr)
            {
                case NoConstraint:
                    curMultyPoint -= _nbP3d * 6 + _nbP2d * 2;
                    break;
                case PassPoint:
                    _typConstraints.SetValue(index++, ipoint);
                    _typConstraints.SetValue(index++, 0);
                    _nbPassPoints++;
                    if (_nbP2d != 0)
                    {
                        jndex = jndex + 4 * _nbP2d;
                    }
                    if (_nbP3d != 0)
                    {
                        jndex = jndex + 6 * _nbP3d;
                    }
                    break;
                case TangencyPoint:
                    _typConstraints.SetValue(index++, ipoint);
                    _typConstraints.SetValue(index++, 1);
                    _nbTangPoints++;
                    if (_nbP2d != 0 && _nbP3d == 0)
                    {
                        if (!MyLineTool.Tangency(_ssp, ipoint, tabV2d))
                        {
                            throw new ConstructionErrorException();
                        }
                        for (jp2d = 1; jp2d <= _nbP2d; jp2d++)
                        {
                            vt2d = tabV2d.Value(jp2d);
                            vt2d = vt2d.Normalized();
                            _tabConstraints.SetValue(jndex++, vt2d.X);
                            _tabConstraints.SetValue(jndex++, vt2d.Y);
                            jndex = jndex + 2;
                            InitTthetaF(2, valcontr, curMultyPoint + (jp2d - 1) * 2, jndex - 4);
                        }
                    }
                    if (_nbP3d != 0 && _nbP2d == 0)
                    {
                        if (!MyLineTool.Tangency(_ssp, ipoint, tabV3d))
                        {
                            throw new ConstructionErrorException();
                        }
                        for (jp3d = 1; jp3d <= _nbP3d; jp3d++)
                        {
                            vt3d = tabV3d.Value(jp3d);
                            vt3d = vt3d.Normalized();
                            _tabConstraints.SetValue(jndex++, vt3d.X);

                            _tabConstraints.SetValue(jndex++, vt3d.Y);

                            _tabConstraints.SetValue(jndex++, vt3d.Z);
                            jndex = jndex + 3;
                            InitTthetaF(3, valcontr, curMultyPoint + (jp3d - 1) * 6, jndex - 6);
                        }
                    }
                    if (_nbP3d != 0 && _nbP2d != 0)
                    {
                        if (!MyLineTool.Tangency(_ssp, ipoint, tabV3d, tabV2d))
                        {
                            throw new ConstructionErrorException();
                        }
                        for (jp3d = 1; jp3d <= _nbP3d; jp3d++)
                        {
                            vt3d = tabV3d.Value(jp3d);
                            vt3d = vt3d.Normalized();
                            _tabConstraints.SetValue(jndex++, vt3d.X);
                            _tabConstraints.SetValue(jndex++, vt3d.Y);
                            _tabConstraints.SetValue(jndex++, vt3d.Z);
                            jndex = jndex + 3;
                            InitTthetaF(3, valcontr, curMultyPoint + (jp3d - 1) * 6, jndex - 6);
                        }

                        for (jp2d = 1; jp2d <= _nbP2d; jp2d++)
                        {
                            vt2d = tabV2d.Value(jp2d);
                            vt2d = vt2d.Normalized();
                            _tabConstraints.SetValue(jndex++, vt2d.X);
                            _tabConstraints.SetValue(jndex++, vt2d.Y);
                            jndex = jndex + 2;
                            InitTthetaF(2, valcontr, curMultyPoint + (jp2d - 1) * 2 + _nbP3d * 6, jndex - 4);
                        }
                    }

                    break;

                case CurvaturePoint:
                    _typConstraints.SetValue(index++, ipoint);
                    _typConstraints.SetValue(index++, 2);
                    _nbCurvPoints++;
                    if (_nbP2d != 0 && _nbP3d == 0)
                    {
                        if (!MyLineTool.Tangency(_ssp, ipoint, tabV2d))
                        {
                            throw new ConstructionErrorException();
                        }
                        if (!MyLineTool.Curvature(_ssp, ipoint, tabV2dcurv))
                        {
                            throw new ConstructionErrorException();
                        }
                        for (jp2d = 1; jp2d <= _nbP2d; jp2d++)
                        {
                            vt2d = tabV2d.Value(jp2d);
                            vt2d = vt2d.Normalized();
                            vc2d = tabV2dcurv.Value(jp2d);
                            if (Math.Abs(Math.Abs(vc2d.AngleTo(vt2d)) - M_PI / 2.0) > global::Kerf.Engine.Foundation.Precision.Angular())
                            {
                                throw new ConstructionErrorException();
                            }
                            _tabConstraints.SetValue(jndex++, vt2d.X);
                            _tabConstraints.SetValue(jndex++, vt2d.Y);
                            _tabConstraints.SetValue(jndex++, vc2d.X);
                            _tabConstraints.SetValue(jndex++, vc2d.Y);
                            InitTthetaF(2, valcontr, curMultyPoint + (jp2d - 1) * 2, jndex - 4);
                        }
                    }

                    if (_nbP3d != 0 && _nbP2d == 0)
                    {
                        if (!MyLineTool.Tangency(_ssp, ipoint, tabV3d))
                        {
                            throw new ConstructionErrorException();
                        }
                        if (!MyLineTool.Curvature(_ssp, ipoint, tabV3dcurv))
                        {
                            throw new ConstructionErrorException();
                        }
                        for (jp3d = 1; jp3d <= _nbP3d; jp3d++)
                        {
                            vt3d = tabV3d.Value(jp3d);
                            vt3d = vt3d.Normalized();
                            vc3d = tabV3dcurv.Value(jp3d);
                            if (!(vc3d.Normalized()).IsNormal(vt3d, global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                throw new ConstructionErrorException();
                            }
                            _tabConstraints.SetValue(jndex++, vt3d.X);
                            _tabConstraints.SetValue(jndex++, vt3d.Y);
                            _tabConstraints.SetValue(jndex++, vt3d.Z);
                            _tabConstraints.SetValue(jndex++, vc3d.X);
                            _tabConstraints.SetValue(jndex++, vc3d.Y);
                            _tabConstraints.SetValue(jndex++, vc3d.Z);
                            InitTthetaF(3, valcontr, curMultyPoint + (jp3d - 1) * 6, jndex - 6);
                        }
                    }
                    if (_nbP3d != 0 && _nbP2d != 0)
                    {
                        if (!MyLineTool.Tangency(_ssp, ipoint, tabV3d, tabV2d))
                        {
                            throw new ConstructionErrorException();
                        }
                        if (!MyLineTool.Curvature(_ssp, ipoint, tabV3dcurv, tabV2dcurv))
                        {
                            throw new ConstructionErrorException();
                        }
                        for (jp3d = 1; jp3d <= _nbP3d; jp3d++)
                        {
                            vt3d = tabV3d.Value(jp3d);
                            vt3d = vt3d.Normalized();
                            vc3d = tabV3dcurv.Value(jp3d);
                            if (!(vc3d.Normalized()).IsNormal(vt3d, global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                throw new ConstructionErrorException();
                            }
                            _tabConstraints.SetValue(jndex++, vt3d.X);
                            _tabConstraints.SetValue(jndex++, vt3d.Y);
                            _tabConstraints.SetValue(jndex++, vt3d.Z);
                            _tabConstraints.SetValue(jndex++, vc3d.X);
                            _tabConstraints.SetValue(jndex++, vc3d.Y);
                            _tabConstraints.SetValue(jndex++, vc3d.Z);
                            InitTthetaF(3, valcontr, curMultyPoint + (jp3d - 1) * 6, jndex - 6);
                        }
                        for (jp2d = 1; jp2d <= _nbP2d; jp2d++)
                        {
                            vt2d = tabV2d.Value(jp2d);
                            vt2d = vt2d.Normalized();
                            vc2d = tabV2dcurv.Value(jp2d);
                            if (Math.Abs(Math.Abs(vc2d.AngleTo(vt2d)) - M_PI / 2.0) > global::Kerf.Engine.Foundation.Precision.Angular())
                            {
                                throw new ConstructionErrorException();
                            }
                            _tabConstraints.SetValue(jndex++, vt2d.X);
                            _tabConstraints.SetValue(jndex++, vt2d.Y);
                            _tabConstraints.SetValue(jndex++, vc2d.X);
                            _tabConstraints.SetValue(jndex++, vc2d.Y);
                            InitTthetaF(2, valcontr, curMultyPoint + (jp2d - 1) * 2 + _nbP3d * 6, jndex - 4);
                        }
                    }
                    break;
                default:
                    throw new ConstructionErrorException();
            }
            curMultyPoint += _nbP3d * 6 + _nbP2d * 2;
        }
        // OverConstraint Detection
        int maxSeg;
        if (_withCutting)
        {
            maxSeg = _maxSegment;
        }
        else
        {
            maxSeg = 1;
        }
        if (((_maxDegree - _nivCont) * maxSeg - _nbPassPoints - 2 * _nbTangPoints - 3 * _nbCurvPoints) < 0)
        {
            _isOverConstr = true;
            _isCreated = false;
        }
        else
        {
            InitSmoothCriterion();
            _isCreated = true;
        }
    }

    /// <summary>Makes the approximation with the current fields. Throws StdFail_NotDone unless <see cref="IsCreated"/>.</summary>
    public void Approximate()
    {
        if (!_isCreated)
        {
            throw new NotDoneException();
        }

        double wQuadratic, wQuality;

        NCollection_Array1<double> ecarts = new(_firstPoint, _lastPoint);

        _smoothCriterion.GetWeight(out wQuadratic, out wQuality);

        Curve? theCurve;

        _smoothCriterion.GetCurve(out theCurve);

        //---------------------------------------------------------------------

        TheMotor(_smoothCriterion, wQuadratic, wQuality, ref theCurve!, ecarts);

        if (_withMinMax && _tolerance < _maxError)
        {
            Adjusting(ref _smoothCriterion, ref wQuadratic, ref wQuality, ref theCurve, ecarts);
        }

        //---------------------------------------------------------------------

        int jp2d, jp3d, ipole, nbElem = theCurve.ElementCount;

        NCollection_Array1<Point3D> tabP3d = new(1, Max(1, _nbP3d));
        NCollection_Array1<Point2D> tabP2d = new(1, Max(1, _nbP2d));
        ReadOnlySpan<double> debfin = [-1.0, 1];

        Point2D p2d = new();
        Point3D p3d = new();
        {
            NCollection_HArray2<double> polynomialIntervalsPtr = new(1, nbElem, 1, 2);

            NCollection_HArray1<int> nbCoeffPtr = new(1, _maxSegment);

            int size = _maxSegment * (_maxDegree + 1) * _dimension;
            NCollection_HArray1<double> coeffPtr = new(1, size);

            coeffPtr.Init(0.0);

            NCollection_HArray1<double> intervallesPtr = new(1, nbElem + 1);

            intervallesPtr.ChangeArray1().Assign(theCurve.Knots());

            theCurve.GetPolynom(coeffPtr.ChangeArray1());

            int ii;

            for (ii = 1; ii <= nbElem; ii++)
            {
                nbCoeffPtr.SetValue(ii, theCurve.Degree(ii) + 1);
            }

            for (ii = polynomialIntervalsPtr.LowerRow(); ii <= polynomialIntervalsPtr.UpperRow(); ii++)
            {
                polynomialIntervalsPtr.SetValue(ii, 1, debfin[0]);
                polynomialIntervalsPtr.SetValue(ii, 2, debfin[1]);
            }
            CompPolynomialToPoles aConverter = new(nbElem,
                                                           _nivCont,
                                                           _dimension,
                                                           _maxDegree,
                                                           nbCoeffPtr,
                                                           coeffPtr,
                                                           polynomialIntervalsPtr,
                                                           intervallesPtr);
            if (aConverter.IsDone)
            {
                NCollection_HArray1<int> mults;
                int nbPoles = aConverter.NbPoles();
                NCollection_Array1<MultiPoint> tabMU = new(1, nbPoles);
                NCollection_Array2<double> poles = aConverter.Poles();
                _knots = new NCollection_HArray1<double>(aConverter.Knots());
                mults = new NCollection_HArray1<int>(aConverter.Multiplicities());

                for (ipole = poles.LowerRow(); ipole <= poles.UpperRow(); ipole++)
                {
                    int index = poles.LowerCol();
                    if (_nbP3d != 0)
                    {
                        for (jp3d = 1; jp3d <= _nbP3d; jp3d++)
                        {
                            p3d = p3d.WithX(poles.Value(ipole, index++));
                            p3d = p3d.WithY(poles.Value(ipole, index++));
                            p3d = p3d.WithZ(poles.Value(ipole, index++));
                            tabP3d.SetValue(jp3d, p3d);
                        }
                    }
                    if (_nbP2d != 0)
                    {
                        for (jp2d = 1; jp2d <= _nbP2d; jp2d++)
                        {
                            p2d = p2d.WithX(poles.Value(ipole, index++));
                            p2d = p2d.WithY(poles.Value(ipole, index++));
                            tabP2d.SetValue(jp2d, p2d);
                        }
                    }
                    if (_nbP2d != 0 && _nbP3d != 0)
                    {
                        MultiPoint multiPoint = new(tabP3d, tabP2d);
                        tabMU.SetValue(ipole, multiPoint);
                    }
                    else if (_nbP2d != 0)
                    {
                        MultiPoint multiPoint = new(tabP2d);
                        tabMU.SetValue(ipole, multiPoint);
                    }
                    else
                    {
                        MultiPoint multiPoint = new(tabP3d);
                        tabMU.SetValue(ipole, multiPoint);
                    }
                }
                MultiBSpCurve curve = new(tabMU, _knots.Array1(), mults.Array1());
                _mbSpCurve = curve;
                _isDone = true;
            }
        }
    }

    /// <summary>True if the creation is done and corresponds to the current fields.</summary>
    public bool IsCreated => _isCreated;

    /// <summary>True if the approximation is done and corresponds to the current fields.</summary>
    public bool IsDone => _isDone;

    /// <summary>True if the problem is over-constrained, in which case the approximation can't be done.</summary>
    public bool IsOverConstrained => _isOverConstr;

    /// <summary>
    /// The B-spline curves approximating the MultiLine (a copy sharing the poles). Throws StdFail_NotDone unless
    /// <see cref="IsDone"/>.
    /// </summary>
    public MultiBSpCurve Value()
    {
        if (!_isDone)
        {
            throw new NotDoneException();
        }
        return new MultiBSpCurve(_mbSpCurve);
    }

    /// <summary>The largest distance between the points of the MultiLine and the curves.</summary>
    public double MaxError()
    {
        if (!_isDone)
        {
            throw new NotDoneException();
        }
        return _maxError;
    }

    /// <summary>The index of the MultiPoint of <see cref="MaxError"/>.</summary>
    public int MaxErrorIndex()
    {
        if (!_isDone)
        {
            throw new NotDoneException();
        }
        return _maxErrorIndex;
    }

    /// <summary>The quadratic error: the sum of the squared distances between the points and the curves.</summary>
    public double QuadraticError()
    {
        if (!_isDone)
        {
            throw new NotDoneException();
        }
        return _criterium[0];
    }

    /// <summary>
    /// Writes into <paramref name="mat"/> the distances between the points of the MultiLine and the curves:
    /// mat(curve, point), rows from 1, columns from the matrix's lower column for the first point.
    /// </summary>
    public void Distance(Matrix mat)
    {
        if (!_isDone)
        {
            throw new NotDoneException();
        }
        int ipoint, jp2d, jp3d, index;
        NCollection_Array1<Point3D> tabP3d = new(1, Max(1, _nbP3d));
        NCollection_Array1<Point2D> tabP2d = new(1, Max(1, _nbP2d));
        int j0 = mat.LowerCol() - _firstPoint;

        Point2D p2d;
        Point3D p3d;

        Point3D pt3d;
        Point2D pt2d;

        for (ipoint = _firstPoint; ipoint <= _lastPoint; ipoint++)
        {
            index = 1;
            if (_nbP3d != 0)
            {
                MyLineTool.Value(_ssp, ipoint, tabP3d);

                for (jp3d = 1; jp3d <= _nbP3d; jp3d++)
                {
                    p3d = tabP3d.Value(jp3d);
                    _mbSpCurve.Value(index, _parameters.Value(ipoint), out pt3d);
                    mat[index++, j0 + ipoint] = p3d.DistanceTo(pt3d);
                }
            }
            if (_nbP2d != 0)
            {
                if (_nbP3d == 0)
                {
                    MyLineTool.Value(_ssp, ipoint, tabP2d);
                }
                else
                {
                    MyLineTool.Value(_ssp, ipoint, tabP3d, tabP2d);
                }
                for (jp2d = 1; jp2d <= _nbP2d; jp2d++)
                {
                    p2d = tabP2d.Value(jp2d);
                    _mbSpCurve.Value(index, _parameters.Value(ipoint), out pt2d);
                    mat[index++, j0 + ipoint] = p2d.DistanceTo(pt2d);
                }
            }
        }
    }

    /// <summary>The average distance between the points of the MultiLine and the curves.</summary>
    public double AverageError()
    {
        if (!_isDone)
        {
            throw new NotDoneException();
        }
        return _averageError;
    }

    /// <summary>The parameters of the points used by the approximation (the object's own array: don't modify).</summary>
    public NCollection_HArray1<double> Parameters()
    {
        if (!_isDone)
        {
            throw new NotDoneException();
        }
        return _parameters;
    }

    /// <summary>The knots used by the approximation (the object's own array: don't modify).</summary>
    public NCollection_HArray1<double> Knots()
    {
        if (!_isDone)
        {
            throw new NotDoneException();
        }
        return _knots;
    }

    /// <summary>The square roots of the three quality criteria (tension, flexion, jerk).</summary>
    public void Criterium(out double vFirstOrder, out double vSecondOrder, out double vThirdOrder)
    {
        if (!_isDone)
        {
            throw new NotDoneException();
        }
        vFirstOrder = _criterium[1];
        vSecondOrder = _criterium[2];
        vThirdOrder = _criterium[3];
    }

    /// <summary>The weights (shares summing to 1) of the three quality criteria.</summary>
    public void CriteriumWeight(out double percent1, out double percent2, out double percent3)
    {
        percent1 = _percent[0];
        percent2 = _percent[1];
        percent3 = _percent[2];
    }

    /// <summary>The maximum degree used in the approximation.</summary>
    public int MaxDegree() => _maxDegree;

    /// <summary>The maximum number of segments used in the approximation.</summary>
    public int MaxSegment() => _maxSegment;

    /// <summary>The continuity used in the approximation.</summary>
    public Continuity Continuity() => _continuity;

    /// <summary>Whether the approximation minimises the maximum error.</summary>
    public bool WithMinMax() => _withMinMax;

    /// <summary>Whether the approximation can insert knots.</summary>
    public bool WithCutting() => _withCutting;

    /// <summary>The tolerance used in the approximation.</summary>
    public double Tolerance() => _tolerance;

    /// <summary>The number of iterations used in the approximation.</summary>
    public int IterationCount => _nbIterations;

    // OCCT's Dump (iostream formatting of the state) is left out (rules, section 6).

    /// <summary>
    /// Sets the constraints (kept, shared) and recomputes the criterion; false if the problem is then
    /// over-constrained.
    /// </summary>
    public bool SetConstraints(NCollection_HArray1<ConstraintCouple> aConstraint)
    {
        _constraints = aConstraint;
        Init();
        return !_isOverConstr;
    }

    /// <summary>Sets (copies) the parameters used by the approximation.</summary>
    public void SetParameters(NCollection_HArray1<double> param)
    {
        _parameters.ChangeArray1().Assign(param.Array1());
    }

    /// <summary>Sets (copies) the knots used by the approximation; always true.</summary>
    public bool SetKnots(NCollection_HArray1<double> knots)
    {
        _knots.ChangeArray1().Assign(knots.Array1());
        return true;
    }

    /// <summary>Sets the maximum degree; false, changing nothing, if the problem would be over-constrained.</summary>
    public bool SetMaxDegree(int degree)
    {
        if (((degree - _nivCont) * _maxSegment - _nbPassPoints - 2 * _nbTangPoints - 3 * _nbCurvPoints) < 0)
        {
            return false;
        }
        else
        {
            _maxDegree = degree;

            InitSmoothCriterion();

            return true;
        }
    }

    /// <summary>Sets the maximum number of segments; false, changing nothing, if the problem would be over-constrained.</summary>
    public bool SetMaxSegment(int nbSegment)
    {
        if (_withCutting
            && ((_maxDegree - _nivCont) * nbSegment - _nbPassPoints - 2 * _nbTangPoints - 3 * _nbCurvPoints) < 0)
        {
            return false;
        }
        else
        {
            _maxSegment = nbSegment;
            return true;
        }
    }

    /// <summary>
    /// Sets the continuity (C0, C1 or C2; Standard_ConstructionError otherwise); false, changing nothing, if the
    /// problem would be over-constrained.
    /// </summary>
    public bool SetContinuity(Continuity c)
    {
        int nivCont = 0;
        switch (c)
        {
            case C0:
                nivCont = 0;
                break;
            case C1:
                nivCont = 1;
                break;
            case C2:
                nivCont = 2;
                break;
            default:
                throw new ConstructionErrorException();
        }
        if (((_maxDegree - nivCont) * _maxSegment - _nbPassPoints - 2 * _nbTangPoints - 3 * _nbCurvPoints) < 0)
        {
            return false;
        }
        else
        {
            _continuity = c;
            _nivCont = nivCont;

            InitSmoothCriterion();
            return true;
        }
    }

    /// <summary>Sets whether the approximation minimises the maximum error.</summary>
    public void SetWithMinMax(bool minMax)
    {
        _withMinMax = minMax;

        InitSmoothCriterion();
    }

    /// <summary>Sets whether the approximation can insert knots; false, changing nothing, if the problem would be over-constrained.</summary>
    public bool SetWithCutting(bool cutting)
    {
        if (!cutting)
        {
            if (((_maxDegree - _nivCont) * _knots.Length() - _nbPassPoints - 2 * _nbTangPoints
                 - 3 * _nbCurvPoints)
                < 0)
            {
                return false;
            }
            else
            {
                _withCutting = cutting;
                InitSmoothCriterion();
                return true;
            }
        }
        else
        {
            if (((_maxDegree - _nivCont) * _maxSegment - _nbPassPoints - 2 * _nbTangPoints
                 - 3 * _nbCurvPoints)
                < 0)
            {
                return false;
            }
            else
            {
                _withCutting = cutting;
                InitSmoothCriterion();
                return true;
            }
        }
    }

    /// <summary>
    /// Sets the weights of the three quality criteria (normalised to sum to 1). Throws Standard_DomainError if one
    /// is negative.
    /// </summary>
    public void SetCriteriumWeight(double percent1, double percent2, double percent3)
    {
        if (percent1 < 0 || percent2 < 0 || percent3 < 0)
        {
            throw new DomainErrorException();
        }
        double total = percent1 + percent2 + percent3;
        _percent[0] = percent1 / total;
        _percent[1] = percent2 / total;
        _percent[2] = percent3 / total;

        InitSmoothCriterion();
    }

    /// <summary>
    /// Sets the weight of the criterion <paramref name="order"/> (1 to 3; Standard_ConstructionError otherwise) and
    /// renormalises the three. Throws Standard_DomainError if <paramref name="percent"/> is negative.
    /// </summary>
    public void SetCriteriumWeight(int order, double percent)
    {
        if (percent < 0)
        {
            throw new DomainErrorException();
        }
        if (order < 1 || order > 3)
        {
            throw new ConstructionErrorException();
        }
        _percent[order - 1] = percent;
        double total = _percent[0] + _percent[1] + _percent[2];
        _percent[0] = _percent[0] / total;
        _percent[1] = _percent[1] / total;
        _percent[2] = _percent[2] / total;

        InitSmoothCriterion();
    }

    /// <summary>Sets the tolerance used in the approximation.</summary>
    public void SetTolerance(double tol)
    {
        _tolerance = tol;
        InitSmoothCriterion();
    }

    /// <summary>Sets the number of iterations used in the approximation.</summary>
    public void SetNbIterations(int iter)
    {
        _nbIterations = iter;
    }

    //====================== Private Methodes =============================//

    private void TheMotor(SmoothCriterion j, double wQuadratic, double wQuality, ref Curve theCurve,
                          NCollection_Array1<double> ecarts)
    {
        const double bigValue = 1.0e37, smallValue = 1.0e-6, smallestValue = 1.0e-9;

        NCollection_HArray1<double> currentTi, newTi, oldTi;
        NCollection_HArray2<int> dependence;
        bool lestim, toOptim, iscut;
        bool isnear = false, again = true;
        int nbEst, icdana, iter;
        // OCCT leaves NumPnt uninitialised: Project sets it only where a distance is positive (occt-issues.md).
        int numPnt = 0;
        int maxNbEst = 5;
        double[] vocri = [bigValue, bigValue, bigValue];
        double erold = bigValue, errmax = bigValue, errmoy, errqua;
        double[] valcri = new double[3];
        double cblong, lnold;
        int nbrPnt = _lastPoint - _firstPoint + 1;
        int nbrConstraint = _nbPassPoints + _nbTangPoints + _nbCurvPoints;
        Curve cCurrent, cOld;
        Curve? cNew = null;
        double epsLength = smallValue;
        double epsDeg;

        double e1, e2, e3;
        double j1min, j2min, j3min;
        int iprog;

        // (0) Init

        j.GetEstimation(out e1, out e2, out e3);
        j1min = 1.0e-8;
        j2min = j3min = (e1 + 1.0e-8) * 1.0e-6;

        if (e1 < j1min)
        {
            e1 = j1min; // Like in
        }
        if (e2 < j2min)
        {
            e2 = j2min; // MOTLIS
        }
        if (e3 < j3min)
        {
            e3 = j3min;
        }

        j.SetEstimation(e1, e2, e3);

        cCurrent = theCurve;
        currentTi = new NCollection_HArray1<double>(1, _parameters.Length());
        currentTi.ChangeArray1().Assign(_parameters.Array1());
        oldTi = new NCollection_HArray1<double>(1, currentTi.Length());
        oldTi.ChangeArray1().Assign(currentTi.Array1());
        cOld = cCurrent;
        lnold = cblong = j.EstLength();
        dependence = j.DependenceTable();

        j.SetCurve(cCurrent);
        Assembly theAssembly = new(dependence.Array2(), j.AssemblyTable());

        //============        Optimization      ============================
        while (again)
        {
            // (1) Loop  Optimization / Estimation
            lestim = true;
            bool lconst = true;
            nbEst = 0;

            j.SetCurve(cCurrent);

            while (lestim)
            {
                //     (1.1) Curve's Optimization.
                epsLength = smallValue * cblong / nbrPnt;
                cNew = new Curve(cCurrent.Dimension(), cCurrent.ElementCount, cCurrent.Base(), epsLength);
                cNew.Knots().Assign(cCurrent.Knots());

                j.SetParameters(currentTi);
                epsDeg = Min(wQuality * .1, cblong * .001);

                Optimization(j, theAssembly, lconst, epsDeg, cNew, currentTi.Array1());

                lconst = false;

                //        (1.2) calculation of quality criteria and improvement
                //              of estimation.
                icdana = j.QualityValues(j1min, j2min, j3min, out valcri[0], out valcri[1], out valcri[2]);

                if (icdana > 0)
                {
                    lconst = true;
                }

                j.ErrorValues(out errmax, out errqua, out errmoy);

                isnear = ((Math.Sqrt(errqua / nbrPnt) < 2 * wQuality) && (_nbIterations > 1));

                //       (1.3) Optimization of ti by orthogonal projection
                //             and calculation of the error at points.

                if (isnear)
                {
                    newTi = new NCollection_HArray1<double>(1, currentTi.Length());
                    Project(cNew, currentTi.Array1(), newTi.ChangeArray1(), ecarts, ref numPnt, out errmax, out errqua,
                            out errmoy, 2);
                }
                else
                {
                    newTi = currentTi;
                }

                //        (1.4) Progression's test
                iprog = 0;
                if ((erold > wQuality) && (errmax < 0.95 * erold))
                {
                    iprog++;
                }
                if ((erold > wQuality) && (errmax < 0.8 * erold))
                {
                    iprog++;
                }
                if ((erold > wQuality) && (errmax < wQuality))
                {
                    iprog++;
                }
                if ((erold > wQuality) && (errmax < 0.99 * erold) && (errmax < 1.1 * wQuality))
                {
                    iprog++;
                }
                if (valcri[0] < 0.975 * vocri[0])
                {
                    iprog++;
                }
                if (valcri[0] < 0.9 * vocri[0])
                {
                    iprog++;
                }
                if (valcri[1] < 0.95 * vocri[1])
                {
                    iprog++;
                }
                if (valcri[1] < 0.8 * vocri[1])
                {
                    iprog++;
                }
                if (valcri[2] < 0.95 * vocri[2])
                {
                    iprog++;
                }
                if (valcri[2] < 0.8 * vocri[2])
                {
                    iprog++;
                }
                if ((vocri[1] > smallestValue) && (vocri[2] > smallestValue))
                {
                    if ((valcri[1] / vocri[1] + 2 * valcri[2] / vocri[2]) < 2.8)
                    {
                        iprog++;
                    }
                }

                if (iprog < 2 && nbEst == 0)
                {
                    //             (1.5) Invalidation of new knots.
                    valcri[0] = vocri[0];
                    valcri[1] = vocri[1];
                    valcri[2] = vocri[2];
                    errmax = erold;
                    cblong = lnold;
                    cCurrent = cOld;
                    currentTi = oldTi;

                    goto L8000; // exit
                }

                vocri[0] = valcri[0];
                vocri[1] = valcri[1];
                vocri[2] = valcri[2];
                lnold = cblong;
                erold = errmax;

                cCurrent = cNew;
                currentTi = newTi;

                //       (1.6) Test if the Estimations seems  OK, else repeat
                nbEst++;
                lestim = ((nbEst < maxNbEst) && (icdana == 2) && (iprog > 0));

                if (lestim && isnear)
                {
                    //           (1.7) Optimization of ti by ACR.

                    StableSort(currentTi);

                    int decima = 4;

                    cCurrent.Length(0.0, 1.0, out cblong);
                    j.EstLength() = cblong;

                    ACR(cCurrent, currentTi.ChangeArray1(), decima);
                    lconst = true;
                }
            }

            //     (2) loop of parametric / geometric optimization

            iter = 1;
            toOptim = ((iter < _nbIterations) && (isnear));

            while (toOptim)
            {
                iter++;
                //     (2.1) Save current results
                vocri[0] = valcri[0];
                vocri[1] = valcri[1];
                vocri[2] = valcri[2];
                erold = errmax;
                lnold = cblong;
                cOld = cCurrent;
                oldTi.ChangeArray1().Assign(currentTi.Array1());

                //     (2.2) Optimization of ti by ACR.

                StableSort(currentTi);

                int decima = 4;

                cCurrent.Length(0.0, 1.0, out cblong);
                j.EstLength() = cblong;

                ACR(cCurrent, currentTi.ChangeArray1(), decima);
                lconst = true;

                //      (2.3) Optimization of curves
                epsLength = smallValue * cblong / nbrPnt;

                cNew = new Curve(cCurrent.Dimension(), cCurrent.ElementCount, cCurrent.Base(), epsLength);
                cNew.Knots().Assign(cCurrent.Knots());

                j.SetParameters(currentTi);

                epsDeg = Min(wQuality * .1, cblong * .001);
                Optimization(j, theAssembly, lconst, epsDeg, cNew, currentTi.Array1());

                cCurrent = cNew;

                //      (2.4) calculation of quality criteria and improvement
                //             of estimation.

                icdana = j.QualityValues(j1min, j2min, j3min, out valcri[0], out valcri[1], out valcri[2]);
                if (icdana > 0)
                {
                    lconst = true;
                }

                j.GetEstimation(out e1, out e2, out e3);
                //       (2.5) Optimization of ti by orthogonal projection

                newTi = new NCollection_HArray1<double>(1, currentTi.Length());
                Project(cCurrent, currentTi.Array1(), newTi.ChangeArray1(), ecarts, ref numPnt, out errmax, out errqua,
                        out errmoy, 2);

                //       (2.6)  Test de non regression

                int iregre = 0;
                if (nbrConstraint < nbrPnt)
                {
                    if ((errmax > wQuality) && (errmax > 1.05 * erold))
                    {
                        iregre++;
                    }
                    if ((errmax > wQuality) && (errmax > 2 * erold))
                    {
                        iregre++;
                    }
                    if ((erold > wQuality) && (errmax <= 0.5 * erold))
                    {
                        iregre--;
                    }
                }
                double E1, E2, E3;
                j.GetEstimation(out E1, out E2, out E3);
                if ((valcri[0] > E1) && (valcri[0] > 1.1 * vocri[0]))
                {
                    iregre++;
                }
                if ((valcri[1] > E2) && (valcri[1] > 1.1 * vocri[1]))
                {
                    iregre++;
                }
                if ((valcri[2] > E3) && (valcri[2] > 1.1 * vocri[2]))
                {
                    iregre++;
                }

                if (iregre >= 2)
                {
                    // (2.7) restore the previous iteration
                    valcri[0] = vocri[0];
                    valcri[1] = vocri[1];
                    valcri[2] = vocri[2];
                    errmax = erold;
                    cblong = lnold;
                    cCurrent = cOld;
                    currentTi.ChangeArray1().Assign(oldTi.Array1());
                    toOptim = false;
                }
                else
                { // Iteration is Ok.
                    cCurrent = cNew;
                    currentTi = newTi;
                }
                if (iter >= _nbIterations)
                {
                    toOptim = false;
                }
            }

            // (3) Optional splitting

            if ((cCurrent.ElementCount < _maxSegment) && _withCutting)
            {
                //    (3.1) Sauvgarde de l'etat precedent
                vocri[0] = valcri[0];
                vocri[1] = valcri[1];
                vocri[2] = valcri[2];
                erold = errmax;
                cOld = cCurrent;
                oldTi.ChangeArray1().Assign(currentTi.Array1());

                //       (3.2) Arrange the ti: Sort + rescale to (0,1)
                //         ---> Sort to ensure proper ordering afterwards.

                StableSort(currentTi);

                if ((currentTi.Value(1) != 0.0) || (currentTi.Value(nbrPnt) != 1.0))
                {
                    double t, delatT = 1.0 / (currentTi.Value(nbrPnt) - currentTi.Value(1));
                    for (int ii = 2; ii < nbrPnt; ii++)
                    {
                        t = (currentTi.Value(ii) - currentTi.Value(1)) * delatT;
                        currentTi.SetValue(ii, t);
                    }
                    currentTi.SetValue(1, 0.0);
                    currentTi.SetValue(nbrPnt, 1.0);
                }

                //       (3.3) Insert new Knots

                SplitCurve(cCurrent, currentTi.Array1(), epsLength, ref cNew, out iscut);
                if (!iscut)
                {
                    again = false;
                }
                else
                {
                    cCurrent = cNew!;
                    // New Knots => New Assembly.
                    j.SetCurve(cNew!);
                    theAssembly = new Assembly(dependence.Array2(), j.AssemblyTable());
                }
            }
            else
            {
                again = false;
            }
        }

    //    ================   Great loop end   ===================

    L8000:

        // (4) Compute the best Error.
        newTi = new NCollection_HArray1<double>(1, currentTi.Length());
        Project(cCurrent, currentTi.Array1(), newTi.ChangeArray1(), ecarts, ref numPnt, out errmax, out errqua,
                out errmoy, 10);

        // (5) field's update

        theCurve = cCurrent;
        j.EstLength() = cblong;
        _parameters.ChangeArray1().Assign(newTi.Array1());
        _criterium[0] = errqua;
        _criterium[1] = Math.Sqrt(valcri[0]);
        _criterium[2] = Math.Sqrt(valcri[1]);
        _criterium[3] = Math.Sqrt(valcri[2]);
        _maxError = errmax;
        _maxErrorIndex = numPnt;
        if (nbrPnt > nbrConstraint)
        {
            _averageError = errmoy / (nbrPnt - nbrConstraint);
        }
        else
        {
            _averageError = errmoy / nbrConstraint;
        }
    }

    private void Optimization(SmoothCriterion j, Assembly a, bool toAssemble, double epsDeg,
                              Curve curve, NCollection_Array1<double> parameters)
    {
        int mxDeg = curve.Base().WorkDegree(), nbElm = curve.ElementCount, nbDim = curve.Dimension();

        NCollection_HArray2<NCollection_HArray1<int>> assTable;

        Matrix h = new(0, mxDeg, 0, mxDeg);
        math_Vector g = new(0, mxDeg), sol = new(1, a.GlobVarCount);

        int el, dim;

        assTable = a.AssemblyTable();
        int nbConstr = _nbPassPoints + _nbTangPoints + _nbCurvPoints;

        double cblong = j.EstLength();

        // Updating Assembly
        if (toAssemble)
        {
            a.NullifyMatrix();
        }
        a.NullifyVector();

        for (el = 1; el <= nbElm; el++)
        {
            if (toAssemble)
            {
                j.Hessian(el, 1, 1, h);
                for (dim = 1; dim <= nbDim; dim++)
                {
                    a.AddMatrix(el, dim, dim, h);
                }
            }

            for (dim = 1; dim <= nbDim; dim++)
            {
                j.Gradient(el, dim, g);
                a.AddVector(el, dim, g);
            }
        }

        // Solution of system
        if (toAssemble)
        {
            if (nbConstr != 0)
            { // Treatment of constraints
                AssemblingConstraints(curve, parameters, cblong, a);
            }
            a.Solve();
        }
        a.Solution(sol);

        // Updating J
        j.SetCurve(curve);
        j.InputVector(sol, assTable);

        // Updating Curve and reduction of degree

        int newdeg;
        double maxError;

        if (nbConstr == 0)
        {
            for (el = 1; el <= nbElm; el++)
            {
                curve.ReduceDegree(el, epsDeg, out newdeg, out maxError);
            }
        }
        else
        {
            NCollection_Array1<double> tabInt = curve.Knots();
            int icnt = 1, p0 = parameters.Lower() - _firstPoint, point;
            for (el = 1; el <= nbElm; el++)
            {
                while ((icnt < nbConstr) && (parameters[p0 + _typConstraints.Value(2 * icnt - 1)] <= tabInt[el]))
                {
                    icnt++;
                }
                point = p0 + _typConstraints.Value(2 * icnt - 1);
                if (parameters[point] <= tabInt[el] || parameters[point] >= tabInt[el + 1])
                {
                    curve.ReduceDegree(el, epsDeg, out newdeg, out maxError);
                }
                else if (curve.Degree(el) < mxDeg)
                {
                    curve.SetDegree(el, mxDeg);
                }
            }
        }
    }

    private void Project(Curve c, NCollection_Array1<double> ti, NCollection_Array1<double> projTi,
                         NCollection_Array1<double> distance, ref int numPoints, out double maxErr, out double quaErr,
                         out double aveErr, int nbIterations = 2)
    {
        // Initialisation

        const double seuil = 1.0e-9, eps = 1.0e-12;

        maxErr = quaErr = aveErr = 0.0;

        int ipnt, nItCv, iter, i, i0 = -_dimension, d0 = distance.Lower() - 1;

        double tNew, dist, t0, dist0, f1, f2, aux, df, ecart;

        bool enCour;

        NCollection_Array1<double> valOfC = new(1, _dimension), firstDerOfC = new(1, _dimension),
                                   secndDerOfC = new(1, _dimension);

        for (ipnt = 1; ipnt <= projTi.Length(); ipnt++)
        {
            i0 += _dimension;

            tNew = ti[ipnt];

            enCour = true;
            nItCv = 0;
            iter = 0;
            c.D0(tNew, valOfC);

            dist = 0;
            for (i = 1; i <= _dimension; i++)
            {
                aux = valOfC[i] - _tabPoints.Value(i0 + i);
                dist += aux * aux;
            }
            dist = Math.Sqrt(dist);

            // ------- Newton's method for solving (C'(t),C(t) - P) = 0

            while (enCour)
            {
                iter++;
                t0 = tNew;
                dist0 = dist;

                c.D2(tNew, secndDerOfC);
                c.D1(tNew, firstDerOfC);

                f1 = f2 = 0.0;
                for (i = 1; i <= _dimension; i++)
                {
                    aux = valOfC[i] - _tabPoints.Value(i0 + i);
                    df = firstDerOfC[i];
                    f1 += aux * df;                       // (C'(t),C(t) - P)
                    f2 += df * df + aux * secndDerOfC[i]; // ((C'(t),C(t) - P))'
                }

                if (Math.Abs(f2) < eps)
                {
                    enCour = false;
                }
                else
                {
                    // Formula of Newton x(k+1) = x(k) - F(x(k))/F'(x(k))
                    tNew -= f1 / f2;
                    if (tNew < 0.0)
                    {
                        tNew = 0.0;
                    }
                    if (tNew > 1.0)
                    {
                        tNew = 1.0;
                    }

                    // Analysis of result

                    c.D0(tNew, valOfC);

                    dist = 0;
                    for (i = 1; i <= _dimension; i++)
                    {
                        aux = valOfC[i] - _tabPoints.Value(i0 + i);
                        dist += aux * aux;
                    }
                    dist = Math.Sqrt(dist);

                    ecart = dist0 - dist;

                    if (ecart <= -seuil)
                    {
                        // No improvement, stop here
                        enCour = false;
                        tNew = t0;
                        dist = dist0;
                    }
                    else if (ecart <= seuil)
                    {
                        // Convergence
                        nItCv++;
                    }
                    else
                    {
                        nItCv = 0;
                    }

                    if ((nItCv >= 2) || (iter >= nbIterations))
                    {
                        enCour = false;
                    }
                }
            }

            projTi[ipnt] = tNew;
            distance[d0 + ipnt] = dist;
            if (dist > maxErr)
            {
                maxErr = dist;
                numPoints = ipnt;
            }
            quaErr += dist * dist;
            aveErr += dist;
        }

        // OCCT: when no distance is positive, NumPoints keeps the caller's value and is shifted again (occt-issues.md).
        numPoints = numPoints + _firstPoint - 1; // Setting NumPoints to interval [myFirstPoint, myLastPoint]
    }

    private void ACR(Curve curve, NCollection_Array1<double> ti, int decima)
    {
        const double eps = 1.0e-8;

        NCollection_Array1<double> knots = curve.Knots();
        int nbrPnt = ti.Length(), tiFirst = ti.Lower(), tiLast = ti.Upper(), kFirst = knots.Lower(),
            kLast = knots.Upper();

        double cbLong, deltaT, vTest, uNew, uOld, du, tPara, tOld, dtInv, ratio;
        int ipnt, ii, iElm, iOld, pOld, pCnt, iCnt = 0;
        int nbCntr = _nbPassPoints + _nbTangPoints + _nbCurvPoints;

        //     (1) Compute the curve length

        curve.Length(ti[tiFirst], ti[tiLast], out cbLong);

        //     (2)  Place the arc-length parameterization in Ti

        if (nbrPnt >= 2)
        {
            //     (2.0) Initialisation
            deltaT = (ti[tiLast] - ti[tiFirst]) / decima;
            vTest = ti[tiFirst] + deltaT;

            if (nbCntr > 0)
            {
                pCnt = _typConstraints.Value(1) - _firstPoint + tiFirst;
                iCnt = 1;
            }
            else
            {
                pCnt = tiLast + 1;
            }

            uOld = 0.0;

            tOld = ti[tiFirst];
            pOld = tiFirst;

            iElm = kFirst;
            iOld = iElm;

            ti[tiFirst] = 0.0;

            for (ipnt = tiFirst + 1; ipnt <= tiLast; ipnt++)
            {
                while ((iCnt <= nbCntr) && (pCnt < ipnt))
                {
                    iCnt++;
                    // OCCT reads the point of constraint NbCntr + 1, past the constraints (occt-issues.md); the port
                    // takes a point no ipnt can equal, as OCCT's value is in practice.
                    pCnt = iCnt <= nbCntr ? _typConstraints.Value(2 * iCnt - 1) - _firstPoint + tiFirst : tiLast + 1;
                }

                tPara = ti[ipnt];

                if (tPara >= vTest || pCnt == ipnt)
                {
                    if (ti[tiLast] - tPara <= 1.0e-2 * deltaT)
                    {
                        ipnt = tiLast;
                        tPara = ti[ipnt];
                    }
                    //        (2.2), (2.3) Compute the curve length
                    curve.Length(ti[tiFirst], tPara, out uNew);

                    uNew /= cbLong;

                    while (knots[iElm + 1] < tPara && iElm < kLast - 1)
                    {
                        iElm++;
                    }

                    //         (2.4) Update the splitting parameters
                    dtInv = 1.0 / (tPara - tOld);
                    du = uNew - uOld;

                    for (ii = iOld + 1; ii <= iElm; ii++)
                    {
                        ratio = (knots[ii] - tOld) * dtInv;
                        knots[ii] = uOld + ratio * du;
                    }

                    //           (2.5) Update the point parameters.

                    // Very strange loop, because it never works (POld+1 > ipnt-1)
                    for (ii = pOld + 1; ii <= ipnt - 1; ii++)
                    {
                        ratio = (ti[ii] - tOld) * dtInv;
                        ti[ii] = uOld + ratio * du;
                    }

                    ti[ipnt] = uNew;

                    uOld = uNew;
                    iOld = iElm;
                    tOld = tPara;
                    pOld = ipnt;
                }
                //        --> New parametric threshold for decimation

                if (tPara >= vTest)
                {
                    vTest += Math.Ceiling((tPara - vTest + eps) / deltaT) * deltaT;
                    if (vTest > 1.0 - eps)
                    {
                        vTest = 1.0;
                    }
                }
            }
        }

        //     --- Adjust the extreme values

        ti[tiFirst] = 0.0;
        ti[tiLast] = 1.0;
        ii = tiLast - 1;
        while (ti[ii] > knots[kLast])
        {
            ti[ii] = 1.0;
            --ii;
        }
        knots[kFirst] = 0.0;
        knots[kLast] = 1.0;
    }

    private static int NearIndex(double t, NCollection_Array1<double> tabPar, double eps, out int flag)
    {
        int loi = tabPar.Lower(), upi = tabPar.Upper();

        flag = 0;

        if (t < tabPar[loi])
        {
            flag = -1;
            return loi;
        }
        if (t > tabPar[upi])
        {
            flag = 1;
            return upi;
        }

        int ibeg = loi, ifin = upi, imidl;

        while (ibeg + 1 != ifin)
        {
            imidl = (ibeg + ifin) / 2;
            if ((t >= tabPar[ibeg]) && (t <= tabPar[imidl]))
            {
                ifin = imidl;
            }
            else
            {
                ibeg = imidl;
            }
        }

        if (Math.Abs(t - tabPar[ifin]) < eps)
        {
            return ifin;
        }

        return ibeg;
    }

    // Calculates the new knots of the elements whose degree is Deg.
    private static void GettingKnots(NCollection_Array1<double> tabPar, Curve inCurve, int deg, ref int nbElm,
                                     NCollection_Array1<double> newKnots)
    {
        const double eps = 1.0e-12;

        NCollection_Array1<double> oldKnots = inCurve.Knots();
        int nbMaxOld = inCurve.ElementCount;
        int nbMax = newKnots.Upper(), ipt, ipt1, ipt2;
        int el = 0, i1 = oldKnots.Lower(), i0 = i1 - 1, flag;
        double tPar;

        while ((nbElm < nbMax) && (el < nbMaxOld))
        {
            el++;
            i0++;
            i1++; // i0, i1 are indexes of left and right knots of element el

            if (inCurve.Degree(el) == deg)
            {
                nbElm++;

                ipt1 = NearIndex(oldKnots[i0], tabPar, eps, out flag);
                if (flag != 0)
                {
                    ipt1 = tabPar.Lower();
                }
                ipt2 = NearIndex(oldKnots[i1], tabPar, eps, out flag);
                if (flag != 0)
                {
                    ipt2 = tabPar.Upper();
                }

                if (ipt2 - ipt1 >= 1)
                {
                    ipt = (ipt1 + ipt2) / 2;
                    if (2 * ipt == ipt1 + ipt2)
                    {
                        tPar = 2.0 * tabPar[ipt];
                    }
                    else
                    {
                        tPar = tabPar[ipt] + tabPar[ipt + 1];
                    }

                    newKnots[nbElm] = (oldKnots[i0] + oldKnots[i1] + tPar) / 4.0;
                }
                else
                {
                    newKnots[nbElm] = (oldKnots[i0] + oldKnots[i1]) / 2.0;
                }
            }
        }
    }

    private void SplitCurve(Curve inCurve, NCollection_Array1<double> ti, double curveTol,
                            ref Curve? outCurve, out bool iscut)
    {
        int nbElmOld = inCurve.ElementCount;

        if (nbElmOld >= _maxSegment)
        {
            iscut = false;
            return;
        }
        int nbElm = nbElmOld;
        NCollection_Array1<double> newKnots = new(nbElm + 1, _maxSegment);
        GettingKnots(ti, inCurve, inCurve.Base().WorkDegree(), ref nbElm, newKnots);
        GettingKnots(ti, inCurve, inCurve.Base().WorkDegree() - 1, ref nbElm, newKnots);
        if (nbElm > nbElmOld)
        {
            iscut = true;

            outCurve = new Curve(inCurve.Dimension(), nbElm, inCurve.Base(), curveTol);
            NCollection_Array1<double> outKnots = outCurve.Knots();
            NCollection_Array1<double> inKnots = inCurve.Knots();

            int i, i0 = outKnots.Lower();
            for (i = inKnots.Lower(); i <= inKnots.Upper(); i++)
            {
                outKnots[i] = inKnots[i];
            }
            for (i = nbElmOld + 1; i <= nbElm; i++)
            {
                outKnots[i + i0] = newKnots[i];
            }

            // C++: std::sort. The values are knots, distinct or equal, so a stable sort gives the same array.
            StableSort(outKnots);
        }
        else
        {
            iscut = false;
        }
    }

    private void InitSmoothCriterion()
    {
        const double eps2 = 1.0e-6, eps3 = 1.0e-9;

        double length;

        InitParameters(out length);

        _smoothCriterion.SetParameters(_parameters);

        double e1, e2, e3;

        InitCriterionEstimations(length, out e1, out e2, out e3);
        _smoothCriterion.EstLength() = length;
        _smoothCriterion.SetEstimation(e1, e2, e3);

        double wQuadratic, wQuality;

        if (!_withMinMax && _tolerance != 0.0)
        {
            wQuality = _tolerance;
        }
        else if (_tolerance == 0.0)
        {
            wQuality = 1.0;
        }
        else
        {
            wQuality = Max(_tolerance, eps2 * length);
        }

        int nbConstr = _nbPassPoints + _nbTangPoints + _nbCurvPoints;
        wQuadratic = Math.Sqrt((double)(_nbPoints - nbConstr)) * wQuality;
        if (wQuadratic > eps3)
        {
            wQuadratic = 1.0 / wQuadratic;
        }

        if (wQuadratic == 0.0)
        {
            wQuadratic = Max(Math.Sqrt(e1), 1.0);
        }

        _smoothCriterion.SetWeight(wQuadratic, wQuality, _percent[0], _percent[1], _percent[2]);

        HermitJacobi theBase = new(_maxDegree, _continuity);
        Curve theCurve;
        int nbElem;
        double curvTol = eps2 * length / _nbPoints;

        // Subdivision of the interval according to constraints
        if (_withCutting && nbConstr != 0)
        {
            InitCutting(theBase, curvTol, out theCurve);
        }
        else
        {
            nbElem = 1;
            theCurve = new Curve(_dimension, nbElem, theBase, curvTol);
            theCurve.Knots().SetValue(theCurve.Knots().Lower(), _parameters.Value(_firstPoint));
            theCurve.Knots().SetValue(theCurve.Knots().Upper(), _parameters.Value(_lastPoint));
        }

        _smoothCriterion.SetCurve(theCurve);
    }

    private void InitParameters(out double length)
    {
        const double eps1 = 1.0e-7 * .01; // Precision::Confusion() * .01

        double aux, dist;
        int i, i0, i1 = 0, ipoint;

        length = 0.0;
        _parameters.SetValue(_firstPoint, length);

        for (ipoint = _firstPoint + 1; ipoint <= _lastPoint; ipoint++)
        {
            i0 = i1;
            i1 += _dimension;
            dist = 0;
            for (i = 1; i <= _dimension; i++)
            {
                aux = _tabPoints.Value(i1 + i) - _tabPoints.Value(i0 + i);
                dist += aux * aux;
            }
            length += Math.Sqrt(dist);
            _parameters.SetValue(ipoint, length);
        }

        if (length <= eps1)
        {
            throw new ConstructionErrorException("AppDef_Variational::InitParameters");
        }

        for (ipoint = _firstPoint + 1; ipoint <= _lastPoint - 1; ipoint++)
        {
            _parameters.SetValue(ipoint, _parameters.Value(ipoint) / length);
        }

        _parameters.SetValue(_lastPoint, 1.0);

        // With few points there is likely underestimation ...
        if (_nbPoints < 10)
        {
            length *= (1.0 + 0.1 / (_nbPoints - 1));
        }
    }

    private void InitCriterionEstimations(double length, out double e1, out double e2, out double e3)
    {
        e1 = length * length;

        const double eps1 = 1.0e-7 * .01; // Precision::Confusion() * .01

        math_Vector vTang1 = new(1, _dimension), vTang2 = new(1, _dimension), vTang3 = new(1, _dimension),
                    vScnd1 = new(1, _dimension), vScnd2 = new(1, _dimension), vScnd3 = new(1, _dimension);

        // ========== Treatment of first point =================

        int ipnt = _firstPoint;

        EstTangent(ipnt, vTang1);
        ipnt++;
        EstTangent(ipnt, vTang2);
        ipnt++;
        EstTangent(ipnt, vTang3);

        ipnt = _firstPoint;
        EstSecnd(ipnt, vTang1, vTang2, length, vScnd1);
        ipnt++;
        EstSecnd(ipnt, vTang1, vTang3, length, vScnd2);

        //  Modified by skv - Fri Apr  8 14:58:12 2005 OCC8559 Begin
        int ind = ipnt;
        double delta = .5 * (_parameters.Value(ind) - _parameters.Value(--ipnt));
        //  Modified by skv - Fri Apr  8 14:58:12 2005 OCC8559 End

        if (delta <= eps1)
        {
            delta = 1.0;
        }

        e2 = vScnd1.Norm2() * delta;

        e3 = (delta > eps1) ? vScnd2.Subtracted(vScnd1).Norm2() / (4.0 * delta) : 0.0;
        // ========== Treatment of internal points =================

        int currPoint = 2;

        for (ipnt = _firstPoint + 1; ipnt < _lastPoint; ipnt++)
        {
            delta = .5 * (_parameters.Value(ipnt + 1) - _parameters.Value(ipnt - 1));

            if (currPoint == 1)
            {
                if (ipnt + 1 != _lastPoint)
                {
                    EstTangent(ipnt + 2, vTang3);
                    EstSecnd(ipnt + 1, vTang1, vTang3, length, vScnd2);
                }
                else
                {
                    EstSecnd(ipnt + 1, vTang1, vTang2, length, vScnd2);
                }

                e2 += vScnd1.Norm2() * delta;
                e3 += (delta > eps1) ? vScnd2.Subtracted(vScnd3).Norm2() / (4.0 * delta) : 0.0;
            }
            else if (currPoint == 2)
            {
                if (ipnt + 1 != _lastPoint)
                {
                    EstTangent(ipnt + 2, vTang1);
                    EstSecnd(ipnt + 1, vTang2, vTang1, length, vScnd3);
                }
                else
                {
                    EstSecnd(ipnt + 1, vTang2, vTang3, length, vScnd3);
                }

                e2 += vScnd2.Norm2() * delta;
                e3 += (delta > eps1) ? vScnd3.Subtracted(vScnd1).Norm2() / (4.0 * delta) : 0.0;
            }
            else
            {
                if (ipnt + 1 != _lastPoint)
                {
                    EstTangent(ipnt + 2, vTang2);
                    EstSecnd(ipnt + 1, vTang3, vTang2, length, vScnd1);
                }
                else
                {
                    EstSecnd(ipnt + 1, vTang3, vTang1, length, vScnd1);
                }

                e2 += vScnd3.Norm2() * delta;
                e3 += (delta > eps1) ? vScnd1.Subtracted(vScnd2).Norm2() / (4.0 * delta) : 0.0;
            }

            currPoint++;
            if (currPoint == 4)
            {
                currPoint = 1;
            }
        }

        // ========== Treatment of last point =================

        delta = .5 * (_parameters.Value(_lastPoint) - _parameters.Value(_lastPoint - 1));
        if (delta <= eps1)
        {
            delta = 1.0;
        }

        double aux;

        if (currPoint == 1)
        {
            e2 += vScnd1.Norm2() * delta;
            aux = vScnd1.Subtracted(vScnd3).Norm2();
            e3 += (delta > eps1) ? aux / (4.0 * delta) : aux;
        }
        else if (currPoint == 2)
        {
            e2 += vScnd2.Norm2() * delta;
            aux = vScnd2.Subtracted(vScnd1).Norm2();
            e3 += (delta > eps1) ? aux / (4.0 * delta) : aux;
        }
        else
        {
            e2 += vScnd3.Norm2() * delta;
            aux = vScnd3.Subtracted(vScnd2).Norm2();
            e3 += (delta > eps1) ? aux / (4.0 * delta) : aux;
        }

        aux = length * length;

        e2 *= aux;
        e3 *= aux;
    }

    private void EstTangent(int ipnt, math_Vector vTang)
    {
        int i;
        const double eps1 = 1.0e-7 * .01; // Precision::Confusion() * .01
        const double epsNorm = 1.0e-9;

        double wpnt = 1.0;

        if (ipnt == _firstPoint)
        {
            // Estimation at first point
            if (_nbPoints < 3)
            {
                wpnt = 0.0;
            }
            else
            {
                int adr1 = 1, adr2 = adr1 + _dimension, adr3 = adr2 + _dimension;

                math_Vector pnt1 = new(_tabPoints, adr1, 1, _dimension);
                math_Vector pnt2 = new(_tabPoints, adr2, 1, _dimension);
                math_Vector pnt3 = new(_tabPoints, adr3, 1, _dimension);

                // Parabolic interpolation
                // if we have parabolic interpolation: F(t) = A0 + A1*t + A2*t*t,
                // first derivative for t=0 is A1 = ((d2-1)*P1 + P2 - d2*P3)/(d*(1-d))
                //       d= |P2-P1|/(|P2-P1|+|P3-P2|), d2 = d*d
                double v1 = pnt2.Subtracted(pnt1).Norm();
                double v2 = 0.0;
                if (v1 > eps1)
                {
                    v2 = pnt3.Subtracted(pnt2).Norm();
                }
                if (v2 > eps1)
                {
                    double d = v1 / (v1 + v2), d1;
                    d1 = 1.0 / (d * (1 - d));
                    d *= d;
                    vTang.Initialized(((d - 1.0) * pnt1 + pnt2 - d * pnt3) * d1);
                }
                else
                {
                    // Simple 2-point estimation

                    vTang.Initialized(pnt2 - pnt1);
                }
            }
        }
        else if (ipnt == _lastPoint)
        {
            // Estimation at last point
            if (_nbPoints < 3)
            {
                wpnt = 0.0;
            }
            else
            {
                // OCCT: the addresses assume myFirstPoint == 1 (occt-issues.md).
                int adr1 = (_lastPoint - 3) * _dimension + 1, adr2 = adr1 + _dimension, adr3 = adr2 + _dimension;

                math_Vector pnt1 = new(_tabPoints, adr1, 1, _dimension);
                math_Vector pnt2 = new(_tabPoints, adr2, 1, _dimension);
                math_Vector pnt3 = new(_tabPoints, adr3, 1, _dimension);

                // Parabolic interpolation
                // if we have parabolic interpolation: F(t) = A0 + A1*t + A2*t*t,
                // first derivative for t=1 is 2*A2 + A1 = ((d2+1)*P1 - P2 - d2*P3)/(d*(1-d))
                //       d= |P2-P1|/(|P2-P1|+|P3-P2|), d2 = d*(d-2)
                double v1 = pnt2.Subtracted(pnt1).Norm();
                double v2 = 0.0;
                if (v1 > eps1)
                {
                    v2 = pnt3.Subtracted(pnt2).Norm();
                }
                if (v2 > eps1)
                {
                    double d = v1 / (v1 + v2), d1;
                    d1 = 1.0 / (d * (1 - d));
                    d *= d - 2;
                    vTang.Initialized(((d + 1.0) * pnt1 - pnt2 - d * pnt3) * d1);
                }
                else
                {
                    // Simple 2-point estimation

                    vTang.Initialized(pnt3 - pnt2);
                }
            }
        }
        else
        {
            int adr1 = (ipnt - _firstPoint - 1) * _dimension + 1, adr2 = adr1 + 2 * _dimension;

            math_Vector pnt1 = new(_tabPoints, adr1, 1, _dimension);
            math_Vector pnt2 = new(_tabPoints, adr2, 1, _dimension);

            vTang.Initialized(pnt2 - pnt1);
        }

        double vnorm = vTang.Norm();

        if (vnorm <= epsNorm)
        {
            vTang.Init(0.0);
        }
        else
        {
            vTang /= vnorm;
        }

        // Estimation with constraints

        double wcnt = 0.0;
        int idCnt = 1;

        // Warning!! Here it is suppoused that all points are in range [myFirstPoint, myLastPoint]

        int nbConstr = _nbPassPoints + _nbTangPoints + _nbCurvPoints;
        math_Vector vCnt = new(1, _dimension, 0.0);

        if (nbConstr > 0)
        {
            // OCCT tests the point before the count, so it reads the point of constraint NbConstr + 1 past the
            // constraints (occt-issues.md); the port tests the count first.
            while (idCnt <= nbConstr && _typConstraints.Value(2 * idCnt - 1) < ipnt)
            {
                idCnt++;
            }
            if (idCnt <= nbConstr && (_typConstraints.Value(2 * idCnt - 1) == ipnt)
                && (_typConstraints.Value(2 * idCnt) >= 1))
            {
                wcnt = 1.0;
                int i0 = 2 * _dimension * (idCnt - 1), k = 0;
                for (i = 1; i <= _nbP3d; i++)
                {
                    for (int j = 1; j <= 3; j++)
                    {
                        vCnt[++k] = _tabConstraints.Value(++i0);
                    }
                    i0 += 3;
                }
                for (i = 1; i <= _nbP2d; i++)
                {
                    for (int j = 1; j <= 2; j++)
                    {
                        vCnt[++k] = _tabConstraints.Value(++i0);
                    }
                    i0 += 2;
                }
            }
        }

        // Averaging of estimation

        double denom = wpnt + wcnt;
        if (denom == 0.0)
        {
            denom = 1.0;
        }
        else
        {
            denom = 1.0 / denom;
        }

        vTang.Initialized((wpnt * vTang + wcnt * vCnt) * denom);

        vnorm = vTang.Norm();

        if (vnorm <= epsNorm)
        {
            vTang.Init(0.0);
        }
        else
        {
            vTang /= vnorm;
        }
    }

    private void EstSecnd(int ipnt, math_Vector vTang1, math_Vector vTang2, double length, math_Vector vScnd)
    {
        int i;

        const double eps = 1.0e-9;

        double wpnt = 1.0;

        double aux;

        if (ipnt == _firstPoint)
        {
            aux = _parameters.Value(ipnt + 1) - _parameters.Value(ipnt);
        }
        else if (ipnt == _lastPoint)
        {
            aux = _parameters.Value(ipnt) - _parameters.Value(ipnt - 1);
        }
        else
        {
            aux = _parameters.Value(ipnt + 1) - _parameters.Value(ipnt - 1);
        }

        if (aux <= eps)
        {
            aux = 1.0;
        }
        else
        {
            aux = 1.0 / aux;
        }

        vScnd.Initialized((vTang2 - vTang1) * aux);

        // Estimation with constraints

        double wcnt = 0.0;
        int idCnt = 1;

        // Warning!! Here it is suppoused that all points are in range [myFirstPoint, myLastPoint]

        int nbConstr = _nbPassPoints + _nbTangPoints + _nbCurvPoints;
        math_Vector vCnt = new(1, _dimension, 0.0);

        if (nbConstr > 0)
        {
            // OCCT tests the point before the count (occt-issues.md); the port tests the count first.
            while (idCnt <= nbConstr && _typConstraints.Value(2 * idCnt - 1) < ipnt)
            {
                idCnt++;
            }

            if (idCnt <= nbConstr && (_typConstraints.Value(2 * idCnt - 1) == ipnt)
                && (_typConstraints.Value(2 * idCnt) >= 2))
            {
                wcnt = 1.0;
                int i0 = 2 * _dimension * (idCnt - 1) + 3, k = 0;
                for (i = 1; i <= _nbP3d; i++)
                {
                    for (int j = 1; j <= 3; j++)
                    {
                        vCnt[++k] = _tabConstraints.Value(++i0);
                    }
                    i0 += 3;
                }
                i0--;
                for (i = 1; i <= _nbP2d; i++)
                {
                    for (int j = 1; j <= 2; j++)
                    {
                        vCnt[++k] = _tabConstraints.Value(++i0);
                    }
                    i0 += 2;
                }
            }
        }

        // Averaging of estimation

        double denom = wpnt + wcnt;
        if (denom == 0.0)
        {
            denom = 1.0;
        }
        else
        {
            denom = 1.0 / denom;
        }

        vScnd.Initialized((wpnt * vScnd + (wcnt * length) * vCnt) * denom);
    }

    private void InitCutting(HermitJacobi aBase, double curvTol, out Curve aCurve)
    {
        // Definition of number of elements
        int orcMx = -1, nCont = 0, i, kk, nbElem;
        int nbConstr = _nbPassPoints + _nbTangPoints + _nbCurvPoints;

        for (i = 1; i <= nbConstr; i++)
        {
            kk = Math.Abs(_typConstraints.Value(2 * i)) + 1;
            orcMx = Max(orcMx, kk);
            nCont += kk;
        }

        if (orcMx > _maxDegree - _nivCont)
        {
            throw new ConstructionErrorException("AppDef_Variational::InitCutting");
        }

        int nLibre = Max(_maxDegree - _nivCont - (_maxDegree + 1) / 4, _nivCont + 1);

        nbElem = (nCont % nLibre == 0) ? nCont / nLibre : nCont / nLibre + 1;

        while ((nbElem > _maxSegment) && (nLibre < _maxDegree - _nivCont))
        {
            nLibre++;
            nbElem = (nCont % nLibre == 0) ? nCont / nLibre : nCont / nLibre + 1;
        }

        if (nbElem > _maxSegment)
        {
            throw new ConstructionErrorException("AppDef_Variational::InitCutting");
        }

        aCurve = new Curve(_dimension, nbElem, aBase, curvTol);

        int nCnt = (nCont - 1) / nbElem + 1;
        int nPlus = nbElem - (nCnt * nbElem - nCont);

        NCollection_Array1<double> knot = aCurve.Knots();

        int iDeb = 0, iFin = nbConstr + 1, nDeb = 0, nFin = 0, indEl = knot.Lower(), iUpper = knot.Upper(),
            nbEl = 0;

        knot[indEl] = _parameters.Value(_firstPoint);
        knot[iUpper] = _parameters.Value(_lastPoint);

        while (nbElem - nbEl > 1)
        {
            indEl++;
            nbEl++;
            if (nPlus == 0)
            {
                nCnt--;
            }

            while (nDeb < nCnt && iDeb < iFin)
            {
                iDeb++;
                nDeb += Math.Abs(_typConstraints.Value(2 * iDeb)) + 1;
            }

            if (nDeb == nCnt)
            {
                nDeb = 0;
                if (nPlus == 1 && _parameters.Value(_typConstraints.Value(2 * iDeb - 1)) > knot[indEl - 1])
                {
                    knot[indEl] = _parameters.Value(_typConstraints.Value(2 * iDeb - 1));
                }
                else
                {
                    knot[indEl] = (_parameters.Value(_typConstraints.Value(2 * iDeb - 1))
                                   + _parameters.Value(_typConstraints.Value(2 * iDeb + 1)))
                                  / 2;
                }
            }
            else
            {
                nDeb -= nCnt;
                knot[indEl] = _parameters.Value(_typConstraints.Value(2 * iDeb - 1));
            }

            nPlus--;
            if (nPlus == 0)
            {
                nCnt--;
            }

            if (nbElem - nbEl == 1)
            {
                break;
            }

            nbEl++;

            while (nFin < nCnt && iDeb < iFin)
            {
                iFin--;
                nFin += Math.Abs(_typConstraints.Value(2 * iFin)) + 1;
            }

            if (nFin == nCnt)
            {
                nFin = 0;
                knot[iUpper + 1 - indEl] = (_parameters.Value(_typConstraints.Value(2 * iFin - 1))
                                            + _parameters.Value(_typConstraints.Value(2 * iFin - 3)))
                                           / 2;
            }
            else
            {
                nFin -= nCnt;
                if (_parameters.Value(_typConstraints.Value(2 * iFin - 1)) < knot[iUpper - indEl + 1])
                {
                    knot[iUpper + 1 - indEl] = _parameters.Value(_typConstraints.Value(2 * iFin - 1));
                }
                else
                {
                    knot[iUpper + 1 - indEl] = (_parameters.Value(_typConstraints.Value(2 * iFin - 1))
                                                + _parameters.Value(_typConstraints.Value(2 * iFin - 3)))
                                               / 2;
                }
            }
        }
    }

    private void Adjusting(ref SmoothCriterion j, ref double wQuadratic, ref double wQuality,
                           ref Curve theCurve, NCollection_Array1<double> ecarts)
    {
        /* Initialized data */

        const int mxiter = 2;
        const double eps1 = 1e-6;
        int nbrPnt = _lastPoint - _firstPoint + 1;
        int nbrConstraint = _nbPassPoints + _nbTangPoints + _nbCurvPoints;
        double curvTol = eps1 * j.EstLength() / nbrPnt;

        /* Local variables */
        int iter, ipnt;
        double ecart, emold, erold, tpara;
        double j1cibl, vtest, vseuil;
        double[] vocri = new double[4];
        int i, numint, flag;
        NCollection_Array1<double> tbpoid = new(_firstPoint, _lastPoint);
        bool loptim, lrejet;
        SmoothCriterion jNew;
        Curve cNew;
        double e1, e2, e3;

        /* (0.b) Initialisations */

        loptim = true;
        iter = 0;
        tbpoid.Init(1.0);

        /* ============   loop on the smoothing engine  ============== */

        vtest = wQuality * .9;
        j1cibl = Math.Sqrt(_criterium[0] / (nbrPnt - nbrConstraint));

        while (loptim)
        {
            ++iter;

            /*     (1) Save the previous state */

            vocri[0] = _criterium[0];
            vocri[1] = _criterium[1];
            vocri[2] = _criterium[2];
            vocri[3] = _criterium[3];
            erold = _maxError;
            emold = _averageError;

            /*     (2) Increase the least squares weight */

            if (j1cibl > vtest)
            {
                wQuadratic = j1cibl / vtest * wQuadratic;
            }

            /*     (3) Increase the weight associated with problematic points */

            vseuil = wQuality * .88;

            for (ipnt = _firstPoint; ipnt <= _lastPoint; ++ipnt)
            {
                if (ecarts[ipnt] > vtest)
                {
                    ecart = (ecarts[ipnt] - vseuil) / wQuality;
                    tbpoid[ipnt] = (ecart * 3 + 1.0) * tbpoid[ipnt];
                }
            }

            /*     (4) Forced splitting */

            if (theCurve.ElementCount < _maxSegment && _withCutting)
            {
                numint = NearIndex(_parameters.Value(_maxErrorIndex), theCurve.Knots(), 0, out flag);

                tpara = (theCurve.Knots()[numint] + theCurve.Knots()[numint + 1]
                         + _parameters.Value(_maxErrorIndex) * 2)
                        / 4;

                cNew = new Curve(_dimension, theCurve.ElementCount + 1, theCurve.Base(), curvTol);

                for (i = 1; i <= numint; i++)
                {
                    cNew.Knots()[i] = theCurve.Knots()[i];
                }
                for (i = numint + 1; i <= theCurve.Knots().Length(); i++)
                {
                    cNew.Knots()[i + 1] = theCurve.Knots()[i];
                }

                cNew.Knots()[numint + 1] = tpara;
            }
            else
            {
                cNew = new Curve(_dimension, theCurve.ElementCount, theCurve.Base(), curvTol);

                cNew.Knots().Assign(theCurve.Knots());
            }

            jNew = new LinearCriteria(_ssp, _firstPoint, _lastPoint);

            jNew.EstLength() = j.EstLength();

            j.GetEstimation(out e1, out e2, out e3);

            jNew.SetEstimation(e1, e2, e3);

            jNew.SetParameters(_parameters);

            jNew.SetWeight(wQuadratic, wQuality, _percent[0], _percent[1], _percent[2]);

            jNew.SetWeight(tbpoid);

            jNew.SetCurve(cNew);

            /*     (5) Relissage */

            TheMotor(jNew, wQuadratic, wQuality, ref cNew, ecarts);

            /*     (6) Tests de rejet */

            j1cibl = Math.Sqrt(_criterium[0] / (nbrPnt - nbrConstraint));
            vseuil = Math.Sqrt(vocri[1]) + (erold - _maxError) * 4;

            lrejet = ((_maxError > wQuality) && (_maxError > erold * 1.01))
                     || (Math.Sqrt(_criterium[1]) > vseuil * 1.05);

            if (lrejet)
            {
                _criterium[0] = vocri[0];
                _criterium[1] = vocri[1];
                _criterium[2] = vocri[2];
                _criterium[3] = vocri[3];
                _maxError = erold;
                _averageError = emold;

                loptim = false;
            }
            else
            {
                j = jNew;
                theCurve = cNew;
                j.SetCurve(theCurve);
            }

            /*     (7) Test de convergence */

            if (((iter >= mxiter) && (_maxSegment == cNew.ElementCount)) || _maxError < wQuality)
            {
                loptim = false;
            }
        }
    }

    private static bool NotParallel(in Vector3D t, out Vector3D v)
    {
        v = t;
        v = v.WithX(v.X + 1.0);
        if (v.CrossMagnitude(t) > 1.0e-12)
        {
            return true;
        }
        v = v.WithY(v.Y + 1.0);
        if (v.CrossMagnitude(t) > 1.0e-12)
        {
            return true;
        }
        v = v.WithZ(v.Z + 1.0);
        return v.CrossMagnitude(t) > 1.0e-12;
    }

    private void AssemblingConstraints(Curve curve, NCollection_Array1<double> parameters, double cblong,
                                       Assembly a)
    {
        int mxDeg = curve.Base().WorkDegree(), nbElm = curve.ElementCount, nbDim = curve.Dimension();

        NCollection_Array1<double> g0 = new(0, mxDeg), g1 = new(0, mxDeg), g2 = new(0, mxDeg);
        math_Vector v0 = new(g0, 0, 0, mxDeg), v1 = new(g1, 0, 0, mxDeg), v2 = new(g2, 0, 0, mxDeg);

        int indexOfConstraint, ng3d, ng2d, nBeg2d, nPass, ngPC1, nTang3d, nTang2d, point, typOfConstr,
            p0 = parameters.Lower() - _firstPoint, curel = 1, el, i, ipnt, ityp, j, k, pnt, curdim, jt,
            ntheta = 6 * _nbP3d + 2 * _nbP2d;
        int nbConstr = _nbPassPoints + _nbTangPoints + _nbCurvPoints;

        //  Ng3d = 3 * NbConstr + 2 * myNbTangPoints + 5 * myNbCurvPoints;
        //  Ng2d = 2 * NbConstr + 1 * myNbTangPoints + 3 * myNbCurvPoints;
        ng3d = 3 * nbConstr + 3 * _nbTangPoints + 5 * _nbCurvPoints;
        ng2d = 2 * nbConstr + 2 * _nbTangPoints + 3 * _nbCurvPoints;
        nBeg2d = ng3d * _nbP3d;
        //  NgPC1 = NbConstr + myNbCurvPoints;
        ngPC1 = nbConstr + _nbTangPoints + _nbCurvPoints;
        nPass = 0;
        nTang3d = 3 * ngPC1;
        nTang2d = 2 * ngPC1;

        NCollection_Array1<double> intervals = curve.Knots();

        double t, r1, r2;

        HermitJacobi myHermitJacobi = curve.Base();
        int order = myHermitJacobi.NivConstr() + 1;

        double uFirst, uLast, coeff, c0, mfact, mfact1;

        a.NullifyConstraint();

        ipnt = -1;
        ityp = 0;
        for (i = 1; i <= nbConstr; i++)
        {
            ipnt += 2;
            ityp += 2;

            point = _typConstraints.Value(ipnt);
            typOfConstr = _typConstraints.Value(ityp);

            t = parameters[p0 + point];

            for (el = curel; el <= nbElm;)
            {
                if (t <= intervals[++el])
                {
                    curel = el - 1;
                    break;
                }
            }

            uFirst = intervals[curel];
            uLast = intervals[curel + 1];
            coeff = (uLast - uFirst) / 2.0;
            c0 = (uLast + uFirst) / 2.0;

            t = (t - c0) / coeff;

            if (typOfConstr == 0)
            {
                myHermitJacobi.D0(t, g0);
                for (k = 1; k < order; k++)
                {
                    mfact = Math.Pow(coeff, k);
                    g0[k] *= mfact;
                    g0[k + order] *= mfact;
                }
            }
            else if (typOfConstr == 1)
            {
                myHermitJacobi.D1(t, g0, g1);
                for (k = 1; k < order; k++)
                {
                    mfact = Math.Pow(coeff, k);
                    g0[k] *= mfact;
                    g0[k + order] *= mfact;
                    g1[k] *= mfact;
                    g1[k + order] *= mfact;
                }
                mfact = 1.0 / coeff;
                for (k = 0; k <= mxDeg; k++)
                {
                    g1[k] *= mfact;
                }
            }
            else
            {
                myHermitJacobi.D2(t, g0, g1, g2);
                for (k = 1; k < order; k++)
                {
                    mfact = Math.Pow(coeff, k);
                    g0[k] *= mfact;
                    g0[k + order] *= mfact;
                    g1[k] *= mfact;
                    g1[k + order] *= mfact;
                    g2[k] *= mfact;
                    g2[k + order] *= mfact;
                }
                mfact = 1.0 / coeff;
                mfact1 = mfact / coeff;
                for (k = 0; k <= mxDeg; k++)
                {
                    g1[k] *= mfact;
                    g2[k] *= mfact1;
                }
            }

            nPass++;

            j = nbDim * (point - _firstPoint);
            int n0 = nPass;
            curdim = 0;
            for (pnt = 1; pnt <= _nbP3d; pnt++)
            {
                indexOfConstraint = n0;
                for (k = 1; k <= 3; k++)
                {
                    curdim++;
                    a.AddConstraint(indexOfConstraint, curel, curdim, v0, _tabPoints.Value(j + k));
                    indexOfConstraint += ngPC1;
                }
                j += 3;
                n0 += ng3d;
            }

            n0 = nPass + nBeg2d;
            for (pnt = 1; pnt <= _nbP2d; pnt++)
            {
                indexOfConstraint = n0;
                for (k = 1; k <= 2; k++)
                {
                    curdim++;
                    a.AddConstraint(indexOfConstraint, curel, curdim, v0, _tabPoints.Value(j + k));
                    indexOfConstraint += ngPC1;
                }
                j += 2;
                n0 += ng2d;
            }

            if (typOfConstr == 1)
            {
                nPass++;
                n0 = nPass;
                j = 2 * nbDim * (i - 1);
                curdim = 0;
                for (pnt = 1; pnt <= _nbP3d; pnt++)
                {
                    indexOfConstraint = n0;
                    for (k = 1; k <= 3; k++)
                    {
                        curdim++;
                        a.AddConstraint(indexOfConstraint, curel, curdim, v1, cblong * _tabConstraints.Value(j + k));
                        indexOfConstraint += ngPC1;
                    }
                    n0 += ng3d;
                    j += 6;
                }

                n0 = nPass + nBeg2d;
                for (pnt = 1; pnt <= _nbP2d; pnt++)
                {
                    indexOfConstraint = n0;
                    for (k = 1; k <= 2; k++)
                    {
                        curdim++;
                        a.AddConstraint(indexOfConstraint, curel, curdim, v1, cblong * _tabConstraints.Value(j + k));
                        indexOfConstraint += ngPC1;
                    }
                    n0 += ng2d;
                    j += 4;
                }
            }
            if (typOfConstr == 2)
            {
                nPass++;
                n0 = nPass;
                j = 2 * nbDim * (i - 1);
                curdim = 0;
                for (pnt = 1; pnt <= _nbP3d; pnt++)
                {
                    indexOfConstraint = n0;
                    for (k = 1; k <= 3; k++)
                    {
                        curdim++;
                        a.AddConstraint(indexOfConstraint, curel, curdim, v1, cblong * _tabConstraints.Value(j + k));
                        indexOfConstraint += ngPC1;
                    }
                    n0 += ng3d;
                    j += 6;
                }

                n0 = nPass + nBeg2d;
                for (pnt = 1; pnt <= _nbP2d; pnt++)
                {
                    indexOfConstraint = n0;
                    for (k = 1; k <= 2; k++)
                    {
                        curdim++;
                        a.AddConstraint(indexOfConstraint, curel, curdim, v1, cblong * _tabConstraints.Value(j + k));
                        indexOfConstraint += ngPC1;
                    }
                    n0 += ng2d;
                    j += 4;
                }

                j = 2 * nbDim * (i - 1) + 3;
                jt = ntheta * (i - 1);
                indexOfConstraint = nTang3d + 1;
                curdim = 0;
                for (pnt = 1; pnt <= _nbP3d; pnt++)
                {
                    r1 = 0.0;
                    r2 = 0.0;
                    for (k = 1; k <= 3; k++)
                    {
                        r1 += _tabConstraints.Value(j + k) * _ttheta.Value(jt + k);
                        r2 += _tabConstraints.Value(j + k) * _ttheta.Value(jt + 3 + k);
                    }
                    r1 *= cblong * cblong;
                    r2 *= cblong * cblong;
                    for (k = 1; k <= 3; k++)
                    {
                        curdim++;
                        if (k > 1)
                        {
                            r1 = r2 = 0.0;
                        }
                        a.AddConstraint(indexOfConstraint, curel, curdim, _tfthet.Value(jt + k) * v2, r1);
                        a.AddConstraint(indexOfConstraint + 1, curel, curdim, _tfthet.Value(jt + 3 + k) * v2, r2);
                    }
                    indexOfConstraint += ng3d;
                    j += 6;
                    jt += 6;
                }

                j--;
                indexOfConstraint = nBeg2d + nTang2d + 1;
                for (pnt = 1; pnt <= _nbP2d; pnt++)
                {
                    r1 = 0.0;
                    for (k = 1; k <= 2; k++)
                    {
                        r1 += _tabConstraints.Value(j + k) * _ttheta.Value(jt + k);
                    }
                    r1 *= cblong * cblong;
                    for (k = 1; k <= 2; k++)
                    {
                        curdim++;
                        if (k > 1)
                        {
                            r1 = 0.0;
                        }
                        a.AddConstraint(indexOfConstraint, curel, curdim, _tfthet.Value(jt + k) * v2, r1);
                    }
                    indexOfConstraint += ng2d;
                    j += 4;
                    jt += 2;
                }

                nTang3d += 2;
                nTang2d += 1;
            }
        }
    }

    private bool InitTthetaF(int ndimen, Constraint typcon, int begin, int jndex)
    {
        if ((ndimen < 2) || (ndimen > 3))
        {
            return false;
        }
        Vector3D t = new(), v = new();
        Vector3D theta1, theta2 = new();
        Vector3D f = new();
        double xx, xy, yy, xz, yz, zz;

        if ((typcon == TangencyPoint) || (typcon == CurvaturePoint))
        {
            t = t.WithX(_tabConstraints.Value(jndex));
            t = t.WithY(_tabConstraints.Value(jndex + 1));
            if (ndimen == 3)
            {
                t = t.WithZ(_tabConstraints.Value(jndex + 2));
            }
            else
            {
                t = t.WithZ(0.0);
            }
            if (ndimen == 2)
            {
                v = v.WithX(0.0);
                v = v.WithY(0.0);
                v = v.WithZ(1.0);
            }
            if (ndimen == 3)
            {
                if (!NotParallel(t, out v))
                {
                    return false;
                }
            }
            theta1 = v ^ t;
            theta1 = theta1.Normalized();
            _ttheta.SetValue(begin, theta1.X);
            _ttheta.SetValue(begin + 1, theta1.Y);
            if (ndimen == 3)
            {
                theta2 = t ^ theta1;
                theta2 = theta2.Normalized();
                _ttheta.SetValue(begin + 2, theta1.Z);
                _ttheta.SetValue(begin + 3, theta2.X);
                _ttheta.SetValue(begin + 4, theta2.Y);
                _ttheta.SetValue(begin + 5, theta2.Z);
            }

            // Calculation of myTfthet
            if (typcon == CurvaturePoint)
            {
                xx = Math.Pow(t.X, 2);
                xy = t.X * t.Y;
                yy = Math.Pow(t.Y, 2);
                if (ndimen == 2)
                {
                    f = f.WithX(yy * theta1.X - xy * theta1.Y);
                    f = f.WithY(xx * theta1.Y - xy * theta1.X);
                    _tfthet.SetValue(begin, f.X);
                    _tfthet.SetValue(begin + 1, f.Y);
                }
                if (ndimen == 3)
                {
                    xz = t.X * t.Z;
                    yz = t.Y * t.Z;
                    zz = Math.Pow(t.Z, 2);

                    f = f.WithX((zz + yy) * theta1.X - xy * theta1.Y - xz * theta1.Z);
                    f = f.WithY((xx + zz) * theta1.Y - xy * theta1.X - yz * theta1.Z);
                    f = f.WithZ((xx + yy) * theta1.Z - xz * theta1.X - yz * theta1.Y);
                    _tfthet.SetValue(begin, f.X);
                    _tfthet.SetValue(begin + 1, f.Y);
                    _tfthet.SetValue(begin + 2, f.Z);
                    f = f.WithX((zz + yy) * theta2.X - xy * theta2.Y - xz * theta2.Z);
                    f = f.WithY((xx + zz) * theta2.Y - xy * theta2.X - yz * theta2.Z);
                    f = f.WithZ((xx + yy) * theta2.Z - xz * theta2.X - yz * theta2.Y);
                    _tfthet.SetValue(begin + 3, f.X);
                    _tfthet.SetValue(begin + 4, f.Y);
                    _tfthet.SetValue(begin + 5, f.Z);
                }
            }
        }
        return true;
    }

    // std::stable_sort with operator<, for the parameter and knot arrays (insertion sort: they are nearly sorted).
    private static void StableSort(NCollection_Array1<double> array)
    {
        Span<double> a = array.AsSpan();
        for (int i = 1; i < a.Length; i++)
        {
            double v = a[i];
            int j = i - 1;
            while (j >= 0 && v < a[j])
            {
                a[j + 1] = a[j];
                j--;
            }
            a[j + 1] = v;
        }
    }
}
