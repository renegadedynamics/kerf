// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Batten.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Batten.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_Batten.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using System.Globalization;

namespace Kerf.Engine.Construction.Fairing;

/// <summary>
/// Constructs 2D curves that simulate physical splines or battens (wooden or plastic beams of constant or linearly
/// increasing section), from two points and constraints of tangency at each.
/// </summary>
/// <remarks>
/// A working object (decision 859), as OCCT's: the setters record the wanted ("new") constraints, <see
/// cref="Compute(out AnalysisCode, int, double)"/> deforms the current curve toward them by homotopy and
/// records the reached ("old") ones, which the getters return. <see cref="Curve"/> returns a new, frozen curve each
/// time. OCCT's <c>operator&lt;&lt;</c> is <see cref="ToString"/>.
/// </remarks>
internal class Batten
{
    protected AnalysisCode _code;
    protected Point2D _oldP1;
    protected Point2D _oldP2;
    protected double _oldAngle1;
    protected double _oldAngle2;
    protected double _oldHeight;
    protected double _oldSlope;
    protected double _oldSlidingFactor;
    protected bool _oldFreeSliding;
    protected int _oldConstraintOrder1;
    protected int _oldConstraintOrder2;
    protected Point2D _newP1;
    protected Point2D _newP2;
    protected double _newAngle1;
    protected double _newAngle2;
    protected double _newHeight;
    protected double _newSlope;
    protected double _newSlidingFactor;
    protected bool _newFreeSliding;
    protected int _newConstraintOrder1;
    protected int _newConstraintOrder2;
    protected int _degree;
    protected NCollection_HArray1<Point2D> _poles;
    protected NCollection_HArray1<double> _flatknots;
    protected NCollection_HArray1<double> _knots;
    protected NCollection_HArray1<int> _mults;

    /// <summary>
    /// A batten through <paramref name="p1"/> and <paramref name="p2"/> of height <paramref name="height"/> and slope
    /// <paramref name="slope"/>. The other parameters start as: FreeSliding false, ConstraintOrder1 and 2 equal to 1,
    /// Angle1 and Angle2 equal to 0, SlidingFactor 1.
    /// </summary>
    /// <exception cref="NullValueException">P1 and P2 are confused (within Precision::Confusion).</exception>
    /// <exception cref="NegativeValueException"><paramref name="height"/> is not positive.</exception>
    public Batten(in Point2D p1, in Point2D p2, double height, double slope = 0)
    {
        _code = AnalysisCode.OK;
        _oldP1 = p1;
        _oldP2 = p2;
        _oldAngle1 = 0;
        _oldAngle2 = 0;
        _oldHeight = height;
        _oldSlope = slope;
        _oldSlidingFactor = 1;
        _oldFreeSliding = false;
        _oldConstraintOrder1 = 1;
        _oldConstraintOrder2 = 1;
        _newP1 = p1;
        _newP2 = p2;
        _newAngle1 = 0;
        _newAngle2 = 0;
        _newHeight = height;
        _newSlope = slope;
        _newSlidingFactor = 1;
        _newFreeSliding = false;
        _newConstraintOrder1 = 1;
        _newConstraintOrder2 = 1;
        _degree = 9;

        if (p1.IsEqual(p2, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            throw new NullValueException("FairCurve : P1 and P2 are confused");
        }
        if (height <= 0)
        {
            throw new NegativeValueException("FairCurve : Height is not positive");
        }
        //
        //   Initialize by a straight line (2 poles)
        //
        NCollection_HArray1<double> iknots = new(1, 2);
        NCollection_HArray1<int> imults = new(1, 2);
        NCollection_HArray1<Point2D> ipoles = new(1, 2);

        iknots.SetValue(1, 0);
        iknots.SetValue(2, 1);

        imults.SetValue(1, 2);
        imults.SetValue(2, 2);

        ipoles.SetValue(1, p1);
        ipoles.SetValue(2, p2);

        //  Increase the degree

        NCollection_HArray1<Point2D> npoles = new(1, _degree + 1);
        NCollection_HArray1<double> nweight = new(1, 2);
        NCollection_HArray1<double> nknots = new(1, 2);
        NCollection_HArray1<int> nmults = new(1, 2);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(1,
                                _degree,
                                false,
                                ipoles.Array1(),
                                global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                iknots.Array1(),
                                imults.Array1(),
                                npoles.ChangeArray1(),
                                nweight.ChangeArray1(),
                                nknots.ChangeArray1(),
                                nmults.ChangeArray1());

        // and impact the result in our fields

        _poles = npoles;
        _knots = nknots;
        _mults = nmults;

        // calculate "plane" nodes

        _flatknots =
          new NCollection_HArray1<double>(1,
                                          global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequenceLength(_mults.Array1(), _degree, false));

        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(_knots.Array1(),
                              _mults.Array1(),
                              _degree,
                              false,
                              _flatknots.ChangeArray1());
    }

    /// <summary>
    /// Sets free sliding (false by default). When sliding is free, the sliding factor is computed to satisfy the
    /// equilibrium of the batten.
    /// </summary>
    public void SetFreeSliding(bool freeSliding)
    {
        _newFreeSliding = freeSliding;
    }

    /// <summary>
    /// Sets the order of the constraint at the first point (1 by default): 0, the curve passes through the point; 1,
    /// with a given tangent; 2, with a given tangent and curvature (FairCurve_MinimalVariation only).
    /// </summary>
    public void SetConstraintOrder1(int constraintOrder)
    {
        _newConstraintOrder1 = constraintOrder;
    }

    /// <summary>Sets the order of the constraint at the second point (1 by default), as <see cref="SetConstraintOrder1"/>.</summary>
    public void SetConstraintOrder2(int constraintOrder)
    {
        _newConstraintOrder2 = constraintOrder;
    }

    /// <summary>Moves the first point (which changes the angles as well).</summary>
    /// <exception cref="NullValueException">P1 and P2 are confused (within Precision::Confusion).</exception>
    public void SetP1(in Point2D p1)
    {
        if (p1.IsEqual(_newP2, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            throw new NullValueException("FairCurve : P1 and P2 are confused");
        }
        Angles(p1, _newP2);
        _newP1 = p1;
    }

    /// <summary>Moves the second point (which changes the angles as well).</summary>
    /// <exception cref="NullValueException">P1 and P2 are confused (within Precision::Confusion).</exception>
    public void SetP2(in Point2D p2)
    {
        if (_newP1.IsEqual(p2, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            throw new NullValueException("FairCurve : P1 and P2 are confused");
        }
        Angles(_newP1, p2);
        _newP2 = p2;
    }

    /// <summary>Sets the angle at the first point (0 by default).</summary>
    public void SetAngle1(double angle1)
    {
        _newAngle1 = angle1;
    }

    /// <summary>Sets the angle at the second point (0 by default).</summary>
    public void SetAngle2(double angle2)
    {
        _newAngle2 = angle2;
    }

    /// <summary>Sets the height of the deformation.</summary>
    /// <exception cref="NegativeValueException"><paramref name="height"/> is not positive.</exception>
    public void SetHeight(double height)
    {
        if (height <= 0)
        {
            throw new NegativeValueException("FairCurve : Height is no positive");
        }
        _newHeight = height;
    }

    /// <summary>Sets the slope.</summary>
    public void SetSlope(double slope)
    {
        _newSlope = slope;
    }

    /// <summary>
    /// Sets the ratio of the batten's length to the reference length (1 by default): a larger value inflates the
    /// batten, a smaller one flattens it. Used when sliding is imposed.
    /// </summary>
    public void SetSlidingFactor(double slidingFactor)
    {
        _newSlidingFactor = slidingFactor;
    }

    /// <summary>
    /// Computes the curve with respect to the constraints. <paramref name="code"/> is OK, NotConverged,
    /// InfiniteSliding or NullHeight; <paramref name="nbIterations"/> and <paramref name="tolerance"/> control the
    /// precision and the time of the computation.
    /// </summary>
    public virtual bool Compute(out AnalysisCode code, int nbIterations = 50, double tolerance = 1.0e-3)
    {
        bool ok = true, end = false;
        double angleMax = 0.7;            // parameter ruling the function of increment ( 40 degrees )
        double angleMin = 2 * M_PI / 100; // parameter ruling the function of increment
                                          // full passage should not cost more than 100 steps.
        double dAngle1, dAngle2, ratio, fraction, toler;
        double oldDist, newDist;
        // the loop runs at least once, and its first call writes the code (decision 860)
        code = AnalysisCode.OK;

        //  Loop of Homotopy : calculation of the step and optimisation

        while (ok && !end)
        {
            dAngle1 = _newAngle1 - _oldAngle1;
            dAngle2 = _newAngle2 - _oldAngle2;
            ratio = 1;
            if (_newConstraintOrder1 > 0)
            {
                fraction =
                  Math.Abs(dAngle1) / (angleMax * Math.Exp(-Math.Abs(_oldAngle1) / angleMax) + angleMin);
                if (fraction > 1)
                {
                    ratio = 1 / fraction;
                }
            }
            if (_newConstraintOrder2 > 0)
            {
                fraction =
                  Math.Abs(dAngle2) / (angleMax * Math.Exp(-Math.Abs(_oldAngle2) / angleMax) + angleMin);
                if (fraction > 1)
                {
                    ratio = (ratio < 1 / fraction ? ratio : 1 / fraction);
                }
            }

            oldDist = _oldP1.DistanceTo(_oldP2);
            newDist = _newP1.DistanceTo(_newP2);
            fraction = Math.Abs(oldDist - newDist) / (oldDist / 3);
            if (fraction > 1)
            {
                ratio = (ratio < 1 / fraction ? ratio : 1 / fraction);
            }

            Vector2D deltaP1 = new(_oldP1, _newP1), deltaP2 = new(_oldP2, _newP2);
            if (ratio == 1)
            {
                end = true;
                toler = tolerance;
            }
            else
            {
                deltaP1 *= ratio;
                deltaP2 *= ratio;
                dAngle1 *= ratio;
                dAngle2 *= ratio;
                toler = 10 * tolerance;
            }

            ok = Compute(deltaP1, deltaP2, dAngle1, dAngle2, out code, nbIterations, toler);

            if (code != AnalysisCode.OK)
            {
                end = true;
            }
            if (_newFreeSliding)
            {
                _newSlidingFactor = _oldSlidingFactor;
            }
            if (_newConstraintOrder1 == 0)
            {
                _newAngle1 = _oldAngle1;
            }
            if (_newConstraintOrder2 == 0)
            {
                _newAngle2 = _oldAngle2;
            }
        }
        _code = code;
        return ok;
    }

    /// <summary>Computes the curve with respect to the delta-constraints.</summary>
    private bool Compute(in Vector2D deltaP1,
                         in Vector2D deltaP2,
                         double deltaAngle1,
                         double deltaAngle2,
                         out AnalysisCode aCode,
                         int nbIterations,
                         double tolerance)
    {
        bool ok, okCompute = true;
        aCode = AnalysisCode.OK;

        // Deformation of the curve by adding a polynom of interpolation
        int l = 2 + _newConstraintOrder1 + _newConstraintOrder2, kk, ii;
        NCollection_Array1<double> knots = new(1, 2);
        knots[1] = 0;
        knots[2] = 1;
        NCollection_Array1<int> mults = new(1, 2);
        NCollection_Array1<Point2D> hermitePoles = new(1, l);
        NCollection_Array1<Point2D> interpolation = new(1, l);
        NCollection_HArray1<Point2D> nPoles = new(1, _poles.Length());

        // Polynoms of Hermite
        Matrix hermiteCoef = new(1, l, 1, l);
        ok = global::Kerf.Engine.Numerics.Polynomials.PLib.HermiteCoefficients(0, 1, _newConstraintOrder1, _newConstraintOrder2, hermiteCoef);
        if (!ok)
        {
            return false;
        }

        // Definition of constraints of interpolation
        NCollection_Array1<Xy> aDelta = new(1, l);
        Vector2D vOld = new(_oldP1, _oldP2), vNew = new(-(_oldP1.XY() + deltaP1.XY()) + (_oldP2.XY() + deltaP2.XY()));
        double dAngleRef = vNew.AngleTo(vOld);

        aDelta[1] = deltaP1.XY();
        kk = 2;
        if (_newConstraintOrder1 > 0)
        {
            Vector2D oldDerive = new(_poles.Value(_poles.Lower()), _poles.Value(_poles.Lower() + 1));
            double knotGap1 = _knots.Value(2) - _knots.Value(1);
            if (Math.Abs(knotGap1) > global::Kerf.Engine.Foundation.Precision.Computational())
            {
                oldDerive *= _degree / knotGap1;
            }
            aDelta[kk] = (oldDerive.Rotated(deltaAngle1 - dAngleRef) - oldDerive).XY();
            kk += 1;
        }
        aDelta[kk] = deltaP2.XY();
        kk += 1;
        if (_newConstraintOrder2 > 0)
        {
            Vector2D oldDerive = new(_poles.Value(_poles.Upper() - 1), _poles.Value(_poles.Upper()));
            double knotGap2 = _knots.Value(_knots.Upper()) - _knots.Value(_knots.Upper() - 1);
            if (Math.Abs(knotGap2) > global::Kerf.Engine.Foundation.Precision.Computational())
            {
                oldDerive *= _degree / knotGap2;
            }
            aDelta[kk] = (oldDerive.Rotated(dAngleRef - deltaAngle2) - oldDerive).XY();
        }

        // Interpolation
        Xy auxXY = new(0, 0);
        for (ii = 1; ii <= l; ii++)
        {
            auxXY.SetCoord(0.0, 0);
            for (kk = 1; kk <= l; kk++)
            {
                auxXY += hermiteCoef[kk, ii] * aDelta[kk];
            }
            interpolation[ii] = interpolation[ii].WithXY(auxXY);
        }
        // Conversion into BSpline of the same structure as the current batten.
        global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(interpolation, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), hermitePoles, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());

        mults.Init(l);

        BSplineCurve2D deltaCurve = new(hermitePoles, knots, mults, l - 1);

        deltaCurve.IncreaseDegree(_degree);
        if (_mults.Length() > 2)
        {
            deltaCurve.InsertKnots(_knots.Array1(), _mults.Array1(), 1.0e-10);
        }

        // Summing
        nPoles.ChangeArray1().Assign(deltaCurve.InternalPoles());
        for (kk = nPoles.Lower(); kk <= nPoles.Upper(); kk++)
        {
            nPoles.ChangeValue(kk) = new Point2D(nPoles.ChangeValue(kk).XY() + _poles.Value(kk).Coord());
        }

        // Intermediary data

        double angle1, angle2, slidingLength,
          alph1 = _oldAngle1 + deltaAngle1, alph2 = _oldAngle2 + deltaAngle2,
          dist = nPoles.Value(nPoles.Upper()).DistanceTo(nPoles.Value(nPoles.Lower())),
          lReference = SlidingOfReference(dist, alph1, alph2);
        Vector2D ox = new(1, 0),
          p1P2 = new(nPoles.Value(nPoles.Upper()).Coord() - nPoles.Value(nPoles.Lower()).Coord());

        // Angles corresponding to axis ox

        angle1 = ox.AngleTo(p1P2) + alph1;
        angle2 = -ox.AngleTo(p1P2) + alph2;

        // Calculation of the length of sliding (imposed or initial);

        if (!_newFreeSliding)
        {
            slidingLength = _newSlidingFactor * lReference;
        }
        else
        {
            if (_oldFreeSliding)
            {
                slidingLength = _oldSlidingFactor * lReference;
            }
            else
            {
                slidingLength = SlidingOfReference(dist, alph1, alph2);
            }
        }

        // Energy and vectors of initialisation
        BattenLaw lBatten = new(_newHeight, _newSlope, slidingLength);
        EnergyOfBatten eBatten = new(_degree + 1,
                                               _flatknots,
                                               nPoles,
                                               _newConstraintOrder1,
                                               _newConstraintOrder2,
                                               lBatten,
                                               slidingLength,
                                               _newFreeSliding,
                                               angle1,
                                               angle2);
        math_Vector vInit = new(1, eBatten.VariableCount);

        // The valeur below is the smallest value of the criterion of flexion.
        double vConvex = 0.01 * Math.Pow(_newHeight / slidingLength, 3);
        if (vConvex < 1.0e-12)
        {
            vConvex = 1.0e-12;
        }

        ok = eBatten.Variable(vInit);

        // Minimisation
        Newton newton = new(eBatten,
                                      tolerance * p1P2.Magnitude / 10,
                                      tolerance,
                                      nbIterations,
                                      vConvex);
        newton.Perform(eBatten, vInit);
        ok = newton.IsDone;

        if (ok)
        {
            _poles = nPoles;
            newton.Location(vInit);
            if (_newFreeSliding)
            {
                _oldSlidingFactor = vInit[vInit.Upper()] / lReference;
            }
            else
            {
                _oldSlidingFactor = _newSlidingFactor;
            }

            if (_newConstraintOrder1 == 0)
            {
                Vector2D tangente = new(_poles.Value(_poles.Lower() + 1).Coord()
                                        - _poles.Value(_poles.Lower()).Coord());
                _oldAngle1 = p1P2.AngleTo(tangente);
            }
            else
            {
                _oldAngle1 = alph1;
            }

            if (_newConstraintOrder2 == 0)
            {
                Vector2D tangente = new(_poles.Value(_poles.Upper()).Coord()
                                        - _poles.Value(_poles.Upper() - 1).Coord());
                _oldAngle2 = (-tangente).AngleTo(-p1P2);
            }
            else
            {
                _oldAngle2 = alph2;
            }

            _oldP1 = _poles.Value(_poles.Lower());
            _oldP2 = _poles.Value(_poles.Upper());
            _oldConstraintOrder1 = _newConstraintOrder1;
            _oldConstraintOrder2 = _newConstraintOrder2;
            _oldFreeSliding = _newFreeSliding;
            _oldSlope = _newSlope;
            _oldHeight = _newHeight;
        }
        else
        {
            double v;
            aCode = eBatten.Status();
            if (!lBatten.Value(0, out v) || !lBatten.Value(1, out v))
            {
                aCode = AnalysisCode.NullHeight;
            }
            else
            {
                okCompute = false;
            }
            return okCompute;
        }

        ok = eBatten.Variable(vInit);

        // Processing of non-convergence
        if (!newton.IsConverged)
        {
            aCode = AnalysisCode.NotConverged;
        }

        // Prevention of infinite sliding
        if (_newFreeSliding && vInit[vInit.Upper()] > 2 * lReference)
        {
            aCode = AnalysisCode.InfiniteSliding;
        }

        // Eventual insertion of Nodes
        bool newKnots = false;
        int nbKnots = _knots.Length();
        double valAngles =
          (Math.Abs(_oldAngle1) + Math.Abs(_oldAngle2) + 2 * Math.Abs(_oldAngle2 - _oldAngle1));
        // Kerf: an infinite angle never fits, and the count overflows on the way (decision 22501)
        for (int aStep = 0; valAngles > (2 * (nbKnots - 2) + 1) * (1 + 2 * nbKnots); StepLimit.Next(ref aStep))
        {
            newKnots = true;
            nbKnots += nbKnots - 1;
        }

        if (newKnots)
        {
            BSplineCurve2D newBS = new(nPoles.Array1(), _knots.Array1(), _mults.Array1(), _degree);

            NCollection_HArray1<int> nMults = new(1, nbKnots);
            nMults.Init(_degree - 3);

            NCollection_HArray1<double> nKnots = new(1, nbKnots);
            for (ii = 1; ii <= nbKnots; ii++)
            {
                nKnots.ChangeValue(ii) = (double)(ii - 1) / (nbKnots - 1);
            }

            newBS.InsertKnots(nKnots.Array1(), nMults.Array1(), 1.0e-10);
            NCollection_HArray1<Point2D> newNPoles = new(newBS.InternalPoles());
            nMults = new NCollection_HArray1<int>(newBS.InternalMultiplicities());
            nKnots = new NCollection_HArray1<double>(newBS.InternalKnots());
            NCollection_HArray1<double> fKnots = new(newBS.InternalKnotSequence());

            _poles = newNPoles;
            _mults = nMults;
            _knots = nKnots;
            _flatknots = fKnots;
        }

        // For eventual debug
        //  Newton.Dump(std::cout);

        return okCompute;
    }

    /// <summary>
    /// The length of sliding of reference for the new constraints. To give a batten <c>b</c> a length L:
    /// <c>b.SetSlidingFactor(L / b.SlidingOfReference())</c>.
    /// </summary>
    public double SlidingOfReference()
    {
        return SlidingOfReference(_newP1.DistanceTo(_newP2), _newAngle1, _newAngle2);
    }

    protected double SlidingOfReference(double dist, double angle1, double angle2)
    {
        double a1, a2;

        // case of angle without constraints
        if ((_newConstraintOrder1 == 0) && (_newConstraintOrder2 == 0))
        {
            return dist;
        }

        if (_newConstraintOrder1 == 0)
        {
            a1 = Math.Abs(Math.Abs(_newAngle2) < M_PI ? angle2 / 2 : M_PI / 2);
        }
        else
        {
            a1 = Math.Abs(angle1);
        }

        if (_newConstraintOrder2 == 0)
        {
            a2 = Math.Abs(Math.Abs(_newAngle1) < M_PI ? angle1 / 2 : M_PI / 2);
        }
        else
        {
            a2 = Math.Abs(angle2);
        }

        // case of angle of the same sign
        if (angle1 * angle2 >= 0)
        {
            return Compute(dist, a1, a2);
        }

        // case of angle of opposite sign
        else
        {
            double sum = a1 + a2;
            if (Math.Abs(sum) < global::Kerf.Engine.Foundation.Precision.Angular())
            {
                return Compute(dist, a1, M_PI / 2);
            }
            double ratio = a1 / sum;
            double angleMilieu = Math.Pow(1 - ratio, 2) * a1 + Math.Pow(ratio, 2) * a2;
            if (angleMilieu > M_PI / 2)
            {
                angleMilieu = M_PI / 2;
            }

            return ratio * Compute(dist, a1, angleMilieu) + (1 - ratio) * Compute(dist, a2, angleMilieu);
        }
    }

    protected double Compute(double dist, double angle1, double angle2)
    {
        double l1 = Compute(dist, angle1);
        double l2 = Compute(dist, angle2), aux;
        if (l1 < l2)
        {
            aux = l2;
            l2 = l1;
            l1 = aux;
        }
        return 0.3 * l1 + 0.7 * l2;
    }

    /// <summary>
    /// The length of the batten for one end's angle: the segment, the circle arc through P1P2 with that angle, or an
    /// extrapolation beyond.
    /// </summary>
    protected double Compute(double dist, double angle)
    {
        if (angle < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return dist;
        } // length of segment P1P2
        if (angle < M_PI / 2)
        {
            return angle * dist / Math.Sin(angle);
        } // length of circle P1P2 respecting ANGLE
        if (angle > M_PI)
        {
            return Math.Sqrt(angle * M_PI) * dist;
        }
        else
        {
            return angle * dist;
        } // linear interpolation
    }

    /// <summary>The free sliding of the latest computation (false by default).</summary>
    public bool GetFreeSliding()
    {
        return _oldFreeSliding;
    }

    /// <summary>The first constraint order of the latest computation.</summary>
    public int GetConstraintOrder1()
    {
        return _oldConstraintOrder1;
    }

    /// <summary>The second constraint order of the latest computation.</summary>
    public int GetConstraintOrder2()
    {
        return _oldConstraintOrder2;
    }

    /// <summary>The first point of the latest computation.</summary>
    public Point2D GetP1()
    {
        return _oldP1;
    }

    /// <summary>The second point of the latest computation.</summary>
    public Point2D GetP2()
    {
        return _oldP2;
    }

    /// <summary>The first angle of the latest computation.</summary>
    public double GetAngle1()
    {
        return _oldAngle1;
    }

    /// <summary>The second angle of the latest computation.</summary>
    public double GetAngle2()
    {
        return _oldAngle2;
    }

    /// <summary>The height (thickness) of the batten of the latest computation.</summary>
    public double GetHeight()
    {
        return _oldHeight;
    }

    /// <summary>The slope of the latest computation.</summary>
    public double GetSlope()
    {
        return _oldSlope;
    }

    /// <summary>The sliding factor of the latest computation.</summary>
    public double GetSlidingFactor()
    {
        return _oldSlidingFactor;
    }

    /// <summary>The computed curve: a new 2D B-spline, frozen (decision 859).</summary>
    public BSplineCurve2D Curve()
    {
        BSplineCurve2D theCurve = new(_poles.Array1(), _knots.Array1(), _mults.Array1(), _degree);
        theCurve.Freeze();
        return theCurve;
    }

    /// <summary>Writes the current state (old and new constraints, the analysis code) to <paramref name="o"/>.</summary>
    public virtual void Dump(TextWriter o)
    {

        o.Write("  Batten       |");
        o.Write(Width7("Old  |   New"));
        o.Write('\n');
        o.Write("  P1    X      |");
        DumpPair(o, _oldP1.X, _newP1.X);
        o.Write("        Y      |");
        DumpPair(o, _oldP1.Y, _newP1.Y);
        o.Write("  P2    X      |");
        DumpPair(o, _oldP2.X, _newP2.X);
        o.Write("        Y      |");
        DumpPair(o, _oldP2.Y, _newP2.Y);
        o.Write("      Angle1   |");
        DumpPair(o, _oldAngle1, _newAngle1);
        o.Write("      Angle2   |");
        DumpPair(o, _oldAngle2, _newAngle2);
        o.Write("      Height   |");
        DumpPair(o, _oldHeight, _newHeight);
        o.Write("      Slope    |");
        DumpPair(o, _oldSlope, _newSlope);
        o.Write(" SlidingFactor |");
        DumpPair(o, _oldSlidingFactor, _newSlidingFactor);
        o.Write(" FreeSliding   |");
        DumpPair(o, _oldFreeSliding ? 1 : 0, _newFreeSliding ? 1 : 0);
        o.Write(" ConstrOrder1  |");
        DumpPair(o, _oldConstraintOrder1, _newConstraintOrder1);
        o.Write(" ConstrOrder2  |");
        DumpPair(o, _oldConstraintOrder2, _newConstraintOrder2);
        DumpCode(o);
    }

    /// <summary>OCCT's <c>operator&lt;&lt;</c>: <see cref="Dump"/> as a string.</summary>
    public override string ToString()
    {
        using StringWriter writer = new(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }

    // o.width(7) applies to the next item only (OCCT writes "old | new" lines with it)
    private protected static string Width7(string text) => StreamUtilities.Setw(7, text);

    private protected static void DumpPair(TextWriter o, double old, double theNew)
    {
        int precision = StreamUtilities.Precision(o);
        o.Write(Width7(StreamUtilities.FormatReal(old, precision)));
        o.Write(" | ");
        o.Write(StreamUtilities.FormatReal(theNew, precision));
        o.Write('\n');
    }

    private protected static void DumpPair(TextWriter o, int old, int theNew)
    {
        o.Write(Width7(old.ToString(CultureInfo.InvariantCulture)));
        o.Write(" | ");
        o.Write(theNew.ToString(CultureInfo.InvariantCulture));
        o.Write('\n');
    }

    private protected void DumpCode(TextWriter o)
    {
        switch (_code)
        {
            case AnalysisCode.OK:
                o.Write("AnalysisCode : Ok" + '\n');
                break;
            case AnalysisCode.NotConverged:
                o.Write("AnalysisCode : NotConverged" + '\n');
                break;
            case AnalysisCode.InfiniteSliding:
                o.Write("AnalysisCode : InfiniteSliding" + '\n');
                break;
            case AnalysisCode.NullHeight:
                o.Write("AnalysisCode : NullHeight" + '\n');
                break;
        }
    }

    private void Angles(in Point2D p1, in Point2D p2)
    {
        Vector2D vOld = new(_newP1, _newP2), vNew = new(p1, p2);
        double dangle = vOld.AngleTo(vNew);
        _newAngle1 -= dangle;
        _newAngle2 += dangle;
    }
}
