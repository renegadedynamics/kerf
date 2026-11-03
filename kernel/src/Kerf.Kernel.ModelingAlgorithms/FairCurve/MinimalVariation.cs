// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_MinimalVariation.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_MinimalVariation.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_MinimalVariation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Fairing.AnalysisCode;

namespace Kerf.Engine.Construction.Fairing;

/// <summary>
/// Computes 2D curves of minimal variation of curvature: a batten whose energy mixes jerk (variation of curvature)
/// and sagging by a physical ratio, with optional curvature constraints at the ends.
/// </summary>
/// <remarks>A working object, as <see cref="Batten"/> (decision 859).</remarks>
public class MinimalVariation : Batten
{
    private double _oldCurvature1;
    private double _oldCurvature2;
    private double _oldPhysicalRatio;
    private double _newCurvature1;
    private double _newCurvature2;
    private double _newPhysicalRatio;

    /// <summary>
    /// A curve through <paramref name="p1"/> and <paramref name="p2"/> of height <paramref name="heigth"/>, slope
    /// <paramref name="slope"/> and physical ratio <paramref name="physicalRatio"/> (0: pure variation of curvature;
    /// 1: a physical batten). Curvatures start at 0, the other parameters as FairCurve_Batten's.
    /// </summary>
    /// <exception cref="NullValueException">P1 and P2 are confused (within Precision::Confusion).</exception>
    /// <exception cref="NegativeValueException"><paramref name="heigth"/> is not positive.</exception>
    public MinimalVariation(in Point2D p1,
                                      in Point2D p2,
                                      double heigth,
                                      double slope = 0,
                                      double physicalRatio = 0)
        : base(p1, p2, heigth, slope)
    {
        _oldCurvature1 = 0;
        _oldCurvature2 = 0;
        _oldPhysicalRatio = physicalRatio;
        _newCurvature1 = 0;
        _newCurvature2 = 0;
        _newPhysicalRatio = physicalRatio;
    }

    /// <summary>Sets the curvature at the first point (used with constraint order 2).</summary>
    public void SetCurvature1(double curvature)
    {
        _newCurvature1 = curvature;
    }

    /// <summary>Sets the curvature at the second point (used with constraint order 2).</summary>
    public void SetCurvature2(double curvature)
    {
        _newCurvature2 = curvature;
    }

    /// <summary>Sets the physical ratio: 0 minimises the variation of curvature only, 1 is a physical batten.</summary>
    /// <exception cref="DomainErrorException"><paramref name="ratio"/> is not in [0, 1].</exception>
    public void SetPhysicalRatio(double ratio)
    {
        Standard_DomainError_Raise_if(ratio < 0 || ratio > 1,
                                      "FairCurve_MinimalVariation::SetPhysicalRatio");
        _newPhysicalRatio = ratio;
    }

    /// <summary>The curvature at the first point of the latest computation.</summary>
    public double GetCurvature1()
    {
        return _oldCurvature1;
    }

    /// <summary>The curvature at the second point of the latest computation.</summary>
    public double GetCurvature2()
    {
        return _oldCurvature2;
    }

    /// <summary>The physical ratio of the latest computation.</summary>
    public double GetPhysicalRatio()
    {
        return _oldPhysicalRatio;
    }

    /// <summary>
    /// Computes the curve with respect to the constraints. <paramref name="aCode"/> is OK, NotConverged,
    /// InfiniteSliding or NullHeight.
    /// </summary>
    public override bool Compute(out AnalysisCode aCode, int nbIterations = 50, double tolerance = 1.0e-3)
    {
        bool ok = true, end = false;
        double angleMax = 0.7; // parameter regulating the function of increment ( 40 degrees )
        double angleMin = 2 * M_PI / 100; // parameter regulating the function of increment
                                          // full passage should not contain more than 100 steps.
        double dAngle1, dAngle2, dRho1, dRho2, ratio, fraction, toler;
        double oldDist, newDist;
        // the loop runs at least once, and its first call writes the code (decision 860)
        aCode = AnalysisCode.OK;

        //  Loop of Homotopy : calculation of the step and optimisation

        while (ok && !end)
        {
            dAngle1 = _newAngle1 - _oldAngle1;
            dAngle2 = _newAngle2 - _oldAngle2;
            dRho1 = _newCurvature1 - _oldCurvature1;
            dRho2 = _newCurvature2 - _oldCurvature2;
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

            if (_newConstraintOrder1 > 1)
            {
                fraction = Math.Abs(dRho1) * oldDist / (2 + Math.Abs(_oldAngle1) + Math.Abs(_oldAngle2));
                if (fraction > 1)
                {
                    ratio = (ratio < 1 / fraction ? ratio : 1 / fraction);
                }
            }

            if (_newConstraintOrder2 > 1)
            {
                fraction = Math.Abs(dRho2) * oldDist / (2 + Math.Abs(_oldAngle1) + Math.Abs(_oldAngle2));
                if (fraction > 1)
                {
                    ratio = (ratio < 1 / fraction ? ratio : 1 / fraction);
                }
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
                dRho1 *= ratio;
                dRho2 *= ratio;
                toler = 10 * tolerance;
            }

            ok = Compute(deltaP1, deltaP2, dAngle1, dAngle2, dRho1, dRho2, out aCode, nbIterations, toler);

            if (aCode != AnalysisCode.OK)
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
            if (_newConstraintOrder1 < 2)
            {
                _newCurvature1 = _oldCurvature1;
            }
            if (_newConstraintOrder2 == 0)
            {
                _newAngle2 = _oldAngle2;
            }
            if (_newConstraintOrder2 < 2)
            {
                _newCurvature2 = _oldCurvature2;
            }
        }
        _code = aCode;
        return ok;
    }

    private bool Compute(in Vector2D deltaP1,
                         in Vector2D deltaP2,
                         double deltaAngle1,
                         double deltaAngle2,
                         double deltaCurvature1,
                         double deltaCurvature2,
                         out AnalysisCode aCode,
                         int nbIterations,
                         double tolerance)
    {
        bool ok, okCompute = true;
        aCode = AnalysisCode.OK;

        // Deformation of the curve by adding a polynom of interpolation
        int l = 2 + _newConstraintOrder1 + _newConstraintOrder2, kk, ii;
        // (OCCT's debug build counts the poles here)
        NCollection_Array1<double> knots = new(1, 2);
        knots[1] = 0;
        knots[2] = 1;
        NCollection_Array1<int> mults = new(1, 2);
        NCollection_Array1<Point2D> hermitePoles = new(1, l);
        NCollection_Array1<Point2D> interpolation = new(1, l);
        NCollection_HArray1<Point2D> nPoles = new(1, _poles.Length());

        // Polynomes of Hermite
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
        double dAngle1 = deltaAngle1 - dAngleRef,
               dAngle2 = dAngleRef - deltaAngle2; // Correction of Delta by the Delta induced by the points.

        aDelta[1] = deltaP1.XY();
        kk = 2;
        if (_newConstraintOrder1 > 0)
        {
            // rotation of the derivative premiereDeltaAngle1
            Vector2D oldDerive = new(_poles.Value(_poles.Lower()), _poles.Value(_poles.Lower() + 1));
            double knotGapL = _knots.Value(_knots.Lower() + 1) - _knots.Value(_knots.Lower());
            if (Math.Abs(knotGapL) > global::Kerf.Engine.Foundation.Precision.Computational())
            {
                oldDerive *= _degree / knotGapL;
            }
            aDelta[kk] = (oldDerive.Rotated(dAngle1) - oldDerive).XY();
            kk += 1;

            if (_newConstraintOrder1 > 1)
            {
                // rotation of the second derivative + adding
                Vector2D oldSeconde = new(_poles.Value(_poles.Lower()).XY() + _poles.Value(_poles.Lower() + 2).XY()
                                          - 2 * _poles.Value(_poles.Lower() + 1).XY());
                double knotGapLSq = knotGapL * knotGapL;
                if (Math.Abs(knotGapLSq) > global::Kerf.Engine.Foundation.Precision.SquareComputational())
                {
                    oldSeconde *= _degree * (_degree - 1) / knotGapLSq;
                }
                double cPrim = oldDerive.Magnitude;
                aDelta[kk] = (oldSeconde.Rotated(dAngle1) - oldSeconde
                              + deltaCurvature1 * cPrim * oldDerive.Rotated(M_PI / 2 + dAngle1))
                               .XY();
                kk += 1;
            }
        }
        aDelta[kk] = deltaP2.XY();
        kk += 1;
        if (_newConstraintOrder2 > 0)
        {
            Vector2D oldDerive = new(_poles.Value(_poles.Upper() - 1), _poles.Value(_poles.Upper()));
            double knotGapU = _knots.Value(_knots.Upper()) - _knots.Value(_knots.Upper() - 1);
            if (Math.Abs(knotGapU) > global::Kerf.Engine.Foundation.Precision.Computational())
            {
                oldDerive *= _degree / knotGapU;
            }
            aDelta[kk] = (oldDerive.Rotated(dAngle2) - oldDerive).XY();
            kk += 1;
            if (_newConstraintOrder2 > 1)
            {
                // rotation of the second derivative + adding
                Vector2D oldSeconde = new(_poles.Value(_poles.Upper()).XY() + _poles.Value(_poles.Upper() - 2).XY()
                                          - 2 * _poles.Value(_poles.Upper() - 1).XY());
                double knotGapUSq = knotGapU * knotGapU;
                if (Math.Abs(knotGapUSq) > global::Kerf.Engine.Foundation.Precision.SquareComputational())
                {
                    oldSeconde *= _degree * (_degree - 1) / knotGapUSq;
                }
                double cPrim = oldDerive.Magnitude;
                aDelta[kk] = (oldSeconde.Rotated(dAngle2) - oldSeconde
                              + deltaCurvature2 * cPrim * oldDerive.Rotated(M_PI / 2 + dAngle2))
                               .XY();
                kk += 1;
            }
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

        // Intermediaires

        double angle1, angle2, slidingLength,
          alph1 = _oldAngle1 + deltaAngle1, alph2 = _oldAngle2 + deltaAngle2,
          rho1 = _oldCurvature1 + deltaCurvature1, rho2 = _oldCurvature2 + deltaCurvature2,
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

        // Energy and vectors of initialization
        BattenLaw lBatten = new(_newHeight, _newSlope, slidingLength);
        EnergyOfMvc emvc = new(_degree + 1,
                                         _flatknots,
                                         nPoles,
                                         _newConstraintOrder1,
                                         _newConstraintOrder2,
                                         lBatten,
                                         _newPhysicalRatio,
                                         slidingLength,
                                         _newFreeSliding,
                                         angle1,
                                         angle2,
                                         rho1,
                                         rho2);
        math_Vector vInit = new(1, emvc.VariableCount);

        // The value below gives an idea about the smallest value of the criterion of flexion.
        double vConvex = 0.01 * Math.Pow(_newHeight / slidingLength, 3);
        if (vConvex < 1.0e-12)
        {
            vConvex = 1.0e-12;
        }

        ok = emvc.Variable(vInit);

        // Minimisation
        Newton newton = new(emvc,
                                      tolerance * (p1P2.Magnitude / 10),
                                      tolerance,
                                      nbIterations,
                                      vConvex);
        newton.Perform(emvc, vInit);
        ok = newton.IsDone;

        if (ok)
        {
            Vector2D tangente = new(), pseudoNormale = new();
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

            if (_newConstraintOrder1 < 2)
            {
                tangente = tangente.WithXY(_poles.Value(_poles.Lower() + 1).XY() - _poles.Value(_poles.Lower()).XY());

                if (_newConstraintOrder1 == 0)
                {
                    _oldAngle1 = p1P2.AngleTo(tangente);
                }
                else
                {
                    _oldAngle1 = alph1;
                }

                pseudoNormale = pseudoNormale.WithXY(_poles.Value(_poles.Lower()).XY()
                                    - 2 * _poles.Value(_poles.Lower() + 1).XY()
                                    + _poles.Value(_poles.Lower() + 2).XY());
                _oldCurvature1 = (((double)_degree - 1) / _degree) * (tangente.Normalized() ^ pseudoNormale)
                                / tangente.SquareMagnitude;
            }
            else
            {
                _oldCurvature1 = rho1;
                _oldAngle1 = alph1;
            }

            if (_newConstraintOrder2 < 2)
            {
                tangente = tangente.WithXY(_poles.Value(_poles.Upper()).XY() - _poles.Value(_poles.Upper() - 1).XY());
                if (_newConstraintOrder2 == 0)
                {
                    _oldAngle2 = (-tangente).AngleTo(-p1P2);
                }
                else
                {
                    _oldAngle2 = alph2;
                }
                pseudoNormale = pseudoNormale.WithXY(_poles.Value(_poles.Upper()).XY()
                                    - 2 * _poles.Value(_poles.Upper() - 1).XY()
                                    + _poles.Value(_poles.Upper() - 2).XY());
                _oldCurvature2 = (((double)_degree - 1) / _degree) * (tangente.Normalized() ^ pseudoNormale)
                                / tangente.SquareMagnitude;
            }
            else
            {
                _oldAngle2 = alph2;
                _oldCurvature2 = rho2;
            }

            _oldP1 = _poles.Value(_poles.Lower());
            _oldP2 = _poles.Value(_poles.Upper());
            _oldConstraintOrder1 = _newConstraintOrder1;
            _oldConstraintOrder2 = _newConstraintOrder2;
            _oldFreeSliding = _newFreeSliding;
            _oldSlope = _newSlope;
            _oldHeight = _newHeight;
            _oldPhysicalRatio = _newPhysicalRatio;
        }
        else
        {
            double v;
            aCode = emvc.Status();
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

        ok = emvc.Variable(vInit);

        // Processing of non convergence
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
        while (valAngles > (2 * (nbKnots - 2) + 1) * (1 + 2 * nbKnots))
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

        // For eventual debug Newton.Dump(std::cout);

        return okCompute;
    }

    /// <summary>Writes the current state (old and new constraints, the analysis code) to <paramref name="o"/>.</summary>
    public override void Dump(TextWriter o)
    {

        o.Write("  MVCurve      |");
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
        o.Write(" Curvature1    |");
        DumpPair(o, _oldCurvature1, _newCurvature1);
        o.Write(" Curvature2    |");
        DumpPair(o, _oldCurvature2, _newCurvature2);
        o.Write("      Height   |");
        DumpPair(o, _oldHeight, _newHeight);
        o.Write("      Slope    |");
        DumpPair(o, _oldSlope, _newSlope);
        o.Write(" PhysicalRatio |");
        DumpPair(o, _oldPhysicalRatio, _newPhysicalRatio);
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
}
