// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_ValidateEdge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_ValidateEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Healing.Checks;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Library;

/// <summary>
/// Computes the max distance between a 3D curve and a curve on a surface, approximately at a finite number of
/// points (the default) or exactly through <see cref="CheckCurveOnSurface"/> (for SameParameter edges).
/// </summary>
/// <remarks>A working object; it keeps the caller's adaptors.</remarks>
internal class ValidateEdge
{
    private readonly CurveAdaptorBase _referenceCurve;
    private readonly CurveOnSurface _otherCurve;
    private readonly bool _sameParameter;
    private int _controlPointsNumber;
    private double _toleranceForChecking;
    private double _calculatedDistance;
    private bool _exitIfToleranceExceeded;
    private bool _isDone;
    private bool _isExactMethod;
    private bool _isMultiThread;

    public ValidateEdge(CurveAdaptorBase referenceCurve,
                                CurveOnSurface otherCurve,
                                bool sameParameter)
    {
        _referenceCurve = referenceCurve;
        _otherCurve = otherCurve;
        _sameParameter = sameParameter;
        _controlPointsNumber = 22;
        _toleranceForChecking = 0;
        _calculatedDistance = 0;
        _exitIfToleranceExceeded = false;
        _isDone = false;
        _isExactMethod = false;
        _isMultiThread = false;
    }

    /// <summary>
    /// Selects the exact method (BRepLib_CheckCurveOnSurface's, slower) instead of a finite number of points. The
    /// exact method is used only for SameParameter edges.
    /// </summary>
    public void SetExactMethod(bool isExact) => _isExactMethod = isExact;

    public bool IsExactMethod => _isExactMethod;

    public void SetParallel(bool isMultiThread) => _isMultiThread = isMultiThread;

    public bool IsParallel => _isMultiThread;

    /// <summary>Sets the number of control points (22 by default).</summary>
    public void SetControlPointsNumber(int controlPointsNumber) => _controlPointsNumber = controlPointsNumber;

    /// <summary>
    /// Stops <see cref="Process"/> as soon as a distance greater than the tolerance (with a margin) is found. No effect
    /// with the exact method.
    /// </summary>
    public void SetExitIfToleranceExceeded(double toleranceForChecking)
    {
        _exitIfToleranceExceeded = true;
        _toleranceForChecking = CorrectTolerance(toleranceForChecking);
    }

    /// <summary>Computes the max distance between the curves.</summary>
    public void Process()
    {
        if (_isExactMethod && _sameParameter)
        {
            ProcessExact();
        }
        else
        {
            ProcessApprox();
        }
    }

    /// <summary>True if the distance has been found at all points.</summary>
    public bool IsDone => _isDone;

    /// <summary>True if the computed distance is less than the tolerance (with a margin).</summary>
    public bool CheckTolerance(double toleranceToCheck)
        => CorrectTolerance(toleranceToCheck) > _calculatedDistance;

    /// <summary>The max distance, increased by a relative 1e-5.</summary>
    public double GetMaxDistance()
    {
        double correctedTolerance = _calculatedDistance * 1.00001;
        return correctedTolerance;
    }

    /// <summary>Increases <paramref name="toleranceToUpdate"/> to the max distance (increased by 1e-5) if smaller.</summary>
    public void UpdateTolerance(ref double toleranceToUpdate)
    {
        double correctedTolerance = _calculatedDistance * 1.00001;
        if (correctedTolerance > toleranceToUpdate)
        {
            toleranceToUpdate = correctedTolerance;
        }
    }

    /// <summary>Adds the precision of the curves' numbers to the tolerance.</summary>
    private double CorrectTolerance(double tolerance)
    {
        SurfaceAdaptorBase surface = _otherCurve.GetSurface()!;
        double curvePrecision = BRepCheckCheck.PrecCurve(_referenceCurve);
        double surfacePrecision = BRepCheckCheck.PrecSurface(surface);
        double toleranceDelta = (curvePrecision > surfacePrecision) ? curvePrecision : surfacePrecision;
        double correctedTolerance = tolerance + toleranceDelta;
        return correctedTolerance;
    }

    /// <summary>The distance at a finite number of points.</summary>
    private void ProcessApprox()
    {
        _isDone = true;
        double squareToleranceForChecking = _toleranceForChecking * _toleranceForChecking;
        double referenceFirstParam = _referenceCurve.FirstParameter();
        double referenceLastParam = _referenceCurve.LastParameter();
        double otherFirstParam = _otherCurve.FirstParameter();
        double otherLastParam = _otherCurve.LastParameter();
        double maxSquareDistance = 0.0;

        int controlPointsNumber = (_controlPointsNumber < 1) ? 1 : _controlPointsNumber;
        bool isProjection =
            (!_sameParameter
             || Math.Abs(otherFirstParam - referenceFirstParam) > global::Kerf.Engine.Foundation.Precision.PConfusion()
             || Math.Abs(otherLastParam - referenceLastParam) > global::Kerf.Engine.Foundation.Precision.PConfusion());

        if (!isProjection)
        {
            for (int i = 0; i <= controlPointsNumber; ++i)
            {
                double controlPointParam =
                    ((controlPointsNumber - i) * referenceFirstParam + i * referenceLastParam) / controlPointsNumber;
                Point3D referencePoint = _referenceCurve.Value(controlPointParam);
                Point3D otherPoint = _otherCurve.Value(controlPointParam);
                double squareDistance = referencePoint.SquareDistanceTo(otherPoint);
                if (squareDistance > maxSquareDistance)
                {
                    maxSquareDistance = squareDistance;
                }

                // Stop process for best performance
                if (_exitIfToleranceExceeded && maxSquareDistance > squareToleranceForChecking)
                {
                    _calculatedDistance = Math.Sqrt(maxSquareDistance);
                    return;
                }
            }
        }
        else
        {
            Point3D referencePoint = _referenceCurve.Value(referenceFirstParam);
            Point3D otherPoint = _otherCurve.Value(otherFirstParam);
            double squareDistance = referencePoint.SquareDistanceTo(otherPoint);
            if (squareDistance > maxSquareDistance)
            {
                maxSquareDistance = squareDistance;
            }

            if (_exitIfToleranceExceeded && maxSquareDistance > squareToleranceForChecking)
            {
                _calculatedDistance = Math.Sqrt(maxSquareDistance);
                return;
            }

            referencePoint = _referenceCurve.Value(referenceLastParam);
            otherPoint = _otherCurve.Value(otherLastParam);
            squareDistance = referencePoint.SquareDistanceTo(otherPoint);
            if (squareDistance > maxSquareDistance)
            {
                maxSquareDistance = squareDistance;
            }

            if (_exitIfToleranceExceeded && maxSquareDistance > squareToleranceForChecking)
            {
                _calculatedDistance = Math.Sqrt(maxSquareDistance);
                return;
            }

            var referenceExtrema = new LocateExtPC();
            var otherExtrema = new LocateExtPC();
            referenceExtrema.Initialize(_referenceCurve,
                                         referenceFirstParam,
                                         referenceLastParam,
                                         _referenceCurve.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()));
            otherExtrema.Initialize(_otherCurve,
                                      otherFirstParam,
                                      otherLastParam,
                                      _otherCurve.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()));
            for (int i = 1; i < controlPointsNumber; i++)
            {
                double referenceParam =
                    ((controlPointsNumber - i) * referenceFirstParam + i * referenceLastParam) / controlPointsNumber;
                Point3D referenceExtremaPoint = _referenceCurve.Value(referenceParam);
                double otherParam =
                    ((controlPointsNumber - i) * otherFirstParam + i * otherLastParam) / controlPointsNumber;
                Point3D otherExtremaPoint = _otherCurve.Value(otherParam);

                referenceExtrema.Perform(otherExtremaPoint, referenceParam);
                if (referenceExtrema.IsDone)
                {
                    if (referenceExtrema.SquareDistance() > maxSquareDistance)
                    {
                        maxSquareDistance = referenceExtrema.SquareDistance();
                    }

                    if (_exitIfToleranceExceeded && maxSquareDistance > squareToleranceForChecking)
                    {
                        _calculatedDistance = Math.Sqrt(maxSquareDistance);
                        return;
                    }
                }
                else
                {
                    _isDone = false;
                    // Stop process for best performance
                    return;
                }

                otherExtrema.Perform(referenceExtremaPoint, otherParam);
                if (otherExtrema.IsDone)
                {
                    if (otherExtrema.SquareDistance() > maxSquareDistance)
                    {
                        maxSquareDistance = otherExtrema.SquareDistance();
                    }

                    if (_exitIfToleranceExceeded && maxSquareDistance > squareToleranceForChecking)
                    {
                        _calculatedDistance = Math.Sqrt(maxSquareDistance);
                        return;
                    }
                }
                else
                {
                    _isDone = false;
                    // Stop process for best performance
                    return;
                }
            }
        }

        _calculatedDistance = Math.Sqrt(maxSquareDistance);
    }

    /// <summary>The exact distance, by GeomLib_CheckCurveOnSurface.</summary>
    private void ProcessExact()
    {
        var checkCurveOnSurface = new CheckCurveOnSurface(_referenceCurve);
        checkCurveOnSurface.SetParallel(_isMultiThread);
        checkCurveOnSurface.Perform(_otherCurve);
        _isDone = checkCurveOnSurface.IsDone;
        if (_isDone)
        {
            _calculatedDistance = checkCurveOnSurface.MaxDistance();
        }
    }
}
