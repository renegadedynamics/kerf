// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_ApproxArcsSegments.hxx
// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_ApproxArcsSegments.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Approximation of a free-form 2D curve by a sequence of arcs of circle and line segments, within a distance
/// tolerance and an angular tolerance at the junctions. The approximation is made by the constructor.
/// </summary>
internal sealed class ApproxArcsSegments2D
{
    /// <summary>The state of the approximation.</summary>
    public enum Status
    {
        StatusOK = 0,
        StatusNotDone,
        StatusError,
    }

    private const int Maxpoints = 100;
    private const double MyCurvatureTolerance = 0.0001;

    private readonly Curve2DAdaptorBase _curve;
    private readonly PPoint2D[] _ext = [new(), new()];

    private readonly double _tolerance;
    private readonly double _angleTolerance;

    private readonly NCollection_Sequence<PPoint2D> _seqParams = new();
    private readonly NCollection_Sequence<Curve2D> _seqCurves = new();
    private Status _status;

    /// <summary>
    /// Approximates theCurve with the distance tolerance theTolerance and the angular tolerance theAngleTol at the
    /// junctions. The adaptor is kept, as OCCT keeps a reference to it, but only during the construction.
    /// </summary>
    public ApproxArcsSegments2D(Curve2DAdaptorBase curve, double tolerance, double angleTol)
    {
        _curve = curve;
        _tolerance = tolerance;
        _angleTolerance = angleTol;
        _status = Status.StatusNotDone;

        _ext[0] = new PPoint2D(_curve.FirstParameter(), _curve);
        _ext[1] = new PPoint2D(_curve.LastParameter(), _curve);

        switch (_curve.GetType())
        {
            case Line:
            {
                // Create a single line segment.
                double dist = _ext[0].Dist(_ext[1]);
                if (dist > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    Axis2D ax2d = new(new Point2D(_ext[0].Point()),
                                         new Direction2D(new Vector2D(new Point2D(_ext[0].Point()), new Point2D(_ext[1].Point()))));
                    LineCurve2D line = new(ax2d);
                    _seqCurves.Append(new TrimmedCurve2D(line, 0.0, dist));
                    _status = Status.StatusOK;
                }
            }
            break;
            case Circle:
            {
                // Create a couple of arcs of equal size.
                PPoint2D pp = new(.5 * (_ext[0].Parameter() + _ext[1].Parameter()), _curve);
                Curve2D? aCurve = MakeCircle(_ext[0], pp);
                if (aCurve is not null)
                {
                    _seqCurves.Append(aCurve);
                    aCurve = MakeCircle(pp, _ext[1]);
                    if (aCurve is not null)
                    {
                        _seqCurves.Append(aCurve);
                    }
                }
            }
            break;
            default:
                MakeFreeform();
                break;
        }

        // Check status of the calculation
        if (_status == Status.StatusNotDone)
        {
            if (!_seqCurves.IsEmpty())
            {
                _status = Status.StatusOK;
            }
            else
            {
                _status = Status.StatusError;
            }
        }
    }

    /// <summary>
    /// The curves of the approximation (lines and arcs of circle, as trimmed curves), in order along the curve; a
    /// new list each call, holding the same curves, frozen.
    /// </summary>
    public IReadOnlyList<Curve2D> GetResult()
    {
        var curves = new Curve2D[_seqCurves.Length()];
        for (int i = 1; i <= _seqCurves.Length(); i++)
        {
            Curve2D curve = _seqCurves.Value(i);
            curve.Freeze();
            curves[i - 1] = curve;
        }

        return ImmutableArrays.Own(curves);
    }

    // Create arc of circle by three points (knowing that myCurve is circle).
    private Curve2D? MakeCircle(in PPoint2D first, in PPoint2D last)
    {
        Curve2D? result = null;
        double paraM = (first.Parameter() + last.Parameter()) * .5;
        _curve.D0(paraM, out Point2D pointM);
        MakeArcOfCircle2d makeArc1 = new(new Point2D(first.Point()), pointM, new Point2D(last.Point()));

        if (makeArc1.IsDone)
        {
            result = makeArc1.Value();
        }

        return result;
    }

    // Create an arc of circle using 2 points and a derivative in the first point.
    private bool MakeArc(in PPoint2D param1,
                         ref PPoint2D param2,
                         bool isFirst,
                         out TrimmedCurve2D? curve)
    {
        curve = null;
        Xy p1 = param1.Point();
        Xy p2 = param2.Point();
        Xy vec = isFirst ? param1.D1() : -param1.D1();

        // Detect the sense (CCW means positive)
        Xy delta = p2 - p1;
        double sense = vec ^ delta;
        if (sense > global::Kerf.Engine.Foundation.Precision.Angular())
        {
            sense = 1.0;
        }
        else if (sense < -global::Kerf.Engine.Foundation.Precision.Angular())
        {
            sense = -1.0;
        }
        else
        {
            return false;
        }

        // Find the centre of the circle
        Xy middle = (p2 + p1) * 0.5;
        double prodP1V = p1 * vec;
        double prodDeM = delta * middle;
        double vprodVD = vec ^ delta;
        double resolution = Constants.Resolution();

        if (vprodVD < -resolution || vprodVD > resolution)
        {
            Point2D center = new((prodP1V * delta.Y() - prodDeM * vec.Y()) / vprodVD,
                                   (prodDeM * vec.X() - prodP1V * delta.X()) / vprodVD);
            double rad = (center.DistanceTo(new Point2D(p1)) + center.DistanceTo(new Point2D(p2))) * 0.5;
            Frame2D ax22d = new(center, new Direction2D(Direction2D.D.X), new Direction2D(Direction2D.D.Y));
            Circle2D cir = new(ax22d, rad);
            CircleCurve2D circ = new(cir);

            // calculation parameters first and last points of arc.
            double alpha1, alpha2;
            if (isFirst)
            {
                alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir, new Point2D(p1));
                alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir, new Point2D(p2));
            }
            else
            {
                alpha1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir, new Point2D(p2));
                alpha2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(cir, new Point2D(p1));
                sense = -sense;
            }

            if (Math.Abs(alpha1 - alpha2) < 1e-100)
            {
                // very small value, just to avoid exact match
                return false;
            }

            // Reverse the circle if the sense is negative
            if (sense < 0.0)
            {
                alpha1 = circ.ReversedParameter(alpha1);
                alpha2 = circ.ReversedParameter(alpha2);
                circ.Reverse();
            }

            curve = new TrimmedCurve2D(circ, alpha1, alpha2);
            // Correct the direction in the opposite point
            Xy radV = param2.Point() - center.XY();
            param2.SetD1(new Xy(-radV.Y() * sense, radV.X() * sense));
            return true;
        }

        // Algorithm failed, possibly because aVec is normal to the chorde
        return false;
    }

    // Make a line from myCurve in the limits by parameter from theFirst to theLast
    private TrimmedCurve2D? MakeLine(ref PPoint2D first,
                                          ref PPoint2D last,
                                          bool isCheck)
    {
        TrimmedCurve2D? result = null;

        Xy slope = last.Point() - first.Point();
        if (Math.Abs(slope.SquareModulus()) < Constants.Resolution())
        {
            return result;
        }

        Direction2D dirLine = new(slope);

        if (isCheck)
        {
            if (first.D1().SquareModulus() < Constants.Resolution() || last.D1().SquareModulus() < Constants.Resolution())
            {
                return result;
            }

            // Angular continuity (G1) is only checked when the end of the line is not
            // on the extremity of the curve
            Span<double> absAngle = [0.0, 0.0];
            if (first.IsNotEqual(_ext[0]))
            {
                double ang = dirLine.AngleTo(new Direction2D(first.D1()));
                absAngle[0] = (ang > 0.0 ? ang : -ang);
            }

            if (last.IsNotEqual(_ext[1]))
            {
                double ang = dirLine.AngleTo(new Direction2D(last.D1()));
                absAngle[1] = (ang > 0.0 ? ang : -ang);
            }

            // if the derivatives in the end points differ from the derivative line
            // more than value of the specified continuity tolerance
            // then a biarc should be built instead of a line.
            double contTolerance = Max(_angleTolerance, 0.01);
            if (absAngle[0] > contTolerance || absAngle[1] > contTolerance)
            {
                return result;
            }
        } // end if (isCheck)

        // building segment of line
        MakeSegment2d makeSeg = new(new Point2D(first.Point()), new Point2D(last.Point()));
        if (makeSeg.IsDone)
        {
            TrimmedCurve2D curve = makeSeg.Value();
            if (CheckCurve(curve, first.Parameter(), last.Parameter()))
            {
                result = curve;
                // correct the derivatives fields in both arguments
                Xy newD1 = last.Point() - first.Point();
                first.SetD1(newD1);
                last.SetD1(newD1);
            }
        }

        return result;
    }

    // Create a sequence of elementary curves from a free-form adaptor curve.
    private bool MakeFreeform()
    {
        NCollection_Sequence<PPoint2D> seqParamPoints = new();
        ref PPoint2D prevParam = ref _ext[0];

        // calculation of the inflection points.
        GetLinearParts(seqParamPoints);
        bool isNoInfPoints = seqParamPoints.IsEmpty();

        NCollection_Sequence<Curve2D?> seqLinearParts = new();
        bool isDone = true;
        int i;
        for (i = 1; i < seqParamPoints.Length(); i += 2)
        {
            Curve2D? lineCurve = null;
            ref PPoint2D param0 = ref seqParamPoints.ChangeValue(i);
            ref PPoint2D param1 = ref seqParamPoints.ChangeValue(i + 1);
            if (param0.IsNotEqual(param1))
            {
                // linear part of the curve lies between odd and even values of i.
                // parameters from parameter's sequence.
                lineCurve = MakeLine(ref param0, ref param1, false);
            }

            seqLinearParts.Append(lineCurve);
        }

        for (i = 1; i < seqParamPoints.Length(); i += 2)
        {
            // approximation for non-linear part preceding the linear part
            if (seqParamPoints[i].IsNotEqual(prevParam))
            {
                int lastInd = _seqCurves.Length();
                isDone = MakeApproximation(ref prevParam, ref seqParamPoints[i]);
                if (isDone && lastInd != 0 && _seqCurves.Length() > lastInd)
                {
                    isDone = CheckContinuity(_seqCurves.Value(lastInd), _seqCurves.Value(lastInd + 1), _angleTolerance);
                }

                if (!isDone)
                {
                    _status = Status.StatusError;
                    break;
                }
            }

            Curve2D? curve = seqLinearParts.Value((i + 1) / 2);
            if (curve is not null)
            {
                _seqCurves.Append(curve);
            }
            else
            {
                ref PPoint2D param0 = ref seqParamPoints.ChangeValue(i);
                ref PPoint2D param1 = ref seqParamPoints.ChangeValue(i + 1);
                int lastInd = _seqCurves.Length();
                isDone = MakeApproximation(ref param0, ref param1);
                if (isDone && lastInd != 0 && _seqCurves.Length() > lastInd)
                {
                    isDone = CheckContinuity(_seqCurves.Value(lastInd), _seqCurves.Value(lastInd + 1), _angleTolerance);
                }

                if (!isDone)
                {
                    _status = Status.StatusError;
                    break;
                }
            }

            prevParam = ref seqParamPoints[i + 1];
        }

        // approximation for non-linear part following the last linear part
        if (isDone && prevParam.IsNotEqual(_ext[1]))
        {
            // Case of a closed edge like an ellipse
            if (isNoInfPoints && (_ext[0].Point() - _ext[1].Point()).Modulus() < _tolerance)
            {
                PPoint2D pPoint = new(0.5 * (_ext[0].Parameter() + _ext[1].Parameter()), _curve);
                isDone = MakeApproximation(ref _ext[0], ref pPoint);
                if (isDone)
                {
                    isDone = MakeApproximation(ref pPoint, ref _ext[1]);
                }
            }
            else
            {
                isDone = MakeApproximation(ref prevParam, ref _ext[1]);
            }

            if (!isDone)
            {
                _status = Status.StatusError;
            }
        }

        return (_seqCurves.Length() != 0 && _status != Status.StatusError);
    }

    // Obtain the linear intervals on the curve using as criteria
    // curvature tolerance (indicating either linear part or inflection)
    private void GetLinearParts(NCollection_Sequence<PPoint2D> seqPar)
    {
        int i;
        // Fill the sequences with values along the curve
        _seqParams.Clear();
        QuasiUniformDeflection quDefAlgo = new(_curve, _tolerance * 0.5);
        bool isUniformDone = quDefAlgo.IsDone;

        Xy lastPnt = _ext[0].Point();
        if (isUniformDone)
        {
            for (i = 1; i <= quDefAlgo.PointCount; i++)
            {
                PPoint2D pp = new(quDefAlgo.Parameter(i), _curve);
                _seqParams.Append(pp);
                lastPnt = pp.Point();
            }
        }
        else
        {
            double paramStep = (_ext[1].Parameter() - _ext[0].Parameter()) / Maxpoints;
            for (i = 1; i <= Maxpoints; i++)
            {
                double param = _ext[0].Parameter() + paramStep * i;
                PPoint2D pp = new(param, _curve);
                _seqParams.Append(pp);
                lastPnt = pp.Point();
            }
        }

        // check if the curve may be linearised
        Xy dir = _ext[1].Point() - _ext[0].Point();
        double mod2 = dir.SquareModulus();
        if (mod2 > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            bool isLinear = true;
            dir /= Math.Sqrt(mod2);
            for (i = 1; i <= _seqParams.Length(); i++)
            {
                // Distance from point "i" to the segment between two extremities
                double dist = dir ^ (_seqParams[i].Point() - _ext[0].Point());
                if (dist > _tolerance * 0.5 || dist < -_tolerance * 0.5)
                {
                    isLinear = false;
                    break;
                }
            }

            if (isLinear)
            {
                seqPar.Append(_ext[0]);
                seqPar.Append(_ext[1]);
                return;
            }
        }

        // check if point for First Parameter is inflection point.
        int indStartLinear = 0;
        PPoint2D lastInflParam = _ext[0];
        PPoint2D firstInflParam = _ext[0];

        // Getting further inflection points with step by parameter.
        // The point with index 1 is the same as myExt[0]
        for (i = 1; i <= _seqParams.Length(); i++)
        {
            PPoint2D curParam = _seqParams[i];
            if (indStartLinear != 0)
            {
                bool isStillInflectionFirst =
                    IsInflectionPoint(firstInflParam.Parameter(), curParam, _curve, _angleTolerance);
                if (IsInflectionPoint(curParam.Parameter(), firstInflParam, _curve, _angleTolerance))
                {
                    lastInflParam = _seqParams[i];
                    while (!isStillInflectionFirst)
                    {
                        if (++indStartLinear >= i)
                        {
                            indStartLinear = 0;
                            break;
                        }

                        firstInflParam = _seqParams[indStartLinear];
                        isStillInflectionFirst =
                            IsInflectionPoint(firstInflParam.Parameter(), curParam, _curve, _angleTolerance);
                    }
                }
                else
                {
                    // Add the interval in the output sequence
                    // The interval is added only if it is more than 10 times the tolerance
                    lastInflParam = FindInflection(lastInflParam, curParam);
                    if (!IsInflectionPoint(firstInflParam.Parameter(), lastInflParam, _curve, _angleTolerance))
                    {
                        firstInflParam = FindInflection(lastInflParam, firstInflParam);
                    }

                    double dist = (firstInflParam.Point() - lastInflParam.Point()).Modulus();
                    if (firstInflParam.Parameter() < lastInflParam.Parameter() && dist > 10 * _tolerance)
                    {
                        seqPar.Append(firstInflParam);
                        seqPar.Append(lastInflParam);
                    }

                    indStartLinear = 0;
                }
            }
            else if (IsInflectionPoint(curParam.Parameter(), _curve))
            {
                lastInflParam = curParam;
                if (i > 1)
                {
                    firstInflParam = FindInflection(curParam, _seqParams[i - 1]);
                }

                indStartLinear = i;
            }
        }

        double dist2 = (firstInflParam.Point() - _ext[1].Point()).Modulus();
        if (indStartLinear != 0 && dist2 > 10 * _tolerance)
        {
            seqPar.Append(firstInflParam);
            seqPar.Append(_ext[1]);
        }
    }

    // Dichotomic search of the boundary of inflection interval, between
    // two parameters on the Curve
    private PPoint2D FindInflection(in PPoint2D paramIsInfl,
                                                in PPoint2D paramNoInfl)
    {
        double lower = paramIsInfl.Parameter();
        double upper = paramNoInfl.Parameter();
        double test = 0.0;
        for (int i = 0; i < 3; i++)
        { // 3 iterations
            test = (lower + upper) * 0.5;
            if (IsInflectionPoint(test, paramIsInfl, _curve, _angleTolerance))
            {
                lower = test;
            }
            else
            {
                upper = test;
            }
        }

        return new PPoint2D(test, _curve);
    }

    // Make approximation non-linear part of the other curve.
    private bool MakeApproximation(ref PPoint2D firstParam, ref PPoint2D lastParam)
    {
        // if difference between parameters is less than Precision::PConfusion
        // approximation was not made.
        bool isDone = false;
        if (lastParam.IsNotEqual(firstParam))
        {
            double distance = (firstParam.Point() - lastParam.Point()).Modulus();
            if (distance < _tolerance)
            {
                Curve2D? curve = MakeLine(ref firstParam, ref lastParam, true);
                isDone = curve is not null;
                if (isDone && _seqCurves.Length() != 0)
                {
                    isDone = CheckContinuity(_seqCurves.Last(), curve!, _angleTolerance);
                }

                if (isDone || distance < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    // OCCT appends the curve even when makeLine returned a null handle (aDistance below
                    // Confusion); the port's sequence holds non-null curves, so a null one is left out.
                    // OCCT GetResult() can then contain null curves (occt-issues.md).
                    if (curve is not null)
                    {
                        _seqCurves.Append(curve);
                    }

                    return isDone;
                }
            }

            // calculate biarc
            isDone = CalculateBiArcs(ref firstParam, ref lastParam);

            // if biarc was not calculated calculation is repeated on half the interval.
            if (!isDone)
            {
                PPoint2D paraM = new(firstParam.Parameter()
                                                    + (lastParam.Parameter() - firstParam.Parameter()) * 0.55,
                                                  _curve);
                isDone = MakeApproximation(ref firstParam, ref paraM);
                if (isDone)
                {
                    isDone = MakeApproximation(ref paraM, ref lastParam);
                }
            }
        }

        return isDone;
    }

    // Method for calculation of a biarc.
    private bool CalculateBiArcs(ref PPoint2D firstParam, ref PPoint2D lastParam)
    {
        double resolution = Constants.Resolution();

        if (firstParam.D1().SquareModulus() < resolution || lastParam.D1().SquareModulus() < resolution)
        {
            return false;
        }

        Xy pnt0 = firstParam.Point(), pnt1 = lastParam.Point();
        Direction2D dir0 = new(firstParam.D1()), dir1 = new(lastParam.D1());

        // Try to approximate the curve by a single arc. The criterion for that is
        // more rigid if the curve is the entire input curve
        // (possible pb. connecting with other boundaries)
        Vector2D delta = new(pnt1 - pnt0);
        double angle1 = delta.AngleTo(new Vector2D(dir0));
        if (angle1 < 0.0)
        {
            angle1 = -angle1;
        }

        double angle2 = delta.AngleTo(new Vector2D(dir1));
        if (angle2 < 0.0)
        {
            angle2 = -angle2;
        }

        // in the case when two angles are equal one arc can be built.
        double angleThreshold = global::Kerf.Engine.Foundation.Precision.Angular() * 10.0;
        if (firstParam.IsNotEqual(_ext[0]) || lastParam.IsNotEqual(_ext[1]))
        {
            angleThreshold = _angleTolerance * 0.1;
        }

        if (Math.Abs(angle1 - angle2) < angleThreshold)
        {
            // protect the theLastParam from modification of D1, when
            // the created arc is rejected.
            PPoint2D aLastParam = lastParam;
            if (!MakeArc(firstParam, ref aLastParam, true, out TrimmedCurve2D? curve))
            {
                return false;
            }

            if (CheckCurve(curve, firstParam.Parameter(), aLastParam.Parameter()))
            {
                lastParam = aLastParam;
                _seqCurves.Append(curve!);
                return true;
            }
        }

        // if one arc was not built or for other cases biarc will be built
        // method for building biarc was taken from article Ahmad H. Nasri et al.
        // "A Recursive Subdivision Algorithm for Piecewise Circular Spline",
        // Computer Graphics Forum, 2001.

        // definition of point of intersection two tangent directions in the points
        // corresponding FirstParameter and LastParameter.
        dir1 = dir1.Reversed();

        // Direct calculation of intersection point, replaces a class call below
        double prod0 = pnt0 ^ dir0.XY(), prod1 = pnt1 ^ dir1.XY(), prod2 = dir1 ^ new Direction2D(dir0.XY()); // C++ converts the gp_XY to a gp_Dir2d implicitly
        Xy intPoint = new((prod0 * dir1.X - prod1 * dir0.X) / prod2,
                              (prod0 * dir1.Y - prod1 * dir0.Y) / prod2);
        Xy diff0 = intPoint - pnt0, diff1 = intPoint - pnt1;
        if (diff0 * dir0.XY() < 0.0 || diff1 * dir1.XY() < 0.0)
        {
            return false;
        }

        // calculation middle point for building biarc.
        double ad1 = diff0.Modulus();
        double ad2 = diff1.Modulus();
        double ad12 = delta.Magnitude;

        double b1 = ad1 / (ad1 + ad2);
        if (Math.Abs(b1 - 0.5) < 0.0001)
        {
            return false;
        }

        Xy xy0 = pnt0 + dir0.XY() * ad12 * ad1 / (ad12 + ad1 + ad2);
        Xy xy1 = pnt1 + dir1.XY() * ad12 * ad2 / (ad12 + ad1 + ad2);

        Xy xYmidArc = xy0 + b1 * (xy1 - xy0);
        PPoint2D paramMidArc =
            GetParameter(xYmidArc, firstParam.Parameter(), lastParam.Parameter(), _curve);

        // building first arc from biarc.
        if (!MakeArc(firstParam, ref paramMidArc, true, out TrimmedCurve2D? curve1))
        {
            return false;
        }

        if (!CheckCurve(curve1, firstParam.Parameter(), paramMidArc.Parameter()))
        {
            return false;
        }

        // building second arc from biarc.
        if (MakeArc(lastParam, ref paramMidArc, false, out TrimmedCurve2D? curve2))
        {
            if (CheckCurve(curve2, paramMidArc.Parameter(), lastParam.Parameter()))
            {
                _seqCurves.Append(curve1!);
                _seqCurves.Append(curve2!);
                return true;
            }
        }

        return false;
    }

    // Method for calculation of a linear interpolation. (Unused by OCCT, ported for completeness.)
    private bool CalculateLines(ref PPoint2D firstParam, in PPoint2D lastParam)
    {
        ref PPoint2D prevParam = ref firstParam;
        for (int i = 1; i <= _seqParams.Length(); i++)
        {
            ref PPoint2D curParam = ref _seqParams.ChangeValue(i);
            if (curParam.Parameter() < prevParam.Parameter())
            {
                continue;
            }

            if (curParam.Parameter() > lastParam.Parameter())
            {
                break;
            }

            // build line segment
            if (curParam.IsNotEqual(prevParam))
            {
                double distance = (curParam.Point() - prevParam.Point()).Modulus();
                if (distance > _tolerance)
                {
                    Curve2D? curve = MakeLine(ref prevParam, ref curParam, false);
                    if (curve is null)
                    {
                        return false;
                    }

                    _seqCurves.Append(curve);
                    prevParam = ref _seqParams[i];
                }
            }
        }

        return true;
    }

    // Checking max deflection Geom curve from Adaptor Curve
    private bool CheckCurve(Curve2D? aCurve, double firstParam, double lastParam)
    {
        if (aCurve is null)
        {
            return false; // check fails on empty input
        }

        bool isUniformDone = !_seqParams.IsEmpty();
        // calculation sequence of the parameters or step by parameter.
        int nbPnts = (isUniformDone ? _seqParams.Length() : Maxpoints);
        double paramStep = (lastParam - firstParam) / Maxpoints;

        Curve2D curve1 = aCurve;
        if (aCurve is TrimmedCurve2D trCurve)
        {
            curve1 = trCurve.BasisCurve;
        }

        Line2D lin2d = new();
        Circle2D circ2d = new();
        bool isLine = curve1 is LineCurve2D;
        bool isCircle = !isLine;
        if (curve1 is LineCurve2D geomLine)
        {
            lin2d = geomLine.Line;
        }
        else
        {
            isCircle = curve1 is CircleCurve2D;
            if (curve1 is CircleCurve2D geomCircle)
            {
                circ2d = geomCircle.Circle;
            }
            else
            {
                return false;
            }
        }

        // calculation of the max deflection points from CurveAdaptor from Geom curve.
        bool isLess = true;
        int i = 1;
        for (; i <= nbPnts && isLess; i++)
        {
            double param = (isUniformDone ? _seqParams.Value(i).Parameter() : (firstParam + i * paramStep));
            if (param < (firstParam - global::Kerf.Engine.Foundation.Precision.PConfusion()) || param > (lastParam + global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                continue;
            }

            // getting point from adaptor curve by specified parameter.
            Point2D projPoint = new(0.0, 0.0);
            _curve.D0(param, out Point2D pointAdaptor);
            double parameterCurve;

            // getting point from geom curve by specified parameter.
            if (isLine)
            {
                parameterCurve = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin2d, pointAdaptor);
                projPoint = global::Kerf.Engine.Geometry.ElCLib.Value(parameterCurve, lin2d);
            }
            else if (isCircle)
            {
                parameterCurve = global::Kerf.Engine.Geometry.ElCLib.Parameter(circ2d, pointAdaptor);
                projPoint = global::Kerf.Engine.Geometry.ElCLib.Value(parameterCurve, circ2d);
            }

            isLess = (projPoint.DistanceTo(pointAdaptor) < _tolerance + global::Kerf.Engine.Foundation.Precision.PConfusion());
        }

        return isLess;
    }

    private static bool CheckContinuity(Curve2D curve1, Curve2D curve2, double angleTol)
    {
        curve1.D1(curve1.LastParameter, out _, out Vector2D v11);
        curve2.D1(curve2.FirstParameter, out _, out Vector2D v21);

        // check continuity with the specified tolerance.
        return (v11.IsParallel(v21, angleTol));
    }

    private static PPoint2D GetParameter(in Xy xy1,
                                                     double firstParam,
                                                     double lastParam,
                                                     Curve2DAdaptorBase curve)
    {
        PPoint2D result;
        double prevParam = firstParam;
        double af1 = firstParam;
        double af2 = lastParam;

        // for finding nearest point use method half division.
        double minDist = RealLast();
        int i = 1;
        for (; i <= Maxpoints; i++)
        {
            result = new PPoint2D(af1, curve);
            double adist1 = (xy1 - result.Point()).Modulus();
            if (adist1 < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return result;
            }

            result = new PPoint2D(af2, curve);
            double adist2 = (xy1 - result.Point()).Modulus();
            if (adist2 < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return result;
            }

            if (minDist <= adist2 - global::Kerf.Engine.Foundation.Precision.Confusion() && minDist <= adist1 - global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                break;
            }

            if (adist1 < adist2 - global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                prevParam = af1;
                minDist = adist1;
                af2 = (af1 + af2) * 0.5;
            }
            else
            {
                prevParam = af2;
                minDist = adist2;
                af1 = (af1 + af2) * 0.5;
            }
        }

        result = new PPoint2D(prevParam, curve);
        return result;
    }

    private static bool IsInflectionPoint(double param, Curve2DAdaptorBase curve)
    {
        curve.D2(param, out _, out Vector2D d1, out Vector2D d2);
        double sqMod = d1.XY().SquareModulus();
        double curvature = Math.Abs(d1.XY() ^ d2.XY()) / (sqMod * Math.Sqrt(sqMod));
        return (curvature < MyCurvatureTolerance);
    }

    private static bool IsInflectionPoint(double param,
                                          in PPoint2D firstInfl,
                                          Curve2DAdaptorBase curve,
                                          double angleTol)
    {
        curve.D2(param, out Point2D p1, out Vector2D d1, out Vector2D d2);
        double sqMod = d1.XY().SquareModulus();
        double curvature = Math.Abs(d1.XY() ^ d2.XY()) / (sqMod * Math.Sqrt(sqMod));
        double contAngle = Math.Abs(new Vector2D(p1.XY() - firstInfl.Point()).AngleTo(d1));
        contAngle = Min(contAngle, Math.Abs(M_PI - contAngle));
        return (curvature < MyCurvatureTolerance && contAngle < angleTol);
    }
}
