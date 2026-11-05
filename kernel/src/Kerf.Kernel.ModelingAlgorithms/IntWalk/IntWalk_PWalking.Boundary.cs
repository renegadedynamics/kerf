// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.Implicit.ConstIsoparametric;
using static Kerf.Engine.Intersection.Walking.StatusDeflection;

namespace Kerf.Engine.Intersection.Walking;

internal partial class PWalking
{
    /// <summary>
    /// Extends the line inside a tangent zone in the direction given by theChoixIso. True if the line was extended
    /// through the zone and the last point computed is outside it (that point is not put into the line).
    /// </summary>
    internal bool ExtendLineInCommonZone(ConstIsoparametric choixIso, bool directionFlag)
    {
        // Caro1 and Caro2
        SurfaceAdaptorBase caro1 = _intersectionOn2S.Function().AuxillarSurface1();
        SurfaceAdaptorBase caro2 = _intersectionOn2S.Function().AuxillarSurface2();
        //
        double uFirst1 = HSurfaceTool.FirstUParameter(caro1);
        double vFirst1 = HSurfaceTool.FirstVParameter(caro1);
        double uLast1 = HSurfaceTool.LastUParameter(caro1);
        double vLast1 = HSurfaceTool.LastVParameter(caro1);

        double uFirst2 = HSurfaceTool.FirstUParameter(caro2);
        double vFirst2 = HSurfaceTool.FirstVParameter(caro2);
        double uLast2 = HSurfaceTool.LastUParameter(caro2);
        double vLast2 = HSurfaceTool.LastVParameter(caro2);

        bool bOutOfTangentZone = false;
        bool bStop = !_intersectionOn2S.IsTangent;
        int dIncKey = 1;
        NCollection_Array1<double> param = new(1, 4);
        StatusDeflection status = StatusDeflection.OK;
        int nbIterWithoutAppend = 0;
        int nbEqualPoints = 0;
        int parit = 0;
        int uvit = 0;
        NCollection_Sequence<PntOn2S> seqOfNewPoint = new();

        SetParameters(param, _previousPoint);

        if (param[1] - uFirst1 < _resoU1)
        {
            return bOutOfTangentZone;
        }
        else if (param[2] - vFirst1 < _resoV1)
        {
            return bOutOfTangentZone;
        }
        else if (param[3] - uFirst2 < _resoU2)
        {
            return bOutOfTangentZone;
        }
        else if (param[4] - vFirst2 < _resoV2)
        {
            return bOutOfTangentZone;
        }

        if (param[1] - uLast1 > -_resoU1)
        {
            return bOutOfTangentZone;
        }
        else if (param[2] - vLast1 > -_resoV1)
        {
            return bOutOfTangentZone;
        }
        else if (param[3] - uLast2 > -_resoU2)
        {
            return bOutOfTangentZone;
        }
        else if (param[4] - vLast2 > -_resoV2)
        {
            return bOutOfTangentZone;
        }

        Span<double> svParam = stackalloc double[4];
        while (!bStop)
        {
            nbIterWithoutAppend++;

            if ((nbIterWithoutAppend > 20) || (nbEqualPoints > 20))
            {
                bStop = true;
                break;
            }
            double f = 0.0;

            switch (choixIso)
            {
                case ConstIsoparametric.UIsoparametricOnCaro1:
                    f = Math.Abs(_previousd1.X);
                    break;
                case ConstIsoparametric.VIsoparametricOnCaro1:
                    f = Math.Abs(_previousd1.Y);
                    break;
                case ConstIsoparametric.UIsoparametricOnCaro2:
                    f = Math.Abs(_previousd2.X);
                    break;
                case ConstIsoparametric.VIsoparametricOnCaro2:
                    f = Math.Abs(_previousd2.Y);
                    break;
            }

            if (f < 0.1)
            {
                f = 0.1;
            }

            SetParameters(param, _previousPoint);

            double dP1 = _sensCheminement * _pasuv[0] * _previousd1.X / f;
            double dP2 = _sensCheminement * _pasuv[1] * _previousd1.Y / f;
            double dP3 = _sensCheminement * _pasuv[2] * _previousd2.X / f;
            double dP4 = _sensCheminement * _pasuv[3] * _previousd2.Y / f;

            if (choixIso == ConstIsoparametric.UIsoparametricOnCaro1 && Math.Abs(dP1) < 1.0e-7)
            {
                dP1 *= (5.0 * (double)dIncKey);
            }
            if (choixIso == ConstIsoparametric.VIsoparametricOnCaro1 && Math.Abs(dP2) < 1.0e-7)
            {
                dP2 *= (5.0 * (double)dIncKey);
            }
            if (choixIso == ConstIsoparametric.UIsoparametricOnCaro2 && Math.Abs(dP3) < 1.0e-7)
            {
                dP3 *= (5.0 * (double)dIncKey);
            }
            if (choixIso == ConstIsoparametric.VIsoparametricOnCaro2 && Math.Abs(dP4) < 1.0e-7)
            {
                dP4 *= (5.0 * (double)dIncKey);
            }

            param[1] += dP1;
            param[2] += dP2;
            param[3] += dP3;
            param[4] += dP4;
            ConstIsoparametric ChoixIso = choixIso;

            for (parit = 0; parit < 4; parit++)
            {
                svParam[parit] = param[parit + 1];
            }
            FunctionSetRoot rsnld = new(_intersectionOn2S.Function());
            ChoixIso = _intersectionOn2S.Perform(param, rsnld, choixIso);

            if (!_intersectionOn2S.IsDone)
            {
                return bOutOfTangentZone;
            }
            else
            {
                if (_intersectionOn2S.IsEmpty)
                {
                    return bOutOfTangentZone;
                }

                status = TestDeflection(ChoixIso, status);

                if (status == StatusDeflection.OK)
                {
                    for (uvit = 0; uvit < 4; uvit++)
                    {
                        if (_pasuv[uvit] < _pasInit[uvit])
                        {
                            _pasuv[uvit] = _pasInit[uvit];
                        }
                    }
                }

                switch (status)
                {
                    case StatusDeflection.ArretSurPointPrecedent:
                    {
                        bStop = true;
                        bOutOfTangentZone = !_intersectionOn2S.IsTangent;
                        break;
                    }
                    case StatusDeflection.PasTropGrand:
                    {
                        for (parit = 0; parit < 4; parit++)
                        {
                            param[parit + 1] = svParam[parit];
                        }
                        bool bDecrease = false;

                        for (uvit = 0; uvit < 4; uvit++)
                        {
                            if (_pasSav[uvit] < _pasInit[uvit])
                            {
                                _pasInit[uvit] -= (_pasInit[uvit] - _pasSav[uvit]) * 0.1;
                                bDecrease = true;
                            }
                        }

                        if (bDecrease)
                        {
                            nbIterWithoutAppend--;
                        }
                        break;
                    }
                    case StatusDeflection.PointConfondu:
                    {
                        for (uvit = 0; uvit < 4; uvit++)
                        {
                            if (_pasuv[uvit] < _pasInit[uvit])
                            {
                                _pasuv[uvit] += (_pasInit[uvit] - _pasuv[uvit]) * 0.1;
                            }
                        }
                        break;
                    }
                    case StatusDeflection.OK:
                    case StatusDeflection.ArretSurPoint:
                    {
                        //
                        bStop = TestArret(directionFlag, param, ref ChoixIso);
                        //

                        //
                        if (!bStop)
                        {
                            _intersectionOn2S.Point().Parameters(out double u11, out double v11, out double u12,
                                                                  out double v12);
                            _previousPoint.Parameters(out double u21, out double v21, out double u22, out double v22);

                            if (((Math.Abs(u11 - u21) < _resoU1) && (Math.Abs(v11 - v21) < _resoV1))
                                || ((Math.Abs(u12 - u22) < _resoU2) && (Math.Abs(v12 - v22) < _resoV2)))
                            {
                                nbEqualPoints++;
                            }
                            else
                            {
                                nbEqualPoints = 0;
                            }
                        }
                        //

                        bStop = bStop || !_intersectionOn2S.IsTangent;
                        bOutOfTangentZone = !_intersectionOn2S.IsTangent;

                        if (!bStop)
                        {
                            bool pointisvalid = false;
                            _intersectionOn2S.Point().Parameters(out double u1, out double v1, out double u2,
                                                                  out double v2);

                            if (u1 <= _uMax1 && u2 <= _uMax2 && v1 <= _vMax1 && v2 <= _vMax2 && u1 >= _uMin1 && u2 >= _uMin2
                                && v1 >= _vMin1 && v2 >= _vMin2)
                            {
                                pointisvalid = true;
                            }

                            if (pointisvalid)
                            {
                                _previousPoint = _intersectionOn2S.Point();
                                _previoustg = _intersectionOn2S.IsTangent;

                                if (!_previoustg)
                                {
                                    _previousd = _intersectionOn2S.Direction();
                                    _previousd1 = _intersectionOn2S.DirectionOnS1();
                                    _previousd2 = _intersectionOn2S.DirectionOnS2();
                                }
                                bool bAddPoint = true;

                                if (_line.PointCount >= 1)
                                {
                                    Point3D pf = _line.Value(1).Value();
                                    Point3D pl = _previousPoint.Value();

                                    if (pf.DistanceTo(pl) < global::Kerf.Engine.Foundation.Precision.Confusion())
                                    {
                                        dIncKey++;
                                        if (dIncKey == 5000)
                                        {
                                            return bOutOfTangentZone;
                                        }
                                        else
                                        {
                                            bAddPoint = false;
                                        }
                                    }
                                }

                                if (bAddPoint)
                                {
                                    seqOfNewPoint.Append(_previousPoint);
                                    nbIterWithoutAppend = 0;
                                }
                            }

                            if (_line.PointCount == 2)
                            {
                                for (uvit = 0; uvit < 4; uvit++)
                                {
                                    _pasSav[uvit] = _pasuv[uvit];
                                }
                            }

                            if (!pointisvalid)
                            {
                                // decrease step if out of bounds
                                // otherwise the same calculations will be
                                // repeated several times
                                if ((u1 > _uMax1) || (u1 < _uMin1))
                                {
                                    _pasuv[0] *= 0.5;
                                }

                                if ((v1 > _vMax1) || (v1 < _vMin1))
                                {
                                    _pasuv[1] *= 0.5;
                                }

                                if ((u2 > _uMax2) || (u2 < _uMin2))
                                {
                                    _pasuv[2] *= 0.5;
                                }

                                if ((v2 > _vMax2) || (v2 < _vMin2))
                                {
                                    _pasuv[3] *= 0.5;
                                }
                            }
                        } // end if(!bStop)
                        else
                        { // if(bStop)
                            if (_close && (_line.PointCount >= 1))
                            {
                                if (!bOutOfTangentZone)
                                {
                                    seqOfNewPoint.Append(_line.Value(1)); // line end
                                }
                                nbIterWithoutAppend = 0;
                            }
                            else
                            {
                                ChoixIso = _intersectionOn2S.Perform(param, rsnld, choixIso);

                                if (_intersectionOn2S.IsEmpty)
                                {
                                    bStop = true; // !myIntersectionOn2S.IsTangent();
                                    bOutOfTangentZone = false; // !myIntersectionOn2S.IsTangent();
                                }
                                else
                                {
                                    bool bAddPoint = true;
                                    bool pointisvalid = false;

                                    _previousPoint = _intersectionOn2S.Point();
                                    _previousPoint.Parameters(out double u1, out double v1, out double u2,
                                                             out double v2);

                                    if (u1 <= _uMax1 && u2 <= _uMax2 && v1 <= _vMax1 && v2 <= _vMax2 && u1 >= _uMin1 && u2 >= _uMin2
                                        && v1 >= _vMin1 && v2 >= _vMin2)
                                    {
                                        pointisvalid = true;
                                    }

                                    if (pointisvalid)
                                    {
                                        if (_line.PointCount >= 1)
                                        {
                                            Point3D pf = _line.Value(1).Value();
                                            Point3D pl = _previousPoint.Value();

                                            if (pf.DistanceTo(pl) < global::Kerf.Engine.Foundation.Precision.Confusion())
                                            {
                                                dIncKey++;
                                                if (dIncKey == 5000)
                                                {
                                                    return bOutOfTangentZone;
                                                }
                                                else
                                                {
                                                    bAddPoint = false;
                                                }
                                            }
                                        }

                                        if (bAddPoint && !bOutOfTangentZone)
                                        {
                                            seqOfNewPoint.Append(_previousPoint);
                                            nbIterWithoutAppend = 0;
                                        }
                                    }
                                }
                            }
                        }
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
        }
        bool bExtendLine = false;
        double eu1 = 0.0, ev1 = 0.0, eu2 = 0.0, ev2 = 0.0;

        int pit = 0;

        for (pit = 0; !bExtendLine && (pit < 2); pit++)
        {
            if (pit == 0)
            {
                _previousPoint.Parameters(out eu1, out ev1, out eu2, out ev2);
            }
            else
            {
                if (seqOfNewPoint.Length() > 0)
                {
                    seqOfNewPoint.Value(seqOfNewPoint.Length()).Parameters(out eu1, out ev1, out eu2, out ev2);
                }
                else
                {
                    break;
                }
            }

            if (((eu1 - _uMin1) < _resoU1) || ((_uMax1 - eu1) < _resoU1) || ((eu2 - _uMin2) < _resoU2)
                || ((_uMax2 - eu2) < _resoU2) || ((ev1 - _vMin1) < _resoV1) || ((_vMax1 - ev1) < _resoV1)
                || ((ev2 - _vMin2) < _resoV2) || ((_vMax2 - ev2) < _resoV2))
            {
                bExtendLine = true;
            }
        }

        if (!bExtendLine)
        {
            //    if(aStatus == IntWalk_OK || aStatus == IntWalk_ArretSurPoint) {
            if (status == StatusDeflection.OK)
            {
                bExtendLine = true;

                if (seqOfNewPoint.Length() > 1)
                {
                    NCollection_Array1<double> firstParams = new(0, 3), lastParams = new(0, 3), resolutions = new(0, 3);
                    resolutions[0] = _resoU1;
                    resolutions[1] = _resoV1;
                    resolutions[2] = _resoU2;
                    resolutions[3] = _resoV2;

                    seqOfNewPoint.Value(1).Parameters(out firstParams[0], out firstParams[1], out firstParams[2],
                                                       out firstParams[3]);
                    seqOfNewPoint.Value(seqOfNewPoint.Length()).Parameters(out lastParams[0], out lastParams[1],
                                                                             out lastParams[2], out lastParams[3]);
                    int indexofiso = 0;

                    if (choixIso == ConstIsoparametric.UIsoparametricOnCaro1)
                    {
                        indexofiso = 0;
                    }
                    if (choixIso == ConstIsoparametric.VIsoparametricOnCaro1)
                    {
                        indexofiso = 1;
                    }
                    if (choixIso == ConstIsoparametric.UIsoparametricOnCaro2)
                    {
                        indexofiso = 2;
                    }
                    if (choixIso == ConstIsoparametric.VIsoparametricOnCaro2)
                    {
                        indexofiso = 3;
                    }

                    int afirstindex = (indexofiso < 2) ? 0 : 2;
                    Vector2D tangentZoneDir = new(
                        new Point2D(firstParams.Value(afirstindex), firstParams.Value(afirstindex + 1)),
                        new Point2D(lastParams.Value(afirstindex), lastParams.Value(afirstindex + 1)));

                    Direction2D isoDir = new(Direction2D.D.Y);

                    if ((indexofiso == 1) || (indexofiso == 3))
                    {
                        isoDir = new Direction2D(Direction2D.D.X);
                    }

                    if (tangentZoneDir.SquareMagnitude > Constants.Resolution())
                    {
                        double piquota = M_PI * 0.25;

                        if (Math.Abs(tangentZoneDir.AngleTo(new Vector2D(isoDir))) > piquota)
                        {
                            int ii = 1, nextii = 2;
                            Vector2D d1 = new(0, 0);
                            double asqresol = Constants.Resolution();
                            asqresol *= asqresol;

                            do
                            {
                                seqOfNewPoint.Value(ii).Parameters(out firstParams[0], out firstParams[1],
                                                                    out firstParams[2], out firstParams[3]);
                                seqOfNewPoint.Value(ii + 1).Parameters(out lastParams[0], out lastParams[1],
                                                                        out lastParams[2], out lastParams[3]);
                                d1 = new Vector2D(
                                    new Point2D(firstParams.Value(afirstindex), firstParams.Value(afirstindex + 1)),
                                    new Point2D(lastParams.Value(afirstindex), lastParams.Value(afirstindex + 1)));
                                ii++;
                            } while ((d1.SquareMagnitude < asqresol) && (ii < seqOfNewPoint.Length()));

                            nextii = ii;

                            while (nextii < seqOfNewPoint.Length())
                            {
                                Vector2D nextd1 = new(0, 0);
                                int jj = nextii;

                                do
                                {
                                    seqOfNewPoint.Value(jj).Parameters(out firstParams[0], out firstParams[1],
                                                                        out firstParams[2], out firstParams[3]);
                                    seqOfNewPoint.Value(jj + 1).Parameters(out lastParams[0], out lastParams[1],
                                                                            out lastParams[2], out lastParams[3]);
                                    nextd1 = new Vector2D(
                                        new Point2D(firstParams.Value(afirstindex), firstParams.Value(afirstindex + 1)),
                                        new Point2D(lastParams.Value(afirstindex), lastParams.Value(afirstindex + 1)));
                                    jj++;
                                } while ((nextd1.SquareMagnitude < asqresol) && (jj < seqOfNewPoint.Length()));
                                nextii = jj;

                                if (Math.Abs(d1.AngleTo(nextd1)) > piquota)
                                {
                                    bExtendLine = false;
                                    break;
                                }
                                d1 = nextd1;
                            }
                        }
                        // end if(fabs(aTangentZoneDir.Angle(anIsoDir)
                    }
                }
            }
        }

        if (!bExtendLine)
        {
            return false;
        }
        int i = 0;

        for (i = 1; i <= seqOfNewPoint.Length(); i++)
        {
            AddAPoint(seqOfNewPoint.Value(i));
        }

        return bOutOfTangentZone;
    }

    /// <summary>
    /// Moves theInit (u1, v1, u2, v2, initialised by the caller) down the gradient of the square distance between
    /// the surfaces' points (B-spline and Bézier surfaces only; true at once for others). True if the distance falls
    /// below 1e-7.
    /// </summary>
    internal bool DistanceMinimizeByGradient(SurfaceAdaptorBase aSurf1, SurfaceAdaptorBase aSurf2,
                                             NCollection_Array1<double> init, double[]? step0 = null)
    {
        const int nbIterMAX = 60;
        const double tol = 1.0e-14;
        double tolNul = 1.0 / global::Kerf.Engine.Foundation.Precision.Infinite();

        // I.e. if theU1 = 0.0 then Epsilon(theU1) = DBL_MIN (~1.0e-308).
        // Work with this number is impossible: there is a dangerous to
        // obtain Floating-point-overflow. Therefore, we limit this value.
        double minAddValU1 = Math.Max(Epsilon(init[1]), tolNul);
        double minAddValV1 = Math.Max(Epsilon(init[2]), tolNul);
        double minAddValU2 = Math.Max(Epsilon(init[3]), tolNul);
        double minAddValV2 = Math.Max(Epsilon(init[4]), tolNul);

        if (aSurf1.GetType() != SurfaceKind.BezierSurface && aSurf1.GetType() != SurfaceKind.BSplineSurface)
        {
            return true;
        }
        if (aSurf2.GetType() != SurfaceKind.BezierSurface && aSurf2.GetType() != SurfaceKind.BSplineSurface)
        {
            return true;
        }

        bool status = false;

        aSurf1.D1(init[1], init[2], out Point3D p1, out Vector3D d1u, out Vector3D d1v);
        aSurf2.D1(init[3], init[4], out Point3D p2, out Vector3D d2U, out Vector3D d2V);

        double sqDistPrev = p1.SquareDistanceTo(p2);

        Vector3D p12 = new(p1, p2);

        double gradFu = -p12.Dot(d1u);
        double gradFv = -p12.Dot(d1v);
        double gradFU = p12.Dot(d2U);
        double gradFV = p12.Dot(d2V);

        double stepU1 = 1.0e-6, stepV1 = 1.0e-6, stepU2 = 1.0e-6, stepV2 = 1.0e-6;

        if (step0 != null)
        {
            stepU1 = step0[0];
            stepV1 = step0[1];
            stepU2 = step0[2];
            stepV2 = step0[3];
        }

        bool flRepeat = true;
        int nbIter = nbIterMAX;

        while (flRepeat)
        {
            double add = gradFu * stepU1;
            double paRu = init[1] - Math.CopySign(Math.Max(Math.Abs(add), minAddValU1), add);

            add = gradFv * stepV1;
            double paRv = init[2] - Math.CopySign(Math.Max(Math.Abs(add), minAddValV1), add);

            add = gradFU * stepU2;
            double parU = init[3] - Math.CopySign(Math.Max(Math.Abs(add), minAddValU2), add);

            add = gradFV * stepV2;
            double parV = init[4] - Math.CopySign(Math.Max(Math.Abs(add), minAddValV2), add);

            aSurf1.D1(paRu, paRv, out Point3D pt1, out d1u, out d1v);
            aSurf2.D1(parU, parV, out Point3D pt2, out d2U, out d2V);

            double sqDist = pt1.SquareDistanceTo(pt2);

            if (sqDist < sqDistPrev)
            {
                sqDistPrev = sqDist;
                init[1] = paRu;
                init[2] = paRv;
                init[3] = parU;
                init[4] = parV;

                status = sqDistPrev < tol;
                stepU1 *= 1.2;
                stepV1 *= 1.2;
                stepU2 *= 1.2;
                stepV2 *= 1.2;
            }
            else
            {
                if (--nbIter < 0)
                {
                    flRepeat = false;
                }
                else
                {
                    aSurf1.D1(init[1], init[2], out pt1, out d1u, out d1v);
                    aSurf2.D1(init[3], init[4], out pt2, out d2U, out d2V);

                    Vector3D pt12 = new(pt1, pt2);
                    gradFu = -pt12.Dot(d1u);
                    gradFv = -pt12.Dot(d1v);
                    gradFU = pt12.Dot(d2U);
                    gradFV = pt12.Dot(d2V);

                    if (step0 != null)
                    {
                        stepU1 = step0[0];
                        stepV1 = step0[1];
                        stepU2 = step0[2];
                        stepV2 = step0[3];
                    }
                    else
                    {
                        stepU1 = stepV1 = stepU2 = stepV2 = 1.0e-6;
                    }
                }
            }
        }

        return status;
    }

    /// <summary>
    /// Newton's iterations for the projection of theP0 on theASurf from (theU0, theV0) (initialised by the caller).
    /// True if the square distance falls below 1e-14.
    /// </summary>
    internal bool DistanceMinimizeByExtrema(SurfaceAdaptorBase aSurf, in Point3D p0, ref double u0,
                                            ref double v0, double[]? step0 = null)
    {
        const double tol = 1.0e-14;
        double sqDistPrev = RealLast();
        double u = u0, v = v0;

        ReadOnlySpan<double> aStep0 = [step0 != null ? step0[0] : 1.0, step0 != null ? step0[1] : 1.0];

        int nbIter = 10;
        do
        {
            aSurf.D2(u, v, out Point3D ps, out Vector3D d1Su, out Vector3D d1Sv, out Vector3D d2Su, out Vector3D d2Sv,
                        out Vector3D _);

            Vector3D vec = new(p0, ps);

            double sqDist = vec.SquareMagnitude;

            if (sqDist >= sqDistPrev)
            {
                break;
            }

            sqDistPrev = sqDist;
            u0 = u;
            v0 = v;
            nbIter--;

            if (sqDistPrev < tol)
            {
                break;
            }

            // Functions
            double f1 = d1Su.Dot(vec), f2 = d1Sv.Dot(vec);

            // Derivatives
            double df1u = d2Su.Dot(vec) + d1Su.Dot(d1Su), df1v = d2Su.Dot(d1Sv), df2u = df1v,
                   df2v = d2Sv.Dot(vec) + d1Sv.Dot(d1Sv);

            double det = df1u * df2v - df1v * df2u;
            u -= aStep0[0] * (df2v * f1 - df1v * f2) / det;
            v += aStep0[1] * (df2u * f1 - df1u * f2) / det;
        } while (nbIter > 0);

        return (sqDistPrev < tol);
    }

    /// <summary>
    /// Snaps thePnt (u1, v1, u2, v2) on a boundary: an intersection point with a parameter locked on the boundary it
    /// is near, then its other parameters onto boundaries within the3DTol. True if the point found is in the domain.
    /// </summary>
    internal bool HandleSingleSingularPoint(SurfaceAdaptorBase aSurf1, SurfaceAdaptorBase aSurf2,
                                            double the3DTol, NCollection_Array1<double> pnt)
    {
        // u1, v1, u2, v2 order is used.
        ReadOnlySpan<double> lowBorder = [aSurf1.FirstUParameter(), aSurf1.FirstVParameter(),
                                           aSurf2.FirstUParameter(), aSurf2.FirstVParameter()];
        ReadOnlySpan<double> uppBorder = [aSurf1.LastUParameter(), aSurf1.LastVParameter(),
                                           aSurf2.LastUParameter(), aSurf2.LastVParameter()];
        ReadOnlySpan<ConstIsoparametric> lockedDir = [ConstIsoparametric.UIsoparametricOnCaro1,
                                                              ConstIsoparametric.VIsoparametricOnCaro1,
                                                              ConstIsoparametric.UIsoparametricOnCaro2,
                                                              ConstIsoparametric.VIsoparametricOnCaro2];

        // Create new intersector with new tolerance.
        TheInt2S anInt = new(aSurf1, aSurf2, the3DTol);
        FunctionSetRoot rsnld = new(anInt.Function());

        Span<double> pars = stackalloc double[4];
        for (int i = 1; i <= 4; ++i)
        {
            if (Math.Abs(pnt[i] - lowBorder[i - 1]) < global::Kerf.Engine.Foundation.Precision.PConfusion()
                || Math.Abs(pnt[i] - uppBorder[i - 1]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                anInt.Perform(pnt, rsnld, lockedDir[i - 1]);

                if (!anInt.IsDone)
                {
                    continue;
                }

                if (anInt.IsEmpty)
                {
                    continue;
                }

                anInt.Point().Parameters(out pars[0], out pars[1], out pars[2], out pars[3]);
                SurfaceAdaptorBase[] surfs = [aSurf1, aSurf2];
                // Local resolutions
                double tol2 = the3DTol * the3DTol;
                Point3D p;
                int k;
                for (k = 0; k < 2; ++k)
                {
                    int iu, iv;
                    iu = 2 * k;
                    iv = iu + 1;
                    surfs[k].D1(pars[iu], pars[iv], out Point3D pInt, out Vector3D du, out Vector3D dv);
                    double mod = du.Magnitude;
                    if (mod > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        double tolU = the3DTol / mod;
                        if (Math.Abs(lowBorder[iu] - pars[iu]) < tolU)
                        {
                            p = surfs[k].Value(lowBorder[iu], pars[iv]);
                            if (pInt.SquareDistanceTo(p) < tol2)
                            {
                                pars[iu] = lowBorder[iu];
                            }
                        }
                        else if (Math.Abs(uppBorder[iu] - pars[iu]) < tolU)
                        {
                            p = surfs[k].Value(uppBorder[iu], pars[iv]);
                            if (pInt.SquareDistanceTo(p) < tol2)
                            {
                                pars[iu] = uppBorder[iu];
                            }
                        }
                    }
                    mod = dv.Magnitude;
                    if (mod > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        double tolV = the3DTol / mod;
                        if (Math.Abs(lowBorder[iv] - pars[iv]) < tolV)
                        {
                            p = surfs[k].Value(pars[iu], lowBorder[iv]);
                            if (pInt.SquareDistanceTo(p) < tol2)
                            {
                                pars[iv] = lowBorder[iv];
                            }
                        }
                        else if (Math.Abs(uppBorder[iv] - pars[iv]) < tolV)
                        {
                            p = surfs[k].Value(pars[iu], uppBorder[iv]);
                            if (pInt.SquareDistanceTo(p) < tol2)
                            {
                                pars[iv] = uppBorder[iv];
                            }
                        }
                    }
                }
                //
                //
                int j;
                for (j = 1; j <= 4; ++j)
                {
                    pnt[j] = pars[j - 1];
                }
                bool isInDomain = true;
                for (j = 1; isInDomain && (j <= 4); ++j)
                {
                    if ((pnt[j] - lowBorder[j - 1] + global::Kerf.Engine.Foundation.Precision.PConfusion())
                            * (pnt[j] - uppBorder[j - 1] - global::Kerf.Engine.Foundation.Precision.PConfusion())
                        > 0.0)
                    {
                        isInDomain = false;
                    }
                }

                if (isInDomain)
                {
                    return true;
                }
            }
        }

        return false;
    }

    /// <summary>
    /// Searches the intersection point on the boundary near (theU1, theV1, theU2, theV2) and puts it at the start
    /// (isTheFirst) or the end of the line, removing the points that would make a hairpin bend. True if added.
    /// </summary>
    internal bool SeekPointOnBoundary(SurfaceAdaptorBase aSurf1, SurfaceAdaptorBase aSurf2, double u1,
                                      double v1, double u2, double v2, bool isTheFirst)
    {
        bool isOK = false;

        // u1, v1, u2, v2 order is used.
        double[] lowBorder = [aSurf1.FirstUParameter(), aSurf1.FirstVParameter(),
                               aSurf2.FirstUParameter(), aSurf2.FirstVParameter()];
        double[] uppBorder = [aSurf1.LastUParameter(), aSurf1.LastVParameter(),
                               aSurf2.LastUParameter(), aSurf2.LastVParameter()];

        // Tune solution tolerance according with object size.
        double res1 = Math.Max(global::Kerf.Engine.Foundation.Precision.PConfusion() / aSurf1.UResolution(1.0),
                                global::Kerf.Engine.Foundation.Precision.PConfusion() / aSurf1.VResolution(1.0));
        double res2 = Math.Max(global::Kerf.Engine.Foundation.Precision.PConfusion() / aSurf2.UResolution(1.0),
                                global::Kerf.Engine.Foundation.Precision.PConfusion() / aSurf2.VResolution(1.0));
        double a3DTol = Math.Max(res1, res2);
        double tol = Math.Max(global::Kerf.Engine.Foundation.Precision.Confusion(), a3DTol);

        // u1, v1, u2, v2 order is used.
        NCollection_Array1<double> pnt = new(1, 4);
        pnt[1] = u1;
        pnt[2] = v1;
        pnt[3] = u2;
        pnt[4] = v2;
        NCollection_Array1<double> singularPnt = new(pnt);

        int nbIter = 20;
        bool status = false;
        do
        {
            nbIter--;
            status = DistanceMinimizeByGradient(aSurf1, aSurf2, pnt);
            if (status && !AdjustToDomain(4, pnt, 1, lowBorder, uppBorder, 0))
            {
                break;
            }

            status = DistanceMinimizeByExtrema(aSurf1, aSurf2.Value(pnt[3], pnt[4]), ref pnt[1], ref pnt[2]);
            if (status && !AdjustToDomain(2, pnt, 1, lowBorder, uppBorder, 0))
            {
                break;
            }

            status = DistanceMinimizeByExtrema(aSurf2, aSurf1.Value(pnt[1], pnt[2]), ref pnt[3], ref pnt[4]);
            if (status && !AdjustToDomain(2, pnt, 3, lowBorder, uppBorder, 2))
            {
                break;
            }
        } while (!status && (nbIter > 0));

        // Handle singular points.
        bool singularStatus = HandleSingleSingularPoint(aSurf1, aSurf2, tol, singularPnt);
        if (singularStatus)
        {
            pnt.Assign(singularPnt);
        }

        if (!status && !singularStatus)
        {
            return isOK;
        }

        Point3D p1 = aSurf1.Value(pnt[1], pnt[2]);
        Point3D p2 = aSurf2.Value(pnt[3], pnt[4]);
        Point3D pInt = new(0.5 * (p1.XYZ() + p2.XYZ()));

        double sqDist = pInt.SquareDistanceTo(p1);
        if (sqDist > tol * tol)
        {
            return isOK;
        }

        // Found point is true intersection point
        PntOn2S ip = new();
        ip.SetValue(pInt, pnt[1], pnt[2], pnt[3], pnt[4]);

        // The main idea of checks below is to define if insertion of
        // addition point (on the boundary) does not lead to invalid
        // intersection curve (e.g. having a loop).
        //
        // Loops are detected with rotation angle of the Walking-line (WL).
        // If there is hairpin bend then insertion is forbidden.

        // There are at least two possible problems:
        //   1. There are some cases when two neighbor points of the WL
        //       are almost coincident (the distance between them is less
        //       than Precision::Confusion). It is impossible to define
        //       rotation angle in these cases. Therefore, points with
        //       "good" distances should be selected.

        //  2. Intersection point on the surface boundary has highest
        //      priority in compare with other "middle" points. Therefore,
        //      if insertion of new point will result in a bend then some
        //      "middle" points should be deleted in order to provide
        //      correct insertion.

        // Problem test cases:
        //   test bugs modalg_5 bug24585_1
        //   test boolean bcut_complex G7
        //   test bugs moddata_2 bug469

        if (isTheFirst)
        {
            while (_line.PointCount > 1)
            {
                int nbPnts = _line.PointCount;

                int pInd = 1;
                for (; pInd <= nbPnts; pInd++)
                {
                    p1 = p1.WithXYZ(_line.Value(pInd).Value().XYZ());
                    if (p1.SquareDistanceTo(pInt) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        break;
                    }
                }

                for (++pInd; pInd <= nbPnts; pInd++)
                {
                    p2 = p2.WithXYZ(_line.Value(pInd).Value().XYZ());
                    if (p1.SquareDistanceTo(p2) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        break;
                    }
                }

                if (pInd > nbPnts)
                {
                    return isOK;
                }

                Xyz dir01 = p1.XYZ() - pInt.XYZ();
                Xyz dir12 = p2.XYZ() - p1.XYZ();

                if (dir01.Dot(dir12) > 0.0)
                {
                    break;
                }

                RemoveAPoint(1);
            }

            p1 = p1.WithXYZ(_line.Value(1).Value().XYZ());
            if (p1.SquareDistanceTo(pInt) <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                RemoveAPoint(1);
            }

            _line.InsertBefore(1, ip);
            isOK = true;
        }
        else
        {
            while (_line.PointCount > 1)
            {
                int nbPnts = _line.PointCount;

                Point3D pPrev = new(), pCurr = new();
                int pInd = nbPnts;
                for (; pInd > 0; pInd--)
                {
                    pCurr = pCurr.WithXYZ(_line.Value(pInd).Value().XYZ());
                    if (pCurr.SquareDistanceTo(pInt) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        break;
                    }
                }

                for (--pInd; pInd > 0; pInd--)
                {
                    pPrev = pPrev.WithXYZ(_line.Value(pInd).Value().XYZ());
                    if (pCurr.SquareDistanceTo(pPrev) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        break;
                    }
                }

                if (pInd < 1)
                {
                    return isOK;
                }

                Xyz dirPC = pCurr.XYZ() - pPrev.XYZ();
                Xyz dirCN = pInt.XYZ() - pCurr.XYZ();

                if (dirPC.Dot(dirCN) > 0.0)
                {
                    break;
                }

                RemoveAPoint(nbPnts);
            }

            int nbPntsL = _line.PointCount;
            p1 = p1.WithXYZ(_line.Value(nbPntsL).Value().XYZ());
            if (p1.SquareDistanceTo(pInt) <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                RemoveAPoint(nbPntsL);
            }
            _line.Add(ip);

            isOK = true;
        }

        return isOK;
    }

    /// <summary>
    /// Puts the ends of the line on the surfaces' boundaries when they are near them (closer than 1e-3 of the
    /// smallest range). True if a point was added.
    /// </summary>
    public bool PutToBoundary(SurfaceAdaptorBase aSurf1, SurfaceAdaptorBase aSurf2)
    {
        double tolMin = global::Kerf.Engine.Foundation.Precision.Confusion();

        bool hasBeenAdded = false;

        double u1bFirst = aSurf1.FirstUParameter();
        double u1bLast = aSurf1.LastUParameter();
        double u2bFirst = aSurf2.FirstUParameter();
        double u2bLast = aSurf2.LastUParameter();
        double v1bFirst = aSurf1.FirstVParameter();
        double v1bLast = aSurf1.LastVParameter();
        double v2bFirst = aSurf2.FirstVParameter();
        double v2bLast = aSurf2.LastVParameter();

        double tol = 1.0;
        tol = Math.Min(tol, u1bLast - u1bFirst);
        tol = Math.Min(tol, u2bLast - u2bFirst);
        tol = Math.Min(tol, v1bLast - v1bFirst);
        tol = Math.Min(tol, v2bLast - v2bFirst) * 1.0e-3;

        if (tol <= 2.0 * tolMin)
        {
            return hasBeenAdded;
        }

        bool isNeedAdding = false;
        IsParallel(_line, true, tol, out bool isU1parallel, out bool isV1parallel);
        IsParallel(_line, false, tol, out bool isU2parallel, out bool isV2parallel);

        _line.Value(1).Parameters(out double u1, out double v1, out double u2, out double v2);
        double delta = 0.0;

        if (!isV1parallel)
        {
            delta = u1 - u1bFirst;
            if ((tolMin < delta) && (delta < tol))
            {
                u1 = u1bFirst;
                isNeedAdding = true;
            }
            else
            {
                delta = u1bLast - u1;
                if ((tolMin < delta) && (delta < tol))
                {
                    u1 = u1bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (!isV2parallel)
        {
            delta = u2 - u2bFirst;
            if ((tolMin < delta) && (delta < tol))
            {
                u2 = u2bFirst;
                isNeedAdding = true;
            }
            else
            {
                delta = u2bLast - u2;
                if ((tolMin < delta) && (delta < tol))
                {
                    u2 = u2bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (!isU1parallel)
        {
            delta = v1 - v1bFirst;
            if ((tolMin < delta) && (delta < tol))
            {
                v1 = v1bFirst;
                isNeedAdding = true;
            }
            else
            {
                delta = v1bLast - v1;
                if ((tolMin < delta) && (delta < tol))
                {
                    v1 = v1bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (!isU2parallel)
        {
            delta = v2 - v2bFirst;
            if ((tolMin < delta) && (delta < tol))
            {
                v2 = v2bFirst;
                isNeedAdding = true;
            }
            else
            {
                delta = v2bLast - v2;
                if ((tolMin < delta) && (delta < tol))
                {
                    v2 = v2bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (isNeedAdding)
        {
            hasBeenAdded = SeekPointOnBoundary(aSurf1, aSurf2, u1, v1, u2, v2, true);
        }

        int nbPnts = _line.PointCount;
        isNeedAdding = false;
        _line.Value(nbPnts).Parameters(out u1, out v1, out u2, out v2);

        if (!isV1parallel)
        {
            delta = u1 - u1bFirst;
            if ((tolMin < delta) && (delta < tol))
            {
                u1 = u1bFirst;
                isNeedAdding = true;
            }
            else
            {
                delta = u1bLast - u1;
                if ((tolMin < delta) && (delta < tol))
                {
                    u1 = u1bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (!isV2parallel)
        {
            delta = u2 - u2bFirst;
            if ((tolMin < delta) && (delta < tol))
            {
                u2 = u2bFirst;
                isNeedAdding = true;
            }
            else
            {
                delta = u2bLast - u2;
                if ((tolMin < delta) && (delta < tol))
                {
                    u2 = u2bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (!isU1parallel)
        {
            delta = v1 - v1bFirst;
            if ((tolMin < delta) && (delta < tol))
            {
                v1 = v1bFirst;
                isNeedAdding = true;
            }
            else
            {
                delta = v1bLast - v1;
                if ((tolMin < delta) && (delta < tol))
                {
                    v1 = v1bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (!isU2parallel)
        {
            delta = v2 - v2bFirst;
            if ((tolMin < delta) && (delta < tol))
            {
                v2 = v2bFirst;
                isNeedAdding = true;
            }
            else
            {
                delta = v2bLast - v2;
                if ((tolMin < delta) && (delta < tol))
                {
                    v2 = v2bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (isNeedAdding)
        {
            hasBeenAdded = SeekPointOnBoundary(aSurf1, aSurf2, u1, v1, u2, v2, false);
        }

        return hasBeenAdded;
    }

    /// <summary>
    /// Inserts exact intersection points in the middle of the segments of the line until it has theMinNbPoints
    /// points (or no more can be found). True if a point was inserted.
    /// </summary>
    public bool SeekAdditionalPoints(SurfaceAdaptorBase aSurf1, SurfaceAdaptorBase aSurf2, int minNbPoints)
    {
        const double tol = 1.0e-14;
        int nbPoints = _line.PointCount;
        if (nbPoints > minNbPoints)
        {
            return true;
        }

        double u1bFirst = aSurf1.FirstUParameter();
        double u1bLast = aSurf1.LastUParameter();
        double u2bFirst = aSurf2.FirstUParameter();
        double u2bLast = aSurf2.LastUParameter();
        double v1bFirst = aSurf1.FirstVParameter();
        double v1bLast = aSurf1.LastVParameter();
        double v2bFirst = aSurf2.FirstVParameter();
        double v2bLast = aSurf2.LastVParameter();

        bool isPrecise = false;

        NCollection_Array1<double> pnt = new(1, 4);
        pnt.Init(0.0);

        int nbPointsPrev = 0;
        while (nbPoints < minNbPoints && (nbPoints != nbPointsPrev))
        {
            nbPointsPrev = nbPoints;
            for (int fp = 1, lp = 2; fp < nbPoints; fp = lp + 1)
            {
                // first and last points in 1st and 2nd surafaces
                lp = fp + 1;
                _line.Value(fp).Parameters(out double u1f, out double v1f, out double u2f, out double v2f);
                _line.Value(lp).Parameters(out double u1l, out double v1l, out double u2l, out double v2l);

                pnt[1] = 0.5 * (u1f + u1l);
                if (pnt[1] < u1bFirst)
                {
                    pnt[1] = u1bFirst;
                }
                if (pnt[1] > u1bLast)
                {
                    pnt[1] = u1bLast;
                }

                pnt[2] = 0.5 * (v1f + v1l);
                if (pnt[2] < v1bFirst)
                {
                    pnt[2] = v1bFirst;
                }
                if (pnt[2] > v1bLast)
                {
                    pnt[2] = v1bLast;
                }

                pnt[3] = 0.5 * (u2f + u2l);
                if (pnt[3] < u2bFirst)
                {
                    pnt[3] = u2bFirst;
                }
                if (pnt[3] > u2bLast)
                {
                    pnt[3] = u2bLast;
                }

                pnt[4] = 0.5 * (v2f + v2l);
                if (pnt[4] < v2bFirst)
                {
                    pnt[4] = v2bFirst;
                }
                if (pnt[4] > v2bLast)
                {
                    pnt[4] = v2bLast;
                }

                bool status = false;
                int nbIter = 5;
                do
                {
                    status = DistanceMinimizeByGradient(aSurf1, aSurf2, pnt);
                    if (status)
                    {
                        break;
                    }

                    status = DistanceMinimizeByExtrema(aSurf1, aSurf2.Value(pnt[3], pnt[4]), ref pnt[1],
                                                        ref pnt[2]);
                    if (status)
                    {
                        break;
                    }

                    status = DistanceMinimizeByExtrema(aSurf2, aSurf1.Value(pnt[1], pnt[2]), ref pnt[3],
                                                        ref pnt[4]);
                    if (status)
                    {
                        break;
                    }
                } while (!status && (--nbIter > 0));

                if (status)
                {
                    Point3D p1 = aSurf1.Value(pnt[1], pnt[2]), p2 = aSurf2.Value(pnt[3], pnt[4]);
                    Point3D pInt = new(0.5 * (p1.XYZ() + p2.XYZ()));

                    double sqDist1 = pInt.SquareDistanceTo(p1), sqDist2 = pInt.SquareDistanceTo(p2);

                    if ((sqDist1 < tol) && (sqDist2 < tol))
                    {
                        PntOn2S ip = new();
                        ip.SetValue(pInt, pnt[1], pnt[2], pnt[3], pnt[4]);
                        _line.InsertBefore(lp, ip);

                        isPrecise = true;

                        if (++nbPoints >= minNbPoints)
                        {
                            break;
                        }
                    }
                    else
                    {
                        lp--;
                    }
                }
            }
        }

        return isPrecise;
    }
}
