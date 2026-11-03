// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Occt.IntImp_ConstIsoparametric;
using static Occt.IntWalk_StatusDeflection;

namespace Occt;

public partial class IntWalk_PWalking
{
    /// <summary>
    /// Extends the line inside a tangent zone in the direction given by theChoixIso. True if the line was extended
    /// through the zone and the last point computed is outside it (that point is not put into the line).
    /// </summary>
    internal bool ExtendLineInCommonZone(IntImp_ConstIsoparametric theChoixIso, bool theDirectionFlag)
    {
        // Caro1 and Caro2
        SurfaceAdaptorBase Caro1 = myIntersectionOn2S.Function().AuxillarSurface1();
        SurfaceAdaptorBase Caro2 = myIntersectionOn2S.Function().AuxillarSurface2();
        //
        double UFirst1 = HSurfaceTool.FirstUParameter(Caro1);
        double VFirst1 = HSurfaceTool.FirstVParameter(Caro1);
        double ULast1 = HSurfaceTool.LastUParameter(Caro1);
        double VLast1 = HSurfaceTool.LastVParameter(Caro1);

        double UFirst2 = HSurfaceTool.FirstUParameter(Caro2);
        double VFirst2 = HSurfaceTool.FirstVParameter(Caro2);
        double ULast2 = HSurfaceTool.LastUParameter(Caro2);
        double VLast2 = HSurfaceTool.LastVParameter(Caro2);

        bool bOutOfTangentZone = false;
        bool bStop = !myIntersectionOn2S.IsTangent();
        int dIncKey = 1;
        NCollection_Array1<double> Param = new(1, 4);
        IntWalk_StatusDeflection aStatus = IntWalk_OK;
        int nbIterWithoutAppend = 0;
        int nbEqualPoints = 0;
        int parit = 0;
        int uvit = 0;
        NCollection_Sequence<IntSurf_PntOn2S> aSeqOfNewPoint = new();

        SetParameters(Param, previousPoint);

        if (Param[1] - UFirst1 < ResoU1)
        {
            return bOutOfTangentZone;
        }
        else if (Param[2] - VFirst1 < ResoV1)
        {
            return bOutOfTangentZone;
        }
        else if (Param[3] - UFirst2 < ResoU2)
        {
            return bOutOfTangentZone;
        }
        else if (Param[4] - VFirst2 < ResoV2)
        {
            return bOutOfTangentZone;
        }

        if (Param[1] - ULast1 > -ResoU1)
        {
            return bOutOfTangentZone;
        }
        else if (Param[2] - VLast1 > -ResoV1)
        {
            return bOutOfTangentZone;
        }
        else if (Param[3] - ULast2 > -ResoU2)
        {
            return bOutOfTangentZone;
        }
        else if (Param[4] - VLast2 > -ResoV2)
        {
            return bOutOfTangentZone;
        }

        Span<double> SvParam = stackalloc double[4];
        while (!bStop)
        {
            nbIterWithoutAppend++;

            if ((nbIterWithoutAppend > 20) || (nbEqualPoints > 20))
            {
                bStop = true;
                break;
            }
            double f = 0.0;

            switch (theChoixIso)
            {
                case IntImp_UIsoparametricOnCaro1:
                    f = Math.Abs(previousd1.X());
                    break;
                case IntImp_VIsoparametricOnCaro1:
                    f = Math.Abs(previousd1.Y());
                    break;
                case IntImp_UIsoparametricOnCaro2:
                    f = Math.Abs(previousd2.X());
                    break;
                case IntImp_VIsoparametricOnCaro2:
                    f = Math.Abs(previousd2.Y());
                    break;
            }

            if (f < 0.1)
            {
                f = 0.1;
            }

            SetParameters(Param, previousPoint);

            double dP1 = sensCheminement * pasuv[0] * previousd1.X() / f;
            double dP2 = sensCheminement * pasuv[1] * previousd1.Y() / f;
            double dP3 = sensCheminement * pasuv[2] * previousd2.X() / f;
            double dP4 = sensCheminement * pasuv[3] * previousd2.Y() / f;

            if (theChoixIso == IntImp_UIsoparametricOnCaro1 && Math.Abs(dP1) < 1.0e-7)
            {
                dP1 *= (5.0 * (double)dIncKey);
            }
            if (theChoixIso == IntImp_VIsoparametricOnCaro1 && Math.Abs(dP2) < 1.0e-7)
            {
                dP2 *= (5.0 * (double)dIncKey);
            }
            if (theChoixIso == IntImp_UIsoparametricOnCaro2 && Math.Abs(dP3) < 1.0e-7)
            {
                dP3 *= (5.0 * (double)dIncKey);
            }
            if (theChoixIso == IntImp_VIsoparametricOnCaro2 && Math.Abs(dP4) < 1.0e-7)
            {
                dP4 *= (5.0 * (double)dIncKey);
            }

            Param[1] += dP1;
            Param[2] += dP2;
            Param[3] += dP3;
            Param[4] += dP4;
            IntImp_ConstIsoparametric ChoixIso = theChoixIso;

            for (parit = 0; parit < 4; parit++)
            {
                SvParam[parit] = Param[parit + 1];
            }
            FunctionSetRoot Rsnld = new(myIntersectionOn2S.Function());
            ChoixIso = myIntersectionOn2S.Perform(Param, Rsnld, theChoixIso);

            if (!myIntersectionOn2S.IsDone())
            {
                return bOutOfTangentZone;
            }
            else
            {
                if (myIntersectionOn2S.IsEmpty())
                {
                    return bOutOfTangentZone;
                }

                aStatus = TestDeflection(ChoixIso, aStatus);

                if (aStatus == IntWalk_OK)
                {
                    for (uvit = 0; uvit < 4; uvit++)
                    {
                        if (pasuv[uvit] < pasInit[uvit])
                        {
                            pasuv[uvit] = pasInit[uvit];
                        }
                    }
                }

                switch (aStatus)
                {
                    case IntWalk_ArretSurPointPrecedent:
                    {
                        bStop = true;
                        bOutOfTangentZone = !myIntersectionOn2S.IsTangent();
                        break;
                    }
                    case IntWalk_PasTropGrand:
                    {
                        for (parit = 0; parit < 4; parit++)
                        {
                            Param[parit + 1] = SvParam[parit];
                        }
                        bool bDecrease = false;

                        for (uvit = 0; uvit < 4; uvit++)
                        {
                            if (pasSav[uvit] < pasInit[uvit])
                            {
                                pasInit[uvit] -= (pasInit[uvit] - pasSav[uvit]) * 0.1;
                                bDecrease = true;
                            }
                        }

                        if (bDecrease)
                        {
                            nbIterWithoutAppend--;
                        }
                        break;
                    }
                    case IntWalk_PointConfondu:
                    {
                        for (uvit = 0; uvit < 4; uvit++)
                        {
                            if (pasuv[uvit] < pasInit[uvit])
                            {
                                pasuv[uvit] += (pasInit[uvit] - pasuv[uvit]) * 0.1;
                            }
                        }
                        break;
                    }
                    case IntWalk_OK:
                    case IntWalk_ArretSurPoint:
                    {
                        //
                        bStop = TestArret(theDirectionFlag, Param, ref ChoixIso);
                        //

                        //
                        if (!bStop)
                        {
                            myIntersectionOn2S.Point().Parameters(out double u11, out double v11, out double u12,
                                                                  out double v12);
                            previousPoint.Parameters(out double u21, out double v21, out double u22, out double v22);

                            if (((Math.Abs(u11 - u21) < ResoU1) && (Math.Abs(v11 - v21) < ResoV1))
                                || ((Math.Abs(u12 - u22) < ResoU2) && (Math.Abs(v12 - v22) < ResoV2)))
                            {
                                nbEqualPoints++;
                            }
                            else
                            {
                                nbEqualPoints = 0;
                            }
                        }
                        //

                        bStop = bStop || !myIntersectionOn2S.IsTangent();
                        bOutOfTangentZone = !myIntersectionOn2S.IsTangent();

                        if (!bStop)
                        {
                            bool pointisvalid = false;
                            myIntersectionOn2S.Point().Parameters(out double u1, out double v1, out double u2,
                                                                  out double v2);

                            if (u1 <= UM1 && u2 <= UM2 && v1 <= VM1 && v2 <= VM2 && u1 >= Um1 && u2 >= Um2
                                && v1 >= Vm1 && v2 >= Vm2)
                            {
                                pointisvalid = true;
                            }

                            if (pointisvalid)
                            {
                                previousPoint = myIntersectionOn2S.Point();
                                previoustg = myIntersectionOn2S.IsTangent();

                                if (!previoustg)
                                {
                                    previousd = myIntersectionOn2S.Direction();
                                    previousd1 = myIntersectionOn2S.DirectionOnS1();
                                    previousd2 = myIntersectionOn2S.DirectionOnS2();
                                }
                                bool bAddPoint = true;

                                if (line.NbPoints() >= 1)
                                {
                                    Point3D pf = line.Value(1).Value();
                                    Point3D pl = previousPoint.Value();

                                    if (pf.Distance(pl) < global::Kerf.Engine.Foundation.Precision.Confusion())
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
                                    aSeqOfNewPoint.Append(previousPoint);
                                    nbIterWithoutAppend = 0;
                                }
                            }

                            if (line.NbPoints() == 2)
                            {
                                for (uvit = 0; uvit < 4; uvit++)
                                {
                                    pasSav[uvit] = pasuv[uvit];
                                }
                            }

                            if (!pointisvalid)
                            {
                                // decrease step if out of bounds
                                // otherwise the same calculations will be
                                // repeated several times
                                if ((u1 > UM1) || (u1 < Um1))
                                {
                                    pasuv[0] *= 0.5;
                                }

                                if ((v1 > VM1) || (v1 < Vm1))
                                {
                                    pasuv[1] *= 0.5;
                                }

                                if ((u2 > UM2) || (u2 < Um2))
                                {
                                    pasuv[2] *= 0.5;
                                }

                                if ((v2 > VM2) || (v2 < Vm2))
                                {
                                    pasuv[3] *= 0.5;
                                }
                            }
                        } // end if(!bStop)
                        else
                        { // if(bStop)
                            if (close && (line.NbPoints() >= 1))
                            {
                                if (!bOutOfTangentZone)
                                {
                                    aSeqOfNewPoint.Append(line.Value(1)); // line end
                                }
                                nbIterWithoutAppend = 0;
                            }
                            else
                            {
                                ChoixIso = myIntersectionOn2S.Perform(Param, Rsnld, theChoixIso);

                                if (myIntersectionOn2S.IsEmpty())
                                {
                                    bStop = true; // !myIntersectionOn2S.IsTangent();
                                    bOutOfTangentZone = false; // !myIntersectionOn2S.IsTangent();
                                }
                                else
                                {
                                    bool bAddPoint = true;
                                    bool pointisvalid = false;

                                    previousPoint = myIntersectionOn2S.Point();
                                    previousPoint.Parameters(out double u1, out double v1, out double u2,
                                                             out double v2);

                                    if (u1 <= UM1 && u2 <= UM2 && v1 <= VM1 && v2 <= VM2 && u1 >= Um1 && u2 >= Um2
                                        && v1 >= Vm1 && v2 >= Vm2)
                                    {
                                        pointisvalid = true;
                                    }

                                    if (pointisvalid)
                                    {
                                        if (line.NbPoints() >= 1)
                                        {
                                            Point3D pf = line.Value(1).Value();
                                            Point3D pl = previousPoint.Value();

                                            if (pf.Distance(pl) < global::Kerf.Engine.Foundation.Precision.Confusion())
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
                                            aSeqOfNewPoint.Append(previousPoint);
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
                previousPoint.Parameters(out eu1, out ev1, out eu2, out ev2);
            }
            else
            {
                if (aSeqOfNewPoint.Length() > 0)
                {
                    aSeqOfNewPoint.Value(aSeqOfNewPoint.Length()).Parameters(out eu1, out ev1, out eu2, out ev2);
                }
                else
                {
                    break;
                }
            }

            if (((eu1 - Um1) < ResoU1) || ((UM1 - eu1) < ResoU1) || ((eu2 - Um2) < ResoU2)
                || ((UM2 - eu2) < ResoU2) || ((ev1 - Vm1) < ResoV1) || ((VM1 - ev1) < ResoV1)
                || ((ev2 - Vm2) < ResoV2) || ((VM2 - ev2) < ResoV2))
            {
                bExtendLine = true;
            }
        }

        if (!bExtendLine)
        {
            //    if(aStatus == IntWalk_OK || aStatus == IntWalk_ArretSurPoint) {
            if (aStatus == IntWalk_OK)
            {
                bExtendLine = true;

                if (aSeqOfNewPoint.Length() > 1)
                {
                    NCollection_Array1<double> FirstParams = new(0, 3), LastParams = new(0, 3), Resolutions = new(0, 3);
                    Resolutions[0] = ResoU1;
                    Resolutions[1] = ResoV1;
                    Resolutions[2] = ResoU2;
                    Resolutions[3] = ResoV2;

                    aSeqOfNewPoint.Value(1).Parameters(out FirstParams[0], out FirstParams[1], out FirstParams[2],
                                                       out FirstParams[3]);
                    aSeqOfNewPoint.Value(aSeqOfNewPoint.Length()).Parameters(out LastParams[0], out LastParams[1],
                                                                             out LastParams[2], out LastParams[3]);
                    int indexofiso = 0;

                    if (theChoixIso == IntImp_UIsoparametricOnCaro1)
                    {
                        indexofiso = 0;
                    }
                    if (theChoixIso == IntImp_VIsoparametricOnCaro1)
                    {
                        indexofiso = 1;
                    }
                    if (theChoixIso == IntImp_UIsoparametricOnCaro2)
                    {
                        indexofiso = 2;
                    }
                    if (theChoixIso == IntImp_VIsoparametricOnCaro2)
                    {
                        indexofiso = 3;
                    }

                    int afirstindex = (indexofiso < 2) ? 0 : 2;
                    Vector2D aTangentZoneDir = new(
                        new Point2D(FirstParams.Value(afirstindex), FirstParams.Value(afirstindex + 1)),
                        new Point2D(LastParams.Value(afirstindex), LastParams.Value(afirstindex + 1)));

                    Direction2D anIsoDir = new(Direction2D.D.Y);

                    if ((indexofiso == 1) || (indexofiso == 3))
                    {
                        anIsoDir = new Direction2D(Direction2D.D.X);
                    }

                    if (aTangentZoneDir.SquareMagnitude() > Constants.Resolution())
                    {
                        double piquota = M_PI * 0.25;

                        if (Math.Abs(aTangentZoneDir.Angle(new Vector2D(anIsoDir))) > piquota)
                        {
                            int ii = 1, nextii = 2;
                            Vector2D d1 = new(0, 0);
                            double asqresol = Constants.Resolution();
                            asqresol *= asqresol;

                            do
                            {
                                aSeqOfNewPoint.Value(ii).Parameters(out FirstParams[0], out FirstParams[1],
                                                                    out FirstParams[2], out FirstParams[3]);
                                aSeqOfNewPoint.Value(ii + 1).Parameters(out LastParams[0], out LastParams[1],
                                                                        out LastParams[2], out LastParams[3]);
                                d1 = new Vector2D(
                                    new Point2D(FirstParams.Value(afirstindex), FirstParams.Value(afirstindex + 1)),
                                    new Point2D(LastParams.Value(afirstindex), LastParams.Value(afirstindex + 1)));
                                ii++;
                            } while ((d1.SquareMagnitude() < asqresol) && (ii < aSeqOfNewPoint.Length()));

                            nextii = ii;

                            while (nextii < aSeqOfNewPoint.Length())
                            {
                                Vector2D nextd1 = new(0, 0);
                                int jj = nextii;

                                do
                                {
                                    aSeqOfNewPoint.Value(jj).Parameters(out FirstParams[0], out FirstParams[1],
                                                                        out FirstParams[2], out FirstParams[3]);
                                    aSeqOfNewPoint.Value(jj + 1).Parameters(out LastParams[0], out LastParams[1],
                                                                            out LastParams[2], out LastParams[3]);
                                    nextd1 = new Vector2D(
                                        new Point2D(FirstParams.Value(afirstindex), FirstParams.Value(afirstindex + 1)),
                                        new Point2D(LastParams.Value(afirstindex), LastParams.Value(afirstindex + 1)));
                                    jj++;
                                } while ((nextd1.SquareMagnitude() < asqresol) && (jj < aSeqOfNewPoint.Length()));
                                nextii = jj;

                                if (Math.Abs(d1.Angle(nextd1)) > piquota)
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

        for (i = 1; i <= aSeqOfNewPoint.Length(); i++)
        {
            AddAPoint(aSeqOfNewPoint.Value(i));
        }

        return bOutOfTangentZone;
    }

    /// <summary>
    /// Moves theInit (u1, v1, u2, v2, initialised by the caller) down the gradient of the square distance between
    /// the surfaces' points (B-spline and Bézier surfaces only; true at once for others). True if the distance falls
    /// below 1e-7.
    /// </summary>
    internal bool DistanceMinimizeByGradient(SurfaceAdaptorBase theASurf1, SurfaceAdaptorBase theASurf2,
                                             NCollection_Array1<double> theInit, double[]? theStep0 = null)
    {
        const int aNbIterMAX = 60;
        const double aTol = 1.0e-14;
        double aTolNul = 1.0 / global::Kerf.Engine.Foundation.Precision.Infinite();

        // I.e. if theU1 = 0.0 then Epsilon(theU1) = DBL_MIN (~1.0e-308).
        // Work with this number is impossible: there is a dangerous to
        // obtain Floating-point-overflow. Therefore, we limit this value.
        double aMinAddValU1 = Math.Max(Epsilon(theInit[1]), aTolNul);
        double aMinAddValV1 = Math.Max(Epsilon(theInit[2]), aTolNul);
        double aMinAddValU2 = Math.Max(Epsilon(theInit[3]), aTolNul);
        double aMinAddValV2 = Math.Max(Epsilon(theInit[4]), aTolNul);

        if (theASurf1.GetType() != SurfaceKind.BezierSurface && theASurf1.GetType() != SurfaceKind.BSplineSurface)
        {
            return true;
        }
        if (theASurf2.GetType() != SurfaceKind.BezierSurface && theASurf2.GetType() != SurfaceKind.BSplineSurface)
        {
            return true;
        }

        bool aStatus = false;

        theASurf1.D1(theInit[1], theInit[2], out Point3D aP1, out Vector3D aD1u, out Vector3D aD1v);
        theASurf2.D1(theInit[3], theInit[4], out Point3D aP2, out Vector3D aD2U, out Vector3D aD2V);

        double aSQDistPrev = aP1.SquareDistance(aP2);

        Vector3D aP12 = new(aP1, aP2);

        double aGradFu = -aP12.Dot(aD1u);
        double aGradFv = -aP12.Dot(aD1v);
        double aGradFU = aP12.Dot(aD2U);
        double aGradFV = aP12.Dot(aD2V);

        double aStepU1 = 1.0e-6, aStepV1 = 1.0e-6, aStepU2 = 1.0e-6, aStepV2 = 1.0e-6;

        if (theStep0 != null)
        {
            aStepU1 = theStep0[0];
            aStepV1 = theStep0[1];
            aStepU2 = theStep0[2];
            aStepV2 = theStep0[3];
        }

        bool flRepeat = true;
        int aNbIter = aNbIterMAX;

        while (flRepeat)
        {
            double anAdd = aGradFu * aStepU1;
            double aPARu = theInit[1] - Math.CopySign(Math.Max(Math.Abs(anAdd), aMinAddValU1), anAdd);

            anAdd = aGradFv * aStepV1;
            double aPARv = theInit[2] - Math.CopySign(Math.Max(Math.Abs(anAdd), aMinAddValV1), anAdd);

            anAdd = aGradFU * aStepU2;
            double aParU = theInit[3] - Math.CopySign(Math.Max(Math.Abs(anAdd), aMinAddValU2), anAdd);

            anAdd = aGradFV * aStepV2;
            double aParV = theInit[4] - Math.CopySign(Math.Max(Math.Abs(anAdd), aMinAddValV2), anAdd);

            theASurf1.D1(aPARu, aPARv, out Point3D aPt1, out aD1u, out aD1v);
            theASurf2.D1(aParU, aParV, out Point3D aPt2, out aD2U, out aD2V);

            double aSQDist = aPt1.SquareDistance(aPt2);

            if (aSQDist < aSQDistPrev)
            {
                aSQDistPrev = aSQDist;
                theInit[1] = aPARu;
                theInit[2] = aPARv;
                theInit[3] = aParU;
                theInit[4] = aParV;

                aStatus = aSQDistPrev < aTol;
                aStepU1 *= 1.2;
                aStepV1 *= 1.2;
                aStepU2 *= 1.2;
                aStepV2 *= 1.2;
            }
            else
            {
                if (--aNbIter < 0)
                {
                    flRepeat = false;
                }
                else
                {
                    theASurf1.D1(theInit[1], theInit[2], out aPt1, out aD1u, out aD1v);
                    theASurf2.D1(theInit[3], theInit[4], out aPt2, out aD2U, out aD2V);

                    Vector3D aPt12 = new(aPt1, aPt2);
                    aGradFu = -aPt12.Dot(aD1u);
                    aGradFv = -aPt12.Dot(aD1v);
                    aGradFU = aPt12.Dot(aD2U);
                    aGradFV = aPt12.Dot(aD2V);

                    if (theStep0 != null)
                    {
                        aStepU1 = theStep0[0];
                        aStepV1 = theStep0[1];
                        aStepU2 = theStep0[2];
                        aStepV2 = theStep0[3];
                    }
                    else
                    {
                        aStepU1 = aStepV1 = aStepU2 = aStepV2 = 1.0e-6;
                    }
                }
            }
        }

        return aStatus;
    }

    /// <summary>
    /// Newton's iterations for the projection of theP0 on theASurf from (theU0, theV0) (initialised by the caller).
    /// True if the square distance falls below 1e-14.
    /// </summary>
    internal bool DistanceMinimizeByExtrema(SurfaceAdaptorBase theASurf, in Point3D theP0, ref double theU0,
                                            ref double theV0, double[]? theStep0 = null)
    {
        const double aTol = 1.0e-14;
        double aSQDistPrev = RealLast();
        double aU = theU0, aV = theV0;

        ReadOnlySpan<double> aStep0 = [theStep0 != null ? theStep0[0] : 1.0, theStep0 != null ? theStep0[1] : 1.0];

        int aNbIter = 10;
        do
        {
            theASurf.D2(aU, aV, out Point3D aPS, out Vector3D aD1Su, out Vector3D aD1Sv, out Vector3D aD2Su, out Vector3D aD2Sv,
                        out Vector3D _);

            Vector3D aVec = new(theP0, aPS);

            double aSQDist = aVec.SquareMagnitude();

            if (aSQDist >= aSQDistPrev)
            {
                break;
            }

            aSQDistPrev = aSQDist;
            theU0 = aU;
            theV0 = aV;
            aNbIter--;

            if (aSQDistPrev < aTol)
            {
                break;
            }

            // Functions
            double aF1 = aD1Su.Dot(aVec), aF2 = aD1Sv.Dot(aVec);

            // Derivatives
            double aDf1u = aD2Su.Dot(aVec) + aD1Su.Dot(aD1Su), aDf1v = aD2Su.Dot(aD1Sv), aDf2u = aDf1v,
                   aDf2v = aD2Sv.Dot(aVec) + aD1Sv.Dot(aD1Sv);

            double aDet = aDf1u * aDf2v - aDf1v * aDf2u;
            aU -= aStep0[0] * (aDf2v * aF1 - aDf1v * aF2) / aDet;
            aV += aStep0[1] * (aDf2u * aF1 - aDf1u * aF2) / aDet;
        } while (aNbIter > 0);

        return (aSQDistPrev < aTol);
    }

    /// <summary>
    /// Snaps thePnt (u1, v1, u2, v2) on a boundary: an intersection point with a parameter locked on the boundary it
    /// is near, then its other parameters onto boundaries within the3DTol. True if the point found is in the domain.
    /// </summary>
    internal bool HandleSingleSingularPoint(SurfaceAdaptorBase theASurf1, SurfaceAdaptorBase theASurf2,
                                            double the3DTol, NCollection_Array1<double> thePnt)
    {
        // u1, v1, u2, v2 order is used.
        ReadOnlySpan<double> aLowBorder = [theASurf1.FirstUParameter(), theASurf1.FirstVParameter(),
                                           theASurf2.FirstUParameter(), theASurf2.FirstVParameter()];
        ReadOnlySpan<double> aUppBorder = [theASurf1.LastUParameter(), theASurf1.LastVParameter(),
                                           theASurf2.LastUParameter(), theASurf2.LastVParameter()];
        ReadOnlySpan<IntImp_ConstIsoparametric> aLockedDir = [IntImp_UIsoparametricOnCaro1,
                                                              IntImp_VIsoparametricOnCaro1,
                                                              IntImp_UIsoparametricOnCaro2,
                                                              IntImp_VIsoparametricOnCaro2];

        // Create new intersector with new tolerance.
        IntWalk_TheInt2S anInt = new(theASurf1, theASurf2, the3DTol);
        FunctionSetRoot aRsnld = new(anInt.Function());

        Span<double> aPars = stackalloc double[4];
        for (int i = 1; i <= 4; ++i)
        {
            if (Math.Abs(thePnt[i] - aLowBorder[i - 1]) < global::Kerf.Engine.Foundation.Precision.PConfusion()
                || Math.Abs(thePnt[i] - aUppBorder[i - 1]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                anInt.Perform(thePnt, aRsnld, aLockedDir[i - 1]);

                if (!anInt.IsDone())
                {
                    continue;
                }

                if (anInt.IsEmpty())
                {
                    continue;
                }

                anInt.Point().Parameters(out aPars[0], out aPars[1], out aPars[2], out aPars[3]);
                SurfaceAdaptorBase[] aSurfs = [theASurf1, theASurf2];
                // Local resolutions
                double aTol2 = the3DTol * the3DTol;
                Point3D aP;
                int k;
                for (k = 0; k < 2; ++k)
                {
                    int iu, iv;
                    iu = 2 * k;
                    iv = iu + 1;
                    aSurfs[k].D1(aPars[iu], aPars[iv], out Point3D aPInt, out Vector3D aDU, out Vector3D aDV);
                    double aMod = aDU.Magnitude();
                    if (aMod > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        double aTolU = the3DTol / aMod;
                        if (Math.Abs(aLowBorder[iu] - aPars[iu]) < aTolU)
                        {
                            aP = aSurfs[k].Value(aLowBorder[iu], aPars[iv]);
                            if (aPInt.SquareDistance(aP) < aTol2)
                            {
                                aPars[iu] = aLowBorder[iu];
                            }
                        }
                        else if (Math.Abs(aUppBorder[iu] - aPars[iu]) < aTolU)
                        {
                            aP = aSurfs[k].Value(aUppBorder[iu], aPars[iv]);
                            if (aPInt.SquareDistance(aP) < aTol2)
                            {
                                aPars[iu] = aUppBorder[iu];
                            }
                        }
                    }
                    aMod = aDV.Magnitude();
                    if (aMod > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        double aTolV = the3DTol / aMod;
                        if (Math.Abs(aLowBorder[iv] - aPars[iv]) < aTolV)
                        {
                            aP = aSurfs[k].Value(aPars[iu], aLowBorder[iv]);
                            if (aPInt.SquareDistance(aP) < aTol2)
                            {
                                aPars[iv] = aLowBorder[iv];
                            }
                        }
                        else if (Math.Abs(aUppBorder[iv] - aPars[iv]) < aTolV)
                        {
                            aP = aSurfs[k].Value(aPars[iu], aUppBorder[iv]);
                            if (aPInt.SquareDistance(aP) < aTol2)
                            {
                                aPars[iv] = aUppBorder[iv];
                            }
                        }
                    }
                }
                //
                //
                int j;
                for (j = 1; j <= 4; ++j)
                {
                    thePnt[j] = aPars[j - 1];
                }
                bool isInDomain = true;
                for (j = 1; isInDomain && (j <= 4); ++j)
                {
                    if ((thePnt[j] - aLowBorder[j - 1] + global::Kerf.Engine.Foundation.Precision.PConfusion())
                            * (thePnt[j] - aUppBorder[j - 1] - global::Kerf.Engine.Foundation.Precision.PConfusion())
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
    internal bool SeekPointOnBoundary(SurfaceAdaptorBase theASurf1, SurfaceAdaptorBase theASurf2, double theU1,
                                      double theV1, double theU2, double theV2, bool isTheFirst)
    {
        bool isOK = false;

        // u1, v1, u2, v2 order is used.
        double[] aLowBorder = [theASurf1.FirstUParameter(), theASurf1.FirstVParameter(),
                               theASurf2.FirstUParameter(), theASurf2.FirstVParameter()];
        double[] aUppBorder = [theASurf1.LastUParameter(), theASurf1.LastVParameter(),
                               theASurf2.LastUParameter(), theASurf2.LastVParameter()];

        // Tune solution tolerance according with object size.
        double aRes1 = Math.Max(global::Kerf.Engine.Foundation.Precision.PConfusion() / theASurf1.UResolution(1.0),
                                global::Kerf.Engine.Foundation.Precision.PConfusion() / theASurf1.VResolution(1.0));
        double aRes2 = Math.Max(global::Kerf.Engine.Foundation.Precision.PConfusion() / theASurf2.UResolution(1.0),
                                global::Kerf.Engine.Foundation.Precision.PConfusion() / theASurf2.VResolution(1.0));
        double a3DTol = Math.Max(aRes1, aRes2);
        double aTol = Math.Max(global::Kerf.Engine.Foundation.Precision.Confusion(), a3DTol);

        // u1, v1, u2, v2 order is used.
        NCollection_Array1<double> aPnt = new(1, 4);
        aPnt[1] = theU1;
        aPnt[2] = theV1;
        aPnt[3] = theU2;
        aPnt[4] = theV2;
        NCollection_Array1<double> aSingularPnt = new(aPnt);

        int aNbIter = 20;
        bool aStatus = false;
        do
        {
            aNbIter--;
            aStatus = DistanceMinimizeByGradient(theASurf1, theASurf2, aPnt);
            if (aStatus && !AdjustToDomain(4, aPnt, 1, aLowBorder, aUppBorder, 0))
            {
                break;
            }

            aStatus = DistanceMinimizeByExtrema(theASurf1, theASurf2.Value(aPnt[3], aPnt[4]), ref aPnt[1], ref aPnt[2]);
            if (aStatus && !AdjustToDomain(2, aPnt, 1, aLowBorder, aUppBorder, 0))
            {
                break;
            }

            aStatus = DistanceMinimizeByExtrema(theASurf2, theASurf1.Value(aPnt[1], aPnt[2]), ref aPnt[3], ref aPnt[4]);
            if (aStatus && !AdjustToDomain(2, aPnt, 3, aLowBorder, aUppBorder, 2))
            {
                break;
            }
        } while (!aStatus && (aNbIter > 0));

        // Handle singular points.
        bool aSingularStatus = HandleSingleSingularPoint(theASurf1, theASurf2, aTol, aSingularPnt);
        if (aSingularStatus)
        {
            aPnt.Assign(aSingularPnt);
        }

        if (!aStatus && !aSingularStatus)
        {
            return isOK;
        }

        Point3D aP1 = theASurf1.Value(aPnt[1], aPnt[2]);
        Point3D aP2 = theASurf2.Value(aPnt[3], aPnt[4]);
        Point3D aPInt = new(0.5 * (aP1.XYZ() + aP2.XYZ()));

        double aSQDist = aPInt.SquareDistance(aP1);
        if (aSQDist > aTol * aTol)
        {
            return isOK;
        }

        // Found point is true intersection point
        IntSurf_PntOn2S anIP = new();
        anIP.SetValue(aPInt, aPnt[1], aPnt[2], aPnt[3], aPnt[4]);

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
            while (line.NbPoints() > 1)
            {
                int aNbPnts = line.NbPoints();

                int aPInd = 1;
                for (; aPInd <= aNbPnts; aPInd++)
                {
                    aP1.SetXYZ(line.Value(aPInd).Value().XYZ());
                    if (aP1.SquareDistance(aPInt) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        break;
                    }
                }

                for (++aPInd; aPInd <= aNbPnts; aPInd++)
                {
                    aP2.SetXYZ(line.Value(aPInd).Value().XYZ());
                    if (aP1.SquareDistance(aP2) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        break;
                    }
                }

                if (aPInd > aNbPnts)
                {
                    return isOK;
                }

                Xyz aDir01 = aP1.XYZ() - aPInt.XYZ();
                Xyz aDir12 = aP2.XYZ() - aP1.XYZ();

                if (aDir01.Dot(aDir12) > 0.0)
                {
                    break;
                }

                RemoveAPoint(1);
            }

            aP1.SetXYZ(line.Value(1).Value().XYZ());
            if (aP1.SquareDistance(aPInt) <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                RemoveAPoint(1);
            }

            line.InsertBefore(1, anIP);
            isOK = true;
        }
        else
        {
            while (line.NbPoints() > 1)
            {
                int aNbPnts = line.NbPoints();

                Point3D aPPrev = new(), aPCurr = new();
                int aPInd = aNbPnts;
                for (; aPInd > 0; aPInd--)
                {
                    aPCurr.SetXYZ(line.Value(aPInd).Value().XYZ());
                    if (aPCurr.SquareDistance(aPInt) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        break;
                    }
                }

                for (--aPInd; aPInd > 0; aPInd--)
                {
                    aPPrev.SetXYZ(line.Value(aPInd).Value().XYZ());
                    if (aPCurr.SquareDistance(aPPrev) > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        break;
                    }
                }

                if (aPInd < 1)
                {
                    return isOK;
                }

                Xyz aDirPC = aPCurr.XYZ() - aPPrev.XYZ();
                Xyz aDirCN = aPInt.XYZ() - aPCurr.XYZ();

                if (aDirPC.Dot(aDirCN) > 0.0)
                {
                    break;
                }

                RemoveAPoint(aNbPnts);
            }

            int aNbPntsL = line.NbPoints();
            aP1.SetXYZ(line.Value(aNbPntsL).Value().XYZ());
            if (aP1.SquareDistance(aPInt) <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            {
                RemoveAPoint(aNbPntsL);
            }
            line.Add(anIP);

            isOK = true;
        }

        return isOK;
    }

    /// <summary>
    /// Puts the ends of the line on the surfaces' boundaries when they are near them (closer than 1e-3 of the
    /// smallest range). True if a point was added.
    /// </summary>
    public bool PutToBoundary(SurfaceAdaptorBase theASurf1, SurfaceAdaptorBase theASurf2)
    {
        double aTolMin = global::Kerf.Engine.Foundation.Precision.Confusion();

        bool hasBeenAdded = false;

        double aU1bFirst = theASurf1.FirstUParameter();
        double aU1bLast = theASurf1.LastUParameter();
        double aU2bFirst = theASurf2.FirstUParameter();
        double aU2bLast = theASurf2.LastUParameter();
        double aV1bFirst = theASurf1.FirstVParameter();
        double aV1bLast = theASurf1.LastVParameter();
        double aV2bFirst = theASurf2.FirstVParameter();
        double aV2bLast = theASurf2.LastVParameter();

        double aTol = 1.0;
        aTol = Math.Min(aTol, aU1bLast - aU1bFirst);
        aTol = Math.Min(aTol, aU2bLast - aU2bFirst);
        aTol = Math.Min(aTol, aV1bLast - aV1bFirst);
        aTol = Math.Min(aTol, aV2bLast - aV2bFirst) * 1.0e-3;

        if (aTol <= 2.0 * aTolMin)
        {
            return hasBeenAdded;
        }

        bool isNeedAdding = false;
        IsParallel(line, true, aTol, out bool isU1parallel, out bool isV1parallel);
        IsParallel(line, false, aTol, out bool isU2parallel, out bool isV2parallel);

        line.Value(1).Parameters(out double u1, out double v1, out double u2, out double v2);
        double aDelta = 0.0;

        if (!isV1parallel)
        {
            aDelta = u1 - aU1bFirst;
            if ((aTolMin < aDelta) && (aDelta < aTol))
            {
                u1 = aU1bFirst;
                isNeedAdding = true;
            }
            else
            {
                aDelta = aU1bLast - u1;
                if ((aTolMin < aDelta) && (aDelta < aTol))
                {
                    u1 = aU1bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (!isV2parallel)
        {
            aDelta = u2 - aU2bFirst;
            if ((aTolMin < aDelta) && (aDelta < aTol))
            {
                u2 = aU2bFirst;
                isNeedAdding = true;
            }
            else
            {
                aDelta = aU2bLast - u2;
                if ((aTolMin < aDelta) && (aDelta < aTol))
                {
                    u2 = aU2bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (!isU1parallel)
        {
            aDelta = v1 - aV1bFirst;
            if ((aTolMin < aDelta) && (aDelta < aTol))
            {
                v1 = aV1bFirst;
                isNeedAdding = true;
            }
            else
            {
                aDelta = aV1bLast - v1;
                if ((aTolMin < aDelta) && (aDelta < aTol))
                {
                    v1 = aV1bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (!isU2parallel)
        {
            aDelta = v2 - aV2bFirst;
            if ((aTolMin < aDelta) && (aDelta < aTol))
            {
                v2 = aV2bFirst;
                isNeedAdding = true;
            }
            else
            {
                aDelta = aV2bLast - v2;
                if ((aTolMin < aDelta) && (aDelta < aTol))
                {
                    v2 = aV2bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (isNeedAdding)
        {
            hasBeenAdded = SeekPointOnBoundary(theASurf1, theASurf2, u1, v1, u2, v2, true);
        }

        int aNbPnts = line.NbPoints();
        isNeedAdding = false;
        line.Value(aNbPnts).Parameters(out u1, out v1, out u2, out v2);

        if (!isV1parallel)
        {
            aDelta = u1 - aU1bFirst;
            if ((aTolMin < aDelta) && (aDelta < aTol))
            {
                u1 = aU1bFirst;
                isNeedAdding = true;
            }
            else
            {
                aDelta = aU1bLast - u1;
                if ((aTolMin < aDelta) && (aDelta < aTol))
                {
                    u1 = aU1bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (!isV2parallel)
        {
            aDelta = u2 - aU2bFirst;
            if ((aTolMin < aDelta) && (aDelta < aTol))
            {
                u2 = aU2bFirst;
                isNeedAdding = true;
            }
            else
            {
                aDelta = aU2bLast - u2;
                if ((aTolMin < aDelta) && (aDelta < aTol))
                {
                    u2 = aU2bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (!isU1parallel)
        {
            aDelta = v1 - aV1bFirst;
            if ((aTolMin < aDelta) && (aDelta < aTol))
            {
                v1 = aV1bFirst;
                isNeedAdding = true;
            }
            else
            {
                aDelta = aV1bLast - v1;
                if ((aTolMin < aDelta) && (aDelta < aTol))
                {
                    v1 = aV1bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (!isU2parallel)
        {
            aDelta = v2 - aV2bFirst;
            if ((aTolMin < aDelta) && (aDelta < aTol))
            {
                v2 = aV2bFirst;
                isNeedAdding = true;
            }
            else
            {
                aDelta = aV2bLast - v2;
                if ((aTolMin < aDelta) && (aDelta < aTol))
                {
                    v2 = aV2bLast;
                    isNeedAdding = true;
                }
            }
        }

        if (isNeedAdding)
        {
            hasBeenAdded = SeekPointOnBoundary(theASurf1, theASurf2, u1, v1, u2, v2, false);
        }

        return hasBeenAdded;
    }

    /// <summary>
    /// Inserts exact intersection points in the middle of the segments of the line until it has theMinNbPoints
    /// points (or no more can be found). True if a point was inserted.
    /// </summary>
    public bool SeekAdditionalPoints(SurfaceAdaptorBase theASurf1, SurfaceAdaptorBase theASurf2, int theMinNbPoints)
    {
        const double aTol = 1.0e-14;
        int aNbPoints = line.NbPoints();
        if (aNbPoints > theMinNbPoints)
        {
            return true;
        }

        double aU1bFirst = theASurf1.FirstUParameter();
        double aU1bLast = theASurf1.LastUParameter();
        double aU2bFirst = theASurf2.FirstUParameter();
        double aU2bLast = theASurf2.LastUParameter();
        double aV1bFirst = theASurf1.FirstVParameter();
        double aV1bLast = theASurf1.LastVParameter();
        double aV2bFirst = theASurf2.FirstVParameter();
        double aV2bLast = theASurf2.LastVParameter();

        bool isPrecise = false;

        NCollection_Array1<double> aPnt = new(1, 4);
        aPnt.Init(0.0);

        int aNbPointsPrev = 0;
        while (aNbPoints < theMinNbPoints && (aNbPoints != aNbPointsPrev))
        {
            aNbPointsPrev = aNbPoints;
            for (int fp = 1, lp = 2; fp < aNbPoints; fp = lp + 1)
            {
                // first and last points in 1st and 2nd surafaces
                lp = fp + 1;
                line.Value(fp).Parameters(out double U1f, out double V1f, out double U2f, out double V2f);
                line.Value(lp).Parameters(out double U1l, out double V1l, out double U2l, out double V2l);

                aPnt[1] = 0.5 * (U1f + U1l);
                if (aPnt[1] < aU1bFirst)
                {
                    aPnt[1] = aU1bFirst;
                }
                if (aPnt[1] > aU1bLast)
                {
                    aPnt[1] = aU1bLast;
                }

                aPnt[2] = 0.5 * (V1f + V1l);
                if (aPnt[2] < aV1bFirst)
                {
                    aPnt[2] = aV1bFirst;
                }
                if (aPnt[2] > aV1bLast)
                {
                    aPnt[2] = aV1bLast;
                }

                aPnt[3] = 0.5 * (U2f + U2l);
                if (aPnt[3] < aU2bFirst)
                {
                    aPnt[3] = aU2bFirst;
                }
                if (aPnt[3] > aU2bLast)
                {
                    aPnt[3] = aU2bLast;
                }

                aPnt[4] = 0.5 * (V2f + V2l);
                if (aPnt[4] < aV2bFirst)
                {
                    aPnt[4] = aV2bFirst;
                }
                if (aPnt[4] > aV2bLast)
                {
                    aPnt[4] = aV2bLast;
                }

                bool aStatus = false;
                int aNbIter = 5;
                do
                {
                    aStatus = DistanceMinimizeByGradient(theASurf1, theASurf2, aPnt);
                    if (aStatus)
                    {
                        break;
                    }

                    aStatus = DistanceMinimizeByExtrema(theASurf1, theASurf2.Value(aPnt[3], aPnt[4]), ref aPnt[1],
                                                        ref aPnt[2]);
                    if (aStatus)
                    {
                        break;
                    }

                    aStatus = DistanceMinimizeByExtrema(theASurf2, theASurf1.Value(aPnt[1], aPnt[2]), ref aPnt[3],
                                                        ref aPnt[4]);
                    if (aStatus)
                    {
                        break;
                    }
                } while (!aStatus && (--aNbIter > 0));

                if (aStatus)
                {
                    Point3D aP1 = theASurf1.Value(aPnt[1], aPnt[2]), aP2 = theASurf2.Value(aPnt[3], aPnt[4]);
                    Point3D aPInt = new(0.5 * (aP1.XYZ() + aP2.XYZ()));

                    double aSQDist1 = aPInt.SquareDistance(aP1), aSQDist2 = aPInt.SquareDistance(aP2);

                    if ((aSQDist1 < aTol) && (aSQDist2 < aTol))
                    {
                        IntSurf_PntOn2S anIP = new();
                        anIP.SetValue(aPInt, aPnt[1], aPnt[2], aPnt[3], aPnt[4]);
                        line.InsertBefore(lp, anIP);

                        isPrecise = true;

                        if (++aNbPoints >= theMinNbPoints)
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
