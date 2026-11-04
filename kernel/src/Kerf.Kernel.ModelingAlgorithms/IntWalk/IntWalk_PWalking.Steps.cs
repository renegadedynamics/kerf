// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.Implicit.Tangence;
using static Kerf.Engine.Intersection.Implicit.ConstIsoparametric;
using static Kerf.Engine.Intersection.Walking.StatusDeflection;

namespace Kerf.Engine.Intersection.Walking;

internal partial class PWalking
{
    // OCC431(apo): modified ->
    private static readonly double CosRef2D = Math.Cos(M_PI / 9.0);
    private const double AngRef2D = M_PI / 2.0;
    private const double D = 7.0;

    /// <summary>
    /// Near a point where the marching fails: divides the steps to try to continue; if the step is too small (on a
    /// border), restarts in the other direction if not done yet, otherwise stops.
    /// </summary>
    public void RepartirOuDiviser(ref bool dejaReparti, ref ConstIsoparametric choixIso, ref bool arrive)
    {
        if (arrive)
        { // restart in the other direction
            if (!dejaReparti)
            {
                arrive = false;
                dejaReparti = true;
                _previousPoint = _line.Value(1);
                _previoustg = false;
                _previousd1 = _firstd1;
                _previousd2 = _firstd2;
                _previousd = _tgdir;
                _tangentIdx = _line.PointCount;
                _tgdir = _tgdir.Reversed();
                _line.Reverse();

                _sensCheminement = -1;
                _tgfirst = _tglast;
                _tglast = false;
                choixIso = _choixIsoSav;
                int nn = _line.PointCount;
                if (nn > 2)
                {
                    _line.Value(nn).Parameters(out double u1, out double v1, out double u2, out double v2);
                    _line.Value(nn - 1).Parameters(out double U1, out double V1, out double U2, out double V2);
                    _pasuv[0] = Math.Abs(u1 - U1);
                    _pasuv[1] = Math.Abs(v1 - V1);
                    _pasuv[2] = Math.Abs(u2 - U2);
                    _pasuv[3] = Math.Abs(v2 - V2);
                }
            }
        }
        else
        {
            if (_pasuv[0] * 0.5 < _resoU1 && _pasuv[1] * 0.5 < _resoV1 && _pasuv[2] * 0.5 < _resoU2
                && _pasuv[3] * 0.5 < _resoV2)
            {
                if (!_previoustg)
                {
                    _tglast = true; // IS IT ENOUGH ????
                }

                if (!dejaReparti)
                { // restart in the other direction
                    dejaReparti = true;
                    _previousPoint = _line.Value(1);
                    _previoustg = false;
                    _previousd1 = _firstd1;
                    _previousd2 = _firstd2;
                    _previousd = _tgdir;
                    _tangentIdx = _line.PointCount;
                    _tgdir = _tgdir.Reversed();
                    _line.Reverse();

                    _sensCheminement = -1;
                    _tgfirst = _tglast;
                    _tglast = false;
                    choixIso = _choixIsoSav;

                    int nn = _line.PointCount;
                    if (nn > 2)
                    {
                        _line.Value(nn).Parameters(out double u1, out double v1, out double u2, out double v2);
                        _line.Value(nn - 1).Parameters(out double U1, out double V1, out double U2, out double V2);
                        _pasuv[0] = Math.Abs(u1 - U1);
                        _pasuv[1] = Math.Abs(v1 - V1);
                        _pasuv[2] = Math.Abs(u2 - U2);
                        _pasuv[3] = Math.Abs(v2 - V2);
                    }
                }
                else
                {
                    arrive = true;
                }
            }
            else
            {
                _pasuv[0] *= 0.5;
                _pasuv[1] *= 0.5;
                _pasuv[2] *= 0.5;
                _pasuv[3] *= 0.5;
            }
        }
    }

    /// <summary>
    /// Tests the step to the current point of the intersector: the deflection between the previous and the current
    /// point (from their tangents), the angles in the parametric spaces and the change of side; adjusts the steps
    /// and returns the verdict. Calls <see cref="_stepTrace"/> with it when set.
    /// </summary>
    public StatusDeflection TestDeflection(ConstIsoparametric choixIso,
                                                   StatusDeflection status)
    {
        StatusDeflection result = TestDeflectionImpl(choixIso, status);
        (_stepTrace ?? s_threadStepTrace)?.Invoke(result, _intersectionOn2S.Point());
        return result;
    }

    // test if vector is observed by calculating an increase of vector
    //     or the previous point and its tangent, the new calculated point and its
    //     tangent; it is possible to find a cube passing by the 2 points and having as a
    //     derivative the tangents of the intersection
    //     calculate the point with parameter 0.5 on cube=p1
    //     calculate the medium point of 2 points of intersection=p2
    //   if arrow/2<=||p1p2||<= arrow consider that the vector is observed
    //   otherwise adjust the step depending on the ratio ||p1p2||/vector
    //   and the previous step
    // test if in  2 tangent planes of surfaces there is no too great angle2d
    // grand : if yes divide the step
    // test if there is no change of side
    private StatusDeflection TestDeflectionImpl(ConstIsoparametric choixIso,
                                                        StatusDeflection status)
    {
        if (_line.PointCount == 1)
        {
            STATIC_BLOCAGE_SUR_PAS_TROP_GRAND = STATIC_PRECEDENT_INFLEXION = 0;
        }

        StatusDeflection aStatus = StatusDeflection.OK;
        double flecheCourante, ratio = 1.0;

        // Caro1 and Caro2
        SurfaceAdaptorBase caro1 = _intersectionOn2S.Function().AuxillarSurface1();
        SurfaceAdaptorBase caro2 = _intersectionOn2S.Function().AuxillarSurface2();

        PntOn2S currentPoint = _intersectionOn2S.Point();
        //==================================================================================
        //=========               S t o p   o n   p o i n t                 ============
        //==================================================================================
        if (_intersectionOn2S.IsTangent)
        {
            return StatusDeflection.ArretSurPoint;
        }

        Direction3D tgCourante = _intersectionOn2S.Direction();

        double cosBetweenTangent = tgCourante.Dot(_previousd);

        //==================================================================================
        //=========   R i s k   o f    i n f l e x i o n   p o i n t  ============
        //==================================================================================
        if (cosBetweenTangent < 0)
        {
            //------------------------------------------------------------
            //-- Risk of inflexion point : Divide the step by 2
            //-- Initialize STATIC_PRECEDENT_INFLEXION so that
            //-- at the next call to return Pas_OK if there is no
            //-- more risk of the point of inflexion
            //------------------------------------------------------------

            _pasuv[0] *= 0.5;
            _pasuv[1] *= 0.5;
            _pasuv[2] *= 0.5;
            _pasuv[3] *= 0.5;
            STATIC_PRECEDENT_INFLEXION += 3;
            if (_pasuv[0] < _resoU1 && _pasuv[1] < _resoV1 && _pasuv[2] < _resoU2 && _pasuv[3] < _resoV2)
            {
                return StatusDeflection.ArretSurPointPrecedent;
            }
            else
            {
                return StatusDeflection.PasTropGrand;
            }
        }
        else
        {
            if (STATIC_PRECEDENT_INFLEXION > 0)
            {
                STATIC_PRECEDENT_INFLEXION--;
                return StatusDeflection.OK;
            }
        }

        //==================================================================================
        //=========  D e t e c t    c o n f u s e d    P o in t s       ===========
        //==================================================================================

        double sqDist = _previousPoint.Value().SquareDistanceTo(currentPoint.Value());

        if (sqDist < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            _pasInit[0] = Math.Max(_pasInit[0], 5.0 * _resoU1);
            _pasInit[1] = Math.Max(_pasInit[1], 5.0 * _resoV1);
            _pasInit[2] = Math.Max(_pasInit[2], 5.0 * _resoU2);
            _pasInit[3] = Math.Max(_pasInit[3], 5.0 * _resoV2);

            for (int i = 0; i < 4; i++)
            {
                _pasuv[i] = Math.Max(_pasuv[i], Math.Min(1.5 * _pasuv[i], _pasInit[i]));
            }
            // Compute local resolution: for OCC26717
            int iIso = (int)choixIso;
            if (Math.Abs(_pasuv[iIso] - _pasInit[iIso]) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                double curU, curV;
                if (choixIso == ConstIsoparametric.UIsoparametricOnCaro1 || choixIso == ConstIsoparametric.VIsoparametricOnCaro1)
                {
                    _previousPoint.ParametersOnS1(out curU, out curV);
                }
                else
                {
                    _previousPoint.ParametersOnS2(out curU, out curV);
                }
                Point3D curPnt = (choixIso == ConstIsoparametric.UIsoparametricOnCaro1 || choixIso == ConstIsoparametric.VIsoparametricOnCaro1)
                                    ? HSurfaceTool.Value(caro1, curU, curV)
                                    : HSurfaceTool.Value(caro2, curU, curV);
                Point3D offsetPnt = new();
                switch (choixIso)
                {
                    case ConstIsoparametric.UIsoparametricOnCaro1:
                        offsetPnt = HSurfaceTool.Value(caro1, curU + _sensCheminement * _pasuv[0], curV);
                        break;
                    case ConstIsoparametric.VIsoparametricOnCaro1:
                        offsetPnt = HSurfaceTool.Value(caro1, curU, curV + _sensCheminement * _pasuv[1]);
                        break;
                    case ConstIsoparametric.UIsoparametricOnCaro2:
                        offsetPnt = HSurfaceTool.Value(caro2, curU + _sensCheminement * _pasuv[2], curV);
                        break;
                    case ConstIsoparametric.VIsoparametricOnCaro2:
                        offsetPnt = HSurfaceTool.Value(caro2, curU, curV + _sensCheminement * _pasuv[3]);
                        break;
                    default:
                        break;
                }
                double refDist = curPnt.DistanceTo(offsetPnt);
                double localResol = 0.0;
                if (refDist > Constants.Resolution())
                {
                    localResol = _pasuv[iIso] * _tolconf / refDist;
                }
                if (_pasuv[iIso] < 2 * localResol)
                {
                    _pasuv[iIso] = _pasInit[iIso] = 2 * localResol;
                }
            }
            ////////////////////////////////////////
            aStatus = StatusDeflection.PointConfondu;
        }

        //==================================================================================
        double du1, dv1, absDu1, absDu2, absDv1, absDv2;
        double du2, dv2;

        _previousPoint.Parameters(out double up1, out double vp1, out double up2, out double vp2);
        currentPoint.Parameters(out double uc1, out double vc1, out double uc2, out double vc2);

        du1 = uc1 - up1;
        dv1 = vc1 - vp1;
        du2 = uc2 - up2;
        dv2 = vc2 - vp2;

        absDu1 = Math.Abs(du1);
        absDu2 = Math.Abs(du2);
        absDv1 = Math.Abs(dv1);
        absDv2 = Math.Abs(dv2);
        //=================================================================================
        //====   S t e p   o f   p  r o g r e s s i o n (between previous and Current)   =======
        //=================================================================================
        if (absDu1 < _resoU1 && absDv1 < _resoV1 && absDu2 < _resoU2 && absDv2 < _resoV2)
        {
            _pasuv[0] = _resoU1;
            _pasuv[1] = _resoV1;
            _pasuv[2] = _resoU2;
            _pasuv[3] = _resoV2;
            return (StatusDeflection.ArretSurPointPrecedent);
        }
        //==================================================================================

        double tolArea = 100.0;
        if (_resoU1 < global::Kerf.Engine.Foundation.Precision.PConfusion() || _resoV1 < global::Kerf.Engine.Foundation.Precision.PConfusion()
            || _resoU2 < global::Kerf.Engine.Foundation.Precision.PConfusion() || _resoV2 < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            tolArea = tolArea * 2.0;
        }

        double cosi1, cosRef1, ang1, angRef1, resoUV1, duv1, d1, tolCoeff1;
        double cosi2, cosRef2, ang2, angRef2, resoUV2, duv2, d2, tolCoeff2;
        cosi1 = du1 * _previousd1.X + dv1 * _previousd1.Y;
        cosi2 = du2 * _previousd2.X + dv2 * _previousd2.Y;
        duv1 = du1 * du1 + dv1 * dv1;
        duv2 = du2 * du2 + dv2 * dv2;
        resoUV1 = _resoU1 * _resoU1 + _resoV1 * _resoV1;
        resoUV2 = _resoU2 * _resoU2 + _resoV2 * _resoV2;
        //
        // modified by NIZNHY-PKV Wed Nov 13 12:25:44 2002 f
        //
        double minDiv2 = global::Kerf.Engine.Foundation.Precision.Confusion();
        minDiv2 = minDiv2 * minDiv2;
        //
        d1 = D;
        if (duv1 > minDiv2)
        {
            d1 = Math.Abs(resoUV1 / duv1);
            d1 = Math.Min(Math.Sqrt(d1) * tolArea, D);
        }
        tolCoeff1 = Math.Exp(d1);
        //
        // modified by NIZNHY-PKV Wed Nov 13 12:34:43 2002 f
        d2 = D;
        if (duv2 > minDiv2)
        {
            d2 = Math.Abs(resoUV2 / duv2);
            d2 = Math.Min(Math.Sqrt(d2) * tolArea, D);
        }
        tolCoeff2 = Math.Exp(d2);
        cosRef1 = CosRef2D / tolCoeff1;
        cosRef2 = CosRef2D / tolCoeff2;
        //
        //==================================================================================
        //== The points are not confused :                                           ==
        //== D e t e c t    t h e   S t o p   a  t   p r e v i o u s  p o i n t ==
        //==                           N o t    T o o    G r e a t (angle in space UV)    ==
        //==                           C h a n g e    o f    s i d e                ==
        //==================================================================================
        if (aStatus != StatusDeflection.PointConfondu)
        {
            if (cosi1 * cosi1 < cosRef1 * duv1 || cosi2 * cosi2 < cosRef2 * duv2)
            {
                _pasuv[0] *= 0.5;
                _pasuv[1] *= 0.5;
                _pasuv[2] *= 0.5;
                _pasuv[3] *= 0.5;
                if (_pasuv[0] < _resoU1 && _pasuv[1] < _resoV1 && _pasuv[2] < _resoU2 && _pasuv[3] < _resoV2)
                {
                    return (StatusDeflection.ArretSurPointPrecedent);
                }
                else
                {
                    _pasuv[0] *= 0.5;
                    _pasuv[1] *= 0.5;
                    _pasuv[2] *= 0.5;
                    _pasuv[3] *= 0.5;
                    return (StatusDeflection.PasTropGrand);
                }
            }
            Direction2D tg2dcourante1 = _intersectionOn2S.DirectionOnS1();
            Direction2D tg2dcourante2 = _intersectionOn2S.DirectionOnS2();
            cosi1 = du1 * tg2dcourante1.X + dv1 * tg2dcourante1.Y;
            cosi2 = du2 * tg2dcourante2.X + dv2 * tg2dcourante2.Y;
            ang1 = Math.Abs(_previousd1.AngleTo(tg2dcourante1));
            ang2 = Math.Abs(_previousd2.AngleTo(tg2dcourante2));
            angRef1 = AngRef2D * tolCoeff1;
            angRef2 = AngRef2D * tolCoeff2;
            //-------------------------------------------------------
            //-- Test : Angle too great in space UV       -----
            //--        Change of  side                      -----
            //-------------------------------------------------------
            if (cosi1 * cosi1 < cosRef1 * duv1 || cosi2 * cosi2 < cosRef2 * duv2 || ang1 > angRef1
                || ang2 > angRef2)
            {
                _pasuv[0] *= 0.5;
                _pasuv[1] *= 0.5;
                _pasuv[2] *= 0.5;
                _pasuv[3] *= 0.5;
                if (_pasuv[0] < _resoU1 && _pasuv[1] < _resoV1 && _pasuv[2] < _resoU2 && _pasuv[3] < _resoV2)
                {
                    return (StatusDeflection.ArretSurPoint);
                }
                else
                {
                    return (StatusDeflection.PasTropGrand);
                }
            }
        }
        //<-OCC431(apo)
        //==================================================================================
        //== D e t e c t i o n   o f    :  Step Too Small
        //==                               STEP TOO Great
        //==================================================================================

        //---------------------------------------
        //-- Estimate of the vector           --
        //---------------------------------------
        flecheCourante =
            Math.Sqrt(Math.Abs((_previousd.XYZ() - tgCourante.XYZ()).SquareModulus() * sqDist)) / 8.0;

        if (flecheCourante <= _fleche * 0.5)
        { //-- Current step too small
            if (flecheCourante > 1e-16)
            {
                ratio = 0.5 * (_fleche / flecheCourante);
            }
            else
            {
                ratio = 10.0;
            }
            double pasSu1 = _pasuv[0];
            double pasSv1 = _pasuv[1];
            double pasSu2 = _pasuv[2];
            double pasSv2 = _pasuv[3];

            //-- In  case if
            //-- a point at U+DeltaU is required, ....
            //-- return a point at U + Epsilon
            //-- Epsilon << DeltaU.

            if (_pasuv[0] < absDu1)
            {
                _pasuv[0] = absDu1;
            }
            if (_pasuv[1] < absDv1)
            {
                _pasuv[1] = absDv1;
            }
            if (_pasuv[2] < absDu2)
            {
                _pasuv[2] = absDu2;
            }
            if (_pasuv[3] < absDv2)
            {
                _pasuv[3] = absDv2;
            }

            if (_pasuv[0] < _resoU1)
            {
                _pasuv[0] = _resoU1;
            }
            if (_pasuv[1] < _resoV1)
            {
                _pasuv[1] = _resoV1;
            }
            if (_pasuv[2] < _resoU2)
            {
                _pasuv[2] = _resoU2;
            }
            if (_pasuv[3] < _resoV2)
            {
                _pasuv[3] = _resoV2;
            }
            //-- if(Ratio>10.0 ) { Ratio=10.0; }
            double r1, r = _pasInit[0] / _pasuv[0];
            r1 = _pasInit[1] / _pasuv[1];
            if (r1 < r)
            {
                r = r1;
            }
            r1 = _pasInit[2] / _pasuv[2];
            if (r1 < r)
            {
                r = r1;
            }
            r1 = _pasInit[3] / _pasuv[3];
            if (r1 < r)
            {
                r = r1;
            }
            if (ratio > r)
            {
                ratio = r;
            }
            _pasuv[0] = Math.Min(ratio * _pasuv[0], _pasInit[0]);
            _pasuv[1] = Math.Min(ratio * _pasuv[1], _pasInit[1]);
            _pasuv[2] = Math.Min(ratio * _pasuv[2], _pasInit[2]);
            _pasuv[3] = Math.Min(ratio * _pasuv[3], _pasInit[3]);
            if (_pasuv[0] != pasSu1 || _pasuv[2] != pasSu2 || _pasuv[1] != pasSv1 || _pasuv[3] != pasSv2)
            {
                if (++STATIC_BLOCAGE_SUR_PAS_TROP_GRAND > 5)
                {
                    STATIC_BLOCAGE_SUR_PAS_TROP_GRAND = 0;
                    return StatusDeflection.PasTropGrand;
                }
            }
            if (aStatus == StatusDeflection.OK)
            {
                STATIC_BLOCAGE_SUR_PAS_TROP_GRAND = 0;
                //-- Try to increase the step
            }
            return aStatus;
        }
        else
        { //-- CurrentVector > vector*0.5
            if (flecheCourante > _fleche)
            { //-- Current step too Great
                ratio = _fleche / flecheCourante;
                _pasuv[0] = ratio * _pasuv[0];
                _pasuv[1] = ratio * _pasuv[1];
                _pasuv[2] = ratio * _pasuv[2];
                _pasuv[3] = ratio * _pasuv[3];
                return StatusDeflection.PasTropGrand;
            }
            else
            { //-- vector/2  <  CurrentVector <= vector
                ratio = 0.75 * (_fleche / flecheCourante);
            }
        }

        if (aStatus != StatusDeflection.PointConfondu)
        {
            // Here, aCosBetweenTangent >= 0.0 definitely.
            //
            // OCCT's comment derives the deflection e between the arc of the osculating circle at the two points
            // and their chord: e = d*(1-sin(B/2))/(2*cos(B/2)), B the angle between the tangents, and compares
            // sin(B/2) = Fs(e) with Fs(tolconf) and Fs(tolconf/2), through cos(B) = 1 - 2*Fs(e)^2 = Fd(e), where
            // Fd(e) is -aCosBetweenTangent.

            double invSqAbsArcDeflMax = 0.25 * sqDist / (_tolconf * _tolconf);
            double sinB2Max = 1.0 - 2.0 / (1.0 + invSqAbsArcDeflMax);

            if (sinB2Max >= 0.0 && (cosBetweenTangent <= 2.0 * sinB2Max * sinB2Max - 1.0))
            { // Real deflection is greater or equal than tolconf
                aStatus = StatusDeflection.PasTropGrand;
            }
            else
            { // Real deflection is less than tolconf
                double invSqAbsArcDeflMin = 4.0 * invSqAbsArcDeflMax;
                double sinB2Min = 1.0 - 2.0 / (1.0 + invSqAbsArcDeflMin);

                if (status != StatusDeflection.PasTropGrand
                    && ((sinB2Min < 0.0) || (cosBetweenTangent >= 2.0 * sinB2Min * sinB2Min - 1.0)))
                { // Real deflection is less than tolconf/2.0
                    aStatus = StatusDeflection.StepTooSmall;
                }
            }

            if (aStatus == StatusDeflection.PasTropGrand)
            {
                _pasuv[0] *= 0.5;
                _pasuv[1] *= 0.5;
                _pasuv[2] *= 0.5;
                _pasuv[3] *= 0.5;
                return aStatus;
            }

            if (aStatus == StatusDeflection.StepTooSmall)
            {
                _pasuv[0] = Math.Max(_pasuv[0], absDu1);
                _pasuv[1] = Math.Max(_pasuv[1], absDv1);
                _pasuv[2] = Math.Max(_pasuv[2], absDu2);
                _pasuv[3] = Math.Max(_pasuv[3], absDv2);

                _pasInit[0] = Math.Max(_pasInit[0], absDu1);
                _pasInit[1] = Math.Max(_pasInit[1], absDv1);
                _pasInit[2] = Math.Max(_pasInit[2], absDu2);
                _pasInit[3] = Math.Max(_pasInit[3], absDv2);

                return aStatus;
            }
        }

        _pasuv[0] = Math.Max(_stepMin[0], Math.Min(Math.Min(ratio * absDu1, _pasuv[0]), _pasInit[0]));
        _pasuv[1] = Math.Max(_stepMin[1], Math.Min(Math.Min(ratio * absDv1, _pasuv[1]), _pasInit[1]));
        _pasuv[2] = Math.Max(_stepMin[2], Math.Min(Math.Min(ratio * absDu2, _pasuv[2]), _pasInit[2]));
        _pasuv[3] = Math.Max(_stepMin[3], Math.Min(Math.Min(ratio * absDv2, _pasuv[3]), _pasInit[3]));

        if (aStatus == StatusDeflection.OK)
        {
            STATIC_BLOCAGE_SUR_PAS_TROP_GRAND = 0;
        }
        return aStatus;
    }

    /// <summary>
    /// Tests whether the point set by Param stays in the natural domain of each surface: if it leaves, reframes it to
    /// the best iso (border); otherwise tests whether the line is closed.
    /// </summary>
    public bool TestArret(bool dejaReparti, NCollection_Array1<double> param, ref ConstIsoparametric choixIso)
    {
        Span<double> uvd = stackalloc double[4];
        Span<double> uvf = stackalloc double[4];
        Span<double> epsuv = stackalloc double[4];
        Span<double> duv = stackalloc double[4];
        Span<double> uvp = stackalloc double[4];
        Span<double> parC = stackalloc double[4];
        Span<double> solParam = stackalloc double[4];
        double dv, dv2;
        double dPc, dPb;
        int i = 0, k = 0;
        epsuv[0] = _resoU1;
        epsuv[1] = _resoV1;
        epsuv[2] = _resoU2;
        epsuv[3] = _resoV2;
        _previousPoint.Parameters(out uvp[0], out uvp[1], out uvp[2], out uvp[3]);

        _intersectionOn2S.Point().Parameters(out solParam[0], out solParam[1], out solParam[2], out solParam[3]);

        bool trouve = false;

        uvd[0] = _uMin1;
        uvf[0] = _uMax1;
        uvd[1] = _vMin1;
        uvf[1] = _vMax1;
        uvd[2] = _uMin2;
        uvf[2] = _uMax2;
        uvd[3] = _vMin2;
        uvf[3] = _vMax2;

        int im1;
        for (i = 1, im1 = 0; i <= 4; i++, im1++)
        {
            switch (i)
            {
                case 1:
                    k = 2;
                    break;
                case 2:
                    k = 1;
                    break;
                case 3:
                    k = 4;
                    break;
                case 4:
                    k = 3;
                    break;
            }
            if (param[i] < (uvd[im1] - epsuv[im1])
                || solParam[im1] < (uvd[im1] - epsuv[im1])) //--     Current -----  Bound Inf ----- Previous
            {
                trouve = true; //--
                dPc = uvp[im1] - param[i]; //--     Previous  - Current
                dPb = uvp[im1] - uvd[im1]; //--     Previous  - Bound Inf
                parC[im1] = uvd[im1]; //--     ParamCorrige
                dv = param[k] - uvp[k - 1]; //--     Current   - Previous (other Direction)
                dv2 = dv * dv;
                if (dv2 > RealEpsilon())
                { //--    Progress at the other Direction ?
                    duv[im1] = dPc * dPb + dv2;
                    duv[im1] = duv[im1] * duv[im1] / (dPc * dPc + dv2) / (dPb * dPb + dv2);
                }
                else
                {
                    duv[im1] = -1.0; //--    If no progress, do not change
                } //--    the choice of iso
            }
            else if (param[i] > (uvf[im1] + epsuv[im1])
                     || solParam[im1] > (uvf[im1] + epsuv[im1])) //--    Previous -----  Bound Sup -----  Current
            {
                trouve = true; //--
                dPc = param[i] - uvp[im1]; //--     Current   - Previous
                dPb = uvf[im1] - uvp[im1]; //--     Bound Sup - Previous
                parC[im1] = uvf[im1]; //--     Param Corrige
                dv = param[k] - uvp[k - 1]; //--     Current   - Previous (other Direction)
                dv2 = dv * dv;
                if (dv2 > RealEpsilon())
                { //--     Progress in other Direction ?
                    duv[im1] = dPc * dPb + dv2;
                    duv[im1] = duv[im1] * duv[im1] / (dPc * dPc + dv2) / (dPb * dPb + dv2);
                }
                else
                {
                    duv[im1] = -1.0; //--    If no progress, do not change
                } //--    the choice of iso
            }
            else
            {
                duv[im1] = -1.0;
                parC[im1] = param[i];
            }
        }

        if (trouve)
        {
            //--------------------------------------------------
            //-- One of Parameters u1,v1,u2,v2 is outside of  --
            //-- the natural limits.                          --
            //-- Find the best direction of                   --
            //-- progress and reframe the parameters.        --
            //--------------------------------------------------
            double ddv = -1.0;
            k = -1;
            for (i = 0; i <= 3; i++)
            {
                param[i + 1] = parC[i];
                if (duv[i] > ddv)
                {
                    ddv = duv[i];
                    k = i;
                }
            }
            if (k != -1)
            {
                choixIso = ChoixRef(k);
            }
            else
            {
                if ((parC[0] <= uvd[0] + epsuv[0]) || (parC[0] >= uvf[0] - epsuv[0]))
                {
                    choixIso = ConstIsoparametric.UIsoparametricOnCaro1;
                }
                else if ((parC[1] <= uvd[1] + epsuv[1]) || (parC[1] >= uvf[1] - epsuv[1]))
                {
                    choixIso = ConstIsoparametric.VIsoparametricOnCaro1;
                }
                else if ((parC[2] <= uvd[2] + epsuv[2]) || (parC[2] >= uvf[2] - epsuv[2]))
                {
                    choixIso = ConstIsoparametric.UIsoparametricOnCaro2;
                }
                else if ((parC[3] <= uvd[3] + epsuv[3]) || (parC[3] >= uvf[3] - epsuv[3]))
                {
                    choixIso = ConstIsoparametric.VIsoparametricOnCaro2;
                }
            }
            _close = false;
            return true;
        }
        else
        {
            if (!dejaReparti)
            { // find if line closed
                double u, v;
                PntOn2S pOn2S1 = _line.Value(1);
                // On S1
                pOn2S1.ParametersOnS1(out u, out v);
                Point2D p1uvS1 = new(u, v);
                _previousPoint.ParametersOnS1(out u, out v);
                Point2D prevuvS1 = new(u, v);
                _intersectionOn2S.Point().ParametersOnS1(out u, out v);
                Point2D myIntersuvS1 = new(u, v);
                bool close2dS1 = (p1uvS1.XY() - prevuvS1.XY()).Dot(p1uvS1.XY() - myIntersuvS1.XY()) < 0.0;
                // On S2
                pOn2S1.ParametersOnS2(out u, out v);
                Point2D p1uvS2 = new(u, v);
                _previousPoint.ParametersOnS2(out u, out v);
                Point2D prevuvS2 = new(u, v);
                _intersectionOn2S.Point().ParametersOnS2(out u, out v);
                Point2D myIntersuvS2 = new(u, v);
                bool close2dS2 = (p1uvS2.XY() - prevuvS2.XY()).Dot(p1uvS2.XY() - myIntersuvS2.XY()) < 0.0;

                _close = close2dS1 && close2dS2;
                return _close;
            }
            else
            {
                return false;
            }
        }
    }
}
