// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.IntImp_ComputeTangence;
using static Occt.IntImp_ConstIsoparametric;
using static Occt.IntWalk_StatusDeflection;

namespace Occt;

public partial class IntWalk_PWalking
{
    // OCC431(apo): modified ->
    private static readonly double CosRef2D = Math.Cos(M_PI / 9.0);
    private const double AngRef2D = M_PI / 2.0;
    private const double d = 7.0;

    /// <summary>
    /// Near a point where the marching fails: divides the steps to try to continue; if the step is too small (on a
    /// border), restarts in the other direction if not done yet, otherwise stops.
    /// </summary>
    public void RepartirOuDiviser(ref bool DejaReparti, ref IntImp_ConstIsoparametric ChoixIso, ref bool Arrive)
    {
        if (Arrive)
        { // restart in the other direction
            if (!DejaReparti)
            {
                Arrive = false;
                DejaReparti = true;
                previousPoint = line.Value(1);
                previoustg = false;
                previousd1 = firstd1;
                previousd2 = firstd2;
                previousd = tgdir;
                myTangentIdx = line.NbPoints();
                tgdir.Reverse();
                line.Reverse();

                sensCheminement = -1;
                tgfirst = tglast;
                tglast = false;
                ChoixIso = choixIsoSav;
                int nn = line.NbPoints();
                if (nn > 2)
                {
                    line.Value(nn).Parameters(out double u1, out double v1, out double u2, out double v2);
                    line.Value(nn - 1).Parameters(out double U1, out double V1, out double U2, out double V2);
                    pasuv[0] = Math.Abs(u1 - U1);
                    pasuv[1] = Math.Abs(v1 - V1);
                    pasuv[2] = Math.Abs(u2 - U2);
                    pasuv[3] = Math.Abs(v2 - V2);
                }
            }
        }
        else
        {
            if (pasuv[0] * 0.5 < ResoU1 && pasuv[1] * 0.5 < ResoV1 && pasuv[2] * 0.5 < ResoU2
                && pasuv[3] * 0.5 < ResoV2)
            {
                if (!previoustg)
                {
                    tglast = true; // IS IT ENOUGH ????
                }

                if (!DejaReparti)
                { // restart in the other direction
                    DejaReparti = true;
                    previousPoint = line.Value(1);
                    previoustg = false;
                    previousd1 = firstd1;
                    previousd2 = firstd2;
                    previousd = tgdir;
                    myTangentIdx = line.NbPoints();
                    tgdir.Reverse();
                    line.Reverse();

                    sensCheminement = -1;
                    tgfirst = tglast;
                    tglast = false;
                    ChoixIso = choixIsoSav;

                    int nn = line.NbPoints();
                    if (nn > 2)
                    {
                        line.Value(nn).Parameters(out double u1, out double v1, out double u2, out double v2);
                        line.Value(nn - 1).Parameters(out double U1, out double V1, out double U2, out double V2);
                        pasuv[0] = Math.Abs(u1 - U1);
                        pasuv[1] = Math.Abs(v1 - V1);
                        pasuv[2] = Math.Abs(u2 - U2);
                        pasuv[3] = Math.Abs(v2 - V2);
                    }
                }
                else
                {
                    Arrive = true;
                }
            }
            else
            {
                pasuv[0] *= 0.5;
                pasuv[1] *= 0.5;
                pasuv[2] *= 0.5;
                pasuv[3] *= 0.5;
            }
        }
    }

    /// <summary>
    /// Tests the step to the current point of the intersector: the deflection between the previous and the current
    /// point (from their tangents), the angles in the parametric spaces and the change of side; adjusts the steps
    /// and returns the verdict. Calls <see cref="StepTrace"/> with it when set.
    /// </summary>
    public IntWalk_StatusDeflection TestDeflection(IntImp_ConstIsoparametric choixIso,
                                                   IntWalk_StatusDeflection theStatus)
    {
        IntWalk_StatusDeflection aResult = TestDeflectionImpl(choixIso, theStatus);
        (StepTrace ?? ThreadStepTrace)?.Invoke(aResult, myIntersectionOn2S.Point());
        return aResult;
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
    private IntWalk_StatusDeflection TestDeflectionImpl(IntImp_ConstIsoparametric choixIso,
                                                        IntWalk_StatusDeflection theStatus)
    {
        if (line.NbPoints() == 1)
        {
            STATIC_BLOCAGE_SUR_PAS_TROP_GRAND = STATIC_PRECEDENT_INFLEXION = 0;
        }

        IntWalk_StatusDeflection aStatus = IntWalk_OK;
        double FlecheCourante, Ratio = 1.0;

        // Caro1 and Caro2
        Adaptor3d_Surface Caro1 = myIntersectionOn2S.Function().AuxillarSurface1();
        Adaptor3d_Surface Caro2 = myIntersectionOn2S.Function().AuxillarSurface2();

        IntSurf_PntOn2S CurrentPoint = myIntersectionOn2S.Point();
        //==================================================================================
        //=========               S t o p   o n   p o i n t                 ============
        //==================================================================================
        if (myIntersectionOn2S.IsTangent())
        {
            return IntWalk_ArretSurPoint;
        }

        Direction3D TgCourante = myIntersectionOn2S.Direction();

        double aCosBetweenTangent = TgCourante.Dot(previousd);

        //==================================================================================
        //=========   R i s k   o f    i n f l e x i o n   p o i n t  ============
        //==================================================================================
        if (aCosBetweenTangent < 0)
        {
            //------------------------------------------------------------
            //-- Risk of inflexion point : Divide the step by 2
            //-- Initialize STATIC_PRECEDENT_INFLEXION so that
            //-- at the next call to return Pas_OK if there is no
            //-- more risk of the point of inflexion
            //------------------------------------------------------------

            pasuv[0] *= 0.5;
            pasuv[1] *= 0.5;
            pasuv[2] *= 0.5;
            pasuv[3] *= 0.5;
            STATIC_PRECEDENT_INFLEXION += 3;
            if (pasuv[0] < ResoU1 && pasuv[1] < ResoV1 && pasuv[2] < ResoU2 && pasuv[3] < ResoV2)
            {
                return IntWalk_ArretSurPointPrecedent;
            }
            else
            {
                return IntWalk_PasTropGrand;
            }
        }
        else
        {
            if (STATIC_PRECEDENT_INFLEXION > 0)
            {
                STATIC_PRECEDENT_INFLEXION--;
                return IntWalk_OK;
            }
        }

        //==================================================================================
        //=========  D e t e c t    c o n f u s e d    P o in t s       ===========
        //==================================================================================

        double aSqDist = previousPoint.Value().SquareDistance(CurrentPoint.Value());

        if (aSqDist < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            pasInit[0] = Math.Max(pasInit[0], 5.0 * ResoU1);
            pasInit[1] = Math.Max(pasInit[1], 5.0 * ResoV1);
            pasInit[2] = Math.Max(pasInit[2], 5.0 * ResoU2);
            pasInit[3] = Math.Max(pasInit[3], 5.0 * ResoV2);

            for (int i = 0; i < 4; i++)
            {
                pasuv[i] = Math.Max(pasuv[i], Math.Min(1.5 * pasuv[i], pasInit[i]));
            }
            // Compute local resolution: for OCC26717
            int iIso = (int)choixIso;
            if (Math.Abs(pasuv[iIso] - pasInit[iIso]) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                double CurU, CurV;
                if (choixIso == IntImp_UIsoparametricOnCaro1 || choixIso == IntImp_VIsoparametricOnCaro1)
                {
                    previousPoint.ParametersOnS1(out CurU, out CurV);
                }
                else
                {
                    previousPoint.ParametersOnS2(out CurU, out CurV);
                }
                Point3D CurPnt = (choixIso == IntImp_UIsoparametricOnCaro1 || choixIso == IntImp_VIsoparametricOnCaro1)
                                    ? Adaptor3d_HSurfaceTool.Value(Caro1, CurU, CurV)
                                    : Adaptor3d_HSurfaceTool.Value(Caro2, CurU, CurV);
                Point3D OffsetPnt = new();
                switch (choixIso)
                {
                    case IntImp_UIsoparametricOnCaro1:
                        OffsetPnt = Adaptor3d_HSurfaceTool.Value(Caro1, CurU + sensCheminement * pasuv[0], CurV);
                        break;
                    case IntImp_VIsoparametricOnCaro1:
                        OffsetPnt = Adaptor3d_HSurfaceTool.Value(Caro1, CurU, CurV + sensCheminement * pasuv[1]);
                        break;
                    case IntImp_UIsoparametricOnCaro2:
                        OffsetPnt = Adaptor3d_HSurfaceTool.Value(Caro2, CurU + sensCheminement * pasuv[2], CurV);
                        break;
                    case IntImp_VIsoparametricOnCaro2:
                        OffsetPnt = Adaptor3d_HSurfaceTool.Value(Caro2, CurU, CurV + sensCheminement * pasuv[3]);
                        break;
                    default:
                        break;
                }
                double RefDist = CurPnt.Distance(OffsetPnt);
                double LocalResol = 0.0;
                if (RefDist > Constants.Resolution())
                {
                    LocalResol = pasuv[iIso] * tolconf / RefDist;
                }
                if (pasuv[iIso] < 2 * LocalResol)
                {
                    pasuv[iIso] = pasInit[iIso] = 2 * LocalResol;
                }
            }
            ////////////////////////////////////////
            aStatus = IntWalk_PointConfondu;
        }

        //==================================================================================
        double Du1, Dv1, AbsDu1, AbsDu2, AbsDv1, AbsDv2;
        double Du2, Dv2;

        previousPoint.Parameters(out double Up1, out double Vp1, out double Up2, out double Vp2);
        CurrentPoint.Parameters(out double Uc1, out double Vc1, out double Uc2, out double Vc2);

        Du1 = Uc1 - Up1;
        Dv1 = Vc1 - Vp1;
        Du2 = Uc2 - Up2;
        Dv2 = Vc2 - Vp2;

        AbsDu1 = Math.Abs(Du1);
        AbsDu2 = Math.Abs(Du2);
        AbsDv1 = Math.Abs(Dv1);
        AbsDv2 = Math.Abs(Dv2);
        //=================================================================================
        //====   S t e p   o f   p  r o g r e s s i o n (between previous and Current)   =======
        //=================================================================================
        if (AbsDu1 < ResoU1 && AbsDv1 < ResoV1 && AbsDu2 < ResoU2 && AbsDv2 < ResoV2)
        {
            pasuv[0] = ResoU1;
            pasuv[1] = ResoV1;
            pasuv[2] = ResoU2;
            pasuv[3] = ResoV2;
            return (IntWalk_ArretSurPointPrecedent);
        }
        //==================================================================================

        double tolArea = 100.0;
        if (ResoU1 < global::Kerf.Engine.Foundation.Precision.PConfusion() || ResoV1 < global::Kerf.Engine.Foundation.Precision.PConfusion()
            || ResoU2 < global::Kerf.Engine.Foundation.Precision.PConfusion() || ResoV2 < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            tolArea = tolArea * 2.0;
        }

        double Cosi1, CosRef1, Ang1, AngRef1, ResoUV1, Duv1, d1, tolCoeff1;
        double Cosi2, CosRef2, Ang2, AngRef2, ResoUV2, Duv2, d2, tolCoeff2;
        Cosi1 = Du1 * previousd1.X() + Dv1 * previousd1.Y();
        Cosi2 = Du2 * previousd2.X() + Dv2 * previousd2.Y();
        Duv1 = Du1 * Du1 + Dv1 * Dv1;
        Duv2 = Du2 * Du2 + Dv2 * Dv2;
        ResoUV1 = ResoU1 * ResoU1 + ResoV1 * ResoV1;
        ResoUV2 = ResoU2 * ResoU2 + ResoV2 * ResoV2;
        //
        // modified by NIZNHY-PKV Wed Nov 13 12:25:44 2002 f
        //
        double aMinDiv2 = global::Kerf.Engine.Foundation.Precision.Confusion();
        aMinDiv2 = aMinDiv2 * aMinDiv2;
        //
        d1 = d;
        if (Duv1 > aMinDiv2)
        {
            d1 = Math.Abs(ResoUV1 / Duv1);
            d1 = Math.Min(Math.Sqrt(d1) * tolArea, d);
        }
        tolCoeff1 = Math.Exp(d1);
        //
        // modified by NIZNHY-PKV Wed Nov 13 12:34:43 2002 f
        d2 = d;
        if (Duv2 > aMinDiv2)
        {
            d2 = Math.Abs(ResoUV2 / Duv2);
            d2 = Math.Min(Math.Sqrt(d2) * tolArea, d);
        }
        tolCoeff2 = Math.Exp(d2);
        CosRef1 = CosRef2D / tolCoeff1;
        CosRef2 = CosRef2D / tolCoeff2;
        //
        //==================================================================================
        //== The points are not confused :                                           ==
        //== D e t e c t    t h e   S t o p   a  t   p r e v i o u s  p o i n t ==
        //==                           N o t    T o o    G r e a t (angle in space UV)    ==
        //==                           C h a n g e    o f    s i d e                ==
        //==================================================================================
        if (aStatus != IntWalk_PointConfondu)
        {
            if (Cosi1 * Cosi1 < CosRef1 * Duv1 || Cosi2 * Cosi2 < CosRef2 * Duv2)
            {
                pasuv[0] *= 0.5;
                pasuv[1] *= 0.5;
                pasuv[2] *= 0.5;
                pasuv[3] *= 0.5;
                if (pasuv[0] < ResoU1 && pasuv[1] < ResoV1 && pasuv[2] < ResoU2 && pasuv[3] < ResoV2)
                {
                    return (IntWalk_ArretSurPointPrecedent);
                }
                else
                {
                    pasuv[0] *= 0.5;
                    pasuv[1] *= 0.5;
                    pasuv[2] *= 0.5;
                    pasuv[3] *= 0.5;
                    return (IntWalk_PasTropGrand);
                }
            }
            Direction2D Tg2dcourante1 = myIntersectionOn2S.DirectionOnS1();
            Direction2D Tg2dcourante2 = myIntersectionOn2S.DirectionOnS2();
            Cosi1 = Du1 * Tg2dcourante1.X() + Dv1 * Tg2dcourante1.Y();
            Cosi2 = Du2 * Tg2dcourante2.X() + Dv2 * Tg2dcourante2.Y();
            Ang1 = Math.Abs(previousd1.Angle(Tg2dcourante1));
            Ang2 = Math.Abs(previousd2.Angle(Tg2dcourante2));
            AngRef1 = AngRef2D * tolCoeff1;
            AngRef2 = AngRef2D * tolCoeff2;
            //-------------------------------------------------------
            //-- Test : Angle too great in space UV       -----
            //--        Change of  side                      -----
            //-------------------------------------------------------
            if (Cosi1 * Cosi1 < CosRef1 * Duv1 || Cosi2 * Cosi2 < CosRef2 * Duv2 || Ang1 > AngRef1
                || Ang2 > AngRef2)
            {
                pasuv[0] *= 0.5;
                pasuv[1] *= 0.5;
                pasuv[2] *= 0.5;
                pasuv[3] *= 0.5;
                if (pasuv[0] < ResoU1 && pasuv[1] < ResoV1 && pasuv[2] < ResoU2 && pasuv[3] < ResoV2)
                {
                    return (IntWalk_ArretSurPoint);
                }
                else
                {
                    return (IntWalk_PasTropGrand);
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
        FlecheCourante =
            Math.Sqrt(Math.Abs((previousd.XYZ() - TgCourante.XYZ()).SquareModulus() * aSqDist)) / 8.0;

        if (FlecheCourante <= fleche * 0.5)
        { //-- Current step too small
            if (FlecheCourante > 1e-16)
            {
                Ratio = 0.5 * (fleche / FlecheCourante);
            }
            else
            {
                Ratio = 10.0;
            }
            double pasSu1 = pasuv[0];
            double pasSv1 = pasuv[1];
            double pasSu2 = pasuv[2];
            double pasSv2 = pasuv[3];

            //-- In  case if
            //-- a point at U+DeltaU is required, ....
            //-- return a point at U + Epsilon
            //-- Epsilon << DeltaU.

            if (pasuv[0] < AbsDu1)
            {
                pasuv[0] = AbsDu1;
            }
            if (pasuv[1] < AbsDv1)
            {
                pasuv[1] = AbsDv1;
            }
            if (pasuv[2] < AbsDu2)
            {
                pasuv[2] = AbsDu2;
            }
            if (pasuv[3] < AbsDv2)
            {
                pasuv[3] = AbsDv2;
            }

            if (pasuv[0] < ResoU1)
            {
                pasuv[0] = ResoU1;
            }
            if (pasuv[1] < ResoV1)
            {
                pasuv[1] = ResoV1;
            }
            if (pasuv[2] < ResoU2)
            {
                pasuv[2] = ResoU2;
            }
            if (pasuv[3] < ResoV2)
            {
                pasuv[3] = ResoV2;
            }
            //-- if(Ratio>10.0 ) { Ratio=10.0; }
            double R1, R = pasInit[0] / pasuv[0];
            R1 = pasInit[1] / pasuv[1];
            if (R1 < R)
            {
                R = R1;
            }
            R1 = pasInit[2] / pasuv[2];
            if (R1 < R)
            {
                R = R1;
            }
            R1 = pasInit[3] / pasuv[3];
            if (R1 < R)
            {
                R = R1;
            }
            if (Ratio > R)
            {
                Ratio = R;
            }
            pasuv[0] = Math.Min(Ratio * pasuv[0], pasInit[0]);
            pasuv[1] = Math.Min(Ratio * pasuv[1], pasInit[1]);
            pasuv[2] = Math.Min(Ratio * pasuv[2], pasInit[2]);
            pasuv[3] = Math.Min(Ratio * pasuv[3], pasInit[3]);
            if (pasuv[0] != pasSu1 || pasuv[2] != pasSu2 || pasuv[1] != pasSv1 || pasuv[3] != pasSv2)
            {
                if (++STATIC_BLOCAGE_SUR_PAS_TROP_GRAND > 5)
                {
                    STATIC_BLOCAGE_SUR_PAS_TROP_GRAND = 0;
                    return IntWalk_PasTropGrand;
                }
            }
            if (aStatus == IntWalk_OK)
            {
                STATIC_BLOCAGE_SUR_PAS_TROP_GRAND = 0;
                //-- Try to increase the step
            }
            return aStatus;
        }
        else
        { //-- CurrentVector > vector*0.5
            if (FlecheCourante > fleche)
            { //-- Current step too Great
                Ratio = fleche / FlecheCourante;
                pasuv[0] = Ratio * pasuv[0];
                pasuv[1] = Ratio * pasuv[1];
                pasuv[2] = Ratio * pasuv[2];
                pasuv[3] = Ratio * pasuv[3];
                return IntWalk_PasTropGrand;
            }
            else
            { //-- vector/2  <  CurrentVector <= vector
                Ratio = 0.75 * (fleche / FlecheCourante);
            }
        }

        if (aStatus != IntWalk_PointConfondu)
        {
            // Here, aCosBetweenTangent >= 0.0 definitely.
            //
            // OCCT's comment derives the deflection e between the arc of the osculating circle at the two points
            // and their chord: e = d*(1-sin(B/2))/(2*cos(B/2)), B the angle between the tangents, and compares
            // sin(B/2) = Fs(e) with Fs(tolconf) and Fs(tolconf/2), through cos(B) = 1 - 2*Fs(e)^2 = Fd(e), where
            // Fd(e) is -aCosBetweenTangent.

            double anInvSqAbsArcDeflMax = 0.25 * aSqDist / (tolconf * tolconf);
            double aSinB2Max = 1.0 - 2.0 / (1.0 + anInvSqAbsArcDeflMax);

            if (aSinB2Max >= 0.0 && (aCosBetweenTangent <= 2.0 * aSinB2Max * aSinB2Max - 1.0))
            { // Real deflection is greater or equal than tolconf
                aStatus = IntWalk_PasTropGrand;
            }
            else
            { // Real deflection is less than tolconf
                double anInvSqAbsArcDeflMin = 4.0 * anInvSqAbsArcDeflMax;
                double aSinB2Min = 1.0 - 2.0 / (1.0 + anInvSqAbsArcDeflMin);

                if (theStatus != IntWalk_PasTropGrand
                    && ((aSinB2Min < 0.0) || (aCosBetweenTangent >= 2.0 * aSinB2Min * aSinB2Min - 1.0)))
                { // Real deflection is less than tolconf/2.0
                    aStatus = IntWalk_StepTooSmall;
                }
            }

            if (aStatus == IntWalk_PasTropGrand)
            {
                pasuv[0] *= 0.5;
                pasuv[1] *= 0.5;
                pasuv[2] *= 0.5;
                pasuv[3] *= 0.5;
                return aStatus;
            }

            if (aStatus == IntWalk_StepTooSmall)
            {
                pasuv[0] = Math.Max(pasuv[0], AbsDu1);
                pasuv[1] = Math.Max(pasuv[1], AbsDv1);
                pasuv[2] = Math.Max(pasuv[2], AbsDu2);
                pasuv[3] = Math.Max(pasuv[3], AbsDv2);

                pasInit[0] = Math.Max(pasInit[0], AbsDu1);
                pasInit[1] = Math.Max(pasInit[1], AbsDv1);
                pasInit[2] = Math.Max(pasInit[2], AbsDu2);
                pasInit[3] = Math.Max(pasInit[3], AbsDv2);

                return aStatus;
            }
        }

        pasuv[0] = Math.Max(myStepMin[0], Math.Min(Math.Min(Ratio * AbsDu1, pasuv[0]), pasInit[0]));
        pasuv[1] = Math.Max(myStepMin[1], Math.Min(Math.Min(Ratio * AbsDv1, pasuv[1]), pasInit[1]));
        pasuv[2] = Math.Max(myStepMin[2], Math.Min(Math.Min(Ratio * AbsDu2, pasuv[2]), pasInit[2]));
        pasuv[3] = Math.Max(myStepMin[3], Math.Min(Math.Min(Ratio * AbsDv2, pasuv[3]), pasInit[3]));

        if (aStatus == IntWalk_OK)
        {
            STATIC_BLOCAGE_SUR_PAS_TROP_GRAND = 0;
        }
        return aStatus;
    }

    /// <summary>
    /// Tests whether the point set by Param stays in the natural domain of each surface: if it leaves, reframes it to
    /// the best iso (border); otherwise tests whether the line is closed.
    /// </summary>
    public bool TestArret(bool DejaReparti, NCollection_Array1<double> Param, ref IntImp_ConstIsoparametric ChoixIso)
    {
        Span<double> Uvd = stackalloc double[4];
        Span<double> Uvf = stackalloc double[4];
        Span<double> Epsuv = stackalloc double[4];
        Span<double> Duv = stackalloc double[4];
        Span<double> Uvp = stackalloc double[4];
        Span<double> ParC = stackalloc double[4];
        Span<double> SolParam = stackalloc double[4];
        double dv, dv2;
        double DPc, DPb;
        int i = 0, k = 0;
        Epsuv[0] = ResoU1;
        Epsuv[1] = ResoV1;
        Epsuv[2] = ResoU2;
        Epsuv[3] = ResoV2;
        previousPoint.Parameters(out Uvp[0], out Uvp[1], out Uvp[2], out Uvp[3]);

        myIntersectionOn2S.Point().Parameters(out SolParam[0], out SolParam[1], out SolParam[2], out SolParam[3]);

        bool Trouve = false;

        Uvd[0] = Um1;
        Uvf[0] = UM1;
        Uvd[1] = Vm1;
        Uvf[1] = VM1;
        Uvd[2] = Um2;
        Uvf[2] = UM2;
        Uvd[3] = Vm2;
        Uvf[3] = VM2;

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
            if (Param[i] < (Uvd[im1] - Epsuv[im1])
                || SolParam[im1] < (Uvd[im1] - Epsuv[im1])) //--     Current -----  Bound Inf ----- Previous
            {
                Trouve = true; //--
                DPc = Uvp[im1] - Param[i]; //--     Previous  - Current
                DPb = Uvp[im1] - Uvd[im1]; //--     Previous  - Bound Inf
                ParC[im1] = Uvd[im1]; //--     ParamCorrige
                dv = Param[k] - Uvp[k - 1]; //--     Current   - Previous (other Direction)
                dv2 = dv * dv;
                if (dv2 > RealEpsilon())
                { //--    Progress at the other Direction ?
                    Duv[im1] = DPc * DPb + dv2;
                    Duv[im1] = Duv[im1] * Duv[im1] / (DPc * DPc + dv2) / (DPb * DPb + dv2);
                }
                else
                {
                    Duv[im1] = -1.0; //--    If no progress, do not change
                } //--    the choice of iso
            }
            else if (Param[i] > (Uvf[im1] + Epsuv[im1])
                     || SolParam[im1] > (Uvf[im1] + Epsuv[im1])) //--    Previous -----  Bound Sup -----  Current
            {
                Trouve = true; //--
                DPc = Param[i] - Uvp[im1]; //--     Current   - Previous
                DPb = Uvf[im1] - Uvp[im1]; //--     Bound Sup - Previous
                ParC[im1] = Uvf[im1]; //--     Param Corrige
                dv = Param[k] - Uvp[k - 1]; //--     Current   - Previous (other Direction)
                dv2 = dv * dv;
                if (dv2 > RealEpsilon())
                { //--     Progress in other Direction ?
                    Duv[im1] = DPc * DPb + dv2;
                    Duv[im1] = Duv[im1] * Duv[im1] / (DPc * DPc + dv2) / (DPb * DPb + dv2);
                }
                else
                {
                    Duv[im1] = -1.0; //--    If no progress, do not change
                } //--    the choice of iso
            }
            else
            {
                Duv[im1] = -1.0;
                ParC[im1] = Param[i];
            }
        }

        if (Trouve)
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
                Param[i + 1] = ParC[i];
                if (Duv[i] > ddv)
                {
                    ddv = Duv[i];
                    k = i;
                }
            }
            if (k != -1)
            {
                ChoixIso = ChoixRef(k);
            }
            else
            {
                if ((ParC[0] <= Uvd[0] + Epsuv[0]) || (ParC[0] >= Uvf[0] - Epsuv[0]))
                {
                    ChoixIso = IntImp_UIsoparametricOnCaro1;
                }
                else if ((ParC[1] <= Uvd[1] + Epsuv[1]) || (ParC[1] >= Uvf[1] - Epsuv[1]))
                {
                    ChoixIso = IntImp_VIsoparametricOnCaro1;
                }
                else if ((ParC[2] <= Uvd[2] + Epsuv[2]) || (ParC[2] >= Uvf[2] - Epsuv[2]))
                {
                    ChoixIso = IntImp_UIsoparametricOnCaro2;
                }
                else if ((ParC[3] <= Uvd[3] + Epsuv[3]) || (ParC[3] >= Uvf[3] - Epsuv[3]))
                {
                    ChoixIso = IntImp_VIsoparametricOnCaro2;
                }
            }
            close = false;
            return true;
        }
        else
        {
            if (!DejaReparti)
            { // find if line closed
                double u, v;
                IntSurf_PntOn2S POn2S1 = line.Value(1);
                // On S1
                POn2S1.ParametersOnS1(out u, out v);
                Point2D P1uvS1 = new(u, v);
                previousPoint.ParametersOnS1(out u, out v);
                Point2D PrevuvS1 = new(u, v);
                myIntersectionOn2S.Point().ParametersOnS1(out u, out v);
                Point2D myIntersuvS1 = new(u, v);
                bool close2dS1 = (P1uvS1.XY() - PrevuvS1.XY()).Dot(P1uvS1.XY() - myIntersuvS1.XY()) < 0.0;
                // On S2
                POn2S1.ParametersOnS2(out u, out v);
                Point2D P1uvS2 = new(u, v);
                previousPoint.ParametersOnS2(out u, out v);
                Point2D PrevuvS2 = new(u, v);
                myIntersectionOn2S.Point().ParametersOnS2(out u, out v);
                Point2D myIntersuvS2 = new(u, v);
                bool close2dS2 = (P1uvS2.XY() - PrevuvS2.XY()).Dot(P1uvS2.XY() - myIntersuvS2.XY()) < 0.0;

                close = close2dS1 && close2dS2;
                return close;
            }
            else
            {
                return false;
            }
        }
    }
}
