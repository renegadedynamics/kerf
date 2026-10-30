// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntImp_ConstIsoparametric;
using static Occt.IntWalk_StatusDeflection;

namespace Occt;

public partial class IntWalk_PWalking
{
    /// <summary>
    /// Walks from ParDep (u1, v1, u2, v2); the box (u1min..v2max) of the expected line sets the initial steps.
    /// </summary>
    public void Perform(NCollection_Array1<double> ParDep, double u1min, double v1min, double u2min, double v2min,
                        double u1max, double v1max, double u2max, double v2max)
    {
        const double aSQDistMax = 1.0e-14;
        // xf

        int NbPasOKConseq = 0;
        NCollection_Array1<double> Param = new(1, 4);
        IntImp_ConstIsoparametric ChoixIso;
        // xt
        //
        done = false;
        //
        // Caro1 and Caro2
        Adaptor3d_Surface Caro1 = myIntersectionOn2S.Function().AuxillarSurface1();
        Adaptor3d_Surface Caro2 = myIntersectionOn2S.Function().AuxillarSurface2();
        //
        double UFirst1 = Adaptor3d_HSurfaceTool.FirstUParameter(Caro1);
        double VFirst1 = Adaptor3d_HSurfaceTool.FirstVParameter(Caro1);
        double ULast1 = Adaptor3d_HSurfaceTool.LastUParameter(Caro1);
        double VLast1 = Adaptor3d_HSurfaceTool.LastVParameter(Caro1);

        double UFirst2 = Adaptor3d_HSurfaceTool.FirstUParameter(Caro2);
        double VFirst2 = Adaptor3d_HSurfaceTool.FirstVParameter(Caro2);
        double ULast2 = Adaptor3d_HSurfaceTool.LastUParameter(Caro2);
        double VLast2 = Adaptor3d_HSurfaceTool.LastVParameter(Caro2);
        //
        ComputePasInit(u1max - u1min, v1max - v1min, u2max - u2min, v2max - v2min);

        for (int i = 0; i < 4; ++i)
        {
            if (pasuv[i] > 10)
            {
                pasuv[i] = 10;
            }

            pasInit[i] = pasSav[i] = pasuv[i];
        }
        //
        line = new IntSurf_LineOn2S();
        //
        for (int i = 1; i <= 4; ++i)
        {
            Param[i] = ParDep[i];
        }
        //-- reproduce steps uv connected to surfaces Caro1 and Caro2
        //-- pasuv[] and pasSav[] are modified during the marching
        for (int i = 0; i < 4; ++i)
        {
            pasSav[i] = pasuv[i] = pasInit[i];
        }

        //-- calculate the first solution point
        math_FunctionSetRoot Rsnld = new(myIntersectionOn2S.Function());
        //
        ChoixIso = myIntersectionOn2S.Perform(Param, Rsnld);
        if (!myIntersectionOn2S.IsDone())
        {
            return;
        }

        //
        if (myIntersectionOn2S.IsEmpty())
        {
            return;
        }
        //
        if (myIntersectionOn2S.IsTangent())
        {
            return;
        }
        //
        bool Arrive, DejaReparti;
        const int RejectIndexMAX = 250000;
        int IncKey, RejectIndex;
        gp_Pnt pf, pl;
        //
        DejaReparti = false;
        IncKey = 0;
        RejectIndex = 0;
        //
        previousPoint = myIntersectionOn2S.Point();
        previoustg = false;
        previousd = myIntersectionOn2S.Direction();
        previousd1 = myIntersectionOn2S.DirectionOnS1();
        previousd2 = myIntersectionOn2S.DirectionOnS2();
        myTangentIdx = 1;
        tgdir = previousd;
        firstd1 = previousd1;
        firstd2 = previousd2;
        tgfirst = tglast = false;
        choixIsoSav = ChoixIso;
        //------------------------------------------------------------
        //-- Test if the first point of marching corresponds
        //-- to a point on borders.
        //-- In this case, DejaReparti is initialized as True
        //--
        pf = previousPoint.Value();
        bool bTestFirstPoint = true;

        SetParameters(Param, previousPoint);

        if (IsTangentExtCheck(Caro1, Caro2, Param[1], Param[2], Param[3], Param[4], myTolTang, pasuv))
        {
            return;
        }

        AddAPoint(previousPoint);
        //
        IntWalk_StatusDeflection aStatus = IntWalk_OK, aPrevStatus = IntWalk_OK;
        bool NoTestDeflection = false;
        Span<double> SvParam = stackalloc double[4];
        Span<double> aNewPnt = stackalloc double[4];
        Span<double> anAbsParamDist = stackalloc double[4];
        double f;
        int LevelOfEmptyInmyIntersectionOn2S = 0;
        int LevelOfPointConfondu = 0;
        int LevelOfIterWithoutAppend = -1;
        //

        ReadOnlySpan<double> aTol = [Epsilon(UM1 - Um1), Epsilon(VM1 - Vm1), Epsilon(UM2 - Um2), Epsilon(VM2 - Vm2)];

        int aPrevNbPoints = line.NbPoints();

        Arrive = false;
        while (!Arrive) // 010
        {
            aPrevStatus = aStatus;

            LevelOfIterWithoutAppend++;
            if (LevelOfIterWithoutAppend > 20)
            {
                Arrive = true;
                if (DejaReparti)
                {
                    break;
                }
                RepartirOuDiviser(ref DejaReparti, ref ChoixIso, ref Arrive);
                LevelOfIterWithoutAppend = 0;
            }
            //
            // compute f
            f = 0.0;
            switch (ChoixIso)
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
                default:
                    break;
            }
            //
            if (f < 0.1)
            {
                f = 0.1;
            }
            //
            SetParameters(Param, previousPoint);
            //
            //--ofv.begin
            double aIncKey, aEps, dP1, dP2, dP3, dP4;
            //
            dP1 = sensCheminement * pasuv[0] * previousd1.X() / f;
            dP2 = sensCheminement * pasuv[1] * previousd1.Y() / f;
            dP3 = sensCheminement * pasuv[2] * previousd2.X() / f;
            dP4 = sensCheminement * pasuv[3] * previousd2.Y() / f;
            //
            aIncKey = 5.0 * (double)IncKey;
            aEps = 1.0e-7;
            if (ChoixIso == IntImp_UIsoparametricOnCaro1 && Math.Abs(dP1) < aEps)
            {
                dP1 *= aIncKey;
            }

            if (ChoixIso == IntImp_VIsoparametricOnCaro1 && Math.Abs(dP2) < aEps)
            {
                dP2 *= aIncKey;
            }

            if (ChoixIso == IntImp_UIsoparametricOnCaro2 && Math.Abs(dP3) < aEps)
            {
                dP3 *= aIncKey;
            }

            if (ChoixIso == IntImp_VIsoparametricOnCaro2 && Math.Abs(dP4) < aEps)
            {
                dP4 *= aIncKey;
            }
            //--ofv.end
            //
            Param[1] += dP1;
            Param[2] += dP2;
            Param[3] += dP3;
            Param[4] += dP4;
            //==========================
            SvParam[0] = Param[1];
            SvParam[1] = Param[2];
            SvParam[2] = Param[3];
            SvParam[3] = Param[4];
            //
            int aTryNumber = 0;
            bool isBadPoint = false;
            IntImp_ConstIsoparametric aBestIso = ChoixIso;
            do
            {
                isBadPoint = false;

                ChoixIso = myIntersectionOn2S.Perform(Param, Rsnld, aBestIso);

                if (myIntersectionOn2S.IsDone() && !myIntersectionOn2S.IsEmpty())
                {
                    // If we go along any surface boundary then it is possible
                    // to find "outboundaried" point.
                    // Nevertheless, if this deflection is quite small, we will be
                    // able to adjust this point to the boundary.

                    myIntersectionOn2S.Point().Parameters(out aNewPnt[0], out aNewPnt[1], out aNewPnt[2],
                                                          out aNewPnt[3]);
                    ReadOnlySpan<double> aParMin = [Um1, Vm1, Um2, Vm2];
                    ReadOnlySpan<double> aParMax = [UM1, VM1, UM2, VM2];

                    for (int i = 0; i < 4; i++)
                    {
                        if (Math.Abs(aNewPnt[i] - aParMin[i]) < aTol[i])
                        {
                            aNewPnt[i] = aParMin[i];
                        }
                        else if (Math.Abs(aNewPnt[i] - aParMax[i]) < aTol[i])
                        {
                            aNewPnt[i] = aParMax[i];
                        }
                    }

                    if (aNewPnt[0] < Um1 || aNewPnt[0] > UM1 || aNewPnt[1] < Vm1 || aNewPnt[1] > VM1
                        || aNewPnt[2] < Um2 || aNewPnt[2] > UM2 || aNewPnt[3] < Vm2 || aNewPnt[3] > VM2)
                    {
                        // Out of borders, process it later.
                        break;
                    }

                    myIntersectionOn2S.ChangePoint().SetValue(aNewPnt[0], aNewPnt[1], aNewPnt[2], aNewPnt[3]);

                    anAbsParamDist[0] = Math.Abs(Param[1] - dP1 - aNewPnt[0]);
                    anAbsParamDist[1] = Math.Abs(Param[2] - dP2 - aNewPnt[1]);
                    anAbsParamDist[2] = Math.Abs(Param[3] - dP3 - aNewPnt[2]);
                    anAbsParamDist[3] = Math.Abs(Param[4] - dP4 - aNewPnt[3]);
                    if (anAbsParamDist[0] < ResoU1 && anAbsParamDist[1] < ResoV1 && anAbsParamDist[2] < ResoU2
                        && anAbsParamDist[3] < ResoV2 && aStatus != IntWalk_PasTropGrand)
                    {
                        isBadPoint = true;
                        aBestIso = (IntImp_ConstIsoparametric)(((int)aBestIso + 1) % 4);
                    }
                }
            } while (isBadPoint && ++aTryNumber <= 4);
            //
            if (!myIntersectionOn2S.IsDone())
            {
                // end of line, division
                Arrive = false;
                Param[1] = SvParam[0];
                Param[2] = SvParam[1];
                Param[3] = SvParam[2];
                Param[4] = SvParam[3];
                RepartirOuDiviser(ref DejaReparti, ref ChoixIso, ref Arrive);
            }
            else // 009
            {
                //== Calculation of exact point from Param(.) is possible
                if (myIntersectionOn2S.IsEmpty())
                {
                    previousPoint.Parameters(out double u1, out double v1, out double u2, out double v2);
                    //
                    Arrive = false;
                    if (u1 < UFirst1 || u1 > ULast1)
                    {
                        Arrive = true;
                    }

                    if (u2 < UFirst2 || u2 > ULast2)
                    {
                        Arrive = true;
                    }

                    if (v1 < VFirst1 || v1 > VLast1)
                    {
                        Arrive = true;
                    }

                    if (v2 < VFirst2 || v2 > VLast2)
                    {
                        Arrive = true;
                    }

                    RepartirOuDiviser(ref DejaReparti, ref ChoixIso, ref Arrive);
                    LevelOfEmptyInmyIntersectionOn2S++;
                    //
                    if (LevelOfEmptyInmyIntersectionOn2S > 10)
                    {
                        pasuv[0] = pasSav[0];
                        pasuv[1] = pasSav[1];
                        pasuv[2] = pasSav[2];
                        pasuv[3] = pasSav[3];
                    }
                }
                else // 008
                {
                    //============================================================
                    //== A point has been found :  T E S T   D E F L E C T I O N
                    //============================================================
                    if (NoTestDeflection)
                    {
                        NoTestDeflection = false;
                    }
                    else
                    {
                        if (--LevelOfEmptyInmyIntersectionOn2S <= 0)
                        {
                            LevelOfEmptyInmyIntersectionOn2S = 0;
                            if (LevelOfIterWithoutAppend < 10)
                            {
                                aStatus = TestDeflection(ChoixIso, aStatus);
                            }
                            else
                            {
                                if (aStatus != IntWalk_StepTooSmall)
                                {
                                    // Bug #0029682 (Boolean intersection with
                                    // fuzzy-option hangs).
                                    // If aStatus == IntWalk_StepTooSmall then
                                    // the counter "LevelOfIterWithoutAppend" will
                                    // be nulified in the future if the step is smaller
                                    // (see "case IntWalk_StepTooSmall:" below).
                                    // Here, we forbid this nulification and thereby provide out from
                                    // the algorithm.
                                    pasuv[0] *= 0.5;
                                    pasuv[1] *= 0.5;
                                    pasuv[2] *= 0.5;
                                    pasuv[3] *= 0.5;
                                }
                            }
                        }
                    }

                    //============================================================
                    //==       T r a i t e m e n t   s u r   S t a t u s        ==
                    //============================================================
                    if (LevelOfPointConfondu > 5)
                    {
                        aStatus = IntWalk_ArretSurPoint;
                        LevelOfPointConfondu = 0;
                    }
                    //
                    if (aStatus == IntWalk_OK)
                    {
                        NbPasOKConseq++;
                        if (NbPasOKConseq >= 5)
                        {
                            NbPasOKConseq = 0;
                            bool pastroppetit;
                            double t;
                            //
                            do
                            {
                                pastroppetit = true;
                                //
                                if (pasuv[0] < pasInit[0])
                                {
                                    t = (pasInit[0] - pasuv[0]) * 0.25;
                                    if (t > 0.1 * pasInit[0])
                                    {
                                        t = 0.1 * pasuv[0];
                                    }

                                    pasuv[0] += t;
                                    pastroppetit = false;
                                }

                                if (pasuv[1] < pasInit[1])
                                {
                                    t = (pasInit[1] - pasuv[1]) * 0.25;
                                    if (t > 0.1 * pasInit[1])
                                    {
                                        t = 0.1 * pasuv[1];
                                    }

                                    pasuv[1] += t;
                                    pastroppetit = false;
                                }

                                if (pasuv[2] < pasInit[2])
                                {
                                    t = (pasInit[2] - pasuv[2]) * 0.25;
                                    if (t > 0.1 * pasInit[2])
                                    {
                                        t = 0.1 * pasuv[2];
                                    }

                                    pasuv[2] += t;
                                    pastroppetit = false;
                                }

                                if (pasuv[3] < pasInit[3])
                                {
                                    t = (pasInit[3] - pasuv[3]) * 0.25;
                                    if (t > 0.1 * pasInit[3])
                                    {
                                        t = 0.1 * pasuv[3];
                                    }
                                    pasuv[3] += t;
                                    pastroppetit = false;
                                }
                                if (pastroppetit)
                                {
                                    if (pasMax < 0.1)
                                    {
                                        pasMax *= 1.1;
                                        pasInit[0] *= 1.1;
                                        pasInit[1] *= 1.1;
                                        pasInit[2] *= 1.1;
                                        pasInit[3] *= 1.1;
                                    }
                                    else
                                    {
                                        pastroppetit = false;
                                    }
                                }
                            } while (pastroppetit);
                        }
                    } // aStatus==IntWalk_OK
                    else
                    {
                        NbPasOKConseq = 0;
                    }

                    //
                    switch (aStatus) // 007
                    {
                        case IntWalk_ArretSurPointPrecedent:
                        {
                            Arrive = false;
                            RepartirOuDiviser(ref DejaReparti, ref ChoixIso, ref Arrive);
                            break;
                        }
                        case IntWalk_PasTropGrand:
                        {
                            Param[1] = SvParam[0];
                            Param[2] = SvParam[1];
                            Param[3] = SvParam[2];
                            Param[4] = SvParam[3];

                            if (LevelOfIterWithoutAppend > 5)
                            {
                                for (int i = 0; i < 4; i++)
                                {
                                    if (pasSav[i] > pasInit[i])
                                    {
                                        continue;
                                    }

                                    double aDelta = (pasInit[i] - pasSav[i]) * 0.25;

                                    if (aDelta > Epsilon(pasInit[i]))
                                    {
                                        pasInit[i] -= aDelta;
                                        if ((aPrevStatus != IntWalk_StepTooSmall) && (line.NbPoints() != aPrevNbPoints))
                                        {
                                            LevelOfIterWithoutAppend = 0;
                                        }

                                        aPrevNbPoints = line.NbPoints();
                                    }
                                }
                            }

                            break;
                        }
                        case IntWalk_PointConfondu:
                        {
                            LevelOfPointConfondu++;

                            if (LevelOfPointConfondu > 5)
                            {
                                bool pastroppetit;
                                //
                                do
                                {
                                    pastroppetit = true;

                                    for (int i = 0; i < 4; i++)
                                    {
                                        if (pasuv[i] < pasInit[i])
                                        {
                                            pasuv[i] += (pasInit[i] - pasuv[i]) * 0.25;
                                            pastroppetit = false;
                                        }
                                    }

                                    if (pastroppetit)
                                    {
                                        if (pasMax < 0.1)
                                        {
                                            pasMax *= 1.1;
                                            pasInit[0] *= 1.1;
                                            pasInit[1] *= 1.1;
                                            pasInit[2] *= 1.1;
                                            pasInit[3] *= 1.1;
                                        }
                                        else
                                        {
                                            pastroppetit = false;
                                        }
                                    }
                                } while (pastroppetit);
                            }

                            break;
                        }
                        case IntWalk_StepTooSmall:
                        {
                            bool hasStepBeenIncreased = false;

                            for (int i = 0; i < 4; i++)
                            {
                                double aNewStep = Math.Min(1.5 * pasuv[i], pasInit[i]);
                                if (aNewStep > pasuv[i])
                                {
                                    pasuv[i] = aNewStep;
                                    hasStepBeenIncreased = true;
                                }
                            }

                            if (hasStepBeenIncreased)
                            {
                                Param[1] = SvParam[0];
                                Param[2] = SvParam[1];
                                Param[3] = SvParam[2];
                                Param[4] = SvParam[3];

                                // In order to avoid cyclic changes
                                // (PasTropGrand --> Decrease step -->
                                // StepTooSmall --> Increase step --> PasTropGrand...)
                                // nullify LevelOfIterWithoutAppend only if the condition
                                // is satisfied:
                                if ((aPrevStatus != IntWalk_PasTropGrand) && (line.NbPoints() != aPrevNbPoints))
                                {
                                    LevelOfIterWithoutAppend = 0;
                                }

                                aPrevNbPoints = line.NbPoints();

                                break;
                            }

                            goto case IntWalk_OK; // [[fallthrough]]
                        }
                        case IntWalk_OK:
                        case IntWalk_ArretSurPoint: // 006
                        {
                            //=======================================================
                            //== Stop Test t   :  Frame on Param(.)     ==
                            //=======================================================
                            // xft arrive here
                            Arrive = TestArret(DejaReparti, Param, ref ChoixIso);
                            // JMB 30th December 1999.
                            // Some statement below should not be put in comment because they are useful.
                            // See grid CTO 909 A1 which infinitely loops
                            if (!Arrive && aStatus == IntWalk_ArretSurPoint)
                            {
                                Arrive = true;
                            }

                            if (Arrive)
                            {
                                NbPasOKConseq = -10;
                            }

                            if (!Arrive) // 005
                            {
                                //=====================================================
                                //== Param(.) is in the limits                       ==
                                //==  and does not end a closed  line                ==
                                //=====================================================
                                //== Check on the current point of myInters
                                bool pointisvalid = false;
                                {
                                    myIntersectionOn2S.Point().Parameters(out double u1, out double v1, out double u2,
                                                                          out double v2);

                                    //
                                    if (u1 <= UM1 && u2 <= UM2 && v1 <= VM1 && v2 <= VM2 && u1 >= Um1 && u2 >= Um2
                                        && v1 >= Vm1 && v2 >= Vm2)
                                    {
                                        pointisvalid = true;
                                    }
                                }

                                //
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
                                    //=====================================================
                                    //== Check on the previous Point
                                    {
                                        previousPoint.Parameters(out double u1, out double v1, out double u2,
                                                                 out double v2);
                                        if (u1 <= UM1 && u2 <= UM2 && v1 <= VM1 && v2 <= VM2 && u1 >= Um1 && u2 >= Um2
                                            && v1 >= Vm1 && v2 >= Vm2)
                                        {
                                            pl = previousPoint.Value();
                                            if (bTestFirstPoint)
                                            {
                                                if (pf.SquareDistance(pl) < aSQDistMax)
                                                {
                                                    IncKey++;
                                                    if (IncKey == 5000)
                                                    {
                                                        return;
                                                    }
                                                    else
                                                    {
                                                        continue;
                                                    }
                                                }
                                                else
                                                {
                                                    bTestFirstPoint = false;
                                                }
                                            }
                                            // Check for 3D cycling: if the walker has moved away from the starting
                                            // point (bTestFirstPoint is false) and later returns near it, the
                                            // intersection curve is closed in 3D. This catches cases where the
                                            // UV-based closure check in TestArret fails (e.g., near-tangent surfaces
                                            // with shared edges).
                                            else if (pf.SquareDistance(pl) < aSQDistMax)
                                            {
                                                close = true;
                                                Arrive = true;
                                                AddAPoint(line.Value(1));
                                                LevelOfIterWithoutAppend = 0;
                                                break;
                                            }
                                            //
                                            AddAPoint(previousPoint);
                                            RejectIndex++;

                                            if (RejectIndex >= RejectIndexMAX)
                                            {
                                                Arrive = true;
                                                break;
                                            }

                                            //
                                            LevelOfIterWithoutAppend = 0;
                                        }
                                    }
                                } // pointisvalid
                                //====================================================

                                if (aStatus == IntWalk_ArretSurPoint)
                                {
                                    RepartirOuDiviser(ref DejaReparti, ref ChoixIso, ref Arrive);
                                }
                                else
                                {
                                    if (line.NbPoints() == 2)
                                    {
                                        pasSav[0] = pasuv[0];
                                        pasSav[1] = pasuv[1];
                                        pasSav[2] = pasuv[2];
                                        pasSav[3] = pasuv[3];

                                        if ((aPrevStatus == IntWalk_PasTropGrand) && (LevelOfIterWithoutAppend > 0))
                                        {
                                            pasInit[0] = pasuv[0];
                                            pasInit[1] = pasuv[1];
                                            pasInit[2] = pasuv[2];
                                            pasInit[3] = pasuv[3];
                                        }
                                    }
                                }
                            } // 005 if(!Arrive)
                            else // 004
                            {
                                if (close)
                                {
                                    //================= la ligne est fermee ===============
                                    AddAPoint(line.Value(1)); // ligne fermee
                                    LevelOfIterWithoutAppend = 0;
                                }
                                else //$$$
                                {
                                    //====================================================
                                    //== Param was not in the limits (was reframed)
                                    //====================================================
                                    bool bPrevNotTangent = !previoustg || !myIntersectionOn2S.IsTangent();

                                    IntImp_ConstIsoparametric SauvChoixIso = ChoixIso;
                                    ChoixIso = myIntersectionOn2S.Perform(Param, Rsnld, ChoixIso);
                                    //
                                    if (!myIntersectionOn2S.IsEmpty()) // 002
                                    {
                                        // mutially outpasses in the square or intersection in corner

                                        if (TestArret(true, Param, ref ChoixIso))
                                        {
                                            NbPasOKConseq = -10;
                                            ChoixIso = myIntersectionOn2S.Perform(Param, Rsnld, ChoixIso);

                                            if (!myIntersectionOn2S.IsEmpty())
                                            {
                                                previousPoint = myIntersectionOn2S.Point();
                                                previoustg = myIntersectionOn2S.IsTangent();

                                                if (!previoustg)
                                                {
                                                    previousd = myIntersectionOn2S.Direction();
                                                    previousd1 = myIntersectionOn2S.DirectionOnS1();
                                                    previousd2 = myIntersectionOn2S.DirectionOnS2();
                                                }

                                                pl = previousPoint.Value();

                                                if (bTestFirstPoint)
                                                {
                                                    if (pf.SquareDistance(pl) < aSQDistMax)
                                                    {
                                                        IncKey++;
                                                        if (IncKey == 5000)
                                                        {
                                                            return;
                                                        }
                                                        else
                                                        {
                                                            continue;
                                                        }
                                                    }
                                                    else
                                                    {
                                                        bTestFirstPoint = false;
                                                    }
                                                }
                                                //
                                                AddAPoint(previousPoint);
                                                RejectIndex++;

                                                if (RejectIndex >= RejectIndexMAX)
                                                {
                                                    Arrive = true;
                                                    break;
                                                }

                                                //
                                                LevelOfIterWithoutAppend = 0;
                                                RepartirOuDiviser(ref DejaReparti, ref ChoixIso, ref Arrive);
                                            }
                                            else
                                            {
                                                // fail framing divides the step
                                                Arrive = false;
                                                RepartirOuDiviser(ref DejaReparti, ref ChoixIso, ref Arrive);
                                                NoTestDeflection = true;
                                                ChoixIso = SauvChoixIso;
                                            }
                                        } // if(TestArret())
                                        else
                                        {
                                            // save the last point
                                            // to revert to it if the current point is out of bounds

                                            IntSurf_PntOn2S previousPointSave = previousPoint;
                                            bool previoustgSave = previoustg;
                                            gp_Dir previousdSave = previousd;
                                            gp_Dir2d previousd1Save = previousd1;
                                            gp_Dir2d previousd2Save = previousd2;

                                            previousPoint = myIntersectionOn2S.Point();
                                            previoustg = myIntersectionOn2S.IsTangent();
                                            Arrive = false;

                                            if (!previoustg)
                                            {
                                                previousd = myIntersectionOn2S.Direction();
                                                previousd1 = myIntersectionOn2S.DirectionOnS1();
                                                previousd2 = myIntersectionOn2S.DirectionOnS2();
                                            }

                                            //========================================
                                            //== Check on PreviousPoint @@

                                            {
                                                previousPoint.Parameters(out double u1, out double v1, out double u2,
                                                                         out double v2);

                                                // To save initial 2d points
                                                gp_Pnt2d ParamPntOnS1 = new(Param[1], Param[2]);
                                                gp_Pnt2d ParamPntOnS2 = new(Param[3], Param[4]);

                                                ///////////////////////////
                                                Param[1] = u1;
                                                Param[2] = v1;
                                                Param[3] = u2;
                                                Param[4] = v2;
                                                //

                                                // xf
                                                bool bFlag1, bFlag2;
                                                double aTol2D = 1.0e-11;
                                                //
                                                bFlag1 = u1 >= Um1 - aTol2D && v1 >= Vm1 - aTol2D && u1 <= UM1 + aTol2D
                                                         && v1 <= VM1 + aTol2D;
                                                bFlag2 = u2 >= Um2 - aTol2D && v2 >= Vm2 - aTol2D && u2 <= UM2 + aTol2D
                                                         && v2 <= VM2 + aTol2D;
                                                if (bFlag1 && bFlag2)
                                                {
                                                    if (line.NbPoints() > 1)
                                                    {
                                                        IntSurf_PntOn2S prevprevPoint = line.Value(line.NbPoints() - 1);
                                                        prevprevPoint.Parameters(out double ppU1, out double ppV1,
                                                                                 out double ppU2, out double ppV2);
                                                        previousPointSave.Parameters(out double pU1, out double pV1,
                                                                                     out double pU2, out double pV2);
                                                        gp_Vec2d V1onS1 = new(new gp_Pnt2d(ppU1, ppV1), new gp_Pnt2d(pU1, pV1));
                                                        gp_Vec2d V2onS1 = new(new gp_Pnt2d(pU1, pV1), new gp_Pnt2d(u1, v1));
                                                        gp_Vec2d V1onS2 = new(new gp_Pnt2d(ppU2, ppV2), new gp_Pnt2d(pU2, pV2));
                                                        gp_Vec2d V2onS2 = new(new gp_Pnt2d(pU2, pV2), new gp_Pnt2d(u2, v2));

                                                        double aDot1 = V1onS1 * V2onS1;
                                                        double aDot2 = V1onS2 * V2onS2;

                                                        if ((aDot1 < 0.0) || (aDot2 < 0.0))
                                                        {
                                                            Arrive = true;
                                                            break;
                                                        }
                                                    }
                                                    // xt
                                                    pl = previousPoint.Value();

                                                    if (bTestFirstPoint)
                                                    {
                                                        if (pf.SquareDistance(pl) < aSQDistMax)
                                                        {
                                                            IncKey++;

                                                            if (IncKey == 5000)
                                                            {
                                                                return;
                                                            }
                                                            else
                                                            {
                                                                continue;
                                                            }
                                                        }
                                                        else
                                                        {
                                                            bTestFirstPoint = false;
                                                        }
                                                    }

                                                    // To avoid walking around the same point
                                                    // in the tangent zone near a border

                                                    if (previoustg)
                                                    {
                                                        // There are three consecutive points:
                                                        // previousPointSave -> ParamPnt -> curPnt.

                                                        previousPointSave.Parameters(out double prevU1, out double prevV1,
                                                                                     out double prevU2, out double prevV2);
                                                        gp_Pnt2d prevPntOnS1 = new(prevU1, prevV1), prevPntOnS2 = new(prevU2, prevV2);
                                                        gp_Pnt2d curPntOnS1 = new(u1, v1), curPntOnS2 = new(u2, v2);
                                                        gp_Vec2d PrevToParamOnS1 = new(prevPntOnS1, ParamPntOnS1);
                                                        gp_Vec2d PrevToCurOnS1 = new(prevPntOnS1, curPntOnS1);
                                                        gp_Vec2d PrevToParamOnS2 = new(prevPntOnS2, ParamPntOnS2);
                                                        gp_Vec2d PrevToCurOnS2 = new(prevPntOnS2, curPntOnS2);
                                                        double MaxAngle = 3 * M_PI / 4;
                                                        double anAngleS1 = 0.0, anAngleS2 = 0.0;
                                                        double aSQMParS1 = PrevToParamOnS1.SquareMagnitude();
                                                        double aSQMParS2 = PrevToParamOnS2.SquareMagnitude();
                                                        double aSQMCurS1 = PrevToCurOnS1.SquareMagnitude();
                                                        double aSQMCurS2 = PrevToCurOnS2.SquareMagnitude();

                                                        if (aSQMCurS1 < gp.Resolution())
                                                        {
                                                            // We came back to the one of previous point.
                                                            // Therefore, we must break;

                                                            anAngleS1 = M_PI;
                                                        }
                                                        else if (aSQMParS1 < gp.Resolution())
                                                        {
                                                            // We are walking along tangent zone.
                                                            // It should be continued.
                                                            anAngleS1 = 0.0;
                                                        }
                                                        else
                                                        {
                                                            anAngleS1 = Math.Abs(PrevToParamOnS1.Angle(PrevToCurOnS1));
                                                        }

                                                        if (aSQMCurS2 < gp.Resolution())
                                                        {
                                                            // We came back to the one of previous point.
                                                            // Therefore, we must break;

                                                            anAngleS2 = M_PI;
                                                        }
                                                        else if (aSQMParS2 < gp.Resolution())
                                                        {
                                                            // We are walking along tangent zone.
                                                            // It should be continued;
                                                            anAngleS2 = 0.0;
                                                        }
                                                        else
                                                        {
                                                            anAngleS2 = Math.Abs(PrevToParamOnS2.Angle(PrevToCurOnS2));
                                                        }

                                                        if ((anAngleS1 > MaxAngle) && (anAngleS2 > MaxAngle))
                                                        {
                                                            Arrive = true;
                                                            break;
                                                        }

                                                        {
                                                            // Check singularity.
                                                            // I.e. check if we are walking along direction, which does
                                                            // not result in coming to any point (i.e. derivative
                                                            // 3D-intersection curve along this direction is equal to 0).
                                                            // A sphere with direction {dU=1, dV=0} from point
                                                            //(U=0, V=M_PI/2) can be considered as example for
                                                            // this case (we cannot find another 3D-point if we go thus).

                                                            // Direction chosen along 1st and 2nd surface correspondingly
                                                            gp_Vec2d aDirS1 = new(prevPntOnS1, curPntOnS1),
                                                                     aDirS2 = new(prevPntOnS2, curPntOnS2);

                                                            myIntersectionOn2S.Function().AuxillarSurface1().D1(
                                                                curPntOnS1.X(), curPntOnS1.Y(), out gp_Pnt aPtemp,
                                                                out gp_Vec aDuS1, out gp_Vec aDvS1);
                                                            myIntersectionOn2S.Function().AuxillarSurface2().D1(
                                                                curPntOnS2.X(), curPntOnS2.Y(), out aPtemp,
                                                                out gp_Vec aDuS2, out gp_Vec aDvS2);

                                                            // Derivative WLine along (it is vector-function indeed)
                                                            // directions chosen
                                                            // F1 - on the 1st surface, F2 - on the 2nd surface.
                                                            // x, y, z - coordinates of derivative vector.
                                                            double aF1x = aDuS1.X() * aDirS1.X() + aDvS1.X() * aDirS1.Y();
                                                            double aF1y = aDuS1.Y() * aDirS1.X() + aDvS1.Y() * aDirS1.Y();
                                                            double aF1z = aDuS1.Z() * aDirS1.X() + aDvS1.Z() * aDirS1.Y();
                                                            double aF2x = aDuS2.X() * aDirS2.X() + aDvS2.X() * aDirS2.Y();
                                                            double aF2y = aDuS2.Y() * aDirS2.X() + aDvS2.Y() * aDirS2.Y();
                                                            double aF2z = aDuS2.Z() * aDirS2.X() + aDvS2.Z() * aDirS2.Y();

                                                            double aF1 = aF1x * aF1x + aF1y * aF1y + aF1z * aF1z;
                                                            double aF2 = aF2x * aF2x + aF2y * aF2y + aF2z * aF2z;

                                                            if ((aF1 < gp.Resolution()) && (aF2 < gp.Resolution()))
                                                            {
                                                                // All derivative are equal to 0. Therefore, there is
                                                                // no point in going along direction chosen.
                                                                Arrive = true;
                                                                break;
                                                            }
                                                        }
                                                    } // if (previoustg) cond.

                                                    ////////////////////////////////////////
                                                    AddAPoint(previousPoint);
                                                    RejectIndex++;

                                                    if (RejectIndex >= RejectIndexMAX)
                                                    {
                                                        Arrive = true;
                                                        break;
                                                    }

                                                    //

                                                    LevelOfIterWithoutAppend = 0;
                                                    Arrive = true;
                                                }
                                                else
                                                {
                                                    // revert to the last correctly calculated point
                                                    previousPoint = previousPointSave;
                                                    previoustg = previoustgSave;
                                                    previousd = previousdSave;
                                                    previousd1 = previousd1Save;
                                                    previousd2 = previousd2Save;
                                                }
                                            }

                                            //
                                            bool wasExtended = false;

                                            if (Arrive && myIntersectionOn2S.IsTangent() && bPrevNotTangent)
                                            {
                                                if (ExtendLineInCommonZone(SauvChoixIso, DejaReparti))
                                                {
                                                    wasExtended = true;
                                                    Arrive = false;
                                                    ChoixIso = SauvChoixIso;
                                                }
                                            }

                                            RepartirOuDiviser(ref DejaReparti, ref ChoixIso, ref Arrive);

                                            if (Arrive && myIntersectionOn2S.IsDone() && !myIntersectionOn2S.IsEmpty()
                                                && myIntersectionOn2S.IsTangent() && bPrevNotTangent && !wasExtended)
                                            {
                                                if (ExtendLineInCommonZone(SauvChoixIso, DejaReparti))
                                                {
                                                    wasExtended = true;
                                                    Arrive = false;
                                                    ChoixIso = SauvChoixIso;
                                                }
                                            }
                                        } // else !TestArret() $
                                    } //$$ end successful framing on border (!myIntersectionOn2S.IsEmpty())
                                    else
                                    {
                                        // echec framing on border; division of step
                                        Arrive = false;
                                        NoTestDeflection = true;
                                        RepartirOuDiviser(ref DejaReparti, ref ChoixIso, ref Arrive);
                                    }
                                } //$$$ end framing on border (!close)
                            } // 004 fin TestArret return Arrive = True
                            break;
                        } // 006case IntWalk_ArretSurPoint:  end Processing Status = OK  or ArretSurPoint
                    } // 007  switch(aStatus)
                } // 008 end processing point  (TEST DEFLECTION)
            } // 009 end processing line (else if myIntersectionOn2S.IsDone())
        } // 010 end if first departure point allows marching  while (!Arrive)

        done = true;
    }
}
