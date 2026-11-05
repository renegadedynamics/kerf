// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Intersection.Implicit.ConstIsoparametric;
using static Kerf.Engine.Intersection.Walking.StatusDeflection;

namespace Kerf.Engine.Intersection.Walking;

internal partial class PWalking
{
    /// <summary>
    /// Walks from ParDep (u1, v1, u2, v2); the box (u1min..v2max) of the expected line sets the initial steps.
    /// </summary>
    public void Perform(NCollection_Array1<double> parDep, double u1min, double v1min, double u2min, double v2min,
                        double u1max, double v1max, double u2max, double v2max)
    {
        const double sqDistMax = 1.0e-14;
        // xf

        int nbPasOKConseq = 0;
        NCollection_Array1<double> param = new(1, 4);
        ConstIsoparametric choixIso;
        // xt
        //
        _done = false;
        //
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
        //
        ComputePasInit(u1max - u1min, v1max - v1min, u2max - u2min, v2max - v2min);

        for (int i = 0; i < 4; ++i)
        {
            if (_pasuv[i] > 10)
            {
                _pasuv[i] = 10;
            }

            _pasInit[i] = _pasSav[i] = _pasuv[i];
        }
        //
        _line = new LineOn2S();
        //
        for (int i = 1; i <= 4; ++i)
        {
            param[i] = parDep[i];
        }
        //-- reproduce steps uv connected to surfaces Caro1 and Caro2
        //-- pasuv[] and pasSav[] are modified during the marching
        for (int i = 0; i < 4; ++i)
        {
            _pasSav[i] = _pasuv[i] = _pasInit[i];
        }

        //-- calculate the first solution point
        FunctionSetRoot rsnld = new(_intersectionOn2S.Function());
        //
        choixIso = _intersectionOn2S.Perform(param, rsnld);
        if (!_intersectionOn2S.IsDone)
        {
            return;
        }

        //
        if (_intersectionOn2S.IsEmpty)
        {
            return;
        }
        //
        if (_intersectionOn2S.IsTangent)
        {
            return;
        }
        //
        bool arrive, dejaReparti;
        const int rejectIndexMAX = 250000;
        int IncKey, rejectIndex;
        Point3D pf, pl;
        //
        dejaReparti = false;
        IncKey = 0;
        rejectIndex = 0;
        //
        _previousPoint = _intersectionOn2S.Point();
        _previoustg = false;
        _previousd = _intersectionOn2S.Direction();
        _previousd1 = _intersectionOn2S.DirectionOnS1();
        _previousd2 = _intersectionOn2S.DirectionOnS2();
        _tangentIdx = 1;
        _tgdir = _previousd;
        _firstd1 = _previousd1;
        _firstd2 = _previousd2;
        _tgfirst = _tglast = false;
        _choixIsoSav = choixIso;
        //------------------------------------------------------------
        //-- Test if the first point of marching corresponds
        //-- to a point on borders.
        //-- In this case, DejaReparti is initialized as True
        //--
        pf = _previousPoint.Value();
        bool bTestFirstPoint = true;

        SetParameters(param, _previousPoint);

        if (IsTangentExtCheck(caro1, caro2, param[1], param[2], param[3], param[4], _tolTang, _pasuv))
        {
            return;
        }

        AddAPoint(_previousPoint);
        //
        StatusDeflection status = StatusDeflection.OK, prevStatus = StatusDeflection.OK;
        bool noTestDeflection = false;
        Span<double> svParam = stackalloc double[4];
        Span<double> newPnt = stackalloc double[4];
        Span<double> absParamDist = stackalloc double[4];
        double f;
        int levelOfEmptyInmyIntersectionOn2S = 0;
        int levelOfPointConfondu = 0;
        int levelOfIterWithoutAppend = -1;
        //

        ReadOnlySpan<double> tol = [Epsilon(_uMax1 - _uMin1), Epsilon(_vMax1 - _vMin1), Epsilon(_uMax2 - _uMin2), Epsilon(_vMax2 - _vMin2)];

        int prevNbPoints = _line.PointCount;

        arrive = false;
        while (!arrive) // 010
        {
            prevStatus = status;

            levelOfIterWithoutAppend++;
            if (levelOfIterWithoutAppend > 20)
            {
                arrive = true;
                if (dejaReparti)
                {
                    break;
                }
                RepartirOuDiviser(ref dejaReparti, ref choixIso, ref arrive);
                levelOfIterWithoutAppend = 0;
            }
            //
            // compute f
            f = 0.0;
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
                default:
                    break;
            }
            //
            if (f < 0.1)
            {
                f = 0.1;
            }
            //
            SetParameters(param, _previousPoint);
            //
            //--ofv.begin
            double incKey, eps, dP1, dP2, dP3, dP4;
            //
            dP1 = _sensCheminement * _pasuv[0] * _previousd1.X / f;
            dP2 = _sensCheminement * _pasuv[1] * _previousd1.Y / f;
            dP3 = _sensCheminement * _pasuv[2] * _previousd2.X / f;
            dP4 = _sensCheminement * _pasuv[3] * _previousd2.Y / f;
            //
            incKey = 5.0 * (double)IncKey;
            eps = 1.0e-7;
            if (choixIso == ConstIsoparametric.UIsoparametricOnCaro1 && Math.Abs(dP1) < eps)
            {
                dP1 *= incKey;
            }

            if (choixIso == ConstIsoparametric.VIsoparametricOnCaro1 && Math.Abs(dP2) < eps)
            {
                dP2 *= incKey;
            }

            if (choixIso == ConstIsoparametric.UIsoparametricOnCaro2 && Math.Abs(dP3) < eps)
            {
                dP3 *= incKey;
            }

            if (choixIso == ConstIsoparametric.VIsoparametricOnCaro2 && Math.Abs(dP4) < eps)
            {
                dP4 *= incKey;
            }
            //--ofv.end
            //
            param[1] += dP1;
            param[2] += dP2;
            param[3] += dP3;
            param[4] += dP4;
            //==========================
            svParam[0] = param[1];
            svParam[1] = param[2];
            svParam[2] = param[3];
            svParam[3] = param[4];
            //
            int tryNumber = 0;
            bool isBadPoint = false;
            ConstIsoparametric bestIso = choixIso;
            do
            {
                isBadPoint = false;

                choixIso = _intersectionOn2S.Perform(param, rsnld, bestIso);

                if (_intersectionOn2S.IsDone && !_intersectionOn2S.IsEmpty)
                {
                    // If we go along any surface boundary then it is possible
                    // to find "outboundaried" point.
                    // Nevertheless, if this deflection is quite small, we will be
                    // able to adjust this point to the boundary.

                    _intersectionOn2S.Point().Parameters(out newPnt[0], out newPnt[1], out newPnt[2],
                                                          out newPnt[3]);
                    ReadOnlySpan<double> parMin = [_uMin1, _vMin1, _uMin2, _vMin2];
                    ReadOnlySpan<double> parMax = [_uMax1, _vMax1, _uMax2, _vMax2];

                    for (int i = 0; i < 4; i++)
                    {
                        if (Math.Abs(newPnt[i] - parMin[i]) < tol[i])
                        {
                            newPnt[i] = parMin[i];
                        }
                        else if (Math.Abs(newPnt[i] - parMax[i]) < tol[i])
                        {
                            newPnt[i] = parMax[i];
                        }
                    }

                    if (newPnt[0] < _uMin1 || newPnt[0] > _uMax1 || newPnt[1] < _vMin1 || newPnt[1] > _vMax1
                        || newPnt[2] < _uMin2 || newPnt[2] > _uMax2 || newPnt[3] < _vMin2 || newPnt[3] > _vMax2)
                    {
                        // Out of borders, process it later.
                        break;
                    }

                    _intersectionOn2S.ChangePoint().SetValue(newPnt[0], newPnt[1], newPnt[2], newPnt[3]);

                    absParamDist[0] = Math.Abs(param[1] - dP1 - newPnt[0]);
                    absParamDist[1] = Math.Abs(param[2] - dP2 - newPnt[1]);
                    absParamDist[2] = Math.Abs(param[3] - dP3 - newPnt[2]);
                    absParamDist[3] = Math.Abs(param[4] - dP4 - newPnt[3]);
                    if (absParamDist[0] < _resoU1 && absParamDist[1] < _resoV1 && absParamDist[2] < _resoU2
                        && absParamDist[3] < _resoV2 && status != StatusDeflection.PasTropGrand)
                    {
                        isBadPoint = true;
                        bestIso = (ConstIsoparametric)(((int)bestIso + 1) % 4);
                    }
                }
            } while (isBadPoint && ++tryNumber <= 4);
            //
            if (!_intersectionOn2S.IsDone)
            {
                // end of line, division
                arrive = false;
                param[1] = svParam[0];
                param[2] = svParam[1];
                param[3] = svParam[2];
                param[4] = svParam[3];
                RepartirOuDiviser(ref dejaReparti, ref choixIso, ref arrive);
            }
            else // 009
            {
                //== Calculation of exact point from Param(.) is possible
                if (_intersectionOn2S.IsEmpty)
                {
                    _previousPoint.Parameters(out double u1, out double v1, out double u2, out double v2);
                    //
                    arrive = false;
                    if (u1 < uFirst1 || u1 > uLast1)
                    {
                        arrive = true;
                    }

                    if (u2 < uFirst2 || u2 > uLast2)
                    {
                        arrive = true;
                    }

                    if (v1 < vFirst1 || v1 > vLast1)
                    {
                        arrive = true;
                    }

                    if (v2 < vFirst2 || v2 > vLast2)
                    {
                        arrive = true;
                    }

                    RepartirOuDiviser(ref dejaReparti, ref choixIso, ref arrive);
                    levelOfEmptyInmyIntersectionOn2S++;
                    //
                    if (levelOfEmptyInmyIntersectionOn2S > 10)
                    {
                        _pasuv[0] = _pasSav[0];
                        _pasuv[1] = _pasSav[1];
                        _pasuv[2] = _pasSav[2];
                        _pasuv[3] = _pasSav[3];
                    }
                }
                else // 008
                {
                    //============================================================
                    //== A point has been found :  T E S T   D E F L E C T I O N
                    //============================================================
                    if (noTestDeflection)
                    {
                        noTestDeflection = false;
                    }
                    else
                    {
                        if (--levelOfEmptyInmyIntersectionOn2S <= 0)
                        {
                            levelOfEmptyInmyIntersectionOn2S = 0;
                            if (levelOfIterWithoutAppend < 10)
                            {
                                status = TestDeflection(choixIso, status);
                            }
                            else
                            {
                                if (status != StatusDeflection.StepTooSmall)
                                {
                                    // Bug #0029682 (Boolean intersection with
                                    // fuzzy-option hangs).
                                    // If aStatus == IntWalk_StepTooSmall then
                                    // the counter "LevelOfIterWithoutAppend" will
                                    // be nulified in the future if the step is smaller
                                    // (see "case IntWalk_StepTooSmall:" below).
                                    // Here, we forbid this nulification and thereby provide out from
                                    // the algorithm.
                                    _pasuv[0] *= 0.5;
                                    _pasuv[1] *= 0.5;
                                    _pasuv[2] *= 0.5;
                                    _pasuv[3] *= 0.5;
                                }
                            }
                        }
                    }

                    //============================================================
                    //==       T r a i t e m e n t   s u r   S t a t u s        ==
                    //============================================================
                    if (levelOfPointConfondu > 5)
                    {
                        status = StatusDeflection.ArretSurPoint;
                        levelOfPointConfondu = 0;
                    }
                    //
                    if (status == StatusDeflection.OK)
                    {
                        nbPasOKConseq++;
                        if (nbPasOKConseq >= 5)
                        {
                            nbPasOKConseq = 0;
                            bool pastroppetit;
                            double t;
                            //
                            do
                            {
                                pastroppetit = true;
                                //
                                if (_pasuv[0] < _pasInit[0])
                                {
                                    t = (_pasInit[0] - _pasuv[0]) * 0.25;
                                    if (t > 0.1 * _pasInit[0])
                                    {
                                        t = 0.1 * _pasuv[0];
                                    }

                                    _pasuv[0] += t;
                                    pastroppetit = false;
                                }

                                if (_pasuv[1] < _pasInit[1])
                                {
                                    t = (_pasInit[1] - _pasuv[1]) * 0.25;
                                    if (t > 0.1 * _pasInit[1])
                                    {
                                        t = 0.1 * _pasuv[1];
                                    }

                                    _pasuv[1] += t;
                                    pastroppetit = false;
                                }

                                if (_pasuv[2] < _pasInit[2])
                                {
                                    t = (_pasInit[2] - _pasuv[2]) * 0.25;
                                    if (t > 0.1 * _pasInit[2])
                                    {
                                        t = 0.1 * _pasuv[2];
                                    }

                                    _pasuv[2] += t;
                                    pastroppetit = false;
                                }

                                if (_pasuv[3] < _pasInit[3])
                                {
                                    t = (_pasInit[3] - _pasuv[3]) * 0.25;
                                    if (t > 0.1 * _pasInit[3])
                                    {
                                        t = 0.1 * _pasuv[3];
                                    }
                                    _pasuv[3] += t;
                                    pastroppetit = false;
                                }
                                if (pastroppetit)
                                {
                                    if (_pasMax < 0.1)
                                    {
                                        _pasMax *= 1.1;
                                        _pasInit[0] *= 1.1;
                                        _pasInit[1] *= 1.1;
                                        _pasInit[2] *= 1.1;
                                        _pasInit[3] *= 1.1;
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
                        nbPasOKConseq = 0;
                    }

                    //
                    switch (status) // 007
                    {
                        case StatusDeflection.ArretSurPointPrecedent:
                        {
                            arrive = false;
                            RepartirOuDiviser(ref dejaReparti, ref choixIso, ref arrive);
                            break;
                        }
                        case StatusDeflection.PasTropGrand:
                        {
                            param[1] = svParam[0];
                            param[2] = svParam[1];
                            param[3] = svParam[2];
                            param[4] = svParam[3];

                            if (levelOfIterWithoutAppend > 5)
                            {
                                for (int i = 0; i < 4; i++)
                                {
                                    if (_pasSav[i] > _pasInit[i])
                                    {
                                        continue;
                                    }

                                    double delta = (_pasInit[i] - _pasSav[i]) * 0.25;

                                    if (delta > Epsilon(_pasInit[i]))
                                    {
                                        _pasInit[i] -= delta;
                                        if ((prevStatus != StatusDeflection.StepTooSmall) && (_line.PointCount != prevNbPoints))
                                        {
                                            levelOfIterWithoutAppend = 0;
                                        }

                                        prevNbPoints = _line.PointCount;
                                    }
                                }
                            }

                            break;
                        }
                        case StatusDeflection.PointConfondu:
                        {
                            levelOfPointConfondu++;

                            if (levelOfPointConfondu > 5)
                            {
                                bool pastroppetit;
                                //
                                do
                                {
                                    pastroppetit = true;

                                    for (int i = 0; i < 4; i++)
                                    {
                                        if (_pasuv[i] < _pasInit[i])
                                        {
                                            _pasuv[i] += (_pasInit[i] - _pasuv[i]) * 0.25;
                                            pastroppetit = false;
                                        }
                                    }

                                    if (pastroppetit)
                                    {
                                        if (_pasMax < 0.1)
                                        {
                                            _pasMax *= 1.1;
                                            _pasInit[0] *= 1.1;
                                            _pasInit[1] *= 1.1;
                                            _pasInit[2] *= 1.1;
                                            _pasInit[3] *= 1.1;
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
                        case StatusDeflection.StepTooSmall:
                        {
                            bool hasStepBeenIncreased = false;

                            for (int i = 0; i < 4; i++)
                            {
                                double newStep = Math.Min(1.5 * _pasuv[i], _pasInit[i]);
                                if (newStep > _pasuv[i])
                                {
                                    _pasuv[i] = newStep;
                                    hasStepBeenIncreased = true;
                                }
                            }

                            if (hasStepBeenIncreased)
                            {
                                param[1] = svParam[0];
                                param[2] = svParam[1];
                                param[3] = svParam[2];
                                param[4] = svParam[3];

                                // In order to avoid cyclic changes
                                // (PasTropGrand --> Decrease step -->
                                // StepTooSmall --> Increase step --> PasTropGrand...)
                                // nullify LevelOfIterWithoutAppend only if the condition
                                // is satisfied:
                                if ((prevStatus != StatusDeflection.PasTropGrand) && (_line.PointCount != prevNbPoints))
                                {
                                    levelOfIterWithoutAppend = 0;
                                }

                                prevNbPoints = _line.PointCount;

                                break;
                            }

                            goto case StatusDeflection.OK; // [[fallthrough]]
                        }
                        case StatusDeflection.OK:
                        case StatusDeflection.ArretSurPoint: // 006
                        {
                            //=======================================================
                            //== Stop Test t   :  Frame on Param(.)     ==
                            //=======================================================
                            // xft arrive here
                            arrive = TestArret(dejaReparti, param, ref choixIso);
                            // JMB 30th December 1999.
                            // Some statement below should not be put in comment because they are useful.
                            // See grid CTO 909 A1 which infinitely loops
                            if (!arrive && status == StatusDeflection.ArretSurPoint)
                            {
                                arrive = true;
                            }

                            if (arrive)
                            {
                                nbPasOKConseq = -10;
                            }

                            if (!arrive) // 005
                            {
                                //=====================================================
                                //== Param(.) is in the limits                       ==
                                //==  and does not end a closed  line                ==
                                //=====================================================
                                //== Check on the current point of myInters
                                bool pointisvalid = false;
                                {
                                    _intersectionOn2S.Point().Parameters(out double u1, out double v1, out double u2,
                                                                          out double v2);

                                    //
                                    if (u1 <= _uMax1 && u2 <= _uMax2 && v1 <= _vMax1 && v2 <= _vMax2 && u1 >= _uMin1 && u2 >= _uMin2
                                        && v1 >= _vMin1 && v2 >= _vMin2)
                                    {
                                        pointisvalid = true;
                                    }
                                }

                                //
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
                                    //=====================================================
                                    //== Check on the previous Point
                                    {
                                        _previousPoint.Parameters(out double u1, out double v1, out double u2,
                                                                 out double v2);
                                        if (u1 <= _uMax1 && u2 <= _uMax2 && v1 <= _vMax1 && v2 <= _vMax2 && u1 >= _uMin1 && u2 >= _uMin2
                                            && v1 >= _vMin1 && v2 >= _vMin2)
                                        {
                                            pl = _previousPoint.Value();
                                            if (bTestFirstPoint)
                                            {
                                                if (pf.SquareDistanceTo(pl) < sqDistMax)
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
                                            else if (pf.SquareDistanceTo(pl) < sqDistMax)
                                            {
                                                _close = true;
                                                arrive = true;
                                                AddAPoint(_line.Value(1));
                                                levelOfIterWithoutAppend = 0;
                                                break;
                                            }
                                            //
                                            AddAPoint(_previousPoint);
                                            rejectIndex++;

                                            if (rejectIndex >= rejectIndexMAX)
                                            {
                                                arrive = true;
                                                break;
                                            }

                                            //
                                            levelOfIterWithoutAppend = 0;
                                        }
                                    }
                                } // pointisvalid
                                //====================================================

                                if (status == StatusDeflection.ArretSurPoint)
                                {
                                    RepartirOuDiviser(ref dejaReparti, ref choixIso, ref arrive);
                                }
                                else
                                {
                                    if (_line.PointCount == 2)
                                    {
                                        _pasSav[0] = _pasuv[0];
                                        _pasSav[1] = _pasuv[1];
                                        _pasSav[2] = _pasuv[2];
                                        _pasSav[3] = _pasuv[3];

                                        if ((prevStatus == StatusDeflection.PasTropGrand) && (levelOfIterWithoutAppend > 0))
                                        {
                                            _pasInit[0] = _pasuv[0];
                                            _pasInit[1] = _pasuv[1];
                                            _pasInit[2] = _pasuv[2];
                                            _pasInit[3] = _pasuv[3];
                                        }
                                    }
                                }
                            } // 005 if(!Arrive)
                            else // 004
                            {
                                if (_close)
                                {
                                    //================= la ligne est fermee ===============
                                    AddAPoint(_line.Value(1)); // ligne fermee
                                    levelOfIterWithoutAppend = 0;
                                }
                                else //$$$
                                {
                                    //====================================================
                                    //== Param was not in the limits (was reframed)
                                    //====================================================
                                    bool bPrevNotTangent = !_previoustg || !_intersectionOn2S.IsTangent;

                                    ConstIsoparametric sauvChoixIso = choixIso;
                                    choixIso = _intersectionOn2S.Perform(param, rsnld, choixIso);
                                    //
                                    if (!_intersectionOn2S.IsEmpty) // 002
                                    {
                                        // mutially outpasses in the square or intersection in corner

                                        if (TestArret(true, param, ref choixIso))
                                        {
                                            nbPasOKConseq = -10;
                                            choixIso = _intersectionOn2S.Perform(param, rsnld, choixIso);

                                            if (!_intersectionOn2S.IsEmpty)
                                            {
                                                _previousPoint = _intersectionOn2S.Point();
                                                _previoustg = _intersectionOn2S.IsTangent;

                                                if (!_previoustg)
                                                {
                                                    _previousd = _intersectionOn2S.Direction();
                                                    _previousd1 = _intersectionOn2S.DirectionOnS1();
                                                    _previousd2 = _intersectionOn2S.DirectionOnS2();
                                                }

                                                pl = _previousPoint.Value();

                                                if (bTestFirstPoint)
                                                {
                                                    if (pf.SquareDistanceTo(pl) < sqDistMax)
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
                                                AddAPoint(_previousPoint);
                                                rejectIndex++;

                                                if (rejectIndex >= rejectIndexMAX)
                                                {
                                                    arrive = true;
                                                    break;
                                                }

                                                //
                                                levelOfIterWithoutAppend = 0;
                                                RepartirOuDiviser(ref dejaReparti, ref choixIso, ref arrive);
                                            }
                                            else
                                            {
                                                // fail framing divides the step
                                                arrive = false;
                                                RepartirOuDiviser(ref dejaReparti, ref choixIso, ref arrive);
                                                noTestDeflection = true;
                                                choixIso = sauvChoixIso;
                                            }
                                        } // if(TestArret())
                                        else
                                        {
                                            // save the last point
                                            // to revert to it if the current point is out of bounds

                                            PntOn2S previousPointSave = _previousPoint;
                                            bool previoustgSave = _previoustg;
                                            Direction3D previousdSave = _previousd;
                                            Direction2D previousd1Save = _previousd1;
                                            Direction2D previousd2Save = _previousd2;

                                            _previousPoint = _intersectionOn2S.Point();
                                            _previoustg = _intersectionOn2S.IsTangent;
                                            arrive = false;

                                            if (!_previoustg)
                                            {
                                                _previousd = _intersectionOn2S.Direction();
                                                _previousd1 = _intersectionOn2S.DirectionOnS1();
                                                _previousd2 = _intersectionOn2S.DirectionOnS2();
                                            }

                                            //========================================
                                            //== Check on PreviousPoint @@

                                            {
                                                _previousPoint.Parameters(out double u1, out double v1, out double u2,
                                                                         out double v2);

                                                // To save initial 2d points
                                                Point2D paramPntOnS1 = new(param[1], param[2]);
                                                Point2D paramPntOnS2 = new(param[3], param[4]);

                                                ///////////////////////////
                                                param[1] = u1;
                                                param[2] = v1;
                                                param[3] = u2;
                                                param[4] = v2;
                                                //

                                                // xf
                                                bool bFlag1, bFlag2;
                                                double tol2D = 1.0e-11;
                                                //
                                                bFlag1 = u1 >= _uMin1 - tol2D && v1 >= _vMin1 - tol2D && u1 <= _uMax1 + tol2D
                                                         && v1 <= _vMax1 + tol2D;
                                                bFlag2 = u2 >= _uMin2 - tol2D && v2 >= _vMin2 - tol2D && u2 <= _uMax2 + tol2D
                                                         && v2 <= _vMax2 + tol2D;
                                                if (bFlag1 && bFlag2)
                                                {
                                                    if (_line.PointCount > 1)
                                                    {
                                                        PntOn2S prevprevPoint = _line.Value(_line.PointCount - 1);
                                                        prevprevPoint.Parameters(out double ppU1, out double ppV1,
                                                                                 out double ppU2, out double ppV2);
                                                        previousPointSave.Parameters(out double pU1, out double pV1,
                                                                                     out double pU2, out double pV2);
                                                        Vector2D v1onS1 = new(new Point2D(ppU1, ppV1), new Point2D(pU1, pV1));
                                                        Vector2D v2onS1 = new(new Point2D(pU1, pV1), new Point2D(u1, v1));
                                                        Vector2D v1onS2 = new(new Point2D(ppU2, ppV2), new Point2D(pU2, pV2));
                                                        Vector2D v2onS2 = new(new Point2D(pU2, pV2), new Point2D(u2, v2));

                                                        double dot1 = v1onS1 * v2onS1;
                                                        double dot2 = v1onS2 * v2onS2;

                                                        if ((dot1 < 0.0) || (dot2 < 0.0))
                                                        {
                                                            arrive = true;
                                                            break;
                                                        }
                                                    }
                                                    // xt
                                                    pl = _previousPoint.Value();

                                                    if (bTestFirstPoint)
                                                    {
                                                        if (pf.SquareDistanceTo(pl) < sqDistMax)
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

                                                    if (_previoustg)
                                                    {
                                                        // There are three consecutive points:
                                                        // previousPointSave -> ParamPnt -> curPnt.

                                                        previousPointSave.Parameters(out double prevU1, out double prevV1,
                                                                                     out double prevU2, out double prevV2);
                                                        Point2D prevPntOnS1 = new(prevU1, prevV1), prevPntOnS2 = new(prevU2, prevV2);
                                                        Point2D curPntOnS1 = new(u1, v1), curPntOnS2 = new(u2, v2);
                                                        Vector2D prevToParamOnS1 = new(prevPntOnS1, paramPntOnS1);
                                                        Vector2D prevToCurOnS1 = new(prevPntOnS1, curPntOnS1);
                                                        Vector2D prevToParamOnS2 = new(prevPntOnS2, paramPntOnS2);
                                                        Vector2D prevToCurOnS2 = new(prevPntOnS2, curPntOnS2);
                                                        double maxAngle = 3 * M_PI / 4;
                                                        double angleS1 = 0.0, angleS2 = 0.0;
                                                        double sqmParS1 = prevToParamOnS1.SquareMagnitude;
                                                        double sqmParS2 = prevToParamOnS2.SquareMagnitude;
                                                        double sqmCurS1 = prevToCurOnS1.SquareMagnitude;
                                                        double sqmCurS2 = prevToCurOnS2.SquareMagnitude;

                                                        if (sqmCurS1 < Constants.Resolution())
                                                        {
                                                            // We came back to the one of previous point.
                                                            // Therefore, we must break;

                                                            angleS1 = M_PI;
                                                        }
                                                        else if (sqmParS1 < Constants.Resolution())
                                                        {
                                                            // We are walking along tangent zone.
                                                            // It should be continued.
                                                            angleS1 = 0.0;
                                                        }
                                                        else
                                                        {
                                                            angleS1 = Math.Abs(prevToParamOnS1.AngleTo(prevToCurOnS1));
                                                        }

                                                        if (sqmCurS2 < Constants.Resolution())
                                                        {
                                                            // We came back to the one of previous point.
                                                            // Therefore, we must break;

                                                            angleS2 = M_PI;
                                                        }
                                                        else if (sqmParS2 < Constants.Resolution())
                                                        {
                                                            // We are walking along tangent zone.
                                                            // It should be continued;
                                                            angleS2 = 0.0;
                                                        }
                                                        else
                                                        {
                                                            angleS2 = Math.Abs(prevToParamOnS2.AngleTo(prevToCurOnS2));
                                                        }

                                                        if ((angleS1 > maxAngle) && (angleS2 > maxAngle))
                                                        {
                                                            arrive = true;
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
                                                            Vector2D dirS1 = new(prevPntOnS1, curPntOnS1),
                                                                     dirS2 = new(prevPntOnS2, curPntOnS2);

                                                            _intersectionOn2S.Function().AuxillarSurface1().D1(
                                                                curPntOnS1.X, curPntOnS1.Y, out Point3D ptemp,
                                                                out Vector3D duS1, out Vector3D dvS1);
                                                            _intersectionOn2S.Function().AuxillarSurface2().D1(
                                                                curPntOnS2.X, curPntOnS2.Y, out ptemp,
                                                                out Vector3D duS2, out Vector3D dvS2);

                                                            // Derivative WLine along (it is vector-function indeed)
                                                            // directions chosen
                                                            // F1 - on the 1st surface, F2 - on the 2nd surface.
                                                            // x, y, z - coordinates of derivative vector.
                                                            double f1x = duS1.X * dirS1.X + dvS1.X * dirS1.Y;
                                                            double f1y = duS1.Y * dirS1.X + dvS1.Y * dirS1.Y;
                                                            double f1z = duS1.Z * dirS1.X + dvS1.Z * dirS1.Y;
                                                            double f2x = duS2.X * dirS2.X + dvS2.X * dirS2.Y;
                                                            double f2y = duS2.Y * dirS2.X + dvS2.Y * dirS2.Y;
                                                            double f2z = duS2.Z * dirS2.X + dvS2.Z * dirS2.Y;

                                                            double f1 = f1x * f1x + f1y * f1y + f1z * f1z;
                                                            double f2 = f2x * f2x + f2y * f2y + f2z * f2z;

                                                            if ((f1 < Constants.Resolution()) && (f2 < Constants.Resolution()))
                                                            {
                                                                // All derivative are equal to 0. Therefore, there is
                                                                // no point in going along direction chosen.
                                                                arrive = true;
                                                                break;
                                                            }
                                                        }
                                                    } // if (previoustg) cond.

                                                    ////////////////////////////////////////
                                                    AddAPoint(_previousPoint);
                                                    rejectIndex++;

                                                    if (rejectIndex >= rejectIndexMAX)
                                                    {
                                                        arrive = true;
                                                        break;
                                                    }

                                                    //

                                                    levelOfIterWithoutAppend = 0;
                                                    arrive = true;
                                                }
                                                else
                                                {
                                                    // revert to the last correctly calculated point
                                                    _previousPoint = previousPointSave;
                                                    _previoustg = previoustgSave;
                                                    _previousd = previousdSave;
                                                    _previousd1 = previousd1Save;
                                                    _previousd2 = previousd2Save;
                                                }
                                            }

                                            //
                                            bool wasExtended = false;

                                            if (arrive && _intersectionOn2S.IsTangent && bPrevNotTangent)
                                            {
                                                if (ExtendLineInCommonZone(sauvChoixIso, dejaReparti))
                                                {
                                                    wasExtended = true;
                                                    arrive = false;
                                                    choixIso = sauvChoixIso;
                                                }
                                            }

                                            RepartirOuDiviser(ref dejaReparti, ref choixIso, ref arrive);

                                            if (arrive && _intersectionOn2S.IsDone && !_intersectionOn2S.IsEmpty
                                                && _intersectionOn2S.IsTangent && bPrevNotTangent && !wasExtended)
                                            {
                                                if (ExtendLineInCommonZone(sauvChoixIso, dejaReparti))
                                                {
                                                    wasExtended = true;
                                                    arrive = false;
                                                    choixIso = sauvChoixIso;
                                                }
                                            }
                                        } // else !TestArret() $
                                    } //$$ end successful framing on border (!myIntersectionOn2S.IsEmpty())
                                    else
                                    {
                                        // echec framing on border; division of step
                                        arrive = false;
                                        noTestDeflection = true;
                                        RepartirOuDiviser(ref dejaReparti, ref choixIso, ref arrive);
                                    }
                                } //$$$ end framing on border (!close)
                            } // 004 fin TestArret return Arrive = True
                            break;
                        } // 006case IntWalk_ArretSurPoint:  end Processing Status = OK  or ArretSurPoint
                    } // 007  switch(aStatus)
                } // 008 end processing point  (TEST DEFLECTION)
            } // 009 end processing line (else if myIntersectionOn2S.IsDone())
        } // 010 end if first departure point allows marching  while (!Arrive)

        _done = true;
    }
}
