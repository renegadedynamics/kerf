// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_IntSS_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.IntPatch_IType;
using static Occt.TopAbs_State;

namespace Occt;

public partial class GeomInt_IntSS
{
    /// <summary>std::sort's operator&lt; on doubles (TrimILineOnSurfBoundaries).</summary>
    private readonly struct DoubleLess : std_heap.ILess<double>
    {
        public bool Less(double theLeft, double theRight) => theLeft < theRight;
    }

    private static void AdjustUPeriodic(Geom_Surface aS, Geom2d_Curve? aC2D)
    {
        if (aC2D is null || !aS.IsUPeriodic())
        {
            return;
        }
        //
        double aEps = Precision.PConfusion(); // 1.e-9
        double aEpsilon = Epsilon(10.0); // 1.77e-15
        //
        aS.Bounds(out double umin, out double umax, out _, out _);
        double aPeriod = aS.UPeriod();

        double aT1 = aC2D.FirstParameter();
        double aT2 = aC2D.LastParameter();
        double aTx = aT1 + 0.467 * (aT2 - aT1);
        gp_Pnt2d aPx = aC2D.Value(aTx);
        //
        double aUx = aPx.X();
        if (Math.Abs(aUx) < aEpsilon)
        {
            aUx = 0.0;
        }
        if (Math.Abs(aUx - aPeriod) < aEpsilon)
        {
            aUx = aPeriod;
        }
        //
        double dU = 0.0;
        while (aUx < (umin - aEps))
        {
            aUx += aPeriod;
            dU += aPeriod;
        }
        while (aUx > (umax + aEps))
        {
            aUx -= aPeriod;
            dU -= aPeriod;
        }
        //
        if (dU != 0.0)
        {
            gp_Vec2d aV2D = new(dU, 0.0);
            aC2D.Translate(aV2D);
        }
    }

    private static void GetQuadric(GeomAdaptor_Surface HS1, ref IntSurf_Quadric quad1)
    {
        switch (HS1.GetType())
        {
            case GeomAbs_Plane:
                quad1.SetValue(HS1.Plane());
                break;
            case GeomAbs_Cylinder:
                quad1.SetValue(HS1.Cylinder());
                break;
            case GeomAbs_Cone:
                quad1.SetValue(HS1.Cone());
                break;
            case GeomAbs_Sphere:
                quad1.SetValue(HS1.Sphere());
                break;
            case GeomAbs_Torus:
                quad1.SetValue(HS1.Torus());
                break;
            default:
                throw new Standard_ConstructionError("GeomInt_IntSS::MakeCurve");
        }
    }

    private static void Parameters(GeomAdaptor_Surface HS1, GeomAdaptor_Surface HS2, in gp_Pnt Ptref, out double U1,
                                   out double V1, out double U2, out double V2)
    {
        IntSurf_Quadric quad1 = new(), quad2 = new();
        //
        GetQuadric(HS1, ref quad1);
        GetQuadric(HS2, ref quad2);
        //
        U1 = V1 = U2 = V2 = 0.0;
        quad1.Parameters(Ptref, ref U1, ref V1);
        quad2.Parameters(Ptref, ref U2, ref V2);
    }

    private static bool ParametersOfNearestPointOnSurface(Extrema_ExtPS theExtr, ref double theU, ref double theV)
    {
        if (!theExtr.IsDone() || theExtr.NbExt() == 0)
        {
            return false;
        }

        int anIndex = 1;
        double aMinSQDist = theExtr.SquareDistance(anIndex);
        for (int i = 2; i <= theExtr.NbExt(); i++)
        {
            double aSQD = theExtr.SquareDistance(i);
            if (aSQD < aMinSQDist)
            {
                aMinSQDist = aSQD;
                anIndex = i;
            }
        }

        theExtr.Point(anIndex).Parameter(out theU, out theV);

        return true;
    }

    private static void GetSegmentBoundary(in IntRes2d_IntersectionSegment theSegm, Geom2d_Curve theCurve,
                                           NCollection_DynamicArray<double> theArrayOfParameters)
    {
        double aU1 = theCurve.FirstParameter(), aU2 = theCurve.LastParameter();

        if (theSegm.HasFirstPoint())
        {
            IntRes2d_IntersectionPoint anIPF = theSegm.FirstPoint();
            aU1 = anIPF.ParamOnFirst();
        }

        if (theSegm.HasLastPoint())
        {
            IntRes2d_IntersectionPoint anIPL = theSegm.LastPoint();
            aU2 = anIPL.ParamOnFirst();
        }

        theArrayOfParameters.Append(aU1);
        theArrayOfParameters.Append(aU2);
    }

    private static void IntersectCurveAndBoundary(Geom2d_Curve? theC2d, Geom2d_Curve?[] theArrBounds,
                                                  int theNumberOfCurves, double theTol,
                                                  NCollection_DynamicArray<double> theArrayOfParameters)
    {
        if (theC2d is null)
        {
            return;
        }

        Geom2dAdaptor_Curve anAC1 = new(theC2d);
        for (int aCurID = 0; aCurID < theNumberOfCurves; aCurID++)
        {
            if (theArrBounds[aCurID] is null)
            {
                continue;
            }

            Geom2dAdaptor_Curve anAC2 = new(theArrBounds[aCurID]!);
            Geom2dInt_GInter anIntCC2d = new(anAC1, anAC2, theTol, theTol);

            if (!anIntCC2d.IsDone() || anIntCC2d.IsEmpty())
            {
                continue;
            }

            for (int aPntID = 1; aPntID <= anIntCC2d.NbPoints(); aPntID++)
            {
                double aParam = anIntCC2d.Point(aPntID).ParamOnFirst();
                theArrayOfParameters.Append(aParam);
            }

            for (int aSegmID = 1; aSegmID <= anIntCC2d.NbSegments(); aSegmID++)
            {
                GetSegmentBoundary(anIntCC2d.Segment(aSegmID), theC2d, theArrayOfParameters);
            }
        }
    }

    // Check if theAHC2d corresponds to a degenerated edge.
    private static bool isDegenerated(GeomAdaptor_Surface theGAHS, Adaptor2d_Curve2d theAHC2d, double theFirstPar,
                                      double theLastPar)
    {
        double aSqTol = Precision.Confusion() * Precision.Confusion();
        gp_Pnt2d aP2d;
        gp_Pnt aP1, aP2;

        theAHC2d.D0(theFirstPar, out aP2d);
        theGAHS.D0(aP2d.X(), aP2d.Y(), out aP1);

        theAHC2d.D0(theLastPar, out aP2d);
        theGAHS.D0(aP2d.X(), aP2d.Y(), out aP2);

        if (aP1.SquareDistance(aP2) > aSqTol)
        {
            return false;
        }

        theAHC2d.D0(0.5 * (theFirstPar + theLastPar), out aP2d);
        theGAHS.D0(aP2d.X(), aP2d.Y(), out aP2);

        return aP1.SquareDistance(aP2) <= aSqTol;
    }

    /// <summary>Makes the Geom curves of the line of index Index of the intersector (with their 2D curves on
    /// request) and appends them to the results.</summary>
    protected void MakeCurve(int Index, Adaptor3d_TopolTool dom1, Adaptor3d_TopolTool dom2, double Tol, bool Approx,
                             bool ApproxS1, bool ApproxS2)
    {
        bool myApprox1, myApprox2, myApprox;
        double Tolpc, myTolApprox;
        IntPatch_IType typl;
        Geom2d_BSplineCurve? H1 = null;
        Geom_Surface aS1, aS2;
        //
        Tolpc = Tol;
        myApprox = Approx;
        myApprox1 = ApproxS1;
        myApprox2 = ApproxS2;
        myTolApprox = 0.0000001;
        //
        GeomAdaptor_Surface myHS1 = this.myHS1!;
        GeomAdaptor_Surface myHS2 = this.myHS2!;
        aS1 = myHS1.Surface()!;
        aS2 = myHS2.Surface()!;
        //
        IntPatch_Line L = myIntersector.Line(Index);
        typl = L.ArcType();
        //
        if (typl == IntPatch_Walking)
        {
            IntPatch_WLine? aWLine = L as IntPatch_WLine;
            if (aWLine is null)
            {
                return;
            }
            L = aWLine;
        }
        //
        // Line Constructor
        myLConstruct.Perform(L);
        if (!myLConstruct.IsDone() || myLConstruct.NbParts() <= 0)
        {
            return;
        }
        // Do the Curve
        bool ok;
        int i, j, aNbParts;
        double fprm, lprm;
        Geom_Curve? newc = null;

        switch (typl)
        {
            // ########################################
            //  Line, Parabola, Hyperbola
            // ########################################
            case IntPatch_Lin:
            case IntPatch_Parabola:
            case IntPatch_Hyperbola:
            {
                if (typl == IntPatch_Lin)
                {
                    newc = new Geom_Line((L as IntPatch_GLine)!.Line());
                }

                else if (typl == IntPatch_Parabola)
                {
                    newc = new Geom_Parabola((L as IntPatch_GLine)!.Parabola());
                }

                else if (typl == IntPatch_Hyperbola)
                {
                    newc = new Geom_Hyperbola((L as IntPatch_GLine)!.Hyperbola());
                }
                //
                aNbParts = myLConstruct.NbParts();
                for (i = 1; i <= aNbParts; i++)
                {
                    myLConstruct.Part(i, out fprm, out lprm);

                    if (!Precision.IsNegativeInfinite(fprm) && !Precision.IsPositiveInfinite(lprm))
                    {
                        Geom_TrimmedCurve aCT3D = new(newc!, fprm, lprm);
                        sline.Append(aCT3D);
                        //
                        if (myApprox1)
                        {
                            Geom2d_Curve? C2d = null;
                            BuildPCurves(fprm, lprm, ref Tolpc, myHS1.Surface(), newc!, ref C2d);
                            if (Tolpc > myTolReached2d || myTolReached2d == 0.0)
                            {
                                myTolReached2d = Tolpc;
                            }
                            slineS1.Append(new Geom2d_TrimmedCurve(C2d!, fprm, lprm));
                        }
                        else
                        {
                            slineS1.Append(H1);
                        }
                        //
                        if (myApprox2)
                        {
                            Geom2d_Curve? C2d = null;
                            BuildPCurves(fprm, lprm, ref Tolpc, myHS2.Surface(), newc!, ref C2d);
                            if (Tolpc > myTolReached2d || myTolReached2d == 0.0)
                            {
                                myTolReached2d = Tolpc;
                            }
                            //
                            slineS2.Append(new Geom2d_TrimmedCurve(C2d!, fprm, lprm));
                        }
                        else
                        {
                            slineS2.Append(H1);
                        }
                    } // if (!Precision::IsNegativeInfinite(fprm) &&  !Precision::IsPositiveInfinite(lprm))

                    else
                    {
                        GeomAbs_SurfaceType typS1 = myHS1.GetType();
                        GeomAbs_SurfaceType typS2 = myHS2.GetType();
                        if (typS1 == GeomAbs_SurfaceOfExtrusion || typS1 == GeomAbs_OffsetSurface
                            || typS1 == GeomAbs_SurfaceOfRevolution || typS2 == GeomAbs_SurfaceOfExtrusion
                            || typS2 == GeomAbs_OffsetSurface || typS2 == GeomAbs_SurfaceOfRevolution)
                        {
                            sline.Append(newc!);
                            slineS1.Append(H1);
                            slineS2.Append(H1);
                            continue;
                        }
                        bool bFNIt, bLPIt;
                        double aTestPrm, dT = 100.0;
                        double u1, v1, u2, v2, TolX;
                        //
                        bFNIt = Precision.IsNegativeInfinite(fprm);
                        bLPIt = Precision.IsPositiveInfinite(lprm);

                        aTestPrm = 0.0;

                        if (bFNIt && !bLPIt)
                        {
                            aTestPrm = lprm - dT;
                        }
                        else if (!bFNIt && bLPIt)
                        {
                            aTestPrm = fprm + dT;
                        }
                        //
                        gp_Pnt ptref = newc!.Value(aTestPrm);
                        //
                        TolX = Precision.Confusion();
                        Parameters(myHS1, myHS2, ptref, out u1, out v1, out u2, out v2);
                        ok = (dom1.Classify(new gp_Pnt2d(u1, v1), TolX) != TopAbs_OUT);
                        if (ok)
                        {
                            ok = (dom2.Classify(new gp_Pnt2d(u2, v2), TolX) != TopAbs_OUT);
                        }
                        if (ok)
                        {
                            sline.Append(newc);
                            slineS1.Append(H1);
                            slineS2.Append(H1);
                        }
                    }
                } // end of for (i=1; i<=myLConstruct.NbParts(); i++)
            } // case IntPatch_Lin:  case IntPatch_Parabola:  case IntPatch_Hyperbola:
            break;

            // ########################################
            //  Circle and Ellipse
            // ########################################
            case IntPatch_Circle:
            case IntPatch_Ellipse:
            {

                if (typl == IntPatch_Circle)
                {
                    newc = new Geom_Circle((L as IntPatch_GLine)!.Circle());
                }
                else
                {
                    newc = new Geom_Ellipse((L as IntPatch_GLine)!.Ellipse());
                }
                //
                double aPeriod, aRealEpsilon;
                //
                aRealEpsilon = RealEpsilon();
                aPeriod = M_PI + M_PI;
                //
                aNbParts = myLConstruct.NbParts();
                //
                for (i = 1; i <= aNbParts; i++)
                {
                    myLConstruct.Part(i, out fprm, out lprm);
                    //
                    if (Math.Abs(fprm) > aRealEpsilon || Math.Abs(lprm - aPeriod) > aRealEpsilon)
                    {
                        //==============================================
                        Geom_TrimmedCurve aTC3D = new(newc, fprm, lprm);
                        //
                        sline.Append(aTC3D);
                        //
                        fprm = aTC3D.FirstParameter();
                        lprm = aTC3D.LastParameter();
                        ////
                        if (myApprox1)
                        {
                            Geom2d_Curve? C2d = null;
                            BuildPCurves(fprm, lprm, ref Tolpc, myHS1.Surface(), newc, ref C2d);
                            if (Tolpc > myTolReached2d || myTolReached2d == 0.0)
                            {
                                myTolReached2d = Tolpc;
                            }
                            slineS1.Append(C2d);
                        }
                        else
                        { ////
                            slineS1.Append(H1);
                        }
                        //
                        if (myApprox2)
                        {
                            Geom2d_Curve? C2d = null;
                            BuildPCurves(fprm, lprm, ref Tolpc, myHS2.Surface(), newc, ref C2d);
                            if (Tolpc > myTolReached2d || myTolReached2d == 0)
                            {
                                myTolReached2d = Tolpc;
                            }
                            slineS2.Append(C2d);
                        }
                        else
                        {
                            slineS2.Append(H1);
                        }
                        //==============================================
                    } // if (std::abs(fprm) > RealEpsilon() || std::abs(lprm-2.*M_PI) > RealEpsilon())
                    //
                    else
                    { //  on regarde si on garde
                        //
                        if (aNbParts == 1)
                        {
                            if (Math.Abs(fprm) < RealEpsilon() && Math.Abs(lprm - 2.0 * M_PI) < RealEpsilon())
                            {
                                Geom_TrimmedCurve aTC3D = new(newc, fprm, lprm);
                                //
                                sline.Append(aTC3D);
                                fprm = aTC3D.FirstParameter();
                                lprm = aTC3D.LastParameter();

                                if (myApprox1)
                                {
                                    Geom2d_Curve? C2d = null;
                                    BuildPCurves(fprm, lprm, ref Tolpc, myHS1.Surface(), newc, ref C2d);
                                    if (Tolpc > myTolReached2d || myTolReached2d == 0)
                                    {
                                        myTolReached2d = Tolpc;
                                    }
                                    slineS1.Append(C2d);
                                }
                                else
                                { ////
                                    slineS1.Append(H1);
                                }

                                if (myApprox2)
                                {
                                    Geom2d_Curve? C2d = null;
                                    BuildPCurves(fprm, lprm, ref Tolpc, myHS2.Surface(), newc, ref C2d);
                                    if (Tolpc > myTolReached2d || myTolReached2d == 0)
                                    {
                                        myTolReached2d = Tolpc;
                                    }
                                    slineS2.Append(C2d);
                                }
                                else
                                {
                                    slineS2.Append(H1);
                                }
                                break;
                            }
                        }
                        //
                        double aTwoPIdiv17, u1, v1, u2, v2, TolX;
                        //
                        aTwoPIdiv17 = 2.0 * M_PI / 17.0;
                        //
                        for (j = 0; j <= 17; j++)
                        {
                            gp_Pnt ptref = newc.Value(j * aTwoPIdiv17);
                            TolX = Precision.Confusion();

                            Parameters(myHS1, myHS2, ptref, out u1, out v1, out u2, out v2);
                            ok = (dom1.Classify(new gp_Pnt2d(u1, v1), TolX) != TopAbs_OUT);
                            if (ok)
                            {
                                ok = (dom2.Classify(new gp_Pnt2d(u2, v2), TolX) != TopAbs_OUT);
                            }
                            if (ok)
                            {
                                sline.Append(newc);
                                //==============================================
                                if (myApprox1)
                                {
                                    Geom2d_Curve? C2d = null;
                                    BuildPCurves(fprm, lprm, ref Tolpc, myHS1.Surface(), newc, ref C2d);
                                    if (Tolpc > myTolReached2d || myTolReached2d == 0)
                                    {
                                        myTolReached2d = Tolpc;
                                    }
                                    slineS1.Append(C2d);
                                }
                                else
                                {
                                    slineS1.Append(H1);
                                }

                                if (myApprox2)
                                {
                                    Geom2d_Curve? C2d = null;
                                    BuildPCurves(fprm, lprm, ref Tolpc, myHS2.Surface(), newc, ref C2d);
                                    if (Tolpc > myTolReached2d || myTolReached2d == 0)
                                    {
                                        myTolReached2d = Tolpc;
                                    }
                                    slineS2.Append(C2d);
                                }
                                else
                                {
                                    slineS2.Append(H1);
                                }
                                break;
                            } //  end of if (ok) {
                        } //  end of for (int j=0; j<=17; j++)
                    } //  end of else { on regarde si on garde
                } // for (i=1; i<=myLConstruct.NbParts(); i++)
            } // IntPatch_Circle: IntPatch_Ellipse
            break;

            // ########################################
            //  Analytic
            // ########################################
            case IntPatch_Analytic:
                // This case was processed earlier (in IntPatch_Intersection)

                break;

            // ########################################
            //  Walking
            // ########################################
            case IntPatch_Walking:
            {
                IntPatch_WLine WL = (L as IntPatch_WLine)!;

                //
                int ifprm, ilprm;
                //
                if (!myApprox)
                {
                    aNbParts = myLConstruct.NbParts();
                    for (i = 1; i <= aNbParts; i++)
                    {
                        myLConstruct.Part(i, out fprm, out lprm);
                        ifprm = (int)fprm;
                        ilprm = (int)lprm;
                        //
                        Geom2d_BSplineCurve? aH1 = null, aH2 = null;

                        if (myApprox1)
                        {
                            aH1 = MakeBSpline2d(WL, ifprm, ilprm, true);
                        }
                        if (myApprox2)
                        {
                            aH2 = MakeBSpline2d(WL, ifprm, ilprm, false);
                        }
                        //
                        Geom_Curve aBSp = MakeBSpline(WL, ifprm, ilprm);
                        //
                        sline.Append(aBSp);
                        slineS1.Append(aH1);
                        slineS2.Append(aH2);
                    }
                }
                //
                else
                {
                    bool bIsDecomposited;
                    int nbiter, aNbSeqOfL;
                    GeomInt_WLApprox theapp3d = new();
                    NCollection_Sequence<IntPatch_Line> aSeqOfL = new();
                    double tol2d, aTolSS;
                    //
                    tol2d = myTolApprox;
                    aTolSS = 2.0e-7;
                    theapp3d.SetParameters(myTolApprox, tol2d, 4, 8, 0, 30, myHS1 != myHS2);
                    //
                    bIsDecomposited = GeomInt_LineTool.DecompositionOfWLine(WL, myHS1, myHS2, aTolSS, myLConstruct,
                                                                            aSeqOfL);
                    //
                    aNbParts = myLConstruct.NbParts();
                    aNbSeqOfL = aSeqOfL.Length();
                    //
                    nbiter = (bIsDecomposited) ? aNbSeqOfL : aNbParts;
                    //
                    for (i = 1; i <= nbiter; i++)
                    {
                        if (bIsDecomposited)
                        {
                            WL = (aSeqOfL.Value(i) as IntPatch_WLine)!;
                            ifprm = 1;
                            ilprm = WL.NbPnts();
                        }
                        else
                        {
                            myLConstruct.Part(i, out fprm, out lprm);
                            ifprm = (int)fprm;
                            ilprm = (int)lprm;
                        }

                        bool anApprox = myApprox;
                        bool anApprox1 = myApprox1;
                        bool anApprox2 = myApprox2;
                        GeomAbs_SurfaceType typs1, typs2;
                        typs1 = myHS1.GetType();
                        typs2 = myHS2.GetType();

                        if (typs1 == GeomAbs_Plane)
                        {
                            anApprox = false;
                            anApprox1 = true;
                        }
                        else if (typs2 == GeomAbs_Plane)
                        {
                            anApprox = false;
                            anApprox2 = true;
                        }

                        Approx_ParametrizationType aParType =
                            ApproxInt_KnotTools.DefineParType(WL, ifprm, ilprm, anApprox, anApprox1, anApprox2);

                        theapp3d.SetParameters(myTolApprox, tol2d, 4, 8, 0, 30, myHS1 != myHS2, aParType);

                        //-- lbr :
                        //-- Si une des surfaces est un plan , on approxime en 2d
                        //-- sur cette surface et on remonte les points 2d en 3d.
                        //
                        if (typs1 == GeomAbs_Plane)
                        {
                            theapp3d.Perform(myHS1, myHS2, WL, false, true, myApprox2, ifprm, ilprm);
                        }
                        else if (typs2 == GeomAbs_Plane)
                        {
                            theapp3d.Perform(myHS1, myHS2, WL, false, myApprox1, true, ifprm, ilprm);
                        }
                        else
                        {
                            //
                            if (myHS1 != myHS2)
                            {
                                if ((typs1 == GeomAbs_BezierSurface || typs1 == GeomAbs_BSplineSurface)
                                    && (typs2 == GeomAbs_BezierSurface || typs2 == GeomAbs_BSplineSurface))
                                {

                                    theapp3d.SetParameters(myTolApprox, tol2d, 4, 8, 0, 30, true, aParType);
                                }
                            }
                            //
                            theapp3d.Perform(myHS1, myHS2, WL, true, myApprox1, myApprox2, ifprm, ilprm);
                        }

                        if (!theapp3d.IsDone())
                        {
                            //
                            Geom2d_BSplineCurve? aH1 = null, aH2 = null;
                            //
                            Geom_Curve aBSp = MakeBSpline(WL, ifprm, ilprm);
                            if (myApprox1)
                            {
                                aH1 = MakeBSpline2d(WL, ifprm, ilprm, true);
                            }
                            if (myApprox2)
                            {
                                aH2 = MakeBSpline2d(WL, ifprm, ilprm, false);
                            }
                            //
                            sline.Append(aBSp);
                            slineS1.Append(aH1);
                            slineS2.Append(aH2);
                        } // if (!theapp3d.IsDone())

                        else
                        {
                            if (myApprox1 || myApprox2 || (typs1 == GeomAbs_Plane || typs2 == GeomAbs_Plane))
                            {
                                if (theapp3d.TolReached2d() > myTolReached2d || myTolReached2d == 0.0)
                                {
                                    myTolReached2d = theapp3d.TolReached2d();
                                }
                            }
                            if (typs1 == GeomAbs_Plane || typs2 == GeomAbs_Plane)
                            {
                                myTolReached3d = myTolReached2d;
                            }
                            else if (theapp3d.TolReached3d() > myTolReached3d || myTolReached3d == 0.0)
                            {
                                myTolReached3d = theapp3d.TolReached3d();
                            }

                            int aNbMultiCurves, nbpoles;
                            //
                            aNbMultiCurves = theapp3d.NbMultiCurves();
                            for (j = 1; j <= aNbMultiCurves; j++)
                            {
                                if (typs1 == GeomAbs_Plane)
                                {
                                    AppParCurves_MultiBSpCurve mbspc = theapp3d.Value(j);
                                    nbpoles = mbspc.NbPoles();

                                    NCollection_Array1<gp_Pnt2d> tpoles2d = new(1, nbpoles);
                                    NCollection_Array1<gp_Pnt> tpoles = new(1, nbpoles);

                                    mbspc.Curve(1, tpoles2d);
                                    gp_Pln Pln = myHS1.Plane();
                                    //
                                    int ik;
                                    for (ik = 1; ik <= nbpoles; ik++)
                                    {
                                        tpoles.SetValue(ik,
                                                        ElSLib.Value(tpoles2d.Value(ik).X(), tpoles2d.Value(ik).Y(), Pln));
                                    }
                                    //
                                    Geom_BSplineCurve BS = new(tpoles, mbspc.Knots(), mbspc.Multiplicities(),
                                                               mbspc.Degree());
                                    GeomLib_CheckBSplineCurve Check = new(BS, myTolCheck, myTolAngCheck);
                                    Check.FixTangent(true, true);
                                    //
                                    sline.Append(BS);
                                    //
                                    if (myApprox1)
                                    {
                                        Geom2d_BSplineCurve BS1 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        GeomLib_Check2dBSplineCurve Check1 = new(BS1, myTolCheck, myTolAngCheck);
                                        Check1.FixTangent(true, true);
                                        //
                                        AdjustUPeriodic(aS1, BS1);
                                        //
                                        slineS1.Append(BS1);
                                    }
                                    else
                                    {
                                        slineS1.Append(H1);
                                    }

                                    if (myApprox2)
                                    {
                                        mbspc.Curve(2, tpoles2d);

                                        Geom2d_BSplineCurve BS2 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        GeomLib_Check2dBSplineCurve newCheck = new(BS2, myTolCheck, myTolAngCheck);
                                        newCheck.FixTangent(true, true);
                                        //
                                        AdjustUPeriodic(aS2, BS2);
                                        //
                                        slineS2.Append(BS2);
                                    }
                                    else
                                    {
                                        slineS2.Append(H1);
                                    }
                                } // if(typs1 == GeomAbs_Plane)
                                //
                                else if (typs2 == GeomAbs_Plane)
                                {
                                    AppParCurves_MultiBSpCurve mbspc = theapp3d.Value(j);
                                    nbpoles = mbspc.NbPoles();

                                    NCollection_Array1<gp_Pnt2d> tpoles2d = new(1, nbpoles);
                                    NCollection_Array1<gp_Pnt> tpoles = new(1, nbpoles);
                                    mbspc.Curve((myApprox1) ? 2 : 1, tpoles2d);
                                    gp_Pln Pln = myHS2.Plane();
                                    //
                                    int ik;
                                    for (ik = 1; ik <= nbpoles; ik++)
                                    {
                                        tpoles.SetValue(ik,
                                                        ElSLib.Value(tpoles2d.Value(ik).X(), tpoles2d.Value(ik).Y(), Pln));
                                    }
                                    //
                                    Geom_BSplineCurve BS = new(tpoles, mbspc.Knots(), mbspc.Multiplicities(),
                                                               mbspc.Degree());
                                    GeomLib_CheckBSplineCurve Check = new(BS, myTolCheck, myTolAngCheck);
                                    Check.FixTangent(true, true);
                                    //
                                    sline.Append(BS);
                                    //
                                    if (myApprox2)
                                    {
                                        Geom2d_BSplineCurve BS1 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        GeomLib_Check2dBSplineCurve Check1 = new(BS1, myTolCheck, myTolAngCheck);
                                        Check1.FixTangent(true, true);
                                        //
                                        //
                                        AdjustUPeriodic(aS2, BS1);
                                        //
                                        slineS2.Append(BS1);
                                    }
                                    else
                                    {
                                        slineS2.Append(H1);
                                    }

                                    if (myApprox1)
                                    {
                                        mbspc.Curve(1, tpoles2d);
                                        Geom2d_BSplineCurve BS2 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        GeomLib_Check2dBSplineCurve Check2 = new(BS2, myTolCheck, myTolAngCheck);
                                        Check2.FixTangent(true, true);
                                        //
                                        //
                                        AdjustUPeriodic(aS1, BS2);
                                        //
                                        slineS1.Append(BS2);
                                    }
                                    else
                                    {
                                        slineS1.Append(H1);
                                    }
                                } // else if(typs2 == GeomAbs_Plane)
                                //
                                else
                                { // typs1!=GeomAbs_Plane && typs2!=GeomAbs_Plane
                                    AppParCurves_MultiBSpCurve mbspc = theapp3d.Value(j);
                                    nbpoles = mbspc.NbPoles();
                                    NCollection_Array1<gp_Pnt> tpoles = new(1, nbpoles);
                                    mbspc.Curve(1, tpoles);
                                    Geom_BSplineCurve BS = new(tpoles, mbspc.Knots(), mbspc.Multiplicities(),
                                                               mbspc.Degree());
                                    GeomLib_CheckBSplineCurve Check = new(BS, myTolCheck, myTolAngCheck);
                                    Check.FixTangent(true, true);
                                    //
                                    // Check IsClosed()
                                    double aDist = Max(BS.StartPoint().XYZ().SquareModulus(),
                                                       BS.EndPoint().XYZ().SquareModulus());
                                    double eps = Epsilon(aDist);
                                    if (BS.StartPoint().SquareDistance(BS.EndPoint()) < 2.0 * eps)
                                    {
                                        // Avoid creating B-splines containing two coincident poles only
                                        if (mbspc.Degree() == 1 && nbpoles == 2)
                                        {
                                            continue;
                                        }

                                        if (!BS.IsClosed() && !BS.IsPeriodic())
                                        {
                                            // force Closed()
                                            gp_Pnt aPm = new((BS.Pole(1).XYZ() + BS.Pole(BS.NbPoles()).XYZ()) / 2.0);
                                            BS.SetPole(1, aPm);
                                            BS.SetPole(BS.NbPoles(), aPm);
                                        }
                                    }
                                    sline.Append(BS);

                                    if (myApprox1)
                                    {
                                        NCollection_Array1<gp_Pnt2d> tpoles2d = new(1, nbpoles);
                                        mbspc.Curve(2, tpoles2d);
                                        Geom2d_BSplineCurve BS1 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        GeomLib_Check2dBSplineCurve newCheck = new(BS1, myTolCheck, myTolAngCheck);
                                        newCheck.FixTangent(true, true);
                                        //
                                        AdjustUPeriodic(aS1, BS1);
                                        //
                                        slineS1.Append(BS1);
                                    }
                                    else
                                    {
                                        slineS1.Append(H1);
                                    }
                                    if (myApprox2)
                                    {
                                        NCollection_Array1<gp_Pnt2d> tpoles2d = new(1, nbpoles);
                                        mbspc.Curve((myApprox1) ? 3 : 2, tpoles2d);
                                        Geom2d_BSplineCurve BS2 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        GeomLib_Check2dBSplineCurve newCheck = new(BS2, myTolCheck, myTolAngCheck);
                                        newCheck.FixTangent(true, true);
                                        //
                                        AdjustUPeriodic(aS2, BS2);
                                        //
                                        slineS2.Append(BS2);
                                    }
                                    else
                                    {
                                        slineS2.Append(H1);
                                    }
                                } // else { // typs1!=GeomAbs_Plane && typs2!=GeomAbs_Plane
                            } // for (j=1; j<=aNbMultiCurves; j++
                        }
                    }
                }
            }
            break;

            case IntPatch_Restriction:
            {
                IntPatch_RLine RL = (L as IntPatch_RLine)!;
                Geom_Curve? aC3d = null;
                Geom2d_Curve? aC2d1 = null, aC2d2 = null;
                double aTolReached = 0.0;
                TreatRLine(RL, myHS1, myHS2, ref aC3d, ref aC2d1, ref aC2d2, ref aTolReached);

                if (aC3d is null)
                {
                    break;
                }

                Bnd_Box2d aBox1 = new(), aBox2 = new();

                double aU1f = myHS1.FirstUParameter(), aV1f = myHS1.FirstVParameter(), aU1l = myHS1.LastUParameter(),
                       aV1l = myHS1.LastVParameter();
                double aU2f = myHS2.FirstUParameter(), aV2f = myHS2.FirstVParameter(), aU2l = myHS2.LastUParameter(),
                       aV2l = myHS2.LastVParameter();

                aBox1.Add(new gp_Pnt2d(aU1f, aV1f));
                aBox1.Add(new gp_Pnt2d(aU1l, aV1l));
                aBox2.Add(new gp_Pnt2d(aU2f, aV2f));
                aBox2.Add(new gp_Pnt2d(aU2l, aV2l));

                NCollection_DynamicArray<double> anArrayOfParameters = new();

                // We consider here that the intersection line is same-parameter-line
                anArrayOfParameters.Append(aC3d.FirstParameter());
                anArrayOfParameters.Append(aC3d.LastParameter());

                TrimILineOnSurfBoundaries(aC2d1, aC2d2, aBox1, aBox2, anArrayOfParameters);

                int aNbIntersSolutionsm1 = anArrayOfParameters.Length() - 1;

                // Trim RLine found.
                for (int anInd = 0; anInd < aNbIntersSolutionsm1; anInd++)
                {
                    double aParF = anArrayOfParameters[anInd], aParL = anArrayOfParameters[anInd + 1];

                    if ((aParL - aParF) <= Precision.PConfusion())
                    {
                        continue;
                    }

                    double aPar = 0.5 * (aParF + aParL);
                    gp_Pnt2d aPt;

                    Geom2d_Curve? aCurv2d1 = null, aCurv2d2 = null;
                    if (aC2d1 is not null)
                    {
                        aC2d1.D0(aPar, out aPt);

                        if (aBox1.IsOut(aPt))
                        {
                            continue;
                        }

                        if (myApprox1)
                        {
                            aCurv2d1 = new Geom2d_TrimmedCurve(aC2d1, aParF, aParL);
                        }
                    }

                    if (aC2d2 is not null)
                    {
                        aC2d2.D0(aPar, out aPt);

                        if (aBox2.IsOut(aPt))
                        {
                            continue;
                        }

                        if (myApprox2)
                        {
                            aCurv2d2 = new Geom2d_TrimmedCurve(aC2d2, aParF, aParL);
                        }
                    }

                    Geom_Curve aCurv3d = new Geom_TrimmedCurve(aC3d, aParF, aParL);

                    sline.Append(aCurv3d);
                    slineS1.Append(aCurv2d1);
                    slineS2.Append(aCurv2d2);
                }
            }
            break;
        }
    }

    /// <summary>
    /// Makes the 3D curve of a restriction line as the approximation of its arc on its surface (degree up to 8, 1000
    /// segments), and the 2D curves on both surfaces (the arc itself, trimmed, and the projection on the other
    /// surface). theC3d stays null if the arc is degenerate or the approximation fails. The curves are new and
    /// unfrozen.
    /// </summary>
    public static void TreatRLine(IntPatch_RLine theRL, GeomAdaptor_Surface theHS1, GeomAdaptor_Surface theHS2,
                                  ref Geom_Curve? theC3d, ref Geom2d_Curve? theC2d1, ref Geom2d_Curve? theC2d2,
                                  ref double theTolReached)
    {
        GeomAdaptor_Surface aGAHS;
        Adaptor2d_Curve2d anAHC2d;
        double tf, tl;
        // It is assumed that 2d curve is 2d line (rectangular surface domain)
        if (theRL.IsArcOnS1())
        {
            aGAHS = theHS1;
            anAHC2d = theRL.ArcOnS1()!;
            theRL.ParamOnS1(out tf, out tl);
            theC2d1 = Geom2dAdaptor.MakeCurve(anAHC2d);
            tf = Max(tf, theC2d1.FirstParameter());
            tl = Min(tl, theC2d1.LastParameter());
            theC2d1 = new Geom2d_TrimmedCurve(theC2d1, tf, tl);
        }
        else if (theRL.IsArcOnS2())
        {
            aGAHS = theHS2;
            anAHC2d = theRL.ArcOnS2()!;
            theRL.ParamOnS2(out tf, out tl);
            theC2d2 = Geom2dAdaptor.MakeCurve(anAHC2d);
            tf = Max(tf, theC2d2.FirstParameter());
            tl = Min(tl, theC2d2.LastParameter());
            theC2d2 = new Geom2d_TrimmedCurve(theC2d2, tf, tl);
        }
        else
        {
            return;
        }

        // Restriction line can correspond to a degenerated edge.
        // In this case we return null-curve.
        if (isDegenerated(aGAHS, anAHC2d, tf, tl))
        {
            return;
        }

        //
        // To provide sameparameter it is necessary to get 3d curve as
        // approximation of curve on surface.
        int aMaxDeg = 8;
        int aMaxSeg = 1000;
        Approx_CurveOnSurface anApp = new(anAHC2d, aGAHS, tf, tl, Precision.Confusion());
        anApp.Perform(aMaxSeg, aMaxDeg, GeomAbs_C1, true, false);
        if (!anApp.HasResult())
        {
            return;
        }

        // The approximation is discarded: its curve is taken unfrozen.
        theC3d = anApp.InternalCurve3d();
        theTolReached = anApp.MaxError3d();
        double aTol = Precision.Confusion();
        if (theRL.IsArcOnS1())
        {
            Geom_Surface aS = GeomAdaptor.MakeSurface(theHS2);
            BuildPCurves(tf, tl, ref aTol, aS, theC3d!, ref theC2d2);
        }
        if (theRL.IsArcOnS2())
        {
            Geom_Surface aS = GeomAdaptor.MakeSurface(theHS1);
            BuildPCurves(tf, tl, ref aTol, aS, theC3d!, ref theC2d1);
        }
        theTolReached = Max(theTolReached, aTol);
    }

    /// <summary>
    /// Projects theCurve between theFirst and theLast on theSurface bounded by [theUmin, theUmax] × [theVmin,
    /// theVmax] into theCurve2d (a 2-pole segment on a range shorter than 2e-9), unless theCurve2d is already set or
    /// theSurface is null; on a U-periodic surface the result is moved into the U range. theTol is the projection's
    /// tolerance in and out.
    /// </summary>
    public static void BuildPCurves(double theFirst, double theLast, double theUmin, double theUmax, double theVmin,
                                    double theVmax, ref double theTol, Geom_Surface? theSurface, Geom_Curve theCurve,
                                    ref Geom2d_Curve? theCurve2d)
    {
        if (theCurve2d is not null || theSurface is null)
        {
            return;
        }
        //
        // in class ProjLib_Function the range of parameters is shrank by 1.e-09
        if ((theLast - theFirst) > 2.0e-09)
        {
            theCurve2d = GeomProjLib.Curve2d(theCurve, theFirst, theLast, theSurface, theUmin, theUmax, theVmin,
                                             theVmax, ref theTol);
            if (theCurve2d is null)
            {
                // proj. a circle that goes through the pole on a sphere to the sphere
                theTol += Precision.Confusion();
                theCurve2d = GeomProjLib.Curve2d(theCurve, theFirst, theLast, theSurface, ref theTol);
            }
            // OCCT dereferences the projection without checking that the second one succeeded (a null 2D curve
            // here is an access violation in OCCT, a NullReferenceException in the port).
            if (theCurve2d!.GetType() == typeof(Geom2d_BSplineCurve))
            {
                // Check first, last knots to avoid problems with trimming
                // First, last knots can differ from f, l because of numerical error
                // of projection and approximation
                // The same checking as in Geom2d_TrimmedCurve
                if ((theCurve2d.FirstParameter() - theFirst > Precision.PConfusion())
                    || (theLast - theCurve2d.LastParameter() > Precision.PConfusion()))
                {
                    Geom2d_BSplineCurve aBspl = (theCurve2d as Geom2d_BSplineCurve)!;
                    NCollection_Array1<double> aKnots = new(aBspl.InternalKnots());
                    BSplCLib.Reparametrize(theFirst, theLast, aKnots);
                    aBspl.SetKnots(aKnots);
                }
            }
        }
        else
        {
            if ((theLast - theFirst) > Epsilon(Math.Abs(theFirst)))
            {
                // The domain of C2d is [Epsilon(std::abs(f)), 2.e-09]
                // On this small range C2d can be considered as segment
                // of line.

                double aU = 0.0, aV = 0.0;
                GeomAdaptor_Surface anAS = new();
                anAS.Load(theSurface);
                Extrema_ExtPS anExtr = new();
                gp_Pnt aP3d1 = theCurve.Value(theFirst);
                gp_Pnt aP3d2 = theCurve.Value(theLast);

                anExtr.SetAlgo(Extrema_ExtAlgo.Extrema_ExtAlgo_Grad);
                anExtr.Initialize(anAS, theUmin, theUmax, theVmin, theVmax, Precision.Confusion(),
                                  Precision.Confusion());
                anExtr.Perform(aP3d1);

                if (ParametersOfNearestPointOnSurface(anExtr, ref aU, ref aV))
                {
                    gp_Pnt2d aP2d1 = new(aU, aV);

                    anExtr.Perform(aP3d2);

                    if (ParametersOfNearestPointOnSurface(anExtr, ref aU, ref aV))
                    {
                        gp_Pnt2d aP2d2 = new(aU, aV);

                        if (aP2d1.Distance(aP2d2) > gp.Resolution())
                        {
                            NCollection_Array1<gp_Pnt2d> poles = new(1, 2);
                            NCollection_Array1<double> knots = new(1, 2);
                            NCollection_Array1<int> mults = new(1, 2);
                            poles[1] = aP2d1;
                            poles[2] = aP2d2;
                            knots[1] = theFirst;
                            knots[2] = theLast;
                            mults[1] = mults[2] = 2;

                            theCurve2d = new Geom2d_BSplineCurve(poles, knots, mults, 1);

                            // Check same parameter in middle point .begin
                            gp_Pnt PMid = theCurve.Value(0.5 * (theFirst + theLast));
                            gp_Pnt2d pmidcurve2d = new(0.5 * (aP2d1.XY() + aP2d2.XY()));
                            gp_Pnt aPC = anAS.Value(pmidcurve2d.X(), pmidcurve2d.Y());
                            double aDist = PMid.Distance(aPC);
                            theTol = Max(aDist, theTol);
                            // Check same parameter in middle point .end
                        }
                    }
                }
            }
        }
        //
        if (theSurface.IsUPeriodic() && theCurve2d is not null)
        {
            // Recadre dans le domaine UV de la face
            double aTm, U0, aEps, period, du;
            bool bAdjust;
            //
            aEps = Precision.PConfusion();
            period = theSurface.UPeriod();
            //
            aTm = 0.5 * (theFirst + theLast);
            gp_Pnt2d pm = theCurve2d.Value(aTm);
            U0 = pm.X();
            //
            bAdjust = GeomInt.AdjustPeriodic(U0, theUmin, theUmax, period, out _, out du, aEps);
            if (bAdjust)
            {
                gp_Vec2d T1 = new(du, 0.0);
                theCurve2d.Translate(T1);
            }
        }
    }

    /// <summary>
    /// Projects C between f and l on S (over S's bounds) into C2d, unless C2d is already set or S is null (see
    /// <see cref="BuildPCurves(double, double, double, double, double, double, ref double, Geom_Surface?, Geom_Curve,
    /// ref Geom2d_Curve?)"/>).
    /// </summary>
    public static void BuildPCurves(double f, double l, ref double Tol, Geom_Surface? S, Geom_Curve C,
                                    ref Geom2d_Curve? C2d)
    {
        if (C2d is not null || S is null)
        {
            return;
        }
        //
        S.Bounds(out double umin, out double umax, out double vmin, out double vmax);

        BuildPCurves(f, l, umin, umax, vmin, vmax, ref Tol, S, C, ref C2d);
    }

    /// <summary>
    /// Appends to theArrayOfParameters the parameters on theC2d1 and theC2d2 of their intersections with the sides of
    /// the boxes theBound1 and theBound2 (the surfaces' rectangular domains), then sorts the array.
    /// </summary>
    public static void TrimILineOnSurfBoundaries(Geom2d_Curve? theC2d1, Geom2d_Curve? theC2d2, in Bnd_Box2d theBound1,
                                                 in Bnd_Box2d theBound2,
                                                 NCollection_DynamicArray<double> theArrayOfParameters)
    {
        // Rectangular boundaries of two surfaces: [0]:U=Ufirst, [1]:U=Ulast,
        //                                         [2]:V=Vfirst, [3]:V=Vlast
        const int aNumberOfCurves = 4;
        Geom2d_Curve?[] aCurS1Bounds = new Geom2d_Curve?[aNumberOfCurves];
        Geom2d_Curve?[] aCurS2Bounds = new Geom2d_Curve?[aNumberOfCurves];

        theBound1.Get(out double aU1f, out double aV1f, out double aU1l, out double aV1l);
        theBound2.Get(out double aU2f, out double aV2f, out double aU2l, out double aV2l);

        double aDelta = aV1l - aV1f;
        if (Math.Abs(aDelta) > RealSmall())
        {
            if (!Precision.IsInfinite(aU1f))
            {
                aCurS1Bounds[0] = new Geom2d_Line(new gp_Pnt2d(aU1f, aV1f), new gp_Dir2d(gp_Dir2d.D.Y));

                if (!Precision.IsInfinite(aDelta))
                {
                    aCurS1Bounds[0] = new Geom2d_TrimmedCurve(aCurS1Bounds[0]!, 0, aDelta);
                }
            }

            if (!Precision.IsInfinite(aU1l))
            {
                aCurS1Bounds[1] = new Geom2d_Line(new gp_Pnt2d(aU1l, aV1f), new gp_Dir2d(gp_Dir2d.D.Y));
                if (!Precision.IsInfinite(aDelta))
                {
                    aCurS1Bounds[1] = new Geom2d_TrimmedCurve(aCurS1Bounds[1]!, 0, aDelta);
                }
            }
        }

        aDelta = aU1l - aU1f;
        if (Math.Abs(aDelta) > RealSmall())
        {
            if (!Precision.IsInfinite(aV1f))
            {
                aCurS1Bounds[2] = new Geom2d_Line(new gp_Pnt2d(aU1f, aV1f), new gp_Dir2d(gp_Dir2d.D.X));
                if (!Precision.IsInfinite(aDelta))
                {
                    aCurS1Bounds[2] = new Geom2d_TrimmedCurve(aCurS1Bounds[2]!, 0, aDelta);
                }
            }

            if (!Precision.IsInfinite(aV1l))
            {
                aCurS1Bounds[3] = new Geom2d_Line(new gp_Pnt2d(aU1f, aV1l), new gp_Dir2d(gp_Dir2d.D.X));
                if (!Precision.IsInfinite(aDelta))
                {
                    aCurS1Bounds[3] = new Geom2d_TrimmedCurve(aCurS1Bounds[3]!, 0, aDelta);
                }
            }
        }

        aDelta = aV2l - aV2f;
        if (Math.Abs(aDelta) > RealSmall())
        {
            if (!Precision.IsInfinite(aU2f))
            {
                aCurS2Bounds[0] = new Geom2d_Line(new gp_Pnt2d(aU2f, aV2f), new gp_Dir2d(gp_Dir2d.D.Y));
                if (!Precision.IsInfinite(aDelta))
                {
                    aCurS2Bounds[0] = new Geom2d_TrimmedCurve(aCurS2Bounds[0]!, 0, aDelta);
                }
            }

            if (!Precision.IsInfinite(aU2l))
            {
                aCurS2Bounds[1] = new Geom2d_Line(new gp_Pnt2d(aU2l, aV2f), new gp_Dir2d(gp_Dir2d.D.Y));
                if (!Precision.IsInfinite(aDelta))
                {
                    aCurS2Bounds[1] = new Geom2d_TrimmedCurve(aCurS2Bounds[1]!, 0, aDelta);
                }
            }
        }

        aDelta = aU2l - aU2f;
        if (Math.Abs(aDelta) > RealSmall())
        {
            if (!Precision.IsInfinite(aV2f))
            {
                aCurS2Bounds[2] = new Geom2d_Line(new gp_Pnt2d(aU2f, aV2f), new gp_Dir2d(gp_Dir2d.D.X));
                if (!Precision.IsInfinite(aDelta))
                {
                    aCurS2Bounds[2] = new Geom2d_TrimmedCurve(aCurS2Bounds[2]!, 0, aDelta);
                }
            }

            if (!Precision.IsInfinite(aV2l))
            {
                aCurS2Bounds[3] = new Geom2d_Line(new gp_Pnt2d(aU2f, aV2l), new gp_Dir2d(gp_Dir2d.D.X));
                if (!Precision.IsInfinite(aDelta))
                {
                    aCurS2Bounds[3] = new Geom2d_TrimmedCurve(aCurS2Bounds[3]!, 0, aDelta);
                }
            }
        }

        double anIntTol = 10.0 * Precision.Confusion();

        IntersectCurveAndBoundary(theC2d1, aCurS1Bounds, aNumberOfCurves, anIntTol, theArrayOfParameters);

        IntersectCurveAndBoundary(theC2d2, aCurS2Bounds, aNumberOfCurves, anIntTol, theArrayOfParameters);

        // MSVC's order (decision 621): only 0.0 and -0.0 tie.
        std_sort.sort(theArrayOfParameters.AsSpan(), new DoubleLess());
    }

    /// <summary>The degree-1 B-spline through the points ideb to ifin of the walking line (knots 0, 1, 2, ...).</summary>
    public static Geom_Curve MakeBSpline(IntPatch_WLine WL, int ideb, int ifin)
    {
        int nbpnt = ifin - ideb + 1;
        NCollection_Array1<gp_Pnt> poles = new(1, nbpnt);
        NCollection_Array1<double> knots = new(1, nbpnt);
        NCollection_Array1<int> mults = new(1, nbpnt);
        int i = 1, ipidebm1 = ideb;
        for (; i <= nbpnt; ipidebm1++, i++)
        {
            poles[i] = WL.Point(ipidebm1).Value();
            mults[i] = 1;
            knots[i] = i - 1;
        }
        mults[1] = mults[nbpnt] = 2;
        return new Geom_BSplineCurve(poles, knots, mults, 1);
    }

    /// <summary>The degree-1 2D B-spline through the parameters on the first (onFirst) or second surface of the
    /// points ideb to ifin of the walking line.</summary>
    public static Geom2d_BSplineCurve MakeBSpline2d(IntPatch_WLine theWLine, int ideb, int ifin, bool onFirst)
    {
        int nbpnt = ifin - ideb + 1;
        NCollection_Array1<gp_Pnt2d> poles = new(1, nbpnt);
        NCollection_Array1<double> knots = new(1, nbpnt);
        NCollection_Array1<int> mults = new(1, nbpnt);
        int i = 1, ipidebm1 = ideb;
        for (; i <= nbpnt; ipidebm1++, i++)
        {
            double U, V;
            if (onFirst)
            {
                theWLine.Point(ipidebm1).ParametersOnS1(out U, out V);
            }
            else
            {
                theWLine.Point(ipidebm1).ParametersOnS2(out U, out V);
            }
            poles[i].SetCoord(U, V);
            mults[i] = 1;
            knots[i] = i - 1;
        }

        mults[1] = mults[nbpnt] = 2;
        return new Geom2d_BSplineCurve(poles, knots, mults, 1);
    }
}
