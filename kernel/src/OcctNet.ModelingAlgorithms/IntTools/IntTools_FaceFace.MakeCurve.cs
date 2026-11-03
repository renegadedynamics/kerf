// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_FaceFace.cxx (MakeCurve)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Library;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Occt.IntPatch_IType;
using static Kerf.Topology.Containment;

namespace Occt;

public partial class IntTools_FaceFace
{
    /// <summary>
    /// Makes the curves of the intersection line of index Index and appends them to the result: trimmed lines,
    /// conics and circles with their projected 2D curves, approximated (or polygonal) B-splines for walking lines,
    /// approximated restriction lines trimmed on the surfaces' domains.
    /// </summary>
    protected void MakeCurve(int Index, TopolTool dom1, TopolTool dom2, double theToler)
    {
        bool bDone, rejectSurface, reApprox, bAvoidLineConstructor;
        bool ok, bPCurvesOk;
        int i, j, aNbParts;
        double fprm, lprm;
        double Tolpc;
        IntPatch_Line L;
        IntPatch_IType typl;
        Curve3D newc;
        //
        const double TOLCHECK = 1.0e-7;
        const double TOLANGCHECK = 1.0e-6;
        //
        rejectSurface = false;
        reApprox = false;
        //
        bPCurvesOk = true;

    reapprox:;

        Tolpc = myTolApprox;
        bAvoidLineConstructor = false;
        L = myIntersector.Line(Index);
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

            int nbp = aWLine.NbPnts();
            IntSurf_PntOn2S p1 = aWLine.Point(1);
            IntSurf_PntOn2S p2 = aWLine.Point(nbp);

            Point3D P1 = p1.Value();
            Point3D P2 = p2.Value();

            if (P1.SquareDistance(P2) < 1.0e-14)
            {
                bAvoidLineConstructor = false;
            }
        }

        typl = L.ArcType();

        if (typl == IntPatch_Restriction)
        {
            bAvoidLineConstructor = true;
        }

        //
        // Line Constructor
        if (!bAvoidLineConstructor)
        {
            myLConstruct.Perform(L);
            //
            bDone = myLConstruct.IsDone();
            if (!bDone)
            {
                return;
            }

            if (typl != IntPatch_Restriction)
            {
                aNbParts = myLConstruct.NbParts();
                if (aNbParts <= 0)
                {
                    return;
                }
            }
        }
        // Do the Curve

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
                    newc = new LineCurve3D((L as IntPatch_GLine)!.Line());
                }

                else if (typl == IntPatch_Parabola)
                {
                    newc = new ParabolaCurve3D((L as IntPatch_GLine)!.Parabola());
                }

                else
                {
                    newc = new HyperbolaCurve3D((L as IntPatch_GLine)!.Hyperbola());
                }
                //
                aNbParts = myLConstruct.NbParts();
                for (i = 1; i <= aNbParts; i++)
                {
                    bool bFNIt, bLPIt;
                    //
                    myLConstruct.Part(i, out fprm, out lprm);
                    //
                    bFNIt = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(fprm);
                    bLPIt = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(lprm);
                    //
                    if (!bFNIt && !bLPIt)
                    {
                        //
                        IntTools_Curve aCurve = new();
                        //
                        TrimmedCurve3D aCT3D = new(newc, fprm, lprm);
                        aCurve.SetCurve(aCT3D);
                        if (typl == IntPatch_Parabola)
                        {
                            double aTolC = IntTools_Tools.CurveTolerance(aCT3D, myTol);
                            aCurve.SetTolerance(aTolC);
                        }
                        //
                        if (myApprox1)
                        {
                            Curve2D? C2d = null;
                            GeomInt_IntSS.BuildPCurves(fprm, lprm, ref Tolpc, myHS1.Surface(), newc, ref C2d);

                            // OCCT: when both projections fail BuildPCurves dereferences null first (unit 5G)
                            if (C2d is null)
                            {
                                continue;
                            }

                            aCurve.SetFirstCurve2d(new TrimmedCurve2D(C2d, fprm, lprm));
                        }
                        //
                        if (myApprox2)
                        {
                            Curve2D? C2d = null;
                            GeomInt_IntSS.BuildPCurves(fprm, lprm, ref Tolpc, myHS2.Surface(), newc, ref C2d);

                            if (C2d is null)
                            {
                                continue;
                            }

                            aCurve.SetSecondCurve2d(new TrimmedCurve2D(C2d, fprm, lprm));
                        }
                        //
                        mySeqOfCurve.Append(aCurve);
                    } // if (!bFNIt && !bLPIt) {
                    else
                    {
                        //  on regarde si on garde
                        //
                        double aTestPrm, dT = 100.0;
                        //
                        aTestPrm = 0.0;
                        if (bFNIt && !bLPIt)
                        {
                            aTestPrm = lprm - dT;
                        }
                        else if (!bFNIt && bLPIt)
                        {
                            aTestPrm = fprm + dT;
                        }
                        else
                        {
                            // i.e, if (bFNIt && bLPIt)
                            aTestPrm = IntTools_Tools.IntermediatePoint(-dT, dT);
                        }
                        //
                        Point3D ptref = newc.Value(aTestPrm);
                        //
                        SurfaceKind typS1 = myHS1.GetType();
                        SurfaceKind typS2 = myHS2.GetType();
                        if (typS1 == SurfaceOfExtrusion || typS1 == SurfaceKind.OffsetSurface
                            || typS1 == SurfaceOfRevolution || typS2 == SurfaceOfExtrusion
                            || typS2 == SurfaceKind.OffsetSurface || typS2 == SurfaceOfRevolution)
                        {
                            BSplineCurve2D? H1 = null;
                            mySeqOfCurve.Append(new IntTools_Curve(newc, H1, H1));
                            continue;
                        }

                        double u1, v1, u2, v2, Tol;

                        Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
                        Parameters(myHS1, myHS2, ptref, out u1, out v1, out u2, out v2);
                        ok = (dom1.Classify(new Point2D(u1, v1), Tol) != Out);
                        if (ok)
                        {
                            ok = (dom2.Classify(new Point2D(u2, v2), Tol) != Out);
                        }
                        if (ok)
                        {
                            BSplineCurve2D? H1 = null;
                            mySeqOfCurve.Append(new IntTools_Curve(newc, H1, H1));
                        }
                    }
                } // for (i=1; i<=aNbParts; i++) {
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
                    newc = new CircleCurve3D((L as IntPatch_GLine)!.Circle());
                }
                else
                { // IntPatch_Ellipse
                    newc = new EllipseCurve3D((L as IntPatch_GLine)!.Ellipse());
                }
                //
                aNbParts = myLConstruct.NbParts();
                //
                double aPeriod, aNul;
                NCollection_Sequence<double> aSeqFprm = new(), aSeqLprm = new();

                aNul = 0.0;
                aPeriod = M_PI + M_PI;

                for (i = 1; i <= aNbParts; i++)
                {
                    myLConstruct.Part(i, out fprm, out lprm);

                    if (fprm < aNul && lprm > aNul)
                    {
                        // interval that goes through 0. is divided on two intervals;
                        while (fprm < aNul || fprm > aPeriod)
                        {
                            fprm = fprm + aPeriod;
                        }
                        while (lprm < aNul || lprm > aPeriod)
                        {
                            lprm = lprm + aPeriod;
                        }
                        //
                        if ((aPeriod - fprm) > Tolpc)
                        {
                            aSeqFprm.Append(fprm);
                            aSeqLprm.Append(aPeriod);
                        }
                        else
                        {
                            Point3D P1 = newc.Value(fprm);
                            Point3D P2 = newc.Value(aPeriod);

                            if (P1.Distance(P2) > myTol)
                            {
                                double anewpar = fprm;

                                if (ParameterOutOfBoundary(fprm, newc, myFace1, myFace2, lprm, false, myTol,
                                                           out anewpar, myContext!))
                                {
                                    fprm = anewpar;
                                }
                                aSeqFprm.Append(fprm);
                                aSeqLprm.Append(aPeriod);
                            }
                        }

                        //
                        if ((lprm - aNul) > Tolpc)
                        {
                            aSeqFprm.Append(aNul);
                            aSeqLprm.Append(lprm);
                        }
                        else
                        {
                            Point3D P1 = newc.Value(aNul);
                            Point3D P2 = newc.Value(lprm);

                            if (P1.Distance(P2) > myTol)
                            {
                                double anewpar = lprm;

                                if (ParameterOutOfBoundary(lprm, newc, myFace1, myFace2, fprm, true, myTol,
                                                           out anewpar, myContext!))
                                {
                                    lprm = anewpar;
                                }
                                aSeqFprm.Append(aNul);
                                aSeqLprm.Append(lprm);
                            }
                        }
                    }
                    else
                    {
                        // usual interval
                        aSeqFprm.Append(fprm);
                        aSeqLprm.Append(lprm);
                    }
                }
                //
                aNbParts = aSeqFprm.Length();
                for (i = 1; i <= aNbParts; i++)
                {
                    fprm = aSeqFprm.Value(i);
                    lprm = aSeqLprm.Value(i);
                    //
                    double aRealEpsilon = RealEpsilon();
                    if (Math.Abs(fprm) > aRealEpsilon || Math.Abs(lprm - 2.0 * M_PI) > aRealEpsilon)
                    {
                        //==============================================
                        ////
                        IntTools_Curve aCurve = new();
                        TrimmedCurve3D aTC3D = new(newc, fprm, lprm);
                        aCurve.SetCurve(aTC3D);
                        fprm = aTC3D.FirstParameter();
                        lprm = aTC3D.LastParameter();
                        ////
                        if (typl == IntPatch_Circle || typl == IntPatch_Ellipse)
                        { ////
                            if (myApprox1)
                            {
                                Curve2D? C2d = null;
                                GeomInt_IntSS.BuildPCurves(fprm, lprm, myHS1.FirstUParameter(),
                                                           myHS1.LastUParameter(), myHS1.FirstVParameter(),
                                                           myHS1.LastVParameter(), ref Tolpc, myHS1.Surface(), newc,
                                                           ref C2d);
                                aCurve.SetFirstCurve2d(C2d);
                            }

                            if (myApprox2)
                            {
                                Curve2D? C2d = null;
                                GeomInt_IntSS.BuildPCurves(fprm, lprm, myHS2.FirstUParameter(),
                                                           myHS2.LastUParameter(), myHS2.FirstVParameter(),
                                                           myHS2.LastVParameter(), ref Tolpc, myHS2.Surface(), newc,
                                                           ref C2d);
                                aCurve.SetSecondCurve2d(C2d);
                            }
                        }
                        //
                        mySeqOfCurve.Append(aCurve);
                        //==============================================
                    } // if (std::abs(fprm) > RealEpsilon() || std::abs(lprm-2.*M_PI) > RealEpsilon())

                    else
                    {
                        //  on regarde si on garde
                        //
                        if (aNbParts == 1)
                        {
                            //           if (std::abs(fprm) < RealEpsilon() &&  std::abs(lprm-2.*M_PI) <
                            //           RealEpsilon()) {
                            if (Math.Abs(fprm) <= aRealEpsilon && Math.Abs(lprm - 2.0 * M_PI) <= aRealEpsilon)
                            {
                                IntTools_Curve aCurve = new();
                                TrimmedCurve3D aTC3D = new(newc, fprm, lprm);
                                aCurve.SetCurve(aTC3D);
                                fprm = aTC3D.FirstParameter();
                                lprm = aTC3D.LastParameter();

                                if (myApprox1)
                                {
                                    Curve2D? C2d = null;
                                    GeomInt_IntSS.BuildPCurves(fprm, lprm, ref Tolpc, myHS1.Surface(), newc, ref C2d);
                                    aCurve.SetFirstCurve2d(C2d);
                                }

                                if (myApprox2)
                                {
                                    Curve2D? C2d = null;
                                    GeomInt_IntSS.BuildPCurves(fprm, lprm, ref Tolpc, myHS2.Surface(), newc, ref C2d);
                                    aCurve.SetSecondCurve2d(C2d);
                                }
                                //
                                mySeqOfCurve.Append(aCurve);
                                break;
                            }
                        }
                        //
                        double aTwoPIdiv17, u1, v1, u2, v2, Tol;

                        aTwoPIdiv17 = 2.0 * M_PI / 17.0;

                        for (j = 0; j <= 17; j++)
                        {
                            Point3D ptref = newc.Value(j * aTwoPIdiv17);
                            Tol = global::Kerf.Engine.Foundation.Precision.Confusion();

                            Parameters(myHS1, myHS2, ptref, out u1, out v1, out u2, out v2);
                            ok = (dom1.Classify(new Point2D(u1, v1), Tol) != Out);
                            if (ok)
                            {
                                ok = (dom2.Classify(new Point2D(u2, v2), Tol) != Out);
                            }
                            if (ok)
                            {
                                IntTools_Curve aCurve = new();
                                aCurve.SetCurve(newc);
                                //==============================================
                                if (typl == IntPatch_Circle || typl == IntPatch_Ellipse)
                                {

                                    if (myApprox1)
                                    {
                                        Curve2D? C2d = null;
                                        GeomInt_IntSS.BuildPCurves(fprm, lprm, myHS1.FirstUParameter(),
                                                                   myHS1.LastUParameter(), myHS1.FirstVParameter(),
                                                                   myHS1.LastVParameter(), ref Tolpc,
                                                                   myHS1.Surface(), newc, ref C2d);
                                        aCurve.SetFirstCurve2d(C2d);
                                    }

                                    if (myApprox2)
                                    {
                                        Curve2D? C2d = null;
                                        GeomInt_IntSS.BuildPCurves(fprm, lprm, myHS2.FirstUParameter(),
                                                                   myHS2.LastUParameter(), myHS2.FirstVParameter(),
                                                                   myHS2.LastVParameter(), ref Tolpc,
                                                                   myHS2.Surface(), newc, ref C2d);
                                        aCurve.SetSecondCurve2d(C2d);
                                    }
                                } //  end of if (typl == IntPatch_Circle || typl == IntPatch_Ellipse)
                                //==============================================
                                //
                                mySeqOfCurve.Append(aCurve);
                                break;

                            } //  end of if (ok) {
                        } //  end of for (int j=0; j<=17; j++)
                    } //  end of else { on regarde si on garde
                } // for (i=1; i<=myLConstruct.NbParts(); i++)
            } // IntPatch_Circle: IntPatch_Ellipse:
            break;

            case IntPatch_Analytic:
                // This case was processed earlier (in IntPatch_Intersection)
                break;

            case IntPatch_Walking:
            {
                IntPatch_WLine WL = (L as IntPatch_WLine)!;

                // INTTOOLS_FACEFACE_DEBUG's WL->Dump(0) goes.

                //
                int ifprm, ilprm;
                //
                if (!myApprox)
                {
                    aNbParts = 1;
                    if (!bAvoidLineConstructor)
                    {
                        aNbParts = myLConstruct.NbParts();
                    }
                    for (i = 1; i <= aNbParts; ++i)
                    {
                        BSplineCurve2D? H1 = null, H2 = null;
                        Curve3D aBSp;
                        //
                        if (bAvoidLineConstructor)
                        {
                            ifprm = 1;
                            ilprm = WL.NbPnts();
                        }
                        else
                        {
                            myLConstruct.Part(i, out fprm, out lprm);
                            ifprm = (int)fprm;
                            ilprm = (int)lprm;
                        }
                        //
                        if (myApprox1)
                        {
                            H1 = GeomInt_IntSS.MakeBSpline2d(WL, ifprm, ilprm, true);
                        }
                        //
                        if (myApprox2)
                        {
                            H2 = GeomInt_IntSS.MakeBSpline2d(WL, ifprm, ilprm, false);
                        }
                        //
                        aBSp = GeomInt_IntSS.MakeBSpline(WL, ifprm, ilprm);
                        IntTools_Curve aIC = new(aBSp, H1, H2);
                        mySeqOfCurve.Append(aIC);
                    } // for (i=1; i<=aNbParts; ++i) {
                } // if (!myApprox) {
                //
                else
                { // X
                    bool bIsDecomposited;
                    int nbiter, aNbSeqOfL;
                    double tol2d, aTolApproxImp;
                    NCollection_Sequence<IntPatch_Line> aSeqOfL = new();
                    GeomInt_WLApprox theapp3d = new();
                    ParametrizationType aParType = ParametrizationType.ChordLength;
                    //
                    bool anApprox1 = myApprox1;
                    bool anApprox2 = myApprox2;
                    //
                    aTolApproxImp = 1.0e-5;
                    tol2d = myTolApprox;

                    SurfaceKind typs1, typs2;
                    typs1 = myHS1.GetType();
                    typs2 = myHS2.GetType();
                    bool anWithPC = true;

                    if (typs1 == SurfaceKind.Cylinder && typs2 == SurfaceKind.Sphere)
                    {
                        anWithPC = ApproxWithPCurves(myHS1.Cylinder(), myHS2.Sphere());
                    }
                    else if (typs1 == SurfaceKind.Sphere && typs2 == SurfaceKind.Cylinder)
                    {
                        anWithPC = ApproxWithPCurves(myHS2.Cylinder(), myHS1.Sphere());
                    }
                    //
                    if (!anWithPC)
                    {
                        // OCCT overwrites the member: 1e-5 stays for the following lines and Performs
                        myTolApprox = aTolApproxImp; // 1.e-5;
                        anApprox1 = false;
                        anApprox2 = false;
                        //
                        tol2d = myTolApprox;
                    }

                    bIsDecomposited = IntTools_WLineTool.DecompositionOfWLine(WL, myHS1, myHS2, myFace1, myFace2,
                                                                              myLConstruct, bAvoidLineConstructor,
                                                                              myTol, aSeqOfL, myContext!);
                    //
                    aNbSeqOfL = aSeqOfL.Length();
                    //
                    double aTolC = 0.0;
                    if (bIsDecomposited)
                    {
                        nbiter = aNbSeqOfL;
                        aTolC = global::Kerf.Engine.Foundation.Precision.Confusion();
                    }
                    else
                    {
                        nbiter = 1;
                        aNbParts = 1;
                        if (!bAvoidLineConstructor)
                        {
                            aNbParts = myLConstruct.NbParts();
                            nbiter = aNbParts;
                        }
                    }
                    //
                    for (i = 1; i <= nbiter; ++i)
                    {
                        if (bIsDecomposited)
                        {
                            WL = (aSeqOfL.Value(i) as IntPatch_WLine)!;
                            ifprm = 1;
                            ilprm = WL.NbPnts();
                        }
                        else
                        {
                            if (bAvoidLineConstructor)
                            {
                                ifprm = 1;
                                ilprm = WL.NbPnts();
                            }
                            else
                            {
                                myLConstruct.Part(i, out fprm, out lprm);
                                ifprm = (int)fprm;
                                ilprm = (int)lprm;
                            }
                        }

                        bool anApprox = myApprox;
                        if (typs1 == Plane)
                        {
                            anApprox = false;
                            anApprox1 = true;
                        }
                        else if (typs2 == Plane)
                        {
                            anApprox = false;
                            anApprox2 = true;
                        }

                        aParType = ApproxInt_KnotTools.DefineParType(WL, ifprm, ilprm, anApprox, anApprox1, anApprox2);
                        if (myHS1 == myHS2)
                        {
                            theapp3d.SetParameters(myTolApprox, tol2d, 4, 8, 0, 30, false, aParType);
                            rejectSurface = true;
                        }
                        else
                        {
                            if (reApprox && !rejectSurface)
                            {
                                theapp3d.SetParameters(myTolApprox, tol2d, 4, 8, 0, 30, false, aParType);
                            }
                            else
                            {
                                int iDegMax, iDegMin, iNbIter;
                                //
                                ApproxParameters(myHS1, myHS2, out iDegMin, out iDegMax, out iNbIter);
                                theapp3d.SetParameters(myTolApprox, tol2d, iDegMin, iDegMax, iNbIter, 30, true, aParType);
                            }
                        }

                        //-- lbr :
                        //-- Si une des surfaces est un plan , on approxime en 2d
                        //-- sur cette surface et on remonte les points 2d en 3d.
                        if (typs1 == Plane)
                        {
                            theapp3d.Perform(myHS1, myHS2, WL, false, true, myApprox2, ifprm, ilprm);
                        }
                        else if (typs2 == Plane)
                        {
                            theapp3d.Perform(myHS1, myHS2, WL, false, myApprox1, true, ifprm, ilprm);
                        }
                        else
                        {
                            //
                            if (myHS1 != myHS2)
                            {
                                if ((typs1 == SurfaceKind.BezierSurface || typs1 == SurfaceKind.BSplineSurface)
                                    && (typs2 == SurfaceKind.BezierSurface || typs2 == SurfaceKind.BSplineSurface))
                                {

                                    theapp3d.SetParameters(myTolApprox, tol2d, 4, 8, 0, 30, true, aParType);

                                    bool bUseSurfaces;
                                    bUseSurfaces = IntTools_WLineTool.NotUseSurfacesForApprox(myFace1, myFace2, WL,
                                                                                              ifprm, ilprm);
                                    if (bUseSurfaces)
                                    {
                                        // ######
                                        rejectSurface = true;
                                        // ######
                                        theapp3d.SetParameters(myTolApprox, tol2d, 4, 8, 0, 30, false, aParType);
                                    }
                                }
                            }
                            //
                            theapp3d.Perform(myHS1, myHS2, WL, true, anApprox1, anApprox2, ifprm, ilprm);
                        }
                        //
                        if (!theapp3d.IsDone())
                        {
                            BSplineCurve2D? H1 = null;
                            BSplineCurve2D? H2 = null;
                            //
                            Curve3D aBSp = GeomInt_IntSS.MakeBSpline(WL, ifprm, ilprm);
                            //
                            if (myApprox1)
                            {
                                H1 = GeomInt_IntSS.MakeBSpline2d(WL, ifprm, ilprm, true);
                            }
                            //
                            if (myApprox2)
                            {
                                H2 = GeomInt_IntSS.MakeBSpline2d(WL, ifprm, ilprm, false);
                            }
                            //
                            IntTools_Curve aIC = new(aBSp, H1, H2);
                            mySeqOfCurve.Append(aIC);
                        }
                        else
                        {
                            if (typs1 == Plane || typs2 == Plane)
                            {
                                //
                                if (typs1 == SurfaceKind.Torus || typs2 == SurfaceKind.Torus)
                                {
                                    if (aTolC < 1.0e-6)
                                    {
                                        aTolC = 1.0e-6;
                                    }
                                }
                            }
                            //
                            int aNbMultiCurves, nbpoles;
                            aNbMultiCurves = theapp3d.NbMultiCurves();
                            for (j = 1; j <= aNbMultiCurves; j++)
                            {
                                if (typs1 == Plane)
                                {
                                    MultiBSpCurve mbspc = theapp3d.Value(j);
                                    nbpoles = mbspc.NbPoles();

                                    NCollection_Array1<Point2D> tpoles2d = new(1, nbpoles);
                                    NCollection_Array1<Point3D> tpoles = new(1, nbpoles);

                                    mbspc.Curve(1, tpoles2d);
                                    Plane3D Pln = myHS1.Plane();
                                    //
                                    int ik;
                                    for (ik = 1; ik <= nbpoles; ik++)
                                    {
                                        tpoles.SetValue(ik, global::Kerf.Engine.Geometry.ElSLib.Value(tpoles2d.Value(ik).X(), tpoles2d.Value(ik).Y(),
                                                                         Pln));
                                    }
                                    //
                                    BSplineCurve3D BS = new(tpoles, mbspc.Knots(), mbspc.Multiplicities(),
                                                               mbspc.Degree());
                                    CheckBSplineCurve Check = new(BS, TOLCHECK, TOLANGCHECK);
                                    Check.FixTangent(true, true);
                                    //
                                    IntTools_Curve aCurve = new();
                                    aCurve.SetCurve(BS);

                                    if (myApprox1)
                                    {
                                        BSplineCurve2D BS1 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        Check2dBSplineCurve Check1 = new(BS1, TOLCHECK, TOLANGCHECK);
                                        Check1.FixTangent(true, true);
                                        //
                                        // ############################################
                                        if (!rejectSurface && !reApprox)
                                        {
                                            bool isValid = IsCurveValid(BS1);
                                            if (!isValid)
                                            {
                                                reApprox = true;
                                                goto reapprox;
                                            }
                                        }
                                        // ############################################
                                        aCurve.SetFirstCurve2d(BS1);
                                    }

                                    if (myApprox2)
                                    {
                                        mbspc.Curve(2, tpoles2d);

                                        BSplineCurve2D BS2 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        Check2dBSplineCurve newCheck = new(BS2, TOLCHECK, TOLANGCHECK);
                                        newCheck.FixTangent(true, true);

                                        // ###########################################
                                        if (!rejectSurface && !reApprox)
                                        {
                                            bool isValid = IsCurveValid(BS2);
                                            if (!isValid)
                                            {
                                                reApprox = true;
                                                goto reapprox;
                                            }
                                        }
                                        // ###########################################
                                        //
                                        aCurve.SetSecondCurve2d(BS2);
                                    }
                                    //
                                    aCurve.SetTolerance(aTolC);
                                    //
                                    mySeqOfCurve.Append(aCurve);

                                } // if(typs1 == GeomAbs_Plane) {

                                else if (typs2 == Plane)
                                {
                                    MultiBSpCurve mbspc = theapp3d.Value(j);
                                    nbpoles = mbspc.NbPoles();

                                    NCollection_Array1<Point2D> tpoles2d = new(1, nbpoles);
                                    NCollection_Array1<Point3D> tpoles = new(1, nbpoles);
                                    mbspc.Curve((myApprox1) ? 2 : 1, tpoles2d);
                                    Plane3D Pln = myHS2.Plane();
                                    //
                                    int ik;
                                    for (ik = 1; ik <= nbpoles; ik++)
                                    {
                                        tpoles.SetValue(ik, global::Kerf.Engine.Geometry.ElSLib.Value(tpoles2d.Value(ik).X(), tpoles2d.Value(ik).Y(),
                                                                         Pln));
                                    }
                                    //
                                    BSplineCurve3D BS = new(tpoles, mbspc.Knots(), mbspc.Multiplicities(),
                                                               mbspc.Degree());
                                    CheckBSplineCurve Check = new(BS, TOLCHECK, TOLANGCHECK);
                                    Check.FixTangent(true, true);
                                    //
                                    IntTools_Curve aCurve = new();
                                    aCurve.SetCurve(BS);
                                    aCurve.SetTolerance(aTolC);

                                    if (myApprox2)
                                    {
                                        BSplineCurve2D BS1 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        Check2dBSplineCurve Check1 = new(BS1, TOLCHECK, TOLANGCHECK);
                                        Check1.FixTangent(true, true);
                                        //
                                        // ###########################################
                                        if (!rejectSurface && !reApprox)
                                        {
                                            bool isValid = IsCurveValid(BS1);
                                            if (!isValid)
                                            {
                                                reApprox = true;
                                                goto reapprox;
                                            }
                                        }
                                        // ###########################################
                                        bPCurvesOk = CheckPCurve(BS1, myFace2, myContext!);
                                        aCurve.SetSecondCurve2d(BS1);
                                    }

                                    if (myApprox1)
                                    {
                                        mbspc.Curve(1, tpoles2d);
                                        BSplineCurve2D BS2 = new(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                      mbspc.Degree());
                                        Check2dBSplineCurve Check2 = new(BS2, TOLCHECK, TOLANGCHECK);
                                        Check2.FixTangent(true, true);
                                        //
                                        // ###########################################
                                        if (!rejectSurface && !reApprox)
                                        {
                                            bool isValid = IsCurveValid(BS2);
                                            if (!isValid)
                                            {
                                                reApprox = true;
                                                goto reapprox;
                                            }
                                        }
                                        // ###########################################
                                        bPCurvesOk = bPCurvesOk && CheckPCurve(BS2, myFace1, myContext!);
                                        aCurve.SetFirstCurve2d(BS2);
                                    }
                                    //
                                    // if points of the pcurves are out of the faces bounds
                                    // create 3d and 2d curves without approximation
                                    if (!bPCurvesOk)
                                    {
                                        BSplineCurve2D? H1 = null, H2 = null;
                                        bPCurvesOk = true;
                                        //
                                        Curve3D aBSp = GeomInt_IntSS.MakeBSpline(WL, ifprm, ilprm);

                                        if (myApprox1)
                                        {
                                            H1 = GeomInt_IntSS.MakeBSpline2d(WL, ifprm, ilprm, true);
                                            bPCurvesOk = CheckPCurve(H1, myFace1, myContext!);
                                        }

                                        if (myApprox2)
                                        {
                                            H2 = GeomInt_IntSS.MakeBSpline2d(WL, ifprm, ilprm, false);
                                            bPCurvesOk = bPCurvesOk && CheckPCurve(H2, myFace2, myContext!);
                                        }
                                        //
                                        // if pcurves created without approximation are out of the
                                        // faces bounds, use approximated 3d and 2d curves
                                        if (bPCurvesOk)
                                        {
                                            IntTools_Curve aIC = new(aBSp, H1, H2, aTolC);
                                            mySeqOfCurve.Append(aIC);
                                        }
                                        else
                                        {
                                            mySeqOfCurve.Append(aCurve);
                                        }
                                    }
                                    else
                                    {
                                        mySeqOfCurve.Append(aCurve);
                                    }

                                } // else if(typs2 == GeomAbs_Plane)
                                //
                                else
                                { // typs2 != GeomAbs_Plane && typs1 != GeomAbs_Plane
                                    bool bIsValid1, bIsValid2;
                                    BSplineCurve3D BS;
                                    IntTools_Curve aCurve = new();
                                    //
                                    bIsValid1 = true;
                                    bIsValid2 = true;
                                    //
                                    MultiBSpCurve mbspc = theapp3d.Value(j);
                                    nbpoles = mbspc.NbPoles();
                                    NCollection_Array1<Point3D> tpoles = new(1, nbpoles);
                                    mbspc.Curve(1, tpoles);
                                    BS = new BSplineCurve3D(tpoles, mbspc.Knots(), mbspc.Multiplicities(),
                                                               mbspc.Degree());
                                    CheckBSplineCurve Check = new(BS, TOLCHECK, TOLANGCHECK);
                                    Check.FixTangent(true, true);
                                    //
                                    aCurve.SetCurve(BS);
                                    aCurve.SetTolerance(aTolC);
                                    //
                                    if (myApprox1)
                                    {
                                        if (anApprox1)
                                        {
                                            BSplineCurve2D BS1;
                                            NCollection_Array1<Point2D> tpoles2d = new(1, nbpoles);
                                            mbspc.Curve(2, tpoles2d);
                                            //
                                            BS1 = new BSplineCurve2D(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                          mbspc.Degree());
                                            Check2dBSplineCurve newCheck = new(BS1, TOLCHECK, TOLANGCHECK);
                                            newCheck.FixTangent(true, true);
                                            //
                                            if (!reApprox)
                                            {
                                                bIsValid1 = CheckPCurve(BS1, myFace1, myContext!);
                                            }
                                            //
                                            aCurve.SetFirstCurve2d(BS1);
                                        }
                                        else
                                        {
                                            BSplineCurve2D? BS1;
                                            fprm = BS.FirstParameter();
                                            lprm = BS.LastParameter();

                                            Curve2D? C2d = null;
                                            double aTol = myTolApprox;
                                            GeomInt_IntSS.BuildPCurves(fprm, lprm, ref aTol, myHS1.Surface(), BS, ref C2d);
                                            BS1 = C2d as BSplineCurve2D;
                                            aCurve.SetFirstCurve2d(BS1);
                                        }
                                    } // if(myApprox1) {
                                    //
                                    if (myApprox2)
                                    {
                                        if (anApprox2)
                                        {
                                            BSplineCurve2D BS2;
                                            NCollection_Array1<Point2D> tpoles2d = new(1, nbpoles);
                                            mbspc.Curve((myApprox1) ? 3 : 2, tpoles2d);
                                            BS2 = new BSplineCurve2D(tpoles2d, mbspc.Knots(), mbspc.Multiplicities(),
                                                                          mbspc.Degree());
                                            Check2dBSplineCurve newCheck = new(BS2, TOLCHECK, TOLANGCHECK);
                                            newCheck.FixTangent(true, true);
                                            //
                                            if (!reApprox)
                                            {
                                                bIsValid2 = CheckPCurve(BS2, myFace2, myContext!);
                                            }
                                            aCurve.SetSecondCurve2d(BS2);
                                        }
                                        else
                                        {
                                            BSplineCurve2D? BS2;
                                            fprm = BS.FirstParameter();
                                            lprm = BS.LastParameter();

                                            Curve2D? C2d = null;
                                            double aTol = myTolApprox;
                                            GeomInt_IntSS.BuildPCurves(fprm, lprm, ref aTol, myHS2.Surface(), BS, ref C2d);
                                            BS2 = C2d as BSplineCurve2D;
                                            aCurve.SetSecondCurve2d(BS2);
                                        }
                                    } // if(myApprox2) {
                                    if (!bIsValid1 || !bIsValid2)
                                    {
                                        // OCCT overwrites the member (it stays 1e-5), and restarts the whole line
                                        // although its earlier parts are already appended (they are appended again)
                                        myTolApprox = aTolApproxImp; // 1.e-5;
                                        tol2d = myTolApprox;
                                        reApprox = true;
                                        goto reapprox;
                                    }
                                    //
                                    mySeqOfCurve.Append(aCurve);
                                }
                            }
                        }
                    }
                } // else { // X
            } // case IntPatch_Walking:{
            break;

            case IntPatch_Restriction:
            {
                IntPatch_RLine RL = (L as IntPatch_RLine)!;

                // INTTOOLS_FACEFACE_DEBUG's RL->Dump(0) goes.

                Curve3D? aC3d = null;
                Curve2D? aC2d1 = null, aC2d2 = null;
                double aTolReached = 0.0;
                GeomInt_IntSS.TreatRLine(RL, myHS1, myHS2, ref aC3d, ref aC2d1, ref aC2d2, ref aTolReached);

                if (aC3d is null)
                {
                    break;
                }

                Box2d aBox1 = new(), aBox2 = new();

                double aU1f = myHS1.FirstUParameter(), aV1f = myHS1.FirstVParameter(),
                       aU1l = myHS1.LastUParameter(), aV1l = myHS1.LastVParameter();
                double aU2f = myHS2.FirstUParameter(), aV2f = myHS2.FirstVParameter(),
                       aU2l = myHS2.LastUParameter(), aV2l = myHS2.LastVParameter();

                aBox1.Add(new Point2D(aU1f, aV1f));
                aBox1.Add(new Point2D(aU1l, aV1l));
                aBox2.Add(new Point2D(aU2f, aV2f));
                aBox2.Add(new Point2D(aU2l, aV2l));

                NCollection_DynamicArray<double> anArrayOfParameters = new();

                // We consider here that the intersection line is same-parameter-line
                anArrayOfParameters.Append(aC3d.FirstParameter());
                anArrayOfParameters.Append(aC3d.LastParameter());

                GeomInt_IntSS.TrimILineOnSurfBoundaries(aC2d1, aC2d2, aBox1, aBox2, anArrayOfParameters);

                // Intersect with true boundaries. After that, enlarge bounding-boxes in order to
                // correct definition, if point on curve is inscribed in the box.
                aBox1.Enlarge(theToler);
                aBox2.Enlarge(theToler);

                int aNbIntersSolutionsm1 = anArrayOfParameters.Length() - 1;

                // Trim RLine found.
                for (int anInd = 0; anInd < aNbIntersSolutionsm1; anInd++)
                {
                    ref double aParF = ref anArrayOfParameters[anInd];
                    ref double aParL = ref anArrayOfParameters[anInd + 1];

                    if ((aParL - aParF) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        // In order to more precise extending to the boundaries of source curves.
                        if (anInd < aNbIntersSolutionsm1 - 1)
                        {
                            aParL = aParF;
                        }

                        continue;
                    }

                    double aPar = 0.5 * (aParF + aParL);
                    Point2D aPt;

                    Curve2D? aCurv2d1 = null, aCurv2d2 = null;
                    if (aC2d1 is not null)
                    {
                        aC2d1.D0(aPar, out aPt);

                        if (aBox1.IsOut(aPt))
                        {
                            continue;
                        }

                        if (myApprox1)
                        {
                            aCurv2d1 = new TrimmedCurve2D(aC2d1, aParF, aParL);
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
                            aCurv2d2 = new TrimmedCurve2D(aC2d2, aParF, aParL);
                        }
                    }

                    Curve3D aCurv3d = new TrimmedCurve3D(aC3d, aParF, aParL);

                    IntTools_Curve aIC = new(aCurv3d, aCurv2d1, aCurv2d2);
                    mySeqOfCurve.Append(aIC);
                }
            }
            break;
            default:
                break;
        }
    }
}
