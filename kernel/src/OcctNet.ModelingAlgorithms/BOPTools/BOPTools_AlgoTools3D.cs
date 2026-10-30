// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools3D.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools3D.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The 3D tools of the Boolean component: normals of faces along edges, points inside faces.</summary>
/// <remarks>
/// <para>
/// Points are reported through <c>ref</c> where OCCT leaves them unwritten on failure (the caller's value is kept).
/// The functions that OCCT gives a context and dereferences unchecked (PointInFace, the PointNearEdge overloads with a
/// context) take a non-null context.
/// </para>
/// <para>
/// PointInFace hatches the face with the context's cached hatcher (<see cref="IntTools_Context.Hatcher"/>), clearing
/// the hatchings first and removing its own at the end; a reused hatcher gives the answers of a fresh one (decision
/// 670's audit, rechecked by unit 6B's tests). <see cref="DoSplitSEAMOnFace(in TopoDS_Edge, in TopoDS_Face)"/> and its
/// two-edge overload write the seam's two pcurves into the split edge they are given (plan 2.3).
/// </para>
/// </remarks>
public static class BOPTools_AlgoTools3D
{
    /// <summary>
    /// Makes the split edge a seam of the face: if its pcurve lies on a period boundary of the face's surface, gives it
    /// a second pcurve translated by the period. Writes into <paramref name="aSplit"/>. False if nothing is done.
    /// </summary>
    public static bool DoSplitSEAMOnFace(in TopoDS_Edge aSplit, in TopoDS_Face aF)
    {
        bool bIsUPeriodic, bIsVPeriodic, bIsLeft;
        double anUPeriod = 0.0, anVPeriod = 0.0;
        double aTol, a, b, aT, anU, dU, anU1;
        double aScPr, anV, dV, anV1;
        double aUmin, aUmax, aVmin, aVmax;
        gp_Pnt2d aP2D;
        gp_Vec2d aVec2D;
        Geom2d_Curve aTmpC1, aTmpC2;
        Geom2d_Curve C2D1;
        Geom_Surface aS;
        BRep_Builder BB = new();
        TopoDS_Edge aSp;
        //
        bIsLeft = false;
        aSp = aSplit;
        aSp.Orientation(TopAbs_FORWARD);
        aTol = BRep_Tool.Tolerance(aSp);
        //
        aS = BRep_Tool.Surface(aF)!;
        //
        aS.Bounds(out aUmin, out aUmax, out aVmin, out aVmax);
        //

        bIsUPeriodic = aS.IsUClosed();
        bIsVPeriodic = aS.IsVClosed();

        if (bIsUPeriodic)
        {
            anUPeriod = aUmax - aUmin;
        }
        if (bIsVPeriodic)
        {
            anVPeriod = aVmax - aVmin;
        }

        //
        if (!bIsUPeriodic && !bIsVPeriodic)
        {

            if (aS is not Geom_RectangularTrimmedSurface aRTS)
            {
                return false;
            }
            else
            {
                Geom_Surface aSB;
                //
                aSB = aRTS.BasisSurface();
                bIsUPeriodic = aSB.IsUPeriodic();
                bIsVPeriodic = aSB.IsVPeriodic();
                //

                if (bIsUPeriodic || bIsVPeriodic)
                {
                    anUPeriod = bIsUPeriodic ? aSB.UPeriod() : 0.0;
                    anVPeriod = bIsVPeriodic ? aSB.VPeriod() : 0.0;
                }
                else
                {
                    bool bIsUClosed = aSB.IsUClosed();
                    bool bIsVClosed = aSB.IsVClosed();
                    aSB.Bounds(out double aGlobalUmin, out double aGlobalUmax, out double aGlobalVmin, out double aGlobalVmax);

                    if (bIsUClosed && Math.Abs(aUmin - aGlobalUmin) < aTol
                        && Math.Abs(aUmax - aGlobalUmax) < aTol)
                    {
                        bIsUPeriodic = true;
                        anUPeriod = aUmax - aUmin;
                    }
                    if (bIsVClosed && Math.Abs(aVmin - aGlobalVmin) < aTol
                        && Math.Abs(aVmax - aGlobalVmax) < aTol)
                    {
                        bIsVPeriodic = true;
                        anVPeriod = aVmax - aVmin;
                    }
                }

                if (!(bIsUPeriodic || bIsVPeriodic))
                {
                    return false;
                }
            } // if !RTS.IsNull
        }
        //
        //---------------------------------------------------
        C2D1 = BRep_Tool.CurveOnSurface(aSp, aF, out a, out b)!;
        //
        aT = BOPTools_AlgoTools2D.IntermediatePoint(a, b);
        C2D1.D1(aT, out aP2D, out aVec2D);
        gp_Dir2d aDir2D1 = new(aVec2D), aDOX = new(gp_Dir2d.D.NX), aDOY = new(gp_Dir2d.D.Y);
        //
        anU = aP2D.X();
        anV = aP2D.Y();
        //
        anU1 = anU;
        anV1 = anV;
        //
        GeomAdaptor_Surface aGAS = new(aS);
        dU = aGAS.UResolution(aTol);
        dV = aGAS.VResolution(aTol);
        //
        if (anUPeriod > 0.0)
        {
            if (Math.Abs(anU - aUmin) < dU)
            {
                bIsLeft = true;
                anU1 = anU + anUPeriod;
            }
            else if (Math.Abs(anU - aUmax) < dU)
            {
                bIsLeft = false;
                anU1 = anU - anUPeriod;
            }
        }
        //
        if (anVPeriod > 0.0)
        {
            if (Math.Abs(anV - aVmin) < dV)
            {
                bIsLeft = true;
                anV1 = anV + anVPeriod;
            }
            else if (Math.Abs(anV - aVmax) < dV)
            {
                bIsLeft = false;
                anV1 = anV - anVPeriod;
            }
        }
        //
        if (anU1 == anU && anV1 == anV)
        {
            return false;
        }
        //
        aScPr = (anU1 == anU) ? aDir2D1 * aDOX : aDir2D1 * aDOY;
        //
        aTmpC1 = (Geom2d_Curve)C2D1.Copy();
        Geom2d_TrimmedCurve aC1 = new(aTmpC1, a, b);
        //
        aTmpC2 = (Geom2d_Curve)C2D1.Copy();
        Geom2d_TrimmedCurve aC2 = new(aTmpC2, a, b);
        gp_Vec2d aTrV = new(anU1 - anU, anV1 - anV);
        aC2.Translate(aTrV);
        //
        if (!bIsLeft)
        {
            if (aScPr < 0.0)
            {
                BB.UpdateEdge(aSp, aC2, aC1, aF, aTol);
            }
            else
            {
                BB.UpdateEdge(aSp, aC1, aC2, aF, aTol);
            }
        }
        else
        {
            if (aScPr < 0.0)
            {
                BB.UpdateEdge(aSp, aC1, aC2, aF, aTol);
            }
            else
            {
                BB.UpdateEdge(aSp, aC2, aC1, aF, aTol);
            }
        }
        return true;
    }

    /// <summary>
    /// Makes the split of a seam edge a seam of the face too: the split's pcurve is matched with one of the original
    /// edge's two pcurves and the other one is made by translation. Writes into <paramref name="theESplit"/>. True if
    /// the split is (or already was) a seam.
    /// </summary>
    public static bool DoSplitSEAMOnFace(in TopoDS_Edge theEOrigin, in TopoDS_Edge theESplit, in TopoDS_Face theFace)
    {
        if (!BRep_Tool.IsClosed(theEOrigin, theFace))
        {
            return false;
        }

        if (BRep_Tool.IsClosed(theESplit, theFace))
        {
            return true;
        }

        TopoDS_Edge aESplit = theESplit;
        aESplit.Orientation(TopAbs_FORWARD);

        TopoDS_Face aFace = theFace;
        aFace.Orientation(TopAbs_FORWARD);

        Geom2d_Curve? aC2DSplit = BRep_Tool.CurveOnSurface(aESplit, aFace, out double aTS1, out double aTS2);
        if (aC2DSplit is null)
        {
            return false;
        }

        double aT1, aT2;
        Geom2d_Curve aC2D1 =
            BRep_Tool.CurveOnSurface(TopoDS.Edge(theEOrigin.Oriented(TopAbs_FORWARD)), aFace, out aT1, out aT2)!;
        Geom2d_Curve aC2D2 =
            BRep_Tool.CurveOnSurface(TopoDS.Edge(theEOrigin.Oriented(TopAbs_REVERSED)), aFace, out aT1, out aT2)!;

        double aT = BOPTools_AlgoTools2D.IntermediatePoint(aTS1, aTS2);
        aC2DSplit.D1(aT, out gp_Pnt2d aPMid, out gp_Vec2d aVTgt);

        // project on original 2d curves
        Geom2dAPI_ProjectPointOnCurve aProjPC1 = new(), aProjPC2 = new();
        aProjPC1.Init(aPMid, aC2D1, aT1, aT2);
        aProjPC2.Init(aPMid, aC2D2, aT1, aT2);

        if (aProjPC1.NbPoints() == 0 && aProjPC2.NbPoints() == 0)
        {
            return false;
        }

        double aDist1 = aProjPC1.NbPoints() != 0 ? aProjPC1.LowerDistance() : RealLast();
        double aDist2 = aProjPC2.NbPoints() != 0 ? aProjPC2.LowerDistance() : RealLast();

        if (aDist1 > Precision.PConfusion() && aDist2 > Precision.PConfusion())
        {
            return false;
        }

        // choose the closest and take corresponding point from the opposite
        gp_Pnt2d aNewPnt = aDist1 < aDist2 ? aC2D2.Value(aProjPC1.LowerDistanceParameter())
                                           : aC2D1.Value(aProjPC2.LowerDistanceParameter());

        Geom2d_Curve aTmpC1 = (Geom2d_Curve)aC2DSplit.Copy();
        Geom2d_Curve aTmpC2 = (Geom2d_Curve)aC2DSplit.Copy();

        Geom2d_TrimmedCurve aC1 = new(aTmpC1, aTS1, aTS2);
        Geom2d_TrimmedCurve aC2 = new(aTmpC2, aTS1, aTS2);

        gp_Vec2d aTrVec = new(aPMid, aNewPnt);
        aC2.Translate(aTrVec);

        gp_Vec2d aVTgtOrigin;
        if (aDist1 < aDist2)
        {
            aC2D1.D1(aProjPC1.LowerDistanceParameter(), out _, out aVTgtOrigin);
        }
        else
        {
            aC2D2.D1(aProjPC2.LowerDistanceParameter(), out _, out aVTgtOrigin);
        }

        double aDot = aVTgt.Dot(aVTgtOrigin);

        if ((aDist1 < aDist2) == (aDot > 0))
        {
            new BRep_Builder().UpdateEdge(aESplit, aC1, aC2, aFace, BRep_Tool.Tolerance(aESplit));
        }
        else
        {
            new BRep_Builder().UpdateEdge(aESplit, aC2, aC1, aFace, BRep_Tool.Tolerance(aESplit));
        }
        return true;
    }

    /// <summary>
    /// The normal of the face's surface at the edge's intermediate point (through the edge's pcurve), reversed for a
    /// REVERSED face.
    /// </summary>
    public static void GetNormalToFaceOnEdge(in TopoDS_Edge aE, in TopoDS_Face aF, out gp_Dir aDNF, IntTools_Context? theContext = null)
    {
        double aT;

        BRep_Tool.CurveOnSurface(aE, aF, out double aT1, out double aT2);
        aT = BOPTools_AlgoTools2D.IntermediatePoint(aT1, aT2);

        GetNormalToFaceOnEdge(aE, aF, aT, out aDNF, theContext);

        if (aF.Orientation() == TopAbs_REVERSED)
        {
            aDNF.Reverse();
        }
    }

    /// <summary>
    /// The normal of the face's surface (DU ^ DV, whatever the face's orientation) at the parameter of the edge,
    /// through the edge's pcurve (made if missing).
    /// </summary>
    public static void GetNormalToFaceOnEdge(in TopoDS_Edge aE, in TopoDS_Face aF1, double aT, out gp_Dir aDNF1,
                                             IntTools_Context? theContext = null)
    {
        double U, V;
        gp_Pnt2d aP2D;

        Geom_Surface aS1 = BRep_Tool.Surface(aF1)!;

        BOPTools_AlgoTools2D.CurveOnSurface(aE, aF1, out Geom2d_Curve? aC2D1, out _, theContext);

        aC2D1!.D0(aT, out aP2D);
        U = aP2D.X();
        V = aP2D.Y();

        aS1.D1(U, V, out _, out gp_Vec aD1U, out gp_Vec aD1V);
        gp_Dir aDD1U = new(aD1U);
        gp_Dir aDD1V = new(aD1V);

        aDNF1 = aDD1U ^ aDD1V;
    }

    /// <summary>
    /// 1 if the two normals coincide in the same sense, -1 if opposite (or orthogonal after coinciding), 0 if they
    /// don't coincide.
    /// </summary>
    public static int SenseFlag(in gp_Dir aDNF1, in gp_Dir aDNF2)
    {
        bool bIsDirsCoinside;
        //
        bIsDirsCoinside = IntTools_Tools.IsDirsCoinside(aDNF1, aDNF2);
        if (!bIsDirsCoinside)
        {
            return 0;
        }

        double aScPr;

        aScPr = aDNF1 * aDNF2;
        if (aScPr < 0.0)
        {
            return -1;
        }
        else if (aScPr > 0.0)
        {
            return 1;
        }
        return -1;
    }

    /// <summary>
    /// The normal DU ^ DV of the surface at (U, V); false (and <paramref name="aDNS"/> unchanged) where a derivative is
    /// null.
    /// </summary>
    public static bool GetNormalToSurface(Geom_Surface aS, double U, double V, ref gp_Dir aDNS)
    {
        aS.D1(U, V, out _, out gp_Vec aD1U, out gp_Vec aD1V);

        double aLenU = aD1U.SquareMagnitude();
        if (aLenU < gp.Resolution())
        {
            return false;
        }

        double aLenV = aD1V.SquareMagnitude();
        if (aLenV < gp.Resolution())
        {
            return false;
        }

        gp_Dir aDD1U = new(aD1U);
        gp_Dir aDD1V = new(aD1V);

        // OCCT: compares DU with itself, so the check always passes (occt-issues.md).
        bool bFlag = IntTools_Tools.IsDirsCoinside(aDD1U, aDD1U);
        if (!bFlag)
        {
            return bFlag;
        }

        aDNS = aDD1U ^ aDD1V;
        return bFlag;
    }

    /// <summary>
    /// A point of the face near the edge at the parameter (stepping <paramref name="aDt2D"/> into the face, no
    /// classification) and the face's normal there. True if the point was found (PointNearEdge returned 0);
    /// <paramref name="aPNear"/> and <paramref name="aDNF"/> are unchanged when the edge has no pcurve on the face.
    /// </summary>
    public static bool GetApproxNormalToFaceOnEdge(in TopoDS_Edge aE, in TopoDS_Face aF, double aT, ref gp_Pnt aPNear,
                                                   ref gp_Dir aDNF, double aDt2D)
    {
        gp_Pnt2d aPx2DNear = new();
        int iErr = PointNearEdge(aE, aF, aT, aDt2D, ref aPx2DNear, ref aPNear);
        if (iErr != 1)
        {
            Geom_Surface aS = BRep_Tool.Surface(aF)!;

            GetNormalToSurface(aS, aPx2DNear.X(), aPx2DNear.Y(), ref aDNF);

            if (aF.Orientation() == TopAbs_REVERSED)
            {
                aDNF.Reverse();
            }
        }
        //
        return (iErr == 0);
    }

    /// <summary>As the overload with a step, the step chosen by PointNearEdge and the point checked to be in the face.</summary>
    public static bool GetApproxNormalToFaceOnEdge(in TopoDS_Edge aE, in TopoDS_Face aF, double aT, ref gp_Pnt aPNear,
                                                   ref gp_Dir aDNF, IntTools_Context theContext)
    {
        gp_Pnt2d aPx2DNear = new();
        int iErr = PointNearEdge(aE, aF, aT, ref aPx2DNear, ref aPNear, theContext);
        if (iErr != 1)
        {
            Geom_Surface aS = BRep_Tool.Surface(aF)!;

            GetNormalToSurface(aS, aPx2DNear.X(), aPx2DNear.Y(), ref aDNF);

            if (aF.Orientation() == TopAbs_REVERSED)
            {
                aDNF.Reverse();
            }
        }
        //
        return (iErr == 0);
    }

    /// <summary>As the overload with a step, the point checked to be in the face.</summary>
    public static bool GetApproxNormalToFaceOnEdge(in TopoDS_Edge aE, in TopoDS_Face aF, double aT, double theStep,
                                                   ref gp_Pnt aPNear, ref gp_Dir aDNF, IntTools_Context theContext)
    {
        gp_Pnt2d aPx2DNear = new();
        int iErr = PointNearEdge(aE, aF, aT, theStep, ref aPx2DNear, ref aPNear, theContext);
        if (iErr != 1)
        {
            Geom_Surface aS = BRep_Tool.Surface(aF)!;

            GetNormalToSurface(aS, aPx2DNear.X(), aPx2DNear.Y(), ref aDNF);

            if (aF.Orientation() == TopAbs_REVERSED)
            {
                aDNF.Reverse();
            }
        }
        //
        return (iErr == 0);
    }

    /// <summary>
    /// The point at the parameter of the edge's pcurve moved by <paramref name="aDt2D"/> (more for edges or faces with
    /// large tolerances) perpendicular to the pcurve, into the face. Returns 1 (outputs unchanged) if the edge has no
    /// pcurve on the face, else 0. No classification.
    /// </summary>
    public static int PointNearEdge(in TopoDS_Edge aE, in TopoDS_Face aF, double aT, double aDt2D, ref gp_Pnt2d aPx2DNear,
                                    ref gp_Pnt aPxNear)
    {
        double aETol, aFTol, transVal;
        GeomAbs_SurfaceType aTS;
        Geom2d_Curve? aC2D;
        Geom_Surface aS;
        //
        aC2D = BRep_Tool.CurveOnSurface(aE, aF, out _, out _);
        int iErr = aC2D is null ? 1 : 0;
        if (iErr != 0)
        {
            return iErr;
        }
        //
        aS = BRep_Tool.Surface(aF)!;
        //
        aC2D!.D1(aT, out gp_Pnt2d aPx2D, out gp_Vec2d aVx2D);
        gp_Dir2d aDx2D = new(aVx2D);

        gp_Dir2d aDP = new();
        aDP.SetCoord(-aDx2D.Y(), aDx2D.X());

        if (aE.Orientation() == TopAbs_REVERSED)
        {
            aDP.Reverse();
        }

        if (aF.Orientation() == TopAbs_REVERSED)
        {
            aDP.Reverse();
        }
        //
        aETol = BRep_Tool.Tolerance(aE);
        aFTol = BRep_Tool.Tolerance(aF);
        // NPAL19220
        GeomAdaptor_Surface aGAS = new(aS);
        aTS = aGAS.GetType();
        if (aTS == GeomAbs_BSplineSurface)
        {
            if (aETol > 1.0e-5)
            {
                aFTol = aETol;
            }
        }
        if (aETol > 1.0e-5 || aFTol > 1.0e-5)
        {
            //
            if (aTS != GeomAbs_Sphere)
            {
                gp_Vec2d transVec = new(aDP);
                transVal = aDt2D + aETol + aFTol;
                if (aTS == GeomAbs_Cylinder)
                { // pkv/909/F8
                    double aR, dT;
                    //
                    gp_Cylinder aCyl = aGAS.Cylinder();
                    aR = aCyl.Radius();
                    dT = 1.0 - transVal / aR;
                    if (dT >= -1 && dT <= 1)
                    {
                        dT = Math.Acos(dT);
                        transVal = dT;
                    }
                }
                //
                transVec.Multiply(transVal);
                aPx2DNear = aPx2D.Translated(transVec);
            }
            else
            {
                aPx2DNear.SetCoord(aPx2D.X() + aDt2D * aDP.X(), aPx2D.Y() + aDt2D * aDP.Y());
            }
        }
        else
        {
            aPx2DNear.SetCoord(aPx2D.X() + aDt2D * aDP.X(), aPx2D.Y() + aDt2D * aDP.Y());
        }
        //
        aS.D0(aPx2DNear.X(), aPx2DNear.Y(), out aPxNear);
        return iErr;
    }

    /// <summary>
    /// As the overload with a step, the step being 1e-4 (1e-3 on cylinders and spheres) or twice the sum of the
    /// tolerances; a point not in the face is replaced by <see cref="PointInFace(in TopoDS_Face, in TopoDS_Edge, double, double, ref gp_Pnt, ref gp_Pnt2d, IntTools_Context)"/>'s.
    /// Returns 0, 1 (no pcurve) or 2 (no point in the face).
    /// </summary>
    public static int PointNearEdge(in TopoDS_Edge aE, in TopoDS_Face aF, double aT, ref gp_Pnt2d aPx2DNear, ref gp_Pnt aPxNear,
                                    IntTools_Context theContext)
    {
        double aTolE, aTolF, dTx, dT2D;
        Geom_Surface aS;
        GeomAdaptor_Surface aGAS = new();
        //
        dT2D = 10.0 * MinStepIn2d(); //~1.e-5;
        //
        aS = BRep_Tool.Surface(aF)!;
        aGAS.Load(aS);
        if (aGAS.GetType() == GeomAbs_Cylinder || aGAS.GetType() == GeomAbs_Sphere)
        {
            dT2D = 10.0 * dT2D;
        }
        //
        aTolE = BRep_Tool.Tolerance(aE);
        aTolF = BRep_Tool.Tolerance(aF);
        dTx = 2.0 * (aTolE + aTolF);
        if (dTx > dT2D)
        {
            dT2D = dTx;
        }
        //
        int iErr = PointNearEdge(aE, aF, aT, dT2D, ref aPx2DNear, ref aPxNear);
        if ((iErr != 1) && !theContext.IsPointInOnFace(aF, aPx2DNear))
        {
            gp_Pnt aP = new();
            gp_Pnt2d aP2d = new();
            //
            iErr = PointInFace(aF, aE, aT, dT2D, ref aP, ref aP2d, theContext);
            if (iErr == 0)
            {
                aPxNear = aP;
                aPx2DNear = aP2d;
            }
            else
            {
                iErr = 2; // point is out of the face
            }
        }
        //
        return iErr;
    }

    /// <summary>
    /// As the overload without a context, then a point not in the face is replaced by PointInFace's. Returns 0, 1 (no
    /// pcurve) or 2 (no point in the face).
    /// </summary>
    public static int PointNearEdge(in TopoDS_Edge aE, in TopoDS_Face aF, double aT, double theStep, ref gp_Pnt2d aPx2DNear,
                                    ref gp_Pnt aPxNear, IntTools_Context theContext)
    {
        int iErr = PointNearEdge(aE, aF, aT, theStep, ref aPx2DNear, ref aPxNear);
        if ((iErr != 1) && !theContext.IsPointInOnFace(aF, aPx2DNear))
        {
            gp_Pnt aP = new();
            gp_Pnt2d aP2d = new();
            //
            iErr = PointInFace(aF, aE, aT, theStep, ref aP, ref aP2d, theContext);
            if (iErr == 0)
            {
                aPxNear = aP;
                aPx2DNear = aP2d;
            }
            else
            {
                iErr = 2; // point is out of the face
            }
        }
        //
        return iErr;
    }

    /// <summary>A point inside the face near the edge's intermediate point, the edge oriented as it is in the face.</summary>
    public static int PointNearEdge(in TopoDS_Edge aE, in TopoDS_Face aF, ref gp_Pnt2d aPInFace2D, ref gp_Pnt aPInFace,
                                    IntTools_Context theContext)
    {
        double aT;
        //
        // 1. compute parameter on edge
        BRep_Tool.Range(aE, out double aT1, out double aT2);
        aT = BOPTools_AlgoTools2D.IntermediatePoint(aT1, aT2);
        //
        // 2. compute point inside the face near the edge
        TopoDS_Face aFF = aF;
        aFF.Orientation(TopAbs_FORWARD);
        OrientEdgeOnFace(aE, aFF, out TopoDS_Edge aERight);
        //
        int iErr = PointNearEdge(aERight, aFF, aT, ref aPInFace2D, ref aPInFace, theContext);
        //
        return iErr;
    }

    /// <summary>The minimal step in 2D (1e-5).</summary>
    public static double MinStepIn2d()
    {
        double dt = 1.0e-5;
        return dt;
    }

    /// <summary>True if no sub-shape of the shape (itself included) has geometry.</summary>
    public static bool IsEmptyShape(in TopoDS_Shape aS)
    {
        bool bHasGeometry = false;
        //
        NCollection_IndexedMap<TopoDS_Shape> myShapes = new(TopTools_ShapeMapHasher.Instance);
        //
        Add(aS, myShapes, ref bHasGeometry);

        return !bHasGeometry;
    }

    private static void Add(in TopoDS_Shape aS, NCollection_IndexedMap<TopoDS_Shape> myShapes, ref bool bHasGeometry)
    {
        int anIndex;
        //
        if (bHasGeometry)
        {
            return;
        }
        //
        if (aS.IsNull())
        {
            return;
        }
        //
        TopoDS_Shape aSx = aS;
        //
        anIndex = myShapes.FindIndex(aSx);
        if (anIndex == 0)
        {
            bHasGeometry = HasGeometry(aSx);
            if (bHasGeometry)
            {
                return;
            }
            //
            TopoDS_Iterator anIt = new(aSx, false, false);
            for (; anIt.More(); anIt.Next())
            {
                TopoDS_Shape aSy = anIt.Value();
                Add(aSy, myShapes, ref bHasGeometry);
                //
                if (bHasGeometry)
                {
                    return;
                }
                //
                myShapes.Add(aSx);
            }
        }
    }

    private static bool HasGeometry(in TopoDS_Shape aS)
    {
        bool bHasGeometry = true;
        TopAbs_ShapeEnum aType = aS.ShapeType();

        if (aType == TopAbs_VERTEX)
        {
            return bHasGeometry;
        }
        //
        else if (aType == TopAbs_EDGE)
        {
            BRep_TEdge TE = (BRep_TEdge)aS.TShape()!;
            NCollection_List<BRep_CurveRepresentation>.Iterator itrc = new(TE.Curves());

            while (itrc.More())
            {
                BRep_CurveRepresentation CR = itrc.Value();
                if (CR.IsCurve3D())
                {
                    if (CR.Curve3D() is not null)
                    {
                        return bHasGeometry;
                    }
                }
                else if (CR.IsCurveOnSurface())
                {
                    return bHasGeometry;
                }
                else if (CR.IsRegularity())
                {
                    return bHasGeometry;
                }
                else if (CR.Polygon3D() is not null) // OCCT: throws Standard_DomainError on any representation
                {                                    // but a 3D polygon (occt-issues.md)
                    return bHasGeometry;
                }
                else if (CR.IsPolygonOnTriangulation())
                {
                    return bHasGeometry;
                }
                else if (CR.IsPolygonOnSurface())
                {
                    return bHasGeometry;
                }
                itrc.Next();
            }
        }
        //
        else if (aType == TopAbs_FACE)
        {
            BRep_TFace TF = (BRep_TFace)aS.TShape()!;
            if (TF.Surface() is not null)
            {
                return bHasGeometry;
            }
            Poly_Triangulation? Tr = TF.Triangulation();
            if (Tr is not null)
            {
                return bHasGeometry;
            }
        }

        return !bHasGeometry;
    }

    /// <summary>
    /// The edge with the orientation it has in the face (for a seam found twice, the edge's own orientation; found once,
    /// the face's occurrence).
    /// </summary>
    public static void OrientEdgeOnFace(in TopoDS_Edge aE, in TopoDS_Face aF, out TopoDS_Edge aERight)
    {
        if (BRep_Tool.IsClosed(aE, aF))
        {
            aERight = aE;
            aERight.Orientation(aE.Orientation());

            int iFoundCount = 0;
            TopoDS_Edge anEdge = aE;
            TopExp_Explorer anExp = new(aF, TopAbs_EDGE);

            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Shape aSS = anExp.Current();

                if (aSS.IsSame(aE))
                {
                    anEdge = TopoDS.Edge(aSS);
                    iFoundCount++;
                }
            }

            if (iFoundCount == 1)
            {
                aERight = anEdge;
            }
            return;
        }

        TopExp_Explorer anExp2 = new(aF, TopAbs_EDGE);
        for (; anExp2.More(); anExp2.Next())
        {
            TopoDS_Shape aSS = anExp2.Current();
            if (aSS.IsSame(aE))
            {
                aERight = aE;
                aERight.Orientation(aSS.Orientation());
                return;
            }
        }
        aERight = aE;
        aERight.Orientation(aE.Orientation());
    }

    /// <summary>
    /// A point inside the face: hatches the face along the V-isoline at the intermediate U (then its mirror) and takes
    /// the middle of the first domain. Returns 0 on success, 1 to 4 (outputs unchanged) on failure.
    /// </summary>
    public static int PointInFace(in TopoDS_Face theF, ref gp_Pnt theP, ref gp_Pnt2d theP2D, IntTools_Context theContext)
    {
        int i, iErr = 1;
        double aUx;
        //
        theContext.UVBounds(theF, out double aUMin, out double aUMax, out _, out _);
        //
        gp_Dir2d aD2D = new(gp_Dir2d.D.Y);
        aUx = IntTools_Tools.IntermediatePoint(aUMin, aUMax);
        //
        for (i = 0; i < 2; ++i)
        {
            gp_Pnt2d aP2D = new(aUx, 0.0);
            Geom2d_Line aL2D = new(aP2D, aD2D);
            iErr = PointInFace(theF, aL2D, ref theP, ref theP2D, theContext);
            if (iErr == 0)
            {
                // done
                break;
            }
            else
            {
                // possible reason - incorrect computation of the 2d box of the face.
                // try to compute the point with the translated line.
                aUx = aUMax - (aUx - aUMin);
            }
        }
        //
        return iErr;
    }

    /// <summary>
    /// A point inside the face along the half-line from the edge's pcurve point at <paramref name="theT"/>,
    /// perpendicular to the pcurve, into the face, <paramref name="theDt2D"/> from the boundary if the domain is long
    /// enough. Returns 0 on success, 5 if the edge has no pcurve, 1 to 4 if hatching fails (outputs unchanged).
    /// </summary>
    public static int PointInFace(in TopoDS_Face theF, in TopoDS_Edge theE, double theT, double theDt2D, ref gp_Pnt theP,
                                  ref gp_Pnt2d theP2D, IntTools_Context theContext)
    {
        int iErr;
        Geom2d_Curve? aC2D;
        //
        iErr = 0;
        aC2D = BRep_Tool.CurveOnSurface(theE, theF, out _, out _);
        if (aC2D is null)
        {
            iErr = 5;
            return iErr;
        }
        //
        aC2D.D1(theT, out gp_Pnt2d aP2D, out gp_Vec2d aV2D);
        gp_Dir2d aD2Dx = new(aV2D);
        //
        gp_Dir2d aD2D = new();
        aD2D.SetCoord(-aD2Dx.Y(), aD2Dx.X());
        //
        if (theE.Orientation() == TopAbs_REVERSED)
        {
            aD2D.Reverse();
        }
        //
        if (theF.Orientation() == TopAbs_REVERSED)
        {
            aD2D.Reverse();
        }
        //
        Geom2d_Line aL2D = new(aP2D, aD2D);
        Geom2d_TrimmedCurve aL2DTrim = new(aL2D, 0.0, Precision.Infinite());
        //
        iErr = PointInFace(theF, aL2DTrim, ref theP, ref theP2D, theContext, theDt2D);
        //
        return iErr;
    }

    /// <summary>
    /// A point inside the face along the 2D line: the line is hatched by the face's cached hatcher and the point is
    /// taken in the first domain, <paramref name="theDt2D"/> from its start if positive and the domain is longer,
    /// else at its intermediate point. Returns 0 on success; 1 if trimming fails, 2 if there is no domain, 3 or 4 if
    /// the domain is open (outputs unchanged).
    /// </summary>
    public static int PointInFace(in TopoDS_Face theF, Geom2d_Curve theL2D, ref gp_Pnt theP, ref gp_Pnt2d theP2D,
                                  IntTools_Context theContext, double theDt2D = 0.0)
    {
        bool bIsDone, bHasFirstPoint, bHasSecondPoint;
        int iErr, aIH, aNbDomains;
        double aVx, aV1, aV2;
        //
        Geom2dHatch_Hatcher aHatcher = theContext.Hatcher(theF);
        //
        Geom2dAdaptor_Curve aHCur = new(theL2D);
        //
        aHatcher.ClrHatchings();
        aIH = aHatcher.AddHatching(aHCur);
        //
        iErr = 0;
        for (;;)
        {
            aHatcher.Trim();
            bIsDone = aHatcher.TrimDone(aIH);
            if (!bIsDone)
            {
                iErr = 1;
                break;
            }
            //
            aHatcher.ComputeDomains(aIH);
            bIsDone = aHatcher.IsDone(aIH);
            if (!bIsDone)
            {
                iErr = 2;
                break;
            }
            //
            aNbDomains = aHatcher.NbDomains(aIH);
            if (aNbDomains == 0)
            {
                iErr = 2;
                break;
            }
            //
            HatchGen_Domain aDomain = aHatcher.Domain(aIH, 1);
            bHasFirstPoint = aDomain.HasFirstPoint();
            if (!bHasFirstPoint)
            {
                iErr = 3;
                break;
            }
            //
            bHasSecondPoint = aDomain.HasSecondPoint();
            if (!bHasSecondPoint)
            {
                iErr = 4;
                break;
            }
            //
            aV1 = aDomain.FirstPoint().Parameter();
            aV2 = aDomain.SecondPoint().Parameter();
            //
            aVx = (theDt2D > 0.0 && (aV2 - aV1) > theDt2D) ? (aV1 + theDt2D)
                                                           : IntTools_Tools.IntermediatePoint(aV1, aV2);
            //
            Geom_Surface aS = BRep_Tool.Surface(theF)!;
            //
            theL2D.D0(aVx, out theP2D);
            aS.D0(theP2D.X(), theP2D.Y(), out theP);
            break;
        }
        //
        aHatcher.RemHatching(aIH);
        return iErr;
    }
}
