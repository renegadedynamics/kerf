// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools3D.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools3D.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
/// 670's audit, rechecked by unit 6B's tests). <see cref="DoSplitSEAMOnFace(in Edge, in Face)"/> and its
/// two-edge overload write the seam's two pcurves into the split edge they are given (plan 2.3).
/// </para>
/// </remarks>
public static class BOPTools_AlgoTools3D
{
    /// <summary>
    /// Makes the split edge a seam of the face: if its pcurve lies on a period boundary of the face's surface, gives it
    /// a second pcurve translated by the period. Writes into <paramref name="aSplit"/>. False if nothing is done.
    /// </summary>
    public static bool DoSplitSEAMOnFace(in Edge aSplit, in Face aF)
    {
        bool bIsUPeriodic, bIsVPeriodic, bIsLeft;
        double anUPeriod = 0.0, anVPeriod = 0.0;
        double aTol, a, b, aT, anU, dU, anU1;
        double aScPr, anV, dV, anV1;
        double aUmin, aUmax, aVmin, aVmax;
        Point2D aP2D;
        Vector2D aVec2D;
        Curve2D aTmpC1, aTmpC2;
        Curve2D C2D1;
        Surface aS;
        BrepBuilder BB = new();
        Edge aSp;
        //
        bIsLeft = false;
        aSp = aSplit;
        aSp.Orientation(Forward);
        aTol = Tool.Tolerance(aSp);
        //
        aS = Tool.Surface(aF)!;
        //
        aS.Bounds(out aUmin, out aUmax, out aVmin, out aVmax);
        //

        bIsUPeriodic = aS.IsUClosed;
        bIsVPeriodic = aS.IsVClosed;

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

            if (aS is not TrimmedSurface aRTS)
            {
                return false;
            }
            else
            {
                Surface aSB;
                //
                aSB = aRTS.BasisSurface();
                bIsUPeriodic = aSB.IsUPeriodic;
                bIsVPeriodic = aSB.IsVPeriodic;
                //

                if (bIsUPeriodic || bIsVPeriodic)
                {
                    anUPeriod = bIsUPeriodic ? aSB.UPeriod() : 0.0;
                    anVPeriod = bIsVPeriodic ? aSB.VPeriod() : 0.0;
                }
                else
                {
                    bool bIsUClosed = aSB.IsUClosed;
                    bool bIsVClosed = aSB.IsVClosed;
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
        C2D1 = Tool.CurveOnSurface(aSp, aF, out a, out b)!;
        //
        aT = BOPTools_AlgoTools2D.IntermediatePoint(a, b);
        C2D1.D1(aT, out aP2D, out aVec2D);
        Direction2D aDir2D1 = new(aVec2D), aDOX = new(Direction2D.D.NX), aDOY = new(Direction2D.D.Y);
        //
        anU = aP2D.X();
        anV = aP2D.Y();
        //
        anU1 = anU;
        anV1 = anV;
        //
        SurfaceAdaptor aGAS = new(aS);
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
        aTmpC1 = (Curve2D)C2D1.Copy();
        TrimmedCurve2D aC1 = new(aTmpC1, a, b);
        //
        aTmpC2 = (Curve2D)C2D1.Copy();
        TrimmedCurve2D aC2 = new(aTmpC2, a, b);
        Vector2D aTrV = new(anU1 - anU, anV1 - anV);
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
    public static bool DoSplitSEAMOnFace(in Edge theEOrigin, in Edge theESplit, in Face theFace)
    {
        if (!Tool.IsClosed(theEOrigin, theFace))
        {
            return false;
        }

        if (Tool.IsClosed(theESplit, theFace))
        {
            return true;
        }

        Edge aESplit = theESplit;
        aESplit.Orientation(Forward);

        Face aFace = theFace;
        aFace.Orientation(Forward);

        Curve2D? aC2DSplit = Tool.CurveOnSurface(aESplit, aFace, out double aTS1, out double aTS2);
        if (aC2DSplit is null)
        {
            return false;
        }

        double aT1, aT2;
        Curve2D aC2D1 =
            Tool.CurveOnSurface(ShapeCasts.Edge(theEOrigin.Oriented(Forward)), aFace, out aT1, out aT2)!;
        Curve2D aC2D2 =
            Tool.CurveOnSurface(ShapeCasts.Edge(theEOrigin.Oriented(Reversed)), aFace, out aT1, out aT2)!;

        double aT = BOPTools_AlgoTools2D.IntermediatePoint(aTS1, aTS2);
        aC2DSplit.D1(aT, out Point2D aPMid, out Vector2D aVTgt);

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

        if (aDist1 > global::Kerf.Engine.Foundation.Precision.PConfusion() && aDist2 > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        // choose the closest and take corresponding point from the opposite
        Point2D aNewPnt = aDist1 < aDist2 ? aC2D2.Value(aProjPC1.LowerDistanceParameter())
                                           : aC2D1.Value(aProjPC2.LowerDistanceParameter());

        Curve2D aTmpC1 = (Curve2D)aC2DSplit.Copy();
        Curve2D aTmpC2 = (Curve2D)aC2DSplit.Copy();

        TrimmedCurve2D aC1 = new(aTmpC1, aTS1, aTS2);
        TrimmedCurve2D aC2 = new(aTmpC2, aTS1, aTS2);

        Vector2D aTrVec = new(aPMid, aNewPnt);
        aC2.Translate(aTrVec);

        Vector2D aVTgtOrigin;
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
            new BrepBuilder().UpdateEdge(aESplit, aC1, aC2, aFace, Tool.Tolerance(aESplit));
        }
        else
        {
            new BrepBuilder().UpdateEdge(aESplit, aC2, aC1, aFace, Tool.Tolerance(aESplit));
        }
        return true;
    }

    /// <summary>
    /// The normal of the face's surface at the edge's intermediate point (through the edge's pcurve), reversed for a
    /// REVERSED face.
    /// </summary>
    public static void GetNormalToFaceOnEdge(in Edge aE, in Face aF, out Direction3D aDNF, IntTools_Context? theContext = null)
    {
        double aT;

        Tool.CurveOnSurface(aE, aF, out double aT1, out double aT2);
        aT = BOPTools_AlgoTools2D.IntermediatePoint(aT1, aT2);

        GetNormalToFaceOnEdge(aE, aF, aT, out aDNF, theContext);

        if (aF.Orientation() == Reversed)
        {
            aDNF.Reverse();
        }
    }

    /// <summary>
    /// The normal of the face's surface (DU ^ DV, whatever the face's orientation) at the parameter of the edge,
    /// through the edge's pcurve (made if missing).
    /// </summary>
    public static void GetNormalToFaceOnEdge(in Edge aE, in Face aF1, double aT, out Direction3D aDNF1,
                                             IntTools_Context? theContext = null)
    {
        double U, V;
        Point2D aP2D;

        Surface aS1 = Tool.Surface(aF1)!;

        BOPTools_AlgoTools2D.CurveOnSurface(aE, aF1, out Curve2D? aC2D1, out _, theContext);

        aC2D1!.D0(aT, out aP2D);
        U = aP2D.X();
        V = aP2D.Y();

        aS1.D1(U, V, out _, out Vector3D aD1U, out Vector3D aD1V);
        Direction3D aDD1U = new(aD1U);
        Direction3D aDD1V = new(aD1V);

        aDNF1 = aDD1U ^ aDD1V;
    }

    /// <summary>
    /// 1 if the two normals coincide in the same sense, -1 if opposite (or orthogonal after coinciding), 0 if they
    /// don't coincide.
    /// </summary>
    public static int SenseFlag(in Direction3D aDNF1, in Direction3D aDNF2)
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
    public static bool GetNormalToSurface(Surface aS, double U, double V, ref Direction3D aDNS)
    {
        aS.D1(U, V, out _, out Vector3D aD1U, out Vector3D aD1V);

        double aLenU = aD1U.SquareMagnitude();
        if (aLenU < Constants.Resolution())
        {
            return false;
        }

        double aLenV = aD1V.SquareMagnitude();
        if (aLenV < Constants.Resolution())
        {
            return false;
        }

        Direction3D aDD1U = new(aD1U);
        Direction3D aDD1V = new(aD1V);

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
    public static bool GetApproxNormalToFaceOnEdge(in Edge aE, in Face aF, double aT, ref Point3D aPNear,
                                                   ref Direction3D aDNF, double aDt2D)
    {
        Point2D aPx2DNear = new();
        int iErr = PointNearEdge(aE, aF, aT, aDt2D, ref aPx2DNear, ref aPNear);
        if (iErr != 1)
        {
            Surface aS = Tool.Surface(aF)!;

            GetNormalToSurface(aS, aPx2DNear.X(), aPx2DNear.Y(), ref aDNF);

            if (aF.Orientation() == Reversed)
            {
                aDNF.Reverse();
            }
        }
        //
        return (iErr == 0);
    }

    /// <summary>As the overload with a step, the step chosen by PointNearEdge and the point checked to be in the face.</summary>
    public static bool GetApproxNormalToFaceOnEdge(in Edge aE, in Face aF, double aT, ref Point3D aPNear,
                                                   ref Direction3D aDNF, IntTools_Context theContext)
    {
        Point2D aPx2DNear = new();
        int iErr = PointNearEdge(aE, aF, aT, ref aPx2DNear, ref aPNear, theContext);
        if (iErr != 1)
        {
            Surface aS = Tool.Surface(aF)!;

            GetNormalToSurface(aS, aPx2DNear.X(), aPx2DNear.Y(), ref aDNF);

            if (aF.Orientation() == Reversed)
            {
                aDNF.Reverse();
            }
        }
        //
        return (iErr == 0);
    }

    /// <summary>As the overload with a step, the point checked to be in the face.</summary>
    public static bool GetApproxNormalToFaceOnEdge(in Edge aE, in Face aF, double aT, double theStep,
                                                   ref Point3D aPNear, ref Direction3D aDNF, IntTools_Context theContext)
    {
        Point2D aPx2DNear = new();
        int iErr = PointNearEdge(aE, aF, aT, theStep, ref aPx2DNear, ref aPNear, theContext);
        if (iErr != 1)
        {
            Surface aS = Tool.Surface(aF)!;

            GetNormalToSurface(aS, aPx2DNear.X(), aPx2DNear.Y(), ref aDNF);

            if (aF.Orientation() == Reversed)
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
    public static int PointNearEdge(in Edge aE, in Face aF, double aT, double aDt2D, ref Point2D aPx2DNear,
                                    ref Point3D aPxNear)
    {
        double aETol, aFTol, transVal;
        SurfaceKind aTS;
        Curve2D? aC2D;
        Surface aS;
        //
        aC2D = Tool.CurveOnSurface(aE, aF, out _, out _);
        int iErr = aC2D is null ? 1 : 0;
        if (iErr != 0)
        {
            return iErr;
        }
        //
        aS = Tool.Surface(aF)!;
        //
        aC2D!.D1(aT, out Point2D aPx2D, out Vector2D aVx2D);
        Direction2D aDx2D = new(aVx2D);

        Direction2D aDP = new();
        aDP.SetCoord(-aDx2D.Y(), aDx2D.X());

        if (aE.Orientation() == Reversed)
        {
            aDP.Reverse();
        }

        if (aF.Orientation() == Reversed)
        {
            aDP.Reverse();
        }
        //
        aETol = Tool.Tolerance(aE);
        aFTol = Tool.Tolerance(aF);
        // NPAL19220
        SurfaceAdaptor aGAS = new(aS);
        aTS = aGAS.GetType();
        if (aTS == SurfaceKind.BSplineSurface)
        {
            if (aETol > 1.0e-5)
            {
                aFTol = aETol;
            }
        }
        if (aETol > 1.0e-5 || aFTol > 1.0e-5)
        {
            //
            if (aTS != SurfaceKind.Sphere)
            {
                Vector2D transVec = new(aDP);
                transVal = aDt2D + aETol + aFTol;
                if (aTS == SurfaceKind.Cylinder)
                { // pkv/909/F8
                    double aR, dT;
                    //
                    Cylinder aCyl = aGAS.Cylinder();
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
    /// tolerances; a point not in the face is replaced by <see cref="PointInFace(in Face, in Edge, double, double, ref Point3D, ref Point2D, IntTools_Context)"/>'s.
    /// Returns 0, 1 (no pcurve) or 2 (no point in the face).
    /// </summary>
    public static int PointNearEdge(in Edge aE, in Face aF, double aT, ref Point2D aPx2DNear, ref Point3D aPxNear,
                                    IntTools_Context theContext)
    {
        double aTolE, aTolF, dTx, dT2D;
        Surface aS;
        SurfaceAdaptor aGAS = new();
        //
        dT2D = 10.0 * MinStepIn2d(); //~1.e-5;
        //
        aS = Tool.Surface(aF)!;
        aGAS.Load(aS);
        if (aGAS.GetType() == SurfaceKind.Cylinder || aGAS.GetType() == SurfaceKind.Sphere)
        {
            dT2D = 10.0 * dT2D;
        }
        //
        aTolE = Tool.Tolerance(aE);
        aTolF = Tool.Tolerance(aF);
        dTx = 2.0 * (aTolE + aTolF);
        if (dTx > dT2D)
        {
            dT2D = dTx;
        }
        //
        int iErr = PointNearEdge(aE, aF, aT, dT2D, ref aPx2DNear, ref aPxNear);
        if ((iErr != 1) && !theContext.IsPointInOnFace(aF, aPx2DNear))
        {
            Point3D aP = new();
            Point2D aP2d = new();
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
    public static int PointNearEdge(in Edge aE, in Face aF, double aT, double theStep, ref Point2D aPx2DNear,
                                    ref Point3D aPxNear, IntTools_Context theContext)
    {
        int iErr = PointNearEdge(aE, aF, aT, theStep, ref aPx2DNear, ref aPxNear);
        if ((iErr != 1) && !theContext.IsPointInOnFace(aF, aPx2DNear))
        {
            Point3D aP = new();
            Point2D aP2d = new();
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
    public static int PointNearEdge(in Edge aE, in Face aF, ref Point2D aPInFace2D, ref Point3D aPInFace,
                                    IntTools_Context theContext)
    {
        double aT;
        //
        // 1. compute parameter on edge
        Tool.Range(aE, out double aT1, out double aT2);
        aT = BOPTools_AlgoTools2D.IntermediatePoint(aT1, aT2);
        //
        // 2. compute point inside the face near the edge
        Face aFF = aF;
        aFF.Orientation(Forward);
        OrientEdgeOnFace(aE, aFF, out Edge aERight);
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
    public static bool IsEmptyShape(in Shape aS)
    {
        bool bHasGeometry = false;
        //
        NCollection_IndexedMap<Shape> myShapes = new(ShapeHasher.Instance);
        //
        Add(aS, myShapes, ref bHasGeometry);

        return !bHasGeometry;
    }

    private static void Add(in Shape aS, NCollection_IndexedMap<Shape> myShapes, ref bool bHasGeometry)
    {
        int anIndex;
        //
        if (bHasGeometry)
        {
            return;
        }
        //
        if (aS.IsNull)
        {
            return;
        }
        //
        Shape aSx = aS;
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
            Iterator anIt = new(aSx, false, false);
            for (; anIt.More(); anIt.Next())
            {
                Shape aSy = anIt.Value();
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

    private static bool HasGeometry(in Shape aS)
    {
        bool bHasGeometry = true;
        ShapeType aType = aS.ShapeType();

        if (aType == ShapeType.Vertex)
        {
            return bHasGeometry;
        }
        //
        else if (aType == ShapeType.Edge)
        {
            BrepTEdge TE = (BrepTEdge)aS.TShape()!;
            NCollection_List<CurveRepresentation>.Iterator itrc = new(TE.Curves());

            while (itrc.More())
            {
                CurveRepresentation CR = itrc.Value();
                if (CR.IsCurve3D)
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
        else if (aType == ShapeType.Face)
        {
            BrepTFace TF = (BrepTFace)aS.TShape()!;
            if (TF.Surface() is not null)
            {
                return bHasGeometry;
            }
            Triangulation? Tr = TF.Triangulation();
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
    public static void OrientEdgeOnFace(in Edge aE, in Face aF, out Edge aERight)
    {
        if (Tool.IsClosed(aE, aF))
        {
            aERight = aE;
            aERight.Orientation(aE.Orientation());

            int iFoundCount = 0;
            Edge anEdge = aE;
            Explorer anExp = new(aF, ShapeType.Edge);

            for (; anExp.More(); anExp.Next())
            {
                Shape aSS = anExp.Current();

                if (aSS.IsSame(aE))
                {
                    anEdge = ShapeCasts.Edge(aSS);
                    iFoundCount++;
                }
            }

            if (iFoundCount == 1)
            {
                aERight = anEdge;
            }
            return;
        }

        Explorer anExp2 = new(aF, ShapeType.Edge);
        for (; anExp2.More(); anExp2.Next())
        {
            Shape aSS = anExp2.Current();
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
    public static int PointInFace(in Face theF, ref Point3D theP, ref Point2D theP2D, IntTools_Context theContext)
    {
        int i, iErr = 1;
        double aUx;
        //
        theContext.UVBounds(theF, out double aUMin, out double aUMax, out _, out _);
        //
        Direction2D aD2D = new(Direction2D.D.Y);
        aUx = IntTools_Tools.IntermediatePoint(aUMin, aUMax);
        //
        for (i = 0; i < 2; ++i)
        {
            Point2D aP2D = new(aUx, 0.0);
            LineCurve2D aL2D = new(aP2D, aD2D);
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
    public static int PointInFace(in Face theF, in Edge theE, double theT, double theDt2D, ref Point3D theP,
                                  ref Point2D theP2D, IntTools_Context theContext)
    {
        int iErr;
        Curve2D? aC2D;
        //
        iErr = 0;
        aC2D = Tool.CurveOnSurface(theE, theF, out _, out _);
        if (aC2D is null)
        {
            iErr = 5;
            return iErr;
        }
        //
        aC2D.D1(theT, out Point2D aP2D, out Vector2D aV2D);
        Direction2D aD2Dx = new(aV2D);
        //
        Direction2D aD2D = new();
        aD2D.SetCoord(-aD2Dx.Y(), aD2Dx.X());
        //
        if (theE.Orientation() == Reversed)
        {
            aD2D.Reverse();
        }
        //
        if (theF.Orientation() == Reversed)
        {
            aD2D.Reverse();
        }
        //
        LineCurve2D aL2D = new(aP2D, aD2D);
        TrimmedCurve2D aL2DTrim = new(aL2D, 0.0, global::Kerf.Engine.Foundation.Precision.Infinite());
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
    public static int PointInFace(in Face theF, Curve2D theL2D, ref Point3D theP, ref Point2D theP2D,
                                  IntTools_Context theContext, double theDt2D = 0.0)
    {
        bool bIsDone, bHasFirstPoint, bHasSecondPoint;
        int iErr, aIH, aNbDomains;
        double aVx, aV1, aV2;
        //
        Geom2dHatch_Hatcher aHatcher = theContext.Hatcher(theF);
        //
        CurveAdaptor2D aHCur = new(theL2D);
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
            Surface aS = Tool.Surface(theF)!;
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
