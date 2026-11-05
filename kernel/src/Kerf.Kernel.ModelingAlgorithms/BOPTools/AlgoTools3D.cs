// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools3D.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools3D.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification.Hatching;
using Kerf.Engine.Collections;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Tools;

/// <summary>The 3D tools of the Boolean component: normals of faces along edges, points inside faces.</summary>
/// <remarks>
/// <para>
/// Points are reported through <c>ref</c> where OCCT leaves them unwritten on failure (the caller's value is kept).
/// The functions that OCCT gives a context and dereferences unchecked (PointInFace, the PointNearEdge overloads with a
/// context) take a non-null context.
/// </para>
/// <para>
/// PointInFace hatches the face with the context's cached hatcher (<see cref="IntersectionContext.Hatcher"/>), clearing
/// the hatchings first and removing its own at the end; a reused hatcher gives the answers of a fresh one (decision
/// 670's audit, rechecked by unit 6B's tests). <see cref="DoSplitSEAMOnFace(in Edge, in Face)"/> and its
/// two-edge overload write the seam's two pcurves into the split edge they are given (plan 2.3).
/// </para>
/// </remarks>
internal static class AlgoTools3D
{
    /// <summary>
    /// Makes the split edge a seam of the face: if its pcurve lies on a period boundary of the face's surface, gives it
    /// a second pcurve translated by the period. Writes into <paramref name="aSplit"/>. False if nothing is done.
    /// </summary>
    public static bool DoSplitSEAMOnFace(in Edge aSplit, in Face aF)
    {
        bool bIsUPeriodic, bIsVPeriodic, bIsLeft;
        double uPeriod = 0.0, vPeriod = 0.0;
        double tol, a, b, t, u, dU, u1;
        double scPr, v, dV, v1;
        double umin, umax, vmin, vmax;
        Point2D p2D;
        Vector2D vec2D;
        Curve2D tmpC1, tmpC2;
        Curve2D c2D1;
        Surface s;
        BrepBuilder bb = new();
        Edge sp;
        //
        bIsLeft = false;
        sp = aSplit;
        sp.Orient(Forward);
        tol = Tool.Tolerance(sp);
        //
        s = Tool.Surface(aF)!;
        //
        s.ParameterBounds(out umin, out umax, out vmin, out vmax);
        //

        bIsUPeriodic = s.IsUClosed;
        bIsVPeriodic = s.IsVClosed;

        if (bIsUPeriodic)
        {
            uPeriod = umax - umin;
        }
        if (bIsVPeriodic)
        {
            vPeriod = vmax - vmin;
        }

        //
        if (!bIsUPeriodic && !bIsVPeriodic)
        {

            if (s is not TrimmedSurface rts)
            {
                return false;
            }
            else
            {
                Surface sb;
                //
                sb = rts.BasisSurface;
                bIsUPeriodic = sb.IsUPeriodic;
                bIsVPeriodic = sb.IsVPeriodic;
                //

                if (bIsUPeriodic || bIsVPeriodic)
                {
                    uPeriod = bIsUPeriodic ? sb.UPeriod : 0.0;
                    vPeriod = bIsVPeriodic ? sb.VPeriod : 0.0;
                }
                else
                {
                    bool bIsUClosed = sb.IsUClosed;
                    bool bIsVClosed = sb.IsVClosed;
                    sb.ParameterBounds(out double globalUmin, out double globalUmax, out double globalVmin, out double globalVmax);

                    if (bIsUClosed && Math.Abs(umin - globalUmin) < tol
                        && Math.Abs(umax - globalUmax) < tol)
                    {
                        bIsUPeriodic = true;
                        uPeriod = umax - umin;
                    }
                    if (bIsVClosed && Math.Abs(vmin - globalVmin) < tol
                        && Math.Abs(vmax - globalVmax) < tol)
                    {
                        bIsVPeriodic = true;
                        vPeriod = vmax - vmin;
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
        c2D1 = Tool.CurveOnSurface(sp, aF, out a, out b)!;
        //
        t = AlgoTools2D.IntermediatePoint(a, b);
        c2D1.D1(t, out p2D, out vec2D);
        Direction2D dir2D1 = new(vec2D), dox = new(Direction2D.D.NX), doy = new(Direction2D.D.Y);
        //
        u = p2D.X;
        v = p2D.Y;
        //
        u1 = u;
        v1 = v;
        //
        SurfaceAdaptor gas = new(s);
        dU = gas.UResolution(tol);
        dV = gas.VResolution(tol);
        //
        if (uPeriod > 0.0)
        {
            if (Math.Abs(u - umin) < dU)
            {
                bIsLeft = true;
                u1 = u + uPeriod;
            }
            else if (Math.Abs(u - umax) < dU)
            {
                bIsLeft = false;
                u1 = u - uPeriod;
            }
        }
        //
        if (vPeriod > 0.0)
        {
            if (Math.Abs(v - vmin) < dV)
            {
                bIsLeft = true;
                v1 = v + vPeriod;
            }
            else if (Math.Abs(v - vmax) < dV)
            {
                bIsLeft = false;
                v1 = v - vPeriod;
            }
        }
        //
        if (u1 == u && v1 == v)
        {
            return false;
        }
        //
        scPr = (u1 == u) ? dir2D1 * dox : dir2D1 * doy;
        //
        tmpC1 = (Curve2D)c2D1.Copy();
        TrimmedCurve2D c1 = new(tmpC1, a, b);
        //
        tmpC2 = (Curve2D)c2D1.Copy();
        TrimmedCurve2D c2 = new(tmpC2, a, b);
        Vector2D trV = new(u1 - u, v1 - v);
        c2.Translate(trV);
        //
        if (!bIsLeft)
        {
            if (scPr < 0.0)
            {
                bb.UpdateEdge(sp, c2, c1, aF, tol);
            }
            else
            {
                bb.UpdateEdge(sp, c1, c2, aF, tol);
            }
        }
        else
        {
            if (scPr < 0.0)
            {
                bb.UpdateEdge(sp, c1, c2, aF, tol);
            }
            else
            {
                bb.UpdateEdge(sp, c2, c1, aF, tol);
            }
        }
        return true;
    }

    /// <summary>
    /// Makes the split of a seam edge a seam of the face too: the split's pcurve is matched with one of the original
    /// edge's two pcurves and the other one is made by translation. Writes into <paramref name="eSplit"/>. True if
    /// the split is (or already was) a seam.
    /// </summary>
    public static bool DoSplitSEAMOnFace(in Edge eOrigin, in Edge eSplit, in Face face)
    {
        if (!Tool.IsClosed(eOrigin, face))
        {
            return false;
        }

        if (Tool.IsClosed(eSplit, face))
        {
            return true;
        }

        Edge aESplit = eSplit;
        aESplit.Orient(Forward);

        Face aFace = face;
        aFace.Orient(Forward);

        Curve2D? c2DSplit = Tool.CurveOnSurface(aESplit, aFace, out double ts1, out double ts2);
        if (c2DSplit is null)
        {
            return false;
        }

        double t1, t2;
        Curve2D c2D1 =
            Tool.CurveOnSurface(ShapeCasts.Edge(eOrigin.Oriented(Forward)), aFace, out t1, out t2)!;
        Curve2D c2D2 =
            Tool.CurveOnSurface(ShapeCasts.Edge(eOrigin.Oriented(Reversed)), aFace, out t1, out t2)!;

        double t = AlgoTools2D.IntermediatePoint(ts1, ts2);
        c2DSplit.D1(t, out Point2D pMid, out Vector2D vTgt);

        // project on original 2d curves
        ProjectPointOnCurve2D projPC1 = new(), projPC2 = new();
        projPC1.Init(pMid, c2D1, t1, t2);
        projPC2.Init(pMid, c2D2, t1, t2);

        if (projPC1.NbPoints() == 0 && projPC2.NbPoints() == 0)
        {
            return false;
        }

        double dist1 = projPC1.NbPoints() != 0 ? projPC1.LowerDistance() : RealLast();
        double dist2 = projPC2.NbPoints() != 0 ? projPC2.LowerDistance() : RealLast();

        if (dist1 > global::Kerf.Engine.Foundation.Precision.PConfusion() && dist2 > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }

        // choose the closest and take corresponding point from the opposite
        Point2D newPnt = dist1 < dist2 ? c2D2.Value(projPC1.LowerDistanceParameter())
                                           : c2D1.Value(projPC2.LowerDistanceParameter());

        Curve2D tmpC1 = (Curve2D)c2DSplit.Copy();
        Curve2D tmpC2 = (Curve2D)c2DSplit.Copy();

        TrimmedCurve2D c1 = new(tmpC1, ts1, ts2);
        TrimmedCurve2D c2 = new(tmpC2, ts1, ts2);

        Vector2D trVec = new(pMid, newPnt);
        c2.Translate(trVec);

        Vector2D vTgtOrigin;
        if (dist1 < dist2)
        {
            c2D1.D1(projPC1.LowerDistanceParameter(), out _, out vTgtOrigin);
        }
        else
        {
            c2D2.D1(projPC2.LowerDistanceParameter(), out _, out vTgtOrigin);
        }

        double dot = vTgt.Dot(vTgtOrigin);

        if ((dist1 < dist2) == (dot > 0))
        {
            new BrepBuilder().UpdateEdge(aESplit, c1, c2, aFace, Tool.Tolerance(aESplit));
        }
        else
        {
            new BrepBuilder().UpdateEdge(aESplit, c2, c1, aFace, Tool.Tolerance(aESplit));
        }
        return true;
    }

    /// <summary>
    /// The normal of the face's surface at the edge's intermediate point (through the edge's pcurve), reversed for a
    /// REVERSED face.
    /// </summary>
    public static void GetNormalToFaceOnEdge(in Edge aE, in Face aF, out Direction3D aDNF, IntersectionContext? context = null)
    {
        double t;

        Tool.CurveOnSurface(aE, aF, out double t1, out double t2);
        t = AlgoTools2D.IntermediatePoint(t1, t2);

        GetNormalToFaceOnEdge(aE, aF, t, out aDNF, context);

        if (aF.Orientation == Reversed)
        {
            aDNF = aDNF.Reversed();
        }
    }

    /// <summary>
    /// The normal of the face's surface (DU ^ DV, whatever the face's orientation) at the parameter of the edge,
    /// through the edge's pcurve (made if missing).
    /// </summary>
    public static void GetNormalToFaceOnEdge(in Edge aE, in Face aF1, double aT, out Direction3D aDNF1,
                                             IntersectionContext? context = null)
    {
        double u, v;
        Point2D p2D;

        Surface s1 = Tool.Surface(aF1)!;

        AlgoTools2D.CurveOnSurface(aE, aF1, out Curve2D? c2D1, out _, context);

        c2D1!.D0(aT, out p2D);
        u = p2D.X;
        v = p2D.Y;

        s1.D1(u, v, out _, out Vector3D d1U, out Vector3D d1V);
        Direction3D dd1U = new(d1U);
        Direction3D dd1V = new(d1V);

        aDNF1 = dd1U ^ dd1V;
    }

    /// <summary>
    /// 1 if the two normals coincide in the same sense, -1 if opposite (or orthogonal after coinciding), 0 if they
    /// don't coincide.
    /// </summary>
    public static int SenseFlag(in Direction3D aDNF1, in Direction3D aDNF2)
    {
        bool bIsDirsCoinside;
        //
        bIsDirsCoinside = IntersectionTools.IsDirsCoinside(aDNF1, aDNF2);
        if (!bIsDirsCoinside)
        {
            return 0;
        }

        double scPr;

        scPr = aDNF1 * aDNF2;
        if (scPr < 0.0)
        {
            return -1;
        }
        else if (scPr > 0.0)
        {
            return 1;
        }
        return -1;
    }

    /// <summary>
    /// The normal DU ^ DV of the surface at (U, V); false (and <paramref name="aDNS"/> unchanged) where a derivative is
    /// null.
    /// </summary>
    public static bool GetNormalToSurface(Surface aS, double u, double v, ref Direction3D aDNS)
    {
        aS.D1(u, v, out _, out Vector3D d1U, out Vector3D d1V);

        double lenU = d1U.SquareMagnitude;
        if (lenU < Constants.Resolution())
        {
            return false;
        }

        double lenV = d1V.SquareMagnitude;
        if (lenV < Constants.Resolution())
        {
            return false;
        }

        Direction3D dd1U = new(d1U);
        Direction3D dd1V = new(d1V);

        // OCCT: compares DU with itself, so the check always passes (occt-issues.md).
        bool bFlag = IntersectionTools.IsDirsCoinside(dd1U, dd1U);
        if (!bFlag)
        {
            return bFlag;
        }

        aDNS = dd1U ^ dd1V;
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
        Point2D px2DNear = new();
        int iErr = PointNearEdge(aE, aF, aT, aDt2D, ref px2DNear, ref aPNear);
        if (iErr != 1)
        {
            Surface s = Tool.Surface(aF)!;

            GetNormalToSurface(s, px2DNear.X, px2DNear.Y, ref aDNF);

            if (aF.Orientation == Reversed)
            {
                aDNF = aDNF.Reversed();
            }
        }
        //
        return (iErr == 0);
    }

    /// <summary>As the overload with a step, the step chosen by PointNearEdge and the point checked to be in the face.</summary>
    public static bool GetApproxNormalToFaceOnEdge(in Edge aE, in Face aF, double aT, ref Point3D aPNear,
                                                   ref Direction3D aDNF, IntersectionContext context)
    {
        Point2D px2DNear = new();
        int iErr = PointNearEdge(aE, aF, aT, ref px2DNear, ref aPNear, context);
        if (iErr != 1)
        {
            Surface s = Tool.Surface(aF)!;

            GetNormalToSurface(s, px2DNear.X, px2DNear.Y, ref aDNF);

            if (aF.Orientation == Reversed)
            {
                aDNF = aDNF.Reversed();
            }
        }
        //
        return (iErr == 0);
    }

    /// <summary>As the overload with a step, the point checked to be in the face.</summary>
    public static bool GetApproxNormalToFaceOnEdge(in Edge aE, in Face aF, double aT, double step,
                                                   ref Point3D aPNear, ref Direction3D aDNF, IntersectionContext context)
    {
        Point2D px2DNear = new();
        int iErr = PointNearEdge(aE, aF, aT, step, ref px2DNear, ref aPNear, context);
        if (iErr != 1)
        {
            Surface s = Tool.Surface(aF)!;

            GetNormalToSurface(s, px2DNear.X, px2DNear.Y, ref aDNF);

            if (aF.Orientation == Reversed)
            {
                aDNF = aDNF.Reversed();
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
        double eTol, fTol, transVal;
        SurfaceKind ts;
        Curve2D? c2D;
        Surface s;
        //
        c2D = Tool.CurveOnSurface(aE, aF, out _, out _);
        int iErr = c2D is null ? 1 : 0;
        if (iErr != 0)
        {
            return iErr;
        }
        //
        s = Tool.Surface(aF)!;
        //
        c2D!.D1(aT, out Point2D px2D, out Vector2D vx2D);
        Direction2D dx2D = new(vx2D);

        Direction2D dp = new Direction2D().WithCoord(-dx2D.Y, dx2D.X);

        if (aE.Orientation == Reversed)
        {
            dp = dp.Reversed();
        }

        if (aF.Orientation == Reversed)
        {
            dp = dp.Reversed();
        }
        //
        eTol = Tool.Tolerance(aE);
        fTol = Tool.Tolerance(aF);
        // NPAL19220
        SurfaceAdaptor gas = new(s);
        ts = gas.GetType();
        if (ts == SurfaceKind.BSplineSurface)
        {
            if (eTol > 1.0e-5)
            {
                fTol = eTol;
            }
        }
        if (eTol > 1.0e-5 || fTol > 1.0e-5)
        {
            //
            if (ts != SurfaceKind.Sphere)
            {
                Vector2D transVec = new(dp);
                transVal = aDt2D + eTol + fTol;
                if (ts == SurfaceKind.Cylinder)
                { // pkv/909/F8
                    double r, dT;
                    //
                    Cylinder3D cyl = gas.Cylinder();
                    r = cyl.Radius;
                    dT = 1.0 - transVal / r;
                    if (dT >= -1 && dT <= 1)
                    {
                        dT = Math.Acos(dT);
                        transVal = dT;
                    }
                }
                //
                transVec = transVec.Multiplied(transVal);
                aPx2DNear = px2D.Translated(transVec);
            }
            else
            {
                aPx2DNear = aPx2DNear.WithCoord(px2D.X + aDt2D * dp.X, px2D.Y + aDt2D * dp.Y);
            }
        }
        else
        {
            aPx2DNear = aPx2DNear.WithCoord(px2D.X + aDt2D * dp.X, px2D.Y + aDt2D * dp.Y);
        }
        //
        s.D0(aPx2DNear.X, aPx2DNear.Y, out aPxNear);
        return iErr;
    }

    /// <summary>
    /// As the overload with a step, the step being 1e-4 (1e-3 on cylinders and spheres) or twice the sum of the
    /// tolerances; a point not in the face is replaced by <see cref="PointInFace(in Face, in Edge, double, double, ref Point3D, ref Point2D, IntersectionContext)"/>'s.
    /// Returns 0, 1 (no pcurve) or 2 (no point in the face).
    /// </summary>
    public static int PointNearEdge(in Edge aE, in Face aF, double aT, ref Point2D aPx2DNear, ref Point3D aPxNear,
                                    IntersectionContext context)
    {
        double tolE, tolF, dTx, dT2D;
        Surface s;
        SurfaceAdaptor gas = new();
        //
        dT2D = 10.0 * MinStepIn2d(); //~1.e-5;
        //
        s = Tool.Surface(aF)!;
        gas.Load(s);
        if (gas.GetType() == SurfaceKind.Cylinder || gas.GetType() == SurfaceKind.Sphere)
        {
            dT2D = 10.0 * dT2D;
        }
        //
        tolE = Tool.Tolerance(aE);
        tolF = Tool.Tolerance(aF);
        dTx = 2.0 * (tolE + tolF);
        if (dTx > dT2D)
        {
            dT2D = dTx;
        }
        //
        int iErr = PointNearEdge(aE, aF, aT, dT2D, ref aPx2DNear, ref aPxNear);
        if ((iErr != 1) && !context.IsPointInOnFace(aF, aPx2DNear))
        {
            Point3D p = new();
            Point2D p2d = new();
            //
            iErr = PointInFace(aF, aE, aT, dT2D, ref p, ref p2d, context);
            if (iErr == 0)
            {
                aPxNear = p;
                aPx2DNear = p2d;
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
    public static int PointNearEdge(in Edge aE, in Face aF, double aT, double step, ref Point2D aPx2DNear,
                                    ref Point3D aPxNear, IntersectionContext context)
    {
        int iErr = PointNearEdge(aE, aF, aT, step, ref aPx2DNear, ref aPxNear);
        if ((iErr != 1) && !context.IsPointInOnFace(aF, aPx2DNear))
        {
            Point3D p = new();
            Point2D p2d = new();
            //
            iErr = PointInFace(aF, aE, aT, step, ref p, ref p2d, context);
            if (iErr == 0)
            {
                aPxNear = p;
                aPx2DNear = p2d;
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
                                    IntersectionContext context)
    {
        double t;
        //
        // 1. compute parameter on edge
        Tool.Range(aE, out double t1, out double t2);
        t = AlgoTools2D.IntermediatePoint(t1, t2);
        //
        // 2. compute point inside the face near the edge
        Face ff = aF;
        ff.Orient(Forward);
        OrientEdgeOnFace(aE, ff, out Edge eRight);
        //
        int iErr = PointNearEdge(eRight, ff, t, ref aPInFace2D, ref aPInFace, context);
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
        IndexedSet<Shape> myShapes = new(ShapeHasher.Instance);
        //
        Add(aS, myShapes, ref bHasGeometry);

        return !bHasGeometry;
    }

    private static void Add(in Shape aS, IndexedSet<Shape> myShapes, ref bool bHasGeometry)
    {
        int index;
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
        Shape sx = aS;
        //
        index = myShapes.IndexOf(sx) + 1;
        if (index == 0)
        {
            bHasGeometry = HasGeometry(sx);
            if (bHasGeometry)
            {
                return;
            }
            //
            Iterator it = new(sx, false, false);
            for (; it.More(); it.Next())
            {
                Shape sy = it.Value();
                Add(sy, myShapes, ref bHasGeometry);
                //
                if (bHasGeometry)
                {
                    return;
                }
                //
                myShapes.Add(sx);
            }
        }
    }

    private static bool HasGeometry(in Shape aS)
    {
        bool bHasGeometry = true;
        ShapeType type = aS.Type;

        if (type == ShapeType.Vertex)
        {
            return bHasGeometry;
        }
        //
        else if (type == ShapeType.Edge)
        {
            BrepTEdge te = (BrepTEdge)aS.TShape()!;
            NCollection_List<CurveRepresentation>.Iterator itrc = new(te.Curves());

            while (itrc.More())
            {
                CurveRepresentation cr = itrc.Value();
                if (cr.IsCurve3D)
                {
                    if (cr.Curve3D() is not null)
                    {
                        return bHasGeometry;
                    }
                }
                else if (cr.IsCurveOnSurface())
                {
                    return bHasGeometry;
                }
                else if (cr.IsRegularity())
                {
                    return bHasGeometry;
                }
                else if (cr.Polygon3D() is not null) // OCCT: throws Standard_DomainError on any representation
                {                                    // but a 3D polygon (occt-issues.md)
                    return bHasGeometry;
                }
                else if (cr.IsPolygonOnTriangulation())
                {
                    return bHasGeometry;
                }
                else if (cr.IsPolygonOnSurface())
                {
                    return bHasGeometry;
                }
                itrc.Next();
            }
        }
        //
        else if (type == ShapeType.Face)
        {
            BrepTFace tf = (BrepTFace)aS.TShape()!;
            if (tf.Surface() is not null)
            {
                return bHasGeometry;
            }
            Triangulation? tr = tf.Triangulation();
            if (tr is not null)
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
            aERight.Orient(aE.Orientation);

            int iFoundCount = 0;
            Edge edge = aE;
            Explorer exp = new(aF, ShapeType.Edge);

            for (; exp.More(); exp.Next())
            {
                Shape ss = exp.Current();

                if (ss.IsSame(aE))
                {
                    edge = ShapeCasts.Edge(ss);
                    iFoundCount++;
                }
            }

            if (iFoundCount == 1)
            {
                aERight = edge;
            }
            return;
        }

        Explorer exp2 = new(aF, ShapeType.Edge);
        for (; exp2.More(); exp2.Next())
        {
            Shape ss = exp2.Current();
            if (ss.IsSame(aE))
            {
                aERight = aE;
                aERight.Orient(ss.Orientation);
                return;
            }
        }
        aERight = aE;
        aERight.Orient(aE.Orientation);
    }

    /// <summary>
    /// A point inside the face: hatches the face along the V-isoline at the intermediate U (then its mirror) and takes
    /// the middle of the first domain. Returns 0 on success, 1 to 4 (outputs unchanged) on failure.
    /// </summary>
    public static int PointInFace(in Face f, ref Point3D p, ref Point2D p2D, IntersectionContext context)
    {
        int i, iErr = 1;
        double ux;
        //
        context.UVBounds(f, out double uMin, out double uMax, out _, out _);
        //
        Direction2D d2D = new(Direction2D.D.Y);
        ux = IntersectionTools.IntermediatePoint(uMin, uMax);
        //
        for (i = 0; i < 2; ++i)
        {
            Point2D aP2D = new(ux, 0.0);
            LineCurve2D l2D = new(aP2D, d2D);
            iErr = PointInFace(f, l2D, ref p, ref p2D, context);
            if (iErr == 0)
            {
                // done
                break;
            }
            else
            {
                // possible reason - incorrect computation of the 2d box of the face.
                // try to compute the point with the translated line.
                ux = uMax - (ux - uMin);
            }
        }
        //
        return iErr;
    }

    /// <summary>
    /// A point inside the face along the half-line from the edge's pcurve point at <paramref name="t"/>,
    /// perpendicular to the pcurve, into the face, <paramref name="dt2D"/> from the boundary if the domain is long
    /// enough. Returns 0 on success, 5 if the edge has no pcurve, 1 to 4 if hatching fails (outputs unchanged).
    /// </summary>
    public static int PointInFace(in Face f, in Edge e, double t, double dt2D, ref Point3D p,
                                  ref Point2D p2D, IntersectionContext context)
    {
        int iErr;
        Curve2D? c2D;
        //
        iErr = 0;
        c2D = Tool.CurveOnSurface(e, f, out _, out _);
        if (c2D is null)
        {
            iErr = 5;
            return iErr;
        }
        //
        c2D.D1(t, out Point2D aP2D, out Vector2D v2D);
        Direction2D d2Dx = new(v2D);
        //
        Direction2D d2D = new Direction2D().WithCoord(-d2Dx.Y, d2Dx.X);
        //
        if (e.Orientation == Reversed)
        {
            d2D = d2D.Reversed();
        }
        //
        if (f.Orientation == Reversed)
        {
            d2D = d2D.Reversed();
        }
        //
        LineCurve2D l2D = new(aP2D, d2D);
        TrimmedCurve2D l2DTrim = new(l2D, 0.0, global::Kerf.Engine.Foundation.Precision.Infinite());
        //
        iErr = PointInFace(f, l2DTrim, ref p, ref p2D, context, dt2D);
        //
        return iErr;
    }

    /// <summary>
    /// A point inside the face along the 2D line: the line is hatched by the face's cached hatcher and the point is
    /// taken in the first domain, <paramref name="dt2D"/> from its start if positive and the domain is longer,
    /// else at its intermediate point. Returns 0 on success; 1 if trimming fails, 2 if there is no domain, 3 or 4 if
    /// the domain is open (outputs unchanged).
    /// </summary>
    public static int PointInFace(in Face f, Curve2D l2D, ref Point3D p, ref Point2D p2D,
                                  IntersectionContext context, double dt2D = 0.0)
    {
        bool bIsDone, bHasFirstPoint, bHasSecondPoint;
        int iErr, ih, nbDomains;
        double vx, v1, v2;
        //
        Hatcher2D hatcher = context.Hatcher(f);
        //
        CurveAdaptor2D hCur = new(l2D);
        //
        hatcher.ClrHatchings();
        ih = hatcher.AddHatching(hCur);
        //
        iErr = 0;
        for (;;)
        {
            hatcher.Trim();
            bIsDone = hatcher.TrimDone(ih);
            if (!bIsDone)
            {
                iErr = 1;
                break;
            }
            //
            hatcher.ComputeDomains(ih);
            bIsDone = hatcher.IsDone(ih);
            if (!bIsDone)
            {
                iErr = 2;
                break;
            }
            //
            nbDomains = hatcher.NbDomains(ih);
            if (nbDomains == 0)
            {
                iErr = 2;
                break;
            }
            //
            HatchingDomain domain = hatcher.Domain(ih, 1);
            bHasFirstPoint = domain.HasFirstPoint;
            if (!bHasFirstPoint)
            {
                iErr = 3;
                break;
            }
            //
            bHasSecondPoint = domain.HasSecondPoint;
            if (!bHasSecondPoint)
            {
                iErr = 4;
                break;
            }
            //
            v1 = domain.FirstPoint().Parameter();
            v2 = domain.SecondPoint().Parameter();
            //
            vx = (dt2D > 0.0 && (v2 - v1) > dt2D) ? (v1 + dt2D)
                                                           : IntersectionTools.IntermediatePoint(v1, v2);
            //
            Surface s = Tool.Surface(f)!;
            //
            l2D.D0(vx, out p2D);
            s.D0(p2D.X, p2D.Y, out p);
            break;
        }
        //
        hatcher.RemHatching(ih);
        return iErr;
    }
}
