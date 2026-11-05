// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Tools.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Tools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>The auxiliary statics of the intersection tools.</summary>
internal static partial class IntersectionTools
{
    /// <summary>0 if the distance between the vertices is less than the sum of their tolerances, -1 otherwise.</summary>
    public static int ComputeVV(in Vertex aV1, in Vertex aV2)
    {
        double tolV1, tolV2, tolSum, d;
        Point3D p1, p2;

        tolV1 = Tool.Tolerance(aV1);
        tolV2 = Tool.Tolerance(aV2);
        tolSum = tolV1 + tolV2;

        p1 = Tool.Pnt(aV1);
        p2 = Tool.Pnt(aV2);
        tolSum = tolSum * tolSum;
        d = p1.SquareDistanceTo(p2);
        if (d < tolSum)
        {
            return 0;
        }
        return -1;
    }

    /// <summary>True if the wire has an INTERNAL edge.</summary>
    public static bool HasInternalEdge(in Wire aW)
    {
        bool bFlag = true;

        Explorer exp = new(aW, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            Orientation or = e.Orientation;
            if (or == Internal)
            {
                return bFlag;
            }
        }
        return !bFlag;
    }

    /// <summary>
    /// Builds a new face on the face's surface (an empty copy of the face taken FORWARD) bounded by the wire. The new
    /// face is unfrozen; the wire is added to it, not copied.
    /// </summary>
    public static void MakeFaceFromWireAndFace(in Wire aW, in Face aF, out Face aFNew)
    {
        Face ff;
        ff = aF;
        ff.Orient(Forward);
        aFNew = ShapeCasts.Face(ff.EmptyCopied());
        BrepBuilder bb = new();
        bb.Add(aFNew, aW);
    }

    /// <summary>The state of the 2D point relative to the face, by a new <see cref="FaceClassifier2D"/> with the face's tolerance.</summary>
    public static Containment ClassifyPointByFace(in Face aF, in Point2D aP2d)
    {
        double faceTolerance;
        Containment state;

        faceTolerance = Tool.Tolerance(aF);
        FaceClassifier2D class2d = new(aF, faceTolerance);
        state = class2d.Perform(aP2d);

        return state;
    }

    /// <summary>True if the edge's point at the parameter is within 1e-6 of one of its vertices (the vertices' tolerances are not used).</summary>
    public static bool IsVertex(in Edge aE, double t)
    {
        double tolV2, d2;
        Vertex v;
        Point3D pv, pt;

        EdgeCurveAdaptor bac = new(aE);
        bac.D0(t, out pt);

        Explorer exp = new(aE, ShapeType.Vertex);
        for (; exp.More(); exp.Next())
        {
            v = ShapeCasts.Vertex(exp.Current());
            _ = Tool.Tolerance(v);
            tolV2 = 1.0e-12; // OCCT: the vertex's tolerance is computed, then overwritten
            pv = Tool.Pnt(v);
            d2 = pv.SquareDistanceTo(pt);
            if (d2 < tolV2)
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>True if the edge's point at the parameter is within the vertex's tolerance.</summary>
    public static bool IsVertex(in Edge aE, in Vertex aV, double t)
    {
        double tolV, tolV2, d2;
        Point3D pv, pt;

        EdgeCurveAdaptor bac = new(aE);
        bac.D0(t, out pt);

        tolV = Tool.Tolerance(aV);
        tolV2 = tolV * tolV;
        pv = Tool.Pnt(aV);
        d2 = pv.SquareDistanceTo(pt);
        return d2 < tolV2;
    }

    /// <summary>True if the middle of the common part's first range, or of its first second range, is at a vertex of its edge.</summary>
    public static bool IsVertex(CommonPrt aCmnPrt)
    {
        bool isVertex;
        double param;

        Edge e1 = aCmnPrt.Edge1();
        ParameterRange r1 = aCmnPrt.Range1();
        param = 0.5 * (r1.First() + r1.Last());
        isVertex = IsVertex(e1, param);

        if (isVertex)
        {
            return true;
        }

        Edge e2 = aCmnPrt.Edge2();
        NCollection_Sequence<ParameterRange> rs2 = aCmnPrt.Ranges2();
        ParameterRange r2 = rs2.Value(1);
        param = 0.5 * (r2.First() + r2.Last());
        isVertex = IsVertex(e2, param);
        return isVertex;
    }

    /// <summary>True if the middle points of the edges' 3D curves are closer than the sum of the edges' tolerances.</summary>
    public static bool IsMiddlePointsEqual(in Edge aE1, in Edge aE2)
    {
        bool bRet;
        double m1, m2, tol1, tol2, sumTol, d2;
        Point3D p1, p2;

        tol1 = Tool.Tolerance(aE1);
        Curve3D c1 = Tool.Curve(aE1, out double f1, out double l1)!;
        m1 = 0.5 * (f1 + l1);
        c1.D0(m1, out p1);

        tol2 = Tool.Tolerance(aE2);
        Curve3D c2 = Tool.Curve(aE2, out double f2, out double l2)!;
        m2 = 0.5 * (f2 + l2);
        c2.D0(m2, out p2);

        sumTol = tol1 + tol2;
        sumTol = sumTol * sumTol;
        d2 = p1.SquareDistanceTo(p2);
        bRet = d2 < sumTol;
        return bRet;
    }

    /// <summary>True if the point is within the vertex's tolerance plus <paramref name="aTolPV"/> plus Precision::Confusion.</summary>
    public static bool IsVertex(in Point3D aP, double aTolPV, in Vertex aV)
    {
        bool bRet;
        double tolV, d, dTol;
        Point3D pv;

        tolV = Tool.Tolerance(aV);
        //
        dTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        tolV = tolV + aTolPV + dTol;
        //
        pv = Tool.Pnt(aV);
        //
        d = pv.SquareDistanceTo(aP);
        tolV = tolV * tolV;
        bRet = (d <= tolV);
        return bRet;
    }

    /// <summary>
    /// A parameter inside [<paramref name="aFirst"/>, <paramref name="aLast"/>], at the fraction 10 e^-pi
    /// (0.43213918) of the range.
    /// </summary>
    public static double IntermediatePoint(double aFirst, double aLast)
    {
        // define parameter division number as 10*e^(-M_PI) = 0.43213918
        const double PAR_T = 0.43213918;
        return (1.0 - PAR_T) * aFirst + PAR_T * aLast;
    }

    /// <summary>
    /// Splits a closed bounded curve in two at its middle (at <see cref="IntermediatePoint"/> for B-spline and Bézier
    /// curves), appending the two halves to <paramref name="aCvs"/>. Returns the number of curves appended (0 or 2).
    /// </summary>
    public static int SplitCurve(in IntersectionCurve ic, NCollection_Sequence<IntersectionCurve> aCvs)
    {
        Curve3D? c3D = ic.Curve();
        if (c3D is null)
        {
            return 0;
        }
        //
        Curve2D? c2D1 = ic.FirstCurve2d();
        Curve2D? c2D2 = ic.SecondCurve2d();
        bool bIsClosed;

        bIsClosed = IsClosed(c3D);
        if (!bIsClosed)
        {
            return 0;
        }

        double f, l, mid;

        //
        f = c3D.FirstParameter;
        l = c3D.LastParameter;
        mid = 0.5 * (f + l);
        CurveAdaptor gac = new(c3D);
        CurveKind ct = gac.GetType();
        if (ct == BSplineCurve || ct == BezierCurve)
        {
            // aMid=0.5*aMid;
            mid = IntermediatePoint(f, l);
        }
        //
        Curve3D c3DNewF, c3DNewL;
        c3DNewF = new TrimmedCurve3D(c3D, f, mid);
        c3DNewL = new TrimmedCurve3D(c3D, mid, l);

        //
        Curve2D? c2D1F = null, c2D1L = null, c2D2F = null, c2D2L = null;
        //
        if (c2D1 is not null)
        {
            c2D1F = new TrimmedCurve2D(c2D1, f, mid);
            c2D1L = new TrimmedCurve2D(c2D1, mid, l);
        }

        if (c2D2 is not null)
        {
            c2D2F = new TrimmedCurve2D(c2D2, f, mid);
            c2D2L = new TrimmedCurve2D(c2D2, mid, l);
        }
        //
        IntersectionCurve ic1 = new(c3DNewF, c2D1F, c2D2F, ic.Tolerance(), ic.TangentialTolerance());
        IntersectionCurve ic2 = new(c3DNewL, c2D1L, c2D2L, ic.Tolerance(), ic.TangentialTolerance());
        //
        aCvs.Append(ic1);
        //
        aCvs.Append(ic2);
        //
        return 2;
    }

    /// <summary>
    /// Keeps the first line and the first line not parallel to it (all curves if one is not a line). Trimmed curves
    /// are replaced by their basis curves in <paramref name="aSIn"/> itself, as OCCT does through a const_cast.
    /// </summary>
    public static void RejectLines(NCollection_Sequence<IntersectionCurve> aSIn, NCollection_Sequence<IntersectionCurve> aSOut)
    {
        int i, j, nb;
        bool bFlag;
        Curve3D? c3D;

        Direction3D d1 = new(), d2;

        aSOut.Clear();

        nb = aSIn.Length();
        for (i = 1; i <= nb; i++)
        {
            c3D = aSIn[i].Curve();
            //
            if (c3D is TrimmedCurve3D gtc)
            {
                c3D = gtc.BasisCurve;
                aSIn[i].SetCurve(c3D); // OCCT writes into its const input
            }
            IntersectionCurve ic = aSIn[i];
            //
            if (c3D is not LineCurve3D gLine)
            {
                aSOut.Clear();
                for (j = 1; j <= nb; j++)
                {
                    aSOut.Append(aSIn.Value(j));
                }
                return;
            }
            //
            Line3D lin = gLine.Line;
            d2 = lin.Direction;
            if (i == 1)
            {
                aSOut.Append(ic);
                d1 = d2;
                continue;
            }

            bFlag = IsDirsCoinside(d1, d2);
            if (!bFlag)
            {
                aSOut.Append(ic);
                return;
            }
        }
    }

    /// <summary>True if the directions are equal or opposite within 0.0002 (as points on the unit sphere).</summary>
    public static bool IsDirsCoinside(in Direction3D d1, in Direction3D d2)
    {
        bool bFlag;
        Point3D p1 = new(d1.X, d1.Y, d1.Z);
        Point3D p2 = new(d2.X, d2.Y, d2.Z);
        double dLim = 0.0002, d;
        d = p1.DistanceTo(p2);
        bFlag = (d < dLim || Math.Abs(2.0 - d) < dLim);
        return bFlag;
    }

    /// <summary>True if the directions are equal or opposite within <paramref name="dLim"/> (as points on the unit sphere).</summary>
    public static bool IsDirsCoinside(in Direction3D d1, in Direction3D d2, double dLim)
    {
        bool bFlag;
        double d;
        //
        Point3D p1 = new(d1.X, d1.Y, d1.Z);
        Point3D p2 = new(d2.X, d2.Y, d2.Z);
        d = p1.DistanceTo(p2);
        bFlag = (d < dLim || Math.Abs(2.0 - d) < dLim);
        return bFlag;
    }

    /// <summary>True if the curve is bounded and its ends are closer than Precision::Confusion.</summary>
    public static bool IsClosed(Curve3D aC3D)
    {
        bool bRet;
        double f, l, dist, pc;
        Point3D p1, p2;

        if (aC3D is not BoundedCurve3D)
        {
            return false;
        }

        f = aC3D.FirstParameter;
        l = aC3D.LastParameter;

        aC3D.D0(f, out p1);
        aC3D.D0(l, out p2);

        //
        pc = global::Kerf.Engine.Foundation.Precision.Confusion();
        pc = pc * pc;
        dist = p1.SquareDistanceTo(p2);
        bRet = dist < pc;
        return bRet;
    }

    /// <summary>
    /// The tolerance of the curve: <paramref name="aTolBase"/>, enlarged for a trimmed parabola by its focal
    /// distance and the trimming parameters.
    /// </summary>
    public static double CurveTolerance(Curve3D? aC3D, double aTolBase)
    {
        double tolReached, tf, tl, tolMin, tolMax;

        tolReached = aTolBase;
        //
        if (aC3D is null)
        {
            return tolReached;
        }
        //
        if (aC3D is not TrimmedCurve3D ct3D)
        {
            return tolReached;
        }
        //
        tolMin = aTolBase;
        tolMax = aTolBase;
        //
        tf = ct3D.FirstParameter;
        tl = ct3D.LastParameter;
        //
        CurveAdaptor gac = new(ct3D);
        CurveKind cType = gac.GetType();
        //
        if (cType == Parabola)
        {
            Curve3D c3DBase = ct3D.BasisCurve;
            ParabolaTolerance(c3DBase, tf, tl, aTolBase, ref tolMin, ref tolMax);
            tolReached = tolMax;
        }
        //
        return tolReached;
    }

    private static void ParabolaTolerance(Curve3D aC3D, double aTf, double aTl, double aTol, ref double aTolMin,
                                          ref double aTolMax)
    {

        aTolMin = aTol;
        aTolMax = aTol;

        if (aC3D is not ParabolaCurve3D gp)
        {
            return;
        }

        int nbPoints;
        double focal, x1, x2, tol1, tol2;
        Point3D pf, pl;
        Parabola3D parab = gp.Parabola;
        Axis3D xAxis = parab.XAxis;
        LineCurve3D gAxis = new(xAxis);

        focal = gp.Focal;
        if (focal == 0.0)
        {
            return;
        }
        //
        // aTol1
        tol1 = aTol;
        x1 = 0.0;
        gp.D0(aTf, out pf);
        ProjectPointOnCurve proj1 = new(pf, gAxis);
        nbPoints = proj1.NbPoints();
        if (nbPoints != 0)
        {
            x1 = proj1.LowerDistanceParameter();
        }
        if (x1 >= 0.0)
        {
            tol1 = aTol * Math.Sqrt(0.5 * x1 / focal);
        }
        if (tol1 == 0.0)
        {
            tol1 = aTol;
        }
        //
        // aTol2
        tol2 = aTol;
        x2 = 0.0;
        gp.D0(aTl, out pl);
        ProjectPointOnCurve proj2 = new(pl, gAxis);
        nbPoints = proj2.NbPoints();
        if (nbPoints != 0)
        {
            x2 = proj2.LowerDistanceParameter();
        }

        if (x2 >= 0.0)
        {
            tol2 = aTol * Math.Sqrt(0.5 * x2 / focal);
        }
        if (tol2 == 0.0)
        {
            tol2 = aTol;
        }
        //
        aTolMax = (tol1 > tol2) ? tol1 : tol2;
        aTolMin = (tol1 < tol2) ? tol1 : tol2;
    }

    /// <summary>
    /// Checks the curve: false if it has no 3D curve or if its box (enlarged by its larger tolerance, added to
    /// <paramref name="box"/>) is thinner than 3 Precision::Confusion.
    /// </summary>
    public static bool CheckCurve(in IntersectionCurve curve, ref Box box)
    {
        Curve3D? c3D = curve.Curve();
        bool bValid = c3D is not null;
        if (!bValid)
        {
            return bValid;
        }
        //
        // Build bounding box for the curve
        Add3dCurve.Add(new CurveAdaptor(c3D!),
                              Max(curve.Tolerance(), curve.TangentialTolerance()),
                              ref box);
        //
        // Estimate the bounding box of the curve comparing it with the
        // minimal length for the curve from which the valid edge can be built -
        // 3*Precision::Confusion():
        // - 2 vertices with the Precision::Confusion() tolerance;
        // - plus Precision::Confusion() as the minimal distance between vertices.
        double tolCmp = 3 * global::Kerf.Engine.Foundation.Precision.Confusion();
        //
        // Check the size of the box using the Bnd_Box::IsThin() method
        // which does not use the gap of the box.
        bValid = !box.IsThin(tolCmp);
        //
        return bValid;
    }

    /// <summary>True if the parameter is closer than the tolerance to an end of the range.</summary>
    public static bool IsOnPave(double aT1, in ParameterRange aRange, double aTolerance)
    {
        bool firstisonpave1, firstisonpave2, bIsOnPave;
        //
        firstisonpave1 = (Math.Abs(aRange.First() - aT1) < aTolerance);
        firstisonpave2 = (Math.Abs(aRange.Last() - aT1) < aTolerance);
        bIsOnPave = (firstisonpave1 || firstisonpave2);
        return bIsOnPave;
    }

    /// <summary>
    /// The parameters of a vertex common part on both edges: the stored vertex parameters where they are in the
    /// ranges, the middles of the ranges otherwise.
    /// </summary>
    public static void VertexParameters(CommonPrt aCPart, out double aT1, out double aT2)
    {
        ParameterRange r1 = aCPart.Range1();
        aT1 = 0.5 * (r1.First() + r1.Last());
        //
        if ((aCPart.VertexParameter1() >= r1.First()) && (aCPart.VertexParameter1() <= r1.Last()))
        {
            aT1 = aCPart.VertexParameter1();
        }
        //
        NCollection_Sequence<ParameterRange> ranges2 = aCPart.Ranges2();
        ParameterRange r2 = ranges2.Value(1);
        aT2 = 0.5 * (r2.First() + r2.Last());
        //
        if ((aCPart.VertexParameter2() >= r2.First()) && (aCPart.VertexParameter2() <= r2.Last()))
        {
            aT2 = aCPart.VertexParameter2();
        }
    }

    /// <summary>The parameter of a vertex common part on the first edge (as <see cref="VertexParameters"/>).</summary>
    public static void VertexParameter(CommonPrt aCPart, out double aT)
    {
        ParameterRange r = aCPart.Range1();
        aT = 0.5 * (r.First() + r.Last());
        if ((aCPart.VertexParameter1() >= r.First()) && (aCPart.VertexParameter1() <= r.Last()))
        {
            aT = aCPart.VertexParameter1();
        }
    }

    /// <summary>True if the parameter is in the range, or within the tolerance of one of its ends.</summary>
    public static bool IsOnPave1(double aTR, in ParameterRange aCPRange, double aTolerance)
    {
        bool bIsOnPave;
        double t1, t2, dT1, dT2;
        //
        t1 = aCPRange.First();
        t2 = aCPRange.Last();
        bIsOnPave = (aTR >= t1 && aTR <= t2);
        if (bIsOnPave)
        {
            return bIsOnPave;
        }
        //
        dT1 = Math.Abs(aTR - t1);
        dT2 = Math.Abs(aTR - t2);
        bIsOnPave = (dT1 <= aTolerance || dT2 <= aTolerance);
        return bIsOnPave;
    }

    /// <summary>True if an end of <paramref name="aR"/> is in <paramref name="aRRef"/> enlarged by the tolerance.</summary>
    public static bool IsInRange(in ParameterRange aRRef, in ParameterRange aR, double aTolerance)
    {
        bool bIsIn;
        //
        aR.Range(out double t1, out double t2);
        aRRef.Range(out double tRef1, out double tRef2);
        //
        tRef1 -= aTolerance;
        tRef2 += aTolerance;
        //
        bIsIn = (t1 >= tRef1 && t1 <= tRef2) || (t2 >= tRef1 && t2 <= tRef2);
        //
        return bIsIn;
    }

    /// <summary>
    /// Intersects the segment [<paramref name="tLin1"/>, <paramref name="tLin2"/>] of the line with the plane.
    /// Returns 0 for an intersection point (the outputs are set), 1 if the segment lies in the plane (both ends within
    /// the sum of the tolerances, signed), 2 if it is on one side, 3 if the intersection is out of the range; the
    /// outputs are written only for 0.
    /// </summary>
    public static int SegPln(in Line3D lin, double tLin1, double tLin2, double tolLin, in Plane3D pln, double tolPln,
                             ref Point3D p, ref double tp, ref double tolP, ref double tPmin, ref double tPmax)
    {
        int iRet;
        double tol, aTP, dist1, dist2, e, h;
        Point3D p1 = new(), p2 = new();
        //
        iRet = 0;
        tol = tolLin + tolPln;
        //
        CoordinateSystem posPln = pln.Position;
        Direction3D dirPln = posPln.Direction;
        Point3D locPln = posPln.Location;
        //
        Direction3D dirLin = lin.Direction;
        Point3D locLin = lin.Location;
        //
        p1 = p1.WithXYZ(locLin.XYZ() + tLin1 * dirLin.XYZ());
        dist1 = dirPln.X * (p1.X - locPln.X) + dirPln.Y * (p1.Y - locPln.Y)
                 + dirPln.Z * (p1.Z - locPln.Z);
        //
        p2 = p2.WithXYZ(locLin.XYZ() + tLin2 * dirLin.XYZ());
        dist2 = dirPln.X * (p2.X - locPln.X) + dirPln.Y * (p2.Y - locPln.Y)
                 + dirPln.Z * (p2.Z - locPln.Z);
        //
        if (dist1 < tol && dist2 < tol)
        {
            iRet = 1; // common block
            return iRet;
        }
        //
        if (dist1 * dist2 > 0.0)
        {
            iRet = 2; // segment lays on one side to the Plane
            return iRet;
        }
        //
        pln.Coefficients(out double a, out double b, out double c, out double d);
        e = a * locLin.X + b * locLin.Y + c * locLin.Z + d;
        h = a * dirLin.X + b * dirLin.Y + c * dirLin.Z;
        aTP = -e / h;
        if (aTP < tLin1 - tol || aTP > tLin2 + tol)
        {
            iRet = 3; // no intersections due to range of the Line
            return iRet;
        }
        //
        tp = aTP;
        p = p.WithXYZ(locLin.XYZ() + aTP * dirLin.XYZ());
        tolP = tol;
        tPmin = tp - tolPln;
        tPmax = tp + tolPln;
        iRet = 0; // intersection point
        return iRet;
    }

    /// <summary>
    /// Computes the maximal distance between the 3D curve and the 2D curve on the surface over [first, last]
    /// (<see cref="CheckCurveOnSurface"/>), enlarged by a relative margin of 1e-5, and the parameter where it
    /// is reached. False if the check fails (the outputs are then 0, where OCCT leaves them unchanged; no caller reads them).
    /// </summary>
    public static bool ComputeTolerance(Curve3D curve3D, Curve2D curve2D, Surface surf, double first, double last,
                                        out double maxDist, out double maxPar) =>
        ComputeTolerance(curve3D, curve2D, surf, first, last, out maxDist, out maxPar, global::Kerf.Engine.Foundation.Precision.PConfusion());

    /// <inheritdoc cref="ComputeTolerance(Curve3D, Curve2D, Surface, double, double, out double, out double)"/>
    public static bool ComputeTolerance(Curve3D curve3D, Curve2D curve2D, Surface surf, double first, double last,
                                        out double maxDist, out double maxPar, double tolRange, bool toRunParallel = false)
    {
        CheckCurveOnSurface cs = new();
        //
        CurveAdaptorBase geomAdaptorCurve = new CurveAdaptor(curve3D, first, last);

        Curve2DAdaptorBase geom2dAdaptorCurve = new CurveAdaptor2D(curve2D, first, last);
        SurfaceAdaptor geomAdaptorSurface = new(surf);

        CurveOnSurface adaptor3dCurveOnSurface = new(geom2dAdaptorCurve, geomAdaptorSurface);

        cs.Init(geomAdaptorCurve, tolRange);
        cs.SetParallel(toRunParallel);
        cs.Perform(adaptor3dCurveOnSurface);
        if (!cs.IsDone)
        {
            // OCCT leaves the outputs unchanged; every caller reads them only on success.
            maxDist = 0.0;
            maxPar = 0.0;
            return false;
        }

        // Obtaining precise result is impossible if we use
        // numeric methods for solution. Therefore, we must provide
        // some margin. Otherwise, in the future
        //(when geometrical properties of the curve will be changed,
        // e.g. after trimming) we will be able to come
        // to the more precise minimum point. As result, this curve with the
        // tolerance computed earlier will become invalid.
        const double eps = (1.0 + 1.0e-5);
        maxDist = eps * cs.MaxDistance();
        maxPar = cs.MaxParameter();
        //
        return true;
    }

    /// <summary>
    /// The parameter range on an edge of tolerance <paramref name="tol1"/> covered by a crossing with an edge of
    /// tolerance <paramref name="tol2"/> at the angle.
    /// </summary>
    public static double ComputeIntRange(double tol1, double tol2, double angle)
    {
        double dt;
        //
        if (Math.Abs(M_PI_2 - angle) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            dt = tol2;
        }
        else
        {
            double a1, a2, anAngle;
            //
            anAngle = (angle > M_PI_2) ? (M_PI - angle) : angle;
            a1 = tol1 * Math.Tan(M_PI_2 - anAngle);
            a2 = tol2 / Math.Sin(anAngle);
            dt = a1 + a2;
        }
        //
        return dt;
    }
}
