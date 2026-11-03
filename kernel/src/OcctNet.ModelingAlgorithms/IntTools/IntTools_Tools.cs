// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Tools.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_Tools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>The auxiliary statics of the intersection tools.</summary>
public static partial class IntTools_Tools
{
    /// <summary>0 if the distance between the vertices is less than the sum of their tolerances, -1 otherwise.</summary>
    public static int ComputeVV(in Vertex aV1, in Vertex aV2)
    {
        double aTolV1, aTolV2, aTolSum, d;
        Point3D aP1, aP2;

        aTolV1 = Tool.Tolerance(aV1);
        aTolV2 = Tool.Tolerance(aV2);
        aTolSum = aTolV1 + aTolV2;

        aP1 = Tool.Pnt(aV1);
        aP2 = Tool.Pnt(aV2);
        aTolSum = aTolSum * aTolSum;
        d = aP1.SquareDistance(aP2);
        if (d < aTolSum)
        {
            return 0;
        }
        return -1;
    }

    /// <summary>True if the wire has an INTERNAL edge.</summary>
    public static bool HasInternalEdge(in Wire aW)
    {
        bool bFlag = true;

        Explorer anExp = new(aW, ShapeType.Edge);
        for (; anExp.More(); anExp.Next())
        {
            Edge aE = ShapeCasts.Edge(anExp.Current());
            Orientation anOr = aE.Orientation();
            if (anOr == Internal)
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
        Face aFF;
        aFF = aF;
        aFF.Orientation(Forward);
        aFNew = ShapeCasts.Face(aFF.EmptyCopied());
        BrepBuilder BB = new();
        BB.Add(aFNew, aW);
    }

    /// <summary>The state of the 2D point relative to the face, by a new <see cref="IntTools_FClass2d"/> with the face's tolerance.</summary>
    public static Containment ClassifyPointByFace(in Face aF, in Point2D aP2d)
    {
        double aFaceTolerance;
        Containment aState;

        aFaceTolerance = Tool.Tolerance(aF);
        IntTools_FClass2d aClass2d = new(aF, aFaceTolerance);
        aState = aClass2d.Perform(aP2d);

        return aState;
    }

    /// <summary>True if the edge's point at the parameter is within 1e-6 of one of its vertices (the vertices' tolerances are not used).</summary>
    public static bool IsVertex(in Edge aE, double t)
    {
        double aTolV2, d2;
        Vertex aV;
        Point3D aPv, aPt;

        EdgeCurveAdaptor aBAC = new(aE);
        aBAC.D0(t, out aPt);

        Explorer anExp = new(aE, ShapeType.Vertex);
        for (; anExp.More(); anExp.Next())
        {
            aV = ShapeCasts.Vertex(anExp.Current());
            _ = Tool.Tolerance(aV);
            aTolV2 = 1.0e-12; // OCCT: the vertex's tolerance is computed, then overwritten
            aPv = Tool.Pnt(aV);
            d2 = aPv.SquareDistance(aPt);
            if (d2 < aTolV2)
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>True if the edge's point at the parameter is within the vertex's tolerance.</summary>
    public static bool IsVertex(in Edge aE, in Vertex aV, double t)
    {
        double aTolV, aTolV2, d2;
        Point3D aPv, aPt;

        EdgeCurveAdaptor aBAC = new(aE);
        aBAC.D0(t, out aPt);

        aTolV = Tool.Tolerance(aV);
        aTolV2 = aTolV * aTolV;
        aPv = Tool.Pnt(aV);
        d2 = aPv.SquareDistance(aPt);
        return d2 < aTolV2;
    }

    /// <summary>True if the middle of the common part's first range, or of its first second range, is at a vertex of its edge.</summary>
    public static bool IsVertex(IntTools_CommonPrt aCmnPrt)
    {
        bool anIsVertex;
        double aParam;

        Edge aE1 = aCmnPrt.Edge1();
        IntTools_Range aR1 = aCmnPrt.Range1();
        aParam = 0.5 * (aR1.First() + aR1.Last());
        anIsVertex = IsVertex(aE1, aParam);

        if (anIsVertex)
        {
            return true;
        }

        Edge aE2 = aCmnPrt.Edge2();
        NCollection_Sequence<IntTools_Range> aRs2 = aCmnPrt.Ranges2();
        IntTools_Range aR2 = aRs2.Value(1);
        aParam = 0.5 * (aR2.First() + aR2.Last());
        anIsVertex = IsVertex(aE2, aParam);
        return anIsVertex;
    }

    /// <summary>True if the middle points of the edges' 3D curves are closer than the sum of the edges' tolerances.</summary>
    public static bool IsMiddlePointsEqual(in Edge aE1, in Edge aE2)
    {
        bool bRet;
        double m1, m2, aTol1, aTol2, aSumTol, aD2;
        Point3D aP1, aP2;

        aTol1 = Tool.Tolerance(aE1);
        Curve3D C1 = Tool.Curve(aE1, out double f1, out double l1)!;
        m1 = 0.5 * (f1 + l1);
        C1.D0(m1, out aP1);

        aTol2 = Tool.Tolerance(aE2);
        Curve3D C2 = Tool.Curve(aE2, out double f2, out double l2)!;
        m2 = 0.5 * (f2 + l2);
        C2.D0(m2, out aP2);

        aSumTol = aTol1 + aTol2;
        aSumTol = aSumTol * aSumTol;
        aD2 = aP1.SquareDistance(aP2);
        bRet = aD2 < aSumTol;
        return bRet;
    }

    /// <summary>True if the point is within the vertex's tolerance plus <paramref name="aTolPV"/> plus Precision::Confusion.</summary>
    public static bool IsVertex(in Point3D aP, double aTolPV, in Vertex aV)
    {
        bool bRet;
        double aTolV, aD, dTol;
        Point3D aPv;

        aTolV = Tool.Tolerance(aV);
        //
        dTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        aTolV = aTolV + aTolPV + dTol;
        //
        aPv = Tool.Pnt(aV);
        //
        aD = aPv.SquareDistance(aP);
        aTolV = aTolV * aTolV;
        bRet = (aD <= aTolV);
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
    public static int SplitCurve(in IntTools_Curve IC, NCollection_Sequence<IntTools_Curve> aCvs)
    {
        Curve3D? aC3D = IC.Curve();
        if (aC3D is null)
        {
            return 0;
        }
        //
        Curve2D? aC2D1 = IC.FirstCurve2d();
        Curve2D? aC2D2 = IC.SecondCurve2d();
        bool bIsClosed;

        bIsClosed = IsClosed(aC3D);
        if (!bIsClosed)
        {
            return 0;
        }

        double aF, aL, aMid;

        //
        aF = aC3D.FirstParameter();
        aL = aC3D.LastParameter();
        aMid = 0.5 * (aF + aL);
        CurveAdaptor aGAC = new(aC3D);
        CurveKind aCT = aGAC.GetType();
        if (aCT == BSplineCurve || aCT == BezierCurve)
        {
            // aMid=0.5*aMid;
            aMid = IntermediatePoint(aF, aL);
        }
        //
        Curve3D aC3DNewF, aC3DNewL;
        aC3DNewF = new TrimmedCurve3D(aC3D, aF, aMid);
        aC3DNewL = new TrimmedCurve3D(aC3D, aMid, aL);

        //
        Curve2D? aC2D1F = null, aC2D1L = null, aC2D2F = null, aC2D2L = null;
        //
        if (aC2D1 is not null)
        {
            aC2D1F = new TrimmedCurve2D(aC2D1, aF, aMid);
            aC2D1L = new TrimmedCurve2D(aC2D1, aMid, aL);
        }

        if (aC2D2 is not null)
        {
            aC2D2F = new TrimmedCurve2D(aC2D2, aF, aMid);
            aC2D2L = new TrimmedCurve2D(aC2D2, aMid, aL);
        }
        //
        IntTools_Curve aIC1 = new(aC3DNewF, aC2D1F, aC2D2F, IC.Tolerance(), IC.TangentialTolerance());
        IntTools_Curve aIC2 = new(aC3DNewL, aC2D1L, aC2D2L, IC.Tolerance(), IC.TangentialTolerance());
        //
        aCvs.Append(aIC1);
        //
        aCvs.Append(aIC2);
        //
        return 2;
    }

    /// <summary>
    /// Keeps the first line and the first line not parallel to it (all curves if one is not a line). Trimmed curves
    /// are replaced by their basis curves in <paramref name="aSIn"/> itself, as OCCT does through a const_cast.
    /// </summary>
    public static void RejectLines(NCollection_Sequence<IntTools_Curve> aSIn, NCollection_Sequence<IntTools_Curve> aSOut)
    {
        int i, j, aNb;
        bool bFlag;
        Curve3D? aC3D;

        Direction3D aD1 = new(), aD2;

        aSOut.Clear();

        aNb = aSIn.Length();
        for (i = 1; i <= aNb; i++)
        {
            aC3D = aSIn[i].Curve();
            //
            if (aC3D is TrimmedCurve3D aGTC)
            {
                aC3D = aGTC.BasisCurve();
                aSIn[i].SetCurve(aC3D); // OCCT writes into its const input
            }
            IntTools_Curve IC = aSIn[i];
            //
            if (aC3D is not LineCurve3D aGLine)
            {
                aSOut.Clear();
                for (j = 1; j <= aNb; j++)
                {
                    aSOut.Append(aSIn.Value(j));
                }
                return;
            }
            //
            Line3D aLin = aGLine.Lin();
            aD2 = aLin.Direction();
            if (i == 1)
            {
                aSOut.Append(IC);
                aD1 = aD2;
                continue;
            }

            bFlag = IsDirsCoinside(aD1, aD2);
            if (!bFlag)
            {
                aSOut.Append(IC);
                return;
            }
        }
    }

    /// <summary>True if the directions are equal or opposite within 0.0002 (as points on the unit sphere).</summary>
    public static bool IsDirsCoinside(in Direction3D D1, in Direction3D D2)
    {
        bool bFlag;
        Point3D P1 = new(D1.X(), D1.Y(), D1.Z());
        Point3D P2 = new(D2.X(), D2.Y(), D2.Z());
        double dLim = 0.0002, d;
        d = P1.Distance(P2);
        bFlag = (d < dLim || Math.Abs(2.0 - d) < dLim);
        return bFlag;
    }

    /// <summary>True if the directions are equal or opposite within <paramref name="dLim"/> (as points on the unit sphere).</summary>
    public static bool IsDirsCoinside(in Direction3D D1, in Direction3D D2, double dLim)
    {
        bool bFlag;
        double d;
        //
        Point3D P1 = new(D1.X(), D1.Y(), D1.Z());
        Point3D P2 = new(D2.X(), D2.Y(), D2.Z());
        d = P1.Distance(P2);
        bFlag = (d < dLim || Math.Abs(2.0 - d) < dLim);
        return bFlag;
    }

    /// <summary>True if the curve is bounded and its ends are closer than Precision::Confusion.</summary>
    public static bool IsClosed(Curve3D aC3D)
    {
        bool bRet;
        double aF, aL, aDist, aPC;
        Point3D aP1, aP2;

        if (aC3D is not BoundedCurve3D)
        {
            return false;
        }

        aF = aC3D.FirstParameter();
        aL = aC3D.LastParameter();

        aC3D.D0(aF, out aP1);
        aC3D.D0(aL, out aP2);

        //
        aPC = global::Kerf.Engine.Foundation.Precision.Confusion();
        aPC = aPC * aPC;
        aDist = aP1.SquareDistance(aP2);
        bRet = aDist < aPC;
        return bRet;
    }

    /// <summary>
    /// The tolerance of the curve: <paramref name="aTolBase"/>, enlarged for a trimmed parabola by its focal
    /// distance and the trimming parameters.
    /// </summary>
    public static double CurveTolerance(Curve3D? aC3D, double aTolBase)
    {
        double aTolReached, aTf, aTl, aTolMin, aTolMax;

        aTolReached = aTolBase;
        //
        if (aC3D is null)
        {
            return aTolReached;
        }
        //
        if (aC3D is not TrimmedCurve3D aCT3D)
        {
            return aTolReached;
        }
        //
        aTolMin = aTolBase;
        aTolMax = aTolBase;
        //
        aTf = aCT3D.FirstParameter();
        aTl = aCT3D.LastParameter();
        //
        CurveAdaptor aGAC = new(aCT3D);
        CurveKind aCType = aGAC.GetType();
        //
        if (aCType == Parabola)
        {
            Curve3D aC3DBase = aCT3D.BasisCurve();
            ParabolaTolerance(aC3DBase, aTf, aTl, aTolBase, ref aTolMin, ref aTolMax);
            aTolReached = aTolMax;
        }
        //
        return aTolReached;
    }

    private static void ParabolaTolerance(Curve3D aC3D, double aTf, double aTl, double aTol, ref double aTolMin,
                                          ref double aTolMax)
    {

        aTolMin = aTol;
        aTolMax = aTol;

        if (aC3D is not ParabolaCurve3D aGP)
        {
            return;
        }

        int aNbPoints;
        double aFocal, aX1, aX2, aTol1, aTol2;
        Point3D aPf, aPl;
        Parabola3D aParab = aGP.Parab();
        Axis3D aXAxis = aParab.XAxis();
        LineCurve3D aGAxis = new(aXAxis);

        aFocal = aGP.Focal();
        if (aFocal == 0.0)
        {
            return;
        }
        //
        // aTol1
        aTol1 = aTol;
        aX1 = 0.0;
        aGP.D0(aTf, out aPf);
        GeomAPI_ProjectPointOnCurve aProj1 = new(aPf, aGAxis);
        aNbPoints = aProj1.NbPoints();
        if (aNbPoints != 0)
        {
            aX1 = aProj1.LowerDistanceParameter();
        }
        if (aX1 >= 0.0)
        {
            aTol1 = aTol * Math.Sqrt(0.5 * aX1 / aFocal);
        }
        if (aTol1 == 0.0)
        {
            aTol1 = aTol;
        }
        //
        // aTol2
        aTol2 = aTol;
        aX2 = 0.0;
        aGP.D0(aTl, out aPl);
        GeomAPI_ProjectPointOnCurve aProj2 = new(aPl, aGAxis);
        aNbPoints = aProj2.NbPoints();
        if (aNbPoints != 0)
        {
            aX2 = aProj2.LowerDistanceParameter();
        }

        if (aX2 >= 0.0)
        {
            aTol2 = aTol * Math.Sqrt(0.5 * aX2 / aFocal);
        }
        if (aTol2 == 0.0)
        {
            aTol2 = aTol;
        }
        //
        aTolMax = (aTol1 > aTol2) ? aTol1 : aTol2;
        aTolMin = (aTol1 < aTol2) ? aTol1 : aTol2;
    }

    /// <summary>
    /// Checks the curve: false if it has no 3D curve or if its box (enlarged by its larger tolerance, added to
    /// <paramref name="theBox"/>) is thinner than 3 Precision::Confusion.
    /// </summary>
    public static bool CheckCurve(in IntTools_Curve theCurve, ref Box theBox)
    {
        Curve3D? aC3D = theCurve.Curve();
        bool bValid = aC3D is not null;
        if (!bValid)
        {
            return bValid;
        }
        //
        // Build bounding box for the curve
        Add3dCurve.Add(new CurveAdaptor(aC3D!),
                              Max(theCurve.Tolerance(), theCurve.TangentialTolerance()),
                              ref theBox);
        //
        // Estimate the bounding box of the curve comparing it with the
        // minimal length for the curve from which the valid edge can be built -
        // 3*Precision::Confusion():
        // - 2 vertices with the Precision::Confusion() tolerance;
        // - plus Precision::Confusion() as the minimal distance between vertices.
        double aTolCmp = 3 * global::Kerf.Engine.Foundation.Precision.Confusion();
        //
        // Check the size of the box using the Bnd_Box::IsThin() method
        // which does not use the gap of the box.
        bValid = !theBox.IsThin(aTolCmp);
        //
        return bValid;
    }

    /// <summary>True if the parameter is closer than the tolerance to an end of the range.</summary>
    public static bool IsOnPave(double aT1, in IntTools_Range aRange, double aTolerance)
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
    public static void VertexParameters(IntTools_CommonPrt aCPart, out double aT1, out double aT2)
    {
        IntTools_Range aR1 = aCPart.Range1();
        aT1 = 0.5 * (aR1.First() + aR1.Last());
        //
        if ((aCPart.VertexParameter1() >= aR1.First()) && (aCPart.VertexParameter1() <= aR1.Last()))
        {
            aT1 = aCPart.VertexParameter1();
        }
        //
        NCollection_Sequence<IntTools_Range> aRanges2 = aCPart.Ranges2();
        IntTools_Range aR2 = aRanges2.Value(1);
        aT2 = 0.5 * (aR2.First() + aR2.Last());
        //
        if ((aCPart.VertexParameter2() >= aR2.First()) && (aCPart.VertexParameter2() <= aR2.Last()))
        {
            aT2 = aCPart.VertexParameter2();
        }
    }

    /// <summary>The parameter of a vertex common part on the first edge (as <see cref="VertexParameters"/>).</summary>
    public static void VertexParameter(IntTools_CommonPrt aCPart, out double aT)
    {
        IntTools_Range aR = aCPart.Range1();
        aT = 0.5 * (aR.First() + aR.Last());
        if ((aCPart.VertexParameter1() >= aR.First()) && (aCPart.VertexParameter1() <= aR.Last()))
        {
            aT = aCPart.VertexParameter1();
        }
    }

    /// <summary>True if the parameter is in the range, or within the tolerance of one of its ends.</summary>
    public static bool IsOnPave1(double aTR, in IntTools_Range aCPRange, double aTolerance)
    {
        bool bIsOnPave;
        double aT1, aT2, dT1, dT2;
        //
        aT1 = aCPRange.First();
        aT2 = aCPRange.Last();
        bIsOnPave = (aTR >= aT1 && aTR <= aT2);
        if (bIsOnPave)
        {
            return bIsOnPave;
        }
        //
        dT1 = Math.Abs(aTR - aT1);
        dT2 = Math.Abs(aTR - aT2);
        bIsOnPave = (dT1 <= aTolerance || dT2 <= aTolerance);
        return bIsOnPave;
    }

    /// <summary>True if an end of <paramref name="aR"/> is in <paramref name="aRRef"/> enlarged by the tolerance.</summary>
    public static bool IsInRange(in IntTools_Range aRRef, in IntTools_Range aR, double aTolerance)
    {
        bool bIsIn;
        //
        aR.Range(out double aT1, out double aT2);
        aRRef.Range(out double aTRef1, out double aTRef2);
        //
        aTRef1 -= aTolerance;
        aTRef2 += aTolerance;
        //
        bIsIn = (aT1 >= aTRef1 && aT1 <= aTRef2) || (aT2 >= aTRef1 && aT2 <= aTRef2);
        //
        return bIsIn;
    }

    /// <summary>
    /// Intersects the segment [<paramref name="theTLin1"/>, <paramref name="theTLin2"/>] of the line with the plane.
    /// Returns 0 for an intersection point (the outputs are set), 1 if the segment lies in the plane (both ends within
    /// the sum of the tolerances, signed), 2 if it is on one side, 3 if the intersection is out of the range; the
    /// outputs are written only for 0.
    /// </summary>
    public static int SegPln(in Line3D theLin, double theTLin1, double theTLin2, double theTolLin, in Plane3D thePln, double theTolPln,
                             ref Point3D theP, ref double theTP, ref double theTolP, ref double theTPmin, ref double theTPmax)
    {
        int iRet;
        double aTol, aTP, aDist1, aDist2, aE, aH;
        Point3D aP1 = new(), aP2 = new();
        //
        iRet = 0;
        aTol = theTolLin + theTolPln;
        //
        CoordinateSystem aPosPln = thePln.Position();
        Direction3D aDirPln = aPosPln.Direction();
        Point3D aLocPln = aPosPln.Location();
        //
        Direction3D aDirLin = theLin.Direction();
        Point3D aLocLin = theLin.Location();
        //
        aP1.SetXYZ(aLocLin.XYZ() + theTLin1 * aDirLin.XYZ());
        aDist1 = aDirPln.X() * (aP1.X() - aLocPln.X()) + aDirPln.Y() * (aP1.Y() - aLocPln.Y())
                 + aDirPln.Z() * (aP1.Z() - aLocPln.Z());
        //
        aP2.SetXYZ(aLocLin.XYZ() + theTLin2 * aDirLin.XYZ());
        aDist2 = aDirPln.X() * (aP2.X() - aLocPln.X()) + aDirPln.Y() * (aP2.Y() - aLocPln.Y())
                 + aDirPln.Z() * (aP2.Z() - aLocPln.Z());
        //
        if (aDist1 < aTol && aDist2 < aTol)
        {
            iRet = 1; // common block
            return iRet;
        }
        //
        if (aDist1 * aDist2 > 0.0)
        {
            iRet = 2; // segment lays on one side to the Plane
            return iRet;
        }
        //
        thePln.Coefficients(out double aA, out double aB, out double aC, out double aD);
        aE = aA * aLocLin.X() + aB * aLocLin.Y() + aC * aLocLin.Z() + aD;
        aH = aA * aDirLin.X() + aB * aDirLin.Y() + aC * aDirLin.Z();
        aTP = -aE / aH;
        if (aTP < theTLin1 - aTol || aTP > theTLin2 + aTol)
        {
            iRet = 3; // no intersections due to range of the Line
            return iRet;
        }
        //
        theTP = aTP;
        theP.SetXYZ(aLocLin.XYZ() + aTP * aDirLin.XYZ());
        theTolP = aTol;
        theTPmin = theTP - theTolPln;
        theTPmax = theTP + theTolPln;
        iRet = 0; // intersection point
        return iRet;
    }

    /// <summary>
    /// Computes the maximal distance between the 3D curve and the 2D curve on the surface over [first, last]
    /// (<see cref="CheckCurveOnSurface"/>), enlarged by a relative margin of 1e-5, and the parameter where it
    /// is reached. False if the check fails (the outputs are then 0, where OCCT leaves them unchanged; no caller reads them).
    /// </summary>
    public static bool ComputeTolerance(Curve3D theCurve3D, Curve2D theCurve2D, Surface theSurf, double theFirst, double theLast,
                                        out double theMaxDist, out double theMaxPar) =>
        ComputeTolerance(theCurve3D, theCurve2D, theSurf, theFirst, theLast, out theMaxDist, out theMaxPar, global::Kerf.Engine.Foundation.Precision.PConfusion());

    /// <inheritdoc cref="ComputeTolerance(Curve3D, Curve2D, Surface, double, double, out double, out double)"/>
    public static bool ComputeTolerance(Curve3D theCurve3D, Curve2D theCurve2D, Surface theSurf, double theFirst, double theLast,
                                        out double theMaxDist, out double theMaxPar, double theTolRange, bool theToRunParallel = false)
    {
        CheckCurveOnSurface aCS = new();
        //
        CurveAdaptorBase aGeomAdaptorCurve = new CurveAdaptor(theCurve3D, theFirst, theLast);

        Curve2DAdaptorBase aGeom2dAdaptorCurve = new CurveAdaptor2D(theCurve2D, theFirst, theLast);
        SurfaceAdaptor aGeomAdaptorSurface = new(theSurf);

        CurveOnSurface anAdaptor3dCurveOnSurface = new(aGeom2dAdaptorCurve, aGeomAdaptorSurface);

        aCS.Init(aGeomAdaptorCurve, theTolRange);
        aCS.SetParallel(theToRunParallel);
        aCS.Perform(anAdaptor3dCurveOnSurface);
        if (!aCS.IsDone)
        {
            // OCCT leaves the outputs unchanged; every caller reads them only on success.
            theMaxDist = 0.0;
            theMaxPar = 0.0;
            return false;
        }

        // Obtaining precise result is impossible if we use
        // numeric methods for solution. Therefore, we must provide
        // some margin. Otherwise, in the future
        //(when geometrical properties of the curve will be changed,
        // e.g. after trimming) we will be able to come
        // to the more precise minimum point. As result, this curve with the
        // tolerance computed earlier will become invalid.
        const double anEps = (1.0 + 1.0e-5);
        theMaxDist = anEps * aCS.MaxDistance();
        theMaxPar = aCS.MaxParameter();
        //
        return true;
    }

    /// <summary>
    /// The parameter range on an edge of tolerance <paramref name="theTol1"/> covered by a crossing with an edge of
    /// tolerance <paramref name="theTol2"/> at the angle.
    /// </summary>
    public static double ComputeIntRange(double theTol1, double theTol2, double theAngle)
    {
        double aDt;
        //
        if (Math.Abs(M_PI_2 - theAngle) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            aDt = theTol2;
        }
        else
        {
            double a1, a2, anAngle;
            //
            anAngle = (theAngle > M_PI_2) ? (M_PI - theAngle) : theAngle;
            a1 = theTol1 * Math.Tan(M_PI_2 - anAngle);
            a2 = theTol2 / Math.Sin(anAngle);
            aDt = a1 + a2;
        }
        //
        return aDt;
    }
}
