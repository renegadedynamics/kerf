// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLineTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2014 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection.Patches;

// ExtendTwoWLines and its helpers.
internal static partial class WLineTool
{
    // Bit-mask is used for information about the operation made in ExtendTwoWLines(...).
    private const uint IntPatchWT_EnAll = 0x00;
    private const uint IntPatchWT_DisLastLast = 0x01;
    private const uint IntPatchWT_DisLastFirst = 0x02;
    private const uint IntPatchWT_DisFirstLast = 0x04;
    private const uint IntPatchWT_DisFirstFirst = 0x08;

    private enum IntPatchWT_WLsConnectionType
    {
        IntPatchWT_NotConnected,
        IntPatchWT_Singular,
        IntPatchWT_Common,
        IntPatchWT_ReqExtend
    }

    /// <summary>
    /// OCCT's <c>const IntSurf_PntOn2S&amp; aPnt = aWLine->Point(i)</c>: a reference to a node of the line's
    /// NCollection_Sequence (a linked list), which stays the same node when points are inserted before it and sees a
    /// value replaced in place (<c>Curve()->Value(i, P)</c>). The port's sequence is List-backed (decision 609), so the
    /// reference is the line and the node's current index, shifted by <see cref="InsertBefore"/>.
    /// </summary>
    private sealed class PntRef
    {
        internal readonly LineOn2S _line;
        internal int _index;

        internal PntRef(LineOn2S line, int index)
        {
            _line = line;
            _index = index;
        }

        internal PntOn2S Value() => _line.Value(_index);
    }

    // theLine->InsertBefore(theIndex, P), keeping OCCT's node references (PntRef) on their nodes.
    private static void InsertBefore(LineOn2S line, int index, in PntOn2S p,
                                     List<PntRef> refs)
    {
        line.InsertBefore(index, p);
        foreach (PntRef aRef in refs)
        {
            if (ReferenceEquals(aRef._line, line) && aRef._index >= index)
            {
                aRef._index++;
            }
        }
    }

    // Returns True if thePmid is intersection point between theS1 and theS2 with given tolerance. In this case,
    // parameters of thePmid on every quadric will be recomputed and returned.
    private static bool IsIntersectionPoint(in Point3D pmid, SurfaceAdaptorBase s1, SurfaceAdaptorBase s2,
                                            in PntOn2S refPt, double tol,
                                            ReadOnlySpan<double> arrPeriods, ref PntOn2S newPt)
    {
        double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;

        switch (s1.GetType())
        {
            case Plane:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(s1.Plane(), pmid, out u1, out v1);
                break;

            case SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(s1.Cylinder(), pmid, out u1, out v1);
                break;

            case SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(s1.Sphere(), pmid, out u1, out v1);
                break;

            case SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(s1.Cone(), pmid, out u1, out v1);
                break;

            case SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(s1.Torus(), pmid, out u1, out v1);
                break;

            default:
                return false;
        }

        switch (s2.GetType())
        {
            case Plane:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(s2.Plane(), pmid, out u2, out v2);
                break;

            case SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(s2.Cylinder(), pmid, out u2, out v2);
                break;

            case SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(s2.Sphere(), pmid, out u2, out v2);
                break;

            case SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(s2.Cone(), pmid, out u2, out v2);
                break;

            case SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(s2.Torus(), pmid, out u2, out v2);
                break;

            default:
                return false;
        }

        newPt.SetValue(pmid, u1, v1, u2, v2);

        SpecialPoints.AdjustPointAndVertex(refPt, arrPeriods, ref newPt);

        Point3D p1 = s1.Value(u1, v1);
        Point3D p2 = s2.Value(u2, v2);

        return (p1.SquareDistanceTo(p2) <= tol * tol);
    }

    // Adds thePOn2S to the begin of theWline.
    private static void ExtendFirst(WLine wline, in PntOn2S addedPt, List<PntRef> refs)
    {
        addedPt.Parameters(out double u1, out double v1, out double u2, out double v2);

        if (addedPt.IsSame(wline.Point(1), global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            wline.Curve().Value(1, addedPt);
            for (int i = 1; i <= wline.VertexCount; i++)
            {
                ref PatchPoint vert = ref wline.ChangeVertex(i);
                if (vert.ParameterOnLine() != 1)
                {
                    break;
                }

                vert.SetParameters(u1, v1, u2, v2);
                vert.SetValue(addedPt.Value());
            }

            return;
        }

        InsertBefore(wline.Curve(), 1, addedPt, refs);

        for (int i = 1; i <= wline.VertexCount; i++)
        {
            ref PatchPoint vert = ref wline.ChangeVertex(i);

            if (vert.ParameterOnLine() == 1)
            {
                vert.SetParameters(u1, v1, u2, v2);
                vert.SetValue(addedPt.Value());
            }
            else
            {
                vert.SetParameter(vert.ParameterOnLine() + 1);
            }
        }
    }

    // Adds thePOn2S to the end of theWline.
    private static void ExtendLast(WLine wline, in PntOn2S addedPt)
    {
        addedPt.Parameters(out double u1, out double v1, out double u2, out double v2);

        int nbPnts = wline.NbPnts();
        if (addedPt.IsSame(wline.Point(nbPnts), global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            wline.Curve().Value(nbPnts, addedPt);
        }
        else
        {
            wline.Curve().Add(addedPt);
        }

        for (int i = wline.VertexCount; i >= 1; i--)
        {
            ref PatchPoint vert = ref wline.ChangeVertex(i);
            if (vert.ParameterOnLine() != nbPnts)
            {
                break;
            }

            vert.SetParameters(u1, v1, u2, v2);
            vert.SetValue(addedPt.Value());
            vert.SetParameter(wline.NbPnts());
        }
    }

    // Checks, if 2D-representation of thePOn2S is in surfaces domain, defined by bounding-boxes theBoxS1 and theBoxS2.
    private static bool IsOutOfDomain(in Box2d boxS1, in Box2d boxS2, in PntOn2S pOn2S,
                                      ReadOnlySpan<double> arrPeriods)
    {
        pOn2S.Parameters(out double u1, out double v1, out double u2, out double v2);

        boxS1.Get(out double u1min, out double v1min, out double u1max, out double v1max);
        boxS2.Get(out double u2min, out double v2min, out double u2max, out double v2max);

        u1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u1, u1min, u1min + arrPeriods[0]);
        v1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v1, v1min, v1min + arrPeriods[1]);
        u2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u2, u2min, u2min + arrPeriods[2]);
        v2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v2, v2min, v2min + arrPeriods[3]);

        return (boxS1.IsOut(new Point2D(u1, v1)) || boxS2.IsOut(new Point2D(u2, v2)));
    }

    // Check if extending is possible (see ExtendTwoWLines).
    private static IntPatchWT_WLsConnectionType CheckArgumentsToExtend(
        SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, in PntOn2S ptWL1, in PntOn2S ptWL2,
        ref PntOn2S newPoint, in Vector3D vec1, in Vector3D vec2, in Vector3D vec3,
        in Box2d boxS1, in Box2d boxS2, double toler3D, ReadOnlySpan<double> arrPeriods)
    {
        double sqToler = toler3D * toler3D;
        IntPatchWT_WLsConnectionType retVal = IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected;
        if (vec3.SquareMagnitude <= sqToler)
        {
            if ((vec1.AngleTo(vec2) > myMaxConcatAngle))
            {
                return retVal;
            }
            else
            {
                retVal = IntPatchWT_WLsConnectionType.IntPatchWT_Common;
            }
        }
        else if ((vec1.AngleTo(vec2) > myMaxConcatAngle) || (vec1.AngleTo(vec3) > myMaxConcatAngle)
                 || (vec2.AngleTo(vec3) > myMaxConcatAngle))
        {
            return retVal;
        }

        Point3D pmid = new Point3D(0.5 * (ptWL1.Value().XYZ() + ptWL2.Value().XYZ()));

        Span<double> newPar = stackalloc double[4];

        // Left-bottom corner (OCCT writes the boxes' upper bounds into aNewPar[0], overwritten below)
        Span<double> parLBC = stackalloc double[4];
        boxS1.Get(out parLBC[0], out parLBC[1], out _, out _);
        boxS2.Get(out parLBC[2], out parLBC[3], out _, out _);

        if (!IsIntersectionPoint(pmid, s1, s2, ptWL1, toler3D, arrPeriods, ref newPoint))
        {
            return IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected;
        }

        if (IsOutOfDomain(boxS1, boxS2, newPoint, arrPeriods))
        {
            return IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected;
        }

        Span<double> parWL1 = stackalloc double[4], parWL2 = stackalloc double[4];

        ptWL1.Parameters(out parWL1[0], out parWL1[1], out parWL1[2], out parWL1[3]);
        ptWL2.Parameters(out parWL2[0], out parWL2[1], out parWL2[2], out parWL2[3]);
        newPoint.Parameters(out newPar[0], out newPar[1], out newPar[2], out newPar[3]);

        BoundRange r1 = new BoundRange(), r2 = new BoundRange();

        bool isOnBoundary = false;
        for (int i = 0; i < 4; i++)
        {
            if (arrPeriods[i] == 0.0)
            {
                // Strictly equal
                continue;
            }

            r1.SetVoid();
            r1.Add(parWL1[i]);
            r1.Add(parWL2[i]);

            if (r1.IsIntersected(parLBC[i], arrPeriods[i]) != BoundRange.IntersectStatus.IntersectStatus_Out)
            {
                // Check, if we intersect surface boundary when we will extend Wline1 or Wline2
                // to theNewPoint
                MinMax(ref parWL1[i], ref parWL2[i]);
                if (newPar[i] > parWL2[i])
                {
                    // Source situation:
                    //
                    //---*---------------*------------*-----
                    //  aParWL1[i]   aParWL2[i]    aNewPar[i]
                    //
                    // After possible adjusting:
                    //
                    //---*---------------*------------*-----
                    //  aParWL1[i]   aNewPar[i]    aParWL2[i]
                    //
                    // Now we will be able to extend every WLine to
                    // aNewPar[i] to make them close to each other.
                    // However, it is necessary to add check if we
                    // intersect boundary.
                    double par = parWL1[i] + arrPeriods[i] * Math.Ceiling((newPar[i] - parWL1[i]) / arrPeriods[i]);
                    parWL1[i] = parWL2[i];
                    parWL2[i] = par;
                }
                else if (newPar[i] < parWL1[i])
                {
                    // See comments to main "if".
                    // Source situation:
                    //
                    //---*---------------*------------*-----
                    //  aNewPar[i]    aParWL1[i]   aParWL2[i]
                    //
                    // After possible adjusting:
                    //
                    //---*---------------*------------*-----
                    //  aParWL1[i]   aNewPar[i]    aParWL2[i]

                    double par = parWL2[i] - arrPeriods[i] * Math.Ceiling((parWL2[i] - newPar[i]) / arrPeriods[i]);
                    parWL2[i] = parWL1[i];
                    parWL1[i] = par;
                }

                r1.SetVoid();
                r2.SetVoid();
                r1.Add(parWL1[i]);
                r1.Add(newPar[i]);
                r2.Add(newPar[i]);
                r2.Add(parWL2[i]);

                if (r1.IsIntersected(parLBC[i], arrPeriods[i]) != BoundRange.IntersectStatus.IntersectStatus_Out
                    || r2.IsIntersected(parLBC[i], arrPeriods[i]) != BoundRange.IntersectStatus.IntersectStatus_Out)
                {
                    return IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected;
                }
                else
                {
                    isOnBoundary = true;
                }
            }
        }

        if (isOnBoundary)
        {
            return IntPatchWT_WLsConnectionType.IntPatchWT_Singular;
        }

        if (retVal == IntPatchWT_WLsConnectionType.IntPatchWT_Common)
        {
            return IntPatchWT_WLsConnectionType.IntPatchWT_Common;
        }

        return IntPatchWT_WLsConnectionType.IntPatchWT_ReqExtend;
    }

    // Performs extending theWLine1 and theWLine2 through their respecting start point.
    private static void ExtendTwoWLFirstFirst(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2,
                                              WLine wLine1, WLine wLine2,
                                              in PntOn2S ptWL1, in PntOn2S ptWL2,
                                              in Vector3D vec1, in Vector3D vec2, in Vector3D vec3,
                                              in Box2d boxS1, in Box2d boxS2, double toler3D,
                                              ReadOnlySpan<double> arrPeriods, ref uint checkResult,
                                              ref bool hasBeenJoined, List<PntRef> refs)
    {
        PntOn2S pOn2S = new PntOn2S();
        IntPatchWT_WLsConnectionType checkRes = CheckArgumentsToExtend(s1, s2, ptWL1, ptWL2, ref pOn2S,
                                                                        vec1, vec2, vec3, boxS1, boxS2,
                                                                        toler3D, arrPeriods);

        if (checkRes != IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected)
        {
            checkResult |= (IntPatchWT_DisFirstLast | IntPatchWT_DisLastFirst);
        }
        else
        {
            return;
        }

        SpecialPoints.AdjustPointAndVertex(ptWL1, arrPeriods, ref pOn2S);
        ExtendFirst(wLine1, pOn2S, refs);
        SpecialPoints.AdjustPointAndVertex(ptWL2, arrPeriods, ref pOn2S);
        ExtendFirst(wLine2, pOn2S, refs);

        if (hasBeenJoined || (checkRes == IntPatchWT_WLsConnectionType.IntPatchWT_Singular))
        {
            return;
        }

        double prm = wLine1.Vertex(1).ParameterOnLine();
        while (wLine1.Vertex(1).ParameterOnLine() == prm)
        {
            wLine1.RemoveVertex(1);
        }

        prm = wLine2.Vertex(1).ParameterOnLine();
        while (wLine2.Vertex(1).ParameterOnLine() == prm)
        {
            wLine2.RemoveVertex(1);
        }

        int nbPts = wLine2.NbPnts();
        for (int nPt = 2; nPt <= nbPts; nPt++)
        {
            PntOn2S pt = wLine2.Point(nPt);
            InsertBefore(wLine1.Curve(), 1, pt, refs);
        }

        for (int nVtx = 1; nVtx <= wLine1.VertexCount; nVtx++)
        {
            ref PatchPoint vert = ref wLine1.ChangeVertex(nVtx);
            double curParam = vert.ParameterOnLine();
            vert.SetParameter(nbPts + curParam - 1);
        }

        for (int nVtx = 1; nVtx <= wLine2.VertexCount; nVtx++)
        {
            ref PatchPoint vert = ref wLine2.ChangeVertex(nVtx);
            double curParam = vert.ParameterOnLine();
            vert.SetParameter(nbPts - curParam + 1);
            wLine1.AddVertex(vert, true);
        }

        hasBeenJoined = true;
    }

    // Performs extending theWLine1 through its start point and theWLine2 through its end point.
    private static void ExtendTwoWLFirstLast(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2,
                                             WLine wLine1, WLine wLine2,
                                             in PntOn2S ptWL1, in PntOn2S ptWL2,
                                             in Vector3D vec1, in Vector3D vec2, in Vector3D vec3,
                                             in Box2d boxS1, in Box2d boxS2, double toler3D,
                                             ReadOnlySpan<double> arrPeriods, ref uint checkResult,
                                             ref bool hasBeenJoined, List<PntRef> refs)
    {
        PntOn2S pOn2S = new PntOn2S();
        IntPatchWT_WLsConnectionType checkRes = CheckArgumentsToExtend(s1, s2, ptWL1, ptWL2, ref pOn2S,
                                                                        vec1, vec2, vec3, boxS1, boxS2,
                                                                        toler3D, arrPeriods);

        if (checkRes != IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected)
        {
            checkResult |= IntPatchWT_DisLastLast;
        }
        else
        {
            return;
        }

        SpecialPoints.AdjustPointAndVertex(ptWL1, arrPeriods, ref pOn2S);
        ExtendFirst(wLine1, pOn2S, refs);
        SpecialPoints.AdjustPointAndVertex(ptWL2, arrPeriods, ref pOn2S);
        ExtendLast(wLine2, pOn2S);

        if (hasBeenJoined || (checkRes == IntPatchWT_WLsConnectionType.IntPatchWT_Singular))
        {
            return;
        }

        double prm = wLine1.Vertex(1).ParameterOnLine();
        while (wLine1.Vertex(1).ParameterOnLine() == prm)
        {
            wLine1.RemoveVertex(1);
        }

        prm = wLine2.Vertex(wLine2.VertexCount).ParameterOnLine();
        while (wLine2.Vertex(wLine2.VertexCount).ParameterOnLine() == prm)
        {
            wLine2.RemoveVertex(wLine2.VertexCount);
        }

        int nbPts = wLine2.NbPnts();
        for (int nPt = nbPts - 1; nPt >= 1; nPt--)
        {
            PntOn2S pt = wLine2.Point(nPt);
            InsertBefore(wLine1.Curve(), 1, pt, refs);
        }

        for (int nVtx = 1; nVtx <= wLine1.VertexCount; nVtx++)
        {
            ref PatchPoint vert = ref wLine1.ChangeVertex(nVtx);
            double curParam = vert.ParameterOnLine();
            vert.SetParameter(nbPts + curParam - 1);
        }

        for (int nVtx = wLine2.VertexCount; nVtx >= 1; nVtx--)
        {
            PatchPoint vert = wLine2.Vertex(nVtx);
            wLine1.AddVertex(vert, true);
        }

        hasBeenJoined = true;
    }

    // Performs extending theWLine1 through its end point and theWLine2 through its start point.
    private static void ExtendTwoWLLastFirst(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2,
                                             WLine wLine1, WLine wLine2,
                                             in PntOn2S ptWL1, in PntOn2S ptWL2,
                                             in Vector3D vec1, in Vector3D vec2, in Vector3D vec3,
                                             in Box2d boxS1, in Box2d boxS2, double toler3D,
                                             ReadOnlySpan<double> arrPeriods, ref uint checkResult,
                                             ref bool hasBeenJoined, List<PntRef> refs)
    {
        PntOn2S pOn2S = new PntOn2S();
        IntPatchWT_WLsConnectionType checkRes = CheckArgumentsToExtend(s1, s2, ptWL1, ptWL2, ref pOn2S,
                                                                        vec1, vec2, vec3, boxS1, boxS2,
                                                                        toler3D, arrPeriods);

        if (checkRes != IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected)
        {
            checkResult |= IntPatchWT_DisLastLast;
        }
        else
        {
            return;
        }

        SpecialPoints.AdjustPointAndVertex(ptWL1, arrPeriods, ref pOn2S);
        ExtendLast(wLine1, pOn2S);
        SpecialPoints.AdjustPointAndVertex(ptWL2, arrPeriods, ref pOn2S);
        ExtendFirst(wLine2, pOn2S, refs);

        if (hasBeenJoined || (checkRes == IntPatchWT_WLsConnectionType.IntPatchWT_Singular))
        {
            return;
        }

        double prm = wLine1.Vertex(wLine1.VertexCount).ParameterOnLine();
        while (wLine1.Vertex(wLine1.VertexCount).ParameterOnLine() == prm)
        {
            wLine1.RemoveVertex(wLine1.VertexCount);
        }

        prm = wLine2.Vertex(1).ParameterOnLine();
        while (wLine2.Vertex(1).ParameterOnLine() == prm)
        {
            wLine2.RemoveVertex(1);
        }

        int nbPts = wLine1.NbPnts();
        for (int nPt = 2; nPt <= wLine2.NbPnts(); nPt++)
        {
            PntOn2S pt = wLine2.Point(nPt);
            wLine1.Curve().Add(pt);
        }

        for (int nVtx = 1; nVtx <= wLine2.VertexCount; nVtx++)
        {
            ref PatchPoint vert = ref wLine2.ChangeVertex(nVtx);
            double curParam = vert.ParameterOnLine();
            vert.SetParameter(nbPts + curParam - 1);
            wLine1.AddVertex(vert, false);
        }

        hasBeenJoined = true;
    }

    // Performs extending theWLine1 and theWLine2 through their end points.
    private static void ExtendTwoWLLastLast(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2,
                                            WLine wLine1, WLine wLine2,
                                            in PntOn2S ptWL1, in PntOn2S ptWL2,
                                            in Vector3D vec1, in Vector3D vec2, in Vector3D vec3,
                                            in Box2d boxS1, in Box2d boxS2, double toler3D,
                                            ReadOnlySpan<double> arrPeriods, ref uint checkResult,
                                            ref bool hasBeenJoined)
    {
        PntOn2S pOn2S = new PntOn2S();
        IntPatchWT_WLsConnectionType checkRes = CheckArgumentsToExtend(s1, s2, ptWL1, ptWL2, ref pOn2S,
                                                                        vec1, vec2, vec3, boxS1, boxS2,
                                                                        toler3D, arrPeriods);

        if (checkRes != IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected)
        {
            checkResult |= IntPatchWT_DisLastLast;
        }
        else
        {
            return;
        }

        SpecialPoints.AdjustPointAndVertex(ptWL1, arrPeriods, ref pOn2S);
        ExtendLast(wLine1, pOn2S);
        SpecialPoints.AdjustPointAndVertex(ptWL2, arrPeriods, ref pOn2S);
        ExtendLast(wLine2, pOn2S);

        if (hasBeenJoined || (checkRes == IntPatchWT_WLsConnectionType.IntPatchWT_Singular))
        {
            return;
        }

        double prm = wLine1.Vertex(wLine1.VertexCount).ParameterOnLine();
        while (wLine1.Vertex(wLine1.VertexCount).ParameterOnLine() == prm)
        {
            wLine1.RemoveVertex(wLine1.VertexCount);
        }

        prm = wLine2.Vertex(wLine2.VertexCount).ParameterOnLine();
        while (wLine2.Vertex(wLine2.VertexCount).ParameterOnLine() == prm)
        {
            wLine2.RemoveVertex(wLine2.VertexCount);
        }

        int nbPts = wLine1.NbPnts() + wLine2.NbPnts();
        for (int nPt = wLine2.NbPnts() - 1; nPt >= 1; nPt--)
        {
            PntOn2S pt = wLine2.Point(nPt);
            wLine1.Curve().Add(pt);
        }

        for (int nVtx = wLine2.VertexCount; nVtx >= 1; nVtx--)
        {
            ref PatchPoint vert = ref wLine2.ChangeVertex(nVtx);
            double curParam = vert.ParameterOnLine();
            vert.SetParameter(nbPts - curParam);
            wLine1.AddVertex(vert, false);
        }

        hasBeenJoined = true;
    }

    private static bool IsNeedSkipWL(WLine wl, in Box2d boxS1, in Box2d boxS2,
                                     ReadOnlySpan<double> arrPeriods)
    {
        double firstp, lastp;
        int nbVtx = wl.VertexCount;

        for (int i = 1; i < nbVtx; i++)
        {
            firstp = wl.Vertex(i).ParameterOnLine();
            lastp = wl.Vertex(i + 1).ParameterOnLine();

            int pmid = (int)((firstp + lastp) / 2);
            PntOn2S aPmid = wl.Point(pmid);

            if (IsOutOfDomain(boxS1, boxS2, aPmid, arrPeriods))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Extends the walking lines of <paramref name="slin"/> whose ends are close (within the angle
    /// <see cref="myMaxConcatAngle"/>) to their common intersection point, joining them where the point isn't on a
    /// surface boundary. <paramref name="arrPeriods"/> holds the surfaces' periods (U1, V1, U2, V2; 0 if not
    /// periodic); the boxes are the surfaces' parametric domains. Lines ending at one of
    /// <paramref name="listOfCriticalPoints"/> aren't extended there. The surfaces must be quadrics.
    /// </summary>
    public static void ExtendTwoWLines(NCollection_Sequence<PatchLine> slin, SurfaceAdaptorBase s1,
                                       SurfaceAdaptorBase s2, double toler3D, ReadOnlySpan<double> arrPeriods,
                                       in Box2d boxS1, in Box2d boxS2,
                                       NCollection_List<Point3D> listOfCriticalPoints)
    {
        if (slin.Length() < 2)
        {
            return;
        }

        Vector3D vec1 = new Vector3D(), vec2 = new Vector3D(), vec3 = new Vector3D();

        uint hasBeenJoinedCounter = 0;

        List<PntRef> refs = new List<PntRef>(8);

        for (int numOfLine1 = 1; numOfLine1 <= slin.Length(); numOfLine1++)
        {
            if (hasBeenJoinedCounter > 0)
            {
                numOfLine1--;
            }

            hasBeenJoinedCounter = 0;

            WLine? wLine1 = slin.Value(numOfLine1) as WLine;

            if (wLine1 is null)
            { // We must have failed to join not-point-lines
                continue;
            }

            int nbPntsWL1 = wLine1.NbPnts();

            if (wLine1.Vertex(1).ParameterOnLine() != 1)
            {
                continue;
            }

            if (wLine1.Vertex(wLine1.VertexCount).ParameterOnLine() != wLine1.NbPnts())
            {
                continue;
            }

            // OCCT's references to the line's nodes (PntRef).
            refs.Clear();
            PntRef pntFWL1 = new PntRef(wLine1.Curve(), 1);
            PntRef pntFp1WL1 = new PntRef(wLine1.Curve(), 2);

            PntRef pntLWL1 = new PntRef(wLine1.Curve(), nbPntsWL1);
            PntRef pntLm1WL1 = new PntRef(wLine1.Curve(), nbPntsWL1 - 1);
            refs.Add(pntFWL1);
            refs.Add(pntFp1WL1);
            refs.Add(pntLWL1);
            refs.Add(pntLm1WL1);

            if (IsNeedSkipWL(wLine1, boxS1, boxS2, arrPeriods))
            {
                continue;
            }

            // Enable/Disable of some check. Bit-mask is used for it.
            // E.g. if 1st point of aWLine1 matches with
            // 1st point of aWLine2 then we do not need in check
            // 1st point of aWLine1 and last point of aWLine2 etc.
            uint checkResult = IntPatchWT_EnAll;

            // If aWLine1 is already connected with another Wline then
            // there is no point in extending.
            for (int numOfLine2 = numOfLine1 + 1; numOfLine2 <= slin.Length(); numOfLine2++)
            {
                WLine? wLine2 = slin.Value(numOfLine2) as WLine;

                if (wLine2 is null)
                {
                    continue;
                }

                PntOn2S pntFWL2 = wLine2.Point(1);
                PntOn2S pntLWL2 = wLine2.Point(wLine2.NbPnts());

                if (!(pntFWL1.Value().IsSame(pntFWL2, toler3D, global::Kerf.Engine.Foundation.Precision.PConfusion()))
                    && !(pntFWL1.Value().IsSame(pntLWL2, toler3D, global::Kerf.Engine.Foundation.Precision.PConfusion())))
                {
                    if (pntFWL1.Value().IsSame(pntFWL2, toler3D) || pntFWL1.Value().IsSame(pntLWL2, toler3D))
                    {
                        checkResult |= IntPatchWT_DisFirstFirst | IntPatchWT_DisFirstLast;
                    }
                }

                if (!(pntLWL1.Value().IsSame(pntFWL2, toler3D, global::Kerf.Engine.Foundation.Precision.PConfusion()))
                    && !(pntLWL1.Value().IsSame(pntLWL2, toler3D, global::Kerf.Engine.Foundation.Precision.PConfusion())))
                {
                    if (pntLWL1.Value().IsSame(pntFWL2, toler3D) || pntLWL1.Value().IsSame(pntLWL2, toler3D))
                    {
                        checkResult |= IntPatchWT_DisLastFirst | IntPatchWT_DisLastLast;
                    }
                }

                if (!listOfCriticalPoints.IsEmpty())
                {
                    for (NCollection_List<Point3D>.Iterator itr = new(listOfCriticalPoints); itr.More();
                         itr.Next())
                    {
                        Point3D pt = itr.Value();
                        if ((checkResult & (IntPatchWT_DisFirstFirst | IntPatchWT_DisFirstLast)) == 0)
                        {
                            if (pt.SquareDistanceTo(pntFWL1.Value().Value()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                checkResult |= IntPatchWT_DisFirstFirst | IntPatchWT_DisFirstLast;
                            }
                        }

                        if ((checkResult & (IntPatchWT_DisLastFirst | IntPatchWT_DisLastLast)) == 0)
                        {
                            if (pt.SquareDistanceTo(pntLWL1.Value().Value()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                checkResult |= IntPatchWT_DisLastFirst | IntPatchWT_DisLastLast;
                            }
                        }

                        if ((checkResult & (IntPatchWT_DisFirstFirst | IntPatchWT_DisLastFirst)) == 0)
                        {
                            if (pt.SquareDistanceTo(pntFWL2.Value()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                checkResult |= IntPatchWT_DisFirstFirst | IntPatchWT_DisLastFirst;
                            }
                        }

                        if ((checkResult & (IntPatchWT_DisFirstLast | IntPatchWT_DisLastLast)) == 0)
                        {
                            if (pt.SquareDistanceTo(pntLWL2.Value()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                checkResult |= IntPatchWT_DisFirstLast | IntPatchWT_DisLastLast;
                            }
                        }
                    }
                }
            }

            if (checkResult
                == (IntPatchWT_DisFirstFirst | IntPatchWT_DisFirstLast | IntPatchWT_DisLastFirst | IntPatchWT_DisLastLast))
            {
                continue;
            }

            for (int numOfLine2 = numOfLine1 + 1; numOfLine2 <= slin.Length(); numOfLine2++)
            {
                WLine? wLine2 = slin.Value(numOfLine2) as WLine;

                if (wLine2 is null)
                {
                    continue;
                }

                if (wLine2.Vertex(1).ParameterOnLine() != 1)
                {
                    continue;
                }

                if (wLine2.Vertex(wLine2.VertexCount).ParameterOnLine() != wLine2.NbPnts())
                {
                    continue;
                }

                bool hasBeenJoined = false;

                int nbPntsWL2 = wLine2.NbPnts();

                // OCCT's references to this line's nodes, kept with aWLine1's for this iteration.
                refs.RemoveRange(4, refs.Count - 4);
                PntRef pntFWL2 = new PntRef(wLine2.Curve(), 1);
                PntRef pntFp1WL2 = new PntRef(wLine2.Curve(), 2);

                PntRef pntLWL2 = new PntRef(wLine2.Curve(), nbPntsWL2);
                PntRef pntLm1WL2 = new PntRef(wLine2.Curve(), nbPntsWL2 - 1);
                refs.Add(pntFWL2);
                refs.Add(pntFp1WL2);
                refs.Add(pntLWL2);
                refs.Add(pntLm1WL2);

                if (IsNeedSkipWL(wLine2, boxS1, boxS2, arrPeriods))
                {
                    continue;
                }

                if ((checkResult & IntPatchWT_DisFirstFirst) == 0)
                { // First/First
                    vec1 = vec1.WithXYZ(pntFp1WL1.Value().Value().XYZ() - pntFWL1.Value().Value().XYZ());
                    vec2 = vec2.WithXYZ(pntFWL2.Value().Value().XYZ() - pntFp1WL2.Value().Value().XYZ());
                    vec3 = vec3.WithXYZ(pntFWL1.Value().Value().XYZ() - pntFWL2.Value().Value().XYZ());

                    ExtendTwoWLFirstFirst(s1, s2, wLine1, wLine2, pntFWL1.Value(), pntFWL2.Value(), vec1,
                                          vec2, vec3, boxS1, boxS2, toler3D, arrPeriods,
                                          ref checkResult, ref hasBeenJoined, refs);
                }

                if ((checkResult & IntPatchWT_DisFirstLast) == 0)
                { // First/Last
                    vec1 = vec1.WithXYZ(pntFp1WL1.Value().Value().XYZ() - pntFWL1.Value().Value().XYZ());
                    vec2 = vec2.WithXYZ(pntLWL2.Value().Value().XYZ() - pntLm1WL2.Value().Value().XYZ());
                    vec3 = vec3.WithXYZ(pntFWL1.Value().Value().XYZ() - pntLWL2.Value().Value().XYZ());

                    ExtendTwoWLFirstLast(s1, s2, wLine1, wLine2, pntFWL1.Value(), pntLWL2.Value(), vec1,
                                         vec2, vec3, boxS1, boxS2, toler3D, arrPeriods,
                                         ref checkResult, ref hasBeenJoined, refs);
                }

                if ((checkResult & IntPatchWT_DisLastFirst) == 0)
                { // Last/First
                    vec1 = vec1.WithXYZ(pntLWL1.Value().Value().XYZ() - pntLm1WL1.Value().Value().XYZ());
                    vec2 = vec2.WithXYZ(pntFp1WL2.Value().Value().XYZ() - pntFWL2.Value().Value().XYZ());
                    vec3 = vec3.WithXYZ(pntFWL2.Value().Value().XYZ() - pntLWL1.Value().Value().XYZ());

                    ExtendTwoWLLastFirst(s1, s2, wLine1, wLine2, pntLWL1.Value(), pntFWL2.Value(), vec1,
                                         vec2, vec3, boxS1, boxS2, toler3D, arrPeriods,
                                         ref checkResult, ref hasBeenJoined, refs);
                }

                if ((checkResult & IntPatchWT_DisLastLast) == 0)
                { // Last/Last
                    vec1 = vec1.WithXYZ(pntLWL1.Value().Value().XYZ() - pntLm1WL1.Value().Value().XYZ());
                    vec2 = vec2.WithXYZ(pntLm1WL2.Value().Value().XYZ() - pntLWL2.Value().Value().XYZ());
                    vec3 = vec3.WithXYZ(pntLWL2.Value().Value().XYZ() - pntLWL1.Value().Value().XYZ());

                    ExtendTwoWLLastLast(s1, s2, wLine1, wLine2, pntLWL1.Value(), pntLWL2.Value(), vec1,
                                        vec2, vec3, boxS1, boxS2, toler3D, arrPeriods,
                                        ref checkResult, ref hasBeenJoined);
                }

                if (hasBeenJoined)
                {
                    hasBeenJoinedCounter++;
                    slin.Remove(numOfLine2);
                    numOfLine2--;
                }
            }
        }
    }
}
