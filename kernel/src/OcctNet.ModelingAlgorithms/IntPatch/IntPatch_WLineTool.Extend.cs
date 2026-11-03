// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLineTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2014 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

// ExtendTwoWLines and its helpers.
public static partial class IntPatch_WLineTool
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
        internal readonly IntSurf_LineOn2S Line;
        internal int Index;

        internal PntRef(IntSurf_LineOn2S theLine, int theIndex)
        {
            Line = theLine;
            Index = theIndex;
        }

        internal IntSurf_PntOn2S Value() => Line.Value(Index);
    }

    // theLine->InsertBefore(theIndex, P), keeping OCCT's node references (PntRef) on their nodes.
    private static void InsertBefore(IntSurf_LineOn2S theLine, int theIndex, in IntSurf_PntOn2S P,
                                     List<PntRef> theRefs)
    {
        theLine.InsertBefore(theIndex, P);
        foreach (PntRef aRef in theRefs)
        {
            if (ReferenceEquals(aRef.Line, theLine) && aRef.Index >= theIndex)
            {
                aRef.Index++;
            }
        }
    }

    // Returns True if thePmid is intersection point between theS1 and theS2 with given tolerance. In this case,
    // parameters of thePmid on every quadric will be recomputed and returned.
    private static bool IsIntersectionPoint(in Point3D thePmid, Adaptor3d_Surface theS1, Adaptor3d_Surface theS2,
                                            in IntSurf_PntOn2S theRefPt, double theTol,
                                            ReadOnlySpan<double> theArrPeriods, ref IntSurf_PntOn2S theNewPt)
    {
        double aU1 = 0.0, aV1 = 0.0, aU2 = 0.0, aV2 = 0.0;

        switch (theS1.GetType())
        {
            case Plane:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(theS1.Plane(), thePmid, out aU1, out aV1);
                break;

            case SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(theS1.Cylinder(), thePmid, out aU1, out aV1);
                break;

            case SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(theS1.Sphere(), thePmid, out aU1, out aV1);
                break;

            case SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(theS1.Cone(), thePmid, out aU1, out aV1);
                break;

            case SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(theS1.Torus(), thePmid, out aU1, out aV1);
                break;

            default:
                return false;
        }

        switch (theS2.GetType())
        {
            case Plane:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(theS2.Plane(), thePmid, out aU2, out aV2);
                break;

            case SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(theS2.Cylinder(), thePmid, out aU2, out aV2);
                break;

            case SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(theS2.Sphere(), thePmid, out aU2, out aV2);
                break;

            case SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(theS2.Cone(), thePmid, out aU2, out aV2);
                break;

            case SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(theS2.Torus(), thePmid, out aU2, out aV2);
                break;

            default:
                return false;
        }

        theNewPt.SetValue(thePmid, aU1, aV1, aU2, aV2);

        IntPatch_SpecialPoints.AdjustPointAndVertex(theRefPt, theArrPeriods, ref theNewPt);

        Point3D aP1 = theS1.Value(aU1, aV1);
        Point3D aP2 = theS2.Value(aU2, aV2);

        return (aP1.SquareDistance(aP2) <= theTol * theTol);
    }

    // Adds thePOn2S to the begin of theWline.
    private static void ExtendFirst(IntPatch_WLine theWline, in IntSurf_PntOn2S theAddedPt, List<PntRef> theRefs)
    {
        theAddedPt.Parameters(out double aU1, out double aV1, out double aU2, out double aV2);

        if (theAddedPt.IsSame(theWline.Point(1), global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            theWline.Curve().Value(1, theAddedPt);
            for (int i = 1; i <= theWline.NbVertex(); i++)
            {
                ref IntPatch_Point aVert = ref theWline.ChangeVertex(i);
                if (aVert.ParameterOnLine() != 1)
                {
                    break;
                }

                aVert.SetParameters(aU1, aV1, aU2, aV2);
                aVert.SetValue(theAddedPt.Value());
            }

            return;
        }

        InsertBefore(theWline.Curve(), 1, theAddedPt, theRefs);

        for (int i = 1; i <= theWline.NbVertex(); i++)
        {
            ref IntPatch_Point aVert = ref theWline.ChangeVertex(i);

            if (aVert.ParameterOnLine() == 1)
            {
                aVert.SetParameters(aU1, aV1, aU2, aV2);
                aVert.SetValue(theAddedPt.Value());
            }
            else
            {
                aVert.SetParameter(aVert.ParameterOnLine() + 1);
            }
        }
    }

    // Adds thePOn2S to the end of theWline.
    private static void ExtendLast(IntPatch_WLine theWline, in IntSurf_PntOn2S theAddedPt)
    {
        theAddedPt.Parameters(out double aU1, out double aV1, out double aU2, out double aV2);

        int aNbPnts = theWline.NbPnts();
        if (theAddedPt.IsSame(theWline.Point(aNbPnts), global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            theWline.Curve().Value(aNbPnts, theAddedPt);
        }
        else
        {
            theWline.Curve().Add(theAddedPt);
        }

        for (int i = theWline.NbVertex(); i >= 1; i--)
        {
            ref IntPatch_Point aVert = ref theWline.ChangeVertex(i);
            if (aVert.ParameterOnLine() != aNbPnts)
            {
                break;
            }

            aVert.SetParameters(aU1, aV1, aU2, aV2);
            aVert.SetValue(theAddedPt.Value());
            aVert.SetParameter(theWline.NbPnts());
        }
    }

    // Checks, if 2D-representation of thePOn2S is in surfaces domain, defined by bounding-boxes theBoxS1 and theBoxS2.
    private static bool IsOutOfDomain(in Box2d theBoxS1, in Box2d theBoxS2, in IntSurf_PntOn2S thePOn2S,
                                      ReadOnlySpan<double> theArrPeriods)
    {
        thePOn2S.Parameters(out double aU1, out double aV1, out double aU2, out double aV2);

        theBoxS1.Get(out double aU1min, out double aV1min, out double aU1max, out double aV1max);
        theBoxS2.Get(out double aU2min, out double aV2min, out double aU2max, out double aV2max);

        aU1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(aU1, aU1min, aU1min + theArrPeriods[0]);
        aV1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(aV1, aV1min, aV1min + theArrPeriods[1]);
        aU2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(aU2, aU2min, aU2min + theArrPeriods[2]);
        aV2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(aV2, aV2min, aV2min + theArrPeriods[3]);

        return (theBoxS1.IsOut(new Point2D(aU1, aV1)) || theBoxS2.IsOut(new Point2D(aU2, aV2)));
    }

    // Check if extending is possible (see ExtendTwoWLines).
    private static IntPatchWT_WLsConnectionType CheckArgumentsToExtend(
        Adaptor3d_Surface theS1, Adaptor3d_Surface theS2, in IntSurf_PntOn2S thePtWL1, in IntSurf_PntOn2S thePtWL2,
        ref IntSurf_PntOn2S theNewPoint, in Vector3D theVec1, in Vector3D theVec2, in Vector3D theVec3,
        in Box2d theBoxS1, in Box2d theBoxS2, double theToler3D, ReadOnlySpan<double> theArrPeriods)
    {
        double aSqToler = theToler3D * theToler3D;
        IntPatchWT_WLsConnectionType aRetVal = IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected;
        if (theVec3.SquareMagnitude() <= aSqToler)
        {
            if ((theVec1.Angle(theVec2) > myMaxConcatAngle))
            {
                return aRetVal;
            }
            else
            {
                aRetVal = IntPatchWT_WLsConnectionType.IntPatchWT_Common;
            }
        }
        else if ((theVec1.Angle(theVec2) > myMaxConcatAngle) || (theVec1.Angle(theVec3) > myMaxConcatAngle)
                 || (theVec2.Angle(theVec3) > myMaxConcatAngle))
        {
            return aRetVal;
        }

        Point3D aPmid = new Point3D(0.5 * (thePtWL1.Value().XYZ() + thePtWL2.Value().XYZ()));

        Span<double> aNewPar = stackalloc double[4];

        // Left-bottom corner (OCCT writes the boxes' upper bounds into aNewPar[0], overwritten below)
        Span<double> aParLBC = stackalloc double[4];
        theBoxS1.Get(out aParLBC[0], out aParLBC[1], out _, out _);
        theBoxS2.Get(out aParLBC[2], out aParLBC[3], out _, out _);

        if (!IsIntersectionPoint(aPmid, theS1, theS2, thePtWL1, theToler3D, theArrPeriods, ref theNewPoint))
        {
            return IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected;
        }

        if (IsOutOfDomain(theBoxS1, theBoxS2, theNewPoint, theArrPeriods))
        {
            return IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected;
        }

        Span<double> aParWL1 = stackalloc double[4], aParWL2 = stackalloc double[4];

        thePtWL1.Parameters(out aParWL1[0], out aParWL1[1], out aParWL1[2], out aParWL1[3]);
        thePtWL2.Parameters(out aParWL2[0], out aParWL2[1], out aParWL2[2], out aParWL2[3]);
        theNewPoint.Parameters(out aNewPar[0], out aNewPar[1], out aNewPar[2], out aNewPar[3]);

        BoundRange aR1 = new BoundRange(), aR2 = new BoundRange();

        bool isOnBoundary = false;
        for (int i = 0; i < 4; i++)
        {
            if (theArrPeriods[i] == 0.0)
            {
                // Strictly equal
                continue;
            }

            aR1.SetVoid();
            aR1.Add(aParWL1[i]);
            aR1.Add(aParWL2[i]);

            if (aR1.IsIntersected(aParLBC[i], theArrPeriods[i]) != BoundRange.IntersectStatus.IntersectStatus_Out)
            {
                // Check, if we intersect surface boundary when we will extend Wline1 or Wline2
                // to theNewPoint
                MinMax(ref aParWL1[i], ref aParWL2[i]);
                if (aNewPar[i] > aParWL2[i])
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
                    double aPar = aParWL1[i] + theArrPeriods[i] * Math.Ceiling((aNewPar[i] - aParWL1[i]) / theArrPeriods[i]);
                    aParWL1[i] = aParWL2[i];
                    aParWL2[i] = aPar;
                }
                else if (aNewPar[i] < aParWL1[i])
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

                    double aPar = aParWL2[i] - theArrPeriods[i] * Math.Ceiling((aParWL2[i] - aNewPar[i]) / theArrPeriods[i]);
                    aParWL2[i] = aParWL1[i];
                    aParWL1[i] = aPar;
                }

                aR1.SetVoid();
                aR2.SetVoid();
                aR1.Add(aParWL1[i]);
                aR1.Add(aNewPar[i]);
                aR2.Add(aNewPar[i]);
                aR2.Add(aParWL2[i]);

                if (aR1.IsIntersected(aParLBC[i], theArrPeriods[i]) != BoundRange.IntersectStatus.IntersectStatus_Out
                    || aR2.IsIntersected(aParLBC[i], theArrPeriods[i]) != BoundRange.IntersectStatus.IntersectStatus_Out)
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

        if (aRetVal == IntPatchWT_WLsConnectionType.IntPatchWT_Common)
        {
            return IntPatchWT_WLsConnectionType.IntPatchWT_Common;
        }

        return IntPatchWT_WLsConnectionType.IntPatchWT_ReqExtend;
    }

    // Performs extending theWLine1 and theWLine2 through their respecting start point.
    private static void ExtendTwoWLFirstFirst(Adaptor3d_Surface theS1, Adaptor3d_Surface theS2,
                                              IntPatch_WLine theWLine1, IntPatch_WLine theWLine2,
                                              in IntSurf_PntOn2S thePtWL1, in IntSurf_PntOn2S thePtWL2,
                                              in Vector3D theVec1, in Vector3D theVec2, in Vector3D theVec3,
                                              in Box2d theBoxS1, in Box2d theBoxS2, double theToler3D,
                                              ReadOnlySpan<double> theArrPeriods, ref uint theCheckResult,
                                              ref bool theHasBeenJoined, List<PntRef> theRefs)
    {
        IntSurf_PntOn2S aPOn2S = new IntSurf_PntOn2S();
        IntPatchWT_WLsConnectionType aCheckRes = CheckArgumentsToExtend(theS1, theS2, thePtWL1, thePtWL2, ref aPOn2S,
                                                                        theVec1, theVec2, theVec3, theBoxS1, theBoxS2,
                                                                        theToler3D, theArrPeriods);

        if (aCheckRes != IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected)
        {
            theCheckResult |= (IntPatchWT_DisFirstLast | IntPatchWT_DisLastFirst);
        }
        else
        {
            return;
        }

        IntPatch_SpecialPoints.AdjustPointAndVertex(thePtWL1, theArrPeriods, ref aPOn2S);
        ExtendFirst(theWLine1, aPOn2S, theRefs);
        IntPatch_SpecialPoints.AdjustPointAndVertex(thePtWL2, theArrPeriods, ref aPOn2S);
        ExtendFirst(theWLine2, aPOn2S, theRefs);

        if (theHasBeenJoined || (aCheckRes == IntPatchWT_WLsConnectionType.IntPatchWT_Singular))
        {
            return;
        }

        double aPrm = theWLine1.Vertex(1).ParameterOnLine();
        while (theWLine1.Vertex(1).ParameterOnLine() == aPrm)
        {
            theWLine1.RemoveVertex(1);
        }

        aPrm = theWLine2.Vertex(1).ParameterOnLine();
        while (theWLine2.Vertex(1).ParameterOnLine() == aPrm)
        {
            theWLine2.RemoveVertex(1);
        }

        int aNbPts = theWLine2.NbPnts();
        for (int aNPt = 2; aNPt <= aNbPts; aNPt++)
        {
            IntSurf_PntOn2S aPt = theWLine2.Point(aNPt);
            InsertBefore(theWLine1.Curve(), 1, aPt, theRefs);
        }

        for (int aNVtx = 1; aNVtx <= theWLine1.NbVertex(); aNVtx++)
        {
            ref IntPatch_Point aVert = ref theWLine1.ChangeVertex(aNVtx);
            double aCurParam = aVert.ParameterOnLine();
            aVert.SetParameter(aNbPts + aCurParam - 1);
        }

        for (int aNVtx = 1; aNVtx <= theWLine2.NbVertex(); aNVtx++)
        {
            ref IntPatch_Point aVert = ref theWLine2.ChangeVertex(aNVtx);
            double aCurParam = aVert.ParameterOnLine();
            aVert.SetParameter(aNbPts - aCurParam + 1);
            theWLine1.AddVertex(aVert, true);
        }

        theHasBeenJoined = true;
    }

    // Performs extending theWLine1 through its start point and theWLine2 through its end point.
    private static void ExtendTwoWLFirstLast(Adaptor3d_Surface theS1, Adaptor3d_Surface theS2,
                                             IntPatch_WLine theWLine1, IntPatch_WLine theWLine2,
                                             in IntSurf_PntOn2S thePtWL1, in IntSurf_PntOn2S thePtWL2,
                                             in Vector3D theVec1, in Vector3D theVec2, in Vector3D theVec3,
                                             in Box2d theBoxS1, in Box2d theBoxS2, double theToler3D,
                                             ReadOnlySpan<double> theArrPeriods, ref uint theCheckResult,
                                             ref bool theHasBeenJoined, List<PntRef> theRefs)
    {
        IntSurf_PntOn2S aPOn2S = new IntSurf_PntOn2S();
        IntPatchWT_WLsConnectionType aCheckRes = CheckArgumentsToExtend(theS1, theS2, thePtWL1, thePtWL2, ref aPOn2S,
                                                                        theVec1, theVec2, theVec3, theBoxS1, theBoxS2,
                                                                        theToler3D, theArrPeriods);

        if (aCheckRes != IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected)
        {
            theCheckResult |= IntPatchWT_DisLastLast;
        }
        else
        {
            return;
        }

        IntPatch_SpecialPoints.AdjustPointAndVertex(thePtWL1, theArrPeriods, ref aPOn2S);
        ExtendFirst(theWLine1, aPOn2S, theRefs);
        IntPatch_SpecialPoints.AdjustPointAndVertex(thePtWL2, theArrPeriods, ref aPOn2S);
        ExtendLast(theWLine2, aPOn2S);

        if (theHasBeenJoined || (aCheckRes == IntPatchWT_WLsConnectionType.IntPatchWT_Singular))
        {
            return;
        }

        double aPrm = theWLine1.Vertex(1).ParameterOnLine();
        while (theWLine1.Vertex(1).ParameterOnLine() == aPrm)
        {
            theWLine1.RemoveVertex(1);
        }

        aPrm = theWLine2.Vertex(theWLine2.NbVertex()).ParameterOnLine();
        while (theWLine2.Vertex(theWLine2.NbVertex()).ParameterOnLine() == aPrm)
        {
            theWLine2.RemoveVertex(theWLine2.NbVertex());
        }

        int aNbPts = theWLine2.NbPnts();
        for (int aNPt = aNbPts - 1; aNPt >= 1; aNPt--)
        {
            IntSurf_PntOn2S aPt = theWLine2.Point(aNPt);
            InsertBefore(theWLine1.Curve(), 1, aPt, theRefs);
        }

        for (int aNVtx = 1; aNVtx <= theWLine1.NbVertex(); aNVtx++)
        {
            ref IntPatch_Point aVert = ref theWLine1.ChangeVertex(aNVtx);
            double aCurParam = aVert.ParameterOnLine();
            aVert.SetParameter(aNbPts + aCurParam - 1);
        }

        for (int aNVtx = theWLine2.NbVertex(); aNVtx >= 1; aNVtx--)
        {
            IntPatch_Point aVert = theWLine2.Vertex(aNVtx);
            theWLine1.AddVertex(aVert, true);
        }

        theHasBeenJoined = true;
    }

    // Performs extending theWLine1 through its end point and theWLine2 through its start point.
    private static void ExtendTwoWLLastFirst(Adaptor3d_Surface theS1, Adaptor3d_Surface theS2,
                                             IntPatch_WLine theWLine1, IntPatch_WLine theWLine2,
                                             in IntSurf_PntOn2S thePtWL1, in IntSurf_PntOn2S thePtWL2,
                                             in Vector3D theVec1, in Vector3D theVec2, in Vector3D theVec3,
                                             in Box2d theBoxS1, in Box2d theBoxS2, double theToler3D,
                                             ReadOnlySpan<double> theArrPeriods, ref uint theCheckResult,
                                             ref bool theHasBeenJoined, List<PntRef> theRefs)
    {
        IntSurf_PntOn2S aPOn2S = new IntSurf_PntOn2S();
        IntPatchWT_WLsConnectionType aCheckRes = CheckArgumentsToExtend(theS1, theS2, thePtWL1, thePtWL2, ref aPOn2S,
                                                                        theVec1, theVec2, theVec3, theBoxS1, theBoxS2,
                                                                        theToler3D, theArrPeriods);

        if (aCheckRes != IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected)
        {
            theCheckResult |= IntPatchWT_DisLastLast;
        }
        else
        {
            return;
        }

        IntPatch_SpecialPoints.AdjustPointAndVertex(thePtWL1, theArrPeriods, ref aPOn2S);
        ExtendLast(theWLine1, aPOn2S);
        IntPatch_SpecialPoints.AdjustPointAndVertex(thePtWL2, theArrPeriods, ref aPOn2S);
        ExtendFirst(theWLine2, aPOn2S, theRefs);

        if (theHasBeenJoined || (aCheckRes == IntPatchWT_WLsConnectionType.IntPatchWT_Singular))
        {
            return;
        }

        double aPrm = theWLine1.Vertex(theWLine1.NbVertex()).ParameterOnLine();
        while (theWLine1.Vertex(theWLine1.NbVertex()).ParameterOnLine() == aPrm)
        {
            theWLine1.RemoveVertex(theWLine1.NbVertex());
        }

        aPrm = theWLine2.Vertex(1).ParameterOnLine();
        while (theWLine2.Vertex(1).ParameterOnLine() == aPrm)
        {
            theWLine2.RemoveVertex(1);
        }

        int aNbPts = theWLine1.NbPnts();
        for (int aNPt = 2; aNPt <= theWLine2.NbPnts(); aNPt++)
        {
            IntSurf_PntOn2S aPt = theWLine2.Point(aNPt);
            theWLine1.Curve().Add(aPt);
        }

        for (int aNVtx = 1; aNVtx <= theWLine2.NbVertex(); aNVtx++)
        {
            ref IntPatch_Point aVert = ref theWLine2.ChangeVertex(aNVtx);
            double aCurParam = aVert.ParameterOnLine();
            aVert.SetParameter(aNbPts + aCurParam - 1);
            theWLine1.AddVertex(aVert, false);
        }

        theHasBeenJoined = true;
    }

    // Performs extending theWLine1 and theWLine2 through their end points.
    private static void ExtendTwoWLLastLast(Adaptor3d_Surface theS1, Adaptor3d_Surface theS2,
                                            IntPatch_WLine theWLine1, IntPatch_WLine theWLine2,
                                            in IntSurf_PntOn2S thePtWL1, in IntSurf_PntOn2S thePtWL2,
                                            in Vector3D theVec1, in Vector3D theVec2, in Vector3D theVec3,
                                            in Box2d theBoxS1, in Box2d theBoxS2, double theToler3D,
                                            ReadOnlySpan<double> theArrPeriods, ref uint theCheckResult,
                                            ref bool theHasBeenJoined)
    {
        IntSurf_PntOn2S aPOn2S = new IntSurf_PntOn2S();
        IntPatchWT_WLsConnectionType aCheckRes = CheckArgumentsToExtend(theS1, theS2, thePtWL1, thePtWL2, ref aPOn2S,
                                                                        theVec1, theVec2, theVec3, theBoxS1, theBoxS2,
                                                                        theToler3D, theArrPeriods);

        if (aCheckRes != IntPatchWT_WLsConnectionType.IntPatchWT_NotConnected)
        {
            theCheckResult |= IntPatchWT_DisLastLast;
        }
        else
        {
            return;
        }

        IntPatch_SpecialPoints.AdjustPointAndVertex(thePtWL1, theArrPeriods, ref aPOn2S);
        ExtendLast(theWLine1, aPOn2S);
        IntPatch_SpecialPoints.AdjustPointAndVertex(thePtWL2, theArrPeriods, ref aPOn2S);
        ExtendLast(theWLine2, aPOn2S);

        if (theHasBeenJoined || (aCheckRes == IntPatchWT_WLsConnectionType.IntPatchWT_Singular))
        {
            return;
        }

        double aPrm = theWLine1.Vertex(theWLine1.NbVertex()).ParameterOnLine();
        while (theWLine1.Vertex(theWLine1.NbVertex()).ParameterOnLine() == aPrm)
        {
            theWLine1.RemoveVertex(theWLine1.NbVertex());
        }

        aPrm = theWLine2.Vertex(theWLine2.NbVertex()).ParameterOnLine();
        while (theWLine2.Vertex(theWLine2.NbVertex()).ParameterOnLine() == aPrm)
        {
            theWLine2.RemoveVertex(theWLine2.NbVertex());
        }

        int aNbPts = theWLine1.NbPnts() + theWLine2.NbPnts();
        for (int aNPt = theWLine2.NbPnts() - 1; aNPt >= 1; aNPt--)
        {
            IntSurf_PntOn2S aPt = theWLine2.Point(aNPt);
            theWLine1.Curve().Add(aPt);
        }

        for (int aNVtx = theWLine2.NbVertex(); aNVtx >= 1; aNVtx--)
        {
            ref IntPatch_Point aVert = ref theWLine2.ChangeVertex(aNVtx);
            double aCurParam = aVert.ParameterOnLine();
            aVert.SetParameter(aNbPts - aCurParam);
            theWLine1.AddVertex(aVert, false);
        }

        theHasBeenJoined = true;
    }

    private static bool IsNeedSkipWL(IntPatch_WLine theWL, in Box2d theBoxS1, in Box2d theBoxS2,
                                     ReadOnlySpan<double> theArrPeriods)
    {
        double aFirstp, aLastp;
        int aNbVtx = theWL.NbVertex();

        for (int i = 1; i < aNbVtx; i++)
        {
            aFirstp = theWL.Vertex(i).ParameterOnLine();
            aLastp = theWL.Vertex(i + 1).ParameterOnLine();

            int pmid = (int)((aFirstp + aLastp) / 2);
            IntSurf_PntOn2S aPmid = theWL.Point(pmid);

            if (IsOutOfDomain(theBoxS1, theBoxS2, aPmid, theArrPeriods))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Extends the walking lines of <paramref name="theSlin"/> whose ends are close (within the angle
    /// <see cref="myMaxConcatAngle"/>) to their common intersection point, joining them where the point isn't on a
    /// surface boundary. <paramref name="theArrPeriods"/> holds the surfaces' periods (U1, V1, U2, V2; 0 if not
    /// periodic); the boxes are the surfaces' parametric domains. Lines ending at one of
    /// <paramref name="theListOfCriticalPoints"/> aren't extended there. The surfaces must be quadrics.
    /// </summary>
    public static void ExtendTwoWLines(NCollection_Sequence<IntPatch_Line> theSlin, Adaptor3d_Surface theS1,
                                       Adaptor3d_Surface theS2, double theToler3D, ReadOnlySpan<double> theArrPeriods,
                                       in Box2d theBoxS1, in Box2d theBoxS2,
                                       NCollection_List<Point3D> theListOfCriticalPoints)
    {
        if (theSlin.Length() < 2)
        {
            return;
        }

        Vector3D aVec1 = new Vector3D(), aVec2 = new Vector3D(), aVec3 = new Vector3D();

        uint hasBeenJoinedCounter = 0;

        List<PntRef> aRefs = new List<PntRef>(8);

        for (int aNumOfLine1 = 1; aNumOfLine1 <= theSlin.Length(); aNumOfLine1++)
        {
            if (hasBeenJoinedCounter > 0)
            {
                aNumOfLine1--;
            }

            hasBeenJoinedCounter = 0;

            IntPatch_WLine? aWLine1 = theSlin.Value(aNumOfLine1) as IntPatch_WLine;

            if (aWLine1 is null)
            { // We must have failed to join not-point-lines
                continue;
            }

            int aNbPntsWL1 = aWLine1.NbPnts();

            if (aWLine1.Vertex(1).ParameterOnLine() != 1)
            {
                continue;
            }

            if (aWLine1.Vertex(aWLine1.NbVertex()).ParameterOnLine() != aWLine1.NbPnts())
            {
                continue;
            }

            // OCCT's references to the line's nodes (PntRef).
            aRefs.Clear();
            PntRef aPntFWL1 = new PntRef(aWLine1.Curve(), 1);
            PntRef aPntFp1WL1 = new PntRef(aWLine1.Curve(), 2);

            PntRef aPntLWL1 = new PntRef(aWLine1.Curve(), aNbPntsWL1);
            PntRef aPntLm1WL1 = new PntRef(aWLine1.Curve(), aNbPntsWL1 - 1);
            aRefs.Add(aPntFWL1);
            aRefs.Add(aPntFp1WL1);
            aRefs.Add(aPntLWL1);
            aRefs.Add(aPntLm1WL1);

            if (IsNeedSkipWL(aWLine1, theBoxS1, theBoxS2, theArrPeriods))
            {
                continue;
            }

            // Enable/Disable of some check. Bit-mask is used for it.
            // E.g. if 1st point of aWLine1 matches with
            // 1st point of aWLine2 then we do not need in check
            // 1st point of aWLine1 and last point of aWLine2 etc.
            uint aCheckResult = IntPatchWT_EnAll;

            // If aWLine1 is already connected with another Wline then
            // there is no point in extending.
            for (int aNumOfLine2 = aNumOfLine1 + 1; aNumOfLine2 <= theSlin.Length(); aNumOfLine2++)
            {
                IntPatch_WLine? aWLine2 = theSlin.Value(aNumOfLine2) as IntPatch_WLine;

                if (aWLine2 is null)
                {
                    continue;
                }

                IntSurf_PntOn2S aPntFWL2 = aWLine2.Point(1);
                IntSurf_PntOn2S aPntLWL2 = aWLine2.Point(aWLine2.NbPnts());

                if (!(aPntFWL1.Value().IsSame(aPntFWL2, theToler3D, global::Kerf.Engine.Foundation.Precision.PConfusion()))
                    && !(aPntFWL1.Value().IsSame(aPntLWL2, theToler3D, global::Kerf.Engine.Foundation.Precision.PConfusion())))
                {
                    if (aPntFWL1.Value().IsSame(aPntFWL2, theToler3D) || aPntFWL1.Value().IsSame(aPntLWL2, theToler3D))
                    {
                        aCheckResult |= IntPatchWT_DisFirstFirst | IntPatchWT_DisFirstLast;
                    }
                }

                if (!(aPntLWL1.Value().IsSame(aPntFWL2, theToler3D, global::Kerf.Engine.Foundation.Precision.PConfusion()))
                    && !(aPntLWL1.Value().IsSame(aPntLWL2, theToler3D, global::Kerf.Engine.Foundation.Precision.PConfusion())))
                {
                    if (aPntLWL1.Value().IsSame(aPntFWL2, theToler3D) || aPntLWL1.Value().IsSame(aPntLWL2, theToler3D))
                    {
                        aCheckResult |= IntPatchWT_DisLastFirst | IntPatchWT_DisLastLast;
                    }
                }

                if (!theListOfCriticalPoints.IsEmpty())
                {
                    for (NCollection_List<Point3D>.Iterator anItr = new(theListOfCriticalPoints); anItr.More();
                         anItr.Next())
                    {
                        Point3D aPt = anItr.Value();
                        if ((aCheckResult & (IntPatchWT_DisFirstFirst | IntPatchWT_DisFirstLast)) == 0)
                        {
                            if (aPt.SquareDistance(aPntFWL1.Value().Value()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                aCheckResult |= IntPatchWT_DisFirstFirst | IntPatchWT_DisFirstLast;
                            }
                        }

                        if ((aCheckResult & (IntPatchWT_DisLastFirst | IntPatchWT_DisLastLast)) == 0)
                        {
                            if (aPt.SquareDistance(aPntLWL1.Value().Value()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                aCheckResult |= IntPatchWT_DisLastFirst | IntPatchWT_DisLastLast;
                            }
                        }

                        if ((aCheckResult & (IntPatchWT_DisFirstFirst | IntPatchWT_DisLastFirst)) == 0)
                        {
                            if (aPt.SquareDistance(aPntFWL2.Value()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                aCheckResult |= IntPatchWT_DisFirstFirst | IntPatchWT_DisLastFirst;
                            }
                        }

                        if ((aCheckResult & (IntPatchWT_DisFirstLast | IntPatchWT_DisLastLast)) == 0)
                        {
                            if (aPt.SquareDistance(aPntLWL2.Value()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                aCheckResult |= IntPatchWT_DisFirstLast | IntPatchWT_DisLastLast;
                            }
                        }
                    }
                }
            }

            if (aCheckResult
                == (IntPatchWT_DisFirstFirst | IntPatchWT_DisFirstLast | IntPatchWT_DisLastFirst | IntPatchWT_DisLastLast))
            {
                continue;
            }

            for (int aNumOfLine2 = aNumOfLine1 + 1; aNumOfLine2 <= theSlin.Length(); aNumOfLine2++)
            {
                IntPatch_WLine? aWLine2 = theSlin.Value(aNumOfLine2) as IntPatch_WLine;

                if (aWLine2 is null)
                {
                    continue;
                }

                if (aWLine2.Vertex(1).ParameterOnLine() != 1)
                {
                    continue;
                }

                if (aWLine2.Vertex(aWLine2.NbVertex()).ParameterOnLine() != aWLine2.NbPnts())
                {
                    continue;
                }

                bool hasBeenJoined = false;

                int aNbPntsWL2 = aWLine2.NbPnts();

                // OCCT's references to this line's nodes, kept with aWLine1's for this iteration.
                aRefs.RemoveRange(4, aRefs.Count - 4);
                PntRef aPntFWL2 = new PntRef(aWLine2.Curve(), 1);
                PntRef aPntFp1WL2 = new PntRef(aWLine2.Curve(), 2);

                PntRef aPntLWL2 = new PntRef(aWLine2.Curve(), aNbPntsWL2);
                PntRef aPntLm1WL2 = new PntRef(aWLine2.Curve(), aNbPntsWL2 - 1);
                aRefs.Add(aPntFWL2);
                aRefs.Add(aPntFp1WL2);
                aRefs.Add(aPntLWL2);
                aRefs.Add(aPntLm1WL2);

                if (IsNeedSkipWL(aWLine2, theBoxS1, theBoxS2, theArrPeriods))
                {
                    continue;
                }

                if ((aCheckResult & IntPatchWT_DisFirstFirst) == 0)
                { // First/First
                    aVec1.SetXYZ(aPntFp1WL1.Value().Value().XYZ() - aPntFWL1.Value().Value().XYZ());
                    aVec2.SetXYZ(aPntFWL2.Value().Value().XYZ() - aPntFp1WL2.Value().Value().XYZ());
                    aVec3.SetXYZ(aPntFWL1.Value().Value().XYZ() - aPntFWL2.Value().Value().XYZ());

                    ExtendTwoWLFirstFirst(theS1, theS2, aWLine1, aWLine2, aPntFWL1.Value(), aPntFWL2.Value(), aVec1,
                                          aVec2, aVec3, theBoxS1, theBoxS2, theToler3D, theArrPeriods,
                                          ref aCheckResult, ref hasBeenJoined, aRefs);
                }

                if ((aCheckResult & IntPatchWT_DisFirstLast) == 0)
                { // First/Last
                    aVec1.SetXYZ(aPntFp1WL1.Value().Value().XYZ() - aPntFWL1.Value().Value().XYZ());
                    aVec2.SetXYZ(aPntLWL2.Value().Value().XYZ() - aPntLm1WL2.Value().Value().XYZ());
                    aVec3.SetXYZ(aPntFWL1.Value().Value().XYZ() - aPntLWL2.Value().Value().XYZ());

                    ExtendTwoWLFirstLast(theS1, theS2, aWLine1, aWLine2, aPntFWL1.Value(), aPntLWL2.Value(), aVec1,
                                         aVec2, aVec3, theBoxS1, theBoxS2, theToler3D, theArrPeriods,
                                         ref aCheckResult, ref hasBeenJoined, aRefs);
                }

                if ((aCheckResult & IntPatchWT_DisLastFirst) == 0)
                { // Last/First
                    aVec1.SetXYZ(aPntLWL1.Value().Value().XYZ() - aPntLm1WL1.Value().Value().XYZ());
                    aVec2.SetXYZ(aPntFp1WL2.Value().Value().XYZ() - aPntFWL2.Value().Value().XYZ());
                    aVec3.SetXYZ(aPntFWL2.Value().Value().XYZ() - aPntLWL1.Value().Value().XYZ());

                    ExtendTwoWLLastFirst(theS1, theS2, aWLine1, aWLine2, aPntLWL1.Value(), aPntFWL2.Value(), aVec1,
                                         aVec2, aVec3, theBoxS1, theBoxS2, theToler3D, theArrPeriods,
                                         ref aCheckResult, ref hasBeenJoined, aRefs);
                }

                if ((aCheckResult & IntPatchWT_DisLastLast) == 0)
                { // Last/Last
                    aVec1.SetXYZ(aPntLWL1.Value().Value().XYZ() - aPntLm1WL1.Value().Value().XYZ());
                    aVec2.SetXYZ(aPntLm1WL2.Value().Value().XYZ() - aPntLWL2.Value().Value().XYZ());
                    aVec3.SetXYZ(aPntLWL2.Value().Value().XYZ() - aPntLWL1.Value().Value().XYZ());

                    ExtendTwoWLLastLast(theS1, theS2, aWLine1, aWLine2, aPntLWL1.Value(), aPntLWL2.Value(), aVec1,
                                        aVec2, aVec3, theBoxS1, theBoxS2, theToler3D, theArrPeriods,
                                        ref aCheckResult, ref hasBeenJoined);
                }

                if (hasBeenJoined)
                {
                    hasBeenJoinedCounter++;
                    theSlin.Remove(aNumOfLine2);
                    aNumOfLine2--;
                }
            }
        }
    }
}
