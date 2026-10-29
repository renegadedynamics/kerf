// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_WireOrder.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_WireOrder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Proposes an order for a set of edges given by the coordinates of their ends (in 3D, in 2D, or both), so that they
/// chain with the smallest gaps, reversing some if needed. A negative index in <see cref="Ordered"/> means the edge is
/// reversed.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Status"/> after <see cref="Perform"/>: 0 the order is unchanged, 1 some
/// edges were reordered, -1 some were reversed, 3 the edges were only shifted (in either direction).
/// </remarks>
public sealed class ShapeAnalysis_WireOrder
{
    private enum ModeType
    {
        Mode2D,
        Mode3D,
        ModeBoth,
    }

    private NCollection_HArray1<int>? myOrd;
    private NCollection_HArray1<int>? myChains;
    private NCollection_HArray1<int>? myCouples = null;
    private NCollection_HSequence<gp_XYZ> myXYZ = null!;
    private NCollection_HSequence<gp_XY> myXY = null!;
    private double myTol;
    private double myGap;
    private int myStat;
    private bool myKeepLoops;
    private ModeType myMode;

    /// <summary>An empty tool in 3D mode, with tolerance Precision::Confusion.</summary>
    public ShapeAnalysis_WireOrder()
    {
        myGap = 0.0;
        myStat = 0;
        myKeepLoops = false;
        myMode = ModeType.Mode3D;
        myTol = Precision.Confusion();
        Clear();
    }

    /// <summary>
    /// An empty tool in 3D mode if <paramref name="theMode3D"/>, else 2D; in both modes if
    /// <paramref name="theModeBoth"/>.
    /// </summary>
    public ShapeAnalysis_WireOrder(bool theMode3D, double theTolerance, bool theModeBoth = false)
    {
        myTol = theTolerance;
        myGap = 0.0;
        myStat = 0;
        myKeepLoops = false;
        if (theModeBoth)
        {
            myMode = ModeType.ModeBoth;
        }
        else
        {
            if (theMode3D)
            {
                myMode = ModeType.Mode3D;
            }
            else
            {
                myMode = ModeType.Mode2D;
            }
        }

        Clear();
    }

    /// <summary>Sets the mode and the tolerance (1e-8 if not positive); clears the edges if the mode changes.</summary>
    public void SetMode(bool theMode3D, double theTolerance, bool theModeBoth = false)
    {
        ModeType aNewMode;
        if (theModeBoth)
        {
            aNewMode = ModeType.ModeBoth;
        }
        else
        {
            if (theMode3D)
            {
                aNewMode = ModeType.Mode3D;
            }
            else
            {
                aNewMode = ModeType.Mode2D;
            }
        }

        if (myMode != aNewMode)
        {
            Clear();
        }

        myMode = aNewMode;
        myOrd = null;
        myStat = 0;
        myGap = 0.0;
        myTol = (theTolerance > 0.0) ? theTolerance : 1.0e-08;
    }

    /// <summary>The working tolerance.</summary>
    public double Tolerance()
    {
        return myTol;
    }

    /// <summary>Clears the list of edges, but not the mode and tolerance.</summary>
    public void Clear()
    {
        myXYZ = new NCollection_HSequence<gp_XYZ>();
        myXY = new NCollection_HSequence<gp_XY>();
        myStat = 0;
        myGap = 0.0;
    }

    /// <summary>Adds an edge by the 3D points of its ends (3D mode only).</summary>
    public void Add(in gp_XYZ theStart3d, in gp_XYZ theEnd3d)
    {
        if (myMode == ModeType.Mode3D)
        {
            myXYZ.Append(theStart3d);
            myXYZ.Append(theEnd3d);
        }
    }

    /// <summary>Adds an edge by the 2D points of its ends (2D mode only).</summary>
    public void Add(in gp_XY theStart2d, in gp_XY theEnd2d)
    {
        if (myMode == ModeType.Mode2D)
        {
            gp_XYZ val = new gp_XYZ();
            val.SetCoord(theStart2d.X(), theStart2d.Y(), 0.0);
            myXYZ.Append(val);
            val.SetCoord(theEnd2d.X(), theEnd2d.Y(), 0.0);
            myXYZ.Append(val);
        }
    }

    /// <summary>Adds an edge by the 3D and 2D points of its ends (both mode only).</summary>
    public void Add(in gp_XYZ theStart3d, in gp_XYZ theEnd3d, in gp_XY theStart2d, in gp_XY theEnd2d)
    {
        if (myMode == ModeType.ModeBoth)
        {
            myXYZ.Append(theStart3d);
            myXYZ.Append(theEnd3d);

            myXY.Append(theStart2d);
            myXY.Append(theEnd2d);
        }
    }

    /// <summary>The count of edges added.</summary>
    public int NbEdges()
    {
        return myXYZ.Length() / 2;
    }

    private static double DISTABS(in gp_XYZ v1, in gp_XYZ v2)
    {
        return Math.Abs(v1.X() - v2.X()) + Math.Abs(v1.Y() - v2.Y()) + Math.Abs(v1.Z() - v2.Z());
    }

    /// <summary>
    /// Whether the loops found by <see cref="Perform"/> are kept one after another (true) or merged into the main
    /// loop at their best place (false, the default).
    /// </summary>
    public ref bool KeepLoopsMode()
    {
        return ref myKeepLoops;
    }

    /// <summary>
    /// Computes the order of the edges, taking the gaps between them into account. <paramref name="closed"/> is
    /// ignored, as in OCCT.
    /// </summary>
    public void Perform(bool closed = true)
    {
        _ = closed;
        myStat = 0;
        int aNbEdges = NbEdges();
        // no edges loaded, nothing to do -- return with status OK
        if (aNbEdges == 0)
        {
            return;
        }

        myOrd = new NCollection_HArray1<int>(1, aNbEdges);
        myOrd.Init(0);

        // sequence of the edge nums in the right order
        var anEdgeSeq = new NCollection_HSequence<int>();
        var aLoops = new NCollection_Sequence<NCollection_HSequence<int>>();

        // the beginnings and ends of the edges
        var aBegins3D = new NCollection_Array1<gp_XYZ>(1, aNbEdges);
        var anEnds3D = new NCollection_Array1<gp_XYZ>(1, aNbEdges);
        var aBegins2D = new NCollection_Array1<gp_XY>(1, aNbEdges);
        var anEnds2D = new NCollection_Array1<gp_XY>(1, aNbEdges);
        for (int i = 1; i <= aNbEdges; i++)
        {
            aBegins3D[i] = myXYZ.Value(2 * i - 1);
            anEnds3D[i] = myXYZ.Value(2 * i);
            if (myMode == ModeType.ModeBoth)
            {
                aBegins2D[i] = myXY.Value(2 * i - 1);
                anEnds2D[i] = myXY.Value(2 * i);
            }
        }

        // the flags that the edges was considered
        var isEdgeUsed = new NCollection_Array1<bool>(1, aNbEdges);
        isEdgeUsed.Init(false);

        double aTol2 = Precision.SquareConfusion();
        double aTolP2 = Precision.SquarePConfusion();

        // take the first edge to the constructed chain
        isEdgeUsed[1] = true;
        gp_Pnt aFirstPnt3D = new gp_Pnt(aBegins3D[1]);
        gp_Pnt aLastPnt3D = new gp_Pnt(anEnds3D[1]);
        gp_Pnt2d aFirstPnt2D = new gp_Pnt2d();
        gp_Pnt2d aLastPnt2D = new gp_Pnt2d();
        if (myMode == ModeType.ModeBoth)
        {
            aFirstPnt2D = new gp_Pnt2d(aBegins2D[1]);
            aLastPnt2D = new gp_Pnt2d(anEnds2D[1]);
        }

        anEdgeSeq.Append(1);

        // cycle until all edges are considered
        for (;;)
        {
            // joint type
            // 0 - the start of the best edge to the end of constructed sequence (nothing to do)
            // 1 - the end of the best edge to the start of constructed sequence (need move the edge)
            // 2 - the end of the best edge to the end of constructed sequence (need to reverse)
            // 3 - the start of the best edge to the start of constructed sequence (need to reverse and move the edge)
            int aBestJointType = 3;
            // the best minimum distance between constructed sequence and the best edge
            double aBestMin3D = RealLast();
            // number of the best edge
            int aBestEdgeNum = 0;
            // the best edge was found
            bool isFound = false;
            bool isConnected = false;
            // loop to find the best edge among all the remaining
            for (int i = 1; i <= aNbEdges; i++)
            {
                if (isEdgeUsed[i])
                {
                    continue;
                }

                // find minimum distance and joint type for 3D and 2D (if necessary) modes
                int aCurJointType;
                double aCurMin;
                // distance for four possible cases
                double aSeqTailEdgeHead = aLastPnt3D.SquareDistance(new gp_Pnt(aBegins3D[i]));
                double aSeqTailEdgeTail = aLastPnt3D.SquareDistance(new gp_Pnt(anEnds3D[i]));
                double aSeqHeadEdgeTail = aFirstPnt3D.SquareDistance(new gp_Pnt(anEnds3D[i]));
                double aSeqHeadEdgeHead = aFirstPnt3D.SquareDistance(new gp_Pnt(aBegins3D[i]));
                // the best distances for joints with head and tail of sequence
                double aMinDistToTail, aMinDistToHead;
                int aTailJoinType, aHeadJointType;
                if (aSeqTailEdgeHead <= aSeqTailEdgeTail)
                {
                    aTailJoinType = 0;
                    aMinDistToTail = aSeqTailEdgeHead;
                }
                else
                {
                    aTailJoinType = 2;
                    aMinDistToTail = aSeqTailEdgeTail;
                }

                if (aSeqHeadEdgeTail <= aSeqHeadEdgeHead)
                {
                    aHeadJointType = 1;
                    aMinDistToHead = aSeqHeadEdgeTail;
                }
                else
                {
                    aHeadJointType = 3;
                    aMinDistToHead = aSeqHeadEdgeHead;
                }

                // comparing the head and the tail cases
                // if distances are close enough then we use rule for joint type: 0 < 1 < 2 < 3
                if (Math.Abs(aMinDistToTail - aMinDistToHead) < aTol2)
                {
                    if (aTailJoinType < aHeadJointType)
                    {
                        aCurJointType = aTailJoinType;
                        aCurMin = aMinDistToTail;
                    }
                    else
                    {
                        aCurJointType = aHeadJointType;
                        aCurMin = aMinDistToHead;
                    }
                }
                else
                {
                    if (aMinDistToTail <= aMinDistToHead)
                    {
                        aCurJointType = aTailJoinType;
                        aCurMin = aMinDistToTail;
                    }
                    else
                    {
                        aCurJointType = aHeadJointType;
                        aCurMin = aMinDistToHead;
                    }
                }

                // update for the best values
                if (myMode == ModeType.ModeBoth)
                {
                    // distances in 2D
                    int aJointMask3D = 0, aJointMask2D = 0;
                    if (aSeqTailEdgeHead < aTol2)
                    {
                        aJointMask3D |= (1 << 0);
                    }

                    if (aSeqTailEdgeTail < aTol2)
                    {
                        aJointMask3D |= (1 << 2);
                    }

                    if (aSeqHeadEdgeTail < aTol2)
                    {
                        aJointMask3D |= (1 << 1);
                    }

                    if (aSeqHeadEdgeHead < aTol2)
                    {
                        aJointMask3D |= (1 << 3);
                    }

                    double aSeqTailEdgeHead2D = aLastPnt2D.SquareDistance(new gp_Pnt2d(aBegins2D[i]));
                    double aSeqTailEdgeTail2D = aLastPnt2D.SquareDistance(new gp_Pnt2d(anEnds2D[i]));
                    double aSeqHeadEdgeTail2D = aFirstPnt2D.SquareDistance(new gp_Pnt2d(anEnds2D[i]));
                    double aSeqHeadEdgeHead2D = aFirstPnt2D.SquareDistance(new gp_Pnt2d(aBegins2D[i]));
                    if (aSeqTailEdgeHead2D < aTolP2)
                    {
                        aJointMask2D |= (1 << 0);
                    }

                    if (aSeqTailEdgeTail2D < aTolP2)
                    {
                        aJointMask2D |= (1 << 2);
                    }

                    if (aSeqHeadEdgeTail2D < aTolP2)
                    {
                        aJointMask2D |= (1 << 1);
                    }

                    if (aSeqHeadEdgeHead2D < aTolP2)
                    {
                        aJointMask2D |= (1 << 3);
                    }

                    // new approch for detecting best edge connection, for all other cases used old 3D algorithm
                    int aFullMask = aJointMask3D & aJointMask2D;
                    if (aFullMask != 0)
                    {
                        // find the best current joint type
                        aCurJointType = 3;
                        for (int j = 0; j < 4; j++)
                        {
                            if ((aFullMask & (1 << j)) != 0)
                            {
                                aCurJointType = j;
                                break;
                            }
                        }

                        if (!isConnected || aCurJointType < aBestJointType)
                        {
                            isFound = true;
                            isConnected = true;
                            switch (aCurJointType)
                            {
                                case 0:
                                    aBestMin3D = aSeqTailEdgeHead;
                                    break;
                                case 1:
                                    aBestMin3D = aSeqHeadEdgeTail;
                                    break;
                                case 2:
                                    aBestMin3D = aSeqTailEdgeTail;
                                    break;
                                case 3:
                                    aBestMin3D = aSeqHeadEdgeHead;
                                    break;
                            }

                            aBestJointType = aCurJointType;
                            aBestEdgeNum = i;
                        }
                    }

                    // if there is still no connection, continue to use ald 3D algorithm
                    if (isConnected)
                    {
                        continue;
                    }
                }

                // if the best distance is still not reached (aBestMin3D > aTol2) or we found a better joint type
                if (aBestMin3D > aTol2 || aCurJointType < aBestJointType)
                {
                    // make a decision that this edge is good enough:
                    // - it gets the best distance but there is fabs(aCurMin3d - aBestMin3d) < aTol2 &&
                    // (aCurJointType < aBestJointType) ?
                    // - it gets the best joint in some cases
                    if (aCurMin < aBestMin3D || ((aCurMin == aBestMin3D || aCurMin < aTol2) && (aCurJointType < aBestJointType)))
                    {
                        isFound = true;
                        aBestMin3D = aCurMin;
                        aBestJointType = aCurJointType;
                        aBestEdgeNum = i;
                    }
                }
            }

            // check that we found edge for connecting
            if (isFound)
            {
                // distance between first and last point in sequence
                double aCloseDist = aFirstPnt3D.SquareDistance(aLastPnt3D);
                // if it's better to insert the edge than to close the loop, just insert the edge according to joint type
                if (aBestMin3D <= RealSmall() || aBestMin3D < aCloseDist)
                {
                    switch (aBestJointType)
                    {
                        case 0:
                            anEdgeSeq.Append(aBestEdgeNum);
                            aLastPnt3D = new gp_Pnt(anEnds3D[aBestEdgeNum]);
                            break;
                        case 1:
                            anEdgeSeq.Prepend(aBestEdgeNum);
                            aFirstPnt3D = new gp_Pnt(aBegins3D[aBestEdgeNum]);
                            break;
                        case 2:
                            anEdgeSeq.Append(-aBestEdgeNum);
                            aLastPnt3D = new gp_Pnt(aBegins3D[aBestEdgeNum]);
                            break;
                        case 3:
                            anEdgeSeq.Prepend(-aBestEdgeNum);
                            aFirstPnt3D = new gp_Pnt(anEnds3D[aBestEdgeNum]);
                            break;
                    }

                    if (myMode == ModeType.ModeBoth)
                    {
                        switch (aBestJointType)
                        {
                            case 0:
                                aLastPnt2D = new gp_Pnt2d(anEnds2D[aBestEdgeNum]);
                                break;
                            case 1:
                                aFirstPnt2D = new gp_Pnt2d(aBegins2D[aBestEdgeNum]);
                                break;
                            case 2:
                                aLastPnt2D = new gp_Pnt2d(aBegins2D[aBestEdgeNum]);
                                break;
                            case 3:
                                aFirstPnt2D = new gp_Pnt2d(anEnds2D[aBestEdgeNum]);
                                break;
                        }
                    }
                }
                // closing loop and creating new one
                else
                {
                    aLoops.Append(anEdgeSeq);
                    anEdgeSeq = new NCollection_HSequence<int>();
                    aFirstPnt3D = new gp_Pnt(aBegins3D[aBestEdgeNum]);
                    aLastPnt3D = new gp_Pnt(anEnds3D[aBestEdgeNum]);
                    if (myMode == ModeType.ModeBoth)
                    {
                        aFirstPnt2D = new gp_Pnt2d(aBegins2D[aBestEdgeNum]);
                        aLastPnt2D = new gp_Pnt2d(anEnds2D[aBestEdgeNum]);
                    }

                    anEdgeSeq.Append(aBestEdgeNum);
                }

                // mark the edge as used
                isEdgeUsed[aBestEdgeNum] = true;
            }
            else
            {
                // the only condition under which we can't find an edge is when all edges are done
                break;
            }
        }

        // append the last loop
        aLoops.Append(anEdgeSeq);

        // handling with constructed loops
        NCollection_HSequence<int> aMainLoop;
        if (myKeepLoops)
        {
            // keeping the loops, adding one after another.
            aMainLoop = new NCollection_HSequence<int>();
            for (int i = 1; i <= aLoops.Length(); i++)
            {
                NCollection_HSequence<int> aCurLoop = aLoops[i];
                aMainLoop.Append(aCurLoop);
            }
        }
        else
        {
            // connecting loops
            aMainLoop = aLoops.First();
            aLoops.Remove(1);
            while (aLoops.Length() != 0)
            {
                // iterate over all loops to find the closest one
                double aMinDist1 = RealLast();
                int aLoopNum1 = 0;
                int aCurLoopIt1 = 0;
                bool aDirect1 = false;
                int aMainLoopIt1 = 0;
                for (int aLoopIt = 1; aLoopIt <= aLoops.Length(); aLoopIt++)
                {
                    NCollection_HSequence<int> aCurLoop = aLoops.Value(aLoopIt);
                    // iterate over all gaps between edges in current loop
                    int aCurLoopIt2 = 0;
                    int aMainLoopIt2 = 0;
                    bool aDirect2 = false;
                    double aMinDist2 = RealLast();
                    int aCurLoopLength = aCurLoop.Length();
                    for (int aCurEdgeIt = 1; aCurEdgeIt <= aCurLoopLength; aCurEdgeIt++)
                    {
                        // get the distance between the current edge and the previous edge taking into account the
                        // edge's orientation
                        int aPrevEdgeIt = aCurEdgeIt == 1 ? aCurLoopLength : aCurEdgeIt - 1;
                        int aCurEdgeIdx = aCurLoop.Value(aCurEdgeIt);
                        int aPrevEdgeIdx = aCurLoop.Value(aPrevEdgeIt);
                        gp_Pnt aCurLoopFirst = new gp_Pnt(aCurEdgeIdx > 0 ? aBegins3D[aCurEdgeIdx] : anEnds3D[-aCurEdgeIdx]);
                        gp_Pnt aCurLoopLast = new gp_Pnt(aPrevEdgeIdx > 0 ? anEnds3D[aPrevEdgeIdx] : aBegins3D[-aPrevEdgeIdx]);
                        // iterate over all gaps between edges in main loop
                        double aMinDist3 = RealLast();
                        int aMainLoopIt3 = 0;
                        bool aDirect3 = false;
                        int aMainLoopLength = aMainLoop.Length();
                        for (int aCurEdgeIt2 = 1; (aCurEdgeIt2 <= aMainLoopLength) && aMinDist3 != 0.0; aCurEdgeIt2++)
                        {
                            // get the distance between the current edge and the next edge taking into account the
                            // edge's orientation
                            int aNextEdgeIt2 = aCurEdgeIt2 == aMainLoopLength ? 1 : aCurEdgeIt2 + 1;
                            int aCurEdgeIdx2 = aMainLoop.Value(aCurEdgeIt2);
                            int aNextEdgeIdx2 = aMainLoop.Value(aNextEdgeIt2);
                            gp_Pnt aMainLoopFirst = new gp_Pnt(aCurEdgeIdx2 > 0 ? anEnds3D[aCurEdgeIdx2] : aBegins3D[-aCurEdgeIdx2]);
                            gp_Pnt aMainLoopLast = new gp_Pnt(aNextEdgeIdx2 > 0 ? aBegins3D[aNextEdgeIdx2] : anEnds3D[-aNextEdgeIdx2]);
                            // getting the sum of square distances if we try to sew the current loop with the main loop
                            // in current positions
                            double aDirectDist = aCurLoopFirst.SquareDistance(aMainLoopFirst) + aCurLoopLast.SquareDistance(aMainLoopLast);
                            double aReverseDist = aCurLoopFirst.SquareDistance(aMainLoopLast) + aCurLoopLast.SquareDistance(aMainLoopFirst);
                            // take the best result
                            double aJoinDist;
                            if ((aDirectDist < aTol2) || (aDirectDist < 2.0 * aReverseDist))
                            {
                                aJoinDist = aDirectDist;
                                aReverseDist = aDirectDist;
                            }
                            else
                            {
                                aJoinDist = aReverseDist;
                            }

                            // check if we found a better distance
                            if (aJoinDist < aMinDist3 && Math.Abs(aMinDist3 - aJoinDist) > aTol2)
                            {
                                aMinDist3 = aJoinDist;
                                aDirect3 = (aDirectDist <= aReverseDist);
                                aMainLoopIt3 = aCurEdgeIt2;
                            }
                        }

                        // check if we found a better distance
                        if (aMinDist3 < aMinDist2 && Math.Abs(aMinDist2 - aMinDist3) > aTol2)
                        {
                            aMinDist2 = aMinDist3;
                            aDirect2 = aDirect3;
                            aMainLoopIt2 = aMainLoopIt3;
                            aCurLoopIt2 = aCurEdgeIt;
                        }
                    }

                    // check if we found a better distance
                    if (aMinDist2 < aMinDist1 && Math.Abs(aMinDist1 - aMinDist2) > aTol2)
                    {
                        aMinDist1 = aMinDist2;
                        aLoopNum1 = aLoopIt;
                        aDirect1 = aDirect2;
                        aMainLoopIt1 = aMainLoopIt2;
                        aCurLoopIt1 = aCurLoopIt2;
                    }
                }

                // insert the found loop into main loop
                NCollection_HSequence<int> aLoop = aLoops.Value(aLoopNum1);
                int aFactor = (aDirect1 ? 1 : -1);
                for (int i = 0; i < aLoop.Length(); i++)
                {
                    int anIdx = (aCurLoopIt1 + i > aLoop.Length() ? aCurLoopIt1 + i - aLoop.Length() : aCurLoopIt1 + i);
                    aMainLoop.InsertAfter(aMainLoopIt1 + i, aLoop.Value(anIdx) * aFactor);
                }

                aLoops.Remove(aLoopNum1);
            }
        }

        // checking the new order of the edges
        //  0 - order is the same
        //  1 - some edges were reordered
        // -1 - some edges were reversed
        int aTempStatus = 0;
        for (int i = 1; i <= aMainLoop.Length(); i++)
        {
            if (i != aMainLoop.Value(i) && aTempStatus >= 0)
            {
                aTempStatus = (aMainLoop.Value(i) > 0 ? 1 : -1);
            }

            myOrd.SetValue(i, aMainLoop.Value(i));
        }

        if (aTempStatus == 0)
        {
            myStat = aTempStatus;
            return;
        }
        else
        {
            // check if edges were only shifted in reverse or forward, not reordered
            bool isShiftReverse = true;
            bool isShiftForward = true;
            int aFirstIdx, aSecondIdx;
            int aLength = aMainLoop.Length();
            for (int i = 1; i <= aLength - 1; i++)
            {
                aFirstIdx = aMainLoop.Value(i);
                aSecondIdx = aMainLoop.Value(i + 1);
                if (aSecondIdx - aFirstIdx != 1 && (aFirstIdx != aLength || aSecondIdx != 1))
                {
                    isShiftForward = false;
                }

                if (aFirstIdx - aSecondIdx != 1 && (aSecondIdx != aLength || aFirstIdx != 1))
                {
                    isShiftReverse = false;
                }
            }

            aFirstIdx = aMainLoop.Value(aLength);
            aSecondIdx = aMainLoop.Value(1);
            if (aSecondIdx - aFirstIdx != 1 && (aFirstIdx != aLength || aSecondIdx != 1))
            {
                isShiftForward = false;
            }

            if (aFirstIdx - aSecondIdx != 1 && (aSecondIdx != aLength || aFirstIdx != 1))
            {
                isShiftReverse = false;
            }

            if (isShiftForward || isShiftReverse)
            {
                aTempStatus = 3;
            }

            myStat = aTempStatus;
            return;
        }
    }

    /// <summary>Tells if <see cref="Perform"/> has been done.</summary>
    public bool IsDone()
    {
        return myOrd is not null;
    }

    /// <summary>The status of <see cref="Perform"/> (see the remarks of the class).</summary>
    public int Status()
    {
        return myStat;
    }

    /// <summary>
    /// The index of the edge that comes at position <paramref name="theIdx"/> in the proposed order (negative if it
    /// is reversed); <paramref name="theIdx"/> itself before Perform.
    /// </summary>
    public int Ordered(int theIdx)
    {
        if (myOrd is null || myOrd.Upper() < theIdx)
        {
            return theIdx;
        }

        int anOldIdx = myOrd.Value(theIdx);
        return (anOldIdx == 0 ? theIdx : anOldIdx);
    }

    /// <summary>The 3D ends of the edge <paramref name="theIdx"/> (swapped for a negative index).</summary>
    public void XYZ(int theIdx, out gp_XYZ theStart3D, out gp_XYZ theEnd3D)
    {
        theStart3D = myXYZ.Value((theIdx > 0 ? 2 * theIdx - 1 : -2 * theIdx));
        theEnd3D = myXYZ.Value((theIdx > 0 ? 2 * theIdx : -2 * theIdx - 1));
    }

    /// <summary>The 2D ends of the edge <paramref name="theIdx"/> (swapped for a negative index).</summary>
    public void XY(int theIdx, out gp_XY theStart2D, out gp_XY theEnd2D)
    {
        if (myMode == ModeType.ModeBoth)
        {
            theStart2D = myXY.Value((theIdx > 0 ? 2 * theIdx - 1 : -2 * theIdx));
            theEnd2D = myXY.Value((theIdx > 0 ? 2 * theIdx : -2 * theIdx - 1));
        }
        else
        {
            gp_XYZ aStart3d = myXYZ.Value((theIdx > 0 ? 2 * theIdx - 1 : -2 * theIdx));
            theStart2D = new gp_XY();
            theStart2D.SetCoord(aStart3d.X(), aStart3d.Y());
            gp_XYZ anEnd3d = myXYZ.Value((theIdx > 0 ? 2 * theIdx : -2 * theIdx - 1));
            theEnd2D = new gp_XY();
            theEnd2D.SetCoord(anEnd3d.X(), anEnd3d.Y());
        }
    }

    /// <summary>
    /// The gap (sum of the absolute coordinate differences) between the end of the edge before position
    /// <paramref name="num"/> and the start of the edge at it, in the proposed order; for 0, the largest gap, which
    /// OCCT never computes (always 0).
    /// </summary>
    public double Gap(int num = 0)
    {
        if (num == 0)
        {
            return myGap;
        }

        int n1 = Ordered(num);
        int n0 = Ordered(num == 1 ? NbEdges() : num - 1);
        //  Distance entre fin (n0) et debut (n1)
        return DISTABS(myXYZ.Value((n0 > 0 ? 2 * n0 : -2 * n0 - 1)), myXYZ.Value((n1 > 0 ? 2 * n1 - 1 : -2 * n1)));
    }

    /// <summary>Determines the chains: sequences of ordered edges whose ends match within <paramref name="gap"/>.</summary>
    public void SetChains(double gap)
    {
        int n0, n1, n2, nb = NbEdges();
        if (nb == 0)
        {
            return;
        }

        var chain = new NCollection_Sequence<int>();
        n0 = 0;
        chain.Append(1); // On demarre la partie
        gp_XYZ f3d, l3d, f13d = new gp_XYZ(), l13d = new gp_XYZ();
        for (n1 = 1; n1 <= nb; n1++)
        {
            if (n0 == 0)
            { // nouvelle boucle
                n0 = n1;
                XYZ(Ordered(n0), out f13d, out l13d);
            }

            // OCCT: the last edge is also checked against the start of its own chain, so an open chain's start is appended twice
            n2 = (n1 == nb) ? n0 : (n1 + 1);
            XYZ(Ordered(n2), out f3d, out l3d);
            if (!f3d.IsEqual(l13d, gap))
            {
                chain.Append(n2);
                n0 = 0;
            }

            f13d = f3d;
            l13d = l3d;
        }

        nb = chain.Length();
        if (nb == 0)
        {
            return;
        }

        myChains = new NCollection_HArray1<int>(1, nb);
        for (n1 = 1; n1 <= nb; n1++)
        {
            myChains.SetValue(n1, chain.Value(n1));
        }
    }

    /// <summary>The count of chains computed by <see cref="SetChains"/>.</summary>
    public int NbChains()
    {
        return (myChains is null ? 0 : myChains.Length());
    }

    /// <summary>The first and last positions of chain <paramref name="num"/>; 0, 0 if there is none.</summary>
    public void Chain(int num, out int n1, out int n2)
    {
        n1 = n2 = 0;
        if (myChains is null)
        {
            return;
        }

        int nb = myChains.Upper();
        if (num == 0 || num > nb)
        {
            return;
        }

        n1 = myChains.Value(num);
        if (num == nb)
        {
            n2 = NbEdges();
        }
        else
        {
            n2 = myChains.Value(num + 1) - 1;
        }
    }

    /// <summary>Determines the couples of edges for which end and start fit inside <paramref name="gap"/>.</summary>
    /// <remarks>Not implemented in OCCT: it does nothing, and <see cref="NbCouples"/> stays 0.</remarks>
    public void SetCouples(double gap)
    {
        _ = gap; // OCCT: SetCouples is empty
    }

    /// <summary>The count of couples (always 0: <see cref="SetCouples"/> isn't implemented).</summary>
    public int NbCouples()
    {
        return (myCouples is null ? 0 : myCouples.Length());
    }

    /// <summary>The two edges of couple <paramref name="num"/>; 0, 0 if there is none.</summary>
    public void Couple(int num, out int n1, out int n2)
    {
        n1 = n2 = 0;
        if (myCouples is null)
        {
            return;
        }

        int nb = myCouples.Upper();
        if (num == 0 || num * 2 > nb)
        {
            return;
        }

        n1 = myCouples.Value(2 * num - 1);
        n2 = myCouples.Value(2 * num);
    }
}
