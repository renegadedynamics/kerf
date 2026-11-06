// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_WireOrder.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_WireOrder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// Proposes an order for a set of edges given by the coordinates of their ends (in 3D, in 2D, or both), so that they
/// chain with the smallest gaps, reversing some if needed. A negative index in <see cref="Ordered"/> means the edge is
/// reversed.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Status"/> after <see cref="Perform"/>: 0 the order is unchanged, 1 some
/// edges were reordered, -1 some were reversed, 3 the edges were only shifted (in either direction).
/// </remarks>
internal sealed class WireOrderAnalysis
{
    private enum ModeType
    {
        Mode2D,
        Mode3D,
        ModeBoth,
    }

    private NCollection_HArray1<int>? _ord;
    private NCollection_HArray1<int>? _chains;
    private NCollection_HArray1<int>? _couples = null;
    private NCollection_HSequence<Xyz> _xyz = null!;
    private NCollection_HSequence<Xy> _xy = null!;
    private double _tol;
    private double _gap;
    private int _stat;
    private bool _keepLoops;
    private ModeType _mode;

    /// <summary>An empty tool in 3D mode, with tolerance Precision::Confusion.</summary>
    public WireOrderAnalysis()
    {
        _gap = 0.0;
        _stat = 0;
        _keepLoops = false;
        _mode = ModeType.Mode3D;
        _tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        Clear();
    }

    /// <summary>
    /// An empty tool in 3D mode if <paramref name="mode3D"/>, else 2D; in both modes if
    /// <paramref name="modeBoth"/>.
    /// </summary>
    public WireOrderAnalysis(bool mode3D, double tolerance, bool modeBoth = false)
    {
        _tol = tolerance;
        _gap = 0.0;
        _stat = 0;
        _keepLoops = false;
        if (modeBoth)
        {
            _mode = ModeType.ModeBoth;
        }
        else
        {
            if (mode3D)
            {
                _mode = ModeType.Mode3D;
            }
            else
            {
                _mode = ModeType.Mode2D;
            }
        }

        Clear();
    }

    /// <summary>Sets the mode and the tolerance (1e-8 if not positive); clears the edges if the mode changes.</summary>
    public void SetMode(bool mode3D, double tolerance, bool modeBoth = false)
    {
        ModeType newMode;
        if (modeBoth)
        {
            newMode = ModeType.ModeBoth;
        }
        else
        {
            if (mode3D)
            {
                newMode = ModeType.Mode3D;
            }
            else
            {
                newMode = ModeType.Mode2D;
            }
        }

        if (_mode != newMode)
        {
            Clear();
        }

        _mode = newMode;
        _ord = null;
        _stat = 0;
        _gap = 0.0;
        _tol = (tolerance > 0.0) ? tolerance : 1.0e-08;
    }

    /// <summary>The working tolerance.</summary>
    public double Tolerance()
    {
        return _tol;
    }

    /// <summary>Clears the list of edges, but not the mode and tolerance.</summary>
    public void Clear()
    {
        _xyz = new NCollection_HSequence<Xyz>();
        _xy = new NCollection_HSequence<Xy>();
        _stat = 0;
        _gap = 0.0;
    }

    /// <summary>Adds an edge by the 3D points of its ends (3D mode only).</summary>
    public void Add(in Xyz start3d, in Xyz end3d)
    {
        if (_mode == ModeType.Mode3D)
        {
            _xyz.Append(start3d);
            _xyz.Append(end3d);
        }
    }

    /// <summary>Adds an edge by the 2D points of its ends (2D mode only).</summary>
    public void Add(in Xy start2d, in Xy end2d)
    {
        if (_mode == ModeType.Mode2D)
        {
            Xyz val = new Xyz();
            val.SetCoord(start2d.X(), start2d.Y(), 0.0);
            _xyz.Append(val);
            val.SetCoord(end2d.X(), end2d.Y(), 0.0);
            _xyz.Append(val);
        }
    }

    /// <summary>Adds an edge by the 3D and 2D points of its ends (both mode only).</summary>
    public void Add(in Xyz start3d, in Xyz end3d, in Xy start2d, in Xy end2d)
    {
        if (_mode == ModeType.ModeBoth)
        {
            _xyz.Append(start3d);
            _xyz.Append(end3d);

            _xy.Append(start2d);
            _xy.Append(end2d);
        }
    }

    /// <summary>The count of edges added.</summary>
    public int EdgeCount
    {
        get
        {
            return _xyz.Length() / 2;
        }
    }

    private static double DISTABS(in Xyz v1, in Xyz v2)
    {
        return Math.Abs(v1.X() - v2.X()) + Math.Abs(v1.Y() - v2.Y()) + Math.Abs(v1.Z() - v2.Z());
    }

    /// <summary>
    /// Whether the loops found by <see cref="Perform"/> are kept one after another (true) or merged into the main
    /// loop at their best place (false, the default).
    /// </summary>
    public ref bool KeepLoopsMode()
    {
        return ref _keepLoops;
    }

    /// <summary>
    /// Computes the order of the edges, taking the gaps between them into account. <paramref name="closed"/> is
    /// ignored, as in OCCT.
    /// </summary>
    public void Perform(bool closed = true)
    {
        _ = closed;
        _stat = 0;
        int nbEdges = EdgeCount;
        // no edges loaded, nothing to do -- return with status OK
        if (nbEdges == 0)
        {
            return;
        }

        _ord = new NCollection_HArray1<int>(1, nbEdges);
        _ord.Init(0);

        // sequence of the edge nums in the right order
        var edgeSeq = new NCollection_HSequence<int>();
        var loops = new NCollection_Sequence<NCollection_HSequence<int>>();

        // the beginnings and ends of the edges
        var begins3D = new NCollection_Array1<Xyz>(1, nbEdges);
        var ends3D = new NCollection_Array1<Xyz>(1, nbEdges);
        var begins2D = new NCollection_Array1<Xy>(1, nbEdges);
        var ends2D = new NCollection_Array1<Xy>(1, nbEdges);
        for (int i = 1; i <= nbEdges; i++)
        {
            begins3D[i] = _xyz.Value(2 * i - 1);
            ends3D[i] = _xyz.Value(2 * i);
            if (_mode == ModeType.ModeBoth)
            {
                begins2D[i] = _xy.Value(2 * i - 1);
                ends2D[i] = _xy.Value(2 * i);
            }
        }

        // the flags that the edges was considered
        var isEdgeUsed = new NCollection_Array1<bool>(1, nbEdges);
        isEdgeUsed.Init(false);

        double tol2 = global::Kerf.Engine.Foundation.Precision.SquareConfusion();
        double tolP2 = global::Kerf.Engine.Foundation.Precision.SquarePConfusion();

        // take the first edge to the constructed chain
        isEdgeUsed[1] = true;
        Point3D firstPnt3D = new Point3D(begins3D[1]);
        Point3D lastPnt3D = new Point3D(ends3D[1]);
        Point2D firstPnt2D = new Point2D();
        Point2D lastPnt2D = new Point2D();
        if (_mode == ModeType.ModeBoth)
        {
            firstPnt2D = new Point2D(begins2D[1]);
            lastPnt2D = new Point2D(ends2D[1]);
        }

        edgeSeq.Append(1);

        // cycle until all edges are considered
        for (;;)
        {
            // joint type
            // 0 - the start of the best edge to the end of constructed sequence (nothing to do)
            // 1 - the end of the best edge to the start of constructed sequence (need move the edge)
            // 2 - the end of the best edge to the end of constructed sequence (need to reverse)
            // 3 - the start of the best edge to the start of constructed sequence (need to reverse and move the edge)
            int bestJointType = 3;
            // the best minimum distance between constructed sequence and the best edge
            double bestMin3D = RealLast();
            // number of the best edge
            int bestEdgeNum = 0;
            // the best edge was found
            bool isFound = false;
            bool isConnected = false;
            // loop to find the best edge among all the remaining
            for (int i = 1; i <= nbEdges; i++)
            {
                if (isEdgeUsed[i])
                {
                    continue;
                }

                // find minimum distance and joint type for 3D and 2D (if necessary) modes
                int curJointType;
                double curMin;
                // distance for four possible cases
                double seqTailEdgeHead = lastPnt3D.SquareDistanceTo(new Point3D(begins3D[i]));
                double seqTailEdgeTail = lastPnt3D.SquareDistanceTo(new Point3D(ends3D[i]));
                double seqHeadEdgeTail = firstPnt3D.SquareDistanceTo(new Point3D(ends3D[i]));
                double seqHeadEdgeHead = firstPnt3D.SquareDistanceTo(new Point3D(begins3D[i]));
                // the best distances for joints with head and tail of sequence
                double minDistToTail, minDistToHead;
                int tailJoinType, headJointType;
                if (seqTailEdgeHead <= seqTailEdgeTail)
                {
                    tailJoinType = 0;
                    minDistToTail = seqTailEdgeHead;
                }
                else
                {
                    tailJoinType = 2;
                    minDistToTail = seqTailEdgeTail;
                }

                if (seqHeadEdgeTail <= seqHeadEdgeHead)
                {
                    headJointType = 1;
                    minDistToHead = seqHeadEdgeTail;
                }
                else
                {
                    headJointType = 3;
                    minDistToHead = seqHeadEdgeHead;
                }

                // comparing the head and the tail cases
                // if distances are close enough then we use rule for joint type: 0 < 1 < 2 < 3
                if (Math.Abs(minDistToTail - minDistToHead) < tol2)
                {
                    if (tailJoinType < headJointType)
                    {
                        curJointType = tailJoinType;
                        curMin = minDistToTail;
                    }
                    else
                    {
                        curJointType = headJointType;
                        curMin = minDistToHead;
                    }
                }
                else
                {
                    if (minDistToTail <= minDistToHead)
                    {
                        curJointType = tailJoinType;
                        curMin = minDistToTail;
                    }
                    else
                    {
                        curJointType = headJointType;
                        curMin = minDistToHead;
                    }
                }

                // update for the best values
                if (_mode == ModeType.ModeBoth)
                {
                    // distances in 2D
                    int jointMask3D = 0, jointMask2D = 0;
                    if (seqTailEdgeHead < tol2)
                    {
                        jointMask3D |= (1 << 0);
                    }

                    if (seqTailEdgeTail < tol2)
                    {
                        jointMask3D |= (1 << 2);
                    }

                    if (seqHeadEdgeTail < tol2)
                    {
                        jointMask3D |= (1 << 1);
                    }

                    if (seqHeadEdgeHead < tol2)
                    {
                        jointMask3D |= (1 << 3);
                    }

                    double seqTailEdgeHead2D = lastPnt2D.SquareDistanceTo(new Point2D(begins2D[i]));
                    double seqTailEdgeTail2D = lastPnt2D.SquareDistanceTo(new Point2D(ends2D[i]));
                    double seqHeadEdgeTail2D = firstPnt2D.SquareDistanceTo(new Point2D(ends2D[i]));
                    double seqHeadEdgeHead2D = firstPnt2D.SquareDistanceTo(new Point2D(begins2D[i]));
                    if (seqTailEdgeHead2D < tolP2)
                    {
                        jointMask2D |= (1 << 0);
                    }

                    if (seqTailEdgeTail2D < tolP2)
                    {
                        jointMask2D |= (1 << 2);
                    }

                    if (seqHeadEdgeTail2D < tolP2)
                    {
                        jointMask2D |= (1 << 1);
                    }

                    if (seqHeadEdgeHead2D < tolP2)
                    {
                        jointMask2D |= (1 << 3);
                    }

                    // new approch for detecting best edge connection, for all other cases used old 3D algorithm
                    int fullMask = jointMask3D & jointMask2D;
                    if (fullMask != 0)
                    {
                        // find the best current joint type
                        curJointType = 3;
                        for (int j = 0; j < 4; j++)
                        {
                            if ((fullMask & (1 << j)) != 0)
                            {
                                curJointType = j;
                                break;
                            }
                        }

                        if (!isConnected || curJointType < bestJointType)
                        {
                            isFound = true;
                            isConnected = true;
                            switch (curJointType)
                            {
                                case 0:
                                    bestMin3D = seqTailEdgeHead;
                                    break;
                                case 1:
                                    bestMin3D = seqHeadEdgeTail;
                                    break;
                                case 2:
                                    bestMin3D = seqTailEdgeTail;
                                    break;
                                case 3:
                                    bestMin3D = seqHeadEdgeHead;
                                    break;
                            }

                            bestJointType = curJointType;
                            bestEdgeNum = i;
                        }
                    }

                    // if there is still no connection, continue to use ald 3D algorithm
                    if (isConnected)
                    {
                        continue;
                    }
                }

                // if the best distance is still not reached (aBestMin3D > aTol2) or we found a better joint type
                if (bestMin3D > tol2 || curJointType < bestJointType)
                {
                    // make a decision that this edge is good enough:
                    // - it gets the best distance but there is fabs(aCurMin3d - aBestMin3d) < aTol2 &&
                    // (aCurJointType < aBestJointType) ?
                    // - it gets the best joint in some cases
                    if (curMin < bestMin3D || ((curMin == bestMin3D || curMin < tol2) && (curJointType < bestJointType)))
                    {
                        isFound = true;
                        bestMin3D = curMin;
                        bestJointType = curJointType;
                        bestEdgeNum = i;
                    }
                }
            }

            // check that we found edge for connecting
            if (isFound)
            {
                // distance between first and last point in sequence
                double closeDist = firstPnt3D.SquareDistanceTo(lastPnt3D);
                // if it's better to insert the edge than to close the loop, just insert the edge according to joint type
                if (bestMin3D <= RealSmall() || bestMin3D < closeDist)
                {
                    switch (bestJointType)
                    {
                        case 0:
                            edgeSeq.Append(bestEdgeNum);
                            lastPnt3D = new Point3D(ends3D[bestEdgeNum]);
                            break;
                        case 1:
                            edgeSeq.Prepend(bestEdgeNum);
                            firstPnt3D = new Point3D(begins3D[bestEdgeNum]);
                            break;
                        case 2:
                            edgeSeq.Append(-bestEdgeNum);
                            lastPnt3D = new Point3D(begins3D[bestEdgeNum]);
                            break;
                        case 3:
                            edgeSeq.Prepend(-bestEdgeNum);
                            firstPnt3D = new Point3D(ends3D[bestEdgeNum]);
                            break;
                    }

                    if (_mode == ModeType.ModeBoth)
                    {
                        switch (bestJointType)
                        {
                            case 0:
                                lastPnt2D = new Point2D(ends2D[bestEdgeNum]);
                                break;
                            case 1:
                                firstPnt2D = new Point2D(begins2D[bestEdgeNum]);
                                break;
                            case 2:
                                lastPnt2D = new Point2D(begins2D[bestEdgeNum]);
                                break;
                            case 3:
                                firstPnt2D = new Point2D(ends2D[bestEdgeNum]);
                                break;
                        }
                    }
                }
                // closing loop and creating new one
                else
                {
                    loops.Append(edgeSeq);
                    edgeSeq = new NCollection_HSequence<int>();
                    firstPnt3D = new Point3D(begins3D[bestEdgeNum]);
                    lastPnt3D = new Point3D(ends3D[bestEdgeNum]);
                    if (_mode == ModeType.ModeBoth)
                    {
                        firstPnt2D = new Point2D(begins2D[bestEdgeNum]);
                        lastPnt2D = new Point2D(ends2D[bestEdgeNum]);
                    }

                    edgeSeq.Append(bestEdgeNum);
                }

                // mark the edge as used
                isEdgeUsed[bestEdgeNum] = true;
            }
            else
            {
                // the only condition under which we can't find an edge is when all edges are done
                break;
            }
        }

        // append the last loop
        loops.Append(edgeSeq);

        // handling with constructed loops
        NCollection_HSequence<int> mainLoop;
        if (_keepLoops)
        {
            // keeping the loops, adding one after another.
            mainLoop = new NCollection_HSequence<int>();
            for (int i = 1; i <= loops.Length(); i++)
            {
                NCollection_HSequence<int> curLoop = loops[i];
                mainLoop.Append(curLoop);
            }
        }
        else
        {
            // connecting loops
            mainLoop = loops.First();
            loops.Remove(1);
            while (loops.Length() != 0)
            {
                // iterate over all loops to find the closest one
                double minDist1 = RealLast();
                int loopNum1 = 0;
                int curLoopIt1 = 0;
                bool direct1 = false;
                int mainLoopIt1 = 0;
                for (int loopIt = 1; loopIt <= loops.Length(); loopIt++)
                {
                    NCollection_HSequence<int> curLoop = loops.Value(loopIt);
                    // iterate over all gaps between edges in current loop
                    int curLoopIt2 = 0;
                    int mainLoopIt2 = 0;
                    bool direct2 = false;
                    double minDist2 = RealLast();
                    int curLoopLength = curLoop.Length();
                    for (int curEdgeIt = 1; curEdgeIt <= curLoopLength; curEdgeIt++)
                    {
                        // get the distance between the current edge and the previous edge taking into account the
                        // edge's orientation
                        int prevEdgeIt = curEdgeIt == 1 ? curLoopLength : curEdgeIt - 1;
                        int curEdgeIdx = curLoop.Value(curEdgeIt);
                        int prevEdgeIdx = curLoop.Value(prevEdgeIt);
                        Point3D curLoopFirst = new Point3D(curEdgeIdx > 0 ? begins3D[curEdgeIdx] : ends3D[-curEdgeIdx]);
                        Point3D curLoopLast = new Point3D(prevEdgeIdx > 0 ? ends3D[prevEdgeIdx] : begins3D[-prevEdgeIdx]);
                        // iterate over all gaps between edges in main loop
                        double minDist3 = RealLast();
                        int mainLoopIt3 = 0;
                        bool direct3 = false;
                        int mainLoopLength = mainLoop.Length();
                        for (int curEdgeIt2 = 1; (curEdgeIt2 <= mainLoopLength) && minDist3 != 0.0; curEdgeIt2++)
                        {
                            // get the distance between the current edge and the next edge taking into account the
                            // edge's orientation
                            int nextEdgeIt2 = curEdgeIt2 == mainLoopLength ? 1 : curEdgeIt2 + 1;
                            int curEdgeIdx2 = mainLoop.Value(curEdgeIt2);
                            int nextEdgeIdx2 = mainLoop.Value(nextEdgeIt2);
                            Point3D mainLoopFirst = new Point3D(curEdgeIdx2 > 0 ? ends3D[curEdgeIdx2] : begins3D[-curEdgeIdx2]);
                            Point3D mainLoopLast = new Point3D(nextEdgeIdx2 > 0 ? begins3D[nextEdgeIdx2] : ends3D[-nextEdgeIdx2]);
                            // getting the sum of square distances if we try to sew the current loop with the main loop
                            // in current positions
                            double directDist = curLoopFirst.SquareDistanceTo(mainLoopFirst) + curLoopLast.SquareDistanceTo(mainLoopLast);
                            double reverseDist = curLoopFirst.SquareDistanceTo(mainLoopLast) + curLoopLast.SquareDistanceTo(mainLoopFirst);
                            // take the best result
                            double joinDist;
                            if ((directDist < tol2) || (directDist < 2.0 * reverseDist))
                            {
                                joinDist = directDist;
                                reverseDist = directDist;
                            }
                            else
                            {
                                joinDist = reverseDist;
                            }

                            // check if we found a better distance
                            if (joinDist < minDist3 && Math.Abs(minDist3 - joinDist) > tol2)
                            {
                                minDist3 = joinDist;
                                direct3 = (directDist <= reverseDist);
                                mainLoopIt3 = curEdgeIt2;
                            }
                        }

                        // check if we found a better distance
                        if (minDist3 < minDist2 && Math.Abs(minDist2 - minDist3) > tol2)
                        {
                            minDist2 = minDist3;
                            direct2 = direct3;
                            mainLoopIt2 = mainLoopIt3;
                            curLoopIt2 = curEdgeIt;
                        }
                    }

                    // check if we found a better distance
                    if (minDist2 < minDist1 && Math.Abs(minDist1 - minDist2) > tol2)
                    {
                        minDist1 = minDist2;
                        loopNum1 = loopIt;
                        direct1 = direct2;
                        mainLoopIt1 = mainLoopIt2;
                        curLoopIt1 = curLoopIt2;
                    }
                }

                // insert the found loop into main loop
                NCollection_HSequence<int> loop = loops.Value(loopNum1);
                int factor = (direct1 ? 1 : -1);
                for (int i = 0; i < loop.Length(); i++)
                {
                    int idx = (curLoopIt1 + i > loop.Length() ? curLoopIt1 + i - loop.Length() : curLoopIt1 + i);
                    mainLoop.InsertAfter(mainLoopIt1 + i, loop.Value(idx) * factor);
                }

                loops.Remove(loopNum1);
            }
        }

        // checking the new order of the edges
        //  0 - order is the same
        //  1 - some edges were reordered
        // -1 - some edges were reversed
        int tempStatus = 0;
        for (int i = 1; i <= mainLoop.Length(); i++)
        {
            if (i != mainLoop.Value(i) && tempStatus >= 0)
            {
                tempStatus = (mainLoop.Value(i) > 0 ? 1 : -1);
            }

            _ord.SetValue(i, mainLoop.Value(i));
        }

        if (tempStatus == 0)
        {
            _stat = tempStatus;
            return;
        }
        else
        {
            // check if edges were only shifted in reverse or forward, not reordered
            bool isShiftReverse = true;
            bool isShiftForward = true;
            int firstIdx, secondIdx;
            int length = mainLoop.Length();
            for (int i = 1; i <= length - 1; i++)
            {
                firstIdx = mainLoop.Value(i);
                secondIdx = mainLoop.Value(i + 1);
                if (secondIdx - firstIdx != 1 && (firstIdx != length || secondIdx != 1))
                {
                    isShiftForward = false;
                }

                if (firstIdx - secondIdx != 1 && (secondIdx != length || firstIdx != 1))
                {
                    isShiftReverse = false;
                }
            }

            firstIdx = mainLoop.Value(length);
            secondIdx = mainLoop.Value(1);
            if (secondIdx - firstIdx != 1 && (firstIdx != length || secondIdx != 1))
            {
                isShiftForward = false;
            }

            if (firstIdx - secondIdx != 1 && (secondIdx != length || firstIdx != 1))
            {
                isShiftReverse = false;
            }

            if (isShiftForward || isShiftReverse)
            {
                tempStatus = 3;
            }

            _stat = tempStatus;
            return;
        }
    }

    /// <summary>Tells if <see cref="Perform"/> has been done.</summary>
    public bool IsDone
    {
        get
        {
            return _ord is not null;
        }
    }

    /// <summary>The status of <see cref="Perform"/> (see the remarks of the class).</summary>
    public int Status()
    {
        return _stat;
    }

    /// <summary>
    /// The index of the edge that comes at position <paramref name="idx"/> in the proposed order (negative if it
    /// is reversed); <paramref name="idx"/> itself before Perform.
    /// </summary>
    public int Ordered(int idx)
    {
        if (_ord is null || _ord.Upper() < idx)
        {
            return idx;
        }

        int oldIdx = _ord.Value(idx);
        return (oldIdx == 0 ? idx : oldIdx);
    }

    /// <summary>The 3D ends of the edge <paramref name="idx"/> (swapped for a negative index).</summary>
    public void XYZ(int idx, out Xyz start3D, out Xyz end3D)
    {
        start3D = _xyz.Value((idx > 0 ? 2 * idx - 1 : -2 * idx));
        end3D = _xyz.Value((idx > 0 ? 2 * idx : -2 * idx - 1));
    }

    /// <summary>The 2D ends of the edge <paramref name="idx"/> (swapped for a negative index).</summary>
    public void XY(int idx, out Xy start2D, out Xy end2D)
    {
        if (_mode == ModeType.ModeBoth)
        {
            start2D = _xy.Value((idx > 0 ? 2 * idx - 1 : -2 * idx));
            end2D = _xy.Value((idx > 0 ? 2 * idx : -2 * idx - 1));
        }
        else
        {
            Xyz start3d = _xyz.Value((idx > 0 ? 2 * idx - 1 : -2 * idx));
            start2D = new Xy();
            start2D.SetCoord(start3d.X(), start3d.Y());
            Xyz end3d = _xyz.Value((idx > 0 ? 2 * idx : -2 * idx - 1));
            end2D = new Xy();
            end2D.SetCoord(end3d.X(), end3d.Y());
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
            return _gap;
        }

        int n1 = Ordered(num);
        int n0 = Ordered(num == 1 ? EdgeCount : num - 1);
        //  Distance entre fin (n0) et debut (n1)
        return DISTABS(_xyz.Value((n0 > 0 ? 2 * n0 : -2 * n0 - 1)), _xyz.Value((n1 > 0 ? 2 * n1 - 1 : -2 * n1)));
    }

    /// <summary>Determines the chains: sequences of ordered edges whose ends match within <paramref name="gap"/>.</summary>
    public void SetChains(double gap)
    {
        int n0, n1, n2, nb = EdgeCount;
        if (nb == 0)
        {
            return;
        }

        var chain = new NCollection_Sequence<int>();
        n0 = 0;
        chain.Append(1); // On demarre la partie
        Xyz f3d, l3d, f13d = new Xyz(), l13d = new Xyz();
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

        _chains = new NCollection_HArray1<int>(1, nb);
        for (n1 = 1; n1 <= nb; n1++)
        {
            _chains.SetValue(n1, chain.Value(n1));
        }
    }

    /// <summary>The count of chains computed by <see cref="SetChains"/>.</summary>
    public int ChainCount
    {
        get
        {
            return (_chains is null ? 0 : _chains.Length());
        }
    }

    /// <summary>The first and last positions of chain <paramref name="num"/>; 0, 0 if there is none.</summary>
    public void Chain(int num, out int n1, out int n2)
    {
        n1 = n2 = 0;
        if (_chains is null)
        {
            return;
        }

        int nb = _chains.Upper();
        if (num == 0 || num > nb)
        {
            return;
        }

        n1 = _chains.Value(num);
        if (num == nb)
        {
            n2 = EdgeCount;
        }
        else
        {
            n2 = _chains.Value(num + 1) - 1;
        }
    }

    /// <summary>Determines the couples of edges for which end and start fit inside <paramref name="gap"/>.</summary>
    /// <remarks>Not implemented in OCCT: it does nothing, and <see cref="CoupleCount"/> stays 0.</remarks>
    public void SetCouples(double gap)
    {
        _ = gap; // OCCT: SetCouples is empty
    }

    /// <summary>The count of couples (always 0: <see cref="SetCouples"/> isn't implemented).</summary>
    public int CoupleCount
    {
        get
        {
            return (_couples is null ? 0 : _couples.Length());
        }
    }

    /// <summary>The two edges of couple <paramref name="num"/>; 0, 0 if there is none.</summary>
    public void Couple(int num, out int n1, out int n2)
    {
        n1 = n2 = 0;
        if (_couples is null)
        {
            return;
        }

        int nb = _couples.Upper();
        if (num == 0 || num * 2 > nb)
        {
            return;
        }

        n1 = _couples.Value(2 * num - 1);
        n2 = _couples.Value(2 * num);
    }
}
