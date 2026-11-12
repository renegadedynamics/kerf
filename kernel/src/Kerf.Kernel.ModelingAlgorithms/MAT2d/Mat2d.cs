// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Mat2d.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Mat2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>The generic algorithm of the bisecting locus: builds the trees of MAT_Bisector from the items of a
/// MAT2d_Tool2d.</summary>
/// <remarks>
/// <para>The map is built from the lowest branches (bisectors of neighbouring elements of the contour) up: (0) the
/// bisectors of neighbouring elements; (1) some elements are removed, giving a new contour; (2) back to 0. Each pass
/// intersects each bisector with its left and right neighbours, cancels the bisectors whose intersections are
/// wrong (reinserting their edges), and removes the edges whose two bisectors meet before meeting their other
/// neighbours. The roots of the trees are the bisectors of the last pass.</para>
/// <para>A working object (decision 21). OCCT's destructor clears the edges of the bisectors and the bisectors of
/// the edges to break reference cycles; the collector needs no help, so it is left out (the graph is built from the
/// roots before the algorithm goes away, and nothing reads those links afterwards). The maps are keyed by integers
/// and only looked up. OCCT leaves the intersection point index uninitialised until the first intersection found;
/// it is 0 in the port.</para>
/// </remarks>
internal sealed class Mat2d
{
    private readonly bool _isOpenResult;
    private int _thenumberofbisectors;
    private bool _semiInfinite;
    private MedialListOfEdge? _theedgelist;
    private MedialListOfEdge? _removedEdgesList;
    private readonly Dictionary<int, int> _typeofbisectortoremove = new();
    private readonly Dictionary<int, MedialBisector> _bisectoronetoremove = new();
    private readonly Dictionary<int, MedialBisector> _bisectortwotoremove = new();
    private readonly Dictionary<int, MedialBisector> _bisectormap = new();
    private MedialListOfBisector? _roots;
    private bool _isDone;

    // Kerf (decision 32501): once the map has gone round a node in a circle (MatCycle has seen a state come back, or
    // one of OCCT's two patches against loops was about to act, and a bisector had just reached a new one where it
    // starts), the bisectors whose two ends are one node as far as the intersections can tell, by index; null until
    // then, and the map runs as OCCT's
    private HashSet<int>? _oneNode;

    // Kerf: the main loop's pass, and the last pass in which a bisector reached a new one where it starts and met the
    // child that ended there
    private int _pass;
    private int _reachedNodeAtPass;

    public Mat2d(bool isOpenResult = false)
    {
        _semiInfinite = false;
        _isDone = false;
        _isOpenResult = isOpenResult;
        _thenumberofbisectors = 0;
        // OCCT's thenumberofedges is set here and never read: left out.
    }

    /// <summary>The bisecting locus of an open wire.</summary>
    public void CreateMatOpen(Tool2d atool)
    {
        bool interrupt = false;
        MedialEdge edgetoremove;

        int noofbisectorstoremove;
        MedialBisector firstbisector, secondbisector;
        MedialEdge edge;
        int intersectionpoint = 0;
        int beginbisector;
        int noofbisectors;

        int nbIterBis = 0;
        int evenNbIterBis = 10;
        NCollection_Array1<int> edgeNumbers = new(1, evenNbIterBis + 1);
        edgeNumbers.Init(-1);
        bool toNullifyNoofbisectorstoremove = false;

        MedialBisector currentbisector;

        int i, k, narea, shift, compact, all;
        int noofedges;
        int numberMaxOfIte;
        double toleranceofconfusion;

        noofedges = atool.NumberOfItems();
        toleranceofconfusion = atool.ToleranceOfConfusion();
        numberMaxOfIte = noofedges * noofedges;

        NCollection_Array1<int> firstarea = new(0, noofedges);
        NCollection_Array1<int> lastarea = new(0, noofedges);
        NCollection_Array1<int> noofarea = new(0, noofedges);

        Span<int> parama = stackalloc int[2];
        Span<int> paramb = stackalloc int[2];

        // Initialisation of the maps.
        _bisectoronetoremove.Clear();
        _bisectortwotoremove.Clear();
        _typeofbisectortoremove.Clear();
        _bisectormap.Clear();
        _oneNode = null;
        _pass = 0;
        _reachedNodeAtPass = int.MinValue / 2;

        _isDone = true;
        noofbisectors = noofedges - 1;
        beginbisector = 0;

        // <theedgelist>: an edge per simple element of the contour.
        _theedgelist = new MedialListOfEdge();
        _removedEdgesList = new MedialListOfEdge();

        for (i = 0; i < noofedges; i++)
        {
            edge = new MedialEdge();
            edge.EdgeNumber(i + 1);
            edge.Distance(-1);
            _theedgelist.BackAdd(edge);
        }

        _theedgelist.Loop();

        // The bisectors from the contour.
        double dist;
        _theedgelist.First();

        for (i = 0; i < _theedgelist.Number() - 1; i++)
        {
            _bisectormap[i] = new MedialBisector();
            _bisectormap.GetValue(i).IndexNumber(i);
            _bisectormap.GetValue(i).FirstEdge(_theedgelist.Current());
            _bisectormap.GetValue(i).FirstVector(atool.TangentBefore(_theedgelist.Current().EdgeNumber(), _isOpenResult));
            _theedgelist.Next();
            _bisectormap.GetValue(i).SecondEdge(_theedgelist.Current());
            _bisectormap.GetValue(i).IssuePoint(atool.FirstPoint(_theedgelist.Current().EdgeNumber(), out dist));
            _bisectormap.GetValue(i).DistIssuePoint(dist);
            _bisectormap.GetValue(i).SecondVector(atool.TangentAfter(_theedgelist.Current().EdgeNumber(), _isOpenResult));
        }

        // The two bisectors of each edge.
        _theedgelist.First();
        _theedgelist.Current().FirstBisector(_bisectormap.GetValue(0));
        _theedgelist.Current().SecondBisector(_bisectormap.GetValue(0));
        _theedgelist.Next();

        for (i = 1; i < _theedgelist.Number() - 1; i++)
        {
            _theedgelist.Current().FirstBisector(_bisectormap.GetValue(i - 1));
            _theedgelist.Current().SecondBisector(_bisectormap.GetValue(i));
            _theedgelist.Next();
        }

        _theedgelist.Current().FirstBisector(_bisectormap.GetValue(_theedgelist.Number() - 2));
        _theedgelist.Current().SecondBisector(_bisectormap.GetValue(_theedgelist.Number() - 2));

        //===========================================================================
        //                         Main loop   (step 2)
        //===========================================================================
        int numberOfIte = 0;
        MatCycle? cycle = new();
        Span<int> loopState = stackalloc int[15];

        while (_theedgelist.Number() > 1)
        {
            _pass = numberOfIte;

            // The geometries of the bisectors, through the tool. (step 2.1)
            for (i = beginbisector; i < noofbisectors; i++)
            {
                atool.CreateBisector(_bisectormap.GetValue(i));
                _thenumberofbisectors++;
            }

            // Exit condition of the main loop.
            if (_theedgelist.Number() < 3)
            {
                break;
            }

            // loop 2: while there are bisectors to remove.
            for (;;)
            {
                nbIterBis++;

                noofbisectorstoremove = 0;
                _theedgelist.First();
                _theedgelist.Next();

                // The intersections of neighbouring bisectors. (step 2.2)
                if (nbIterBis <= evenNbIterBis + 1)
                {
                    edgeNumbers[nbIterBis] = _theedgelist.Number();
                }
                else
                {
                    for (k = 1; k <= evenNbIterBis; k++)
                    {
                        edgeNumbers[k] = edgeNumbers[k + 1];
                    }
                    edgeNumbers[evenNbIterBis + 1] = _theedgelist.Number();
                }
                if (edgeNumbers[evenNbIterBis + 1] == edgeNumbers[1])
                {
                    // Kerf: before OCCT's patch stops the cancelling for good, the node the loop goes round, if it
                    // goes round one, is taken as one, and the patch counts again from here (decision 32501)
                    if (!toNullifyNoofbisectorstoremove && TakesNodesAsOne(1))
                    {
                        cycle = new MatCycle();
                        edgeNumbers.Init(-1);
                        nbIterBis = 1;
                        edgeNumbers[1] = _theedgelist.Number();
                    }
                    else
                    {
                        toNullifyNoofbisectorstoremove = true;
                    }
                }

                for (i = 1; i < _theedgelist.Number() - 1; i++)
                {
                    edge = _theedgelist.Current();
                    if (edge.Distance() == -1.0)
                    {
                        firstbisector = edge.FirstBisector();
                        secondbisector = edge.SecondBisector();
                        edge.Distance(atool.IntersectBisector(firstbisector, secondbisector, ref intersectionpoint));
                        edge.IntersectionPoint(intersectionpoint);

                        if (edge.Distance() == global::Kerf.Engine.Foundation.Precision.Infinite())
                        {
                            if (firstbisector.IndexNumber() >= beginbisector
                                || secondbisector.IndexNumber() >= beginbisector)
                            {
                                Intersect(atool, 0, ref noofbisectorstoremove, firstbisector, secondbisector);
                            }
                        }
                        else
                        {
                            if (firstbisector.IndexNumber() >= beginbisector)
                            {
                                Intersect(atool, 1, ref noofbisectorstoremove, firstbisector, secondbisector);
                            }
                            if (secondbisector.IndexNumber() >= beginbisector)
                            {
                                Intersect(atool, 2, ref noofbisectorstoremove, firstbisector, secondbisector);
                            }
                        }
                    }
                    _theedgelist.Next();
                }

                // Exit test of loop 2.
                if (toNullifyNoofbisectorstoremove)
                {
                    noofbisectorstoremove = 0;
                }
                if (noofbisectorstoremove == 0)
                {
                    break;
                }

                // Cancelling the bisectors to remove. (step 2.4)
                CancelBisectors(atool, noofbisectorstoremove);
            }

            // The parameters of the intersections on the bisectors of each edge, and the portions of the contour
            // to remove. (step 2.5)
            _theedgelist.First();
            _theedgelist.Next();

            currentbisector = _theedgelist.Current().FirstBisector();
            ParamSigns(atool, currentbisector, toleranceofconfusion, _oneNode, out parama[0], out paramb[0]);

            narea = -1;

            for (i = 1; i < _theedgelist.Number() - 1; i++)
            {
                currentbisector = _theedgelist.Current().SecondBisector();
                ParamSigns(atool, currentbisector, toleranceofconfusion, _oneNode, out parama[1], out paramb[1]);

                // Is the edge to remove from the contour? The portions of the contour to remove: narea is their
                // number, firstarea[i] and lastarea[i] the indices of the first and last edges of portion i.
                if (paramb[0] > 0 && parama[1] > 0)
                {
                    AddToArea(ref narea, firstarea, lastarea, noofarea);
                }

                parama[0] = parama[1];
                paramb[0] = paramb[1];
                _theedgelist.Next();
            }

            compact = 0;
            if (narea > 0)
            {
                if (lastarea[narea] == _theedgelist.Number() && firstarea[0] == 1)
                {
                    firstarea[0] = firstarea[narea];
                    noofarea[0] = noofarea[0] + noofarea[narea];
                    compact = noofarea[narea];
                    narea--;
                }
            }

            narea++;

            // Exit of the main loop if no edge is to be removed. (step 2.6)
            if (narea == 0)
            {
                interrupt = true;
                break;
            }

            // Removal of the edges from the contour: the new contour, and the bisectors of the new neighbouring
            // edges.
            beginbisector = noofbisectors;
            shift = 0;
            all = 0;
            if (narea == 1 && noofarea[0] == _theedgelist.Number())
            {
                all = 1;
            }

            for (i = 0; i < narea; i++)
            {
                if (i == 1)
                {
                    shift = shift - compact;
                }
                _theedgelist.First();
                _theedgelist.Next();
                edgetoremove = _theedgelist.Brackets(firstarea[i] - shift);

                RemoveArea(atool, ref edgetoremove, ref noofbisectors, ref shift, all, noofarea[i]);
            }

            // Kerf: a pass that ends in the state an earlier pass ended in is followed by the same passes until the
            // count below runs out, so the count moves on to the pass that leaves the map as the last would (MatCycle)
            if (cycle is not null && numberOfIte <= numberMaxOfIte)
            {
                loopState[0] = Math.Min(nbIterBis, evenNbIterBis + 2);
                loopState[1] = toNullifyNoofbisectorstoremove ? 1 : 0;
                loopState[2] = noofbisectors - beginbisector;
                for (k = 1; k <= evenNbIterBis + 1; k++)
                {
                    loopState[2 + k] = edgeNumbers[k];
                }
                loopState[14] = _oneNode?.Count ?? -1;
                int period = cycle.Step(numberOfIte, _theedgelist, atool, beginbisector, loopState, intersectionpoint);
                if (period > 0 && TakesNodesAsOne(period))
                {
                    // The first time round a node, it is taken as one (Intersect, decision 32501), and the search for
                    // a repeat starts afresh
                    cycle = new MatCycle();
                }
                else if (period > 0)
                {
                    numberOfIte = numberMaxOfIte + 1 - (numberMaxOfIte + 1 - numberOfIte) % period;
                    cycle = null;
                }
            }

            // The number of iterations: each removes an element of the contour that is never reinserted, so it
            // must stay below the number of elements; the maximum is numberofedges * numberofedges.
            if (numberOfIte > numberMaxOfIte)
            {
                _isDone = false; // Failure of the map.
                break;
            }
            numberOfIte++;
        }

        // step 3. interrupt: semi-infinite bisectors.
        if (interrupt)
        {
            _semiInfinite = true;
        }
        else
        {
            _semiInfinite = false;
            LastTwoEdges(atool, noofbisectors);
        }

        if (_semiInfinite)
        {
            beginbisector = noofbisectors;
            _theedgelist.First();
            for (i = 1; i < _theedgelist.Number(); i++)
            {
                edge = _theedgelist.Current();
                _bisectormap[noofbisectors] = edge.SecondBisector();
                noofbisectors++;
                _theedgelist.Next();
            }
        }

        // The roots.
        Roots(beginbisector, noofbisectors);
    }

    /// <summary>The bisecting locus.</summary>
    public void CreateMat(Tool2d atool)
    {
        bool interrupt = false;
        MedialEdge edgetoremove;

        int noofbisectorstoremove;
        MedialBisector firstbisector, secondbisector;
        MedialEdge edge;
        int intersectionpoint = 0;
        int beginbisector;
        int noofbisectors;

        int nbIterBis = 0;
        int evenNbIterBis = 10;
        NCollection_Array1<int> edgeNumbers = new(1, evenNbIterBis + 1);
        edgeNumbers.Init(-1);
        bool toNullifyNoofbisectorstoremove = false;

        MedialBisector currentbisector;

        int i, k, narea, shift, compact, all;
        int noofedges;
        int numberMaxOfIte;
        double toleranceofconfusion;

        noofedges = atool.NumberOfItems();
        toleranceofconfusion = atool.ToleranceOfConfusion();
        numberMaxOfIte = noofedges * noofedges;

        NCollection_Array1<int> firstarea = new(0, noofedges);
        NCollection_Array1<int> lastarea = new(0, noofedges);
        NCollection_Array1<int> noofarea = new(0, noofedges);

        Span<int> parama = stackalloc int[2];
        Span<int> paramb = stackalloc int[2];

        int nbOfNarea1 = 0, prefNarea = 0, nbMaxNarea1 = 10;
        Span<int> nbElts = stackalloc int[2] { 0, 0 };
        Span<int> countElts = stackalloc int[2] { 0, 0 };
        bool isBreak = false;

        // Initialisation of the maps.
        _bisectoronetoremove.Clear();
        _bisectortwotoremove.Clear();
        _typeofbisectortoremove.Clear();
        _bisectormap.Clear();
        _oneNode = null;
        _pass = 0;
        _reachedNodeAtPass = int.MinValue / 2;

        _isDone = true;
        noofbisectors = noofedges;
        beginbisector = 0;

        // <theedgelist>: an edge per simple element of the contour.
        _theedgelist = new MedialListOfEdge();
        _removedEdgesList = new MedialListOfEdge();

        for (i = 0; i < noofedges; i++)
        {
            edge = new MedialEdge();
            edge.EdgeNumber(i + 1);
            edge.Distance(-1);
            _theedgelist.BackAdd(edge);
        }

        _theedgelist.Loop();

        // The bisectors from the contour.
        double dist;
        _theedgelist.First();

        for (i = 0; i < _theedgelist.Number(); i++)
        {
            _bisectormap[i] = new MedialBisector();
            _bisectormap.GetValue(i).IndexNumber(i);
            _bisectormap.GetValue(i).FirstEdge(_theedgelist.Current());
            _bisectormap.GetValue(i).FirstVector(atool.TangentBefore(_theedgelist.Current().EdgeNumber(), _isOpenResult));
            _theedgelist.Next();
            _bisectormap.GetValue(i).SecondEdge(_theedgelist.Current());
            _bisectormap.GetValue(i).IssuePoint(atool.FirstPoint(_theedgelist.Current().EdgeNumber(), out dist));
            _bisectormap.GetValue(i).DistIssuePoint(dist);
            _bisectormap.GetValue(i).SecondVector(atool.TangentAfter(_theedgelist.Current().EdgeNumber(), _isOpenResult));
        }

        // The two bisectors of each edge.
        _theedgelist.First();

        for (i = 0; i < _theedgelist.Number(); i++)
        {
            _theedgelist.Current().FirstBisector(_bisectormap.GetValue((i - 1 + noofbisectors) % noofbisectors));
            _theedgelist.Current().SecondBisector(_bisectormap.GetValue(i));
            _theedgelist.Next();
        }

        //===========================================================================
        //                         Main loop   (step 2)
        //===========================================================================
        int numberOfIte = 0;
        MatCycle? cycle = new();
        Span<int> loopState = stackalloc int[22];

        while (_theedgelist.Number() > 1)
        {
            _pass = numberOfIte;

            // The geometries of the bisectors, through the tool. (step 2.1)
            int nbBis = noofbisectors - beginbisector;
            for (i = beginbisector; i < noofbisectors; i++)
            {
                atool.CreateBisector(_bisectormap.GetValue(i));
                _thenumberofbisectors++;
            }

            // Patch to prevent an infinite loop because of bad geometry.
            if (nbBis == 1)
            {
                if (prefNarea == 1)
                {
                    nbOfNarea1++;
                    int edge1number = _bisectormap.GetValue(beginbisector).FirstEdge().EdgeNumber();
                    int edge2number = _bisectormap.GetValue(beginbisector).SecondEdge().EdgeNumber();
                    if (nbElts[0] == edge1number)
                    {
                        countElts[0]++;
                    }
                    else
                    {
                        countElts[0] = 0;
                        nbElts[0] = edge1number;
                    }
                    if (nbElts[1] == edge2number)
                    {
                        countElts[1]++;
                    }
                    else
                    {
                        countElts[1] = 0;
                        nbElts[1] = edge2number;
                    }
                    if (nbOfNarea1 >= nbMaxNarea1
                        && (countElts[0] >= nbMaxNarea1 || countElts[1] >= nbMaxNarea1))
                    {
                        // Kerf: before OCCT's patch breaks the loop off, the node it goes round, if it goes round
                        // one, is taken as one, and the patch counts again from here (decision 32501)
                        if (!isBreak && TakesNodesAsOne(1))
                        {
                            cycle = new MatCycle();
                            nbOfNarea1 = 0;
                            countElts[0] = 0;
                            countElts[1] = 0;
                        }
                        else
                        {
                            isBreak = true;
                        }
                    }
                }
                else
                {
                    nbOfNarea1 = 0;
                    countElts[0] = 0;
                    countElts[1] = 0;
                }
            }
            prefNarea = nbBis;

            // Exit condition of the main loop.
            if (_theedgelist.Number() < 3)
            {
                break;
            }

            // loop 2: while there are bisectors to remove.
            for (;;)
            {
                nbIterBis++;

                noofbisectorstoremove = 0;
                _theedgelist.First();

                // The intersections of neighbouring bisectors. (step 2.2)
                if (nbIterBis <= evenNbIterBis + 1)
                {
                    edgeNumbers[nbIterBis] = _theedgelist.Number();
                }
                else
                {
                    for (k = 1; k <= evenNbIterBis; k++)
                    {
                        edgeNumbers[k] = edgeNumbers[k + 1];
                    }
                    edgeNumbers[evenNbIterBis + 1] = _theedgelist.Number();
                }
                if (edgeNumbers[evenNbIterBis + 1] == edgeNumbers[1])
                {
                    // Kerf: before OCCT's patch stops the cancelling for good, the node the loop goes round, if it
                    // goes round one, is taken as one, and the patch counts again from here (decision 32501)
                    if (!toNullifyNoofbisectorstoremove && TakesNodesAsOne(1))
                    {
                        cycle = new MatCycle();
                        edgeNumbers.Init(-1);
                        nbIterBis = 1;
                        edgeNumbers[1] = _theedgelist.Number();
                    }
                    else
                    {
                        toNullifyNoofbisectorstoremove = true;
                    }
                }

                for (i = 0; i < _theedgelist.Number(); i++)
                {
                    edge = _theedgelist.Current();
                    if (edge.Distance() == -1.0)
                    {
                        firstbisector = edge.FirstBisector();
                        secondbisector = edge.SecondBisector();
                        edge.Distance(atool.IntersectBisector(firstbisector, secondbisector, ref intersectionpoint));
                        edge.IntersectionPoint(intersectionpoint);

                        if (edge.Distance() == global::Kerf.Engine.Foundation.Precision.Infinite())
                        {
                            if (firstbisector.IndexNumber() >= beginbisector
                                || secondbisector.IndexNumber() >= beginbisector)
                            {
                                Intersect(atool, 0, ref noofbisectorstoremove, firstbisector, secondbisector);
                            }
                        }
                        else
                        {
                            if (firstbisector.IndexNumber() >= beginbisector)
                            {
                                Intersect(atool, 1, ref noofbisectorstoremove, firstbisector, secondbisector);
                            }
                            if (secondbisector.IndexNumber() >= beginbisector)
                            {
                                Intersect(atool, 2, ref noofbisectorstoremove, firstbisector, secondbisector);
                            }
                        }
                    }
                    _theedgelist.Next();
                }

                // Exit test of loop 2.
                if (toNullifyNoofbisectorstoremove)
                {
                    noofbisectorstoremove = 0;
                }
                if (noofbisectorstoremove == 0)
                {
                    break;
                }

                // Cancelling the bisectors to remove. (step 2.4)
                CancelBisectors(atool, noofbisectorstoremove);
            }

            // The parameters of the intersections on the bisectors of each edge, and the portions of the contour
            // to remove. (step 2.5)
            _theedgelist.First();

            currentbisector = _theedgelist.Current().FirstBisector();
            ParamSigns(atool, currentbisector, toleranceofconfusion, _oneNode, out parama[0], out paramb[0]);

            narea = -1;

            for (i = 0; i < _theedgelist.Number(); i++)
            {
                currentbisector = _theedgelist.Current().SecondBisector();
                ParamSigns(atool, currentbisector, toleranceofconfusion, _oneNode, out parama[1], out paramb[1]);

                // Is the edge to remove from the contour? The portions of the contour to remove: narea is their
                // number, firstarea[i] and lastarea[i] the indices of the first and last edges of portion i.
                if (paramb[0] > 0 && parama[1] > 0)
                {
                    AddToArea(ref narea, firstarea, lastarea, noofarea);
                }

                parama[0] = parama[1];
                paramb[0] = paramb[1];
                _theedgelist.Next();
            }

            compact = 0;
            if (narea > 0)
            {
                if (lastarea[narea] == _theedgelist.Number() && firstarea[0] == 1)
                {
                    firstarea[0] = firstarea[narea];
                    noofarea[0] = noofarea[0] + noofarea[narea];
                    compact = noofarea[narea];
                    narea--;
                }
            }

            narea++;

            // Patch to break an infinite loop.
            if (narea == 1 && isBreak)
            {
                narea = 0;
            }

            // Exit of the main loop if no edge is to be removed. (step 2.6)
            if (narea == 0)
            {
                interrupt = true;
                break;
            }

            // Removal of the edges from the contour: the new contour, and the bisectors of the new neighbouring
            // edges.
            beginbisector = noofbisectors;
            shift = 0;
            all = 0;
            if (narea == 1 && noofarea[0] == _theedgelist.Number())
            {
                all = 1;
            }

            for (i = 0; i < narea; i++)
            {
                if (i == 1)
                {
                    shift = shift - compact;
                }
                _theedgelist.First();
                edgetoremove = _theedgelist.Brackets(firstarea[i] - shift);

                RemoveArea(atool, ref edgetoremove, ref noofbisectors, ref shift, all, noofarea[i]);
            }

            // Kerf: a pass that ends in the state an earlier pass ended in is followed by the same passes until the
            // count below runs out, so the count moves on to the pass that leaves the map as the last would (MatCycle)
            if (cycle is not null && numberOfIte <= numberMaxOfIte)
            {
                loopState[0] = Math.Min(nbIterBis, evenNbIterBis + 2);
                loopState[1] = toNullifyNoofbisectorstoremove ? 1 : 0;
                loopState[2] = noofbisectors - beginbisector;
                for (k = 1; k <= evenNbIterBis + 1; k++)
                {
                    loopState[2 + k] = edgeNumbers[k];
                }
                loopState[14] = nbOfNarea1;
                loopState[15] = prefNarea;
                loopState[16] = nbElts[0];
                loopState[17] = nbElts[1];
                loopState[18] = countElts[0];
                loopState[19] = countElts[1];
                loopState[20] = isBreak ? 1 : 0;
                loopState[21] = _oneNode?.Count ?? -1;
                int period = cycle.Step(numberOfIte, _theedgelist, atool, beginbisector, loopState, intersectionpoint);
                if (period > 0 && TakesNodesAsOne(period))
                {
                    // The first time round a node, it is taken as one (Intersect, decision 32501), and the search for
                    // a repeat starts afresh
                    cycle = new MatCycle();
                }
                else if (period > 0)
                {
                    numberOfIte = numberMaxOfIte + 1 - (numberMaxOfIte + 1 - numberOfIte) % period;
                    cycle = null;
                }
            }

            // The number of iterations: each removes an element of the contour that is never reinserted, so it
            // must stay below the number of elements; the maximum is numberofedges * numberofedges.
            if (numberOfIte > numberMaxOfIte)
            {
                _isDone = false; // Failure of the map.
                break;
            }
            numberOfIte++;
        }

        // step 3. interrupt: semi-infinite bisectors.
        if (interrupt)
        {
            _semiInfinite = true;
        }
        else
        {
            _semiInfinite = false;
            LastTwoEdges(atool, noofbisectors);
        }

        if (_semiInfinite)
        {
            beginbisector = noofbisectors;
            _theedgelist.First();
            for (i = 0; i < _theedgelist.Number(); i++)
            {
                edge = _theedgelist.Current();
                _bisectormap[noofbisectors] = edge.SecondBisector();
                noofbisectors++;
                _theedgelist.Next();
            }
        }

        // The roots.
        Roots(beginbisector, noofbisectors);
    }

    // Kerf: true, once in a map, if a bisector reached a new one where it starts within the last `passes` passes: the
    // loop found goes round a node, and from here such nodes are taken as one (decision 32501)
    private bool TakesNodesAsOne(int passes)
    {
        if (_oneNode is not null || _reachedNodeAtPass < _pass - passes)
        {
            return false;
        }
        _oneNode = new HashSet<int>();
        return true;
    }

    // OCCT's step 2.4, the same in CreateMat and CreateMatOpen (written once here): cancels the bisectors to
    // remove, going down each tree from bisectoronetoremove(i) to bisectortwotoremove(i).
    private void CancelBisectors(Tool2d atool, int noofbisectorstoremove)
    {
        MedialBisector bisectortoremove, currentbisector;
        MedialListOfBisector currentbisectorlist;
        MedialEdge previousedge, currentedge;

        for (int i = 0; i < noofbisectorstoremove; i++)
        {
            bisectortoremove = _bisectoronetoremove.GetValue(i);

            // Destruction of the bisectors descending from <bisectortoremove>, down the tree to
            // <bisectortwotoremove(i)>.
            for (;;)
            {
                // Cancelling <bisectortoremove>.
                _thenumberofbisectors--;
                currentbisectorlist = bisectortoremove.List();
                currentbisectorlist.First();
                currentbisector = currentbisectorlist.FirstItem();
                previousedge = currentbisector.FirstEdge();
                _theedgelist!.Init(previousedge);
                previousedge.Distance(-1.0);
                previousedge.FirstBisector().SecondParameter(global::Kerf.Engine.Foundation.Precision.Infinite());
                previousedge.SecondBisector().FirstParameter(global::Kerf.Engine.Foundation.Precision.Infinite());

                // Cancelling the children of <currentbisector>.
                while (currentbisectorlist.More())
                {
                    currentbisector = currentbisectorlist.Current();
                    currentedge = currentbisector.SecondEdge();

                    // Reinsertion of the edge in the contour.
                    _theedgelist.LinkAfter(currentedge);
                    _theedgelist.Next();

                    currentedge.FirstBisector(currentbisector);
                    previousedge.SecondBisector(currentbisector);

                    // Cancelling the intersection: the children that made it are extended to infinity.
                    currentbisector.FirstParameter(global::Kerf.Engine.Foundation.Precision.Infinite());
                    currentbisector.SecondParameter(global::Kerf.Engine.Foundation.Precision.Infinite());

                    atool.TrimBisector(currentbisector);

                    currentedge.Distance(-1.0);
                    currentedge.FirstBisector().SecondParameter(global::Kerf.Engine.Foundation.Precision.Infinite());
                    currentedge.SecondBisector().FirstParameter(global::Kerf.Engine.Foundation.Precision.Infinite());

                    previousedge = currentedge;
                    currentbisectorlist.Next();
                }

                _removedEdgesList!.BackAdd(_theedgelist.Current());
                _theedgelist.Unlink();

                // Exit test of the cancelling loop.
                if (bisectortoremove.BisectorNumber() == _bisectortwotoremove.GetValue(i).BisectorNumber())
                {
                    break;
                }

                // Down the tree.
                if (_typeofbisectortoremove.GetValue(i) == 1)
                {
                    bisectortoremove = bisectortoremove.FirstBisector();
                }
                else
                {
                    bisectortoremove = bisectortoremove.LastBisector();
                }
            }
        }
    }

    // OCCT's sign analysis of the parameters of a bisector (step 2.5), written out twice in each of CreateMat and
    // CreateMatOpen.
    private static void ParamSigns(Tool2d atool, MedialBisector currentbisector, double toleranceofconfusion,
        HashSet<int>? oneNode, out int parama, out int paramb)
    {
        if (currentbisector.FirstParameter() == global::Kerf.Engine.Foundation.Precision.Infinite()
            && currentbisector.SecondParameter() == global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            parama = -1;
            paramb = -1;
        }
        else if (currentbisector.FirstParameter() == global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            parama = -1;
            paramb = 1;
        }
        else if (currentbisector.SecondParameter() == global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            paramb = -1;
            parama = 1;
        }
        else if (oneNode is not null && oneNode.Contains(currentbisector.IndexNumber()))
        {
            // Kerf: its two ends are further apart than the tolerance, and one node all the same (decision 32501)
            parama = 1;
            paramb = 1;
        }
        else if (atool.Distance(currentbisector, currentbisector.FirstParameter(), currentbisector.SecondParameter())
                 > toleranceofconfusion)
        {
            if ((currentbisector.FirstParameter() - currentbisector.SecondParameter()) * currentbisector.Sense() > 0.0)
            {
                parama = -1;
                paramb = 1;
            }
            else
            {
                paramb = -1;
                parama = 1;
            }
        }
        else
        {
            parama = 1;
            paramb = 1;
        }
    }

    // Adds the current edge to the portions of the contour to remove (step 2.5).
    private void AddToArea(ref int narea, NCollection_Array1<int> firstarea, NCollection_Array1<int> lastarea,
        NCollection_Array1<int> noofarea)
    {
        if (narea < 0)
        {
            firstarea[++narea] = _theedgelist!.Index();
            lastarea[narea] = firstarea[narea];
            noofarea[narea] = 1;
        }
        else
        {
            if (_theedgelist!.Index() == lastarea[narea] + 1)
            {
                lastarea[narea]++;
                noofarea[narea]++;
            }
            else
            {
                firstarea[++narea] = _theedgelist.Index();
                lastarea[narea] = firstarea[narea];
                noofarea[narea] = 1;
            }
        }
    }

    // Removes a portion of noofarea edges from edgetoremove and creates the bisector of the new neighbours (the
    // body of OCCT's loop over the portions, the same in CreateMat and CreateMatOpen).
    private void RemoveArea(Tool2d atool, ref MedialEdge edgetoremove, ref int noofbisectors, ref int shift,
        int all, int noofarea)
    {
        MedialListOfEdge theedges = _theedgelist!;

        edgetoremove.FirstBisector().EndPoint(edgetoremove.IntersectionPoint());

        edgetoremove.FirstBisector().FirstParameter(edgetoremove.FirstBisector().SecondParameter());

        atool.TrimBisector(edgetoremove.FirstBisector());

        // Kerf: a portion with a bisector taken as one node is a node spread over more than the tolerance, and its new
        // bisector starts from the point of the portion nearest the contour, not from its first edge's, so that what
        // reaches the node later is not behind where the bisector starts (decision 32501)
        double issueDistance = edgetoremove.Distance();
        int issuePoint = edgetoremove.IntersectionPoint();
        if (_oneNode is { Count: > 0 })
        {
            double leastDistance = issueDistance;
            int nearestPoint = issuePoint;
            bool spread = _oneNode.Contains(edgetoremove.FirstBisector().IndexNumber())
                          || _oneNode.Contains(edgetoremove.SecondBisector().IndexNumber());
            for (int j = 1; j < noofarea; j++)
            {
                theedges.Next();
                MedialEdge edge = theedges.Current();
                spread = spread || _oneNode.Contains(edge.SecondBisector().IndexNumber());
                if (edge.Distance() < leastDistance)
                {
                    leastDistance = edge.Distance();
                    nearestPoint = edge.IntersectionPoint();
                }
            }
            for (int j = 1; j < noofarea; j++)
            {
                theedges.Previous();
            }
            if (spread)
            {
                issueDistance = leastDistance;
                issuePoint = nearestPoint;
            }
        }

        _bisectormap[noofbisectors] = new MedialBisector();
        _bisectormap.GetValue(noofbisectors).IndexNumber(noofbisectors);
        _bisectormap.GetValue(noofbisectors).DistIssuePoint(issueDistance);
        _bisectormap.GetValue(noofbisectors).IssuePoint(issuePoint);
        _bisectormap.GetValue(noofbisectors).FirstEdge(theedges.PreviousItem());
        _bisectormap.GetValue(noofbisectors).AddBisector(edgetoremove.FirstBisector());

        for (int j = 0; j < noofarea; j++)
        {
            _removedEdgesList!.BackAdd(theedges.Current());
            theedges.Unlink();
            theedges.Next();
            shift++;

            if (all == 0 || j + 1 != noofarea)
            {
                _bisectormap.GetValue(noofbisectors).AddBisector(edgetoremove.SecondBisector());
            }
            edgetoremove.SecondBisector().EndPoint(edgetoremove.IntersectionPoint());

            edgetoremove.SecondBisector().SecondParameter(edgetoremove.SecondBisector().FirstParameter());

            atool.TrimBisector(edgetoremove.SecondBisector());

            edgetoremove = theedges.Current();
        }

        _bisectormap.GetValue(noofbisectors).SecondEdge(theedges.Current());

        theedges.PreviousItem().SecondBisector(_bisectormap.GetValue(noofbisectors));
        theedges.Current().FirstBisector(_bisectormap.GetValue(noofbisectors));

        _bisectormap.GetValue(noofbisectors).FirstVector(atool.Tangent(_bisectormap.GetValue(noofbisectors).FirstBisector().BisectorNumber()));

        _bisectormap.GetValue(noofbisectors).SecondVector(atool.Tangent(_bisectormap.GetValue(noofbisectors).LastBisector().BisectorNumber()));

        noofbisectors++;

        theedges.PreviousItem().Distance(-1);
        theedges.Current().Distance(-1);

        theedges.PreviousItem().FirstBisector().SecondParameter(global::Kerf.Engine.Foundation.Precision.Infinite());
        theedges.Current().SecondBisector().FirstParameter(global::Kerf.Engine.Foundation.Precision.Infinite());
    }

    // Step 3 when the main loop ended without interruption: if two edges are left, their last two bisectors
    // separate the same edges (OCCT notes the branch is no longer reachable: two edges are processed in the main
    // loop).
    private void LastTwoEdges(Tool2d atool, int noofbisectors)
    {
        MedialEdge edge;

        if (_theedgelist!.Number() > 1)
        {
            _theedgelist.First();
            edge = _theedgelist.Current();
            if (edge.FirstBisector().IndexNumber() == noofbisectors - 1)
            {
                if (atool.TrimBisector(edge.SecondBisector(), edge.FirstBisector().IssuePoint()))
                {
                    if (edge.SecondBisector().EndPoint() == 0)
                    {
                        edge.SecondBisector().EndPoint(edge.FirstBisector().IssuePoint());
                    }
                    _bisectormap.GetValue(noofbisectors - 1).AddBisector(edge.SecondBisector());
                }
                else
                {
                    _semiInfinite = true;
                }
            }
            else
            {
                if (atool.TrimBisector(edge.FirstBisector(), edge.SecondBisector().IssuePoint()))
                {
                    if (edge.FirstBisector().EndPoint() == 0)
                    {
                        edge.FirstBisector().EndPoint(edge.SecondBisector().IssuePoint());
                    }
                    _bisectormap.GetValue(noofbisectors - 1).AddBisector(edge.FirstBisector());
                }
                else
                {
                    _semiInfinite = true;
                }
            }
            if (!_semiInfinite)
            {
                _thenumberofbisectors--;
                _bisectormap.GetValue(noofbisectors - 1).SecondEdge(edge);
                _bisectormap.GetValue(noofbisectors - 1).BisectorNumber(-1);
            }
        }
    }

    // The roots of the trees of bisectors.
    private void Roots(int beginbisector, int noofbisectors)
    {
        _roots = new MedialListOfBisector();

        if (_bisectormap.GetValue(noofbisectors - 1).BisectorNumber() == -1)
        {
            _roots = _bisectormap.GetValue(noofbisectors - 1).List();
            if (_roots.IsEmpty)
            {
                // A single element (a contour of one closed edge): the main loop never runs and the only bisector
                // is never built. The map fails, where OCCT dereferenced the empty list's null item (decision 1438).
                _isDone = false;
                return;
            }
            _roots.First();
            _roots.Current().FirstEdge().Distance(_bisectormap.GetValue(noofbisectors - 1).DistIssuePoint());
        }
        else
        {
            for (int i = beginbisector; i < noofbisectors; i++)
            {
                _roots.BackAdd(_bisectormap.GetValue(i));
            }
        }
    }

    // Loads the bisectors to remove.
    private void LoadBisectorsToRemove(ref int noofbisectorstoremove, double distance1, double distance2,
        MedialBisector firstbisectortoremove1, MedialBisector firstbisectortoremove2, MedialBisector? lastbisectortoremove1,
        MedialBisector? lastbisectortoremove2)
    {
        int found, index;
        MedialBisector[] firstbisectortoremove = [firstbisectortoremove1, firstbisectortoremove2];
        MedialBisector?[] lastbisectortoremove = [lastbisectortoremove1, lastbisectortoremove2];

        if (distance1 < global::Kerf.Engine.Foundation.Precision.Infinite() && distance2 == global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            index = 0;
        }
        else if (distance2 < global::Kerf.Engine.Foundation.Precision.Infinite() && distance1 == global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            index = 1;
        }
        else
        {
            index = -1;
        }

        if (index != -1)
        {
            found = noofbisectorstoremove;
            for (int j = 0; j < noofbisectorstoremove; j++)
            {
                if (_bisectoronetoremove.GetValue(j).BisectorNumber() == firstbisectortoremove[index].BisectorNumber())
                {
                    found = j;
                    if (_bisectortwotoremove.GetValue(j).BisectorNumber() < lastbisectortoremove[index]!.BisectorNumber())
                    {
                        found = -1;
                    }
                    break;
                }
            }

            if (found != -1)
            {
                _bisectoronetoremove[found] = firstbisectortoremove[index];
                _bisectortwotoremove[found] = lastbisectortoremove[index]!;
                _typeofbisectortoremove[found] = index + 1;

                if (found == noofbisectorstoremove)
                {
                    noofbisectorstoremove++;
                }
            }
        }
    }

    // If aside = 0: intersects firstbisector with the leftmost descendants of secondbisector
    // (secondbisector->FirstBisector()->FirstBisector()...) and secondbisector with the rightmost descendants of
    // firstbisector (firstbisector->LastBisector()->LastBisector()...). If aside = 1: firstbisector with its own
    // leftmost and rightmost descendants. If aside = 2: secondbisector with its own.
    private void Intersect(Tool2d atool, int aside, ref int noofbisectortoremove, MedialBisector firstbisector,
        MedialBisector secondbisector)
    {
        int bisectornumber;
        double distant, saveparameter;
        Span<double> distance = stackalloc double[2];
        int intersectionpoint = 0;
        MedialBisector lastbisector, previousbisector;
        MedialBisector[] firstbisectortoremove = new MedialBisector[2];
        MedialBisector?[] lastbisectortoremove = new MedialBisector?[2];

        distance[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
        distance[1] = global::Kerf.Engine.Foundation.Precision.Infinite();

        // Kerf: the two bisectors were found to meet where one of them starts, which does not count (decision 32501)
        int refusedAtStartOf = aside == 0 ? atool.RefusedAtStartOf : 0;
        global::Kerf.Geometry.Point2D refusedPoint = refusedAtStartOf != 0 ? atool.RefusedPoint : default;

        for (bisectornumber = 0; bisectornumber < 2; bisectornumber++)
        {
            if (aside == 0)
            {
                if (bisectornumber == 0)
                {
                    firstbisectortoremove[bisectornumber] = secondbisector;
                }
                else
                {
                    firstbisectortoremove[bisectornumber] = firstbisector;
                }
            }
            else if (aside == 1)
            {
                firstbisectortoremove[bisectornumber] = firstbisector;
            }
            else
            {
                firstbisectortoremove[bisectornumber] = secondbisector;
            }

            lastbisector = firstbisectortoremove[bisectornumber];

            if (aside == 0)
            {
                previousbisector = firstbisectortoremove[bisectornumber];
            }
            else
            {
                if (firstbisectortoremove[bisectornumber].List().IsEmpty)
                {
                    continue;
                }

                if (bisectornumber == 0)
                {
                    previousbisector = firstbisectortoremove[bisectornumber].FirstBisector();
                }
                else
                {
                    previousbisector = firstbisectortoremove[bisectornumber].LastBisector();
                }
            }

            distant = distance[bisectornumber];
            while (!previousbisector.List().IsEmpty)
            {
                if (bisectornumber == 0)
                {
                    previousbisector = previousbisector.FirstBisector();
                }
                else
                {
                    previousbisector = previousbisector.LastBisector();
                }

                if (aside == 1 || (aside == 0 && bisectornumber == 0))
                {
                    saveparameter = previousbisector.FirstParameter();
                    distant = atool.IntersectBisector(firstbisector, previousbisector, ref intersectionpoint);
                    previousbisector.FirstParameter(saveparameter);
                }
                else
                {
                    saveparameter = previousbisector.SecondParameter();
                    distant = atool.IntersectBisector(previousbisector, secondbisector, ref intersectionpoint);
                    previousbisector.SecondParameter(saveparameter);
                }

                if (distant < global::Kerf.Engine.Foundation.Precision.Infinite())
                {
                    distance[bisectornumber] = distant;
                    lastbisectortoremove[bisectornumber] = lastbisector;

                    // Kerf: a bisector that reaches a new one where it starts, and crosses the child that ended there
                    // at that same point as far as the two can tell, has reached the node the child ended at. Once
                    // the map is known to go round such a node, the two ends of the child are one node (decision 32501)
                    if (refusedAtStartOf == 2 - bisectornumber
                        && atool.CrossAt(bisectornumber == 0 ? firstbisector : secondbisector, previousbisector,
                            intersectionpoint, refusedPoint, atool.ToleranceOfConfusion()))
                    {
                        _reachedNodeAtPass = _pass;
                        _oneNode?.Add(previousbisector.IndexNumber());
                    }
                }

                lastbisector = previousbisector;
            }
        }

        // Loading the bisectors to remove.
        LoadBisectorsToRemove(ref noofbisectortoremove, distance[0], distance[1], firstbisectortoremove[0],
            firstbisectortoremove[1], lastbisectortoremove[0], lastbisectortoremove[1]);
    }

    /// <summary>Starts an iteration over the roots of the trees of bisectors.</summary>
    public void Init() => _roots!.First();

    /// <summary>False when there are no more roots.</summary>
    public bool More() => _roots!.More();

    /// <summary>Moves to the next root.</summary>
    public void Next() => _roots!.Next();

    /// <summary>The current root.</summary>
    public MedialBisector Bisector() => _roots!.Current();

    /// <summary>True if there are semi-infinite bisectors (then there is a tree per semi-infinite bisector).</summary>
    public bool SemiInfinite() => _semiInfinite;

    /// <summary>The total number of bisectors.</summary>
    public int NumberOfBisectors() => _thenumberofbisectors;

    /// <summary>True if CreateMat succeeded.</summary>
    public bool IsDone => _isDone;
}
