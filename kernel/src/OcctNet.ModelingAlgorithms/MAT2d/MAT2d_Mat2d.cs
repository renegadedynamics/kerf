// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Mat2d.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Mat2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public sealed class MAT2d_Mat2d
{
    private readonly bool myIsOpenResult;
    private int thenumberofbisectors;
    private bool semiInfinite;
    private MAT_ListOfEdge? theedgelist;
    private MAT_ListOfEdge? RemovedEdgesList;
    private readonly NCollection_DataMap<int, int> typeofbisectortoremove = new();
    private readonly NCollection_DataMap<int, MAT_Bisector> bisectoronetoremove = new();
    private readonly NCollection_DataMap<int, MAT_Bisector> bisectortwotoremove = new();
    private readonly NCollection_DataMap<int, MAT_Bisector> bisectormap = new();
    private MAT_ListOfBisector? roots;
    private bool isDone;

    public MAT2d_Mat2d(bool IsOpenResult = false)
    {
        semiInfinite = false;
        isDone = false;
        myIsOpenResult = IsOpenResult;
        thenumberofbisectors = 0;
        // OCCT's thenumberofedges is set here and never read: left out.
    }

    /// <summary>The bisecting locus of an open wire.</summary>
    public void CreateMatOpen(MAT2d_Tool2d atool)
    {
        bool interrupt = false;
        MAT_Edge edgetoremove;

        int noofbisectorstoremove;
        MAT_Bisector firstbisector, secondbisector;
        MAT_Edge edge;
        int intersectionpoint = 0;
        int beginbisector;
        int noofbisectors;

        int NbIterBis = 0;
        int EvenNbIterBis = 10;
        NCollection_Array1<int> EdgeNumbers = new(1, EvenNbIterBis + 1);
        EdgeNumbers.Init(-1);
        bool ToNullifyNoofbisectorstoremove = false;

        MAT_Bisector currentbisector;

        int i, k, narea, shift, compact, all;
        int noofedges;
        int NumberMaxOfIte;
        double toleranceofconfusion;

        noofedges = atool.NumberOfItems();
        toleranceofconfusion = atool.ToleranceOfConfusion();
        NumberMaxOfIte = noofedges * noofedges;

        NCollection_Array1<int> firstarea = new(0, noofedges);
        NCollection_Array1<int> lastarea = new(0, noofedges);
        NCollection_Array1<int> noofarea = new(0, noofedges);

        Span<int> parama = stackalloc int[2];
        Span<int> paramb = stackalloc int[2];

        // Initialisation of the maps.
        bisectoronetoremove.Clear();
        bisectortwotoremove.Clear();
        typeofbisectortoremove.Clear();
        bisectormap.Clear();

        isDone = true;
        noofbisectors = noofedges - 1;
        beginbisector = 0;

        // <theedgelist>: an edge per simple element of the contour.
        theedgelist = new MAT_ListOfEdge();
        RemovedEdgesList = new MAT_ListOfEdge();

        for (i = 0; i < noofedges; i++)
        {
            edge = new MAT_Edge();
            edge.EdgeNumber(i + 1);
            edge.Distance(-1);
            theedgelist.BackAdd(edge);
        }

        theedgelist.Loop();

        // The bisectors from the contour.
        double Dist;
        theedgelist.First();

        for (i = 0; i < theedgelist.Number() - 1; i++)
        {
            bisectormap.Bind(i, new MAT_Bisector());
            bisectormap[i].IndexNumber(i);
            bisectormap[i].FirstEdge(theedgelist.Current());
            bisectormap[i].FirstVector(atool.TangentBefore(theedgelist.Current().EdgeNumber(), myIsOpenResult));
            theedgelist.Next();
            bisectormap[i].SecondEdge(theedgelist.Current());
            bisectormap[i].IssuePoint(atool.FirstPoint(theedgelist.Current().EdgeNumber(), out Dist));
            bisectormap[i].DistIssuePoint(Dist);
            bisectormap[i].SecondVector(atool.TangentAfter(theedgelist.Current().EdgeNumber(), myIsOpenResult));
        }

        // The two bisectors of each edge.
        theedgelist.First();
        theedgelist.Current().FirstBisector(bisectormap[0]);
        theedgelist.Current().SecondBisector(bisectormap[0]);
        theedgelist.Next();

        for (i = 1; i < theedgelist.Number() - 1; i++)
        {
            theedgelist.Current().FirstBisector(bisectormap[i - 1]);
            theedgelist.Current().SecondBisector(bisectormap[i]);
            theedgelist.Next();
        }

        theedgelist.Current().FirstBisector(bisectormap[theedgelist.Number() - 2]);
        theedgelist.Current().SecondBisector(bisectormap[theedgelist.Number() - 2]);

        //===========================================================================
        //                         Main loop   (step 2)
        //===========================================================================
        int NumberOfIte = 0;

        while (theedgelist.Number() > 1)
        {
            // The geometries of the bisectors, through the tool. (step 2.1)
            for (i = beginbisector; i < noofbisectors; i++)
            {
                atool.CreateBisector(bisectormap[i]);
                thenumberofbisectors++;
            }

            // Exit condition of the main loop.
            if (theedgelist.Number() < 3)
            {
                break;
            }

            // loop 2: while there are bisectors to remove.
            for (;;)
            {
                NbIterBis++;

                noofbisectorstoremove = 0;
                theedgelist.First();
                theedgelist.Next();

                // The intersections of neighbouring bisectors. (step 2.2)
                if (NbIterBis <= EvenNbIterBis + 1)
                {
                    EdgeNumbers[NbIterBis] = theedgelist.Number();
                }
                else
                {
                    for (k = 1; k <= EvenNbIterBis; k++)
                    {
                        EdgeNumbers[k] = EdgeNumbers[k + 1];
                    }
                    EdgeNumbers[EvenNbIterBis + 1] = theedgelist.Number();
                }
                if (EdgeNumbers[EvenNbIterBis + 1] == EdgeNumbers[1])
                {
                    ToNullifyNoofbisectorstoremove = true;
                }

                for (i = 1; i < theedgelist.Number() - 1; i++)
                {
                    edge = theedgelist.Current();
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
                    theedgelist.Next();
                }

                // Exit test of loop 2.
                if (ToNullifyNoofbisectorstoremove)
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
            theedgelist.First();
            theedgelist.Next();

            currentbisector = theedgelist.Current().FirstBisector();
            ParamSigns(atool, currentbisector, toleranceofconfusion, out parama[0], out paramb[0]);

            narea = -1;

            for (i = 1; i < theedgelist.Number() - 1; i++)
            {
                currentbisector = theedgelist.Current().SecondBisector();
                ParamSigns(atool, currentbisector, toleranceofconfusion, out parama[1], out paramb[1]);

                // Is the edge to remove from the contour? The portions of the contour to remove: narea is their
                // number, firstarea[i] and lastarea[i] the indices of the first and last edges of portion i.
                if (paramb[0] > 0 && parama[1] > 0)
                {
                    AddToArea(ref narea, firstarea, lastarea, noofarea);
                }

                parama[0] = parama[1];
                paramb[0] = paramb[1];
                theedgelist.Next();
            }

            compact = 0;
            if (narea > 0)
            {
                if (lastarea[narea] == theedgelist.Number() && firstarea[0] == 1)
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
            if (narea == 1 && noofarea[0] == theedgelist.Number())
            {
                all = 1;
            }

            for (i = 0; i < narea; i++)
            {
                if (i == 1)
                {
                    shift = shift - compact;
                }
                theedgelist.First();
                theedgelist.Next();
                edgetoremove = theedgelist.Brackets(firstarea[i] - shift);

                RemoveArea(atool, ref edgetoremove, ref noofbisectors, ref shift, all, noofarea[i]);
            }

            // The number of iterations: each removes an element of the contour that is never reinserted, so it
            // must stay below the number of elements; the maximum is numberofedges * numberofedges.
            if (NumberOfIte > NumberMaxOfIte)
            {
                isDone = false; // Failure of the map.
                break;
            }
            NumberOfIte++;
        }

        // step 3. interrupt: semi-infinite bisectors.
        if (interrupt)
        {
            semiInfinite = true;
        }
        else
        {
            semiInfinite = false;
            LastTwoEdges(atool, noofbisectors);
        }

        if (semiInfinite)
        {
            beginbisector = noofbisectors;
            theedgelist.First();
            for (i = 1; i < theedgelist.Number(); i++)
            {
                edge = theedgelist.Current();
                bisectormap.Bind(noofbisectors, edge.SecondBisector());
                noofbisectors++;
                theedgelist.Next();
            }
        }

        // The roots.
        Roots(beginbisector, noofbisectors);
    }

    /// <summary>The bisecting locus.</summary>
    public void CreateMat(MAT2d_Tool2d atool)
    {
        bool interrupt = false;
        MAT_Edge edgetoremove;

        int noofbisectorstoremove;
        MAT_Bisector firstbisector, secondbisector;
        MAT_Edge edge;
        int intersectionpoint = 0;
        int beginbisector;
        int noofbisectors;

        int NbIterBis = 0;
        int EvenNbIterBis = 10;
        NCollection_Array1<int> EdgeNumbers = new(1, EvenNbIterBis + 1);
        EdgeNumbers.Init(-1);
        bool ToNullifyNoofbisectorstoremove = false;

        MAT_Bisector currentbisector;

        int i, k, narea, shift, compact, all;
        int noofedges;
        int NumberMaxOfIte;
        double toleranceofconfusion;

        noofedges = atool.NumberOfItems();
        toleranceofconfusion = atool.ToleranceOfConfusion();
        NumberMaxOfIte = noofedges * noofedges;

        NCollection_Array1<int> firstarea = new(0, noofedges);
        NCollection_Array1<int> lastarea = new(0, noofedges);
        NCollection_Array1<int> noofarea = new(0, noofedges);

        Span<int> parama = stackalloc int[2];
        Span<int> paramb = stackalloc int[2];

        int aNbOfNarea1 = 0, aPrefNarea = 0, aNbMaxNarea1 = 10;
        Span<int> aNbElts = stackalloc int[2] { 0, 0 };
        Span<int> aCountElts = stackalloc int[2] { 0, 0 };
        bool isBreak = false;

        // Initialisation of the maps.
        bisectoronetoremove.Clear();
        bisectortwotoremove.Clear();
        typeofbisectortoremove.Clear();
        bisectormap.Clear();

        isDone = true;
        noofbisectors = noofedges;
        beginbisector = 0;

        // <theedgelist>: an edge per simple element of the contour.
        theedgelist = new MAT_ListOfEdge();
        RemovedEdgesList = new MAT_ListOfEdge();

        for (i = 0; i < noofedges; i++)
        {
            edge = new MAT_Edge();
            edge.EdgeNumber(i + 1);
            edge.Distance(-1);
            theedgelist.BackAdd(edge);
        }

        theedgelist.Loop();

        // The bisectors from the contour.
        double Dist;
        theedgelist.First();

        for (i = 0; i < theedgelist.Number(); i++)
        {
            bisectormap.Bind(i, new MAT_Bisector());
            bisectormap[i].IndexNumber(i);
            bisectormap[i].FirstEdge(theedgelist.Current());
            bisectormap[i].FirstVector(atool.TangentBefore(theedgelist.Current().EdgeNumber(), myIsOpenResult));
            theedgelist.Next();
            bisectormap[i].SecondEdge(theedgelist.Current());
            bisectormap[i].IssuePoint(atool.FirstPoint(theedgelist.Current().EdgeNumber(), out Dist));
            bisectormap[i].DistIssuePoint(Dist);
            bisectormap[i].SecondVector(atool.TangentAfter(theedgelist.Current().EdgeNumber(), myIsOpenResult));
        }

        // The two bisectors of each edge.
        theedgelist.First();

        for (i = 0; i < theedgelist.Number(); i++)
        {
            theedgelist.Current().FirstBisector(bisectormap[(i - 1 + noofbisectors) % noofbisectors]);
            theedgelist.Current().SecondBisector(bisectormap[i]);
            theedgelist.Next();
        }

        //===========================================================================
        //                         Main loop   (step 2)
        //===========================================================================
        int NumberOfIte = 0;

        while (theedgelist.Number() > 1)
        {
            // The geometries of the bisectors, through the tool. (step 2.1)
            int aNbBis = noofbisectors - beginbisector;
            for (i = beginbisector; i < noofbisectors; i++)
            {
                atool.CreateBisector(bisectormap[i]);
                thenumberofbisectors++;
            }

            // Patch to prevent an infinite loop because of bad geometry.
            if (aNbBis == 1)
            {
                if (aPrefNarea == 1)
                {
                    aNbOfNarea1++;
                    int edge1number = bisectormap[beginbisector].FirstEdge().EdgeNumber();
                    int edge2number = bisectormap[beginbisector].SecondEdge().EdgeNumber();
                    if (aNbElts[0] == edge1number)
                    {
                        aCountElts[0]++;
                    }
                    else
                    {
                        aCountElts[0] = 0;
                        aNbElts[0] = edge1number;
                    }
                    if (aNbElts[1] == edge2number)
                    {
                        aCountElts[1]++;
                    }
                    else
                    {
                        aCountElts[1] = 0;
                        aNbElts[1] = edge2number;
                    }
                    if (aNbOfNarea1 >= aNbMaxNarea1
                        && (aCountElts[0] >= aNbMaxNarea1 || aCountElts[1] >= aNbMaxNarea1))
                    {
                        isBreak = true;
                    }
                }
                else
                {
                    aNbOfNarea1 = 0;
                    aCountElts[0] = 0;
                    aCountElts[1] = 0;
                }
            }
            aPrefNarea = aNbBis;

            // Exit condition of the main loop.
            if (theedgelist.Number() < 3)
            {
                break;
            }

            // loop 2: while there are bisectors to remove.
            for (;;)
            {
                NbIterBis++;

                noofbisectorstoremove = 0;
                theedgelist.First();

                // The intersections of neighbouring bisectors. (step 2.2)
                if (NbIterBis <= EvenNbIterBis + 1)
                {
                    EdgeNumbers[NbIterBis] = theedgelist.Number();
                }
                else
                {
                    for (k = 1; k <= EvenNbIterBis; k++)
                    {
                        EdgeNumbers[k] = EdgeNumbers[k + 1];
                    }
                    EdgeNumbers[EvenNbIterBis + 1] = theedgelist.Number();
                }
                if (EdgeNumbers[EvenNbIterBis + 1] == EdgeNumbers[1])
                {
                    ToNullifyNoofbisectorstoremove = true;
                }

                for (i = 0; i < theedgelist.Number(); i++)
                {
                    edge = theedgelist.Current();
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
                    theedgelist.Next();
                }

                // Exit test of loop 2.
                if (ToNullifyNoofbisectorstoremove)
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
            theedgelist.First();

            currentbisector = theedgelist.Current().FirstBisector();
            ParamSigns(atool, currentbisector, toleranceofconfusion, out parama[0], out paramb[0]);

            narea = -1;

            for (i = 0; i < theedgelist.Number(); i++)
            {
                currentbisector = theedgelist.Current().SecondBisector();
                ParamSigns(atool, currentbisector, toleranceofconfusion, out parama[1], out paramb[1]);

                // Is the edge to remove from the contour? The portions of the contour to remove: narea is their
                // number, firstarea[i] and lastarea[i] the indices of the first and last edges of portion i.
                if (paramb[0] > 0 && parama[1] > 0)
                {
                    AddToArea(ref narea, firstarea, lastarea, noofarea);
                }

                parama[0] = parama[1];
                paramb[0] = paramb[1];
                theedgelist.Next();
            }

            compact = 0;
            if (narea > 0)
            {
                if (lastarea[narea] == theedgelist.Number() && firstarea[0] == 1)
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
            if (narea == 1 && noofarea[0] == theedgelist.Number())
            {
                all = 1;
            }

            for (i = 0; i < narea; i++)
            {
                if (i == 1)
                {
                    shift = shift - compact;
                }
                theedgelist.First();
                edgetoremove = theedgelist.Brackets(firstarea[i] - shift);

                RemoveArea(atool, ref edgetoremove, ref noofbisectors, ref shift, all, noofarea[i]);
            }

            // The number of iterations: each removes an element of the contour that is never reinserted, so it
            // must stay below the number of elements; the maximum is numberofedges * numberofedges.
            if (NumberOfIte > NumberMaxOfIte)
            {
                isDone = false; // Failure of the map.
                break;
            }
            NumberOfIte++;
        }

        // step 3. interrupt: semi-infinite bisectors.
        if (interrupt)
        {
            semiInfinite = true;
        }
        else
        {
            semiInfinite = false;
            LastTwoEdges(atool, noofbisectors);
        }

        if (semiInfinite)
        {
            beginbisector = noofbisectors;
            theedgelist.First();
            for (i = 0; i < theedgelist.Number(); i++)
            {
                edge = theedgelist.Current();
                bisectormap.Bind(noofbisectors, edge.SecondBisector());
                noofbisectors++;
                theedgelist.Next();
            }
        }

        // The roots.
        Roots(beginbisector, noofbisectors);
    }

    // OCCT's step 2.4, the same in CreateMat and CreateMatOpen (written once here): cancels the bisectors to
    // remove, going down each tree from bisectoronetoremove(i) to bisectortwotoremove(i).
    private void CancelBisectors(MAT2d_Tool2d atool, int noofbisectorstoremove)
    {
        MAT_Bisector bisectortoremove, currentbisector;
        MAT_ListOfBisector currentbisectorlist;
        MAT_Edge previousedge, currentedge;

        for (int i = 0; i < noofbisectorstoremove; i++)
        {
            bisectortoremove = bisectoronetoremove[i];

            // Destruction of the bisectors descending from <bisectortoremove>, down the tree to
            // <bisectortwotoremove(i)>.
            for (;;)
            {
                // Cancelling <bisectortoremove>.
                thenumberofbisectors--;
                currentbisectorlist = bisectortoremove.List();
                currentbisectorlist.First();
                currentbisector = currentbisectorlist.FirstItem();
                previousedge = currentbisector.FirstEdge();
                theedgelist!.Init(previousedge);
                previousedge.Distance(-1.0);
                previousedge.FirstBisector().SecondParameter(global::Kerf.Engine.Foundation.Precision.Infinite());
                previousedge.SecondBisector().FirstParameter(global::Kerf.Engine.Foundation.Precision.Infinite());

                // Cancelling the children of <currentbisector>.
                while (currentbisectorlist.More())
                {
                    currentbisector = currentbisectorlist.Current();
                    currentedge = currentbisector.SecondEdge();

                    // Reinsertion of the edge in the contour.
                    theedgelist.LinkAfter(currentedge);
                    theedgelist.Next();

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

                RemovedEdgesList!.BackAdd(theedgelist.Current());
                theedgelist.Unlink();

                // Exit test of the cancelling loop.
                if (bisectortoremove.BisectorNumber() == bisectortwotoremove[i].BisectorNumber())
                {
                    break;
                }

                // Down the tree.
                if (typeofbisectortoremove[i] == 1)
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
    private static void ParamSigns(MAT2d_Tool2d atool, MAT_Bisector currentbisector, double toleranceofconfusion,
        out int parama, out int paramb)
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
            firstarea[++narea] = theedgelist!.Index();
            lastarea[narea] = firstarea[narea];
            noofarea[narea] = 1;
        }
        else
        {
            if (theedgelist!.Index() == lastarea[narea] + 1)
            {
                lastarea[narea]++;
                noofarea[narea]++;
            }
            else
            {
                firstarea[++narea] = theedgelist.Index();
                lastarea[narea] = firstarea[narea];
                noofarea[narea] = 1;
            }
        }
    }

    // Removes a portion of noofarea edges from edgetoremove and creates the bisector of the new neighbours (the
    // body of OCCT's loop over the portions, the same in CreateMat and CreateMatOpen).
    private void RemoveArea(MAT2d_Tool2d atool, ref MAT_Edge edgetoremove, ref int noofbisectors, ref int shift,
        int all, int noofarea)
    {
        MAT_ListOfEdge theedges = theedgelist!;

        edgetoremove.FirstBisector().EndPoint(edgetoremove.IntersectionPoint());

        edgetoremove.FirstBisector().FirstParameter(edgetoremove.FirstBisector().SecondParameter());

        atool.TrimBisector(edgetoremove.FirstBisector());

        bisectormap.Bind(noofbisectors, new MAT_Bisector());
        bisectormap[noofbisectors].IndexNumber(noofbisectors);
        bisectormap[noofbisectors].DistIssuePoint(edgetoremove.Distance());
        bisectormap[noofbisectors].IssuePoint(edgetoremove.IntersectionPoint());
        bisectormap[noofbisectors].FirstEdge(theedges.PreviousItem());
        bisectormap[noofbisectors].AddBisector(edgetoremove.FirstBisector());

        for (int j = 0; j < noofarea; j++)
        {
            RemovedEdgesList!.BackAdd(theedges.Current());
            theedges.Unlink();
            theedges.Next();
            shift++;

            if (all == 0 || j + 1 != noofarea)
            {
                bisectormap[noofbisectors].AddBisector(edgetoremove.SecondBisector());
            }
            edgetoremove.SecondBisector().EndPoint(edgetoremove.IntersectionPoint());

            edgetoremove.SecondBisector().SecondParameter(edgetoremove.SecondBisector().FirstParameter());

            atool.TrimBisector(edgetoremove.SecondBisector());

            edgetoremove = theedges.Current();
        }

        bisectormap[noofbisectors].SecondEdge(theedges.Current());

        theedges.PreviousItem().SecondBisector(bisectormap[noofbisectors]);
        theedges.Current().FirstBisector(bisectormap[noofbisectors]);

        bisectormap[noofbisectors].FirstVector(atool.Tangent(bisectormap[noofbisectors].FirstBisector().BisectorNumber()));

        bisectormap[noofbisectors].SecondVector(atool.Tangent(bisectormap[noofbisectors].LastBisector().BisectorNumber()));

        noofbisectors++;

        theedges.PreviousItem().Distance(-1);
        theedges.Current().Distance(-1);

        theedges.PreviousItem().FirstBisector().SecondParameter(global::Kerf.Engine.Foundation.Precision.Infinite());
        theedges.Current().SecondBisector().FirstParameter(global::Kerf.Engine.Foundation.Precision.Infinite());
    }

    // Step 3 when the main loop ended without interruption: if two edges are left, their last two bisectors
    // separate the same edges (OCCT notes the branch is no longer reachable: two edges are processed in the main
    // loop).
    private void LastTwoEdges(MAT2d_Tool2d atool, int noofbisectors)
    {
        MAT_Edge edge;

        if (theedgelist!.Number() > 1)
        {
            theedgelist.First();
            edge = theedgelist.Current();
            if (edge.FirstBisector().IndexNumber() == noofbisectors - 1)
            {
                if (atool.TrimBisector(edge.SecondBisector(), edge.FirstBisector().IssuePoint()))
                {
                    if (edge.SecondBisector().EndPoint() == 0)
                    {
                        edge.SecondBisector().EndPoint(edge.FirstBisector().IssuePoint());
                    }
                    bisectormap[noofbisectors - 1].AddBisector(edge.SecondBisector());
                }
                else
                {
                    semiInfinite = true;
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
                    bisectormap[noofbisectors - 1].AddBisector(edge.FirstBisector());
                }
                else
                {
                    semiInfinite = true;
                }
            }
            if (!semiInfinite)
            {
                thenumberofbisectors--;
                bisectormap[noofbisectors - 1].SecondEdge(edge);
                bisectormap[noofbisectors - 1].BisectorNumber(-1);
            }
        }
    }

    // The roots of the trees of bisectors.
    private void Roots(int beginbisector, int noofbisectors)
    {
        roots = new MAT_ListOfBisector();

        if (bisectormap[noofbisectors - 1].BisectorNumber() == -1)
        {
            // OCCT: with a single element (a contour of one closed edge) the main loop never runs, the only bisector
            // is never built and its list is empty; Current() is a null handle, dereferenced (an access violation
            // natively, a NullReferenceException here).
            roots = bisectormap[noofbisectors - 1].List();
            roots.First();
            roots.Current().FirstEdge().Distance(bisectormap[noofbisectors - 1].DistIssuePoint());
        }
        else
        {
            for (int i = beginbisector; i < noofbisectors; i++)
            {
                roots.BackAdd(bisectormap[i]);
            }
        }
    }

    // Loads the bisectors to remove.
    private void LoadBisectorsToRemove(ref int noofbisectorstoremove, double distance1, double distance2,
        MAT_Bisector firstbisectortoremove1, MAT_Bisector firstbisectortoremove2, MAT_Bisector? lastbisectortoremove1,
        MAT_Bisector? lastbisectortoremove2)
    {
        int found, index;
        MAT_Bisector[] firstbisectortoremove = [firstbisectortoremove1, firstbisectortoremove2];
        MAT_Bisector?[] lastbisectortoremove = [lastbisectortoremove1, lastbisectortoremove2];

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
                if (bisectoronetoremove[j].BisectorNumber() == firstbisectortoremove[index].BisectorNumber())
                {
                    found = j;
                    if (bisectortwotoremove[j].BisectorNumber() < lastbisectortoremove[index]!.BisectorNumber())
                    {
                        found = -1;
                    }
                    break;
                }
            }

            if (found != -1)
            {
                bisectoronetoremove.Bind(found, firstbisectortoremove[index]);
                bisectortwotoremove.Bind(found, lastbisectortoremove[index]!);
                typeofbisectortoremove.Bind(found, index + 1);

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
    private void Intersect(MAT2d_Tool2d atool, int aside, ref int noofbisectortoremove, MAT_Bisector firstbisector,
        MAT_Bisector secondbisector)
    {
        int bisectornumber;
        double distant, saveparameter;
        Span<double> distance = stackalloc double[2];
        int intersectionpoint = 0;
        MAT_Bisector lastbisector, previousbisector;
        MAT_Bisector[] firstbisectortoremove = new MAT_Bisector[2];
        MAT_Bisector?[] lastbisectortoremove = new MAT_Bisector?[2];

        distance[0] = global::Kerf.Engine.Foundation.Precision.Infinite();
        distance[1] = global::Kerf.Engine.Foundation.Precision.Infinite();

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
                if (firstbisectortoremove[bisectornumber].List().IsEmpty())
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
            while (!previousbisector.List().IsEmpty())
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
                }

                lastbisector = previousbisector;
            }
        }

        // Loading the bisectors to remove.
        LoadBisectorsToRemove(ref noofbisectortoremove, distance[0], distance[1], firstbisectortoremove[0],
            firstbisectortoremove[1], lastbisectortoremove[0], lastbisectortoremove[1]);
    }

    /// <summary>Starts an iteration over the roots of the trees of bisectors.</summary>
    public void Init() => roots!.First();

    /// <summary>False when there are no more roots.</summary>
    public bool More() => roots!.More();

    /// <summary>Moves to the next root.</summary>
    public void Next() => roots!.Next();

    /// <summary>The current root.</summary>
    public MAT_Bisector Bisector() => roots!.Current();

    /// <summary>True if there are semi-infinite bisectors (then there is a tree per semi-infinite bisector).</summary>
    public bool SemiInfinite() => semiInfinite;

    /// <summary>The total number of bisectors.</summary>
    public int NumberOfBisectors() => thenumberofbisectors;

    /// <summary>True if CreateMat succeeded.</summary>
    public bool IsDone() => isDone;
}
