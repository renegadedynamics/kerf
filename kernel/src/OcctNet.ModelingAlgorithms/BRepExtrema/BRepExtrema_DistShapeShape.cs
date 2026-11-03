// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_DistShapeShape.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_DistShapeShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Occt.BRepExtrema_SupportType;
using static Occt.Extrema_ExtAlgo;
using static Occt.Extrema_ExtFlag;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The minimum distance between two shapes (compound, compsolid, solid, shell, face, wire, edge or vertex): its
/// value, and the pairs of points that realise it with their supports and parameters.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). It only reads the shapes. The supports it returns are sub-shapes of the inputs,
/// except for an infinite edge or face facing a finite one, whose support is a trimmed copy built and frozen by
/// <see cref="BRepExtrema_DistanceSS"/>.
/// </para>
/// <para>
/// OCCT splits the work into tasks by the number of threads of OSD_ThreadPool's default pool, in single-threaded mode
/// too, and the split decides which pairs of sub-shapes are measured and in which order the solutions come. The port
/// takes <c>OSD_Parallel.NbLogicalProcessors()</c> (<see cref="Environment.ProcessorCount"/>), as that pool does
/// (one thread per logical processor), so it splits as native OCCT does on the same machine, serial or parallel. In
/// multi-threaded mode the solid treatment's result depends on which thread finds an inner point first, as in OCCT.
/// </para>
/// </remarks>
public sealed class BRepExtrema_DistShapeShape
{
    private double myDistRef;
    private bool myIsDone;
    private readonly NCollection_Sequence<BRepExtrema_SolutionElem> mySolutionsShape1 = new();
    private readonly NCollection_Sequence<BRepExtrema_SolutionElem> mySolutionsShape2 = new();
    private bool myInnerSol;
    private double myEps;
    private TopoDS_Shape myShape1;
    private TopoDS_Shape myShape2;
    private readonly NCollection_IndexedMap<TopoDS_Shape> myMapV1 = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedMap<TopoDS_Shape> myMapV2 = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedMap<TopoDS_Shape> myMapE1 = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedMap<TopoDS_Shape> myMapE2 = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedMap<TopoDS_Shape> myMapF1 = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedMap<TopoDS_Shape> myMapF2 = new(TopTools_ShapeMapHasher.Instance);
    private bool myIsInitS1;
    private bool myIsInitS2;
#pragma warning disable CS0414 // OCCT keeps the obsolete flag and algorithm without reading them
    private Extrema_ExtFlag myFlag;
    private Extrema_ExtAlgo myAlgo;
#pragma warning restore CS0414
    private readonly NCollection_Array1<Box> myBV1 = new();
    private readonly NCollection_Array1<Box> myBV2 = new();
    private readonly NCollection_Array1<Box> myBE1 = new();
    private readonly NCollection_Array1<Box> myBE2 = new();
    private readonly NCollection_Array1<Box> myBF1 = new();
    private readonly NCollection_Array1<Box> myBF2 = new();
    private bool myIsMultiThread;

    /// <summary>OSD_ThreadPool::DefaultPool()->NbThreads(): one thread per logical processor.</summary>
    private static int NbThreads() => OsdParallel.LogicalProcessorCount;

    //=================================================================================================

    private static void Decomposition(in TopoDS_Shape S,
                                      NCollection_IndexedMap<TopoDS_Shape> MapV,
                                      NCollection_IndexedMap<TopoDS_Shape> MapE,
                                      NCollection_IndexedMap<TopoDS_Shape> MapF)
    {
        MapV.Clear();
        MapE.Clear();
        MapF.Clear();
        TopExp.MapShapes(S, TopAbs_VERTEX, MapV);
        TopExp.MapShapes(S, TopAbs_EDGE, MapE);
        TopExp.MapShapes(S, TopAbs_FACE, MapF);
    }

    private static void BoxCalculation(NCollection_IndexedMap<TopoDS_Shape> Map, NCollection_Array1<Box> SBox)
    {
        for (int i = 1; i <= Map.Extent(); i++)
        {
            var box = new Box();
            BRepBndLib.Add(Map[i], ref box);
            SBox[i] = box;
        }
    }

    private static double DistanceInitiale(in TopoDS_Vertex V1, in TopoDS_Vertex V2)
        => BRep_Tool.Pnt(V1).Distance(BRep_Tool.Pnt(V2));

    /// <summary>A pair of sub-shapes to check, with the distance between their boxes.</summary>
    private readonly record struct BRepExtrema_CheckPair(int Index1, int Index2, double Distance);

    private readonly record struct IndexBand(int First, int Last);

    private sealed class ThreadSolution
    {
        public readonly NCollection_Array1<NCollection_Sequence<BRepExtrema_SolutionElem>> Shape1;
        public readonly NCollection_Array1<NCollection_Sequence<BRepExtrema_SolutionElem>> Shape2;
        public readonly NCollection_Array1<double> Dist;

        public ThreadSolution(int theTaskNum)
        {
            Shape1 = new NCollection_Array1<NCollection_Sequence<BRepExtrema_SolutionElem>>(0, theTaskNum - 1);
            Shape2 = new NCollection_Array1<NCollection_Sequence<BRepExtrema_SolutionElem>>(0, theTaskNum - 1);
            for (int i = 0; i < theTaskNum; i++)
            {
                Shape1[i] = new NCollection_Sequence<BRepExtrema_SolutionElem>();
                Shape2[i] = new NCollection_Sequence<BRepExtrema_SolutionElem>();
            }
            Dist = new NCollection_Array1<double>(0, theTaskNum - 1);
            Dist.Init(double.MaxValue);
        }
    }

    private sealed class VertexFunctor
    {
        public readonly NCollection_Array1<IndexBand> BandArray;
        public readonly ThreadSolution Solution;
        public NCollection_IndexedMap<TopoDS_Shape>? Map1;
        public NCollection_IndexedMap<TopoDS_Shape>? Map2;
        public readonly ProgressScope Scope;
        public readonly NCollection_Array1<ProgressRange> Ranges;
        public double Eps;
        public double StartDist;

        public VertexFunctor(NCollection_Array1<IndexBand> theBandArray, ProgressRange theRange)
        {
            BandArray = theBandArray;
            Solution = new ThreadSolution(theBandArray.Length());
            Scope = new ProgressScope(theRange, "Vertices distances calculating", theBandArray.Length());
            Ranges = new NCollection_Array1<ProgressRange>(0, theBandArray.Length() - 1);
            Eps = global::Kerf.Engine.Foundation.Precision.Confusion();
            StartDist = 0.0;
            for (int i = 0; i < theBandArray.Length(); ++i)
            {
                Ranges.SetValue(i, Scope.Next());
            }
        }

        public void Invoke(int theIndex)
        {
            int aCount2 = Map2!.Extent();
            int aFirst = BandArray.Value(theIndex).First;
            int aLast = BandArray.Value(theIndex).Last;
            Solution.Dist[theIndex] = StartDist;

            using var aScope = new ProgressScope(Ranges[theIndex], null, (double)aLast - aFirst);

            for (int anIdx1 = aFirst; anIdx1 <= aLast; ++anIdx1)
            {
                if (!aScope.More())
                {
                    break;
                }
                aScope.Next();

                TopoDS_Vertex aVertex1 = TopoDS.Vertex(Map1!.FindKey(anIdx1));
                Point3D aPoint1 = BRep_Tool.Pnt(aVertex1);
                for (int anIdx2 = 1; anIdx2 <= aCount2; ++anIdx2)
                {
                    TopoDS_Vertex aVertex2 = TopoDS.Vertex(Map2.FindKey(anIdx2));
                    Point3D aPoint2 = BRep_Tool.Pnt(aVertex2);
                    double aDist = aPoint1.Distance(aPoint2);
                    {
                        if (aDist < Solution.Dist[theIndex] - Eps)
                        {
                            var Sol1 = new BRepExtrema_SolutionElem(aDist, aPoint1, BRepExtrema_IsVertex, aVertex1);
                            var Sol2 = new BRepExtrema_SolutionElem(aDist, aPoint2, BRepExtrema_IsVertex, aVertex2);

                            Solution.Shape1[theIndex].Clear();
                            Solution.Shape2[theIndex].Clear();
                            Solution.Shape1[theIndex].Append(Sol1);
                            Solution.Shape2[theIndex].Append(Sol2);

                            Solution.Dist[theIndex] = aDist;
                        }
                        else if (Math.Abs(aDist - Solution.Dist[theIndex]) < Eps)
                        {
                            var Sol1 = new BRepExtrema_SolutionElem(aDist, aPoint1, BRepExtrema_IsVertex, aVertex1);
                            var Sol2 = new BRepExtrema_SolutionElem(aDist, aPoint2, BRepExtrema_IsVertex, aVertex2);
                            Solution.Shape1[theIndex].Append(Sol1);
                            Solution.Shape2[theIndex].Append(Sol2);

                            if (Solution.Dist[theIndex] > aDist)
                            {
                                Solution.Dist[theIndex] = aDist;
                            }
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    private bool DistanceVertVert(NCollection_IndexedMap<TopoDS_Shape> theMap1,
                                  NCollection_IndexedMap<TopoDS_Shape> theMap2,
                                  ProgressRange theRange)
    {
        int aCount1 = theMap1.Extent();
        int aMinTaskSize = aCount1 < 10 ? aCount1 : 10;
        int aNbThreads = NbThreads();
        int aNbTasks = aNbThreads;
        int aTaskSize = (int)Math.Ceiling((double)aCount1 / aNbTasks);
        if (aTaskSize < aMinTaskSize)
        {
            aTaskSize = aMinTaskSize;
            aNbTasks = (int)Math.Ceiling((double)aCount1 / aTaskSize);
        }

        int aFirstIndex = 1;
        var aBandArray = new NCollection_Array1<IndexBand>(0, aNbTasks - 1);
        using var aDistScope = new ProgressScope(theRange, null, 1);

        for (int anI = 0; anI < aBandArray.Length(); ++anI)
        {
            if (aCount1 < aFirstIndex + aTaskSize - 1)
            {
                aTaskSize = aCount1 - aFirstIndex + 1;
            }
            aBandArray.SetValue(anI, new IndexBand(aFirstIndex, aFirstIndex + aTaskSize - 1));
            aFirstIndex += aTaskSize;
        }

        var aFunctor = new VertexFunctor(aBandArray, aDistScope.Next());
        aFunctor.Map1 = theMap1;
        aFunctor.Map2 = theMap2;
        aFunctor.StartDist = myDistRef;
        aFunctor.Eps = myEps;

        OsdParallel.For(0, aNbTasks, aFunctor.Invoke, !myIsMultiThread);
        aFunctor.Scope.Close();
        if (!aDistScope.More())
        {
            return false;
        }
        for (int anI = 0; anI < aFunctor.Solution.Dist.Length(); ++anI)
        {
            double aDist = aFunctor.Solution.Dist[anI];
            if (aDist < myDistRef - myEps)
            {
                mySolutionsShape1.Clear();
                mySolutionsShape2.Clear();
                mySolutionsShape1.Append(aFunctor.Solution.Shape1[anI]);
                mySolutionsShape2.Append(aFunctor.Solution.Shape2[anI]);
                myDistRef = aDist;
            }
            else if (Math.Abs(aDist - myDistRef) < myEps)
            {
                mySolutionsShape1.Append(aFunctor.Solution.Shape1[anI]);
                mySolutionsShape2.Append(aFunctor.Solution.Shape2[anI]);
                myDistRef = aDist;
            }
        }
        return true;
    }

    //=================================================================================================

    private sealed class DistanceFunctor
    {
        public readonly NCollection_Array1<NCollection_Array1<BRepExtrema_CheckPair>> ArrayOfArrays;
        public readonly ThreadSolution Solution;
        public NCollection_IndexedMap<TopoDS_Shape>? Map1;
        public NCollection_IndexedMap<TopoDS_Shape>? Map2;
        public NCollection_Array1<Box>? LBox1;
        public NCollection_Array1<Box>? LBox2;
        public readonly ProgressScope Scope;
        public readonly NCollection_Array1<ProgressRange> Ranges;
        public double Eps;
        public double StartDist;

        public DistanceFunctor(NCollection_Array1<NCollection_Array1<BRepExtrema_CheckPair>> theArrayOfArrays,
                               ProgressRange theRange)
        {
            ArrayOfArrays = theArrayOfArrays;
            Solution = new ThreadSolution(ArrayOfArrays.Length());
            Scope = new ProgressScope(theRange, "Shapes distances calculating", theArrayOfArrays.Length());
            Ranges = new NCollection_Array1<ProgressRange>(0, theArrayOfArrays.Length() - 1);
            Eps = global::Kerf.Engine.Foundation.Precision.Confusion();
            StartDist = 0.0;
            for (int i = 0; i < theArrayOfArrays.Length(); ++i)
            {
                Ranges.SetValue(i, Scope.Next());
            }
        }

        public void Invoke(int theIndex)
        {
            using var aScope = new ProgressScope(Ranges[theIndex], null, ArrayOfArrays.Value(theIndex).Length());
            Solution.Dist[theIndex] = StartDist;
            for (int i = 0; i < ArrayOfArrays.Value(theIndex).Length(); i++)
            {
                if (!aScope.More())
                {
                    return;
                }
                aScope.Next();
                BRepExtrema_CheckPair aPair = ArrayOfArrays.Value(theIndex).Value(i);
                if (aPair.Distance > Solution.Dist[theIndex] + Eps)
                {
                    break; // early search termination
                }
                Box aBox1 = LBox1!.Value(aPair.Index1);
                Box aBox2 = LBox2!.Value(aPair.Index2);
                TopoDS_Shape aShape1 = Map1!.FindKey(aPair.Index1);
                TopoDS_Shape aShape2 = Map2!.FindKey(aPair.Index2);
                var aDistTool = new BRepExtrema_DistanceSS(aShape1, aShape2, aBox1, aBox2, Solution.Dist[theIndex], Eps);
                double aDist = aDistTool.DistValue();
                if (aDistTool.IsDone())
                {
                    if (aDist < Solution.Dist[theIndex] - Eps)
                    {
                        Solution.Shape1[theIndex].Clear();
                        Solution.Shape2[theIndex].Clear();

                        var aSeq1 = new NCollection_Sequence<BRepExtrema_SolutionElem>(aDistTool.InternalSeq1Value());
                        var aSeq2 = new NCollection_Sequence<BRepExtrema_SolutionElem>(aDistTool.InternalSeq2Value());

                        Solution.Shape1[theIndex].Append(aSeq1);
                        Solution.Shape2[theIndex].Append(aSeq2);

                        Solution.Dist[theIndex] = aDistTool.DistValue();
                    }
                    else if (Math.Abs(aDist - Solution.Dist[theIndex]) < Eps)
                    {
                        var aSeq1 = new NCollection_Sequence<BRepExtrema_SolutionElem>(aDistTool.InternalSeq1Value());
                        var aSeq2 = new NCollection_Sequence<BRepExtrema_SolutionElem>(aDistTool.InternalSeq2Value());

                        Solution.Shape1[theIndex].Append(aSeq1);
                        Solution.Shape2[theIndex].Append(aSeq2);
                        if (Solution.Dist[theIndex] > aDist)
                        {
                            Solution.Dist[theIndex] = aDist;
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    private sealed class DistancePairFunctor
    {
        public readonly NCollection_Array1<IndexBand> BandArray;
        public readonly NCollection_Array1<NCollection_DynamicArray<BRepExtrema_CheckPair>> PairList;
        public NCollection_Array1<Box>? LBox1;
        public NCollection_Array1<Box>? LBox2;
        public readonly ProgressScope Scope;
        public readonly NCollection_Array1<ProgressRange> Ranges;
        public double DistRef;
        public double Eps;

        public DistancePairFunctor(NCollection_Array1<IndexBand> theBandArray, ProgressRange theRange)
        {
            BandArray = theBandArray;
            PairList = new NCollection_Array1<NCollection_DynamicArray<BRepExtrema_CheckPair>>(0, theBandArray.Length() - 1);
            for (int i = 0; i < theBandArray.Length(); ++i)
            {
                PairList[i] = new NCollection_DynamicArray<BRepExtrema_CheckPair>();
            }
            Scope = new ProgressScope(theRange, "Boxes distances calculating", theBandArray.Length());
            Ranges = new NCollection_Array1<ProgressRange>(0, theBandArray.Length() - 1);
            DistRef = 0;
            Eps = global::Kerf.Engine.Foundation.Precision.Confusion();
            for (int i = 0; i < theBandArray.Length(); ++i)
            {
                Ranges.SetValue(i, Scope.Next());
            }
        }

        public void Invoke(int theIndex)
        {
            int aFirst = BandArray.Value(theIndex).First;
            int aLast = BandArray.Value(theIndex).Last;

            using var aScope = new ProgressScope(Ranges[theIndex], null, (double)aLast - aFirst);

            for (int anIdx1 = aFirst; anIdx1 <= aLast; ++anIdx1)
            {
                if (!aScope.More())
                {
                    break;
                }
                aScope.Next();

                for (int anIdx2 = 1; anIdx2 <= LBox2!.Length(); ++anIdx2)
                {
                    Box aBox1 = LBox1!.Value(anIdx1);
                    Box aBox2 = LBox2.Value(anIdx2);
                    if (aBox1.IsVoid || aBox2.IsVoid)
                    {
                        continue;
                    }

                    double aDist = aBox1.Distance(aBox2);
                    if (aDist - DistRef < Eps)
                    {
                        PairList[theIndex].Append(new BRepExtrema_CheckPair(anIdx1, anIdx2, aDist));
                    }
                }
            }
        }

        public int ListSize()
        {
            int aSize = 0;
            for (int anI = PairList.Lower(); anI <= PairList.Upper(); ++anI)
            {
                aSize += PairList[anI].Length();
            }
            return aSize;
        }
    }

    //=================================================================================================

    private bool DistanceMapMap(NCollection_IndexedMap<TopoDS_Shape> theMap1,
                                NCollection_IndexedMap<TopoDS_Shape> theMap2,
                                NCollection_Array1<Box> theLBox1,
                                NCollection_Array1<Box> theLBox2,
                                ProgressRange theRange)
    {
        int aCount1 = theMap1.Extent();
        int aCount2 = theMap2.Extent();

        if (aCount1 == 0 || aCount2 == 0)
        {
            return true;
        }

        using var aTwinScope = new ProgressScope(theRange, null, 1.0);

        int aNbThreads = NbThreads();
        int aMinPairTaskSize = aCount1 < 10 ? aCount1 : 10;
        int aNbPairTasks = aNbThreads;
        int aPairTaskSize = (int)Math.Ceiling((double)aCount1 / aNbPairTasks);
        if (aPairTaskSize < aMinPairTaskSize)
        {
            aPairTaskSize = aMinPairTaskSize;
            aNbPairTasks = (int)Math.Ceiling((double)aCount1 / aPairTaskSize);
        }

        int aFirstIndex = 1;
        var aBandArray = new NCollection_Array1<IndexBand>(0, aNbPairTasks - 1);

        for (int anI = 0; anI < aBandArray.Length(); ++anI)
        {
            if (aCount1 < aFirstIndex + aPairTaskSize - 1)
            {
                aPairTaskSize = aCount1 - aFirstIndex + 1;
            }
            aBandArray.SetValue(anI, new IndexBand(aFirstIndex, aFirstIndex + aPairTaskSize - 1));
            aFirstIndex += aPairTaskSize;
        }

        aTwinScope.Next(0.15);
        var aPairFunctor = new DistancePairFunctor(aBandArray, aTwinScope.Next(0.15));
        aPairFunctor.LBox1 = theLBox1;
        aPairFunctor.LBox2 = theLBox2;
        aPairFunctor.DistRef = myDistRef;
        aPairFunctor.Eps = myEps;

        OsdParallel.For(0, aNbPairTasks, aPairFunctor.Invoke, !myIsMultiThread);
        aPairFunctor.Scope.Close();
        if (!aTwinScope.More())
        {
            return false;
        }
        int aListSize = aPairFunctor.ListSize();
        if (aListSize == 0)
        {
            return true;
        }
        var aPairList = new BRepExtrema_CheckPair[aListSize];
        int aListIndex = 0;
        for (int anI = 0; anI < aPairFunctor.PairList.Length(); ++anI)
        {
            for (int aJ = 0; aJ < aPairFunctor.PairList[anI].Length(); ++aJ)
            {
                aPairList[aListIndex] = aPairFunctor.PairList[anI][aJ];
                ++aListIndex;
            }
        }

        // std::stable_sort by the distance (LINQ's OrderBy is a stable sort)
        aPairList = [.. aPairList.OrderBy(static thePair => thePair.Distance)];

        int aMapSize = aPairList.Length;
        int aNbTasks = aMapSize < aNbThreads ? aMapSize : aNbThreads;
        int aTaskSize = (int)Math.Ceiling((double)aMapSize / aNbTasks);

        var anArrayOfArray = new NCollection_Array1<NCollection_Array1<BRepExtrema_CheckPair>>(0, aNbTasks - 1);
        for (int anI = 0; anI < aNbTasks; ++anI)
        {
            anArrayOfArray[anI] = new NCollection_Array1<BRepExtrema_CheckPair>();
        }
        // Since aPairList is sorted in ascending order of distances between Bnd_Boxes,
        // BRepExtrema_CheckPair are distributed to tasks one by one from smallest to largest,
        // and not ranges, as for DistancePairFunctor.
        // Since aMapSize may not be divisible entirely by the number of tasks,
        // some tasks should receive one BRepExtrema_CheckPair less than the rest.
        // aLastRowLimit defines the task number from which to start tasks containing
        // fewer BRepExtrema_CheckPair
        int aLastRowLimit = ((aMapSize % aNbTasks) == 0) ? aNbTasks : (aMapSize % aNbTasks);
        for (int anI = 0; anI < aTaskSize; ++anI)
        {
            for (int aJ = 0; aJ < aNbTasks; ++aJ)
            {
                if (anI == 0)
                {
                    int aVectorSize = aTaskSize;
                    if (aJ >= aLastRowLimit)
                    {
                        aVectorSize--;
                    }
                    anArrayOfArray[aJ].Resize(0, aVectorSize - 1, false);
                }
                if (anI < anArrayOfArray[aJ].Length())
                {
                    anArrayOfArray[aJ][anI] = aPairList[anI * aNbTasks + aJ];
                }
                else
                {
                    break;
                }
            }
        }
        var aFunctor = new DistanceFunctor(anArrayOfArray, aTwinScope.Next(0.85));
        aFunctor.Map1 = theMap1;
        aFunctor.Map2 = theMap2;
        aFunctor.LBox1 = theLBox1;
        aFunctor.LBox2 = theLBox2;
        aFunctor.Eps = myEps;
        aFunctor.StartDist = myDistRef;

        OsdParallel.For(0, aNbTasks, aFunctor.Invoke, !myIsMultiThread);
        aFunctor.Scope.Close();
        if (!aTwinScope.More())
        {
            return false;
        }

        for (int anI = 0; anI < aFunctor.Solution.Dist.Length(); ++anI)
        {
            double aDist = aFunctor.Solution.Dist[anI];
            if (aDist < myDistRef - myEps)
            {
                mySolutionsShape1.Clear();
                mySolutionsShape2.Clear();
                mySolutionsShape1.Append(aFunctor.Solution.Shape1[anI]);
                mySolutionsShape2.Append(aFunctor.Solution.Shape2[anI]);
                myDistRef = aDist;
            }
            else if (Math.Abs(aDist - myDistRef) < myEps)
            {
                mySolutionsShape1.Append(aFunctor.Solution.Shape1[anI]);
                mySolutionsShape2.Append(aFunctor.Solution.Shape2[anI]);
                if (myDistRef > aDist)
                {
                    myDistRef = aDist;
                }
            }
        }
        return true;
    }

    //=================================================================================================

    /// <summary>An empty tool.</summary>
    public BRepExtrema_DistShapeShape()
    {
        myDistRef = 0.0;
        myIsDone = false;
        myInnerSol = false;
        myEps = global::Kerf.Engine.Foundation.Precision.Confusion();
        myIsInitS1 = false;
        myIsInitS2 = false;
        myFlag = Extrema_ExtFlag_MINMAX;
        myAlgo = Extrema_ExtAlgo_Grad;
        myIsMultiThread = false;
    }

    /// <summary>
    /// Computes the minimum distance between the shapes (value and pairs of points) with the default deflection
    /// (Precision::Confusion()) in single-threaded mode. <paramref name="F"/> and <paramref name="A"/> are obsolete
    /// and unused.
    /// </summary>
    public BRepExtrema_DistShapeShape(in TopoDS_Shape Shape1,
                                      in TopoDS_Shape Shape2,
                                      Extrema_ExtFlag F = Extrema_ExtFlag_MINMAX,
                                      Extrema_ExtAlgo A = Extrema_ExtAlgo_Grad,
                                      ProgressRange theRange = default)
    {
        myDistRef = 0.0;
        myIsDone = false;
        myInnerSol = false;
        myEps = global::Kerf.Engine.Foundation.Precision.Confusion();
        myIsInitS1 = false;
        myIsInitS2 = false;
        myFlag = F;
        myAlgo = A;
        myIsMultiThread = false;
        LoadS1(Shape1);
        LoadS2(Shape2);
        Perform(theRange);
    }

    /// <summary>
    /// Computes the minimum distance between the shapes in single-threaded mode; solutions within
    /// <paramref name="theDeflection"/> of the minimum are kept. <paramref name="F"/> and <paramref name="A"/> are
    /// obsolete and unused.
    /// </summary>
    public BRepExtrema_DistShapeShape(in TopoDS_Shape Shape1,
                                      in TopoDS_Shape Shape2,
                                      double theDeflection,
                                      Extrema_ExtFlag F = Extrema_ExtFlag_MINMAX,
                                      Extrema_ExtAlgo A = Extrema_ExtAlgo_Grad,
                                      ProgressRange theRange = default)
    {
        myDistRef = 0.0;
        myIsDone = false;
        myInnerSol = false;
        myEps = theDeflection;
        myIsInitS1 = false;
        myIsInitS2 = false;
        myFlag = F;
        myAlgo = A;
        myIsMultiThread = false;
        LoadS1(Shape1);
        LoadS2(Shape2);
        Perform(theRange);
    }

    /// <summary>Sets the deflection: the largest deviation of the kept solutions from the minimum distance.</summary>
    public void SetDeflection(double theDeflection) => myEps = theDeflection;

    /// <summary>Loads the first shape.</summary>
    public void LoadS1(in TopoDS_Shape Shape1)
    {
        myShape1 = Shape1;
        myIsInitS1 = false;
        Decomposition(Shape1, myMapV1, myMapE1, myMapF1);
    }

    /// <summary>Loads the second shape.</summary>
    public void LoadS2(in TopoDS_Shape Shape2)
    {
        myShape2 = Shape2;
        myIsInitS2 = false;
        Decomposition(Shape2, myMapV2, myMapE2, myMapF2);
    }

    //=================================================================================================

    private sealed class TreatmentFunctor
    {
        public readonly NCollection_Array1<NCollection_Array1<TopoDS_Shape>> ArrayOfArrays;
        public NCollection_Sequence<BRepExtrema_SolutionElem>? SolutionsShape1;
        public NCollection_Sequence<BRepExtrema_SolutionElem>? SolutionsShape2;
        public TopoDS_Shape Shape;
        public readonly ProgressScope Scope;
        public readonly NCollection_Array1<ProgressRange> Ranges;
        public double DistRef;
        public volatile bool InnerSol;
        public volatile bool IsDone;
        public Lock? Mutex;

        public TreatmentFunctor(NCollection_Array1<NCollection_Array1<TopoDS_Shape>> theArrayOfArrays,
                                ProgressRange theRange)
        {
            ArrayOfArrays = theArrayOfArrays;
            Scope = new ProgressScope(theRange, "Search for the inner solid", theArrayOfArrays.Length());
            Ranges = new NCollection_Array1<ProgressRange>(0, theArrayOfArrays.Length() - 1);
            for (int i = 0; i < theArrayOfArrays.Length(); ++i)
            {
                Ranges.SetValue(i, Scope.Next());
            }
        }

        private void Record(in Point3D aPnt, in TopoDS_Vertex aVertex)
        {
            InnerSol = true;
            DistRef = 0.0;
            IsDone = true;
            var aSolElem = new BRepExtrema_SolutionElem(0, aPnt, BRepExtrema_IsVertex, aVertex);
            SolutionsShape1!.Append(aSolElem);
            SolutionsShape2!.Append(aSolElem);
        }

        public void Invoke(int theIndex)
        {
            const double aTolerance = 0.001;
            using var aScope = new ProgressScope(Ranges[theIndex], null, ArrayOfArrays.Value(theIndex).Length());
            var aClassifier = new BRepClass3d_SolidClassifier(Shape);

            for (int i = 0; i < ArrayOfArrays.Value(theIndex).Length(); i++)
            {
                if (!aScope.More())
                {
                    break;
                }
                aScope.Next();
                if (IsDone)
                {
                    break;
                }

                TopoDS_Vertex aVertex = TopoDS.Vertex(ArrayOfArrays.Value(theIndex).Value(i));
                Point3D aPnt = BRep_Tool.Pnt(aVertex);
                aClassifier.Perform(aPnt, aTolerance);
                if (aClassifier.State() == TopAbs_IN)
                {
                    if (Mutex is not null)
                    {
                        lock (Mutex)
                        {
                            Record(aPnt, aVertex);
                        }
                    }
                    else
                    {
                        Record(aPnt, aVertex);
                    }
                    break;
                }
            }
        }
    }

    //=================================================================================================

    private bool SolidTreatment(in TopoDS_Shape theShape,
                                NCollection_IndexedMap<TopoDS_Shape> theVertexMap,
                                ProgressRange theRange)
    {
        int aMapSize = theVertexMap.Extent();
        const int aMinTaskSize = 3;
        int aNbThreads = NbThreads();
        int aNbTasks = aNbThreads * 10;
        int aTaskSize = (int)Math.Ceiling((double)aMapSize / aNbTasks);
        if (aTaskSize < aMinTaskSize)
        {
            aTaskSize = aMinTaskSize;
            aNbTasks = (int)Math.Ceiling((double)aMapSize / aTaskSize);
        }

        var anArrayOfArray = new NCollection_Array1<NCollection_Array1<TopoDS_Shape>>(0, aNbTasks - 1);
        for (int anI = 0; anI < aNbTasks; ++anI)
        {
            anArrayOfArray[anI] = new NCollection_Array1<TopoDS_Shape>();
        }
        for (int anI = 1; anI <= aMapSize; ++anI)
        {
            int aVectIndex = (anI - 1) / aTaskSize;
            int aShapeIndex = (anI - 1) % aTaskSize;
            if (aShapeIndex == 0)
            {
                int aVectorSize = aTaskSize;
                int aTailSize = aMapSize - aVectIndex * aTaskSize;
                if (aTailSize < aTaskSize)
                {
                    aVectorSize = aTailSize;
                }
                anArrayOfArray[aVectIndex].Resize(0, aVectorSize - 1, false);
            }
            anArrayOfArray[aVectIndex][aShapeIndex] = theVertexMap[anI];
        }

        // Local variables for thread-safe communication during the parallel section (OCCT's atomics)
        using var aScope = new ProgressScope(theRange, "Solid treatment", aNbTasks);
        var aFunctor = new TreatmentFunctor(anArrayOfArray, aScope.Next());
        aFunctor.SolutionsShape1 = mySolutionsShape1;
        aFunctor.SolutionsShape2 = mySolutionsShape2;
        aFunctor.Shape = theShape;
        aFunctor.DistRef = myDistRef;
        aFunctor.InnerSol = myInnerSol;
        aFunctor.IsDone = myIsDone;
        if (myIsMultiThread && aFunctor.Mutex is null)
        {
            aFunctor.Mutex = new Lock();
        }

        OsdParallel.For(0, aNbTasks, aFunctor.Invoke, !myIsMultiThread);
        aFunctor.Scope.Close();

        // Copy the results back to the class members after the parallel section completes
        myDistRef = aFunctor.DistRef;
        myInnerSol = aFunctor.InnerSol;
        myIsDone = aFunctor.IsDone;

        return aScope.More();
    }

    //=================================================================================================

    /// <summary>
    /// Computes the minimum distance (value and pairs of points); solutions within the deflection of the minimum are
    /// kept. Returns <see cref="IsDone"/>.
    /// </summary>
    public bool Perform(ProgressRange theRange = default)
    {
        myIsDone = false;
        myInnerSol = false;
        mySolutionsShape1.Clear();
        mySolutionsShape2.Clear();

        if (myShape1.IsNull() || myShape2.IsNull())
        {
            return false;
        }

        // Treatment of solids
        bool anIsSolid1 = (myShape1.ShapeType() == TopAbs_SOLID) || (myShape1.ShapeType() == TopAbs_COMPSOLID);
        bool anIsSolid2 = (myShape2.ShapeType() == TopAbs_SOLID) || (myShape2.ShapeType() == TopAbs_COMPSOLID);
        int aRootStepsNum = 9; // By num of DistanceMapMap calls
        aRootStepsNum = anIsSolid1 ? aRootStepsNum + 1 : aRootStepsNum;
        aRootStepsNum = anIsSolid2 ? aRootStepsNum + 1 : aRootStepsNum;
        using var aRootScope = new ProgressScope(theRange, "calculating distance", aRootStepsNum);

        if (anIsSolid1)
        {
            if (!SolidTreatment(myShape1, myMapV2, aRootScope.Next()))
            {
                return false;
            }
        }

        if (anIsSolid2 && (!myInnerSol))
        {
            if (!SolidTreatment(myShape2, myMapV1, aRootScope.Next()))
            {
                return false;
            }
        }

        if (!myInnerSol)
        {
            if (!myIsInitS1) // rebuild cached data for 1st shape
            {
                if (!myMapV1.IsEmpty())
                {
                    myBV1.Resize(1, myMapV1.Extent(), false);
                }
                if (!myMapE1.IsEmpty())
                {
                    myBE1.Resize(1, myMapE1.Extent(), false);
                }
                if (!myMapF1.IsEmpty())
                {
                    myBF1.Resize(1, myMapF1.Extent(), false);
                }

                BoxCalculation(myMapV1, myBV1);
                BoxCalculation(myMapE1, myBE1);
                BoxCalculation(myMapF1, myBF1);

                myIsInitS1 = true;
            }

            if (!myIsInitS2) // rebuild cached data for 2nd shape
            {
                if (!myMapV2.IsEmpty())
                {
                    myBV2.Resize(1, myMapV2.Extent(), false);
                }
                if (!myMapE2.IsEmpty())
                {
                    myBE2.Resize(1, myMapE2.Extent(), false);
                }
                if (!myMapF2.IsEmpty())
                {
                    myBF2.Resize(1, myMapF2.Extent(), false);
                }

                BoxCalculation(myMapV2, myBV2);
                BoxCalculation(myMapE2, myBE2);
                BoxCalculation(myMapF2, myBF2);

                myIsInitS2 = true;
            }

            if (myMapV1.Extent() != 0 && myMapV2.Extent() != 0)
            {
                TopoDS_Vertex V1 = TopoDS.Vertex(myMapV1[1]);
                TopoDS_Vertex V2 = TopoDS.Vertex(myMapV2[1]);
                myDistRef = DistanceInitiale(V1, V2);
            }
            else
            {
                myDistRef = 1.0e30; // szv:!!!
            }

            if (!DistanceVertVert(myMapV1, myMapV2, aRootScope.Next()))
            {
                return false;
            }
            if (!DistanceMapMap(myMapV1, myMapE2, myBV1, myBE2, aRootScope.Next()))
            {
                return false;
            }
            if (!DistanceMapMap(myMapE1, myMapV2, myBE1, myBV2, aRootScope.Next()))
            {
                return false;
            }
            if (!DistanceMapMap(myMapV1, myMapF2, myBV1, myBF2, aRootScope.Next()))
            {
                return false;
            }
            if (!DistanceMapMap(myMapF1, myMapV2, myBF1, myBV2, aRootScope.Next()))
            {
                return false;
            }
            if (!DistanceMapMap(myMapE1, myMapE2, myBE1, myBE2, aRootScope.Next()))
            {
                return false;
            }
            if (!DistanceMapMap(myMapE1, myMapF2, myBE1, myBF2, aRootScope.Next()))
            {
                return false;
            }
            if (!DistanceMapMap(myMapF1, myMapE2, myBF1, myBE2, aRootScope.Next()))
            {
                return false;
            }

            if (Math.Abs(myDistRef) > myEps)
            {
                if (!DistanceMapMap(myMapF1, myMapF2, myBF1, myBF2, aRootScope.Next()))
                {
                    return false;
                }
            }

            //  Modified by Sergey KHROMOV - Tue Mar  6 11:55:03 2001 Begin
            // OCCT: the index isn't stepped back after a removal, so the solution after a removed one is not checked
            // (occt-issues.md).
            int i = 1;
            for (; i <= mySolutionsShape1.Length(); i++)
            {
                if (mySolutionsShape1.Value(i).Dist() > myDistRef + myEps)
                {
                    mySolutionsShape1.Remove(i);
                    mySolutionsShape2.Remove(i);
                }
            }
            //  Modified by Sergey KHROMOV - Tue Mar  6 11:55:04 2001 End
            myIsDone = (mySolutionsShape1.Length() > 0);
        }

        return myIsDone;
    }

    /// <summary>True if the minimum distance is found.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>The number of solutions satisfying the minimum distance.</summary>
    public int NbSolution() => mySolutionsShape1.Length();

    /// <summary>The minimum distance; throws StdFail_NotDone if there is no solution.</summary>
    public double Value()
    {
        if (!myIsDone)
        {
            throw new NotDoneException("BRepExtrema_DistShapeShape::Value: There's no solution ");
        }

        return myDistRef;
    }

    /// <summary>True if one of the shapes is a solid and the other is completely or partially inside it.</summary>
    public bool InnerSolution() => myInnerSol;

    /// <summary>The point of the <paramref name="N"/>th solution on the first shape.</summary>
    public Point3D PointOnShape1(int N) => mySolutionsShape1.Value(N).Point();

    /// <summary>The point of the <paramref name="N"/>th solution on the second shape.</summary>
    public Point3D PointOnShape2(int N) => mySolutionsShape2.Value(N).Point();

    /// <summary>The kind of support of the <paramref name="N"/>th solution on the first shape.</summary>
    public BRepExtrema_SupportType SupportTypeShape1(int N) => mySolutionsShape1.Value(N).SupportKind();

    /// <summary>The kind of support of the <paramref name="N"/>th solution on the second shape.</summary>
    public BRepExtrema_SupportType SupportTypeShape2(int N) => mySolutionsShape2.Value(N).SupportKind();

    /// <summary>The support (a vertex, an edge or a face) of the <paramref name="N"/>th solution on the first shape.</summary>
    public TopoDS_Shape SupportOnShape1(int N)
    {
        if (!myIsDone)
        {
            throw new NotDoneException("BRepExtrema_DistShapeShape::SupportOnShape1: There's no solution ");
        }

        BRepExtrema_SolutionElem sol = mySolutionsShape1.Value(N);
        switch (sol.SupportKind())
        {
            case BRepExtrema_IsVertex:
                return sol.Vertex();
            case BRepExtrema_IsOnEdge:
                return sol.Edge();
            case BRepExtrema_IsInFace:
                return sol.Face();
        }
        return new TopoDS_Shape();
    }

    /// <summary>The support (a vertex, an edge or a face) of the <paramref name="N"/>th solution on the second shape.</summary>
    public TopoDS_Shape SupportOnShape2(int N)
    {
        if (!myIsDone)
        {
            throw new NotDoneException("BRepExtrema_DistShapeShape::SupportOnShape2: There's no solution ");
        }

        BRepExtrema_SolutionElem sol = mySolutionsShape2.Value(N);
        switch (sol.SupportKind())
        {
            case BRepExtrema_IsVertex:
                return sol.Vertex();
            case BRepExtrema_IsOnEdge:
                return sol.Edge();
            case BRepExtrema_IsInFace:
                return sol.Face();
        }
        return new TopoDS_Shape();
    }

    /// <summary>The parameter of the <paramref name="N"/>th solution on an edge of the first shape.</summary>
    public void ParOnEdgeS1(int N, out double t)
    {
        if (!myIsDone)
        {
            throw new NotDoneException("BRepExtrema_DistShapeShape::ParOnEdgeS1: There's no solution");
        }

        BRepExtrema_SolutionElem sol = mySolutionsShape1.Value(N);
        if (sol.SupportKind() != BRepExtrema_IsOnEdge)
        {
            throw new BRepExtrema_UnCompatibleShape(
                "BRepExtrema_DistShapeShape::ParOnEdgeS1: ParOnEdgeS1 is impossible without EDGE");
        }

        sol.EdgeParameter(out t);
    }

    /// <summary>The parameter of the <paramref name="N"/>th solution on an edge of the second shape.</summary>
    public void ParOnEdgeS2(int N, out double t)
    {
        if (!myIsDone)
        {
            throw new NotDoneException("BRepExtrema_DistShapeShape::ParOnEdgeS2: There's no solution");
        }

        BRepExtrema_SolutionElem sol = mySolutionsShape2.Value(N);
        if (sol.SupportKind() != BRepExtrema_IsOnEdge)
        {
            throw new BRepExtrema_UnCompatibleShape(
                "BRepExtrema_DistShapeShape::ParOnEdgeS2: ParOnEdgeS2 is impossible without EDGE");
        }

        sol.EdgeParameter(out t);
    }

    /// <summary>The parameters of the <paramref name="N"/>th solution in a face of the first shape.</summary>
    public void ParOnFaceS1(int N, out double u, out double v)
    {
        if (!myIsDone)
        {
            throw new NotDoneException("BRepExtrema_DistShapeShape::ParOnFaceS1: There's no solution");
        }

        BRepExtrema_SolutionElem sol = mySolutionsShape1.Value(N);
        if (sol.SupportKind() != BRepExtrema_IsInFace)
        {
            throw new BRepExtrema_UnCompatibleShape(
                "BRepExtrema_DistShapeShape::ParOnFaceS1: ParOnFaceS1 is impossible without FACE");
        }

        sol.FaceParameter(out u, out v);
    }

    /// <summary>The parameters of the <paramref name="N"/>th solution in a face of the second shape.</summary>
    public void ParOnFaceS2(int N, out double u, out double v)
    {
        if (!myIsDone)
        {
            throw new NotDoneException("BRepExtrema_DistShapeShape::ParOnFaceS2: There's no solution");
        }

        BRepExtrema_SolutionElem sol = mySolutionsShape2.Value(N);
        if (sol.SupportKind() != BRepExtrema_IsInFace)
        {
            throw new BRepExtrema_UnCompatibleShape(
                "BRepExtrema_DistShapeShape::ParOnFaceS2:ParOnFaceS2 is impossible without FACE ");
        }

        sol.FaceParameter(out u, out v);
    }

    /// <summary>Writes the state of the object (OCCT's Dump; reals at the stream's default precision, 6 digits).</summary>
    public void Dump(TextWriter o)
    {
        int i;
        double r1, r2;

        o.Write("the distance  value is :  " + F(Value()) + '\n');
        o.Write("the number of solutions is :" + NbSolution() + '\n');
        o.Write('\n');
        for (i = 1; i <= NbSolution(); i++)
        {
            o.Write("solution number " + i + ": " + '\n');
            o.Write("the type of the solution on the first shape is " + (int)SupportTypeShape1(i) + '\n');
            o.Write("the type of the solution on the second shape is " + (int)SupportTypeShape2(i) + '\n');
            o.Write("the coordinates of  the point on the first shape are: " + '\n');
            o.Write("X=" + F(PointOnShape1(i).X()) + " Y=" + F(PointOnShape1(i).Y()) + " Z=" + F(PointOnShape1(i).Z()) + '\n');
            o.Write("the coordinates of  the point on the second shape are: " + '\n');
            o.Write("X=" + F(PointOnShape2(i).X()) + " Y=" + F(PointOnShape2(i).Y()) + " Z=" + F(PointOnShape2(i).Z()) + '\n');

            switch (SupportTypeShape1(i))
            {
                case BRepExtrema_IsVertex:
                    break;
                case BRepExtrema_IsOnEdge:
                    ParOnEdgeS1(i, out r1);
                    o.Write("parameter on the first edge :  t= " + F(r1) + '\n');
                    break;
                case BRepExtrema_IsInFace:
                    ParOnFaceS1(i, out r1, out r2);
                    o.Write("parameters on the first face :  u= " + F(r1) + " v=" + F(r2) + '\n');
                    break;
            }
            switch (SupportTypeShape2(i))
            {
                case BRepExtrema_IsVertex:
                    break;
                case BRepExtrema_IsOnEdge:
                    ParOnEdgeS2(i, out r1);
                    o.Write("parameter on the second edge : t=" + F(r1) + '\n');
                    break;
                case BRepExtrema_IsInFace:
                    ParOnFaceS2(i, out r1, out r2);
                    o.Write("parameters on the second face : u= " + F(r1) + " v=" + F(r2) + '\n');
                    break;
            }
            o.Write('\n');
        }

        static string F(double theValue) => StreamUtilities.FormatReal(theValue, 6);
    }

    /// <summary>Sets an unused parameter (obsolete).</summary>
    public void SetFlag(Extrema_ExtFlag F) => myFlag = F;

    /// <summary>Sets an unused parameter (obsolete).</summary>
    public void SetAlgo(Extrema_ExtAlgo A) => myAlgo = A;

    /// <summary>Computes in parallel when true (the default is false).</summary>
    public void SetMultiThread(bool theIsMultiThread) => myIsMultiThread = theIsMultiThread;

    /// <summary>True if the computation is performed in parallel.</summary>
    public bool IsMultiThread() => myIsMultiThread;
}
