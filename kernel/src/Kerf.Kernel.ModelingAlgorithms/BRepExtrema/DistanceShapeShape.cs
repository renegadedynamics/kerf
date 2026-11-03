// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_DistShapeShape.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_DistShapeShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Measurement.Distance.SupportType;
using static Kerf.Engine.Extrema.ExtAlgo;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Measurement.Distance;

/// <summary>
/// The minimum distance between two shapes (compound, compsolid, solid, shell, face, wire, edge or vertex): its
/// value, and the pairs of points that realise it with their supports and parameters.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). It only reads the shapes. The supports it returns are sub-shapes of the inputs,
/// except for an infinite edge or face facing a finite one, whose support is a trimmed copy built and frozen by
/// <see cref="DistanceSS"/>.
/// </para>
/// <para>
/// OCCT splits the work into tasks by the number of threads of OSD_ThreadPool's default pool, in single-threaded mode
/// too, and the split decides which pairs of sub-shapes are measured and in which order the solutions come. The port
/// takes <c>OSD_Parallel.NbLogicalProcessors()</c> (<see cref="Environment.ProcessorCount"/>), as that pool does
/// (one thread per logical processor), so it splits as native OCCT does on the same machine, serial or parallel. In
/// multi-threaded mode the solid treatment's result depends on which thread finds an inner point first, as in OCCT.
/// </para>
/// </remarks>
public sealed class DistanceShapeShape
{
    private double _distRef;
    private bool _isDone;
    private readonly NCollection_Sequence<SolutionElem> _solutionsShape1 = new();
    private readonly NCollection_Sequence<SolutionElem> _solutionsShape2 = new();
    private bool _innerSol;
    private double _eps;
    private Shape _shape1;
    private Shape _shape2;
    private readonly IndexedSet<Shape> _mapV1 = new(ShapeHasher.Instance);
    private readonly IndexedSet<Shape> _mapV2 = new(ShapeHasher.Instance);
    private readonly IndexedSet<Shape> _mapE1 = new(ShapeHasher.Instance);
    private readonly IndexedSet<Shape> _mapE2 = new(ShapeHasher.Instance);
    private readonly IndexedSet<Shape> _mapF1 = new(ShapeHasher.Instance);
    private readonly IndexedSet<Shape> _mapF2 = new(ShapeHasher.Instance);
    private bool _isInitS1;
    private bool _isInitS2;
#pragma warning disable CS0414 // OCCT keeps the obsolete flag and algorithm without reading them
    private ExtFlag _flag;
    private ExtAlgo _algo;
#pragma warning restore CS0414
    private readonly NCollection_Array1<Box> _bv1 = new();
    private readonly NCollection_Array1<Box> _bv2 = new();
    private readonly NCollection_Array1<Box> _be1 = new();
    private readonly NCollection_Array1<Box> _be2 = new();
    private readonly NCollection_Array1<Box> _bf1 = new();
    private readonly NCollection_Array1<Box> _bf2 = new();
    private bool _isMultiThread;

    /// <summary>OSD_ThreadPool::DefaultPool()->NbThreads(): one thread per logical processor.</summary>
    private static int ThreadCount => OsdParallel.LogicalProcessorCount;

    //=================================================================================================

    private static void Decomposition(in Shape s,
                                      IndexedSet<Shape> mapV,
                                      IndexedSet<Shape> mapE,
                                      IndexedSet<Shape> mapF)
    {
        mapV.Clear();
        mapE.Clear();
        mapF.Clear();
        Exploration.MapShapes(s, ShapeType.Vertex, mapV);
        Exploration.MapShapes(s, ShapeType.Edge, mapE);
        Exploration.MapShapes(s, ShapeType.Face, mapF);
    }

    private static void BoxCalculation(IndexedSet<Shape> map, NCollection_Array1<Box> sBox)
    {
        for (int i = 1; i <= map.Count; i++)
        {
            var box = new Box();
            Bounds.Add(map[i - 1], ref box);
            sBox[i] = box;
        }
    }

    private static double DistanceInitiale(in Vertex v1, in Vertex v2)
        => Tool.Pnt(v1).Distance(Tool.Pnt(v2));

    /// <summary>A pair of sub-shapes to check, with the distance between their boxes.</summary>
    private readonly record struct BRepExtrema_CheckPair(int Index1, int Index2, double Distance);

    private readonly record struct IndexBand(int First, int Last);

    private sealed class ThreadSolution
    {
        public readonly NCollection_Array1<NCollection_Sequence<SolutionElem>> Shape1;
        public readonly NCollection_Array1<NCollection_Sequence<SolutionElem>> Shape2;
        public readonly NCollection_Array1<double> Dist;

        public ThreadSolution(int taskNum)
        {
            Shape1 = new NCollection_Array1<NCollection_Sequence<SolutionElem>>(0, taskNum - 1);
            Shape2 = new NCollection_Array1<NCollection_Sequence<SolutionElem>>(0, taskNum - 1);
            for (int i = 0; i < taskNum; i++)
            {
                Shape1[i] = new NCollection_Sequence<SolutionElem>();
                Shape2[i] = new NCollection_Sequence<SolutionElem>();
            }
            Dist = new NCollection_Array1<double>(0, taskNum - 1);
            Dist.Init(double.MaxValue);
        }
    }

    private sealed class VertexFunctor
    {
        public readonly NCollection_Array1<IndexBand> BandArray;
        public readonly ThreadSolution Solution;
        public IndexedSet<Shape>? Map1;
        public IndexedSet<Shape>? Map2;
        public readonly ProgressScope Scope;
        public readonly NCollection_Array1<ProgressRange> Ranges;
        public double Eps;
        public double StartDist;

        public VertexFunctor(NCollection_Array1<IndexBand> bandArray, ProgressRange range)
        {
            BandArray = bandArray;
            Solution = new ThreadSolution(bandArray.Length());
            Scope = new ProgressScope(range, "Vertices distances calculating", bandArray.Length());
            Ranges = new NCollection_Array1<ProgressRange>(0, bandArray.Length() - 1);
            Eps = global::Kerf.Engine.Foundation.Precision.Confusion();
            StartDist = 0.0;
            for (int i = 0; i < bandArray.Length(); ++i)
            {
                Ranges.SetValue(i, Scope.Next());
            }
        }

        public void Invoke(int index)
        {
            int count2 = Map2!.Count;
            int first = BandArray.Value(index).First;
            int last = BandArray.Value(index).Last;
            Solution.Dist[index] = StartDist;

            using var scope = new ProgressScope(Ranges[index], null, (double)last - first);

            for (int idx1 = first; idx1 <= last; ++idx1)
            {
                if (!scope.More())
                {
                    break;
                }
                scope.Next();

                Vertex vertex1 = ShapeCasts.Vertex(Map1![idx1 - 1]);
                Point3D point1 = Tool.Pnt(vertex1);
                for (int idx2 = 1; idx2 <= count2; ++idx2)
                {
                    Vertex vertex2 = ShapeCasts.Vertex(Map2[idx2 - 1]);
                    Point3D point2 = Tool.Pnt(vertex2);
                    double dist = point1.Distance(point2);
                    {
                        if (dist < Solution.Dist[index] - Eps)
                        {
                            var sol1 = new SolutionElem(dist, point1, SupportType.IsVertex, vertex1);
                            var sol2 = new SolutionElem(dist, point2, SupportType.IsVertex, vertex2);

                            Solution.Shape1[index].Clear();
                            Solution.Shape2[index].Clear();
                            Solution.Shape1[index].Append(sol1);
                            Solution.Shape2[index].Append(sol2);

                            Solution.Dist[index] = dist;
                        }
                        else if (Math.Abs(dist - Solution.Dist[index]) < Eps)
                        {
                            var sol1 = new SolutionElem(dist, point1, SupportType.IsVertex, vertex1);
                            var sol2 = new SolutionElem(dist, point2, SupportType.IsVertex, vertex2);
                            Solution.Shape1[index].Append(sol1);
                            Solution.Shape2[index].Append(sol2);

                            if (Solution.Dist[index] > dist)
                            {
                                Solution.Dist[index] = dist;
                            }
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    private bool DistanceVertVert(IndexedSet<Shape> map1,
                                  IndexedSet<Shape> map2,
                                  ProgressRange range)
    {
        int count1 = map1.Count;
        int minTaskSize = count1 < 10 ? count1 : 10;
        int nbThreads = ThreadCount;
        int nbTasks = nbThreads;
        int taskSize = (int)Math.Ceiling((double)count1 / nbTasks);
        if (taskSize < minTaskSize)
        {
            taskSize = minTaskSize;
            nbTasks = (int)Math.Ceiling((double)count1 / taskSize);
        }

        int firstIndex = 1;
        var bandArray = new NCollection_Array1<IndexBand>(0, nbTasks - 1);
        using var distScope = new ProgressScope(range, null, 1);

        for (int i = 0; i < bandArray.Length(); ++i)
        {
            if (count1 < firstIndex + taskSize - 1)
            {
                taskSize = count1 - firstIndex + 1;
            }
            bandArray.SetValue(i, new IndexBand(firstIndex, firstIndex + taskSize - 1));
            firstIndex += taskSize;
        }

        var functor = new VertexFunctor(bandArray, distScope.Next());
        functor.Map1 = map1;
        functor.Map2 = map2;
        functor.StartDist = _distRef;
        functor.Eps = _eps;

        OsdParallel.For(0, nbTasks, functor.Invoke, !_isMultiThread);
        functor.Scope.Close();
        if (!distScope.More())
        {
            return false;
        }
        for (int i = 0; i < functor.Solution.Dist.Length(); ++i)
        {
            double dist = functor.Solution.Dist[i];
            if (dist < _distRef - _eps)
            {
                _solutionsShape1.Clear();
                _solutionsShape2.Clear();
                _solutionsShape1.Append(functor.Solution.Shape1[i]);
                _solutionsShape2.Append(functor.Solution.Shape2[i]);
                _distRef = dist;
            }
            else if (Math.Abs(dist - _distRef) < _eps)
            {
                _solutionsShape1.Append(functor.Solution.Shape1[i]);
                _solutionsShape2.Append(functor.Solution.Shape2[i]);
                _distRef = dist;
            }
        }
        return true;
    }

    //=================================================================================================

    private sealed class DistanceFunctor
    {
        public readonly NCollection_Array1<NCollection_Array1<BRepExtrema_CheckPair>> ArrayOfArrays;
        public readonly ThreadSolution Solution;
        public IndexedSet<Shape>? Map1;
        public IndexedSet<Shape>? Map2;
        public NCollection_Array1<Box>? LBox1;
        public NCollection_Array1<Box>? LBox2;
        public readonly ProgressScope Scope;
        public readonly NCollection_Array1<ProgressRange> Ranges;
        public double Eps;
        public double StartDist;

        public DistanceFunctor(NCollection_Array1<NCollection_Array1<BRepExtrema_CheckPair>> arrayOfArrays,
                               ProgressRange range)
        {
            ArrayOfArrays = arrayOfArrays;
            Solution = new ThreadSolution(ArrayOfArrays.Length());
            Scope = new ProgressScope(range, "Shapes distances calculating", arrayOfArrays.Length());
            Ranges = new NCollection_Array1<ProgressRange>(0, arrayOfArrays.Length() - 1);
            Eps = global::Kerf.Engine.Foundation.Precision.Confusion();
            StartDist = 0.0;
            for (int i = 0; i < arrayOfArrays.Length(); ++i)
            {
                Ranges.SetValue(i, Scope.Next());
            }
        }

        public void Invoke(int index)
        {
            using var scope = new ProgressScope(Ranges[index], null, ArrayOfArrays.Value(index).Length());
            Solution.Dist[index] = StartDist;
            for (int i = 0; i < ArrayOfArrays.Value(index).Length(); i++)
            {
                if (!scope.More())
                {
                    return;
                }
                scope.Next();
                BRepExtrema_CheckPair pair = ArrayOfArrays.Value(index).Value(i);
                if (pair.Distance > Solution.Dist[index] + Eps)
                {
                    break; // early search termination
                }
                Box box1 = LBox1!.Value(pair.Index1);
                Box box2 = LBox2!.Value(pair.Index2);
                Shape shape1 = Map1![pair.Index1 - 1];
                Shape shape2 = Map2![pair.Index2 - 1];
                var distTool = new DistanceSS(shape1, shape2, box1, box2, Solution.Dist[index], Eps);
                double dist = distTool.DistValue();
                if (distTool.IsDone)
                {
                    if (dist < Solution.Dist[index] - Eps)
                    {
                        Solution.Shape1[index].Clear();
                        Solution.Shape2[index].Clear();

                        var seq1 = new NCollection_Sequence<SolutionElem>(distTool.InternalSeq1Value());
                        var seq2 = new NCollection_Sequence<SolutionElem>(distTool.InternalSeq2Value());

                        Solution.Shape1[index].Append(seq1);
                        Solution.Shape2[index].Append(seq2);

                        Solution.Dist[index] = distTool.DistValue();
                    }
                    else if (Math.Abs(dist - Solution.Dist[index]) < Eps)
                    {
                        var seq1 = new NCollection_Sequence<SolutionElem>(distTool.InternalSeq1Value());
                        var seq2 = new NCollection_Sequence<SolutionElem>(distTool.InternalSeq2Value());

                        Solution.Shape1[index].Append(seq1);
                        Solution.Shape2[index].Append(seq2);
                        if (Solution.Dist[index] > dist)
                        {
                            Solution.Dist[index] = dist;
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
        public readonly NCollection_Array1<List<BRepExtrema_CheckPair>> PairList;
        public NCollection_Array1<Box>? LBox1;
        public NCollection_Array1<Box>? LBox2;
        public readonly ProgressScope Scope;
        public readonly NCollection_Array1<ProgressRange> Ranges;
        public double DistRef;
        public double Eps;

        public DistancePairFunctor(NCollection_Array1<IndexBand> bandArray, ProgressRange range)
        {
            BandArray = bandArray;
            PairList = new NCollection_Array1<List<BRepExtrema_CheckPair>>(0, bandArray.Length() - 1);
            for (int i = 0; i < bandArray.Length(); ++i)
            {
                PairList[i] = new List<BRepExtrema_CheckPair>();
            }
            Scope = new ProgressScope(range, "Boxes distances calculating", bandArray.Length());
            Ranges = new NCollection_Array1<ProgressRange>(0, bandArray.Length() - 1);
            DistRef = 0;
            Eps = global::Kerf.Engine.Foundation.Precision.Confusion();
            for (int i = 0; i < bandArray.Length(); ++i)
            {
                Ranges.SetValue(i, Scope.Next());
            }
        }

        public void Invoke(int index)
        {
            int first = BandArray.Value(index).First;
            int last = BandArray.Value(index).Last;

            using var scope = new ProgressScope(Ranges[index], null, (double)last - first);

            for (int idx1 = first; idx1 <= last; ++idx1)
            {
                if (!scope.More())
                {
                    break;
                }
                scope.Next();

                for (int idx2 = 1; idx2 <= LBox2!.Length(); ++idx2)
                {
                    Box box1 = LBox1!.Value(idx1);
                    Box box2 = LBox2.Value(idx2);
                    if (box1.IsVoid || box2.IsVoid)
                    {
                        continue;
                    }

                    double dist = box1.Distance(box2);
                    if (dist - DistRef < Eps)
                    {
                        PairList[index].Add(new BRepExtrema_CheckPair(idx1, idx2, dist));
                    }
                }
            }
        }

        public int ListSize()
        {
            int size = 0;
            for (int i = PairList.Lower(); i <= PairList.Upper(); ++i)
            {
                size += PairList[i].Count;
            }
            return size;
        }
    }

    //=================================================================================================

    private bool DistanceMapMap(IndexedSet<Shape> map1,
                                IndexedSet<Shape> map2,
                                NCollection_Array1<Box> lBox1,
                                NCollection_Array1<Box> lBox2,
                                ProgressRange range)
    {
        int count1 = map1.Count;
        int count2 = map2.Count;

        if (count1 == 0 || count2 == 0)
        {
            return true;
        }

        using var twinScope = new ProgressScope(range, null, 1.0);

        int nbThreads = ThreadCount;
        int minPairTaskSize = count1 < 10 ? count1 : 10;
        int nbPairTasks = nbThreads;
        int pairTaskSize = (int)Math.Ceiling((double)count1 / nbPairTasks);
        if (pairTaskSize < minPairTaskSize)
        {
            pairTaskSize = minPairTaskSize;
            nbPairTasks = (int)Math.Ceiling((double)count1 / pairTaskSize);
        }

        int firstIndex = 1;
        var bandArray = new NCollection_Array1<IndexBand>(0, nbPairTasks - 1);

        for (int i = 0; i < bandArray.Length(); ++i)
        {
            if (count1 < firstIndex + pairTaskSize - 1)
            {
                pairTaskSize = count1 - firstIndex + 1;
            }
            bandArray.SetValue(i, new IndexBand(firstIndex, firstIndex + pairTaskSize - 1));
            firstIndex += pairTaskSize;
        }

        twinScope.Next(0.15);
        var pairFunctor = new DistancePairFunctor(bandArray, twinScope.Next(0.15));
        pairFunctor.LBox1 = lBox1;
        pairFunctor.LBox2 = lBox2;
        pairFunctor.DistRef = _distRef;
        pairFunctor.Eps = _eps;

        OsdParallel.For(0, nbPairTasks, pairFunctor.Invoke, !_isMultiThread);
        pairFunctor.Scope.Close();
        if (!twinScope.More())
        {
            return false;
        }
        int listSize = pairFunctor.ListSize();
        if (listSize == 0)
        {
            return true;
        }
        var pairList = new BRepExtrema_CheckPair[listSize];
        int listIndex = 0;
        for (int i = 0; i < pairFunctor.PairList.Length(); ++i)
        {
            for (int j = 0; j < pairFunctor.PairList[i].Count; ++j)
            {
                pairList[listIndex] = pairFunctor.PairList[i][j];
                ++listIndex;
            }
        }

        // std::stable_sort by the distance (LINQ's OrderBy is a stable sort)
        pairList = [.. pairList.OrderBy(static pair => pair.Distance)];

        int mapSize = pairList.Length;
        int nbTasks = mapSize < nbThreads ? mapSize : nbThreads;
        int taskSize = (int)Math.Ceiling((double)mapSize / nbTasks);

        var arrayOfArray = new NCollection_Array1<NCollection_Array1<BRepExtrema_CheckPair>>(0, nbTasks - 1);
        for (int i = 0; i < nbTasks; ++i)
        {
            arrayOfArray[i] = new NCollection_Array1<BRepExtrema_CheckPair>();
        }
        // Since aPairList is sorted in ascending order of distances between Bnd_Boxes,
        // BRepExtrema_CheckPair are distributed to tasks one by one from smallest to largest,
        // and not ranges, as for DistancePairFunctor.
        // Since aMapSize may not be divisible entirely by the number of tasks,
        // some tasks should receive one BRepExtrema_CheckPair less than the rest.
        // aLastRowLimit defines the task number from which to start tasks containing
        // fewer BRepExtrema_CheckPair
        int lastRowLimit = ((mapSize % nbTasks) == 0) ? nbTasks : (mapSize % nbTasks);
        for (int i = 0; i < taskSize; ++i)
        {
            for (int j = 0; j < nbTasks; ++j)
            {
                if (i == 0)
                {
                    int vectorSize = taskSize;
                    if (j >= lastRowLimit)
                    {
                        vectorSize--;
                    }
                    arrayOfArray[j].Resize(0, vectorSize - 1, false);
                }
                if (i < arrayOfArray[j].Length())
                {
                    arrayOfArray[j][i] = pairList[i * nbTasks + j];
                }
                else
                {
                    break;
                }
            }
        }
        var functor = new DistanceFunctor(arrayOfArray, twinScope.Next(0.85));
        functor.Map1 = map1;
        functor.Map2 = map2;
        functor.LBox1 = lBox1;
        functor.LBox2 = lBox2;
        functor.Eps = _eps;
        functor.StartDist = _distRef;

        OsdParallel.For(0, nbTasks, functor.Invoke, !_isMultiThread);
        functor.Scope.Close();
        if (!twinScope.More())
        {
            return false;
        }

        for (int i = 0; i < functor.Solution.Dist.Length(); ++i)
        {
            double dist = functor.Solution.Dist[i];
            if (dist < _distRef - _eps)
            {
                _solutionsShape1.Clear();
                _solutionsShape2.Clear();
                _solutionsShape1.Append(functor.Solution.Shape1[i]);
                _solutionsShape2.Append(functor.Solution.Shape2[i]);
                _distRef = dist;
            }
            else if (Math.Abs(dist - _distRef) < _eps)
            {
                _solutionsShape1.Append(functor.Solution.Shape1[i]);
                _solutionsShape2.Append(functor.Solution.Shape2[i]);
                if (_distRef > dist)
                {
                    _distRef = dist;
                }
            }
        }
        return true;
    }

    //=================================================================================================

    /// <summary>An empty tool.</summary>
    public DistanceShapeShape()
    {
        _distRef = 0.0;
        _isDone = false;
        _innerSol = false;
        _eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        _isInitS1 = false;
        _isInitS2 = false;
        _flag = ExtFlag_MINMAX;
        _algo = ExtAlgo_Grad;
        _isMultiThread = false;
    }

    /// <summary>
    /// Computes the minimum distance between the shapes (value and pairs of points) with the default deflection
    /// (Precision::Confusion()) in single-threaded mode. <paramref name="f"/> and <paramref name="a"/> are obsolete
    /// and unused.
    /// </summary>
    public DistanceShapeShape(in Shape shape1,
                                      in Shape shape2,
                                      ExtFlag f = ExtFlag_MINMAX,
                                      ExtAlgo a = ExtAlgo_Grad,
                                      ProgressRange range = default)
    {
        _distRef = 0.0;
        _isDone = false;
        _innerSol = false;
        _eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        _isInitS1 = false;
        _isInitS2 = false;
        _flag = f;
        _algo = a;
        _isMultiThread = false;
        LoadS1(shape1);
        LoadS2(shape2);
        Perform(range);
    }

    /// <summary>
    /// Computes the minimum distance between the shapes in single-threaded mode; solutions within
    /// <paramref name="deflection"/> of the minimum are kept. <paramref name="f"/> and <paramref name="a"/> are
    /// obsolete and unused.
    /// </summary>
    public DistanceShapeShape(in Shape shape1,
                                      in Shape shape2,
                                      double deflection,
                                      ExtFlag f = ExtFlag_MINMAX,
                                      ExtAlgo a = ExtAlgo_Grad,
                                      ProgressRange range = default)
    {
        _distRef = 0.0;
        _isDone = false;
        _innerSol = false;
        _eps = deflection;
        _isInitS1 = false;
        _isInitS2 = false;
        _flag = f;
        _algo = a;
        _isMultiThread = false;
        LoadS1(shape1);
        LoadS2(shape2);
        Perform(range);
    }

    /// <summary>Sets the deflection: the largest deviation of the kept solutions from the minimum distance.</summary>
    public void SetDeflection(double deflection) => _eps = deflection;

    /// <summary>Loads the first shape.</summary>
    public void LoadS1(in Shape shape1)
    {
        _shape1 = shape1;
        _isInitS1 = false;
        Decomposition(shape1, _mapV1, _mapE1, _mapF1);
    }

    /// <summary>Loads the second shape.</summary>
    public void LoadS2(in Shape shape2)
    {
        _shape2 = shape2;
        _isInitS2 = false;
        Decomposition(shape2, _mapV2, _mapE2, _mapF2);
    }

    //=================================================================================================

    private sealed class TreatmentFunctor
    {
        public readonly NCollection_Array1<NCollection_Array1<Shape>> ArrayOfArrays;
        public NCollection_Sequence<SolutionElem>? SolutionsShape1;
        public NCollection_Sequence<SolutionElem>? SolutionsShape2;
        public Shape Shape;
        public readonly ProgressScope Scope;
        public readonly NCollection_Array1<ProgressRange> Ranges;
        public double DistRef;
        public volatile bool InnerSol;
        public volatile bool IsDone;
        public Lock? Mutex;

        public TreatmentFunctor(NCollection_Array1<NCollection_Array1<Shape>> arrayOfArrays,
                                ProgressRange range)
        {
            ArrayOfArrays = arrayOfArrays;
            Scope = new ProgressScope(range, "Search for the inner solid", arrayOfArrays.Length());
            Ranges = new NCollection_Array1<ProgressRange>(0, arrayOfArrays.Length() - 1);
            for (int i = 0; i < arrayOfArrays.Length(); ++i)
            {
                Ranges.SetValue(i, Scope.Next());
            }
        }

        private void Record(in Point3D aPnt, in Vertex aVertex)
        {
            InnerSol = true;
            DistRef = 0.0;
            IsDone = true;
            var solElem = new SolutionElem(0, aPnt, SupportType.IsVertex, aVertex);
            SolutionsShape1!.Append(solElem);
            SolutionsShape2!.Append(solElem);
        }

        public void Invoke(int index)
        {
            const double tolerance = 0.001;
            using var scope = new ProgressScope(Ranges[index], null, ArrayOfArrays.Value(index).Length());
            var classifier = new SolidClassifier(Shape);

            for (int i = 0; i < ArrayOfArrays.Value(index).Length(); i++)
            {
                if (!scope.More())
                {
                    break;
                }
                scope.Next();
                if (IsDone)
                {
                    break;
                }

                Vertex vertex = ShapeCasts.Vertex(ArrayOfArrays.Value(index).Value(i));
                Point3D pnt = Tool.Pnt(vertex);
                classifier.Perform(pnt, tolerance);
                if (classifier.State() == In)
                {
                    if (Mutex is not null)
                    {
                        lock (Mutex)
                        {
                            Record(pnt, vertex);
                        }
                    }
                    else
                    {
                        Record(pnt, vertex);
                    }
                    break;
                }
            }
        }
    }

    //=================================================================================================

    private bool SolidTreatment(in Shape shape,
                                IndexedSet<Shape> vertexMap,
                                ProgressRange range)
    {
        int mapSize = vertexMap.Count;
        const int minTaskSize = 3;
        int nbThreads = ThreadCount;
        int nbTasks = nbThreads * 10;
        int taskSize = (int)Math.Ceiling((double)mapSize / nbTasks);
        if (taskSize < minTaskSize)
        {
            taskSize = minTaskSize;
            nbTasks = (int)Math.Ceiling((double)mapSize / taskSize);
        }

        var arrayOfArray = new NCollection_Array1<NCollection_Array1<Shape>>(0, nbTasks - 1);
        for (int i = 0; i < nbTasks; ++i)
        {
            arrayOfArray[i] = new NCollection_Array1<Shape>();
        }
        for (int i = 1; i <= mapSize; ++i)
        {
            int vectIndex = (i - 1) / taskSize;
            int shapeIndex = (i - 1) % taskSize;
            if (shapeIndex == 0)
            {
                int vectorSize = taskSize;
                int tailSize = mapSize - vectIndex * taskSize;
                if (tailSize < taskSize)
                {
                    vectorSize = tailSize;
                }
                arrayOfArray[vectIndex].Resize(0, vectorSize - 1, false);
            }
            arrayOfArray[vectIndex][shapeIndex] = vertexMap[i - 1];
        }

        // Local variables for thread-safe communication during the parallel section (OCCT's atomics)
        using var scope = new ProgressScope(range, "Solid treatment", nbTasks);
        var functor = new TreatmentFunctor(arrayOfArray, scope.Next());
        functor.SolutionsShape1 = _solutionsShape1;
        functor.SolutionsShape2 = _solutionsShape2;
        functor.Shape = shape;
        functor.DistRef = _distRef;
        functor.InnerSol = _innerSol;
        functor.IsDone = _isDone;
        if (_isMultiThread && functor.Mutex is null)
        {
            functor.Mutex = new Lock();
        }

        OsdParallel.For(0, nbTasks, functor.Invoke, !_isMultiThread);
        functor.Scope.Close();

        // Copy the results back to the class members after the parallel section completes
        _distRef = functor.DistRef;
        _innerSol = functor.InnerSol;
        _isDone = functor.IsDone;

        return scope.More();
    }

    //=================================================================================================

    /// <summary>
    /// Computes the minimum distance (value and pairs of points); solutions within the deflection of the minimum are
    /// kept. Returns <see cref="IsDone"/>.
    /// </summary>
    public bool Perform(ProgressRange range = default)
    {
        _isDone = false;
        _innerSol = false;
        _solutionsShape1.Clear();
        _solutionsShape2.Clear();

        if (_shape1.IsNull || _shape2.IsNull)
        {
            return false;
        }

        // Treatment of solids
        bool isSolid1 = (_shape1.ShapeType() == ShapeType.Solid) || (_shape1.ShapeType() == ShapeType.CompSolid);
        bool isSolid2 = (_shape2.ShapeType() == ShapeType.Solid) || (_shape2.ShapeType() == ShapeType.CompSolid);
        int rootStepsNum = 9; // By num of DistanceMapMap calls
        rootStepsNum = isSolid1 ? rootStepsNum + 1 : rootStepsNum;
        rootStepsNum = isSolid2 ? rootStepsNum + 1 : rootStepsNum;
        using var rootScope = new ProgressScope(range, "calculating distance", rootStepsNum);

        if (isSolid1)
        {
            if (!SolidTreatment(_shape1, _mapV2, rootScope.Next()))
            {
                return false;
            }
        }

        if (isSolid2 && (!_innerSol))
        {
            if (!SolidTreatment(_shape2, _mapV1, rootScope.Next()))
            {
                return false;
            }
        }

        if (!_innerSol)
        {
            if (!_isInitS1) // rebuild cached data for 1st shape
            {
                if (_mapV1.Count != 0)
                {
                    _bv1.Resize(1, _mapV1.Count, false);
                }
                if (_mapE1.Count != 0)
                {
                    _be1.Resize(1, _mapE1.Count, false);
                }
                if (_mapF1.Count != 0)
                {
                    _bf1.Resize(1, _mapF1.Count, false);
                }

                BoxCalculation(_mapV1, _bv1);
                BoxCalculation(_mapE1, _be1);
                BoxCalculation(_mapF1, _bf1);

                _isInitS1 = true;
            }

            if (!_isInitS2) // rebuild cached data for 2nd shape
            {
                if (_mapV2.Count != 0)
                {
                    _bv2.Resize(1, _mapV2.Count, false);
                }
                if (_mapE2.Count != 0)
                {
                    _be2.Resize(1, _mapE2.Count, false);
                }
                if (_mapF2.Count != 0)
                {
                    _bf2.Resize(1, _mapF2.Count, false);
                }

                BoxCalculation(_mapV2, _bv2);
                BoxCalculation(_mapE2, _be2);
                BoxCalculation(_mapF2, _bf2);

                _isInitS2 = true;
            }

            if (_mapV1.Count != 0 && _mapV2.Count != 0)
            {
                Vertex v1 = ShapeCasts.Vertex(_mapV1[0]);
                Vertex v2 = ShapeCasts.Vertex(_mapV2[0]);
                _distRef = DistanceInitiale(v1, v2);
            }
            else
            {
                _distRef = 1.0e30; // szv:!!!
            }

            if (!DistanceVertVert(_mapV1, _mapV2, rootScope.Next()))
            {
                return false;
            }
            if (!DistanceMapMap(_mapV1, _mapE2, _bv1, _be2, rootScope.Next()))
            {
                return false;
            }
            if (!DistanceMapMap(_mapE1, _mapV2, _be1, _bv2, rootScope.Next()))
            {
                return false;
            }
            if (!DistanceMapMap(_mapV1, _mapF2, _bv1, _bf2, rootScope.Next()))
            {
                return false;
            }
            if (!DistanceMapMap(_mapF1, _mapV2, _bf1, _bv2, rootScope.Next()))
            {
                return false;
            }
            if (!DistanceMapMap(_mapE1, _mapE2, _be1, _be2, rootScope.Next()))
            {
                return false;
            }
            if (!DistanceMapMap(_mapE1, _mapF2, _be1, _bf2, rootScope.Next()))
            {
                return false;
            }
            if (!DistanceMapMap(_mapF1, _mapE2, _bf1, _be2, rootScope.Next()))
            {
                return false;
            }

            if (Math.Abs(_distRef) > _eps)
            {
                if (!DistanceMapMap(_mapF1, _mapF2, _bf1, _bf2, rootScope.Next()))
                {
                    return false;
                }
            }

            //  Modified by Sergey KHROMOV - Tue Mar  6 11:55:03 2001 Begin
            // OCCT: the index isn't stepped back after a removal, so the solution after a removed one is not checked
            // (occt-issues.md).
            int i = 1;
            for (; i <= _solutionsShape1.Length(); i++)
            {
                if (_solutionsShape1.Value(i).Dist() > _distRef + _eps)
                {
                    _solutionsShape1.Remove(i);
                    _solutionsShape2.Remove(i);
                }
            }
            //  Modified by Sergey KHROMOV - Tue Mar  6 11:55:04 2001 End
            _isDone = (_solutionsShape1.Length() > 0);
        }

        return _isDone;
    }

    /// <summary>True if the minimum distance is found.</summary>
    public bool IsDone => _isDone;

    /// <summary>The number of solutions satisfying the minimum distance.</summary>
    public int SolutionCount => _solutionsShape1.Length();

    /// <summary>The minimum distance; throws StdFail_NotDone if there is no solution.</summary>
    public double Value()
    {
        if (!_isDone)
        {
            throw new NotDoneException("BRepExtrema_DistShapeShape::Value: There's no solution ");
        }

        return _distRef;
    }

    /// <summary>True if one of the shapes is a solid and the other is completely or partially inside it.</summary>
    public bool InnerSolution() => _innerSol;

    /// <summary>The point of the <paramref name="n"/>th solution on the first shape.</summary>
    public Point3D PointOnShape1(int n) => _solutionsShape1.Value(n).Point();

    /// <summary>The point of the <paramref name="n"/>th solution on the second shape.</summary>
    public Point3D PointOnShape2(int n) => _solutionsShape2.Value(n).Point();

    /// <summary>The kind of support of the <paramref name="n"/>th solution on the first shape.</summary>
    public SupportType SupportTypeShape1(int n) => _solutionsShape1.Value(n).SupportKind();

    /// <summary>The kind of support of the <paramref name="n"/>th solution on the second shape.</summary>
    public SupportType SupportTypeShape2(int n) => _solutionsShape2.Value(n).SupportKind();

    /// <summary>The support (a vertex, an edge or a face) of the <paramref name="n"/>th solution on the first shape.</summary>
    public Shape SupportOnShape1(int n)
    {
        if (!_isDone)
        {
            throw new NotDoneException("BRepExtrema_DistShapeShape::SupportOnShape1: There's no solution ");
        }

        SolutionElem sol = _solutionsShape1.Value(n);
        switch (sol.SupportKind())
        {
            case SupportType.IsVertex:
                return sol.Vertex();
            case SupportType.IsOnEdge:
                return sol.Edge();
            case SupportType.IsInFace:
                return sol.Face();
        }
        return new Shape();
    }

    /// <summary>The support (a vertex, an edge or a face) of the <paramref name="n"/>th solution on the second shape.</summary>
    public Shape SupportOnShape2(int n)
    {
        if (!_isDone)
        {
            throw new NotDoneException("BRepExtrema_DistShapeShape::SupportOnShape2: There's no solution ");
        }

        SolutionElem sol = _solutionsShape2.Value(n);
        switch (sol.SupportKind())
        {
            case SupportType.IsVertex:
                return sol.Vertex();
            case SupportType.IsOnEdge:
                return sol.Edge();
            case SupportType.IsInFace:
                return sol.Face();
        }
        return new Shape();
    }

    /// <summary>The parameter of the <paramref name="n"/>th solution on an edge of the first shape.</summary>
    public void ParOnEdgeS1(int n, out double t)
    {
        if (!_isDone)
        {
            throw new NotDoneException("BRepExtrema_DistShapeShape::ParOnEdgeS1: There's no solution");
        }

        SolutionElem sol = _solutionsShape1.Value(n);
        if (sol.SupportKind() != SupportType.IsOnEdge)
        {
            throw new IncompatibleShapeException(
                "BRepExtrema_DistShapeShape::ParOnEdgeS1: ParOnEdgeS1 is impossible without EDGE");
        }

        sol.EdgeParameter(out t);
    }

    /// <summary>The parameter of the <paramref name="n"/>th solution on an edge of the second shape.</summary>
    public void ParOnEdgeS2(int n, out double t)
    {
        if (!_isDone)
        {
            throw new NotDoneException("BRepExtrema_DistShapeShape::ParOnEdgeS2: There's no solution");
        }

        SolutionElem sol = _solutionsShape2.Value(n);
        if (sol.SupportKind() != SupportType.IsOnEdge)
        {
            throw new IncompatibleShapeException(
                "BRepExtrema_DistShapeShape::ParOnEdgeS2: ParOnEdgeS2 is impossible without EDGE");
        }

        sol.EdgeParameter(out t);
    }

    /// <summary>The parameters of the <paramref name="n"/>th solution in a face of the first shape.</summary>
    public void ParOnFaceS1(int n, out double u, out double v)
    {
        if (!_isDone)
        {
            throw new NotDoneException("BRepExtrema_DistShapeShape::ParOnFaceS1: There's no solution");
        }

        SolutionElem sol = _solutionsShape1.Value(n);
        if (sol.SupportKind() != SupportType.IsInFace)
        {
            throw new IncompatibleShapeException(
                "BRepExtrema_DistShapeShape::ParOnFaceS1: ParOnFaceS1 is impossible without FACE");
        }

        sol.FaceParameter(out u, out v);
    }

    /// <summary>The parameters of the <paramref name="n"/>th solution in a face of the second shape.</summary>
    public void ParOnFaceS2(int n, out double u, out double v)
    {
        if (!_isDone)
        {
            throw new NotDoneException("BRepExtrema_DistShapeShape::ParOnFaceS2: There's no solution");
        }

        SolutionElem sol = _solutionsShape2.Value(n);
        if (sol.SupportKind() != SupportType.IsInFace)
        {
            throw new IncompatibleShapeException(
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
        o.Write("the number of solutions is :" + SolutionCount + '\n');
        o.Write('\n');
        for (i = 1; i <= SolutionCount; i++)
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
                case SupportType.IsVertex:
                    break;
                case SupportType.IsOnEdge:
                    ParOnEdgeS1(i, out r1);
                    o.Write("parameter on the first edge :  t= " + F(r1) + '\n');
                    break;
                case SupportType.IsInFace:
                    ParOnFaceS1(i, out r1, out r2);
                    o.Write("parameters on the first face :  u= " + F(r1) + " v=" + F(r2) + '\n');
                    break;
            }
            switch (SupportTypeShape2(i))
            {
                case SupportType.IsVertex:
                    break;
                case SupportType.IsOnEdge:
                    ParOnEdgeS2(i, out r1);
                    o.Write("parameter on the second edge : t=" + F(r1) + '\n');
                    break;
                case SupportType.IsInFace:
                    ParOnFaceS2(i, out r1, out r2);
                    o.Write("parameters on the second face : u= " + F(r1) + " v=" + F(r2) + '\n');
                    break;
            }
            o.Write('\n');
        }

        static string F(double value) => StreamUtilities.FormatReal(value, 6);
    }

    /// <summary>Sets an unused parameter (obsolete).</summary>
    public void SetFlag(ExtFlag f) => _flag = f;

    /// <summary>Sets an unused parameter (obsolete).</summary>
    public void SetAlgo(ExtAlgo a) => _algo = a;

    /// <summary>Computes in parallel when true (the default is false).</summary>
    public void SetMultiThread(bool isMultiThread) => _isMultiThread = isMultiThread;

    /// <summary>True if the computation is performed in parallel.</summary>
    public bool IsMultiThread => _isMultiThread;
}
