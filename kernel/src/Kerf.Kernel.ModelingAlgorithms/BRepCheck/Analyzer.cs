// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Analyzer.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Analyzer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using static Kerf.Engine.Healing.Checks.StatusCheck;

namespace Kerf.Engine.Healing.Checks;

/// <summary>
/// Checks a shape and all its sub-shapes: each vertex, edge, wire, face, shell and solid is checked on its own and
/// in the context of the shapes that contain it. <see cref="IsValid()"/> is true if no fault was found;
/// <see cref="Result"/> gives the statuses of each sub-shape.
/// </summary>
/// <remarks>
/// <para>
/// With <c>GeomControls</c> false only the topology is checked. The geometric controls are: for an edge,
/// InvalidCurveOnClosedSurface, InvalidCurveOnSurface and InvalidSameParameterFlag; for a face, UnorientableShape and
/// IntersectingWires; for a wire, SelfIntersectingWire.
/// </para>
/// <para>
/// The parallel mode checks the sub-shapes on several threads (in chunks of the map of sub-shapes, as OCCT does) and
/// gives the same statuses as the serial mode. The analyzer only reads the shape; the shapes it builds to classify
/// wires and shells are its own and never handed out.
/// </para>
/// <para>
/// OCCT catches <c>Standard_Failure</c> around each check, with OCC_CATCH_SIGNALS turning access violations into
/// failures, and records <see cref="CheckFail"/>. The port catches <see cref="FailureException"/> and the
/// .NET exceptions that stand for those signals (null references, invalid casts of handles, indices out of range,
/// integer division by zero).
/// </para>
/// </remarks>
internal class Analyzer
{
    private Shape _shape;

    // OCCT declares this map without a hasher: the default one compares shapes with operator== (IsEqual), so a shape
    // met with two orientations (a seam edge, a vertex shared by edges) has one result per orientation, and the face
    // check (which skips shapes already seen with IsSame) puts the context on the first one only
    private readonly IndexedMap<Shape, ResultCheck?> _map =
        new(EqualityComparer<Shape>.Default);

    private bool _isParallel;
    private bool _isExact;

    /// <summary>
    /// Checks <paramref name="s"/>; with <paramref name="geomControls"/> false only topological information is
    /// checked. <paramref name="isParallel"/> checks the sub-shapes on several threads;
    /// <paramref name="isExact"/> measures edges on faces exactly (see <see cref="SetExactMethod"/>).
    /// </summary>
    public Analyzer(in Shape s,
                              bool geomControls = true,
                              bool isParallel = false,
                              bool isExact = false)
    {
        _shape = new Shape();
        _isParallel = isParallel;
        _isExact = isExact;
        Init(s, geomControls);
    }

    /// <summary>
    /// Checks <paramref name="shape"/> again, from scratch; with <paramref name="b"/> false only topological
    /// information is checked.
    /// </summary>
    public void Init(in Shape shape, bool b = true)
    {
        if (shape.IsNull)
        {
            throw new NullObjectException("BRepCheck_Analyzer::Init() - NULL shape");
        }

        _shape = shape;
        _map.Clear();
        Put(shape, b);
        Perform();
    }

    /// <summary>
    /// Sets the method that measures the distance between an edge's curves (used by the next <see cref="Init"/>): in
    /// a finite number of points (false, the default: faster, possibly not exact) or exactly with
    /// BRepLib_CheckCurveOnSurface (true: slower; only for same-parameter edges).
    /// </summary>
    public void SetExactMethod(bool isExact) => _isExact = isExact;

    /// <summary>Whether the exact method is selected.</summary>
    public bool IsExactMethod => _isExact;

    /// <summary>Sets the parallel mode (used by the next <see cref="Init"/>).</summary>
    public void SetParallel(bool isParallel) => _isParallel = isParallel;

    /// <summary>Whether the parallel mode is set.</summary>
    public bool IsParallel => _isParallel;

    /// <summary>
    /// True if no fault was found on <paramref name="s"/>, a sub-shape of the checked shape, or on any of its
    /// sub-shapes, on its own or in its context in <paramref name="s"/>. Only the first status of each list is read,
    /// as in OCCT.
    /// </summary>
    public bool IsValid(in Shape s)
    {
        if (_map.GetValue(s) is { } result)
        {
            var itl = new NCollection_List<StatusCheck>.Iterator();
            itl.Initialize(result.Status());
            if (itl.Value() != StatusCheck.NoError)
            {
                // a voir
                return false;
            }
        }

        for (var theIterator = new Iterator(s); theIterator.More(); theIterator.Next())
        {
            if (!IsValid(theIterator.Value()))
            {
                return false;
            }
        }

        switch (s.Type)
        {
            case ShapeType.Edge:
            {
                return ValidSub(s, ShapeType.Vertex);
            }
            case ShapeType.Face:
            {
                bool valid = ValidSub(s, ShapeType.Wire);
                valid = valid && ValidSub(s, ShapeType.Edge);
                valid = valid && ValidSub(s, ShapeType.Vertex);
                return valid;
            }
            case ShapeType.Shell:
                break;
            case ShapeType.Solid:
                return ValidSub(s, ShapeType.Shell);
            default:
                break;
        }

        return true;
    }

    /// <summary>
    /// True if no fault was found on the shape or any of its sub-shapes: the topology is correct and the edges'
    /// parameterisation is correct (see OCCT's documentation of BRepCheck_Analyzer::IsValid).
    /// </summary>
    public bool IsValid() => IsValid(_shape);

    /// <summary>
    /// The check of <paramref name="subS"/>, a sub-shape of the checked shape (null for compounds and
    /// compsolids). Throws <see cref="NoSuchObjectException"/> for a shape that isn't a sub-shape.
    /// </summary>
    public ResultCheck? Result(in Shape subS) => _map.GetValue(subS);

    /// <summary>Every checked sub-shape with its check (null for compounds and compsolids), in the order checked: the
    /// public <c>HealingOperations.Validate</c>'s report (unit 10L3).</summary>
    internal IEnumerable<(Shape Shape, ResultCheck? Check)> Results()
    {
        for (int i = 0; i < _map.Count; i++)
        {
            yield return (_map.KeyAt(i), _map.ValueAt(i));
        }
    }

    // The exceptions OCCT's catch (Standard_Failure) handles under OCC_CATCH_SIGNALS
    private static bool IsCaught(Exception exception)
        => (Refusal.IsFailure(exception) || exception is NullReferenceException
               or InvalidCastException
               or IndexOutOfRangeException
               or DivideByZeroException);

    private void Put(in Shape shape, bool b)
    {
        if (_map.ContainsKey(shape))
        {
            return;
        }

        ResultCheck? hr = null;
        switch (shape.Type)
        {
            case ShapeType.Vertex:
                hr = new VertexCheck(ShapeCasts.Vertex(shape));
                break;
            case ShapeType.Edge:
            {
                var resEdge = new EdgeCheck(ShapeCasts.Edge(shape));
                resEdge.GeometricControls(b);
                resEdge.SetExactMethod(_isExact);
                hr = resEdge;
                break;
            }
            case ShapeType.Wire:
            {
                var resWire = new WireCheck(ShapeCasts.Wire(shape));
                resWire.GeometricControls(b);
                hr = resWire;
                break;
            }
            case ShapeType.Face:
            {
                var resFace = new FaceCheck(ShapeCasts.Face(shape));
                resFace.GeometricControls(b);
                hr = resFace;
                break;
            }
            case ShapeType.Shell:
                hr = new ShellCheck(ShapeCasts.Shell(shape));
                break;
            case ShapeType.Solid:
                hr = new SolidCheck(ShapeCasts.Solid(shape));
                break;
            case ShapeType.CompSolid:
            case ShapeType.Compound:
                break;
            default:
                break;
        }

        _map.Add(shape, hr);

        for (var theIterator = new Iterator(shape); theIterator.More(); theIterator.Next())
        {
            Put(theIterator.Value(), b); // performs minimum on each shape
        }
    }

    // Sub-shapes, each counted once for each way round it's met: about 40 faces
    private const int MinParallelSize = 500;

    private void Perform()
    {
        int mapSize = _map.Count;
        // Handing a small shape to the pool costs more than it saves, and the result is the same on any number of threads (unit 11R).
        // So a small check has no locks and no copies of the adaptors either. The exact method is asked of the edges as it was: it has its own parallel loop.
        bool onPool = _isParallel && mapSize >= MinParallelSize;
        if (onPool || (_isParallel && _isExact))
        {
            for (int i = 0; i < mapSize; ++i)
            {
                _map.ValueAt(i)?.SetParallel(true);
            }
        }

        if (!onPool)
        {
            new BRepCheck_ParallelAnalyzer(_map).Perform(0, mapSize);
            return;
        }

        const int minTaskSize = 10;
        // OSD_ThreadPool::DefaultPool()->NbThreads() is the number of logical processors
        int nbThreads = OsdParallel.LogicalProcessorCount;
        int nbTasks = nbThreads * 10;
        int taskSize = (int)Math.Ceiling((double)mapSize / nbTasks);
        if (taskSize < minTaskSize)
        {
            taskSize = minTaskSize;
            nbTasks = (int)Math.Ceiling((double)mapSize / taskSize);
        }

        var arrayOfArray = new NCollection_Array1<Shape[]>(0, nbTasks - 1);
        // OCCT's arrays are default-constructed empty: chunks past the last shape stay empty
        for (int i = 0; i < nbTasks; ++i)
        {
            arrayOfArray[i] = [];
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

                arrayOfArray[vectIndex] = new Shape[vectorSize];
            }

            arrayOfArray[vectIndex][shapeIndex] = _map.KeyAt(i - 1);
        }

        var parallelAnalyzer = new BRepCheck_ParallelAnalyzer(_map);
        OsdParallel.For(0,
                        arrayOfArray.Length(),
                        vectorIndex => parallelAnalyzer.Perform(arrayOfArray[vectorIndex]));
    }

    private bool ValidSub(in Shape s, ShapeType subType)
    {
        var itl = new NCollection_List<StatusCheck>.Iterator();
        var exp = new Explorer();
        for (exp.Init(s, subType); exp.More(); exp.Next())
        {
            ResultCheck rv = _map.GetValue(exp.Current())!;
            for (rv.InitContextIterator(); rv.MoreShapeInContext(); rv.NextShapeInContext())
            {
                if (rv.ContextualShape().IsSame(s))
                {
                    break;
                }
            }

            if (!rv.MoreShapeInContext())
            {
                break;
            }

            for (itl.Initialize(rv.StatusOnShape()); itl.More(); itl.Next())
            {
                if (itl.Value() != StatusCheck.NoError)
                {
                    return false;
                }
            }
        }

        return true;
    }

    /// <summary>OCCT's functor for multi-threaded execution: checks one chunk of the sub-shapes in their contexts.</summary>
    private sealed class BRepCheck_ParallelAnalyzer
    {
        private readonly IndexedMap<Shape, ResultCheck?> _map;

        public BRepCheck_ParallelAnalyzer(IndexedMap<Shape, ResultCheck?> map)
        {
            _map = map;
        }

        /// <summary>A status list of a result, read under the result's lock in parallel mode.</summary>
        private static bool HasStatusOnShape(ResultCheck result, in Shape shape,
                                             Func<StatusCheck, bool> stop)
        {
            bool locked = result.IsParallel;
            if (locked)
            {
                result._mutex.Enter();
            }

            try
            {
                if (result.IsStatusOnShape(shape))
                {
                    var itl = new NCollection_List<StatusCheck>.Iterator(result.StatusOnShape(shape));
                    for (; itl.More(); itl.Next())
                    {
                        if (stop(itl.Value()))
                        {
                            return true;
                        }
                    }
                }

                return false;
            }
            finally
            {
                if (locked)
                {
                    result._mutex.Exit();
                }
            }
        }

        /// <summary>The sub-shapes from <paramref name="first"/> up to <paramref name="end"/>, in the map's order, here on this thread.</summary>
        public void Perform(int first, int end)
        {
            var mapS = new IndexedSet<Shape>(ShapeHasher.Instance);
            for (int i = first; i < end; ++i)
            {
                Perform(_map.KeyAt(i), _map.ValueAt(i), mapS);
            }
        }

        /// <summary>The given sub-shapes, here on this thread.</summary>
        public void Perform(Shape[] shapes)
        {
            var mapS = new IndexedSet<Shape>(ShapeHasher.Instance);
            for (int i = 0; i < shapes.Length; ++i)
            {
                Perform(shapes[i], _map.GetValue(shapes[i]), mapS);
            }
        }

        // mapS: scratch, kept from one sub-shape to the next
        private void Perform(in Shape shape, ResultCheck? result, IndexedSet<Shape> mapS)
        {
            var exp = new Explorer();
            ShapeType type = shape.Type;
            {
                switch (type)
                {
                    case ShapeType.Vertex:
                    {
                        // There is no need to check anything.
                        break;
                    }
                    case ShapeType.Edge:
                    {
                        try
                        {
                            var resEdge = (EdgeCheck)result!;
                            StatusCheck ste = resEdge.CheckPolygonOnTriangulation(ShapeCasts.Edge(shape));
                            if (ste != StatusCheck.NoError)
                            {
                                resEdge.SetStatus(ste);
                            }
                        }
                        catch (Exception exception) when (IsCaught(exception))
                        {
                            result?.SetFailStatus(shape);
                        }

                        mapS.Clear();
                        for (exp.Init(shape, ShapeType.Vertex); exp.More(); exp.Next())
                        {
                            Shape vertex = exp.Current();
                            ResultCheck? resOfVertex = _map.GetValue(vertex);
                            try
                            {
                                if (mapS.TryAdd(vertex))
                                {
                                    resOfVertex!.InContext(shape);
                                }
                            }
                            catch (Exception exception) when (IsCaught(exception))
                            {
                                result?.SetFailStatus(shape);

                                if (resOfVertex is not null)
                                {
                                    resOfVertex.SetFailStatus(vertex);
                                    resOfVertex.SetFailStatus(shape);
                                }
                            }
                        }

                        break;
                    }
                    case ShapeType.Wire:
                    {
                        break;
                    }
                    case ShapeType.Face:
                    {
                        mapS.Clear();
                        for (exp.Init(shape, ShapeType.Vertex); exp.More(); exp.Next())
                        {
                            ResultCheck? faceVertexRes = _map.GetValue(exp.Current());
                            try
                            {
                                if (mapS.TryAdd(exp.Current()))
                                {
                                    faceVertexRes!.InContext(shape);
                                }
                            }
                            catch (Exception exception) when (IsCaught(exception))
                            {
                                result?.SetFailStatus(shape);

                                if (faceVertexRes is not null)
                                {
                                    faceVertexRes.SetFailStatus(exp.Current());
                                    faceVertexRes.SetFailStatus(shape);
                                }
                            }
                        }

                        bool performwire = true;
                        bool isInvalidTolerance = false;
                        mapS.Clear();
                        for (exp.Init(shape, ShapeType.Edge); exp.More(); exp.Next())
                        {
                            ResultCheck? faceEdgeRes = _map.GetValue(exp.Current());
                            try
                            {
                                if (mapS.TryAdd(exp.Current()))
                                {
                                    faceEdgeRes!.InContext(shape);

                                    if (performwire)
                                    {
                                        if (HasStatusOnShape(faceEdgeRes,
                                                             shape,
                                                             ste => ste == StatusCheck.NoCurveOnSurface
                                                                    || ste == StatusCheck.InvalidCurveOnSurface
                                                                    || ste == StatusCheck.InvalidRange
                                                                    || ste == StatusCheck.InvalidCurveOnClosedSurface))
                                        {
                                            performwire = false;
                                        }
                                    }
                                }
                            }
                            catch (Exception exception) when (IsCaught(exception))
                            {
                                result?.SetFailStatus(shape);

                                if (faceEdgeRes is not null)
                                {
                                    faceEdgeRes.SetFailStatus(exp.Current());
                                    faceEdgeRes.SetFailStatus(shape);
                                }
                            }
                        }

                        bool orientofwires = performwire;
                        for (exp.Init(shape, ShapeType.Wire); exp.More(); exp.Next())
                        {
                            ResultCheck? faceWireRes = _map.GetValue(exp.Current());
                            try
                            {
                                faceWireRes!.InContext(shape);

                                if (orientofwires)
                                {
                                    if (HasStatusOnShape(faceWireRes, shape, ste => ste != StatusCheck.NoError))
                                    {
                                        orientofwires = false;
                                    }
                                }
                            }
                            catch (Exception exception) when (IsCaught(exception))
                            {
                                result?.SetFailStatus(shape);

                                if (faceWireRes is not null)
                                {
                                    faceWireRes.SetFailStatus(exp.Current());
                                    faceWireRes.SetFailStatus(shape);
                                }
                            }
                        }

                        try
                        {
                            var faceRes = (FaceCheck)result!;
                            if (isInvalidTolerance)
                            {
                                faceRes.SetStatus(StatusCheck.InvalidToleranceValue);
                            }
                            else if (performwire)
                            {
                                if (orientofwires)
                                {
                                    faceRes.OrientationOfWires(true); // on enregistre
                                }
                                else
                                {
                                    faceRes.SetUnorientable();
                                }
                            }
                            else
                            {
                                faceRes.SetUnorientable();
                            }
                        }
                        catch (Exception exception) when (IsCaught(exception))
                        {
                            result?.SetFailStatus(shape);

                            for (exp.Init(shape, ShapeType.Wire); exp.More(); exp.Next())
                            {
                                ResultCheck? faceCatchRes = _map.GetValue(exp.Current());
                                if (faceCatchRes is not null)
                                {
                                    faceCatchRes.SetFailStatus(exp.Current());
                                    faceCatchRes.SetFailStatus(shape);
                                    result!.SetFailStatus(exp.Current());
                                }
                            }
                        }

                        break;
                    }
                    case ShapeType.Shell:
                    {
                        break;
                    }
                    case ShapeType.Solid:
                    {
                        exp.Init(shape, ShapeType.Shell);
                        for (; exp.More(); exp.Next())
                        {
                            Shape shell = exp.Current();
                            ResultCheck? solidRes = _map.GetValue(shell);
                            try
                            {
                                solidRes!.InContext(shape);
                            }
                            catch (Exception exception) when (IsCaught(exception))
                            {
                                result?.SetFailStatus(shape);

                                if (solidRes is not null)
                                {
                                    solidRes.SetFailStatus(exp.Current());
                                    solidRes.SetFailStatus(shape);
                                }
                            }
                        }

                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
        }
    }
}
