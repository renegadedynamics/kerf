// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Result.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Result.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Checks.StatusCheck;

namespace Kerf.Engine.Healing.Checks;

/// <summary>
/// The result of checking one shape: its own statuses, and its statuses in the context of the shapes that contain it
/// (<see cref="InContext"/>). The base of BRepCheck_Vertex, Edge, Wire, Face, Shell and Solid.
/// </summary>
/// <remarks>
/// A working object, as in OCCT: the status lists it returns are its own (OCCT's
/// <c>BRepCheck_ListOfStatus</c>, <c>NCollection_List&lt;BRepCheck_Status&gt;</c>). The context iterator visits the
/// contexts in the order they were checked (OCCT's map iterates in hash order, decision 18).
/// </remarks>
public abstract class ResultCheck : Transient
{
    protected Shape _shape;
    protected bool _min;
    protected bool _blind;
    protected bool _isParallel;

    /// <summary>The status lists of the shape (keyed by itself) and of its contexts.</summary>
    protected readonly NCollection_DataMap<Shape, NCollection_List<StatusCheck>> _map =
        new(ShapeHasher.Instance);

    /// <summary>OCCT's <c>std::mutex myMutex</c>, taken when <see cref="IsParallel"/>.</summary>
    /// <remarks>Internal as well: OCCT makes BRepCheck_ParallelAnalyzer a friend.</remarks>
    protected internal readonly Lock _mutex = new();

    // OCCT's map iterator: a snapshot of the map taken by InitContextIterator
    private KeyValuePair<Shape, NCollection_List<StatusCheck>>[] _iterItems = [];
    private int _iterIndex;

    protected ResultCheck()
    {
        _shape = new Shape();
        _min = false;
        _blind = false;
    }

    /// <summary>Starts a check of <paramref name="s"/>: clears the statuses and runs <see cref="Minimum"/>.</summary>
    public void Init(in Shape s)
    {
        _shape = s;
        _min = false;
        _blind = false;
        _map.Clear();
        Minimum();
    }

    /// <summary>Checks the shape in the context of <paramref name="contextShape"/>, which contains it.</summary>
    public abstract void InContext(in Shape contextShape);

    /// <summary>The checks of the shape on its own.</summary>
    public abstract void Minimum();

    /// <summary>The checks that need no context and aren't in the minimum.</summary>
    public abstract void Blind();

    /// <summary>Adds <see cref="CheckFail"/> to the statuses on <paramref name="s"/>.</summary>
    public void SetFailStatus(in Shape s)
    {
        bool locked = _isParallel;
        if (locked)
        {
            _mutex.Enter();
        }

        try
        {
            if (!_map.Find(s, out NCollection_List<StatusCheck>? list))
            {
                list = new NCollection_List<StatusCheck>();
                _map.Bind(s, list);
            }

            BRepCheckCheck.Add(list, StatusCheck.CheckFail);
        }
        finally
        {
            if (locked)
            {
                _mutex.Exit();
            }
        }
    }

    /// <summary>The statuses of the shape itself.</summary>
    public NCollection_List<StatusCheck> Status() => _map.Find(_shape);

    public bool IsMinimum => _min;

    public bool IsBlind => _blind;

    /// <summary>Starts iterating over the contexts in which the shape was checked.</summary>
    public void InitContextIterator()
    {
        _iterItems = [.. _map];
        _iterIndex = 0;
        // At least 1 element : the Shape itself
        // OCCT.NET: OCCT reads the key of an empty map; the port checks first
        if (MoreShapeInContext() && ContextualShape().IsSame(_shape))
        {
            _iterIndex++;
        }
    }

    public bool MoreShapeInContext() => _iterIndex < _iterItems.Length;

    public Shape ContextualShape() => _iterItems[_iterIndex].Key;

    public NCollection_List<StatusCheck> StatusOnShape() => _iterItems[_iterIndex].Value;

    public void NextShapeInContext()
    {
        _iterIndex++;
        if (MoreShapeInContext() && ContextualShape().IsSame(_shape))
        {
            _iterIndex++;
        }
    }

    /// <summary>Sets the parallel execution flag for sub-algorithms.</summary>
    public void SetParallel(bool isParallel) => _isParallel = isParallel;

    /// <summary>True if sub-algorithms should use parallel execution.</summary>
    public bool IsParallel => _isParallel;

    public bool IsStatusOnShape(in Shape shape) => _map.IsBound(shape);

    public NCollection_List<StatusCheck> StatusOnShape(in Shape shape) => _map.Find(shape);
}
