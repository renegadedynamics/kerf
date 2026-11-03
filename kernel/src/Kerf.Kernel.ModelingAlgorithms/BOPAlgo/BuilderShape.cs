// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderShape.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;

namespace Kerf.Engine.Booleans;

/// <summary>The root of the Boolean component's algorithms that build a shape and its history.</summary>
/// <remarks>
/// <see cref="Shape"/> and <see cref="History"/> are publication points (decision 414): they freeze the result graph.
/// Ported code that goes on building reads <see cref="InternalShape"/> and <see cref="InternalHistory"/>
/// (decision 639). The history's lists are read-only lists (BRepTools_History's port).
/// </remarks>
public abstract class BuilderShape : BooleanAlgorithm
{
    protected Shape _shape = new();                                                //!< Result of the operation
    protected NCollection_List<Shape> _histShapes = new();                         //!< Storer for the history shapes
    protected HashSet<Shape> _mapShape = new(ShapeHasher.Instance); //!< cached map of all arguments shapes
    protected bool _fillHistory;                                                          //!< Controls the history filling
    protected History? _history;                                                //!< History tool

    protected BuilderShape()
    {
        _fillHistory = true;
    }

    /// <summary>The result of the operation (frozen).</summary>
    public Shape Shape()
    {
        _shape.TShape()?.FreezeGraph();
        return _shape;
    }

    /// <summary>The result, unfrozen, for ported code.</summary>
    internal Shape InternalShape() => _shape;

    /// <summary>The shapes modified from <paramref name="s"/>.</summary>
    public ISchematiqList<Shape> Modified(in Shape s)
    {
        if (_fillHistory && _history is not null)
        {
            return _history.Modified(s);
        }
        return PackedList<Shape>.Empty;
    }

    /// <summary>The shapes generated from <paramref name="s"/>.</summary>
    public ISchematiqList<Shape> Generated(in Shape s)
    {
        if (_fillHistory && _history is not null)
        {
            return _history.Generated(s);
        }
        return PackedList<Shape>.Empty;
    }

    /// <summary>True if <paramref name="s"/> has been deleted: neither it nor a split of it is in the result.</summary>
    public bool IsDeleted(in Shape s) => _fillHistory && _history is not null && _history.IsRemoved(s);

    public bool HasModified => _fillHistory && _history is not null && _history.HasModified;

    public bool HasGenerated => _fillHistory && _history is not null && _history.HasGenerated;

    public bool HasDeleted => _fillHistory && _history is not null && _history.HasRemoved;

    /// <summary>The history of the operation (null when not collected); the result graph is frozen first.</summary>
    public History? History()
    {
        _shape.TShape()?.FreezeGraph();
        return InternalHistory();
    }

    /// <summary>OCCT's <c>History()</c>, without freezing: makes an empty history if there is none.</summary>
    internal History? InternalHistory()
    {
        if (_fillHistory)
        {
            _history ??= new History();
            return _history;
        }
        return null;
    }

    /// <summary>Enables or disables the collection of the history.</summary>
    public void SetToFillHistory(bool histFlag) => _fillHistory = histFlag;

    public bool HasHistory => _fillHistory;

    public override void Clear()
    {
        base.Clear();
        _history = null;
        _mapShape.Clear();
    }
}
