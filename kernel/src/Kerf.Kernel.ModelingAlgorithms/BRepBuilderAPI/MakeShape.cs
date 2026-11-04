// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeShape.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;

namespace Kerf.Engine.Construction.Api;

/// <summary>The root of the BRepBuilderAPI algorithms that build a shape.</summary>
/// <remarks>
/// <para><see cref="Shape"/>, <see cref="Generated"/> and <see cref="Modified"/> are publication points
/// (docs/phase3-design.md, section 4.2): they freeze what they return. OCCT's virtuals are the internal forms
/// <see cref="InternalShape"/>, <see cref="InternalGenerated"/> and <see cref="InternalModified"/>, which don't freeze;
/// derived algorithms override those, and ported code that goes on building on a result reads them.</para>
/// <para>The algorithm is a <see cref="History.IAlgorithm"/>, so its history can be collected with
/// <c>new BRepTools_History(arguments, algorithm)</c> or merged with <see cref="History.Merge(NCollection_List{Shape}, History.IAlgorithm)"/>.</para>
/// </remarks>
internal abstract class MakeShape : Command, History.IAlgorithm
{
    protected Shape _shape = new();
    protected NCollection_List<Shape> _generated = new();

    protected MakeShape()
    {
    }

    /// <summary>This is called by <see cref="Shape"/>. It does nothing but may be redefined.</summary>
    public virtual void Build(ProgressRange range = default)
    {
    }

    /// <summary>
    /// The shape built by the algorithm (frozen). Builds it if the command isn't done.
    /// </summary>
    /// <exception cref="NotDoneException">The shape was not built.</exception>
    public Shape Shape()
    {
        Shape shape = InternalShape();
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary><see cref="Shape"/> without freezing (OCCT's virtual), for ported code and derived algorithms.</summary>
    internal virtual Shape InternalShape()
    {
        if (!IsDone)
        {
            Build();
            Check();
        }
        return _shape;
    }

    /// <summary>The result (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Shape(MakeShape maker) => maker.Shape();

    /// <summary>The shapes generated from the shape <paramref name="s"/> (frozen).</summary>
    public ISchematiqList<Shape> Generated(in Shape s)
    {
        BeforePublishingLists();
        return PublishedList(InternalGenerated(s));
    }

    /// <summary>The algorithm's own list behind <see cref="Generated"/>, unfrozen (OCCT's virtual).</summary>
    internal virtual NCollection_List<Shape> InternalGenerated(in Shape s)
    {
        _generated.Clear();
        return _generated;
    }

    /// <summary>The shapes modified from the shape <paramref name="s"/> (frozen).</summary>
    public ISchematiqList<Shape> Modified(in Shape s)
    {
        BeforePublishingLists();
        return PublishedList(InternalModified(s));
    }

    /// <summary>
    /// Called by <see cref="Generated"/> and <see cref="Modified"/> before they publish: an algorithm whose lists hold
    /// sub-shapes of its result freezes the whole result here (decision 414: a sub-shape accessor freezes the whole
    /// result with it). Does nothing by default.
    /// </summary>
    private protected virtual void BeforePublishingLists()
    {
    }

    /// <summary>The algorithm's own list behind <see cref="Modified"/>, unfrozen (OCCT's virtual).</summary>
    internal virtual NCollection_List<Shape> InternalModified(in Shape s)
    {
        _generated.Clear();
        return _generated;
    }

    /// <summary>True if the shape <paramref name="s"/> has been deleted.</summary>
    public virtual bool IsDeleted(in Shape s)
    {
        return false;
    }

    bool History.IAlgorithm.IsDeleted(in Shape s) => IsDeleted(s);

    IEnumerable<Shape> History.IAlgorithm.Modified(in Shape s) => Modified(s);

    IEnumerable<Shape> History.IAlgorithm.Generated(in Shape s) => Generated(s);

    // A frozen copy of a list of shapes, each shape's graph frozen.
    private protected static ISchematiqList<Shape> PublishedList(NCollection_List<Shape> list)
    {
        if (list.IsEmpty())
        {
            return PackedList<Shape>.Empty;
        }
        var shapes = new Shape[list.Extent()];
        int i = 0;
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            shapes[i] = it.Value();
            shapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return PackedList.Own(shapes);
    }
}
