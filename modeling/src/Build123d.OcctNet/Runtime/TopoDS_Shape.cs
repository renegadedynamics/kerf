using System.Runtime.CompilerServices;

namespace Build123d.Occt;

public partial class TopoDS_Shape
{
    // The shape. OCCT.NET freezes every shape it hands out; one being built by TopoDS_Builder stays open, and is
    // frozen when anything else first reads it
    private global::Occt.TopoDS_Shape _v;
    private bool _open;

    internal global::Occt.TopoDS_Shape V
    {
        get
        {
            if (_open)
                Close();
            return _v;
        }
        set
        {
            _v = value;
            _open = false;
        }
    }

    /// <summary>Starts building this shape: it stays open to the builder until something else reads it.</summary>
    internal void Open(global::Occt.TopoDS_Shape fresh)
    {
        _v = fresh;
        _open = true;
    }

    internal bool IsOpen => _open;

    /// <summary>The open shape, for the builder.</summary>
    internal ref global::Occt.TopoDS_Shape OpenValue => ref _v;

    private void Close()
    {
        if (_v.TShape() is { } tshape)
            Internals.FreezeGraph(tshape);
        _open = false;
    }

    // The in-place forms, internal to OCCT.NET, change this wrapper's own copy, as the native binding's do

    public void Nullify() => V = new global::Occt.TopoDS_Shape();

    public void Location(TopLoc_Location theLoc) => Location(theLoc, false);

    public void Location(TopLoc_Location theLoc, bool theRaiseExc)
    {
        ArgumentNullException.ThrowIfNull(theLoc);
        try
        {
            V = V.Located(theLoc.V, theRaiseExc);
        }
        catch (global::Occt.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }

    public void Orientation(TopAbs_Orientation theOrient) => V = V.Oriented((global::Occt.TopAbs_Orientation)theOrient);

    public void Move(TopLoc_Location thePosition) => Move(thePosition, false);

    public void Move(TopLoc_Location thePosition, bool theRaiseExc)
    {
        ArgumentNullException.ThrowIfNull(thePosition);
        try
        {
            V = V.Moved(thePosition.V, theRaiseExc);
        }
        catch (global::Occt.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }

    public void Reverse() => V = V.Reversed();

    public void Complement() => V = V.Complemented();

    public void Compose(TopAbs_Orientation theOrient) => V = V.Composed((global::Occt.TopAbs_Orientation)theOrient);

    /// <summary>
    /// <c>std::hash&lt;TopoDS_Shape&gt;</c>: equal for shapes that are <c>IsSame</c> (the same TShape at the
    /// same Location), whatever their orientation.
    /// </summary>
    public ulong StdHash() => unchecked((uint)V.GetHashCode());
}

/// <summary>
/// OCCT.NET members that aren't public, reached without reflection. Each is something OCCT.NET has no public way to do
/// yet (docs/design.md, "The OCCT.NET backend").
/// </summary>
internal static class Internals
{
    [UnsafeAccessor(UnsafeAccessorKind.Method, Name = "FreezeGraph")]
    internal static extern void FreezeGraph(global::Occt.TopoDS_TShape tshape);

    [UnsafeAccessor(UnsafeAccessorKind.Constructor)]
    internal static extern global::Occt.TopoDS_Builder NewBuilder();

    [UnsafeAccessor(UnsafeAccessorKind.Method, Name = "MakeWire")]
    internal static extern void MakeWire(global::Occt.TopoDS_Builder b, ref global::Occt.TopoDS_Wire w);

    [UnsafeAccessor(UnsafeAccessorKind.Method, Name = "MakeShell")]
    internal static extern void MakeShell(global::Occt.TopoDS_Builder b, ref global::Occt.TopoDS_Shell s);

    [UnsafeAccessor(UnsafeAccessorKind.Method, Name = "MakeSolid")]
    internal static extern void MakeSolid(global::Occt.TopoDS_Builder b, ref global::Occt.TopoDS_Solid s);

    [UnsafeAccessor(UnsafeAccessorKind.Method, Name = "MakeCompSolid")]
    internal static extern void MakeCompSolid(global::Occt.TopoDS_Builder b, ref global::Occt.TopoDS_CompSolid c);

    [UnsafeAccessor(UnsafeAccessorKind.Method, Name = "MakeCompound")]
    internal static extern void MakeCompound(global::Occt.TopoDS_Builder b, ref global::Occt.TopoDS_Compound c);

    [UnsafeAccessor(UnsafeAccessorKind.Method, Name = "Add")]
    internal static extern void Add(global::Occt.TopoDS_Builder b, in global::Occt.TopoDS_Shape shape, in global::Occt.TopoDS_Shape component);

    [UnsafeAccessor(UnsafeAccessorKind.Method, Name = "Remove")]
    internal static extern void Remove(global::Occt.TopoDS_Builder b, in global::Occt.TopoDS_Shape shape, in global::Occt.TopoDS_Shape component);

    // BRepTools_History has no public copy, and Merge into an empty history isn't one (it drops the removal of a shape
    // that also generated or was modified), so its maps are copied as they are, as the native shim's copy constructor does
    [UnsafeAccessor(UnsafeAccessorKind.Field, Name = "myShapeToModified")]
    internal static extern ref global::Occt.NCollection_DataMap<global::Occt.TopoDS_Shape, global::Occt.NCollection_List<global::Occt.TopoDS_Shape>> Modified(global::Occt.BRepTools_History h);

    [UnsafeAccessor(UnsafeAccessorKind.Field, Name = "myShapeToGenerated")]
    internal static extern ref global::Occt.NCollection_DataMap<global::Occt.TopoDS_Shape, global::Occt.NCollection_List<global::Occt.TopoDS_Shape>> Generated(global::Occt.BRepTools_History h);

    [UnsafeAccessor(UnsafeAccessorKind.Field, Name = "myRemoved")]
    internal static extern ref global::Occt.NCollection_Map<global::Occt.TopoDS_Shape> Removed(global::Occt.BRepTools_History h);
}
