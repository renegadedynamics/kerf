namespace Build123d.Occt;

// OCCT.NET keeps its builders internal: a builder changes shapes in place, and OCCT.NET's shapes are frozen once
// handed out. A shape made here stays open while it's built, and is frozen the first time anything else reads it.

public partial class TopoDS_Builder
{
    public TopoDS_Builder() : this(Internals.NewBuilder(), true) { }

    public TopoDS_Builder(TopoDS_Builder other) : this(Internals.NewBuilder(), true) => ArgumentNullException.ThrowIfNull(other);

    public void MakeWire(TopoDS_Wire W)
    {
        ArgumentNullException.ThrowIfNull(W);
        var w = new global::Occt.TopoDS_Wire();
        Internals.MakeWire(Impl, ref w);
        W.Open(w);
    }

    public void MakeShell(TopoDS_Shell S)
    {
        ArgumentNullException.ThrowIfNull(S);
        var s = new global::Occt.TopoDS_Shell();
        Internals.MakeShell(Impl, ref s);
        S.Open(s);
    }

    public void MakeSolid(TopoDS_Solid S)
    {
        ArgumentNullException.ThrowIfNull(S);
        var s = new global::Occt.TopoDS_Solid();
        Internals.MakeSolid(Impl, ref s);
        S.Open(s);
    }

    public void MakeCompSolid(TopoDS_CompSolid C)
    {
        ArgumentNullException.ThrowIfNull(C);
        var c = new global::Occt.TopoDS_CompSolid();
        Internals.MakeCompSolid(Impl, ref c);
        C.Open(c);
    }

    public void MakeCompound(TopoDS_Compound C)
    {
        ArgumentNullException.ThrowIfNull(C);
        var c = new global::Occt.TopoDS_Compound();
        Internals.MakeCompound(Impl, ref c);
        C.Open(c);
    }

    public void Add(TopoDS_Shape S, TopoDS_Shape C)
    {
        ArgumentNullException.ThrowIfNull(S);
        ArgumentNullException.ThrowIfNull(C);
        var component = C.V;
        if (!S.IsOpen)
            throw new OcctException("TopoDS_FrozenShape", "TopoDS_Builder::Add: under OCCT.NET, only a shape this builder made can be added to");
        try
        {
            Internals.Add(Impl, in S.OpenValue, in component);
        }
        catch (global::Occt.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }

    public void Remove(TopoDS_Shape S, TopoDS_Shape C)
    {
        ArgumentNullException.ThrowIfNull(S);
        ArgumentNullException.ThrowIfNull(C);
        var component = C.V;
        if (!S.IsOpen)
            throw new OcctException("TopoDS_FrozenShape", "TopoDS_Builder::Remove: under OCCT.NET, only a shape this builder made can be removed from");
        try
        {
            Internals.Remove(Impl, in S.OpenValue, in component);
        }
        catch (global::Occt.Standard_Failure e)
        {
            throw OcctException.From(e);
        }
    }
}

public partial class BRep_Builder
{
    public BRep_Builder() : base(Internals.NewBuilder(), true) { }

    public BRep_Builder(BRep_Builder other) : base(Internals.NewBuilder(), true) => ArgumentNullException.ThrowIfNull(other);
}
