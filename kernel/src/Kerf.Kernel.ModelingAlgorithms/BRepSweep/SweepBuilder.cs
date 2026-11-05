// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Builder.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Builder.lxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Brep;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Sweeping;

/// <summary>Builds the empty shapes of a sweep and adds sub-shapes to them, over a <see cref="BrepBuilder"/>.</summary>
/// <remarks>
/// OCCT casts the <c>TopoDS_Shape&amp;</c> argument to the type it makes (<c>TopoDS::Compound(aCompound)</c>); the port
/// makes the typed shape in a local and assigns it back.
/// </remarks>
internal sealed class SweepBuilder
{
    private readonly BrepBuilder _builder;

    public SweepBuilder(BrepBuilder aBuilder)
    {
        _builder = aBuilder;
    }

    public BrepBuilder Builder() => _builder;

    public void MakeCompound(ref Shape aCompound)
    {
        Compound c = new();
        _builder.MakeCompound(ref c);
        aCompound = c;
    }

    public void MakeCompSolid(ref Shape aCompSolid)
    {
        CompSolid c = new();
        _builder.MakeCompSolid(ref c);
        aCompSolid = c;
    }

    public void MakeSolid(ref Shape aSolid)
    {
        Solid s = new();
        _builder.MakeSolid(ref s);
        aSolid = s;
    }

    public void MakeShell(ref Shape aShell)
    {
        Shell s = new();
        _builder.MakeShell(ref s);
        aShell = s;
    }

    public void MakeWire(ref Shape aWire)
    {
        Wire w = new();
        _builder.MakeWire(ref w);
        aWire = w;
    }

    /// <summary>Adds the second shape, set to <paramref name="orient"/>, to the first.</summary>
    public void Add(in Shape aShape1, in Shape aShape2, Orientation orient)
    {
        Shape comp = aShape2;
        comp.Orient(orient);
        _builder.Add(aShape1, comp);
    }

    /// <summary>Adds the second shape to the first.</summary>
    public void Add(in Shape aShape1, in Shape aShape2)
    {
        _builder.Add(aShape1, aShape2);
    }
}
