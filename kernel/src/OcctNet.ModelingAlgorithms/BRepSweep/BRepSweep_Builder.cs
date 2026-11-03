// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Builder.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Builder.lxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Brep;

using Kerf.Topology;

namespace Occt;

/// <summary>Builds the empty shapes of a sweep and adds sub-shapes to them, over a <see cref="BrepBuilder"/>.</summary>
/// <remarks>
/// OCCT casts the <c>TopoDS_Shape&amp;</c> argument to the type it makes (<c>TopoDS::Compound(aCompound)</c>); the port
/// makes the typed shape in a local and assigns it back.
/// </remarks>
internal sealed class BRepSweep_Builder
{
    private readonly BrepBuilder myBuilder;

    public BRepSweep_Builder(BrepBuilder aBuilder)
    {
        myBuilder = aBuilder;
    }

    public BrepBuilder Builder() => myBuilder;

    public void MakeCompound(ref Shape aCompound)
    {
        Compound C = new();
        myBuilder.MakeCompound(ref C);
        aCompound = C;
    }

    public void MakeCompSolid(ref Shape aCompSolid)
    {
        CompSolid C = new();
        myBuilder.MakeCompSolid(ref C);
        aCompSolid = C;
    }

    public void MakeSolid(ref Shape aSolid)
    {
        Solid S = new();
        myBuilder.MakeSolid(ref S);
        aSolid = S;
    }

    public void MakeShell(ref Shape aShell)
    {
        Shell S = new();
        myBuilder.MakeShell(ref S);
        aShell = S;
    }

    public void MakeWire(ref Shape aWire)
    {
        Wire W = new();
        myBuilder.MakeWire(ref W);
        aWire = W;
    }

    /// <summary>Adds the second shape, set to <paramref name="Orient"/>, to the first.</summary>
    public void Add(in Shape aShape1, in Shape aShape2, Orientation Orient)
    {
        Shape aComp = aShape2;
        aComp.Orientation(Orient);
        myBuilder.Add(aShape1, aComp);
    }

    /// <summary>Adds the second shape to the first.</summary>
    public void Add(in Shape aShape1, in Shape aShape2)
    {
        myBuilder.Add(aShape1, aShape2);
    }
}
