// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Builder.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Builder.lxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Builds the empty shapes of a sweep and adds sub-shapes to them, over a <see cref="BRep_Builder"/>.</summary>
/// <remarks>
/// OCCT casts the <c>TopoDS_Shape&amp;</c> argument to the type it makes (<c>TopoDS::Compound(aCompound)</c>); the port
/// makes the typed shape in a local and assigns it back.
/// </remarks>
internal sealed class BRepSweep_Builder
{
    private readonly BRep_Builder myBuilder;

    public BRepSweep_Builder(BRep_Builder aBuilder)
    {
        myBuilder = aBuilder;
    }

    public BRep_Builder Builder() => myBuilder;

    public void MakeCompound(ref TopoDS_Shape aCompound)
    {
        TopoDS_Compound C = new();
        myBuilder.MakeCompound(ref C);
        aCompound = C;
    }

    public void MakeCompSolid(ref TopoDS_Shape aCompSolid)
    {
        TopoDS_CompSolid C = new();
        myBuilder.MakeCompSolid(ref C);
        aCompSolid = C;
    }

    public void MakeSolid(ref TopoDS_Shape aSolid)
    {
        TopoDS_Solid S = new();
        myBuilder.MakeSolid(ref S);
        aSolid = S;
    }

    public void MakeShell(ref TopoDS_Shape aShell)
    {
        TopoDS_Shell S = new();
        myBuilder.MakeShell(ref S);
        aShell = S;
    }

    public void MakeWire(ref TopoDS_Shape aWire)
    {
        TopoDS_Wire W = new();
        myBuilder.MakeWire(ref W);
        aWire = W;
    }

    /// <summary>Adds the second shape, set to <paramref name="Orient"/>, to the first.</summary>
    public void Add(in TopoDS_Shape aShape1, in TopoDS_Shape aShape2, TopAbs_Orientation Orient)
    {
        TopoDS_Shape aComp = aShape2;
        aComp.Orientation(Orient);
        myBuilder.Add(aShape1, aComp);
    }

    /// <summary>Adds the second shape to the first.</summary>
    public void Add(in TopoDS_Shape aShape1, in TopoDS_Shape aShape2)
    {
        myBuilder.Add(aShape1, aShape2);
    }
}
