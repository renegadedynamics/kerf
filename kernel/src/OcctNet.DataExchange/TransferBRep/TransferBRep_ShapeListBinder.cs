// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeListBinder.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeListBinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Topology;

using Kerf.Topology;

namespace Occt;

/// <summary>
/// This binder binds several (a list of) shapes with a starting entity, when this entity itself corresponds to a
/// simple list of shapes. Each part is not seen as a sub-result of an independent component, but as an item of a
/// built-in list.
/// </summary>
public class TransferBRep_ShapeListBinder : Transfer_Binder
{
    public TransferBRep_ShapeListBinder()
    {
        theres = new NCollection_HSequence<Shape>();
    }

    public TransferBRep_ShapeListBinder(NCollection_HSequence<Shape>? list)
    {
        theres = list;
    }

    public override bool IsMultiple()
    {
        return NbShapes() > 1;
    }

    public override Type? ResultType()
    {
        return typeof(TransferBRep_ShapeListBinder);
    }

    public override string ResultTypeName()
    {
        return "list(TopoDS_Shape)";
    }

    /// <summary>Adds an item to the result list.</summary>
    public void AddResult(in Shape shape)
    {
        theres.Append(shape);
    }

    public NCollection_HSequence<Shape>? Result()
    {
        return theres;
    }

    /// <summary>Changes an already defined sub-result.</summary>
    public void SetResult(int num, in Shape shape)
    {
        theres.SetValue(num, shape);
    }

    public int NbShapes()
    {
        return theres.Length();
    }

    public Shape Shape(int num)
    {
        return theres.Value(num);
    }

    public ShapeType ShapeType(int num)
    {
        return theres.Value(num).ShapeType();
    }

    public Vertex Vertex(int num)
    {
        return ShapeCasts.Vertex(theres.Value(num));
    }

    public Edge Edge(int num)
    {
        return ShapeCasts.Edge(theres.Value(num));
    }

    public Wire Wire(int num)
    {
        return ShapeCasts.Wire(theres.Value(num));
    }

    public Face Face(int num)
    {
        return ShapeCasts.Face(theres.Value(num));
    }

    public Shell Shell(int num)
    {
        return ShapeCasts.Shell(theres.Value(num));
    }

    public Solid Solid(int num)
    {
        return ShapeCasts.Solid(theres.Value(num));
    }

    public CompSolid CompSolid(int num)
    {
        return ShapeCasts.CompSolid(theres.Value(num));
    }

    public Compound Compound(int num)
    {
        return ShapeCasts.Compound(theres.Value(num));
    }

    private readonly NCollection_HSequence<Shape>? theres;
}
