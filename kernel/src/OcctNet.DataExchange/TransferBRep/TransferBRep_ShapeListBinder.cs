// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeListBinder.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeListBinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

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
        theres = new NCollection_HSequence<TopoDS_Shape>();
    }

    public TransferBRep_ShapeListBinder(NCollection_HSequence<TopoDS_Shape>? list)
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
    public void AddResult(in TopoDS_Shape shape)
    {
        theres.Append(shape);
    }

    public NCollection_HSequence<TopoDS_Shape>? Result()
    {
        return theres;
    }

    /// <summary>Changes an already defined sub-result.</summary>
    public void SetResult(int num, in TopoDS_Shape shape)
    {
        theres.SetValue(num, shape);
    }

    public int NbShapes()
    {
        return theres.Length();
    }

    public TopoDS_Shape Shape(int num)
    {
        return theres.Value(num);
    }

    public TopAbs_ShapeEnum ShapeType(int num)
    {
        return theres.Value(num).ShapeType();
    }

    public TopoDS_Vertex Vertex(int num)
    {
        return TopoDS.Vertex(theres.Value(num));
    }

    public TopoDS_Edge Edge(int num)
    {
        return TopoDS.Edge(theres.Value(num));
    }

    public TopoDS_Wire Wire(int num)
    {
        return TopoDS.Wire(theres.Value(num));
    }

    public TopoDS_Face Face(int num)
    {
        return TopoDS.Face(theres.Value(num));
    }

    public TopoDS_Shell Shell(int num)
    {
        return TopoDS.Shell(theres.Value(num));
    }

    public TopoDS_Solid Solid(int num)
    {
        return TopoDS.Solid(theres.Value(num));
    }

    public TopoDS_CompSolid CompSolid(int num)
    {
        return TopoDS.CompSolid(theres.Value(num));
    }

    public TopoDS_Compound Compound(int num)
    {
        return TopoDS.Compound(theres.Value(num));
    }

    private readonly NCollection_HSequence<TopoDS_Shape>? theres;
}
