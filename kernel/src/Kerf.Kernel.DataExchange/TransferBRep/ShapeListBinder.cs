// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeListBinder.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeListBinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Topology;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// This binder binds several (a list of) shapes with a starting entity, when this entity itself corresponds to a
/// simple list of shapes. Each part is not seen as a sub-result of an independent component, but as an item of a
/// built-in list.
/// </summary>
internal class ShapeListBinder : Binder
{
    public ShapeListBinder()
    {
        _theres = new NCollection_HSequence<Shape>();
    }

    public ShapeListBinder(NCollection_HSequence<Shape>? list)
    {
        _theres = list;
    }

    public override bool IsMultiple()
    {
        return ShapeCount > 1;
    }

    public override Type? ResultType()
    {
        return typeof(ShapeListBinder);
    }

    public override string ResultTypeName()
    {
        return "list(TopoDS_Shape)";
    }

    /// <summary>Adds an item to the result list.</summary>
    public void AddResult(in Shape shape)
    {
        _theres.Append(shape);
    }

    public NCollection_HSequence<Shape>? Result()
    {
        return _theres;
    }

    /// <summary>Changes an already defined sub-result.</summary>
    public void SetResult(int num, in Shape shape)
    {
        _theres.SetValue(num, shape);
    }

    public int ShapeCount
    {
        get
        {
            return _theres.Length();
        }
    }

    public Shape Shape(int num)
    {
        return _theres.Value(num);
    }

    public ShapeType ShapeType(int num)
    {
        return _theres.Value(num).Type;
    }

    public Vertex Vertex(int num)
    {
        return ShapeCasts.Vertex(_theres.Value(num));
    }

    public Edge Edge(int num)
    {
        return ShapeCasts.Edge(_theres.Value(num));
    }

    public Wire Wire(int num)
    {
        return ShapeCasts.Wire(_theres.Value(num));
    }

    public Face Face(int num)
    {
        return ShapeCasts.Face(_theres.Value(num));
    }

    public Shell Shell(int num)
    {
        return ShapeCasts.Shell(_theres.Value(num));
    }

    public Solid Solid(int num)
    {
        return ShapeCasts.Solid(_theres.Value(num));
    }

    public CompSolid CompSolid(int num)
    {
        return ShapeCasts.CompSolid(_theres.Value(num));
    }

    public Compound Compound(int num)
    {
        return ShapeCasts.Compound(_theres.Value(num));
    }

    private readonly NCollection_HSequence<Shape>? _theres;
}
