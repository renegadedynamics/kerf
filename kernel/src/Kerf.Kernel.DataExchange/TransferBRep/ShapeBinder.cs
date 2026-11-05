// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeBinder.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeBinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>A ShapeBinder is a BinderOfShape with some additional services to cast the Result under various kinds of Shapes.</summary>
internal class ShapeBinder : BinderOfShape
{
    /// <summary>Creates an empty ShapeBinder.</summary>
    public ShapeBinder()
    {
    }

    /// <summary>Creates a ShapeBinder with a result.</summary>
    public ShapeBinder(in Shape shape)
        : base(shape)
    {
    }

    /// <summary>Returns the Type of the Shape Result (under TopAbs form).</summary>
    public ShapeType ShapeType()
    {
        if (!HasResult)
        {
            return global::Kerf.Topology.ShapeType.Shape;
        }
        return Result().Type;
    }

    public Vertex Vertex()
    {
        return ShapeCasts.Vertex(Result());
    }

    public Edge Edge()
    {
        return ShapeCasts.Edge(Result());
    }

    public Wire Wire()
    {
        return ShapeCasts.Wire(Result());
    }

    public Face Face()
    {
        return ShapeCasts.Face(Result());
    }

    public Shell Shell()
    {
        return ShapeCasts.Shell(Result());
    }

    public Solid Solid()
    {
        return ShapeCasts.Solid(Result());
    }

    public CompSolid CompSolid()
    {
        return ShapeCasts.CompSolid(Result());
    }

    public Compound Compound()
    {
        return ShapeCasts.Compound(Result());
    }
}
