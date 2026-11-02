// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeBinder.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_ShapeBinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>A ShapeBinder is a BinderOfShape with some additional services to cast the Result under various kinds of Shapes.</summary>
public class TransferBRep_ShapeBinder : TransferBRep_BinderOfShape
{
    /// <summary>Creates an empty ShapeBinder.</summary>
    public TransferBRep_ShapeBinder()
    {
    }

    /// <summary>Creates a ShapeBinder with a result.</summary>
    public TransferBRep_ShapeBinder(in TopoDS_Shape shape)
        : base(shape)
    {
    }

    /// <summary>Returns the Type of the Shape Result (under TopAbs form).</summary>
    public TopAbs_ShapeEnum ShapeType()
    {
        if (!HasResult())
        {
            return TopAbs_SHAPE;
        }
        return Result().ShapeType();
    }

    public TopoDS_Vertex Vertex()
    {
        return TopoDS.Vertex(Result());
    }

    public TopoDS_Edge Edge()
    {
        return TopoDS.Edge(Result());
    }

    public TopoDS_Wire Wire()
    {
        return TopoDS.Wire(Result());
    }

    public TopoDS_Face Face()
    {
        return TopoDS.Face(Result());
    }

    public TopoDS_Shell Shell()
    {
        return TopoDS.Shell(Result());
    }

    public TopoDS_Solid Solid()
    {
        return TopoDS.Solid(Result());
    }

    public TopoDS_CompSolid CompSolid()
    {
        return TopoDS.CompSolid(Result());
    }

    public TopoDS_Compound Compound()
    {
        return TopoDS.Compound(Result());
    }
}
