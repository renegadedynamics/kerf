// SKELETON (unit 6H3): the signatures of ShapeFix_Shape, so that the forks porting ShapeFix_FixSmallFace and the
// others compile. Replaced by the port of ShapeFix_Shape.hxx/.lxx/.cxx.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

#pragma warning disable CS1591
public partial class ShapeFix_Shape : ShapeFix_Root
{
    public ShapeFix_Shape() => throw new NotImplementedException("6H3 skeleton");

    public ShapeFix_Shape(in TopoDS_Shape shape) => throw new NotImplementedException("6H3 skeleton");

    public void Init(in TopoDS_Shape shape) => throw new NotImplementedException("6H3 skeleton");

    public bool Perform(Message_ProgressRange theProgress = default) => throw new NotImplementedException("6H3 skeleton");

    /// <summary>The result, frozen.</summary>
    public TopoDS_Shape Shape() => throw new NotImplementedException("6H3 skeleton");

    /// <summary>The result, unfrozen, for ported callers working on shapes they own.</summary>
    internal TopoDS_Shape InternalShape() => throw new NotImplementedException("6H3 skeleton");

    public ShapeFix_Face FixFaceTool() => throw new NotImplementedException("6H3 skeleton");

    public ShapeFix_Wire FixWireTool() => throw new NotImplementedException("6H3 skeleton");

    public ShapeFix_Edge FixEdgeTool() => throw new NotImplementedException("6H3 skeleton");

    public bool Status(ShapeExtend_Status status) => throw new NotImplementedException("6H3 skeleton");

    public override void SetMsgRegistrator(ShapeExtend_BasicMsgRegistrator? msgreg) => throw new NotImplementedException("6H3 skeleton");

    public override void SetPrecision(double preci) => throw new NotImplementedException("6H3 skeleton");

    public override void SetMinTolerance(double mintol) => throw new NotImplementedException("6H3 skeleton");

    public override void SetMaxTolerance(double maxtol) => throw new NotImplementedException("6H3 skeleton");

    public ref int FixSolidMode() => throw new NotImplementedException("6H3 skeleton");
    public ref int FixFreeShellMode() => throw new NotImplementedException("6H3 skeleton");
    public ref int FixFreeFaceMode() => throw new NotImplementedException("6H3 skeleton");
    public ref int FixFreeWireMode() => throw new NotImplementedException("6H3 skeleton");
    public ref int FixSameParameterMode() => throw new NotImplementedException("6H3 skeleton");
    public ref int FixVertexPositionMode() => throw new NotImplementedException("6H3 skeleton");
    public ref int FixVertexTolMode() => throw new NotImplementedException("6H3 skeleton");
}
#pragma warning restore CS1591
