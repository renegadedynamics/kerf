// SKELETON (unit 6H3): the signatures of ShapeFix_Face, so that the forks porting ShapeFix_Shell, Shape, FixSmallFace
// and FaceConnect compile. Replaced by the port of ShapeFix_Face.hxx/.lxx/.cxx.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

#pragma warning disable CS1591
public partial class ShapeFix_Face : ShapeFix_Root
{
    protected ShapeAnalysis_Surface? mySurf;
    protected TopoDS_Face            myFace;
    protected TopoDS_Shape           myResult;
    protected ShapeFix_Wire          myFixWire;
    protected bool                   myFwd;
    protected int                    myStatus;

    private int myFixWireMode;
    private int myFixOrientationMode;
    private int myFixAddNaturalBoundMode;
    private int myFixMissingSeamMode;
    private int myFixSmallAreaWireMode;
    private int myRemoveSmallAreaFaceMode;
    private int myFixLoopWiresMode;
    private int myFixIntersectingWiresMode;
    private int myFixSplitFaceMode;
    private int myAutoCorrectPrecisionMode;
    private int myFixPeriodicDegenerated;

    public ShapeFix_Face()
    {
        myFace    = new TopoDS_Face();
        myResult  = new TopoDS_Shape();
        myFixWire = new ShapeFix_Wire();
        throw new NotImplementedException("6H3 skeleton");
    }

    public ShapeFix_Face(in TopoDS_Face face) : this() => throw new NotImplementedException("6H3 skeleton");

    public virtual void ClearModes() => throw new NotImplementedException("6H3 skeleton");

    public void Init(in TopoDS_Face face) => throw new NotImplementedException("6H3 skeleton");

    public void Init(Geom_Surface surf, double preci, bool fwd = true) => throw new NotImplementedException("6H3 skeleton");

    public void Init(ShapeAnalysis_Surface surf, double preci, bool fwd = true) => throw new NotImplementedException("6H3 skeleton");

    public override void SetMsgRegistrator(ShapeExtend_BasicMsgRegistrator? msgreg) => throw new NotImplementedException("6H3 skeleton");

    public override void SetPrecision(double preci) => throw new NotImplementedException("6H3 skeleton");

    public override void SetMinTolerance(double mintol) => throw new NotImplementedException("6H3 skeleton");

    public override void SetMaxTolerance(double maxtol) => throw new NotImplementedException("6H3 skeleton");

    public ref int FixWireMode() => ref myFixWireMode;
    public ref int FixOrientationMode() => ref myFixOrientationMode;
    public ref int FixAddNaturalBoundMode() => ref myFixAddNaturalBoundMode;
    public ref int FixMissingSeamMode() => ref myFixMissingSeamMode;
    public ref int FixSmallAreaWireMode() => ref myFixSmallAreaWireMode;
    public ref int RemoveSmallAreaFaceMode() => ref myRemoveSmallAreaFaceMode;
    public ref int FixIntersectingWiresMode() => ref myFixIntersectingWiresMode;
    public ref int FixLoopWiresMode() => ref myFixLoopWiresMode;
    public ref int FixSplitFaceMode() => ref myFixSplitFaceMode;
    public ref int AutoCorrectPrecisionMode() => ref myAutoCorrectPrecisionMode;
    public ref int FixPeriodicDegeneratedMode() => ref myFixPeriodicDegenerated;

    /// <summary>The fixed face, frozen (the public result).</summary>
    public TopoDS_Face Face() => throw new NotImplementedException("6H3 skeleton");

    /// <summary>The result (a face or a shell), frozen.</summary>
    public TopoDS_Shape Result() => throw new NotImplementedException("6H3 skeleton");

    /// <summary>The fixed face, unfrozen, for ported callers working on shapes they own.</summary>
    internal TopoDS_Face InternalFace() => myFace;

    /// <summary>The result, unfrozen, for ported callers working on shapes they own.</summary>
    internal TopoDS_Shape InternalResult() => myResult;

    public void Add(in TopoDS_Wire wire) => throw new NotImplementedException("6H3 skeleton");

    public bool Perform(Message_ProgressRange theProgress = default) => throw new NotImplementedException("6H3 skeleton");

    public bool FixOrientation() => throw new NotImplementedException("6H3 skeleton");

    public bool FixOrientation(NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MapWires)
        => throw new NotImplementedException("6H3 skeleton");

    public bool FixAddNaturalBound() => throw new NotImplementedException("6H3 skeleton");

    public bool FixMissingSeam() => throw new NotImplementedException("6H3 skeleton");

    public bool FixSmallAreaWire(bool theIsRemoveSmallFace) => throw new NotImplementedException("6H3 skeleton");

    public bool FixLoopWire(NCollection_Sequence<TopoDS_Shape> aResWires) => throw new NotImplementedException("6H3 skeleton");

    public bool FixIntersectingWires() => throw new NotImplementedException("6H3 skeleton");

    public bool FixWiresTwoCoincEdges() => throw new NotImplementedException("6H3 skeleton");

    public bool FixSplitFace(NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MapWires)
        => throw new NotImplementedException("6H3 skeleton");

    public bool FixPeriodicDegenerated() => throw new NotImplementedException("6H3 skeleton");

    public bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatus, status);

    public ShapeFix_Wire FixWireTool() => myFixWire;
}
#pragma warning restore CS1591
