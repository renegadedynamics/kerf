// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FaceDivide.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FaceDivide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Divides a face (both its surface and its boundary) with criteria: the surface is split with the split surface tool
/// and the face rebuilt on the patches with ShapeFix_ComposeShell (giving a shell), then the edges of the resulting
/// faces are divided with the wire divide tool. The changes are recorded in the context (created if none is set).
/// </summary>
/// <remarks>
/// A working object (decision 21). The face given is not modified (decision 8402): ShapeFix_ComposeShell, which edits
/// the face's edges, works on a copy of a frozen face (decision 583) and gives back the caller's unchanged
/// sub-shapes; the wire divider only builds new shapes. <see cref="Result"/> freezes what it returns; ported callers
/// read <see cref="InternalResult"/>.
/// </remarks>
public class ShapeUpgrade_FaceDivide : ShapeUpgrade_Tool
{
    /// <summary>The face.</summary>
    protected TopoDS_Face myFace;
    /// <summary>The result (a face or a shell).</summary>
    protected TopoDS_Shape myResult;
    /// <summary>The mode for trimming (segmenting) the surface by the wire's UV bounds.</summary>
    protected bool mySegmentMode;
    /// <summary>The status.</summary>
    protected int myStatus;

    private ShapeUpgrade_SplitSurface? mySplitSurfaceTool;
    private ShapeUpgrade_WireDivide? myWireDivideTool;

    /// <summary>Empty constructor.</summary>
    public ShapeUpgrade_FaceDivide()
    {
        myFace = new TopoDS_Face();
        myResult = new TopoDS_Shape();
        myStatus = 0;
        mySegmentMode = true;
        mySplitSurfaceTool = new ShapeUpgrade_SplitSurface();
        myWireDivideTool = new ShapeUpgrade_WireDivide();
    }

    /// <summary>Initializes by a face.</summary>
    public ShapeUpgrade_FaceDivide(in TopoDS_Face F) : this()
    {
        Init(F);
    }

    /// <summary>Initializes by a face.</summary>
    public void Init(in TopoDS_Face F)
    {
        myResult = myFace = F;
    }

    /// <summary>Sets the mode for trimming (segmenting) the surface by the wire's UV bounds.</summary>
    public void SetSurfaceSegmentMode(bool Segment)
    {
        mySegmentMode = Segment;
    }

    /// <summary>
    /// Performs the splitting and computes the resulting shell (the context keeps track of former splittings); true if
    /// something was done.
    /// </summary>
    public virtual bool Perform(double theArea = 0.0)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (myFace.IsNull())
        {
            return false;
        }
        if (Context() is null)
        {
            SetContext(new ShapeBuild_ReShape());
        }
        myResult = myFace;
        SplitSurface(theArea);
        SplitCurves();
        return Status(ShapeExtend_DONE);
    }

    /// <summary>
    /// Splits the surface and rebuilds the face on the patches (with ShapeFix_ComposeShell); true if the surface was
    /// split.
    /// </summary>
    public virtual bool SplitSurface(double theArea = 0.0)
    {
        ShapeUpgrade_SplitSurface? SplitSurf = GetSplitSurfaceTool();
        if (SplitSurf is null)
        {
            return false;
        }

        // myResult should be face; else return with FAIL
        if (myResult.IsNull() || myResult.ShapeType() != TopAbs_FACE)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
            return false;
        }
        TopoDS_Face face = TopoDS.Face(myResult);

        Geom_Surface surf = BRep_Tool.Surface(face, out TopLoc_Location L)!;

        //  BRepTools::UVBounds(myFace,Uf,Ul,Vf,Vl);
        ShapeAnalysis.GetFaceUVBounds(face, out double Uf, out double Ul, out double Vf, out double Vl);
        if (Occt.Precision.IsInfinite(Uf) || Occt.Precision.IsInfinite(Ul) || Occt.Precision.IsInfinite(Vf)
            || Occt.Precision.IsInfinite(Vl))
        {
            return false;
        }

        // make little extension to ensure all pcurves fit inside new surface bounds
        surf.Bounds(out double aSUf, out double aSUl, out double aSVf, out double aSVl);
        if (!surf.IsUPeriodic())
        {
            double dU = (Ul - Uf) * 0.01;
            if (Uf > aSUf)
            {
                Uf -= Math.Min(dU, Uf - aSUf);
            }
            if (Ul < aSUl)
            {
                Ul += Math.Min(dU, aSUl - Ul);
            }
        }
        if (!surf.IsVPeriodic())
        {
            double dV = (Vl - Vf) * 0.01;
            if (Vf > aSVf)
            {
                Vf -= Math.Min(dV, Vf - aSVf);
            }
            if (Vl < aSVl)
            {
                Vl += Math.Min(dV, aSVl - Vl);
            }
        }

        SplitSurf.Init(surf, Uf, Ul, Vf, Vl, theArea);
        SplitSurf.Perform(mySegmentMode);

        // If surface was neither split nor modified, do nothing
        if (!SplitSurf.Status(ShapeExtend_DONE))
        {
            return false;
        }

        // if surface was modified, force copying all vertices (and edges as consequence)
        // to protect original shape from increasing tolerance after SameParameter
        if (SplitSurf.Status(ShapeExtend_DONE3))
        {
            for (var exp = new TopExp_Explorer(face, TopAbs_VERTEX); exp.More(); exp.Next())
            {
                if (Context()!.IsRecorded(exp.Current()))
                {
                    continue;
                }
                TopoDS_Shape emptyCopied = exp.Current().EmptyCopied();
                TopoDS_Vertex V = TopoDS.Vertex(emptyCopied);
                Context()!.Replace(exp.Current(), V);
            }
        }

        ShapeExtend_CompositeSurface Grid = SplitSurf.ResSurfaces()!;

        var CompShell = new ShapeFix_ComposeShell();
        CompShell.Init(Grid, L, face, Precision());
        CompShell.SetContext(Context());
        CompShell.SetMaxTolerance(MaxTolerance());
        ShapeUpgrade_WireDivide? SplitWire = GetWireDivideTool();
        if (SplitWire is not null)
        {
            CompShell.SetTransferParamTool(GetWireDivideTool()!.GetTransferParamTool()!);
        }

        CompShell.Perform();
        if (CompShell.Status(ShapeExtend_FAIL) || !CompShell.Status(ShapeExtend_DONE))
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
        }

        // OCCT.NET: the result unfrozen, the caller's sub-shapes given back where the face was copied (decision 8402)
        myResult = CompShell.InternalFinishedResult();
        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        return true;
    }

    /// <summary>Divides the edges of the faces of the result with the wire divide tool; true if something was done.</summary>
    public virtual bool SplitCurves()
    {
        ShapeUpgrade_WireDivide? SplitWire = GetWireDivideTool();
        if (SplitWire is null)
        {
            return false;
        }

        SplitWire.SetMaxTolerance(MaxTolerance());
        for (var explf = new TopExp_Explorer(myResult, TopAbs_FACE); explf.More(); explf.Next())
        {
            TopoDS_Shape S = Context()!.InternalApply(explf.Current(), TopAbs_SHAPE);

            // S should be face; else return with FAIL
            if (S.IsNull() || S.ShapeType() != TopAbs_FACE)
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
                return false;
            }

            TopoDS_Face F = TopoDS.Face(S);
            SplitWire.SetFace(F);
            for (var wi = new TopoDS_Iterator(F, false); wi.More(); wi.Next())
            {
                // TopoDS_Wire wire = TopoDS::Wire ( wi.Value() );
                //  modifications already defined in context are to be applied inside SplitWire
                if (wi.Value().ShapeType() != TopAbs_WIRE)
                {
                    continue;
                }

                TopoDS_Wire wire = TopoDS.Wire(wi.Value());
                SplitWire.Load(wire);
                SplitWire.SetContext(Context());
                SplitWire.Perform();
                if (SplitWire.Status(ShapeExtend_FAIL))
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                }
                if (SplitWire.Status(ShapeExtend_DONE))
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                    Context()!.Replace(wire, SplitWire.InternalWire());
                }
            }
        }
        myResult = Context()!.InternalApply(myResult);
        return Status(ShapeExtend_DONE);
    }

    /// <summary>Gives the resulting shell or face (the initial face if not done, null if not initialized), frozen.</summary>
    public TopoDS_Shape Result()
    {
        myResult.TShape()?.FreezeGraph();
        return myResult;
    }

    /// <summary><see cref="Result"/> without freezing it, for ported callers.</summary>
    internal TopoDS_Shape InternalResult() => myResult;

    /// <summary>
    /// Queries the status of the last <see cref="Perform"/>: OK (no splitting done), DONE1 (some edges split), DONE2
    /// (the surface split), FAIL1 (errors in splitting edges), FAIL2 (errors in splitting the surface), FAIL3 (the
    /// result isn't a face).
    /// </summary>
    public bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatus, status);

    /// <summary>Sets the tool for splitting the surface.</summary>
    public void SetSplitSurfaceTool(ShapeUpgrade_SplitSurface? splitSurfaceTool)
    {
        mySplitSurfaceTool = splitSurfaceTool;
    }

    /// <summary>Sets the tool for dividing the edges (none: no division of the edges).</summary>
    public void SetWireDivideTool(ShapeUpgrade_WireDivide? wireDivideTool)
    {
        myWireDivideTool = wireDivideTool;
    }

    /// <summary>Returns the tool for splitting the surface.</summary>
    public virtual ShapeUpgrade_SplitSurface? GetSplitSurfaceTool() => mySplitSurfaceTool;

    /// <summary>Returns the tool for dividing the edges.</summary>
    public virtual ShapeUpgrade_WireDivide? GetWireDivideTool() => myWireDivideTool;
}
