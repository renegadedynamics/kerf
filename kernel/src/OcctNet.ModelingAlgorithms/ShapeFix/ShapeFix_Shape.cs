// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Shape.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Shape.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Shape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Fixes a shape of any type: dispatches it, by type, to ShapeFix_Solid, ShapeFix_Shell, ShapeFix_Face, ShapeFix_Wire
/// or ShapeFix_Edge (going down a compound or a compsolid), then makes the edges SameParameter and checks the
/// tolerances of the vertices.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). <see cref="Init"/> records the shape (and makes a context considering locations if
/// none was set). <see cref="Perform"/> edits the shape it works on in place, as OCCT's: a shape holding frozen shapes
/// (any shape a caller holds) is copied first (<see cref="BRepTools_CopyAtEntry"/>, decision 583), so the caller's
/// shape never changes. At the end of <see cref="Perform"/> the copies the fixes left unchanged are the caller's own
/// again, in the result and in the context, and the context records the sub-shapes changed in place, so it answers for
/// the caller's shapes. A shape needing nothing gives back the input itself.
/// </para>
/// <para>
/// <see cref="Shape"/> freezes the result; ported code fixing a shape it owns reads <see cref="InternalShape"/>. The
/// tools (<see cref="FixSolidTool"/> down to <see cref="FixEdgeTool"/>) are this tool's, as OCCT's: set their modes
/// before <see cref="Perform"/>.
/// </para>
/// </remarks>
public class ShapeFix_Shape : ShapeFix_Root
{
    /// <summary>The result.</summary>
    protected TopoDS_Shape                  myResult;
    /// <summary>The solid fixing tool (holding the shell, face, wire and edge tools).</summary>
    protected ShapeFix_Solid                myFixSolid;
    /// <summary>The shapes (without location) already fixed by this tool.</summary>
    protected NCollection_Map<TopoDS_Shape> myMapFixingShape;
    /// <summary>The mode of the solid fixes.</summary>
    protected int                           myFixSolidMode;
    /// <summary>The mode of the fixes of free shells.</summary>
    protected int                           myFixShellMode;
    /// <summary>The mode of the fixes of free faces.</summary>
    protected int                           myFixFaceMode;
    /// <summary>The mode of the fixes of free wires.</summary>
    protected int                           myFixWireMode;
    /// <summary>The mode of the SameParameter fix.</summary>
    protected int                           myFixSameParameterMode;
    /// <summary>The mode of the fix of the vertex positions.</summary>
    protected int                           myFixVertexPositionMode;
    /// <summary>The mode of the fix of the vertex tolerances.</summary>
    protected int                           myFixVertexTolMode;
    /// <summary>The status.</summary>
    protected int                           myStatus;

    // The shapes added to myMapFixingShape during the current Perform (keys over the working copy).
    private List<TopoDS_Shape>? myAddedKeys;

    /// <summary>Empty constructor.</summary>
    public ShapeFix_Shape()
    {
        myResult                = new TopoDS_Shape();
        myMapFixingShape        = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        myStatus                = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myFixSolidMode          = -1;
        myFixShellMode          = -1;
        myFixFaceMode           = -1;
        myFixWireMode           = -1;
        myFixSameParameterMode  = -1;
        myFixVertexPositionMode = 0;
        myFixVertexTolMode      = -1;
        myFixSolid              = new ShapeFix_Solid();
    }

    /// <summary>Initializes by a shape.</summary>
    public ShapeFix_Shape(in TopoDS_Shape shape)
    {
        myResult                = new TopoDS_Shape();
        myMapFixingShape        = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        myStatus                = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myFixSolidMode          = -1;
        myFixShellMode          = -1;
        myFixFaceMode           = -1;
        myFixWireMode           = -1;
        myFixSameParameterMode  = -1;
        myFixSolid              = new ShapeFix_Solid();
        myFixVertexPositionMode = 0;
        myFixVertexTolMode      = -1;
        Init(shape);
    }

    /// <summary>Initializes by a shape; makes a context considering locations if none was set.</summary>
    public void Init(in TopoDS_Shape shape)
    {
        myShape = shape;
        if (Context() is null)
        {
            SetContext(new ShapeBuild_ReShape());
            Context()!.ModeConsiderLocation(true);
        }
        myResult = myShape;
    }

    /// <summary>
    /// Fixes the shape (by type, see the class), then makes its edges SameParameter and checks the tolerances of its
    /// vertices, as the modes allow. Returns true if something was done.
    /// </summary>
    public bool Perform(Message_ProgressRange theProgress = default)
    {
        // OCCT's early return (a shape this tool already fixed, or one the context made) runs on the caller's shape
        TopoDS_Shape aShapeNullLoc = myShape.Located(new TopLoc_Location());
        if (Context()!.IsNewShape(myShape) || myMapFixingShape.Contains(aShapeNullLoc))
        {
            return perform(theProgress);
        }

        var          anEntry = new BRepTools_CopyAtEntry(myShape, Context());
        TopoDS_Shape anInput = myShape;
        myShape     = anEntry.Shape();
        myAddedKeys = [];
        bool status = perform(theProgress);
        myShape  = anInput;
        myResult = anEntry.Finish(myResult);

        // the shapes fixed, as the caller's (a further Perform on them takes OCCT's early return)
        if (anEntry.IsCopied())
        {
            foreach (TopoDS_Shape aKey in myAddedKeys)
            {
                myMapFixingShape.Remove(aKey);
                myMapFixingShape.Add(anEntry.Original(aKey));
            }
        }
        myAddedKeys = null;
        return status;
    }

    // ShapeFix_Shape::Perform on the working shape (recursive over compounds)
    private bool perform(Message_ProgressRange theProgress)
    {
        ShapeBuild_ReShape aContext                = Context()!;
        int                savFixSmallAreaWireMode = 0;
        int                savFixVertexTolMode     = myFixVertexTolMode;
        ShapeFix_Face      fft                     = FixFaceTool();
        savFixSmallAreaWireMode = fft.FixSmallAreaWireMode();
        if (savFixSmallAreaWireMode == -1 && myShape.ShapeType() == TopAbs_FACE)
        {
            fft.FixSmallAreaWireMode() = 1;
        }

        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        bool             status = false;
        TopAbs_ShapeEnum st;

        // gka fix for sharing assembly
        TopLoc_Location nullLoc = new(), L;
        L = myShape.Location();
        TopoDS_Shape aShapeNullLoc = myShape;
        bool         aIsRecorded   = aContext.IsNewShape(myShape);
        aShapeNullLoc.Location(nullLoc);
        if (aIsRecorded || myMapFixingShape.Contains(aShapeNullLoc))
        {
            myShape.Location(L, false);
            myResult = aContext.InternalApply(myShape);
            status   = true;
            return status;
        }
        myMapFixingShape.Add(aShapeNullLoc);
        myAddedKeys?.Add(aShapeNullLoc);
        //---------------------------------------
        myShape.Location(L, false);
        TopoDS_Shape S = aContext.InternalApply(myShape);
        if (NeedFix(myFixVertexPositionMode))
        {
            ShapeFix.FixVertexPosition(ref S, Precision(), aContext);
        }

        st = S.ShapeType();

        // Open progress indication scope for the following fix stages:
        // - Fix on Solid or Shell;
        // - Fix same parameterization;
        using var aPS = new Message_ProgressScope(theProgress, "Fixing stage", 2);

        switch (st)
        {
            case TopAbs_COMPOUND:
            case TopAbs_COMPSOLID:
            {
                TopoDS_Shape shape                   = myShape;
                int          savFixSameParameterMode = myFixSameParameterMode;
                myFixSameParameterMode = 0;
                myFixVertexTolMode     = 0;
                int aShapesNb          = S.NbChildren();

                // Open progress indication scope for sub-shape fixing
                using var aPSSubShape = new Message_ProgressScope(aPS.Next(), "Fixing sub-shape", aShapesNb);
                for (var anIter = new TopoDS_Iterator(S); anIter.More() && aPSSubShape.More(); anIter.Next())
                {
                    myShape = anIter.Value();
                    if (perform(aPSSubShape.Next()))
                    {
                        status = true;
                    }
                }
                if (!aPSSubShape.More())
                {
                    return false; // aborted execution
                }
                myFixSameParameterMode = savFixSameParameterMode;
                myFixVertexTolMode     = savFixVertexTolMode;
                myShape                = shape;
                break;
            }
            case TopAbs_SOLID:
            {
                if (!NeedFix(myFixSolidMode))
                {
                    break;
                }
                myFixSolid.Init(TopoDS.Solid(S));
                myFixSolid.SetContext(aContext);

                if (myFixSolid.Perform(aPS.Next()))
                {
                    status = true;
                }

                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                break;
            }
            case TopAbs_SHELL:
            {
                if (!NeedFix(myFixShellMode))
                {
                    break;
                }
                ShapeFix_Shell sfsh = FixShellTool();
                sfsh.Init(TopoDS.Shell(S));
                sfsh.SetContext(aContext);

                if (sfsh.Perform(aPS.Next()))
                {
                    status = true;
                }

                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                break;
            }
            case TopAbs_FACE:
            {
                if (!NeedFix(myFixFaceMode))
                {
                    break;
                }
                ShapeFix_Face sff         = FixFaceTool();
                bool          savTopoMode = sff.FixWireTool().ModifyTopologyMode();
                sff.FixWireTool().ModifyTopologyMode() = true;
                sff.Init(TopoDS.Face(S));
                sff.SetContext(aContext);

                if (sff.Perform())
                {
                    status = true;
                }
                sff.FixWireTool().ModifyTopologyMode() = savTopoMode;

                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                break;
            }
            case TopAbs_WIRE:
            {
                if (!NeedFix(myFixWireMode))
                {
                    break;
                }
                ShapeFix_Wire sfw           = FixWireTool();
                bool          savTopoMode   = sfw.ModifyTopologyMode();
                bool          savClosedMode = sfw.ClosedWireMode();
                sfw.ModifyTopologyMode() = true;
                if (!S.Closed())
                {
                    sfw.ClosedWireMode() = false;
                }
                sfw.SetFace(new TopoDS_Face());
                sfw.Load(TopoDS.Wire(S));
                sfw.SetContext(aContext);
                if (sfw.Perform())
                {
                    status = true;
                    aContext.Replace(S, sfw.InternalWire()); // replace for wire only
                }
                sfw.ModifyTopologyMode() = savTopoMode;
                sfw.ClosedWireMode()     = savClosedMode;
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                break;
            }
            case TopAbs_EDGE:
            {
                ShapeFix_Edge sfe = FixEdgeTool();
                sfe.SetContext(aContext);
                if (sfe.FixVertexTolerance(TopoDS.Edge(S)))
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                }
                break;
            }
            case TopAbs_VERTEX:
            case TopAbs_SHAPE:
            default:
                break;
        }
        if (!aPS.More())
        {
            return false; // aborted execution
        }

        myResult = aContext.InternalApply(S);

        if (NeedFix(myFixSameParameterMode))
        {
            SameParameter(myResult, false, aPS.Next());

            if (!aPS.More())
            {
                return false; // aborted execution
            }
        }
        if (NeedFix(myFixVertexTolMode))
        {
            int nbF    = 0;
            var anExpF = new TopExp_Explorer(myResult, TopAbs_FACE);
            for (; anExpF.More() && nbF <= 1; anExpF.Next())
            {
                nbF++;
            }
            if (nbF > 1)
            {
                // fix for bug  0025455
                //  for case when vertex belong to the different faces it is necessary to check vertices
                //  tolerances
                // after all fixes.
                // This fix it should be performed for example for case when cutting edge was performed.
                ShapeFix_Edge sfe = FixEdgeTool();
                for (anExpF.ReInit(); anExpF.More(); anExpF.Next())
                {
                    TopoDS_Face aF     = TopoDS.Face(anExpF.Current());
                    var         anExpE = new TopExp_Explorer(aF, TopAbs_EDGE);
                    for (; anExpE.More(); anExpE.Next())
                    {
                        sfe.FixVertexTolerance(TopoDS.Edge(anExpE.Current()), aF);
                    }
                }
            }
        }

        myResult = aContext.InternalApply(myResult);

        fft.FixSmallAreaWireMode() = savFixSmallAreaWireMode;

        return status;
    }

    /// <summary>Fixes SameParameter of the edges of the shape (ShapeFix::SameParameter). Writes into the shape.</summary>
    protected virtual void SameParameter(in TopoDS_Shape sh, bool enforce, Message_ProgressRange theProgress = default)
    {
        ShapeFix.SameParameter(sh, enforce, 0.0, theProgress);
    }

    /// <summary>Returns the resulting shape, frozen.</summary>
    public TopoDS_Shape Shape()
    {
        myResult.TShape()?.FreezeGraph();
        return myResult;
    }

    /// <summary>Returns the resulting shape, unfrozen, for ported callers fixing a shape they own.</summary>
    internal TopoDS_Shape InternalShape() => myResult;

    /// <summary>Returns the solid fixing tool.</summary>
    public ShapeFix_Solid FixSolidTool() => myFixSolid;

    /// <summary>Returns the shell fixing tool.</summary>
    public ShapeFix_Shell FixShellTool() => myFixSolid.FixShellTool();

    /// <summary>Returns the face fixing tool.</summary>
    public ShapeFix_Face FixFaceTool() => myFixSolid.FixShellTool().FixFaceTool();

    /// <summary>Returns the wire fixing tool.</summary>
    public ShapeFix_Wire FixWireTool() => myFixSolid.FixShellTool().FixFaceTool().FixWireTool();

    /// <summary>Returns the edge fixing tool.</summary>
    public ShapeFix_Edge FixEdgeTool() => myFixSolid.FixShellTool().FixFaceTool().FixWireTool().FixEdgeTool();

    /// <summary>
    /// Returns the status: OK nothing done; DONE1 vertex tolerances fixed on a free edge; DONE2 a free wire fixed;
    /// DONE3 a free face fixed; DONE4 a shell or a solid fixed.
    /// </summary>
    public bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatus, status);

    /// <summary>Sets the message registrator, for this tool and its tools.</summary>
    public override void SetMsgRegistrator(ShapeExtend_BasicMsgRegistrator? msgreg)
    {
        base.SetMsgRegistrator(msgreg);
        myFixSolid.SetMsgRegistrator(msgreg);
    }

    /// <summary>Sets the basic precision, for this tool and its tools.</summary>
    public override void SetPrecision(double preci)
    {
        base.SetPrecision(preci);
        myFixSolid.SetPrecision(preci);
    }

    /// <summary>Sets the minimal tolerance, for this tool and its tools.</summary>
    public override void SetMinTolerance(double mintol)
    {
        base.SetMinTolerance(mintol);
        myFixSolid.SetMinTolerance(mintol);
    }

    /// <summary>Sets the maximal tolerance, for this tool and its tools.</summary>
    public override void SetMaxTolerance(double maxtol)
    {
        base.SetMaxTolerance(maxtol);
        myFixSolid.SetMaxTolerance(maxtol);
    }

    /// <summary>The mode of the solid fixes (-1 default, 0 off, 1 on).</summary>
    public ref int FixSolidMode() => ref myFixSolidMode;

    /// <summary>The mode of the fixes of free shells (-1 default, 0 off, 1 on).</summary>
    public ref int FixFreeShellMode() => ref myFixShellMode;

    /// <summary>The mode of the fixes of free faces (-1 default, 0 off, 1 on).</summary>
    public ref int FixFreeFaceMode() => ref myFixFaceMode;

    /// <summary>The mode of the fixes of free wires (-1 default, 0 off, 1 on).</summary>
    public ref int FixFreeWireMode() => ref myFixWireMode;

    /// <summary>The mode of the SameParameter fix (-1 default, 0 off, 1 on).</summary>
    public ref int FixSameParameterMode() => ref myFixSameParameterMode;

    /// <summary>The mode of the fix of the vertex positions (0 by default: off).</summary>
    public ref int FixVertexPositionMode() => ref myFixVertexPositionMode;

    /// <summary>The mode of the check of the vertex tolerances after all fixes (-1 default, 0 off, 1 on).</summary>
    public ref int FixVertexTolMode() => ref myFixVertexTolMode;
}
