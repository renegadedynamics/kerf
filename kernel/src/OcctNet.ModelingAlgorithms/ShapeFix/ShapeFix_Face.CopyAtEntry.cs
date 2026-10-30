// OCCT.NET: the copy at the entry of ShapeFix_Face (docs/phase5-6-plan.md, section 2.7; decision 583) and its public
// results. Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

public partial class ShapeFix_Face
{
    // The copy of the loaded face, from the first fix to the public result; null when the loaded face holds no frozen
    // shape (a face ported code owns, fixed in place as in OCCT).
    private BRepTools_CopyAtEntry? myEntry;

    // True when the loaded face must be checked (and copied if frozen) before the next fix: after a load and after a
    // public result.
    private bool myToEnter;

    /// <summary>
    /// Returns the fixed face (the last face processed when the result has several), frozen. Where the loaded face was
    /// frozen, the sub-shapes the fixes left unchanged are the caller's own, and the context answers for the caller's
    /// shapes.
    /// </summary>
    public TopoDS_Face Face()
    {
        endWork();
        myFace.TShape()?.FreezeGraph();
        return myFace;
    }

    /// <summary>
    /// Returns the result of the fixes (a face, or a shell or compound of faces after a split), frozen; see
    /// <see cref="Face"/>.
    /// </summary>
    public TopoDS_Shape Result()
    {
        endWork();
        myResult.TShape()?.FreezeGraph();
        return myResult;
    }

    /// <summary>The fixed face, unfrozen, for ported callers fixing a face they own (ShapeFix_Shell, ...).</summary>
    internal TopoDS_Face InternalFace() => myFace;

    /// <summary>The result, unfrozen, for ported callers fixing a face they own.</summary>
    internal TopoDS_Shape InternalResult() => myResult;

    /// <summary>Sets the context; a context set by a fix after the copy at the entry is re-keyed by the result too.</summary>
    public override void SetContext(ShapeBuild_ReShape? context)
    {
        base.SetContext(context);
        myEntry?.AttachContext(context);
    }

    // Called by the loaders.
    private void loaded()
    {
        myToEnter = true;
        myEntry   = null;
    }

    /// <summary>
    /// Called at the start of <see cref="Perform"/> and of every public fix: after a load or a public result, copies the
    /// loaded face when it holds a frozen shape, and works on the copy.
    /// </summary>
    private void enterWork()
    {
        if (!myToEnter)
        {
            return;
        }
        myToEnter = false;
        myEntry   = null;
        if (myFace.IsNull() || !BRepTools.HoldsFrozen(myFace))
        {
            return;
        }

        myEntry = new BRepTools_CopyAtEntry(myFace, Context());
        myFace  = TopoDS.Face(myEntry.Shape());
        myShape = myFace;
    }

    /// <summary>
    /// Before a public result: gives back the caller's own sub-shapes wherever the fixes left the copies unchanged, in
    /// the result, the face and the context, and records in the context the copies changed in place. The next fix
    /// copies again (the published shapes are frozen).
    /// </summary>
    private void endWork()
    {
        myToEnter = true;
        BRepTools_CopyAtEntry? anEntry = myEntry;
        myEntry = null;
        if (anEntry is null)
        {
            return;
        }

        myResult = anEntry.Finish(myResult, [myFace]);
        if (!myFace.IsNull())
        {
            myFace = TopoDS.Face(anEntry.Final(myFace));
        }
        myShape = myFace;
    }
}
