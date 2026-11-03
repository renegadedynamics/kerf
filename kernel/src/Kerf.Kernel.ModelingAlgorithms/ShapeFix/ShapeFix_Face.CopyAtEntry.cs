// OCCT.NET: the copy at the entry of ShapeFix_Face (docs/phase5-6-plan.md, section 2.7; decision 583) and its public
// results. Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Build;
using Kerf.Engine.Topology;

using Kerf.Engine.Topology.Tools;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Healing.Fixes;

public partial class FaceFixer
{
    // The copy of the loaded face, from the first fix to the public result; null when the loaded face holds no frozen
    // shape (a face ported code owns, fixed in place as in OCCT).
    private CopyAtEntry? _entry;

    // True when the loaded face must be checked (and copied if frozen) before the next fix: after a load and after a
    // public result.
    private bool _toEnter;

    /// <summary>
    /// Returns the fixed face (the last face processed when the result has several), frozen. Where the loaded face was
    /// frozen, the sub-shapes the fixes left unchanged are the caller's own, and the context answers for the caller's
    /// shapes.
    /// </summary>
    public Face Face()
    {
        EndWork();
        _face.TShape()?.FreezeGraph();
        return _face;
    }

    /// <summary>
    /// Returns the result of the fixes (a face, or a shell or compound of faces after a split), frozen; see
    /// <see cref="Face"/>.
    /// </summary>
    public Shape Result()
    {
        EndWork();
        _result.TShape()?.FreezeGraph();
        return _result;
    }

    /// <summary>The fixed face, unfrozen, for ported callers fixing a face they own (ShapeFix_Shell, ...).</summary>
    internal Face InternalFace() => _face;

    /// <summary>The result, unfrozen, for ported callers fixing a face they own.</summary>
    internal Shape InternalResult() => _result;

    /// <summary>Sets the context; a context set by a fix after the copy at the entry is re-keyed by the result too.</summary>
    public override void SetContext(ReShapeBuilder? context)
    {
        base.SetContext(context);
        _entry?.AttachContext(context);
    }

    // Called by the loaders.
    private void Loaded()
    {
        _toEnter = true;
        _entry   = null;
    }

    /// <summary>
    /// Called at the start of <see cref="Perform"/> and of every public fix: after a load or a public result, copies the
    /// loaded face when it holds a frozen shape, and works on the copy.
    /// </summary>
    private void EnterWork()
    {
        if (!_toEnter)
        {
            return;
        }
        _toEnter = false;
        _entry   = null;
        if (_face.IsNull || !BrepTools.HoldsFrozen(_face))
        {
            return;
        }

        _entry = new CopyAtEntry(_face, Context());
        _face  = ShapeCasts.Face(_entry.Shape());
        _shape = _face;
    }

    /// <summary>
    /// Before a public result: gives back the caller's own sub-shapes wherever the fixes left the copies unchanged, in
    /// the result, the face and the context, and records in the context the copies changed in place. The next fix
    /// copies again (the published shapes are frozen).
    /// </summary>
    private void EndWork()
    {
        _toEnter = true;
        CopyAtEntry? entry = _entry;
        _entry = null;
        if (entry is null)
        {
            return;
        }

        _result = entry.Finish(_result, [_face]);
        if (!_face.IsNull)
        {
            _face = ShapeCasts.Face(entry.Final(_face));
        }
        _shape = _face;
    }
}
