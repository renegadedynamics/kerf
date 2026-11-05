// Kerf: the working overlay of ShapeFix_Face (a copy at the entry until Kerf unit 10H4; decision 583) and its
// public results. Not a port of an OCCT file.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Build;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Healing.Fixes;

internal partial class FaceFixer
{
    // The working overlay of the loaded face, from the first fix to the public result (Kerf unit 10H4): this tool's own
    // (detached, entered by each fix), or the overlay of the operation that runs this tool (ShapeFix_Shell's, ...),
    // which this tool doesn't finish. Null before the first fix.
    private WorkingOverlay? _overlay;
    private bool _ownsOverlay;

    // True when the next fix starts a new overlay: after a load and after a public result.
    private bool _toEnter;

    /// <summary>
    /// Returns the fixed face (the last face processed when the result has several), frozen. It holds new TShapes for
    /// what the fixes changed and the caller's own sub-shapes for the rest, and the context answers for the caller's
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

    /// <summary>Sets the context; a context set by a fix after the first is brought up to date by the result too.</summary>
    public override void SetContext(ReShapeBuilder? context)
    {
        base.SetContext(context);
        if (_ownsOverlay)
        {
            _overlay?.AttachContext(context);
        }
    }

    // Called by the loaders.
    private void Loaded()
    {
        _toEnter     = true;
        _overlay     = null;
        _ownsOverlay = false;
    }

    /// <summary>
    /// Called at the start of <see cref="Perform"/> and of every public fix (Kerf unit 10H4): after a load or a public
    /// result, takes the overlay the fixes work in (the current operation's, or a detached one of this tool's); then
    /// makes it current until the returned scope is disposed. The fixes run on the loaded face itself.
    /// </summary>
    private WorkingOverlay.Scope EnterWork()
    {
        if (_toEnter)
        {
            _toEnter     = false;
            _overlay     = null;
            _ownsOverlay = false;
            if (!_face.IsNull)
            {
                _overlay = WorkingOverlay.ForTool(_face, Context(), out _ownsOverlay);
            }
        }
        return _overlay?.Enter() ?? default;
    }

    /// <summary>
    /// Before a public result: finishes this tool's overlay, so the result and the face hold new TShapes for what the
    /// fixes changed and the caller's own sub-shapes for the rest, and the context records the sub-shapes changed in
    /// place. The next fix starts a new overlay (the published shapes are frozen).
    /// </summary>
    private void EndWork()
    {
        _toEnter = true;
        WorkingOverlay? overlay = _ownsOverlay ? _overlay : null;
        _overlay     = null;
        _ownsOverlay = false;
        if (overlay is null)
        {
            return;
        }

        _result = overlay.Finish(_result, [_face]);
        if (!_face.IsNull)
        {
            _face = ShapeCasts.Face(overlay.Final(_face));
        }
        _shape = _face;
    }
}
