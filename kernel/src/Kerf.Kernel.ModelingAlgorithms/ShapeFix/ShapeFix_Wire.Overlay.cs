// OCCT.NET: the working overlay of ShapeFix_Wire (a copy at the entry until Kerf unit 10H4; decision 583) and its
// public results. Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;

using Kerf.Engine.Topology.Brep;

using Kerf.Engine.Topology.Tools;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Healing.Fixes;

public partial class WireFixer
{
    // The working overlay of the loaded edges, from the first fix to the public result (Kerf unit 10H4): this tool's
    // own (detached, entered by each fix), or the overlay of the operation that runs this tool (ShapeFix_Face's, ...),
    // which this tool doesn't finish. Null before the first fix.
    private WorkingOverlay? _overlay;
    private bool _ownsOverlay;

    // True when the next fix starts a new overlay: after Load, Init and a public result.
    private bool _toEnter;

    /// <summary>
    /// Returns the resulting wire (made with a builder): the edges of <see cref="WireData"/>, frozen. It holds new
    /// TShapes for the edges and vertices the fixes changed and the caller's own for the rest, and the context records
    /// the ones changed in place.
    /// </summary>
    public Wire Wire()
    {
        EndWork();
        return _analyzer.WireData()!.Wire();
    }

    /// <summary>
    /// Returns the resulting wire made with BRepBuilderAPI_MakeWire (which can merge vertices), frozen; see
    /// <see cref="Wire"/>.
    /// </summary>
    public Wire WireAPIMake()
    {
        EndWork();
        return _analyzer.WireData()!.WireAPIMake();
    }

    /// <summary>
    /// The resulting wire, unfrozen, for ported code fixing a wire it owns (ShapeFix_Face, UnifySameDomain): its edges
    /// are the working ones.
    /// </summary>
    internal Wire InternalWire() => _analyzer.WireData()!.InternalWire();

    /// <summary>The resulting wire made with BRepBuilderAPI_MakeWire, unfrozen; see <see cref="InternalWire"/>.</summary>
    internal Wire InternalWireAPIMake() => _analyzer.WireData()!.InternalWireAPIMake();

    /// <summary>
    /// Called at the start of <see cref="Perform"/> and of every public fix (Kerf unit 10H4): after a load or a public
    /// result, takes the overlay the fixes work in (the current operation's, or a detached one of this tool's over the
    /// loaded edges and the non-manifold ones); then makes it current until the returned scope is disposed. The fixes
    /// run on the loaded edges themselves.
    /// </summary>
    private WorkingOverlay.Scope EnterWork()
    {
        if (_toEnter)
        {
            _toEnter     = false;
            _overlay     = WorkingOverlay.Writing;
            _ownsOverlay = false;
            ExtendWireData? sbwd = _analyzer.WireData();
            if (_overlay is null && sbwd is not null)
            {
                _overlay     = WorkingOverlay.Detached(EdgesCompound(sbwd), Context());
                _ownsOverlay = true;
            }
        }
        return _overlay?.Enter() ?? default;
    }

    /// <summary>
    /// Before a public result: finishes this tool's overlay, so the wire data holds new TShapes for the edges the fixes
    /// changed and the caller's own for the rest, and the context records the ones changed in place. The next fix
    /// starts a new overlay (the published edges are frozen).
    /// </summary>
    private void EndWork()
    {
        _toEnter = true;
        WorkingOverlay? overlay = _ownsOverlay ? _overlay : null;
        _overlay     = null;
        _ownsOverlay = false;
        ExtendWireData? sbwd = _analyzer.WireData();
        if (overlay is null || sbwd is null)
        {
            return;
        }

        Shape final = overlay.Finish(EdgesCompound(sbwd));
        SetEdges(sbwd, final);
    }

    // The edges, then the non-manifold edges, of the wire data, appended to a new compound as they are (no Free flag
    // changed: Builder.Add would clear the frozen edges' outside an overlay).
    private static Compound EdgesCompound(ExtendWireData sbwd)
    {
        var compound = new Compound();
        new BrepBuilder().MakeCompound(ref compound);
        TShape tShape = compound.TShape()!;
        for (int i = 1; i <= sbwd.EdgeCount; i++)
        {
            tShape._shapes.Append(sbwd.Edge(i));
        }
        NCollection_HSequence<Shape> nm = sbwd.InternalNonmanifoldEdges();
        for (int i = 1; i <= nm.Length(); i++)
        {
            tShape._shapes.Append(nm[i]);
        }
        return compound;
    }

    // Puts back into the wire data, in order, the children of a compound made by edgesCompound.
    private static void SetEdges(ExtendWireData sbwd, in Shape compound)
    {
        int nbEdges = sbwd.EdgeCount;
        NCollection_HSequence<Shape> nm = sbwd.InternalNonmanifoldEdges();
        int i = 1;
        for (var it = new Iterator(compound, false, false); it.More(); it.Next(), i++)
        {
            Shape edge = it.Value();
            if (i <= nbEdges)
            {
                sbwd.InternalSetEdge(ShapeCasts.Edge(edge), i);
            }
            else
            {
                nm.SetValue(i - nbEdges, edge);
            }
        }
    }
}
