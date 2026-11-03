// OCCT.NET: the copy at the entry of ShapeFix_Wire (docs/phase5-6-plan.md, section 2.7; decision 583) and its public
// results. Not a port of an OCCT file.
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
    // The copy of the loaded edges, from the first fix to the public result; null when the loaded edges hold no frozen
    // shape (they are the caller's working shapes, fixed in place as in OCCT).
    private CopyAtEntry? _entry;

    // True when the loaded edges must be checked (and copied if frozen) before the next fix: after Load, Init and a
    // public result.
    private bool _toEnter;

    /// <summary>
    /// Returns the resulting wire (made with a builder): the edges of <see cref="WireData"/>, frozen. Where the loaded
    /// wire was frozen, the edges and vertices the fixes left unchanged are the caller's own, and the context records
    /// the ones they changed in place.
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
    /// Called at the start of <see cref="Perform"/> and of every public fix: after a load or a public result, copies
    /// the loaded edges (and the non-manifold ones) when they hold a frozen shape, and works on the copies.
    /// </summary>
    private void EnterWork()
    {
        if (!_toEnter)
        {
            return;
        }
        _toEnter = false;
        _entry   = null;

        ExtendWireData? sbwd = _analyzer.WireData();
        if (sbwd is null)
        {
            return;
        }
        Compound edges = EdgesCompound(sbwd);
        if (!BrepTools.HoldsFrozen(edges))
        {
            return;
        }

        _entry = new CopyAtEntry(edges, Context());
        Shape copy = _entry.Shape();
        SetEdges(sbwd, copy);

        // The copy clears the Free flag of the edges it puts in the compound; give each copy its original's (a cache
        // bit, which KeepUnchanged ignores), so the fixes that test E.Free() (FixConnected, ...) take OCCT's branch.
        var original = new Iterator(edges, false, false);
        for (var copied = new Iterator(copy, false, false); copied.More(); copied.Next(), original.Next())
        {
            copied.Value().TShape()!.Free(original.Value().TShape()!.Free());
        }
    }

    /// <summary>
    /// Before a public result: gives back the caller's own edges and vertices wherever the fixes left the copies
    /// unchanged, in the wire data and in the context, and records in the context the copies changed in place. The
    /// next fix copies again (the published edges are frozen).
    /// </summary>
    private void EndWork()
    {
        _toEnter = true;
        CopyAtEntry? entry = _entry;
        _entry = null;
        ExtendWireData? sbwd = _analyzer.WireData();
        if (entry is null || sbwd is null)
        {
            return;
        }

        Compound worked = EdgesCompound(sbwd);
        Shape final = entry.Finish(worked);
        SetEdges(sbwd, final);
    }

    // The edges, then the non-manifold edges, of the wire data, appended to a new compound as they are (no Free flag
    // changed).
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
