// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireToFace.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_WireToFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Booleans.Legacy.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Booleans.Legacy.Tools.OutCurveType;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>
/// Builds faces from a set of wires and a face: the wires are classified against each other on the face's surface,
/// and each outer wire makes a face with the wires inside it.
/// </summary>
/// <remarks>
/// <para>
/// The edges of the wires must have a pcurve on the face's surface, unless it is a plane. The faces are empty copies
/// of the face (same surface, tolerance and orientation) holding the wires themselves.
/// </para>
/// <para>
/// OCCT flags every wire closed and stores computed pcurves on edges without one: it writes into the caller's wires.
/// The port's <see cref="MakeFaces"/> works on a copy of frozen wires (<see cref="CopyAtEntry"/>) and gives
/// the caller's wires and edges back wherever nothing was written, so the faces hold the caller's own wires as
/// native's do when the wires are already flagged closed (decision 793). The faces handed out are frozen.
/// </para>
/// </remarks>
public sealed class WireToFace
{
    private readonly NCollection_List<Shape> _lw = new();

    /// <summary>An empty set of wires.</summary>
    public WireToFace()
    {
    }

    /// <summary>Removes the wires.</summary>
    public void Init()
    {
        _lw.Clear();
    }

    /// <summary>Adds the wire <paramref name="w"/>.</summary>
    public void AddWire(in Wire w)
    {
        _lw.Append(w);
    }

    /// <summary>
    /// Replaces the contents of <paramref name="lf"/> by the faces the wires bound on <paramref name="f"/> (frozen).
    /// </summary>
    public void MakeFaces(in Face f, NCollection_List<Shape> lf)
    {
        // OCCT.NET: copy at entry (decision 793)
        Shape holder = HolderOf(_lw);
        CopyAtEntry entry = new(holder);
        if (!entry.IsCopied)
        {
            InternalMakeFaces(f, lf);
        }
        else
        {
            NCollection_List<Shape> saved = new(_lw);
            _lw.Clear();
            for (NCollection_List<Shape>.Iterator it = new(saved); it.More(); it.Next())
            {
                _lw.Append(entry.Copied(it.Value()));
            }
            try
            {
                InternalMakeFaces(f, lf);
            }
            finally
            {
                _lw.Assign(saved);
            }
            entry.Finish(new Shape(), lf);
        }
        for (NCollection_List<Shape>.Iterator it = new(lf); it.More(); it.Next())
        {
            it.Value().TShape()?.FreezeGraph();
        }
    }

    /// <summary>OCCT's <c>MakeFaces</c>, in place and without freezing, for ported callers.</summary>
    internal void InternalMakeFaces(in Face f, NCollection_List<Shape> lf)
    {
        lf.Clear();

        LegacyWireEdgeSet wes = new(f);
        for (NCollection_List<Shape>.Iterator it = new(_lw); it.More(); it.Next())
        {
            wes.AddShape(it.Value());
        }

        bool forceClass = true;
        BuildFaceBuilder fb = new();
        fb.InitFaceBuilder(wes, f, forceClass);

        DsBuildTool bt = new(OutCurveType.Approx);
        LegacyBuilder b = new(bt);
        b.MakeFaces(f, fb, lf);
    }

    /// <summary>
    /// OCCT.NET: an unfrozen compound holding <paramref name="shapes"/> as children, appended directly so their
    /// Free flags don't change, to copy them at once with <see cref="CopyAtEntry"/> (shared sub-shapes stay
    /// shared).
    /// </summary>
    internal static Shape HolderOf(NCollection_List<Shape> shapes)
    {
        Compound holder = new();
        new BrepBuilder().MakeCompound(ref holder);
        TShape holderTShape = holder.TShape()!;
        for (NCollection_List<Shape>.Iterator it = new(shapes); it.More(); it.Next())
        {
            if (!it.Value().IsNull)
            {
                holderTShape._shapes.Append(it.Value());
            }
        }
        return holder;
    }
}
