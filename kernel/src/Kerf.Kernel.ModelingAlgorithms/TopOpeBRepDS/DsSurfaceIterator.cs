// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceIterator.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>Iterates over the interferences of a list whose geometry is a surface: the surfaces on a solid.</summary>
/// <remarks>
/// As in OCCT, the condition on the geometry kind is set after the iterator has moved to the list's first
/// interference, without matching again: the first <see cref="DsInterferenceIterator.Value"/> is the list's
/// first interference, whatever its kind; <see cref="DsInterferenceIterator.Next"/> then skips the others
/// (occt-issues, unit 7L).
/// </remarks>
internal sealed class DsSurfaceIterator : DsInterferenceIterator
{
    /// <summary>An iterator over the surfaces of the interferences of <paramref name="l"/>.</summary>
    public DsSurfaceIterator(NCollection_List<DsInterference> l)
        : base(l)
    {
        // OCCT: no Match() after setting the condition
        GeometryKind(DsKind.Surface);
    }

    /// <summary>The index of the current surface in the data structure.</summary>
    public int Current()
    {
        DsInterference i = Value();
        int g = i.Geometry();
        return g;
    }

    /// <summary>The orientation of the current interference's transition for the state <paramref name="s"/>.</summary>
    public Orientation Orientation(Containment s)
    {
        DsInterference i = Value();
        DsTransition t = i.Transition();
        Orientation o = t.Orientation(s);
        return o;
    }
}
