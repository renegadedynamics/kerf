// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceIterator.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using static Occt.TopOpeBRepDS_Kind;

namespace Occt;

/// <summary>Iterates over the interferences of a list whose geometry is a surface: the surfaces on a solid.</summary>
/// <remarks>
/// As in OCCT, the condition on the geometry kind is set after the iterator has moved to the list's first
/// interference, without matching again: the first <see cref="TopOpeBRepDS_InterferenceIterator.Value"/> is the list's
/// first interference, whatever its kind; <see cref="TopOpeBRepDS_InterferenceIterator.Next"/> then skips the others
/// (occt-issues, unit 7L).
/// </remarks>
public sealed class TopOpeBRepDS_SurfaceIterator : TopOpeBRepDS_InterferenceIterator
{
    /// <summary>An iterator over the surfaces of the interferences of <paramref name="L"/>.</summary>
    public TopOpeBRepDS_SurfaceIterator(NCollection_List<TopOpeBRepDS_Interference> L)
        : base(L)
    {
        // OCCT: no Match() after setting the condition
        GeometryKind(TopOpeBRepDS_SURFACE);
    }

    /// <summary>The index of the current surface in the data structure.</summary>
    public int Current()
    {
        TopOpeBRepDS_Interference i = Value();
        int g = i.Geometry();
        return g;
    }

    /// <summary>The orientation of the current interference's transition for the state <paramref name="S"/>.</summary>
    public Orientation Orientation(Containment S)
    {
        TopOpeBRepDS_Interference i = Value();
        TopOpeBRepDS_Transition t = i.Transition();
        Orientation o = t.Orientation(S);
        return o;
    }
}
