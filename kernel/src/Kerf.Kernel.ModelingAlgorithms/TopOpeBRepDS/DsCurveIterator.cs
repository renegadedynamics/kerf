// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurveIterator.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurveIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Booleans.Legacy.Data.DsKind;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>Iterates over the interferences of a list whose geometry is a curve: the curves on a face or surface.</summary>
internal sealed class DsCurveIterator : DsInterferenceIterator
{
    /// <summary>An iterator over the curves of the interferences of <paramref name="l"/>.</summary>
    public DsCurveIterator(NCollection_List<DsInterference> l)
        : base(l)
    {
        Match();
    }

    /// <summary>Whether the geometry of <paramref name="i"/> is a CURVE.</summary>
    public override bool MatchInterference(DsInterference i)
    {
        DsKind gt = i.GeometryType();
        bool r = (gt == DsKind.Curve);
        return r;
    }

    /// <summary>The index of the current curve in the data structure.</summary>
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

    /// <summary>The pcurve of the current interference.</summary>
    /// <exception cref="InvalidCastException">The interference is not a surface/curve interference.</exception>
    public Curve2D? PCurve()
    {
        // OCCT reinterprets the handle without checking its type; the port's cast throws on a mismatch
        return ((DsSurfaceCurveInterference)Value()).PCurve();
    }
}
