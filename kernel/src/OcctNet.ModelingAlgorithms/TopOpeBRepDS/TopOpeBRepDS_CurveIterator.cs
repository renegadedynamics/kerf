// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurveIterator.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurveIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopOpeBRepDS_Kind;

namespace Occt;

/// <summary>Iterates over the interferences of a list whose geometry is a curve: the curves on a face or surface.</summary>
public sealed class TopOpeBRepDS_CurveIterator : TopOpeBRepDS_InterferenceIterator
{
    /// <summary>An iterator over the curves of the interferences of <paramref name="L"/>.</summary>
    public TopOpeBRepDS_CurveIterator(NCollection_List<TopOpeBRepDS_Interference> L)
        : base(L)
    {
        Match();
    }

    /// <summary>Whether the geometry of <paramref name="I"/> is a CURVE.</summary>
    public override bool MatchInterference(TopOpeBRepDS_Interference I)
    {
        TopOpeBRepDS_Kind GT = I.GeometryType();
        bool r = (GT == TopOpeBRepDS_CURVE);
        return r;
    }

    /// <summary>The index of the current curve in the data structure.</summary>
    public int Current()
    {
        TopOpeBRepDS_Interference I = Value();
        int G = I.Geometry();
        return G;
    }

    /// <summary>The orientation of the current interference's transition for the state <paramref name="S"/>.</summary>
    public TopAbs_Orientation Orientation(TopAbs_State S)
    {
        TopOpeBRepDS_Interference I = Value();
        TopOpeBRepDS_Transition T = I.Transition();
        TopAbs_Orientation o = T.Orientation(S);
        return o;
    }

    /// <summary>The pcurve of the current interference.</summary>
    /// <exception cref="InvalidCastException">The interference is not a surface/curve interference.</exception>
    public Geom2d_Curve? PCurve()
    {
        // OCCT reinterprets the handle without checking its type; the port's cast throws on a mismatch
        return ((TopOpeBRepDS_SurfaceCurveInterference)Value()).PCurve();
    }
}
