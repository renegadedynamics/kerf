// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ShapeShapeInterference.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ShapeShapeInterference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>An interference of a shape on a shape: for example a vertex of an edge found on another edge or a face.</summary>
internal class DsShapeShapeInterference : DsInterference
{
    private bool _gBound;
    private DsConfig _c;

    /// <summary>
    /// The interference of the shape <paramref name="g"/> on the shape <paramref name="s"/>. <paramref name="gBound"/>
    /// tells whether <paramref name="g"/> bounds <paramref name="s"/>; <paramref name="c"/> how the two shapes share
    /// their geometry (not, or with the same or opposite orientations).
    /// </summary>
    public DsShapeShapeInterference(in DsTransition t,
                                               DsKind st,
                                               int s,
                                               DsKind gt,
                                               int g,
                                               bool gBound,
                                               DsConfig c)
        : base(t, st, s, gt, g)
    {
        _gBound = gBound;
        _c = c;
    }

    /// <summary>How the two shapes share their geometry.</summary>
    public DsConfig Config() => _c;

    /// <summary>Whether the geometry bounds the support.</summary>
    public bool GBound() => _gBound;

    internal void SetGBound(bool b)
    {
        _gBound = b;
    }
}
