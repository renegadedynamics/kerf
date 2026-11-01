// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ShapeShapeInterference.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ShapeShapeInterference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An interference of a shape on a shape: for example a vertex of an edge found on another edge or a face.</summary>
public class TopOpeBRepDS_ShapeShapeInterference : TopOpeBRepDS_Interference
{
    private bool myGBound;
    private TopOpeBRepDS_Config myC;

    /// <summary>
    /// The interference of the shape <paramref name="G"/> on the shape <paramref name="S"/>. <paramref name="GBound"/>
    /// tells whether <paramref name="G"/> bounds <paramref name="S"/>; <paramref name="C"/> how the two shapes share
    /// their geometry (not, or with the same or opposite orientations).
    /// </summary>
    public TopOpeBRepDS_ShapeShapeInterference(in TopOpeBRepDS_Transition T,
                                               TopOpeBRepDS_Kind ST,
                                               int S,
                                               TopOpeBRepDS_Kind GT,
                                               int G,
                                               bool GBound,
                                               TopOpeBRepDS_Config C)
        : base(T, ST, S, GT, G)
    {
        myGBound = GBound;
        myC = C;
    }

    /// <summary>How the two shapes share their geometry.</summary>
    public TopOpeBRepDS_Config Config() => myC;

    /// <summary>Whether the geometry bounds the support.</summary>
    public bool GBound() => myGBound;

    internal void SetGBound(bool b)
    {
        myGBound = b;
    }
}
