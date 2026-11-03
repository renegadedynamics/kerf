// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceCurveInterference.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_SurfaceCurveInterference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>An interference with a 2D curve: a curve on a face or surface, with its pcurve there.</summary>
public sealed class TopOpeBRepDS_SurfaceCurveInterference : TopOpeBRepDS_Interference
{
    private Curve2D? myPCurve;

    /// <summary>An empty interference.</summary>
    public TopOpeBRepDS_SurfaceCurveInterference()
    {
    }

    /// <summary>
    /// The interference of the geometry <paramref name="G"/> on the support <paramref name="S"/>, with the pcurve
    /// <paramref name="PC"/> on the support.
    /// </summary>
    public TopOpeBRepDS_SurfaceCurveInterference(in TopOpeBRepDS_Transition T,
                                                 TopOpeBRepDS_Kind ST,
                                                 int S,
                                                 TopOpeBRepDS_Kind GT,
                                                 int G,
                                                 Curve2D? PC)
        : base(T, ST, S, GT, G)
    {
        myPCurve = PC;
    }

    /// <summary>A copy of the transition, support and geometry of <paramref name="I"/>, with no pcurve.</summary>
    public TopOpeBRepDS_SurfaceCurveInterference(TopOpeBRepDS_Interference I)
        : base(I)
    {
    }

    /// <summary>The pcurve on the support.</summary>
    public Curve2D? PCurve() => myPCurve;

    internal void PCurve(Curve2D? PC)
    {
        myPCurve = PC;
    }
}
