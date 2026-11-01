// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurvePointInterference.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurvePointInterference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An interference with a parameter: a point or vertex on a curve or edge, at a parameter of it.</summary>
public sealed class TopOpeBRepDS_CurvePointInterference : TopOpeBRepDS_Interference
{
    private double myParam;

    /// <summary>
    /// The interference of the geometry <paramref name="G"/> on the support <paramref name="S"/> at the parameter
    /// <paramref name="P"/> of the support.
    /// </summary>
    public TopOpeBRepDS_CurvePointInterference(in TopOpeBRepDS_Transition T,
                                               TopOpeBRepDS_Kind ST,
                                               int S,
                                               TopOpeBRepDS_Kind GT,
                                               int G,
                                               double P)
        : base(T, ST, S, GT, G)
    {
        myParam = P;
    }

    /// <summary>The parameter.</summary>
    public double Parameter() => myParam;

    internal void Parameter(double P)
    {
        myParam = P;
    }
}
