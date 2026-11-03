// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurvePointInterference.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurvePointInterference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>An interference with a parameter: a point or vertex on a curve or edge, at a parameter of it.</summary>
public sealed class DsCurvePointInterference : DsInterference
{
    private double _param;

    /// <summary>
    /// The interference of the geometry <paramref name="g"/> on the support <paramref name="s"/> at the parameter
    /// <paramref name="p"/> of the support.
    /// </summary>
    public DsCurvePointInterference(in DsTransition t,
                                               DsKind st,
                                               int s,
                                               DsKind gt,
                                               int g,
                                               double p)
        : base(t, st, s, gt, g)
    {
        _param = p;
    }

    /// <summary>The parameter.</summary>
    public double Parameter() => _param;

    internal void Parameter(double p)
    {
        _param = p;
    }
}
