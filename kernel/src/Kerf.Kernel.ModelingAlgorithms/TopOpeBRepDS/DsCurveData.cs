// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurveData.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurveData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>A curve of the data structure with its interferences.</summary>
internal sealed class DsCurveData : DsGeometryData
{
    internal DsCurve _curve;

    /// <summary>An empty curve.</summary>
    public DsCurveData()
    {
        _curve = new DsCurve();
    }

    /// <summary>A copy of the curve <paramref name="c"/>, with no interferences.</summary>
    public DsCurveData(in DsCurve c)
    {
        _curve = c;
    }
}
