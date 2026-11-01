// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurveData.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_CurveData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A curve of the data structure with its interferences.</summary>
public sealed class TopOpeBRepDS_CurveData : TopOpeBRepDS_GeometryData
{
    internal TopOpeBRepDS_Curve myCurve;

    /// <summary>An empty curve.</summary>
    public TopOpeBRepDS_CurveData()
    {
        myCurve = new TopOpeBRepDS_Curve();
    }

    /// <summary>A copy of the curve <paramref name="C"/>, with no interferences.</summary>
    public TopOpeBRepDS_CurveData(in TopOpeBRepDS_Curve C)
    {
        myCurve = C;
    }
}
