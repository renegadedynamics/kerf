// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_PointData.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_PointData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>A point of the data structure with its interferences and the indices of two shapes.</summary>
internal sealed class DsPointData : DsGeometryData
{
    internal DsPoint _point;
    private int _s1;
    private int _s2;

    /// <summary>An empty point.</summary>
    public DsPointData()
    {
        _point = new DsPoint();
        _s1 = 0;
        _s2 = 0;
    }

    /// <summary>A copy of the point <paramref name="p"/>.</summary>
    public DsPointData(in DsPoint p)
    {
        _point = p;
        _s1 = 0;
        _s2 = 0;
    }

    /// <summary>A copy of the point <paramref name="p"/> with the shape indices <paramref name="i1"/> and <paramref name="i2"/>.</summary>
    public DsPointData(in DsPoint p, int i1, int i2)
    {
        _point = p;
        _s1 = i1;
        _s2 = i2;
    }

    internal void SetShapes(int i1, int i2)
    {
        _s1 = i1;
        _s2 = i2;
    }

    /// <summary>The two shape indices.</summary>
    public void GetShapes(out int i1, out int i2)
    {
        i1 = _s1;
        i2 = _s2;
    }
}
