// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_PointData.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_PointData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A point of the data structure with its interferences and the indices of two shapes.</summary>
public sealed class TopOpeBRepDS_PointData : TopOpeBRepDS_GeometryData
{
    internal TopOpeBRepDS_Point myPoint;
    private int myS1;
    private int myS2;

    /// <summary>An empty point.</summary>
    public TopOpeBRepDS_PointData()
    {
        myPoint = new TopOpeBRepDS_Point();
        myS1 = 0;
        myS2 = 0;
    }

    /// <summary>A copy of the point <paramref name="P"/>.</summary>
    public TopOpeBRepDS_PointData(in TopOpeBRepDS_Point P)
    {
        myPoint = P;
        myS1 = 0;
        myS2 = 0;
    }

    /// <summary>A copy of the point <paramref name="P"/> with the shape indices <paramref name="I1"/> and <paramref name="I2"/>.</summary>
    public TopOpeBRepDS_PointData(in TopOpeBRepDS_Point P, int I1, int I2)
    {
        myPoint = P;
        myS1 = I1;
        myS2 = I2;
    }

    internal void SetShapes(int I1, int I2)
    {
        myS1 = I1;
        myS2 = I2;
    }

    /// <summary>The two shape indices.</summary>
    public void GetShapes(out int I1, out int I2)
    {
        I1 = myS1;
        I2 = myS2;
    }
}
