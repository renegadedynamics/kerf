using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>The range samples and the localisation data of IntTools.</summary>
public class IntTools_RangeSampleTests
{
    [Fact]
    public void BaseRangeSample()
    {
        Assert.Equal(0, new IntTools_BaseRangeSample().GetDepth());
        IntTools_BaseRangeSample aSample = new(3);
        Assert.Equal(3, aSample.GetDepth());
        aSample.SetDepth(4);
        Assert.Equal(new IntTools_BaseRangeSample(4), aSample);
        Assert.Equal(5, aSample.WithDepth(5).GetDepth());
    }

    [Fact]
    public void CurveRangeSample_IndexDepthAndRange()
    {
        IntTools_CurveRangeSample aSample = new(4);
        Assert.Equal(4, aSample.GetRangeIndex());
        Assert.Equal(0, aSample.GetDepth());

        // depth 0: the whole range
        Assert.Equal(new IntTools_Range(0.0, 9.0), aSample.GetRange(0.0, 9.0, 3));

        // depth 2, 3 samples: 9 parts of length 1; index 4 is [4, 5]
        aSample.SetDepth(2);
        Assert.Equal(new IntTools_Range(4.0, 5.0), aSample.GetRange(0.0, 9.0, 3));
        Assert.Equal(12, aSample.GetRangeIndexDeeper(3));
        Assert.Equal(new IntTools_BaseRangeSample(2), aSample.BaseRangeSample());
    }

    [Fact]
    public void CurveRangeSample_EqualityAndHashing()
    {
        IntTools_CurveRangeSample a = new IntTools_CurveRangeSample(2).WithDepth(1);
        IntTools_CurveRangeSample b = new IntTools_CurveRangeSample().WithRangeIndex(2).WithDepth(1);
        Assert.True(a.IsEqual(b));
        Assert.Equal(a, b);
        Assert.True(a == b);
        Assert.Equal(a.GetHashCode(), b.GetHashCode());
        Assert.NotEqual(a, b.WithDepth(2));
        Assert.NotEqual(a, b.WithRangeIndex(3));

        NCollection_Map<IntTools_CurveRangeSample> aMap = new();
        Assert.True(aMap.Add(a));
        Assert.False(aMap.Add(b));
        Assert.True(aMap.Add(b.WithDepth(2)));
    }

    [Fact]
    public void SurfaceRangeSample_RangesIndicesDepths()
    {
        IntTools_SurfaceRangeSample aSample = new(1, 2, 3, 4);
        aSample.GetIndexes(out int iu, out int iv);
        aSample.GetDepths(out int du, out int dv);
        Assert.Equal((1, 3, 2, 4), (iu, iv, du, dv));
        Assert.Equal(1, aSample.GetIndexU());
        Assert.Equal(4, aSample.GetDepthV());

        aSample.GetRanges(out IntTools_CurveRangeSample aRU, out IntTools_CurveRangeSample aRV);
        Assert.Equal(new IntTools_CurveRangeSample(1).WithDepth(2), aRU);
        Assert.Equal(new IntTools_CurveRangeSample(3).WithDepth(4), aRV);
        Assert.Equal(aRU, aSample.GetSampleRangeU());
        Assert.Equal(new IntTools_SurfaceRangeSample(aRU, aRV), aSample);

        // U: depth 2 of 2 samples cuts [0, 8] into 4 parts; index 1 is [2, 4]
        Assert.Equal(new IntTools_Range(2.0, 4.0), aSample.GetRangeU(0.0, 8.0, 2));
        Assert.Equal(2, aSample.GetRangeIndexUDeeper(2));
        Assert.Equal(6, aSample.GetRangeIndexVDeeper(2));

        aSample.SetIndexes(5, 6);
        aSample.SetDepthU(0);
        Assert.Equal(new IntTools_SurfaceRangeSample(5, 0, 6, 4), aSample);
        Assert.Equal(new IntTools_Range(0.0, 8.0), aSample.GetRangeU(0.0, 8.0, 2));
        Assert.Equal(aSample.GetHashCode(), new IntTools_SurfaceRangeSample(5, 0, 6, 4).GetHashCode());
        Assert.NotEqual(aSample, aSample.WithIndexV(7));
        Assert.True(aSample != aSample.WithDepthU(1));

        IntTools_SurfaceRangeSample aCopy = aSample; // OCCT's copy constructor and Assign are a plain copy
        aCopy.SetIndexU(9);
        Assert.Equal(5, aSample.GetIndexU());
    }

    [Fact]
    public void CurveRangeLocalizeData_OutRangesAndBoxes()
    {
        IntTools_CurveRangeLocalizeData aData = new(10, 1.0e-3);
        Assert.Equal(10, aData.GetNbSample());
        Assert.Equal(1.0e-3, aData.GetMinRange());

        IntTools_CurveRangeSample aS1 = new(1), aS2 = new(2), aS3 = new IntTools_CurveRangeSample(3).WithDepth(1);
        Box aBox = new();
        aBox.Update(0, 0, 0, 1, 1, 1);
        aData.AddBox(aS1, aBox);
        aData.AddBox(aS2, aBox);

        Box aFound = new();
        Assert.True(aData.FindBox(aS1, ref aFound));
        Assert.False(aFound.IsVoid);
        Box aMissing = new();
        Assert.False(aData.FindBox(aS3, ref aMissing));
        Assert.True(aMissing.IsVoid);

        // an out range forgets its box
        aData.AddOutRange(aS2);
        aData.AddOutRange(aS3);
        aData.AddOutRange(aS2);
        Assert.True(aData.IsRangeOut(aS2));
        Assert.False(aData.IsRangeOut(aS1));
        Assert.False(aData.FindBox(aS2, ref aMissing));

        // listed in insertion order (OCCT: hash order), appended to the list
        NCollection_List<IntTools_CurveRangeSample> aList = new();
        aList.Append(aS1);
        aData.ListRangeOut(aList);
        Assert.Equal(new[] { aS1, aS2, aS3 }, aList);
    }

    [Fact]
    public void SurfaceRangeLocalizeData_OutRangesAndBoxes()
    {
        IntTools_SurfaceRangeLocalizeData aDefault = new();
        Assert.Equal((1, 1, 0.0, 0.0), (aDefault.GetNbSampleU(), aDefault.GetNbSampleV(), aDefault.GetMinRangeU(), aDefault.GetMinRangeV()));

        IntTools_SurfaceRangeLocalizeData aData = new(3, 4, 0.1, 0.2);
        Assert.Equal((3, 4, 0.1, 0.2), (aData.GetNbSampleU(), aData.GetNbSampleV(), aData.GetMinRangeU(), aData.GetMinRangeV()));

        IntTools_SurfaceRangeSample aS1 = new(0, 1, 0, 1), aS2 = new(1, 1, 0, 1);
        Box aBox = new();
        aBox.Update(0, 0, 0, 1, 1, 1);
        aData.AddBox(aS1, aBox);
        aData.AddOutRange(aS1);
        aData.AddOutRange(aS2);
        Box aFound = new();
        Assert.True(aData.FindBox(aS1, ref aFound)); // the surface's out range keeps its box
        Assert.True(aData.IsRangeOut(aS2));

        NCollection_List<IntTools_SurfaceRangeSample> aList = new();
        aData.ListRangeOut(aList);
        Assert.Equal(new[] { aS1, aS2 }, aList);

        // a copy has maps of its own
        IntTools_SurfaceRangeLocalizeData aCopy = new(aData);
        aCopy.RemoveRangeOutAll();
        Assert.False(aCopy.IsRangeOut(aS1));
        Assert.True(aData.IsRangeOut(aS1));
        Assert.True(aCopy.FindBox(aS1, ref aFound));
    }

    [Fact]
    public void SurfaceRangeLocalizeData_GridAndFrame()
    {
        IntTools_SurfaceRangeLocalizeData aData = new(3, 3, 0.0, 0.0);
        Assert.Equal(0, aData.GetRangeUGrid());
        aData.SetGridDeflection(0.01);
        Assert.Equal(0.01, aData.GetGridDeflection());

        aData.SetRangeUGrid(3);
        aData.SetRangeVGrid(2); // the grid of points is made when both are set
        Assert.Equal(3, aData.GetRangeUGrid());
        Assert.Equal(2, aData.GetRangeVGrid());
        for (int i = 1; i <= 3; ++i)
        {
            aData.SetUParam(i, i - 1.0);
            for (int j = 1; j <= 2; ++j)
            {
                aData.SetGridPoint(i, j, new Point3D(i, j, 0));
            }
        }
        aData.SetVParam(1, 0.0);
        aData.SetVParam(2, 1.0);
        Assert.Equal(2.0, aData.GetUParam(3));
        Assert.Equal(new Point3D(2, 1, 0), aData.GetGridPoint(2, 1));

        // the frame keeps the parameters strictly inside: U 1, 2 (indices 2, 3), V 0 (index 1)
        aData.SetFrame(0.5, 2.5, -1.0, 0.5);
        Assert.Equal(2, aData.GetNBUPointsInFrame());
        Assert.Equal(1, aData.GetNBVPointsInFrame());
        Assert.Equal(1.0, aData.GetUParamInFrame(1));
        Assert.Equal(2.0, aData.GetUParamInFrame(2));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aData.GetUParamInFrame(3));
        Assert.Equal(0.0, aData.GetVParamInFrame(1));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aData.GetVParamInFrame(2));
        Assert.Equal(new Point3D(2, 1, 0), aData.GetPointInFrame(1, 1));
        Assert.Equal(new Point3D(0, 0, 0), aData.GetPointInFrame(1, 2)); // outside: the origin

        // a frame above every parameter is empty: min 4 (none found), max 3
        aData.SetFrame(5.0, 6.0, 5.0, 6.0);
        Assert.Equal(0, aData.GetNBUPointsInFrame());

        // a copy shares the grid arrays (OCCT copies the handles)
        IntTools_SurfaceRangeLocalizeData aCopy = new(aData);
        aCopy.SetUParam(1, 42.0);
        Assert.Equal(42.0, aData.GetUParam(1));

        // the same size keeps the arrays; ClearGrid drops them
        aData.SetRangeUGrid(3);
        Assert.Equal(42.0, aData.GetUParam(1));
        aData.ClearGrid();
        Assert.Equal(0, aData.GetRangeUGrid());
        Assert.Equal(0, aData.GetRangeVGrid());
        Assert.Equal(0.0, aData.GetGridDeflection());
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Infinite(), aData.GetUParamInFrame(1));
        Assert.Equal(new Point3D(0, 0, 0), aData.GetPointInFrame(1, 1));

        // Assign keeps this object's grid when the other has none
        aCopy.Assign(aData);
        Assert.Equal(3, aCopy.GetRangeUGrid());
    }
}
