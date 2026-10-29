// Port of: src/FoundationClasses/TKMath/GTests/Bnd_Range_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class Bnd_RangeTest
{
    [Fact]
    public void DefaultConstructor_IsVoid()
    {
        Bnd_Range aRange = new();
        Assert.True(aRange.IsVoid());
        Assert.True(aRange.Delta() < 0.0);
    }

    [Fact]
    public void ParameterizedConstructor()
    {
        Bnd_Range aRange = new(3.0, 15.0);
        Assert.False(aRange.IsVoid());
        GTest.ExpectDoubleEq(aRange.Delta(), 12.0);
    }

    [Fact]
    public void ParameterizedConstructor_InvalidRange()
    {
        Assert.ThrowsAny<Standard_ConstructionError>(() => new Bnd_Range(10.0, 5.0));
    }

    [Fact]
    public void ParameterizedConstructor_PointRange()
    {
        Bnd_Range aRange = new(5.0, 5.0);
        Assert.False(aRange.IsVoid());
        GTest.ExpectDoubleEq(aRange.Delta(), 0.0);
    }

    [Fact]
    public void GetMin_GetMax_GetBounds()
    {
        Bnd_Range aRange = new(2.0, 8.0);
        double    aMin = 0.0, aMax = 0.0;
        Assert.True(aRange.GetMin(ref aMin));
        GTest.ExpectDoubleEq(aMin, 2.0);
        Assert.True(aRange.GetMax(ref aMax));
        GTest.ExpectDoubleEq(aMax, 8.0);

        double aFirst = 0.0, aLast = 0.0;
        Assert.True(aRange.GetBounds(ref aFirst, ref aLast));
        GTest.ExpectDoubleEq(aFirst, 2.0);
        GTest.ExpectDoubleEq(aLast, 8.0);
    }

    [Fact]
    public void GetMin_GetMax_Void()
    {
        Bnd_Range aRange = new();
        double    aVal = 0.0;
        Assert.False(aRange.GetMin(ref aVal));
        Assert.False(aRange.GetMax(ref aVal));

        double aFirst = 0.0, aLast = 0.0;
        Assert.False(aRange.GetBounds(ref aFirst, ref aLast));
    }

    [Fact]
    public void GetIntermediatePoint()
    {
        Bnd_Range aRange = new(10.0, 20.0);
        double    aPar = 0.0;
        Assert.True(aRange.GetIntermediatePoint(0.0, ref aPar));
        GTest.ExpectDoubleEq(aPar, 10.0);
        Assert.True(aRange.GetIntermediatePoint(0.5, ref aPar));
        GTest.ExpectDoubleEq(aPar, 15.0);
        Assert.True(aRange.GetIntermediatePoint(1.0, ref aPar));
        GTest.ExpectDoubleEq(aPar, 20.0);
    }

    [Fact]
    public void GetIntermediatePoint_Void()
    {
        Bnd_Range aRange = new();
        double    aPar = 0.0;
        Assert.False(aRange.GetIntermediatePoint(0.5, ref aPar));
    }

    [Fact]
    public void SetVoid()
    {
        Bnd_Range aRange = new(1.0, 5.0);
        Assert.False(aRange.IsVoid());
        aRange.SetVoid();
        Assert.True(aRange.IsVoid());
    }

    [Fact]
    public void Add_Double_ToVoid()
    {
        Bnd_Range aRange = new();
        aRange.Add(5.0);
        Assert.False(aRange.IsVoid());
        double aMin = 0.0, aMax = 0.0;
        Assert.True(aRange.GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 5.0);
        GTest.ExpectDoubleEq(aMax, 5.0);
    }

    [Fact]
    public void Add_Double_Extends()
    {
        Bnd_Range aRange = new(3.0, 7.0);
        aRange.Add(1.0);
        aRange.Add(10.0);
        double aMin = 0.0, aMax = 0.0;
        Assert.True(aRange.GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 1.0);
        GTest.ExpectDoubleEq(aMax, 10.0);
    }

    [Fact]
    public void Add_Range_ToVoid()
    {
        Bnd_Range aRange = new();
        Bnd_Range anOther = new(3.0, 7.0);
        aRange.Add(anOther);
        double aMin = 0.0, aMax = 0.0;
        Assert.True(aRange.GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 3.0);
        GTest.ExpectDoubleEq(aMax, 7.0);
    }

    [Fact]
    public void Add_VoidRange()
    {
        Bnd_Range aRange = new(3.0, 7.0);
        Bnd_Range aVoid = new();
        aRange.Add(aVoid);
        // Should remain unchanged
        double aMin = 0.0, aMax = 0.0;
        Assert.True(aRange.GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 3.0);
        GTest.ExpectDoubleEq(aMax, 7.0);
    }

    [Fact]
    public void Add_Range_VoidToVoid()
    {
        Bnd_Range aRange = new();
        Bnd_Range aVoid = new();
        aRange.Add(aVoid);
        Assert.True(aRange.IsVoid());
    }

    [Fact]
    public void Add_Range_Extends()
    {
        Bnd_Range aRange = new(3.0, 7.0);
        Bnd_Range anOther = new(1.0, 10.0);
        aRange.Add(anOther);
        double aMin = 0.0, aMax = 0.0;
        Assert.True(aRange.GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 1.0);
        GTest.ExpectDoubleEq(aMax, 10.0);
    }

    [Fact]
    public void Common_Overlapping()
    {
        Bnd_Range aRange = new(1.0, 10.0);
        Bnd_Range anOther = new(5.0, 15.0);
        aRange.Common(anOther);
        double aMin = 0.0, aMax = 0.0;
        Assert.True(aRange.GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 5.0);
        GTest.ExpectDoubleEq(aMax, 10.0);
    }

    [Fact]
    public void Common_NoOverlap()
    {
        Bnd_Range aRange = new(1.0, 5.0);
        Bnd_Range anOther = new(7.0, 10.0);
        aRange.Common(anOther);
        Assert.True(aRange.IsVoid());
    }

    [Fact]
    public void Common_WithVoid()
    {
        Bnd_Range aRange = new(1.0, 5.0);
        Bnd_Range aVoid = new();
        aRange.Common(aVoid);
        Assert.True(aRange.IsVoid());
    }

    [Fact]
    public void Union_Overlapping()
    {
        Bnd_Range aRange = new(1.0, 7.0);
        Bnd_Range anOther = new(5.0, 15.0);
        Assert.True(aRange.Union(anOther));
        double aMin = 0.0, aMax = 0.0;
        Assert.True(aRange.GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 1.0);
        GTest.ExpectDoubleEq(aMax, 15.0);
    }

    [Fact]
    public void Union_Separated()
    {
        Bnd_Range aRange = new(1.0, 5.0);
        Bnd_Range anOther = new(7.0, 10.0);
        Assert.False(aRange.Union(anOther));
        // aRange should remain unchanged
        double aMin = 0.0, aMax = 0.0;
        Assert.True(aRange.GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 1.0);
        GTest.ExpectDoubleEq(aMax, 5.0);
    }

    [Fact]
    public void Union_WithVoid()
    {
        Bnd_Range aRange = new(1.0, 5.0);
        Bnd_Range aVoid = new();
        Assert.False(aRange.Union(aVoid));
    }

    [Fact]
    public void IsIntersected_Void()
    {
        Bnd_Range aRange = new();
        Assert.Equal(Bnd_Range.IntersectStatus.IntersectStatus_Out, aRange.IsIntersected(5.0));
    }

    [Fact]
    public void IsIntersected_Contains()
    {
        Bnd_Range aRange = new(3.0, 15.0);
        Assert.Equal(Bnd_Range.IntersectStatus.IntersectStatus_In, aRange.IsIntersected(5.0));
    }

    [Fact]
    public void IsIntersected_Outside()
    {
        Bnd_Range aRange = new(3.0, 15.0);
        Assert.Equal(Bnd_Range.IntersectStatus.IntersectStatus_Out, aRange.IsIntersected(20.0));
        Assert.Equal(Bnd_Range.IntersectStatus.IntersectStatus_Out, aRange.IsIntersected(1.0));
    }

    [Fact]
    public void IsIntersected_OnBoundary()
    {
        Bnd_Range aRange = new(3.0, 15.0);
        Assert.Equal(Bnd_Range.IntersectStatus.IntersectStatus_Boundary, aRange.IsIntersected(3.0));
        Assert.Equal(Bnd_Range.IntersectStatus.IntersectStatus_Boundary, aRange.IsIntersected(15.0));
    }

    [Fact]
    public void IsIntersected_PointRange()
    {
        // When myFirst == myLast, only Out or Boundary possible
        Bnd_Range aRange = new(5.0, 5.0);
        Assert.Equal(Bnd_Range.IntersectStatus.IntersectStatus_Boundary, aRange.IsIntersected(5.0));
        Assert.Equal(Bnd_Range.IntersectStatus.IntersectStatus_Out, aRange.IsIntersected(3.0));
    }

    [Fact]
    public void IsIntersected_Periodic_Contains()
    {
        // Range [3, 15], period 4, val 1
        // Check: val + k*period in [3, 15]
        // k=1: 1+4=5, inside [3,15] -> In
        Bnd_Range aRange = new(3.0, 15.0);
        Assert.Equal(Bnd_Range.IntersectStatus.IntersectStatus_In, aRange.IsIntersected(1.0, 4.0));
    }

    [Fact]
    public void IsIntersected_Periodic_OnBoundary()
    {
        // Range [3, 15], period 4, val 3 -> val + 0*4 = 3 exactly
        Bnd_Range aRange = new(3.0, 15.0);
        Assert.Equal(Bnd_Range.IntersectStatus.IntersectStatus_Boundary, aRange.IsIntersected(3.0, 4.0));
    }

    [Fact]
    public void IsIntersected_Periodic_Outside()
    {
        // Range [3.5, 3.9], period 4, val 0
        // Check: 0 + k*4 in [3.5, 3.9] -> k=1 gives 4.0, not in range
        Bnd_Range aRange = new(3.5, 3.9);
        Assert.Equal(Bnd_Range.IntersectStatus.IntersectStatus_Out, aRange.IsIntersected(0.0, 4.0));
    }

    [Fact]
    public void Split_NoIntersection()
    {
        Bnd_Range aRange = new(3.0, 15.0);
        NCollection_List<Bnd_Range> aList = new();
        aRange.Split(20.0, aList);
        Assert.Equal(1, aList.Size());
        double aMin = 0.0, aMax = 0.0;
        Assert.True(aList.First().GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 3.0);
        GTest.ExpectDoubleEq(aMax, 15.0);
    }

    [Fact]
    public void Split_AtInteriorPoint()
    {
        Bnd_Range aRange = new(3.0, 15.0);
        NCollection_List<Bnd_Range> aList = new();
        aRange.Split(5.0, aList);
        Assert.Equal(2, aList.Size());
        double aMin1 = 0.0, aMax1 = 0.0;
        Assert.True(aList.First().GetBounds(ref aMin1, ref aMax1));
        GTest.ExpectDoubleEq(aMin1, 3.0);
        GTest.ExpectDoubleEq(aMax1, 5.0);
        double aMin2 = 0.0, aMax2 = 0.0;
        Assert.True(aList.Last().GetBounds(ref aMin2, ref aMax2));
        GTest.ExpectDoubleEq(aMin2, 5.0);
        GTest.ExpectDoubleEq(aMax2, 15.0);
    }

    [Fact]
    public void Split_Periodic()
    {
        // Range [3, 15], split at val=5, period=4
        // Split points: 5, 9, 13 -> sub-ranges: [3,5], [5,9], [9,13], [13,15]
        Bnd_Range aRange = new(3.0, 15.0);
        NCollection_List<Bnd_Range> aList = new();
        aRange.Split(5.0, aList, 4.0);
        Assert.Equal(4, aList.Size());
    }

    [Fact]
    public void Enlarge()
    {
        Bnd_Range aRange = new(5.0, 10.0);
        aRange.Enlarge(2.0);
        double aMin = 0.0, aMax = 0.0;
        Assert.True(aRange.GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 3.0);
        GTest.ExpectDoubleEq(aMax, 12.0);
    }

    [Fact]
    public void Enlarge_Void()
    {
        Bnd_Range aRange = new();
        aRange.Enlarge(2.0);
        Assert.True(aRange.IsVoid());
    }

    [Fact]
    public void Shifted()
    {
        Bnd_Range aRange = new(3.0, 7.0);
        Bnd_Range aShifted = aRange.Shifted(10.0);
        double    aMin = 0.0, aMax = 0.0;
        Assert.True(aShifted.GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 13.0);
        GTest.ExpectDoubleEq(aMax, 17.0);
        // Original unchanged
        Assert.True(aRange.GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 3.0);
        GTest.ExpectDoubleEq(aMax, 7.0);
    }

    [Fact]
    public void Shifted_Void()
    {
        Bnd_Range aRange = new();
        Bnd_Range aShifted = aRange.Shifted(10.0);
        Assert.True(aShifted.IsVoid());
    }

    [Fact]
    public void Shift()
    {
        Bnd_Range aRange = new(3.0, 7.0);
        aRange.Shift(10.0);
        double aMin = 0.0, aMax = 0.0;
        Assert.True(aRange.GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 13.0);
        GTest.ExpectDoubleEq(aMax, 17.0);
    }

    [Fact]
    public void TrimFrom()
    {
        Bnd_Range aRange = new(3.0, 10.0);
        aRange.TrimFrom(5.0);
        double aMin = 0.0, aMax = 0.0;
        Assert.True(aRange.GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 5.0);
        GTest.ExpectDoubleEq(aMax, 10.0);
    }

    [Fact]
    public void TrimFrom_MakesVoid()
    {
        Bnd_Range aRange = new(3.0, 10.0);
        aRange.TrimFrom(15.0);
        Assert.True(aRange.IsVoid());
    }

    [Fact]
    public void TrimTo()
    {
        Bnd_Range aRange = new(3.0, 10.0);
        aRange.TrimTo(7.0);
        double aMin = 0.0, aMax = 0.0;
        Assert.True(aRange.GetBounds(ref aMin, ref aMax));
        GTest.ExpectDoubleEq(aMin, 3.0);
        GTest.ExpectDoubleEq(aMax, 7.0);
    }

    [Fact]
    public void TrimTo_MakesVoid()
    {
        Bnd_Range aRange = new(3.0, 10.0);
        aRange.TrimTo(1.0);
        Assert.True(aRange.IsVoid());
    }

    [Fact]
    public void IsOut_Value()
    {
        Bnd_Range aRange = new(3.0, 10.0);
        Assert.False(aRange.IsOut(5.0));
        Assert.False(aRange.IsOut(3.0));
        Assert.False(aRange.IsOut(10.0));
        Assert.True(aRange.IsOut(1.0));
        Assert.True(aRange.IsOut(15.0));
    }

    [Fact]
    public void IsOut_Void()
    {
        Bnd_Range aRange = new();
        Assert.True(aRange.IsOut(5.0));
    }

    [Fact]
    public void IsOut_Range()
    {
        Bnd_Range aRange = new(3.0, 10.0);
        Assert.False(aRange.IsOut(new Bnd_Range(5.0, 8.0)));
        Assert.False(aRange.IsOut(new Bnd_Range(1.0, 5.0)));
        Assert.True(aRange.IsOut(new Bnd_Range(11.0, 15.0)));
    }

    [Fact]
    public void IsOut_Range_Void()
    {
        Bnd_Range aRange = new(3.0, 10.0);
        Bnd_Range aVoid = new();
        Assert.True(aRange.IsOut(aVoid));
        Assert.True(aVoid.IsOut(aRange));
    }

    [Fact]
    public void Equality()
    {
        Bnd_Range aRange1 = new(3.0, 10.0);
        Bnd_Range aRange2 = new(3.0, 10.0);
        Bnd_Range aRange3 = new(3.0, 11.0);
        Assert.True(aRange1 == aRange2);
        Assert.False(aRange1 == aRange3);
    }

    [Fact]
    public void Contains_Value()
    {
        Bnd_Range aRange = new(3.0, 10.0);
        Assert.True(aRange.Contains(5.0));
        Assert.True(aRange.Contains(3.0));
        Assert.True(aRange.Contains(10.0));
        Assert.False(aRange.Contains(2.0));
        Assert.False(aRange.Contains(11.0));

        Bnd_Range aVoid = new();
        Assert.False(aVoid.Contains(5.0));
    }

    [Fact]
    public void Intersects_Range()
    {
        Bnd_Range aRange1 = new(3.0, 10.0);
        Bnd_Range aRange2 = new(5.0, 15.0);
        Bnd_Range aRange3 = new(11.0, 20.0);
        Assert.True(aRange1.Intersects(aRange2));
        Assert.False(aRange1.Intersects(aRange3));

        Bnd_Range aVoid = new();
        Assert.False(aRange1.Intersects(aVoid));
    }

    [Fact]
    public void Min_Max_DirectAccess()
    {
        Bnd_Range aRange = new(3.0, 10.0);
        Assert.True(aRange.Min().HasValue);
        Assert.True(aRange.Max().HasValue);
        GTest.ExpectDoubleEq(aRange.Min()!.Value, 3.0);
        GTest.ExpectDoubleEq(aRange.Max()!.Value, 10.0);
    }

    [Fact]
    public void Min_Max_NulloptOnVoid()
    {
        Bnd_Range aVoid = new();
        Assert.False(aVoid.Min().HasValue);
        Assert.False(aVoid.Max().HasValue);
    }

    [Fact]
    public void Get_StructuredBindings()
    {
        Bnd_Range aRange = new(3.0, 10.0);
        var      aBounds = aRange.Get();
        Assert.True(aBounds.HasValue);
        var (aMin, aMax) = aBounds.Value;
        GTest.ExpectDoubleEq(aMin, 3.0);
        GTest.ExpectDoubleEq(aMax, 10.0);
    }

    [Fact]
    public void Get_NulloptOnVoid()
    {
        Bnd_Range aVoid = new();
        Assert.False(aVoid.Get().HasValue);
    }

    [Fact]
    public void Center()
    {
        Bnd_Range aRange = new(2.0, 8.0);
        Assert.True(aRange.Center().HasValue);
        GTest.ExpectDoubleEq(aRange.Center()!.Value, 5.0);

        Bnd_Range aPoint = new(3.0, 3.0);
        Assert.True(aPoint.Center().HasValue);
        GTest.ExpectDoubleEq(aPoint.Center()!.Value, 3.0);
    }

    [Fact]
    public void Center_NulloptOnVoid()
    {
        Bnd_Range aVoid = new();
        Assert.False(aVoid.Center().HasValue);
    }
}
