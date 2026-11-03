// Port of: src/FoundationClasses/TKMath/GTests/Bnd_Sphere_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

namespace Kerf.Kernel.Foundation.Tests;

public class Bnd_SphereTest
{
    [Fact]
    public void DefaultConstructor()
    {
        BoundingSphere aSphere = new();
        GTest.ExpectDoubleEq(aSphere.Center().X(), 0.0);
        GTest.ExpectDoubleEq(aSphere.Center().Y(), 0.0);
        GTest.ExpectDoubleEq(aSphere.Center().Z(), 0.0);
        GTest.ExpectDoubleEq(aSphere.Radius(), 0.0);
        Assert.False(aSphere.IsValid);
        Assert.Equal(0, aSphere.U());
        Assert.Equal(0, aSphere.V());
    }

    [Fact]
    public void ParameterizedConstructor()
    {
        Xyz aCenter = new(1.0, 2.0, 3.0);
        BoundingSphere aSphere = new(aCenter, 5.0, 10, 20);
        GTest.ExpectDoubleEq(aSphere.Center().X(), 1.0);
        GTest.ExpectDoubleEq(aSphere.Center().Y(), 2.0);
        GTest.ExpectDoubleEq(aSphere.Center().Z(), 3.0);
        GTest.ExpectDoubleEq(aSphere.Radius(), 5.0);
        Assert.Equal(10, aSphere.U());
        Assert.Equal(20, aSphere.V());
    }

    [Fact]
    public void Validity()
    {
        BoundingSphere aSphere = new();
        Assert.False(aSphere.IsValid);
        aSphere.SetValid(true);
        Assert.True(aSphere.IsValid);
        aSphere.SetValid(false);
        Assert.False(aSphere.IsValid);
    }

    [Fact]
    public void Distance()
    {
        Xyz aCenter = new(0.0, 0.0, 0.0);
        BoundingSphere aSphere = new(aCenter, 1.0, 0, 0);
        Xyz aPoint = new(3.0, 4.0, 0.0);
        double aDist = aSphere.Distance(aPoint);
        GTest.ExpectDoubleEq(aDist, 5.0);
    }

    [Fact]
    public void SquareDistance()
    {
        Xyz aCenter = new(0.0, 0.0, 0.0);
        BoundingSphere aSphere = new(aCenter, 1.0, 0, 0);
        Xyz aPoint = new(3.0, 4.0, 0.0);
        GTest.ExpectDoubleEq(aSphere.SquareDistance(aPoint), 25.0);
    }

    [Fact]
    public void Distances_PointOutside()
    {
        Xyz aCenter = new(0.0, 0.0, 0.0);
        BoundingSphere aSphere = new(aCenter, 2.0, 0, 0);
        Xyz aPoint = new(5.0, 0.0, 0.0);
        double       aMin = 0.0, aMax = 0.0;
        aSphere.Distances(aPoint, out aMin, out aMax);
        GTest.ExpectDoubleEq(aMin, 3.0);
        GTest.ExpectDoubleEq(aMax, 7.0);
    }

    [Fact]
    public void Distances_PointInside()
    {
        Xyz aCenter = new(0.0, 0.0, 0.0);
        BoundingSphere aSphere = new(aCenter, 5.0, 0, 0);
        Xyz aPoint = new(1.0, 0.0, 0.0);
        double       aMin = 0.0, aMax = 0.0;
        aSphere.Distances(aPoint, out aMin, out aMax);
        GTest.ExpectDoubleEq(aMin, 0.0);
        GTest.ExpectDoubleEq(aMax, 6.0);
    }

    [Fact]
    public void SquareDistances_PointOutside()
    {
        Xyz aCenter = new(0.0, 0.0, 0.0);
        BoundingSphere aSphere = new(aCenter, 2.0, 0, 0);
        Xyz aPoint = new(5.0, 0.0, 0.0);
        double       aMin = 0.0, aMax = 0.0;
        aSphere.SquareDistances(aPoint, out aMin, out aMax);
        // d^2 = 25, r^2 = 4
        // min = d^2 - r^2 = 21, max = d^2 + r^2 = 29
        GTest.ExpectDoubleEq(aMin, 21.0);
        GTest.ExpectDoubleEq(aMax, 29.0);
    }

    [Fact]
    public void SquareDistances_PointInside()
    {
        Xyz aCenter = new(0.0, 0.0, 0.0);
        BoundingSphere aSphere = new(aCenter, 5.0, 0, 0);
        Xyz aPoint = new(1.0, 0.0, 0.0);
        double       aMin = 0.0, aMax = 0.0;
        aSphere.SquareDistances(aPoint, out aMin, out aMax);
        // d^2 = 1, r^2 = 25
        // d^2 < r^2 -> min = 0, max = d^2 + r^2 = 26
        GTest.ExpectDoubleEq(aMin, 0.0);
        GTest.ExpectDoubleEq(aMax, 26.0);
    }

    [Fact]
    public void SquareDistances_PointAtCenter()
    {
        Xyz aCenter = new(0.0, 0.0, 0.0);
        BoundingSphere aSphere = new(aCenter, 3.0, 0, 0);
        Xyz aPoint = new(0.0, 0.0, 0.0);
        double       aMin = 0.0, aMax = 0.0;
        aSphere.SquareDistances(aPoint, out aMin, out aMax);
        // d^2 = 0, r^2 = 9
        // d^2 < r^2 -> min = 0, max = 0 + 9 = 9
        GTest.ExpectDoubleEq(aMin, 0.0);
        GTest.ExpectDoubleEq(aMax, 9.0);
    }

    [Fact]
    public void SquareDistances_PointOnSurface()
    {
        Xyz aCenter = new(0.0, 0.0, 0.0);
        BoundingSphere aSphere = new(aCenter, 3.0, 0, 0);
        Xyz aPoint = new(3.0, 0.0, 0.0);
        double       aMin = 0.0, aMax = 0.0;
        aSphere.SquareDistances(aPoint, out aMin, out aMax);
        // d^2 = 9, r^2 = 9
        // d^2 < r^2 is false (equal), so min = d^2 - r^2 = 0, max = d^2 + r^2 = 18
        GTest.ExpectDoubleEq(aMin, 0.0);
        GTest.ExpectDoubleEq(aMax, 18.0);
    }

    [Fact]
    public void Project()
    {
        Xyz aCenter = new(1.0, 2.0, 3.0);
        BoundingSphere aSphere = new(aCenter, 5.0, 0, 0);
        Xyz aNode = new(10.0, 20.0, 30.0);
        Xyz aProjNode = new();
        double       aDist    = 0.0;
        bool         anInside = false;
        bool   isOk     = aSphere.Project(aNode, out aProjNode, out aDist, out anInside);
        Assert.True(isOk);
        Assert.True(anInside);
        GTest.ExpectDoubleEq(aProjNode.X(), aCenter.X());
        GTest.ExpectDoubleEq(aProjNode.Y(), aCenter.Y());
        GTest.ExpectDoubleEq(aProjNode.Z(), aCenter.Z());
    }

    [Fact]
    public void Add_EnclosingSphere()
    {
        BoundingSphere aSphere1 = new(new Xyz(0.0, 0.0, 0.0), 10.0, 0, 0);
        BoundingSphere aSphere2 = new(new Xyz(1.0, 0.0, 0.0), 2.0, 0, 0);
        aSphere1.Add(aSphere2);
        // aSphere1 already encloses aSphere2, should remain unchanged
        GTest.ExpectDoubleEq(aSphere1.Radius(), 10.0);
    }

    [Fact]
    public void Add_EnclosedBySphere()
    {
        BoundingSphere aSphere1 = new(new Xyz(0.0, 0.0, 0.0), 2.0, 0, 0);
        BoundingSphere aSphere2 = new(new Xyz(0.0, 0.0, 0.0), 10.0, 0, 0);
        aSphere1.Add(aSphere2);
        // aSphere2 encloses aSphere1, should take aSphere2
        GTest.ExpectDoubleEq(aSphere1.Radius(), 10.0);
    }

    [Fact]
    public void Add_PartialOverlap()
    {
        BoundingSphere aSphere1 = new(new Xyz(0.0, 0.0, 0.0), 3.0, 0, 0);
        BoundingSphere aSphere2 = new(new Xyz(5.0, 0.0, 0.0), 3.0, 0, 0);
        aSphere1.Add(aSphere2);
        // Combined radius should be (5 + 3 + 3) / 2 = 5.5
        GTest.ExpectDoubleEq(aSphere1.Radius(), 5.5);
    }

    [Fact]
    public void IsOut_Separated()
    {
        BoundingSphere aSphere1 = new(new Xyz(0.0, 0.0, 0.0), 1.0, 0, 0);
        BoundingSphere aSphere2 = new(new Xyz(10.0, 0.0, 0.0), 1.0, 0, 0);
        Assert.True(aSphere1.IsOut(aSphere2));
    }

    [Fact]
    public void IsOut_Overlapping()
    {
        BoundingSphere aSphere1 = new(new Xyz(0.0, 0.0, 0.0), 3.0, 0, 0);
        BoundingSphere aSphere2 = new(new Xyz(4.0, 0.0, 0.0), 3.0, 0, 0);
        Assert.False(aSphere1.IsOut(aSphere2));
    }

    [Fact]
    public void IsOut_PointWithMaxDist()
    {
        BoundingSphere aSphere = new(new Xyz(0.0, 0.0, 0.0), 5.0, 0, 0);
        Xyz aPoint = new(20.0, 0.0, 0.0);
        double       aMaxDist = 100.0;
        aSphere.SetValid(true);
        Assert.False(aSphere.IsOut(aPoint, ref aMaxDist));
        // aMaxDist should be updated to aCurMaxDist = 20 + 5 = 25
        GTest.ExpectDoubleEq(aMaxDist, 25.0);
    }

    [Fact]
    public void IsOut_PointTooFar()
    {
        BoundingSphere aSphere = new(new Xyz(0.0, 0.0, 0.0), 1.0, 0, 0);
        Xyz aPoint = new(20.0, 0.0, 0.0);
        double       aMaxDist = 10.0;
        Assert.True(aSphere.IsOut(aPoint, ref aMaxDist));
    }

    [Fact]
    public void SquareExtent()
    {
        BoundingSphere aSphere = new(new Xyz(0.0, 0.0, 0.0), 3.0, 0, 0);
        GTest.ExpectDoubleEq(aSphere.SquareExtent(), 36.0);
    }
}
