// Port of: src/FoundationClasses/TKMath/GTests/Bnd_Box_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class Bnd_BoxTest
{
    [Fact]
    public void DefaultConstructor()
    {
        Box aBox = new();

        Assert.True(aBox.IsVoid, "Default constructed box should be void");
        Assert.False(aBox.IsWhole, "Default constructed box should not be whole");
        GTest.ExpectDoubleEq(0.0, aBox.GetGap());
    }

    [Fact]
    public void PointConstructor()
    {
        Point3D aMinPnt = new(1.0, 2.0, 3.0);
        Point3D aMaxPnt = new(4.0, 5.0, 6.0);

        Box aBox = new(aMinPnt, aMaxPnt);

        Assert.False(aBox.IsVoid, "Box constructed with points should not be void");
        Assert.False(aBox.IsWhole, "Box constructed with points should not be whole");

        double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
        aBox.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        GTest.ExpectDoubleEq(1.0, aXmin);
        GTest.ExpectDoubleEq(2.0, aYmin);
        GTest.ExpectDoubleEq(3.0, aZmin);
        GTest.ExpectDoubleEq(4.0, aXmax);
        GTest.ExpectDoubleEq(5.0, aYmax);
        GTest.ExpectDoubleEq(6.0, aZmax);
    }

    [Fact]
    public void SetWithPoint()
    {
        Box aBox = new();
        Point3D aPnt = new(1.5, 2.5, 3.5);

        aBox.Set(aPnt);

        Assert.False(aBox.IsVoid, "Box should not be void after Set");

        double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
        aBox.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        GTest.ExpectDoubleEq(1.5, aXmin);
        GTest.ExpectDoubleEq(1.5, aXmax);
        GTest.ExpectDoubleEq(2.5, aYmin);
        GTest.ExpectDoubleEq(2.5, aYmax);
        GTest.ExpectDoubleEq(3.5, aZmin);
        GTest.ExpectDoubleEq(3.5, aZmax);
    }

    [Fact]
    public void SetWithPointAndDirection()
    {
        Box aBox = new();
        Point3D aPnt = new(1.0, 2.0, 3.0);
        Direction3D aDir = new(Direction3D.D.X);

        aBox.Set(aPnt, aDir);

        Assert.False(aBox.IsVoid, "Box should not be void after Set");
        Assert.True(aBox.IsOpenXmax, "Box should be open in positive X direction");
        Assert.False(aBox.IsOpenXmin, "Box should not be open in negative X direction");
    }

    [Fact]
    public void UpdateWithBounds()
    {
        Box aBox = new();

        aBox.Update(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

        Assert.False(aBox.IsVoid, "Box should not be void after Update");

        double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
        aBox.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        GTest.ExpectDoubleEq(1.0, aXmin);
        GTest.ExpectDoubleEq(2.0, aYmin);
        GTest.ExpectDoubleEq(3.0, aZmin);
        GTest.ExpectDoubleEq(4.0, aXmax);
        GTest.ExpectDoubleEq(5.0, aYmax);
        GTest.ExpectDoubleEq(6.0, aZmax);
    }

    [Fact]
    public void UpdateWithPoint()
    {
        Box aBox = new();

        // First point establishes the box
        aBox.Update(1.0, 2.0, 3.0);

        double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
        aBox.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        GTest.ExpectDoubleEq(1.0, aXmin);
        GTest.ExpectDoubleEq(1.0, aXmax);

        // Second point expands the box
        aBox.Update(0.5, 2.5, 3.5);
        aBox.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        GTest.ExpectDoubleEq(0.5, aXmin);
        GTest.ExpectDoubleEq(1.0, aXmax);
        GTest.ExpectDoubleEq(2.0, aYmin);
        GTest.ExpectDoubleEq(2.5, aYmax);
    }

    [Fact]
    public void UpdateExpansion()
    {
        Box aBox = new();
        aBox.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

        // Update with bounds that expand the box
        aBox.Update(-1.0, -1.0, -1.0, 2.0, 2.0, 2.0);

        double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
        aBox.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        GTest.ExpectDoubleEq(-1.0, aXmin);
        GTest.ExpectDoubleEq(-1.0, aYmin);
        GTest.ExpectDoubleEq(-1.0, aZmin);
        GTest.ExpectDoubleEq(2.0, aXmax);
        GTest.ExpectDoubleEq(2.0, aYmax);
        GTest.ExpectDoubleEq(2.0, aZmax);
    }

    [Fact]
    public void GapOperations()
    {
        Box aBox = new();

        GTest.ExpectDoubleEq(0.0, aBox.GetGap());

        aBox.SetGap(0.5);
        GTest.ExpectDoubleEq(0.5, aBox.GetGap());

        aBox.Enlarge(0.3);
        GTest.ExpectDoubleEq(0.5, aBox.GetGap());

        aBox.Enlarge(0.8);
        GTest.ExpectDoubleEq(0.8, aBox.GetGap());

        aBox.Enlarge(-1.0);
        GTest.ExpectDoubleEq(1.0, aBox.GetGap());
    }

    [Fact]
    public void CornerMethods()
    {
        Box aBox = new();
        aBox.Update(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        aBox.SetGap(0.1);

        Point3D aCornerMin = aBox.CornerMin();
        Point3D aCornerMax = aBox.CornerMax();

        GTest.ExpectDoubleEq(0.9, aCornerMin.X);
        GTest.ExpectDoubleEq(1.9, aCornerMin.Y);
        GTest.ExpectDoubleEq(2.9, aCornerMin.Z);

        GTest.ExpectDoubleEq(4.1, aCornerMax.X);
        GTest.ExpectDoubleEq(5.1, aCornerMax.Y);
        GTest.ExpectDoubleEq(6.1, aCornerMax.Z);
    }

    [Fact]
    public void CornerMethodsVoidBox()
    {
        Box aBox = new(); // void box

        Assert.ThrowsAny<ConstructionErrorException>(() => aBox.CornerMin());
        Assert.ThrowsAny<ConstructionErrorException>(() => aBox.CornerMax());
    }

    [Fact]
    public void CornerMethodsOpenBox()
    {
        Box aBox = new();
        aBox.Update(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        aBox.OpenXmax();
        aBox.OpenYmin();

        Point3D aCornerMin = aBox.CornerMin();
        Point3D aCornerMax = aBox.CornerMax();

        GTest.ExpectDoubleEq(-1e100, aCornerMin.Y);
        GTest.ExpectDoubleEq(1e100, aCornerMax.X);
    }

    [Fact]
    public void ThinnessMethods()
    {
        Box aBox = new();

        Assert.True(aBox.IsXThin(1.0), "Void box should be considered thin");
        Assert.True(aBox.IsYThin(1.0), "Void box should be considered thin");
        Assert.True(aBox.IsZThin(1.0), "Void box should be considered thin");
        Assert.True(aBox.IsThin(1.0), "Void box should be considered thin");

        // Create a thin box in X direction
        aBox.Update(1.0, 0.0, 0.0, 1.01, 2.0, 2.0);

        Assert.True(aBox.IsXThin(0.1), "Thin X dimension should be detected");
        Assert.False(aBox.IsYThin(0.1), "Thick Y dimension should not be thin");
        Assert.False(aBox.IsZThin(0.1), "Thick Z dimension should not be thin");
        Assert.False(aBox.IsThin(0.1), "Box with only one thin dimension should not be overall thin");

        // Create a box that is thin in all dimensions
        Box aThinBox = new();
        aThinBox.Update(0.0, 0.0, 0.0, 0.01, 0.01, 0.01);

        Assert.True(aThinBox.IsXThin(0.1), "All dimensions should be thin");
        Assert.True(aThinBox.IsYThin(0.1), "All dimensions should be thin");
        Assert.True(aThinBox.IsZThin(0.1), "All dimensions should be thin");
        Assert.True(aThinBox.IsThin(0.1), "Box thin in all dimensions should be overall thin");
    }

    [Fact]
    public void ThinnessWithOpenBox()
    {
        Box aBox = new();
        aBox.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        aBox.OpenXmin();

        Assert.False(aBox.IsXThin(0.1), "Open dimension should not be considered thin");

        aBox.SetWhole();
        Assert.False(aBox.IsXThin(0.1), "Whole box should not be considered thin");
        Assert.False(aBox.IsThin(0.1), "Whole box should not be considered thin");
    }

    [Fact]
    public void TransformationIdentity()
    {
        Box aBox = new();
        aBox.Update(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        aBox.SetGap(0.1);

        Transform3D anIdentity = new();
        Box aTransformed = aBox.Transformed(anIdentity);

        double aXmin1, aYmin1, aZmin1, aXmax1, aYmax1, aZmax1;
        double aXmin2, aYmin2, aZmin2, aXmax2, aYmax2, aZmax2;

        aBox.Get(out aXmin1, out aYmin1, out aZmin1, out aXmax1, out aYmax1, out aZmax1);
        aTransformed.Get(out aXmin2, out aYmin2, out aZmin2, out aXmax2, out aYmax2, out aZmax2);

        GTest.ExpectDoubleEq(aXmin1, aXmin2);
        GTest.ExpectDoubleEq(aYmin1, aYmin2);
        GTest.ExpectDoubleEq(aZmin1, aZmin2);
        GTest.ExpectDoubleEq(aXmax1, aXmax2);
        GTest.ExpectDoubleEq(aYmax1, aYmax2);
        GTest.ExpectDoubleEq(aZmax1, aZmax2);
        GTest.ExpectDoubleEq(aBox.GetGap(), aTransformed.GetGap());
    }

    [Fact]
    public void TransformationTranslation()
    {
        Box aBox = new();
        aBox.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

        Transform3D aTranslation = new Transform3D().WithTranslation(new Vector3D(2.0, 3.0, 4.0));

        Box aTransformed = aBox.Transformed(aTranslation);

        double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
        aTransformed.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        GTest.ExpectDoubleEq(2.0, aXmin);
        GTest.ExpectDoubleEq(3.0, aYmin);
        GTest.ExpectDoubleEq(4.0, aZmin);
        GTest.ExpectDoubleEq(3.0, aXmax);
        GTest.ExpectDoubleEq(4.0, aYmax);
        GTest.ExpectDoubleEq(5.0, aZmax);
    }

    [Fact]
    public void TransformationVoidBox()
    {
        Box aVoidBox = new();
        Transform3D aRotation = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), M_PI / 4);

        Box aTransformed = aVoidBox.Transformed(aRotation);

        Assert.True(aTransformed.IsVoid, "Transformed void box should remain void");
    }

    [Fact]
    public void AddBox()
    {
        Box aBox1 = new();
        aBox1.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        aBox1.SetGap(0.1);

        Box aBox2 = new();
        aBox2.Update(0.5, 0.5, 0.5, 2.0, 2.0, 2.0);
        aBox2.SetGap(0.2);

        aBox1.Add(aBox2);

        double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
        aBox1.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        // Get() returns bounds including gap, so expect gap-adjusted values
        GTest.ExpectDoubleEq(-0.2, aXmin);
        GTest.ExpectDoubleEq(-0.2, aYmin);
        GTest.ExpectDoubleEq(-0.2, aZmin);
        GTest.ExpectDoubleEq(2.2, aXmax);
        GTest.ExpectDoubleEq(2.2, aYmax);
        GTest.ExpectDoubleEq(2.2, aZmax);
        GTest.ExpectDoubleEq(0.2, aBox1.GetGap());
    }

    [Fact]
    public void AddVoidBox()
    {
        Box aBox = new();
        aBox.Update(1.0, 1.0, 1.0, 2.0, 2.0, 2.0);

        Box aVoidBox = new();
        aBox.Add(aVoidBox);

        double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
        aBox.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        GTest.ExpectDoubleEq(1.0, aXmin);
        GTest.ExpectDoubleEq(2.0, aXmax);
    }

    [Fact]
    public void AddToVoidBox()
    {
        Box aVoidBox = new();
        Box aBox = new();
        aBox.Update(1.0, 1.0, 1.0, 2.0, 2.0, 2.0);

        aVoidBox.Add(aBox);

        Assert.False(aVoidBox.IsVoid, "Adding to void box should make it non-void");

        double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
        aVoidBox.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        GTest.ExpectDoubleEq(1.0, aXmin);
        GTest.ExpectDoubleEq(2.0, aXmax);
    }

    [Fact]
    public void AddPoint()
    {
        Box aBox = new();
        aBox.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

        Point3D aPnt = new(2.0, -1.0, 0.5);
        aBox.Add(aPnt);

        double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
        aBox.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        GTest.ExpectDoubleEq(0.0, aXmin);
        GTest.ExpectDoubleEq(-1.0, aYmin);
        GTest.ExpectDoubleEq(0.0, aZmin);
        GTest.ExpectDoubleEq(2.0, aXmax);
        GTest.ExpectDoubleEq(1.0, aYmax);
        GTest.ExpectDoubleEq(1.0, aZmax);
    }

    [Fact]
    public void AddDirection()
    {
        Box aBox = new();
        aBox.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

        Direction3D aDir = new(Direction3D.D.X);
        aBox.Add(aDir);

        Assert.True(aBox.IsOpenXmax, "Positive X direction should open Xmax");
        Assert.False(aBox.IsOpenXmin, "Positive X direction should not open Xmin");
        Assert.False(aBox.IsOpenYmax, "X direction should not affect Y");
        Assert.False(aBox.IsOpenYmin, "X direction should not affect Y");
    }

    [Fact]
    public void AddPointWithDirection()
    {
        Box aBox = new();
        Point3D aPnt = new(1.0, 2.0, 3.0);
        Direction3D aDir = new(Direction3D.D.NX);

        aBox.Add(aPnt, aDir);

        Assert.False(aBox.IsVoid, "Box should not be void after adding point");
        Assert.True(aBox.IsOpenXmin, "Negative X direction should open Xmin");
        Assert.False(aBox.IsOpenXmax, "Negative X direction should not open Xmax");
    }

    [Fact]
    public void IsOutPoint()
    {
        Box aBox = new();
        aBox.Update(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);

        Point3D anInsidePnt = new(1.0, 1.0, 1.0);
        Point3D anOutsidePnt = new(3.0, 1.0, 1.0);
        Point3D aBoundaryPnt = new(2.0, 2.0, 2.0);

        Assert.False(aBox.IsOut(anInsidePnt), "Point inside should not be out");
        Assert.True(aBox.IsOut(anOutsidePnt), "Point outside should be out");
        Assert.False(aBox.IsOut(aBoundaryPnt), "Point on boundary should not be out");
    }

    [Fact]
    public void IsOutPointWithGap()
    {
        Box aBox = new();
        aBox.Update(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
        aBox.SetGap(0.1);

        Point3D aNearBoundaryPnt = new(2.05, 1.0, 1.0);
        Point3D aFarBoundaryPnt = new(2.15, 1.0, 1.0);

        Assert.False(aBox.IsOut(aNearBoundaryPnt), "Point within gap should not be out");
        Assert.True(aBox.IsOut(aFarBoundaryPnt), "Point beyond gap should be out");
    }

    [Fact]
    public void IsOutVoidBox()
    {
        Box aVoidBox = new();
        Point3D anAnyPnt = new(1.0, 2.0, 3.0);

        Assert.True(aVoidBox.IsOut(anAnyPnt), "Void box should exclude all points");
    }

    [Fact]
    public void IsOutWholeBox()
    {
        Box aWholeBox = new();
        aWholeBox.SetWhole();

        Point3D anAnyPnt = new(1000.0, 2000.0, 3000.0);

        Assert.False(aWholeBox.IsOut(anAnyPnt), "Whole box should include all points");
    }

    [Fact]
    public void IsOutBox()
    {
        Box aBox1 = new();
        aBox1.Update(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);

        Box anOverlappingBox = new();
        anOverlappingBox.Update(1.0, 1.0, 1.0, 3.0, 3.0, 3.0);

        Box aSeparateBox = new();
        aSeparateBox.Update(3.0, 3.0, 3.0, 4.0, 4.0, 4.0);

        Box aTouchingBox = new();
        aTouchingBox.Update(2.0, 2.0, 2.0, 3.0, 3.0, 3.0);

        Assert.False(aBox1.IsOut(anOverlappingBox), "Overlapping boxes should not be out");
        Assert.True(aBox1.IsOut(aSeparateBox), "Separate boxes should be out");
        Assert.False(aBox1.IsOut(aTouchingBox), "Touching boxes should not be out");
    }

    [Fact]
    public void IsOutPlane()
    {
        Box aBox = new();
        aBox.Update(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);

        // Plane that intersects the box
        Plane3D anIntersectingPln = new(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.X));

        // Plane that doesn't intersect the box
        Plane3D aSeparatePln = new(new Point3D(2, 0, 0), new Direction3D(Direction3D.D.X));

        Assert.False(aBox.IsOut(anIntersectingPln), "Intersecting plane should not be out");
        Assert.True(aBox.IsOut(aSeparatePln), "Separate plane should be out");
    }

    [Fact]
    public void IsOutLine()
    {
        Box aBox = new();
        aBox.Update(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);

        // Line that passes through the box
        Line3D anIntersectingLin = new(new Point3D(-1, 1, 1), new Direction3D(Direction3D.D.X));

        // Line that misses the box
        Line3D aSeparateLin = new(new Point3D(-1, 3, 1), new Direction3D(Direction3D.D.X));

        Assert.False(aBox.IsOut(anIntersectingLin), "Intersecting line should not be out");
        Assert.True(aBox.IsOut(aSeparateLin), "Separate line should be out");
    }

    [Fact]
    public void Distance()
    {
        Box aBox1 = new();
        aBox1.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

        Box aBox2 = new();
        aBox2.Update(2.0, 0.0, 0.0, 3.0, 1.0, 1.0); // 1 unit away in X direction

        double aDistance = aBox1.Distance(aBox2);
        GTest.ExpectDoubleEq(1.0, aDistance);

        // Overlapping boxes
        Box anOverlappingBox = new();
        anOverlappingBox.Update(0.5, 0.5, 0.5, 1.5, 1.5, 1.5);

        double anOverlapDistance = aBox1.Distance(anOverlappingBox);
        GTest.ExpectDoubleEq(0.0, anOverlapDistance);
    }

    [Fact]
    public void OpenCloseMethods()
    {
        Box aBox = new();
        aBox.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

        Assert.False(aBox.IsOpen, "Initially box should not be open");

        aBox.OpenXmin();
        Assert.True(aBox.IsOpenXmin, "OpenXmin should make Xmin open");
        Assert.True(aBox.IsOpen, "Box with any open side should be considered open");

        aBox.OpenXmax();
        aBox.OpenYmin();
        aBox.OpenYmax();
        aBox.OpenZmin();
        aBox.OpenZmax();

        Assert.True(aBox.IsOpenXmax, "All directions should be openable");
        Assert.True(aBox.IsOpenYmin, "All directions should be openable");
        Assert.True(aBox.IsOpenYmax, "All directions should be openable");
        Assert.True(aBox.IsOpenZmin, "All directions should be openable");
        Assert.True(aBox.IsOpenZmax, "All directions should be openable");
        Assert.True(aBox.IsWhole, "Box open in all directions should be whole");
    }

    [Fact]
    public void VoidWholeStates()
    {
        Box aBox = new();

        Assert.True(aBox.IsVoid, "Default box should be void");
        Assert.False(aBox.IsWhole, "Default box should not be whole");

        aBox.SetWhole();
        Assert.False(aBox.IsVoid, "Whole box should not be void");
        Assert.True(aBox.IsWhole, "SetWhole should make box whole");

        aBox.SetVoid();
        Assert.True(aBox.IsVoid, "SetVoid should make box void");
        Assert.False(aBox.IsWhole, "Void box should not be whole");

        aBox.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        Assert.False(aBox.IsVoid, "Box with bounds should not be void");
        Assert.False(aBox.IsWhole, "Bounded box should not be whole");

        Assert.True(aBox.HasFinitePart, "Bounded box should have finite part");
    }

    [Fact]
    public void EdgeCases()
    {
        Box aBox = new();

        // Test with very large numbers
        double aLargeValue = 1e10;
        aBox.Update(-aLargeValue, -aLargeValue, -aLargeValue, aLargeValue, aLargeValue, aLargeValue);

        Assert.False(aBox.IsVoid, "Box with large values should be valid");

        // Test with very small differences
        Box aSmallBox = new();
        aSmallBox.Update(0.0, 0.0, 0.0, 1e-10, 1e-10, 1e-10);

        Assert.False(aSmallBox.IsVoid, "Box with tiny dimensions should be valid");
        Assert.True(aSmallBox.IsXThin(1e-9), "Very small box should be considered thin");
    }

    [Fact]
    public void TransformationWithOpenBox()
    {
        Box aBox = new();
        aBox.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        aBox.OpenXmax();

        Transform3D aRotation = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), M_PI / 2);

        Box aTransformed = aBox.Transformed(aRotation);

        Assert.True(aTransformed.IsOpen, "Transformed open box should remain open");
        Assert.False(aTransformed.IsVoid, "Transformed open box should have finite part");
    }

    [Fact]
    public void SquareExtent()
    {
        Box aBox = new();
        aBox.Update(0.0, 0.0, 0.0, 2.0, 3.0, 4.0);

        // SquareExtent appears to calculate the square of the diagonal length
        // For a box (0,0,0) to (2,3,4), diagonal = sqrt(2^2+3^2+4^2) = sqrt(29)
        // So SquareExtent = 29
        GTest.ExpectDoubleEq(29.0, aBox.SquareExtent());
    }

    [Fact]
    public void SetVoidAndWholeTransitions()
    {
        Box aBox = new();
        aBox.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

        Assert.False(aBox.IsVoid, "Box with bounds should not be void");

        aBox.SetVoid();
        Assert.True(aBox.IsVoid, "SetVoid should make box void");

        aBox.SetWhole();
        Assert.True(aBox.IsWhole, "SetWhole should make box whole");
        Assert.False(aBox.IsVoid, "Whole box should not be void");
    }

    [Fact]
    public void DirectOpenOperations()
    {
        Box aBox = new();
        aBox.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

        // Test individual open operations
        aBox.OpenXmin();
        Assert.True(aBox.IsOpenXmin, "OpenXmin should work");

        aBox.OpenXmax();
        Assert.True(aBox.IsOpenXmax, "OpenXmax should work");

        aBox.OpenYmin();
        Assert.True(aBox.IsOpenYmin, "OpenYmin should work");

        aBox.OpenYmax();
        Assert.True(aBox.IsOpenYmax, "OpenYmax should work");

        aBox.OpenZmin();
        Assert.True(aBox.IsOpenZmin, "OpenZmin should work");

        aBox.OpenZmax();
        Assert.True(aBox.IsOpenZmax, "OpenZmax should work");
    }

    [Fact]
    public void IsOutWithTransformation()
    {
        Box aBox1 = new();
        aBox1.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

        Box aBox2 = new();
        aBox2.Update(2.0, 0.0, 0.0, 3.0, 1.0, 1.0);

        Transform3D aTransf = new();
        Assert.True(aBox1.IsOut(aBox2, aTransf), "Separate boxes should be out");

        // Transform to overlap
        aTransf = aTransf.WithTranslation(new Vector3D(-1.5, 0.0, 0.0));
        Assert.False(aBox1.IsOut(aBox2, aTransf), "Overlapping transformed boxes should not be out");
    }

    [Fact]
    public void IsOutWithTwoTransformations()
    {
        Box aBox1 = new();
        aBox1.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

        Box aBox2 = new();
        aBox2.Update(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);

        Transform3D aT1 = new(), aT2 = new();
        aT1 = aT1.WithTranslation(new Vector3D(0.0, 0.0, 0.0));
        aT2 = aT2.WithTranslation(new Vector3D(2.0, 0.0, 0.0));

        Assert.True(aBox1.IsOut(aT1, aBox2, aT2), "Separated transformed boxes should be out");
    }

    [Fact]
    public void IsOutLineSegment()
    {
        Box aBox = new();
        aBox.Update(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);

        // Line segment that passes through box
        Point3D aP1 = new(-1.0, 1.0, 1.0);
        Point3D aP2 = new(3.0, 1.0, 1.0);
        Direction3D aDir = new(Direction3D.D.X);

        Assert.False(aBox.IsOut(aP1, aP2, aDir), "Line segment through box should not be out");

        // Line segment that misses box
        Point3D aP3 = new(-1.0, 3.0, 1.0);
        Point3D aP4 = new(3.0, 3.0, 1.0);

        Assert.True(aBox.IsOut(aP3, aP4, aDir), "Line segment missing box should be out");
    }

    [Fact]
    public void UpdateExpandsCorrectly()
    {
        Box aBox = new();

        // Start with a point
        aBox.Update(1.0, 1.0, 1.0);

        double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
        aBox.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        GTest.ExpectDoubleEq(1.0, aXmin);
        GTest.ExpectDoubleEq(1.0, aXmax);

        // Add another point that expands in all directions
        aBox.Update(-0.5, 2.5, 0.5);
        aBox.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        GTest.ExpectDoubleEq(-0.5, aXmin);
        GTest.ExpectDoubleEq(1.0, aXmax);
        GTest.ExpectDoubleEq(1.0, aYmin);
        GTest.ExpectDoubleEq(2.5, aYmax);
    }

    // DumpJsonAndInitFromJson is left out: DumpJson and InitFromJson are not ported (ToString replaces them).

    [Fact]
    public void OCC16485_CumulativeEnlargeTolerance()
    {
        // Create points with X coordinate varying from 0 to 1000
        // and compute cumulative bounding box by adding boxes for all the
        // points, enlarged on tolerance
        double aTol    = 1e-3;
        int    aNbStep = 1000;
        Box aBox = new();

        for (int i = 0; i <= aNbStep; i++)
        {
            Point3D aP = new(i, 0.0, 0.0);
            Box aB = new();
            aB.Add(aP);
            aB.Enlarge(aTol);
            aB.Add(aBox);
            aBox = aB; // This was causing XMin to grow each time (regression)
        }

        double aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
        aBox.Get(out aXmin, out aYmin, out aZmin, out aXmax, out aYmax, out aZmax);

        // Verify that Xmin is approximately -tolerance (not growing with iterations)
        Assert.Equal(aXmin, -aTol, 1e-10);
        Assert.Equal(aXmax, aNbStep + aTol, 1e-10);
    }

    [Fact]
    public void Contains_Point()
    {
        Box aBox = new(new Point3D(0, 0, 0), new Point3D(10, 10, 10));
        Assert.True(aBox.Contains(new Point3D(5, 5, 5)));
        Assert.True(aBox.Contains(new Point3D(0, 0, 0)));
        Assert.False(aBox.Contains(new Point3D(-1, 5, 5)));
        Assert.False(aBox.Contains(new Point3D(5, 5, 11)));
    }

    [Fact]
    public void Intersects_Box()
    {
        Box aBox1 = new(new Point3D(0, 0, 0), new Point3D(10, 10, 10));
        Box aBox2 = new(new Point3D(5, 5, 5), new Point3D(15, 15, 15));
        Box aBox3 = new(new Point3D(20, 20, 20), new Point3D(30, 30, 30));
        Assert.True(aBox1.Intersects(aBox2));
        Assert.False(aBox1.Intersects(aBox3));
    }

    [Fact]
    public void Center()
    {
        Box aBox = new(new Point3D(0, 0, 0), new Point3D(10, 20, 30));
        var    aCenter = aBox.Center();
        Assert.True(aCenter.HasValue);
        GTest.ExpectDoubleEq(aCenter.Value.X, 5.0);
        GTest.ExpectDoubleEq(aCenter.Value.Y, 10.0);
        GTest.ExpectDoubleEq(aCenter.Value.Z, 15.0);
    }

    [Fact]
    public void Center_VoidNullopt()
    {
        Box aVoid = new();
        Assert.False(aVoid.Center().HasValue);
    }
}
