// Port of: src/FoundationClasses/TKMath/GTests/Bnd_BoundSortBox_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace OcctNet.Foundation.Tests;

// Test fixture for Bnd_BoundSortBox tests (SetUp is the constructor: xUnit creates one instance per test)
public class Bnd_BoundSortBoxTest
{
    // Common test data
    private readonly Box mySmallBox;
    private readonly Box myLargeBox;
    private readonly Box myOffsetBox;
    private readonly Box myNonIntersectingBox;
    private readonly Box myGlobalBox;
    private readonly NCollection_HArray1<Box> myBoxes;

    public Bnd_BoundSortBoxTest()
    {
        // Setup common testing environment
        mySmallBox = CreateBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        myLargeBox = CreateBox(-10.0, -10.0, -10.0, 10.0, 10.0, 10.0);
        myOffsetBox = CreateBox(5.0, 5.0, 5.0, 7.0, 7.0, 7.0);
        myNonIntersectingBox = CreateBox(20.0, 20.0, 20.0, 30.0, 30.0, 30.0);

        // Create array of test boxes
        myBoxes = new NCollection_HArray1<Box>(1, 4);
        myBoxes.SetValue(1, mySmallBox);
        myBoxes.SetValue(2, myLargeBox);
        myBoxes.SetValue(3, myOffsetBox);
        myBoxes.SetValue(4, myNonIntersectingBox);

        // Create a global bounding box that contains all test boxes
        myGlobalBox = CreateBox(-20.0, -20.0, -20.0, 40.0, 40.0, 40.0);
    }

    internal static Box CreateBox(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax)
    {
        Box box = new();
        box.Update(xmin, ymin, zmin, xmax, ymax, zmax);
        return box;
    }

    [Fact]
    public void InitializeWithBoxes()
    {
        BoundSortBox sortBox = new();
        sortBox.Initialize(myBoxes);

        // Test comparing with a box that intersects mySmallBox
        Box testBox = CreateBox(0.5, 0.5, 0.5, 1.5, 1.5, 1.5);
        NCollection_List<int> result = sortBox.Compare(testBox);

        Assert.Equal(2, result.Extent());

        // Check that box indices 1 (mySmallBox) and 2 (myLargeBox) are in the result
        bool foundSmall = false;
        bool foundLarge = false;
        for (NCollection_List<int>.Iterator it = new(result); it.More(); it.Next())
        {
            if (it.Value() == 1)
            {
                foundSmall = true;
            }
            if (it.Value() == 2)
            {
                foundLarge = true;
            }
        }
        Assert.True(foundSmall, "Small box (index 1) should be in the result");
        Assert.True(foundLarge, "Large box (index 2) should be in the result");
    }

    [Fact]
    public void InitializeWithEnclosingBox()
    {
        BoundSortBox sortBox = new();
        sortBox.Initialize(myGlobalBox, myBoxes);

        // Test comparing with myOffsetBox
        NCollection_List<int> result = sortBox.Compare(myOffsetBox);

        Assert.Equal(2, result.Extent());

        // Check that box indices 2 (myLargeBox) and 3 (myOffsetBox) are in the result
        bool foundLarge  = false;
        bool foundOffset = false;
        for (NCollection_List<int>.Iterator it = new(result); it.More(); it.Next())
        {
            if (it.Value() == 2)
            {
                foundLarge = true;
            }
            if (it.Value() == 3)
            {
                foundOffset = true;
            }
        }
        Assert.True(foundLarge, "Large box (index 2) should be in the result");
        Assert.True(foundOffset, "Offset box (index 3) should be in the result");
    }

    [Fact]
    public void InitializeWithCount()
    {
        BoundSortBox sortBox = new();
        sortBox.Initialize(myGlobalBox, 3);

        // Add boxes manually
        sortBox.Add(mySmallBox, 1);
        sortBox.Add(myLargeBox, 2);
        sortBox.Add(myNonIntersectingBox, 3);

        // Test comparing with a box that should only intersect myLargeBox
        Box testBox = CreateBox(-5.0, -5.0, -5.0, -2.0, -2.0, -2.0);
        NCollection_List<int> result = sortBox.Compare(testBox);

        Assert.Equal(1, result.Extent());

        // Verify that only the large box was found
        Assert.Equal(2, result.First());
    }

    [Fact]
    public void CompareWithPlane()
    {
        BoundSortBox sortBox = new();
        sortBox.Initialize(myBoxes);

        // Create a plane that intersects the large box but not others
        Point3D point = new(0.0, 0.0, 9.0);
        Direction3D direction = new(Direction3D.D.Z);
        Plane3D plane = new(point, direction);

        NCollection_List<int> result = sortBox.Compare(plane);

        // Only the large box should intersect this plane
        Assert.Equal(1, result.Extent());
        Assert.Equal(2, result.First());
    }

    [Fact]
    public void VoidBoxes()
    {
        NCollection_HArray1<Box> boxes = new NCollection_HArray1<Box>(1, 2);
        Box void_box = new(); // Default constructed box is void
        boxes.SetValue(1, void_box);
        boxes.SetValue(2, mySmallBox);

        BoundSortBox sortBox = new();
        sortBox.Initialize(boxes);

        // Test comparing with mySmallBox - should only find itself
        NCollection_List<int> result = sortBox.Compare(mySmallBox);

        Assert.Equal(1, result.Extent());
        Assert.Equal(2, result.First());

        // Test comparing with void box - should find nothing
        NCollection_List<int> void_result = sortBox.Compare(void_box);
        Assert.Equal(0, void_result.Extent());
    }

    [Fact]
    public void TouchingBoxes()
    {
        // Create boxes that touch at a single point
        Box box1 = CreateBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        Box box2 = CreateBox(1.0, 1.0, 1.0, 2.0, 2.0, 2.0); // Touches box1 at (1,1,1)

        NCollection_HArray1<Box> boxes = new NCollection_HArray1<Box>(1, 2);
        boxes.SetValue(1, box1);
        boxes.SetValue(2, box2);

        BoundSortBox sortBox = new();
        sortBox.Initialize(boxes);

        // Test comparing with box1
        NCollection_List<int> result1 = sortBox.Compare(box1);
        Assert.Equal(2, result1.Extent());

        // Test comparing with box2
        NCollection_List<int> result2 = sortBox.Compare(box2);
        Assert.Equal(2, result2.Extent());
    }

    [Fact]
    public void DisjointBoxes()
    {
        Box farBox = CreateBox(100.0, 100.0, 100.0, 110.0, 110.0, 110.0);

        NCollection_HArray1<Box> boxes = new NCollection_HArray1<Box>(1, 2);
        boxes.SetValue(1, mySmallBox);
        boxes.SetValue(2, farBox);

        // Create enclosing box large enough
        Box enclosingBox = CreateBox(-10.0, -10.0, -10.0, 120.0, 120.0, 120.0);

        BoundSortBox sortBox = new();
        sortBox.Initialize(enclosingBox, boxes);

        // Test comparing with a box near mySmallBox
        Box testBox = CreateBox(0.5, 0.5, 0.5, 1.5, 1.5, 1.5);
        NCollection_List<int> result = sortBox.Compare(testBox);

        Assert.Equal(1, result.Extent());
        Assert.Equal(1, result.First());
    }

    [Fact]
    public void DegenerateBoxes()
    {
        // Create degenerate boxes
        Box pointBox = new();
        pointBox.Update(1.0, 1.0, 1.0, 1.0, 1.0, 1.0); // Point at (1,1,1)

        Box lineBox = new();
        lineBox.Update(2.0, 0.0, 0.0, 5.0, 0.0, 0.0); // Line along X from (2,0,0) to (5,0,0)

        Box planeBox = new();
        planeBox.Update(0.0, 0.0, 3.0, 3.0, 3.0, 3.0); // XY plane at Z=3

        NCollection_HArray1<Box> boxes = new NCollection_HArray1<Box>(1, 3);
        boxes.SetValue(1, pointBox);
        boxes.SetValue(2, lineBox);
        boxes.SetValue(3, planeBox);

        BoundSortBox sortBox = new();
        sortBox.Initialize(boxes);

        // Test with a box that should intersect all three degenerate boxes
        Box testBox = CreateBox(0.0, 0.0, 0.0, 6.0, 6.0, 6.0);
        NCollection_List<int> result = sortBox.Compare(testBox);

        Assert.Equal(3, result.Extent());

        // Test with a box that should intersect only the point
        Box pointTestBox = CreateBox(0.5, 0.5, 0.5, 1.5, 1.5, 1.5);
        NCollection_List<int> pointResult = sortBox.Compare(pointTestBox);

        Assert.Equal(1, pointResult.Extent());
        Assert.Equal(1, pointResult.First());
    }
}

// Bnd_BoundSortBox_Test.BUC60729_InitializeWithFaceBoxes bounds the faces of a box solid (BRepPrimAPI, BRepBndLib):
// it is in OcctNet.ModelingAlgorithms.Tests (Bnd/Bnd_ShapeGTests.cs).
