// Port of: src/FoundationClasses/TKMath/GTests/Bnd_BoundSortBox_Test.cxx
// Port of: src/FoundationClasses/TKMath/GTests/Bnd_OBB_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The cases of OCCT's Bnd_BoundSortBox and Bnd_OBB GTests that bound shapes (BRepPrimAPI_MakeBox and BRepBndLib):
/// the rest are in OcctNet.Foundation.Tests, which doesn't reference the topology.
/// </summary>
public class Bnd_BoundSortBox_Test
{
    // Test BUC60729: Initialize Bnd_BoundSortBox with face boxes from a solid (migrated from QABugs_3.cxx)
    [Fact]
    public void BUC60729_InitializeWithFaceBoxes()
    {
        // Create a simple box solid
        Shape aShape = new BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Solid();

        // Create main bounding box for the shape
        Box aMainBox = new();
        BRepBndLib.Add(aShape, ref aMainBox);

        // Initialize BoundSortBox with 6 boxes (for 6 faces of the cube)
        const int aMaxNbrBox = 6;
        BoundSortBox aBoundSortBox = new();
        aBoundSortBox.Initialize(aMainBox, aMaxNbrBox);

        // Iterate through faces and add their bounding boxes
        Explorer aExplorer = new(aShape, ShapeType.Face);
        int i;

        for (i = 1, aExplorer.ReInit(); aExplorer.More(); aExplorer.Next(), i++)
        {
            Shape aFace = aExplorer.Current();
            Box aBox = new();
            BRepBndLib.Add(aFace, ref aBox);
            aBoundSortBox.Add(aBox, i);
        }

        // Verify that 6 faces were processed
        Assert.Equal(7, i);
    }
}

public class Bnd_OBB_Test
{
    // Bug OCC30704: Oriented bounding box (Bnd_OBB) gives a wrong result if a box is added to a void box.
    [Fact]
    public void OCC30704_AddBoundingBoxToVoidBox()
    {
        // Make a shape somewhere far from (0, 0, 0)
        BRepPrimAPI_MakeBox aMkBox = new(new Point3D(100, 100, 100), 100, 100, 100);
        Shape aBox = aMkBox.Shape();

        // Add a bounding box of a shape to a void bounding box
        OrientedBox aVoidBox = new(), aOBB = new();
        BRepBndLib.AddOBB(aBox, ref aOBB, false, false, false);
        aVoidBox.Add(aOBB);

        // Check the center point of the bounding box
        Xyz aCenter = aVoidBox.Center();
        Assert.Equal(150.0, aCenter.X());
        Assert.Equal(150.0, aCenter.Y());
        Assert.Equal(150.0, aCenter.Z());
    }
}
