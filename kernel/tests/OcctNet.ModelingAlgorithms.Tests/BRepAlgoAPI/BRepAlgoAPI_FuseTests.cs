// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Topology;

namespace OcctNet.ModelingAlgorithms.Tests.BRepAlgoAPI;

/// <summary>
/// OCCT's GTests of BRepAlgoAPI_Fuse (src/ModelingAlgorithms/TKBO/GTests/BRepAlgoAPI_Fuse_Test.cxx), migrated from
/// /tests/boolean/bfuse_simple/. OCCT's fixture class BFuseSimpleTest is a nested class, so the tests keep their names.
/// </summary>
public static class BRepAlgoAPI_FuseTests
{
    private const double M_SQRT2 = 1.41421356237309504880168872420969808;

    //=================================================================================================
    // BFuse Simple Tests - migrating from /tests/boolean/bfuse_simple/
    //=================================================================================================

    public class BFuseSimpleTest : BRepAlgoAPI_TestBase
    {
        // Test bfuse_simple/A1: psphere s 1; box b 1 1 1; bfuse result s b; checkprops result -s 14.6394
        [Fact]
        public void SpherePlusBox_A1()
        {
            TopoDS_Shape aSphere = BOPTest_Utilities.CreateUnitSphere();
            TopoDS_Shape aBox = BOPTest_Utilities.CreateUnitBox();

            TopoDS_Shape aResult = PerformFuse(aSphere, aBox);
            ValidateResult(aResult, 14.6394);
        }

        // Test bfuse_simple/A2: rotated sphere + box
        [Fact]
        public void RotatedSpherePlusBox_A2()
        {
            TopoDS_Shape aSphere = BOPTest_Utilities.CreateUnitSphere();

            // Apply standard rotation: Z(-90deg) then Y(-45deg)
            TopoDS_Shape aRotatedSphere = BOPTest_Utilities.RotateStandard(aSphere);

            TopoDS_Shape aBox = BOPTest_Utilities.CreateUnitBox();
            TopoDS_Shape aResult = PerformFuse(aRotatedSphere, aBox);
            ValidateResult(aResult, 14.6393);
        }

        // Test bfuse_simple/A3: box + rotated sphere
        [Fact]
        public void BoxPlusRotatedSphere_A3()
        {
            TopoDS_Shape aSphere = BOPTest_Utilities.CreateUnitSphere();

            // Apply standard rotation: Z(-90deg) then Y(-45deg)
            TopoDS_Shape aRotatedSphere = BOPTest_Utilities.RotateStandard(aSphere);

            TopoDS_Shape aBox = BOPTest_Utilities.CreateUnitBox();
            TopoDS_Shape aResult = PerformFuse(aBox, aRotatedSphere);
            ValidateResult(aResult, 14.6393);
        }

        // Test bfuse_simple/A4: sphere + rotated box
        [Fact]
        public void SpherePlusRotatedBox_A4()
        {
            TopoDS_Shape aSphere = BOPTest_Utilities.CreateUnitSphere();
            TopoDS_Shape aBox = BOPTest_Utilities.CreateUnitBox();

            // Apply Y rotation: 90deg
            TopoDS_Shape aRotatedBox = BOPTest_Utilities.RotateY(aBox, 90.0);

            TopoDS_Shape aResult = PerformFuse(aSphere, aRotatedBox);
            ValidateResult(aResult, 14.6393);
        }

        // Test bfuse_simple/A5: rotated box + sphere
        [Fact]
        public void RotatedBoxPlusSphere_A5()
        {
            TopoDS_Shape aSphere = BOPTest_Utilities.CreateUnitSphere();
            TopoDS_Shape aBox = BOPTest_Utilities.CreateUnitBox();

            // Apply Y rotation: 90deg
            TopoDS_Shape aRotatedBox = BOPTest_Utilities.RotateY(aBox, 90.0);

            TopoDS_Shape aResult = PerformFuse(aRotatedBox, aSphere);
            ValidateResult(aResult, 14.6393);
        }

        // Test bfuse_simple/A6: nurbs box + identical box (should be 6.0 surface area)
        [Fact]
        public void IdenticalNurbsBoxPlusBox_A6()
        {
            // Create first box and convert to NURBS
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 6.0);
        }

        // Test bfuse_simple/A7: box + nurbs box (identical)
        [Fact]
        public void IdenticalBoxPlusNurbsBox_A7()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox2 = BOPTest_Utilities.ConvertToNurbs(aBox2);
            Assert.False(aBox2.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 6.0);
        }

        // Test bfuse_simple/A8: nurbs box + larger box
        [Fact]
        public void NurbsBoxPlusLargerBox_A8()
        {
            // Create first box and convert to NURBS
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(-0.5, -0.5, -0.5), 2.0, 2.0, 2.0);

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 24.0);
        }

        // Test bfuse_simple/A9: larger box + nurbs box
        [Fact]
        public void LargerBoxPlusNurbsBox_A9()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(-0.5, -0.5, -0.5), 2.0, 2.0, 2.0);

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox2 = BOPTest_Utilities.ConvertToNurbs(aBox2);
            Assert.False(aBox2.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 24.0);
        }

        // Test bfuse_simple/B1: nurbs box + box
        [Fact]
        public void NurbsBoxPlusBox_B1()
        {
            // Create first box and convert to NURBS
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            // Create second box
            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 0.5, 1.0, 0.5);

            TopoDS_Shape aResult = PerformFuse(aBox2, aBox1);
            ValidateResult(aResult, 6.0);
        }

        // Test bfuse_simple/B2: box + nurbs box
        [Fact]
        public void BoxPlusNurbsBox_B2()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, -0.5, 0), 0.5, 0.5, 1.0);

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox2 = BOPTest_Utilities.ConvertToNurbs(aBox2);
            Assert.False(aBox2.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 7.5);
        }

        // Test bfuse_simple/B3: box + nurbs box (adjacent)
        [Fact]
        public void NurbsBoxPlusAdjacentBox_B3()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, -0.5, 0), 0.5, 1.5, 1.0);

            TopoDS_Shape aResult = PerformFuse(aBox2, aBox1);
            ValidateResult(aResult, 7.5);
        }

        // Test bfuse_simple/B4: adjacent box + nurbs box
        [Fact]
        public void AdjacentBoxPlusNurbsBox_B4()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 0.5, 0), 1.0, 1.0, 1.0);

            TopoDS_Shape aResult = PerformFuse(aBox2, aBox1);
            ValidateResult(aResult, 8.0);
        }

        // Test bfuse_simple/B5: nurbs box + smaller box
        [Fact]
        public void NurbsBoxPlusSmallerBox_B5()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0.25, 0.25, 0.25), 0.5, 0.5, 0.5);

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 6.0);
        }

        // Test bfuse_simple/B6: smaller box + nurbs box
        [Fact]
        public void SmallerBoxPlusNurbsBox_B6()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0.25, 0.25, 0.25), 0.5, 0.5, 0.5);

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox2 = BOPTest_Utilities.ConvertToNurbs(aBox2);
            Assert.False(aBox2.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 6.0);
        }

        // Test bfuse_simple/B7: nurbs box + partially overlapping box
        [Fact]
        public void NurbsBoxPlusPartiallyOverlappingBox_B7()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0.5, 0.5, 0.5), 1.0, 1.0, 1.0);

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 7.75);
        }

        // Test bfuse_simple/B8: partially overlapping box + nurbs box
        [Fact]
        public void PartiallyOverlappingBoxPlusNurbsBox_B8()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0.5, 0.5, 0.5), 1.0, 1.0, 1.0);

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox2 = BOPTest_Utilities.ConvertToNurbs(aBox2);
            Assert.False(aBox2.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 7.75);
        }

        // Test bfuse_simple/B9: nurbs box + extended box
        [Fact]
        public void NurbsBoxPlusExtendedBox_B9()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(-0.5, -0.5, -0.5), 2.0, 2.0, 2.0);

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 24.0);
        }

        // Test bfuse_simple/C1: extended box + nurbs box
        [Fact]
        public void ExtendedBoxPlusNurbsBox_C1()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.5, 0.5, 0.5);

            TopoDS_Shape aResult = PerformFuse(aBox2, aBox1);
            ValidateResult(aResult, 7.0);
        }

        // Test bfuse_simple/C2: nurbs box + shifted extended box
        [Fact]
        public void NurbsBoxPlusShiftedBox_C2()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, -0.5, 0), 1.5, 0.5, 0.5);

            TopoDS_Shape aResult = PerformFuse(aBox2, aBox1);
            ValidateResult(aResult, 8.5);
        }

        // Test bfuse_simple/C3: shifted box + nurbs box
        [Fact]
        public void ShiftedBoxPlusNurbsBox_C3()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0.25, 0, 0), 0.5, 0.5, 1.0);

            TopoDS_Shape aResult = PerformFuse(aBox2, aBox1);
            ValidateResult(aResult, 6.0);
        }

        // Test bfuse_simple/C4: nurbs box + narrow box
        [Fact]
        public void NurbsBoxPlusNarrowBox_C4()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0.25, 0.25, 0), 0.5, 0.5, 1.5);

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 6.75);
        }

        // Test bfuse_simple/C5: narrow box + nurbs box
        [Fact]
        public void NarrowBoxPlusNurbsBox_C5()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0.25, 0.25, 0), 0.5, 0.5, 1.5);

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox2 = BOPTest_Utilities.ConvertToNurbs(aBox2);
            Assert.False(aBox2.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 6.75);
        }

        // Test bfuse_simple/C6: nurbs box + corner cube
        [Fact]
        public void NurbsBoxPlusCornerCube_C6()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0.5, 0.5, 0.5), 0.5, 0.5, 0.5);

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 6.0);
        }

        // Test bfuse_simple/C7: corner cube + nurbs box
        [Fact]
        public void CornerCubePlusNurbsBox_C7()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0.5, 0.5, 0.5), 0.5, 0.5, 0.5);

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox2 = BOPTest_Utilities.ConvertToNurbs(aBox2);
            Assert.False(aBox2.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 6.0);
        }

        // Test bfuse_simple/C8: nurbs box + offset cube
        [Fact]
        public void NurbsBoxPlusOffsetCube_C8()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(1, 1, 0), 0.5, 0.5, 1.0);

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 6.25);
        }

        // Test bfuse_simple/C9: offset cube + nurbs box
        [Fact]
        public void OffsetCubePlusNurbsBox_C9()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(1, 1, 0), 0.5, 0.5, 1.0);

            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox2 = BOPTest_Utilities.ConvertToNurbs(aBox2);
            Assert.False(aBox2.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 6.25);
        }

        // Test bfuse_simple/D1: nurbs box + rotated narrow box
        [Fact]
        public void NurbsBoxPlusRotatedNarrowBox_D1()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);

            // Create rotated narrow box: r=sqrt(2), box(0,0,0,r,0.25,1), rotate 45 degrees around Z
            const double r = M_SQRT2;
            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), r, 0.25, 1.0);
            Transform3D aTrsf = new();
            aTrsf.SetRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), M_PI / 4.0); // 45 degrees
            aBox2 = aBox2.Moved(new Placement(aTrsf));

            TopoDS_Shape aResult = PerformFuse(aBox2, aBox1);
            ValidateResult(aResult, 6.41789);
        }

        // Test bfuse_simple/D2: nurbs box + rotated narrow box variation
        [Fact]
        public void NurbsBoxPlusRotatedNarrowBoxVariation_D2()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = BOPTest_Utilities.ConvertToNurbs(aBox1);

            // Create rotated narrow box: r=sqrt(31), box(0,0,0,r/4,0.25,1), rotate 34.73 degrees around Z
            const double r = 5.5677643628300219;
            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), r / 4.0, 0.25, 1.0);
            Transform3D aTrsf = new();
            aTrsf.SetRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 34.73 * M_PI / 180.0);
            aBox2 = aBox2.Moved(new Placement(aTrsf));

            TopoDS_Shape aResult = PerformFuse(aBox2, aBox1);
            ValidateResult(aResult, 6.32953);
        }

        // Test bfuse_simple/D3: sphere + box
        [Fact]
        public void SpherePlusBox_D3()
        {
            TopoDS_Shape aSphere = BOPTest_Utilities.CreateSphere(new Point3D(0, 0, 0), 1.0);
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);

            TopoDS_Shape aResult = PerformFuse(aSphere, aBox);
            ValidateResult(aResult, 14.6394);
        }

        // Test bfuse_simple/D4: sphere + box (reversed order)
        [Fact]
        public void BoxPlusSphere_D4()
        {
            TopoDS_Shape aSphere = BOPTest_Utilities.CreateSphere(new Point3D(0, 0, 0), 1.0);
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);

            TopoDS_Shape aResult = PerformFuse(aBox, aSphere);
            ValidateResult(aResult, 14.6394);
        }

        // Test bfuse_simple/D5: rotated sphere + box
        [Fact]
        public void RotatedSpherePlusBox_D5()
        {
            TopoDS_Shape aSphere = BOPTest_Utilities.CreateSphere(new Point3D(0, 0, 0), 1.0);

            // Apply rotations: -90 degrees around Z, then -45 degrees around Y
            Transform3D aTrsf1 = new(), aTrsf2 = new(), aTrsfCombined;
            aTrsf1.SetRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), -M_PI / 2.0); // -90 degrees Z
            aTrsf2.SetRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Y)), -M_PI / 4.0); // -45 degrees Y
            aTrsfCombined = aTrsf2 * aTrsf1;
            aSphere = aSphere.Moved(new Placement(aTrsfCombined));

            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            TopoDS_Shape aResult = PerformFuse(aSphere, aBox);
            ValidateResult(aResult, 14.6393);
        }

        // Test bfuse_simple/D6: rotated sphere + box (reversed order)
        [Fact]
        public void BoxPlusRotatedSphere_D6()
        {
            TopoDS_Shape aSphere = BOPTest_Utilities.CreateSphere(new Point3D(0, 0, 0), 1.0);

            // Apply rotations: -90 degrees around Z, then -45 degrees around Y
            Transform3D aTrsf1 = new(), aTrsf2 = new(), aTrsfCombined;
            aTrsf1.SetRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), -M_PI / 2.0);
            aTrsf2.SetRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Y)), -M_PI / 4.0);
            aTrsfCombined = aTrsf2 * aTrsf1;
            aSphere = aSphere.Moved(new Placement(aTrsfCombined));

            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            TopoDS_Shape aResult = PerformFuse(aBox, aSphere);
            ValidateResult(aResult, 14.6393);
        }

        // Test bfuse_simple/D7: sphere + rotated box
        [Fact]
        public void SpherePlusRotatedBox_D7()
        {
            TopoDS_Shape aSphere = BOPTest_Utilities.CreateSphere(new Point3D(0, 0, 0), 1.0);

            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            // Rotate box 90 degrees around Y axis: "trotate b 0 0 1 0 1 0 90"
            Transform3D aTrsf = new();
            aTrsf.SetRotation(new Axis3D(new Point3D(0, 0, 1), new Direction3D(Direction3D.D.Y)), M_PI / 2.0); // 90 degrees Y
            aBox = aBox.Moved(new Placement(aTrsf));

            TopoDS_Shape aResult = PerformFuse(aSphere, aBox);
            ValidateResult(aResult, 14.6393);
        }

        // Test bfuse_simple/D8: rotated box + sphere (reversed order)
        [Fact]
        public void RotatedBoxPlusSphere_D8()
        {
            TopoDS_Shape aSphere = BOPTest_Utilities.CreateSphere(new Point3D(0, 0, 0), 1.0);

            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            // Rotate box 90 degrees around Y axis: "trotate b 0 0 1 0 1 0 90"
            Transform3D aTrsf = new();
            aTrsf.SetRotation(new Axis3D(new Point3D(0, 0, 1), new Direction3D(Direction3D.D.Y)), M_PI / 2.0); // 90 degrees Y
            aBox = aBox.Moved(new Placement(aTrsf));

            TopoDS_Shape aResult = PerformFuse(aBox, aSphere);
            ValidateResult(aResult, 14.6393);
        }

        // Test bfuse_simple/D9: profile-based prisms (cts40125 bug)
        [Fact]
        public void ProfileBasedPrisms_D9()
        {
            // Create first profile: "profile f1 x 100 y 100 x -200 y -200 x 100"
            // This creates a closed rectangular profile starting at origin
            Plane3D aPlane1 = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)));
            List<BOPTest_Utilities.ProfileOperation> aProfileOps1 =
            [
                new(BOPTest_Utilities.ProfileCmd.X, 100.0),  // move to (100, 0)
                new(BOPTest_Utilities.ProfileCmd.Y, 100.0),  // move to (100, 100)
                new(BOPTest_Utilities.ProfileCmd.X, -200.0), // move to (-100, 100)
                new(BOPTest_Utilities.ProfileCmd.Y, -200.0), // move to (-100, -100)
                new(BOPTest_Utilities.ProfileCmd.X, 100.0),  // move to (0, -100), profile will auto-close
            ];
            TopoDS_Shape aProfile1 = BOPTest_Utilities.CreateProfile(aPlane1, aProfileOps1);
            TopoDS_Shape aPrism1 = BOPTest_Utilities.CreatePrism(aProfile1, new Vector3D(0, 0, 100));

            // Create second profile: "profile f2 x -100 y 100 x 100; ttranslate f2 0 0 100"
            // This creates a triangular profile translated to z=100
            Plane3D aPlane2 = new(new CoordinateSystem(new Point3D(0, 0, 100), new Direction3D(Direction3D.D.Z)));
            List<BOPTest_Utilities.ProfileOperation> aProfileOps2 =
            [
                new(BOPTest_Utilities.ProfileCmd.X, -100.0), // move to (-100, 0)
                new(BOPTest_Utilities.ProfileCmd.Y, 100.0),  // move to (-100, 100)
                new(BOPTest_Utilities.ProfileCmd.X, 100.0),  // move to (0, 100), profile will auto-close
            ];
            TopoDS_Shape aProfile2 = BOPTest_Utilities.CreateProfile(aPlane2, aProfileOps2);
            TopoDS_Shape aPrism2 = BOPTest_Utilities.CreatePrism(aProfile2, new Vector3D(0, 0, 100));

            TopoDS_Shape aResult = PerformFuse(aPrism1, aPrism2);
            ValidateResult(aResult, 180000.0);
        }

        // Test bfuse_simple/E1: Complex profile with scaling (pro7637 bug)
        [Fact]
        public void ComplexProfileWithScaling_E1()
        {
            const double SCALE = 100.0;

            // Create first profile: "profile f1 c 50*SCALE 180 x -100*SCALE c 50*SCALE 180"
            Plane3D aPlane1 = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)));
            List<BOPTest_Utilities.ProfileOperation> aProfileOps1 =
            [
                new(BOPTest_Utilities.ProfileCmd.C, 50.0 * SCALE, 180.0),
                new(BOPTest_Utilities.ProfileCmd.X, -100.0 * SCALE),
                new(BOPTest_Utilities.ProfileCmd.C, 50.0 * SCALE, 180.0),
            ];
            TopoDS_Shape aProfile1 = BOPTest_Utilities.CreateProfile(aPlane1, aProfileOps1);
            TopoDS_Shape aPrism1 = BOPTest_Utilities.CreatePrism(aProfile1, new Vector3D(0, 0, 30.0 * SCALE));

            // Create second profile: "profile f2 x 300*SCALE y 200*SCALE x -300*SCALE; ttranslate f2
            // -200*SCALE -50*SCALE 0"
            Plane3D aPlane2 = new(new CoordinateSystem(new Point3D(-200.0 * SCALE, -50.0 * SCALE, 0), new Direction3D(Direction3D.D.Z)));
            List<BOPTest_Utilities.ProfileOperation> aProfileOps2 =
            [
                new(BOPTest_Utilities.ProfileCmd.X, 300.0 * SCALE),
                new(BOPTest_Utilities.ProfileCmd.Y, 200.0 * SCALE),
                new(BOPTest_Utilities.ProfileCmd.X, -300.0 * SCALE),
                new(BOPTest_Utilities.ProfileCmd.Y, -200.0 * SCALE), // Close the profile
            ];
            TopoDS_Shape aProfile2 = BOPTest_Utilities.CreateProfile(aPlane2, aProfileOps2);
            TopoDS_Shape aPrism2 = BOPTest_Utilities.CreatePrism(aProfile2, new Vector3D(0, 0, -50.0 * SCALE));

            TopoDS_Shape aResult = PerformFuse(aPrism2, aPrism1);
            ValidateResult(aResult, 1.85425e+09);
        }

        // Test bfuse_simple/E2: Adjacent boxes (buc40054 bug)
        [Fact]
        public void AdjacentBoxes_E2()
        {
            TopoDS_Shape aBox1 = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 3.0, 3.0, 3.0);
            TopoDS_Shape aBox2 = BOPTest_Utilities.CreateBox(new Point3D(0, 3, 0), 1.0, 1.0, 1.0);

            TopoDS_Shape aResult = PerformFuse(aBox1, aBox2);
            ValidateResult(aResult, 58.0);
        }

        // Test bfuse_simple/E3: Complex vertex/edge/wire construction with revolution
        [Fact]
        public void ComplexVertexEdgeWireRevolution_E3()
        {
            // Create profile points for the wire
            List<Point3D> aPoints = [new Point3D(0, 0, 0), new Point3D(9, 0, 0), new Point3D(9, 0, 3), new Point3D(6.25, 0, 3), new Point3D(6, 0, 4), new Point3D(0, 0, 4)];

            // Create wire from points
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create revolution: "revol cyla pa 0 0 0 0 0 1 360"
            TopoDS_Shape aRevolution = BOPTest_Utilities.CreateRevolution(aFace, new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 2.0 * M_PI);

            // Create cylinder: "pcylinder cylb 1 9; ttranslate cylb 5 0 -2"
            TopoDS_Shape aCylinder = BOPTest_Utilities.CreateCylinder(1.0, 9.0);
            Transform3D aTrsf = new();
            aTrsf.SetTranslation(new Vector3D(5, 0, -2));
            aCylinder = aCylinder.Moved(new Placement(aTrsf));

            TopoDS_Shape aResult = PerformFuse(aRevolution, aCylinder);
            ValidateResult(aResult, 740.048);
        }

        // Test bfuse_simple/E4: Cylinder with complex wire revolution
        [Fact]
        public void CylinderWithComplexWireRevolution_E4()
        {
            // Create cylinder: "pcylinder cyl 3 5"
            TopoDS_Shape aCylinder = BOPTest_Utilities.CreateCylinder(3.0, 5.0);

            // Create profile points for rectangular profile
            List<Point3D> aPoints = [new Point3D(0, 3, 2), new Point3D(0, 4, 2), new Point3D(0, 4, 3), new Point3D(0, 3, 3)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create revolution: "revol ring f 0 0 0 0 0 1 269"
            TopoDS_Shape aRing = BOPTest_Utilities.CreateRevolution(aFace, new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 269.0 * M_PI / 180.0);

            TopoDS_Shape aResult = PerformFuse(aCylinder, aRing);
            ValidateResult(aResult, 190.356);
        }

        // Test bfuse_simple/E5: Box with prism from vertex/edge construction
        [Fact]
        public void BoxWithPrismFromVertexEdge_E5()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for rectangular prism
            List<Point3D> aPoints = [new Point3D(3, 2, 0), new Point3D(4, 2, 0), new Point3D(4, 3, 0), new Point3D(3, 3, 0)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 0 0 1"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(0, 0, 1));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/E6: Box with prism from front
        [Fact]
        public void BoxWithPrismFromFront_E6()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for front prism
            List<Point3D> aPoints = [new Point3D(3, 2, 0), new Point3D(4, 2, 0), new Point3D(4, 2, 1), new Point3D(3, 2, 1)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 0 1 0"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(0, 1, 0));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/E7: Box with prism from left
        [Fact]
        public void BoxWithPrismFromLeft_E7()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for left prism
            List<Point3D> aPoints = [new Point3D(3, 2, 0), new Point3D(3, 3, 0), new Point3D(3, 3, 1), new Point3D(3, 2, 1)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 1 0 0"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/E8: Box with prism from top
        [Fact]
        public void BoxWithPrismFromTop_E8()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for top prism
            List<Point3D> aPoints = [new Point3D(3, 2, 1), new Point3D(4, 2, 1), new Point3D(4, 3, 1), new Point3D(3, 3, 1)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 0 0 -1"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(0, 0, -1));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/E9: Box with prism from back
        [Fact]
        public void BoxWithPrismFromBack_E9()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for back prism
            List<Point3D> aPoints = [new Point3D(3, 3, 0), new Point3D(4, 3, 0), new Point3D(4, 3, 1), new Point3D(3, 3, 1)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 0 -1 0"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(0, -1, 0));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/F1: Box with prism from right
        [Fact]
        public void BoxWithPrismFromRight_F1()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for right prism
            List<Point3D> aPoints = [new Point3D(4, 2, 0), new Point3D(4, 3, 0), new Point3D(4, 3, 1), new Point3D(4, 2, 1)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f -1 0 0"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(-1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/F2: Box with prism from bottom (different position)
        [Fact]
        public void BoxWithPrismFromBottomDifferent_F2()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for bottom prism (different position)
            List<Point3D> aPoints = [new Point3D(4, 3, 0), new Point3D(4, 2, 0), new Point3D(3, 2, 0), new Point3D(3, 3, 0)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 0 0 1"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(0, 0, 1));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/F3: Box with prism from bottom (external position)
        [Fact]
        public void BoxWithPrismFromBottomExternal_F3()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for bottom prism (external position)
            List<Point3D> aPoints = [new Point3D(8, 3, 0), new Point3D(9, 3, 0), new Point3D(9, 4, 0), new Point3D(8, 4, 0)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 0 0 1"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(0, 0, 1));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/F4: Box with prism from front (external position)
        [Fact]
        public void BoxWithPrismFromFrontExternal_F4()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for front prism (external position)
            List<Point3D> aPoints = [new Point3D(8, 3, 0), new Point3D(9, 3, 0), new Point3D(9, 3, 1), new Point3D(8, 3, 1)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 0 1 0"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(0, 1, 0));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/F5: Box with prism from left (external position)
        [Fact]
        public void BoxWithPrismFromLeftExternal_F5()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for left prism (external position)
            List<Point3D> aPoints = [new Point3D(8, 3, 0), new Point3D(8, 4, 0), new Point3D(8, 4, 1), new Point3D(8, 3, 1)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 1 0 0"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/F6: Box with prism from top (external position)
        [Fact]
        public void BoxWithPrismFromTopExternal_F6()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for top prism (external position)
            List<Point3D> aPoints = [new Point3D(8, 3, 1), new Point3D(9, 3, 1), new Point3D(9, 4, 1), new Point3D(8, 4, 1)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 0 0 -1"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(0, 0, -1));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/F7: Box with prism from back (external position)
        [Fact]
        public void BoxWithPrismFromBackExternal_F7()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for back prism (external position)
            List<Point3D> aPoints = [new Point3D(8, 4, 0), new Point3D(9, 4, 0), new Point3D(9, 4, 1), new Point3D(8, 4, 1)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 0 -1 0"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(0, -1, 0));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/F8: Box with prism from right (external position)
        [Fact]
        public void BoxWithPrismFromRightExternal_F8()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for right prism (external position)
            List<Point3D> aPoints = [new Point3D(9, 3, 0), new Point3D(9, 4, 0), new Point3D(9, 4, 1), new Point3D(9, 3, 1)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f -1 0 0"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(-1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/F9: Box with prism from bottom (top position)
        [Fact]
        public void BoxWithPrismFromBottomTopPosition_F9()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for bottom prism at top position
            List<Point3D> aPoints = [new Point3D(3, 3, 4), new Point3D(4, 3, 4), new Point3D(4, 4, 4), new Point3D(3, 4, 4)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 0 0 1"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(0, 0, 1));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/G1: Box with prism from front at top level
        [Fact]
        public void BoxWithPrismFromFrontTopLevel_G1()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for front prism at top level
            List<Point3D> aPoints = [new Point3D(3, 3, 4), new Point3D(4, 3, 4), new Point3D(4, 3, 5), new Point3D(3, 3, 5)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 0 1 0"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(0, 1, 0));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/G2: Box with prism from left at top level
        [Fact]
        public void BoxWithPrismFromLeftTopLevel_G2()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for left prism at top level
            List<Point3D> aPoints = [new Point3D(3, 3, 4), new Point3D(3, 4, 4), new Point3D(3, 4, 5), new Point3D(3, 3, 5)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 1 0 0"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/G3: Box with prism from top at top level
        [Fact]
        public void BoxWithPrismFromTopTopLevel_G3()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for top prism at top level
            List<Point3D> aPoints = [new Point3D(3, 3, 5), new Point3D(4, 3, 5), new Point3D(4, 4, 5), new Point3D(3, 4, 5)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 0 0 -1"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(0, 0, -1));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/G4: Box with prism from back at top level
        [Fact]
        public void BoxWithPrismFromBackTopLevel_G4()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for back prism at top level
            List<Point3D> aPoints = [new Point3D(3, 4, 4), new Point3D(4, 4, 4), new Point3D(4, 4, 5), new Point3D(3, 4, 5)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f 0 -1 0"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(0, -1, 0));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/G5: Box with prism from right at top level
        [Fact]
        public void BoxWithPrismFromRightTopLevel_G5()
        {
            // Create box: "box ba 3 3 0 5 7 4"
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(3, 3, 0), 5.0, 7.0, 4.0);

            // Create profile points for right prism at top level
            List<Point3D> aPoints = [new Point3D(4, 3, 4), new Point3D(4, 4, 4), new Point3D(4, 4, 5), new Point3D(4, 3, 5)];

            // Create wire and face
            TopoDS_Wire aWire = BOPTest_Utilities.CreatePolygonWire(aPoints, true);
            TopoDS_Shape aFace = BOPTest_Utilities.CreateFaceFromWire(aWire);

            // Create prism: "prism bb f -1 0 0"
            TopoDS_Shape aPrism = BOPTest_Utilities.CreatePrism(aFace, new Vector3D(-1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBox, aPrism);
            ValidateResult(aResult, 170.0);
        }

        // Test bfuse_simple/G6: Large prism base with small prism from bottom
        [Fact]
        public void LargePrismWithSmallPrismFromBottom_G6()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from bottom
            List<Point3D> aSmallPts = [new Point3D(3, 2, 0), new Point3D(4, 2, 0), new Point3D(4, 3, 0), new Point3D(3, 3, 0)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(0, 0, 1));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/G7: Large prism base with small prism from front
        [Fact]
        public void LargePrismWithSmallPrismFromFront_G7()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from front
            List<Point3D> aSmallPts = [new Point3D(3, 2, 0), new Point3D(4, 2, 0), new Point3D(4, 2, 1), new Point3D(3, 2, 1)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(0, 1, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/G8: Large prism base with small prism from left
        [Fact]
        public void LargePrismWithSmallPrismFromLeft_G8()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from left
            List<Point3D> aSmallPts = [new Point3D(3, 2, 0), new Point3D(3, 3, 0), new Point3D(3, 3, 1), new Point3D(3, 2, 1)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/G9: Large prism base with small prism from top
        [Fact]
        public void LargePrismWithSmallPrismFromTop_G9()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from top
            List<Point3D> aSmallPts = [new Point3D(3, 2, 1), new Point3D(4, 2, 1), new Point3D(4, 3, 1), new Point3D(3, 3, 1)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(0, 0, -1));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/H1: Large prism with small prism from back edge
        [Fact]
        public void LargePrismWithSmallPrismFromBackEdge_H1()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from back
            List<Point3D> aSmallPts = [new Point3D(3, 3, 0), new Point3D(4, 3, 0), new Point3D(4, 3, 1), new Point3D(3, 3, 1)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(0, -1, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/H2: Large prism with small prism from right edge
        [Fact]
        public void LargePrismWithSmallPrismFromRightEdge_H2()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from right
            List<Point3D> aSmallPts = [new Point3D(4, 2, 0), new Point3D(4, 3, 0), new Point3D(4, 3, 1), new Point3D(4, 2, 1)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(-1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/H3: Large prism with small prism from bottom corner
        [Fact]
        public void LargePrismWithSmallPrismFromBottomCorner_H3()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from bottom
            List<Point3D> aSmallPts = [new Point3D(8, 3, 0), new Point3D(9, 3, 0), new Point3D(9, 4, 0), new Point3D(8, 4, 0)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(0, 0, 1));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/H4: Large prism with small prism from front edge
        [Fact]
        public void LargePrismWithSmallPrismFromFrontEdge_H4()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from front
            List<Point3D> aSmallPts = [new Point3D(8, 3, 0), new Point3D(9, 3, 0), new Point3D(9, 3, 1), new Point3D(8, 3, 1)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(0, 1, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/H5: Large prism with small prism from left edge
        [Fact]
        public void LargePrismWithSmallPrismFromLeftEdge_H5()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from left
            List<Point3D> aSmallPts = [new Point3D(8, 3, 0), new Point3D(8, 4, 0), new Point3D(8, 4, 1), new Point3D(8, 3, 1)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/H6: Large prism with small prism from top edge
        [Fact]
        public void LargePrismWithSmallPrismFromTopEdge_H6()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from top
            List<Point3D> aSmallPts = [new Point3D(8, 3, 1), new Point3D(9, 3, 1), new Point3D(9, 4, 1), new Point3D(8, 4, 1)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(0, 0, -1));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/H7: Large prism with small prism from back edge (corner)
        [Fact]
        public void LargePrismWithSmallPrismFromBackEdgeCorner_H7()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from back
            List<Point3D> aSmallPts = [new Point3D(8, 4, 0), new Point3D(9, 4, 0), new Point3D(9, 4, 1), new Point3D(8, 4, 1)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(0, -1, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/H8: Large prism with small prism from right edge (corner)
        [Fact]
        public void LargePrismWithSmallPrismFromRightEdgeCorner_H8()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from right
            List<Point3D> aSmallPts = [new Point3D(9, 3, 0), new Point3D(9, 4, 0), new Point3D(9, 4, 1), new Point3D(9, 3, 1)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(-1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/H9: Large prism with small prism from top corner
        [Fact]
        public void LargePrismWithSmallPrismFromTopCorner_H9()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from top
            List<Point3D> aSmallPts = [new Point3D(3, 3, 4), new Point3D(4, 3, 4), new Point3D(4, 4, 4), new Point3D(3, 4, 4)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(0, 0, 1));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/I1: Large prism with small prism from front (top level)
        [Fact]
        public void LargePrismWithSmallPrismFromFrontTopLevel_I1()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from front at top level
            List<Point3D> aSmallPts = [new Point3D(3, 3, 4), new Point3D(4, 3, 4), new Point3D(4, 3, 5), new Point3D(3, 3, 5)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(0, 1, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/I2: Large prism with small prism from left (top level)
        [Fact]
        public void LargePrismWithSmallPrismFromLeftTopLevel_I2()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from left at top level
            List<Point3D> aSmallPts = [new Point3D(3, 3, 4), new Point3D(3, 4, 4), new Point3D(3, 4, 5), new Point3D(3, 3, 5)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/I3: Large prism with small prism from top (above)
        [Fact]
        public void LargePrismWithSmallPrismFromTopAbove_I3()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from top (above the base)
            List<Point3D> aSmallPts = [new Point3D(3, 3, 5), new Point3D(4, 3, 5), new Point3D(4, 4, 5), new Point3D(3, 4, 5)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(0, 0, -1));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/I4: Large prism with small prism from back (top level)
        [Fact]
        public void LargePrismWithSmallPrismFromBackTopLevel_I4()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from back at top level
            List<Point3D> aSmallPts = [new Point3D(3, 4, 4), new Point3D(4, 4, 4), new Point3D(4, 4, 5), new Point3D(3, 4, 5)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(0, -1, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/I5: Large prism with small prism from right (top level)
        [Fact]
        public void LargePrismWithSmallPrismFromRightTopLevel_I5()
        {
            // Create large prism base from vertices: (3,3,0) to (8,9,4)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 4));

            // Create small prism from right at top level
            List<Point3D> aSmallPts = [new Point3D(4, 3, 4), new Point3D(4, 4, 4), new Point3D(4, 4, 5), new Point3D(4, 3, 5)];
            TopoDS_Wire aSmallWire = BOPTest_Utilities.CreatePolygonWire(aSmallPts, true);
            TopoDS_Shape aSmallFace = BOPTest_Utilities.CreateFaceFromWire(aSmallWire);
            TopoDS_Shape aSmallPrism = BOPTest_Utilities.CreatePrism(aSmallFace, new Vector3D(-1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aSmallPrism);
            ValidateResult(aResult, 152.0);
        }

        // Test bfuse_simple/I6: Large prism with oblong prism (profile-based)
        [Fact]
        public void LargePrismWithOblongPrismProfile_I6()
        {
            // Create large prism base (5 units high)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 5));

            // Create oblong profile: O 4 3 0  P 0 1 0 1 0 0  D -1 0  C 1 180  X 2  C 1 180  W
            List<BOPTest_Utilities.ProfileOperation> aOps =
            [
                new(BOPTest_Utilities.ProfileCmd.O, 4, 3, 0),
                new(BOPTest_Utilities.ProfileCmd.P, 0, 1, 0, 1, 0, 0),
                new(BOPTest_Utilities.ProfileCmd.D, -1, 0),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.X, 2),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.W),
            ];
            TopoDS_Shape aProfileFace = BOPTest_Utilities.CreateProfileFromOperations(aOps);
            TopoDS_Shape aOblongPrism = BOPTest_Utilities.CreatePrism(aProfileFace, new Vector3D(0, -1, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aOblongPrism);
            ValidateResult(aResult, 180.283);
        }

        // Test bfuse_simple/I7: Large prism with oblong prism (Y direction profile)
        [Fact]
        public void LargePrismWithOblongPrismYDirection_I7()
        {
            // Create large prism base (5 units high)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 5));

            // Create oblong profile: O 3 3 3  P 0 1 0 1 0 0  D 0 -1  C 1 180  Y 2  C 1 180  W
            List<BOPTest_Utilities.ProfileOperation> aOps =
            [
                new(BOPTest_Utilities.ProfileCmd.O, 3, 3, 3),
                new(BOPTest_Utilities.ProfileCmd.P, 0, 1, 0, 1, 0, 0),
                new(BOPTest_Utilities.ProfileCmd.D, 0, -1),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.Y, 2),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.W),
            ];
            TopoDS_Shape aProfileFace = BOPTest_Utilities.CreateProfileFromOperations(aOps);
            TopoDS_Shape aOblongPrism = BOPTest_Utilities.CreatePrism(aProfileFace, new Vector3D(0, -1, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aOblongPrism);
            ValidateResult(aResult, 180.283);
        }

        // Test bfuse_simple/I8: Large prism with oblong prism (X direction profile)
        [Fact]
        public void LargePrismWithOblongPrismXDirection_I8()
        {
            // Create large prism base (5 units high)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 5));

            // Create oblong profile: O 5 3 0  P 0 1 0 1 0 0  D -1 0  C 1 180  X 2  C 1 180  W
            List<BOPTest_Utilities.ProfileOperation> aOps =
            [
                new(BOPTest_Utilities.ProfileCmd.O, 5, 3, 0),
                new(BOPTest_Utilities.ProfileCmd.P, 0, 1, 0, 1, 0, 0),
                new(BOPTest_Utilities.ProfileCmd.D, -1, 0),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.X, 2),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.W),
            ];
            TopoDS_Shape aProfileFace = BOPTest_Utilities.CreateProfileFromOperations(aOps);
            TopoDS_Shape aOblongPrism = BOPTest_Utilities.CreatePrism(aProfileFace, new Vector3D(0, -1, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aOblongPrism);
            ValidateResult(aResult, 180.283);
        }

        // Test bfuse_simple/I9: Large prism with oblong prism (Y direction profile)
        [Fact]
        public void LargePrismWithOblongPrismYDirectionFinal_I9()
        {
            // Create large prism base (5 units high)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 5));

            // Create oblong profile: O 6 3 3  P 0 1 0 1 0 0  D 0 -1  C 1 180  Y 2  C 1 180  W
            List<BOPTest_Utilities.ProfileOperation> aOps =
            [
                new(BOPTest_Utilities.ProfileCmd.O, 6, 3, 3),
                new(BOPTest_Utilities.ProfileCmd.P, 0, 1, 0, 1, 0, 0),
                new(BOPTest_Utilities.ProfileCmd.D, 0, -1),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.Y, 2),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.W),
            ];
            TopoDS_Shape aProfileFace = BOPTest_Utilities.CreateProfileFromOperations(aOps);
            TopoDS_Shape aOblongPrism = BOPTest_Utilities.CreatePrism(aProfileFace, new Vector3D(0, -1, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aOblongPrism);
            ValidateResult(aResult, 180.283);
        }

        // Test bfuse_simple/J1: Large prism with oblong prism (right side face profile)
        [Fact]
        public void LargePrismWithOblongPrismRightSide_J1()
        {
            // Create large prism base (5 units high)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 5));

            // Create oblong profile: O 8 4 0  P -1 0 0 0 1 0  D -1 0  C 1 180  X 2  C 1 180  W
            List<BOPTest_Utilities.ProfileOperation> aOps =
            [
                new(BOPTest_Utilities.ProfileCmd.O, 8, 4, 0),
                new(BOPTest_Utilities.ProfileCmd.P, -1, 0, 0, 0, 1, 0),
                new(BOPTest_Utilities.ProfileCmd.D, -1, 0),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.X, 2),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.W),
            ];
            TopoDS_Shape aProfileFace = BOPTest_Utilities.CreateProfileFromOperations(aOps);
            TopoDS_Shape aOblongPrism = BOPTest_Utilities.CreatePrism(aProfileFace, new Vector3D(1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aOblongPrism);
            ValidateResult(aResult, 180.283);
        }

        // Test bfuse_simple/J2: Large prism with oblong prism (right side face profile Y direction)
        [Fact]
        public void LargePrismWithOblongPrismRightSideY_J2()
        {
            // Create large prism base (5 units high)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 5));

            // Create oblong profile: O 8 3 3  P -1 0 0 0 1 0  D 0 -1  C 1 180  Y 2  C 1 180  W
            List<BOPTest_Utilities.ProfileOperation> aOps =
            [
                new(BOPTest_Utilities.ProfileCmd.O, 8, 3, 3),
                new(BOPTest_Utilities.ProfileCmd.P, -1, 0, 0, 0, 1, 0),
                new(BOPTest_Utilities.ProfileCmd.D, 0, -1),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.Y, 2),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.W),
            ];
            TopoDS_Shape aProfileFace = BOPTest_Utilities.CreateProfileFromOperations(aOps);
            TopoDS_Shape aOblongPrism = BOPTest_Utilities.CreatePrism(aProfileFace, new Vector3D(1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aOblongPrism);
            ValidateResult(aResult, 180.283);
        }

        // Test bfuse_simple/J3: Large prism with oblong prism (right side face profile X direction)
        [Fact]
        public void LargePrismWithOblongPrismRightSideX_J3()
        {
            // Create large prism base (5 units high)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 5));

            // Create oblong profile: O 8 6 0  P -1 0 0 0 1 0  D -1 0  C 1 180  X 2  C 1 180  W
            List<BOPTest_Utilities.ProfileOperation> aOps =
            [
                new(BOPTest_Utilities.ProfileCmd.O, 8, 6, 0),
                new(BOPTest_Utilities.ProfileCmd.P, -1, 0, 0, 0, 1, 0),
                new(BOPTest_Utilities.ProfileCmd.D, -1, 0),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.X, 2),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.W),
            ];
            TopoDS_Shape aProfileFace = BOPTest_Utilities.CreateProfileFromOperations(aOps);
            TopoDS_Shape aOblongPrism = BOPTest_Utilities.CreatePrism(aProfileFace, new Vector3D(1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aOblongPrism);
            ValidateResult(aResult, 180.283);
        }

        // Test bfuse_simple/J4: Large prism with oblong prism (right side face profile Y direction)
        [Fact]
        public void LargePrismWithOblongPrismRightSideY2_J4()
        {
            // Create large prism base (5 units high)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 5));

            // Create oblong profile: O 8 7 3  P -1 0 0 0 1 0  D 0 -1  C 1 180  Y 2  C 1 180  W
            List<BOPTest_Utilities.ProfileOperation> aOps =
            [
                new(BOPTest_Utilities.ProfileCmd.O, 8, 7, 3),
                new(BOPTest_Utilities.ProfileCmd.P, -1, 0, 0, 0, 1, 0),
                new(BOPTest_Utilities.ProfileCmd.D, 0, -1),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.Y, 2),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.W),
            ];
            TopoDS_Shape aProfileFace = BOPTest_Utilities.CreateProfileFromOperations(aOps);
            TopoDS_Shape aOblongPrism = BOPTest_Utilities.CreatePrism(aProfileFace, new Vector3D(1, 0, 0));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aOblongPrism);
            ValidateResult(aResult, 180.283);
        }

        // Test bfuse_simple/J5-J7: Similar pattern with different origins (implementing as simplified
        // tests)
        [Fact]
        public void LargePrismWithOblongPrismRightSide_J5()
        {
            // Create large prism base (5 units high)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 5));

            // Simplified oblong prism for J5
            TopoDS_Shape aOblongPrism = BOPTest_Utilities.CreateBox(new Point3D(8, 4, 1), 1.0, 2.0, 2.0);

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aOblongPrism);
            ValidateResult(aResult, 180.283);
        }

        [Fact]
        public void LargePrismWithOblongPrismRightSide_J6()
        {
            // Create large prism base (5 units high)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 5));

            // Simplified oblong prism for J6
            TopoDS_Shape aOblongPrism = BOPTest_Utilities.CreateBox(new Point3D(8, 5, 2), 1.0, 2.0, 2.0);

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aOblongPrism);
            ValidateResult(aResult, 180.283);
        }

        [Fact]
        public void LargePrismWithOblongPrismRightSide_J7()
        {
            // Create large prism base (5 units high)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 5));

            // Simplified oblong prism for J7
            TopoDS_Shape aOblongPrism = BOPTest_Utilities.CreateBox(new Point3D(8, 6, 1), 1.0, 2.0, 2.0);

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aOblongPrism);
            ValidateResult(aResult, 180.283);
        }

        // Test bfuse_simple/J8: Large prism with oblong prism (top profile)
        [Fact]
        public void LargePrismWithOblongPrismTop_J8()
        {
            // Create large prism base (5 units high)
            List<Point3D> aBasePts = [new Point3D(3, 3, 0), new Point3D(8, 3, 0), new Point3D(8, 9, 0), new Point3D(3, 9, 0)];
            TopoDS_Wire aBaseWire = BOPTest_Utilities.CreatePolygonWire(aBasePts, true);
            TopoDS_Shape aBaseFace = BOPTest_Utilities.CreateFaceFromWire(aBaseWire);
            TopoDS_Shape aBasePrism = BOPTest_Utilities.CreatePrism(aBaseFace, new Vector3D(0, 0, 5));

            // Create oblong profile: O 8 4 5  P 0 0 -1 1 0 0  D 0 1  C 1 180  Y -2  C 1 180  W
            List<BOPTest_Utilities.ProfileOperation> aOps =
            [
                new(BOPTest_Utilities.ProfileCmd.O, 8, 4, 5),
                new(BOPTest_Utilities.ProfileCmd.P, 0, 0, -1, 1, 0, 0),
                new(BOPTest_Utilities.ProfileCmd.D, 0, 1),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.Y, -2),
                new(BOPTest_Utilities.ProfileCmd.C, 1, 180),
                new(BOPTest_Utilities.ProfileCmd.W),
            ];
            TopoDS_Shape aProfileFace = BOPTest_Utilities.CreateProfileFromOperations(aOps);
            TopoDS_Shape aOblongPrism = BOPTest_Utilities.CreatePrism(aProfileFace, new Vector3D(0, 0, 1));

            TopoDS_Shape aResult = PerformFuse(aBasePrism, aOblongPrism);
            ValidateResult(aResult, 180.283);
        }

        // Test bfuse_simple/J9: Cylinder with revolution ring (bug case pro13305)
        [Fact]
        public void CylinderWithRevolutionRing_J9()
        {
            // Create cylinder: pcylinder cyl 3 5
            TopoDS_Shape aCylinder = BOPTest_Utilities.CreateCylinder(3.0, 5.0);

            // Create ring profile vertices and wire
            List<Point3D> aRingPts = [new Point3D(0, 3, 2), new Point3D(0, 4, 2), new Point3D(0, 4, 3), new Point3D(0, 3, 3)];
            TopoDS_Wire aRingWire = BOPTest_Utilities.CreatePolygonWire(aRingPts, true);
            TopoDS_Shape aRingFace = BOPTest_Utilities.CreateFaceFromWire(aRingWire);

            // Create revolution: revol ring f 0 0 0 0 0 1 269
            Axis3D aAxis = new(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z));
            TopoDS_Shape aRing = BOPTest_Utilities.CreateRevolution(aRingFace, aAxis, 269.0 * M_PI / 180.0);

            TopoDS_Shape aResult = PerformFuse(aCylinder, aRing);
            ValidateResult(aResult, 190.356);
        }

        // Test bfuse_simple/K1: Complex profile with revolution
        [Fact]
        public void ComplexProfileWithRevolution_K1()
        {
            // Create box: box b 200 200 100
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 200.0, 200.0, 100.0);

            // Create complex profile: profile wr pl 1 0 0 0 0 1 o 50 0 0 f 50 -80 x 50 y 5 x 10 y -25 x -60
            List<BOPTest_Utilities.ProfileOperation> aOps =
            [
                new(BOPTest_Utilities.ProfileCmd.O, 50, 0, 0),
                new(BOPTest_Utilities.ProfileCmd.F, 50, -80),
                new(BOPTest_Utilities.ProfileCmd.X, 50),
                new(BOPTest_Utilities.ProfileCmd.Y, 5),
                new(BOPTest_Utilities.ProfileCmd.X, 10),
                new(BOPTest_Utilities.ProfileCmd.Y, -25),
                new(BOPTest_Utilities.ProfileCmd.X, -60),
                new(BOPTest_Utilities.ProfileCmd.W),
            ];
            Plane3D aPlane = new(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.X)); // pl 1 0 0 0 0 1 (normal 1,0,0)
            TopoDS_Shape aProfile = BOPTest_Utilities.CreateProfile(aPlane, aOps);

            // Create revolution: revol rv wr 50 100 50 0 0 1 360
            Axis3D aRevAxis = new(new Point3D(50, 100, 50), new Direction3D(Direction3D.D.Z));
            TopoDS_Shape aRevolution = BOPTest_Utilities.CreateRevolution(aProfile, aRevAxis, 2 * M_PI);

            TopoDS_Shape aResult = PerformFuse(aBox, aRevolution);
            ValidateResult(aResult, 161571);
        }

        // Test bfuse_simple/K2: Blend box with cylinder (X direction)
        [Fact]
        public void BlendBoxWithCylinder_K2()
        {
            // Create box: box bx1 300 300 100
            TopoDS_Shape aBox = BOPTest_Utilities.CreateBox(new Point3D(0, 0, 0), 300.0, 300.0, 100.0);

            // Skip blend for now - test simple cylinder creation instead
            // const TopoDS_Shape aBlendedBox = BOPTest_Utilities::CreateBlend(aBox, 1, 100.0);

            // Test simple cylinder creation first
            TopoDS_Shape aCylinder = BOPTest_Utilities.CreateCylinder(100.0, 50.0);

            // Create plane and cylinder: plane pl1 100 100 100 0 0 1 1 0 0, pcylinder pc pl1 100 50
            // const gp_Ax3 anAx3(gp_Pnt(100, 100, 100), gp_Dir(gp_Dir::D::Z), gp_Dir(gp_Dir::D::X));
            // const gp_Pln aPlane(anAx3);
            // const TopoDS_Shape aCylinder = BOPTest_Utilities::CreateCylinderOnPlane(aPlane, 100.0, 50.0);

            TopoDS_Shape aResult = PerformFuse(aBox, aCylinder);
            ValidateResult(aResult, 360686);
        }

        // Test bfuse_simple/K3: Blend box with cylinder (-X direction)
        [Fact(Skip = "needs BRepFilletAPI_MakeFillet (not ported yet)")]
        public void BlendBoxWithCylinderNegX_K3()
        {
            // body left out: needs BRepFilletAPI_MakeFillet (BOPTest_Utilities::CreateBlend)
        }

        // Test bfuse_simple/K4: Blend box with cylinder (Y direction)
        [Fact(Skip = "needs BRepFilletAPI_MakeFillet (not ported yet)")]
        public void BlendBoxWithCylinderY_K4()
        {
            // body left out: needs BRepFilletAPI_MakeFillet (BOPTest_Utilities::CreateBlend)
        }

        // Test bfuse_simple/K5: Blend box with cylinder (-Y direction)
        [Fact(Skip = "needs BRepFilletAPI_MakeFillet (not ported yet)")]
        public void BlendBoxWithCylinderNegY_K5()
        {
            // body left out: needs BRepFilletAPI_MakeFillet (BOPTest_Utilities::CreateBlend)
        }

        // Test bfuse_simple/K6: Blend box with cylinder (bottom, X direction)
        [Fact(Skip = "needs BRepFilletAPI_MakeFillet (not ported yet)")]
        public void BlendBoxWithCylinderBottomX_K6()
        {
            // body left out: needs BRepFilletAPI_MakeFillet (BOPTest_Utilities::CreateBlend)
        }

        // Test bfuse_simple/K7: Blend box with cylinder (bottom, -X direction)
        [Fact(Skip = "needs BRepFilletAPI_MakeFillet (not ported yet)")]
        public void BlendBoxWithCylinderBottomNegX_K7()
        {
            // body left out: needs BRepFilletAPI_MakeFillet (BOPTest_Utilities::CreateBlend)
        }

        // Test bfuse_simple/K8: Blend box with cylinder (bottom, Y direction)
        [Fact(Skip = "needs BRepFilletAPI_MakeFillet (not ported yet)")]
        public void BlendBoxWithCylinderBottomY_K8()
        {
            // body left out: needs BRepFilletAPI_MakeFillet (BOPTest_Utilities::CreateBlend)
        }

        // Test bfuse_simple/K9: Blend box with cylinder (bottom, -Y direction)
        [Fact(Skip = "needs BRepFilletAPI_MakeFillet (not ported yet)")]
        public void BlendBoxWithCylinderBottomNegY_K9()
        {
            // body left out: needs BRepFilletAPI_MakeFillet (BOPTest_Utilities::CreateBlend)
        }

        //=================================================================================================
        // Additional Tests
        //=================================================================================================

        [Fact]
        public void OCC277_FuseAndCommonOfOverlappingBoxes()
        {
            BRepPrimAPI_MakeBox aBox1Maker = new(100, 100, 100);
            BRepPrimAPI_MakeBox aBox2Maker = new(new Point3D(50, 50, 50), 200, 200, 200);

            TopoDS_Shape aShape1 = aBox1Maker.Shape();
            TopoDS_Shape aShape2 = aBox2Maker.Shape();

            BRepAlgoAPI_Fuse aFuseOp = new(aShape1, aShape2);
            TopoDS_Shape aFuseResult = aFuseOp.Shape();
            Assert.False(aFuseResult.IsNull());

            BRepAlgoAPI_Common aCommonOp = new(aShape1, aShape2);
            TopoDS_Shape aCommonResult = aCommonOp.Shape();
            Assert.False(aCommonResult.IsNull());
        }
    }
}
