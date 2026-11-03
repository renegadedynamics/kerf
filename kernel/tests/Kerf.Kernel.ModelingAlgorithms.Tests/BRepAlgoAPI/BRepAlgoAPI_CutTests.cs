// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;
using static Kerf.Kernel.ModelingAlgorithms.Tests.BRepAlgoAPI.BOPTest_Utilities;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepAlgoAPI;

/// <summary>
/// OCCT's GTests of BRepAlgoAPI_Cut (src/ModelingAlgorithms/TKBO/GTests/BRepAlgoAPI_Cut_Test.cxx): the bcut_simple
/// series migrated from /tests/boolean/bcut_simple/ (fixture BCutSimpleTest) and the OCC825 bug test. OCCT's fixture
/// classes are nested classes, so the tests keep their names.
/// </summary>
public static class BRepAlgoAPI_CutTests
{
    private static int countSubShapes(in Shape theShape, ShapeType theType)
    {
        int aCount = 0;
        for (Explorer anExp = new(theShape, theType); anExp.More(); anExp.Next())
        {
            ++aCount;
        }
        return aCount;
    }

    //=============================================================================================
    // BCut Simple Tests - migrating from /tests/boolean/bcut_simple/
    //=============================================================================================

    public class BCutSimpleTest : BRepAlgoAPI_TestBase
    {
        // Test bcut_simple/A1: psphere s 1; box b 1 1 1; bcut result s b; checkprops result -s 13.3518
        [Fact]
        public void SphereMinusBox_A1()
        {
            Shape aSphere = CreateUnitSphere();
            Shape aBox = CreateUnitBox();

            Shape aResult = PerformCut(aSphere, aBox);
            ValidateResult(aResult, 13.3518);
        }

        // Test bcut_simple/A2: rotated sphere - box
        [Fact]
        public void RotatedSphereMinusBox_A2()
        {
            Shape aSphere = CreateUnitSphere();

            // Apply standard rotation: Z(-90deg) then Y(-45deg)
            Shape aRotatedSphere = RotateStandard(aSphere);

            Shape aBox = CreateUnitBox();
            Shape aResult = PerformCut(aRotatedSphere, aBox);
            ValidateResult(aResult, 13.3517);
        }

        // Test bcut_simple/A3: box - rotated sphere (reverse operation)
        [Fact]
        public void BoxMinusRotatedSphere_A3()
        {
            Shape aSphere = CreateUnitSphere();

            // Apply standard rotation: Z(-90deg) then Y(-45deg)
            Shape aRotatedSphere = RotateStandard(aSphere);

            Shape aBox = CreateUnitBox();
            Shape aResult = PerformCut(aBox, aRotatedSphere); // Note: reversed order
            ValidateResult(aResult, 5.2146);
        }

        // Test bcut_simple/A4: sphere - Y-rotated box
        [Fact]
        public void SphereMinusRotatedBox_A4()
        {
            Shape aSphere = CreateUnitSphere();
            Shape aBox = CreateUnitBox();
            Shape aRotatedBox = RotateY90(aBox);

            Shape aResult = PerformCut(aSphere, aRotatedBox);
            ValidateResult(aResult, 13.3517);
        }

        // Test bcut_simple/A5: Y-rotated box - sphere (reverse operation)
        [Fact]
        public void RotatedBoxMinusSphere_A5()
        {
            Shape aSphere = CreateUnitSphere();
            Shape aBox = CreateUnitBox();
            Shape aRotatedBox = RotateY90(aBox);

            Shape aResult = PerformCut(aRotatedBox, aSphere);
            ValidateResult(aResult, 5.2146);
        }

        // Test bcut_simple/A6: identical NURBS box - box (should result in empty)
        [Fact]
        public void IdenticalNurbsBoxMinusBox_A6()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);

            Shape aResult = PerformCut(aNurbsBox, aBox);
            ValidateResult(aResult, -1.0, -1.0, true); // Expected empty
        }

        // Test bcut_simple/A7: identical box - NURBS box (should result in empty)
        [Fact]
        public void IdenticalBoxMinusNurbsBox_A7()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);

            Shape aResult = PerformCut(aBox, aNurbsBox);
            ValidateResult(aResult, -1.0, -1.0, true); // Expected empty
        }

        // Test bcut_simple/A8: NURBS box - larger box (should result in empty)
        [Fact]
        public void NurbsBoxMinusLargerBox_A8()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aLargerBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.5, 1.0);

            Shape aResult = PerformCut(aNurbsBox, aLargerBox);
            ValidateResult(aResult, -1.0, -1.0, true); // Expected empty
        }

        // Test bcut_simple/A9: larger box - NURBS box
        [Fact]
        public void LargerBoxMinusNurbsBox_A9()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aLargerBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.5, 1.0);

            Shape aResult = PerformCut(aLargerBox, aNurbsBox);
            ValidateResult(aResult, 4.0);
        }

        // Test bcut_simple/B1: nurbs box - box
        [Fact]
        public void NurbsBoxMinusBox_B1()
        {
            // Create first box and convert to NURBS
            Shape aBox1 = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull, "Failed to convert to NURBS");

            // Create second box
            Shape aBox2 = CreateBox(new Point3D(0, 1, 0), 1.0, 0.5, 1.0);

            Shape aResult = PerformCut(aBox1, aBox2);
            ValidateResult(aResult, 6.0);
        }

        // Test bcut_simple/B2: box - NURBS box (reverse B1)
        [Fact]
        public void BoxMinusNurbsBox_B2()
        {
            // Create first box and convert to NURBS
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            // Create second box
            Shape aBox = CreateBox(new Point3D(0, 1, 0), 1.0, 0.5, 1.0);

            Shape aResult = PerformCut(aBox, aNurbsBox);
            ValidateResult(aResult, 4.0);
        }

        // Test bcut_simple/B3: NURBS box - adjacent box
        [Fact]
        public void NurbsBoxMinusAdjacentBox_B3()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aAdjacentBox = CreateBox(new Point3D(1, 1, 0), 1.0, 1.0, 1.0);

            Shape aResult = PerformCut(aNurbsBox, aAdjacentBox);
            ValidateResult(aResult, 6.0);
        }

        // Test bcut_simple/B4: adjacent box - NURBS box (reverse B3)
        [Fact]
        public void AdjacentBoxMinusNurbsBox_B4()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aAdjacentBox = CreateBox(new Point3D(1, 1, 0), 1.0, 1.0, 1.0);

            Shape aResult = PerformCut(aAdjacentBox, aNurbsBox);
            ValidateResult(aResult, 6.0);
        }

        // Test bcut_simple/B5: NURBS box - smaller contained box
        [Fact]
        public void NurbsBoxMinusSmallerBox_B5()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aSmallerBox = CreateBox(new Point3D(0, 0, 0), 0.5, 1.0, 0.5);

            Shape aResult = PerformCut(aNurbsBox, aSmallerBox);
            ValidateResult(aResult, 5.5);
        }

        // Test bcut_simple/B6: smaller box - NURBS box (should result in empty)
        [Fact]
        public void SmallerBoxMinusNurbsBox_B6()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aSmallerBox = CreateBox(new Point3D(0, 0, 0), 0.5, 1.0, 0.5);

            Shape aResult = PerformCut(aSmallerBox, aNurbsBox);
            ValidateResult(aResult, -1.0, -1.0, true); // Expected empty
        }

        // Test bcut_simple/B7: NURBS box - partially overlapping box
        [Fact]
        public void NurbsBoxMinusPartiallyOverlappingBox_B7()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aPartialBox = CreateBox(new Point3D(0, -0.5, 0), 0.5, 0.5, 1.0);

            Shape aResult = PerformCut(aNurbsBox, aPartialBox);
            ValidateResult(aResult, 6.0);
        }

        // Test bcut_simple/B8: partially overlapping box - NURBS box
        [Fact]
        public void PartiallyOverlappingBoxMinusNurbsBox_B8()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aPartialBox = CreateBox(new Point3D(0, -0.5, 0), 0.5, 0.5, 1.0);

            Shape aResult = PerformCut(aPartialBox, aNurbsBox);
            ValidateResult(aResult, 2.5);
        }

        // Test bcut_simple/B9: NURBS box - extended overlapping box
        [Fact]
        public void NurbsBoxMinusExtendedBox_B9()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aExtendedBox = CreateBox(new Point3D(0, -0.5, 0), 0.5, 1.5, 1.0);

            Shape aResult = PerformCut(aNurbsBox, aExtendedBox);
            ValidateResult(aResult, 4.0);
        }

        // Test bcut_simple/C1: extended box - NURBS box (from B8 setup)
        [Fact]
        public void ExtendedBoxMinusNurbsBox_C1()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aExtendedBox = CreateBox(new Point3D(0, -0.5, 0), 0.5, 1.5, 1.0);

            Shape aResult = PerformCut(aExtendedBox, aNurbsBox);
            ValidateResult(aResult, 2.5);
        }

        // Test bcut_simple/C2: NURBS box - shifted overlapping box
        [Fact]
        public void NurbsBoxMinusShiftedBox_C2()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aShiftedBox = CreateBox(new Point3D(0, 0.5, 0), 1.0, 1.0, 1.0);

            Shape aResult = PerformCut(aNurbsBox, aShiftedBox);
            ValidateResult(aResult, 4.0);
        }

        // Test bcut_simple/C3: shifted box - NURBS box (reverse C2)
        [Fact]
        public void ShiftedBoxMinusNurbsBox_C3()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aShiftedBox = CreateBox(new Point3D(0, 0.5, 0), 1.0, 1.0, 1.0);

            Shape aResult = PerformCut(aShiftedBox, aNurbsBox);
            ValidateResult(aResult, 4.0);
        }

        // Test bcut_simple/C4: NURBS box - narrow contained box
        [Fact]
        public void NurbsBoxMinusNarrowBox_C4()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aNarrowBox = CreateBox(new Point3D(0, 0.25, 0), 1.0, 0.5, 1.0);

            Shape aResult = PerformCut(aNurbsBox, aNarrowBox);
            ValidateResult(aResult, 6.0);
        }

        // Test bcut_simple/C5: narrow box - NURBS box (should result in empty)
        [Fact]
        public void NarrowBoxMinusNurbsBox_C5()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aNarrowBox = CreateBox(new Point3D(0, 0.25, 0), 1.0, 0.5, 1.0);

            Shape aResult = PerformCut(aNarrowBox, aNurbsBox);
            ValidateResult(aResult, -1.0, -1.0, true); // Expected empty
        }

        // Test bcut_simple/C6: NURBS box - small corner cube
        [Fact]
        public void NurbsBoxMinusCornerCube_C6()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aCornerCube = CreateBox(new Point3D(0, 0, 0), 0.5, 0.5, 0.5);

            Shape aResult = PerformCut(aNurbsBox, aCornerCube);
            ValidateResult(aResult, 6.0);
        }

        // Test bcut_simple/C7: small corner cube - NURBS box (should result in empty)
        [Fact]
        public void CornerCubeMinusNurbsBox_C7()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aCornerCube = CreateBox(new Point3D(0, 0, 0), 0.5, 0.5, 0.5);

            Shape aResult = PerformCut(aCornerCube, aNurbsBox);
            ValidateResult(aResult, -1.0, -1.0, true); // Expected empty
        }

        // Test bcut_simple/C8: NURBS box - offset small cube
        [Fact]
        public void NurbsBoxMinusOffsetCube_C8()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aOffsetCube = CreateBox(new Point3D(0, -0.5, 0), 0.5, 0.5, 0.5);

            Shape aResult = PerformCut(aNurbsBox, aOffsetCube);
            ValidateResult(aResult, 6.0);
        }

        // Test bcut_simple/C9: offset small cube - NURBS box
        [Fact]
        public void OffsetCubeMinusNurbsBox_C9()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aOffsetCube = CreateBox(new Point3D(0, -0.5, 0), 0.5, 0.5, 0.5);

            Shape aResult = PerformCut(aOffsetCube, aNurbsBox);
            ValidateResult(aResult, 1.5);
        }

        // Test bcut_simple/D1: NURBS box - offset corner cube
        [Fact]
        public void NurbsBoxMinusOffsetCornerCube_D1()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aOffsetCornerCube = CreateBox(new Point3D(0, -0.5, -0.5), 0.5, 0.5, 0.5);

            Shape aResult = PerformCut(aNurbsBox, aOffsetCornerCube);
            ValidateResult(aResult, 6.0);
        }

        // Test bcut_simple/D2: offset corner cube - NURBS box
        [Fact]
        public void OffsetCornerCubeMinusNurbsBox_D2()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aOffsetCornerCube = CreateBox(new Point3D(0, -0.5, -0.5), 0.5, 0.5, 0.5);

            Shape aResult = PerformCut(aOffsetCornerCube, aNurbsBox);
            ValidateResult(aResult, 1.5);
        }

        // Test bcut_simple/D3: NURBS box - shifted corner cube
        [Fact]
        public void NurbsBoxMinusShiftedCornerCube_D3()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aShiftedCornerCube = CreateBox(new Point3D(-0.5, -0.5, -0.5), 0.5, 0.5, 0.5);

            Shape aResult = PerformCut(aNurbsBox, aShiftedCornerCube);
            ValidateResult(aResult, 6.0);
        }

        // Test bcut_simple/D4: shifted corner cube - NURBS box
        [Fact]
        public void ShiftedCornerCubeMinusNurbsBox_D4()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aShiftedCornerCube = CreateBox(new Point3D(-0.5, -0.5, -0.5), 0.5, 0.5, 0.5);

            Shape aResult = PerformCut(aShiftedCornerCube, aNurbsBox);
            ValidateResult(aResult, 1.5);
        }

        // Test bcut_simple/D5: NURBS box - extended X box
        [Fact]
        public void NurbsBoxMinusExtendedXBox_D5()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aExtendedXBox = CreateBox(new Point3D(0, 0, 0), 1.5, 0.5, 0.5);

            Shape aResult = PerformCut(aNurbsBox, aExtendedXBox);
            ValidateResult(aResult, 5.5);
        }

        // Test bcut_simple/D6: extended X box - NURBS box
        [Fact]
        public void ExtendedXBoxMinusNurbsBox_D6()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aExtendedXBox = CreateBox(new Point3D(0, 0, 0), 1.5, 0.5, 0.5);

            Shape aResult = PerformCut(aExtendedXBox, aNurbsBox);
            ValidateResult(aResult, 1.5);
        }

        // Test bcut_simple/D7: NURBS box - offset extended X box
        [Fact]
        public void NurbsBoxMinusOffsetExtendedXBox_D7()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aOffsetExtendedXBox = CreateBox(new Point3D(0, -0.5, 0), 1.5, 0.5, 0.5);

            Shape aResult = PerformCut(aNurbsBox, aOffsetExtendedXBox);
            ValidateResult(aResult, 6.0);
        }

        // Test bcut_simple/D8: offset extended X box - NURBS box
        [Fact]
        public void OffsetExtendedXBoxMinusNurbsBox_D8()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aOffsetExtendedXBox = CreateBox(new Point3D(0, -0.5, 0), 1.5, 0.5, 0.5);

            Shape aResult = PerformCut(aOffsetExtendedXBox, aNurbsBox);
            ValidateResult(aResult, 3.5);
        }

        // Test bcut_simple/D9: NURBS box - shifted narrow box
        [Fact]
        public void NurbsBoxMinusShiftedNarrowBox_D9()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aShiftedNarrowBox = CreateBox(new Point3D(0.25, 0, 0), 0.5, 0.5, 1.0);

            Shape aResult = PerformCut(aNurbsBox, aShiftedNarrowBox);
            ValidateResult(aResult, 6.5);
        }

        // Test bcut_simple/E1: shifted narrow box - NURBS box (should result in empty)
        [Fact]
        public void ShiftedNarrowBoxMinusNurbsBox_E1()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aShiftedNarrowBox = CreateBox(new Point3D(0.25, 0, 0), 0.5, 0.5, 1.0);

            Shape aResult = PerformCut(aShiftedNarrowBox, aNurbsBox);
            ValidateResult(aResult, -1.0, -1.0, true); // Expected empty
        }

        // Test bcut_simple/E2: NURBS box - offset shifted narrow box
        [Fact]
        public void NurbsBoxMinusOffsetShiftedNarrowBox_E2()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aOffsetShiftedNarrowBox = CreateBox(new Point3D(0.25, -0.5, 0), 0.5, 0.5, 1.0);

            Shape aResult = PerformCut(aNurbsBox, aOffsetShiftedNarrowBox);
            ValidateResult(aResult, 6.0);
        }

        // Test bcut_simple/E3: offset shifted narrow box - NURBS box
        [Fact]
        public void OffsetShiftedNarrowBoxMinusNurbsBox_E3()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aOffsetShiftedNarrowBox = CreateBox(new Point3D(0.25, -0.5, 0), 0.5, 0.5, 1.0);

            Shape aResult = PerformCut(aOffsetShiftedNarrowBox, aNurbsBox);
            ValidateResult(aResult, 2.5);
        }

        // Test bcut_simple/E4: NURBS box - extended Y shifted narrow box
        [Fact]
        public void NurbsBoxMinusExtendedYShiftedNarrowBox_E4()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aExtendedYShiftedNarrowBox = CreateBox(new Point3D(0.25, 0, 0), 0.5, 1.5, 1.0);

            Shape aResult = PerformCut(aNurbsBox, aExtendedYShiftedNarrowBox);
            ValidateResult(aResult, 6.0);
        }

        // Test bcut_simple/E5: extended Y shifted narrow box - NURBS box
        [Fact]
        public void ExtendedYShiftedNarrowBoxMinusNurbsBox_E5()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aExtendedYShiftedNarrowBox = CreateBox(new Point3D(0.25, 0, 0), 0.5, 1.5, 1.0);

            Shape aResult = PerformCut(aExtendedYShiftedNarrowBox, aNurbsBox);
            ValidateResult(aResult, 2.5);
        }

        // Test bcut_simple/E6: NURBS box - right half box
        [Fact]
        public void NurbsBoxMinusRightHalfBox_E6()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aRightHalfBox = CreateBox(new Point3D(0.5, 0, 0), 1.0, 1.0, 0.5);

            Shape aResult = PerformCut(aNurbsBox, aRightHalfBox);
            ValidateResult(aResult, 5.5);
        }

        // Test bcut_simple/E7: right half box - NURBS box
        [Fact]
        public void RightHalfBoxMinusNurbsBox_E7()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aRightHalfBox = CreateBox(new Point3D(0.5, 0, 0), 1.0, 1.0, 0.5);

            Shape aResult = PerformCut(aRightHalfBox, aNurbsBox);
            ValidateResult(aResult, 2.5);
        }

        // Test bcut_simple/E8: NURBS box - offset right half box
        [Fact]
        public void NurbsBoxMinusOffsetRightHalfBox_E8()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aOffsetRightHalfBox = CreateBox(new Point3D(0.5, 0, -0.5), 1.0, 1.0, 0.5);

            Shape aResult = PerformCut(aNurbsBox, aOffsetRightHalfBox);
            ValidateResult(aResult, 6.0);
        }

        // Test bcut_simple/E9: offset right half box - NURBS box
        [Fact]
        public void OffsetRightHalfBoxMinusNurbsBox_E9()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            Assert.False(aNurbsBox.IsNull, "Failed to convert to NURBS");

            Shape aOffsetRightHalfBox = CreateBox(new Point3D(0.5, 0, -0.5), 1.0, 1.0, 0.5);

            Shape aResult = PerformCut(aOffsetRightHalfBox, aNurbsBox);
            ValidateResult(aResult, 4.0);
        }

        // Test bcut_simple/F1: NURBS box - rotated rectangular box (sqrt(2) x sqrt(2)/2 x 1, 45deg)
        [Fact]
        public void NurbsBoxMinusRotatedRectangularBox_F1()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            const double r = M_SQRT2;
            Shape aRectangularBox = CreateBox(new Point3D(0, 0, 0), r, r / 2.0, 1.0);
            aRectangularBox = RotateShape(aRectangularBox, new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 45.0 * M_PI / 180.0);
            Shape aResult = PerformCut(aNurbsBox, aRectangularBox);
            ValidateResult(aResult, 4.41421);
        }

        // Test bcut_simple/F2: rotated rectangular box - NURBS box
        [Fact]
        public void RotatedRectangularBoxMinusNurbsBox_F2()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            const double r = M_SQRT2;
            Shape aRectangularBox = CreateBox(new Point3D(0, 0, 0), r, r / 2.0, 1.0);
            aRectangularBox = RotateShape(aRectangularBox, new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 45.0 * M_PI / 180.0);
            Shape aResult = PerformCut(aRectangularBox, aNurbsBox);
            ValidateResult(aResult, 5.82843);
        }

        // Test bcut_simple/F3: NURBS box - rotated square box (sqrt(2)/2 x sqrt(2)/2 x 1, 45deg)
        [Fact]
        public void NurbsBoxMinusRotatedSquareBox_F3()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            double r = Math.Sqrt(2.0) / 2.0;
            Shape aSquareBox = CreateBox(new Point3D(0, 0, 0), r, r, 1.0);
            aSquareBox = RotateShape(aSquareBox, new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 45.0 * M_PI / 180.0);
            Shape aResult = PerformCut(aNurbsBox, aSquareBox);
            ValidateResult(aResult, 5.91421);
        }

        // Test bcut_simple/F4: rotated square box - NURBS box
        [Fact]
        public void RotatedSquareBoxMinusNurbsBox_F4()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            double r = Math.Sqrt(2.0) / 2.0;
            Shape aSquareBox = CreateBox(new Point3D(0, 0, 0), r, r, 1.0);
            aSquareBox = RotateShape(aSquareBox, new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 45.0 * M_PI / 180.0);
            Shape aResult = PerformCut(aSquareBox, aNurbsBox);
            ValidateResult(aResult, 2.91421);
        }

        // Test bcut_simple/F5: NURBS box - rotated thin box (sqrt(2) x 0.25 x 1, 45deg)
        [Fact]
        public void NurbsBoxMinusRotatedThinBox_F5()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            const double r = M_SQRT2;
            Shape aThinBox = CreateBox(new Point3D(0, 0, 0), r, 0.25, 1.0);
            aThinBox = RotateShape(aThinBox, new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 45.0 * M_PI / 180.0);
            Shape aResult = PerformCut(aNurbsBox, aThinBox);
            ValidateResult(aResult, 7.03921);
        }

        // Test bcut_simple/F6: rotated thin box - NURBS box
        [Fact]
        public void RotatedThinBoxMinusNurbsBox_F6()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            const double r = M_SQRT2;
            Shape aThinBox = CreateBox(new Point3D(0, 0, 0), r, 0.25, 1.0);
            aThinBox = RotateShape(aThinBox, new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 45.0 * M_PI / 180.0);
            Shape aResult = PerformCut(aThinBox, aNurbsBox);
            ValidateResult(aResult, 1.83211);
        }

        // Test bcut_simple/F7: NURBS box - rotated narrow box (sqrt(31)/4 x 0.25 x 1, 34.73deg)
        [Fact]
        public void NurbsBoxMinusRotatedNarrowBox_F7()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            const double r = 5.5677643628300219; // sqrt(31.0)
            Shape aNarrowBox = CreateBox(new Point3D(0, 0, 0), r / 4.0, 0.25, 1.0);
            aNarrowBox = RotateShape(aNarrowBox, new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 34.73 * M_PI / 180.0);
            Shape aResult = PerformCut(aNurbsBox, aNarrowBox);
            ValidateResult(aResult, 7.21677);
        }

        // Test bcut_simple/F8: rotated narrow box - NURBS box
        [Fact]
        public void RotatedNarrowBoxMinusNurbsBox_F8()
        {
            Shape aNurbsBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aNurbsBox = ConvertToNurbs(aNurbsBox);
            const double r = 5.5677643628300219;
            Shape aNarrowBox = CreateBox(new Point3D(0, 0, 0), r / 4.0, 0.25, 1.0);
            aNarrowBox = RotateShape(aNarrowBox, new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 34.73 * M_PI / 180.0);
            Shape aResult = PerformCut(aNarrowBox, aNurbsBox);
            ValidateResult(aResult, 1.54631);
        }

        // Test bcut_simple/F9: sphere - box
        [Fact]
        public void SphereMinusBox_F9()
        {
            Shape aSphere = CreateSphere(new Point3D(0, 0, 0), 1.0);
            Shape aBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            Shape aResult = PerformCut(aSphere, aBox);
            ValidateResult(aResult, 13.3518);
        }

        // Test bcut_simple/G1: box - sphere
        [Fact]
        public void BoxMinusSphere_G1()
        {
            Shape aSphere = CreateSphere(new Point3D(0, 0, 0), 1.0);
            Shape aBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            Shape aResult = PerformCut(aBox, aSphere);
            ValidateResult(aResult, 5.2146);
        }

        // Test bcut_simple/G2: rotated sphere - box
        [Fact]
        public void RotatedSphereMinusBox_G2()
        {
            Shape aSphere = CreateSphere(new Point3D(0, 0, 0), 1.0);
            aSphere = RotateShape(aSphere, new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), -90.0 * M_PI / 180.0);
            aSphere = RotateShape(aSphere, new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Y)), -45.0 * M_PI / 180.0);
            Shape aBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            Shape aResult = PerformCut(aSphere, aBox);
            ValidateResult(aResult, 13.3517);
        }

        // Test bcut_simple/G3: box - rotated sphere
        [Fact]
        public void BoxMinusRotatedSphere_G3()
        {
            Shape aSphere = CreateSphere(new Point3D(0, 0, 0), 1.0);
            aSphere = RotateShape(aSphere, new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), -90.0 * M_PI / 180.0);
            aSphere = RotateShape(aSphere, new Axis3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Y)), -45.0 * M_PI / 180.0);
            Shape aBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            Shape aResult = PerformCut(aBox, aSphere);
            ValidateResult(aResult, 5.2146);
        }

        // Test bcut_simple/G4: sphere - rotated box
        [Fact]
        public void SphereMinusRotatedBox_G4()
        {
            Shape aSphere = CreateSphere(new Point3D(0, 0, 0), 1.0);
            Shape aBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox = RotateShape(aBox, new Axis3D(new Point3D(0, 0, 1), new Direction3D(Direction3D.D.Y)), 90.0 * M_PI / 180.0);
            Shape aResult = PerformCut(aSphere, aBox);
            ValidateResult(aResult, 13.3517);
        }

        // Test bcut_simple/G5: rotated box - sphere
        [Fact]
        public void RotatedBoxMinusSphere_G5()
        {
            Shape aSphere = CreateSphere(new Point3D(0, 0, 0), 1.0);
            Shape aBox = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox = RotateShape(aBox, new Axis3D(new Point3D(0, 0, 1), new Direction3D(Direction3D.D.Y)), 90.0 * M_PI / 180.0);
            Shape aResult = PerformCut(aBox, aSphere);
            ValidateResult(aResult, 5.2146);
        }

        // Test bcut_simple/G6: Complex profile/revol operation (pro5363 bug)
        [Fact]
        public void ComplexProfileRevolOperation_G6()
        {
            // Create box: "box b 100 100 40"
            Shape aBox = CreateBox(new Point3D(0, 0, 0), 100.0, 100.0, 40.0);

            // Create exact G6 profile: "profile rev S b_4 F 50 20 Y 50 C 10 180 Y -50 C 10 180"
            Plane3D aPlane = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z)); // Face b_4 of box (top face at Z=40)
            List<ProfileOperation> aProfileOps =
            [
                new ProfileOperation(ProfileCmd.F, 50.0, 20.0), // F 50 20
                new ProfileOperation(ProfileCmd.Y, 50.0), // Y 50
                new ProfileOperation(ProfileCmd.C, 10.0, 180.0), // C 10 180
                new ProfileOperation(ProfileCmd.Y, -50.0), // Y -50
                new ProfileOperation(ProfileCmd.C, 10.0, 180.0), // C 10 180
            ];
            Shape aProfile = CreateProfile(aPlane, aProfileOps);

            // Create revolution: "revol rev2 rev 0 0 50 0 1 0 360" (around Y-axis at Z=50)
            Shape aRevolution = CreateRevolution(aProfile, new Axis3D(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Y)), 2.0 * M_PI);

            // Perform boolean cut operation: "bcut result b rev2"
            Shape aResult = PerformCut(aBox, aRevolution);
            ValidateResult(aResult, 41187.4);
        }

        // Test bcut_simple/G7: Box minus translated box (buc40054 bug)
        [Fact]
        public void BoxMinusTranslatedBox_G7()
        {
            Shape aBox1 = CreateBox(new Point3D(0, 0, 0), 3.0, 3.0, 3.0);
            Shape aBox2 = CreateBox(new Point3D(0, 3, 0), 1.0, 1.0, 1.0);
            Shape aResult = PerformCut(aBox1, aBox2);
            ValidateResult(aResult, 54.0);
        }

        // Test bcut_simple/G8: Box minus prism (BUC40188 bug)
        [Fact]
        public void BoxMinusPrism_G8()
        {
            // Create box mb (-0.5, -0.5, -0.5, 1, 1, 1)
            Shape aBoxMb = CreateBox(new Point3D(-0.5, -0.5, -0.5), 1.0, 1.0, 1.0);

            // Get face mb_1 (approximated as first face)
            Face aFace = GetFaceByIndex(aBoxMb, 1);

            // Create semi-infinite prism (simulated with very large prism: prism pryz mb_1 1 0 0 SemiInf)
            // Semi-infinite prism approximated as large prism extending 1000 units in X direction
            Shape aSemiInfPrism = CreatePrism(aFace, new Vector3D(1000.0, 0.0, 0.0));

            // Create box ab (0, -1, -1, 2, 2, 2)
            Shape aBoxAb = CreateBox(new Point3D(0, -1, -1), 2.0, 2.0, 2.0);

            // Perform boolean cut operation: bcut result ab pryz
            Shape aResult = PerformCut(aBoxAb, aSemiInfPrism);
            ValidateResult(aResult, 30.0);
        }

        // Test bcut_simple/G9: Complex cylinder/cone fuse minus translated cylinder
        [Fact]
        public void ComplexCylinderConeOperation_G9()
        {
            // Create cylinder and cone, fuse them
            Shape aCylinder = CreateCylinder(9.0, 3.0);
            Shape aCone = CreateCone(7.0, 6.0, 4.0);

            Fuse aFuseOp = new(aCylinder, aCone);
            Assert.True(aFuseOp.IsDone, "Fuse operation failed");
            Shape aBody = aFuseOp.Shape();

            // Create small cylinder and translate it
            Shape aSmallCylinder = CreateCylinder(1.0, 9.0);
            aSmallCylinder = TranslateShape(aSmallCylinder, new Vector3D(5.0, 0.0, -2.0));

            Shape aResult = PerformCut(aBody, aSmallCylinder);
            ValidateResult(aResult, 727.481);
        }

        // Test bcut_simple/H1: Complex custom polygon prism minus box (MFA bug collection)
        [Fact]
        public void ComplexPolygonPrismMinusBox_H1()
        {
            // Create custom polygon from H1 test vertices: v1(0,0,0) v2(1,0,0) v3(1,3,0) v4(2,3,0) v5(2,0,0)
            // v6(3,0,0) v7(3,5,0) v8(0,5,0)
            List<Point3D> aPoints = [];
            aPoints.Add(new Point3D(0, 0, 0)); // v1
            aPoints.Add(new Point3D(1, 0, 0)); // v2
            aPoints.Add(new Point3D(1, 3, 0)); // v3
            aPoints.Add(new Point3D(2, 3, 0)); // v4
            aPoints.Add(new Point3D(2, 0, 0)); // v5
            aPoints.Add(new Point3D(3, 0, 0)); // v6
            aPoints.Add(new Point3D(3, 5, 0)); // v7
            aPoints.Add(new Point3D(0, 5, 0)); // v8

            // Create wire and extrude to solid (prism sol p 0 0 2)
            Wire aWire = CreatePolygonWire(aPoints, true);
            Shape aFace = CreateFaceFromWire(aWire);
            Shape aSolid = CreatePrism(aFace, new Vector3D(0, 0, 2));

            // Create box: box b -1 2 1 5 1 3 (corner at (-1,2,1), dimensions 5x1x3)
            Shape aBox = CreateBox(new Point3D(-1, 2, 1), 5.0, 1.0, 3.0);

            // Perform boolean cut: bcut result sol b
            Shape aResult = PerformCut(aSolid, aBox);
            ValidateResult(aResult, 68.0);
        }

        // Test bcut_simple/H2: Complex custom polygon prism minus box (pro13306 bug)
        [Fact]
        public void ComplexPolygonPrismMinusBox_H2()
        {
            // H2 is identical to H1 - same polygon and same box
            // Create custom polygon from H2 test vertices (same as H1): v1(0,0,0) v2(1,0,0) v3(1,3,0)
            // v4(2,3,0) v5(2,0,0) v6(3,0,0) v7(3,5,0) v8(0,5,0)
            List<Point3D> aPoints = [];
            aPoints.Add(new Point3D(0, 0, 0)); // v1
            aPoints.Add(new Point3D(1, 0, 0)); // v2
            aPoints.Add(new Point3D(1, 3, 0)); // v3
            aPoints.Add(new Point3D(2, 3, 0)); // v4
            aPoints.Add(new Point3D(2, 0, 0)); // v5
            aPoints.Add(new Point3D(3, 0, 0)); // v6
            aPoints.Add(new Point3D(3, 5, 0)); // v7
            aPoints.Add(new Point3D(0, 5, 0)); // v8

            // Create wire and extrude to solid (prism sol p 0 0 2)
            Wire aWire = CreatePolygonWire(aPoints, true);
            Shape aFace = CreateFaceFromWire(aWire);
            Shape aSolid = CreatePrism(aFace, new Vector3D(0, 0, 2));

            // Create box: box b -1 2 1 5 1 3 (corner at (-1,2,1), dimensions 5x1x3)
            Shape aBox = CreateBox(new Point3D(-1, 2, 1), 5.0, 1.0, 3.0);

            // Perform boolean cut: bcut result sol b
            Shape aResult = PerformCut(aSolid, aBox);
            ValidateResult(aResult, 68.0);
        }

        // Test bcut_simple/H3: Complex cylinder/cone fuse minus translated cylinder (pro13307 bug)
        [Fact]
        public void ComplexCylinderConeOperationPro13307_H3()
        {
            // This is similar to G9 but with a different bug reference
            // Create cylinder and cone, fuse them
            Shape aCylinder = CreateCylinder(9.0, 3.0);
            Shape aCone = CreateCone(7.0, 6.0, 4.0);

            Fuse aFuseOp = new(aCylinder, aCone);
            Assert.True(aFuseOp.IsDone, "Fuse operation failed");
            Shape aBody = aFuseOp.Shape();

            // Create small cylinder and translate it (x direction as per comment)
            Shape aSmallCylinder = CreateCylinder(1.0, 9.0);
            aSmallCylinder = TranslateShape(aSmallCylinder, new Vector3D(5.0, 0.0, -2.0));

            Shape aResult = PerformCut(aBody, aSmallCylinder);
            ValidateResult(aResult, 727.481);
        }

        // Test bcut_simple/H4: Complex profile operation FORWARD FORWARD
        [Fact]
        public void ComplexProfileForwardForward_H4()
        {
            // Create profile p1: "profile p1 o 0 0 40 x 150 y 200 x -150"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0), // o 0 0 40
                new ProfileOperation(ProfileCmd.X, 150.0), // x 150
                new ProfileOperation(ProfileCmd.Y, 200.0), // y 200
                new ProfileOperation(ProfileCmd.X, -150.0), // x -150
            ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            // Create profile p2: "profile p2 o 0 0 50 f 25 25 y 100 x 75 y -100"
            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 25.0, 25.0), // f 25 25
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
            ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            // First cut operation: bcut po1 pr1 pr2
            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            // Create profile p3: "profile p3 p 0 0 -1 1 0 0 o 0 0 50 f 50 -75 x 75 y -100 x -75"
            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 50.0, -75.0), // f 50 -75
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -30));

            // Final cut operation: bcut result po1 pr3
            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 98000.0);
        }

        // Test bcut_simple/H5: Complex profile operation FORWARD FORWARD (variation)
        [Fact]
        public void ComplexProfileForwardForwardVariation_H5()
        {
            // H5 has same profiles as H4, but p3 has different sequence: "f 50 -75 y -100 x 75 y 100"

            // Create profile p1: same as H4 "profile p1 o 0 0 40 x 150 y 200 x -150"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0), // o 0 0 40
                new ProfileOperation(ProfileCmd.X, 150.0), // x 150
                new ProfileOperation(ProfileCmd.Y, 200.0), // y 200
                new ProfileOperation(ProfileCmd.X, -150.0), // x -150
            ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            // Create profile p2: same as H4 "profile p2 o 0 0 50 f 25 25 y 100 x 75 y -100"
            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 25.0, 25.0), // f 25 25
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
            ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            // Create profile p3: H5 variation "profile p3 p 0 0 -1 1 0 0 o 0 0 50 f 50 -75 y -100 x 75 y 100"
            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 50.0, -75.0), // f 50 -75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
            ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -30));

            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 98000.0);
        }

        // Test bcut_simple/H6: Complex profile operation FORWARD REVERSED
        [Fact]
        public void ComplexProfileForwardReversed_H6()
        {
            // Create profile p1: same as H4/H5 "profile p1 o 0 0 40 x 150 y 200 x -150"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0), // o 0 0 40
                new ProfileOperation(ProfileCmd.X, 150.0), // x 150
                new ProfileOperation(ProfileCmd.Y, 200.0), // y 200
                new ProfileOperation(ProfileCmd.X, -150.0), // x -150
            ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            // Create profile p2: same as H4/H5 "profile p2 o 0 0 50 f 25 25 y 100 x 75 y -100"
            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 25.0, 25.0), // f 25 25
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
            ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            // Create profile p3: H6 variation "profile p3 o 0 0 50 f 50 75 x 75 y 100 x -75"
            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 50.0, 75.0), // f 50 75
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -30));

            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 98000.0);
        }

        // Test bcut_simple/H7: Complex profile operation FORWARD REVERSED (variation)
        [Fact]
        public void ComplexProfileForwardReversedVariation_H7()
        {
            // Create profile p1: same as other H tests "profile p1 o 0 0 40 x 150 y 200 x -150"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0), // o 0 0 40
                new ProfileOperation(ProfileCmd.X, 150.0), // x 150
                new ProfileOperation(ProfileCmd.Y, 200.0), // y 200
                new ProfileOperation(ProfileCmd.X, -150.0), // x -150
            ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            // Create profile p2: same as other H tests "profile p2 o 0 0 50 f 25 25 y 100 x 75 y -100"
            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 25.0, 25.0), // f 25 25
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
            ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            // Create profile p3: H7 variation "profile p3 o 0 0 50 f 50 75 y 100 x 75 y -100"
            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 50.0, 75.0), // f 50 75
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
            ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -30));

            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 98000.0);
        }

        // Test bcut_simple/H8: Complex profile operation REVERSED FORWARD
        [Fact]
        public void ComplexProfileReversedForward_H8()
        {
            // Create profile p1: "profile p1 p 0 0 -1 1 0 0 o 0 0 40 y -200 x 150 y 200"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z)); // Use same approach as working H4/H5
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0), // o 0 0 40
                new ProfileOperation(ProfileCmd.Y, -200.0), // y -200
                new ProfileOperation(ProfileCmd.X, 150.0), // x 150
                new ProfileOperation(ProfileCmd.Y, 200.0), // y 200
            ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            // Create profile p2: "profile p2 p 0 0 -1 1 0 0 o 0 0 50 f 25 -25 x 75 y -100 x -75"
            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 25.0, -25.0), // f 25 -25
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            // Create profile p3: "profile p3 p 0 0 -1 1 0 0 o 0 0 50 f 50 -75 x 75 y -100 x -75"
            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 50.0, -75.0), // f 50 -75
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -30));

            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 98000.0);
        }

        // Test bcut_simple/H9: Complex profile operation REVERSED FORWARD (variation)
        [Fact]
        public void ComplexProfileReversedForwardVariation_H9()
        {
            // Create profile p1: same as H8 "profile p1 p 0 0 -1 1 0 0 o 0 0 40 y -200 x 150 y 200"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0), // o 0 0 40
                new ProfileOperation(ProfileCmd.Y, -200.0), // y -200
                new ProfileOperation(ProfileCmd.X, 150.0), // x 150
                new ProfileOperation(ProfileCmd.Y, 200.0), // y 200
            ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            // Create profile p2: same as H8 "profile p2 p 0 0 -1 1 0 0 o 0 0 50 f 25 -25 x 75 y -100 x -75"
            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 25.0, -25.0), // f 25 -25
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            // Create profile p3: H9 variation "profile p3 p 0 0 -1 1 0 0 o 0 0 50 f 50 -75 y -100 x 75 y 100"
            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 50.0, -75.0), // f 50 -75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
            ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -30));

            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 98000.0);
        }

        // Test bcut_simple/I1: Complex profile operation REVERSED REVERSED
        [Fact]
        public void ComplexProfileReversedReversed_I1()
        {
            // Create profile p1: same as H8/H9 "profile p1 p 0 0 -1 1 0 0 o 0 0 40 y -200 x 150 y 200"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0), // o 0 0 40
                new ProfileOperation(ProfileCmd.Y, -200.0), // y -200
                new ProfileOperation(ProfileCmd.X, 150.0), // x 150
                new ProfileOperation(ProfileCmd.Y, 200.0), // y 200
            ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            // Create profile p2: same as H8/H9 "profile p2 p 0 0 -1 1 0 0 o 0 0 50 f 25 -25 x 75 y -100 x
            // -75"
            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 25.0, -25.0), // f 25 -25
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            // Create profile p3: I1 "profile p3 o 0 0 50 f 50 75 x 75 y 100 x -75" (similar to H6)
            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 50.0, 75.0), // f 50 75
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -30));

            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 98000.0);
        }

        // Test bcut_simple/I2: Complex profile operation REVERSED REVERSED (variation)
        [Fact]
        public void ComplexProfileReversedReversedVariation_I2()
        {
            // Create profile p1: same as I1 "profile p1 p 0 0 -1 1 0 0 o 0 0 40 y -200 x 150 y 200"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0), // o 0 0 40
                new ProfileOperation(ProfileCmd.Y, -200.0), // y -200
                new ProfileOperation(ProfileCmd.X, 150.0), // x 150
                new ProfileOperation(ProfileCmd.Y, 200.0), // y 200
            ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            // Create profile p2: same as I1 "profile p2 p 0 0 -1 1 0 0 o 0 0 50 f 25 -25 x 75 y -100 x -75"
            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 25.0, -25.0), // f 25 -25
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            // Create profile p3: I2 "profile p3 o 0 0 50 f 50 75 y 100 x 75 y -100" (similar to H7)
            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 50.0, 75.0), // f 50 75
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
            ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -30));

            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 98000.0);
        }

        // Test bcut_simple/I3: Complex profile operation FORWARD FORWARD (variation)
        [Fact]
        public void ComplexProfileForwardForwardVariation2_I3()
        {
            // Create profile p1: "profile p1 o 0 0 40 f 0 50 x 150 y 100 x -150"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0), // o 0 0 40
                new ProfileOperation(ProfileCmd.F, 0.0, 50.0), // f 0 50
                new ProfileOperation(ProfileCmd.X, 150.0), // x 150
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, -150.0), // x -150
            ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            // Create profile p2: same as I1/I2 but different plane "profile p2 o 0 0 50 f 25 25 y 100 x 75 y
            // -100"
            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 25.0, 25.0), // f 25 25
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
            ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            // Create profile p3: same as H8 "profile p3 p 0 0 -1 1 0 0 o 0 0 50 f 50 -75 x 75 y -100 x -75"
            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 50.0, -75.0), // f 50 -75
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -30));

            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 52000.0);
        }

        // Test bcut_simple/I4: Complex profile operation FORWARD FORWARD (variation 2)
        [Fact]
        public void ComplexProfileForwardForwardVariation3_I4()
        {
            // Create profile p1: same as I3 "profile p1 o 0 0 40 f 0 50 x 150 y 100 x -150"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0), // o 0 0 40
                new ProfileOperation(ProfileCmd.F, 0.0, 50.0), // f 0 50
                new ProfileOperation(ProfileCmd.X, 150.0), // x 150
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, -150.0), // x -150
            ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            // Create profile p2: same as I3 "profile p2 o 0 0 50 f 25 25 y 100 x 75 y -100"
            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 25.0, 25.0), // f 25 25
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
            ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            // Create profile p3: I4 "profile p3 p 0 0 -1 1 0 0 o 0 0 50 f 50 -75 y -100 x 75 y 100" with
            // prism height -3
            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 50.0, -75.0), // f 50 -75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
            ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -3)); // Note: -3, not -30!

            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 53000.0);
        }

        // Test bcut_simple/I5: Complex profile operation FORWARD REVERSED (variation 2)
        [Fact]
        public void ComplexProfileForwardReversedVariation2_I5()
        {
            // Create profile p1: same as I3/I4 "profile p1 o 0 0 40 f 0 50 x 150 y 100 x -150"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0), // o 0 0 40
                new ProfileOperation(ProfileCmd.F, 0.0, 50.0), // f 0 50
                new ProfileOperation(ProfileCmd.X, 150.0), // x 150
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, -150.0), // x -150
            ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            // Create profile p2: same as I3/I4 "profile p2 o 0 0 50 f 25 25 y 100 x 75 y -100"
            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 25.0, 25.0), // f 25 25
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
            ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            // Create profile p3: I5 "profile p3 o 0 0 50 f 50 75 x 75 y 100 x -75" (similar to H6/I1)
            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 50.0, 75.0), // f 50 75
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -30));

            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 52000.0);
        }

        // Test bcut_simple/I6: Complex profile operation FORWARD REVERSED (variation 3)
        [Fact]
        public void ComplexProfileForwardReversedVariation3_I6()
        {
            // Same profiles as I5 but different p3 - "profile p3 o 0 0 50 f 50 75 y 100 x 75 y -100"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0),
                new ProfileOperation(ProfileCmd.F, 0.0, 50.0),
                new ProfileOperation(ProfileCmd.X, 150.0),
                new ProfileOperation(ProfileCmd.Y, 100.0),
                new ProfileOperation(ProfileCmd.X, -150.0),
        ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0),
                new ProfileOperation(ProfileCmd.F, 25.0, 25.0),
                new ProfileOperation(ProfileCmd.Y, 100.0),
                new ProfileOperation(ProfileCmd.X, 75.0),
                new ProfileOperation(ProfileCmd.Y, -100.0),
        ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0),
                new ProfileOperation(ProfileCmd.F, 50.0, 75.0),
                new ProfileOperation(ProfileCmd.Y, 100.0),
                new ProfileOperation(ProfileCmd.X, 75.0),
                new ProfileOperation(ProfileCmd.Y, -100.0),
        ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -30));

            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 52000.0);
        }

        // Test bcut_simple/I7: Complex profile operation REVERSED FORWARD (variation 2)
        [Fact]
        public void ComplexProfileReversedForwardVariation2_I7()
        {
            // Create profile p1: "profile p1 p 0 0 -1 1 0 0 o 0 0 40 f 0 -50 y -100 x 150 y 100"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0), // o 0 0 40
                new ProfileOperation(ProfileCmd.F, 0.0, -50.0), // f 0 -50
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, 150.0), // x 150
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
            ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            // Create profile p2: "profile p2 p 0 0 -1 1 0 0 o 0 0 50 f 25 -25 x 75 y -100 x -75"
            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 25.0, -25.0), // f 25 -25
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            // Create profile p3: same as H8 "profile p3 p 0 0 -1 1 0 0 o 0 0 50 f 50 -75 x 75 y -100 x -75"
            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 50.0, -75.0), // f 50 -75
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -30));

            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 52000.0);
        }

        // Test bcut_simple/I8: Complex profile operation REVERSED FORWARD (variation 3)
        [Fact]
        public void ComplexProfileReversedForwardVariation3_I8()
        {
            // Create profile p1: "profile p1 p 0 0 -1 1 0 0 o 0 0 40 f 0 -50 y -100 x 150 y 100"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0), // o 0 0 40
                new ProfileOperation(ProfileCmd.F, 0.0, -50.0), // f 0 -50
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, 150.0), // x 150
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
            ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            // Create profile p2: "profile p2 p 0 0 -1 1 0 0 o 0 0 50 f 25 -25 x 75 y -100 x -75"
            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 25.0, -25.0), // f 25 -25
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            // Create profile p3: "profile p3 p 0 0 -1 1 0 0 o 0 0 50 f 50 -75 y -100 x 75 y 100"
            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 50.0, -75.0), // f 50 -75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
            ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -30));

            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 52000.0);
        }

        // Test bcut_simple/I9: Complex profile operation REVERSED REVERSED (variation 2)
        [Fact]
        public void ComplexProfileReversedReversedVariation2_I9()
        {
            // Create profile p1: "profile p1 p 0 0 -1 1 0 0 o 0 0 40 f 0 -50 y -100 x 150 y 100"
            Plane3D aPlane1 = new(new Point3D(0, 0, 40), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps1 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 40.0), // o 0 0 40
                new ProfileOperation(ProfileCmd.F, 0.0, -50.0), // f 0 -50
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, 150.0), // x 150
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
            ];
            Shape aProfile1 = CreateProfile(aPlane1, aProfileOps1);
            Shape aPrism1 = CreatePrism(aProfile1, new Vector3D(0, 0, -40));

            // Create profile p2: "profile p2 p 0 0 -1 1 0 0 o 0 0 50 f 25 -25 x 75 y -100 x -75"
            Plane3D aPlane2 = new(new Point3D(0, 0, 50), new Direction3D(Direction3D.D.Z));
            List<ProfileOperation> aProfileOps2 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 25.0, -25.0), // f 25 -25
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, -100.0), // y -100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile2 = CreateProfile(aPlane2, aProfileOps2);
            Shape aPrism2 = CreatePrism(aProfile2, new Vector3D(0, 0, -30));

            Cut aCutOp1 = new(aPrism1, aPrism2);
            Assert.True(aCutOp1.IsDone, "First cut operation failed");
            Shape aIntermediate = aCutOp1.Shape();

            // Create profile p3: "profile p3 o 0 0 50 f 50 75 x 75 y 100 x -75"
            List<ProfileOperation> aProfileOps3 =
            [
                new ProfileOperation(ProfileCmd.O, 0.0, 0.0, 50.0), // o 0 0 50
                new ProfileOperation(ProfileCmd.F, 50.0, 75.0), // f 50 75
                new ProfileOperation(ProfileCmd.X, 75.0), // x 75
                new ProfileOperation(ProfileCmd.Y, 100.0), // y 100
                new ProfileOperation(ProfileCmd.X, -75.0), // x -75
            ];
            Shape aProfile3 = CreateProfile(aPlane2, aProfileOps3);
            Shape aPrism3 = CreatePrism(aProfile3, new Vector3D(0, 0, -30));

            Shape aResult = PerformCut(aIntermediate, aPrism3);
            ValidateResult(aResult, 52000.0);
        }
    }

    //=============================================================================================
    // OCC825 Bug Tests
    //=============================================================================================

    public class BRepAlgoAPI_CutTest
    {
        // OCC825: BRepAlgoAPI_Cut of two spheres with a BSpline-defined half-space produces valid shapes.
        // Bug: BRepAlgoAPI_Cut failed on sphere and b-spline face.
        // Verifies both cut results are valid and have expected surface area (~5890.42).
        [Fact]
        public void Bug825_SphereHalfSpaceCut()
        {
            const double aSize = 50.0;
            NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
            aPoles[1, 1].SetCoord(-aSize, 0, -aSize);
            aPoles[1, 2].SetCoord(-aSize, 0, aSize);
            aPoles[2, 1].SetCoord(aSize, 0, -aSize);
            aPoles[2, 2].SetCoord(aSize, 0, aSize);

            BezierSurface aBezSurf = new(aPoles);
            BSplineSurface aBSpSurf = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(aBezSurf);
            MakeFace aFaceMaker = new(aBSpSurf, global::Kerf.Engine.Foundation.Precision.Confusion());
            Assert.True(aFaceMaker.IsDone);
            Face aFace = aFaceMaker.Face();

            MakeHalfSpace aHSpaceMaker = new(aFace, new Point3D(0, aSize, 0));
            Assert.True(aHSpaceMaker.IsDone);
            Shape aHsp = aHSpaceMaker.Solid();

            Shape aSph1 = new MakeSphere(new Point3D(0.0, 0.0, 0.0), 25.0).Shape();
            Shape aSph2 = new MakeSphere(new Point3D(0.0, 0.00001, 0.0), 25.0).Shape();

            Cut aCut1 = new(aSph1, aHsp);
            Assert.True(aCut1.IsDone, "BRepAlgoAPI_Cut failed for first sphere");
            Shape aResult1 = aCut1.Shape();
            Assert.False(aResult1.IsNull);

            Cut aCut2 = new(aSph2, aHsp);
            Assert.True(aCut2.IsDone, "BRepAlgoAPI_Cut failed for second sphere");
            Shape aResult2 = aCut2.Shape();
            Assert.False(aResult2.IsNull);

            Properties aProps1 = new();
            MassPropertiesBuilder.SurfaceProperties(aResult1, aProps1);
            Assert.Equal(5890.42, aProps1.Mass(), 5.89042);
            Assert.Equal(2, countSubShapes(aResult1, ShapeType.Face));
            Assert.Equal(1, countSubShapes(aResult1, ShapeType.Shell));
            Assert.Equal(1, countSubShapes(aResult1, ShapeType.Solid));
            Assert.Equal(ShapeType.Compound, aResult1.ShapeType());

            Properties aProps2 = new();
            MassPropertiesBuilder.SurfaceProperties(aResult2, aProps2);
            Assert.Equal(5890.42, aProps2.Mass(), 5.89042);
            Assert.Equal(2, countSubShapes(aResult2, ShapeType.Face));
            Assert.Equal(1, countSubShapes(aResult2, ShapeType.Shell));
            Assert.Equal(1, countSubShapes(aResult2, ShapeType.Solid));
            Assert.Equal(ShapeType.Compound, aResult2.ShapeType());
        }
    }
}
