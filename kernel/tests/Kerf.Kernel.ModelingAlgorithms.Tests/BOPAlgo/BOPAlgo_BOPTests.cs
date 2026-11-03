// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Booleans.Operation;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// OCCT's GTests of BOPAlgo_BOP (src/ModelingAlgorithms/TKBO/GTests/BOPAlgo_BOP_Test.cxx), with the helpers of
/// BOPTest_Utilities.pxx and BOPAlgo_TestBase that they use. OCCT's fixture classes (DirectOperations,
/// TwoStepOperations, ComplexOperations, DegenerateTool) are nested classes, so the tests keep their names.
/// </summary>
public static class BOPAlgo_BOPTests
{
    // ------------------------------------------------------------------------------- BOPTest_Utilities

    private static double DefaultTolerance() => 1.0e-6;

    private static double DefaultFuzzyValue() => 1.0e-8;

    private static double GetSurfaceArea(in Shape theShape)
    {
        if (theShape.IsNull)
        {
            return 0.0;
        }
        Properties aProps = new();
        MassPropertiesBuilder.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static double GetVolume(in Shape theShape)
    {
        if (theShape.IsNull)
        {
            return 0.0;
        }
        Properties aProps = new();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static bool IsEmpty(in Shape theShape, double theTolerance) => GetSurfaceArea(theShape) <= theTolerance;

    private static Shape CreateUnitBox() => new MakeBox(1.0, 1.0, 1.0).Shape();

    private static Shape CreateBox(Point3D theCorner, double theX, double theY, double theZ) =>
        new MakeBox(theCorner, theX, theY, theZ).Shape();

    private static Shape CreateUnitSphere() => new MakeSphere(1.0).Shape();

    private static Shape CreateSphere(Point3D theCenter, double theRadius) => new MakeSphere(theCenter, theRadius).Shape();

    private static Shape CreateCylinder(double theRadius, double theHeight) => new ApiMakeCylinder(theRadius, theHeight).Shape();

    private static Shape ConvertToNurbs(in Shape theShape)
    {
        NurbsConvert aNurbsConverter = new(theShape);
        if (!aNurbsConverter.IsDone)
        {
            return new Shape(); // Return null shape on failure
        }
        return aNurbsConverter.Shape();
    }

    // ------------------------------------------------------------------------------- BOPAlgo_TestBase

    /// <summary>OCCT's BOPAlgo_TestBase fixture.</summary>
    public abstract class BOPAlgo_TestBase
    {
        protected double myTolerance = DefaultTolerance();
        protected double myFuzzyValue = DefaultFuzzyValue();
        protected PaveFiller? myPaveFiller;

        protected Shape PerformDirectBOP(in Shape theShape1, in Shape theShape2, Operation theOp)
        {
            Bop aBOP = new();

            aBOP.AddArgument(theShape1);
            aBOP.AddTool(theShape2);
            aBOP.SetOperation(theOp);
            aBOP.SetFuzzyValue(myFuzzyValue);
            aBOP.SetRunParallel(false);
            aBOP.SetNonDestructive(false);

            aBOP.Perform();

            Assert.False(aBOP.HasErrors, "Direct BOP operation failed");
            return aBOP.Shape();
        }

        protected Shape PerformTwoStepBOP(in Shape theShape1, in Shape theShape2, Operation theOp)
        {
            PreparePaveFiller(theShape1, theShape2);
            return PerformBOPWithPaveFiller(theOp);
        }

        protected void PreparePaveFiller(in Shape theShape1, in Shape theShape2)
        {
            NCollection_List<Shape> aLC = new();
            aLC.Append(theShape1);
            aLC.Append(theShape2);

            myPaveFiller = new PaveFiller();

            myPaveFiller.SetArguments(aLC);
            myPaveFiller.SetFuzzyValue(myFuzzyValue);
            myPaveFiller.SetRunParallel(false);
            myPaveFiller.SetNonDestructive(false);

            myPaveFiller.Perform();
            Assert.False(myPaveFiller.HasErrors, "PaveFiller preparation failed");
        }

        protected Shape PerformBOPWithPaveFiller(Operation theOp)
        {
            Assert.NotNull(myPaveFiller); // PaveFiller must be prepared first

            Bop aBOP = new();
            NCollection_List<Shape> aArguments = myPaveFiller.Arguments();
            Assert.Equal(2, aArguments.Extent()); // Wrong number of arguments

            Shape aS1 = aArguments.First();
            Shape aS2 = aArguments.Last();

            aBOP.AddArgument(aS1);
            aBOP.AddTool(aS2);
            aBOP.SetOperation(theOp);
            aBOP.SetRunParallel(false);

            aBOP.PerformWithFiller(myPaveFiller);

            Assert.False(aBOP.HasErrors, "BOP operation with PaveFiller failed");
            return aBOP.Shape();
        }

        protected void ValidateResult(in Shape theResult,
                                      double theExpectedSurfaceArea = -1.0,
                                      double theExpectedVolume = -1.0,
                                      bool theExpectedEmpty = false)
        {
            if (theExpectedEmpty)
            {
                Assert.True(IsEmpty(theResult, myTolerance), "Result should be empty");
                return;
            }

            Assert.False(theResult.IsNull, "Result shape should not be null");

            if (theExpectedSurfaceArea >= 0.0)
            {
                double aSurfaceArea = GetSurfaceArea(theResult);
                Assert.Equal(theExpectedSurfaceArea, aSurfaceArea, 5000.0); // Surface area mismatch
            }

            if (theExpectedVolume >= 0.0)
            {
                double aVolume = GetVolume(theResult);
                Assert.Equal(theExpectedVolume, aVolume, myTolerance); // Volume mismatch
            }
        }
    }

    // ------------------------------------------------------------------------------- Direct BOP operations
    // (equivalent to bcut, bfuse, bcommon, btuc commands)

    public class BOPAlgo_DirectOperationsTest : BOPAlgo_TestBase
    {
        // Test direct cut operation: bcut result sphere box
        [Fact]
        public void DirectCut_SphereMinusBox()
        {
            Shape aSphere = CreateUnitSphere();
            Shape aBox = CreateUnitBox();

            Shape aResult = PerformDirectBOP(aSphere, aBox, Operation.Cut);
            Assert.False(aResult.IsNull, "Result shape should not be null");

            double aSurfaceArea = GetSurfaceArea(aResult);
            Assert.True(aSurfaceArea > 0.0, "Cut result should have positive surface area");
        }

        // Test direct fuse operation: bfuse result sphere box
        [Fact]
        public void DirectFuse_SpherePlusBox()
        {
            Shape aSphere = CreateUnitSphere();
            Shape aBox = CreateUnitBox();

            Shape aResult = PerformDirectBOP(aSphere, aBox, Operation.Fuse);
            Assert.False(aResult.IsNull, "Result shape should not be null");

            double aVolume = GetVolume(aResult);
            double aSphereVolume = GetVolume(aSphere);
            double aBoxVolume = GetVolume(aBox);
            Assert.True(aVolume > aSphereVolume, "Fuse result should be larger than sphere alone");
            Assert.True(aVolume > aBoxVolume, "Fuse result should be larger than box alone");
        }

        // Test direct common operation: bcommon result box1 box2
        [Fact]
        public void DirectCommon_OverlappingBoxes()
        {
            Shape aBox1 = CreateBox(new Point3D(0, 0, 0), 2.0, 2.0, 2.0);
            Shape aBox2 = CreateBox(new Point3D(1, 1, 1), 2.0, 2.0, 2.0);

            Shape aResult = PerformDirectBOP(aBox1, aBox2, Operation.Common);
            ValidateResult(aResult, -1.0, 1.0); // Expected volume = 1.0
        }

        // Test direct tuc operation: btuc result box1 box2
        [Fact]
        public void DirectTUC_IdenticalBoxes()
        {
            Shape aBox1 = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            Shape aBox2 = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);

            Shape aResult = PerformDirectBOP(aBox1, aBox2, Operation.CUT21);
            ValidateResult(aResult, -1.0, -1.0, true); // Expected empty
        }

        // Test with NURBS converted shapes
        [Fact]
        public void DirectCut_NurbsBoxMinusBox()
        {
            Shape aBox1 = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull, "Failed to convert to NURBS");

            Shape aBox2 = CreateBox(new Point3D(0, 1, 0), 1.0, 0.5, 1.0);

            Shape aResult = PerformDirectBOP(aBox1, aBox2, Operation.Cut);
            double aSurfaceArea = GetSurfaceArea(aResult);
            Assert.True(aSurfaceArea > 0.0, "NURBS cut result should have positive surface area");
        }
    }

    // ------------------------------------------------------------------------------- Two-step BOP operations
    // (equivalent to bop + bopXXX commands)

    public class BOPAlgo_TwoStepOperationsTest : BOPAlgo_TestBase
    {
        // Test two-step cut operation: bop sphere box; bopcut result
        [Fact]
        public void TwoStepCut_SphereMinusBox()
        {
            Shape aSphere = CreateUnitSphere();
            Shape aBox = CreateUnitBox();

            Shape aResult = PerformTwoStepBOP(aSphere, aBox, Operation.Cut);
            double aSurfaceArea = GetSurfaceArea(aResult);
            Assert.True(aSurfaceArea > 0.0, "Two-step cut result should have positive surface area");
        }

        // Test two-step fuse operation: bop sphere box; bopfuse result
        [Fact]
        public void TwoStepFuse_SpherePlusBox()
        {
            Shape aSphere = CreateUnitSphere();
            Shape aBox = CreateUnitBox();

            Shape aResult = PerformTwoStepBOP(aSphere, aBox, Operation.Fuse);
            Assert.False(aResult.IsNull, "Result shape should not be null");

            double aVolume = GetVolume(aResult);
            double aSphereVolume = GetVolume(aSphere);
            double aBoxVolume = GetVolume(aBox);
            Assert.True(aVolume > aSphereVolume, "Two-step fuse result should be larger than sphere alone");
            Assert.True(aVolume > aBoxVolume, "Two-step fuse result should be larger than box alone");
        }

        // Test two-step common operation: bop box1 box2; bopcommon result
        [Fact]
        public void TwoStepCommon_OverlappingBoxes()
        {
            Shape aBox1 = CreateBox(new Point3D(0, 0, 0), 2.0, 2.0, 2.0);
            Shape aBox2 = CreateBox(new Point3D(1, 1, 1), 2.0, 2.0, 2.0);

            Shape aResult = PerformTwoStepBOP(aBox1, aBox2, Operation.Common);
            ValidateResult(aResult, -1.0, 1.0); // Expected volume = 1.0
        }

        // Test two-step tuc operation: bop box1 box2; boptuc result
        [Fact]
        public void TwoStepTUC_IdenticalBoxes()
        {
            Shape aBox1 = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            Shape aBox2 = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);

            Shape aResult = PerformTwoStepBOP(aBox1, aBox2, Operation.CUT21);
            ValidateResult(aResult, -1.0, -1.0, true); // Expected empty
        }
    }

    // ------------------------------------------------------------------------------- Complex operations

    public class BOPAlgo_ComplexOperationsTest : BOPAlgo_TestBase
    {
        // Test multiple intersecting primitives
        [Fact]
        public void MultipleIntersectingPrimitives()
        {
            Shape aSphere = CreateSphere(new Point3D(0, 0, 0), 1.5);
            Shape aCylinder = CreateCylinder(0.8, 3.0);
            Shape aBox = CreateBox(new Point3D(-0.5, -0.5, -0.5), 1.0, 1.0, 1.0);

            // First intersect sphere with cylinder
            Shape aIntermediate = PerformDirectBOP(aSphere, aCylinder, Operation.Common);
            Assert.False(aIntermediate.IsNull, "Intermediate result should not be null");

            // Then fuse with box
            Shape aFinalResult = PerformDirectBOP(aIntermediate, aBox, Operation.Fuse);
            double aVolume = GetVolume(aFinalResult);
            Assert.True(aVolume > 0.0, "Complex operation result should have positive volume");
        }

        // Test comparison between direct and two-step operations
        [Fact]
        public void DirectVsTwoStepComparison()
        {
            Shape aSphere = CreateUnitSphere();
            Shape aBox = CreateUnitBox();

            // Perform direct operation
            Shape aDirectResult = PerformDirectBOP(aSphere, aBox, Operation.Fuse);

            // Perform two-step operation
            Shape aTwoStepResult = PerformTwoStepBOP(aSphere, aBox, Operation.Fuse);

            // Results should be equivalent
            double aDirectVolume = GetVolume(aDirectResult);
            double aTwoStepVolume = GetVolume(aTwoStepResult);

            Assert.Equal(aDirectVolume, aTwoStepVolume, myTolerance);
        }
    }

    // ------------------------------------------------------------------------------- Degenerate thin-tool tests

    public class BOPAlgo_DegenerateToolTest : BOPAlgo_TestBase
    {
        [Fact]
        public void Cut_AxisAlignedThinTool_NearlyPreservesBoxVolume()
        {
            Shape aBox = CreateBox(new Point3D(0.0, 0.0, 0.0), 100.0, 100.0, 100.0);
            Shape aThin = CreateBox(new Point3D(-500.0, 25.0, -500.0), 1500.0, 1.0e-6, 1500.0);

            Shape aRes = PerformDirectBOP(aBox, aThin, Operation.Cut);
            double aBoxVol = GetVolume(aBox);
            double aOverlapV = 100.0 * 1.0e-6 * 100.0;
            Assert.False(aRes.IsNull);
            Assert.Equal(aBoxVol - aOverlapV, GetVolume(aRes), 1.0e-4);
        }

        [Fact]
        public void Fuse_AxisAlignedThinTool_AddsNonOverlappingSlice()
        {
            Shape aBox = CreateBox(new Point3D(0.0, 0.0, 0.0), 100.0, 100.0, 100.0);
            Shape aThin = CreateBox(new Point3D(-500.0, 25.0, -500.0), 1500.0, 1.0e-6, 1500.0);

            Shape aRes = PerformDirectBOP(aBox, aThin, Operation.Fuse);
            double aBoxVol = GetVolume(aBox);
            double aThinVol = 1500.0 * 1.0e-6 * 1500.0;
            double aOverlapV = 100.0 * 1.0e-6 * 100.0;
            Assert.False(aRes.IsNull);
            Assert.Equal(aBoxVol + aThinVol - aOverlapV, GetVolume(aRes), 1.0e-4);
        }

        [Fact]
        public void Cut_LegitimateThinSlab_NotTreatedAsEmpty()
        {
            Shape aBox = CreateBox(new Point3D(0.0, 0.0, 0.0), 100.0, 100.0, 100.0);
            Shape aSlab = CreateBox(new Point3D(0.0, 0.0, 50.0), 100.0, 100.0, 1.0);

            Shape aRes = PerformDirectBOP(aBox, aSlab, Operation.Cut);
            Assert.False(aRes.IsNull);
            Assert.Equal(GetVolume(aBox) - GetVolume(aSlab), GetVolume(aRes), myTolerance);
        }

        [Fact]
        public void Common_SolidAndPlanarFace_Unaffected()
        {
            Shape aBox = CreateBox(new Point3D(0.0, 0.0, 0.0), 100.0, 100.0, 100.0);
            Plane3D aPln = new(new Point3D(0.0, 0.0, 50.0), new Direction3D(0.0, 0.0, 1.0));
            MakeFace aFaceMaker = new(aPln, -200.0, 200.0, -200.0, 200.0);
            Assert.True(aFaceMaker.IsDone);
            Shape aFace = aFaceMaker.Face();

            Shape aRes = PerformDirectBOP(aBox, aFace, Operation.Common);
            Assert.False(aRes.IsNull);
            Assert.Equal(1.0e4, GetSurfaceArea(aRes), 1.0);
        }

        [Fact]
        public void Cut_BySemiInfinitePrism_Unaffected()
        {
            Shape aBox = CreateBox(new Point3D(0.0, -1.0, -1.0), 2.0, 2.0, 2.0);
            Plane3D aPln = new(new Point3D(-0.5, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
            MakeFace aFaceMaker = new(aPln, -0.5, 0.5, -0.5, 0.5);
            Assert.True(aFaceMaker.IsDone);
            ApiMakePrism aPrismMaker = new(aFaceMaker.Face(), new Direction3D(1.0, 0.0, 0.0), true);
            Assert.True(aPrismMaker.IsDone);

            Shape aRes = PerformDirectBOP(aBox, aPrismMaker.Shape(), Operation.Cut);
            Assert.False(aRes.IsNull);
            Assert.True(GetVolume(aRes) > 1.0);
        }

        [Fact]
        public void Common_SolidAndHalfspace_Unaffected()
        {
            Shape aBox = CreateBox(new Point3D(0.0, 0.0, -30.0), 150.0, 200.0, 200.0);
            Plane3D aPln = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
            MakeFace aFaceMaker = new(aPln, -250.0, 250.0, -250.0, 250.0);
            Assert.True(aFaceMaker.IsDone);
            MakeHalfSpace aHSMaker = new(aFaceMaker.Face(), new Point3D(0.0, 0.0, -100.0));
            Shape aHalfSpace = aHSMaker.Solid();

            Shape aRes = PerformDirectBOP(aBox, aHalfSpace, Operation.Common);
            Assert.False(aRes.IsNull);
            Assert.True(GetVolume(aRes) > 1.0);
        }
    }
}
