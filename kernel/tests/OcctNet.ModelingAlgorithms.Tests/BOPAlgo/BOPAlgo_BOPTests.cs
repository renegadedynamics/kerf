// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.BOPAlgo_Operation;

namespace OcctNet.ModelingAlgorithms.Tests;

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

    private static double GetSurfaceArea(in TopoDS_Shape theShape)
    {
        if (theShape.IsNull())
        {
            return 0.0;
        }
        GProp_GProps aProps = new();
        BRepGProp.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static double GetVolume(in TopoDS_Shape theShape)
    {
        if (theShape.IsNull())
        {
            return 0.0;
        }
        GProp_GProps aProps = new();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static bool IsEmpty(in TopoDS_Shape theShape, double theTolerance) => GetSurfaceArea(theShape) <= theTolerance;

    private static TopoDS_Shape CreateUnitBox() => new BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape();

    private static TopoDS_Shape CreateBox(Point3D theCorner, double theX, double theY, double theZ) =>
        new BRepPrimAPI_MakeBox(theCorner, theX, theY, theZ).Shape();

    private static TopoDS_Shape CreateUnitSphere() => new BRepPrimAPI_MakeSphere(1.0).Shape();

    private static TopoDS_Shape CreateSphere(Point3D theCenter, double theRadius) => new BRepPrimAPI_MakeSphere(theCenter, theRadius).Shape();

    private static TopoDS_Shape CreateCylinder(double theRadius, double theHeight) => new BRepPrimAPI_MakeCylinder(theRadius, theHeight).Shape();

    private static TopoDS_Shape ConvertToNurbs(in TopoDS_Shape theShape)
    {
        BRepBuilderAPI_NurbsConvert aNurbsConverter = new(theShape);
        if (!aNurbsConverter.IsDone())
        {
            return new TopoDS_Shape(); // Return null shape on failure
        }
        return aNurbsConverter.Shape();
    }

    // ------------------------------------------------------------------------------- BOPAlgo_TestBase

    /// <summary>OCCT's BOPAlgo_TestBase fixture.</summary>
    public abstract class BOPAlgo_TestBase
    {
        protected double myTolerance = DefaultTolerance();
        protected double myFuzzyValue = DefaultFuzzyValue();
        protected BOPAlgo_PaveFiller? myPaveFiller;

        protected TopoDS_Shape PerformDirectBOP(in TopoDS_Shape theShape1, in TopoDS_Shape theShape2, BOPAlgo_Operation theOp)
        {
            BOPAlgo_BOP aBOP = new();

            aBOP.AddArgument(theShape1);
            aBOP.AddTool(theShape2);
            aBOP.SetOperation(theOp);
            aBOP.SetFuzzyValue(myFuzzyValue);
            aBOP.SetRunParallel(false);
            aBOP.SetNonDestructive(false);

            aBOP.Perform();

            Assert.False(aBOP.HasErrors(), "Direct BOP operation failed");
            return aBOP.Shape();
        }

        protected TopoDS_Shape PerformTwoStepBOP(in TopoDS_Shape theShape1, in TopoDS_Shape theShape2, BOPAlgo_Operation theOp)
        {
            PreparePaveFiller(theShape1, theShape2);
            return PerformBOPWithPaveFiller(theOp);
        }

        protected void PreparePaveFiller(in TopoDS_Shape theShape1, in TopoDS_Shape theShape2)
        {
            NCollection_List<TopoDS_Shape> aLC = new();
            aLC.Append(theShape1);
            aLC.Append(theShape2);

            myPaveFiller = new BOPAlgo_PaveFiller();

            myPaveFiller.SetArguments(aLC);
            myPaveFiller.SetFuzzyValue(myFuzzyValue);
            myPaveFiller.SetRunParallel(false);
            myPaveFiller.SetNonDestructive(false);

            myPaveFiller.Perform();
            Assert.False(myPaveFiller.HasErrors(), "PaveFiller preparation failed");
        }

        protected TopoDS_Shape PerformBOPWithPaveFiller(BOPAlgo_Operation theOp)
        {
            Assert.NotNull(myPaveFiller); // PaveFiller must be prepared first

            BOPAlgo_BOP aBOP = new();
            NCollection_List<TopoDS_Shape> aArguments = myPaveFiller.Arguments();
            Assert.Equal(2, aArguments.Extent()); // Wrong number of arguments

            TopoDS_Shape aS1 = aArguments.First();
            TopoDS_Shape aS2 = aArguments.Last();

            aBOP.AddArgument(aS1);
            aBOP.AddTool(aS2);
            aBOP.SetOperation(theOp);
            aBOP.SetRunParallel(false);

            aBOP.PerformWithFiller(myPaveFiller);

            Assert.False(aBOP.HasErrors(), "BOP operation with PaveFiller failed");
            return aBOP.Shape();
        }

        protected void ValidateResult(in TopoDS_Shape theResult,
                                      double theExpectedSurfaceArea = -1.0,
                                      double theExpectedVolume = -1.0,
                                      bool theExpectedEmpty = false)
        {
            if (theExpectedEmpty)
            {
                Assert.True(IsEmpty(theResult, myTolerance), "Result should be empty");
                return;
            }

            Assert.False(theResult.IsNull(), "Result shape should not be null");

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
            TopoDS_Shape aSphere = CreateUnitSphere();
            TopoDS_Shape aBox = CreateUnitBox();

            TopoDS_Shape aResult = PerformDirectBOP(aSphere, aBox, BOPAlgo_CUT);
            Assert.False(aResult.IsNull(), "Result shape should not be null");

            double aSurfaceArea = GetSurfaceArea(aResult);
            Assert.True(aSurfaceArea > 0.0, "Cut result should have positive surface area");
        }

        // Test direct fuse operation: bfuse result sphere box
        [Fact]
        public void DirectFuse_SpherePlusBox()
        {
            TopoDS_Shape aSphere = CreateUnitSphere();
            TopoDS_Shape aBox = CreateUnitBox();

            TopoDS_Shape aResult = PerformDirectBOP(aSphere, aBox, BOPAlgo_FUSE);
            Assert.False(aResult.IsNull(), "Result shape should not be null");

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
            TopoDS_Shape aBox1 = CreateBox(new Point3D(0, 0, 0), 2.0, 2.0, 2.0);
            TopoDS_Shape aBox2 = CreateBox(new Point3D(1, 1, 1), 2.0, 2.0, 2.0);

            TopoDS_Shape aResult = PerformDirectBOP(aBox1, aBox2, BOPAlgo_COMMON);
            ValidateResult(aResult, -1.0, 1.0); // Expected volume = 1.0
        }

        // Test direct tuc operation: btuc result box1 box2
        [Fact]
        public void DirectTUC_IdenticalBoxes()
        {
            TopoDS_Shape aBox1 = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            TopoDS_Shape aBox2 = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);

            TopoDS_Shape aResult = PerformDirectBOP(aBox1, aBox2, BOPAlgo_CUT21);
            ValidateResult(aResult, -1.0, -1.0, true); // Expected empty
        }

        // Test with NURBS converted shapes
        [Fact]
        public void DirectCut_NurbsBoxMinusBox()
        {
            TopoDS_Shape aBox1 = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            aBox1 = ConvertToNurbs(aBox1);
            Assert.False(aBox1.IsNull(), "Failed to convert to NURBS");

            TopoDS_Shape aBox2 = CreateBox(new Point3D(0, 1, 0), 1.0, 0.5, 1.0);

            TopoDS_Shape aResult = PerformDirectBOP(aBox1, aBox2, BOPAlgo_CUT);
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
            TopoDS_Shape aSphere = CreateUnitSphere();
            TopoDS_Shape aBox = CreateUnitBox();

            TopoDS_Shape aResult = PerformTwoStepBOP(aSphere, aBox, BOPAlgo_CUT);
            double aSurfaceArea = GetSurfaceArea(aResult);
            Assert.True(aSurfaceArea > 0.0, "Two-step cut result should have positive surface area");
        }

        // Test two-step fuse operation: bop sphere box; bopfuse result
        [Fact]
        public void TwoStepFuse_SpherePlusBox()
        {
            TopoDS_Shape aSphere = CreateUnitSphere();
            TopoDS_Shape aBox = CreateUnitBox();

            TopoDS_Shape aResult = PerformTwoStepBOP(aSphere, aBox, BOPAlgo_FUSE);
            Assert.False(aResult.IsNull(), "Result shape should not be null");

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
            TopoDS_Shape aBox1 = CreateBox(new Point3D(0, 0, 0), 2.0, 2.0, 2.0);
            TopoDS_Shape aBox2 = CreateBox(new Point3D(1, 1, 1), 2.0, 2.0, 2.0);

            TopoDS_Shape aResult = PerformTwoStepBOP(aBox1, aBox2, BOPAlgo_COMMON);
            ValidateResult(aResult, -1.0, 1.0); // Expected volume = 1.0
        }

        // Test two-step tuc operation: bop box1 box2; boptuc result
        [Fact]
        public void TwoStepTUC_IdenticalBoxes()
        {
            TopoDS_Shape aBox1 = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);
            TopoDS_Shape aBox2 = CreateBox(new Point3D(0, 0, 0), 1.0, 1.0, 1.0);

            TopoDS_Shape aResult = PerformTwoStepBOP(aBox1, aBox2, BOPAlgo_CUT21);
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
            TopoDS_Shape aSphere = CreateSphere(new Point3D(0, 0, 0), 1.5);
            TopoDS_Shape aCylinder = CreateCylinder(0.8, 3.0);
            TopoDS_Shape aBox = CreateBox(new Point3D(-0.5, -0.5, -0.5), 1.0, 1.0, 1.0);

            // First intersect sphere with cylinder
            TopoDS_Shape aIntermediate = PerformDirectBOP(aSphere, aCylinder, BOPAlgo_COMMON);
            Assert.False(aIntermediate.IsNull(), "Intermediate result should not be null");

            // Then fuse with box
            TopoDS_Shape aFinalResult = PerformDirectBOP(aIntermediate, aBox, BOPAlgo_FUSE);
            double aVolume = GetVolume(aFinalResult);
            Assert.True(aVolume > 0.0, "Complex operation result should have positive volume");
        }

        // Test comparison between direct and two-step operations
        [Fact]
        public void DirectVsTwoStepComparison()
        {
            TopoDS_Shape aSphere = CreateUnitSphere();
            TopoDS_Shape aBox = CreateUnitBox();

            // Perform direct operation
            TopoDS_Shape aDirectResult = PerformDirectBOP(aSphere, aBox, BOPAlgo_FUSE);

            // Perform two-step operation
            TopoDS_Shape aTwoStepResult = PerformTwoStepBOP(aSphere, aBox, BOPAlgo_FUSE);

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
            TopoDS_Shape aBox = CreateBox(new Point3D(0.0, 0.0, 0.0), 100.0, 100.0, 100.0);
            TopoDS_Shape aThin = CreateBox(new Point3D(-500.0, 25.0, -500.0), 1500.0, 1.0e-6, 1500.0);

            TopoDS_Shape aRes = PerformDirectBOP(aBox, aThin, BOPAlgo_CUT);
            double aBoxVol = GetVolume(aBox);
            double aOverlapV = 100.0 * 1.0e-6 * 100.0;
            Assert.False(aRes.IsNull());
            Assert.Equal(aBoxVol - aOverlapV, GetVolume(aRes), 1.0e-4);
        }

        [Fact]
        public void Fuse_AxisAlignedThinTool_AddsNonOverlappingSlice()
        {
            TopoDS_Shape aBox = CreateBox(new Point3D(0.0, 0.0, 0.0), 100.0, 100.0, 100.0);
            TopoDS_Shape aThin = CreateBox(new Point3D(-500.0, 25.0, -500.0), 1500.0, 1.0e-6, 1500.0);

            TopoDS_Shape aRes = PerformDirectBOP(aBox, aThin, BOPAlgo_FUSE);
            double aBoxVol = GetVolume(aBox);
            double aThinVol = 1500.0 * 1.0e-6 * 1500.0;
            double aOverlapV = 100.0 * 1.0e-6 * 100.0;
            Assert.False(aRes.IsNull());
            Assert.Equal(aBoxVol + aThinVol - aOverlapV, GetVolume(aRes), 1.0e-4);
        }

        [Fact]
        public void Cut_LegitimateThinSlab_NotTreatedAsEmpty()
        {
            TopoDS_Shape aBox = CreateBox(new Point3D(0.0, 0.0, 0.0), 100.0, 100.0, 100.0);
            TopoDS_Shape aSlab = CreateBox(new Point3D(0.0, 0.0, 50.0), 100.0, 100.0, 1.0);

            TopoDS_Shape aRes = PerformDirectBOP(aBox, aSlab, BOPAlgo_CUT);
            Assert.False(aRes.IsNull());
            Assert.Equal(GetVolume(aBox) - GetVolume(aSlab), GetVolume(aRes), myTolerance);
        }

        [Fact]
        public void Common_SolidAndPlanarFace_Unaffected()
        {
            TopoDS_Shape aBox = CreateBox(new Point3D(0.0, 0.0, 0.0), 100.0, 100.0, 100.0);
            Plane3D aPln = new(new Point3D(0.0, 0.0, 50.0), new Direction3D(0.0, 0.0, 1.0));
            BRepBuilderAPI_MakeFace aFaceMaker = new(aPln, -200.0, 200.0, -200.0, 200.0);
            Assert.True(aFaceMaker.IsDone());
            TopoDS_Shape aFace = aFaceMaker.Face();

            TopoDS_Shape aRes = PerformDirectBOP(aBox, aFace, BOPAlgo_COMMON);
            Assert.False(aRes.IsNull());
            Assert.Equal(1.0e4, GetSurfaceArea(aRes), 1.0);
        }

        [Fact]
        public void Cut_BySemiInfinitePrism_Unaffected()
        {
            TopoDS_Shape aBox = CreateBox(new Point3D(0.0, -1.0, -1.0), 2.0, 2.0, 2.0);
            Plane3D aPln = new(new Point3D(-0.5, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
            BRepBuilderAPI_MakeFace aFaceMaker = new(aPln, -0.5, 0.5, -0.5, 0.5);
            Assert.True(aFaceMaker.IsDone());
            BRepPrimAPI_MakePrism aPrismMaker = new(aFaceMaker.Face(), new Direction3D(1.0, 0.0, 0.0), true);
            Assert.True(aPrismMaker.IsDone());

            TopoDS_Shape aRes = PerformDirectBOP(aBox, aPrismMaker.Shape(), BOPAlgo_CUT);
            Assert.False(aRes.IsNull());
            Assert.True(GetVolume(aRes) > 1.0);
        }

        [Fact]
        public void Common_SolidAndHalfspace_Unaffected()
        {
            TopoDS_Shape aBox = CreateBox(new Point3D(0.0, 0.0, -30.0), 150.0, 200.0, 200.0);
            Plane3D aPln = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
            BRepBuilderAPI_MakeFace aFaceMaker = new(aPln, -250.0, 250.0, -250.0, 250.0);
            Assert.True(aFaceMaker.IsDone());
            BRepPrimAPI_MakeHalfSpace aHSMaker = new(aFaceMaker.Face(), new Point3D(0.0, 0.0, -100.0));
            TopoDS_Shape aHalfSpace = aHSMaker.Solid();

            TopoDS_Shape aRes = PerformDirectBOP(aBox, aHalfSpace, BOPAlgo_COMMON);
            Assert.False(aRes.IsNull());
            Assert.True(GetVolume(aRes) > 1.0);
        }
    }
}
