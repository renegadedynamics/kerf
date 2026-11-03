// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of OCCT's GTest src/ModelingAlgorithms/TKHelix/GTests/HelixBRep_BuilderHelix_Integration_Test.cxx (HelixBRepTest),
// and the port's own tests of the builder's frozen result.

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

public class HelixBRep_BuilderHelix_IntegrationTests
{
    // Standard axis aligned with Z
    private readonly CoordinateSystem myAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
    private const double myTolerance = 1.0e-4;

    // Helper method to validate helix wire properties
    private static void ValidateHelixWire(in Wire theWire, double theExpectedLength, double theTolerance = 0.05)
    {
        Assert.False(theWire.IsNull);
        Assert.Equal(ShapeType.Wire, theWire.ShapeType());
        // Check that wire has edges
        int aNbEdges = HelixBRep_BuilderHelixTests.CountShapes(theWire, ShapeType.Edge);
        Assert.True(aNbEdges > 0);
        // Compute wire length
        double aWireLength = HelixBRep_BuilderHelixTests.ComputeWireLength(theWire);
        if (theExpectedLength > 0)
        {
            Assert.Equal(theExpectedLength, aWireLength, theExpectedLength * theTolerance);
        }
    }

    // Test Case A1 - Pure cylindrical helix (TCL test A1)
    [Fact]
    public void TCL_Test_A1_PureCylindricalHelix()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        // Parameters from TCL test A1: helix, D1 = 100, 1 part, pitch = 5, number of turns, PFi=0
        NCollection_Array1<double> aHeights = new(1, 1);
        aHeights[1] = 100.0; // H1 = 100
        NCollection_Array1<double> aPitches = new(1, 1);
        aPitches[1] = 5.0; // P1 = 5 (number of turns)
        NCollection_Array1<bool> aIsPitches = new(1, 1);
        aIsPitches[1] = false; // PF1 = 0 means number of turns
        aBuilder.SetParameters(myAxis, 100.0, aHeights, aPitches, aIsPitches);
        aBuilder.SetApproxParameters(myTolerance, 8, C1);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        Shape aHelixWire = aBuilder.Shape();
        Assert.False(aHelixWire.IsNull);
        Assert.Equal(ShapeType.Wire, aHelixWire.ShapeType());
        // Calculate expected helix length
        double aCircumference = M_PI * 100.0; // D1 = 100
        double aTurns = 100.0 / 20.0;         // Height/Pitch for number of turns mode (20 = 100/5)
        double aExpectedLength = aTurns * Math.Sqrt(aCircumference * aCircumference + 20.0 * 20.0);
        ValidateHelixWire(ShapeCasts.Wire(aHelixWire), aExpectedLength);
    }

    // Test Case B1 - Pure cylindrical helix with comphelix (TCL test B1)
    [Fact]
    public void TCL_Test_B1_CompositeCylindricalHelix()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        // Parameters from TCL test B1: comphelix, D1 = D2 = 100, 1 part, pitch = 20, PFi=1
        NCollection_Array1<double> aDiams = new(1, 2);
        aDiams[1] = 100.0; // D1
        aDiams[2] = 100.0; // D2
        NCollection_Array1<double> aHeights = new(1, 1);
        aHeights[1] = 100.0; // H1
        NCollection_Array1<double> aPitches = new(1, 1);
        aPitches[1] = 20.0; // P1
        NCollection_Array1<bool> aIsPitches = new(1, 1);
        aIsPitches[1] = true; // PF1 = 1 means pitch
        aBuilder.SetParameters(myAxis, aDiams, aHeights, aPitches, aIsPitches);
        aBuilder.SetApproxParameters(myTolerance, 8, C1);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        Shape aHelixWire = aBuilder.Shape();
        Assert.False(aHelixWire.IsNull);
        // Calculate expected helix length - same as A1 since geometry is identical
        double aCircumference = M_PI * 100.0;
        double aTurns = 100.0 / 20.0; // Height/Pitch
        double aExpectedLength = aTurns * Math.Sqrt(aCircumference * aCircumference + 20.0 * 20.0);
        ValidateHelixWire(ShapeCasts.Wire(aHelixWire), aExpectedLength);
    }

    // Test Case C1 - Spiral helix (TCL test C1)
    [Fact]
    public void TCL_Test_C1_SpiralHelix()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        // Parameters from TCL test C1: spiral, D1 = 100, D2 = 20, 3 parts
        NCollection_Array1<double> aHeights = new(1, 3);
        aHeights[1] = 20.0; // H1
        aHeights[2] = 60.0; // H2
        aHeights[3] = 20.0; // H3
        NCollection_Array1<double> aPitches = new(1, 3);
        aPitches[1] = 2.0; // P1 (number of turns)
        aPitches[2] = 6.0; // P2 (number of turns)
        aPitches[3] = 2.0; // P3 (number of turns)
        NCollection_Array1<bool> aIsPitches = new(1, 3);
        aIsPitches[1] = false; // PF1 = 0
        aIsPitches[2] = false; // PF2 = 0
        aIsPitches[3] = false; // PF3 = 0
        aBuilder.SetParameters(myAxis, 100.0, 20.0, aHeights, aPitches, aIsPitches);
        aBuilder.SetApproxParameters(myTolerance, 8, C1);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        Shape aHelixWire = aBuilder.Shape();
        Assert.False(aHelixWire.IsNull);
        // For spiral helix, length calculation is more complex due to changing diameter
        // Just validate basic properties
        ValidateHelixWire(ShapeCasts.Wire(aHelixWire), 0); // Don't check exact length for complex spiral
    }

    // Test Case F1 - Using helix2 interface (TCL test F1)
    [Fact]
    public void TCL_Test_F1_Helix2Interface()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        // Parameters from TCL test F1: comphelix2 with turns interface
        NCollection_Array1<double> aPitches = new(1, 3);
        aPitches[1] = 10.0; // P1
        aPitches[2] = 10.0; // P2
        aPitches[3] = 10.0; // P3
        NCollection_Array1<double> aNbTurns = new(1, 3);
        aNbTurns[1] = 2.0; // N1
        aNbTurns[2] = 6.0; // N2
        aNbTurns[3] = 2.0; // N3
        aBuilder.SetParameters(myAxis, 100.0, aPitches, aNbTurns);
        aBuilder.SetApproxParameters(myTolerance, 8, C1);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        Shape aHelixWire = aBuilder.Shape();
        Assert.False(aHelixWire.IsNull);
        // Calculate expected length for cylindrical helix with multiple parts
        double aCircumference = M_PI * 100.0;
        double aTotalTurns = 2.0 + 6.0 + 2.0; // Sum of turns
        double aExpectedLength = aTotalTurns * Math.Sqrt(aCircumference * aCircumference + 10.0 * 10.0);
        ValidateHelixWire(ShapeCasts.Wire(aHelixWire), aExpectedLength);
    }

    // Test Case E1 - Complex customer example (TCL test E1)
    [Fact]
    public void TCL_Test_E1_CustomerExample()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        // Parameters from TCL test E1: Customer example with 5 parts
        NCollection_Array1<double> aDiams = new(1, 6);
        aDiams[1] = 150.0; // D1
        aDiams[2] = 150.0; // D2
        aDiams[3] = 150.0; // D3
        aDiams[4] = 123.0; // D4
        aDiams[5] = 123.0; // D5
        aDiams[6] = 123.0; // D6
        // Calculate heights from pitches and turns
        NCollection_Array1<double> aHeights = new(1, 5);
        aHeights[1] = 0.75 * 13.0; // N1 * P1
        aHeights[2] = 2.1 * 64.0;  // N2 * P2
        aHeights[3] = 2.25 * 50.0; // N3 * P3
        aHeights[4] = 2.5 * 45.0;  // N4 * P4
        aHeights[5] = 0.75 * 13.0; // N5 * P5
        NCollection_Array1<double> aPitches = new(1, 5);
        aPitches[1] = 0.75; // N1 (number of turns)
        aPitches[2] = 2.1;  // N2
        aPitches[3] = 2.25; // N3
        aPitches[4] = 2.5;  // N4
        aPitches[5] = 0.75; // N5
        NCollection_Array1<bool> aIsPitches = new(1, 5);
        aIsPitches[1] = false; // Number of turns mode
        aIsPitches[2] = false;
        aIsPitches[3] = false;
        aIsPitches[4] = false;
        aIsPitches[5] = false;
        aBuilder.SetParameters(myAxis, aDiams, aHeights, aPitches, aIsPitches);
        aBuilder.SetApproxParameters(myTolerance, 8, C1);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        Shape aHelixWire = aBuilder.Shape();
        Assert.False(aHelixWire.IsNull);
        // Complex customer example - just validate basic properties
        ValidateHelixWire(ShapeCasts.Wire(aHelixWire), 0); // Don't check exact length for complex multi-part helix
    }

    // Test error conditions
    [Fact]
    public void ErrorConditions_InvalidDimensions()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        // Test mismatched array dimensions
        NCollection_Array1<double> aDiams = new(1, 3);   // 3 elements = 2 parts
        NCollection_Array1<double> aHeights = new(1, 1); // 1 element - should cause error (need 2 for 2 parts)
        NCollection_Array1<double> aPitches = new(1, 1); // 1 element - should cause error (need 2 for 2 parts)
        NCollection_Array1<bool> aIsPitches = new(1, 1); // 1 element - should cause error (need 2 for 2 parts)
        aDiams[1] = 100.0;
        aDiams[2] = 100.0;
        aDiams[3] = 100.0;
        aHeights[1] = 50.0;
        aPitches[1] = 10.0;
        aIsPitches[1] = true;
        Assert.ThrowsAny<ConstructionErrorException>(() => aBuilder.SetParameters(myAxis, aDiams, aHeights, aPitches, aIsPitches));
    }

    [Fact]
    public void ErrorConditions_TurnsInterface()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        // Test mismatched dimensions in turns interface
        NCollection_Array1<double> aPitches = new(1, 2);
        NCollection_Array1<double> aNbTurns = new(1, 1); // Mismatched size
        aPitches[1] = 10.0;
        aPitches[2] = 10.0;
        aNbTurns[1] = 5.0;
        Assert.ThrowsAny<ConstructionErrorException>(() => aBuilder.SetParameters(myAxis, 100.0, aPitches, aNbTurns));
    }

    // Test tolerance and approximation quality
    [Fact]
    public void ApproximationQuality()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        NCollection_Array1<double> aHeights = new(1, 1);
        aHeights[1] = 50.0;
        NCollection_Array1<double> aPitches = new(1, 1);
        aPitches[1] = 10.0;
        NCollection_Array1<bool> aIsPitches = new(1, 1);
        aIsPitches[1] = true;
        aBuilder.SetParameters(myAxis, 50.0, aHeights, aPitches, aIsPitches);
        // Test different tolerance levels
        double aTightTolerance = 1.0e-6;
        aBuilder.SetApproxParameters(aTightTolerance, 8, C2);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        double aToleranceReached = aBuilder.ToleranceReached();
        Assert.True(aToleranceReached > 0.0);
        // For complex geometry, tolerance reached might be higher than requested
        // but should be reasonable
        Assert.True(aToleranceReached < aTightTolerance * 1000);
    }

    // Test parameter validation
    [Fact]
    public void ParameterValidation()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        NCollection_Array1<double> aHeights = new(1, 1);
        NCollection_Array1<double> aPitches = new(1, 1);
        NCollection_Array1<bool> aIsPitches = new(1, 1);
        // Test very small height - should cause error during Perform()
        aHeights[1] = 1.0e-8;
        aPitches[1] = 10.0;
        aIsPitches[1] = true;
        aBuilder.SetParameters(myAxis, 50.0, aHeights, aPitches, aIsPitches);
        aBuilder.SetApproxParameters(myTolerance, 8, C1);
        aBuilder.Perform();
        Assert.NotEqual(0, aBuilder.ErrorStatus()); // Should have error status 12 (invalid height)
        // Test zero pitch - should cause error
        aHeights[1] = 50.0;
        aPitches[1] = 0.0;
        aBuilder.SetParameters(myAxis, 50.0, aHeights, aPitches, aIsPitches);
        aBuilder.Perform();
        Assert.NotEqual(0, aBuilder.ErrorStatus()); // Should have error status 11 (invalid pitch)
    }

    // Test wire continuity across multiple parts
    [Fact]
    public void MultiPartContinuity()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        // 2-part helix with different parameters
        NCollection_Array1<double> aDiams = new(1, 3);
        aDiams[1] = 80.0;
        aDiams[2] = 60.0;
        aDiams[3] = 40.0;
        NCollection_Array1<double> aHeights = new(1, 2);
        aHeights[1] = 40.0;
        aHeights[2] = 40.0;
        NCollection_Array1<double> aPitches = new(1, 2);
        aPitches[1] = 8.0;
        aPitches[2] = 12.0;
        NCollection_Array1<bool> aIsPitches = new(1, 2);
        aIsPitches[1] = true;
        aIsPitches[2] = true;
        aBuilder.SetParameters(myAxis, aDiams, aHeights, aPitches, aIsPitches);
        aBuilder.SetApproxParameters(myTolerance, 8, C1);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        Shape aHelixWire = aBuilder.Shape();
        Assert.False(aHelixWire.IsNull);
        Assert.Equal(ShapeType.Wire, aHelixWire.ShapeType());
        // Check that wire has multiple edges
        int aNbEdges = HelixBRep_BuilderHelixTests.CountShapes(aHelixWire, ShapeType.Edge);
        Assert.True(aNbEdges > 1); // Should have multiple edges for multiple parts
        // Verify wire continuity by checking connectivity
        Wire aWire = ShapeCasts.Wire(aHelixWire);
        var aCheck = new BRepCheck_Analyzer(aWire);
        Assert.True(aCheck.IsValid());
    }

    // ---- the port's own ----

    /// <summary>
    /// The wire is frozen when handed out; a multi-part wire's smoothed edges carry new curves, and every curve of the
    /// wire is frozen (decision 994). Statuses before any parameters are set are OCCT's.
    /// </summary>
    [Fact]
    public void The_result_is_frozen_and_the_initial_statuses_are_OCCTs()
    {
        var aBuilder = new HelixBRep_BuilderHelix();
        Assert.Equal(1, aBuilder.ErrorStatus());
        Assert.Equal(1, aBuilder.WarningStatus());
        Assert.Equal(99.0, aBuilder.ToleranceReached());
        aBuilder.Perform(); // nothing set: does nothing
        Assert.True(aBuilder.Shape().IsNull);

        NCollection_Array1<double> aDiams = new(1, 3);
        aDiams[1] = 80.0;
        aDiams[2] = 60.0;
        aDiams[3] = 40.0;
        NCollection_Array1<double> aPitches = new(1, 2);
        aPitches[1] = 8.0;
        aPitches[2] = 12.0;
        NCollection_Array1<double> aNbTurns = new(1, 2);
        aNbTurns[1] = 2.5;
        aNbTurns[2] = 3.0;
        aBuilder.SetParameters(myAxis, aDiams, aPitches, aNbTurns);
        aBuilder.Perform();
        Assert.Equal(0, aBuilder.ErrorStatus());
        Shape aWire = aBuilder.Shape();
        Assert.True(aWire.TShape()!.IsFrozen);
        for (var anExp = new Explorer(aWire, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anExp.Current());
            Assert.True(anEdge.TShape()!.IsFrozen);
            Assert.True(Tool.Curve(anEdge, out _, out _)!.IsFrozen);
        }
        // a second Perform builds a new wire and leaves the first unchanged
        aBuilder.Perform();
        Assert.False(aWire.IsSame(aBuilder.Shape()));
        Assert.True(new BRepCheck_Analyzer(aWire).IsValid());
    }
}
