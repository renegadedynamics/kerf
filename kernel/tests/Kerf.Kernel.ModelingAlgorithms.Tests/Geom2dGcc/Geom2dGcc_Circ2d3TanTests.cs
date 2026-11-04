// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/Geom2dGcc_Circ2d3Tan_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTests of Geom2dGcc_Circ2d3Tan (three circles, solved through GccAna_Circ2d3Tan).</summary>
public class Geom2dGcc_Circ2d3TanTests
{
    private readonly double myTolerance = global::Kerf.Engine.Foundation.Precision.Confusion();

    /// <summary>Verify that a solution is tangent to all three input circles.</summary>
    private static void verifyTangencyConstraints(in Circle2D theSolution,
                                                  in Circle2D theCircle1,
                                                  in Circle2D theCircle2,
                                                  in Circle2D theCircle3,
                                                  int theSolutionIndex,
                                                  double theTolerance = 1e-6)
    {
        Point2D aSolCenter = theSolution.Location;
        double aSolRadius = theSolution.Radius;

        // Check tangency to each input circle
        for (int i = 1; i <= 3; i++)
        {
            Circle2D aCircle = (i == 1) ? theCircle1 : (i == 2) ? theCircle2 : theCircle3;
            Point2D aCircCenter = aCircle.Location;
            double aCircRadius = aCircle.Radius;

            double aDistanceCenters = aSolCenter.DistanceTo(aCircCenter);

            // For tangency: distance = |R1 - R2| (internal) or R1 + R2 (external)
            double aExpectedDistExt = aSolRadius + aCircRadius;
            double aExpectedDistInt = Math.Abs(aSolRadius - aCircRadius);

            bool isTangent = (Math.Abs(aDistanceCenters - aExpectedDistExt) <= theTolerance)
                             || (Math.Abs(aDistanceCenters - aExpectedDistInt) <= theTolerance);

            Assert.True(isTangent, $"Solution {theSolutionIndex} should be tangent to circle {i} (distance="
                                   + $"{aDistanceCenters}, expected external={aExpectedDistExt}, expected internal="
                                   + $"{aExpectedDistInt})");
        }
    }

    /// <summary>Create a qualified curve from circle parameters.</summary>
    private static global::Kerf.Engine.Construction.Constraints.QualifiedCurve createQualifiedCircle(double theX, double theY, double theRadius)
    {
        Point2D aCenter = new(theX, theY);
        CircleCurve2D aCircle = new(new Circle2D(new Axis2D(aCenter, new Direction2D(1, 0)), theRadius));
        CurveAdaptor2D anAdaptor = new(aCircle);
        return global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(anAdaptor);
    }

    private static Circle2D Circ(double theX, double theY, double theR)
        => new(new Axis2D(new Point2D(theX, theY), new Direction2D(1, 0)), theR);

    /// <summary>Verify solution validity.</summary>
    private static void verifySolutionValidity(in Circle2D theSolution,
                                               int theSolutionIndex,
                                               double theMinX = -10000.0,
                                               double theMaxX = 10000.0,
                                               double theMinY = -10000.0,
                                               double theMaxY = 10000.0,
                                               double theMaxRadius = 100000.0)
    {
        Assert.True(theSolution.Radius > 0, $"Solution {theSolutionIndex} should have positive radius");
        Assert.True(theSolution.Radius < theMaxRadius, $"Solution {theSolutionIndex} should have reasonable radius");

        Point2D aCenter = theSolution.Location;
        Assert.True(aCenter.X > theMinX, $"Solution {theSolutionIndex} X coordinate should be reasonable");
        Assert.True(aCenter.X < theMaxX, $"Solution {theSolutionIndex} X coordinate should be reasonable");
        Assert.True(aCenter.Y > theMinY, $"Solution {theSolutionIndex} Y coordinate should be reasonable");
        Assert.True(aCenter.Y < theMaxY, $"Solution {theSolutionIndex} Y coordinate should be reasonable");
    }

    // Test the exact BUC60622 regression case
    [Fact]
    public void BUC60622_RegressionCase()
    {
        // BUC60622 test case: 3 circles that should produce 3 tangent solutions
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual1 = createQualifiedCircle(500.0, 1800.0, 500.0);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual2 = createQualifiedCircle(500.0, 1900.0, 400.0);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual3 = createQualifiedCircle(700.0, 1900.0, 200.0);

        Circle2D aInputCirc1 = Circ(500.0, 1800.0, 500.0);
        Circle2D aInputCirc2 = Circ(500.0, 1900.0, 400.0);
        Circle2D aInputCirc3 = Circ(700.0, 1900.0, 200.0);

        global::Kerf.Engine.Construction.Constraints.Circ2d3Tan aTangentSolver = new(aQual1, aQual2, aQual3, myTolerance, 0, 0, 0);

        Assert.True(aTangentSolver.IsDone, "Algorithm should succeed");

        int aNbSolutions = aTangentSolver.SolutionCount;

        // The regression test: we should find exactly 3 solutions for BUC60622 case
        Assert.Equal(3, aNbSolutions);

        for (int i = 1; i <= aNbSolutions; i++)
        {
            Circle2D aSol = aTangentSolver.ThisSolution(i);
            verifySolutionValidity(aSol, i, -1000.0, 2000.0, 1000.0, 3000.0, 10000.0);
            verifyTangencyConstraints(aSol, aInputCirc1, aInputCirc2, aInputCirc3, i);
        }

        if (aNbSolutions >= 3)
        {
            // Solution 1: center(500, 1900), radius=400 - this is actually one of the input circles
            Circle2D aSol1 = aTangentSolver.ThisSolution(1);
            Assert.Equal(500.0, aSol1.Location.X, 1.0);
            Assert.Equal(1900.0, aSol1.Location.Y, 1.0);
            Assert.Equal(400.0, aSol1.Radius, 1.0);
        }
    }

    // Test with different tolerance values to demonstrate the filtering issue
    [Fact]
    public void ToleranceImpact_Analysis()
    {
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual1 = createQualifiedCircle(500.0, 1800.0, 500.0);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual2 = createQualifiedCircle(500.0, 1900.0, 400.0);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual3 = createQualifiedCircle(700.0, 1900.0, 200.0);

        double[] aTestTolerances = [global::Kerf.Engine.Foundation.Precision.Confusion(), 1e-12, 1e-10, 1e-8];

        int aDefaultSolCount = 0;

        foreach (double aTol in aTestTolerances)
        {
            global::Kerf.Engine.Construction.Constraints.Circ2d3Tan aSolver = new(aQual1, aQual2, aQual3, aTol, 0, 0, 0);

            Assert.True(aSolver.IsDone, $"Algorithm should succeed with tolerance {aTol}");

            int aNbSol = aSolver.SolutionCount;

            if (aTol == global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                aDefaultSolCount = aNbSol;
                Assert.True(aNbSol >= 1, "Should find at least 1 solution with default tolerance");
            }
            else
            {
                Assert.True(aNbSol >= 1, $"Should find at least 1 solution with tolerance {aTol}");
                Assert.True(aNbSol <= aDefaultSolCount + 2,
                            $"Solution count shouldn't increase dramatically with tolerance {aTol}");
            }

            for (int i = 1; i <= aNbSol; i++)
            {
                Circle2D aSol = aSolver.ThisSolution(i);
                verifySolutionValidity(aSol, i, -1000.0, 2000.0, 1000.0, 3000.0, 10000.0);
            }
        }
    }

    // Test a simpler case that should definitely work
    [Fact]
    public void Simple_ThreeCircle_Case()
    {
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual1 = createQualifiedCircle(0.0, 0.0, 2.0);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual2 = createQualifiedCircle(10.0, 0.0, 2.0);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual3 = createQualifiedCircle(5.0, 8.0, 2.0);

        Circle2D aInputCirc1 = Circ(0.0, 0.0, 2.0);
        Circle2D aInputCirc2 = Circ(10.0, 0.0, 2.0);
        Circle2D aInputCirc3 = Circ(5.0, 8.0, 2.0);

        global::Kerf.Engine.Construction.Constraints.Circ2d3Tan aSolver = new(aQual1, aQual2, aQual3, myTolerance, 0, 0, 0);

        Assert.True(aSolver.IsDone, "Simple case should always work");

        int aNbSol = aSolver.SolutionCount;
        Assert.True(aNbSol >= 1, "Should find at least one solution for simple case");

        for (int i = 1; i <= aNbSol; i++)
        {
            Circle2D aSol = aSolver.ThisSolution(i);
            verifySolutionValidity(aSol, i);
            verifyTangencyConstraints(aSol, aInputCirc1, aInputCirc2, aInputCirc3, i);
        }
    }

    // Test with concentric circles to verify edge cases
    [Fact]
    public void Concentric_Circles_EdgeCase()
    {
        // Concentric circles at (0,0) with radii 1, 3 and a separate circle at (10,0) radius 2: no solutions
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual1 = createQualifiedCircle(0.0, 0.0, 1.0);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual2 = createQualifiedCircle(0.0, 0.0, 3.0);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual3 = createQualifiedCircle(10.0, 0.0, 2.0);

        global::Kerf.Engine.Construction.Constraints.Circ2d3Tan aSolver = new(aQual1, aQual2, aQual3, myTolerance, 0, 0, 0);

        Assert.True(aSolver.IsDone, "Algorithm should complete successfully even when no solutions exist");

        int aNbSol = aSolver.SolutionCount;
        Assert.Equal(0, aNbSol);
    }

    // Test with very small circles
    [Fact]
    public void SmallCircles_PrecisionTest()
    {
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual1 = createQualifiedCircle(0.0, 0.0, 0.01);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual2 = createQualifiedCircle(0.1, 0.0, 0.01);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual3 = createQualifiedCircle(0.05, 0.08, 0.01);

        Circle2D aInputCirc1 = Circ(0.0, 0.0, 0.01);
        Circle2D aInputCirc2 = Circ(0.1, 0.0, 0.01);
        Circle2D aInputCirc3 = Circ(0.05, 0.08, 0.01);

        global::Kerf.Engine.Construction.Constraints.Circ2d3Tan aSolver = new(aQual1, aQual2, aQual3, myTolerance, 0, 0, 0);

        Assert.True(aSolver.IsDone, "Small circles should be handled correctly");

        int aNbSol = aSolver.SolutionCount;
        Assert.True(aNbSol >= 1, "Should find at least one solution for small circles");

        for (int i = 1; i <= aNbSol; i++)
        {
            Circle2D aSol = aSolver.ThisSolution(i);
            verifySolutionValidity(aSol, i, -10.0, 10.0, -10.0, 10.0, 10.0);
            verifyTangencyConstraints(aSol, aInputCirc1, aInputCirc2, aInputCirc3, i, 1e-3);
        }
    }

    // Test with large circles
    [Fact]
    public void LargeCircles_ScalingTest()
    {
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual1 = createQualifiedCircle(0.0, 0.0, 1000.0);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual2 = createQualifiedCircle(5000.0, 0.0, 1500.0);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual3 = createQualifiedCircle(2500.0, 4000.0, 800.0);

        Circle2D aInputCirc1 = Circ(0.0, 0.0, 1000.0);
        Circle2D aInputCirc2 = Circ(5000.0, 0.0, 1500.0);
        Circle2D aInputCirc3 = Circ(2500.0, 4000.0, 800.0);

        global::Kerf.Engine.Construction.Constraints.Circ2d3Tan aSolver = new(aQual1, aQual2, aQual3, myTolerance, 0, 0, 0);

        Assert.True(aSolver.IsDone, "Large circles should be handled correctly");

        int aNbSol = aSolver.SolutionCount;
        Assert.True(aNbSol >= 1, "Should find at least one solution for large circles");

        for (int i = 1; i <= aNbSol; i++)
        {
            Circle2D aSol = aSolver.ThisSolution(i);
            verifySolutionValidity(aSol, i, -10000.0, 10000.0, -10000.0, 10000.0, 50000.0);
            verifyTangencyConstraints(aSol, aInputCirc1, aInputCirc2, aInputCirc3, i, 1.0);
        }
    }

    // Test with circles in a line configuration
    [Fact]
    public void LinearConfiguration_GeometricTest()
    {
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual1 = createQualifiedCircle(0.0, 0.0, 1.0);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual2 = createQualifiedCircle(5.0, 0.0, 1.5);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual3 = createQualifiedCircle(10.0, 0.0, 1.2);

        Circle2D aInputCirc1 = Circ(0.0, 0.0, 1.0);
        Circle2D aInputCirc2 = Circ(5.0, 0.0, 1.5);
        Circle2D aInputCirc3 = Circ(10.0, 0.0, 1.2);

        global::Kerf.Engine.Construction.Constraints.Circ2d3Tan aSolver = new(aQual1, aQual2, aQual3, myTolerance, 0, 0, 0);

        Assert.True(aSolver.IsDone, "Linear configuration should be solvable");

        int aNbSol = aSolver.SolutionCount;
        Assert.True(aNbSol >= 1, "Should find at least one solution for linear configuration");

        for (int i = 1; i <= aNbSol; i++)
        {
            Circle2D aSol = aSolver.ThisSolution(i);
            verifySolutionValidity(aSol, i);
            verifyTangencyConstraints(aSol, aInputCirc1, aInputCirc2, aInputCirc3, i);
        }
    }

    // Test with touching circles
    [Fact]
    public void TouchingCircles_DegenerateCase()
    {
        // Two circles that touch each other externally plus a third separate circle
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual1 = createQualifiedCircle(0.0, 0.0, 2.0);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual2 = createQualifiedCircle(4.0, 0.0, 2.0); // Touch at (2,0)
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQual3 = createQualifiedCircle(2.0, 5.0, 1.5);

        Circle2D aInputCirc1 = Circ(0.0, 0.0, 2.0);
        Circle2D aInputCirc2 = Circ(4.0, 0.0, 2.0);
        Circle2D aInputCirc3 = Circ(2.0, 5.0, 1.5);

        global::Kerf.Engine.Construction.Constraints.Circ2d3Tan aSolver = new(aQual1, aQual2, aQual3, myTolerance, 0, 0, 0);

        Assert.True(aSolver.IsDone, "Touching circles configuration should be solvable");

        int aNbSol = aSolver.SolutionCount;
        Assert.True(aNbSol >= 1, "Should find at least one solution for touching circles case");

        for (int i = 1; i <= aNbSol; i++)
        {
            Circle2D aSol = aSolver.ThisSolution(i);
            verifySolutionValidity(aSol, i);
            verifyTangencyConstraints(aSol, aInputCirc1, aInputCirc2, aInputCirc3, i);
        }
    }

    // Not an OCCT test: the iterative path (a free-form argument). Geom2dGcc_FunctionTanCuCuCu's Values computes
    // nothing (OCCT), so math_FunctionSetRoot sees the zero values of its own vectors and stops at the starting point:
    // the circle through the three starting points is kept only when it is already tangent. Native OCCT reads
    // uninitialised storage there (occt-issues.md), so these outcomes are the port's.
    private static global::Kerf.Engine.Construction.Constraints.QualifiedCurve StraightBSpline()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 2);
        aPoles[1] = new Point2D(-5.0, 3.0);
        aPoles[2] = new Point2D(5.0, 3.0);
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 2;
        aMults[2] = 2;
        return global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(new CurveAdaptor2D(new BSplineCurve2D(aPoles, aKnots, aMults, 1)));
    }

    [Fact]
    public void Iterative_StartOnSolution_IsKept_WithItsThreeTangencies()
    {
        double anA = Math.Atan2(0.6, 0.8);
        global::Kerf.Engine.Construction.Constraints.Circ2d3Tan aSolver = new(createQualifiedCircle(0.0, 0.0, 1.0), createQualifiedCircle(4.0, 0.0, 1.0),
                                           StraightBSpline(), 1.0e-9, anA, Math.PI - anA, 0.7);
        Assert.True(aSolver.IsDone);
        Assert.Equal(1, aSolver.SolutionCount);
        Circle2D aSol = aSolver.ThisSolution(1);
        Assert.Equal(2.0, aSol.Location.X, 1.0e-9);
        Assert.Equal(1.5, aSol.Location.Y, 1.0e-9);
        Assert.Equal(1.5, aSol.Radius, 1.0e-9);
        aSolver.Tangency1(1, out double aSol1, out double aPar1, out Point2D aP1);
        Assert.Equal(anA, aPar1, 1.0e-12);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aSol1, aSol).DistanceTo(aP1) < 1.0e-9);
        // the third tangency, on the line y = 3 (OCCT's IsTheSame3 was always true, so it was never given: decision 1546)
        Assert.False(aSolver.IsTheSame3(1));
        aSolver.Tangency3(1, out double aSol3, out _, out Point2D aP3);
        Assert.True(aP3.DistanceTo(new Point2D(2.0, 3.0)) < 1.0e-9);
        Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aSol3, aSol).DistanceTo(aP3) < 1.0e-9);
    }

    [Fact]
    public void Iterative_StartOffSolution_IsNotDone()
    {
        global::Kerf.Engine.Construction.Constraints.Circ2d3Tan aSolver = new(createQualifiedCircle(0.0, 0.0, 1.0), createQualifiedCircle(4.0, 0.0, 1.0),
                                           StraightBSpline(), 1.0e-9, 0.5, 2.5, 0.6);
        Assert.False(aSolver.IsDone);
        Assert.Equal(1, aSolver.SolutionCount);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => aSolver.ThisSolution(1));
    }
}
