// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/Geom2dGcc_Circ2d3Tan_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTests of Geom2dGcc_Circ2d3Tan (three circles, solved through GccAna_Circ2d3Tan).</summary>
public class Geom2dGcc_Circ2d3TanTests
{
    private readonly double myTolerance = Precision.Confusion();

    /// <summary>Verify that a solution is tangent to all three input circles.</summary>
    private static void verifyTangencyConstraints(in gp_Circ2d theSolution,
                                                  in gp_Circ2d theCircle1,
                                                  in gp_Circ2d theCircle2,
                                                  in gp_Circ2d theCircle3,
                                                  int theSolutionIndex,
                                                  double theTolerance = 1e-6)
    {
        gp_Pnt2d aSolCenter = theSolution.Location();
        double aSolRadius = theSolution.Radius();

        // Check tangency to each input circle
        for (int i = 1; i <= 3; i++)
        {
            gp_Circ2d aCircle = (i == 1) ? theCircle1 : (i == 2) ? theCircle2 : theCircle3;
            gp_Pnt2d aCircCenter = aCircle.Location();
            double aCircRadius = aCircle.Radius();

            double aDistanceCenters = aSolCenter.Distance(aCircCenter);

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
    private static Geom2dGcc_QualifiedCurve createQualifiedCircle(double theX, double theY, double theRadius)
    {
        gp_Pnt2d aCenter = new(theX, theY);
        Geom2d_Circle aCircle = new(new gp_Circ2d(new gp_Ax2d(aCenter, new gp_Dir2d(1, 0)), theRadius));
        Geom2dAdaptor_Curve anAdaptor = new(aCircle);
        return Geom2dGcc.Unqualified(anAdaptor);
    }

    private static gp_Circ2d Circ(double theX, double theY, double theR)
        => new(new gp_Ax2d(new gp_Pnt2d(theX, theY), new gp_Dir2d(1, 0)), theR);

    /// <summary>Verify solution validity.</summary>
    private static void verifySolutionValidity(in gp_Circ2d theSolution,
                                               int theSolutionIndex,
                                               double theMinX = -10000.0,
                                               double theMaxX = 10000.0,
                                               double theMinY = -10000.0,
                                               double theMaxY = 10000.0,
                                               double theMaxRadius = 100000.0)
    {
        Assert.True(theSolution.Radius() > 0, $"Solution {theSolutionIndex} should have positive radius");
        Assert.True(theSolution.Radius() < theMaxRadius, $"Solution {theSolutionIndex} should have reasonable radius");

        gp_Pnt2d aCenter = theSolution.Location();
        Assert.True(aCenter.X() > theMinX, $"Solution {theSolutionIndex} X coordinate should be reasonable");
        Assert.True(aCenter.X() < theMaxX, $"Solution {theSolutionIndex} X coordinate should be reasonable");
        Assert.True(aCenter.Y() > theMinY, $"Solution {theSolutionIndex} Y coordinate should be reasonable");
        Assert.True(aCenter.Y() < theMaxY, $"Solution {theSolutionIndex} Y coordinate should be reasonable");
    }

    // Test the exact BUC60622 regression case
    [Fact]
    public void BUC60622_RegressionCase()
    {
        // BUC60622 test case: 3 circles that should produce 3 tangent solutions
        Geom2dGcc_QualifiedCurve aQual1 = createQualifiedCircle(500.0, 1800.0, 500.0);
        Geom2dGcc_QualifiedCurve aQual2 = createQualifiedCircle(500.0, 1900.0, 400.0);
        Geom2dGcc_QualifiedCurve aQual3 = createQualifiedCircle(700.0, 1900.0, 200.0);

        gp_Circ2d aInputCirc1 = Circ(500.0, 1800.0, 500.0);
        gp_Circ2d aInputCirc2 = Circ(500.0, 1900.0, 400.0);
        gp_Circ2d aInputCirc3 = Circ(700.0, 1900.0, 200.0);

        Geom2dGcc_Circ2d3Tan aTangentSolver = new(aQual1, aQual2, aQual3, myTolerance, 0, 0, 0);

        Assert.True(aTangentSolver.IsDone(), "Algorithm should succeed");

        int aNbSolutions = aTangentSolver.NbSolutions();

        // The regression test: we should find exactly 3 solutions for BUC60622 case
        Assert.Equal(3, aNbSolutions);

        for (int i = 1; i <= aNbSolutions; i++)
        {
            gp_Circ2d aSol = aTangentSolver.ThisSolution(i);
            verifySolutionValidity(aSol, i, -1000.0, 2000.0, 1000.0, 3000.0, 10000.0);
            verifyTangencyConstraints(aSol, aInputCirc1, aInputCirc2, aInputCirc3, i);
        }

        if (aNbSolutions >= 3)
        {
            // Solution 1: center(500, 1900), radius=400 - this is actually one of the input circles
            gp_Circ2d aSol1 = aTangentSolver.ThisSolution(1);
            Assert.Equal(500.0, aSol1.Location().X(), 1.0);
            Assert.Equal(1900.0, aSol1.Location().Y(), 1.0);
            Assert.Equal(400.0, aSol1.Radius(), 1.0);
        }
    }

    // Test with different tolerance values to demonstrate the filtering issue
    [Fact]
    public void ToleranceImpact_Analysis()
    {
        Geom2dGcc_QualifiedCurve aQual1 = createQualifiedCircle(500.0, 1800.0, 500.0);
        Geom2dGcc_QualifiedCurve aQual2 = createQualifiedCircle(500.0, 1900.0, 400.0);
        Geom2dGcc_QualifiedCurve aQual3 = createQualifiedCircle(700.0, 1900.0, 200.0);

        double[] aTestTolerances = [Precision.Confusion(), 1e-12, 1e-10, 1e-8];

        int aDefaultSolCount = 0;

        foreach (double aTol in aTestTolerances)
        {
            Geom2dGcc_Circ2d3Tan aSolver = new(aQual1, aQual2, aQual3, aTol, 0, 0, 0);

            Assert.True(aSolver.IsDone(), $"Algorithm should succeed with tolerance {aTol}");

            int aNbSol = aSolver.NbSolutions();

            if (aTol == Precision.Confusion())
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
                gp_Circ2d aSol = aSolver.ThisSolution(i);
                verifySolutionValidity(aSol, i, -1000.0, 2000.0, 1000.0, 3000.0, 10000.0);
            }
        }
    }

    // Test a simpler case that should definitely work
    [Fact]
    public void Simple_ThreeCircle_Case()
    {
        Geom2dGcc_QualifiedCurve aQual1 = createQualifiedCircle(0.0, 0.0, 2.0);
        Geom2dGcc_QualifiedCurve aQual2 = createQualifiedCircle(10.0, 0.0, 2.0);
        Geom2dGcc_QualifiedCurve aQual3 = createQualifiedCircle(5.0, 8.0, 2.0);

        gp_Circ2d aInputCirc1 = Circ(0.0, 0.0, 2.0);
        gp_Circ2d aInputCirc2 = Circ(10.0, 0.0, 2.0);
        gp_Circ2d aInputCirc3 = Circ(5.0, 8.0, 2.0);

        Geom2dGcc_Circ2d3Tan aSolver = new(aQual1, aQual2, aQual3, myTolerance, 0, 0, 0);

        Assert.True(aSolver.IsDone(), "Simple case should always work");

        int aNbSol = aSolver.NbSolutions();
        Assert.True(aNbSol >= 1, "Should find at least one solution for simple case");

        for (int i = 1; i <= aNbSol; i++)
        {
            gp_Circ2d aSol = aSolver.ThisSolution(i);
            verifySolutionValidity(aSol, i);
            verifyTangencyConstraints(aSol, aInputCirc1, aInputCirc2, aInputCirc3, i);
        }
    }

    // Test with concentric circles to verify edge cases
    [Fact]
    public void Concentric_Circles_EdgeCase()
    {
        // Concentric circles at (0,0) with radii 1, 3 and a separate circle at (10,0) radius 2: no solutions
        Geom2dGcc_QualifiedCurve aQual1 = createQualifiedCircle(0.0, 0.0, 1.0);
        Geom2dGcc_QualifiedCurve aQual2 = createQualifiedCircle(0.0, 0.0, 3.0);
        Geom2dGcc_QualifiedCurve aQual3 = createQualifiedCircle(10.0, 0.0, 2.0);

        Geom2dGcc_Circ2d3Tan aSolver = new(aQual1, aQual2, aQual3, myTolerance, 0, 0, 0);

        Assert.True(aSolver.IsDone(), "Algorithm should complete successfully even when no solutions exist");

        int aNbSol = aSolver.NbSolutions();
        Assert.Equal(0, aNbSol);
    }

    // Test with very small circles
    [Fact]
    public void SmallCircles_PrecisionTest()
    {
        Geom2dGcc_QualifiedCurve aQual1 = createQualifiedCircle(0.0, 0.0, 0.01);
        Geom2dGcc_QualifiedCurve aQual2 = createQualifiedCircle(0.1, 0.0, 0.01);
        Geom2dGcc_QualifiedCurve aQual3 = createQualifiedCircle(0.05, 0.08, 0.01);

        gp_Circ2d aInputCirc1 = Circ(0.0, 0.0, 0.01);
        gp_Circ2d aInputCirc2 = Circ(0.1, 0.0, 0.01);
        gp_Circ2d aInputCirc3 = Circ(0.05, 0.08, 0.01);

        Geom2dGcc_Circ2d3Tan aSolver = new(aQual1, aQual2, aQual3, myTolerance, 0, 0, 0);

        Assert.True(aSolver.IsDone(), "Small circles should be handled correctly");

        int aNbSol = aSolver.NbSolutions();
        Assert.True(aNbSol >= 1, "Should find at least one solution for small circles");

        for (int i = 1; i <= aNbSol; i++)
        {
            gp_Circ2d aSol = aSolver.ThisSolution(i);
            verifySolutionValidity(aSol, i, -10.0, 10.0, -10.0, 10.0, 10.0);
            verifyTangencyConstraints(aSol, aInputCirc1, aInputCirc2, aInputCirc3, i, 1e-3);
        }
    }

    // Test with large circles
    [Fact]
    public void LargeCircles_ScalingTest()
    {
        Geom2dGcc_QualifiedCurve aQual1 = createQualifiedCircle(0.0, 0.0, 1000.0);
        Geom2dGcc_QualifiedCurve aQual2 = createQualifiedCircle(5000.0, 0.0, 1500.0);
        Geom2dGcc_QualifiedCurve aQual3 = createQualifiedCircle(2500.0, 4000.0, 800.0);

        gp_Circ2d aInputCirc1 = Circ(0.0, 0.0, 1000.0);
        gp_Circ2d aInputCirc2 = Circ(5000.0, 0.0, 1500.0);
        gp_Circ2d aInputCirc3 = Circ(2500.0, 4000.0, 800.0);

        Geom2dGcc_Circ2d3Tan aSolver = new(aQual1, aQual2, aQual3, myTolerance, 0, 0, 0);

        Assert.True(aSolver.IsDone(), "Large circles should be handled correctly");

        int aNbSol = aSolver.NbSolutions();
        Assert.True(aNbSol >= 1, "Should find at least one solution for large circles");

        for (int i = 1; i <= aNbSol; i++)
        {
            gp_Circ2d aSol = aSolver.ThisSolution(i);
            verifySolutionValidity(aSol, i, -10000.0, 10000.0, -10000.0, 10000.0, 50000.0);
            verifyTangencyConstraints(aSol, aInputCirc1, aInputCirc2, aInputCirc3, i, 1.0);
        }
    }

    // Test with circles in a line configuration
    [Fact]
    public void LinearConfiguration_GeometricTest()
    {
        Geom2dGcc_QualifiedCurve aQual1 = createQualifiedCircle(0.0, 0.0, 1.0);
        Geom2dGcc_QualifiedCurve aQual2 = createQualifiedCircle(5.0, 0.0, 1.5);
        Geom2dGcc_QualifiedCurve aQual3 = createQualifiedCircle(10.0, 0.0, 1.2);

        gp_Circ2d aInputCirc1 = Circ(0.0, 0.0, 1.0);
        gp_Circ2d aInputCirc2 = Circ(5.0, 0.0, 1.5);
        gp_Circ2d aInputCirc3 = Circ(10.0, 0.0, 1.2);

        Geom2dGcc_Circ2d3Tan aSolver = new(aQual1, aQual2, aQual3, myTolerance, 0, 0, 0);

        Assert.True(aSolver.IsDone(), "Linear configuration should be solvable");

        int aNbSol = aSolver.NbSolutions();
        Assert.True(aNbSol >= 1, "Should find at least one solution for linear configuration");

        for (int i = 1; i <= aNbSol; i++)
        {
            gp_Circ2d aSol = aSolver.ThisSolution(i);
            verifySolutionValidity(aSol, i);
            verifyTangencyConstraints(aSol, aInputCirc1, aInputCirc2, aInputCirc3, i);
        }
    }

    // Test with touching circles
    [Fact]
    public void TouchingCircles_DegenerateCase()
    {
        // Two circles that touch each other externally plus a third separate circle
        Geom2dGcc_QualifiedCurve aQual1 = createQualifiedCircle(0.0, 0.0, 2.0);
        Geom2dGcc_QualifiedCurve aQual2 = createQualifiedCircle(4.0, 0.0, 2.0); // Touch at (2,0)
        Geom2dGcc_QualifiedCurve aQual3 = createQualifiedCircle(2.0, 5.0, 1.5);

        gp_Circ2d aInputCirc1 = Circ(0.0, 0.0, 2.0);
        gp_Circ2d aInputCirc2 = Circ(4.0, 0.0, 2.0);
        gp_Circ2d aInputCirc3 = Circ(2.0, 5.0, 1.5);

        Geom2dGcc_Circ2d3Tan aSolver = new(aQual1, aQual2, aQual3, myTolerance, 0, 0, 0);

        Assert.True(aSolver.IsDone(), "Touching circles configuration should be solvable");

        int aNbSol = aSolver.NbSolutions();
        Assert.True(aNbSol >= 1, "Should find at least one solution for touching circles case");

        for (int i = 1; i <= aNbSol; i++)
        {
            gp_Circ2d aSol = aSolver.ThisSolution(i);
            verifySolutionValidity(aSol, i);
            verifyTangencyConstraints(aSol, aInputCirc1, aInputCirc2, aInputCirc3, i);
        }
    }

    // Not an OCCT test: the iterative path (a free-form argument). Geom2dGcc_FunctionTanCuCuCu's Values computes
    // nothing (OCCT), so math_FunctionSetRoot sees the zero values of its own vectors and stops at the starting point:
    // the circle through the three starting points is kept only when it is already tangent. Native OCCT reads
    // uninitialised storage there (occt-issues.md), so these outcomes are the port's.
    private static Geom2dGcc_QualifiedCurve StraightBSpline()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 2);
        aPoles[1] = new gp_Pnt2d(-5.0, 3.0);
        aPoles[2] = new gp_Pnt2d(5.0, 3.0);
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 2;
        aMults[2] = 2;
        return Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 1)));
    }

    [Fact]
    public void Iterative_StartOnSolution_IsKept_AndTangency3Throws()
    {
        double anA = Math.Atan2(0.6, 0.8);
        Geom2dGcc_Circ2d3Tan aSolver = new(createQualifiedCircle(0.0, 0.0, 1.0), createQualifiedCircle(4.0, 0.0, 1.0),
                                           StraightBSpline(), 1.0e-9, anA, Math.PI - anA, 0.7);
        Assert.True(aSolver.IsDone());
        Assert.Equal(1, aSolver.NbSolutions());
        gp_Circ2d aSol = aSolver.ThisSolution(1);
        Assert.Equal(2.0, aSol.Location().X(), 1.0e-9);
        Assert.Equal(1.5, aSol.Location().Y(), 1.0e-9);
        Assert.Equal(1.5, aSol.Radius(), 1.0e-9);
        aSolver.Tangency1(1, out _, out double aPar1, out _);
        Assert.Equal(anA, aPar1, 1.0e-12);
        // OCCT: Geom2dGcc_Circ2d3TanIter::IsTheSame3 is always true, so the third tangency is never given.
        Assert.True(aSolver.IsTheSame3(1));
        Assert.Throws<StdFail_NotDone>(() => aSolver.Tangency3(1, out _, out _, out _));
    }

    [Fact]
    public void Iterative_StartOffSolution_IsNotDone()
    {
        Geom2dGcc_Circ2d3Tan aSolver = new(createQualifiedCircle(0.0, 0.0, 1.0), createQualifiedCircle(4.0, 0.0, 1.0),
                                           StraightBSpline(), 1.0e-9, 0.5, 2.5, 0.6);
        Assert.False(aSolver.IsDone());
        Assert.Equal(1, aSolver.NbSolutions());
        Assert.Throws<StdFail_NotDone>(() => aSolver.ThisSolution(1));
    }
}
