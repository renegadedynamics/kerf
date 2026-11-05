// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// GccAna_Circ2d3Tan on OCCT's documented configurations: the circles tangent to three circles, lines or points.
/// Every solution is checked against its arguments (tangency, pass-through, the tangency points and their
/// parameters), and qualified arguments against the solutions they keep.
/// </summary>
public class GccAna_Circ2d3TanTests
{
    private const double Tol = 1.0e-7;

    /// <summary>An argument: a qualified circle, a qualified line or a point.</summary>
    private readonly record struct Arg(int Kind, Circle2D C, Line2D L, Point2D P)
    {
        public static Arg Of(in global::Kerf.Engine.Construction.Constraints.QualifiedCirc Q) => new(0, Q.Qualified(), new Line2D(), new Point2D());

        public static Arg Of(in global::Kerf.Engine.Construction.Constraints.QualifiedLin Q) => new(1, new Circle2D(), Q.Qualified(), new Point2D());

        public static Arg Of(in Point2D P) => new(2, new Circle2D(), new Line2D(), P);
    }

    private static Circle2D Circ(double theX, double theY, double theR)
        => new(new Axis2D(new Point2D(theX, theY), new Direction2D(Direction2D.D.X)), theR);

    private static Line2D Lin(double theX, double theY, double theDX, double theDY)
        => new(new Point2D(theX, theY), new Direction2D(theDX, theDY));

    /// <summary>The position of the solution relative to a circle, as GccEnt names it.</summary>
    private static global::Kerf.Engine.Construction.Constraints.Qualifier Classify(in Circle2D theSol, in Circle2D theArg)
    {
        double d = theSol.Location.DistanceTo(theArg.Location);
        double r = theSol.Radius;
        double R = theArg.Radius;
        if (Math.Abs(d - (R + r)) <= Tol)
        {
            return global::Kerf.Engine.Construction.Constraints.Qualifier.Outside;
        }
        if (Math.Abs(d + r - R) <= Tol)
        {
            return global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed;
        }
        if (Math.Abs(d + R - r) <= Tol)
        {
            return global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing;
        }
        return global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier;
    }

    /// <summary>The side of a line the solution is on: enclosed on the left, outside on the right.</summary>
    private static global::Kerf.Engine.Construction.Constraints.Qualifier Classify(in Circle2D theSol, in Line2D theArg)
    {
        double aCross = theArg.Direction.XY() ^ (theSol.Location.XY() - theArg.Location.XY());
        return aCross > 0.0 ? global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed : global::Kerf.Engine.Construction.Constraints.Qualifier.Outside;
    }

    private static void AssertNear(in Point2D theExpected, in Point2D theActual, double theTol = Tol)
        => Assert.True(theExpected.DistanceTo(theActual) <= theTol, $"{theExpected} != {theActual}");

    private delegate void TangencyFn(int theIndex, out double theParSol, out double theParArg, out Point2D thePnt);

    /// <summary>Checks that every solution touches each argument, and the tangency data the solver gives.</summary>
    private static void CheckSolutions(AnalyticCirc2d3Tan theSolver, Arg theA1, Arg theA2, Arg theA3,
                                       int theSkipTangency = 0)
    {
        Assert.True(theSolver.IsDone);
        Arg[] anArgs = [theA1, theA2, theA3];
        TangencyFn[] aTangency = [theSolver.Tangency1, theSolver.Tangency2, theSolver.Tangency3];
        Func<int, bool>[] aSame = [theSolver.IsTheSame1, theSolver.IsTheSame2, theSolver.IsTheSame3];
        for (int i = 1; i <= theSolver.SolutionCount; i++)
        {
            Circle2D aSol = theSolver.ThisSolution(i);
            for (int k = 0; k < 3; k++)
            {
                Arg anArg = anArgs[k];
                switch (anArg.Kind)
                {
                    case 0:
                        Assert.NotEqual(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, Classify(aSol, anArg.C));
                        break;
                    case 1:
                        Assert.Equal(aSol.Radius, anArg.L.DistanceTo(aSol.Location), Tol);
                        break;
                    default:
                        Assert.Equal(aSol.Radius, aSol.Location.DistanceTo(anArg.P), Tol);
                        break;
                }
                if (aSame[k](i) || theSkipTangency == k + 1)
                {
                    continue;
                }
                aTangency[k](i, out double aParSol, out double aParArg, out Point2D aPnt);
                // the tangency point is on the solution, at ParSol
                Assert.Equal(aSol.Radius, aSol.Location.DistanceTo(aPnt), Tol);
                AssertNear(aPnt, global::Kerf.Engine.Geometry.ElCLib.Value(aParSol, aSol));
                switch (anArg.Kind)
                {
                    case 0:
                        Assert.Equal(anArg.C.Radius, anArg.C.Location.DistanceTo(aPnt), Tol);
                        AssertNear(aPnt, global::Kerf.Engine.Geometry.ElCLib.Value(aParArg, anArg.C));
                        break;
                    case 1:
                        Assert.Equal(0.0, anArg.L.DistanceTo(aPnt), Tol);
                        AssertNear(aPnt, global::Kerf.Engine.Geometry.ElCLib.Value(aParArg, anArg.L));
                        break;
                    default:
                        AssertNear(anArg.P, aPnt);
                        Assert.Equal(0.0, aParArg);
                        break;
                }
            }
        }
    }

    private static readonly global::Kerf.Engine.Construction.Constraints.Qualifier[] theCircQualifiers = [global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside];
    private static readonly global::Kerf.Engine.Construction.Constraints.Qualifier[] theLinQualifiers = [global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside];

    //=========================================================================
    // Three circles
    //=========================================================================

    private static readonly Circle2D C1 = Circ(0.0, 0.0, 1.0);
    private static readonly Circle2D C2 = Circ(5.0, 0.0, 1.5);
    private static readonly Circle2D C3 = Circ(2.0, 4.0, 0.75);

    [Fact]
    public void ThreeCircles_Unqualified_EightApolloniusSolutions()
    {
        AnalyticCirc2d3Tan aSolver = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(C1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(C2), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(C3), Tol);
        CheckSolutions(aSolver, Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(C1)), Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(C2)),
                       Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(C3)));
        Assert.Equal(8, aSolver.SolutionCount);

        // the eight solutions are the eight combinations of outside and enclosing
        HashSet<(global::Kerf.Engine.Construction.Constraints.Qualifier, global::Kerf.Engine.Construction.Constraints.Qualifier, global::Kerf.Engine.Construction.Constraints.Qualifier)> aCombinations = [];
        for (int i = 1; i <= 8; i++)
        {
            Circle2D aSol = aSolver.ThisSolution(i);
            aCombinations.Add((Classify(aSol, C1), Classify(aSol, C2), Classify(aSol, C3)));
            aSolver.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out _, out _);
            Assert.Equal(Classify(aSol, C1), aQ1);
        }
        Assert.Equal(8, aCombinations.Count);
    }

    [Fact]
    public void ThreeCircles_Qualified_KeepOnlyMatchingSolutions()
    {
        int aTotal = 0;
        foreach (global::Kerf.Engine.Construction.Constraints.Qualifier aQ1 in theCircQualifiers)
        {
            foreach (global::Kerf.Engine.Construction.Constraints.Qualifier aQ2 in theCircQualifiers)
            {
                foreach (global::Kerf.Engine.Construction.Constraints.Qualifier aQ3 in theCircQualifiers)
                {
                    global::Kerf.Engine.Construction.Constraints.QualifiedCirc aQC1 = new(C1, aQ1), aQC2 = new(C2, aQ2), aQC3 = new(C3, aQ3);
                    AnalyticCirc2d3Tan aSolver = new(aQC1, aQC2, aQC3, Tol);
                    CheckSolutions(aSolver, Arg.Of(aQC1), Arg.Of(aQC2), Arg.Of(aQC3));
                    for (int i = 1; i <= aSolver.SolutionCount; i++)
                    {
                        Circle2D aSol = aSolver.ThisSolution(i);
                        Assert.Equal(aQ1, Classify(aSol, C1));
                        Assert.Equal(aQ2, Classify(aSol, C2));
                        Assert.Equal(aQ3, Classify(aSol, C3));
                        aSolver.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aW1, out global::Kerf.Engine.Construction.Constraints.Qualifier aW2, out global::Kerf.Engine.Construction.Constraints.Qualifier aW3);
                        Assert.Equal((aQ1, aQ2, aQ3), (aW1, aW2, aW3));
                    }
                    aTotal += aSolver.SolutionCount;
                }
            }
        }
        // the qualified runs partition the eight unqualified solutions
        Assert.Equal(8, aTotal);
    }

    [Fact]
    public void ThreeCircles_OutsideAndEnclosing_OneSolutionEach()
    {
        AnalyticCirc2d3Tan anInner = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(C1), global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(C2), global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(C3), Tol);
        Assert.Equal(1, anInner.SolutionCount);
        AnalyticCirc2d3Tan anOuter = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(C1), global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(C2), global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosing(C3), Tol);
        Assert.Equal(1, anOuter.SolutionCount);
        Assert.True(anOuter.ThisSolution(1).Radius > anInner.ThisSolution(1).Radius);
        AnalyticCirc2d3Tan anEnclosed = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(C1), global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(C2), global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(C3), Tol);
        Assert.Equal(0, anEnclosed.SolutionCount);
    }

    [Fact]
    public void ThreeCircles_CoincidentArguments_NotDone()
    {
        AnalyticCirc2d3Tan aSolver = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(C1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(C1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(C3), Tol);
        Assert.False(aSolver.IsDone);
        EngineAssert.Throws<NotDoneException>(() => aSolver.ThisSolution(1));
    }

    [Fact]
    public void ThreeCircles_TheSame_WhenAnArgumentIsASolution()
    {
        // C3 is tangent to C1 and C2, so it is one of the solutions, the same as the third argument
        Circle2D aA = Circ(0.0, 0.0, 1.0);
        Circle2D aB = Circ(4.0, 0.0, 1.0);
        Circle2D aC = Circ(2.0, 0.0, 1.0);
        AnalyticCirc2d3Tan aSolver = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aA), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aB), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), Tol);
        Assert.True(aSolver.IsDone);
        bool aFound = false;
        for (int i = 1; i <= aSolver.SolutionCount; i++)
        {
            if (aSolver.IsTheSame3(i))
            {
                aFound = true;
                EngineAssert.Throws<NotDoneException>(() => aSolver.Tangency3(i, out _, out _, out _));
            }
        }
        Assert.True(aFound);
    }

    //=========================================================================
    // Two circles and a line
    //=========================================================================

    private static readonly Line2D LX = Lin(0.0, 0.0, 1.0, 0.0);

    [Fact]
    public void TwoCirclesLine_Unqualified()
    {
        Circle2D aA = Circ(0.0, 3.0, 1.0);
        Circle2D aB = Circ(5.0, 2.0, 1.5);
        AnalyticCirc2d3Tan aSolver = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aA), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aB), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(LX), Tol);
        CheckSolutions(aSolver, Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aA)), Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aB)),
                       Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(LX)));
        Assert.True(aSolver.SolutionCount >= 4);
    }

    [Fact]
    public void TwoCirclesLine_Qualified_KeepsOnlySolutionsTangentToAllThree()
    {
        // OCCT also returned circles touching the first circle only (it didn't reject a centre whose radius failed
        // the second circle's or the line's check); every solution now touches all three as asked (decision 1540)
        Circle2D aA = Circ(0.0, 3.0, 1.0);
        Circle2D aB = Circ(5.0, 2.0, 1.5);
        global::Kerf.Engine.Construction.Constraints.QualifiedCirc aQA = global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(aA), aQB = global::Kerf.Engine.Construction.Constraints.GccEnt.Outside(aB);
        global::Kerf.Engine.Construction.Constraints.QualifiedLin aQL = global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(LX);
        AnalyticCirc2d3Tan aSolver = new(aQA, aQB, aQL, Tol);
        CheckSolutions(aSolver, Arg.Of(aQA), Arg.Of(aQB), Arg.Of(aQL));
        Assert.True(aSolver.SolutionCount >= 1);
        for (int i = 1; i <= aSolver.SolutionCount; i++)
        {
            Circle2D aSol = aSolver.ThisSolution(i);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, Classify(aSol, aA));
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Outside, Classify(aSol, aB));
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, Classify(aSol, LX));
        }
    }

    [Fact]
    public void CircleTwoLines_Unqualified()
    {
        Circle2D aC = Circ(5.0, 5.0, 1.0);
        Line2D aLY = Lin(0.0, 0.0, 0.0, 1.0);
        AnalyticCirc2d3Tan aSolver = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(LX), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aLY), Tol);
        CheckSolutions(aSolver, Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC)), Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(LX)),
                       Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aLY)));
        Assert.True(aSolver.SolutionCount >= 4);
        for (int i = 1; i <= aSolver.SolutionCount; i++)
        {
            // the centres are on the bisectors of the axes
            Point2D aP = aSolver.ThisSolution(i).Location;
            Assert.Equal(Math.Abs(aP.X), Math.Abs(aP.Y), Tol);
        }
    }

    //=========================================================================
    // Three lines
    //=========================================================================

    // a counter-clockwise right triangle (0,0), (4,0), (0,3): its inside is on the left of each side
    private static readonly Line2D T1 = Lin(0.0, 0.0, 1.0, 0.0);
    private static readonly Line2D T2 = Lin(4.0, 0.0, -4.0, 3.0);
    private static readonly Line2D T3 = Lin(0.0, 3.0, 0.0, -1.0);

    [Fact]
    public void ThreeLines_Triangle_IncircleAndThreeExcircles()
    {
        AnalyticCirc2d3Tan aSolver = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(T1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(T2), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(T3), Tol);
        CheckSolutions(aSolver, Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(T1)), Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(T2)),
                       Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(T3)));
        Assert.Equal(4, aSolver.SolutionCount);
        double[] aRadii = new double[4];
        for (int i = 1; i <= 4; i++)
        {
            Circle2D aSol = aSolver.ThisSolution(i);
            aRadii[i - 1] = aSol.Radius;
            aSolver.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ3);
            Assert.Equal((Classify(aSol, T1), Classify(aSol, T2), Classify(aSol, T3)), (aQ1, aQ2, aQ3));
        }
        Array.Sort(aRadii);
        // inradius (a + b - c) / 2 = 1, exradii 2, 3 and 6 for the 3-4-5 triangle
        Assert.Equal([1.0, 2.0, 3.0, 6.0], aRadii.Select(r => Math.Round(r, 9)).ToArray());
    }

    [Fact]
    public void ThreeLines_Enclosed_Incircle()
    {
        global::Kerf.Engine.Construction.Constraints.QualifiedLin aQ1 = global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(T1), aQ2 = global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(T2), aQ3 = global::Kerf.Engine.Construction.Constraints.GccEnt.Enclosed(T3);
        AnalyticCirc2d3Tan aSolver = new(aQ1, aQ2, aQ3, Tol);
        CheckSolutions(aSolver, Arg.Of(aQ1), Arg.Of(aQ2), Arg.Of(aQ3));
        Assert.Equal(1, aSolver.SolutionCount);
        Assert.Equal(1.0, aSolver.ThisSolution(1).Radius, Tol);
        AssertNear(new Point2D(1.0, 1.0), aSolver.ThisSolution(1).Location);
    }

    [Fact]
    public void ThreeLines_Qualified_PartitionTheSolutions()
    {
        int aTotal = 0;
        foreach (global::Kerf.Engine.Construction.Constraints.Qualifier aQ1 in theLinQualifiers)
        {
            foreach (global::Kerf.Engine.Construction.Constraints.Qualifier aQ2 in theLinQualifiers)
            {
                foreach (global::Kerf.Engine.Construction.Constraints.Qualifier aQ3 in theLinQualifiers)
                {
                    global::Kerf.Engine.Construction.Constraints.QualifiedLin aQL1 = new(T1, aQ1), aQL2 = new(T2, aQ2), aQL3 = new(T3, aQ3);
                    AnalyticCirc2d3Tan aSolver = new(aQL1, aQL2, aQL3, Tol);
                    CheckSolutions(aSolver, Arg.Of(aQL1), Arg.Of(aQL2), Arg.Of(aQL3));
                    for (int i = 1; i <= aSolver.SolutionCount; i++)
                    {
                        Circle2D aSol = aSolver.ThisSolution(i);
                        Assert.Equal((aQ1, aQ2, aQ3), (Classify(aSol, T1), Classify(aSol, T2), Classify(aSol, T3)));
                    }
                    aTotal += aSolver.SolutionCount;
                }
            }
        }
        Assert.Equal(4, aTotal);
    }

    [Fact]
    public void ThreeLines_EnclosingQualifier_Throws()
    {
        EngineAssert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(
            () => new AnalyticCirc2d3Tan(new global::Kerf.Engine.Construction.Constraints.QualifiedLin(T1, global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(T2),
                                        global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(T3), Tol));
    }

    //=========================================================================
    // With points
    //=========================================================================

    [Fact]
    public void TwoCirclesPoint()
    {
        Circle2D aA = Circ(0.0, 0.0, 1.0);
        Circle2D aB = Circ(6.0, 0.0, 2.0);
        Point2D aP = new(3.0, 4.0);
        AnalyticCirc2d3Tan aSolver = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aA), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aB), aP, Tol);
        CheckSolutions(aSolver, Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aA)), Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aB)), Arg.Of(aP));
        Assert.True(aSolver.SolutionCount >= 2);
        for (int i = 1; i <= aSolver.SolutionCount; i++)
        {
            aSolver.WhichQualifier(i, out _, out _, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ3);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, aQ3);
        }
    }

    [Fact]
    public void CircleLinePoint()
    {
        Circle2D aC = Circ(0.0, 4.0, 1.0);
        Point2D aP = new(4.0, 2.0);
        AnalyticCirc2d3Tan aSolver = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(LX), aP, Tol);
        // OCCT put the first tangency point on the far side of the solution unless the solution was enclosed in the
        // circle; it is where the two touch (decision 1540), which CheckSolutions checks
        CheckSolutions(aSolver, Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC)), Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(LX)), Arg.Of(aP));
        Assert.Equal(4, aSolver.SolutionCount);
    }

    [Fact]
    public void TwoLinesPoint_TwoCirclesTangentToTheAxes()
    {
        Line2D aLY = Lin(0.0, 0.0, 0.0, 1.0);
        Point2D aP = new(1.0, 2.0);
        AnalyticCirc2d3Tan aSolver = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(LX), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aLY), aP, Tol);
        CheckSolutions(aSolver, Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(LX)), Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aLY)), Arg.Of(aP));
        // centres (t, t) with (1 - t)^2 + (2 - t)^2 = t^2: t = 1 or 5
        Assert.Equal(2, aSolver.SolutionCount);
        double[] aRadii = [aSolver.ThisSolution(1).Radius, aSolver.ThisSolution(2).Radius];
        Array.Sort(aRadii);
        Assert.Equal(1.0, aRadii[0], Tol);
        Assert.Equal(5.0, aRadii[1], Tol);
    }

    [Fact]
    public void CircleTwoPoints()
    {
        Circle2D aC = Circ(0.0, 0.0, 1.0);
        Point2D aP2 = new(3.0, 0.0);
        Point2D aP3 = new(3.0, 2.0);
        AnalyticCirc2d3Tan aSolver = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC), aP2, aP3, Tol);
        CheckSolutions(aSolver, Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(aC)), Arg.Of(aP2), Arg.Of(aP3));
        Assert.Equal(2, aSolver.SolutionCount);
    }

    [Fact]
    public void LineTwoPoints_TwoSolutions()
    {
        Point2D aP2 = new(0.0, 1.0);
        Point2D aP3 = new(0.0, 3.0);
        AnalyticCirc2d3Tan aSolver = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(LX), aP2, aP3, Tol);
        CheckSolutions(aSolver, Arg.Of(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(LX)), Arg.Of(aP2), Arg.Of(aP3));
        // centres (x, 2) at distance 2 from (0, 1): x = +-sqrt(3)
        Assert.Equal(2, aSolver.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(2.0, aSolver.ThisSolution(i).Radius, Tol);
            Assert.Equal(Math.Sqrt(3.0), Math.Abs(aSolver.ThisSolution(i).Location.X), Tol);
        }
    }

    [Fact]
    public void ThreePoints_Circumcircle()
    {
        Point2D aP1 = new(0.0, 0.0), aP2 = new(2.0, 0.0), aP3 = new(0.0, 2.0);
        AnalyticCirc2d3Tan aSolver = new(aP1, aP2, aP3, Tol);
        CheckSolutions(aSolver, Arg.Of(aP1), Arg.Of(aP2), Arg.Of(aP3));
        Assert.Equal(1, aSolver.SolutionCount);
        AssertNear(new Point2D(1.0, 1.0), aSolver.ThisSolution(1).Location);
        Assert.Equal(Math.Sqrt(2.0), aSolver.ThisSolution(1).Radius, Tol);
        aSolver.WhichQualifier(1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ3);
        Assert.Equal((global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier), (aQ1, aQ2, aQ3));
    }

    [Fact]
    public void ThreePoints_Collinear_NoSolution()
    {
        AnalyticCirc2d3Tan aSolver = new(new Point2D(0.0, 0.0), new Point2D(1.0, 0.0), new Point2D(3.0, 0.0), Tol);
        Assert.True(aSolver.IsDone);
        Assert.Equal(0, aSolver.SolutionCount);
    }

    [Fact]
    public void ThreePoints_Coincident_ZeroRadius()
    {
        Point2D aP = new(1.0, 2.0);
        AnalyticCirc2d3Tan aSolver = new(aP, aP, aP, Tol);
        Assert.Equal(1, aSolver.SolutionCount);
        Assert.Equal(0.0, aSolver.ThisSolution(1).Radius);
    }

    //=========================================================================
    // Accessors
    //=========================================================================

    [Fact]
    public void Accessors_OutOfRange_Throw()
    {
        AnalyticCirc2d3Tan aSolver = new(new Point2D(0.0, 0.0), new Point2D(2.0, 0.0), new Point2D(0.0, 2.0), Tol);
        EngineAssert.Throws<OutOfRangeException>(() => aSolver.ThisSolution(0));
        EngineAssert.Throws<OutOfRangeException>(() => aSolver.ThisSolution(2));
        EngineAssert.Throws<OutOfRangeException>(() => aSolver.WhichQualifier(2, out _, out _, out _));
        EngineAssert.Throws<OutOfRangeException>(() => aSolver.Tangency1(2, out _, out _, out _));
        EngineAssert.Throws<OutOfRangeException>(() => aSolver.IsTheSame3(0));
    }

    [Fact]
    public void Constructor_NoQualifier_Throws()
    {
        EngineAssert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(
            () => new AnalyticCirc2d3Tan(new global::Kerf.Engine.Construction.Constraints.QualifiedCirc(C1, global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(C2),
                                        global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(C3), Tol));
        EngineAssert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(
            () => new AnalyticCirc2d3Tan(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(C1), new global::Kerf.Engine.Construction.Constraints.QualifiedLin(LX, global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier),
                                        new Point2D(1.0, 1.0), Tol));
    }
}
