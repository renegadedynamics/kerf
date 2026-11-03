// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// AdvApprox_ApproxAFunction, AdvApprox_SimpleApprox and the cutting tools on functions with known results
/// (OCCT has no GTests for AdvApprox): polynomials are reproduced, smooth functions are approximated within their
/// tolerances, and the B-spline results have the requested continuity.
/// </summary>
public class AdvApprox_ApproxAFunctionTests
{
    /// <summary>A function given by a delegate filling the values or derivatives.</summary>
    private sealed class Evaluator(Action<double, int, Span<double>> theF) : EvaluatorFunction
    {
        public int NbCalls;

        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Parameter,
                                      int DerivativeRequest, Span<double> Result, out int ErrorCode)
        {
            NbCalls++;
            theF(Parameter, DerivativeRequest, Result[..Dimension]);
            ErrorCode = 0;
        }
    }

    private sealed class FailingEvaluator : EvaluatorFunction
    {
        public override void Evaluate(int Dimension, ReadOnlySpan<double> StartEnd, double Parameter,
                                      int DerivativeRequest, Span<double> Result, out int ErrorCode)
        {
            Result[..Dimension].Clear();
            ErrorCode = 1;
        }
    }

    // p(t) = 1 + 2t - 3t^2 + 0.5t^3 and its derivatives
    private static void Cubic(double t, int theDeriv, Span<double> theRes)
    {
        theRes[0] = theDeriv switch
        {
            0 => 1 + 2 * t - 3 * t * t + 0.5 * t * t * t,
            1 => 2 - 6 * t + 1.5 * t * t,
            2 => -6 + 3 * t,
            _ => 0,
        };
    }

    private static double SinD(double t, int theDeriv) => theDeriv switch
    {
        0 => Math.Sin(t),
        1 => Math.Cos(t),
        2 => -Math.Sin(t),
        _ => -Math.Cos(t),
    };

    private static double CosD(double t, int theDeriv) => SinD(t + Math.PI / 2, theDeriv);

    private static NCollection_HArray1<double> Tol(params double[] theValues)
    {
        NCollection_HArray1<double> aTol = new(1, theValues.Length);
        for (int i = 0; i < theValues.Length; i++)
        {
            aTol[i + 1] = theValues[i];
        }
        return aTol;
    }

    private static double Eval1d(ApproxAFunction theApprox, int theIndex, double theU)
    {
        NCollection_Array1<double> aPoles = new(1, theApprox.NbPoles());
        theApprox.Poles1d(theIndex, aPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(theU, 0, theApprox.Degree(), false, aPoles, null, theApprox.Knots()!,
                    theApprox.Multiplicities(), out double aP);
        return aP;
    }

    private static Point2D Eval2d(ApproxAFunction theApprox, int theIndex, double theU)
    {
        NCollection_Array1<Point2D> aPoles = new(1, theApprox.NbPoles());
        theApprox.Poles2d(theIndex, aPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(theU, 0, theApprox.Degree(), false, aPoles, null, theApprox.Knots()!,
                    theApprox.Multiplicities(), out Point2D aP);
        return aP;
    }

    private static Point3D Eval3d(ApproxAFunction theApprox, int theIndex, double theU)
    {
        NCollection_Array1<Point3D> aPoles = new(1, theApprox.NbPoles());
        theApprox.Poles(theIndex, aPoles);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(theU, 0, theApprox.Degree(), false, aPoles, null, theApprox.Knots()!,
                    theApprox.Multiplicities(), out Point3D aP);
        return aP;
    }

    [Theory]
    [InlineData(C0)]
    [InlineData(C1)]
    [InlineData(C2)]
    public void Polynomial_is_reproduced(Continuity theContinuity)
    {
        Evaluator aF = new(Cubic);
        ApproxAFunction anApprox = new(1, 0, 0, Tol(1e-9), null, null, 0.0, 2.0, theContinuity, 10, 5, aF);

        Assert.True(anApprox.IsDone);
        Assert.True(anApprox.HasResult);
        Assert.Equal(1, anApprox.NumSubSpaces(1));
        Assert.Equal(0, anApprox.NumSubSpaces(2));
        Assert.True(anApprox.MaxError(1, 1) <= 1e-9);
        Assert.True(anApprox.AverageError(1, 1) <= anApprox.MaxError(1, 1));
        // One segment is enough for a cubic
        Assert.Equal(2, anApprox.KnotCount);
        Assert.Equal(0.0, anApprox.Knots()![1]);
        Assert.Equal(2.0, anApprox.Knots()![2]);

        Span<double> anExpected = stackalloc double[1];
        for (int i = 0; i <= 40; i++)
        {
            double t = 2.0 * i / 40;
            Cubic(t, 0, anExpected);
            Assert.Equal(anExpected[0], Eval1d(anApprox, 1, t), 1e-12);
        }
    }

    [Fact]
    public void Sine_is_approximated_within_tolerance()
    {
        const double aTol = 1e-7;
        Evaluator aF = new((t, d, r) => r[0] = SinD(t, d));
        ApproxAFunction anApprox =
            new(1, 0, 0, Tol(aTol), null, null, 0.0, 2 * Math.PI, C1, 8, 20, aF);

        Assert.True(anApprox.IsDone);
        Assert.True(anApprox.MaxError(1, 1) <= aTol);
        Assert.True(anApprox.KnotCount > 2, "a degree 8 polynomial can't approximate sin over a period to 1e-7");
        Assert.True(anApprox.Degree() <= 8);

        double aMaxErr = 0;
        for (int i = 0; i <= 500; i++)
        {
            double t = 2 * Math.PI * i / 500;
            aMaxErr = Math.Max(aMaxErr, Math.Abs(Eval1d(anApprox, 1, t) - Math.Sin(t)));
        }
        Assert.True(aMaxErr <= aTol, $"max error {aMaxErr}");
    }

    [Fact]
    public void Several_subspaces_are_approximated_together()
    {
        // 1D: exp(t); 2D: a circle of radius 2; 3D: a helix
        Evaluator aF = new((t, d, r) =>
        {
            r[0] = Math.Exp(t);
            r[1] = 2 * CosD(t, d);
            r[2] = 2 * SinD(t, d);
            r[3] = CosD(t, d);
            r[4] = SinD(t, d);
            r[5] = d switch { 0 => 0.5 * t, 1 => 0.5, _ => 0 };
        });
        ApproxAFunction anApprox =
            new(1, 1, 1, Tol(1e-6), Tol(1e-5), Tol(1e-4), 0.0, 3.0, C2, 11, 30, aF);

        Assert.True(anApprox.IsDone);
        Assert.Equal(1, anApprox.NumSubSpaces(1));
        Assert.Equal(1, anApprox.NumSubSpaces(2));
        Assert.Equal(1, anApprox.NumSubSpaces(3));
        Assert.True(anApprox.MaxError(1, 1) <= 1e-6);
        Assert.True(anApprox.MaxError(2, 1) <= 1e-5);
        Assert.True(anApprox.MaxError(3, 1) <= 1e-4);
        Assert.Equal(anApprox.NbPoles(), anApprox.Poles1d()!.ColLength());
        Assert.Equal(anApprox.NbPoles(), anApprox.Poles2d()!.ColLength());
        Assert.Equal(anApprox.NbPoles(), anApprox.Poles()!.ColLength());

        double anErr1 = 0, anErr2 = 0, anErr3 = 0;
        for (int i = 0; i <= 300; i++)
        {
            double t = 3.0 * i / 300;
            anErr1 = Math.Max(anErr1, Math.Abs(Eval1d(anApprox, 1, t) - Math.Exp(t)));
            anErr2 = Math.Max(anErr2, Eval2d(anApprox, 1, t).Distance(new Point2D(2 * Math.Cos(t), 2 * Math.Sin(t))));
            anErr3 = Math.Max(anErr3, Eval3d(anApprox, 1, t).Distance(new Point3D(Math.Cos(t), Math.Sin(t), 0.5 * t)));
        }
        Assert.True(anErr1 <= 1e-6, $"1D error {anErr1}");
        Assert.True(anErr2 <= 1e-5, $"2D error {anErr2}");
        Assert.True(anErr3 <= 1e-4, $"3D error {anErr3}");
    }

    [Theory]
    [InlineData(C0, 0)]
    [InlineData(C1, 1)]
    [InlineData(C2, 2)]
    public void Result_has_the_requested_continuity(Continuity theContinuity, int theOrder)
    {
        Evaluator aF = new((t, d, r) => r[0] = SinD(3 * t, d) * Math.Pow(3, d));
        ApproxAFunction anApprox = new(1, 0, 0, Tol(1e-8), null, null, 0.0, 4.0, theContinuity, 7, 50, aF);

        Assert.True(anApprox.IsDone);
        NCollection_HArray1<int> aMults = anApprox.Multiplicities()!;
        Assert.True(aMults.Length() > 2);
        Assert.Equal(anApprox.Degree() + 1, aMults[aMults.Lower()]);
        Assert.Equal(anApprox.Degree() + 1, aMults[aMults.Upper()]);
        for (int i = aMults.Lower() + 1; i < aMults.Upper(); i++)
        {
            Assert.True(aMults[i] <= anApprox.Degree() - theOrder,
                        $"multiplicity {aMults[i]} at a knot of a degree {anApprox.Degree()} C{theOrder} B-spline");
        }
    }

    [Fact]
    public void PrefCutting_cuts_at_the_preferred_points()
    {
        // A function with a discontinuous second derivative at 0.7: the C1 approximation must cut there
        Evaluator aF = new((t, d, r) =>
        {
            double s = t - 0.7;
            r[0] = s <= 0 ? 0 : d switch { 0 => s * s * s, 1 => 3 * s * s, 2 => 6 * s, _ => 6 };
        });
        PrefCutting aCut = new(NCollection_Array1<double>.From([0.7]));
        ApproxAFunction anApprox =
            new(1, 0, 0, Tol(1e-10), null, null, 0.0, 2.0, C1, 6, 10, aF, aCut);

        Assert.True(anApprox.IsDone);
        NCollection_HArray1<double> aKnots = anApprox.Knots()!;
        Assert.Contains(0.7, aKnots.AsSpan().ToArray());
        // A cubic on each side: two pieces, each exact
        Assert.Equal(3, aKnots.Length());
        Assert.Equal(0.0, Eval1d(anApprox, 1, 0.3), 1e-12);
        Assert.Equal(Math.Pow(1.3, 3), Eval1d(anApprox, 1, 2.0), 1e-10);
    }

    [Fact]
    public void Accessors_return_copies()
    {
        Evaluator aF = new(Cubic);
        ApproxAFunction anApprox = new(1, 0, 0, Tol(1e-9), null, null, 0.0, 2.0, C1, 10, 5, aF);

        NCollection_HArray1<double> aKnots = anApprox.Knots()!;
        aKnots[1] = 42;
        Assert.Equal(0.0, anApprox.Knots()![1]);

        NCollection_HArray2<double> aPoles = anApprox.Poles1d()!;
        double aFirst = aPoles[1, 1];
        aPoles[1, 1] = 42;
        Assert.Equal(aFirst, anApprox.Poles1d()![1, 1]);

        NCollection_HArray1<double> anErrors = anApprox.MaxError(1)!;
        anErrors[1] = 42;
        Assert.NotEqual(42, anApprox.MaxError(1, 1));

        Assert.Null(anApprox.Poles2d());
        Assert.Null(anApprox.Poles());
        Assert.Null(anApprox.MaxError(2));
        Assert.Null(anApprox.AverageError(3));
        Assert.ThrowsAny<OutOfRangeException>(() => anApprox.MaxError(4));
        Assert.ThrowsAny<ConstructionErrorException>(() => anApprox.NumSubSpaces(0));
    }

    [Fact]
    public void Invalid_input_throws_and_evaluation_errors_give_no_result()
    {
        Evaluator aF = new(Cubic);
        Assert.ThrowsAny<ConstructionErrorException>(
            () => new ApproxAFunction(0, 0, 0, null, null, null, 0, 1, C1, 5, 5, aF));
        Assert.ThrowsAny<ConstructionErrorException>(
            () => new ApproxAFunction(1, 0, 0, Tol(1e-6), null, null, 1, 0, C1, 5, 5, aF));
        Assert.ThrowsAny<ConstructionErrorException>(
            () => new ApproxAFunction(1, 0, 0, Tol(1e-6), null, null, 0, 1, G1, 5, 5, aF));

        ApproxAFunction anApprox =
            new(1, 0, 0, Tol(1e-6), null, null, 0, 1, C1, 5, 5, new FailingEvaluator());
        Assert.False(anApprox.IsDone);
        Assert.False(anApprox.HasResult);
        Assert.Equal(0, anApprox.NbPoles());
        Assert.Null(anApprox.Knots());
    }

    [Fact]
    public void Unreachable_tolerance_gives_a_result_that_is_not_done()
    {
        // Too few segments for the tolerance: the best result is kept, flagged by the error
        Evaluator aF = new((t, d, r) => r[0] = SinD(10 * t, d) * Math.Pow(10, d));
        ApproxAFunction anApprox = new(1, 0, 0, Tol(1e-12), null, null, 0, 3, C1, 5, 2, aF);
        Assert.True(anApprox.HasResult);
        Assert.True(anApprox.MaxError(1, 1) > 1e-12);
        Assert.True(anApprox.KnotCount <= 3);
    }

    [Fact]
    public void Approximation_checks_its_dimensions()
    {
        Evaluator aF = new(Cubic);
        NCollection_Array1<int> aDims = NCollection_Array1<int>.From([2]);
        NCollection_Array1<double> aTols = NCollection_Array1<double>.From([1e-6]);
        NCollection_Array1<int> aNbCoeffs = new(1, 4);
        NCollection_Array1<double> aCoeffs = new(1, 4 * 6 * 1);
        NCollection_Array1<double> anIntervals = new(1, 5);
        NCollection_Array1<double> anErrMax = new(1, 4), anErrAvg = new(1, 4);
        ApproxAFunction.Approximation(1, 1, aDims, 0, 1, aF, new DichoCutting(), 1, 6, 4,
                                                aTols, 1, out _, aNbCoeffs, aCoeffs, anIntervals, anErrMax,
                                                anErrAvg, out int anError);
        Assert.Equal(1, anError);

        aDims[1] = 1;
        ApproxAFunction.Approximation(1, 1, aDims, 0, 1, aF, new DichoCutting(), 1, 6, 4,
                                                aTols, 1, out int aNbCurves, aNbCoeffs, aCoeffs, anIntervals,
                                                anErrMax, anErrAvg, out anError);
        Assert.Equal(0, anError);
        Assert.Equal(1, aNbCurves);
        Assert.Equal(0.0, anIntervals[1]);
        Assert.Equal(1.0, anIntervals[2]);
        // The coefficients are those of the cubic on [-1, 1]: p((t + 1) / 2) = 1.3125 - 0.3125 t - 0.5625 t^2
        // + 0.0625 t^3
        Assert.Equal(4, aNbCoeffs[1]);
        Assert.Equal(1.3125, aCoeffs[1], 1e-12);
        Assert.Equal(-0.3125, aCoeffs[2], 1e-12);
        Assert.Equal(-0.5625, aCoeffs[3], 1e-12);
        Assert.Equal(0.0625, aCoeffs[4], 1e-12);
    }

    [Fact]
    public void Dump_writes_the_errors()
    {
        Evaluator aF = new(Cubic);
        ApproxAFunction anApprox = new(1, 0, 0, Tol(1e-9), null, null, 0.0, 2.0, C1, 10, 5, aF);
        StringWriter aWriter = new();
        anApprox.Dump(aWriter);
        Assert.StartsWith("Dump of ApproxAFunction\nError(s) 1d = \n   ", aWriter.ToString());
    }
}
