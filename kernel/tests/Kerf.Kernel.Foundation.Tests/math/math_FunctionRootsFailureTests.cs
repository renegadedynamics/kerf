// Port of: src/FoundationClasses/TKMath/GTests/math_FunctionRoots_Test.cxx (math_RootFailureTest, upstream 71658f6c8f)
// Open CASCADE Technology after V8_0_1 (71658f6c8f). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// Upstream 71658f6c8f's tests of math_FunctionRoots (decision 2390): an extremum near zero that misses the target, the
/// states of the roots, and callbacks that fail or give NaN, which are no longer a result. Its test of a root on a
/// sample with the function's return beside it isn't ported, as that part of the commit isn't.
/// </summary>
public class math_FunctionRootsFailureTests
{
    // x (x - gap) + target, or the same of 1 - x: a root on the sample at 0 (or 1) and another a gap away, inside the
    // first interval
    private sealed class ReturnCrossingFunction(double theTarget, bool theReverse = false, double theGap = 0.01)
        : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theValue)
        {
            double aParameter = theReverse ? 1.0 - theX : theX;
            theValue = aParameter * (aParameter - theGap) + theTarget;
            return true;
        }

        public override bool Derivative(double theX, out double theValue)
        {
            double aParameter = theReverse ? 1.0 - theX : theX;
            theValue = (2.0 * aParameter - theGap) * (theReverse ? -1.0 : 1.0);
            return true;
        }

        public override bool Values(double theX, out double theValue, out double theDerivative)
        {
            Value(theX, out theValue);
            return Derivative(theX, out theDerivative);
        }
    }

    // Fails, or gives NaN, everywhere
    private sealed class FailedRootFunction(bool theNonFinite) : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theValue)
        {
            theValue = double.NaN;
            return theNonFinite;
        }

        public override bool Derivative(double theX, out double theValue)
        {
            theValue = double.NaN;
            return false;
        }

        public override bool Values(double theX, out double theValue, out double theDerivative)
        {
            theDerivative = double.NaN;
            return Value(theX, out theValue) && Derivative(theX, out theDerivative);
        }
    }

    // (x - 0.51)^2 + 0.001, whose derivative fails at its minimum
    private sealed class InteriorDerivativeFailure(bool theNonFinite) : FunctionWithDerivative
    {
        public bool HasFailed { get; private set; }

        public override bool Value(double theX, out double theValue)
        {
            theValue = (theX - 0.51) * (theX - 0.51) + 0.001;
            return true;
        }

        public override bool Derivative(double theX, out double theValue)
        {
            if (Math.Abs(theX - 0.51) < 1.0e-8)
            {
                HasFailed = true;
                theValue = double.NaN;
                return theNonFinite;
            }
            theValue = 2.0 * (theX - 0.51);
            return true;
        }

        public override bool Values(double theX, out double theValue, out double theDerivative)
        {
            Value(theX, out theValue);
            return Derivative(theX, out theDerivative);
        }
    }

    // -(x - 0.51)^2: its maximum, 0, is 0.001 below the target asked
    private sealed class NegativeQuadraticFunction : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theValue)
        {
            theValue = -(theX - 0.51) * (theX - 0.51);
            return true;
        }

        public override bool Derivative(double theX, out double theValue)
        {
            theValue = -2.0 * (theX - 0.51);
            return true;
        }

        public override bool Values(double theX, out double theValue, out double theDerivative)
        {
            Value(theX, out theValue);
            return Derivative(theX, out theDerivative);
        }
    }

    // x^2 - 4, whose state is the side of 0 last evaluated
    private sealed class QuadraticWithState : FunctionWithDerivative
    {
        private double _last;

        public override bool Value(double theX, out double theF)
        {
            _last = theX;
            theF = theX * theX - 4.0;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = 2.0 * theX;
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            _last = theX;
            theF = theX * theX - 4.0;
            theD = 2.0 * theX;
            return true;
        }

        public override int GetStateNumber() => _last < 0.0 ? 1 : 2;
    }

    [Fact]
    public void An_extremum_near_zero_is_no_root_of_another_target()
    {
        // Upstream's SampledRootAndReturnCrossing, its maximum: -(x - 0.51)^2 is at most 0, a thousandth under the target
        NegativeQuadraticFunction aMaximum = new();
        FunctionRoots aNoRoots = new(aMaximum, 0.0, 1.0, 10, 1.0e-12, 1.0e-12, 0.0, 0.001);
        Assert.True(aNoRoots.IsDone);
        Assert.Equal(0, aNoRoots.SolutionCount);

        // The second root rounds to the sampled upper bound.
        ReturnCrossingFunction aCloseRoots = new(0.0, true, Precision.Computational() * 0.25);
        FunctionRoots aClose = new(aCloseRoots, 0.0, 1.0, 10, Precision.PConfusion());
        Assert.True(aClose.IsDone);
        Assert.Equal(1, aClose.SolutionCount);
        Assert.Equal(1.0, aClose.Value(1));
    }

    [Fact]
    public void CallbackFailureIsNotAResult()
    {
        foreach (bool isNonFinite in new[] { false, true })
        {
            FailedRootFunction aFunction = new(isNonFinite);
            FunctionRoots aRoots = new(aFunction, 0.0, 1.0, 10, 1.0e-8, 1.0e-8, 0.0);
            Assert.False(aRoots.IsDone);
        }
    }

    [Fact]
    public void InteriorDerivativeFailureIsNotAnEmptySuccess()
    {
        foreach (bool isNonFinite in new[] { false, true })
        {
            InteriorDerivativeFailure aFunction = new(isNonFinite);
            FunctionRoots aRoots = new(aFunction, 0.0, 1.0, 10, 1.0e-12, 1.0e-12, 0.0);
            Assert.True(aFunction.HasFailed);
            Assert.False(aRoots.IsDone);
        }
    }

    [Fact]
    public void StateNumberAccess()
    {
        QuadraticWithState aFunc = new();
        FunctionRoots aRootFinder = new(aFunc, -5.0, 5.0, 20, 1.0e-10, 1.0e-10);
        Assert.True(aRootFinder.IsDone);
        Assert.Equal(2, aRootFinder.SolutionCount);
        Assert.Equal(1, aRootFinder.StateNumber(1));
        Assert.Equal(2, aRootFinder.StateNumber(2));
    }

    [Fact]
    public void ShiftedTarget()
    {
        // F(x) - (-3) = x*x - 1 has two simple roots.
        double aTolerance = Precision.PConfusion();
        QuadraticWithState aFunc = new();
        FunctionRoots aRootFinder = new(aFunc, -2.0, 2.0, 20, aTolerance, aTolerance, 0.0, -3.0);
        Assert.True(aRootFinder.IsDone);
        Assert.Equal(2, aRootFinder.SolutionCount);
        Assert.Equal(-1.0, aRootFinder.Value(1), aTolerance);
        Assert.Equal(1.0, aRootFinder.Value(2), aTolerance);
    }
}
