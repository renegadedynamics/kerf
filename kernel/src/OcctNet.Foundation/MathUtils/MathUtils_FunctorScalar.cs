// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_FunctorScalar.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// OCCT's solvers are templates over any type with the right members ("Function type with Value(double, double&)
// method"). Here each such member set is an interface, and the solvers are generic methods constrained on it
// (`where Function : MathUtils.IScalarFunction`). A struct functor gets code specialised for it by the JIT, with
// the call inlined as C++ does; a class functor goes through the interface. Functors are passed by value: a struct
// functor whose Value changes its own fields should be a class instead (C++ passes them by reference).
//
// The library functors below are sealed classes. The lambda wrappers take a delegate; a C# lambda passed to
// MakeScalar is written `(double x, out double y) => { ... }` (or `(x, out y) => ...`).

/// <summary>Non-virtual functor classes for scalar (1D) functions.</summary>
public static partial class MathUtils
{
    /// <summary>A scalar function: C++'s "type with <c>Value(double theX, double&amp; theF)</c> method".</summary>
    public interface IScalarFunction
    {
        /// <summary>Evaluates the function at theX; false if it could not be evaluated.</summary>
        bool Value(double theX, out double theY);
    }

    /// <summary>A scalar function with its derivative: C++'s "type with <c>Values(double, double&amp;, double&amp;)</c> method".</summary>
    public interface IScalarFunctionWithDerivative
    {
        /// <summary>Evaluates the function and its derivative at theX; false if they could not be evaluated.</summary>
        bool Values(double theX, out double theY, out double theDY);
    }

    /// <summary>Signature of the callable wrapped by <see cref="ScalarLambda"/>: bool(double theX, double&amp; theY).</summary>
    public delegate bool ScalarLambdaFunction(double theX, out double theY);

    /// <summary>Signature of the callable wrapped by <see cref="ScalarLambdaWithDerivative"/>.</summary>
    public delegate bool ScalarLambdaWithDerivativeFunction(double theX, out double theY, out double theDY);

    /// <summary>Lambda wrapper for scalar functions with value only.</summary>
    /// <remarks>C++ is templated on the lambda type; here it holds a delegate.</remarks>
    public sealed class ScalarLambda : IScalarFunction
    {
        private readonly ScalarLambdaFunction myLambda;

        public ScalarLambda(ScalarLambdaFunction theLambda)
        {
            myLambda = theLambda;
        }

        public bool Value(double theX, out double theY) => myLambda(theX, out theY);
    }

    /// <summary>Lambda wrapper for scalar functions with value and derivative.</summary>
    public sealed class ScalarLambdaWithDerivative : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly ScalarLambdaWithDerivativeFunction myLambda;

        public ScalarLambdaWithDerivative(ScalarLambdaWithDerivativeFunction theLambda)
        {
            myLambda = theLambda;
        }

        public bool Values(double theX, out double theY, out double theDY) => myLambda(theX, out theY, out theDY);

        /// <summary>Evaluates only the function value (for algorithms that don't need derivative).</summary>
        public bool Value(double theX, out double theY)
        {
            return myLambda(theX, out theY, out _);
        }
    }

    /// <summary>Helper function to create ScalarLambda.</summary>
    public static ScalarLambda MakeScalar(ScalarLambdaFunction theLambda)
    {
        return new ScalarLambda(theLambda);
    }

    /// <summary>Helper function to create ScalarLambdaWithDerivative.</summary>
    public static ScalarLambdaWithDerivative MakeScalarWithDerivative(ScalarLambdaWithDerivativeFunction theLambda)
    {
        return new ScalarLambdaWithDerivative(theLambda);
    }

    /// <summary>Polynomial functor: f(x) = sum(a[i] * x^i), coefficients in ascending power order.</summary>
    public sealed class Polynomial : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly math_Vector myCoeffs;

        /// <summary>Constructor from a coefficient list (C++'s initializer list), ascending power order.</summary>
        public Polynomial(params ReadOnlySpan<double> theCoeffs)
        {
            myCoeffs = new math_Vector(0, theCoeffs.Length - 1);
            int anIdx = 0;
            foreach (double aCoeff in theCoeffs)
            {
                myCoeffs[anIdx++] = aCoeff;
            }
        }

        /// <summary>Constructor from math_Vector (copied), ascending power order.</summary>
        public Polynomial(math_Vector theCoeffs)
        {
            myCoeffs = new math_Vector(theCoeffs);
        }

        /// <summary>Evaluates polynomial at theX using Horner's method.</summary>
        public bool Value(double theX, out double theY)
        {
            if (myCoeffs.Length() <= 0)
            {
                theY = 0.0;
                return true;
            }

            // Horner's method: p(x) = a[0] + x*(a[1] + x*(a[2] + ...))
            int aLast = myCoeffs.Upper();
            theY = myCoeffs[aLast];
            for (int i = aLast - 1; i >= myCoeffs.Lower(); --i)
            {
                theY = theY * theX + myCoeffs[i];
            }
            return true;
        }

        /// <summary>Evaluates polynomial and its derivative at theX.</summary>
        public bool Values(double theX, out double theY, out double theDY)
        {
            if (myCoeffs.Length() <= 0)
            {
                theY = 0.0;
                theDY = 0.0;
                return true;
            }

            int n = myCoeffs.Length();
            if (n == 1)
            {
                theY = myCoeffs[myCoeffs.Lower()];
                theDY = 0.0;
                return true;
            }

            // Horner's method for value and derivative simultaneously
            int aLower = myCoeffs.Lower();
            int aLast = myCoeffs.Upper();
            theY = myCoeffs[aLast];
            theDY = 0.0;
            for (int i = aLast - 1; i >= aLower; --i)
            {
                theDY = theDY * theX + theY;
                theY = theY * theX + myCoeffs[i];
            }
            return true;
        }

        /// <summary>Returns the degree of the polynomial (number of coefficients - 1).</summary>
        public int Degree() => myCoeffs.Length() <= 0 ? 0 : myCoeffs.Length() - 1;

        /// <summary>Returns coefficient by index (0 = constant term), 0 out of range.</summary>
        public double Coefficient(int theIndex)
        {
            int aIdx = myCoeffs.Lower() + theIndex;
            return (aIdx >= myCoeffs.Lower() && aIdx <= myCoeffs.Upper()) ? myCoeffs[aIdx] : 0.0;
        }
    }

    /// <summary>Rational function functor: f(x) = P(x) / Q(x).</summary>
    public sealed class Rational : IScalarFunction
    {
        private readonly math_Vector myNum;
        private readonly math_Vector myDenom;

        /// <summary>Constructor from math_Vector (copied), ascending power order.</summary>
        public Rational(math_Vector theNum, math_Vector theDenom)
        {
            myNum = new math_Vector(theNum);
            myDenom = new math_Vector(theDenom);
        }

        /// <summary>Constructor from coefficient lists (C++'s initializer lists).</summary>
        public Rational(ReadOnlySpan<double> theNum, ReadOnlySpan<double> theDenom)
        {
            myNum = new math_Vector(0, theNum.Length - 1);
            myDenom = new math_Vector(0, theDenom.Length - 1);
            int anIdx = 0;
            foreach (double aCoeff in theNum)
            {
                myNum[anIdx++] = aCoeff;
            }
            anIdx = 0;
            foreach (double aCoeff in theDenom)
            {
                myDenom[anIdx++] = aCoeff;
            }
        }

        /// <summary>Evaluates rational function at theX; false if the denominator is zero.</summary>
        public bool Value(double theX, out double theY)
        {
            double aNum = 0.0;
            double aDenom = 0.0;

            // Evaluate numerator using Horner's method
            if (myNum.Length() > 0)
            {
                int aLast = myNum.Upper();
                aNum = myNum[aLast];
                for (int i = aLast - 1; i >= myNum.Lower(); --i)
                {
                    aNum = aNum * theX + myNum[i];
                }
            }

            // Evaluate denominator using Horner's method
            if (myDenom.Length() > 0)
            {
                int aLast = myDenom.Upper();
                aDenom = myDenom[aLast];
                for (int i = aLast - 1; i >= myDenom.Lower(); --i)
                {
                    aDenom = aDenom * theX + myDenom[i];
                }
            }

            // Check for division by zero
            if (Math.Abs(aDenom) < 1e-15)
            {
                theY = 0.0;
                return false;
            }

            theY = aNum / aDenom;
            return true;
        }
    }

    /// <summary>Composite functor: f(g(x)).</summary>
    public sealed class Composite<Outer, Inner> : IScalarFunction
        where Outer : IScalarFunction
        where Inner : IScalarFunction
    {
        private Outer myOuter;
        private Inner myInner;

        public Composite(Outer theOuter, Inner theInner)
        {
            myOuter = theOuter;
            myInner = theInner;
        }

        /// <summary>Evaluates f(g(theX)); false if either evaluation fails.</summary>
        public bool Value(double theX, out double theY)
        {
            if (!myInner.Value(theX, out double aInner))
            {
                theY = 0.0;
                return false;
            }
            return myOuter.Value(aInner, out theY);
        }
    }

    public static Composite<Outer, Inner> MakeComposite<Outer, Inner>(Outer theOuter, Inner theInner)
        where Outer : IScalarFunction
        where Inner : IScalarFunction
    {
        return new Composite<Outer, Inner>(theOuter, theInner);
    }

    /// <summary>Sum of functions functor: f(x) + g(x).</summary>
    public sealed class Sum<F, G> : IScalarFunction
        where F : IScalarFunction
        where G : IScalarFunction
    {
        private F myF;
        private G myG;

        public Sum(F theF, G theG)
        {
            myF = theF;
            myG = theG;
        }

        public bool Value(double theX, out double theY)
        {
            theY = 0.0;
            if (!myF.Value(theX, out double aF))
            {
                return false;
            }
            if (!myG.Value(theX, out double aG))
            {
                return false;
            }
            theY = aF + aG;
            return true;
        }
    }

    public static Sum<F, G> MakeSum<F, G>(F theF, G theG)
        where F : IScalarFunction
        where G : IScalarFunction
    {
        return new Sum<F, G>(theF, theG);
    }

    /// <summary>Difference of functions functor: f(x) - g(x).</summary>
    public sealed class Difference<F, G> : IScalarFunction
        where F : IScalarFunction
        where G : IScalarFunction
    {
        private F myF;
        private G myG;

        public Difference(F theF, G theG)
        {
            myF = theF;
            myG = theG;
        }

        public bool Value(double theX, out double theY)
        {
            theY = 0.0;
            if (!myF.Value(theX, out double aF))
            {
                return false;
            }
            if (!myG.Value(theX, out double aG))
            {
                return false;
            }
            theY = aF - aG;
            return true;
        }
    }

    public static Difference<F, G> MakeDifference<F, G>(F theF, G theG)
        where F : IScalarFunction
        where G : IScalarFunction
    {
        return new Difference<F, G>(theF, theG);
    }

    /// <summary>Product of functions functor: f(x) * g(x).</summary>
    public sealed class Product<F, G> : IScalarFunction
        where F : IScalarFunction
        where G : IScalarFunction
    {
        private F myF;
        private G myG;

        public Product(F theF, G theG)
        {
            myF = theF;
            myG = theG;
        }

        public bool Value(double theX, out double theY)
        {
            theY = 0.0;
            if (!myF.Value(theX, out double aF))
            {
                return false;
            }
            if (!myG.Value(theX, out double aG))
            {
                return false;
            }
            theY = aF * aG;
            return true;
        }
    }

    public static Product<F, G> MakeProduct<F, G>(F theF, G theG)
        where F : IScalarFunction
        where G : IScalarFunction
    {
        return new Product<F, G>(theF, theG);
    }

    /// <summary>Quotient of functions functor: f(x) / g(x); fails where |g(x)| &lt; 1e-15.</summary>
    public sealed class Quotient<F, G> : IScalarFunction
        where F : IScalarFunction
        where G : IScalarFunction
    {
        private F myF;
        private G myG;

        public Quotient(F theF, G theG)
        {
            myF = theF;
            myG = theG;
        }

        public bool Value(double theX, out double theY)
        {
            theY = 0.0;
            if (!myF.Value(theX, out double aF))
            {
                return false;
            }
            if (!myG.Value(theX, out double aG))
            {
                return false;
            }
            if (Math.Abs(aG) < 1e-15)
            {
                return false;
            }
            theY = aF / aG;
            return true;
        }
    }

    public static Quotient<F, G> MakeQuotient<F, G>(F theF, G theG)
        where F : IScalarFunction
        where G : IScalarFunction
    {
        return new Quotient<F, G>(theF, theG);
    }

    /// <summary>Scaled function functor: c * f(x).</summary>
    public sealed class Scaled<F> : IScalarFunction
        where F : IScalarFunction
    {
        private F myF;
        private readonly double myScale;

        public Scaled(F theF, double theScale)
        {
            myF = theF;
            myScale = theScale;
        }

        public bool Value(double theX, out double theY)
        {
            theY = 0.0;
            if (!myF.Value(theX, out double aF))
            {
                return false;
            }
            theY = myScale * aF;
            return true;
        }
    }

    public static Scaled<F> MakeScaled<F>(F theF, double theScale)
        where F : IScalarFunction
    {
        return new Scaled<F>(theF, theScale);
    }

    /// <summary>Shifted function functor: f(x) + c.</summary>
    public sealed class Shifted<F> : IScalarFunction
        where F : IScalarFunction
    {
        private F myF;
        private readonly double myShift;

        public Shifted(F theF, double theShift)
        {
            myF = theF;
            myShift = theShift;
        }

        public bool Value(double theX, out double theY)
        {
            theY = 0.0;
            if (!myF.Value(theX, out double aF))
            {
                return false;
            }
            theY = aF + myShift;
            return true;
        }
    }

    public static Shifted<F> MakeShifted<F>(F theF, double theShift)
        where F : IScalarFunction
    {
        return new Shifted<F>(theF, theShift);
    }

    /// <summary>Negated function functor: -f(x).</summary>
    public sealed class Negated<F> : IScalarFunction
        where F : IScalarFunction
    {
        private F myF;

        public Negated(F theF)
        {
            myF = theF;
        }

        public bool Value(double theX, out double theY)
        {
            theY = 0.0;
            if (!myF.Value(theX, out double aF))
            {
                return false;
            }
            theY = -aF;
            return true;
        }
    }

    public static Negated<F> MakeNegated<F>(F theF)
        where F : IScalarFunction
    {
        return new Negated<F>(theF);
    }

    /// <summary>Constant function functor: f(x) = c.</summary>
    public sealed class Constant : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly double myValue;

        public Constant(double theValue)
        {
            myValue = theValue;
        }

        public bool Value(double theX, out double theY)
        {
            theY = myValue;
            return true;
        }

        public bool Values(double theX, out double theY, out double theDY)
        {
            theY = myValue;
            theDY = 0.0;
            return true;
        }
    }

    /// <summary>Linear function functor: f(x) = a*x + b.</summary>
    public sealed class Linear : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly double mySlope;
        private readonly double myIntercept;

        public Linear(double theSlope, double theIntercept)
        {
            mySlope = theSlope;
            myIntercept = theIntercept;
        }

        public bool Value(double theX, out double theY)
        {
            theY = mySlope * theX + myIntercept;
            return true;
        }

        public bool Values(double theX, out double theY, out double theDY)
        {
            theY = mySlope * theX + myIntercept;
            theDY = mySlope;
            return true;
        }
    }

    /// <summary>Sine function functor: f(x) = A * sin(omega * x + phi) + c.</summary>
    public sealed class Sine : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly double myAmplitude;
        private readonly double myFrequency;
        private readonly double myPhase;
        private readonly double myOffset;

        public Sine(double theAmplitude = 1.0, double theFrequency = 1.0, double thePhase = 0.0, double theOffset = 0.0)
        {
            myAmplitude = theAmplitude;
            myFrequency = theFrequency;
            myPhase = thePhase;
            myOffset = theOffset;
        }

        public bool Value(double theX, out double theY)
        {
            theY = myAmplitude * Math.Sin(myFrequency * theX + myPhase) + myOffset;
            return true;
        }

        public bool Values(double theX, out double theY, out double theDY)
        {
            double anArg = myFrequency * theX + myPhase;
            theY = myAmplitude * Math.Sin(anArg) + myOffset;
            theDY = myAmplitude * myFrequency * Math.Cos(anArg);
            return true;
        }
    }

    /// <summary>Cosine function functor: f(x) = A * cos(omega * x + phi) + c.</summary>
    public sealed class Cosine : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly double myAmplitude;
        private readonly double myFrequency;
        private readonly double myPhase;
        private readonly double myOffset;

        public Cosine(double theAmplitude = 1.0, double theFrequency = 1.0, double thePhase = 0.0, double theOffset = 0.0)
        {
            myAmplitude = theAmplitude;
            myFrequency = theFrequency;
            myPhase = thePhase;
            myOffset = theOffset;
        }

        public bool Value(double theX, out double theY)
        {
            theY = myAmplitude * Math.Cos(myFrequency * theX + myPhase) + myOffset;
            return true;
        }

        public bool Values(double theX, out double theY, out double theDY)
        {
            double anArg = myFrequency * theX + myPhase;
            theY = myAmplitude * Math.Cos(anArg) + myOffset;
            theDY = -myAmplitude * myFrequency * Math.Sin(anArg);
            return true;
        }
    }

    /// <summary>Exponential function functor: f(x) = a * exp(b * x) + c.</summary>
    public sealed class Exponential : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly double myScale;
        private readonly double myRate;
        private readonly double myOffset;

        public Exponential(double theScale = 1.0, double theRate = 1.0, double theOffset = 0.0)
        {
            myScale = theScale;
            myRate = theRate;
            myOffset = theOffset;
        }

        public bool Value(double theX, out double theY)
        {
            theY = myScale * Math.Exp(myRate * theX) + myOffset;
            return true;
        }

        public bool Values(double theX, out double theY, out double theDY)
        {
            double anExp = Math.Exp(myRate * theX);
            theY = myScale * anExp + myOffset;
            theDY = myScale * myRate * anExp;
            return true;
        }
    }

    /// <summary>Power function functor: f(x) = a * x^n + c; fails for x &lt; 0 with a non-integer exponent.</summary>
    public sealed class Power : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly double myExponent;
        private readonly double myScale;
        private readonly double myOffset;

        public Power(double theExponent, double theScale = 1.0, double theOffset = 0.0)
        {
            myExponent = theExponent;
            myScale = theScale;
            myOffset = theOffset;
        }

        public bool Value(double theX, out double theY)
        {
            if (theX < 0.0 && myExponent != Math.Floor(myExponent))
            {
                theY = 0.0;
                return false;
            }
            theY = myScale * Math.Pow(theX, myExponent) + myOffset;
            return true;
        }

        public bool Values(double theX, out double theY, out double theDY)
        {
            if (theX < 0.0 && myExponent != Math.Floor(myExponent))
            {
                theY = 0.0;
                theDY = 0.0;
                return false;
            }
            double aPow = Math.Pow(theX, myExponent);
            theY = myScale * aPow + myOffset;
            if (Math.Abs(theX) < 1e-15)
            {
                theDY = (myExponent == 1.0) ? myScale : 0.0;
            }
            else
            {
                theDY = myScale * myExponent * aPow / theX;
            }
            return true;
        }
    }

    /// <summary>Gaussian function functor: f(x) = a * exp(-((x - mu)^2) / (2 * sigma^2)).</summary>
    public sealed class Gaussian : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly double myAmplitude;
        private readonly double myMean;
        private readonly double mySigma;

        public Gaussian(double theAmplitude = 1.0, double theMean = 0.0, double theSigma = 1.0)
        {
            myAmplitude = theAmplitude;
            myMean = theMean;
            mySigma = theSigma;
        }

        public bool Value(double theX, out double theY)
        {
            if (Math.Abs(mySigma) < 1e-15)
            {
                theY = 0.0;
                return false;
            }
            double aZ = (theX - myMean) / mySigma;
            theY = myAmplitude * Math.Exp(-0.5 * aZ * aZ);
            return true;
        }

        public bool Values(double theX, out double theY, out double theDY)
        {
            if (Math.Abs(mySigma) < 1e-15)
            {
                theY = 0.0;
                theDY = 0.0;
                return false;
            }
            double aZ = (theX - myMean) / mySigma;
            double aExp = Math.Exp(-0.5 * aZ * aZ);
            theY = myAmplitude * aExp;
            theDY = -myAmplitude * aZ * aExp / mySigma;
            return true;
        }
    }
}
