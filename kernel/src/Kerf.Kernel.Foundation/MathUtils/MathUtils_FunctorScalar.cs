// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_FunctorScalar.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Utilities;

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
        bool Value(double x, out double y);
    }

    /// <summary>A scalar function with its derivative: C++'s "type with <c>Values(double, double&amp;, double&amp;)</c> method".</summary>
    public interface IScalarFunctionWithDerivative
    {
        /// <summary>Evaluates the function and its derivative at theX; false if they could not be evaluated.</summary>
        bool Values(double x, out double y, out double dy);
    }

    /// <summary>Signature of the callable wrapped by <see cref="ScalarLambda"/>: bool(double theX, double&amp; theY).</summary>
    public delegate bool ScalarLambdaFunction(double x, out double y);

    /// <summary>Signature of the callable wrapped by <see cref="ScalarLambdaWithDerivative"/>.</summary>
    public delegate bool ScalarLambdaWithDerivativeFunction(double x, out double y, out double dy);

    /// <summary>Lambda wrapper for scalar functions with value only.</summary>
    /// <remarks>C++ is templated on the lambda type; here it holds a delegate.</remarks>
    public sealed class ScalarLambda : IScalarFunction
    {
        private readonly ScalarLambdaFunction _lambda;

        public ScalarLambda(ScalarLambdaFunction lambda)
        {
            _lambda = lambda;
        }

        public bool Value(double x, out double y) => _lambda(x, out y);
    }

    /// <summary>Lambda wrapper for scalar functions with value and derivative.</summary>
    public sealed class ScalarLambdaWithDerivative : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly ScalarLambdaWithDerivativeFunction _lambda;

        public ScalarLambdaWithDerivative(ScalarLambdaWithDerivativeFunction lambda)
        {
            _lambda = lambda;
        }

        public bool Values(double x, out double y, out double dy) => _lambda(x, out y, out dy);

        /// <summary>Evaluates only the function value (for algorithms that don't need derivative).</summary>
        public bool Value(double x, out double y)
        {
            return _lambda(x, out y, out _);
        }
    }

    /// <summary>Helper function to create ScalarLambda.</summary>
    public static ScalarLambda MakeScalar(ScalarLambdaFunction lambda)
    {
        return new ScalarLambda(lambda);
    }

    /// <summary>Helper function to create ScalarLambdaWithDerivative.</summary>
    public static ScalarLambdaWithDerivative MakeScalarWithDerivative(ScalarLambdaWithDerivativeFunction lambda)
    {
        return new ScalarLambdaWithDerivative(lambda);
    }

    /// <summary>Polynomial functor: f(x) = sum(a[i] * x^i), coefficients in ascending power order.</summary>
    public sealed class Polynomial : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly math_Vector _coeffs;

        /// <summary>Constructor from a coefficient list (C++'s initializer list), ascending power order.</summary>
        public Polynomial(params ReadOnlySpan<double> coeffs)
        {
            _coeffs = new math_Vector(0, coeffs.Length - 1);
            int idx = 0;
            foreach (double coeff in coeffs)
            {
                _coeffs[idx++] = coeff;
            }
        }

        /// <summary>Constructor from math_Vector (copied), ascending power order.</summary>
        public Polynomial(math_Vector coeffs)
        {
            _coeffs = new math_Vector(coeffs);
        }

        /// <summary>Evaluates polynomial at theX using Horner's method.</summary>
        public bool Value(double x, out double y)
        {
            if (_coeffs.Length() <= 0)
            {
                y = 0.0;
                return true;
            }

            // Horner's method: p(x) = a[0] + x*(a[1] + x*(a[2] + ...))
            int last = _coeffs.Upper();
            y = _coeffs[last];
            for (int i = last - 1; i >= _coeffs.Lower(); --i)
            {
                y = y * x + _coeffs[i];
            }
            return true;
        }

        /// <summary>Evaluates polynomial and its derivative at theX.</summary>
        public bool Values(double x, out double y, out double dy)
        {
            if (_coeffs.Length() <= 0)
            {
                y = 0.0;
                dy = 0.0;
                return true;
            }

            int n = _coeffs.Length();
            if (n == 1)
            {
                y = _coeffs[_coeffs.Lower()];
                dy = 0.0;
                return true;
            }

            // Horner's method for value and derivative simultaneously
            int lower = _coeffs.Lower();
            int last = _coeffs.Upper();
            y = _coeffs[last];
            dy = 0.0;
            for (int i = last - 1; i >= lower; --i)
            {
                dy = dy * x + y;
                y = y * x + _coeffs[i];
            }
            return true;
        }

        /// <summary>Returns the degree of the polynomial (number of coefficients - 1).</summary>
        public int Degree() => _coeffs.Length() <= 0 ? 0 : _coeffs.Length() - 1;

        /// <summary>Returns coefficient by index (0 = constant term), 0 out of range.</summary>
        public double Coefficient(int index)
        {
            int idx = _coeffs.Lower() + index;
            return (idx >= _coeffs.Lower() && idx <= _coeffs.Upper()) ? _coeffs[idx] : 0.0;
        }
    }

    /// <summary>Rational function functor: f(x) = P(x) / Q(x).</summary>
    public sealed class Rational : IScalarFunction
    {
        private readonly math_Vector _num;
        private readonly math_Vector _denom;

        /// <summary>Constructor from math_Vector (copied), ascending power order.</summary>
        public Rational(math_Vector num, math_Vector denom)
        {
            _num = new math_Vector(num);
            _denom = new math_Vector(denom);
        }

        /// <summary>Constructor from coefficient lists (C++'s initializer lists).</summary>
        public Rational(ReadOnlySpan<double> num, ReadOnlySpan<double> denom)
        {
            _num = new math_Vector(0, num.Length - 1);
            _denom = new math_Vector(0, denom.Length - 1);
            int idx = 0;
            foreach (double coeff in num)
            {
                _num[idx++] = coeff;
            }
            idx = 0;
            foreach (double coeff in denom)
            {
                _denom[idx++] = coeff;
            }
        }

        /// <summary>Evaluates rational function at theX; false if the denominator is zero.</summary>
        public bool Value(double x, out double y)
        {
            double num = 0.0;
            double denom = 0.0;

            // Evaluate numerator using Horner's method
            if (_num.Length() > 0)
            {
                int last = _num.Upper();
                num = _num[last];
                for (int i = last - 1; i >= _num.Lower(); --i)
                {
                    num = num * x + _num[i];
                }
            }

            // Evaluate denominator using Horner's method
            if (_denom.Length() > 0)
            {
                int last = _denom.Upper();
                denom = _denom[last];
                for (int i = last - 1; i >= _denom.Lower(); --i)
                {
                    denom = denom * x + _denom[i];
                }
            }

            // Check for division by zero
            if (Math.Abs(denom) < 1e-15)
            {
                y = 0.0;
                return false;
            }

            y = num / denom;
            return true;
        }
    }

    /// <summary>Composite functor: f(g(x)).</summary>
    public sealed class Composite<Outer, Inner> : IScalarFunction
        where Outer : IScalarFunction
        where Inner : IScalarFunction
    {
        private Outer _outer;
        private Inner _inner;

        public Composite(Outer outer, Inner inner)
        {
            _outer = outer;
            _inner = inner;
        }

        /// <summary>Evaluates f(g(theX)); false if either evaluation fails.</summary>
        public bool Value(double x, out double y)
        {
            if (!_inner.Value(x, out double inner))
            {
                y = 0.0;
                return false;
            }
            return _outer.Value(inner, out y);
        }
    }

    public static Composite<Outer, Inner> MakeComposite<Outer, Inner>(Outer outer, Inner inner)
        where Outer : IScalarFunction
        where Inner : IScalarFunction
    {
        return new Composite<Outer, Inner>(outer, inner);
    }

    /// <summary>Sum of functions functor: f(x) + g(x).</summary>
    public sealed class Sum<F, G> : IScalarFunction
        where F : IScalarFunction
        where G : IScalarFunction
    {
        private F _f;
        private G _g;

        public Sum(F f, G g)
        {
            _f = f;
            _g = g;
        }

        public bool Value(double x, out double y)
        {
            y = 0.0;
            if (!_f.Value(x, out double f))
            {
                return false;
            }
            if (!_g.Value(x, out double g))
            {
                return false;
            }
            y = f + g;
            return true;
        }
    }

    public static Sum<F, G> MakeSum<F, G>(F f, G g)
        where F : IScalarFunction
        where G : IScalarFunction
    {
        return new Sum<F, G>(f, g);
    }

    /// <summary>Difference of functions functor: f(x) - g(x).</summary>
    public sealed class Difference<F, G> : IScalarFunction
        where F : IScalarFunction
        where G : IScalarFunction
    {
        private F _f;
        private G _g;

        public Difference(F f, G g)
        {
            _f = f;
            _g = g;
        }

        public bool Value(double x, out double y)
        {
            y = 0.0;
            if (!_f.Value(x, out double f))
            {
                return false;
            }
            if (!_g.Value(x, out double g))
            {
                return false;
            }
            y = f - g;
            return true;
        }
    }

    public static Difference<F, G> MakeDifference<F, G>(F f, G g)
        where F : IScalarFunction
        where G : IScalarFunction
    {
        return new Difference<F, G>(f, g);
    }

    /// <summary>Product of functions functor: f(x) * g(x).</summary>
    public sealed class Product<F, G> : IScalarFunction
        where F : IScalarFunction
        where G : IScalarFunction
    {
        private F _f;
        private G _g;

        public Product(F f, G g)
        {
            _f = f;
            _g = g;
        }

        public bool Value(double x, out double y)
        {
            y = 0.0;
            if (!_f.Value(x, out double f))
            {
                return false;
            }
            if (!_g.Value(x, out double g))
            {
                return false;
            }
            y = f * g;
            return true;
        }
    }

    public static Product<F, G> MakeProduct<F, G>(F f, G g)
        where F : IScalarFunction
        where G : IScalarFunction
    {
        return new Product<F, G>(f, g);
    }

    /// <summary>Quotient of functions functor: f(x) / g(x); fails where |g(x)| &lt; 1e-15.</summary>
    public sealed class Quotient<F, G> : IScalarFunction
        where F : IScalarFunction
        where G : IScalarFunction
    {
        private F _f;
        private G _g;

        public Quotient(F f, G g)
        {
            _f = f;
            _g = g;
        }

        public bool Value(double x, out double y)
        {
            y = 0.0;
            if (!_f.Value(x, out double f))
            {
                return false;
            }
            if (!_g.Value(x, out double g))
            {
                return false;
            }
            if (Math.Abs(g) < 1e-15)
            {
                return false;
            }
            y = f / g;
            return true;
        }
    }

    public static Quotient<F, G> MakeQuotient<F, G>(F f, G g)
        where F : IScalarFunction
        where G : IScalarFunction
    {
        return new Quotient<F, G>(f, g);
    }

    /// <summary>Scaled function functor: c * f(x).</summary>
    public sealed class Scaled<F> : IScalarFunction
        where F : IScalarFunction
    {
        private F _f;
        private readonly double _scale;

        public Scaled(F f, double scale)
        {
            _f = f;
            _scale = scale;
        }

        public bool Value(double x, out double y)
        {
            y = 0.0;
            if (!_f.Value(x, out double f))
            {
                return false;
            }
            y = _scale * f;
            return true;
        }
    }

    public static Scaled<F> MakeScaled<F>(F f, double scale)
        where F : IScalarFunction
    {
        return new Scaled<F>(f, scale);
    }

    /// <summary>Shifted function functor: f(x) + c.</summary>
    public sealed class Shifted<F> : IScalarFunction
        where F : IScalarFunction
    {
        private F _f;
        private readonly double _shift;

        public Shifted(F f, double shift)
        {
            _f = f;
            _shift = shift;
        }

        public bool Value(double x, out double y)
        {
            y = 0.0;
            if (!_f.Value(x, out double f))
            {
                return false;
            }
            y = f + _shift;
            return true;
        }
    }

    public static Shifted<F> MakeShifted<F>(F f, double shift)
        where F : IScalarFunction
    {
        return new Shifted<F>(f, shift);
    }

    /// <summary>Negated function functor: -f(x).</summary>
    public sealed class Negated<F> : IScalarFunction
        where F : IScalarFunction
    {
        private F _f;

        public Negated(F f)
        {
            _f = f;
        }

        public bool Value(double x, out double y)
        {
            y = 0.0;
            if (!_f.Value(x, out double f))
            {
                return false;
            }
            y = -f;
            return true;
        }
    }

    public static Negated<F> MakeNegated<F>(F f)
        where F : IScalarFunction
    {
        return new Negated<F>(f);
    }

    /// <summary>Constant function functor: f(x) = c.</summary>
    public sealed class Constant : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly double _value;

        public Constant(double value)
        {
            _value = value;
        }

        public bool Value(double x, out double y)
        {
            y = _value;
            return true;
        }

        public bool Values(double x, out double y, out double dy)
        {
            y = _value;
            dy = 0.0;
            return true;
        }
    }

    /// <summary>Linear function functor: f(x) = a*x + b.</summary>
    public sealed class Linear : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly double _slope;
        private readonly double _intercept;

        public Linear(double slope, double intercept)
        {
            _slope = slope;
            _intercept = intercept;
        }

        public bool Value(double x, out double y)
        {
            y = _slope * x + _intercept;
            return true;
        }

        public bool Values(double x, out double y, out double dy)
        {
            y = _slope * x + _intercept;
            dy = _slope;
            return true;
        }
    }

    /// <summary>Sine function functor: f(x) = A * sin(omega * x + phi) + c.</summary>
    public sealed class Sine : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly double _amplitude;
        private readonly double _frequency;
        private readonly double _phase;
        private readonly double _offset;

        public Sine(double amplitude = 1.0, double frequency = 1.0, double phase = 0.0, double offset = 0.0)
        {
            _amplitude = amplitude;
            _frequency = frequency;
            _phase = phase;
            _offset = offset;
        }

        public bool Value(double x, out double y)
        {
            y = _amplitude * Math.Sin(_frequency * x + _phase) + _offset;
            return true;
        }

        public bool Values(double x, out double y, out double dy)
        {
            double arg = _frequency * x + _phase;
            y = _amplitude * Math.Sin(arg) + _offset;
            dy = _amplitude * _frequency * Math.Cos(arg);
            return true;
        }
    }

    /// <summary>Cosine function functor: f(x) = A * cos(omega * x + phi) + c.</summary>
    public sealed class Cosine : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly double _amplitude;
        private readonly double _frequency;
        private readonly double _phase;
        private readonly double _offset;

        public Cosine(double amplitude = 1.0, double frequency = 1.0, double phase = 0.0, double offset = 0.0)
        {
            _amplitude = amplitude;
            _frequency = frequency;
            _phase = phase;
            _offset = offset;
        }

        public bool Value(double x, out double y)
        {
            y = _amplitude * Math.Cos(_frequency * x + _phase) + _offset;
            return true;
        }

        public bool Values(double x, out double y, out double dy)
        {
            double arg = _frequency * x + _phase;
            y = _amplitude * Math.Cos(arg) + _offset;
            dy = -_amplitude * _frequency * Math.Sin(arg);
            return true;
        }
    }

    /// <summary>Exponential function functor: f(x) = a * exp(b * x) + c.</summary>
    public sealed class Exponential : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly double _scale;
        private readonly double _rate;
        private readonly double _offset;

        public Exponential(double scale = 1.0, double rate = 1.0, double offset = 0.0)
        {
            _scale = scale;
            _rate = rate;
            _offset = offset;
        }

        public bool Value(double x, out double y)
        {
            y = _scale * Math.Exp(_rate * x) + _offset;
            return true;
        }

        public bool Values(double x, out double y, out double dy)
        {
            double exp = Math.Exp(_rate * x);
            y = _scale * exp + _offset;
            dy = _scale * _rate * exp;
            return true;
        }
    }

    /// <summary>Power function functor: f(x) = a * x^n + c; fails for x &lt; 0 with a non-integer exponent.</summary>
    public sealed class Power : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly double _exponent;
        private readonly double _scale;
        private readonly double _offset;

        public Power(double exponent, double scale = 1.0, double offset = 0.0)
        {
            _exponent = exponent;
            _scale = scale;
            _offset = offset;
        }

        public bool Value(double x, out double y)
        {
            if (x < 0.0 && _exponent != Math.Floor(_exponent))
            {
                y = 0.0;
                return false;
            }
            y = _scale * Math.Pow(x, _exponent) + _offset;
            return true;
        }

        public bool Values(double x, out double y, out double dy)
        {
            if (x < 0.0 && _exponent != Math.Floor(_exponent))
            {
                y = 0.0;
                dy = 0.0;
                return false;
            }
            double pow = Math.Pow(x, _exponent);
            y = _scale * pow + _offset;
            if (Math.Abs(x) < 1e-15)
            {
                dy = (_exponent == 1.0) ? _scale : 0.0;
            }
            else
            {
                dy = _scale * _exponent * pow / x;
            }
            return true;
        }
    }

    /// <summary>Gaussian function functor: f(x) = a * exp(-((x - mu)^2) / (2 * sigma^2)).</summary>
    public sealed class Gaussian : IScalarFunction, IScalarFunctionWithDerivative
    {
        private readonly double _amplitude;
        private readonly double _mean;
        private readonly double _sigma;

        public Gaussian(double amplitude = 1.0, double mean = 0.0, double sigma = 1.0)
        {
            _amplitude = amplitude;
            _mean = mean;
            _sigma = sigma;
        }

        public bool Value(double x, out double y)
        {
            if (Math.Abs(_sigma) < 1e-15)
            {
                y = 0.0;
                return false;
            }
            double z = (x - _mean) / _sigma;
            y = _amplitude * Math.Exp(-0.5 * z * z);
            return true;
        }

        public bool Values(double x, out double y, out double dy)
        {
            if (Math.Abs(_sigma) < 1e-15)
            {
                y = 0.0;
                dy = 0.0;
                return false;
            }
            double z = (x - _mean) / _sigma;
            double exp = Math.Exp(-0.5 * z * z);
            y = _amplitude * exp;
            dy = -_amplitude * z * exp / _sigma;
            return true;
        }
    }
}
