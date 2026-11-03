// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_FunctorVector.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Utilities;

// The member sets OCCT's N-dimensional solvers expect of their template arguments, as interfaces (see
// MathUtils_FunctorScalar.cs). Vectors and matrices are passed as references and filled in place.

/// <summary>Non-virtual functor classes for vector (N-D) functions.</summary>
public static partial class MathUtils
{
    /// <summary>A function of N variables: C++'s "type with <c>Value(const math_Vector&amp;, double&amp;)</c> method".</summary>
    public interface IVectorFunction
    {
        /// <summary>Evaluates the function at theX; false if it could not be evaluated.</summary>
        bool Value(math_Vector x, out double y);
    }

    /// <summary>A function of N variables with its gradient (<c>Value</c> and <c>Gradient(const math_Vector&amp;, math_Vector&amp;)</c>).</summary>
    public interface IVectorFunctionWithGradient : IVectorFunction
    {
        /// <summary>Computes the gradient at theX into theG; false if it could not be computed.</summary>
        bool Gradient(math_Vector x, math_Vector g);
    }

    /// <summary>A function of N variables with its gradient and Hessian (<c>Hessian(const math_Vector&amp;, math_Matrix&amp;)</c>).</summary>
    public interface IVectorFunctionWithHessian : IVectorFunctionWithGradient
    {
        /// <summary>Computes the Hessian at theX into theH; false if it could not be computed.</summary>
        bool Hessian(math_Vector x, Matrix h);
    }

    /// <summary>A vector-valued function F: R^n -> R^m: C++'s "type with <c>Value(const math_Vector&amp; x, math_Vector&amp; F)</c> method".</summary>
    public interface ISystemFunction
    {
        /// <summary>Computes F(theX) into theF; false if it could not be computed.</summary>
        bool Value(math_Vector x, math_Vector f);
    }

    /// <summary>Signature of the callable wrapped by <see cref="VectorLambda"/> (and the value part of <see cref="VectorLambdaWithGradient"/>).</summary>
    public delegate bool VectorLambdaFunction(math_Vector x, out double y);

    /// <summary>Signature of the gradient callable wrapped by <see cref="VectorLambdaWithGradient"/>.</summary>
    public delegate bool VectorGradientLambdaFunction(math_Vector x, math_Vector g);

    /// <summary>Signature of the callable wrapped by <see cref="SystemLambda"/>.</summary>
    public delegate bool SystemLambdaFunction(math_Vector x, math_Vector f);

    /// <summary>Lambda wrapper for N-D scalar functions with value only.</summary>
    public sealed class VectorLambda : IVectorFunction
    {
        private readonly VectorLambdaFunction _lambda;

        public VectorLambda(VectorLambdaFunction lambda)
        {
            _lambda = lambda;
        }

        public bool Value(math_Vector x, out double y) => _lambda(x, out y);
    }

    /// <summary>Lambda wrapper for N-D scalar functions with value and gradient.</summary>
    public sealed class VectorLambdaWithGradient : IVectorFunctionWithGradient
    {
        private readonly VectorLambdaFunction _valueLambda;
        private readonly VectorGradientLambdaFunction _gradLambda;

        public VectorLambdaWithGradient(VectorLambdaFunction valueLambda, VectorGradientLambdaFunction gradLambda)
        {
            _valueLambda = valueLambda;
            _gradLambda = gradLambda;
        }

        public bool Value(math_Vector x, out double y) => _valueLambda(x, out y);

        public bool Gradient(math_Vector x, math_Vector g)
        {
            return _gradLambda(x, g);
        }

        /// <summary>Evaluates value and gradient.</summary>
        public bool Values(math_Vector x, out double y, math_Vector g)
        {
            return _valueLambda(x, out y) && _gradLambda(x, g);
        }
    }

    /// <summary>Helper function to create VectorLambda.</summary>
    public static VectorLambda MakeVector(VectorLambdaFunction lambda)
    {
        return new VectorLambda(lambda);
    }

    /// <summary>Helper function to create VectorLambdaWithGradient.</summary>
    public static VectorLambdaWithGradient MakeVectorWithGradient(VectorLambdaFunction valueLambda,
                                                                  VectorGradientLambdaFunction gradLambda)
    {
        return new VectorLambdaWithGradient(valueLambda, gradLambda);
    }

    /// <summary>Quadratic form functor: f(x) = x^T A x + b^T x + c.</summary>
    public sealed class QuadraticForm : IVectorFunctionWithGradient
    {
        private readonly Matrix _a;
        private readonly math_Vector _b;
        private readonly double _c;

        /// <summary>The matrix and vector are copied, as in C++.</summary>
        public QuadraticForm(Matrix a, math_Vector b, double c)
        {
            _a = new Matrix(a);
            _b = new math_Vector(b);
            _c = c;
        }

        public bool Value(math_Vector x, out double y)
        {
            y = _c;
            for (int i = x.Lower(); i <= x.Upper(); ++i)
            {
                for (int j = x.Lower(); j <= x.Upper(); ++j)
                {
                    y += x[i] * _a[i, j] * x[j];
                }
            }
            for (int i = x.Lower(); i <= x.Upper(); ++i)
            {
                y += _b[i] * x[i];
            }
            return true;
        }

        /// <summary>Gradient: (A + A^T) x + b.</summary>
        public bool Gradient(math_Vector x, math_Vector g)
        {
            for (int i = x.Lower(); i <= x.Upper(); ++i)
            {
                g[i] = _b[i];
                for (int j = x.Lower(); j <= x.Upper(); ++j)
                {
                    g[i] += (_a[i, j] + _a[j, i]) * x[j];
                }
            }
            return true;
        }

        public bool Values(math_Vector x, out double y, math_Vector g)
        {
            return Value(x, out y) && Gradient(x, g);
        }
    }

    /// <summary>Rosenbrock function: f(x, y) = (a - x)^2 + b(y - x^2)^2, minimum at (a, a^2).</summary>
    public sealed class Rosenbrock : IVectorFunctionWithGradient
    {
        private readonly double _a;
        private readonly double _b;

        public Rosenbrock(double a = 1.0, double b = 100.0)
        {
            _a = a;
            _b = b;
        }

        public bool Value(math_Vector theX, out double theY)
        {
            double x = theX[theX.Lower()];
            double y = theX[theX.Lower() + 1];
            double t1 = _a - x;
            double t2 = y - x * x;
            theY = t1 * t1 + _b * t2 * t2;
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector g)
        {
            double x = theX[theX.Lower()];
            double y = theX[theX.Lower() + 1];
            double t2 = y - x * x;
            g[g.Lower()] = -2.0 * (_a - x) - 4.0 * _b * x * t2;
            g[g.Lower() + 1] = 2.0 * _b * t2;
            return true;
        }

        public bool Values(math_Vector x, out double y, math_Vector g)
        {
            return Value(x, out y) && Gradient(x, g);
        }
    }

    /// <summary>Sphere function: f(x) = sum(x_i^2), minimum at the origin.</summary>
    public sealed class Sphere : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector x, out double y)
        {
            y = 0.0;
            for (int i = x.Lower(); i <= x.Upper(); ++i)
            {
                y += x[i] * x[i];
            }
            return true;
        }

        public bool Gradient(math_Vector x, math_Vector g)
        {
            for (int i = x.Lower(); i <= x.Upper(); ++i)
            {
                g[i] = 2.0 * x[i];
            }
            return true;
        }

        public bool Values(math_Vector x, out double y, math_Vector g)
        {
            return Value(x, out y) && Gradient(x, g);
        }
    }

    /// <summary>Booth function: f(x, y) = (x + 2y - 7)^2 + (2x + y - 5)^2, minimum at (1, 3).</summary>
    public sealed class Booth : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theY)
        {
            double x = theX[theX.Lower()];
            double y = theX[theX.Lower() + 1];
            double t1 = x + 2.0 * y - 7.0;
            double t2 = 2.0 * x + y - 5.0;
            theY = t1 * t1 + t2 * t2;
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector g)
        {
            double x = theX[theX.Lower()];
            double y = theX[theX.Lower() + 1];
            double t1 = x + 2.0 * y - 7.0;
            double t2 = 2.0 * x + y - 5.0;
            g[g.Lower()] = 2.0 * t1 + 4.0 * t2;
            g[g.Lower() + 1] = 4.0 * t1 + 2.0 * t2;
            return true;
        }

        public bool Values(math_Vector x, out double y, math_Vector g)
        {
            return Value(x, out y) && Gradient(x, g);
        }
    }

    /// <summary>Beale function, minimum at (3, 0.5).</summary>
    public sealed class Beale : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theY)
        {
            double x = theX[theX.Lower()];
            double y = theX[theX.Lower() + 1];
            double t1 = 1.5 - x + x * y;
            double t2 = 2.25 - x + x * y * y;
            double t3 = 2.625 - x + x * y * y * y;
            theY = t1 * t1 + t2 * t2 + t3 * t3;
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector g)
        {
            double x = theX[theX.Lower()];
            double y = theX[theX.Lower() + 1];
            double y2 = y * y;
            double y3 = y2 * y;
            double t1 = 1.5 - x + x * y;
            double t2 = 2.25 - x + x * y2;
            double t3 = 2.625 - x + x * y3;
            g[g.Lower()] = 2.0 * ((y - 1.0) * t1 + (y2 - 1.0) * t2 + (y3 - 1.0) * t3);
            g[g.Lower() + 1] = 2.0 * x * (t1 + 2.0 * y * t2 + 3.0 * y2 * t3);
            return true;
        }

        public bool Values(math_Vector x, out double y, math_Vector g)
        {
            return Value(x, out y) && Gradient(x, g);
        }
    }

    /// <summary>Himmelblau function: f(x, y) = (x^2 + y - 11)^2 + (x + y^2 - 7)^2, four minima.</summary>
    public sealed class Himmelblau : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theY)
        {
            double x = theX[theX.Lower()];
            double y = theX[theX.Lower() + 1];
            double t1 = x * x + y - 11.0;
            double t2 = x + y * y - 7.0;
            theY = t1 * t1 + t2 * t2;
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector g)
        {
            double x = theX[theX.Lower()];
            double y = theX[theX.Lower() + 1];
            double t1 = x * x + y - 11.0;
            double t2 = x + y * y - 7.0;
            g[g.Lower()] = 4.0 * x * t1 + 2.0 * t2;
            g[g.Lower() + 1] = 2.0 * t1 + 4.0 * y * t2;
            return true;
        }

        public bool Values(math_Vector x, out double y, math_Vector g)
        {
            return Value(x, out y) && Gradient(x, g);
        }
    }

    /// <summary>Rastrigin function: f(x) = A*n + sum(x_i^2 - A*cos(2*pi*x_i)), minimum at the origin.</summary>
    public sealed class Rastrigin : IVectorFunctionWithGradient
    {
        private readonly double _a;

        public Rastrigin(double a = 10.0)
        {
            _a = a;
        }

        public bool Value(math_Vector x, out double y)
        {
            const double twoPi = 2.0 * 3.14159265358979323846;
            int n = x.Upper() - x.Lower() + 1;
            y = _a * (double)n;
            for (int i = x.Lower(); i <= x.Upper(); ++i)
            {
                y += x[i] * x[i] - _a * Math.Cos(twoPi * x[i]);
            }
            return true;
        }

        public bool Gradient(math_Vector x, math_Vector g)
        {
            const double twoPi = 2.0 * 3.14159265358979323846;
            for (int i = x.Lower(); i <= x.Upper(); ++i)
            {
                g[i] = 2.0 * x[i] + _a * twoPi * Math.Sin(twoPi * x[i]);
            }
            return true;
        }

        public bool Values(math_Vector x, out double y, math_Vector g)
        {
            return Value(x, out y) && Gradient(x, g);
        }
    }

    /// <summary>Ackley function, minimum at the origin. Value only.</summary>
    public sealed class Ackley : IVectorFunction
    {
        private readonly double _a;
        private readonly double _b;
        private readonly double _c;

        public Ackley(double a = 20.0, double b = 0.2, double c = 2.0 * 3.14159265358979323846)
        {
            _a = a;
            _b = b;
            _c = c;
        }

        public bool Value(math_Vector x, out double y)
        {
            const double e = 2.718281828459045;
            int n = x.Upper() - x.Lower() + 1;
            double sumSq = 0.0;
            double sumCos = 0.0;
            for (int i = x.Lower(); i <= x.Upper(); ++i)
            {
                sumSq += x[i] * x[i];
                sumCos += Math.Cos(_c * x[i]);
            }
            y = -_a * Math.Exp(-_b * Math.Sqrt(sumSq / n)) - Math.Exp(sumCos / n) + _a + e;
            return true;
        }
    }

    /// <summary>Linear least squares residual: f(x) = ||Ax - b||^2.</summary>
    public sealed class LinearResidual : IVectorFunctionWithGradient
    {
        private readonly Matrix _a;
        private readonly math_Vector _b;

        /// <summary>The matrix and vector are copied, as in C++.</summary>
        public LinearResidual(Matrix a, math_Vector b)
        {
            _a = new Matrix(a);
            _b = new math_Vector(b);
        }

        public bool Value(math_Vector x, out double y)
        {
            y = 0.0;
            for (int i = _a.LowerRow(); i <= _a.UpperRow(); ++i)
            {
                double residual = -_b[i];
                for (int j = _a.LowerCol(); j <= _a.UpperCol(); ++j)
                {
                    residual += _a[i, j] * x[j];
                }
                y += residual * residual;
            }
            return true;
        }

        /// <summary>Gradient: 2 A^T (Ax - b).</summary>
        public bool Gradient(math_Vector x, math_Vector g)
        {
            int m = _a.UpperRow() - _a.LowerRow() + 1;
            math_Vector residual = new(1, m);
            for (int i = _a.LowerRow(); i <= _a.UpperRow(); ++i)
            {
                residual[i - _a.LowerRow() + 1] = -_b[i];
                for (int j = _a.LowerCol(); j <= _a.UpperCol(); ++j)
                {
                    residual[i - _a.LowerRow() + 1] += _a[i, j] * x[j];
                }
            }
            for (int j = _a.LowerCol(); j <= _a.UpperCol(); ++j)
            {
                g[j] = 0.0;
                for (int i = _a.LowerRow(); i <= _a.UpperRow(); ++i)
                {
                    g[j] += 2.0 * _a[i, j] * residual[i - _a.LowerRow() + 1];
                }
            }
            return true;
        }

        public bool Values(math_Vector x, out double y, math_Vector g)
        {
            return Value(x, out y) && Gradient(x, g);
        }
    }

    /// <summary>Lambda wrapper for a system of equations F(x) = 0.</summary>
    public sealed class SystemLambda : ISystemFunction
    {
        private readonly SystemLambdaFunction _lambda;
        private readonly int _nbEquations;

        public SystemLambda(SystemLambdaFunction lambda, int nbEquations)
        {
            _lambda = lambda;
            _nbEquations = nbEquations;
        }

        public int EquationCount => _nbEquations;

        public bool Value(math_Vector x, math_Vector f) => _lambda(x, f);
    }

    /// <summary>Helper function to create SystemLambda.</summary>
    public static SystemLambda MakeSystem(SystemLambdaFunction lambda, int nbEquations)
    {
        return new SystemLambda(lambda, nbEquations);
    }
}
