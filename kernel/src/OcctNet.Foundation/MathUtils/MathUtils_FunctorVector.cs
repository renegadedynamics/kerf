// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_FunctorVector.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// The member sets OCCT's N-dimensional solvers expect of their template arguments, as interfaces (see
// MathUtils_FunctorScalar.cs). Vectors and matrices are passed as references and filled in place.

/// <summary>Non-virtual functor classes for vector (N-D) functions.</summary>
public static partial class MathUtils
{
    /// <summary>A function of N variables: C++'s "type with <c>Value(const math_Vector&amp;, double&amp;)</c> method".</summary>
    public interface IVectorFunction
    {
        /// <summary>Evaluates the function at theX; false if it could not be evaluated.</summary>
        bool Value(math_Vector theX, out double theY);
    }

    /// <summary>A function of N variables with its gradient (<c>Value</c> and <c>Gradient(const math_Vector&amp;, math_Vector&amp;)</c>).</summary>
    public interface IVectorFunctionWithGradient : IVectorFunction
    {
        /// <summary>Computes the gradient at theX into theG; false if it could not be computed.</summary>
        bool Gradient(math_Vector theX, math_Vector theG);
    }

    /// <summary>A function of N variables with its gradient and Hessian (<c>Hessian(const math_Vector&amp;, math_Matrix&amp;)</c>).</summary>
    public interface IVectorFunctionWithHessian : IVectorFunctionWithGradient
    {
        /// <summary>Computes the Hessian at theX into theH; false if it could not be computed.</summary>
        bool Hessian(math_Vector theX, math_Matrix theH);
    }

    /// <summary>A vector-valued function F: R^n -> R^m: C++'s "type with <c>Value(const math_Vector&amp; x, math_Vector&amp; F)</c> method".</summary>
    public interface ISystemFunction
    {
        /// <summary>Computes F(theX) into theF; false if it could not be computed.</summary>
        bool Value(math_Vector theX, math_Vector theF);
    }

    /// <summary>Signature of the callable wrapped by <see cref="VectorLambda"/> (and the value part of <see cref="VectorLambdaWithGradient"/>).</summary>
    public delegate bool VectorLambdaFunction(math_Vector theX, out double theY);

    /// <summary>Signature of the gradient callable wrapped by <see cref="VectorLambdaWithGradient"/>.</summary>
    public delegate bool VectorGradientLambdaFunction(math_Vector theX, math_Vector theG);

    /// <summary>Signature of the callable wrapped by <see cref="SystemLambda"/>.</summary>
    public delegate bool SystemLambdaFunction(math_Vector theX, math_Vector theF);

    /// <summary>Lambda wrapper for N-D scalar functions with value only.</summary>
    public sealed class VectorLambda : IVectorFunction
    {
        private readonly VectorLambdaFunction myLambda;

        public VectorLambda(VectorLambdaFunction theLambda)
        {
            myLambda = theLambda;
        }

        public bool Value(math_Vector theX, out double theY) => myLambda(theX, out theY);
    }

    /// <summary>Lambda wrapper for N-D scalar functions with value and gradient.</summary>
    public sealed class VectorLambdaWithGradient : IVectorFunctionWithGradient
    {
        private readonly VectorLambdaFunction myValueLambda;
        private readonly VectorGradientLambdaFunction myGradLambda;

        public VectorLambdaWithGradient(VectorLambdaFunction theValueLambda, VectorGradientLambdaFunction theGradLambda)
        {
            myValueLambda = theValueLambda;
            myGradLambda = theGradLambda;
        }

        public bool Value(math_Vector theX, out double theY) => myValueLambda(theX, out theY);

        public bool Gradient(math_Vector theX, math_Vector theG)
        {
            return myGradLambda(theX, theG);
        }

        /// <summary>Evaluates value and gradient.</summary>
        public bool Values(math_Vector theX, out double theY, math_Vector theG)
        {
            return myValueLambda(theX, out theY) && myGradLambda(theX, theG);
        }
    }

    /// <summary>Helper function to create VectorLambda.</summary>
    public static VectorLambda MakeVector(VectorLambdaFunction theLambda)
    {
        return new VectorLambda(theLambda);
    }

    /// <summary>Helper function to create VectorLambdaWithGradient.</summary>
    public static VectorLambdaWithGradient MakeVectorWithGradient(VectorLambdaFunction theValueLambda,
                                                                  VectorGradientLambdaFunction theGradLambda)
    {
        return new VectorLambdaWithGradient(theValueLambda, theGradLambda);
    }

    /// <summary>Quadratic form functor: f(x) = x^T A x + b^T x + c.</summary>
    public sealed class QuadraticForm : IVectorFunctionWithGradient
    {
        private readonly math_Matrix myA;
        private readonly math_Vector myB;
        private readonly double myC;

        /// <summary>The matrix and vector are copied, as in C++.</summary>
        public QuadraticForm(math_Matrix theA, math_Vector theB, double theC)
        {
            myA = new math_Matrix(theA);
            myB = new math_Vector(theB);
            myC = theC;
        }

        public bool Value(math_Vector theX, out double theY)
        {
            theY = myC;
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                for (int j = theX.Lower(); j <= theX.Upper(); ++j)
                {
                    theY += theX[i] * myA[i, j] * theX[j];
                }
            }
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theY += myB[i] * theX[i];
            }
            return true;
        }

        /// <summary>Gradient: (A + A^T) x + b.</summary>
        public bool Gradient(math_Vector theX, math_Vector theG)
        {
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theG[i] = myB[i];
                for (int j = theX.Lower(); j <= theX.Upper(); ++j)
                {
                    theG[i] += (myA[i, j] + myA[j, i]) * theX[j];
                }
            }
            return true;
        }

        public bool Values(math_Vector theX, out double theY, math_Vector theG)
        {
            return Value(theX, out theY) && Gradient(theX, theG);
        }
    }

    /// <summary>Rosenbrock function: f(x, y) = (a - x)^2 + b(y - x^2)^2, minimum at (a, a^2).</summary>
    public sealed class Rosenbrock : IVectorFunctionWithGradient
    {
        private readonly double myA;
        private readonly double myB;

        public Rosenbrock(double theA = 1.0, double theB = 100.0)
        {
            myA = theA;
            myB = theB;
        }

        public bool Value(math_Vector theX, out double theY)
        {
            double x = theX[theX.Lower()];
            double y = theX[theX.Lower() + 1];
            double t1 = myA - x;
            double t2 = y - x * x;
            theY = t1 * t1 + myB * t2 * t2;
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theG)
        {
            double x = theX[theX.Lower()];
            double y = theX[theX.Lower() + 1];
            double t2 = y - x * x;
            theG[theG.Lower()] = -2.0 * (myA - x) - 4.0 * myB * x * t2;
            theG[theG.Lower() + 1] = 2.0 * myB * t2;
            return true;
        }

        public bool Values(math_Vector theX, out double theY, math_Vector theG)
        {
            return Value(theX, out theY) && Gradient(theX, theG);
        }
    }

    /// <summary>Sphere function: f(x) = sum(x_i^2), minimum at the origin.</summary>
    public sealed class Sphere : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theY)
        {
            theY = 0.0;
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theY += theX[i] * theX[i];
            }
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theG)
        {
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theG[i] = 2.0 * theX[i];
            }
            return true;
        }

        public bool Values(math_Vector theX, out double theY, math_Vector theG)
        {
            return Value(theX, out theY) && Gradient(theX, theG);
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

        public bool Gradient(math_Vector theX, math_Vector theG)
        {
            double x = theX[theX.Lower()];
            double y = theX[theX.Lower() + 1];
            double t1 = x + 2.0 * y - 7.0;
            double t2 = 2.0 * x + y - 5.0;
            theG[theG.Lower()] = 2.0 * t1 + 4.0 * t2;
            theG[theG.Lower() + 1] = 4.0 * t1 + 2.0 * t2;
            return true;
        }

        public bool Values(math_Vector theX, out double theY, math_Vector theG)
        {
            return Value(theX, out theY) && Gradient(theX, theG);
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

        public bool Gradient(math_Vector theX, math_Vector theG)
        {
            double x = theX[theX.Lower()];
            double y = theX[theX.Lower() + 1];
            double y2 = y * y;
            double y3 = y2 * y;
            double t1 = 1.5 - x + x * y;
            double t2 = 2.25 - x + x * y2;
            double t3 = 2.625 - x + x * y3;
            theG[theG.Lower()] = 2.0 * ((y - 1.0) * t1 + (y2 - 1.0) * t2 + (y3 - 1.0) * t3);
            theG[theG.Lower() + 1] = 2.0 * x * (t1 + 2.0 * y * t2 + 3.0 * y2 * t3);
            return true;
        }

        public bool Values(math_Vector theX, out double theY, math_Vector theG)
        {
            return Value(theX, out theY) && Gradient(theX, theG);
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

        public bool Gradient(math_Vector theX, math_Vector theG)
        {
            double x = theX[theX.Lower()];
            double y = theX[theX.Lower() + 1];
            double t1 = x * x + y - 11.0;
            double t2 = x + y * y - 7.0;
            theG[theG.Lower()] = 4.0 * x * t1 + 2.0 * t2;
            theG[theG.Lower() + 1] = 2.0 * t1 + 4.0 * y * t2;
            return true;
        }

        public bool Values(math_Vector theX, out double theY, math_Vector theG)
        {
            return Value(theX, out theY) && Gradient(theX, theG);
        }
    }

    /// <summary>Rastrigin function: f(x) = A*n + sum(x_i^2 - A*cos(2*pi*x_i)), minimum at the origin.</summary>
    public sealed class Rastrigin : IVectorFunctionWithGradient
    {
        private readonly double myA;

        public Rastrigin(double theA = 10.0)
        {
            myA = theA;
        }

        public bool Value(math_Vector theX, out double theY)
        {
            const double aTwoPi = 2.0 * 3.14159265358979323846;
            int n = theX.Upper() - theX.Lower() + 1;
            theY = myA * (double)n;
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theY += theX[i] * theX[i] - myA * Math.Cos(aTwoPi * theX[i]);
            }
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theG)
        {
            const double aTwoPi = 2.0 * 3.14159265358979323846;
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theG[i] = 2.0 * theX[i] + myA * aTwoPi * Math.Sin(aTwoPi * theX[i]);
            }
            return true;
        }

        public bool Values(math_Vector theX, out double theY, math_Vector theG)
        {
            return Value(theX, out theY) && Gradient(theX, theG);
        }
    }

    /// <summary>Ackley function, minimum at the origin. Value only.</summary>
    public sealed class Ackley : IVectorFunction
    {
        private readonly double myA;
        private readonly double myB;
        private readonly double myC;

        public Ackley(double theA = 20.0, double theB = 0.2, double theC = 2.0 * 3.14159265358979323846)
        {
            myA = theA;
            myB = theB;
            myC = theC;
        }

        public bool Value(math_Vector theX, out double theY)
        {
            const double aE = 2.718281828459045;
            int n = theX.Upper() - theX.Lower() + 1;
            double aSumSq = 0.0;
            double aSumCos = 0.0;
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                aSumSq += theX[i] * theX[i];
                aSumCos += Math.Cos(myC * theX[i]);
            }
            theY = -myA * Math.Exp(-myB * Math.Sqrt(aSumSq / n)) - Math.Exp(aSumCos / n) + myA + aE;
            return true;
        }
    }

    /// <summary>Linear least squares residual: f(x) = ||Ax - b||^2.</summary>
    public sealed class LinearResidual : IVectorFunctionWithGradient
    {
        private readonly math_Matrix myA;
        private readonly math_Vector myB;

        /// <summary>The matrix and vector are copied, as in C++.</summary>
        public LinearResidual(math_Matrix theA, math_Vector theB)
        {
            myA = new math_Matrix(theA);
            myB = new math_Vector(theB);
        }

        public bool Value(math_Vector theX, out double theY)
        {
            theY = 0.0;
            for (int i = myA.LowerRow(); i <= myA.UpperRow(); ++i)
            {
                double aResidual = -myB[i];
                for (int j = myA.LowerCol(); j <= myA.UpperCol(); ++j)
                {
                    aResidual += myA[i, j] * theX[j];
                }
                theY += aResidual * aResidual;
            }
            return true;
        }

        /// <summary>Gradient: 2 A^T (Ax - b).</summary>
        public bool Gradient(math_Vector theX, math_Vector theG)
        {
            int m = myA.UpperRow() - myA.LowerRow() + 1;
            math_Vector aResidual = new(1, m);
            for (int i = myA.LowerRow(); i <= myA.UpperRow(); ++i)
            {
                aResidual[i - myA.LowerRow() + 1] = -myB[i];
                for (int j = myA.LowerCol(); j <= myA.UpperCol(); ++j)
                {
                    aResidual[i - myA.LowerRow() + 1] += myA[i, j] * theX[j];
                }
            }
            for (int j = myA.LowerCol(); j <= myA.UpperCol(); ++j)
            {
                theG[j] = 0.0;
                for (int i = myA.LowerRow(); i <= myA.UpperRow(); ++i)
                {
                    theG[j] += 2.0 * myA[i, j] * aResidual[i - myA.LowerRow() + 1];
                }
            }
            return true;
        }

        public bool Values(math_Vector theX, out double theY, math_Vector theG)
        {
            return Value(theX, out theY) && Gradient(theX, theG);
        }
    }

    /// <summary>Lambda wrapper for a system of equations F(x) = 0.</summary>
    public sealed class SystemLambda : ISystemFunction
    {
        private readonly SystemLambdaFunction myLambda;
        private readonly int myNbEquations;

        public SystemLambda(SystemLambdaFunction theLambda, int theNbEquations)
        {
            myLambda = theLambda;
            myNbEquations = theNbEquations;
        }

        public int NbEquations() => myNbEquations;

        public bool Value(math_Vector theX, math_Vector theF) => myLambda(theX, theF);
    }

    /// <summary>Helper function to create SystemLambda.</summary>
    public static SystemLambda MakeSystem(SystemLambdaFunction theLambda, int theNbEquations)
    {
        return new SystemLambda(theLambda, theNbEquations);
    }
}
