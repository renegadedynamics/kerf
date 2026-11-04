// Port of: src/FoundationClasses/TKMath/math/math_NewtonMinimum.hxx
// Port of: src/FoundationClasses/TKMath/math/math_NewtonMinimum.lxx
// Port of: src/FoundationClasses/TKMath/math/math_NewtonMinimum.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Occt;
using System.Globalization;
using static Kerf.Engine.Numerics.NumericStatus;

namespace Kerf.Engine.Numerics;

/// <summary>Newton's method to find a minimum of a function of several variables whose hessian is known.</summary>
/// <remarks>OCCT declares <c>IsConvex()</c> but never defines it, so it is left out.</remarks>
internal class NewtonMinimum
{
    protected NumericStatus _theStatus;
    protected math_Vector _theLocation;
    protected math_Vector _theGradient;
    protected math_Vector _theStep;
    protected Matrix _theHessian;
    protected double _previousMinimum;
    protected double _theMinimum;
    protected double _minEigenValue;
    protected double _xTol;
    protected double _cTol;
    protected int _nbiter;
    protected bool _noConvexTreatement;
    protected bool _convex;
    protected bool _isBoundsDefined;
    protected math_Vector _left;
    protected math_Vector _right;

    private bool _done;
    private int _itermax;

    // The eigenvalue and LU solvers, kept from one iteration and one Perform to the next (unit 10J; performance.md's
    // proposal 4): OCCT constructs both at every iteration, and in Bento's "extrude until" they were most of the bytes
    // allocated. They hold no state between iterations: each Perform overwrites all of it.
    private Jacobi? _eigen;
    private Gauss? _lu;

    /// <summary>
    /// Prepares the algorithm; <see cref="Perform"/> does the job. Iterations stop if (!theWithSingularity) and the
    /// hessian is not definite positive (its smallest eigenvalue below theConvexity), or when
    /// <see cref="IsConverged"/> is true for 2 successive iterations.
    /// </summary>
    public NewtonMinimum(MultipleVarFunctionWithHessian function, double tolerance = 1.0e-7,
                              int nbIterations = 40, double convexity = 1.0e-6, bool withSingularity = true)
    {
        _theStatus = NotBracketed;
        _theLocation = new math_Vector(1, function.VariableCount);
        _theGradient = new math_Vector(1, function.VariableCount);
        _theStep = new math_Vector(1, function.VariableCount, 10.0 * tolerance);
        _theHessian = new Matrix(1, function.VariableCount, 1, function.VariableCount);
        _previousMinimum = 0.0;
        _theMinimum = 0.0;
        _minEigenValue = 0.0;
        _xTol = tolerance;
        _cTol = convexity;
        _nbiter = 0;
        _noConvexTreatement = withSingularity;
        _convex = true;
        _isBoundsDefined = false;
        _left = new math_Vector(1, function.VariableCount, 0.0);
        _right = new math_Vector(1, function.VariableCount, 0.0);
        _done = false;
        _itermax = nbIterations;
    }

    /// <summary>Sets the boundaries for conditional optimization.</summary>
    public void SetBoundary(math_Vector leftBorder, math_Vector rightBorder)
    {
        _left.Initialized(leftBorder);
        _right.Initialized(rightBorder);
        _isBoundsDefined = true;
    }

    /// <summary>Searches the minimum from <paramref name="startingPoint"/>.</summary>
    public void Perform(MultipleVarFunctionWithHessian f, math_Vector startingPoint)
    {
        math_Vector point1 = new(1, f.VariableCount);
        point1.Initialized(startingPoint);
        math_Vector point2 = new(1, f.VariableCount);
        math_Vector precedent = point1;
        math_Vector suivant = point2;

        bool ok = true;
        int nbConv = 0, ii, nreduction;
        double vPrecedent, vItere;

        _done = true;
        _theStatus = OK;
        _nbiter = 0;

        while (ok && (nbConv < 2))
        {
            _nbiter++;

            // Positionnement

            ok = f.Values(precedent, out vPrecedent, _theGradient, _theHessian);
            if (!ok)
            {
                _done = false;
                _theStatus = FunctionError;
                return;
            }

            if (_nbiter == 1)
            {
                _previousMinimum = vPrecedent;
                _theMinimum = vPrecedent;
            }

            // Traitement de la non convexite

            Jacobi calculVP = _eigen ??= new Jacobi(_theHessian.RowNumber());
            calculVP.Perform(_theHessian);
            if (!calculVP.IsDone)
            {
                _done = false;
                _theStatus = FunctionError;
                return;
            }

            _minEigenValue = calculVP.Values()[calculVP.Values().Min()];

            if (_minEigenValue < _cTol)
            {
                _convex = false;
                if (_noConvexTreatement && // Treatment is allowed.
                    Math.Abs(_minEigenValue) > _cTol) // Treatment will have effect.
                {
                    double delta = _cTol + 0.1 * Math.Abs(_minEigenValue) - _minEigenValue;
                    for (ii = 1; ii <= _theGradient.Length(); ii++)
                    {
                        _theHessian[ii, ii] += delta;
                    }
                }
                else
                {
                    _done = false;
                    _theStatus = FunctionError;
                    return;
                }
            }

            // Schemas de Newton

            Gauss lu = _lu ??= new Gauss(_theHessian.RowNumber());
            lu.Perform(_theHessian, _cTol / 100);
            if (!lu.IsDone)
            {
                _done = false;
                _theStatus = DirectionSearchError;
                return;
            }

            lu.Solve(_theGradient, _theStep);

            if (_isBoundsDefined)
            {
                // Project point on bounds or nullify TheStep coords if point lies on boundary.

                suivant.Subtract(precedent, _theStep); // OCCT: suivant = precedent - TheStep, without the temporary (unit 9E)
                double mult = RealLast();
                for (int idx = 1; idx <= _left.Upper(); idx++)
                {
                    double absStep = Math.Abs(_theStep[idx]);
                    if (absStep < Constants.Resolution())
                    {
                        continue;
                    }

                    if (suivant.Value(idx) < _left[idx])
                    {
                        double value = Math.Abs(precedent.Value(idx) - _left[idx]) / absStep;
                        mult = Min(value, mult);
                    }

                    if (suivant.Value(idx) > _right[idx])
                    {
                        double value = Math.Abs(precedent.Value(idx) - _right[idx]) / absStep;
                        mult = Min(value, mult);
                    }
                }

                if (mult != RealLast())
                {
                    if (mult > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        // Project point into param space.
                        _theStep *= mult;
                    }
                    else
                    {
                        // Old point on border and new point out of border:
                        // Nullify corresponding TheStep indexes.
                        for (int idx = 1; idx <= _left.Upper(); idx++)
                        {
                            if ((Math.Abs(precedent.Value(idx) - _right[idx]) < global::Kerf.Engine.Foundation.Precision.PConfusion() && _theStep[idx] < 0.0)
                                || (Math.Abs(precedent.Value(idx) - _left[idx]) < global::Kerf.Engine.Foundation.Precision.PConfusion() && _theStep[idx] > 0.0))
                            {
                                _theStep[idx] = 0.0;
                            }
                        }
                    }
                }
            }

            bool hasProblem = false;
            do
            {
                suivant.Subtract(precedent, _theStep); // OCCT: suivant = precedent - TheStep, without the temporary (unit 9E)
                //  Gestion de la convergence
                hasProblem = !(f.Value(suivant, out _theMinimum));
                if (hasProblem)
                {
                    _theStep /= 2.0;
                }
            } while (hasProblem);

            if (IsConverged)
            {
                nbConv++;
            }
            else
            {
                nbConv = 0;
            }

            //  Controle et corrections.

            vItere = _theMinimum;
            _theMinimum = _previousMinimum;
            nreduction = 0;
            while (vItere > vPrecedent && nreduction < 10)
            {
                _theStep *= 0.4;
                suivant.Subtract(precedent, _theStep); // OCCT: suivant = precedent - TheStep, without the temporary (unit 9E)
                f.Value(suivant, out vItere);
                nreduction++;
            }

            if (vItere <= vPrecedent)
            {
                math_Vector auxiliaire = precedent;
                precedent = suivant;
                suivant = auxiliaire;
                _previousMinimum = vPrecedent;
                _theMinimum = vItere;
                ok = (_nbiter < _itermax);
                if (!ok && nbConv < 2)
                {
                    _theStatus = TooManyIterations;
                }
            }
            else
            {
                ok = false;
                _theStatus = DirectionSearchError;
            }
        }

        _theLocation.Initialized(precedent);
    }

    /// <summary>
    /// Called at the end of each iteration to check the convergence:
    /// || Xi+1 - Xi || &lt; Tolerance or || F(Xi+1) - F(Xi)|| &lt; Tolerance * || F(Xi) ||. A sub-class can redefine it.
    /// </summary>
    public virtual bool IsConverged
    {
        get
        {
            return ((_theStep.Norm() <= _xTol) || (Math.Abs(_theMinimum - _previousMinimum) <= _xTol * Math.Abs(_previousMinimum)));
        }
    }

    /// <summary>False if an error has occurred.</summary>
    public bool IsDone => _done;

    /// <summary>The location of the minimum (shared: don't modify). Throws <see cref="NotDoneException"/> if an error occurred.</summary>
    public math_Vector Location()
    {
        StdFail_NotDone_Raise_if(!_done, "NewtonMinimum");
        return _theLocation;
    }

    /// <summary>Copies the location of the minimum into <paramref name="loc"/>.</summary>
    public void Location(math_Vector loc)
    {
        StdFail_NotDone_Raise_if(!_done, "NewtonMinimum");
        loc.Initialized(_theLocation);
    }

    /// <summary>The value of the minimum. Throws <see cref="NotDoneException"/> if an error occurred.</summary>
    public double Minimum()
    {
        StdFail_NotDone_Raise_if(!_done, "NewtonMinimum");
        return _theMinimum;
    }

    /// <summary>The gradient at the minimum (shared: don't modify). Throws <see cref="NotDoneException"/> if an error occurred.</summary>
    public math_Vector Gradient()
    {
        StdFail_NotDone_Raise_if(!_done, "NewtonMinimum");
        return _theGradient;
    }

    /// <summary>Copies the gradient at the minimum into <paramref name="grad"/>.</summary>
    public void Gradient(math_Vector grad)
    {
        StdFail_NotDone_Raise_if(!_done, "NewtonMinimum");
        grad.Initialized(_theGradient);
    }

    /// <summary>The number of iterations done. Throws <see cref="NotDoneException"/> if an error occurred.</summary>
    public int IterationCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, "NewtonMinimum");
            return _nbiter;
        }
    }

    /// <summary>The status of the computation.</summary>
    public NumericStatus GetStatus() => _theStatus;

    public void Dump(TextWriter o)
    {
        o.Write("math_Newton Optimisation: ");
        o.Write(" Done   =" + (_done ? 1 : 0) + '\n');
        o.Write(" Status = " + (int)_theStatus + '\n');
        o.Write(" Location Vector = " + Location() + '\n');
        o.Write(" Minimum value = " + Minimum().ToString(CultureInfo.InvariantCulture) + '\n');
        o.Write(" Previous value = " + _previousMinimum.ToString(CultureInfo.InvariantCulture) + '\n');
        o.Write(" Number of iterations = " + IterationCount + '\n');
        o.Write(" Convexity = " + (_convex ? 1 : 0) + '\n');
        o.Write(" Eigen Value = " + _minEigenValue.ToString(CultureInfo.InvariantCulture) + '\n');
    }

    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
