// Port of: src/FoundationClasses/TKMath/math/math_NewtonMinimum.hxx
// Port of: src/FoundationClasses/TKMath/math/math_NewtonMinimum.lxx
// Port of: src/FoundationClasses/TKMath/math/math_NewtonMinimum.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.math_Status;

namespace Occt;

/// <summary>Newton's method to find a minimum of a function of several variables whose hessian is known.</summary>
/// <remarks>OCCT declares <c>IsConvex()</c> but never defines it, so it is left out.</remarks>
public class math_NewtonMinimum
{
    protected math_Status TheStatus;
    protected math_Vector TheLocation;
    protected math_Vector TheGradient;
    protected math_Vector TheStep;
    protected math_Matrix TheHessian;
    protected double PreviousMinimum;
    protected double TheMinimum;
    protected double MinEigenValue;
    protected double XTol;
    protected double CTol;
    protected int nbiter;
    protected bool NoConvexTreatement;
    protected bool Convex;
    protected bool myIsBoundsDefined;
    protected math_Vector myLeft;
    protected math_Vector myRight;

    private bool Done;
    private int Itermax;

    /// <summary>
    /// Prepares the algorithm; <see cref="Perform"/> does the job. Iterations stop if (!theWithSingularity) and the
    /// hessian is not definite positive (its smallest eigenvalue below theConvexity), or when
    /// <see cref="IsConverged"/> is true for 2 successive iterations.
    /// </summary>
    public math_NewtonMinimum(math_MultipleVarFunctionWithHessian theFunction, double theTolerance = 1.0e-7,
                              int theNbIterations = 40, double theConvexity = 1.0e-6, bool theWithSingularity = true)
    {
        TheStatus = math_NotBracketed;
        TheLocation = new math_Vector(1, theFunction.NbVariables());
        TheGradient = new math_Vector(1, theFunction.NbVariables());
        TheStep = new math_Vector(1, theFunction.NbVariables(), 10.0 * theTolerance);
        TheHessian = new math_Matrix(1, theFunction.NbVariables(), 1, theFunction.NbVariables());
        PreviousMinimum = 0.0;
        TheMinimum = 0.0;
        MinEigenValue = 0.0;
        XTol = theTolerance;
        CTol = theConvexity;
        nbiter = 0;
        NoConvexTreatement = theWithSingularity;
        Convex = true;
        myIsBoundsDefined = false;
        myLeft = new math_Vector(1, theFunction.NbVariables(), 0.0);
        myRight = new math_Vector(1, theFunction.NbVariables(), 0.0);
        Done = false;
        Itermax = theNbIterations;
    }

    /// <summary>Sets the boundaries for conditional optimization.</summary>
    public void SetBoundary(math_Vector theLeftBorder, math_Vector theRightBorder)
    {
        myLeft.Initialized(theLeftBorder);
        myRight.Initialized(theRightBorder);
        myIsBoundsDefined = true;
    }

    /// <summary>Searches the minimum from <paramref name="StartingPoint"/>.</summary>
    public void Perform(math_MultipleVarFunctionWithHessian F, math_Vector StartingPoint)
    {
        math_Vector Point1 = new(1, F.NbVariables());
        Point1.Initialized(StartingPoint);
        math_Vector Point2 = new(1, F.NbVariables());
        math_Vector precedent = Point1;
        math_Vector suivant = Point2;

        bool Ok = true;
        int NbConv = 0, ii, Nreduction;
        double VPrecedent, VItere;

        Done = true;
        TheStatus = math_OK;
        nbiter = 0;

        while (Ok && (NbConv < 2))
        {
            nbiter++;

            // Positionnement

            Ok = F.Values(precedent, out VPrecedent, TheGradient, TheHessian);
            if (!Ok)
            {
                Done = false;
                TheStatus = math_FunctionError;
                return;
            }

            if (nbiter == 1)
            {
                PreviousMinimum = VPrecedent;
                TheMinimum = VPrecedent;
            }

            // Traitement de la non convexite

            math_Jacobi CalculVP = new(TheHessian);
            if (!CalculVP.IsDone())
            {
                Done = false;
                TheStatus = math_FunctionError;
                return;
            }

            MinEigenValue = CalculVP.Values()[CalculVP.Values().Min()];

            if (MinEigenValue < CTol)
            {
                Convex = false;
                if (NoConvexTreatement && // Treatment is allowed.
                    Math.Abs(MinEigenValue) > CTol) // Treatment will have effect.
                {
                    double Delta = CTol + 0.1 * Math.Abs(MinEigenValue) - MinEigenValue;
                    for (ii = 1; ii <= TheGradient.Length(); ii++)
                    {
                        TheHessian[ii, ii] += Delta;
                    }
                }
                else
                {
                    Done = false;
                    TheStatus = math_FunctionError;
                    return;
                }
            }

            // Schemas de Newton

            math_Gauss LU = new(TheHessian, CTol / 100);
            if (!LU.IsDone())
            {
                Done = false;
                TheStatus = math_DirectionSearchError;
                return;
            }

            LU.Solve(TheGradient, TheStep);

            if (myIsBoundsDefined)
            {
                // Project point on bounds or nullify TheStep coords if point lies on boundary.

                suivant.Initialized(precedent - TheStep);
                double aMult = RealLast();
                for (int anIdx = 1; anIdx <= myLeft.Upper(); anIdx++)
                {
                    double anAbsStep = Math.Abs(TheStep[anIdx]);
                    if (anAbsStep < gp.Resolution())
                    {
                        continue;
                    }

                    if (suivant.Value(anIdx) < myLeft[anIdx])
                    {
                        double aValue = Math.Abs(precedent.Value(anIdx) - myLeft[anIdx]) / anAbsStep;
                        aMult = Min(aValue, aMult);
                    }

                    if (suivant.Value(anIdx) > myRight[anIdx])
                    {
                        double aValue = Math.Abs(precedent.Value(anIdx) - myRight[anIdx]) / anAbsStep;
                        aMult = Min(aValue, aMult);
                    }
                }

                if (aMult != RealLast())
                {
                    if (aMult > Precision.PConfusion())
                    {
                        // Project point into param space.
                        TheStep *= aMult;
                    }
                    else
                    {
                        // Old point on border and new point out of border:
                        // Nullify corresponding TheStep indexes.
                        for (int anIdx = 1; anIdx <= myLeft.Upper(); anIdx++)
                        {
                            if ((Math.Abs(precedent.Value(anIdx) - myRight[anIdx]) < Precision.PConfusion() && TheStep[anIdx] < 0.0)
                                || (Math.Abs(precedent.Value(anIdx) - myLeft[anIdx]) < Precision.PConfusion() && TheStep[anIdx] > 0.0))
                            {
                                TheStep[anIdx] = 0.0;
                            }
                        }
                    }
                }
            }

            bool hasProblem = false;
            do
            {
                suivant.Initialized(precedent - TheStep);
                //  Gestion de la convergence
                hasProblem = !(F.Value(suivant, out TheMinimum));
                if (hasProblem)
                {
                    TheStep /= 2.0;
                }
            } while (hasProblem);

            if (IsConverged())
            {
                NbConv++;
            }
            else
            {
                NbConv = 0;
            }

            //  Controle et corrections.

            VItere = TheMinimum;
            TheMinimum = PreviousMinimum;
            Nreduction = 0;
            while (VItere > VPrecedent && Nreduction < 10)
            {
                TheStep *= 0.4;
                suivant.Initialized(precedent - TheStep);
                F.Value(suivant, out VItere);
                Nreduction++;
            }

            if (VItere <= VPrecedent)
            {
                math_Vector auxiliaire = precedent;
                precedent = suivant;
                suivant = auxiliaire;
                PreviousMinimum = VPrecedent;
                TheMinimum = VItere;
                Ok = (nbiter < Itermax);
                if (!Ok && NbConv < 2)
                {
                    TheStatus = math_TooManyIterations;
                }
            }
            else
            {
                Ok = false;
                TheStatus = math_DirectionSearchError;
            }
        }

        TheLocation.Initialized(precedent);
    }

    /// <summary>
    /// Called at the end of each iteration to check the convergence:
    /// || Xi+1 - Xi || &lt; Tolerance or || F(Xi+1) - F(Xi)|| &lt; Tolerance * || F(Xi) ||. A sub-class can redefine it.
    /// </summary>
    public virtual bool IsConverged()
    {
        return ((TheStep.Norm() <= XTol) || (Math.Abs(TheMinimum - PreviousMinimum) <= XTol * Math.Abs(PreviousMinimum)));
    }

    /// <summary>False if an error has occurred.</summary>
    public bool IsDone() => Done;

    /// <summary>The location of the minimum (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if an error occurred.</summary>
    public math_Vector Location()
    {
        StdFail_NotDone_Raise_if(!Done, "NewtonMinimum");
        return TheLocation;
    }

    /// <summary>Copies the location of the minimum into <paramref name="Loc"/>.</summary>
    public void Location(math_Vector Loc)
    {
        StdFail_NotDone_Raise_if(!Done, "NewtonMinimum");
        Loc.Initialized(TheLocation);
    }

    /// <summary>The value of the minimum. Throws <see cref="StdFail_NotDone"/> if an error occurred.</summary>
    public double Minimum()
    {
        StdFail_NotDone_Raise_if(!Done, "NewtonMinimum");
        return TheMinimum;
    }

    /// <summary>The gradient at the minimum (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if an error occurred.</summary>
    public math_Vector Gradient()
    {
        StdFail_NotDone_Raise_if(!Done, "NewtonMinimum");
        return TheGradient;
    }

    /// <summary>Copies the gradient at the minimum into <paramref name="Grad"/>.</summary>
    public void Gradient(math_Vector Grad)
    {
        StdFail_NotDone_Raise_if(!Done, "NewtonMinimum");
        Grad.Initialized(TheGradient);
    }

    /// <summary>The number of iterations done. Throws <see cref="StdFail_NotDone"/> if an error occurred.</summary>
    public int NbIterations()
    {
        StdFail_NotDone_Raise_if(!Done, "NewtonMinimum");
        return nbiter;
    }

    /// <summary>The status of the computation.</summary>
    public math_Status GetStatus() => TheStatus;

    public void Dump(TextWriter o)
    {
        o.Write("math_Newton Optimisation: ");
        o.Write(" Done   =" + (Done ? 1 : 0) + '\n');
        o.Write(" Status = " + (int)TheStatus + '\n');
        o.Write(" Location Vector = " + Location() + '\n');
        o.Write(" Minimum value = " + Minimum().ToString(CultureInfo.InvariantCulture) + '\n');
        o.Write(" Previous value = " + PreviousMinimum.ToString(CultureInfo.InvariantCulture) + '\n');
        o.Write(" Number of iterations = " + NbIterations() + '\n');
        o.Write(" Convexity = " + (Convex ? 1 : 0) + '\n');
        o.Write(" Eigen Value = " + MinEigenValue.ToString(CultureInfo.InvariantCulture) + '\n');
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
