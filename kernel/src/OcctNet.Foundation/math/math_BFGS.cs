// Port of: src/FoundationClasses/TKMath/math/math_BFGS.hxx
// Port of: src/FoundationClasses/TKMath/math/math_BFGS.lxx
// Port of: src/FoundationClasses/TKMath/math/math_BFGS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.math_Status;

namespace Occt;

/// <summary>
/// The Broyden-Fletcher-Goldfarb-Shanno variant of the Davidson-Fletcher-Powell method to find a minimum of a
/// function of several variables whose gradient is known. Optionally bounded (<see cref="SetBoundary"/>).
/// </summary>
public class math_BFGS
{
    protected math_Status TheStatus;
    protected math_Vector TheLocation;
    protected math_Vector TheGradient;
    protected double PreviousMinimum;
    protected double TheMinimum;
    protected double XTol;
    protected double EPSZ;
    protected int nbiter;
    protected bool myIsBoundsDefined;
    protected math_Vector myLeft;
    protected math_Vector myRight;

    private bool Done;
    private int Itermax;

    // l'utilisation de math_BrentMinumim pur trouver un minimum dans une direction
    // donnee n'est pas du tout optimale. voir peut etre interpolation cubique
    // classique et aussi essayer "recherche unidimensionnelle economique"
    // PROGRAMMATION MATHEMATIQUE (theorie et algorithmes) tome1 page 82.

    // Target function for 1D problem, point and direction are known.
    private sealed class DirFunction : math_FunctionWithDerivative
    {
        private readonly math_Vector P0;
        private readonly math_Vector Dir;
        private readonly math_Vector P;
        private readonly math_Vector G;
        private readonly math_MultipleVarFunctionWithGradient F;

        public DirFunction(math_Vector V1, math_Vector V2, math_Vector V3, math_Vector V4, math_MultipleVarFunctionWithGradient f)
        {
            P0 = V1;
            Dir = V2;
            P = V3;
            G = V4;
            F = f;
        }

        public void Initialize(math_Vector p0, math_Vector dir)
        {
            P0.Initialized(p0);
            Dir.Initialized(dir);
        }

        public void TheGradient(math_Vector Grad) => Grad.Initialized(G);

        public override bool Value(double x, out double fval)
        {
            P.Initialized(Dir);
            P.Multiply(x);
            P.Add(P0);
            fval = 0.0;
            return F.Value(P, out fval);
        }

        public override bool Values(double x, out double fval, out double D)
        {
            P.Initialized(Dir);
            P.Multiply(x);
            P.Add(P0);
            fval = D = 0.0;
            if (F.Values(P, out fval, G))
            {
                D = G.Multiplied(Dir);
                return true;
            }

            return false;
        }

        public override bool Derivative(double x, out double D)
        {
            P.Initialized(Dir);
            P.Multiply(x);
            P.Add(P0);
            double fval;
            D = 0.0;
            if (F.Values(P, out fval, G))
            {
                D = G.Multiplied(Dir);
                return true;
            }

            return false;
        }
    }

    // Computes the appropriate initial value of scale factor to apply
    // to the direction to approach to the minimum of the function
    private static bool ComputeInitScale(double theF0, math_Vector theDir, math_Vector theGr, out double theScale)
    {
        theScale = 0.0;
        double dy1 = theGr * theDir;
        if (Math.Abs(dy1) < RealSmall())
        {
            return false;
        }

        double aHnr1 = theDir.Norm2();
        double alfa = 0.7 * (-theF0) / dy1;
        theScale = 0.015 / Math.Sqrt(aHnr1);
        if (theScale > alfa)
        {
            theScale = alfa;
        }

        return true;
    }

    // For a given point and direction, and bounding box, finds the min and max scale factors with which the point
    // reaches the borders if we apply translation Point+Dir*Scale. False if the point is out of bounds.
    private static bool ComputeMinMaxScale(math_Vector thePoint, math_Vector theDir, math_Vector theLeft, math_Vector theRight,
                                           ref double theMinScale, ref double theMaxScale)
    {
        for (int anIdx = 1; anIdx <= theLeft.Upper(); anIdx++)
        {
            double aLeft = theLeft[anIdx] - thePoint[anIdx];
            double aRight = theRight[anIdx] - thePoint[anIdx];
            if (Math.Abs(theDir[anIdx]) > RealSmall())
            {
                // Use PConfusion to get off a little from the bounds to prevent
                // possible refuse in Value function.
                double aLScale = (aLeft + Precision.PConfusion()) / theDir[anIdx];
                double aRScale = (aRight - Precision.PConfusion()) / theDir[anIdx];
                if (Math.Abs(aLeft) < Precision.PConfusion())
                {
                    // Point is on the left border.
                    theMaxScale = Min(theMaxScale, Max(0.0, aRScale));
                    theMinScale = Max(theMinScale, Min(0.0, aRScale));
                }
                else if (Math.Abs(aRight) < Precision.PConfusion())
                {
                    // Point is on the right border.
                    theMaxScale = Min(theMaxScale, Max(0.0, aLScale));
                    theMinScale = Max(theMinScale, Min(0.0, aLScale));
                }
                else if (aLeft * aRight < 0)
                {
                    // Point is inside allowed range.
                    theMaxScale = Min(theMaxScale, Max(aLScale, aRScale));
                    theMinScale = Max(theMinScale, Min(aLScale, aRScale));
                }
                else
                {
                    // point is out of bounds
                    return false;
                }
            }
            else
            {
                // Direction is parallel to the border.
                // Check that the point is not out of bounds
                if (aLeft > Precision.PConfusion() || aRight < -Precision.PConfusion())
                {
                    return false;
                }
            }
        }

        return true;
    }

    // Solves 1D minimization problem when point and directions are known.
    private static bool MinimizeDirection(math_Vector P, double F0, math_Vector Gr, math_Vector Dir, ref double Result,
                                          DirFunction F, bool isBounds, math_Vector theLeft, math_Vector theRight)
    {
        double lambda;
        if (!ComputeInitScale(F0, Dir, Gr, out lambda))
        {
            return false;
        }

        // by default the scaling range is unlimited
        double aMinLambda = -Precision.Infinite();
        double aMaxLambda = Precision.Infinite();
        if (isBounds)
        {
            // limit the scaling range taking into account the bounds
            if (!ComputeMinMaxScale(P, Dir, theLeft, theRight, ref aMinLambda, ref aMaxLambda))
            {
                return false;
            }

            if (aMinLambda > -Precision.PConfusion() && aMaxLambda < Precision.PConfusion())
            {
                // Point is on the border and the direction shows outside.
                // Make direction to go along the border
                for (int anIdx = 1; anIdx <= theLeft.Upper(); anIdx++)
                {
                    if ((Math.Abs(P[anIdx] - theRight[anIdx]) < Precision.PConfusion() && Dir[anIdx] > 0.0)
                        || (Math.Abs(P[anIdx] - theLeft[anIdx]) < Precision.PConfusion() && Dir[anIdx] < 0.0))
                    {
                        Dir[anIdx] = 0.0;
                    }
                }

                // re-compute scale values with new direction
                if (!ComputeInitScale(F0, Dir, Gr, out lambda))
                {
                    return false;
                }

                if (!ComputeMinMaxScale(P, Dir, theLeft, theRight, ref aMinLambda, ref aMaxLambda))
                {
                    return false;
                }
            }

            lambda = Min(lambda, aMaxLambda);
            lambda = Max(lambda, aMinLambda);
        }

        F.Initialize(P, Dir);
        double F1;
        if (!F.Value(lambda, out F1))
        {
            return false;
        }

        math_BracketMinimum Bracket = new(0.0, lambda);
        if (isBounds)
        {
            Bracket.SetLimits(aMinLambda, aMaxLambda);
        }

        Bracket.SetFA(F0);
        Bracket.SetFB(F1);
        Bracket.Perform(F);
        if (Bracket.IsDone())
        {
            // find minimum inside the bracket
            double ax, xx, bx, Fax, Fxx, Fbx;
            Bracket.Values(out ax, out xx, out bx);
            Bracket.FunctionValues(out Fax, out Fxx, out Fbx);

            int niter = 100;
            double tol = 1.0e-03;
            math_BrentMinimum Sol = new(tol, Fxx, niter, 1.0e-08);
            Sol.Perform(F, ax, xx, bx);
            if (Sol.IsDone())
            {
                double Scale = Sol.Location();
                Result = Sol.Minimum();
                Dir.Multiply(Scale);
                P.Add(Dir);
                return true;
            }
        }
        else if (isBounds)
        {
            // Bracket definition is failure. If the bounds are defined then
            // set current point to intersection with bounds
            double aFMin, aFMax;
            if (!F.Value(aMinLambda, out aFMin))
            {
                return false;
            }

            if (!F.Value(aMaxLambda, out aFMax))
            {
                return false;
            }

            double aBestLambda;
            if (aFMin < aFMax)
            {
                aBestLambda = aMinLambda;
                Result = aFMin;
            }
            else
            {
                aBestLambda = aMaxLambda;
                Result = aFMax;
            }

            Dir.Multiply(aBestLambda);
            P.Add(Dir);
            return true;
        }

        return false;
    }

    /// <summary>Prepares the algorithm; <see cref="Perform"/> does the job.</summary>
    public math_BFGS(int NbVariables, double Tolerance = 1.0e-8, int NbIterations = 200, double ZEPS = 1.0e-12)
    {
        TheStatus = math_OK;
        TheLocation = new math_Vector(1, NbVariables);
        TheGradient = new math_Vector(1, NbVariables);
        PreviousMinimum = 0.0;
        TheMinimum = 0.0;
        XTol = Tolerance;
        EPSZ = ZEPS;
        nbiter = 0;
        myIsBoundsDefined = false;
        myLeft = new math_Vector(1, NbVariables, 0.0);
        myRight = new math_Vector(1, NbVariables, 0.0);
        Done = false;
        Itermax = NbIterations;
    }

    /// <summary>Sets the boundaries for conditional optimization; the starting point must be within them.</summary>
    public void SetBoundary(math_Vector theLeftBorder, math_Vector theRightBorder)
    {
        myLeft.Initialized(theLeftBorder);
        myRight.Initialized(theRightBorder);
        myIsBoundsDefined = true;
    }

    /// <summary>Minimizes F from <paramref name="StartingPoint"/>.</summary>
    public void Perform(math_MultipleVarFunctionWithGradient F, math_Vector StartingPoint)
    {
        int n = TheLocation.Length();
        bool Good = true;
        int j, i;
        double fae, fad, fac;

        math_Vector xi = new(1, n), dg = new(1, n), hdg = new(1, n);
        math_Matrix hessin = new(1, n, 1, n);
        hessin.Init(0.0);

        math_Vector Temp1 = new(1, n);
        math_Vector Temp2 = new(1, n);
        math_Vector Temp3 = new(1, n);
        math_Vector Temp4 = new(1, n);
        DirFunction F_Dir = new(Temp1, Temp2, Temp3, Temp4, F);

        TheLocation.Initialized(StartingPoint);
        Good = F.Values(TheLocation, out PreviousMinimum, TheGradient);
        if (!Good)
        {
            Done = false;
            TheStatus = math_FunctionError;
            return;
        }

        for (i = 1; i <= n; i++)
        {
            hessin[i, i] = 1.0;
            xi[i] = -TheGradient[i];
        }

        for (nbiter = 1; nbiter <= Itermax; nbiter++)
        {
            TheMinimum = PreviousMinimum;
            bool IsGood = MinimizeDirection(TheLocation, TheMinimum, TheGradient, xi, ref TheMinimum, F_Dir,
                                            myIsBoundsDefined, myLeft, myRight);

            if (IsSolutionReached(F))
            {
                Done = true;
                TheStatus = math_OK;
                return;
            }

            if (!IsGood)
            {
                Done = false;
                TheStatus = math_DirectionSearchError;
                return;
            }

            PreviousMinimum = TheMinimum;

            dg.Initialized(TheGradient);

            Good = F.Values(TheLocation, out TheMinimum, TheGradient);
            if (!Good)
            {
                Done = false;
                TheStatus = math_FunctionError;
                return;
            }

            for (i = 1; i <= n; i++)
            {
                dg[i] = TheGradient[i] - dg[i];
            }

            for (i = 1; i <= n; i++)
            {
                hdg[i] = 0.0;
                for (j = 1; j <= n; j++)
                {
                    hdg[i] += hessin[i, j] * dg[j];
                }
            }

            fac = fae = 0.0;
            for (i = 1; i <= n; i++)
            {
                fac += dg[i] * xi[i];
                fae += dg[i] * hdg[i];
            }

            fac = 1.0 / fac;
            fad = 1.0 / fae;

            for (i = 1; i <= n; i++)
            {
                dg[i] = fac * xi[i] - fad * hdg[i];
            }

            for (i = 1; i <= n; i++)
            {
                for (j = 1; j <= n; j++)
                {
                    hessin[i, j] += fac * xi[i] * xi[j] - fad * hdg[i] * hdg[j] + fae * dg[i] * dg[j];
                }
            }

            for (i = 1; i <= n; i++)
            {
                xi[i] = 0.0;
                for (j = 1; j <= n; j++)
                {
                    xi[i] -= hessin[i, j] * TheGradient[j];
                }
            }
        }

        Done = false;
        TheStatus = math_TooManyIterations;
    }

    /// <summary>
    /// True when 2.0 * abs(Fi - Fi-1) &lt;= Tolerance * (abs(Fi) + abs(Fi-1) + ZEPS). A sub-class can redefine it.
    /// </summary>
    public virtual bool IsSolutionReached(math_MultipleVarFunctionWithGradient F)
    {
        return 2.0 * Math.Abs(TheMinimum - PreviousMinimum) <= XTol * (Math.Abs(TheMinimum) + Math.Abs(PreviousMinimum) + EPSZ);
    }

    public bool IsDone() => Done;

    /// <summary>The location of the minimum (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if it was not found.</summary>
    public math_Vector Location()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return TheLocation;
    }

    /// <summary>Copies the location of the minimum into <paramref name="Loc"/>.</summary>
    public void Location(math_Vector Loc)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        Loc.Initialized(TheLocation);
    }

    /// <summary>The value of the minimum. Throws <see cref="StdFail_NotDone"/> if it was not found.</summary>
    public double Minimum()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return TheMinimum;
    }

    /// <summary>The gradient at the minimum (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if it was not found.</summary>
    public math_Vector Gradient()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return TheGradient;
    }

    /// <summary>Copies the gradient at the minimum into <paramref name="Grad"/>.</summary>
    public void Gradient(math_Vector Grad)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        Grad.Initialized(TheGradient);
    }

    /// <summary>The number of iterations done. Throws <see cref="StdFail_NotDone"/> if the minimum was not found.</summary>
    public int NbIterations()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return nbiter;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_BFGS resolution: ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Location Vector = " + Location() + "\n");
            o.Write(" Minimum value = " + Minimum().ToString(CultureInfo.InvariantCulture) + "\n");
            o.Write(" Number of iterations = " + NbIterations() + "\n");
        }
        else
        {
            o.Write(" Status = not Done because " + (int)TheStatus + "\n");
        }
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
