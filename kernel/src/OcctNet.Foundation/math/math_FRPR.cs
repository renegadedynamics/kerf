// Port of: src/FoundationClasses/TKMath/math/math_FRPR.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FRPR.lxx
// Port of: src/FoundationClasses/TKMath/math/math_FRPR.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.math_Status;

namespace Occt;

/// <summary>
/// The Fletcher-Reeves-Polak-Ribiere conjugate gradient method to find a minimum of a function of several
/// variables whose gradient is known.
/// </summary>
public class math_FRPR
{
    protected math_Vector TheLocation;
    protected math_Vector TheGradient;
    protected double TheMinimum;
    protected double PreviousMinimum;
    protected double XTol;
    protected double EPSZ;

    private bool Done;
    private int Iter;
    private int State;
    private math_Status TheStatus;
    private int Itermax;

    // l'utilisation de math_BrentMinumim pur trouver un minimum dans une direction
    // donnee n'est pas du tout optimale. voir peut etre interpolation cubique
    // classique et aussi essayer "recherche unidimensionnelle economique"
    // PROGRAMMATION MATHEMATIQUE (theorie et algorithmes) tome1 page 82.

    // The function along a direction: F(P0 + x * Dir).
    private sealed class DirFunctionTer : math_Function
    {
        private readonly math_Vector P0;
        private readonly math_Vector Dir;
        private readonly math_Vector P;
        private readonly math_MultipleVarFunction F;

        public DirFunctionTer(math_Vector V1, math_Vector V2, math_Vector V3, math_MultipleVarFunction f)
        {
            P0 = V1;
            Dir = V2;
            P = V3;
            F = f;
        }

        public void Initialize(math_Vector p0, math_Vector dir)
        {
            P0.Initialized(p0);
            Dir.Initialized(dir);
        }

        public override bool Value(double x, out double fval)
        {
            P.Initialized(Dir);
            P.Multiply(x);
            P.Add(P0);
            fval = 0.0;
            return F.Value(P, out fval);
        }
    }

    private static bool MinimizeDirection(math_Vector P, math_Vector Dir, ref double Result, DirFunctionTer F)
    {
        double ax, xx, bx;

        F.Initialize(P, Dir);
        math_BracketMinimum Bracket = new(F, 0.0, 1.0);
        if (Bracket.IsDone())
        {
            Bracket.Values(out ax, out xx, out bx);
            math_BrentMinimum Sol = new(1.0e-10);
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

        return false;
    }

    /// <summary>Prepares the algorithm; <see cref="Perform"/> does the job.</summary>
    public math_FRPR(math_MultipleVarFunctionWithGradient theFunction, double theTolerance, int theNbIterations = 200, double theZEPS = 1.0e-12)
    {
        TheLocation = new math_Vector(1, theFunction.NbVariables());
        TheGradient = new math_Vector(1, theFunction.NbVariables());
        TheMinimum = 0.0;
        PreviousMinimum = 0.0;
        XTol = theTolerance;
        EPSZ = theZEPS;
        Done = false;
        Iter = 0;
        State = 0;
        TheStatus = math_NotBracketed;
        Itermax = theNbIterations;
    }

    /// <summary>
    /// Minimizes F from <paramref name="StartingPoint"/>. The solution F = Fi is found when
    /// 2.0 * abs(Fi - Fi-1) &lt;= Tolerance * (abs(Fi) + abs(Fi-1) + ZEPS).
    /// </summary>
    public void Perform(math_MultipleVarFunctionWithGradient F, math_Vector StartingPoint)
    {
        bool Good;
        int n = TheLocation.Length();
        int j, its;
        double gg, gam, dgg;

        math_Vector g = new(1, n), h = new(1, n);

        math_Vector Temp1 = new(1, n);
        math_Vector Temp2 = new(1, n);
        math_Vector Temp3 = new(1, n);
        DirFunctionTer F_Dir = new(Temp1, Temp2, Temp3, F);

        TheLocation.Initialized(StartingPoint);
        Good = F.Values(TheLocation, out PreviousMinimum, TheGradient);
        if (!Good)
        {
            Done = false;
            TheStatus = math_FunctionError;
            return;
        }

        g.Initialized(-TheGradient);
        h.Initialized(g);
        TheGradient.Initialized(g);

        for (its = 1; its <= Itermax; its++)
        {
            Iter = its;

            bool IsGood = MinimizeDirection(TheLocation, TheGradient, ref TheMinimum, F_Dir);
            if (IsSolutionReached(F))
            {
                Done = true;
                State = F.GetStateNumber();
                TheStatus = math_OK;
                return;
            }

            if (!IsGood)
            {
                Done = false;
                TheStatus = math_DirectionSearchError;
                return;
            }

            Good = F.Values(TheLocation, out PreviousMinimum, TheGradient);
            if (!Good)
            {
                Done = false;
                TheStatus = math_FunctionError;
                return;
            }

            dgg = 0.0;
            gg = 0.0;

            for (j = 1; j <= n; j++)
            {
                gg += g[j] * g[j];
                //	   dgg += TheGradient(j)*TheGradient(j);  //for Fletcher-Reeves
                dgg += (TheGradient[j] + g[j]) * TheGradient[j]; // for Polak-Ribiere
            }

            if (gg == 0.0)
            {
                // Unlikely. If gradient is exactly 0 then we are already done.
                Done = false;
                TheStatus = math_FunctionError;
                return;
            }

            gam = dgg / gg;
            g.Initialized(-TheGradient);
            TheGradient.Initialized(g + gam * h);
            h.Initialized(TheGradient);
        }

        Done = false;
        TheStatus = math_TooManyIterations;
    }

    /// <summary>
    /// True when 2.0 * abs(Fi - Fi-1) &lt;= Tolerance * (abs(Fi) + abs(Fi-1) + ZEPS). A sub-class can redefine it.
    /// </summary>
    public virtual bool IsSolutionReached(math_MultipleVarFunctionWithGradient theFunction)
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
        return Iter;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_FRPR ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Location Vector = " + TheLocation + "\n");
            o.Write(" Minimum value = " + TheMinimum.ToString(CultureInfo.InvariantCulture) + "\n");
            o.Write(" Number of iterations = " + Iter + "\n");
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
