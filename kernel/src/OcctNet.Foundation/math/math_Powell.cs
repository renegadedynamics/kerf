// Port of: src/FoundationClasses/TKMath/math/math_Powell.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Powell.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Powell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.math_Status;

namespace Occt;

/// <summary>Powell's method to find a minimum of a function of several variables, without its gradient.</summary>
public class math_Powell
{
    protected math_Vector TheLocation;
    protected double TheMinimum;
    protected double TheLocationError;
    protected double PreviousMinimum;
    protected double XTol;
    protected double EPSZ;

    private bool Done;
    private int Iter;
    private math_Status TheStatus;
    private math_Matrix TheDirections;
    private int State;
    private int Itermax;

    private static double SQR(double a) => a * a;

    // The function along a direction: F(P0 + x * Dir).
    private sealed class DirFunctionBis : math_Function
    {
        private readonly math_Vector P0;
        private readonly math_Vector Dir;
        private readonly math_Vector P;
        private readonly math_MultipleVarFunction F;

        public DirFunctionBis(math_Vector V1, math_Vector V2, math_Vector V3, math_MultipleVarFunction f)
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

    private static bool MinimizeDirection(math_Vector P, math_Vector Dir, ref double Result, DirFunctionBis F)
    {
        double ax;
        double xx;
        double bx;

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
    public math_Powell(math_MultipleVarFunction theFunction, double theTolerance, int theNbIterations = 200, double theZEPS = 1.0e-12)
    {
        TheLocation = new math_Vector(1, theFunction.NbVariables());
        TheMinimum = RealLast();
        TheLocationError = RealLast();
        PreviousMinimum = RealLast();
        XTol = theTolerance;
        EPSZ = theZEPS;
        Done = false;
        Iter = 0;
        TheStatus = math_NotBracketed;
        TheDirections = new math_Matrix(1, theFunction.NbVariables(), 1, theFunction.NbVariables());
        State = 0;
        Itermax = theNbIterations;
    }

    /// <summary>
    /// Minimizes F from <paramref name="StartingPoint"/>, the columns of <paramref name="StartingDirections"/> being
    /// the initial directions. The solution F = Fi is found when
    /// 2.0 * abs(Fi - Fi-1) &lt;= Tolerance * (abs(Fi) + abs(Fi-1) + ZEPS).
    /// </summary>
    public void Perform(math_MultipleVarFunction F, math_Vector StartingPoint, math_Matrix StartingDirections)
    {
        Done = false;
        int i, ibig, j;
        double t, fptt, del;
        int n = TheLocation.Length();
        math_Vector pt = new(1, n);
        math_Vector ptt = new(1, n);
        math_Vector xit = new(1, n);
        math_Vector Temp1 = new(1, n);
        math_Vector Temp2 = new(1, n);
        math_Vector Temp3 = new(1, n);
        DirFunctionBis F_Dir = new(Temp1, Temp2, Temp3, F);

        TheLocation.Initialized(StartingPoint);
        TheDirections.Initialized(StartingDirections);
        pt.Initialized(TheLocation); // sauvegarde du point initial

        for (Iter = 1; Iter <= Itermax; Iter++)
        {
            F.Value(TheLocation, out PreviousMinimum);
            ibig = 0;
            del = 0.0;
            for (i = 1; i <= n; i++)
            {
                for (j = 1; j <= n; j++)
                {
                    xit[j] = TheDirections[j, i];
                }

                F.Value(TheLocation, out fptt);
                bool IsGood = MinimizeDirection(TheLocation, xit, ref TheMinimum, F_Dir);

                if (!IsGood)
                {
                    Done = false;
                    TheStatus = math_DirectionSearchError;
                    return;
                }

                if (Math.Abs(fptt - TheMinimum) > del)
                {
                    del = Math.Abs(fptt - TheMinimum);
                    ibig = i;
                }
            }

            if (IsSolutionReached(F))
            {
                // Termination criterion
                State = F.GetStateNumber();
                Done = true;
                TheStatus = math_OK;
                return;
            }

            if (Iter == Itermax)
            {
                Done = false;
                TheStatus = math_TooManyIterations;
                return;
            }

            ptt.Initialized(2.0 * TheLocation - pt);
            xit.Initialized(TheLocation - pt);
            pt.Initialized(TheLocation);

            // Valeur de la fonction au point extrapole:
            F.Value(ptt, out fptt);

            if (fptt < PreviousMinimum)
            {
                t = 2.0 * (PreviousMinimum - 2.0 * TheMinimum + fptt) * SQR(PreviousMinimum - TheMinimum - del)
                    - del * SQR(PreviousMinimum - fptt);
                if (t < 0.0)
                {
                    // Minimisation along the direction
                    bool IsGood = MinimizeDirection(TheLocation, xit, ref TheMinimum, F_Dir);
                    if (!IsGood)
                    {
                        Done = false;
                        TheStatus = math_FunctionError;
                        return;
                    }

                    for (j = 1; j <= n; j++)
                    {
                        TheDirections[j, ibig] = xit[j];
                    }
                }
            }
        }
    }

    /// <summary>
    /// True when 2.0 * abs(Fi - Fi-1) &lt;= Tolerance * (abs(Fi) + abs(Fi-1) + ZEPS). A sub-class can redefine it.
    /// </summary>
    public virtual bool IsSolutionReached(math_MultipleVarFunction theFunction)
    {
        return 2.0 * Math.Abs(PreviousMinimum - TheMinimum) <= XTol * (Math.Abs(PreviousMinimum) + Math.Abs(TheMinimum) + EPSZ);
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

    /// <summary>The number of iterations done. Throws <see cref="StdFail_NotDone"/> if the minimum was not found.</summary>
    public int NbIterations()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Iter;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_Powell resolution:");
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
