// Port of: src/FoundationClasses/TKMath/math/math_NewtonFunctionRoot.hxx
// Port of: src/FoundationClasses/TKMath/math/math_NewtonFunctionRoot.lxx
// Port of: src/FoundationClasses/TKMath/math/math_NewtonFunctionRoot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// Newton's method to find a root of a function of a single variable from an initial guess near it. The derivative
/// is required.
/// </summary>
public class math_NewtonFunctionRoot
{
    private bool Done;
    private double X;
    private double Fx;
    private double DFx;
    private int It;
    private double EpsilonX;
    private double EpsilonF;
    private int Itermax;
    private double Binf;
    private double Bsup;

    /// <summary>
    /// Finds the root of <paramref name="F"/> from <paramref name="Guess"/>, inside [A, B]. The solution is found when
    /// abs(Xi - Xi-1) &lt;= EpsX and abs(F(Xi)) &lt;= EpsF.
    /// </summary>
    public math_NewtonFunctionRoot(math_FunctionWithDerivative F, double Guess, double EpsX, double EpsF,
                                   double A, double B, int NbIterations = 100)
    {
        EpsilonX = EpsX;
        EpsilonF = EpsF;
        Binf = A;
        Bsup = B;
        Itermax = NbIterations;
        Done = false;
        X = RealLast();
        DFx = 0;
        Fx = RealLast();
        It = 0;
        Perform(F, Guess);
    }

    /// <summary>Initializes the fields, for a sub-class; <see cref="Perform"/> does the job.</summary>
    public math_NewtonFunctionRoot(double A, double B, double EpsX, double EpsF, int NbIterations = 100)
    {
        Binf = A;
        Bsup = B;
        EpsilonX = EpsX;
        EpsilonF = EpsF;
        Itermax = NbIterations;
        Done = false;
        X = RealLast();
        DFx = 0;
        Fx = RealLast();
        It = 0;
    }

    /// <summary>
    /// Finds the root of <paramref name="F"/> from <paramref name="Guess"/>. The solution is found when
    /// abs(Xi - Xi-1) &lt;= EpsX and abs(F(Xi)) &lt;= EpsF.
    /// </summary>
    public math_NewtonFunctionRoot(math_FunctionWithDerivative F, double Guess, double EpsX, double EpsF, int NbIterations = 100)
    {
        EpsilonX = EpsX;
        EpsilonF = EpsF;
        Itermax = NbIterations;
        Binf = RealFirst();
        Bsup = RealLast();
        Done = false;
        X = RealLast();
        DFx = 0;
        Fx = RealLast();
        It = 0;
        Perform(F, Guess);
    }

    /// <summary>Performs the job; used by the constructors.</summary>
    public void Perform(math_FunctionWithDerivative F, double Guess)
    {
        double Dx;
        bool Ok;
        double AA, BB;

        //--------------------------------------------------
        //-- lbr le 12 Nov 97
        //-- la meilleure estimation n est pas sauvee et on
        //-- renvoie une solution plus fausse que Guess
        double BestX = X, BestFx = RealLast();
        //--

        if (Binf < Bsup)
        {
            AA = Binf;
            BB = Bsup;
        }
        else
        {
            AA = Bsup;
            BB = Binf;
        }

        Dx = RealLast();
        Fx = RealLast();
        X = Guess;
        It = 1;
        while ((It <= Itermax) && ((Math.Abs(Dx) > EpsilonX) || (Math.Abs(Fx) > EpsilonF)))
        {
            Ok = F.Values(X, out Fx, out DFx);

            double AbsFx = Fx;
            if (AbsFx < 0)
            {
                AbsFx = -AbsFx;
            }

            if (AbsFx < BestFx)
            {
                BestFx = AbsFx;
                BestX = X;
            }

            if (Ok)
            {
                if (DFx == 0.0)
                {
                    Done = false;
                    It = Itermax + 1;
                }
                else
                {
                    Dx = Fx / DFx;
                    X -= Dx;
                    // Limitation des variations de X:
                    if (X <= AA)
                    {
                        X = AA;
                    }

                    if (X >= BB)
                    {
                        X = BB;
                    }

                    It++;
                }
            }
            else
            {
                Done = false;
                It = Itermax + 1;
            }
        }

        X = BestX;

        Done = It <= Itermax;
    }

    public bool IsDone() => Done;

    /// <summary>The root. Throws <see cref="StdFail_NotDone"/> if it was not found.</summary>
    public double Root()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return X;
    }

    /// <summary>The derivative at the root. Throws <see cref="StdFail_NotDone"/> if it was not found.</summary>
    public double Derivative()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return DFx;
    }

    /// <summary>The value of the function at the root. Throws <see cref="StdFail_NotDone"/> if it was not found.</summary>
    public double Value()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Fx;
    }

    /// <summary>The number of iterations done. Throws <see cref="StdFail_NotDone"/> if the root was not found.</summary>
    public int NbIterations()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return It;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_NewtonFunctionRoot ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Location found = " + X.ToString(CultureInfo.InvariantCulture) + "\n");
            o.Write(" function value at this minimum = " + Fx.ToString(CultureInfo.InvariantCulture) + "\n");
            o.Write(" Number of iterations = " + It + "\n");
        }
        else
        {
            o.Write("Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
