// Port of: src/FoundationClasses/TKMath/math/math_BracketMinimum.hxx
// Port of: src/FoundationClasses/TKMath/math/math_BracketMinimum.lxx
// Port of: src/FoundationClasses/TKMath/math/math_BracketMinimum.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// From two distinct initial points, computes three points (a, b, c) which bracket a minimum of a function:
/// a &lt; b &lt; c, F(b) &lt; F(a) and F(b) &lt; F(c).
/// </summary>
/// <remarks>
/// By default the parameter is not limited; <see cref="SetLimits"/> defines the allowed range. If no minimum is found
/// within the limits, <see cref="IsDone"/> is false. The caller must give A and B within the limits.
/// </remarks>
public class math_BracketMinimum
{
    private const double GOLD = 1.618034;
    private const double GLIMIT = 100.0;
    private const double TINY = 1.0e-20;

    private bool Done;
    private double Ax;
    private double Bx;
    private double Cx;
    private double FAx;
    private double FBx;
    private double FCx;
    private double myLeft;
    private double myRight;
    private bool myIsLimited;
    private bool myFA;
    private bool myFB;

    /// <summary>Prepares A and B only; <see cref="Perform"/> does the job.</summary>
    public math_BracketMinimum(double A, double B)
    {
        Done = false;
        Ax = A;
        Bx = B;
        Cx = 0.0;
        FAx = 0.0;
        FBx = 0.0;
        FCx = 0.0;
        myLeft = -Precision.Infinite();
        myRight = Precision.Infinite();
        myIsLimited = false;
        myFA = false;
        myFB = false;
    }

    /// <summary>Computes a bracketing triplet from the initial values <paramref name="A"/> and <paramref name="B"/>.</summary>
    public math_BracketMinimum(math_Function F, double A, double B) : this(A, B)
    {
        Perform(F);
    }

    /// <summary>Computes a bracketing triplet when F(A) is known.</summary>
    public math_BracketMinimum(math_Function F, double A, double B, double FA) : this(A, B)
    {
        FAx = FA;
        myFA = true;
        Perform(F);
    }

    /// <summary>Computes a bracketing triplet when F(A) and F(B) are known.</summary>
    public math_BracketMinimum(math_Function F, double A, double B, double FA, double FB) : this(A, B)
    {
        FAx = FA;
        FBx = FB;
        myFA = true;
        myFB = true;
        Perform(F);
    }

    /// <summary>Limits the parameter to [theLeft, theRight].</summary>
    public void SetLimits(double theLeft, double theRight)
    {
        myLeft = theLeft;
        myRight = theRight;
        myIsLimited = true;
    }

    /// <summary>Sets the function value at A.</summary>
    public void SetFA(double theValue)
    {
        FAx = theValue;
        myFA = true;
    }

    /// <summary>Sets the function value at B.</summary>
    public void SetFB(double theValue)
    {
        FBx = theValue;
        myFB = true;
    }

    /// <summary>Limits the given value to the range [myLeft, myRight].</summary>
    private double Limited(double theValue)
    {
        return theValue < myLeft ? myLeft : theValue > myRight ? myRight : theValue;
    }

    /// <summary>
    /// Limits C and computes the function there. If C falls between A and B, swaps B and C with their values.
    /// False if C becomes equal to B or the function fails.
    /// </summary>
    private bool LimitAndMayBeSwap(math_Function F, double theA, ref double theB, ref double theFB, ref double theC, ref double theFC)
    {
        theC = Limited(theC);
        if (Math.Abs(theB - theC) < Precision.PConfusion())
        {
            return false;
        }

        bool OK = F.Value(theC, out theFC);
        if (!OK)
        {
            return false;
        }

        // check that B is between A and C
        if ((theA - theB) * (theB - theC) < 0)
        {
            // swap B and C: SHFT(dum, theB, theC, dum); SHFT(dum, theFB, theFC, dum);
            double dum;
            dum = theB;
            theB = theC;
            theC = dum;
            dum = theFB;
            theFB = theFC;
            theFC = dum;
        }

        return true;
    }

    /// <summary>Performs the job; the constructors taking a function call it.</summary>
    public void Perform(math_Function F)
    {
        bool OK;
        double ulim, u, r, q, fu, dum;

        Done = false;

        double Lambda = GOLD;
        if (!myFA)
        {
            OK = F.Value(Ax, out FAx);
            if (!OK)
            {
                return;
            }
        }

        if (!myFB)
        {
            OK = F.Value(Bx, out FBx);
            if (!OK)
            {
                return;
            }
        }

        if (FBx > FAx)
        {
            // SHFT(dum, Ax, Bx, dum); SHFT(dum, FBx, FAx, dum);
            dum = Ax;
            Ax = Bx;
            Bx = dum;
            dum = FBx;
            FBx = FAx;
            FAx = dum;
        }

        // get next prob after (A, B)
        Cx = Bx + Lambda * (Bx - Ax);
        if (myIsLimited)
        {
            OK = LimitAndMayBeSwap(F, Ax, ref Bx, ref FBx, ref Cx, ref FCx);
            if (!OK)
            {
                return;
            }
        }
        else
        {
            OK = F.Value(Cx, out FCx);
            if (!OK)
            {
                return;
            }
        }

        while (FBx > FCx)
        {
            r = (Bx - Ax) * (FBx - FCx);
            q = (Bx - Cx) * (FBx - FAx);
            u = Bx - ((Bx - Cx) * q - (Bx - Ax) * r) / (2.0 * SIGN(MAX(Math.Abs(q - r), TINY), q - r));
            ulim = Bx + GLIMIT * (Cx - Bx);
            if (myIsLimited)
            {
                ulim = Limited(ulim);
            }

            fu = 0.0; // C++ leaves fu uninitialised; every branch below sets it before use
            if ((Bx - u) * (u - Cx) > 0.0)
            {
                // u is between B and C
                OK = F.Value(u, out fu);
                if (!OK)
                {
                    return;
                }

                if (fu < FCx)
                {
                    // solution is found (B, u, c)
                    Ax = Bx;
                    Bx = u;
                    FAx = FBx;
                    FBx = fu;
                    Done = true;
                    return;
                }
                else if (fu > FBx)
                {
                    // solution is found (A, B, u)
                    Cx = u;
                    FCx = fu;
                    Done = true;
                    return;
                }

                // get next prob after (B, C)
                u = Cx + Lambda * (Cx - Bx);
                if (myIsLimited)
                {
                    OK = LimitAndMayBeSwap(F, Bx, ref Cx, ref FCx, ref u, ref fu);
                    if (!OK)
                    {
                        return;
                    }
                }
                else
                {
                    OK = F.Value(u, out fu);
                    if (!OK)
                    {
                        return;
                    }
                }
            }
            else if ((Cx - u) * (u - ulim) > 0.0)
            {
                // u is beyond C but between C and limit
                OK = F.Value(u, out fu);
                if (!OK)
                {
                    return;
                }
            }
            else if ((u - ulim) * (ulim - Cx) >= 0.0)
            {
                // u is beyond limit
                u = ulim;
                OK = F.Value(u, out fu);
                if (!OK)
                {
                    return;
                }
            }
            else
            {
                // u tends to approach to the side of A,
                // so reset it to the next prob after (B, C)
                u = Cx + GOLD * (Cx - Bx);
                if (myIsLimited)
                {
                    OK = LimitAndMayBeSwap(F, Bx, ref Cx, ref FCx, ref u, ref fu);
                    if (!OK)
                    {
                        return;
                    }
                }
                else
                {
                    OK = F.Value(u, out fu);
                    if (!OK)
                    {
                        return;
                    }
                }
            }

            // SHFT(Ax, Bx, Cx, u); SHFT(FAx, FBx, FCx, fu);
            Ax = Bx;
            Bx = Cx;
            Cx = u;
            FAx = FBx;
            FBx = FCx;
            FCx = fu;
        }

        Done = true;
    }

    public bool IsDone() => Done;

    /// <summary>The bracketing triplet of abscissae. Throws <see cref="StdFail_NotDone"/> if the algorithm failed.</summary>
    public void Values(out double A, out double B, out double C)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        A = Ax;
        B = Bx;
        C = Cx;
    }

    /// <summary>The function values at the triplet. Throws <see cref="StdFail_NotDone"/> if the algorithm failed.</summary>
    public void FunctionValues(out double FA, out double FB, out double FC)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        FA = FAx;
        FB = FBx;
        FC = FCx;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_BracketMinimum ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write(" The bracketed triplet is: " + '\n');
            o.Write(FormattableString.Invariant($"{Ax}, {Bx}, {Cx}") + '\n');
            o.Write(" The corresponding function values are: " + '\n');
            o.Write(FormattableString.Invariant($"{FAx}, {FBx}, {FCx}") + '\n');
        }
        else
        {
            o.Write(" Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }

    private static double MAX(double a, double b) => a > b ? a : b;

    private static double SIGN(double a, double b) => b > 0.0 ? Math.Abs(a) : -Math.Abs(a);
}
