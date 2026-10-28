// Port of: src/FoundationClasses/TKMath/math/math_BissecNewton.hxx
// Port of: src/FoundationClasses/TKMath/math/math_BissecNewton.lxx
// Port of: src/FoundationClasses/TKMath/math/math_BissecNewton.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.math_Status;

namespace Occt;

/// <summary>
/// A combination of the Newton-Raphson and bisection methods to find a root of a function between two bounds.
/// The derivative is required.
/// </summary>
public class math_BissecNewton
{
    protected math_Status TheStatus;
    protected double XTol;
    protected double x;
    protected double dx;
    protected double f;
    protected double df;

    private bool Done;

    /// <param name="theXTolerance">The tolerance of the algorithm.</param>
    public math_BissecNewton(double theXTolerance)
    {
        TheStatus = math_NotBracketed;
        XTol = theXTolerance;
        x = 0.0;
        dx = 0.0;
        f = 0.0;
        df = 0.0;
        Done = false;
    }

    /// <summary>
    /// Finds the root of <paramref name="F"/> between <paramref name="Bound1"/> and <paramref name="Bound2"/>. The
    /// solution is found when abs(Xi - Xi-1) &lt;= TolX and F(Xi) * F(Xi-1) &lt;= 0.
    /// </summary>
    public void Perform(math_FunctionWithDerivative F, double Bound1, double Bound2, int NbIterations = 100)
    {
        bool GOOD;
        int j;
        double dxold, fh, fl;
        double temp, xh, xl;

        GOOD = F.Values(Bound1, out fl, out df);
        if (!GOOD)
        {
            Done = false;
            TheStatus = math_FunctionError;
            return;
        }

        GOOD = F.Values(Bound2, out fh, out df);
        if (!GOOD)
        {
            Done = false;
            TheStatus = math_FunctionError;
            return;
        }

        //  Modified by Sergey KHROMOV - Wed Jan 22 12:06:45 2003 Begin
        double aFTol = RealEpsilon();
        //   if(fl * fh >= 0.0) {
        if (fl * fh > aFTol * aFTol)
        {
            Done = false;
            TheStatus = math_NotBracketed;
            return;
        }

        //   if(fl < 0.0) {
        if (fl < -aFTol || (fl < aFTol && fh < -aFTol))
        {
            xl = Bound1;
            xh = Bound2;
        }
        else
        {
            xl = Bound2;
            xh = Bound1;
        }
        //  Modified by Sergey KHROMOV - Wed Jan 22 12:06:49 2003 End

        x = 0.5 * (Bound1 + Bound2);
        dxold = Math.Abs(Bound2 - Bound1);
        dx = dxold;
        GOOD = F.Values(x, out f, out df);
        if (!GOOD)
        {
            Done = false;
            TheStatus = math_FunctionError;
            return;
        }

        for (j = 1; j <= NbIterations; j++)
        {
            if ((((x - xh) * df - f) * ((x - xl) * df - f) >= 0.0) || (Math.Abs(2.0 * f) > Math.Abs(dxold * df)))
            {
                dxold = dx;
                dx = 0.5 * (xh - xl);
                x = xl + dx;
                if (Math.Abs(dx) < XTol)
                {
                    TheStatus = math_OK;
                    Done = true;
                    return;
                }
            }
            else
            {
                dxold = dx;
                dx = f / df;
                temp = x;
                x -= dx;
                if (temp == x)
                {
                    TheStatus = math_OK;
                    Done = true;
                    return;
                }
            }

            if (IsSolutionReached(F))
            {
                TheStatus = math_OK;
                Done = true;
                return;
            }

            GOOD = F.Values(x, out f, out df);
            if (!GOOD)
            {
                Done = false;
                TheStatus = math_FunctionError;
                return;
            }

            if (f < 0.0)
            {
                xl = x;
            }
            else if (f > 0.0)
            {
                xh = x;
            }
            else
            {
                TheStatus = math_OK;
                Done = true;
                return;
            }
        }

        TheStatus = math_TooManyIterations;
        Done = false;
    }

    /// <summary>
    /// Called at the end of each iteration to check if the solution is found. A sub-class can redefine it to stop
    /// the iterations on its own test.
    /// </summary>
    public virtual bool IsSolutionReached(math_FunctionWithDerivative theFunction)
    {
        return Math.Abs(dx) <= XTol;
    }

    public bool IsDone() => Done;

    /// <summary>The root. Throws <see cref="StdFail_NotDone"/> if it was not found.</summary>
    public double Root()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return x;
    }

    /// <summary>The derivative at the root. Throws <see cref="StdFail_NotDone"/> if it was not found.</summary>
    public double Derivative()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return df;
    }

    /// <summary>The value of the function at the root. Throws <see cref="StdFail_NotDone"/> if it was not found.</summary>
    public double Value()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return f;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_BissecNewton ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write(" The Root  is: " + x.ToString(CultureInfo.InvariantCulture) + '\n');
            o.Write(" The value at this Root is: " + f.ToString(CultureInfo.InvariantCulture) + '\n');
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
}
