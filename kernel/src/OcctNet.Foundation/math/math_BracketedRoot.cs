// Port of: src/FoundationClasses/TKMath/math/math_BracketedRoot.hxx
// Port of: src/FoundationClasses/TKMath/math/math_BracketedRoot.lxx
// Port of: src/FoundationClasses/TKMath/math/math_BracketedRoot.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>Brent's method to find a root of a function between two bounds, without its derivative.</summary>
public class math_BracketedRoot
{
    private bool Done;
    private double TheRoot;
    private double TheError;
    private int NbIter;

    // reference algorithm:
    //                   Brent method
    //                   numerical recipes in C (p. 269)

    /// <summary>
    /// Finds the root of <paramref name="F"/> between <paramref name="Bound1"/> and <paramref name="Bound2"/>; fails if
    /// F(Bound1) * F(Bound2) &gt; 0. The solution is found when abs(Xi - Xi-1) &lt;= Tolerance.
    /// </summary>
    public math_BracketedRoot(math_Function F, double Bound1, double Bound2, double Tolerance,
                              int NbIterations = 100, double ZEPS = 1.0e-12)
    {
        double Fa, Fc, a, c = 0, d = 0, e = 0;
        double min1, min2, p, q, r, s, tol1, xm;

        a = Bound1;
        TheRoot = Bound2;
        F.Value(a, out Fa);
        F.Value(TheRoot, out TheError);
        if (Fa * TheError > 0.0)
        {
            Done = false;
        }
        else
        {
            Fc = TheError;
            for (NbIter = 1; NbIter <= NbIterations; NbIter++)
            {
                if (TheError * Fc > 0.0)
                {
                    c = a; // rename a TheRoot c and adjust bounding interval d
                    Fc = Fa;
                    d = TheRoot - a;
                    e = d;
                }

                if (Math.Abs(Fc) < Math.Abs(Fa))
                {
                    a = TheRoot;
                    TheRoot = c;
                    c = a;
                    Fa = TheError;
                    TheError = Fc;
                    Fc = Fa;
                }

                tol1 = 2.0 * ZEPS * Math.Abs(TheRoot) + 0.5 * Tolerance; // convergence check
                xm = 0.5 * (c - TheRoot);
                if (Math.Abs(xm) <= tol1 || TheError == 0.0)
                {
                    Done = true;
                    return;
                }

                if (Math.Abs(e) >= tol1 && Math.Abs(Fa) > Math.Abs(TheError))
                {
                    s = TheError / Fa; // attempt inverse quadratic interpolation
                    if (a == c)
                    {
                        p = 2.0 * xm * s;
                        q = 1.0 - s;
                    }
                    else
                    {
                        q = Fa / Fc;
                        r = TheError / Fc;
                        p = s * (2.0 * xm * q * (q - r) - (TheRoot - a) * (r - 1.0));
                        q = (q - 1.0) * (r - 1.0) * (s - 1.0);
                    }

                    if (p > 0.0)
                    {
                        q = -q;
                    } // check whether in bounds

                    p = Math.Abs(p);
                    min1 = 3.0 * xm * q - Math.Abs(tol1 * q);
                    min2 = Math.Abs(e * q);
                    if (2.0 * p < (min1 < min2 ? min1 : min2))
                    {
                        e = d; // accept interpolation
                        d = p / q;
                    }
                    else
                    {
                        d = xm; // interpolation failed,use bissection
                        e = d;
                    }
                }
                else
                { // bounds decreasing too slowly ,use bissection
                    d = xm;
                    e = d;
                }

                a = TheRoot; // move last best guess to a
                Fa = TheError;
                if (Math.Abs(d) > tol1)
                { // evaluate new trial root
                    TheRoot += d;
                }
                else
                {
                    TheRoot += (xm > 0.0 ? Math.Abs(tol1) : -Math.Abs(tol1));
                }

                F.Value(TheRoot, out TheError);
            }

            Done = false;
        }
    }

    public bool IsDone() => Done;

    /// <summary>The root. Throws <see cref="StdFail_NotDone"/> if it was not found.</summary>
    public double Root()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return TheRoot;
    }

    /// <summary>The value of the function at the root. Throws <see cref="StdFail_NotDone"/> if it was not found.</summary>
    public double Value()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return TheError;
    }

    /// <summary>The number of iterations done. Throws <see cref="StdFail_NotDone"/> if the root was not found.</summary>
    public int NbIterations()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return NbIter;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_BracketedRoot ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Number of iterations = " + NbIter + '\n');
            o.Write(" The Root is: " + TheRoot.ToString(CultureInfo.InvariantCulture) + '\n');
            o.Write(" The value at the root is: " + TheError.ToString(CultureInfo.InvariantCulture) + '\n');
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
