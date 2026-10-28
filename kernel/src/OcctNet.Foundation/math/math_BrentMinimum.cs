// Port of: src/FoundationClasses/TKMath/math/math_BrentMinimum.hxx
// Port of: src/FoundationClasses/TKMath/math/math_BrentMinimum.lxx
// Port of: src/FoundationClasses/TKMath/math/math_BrentMinimum.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>Brent's method to find a minimum of a function of a single variable, without its derivative.</summary>
public class math_BrentMinimum
{
    private const double CGOLD = 0.3819660; // 0.5*(3 - sqrt(5))

    protected double a;
    protected double b;
    protected double x;
    protected double fx;
    protected double fv;
    protected double fw;
    protected double XTol;
    protected double EPSZ;

    private bool Done;
    private int iter;
    private int Itermax;
    private bool myF;

    /// <summary>Prepares the algorithm; <see cref="Perform"/> does the job.</summary>
    public math_BrentMinimum(double theTolX, int theNbIterations = 100, double theZEPS = 1.0e-12)
    {
        a = 0.0;
        b = 0.0;
        x = 0.0;
        fx = 0.0;
        fv = 0.0;
        fw = 0.0;
        XTol = theTolX;
        EPSZ = theZEPS;
        Done = false;
        iter = 0;
        Itermax = theNbIterations;
        myF = false;
    }

    /// <summary>Prepares the algorithm when F(Bx) is known; <see cref="Perform"/> does the job.</summary>
    public math_BrentMinimum(double theTolX, double theFbx, int theNbIterations = 100, double theZEPS = 1.0e-12)
    {
        a = 0.0;
        b = 0.0;
        x = 0.0;
        fx = theFbx;
        fv = 0.0;
        fw = 0.0;
        XTol = theTolX;
        EPSZ = theZEPS;
        Done = false;
        iter = 0;
        Itermax = theNbIterations;
        myF = true;
    }

    private static void SHFT(ref double theA, ref double theB, ref double theC, double theD)
    {
        theA = theB;
        theB = theC;
        theC = theD;
    }

    /// <summary>
    /// Minimizes F from a bracketing triplet ax, bx, cx (bx between ax and cx, F(bx) less than F(ax) and F(cx)).
    /// The solution is found when abs(Xi - Xi-1) &lt;= TolX * abs(Xi) + ZEPS.
    /// </summary>
    public void Perform(math_Function F, double ax, double bx, double cx)
    {
        bool OK;
        double etemp, fu, p, q, r;
        double tol1, tol2, u, v, w, xm;
        double e = 0.0;
        double d = RealLast();

        a = ((ax < cx) ? ax : cx);
        b = ((ax > cx) ? ax : cx);
        x = w = v = bx;
        if (!myF)
        {
            OK = F.Value(x, out fx);
            if (!OK)
            {
                return;
            }
        }

        fw = fv = fx;
        for (iter = 1; iter <= Itermax; iter++)
        {
            xm = 0.5 * (a + b);
            tol1 = XTol * Math.Abs(x) + EPSZ;
            tol2 = 2.0 * tol1;
            if (IsSolutionReached(F))
            {
                Done = true;
                return;
            }

            if (Math.Abs(e) > tol1)
            {
                r = (x - w) * (fx - fv);
                q = (x - v) * (fx - fw);
                p = (x - v) * q - (x - w) * r;
                q = 2.0 * (q - r);
                if (q > 0.0)
                {
                    p = -p;
                }

                q = Math.Abs(q);
                etemp = e;
                e = d;
                if (Math.Abs(p) >= Math.Abs(0.5 * q * etemp) || p <= q * (a - x) || p >= q * (b - x))
                {
                    e = (x >= xm ? a - x : b - x);
                    d = CGOLD * e;
                }
                else
                {
                    d = p / q;
                    u = x + d;
                    if (u - a < tol2 || b - u < tol2)
                    {
                        d = Math.CopySign(tol1, xm - x);
                    }
                }
            }
            else
            {
                e = (x >= xm ? a - x : b - x);
                d = CGOLD * e;
            }

            u = (Math.Abs(d) >= tol1 ? x + d : x + Math.CopySign(tol1, d));
            OK = F.Value(u, out fu);
            if (!OK)
            {
                return;
            }

            if (fu <= fx)
            {
                if (u >= x)
                {
                    a = x;
                }
                else
                {
                    b = x;
                }

                SHFT(ref v, ref w, ref x, u);
                SHFT(ref fv, ref fw, ref fx, fu);
            }
            else
            {
                if (u < x)
                {
                    a = u;
                }
                else
                {
                    b = u;
                }

                if (fu <= fw || w == x)
                {
                    v = w;
                    w = u;
                    fv = fw;
                    fw = fu;
                }
                else if (fu <= fv || v == x || v == w)
                {
                    v = u;
                    fv = fu;
                }
            }
        }

        Done = false;
    }

    /// <summary>
    /// Called at the end of each iteration to check if the solution is found. A sub-class can redefine it to stop
    /// the iterations on its own test.
    /// </summary>
    public virtual bool IsSolutionReached(math_Function theFunction)
    {
        double TwoTol = 2.0 * (XTol * Math.Abs(x) + EPSZ);
        return (x <= (TwoTol + a)) && (x >= (b - TwoTol));
    }

    public bool IsDone() => Done;

    /// <summary>The location of the minimum. Throws <see cref="StdFail_NotDone"/> if it was not found.</summary>
    public double Location()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return x;
    }

    /// <summary>The value of the minimum. Throws <see cref="StdFail_NotDone"/> if it was not found.</summary>
    public double Minimum()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return fx;
    }

    /// <summary>The number of iterations done. Throws <see cref="StdFail_NotDone"/> if the minimum was not found.</summary>
    public int NbIterations()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return iter;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_BrentMinimum ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Location value = " + x.ToString(CultureInfo.InvariantCulture) + "\n");
            o.Write(" Minimum value = " + fx.ToString(CultureInfo.InvariantCulture) + "\n");
            o.Write(" Number of iterations = " + iter + "\n");
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
