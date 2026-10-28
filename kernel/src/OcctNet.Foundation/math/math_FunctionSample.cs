// Port of: src/FoundationClasses/TKMath/math/math_FunctionSample.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionSample.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A default sample of a function defined between two bounds A and B: N equally spaced parameters.</summary>
public class math_FunctionSample
{
    private double a;
    private double b;
    private int n;

    public math_FunctionSample(double A, double B, int N)
    {
        a = A;
        b = B;
        n = N;
    }

    /// <summary>The bounds of the parameters.</summary>
    public virtual void Bounds(out double A, out double B)
    {
        A = a;
        B = b;
    }

    /// <summary>The number of sample points.</summary>
    public int NbPoints()
    {
        return n;
    }

    /// <summary>
    /// The parameter of the point <paramref name="Index"/>, in 1..NbPoints. Throws <see cref="Standard_OutOfRange"/>
    /// otherwise (OCCT compiles this check out of math_FunctionSample.cxx; the port keeps it).
    /// </summary>
    public virtual double GetParameter(int Index)
    {
        Standard_OutOfRange_Raise_if((Index <= 0) || (Index > n), " ");
        return ((n - Index) * a + (Index - 1) * b) / (n - 1);
    }
}
