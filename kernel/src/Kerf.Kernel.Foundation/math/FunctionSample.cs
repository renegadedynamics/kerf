// Port of: src/FoundationClasses/TKMath/math/math_FunctionSample.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionSample.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Numerics;

/// <summary>A default sample of a function defined between two bounds A and B: N equally spaced parameters.</summary>
internal class FunctionSample
{
    private double _a;
    private double _b;
    private int _n;

    public FunctionSample(double a, double b, int n)
    {
        _a = a;
        _b = b;
        _n = n;
    }

    /// <summary>The bounds of the parameters.</summary>
    public virtual void Bounds(out double a, out double b)
    {
        a = _a;
        b = _b;
    }

    /// <summary>The number of sample points.</summary>
    public int PointCount
    {
        get
        {
            return _n;
        }
    }

    /// <summary>
    /// The parameter of the point <paramref name="index"/>, in 1..NbPoints. Throws <see cref="OutOfRangeException"/>
    /// otherwise (OCCT compiles this check out of math_FunctionSample.cxx; the port keeps it).
    /// </summary>
    public virtual double GetParameter(int index)
    {
        Standard_OutOfRange_Raise_if((index <= 0) || (index > _n), " ");
        return ((_n - index) * _a + (index - 1) * _b) / (_n - 1);
    }
}
