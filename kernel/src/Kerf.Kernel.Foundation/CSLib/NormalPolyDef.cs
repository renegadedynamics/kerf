// Port of: src/FoundationClasses/TKMath/CSLib/CSLib_NormalPolyDef.hxx
// Port of: src/FoundationClasses/TKMath/CSLib/CSLib_NormalPolyDef.cxx


using Kerf.Engine.Numerics;

using Occt;

namespace Kerf.Engine.Geometry;

/// <summary>
/// The polynomial F(X) = Sum_{i=0}^{k0} C(k0,i) * cos^i(X) * sin^(k0-i)(X) * li(i), whose sign tells whether the
/// normal at a singular point of a surface is defined (used by <see cref="CSLib"/>).
/// </summary>
internal class NormalPolyDef : FunctionWithDerivative
{
    private readonly int                        _k0;    // Polynomial degree
    private readonly NCollection_Array1<double> _taBli; // Coefficients li(i) for i = 0..myK0

    /// <summary>Creates the polynomial of degree <paramref name="k0"/> with the coefficients <paramref name="li"/>(0..theK0).</summary>
    public NormalPolyDef(int k0, NCollection_Array1<double> li)
    {
        _k0    = k0;
        _taBli = new NCollection_Array1<double>(0, k0);
        for (int i = 0; i <= k0; ++i)
        {
            _taBli[i] = li[i];
        }
    }

    public override bool Value(double x, out double f)
    {
        f = 0.0;

        double cos = Math.Cos(x);
        double sin = Math.Sin(x);

        // At singular points (cos or sin near zero), return zero to avoid numerical instability.
        if (Math.Abs(cos) <= RealSmall() || Math.Abs(sin) <= RealSmall())
        {
            return true;
        }

        for (int i = 0; i <= _k0; ++i)
        {
            f += global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(_k0, i) * Math.Pow(cos, i) * Math.Pow(sin, _k0 - i) * _taBli[i];
        }

        return true;
    }

    public override bool Derivative(double x, out double d)
    {
        d = 0.0;

        double cos = Math.Cos(x);
        double sin = Math.Sin(x);

        // At singular points, return zero derivative.
        if (Math.Abs(cos) <= RealSmall() || Math.Abs(sin) <= RealSmall())
        {
            return true;
        }

        // dF/dX = Sum_{i=0}^{k0} C(k0,i) * cos^(i-1)(X) * sin^(k0-i-1)(X) * (k0*cos^2(X) - i) * li(i)
        for (int i = 0; i <= _k0; ++i)
        {
            d += global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(_k0, i) * Math.Pow(cos, i - 1) * Math.Pow(sin, _k0 - i - 1)
                    * (_k0 * cos * cos - i) * _taBli[i];
        }

        return true;
    }

    public override bool Values(double x, out double f, out double d)
    {
        f = 0.0;
        d = 0.0;

        double cos = Math.Cos(x);
        double sin = Math.Sin(x);

        // At singular points, return zeros.
        if (Math.Abs(cos) <= RealSmall() || Math.Abs(sin) <= RealSmall())
        {
            return true;
        }

        for (int i = 0; i <= _k0; ++i)
        {
            double binCoeff = global::Kerf.Engine.Numerics.Polynomials.PLib.Bin(_k0, i);
            double liCoeff  = _taBli[i];

            f += binCoeff * Math.Pow(cos, i) * Math.Pow(sin, _k0 - i) * liCoeff;
            d += binCoeff * Math.Pow(cos, i - 1) * Math.Pow(sin, _k0 - i - 1)
                    * (_k0 * cos * cos - i) * liCoeff;
        }

        return true;
    }
}
