// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_PolyFunc.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_PolyFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Geometry.Library;

/// <summary>
/// The derivative of a polynomial, as a function with its derivative: built from the coefficients of the
/// polynomial (constant term first), it evaluates the derivative polynomial and its own derivative.
/// </summary>
internal sealed class PolyFunc : FunctionWithDerivative
{
    private readonly math_Vector _coeffs;

    public PolyFunc(math_Vector coeffs)
    {
        _coeffs = new math_Vector(1, coeffs.Length() - 1);
        // Construct the derivative polynomial
        for (int ii = 1; ii <= _coeffs.Length(); ii++)
        {
            _coeffs[ii] = ii * coeffs[ii + 1];
        }
    }

    public override bool Value(double x, out double f)
    {
        Span<double> ff = stackalloc double[1];
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(x, 0, _coeffs.Length() - 1, 1, _coeffs.AsSpan(), ff);
        f = ff[0];
        return true;
    }

    public override bool Derivative(double x, out double d)
    {
        math_Vector aux = new(1, 2);
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(x, 1, _coeffs.Length() - 1, 1, _coeffs.AsSpan(), aux.AsSpan());
        d = aux[2];
        return true;
    }

    public override bool Values(double x, out double f, out double d)
    {
        math_Vector aux = new(1, 2);
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(x, 1, _coeffs.Length() - 1, 1, _coeffs.AsSpan(), aux.AsSpan());
        f = aux[1];
        d = aux[2];
        return true;
    }
}
