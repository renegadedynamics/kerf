// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_PolyFunc.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_PolyFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The derivative of a polynomial, as a function with its derivative: built from the coefficients of the
/// polynomial (constant term first), it evaluates the derivative polynomial and its own derivative.
/// </summary>
public sealed class GeomLib_PolyFunc : math_FunctionWithDerivative
{
    private readonly math_Vector myCoeffs;

    public GeomLib_PolyFunc(math_Vector Coeffs)
    {
        myCoeffs = new math_Vector(1, Coeffs.Length() - 1);
        // Construct the derivative polynomial
        for (int ii = 1; ii <= myCoeffs.Length(); ii++)
        {
            myCoeffs[ii] = ii * Coeffs[ii + 1];
        }
    }

    public override bool Value(double X, out double F)
    {
        Span<double> ff = stackalloc double[1];
        PLib.EvalPolynomial(X, 0, myCoeffs.Length() - 1, 1, myCoeffs.AsSpan(), ff);
        F = ff[0];
        return true;
    }

    public override bool Derivative(double X, out double D)
    {
        math_Vector Aux = new(1, 2);
        PLib.EvalPolynomial(X, 1, myCoeffs.Length() - 1, 1, myCoeffs.AsSpan(), Aux.AsSpan());
        D = Aux[2];
        return true;
    }

    public override bool Values(double X, out double F, out double D)
    {
        math_Vector Aux = new(1, 2);
        PLib.EvalPolynomial(X, 1, myCoeffs.Length() - 1, 1, myCoeffs.AsSpan(), Aux.AsSpan());
        F = Aux[1];
        D = Aux[2];
        return true;
    }
}
