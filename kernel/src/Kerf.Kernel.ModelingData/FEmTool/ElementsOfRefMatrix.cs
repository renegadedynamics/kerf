// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_ElementsOfRefMatrix.hxx
// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_ElementsOfRefMatrix.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Engine.Numerics.Polynomials;

namespace Kerf.Engine.Approximation.FiniteElements;

/// <summary>
/// The functions whose Gauss integrals are the elements of the reference matrix of the linear criteria
/// (tension, flexion, jerk): <c>Pi(u)' * Pj(u)'</c>, <c>Pi(u)'' * Pj(u)''</c> or <c>Pi(u)''' * Pj(u)'''</c> for
/// every i &lt;= j, where Pi is the i-th function of the Hermite-Jacobi base.
/// </summary>
internal class ElementsOfRefMatrix : FunctionSet
{
    private readonly HermitJacobi _base;
    private readonly int _derOrder;
    private readonly int _nbEquations;

    /// <summary>The products of the derivatives of order <paramref name="derOrder"/> (0 to 3) of the base.</summary>
    public ElementsOfRefMatrix(HermitJacobi theBase, int derOrder)
    {
        _base = theBase;
        if (derOrder < 0 || derOrder > 3)
        {
            throw new ConstructionErrorException("FEmTool_ElementsOfRefMatrix");
        }

        _derOrder = derOrder;
        _nbEquations = (_base.WorkDegree() + 2) * (_base.WorkDegree() + 1) / 2;
    }

    /// <summary>1.</summary>
    public override int NbVariables() => 1;

    public override int NbEquations() => _nbEquations;

    /// <summary>
    /// The values for the variable <paramref name="x"/>: only for i &lt;= j, in the order P0*P0, P0*P1, P0*P2, ...,
    /// P1*P1, P1*P2, ... (the upper triangle of the matrix {PiPj}).
    /// </summary>
    public override bool Value(math_Vector x, math_Vector f)
    {
        if (f.Length() < _nbEquations)
        {
            throw new OutOfRangeException("FEmTool_ElementsOfRefMatrix::Value");
        }

        double u = x[x.Lower()];
        NCollection_Array1<double> basis = new(0, _base.WorkDegree()), aux = new(0, _base.WorkDegree());

        switch (_derOrder)
        {
            case 0:
                _base.D0(u, basis);
                break;
            case 1:
                _base.D1(u, aux, basis);
                break;
            case 2:
                _base.D2(u, aux, aux, basis);
                break;
            case 3:
                _base.D3(u, aux, aux, aux, basis);
                break;
        }

        int i, j, ii = 0;
        for (i = 0; i <= _base.WorkDegree(); i++)
        {
            for (j = i; j <= _base.WorkDegree(); j++)
            {
                f[f.Lower() + ii] = basis[i] * basis[j];
                ii++;
            }
        }

        return true;
    }
}
