// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_ElementsOfRefMatrix.hxx
// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_ElementsOfRefMatrix.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Engine.Numerics.Polynomials;

namespace Occt;

/// <summary>
/// The functions whose Gauss integrals are the elements of the reference matrix of the linear criteria
/// (tension, flexion, jerk): <c>Pi(u)' * Pj(u)'</c>, <c>Pi(u)'' * Pj(u)''</c> or <c>Pi(u)''' * Pj(u)'''</c> for
/// every i &lt;= j, where Pi is the i-th function of the Hermite-Jacobi base.
/// </summary>
public class FEmTool_ElementsOfRefMatrix : FunctionSet
{
    private readonly HermitJacobi myBase;
    private readonly int myDerOrder;
    private readonly int myNbEquations;

    /// <summary>The products of the derivatives of order <paramref name="DerOrder"/> (0 to 3) of the base.</summary>
    public FEmTool_ElementsOfRefMatrix(HermitJacobi TheBase, int DerOrder)
    {
        myBase = TheBase;
        if (DerOrder < 0 || DerOrder > 3)
        {
            throw new ConstructionErrorException("FEmTool_ElementsOfRefMatrix");
        }

        myDerOrder = DerOrder;
        myNbEquations = (myBase.WorkDegree() + 2) * (myBase.WorkDegree() + 1) / 2;
    }

    /// <summary>1.</summary>
    public override int NbVariables() => 1;

    public override int NbEquations() => myNbEquations;

    /// <summary>
    /// The values for the variable <paramref name="X"/>: only for i &lt;= j, in the order P0*P0, P0*P1, P0*P2, ...,
    /// P1*P1, P1*P2, ... (the upper triangle of the matrix {PiPj}).
    /// </summary>
    public override bool Value(math_Vector X, math_Vector F)
    {
        if (F.Length() < myNbEquations)
        {
            throw new OutOfRangeException("FEmTool_ElementsOfRefMatrix::Value");
        }

        double u = X[X.Lower()];
        NCollection_Array1<double> Basis = new(0, myBase.WorkDegree()), Aux = new(0, myBase.WorkDegree());

        switch (myDerOrder)
        {
            case 0:
                myBase.D0(u, Basis);
                break;
            case 1:
                myBase.D1(u, Aux, Basis);
                break;
            case 2:
                myBase.D2(u, Aux, Aux, Basis);
                break;
            case 3:
                myBase.D3(u, Aux, Aux, Aux, Basis);
                break;
        }

        int i, j, ii = 0;
        for (i = 0; i <= myBase.WorkDegree(); i++)
        {
            for (j = i; j <= myBase.WorkDegree(); j++)
            {
                F[F.Lower() + ii] = Basis[i] * Basis[j];
                ii++;
            }
        }

        return true;
    }
}
