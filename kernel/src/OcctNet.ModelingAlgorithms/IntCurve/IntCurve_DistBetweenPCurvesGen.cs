// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_DistBetweenPCurvesGen.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The vector C1(u) - C2(v) between the points of two 2D curves, as a function of (u, v), with its Jacobian.
/// </summary>
/// <remarks>
/// OCCT's generic file IntCurve_DistBetweenPCurvesGen.gxx, ported once as a generic class (decision 3504). Keeps the
/// curves as OCCT keeps pointers to them.
/// </remarks>
public class IntCurve_DistBetweenPCurvesGen<TheCurve, TheCurveTool> : math_FunctionSetWithDerivatives
    where TheCurveTool : IIntCurve_CurveTool<TheCurve>
{
    private readonly TheCurve thecurve1;
    private readonly TheCurve thecurve2;

    public IntCurve_DistBetweenPCurvesGen(in TheCurve C1, in TheCurve C2)
    {
        thecurve1 = C1;
        thecurve2 = C2;
    }

    public override int NbVariables() => (2);

    public override int NbEquations() => (2);

    public override bool Value(math_Vector X, math_Vector F)
    {
        TheCurveTool.D0(thecurve1, X[1], out gp_Pnt2d P1);
        TheCurveTool.D0(thecurve2, X[2], out gp_Pnt2d P2);
        F[1] = P1.X() - P2.X();
        F[2] = P1.Y() - P2.Y();
        return (true);
    }

    public override bool Derivatives(math_Vector X, math_Matrix D)
    {
        gp_Vec2d T;
        TheCurveTool.D1(thecurve1, X[1], out _, out T);
        D.Value(1, 1) = T.X();
        D.Value(2, 1) = T.Y();

        TheCurveTool.D1(thecurve2, X[2], out _, out T);
        D.Value(1, 2) = -(T.X());
        D.Value(2, 2) = -(T.Y());

        return (true);
    }

    public override bool Values(math_Vector X, math_Vector F, math_Matrix D)
    {
        gp_Vec2d T;
        gp_Pnt2d P1, P2;

        TheCurveTool.D1(thecurve1, X[1], out P1, out T);
        D.Value(1, 1) = T.X();
        D.Value(2, 1) = T.Y();

        TheCurveTool.D1(thecurve2, X[2], out P2, out T);
        D.Value(1, 2) = -(T.X());
        D.Value(2, 2) = -(T.Y());

        F.Value(1) = P1.X() - P2.X();
        F.Value(2) = P1.Y() - P2.Y();

        return (true);
    }
}
