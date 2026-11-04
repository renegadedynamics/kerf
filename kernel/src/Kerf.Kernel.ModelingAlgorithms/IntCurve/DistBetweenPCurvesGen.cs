// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_DistBetweenPCurvesGen.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>
/// The vector C1(u) - C2(v) between the points of two 2D curves, as a function of (u, v), with its Jacobian.
/// </summary>
/// <remarks>
/// OCCT's generic file IntCurve_DistBetweenPCurvesGen.gxx, ported once as a generic class (decision 324). Keeps the
/// curves as OCCT keeps pointers to them.
/// </remarks>
internal class DistBetweenPCurvesGen<TheCurve, TheCurveTool> : FunctionSetWithDerivatives
    where TheCurveTool : ICurveCurveTool<TheCurve>
{
    private readonly TheCurve _thecurve1;
    private readonly TheCurve _thecurve2;

    public DistBetweenPCurvesGen(in TheCurve c1, in TheCurve c2)
    {
        _thecurve1 = c1;
        _thecurve2 = c2;
    }

    public override int NbVariables() => (2);

    public override int NbEquations() => (2);

    public override bool Value(math_Vector x, math_Vector f)
    {
        TheCurveTool.D0(_thecurve1, x[1], out Point2D p1);
        TheCurveTool.D0(_thecurve2, x[2], out Point2D p2);
        f[1] = p1.X - p2.X;
        f[2] = p1.Y - p2.Y;
        return (true);
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        Vector2D t;
        TheCurveTool.D1(_thecurve1, x[1], out _, out t);
        d.Value(1, 1) = t.X;
        d.Value(2, 1) = t.Y;

        TheCurveTool.D1(_thecurve2, x[2], out _, out t);
        d.Value(1, 2) = -(t.X);
        d.Value(2, 2) = -(t.Y);

        return (true);
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Vector2D t;
        Point2D p1, p2;

        TheCurveTool.D1(_thecurve1, x[1], out p1, out t);
        d.Value(1, 1) = t.X;
        d.Value(2, 1) = t.Y;

        TheCurveTool.D1(_thecurve2, x[2], out p2, out t);
        d.Value(1, 2) = -(t.X);
        d.Value(2, 2) = -(t.Y);

        f.Value(1) = p1.X - p2.X;
        f.Value(2) = p1.Y - p2.Y;

        return (true);
    }
}
