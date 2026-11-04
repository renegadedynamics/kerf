// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheQuadCurvFuncOfTheQuadCurvExactHInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheQuadCurvFuncOfTheQuadCurvExactHInter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The implicit function of a quadric along a curve adaptor, F(w) = Q(C(w)), with its derivative, whose roots are
/// the intersections of the curve and the quadric.
/// </summary>
internal sealed class TheQuadCurvFuncOfTheQuadCurvExactHInter
    : FunctionWithDerivative,
      ITheQuadCurvFunc<TheQuadCurvFuncOfTheQuadCurvExactHInter, CurveAdaptorBase>
{
    private readonly SurfaceQuadric _quadric;
    private readonly CurveAdaptorBase _curve;

    /// <summary>The function of the quadric <paramref name="q"/> along the curve <paramref name="c"/> (kept by reference).</summary>
    public TheQuadCurvFuncOfTheQuadCurvExactHInter(in SurfaceQuadric q, CurveAdaptorBase c)
    {
        _quadric = q;
        _curve = c;
    }

    static TheQuadCurvFuncOfTheQuadCurvExactHInter ITheQuadCurvFunc<TheQuadCurvFuncOfTheQuadCurvExactHInter, CurveAdaptorBase>.Create(
        in SurfaceQuadric q, in CurveAdaptorBase c) => new(q, c);

    public override bool Value(double param, out double f)
    {
        f = _quadric.Distance(TheHCurveTool.Value(_curve, param));
        return (true);
    }

    public override bool Derivative(double param, out double d)
    {
        TheHCurveTool.D1(_curve, param, out Point3D p, out Vector3D t);
        d = t.Dot(_quadric.Gradient(p));
        return (true);
    }

    public override bool Values(double param, out double f, out double d)
    {
        TheHCurveTool.D1(_curve, param, out Point3D p, out Vector3D t);
        f = 0.0;
        Vector3D grad = new();
        _quadric.ValAndGrad(p, ref f, ref grad);
        d = t.Dot(grad);
        return (true);
    }
}
