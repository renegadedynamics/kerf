// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_CSFunction.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_CSFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The function whose zeros are the intersections of a curve on a surface S2 (a 2D curve C) with a surface S1:
/// F(u, v, w) = S1(u, v) - S2(C(w)), with its derivatives, for IntImp_IntCS (<see cref="CurvIntSurf"/>).
/// </summary>
/// <remarks>
/// OCCT keeps pointers to the caller's handles; the port keeps the adaptors (by reference, as the handles).
/// </remarks>
internal class PatchCSFunction : FunctionSetWithDerivatives, ICSFunction<SurfaceAdaptorBase, Curve2DAdaptorBase>
{
    private readonly Curve2DAdaptorBase _curve;
    private readonly SurfaceAdaptorBase _surface1;
    private readonly SurfaceAdaptorBase _surface2;
    private Point3D _p;
    private double _f;

    /// <summary><paramref name="s1"/> is the surface searched; <paramref name="c"/> a curve on <paramref name="s2"/>.</summary>
    public PatchCSFunction(SurfaceAdaptorBase s1, Curve2DAdaptorBase c, SurfaceAdaptorBase s2)
    {
        _surface1 = s1;
        _surface2 = s2;
        _curve = c;
        _f = 0.0;
        _p = new Point3D();
    }

    public override int NbVariables() => 3;

    public override int NbEquations() => 3;

    public override bool Value(math_Vector x, math_Vector f)
    {
        Point3D psurf = HSurfaceTool.Value(_surface1, x[1], x[2]);
        Point2D p2d = PatchHCurve2dTool.Value(_curve, x[3]);
        Point3D pcurv = HSurfaceTool.Value(_surface2, p2d.X, p2d.Y);

        f[1] = psurf.X - pcurv.X;
        f[2] = psurf.Y - pcurv.Y;
        f[3] = psurf.Z - pcurv.Z;
        _f = f[1] * f[1] + f[2] * f[2] + f[3] * f[3];
        _p = new Point3D((psurf.XYZ() + pcurv.XYZ()) / 2.0);
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        Vector3D d1w = new();

        HSurfaceTool.D1(_surface1, x[1], x[2], out _, out Vector3D D1u, out Vector3D D1v);
        PatchHCurve2dTool.D1(_curve, x[3], out Point2D p2d, out Vector2D d2d);
        HSurfaceTool.D1(_surface2, p2d.X, p2d.Y, out _, out Vector3D d1u, out Vector3D d1v);
        d1w = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);

        d[1, 1] = D1u.X;
        d[1, 2] = D1v.X;
        d[1, 3] = -d1w.X;
        d[2, 1] = D1u.Y;
        d[2, 2] = D1v.Y;
        d[2, 3] = -d1w.Y;
        d[3, 1] = D1u.Z;
        d[3, 2] = D1v.Z;
        d[3, 3] = -d1w.Z;
        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Vector3D d1w = new();

        HSurfaceTool.D1(_surface1, x[1], x[2], out Point3D psurf, out Vector3D D1u, out Vector3D D1v);
        PatchHCurve2dTool.D1(_curve, x[3], out Point2D p2d, out Vector2D d2d);
        HSurfaceTool.D1(_surface2, p2d.X, p2d.Y, out Point3D pcurv, out Vector3D d1u, out Vector3D d1v);
        d1w = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);

        d[1, 1] = D1u.X;
        d[1, 2] = D1v.X;
        d[1, 3] = -d1w.X;
        d[2, 1] = D1u.Y;
        d[2, 2] = D1v.Y;
        d[2, 3] = -d1w.Y;
        d[3, 1] = D1u.Z;
        d[3, 2] = D1v.Z;
        d[3, 3] = -d1w.Z;
        f[1] = psurf.X - pcurv.X;
        f[2] = psurf.Y - pcurv.Y;
        f[3] = psurf.Z - pcurv.Z;
        _f = f[1] * f[1] + f[2] * f[2] + f[3] * f[3];
        _p = new Point3D((psurf.XYZ() + pcurv.XYZ()) / 2.0);
        return true;
    }

    /// <summary>The middle of the two points of the last evaluation.</summary>
    public Point3D Point() => _p;

    /// <summary>The square distance of the last evaluation.</summary>
    public double Root() => _f;

    public SurfaceAdaptorBase AuxillarSurface() => _surface1;

    public Curve2DAdaptorBase AuxillarCurve() => _curve;
}
