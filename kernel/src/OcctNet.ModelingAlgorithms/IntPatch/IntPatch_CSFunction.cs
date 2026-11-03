// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_CSFunction.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_CSFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The function whose zeros are the intersections of a curve on a surface S2 (a 2D curve C) with a surface S1:
/// F(u, v, w) = S1(u, v) - S2(C(w)), with its derivatives, for IntImp_IntCS (<see cref="IntPatch_CurvIntSurf"/>).
/// </summary>
/// <remarks>
/// OCCT keeps pointers to the caller's handles; the port keeps the adaptors (by reference, as the handles).
/// </remarks>
public class IntPatch_CSFunction : FunctionSetWithDerivatives, IIntImp_CSFunction<Adaptor3d_Surface, Adaptor2d_Curve2d>
{
    private readonly Adaptor2d_Curve2d curve;
    private readonly Adaptor3d_Surface surface1;
    private readonly Adaptor3d_Surface surface2;
    private Point3D p;
    private double f;

    /// <summary><paramref name="S1"/> is the surface searched; <paramref name="C"/> a curve on <paramref name="S2"/>.</summary>
    public IntPatch_CSFunction(Adaptor3d_Surface S1, Adaptor2d_Curve2d C, Adaptor3d_Surface S2)
    {
        surface1 = S1;
        surface2 = S2;
        curve = C;
        f = 0.0;
        p = new Point3D();
    }

    public override int NbVariables() => 3;

    public override int NbEquations() => 3;

    public override bool Value(math_Vector X, math_Vector F)
    {
        Point3D Psurf = Adaptor3d_HSurfaceTool.Value(surface1, X[1], X[2]);
        Point2D p2d = IntPatch_HCurve2dTool.Value(curve, X[3]);
        Point3D Pcurv = Adaptor3d_HSurfaceTool.Value(surface2, p2d.X(), p2d.Y());

        F[1] = Psurf.X() - Pcurv.X();
        F[2] = Psurf.Y() - Pcurv.Y();
        F[3] = Psurf.Z() - Pcurv.Z();
        f = F[1] * F[1] + F[2] * F[2] + F[3] * F[3];
        p = new Point3D((Psurf.XYZ() + Pcurv.XYZ()) / 2.0);
        return true;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        Vector3D D1w = new();

        Adaptor3d_HSurfaceTool.D1(surface1, X[1], X[2], out _, out Vector3D D1u, out Vector3D D1v);
        IntPatch_HCurve2dTool.D1(curve, X[3], out Point2D p2d, out Vector2D d2d);
        Adaptor3d_HSurfaceTool.D1(surface2, p2d.X(), p2d.Y(), out _, out Vector3D d1u, out Vector3D d1v);
        D1w.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);

        D[1, 1] = D1u.X();
        D[1, 2] = D1v.X();
        D[1, 3] = -D1w.X();
        D[2, 1] = D1u.Y();
        D[2, 2] = D1v.Y();
        D[2, 3] = -D1w.Y();
        D[3, 1] = D1u.Z();
        D[3, 2] = D1v.Z();
        D[3, 3] = -D1w.Z();
        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        Vector3D D1w = new();

        Adaptor3d_HSurfaceTool.D1(surface1, X[1], X[2], out Point3D Psurf, out Vector3D D1u, out Vector3D D1v);
        IntPatch_HCurve2dTool.D1(curve, X[3], out Point2D p2d, out Vector2D d2d);
        Adaptor3d_HSurfaceTool.D1(surface2, p2d.X(), p2d.Y(), out Point3D Pcurv, out Vector3D d1u, out Vector3D d1v);
        D1w.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);

        D[1, 1] = D1u.X();
        D[1, 2] = D1v.X();
        D[1, 3] = -D1w.X();
        D[2, 1] = D1u.Y();
        D[2, 2] = D1v.Y();
        D[2, 3] = -D1w.Y();
        D[3, 1] = D1u.Z();
        D[3, 2] = D1v.Z();
        D[3, 3] = -D1w.Z();
        F[1] = Psurf.X() - Pcurv.X();
        F[2] = Psurf.Y() - Pcurv.Y();
        F[3] = Psurf.Z() - Pcurv.Z();
        f = F[1] * F[1] + F[2] * F[2] + F[3] * F[3];
        p = new Point3D((Psurf.XYZ() + Pcurv.XYZ()) / 2.0);
        return true;
    }

    /// <summary>The middle of the two points of the last evaluation.</summary>
    public Point3D Point() => p;

    /// <summary>The square distance of the last evaluation.</summary>
    public double Root() => f;

    public Adaptor3d_Surface AuxillarSurface() => surface1;

    public Adaptor2d_Curve2d AuxillarCurve() => curve;
}
