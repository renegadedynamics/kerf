// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ZerCSParFunc.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The static tool over a 3D curve that the generic <c>IntImp_ZerCSParFunc</c> uses (OCCT's TheCurveTool macro:
/// IntCurveSurface_TheHCurveTool, HLRBRep_LineTool).
/// </summary>
public interface IIntImp_ZerCSParCurveTool<TheCurve> : IIntImp_CurveTool<TheCurve>
{
    static abstract Point3D Value(in TheCurve C, double U);

    static abstract void D1(in TheCurve C, double U, out Point3D P, out Vector3D V);
}

/// <summary>
/// The static tool over a parametric surface that the generic <c>IntImp_ZerCSParFunc</c> uses (OCCT's
/// ThePSurfaceTool macro: Adaptor3d_HSurfaceTool, HLRBRep_SurfaceTool).
/// </summary>
public interface IIntImp_ZerCSParPSurfaceTool<ThePSurface> : IIntImp_PSurfaceTool<ThePSurface>
{
    static abstract Point3D Value(in ThePSurface S, double U, double V);

    static abstract void D1(in ThePSurface S, double U, double V, out Point3D P, out Vector3D D1U, out Vector3D D1V);
}

/// <summary>
/// The function whose zeros are the intersections of a curve C(w) and a parametric surface S(u, v):
/// F(u, v, w) = S(u, v) - C(w), with its derivatives, for math_FunctionSetRoot.
/// </summary>
/// <remarks>
/// OCCT's generic file IntImp_ZerCSParFunc.gxx, ported once as a generic class (decision 202's approach). OCCT's
/// instantiations (IntCurveSurface_TheCSFunctionOfHInter, and HLRBRep's later) are sealed subclasses. The function
/// keeps the curve and the surface it is given (adaptors: by reference, as OCCT's handles).
/// </remarks>
public class IntImp_ZerCSParFunc<ThePSurface, ThePSurfaceTool, TheCurve, TheCurveTool>
    : FunctionSetWithDerivatives, IIntImp_CSFunction<ThePSurface, TheCurve>
    where ThePSurfaceTool : IIntImp_ZerCSParPSurfaceTool<ThePSurface>
    where TheCurveTool : IIntImp_ZerCSParCurveTool<TheCurve>
{
    private readonly ThePSurface surface;
    private readonly TheCurve curve;
    private Point3D p;
    private double f;

    public IntImp_ZerCSParFunc(in ThePSurface S, in TheCurve C)
    {
        surface = S;
        curve = C;
        p = new Point3D(0.0, 0.0, 0.0);
        f = 0.0;
    }

    public override int NbVariables() => 3;

    public override int NbEquations() => 3;

    public override bool Value(math_Vector X, math_Vector F)
    {
        Point3D Psurf = ThePSurfaceTool.Value(surface, X[1], X[2]);
        Point3D Pcurv = TheCurveTool.Value(curve, X[3]);
        double f1, f2, f3;
        F[1] = f1 = Psurf.X() - Pcurv.X();
        F[2] = f2 = Psurf.Y() - Pcurv.Y();
        F[3] = f3 = Psurf.Z() - Pcurv.Z();
        f = f1 * f1 + f2 * f2 + f3 * f3;
        p = new Point3D((Psurf.XYZ() + Pcurv.XYZ()) * 0.5);
        return true;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        ThePSurfaceTool.D1(surface, X[1], X[2], out _, out Vector3D D1u, out Vector3D D1v);
        TheCurveTool.D1(curve, X[3], out _, out Vector3D D1w);
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
        ThePSurfaceTool.D1(surface, X[1], X[2], out Point3D Psurf, out Vector3D D1u, out Vector3D D1v);
        TheCurveTool.D1(curve, X[3], out Point3D Pcurv, out Vector3D D1w);
        D[1, 1] = D1u.X();
        D[1, 2] = D1v.X();
        D[1, 3] = -D1w.X();
        D[2, 1] = D1u.Y();
        D[2, 2] = D1v.Y();
        D[2, 3] = -D1w.Y();
        D[3, 1] = D1u.Z();
        D[3, 2] = D1v.Z();
        D[3, 3] = -D1w.Z();

        double f1, f2, f3;
        F[1] = f1 = Psurf.X() - Pcurv.X();
        F[2] = f2 = Psurf.Y() - Pcurv.Y();
        F[3] = f3 = Psurf.Z() - Pcurv.Z();
        f = f1 * f1 + f2 * f2 + f3 * f3;
        p = new Point3D((Psurf.XYZ() + Pcurv.XYZ()) * 0.5);
        return true;
    }

    /// <summary>The middle of the surface and curve points of the last evaluation.</summary>
    public Point3D Point() => p;

    /// <summary>The square distance between the surface and curve points of the last evaluation.</summary>
    public double Root() => f;

    public ThePSurface AuxillarSurface() => surface;

    public TheCurve AuxillarCurve() => curve;
}
