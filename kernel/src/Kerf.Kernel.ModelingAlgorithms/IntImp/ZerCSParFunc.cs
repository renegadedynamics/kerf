// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ZerCSParFunc.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Implicit;

/// <summary>
/// The static tool over a 3D curve that the generic <c>IntImp_ZerCSParFunc</c> uses (OCCT's TheCurveTool macro:
/// IntCurveSurface_TheHCurveTool, HLRBRep_LineTool).
/// </summary>
internal interface IZerCSParCurveTool<TheCurve> : IImplicitCurveTool<TheCurve>
{
    static abstract Point3D Value(in TheCurve c, double u);

    static abstract void D1(in TheCurve c, double u, out Point3D p, out Vector3D v);
}

/// <summary>
/// The static tool over a parametric surface that the generic <c>IntImp_ZerCSParFunc</c> uses (OCCT's
/// ThePSurfaceTool macro: Adaptor3d_HSurfaceTool, HLRBRep_SurfaceTool).
/// </summary>
internal interface IZerCSParPSurfaceTool<ThePSurface> : IPSurfaceTool<ThePSurface>
{
    static abstract Point3D Value(in ThePSurface s, double u, double v);

    static abstract void D1(in ThePSurface s, double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V);
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
internal class ZerCSParFunc<ThePSurface, ThePSurfaceTool, TheCurve, TheCurveTool>
    : FunctionSetWithDerivatives, ICSFunction<ThePSurface, TheCurve>
    where ThePSurfaceTool : IZerCSParPSurfaceTool<ThePSurface>
    where TheCurveTool : IZerCSParCurveTool<TheCurve>
{
    private readonly ThePSurface _surface;
    private readonly TheCurve _curve;
    private Point3D _p;
    private double _f;

    public ZerCSParFunc(in ThePSurface s, in TheCurve c)
    {
        _surface = s;
        _curve = c;
        _p = new Point3D(0.0, 0.0, 0.0);
        _f = 0.0;
    }

    public override int NbVariables() => 3;

    public override int NbEquations() => 3;

    public override bool Value(math_Vector x, math_Vector f)
    {
        Point3D psurf = ThePSurfaceTool.Value(_surface, x[1], x[2]);
        Point3D pcurv = TheCurveTool.Value(_curve, x[3]);
        double f1, f2, f3;
        f[1] = f1 = psurf.X - pcurv.X;
        f[2] = f2 = psurf.Y - pcurv.Y;
        f[3] = f3 = psurf.Z - pcurv.Z;
        _f = f1 * f1 + f2 * f2 + f3 * f3;
        _p = new Point3D((psurf.XYZ() + pcurv.XYZ()) * 0.5);
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        ThePSurfaceTool.D1(_surface, x[1], x[2], out _, out Vector3D d1u, out Vector3D d1v);
        TheCurveTool.D1(_curve, x[3], out _, out Vector3D d1w);
        d[1, 1] = d1u.X;
        d[1, 2] = d1v.X;
        d[1, 3] = -d1w.X;
        d[2, 1] = d1u.Y;
        d[2, 2] = d1v.Y;
        d[2, 3] = -d1w.Y;
        d[3, 1] = d1u.Z;
        d[3, 2] = d1v.Z;
        d[3, 3] = -d1w.Z;
        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        ThePSurfaceTool.D1(_surface, x[1], x[2], out Point3D psurf, out Vector3D d1u, out Vector3D d1v);
        TheCurveTool.D1(_curve, x[3], out Point3D pcurv, out Vector3D d1w);
        d[1, 1] = d1u.X;
        d[1, 2] = d1v.X;
        d[1, 3] = -d1w.X;
        d[2, 1] = d1u.Y;
        d[2, 2] = d1v.Y;
        d[2, 3] = -d1w.Y;
        d[3, 1] = d1u.Z;
        d[3, 2] = d1v.Z;
        d[3, 3] = -d1w.Z;

        double f1, f2, f3;
        f[1] = f1 = psurf.X - pcurv.X;
        f[2] = f2 = psurf.Y - pcurv.Y;
        f[3] = f3 = psurf.Z - pcurv.Z;
        _f = f1 * f1 + f2 * f2 + f3 * f3;
        _p = new Point3D((psurf.XYZ() + pcurv.XYZ()) * 0.5);
        return true;
    }

    /// <summary>The middle of the surface and curve points of the last evaluation.</summary>
    public Point3D Point() => _p;

    /// <summary>The square distance between the surface and curve points of the last evaluation.</summary>
    public double Root() => _f;

    public ThePSurface AuxillarSurface() => _surface;

    public TheCurve AuxillarCurve() => _curve;
}
