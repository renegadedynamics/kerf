// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_PlanFunc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_PlanFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// The function F(w) = V . (C(w) - P): the signed distance (times |V|) from the plane through P normal to V to the
/// point of the curve C of parameter w. Its root is where the curve crosses the plane.
/// </summary>
public class PlanFunc : FunctionWithDerivative
{
    private Xyz _pnt;
    private Xyz _vec;
    private Xyz _v;
    private Point3D _g;
    private readonly CurveAdaptorBase _curve;

    public PlanFunc(in Point3D p, in Vector3D v, CurveAdaptorBase c)
    {
        _curve = c;
        _pnt = p.XYZ();
        _vec = v.XYZ();
    }

    public override bool Value(double x, out double f)
    {
        _curve.D0(x, out _g);
        _v.SetLinearForm(-1, _pnt, _g.XYZ());
        f = _vec.Dot(_v);
        return true;
    }

    public override bool Derivative(double x, out double d)
    {
        _curve.D1(x, out _g, out Vector3D dg);
        d = _vec.Dot(dg.XYZ());
        return true;
    }

    public override bool Values(double x, out double f, out double d)
    {
        _curve.D1(x, out _g, out Vector3D dg);
        _v.SetLinearForm(-1, _pnt, _g.XYZ());
        f = _vec.Dot(_v);
        d = _vec.Dot(dg.XYZ());
        return true;
    }

    /// <summary>Does nothing (OCCT leaves it empty): the outputs are left as given.</summary>
    public void D2(double x, ref double f, ref double d1, ref double d2)
    {
    }

    /// <summary>The derivative DF of F with respect to the plane's parameter t, given dP/dt and dV/dt.</summary>
    public void DEDT(double x, in Vector3D dPnt, in Vector3D dVec, out double dfdt)
    {
        _curve.D0(x, out _g);
        _v.SetLinearForm(-1, _pnt, _g.XYZ());
        dfdt = dVec.Dot(new Vector3D(_v)) - _vec.Dot(dPnt.XYZ());
    }

    /// <summary>The first derivative DFDT in t (OCCT computes only it; the second derivatives are left as given).</summary>
    public void D2E(double x, in Vector3D dp, in Vector3D d2P, in Vector3D dv, in Vector3D d2V, out double dfdt,
                    ref double d2FDT2, ref double d2FDTDX)
    {
        _curve.D1(x, out _g, out _);
        Xyz dvdt = new Xyz();
        _v.SetLinearForm(-1, _pnt, _g.XYZ());
        dvdt.SetLinearForm(-1, dp.XYZ(), _g.XYZ());
        dfdt = dv.Dot(new Vector3D(_v)) + _vec.Dot(dvdt);
    }
}
