// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_PlanFunc.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_PlanFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The function F(w) = V . (C(w) - P): the signed distance (times |V|) from the plane through P normal to V to the
/// point of the curve C of parameter w. Its root is where the curve crosses the plane.
/// </summary>
public class GeomFill_PlanFunc : FunctionWithDerivative
{
    private Xyz myPnt;
    private Xyz myVec;
    private Xyz V;
    private Point3D G;
    private readonly Adaptor3d_Curve myCurve;

    public GeomFill_PlanFunc(in Point3D theP, in Vector3D theV, Adaptor3d_Curve theC)
    {
        myCurve = theC;
        myPnt = theP.XYZ();
        myVec = theV.XYZ();
    }

    public override bool Value(double X, out double F)
    {
        myCurve.D0(X, out G);
        V.SetLinearForm(-1, myPnt, G.XYZ());
        F = myVec.Dot(V);
        return true;
    }

    public override bool Derivative(double X, out double D)
    {
        myCurve.D1(X, out G, out Vector3D dg);
        D = myVec.Dot(dg.XYZ());
        return true;
    }

    public override bool Values(double X, out double F, out double D)
    {
        myCurve.D1(X, out G, out Vector3D dg);
        V.SetLinearForm(-1, myPnt, G.XYZ());
        F = myVec.Dot(V);
        D = myVec.Dot(dg.XYZ());
        return true;
    }

    /// <summary>Does nothing (OCCT leaves it empty): the outputs are left as given.</summary>
    public void D2(double X, ref double F, ref double D1, ref double D2)
    {
    }

    /// <summary>The derivative DF of F with respect to the plane's parameter t, given dP/dt and dV/dt.</summary>
    public void DEDT(double X, in Vector3D DPnt, in Vector3D DVec, out double DFDT)
    {
        myCurve.D0(X, out G);
        V.SetLinearForm(-1, myPnt, G.XYZ());
        DFDT = DVec.Dot(new Vector3D(V)) - myVec.Dot(DPnt.XYZ());
    }

    /// <summary>The first derivative DFDT in t (OCCT computes only it; the second derivatives are left as given).</summary>
    public void D2E(double X, in Vector3D DP, in Vector3D D2P, in Vector3D DV, in Vector3D D2V, out double DFDT,
                    ref double D2FDT2, ref double D2FDTDX)
    {
        myCurve.D1(X, out G, out _);
        Xyz DVDT = new Xyz();
        V.SetLinearForm(-1, myPnt, G.XYZ());
        DVDT.SetLinearForm(-1, DP.XYZ(), G.XYZ());
        DFDT = DV.Dot(new Vector3D(V)) + myVec.Dot(DVDT);
    }
}
