// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SimpleBound.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_SimpleBound.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A 3d curve as a boundary of a GeomFill_ConstrainedFilling, not attached to a surface (no tangency constraint).
/// The filling surface will be within Tol3d of it; Tolang is kept but not used.
/// </summary>
/// <remarks>
/// A working object (decision 806): it keeps the caller's adaptor, as OCCT keeps the handle, and never changes it.
/// <see cref="Reparametrize"/> sets a reparametrisation law, a Law_BSpFunc whose B-spline is frozen.
/// </remarks>
public class GeomFill_SimpleBound : GeomFill_Boundary
{
    private readonly CurveAdaptorBase myC3d;
    private Law_Function? myPar;

    public GeomFill_SimpleBound(CurveAdaptorBase Curve, double Tol3d, double Tolang)
        : base(Tol3d, Tolang)
    {
        myC3d = Curve;
    }

    public override Point3D Value(double U)
    {
        double x = U;
        if (myPar is not null)
        {
            x = myPar.Value(U);
        }
        return myC3d.Value(x);
    }

    public override void D1(double U, out Point3D P, out Vector3D V)
    {
        double x = U, dx = 1.0;
        if (myPar is not null)
        {
            myPar.D1(U, out x, out dx);
        }
        myC3d.D1(x, out P, out V);
        V.Multiply(dx);
    }

    public override void Reparametrize(double First, double Last, bool HasDF, bool HasDL, double DF, double DL, bool Rev)
    {
        Law_BSpline curve = Law.Reparametrize(myC3d, First, Last, HasDF, HasDL, DF, DL, Rev, 30);
        myPar = new Law_BSpFunc();
        ((Law_BSpFunc)myPar).SetCurve(curve);
    }

    public override void Bounds(out double First, out double Last)
    {
        if (myPar is not null)
        {
            myPar.Bounds(out First, out Last);
        }
        else
        {
            First = myC3d.FirstParameter();
            Last = myC3d.LastParameter();
        }
    }

    public override bool IsDegenerated() => false;
}
