// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_CurvPointRadInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_CurvPointRadInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// The inverse of the restriction/restriction function at an extremity: the section plane of the first curve (the
/// guide) through a given point and a point of the second curve. The variables are the parameters on the two curves.
/// </summary>
internal class CurvPointRadInv : CurvPointFuncInv
{
    private readonly CurveAdaptorBase _curv1;
    private readonly CurveAdaptorBase _curv2;
    private Point3D _point;
#pragma warning disable CS0414 // set and never read, as in OCCT
    private int _choix;
#pragma warning restore CS0414

    public CurvPointRadInv(CurveAdaptorBase c1, CurveAdaptorBase c2)
    {
        _curv1 = c1;
        _curv2 = c2;
        _choix = 0;
    }

    /// <summary>Sets the "quadrant" <paramref name="choix"/> (unused by the equations, as in OCCT).</summary>
    public void Set(int choix)
    {
        _choix = choix;
    }

    public override int NbEquations()
    {
        return 2;
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        double theD;
        Point3D ptcur1, ptcur2;
        Vector3D d1cur1, d1cur2;
        Xyz nplan;
        _curv1.D1(x[1], out ptcur1, out d1cur1);
        nplan = d1cur1.Normalized().XYZ();
        theD = -(nplan.Dot(ptcur1.XYZ()));
        _curv2.D1(x[2], out ptcur2, out d1cur2);
        f[1] = nplan.Dot(_point.XYZ()) + theD;
        f[2] = nplan.Dot(ptcur2.XYZ()) + theD;
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        Point3D ptcur1, ptcur2;
        Vector3D d1cur1, d2cur1, d1cur2, nplan, dnplan = new();
        double dtheD, normd1cur1, unsurnormd1cur1;

        _curv1.D2(x[1], out ptcur1, out d1cur1, out d2cur1);

        normd1cur1 = d1cur1.Magnitude;
        unsurnormd1cur1 = 1.0 / normd1cur1;
        nplan = unsurnormd1cur1 * d1cur1;
        dnplan = Vector3D.LinearForm(-nplan.Dot(d2cur1), nplan, d2cur1);
        dnplan = dnplan.Multiplied(unsurnormd1cur1);
        dtheD = -nplan.XYZ().Dot(d1cur1.XYZ()) - dnplan.XYZ().Dot(ptcur1.XYZ());
        d[1, 1] = dnplan.XYZ().Dot(_point.XYZ()) + dtheD;
        d[1, 2] = 0.0;
        _curv2.D1(x[2], out ptcur2, out d1cur2);
        d[2, 1] = dnplan.XYZ().Dot(ptcur2.XYZ()) + dtheD;
        d[2, 2] = nplan.Dot(d1cur2);

        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Value(x, f);
        Derivatives(x, d);

        return true;
    }

    public override void Set(in Point3D p)
    {
        _point = p;
    }

    public override void GetTolerance(math_Vector tolerance, double tol)
    {
        tolerance[1] = _curv1.Resolution(tol);
        tolerance[2] = _curv2.Resolution(tol);
    }

    public override void GetBounds(math_Vector infBound, math_Vector supBound)
    {
        infBound[1] = _curv1.FirstParameter();
        supBound[1] = _curv1.LastParameter();
        infBound[2] = _curv2.FirstParameter();
        supBound[2] = _curv2.LastParameter();
    }

    public override bool IsSolution(math_Vector sol, double tol)
    {
        math_Vector valsol = new(1, 2);
        Value(sol, valsol);
        return Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol;
    }
}
