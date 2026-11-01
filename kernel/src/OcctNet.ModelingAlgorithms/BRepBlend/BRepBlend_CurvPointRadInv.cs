// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_CurvPointRadInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_CurvPointRadInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The inverse of the restriction/restriction function at an extremity: the section plane of the first curve (the
/// guide) through a given point and a point of the second curve. The variables are the parameters on the two curves.
/// </summary>
public class BRepBlend_CurvPointRadInv : Blend_CurvPointFuncInv
{
    private readonly Adaptor3d_Curve curv1;
    private readonly Adaptor3d_Curve curv2;
    private gp_Pnt point;
#pragma warning disable CS0414 // set and never read, as in OCCT
    private int choix;
#pragma warning restore CS0414

    public BRepBlend_CurvPointRadInv(Adaptor3d_Curve C1, Adaptor3d_Curve C2)
    {
        curv1 = C1;
        curv2 = C2;
        choix = 0;
    }

    /// <summary>Sets the "quadrant" <paramref name="Choix"/> (unused by the equations, as in OCCT).</summary>
    public void Set(int Choix)
    {
        choix = Choix;
    }

    public override int NbEquations()
    {
        return 2;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        double theD;
        gp_Pnt ptcur1, ptcur2;
        gp_Vec d1cur1, d1cur2;
        gp_XYZ nplan;
        curv1.D1(X[1], out ptcur1, out d1cur1);
        nplan = d1cur1.Normalized().XYZ();
        theD = -(nplan.Dot(ptcur1.XYZ()));
        curv2.D1(X[2], out ptcur2, out d1cur2);
        F[1] = nplan.Dot(point.XYZ()) + theD;
        F[2] = nplan.Dot(ptcur2.XYZ()) + theD;
        return true;
    }

    public override bool Derivatives(math_Vector X, math_Matrix D)
    {
        gp_Pnt ptcur1, ptcur2;
        gp_Vec d1cur1, d2cur1, d1cur2, nplan, dnplan = new();
        double dtheD, normd1cur1, unsurnormd1cur1;

        curv1.D2(X[1], out ptcur1, out d1cur1, out d2cur1);

        normd1cur1 = d1cur1.Magnitude();
        unsurnormd1cur1 = 1.0 / normd1cur1;
        nplan = unsurnormd1cur1 * d1cur1;
        dnplan.SetLinearForm(-nplan.Dot(d2cur1), nplan, d2cur1);
        dnplan.Multiply(unsurnormd1cur1);
        dtheD = -nplan.XYZ().Dot(d1cur1.XYZ()) - dnplan.XYZ().Dot(ptcur1.XYZ());
        D[1, 1] = dnplan.XYZ().Dot(point.XYZ()) + dtheD;
        D[1, 2] = 0.0;
        curv2.D1(X[2], out ptcur2, out d1cur2);
        D[2, 1] = dnplan.XYZ().Dot(ptcur2.XYZ()) + dtheD;
        D[2, 2] = nplan.Dot(d1cur2);

        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, math_Matrix D)
    {
        Value(X, F);
        Derivatives(X, D);

        return true;
    }

    public override void Set(in gp_Pnt P)
    {
        point = P;
    }

    public override void GetTolerance(math_Vector Tolerance, double Tol)
    {
        Tolerance[1] = curv1.Resolution(Tol);
        Tolerance[2] = curv2.Resolution(Tol);
    }

    public override void GetBounds(math_Vector InfBound, math_Vector SupBound)
    {
        InfBound[1] = curv1.FirstParameter();
        SupBound[1] = curv1.LastParameter();
        InfBound[2] = curv2.FirstParameter();
        SupBound[2] = curv2.LastParameter();
    }

    public override bool IsSolution(math_Vector Sol, double Tol)
    {
        math_Vector valsol = new(1, 2);
        Value(Sol, valsol);
        return Math.Abs(valsol[1]) <= Tol && Math.Abs(valsol[2]) <= Tol;
    }
}
