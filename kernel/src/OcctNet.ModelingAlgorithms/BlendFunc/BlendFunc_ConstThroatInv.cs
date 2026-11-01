// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstThroatInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstThroatInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The function of a chamfer of constant throat with one end on a surface's boundary.</summary>
public class BlendFunc_ConstThroatInv : BlendFunc_GenChamfInv
{
    protected double Throat;
    protected double param;
    protected double sign1;
    protected double sign2;

    protected gp_Pnt ptgui;
    protected gp_Vec nplan;
    protected double normtg;
    protected double theD;
    protected gp_Vec d1gui;
    protected gp_Vec d2gui;

    protected gp_Pnt pts1;
    protected gp_Pnt pts2;
    protected gp_Vec d1u1;
    protected gp_Vec d1v1;
    protected gp_Vec d1u2;
    protected gp_Vec d1v2;

    public BlendFunc_ConstThroatInv(Adaptor3d_Surface S1, Adaptor3d_Surface S2, Adaptor3d_Curve C)
        : base(S1, S2, C)
    {
        Throat = 0.0;
        param = 0.0;
        sign1 = 0.0;
        sign2 = 0.0;
        normtg = 0.0;
        theD = 0.0;
    }

    /// <summary>Sets the throat and the "quadrant"; the second distance is not used.</summary>
    public override void Set(double theThroat, double Dist2, int Choix)
    {
        Throat = theThroat;
        choix = Choix;
        switch (choix)
        {
            case 1:
            case 2:
                {
                    sign1 = -1;
                    sign2 = -1;
                }
                break;
            case 3:
            case 4:
                {
                    sign1 = 1;
                    sign2 = -1;
                }
                break;
            case 5:
            case 6:
                {
                    sign1 = 1;
                    sign2 = 1;
                }
                break;
            case 7:
            case 8:
                {
                    sign1 = -1;
                    sign2 = 1;
                }
                break;
            default:
                sign1 = -1;
                sign2 = -1;
                break;
        }
    }

    public override bool IsSolution(math_Vector Sol, double Tol)
    {
        math_Vector valsol = new(1, 4);
        Value(Sol, valsol);
        return Math.Abs(valsol[1]) <= Tol && Math.Abs(valsol[2]) <= Tol
               && Math.Abs(valsol[3]) <= Tol * Tol && Math.Abs(valsol[4]) <= Tol * Tol;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        csurf!.D1(X[1], out gp_Pnt2d p2d, out _);

        param = X[2];
        curv.D2(param, out ptgui, out d1gui, out d2gui);
        normtg = d1gui.Magnitude();
        nplan = d1gui.Normalized();
        theD = -(nplan.XYZ().Dot(ptgui.XYZ()));

        math_Vector XX = new(1, 4);
        if (first)
        {
            XX[1] = p2d.X();
            XX[2] = p2d.Y();
            XX[3] = X[3];
            XX[4] = X[4];
        }
        else
        {
            XX[1] = X[3];
            XX[2] = X[4];
            XX[3] = p2d.X();
            XX[4] = p2d.Y();
        }

        surf1.D0(XX[1], XX[2], out pts1);
        surf2.D0(XX[3], XX[4], out pts2);

        F[1] = nplan.XYZ().Dot(pts1.XYZ()) + theD;
        F[2] = nplan.XYZ().Dot(pts2.XYZ()) + theD;

        gp_Pnt ptmid = new gp_Pnt((pts1.XYZ() + pts2.XYZ()) / 2);
        gp_Vec vmid = new gp_Vec(ptgui, ptmid);

        F[3] = vmid.SquareMagnitude() - Throat * Throat;

        gp_Vec vref1 = new gp_Vec(ptgui, pts1);
        gp_Vec vref2 = new gp_Vec(ptgui, pts2);

        F[4] = vref1.SquareMagnitude() - vref2.SquareMagnitude();

        return true;
    }

    public override bool Derivatives(math_Vector X, math_Matrix D)
    {
        gp_Vec dnplan = new gp_Vec(), temp = new gp_Vec(), temp1 = new gp_Vec(), temp2 = new gp_Vec(),
               tempmid = new gp_Vec();
        math_Vector XX = new(1, 4);

        csurf!.D1(X[1], out gp_Pnt2d p2d, out gp_Vec2d v2d);

        param = X[2];
        curv.D2(param, out ptgui, out d1gui, out d2gui);
        normtg = d1gui.Magnitude();
        nplan = d1gui.Normalized();
        theD = -(nplan.XYZ().Dot(ptgui.XYZ()));

        dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

        // OCCT: temp1, temp2 and tempmid use the points of the previous call, before the D1 below (decision 1040).
        temp1.SetXYZ(pts1.XYZ() - ptgui.XYZ());
        temp2.SetXYZ(pts2.XYZ() - ptgui.XYZ());
        tempmid.SetXYZ((pts1.XYZ() + pts2.XYZ()) / 2 - ptgui.XYZ());

        if (first)
        {
            XX[1] = p2d.X();
            XX[2] = p2d.Y();
            XX[3] = X[3];
            XX[4] = X[4];
        }
        else
        {
            XX[1] = X[3];
            XX[2] = X[4];
            XX[3] = p2d.X();
            XX[4] = p2d.Y();
        }

        surf1.D1(XX[1], XX[2], out pts1, out d1u1, out d1v1);
        surf2.D1(XX[3], XX[4], out pts2, out d1u2, out d1v2);

        if (first)
        {
            // p2d = pts est sur surf1
            temp.SetLinearForm(v2d.X(), d1u1, v2d.Y(), d1v1);
            D[1, 1] = nplan.Dot(temp);
            D[2, 1] = 0.0;
            D[3, 1] = new gp_Vec(ptgui, pts1).Dot(temp);
            D[4, 1] = 2 * (new gp_Vec(ptgui, pts1).Dot(temp));

            D[1, 3] = 0.0;
            D[1, 4] = 0.0;
            D[2, 3] = nplan.Dot(d1u2);
            D[2, 4] = nplan.Dot(d1v2);
            D[3, 3] = new gp_Vec((pts1.XYZ() + pts2.XYZ()) / 2 - ptgui.XYZ()).Dot(d1u2);
            D[3, 4] = new gp_Vec((pts1.XYZ() + pts2.XYZ()) / 2 - ptgui.XYZ()).Dot(d1v2);
            D[4, 3] = -2.0 * new gp_Vec(ptgui, pts2).Dot(d1u2);
            D[4, 4] = -2.0 * new gp_Vec(ptgui, pts2).Dot(d1v2);
        }
        else
        {
            //  p2d = pts est sur surf2
            temp.SetLinearForm(v2d.X(), d1u2, v2d.Y(), d1v2);
            D[1, 1] = 0.0;
            D[2, 1] = nplan.Dot(temp);
            D[3, 1] = new gp_Vec(ptgui, pts2).Dot(temp);
            D[4, 1] = -2 * (new gp_Vec(ptgui, pts2).Dot(temp));

            D[1, 3] = nplan.Dot(d1u1);
            D[1, 4] = nplan.Dot(d1v1);
            D[2, 3] = 0.0;
            D[2, 4] = 0.0;
            D[3, 3] = new gp_Vec((pts1.XYZ() + pts2.XYZ()) / 2 - ptgui.XYZ()).Dot(d1u1);
            D[3, 4] = new gp_Vec((pts1.XYZ() + pts2.XYZ()) / 2 - ptgui.XYZ()).Dot(d1v1);
            D[4, 3] = 2.0 * new gp_Vec(ptgui, pts1).Dot(d1u1);
            D[4, 4] = 2.0 * new gp_Vec(ptgui, pts1).Dot(d1v1);
        }

        D[1, 2] = dnplan.Dot(temp1) - nplan.Dot(d1gui);
        D[2, 2] = dnplan.Dot(temp2) - nplan.Dot(d1gui);
        D[3, 2] = -2.0 * d1gui.Dot(tempmid);
        D[4, 2] = 2.0 * d1gui.Dot(temp1) - 2.0 * d1gui.Dot(temp2);

        return true;
    }
}
