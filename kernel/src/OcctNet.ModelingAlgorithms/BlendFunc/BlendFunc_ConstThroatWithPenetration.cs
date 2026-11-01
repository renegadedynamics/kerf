// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstThroatWithPenetration.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstThroatWithPenetration.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function of a chamfer of constant throat whose section is a right-angled triangle: the first surface is
/// virtually offset into the solid, the apex lies on its intersection with the second surface, the right angle is at
/// the top of the chamfer, and the leg from the apex to the top (the throat) is constant.
/// </summary>
public class BlendFunc_ConstThroatWithPenetration : BlendFunc_ConstThroat
{
    public BlendFunc_ConstThroatWithPenetration(Adaptor3d_Surface S1, Adaptor3d_Surface S2, Adaptor3d_Curve C)
        : base(S1, S2, C)
    {
    }

    public override bool IsSolution(math_Vector Sol, double Tol)
    {
        math_Vector secmember = new(1, 4), valsol = new(1, 4);
        math_Matrix gradsol = new(1, 4, 1, 4);

        Value(Sol, valsol);
        Derivatives(Sol, gradsol);

        tol = Tol;

        gp_Vec dnplan = new gp_Vec(), temp1 = new gp_Vec(), temp2 = new gp_Vec(), temp3 = new gp_Vec();

        if (Math.Abs(valsol[1]) <= Tol && Math.Abs(valsol[2]) <= Tol && Math.Abs(valsol[3]) <= Tol * Tol
            && Math.Abs(valsol[4]) <= Tol)
        {
            dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

            temp1.SetXYZ(pts1.XYZ() - ptgui.XYZ());
            temp2.SetXYZ(pts2.XYZ() - ptgui.XYZ());
            temp3.SetXYZ(pts2.XYZ() - pts1.XYZ());

            surf1.D1(Sol[1], Sol[2], out pts1, out d1u1, out d1v1);
            surf2.D1(Sol[3], Sol[4], out pts2, out d1u2, out d1v2);

            secmember[1] = nplan.Dot(d1gui) - dnplan.Dot(temp1);
            secmember[2] = nplan.Dot(d1gui) - dnplan.Dot(temp2);
            secmember[3] = 2.0 * d1gui.Dot(temp1);
            secmember[4] = d1gui.Dot(temp3);

            math_Gauss Resol = new(gradsol);
            if (Resol.IsDone())
            {
                Resol.Solve(secmember);
                tg1.SetLinearForm(secmember[1], d1u1, secmember[2], d1v1);
                tg2.SetLinearForm(secmember[3], d1u2, secmember[4], d1v2);
                tg12d.SetCoord(secmember[1], secmember[2]);
                tg22d.SetCoord(secmember[3], secmember[4]);
                istangent = false;
            }
            else
            {
                istangent = true;
            }

            distmin = Min(distmin, pts1.Distance(pts2));

            return true;
        }

        return false;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        surf1.D0(X[1], X[2], out pts1);
        surf2.D0(X[3], X[4], out pts2);

        F[1] = nplan.XYZ().Dot(pts1.XYZ()) + theD;
        F[2] = nplan.XYZ().Dot(pts2.XYZ()) + theD;

        gp_Vec vref = new gp_Vec(ptgui, pts1);

        F[3] = vref.SquareMagnitude() - Throat * Throat;

        gp_Vec vec12 = new gp_Vec(pts1, pts2);

        F[4] = vref.Dot(vec12);

        return true;
    }

    public override bool Derivatives(math_Vector X, math_Matrix D)
    {
        surf1.D1(X[1], X[2], out pts1, out d1u1, out d1v1);
        surf2.D1(X[3], X[4], out pts2, out d1u2, out d1v2);

        D[1, 1] = nplan.Dot(d1u1);
        D[1, 2] = nplan.Dot(d1v1);
        D[1, 3] = 0.0;
        D[1, 4] = 0.0;
        D[2, 1] = 0.0;
        D[2, 2] = 0.0;
        D[2, 3] = nplan.Dot(d1u2);
        D[2, 4] = nplan.Dot(d1v2);
        D[3, 1] = 2.0 * new gp_Vec(ptgui, pts1).Dot(d1u1);
        D[3, 2] = 2.0 * new gp_Vec(ptgui, pts1).Dot(d1v1);
        D[3, 3] = 0.0;
        D[3, 4] = 0.0;
        D[4, 1] = d1u1.Dot(new gp_Vec(pts1, pts2)) - new gp_Vec(ptgui, pts1).Dot(d1u1);
        D[4, 2] = d1v1.Dot(new gp_Vec(pts1, pts2)) - new gp_Vec(ptgui, pts1).Dot(d1v1);
        D[4, 3] = new gp_Vec(ptgui, pts1).Dot(d1u2);
        D[4, 4] = new gp_Vec(ptgui, pts1).Dot(d1v2);

        return true;
    }

    public override gp_Vec TangentOnS1()
    {
        if (istangent)
        {
            throw new Standard_DomainError("BlendFunc_ConstThroatWithPenetration::TangentOnS1");
        }

        return tg1;
    }

    public override gp_Vec TangentOnS2()
    {
        if (istangent)
        {
            throw new Standard_DomainError("BlendFunc_ConstThroatWithPenetration::TangentOnS2");
        }

        return tg2;
    }

    public override gp_Vec2d Tangent2dOnS1()
    {
        if (istangent)
        {
            throw new Standard_DomainError("BlendFunc_ConstThroatWithPenetration::Tangent2dOnS1");
        }

        return tg12d;
    }

    public override gp_Vec2d Tangent2dOnS2()
    {
        if (istangent)
        {
            throw new Standard_DomainError("BlendFunc_ConstThroatWithPenetration::Tangent2dOnS2");
        }

        return tg22d;
    }

    /// <summary>Not implemented: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double GetSectionSize()
    {
        throw new Standard_NotImplemented("BlendFunc_ConstThroatWithPenetration::GetSectionSize()");
    }
}
