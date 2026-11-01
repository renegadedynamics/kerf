// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Corde.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Corde.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The point pts of a surface on the plane normal to a guide curve at a chosen parameter, at a given distance from
/// the guide: X(1), X(2) are the parameters U, V of pts on the surface.
/// </summary>
/// <remarks>
/// OCCT holds it by value inside the chamfer functions; here it is a working class (decision 21).
/// </remarks>
public sealed class BlendFunc_Corde
{
    private readonly Adaptor3d_Surface surf;
    private readonly Adaptor3d_Curve guide;
    private gp_Pnt pts;
    // OCCT's pt2d is never used: left out.
    private double dis;
    private double normtg;
    private double theD;
    private gp_Pnt ptgui;
    private gp_Vec nplan;
    private gp_Vec d1gui;
    private gp_Vec d2gui;
    private gp_Vec tgs;
    private gp_Vec2d tg2d;
    private bool istangent;

    public BlendFunc_Corde(Adaptor3d_Surface S, Adaptor3d_Curve CG)
    {
        surf = S;
        guide = CG;
        dis = 0.0;
        normtg = 0.0;
        theD = 0.0;
        istangent = false;
    }

    public void SetDist(double Dist)
    {
        dis = Dist;
    }

    public void SetParam(double Param)
    {
        guide.D2(Param, out ptgui, out d1gui, out d2gui);
        normtg = d1gui.Magnitude();
        nplan = d1gui.Normalized();
        theD = -(nplan.XYZ().Dot(ptgui.XYZ()));
    }

    //=======================================================================
    // function : Value
    // purpose  : returns F(U,V)
    //=======================================================================
    public bool Value(math_Vector X, math_Vector F)
    {
        surf.D1(X[1], X[2], out pts, out _, out _);

        F[1] = nplan.XYZ().Dot(pts.XYZ()) + theD;
        gp_Vec vref = new gp_Vec(ptgui, pts);

        F[2] = vref.SquareMagnitude() - dis * dis;

        return true;
    }

    //=======================================================================
    // function : Derivatives
    // purpose  : D = grad F(U,V)
    //=======================================================================
    public bool Derivatives(math_Vector X, math_Matrix D)
    {
        surf.D1(X[1], X[2], out pts, out gp_Vec d1u, out gp_Vec d1v);

        D[1, 1] = nplan.Dot(d1u);
        D[1, 2] = nplan.Dot(d1v);
        D[2, 1] = 2.0 * new gp_Vec(ptgui, pts).Dot(d1u);
        D[2, 2] = 2.0 * new gp_Vec(ptgui, pts).Dot(d1v);

        return true;
    }

    public gp_Pnt PointOnS()
    {
        return pts;
    }

    public gp_Pnt PointOnGuide()
    {
        return ptgui;
    }

    public gp_Vec NPlan()
    {
        return nplan;
    }

    public bool IsTangencyPoint()
    {
        return istangent;
    }

    public gp_Vec TangentOnS()
    {
        if (istangent)
        {
            throw new Standard_DomainError("BlendFunc_Corde::TangentOnS");
        }

        return tgs;
    }

    public gp_Vec2d Tangent2dOnS()
    {
        if (istangent)
        {
            throw new Standard_DomainError("BlendFunc_Corde::Tangent2dOnS");
        }

        return tg2d;
    }

    /// <summary>The derivative of F in the guide's parameter at the solution <paramref name="Sol"/>.</summary>
    public void DerFguide(math_Vector Sol, ref gp_Vec2d DerF)
    {
        surf.D1(Sol[1], Sol[2], out pts, out _, out _);
        gp_Vec dnplan = new gp_Vec();
        dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

        gp_Vec temp = new gp_Vec(pts.XYZ() - ptgui.XYZ());

        DerF.SetX(dnplan.Dot(temp) - nplan.Dot(d1gui));
        DerF.SetY(-2.0 * d1gui.Dot(temp));
    }

    public bool IsSolution(math_Vector Sol, double Tol)
    {
        math_Vector secmember = new(1, 2), valsol = new(1, 2);
        math_Matrix gradsol = new(1, 2, 1, 2);
        gp_Vec dnplan = new gp_Vec(), temp = new gp_Vec();

        Value(Sol, valsol);
        Derivatives(Sol, gradsol);
        if (Math.Abs(valsol[1]) <= Tol && Math.Abs(valsol[2]) <= Tol * Tol)
        {
            surf.D1(Sol[1], Sol[2], out pts, out gp_Vec d1u, out gp_Vec d1v);
            dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

            temp.SetXYZ(pts.XYZ() - ptgui.XYZ());

            secmember[1] = nplan.Dot(d1gui) - dnplan.Dot(temp);
            secmember[2] = 2.0 * d1gui.Dot(temp);

            //  gradsol*der = secmember
            //  with  der(1) = dU/dW, der(2) = dU/dW, W is the guide parameter

            math_Gauss Resol = new(gradsol);
            if (Resol.IsDone())
            {
                Resol.Solve(secmember);
                tgs.SetLinearForm(secmember[1], d1u, secmember[2], d1v);
                tg2d.SetCoord(secmember[1], secmember[2]);
                istangent = false;
            }
            else
            {
                istangent = true;
            }

            return true;
        }

        return false;
    }
}
