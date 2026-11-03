// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Corde.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Corde.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

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
    private readonly SurfaceAdaptorBase surf;
    private readonly CurveAdaptorBase guide;
    private Point3D pts;
    // OCCT's pt2d is never used: left out.
    private double dis;
    private double normtg;
    private double theD;
    private Point3D ptgui;
    private Vector3D nplan;
    private Vector3D d1gui;
    private Vector3D d2gui;
    private Vector3D tgs;
    private Vector2D tg2d;
    private bool istangent;

    public BlendFunc_Corde(SurfaceAdaptorBase S, CurveAdaptorBase CG)
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
        Vector3D vref = new Vector3D(ptgui, pts);

        F[2] = vref.SquareMagnitude() - dis * dis;

        return true;
    }

    //=======================================================================
    // function : Derivatives
    // purpose  : D = grad F(U,V)
    //=======================================================================
    public bool Derivatives(math_Vector X, Matrix D)
    {
        surf.D1(X[1], X[2], out pts, out Vector3D d1u, out Vector3D d1v);

        D[1, 1] = nplan.Dot(d1u);
        D[1, 2] = nplan.Dot(d1v);
        D[2, 1] = 2.0 * new Vector3D(ptgui, pts).Dot(d1u);
        D[2, 2] = 2.0 * new Vector3D(ptgui, pts).Dot(d1v);

        return true;
    }

    public Point3D PointOnS()
    {
        return pts;
    }

    public Point3D PointOnGuide()
    {
        return ptgui;
    }

    public Vector3D NPlan()
    {
        return nplan;
    }

    public bool IsTangencyPoint()
    {
        return istangent;
    }

    public Vector3D TangentOnS()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_Corde::TangentOnS");
        }

        return tgs;
    }

    public Vector2D Tangent2dOnS()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_Corde::Tangent2dOnS");
        }

        return tg2d;
    }

    /// <summary>The derivative of F in the guide's parameter at the solution <paramref name="Sol"/>.</summary>
    public void DerFguide(math_Vector Sol, ref Vector2D DerF)
    {
        surf.D1(Sol[1], Sol[2], out pts, out _, out _);
        Vector3D dnplan = new Vector3D();
        dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

        Vector3D temp = new Vector3D(pts.XYZ() - ptgui.XYZ());

        DerF.SetX(dnplan.Dot(temp) - nplan.Dot(d1gui));
        DerF.SetY(-2.0 * d1gui.Dot(temp));
    }

    public bool IsSolution(math_Vector Sol, double Tol)
    {
        math_Vector secmember = new(1, 2), valsol = new(1, 2);
        Matrix gradsol = new(1, 2, 1, 2);
        Vector3D dnplan = new Vector3D(), temp = new Vector3D();

        Value(Sol, valsol);
        Derivatives(Sol, gradsol);
        if (Math.Abs(valsol[1]) <= Tol && Math.Abs(valsol[2]) <= Tol * Tol)
        {
            surf.D1(Sol[1], Sol[2], out pts, out Vector3D d1u, out Vector3D d1v);
            dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

            temp.SetXYZ(pts.XYZ() - ptgui.XYZ());

            secmember[1] = nplan.Dot(d1gui) - dnplan.Dot(temp);
            secmember[2] = 2.0 * d1gui.Dot(temp);

            //  gradsol*der = secmember
            //  with  der(1) = dU/dW, der(2) = dU/dW, W is the guide parameter

            Gauss Resol = new(gradsol);
            if (Resol.IsDone)
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
