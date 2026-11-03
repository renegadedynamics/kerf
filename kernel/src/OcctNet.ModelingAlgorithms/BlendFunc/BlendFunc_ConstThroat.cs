// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstThroat.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstThroat.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The function of a symmetric chamfer of constant throat: the height of the isosceles triangle in the section.
/// </summary>
public class BlendFunc_ConstThroat : BlendFunc_GenChamfer
{
    protected Point3D pts1;
    protected Point3D pts2;
    protected Vector3D d1u1;
    protected Vector3D d1v1;
    protected Vector3D d1u2;
    protected Vector3D d1v2;
    protected bool istangent;
    protected Vector3D tg1;
    protected Vector2D tg12d;
    protected Vector3D tg2;
    protected Vector2D tg22d;
    protected double param;
    protected double Throat;

    protected Point3D ptgui;
    protected Vector3D nplan;
    protected double normtg;
    protected double theD;
    protected Vector3D d1gui;
    protected Vector3D d2gui;

    public BlendFunc_ConstThroat(SurfaceAdaptorBase S1, SurfaceAdaptorBase S2, CurveAdaptorBase C)
        : base(S1, S2, C)
    {
        istangent = false;
        param = 0.0;
        Throat = 0.0;
        normtg = 0.0;
        theD = 0.0;
    }

    /// <summary>Sets the throat and the "quadrant"; the second distance is not used.</summary>
    public override void Set(double aThroat, double Dist2, int Choix)
    {
        Throat = aThroat;
        choix = Choix;
    }

    public override void Set(double Param)
    {
        param = Param;
        curv.D2(param, out ptgui, out d1gui, out d2gui);
        normtg = d1gui.Magnitude();
        nplan = d1gui.Normalized();
        theD = -(nplan.XYZ().Dot(ptgui.XYZ()));
    }

    public override bool IsSolution(math_Vector Sol, double Tol)
    {
        math_Vector secmember = new(1, 4), valsol = new(1, 4);
        Matrix gradsol = new(1, 4, 1, 4);

        Value(Sol, valsol);
        Derivatives(Sol, gradsol);

        tol = Tol;

        Vector3D dnplan = new Vector3D(), temp1 = new Vector3D(), temp2 = new Vector3D(), tempmid = new Vector3D();

        if (Math.Abs(valsol[1]) <= Tol && Math.Abs(valsol[2]) <= Tol && Math.Abs(valsol[3]) <= Tol * Tol
            && Math.Abs(valsol[4]) <= Tol * Tol)
        {
            dnplan.SetLinearForm(1.0 / normtg, d2gui, -1.0 / normtg * (nplan.Dot(d2gui)), nplan);

            temp1.SetXYZ(pts1.XYZ() - ptgui.XYZ());
            temp2.SetXYZ(pts2.XYZ() - ptgui.XYZ());
            tempmid.SetXYZ((pts1.XYZ() + pts2.XYZ()) / 2 - ptgui.XYZ());

            surf1.D1(Sol[1], Sol[2], out pts1, out d1u1, out d1v1);
            surf2.D1(Sol[3], Sol[4], out pts2, out d1u2, out d1v2);

            secmember[1] = nplan.Dot(d1gui) - dnplan.Dot(temp1);
            secmember[2] = nplan.Dot(d1gui) - dnplan.Dot(temp2);
            secmember[3] = 2.0 * d1gui.Dot(tempmid);
            secmember[4] = 2.0 * d1gui.Dot(temp2) - 2.0 * d1gui.Dot(temp1);

            Gauss Resol = new(gradsol);
            if (Resol.IsDone)
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

        Point3D ptmid = new Point3D((pts1.XYZ() + pts2.XYZ()) / 2);
        Vector3D vmid = new Vector3D(ptgui, ptmid);

        F[3] = vmid.SquareMagnitude() - Throat * Throat;

        Vector3D vref1 = new Vector3D(ptgui, pts1);
        Vector3D vref2 = new Vector3D(ptgui, pts2);

        F[4] = vref1.SquareMagnitude() - vref2.SquareMagnitude();

        return true;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
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
        D[3, 1] = new Vector3D((pts1.XYZ() + pts2.XYZ()) / 2 - ptgui.XYZ()).Dot(d1u1);
        D[3, 2] = new Vector3D((pts1.XYZ() + pts2.XYZ()) / 2 - ptgui.XYZ()).Dot(d1v1);
        D[3, 3] = new Vector3D((pts1.XYZ() + pts2.XYZ()) / 2 - ptgui.XYZ()).Dot(d1u2);
        D[3, 4] = new Vector3D((pts1.XYZ() + pts2.XYZ()) / 2 - ptgui.XYZ()).Dot(d1v2);
        D[4, 1] = 2.0 * new Vector3D(ptgui, pts1).Dot(d1u1);
        D[4, 2] = 2.0 * new Vector3D(ptgui, pts1).Dot(d1v1);
        D[4, 3] = -2.0 * new Vector3D(ptgui, pts2).Dot(d1u2);
        D[4, 4] = -2.0 * new Vector3D(ptgui, pts2).Dot(d1v2);

        return true;
    }

    public override Point3D PointOnS1()
    {
        return pts1;
    }

    public override Point3D PointOnS2()
    {
        return pts2;
    }

    public override bool IsTangencyPoint()
    {
        return istangent;
    }

    public override Vector3D TangentOnS1()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_ConstThroat::TangentOnS1");
        }

        return tg1;
    }

    public override Vector3D TangentOnS2()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_ConstThroat::TangentOnS2");
        }

        return tg2;
    }

    public override Vector2D Tangent2dOnS1()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_ConstThroat::Tangent2dOnS1");
        }

        return tg12d;
    }

    public override Vector2D Tangent2dOnS2()
    {
        if (istangent)
        {
            throw new DomainErrorException("BlendFunc_ConstThroat::Tangent2dOnS2");
        }

        return tg22d;
    }

    public override void Tangent(double U1, double V1, double U2, double V2, out Vector3D TgF, out Vector3D TgL,
                                 out Vector3D NmF, out Vector3D NmL)
    {
        bool revF = false;
        bool revL = false;

        surf1.D1(U1, V1, out _, out Vector3D d1u, out Vector3D d1v);
        NmF = d1u.Crossed(d1v);

        surf2.D1(U2, V2, out _, out d1u, out d1v);
        NmL = d1u.Crossed(d1v);

        TgF = (nplan.Crossed(NmF)).Normalized();
        TgL = (nplan.Crossed(NmL)).Normalized();

        if ((choix == 2) || (choix == 5))
        {
            revF = true;
            revL = true;
        }

        if ((choix == 4) || (choix == 7))
        {
            revL = true;
        }

        if ((choix == 3) || (choix == 8))
        {
            revF = true;
        }

        if (revF)
        {
            TgF.Reverse();
        }

        if (revL)
        {
            TgL.Reverse();
        }
    }

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double GetSectionSize()
    {
        throw new MethodNotImplementedException("BlendFunc_ConstThroat::GetSectionSize()");
    }
}
