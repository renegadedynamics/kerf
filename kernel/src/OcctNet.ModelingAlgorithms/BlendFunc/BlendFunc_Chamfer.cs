// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Chamfer.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Chamfer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Modified : 20/08/96 PMN Ajout des methodes (Nb)Intervals et IsRationnal
// Modified : 30/12/96 PMN Ajout GetMinimalWeight, GetSectionSize;

namespace Occt;

/// <summary>The function of an "ordinary" chamfer: constant distances from the spine to the two surfaces.</summary>
public class BlendFunc_Chamfer : BlendFunc_GenChamfer
{
    private readonly BlendFunc_Corde corde1;
    private readonly BlendFunc_Corde corde2;

    public BlendFunc_Chamfer(Adaptor3d_Surface S1, Adaptor3d_Surface S2, Adaptor3d_Curve CG)
        : base(S1, S2, CG)
    {
        corde1 = new BlendFunc_Corde(S1, CG);
        corde2 = new BlendFunc_Corde(S2, CG);
    }

    public override void Set(double Dist1, double Dist2, int Choix)
    {
        corde1.SetDist(Dist1);
        corde2.SetDist(Dist2);
        choix = Choix;
    }

    public override void Set(double Param)
    {
        corde1.SetParam(Param);
        corde2.SetParam(Param);
    }

    public override bool IsSolution(math_Vector Sol, double Tol)
    {
        math_Vector Sol1 = new(1, 2), Sol2 = new(1, 2);

        Sol1[1] = Sol[1];
        Sol1[2] = Sol[2];
        Sol2[1] = Sol[3];
        Sol2[2] = Sol[4];

        bool issol = corde1.IsSolution(Sol1, Tol);
        issol = issol && corde2.IsSolution(Sol2, Tol);
        tol = Tol;
        if (issol)
        {
            distmin = Min(distmin, corde1.PointOnS().Distance(corde2.PointOnS()));
        }

        return issol;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        math_Vector x = new(1, 2), f = new(1, 2);

        x[1] = X[1];
        x[2] = X[2];
        corde1.Value(x, f);
        F[1] = f[1];
        F[2] = f[2];

        x[1] = X[3];
        x[2] = X[4];
        corde2.Value(x, f);
        F[3] = f[1];
        F[4] = f[2];

        return true;
    }

    public override bool Derivatives(math_Vector X, math_Matrix D)
    {
        int i, j;
        math_Vector x = new(1, 2);
        math_Matrix d = new(1, 2, 1, 2);

        x[1] = X[1];
        x[2] = X[2];
        corde1.Derivatives(x, d);
        for (i = 1; i < 3; i++)
        {
            for (j = 1; j < 3; j++)
            {
                D[i, j] = d[i, j];
                D[i, j + 2] = 0.0;
            }
        }

        x[1] = X[3];
        x[2] = X[4];
        corde2.Derivatives(x, d);
        for (i = 1; i < 3; i++)
        {
            for (j = 1; j < 3; j++)
            {
                D[i + 2, j + 2] = d[i, j];
                D[i + 2, j] = 0.0;
            }
        }

        return true;
    }

    public override gp_Pnt PointOnS1()
    {
        return corde1.PointOnS();
    }

    public override gp_Pnt PointOnS2()
    {
        return corde2.PointOnS();
    }

    public override bool IsTangencyPoint()
    {
        return corde1.IsTangencyPoint() && corde2.IsTangencyPoint();
    }

    public override gp_Vec TangentOnS1()
    {
        return corde1.TangentOnS();
    }

    public override gp_Vec TangentOnS2()
    {
        return corde2.TangentOnS();
    }

    public override gp_Vec2d Tangent2dOnS1()
    {
        return corde1.Tangent2dOnS();
    }

    public override gp_Vec2d Tangent2dOnS2()
    {
        return corde2.Tangent2dOnS();
    }

    //=======================================================================
    // function : Tangent
    // purpose  : TgF,NmF et TgL,NmL les tangentes et normales respectives
    //           aux surfaces S1 et S2
    //=======================================================================
    public override void Tangent(double U1, double V1, double U2, double V2, out gp_Vec TgF, out gp_Vec TgL,
                                 out gp_Vec NmF, out gp_Vec NmL)
    {
        gp_Vec nplan;
        bool revF = false;
        bool revL = false;

        nplan = corde1.NPlan();

        surf1.D1(U1, V1, out _, out gp_Vec d1u1, out gp_Vec d1v1);
        NmF = d1u1.Crossed(d1v1);

        surf2.D1(U2, V2, out _, out gp_Vec d1u2, out gp_Vec d1v2);
        NmL = d1u2.Crossed(d1v2);

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

    //=======================================================================
    // function : GetSectionSize
    // purpose  : Non implementee (non necessaire car non rationel)
    //=======================================================================
    /// <summary>Not implemented: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double GetSectionSize()
    {
        throw new Standard_NotImplemented("BlendFunc_Chamfer::GetSectionSize()");
    }
}
