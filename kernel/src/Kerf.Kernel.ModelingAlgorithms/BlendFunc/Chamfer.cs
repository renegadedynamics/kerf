// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Chamfer.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Chamfer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Modified : 20/08/96 PMN Ajout des methodes (Nb)Intervals et IsRationnal
// Modified : 30/12/96 PMN Ajout GetMinimalWeight, GetSectionSize;

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>The function of an "ordinary" chamfer: constant distances from the spine to the two surfaces.</summary>
public class Chamfer : GenChamfer
{
    private readonly Corde _corde1;
    private readonly Corde _corde2;

    public Chamfer(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, CurveAdaptorBase cg)
        : base(s1, s2, cg)
    {
        _corde1 = new Corde(s1, cg);
        _corde2 = new Corde(s2, cg);
    }

    public override void Set(double dist1, double dist2, int choix)
    {
        _corde1.SetDist(dist1);
        _corde2.SetDist(dist2);
        _choix = choix;
    }

    public override void Set(double param)
    {
        _corde1.SetParam(param);
        _corde2.SetParam(param);
    }

    public override bool IsSolution(math_Vector sol, double tol)
    {
        math_Vector sol1 = new(1, 2), sol2 = new(1, 2);

        sol1[1] = sol[1];
        sol1[2] = sol[2];
        sol2[1] = sol[3];
        sol2[2] = sol[4];

        bool issol = _corde1.IsSolution(sol1, tol);
        issol = issol && _corde2.IsSolution(sol2, tol);
        _tol = tol;
        if (issol)
        {
            _distmin = Min(_distmin, _corde1.PointOnS().Distance(_corde2.PointOnS()));
        }

        return issol;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        math_Vector x = new(1, 2), f = new(1, 2);

        x[1] = X[1];
        x[2] = X[2];
        _corde1.Value(x, f);
        F[1] = f[1];
        F[2] = f[2];

        x[1] = X[3];
        x[2] = X[4];
        _corde2.Value(x, f);
        F[3] = f[1];
        F[4] = f[2];

        return true;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        int i, j;
        math_Vector x = new(1, 2);
        Matrix d = new(1, 2, 1, 2);

        x[1] = X[1];
        x[2] = X[2];
        _corde1.Derivatives(x, d);
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
        _corde2.Derivatives(x, d);
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

    public override Point3D PointOnS1()
    {
        return _corde1.PointOnS();
    }

    public override Point3D PointOnS2()
    {
        return _corde2.PointOnS();
    }

    public override bool IsTangencyPoint
    {
        get
        {
            return _corde1.IsTangencyPoint && _corde2.IsTangencyPoint;
        }
    }

    public override Vector3D TangentOnS1()
    {
        return _corde1.TangentOnS();
    }

    public override Vector3D TangentOnS2()
    {
        return _corde2.TangentOnS();
    }

    public override Vector2D Tangent2dOnS1()
    {
        return _corde1.Tangent2dOnS();
    }

    public override Vector2D Tangent2dOnS2()
    {
        return _corde2.Tangent2dOnS();
    }

    //=======================================================================
    // function : Tangent
    // purpose  : TgF,NmF et TgL,NmL les tangentes et normales respectives
    //           aux surfaces S1 et S2
    //=======================================================================
    public override void Tangent(double u1, double v1, double u2, double v2, out Vector3D tgF, out Vector3D tgL,
                                 out Vector3D nmF, out Vector3D nmL)
    {
        Vector3D nplan;
        bool revF = false;
        bool revL = false;

        nplan = _corde1.NPlan();

        _surf1.D1(u1, v1, out _, out Vector3D d1u1, out Vector3D d1v1);
        nmF = d1u1.Crossed(d1v1);

        _surf2.D1(u2, v2, out _, out Vector3D d1u2, out Vector3D d1v2);
        nmL = d1u2.Crossed(d1v2);

        tgF = (nplan.Crossed(nmF)).Normalized();
        tgL = (nplan.Crossed(nmL)).Normalized();

        if ((_choix == 2) || (_choix == 5))
        {
            revF = true;
            revL = true;
        }

        if ((_choix == 4) || (_choix == 7))
        {
            revL = true;
        }

        if ((_choix == 3) || (_choix == 8))
        {
            revF = true;
        }

        if (revF)
        {
            tgF = tgF.Reversed();
        }

        if (revL)
        {
            tgL = tgL.Reversed();
        }
    }

    //=======================================================================
    // function : GetSectionSize
    // purpose  : Non implementee (non necessaire car non rationel)
    //=======================================================================
    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double GetSectionSize()
    {
        throw new MethodNotImplementedException("BlendFunc_Chamfer::GetSectionSize()");
    }
}
