// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ChamfInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ChamfInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>The function of a chamfer of two constant distances with one end on a surface's boundary.</summary>
public class ChamfInv : GenChamfInv
{
    private readonly Corde _corde1;
    private readonly Corde _corde2;

    public ChamfInv(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, CurveAdaptorBase c)
        : base(s1, s2, c)
    {
        _corde1 = new Corde(_surf1, _curv);
        _corde2 = new Corde(_surf2, _curv);
    }

    public override void Set(double dist1, double dist2, int choix)
    {
        double dis1, dis2;
        _choix = choix;
        switch (_choix)
        {
            case 1:
            case 2:
                {
                    dis1 = -dist1;
                    dis2 = -dist2;
                }
                break;
            case 3:
            case 4:
                {
                    dis1 = dist1;
                    dis2 = -dist2;
                }
                break;
            case 5:
            case 6:
                {
                    dis1 = dist1;
                    dis2 = dist2;
                }
                break;
            case 7:
            case 8:
                {
                    dis1 = -dist1;
                    dis2 = dist2;
                }
                break;
            default:
                dis1 = -dist1;
                dis2 = -dist2;
                break;
        }

        _corde1.SetDist(dis1);
        _corde2.SetDist(dis2);
    }

    public override bool IsSolution(math_Vector sol, double tol)
    {
        _csurf!.D1(sol[1], out Point2D p2d, out _);

        math_Vector sol1 = new(1, 2), sol2 = new(1, 2);
        bool issol;

        sol1[1] = p2d.X();
        sol1[2] = p2d.Y();

        sol2[1] = sol[3];
        sol2[2] = sol[4];

        if (_first)
        {
            issol = _corde1.IsSolution(sol1, tol);
            issol = issol && _corde2.IsSolution(sol2, tol);
        }
        else
        {
            issol = _corde1.IsSolution(sol2, tol);
            issol = issol && _corde2.IsSolution(sol1, tol);
        }

        return issol;
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        _csurf!.D1(x[1], out Point2D p2d, out _);

        _corde1.SetParam(x[2]);
        _corde2.SetParam(x[2]);

        math_Vector x1 = new(1, 2), f1 = new(1, 2), x2 = new(1, 2), f2 = new(1, 2);

        x1[1] = p2d.X();
        x1[2] = p2d.Y();
        x2[1] = x[3];
        x2[2] = x[4];

        if (_first)
        {
            _corde1.Value(x1, f1);
            _corde2.Value(x2, f2);
        }
        else
        {
            _corde1.Value(x2, f1);
            _corde2.Value(x1, f2);
        }

        f[1] = f1[1];
        f[2] = f1[2];
        f[3] = f2[1];
        f[4] = f2[2];

        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        int i, j;
        Vector2D df1 = new Vector2D(), df2 = new Vector2D();
        Point3D pts, ptgui;
        Vector3D temp = new Vector3D(), d1u, d1v, nplan;
        math_Vector x1 = new(1, 2), x2 = new(1, 2);
        Matrix d1 = new(1, 2, 1, 2), d2 = new(1, 2, 1, 2);

        _csurf!.D1(x[1], out Point2D p2d, out Vector2D v2d);
        _corde1.SetParam(x[2]);
        _corde2.SetParam(x[2]);

        x1[1] = p2d.X();
        x1[2] = p2d.Y();
        x2[1] = x[3];
        x2[2] = x[4];

        if (_first)
        {
            // p2d = pts est sur surf1
            ptgui = _corde1.PointOnGuide();
            nplan = _corde1.NPlan();
            _corde2.Derivatives(x2, d2);
            _corde1.DerFguide(x1, ref df1);
            _corde2.DerFguide(x2, ref df2);
            _surf1.D1(x1[1], x1[2], out pts, out d1u, out d1v);
        }
        else
        {
            //  p2d = pts est sur surf2
            ptgui = _corde2.PointOnGuide();
            nplan = _corde2.NPlan();
            _corde1.Derivatives(x2, d1);
            _corde1.DerFguide(x2, ref df1);
            _corde2.DerFguide(x1, ref df2);
            _surf2.D1(x1[1], x1[2], out pts, out d1u, out d1v);
        }

        // derivees par rapport a T
        temp = Vector3D.LinearForm(v2d.X(), d1u, v2d.Y(), d1v);
        if (_first)
        {
            d[1, 1] = nplan.Dot(temp);
            d[2, 1] = 2 * (new Vector3D(ptgui, pts).Dot(temp));
            d[3, 1] = 0.0;
            d[4, 1] = 0.0;
        }
        else
        {
            d[1, 1] = 0.0;
            d[2, 1] = 0.0;
            d[3, 1] = nplan.Dot(temp);
            d[4, 1] = 2 * (new Vector3D(ptgui, pts).Dot(temp));
        }

        // derivees par rapport a W
        d[1, 2] = df1.X();
        d[2, 2] = df1.Y();
        d[3, 2] = df2.X();
        d[4, 2] = df2.Y();

        // derivees par rapport a U et V
        if (_first)
        {
            for (i = 1; i < 3; i++)
            {
                for (j = 3; j < 5; j++)
                {
                    d[i, j] = 0.0;
                    d[i + 2, j] = d2[i, j - 2];
                }
            }
        }
        else
        {
            for (i = 1; i < 3; i++)
            {
                for (j = 3; j < 5; j++)
                {
                    d[i, j] = d1[i, j - 2];
                    d[i + 2, j] = 0.0;
                }
            }
        }

        return true;
    }
}
