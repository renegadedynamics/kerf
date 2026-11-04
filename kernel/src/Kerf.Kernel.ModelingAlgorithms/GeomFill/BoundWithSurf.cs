// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BoundWithSurf.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BoundWithSurf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>
/// A curve on a surface as a boundary of a GeomFill_ConstrainedFilling: the filling surface will be within Tol3d of
/// the curve, and its normal along the curve within Tolang of the normalised normal of the supporting surface.
/// </summary>
/// <remarks>
/// A working object (decision 806). OCCT copies the curve-on-surface adaptor into the boundary; the port keeps a
/// shallow copy (<c>ShallowCopy</c>, sharing the geometry) and never changes the caller's.
/// <see cref="Reparametrize"/> sets a reparametrisation law, a Law_BSpFunc whose B-spline is frozen.
/// </remarks>
internal class BoundWithSurf : Boundary
{
    private readonly CurveOnSurface _conS;
    private LawsFunction? _par;

    public BoundWithSurf(CurveOnSurface curveOnSurf, double tol3d, double tolang)
        : base(tol3d, tolang)
    {
        _conS = (CurveOnSurface)curveOnSurf.ShallowCopy();
    }

    public override Point3D Value(double u)
    {
        double x = u;
        if (_par is not null)
        {
            x = _par.Value(u);
        }
        return _conS.Value(x);
    }

    public override void D1(double u, out Point3D p, out Vector3D v)
    {
        double x = u, dx = 1.0;
        if (_par is not null)
        {
            _par.D1(u, out x, out dx);
        }
        _conS.D1(x, out p, out v);
        v = v.Multiplied(dx);
    }

    public override bool HasNormals => true;

    public override Vector3D Norm(double u)
    {
        // voir s il ne faudrait pas utiliser LProp ou autre.
        if (!HasNormals)
        {
            throw new FailureException("BoundWithSurf Norm : pas de contrainte");
        }

        double x, y;
        double w = u;
        if (_par is not null)
        {
            w = _par.Value(u);
        }
        _conS.GetCurve()!.Value(w).Coord(out x, out y);
        Point3D p;
        Vector3D su, sv;
        _conS.GetSurface()!.D1(x, y, out p, out su, out sv);
        su = su.Crossed(sv);
        su = su.Normalized();
        return su;
    }

    public override void D1Norm(double u, out Vector3D n, out Vector3D dn)
    {
        if (!HasNormals)
        {
            throw new FailureException("BoundWithSurf Norm : pas de contrainte");
        }

        Point2D p2d;
        Vector2D v2d;
        double x, y, dx, dy;
        double w = u, dw = 1.0;
        if (_par is not null)
        {
            _par.D1(u, out w, out dw);
        }
        _conS.GetCurve()!.D1(w, out p2d, out v2d);
        p2d.Coord(out x, out y);
        v2d = v2d.Multiplied(dw);
        v2d.Coord(out dx, out dy);
        Point3D p;
        Vector3D su, sv, suu, suv, svv;
        _conS.GetSurface()!.D2(x, y, out p, out su, out sv, out suu, out svv, out suv);
        n = su.Crossed(sv);
        n = n.Normalized();
        double nsuu = n.Dot(suu), nsuv = n.Dot(suv), nsvv = n.Dot(svv);
        double susu = su.Dot(su), susv = su.Dot(sv), svsv = sv.Dot(sv);
        double deno = (susu * svsv) - (susv * susv);
        if (Math.Abs(deno) < 1.0e-16)
        {
            // on embraye sur un calcul approche, c est mieux que rien!?!
            Vector3D temp = Norm(u + 1.0e-12);
            dn = n.Multiplied(-1.0);
            dn = dn.Added(temp);
            dn = dn.Multiplied(1.0e-12);
        }
        else
        {
            double a = (-nsuu * svsv + nsuv * susv) / deno;
            double b = (nsuu * susv - nsuv * susu) / deno;
            double c = (-nsuv * svsv + nsvv * susv) / deno;
            double d = (nsuv * susv - nsvv * susu) / deno;

            Vector3D temp1 = su.Multiplied(a);
            Vector3D temp2 = sv.Multiplied(b);
            temp1 = temp1.Added(temp2);
            temp2 = su.Multiplied(c);
            Vector3D temp3 = sv.Multiplied(d);
            temp2 = temp2.Added(temp3);
            temp1 = temp1.Multiplied(dx);
            temp2 = temp2.Multiplied(dy);
            dn = temp1.Added(temp2);
        }
    }

    public override void Reparametrize(double first, double last, bool hasDF, bool hasDL, double df, double dl, bool rev)
    {
        BSpline curve = global::Kerf.Engine.Construction.Laws.Law.Reparametrize(_conS, first, last, hasDF, hasDL, df, dl, rev, 30);
        _par = new BSpFunc();
        ((BSpFunc)_par).SetCurve(curve);
    }

    public override void Bounds(out double first, out double last)
    {
        if (_par is not null)
        {
            _par.Bounds(out first, out last);
        }
        else
        {
            first = _conS.FirstParameter();
            last = _conS.LastParameter();
        }
    }

    public override bool IsDegenerated => false;
}
