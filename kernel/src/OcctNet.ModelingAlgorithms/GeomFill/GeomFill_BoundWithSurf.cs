// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BoundWithSurf.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_BoundWithSurf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A curve on a surface as a boundary of a GeomFill_ConstrainedFilling: the filling surface will be within Tol3d of
/// the curve, and its normal along the curve within Tolang of the normalised normal of the supporting surface.
/// </summary>
/// <remarks>
/// A working object (decision 5951). OCCT copies the curve-on-surface adaptor into the boundary; the port keeps a
/// shallow copy (<c>ShallowCopy</c>, sharing the geometry) and never changes the caller's.
/// <see cref="Reparametrize"/> sets a reparametrisation law, a Law_BSpFunc whose B-spline is frozen.
/// </remarks>
public class GeomFill_BoundWithSurf : GeomFill_Boundary
{
    private readonly Adaptor3d_CurveOnSurface myConS;
    private Law_Function? myPar;

    public GeomFill_BoundWithSurf(Adaptor3d_CurveOnSurface CurveOnSurf, double Tol3d, double Tolang)
        : base(Tol3d, Tolang)
    {
        myConS = (Adaptor3d_CurveOnSurface)CurveOnSurf.ShallowCopy();
    }

    public override gp_Pnt Value(double U)
    {
        double x = U;
        if (myPar is not null)
        {
            x = myPar.Value(U);
        }
        return myConS.Value(x);
    }

    public override void D1(double U, out gp_Pnt P, out gp_Vec V)
    {
        double x = U, dx = 1.0;
        if (myPar is not null)
        {
            myPar.D1(U, out x, out dx);
        }
        myConS.D1(x, out P, out V);
        V.Multiply(dx);
    }

    public override bool HasNormals() => true;

    public override gp_Vec Norm(double U)
    {
        // voir s il ne faudrait pas utiliser LProp ou autre.
        if (!HasNormals())
        {
            throw new Standard_Failure("BoundWithSurf Norm : pas de contrainte");
        }

        double x, y;
        double w = U;
        if (myPar is not null)
        {
            w = myPar.Value(U);
        }
        myConS.GetCurve()!.Value(w).Coord(out x, out y);
        gp_Pnt P;
        gp_Vec Su, Sv;
        myConS.GetSurface()!.D1(x, y, out P, out Su, out Sv);
        Su.Cross(Sv);
        Su.Normalize();
        return Su;
    }

    public override void D1Norm(double U, out gp_Vec N, out gp_Vec DN)
    {
        if (!HasNormals())
        {
            throw new Standard_Failure("BoundWithSurf Norm : pas de contrainte");
        }

        gp_Pnt2d P2d;
        gp_Vec2d V2d;
        double x, y, dx, dy;
        double w = U, dw = 1.0;
        if (myPar is not null)
        {
            myPar.D1(U, out w, out dw);
        }
        myConS.GetCurve()!.D1(w, out P2d, out V2d);
        P2d.Coord(out x, out y);
        V2d.Multiply(dw);
        V2d.Coord(out dx, out dy);
        gp_Pnt P;
        gp_Vec Su, Sv, Suu, Suv, Svv;
        myConS.GetSurface()!.D2(x, y, out P, out Su, out Sv, out Suu, out Svv, out Suv);
        N = Su.Crossed(Sv);
        N.Normalize();
        double nsuu = N.Dot(Suu), nsuv = N.Dot(Suv), nsvv = N.Dot(Svv);
        double susu = Su.Dot(Su), susv = Su.Dot(Sv), svsv = Sv.Dot(Sv);
        double deno = (susu * svsv) - (susv * susv);
        if (Math.Abs(deno) < 1.0e-16)
        {
            // on embraye sur un calcul approche, c est mieux que rien!?!
            gp_Vec temp = Norm(U + 1.0e-12);
            DN = N.Multiplied(-1.0);
            DN.Add(temp);
            DN.Multiply(1.0e-12);
        }
        else
        {
            double a = (-nsuu * svsv + nsuv * susv) / deno;
            double b = (nsuu * susv - nsuv * susu) / deno;
            double c = (-nsuv * svsv + nsvv * susv) / deno;
            double d = (nsuv * susv - nsvv * susu) / deno;

            gp_Vec temp1 = Su.Multiplied(a);
            gp_Vec temp2 = Sv.Multiplied(b);
            temp1.Add(temp2);
            temp2 = Su.Multiplied(c);
            gp_Vec temp3 = Sv.Multiplied(d);
            temp2.Add(temp3);
            temp1.Multiply(dx);
            temp2.Multiply(dy);
            DN = temp1.Added(temp2);
        }
    }

    public override void Reparametrize(double First, double Last, bool HasDF, bool HasDL, double DF, double DL, bool Rev)
    {
        Law_BSpline curve = Law.Reparametrize(myConS, First, Last, HasDF, HasDL, DF, DL, Rev, 30);
        myPar = new Law_BSpFunc();
        ((Law_BSpFunc)myPar).SetCurve(curve);
    }

    public override void Bounds(out double First, out double Last)
    {
        if (myPar is not null)
        {
            myPar.Bounds(out First, out Last);
        }
        else
        {
            First = myConS.FirstParameter();
            Last = myConS.LastParameter();
        }
    }

    public override bool IsDegenerated() => false;
}
