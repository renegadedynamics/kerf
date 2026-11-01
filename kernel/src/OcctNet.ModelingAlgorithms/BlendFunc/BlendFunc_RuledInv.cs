// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_RuledInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_RuledInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The ruled blend function with one end on a surface's boundary.</summary>
public class BlendFunc_RuledInv : Blend_FuncInv
{
    private readonly Adaptor3d_Surface surf1;
    private readonly Adaptor3d_Surface surf2;
    private readonly Adaptor3d_Curve curv;
    private Adaptor2d_Curve2d? csurf;
    private bool first;

    public BlendFunc_RuledInv(Adaptor3d_Surface S1, Adaptor3d_Surface S2, Adaptor3d_Curve C)
    {
        surf1 = S1;
        surf2 = S2;
        curv = C;
        first = false;
    }

    public override void Set(bool OnFirst, Adaptor2d_Curve2d C)
    {
        first = OnFirst;
        csurf = C;
    }

    /// <summary>Returns 4.</summary>
    public override int NbEquations()
    {
        return 4;
    }

    public override void GetTolerance(math_Vector Tolerance, double Tol)
    {
        Tolerance[1] = csurf!.Resolution(Tol);
        Tolerance[2] = curv.Resolution(Tol);
        if (first)
        {
            Tolerance[3] = surf2.UResolution(Tol);
            Tolerance[4] = surf2.VResolution(Tol);
        }
        else
        {
            Tolerance[3] = surf1.UResolution(Tol);
            Tolerance[4] = surf1.VResolution(Tol);
        }
    }

    public override void GetBounds(math_Vector InfBound, math_Vector SupBound)
    {
        InfBound[1] = csurf!.FirstParameter();
        InfBound[2] = curv.FirstParameter();
        SupBound[1] = csurf.LastParameter();
        SupBound[2] = curv.LastParameter();

        if (first)
        {
            InfBound[3] = surf2.FirstUParameter();
            InfBound[4] = surf2.FirstVParameter();
            SupBound[3] = surf2.LastUParameter();
            SupBound[4] = surf2.LastVParameter();
            if (!Precision.IsInfinite(InfBound[3]) && !Precision.IsInfinite(SupBound[3]))
            {
                double range = (SupBound[3] - InfBound[3]);
                InfBound[3] -= range;
                SupBound[3] += range;
            }

            if (!Precision.IsInfinite(InfBound[4]) && !Precision.IsInfinite(SupBound[4]))
            {
                double range = (SupBound[4] - InfBound[4]);
                InfBound[4] -= range;
                SupBound[4] += range;
            }
        }
        else
        {
            InfBound[3] = surf1.FirstUParameter();
            InfBound[4] = surf1.FirstVParameter();
            SupBound[3] = surf1.LastUParameter();
            SupBound[4] = surf1.LastVParameter();
            if (!Precision.IsInfinite(InfBound[3]) && !Precision.IsInfinite(SupBound[3]))
            {
                double range = (SupBound[3] - InfBound[3]);
                InfBound[3] -= range;
                SupBound[3] += range;
            }

            if (!Precision.IsInfinite(InfBound[4]) && !Precision.IsInfinite(SupBound[4]))
            {
                double range = (SupBound[4] - InfBound[4]);
                InfBound[4] -= range;
                SupBound[4] += range;
            }
        }
    }

    public override bool IsSolution(math_Vector Sol, double Tol)
    {
        math_Vector valsol = new(1, 4);
        Value(Sol, valsol);
        return Math.Abs(valsol[1]) <= Tol && Math.Abs(valsol[2]) <= Tol && Math.Abs(valsol[3]) <= Tol
               && Math.Abs(valsol[4]) <= Tol;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        curv.D1(X[2], out gp_Pnt ptcur, out gp_Vec d1cur);
        gp_XYZ nplan = d1cur.Normalized().XYZ();
        double theD = -(nplan.Dot(ptcur.XYZ()));
        gp_Pnt2d pt2d = csurf!.Value(X[1]);

        gp_Pnt pts1, pts2;
        gp_Vec d1u1, d1v1, d1u2, d1v2;
        if (first)
        {
            surf1.D1(pt2d.X(), pt2d.Y(), out pts1, out d1u1, out d1v1);
            surf2.D1(X[3], X[4], out pts2, out d1u2, out d1v2);
        }
        else
        {
            surf1.D1(X[3], X[4], out pts1, out d1u1, out d1v1);
            surf2.D1(pt2d.X(), pt2d.Y(), out pts2, out d1u2, out d1v2);
        }

        gp_XYZ temp = pts2.XYZ() - pts1.XYZ();

        gp_XYZ ns1 = d1u1.Crossed(d1v1).XYZ();
        gp_XYZ ns2 = d1u2.Crossed(d1v2).XYZ();
        double norm1 = nplan.Crossed(ns1).Modulus();
        double norm2 = nplan.Crossed(ns2).Modulus();
        ns1.SetLinearForm(nplan.Dot(ns1) / norm1, nplan, -1.0 / norm1, ns1);
        ns2.SetLinearForm(nplan.Dot(ns2) / norm2, nplan, -1.0 / norm2, ns2);

        F[1] = (nplan.Dot(pts1.XYZ())) + theD;
        F[2] = (nplan.Dot(pts2.XYZ())) + theD;

        F[3] = temp.Dot(ns1);
        F[4] = temp.Dot(ns2);

        return true;
    }

    public override bool Derivatives(math_Vector X, math_Matrix D)
    {
        return ComputeDerivatives(X, null, D);
    }

    public override bool Values(math_Vector X, math_Vector F, math_Matrix D)
    {
        return ComputeDerivatives(X, F, D);
    }

    // OCCT's Derivatives and Values share their code: Values also computes F (where marked).
    private bool ComputeDerivatives(math_Vector X, math_Vector? F, math_Matrix D)
    {
        curv.D2(X[2], out gp_Pnt ptcur, out gp_Vec d1cur, out gp_Vec d2cur);
        double normtgcur = d1cur.Magnitude();
        gp_Vec nplan = d1cur.Normalized();
        double theD = -(nplan.XYZ().Dot(ptcur.XYZ()));

        gp_Vec dnplan = new gp_Vec();
        dnplan.SetLinearForm(-nplan.Dot(d2cur), nplan, d2cur);
        dnplan /= normtgcur;

        csurf!.D1(X[1], out gp_Pnt2d p2d, out gp_Vec2d v2d);

        gp_Pnt pts1, pts2;
        gp_Vec d1u1, d1v1, d1u2, d1v2;
        gp_Vec d2u1, d2v1, d2u2, d2v2, d2uv1, d2uv2;
        gp_Vec dpdt = new gp_Vec(), p1p2;

        if (first)
        {
            surf1.D2(p2d.X(), p2d.Y(), out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            surf2.D2(X[3], X[4], out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
            dpdt.SetLinearForm(v2d.X(), d1u1, v2d.Y(), d1v1);
            p1p2 = new gp_Vec(pts1, pts2);
            D[1, 1] = dpdt.Dot(nplan);
            D[1, 2] = dnplan.XYZ().Dot(pts1.XYZ() - ptcur.XYZ()) - normtgcur;
            D[1, 3] = 0.0;
            D[1, 4] = 0.0;
            D[2, 1] = 0.0;
            D[2, 2] = dnplan.XYZ().Dot(pts2.XYZ() - ptcur.XYZ()) - normtgcur;
            D[2, 3] = d1u2.Dot(nplan);
            D[2, 4] = d1v2.Dot(nplan);
        }
        else
        {
            surf1.D2(X[3], X[4], out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            surf2.D2(p2d.X(), p2d.Y(), out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
            dpdt.SetLinearForm(v2d.X(), d1u2, v2d.Y(), d1v2);
            p1p2 = new gp_Vec(pts1, pts2);
            D[1, 1] = 0.0;
            D[1, 2] = dnplan.XYZ().Dot(pts1.XYZ() - ptcur.XYZ()) - normtgcur;
            D[1, 3] = d1u1.Dot(nplan);
            D[1, 4] = d1v1.Dot(nplan);
            D[2, 1] = dpdt.Dot(nplan);
            D[2, 2] = dnplan.XYZ().Dot(pts2.XYZ() - ptcur.XYZ()) - normtgcur;
            D[2, 3] = 0.0;
            D[2, 4] = 0.0;
        }

        gp_Vec ns1 = d1u1.Crossed(d1v1);
        gp_Vec ns2 = d1u2.Crossed(d1v2);
        gp_Vec ncrossns1 = nplan.Crossed(ns1);
        gp_Vec ncrossns2 = nplan.Crossed(ns2);
        double norm1 = ncrossns1.Magnitude();
        double norm2 = ncrossns2.Magnitude();

        double ndotns1 = nplan.Dot(ns1);
        double ndotns2 = nplan.Dot(ns2);

        gp_Vec nor1 = new gp_Vec(), nor2 = new gp_Vec();
        nor1.SetLinearForm(ndotns1 / norm1, nplan, -1.0 / norm1, ns1);
        nor2.SetLinearForm(ndotns2 / norm2, nplan, -1.0 / norm2, ns2);

        if (F is not null)
        {
            // Values only
            F[1] = (nplan.Dot(new gp_Vec(pts1.XYZ()))) + theD;
            F[2] = (nplan.Dot(new gp_Vec(pts2.XYZ()))) + theD;

            F[3] = p1p2.Dot(nor1);
            F[4] = p1p2.Dot(nor2);
        }

        if (first)
        {
            D[3, 3] = d1u2.Dot(nor1);
            D[3, 4] = d1v2.Dot(nor1);
            D[4, 1] = -(dpdt.Dot(nor2));
        }
        else
        {
            D[3, 1] = dpdt.Dot(nor1);
            D[4, 3] = -(d1u1.Dot(nor2));
            D[4, 4] = -(d1v1.Dot(nor2));
        }

        gp_Vec resul1 = new gp_Vec(), resul2 = new gp_Vec(), temp;
        double grosterme;

        // Derivee de nor1 par rapport a u1
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns1.Dot(nplan.Crossed(temp)) / norm1 / norm1;
        resul1.SetLinearForm(-(grosterme * ndotns1 - nplan.Dot(temp)) / norm1, nplan, grosterme / norm1, ns1,
                             -1.0 / norm1, temp);

        // Derivee par rapport a v1
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns1.Dot(nplan.Crossed(temp)) / norm1 / norm1;
        resul2.SetLinearForm(-(grosterme * ndotns1 - nplan.Dot(temp)) / norm1, nplan, grosterme / norm1, ns1,
                             -1.0 / norm1, temp);

        if (first)
        {
            resul1.SetLinearForm(v2d.X(), resul1, v2d.Y(), resul2);
            D[3, 1] = p1p2.Dot(resul1) - (dpdt.Dot(nor1));
        }
        else
        {
            D[3, 3] = -(d1u1.Dot(nor1)) + p1p2.Dot(resul1);
            D[3, 4] = -(d1v1.Dot(nor1)) + p1p2.Dot(resul2);
        }

        // Derivee de nor2 par rapport a u2
        temp = d2u2.Crossed(d1v2).Added(d1u2.Crossed(d2uv2));
        grosterme = ncrossns2.Dot(nplan.Crossed(temp)) / norm2 / norm2;
        resul1.SetLinearForm(-(grosterme * ndotns2 - nplan.Dot(temp)) / norm2, nplan, grosterme / norm2, ns2,
                             -1.0 / norm2, temp);

        // Derivee par rapport a v2
        temp = d2uv2.Crossed(d1v2).Added(d1u2.Crossed(d2v2));
        grosterme = ncrossns2.Dot(nplan.Crossed(temp)) / norm2 / norm2;
        resul2.SetLinearForm(-(grosterme * ndotns2 - nplan.Dot(temp)) / norm2, nplan, grosterme / norm2, ns2,
                             -1.0 / norm2, temp);

        if (first)
        {
            D[4, 3] = d1u2.Dot(nor2) + p1p2.Dot(resul1);
            D[4, 4] = d1v2.Dot(nor2) + p1p2.Dot(resul2);
        }
        else
        {
            resul1.SetLinearForm(v2d.X(), resul1, v2d.Y(), resul2);
            D[4, 1] = p1p2.Dot(resul1) + dpdt.Dot(nor2);
        }

        // derivee par rapport a w (parametre sur ligne guide)
        grosterme = ncrossns1.Dot(dnplan.Crossed(ns1)) / norm1 / norm1;
        resul1.SetLinearForm(-(grosterme * ndotns1 - dnplan.Dot(ns1)) / norm1, nplan, ndotns1 / norm1, dnplan,
                             grosterme / norm1, ns1);

        grosterme = ncrossns2.Dot(dnplan.Crossed(ns2)) / norm2 / norm2;
        resul2.SetLinearForm(-(grosterme * ndotns2 - dnplan.Dot(ns2)) / norm2, nplan, ndotns2 / norm2, dnplan,
                             grosterme / norm2, ns2);

        D[3, 2] = p1p2.Dot(resul1);
        D[4, 2] = p1p2.Dot(resul2);

        return true;
    }
}
