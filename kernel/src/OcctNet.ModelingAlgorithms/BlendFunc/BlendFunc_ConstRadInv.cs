// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstRadInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstRadInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The constant-radius fillet function with one end on a curve on one of the surfaces (a restriction): its
/// variables are the parameter on the curve on surface, the guide's parameter, and (U, V) on the other surface.
/// </summary>
public class BlendFunc_ConstRadInv : Blend_FuncInv
{
    private const double Eps = 1.0e-15;

    private readonly SurfaceAdaptorBase surf1;
    private readonly SurfaceAdaptorBase surf2;
    private readonly CurveAdaptorBase curv;
    private Curve2DAdaptorBase? csurf;
    private double ray1;
    private double ray2;
    private int choix;
    private bool first;

    public BlendFunc_ConstRadInv(SurfaceAdaptorBase S1, SurfaceAdaptorBase S2, CurveAdaptorBase C)
    {
        surf1 = S1;
        surf2 = S2;
        curv = C;
        ray1 = 0.0;
        ray2 = 0.0;
        choix = 0;
        first = false;
    }

    /// <summary>Sets the radius and the "quadrant" <paramref name="Choix"/>, as BlendFunc_ConstRad.</summary>
    public void Set(double R, int Choix)
    {
        choix = Choix;
        switch (choix)
        {
            case 1:
            case 2:
                {
                    ray1 = -R;
                    ray2 = -R;
                }
                break;
            case 3:
            case 4:
                {
                    ray1 = R;
                    ray2 = -R;
                }
                break;
            case 5:
            case 6:
                {
                    ray1 = R;
                    ray2 = R;
                }
                break;
            case 7:
            case 8:
                {
                    ray1 = -R;
                    ray2 = R;
                }
                break;
            default:
                ray1 = ray2 = -R;
                break;
        }
    }

    public override void Set(bool OnFirst, Curve2DAdaptorBase C)
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
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(InfBound[3]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(SupBound[3]))
            {
                double range = (SupBound[3] - InfBound[3]);
                InfBound[3] -= range;
                SupBound[3] += range;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(InfBound[4]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(SupBound[4]))
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
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(InfBound[3]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(SupBound[3]))
            {
                double range = (SupBound[3] - InfBound[3]);
                InfBound[3] -= range;
                SupBound[3] += range;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(InfBound[4]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(SupBound[4]))
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
        return Math.Abs(valsol[1]) <= Tol
               && valsol[2] * valsol[2] + valsol[3] * valsol[3] + valsol[4] * valsol[4] <= Tol * Tol;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        curv.D1(X[2], out Point3D ptcur, out Vector3D d1cur);
        Vector3D nplan = d1cur.Normalized();
        double theD = -(nplan.XYZ().Dot(ptcur.XYZ()));
        Point2D pt2d = csurf!.Value(X[1]);

        Point3D pts1, pts2;
        Vector3D d1u1, d1v1, d1u2, d1v2;
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

        F[1] = (nplan.X() * (pts1.X() + pts2.X()) + nplan.Y() * (pts1.Y() + pts2.Y())
                + nplan.Z() * (pts1.Z() + pts2.Z()))
                 / 2.0
               + theD;

        Vector3D ns1 = d1u1.Crossed(d1v1);
        if (ns1.Magnitude() < Eps)
        {
            if (first)
            {
                BlendFunc.ComputeNormal(surf1, pt2d, ref ns1);
            }
            else
            {
                Point2D P = new Point2D(X[3], X[4]);
                BlendFunc.ComputeNormal(surf1, P, ref ns1);
            }
        }

        Vector3D ns2 = d1u2.Crossed(d1v2);
        if (ns2.Magnitude() < Eps)
        {
            if (!first)
            {
                BlendFunc.ComputeNormal(surf2, pt2d, ref ns2);
            }
            else
            {
                Point2D P = new Point2D(X[3], X[4]);
                BlendFunc.ComputeNormal(surf2, P, ref ns2);
            }
        }

        double norm1 = nplan.Crossed(ns1).Magnitude();
        double norm2 = nplan.Crossed(ns2).Magnitude();
        if (norm1 < Eps)
        {
            norm1 = 1;
        }

        if (norm2 < Eps)
        {
            norm2 = 1; // Unsatisfactory, but it is not necessary to stop
        }

        Vector3D resul = new Vector3D();
        ns1.SetLinearForm(nplan.Dot(ns1) / norm1, nplan, -1.0 / norm1, ns1);
        ns2.SetLinearForm(nplan.Dot(ns2) / norm2, nplan, -1.0 / norm2, ns2);
        resul.SetLinearForm(ray1, ns1, -1.0, new Vector3D(pts2.XYZ()), -ray2, ns2, new Vector3D(pts1.XYZ()));
        F[2] = resul.X();
        F[3] = resul.Y();
        F[4] = resul.Z();
        return true;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        Vector3D d1u1, d1v1, d1u2, d1v2;
        Vector3D d2u1, d2v1, d2uv1, d2u2, d2v2, d2uv2;
        Vector3D ns1, ns2, nplan, dnplan = new Vector3D(), ncrossns1, ncrossns2, temp = new Vector3D();
        Point3D pts1, pts2;
        double norm1, norm2, ndotns1, ndotns2, normtgcur;
        double theD;

        curv.D2(X[2], out Point3D ptcur, out Vector3D d1cur, out Vector3D d2cur);
        normtgcur = d1cur.Magnitude();
        nplan = d1cur.Normalized();
        theD = -(nplan.XYZ().Dot(ptcur.XYZ()));

        // OCCT: the plane's offset theD stands where Values has -nplan.Dot(d2cur) (decision 1038).
        dnplan.SetLinearForm(theD, nplan, d2cur);
        dnplan /= normtgcur;

        csurf!.D1(X[1], out Point2D p2d, out Vector2D v2d);

        if (first)
        {
            surf1.D2(p2d.X(), p2d.Y(), out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            surf2.D2(X[3], X[4], out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
            temp.SetLinearForm(v2d.X(), d1u1, v2d.Y(), d1v1);
            D[1, 1] = nplan.Dot(temp) / 2.0;
            temp.SetXYZ(0.5 * (pts1.XYZ() + pts2.XYZ()) - ptcur.XYZ());
            D[1, 2] = dnplan.Dot(temp) - normtgcur;
            D[1, 3] = nplan.Dot(d1u2) / 2.0;
            D[1, 4] = nplan.Dot(d1v2) / 2.0;
        }
        else
        {
            surf1.D2(X[3], X[4], out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            surf2.D2(p2d.X(), p2d.Y(), out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
            temp.SetLinearForm(v2d.X(), d1u2, v2d.Y(), d1v2);
            D[1, 1] = nplan.Dot(temp) / 2.0;
            temp.SetXYZ(0.5 * (pts1.XYZ() + pts2.XYZ()) - ptcur.XYZ());
            D[1, 2] = dnplan.Dot(temp) - normtgcur;
            D[1, 3] = nplan.Dot(d1u1) / 2.0;
            D[1, 4] = nplan.Dot(d1v1) / 2.0;
        }

        ns1 = d1u1.Crossed(d1v1);
        if (ns1.Magnitude() < Eps)
        {
            if (first)
            {
                BlendFunc.ComputeNormal(surf1, p2d, ref ns1);
            }
            else
            {
                Point2D P = new Point2D(X[3], X[4]);
                BlendFunc.ComputeNormal(surf1, P, ref ns1);
            }
        }

        ns2 = d1u2.Crossed(d1v2);
        if (ns2.Magnitude() < Eps)
        {
            if (!first)
            {
                BlendFunc.ComputeNormal(surf2, p2d, ref ns2);
            }
            else
            {
                Point2D P = new Point2D(X[3], X[4]);
                BlendFunc.ComputeNormal(surf2, P, ref ns2);
            }
        }

        ncrossns1 = nplan.Crossed(ns1);
        ncrossns2 = nplan.Crossed(ns2);
        norm1 = ncrossns1.Magnitude();
        norm2 = ncrossns2.Magnitude();
        if (norm1 < Eps)
        {
            norm1 = 1; // Unsatisfactory, but it is not necessary to stop
        }

        if (norm2 < Eps)
        {
            norm2 = 1; // Unsatisfactory, but it is not necessary to stop
        }

        ndotns1 = nplan.Dot(ns1);
        ndotns2 = nplan.Dot(ns2);

        DerivativesTail(X, D, d1u1, d1v1, d1u2, d1v2, d2u1, d2v1, d2uv1, d2u2, d2v2, d2uv2, ns1, ns2, nplan, dnplan,
                        ncrossns1, ncrossns2, v2d, norm1, norm2, ndotns1, ndotns2);
        return true;
    }

    // The derivatives in the 2nd to 4th rows, which Derivatives and Values compute with the same code.
    private void DerivativesTail(math_Vector X, Matrix D, in Vector3D d1u1, in Vector3D d1v1, in Vector3D d1u2,
                                 in Vector3D d1v2, in Vector3D d2u1, in Vector3D d2v1, in Vector3D d2uv1, in Vector3D d2u2,
                                 in Vector3D d2v2, in Vector3D d2uv2, in Vector3D ns1, in Vector3D ns2, in Vector3D nplan,
                                 in Vector3D dnplan, in Vector3D ncrossns1, in Vector3D ncrossns2, in Vector2D v2d,
                                 double norm1, double norm2, double ndotns1, double ndotns2)
    {
        Vector3D resul1 = new Vector3D(), resul2 = new Vector3D(), temp;
        double grosterme;

        // Derived compared to u1
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns1.Dot(nplan.Crossed(temp)) / norm1 / norm1;
        resul1.SetLinearForm(-ray1 / norm1 * (grosterme * ndotns1 - nplan.Dot(temp)), nplan,
                             ray1 * grosterme / norm1, ns1, -ray1 / norm1, temp, d1u1);

        // Derived compared to v1
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns1.Dot(nplan.Crossed(temp)) / norm1 / norm1;
        resul2.SetLinearForm(-ray1 / norm1 * (grosterme * ndotns1 - nplan.Dot(temp)), nplan,
                             ray1 * grosterme / norm1, ns1, -ray1 / norm1, temp, d1v1);

        if (first)
        {
            D[2, 1] = v2d.X() * resul1.X() + v2d.Y() * resul2.X();
            D[3, 1] = v2d.X() * resul1.Y() + v2d.Y() * resul2.Y();
            D[4, 1] = v2d.X() * resul1.Z() + v2d.Y() * resul2.Z();
        }
        else
        {
            D[2, 3] = resul1.X();
            D[3, 3] = resul1.Y();
            D[4, 3] = resul1.Z();
            D[2, 4] = resul2.X();
            D[3, 4] = resul2.Y();
            D[4, 4] = resul2.Z();
        }

        // derived compared to w (parameter on guideline)
        // It is assumed that the radius is constant
        grosterme = ncrossns1.Dot(dnplan.Crossed(ns1)) / norm1 / norm1;
        resul1.SetLinearForm(-ray1 / norm1 * (grosterme * ndotns1 - dnplan.Dot(ns1)), nplan, ray1 * ndotns1 / norm1,
                             dnplan, ray1 * grosterme / norm1, ns1);

        grosterme = ncrossns2.Dot(dnplan.Crossed(ns2)) / norm2 / norm2;
        resul2.SetLinearForm(ray2 / norm2 * (grosterme * ndotns2 - dnplan.Dot(ns2)), nplan, -ray2 * ndotns2 / norm2,
                             dnplan, -ray2 * grosterme / norm2, ns2);

        D[2, 2] = resul1.X() + resul2.X();
        D[3, 2] = resul1.Y() + resul2.Y();
        D[4, 2] = resul1.Z() + resul2.Z();

        // Derived compared to u2
        temp = d2u2.Crossed(d1v2).Added(d1u2.Crossed(d2uv2));
        grosterme = ncrossns2.Dot(nplan.Crossed(temp)) / norm2 / norm2;
        resul1.SetLinearForm(ray2 / norm2 * (grosterme * ndotns2 - nplan.Dot(temp)), nplan,
                             -ray2 * grosterme / norm2, ns2, ray2 / norm2, temp);
        resul1.Subtract(d1u2);

        // Derived compared to v2
        temp = d2uv2.Crossed(d1v2).Added(d1u2.Crossed(d2v2));
        grosterme = ncrossns2.Dot(nplan.Crossed(temp)) / norm2 / norm2;
        resul2.SetLinearForm(ray2 / norm2 * (grosterme * ndotns2 - nplan.Dot(temp)), nplan,
                             -ray2 * grosterme / norm2, ns2, ray2 / norm2, temp);
        resul2.Subtract(d1v2);

        if (!first)
        {
            D[2, 1] = v2d.X() * resul1.X() + v2d.Y() * resul2.X();
            D[3, 1] = v2d.X() * resul1.Y() + v2d.Y() * resul2.Y();
            D[4, 1] = v2d.X() * resul1.Z() + v2d.Y() * resul2.Z();
        }
        else
        {
            D[2, 3] = resul1.X();
            D[3, 3] = resul1.Y();
            D[4, 3] = resul1.Z();
            D[2, 4] = resul2.X();
            D[3, 4] = resul2.Y();
            D[4, 4] = resul2.Z();
        }
    }

    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        Vector3D d1u1, d1v1, d1u2, d1v2;
        Vector3D d2u1, d2v1, d2uv1, d2u2, d2v2, d2uv2;
        Vector3D ns1, ns2, nplan, dnplan = new Vector3D(), ncrossns1, ncrossns2, resul1 = new Vector3D(),
               temp = new Vector3D();
        Point3D pts1, pts2;
        double norm1, norm2, ndotns1, ndotns2, normtgcur;
        double theD;

        curv.D2(X[2], out Point3D ptcur, out Vector3D d1cur, out Vector3D d2cur);
        normtgcur = d1cur.Magnitude();
        nplan = d1cur.Normalized();
        theD = -(nplan.XYZ().Dot(ptcur.XYZ()));
        dnplan.SetLinearForm(-nplan.Dot(d2cur), nplan, d2cur);
        dnplan /= normtgcur;

        csurf!.D1(X[1], out Point2D p2d, out Vector2D v2d);

        if (first)
        {
            surf1.D2(p2d.X(), p2d.Y(), out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            surf2.D2(X[3], X[4], out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
            temp.SetLinearForm(v2d.X(), d1u1, v2d.Y(), d1v1);
            D[1, 1] = nplan.Dot(temp) / 2.0;
            temp.SetXYZ(0.5 * (pts1.XYZ() + pts2.XYZ()) - ptcur.XYZ());
            D[1, 2] = dnplan.Dot(temp) - normtgcur;
            D[1, 3] = nplan.Dot(d1u2) / 2.0;
            D[1, 4] = nplan.Dot(d1v2) / 2.0;
        }
        else
        {
            surf1.D2(X[3], X[4], out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            surf2.D2(p2d.X(), p2d.Y(), out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
            temp.SetLinearForm(v2d.X(), d1u2, v2d.Y(), d1v2);
            D[1, 1] = nplan.Dot(temp) / 2.0;
            temp.SetXYZ(0.5 * (pts1.XYZ() + pts2.XYZ()) - ptcur.XYZ());
            D[1, 2] = dnplan.Dot(temp) - normtgcur;
            D[1, 3] = nplan.Dot(d1u1) / 2.0;
            D[1, 4] = nplan.Dot(d1v1) / 2.0;
        }

        F[1] = (nplan.X() * (pts1.X() + pts2.X()) + nplan.Y() * (pts1.Y() + pts2.Y())
                + nplan.Z() * (pts1.Z() + pts2.Z()))
                 / 2.0
               + theD;

        ns1 = d1u1.Crossed(d1v1);
        if (ns1.Magnitude() < Eps)
        {
            if (first)
            {
                BlendFunc.ComputeNormal(surf1, p2d, ref ns1);
            }
            else
            {
                Point2D P = new Point2D(X[3], X[4]);
                BlendFunc.ComputeNormal(surf1, P, ref ns1);
            }
        }

        ns2 = d1u2.Crossed(d1v2);
        if (ns2.Magnitude() < Eps)
        {
            if (!first)
            {
                BlendFunc.ComputeNormal(surf2, p2d, ref ns2);
            }
            else
            {
                Point2D P = new Point2D(X[3], X[4]);
                BlendFunc.ComputeNormal(surf2, P, ref ns2);
            }
        }

        ncrossns1 = nplan.Crossed(ns1);
        ncrossns2 = nplan.Crossed(ns2);
        norm1 = ncrossns1.Magnitude();
        norm2 = ncrossns2.Magnitude();
        if (norm1 < Eps)
        {
            norm1 = 1; // Unsatisfactory, but it is not necessary to stop
        }

        if (norm2 < Eps)
        {
            norm2 = 1; // Unsatisfactory, but it is not necessary to stop
        }

        ndotns1 = nplan.Dot(ns1);
        ndotns2 = nplan.Dot(ns2);

        temp.SetLinearForm(ndotns1 / norm1, nplan, -1.0 / norm1, ns1);
        resul1.SetLinearForm(ray1, temp, new Vector3D(pts2, pts1));
        temp.SetLinearForm(ndotns2 / norm2, nplan, -1.0 / norm2, ns2);
        resul1.Subtract(ray2 * temp);

        F[2] = resul1.X();
        F[3] = resul1.Y();
        F[4] = resul1.Z();

        DerivativesTail(X, D, d1u1, d1v1, d1u2, d1v2, d2u1, d2v1, d2uv1, d2u2, d2v2, d2uv2, ns1, ns2, nplan, dnplan,
                        ncrossns1, ncrossns2, v2d, norm1, norm2, ndotns1, ndotns2);
        return true;
    }
}
