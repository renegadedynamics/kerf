// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ChAsymInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ChAsymInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>The function of an asymmetric chamfer with one end on a surface's boundary.</summary>
public class BlendFunc_ChAsymInv : Blend_FuncInv
{
    private readonly SurfaceAdaptorBase surf1;
    private readonly SurfaceAdaptorBase surf2;
    private double dist1;
    private double angle;
    private double tgang;
    private readonly CurveAdaptorBase curv;
    private Curve2DAdaptorBase? csurf;
    private int choix;
    private bool first;
    private readonly math_Vector FX;
    private readonly Matrix DX;

    public BlendFunc_ChAsymInv(SurfaceAdaptorBase S1, SurfaceAdaptorBase S2, CurveAdaptorBase C)
    {
        surf1 = S1;
        surf2 = S2;
        dist1 = RealLast();
        angle = RealLast();
        tgang = RealLast();
        curv = C;
        choix = 0;
        first = false;
        FX = new math_Vector(1, 4);
        DX = new Matrix(1, 4, 1, 4);
        _ = angle;
    }

    /// <summary>Sets the distance on the first surface, the angle and the "quadrant".</summary>
    public void Set(double Dist1, double Angle, int Choix)
    {
        dist1 = Math.Abs(Dist1);
        angle = Angle;
        tgang = Math.Tan(Angle);
        choix = Choix;
    }

    /// <summary>Returns 4.</summary>
    public override int NbEquations()
    {
        return 4;
    }

    public override void Set(bool OnFirst, Curve2DAdaptorBase C)
    {
        first = OnFirst;
        csurf = C;
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
        Point3D pts1, pts2;
        Vector3D nplan, Nsurf1, tsurf1;
        Vector3D d1u1, d1v1;

        curv.D1(Sol[2], out _, out Vector3D d1gui);
        nplan = d1gui.Normalized();

        Point2D pt2d = csurf!.Value(Sol[1]);

        if (first)
        {
            surf1.D1(pt2d.X(), pt2d.Y(), out pts1, out d1u1, out d1v1);
            pts2 = surf2.Value(Sol[3], Sol[4]);
        }
        else
        {
            surf1.D1(Sol[3], Sol[4], out pts1, out d1u1, out d1v1);
            pts2 = surf2.Value(pt2d.X(), pt2d.Y());
        }

        Nsurf1 = d1u1.Crossed(d1v1);
        tsurf1 = Nsurf1.Crossed(nplan);

        Vector3D s1s2 = new Vector3D(pts1, pts2);
        double PScaInv = 1.0 / tsurf1.Dot(s1s2), temp;
        double Nordu1 = d1u1.Magnitude(), Nordv1 = d1v1.Magnitude();

        temp = 2.0 * (Nordu1 + Nordv1) * s1s2.Magnitude() + 2.0 * Nordu1 * Nordv1;

        Value(Sol, valsol);

        return Math.Abs(valsol[1]) < Tol && Math.Abs(valsol[2]) < Tol
               && Math.Abs(valsol[3]) < 2.0 * dist1 * Tol
               && Math.Abs(valsol[4]) < Tol * (1.0 + tgang) * Math.Abs(PScaInv) * temp;
    }

    /// <summary>The values (from <paramref name="DegF"/> 0) and the derivatives (to <paramref name="DegL"/> 1).</summary>
    public bool ComputeValues(math_Vector X, int DegF, int DegL)
    {
        if (DegF > DegL)
        {
            return false;
        }

        Vector3D nplan, dnplan = new Vector3D(), d1gui, d2gui, d1u1, d1v1, d2u1 = new Vector3D(), d2v1 = new Vector3D(),
               d2uv1 = new Vector3D(), d1u2 = new Vector3D(), d1v2 = new Vector3D();
        Vector3D Nsurf1, tsurf1;
        Point3D pts1, pts2, ptgui;
        double PScaInv, F4;
        double Normg = 0.0;
        Point2D pt2d;
        Vector2D v2d = new Vector2D();

        if ((DegF == 0) && (DegL == 0))
        {
            curv.D1(X[2], out ptgui, out d1gui);
            nplan = d1gui.Normalized();
            if (choix % 2 != 0)
            {
                nplan.Reverse();
            }

            pt2d = csurf!.Value(X[1]);

            if (first)
            {
                surf1.D1(pt2d.X(), pt2d.Y(), out pts1, out d1u1, out d1v1);
                pts2 = surf2.Value(X[3], X[4]);
            }
            else
            {
                surf1.D1(X[3], X[4], out pts1, out d1u1, out d1v1);
                pts2 = surf2.Value(pt2d.X(), pt2d.Y());
            }
        }
        else
        {
            curv.D2(X[2], out ptgui, out d1gui, out d2gui);
            nplan = d1gui.Normalized();
            Normg = d1gui.Magnitude();
            dnplan = (d2gui - nplan.Dot(d2gui) * nplan) / Normg;

            if (choix % 2 != 0)
            {
                nplan.Reverse();
                dnplan.Reverse();
                Normg = -Normg;
            }

            csurf!.D1(X[1], out pt2d, out v2d);

            if (first)
            {
                surf1.D2(pt2d.X(), pt2d.Y(), out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
                surf2.D1(X[3], X[4], out pts2, out d1u2, out d1v2);
            }
            else
            {
                surf1.D2(X[3], X[4], out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
                surf2.D1(pt2d.X(), pt2d.Y(), out pts2, out d1u2, out d1v2);
            }
        }

        Vector3D nps1 = new Vector3D(ptgui, pts1), s1s2 = new Vector3D(pts1, pts2);
        Nsurf1 = d1u1.Crossed(d1v1);
        tsurf1 = Nsurf1.Crossed(nplan);
        PScaInv = 1.0 / s1s2.Dot(tsurf1);
        F4 = nplan.Dot(tsurf1.Crossed(s1s2)) * PScaInv;

        if (DegF == 0)
        {
            double Dist;
            Dist = ptgui.XYZ().Dot(nplan.XYZ());

            FX[1] = pts1.XYZ().Dot(nplan.XYZ()) - Dist;
            FX[2] = pts2.XYZ().Dot(nplan.XYZ()) - Dist;
            FX[3] = dist1 * dist1 - nps1.SquareMagnitude();
            FX[4] = tgang - F4;
        }

        if (DegL == 1)
        {
            Vector3D dwtsurf1, tempVec;
            double temp;
            Vector3D nps2 = new Vector3D(ptgui, pts2);

            if (first)
            {
                Vector3D dw1du1, dw1dv1, dw1csurf, dw1pts1;
                dw1pts1 = v2d.X() * d1u1 + v2d.Y() * d1v1;
                dw1du1 = v2d.X() * d2u1 + v2d.Y() * d2uv1;
                dw1dv1 = v2d.X() * d2uv1 + v2d.Y() * d2v1;
                dw1csurf = (dw1du1.Crossed(d1v1) + d1u1.Crossed(dw1dv1)).Crossed(nplan);
                dwtsurf1 = Nsurf1.Crossed(dnplan);

                DX[1, 1] = nplan.Dot(dw1pts1);
                DX[1, 2] = dnplan.Dot(nps1) - Normg;
                DX[1, 3] = 0.0;
                DX[1, 4] = 0.0;

                DX[2, 1] = 0.0;
                DX[2, 2] = dnplan.Dot(nps2) - Normg;
                DX[2, 3] = nplan.Dot(d1u2);
                DX[2, 4] = nplan.Dot(d1v2);

                tempVec = 2.0 * nps1;
                DX[3, 1] = -dw1pts1.Dot(tempVec);
                DX[3, 2] = d1gui.Dot(tempVec);
                DX[3, 3] = 0.0;
                DX[3, 4] = 0.0;

                temp = F4 * (dw1csurf.Dot(s1s2) - tsurf1.Dot(dw1pts1));
                temp += nplan.Dot(tsurf1.Crossed(dw1pts1) - dw1csurf.Crossed(s1s2));
                DX[4, 1] = PScaInv * temp;

                temp = F4 * dwtsurf1.Dot(s1s2);
                // OCCT: disagrees with finite differences on a curved guide (decision 1040)
                temp -= dnplan.Dot(tempVec) + nplan.Dot(dwtsurf1.Crossed(s1s2));
                DX[4, 2] = PScaInv * temp;

                temp = F4 * tsurf1.Dot(d1u2) - nplan.Dot(tsurf1.Crossed(d1u2));
                DX[4, 3] = PScaInv * temp;

                temp = F4 * tsurf1.Dot(d1v2) - nplan.Dot(tsurf1.Crossed(d1v2));
                DX[4, 4] = PScaInv * temp;
            }
            else
            {
                Vector3D d1utsurf1, d1vtsurf1, dw2pts2;
                d1utsurf1 = (d2u1.Crossed(d1v1) + d1u1.Crossed(d2uv1)).Crossed(nplan);
                d1vtsurf1 = (d2uv1.Crossed(d1v1) + d1u1.Crossed(d2v1)).Crossed(nplan);
                dw2pts2 = v2d.X() * d1u2 + v2d.Y() * d1v2;
                dwtsurf1 = Nsurf1.Crossed(dnplan);

                DX[1, 1] = 0.0;
                DX[1, 2] = dnplan.Dot(nps1) - Normg;
                DX[1, 3] = nplan.Dot(d1u1);
                DX[1, 4] = nplan.Dot(d1v1);

                DX[2, 1] = nplan.Dot(dw2pts2);
                DX[2, 2] = dnplan.Dot(nps2) - Normg;
                DX[2, 3] = 0.0;
                DX[2, 4] = 0.0;

                tempVec = 2.0 * nps1;
                DX[3, 1] = 0.0;
                DX[3, 2] = d1gui.Dot(tempVec);
                tempVec.Reverse();
                DX[3, 3] = d1u1.Dot(tempVec);
                DX[3, 4] = d1v1.Dot(tempVec);

                temp = F4 * tsurf1.Dot(dw2pts2) - nplan.Dot(tsurf1.Crossed(dw2pts2));
                DX[4, 1] = PScaInv * temp;

                temp = F4 * dwtsurf1.Dot(s1s2);
                temp -= dnplan.Dot(tempVec) + nplan.Dot(dwtsurf1.Crossed(s1s2));
                DX[4, 2] = PScaInv * temp;

                temp = F4 * (d1utsurf1.Dot(s1s2) - tsurf1.Dot(d1u1));
                temp += nplan.Dot(tsurf1.Crossed(d1u1) - d1utsurf1.Crossed(s1s2));
                DX[4, 3] = PScaInv * temp;

                temp = F4 * (d1vtsurf1.Dot(s1s2) - tsurf1.Dot(d1v1));
                temp += nplan.Dot(tsurf1.Crossed(d1v1) - d1vtsurf1.Crossed(s1s2));
                DX[4, 4] = PScaInv * temp;
            }
        }

        return true;
    }

    public override bool Value(math_Vector X, math_Vector F)
    {
        bool Error = ComputeValues(X, 0, 0);
        F.Initialized(FX);
        return Error;
    }

    public override bool Derivatives(math_Vector X, Matrix D)
    {
        bool Error = ComputeValues(X, 1, 1);
        D.Initialized(DX);
        return Error;
    }

    public override bool Values(math_Vector X, math_Vector F, Matrix D)
    {
        bool Error = ComputeValues(X, 0, 1);
        F.Initialized(FX);
        D.Initialized(DX);
        return Error;
        // A commented-out check against finite differences follows in OCCT.
    }
}
