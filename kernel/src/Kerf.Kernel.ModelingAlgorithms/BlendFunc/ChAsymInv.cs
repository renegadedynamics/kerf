// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ChAsymInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ChAsymInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>The function of an asymmetric chamfer with one end on a surface's boundary.</summary>
internal class ChAsymInv : FuncInv
{
    private readonly SurfaceAdaptorBase _surf1;
    private readonly SurfaceAdaptorBase _surf2;
    private double _dist1;
    private double _angle;
    private double _tgang;
    private readonly CurveAdaptorBase _curv;
    private Curve2DAdaptorBase? _csurf;
    private int _choix;
    private bool _first;
    private readonly math_Vector _fx;
    private readonly Matrix _dx;

    public ChAsymInv(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, CurveAdaptorBase c)
    {
        _surf1 = s1;
        _surf2 = s2;
        _dist1 = RealLast();
        _angle = RealLast();
        _tgang = RealLast();
        _curv = c;
        _choix = 0;
        _first = false;
        _fx = new math_Vector(1, 4);
        _dx = new Matrix(1, 4, 1, 4);
        _ = _angle;
    }

    /// <summary>Sets the distance on the first surface, the angle and the "quadrant".</summary>
    public void Set(double dist1, double angle, int choix)
    {
        _dist1 = Math.Abs(dist1);
        _angle = angle;
        _tgang = Math.Tan(angle);
        _choix = choix;
    }

    /// <summary>Returns 4.</summary>
    public override int NbEquations()
    {
        return 4;
    }

    public override void Set(bool onFirst, Curve2DAdaptorBase c)
    {
        _first = onFirst;
        _csurf = c;
    }

    public override void GetTolerance(math_Vector tolerance, double tol)
    {
        tolerance[1] = _csurf!.Resolution(tol);
        tolerance[2] = _curv.Resolution(tol);
        if (_first)
        {
            tolerance[3] = _surf2.UResolution(tol);
            tolerance[4] = _surf2.VResolution(tol);
        }
        else
        {
            tolerance[3] = _surf1.UResolution(tol);
            tolerance[4] = _surf1.VResolution(tol);
        }
    }

    public override void GetBounds(math_Vector infBound, math_Vector supBound)
    {
        infBound[1] = _csurf!.FirstParameter();
        infBound[2] = _curv.FirstParameter();
        supBound[1] = _csurf.LastParameter();
        supBound[2] = _curv.LastParameter();

        if (_first)
        {
            infBound[3] = _surf2.FirstUParameter();
            infBound[4] = _surf2.FirstVParameter();
            supBound[3] = _surf2.LastUParameter();
            supBound[4] = _surf2.LastVParameter();
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(infBound[3]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(supBound[3]))
            {
                double range = (supBound[3] - infBound[3]);
                infBound[3] -= range;
                supBound[3] += range;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(infBound[4]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(supBound[4]))
            {
                double range = (supBound[4] - infBound[4]);
                infBound[4] -= range;
                supBound[4] += range;
            }
        }
        else
        {
            infBound[3] = _surf1.FirstUParameter();
            infBound[4] = _surf1.FirstVParameter();
            supBound[3] = _surf1.LastUParameter();
            supBound[4] = _surf1.LastVParameter();
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(infBound[3]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(supBound[3]))
            {
                double range = (supBound[3] - infBound[3]);
                infBound[3] -= range;
                supBound[3] += range;
            }

            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(infBound[4]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(supBound[4]))
            {
                double range = (supBound[4] - infBound[4]);
                infBound[4] -= range;
                supBound[4] += range;
            }
        }
    }

    public override bool IsSolution(math_Vector sol, double tol)
    {
        math_Vector valsol = new(1, 4);
        Point3D pts1, pts2;
        Vector3D nplan, nsurf1, tsurf1;
        Vector3D d1u1, d1v1;

        _curv.D1(sol[2], out _, out Vector3D d1gui);
        nplan = d1gui.Normalized();

        Point2D pt2d = _csurf!.Value(sol[1]);

        if (_first)
        {
            _surf1.D1(pt2d.X, pt2d.Y, out pts1, out d1u1, out d1v1);
            pts2 = _surf2.Value(sol[3], sol[4]);
        }
        else
        {
            _surf1.D1(sol[3], sol[4], out pts1, out d1u1, out d1v1);
            pts2 = _surf2.Value(pt2d.X, pt2d.Y);
        }

        nsurf1 = d1u1.Crossed(d1v1);
        tsurf1 = nsurf1.Crossed(nplan);

        Vector3D s1s2 = new Vector3D(pts1, pts2);
        double pScaInv = 1.0 / tsurf1.Dot(s1s2), temp;
        double nordu1 = d1u1.Magnitude, nordv1 = d1v1.Magnitude;

        temp = 2.0 * (nordu1 + nordv1) * s1s2.Magnitude + 2.0 * nordu1 * nordv1;

        Value(sol, valsol);

        return Math.Abs(valsol[1]) < tol && Math.Abs(valsol[2]) < tol
               && Math.Abs(valsol[3]) < 2.0 * _dist1 * tol
               && Math.Abs(valsol[4]) < tol * (1.0 + _tgang) * Math.Abs(pScaInv) * temp;
    }

    /// <summary>The values (from <paramref name="degF"/> 0) and the derivatives (to <paramref name="degL"/> 1).</summary>
    public bool ComputeValues(math_Vector x, int degF, int degL)
    {
        if (degF > degL)
        {
            return false;
        }

        Vector3D nplan, dnplan = new Vector3D(), d1gui, d2gui, d1u1, d1v1, d2u1 = new Vector3D(), d2v1 = new Vector3D(),
               d2uv1 = new Vector3D(), d1u2 = new Vector3D(), d1v2 = new Vector3D();
        Vector3D nsurf1, tsurf1;
        Point3D pts1, pts2, ptgui;
        double pScaInv, f4;
        double normg = 0.0;
        Point2D pt2d;
        Vector2D v2d = new Vector2D();

        if ((degF == 0) && (degL == 0))
        {
            _curv.D1(x[2], out ptgui, out d1gui);
            nplan = d1gui.Normalized();
            if (_choix % 2 != 0)
            {
                nplan = nplan.Reversed();
            }

            pt2d = _csurf!.Value(x[1]);

            if (_first)
            {
                _surf1.D1(pt2d.X, pt2d.Y, out pts1, out d1u1, out d1v1);
                pts2 = _surf2.Value(x[3], x[4]);
            }
            else
            {
                _surf1.D1(x[3], x[4], out pts1, out d1u1, out d1v1);
                pts2 = _surf2.Value(pt2d.X, pt2d.Y);
            }
        }
        else
        {
            _curv.D2(x[2], out ptgui, out d1gui, out d2gui);
            nplan = d1gui.Normalized();
            normg = d1gui.Magnitude;
            dnplan = (d2gui - nplan.Dot(d2gui) * nplan) / normg;

            if (_choix % 2 != 0)
            {
                nplan = nplan.Reversed();
                dnplan = dnplan.Reversed();
                normg = -normg;
            }

            _csurf!.D1(x[1], out pt2d, out v2d);

            if (_first)
            {
                _surf1.D2(pt2d.X, pt2d.Y, out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
                _surf2.D1(x[3], x[4], out pts2, out d1u2, out d1v2);
            }
            else
            {
                _surf1.D2(x[3], x[4], out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
                _surf2.D1(pt2d.X, pt2d.Y, out pts2, out d1u2, out d1v2);
            }
        }

        Vector3D nps1 = new Vector3D(ptgui, pts1), s1s2 = new Vector3D(pts1, pts2);
        nsurf1 = d1u1.Crossed(d1v1);
        tsurf1 = nsurf1.Crossed(nplan);
        pScaInv = 1.0 / s1s2.Dot(tsurf1);
        f4 = nplan.Dot(tsurf1.Crossed(s1s2)) * pScaInv;

        if (degF == 0)
        {
            double dist;
            dist = ptgui.XYZ().Dot(nplan.XYZ());

            _fx[1] = pts1.XYZ().Dot(nplan.XYZ()) - dist;
            _fx[2] = pts2.XYZ().Dot(nplan.XYZ()) - dist;
            _fx[3] = _dist1 * _dist1 - nps1.SquareMagnitude;
            _fx[4] = _tgang - f4;
        }

        if (degL == 1)
        {
            Vector3D dwtsurf1, tempVec;
            double temp;
            Vector3D nps2 = new Vector3D(ptgui, pts2);

            if (_first)
            {
                Vector3D dw1du1, dw1dv1, dw1csurf, dw1pts1;
                dw1pts1 = v2d.X * d1u1 + v2d.Y * d1v1;
                dw1du1 = v2d.X * d2u1 + v2d.Y * d2uv1;
                dw1dv1 = v2d.X * d2uv1 + v2d.Y * d2v1;
                dw1csurf = (dw1du1.Crossed(d1v1) + d1u1.Crossed(dw1dv1)).Crossed(nplan);
                dwtsurf1 = nsurf1.Crossed(dnplan);

                _dx[1, 1] = nplan.Dot(dw1pts1);
                _dx[1, 2] = dnplan.Dot(nps1) - normg;
                _dx[1, 3] = 0.0;
                _dx[1, 4] = 0.0;

                _dx[2, 1] = 0.0;
                _dx[2, 2] = dnplan.Dot(nps2) - normg;
                _dx[2, 3] = nplan.Dot(d1u2);
                _dx[2, 4] = nplan.Dot(d1v2);

                tempVec = 2.0 * nps1;
                _dx[3, 1] = -dw1pts1.Dot(tempVec);
                _dx[3, 2] = d1gui.Dot(tempVec);
                _dx[3, 3] = 0.0;
                _dx[3, 4] = 0.0;

                temp = f4 * (dw1csurf.Dot(s1s2) - tsurf1.Dot(dw1pts1));
                temp += nplan.Dot(tsurf1.Crossed(dw1pts1) - dw1csurf.Crossed(s1s2));
                _dx[4, 1] = pScaInv * temp;

                temp = f4 * dwtsurf1.Dot(s1s2);
                // OCCT: disagrees with finite differences on a curved guide (decision 1040)
                temp -= dnplan.Dot(tempVec) + nplan.Dot(dwtsurf1.Crossed(s1s2));
                _dx[4, 2] = pScaInv * temp;

                temp = f4 * tsurf1.Dot(d1u2) - nplan.Dot(tsurf1.Crossed(d1u2));
                _dx[4, 3] = pScaInv * temp;

                temp = f4 * tsurf1.Dot(d1v2) - nplan.Dot(tsurf1.Crossed(d1v2));
                _dx[4, 4] = pScaInv * temp;
            }
            else
            {
                Vector3D d1utsurf1, d1vtsurf1, dw2pts2;
                d1utsurf1 = (d2u1.Crossed(d1v1) + d1u1.Crossed(d2uv1)).Crossed(nplan);
                d1vtsurf1 = (d2uv1.Crossed(d1v1) + d1u1.Crossed(d2v1)).Crossed(nplan);
                dw2pts2 = v2d.X * d1u2 + v2d.Y * d1v2;
                dwtsurf1 = nsurf1.Crossed(dnplan);

                _dx[1, 1] = 0.0;
                _dx[1, 2] = dnplan.Dot(nps1) - normg;
                _dx[1, 3] = nplan.Dot(d1u1);
                _dx[1, 4] = nplan.Dot(d1v1);

                _dx[2, 1] = nplan.Dot(dw2pts2);
                _dx[2, 2] = dnplan.Dot(nps2) - normg;
                _dx[2, 3] = 0.0;
                _dx[2, 4] = 0.0;

                tempVec = 2.0 * nps1;
                _dx[3, 1] = 0.0;
                _dx[3, 2] = d1gui.Dot(tempVec);
                tempVec = tempVec.Reversed();
                _dx[3, 3] = d1u1.Dot(tempVec);
                _dx[3, 4] = d1v1.Dot(tempVec);

                temp = f4 * tsurf1.Dot(dw2pts2) - nplan.Dot(tsurf1.Crossed(dw2pts2));
                _dx[4, 1] = pScaInv * temp;

                temp = f4 * dwtsurf1.Dot(s1s2);
                temp -= dnplan.Dot(tempVec) + nplan.Dot(dwtsurf1.Crossed(s1s2));
                _dx[4, 2] = pScaInv * temp;

                temp = f4 * (d1utsurf1.Dot(s1s2) - tsurf1.Dot(d1u1));
                temp += nplan.Dot(tsurf1.Crossed(d1u1) - d1utsurf1.Crossed(s1s2));
                _dx[4, 3] = pScaInv * temp;

                temp = f4 * (d1vtsurf1.Dot(s1s2) - tsurf1.Dot(d1v1));
                temp += nplan.Dot(tsurf1.Crossed(d1v1) - d1vtsurf1.Crossed(s1s2));
                _dx[4, 4] = pScaInv * temp;
            }
        }

        return true;
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        bool error = ComputeValues(x, 0, 0);
        f.Initialized(_fx);
        return error;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        bool error = ComputeValues(x, 1, 1);
        d.Initialized(_dx);
        return error;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        bool error = ComputeValues(x, 0, 1);
        f.Initialized(_fx);
        d.Initialized(_dx);
        return error;
        // A commented-out check against finite differences follows in OCCT.
    }
}
