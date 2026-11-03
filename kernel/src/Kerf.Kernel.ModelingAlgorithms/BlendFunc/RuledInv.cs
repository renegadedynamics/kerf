// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_RuledInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_RuledInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>The ruled blend function with one end on a surface's boundary.</summary>
public class RuledInv : FuncInv
{
    private readonly SurfaceAdaptorBase _surf1;
    private readonly SurfaceAdaptorBase _surf2;
    private readonly CurveAdaptorBase _curv;
    private Curve2DAdaptorBase? _csurf;
    private bool _first;

    public RuledInv(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, CurveAdaptorBase c)
    {
        _surf1 = s1;
        _surf2 = s2;
        _curv = c;
        _first = false;
    }

    public override void Set(bool onFirst, Curve2DAdaptorBase c)
    {
        _first = onFirst;
        _csurf = c;
    }

    /// <summary>Returns 4.</summary>
    public override int NbEquations()
    {
        return 4;
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
        Value(sol, valsol);
        return Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol && Math.Abs(valsol[3]) <= tol
               && Math.Abs(valsol[4]) <= tol;
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        _curv.D1(x[2], out Point3D ptcur, out Vector3D d1cur);
        Xyz nplan = d1cur.Normalized().XYZ();
        double theD = -(nplan.Dot(ptcur.XYZ()));
        Point2D pt2d = _csurf!.Value(x[1]);

        Point3D pts1, pts2;
        Vector3D d1u1, d1v1, d1u2, d1v2;
        if (_first)
        {
            _surf1.D1(pt2d.X, pt2d.Y, out pts1, out d1u1, out d1v1);
            _surf2.D1(x[3], x[4], out pts2, out d1u2, out d1v2);
        }
        else
        {
            _surf1.D1(x[3], x[4], out pts1, out d1u1, out d1v1);
            _surf2.D1(pt2d.X, pt2d.Y, out pts2, out d1u2, out d1v2);
        }

        Xyz temp = pts2.XYZ() - pts1.XYZ();

        Xyz ns1 = d1u1.Crossed(d1v1).XYZ();
        Xyz ns2 = d1u2.Crossed(d1v2).XYZ();
        double norm1 = nplan.Crossed(ns1).Modulus();
        double norm2 = nplan.Crossed(ns2).Modulus();
        ns1.SetLinearForm(nplan.Dot(ns1) / norm1, nplan, -1.0 / norm1, ns1);
        ns2.SetLinearForm(nplan.Dot(ns2) / norm2, nplan, -1.0 / norm2, ns2);

        f[1] = (nplan.Dot(pts1.XYZ())) + theD;
        f[2] = (nplan.Dot(pts2.XYZ())) + theD;

        f[3] = temp.Dot(ns1);
        f[4] = temp.Dot(ns2);

        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        return ComputeDerivatives(x, null, d);
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        return ComputeDerivatives(x, f, d);
    }

    // OCCT's Derivatives and Values share their code: Values also computes F (where marked).
    private bool ComputeDerivatives(math_Vector x, math_Vector? f, Matrix d)
    {
        _curv.D2(x[2], out Point3D ptcur, out Vector3D d1cur, out Vector3D d2cur);
        double normtgcur = d1cur.Magnitude;
        Vector3D nplan = d1cur.Normalized();
        double theD = -(nplan.XYZ().Dot(ptcur.XYZ()));

        Vector3D dnplan = new Vector3D();
        dnplan = Vector3D.LinearForm(-nplan.Dot(d2cur), nplan, d2cur);
        dnplan /= normtgcur;

        _csurf!.D1(x[1], out Point2D p2d, out Vector2D v2d);

        Point3D pts1, pts2;
        Vector3D d1u1, d1v1, d1u2, d1v2;
        Vector3D d2u1, d2v1, d2u2, d2v2, d2uv1, d2uv2;
        Vector3D dpdt = new Vector3D(), p1p2;

        if (_first)
        {
            _surf1.D2(p2d.X, p2d.Y, out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            _surf2.D2(x[3], x[4], out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
            dpdt = Vector3D.LinearForm(v2d.X, d1u1, v2d.Y, d1v1);
            p1p2 = new Vector3D(pts1, pts2);
            d[1, 1] = dpdt.Dot(nplan);
            d[1, 2] = dnplan.XYZ().Dot(pts1.XYZ() - ptcur.XYZ()) - normtgcur;
            d[1, 3] = 0.0;
            d[1, 4] = 0.0;
            d[2, 1] = 0.0;
            d[2, 2] = dnplan.XYZ().Dot(pts2.XYZ() - ptcur.XYZ()) - normtgcur;
            d[2, 3] = d1u2.Dot(nplan);
            d[2, 4] = d1v2.Dot(nplan);
        }
        else
        {
            _surf1.D2(x[3], x[4], out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            _surf2.D2(p2d.X, p2d.Y, out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
            dpdt = Vector3D.LinearForm(v2d.X, d1u2, v2d.Y, d1v2);
            p1p2 = new Vector3D(pts1, pts2);
            d[1, 1] = 0.0;
            d[1, 2] = dnplan.XYZ().Dot(pts1.XYZ() - ptcur.XYZ()) - normtgcur;
            d[1, 3] = d1u1.Dot(nplan);
            d[1, 4] = d1v1.Dot(nplan);
            d[2, 1] = dpdt.Dot(nplan);
            d[2, 2] = dnplan.XYZ().Dot(pts2.XYZ() - ptcur.XYZ()) - normtgcur;
            d[2, 3] = 0.0;
            d[2, 4] = 0.0;
        }

        Vector3D ns1 = d1u1.Crossed(d1v1);
        Vector3D ns2 = d1u2.Crossed(d1v2);
        Vector3D ncrossns1 = nplan.Crossed(ns1);
        Vector3D ncrossns2 = nplan.Crossed(ns2);
        double norm1 = ncrossns1.Magnitude;
        double norm2 = ncrossns2.Magnitude;

        double ndotns1 = nplan.Dot(ns1);
        double ndotns2 = nplan.Dot(ns2);

        Vector3D nor1 = new Vector3D(), nor2 = new Vector3D();
        nor1 = Vector3D.LinearForm(ndotns1 / norm1, nplan, -1.0 / norm1, ns1);
        nor2 = Vector3D.LinearForm(ndotns2 / norm2, nplan, -1.0 / norm2, ns2);

        if (f is not null)
        {
            // Values only
            f[1] = (nplan.Dot(new Vector3D(pts1.XYZ()))) + theD;
            f[2] = (nplan.Dot(new Vector3D(pts2.XYZ()))) + theD;

            f[3] = p1p2.Dot(nor1);
            f[4] = p1p2.Dot(nor2);
        }

        if (_first)
        {
            d[3, 3] = d1u2.Dot(nor1);
            d[3, 4] = d1v2.Dot(nor1);
            d[4, 1] = -(dpdt.Dot(nor2));
        }
        else
        {
            d[3, 1] = dpdt.Dot(nor1);
            d[4, 3] = -(d1u1.Dot(nor2));
            d[4, 4] = -(d1v1.Dot(nor2));
        }

        Vector3D resul1 = new Vector3D(), resul2 = new Vector3D(), temp;
        double grosterme;

        // Derivee de nor1 par rapport a u1
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns1.Dot(nplan.Crossed(temp)) / norm1 / norm1;
        resul1 = Vector3D.LinearForm(-(grosterme * ndotns1 - nplan.Dot(temp)) / norm1, nplan, grosterme / norm1, ns1,
                             -1.0 / norm1, temp);

        // Derivee par rapport a v1
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns1.Dot(nplan.Crossed(temp)) / norm1 / norm1;
        resul2 = Vector3D.LinearForm(-(grosterme * ndotns1 - nplan.Dot(temp)) / norm1, nplan, grosterme / norm1, ns1,
                             -1.0 / norm1, temp);

        if (_first)
        {
            resul1 = Vector3D.LinearForm(v2d.X, resul1, v2d.Y, resul2);
            d[3, 1] = p1p2.Dot(resul1) - (dpdt.Dot(nor1));
        }
        else
        {
            d[3, 3] = -(d1u1.Dot(nor1)) + p1p2.Dot(resul1);
            d[3, 4] = -(d1v1.Dot(nor1)) + p1p2.Dot(resul2);
        }

        // Derivee de nor2 par rapport a u2
        temp = d2u2.Crossed(d1v2).Added(d1u2.Crossed(d2uv2));
        grosterme = ncrossns2.Dot(nplan.Crossed(temp)) / norm2 / norm2;
        resul1 = Vector3D.LinearForm(-(grosterme * ndotns2 - nplan.Dot(temp)) / norm2, nplan, grosterme / norm2, ns2,
                             -1.0 / norm2, temp);

        // Derivee par rapport a v2
        temp = d2uv2.Crossed(d1v2).Added(d1u2.Crossed(d2v2));
        grosterme = ncrossns2.Dot(nplan.Crossed(temp)) / norm2 / norm2;
        resul2 = Vector3D.LinearForm(-(grosterme * ndotns2 - nplan.Dot(temp)) / norm2, nplan, grosterme / norm2, ns2,
                             -1.0 / norm2, temp);

        if (_first)
        {
            d[4, 3] = d1u2.Dot(nor2) + p1p2.Dot(resul1);
            d[4, 4] = d1v2.Dot(nor2) + p1p2.Dot(resul2);
        }
        else
        {
            resul1 = Vector3D.LinearForm(v2d.X, resul1, v2d.Y, resul2);
            d[4, 1] = p1p2.Dot(resul1) + dpdt.Dot(nor2);
        }

        // derivee par rapport a w (parametre sur ligne guide)
        grosterme = ncrossns1.Dot(dnplan.Crossed(ns1)) / norm1 / norm1;
        resul1 = Vector3D.LinearForm(-(grosterme * ndotns1 - dnplan.Dot(ns1)) / norm1, nplan, ndotns1 / norm1, dnplan,
                             grosterme / norm1, ns1);

        grosterme = ncrossns2.Dot(dnplan.Crossed(ns2)) / norm2 / norm2;
        resul2 = Vector3D.LinearForm(-(grosterme * ndotns2 - dnplan.Dot(ns2)) / norm2, nplan, ndotns2 / norm2, dnplan,
                             grosterme / norm2, ns2);

        d[3, 2] = p1p2.Dot(resul1);
        d[4, 2] = p1p2.Dot(resul2);

        return true;
    }
}
