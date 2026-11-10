// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstRadInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstRadInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>
/// The constant-radius fillet function with one end on a curve on one of the surfaces (a restriction): its
/// variables are the parameter on the curve on surface, the guide's parameter, and (U, V) on the other surface.
/// </summary>
internal class ConstRadInv : FuncInv
{
    private const double Eps = 1.0e-15;

    private readonly SurfaceAdaptorBase _surf1;
    private readonly SurfaceAdaptorBase _surf2;
    private readonly CurveAdaptorBase _curv;
    private Curve2DAdaptorBase? _csurf;
    private double _ray1;
    private double _ray2;
    private int _choix;
    private bool _first;

    public ConstRadInv(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, CurveAdaptorBase c)
    {
        _surf1 = s1;
        _surf2 = s2;
        _curv = c;
        _ray1 = 0.0;
        _ray2 = 0.0;
        _choix = 0;
        _first = false;
    }

    /// <summary>Sets the radius and the "quadrant" <paramref name="choix"/>, as BlendFunc_ConstRad.</summary>
    public void Set(double r, int choix)
    {
        _choix = choix;
        switch (_choix)
        {
            case 1:
            case 2:
                {
                    _ray1 = -r;
                    _ray2 = -r;
                }
                break;
            case 3:
            case 4:
                {
                    _ray1 = r;
                    _ray2 = -r;
                }
                break;
            case 5:
            case 6:
                {
                    _ray1 = r;
                    _ray2 = r;
                }
                break;
            case 7:
            case 8:
                {
                    _ray1 = -r;
                    _ray2 = r;
                }
                break;
            default:
                _ray1 = _ray2 = -r;
                break;
        }
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
        return Math.Abs(valsol[1]) <= tol
               && valsol[2] * valsol[2] + valsol[3] * valsol[3] + valsol[4] * valsol[4] <= tol * tol;
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        _curv.D1(x[2], out Point3D ptcur, out Vector3D d1cur);
        Vector3D nplan = d1cur.Normalized();
        double theD = -(nplan.XYZ().Dot(ptcur.XYZ()));
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

        f[1] = (nplan.X * (pts1.X + pts2.X) + nplan.Y * (pts1.Y + pts2.Y)
                + nplan.Z * (pts1.Z + pts2.Z))
                 / 2.0
               + theD;

        Vector3D ns1 = d1u1.Crossed(d1v1);
        if (ns1.Magnitude < Eps)
        {
            if (_first)
            {
                global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(_surf1, pt2d, ref ns1);
            }
            else
            {
                Point2D p = new Point2D(x[3], x[4]);
                global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(_surf1, p, ref ns1);
            }
        }

        Vector3D ns2 = d1u2.Crossed(d1v2);
        if (ns2.Magnitude < Eps)
        {
            if (!_first)
            {
                global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(_surf2, pt2d, ref ns2);
            }
            else
            {
                Point2D p = new Point2D(x[3], x[4]);
                global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(_surf2, p, ref ns2);
            }
        }

        // The surfaces' own normals, as BlendFunc_ConstRad's (decision 30001).
        Vector3D resul = new Vector3D();
        ns1 = -ConstRad.InverseNorm(ns1) * ns1;
        ns2 = -ConstRad.InverseNorm(ns2) * ns2;
        resul = Vector3D.LinearForm(_ray1, ns1, -1.0, new Vector3D(pts2.XYZ()), -_ray2, ns2, new Vector3D(pts1.XYZ()));
        f[2] = resul.X;
        f[3] = resul.Y;
        f[4] = resul.Z;
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        Vector3D d1u1, d1v1, d1u2, d1v2;
        Vector3D d2u1, d2v1, d2uv1, d2u2, d2v2, d2uv2;
        Vector3D ns1, ns2, nplan, dnplan = new Vector3D(), temp = new Vector3D();
        Point3D pts1, pts2;
        double invnorm1, invnorm2, normtgcur;

        _curv.D2(x[2], out Point3D ptcur, out Vector3D d1cur, out Vector3D d2cur);
        normtgcur = d1cur.Magnitude;
        nplan = d1cur.Normalized();

        // OCCT has the plane's offset where Values has -nplan.Dot(d2cur) (decision 1038); Kerf's agree since the
        // function no longer matches native's (decision 30001).
        dnplan = Vector3D.LinearForm(-nplan.Dot(d2cur), nplan, d2cur);
        dnplan /= normtgcur;

        _csurf!.D1(x[1], out Point2D p2d, out Vector2D v2d);

        if (_first)
        {
            _surf1.D2(p2d.X, p2d.Y, out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            _surf2.D2(x[3], x[4], out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
            temp = Vector3D.LinearForm(v2d.X, d1u1, v2d.Y, d1v1);
            d[1, 1] = nplan.Dot(temp) / 2.0;
            temp = temp.WithXYZ(0.5 * (pts1.XYZ() + pts2.XYZ()) - ptcur.XYZ());
            d[1, 2] = dnplan.Dot(temp) - normtgcur;
            d[1, 3] = nplan.Dot(d1u2) / 2.0;
            d[1, 4] = nplan.Dot(d1v2) / 2.0;
        }
        else
        {
            _surf1.D2(x[3], x[4], out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            _surf2.D2(p2d.X, p2d.Y, out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
            temp = Vector3D.LinearForm(v2d.X, d1u2, v2d.Y, d1v2);
            d[1, 1] = nplan.Dot(temp) / 2.0;
            temp = temp.WithXYZ(0.5 * (pts1.XYZ() + pts2.XYZ()) - ptcur.XYZ());
            d[1, 2] = dnplan.Dot(temp) - normtgcur;
            d[1, 3] = nplan.Dot(d1u1) / 2.0;
            d[1, 4] = nplan.Dot(d1v1) / 2.0;
        }

        ns1 = d1u1.Crossed(d1v1);
        if (ns1.Magnitude < Eps)
        {
            if (_first)
            {
                global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(_surf1, p2d, ref ns1);
            }
            else
            {
                Point2D p = new Point2D(x[3], x[4]);
                global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(_surf1, p, ref ns1);
            }
        }

        ns2 = d1u2.Crossed(d1v2);
        if (ns2.Magnitude < Eps)
        {
            if (!_first)
            {
                global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(_surf2, p2d, ref ns2);
            }
            else
            {
                Point2D p = new Point2D(x[3], x[4]);
                global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(_surf2, p, ref ns2);
            }
        }

        // The surfaces' own normals, as BlendFunc_ConstRad's (decision 30001).
        invnorm1 = ConstRad.InverseNorm(ns1);
        invnorm2 = ConstRad.InverseNorm(ns2);

        DerivativesTail(d, d1u1, d1v1, d1u2, d1v2, d2u1, d2v1, d2uv1, d2u2, d2v2, d2uv2, ns1, ns2, v2d, invnorm1, invnorm2);
        return true;
    }

    // The derivatives in the 2nd to 4th rows, which Derivatives and Values compute with the same code: the centre
    // pts1 - ray1 * ns1 / |ns1| = pts2 - ray2 * ns2 / |ns2|, with invnorm1 = 1 / |ns1| and invnorm2 = 1 / |ns2|. The
    // surfaces' own normals don't depend on the guide (decision 30001).
    private void DerivativesTail(Matrix d, in Vector3D d1u1, in Vector3D d1v1, in Vector3D d1u2,
                                 in Vector3D d1v2, in Vector3D d2u1, in Vector3D d2v1, in Vector3D d2uv1, in Vector3D d2u2,
                                 in Vector3D d2v2, in Vector3D d2uv2, in Vector3D ns1, in Vector3D ns2, in Vector2D v2d,
                                 double invnorm1, double invnorm2)
    {
        Vector3D resul1, resul2;
        Vector3D nn1 = invnorm1 * ns1, nn2 = invnorm2 * ns2;

        // Derived compared to u1
        resul1 = d1u1 - _ray1 * ConstRad.UnitNormalD1(nn1, invnorm1, d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1)));

        // Derived compared to v1
        resul2 = d1v1 - _ray1 * ConstRad.UnitNormalD1(nn1, invnorm1, d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1)));

        if (_first)
        {
            d[2, 1] = v2d.X * resul1.X + v2d.Y * resul2.X;
            d[3, 1] = v2d.X * resul1.Y + v2d.Y * resul2.Y;
            d[4, 1] = v2d.X * resul1.Z + v2d.Y * resul2.Z;
        }
        else
        {
            d[2, 3] = resul1.X;
            d[3, 3] = resul1.Y;
            d[4, 3] = resul1.Z;
            d[2, 4] = resul2.X;
            d[3, 4] = resul2.Y;
            d[4, 4] = resul2.Z;
        }

        // derived compared to w (parameter on guideline): the normals don't move with it
        d[2, 2] = 0;
        d[3, 2] = 0;
        d[4, 2] = 0;

        // Derived compared to u2
        resul1 = _ray2 * ConstRad.UnitNormalD1(nn2, invnorm2, d2u2.Crossed(d1v2).Added(d1u2.Crossed(d2uv2))) - d1u2;

        // Derived compared to v2
        resul2 = _ray2 * ConstRad.UnitNormalD1(nn2, invnorm2, d2uv2.Crossed(d1v2).Added(d1u2.Crossed(d2v2))) - d1v2;

        if (!_first)
        {
            d[2, 1] = v2d.X * resul1.X + v2d.Y * resul2.X;
            d[3, 1] = v2d.X * resul1.Y + v2d.Y * resul2.Y;
            d[4, 1] = v2d.X * resul1.Z + v2d.Y * resul2.Z;
        }
        else
        {
            d[2, 3] = resul1.X;
            d[3, 3] = resul1.Y;
            d[4, 3] = resul1.Z;
            d[2, 4] = resul2.X;
            d[3, 4] = resul2.Y;
            d[4, 4] = resul2.Z;
        }
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        Vector3D d1u1, d1v1, d1u2, d1v2;
        Vector3D d2u1, d2v1, d2uv1, d2u2, d2v2, d2uv2;
        Vector3D ns1, ns2, nplan, dnplan = new Vector3D(), resul1 = new Vector3D(),
               temp = new Vector3D();
        Point3D pts1, pts2;
        double invnorm1, invnorm2, normtgcur;
        double theD;

        _curv.D2(x[2], out Point3D ptcur, out Vector3D d1cur, out Vector3D d2cur);
        normtgcur = d1cur.Magnitude;
        nplan = d1cur.Normalized();
        theD = -(nplan.XYZ().Dot(ptcur.XYZ()));
        dnplan = Vector3D.LinearForm(-nplan.Dot(d2cur), nplan, d2cur);
        dnplan /= normtgcur;

        _csurf!.D1(x[1], out Point2D p2d, out Vector2D v2d);

        if (_first)
        {
            _surf1.D2(p2d.X, p2d.Y, out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            _surf2.D2(x[3], x[4], out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
            temp = Vector3D.LinearForm(v2d.X, d1u1, v2d.Y, d1v1);
            d[1, 1] = nplan.Dot(temp) / 2.0;
            temp = temp.WithXYZ(0.5 * (pts1.XYZ() + pts2.XYZ()) - ptcur.XYZ());
            d[1, 2] = dnplan.Dot(temp) - normtgcur;
            d[1, 3] = nplan.Dot(d1u2) / 2.0;
            d[1, 4] = nplan.Dot(d1v2) / 2.0;
        }
        else
        {
            _surf1.D2(x[3], x[4], out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            _surf2.D2(p2d.X, p2d.Y, out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
            temp = Vector3D.LinearForm(v2d.X, d1u2, v2d.Y, d1v2);
            d[1, 1] = nplan.Dot(temp) / 2.0;
            temp = temp.WithXYZ(0.5 * (pts1.XYZ() + pts2.XYZ()) - ptcur.XYZ());
            d[1, 2] = dnplan.Dot(temp) - normtgcur;
            d[1, 3] = nplan.Dot(d1u1) / 2.0;
            d[1, 4] = nplan.Dot(d1v1) / 2.0;
        }

        f[1] = (nplan.X * (pts1.X + pts2.X) + nplan.Y * (pts1.Y + pts2.Y)
                + nplan.Z * (pts1.Z + pts2.Z))
                 / 2.0
               + theD;

        ns1 = d1u1.Crossed(d1v1);
        if (ns1.Magnitude < Eps)
        {
            if (_first)
            {
                global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(_surf1, p2d, ref ns1);
            }
            else
            {
                Point2D p = new Point2D(x[3], x[4]);
                global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(_surf1, p, ref ns1);
            }
        }

        ns2 = d1u2.Crossed(d1v2);
        if (ns2.Magnitude < Eps)
        {
            if (!_first)
            {
                global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(_surf2, p2d, ref ns2);
            }
            else
            {
                Point2D p = new Point2D(x[3], x[4]);
                global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(_surf2, p, ref ns2);
            }
        }

        // The surfaces' own normals, as BlendFunc_ConstRad's (decision 30001).
        invnorm1 = ConstRad.InverseNorm(ns1);
        invnorm2 = ConstRad.InverseNorm(ns2);

        resul1 = Vector3D.LinearForm(-_ray1 * invnorm1, ns1, new Vector3D(pts2, pts1));
        resul1 = resul1.Added(_ray2 * invnorm2 * ns2);

        f[2] = resul1.X;
        f[3] = resul1.Y;
        f[4] = resul1.Z;

        DerivativesTail(d, d1u1, d1v1, d1u2, d1v2, d2u1, d2v1, d2uv1, d2u2, d2v2, d2uv2, ns1, ns2, v2d, invnorm1, invnorm2);
        return true;
    }
}
