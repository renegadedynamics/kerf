// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_EvolRadInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_EvolRadInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>
/// The evolving-radius fillet function with one end on a curve on one of the surfaces (a restriction): its
/// variables are the parameter on the curve on surface, the guide's parameter, and (U, V) on the other surface.
/// </summary>
public class EvolRadInv : FuncInv
{
    private const double Eps = 1.0e-15;

    private readonly SurfaceAdaptorBase _surf1;
    private readonly SurfaceAdaptorBase _surf2;
    private readonly CurveAdaptorBase _curv;
    private Curve2DAdaptorBase? _csurf;
    private readonly LawsFunction _fevol;
    private double _sg1;
    private double _sg2;
    private int _choix;
    private bool _first;

    /// <remarks>OCCT leaves the "quadrant" and the side uninitialised; here they are zero.</remarks>
    public EvolRadInv(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, CurveAdaptorBase c, LawsFunction law)
    {
        _surf1 = s1;
        _surf2 = s2;
        _curv = c;
        _fevol = law;
    }

    /// <summary>Sets the "quadrant" <paramref name="choix"/>, as BlendFunc_EvolRad.</summary>
    public void Set(int choix)
    {
        _choix = choix;
        switch (_choix)
        {
            case 1:
            case 2:
                {
                    _sg1 = -1.0;
                    _sg2 = -1.0;
                }
                break;
            case 3:
            case 4:
                {
                    _sg1 = 1.0;
                    _sg2 = -1.0;
                }
                break;
            case 5:
            case 6:
                {
                    _sg1 = 1.0;
                    _sg2 = 1.0;
                }
                break;
            case 7:
            case 8:
                {
                    _sg1 = -1.0;
                    _sg2 = 1.0;
                }
                break;
            default:
                _sg1 = _sg2 = -1.0;
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
               && (valsol[2] * valsol[2] + valsol[3] * valsol[3] + valsol[4] * valsol[4]) <= tol * tol;

    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        double ray = _fevol.Value(x[2]);

        _curv.D1(x[2], out Point3D ptcur, out Vector3D d1cur);
        Vector3D nplan = d1cur.Normalized();
        double theD = -(nplan.XYZ().Dot(ptcur.XYZ()));
        Point2D pt2d = _csurf!.Value(x[1]);

        Point3D pts1, pts2;
        Vector3D d1u1, d1v1, d1u2, d1v2;
        if (_first)
        {
            _surf1.D1(pt2d.X(), pt2d.Y(), out pts1, out d1u1, out d1v1);
            _surf2.D1(x[3], x[4], out pts2, out d1u2, out d1v2);
        }
        else
        {
            _surf1.D1(x[3], x[4], out pts1, out d1u1, out d1v1);
            _surf2.D1(pt2d.X(), pt2d.Y(), out pts2, out d1u2, out d1v2);
        }

        f[1] = (nplan.X() * (pts1.X() + pts2.X()) + nplan.Y() * (pts1.Y() + pts2.Y())
                + nplan.Z() * (pts1.Z() + pts2.Z()))
                 / 2.0
               + theD;

        Vector3D ns1 = d1u1.Crossed(d1v1);
        if (ns1.Magnitude() < Eps)
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

        Vector3D ns2 = new Vector3D(d1u2.Crossed(d1v2).XYZ());
        if (ns2.Magnitude() < Eps)
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

        Vector3D ncrossns1 = nplan.Crossed(ns1);
        Vector3D ncrossns2 = nplan.Crossed(ns2);
        double norm1 = ncrossns1.Magnitude();
        double norm2 = ncrossns2.Magnitude();

        if (norm1 < Eps)
        {
            norm1 = 1.0;
        }

        if (norm2 < Eps)
        {
            norm2 = 1.0;
        }

        Vector3D resul = new Vector3D();
        double ndotns1 = nplan.Dot(ns1);
        double ndotns2 = nplan.Dot(ns2);
        ns1.SetLinearForm(ndotns1 / norm1, nplan, -1.0 / norm1, ns1);
        ns2.SetLinearForm(ndotns2 / norm2, nplan, -1.0 / norm2, ns2);
        resul.SetLinearForm(_sg1 * ray, ns1, -1.0, new Vector3D(pts2.XYZ()), -_sg2 * ray, ns2, new Vector3D(pts1.XYZ()));
        f[2] = resul.X();
        f[3] = resul.Y();
        f[4] = resul.Z();
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        math_Vector f = new(1, 4);
        return Values(x, f, d);
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        _fevol.D1(x[2], out double ray, out double dray);

        _curv.D2(x[2], out Point3D ptcur, out Vector3D d1cur, out Vector3D d2cur);
        double normtgcur = d1cur.Magnitude();
        Vector3D nplan = d1cur.Normalized();
        double theD = -(nplan.XYZ().Dot(ptcur.XYZ()));

        Vector3D dnplan = new Vector3D();
        dnplan.SetLinearForm(-nplan.Dot(d2cur), nplan, d2cur);
        dnplan /= normtgcur;

        _csurf!.D1(x[1], out Point2D p2d, out Vector2D v2d);

        Point3D pts1, pts2;
        Vector3D d1u1, d1v1, d1u2, d1v2;
        Vector3D d2u1, d2v1, d2u2, d2v2, d2uv1, d2uv2;
        Vector3D temp = new Vector3D();
        if (_first)
        {
            _surf1.D2(p2d.X(), p2d.Y(), out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            _surf2.D2(x[3], x[4], out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
            temp.SetLinearForm(v2d.X(), d1u1, v2d.Y(), d1v1);
            d[1, 1] = nplan.Dot(temp) / 2.0;
            temp.SetXYZ(0.5 * (pts1.XYZ() + pts2.XYZ()) - ptcur.XYZ());
            d[1, 2] = dnplan.Dot(temp) - normtgcur;
            d[1, 3] = nplan.Dot(d1u2) / 2.0;
            d[1, 4] = nplan.Dot(d1v2) / 2.0;
        }
        else
        {
            _surf1.D2(x[3], x[4], out pts1, out d1u1, out d1v1, out d2u1, out d2v1, out d2uv1);
            _surf2.D2(p2d.X(), p2d.Y(), out pts2, out d1u2, out d1v2, out d2u2, out d2v2, out d2uv2);
            temp.SetLinearForm(v2d.X(), d1u2, v2d.Y(), d1v2);
            d[1, 1] = nplan.Dot(temp) / 2.0;
            temp.SetXYZ(0.5 * (pts1.XYZ() + pts2.XYZ()) - ptcur.XYZ());
            d[1, 2] = dnplan.Dot(temp) - normtgcur;
            d[1, 3] = nplan.Dot(d1u1) / 2.0;
            d[1, 4] = nplan.Dot(d1v1) / 2.0;
        }

        f[1] = (nplan.X() * (pts1.X() + pts2.X()) + nplan.Y() * (pts1.Y() + pts2.Y())
                + nplan.Z() * (pts1.Z() + pts2.Z()))
                 / 2.0
               + theD;

        Vector3D ns1 = d1u1.Crossed(d1v1);
        if (ns1.Magnitude() < Eps)
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

        Vector3D ns2 = d1u2.Crossed(d1v2);
        if (ns2.Magnitude() < Eps)
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

        Vector3D ncrossns1 = nplan.Crossed(ns1);
        Vector3D ncrossns2 = nplan.Crossed(ns2);
        double norm1 = ncrossns1.Magnitude();
        double norm2 = ncrossns2.Magnitude();
        if (norm1 < Eps)
        {
            norm1 = 1.0;
        }

        if (norm2 < Eps)
        {
            norm2 = 1.0;
        }

        Vector3D resul1 = new Vector3D(), resul2 = new Vector3D();
        double grosterme;
        double ndotns1 = nplan.Dot(ns1);
        double ndotns2 = nplan.Dot(ns2);

        temp.SetLinearForm(ndotns1 / norm1, nplan, -1.0 / norm1, ns1);
        resul1.SetLinearForm(_sg1 * ray, temp, new Vector3D(pts2, pts1));
        temp.SetLinearForm(ndotns2 / norm2, nplan, -1.0 / norm2, ns2);
        resul1.Subtract(_sg2 * ray * temp);

        f[2] = resul1.X();
        f[3] = resul1.Y();
        f[4] = resul1.Z();

        // Derivee par rapport a u1
        temp = d2u1.Crossed(d1v1).Added(d1u1.Crossed(d2uv1));
        grosterme = ncrossns1.Dot(nplan.Crossed(temp)) / norm1 / norm1;
        resul1.SetLinearForm(-_sg1 * ray / norm1 * (grosterme * ndotns1 - nplan.Dot(temp)), nplan,
                             _sg1 * ray * grosterme / norm1, ns1, -_sg1 * ray / norm1, temp, d1u1);

        // Derivee par rapport a v1
        temp = d2uv1.Crossed(d1v1).Added(d1u1.Crossed(d2v1));
        grosterme = ncrossns1.Dot(nplan.Crossed(temp)) / norm1 / norm1;
        resul2.SetLinearForm(-_sg1 * ray / norm1 * (grosterme * ndotns1 - nplan.Dot(temp)), nplan,
                             _sg1 * ray * grosterme / norm1, ns1, -_sg1 * ray / norm1, temp, d1v1);

        if (_first)
        {
            d[2, 1] = v2d.X() * resul1.X() + v2d.Y() * resul2.X();
            d[3, 1] = v2d.X() * resul1.Y() + v2d.Y() * resul2.Y();
            d[4, 1] = v2d.X() * resul1.Z() + v2d.Y() * resul2.Z();
        }
        else
        {
            d[2, 3] = resul1.X();
            d[3, 3] = resul1.Y();
            d[4, 3] = resul1.Z();
            d[2, 4] = resul2.X();
            d[3, 4] = resul2.Y();
            d[4, 4] = resul2.Z();
        }

        // derivee par rapport a w (parametre sur ligne guide)
        grosterme = ncrossns1.Dot(dnplan.Crossed(ns1)) / norm1 / norm1;
        resul1.SetLinearForm(-_sg1 / norm1 * (grosterme * ndotns1 - dnplan.Dot(ns1)), nplan, _sg1 * ndotns1 / norm1,
                             dnplan, _sg1 * grosterme / norm1, ns1);

        grosterme = ncrossns2.Dot(dnplan.Crossed(ns2)) / norm2 / norm2;
        resul2.SetLinearForm(_sg2 / norm2 * (grosterme * ndotns2 - dnplan.Dot(ns2)), nplan, -_sg2 * ndotns2 / norm2,
                             dnplan, -_sg2 * grosterme / norm2, ns2);

        d[2, 2] = ray * (resul1.X() + resul2.X());
        d[3, 2] = ray * (resul1.Y() + resul2.Y());
        d[4, 2] = ray * (resul1.Z() + resul2.Z());

        temp.SetLinearForm(_sg1 * ndotns1 / norm1 - _sg2 * ndotns2 / norm2, nplan, -_sg1 / norm1, ns1, _sg2 / norm2,
                           ns2);

        d[2, 2] += dray * temp.X();
        d[3, 2] += dray * temp.Y();
        d[4, 2] += dray * temp.Z();

        // Derivee par rapport a u2
        temp = d2u2.Crossed(d1v2).Added(d1u2.Crossed(d2uv2));
        grosterme = ncrossns2.Dot(nplan.Crossed(temp)) / norm2 / norm2;
        resul1.SetLinearForm(_sg2 * ray / norm2 * (grosterme * ndotns2 - nplan.Dot(temp)), nplan,
                             -_sg2 * ray * grosterme / norm2, ns2, _sg2 * ray / norm2, temp);
        resul1.Subtract(d1u2);

        // Derivee par rapport a v2
        temp = d2uv2.Crossed(d1v2).Added(d1u2.Crossed(d2v2));
        grosterme = ncrossns2.Dot(nplan.Crossed(temp)) / norm2 / norm2;
        resul2.SetLinearForm(_sg2 * ray / norm2 * (grosterme * ndotns2 - nplan.Dot(temp)), nplan,
                             -_sg2 * ray * grosterme / norm2, ns2, _sg2 * ray / norm2, temp);
        resul2.Subtract(d1v2);

        if (!_first)
        {
            d[2, 1] = v2d.X() * resul1.X() + v2d.Y() * resul2.X();
            d[3, 1] = v2d.X() * resul1.Y() + v2d.Y() * resul2.Y();
            d[4, 1] = v2d.X() * resul1.Z() + v2d.Y() * resul2.Z();
        }
        else
        {
            d[2, 3] = resul1.X();
            d[3, 3] = resul1.Y();
            d[4, 3] = resul1.Z();
            d[2, 4] = resul2.X();
            d[3, 4] = resul2.Y();
            d[4, 4] = resul2.Z();
        }

        return true;
    }
}
