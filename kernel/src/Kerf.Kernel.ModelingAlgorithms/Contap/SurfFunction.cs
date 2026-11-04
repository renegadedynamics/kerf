// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_SurfFunction.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_SurfFunction.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_SurfFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.HiddenLines.Contours.ContourTFunction;

namespace Kerf.Engine.HiddenLines.Contours;

/// <summary>
/// The contour function on a surface: F(u, v) = N(u, v) . D / mean, the normal's component along the view direction
/// (or towards the eye, or less the draft angle's cosine), scaled by the mean normal length; its zeros are the
/// contour. The function of the contour's walking and interior search.
/// </summary>
/// <remarks>
/// A working object. It implements <see cref="IWFunction{TSelf}"/> (the walking's function) and so
/// <see cref="ISurfFunction{ThePSurface}"/>. With <see cref="DraftPrs"/> OCCT computes nothing and
/// leaves the outputs as they were, as the port does.
/// </remarks>
internal sealed class SurfFunction : FunctionSetWithDerivatives, IWFunction<SurfFunction>
{
    private SurfaceAdaptorBase? _surf;
    private double _mean;
    private ContourTFunction _type;
    private Direction3D _dir;
    private Point3D _eye;
    private double _ang;
    private double _cosAng;
    private double _tol;
    private Point3D _solpt;
    private double _valf;
    private double _usol;
    private double _vsol;
    private double _fpu;
    private double _fpv;
    private Direction2D _d2d;
    private Vector3D _d3d;
    private bool _tangent;
    private bool _computed;
    private bool _derived;

    public SurfFunction()
    {
        _mean = 1.0;
        _type = ContourTFunction.ContourStd;
        _dir = new Direction3D(Direction3D.D.Z);
        _eye = new Point3D();
        _ang = 0.0;
        _cosAng = 0.0; // PI/2 - Angle de depouille
        _tol = 1.0e-6;
        _solpt = new Point3D();
        _valf = 0.0;
        _usol = 0.0;
        _vsol = 0.0;
        _fpu = 0.0;
        _fpv = 0.0;
        _d2d = new Direction2D();
        _d3d = new Vector3D();
        _tangent = false;
        _computed = false;
        _derived = false;
    }

    /// <summary>A copy of <paramref name="other"/> (OCCT's implicit copy constructor; decision 686).</summary>
    public SurfFunction(SurfFunction other)
    {
        _surf = other._surf;
        _mean = other._mean;
        _type = other._type;
        _dir = other._dir;
        _eye = other._eye;
        _ang = other._ang;
        _cosAng = other._cosAng;
        _tol = other._tol;
        _solpt = other._solpt;
        _valf = other._valf;
        _usol = other._usol;
        _vsol = other._vsol;
        _fpu = other._fpu;
        _fpv = other._fpv;
        _d2d = other._d2d;
        _d3d = other._d3d;
        _tangent = other._tangent;
        _computed = other._computed;
        _derived = other._derived;
    }

    static SurfFunction IWFunction<SurfFunction>.Copy(SurfFunction other) =>
        new(other);

    /// <summary>Sets the surface, and the mean length of its normal at its sample points.</summary>
    public void Set(SurfaceAdaptorBase s)
    {
        _surf = s;
        int i;
        int nbs = HContTool.NbSamplePoints(s);
        double u, v;
        Vector3D norm;
        if (nbs > 0)
        {
            _mean = 0.0;
            for (i = 1; i <= nbs; i++)
            {
                HContTool.SamplePoint(s, i, out u, out v);
                //      Adaptor3d_HSurfaceTool::D1(S,U,V,solpt,d1u,d1v);
                //      myMean = myMean + d1u.Crossed(d1v).Magnitude();
                SurfProps.Normale(s, u, v, out _solpt, out norm);
                _mean = _mean + norm.Magnitude;
            }
            _mean = _mean / ((double)nbs);
        }
        _computed = false;
        _derived = false;
    }

    /// <summary>A silhouette for the eye <paramref name="eye"/>.</summary>
    public void Set(in Point3D eye)
    {
        _type = ContourTFunction.ContourPrs; // pers
        _eye = eye;
        _ang = 0.0;
    }

    /// <summary>A silhouette for the view direction <paramref name="direction"/>.</summary>
    public void Set(in Direction3D direction)
    {
        _type = ContourTFunction.ContourStd; // Contour app
        _dir = direction;
        _ang = 0.0;
    }

    /// <summary>A draft contour of angle <paramref name="angle"/> for the direction <paramref name="direction"/>.</summary>
    public void Set(in Direction3D direction, double angle)
    {
        _type = ContourTFunction.DraftStd; // Contour vu
        _dir = direction;
        _ang = angle;
        _cosAng = Math.Cos(M_PI / 2.0 + angle);
    }

    /// <summary>A draft contour of angle <paramref name="angle"/> for the eye <paramref name="eye"/>.</summary>
    public void Set(in Point3D eye, double angle)
    {
        _type = ContourTFunction.DraftPrs; // Contour vu "conique"...
        _eye = eye;
        _ang = angle;
        _cosAng = Math.Cos(M_PI / 2.0 + angle);
    }

    /// <summary>Sets the tolerance (at least 1e-12).</summary>
    public void Set(double tolerance)
    {
        _tol = Max(tolerance, 1.0e-12);
    }

    public override int NbVariables() => 2;

    public override int NbEquations() => 1;

    public override bool Value(math_Vector x, math_Vector f)
    {
        _usol = x[1];
        _vsol = x[2];
        //  Adaptor3d_HSurfaceTool::D1(mySurf,Usol,Vsol,solpt,d1u,d1v);
        //  gp_Vec norm(d1u.Crossed(d1v));
        SurfProps.Normale(_surf!, _usol, _vsol, out _solpt, out Vector3D norm);
        switch (_type)
        {
            case ContourTFunction.ContourStd:
                f[1] = _valf = (norm.Dot(new Vector3D(_dir))) / _mean;
                break;
            case ContourTFunction.ContourPrs:
                f[1] = _valf = (norm.Dot(new Vector3D(_eye, _solpt))) / _mean;
                break;
            case ContourTFunction.DraftStd:
                f[1] = _valf = (norm.Dot(new Vector3D(_dir)) - _cosAng * norm.Magnitude) / _mean;
                break;
            default:
                break;
        }
        _computed = false;
        _derived = false;
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix grad)
    {
        //  gp_Vec d2u,d2v,d2uv;
        _usol = x[1];
        _vsol = x[2];
        //  Adaptor3d_HSurfaceTool::D2(mySurf,Usol,Vsol,solpt,d1u,d1v,d2u,d2v,d2uv);

        SurfProps.NormAndDn(_surf!, _usol, _vsol, out _solpt, out Vector3D norm, out Vector3D dnu, out Vector3D dnv);

        switch (_type)
        {
            case ContourTFunction.ContourStd:
                grad[1, 1] = (dnu.Dot(new Vector3D(_dir))) / _mean;
                grad[1, 2] = (dnv.Dot(new Vector3D(_dir))) / _mean;
                break;
            case ContourTFunction.ContourPrs:
            {
                Vector3D ep = new(_eye, _solpt);
                grad[1, 1] = (dnu.Dot(ep)) / _mean;
                grad[1, 2] = (dnv.Dot(ep)) / _mean;
            }
            break;
            case ContourTFunction.DraftStd:
                norm = norm.Normalized();
                grad[1, 1] = (dnu.Dot(new Vector3D(_dir)) - _cosAng * dnu.Dot(norm)) / _mean;
                grad[1, 2] = (dnv.Dot(new Vector3D(_dir)) - _cosAng * dnv.Dot(norm)) / _mean;
                break;
            case ContourTFunction.DraftPrs:
            default:
                break;
        }
        _fpu = grad[1, 1];
        _fpv = grad[1, 2];
        _computed = false;
        _derived = true;
        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix grad)
    {
        //  gp_Vec d2u,d2v,d2uv;

        _usol = x[1];
        _vsol = x[2];
        //  Adaptor3d_HSurfaceTool::D2(mySurf,Usol,Vsol,solpt,d1u,d1v,d2u,d2v,d2uv);
        //  gp_Vec norm(d1u.Crossed(d1v));
        SurfProps.NormAndDn(_surf!, _usol, _vsol, out _solpt, out Vector3D norm, out Vector3D dnu, out Vector3D dnv);

        switch (_type)
        {
            case ContourTFunction.ContourStd:
                f[1] = (norm.Dot(new Vector3D(_dir))) / _mean;
                grad[1, 1] = (dnu.Dot(new Vector3D(_dir))) / _mean;
                grad[1, 2] = (dnv.Dot(new Vector3D(_dir))) / _mean;
                break;
            case ContourTFunction.ContourPrs:
            {
                Vector3D ep = new(_eye, _solpt);
                f[1] = (norm.Dot(ep)) / _mean;
                grad[1, 1] = (dnu.Dot(ep)) / _mean;
                grad[1, 2] = (dnv.Dot(ep)) / _mean;
            }
            break;
            case ContourTFunction.DraftStd:
                f[1] = (norm.Dot(new Vector3D(_dir)) - _cosAng * norm.Magnitude) / _mean;
                norm = norm.Normalized();
                grad[1, 1] = (dnu.Dot(new Vector3D(_dir)) - _cosAng * dnu.Dot(norm)) / _mean;
                grad[1, 2] = (dnv.Dot(new Vector3D(_dir)) - _cosAng * dnv.Dot(norm)) / _mean;
                break;
            case ContourTFunction.DraftPrs:
            default:
                break;
        }
        _valf = f[1];
        _fpu = grad[1, 1];
        _fpv = grad[1, 2];
        _computed = false;
        _derived = true;
        return true;
    }

    /// <summary>The value of the function at the last evaluated point.</summary>
    public double Root() => _valf;

    /// <summary>The tolerance below which the function is null.</summary>
    public double Tolerance() => _tol;

    /// <summary>The last evaluated point.</summary>
    public Point3D Point() => _solpt;

    /// <summary>
    /// True if the contour is singular (the gradient vanishes) at the last evaluated point; otherwise computes its
    /// tangent (<see cref="Direction3d"/>, <see cref="Direction2d"/>).
    /// </summary>
    public bool IsTangent()
    {
        if (!_computed)
        {
            _computed = true;
            if (!_derived)
            {
                //      gp_Vec d2u,d2v,d2uv;
                //      Adaptor3d_HSurfaceTool::D2(mySurf, Usol, Vsol, solpt, d1u, d1v, d2u, d2v, d2uv);
                SurfProps.NormAndDn(_surf!, _usol, _vsol, out _solpt, out Vector3D norm, out Vector3D dnu,
                                           out Vector3D dnv);

                switch (_type)
                {
                    case ContourTFunction.ContourStd:
                        _fpu = (dnu.Dot(new Vector3D(_dir))) / _mean;
                        _fpv = (dnv.Dot(new Vector3D(_dir))) / _mean;
                        break;
                    case ContourTFunction.ContourPrs:
                    {
                        Vector3D ep = new(_eye, _solpt);
                        _fpu = (dnu.Dot(ep)) / _mean;
                        _fpv = (dnv.Dot(ep)) / _mean;
                    }
                    break;
                    case ContourTFunction.DraftStd:
                        norm = norm.Normalized();
                        _fpu = (dnu.Dot(new Vector3D(_dir)) - _cosAng * dnu.Dot(norm)) / _mean;
                        _fpv = (dnv.Dot(new Vector3D(_dir)) - _cosAng * dnv.Dot(norm)) / _mean;
                        break;
                    case ContourTFunction.DraftPrs:
                    default:
                        break;
                }
                _derived = true;
            }
            _tangent = false;
            double d = Math.Sqrt(_fpu * _fpu + _fpv * _fpv);

            if (d <= Constants.Resolution())
            {
                _tangent = true;
            }
            else
            {
                _d2d = new Direction2D(-_fpv, _fpu);
                HSurfaceTool.D1(_surf!, _usol, _vsol, out _solpt, out Vector3D d1u, out Vector3D d1v); // ajout jag 02.95

                Xyz d3dxyz = -_fpv * d1u.XYZ();
                d3dxyz.Add(_fpu * d1v.XYZ());
                _d3d = _d3d.WithXYZ(d3dxyz);

                // jag 940616    if (d3d.Magnitude() <= Tolpetit) {
                if (_d3d.Magnitude <= _tol)
                {
                    _tangent = true;
                }
            }
        }
        return _tangent;
    }

    /// <summary>The tangent of the contour in 3D. Throws StdFail_UndefinedDerivative at a singular point.</summary>
    public Vector3D Direction3d()
    {
        if (IsTangent())
        {
            throw new UndefinedDerivativeException();
        }
        return _d3d;
    }

    /// <summary>The tangent in the parametric space. Throws StdFail_UndefinedDerivative at a singular point.</summary>
    public Direction2D Direction2d()
    {
        if (IsTangent())
        {
            throw new UndefinedDerivativeException();
        }
        return _d2d;
    }

    public ContourTFunction FunctionType() => _type;

    public Point3D Eye() => _eye;

    public Direction3D Direction() => _dir;

    public double Angle() => _ang;

    public SurfaceAdaptorBase Surface() => _surf!;

    /// <summary>The surface (the walking's name for it).</summary>
    public SurfaceAdaptorBase PSurface() => Surface();
}
