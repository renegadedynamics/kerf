// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ZerImpFunc.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ZerImpFunc.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Intersection.Implicit;

/// <summary>
/// The function whose zeros are the points of a parametric surface S(u, v) on an implicit surface (a quadric):
/// F(u, v) = Q(S(u, v)), with its gradient, for math_FunctionSetRoot; and, at a root, the tangent of the
/// intersection line in 3D and in the parametric space.
/// </summary>
/// <remarks>
/// OCCT's generic file IntImp_ZerImpFunc.gxx, ported once as a generic class over the parametric surface and its tool
/// (decisions 202, 337 and 607). OCCT's macro TheISurface is IntSurf_Quadric and TheISurfaceTool IntSurf_QuadricTool
/// in every instantiation (IntPatch_TheSurfFunction, GeomInt's and BRepApprox's TheZerImpFunc...), so the body names
/// them. OCCT's instantiations are sealed subclasses with the constructors forwarded.
/// <para>
/// OCCT keeps pointers to the caller's surface handle and quadric. The port keeps the surface (an adaptor, by
/// reference, as the handle) and a copy of the quadric, a struct: every OCCT caller builds the quadric before
/// <see cref="SetImplicitSurface"/> and never changes it afterwards (decision 608).
/// </para>
/// </remarks>
internal class ZerImpFunc<ThePSurface, ThePSurfaceTool> : FunctionSetWithDerivatives
    where ThePSurfaceTool : IZerCSParPSurfaceTool<ThePSurface>
{
    private const double EpsAng2 = 1.0e-16;
    private const double Tolpetit = 1.0e-16;

    private ThePSurface _surf;
    private SurfaceQuadric _func;
    private double _u;
    private double _v;
    private double _tol;
    private Point3D _pntsol;
    private double _valf;
    private bool _computed;
    private bool _tangent;
    private double _tgdu;
    private double _tgdv;
    private Vector3D _gradient;
    private bool _derived;
    private Vector3D _d1u;
    private Vector3D _d1v;
    private Vector3D _d3d;
    private Direction2D _d2d;

    /// <summary>An empty function; <see cref="Set(ThePSurface)"/> and <see cref="SetImplicitSurface"/> load it.</summary>
    public ZerImpFunc()
    {
        _surf = default!;
        _func = new SurfaceQuadric();
        _u = 0.0;
        _v = 0.0;
        _tol = 0.0;
        _pntsol = new Point3D();
        _valf = 0.0;
        _computed = false;
        _tangent = false;
        _tgdu = 0.0;
        _tgdv = 0.0;
        _gradient = new Vector3D();
        _derived = false;
        _d1u = new Vector3D();
        _d1v = new Vector3D();
        _d3d = new Vector3D();
        _d2d = new Direction2D();
    }

    public ZerImpFunc(ThePSurface ps, in SurfaceQuadric IS)
        : this()
    {
        _surf = ps;
        _func = IS;
    }

    public ZerImpFunc(in SurfaceQuadric IS)
        : this()
    {
        _func = IS;
    }

    /// <summary>
    /// A copy of <paramref name="other"/>, as OCCT's implicit copy constructor: the same surface (a reference) and
    /// a copy of the quadric, with the state of the last evaluation (IntWalk_IWalking's duplicate of its function).
    /// </summary>
    protected ZerImpFunc(ZerImpFunc<ThePSurface, ThePSurfaceTool> other)
    {
        _surf = other._surf;
        _func = other._func;
        _u = other._u;
        _v = other._v;
        _tol = other._tol;
        _pntsol = other._pntsol;
        _valf = other._valf;
        _computed = other._computed;
        _tangent = other._tangent;
        _tgdu = other._tgdu;
        _tgdv = other._tgdv;
        _gradient = other._gradient;
        _derived = other._derived;
        _d1u = other._d1u;
        _d1v = other._d1v;
        _d3d = other._d3d;
        _d2d = other._d2d;
    }

    public void Set(ThePSurface ps) => _surf = ps;

    public void SetImplicitSurface(in SurfaceQuadric IS) => _func = IS;

    public void Set(double tol) => _tol = tol;

    public override int NbVariables() => 2;

    public override int NbEquations() => 1;

    public override bool Value(math_Vector x, math_Vector f)
    {
        _u = x[1];
        _v = x[2];
        _pntsol = ThePSurfaceTool.Value(_surf, _u, _v);
        _valf = QuadricTool.Value(_func, _pntsol.X, _pntsol.Y, _pntsol.Z);
        f[1] = _valf;
        _computed = false;
        _derived = false;
        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        _u = x[1];
        _v = x[2];
        ThePSurfaceTool.D1(_surf, _u, _v, out _pntsol, out _d1u, out _d1v);
        QuadricTool.Gradient(_func, _pntsol.X, _pntsol.Y, _pntsol.Z, out _gradient);
        d[1, 1] = _d1u.Dot(_gradient);
        d[1, 2] = _d1v.Dot(_gradient);
        _computed = false;
        _derived = true;
        return true;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        _u = x[1];
        _v = x[2];
        ThePSurfaceTool.D1(_surf, _u, _v, out _pntsol, out _d1u, out _d1v);
        QuadricTool.ValueAndGradient(_func, _pntsol.X, _pntsol.Y, _pntsol.Z, ref _valf, ref _gradient);
        f[1] = _valf;
        d[1, 1] = _d1u.Dot(_gradient);
        d[1, 2] = _d1v.Dot(_gradient);
        _computed = false;
        _derived = true;
        return true;
    }

    /// <summary>The value of the last evaluation.</summary>
    public double Root() => _valf;

    /// <summary>The value Tol such that the function is null where |Root()| &lt; Tol.</summary>
    public double Tolerance() => _tol;

    /// <summary>The point of the last evaluation.</summary>
    public Point3D Point() => _pntsol;

    /// <summary>
    /// True if the surfaces are tangent at the last evaluated point; otherwise computes the tangent of the
    /// intersection (<see cref="Direction3d"/>, <see cref="Direction2d"/>).
    /// </summary>
    public bool IsTangent()
    {
        if (!_computed)
        {
            _computed = true;
            if (!_derived)
            {
                ThePSurfaceTool.D1(_surf, _u, _v, out _pntsol, out _d1u, out _d1v);
                _derived = true;
            }

            _tgdu = _gradient.Dot(_d1v);
            _tgdv = -_gradient.Dot(_d1u);
            double n2grad = _gradient.SquareMagnitude;
            double N2grad_EpsAng2 = n2grad * EpsAng2;
            double n2d1u = _d1u.SquareMagnitude;
            double n2d1v = _d1v.SquareMagnitude;
            _tangent = (_tgdu * _tgdu <= N2grad_EpsAng2 * n2d1v) && (_tgdv * _tgdv <= N2grad_EpsAng2 * n2d1u);
            if (!_tangent)
            {
                _d3d = Vector3D.LinearForm(_tgdu, _d1u, _tgdv, _d1v);
                _d2d = new Direction2D(_tgdu, _tgdv);
                if (_d3d.Magnitude <= Tolpetit)
                { // jag
                    _tangent = true;
                }
            }
        }
        return _tangent;
    }

    /// <summary>The tangent of the intersection in 3D. Throws StdFail_UndefinedDerivative at a tangent point.</summary>
    public Vector3D Direction3d()
    {
        if (IsTangent())
        {
            throw new UndefinedDerivativeException();
        }
        return _d3d;
    }

    /// <summary>The tangent in the parametric space. Throws StdFail_UndefinedDerivative at a tangent point.</summary>
    public Direction2D Direction2d()
    {
        if (IsTangent())
        {
            throw new UndefinedDerivativeException();
        }
        return _d2d;
    }

    /// <summary>The parametric surface.</summary>
    public ThePSurface PSurface() => _surf;

    /// <summary>The implicit surface (the function's copy).</summary>
    public SurfaceQuadric ISurface() => _func;
}
