// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncPSNorm.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncPSNorm.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The system whose roots are the extrema of the distance between a point P and a surface S:
/// F1(u, v) = (S - P).Su and F2(u, v) = (S - P).Sv, with its derivatives, for math_FunctionSetRoot. Each root found
/// (GetStateNumber) is stored with its square distance, unless it repeats one already found.
/// </summary>
/// <remarks>
/// OCCT keeps a pointer to the surface; the port keeps the adaptor, which must outlive the function.
/// </remarks>
internal class FuncPSNorm : FunctionSetWithDerivatives
{
    private Point3D _p = new();
    private SurfaceAdaptorBase? _s;
    private double _u;
    private double _v;
    private Point3D _ps = new();
    private readonly NCollection_Sequence<double> _sqDist = new();
    private readonly NCollection_Sequence<POnSurf> _point = new();
    private bool _pinit;
    private bool _sinit;

    public FuncPSNorm()
    {
        _s = null;
        _u = 0.0;
        _v = 0.0;
        _pinit = false;
        _sinit = false;
    }

    public FuncPSNorm(in Point3D p, SurfaceAdaptorBase s)
    {
        _u = 0.0;
        _v = 0.0;
        _p = p;
        _s = s;
        _pinit = true;
        _sinit = true;
    }

    /// <summary>Sets the surface, and forgets the extrema found.</summary>
    public void Initialize(SurfaceAdaptorBase s)
    {
        _s = s;
        _sinit = true;
        _point.Clear();
        _sqDist.Clear();
    }

    /// <summary>Sets the point, and forgets the extrema found.</summary>
    public void SetPoint(in Point3D p)
    {
        _p = p;
        _pinit = true;
        _point.Clear();
        _sqDist.Clear();
    }

    public override int NbVariables() => 2;

    public override int NbEquations() => 2;

    /// <summary>Calculates Fi(U, V).</summary>
    public override bool Value(math_Vector uv, math_Vector f)
    {
        if (!_pinit || !_sinit)
        {
            throw new TypeMismatchException();
        }
        _u = uv[1];
        _v = uv[2];
        _s!.D1(_u, _v, out _ps, out Vector3D dus, out Vector3D dvs);

        Vector3D pPs = new(_p, _ps);

        f[1] = pPs.Dot(dus);
        f[2] = pPs.Dot(dvs);

        return true;
    }

    /// <summary>Calculates Fi'(U, V).</summary>
    public override bool Derivatives(math_Vector uv, Matrix df)
    {
        math_Vector f = new(1, 2);
        return Values(uv, f, df);
    }

    /// <summary>Calculates Fi(U, V) and Fi'(U, V).</summary>
    public override bool Values(math_Vector uv, math_Vector f, Matrix df)
    {
        if (!_pinit || !_sinit)
        {
            throw new TypeMismatchException();
        }
        _u = uv[1];
        _v = uv[2];
        _s!.D2(_u, _v, out _ps, out Vector3D dus, out Vector3D dvs, out Vector3D duus, out Vector3D dvvs, out Vector3D duvs);

        Vector3D pPs = new(_p, _ps);

        df[1, 1] = dus.SquareMagnitude + pPs.Dot(duus);
        df[1, 2] = dvs.Dot(dus) + pPs.Dot(duvs);
        df[2, 1] = df[1, 2];
        df[2, 2] = dvs.SquareMagnitude + pPs.Dot(dvvs);

        // 3. Value
        f[1] = pPs.Dot(dus);
        f[2] = pPs.Dot(dvs);

        return true;
    }

    /// <summary>Saves the extremum found, unless it repeats one already saved.</summary>
    public override int GetStateNumber()
    {
        if (!_pinit || !_sinit)
        {
            throw new TypeMismatchException();
        }
        // comparison of solution with previous solutions
        int i = 1, nbSol = _sqDist.Length();
        double tol2d = global::Kerf.Engine.Foundation.Precision.PConfusion() * global::Kerf.Engine.Foundation.Precision.PConfusion();

        for (; i <= nbSol; i++)
        {
            _point[i].Parameter(out double u, out double v);
            if (((_u - u) * (_u - u) + (_v - v) * (_v - v)) <= tol2d)
            {
                break;
            }
        }
        if (i <= nbSol)
        {
            return 0;
        }
        _sqDist.Append(_ps.SquareDistanceTo(_p));
        _point.Append(new POnSurf(_u, _v, _ps));
        return 0;
    }

    /// <summary>The number of extrema found.</summary>
    public int ExtCount => _sqDist.Length();

    /// <summary>The square distance of the Nth extremum.</summary>
    public double SquareDistance(int n)
    {
        if (!_pinit || !_sinit)
        {
            throw new TypeMismatchException();
        }
        return _sqDist.Value(n);
    }

    /// <summary>The Nth extremum.</summary>
    public POnSurf Point(int n)
    {
        if (!_pinit || !_sinit)
        {
            throw new TypeMismatchException();
        }
        return _point.Value(n);
    }
}
