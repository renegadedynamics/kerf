// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncExtCS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncExtCS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The system whose roots are the extrema of the distance between a curve C and a surface S:
/// F1(t, u, v) = (C(t) - S(u, v)).C'(t), F2 = (C(t) - S(u, v)).Su, F3 = (C(t) - S(u, v)).Sv, with its derivatives,
/// for math_FunctionSetRoot. Each root found (GetStateNumber) is stored with its square distance, unless its curve
/// parameter repeats one already found.
/// </summary>
/// <remarks>
/// OCCT keeps pointers to the curve and the surface; the port keeps the adaptors, which must outlive the function.
/// <see cref="SquareDistances"/>, <see cref="PointsOnCurve"/> and <see cref="PointsOnSurf"/> return the stored
/// sequences themselves, for the caller to change (OCCT's non-const references).
/// </remarks>
internal class FuncExtCS : FunctionSetWithDerivatives
{
    private CurveAdaptorBase? _c;
    private SurfaceAdaptorBase? _s;
    private Point3D _p1 = new();
    private Point3D _p2 = new();
    private double _myt;
    private double _u;
    private double _v;
    private readonly NCollection_Sequence<double> _sqDist = new();
    private readonly NCollection_Sequence<POnCurv> _point1 = new();
    private readonly NCollection_Sequence<POnSurf> _point2 = new();
    private bool _cinit;
    private bool _sinit;

    /*-----------------------------------------------------------------------------
     Function to search for an extremal distance between a curve C and a surface S.
    { F1(t,u,v) = (C(t)-S(u,v)).Dtc(t) }
    { F2(t,u,v) = (C(t)-S(u,v)).Dus(u,v) }
    { F3(t,u,v) = (C(t)-S(u,v)).Dvs(u,v) }
    { Dtf1(t,u,v) = Dtc(t).Dtc(t)+(C(t)-S(u,v)).Dttc(t)
                  = ||Dtc(t)||**2+(C(t)-S(u,v)).Dttc(t) }
    { Duf1(t,u,v) = -Dus(u,v).Dtc(t) }
    { Dvf1(t,u,v) = -Dvs(u,v).Dtc(t) }
    { Dtf2(t,u,v) = Dtc(t).Dus(u,v) }
    { Duf2(t,u,v) = -Dus(u,v).Dus(u,v)+(C(t)-S(u,v)).Duus(u,v)
                  = -||Dus(u,v)||**2+(C(t)-S(u,v)).Duus(u,v) }
    { Dvf2(t,u,v) = -Dvs(u,v).Dus(u,v)+(C(t)-S(u,v)).Duvs(u,v) }
    { Dtf3(t,u,v) = Dtc(t).Dvs(u,v) }
    { Duf3(t,u,v) = -Dus(u,v).Dvs(u,v)+(C(t)-S(u,v)).Duvs(u,v) }
    { Dvf3(t,u,v) = -Dvs(u,v).Dvs(u,v)+(C(t)-S(u,v)).Dvvs(u,v) }
    ----------------------------------------------------------------------------*/

    public FuncExtCS()
    {
        _c = null;
        _s = null;
        _myt = 0.0;
        _u = 0.0;
        _v = 0.0;
        _cinit = false;
        _sinit = false;
    }

    public FuncExtCS(CurveAdaptorBase c, SurfaceAdaptorBase s)
    {
        Initialize(c, s);
    }

    /// <summary>Sets the curve and the surface, and forgets the extrema found.</summary>
    public void Initialize(CurveAdaptorBase c, SurfaceAdaptorBase s)
    {
        _c = c;
        _s = s;
        _cinit = true;
        _sinit = true;
        _point1.Clear();
        _point2.Clear();
        _sqDist.Clear();
    }

    public override int NbVariables() => (3);

    public override int NbEquations() => (3);

    /// <summary>Calculates Fi(t, u, v).</summary>
    public override bool Value(math_Vector uv, math_Vector f)
    {
        if (!_cinit || !_sinit)
        {
            throw new TypeMismatchException();
        }

        _myt = uv[1];
        _u = uv[2];
        _v = uv[3];

        _c!.D1(_myt, out _p1, out Vector3D dtc);
        _s!.D1(_u, _v, out _p2, out Vector3D dus, out Vector3D dvs);

        Vector3D p1P2 = new(_p2, _p1);

        f[1] = p1P2.Dot(dtc);
        f[2] = p1P2.Dot(dus);
        f[3] = p1P2.Dot(dvs);

        return true;
    }

    /// <summary>Calculates Fi'(t, u, v).</summary>
    public override bool Derivatives(math_Vector uv, Matrix df)
    {
        math_Vector f = new(1, 3);
        return Values(uv, f, df);
    }

    /// <summary>Calculates Fi(t, u, v) and Fi'(t, u, v).</summary>
    public override bool Values(math_Vector uv, math_Vector f, Matrix df)
    {
        if (!_cinit || !_sinit)
        {
            throw new TypeMismatchException();
        }

        _myt = uv[1];
        _u = uv[2];
        _v = uv[3];

        _c!.D2(_myt, out _p1, out Vector3D dtc, out Vector3D dttc);
        _s!.D2(_u, _v, out _p2, out Vector3D dus, out Vector3D dvs, out Vector3D duus, out Vector3D dvvs, out Vector3D duvs);

        Vector3D p1P2 = new(_p2, _p1);

        f[1] = p1P2.Dot(dtc);
        f[2] = p1P2.Dot(dus);
        f[3] = p1P2.Dot(dvs);

        df[1, 1] = dtc.SquareMagnitude + p1P2.Dot(dttc);
        df[1, 2] = -dus.Dot(dtc);
        df[1, 3] = -dvs.Dot(dtc);

        df[2, 1] = -df[1, 2]; // Dtc.Dot(Dus);
        df[2, 2] = -dus.SquareMagnitude + p1P2.Dot(duus);
        df[2, 3] = -dvs.Dot(dus) + p1P2.Dot(duvs);

        df[3, 1] = -df[1, 3]; // Dtc.Dot(Dvs);
        df[3, 2] = df[2, 3]; // -Dus.Dot(Dvs)+P1P2.Dot(Duvs);
        df[3, 3] = -dvs.SquareMagnitude + p1P2.Dot(dvvs);

        return true;
    }

    /// <summary>Saves the extremum found, unless its curve parameter repeats one already saved.</summary>
    public override int GetStateNumber()
    {
        if (!_cinit || !_sinit)
        {
            throw new TypeMismatchException();
        }
        // comparison of solution with previous solutions
        double tol2d = global::Kerf.Engine.Foundation.Precision.SquarePConfusion();
        int i = 1, nbSol = _sqDist.Length();
        for (; i <= nbSol; i++)
        {
            double t = _point1[i].Parameter();
            t -= _myt;
            t *= t;
            if (t <= tol2d)
            {
                break;
            }
        }
        if (i <= nbSol)
        {
            return 0;
        }
        _sqDist.Append(_p1.SquareDistanceTo(_p2));
        _point1.Append(new POnCurv(_myt, _p1));
        _point2.Append(new POnSurf(_u, _v, _p2));
        return 0;
    }

    /// <summary>The number of extrema found.</summary>
    public int ExtCount => _sqDist.Length();

    /// <summary>The square distance of the Nth extremum.</summary>
    public double SquareDistance(int n)
    {
        if (!_cinit || !_sinit)
        {
            throw new TypeMismatchException();
        }
        return _sqDist.Value(n);
    }

    /// <summary>The Nth extremum on the curve.</summary>
    public POnCurv PointOnCurve(int n)
    {
        if (!_cinit || !_sinit)
        {
            throw new TypeMismatchException();
        }
        return _point1.Value(n);
    }

    /// <summary>The Nth extremum on the surface.</summary>
    public POnSurf PointOnSurface(int n)
    {
        if (!_cinit || !_sinit)
        {
            throw new TypeMismatchException();
        }
        return _point2.Value(n);
    }

    /// <summary>The stored square distances (shared: the caller may change them).</summary>
    public NCollection_Sequence<double> SquareDistances() => _sqDist;

    /// <summary>The stored points on the curve (shared: the caller may change them).</summary>
    public NCollection_Sequence<POnCurv> PointsOnCurve() => _point1;

    /// <summary>The stored points on the surface (shared: the caller may change them).</summary>
    public NCollection_Sequence<POnSurf> PointsOnSurf() => _point2;
}
