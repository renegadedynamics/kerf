// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Extrema.ExtAlgo;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extrema of the distance between a point and a surface: analytically on planes, cylinders, cones, spheres and
/// tori (Extrema_ExtPElS), with Extrema_ExtPExtS and Extrema_ExtPRevS on swept surfaces, and by sampling
/// (Extrema_GenExtPS) otherwise. Solutions outside the parameter range (put in the period first) are dropped.
/// </summary>
/// <remarks>The surface adaptor is kept, as OCCT keeps a pointer: it must outlive the algorithm.</remarks>
internal class ExtPS
{
    private SurfaceAdaptorBase? _s;
    private bool _done;
    private readonly ExtPElS _extPElS = new();
    private readonly GenExtPS _extPS = new();
    private readonly NCollection_Sequence<POnSurf> _points = new();
    private double _myuinf;
    private double _myusup;
    private double _myvinf;
    private double _myvsup;
    private double _mytolu;
    private double _mytolv;
    private double _d11;
    private double _d12;
    private double _d21;
    private double _d22;
    private Point3D _p11 = new();
    private Point3D _p12 = new();
    private Point3D _p21 = new();
    private Point3D _p22 = new();
    private readonly NCollection_Sequence<double> _sqDist = new();
    private SurfaceKind _mytype;
    private ExtPExtS? _extPExtS;
    private ExtPRevS? _extPRevS;

    private static bool IsoIsDeg(SurfaceAdaptorBase s, double param, IsoType it, double tolMin, double tolMax)
    {
        double u1 = 0.0, u2 = 0.0, v1 = 0.0, v2 = 0.0, t;
        bool along = true;
        u1 = s.FirstUParameter();
        u2 = s.LastUParameter();
        v1 = s.FirstVParameter();
        v2 = s.LastVParameter();
        double step, d1NormMax;
        if (it == IsoV)
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u1) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(u2))
            {
                step = (u2 - u1) / 10;
                if (step < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return false;
                }
                d1NormMax = 0.0;

                for (t = u1; t <= u2; t = t + step)
                {
                    s.D1(t, param, out _, out Vector3D d1U, out _);
                    d1NormMax = Max(d1NormMax, d1U.Magnitude);
                }

                if (d1NormMax > tolMax || d1NormMax < tolMin)
                {
                    along = false;
                }
            }
        }
        else
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(v1) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(v2))
            {
                step = (v2 - v1) / 10;
                if (step < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    return false;
                }
                d1NormMax = 0.0;
                for (t = v1; t <= v2; t = t + step)
                {
                    s.D1(param, t, out _, out _, out Vector3D d1V);
                    d1NormMax = Max(d1NormMax, d1V.Magnitude);
                }

                if (d1NormMax > tolMax || d1NormMax < tolMin)
                {
                    along = false;
                }
            }
        }
        return along;
    }

    private void TreatSolution(in POnSurf ps, double val)
    {
        ps.Parameter(out double u, out double v);
        if (_s!.IsUPeriodic)
        {
            u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, _myuinf, _myuinf + _s.UPeriod());

            // Handle trimmed surfaces.
            if (u > _myusup + _mytolu)
            {
                u -= _s.UPeriod();
            }
            if (u < _myuinf - _mytolu)
            {
                u += _s.UPeriod();
            }
        }
        if (_s.IsVPeriodic)
        {
            v = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, _myvinf, _myvinf + _s.VPeriod());

            // Handle trimmed surfaces.
            if (v > _myvsup + _mytolv)
            {
                v -= _s.VPeriod();
            }
            if (v < _myvinf - _mytolv)
            {
                v += _s.VPeriod();
            }
        }
        if ((_myuinf - u) <= _mytolu && (u - _myusup) <= _mytolu && (_myvinf - v) <= _mytolv && (v - _myvsup) <= _mytolv)
        {
            _points.Append(new POnSurf(u, v, ps.Value()));
            _sqDist.Append(val);
        }
    }

    public ExtPS()
    {
        _s = null;
        _done = false;
        _myuinf = 0.0;
        _myusup = 0.0;
        _myvinf = 0.0;
        _myvsup = 0.0;
        _mytolu = 0.0;
        _mytolv = 0.0;
        _d11 = 0.0;
        _d12 = 0.0;
        _d21 = 0.0;
        _d22 = 0.0;
        _mytype = OtherSurface;
    }

    /// <summary>
    /// The extrema between theP and theS on theS's whole range. theTolU and theTolV are the parametric tolerances of
    /// the solutions.
    /// </summary>
    public ExtPS(in Point3D p, SurfaceAdaptorBase s, double tolU, double tolV,
        ExtFlag f = ExtFlag_MINMAX, ExtAlgo a = ExtAlgo_Grad)
    {
        _extPS.SetFlag(f);
        _extPS.SetAlgo(a);

        Initialize(s, s.FirstUParameter(), s.LastUParameter(), s.FirstVParameter(), s.LastVParameter(),
            tolU, tolV);

        Perform(p);
    }

    /// <summary>The extrema between theP and theS on [theUinf, theUsup] x [theVinf, theVsup].</summary>
    public ExtPS(in Point3D p, SurfaceAdaptorBase s, double uinf, double usup, double vinf,
        double vsup, double tolU, double tolV, ExtFlag f = ExtFlag_MINMAX,
        ExtAlgo a = ExtAlgo_Grad)
    {
        _extPS.SetFlag(f);
        _extPS.SetAlgo(a);

        Initialize(s, uinf, usup, vinf, vsup, tolU, tolV);

        Perform(p);
    }

    /// <summary>Sets the surface and its range (infinite bounds become ±1e10) and the tolerances.</summary>
    public void Initialize(SurfaceAdaptorBase s, double uinf, double usup, double vinf, double vsup,
        double tolU, double tolV)
    {
        _s = s;
        _myuinf = uinf;
        _myusup = usup;
        _myvinf = vinf;
        _myvsup = vsup;

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_myuinf))
        {
            _myuinf = -1e10;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_myusup))
        {
            _myusup = 1e10;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(_myvinf))
        {
            _myvinf = -1e10;
        }
        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(_myvsup))
        {
            _myvsup = 1e10;
        }

        _mytolu = tolU;
        _mytolv = tolV;
        _mytype = _s.GetType();

        bool isB = (_s.GetType() == SurfaceKind.BSplineSurface || _s.GetType() == SurfaceKind.BezierSurface);

        int nbU = (isB) ? 44 : 32;
        int nbV = (isB) ? 44 : 32;

        bool bUIsoIsDeg = false, bVIsoIsDeg = false;

        if (_s.GetType() != Plane)
        {
            bUIsoIsDeg = IsoIsDeg(s, _myuinf, IsoU, 0.0, 1.0e-9)
                         || IsoIsDeg(s, _myusup, IsoU, 0.0, 1.0e-9);
            bVIsoIsDeg = IsoIsDeg(s, _myvinf, IsoV, 0.0, 1.0e-9)
                         || IsoIsDeg(s, _myvsup, IsoV, 0.0, 1.0e-9);
        }

        if (bUIsoIsDeg)
        {
            nbU = 300;
        }
        if (bVIsoIsDeg)
        {
            nbV = 300;
        }

        _extPS.Initialize(_s, nbU, nbV, _myuinf, _myusup, _myvinf, _myvsup, _mytolu, _mytolv);

        _extPExtS = null;
        _extPRevS = null;
    }

    /// <summary>Computes the extrema for thePoint. Throws if the surface has not been initialized.</summary>
    public void Perform(in Point3D point)
    {
        _points.Clear();
        _sqDist.Clear();

        switch (_mytype)
        {
            case SurfaceKind.Cylinder:
                _extPElS.Perform(point, _s!.Cylinder(), global::Kerf.Engine.Foundation.Precision.Confusion());
                break;
            case Plane:
                _extPElS.Perform(point, _s!.Plane(), global::Kerf.Engine.Foundation.Precision.Confusion());
                break;
            case SurfaceKind.Cone:
                _extPElS.Perform(point, _s!.Cone(), global::Kerf.Engine.Foundation.Precision.Confusion());
                break;
            case SurfaceKind.Sphere:
                _extPElS.Perform(point, _s!.Sphere(), global::Kerf.Engine.Foundation.Precision.Confusion());
                break;
            case SurfaceKind.Torus:
                _extPElS.Perform(point, _s!.Torus(), global::Kerf.Engine.Foundation.Precision.Confusion());
                break;

            case SurfaceOfExtrusion:
                {
                    if (_extPExtS is null)
                    {
                        SurfaceOfLinearExtrusion s =
                            new(_s!.BasisCurve(), _s.Direction());

                        _extPExtS = new ExtPExtS(point, s, _myuinf, _myusup, _myvinf, _myvsup, _mytolu, _mytolv);
                    }
                    else
                    {
                        _extPExtS.Perform(point);
                    }

                    _done = _extPExtS.IsDone;
                    if (_done)
                    {
                        for (int idx = 1; idx <= _extPExtS.NbExt(); ++idx)
                        {
                            TreatSolution(_extPExtS.Point(idx), _extPExtS.SquareDistance(idx));
                        }
                    }

                    return;
                }

            case SurfaceOfRevolution:
                {
                    if (_extPRevS is null)
                    {
                        RevolutionSurfaceAdaptor s = new(_s!.BasisCurve(), _s.AxeOfRevolution());

                        _extPRevS = new ExtPRevS(point, s, _myuinf, _myusup, _myvinf, _myvsup, _mytolu, _mytolv);
                    }
                    else
                    {
                        _extPRevS.Perform(point);
                    }

                    _done = _extPRevS.IsDone;
                    if (_done)
                    {
                        for (int idx = 1; idx <= _extPRevS.ExtCount; ++idx)
                        {
                            TreatSolution(_extPRevS.Point(idx), _extPRevS.SquareDistance(idx));
                        }
                    }

                    return;
                }

            default:
                {
                    _extPS.Perform(point);
                    _done = _extPS.IsDone;
                    if (_done)
                    {
                        for (int idx = 1; idx <= _extPS.ExtCount; ++idx)
                        {
                            TreatSolution(_extPS.Point(idx), _extPS.SquareDistance(idx));
                        }
                    }
                    return;
                }
        }

        _done = _extPElS.IsDone;
        if (_done)
        {
            for (int idx = 1; idx <= _extPElS.ExtCount; ++idx)
            {
                TreatSolution(_extPElS.Point(idx), _extPElS.SquareDistance(idx));
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _done;

    /// <summary>The Nth square distance.</summary>
    public double SquareDistance(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _sqDist.Value(n);
    }

    /// <summary>The number of extremum distances.</summary>
    public int ExtCount
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _sqDist.Length();
        }
    }

    /// <summary>The point of the Nth extremum distance.</summary>
    public POnSurf Point(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _points.Value(n);
    }

    /// <summary>
    /// The square distances to the corners of the range and the corner points. OCCT never computes them: they are 0
    /// and the origin, or (after the parameterised constructors) whatever was in memory.
    /// </summary>
    public void TrimmedSquareDistances(out double dUfVf, out double dUfVl, out double dUlVf, out double dUlVl,
        out Point3D pUfVf, out Point3D pUfVl, out Point3D pUlVf, out Point3D pUlVl)
    {
        dUfVf = _d11; // OCCT: d11, ..., P22 are never computed
        dUfVl = _d12;
        dUlVf = _d21;
        dUlVl = _d22;
        pUfVf = _p11;
        pUfVl = _p12;
        pUlVf = _p21;
        pUlVl = _p22;
    }

    public void SetFlag(ExtFlag f) => _extPS.SetFlag(f);

    public void SetAlgo(ExtAlgo a) => _extPS.SetAlgo(a);
}
