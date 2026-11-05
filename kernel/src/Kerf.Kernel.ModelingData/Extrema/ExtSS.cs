// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtSS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtSS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extrema of the distance between two surfaces: Extrema_ExtElSS for two planes, Extrema_GenExtSS (20 x 20
/// samples) otherwise. Extrema whose parameters fall outside the given ranges (beyond the tolerances) are dropped;
/// periodic parameters are first put into the period that starts at the range's start.
/// </summary>
/// <remarks>
/// The second surface is kept by reference, as OCCT keeps it: it must outlive the algorithm. For parallel planes
/// <see cref="ExtCount"/> is 1 with the distance, and there are no points (<see cref="Points"/> throws, where OCCT's
/// release build reads past its empty sequences).
/// </remarks>
internal class ExtSS
{
    private SurfaceAdaptorBase? _s2;
    private bool _done;
    private bool _isPar;
    private readonly ExtElSS _extElSS = new();
    private readonly NCollection_Sequence<POnSurf> _pOnS1 = new();
    private readonly NCollection_Sequence<POnSurf> _pOnS2 = new();
    private double _myuinf1;
    private double _myusup1;
    private double _myvinf1;
    private double _myvsup1;
    private double _myuinf2;
    private double _myusup2;
    private double _myvinf2;
    private double _myvsup2;
    private double _mytolS1;
    private double _mytolS2;
    private readonly NCollection_Sequence<double> _sqDist = new();
    private SurfaceKind _stype;

    /// <summary>An empty algorithm.</summary>
    public ExtSS()
    {
        _s2 = null;
        _done = false;
        _isPar = false;
        _myuinf1 = 0.0;
        _myusup1 = 0.0;
        _myvinf1 = 0.0;
        _myvsup1 = 0.0;
        _myuinf2 = 0.0;
        _myusup2 = 0.0;
        _myvinf2 = 0.0;
        _myvsup2 = 0.0;
        _mytolS1 = 0.0;
        _mytolS2 = 0.0;
        _stype = OtherSurface;
    }

    /// <summary>Computes the extrema between S1 and S2 over their whole ranges.</summary>
    public ExtSS(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, double tolS1, double tolS2)
    {
        Initialize(s2, s2.FirstUParameter(), s2.LastUParameter(), s2.FirstVParameter(), s2.LastVParameter(), tolS2);

        Perform(s1, s1.FirstUParameter(), s1.LastUParameter(), s1.FirstVParameter(), s1.LastVParameter(), tolS1);
    }

    /// <summary>Computes the extrema between S1 and S2 over the given ranges.</summary>
    public ExtSS(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, double uinf1, double usup1, double vinf1,
        double vsup1, double uinf2, double usup2, double vinf2, double vsup2, double tolS1, double tolS2)
    {
        Initialize(s2, uinf2, usup2, vinf2, vsup2, tolS2);
        Perform(s1, uinf1, usup1, vinf1, vsup1, tolS1);
    }

    /// <summary>Sets the second surface and its range.</summary>
    public void Initialize(SurfaceAdaptorBase s2, double uinf2, double usup2, double vinf2, double vsup2, double tolS2)
    {
        _s2 = s2;
        _isPar = false;
        _myuinf2 = uinf2;
        _myusup2 = usup2;
        _myvinf2 = vinf2;
        _myvsup2 = vsup2;
        _mytolS2 = tolS2;
        _stype = s2.GetType();
    }

    /// <summary>Computes the extrema with S1 over the given range (the second surface must have been initialised).</summary>
    public void Perform(SurfaceAdaptorBase s1, double uinf1, double usup1, double vinf1, double vsup1, double tolS1)
    {
        if (_s2 is null)
        {
            throw new NotDoneException("Extrema_ExtSS: the second surface is not initialized");
        }
        _myuinf1 = uinf1;
        _myusup1 = usup1;
        _myvinf1 = vinf1;
        _myvsup1 = vsup1;
        _mytolS1 = tolS1;
        _pOnS1.Clear();
        _pOnS2.Clear();
        _sqDist.Clear();
        int i;
        SurfaceKind myS1type = s1.GetType();
        const int nbU = 20, nbV = 20;

        switch (myS1type)
        {
            case Plane:
            {
                switch (_stype)
                {
                    case Plane:
                    {
                        _extElSS.Perform(s1.Plane(), _s2.Plane());
                    }
                        break;
                    default:
                    {
                        GenExtSS ext = new(s1, _s2, nbU, nbV, _mytolS1, _mytolS2);
                        _done = ext.IsDone;
                        if (_done)
                        {
                            int nbExt = ext.ExtCount;
                            double u1, v1, u2, v2;
                            POnSurf ps1;
                            POnSurf ps2;
                            for (i = 1; i <= nbExt; i++)
                            {
                                ps1 = ext.PointOnS1(i);
                                ps2 = ext.PointOnS2(i);
                                ps1.Parameter(out u1, out v1);
                                ps2.Parameter(out u2, out v2);
                                if (s1.IsUPeriodic)
                                {
                                    u1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u1, _myuinf1, _myuinf1 + s1.UPeriod());
                                }
                                if (s1.IsVPeriodic)
                                {
                                    v1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v1, _myvinf1, _myvinf1 + s1.VPeriod());
                                }
                                if (_s2.IsUPeriodic)
                                {
                                    u2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u2, _myuinf2, _myuinf2 + _s2.UPeriod());
                                }
                                if (_s2.IsVPeriodic)
                                {
                                    v2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v2, _myvinf2, _myvinf2 + _s2.VPeriod());
                                }

                                if ((_myuinf1 - u1) <= _mytolS1 && (u1 - _myusup1) <= _mytolS1
                                    && (_myvinf1 - v1) <= _mytolS1 && (v1 - _myvsup1) <= _mytolS1
                                    && (_myuinf2 - u2) <= _mytolS2 && (u2 - _myusup2) <= _mytolS2
                                    && (_myvinf2 - v2) <= _mytolS2 && (v2 - _myvsup2) <= _mytolS2)
                                {
                                    _sqDist.Append(ext.SquareDistance(i));
                                    _pOnS1.Append(new POnSurf(u1, v1, ps1.Value()));
                                    _pOnS2.Append(new POnSurf(u2, v2, ps2.Value()));
                                }
                            }
                        }
                        return;
                    }
                }
                break;
            }
            default:
            {
                GenExtSS ext = new(s1, _s2, nbU, nbV, _mytolS1, _mytolS2);
                _done = ext.IsDone;
                if (_done)
                {
                    int nbExt = ext.ExtCount;
                    double u1, v1, u2, v2;
                    POnSurf ps1;
                    POnSurf ps2;
                    for (i = 1; i <= nbExt; i++)
                    {
                        ps1 = ext.PointOnS1(i);
                        ps2 = ext.PointOnS2(i);
                        ps1.Parameter(out u1, out v1);
                        ps2.Parameter(out u2, out v2);
                        if (s1.IsUPeriodic)
                        {
                            u1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u1, _myuinf1, _myuinf1 + s1.UPeriod());
                        }
                        if (s1.IsVPeriodic)
                        {
                            v1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v1, _myvinf1, _myvinf1 + s1.VPeriod());
                        }
                        if (_s2.IsUPeriodic)
                        {
                            u2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u2, _myuinf2, _myuinf2 + _s2.UPeriod());
                        }
                        if (_s2.IsVPeriodic)
                        {
                            v2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v2, _myvinf2, _myvinf2 + _s2.VPeriod());
                        }

                        if ((_myuinf1 - u1) <= _mytolS1 && (u1 - _myusup1) <= _mytolS1 && (_myvinf1 - v1) <= _mytolS1
                            && (v1 - _myvsup1) <= _mytolS1 && (_myuinf2 - u2) <= _mytolS2 && (u2 - _myusup2) <= _mytolS2
                            && (_myvinf2 - v2) <= _mytolS2 && (v2 - _myvsup2) <= _mytolS2)
                        {
                            _sqDist.Append(ext.SquareDistance(i));
                            _pOnS1.Append(new POnSurf(u1, v1, ps1.Value()));
                            _pOnS2.Append(new POnSurf(u2, v2, ps2.Value()));
                        }
                    }
                }
                return;
            }
        }

        _done = _extElSS.IsDone;
        if (_done)
        {
            _isPar = _extElSS.IsParallel;
            if (_isPar)
            {
                _sqDist.Append(_extElSS.SquareDistance(1));
            }
            else
            {
                int nbExt = _extElSS.ExtCount;
                double u1, v1, u2, v2;
                for (i = 1; i <= nbExt; i++)
                {
                    _extElSS.Points(i, out POnSurf ps1, out POnSurf ps2);
                    ps1.Parameter(out u1, out v1);
                    ps2.Parameter(out u2, out v2);
                    if ((_myuinf1 - u1) <= _mytolS1 && (u1 - _myusup1) <= _mytolS1 && (_myvinf1 - v1) <= _mytolS1
                        && (v1 - _myvsup1) <= _mytolS1 && (_myuinf2 - u2) <= _mytolS2 && (u2 - _myusup2) <= _mytolS2
                        && (_myvinf2 - v2) <= _mytolS2 && (v2 - _myvsup2) <= _mytolS2)
                    {
                        _sqDist.Append(_extElSS.SquareDistance(i));
                        _pOnS1.Append(ps1);
                        _pOnS2.Append(ps2);
                    }
                }
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _done;

    /// <summary>True if the surfaces are parallel planes.</summary>
    public bool IsParallel
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _isPar;
        }
    }

    /// <summary>The square distance of the Nth extremum.</summary>
    public double SquareDistance(int n)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        return _sqDist.Value(n);
    }

    /// <summary>The number of extrema (1 for parallel planes).</summary>
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

    /// <summary>The points of the Nth extremum on S1 and S2.</summary>
    public void Points(int n, out POnSurf p1, out POnSurf p2)
    {
        if (n < 1 || n > ExtCount)
        {
            throw new OutOfRangeException();
        }

        p1 = _pOnS1.Value(n);
        p2 = _pOnS2.Value(n);
    }
}
