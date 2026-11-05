// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_PWalking.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_PWalking.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_PWalking.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Intersection.Implicit.ConstIsoparametric;
using static Kerf.Engine.Intersection.Walking.StatusDeflection;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>
/// The walking of a parametric/parametric intersection: from a start point (u1, v1, u2, v2), the polyline of
/// points common to two surfaces, marched in both directions with an adaptive step (IntWalk_TheInt2S at each step)
/// until the line leaves the surfaces' domains, closes, or stops on a tangent point.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's class, a working object: <see cref="Perform(NCollection_Array1{double})"/> computes the line, which
/// <see cref="Line"/> returns (the line itself, not a copy; IntPatch_PrmPrmIntersection edits it). OCCT's protected
/// members are internal (IntPatch and GeomInt call none of them from outside). The step arrays (<c>pasuv</c>,
/// <c>pasInit</c>, ...) are arrays allocated with the object. OCCT_DEBUG blocks and the <c>REGLAGE</c> dump of
/// <see cref="AddAPoint"/> are left out.
/// </para>
/// <para>
/// <see cref="_stepTrace"/> is a test hook (decision 687): when set, it is called each time
/// <see cref="TestDeflection"/> returns, with the status returned and the point of the step being tested, so that
/// a walk can be compared step by step with a native recording. It changes nothing in the computation.
/// </para>
/// </remarks>
internal partial class PWalking
{
    private bool _done;
    private LineOn2S _line = new();
    private bool _close;
    private bool _tgfirst;
    private bool _tglast;
    private int _tangentIdx;
    private Direction3D _tgdir = new();
    private readonly double _fleche;
    private double _pasMax;
    private readonly double _tolconf;
    private readonly double _tolTang;
    private readonly double[] _pasuv = new double[4];
    private readonly double[] _stepMin = new double[4];
    private readonly double[] _pasSav = new double[4];
    private readonly double[] _pasInit = new double[4];
    private double _uMin1;
    private double _uMax1;
    private double _vMin1;
    private double _vMax1;
    private double _uMin2;
    private double _uMax2;
    private double _vMin2;
    private double _vMax2;
    private double _resoU1;
    private double _resoU2;
    private double _resoV1;
    private double _resoV2;
    private int _sensCheminement;
    private ConstIsoparametric _choixIsoSav;
    private PntOn2S _previousPoint = new();
    private bool _previoustg;
    private Direction3D _previousd = new();
    private Direction2D _previousd1 = new();
    private Direction2D _previousd2 = new();
    private Direction2D _firstd1 = new();
    private Direction2D _firstd2 = new();
    private readonly TheInt2S _intersectionOn2S;
    private int STATIC_BLOCAGE_SUR_PAS_TROP_GRAND;
    private int STATIC_PRECEDENT_INFLEXION;

    /// <summary>
    /// Test hook (decision 687): called each time <see cref="TestDeflection"/> returns, with its status and the
    /// point being tested (<c>myIntersectionOn2S.Point()</c>).
    /// </summary>
    internal Action<StatusDeflection, PntOn2S>? _stepTrace;

    /// <summary>
    /// The same hook for the walkings that other algorithms create (IntPatch_PrmPrmIntersection's), on the current
    /// thread only; <see cref="_stepTrace"/> takes precedence.
    /// </summary>
    [ThreadStatic]
    internal static Action<StatusDeflection, PntOn2S>? s_threadStepTrace;

    /// <summary>
    /// Prepares the walking of <paramref name="caro1"/> and <paramref name="caro2"/>. TolTangency is the tolerance of
    /// the tangency test, Epsilon the 3D tolerance of the deflection test, Deflection the largest deflection between
    /// consecutive points, Increment the largest step, relative to the parametric ranges.
    /// </summary>
    public PWalking(SurfaceAdaptorBase caro1, SurfaceAdaptorBase caro2, double tolTangency, double epsilon,
                            double deflection, double increment)
    {
        _done = true;
        _close = false;
        _tgfirst = false;
        _tglast = false;
        _tangentIdx = 0;
        _fleche = deflection;
        _pasMax = 0.0;
        _tolconf = epsilon;
        _tolTang = tolTangency;
        _sensCheminement = 1;
        _previoustg = false;
        _intersectionOn2S = new TheInt2S(caro1, caro2, tolTangency);
        STATIC_BLOCAGE_SUR_PAS_TROP_GRAND = 0;
        STATIC_PRECEDENT_INFLEXION = 0;

        double kelarg = 20.0;
        //
        _pasMax = increment * 0.2; //-- June 25 99 after problems with precision
        _uMin1 = HSurfaceTool.FirstUParameter(caro1);
        _vMin1 = HSurfaceTool.FirstVParameter(caro1);
        _uMax1 = HSurfaceTool.LastUParameter(caro1);
        _vMax1 = HSurfaceTool.LastVParameter(caro1);

        _uMin2 = HSurfaceTool.FirstUParameter(caro2);
        _vMin2 = HSurfaceTool.FirstVParameter(caro2);
        _uMax2 = HSurfaceTool.LastUParameter(caro2);
        _vMax2 = HSurfaceTool.LastVParameter(caro2);

        _resoU1 = HSurfaceTool.UResolution(caro1, global::Kerf.Engine.Foundation.Precision.Confusion());
        _resoV1 = HSurfaceTool.VResolution(caro1, global::Kerf.Engine.Foundation.Precision.Confusion());

        _resoU2 = HSurfaceTool.UResolution(caro2, global::Kerf.Engine.Foundation.Precision.Confusion());
        _resoV2 = HSurfaceTool.VResolution(caro2, global::Kerf.Engine.Foundation.Precision.Confusion());

        double newreso;
        double maxval;
        double maxval2;
        //
        maxval = Math.Abs(_uMin1);
        maxval2 = Math.Abs(_uMax1);
        if (maxval2 > maxval)
        {
            maxval = maxval2;
        }
        newreso = _resoU1 * maxval;
        if (newreso > _resoU1 && newreso < 10)
        {
            _resoU1 = newreso;
        }

        maxval = Math.Abs(_uMin2);
        maxval2 = Math.Abs(_uMax2);
        if (maxval2 > maxval)
        {
            maxval = maxval2;
        }
        newreso = _resoU2 * maxval;
        if (newreso > _resoU2 && newreso < 10)
        {
            _resoU2 = newreso;
        }

        maxval = Math.Abs(_vMin1);
        maxval2 = Math.Abs(_vMax1);
        if (maxval2 > maxval)
        {
            maxval = maxval2;
        }
        newreso = _resoV1 * maxval;
        if (newreso > _resoV1 && newreso < 10)
        {
            _resoV1 = newreso;
        }

        maxval = Math.Abs(_vMin2);
        maxval2 = Math.Abs(_vMax2);
        if (maxval2 > maxval)
        {
            maxval = maxval2;
        }
        newreso = _resoV2 * maxval;
        if (newreso > _resoV2 && newreso < 10)
        {
            _resoV2 = newreso;
        }

        _pasuv[0] = _pasMax * Math.Abs(_uMax1 - _uMin1);
        _pasuv[1] = _pasMax * Math.Abs(_vMax1 - _vMin1);
        _pasuv[2] = _pasMax * Math.Abs(_uMax2 - _uMin2);
        _pasuv[3] = _pasMax * Math.Abs(_vMax2 - _vMin2);

        if (_resoU1 > 0.0001 * _pasuv[0])
        {
            _resoU1 = 0.00001 * _pasuv[0];
        }
        if (_resoV1 > 0.0001 * _pasuv[1])
        {
            _resoV1 = 0.00001 * _pasuv[1];
        }
        if (_resoU2 > 0.0001 * _pasuv[2])
        {
            _resoU2 = 0.00001 * _pasuv[2];
        }
        if (_resoV2 > 0.0001 * _pasuv[3])
        {
            _resoV2 = 0.00001 * _pasuv[3];
        }

        if (!HSurfaceTool.IsUPeriodic(caro1))
        {
            // UM1+=KELARG*pasuv[0];  Um1-=KELARG*pasuv[0];
        }
        else
        {
            double t = _uMax1 - _uMin1;
            if (t < HSurfaceTool.UPeriod(caro1))
            {
                t = 0.5 * (HSurfaceTool.UPeriod(caro1) - t);
                t = (t > kelarg * _pasuv[0]) ? kelarg * _pasuv[0] : t;
                _uMax1 += t;
                _uMin1 -= t;
            }
        }

        if (!HSurfaceTool.IsVPeriodic(caro1))
        {
            // VM1+=KELARG*pasuv[1];  Vm1-=KELARG*pasuv[1];
        }
        else
        {
            double t = _vMax1 - _vMin1;
            if (t < HSurfaceTool.VPeriod(caro1))
            {
                t = 0.5 * (HSurfaceTool.VPeriod(caro1) - t);
                t = (t > kelarg * _pasuv[1]) ? kelarg * _pasuv[1] : t;
                _vMax1 += t;
                _vMin1 -= t;
            }
        }

        if (!HSurfaceTool.IsUPeriodic(caro2))
        {
            // UM2+=KELARG*pasuv[2];  Um2-=KELARG*pasuv[2];
        }
        else
        {
            double t = _uMax2 - _uMin2;
            if (t < HSurfaceTool.UPeriod(caro2))
            {
                t = 0.5 * (HSurfaceTool.UPeriod(caro2) - t);
                t = (t > kelarg * _pasuv[2]) ? kelarg * _pasuv[2] : t;
                _uMax2 += t;
                _uMin2 -= t;
            }
        }

        if (!HSurfaceTool.IsVPeriodic(caro2))
        {
            // VM2+=KELARG*pasuv[3];  Vm2-=KELARG*pasuv[3];
        }
        else
        {
            double t = _vMax2 - _vMin2;
            if (t < HSurfaceTool.VPeriod(caro2))
            {
                t = 0.5 * (HSurfaceTool.VPeriod(caro2) - t);
                t = (t > kelarg * _pasuv[3]) ? kelarg * _pasuv[3] : t;
                _vMax2 += t;
                _vMin2 -= t;
            }
        }

        _stepMin[0] = 100.0 * _resoU1;
        _stepMin[1] = 100.0 * _resoV1;
        _stepMin[2] = 100.0 * _resoU2;
        _stepMin[3] = 100.0 * _resoV2;

        //-- ComputePasInit(pasuv,Um1,UM1,Vm1,VM1,Um2,UM2,Vm2,VM2,Caro1,Caro2);

        for (int i = 0; i <= 3; i++)
        {
            if (_pasuv[i] > 10)
            {
                _pasuv[i] = 10;
            }
            _pasInit[i] = _pasSav[i] = _pasuv[i];
        }
    }

    /// <summary>
    /// Prepares the walking as the other constructor, with the domains enlarged, and walks from (U1, V1, U2, V2).
    /// </summary>
    /// <remarks>
    /// OCCT leaves <c>tgfirst</c>, <c>tglast</c>, <c>myTangentIdx</c> and <c>previoustg</c> uninitialised here; Perform
    /// sets them before reading them, and the port's fields start false and 0.
    /// </remarks>
    public PWalking(SurfaceAdaptorBase caro1, SurfaceAdaptorBase caro2, double tolTangency, double epsilon,
                            double deflection, double increment, double u1, double v1, double u2, double v2)
    {
        _done = true;
        _close = false;
        _fleche = deflection;
        _tolconf = epsilon;
        _tolTang = tolTangency;
        _sensCheminement = 1;
        _intersectionOn2S = new TheInt2S(caro1, caro2, tolTangency);
        STATIC_BLOCAGE_SUR_PAS_TROP_GRAND = 0;
        STATIC_PRECEDENT_INFLEXION = 0;

        double kelarg = 20.0;
        //
        _pasMax = increment * 0.2; //-- June 25 99 after problems with precision
        //
        _uMin1 = HSurfaceTool.FirstUParameter(caro1);
        _vMin1 = HSurfaceTool.FirstVParameter(caro1);
        _uMax1 = HSurfaceTool.LastUParameter(caro1);
        _vMax1 = HSurfaceTool.LastVParameter(caro1);

        _uMin2 = HSurfaceTool.FirstUParameter(caro2);
        _vMin2 = HSurfaceTool.FirstVParameter(caro2);
        _uMax2 = HSurfaceTool.LastUParameter(caro2);
        _vMax2 = HSurfaceTool.LastVParameter(caro2);

        _resoU1 = HSurfaceTool.UResolution(caro1, global::Kerf.Engine.Foundation.Precision.Confusion());
        _resoV1 = HSurfaceTool.VResolution(caro1, global::Kerf.Engine.Foundation.Precision.Confusion());

        _resoU2 = HSurfaceTool.UResolution(caro2, global::Kerf.Engine.Foundation.Precision.Confusion());
        _resoV2 = HSurfaceTool.VResolution(caro2, global::Kerf.Engine.Foundation.Precision.Confusion());
        //
        double newreso, maxval, maxval2;
        //
        maxval = Math.Abs(_uMin1);
        maxval2 = Math.Abs(_uMax1);
        if (maxval2 > maxval)
        {
            maxval = maxval2;
        }
        newreso = _resoU1 * maxval;
        if (newreso > _resoU1)
        {
            _resoU1 = newreso;
        }
        //
        maxval = Math.Abs(_uMin2);
        maxval2 = Math.Abs(_uMax2);
        if (maxval2 > maxval)
        {
            maxval = maxval2;
        }
        newreso = _resoU2 * maxval;
        if (newreso > _resoU2)
        {
            _resoU2 = newreso;
        }
        //
        maxval = Math.Abs(_vMin1);
        maxval2 = Math.Abs(_vMax1);
        if (maxval2 > maxval)
        {
            maxval = maxval2;
        }
        newreso = _resoV1 * maxval;
        if (newreso > _resoV1)
        {
            _resoV1 = newreso;
        }
        //
        maxval = Math.Abs(_vMin2);
        maxval2 = Math.Abs(_vMax2);
        if (maxval2 > maxval)
        {
            maxval = maxval2;
        }
        newreso = _resoV2 * maxval;
        if (newreso > _resoV2)
        {
            _resoV2 = newreso;
        }
        //
        _pasuv[0] = _pasMax * Math.Abs(_uMax1 - _uMin1);
        _pasuv[1] = _pasMax * Math.Abs(_vMax1 - _vMin1);
        _pasuv[2] = _pasMax * Math.Abs(_uMax2 - _uMin2);
        _pasuv[3] = _pasMax * Math.Abs(_vMax2 - _vMin2);
        //
        if (!HSurfaceTool.IsUPeriodic(caro1))
        {
            _uMax1 += kelarg * _pasuv[0];
            _uMin1 -= kelarg * _pasuv[0];
        }
        else
        {
            double t = _uMax1 - _uMin1;
            if (t < HSurfaceTool.UPeriod(caro1))
            {
                t = 0.5 * (HSurfaceTool.UPeriod(caro1) - t);
                t = (t > kelarg * _pasuv[0]) ? kelarg * _pasuv[0] : t;
                _uMax1 += t;
                _uMin1 -= t;
            }
        }
        //
        if (!HSurfaceTool.IsVPeriodic(caro1))
        {
            _vMax1 += kelarg * _pasuv[1];
            _vMin1 -= kelarg * _pasuv[1];
        }
        else
        {
            double t = _vMax1 - _vMin1;
            if (t < HSurfaceTool.VPeriod(caro1))
            {
                t = 0.5 * (HSurfaceTool.VPeriod(caro1) - t);
                t = (t > kelarg * _pasuv[1]) ? kelarg * _pasuv[1] : t;
                _vMax1 += t;
                _vMin1 -= t;
            }
        }
        //
        if (!HSurfaceTool.IsUPeriodic(caro2))
        {
            _uMax2 += kelarg * _pasuv[2];
            _uMin2 -= kelarg * _pasuv[2];
        }
        else
        {
            double t = _uMax2 - _uMin2;
            if (t < HSurfaceTool.UPeriod(caro2))
            {
                t = 0.5 * (HSurfaceTool.UPeriod(caro2) - t);
                t = (t > kelarg * _pasuv[2]) ? kelarg * _pasuv[2] : t;
                _uMax2 += t;
                _uMin2 -= t;
            }
        }

        if (!HSurfaceTool.IsVPeriodic(caro2))
        {
            _vMax2 += kelarg * _pasuv[3];
            _vMin2 -= kelarg * _pasuv[3];
        }
        else
        {
            double t = _vMax2 - _vMin2;
            if (t < HSurfaceTool.VPeriod(caro2))
            {
                t = 0.5 * (HSurfaceTool.VPeriod(caro2) - t);
                t = (t > kelarg * _pasuv[3]) ? kelarg * _pasuv[3] : t;
                _vMax2 += t;
                _vMin2 -= t;
            }
        }
        //-- ComputePasInit(pasuv,Um1,UM1,Vm1,VM1,Um2,UM2,Vm2,VM2,Caro1,Caro2);

        for (int i = 0; i <= 3; i++)
        {
            _pasInit[i] = _pasSav[i] = _pasuv[i];
        }

        if (_resoU1 > 0.0001 * _pasuv[0])
        {
            _resoU1 = 0.00001 * _pasuv[0];
        }
        if (_resoV1 > 0.0001 * _pasuv[1])
        {
            _resoV1 = 0.00001 * _pasuv[1];
        }
        if (_resoU2 > 0.0001 * _pasuv[2])
        {
            _resoU2 = 0.00001 * _pasuv[2];
        }
        if (_resoV2 > 0.0001 * _pasuv[3])
        {
            _resoV2 = 0.00001 * _pasuv[3];
        }

        _stepMin[0] = 100.0 * _resoU1;
        _stepMin[1] = 100.0 * _resoV1;
        _stepMin[2] = 100.0 * _resoU2;
        _stepMin[3] = 100.0 * _resoV2;

        //
        NCollection_Array1<double> par = new(1, 4);
        par[1] = u1;
        par[2] = v1;
        par[3] = u2;
        par[4] = v2;
        Perform(par);
    }

    // =====================================================================================================
    // The accessors (IntWalk_PWalking.lxx)
    // =====================================================================================================

    /// <summary>True if the walking succeeded.</summary>
    public bool IsDone => _done;

    /// <summary>The number of points of the line. Throws StdFail_NotDone if not done.</summary>
    public int PointCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _line.PointCount;
        }
    }

    /// <summary>The point <paramref name="index"/> of the line. Throws StdFail_NotDone if not done.</summary>
    public PntOn2S Value(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _line.Value(index);
    }

    /// <summary>The line (the walking's own, not a copy). Throws StdFail_NotDone if not done.</summary>
    public LineOn2S Line()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _line;
    }

    public bool TangentAtFirst()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _tgfirst;
    }

    public bool TangentAtLast()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _tglast;
    }

    public bool IsClosed
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _close;
        }
    }

    /// <summary>The tangent of the line at the point <paramref name="index"/>.</summary>
    public Direction3D TangentAtLine(out int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        index = _tangentIdx;
        return _tgdir;
    }

    /// <summary>Appends a point to the line.</summary>
    public void AddAPoint(in PntOn2S pOn2S)
    {
        _line.Add(pOn2S);
        _tangentIdx = Math.Max(_tangentIdx, 1);
    }

    /// <summary>Removes the point <paramref name="index"/> (or the last one if beyond) from the line.</summary>
    public void RemoveAPoint(int index)
    {
        int idx = Math.Min(index, _line.PointCount);
        if (idx < 1)
        {
            return;
        }
        if (idx <= _tangentIdx)
        {
            _tangentIdx--;
            if (_tangentIdx < 1)
            {
                _tangentIdx = 1;
            }
        }
        _line.RemovePoint(idx);
    }

    /// <summary>The largest step along the parameter <paramref name="index"/> (0 to 3: u1, v1, u2, v2).</summary>
    public double MaxStep(int index)
    {
        Standard_OutOfRange_Raise_if((index < 0) || (index > 3),
                                     "IntWalk_PWalking::MaxStep() - index is out of range");
        return _pasInit[index];
    }

    // =====================================================================================================
    // IntWalk_PWalking.cxx
    // =====================================================================================================

    /// <summary>Estimates the largest steps, to avoid abrupt changes when the iso changes.</summary>
    internal void ComputePasInit(double deltaU1, double deltaV1, double deltaU2, double deltaV2)
    {
        const double rangePart = 0.01;
        double increment = 2.0 * _pasMax;
        SurfaceAdaptorBase caro1 = _intersectionOn2S.Function().AuxillarSurface1();
        SurfaceAdaptorBase caro2 = _intersectionOn2S.Function().AuxillarSurface2();

        double aDeltaU1 = Math.Abs(_uMax1 - _uMin1);
        double aDeltaV1 = Math.Abs(_vMax1 - _vMin1);
        double aDeltaU2 = Math.Abs(_uMax2 - _uMin2);
        double aDeltaV2 = Math.Abs(_vMax2 - _vMin2);

        //-- limit the reduction of uv box estimate to 0.01 natural box
        //--  theDeltaU1 : On box of Inter
        //-- aDeltaU1 : On parametric space
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(aDeltaU1))
        {
            _pasuv[0] = Math.Max(increment * Math.Max(deltaU1, rangePart * aDeltaU1), _pasuv[0]);
        }
        else
        {
            _pasuv[0] = Math.Max(increment * deltaU1, _pasuv[0]);
        }

        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(aDeltaV1))
        {
            _pasuv[1] = Math.Max(increment * Math.Max(deltaV1, rangePart * aDeltaV1), _pasuv[1]);
        }
        else
        {
            _pasuv[1] = Math.Max(increment * deltaV1, _pasuv[1]);
        }

        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(aDeltaU2))
        {
            _pasuv[2] = Math.Max(increment * Math.Max(deltaU2, rangePart * aDeltaU2), _pasuv[2]);
        }
        else
        {
            _pasuv[2] = Math.Max(increment * deltaU2, _pasuv[2]);
        }

        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(aDeltaV2))
        {
            _pasuv[3] = Math.Max(increment * Math.Max(deltaV2, rangePart * aDeltaV2), _pasuv[3]);
        }
        else
        {
            _pasuv[3] = Math.Max(increment * deltaV2, _pasuv[3]);
        }

        double resoU1tol = HSurfaceTool.UResolution(caro1, _tolconf);
        double resoV1tol = HSurfaceTool.VResolution(caro1, _tolconf);
        double resoU2tol = HSurfaceTool.UResolution(caro2, _tolconf);
        double resoV2tol = HSurfaceTool.VResolution(caro2, _tolconf);

        _stepMin[0] = Math.Max(_stepMin[0], 2.0 * resoU1tol);
        _stepMin[1] = Math.Max(_stepMin[1], 2.0 * resoV1tol);
        _stepMin[2] = Math.Max(_stepMin[2], 2.0 * resoU2tol);
        _stepMin[3] = Math.Max(_stepMin[3], 2.0 * resoV2tol);

        for (int i = 0; i < 4; i++)
        {
            _pasuv[i] = Math.Max(_stepMin[i], _pasuv[i]);
        }
    }

    /// <summary>
    /// Checks if theLine is parallel to a boundary of the surface given by theCheckSurf1 (oscillations of at most
    /// theToler). Lines too short to tell are classified as parallel.
    /// </summary>
    private static void IsParallel(LineOn2S line, bool checkSurf1, double toler,
                                   out bool isUparallel, out bool isVparallel)
    {
        const int nbPointsMAX = 23;

        isUparallel = isVparallel = true;

        int nbLinePnts = line.PointCount;
        int nbPoints = nbLinePnts;
        if (nbPoints > nbPointsMAX)
        {
            nbPoints = nbPointsMAX;
        }
        else if (nbPoints < 3)
        {
            // Here we cannot estimate parallelism.
            // Do all same as for small lines
            return;
        }

        double step = IntToReal(line.PointCount) / nbPoints;
        double nPoint = 1.0;

        double umin = RealLast(), umax = RealFirst(), vmin = RealLast(), vmax = RealFirst();
        for (int num = 1; num <= nbPoints; num++, nPoint += step)
        {
            // Fix possible "out of parameter" case.
            if (nPoint > nbLinePnts)
            {
                nPoint = nbLinePnts;
            }

            double u, v;
            if (checkSurf1)
            {
                line.Value(RealToInt(nPoint)).ParametersOnS1(out u, out v);
            }
            else
            {
                line.Value(RealToInt(nPoint)).ParametersOnS2(out u, out v);
            }

            if (u < umin)
            {
                umin = u;
            }

            if (u > umax)
            {
                umax = u;
            }

            if (v < vmin)
            {
                vmin = v;
            }

            if (v > vmax)
            {
                vmax = v;
            }
        }

        isVparallel = ((umax - umin) < toler);
        isUparallel = ((vmax - vmin) < toler);
    }

    /// <summary>
    /// Clamps theNbElem parameters (from theParam[theFirst]) to the borders (from index theFirstBorder); true if one
    /// changed. OCCT passes pointers into the arrays.
    /// </summary>
    private static bool AdjustToDomain(int nbElem, NCollection_Array1<double> param, int first,
                                       double[] lowBorder, double[] uppBorder, int firstBorder)
    {
        bool retVal = false;
        for (int i = 0; i < nbElem; i++)
        {
            ref double aParam = ref param[first + i];
            if ((aParam - lowBorder[firstBorder + i]) < -global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                aParam = lowBorder[firstBorder + i];
                retVal = true;
            }

            if ((aParam - uppBorder[firstBorder + i]) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                aParam = uppBorder[firstBorder + i];
                retVal = true;
            }
        }

        return retVal;
    }

    /// <summary>Computes the first point from ParDep, without walking.</summary>
    public bool PerformFirstPoint(NCollection_Array1<double> parDep, ref PntOn2S firstPoint)
    {
        _sensCheminement = 1;
        _close = false;
        //
        int i;
        NCollection_Array1<double> param = new(1, 4);
        //
        for (i = 1; i <= 4; ++i)
        {
            param[i] = parDep[i];
        }
        //-- calculate the first solution point
        FunctionSetRoot rsnld = new(_intersectionOn2S.Function());
        //
        _intersectionOn2S.Perform(param, rsnld);
        if (!_intersectionOn2S.IsDone)
        {
            return false;
        }

        if (_intersectionOn2S.IsEmpty)
        {
            return false;
        }

        firstPoint = _intersectionOn2S.Point();
        return true;
    }

    /// <summary>Walks from ParDep (u1, v1, u2, v2) in the surfaces' domains.</summary>
    public void Perform(NCollection_Array1<double> parDep)
    {
        Perform(parDep, _uMin1, _vMin1, _uMin2, _vMin2, _uMax1, _vMax1, _uMax2, _vMax2);
    }

    /// <summary>The square distance between thePnt and theSurf, from (theU0, theV0).</summary>
    private static double SQDistPointSurface(in Point3D pnt, SurfaceAdaptorBase surf, double u0, double v0)
    {
        GenLocateExtPS extPS = new(surf);
        extPS.Perform(pnt, u0, v0);

        if (!extPS.IsDone)
        {
            return RealLast();
        }

        return extPS.SquareDistance();
    }

    /// <summary>
    /// Additional check that the surfaces are tangent: whether points of each, around the start point, lie on the
    /// other (with the tolerance given).
    /// </summary>
    private static bool IsTangentExtCheck(SurfaceAdaptorBase surf1, SurfaceAdaptorBase surf2, double u10,
                                          double v10, double u20, double v20, double toler,
                                          double[] arrStep)
    {
        {
            surf1.D1(u10, v10, out Point3D pt, out Vector3D du1, out Vector3D dv1);
            surf2.D1(u20, v20, out pt, out Vector3D du2, out Vector3D dv2);

            Vector3D n1 = du1.Crossed(dv1), n2 = du2.Crossed(dv2);
            double dp = n1.Dot(n2), sq1 = n1.SquareMagnitude, sq2 = n2.SquareMagnitude;

            if ((sq1 < RealSmall()) || (sq2 < RealSmall()))
            {
                return true; // Tangent
            }

            if (dp * dp < 0.9998 * sq1 * sq2)
            { // cos(ang N1<->N2) < 0.9999
                return false; // Not tangent
            }
        }

        // For two faces (2^2 = 4)
        double sqToler = 4.0 * toler * toler;
        const int nbItems = 4;
        ReadOnlySpan<double> parUS1 = [u10 + arrStep[0], u10 - arrStep[0], u10, u10];
        ReadOnlySpan<double> parVS1 = [v10, v10, v10 + arrStep[1], v10 - arrStep[1]];
        ReadOnlySpan<double> parUS2 = [u20 + arrStep[2], u20 - arrStep[2], u20, u20];
        ReadOnlySpan<double> parVS2 = [v20, v20, v20 + arrStep[3], v20 - arrStep[3]];

        for (int i = 0; i < nbItems; i++)
        {
            Point3D p = surf1.Value(parUS1[i], parVS1[i]);
            double sqDist = SQDistPointSurface(p, surf2, u20, v20);
            if (sqDist > sqToler)
            {
                return false;
            }
        }

        for (int i = 0; i < nbItems; i++)
        {
            Point3D p = surf2.Value(parUS2[i], parVS2[i]);
            double sqDist = SQDistPointSurface(p, surf1, u10, v10);
            if (sqDist > sqToler)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>Writes the parameters of a point into Param(1..4).</summary>
    private static void SetParameters(NCollection_Array1<double> param, in PntOn2S point)
    {
        point.Parameters(out double u1, out double v1, out double u2, out double v2);
        param[1] = u1;
        param[2] = v1;
        param[3] = u2;
        param[4] = v2;
    }
}
