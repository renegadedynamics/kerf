// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaCurveSurface.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaCurveSurface.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaCurveSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Api;

/// <summary>
/// Computes the extrema of the distance between a 3D curve and a surface (through <see cref="ExtCS"/>), and
/// the nearest pair among them.
/// </summary>
/// <remarks>
/// A working object (decision 21). The adaptors are made by <c>Init</c>, as OCCT's locals; the extrema algorithm
/// keeps the surface adaptor (in OCCT a pointer to a destroyed local). OCCT's implicit conversions to <c>int</c>
/// and <c>double</c> are explicit conversions (decision 28).
/// </remarks>
public class ExtremaCurveSurface
{
    private bool _isDone;
    private int _index;
    private readonly ExtCS _extCS = new();

    /// <summary>An empty algorithm, to be initialised with <c>Init</c>.</summary>
    public ExtremaCurveSurface()
    {
        _isDone = false;
        _index = 0;
    }

    /// <summary>The extrema between the whole <paramref name="curve"/> and <paramref name="surface"/>.</summary>
    public ExtremaCurveSurface(Curve3D curve, Surface surface)
    {
        Init(curve, surface);
    }

    /// <summary>
    /// The extrema between <paramref name="curve"/> on [Wmin, Wmax] and <paramref name="surface"/> on
    /// [Umin, Umax] x [Vmin, Vmax].
    /// </summary>
    public ExtremaCurveSurface(Curve3D curve, Surface surface, double wmin, double wmax, double umin,
                                       double umax, double vmin, double vmax)
    {
        Init(curve, surface, wmin, wmax, umin, umax, vmin, vmax);
    }

    /// <summary>Computes the extrema between the whole <paramref name="curve"/> and <paramref name="surface"/>.</summary>
    public void Init(Curve3D curve, Surface surface)
    {
        CurveAdaptor theCurve = new(curve);
        SurfaceAdaptor theSurface = new(surface);

        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _extCS.Initialize(theSurface, tol, tol);
        _extCS.Perform(theCurve, theCurve.FirstParameter(), theCurve.LastParameter());
        _isDone = _extCS.IsDone && (_extCS.IsParallel || _extCS.ExtCount > 0);

        if (_isDone)
        {
            // evaluate the lower distance and its index;
            double dist2, dist2Min = _extCS.SquareDistance(1);
            _index = 1;

            for (int i = 2; i <= _extCS.ExtCount; i++)
            {
                dist2 = _extCS.SquareDistance(i);
                if (dist2 < dist2Min)
                {
                    dist2Min = dist2;
                    _index = i;
                }
            }
        }
    }

    /// <summary>
    /// Computes the extrema between <paramref name="curve"/> on [Wmin, Wmax] and <paramref name="surface"/> on
    /// [Umin, Umax] x [Vmin, Vmax].
    /// </summary>
    public void Init(Curve3D curve, Surface surface, double wmin, double wmax, double umin, double umax,
                     double vmin, double vmax)
    {
        CurveAdaptor theCurve = new(curve, wmin, wmax);
        SurfaceAdaptor theSurface = new(surface, umin, umax, vmin, vmax);

        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _extCS.Initialize(theSurface, umin, umax, vmin, vmax, tol, tol);
        _extCS.Perform(theCurve, wmin, wmax);
        _isDone = _extCS.IsDone && (_extCS.IsParallel || _extCS.ExtCount > 0);

        if (_isDone)
        {
            // evaluate the lower distance and its index;
            double dist2, dist2Min = _extCS.SquareDistance(1);
            _index = 1;

            for (int i = 2; i <= _extCS.ExtCount; i++)
            {
                dist2 = _extCS.SquareDistance(i);
                if (dist2 < dist2Min)
                {
                    dist2Min = dist2;
                    _index = i;
                }
            }
        }
    }

    /// <summary>The number of extrema (0 when the computation failed).</summary>
    public int NbExtrema()
    {
        if (_isDone)
        {
            return _extCS.ExtCount;
        }
        else
        {
            return 0;
        }
    }

    /// <summary>OCCT's <c>operator int</c>: <see cref="NbExtrema"/>.</summary>
    public static explicit operator int(ExtremaCurveSurface ext) => ext.NbExtrema();

    /// <summary>The points of the extremum of index <paramref name="index"/>, on the curve and on the surface.</summary>
    public void Points(int index, out Point3D p1, out Point3D p2)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Points");

        _extCS.Points(index, out POnCurv pc, out POnSurf ps);

        p1 = pc.Value();
        p2 = ps.Value();
    }

    /// <summary>The curve parameter and surface parameters of the extremum of index <paramref name="index"/>.</summary>
    public void Parameters(int index, out double w, out double u, out double v)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Parameters");

        _extCS.Points(index, out POnCurv pc, out POnSurf ps);

        w = pc.Parameter();
        ps.Parameter(out u, out v);
    }

    /// <summary>The distance of the extremum of index <paramref name="index"/>.</summary>
    public double Distance(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Distance");

        return Math.Sqrt(_extCS.SquareDistance(index));
    }

    /// <summary>True if the curve lies on a surface parallel to the surface.</summary>
    public bool IsParallel => _extCS.IsParallel;

    /// <summary>The points of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public void NearestPoints(out Point3D pc, out Point3D ps)
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ExtremaCurveSurface::NearestPoints");

        Points(_index, out pc, out ps);
    }

    /// <summary>The parameters of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public void LowerDistanceParameters(out double w, out double u, out double v)
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ExtremaCurveSurface::LowerDistanceParameters");

        Parameters(_index, out w, out u, out v);
    }

    /// <summary>The distance of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public double LowerDistance()
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ExtremaCurveSurface::LowerDistance");

        return Math.Sqrt(_extCS.SquareDistance(_index));
    }

    /// <summary>OCCT's <c>operator double</c>: <see cref="LowerDistance"/>.</summary>
    public static explicit operator double(ExtremaCurveSurface ext) => ext.LowerDistance();

    /// <summary>The extrema algorithm, for advanced use.</summary>
    public ExtCS Extrema() => _extCS;
}
