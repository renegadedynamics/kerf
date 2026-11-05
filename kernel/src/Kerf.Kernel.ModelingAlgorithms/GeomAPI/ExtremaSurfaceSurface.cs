// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaSurfaceSurface.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaSurfaceSurface.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaSurfaceSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Api;

/// <summary>
/// Computes the extrema of the distance between two surfaces (through <see cref="ExtSS"/>), and the nearest
/// pair among them.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT constructs an <c>Extrema_ExtSS</c> in each <c>Init</c> and copies it into
/// its member; the port keeps the constructed object (decision 224). OCCT's implicit conversions to <c>int</c> and
/// <c>double</c> are explicit conversions (decision 28).
/// </remarks>
internal class ExtremaSurfaceSurface
{
    private bool _isDone;
    private int _index;
    private ExtSS _extSS = new();

    /// <summary>An empty algorithm, to be initialised with <c>Init</c>.</summary>
    public ExtremaSurfaceSurface()
    {
        _isDone = false;
        _index = 0;
    }

    /// <summary>The extrema between the whole surfaces <paramref name="s1"/> and <paramref name="s2"/>.</summary>
    public ExtremaSurfaceSurface(Surface s1, Surface s2)
    {
        Init(s1, s2);
    }

    /// <summary>The extrema between <paramref name="s1"/> and <paramref name="s2"/> on the given ranges.</summary>
    public ExtremaSurfaceSurface(Surface s1, Surface s2, double u1min, double u1max, double v1min,
                                         double v1max, double u2min, double u2max, double v2min, double v2max)
    {
        Init(s1, s2, u1min, u1max, v1min, v1max, u2min, u2max, v2min, v2max);
    }

    /// <summary>Computes the extrema between the whole surfaces <paramref name="s1"/> and <paramref name="s2"/>.</summary>
    public void Init(Surface s1, Surface s2)
    {
        SurfaceAdaptor theSurface1 = new(s1);
        SurfaceAdaptor theSurface2 = new(s2);

        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();

        ExtSS theExtSS = new(theSurface1, theSurface2, tol, tol);

        _extSS = theExtSS;
        _isDone = _extSS.IsDone && (_extSS.ExtCount > 0);

        if (_isDone)
        {
            // evaluate the lower distance and its index;
            double dist2, dist2Min = _extSS.SquareDistance(1);
            _index = 1;

            for (int i = 2; i <= _extSS.ExtCount; i++)
            {
                dist2 = _extSS.SquareDistance(i);
                if (dist2 < dist2Min)
                {
                    dist2Min = dist2;
                    _index = i;
                }
            }
        }
    }

    /// <summary>Computes the extrema between <paramref name="s1"/> and <paramref name="s2"/> on the given ranges.</summary>
    public void Init(Surface s1, Surface s2, double u1min, double u1max, double v1min, double v1max,
                     double u2min, double u2max, double v2min, double v2max)
    {
        SurfaceAdaptor theSurface1 = new(s1, u1min, u1max, v1min, v1max);
        SurfaceAdaptor theSurface2 = new(s2, u2min, u2max, v2min, v2max);

        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        ExtSS theExtSS = new(theSurface1, theSurface2, u1min, u1max, v1min, v1max, u2min, u2max, v2min, v2max,
                                     tol, tol);

        _extSS = theExtSS;
        _isDone = _extSS.IsDone && (_extSS.ExtCount > 0);

        if (_isDone)
        {
            // evaluate the lower distance and its index;
            double dist2, dist2Min = _extSS.SquareDistance(1);
            _index = 1;

            for (int i = 2; i <= _extSS.ExtCount; i++)
            {
                dist2 = _extSS.SquareDistance(i);
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
            return _extSS.ExtCount;
        }
        else
        {
            return 0;
        }
    }

    /// <summary>OCCT's <c>operator int</c>: <see cref="NbExtrema"/>.</summary>
    public static explicit operator int(ExtremaSurfaceSurface ext) => ext.NbExtrema();

    /// <summary>The points of the extremum of index <paramref name="index"/>, on the first and second surface.</summary>
    public void Points(int index, out Point3D p1, out Point3D p2)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Points");

        _extSS.Points(index, out POnSurf ps1, out POnSurf ps2);

        p1 = ps1.Value();
        p2 = ps2.Value();
    }

    /// <summary>The parameters of the extremum of index <paramref name="index"/> on the two surfaces.</summary>
    public void Parameters(int index, out double u1, out double v1, out double u2, out double v2)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Parameters");

        _extSS.Points(index, out POnSurf ps1, out POnSurf ps2);

        ps1.Parameter(out u1, out v1);
        ps2.Parameter(out u2, out v2);
    }

    /// <summary>The distance of the extremum of index <paramref name="index"/>.</summary>
    public double Distance(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Distance");

        return Math.Sqrt(_extSS.SquareDistance(index));
    }

    /// <summary>True if the surfaces are parallel.</summary>
    public bool IsParallel => _extSS.IsParallel;

    /// <summary>The points of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public void NearestPoints(out Point3D p1, out Point3D p2)
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ExtremaSurfaceSurface::NearestPoints");

        Points(_index, out p1, out p2);
    }

    /// <summary>The parameters of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public void LowerDistanceParameters(out double u1, out double v1, out double u2, out double v2)
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ExtremaSurfaceSurface::LowerDistanceParameters");

        Parameters(_index, out u1, out v1, out u2, out v2);
    }

    /// <summary>The distance of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public double LowerDistance()
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ExtremaSurfaceSurface::LowerDistance");

        return Math.Sqrt(_extSS.SquareDistance(_index));
    }

    /// <summary>OCCT's <c>operator double</c>: <see cref="LowerDistance"/>.</summary>
    public static explicit operator double(ExtremaSurfaceSurface ext) => ext.LowerDistance();

    /// <summary>The extrema algorithm, for advanced use.</summary>
    public ExtSS Extrema() => _extSS;
}
