// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_ProjectPointOnCurve.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_ProjectPointOnCurve.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dAPI/Geom2dAPI_ProjectPointOnCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Api;

/// <summary>All the orthogonal projections of a 2D point onto a 2D curve.</summary>
/// <remarks>
/// An algorithm object (decision 117). OCCT copies the Extrema_ExtPC2d it builds into its field; the port keeps
/// the object it builds (decision 224). OCCT's implicit conversions to int, gp_Pnt2d and double are explicit
/// operators (decision 327).
/// </remarks>
public class ProjectPointOnCurve2D
{
    private bool _isDone;
    private int _index;
    private ExtPC2d _extPC = new();
    private readonly CurveAdaptor2D _c = new();

    /// <summary>An empty projector; use <see cref="Init(in Point2D, Curve2D)"/>.</summary>
    public ProjectPointOnCurve2D()
    {
        _index = -1;
        _isDone = false;
    }

    /// <summary>The projections of <paramref name="p"/> on <paramref name="curve"/>.</summary>
    public ProjectPointOnCurve2D(in Point2D p, Curve2D curve)
    {
        Init(p, curve);
    }

    /// <summary>
    /// The projections of <paramref name="p"/> on the part of <paramref name="curve"/> between the parameters
    /// <paramref name="umin"/> and <paramref name="usup"/>.
    /// </summary>
    public ProjectPointOnCurve2D(in Point2D p, Curve2D curve, double umin, double usup)
    {
        Init(p, curve, umin, usup);
    }

    /// <summary>Computes the projections of <paramref name="p"/> on <paramref name="curve"/>.</summary>
    public void Init(in Point2D p, Curve2D curve)
    {
        Init(p, curve, curve.FirstParameter(), curve.LastParameter());
    }

    /// <summary>
    /// Computes the projections of <paramref name="p"/> on the part of <paramref name="curve"/> between the
    /// parameters <paramref name="umin"/> and <paramref name="usup"/>.
    /// </summary>
    public void Init(in Point2D p, Curve2D curve, double umin, double usup)
    {
        _c.Load(curve, umin, usup);
        ExtPC2d theExtPC2d = new(p, _c);
        _extPC = theExtPC2d;
        _isDone = _extPC.IsDone && (_extPC.ExtCount > 0);

        // evaluate the lower distance and its index;
        if (_isDone)
        {
            double dist2, dist2Min = _extPC.SquareDistance(1);
            _index = 1;

            for (int i = 2; i <= _extPC.ExtCount; i++)
            {
                dist2 = _extPC.SquareDistance(i);
                if (dist2 < dist2Min)
                {
                    dist2Min = dist2;
                    _index = i;
                }
            }
        }
    }

    /// <summary>The number of projections (0 if the computation failed).</summary>
    public int NbPoints()
    {
        if (_isDone)
        {
            return _extPC.ExtCount;
        }
        else
        {
            return 0;
        }
    }

    /// <summary>The projection of index <paramref name="index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="index"/> is not in [1, NbPoints].</exception>
    public Point2D Point(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbPoints(), "Geom2dAPI_ProjectPointOnCurve::Point");
        return (_extPC.Point(index)).Value();
    }

    /// <summary>The parameter on the curve of the projection of index <paramref name="index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="index"/> is not in [1, NbPoints].</exception>
    public double Parameter(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbPoints(), "Geom2dAPI_ProjectPointOnCurve::Parameter");
        return (_extPC.Point(index)).Parameter();
    }

    /// <summary>The parameter on the curve of the projection of index <paramref name="index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="index"/> is not in [1, NbPoints].</exception>
    public void Parameter(int index, out double u)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbPoints(), "Geom2dAPI_ProjectPointOnCurve::Parameter");
        u = (_extPC.Point(index)).Parameter();
    }

    /// <summary>The distance between the point and its projection of index <paramref name="index"/>.</summary>
    /// <exception cref="OutOfRangeException"><paramref name="index"/> is not in [1, NbPoints].</exception>
    public double Distance(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbPoints(), "Geom2dAPI_ProjectPointOnCurve::Distance");
        return Math.Sqrt(_extPC.SquareDistance(index));
    }

    /// <summary>The nearest projection of the point on the curve.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public Point2D NearestPoint()
    {
        StdFail_NotDone_Raise_if(!_isDone, "Geom2dAPI_ProjectPointOnCurve:NearestPoint");
        return (_extPC.Point(_index)).Value();
    }

    /// <summary>The parameter on the curve of the nearest projection.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double LowerDistanceParameter()
    {
        StdFail_NotDone_Raise_if(!_isDone, "Geom2dAPI_ProjectPointOnCurve:LowerDistanceParameter");
        return (_extPC.Point(_index)).Parameter();
    }

    /// <summary>The distance between the point and its nearest projection.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double LowerDistance()
    {
        StdFail_NotDone_Raise_if(!_isDone, "Geom2dAPI_ProjectPointOnCurve:LowerDistance");
        return Math.Sqrt(_extPC.SquareDistance(_index));
    }

    /// <summary>The Extrema algorithm used.</summary>
    public ExtPC2d Extrema() => _extPC;

    /// <summary>OCCT's <c>operator int()</c>: <see cref="NbPoints"/>.</summary>
    public static explicit operator int(ProjectPointOnCurve2D proj) => proj.NbPoints();

    /// <summary>OCCT's <c>operator gp_Pnt2d()</c>: <see cref="NearestPoint"/>.</summary>
    public static explicit operator Point2D(ProjectPointOnCurve2D proj) => proj.NearestPoint();

    /// <summary>OCCT's <c>operator double()</c>: <see cref="LowerDistance"/>.</summary>
    public static explicit operator double(ProjectPointOnCurve2D proj) => proj.LowerDistance();
}
