// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ProjectPointOnCurve.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ProjectPointOnCurve.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ProjectPointOnCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Api;

/// <summary>
/// Computes all the orthogonal projections of a 3D point onto a 3D curve (the extrema of the distance, through
/// <see cref="CurvePointExtrema"/>), and the nearest of them.
/// </summary>
/// <remarks>
/// A working object (decision 21). <see cref="Init(Curve3D, double, double)"/> followed by several
/// <see cref="Perform"/> calls reuses the loaded curve. OCCT's implicit conversions to <c>int</c>
/// (<see cref="NbPoints"/>), <c>gp_Pnt</c> (<see cref="NearestPoint"/>) and <c>double</c>
/// (<see cref="LowerDistance"/>) are explicit conversions (decision 28).
/// </remarks>
public class ProjectPointOnCurve
{
    private bool _isDone;
    private int _index;
    private readonly CurvePointExtrema _extPC = new();
    private readonly CurveAdaptor _c = new();

    /// <summary>An empty projector, to be initialised with <c>Init</c>.</summary>
    public ProjectPointOnCurve()
    {
        _isDone = false;
        _index = 0;
    }

    /// <summary>Projects <paramref name="p"/> onto the whole of <paramref name="curve"/>.</summary>
    public ProjectPointOnCurve(in Point3D p, Curve3D curve)
    {
        Init(p, curve);
    }

    /// <summary>Projects <paramref name="p"/> onto <paramref name="curve"/> on [Umin, Usup].</summary>
    public ProjectPointOnCurve(in Point3D p, Curve3D curve, double umin, double usup)
    {
        Init(p, curve, umin, usup);
    }

    /// <summary>Projects <paramref name="p"/> onto the whole of <paramref name="curve"/>.</summary>
    public void Init(in Point3D p, Curve3D curve)
    {
        _c.Load(curve);
        _extPC.Initialize(_c, _c.FirstParameter(), _c.LastParameter());
        _extPC.Perform(p);

        _isDone = _extPC.IsDone && (_extPC.ExtCount > 0);

        if (_isDone)
        {
            // evaluate the lower distance and its index;
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

    /// <summary>Projects <paramref name="p"/> onto <paramref name="curve"/> on [Umin, Usup].</summary>
    public void Init(in Point3D p, Curve3D curve, double umin, double usup)
    {
        _c.Load(curve, umin, usup);
        _extPC.Initialize(_c, _c.FirstParameter(), _c.LastParameter());
        _extPC.Perform(p);

        _isDone = _extPC.IsDone && (_extPC.ExtCount > 0);

        if (_isDone)
        {
            // evaluate the lower distance and its index;
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

    /// <summary>Loads <paramref name="curve"/> on [Umin, Usup], for projections by <see cref="Perform"/>.</summary>
    public void Init(Curve3D curve, double umin, double usup)
    {
        _c.Load(curve, umin, usup);
        _extPC.Initialize(_c, umin, usup);
        _isDone = false;
    }

    /// <summary>Projects <paramref name="aP3D"/> onto the curve loaded by <c>Init</c>.</summary>
    public void Perform(in Point3D aP3D)
    {
        _extPC.Perform(aP3D);

        _isDone = _extPC.IsDone && (_extPC.ExtCount > 0);
        if (_isDone)
        {
            // evaluate the lower distance and its index;
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

    /// <summary>The number of projections (0 when the computation failed).</summary>
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

    /// <summary>The projection of index <paramref name="index"/> (1..NbPoints).</summary>
    public Point3D Point(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbPoints(), "GeomAPI_ProjectPointOnCurve::Point");
        return (_extPC.Point(index)).Value();
    }

    /// <summary>The curve parameter of the projection of index <paramref name="index"/>.</summary>
    public double Parameter(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbPoints(), "GeomAPI_ProjectPointOnCurve::Parameter");
        return (_extPC.Point(index)).Parameter();
    }

    /// <summary>The curve parameter of the projection of index <paramref name="index"/>.</summary>
    public void Parameter(int index, out double u)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbPoints(), "GeomAPI_ProjectPointOnCurve::Parameter");
        u = (_extPC.Point(index)).Parameter();
    }

    /// <summary>The distance from the point to the projection of index <paramref name="index"/>.</summary>
    public double Distance(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbPoints(), "GeomAPI_ProjectPointOnCurve::Distance");
        return Math.Sqrt(_extPC.SquareDistance(index));
    }

    /// <summary>The nearest projection. Throws StdFail_NotDone if there is none.</summary>
    public Point3D NearestPoint()
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ProjectPointOnCurve::NearestPoint");

        return (_extPC.Point(_index)).Value();
    }

    /// <summary>OCCT's <c>operator int</c>: <see cref="NbPoints"/>.</summary>
    public static explicit operator int(ProjectPointOnCurve proj) => proj.NbPoints();

    /// <summary>OCCT's <c>operator gp_Pnt</c>: <see cref="NearestPoint"/>.</summary>
    public static explicit operator Point3D(ProjectPointOnCurve proj) => proj.NearestPoint();

    /// <summary>The parameter of the nearest projection. Throws StdFail_NotDone if there is none.</summary>
    public double LowerDistanceParameter()
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ProjectPointOnCurve::LowerDistanceParameter");

        return (_extPC.Point(_index)).Parameter();
    }

    /// <summary>The distance to the nearest projection. Throws StdFail_NotDone if there is none.</summary>
    public double LowerDistance()
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ProjectPointOnCurve::LowerDistance");

        return Math.Sqrt(_extPC.SquareDistance(_index));
    }

    /// <summary>OCCT's <c>operator double</c>: <see cref="LowerDistance"/>.</summary>
    public static explicit operator double(ProjectPointOnCurve proj) => proj.LowerDistance();

    /// <summary>The extrema algorithm, for advanced use.</summary>
    public CurvePointExtrema Extrema() => _extPC;
}
