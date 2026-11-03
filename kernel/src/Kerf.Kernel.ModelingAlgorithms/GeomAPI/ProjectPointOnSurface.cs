// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ProjectPointOnSurf.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ProjectPointOnSurf.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ProjectPointOnSurf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Extrema.ExtAlgo;

namespace Kerf.Engine.Geometry.Api;

/// <summary>
/// Computes all the orthogonal projections of a point onto a surface (the extrema of the distance, through
/// <see cref="ExtPS"/>), and the nearest of them.
/// </summary>
/// <remarks>
/// A working object (decision 21). Its <see cref="ExtPS"/> is created once and kept: each <c>Init</c>
/// loads the surface into the projector's own <see cref="SurfaceAdaptor"/> (which the ExtPS keeps by
/// reference), sets the algorithm and initialises; a flag set with <see cref="SetExtremaFlag"/> is kept across
/// <c>Init</c>, as in OCCT. OCCT's implicit conversions to <c>int</c>, <c>gp_Pnt</c> and <c>double</c> are explicit
/// conversions (decision 28).
/// </remarks>
public class ProjectPointOnSurface
{
    private bool _isDone;
    private int _index;
    private readonly ExtPS _extPS = new();
    private readonly SurfaceAdaptor _geomAdaptor = new();

    /// <summary>An empty projector, to be initialised with <c>Init</c>.</summary>
    public ProjectPointOnSurface()
    {
        _isDone = false;
        _index = 0;
    }

    /// <summary>Projects <paramref name="p"/> onto the whole of <paramref name="surface"/>.</summary>
    public ProjectPointOnSurface(in Point3D p, Surface surface, ExtAlgo projAlgo = ExtAlgo_Grad)
    {
        Init(p, surface, projAlgo);
    }

    /// <summary>Projects <paramref name="p"/> onto the whole of <paramref name="surface"/> with a tolerance.</summary>
    public ProjectPointOnSurface(in Point3D p, Surface surface, double tolerance,
                                      ExtAlgo projAlgo = ExtAlgo_Grad)
    {
        Init(p, surface, tolerance, projAlgo);
    }

    /// <summary>Projects <paramref name="p"/> onto <paramref name="surface"/> on [Umin, Usup] x [Vmin, Vsup].</summary>
    public ProjectPointOnSurface(in Point3D p, Surface surface, double umin, double usup, double vmin,
                                      double vsup, ExtAlgo projAlgo = ExtAlgo_Grad)
    {
        Init(p, surface, umin, usup, vmin, vsup, projAlgo);
    }

    /// <summary>
    /// Projects <paramref name="p"/> onto <paramref name="surface"/> on [Umin, Usup] x [Vmin, Vsup] with a tolerance.
    /// </summary>
    public ProjectPointOnSurface(in Point3D p, Surface surface, double umin, double usup, double vmin,
                                      double vsup, double tolerance, ExtAlgo projAlgo = ExtAlgo_Grad)
    {
        Init(p, surface, umin, usup, vmin, vsup, tolerance, projAlgo);
    }

    private void Init()
    {
        _isDone = _extPS.IsDone && (_extPS.ExtCount > 0);

        if (_isDone)
        {
            // evaluate the lower distance and its index;
            double dist2, dist2Min = _extPS.SquareDistance(1);
            _index = 1;

            for (int i = 2; i <= _extPS.ExtCount; i++)
            {
                dist2 = _extPS.SquareDistance(i);
                if (dist2 < dist2Min)
                {
                    dist2Min = dist2;
                    _index = i;
                }
            }
        }
    }

    /// <summary>Projects <paramref name="p"/> onto the whole of <paramref name="surface"/>.</summary>
    public void Init(in Point3D p, Surface surface, ExtAlgo projAlgo = ExtAlgo_Grad)
    {
        Init(p, surface, global::Kerf.Engine.Foundation.Precision.Confusion(), projAlgo);
    }

    /// <summary>Projects <paramref name="p"/> onto the whole of <paramref name="surface"/> with a tolerance.</summary>
    public void Init(in Point3D p, Surface surface, double tolerance, ExtAlgo projAlgo = ExtAlgo_Grad)
    {
        surface.ParameterBounds(out double umin, out double usup, out double vmin, out double vsup);
        _geomAdaptor.Load(surface, umin, usup, vmin, vsup);
        _extPS.SetAlgo(projAlgo);
        _extPS.Initialize(_geomAdaptor, umin, usup, vmin, vsup, tolerance, tolerance);
        _extPS.Perform(p);
        Init();
    }

    /// <summary>Projects <paramref name="p"/> onto <paramref name="surface"/> on [Umin, Usup] x [Vmin, Vsup].</summary>
    public void Init(in Point3D p, Surface surface, double umin, double usup, double vmin, double vsup,
                     ExtAlgo projAlgo = ExtAlgo_Grad)
    {
        double tolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _geomAdaptor.Load(surface, umin, usup, vmin, vsup);
        _extPS.SetAlgo(projAlgo);
        _extPS.Initialize(_geomAdaptor, umin, usup, vmin, vsup, tolerance, tolerance);
        _extPS.Perform(p);
        Init();
    }

    /// <summary>
    /// Projects <paramref name="p"/> onto <paramref name="surface"/> on [Umin, Usup] x [Vmin, Vsup] with a tolerance.
    /// </summary>
    public void Init(in Point3D p, Surface surface, double umin, double usup, double vmin, double vsup,
                     double tolerance, ExtAlgo projAlgo = ExtAlgo_Grad)
    {
        _geomAdaptor.Load(surface, umin, usup, vmin, vsup);
        _extPS.SetAlgo(projAlgo);
        _extPS.Initialize(_geomAdaptor, umin, usup, vmin, vsup, tolerance, tolerance);
        _extPS.Perform(p);
        Init();
    }

    /// <summary>Loads <paramref name="surface"/> on [Umin, Usup] x [Vmin, Vsup], for projections by <see cref="Perform"/>.</summary>
    public void Init(Surface surface, double umin, double usup, double vmin, double vsup,
                     ExtAlgo projAlgo = ExtAlgo_Grad)
    {
        double tolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
        _geomAdaptor.Load(surface, umin, usup, vmin, vsup);
        _extPS.SetAlgo(projAlgo);
        _extPS.Initialize(_geomAdaptor, umin, usup, vmin, vsup, tolerance, tolerance);
        _isDone = false;
    }

    /// <summary>
    /// Loads <paramref name="surface"/> on [Umin, Usup] x [Vmin, Vsup] with a tolerance, for projections by
    /// <see cref="Perform"/>.
    /// </summary>
    public void Init(Surface surface, double umin, double usup, double vmin, double vsup, double tolerance,
                     ExtAlgo projAlgo = ExtAlgo_Grad)
    {
        _geomAdaptor.Load(surface, umin, usup, vmin, vsup);
        _extPS.SetAlgo(projAlgo);
        _extPS.Initialize(_geomAdaptor, umin, usup, vmin, vsup, tolerance, tolerance);
        _isDone = false;
    }

    /// <summary>Sets the extrema algorithm (gradient or tree) for the next <c>Init</c>.</summary>
    public void SetExtremaAlgo(ExtAlgo algo) => _extPS.SetAlgo(algo);

    /// <summary>Sets which extrema to compute (minima, maxima or both); kept across <c>Init</c>.</summary>
    public void SetExtremaFlag(ExtFlag extFlag) => _extPS.SetFlag(extFlag);

    /// <summary>Projects <paramref name="p"/> onto the surface loaded by <c>Init</c>.</summary>
    public void Perform(in Point3D p)
    {
        _extPS.Perform(p);
        Init();
    }

    /// <summary>True if at least one projection was found.</summary>
    public bool IsDone => _isDone;

    /// <summary>The number of projections (0 when the computation failed).</summary>
    public int NbPoints()
    {
        if (_isDone)
        {
            return _extPS.ExtCount;
        }
        else
        {
            return 0;
        }
    }

    /// <summary>The projection of index <paramref name="index"/> (1..NbPoints).</summary>
    public Point3D Point(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbPoints(), "GeomAPI_ProjectPointOnSurf::Point");
        return (_extPS.Point(index)).Value();
    }

    /// <summary>The surface parameters of the projection of index <paramref name="index"/>.</summary>
    public void Parameters(int index, out double u, out double v)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbPoints(), "GeomAPI_ProjectPointOnSurf::Parameter");
        (_extPS.Point(index)).Parameter(out u, out v);
    }

    /// <summary>The distance from the point to the projection of index <paramref name="index"/>.</summary>
    public double Distance(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > NbPoints(), "GeomAPI_ProjectPointOnSurf::Distance");
        return Math.Sqrt(_extPS.SquareDistance(index));
    }

    /// <summary>The nearest projection. Throws StdFail_NotDone if there is none.</summary>
    public Point3D NearestPoint()
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ProjectPointOnSurf::NearestPoint");

        return (_extPS.Point(_index)).Value();
    }

    /// <summary>OCCT's <c>operator int</c>: <see cref="NbPoints"/>.</summary>
    public static explicit operator int(ProjectPointOnSurface proj) => proj.NbPoints();

    /// <summary>OCCT's <c>operator gp_Pnt</c>: <see cref="NearestPoint"/>.</summary>
    public static explicit operator Point3D(ProjectPointOnSurface proj) => proj.NearestPoint();

    /// <summary>The parameters of the nearest projection. Throws StdFail_NotDone if there is none.</summary>
    public void LowerDistanceParameters(out double u, out double v)
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ProjectPointOnSurf::LowerDistanceParameters");

        (_extPS.Point(_index)).Parameter(out u, out v);
    }

    /// <summary>The distance to the nearest projection. Throws StdFail_NotDone if there is none.</summary>
    public double LowerDistance()
    {
        StdFail_NotDone_Raise_if(!_isDone, "GeomAPI_ProjectPointOnSurf::LowerDistance");

        return Math.Sqrt(_extPS.SquareDistance(_index));
    }

    /// <summary>OCCT's <c>operator double</c>: <see cref="LowerDistance"/>.</summary>
    public static explicit operator double(ProjectPointOnSurface proj) => proj.LowerDistance();

    /// <summary>The extrema algorithm, for advanced use.</summary>
    public ExtPS Extrema() => _extPS;
}
