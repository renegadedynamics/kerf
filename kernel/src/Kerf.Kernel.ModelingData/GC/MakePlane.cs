// Port of: src/ModelingData/TKGeomBase/GC/GC_MakePlane.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakePlane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a Geom_Plane: from a gp_Pln, a point and a normal, a cartesian equation, three points, parallel
/// to a plane at a distance or through a point, or normal to an axis.
/// </summary>
internal sealed class MakePlane : Root
{
    private PlaneSurface? _thePlane;

    /// <summary>A plane from the gp_Pln Pl.</summary>
    public MakePlane(in Plane3D pl)
    {
        _theError = Done;
        _thePlane = new PlaneSurface(pl);
    }

    /// <summary>A plane through P with normal V.</summary>
    public MakePlane(in Point3D p, in Direction3D v)
    {
        _theError = Done;
        _thePlane = new PlaneSurface(p, v);
    }

    /// <summary>The plane A·X + B·Y + C·Z + D = 0; fails with gce_BadEquation if (A, B, C) is null.</summary>
    public MakePlane(double a, double b, double c, double d)
    {
        if (Math.Sqrt(a * a + b * b + c * c) <= Constants.Resolution())
        {
            _theError = BadEquation;
        }
        else
        {
            _theError = Done;
            _thePlane = new PlaneSurface(new Plane3D(a, b, c, d));
        }
    }

    /// <summary>The plane through three points; fails with the status of gce_MakePln.</summary>
    public MakePlane(in Point3D p1, in Point3D p2, in Point3D p3)
    {
        MakePln pl = new MakePln(p1, p2, p3);
        _theError = pl.Status();
        if (_theError == Done)
        {
            _thePlane = new PlaneSurface(pl.Value());
        }
    }

    /// <summary>A plane parallel to Pl at the signed distance Dist.</summary>
    public MakePlane(in Plane3D pl, double dist)
    {
        MakePln maker = new MakePln(pl, dist);
        _theError = maker.Status();
        if (_theError == Done)
        {
            _thePlane = new PlaneSurface(maker.Value());
        }
    }

    /// <summary>A plane parallel to Pl through Point.</summary>
    public MakePlane(in Plane3D pl, in Point3D point)
    {
        MakePln maker = new MakePln(pl, point);
        _theError = maker.Status();
        if (_theError == Done)
        {
            _thePlane = new PlaneSurface(maker.Value());
        }
    }

    /// <summary>The plane through the location of Axis, normal to it.</summary>
    public MakePlane(in Axis3D axis)
    {
        MakePln maker = new MakePln(axis);
        _theError = maker.Status();
        if (_theError == Done)
        {
            _thePlane = new PlaneSurface(maker.Value());
        }
    }

    /// <summary>The constructed plane (frozen).</summary>
    /// <exception cref="NotDoneException">No plane was constructed.</exception>
    public PlaneSurface Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakePlane::Value() - no result");
        _thePlane!.Freeze();
        return _thePlane;
    }

    /// <summary>The constructed plane (OCCT's implicit conversion).</summary>
    public static explicit operator PlaneSurface(MakePlane maker) => maker.Value();
}
