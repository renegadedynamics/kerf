// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeCircle.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeCircle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a Geom_Circle: from a gp_Circ, an axis system and a radius, coaxial to a circle at a distance or
/// through a point, through three points, from a center and a normal, a center and a point on the axis, or an axis.
/// </summary>
internal sealed class MakeCircle : Root
{
    private CircleCurve3D? _theCircle;

    /// <summary>A circle from the gp_Circ C.</summary>
    public MakeCircle(in Circle3D c)
    {
        _theError = Done;
        _theCircle = new CircleCurve3D(c);
    }

    /// <summary>A circle of radius Radius in the plane of A2; fails with gce_NegativeRadius if Radius is negative.</summary>
    public MakeCircle(in Frame3D a2, double radius)
    {
        if (radius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            _theError = Done;
            _theCircle = new CircleCurve3D(new Circle3D(a2, radius));
        }
    }

    /// <summary>A circle coaxial to Circ through Point.</summary>
    public MakeCircle(in Circle3D circ, in Point3D point)
    {
        // gce_MakeCirc(Circ, Point) always succeeds, so its implicit conversion can't throw.
        Circle3D c = new MakeCirc(circ, point).Value();
        _theCircle = new CircleCurve3D(c);
        _theError = Done;
    }

    /// <summary>A circle coaxial to Circ at the signed distance Dist; fails with gce_NegativeRadius.</summary>
    public MakeCircle(in Circle3D circ, double dist)
    {
        MakeCirc c = new MakeCirc(circ, dist);
        _theError = c.Status();
        if (_theError == Done)
        {
            _theCircle = new CircleCurve3D(c.Value());
        }
    }

    /// <summary>A circle through three points; fails with the status of gce_MakeCirc.</summary>
    public MakeCircle(in Point3D p1, in Point3D p2, in Point3D p3)
    {
        MakeCirc c = new MakeCirc(p1, p2, p3);
        _theError = c.Status();
        if (_theError == Done)
        {
            _theCircle = new CircleCurve3D(c.Value());
        }
    }

    /// <summary>A circle of center Point, normal Norm and radius Radius.</summary>
    public MakeCircle(in Point3D point, in Direction3D norm, double radius)
    {
        MakeCirc c = new MakeCirc(point, norm, radius);
        _theError = c.Status();
        if (_theError == Done)
        {
            _theCircle = new CircleCurve3D(c.Value());
        }
    }

    /// <summary>A circle of center Point, with its axis through PtAxis, and radius Radius.</summary>
    public MakeCircle(in Point3D point, in Point3D ptAxis, double radius)
    {
        MakeCirc c = new MakeCirc(point, ptAxis, radius);
        _theError = c.Status();
        if (_theError == Done)
        {
            _theCircle = new CircleCurve3D(c.Value());
        }
    }

    /// <summary>A circle of axis Axis and radius Radius.</summary>
    public MakeCircle(in Axis3D axis, double radius)
    {
        MakeCirc c = new MakeCirc(axis, radius);
        _theError = c.Status();
        if (_theError == Done)
        {
            _theCircle = new CircleCurve3D(c.Value());
        }
    }

    /// <summary>The constructed circle (frozen).</summary>
    /// <exception cref="NotDoneException">No circle was constructed.</exception>
    public CircleCurve3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeCircle::Value() - no result");
        _theCircle!.Freeze();
        return _theCircle;
    }

    /// <summary>The constructed circle (OCCT's implicit conversion).</summary>
    public static explicit operator CircleCurve3D(MakeCircle maker) => maker.Value();
}
