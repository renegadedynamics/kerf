// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeCircle2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeCircle2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeCircle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a Geom2d_Circle: from a gp_Circ2d, an axis and a radius, concentric to a circle through a point
/// or at a distance, through three points, or from a center and a radius or a point.
/// </summary>
internal sealed class MakeCircle2d : Root
{
    private CircleCurve2D? _theCircle;

    /// <summary>A circle from the gp_Circ2d C.</summary>
    public MakeCircle2d(in Circle2D c)
    {
        _theError = Done;
        _theCircle = new CircleCurve2D(c);
    }

    /// <summary>A circle of radius Radius on the axis A; fails with gce_NegativeRadius.</summary>
    public MakeCircle2d(in Axis2D a, double radius, bool sense = true)
    {
        if (radius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            _theError = Done;
            _theCircle = new CircleCurve2D(a, radius, sense);
        }
    }

    /// <summary>A circle of radius Radius in the axis system A; fails with gce_NegativeRadius.</summary>
    public MakeCircle2d(in Frame2D a, double radius)
    {
        if (radius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            _theError = Done;
            _theCircle = new CircleCurve2D(a, radius);
        }
    }

    /// <summary>A circle concentric to Circ through Point.</summary>
    public MakeCircle2d(in Circle2D circ, in Point2D point)
    {
        MakeCirc2d c = new MakeCirc2d(circ, point);
        _theError = c.Status();
        if (_theError == Done)
        {
            _theCircle = new CircleCurve2D(c.Value());
        }
    }

    /// <summary>A circle concentric to Circ at the signed distance Dist; fails with gce_NegativeRadius.</summary>
    public MakeCircle2d(in Circle2D circ, double dist)
    {
        MakeCirc2d c = new MakeCirc2d(circ, dist);
        _theError = c.Status();
        if (_theError == Done)
        {
            _theCircle = new CircleCurve2D(c.Value());
        }
    }

    /// <summary>A circle through three points; fails with the status of gce_MakeCirc2d.</summary>
    public MakeCircle2d(in Point2D p1, in Point2D p2, in Point2D p3)
    {
        MakeCirc2d c = new MakeCirc2d(p1, p2, p3);
        _theError = c.Status();
        if (_theError == Done)
        {
            _theCircle = new CircleCurve2D(c.Value());
        }
    }

    /// <summary>A circle of center Point and radius Radius; fails with gce_NegativeRadius.</summary>
    public MakeCircle2d(in Point2D point, double radius, bool sense = true)
    {
        MakeCirc2d c = new MakeCirc2d(point, radius, sense);
        _theError = c.Status();
        if (_theError == Done)
        {
            _theCircle = new CircleCurve2D(c.Value());
        }
    }

    /// <summary>A circle of center Center through Point.</summary>
    public MakeCircle2d(in Point2D center, in Point2D point, bool sense = true)
    {
        MakeCirc2d c = new MakeCirc2d(center, point, sense);
        _theError = c.Status();
        if (_theError == Done)
        {
            _theCircle = new CircleCurve2D(c.Value());
        }
    }

    /// <summary>The constructed circle (frozen).</summary>
    /// <exception cref="NotDoneException">No circle was constructed.</exception>
    public CircleCurve2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeCircle2d::Value() - no result");
        _theCircle!.Freeze();
        return _theCircle;
    }

    /// <summary>The constructed circle (OCCT's implicit conversion).</summary>
    public static explicit operator CircleCurve2D(MakeCircle2d maker) => maker.Value();
}
