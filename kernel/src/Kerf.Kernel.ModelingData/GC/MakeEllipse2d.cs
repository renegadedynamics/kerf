// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeEllipse2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeEllipse2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeEllipse.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a Geom2d_Ellipse: from a gp_Elips2d, an axis (system) and two radii, or two apexes and the center.
/// </summary>
internal sealed class MakeEllipse2d : Root
{
    private EllipseCurve2D? _theEllipse;

    /// <summary>An ellipse from the gp_Elips2d E.</summary>
    public MakeEllipse2d(in Ellipse2D e)
    {
        _theError = Done;
        _theEllipse = new EllipseCurve2D(e);
    }

    /// <summary>An ellipse in the axis system Axis; fails with the status of gce_MakeElips2d.</summary>
    public MakeEllipse2d(in Frame2D axis, double majorRadius, double minorRadius)
    {
        MakeElips2d e = new MakeElips2d(axis, majorRadius, minorRadius);
        _theError = e.Status();
        if (_theError == Done)
        {
            _theEllipse = new EllipseCurve2D(e.Value());
        }
    }

    /// <summary>An ellipse on the major axis MajorAxis; fails with the status of gce_MakeElips2d.</summary>
    public MakeEllipse2d(in Axis2D majorAxis, double majorRadius, double minorRadius, bool sense = true)
    {
        MakeElips2d e = new MakeElips2d(majorAxis, majorRadius, minorRadius, sense);
        _theError = e.Status();
        if (_theError == Done)
        {
            _theEllipse = new EllipseCurve2D(e.Value());
        }
    }

    /// <summary>An ellipse from two apexes and its center; fails with the status of gce_MakeElips2d.</summary>
    public MakeEllipse2d(in Point2D s1, in Point2D s2, in Point2D center)
    {
        MakeElips2d e = new MakeElips2d(s1, s2, center);
        _theError = e.Status();
        if (_theError == Done)
        {
            _theEllipse = new EllipseCurve2D(e.Value());
        }
    }

    /// <summary>The constructed ellipse (frozen).</summary>
    /// <exception cref="NotDoneException">No ellipse was constructed.</exception>
    public EllipseCurve2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeEllipse2d::Value() - no result");
        _theEllipse!.Freeze();
        return _theEllipse;
    }

    /// <summary>The constructed ellipse (OCCT's implicit conversion).</summary>
    public static explicit operator EllipseCurve2D(MakeEllipse2d maker) => maker.Value();
}
