// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeEllipse.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeEllipse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a Geom_Ellipse: from a gp_Elips, an axis system and two radii, or two apexes and the center.
/// </summary>
internal sealed class MakeEllipse : Root
{
    private EllipseCurve3D? _theEllipse;

    /// <summary>An ellipse from the gp_Elips E.</summary>
    public MakeEllipse(in Ellipse3D e)
    {
        _theError = Done;
        _theEllipse = new EllipseCurve3D(e);
    }

    /// <summary>
    /// An ellipse in the axis system A2; fails with gce_NegativeRadius if MinorRadius is negative and with
    /// gce_InvertAxis if MajorRadius is less than MinorRadius.
    /// </summary>
    public MakeEllipse(in Frame3D a2, double majorRadius, double minorRadius)
    {
        if (minorRadius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else if (majorRadius < minorRadius)
        {
            _theError = InvertAxis;
        }
        else
        {
            _theError = Done;
            _theEllipse = new EllipseCurve3D(new Ellipse3D(a2, majorRadius, minorRadius));
        }
    }

    /// <summary>
    /// An ellipse of center Center, with S1 at the end of the major axis and S2 defining the minor radius; fails with
    /// the status of gce_MakeElips.
    /// </summary>
    public MakeEllipse(in Point3D s1, in Point3D s2, in Point3D center)
    {
        MakeElips e = new MakeElips(s1, s2, center);
        _theError = e.Status();
        if (_theError == Done)
        {
            _theEllipse = new EllipseCurve3D(e.Value());
        }
    }

    /// <summary>The constructed ellipse (frozen).</summary>
    /// <exception cref="NotDoneException">No ellipse was constructed.</exception>
    public EllipseCurve3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeEllipse::Value() - no result");
        _theEllipse!.Freeze();
        return _theEllipse;
    }

    /// <summary>The constructed ellipse (OCCT's implicit conversion).</summary>
    public static explicit operator EllipseCurve3D(MakeEllipse maker) => maker.Value();
}
