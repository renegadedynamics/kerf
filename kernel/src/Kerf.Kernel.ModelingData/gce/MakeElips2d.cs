// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeElips2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeElips2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>
/// Constructions of a gp_Elips2d: from a major axis (or an axis system) and radii, or from a center and two points.
/// </summary>
internal sealed class MakeElips2d : Root
{
    private Ellipse2D _theElips2d = new();

    /// <summary>
    /// An ellipse from its center and two points: S1 gives the major axis and radius, the distance from S2 to the
    /// major axis the minor radius. Fails with gce_InvertAxis if the minor radius is greater than the major one and
    /// gce_NullAxis if it is null.
    /// </summary>
    public MakeElips2d(in Point2D s1, in Point2D s2, in Point2D center)
    {
        double d1 = s1.DistanceTo(center);
        Direction2D xAxis = new(s1.XY() - center.XY());
        Direction2D yAxis = new(s2.XY() - center.XY());
        double d2 = new Line2D(center, xAxis).DistanceTo(s2);
        if (d1 < d2)
        {
            _theError = InvertAxis;
        }
        else if (d2 < Constants.Resolution())
        {
            _theError = NullAxis;
        }
        else
        {
            _theElips2d = new Ellipse2D(new Frame2D(center, xAxis, yAxis), d1, d2);
            _theError = Done;
        }
    }

    /// <summary>
    /// An ellipse from its major axis (whose location is the center), its radii and a sense of parametrization.
    /// Fails with gce_NegativeRadius if MajorRadius &lt; 0 and gce_InvertRadius if MajorRadius &lt; MinorRadius.
    /// </summary>
    public MakeElips2d(in Axis2D majorAxis, double majorRadius, double minorRadius, bool sense = true)
    {
        if (majorRadius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else if (majorRadius < minorRadius)
        {
            _theError = InvertRadius;
        }
        else
        {
            _theElips2d = new Ellipse2D(majorAxis, majorRadius, minorRadius, sense);
            _theError = Done;
        }
    }

    /// <summary>
    /// An ellipse from its axis system (whose location is the center) and its radii. Fails with gce_NegativeRadius if
    /// MajorRadius &lt; 0 and gce_InvertRadius if MajorRadius &lt; MinorRadius.
    /// </summary>
    public MakeElips2d(in Frame2D a, double majorRadius, double minorRadius)
    {
        if (majorRadius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else if (majorRadius < minorRadius)
        {
            _theError = InvertRadius;
        }
        else
        {
            _theElips2d = new Ellipse2D(a, majorRadius, minorRadius);
            _theError = Done;
        }
    }

    /// <summary>The constructed ellipse.</summary>
    /// <exception cref="NotDoneException">No ellipse was constructed.</exception>
    public Ellipse2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakeElips2d::Value() - no result");
        return _theElips2d;
    }

    /// <summary>Alias for Value().</summary>
    public Ellipse2D Operator() => Value();

    /// <summary>The constructed ellipse (OCCT's implicit conversion operator).</summary>
    public static explicit operator Ellipse2D(MakeElips2d maker) => maker.Operator();
}
