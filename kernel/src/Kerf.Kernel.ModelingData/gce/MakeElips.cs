// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeElips.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeElips.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>Constructions of a gp_Elips: from an axis system and radii, or from a center and two points.</summary>
public sealed class MakeElips : Root
{
    private Ellipse3D _theElips = new();

    /// <summary>
    /// An ellipse with its major radius on the X axis of A2 and its minor radius on its Y axis. The radii may be
    /// equal. Fails with gce_InvertRadius if MajorRadius &lt; MinorRadius and gce_NegativeRadius if MinorRadius &lt; 0.
    /// </summary>
    public MakeElips(in Frame3D a2, double majorRadius, double minorRadius)
    {
        if (majorRadius < minorRadius)
        {
            _theError = InvertRadius;
        }
        else if (minorRadius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            _theElips = new Ellipse3D(a2, majorRadius, minorRadius);
            _theError = Done;
        }
    }

    /// <summary>
    /// An ellipse from its center and two points: S1 gives the major axis and radius, the distance from S2 to the
    /// major axis the minor radius. Fails with gce_NullAxis if S1 and Center coincide and gce_InvertAxis if the minor
    /// radius is null or greater than the major radius, or the points are collinear.
    /// </summary>
    public MakeElips(in Point3D s1, in Point3D s2, in Point3D center)
    {
        double d1 = s1.DistanceTo(center);
        if (d1 < Constants.Resolution())
        {
            _theError = NullAxis;
        }
        else
        {
            Direction3D xAxis = new(s1.XYZ() - center.XYZ());
            double d2 = new Line3D(center, xAxis).DistanceTo(s2);
            if (d1 < d2 || d2 < Constants.Resolution())
            {
                _theError = InvertAxis;
            }
            else
            {
                Xyz cross = xAxis.XYZ().Crossed(s2.XYZ() - center.XYZ());
                if (cross.Modulus() <= Constants.Resolution())
                {
                    _theError = InvertAxis;
                }
                else
                {
                    Direction3D norm = new(cross);
                    _theElips = new Ellipse3D(new Frame3D(center, norm, xAxis), d1, d2);
                    _theError = Done;
                }
            }
        }
    }

    /// <summary>The constructed ellipse.</summary>
    /// <exception cref="NotDoneException">No ellipse was constructed.</exception>
    public Ellipse3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakeElips::Value() - no result");
        return _theElips;
    }

    /// <summary>Alias for Value().</summary>
    public Ellipse3D Operator() => Value();

    /// <summary>The constructed ellipse (OCCT's implicit conversion operator).</summary>
    public static explicit operator Ellipse3D(MakeElips maker) => maker.Operator();
}
