// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeHypr.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeHypr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>
/// Constructions of a gp_Hypr: from a center and two points (one on the major axis), or from an axis system and
/// radii. The X axis of the hyperbola is its major axis, the Y axis its minor axis; the major radius may be lower
/// than the minor radius.
/// </summary>
public sealed class MakeHypr : Root
{
    private Hyperbola3D _theHypr = new();

    /// <summary>
    /// A hyperbola from its center and two points: Center-S1 is the major axis and its length the major radius, and
    /// the distance from S2 to that axis is the minor radius. Fails with gce_ConfusedPoints if two of the points
    /// coincide and gce_ColinearPoints if they are collinear.
    /// </summary>
    public MakeHypr(in Point3D s1, in Point3D s2, in Point3D center)
    {
        Xyz vecS1 = s1.XYZ() - center.XYZ();
        Xyz vecS2 = s2.XYZ() - center.XYZ();
        double distS1 = vecS1.Modulus();
        double distS2 = vecS2.Modulus();
        if (distS1 <= Constants.Resolution() || distS2 <= Constants.Resolution()
            || s1.DistanceTo(s2) <= Constants.Resolution())
        {
            _theError = ConfusedPoints;
            return;
        }
        Direction3D xAxis = new(vecS1);
        Line3D line = new(center, xAxis);
        double minorDist = line.DistanceTo(s2);
        if (minorDist <= Constants.Resolution())
        {
            _theError = ColinearPoints;
            return;
        }
        Xyz cross = xAxis.XYZ().Crossed(vecS2);
        if (cross.Modulus() <= Constants.Resolution())
        {
            _theError = ColinearPoints;
            return;
        }
        Direction3D norm = new(cross);
        _theHypr = new Hyperbola3D(new Frame3D(center, norm, xAxis), distS1, minorDist);
        _theError = Done;
    }

    /// <summary>
    /// A hyperbola X*X / (MajorRadius*MajorRadius) - Y*Y / (MinorRadius*MinorRadius) = 1 in the system A2; fails with
    /// gce_NegativeRadius if a radius is negative.
    /// </summary>
    public MakeHypr(in Frame3D a2, double majorRadius, double minorRadius)
    {
        if (majorRadius < 0.0 || minorRadius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            _theHypr = new Hyperbola3D(a2, majorRadius, minorRadius);
            _theError = Done;
        }
    }

    /// <summary>The constructed hyperbola.</summary>
    /// <exception cref="NotDoneException">No hyperbola was constructed.</exception>
    public Hyperbola3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakeHypr::Value() - no result");
        return _theHypr;
    }

    /// <summary>Alias for Value().</summary>
    public Hyperbola3D Operator() => Value();

    /// <summary>The constructed hyperbola (OCCT's implicit conversion operator).</summary>
    public static explicit operator Hyperbola3D(MakeHypr maker) => maker.Operator();
}
