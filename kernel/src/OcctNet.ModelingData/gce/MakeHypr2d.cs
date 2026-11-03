// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeHypr2d.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeHypr2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>
/// Constructions of a gp_Hypr2d: from a center and two points (one on the major axis), from a major axis and radii,
/// or from an axis system and radii. The major radius may be lower than the minor radius.
/// </summary>
public sealed class MakeHypr2d : Root
{
    private Hyperbola2D _theHypr2d = new();

    /// <summary>
    /// A hyperbola from its center and two points: Center-S1 is the major axis and its length the major radius, and
    /// the distance from S2 to that axis is the minor radius. Fails with gce_ConfusedPoints if two of the points
    /// coincide and gce_ColinearPoints if they are collinear.
    /// </summary>
    public MakeHypr2d(in Point2D s1, in Point2D s2, in Point2D center)
    {
        Xy vecS1 = s1.XY() - center.XY();
        Xy vecS2 = s2.XY() - center.XY();
        double distS1 = vecS1.Modulus();
        double distS2 = vecS2.Modulus();
        if (distS1 <= Constants.Resolution() || distS2 <= Constants.Resolution()
            || s1.Distance(s2) <= Constants.Resolution())
        {
            _theError = ConfusedPoints;
            return;
        }
        Direction2D xAxis = new(vecS1);
        Direction2D yAxis = new(vecS2);
        Frame2D axis = new(center, xAxis, yAxis);
        Line2D line = new(center, xAxis);
        double minorDist = line.Distance(s2);
        if (minorDist <= Constants.Resolution())
        {
            _theError = ColinearPoints;
            return;
        }
        _theHypr2d = new Hyperbola2D(axis, distS1, minorDist);
        _theError = Done;
    }

    /// <summary>
    /// A hyperbola from its major axis (whose location is the center), its radii and a sense (false uses the opposite
    /// direction of MajorAxis); fails with gce_NegativeRadius if a radius is negative.
    /// </summary>
    public MakeHypr2d(in Axis2D majorAxis, double majorRadius, double minorRadius, bool sense)
    {
        if (majorRadius < 0.0 || minorRadius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            _theHypr2d = new Hyperbola2D(majorAxis, majorRadius, minorRadius, sense);
            _theError = Done;
        }
    }

    /// <summary>
    /// A hyperbola from its axis system (whose location is the center, whose X axis is the major axis) and its radii;
    /// fails with gce_NegativeRadius if a radius is negative.
    /// </summary>
    public MakeHypr2d(in Frame2D a, double majorRadius, double minorRadius)
    {
        if (majorRadius < 0.0 || minorRadius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            _theHypr2d = new Hyperbola2D(a, majorRadius, minorRadius);
            _theError = Done;
        }
    }

    /// <summary>The constructed hyperbola.</summary>
    /// <exception cref="NotDoneException">No hyperbola was constructed.</exception>
    public Hyperbola2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakeHypr2d::Value() - no result");
        return _theHypr2d;
    }

    /// <summary>Alias for Value().</summary>
    public Hyperbola2D Operator() => Value();

    /// <summary>The constructed hyperbola (OCCT's implicit conversion operator).</summary>
    public static explicit operator Hyperbola2D(MakeHypr2d maker) => maker.Operator();
}
