// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeSegment2d.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeSegment2d.cxx
// Port of: src/ModelingData/TKGeomBase/GCE2d/GCE2d_MakeSegment.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a line segment in 2D as a Geom2d_TrimmedCurve: between two points, from a point and a direction
/// to the projection of a point, or on a line between two points or parameters.
/// </summary>
public sealed class MakeSegment2d : Root
{
    private TrimmedCurve2D? _theSegment;

    /// <summary>
    /// The segment of the line through P1 with direction V, from P1 to the projection of P2; fails with
    /// gce_ConfusedPoints if that projection is P1.
    /// </summary>
    public MakeSegment2d(in Point2D p1, in Direction2D v, in Point2D p2)
    {
        Line2D line = new Line2D(p1, v);
        double ulast = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, p2);
        if (Math.Abs(ulast) > Constants.Resolution())
        {
            LineCurve2D l = new LineCurve2D(line);
            _theSegment = new TrimmedCurve2D(l, 0.0, ulast, true);
            _theError = Done;
        }
        else
        {
            _theError = ConfusedPoints;
        }
    }

    /// <summary>The segment from P1 to P2; fails with gce_ConfusedPoints if they coincide.</summary>
    public MakeSegment2d(in Point2D p1, in Point2D p2)
    {
        double dist = p1.Distance(p2);
        if (dist > Constants.Resolution())
        {
            // GC_MakeLine2d(P1, P2) succeeds here: it only fails below gp::Resolution().
            LineCurve2D l = new MakeLine2d(p1, p2).Value();
            _theSegment = new TrimmedCurve2D(l, 0.0, dist, true);
            _theError = Done;
        }
        else
        {
            _theError = ConfusedPoints;
        }
    }

    /// <summary>The segment of Line from the parameter of Point to the parameter U.</summary>
    public MakeSegment2d(in Line2D line, in Point2D point, double u)
    {
        double ufirst = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, point);
        LineCurve2D l = new LineCurve2D(line);
        _theSegment = new TrimmedCurve2D(l, ufirst, u, true);
        _theError = Done;
    }

    /// <summary>The segment of Line between the parameters of P1 and P2.</summary>
    public MakeSegment2d(in Line2D line, in Point2D p1, in Point2D p2)
    {
        double ufirst = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, p1);
        double ulast = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, p2);
        LineCurve2D l = new LineCurve2D(line);
        _theSegment = new TrimmedCurve2D(l, ufirst, ulast, true);
        _theError = Done;
    }

    /// <summary>The segment of Line between the parameters U1 and U2.</summary>
    public MakeSegment2d(in Line2D line, double u1, double u2)
    {
        LineCurve2D l = new LineCurve2D(line);
        _theSegment = new TrimmedCurve2D(l, u1, u2, true);
        _theError = Done;
    }

    /// <summary>The constructed segment (frozen).</summary>
    /// <exception cref="NotDoneException">No segment was constructed.</exception>
    public TrimmedCurve2D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeSegment2d::Value() - no result");
        _theSegment!.Freeze();
        return _theSegment;
    }

    /// <summary>The constructed segment (OCCT's implicit conversion).</summary>
    public static explicit operator TrimmedCurve2D(MakeSegment2d maker) => maker.Value();
}
