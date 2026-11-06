// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeSegment.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeSegment.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a line segment in 3D as a Geom_TrimmedCurve: between two points, or on a line between two
/// points or parameters.
/// </summary>
internal sealed class MakeSegment : Root
{
    private TrimmedCurve3D? _theSegment;

    /// <summary>The segment from P1 to P2, parameterised by length from 0 at P1.</summary>
    /// <exception cref="NotDoneException">
    /// P1 and P2 coincide. OCCT doesn't check: its release build goes on with a null line and crashes.
    /// </exception>
    public MakeSegment(in Point3D p1, in Point3D p2)
    {
        double dist = p1.DistanceTo(p2);
        // OCCT converts GC_MakeLine implicitly; Value() throws here when P1 and P2 coincide (see the exception).
        LineCurve3D l = new MakeLine(p1, p2).Value();
        _theSegment = new TrimmedCurve3D(l, 0.0, dist, true);
        _theError = Done;
    }

    /// <summary>The segment of Line from the parameter of Point to the parameter U.</summary>
    public MakeSegment(in Line3D line, in Point3D point, double u)
    {
        double ufirst = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, point);
        LineCurve3D l = new LineCurve3D(line);
        _theSegment = new TrimmedCurve3D(l, ufirst, u, true);
        _theError = Done;
    }

    /// <summary>The segment of Line between the parameters of P1 and P2.</summary>
    public MakeSegment(in Line3D line, in Point3D p1, in Point3D p2)
    {
        double ufirst = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, p1);
        double ulast = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, p2);
        LineCurve3D l = new LineCurve3D(line);
        _theSegment = new TrimmedCurve3D(l, ufirst, ulast, true);
        _theError = Done;
    }

    /// <summary>The segment of Line between the parameters U1 and U2.</summary>
    public MakeSegment(in Line3D line, double u1, double u2)
    {
        LineCurve3D l = new LineCurve3D(line);
        _theSegment = new TrimmedCurve3D(l, u1, u2, true);
        _theError = Done;
    }

    /// <summary>The constructed segment (frozen).</summary>
    /// <exception cref="NotDoneException">No segment was constructed.</exception>
    public TrimmedCurve3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeSegment::Value() - no result");
        _theSegment!.Freeze();
        return _theSegment;
    }

    /// <summary>The constructed segment (OCCT's implicit conversion).</summary>
    public static explicit operator TrimmedCurve3D(MakeSegment maker) => maker.Value();
}
