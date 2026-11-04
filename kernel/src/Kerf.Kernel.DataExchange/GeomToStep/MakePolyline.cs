// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakePolyline.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakePolyline.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakePolyline_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of an array of points from gp to a STEP polyline.</summary>
internal sealed class MakePolyline : GeometryRoot
{
    private StepPolyline _polyline;

    //=============================================================================
    // Creation d' une polyline Step a partir d' une Array1OfPnt
    //=============================================================================

    /// <summary>3D points, divided by the length factor (the array's lower bound must be 1, as OCCT assumes).</summary>
    public MakePolyline(NCollection_Array1<Point3D> p, in Factors localFactors)
    {
        int n, i;
        NCollection_HArray1<StepCartesianPoint> points;
        StepCartesianPoint p2;
        Point3D p1;

        n = p.Length();
        points = new NCollection_HArray1<StepCartesianPoint>(1, n);
        for (i = p.Lower(); i <= p.Upper(); i++)
        {
            p1 = p.Value(i);
            MakeCartesianPoint mkPoint = new MakeCartesianPoint(p1, localFactors.LengthFactor());
            p2 = mkPoint.Value();
            points.SetValue(i, p2);
        }
        Make(points);
    }

    /// <summary>3D points (default factors).</summary>
    public MakePolyline(NCollection_Array1<Point3D> p) : this(p, new Factors())
    {
    }

    //=============================================================================
    // Creation d' une polyline Step a partir d' une Array1OfPnt2d
    //=============================================================================

    /// <summary>2D points (the array's lower bound must be 1).</summary>
    public MakePolyline(NCollection_Array1<Point2D> p, in Factors localFactors)
    {
        int n, i;
        NCollection_HArray1<StepCartesianPoint> points;
        StepCartesianPoint p2;
        Point2D p1;

        n = p.Length();
        points = new NCollection_HArray1<StepCartesianPoint>(1, n);
        for (i = p.Lower(); i <= p.Upper(); i++)
        {
            p1 = p.Value(i);
            MakeCartesianPoint mkPoint = new MakeCartesianPoint(p1, localFactors.LengthFactor());
            p2 = mkPoint.Value();
            points.SetValue(i, p2);
        }
        Make(points);
    }

    /// <summary>2D points.</summary>
    public MakePolyline(NCollection_Array1<Point2D> p) : this(p, new Factors())
    {
    }

    // The end of GeomToStep_MakePolyline_gen.pxx, the body OCCT includes in both constructors
    private void Make(NCollection_HArray1<StepCartesianPoint> aPoints)
    {
        _polyline = new StepPolyline();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        _polyline.Init(name, aPoints);
        _done = true;
    }

    /// <summary>The polyline made.</summary>
    public StepPolyline Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakePolyline::Value() - no result");
        return _polyline;
    }
}
