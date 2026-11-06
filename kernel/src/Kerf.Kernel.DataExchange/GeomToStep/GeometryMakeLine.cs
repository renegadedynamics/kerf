// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeLine.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeLine.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeLine_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of Line from Geom and Geom2d (and Lin and Lin2d from gp) to a STEP line.</summary>
/// <remarks>
/// The line's vector is its unit direction made a vector: for a 3D line its magnitude is 1 divided by the length
/// factor, as OCCT writes it.
/// </remarks>
internal sealed class GeometryMakeLine : GeometryRoot
{
    private StepLine _line;

    //=============================================================================
    // Creation d' une line de prostep a partir d' une Lin de gp
    //=============================================================================

    /// <summary>A 3D line, its location divided by the length factor.</summary>
    public GeometryMakeLine(in Line3D l, in Factors localFactors)
    {
        Make(new MakeCartesianPoint(l.Location, localFactors.LengthFactor()),
             new MakeVector(new Vector3D(l.Direction), localFactors));
    }

    /// <summary>A 3D line (default factors).</summary>
    public GeometryMakeLine(in Line3D l) : this(l, new Factors())
    {
    }

    //=============================================================================
    // Creation d' une line de prostep a partir d' une Lin2d de gp
    //=============================================================================

    /// <summary>A 2D line.</summary>
    public GeometryMakeLine(in Line2D l, in Factors localFactors)
    {
        Make(new MakeCartesianPoint(l.Location, localFactors.LengthFactor()),
             new MakeVector(new Vector2D(l.Direction), localFactors));
    }

    /// <summary>A 2D line.</summary>
    public GeometryMakeLine(in Line2D l) : this(l, new Factors())
    {
    }

    //=============================================================================
    // Creation d' une line de prostep a partir d' une Line de Geom
    //=============================================================================

    /// <summary>A 3D line, its location divided by the length factor.</summary>
    public GeometryMakeLine(LineCurve3D gline, in Factors localFactors)
    {
        Line3D l;
        l = gline.Line;
        Make(new MakeCartesianPoint(l.Location, localFactors.LengthFactor()),
             new MakeVector(new Vector3D(l.Direction), localFactors));
    }

    /// <summary>A 3D line (default factors).</summary>
    public GeometryMakeLine(LineCurve3D gline) : this(gline, new Factors())
    {
    }

    //=============================================================================
    // Creation d' une line de prostep a partir d' une Line de Geom2d
    //=============================================================================

    /// <summary>A 2D line.</summary>
    public GeometryMakeLine(LineCurve2D gline, in Factors localFactors)
    {
        Line2D l;
        l = gline.Line;
        Make(new MakeCartesianPoint(l.Location, localFactors.LengthFactor()),
             new MakeVector(new Vector2D(l.Direction), localFactors));
    }

    /// <summary>A 2D line.</summary>
    public GeometryMakeLine(LineCurve2D gline) : this(gline, new Factors())
    {
    }

    // GeomToStep_MakeLine_gen.pxx, the body OCCT includes in each constructor (its point and vector made first)
    private void Make(MakeCartesianPoint mkPoint, MakeVector mkVector)
    {
        StepLine lin = new StepLine();
        StepCartesianPoint pnt;
        StepVector dir;

        pnt = mkPoint.Value();
        dir = mkVector.Value();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        lin.Init(name, pnt, dir);
        _line = lin;
        _done = true;
    }

    /// <summary>The line made.</summary>
    public StepLine Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeLine::Value() - no result");
        return _line;
    }
}
