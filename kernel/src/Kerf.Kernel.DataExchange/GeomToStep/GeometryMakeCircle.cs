// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCircle.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCircle.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCircle_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.FromGeometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of Circle from Geom and Geom2d (and Circ from gp) to a STEP circle.</summary>
internal sealed class GeometryMakeCircle : GeometryRoot
{
    private StepCircle _circle;

    //=============================================================================
    // Creation d' un cercle de prostep a partir d' un cercle 3d de gp
    //=============================================================================

    /// <summary>A 3D circle, its position and radius divided by the length factor.</summary>
    public GeometryMakeCircle(in Circle3D c, in Factors localFactors)
    {
        Make(c, localFactors);
    }

    /// <summary>A 3D circle (default factors).</summary>
    public GeometryMakeCircle(in Circle3D c) : this(c, new Factors())
    {
    }

    //=============================================================================
    // Creation d' un cercle de prostep a partir d' un cercle de
    // Geom
    //=============================================================================

    /// <summary>A 3D circle, its position and radius divided by the length factor.</summary>
    public GeometryMakeCircle(CircleCurve3D cer, in Factors localFactors)
    {
        Circle3D c;
        c = cer.Circle;
        Make(c, localFactors);
    }

    /// <summary>A 3D circle (default factors).</summary>
    public GeometryMakeCircle(CircleCurve3D cer) : this(cer, new Factors())
    {
    }

    //=============================================================================
    // Creation d' un cercle 2d de prostep a partir d' un cercle de
    // Geom2d
    //=============================================================================

    /// <summary>A 2D circle.</summary>
    public GeometryMakeCircle(CircleCurve2D cer, in Factors localFactors)
    {
        Circle2D c2d;
        c2d = cer.Circle;

        StepCircle cStep = new StepCircle();
        StepAxis2Placement ax2 = new StepAxis2Placement();
        StepAxis2Placement2d ax2Step;
        double rayon;

        MakeAxis2Placement2d mkAxis2 = new MakeAxis2Placement2d(c2d.Position, localFactors);
        ax2Step = mkAxis2.Value();
        rayon = c2d.Radius;
        ax2.SetValue(ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        cStep.Init(name, ax2, rayon);
        _circle = cStep;
        _done = true;
    }

    /// <summary>A 2D circle.</summary>
    public GeometryMakeCircle(CircleCurve2D cer) : this(cer, new Factors())
    {
    }

    // GeomToStep_MakeCircle_gen.pxx, the body OCCT includes in the 3D constructors
    private void Make(in Circle3D c, in Factors localFactors)
    {
        StepCircle cStep = new StepCircle();
        StepAxis2Placement ax2 = new StepAxis2Placement();
        StepAxis2Placement3d ax2Step;
        double rayon;

        Axis2Placement3dMaker mkAxis2 = new Axis2Placement3dMaker(c.Position, localFactors);
        ax2Step = mkAxis2.Value();
        rayon = c.Radius;
        ax2.SetValue(ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        cStep.Init(name, ax2, rayon / localFactors.LengthFactor());
        _circle = cStep;
        _done = true;
    }

    /// <summary>The circle made.</summary>
    public StepCircle Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeCircle::Value() - no result");
        return _circle;
    }
}
