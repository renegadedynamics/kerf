// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeEllipse.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeEllipse.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeEllipse_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.FromGeometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of Ellipse from Geom and Geom2d (and Elips from gp) to a STEP ellipse.</summary>
internal sealed class GeometryMakeEllipse : GeometryRoot
{
    private StepEllipse _ellipse;

    //=============================================================================
    // Creation d'une ellipse de prostep a partir d'une ellipse 3d de gp
    //=============================================================================

    /// <summary>A 3D ellipse, its position and radii divided by the length factor.</summary>
    public GeometryMakeEllipse(in Ellipse3D e, in Factors localFactors)
    {
        Make(e, localFactors);
    }

    /// <summary>A 3D ellipse (default factors).</summary>
    public GeometryMakeEllipse(in Ellipse3D e) : this(e, new Factors())
    {
    }

    //=============================================================================
    // Creation d'une ellipse de prostep a partir d'une ellipse de
    // Geom
    //=============================================================================

    /// <summary>A 3D ellipse, its position and radii divided by the length factor.</summary>
    public GeometryMakeEllipse(EllipseCurve3D cer, in Factors localFactors)
    {
        Ellipse3D e;
        e = cer.Ellipse;
        Make(e, localFactors);
    }

    /// <summary>A 3D ellipse (default factors).</summary>
    public GeometryMakeEllipse(EllipseCurve3D cer) : this(cer, new Factors())
    {
    }

    //=============================================================================
    // Creation d'une ellipse 2d de prostep a partir d'une ellipse de
    // Geom2d
    //=============================================================================

    /// <summary>A 2D ellipse.</summary>
    public GeometryMakeEllipse(EllipseCurve2D cer, in Factors localFactors)
    {
        Ellipse2D e2d;
        e2d = cer.Ellipse;

        StepEllipse eStep = new StepEllipse();
        StepAxis2Placement ax2 = new StepAxis2Placement();
        StepAxis2Placement2d ax2Step;
        double majorR, minorR;

        MakeAxis2Placement2d mkAxis2 = new MakeAxis2Placement2d(e2d.Axis, localFactors);
        ax2Step = mkAxis2.Value();
        majorR = e2d.MajorRadius;
        minorR = e2d.MinorRadius;
        ax2.SetValue(ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        eStep.Init(name, ax2, majorR, minorR);
        _ellipse = eStep;
        _done = true;
    }

    /// <summary>A 2D ellipse.</summary>
    public GeometryMakeEllipse(EllipseCurve2D cer) : this(cer, new Factors())
    {
    }

    // GeomToStep_MakeEllipse_gen.pxx, the body OCCT includes in the 3D constructors
    private void Make(in Ellipse3D e, in Factors localFactors)
    {
        StepEllipse eStep = new StepEllipse();
        StepAxis2Placement ax2 = new StepAxis2Placement();
        StepAxis2Placement3d ax2Step;
        double majorR, minorR;

        Axis2Placement3dMaker mkAxis2 = new Axis2Placement3dMaker(e.Position, localFactors);
        ax2Step = mkAxis2.Value();
        majorR = e.MajorRadius;
        minorR = e.MinorRadius;
        ax2.SetValue(ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        double fact = localFactors.LengthFactor();
        eStep.Init(name, ax2, majorR / fact, minorR / fact);
        _ellipse = eStep;
        _done = true;
    }

    /// <summary>The ellipse made.</summary>
    public StepEllipse Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeEllipse::Value() - no result");
        return _ellipse;
    }
}
