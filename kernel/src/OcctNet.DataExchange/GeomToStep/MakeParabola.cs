// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeParabola.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeParabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.FromGeometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of Parabola from Geom and Geom2d to a STEP parabola.</summary>
public sealed class MakeParabola : GeometryRoot
{
    private StepParabola _parabola;

    //=============================================================================
    // Creation d'une Parabola de prostep a partir d'une Parabola de
    // Geom2d
    //=============================================================================

    /// <summary>A 2D parabola.</summary>
    public MakeParabola(ParabolaCurve2D c, in Factors localFactors)
    {
        Parabola2D gpPar;
        gpPar = c.Parab2d();

        StepParabola pStep = new StepParabola();
        StepAxis2Placement ax2 = new StepAxis2Placement();
        StepAxis2Placement2d ax2Step;
        double focal;

        MakeAxis2Placement2d mkAxis2 = new MakeAxis2Placement2d(gpPar.Axis(), localFactors);
        ax2Step = mkAxis2.Value();
        focal = gpPar.Focal();
        ax2.SetValue(ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        pStep.Init(name, ax2, focal);
        _parabola = pStep;
        _done = true;
    }

    /// <summary>A 2D parabola.</summary>
    public MakeParabola(ParabolaCurve2D c) : this(c, new Factors())
    {
    }

    //=============================================================================
    // Creation d'une Parabola de prostep a partir d'une Parabola de
    // Geom
    //=============================================================================

    /// <summary>A 3D parabola, its position and focal length divided by the length factor.</summary>
    public MakeParabola(ParabolaCurve3D c, in Factors localFactors)
    {
        Parabola3D gpPar;
        gpPar = c.Parab();

        StepParabola pStep = new StepParabola();
        StepAxis2Placement ax2 = new StepAxis2Placement();
        StepAxis2Placement3d ax2Step;
        double focal;

        Axis2Placement3dMaker mkAxis2 = new Axis2Placement3dMaker(gpPar.Position(), localFactors);
        ax2Step = mkAxis2.Value();
        focal = gpPar.Focal();
        ax2.SetValue(ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        pStep.Init(name, ax2, focal / localFactors.LengthFactor());
        _parabola = pStep;
        _done = true;
    }

    /// <summary>A 3D parabola (default factors).</summary>
    public MakeParabola(ParabolaCurve3D c) : this(c, new Factors())
    {
    }

    //=============================================================================
    // return the result
    //=============================================================================

    /// <summary>The parabola made.</summary>
    public StepParabola Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeParabola::Value() - no result");
        return _parabola;
    }
}
