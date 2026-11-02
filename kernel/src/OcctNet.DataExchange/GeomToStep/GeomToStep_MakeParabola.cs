// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeParabola.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeParabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The mapping of Parabola from Geom and Geom2d to a STEP parabola.</summary>
public sealed class GeomToStep_MakeParabola : GeomToStep_Root
{
    private StepGeom_Parabola theParabola;

    //=============================================================================
    // Creation d'une Parabola de prostep a partir d'une Parabola de
    // Geom2d
    //=============================================================================

    /// <summary>A 2D parabola.</summary>
    public GeomToStep_MakeParabola(Geom2d_Parabola C, in StepData_Factors theLocalFactors)
    {
        gp_Parab2d gpPar;
        gpPar = C.Parab2d();

        StepGeom_Parabola PStep = new StepGeom_Parabola();
        StepGeom_Axis2Placement Ax2 = new StepGeom_Axis2Placement();
        StepGeom_Axis2Placement2d Ax2Step;
        double focal;

        GeomToStep_MakeAxis2Placement2d MkAxis2 = new GeomToStep_MakeAxis2Placement2d(gpPar.Axis(), theLocalFactors);
        Ax2Step = MkAxis2.Value();
        focal = gpPar.Focal();
        Ax2.SetValue(Ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        PStep.Init(name, Ax2, focal);
        theParabola = PStep;
        done = true;
    }

    /// <summary>A 2D parabola.</summary>
    public GeomToStep_MakeParabola(Geom2d_Parabola C) : this(C, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d'une Parabola de prostep a partir d'une Parabola de
    // Geom
    //=============================================================================

    /// <summary>A 3D parabola, its position and focal length divided by the length factor.</summary>
    public GeomToStep_MakeParabola(Geom_Parabola C, in StepData_Factors theLocalFactors)
    {
        gp_Parab gpPar;
        gpPar = C.Parab();

        StepGeom_Parabola PStep = new StepGeom_Parabola();
        StepGeom_Axis2Placement Ax2 = new StepGeom_Axis2Placement();
        StepGeom_Axis2Placement3d Ax2Step;
        double focal;

        GeomToStep_MakeAxis2Placement3d MkAxis2 = new GeomToStep_MakeAxis2Placement3d(gpPar.Position(), theLocalFactors);
        Ax2Step = MkAxis2.Value();
        focal = gpPar.Focal();
        Ax2.SetValue(Ax2Step);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        PStep.Init(name, Ax2, focal / theLocalFactors.LengthFactor());
        theParabola = PStep;
        done = true;
    }

    /// <summary>A 3D parabola (default factors).</summary>
    public GeomToStep_MakeParabola(Geom_Parabola C) : this(C, new StepData_Factors())
    {
    }

    //=============================================================================
    // return the result
    //=============================================================================

    /// <summary>The parabola made.</summary>
    public StepGeom_Parabola Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeParabola::Value() - no result");
        return theParabola;
    }
}
