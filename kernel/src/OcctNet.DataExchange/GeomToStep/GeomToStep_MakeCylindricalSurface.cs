// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCylindricalSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCylindricalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The mapping of CylindricalSurface from Geom to a STEP cylindrical_surface.</summary>
public sealed class GeomToStep_MakeCylindricalSurface : GeomToStep_Root
{
    private StepGeom_CylindricalSurface theCylindricalSurface;

    //=============================================================================
    // Creation d' une conical_surface de prostep a partir d' une
    // CylindricalSurface de Geom
    //=============================================================================

    /// <summary>A cylinder, its position and radius divided by the length factor.</summary>
    public GeomToStep_MakeCylindricalSurface(Geom_CylindricalSurface CS, in StepData_Factors theLocalFactors)
    {
        StepGeom_CylindricalSurface CSstep;
        StepGeom_Axis2Placement3d aPosition;
        double aRadius;

        GeomToStep_MakeAxis2Placement3d MkAxis2 = new GeomToStep_MakeAxis2Placement3d(CS.Position(), theLocalFactors);
        aPosition = MkAxis2.Value();
        aRadius = CS.Radius();
        CSstep = new StepGeom_CylindricalSurface();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        CSstep.Init(name, aPosition, aRadius / theLocalFactors.LengthFactor());
        theCylindricalSurface = CSstep;
        done = true;
    }

    /// <summary>A cylinder (default factors).</summary>
    public GeomToStep_MakeCylindricalSurface(Geom_CylindricalSurface CS) : this(CS, new StepData_Factors())
    {
    }

    /// <summary>The cylindrical_surface made.</summary>
    public StepGeom_CylindricalSurface Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeCylindricalSurface::Value() - no result");
        return theCylindricalSurface;
    }
}
