// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeDirection.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeDirection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Geometry;

namespace Occt;

/// <summary>The mapping of directions (Direction from Geom and Geom2d, Dir and Dir2d from gp) to a STEP direction.</summary>
public sealed class GeomToStep_MakeDirection : GeomToStep_Root
{
    private StepGeom_Direction theDirection;

    //=============================================================================
    // Creation d' une direction de prostep a partir d' une Dir de gp
    //=============================================================================

    /// <summary>A 3D direction.</summary>
    public GeomToStep_MakeDirection(in Direction3D D)
    {
        StepGeom_Direction Dir = new StepGeom_Direction();
        NCollection_HArray1<double> aDirRatios = new NCollection_HArray1<double>(1, 3);
        D.Coord(out double X, out double Y, out double Z);
        aDirRatios.SetValue(1, X);
        aDirRatios.SetValue(2, Y);
        aDirRatios.SetValue(3, Z);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Dir.Init(name, aDirRatios);
        theDirection = Dir;
        done = true;
    }

    //=============================================================================
    // Creation d' une direction de prostep a partir d' une Dir2d de gp
    //=============================================================================

    /// <summary>A 2D direction.</summary>
    public GeomToStep_MakeDirection(in Direction2D D)
    {
        StepGeom_Direction Dir = new StepGeom_Direction();
        NCollection_HArray1<double> aDirRatios = new NCollection_HArray1<double>(1, 2);
        D.Coord(out double X, out double Y);
        aDirRatios.SetValue(1, X);
        aDirRatios.SetValue(2, Y);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Dir.Init(name, aDirRatios);
        theDirection = Dir;
        done = true;
    }

    //=============================================================================
    // Creation d' une direction de prostep a partir d' une Direction de Geom
    //=============================================================================

    /// <summary>A 3D direction.</summary>
    public GeomToStep_MakeDirection(Geom_Direction Direc)
    {
        Direction3D D;
        StepGeom_Direction Dir = new StepGeom_Direction();
        NCollection_HArray1<double> aDirRatios = new NCollection_HArray1<double>(1, 3);
        D = Direc.Dir();
        D.Coord(out double X, out double Y, out double Z);
        aDirRatios.SetValue(1, X);
        aDirRatios.SetValue(2, Y);
        aDirRatios.SetValue(3, Z);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Dir.Init(name, aDirRatios);
        theDirection = Dir;
        done = true;
    }

    //=============================================================================
    // Creation d' une direction de prostep a partir d' une Direction de Geom2d
    //=============================================================================

    /// <summary>A 2D direction.</summary>
    public GeomToStep_MakeDirection(Geom2d_Direction Direc)
    {
        Direction2D D;
        StepGeom_Direction Dir = new StepGeom_Direction();
        NCollection_HArray1<double> aDirRatios = new NCollection_HArray1<double>(1, 2);
        D = Direc.Dir2d();
        D.Coord(out double X, out double Y);
        aDirRatios.SetValue(1, X);
        aDirRatios.SetValue(2, Y);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Dir.Init(name, aDirRatios);
        theDirection = Dir;
        done = true;
    }

    /// <summary>The direction made.</summary>
    public StepGeom_Direction Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeDirection::Value() - no result");
        return theDirection;
    }
}
