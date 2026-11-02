// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCartesianPoint.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCartesianPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// The mapping of points (CartesianPoint from Geom and Geom2d, Pnt and Pnt2d from gp) to a STEP cartesian_point.
/// </summary>
public sealed class GeomToStep_MakeCartesianPoint : GeomToStep_Root
{
    private StepGeom_CartesianPoint theCartesianPoint;

    //=============================================================================
    // Creation d' un cartesian_point de prostep a partir d' un point3d de gp
    //=============================================================================

    /// <summary>A 3D point, its coordinates divided by <paramref name="aFactor"/> (the length factor).</summary>
    public GeomToStep_MakeCartesianPoint(in gp_Pnt P, double aFactor)
    {
        StepGeom_CartesianPoint Pstep = new StepGeom_CartesianPoint();
        P.Coord(out double X, out double Y, out double Z);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Pstep.Init3D(name, X / aFactor, Y / aFactor, Z / aFactor);
        theCartesianPoint = Pstep;
        done = true;
    }

    //=============================================================================
    // Creation d' un cartesian_point de prostep a partir d' un point 2d de gp
    //=============================================================================

    /// <summary>A 2D point (the factor isn't used: 2D points are parameters).</summary>
    public GeomToStep_MakeCartesianPoint(in gp_Pnt2d P, double aFactor)
    {
        StepGeom_CartesianPoint Pstep = new StepGeom_CartesianPoint();
        P.Coord(out double X, out double Y);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Pstep.Init2D(name, X, Y);
        theCartesianPoint = Pstep;
        done = true;
    }

    //=============================================================================
    // Creation d' un cartesian_point de prostep a partir d' un point 3d de Geom
    //=============================================================================

    /// <summary>A 3D point, its coordinates divided by <paramref name="aFactor"/>.</summary>
    public GeomToStep_MakeCartesianPoint(Geom_CartesianPoint P, double aFactor)
    {
        StepGeom_CartesianPoint Pstep = new StepGeom_CartesianPoint();
        P.Coord(out double X, out double Y, out double Z);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Pstep.Init3D(name, X / aFactor, Y / aFactor, Z / aFactor);
        theCartesianPoint = Pstep;
        done = true;
    }

    //=============================================================================
    // Creation d' un cartesian_point de prostep a partir d' un point 2d de Geom2d
    //=============================================================================

    /// <summary>A 2D point.</summary>
    public GeomToStep_MakeCartesianPoint(Geom2d_CartesianPoint P)
    {
        StepGeom_CartesianPoint Pstep = new StepGeom_CartesianPoint();
        P.Coord(out double X, out double Y);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Pstep.Init2D(name, X, Y);
        theCartesianPoint = Pstep;
        done = true;
    }

    /// <summary>The cartesian_point made.</summary>
    public StepGeom_CartesianPoint Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeCartesianPoint::Value() - no result");
        return theCartesianPoint;
    }
}
