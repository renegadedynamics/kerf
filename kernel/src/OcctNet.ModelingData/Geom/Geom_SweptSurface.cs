// Port of: src/ModelingData/TKG3d/Geom/Geom_SweptSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_SweptSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A surface built by sweeping a curve: the surface of revolution and the surface of linear extrusion.
/// </summary>
public abstract class Geom_SweptSurface : Geom_Surface
{
    protected Geom_Curve? basisCurve;
    protected gp_Dir direction = new gp_Dir();
    protected GeomAbs_Shape smooth;

    /// <summary>The continuity of the surface.</summary>
    public override GeomAbs_Shape Continuity() => smooth;

    /// <summary>The reference direction: the axis of revolution, or the direction of extrusion.</summary>
    public gp_Dir Direction() => direction;

    /// <summary>The swept curve: the revolved curve, or the extruded curve.</summary>
    /// <remarks>Set by the constructors of the derived classes; null only while one is running.</remarks>
    public Geom_Curve BasisCurve() => basisCurve!;
}
