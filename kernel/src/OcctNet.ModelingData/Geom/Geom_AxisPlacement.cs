// Port of: src/ModelingData/TKG3d/Geom/Geom_AxisPlacement.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_AxisPlacement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The common behaviour of positioning systems in 3D space: the axis (<see cref="Geom_Axis1Placement"/>) and the
/// right-handed coordinate system (<see cref="Geom_Axis2Placement"/>). Both have an origin (location) and a main
/// direction, which together form the main axis.
/// </summary>
public abstract class Geom_AxisPlacement : Geom_Geometry
{
    protected gp_Ax1 axis = new gp_Ax1();

    /// <summary>Assigns A1 as the main axis (origin and main direction).</summary>
    /// <remarks>
    /// OCCT documents that the X and Y directions of a <see cref="Geom_Axis2Placement"/> are recomputed, but the code
    /// only assigns the axis (see docs/occt-issues.md).
    /// </remarks>
    internal void SetAxis(in gp_Ax1 A1)
    {
        CheckMutable();
        // OCCT: for a Geom_Axis2Placement the X and Y directions are not recomputed, as documented, so they are no
        // longer perpendicular to the new main direction.
        axis = A1;
    }

    /// <summary>
    /// Changes the main direction; for a <see cref="Geom_Axis2Placement"/> the X and Y directions are recomputed.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">A Geom_Axis2Placement whose X direction is parallel to V.</exception>
    protected internal abstract void SetDirection(in gp_Dir V);

    /// <summary>Assigns P as the origin.</summary>
    internal void SetLocation(in gp_Pnt P)
    {
        CheckMutable();
        axis.SetLocation(P);
    }

    /// <summary>A copy with the main axis A1 (as <c>SetAxis</c>, X and Y directions unchanged).</summary>
    public Geom_AxisPlacement WithAxis(in gp_Ax1 A1)
    {
        Geom_AxisPlacement aCopy = (Geom_AxisPlacement)Copy();
        aCopy.SetAxis(A1);
        return aCopy;
    }

    /// <summary>A copy with the main direction V.</summary>
    public Geom_AxisPlacement WithDirection(in gp_Dir V)
    {
        Geom_AxisPlacement aCopy = (Geom_AxisPlacement)Copy();
        aCopy.SetDirection(V);
        return aCopy;
    }

    /// <summary>A copy with the origin P.</summary>
    public Geom_AxisPlacement WithLocation(in gp_Pnt P)
    {
        Geom_AxisPlacement aCopy = (Geom_AxisPlacement)Copy();
        aCopy.SetLocation(P);
        return aCopy;
    }

    /// <summary>The angle, in [0, PI], between the main directions of this and Other.</summary>
    public double Angle(Geom_AxisPlacement Other) => axis.Angle(Other.Axis());

    /// <summary>The main axis (origin, main direction).</summary>
    public gp_Ax1 Axis() => axis;

    /// <summary>The main direction.</summary>
    public gp_Dir Direction() => axis.Direction();

    /// <summary>The origin.</summary>
    public gp_Pnt Location() => axis.Location();
}
