// Port of: src/ModelingData/TKG3d/Geom/Geom_AxisPlacement.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_AxisPlacement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>
/// The common behaviour of positioning systems in 3D space: the axis (<see cref="Axis1Placement"/>) and the
/// right-handed coordinate system (<see cref="Axis2Placement"/>). Both have an origin (location) and a main
/// direction, which together form the main axis.
/// </summary>
internal abstract class AxisPlacement : Geometry3D
{
    private protected Axis3D _axis = new Axis3D();

    /// <summary>Assigns A1 as the main axis (origin and main direction).</summary>
    /// <remarks>
    /// OCCT documents that the X and Y directions of a <see cref="Axis2Placement"/> are recomputed, but the code
    /// only assigns the axis (see docs/occt-issues.md).
    /// </remarks>
    internal void SetAxis(in Axis3D a1)
    {
        CheckMutable();
        // OCCT: for a Geom_Axis2Placement the X and Y directions are not recomputed, as documented, so they are no
        // longer perpendicular to the new main direction.
        _axis = a1;
    }

    /// <summary>
    /// Changes the main direction; for a <see cref="Axis2Placement"/> the X and Y directions are recomputed.
    /// </summary>
    /// <exception cref="ConstructionErrorException">A Geom_Axis2Placement whose X direction is parallel to V.</exception>
    protected internal abstract void SetDirection(in Direction3D v);

    /// <summary>Assigns P as the origin.</summary>
    internal void SetLocation(in Point3D p)
    {
        CheckMutable();
        _axis = _axis.WithLocation(p);
    }

    /// <summary>A copy with the main axis A1 (as <c>SetAxis</c>, X and Y directions unchanged).</summary>
    public AxisPlacement WithAxis(in Axis3D a1)
    {
        AxisPlacement copy = (AxisPlacement)Copy();
        copy.SetAxis(a1);
        return copy;
    }

    /// <summary>A copy with the main direction V.</summary>
    public AxisPlacement WithDirection(in Direction3D v)
    {
        AxisPlacement copy = (AxisPlacement)Copy();
        copy.SetDirection(v);
        return copy;
    }

    /// <summary>A copy with the origin P.</summary>
    public AxisPlacement WithLocation(in Point3D p)
    {
        AxisPlacement copy = (AxisPlacement)Copy();
        copy.SetLocation(p);
        return copy;
    }

    /// <summary>The angle, in [0, PI], between the main directions of this and Other.</summary>
    public double Angle(AxisPlacement other) => _axis.AngleTo(other.Axis());

    /// <summary>The main axis (origin, main direction).</summary>
    public Axis3D Axis() => _axis;

    /// <summary>The main direction.</summary>
    public Direction3D Direction() => _axis.Direction;

    /// <summary>The origin.</summary>
    public Point3D Location() => _axis.Location;
}
