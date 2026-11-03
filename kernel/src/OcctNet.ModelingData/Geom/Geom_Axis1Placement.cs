// Port of: src/ModelingData/TKG3d/Geom/Geom_Axis1Placement.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Axis1Placement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>An axis in 3D space, an origin and a unit vector (a shareable <see cref="Axis3D"/>).</summary>
public class Geom_Axis1Placement : Geom_AxisPlacement
{
    /// <summary>An axis equal to A1.</summary>
    public Geom_Axis1Placement(in Axis3D A1) => axis = A1;

    /// <summary>The axis of origin P and direction V.</summary>
    public Geom_Axis1Placement(in Point3D P, in Direction3D V) => axis = new Axis3D(P, V);

    /// <summary>A new, unfrozen axis equal to this one.</summary>
    public override Geom_Geometry Copy()
    {
        Geom_Axis1Placement A1;
        A1 = new Geom_Axis1Placement(axis);
        return A1;
    }

    /// <summary>Assigns V as the direction.</summary>
    protected internal override void SetDirection(in Direction3D V)
    {
        CheckMutable();
        axis.SetDirection(V);
    }

    /// <summary>The axis as a <see cref="Axis3D"/>.</summary>
    public Axis3D Ax1() => Axis();

    /// <summary>Reverses the direction in place.</summary>
    internal void Reverse()
    {
        CheckMutable();
        axis.Reverse();
    }

    /// <summary>Applies T to this axis.</summary>
    protected internal override void Transform(in Transform3D T)
    {
        CheckMutable();
        axis.Transform(T);
    }

    /// <summary>A copy with the direction reversed.</summary>
    public Geom_Axis1Placement Reversed()
    {
        Axis3D A1 = axis;
        A1.Reverse();
        Geom_Axis1Placement Temp = new Geom_Axis1Placement(A1);
        return Temp;
    }
}
