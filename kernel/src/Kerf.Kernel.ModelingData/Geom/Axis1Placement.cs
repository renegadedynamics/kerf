// Port of: src/ModelingData/TKG3d/Geom/Geom_Axis1Placement.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Axis1Placement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>An axis in 3D space, an origin and a unit vector (a shareable <see cref="Axis3D"/>).</summary>
public class Axis1Placement : AxisPlacement
{
    /// <summary>An axis equal to A1.</summary>
    public Axis1Placement(in Axis3D a1) => _axis = a1;

    /// <summary>The axis of origin P and direction V.</summary>
    public Axis1Placement(in Point3D p, in Direction3D v) => _axis = new Axis3D(p, v);

    /// <summary>A new, unfrozen axis equal to this one.</summary>
    public override Geometry3D Copy()
    {
        Axis1Placement a1;
        a1 = new Axis1Placement(_axis);
        return a1;
    }

    /// <summary>Assigns V as the direction.</summary>
    protected internal override void SetDirection(in Direction3D v)
    {
        CheckMutable();
        _axis.SetDirection(v);
    }

    /// <summary>The axis as a <see cref="Axis3D"/>.</summary>
    public Axis3D Ax1() => Axis();

    /// <summary>Reverses the direction in place.</summary>
    internal void Reverse()
    {
        CheckMutable();
        _axis.Reverse();
    }

    /// <summary>Applies T to this axis.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        _axis.Transform(t);
    }

    /// <summary>A copy with the direction reversed.</summary>
    public Axis1Placement Reversed()
    {
        Axis3D a1 = _axis;
        a1.Reverse();
        Axis1Placement temp = new Axis1Placement(a1);
        return temp;
    }
}
