// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_AxisPlacement.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_AxisPlacement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>An axis in the plane, an origin and a unit vector (a shareable <see cref="Axis2D"/>).</summary>
public class Geom2d_AxisPlacement : Geom2d_Geometry
{
    private Axis2D axis;

    /// <summary>An axis equal to A.</summary>
    public Geom2d_AxisPlacement(in Axis2D A) => axis = A;

    /// <summary>The axis of origin P and direction V.</summary>
    public Geom2d_AxisPlacement(in Point2D P, in Direction2D V) => axis = new Axis2D(P, V);

    /// <summary>A new, unfrozen axis equal to this one.</summary>
    public override Geom2d_Geometry Copy()
    {
        Geom2d_AxisPlacement A;
        A = new Geom2d_AxisPlacement(axis);
        return A;
    }

    /// <summary>The axis as a <see cref="Axis2D"/>.</summary>
    public Axis2D Ax2d() => axis;

    /// <summary>The direction.</summary>
    public Direction2D Direction() => axis.Direction();

    /// <summary>The origin.</summary>
    public Point2D Location() => axis.Location();

    /// <summary>Reverses the direction in place.</summary>
    internal void Reverse()
    {
        CheckMutable();
        axis.Reverse();
    }

    /// <summary>A copy with the direction reversed.</summary>
    public Geom2d_AxisPlacement Reversed()
    {
        Axis2D A = axis;
        A.Reverse();
        Geom2d_AxisPlacement Temp = new Geom2d_AxisPlacement(A);
        return Temp;
    }

    /// <summary>Applies T to this axis.</summary>
    protected internal override void Transform(in Transform2D T)
    {
        CheckMutable();
        axis.Transform(T);
    }

    internal void SetAxis(in Axis2D A)
    {
        CheckMutable();
        axis = A;
    }

    internal void SetLocation(in Point2D P)
    {
        CheckMutable();
        axis.SetLocation(P);
    }

    internal void SetDirection(in Direction2D V)
    {
        CheckMutable();
        axis.SetDirection(V);
    }

    /// <summary>The angle, in [-PI, PI], between the directions of this axis and Other.</summary>
    public double Angle(Geom2d_AxisPlacement Other) => axis.Angle(Other.Ax2d());

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom2d_AxisPlacement WithAxis(in Axis2D A)
    {
        Geom2d_AxisPlacement aCopy = (Geom2d_AxisPlacement)Copy();
        aCopy.SetAxis(A);
        return aCopy;
    }

    public Geom2d_AxisPlacement WithLocation(in Point2D P)
    {
        Geom2d_AxisPlacement aCopy = (Geom2d_AxisPlacement)Copy();
        aCopy.SetLocation(P);
        return aCopy;
    }

    public Geom2d_AxisPlacement WithDirection(in Direction2D V)
    {
        Geom2d_AxisPlacement aCopy = (Geom2d_AxisPlacement)Copy();
        aCopy.SetDirection(V);
        return aCopy;
    }
}
