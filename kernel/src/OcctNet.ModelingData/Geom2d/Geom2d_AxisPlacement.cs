// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_AxisPlacement.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_AxisPlacement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An axis in the plane, an origin and a unit vector (a shareable <see cref="gp_Ax2d"/>).</summary>
public class Geom2d_AxisPlacement : Geom2d_Geometry
{
    private gp_Ax2d axis;

    /// <summary>An axis equal to A.</summary>
    public Geom2d_AxisPlacement(in gp_Ax2d A) => axis = A;

    /// <summary>The axis of origin P and direction V.</summary>
    public Geom2d_AxisPlacement(in gp_Pnt2d P, in gp_Dir2d V) => axis = new gp_Ax2d(P, V);

    /// <summary>A new, unfrozen axis equal to this one.</summary>
    public override Geom2d_Geometry Copy()
    {
        Geom2d_AxisPlacement A;
        A = new Geom2d_AxisPlacement(axis);
        return A;
    }

    /// <summary>The axis as a <see cref="gp_Ax2d"/>.</summary>
    public gp_Ax2d Ax2d() => axis;

    /// <summary>The direction.</summary>
    public gp_Dir2d Direction() => axis.Direction();

    /// <summary>The origin.</summary>
    public gp_Pnt2d Location() => axis.Location();

    /// <summary>Reverses the direction in place.</summary>
    internal void Reverse()
    {
        CheckMutable();
        axis.Reverse();
    }

    /// <summary>A copy with the direction reversed.</summary>
    public Geom2d_AxisPlacement Reversed()
    {
        gp_Ax2d A = axis;
        A.Reverse();
        Geom2d_AxisPlacement Temp = new Geom2d_AxisPlacement(A);
        return Temp;
    }

    /// <summary>Applies T to this axis.</summary>
    protected internal override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        axis.Transform(T);
    }

    internal void SetAxis(in gp_Ax2d A)
    {
        CheckMutable();
        axis = A;
    }

    internal void SetLocation(in gp_Pnt2d P)
    {
        CheckMutable();
        axis.SetLocation(P);
    }

    internal void SetDirection(in gp_Dir2d V)
    {
        CheckMutable();
        axis.SetDirection(V);
    }

    /// <summary>The angle, in [-PI, PI], between the directions of this axis and Other.</summary>
    public double Angle(Geom2d_AxisPlacement Other) => axis.Angle(Other.Ax2d());

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom2d_AxisPlacement WithAxis(in gp_Ax2d A)
    {
        Geom2d_AxisPlacement aCopy = (Geom2d_AxisPlacement)Copy();
        aCopy.SetAxis(A);
        return aCopy;
    }

    public Geom2d_AxisPlacement WithLocation(in gp_Pnt2d P)
    {
        Geom2d_AxisPlacement aCopy = (Geom2d_AxisPlacement)Copy();
        aCopy.SetLocation(P);
        return aCopy;
    }

    public Geom2d_AxisPlacement WithDirection(in gp_Dir2d V)
    {
        Geom2d_AxisPlacement aCopy = (Geom2d_AxisPlacement)Copy();
        aCopy.SetDirection(V);
        return aCopy;
    }
}
