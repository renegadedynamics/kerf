// Port of: src/ModelingData/TKG3d/Geom/Geom_Axis2Placement.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Axis2Placement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A right-handed coordinate system in 3D space (a shareable <see cref="gp_Ax2"/>): an origin and three
/// orthogonal unit vectors, X, Y and the main direction (Z = X ^ Y). Changing the main direction recomputes X
/// and Y; changing X or Y keeps the main direction.
/// </summary>
public class Geom_Axis2Placement : Geom_AxisPlacement
{
    private gp_Dir vxdir = new gp_Dir();
    private gp_Dir vydir = new gp_Dir();

    /// <summary>A coordinate system equal to A2.</summary>
    public Geom_Axis2Placement(in gp_Ax2 A2)
    {
        vxdir = A2.XDirection();
        vydir = A2.YDirection();
        axis = A2.Axis();
    }

    /// <summary>
    /// The coordinate system of origin P, main direction N and X direction N ^ (Vx ^ N) (Vx itself if it is
    /// normal to N).
    /// </summary>
    /// <exception cref="Standard_ConstructionError">N and Vx are parallel.</exception>
    public Geom_Axis2Placement(in gp_Pnt P, in gp_Dir N, in gp_Dir Vx)
    {
        axis = new gp_Ax1(P, N);
        vxdir = N.CrossCrossed(Vx, N);
        vydir = N.Crossed(vxdir);
    }

    private Geom_Axis2Placement(in gp_Pnt P, in gp_Dir Vz, in gp_Dir Vx, in gp_Dir Vy)
    {
        vxdir = Vx;
        vydir = Vy;
        axis.SetLocation(P);
        axis.SetDirection(Vz);
    }

    /// <summary>A new, unfrozen coordinate system equal to this one.</summary>
    public override Geom_Geometry Copy()
    {
        Geom_Axis2Placement A2;
        A2 = new Geom_Axis2Placement(axis.Location(), axis.Direction(), vxdir, vydir);
        return A2;
    }

    /// <summary>The X direction.</summary>
    public gp_Dir XDirection() => vxdir;

    /// <summary>The Y direction.</summary>
    public gp_Dir YDirection() => vydir;

    internal void SetAx2(in gp_Ax2 A2)
    {
        CheckMutable();
        vxdir = A2.XDirection();
        vydir = A2.YDirection();
        axis = A2.Axis();
    }

    /// <summary>Changes the main direction; the new X direction is V ^ (previous X ^ V).</summary>
    /// <exception cref="Standard_ConstructionError">V is parallel to the previous X direction.</exception>
    protected internal override void SetDirection(in gp_Dir V)
    {
        CheckMutable();
        axis.SetDirection(V);
        vxdir = V.CrossCrossed(vxdir, V);
        vydir = V.Crossed(vxdir);
    }

    /// <summary>Changes the X direction to Direction ^ (Vx ^ Direction); the main direction is kept.</summary>
    /// <exception cref="Standard_ConstructionError">Vx is parallel to the main direction.</exception>
    internal void SetXDirection(in gp_Dir Vx)
    {
        CheckMutable();
        vxdir = axis.Direction().CrossCrossed(Vx, axis.Direction());
        vydir = axis.Direction().Crossed(vxdir);
    }

    /// <summary>Changes the Y direction to the projection of Vy; the main direction is kept, X is recomputed.</summary>
    /// <exception cref="Standard_ConstructionError">Vy is parallel to the main direction.</exception>
    internal void SetYDirection(in gp_Dir Vy)
    {
        CheckMutable();
        vxdir = Vy.Crossed(axis.Direction());
        vydir = (axis.Direction()).Crossed(vxdir);
    }

    /// <summary>The coordinate system as a <see cref="gp_Ax2"/>.</summary>
    public gp_Ax2 Ax2() => new gp_Ax2(axis.Location(), axis.Direction(), vxdir);

    /// <summary>
    /// Applies T: the origin and the X and Y directions are transformed, and the main direction becomes X ^ Y.
    /// </summary>
    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        // axis.Location().Transform (T);
        axis.SetLocation(axis.Location().Transformed(T)); // 10-03-93
        vxdir.Transform(T);
        vydir.Transform(T);
        axis.SetDirection(vxdir.Crossed(vydir));
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom_Axis2Placement WithAx2(in gp_Ax2 A2)
    {
        Geom_Axis2Placement aCopy = (Geom_Axis2Placement)Copy();
        aCopy.SetAx2(A2);
        return aCopy;
    }

    public Geom_Axis2Placement WithXDirection(in gp_Dir Vx)
    {
        Geom_Axis2Placement aCopy = (Geom_Axis2Placement)Copy();
        aCopy.SetXDirection(Vx);
        return aCopy;
    }

    public Geom_Axis2Placement WithYDirection(in gp_Dir Vy)
    {
        Geom_Axis2Placement aCopy = (Geom_Axis2Placement)Copy();
        aCopy.SetYDirection(Vy);
        return aCopy;
    }
}
