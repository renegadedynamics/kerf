// Port of: src/ModelingData/TKG3d/Geom/Geom_Axis2Placement.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Axis2Placement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>
/// A right-handed coordinate system in 3D space (a shareable <see cref="Frame3D"/>): an origin and three
/// orthogonal unit vectors, X, Y and the main direction (Z = X ^ Y). Changing the main direction recomputes X
/// and Y; changing X or Y keeps the main direction.
/// </summary>
internal class Axis2Placement : AxisPlacement
{
    private Direction3D _vxdir = new Direction3D();
    private Direction3D _vydir = new Direction3D();

    /// <summary>A coordinate system equal to A2.</summary>
    public Axis2Placement(in Frame3D a2)
    {
        _vxdir = a2.XDirection;
        _vydir = a2.YDirection;
        _axis = a2.Axis;
    }

    /// <summary>
    /// The coordinate system of origin P, main direction N and X direction N ^ (Vx ^ N) (Vx itself if it is
    /// normal to N).
    /// </summary>
    /// <exception cref="ConstructionErrorException">N and Vx are parallel.</exception>
    public Axis2Placement(in Point3D p, in Direction3D n, in Direction3D vx)
    {
        _axis = new Axis3D(p, n);
        _vxdir = n.CrossCrossed(vx, n);
        _vydir = n.Crossed(_vxdir);
    }

    private Axis2Placement(in Point3D p, in Direction3D vz, in Direction3D vx, in Direction3D vy)
    {
        _vxdir = vx;
        _vydir = vy;
        _axis = _axis.WithLocation(p);
        _axis = _axis.WithDirection(vz);
    }

    /// <summary>A new, unfrozen coordinate system equal to this one.</summary>
    public override Geometry3D Copy()
    {
        Axis2Placement a2;
        a2 = new Axis2Placement(_axis.Location, _axis.Direction, _vxdir, _vydir);
        return a2;
    }

    /// <summary>The X direction.</summary>
    public Direction3D XDirection() => _vxdir;

    /// <summary>The Y direction.</summary>
    public Direction3D YDirection() => _vydir;

    internal void SetAx2(in Frame3D a2)
    {
        CheckMutable();
        _vxdir = a2.XDirection;
        _vydir = a2.YDirection;
        _axis = a2.Axis;
    }

    /// <summary>Changes the main direction; the new X direction is V ^ (previous X ^ V).</summary>
    /// <exception cref="ConstructionErrorException">V is parallel to the previous X direction.</exception>
    protected internal override void SetDirection(in Direction3D v)
    {
        CheckMutable();
        _axis = _axis.WithDirection(v);
        _vxdir = v.CrossCrossed(_vxdir, v);
        _vydir = v.Crossed(_vxdir);
    }

    /// <summary>Changes the X direction to Direction ^ (Vx ^ Direction); the main direction is kept.</summary>
    /// <exception cref="ConstructionErrorException">Vx is parallel to the main direction.</exception>
    internal void SetXDirection(in Direction3D vx)
    {
        CheckMutable();
        _vxdir = _axis.Direction.CrossCrossed(vx, _axis.Direction);
        _vydir = _axis.Direction.Crossed(_vxdir);
    }

    /// <summary>Changes the Y direction to the projection of Vy; the main direction is kept, X is recomputed.</summary>
    /// <exception cref="ConstructionErrorException">Vy is parallel to the main direction.</exception>
    internal void SetYDirection(in Direction3D vy)
    {
        CheckMutable();
        _vxdir = vy.Crossed(_axis.Direction);
        _vydir = (_axis.Direction).Crossed(_vxdir);
    }

    /// <summary>The coordinate system as a <see cref="Frame3D"/>.</summary>
    public Frame3D Ax2() => new Frame3D(_axis.Location, _axis.Direction, _vxdir);

    /// <summary>
    /// Applies T: the origin and the X and Y directions are transformed, and the main direction becomes X ^ Y.
    /// </summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        // axis.Location().Transform (T);
        _axis = _axis.WithLocation(_axis.Location.Transformed(t)); // 10-03-93
        _vxdir = _vxdir.Transformed(t);
        _vydir = _vydir.Transformed(t);
        _axis = _axis.WithDirection(_vxdir.Crossed(_vydir));
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Axis2Placement WithAx2(in Frame3D a2)
    {
        Axis2Placement copy = (Axis2Placement)Copy();
        copy.SetAx2(a2);
        return copy;
    }

    public Axis2Placement WithXDirection(in Direction3D vx)
    {
        Axis2Placement copy = (Axis2Placement)Copy();
        copy.SetXDirection(vx);
        return copy;
    }

    public Axis2Placement WithYDirection(in Direction3D vy)
    {
        Axis2Placement copy = (Axis2Placement)Copy();
        copy.SetYDirection(vy);
        return copy;
    }
}
