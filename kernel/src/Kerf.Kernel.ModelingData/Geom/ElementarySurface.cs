// Port of: src/ModelingData/TKG3d/Geom/Geom_ElementarySurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_ElementarySurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Geometry;

/// <summary>
/// A surface with a simple parametric equation in a local coordinate system (gp_Ax3): the plane, the cylinder,
/// the cone, the sphere and the torus. The surface is oriented: its normal is always defined.
/// </summary>
/// <remarks>
/// The local coordinate system need not be right-handed: if it is left-handed, the main direction is
/// -(XDirection ^ YDirection) and the trigonometric senses defined by the two are opposite.
/// </remarks>
public abstract class ElementarySurface : Surface
{
    /// <summary>Only the kernel derives from it (unit 10L1).</summary>
    private protected ElementarySurface()
    {
    }

    private protected CoordinateSystem _pos = new CoordinateSystem();

    /// <summary>Changes the main axis (Z axis).</summary>
    /// <exception cref="ConstructionErrorException">theA1 is parallel to the X axis of the coordinate system.</exception>
    internal void SetAxis(in Axis3D a1)
    {
        CheckMutable();
        _pos = _pos.WithAxis(a1);
    }

    internal void SetLocation(in Point3D loc)
    {
        CheckMutable();
        _pos = _pos.WithLocation(loc);
    }

    internal void SetPosition(in CoordinateSystem ax3)
    {
        CheckMutable();
        _pos = ax3;
    }

    /// <summary>A copy with the main axis theA1.</summary>
    public ElementarySurface WithAxis(in Axis3D a1)
    {
        ElementarySurface copy = (ElementarySurface)Copy();
        copy.SetAxis(a1);
        return copy;
    }

    /// <summary>A copy whose local coordinate system is located at theLoc.</summary>
    public ElementarySurface WithLocation(in Point3D loc)
    {
        ElementarySurface copy = (ElementarySurface)Copy();
        copy.SetLocation(loc);
        return copy;
    }

    /// <summary>A copy with the local coordinate system theAx3.</summary>
    public ElementarySurface WithPosition(in CoordinateSystem ax3)
    {
        ElementarySurface copy = (ElementarySurface)Copy();
        copy.SetPosition(ax3);
        return copy;
    }

    /// <summary>The main axis (Z axis) of the surface.</summary>
    public Axis3D Axis => _pos.Axis;

    /// <summary>The origin of the local coordinate system.</summary>
    public Point3D Location => _pos.Location;

    /// <summary>The local coordinate system.</summary>
    public CoordinateSystem Position => _pos;

    /// <summary>Reverses the U direction (reverses the Y direction of the local coordinate system).</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        _pos = _pos.YReversed();
    }

    /// <summary>Reverses the V direction (reverses the main direction of the local coordinate system).</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        _pos = _pos.ZReversed();
    }

    /// <summary>CN, the continuity of every elementary surface.</summary>
    public override Continuity Continuity => CN;

    /// <summary>True.</summary>
    internal override bool IsCNu(int n) => true;

    /// <summary>True.</summary>
    internal override bool IsCNv(int n) => true;
}
