// Port of: src/ModelingData/TKG3d/Geom/Geom_ElementarySurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_ElementarySurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A surface with a simple parametric equation in a local coordinate system (gp_Ax3): the plane, the cylinder,
/// the cone, the sphere and the torus. The surface is oriented: its normal is always defined.
/// </summary>
/// <remarks>
/// The local coordinate system need not be right-handed: if it is left-handed, the main direction is
/// -(XDirection ^ YDirection) and the trigonometric senses defined by the two are opposite.
/// </remarks>
public abstract class Geom_ElementarySurface : Geom_Surface
{
    protected gp_Ax3 pos = new gp_Ax3();

    /// <summary>Changes the main axis (Z axis).</summary>
    /// <exception cref="Standard_ConstructionError">theA1 is parallel to the X axis of the coordinate system.</exception>
    internal void SetAxis(in gp_Ax1 theA1)
    {
        CheckMutable();
        pos.SetAxis(theA1);
    }

    internal void SetLocation(in gp_Pnt theLoc)
    {
        CheckMutable();
        pos.SetLocation(theLoc);
    }

    internal void SetPosition(in gp_Ax3 theAx3)
    {
        CheckMutable();
        pos = theAx3;
    }

    /// <summary>A copy with the main axis theA1.</summary>
    public Geom_ElementarySurface WithAxis(in gp_Ax1 theA1)
    {
        Geom_ElementarySurface aCopy = (Geom_ElementarySurface)Copy();
        aCopy.SetAxis(theA1);
        return aCopy;
    }

    /// <summary>A copy whose local coordinate system is located at theLoc.</summary>
    public Geom_ElementarySurface WithLocation(in gp_Pnt theLoc)
    {
        Geom_ElementarySurface aCopy = (Geom_ElementarySurface)Copy();
        aCopy.SetLocation(theLoc);
        return aCopy;
    }

    /// <summary>A copy with the local coordinate system theAx3.</summary>
    public Geom_ElementarySurface WithPosition(in gp_Ax3 theAx3)
    {
        Geom_ElementarySurface aCopy = (Geom_ElementarySurface)Copy();
        aCopy.SetPosition(theAx3);
        return aCopy;
    }

    /// <summary>The main axis (Z axis) of the surface.</summary>
    public gp_Ax1 Axis() => pos.Axis();

    /// <summary>The origin of the local coordinate system.</summary>
    public gp_Pnt Location() => pos.Location();

    /// <summary>The local coordinate system.</summary>
    public gp_Ax3 Position() => pos;

    /// <summary>Reverses the U direction (reverses the Y direction of the local coordinate system).</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        pos.YReverse();
    }

    /// <summary>Reverses the V direction (reverses the main direction of the local coordinate system).</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        pos.ZReverse();
    }

    /// <summary>CN, the continuity of every elementary surface.</summary>
    public override GeomAbs_Shape Continuity() => GeomAbs_CN;

    /// <summary>True.</summary>
    public override bool IsCNu(int N) => true;

    /// <summary>True.</summary>
    public override bool IsCNv(int N) => true;
}
