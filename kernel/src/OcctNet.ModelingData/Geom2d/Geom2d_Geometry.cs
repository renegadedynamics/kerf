// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Geometry.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Geometry.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The root of the geometric objects of the Geom2d package (points, vectors, axes, curves in the plane). Only
/// transformations that keep the nature of the geometry apply (<see cref="gp_Trsf2d"/>).
/// </summary>
/// <remarks>
/// Geom2d objects are immutable to callers: <see cref="Transformed"/>, <see cref="Mirrored(in gp_Pnt2d)"/>, ...
/// and the <c>With</c> methods of the derived classes return changed copies. Inside OCCT.NET the in-place
/// mutators remain (internal), and throw <see cref="Standard_ImmutableObject"/> once the object is frozen.
/// <see cref="Copy"/> always returns an unfrozen object.
/// </remarks>
public abstract class Geom2d_Geometry : Standard_Transient
{
    /// <summary>Mirrors this geometry about the point P.</summary>
    internal void Mirror(in gp_Pnt2d P)
    {
        CheckMutable();
        gp_Trsf2d T = new gp_Trsf2d();
        T.SetMirror(P);
        Transform(T);
    }

    /// <summary>Mirrors this geometry about the axis A.</summary>
    internal void Mirror(in gp_Ax2d A)
    {
        CheckMutable();
        gp_Trsf2d T = new gp_Trsf2d();
        T.SetMirror(A);
        Transform(T);
    }

    /// <summary>Rotates this geometry by Ang radians about P.</summary>
    internal void Rotate(in gp_Pnt2d P, double Ang)
    {
        CheckMutable();
        gp_Trsf2d T = new gp_Trsf2d();
        T.SetRotation(P, Ang);
        Transform(T);
    }

    /// <summary>Scales this geometry by S about P.</summary>
    internal void Scale(in gp_Pnt2d P, double S)
    {
        CheckMutable();
        gp_Trsf2d T = new gp_Trsf2d();
        T.SetScale(P, S);
        Transform(T);
    }

    /// <summary>Translates this geometry by V.</summary>
    internal void Translate(in gp_Vec2d V)
    {
        CheckMutable();
        gp_Trsf2d T = new gp_Trsf2d();
        T.SetTranslation(V);
        Transform(T);
    }

    /// <summary>Translates this geometry from P1 to P2.</summary>
    internal void Translate(in gp_Pnt2d P1, in gp_Pnt2d P2)
    {
        CheckMutable();
        gp_Vec2d V = new gp_Vec2d(P1, P2);
        Translate(V);
    }

    /// <summary>
    /// Transforms this geometry in place. Every concrete class implements it, starting with
    /// <see cref="Standard_Transient.CheckMutable"/>; the other transformations go through it.
    /// </summary>
    protected internal abstract void Transform(in gp_Trsf2d T);

    /// <summary>A copy mirrored about the point P.</summary>
    public Geom2d_Geometry Mirrored(in gp_Pnt2d P)
    {
        Geom2d_Geometry G = Copy();
        G.Mirror(P);
        return G;
    }

    /// <summary>A copy mirrored about the axis A.</summary>
    public Geom2d_Geometry Mirrored(in gp_Ax2d A)
    {
        Geom2d_Geometry G = Copy();
        G.Mirror(A);
        return G;
    }

    /// <summary>A copy rotated by Ang radians about P.</summary>
    public Geom2d_Geometry Rotated(in gp_Pnt2d P, double Ang)
    {
        Geom2d_Geometry G = Copy();
        G.Rotate(P, Ang);
        return G;
    }

    /// <summary>A copy scaled by S about P.</summary>
    public Geom2d_Geometry Scaled(in gp_Pnt2d P, double S)
    {
        Geom2d_Geometry G = Copy();
        G.Scale(P, S);
        return G;
    }

    /// <summary>A copy transformed by T.</summary>
    public Geom2d_Geometry Transformed(in gp_Trsf2d T)
    {
        Geom2d_Geometry G = Copy();
        G.Transform(T);
        return G;
    }

    /// <summary>A copy translated by V.</summary>
    public Geom2d_Geometry Translated(in gp_Vec2d V)
    {
        Geom2d_Geometry G = Copy();
        G.Translate(V);
        return G;
    }

    /// <summary>A copy translated from P1 to P2.</summary>
    public Geom2d_Geometry Translated(in gp_Pnt2d P1, in gp_Pnt2d P2)
    {
        Geom2d_Geometry G = Copy();
        G.Translate(P1, P2);
        return G;
    }

    /// <summary>A new object equal to this one, and not frozen even if this one is.</summary>
    public abstract Geom2d_Geometry Copy();
}
