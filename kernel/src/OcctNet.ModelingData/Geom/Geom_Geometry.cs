// Port of: src/ModelingData/TKG3d/Geom/Geom_Geometry.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Geometry.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The root of the geometric objects of the Geom package (points, vectors, axes, curves, surfaces). Only
/// transformations that keep the nature of the geometry apply: translations, rotations, symmetries, scales and
/// their compositions (<see cref="gp_Trsf"/>).
/// </summary>
/// <remarks>
/// Geom objects are immutable to callers: <see cref="Transformed"/>, <see cref="Mirrored(in gp_Pnt)"/>,
/// <see cref="Rotated"/>, ... and the <c>With</c> methods of the derived classes return changed copies. Inside
/// OCCT.NET the in-place mutators remain (internal), and throw <see cref="Standard_ImmutableObject"/> once the
/// object is frozen (shared, for instance by topology). <see cref="Copy"/> always returns an unfrozen object.
/// </remarks>
public abstract class Geom_Geometry : Standard_Transient
{
    /// <summary>Mirrors this geometry about the point P.</summary>
    internal void Mirror(in gp_Pnt P)
    {
        CheckMutable();
        gp_Trsf T = new gp_Trsf();
        T.SetMirror(P);
        Transform(T);
    }

    /// <summary>Mirrors this geometry about the axis A1.</summary>
    internal void Mirror(in gp_Ax1 A1)
    {
        CheckMutable();
        gp_Trsf T = new gp_Trsf();
        T.SetMirror(A1);
        Transform(T);
    }

    /// <summary>Mirrors this geometry about the plane (Location, XDirection, YDirection) of A2.</summary>
    internal void Mirror(in gp_Ax2 A2)
    {
        CheckMutable();
        gp_Trsf T = new gp_Trsf();
        T.SetMirror(A2);
        Transform(T);
    }

    /// <summary>Rotates this geometry by Ang radians about the axis A1.</summary>
    internal void Rotate(in gp_Ax1 A1, double Ang)
    {
        CheckMutable();
        gp_Trsf T = new gp_Trsf();
        T.SetRotation(A1, Ang);
        Transform(T);
    }

    /// <summary>Scales this geometry by S about the point P.</summary>
    internal void Scale(in gp_Pnt P, double S)
    {
        CheckMutable();
        gp_Trsf T = new gp_Trsf();
        T.SetScale(P, S);
        Transform(T);
    }

    /// <summary>Translates this geometry by V.</summary>
    internal void Translate(in gp_Vec V)
    {
        CheckMutable();
        gp_Trsf T = new gp_Trsf();
        T.SetTranslation(V);
        Transform(T);
    }

    /// <summary>Translates this geometry from P1 to P2.</summary>
    internal void Translate(in gp_Pnt P1, in gp_Pnt P2)
    {
        CheckMutable();
        gp_Vec V = new gp_Vec(P1, P2);
        Translate(V);
    }

    /// <summary>
    /// Transforms this geometry in place. Every concrete class implements it, starting with
    /// <see cref="Standard_Transient.CheckMutable"/>; the other transformations go through it.
    /// </summary>
    protected internal abstract void Transform(in gp_Trsf T);

    /// <summary>A copy mirrored about the point P.</summary>
    public Geom_Geometry Mirrored(in gp_Pnt P)
    {
        Geom_Geometry G = Copy();
        G.Mirror(P);
        return G;
    }

    /// <summary>A copy mirrored about the axis A1.</summary>
    public Geom_Geometry Mirrored(in gp_Ax1 A1)
    {
        Geom_Geometry G = Copy();
        G.Mirror(A1);
        return G;
    }

    /// <summary>A copy mirrored about the plane of A2.</summary>
    public Geom_Geometry Mirrored(in gp_Ax2 A2)
    {
        Geom_Geometry G = Copy();
        G.Mirror(A2);
        return G;
    }

    /// <summary>A copy rotated by Ang radians about A1.</summary>
    public Geom_Geometry Rotated(in gp_Ax1 A1, double Ang)
    {
        Geom_Geometry G = Copy();
        G.Rotate(A1, Ang);
        return G;
    }

    /// <summary>A copy scaled by S about P.</summary>
    public Geom_Geometry Scaled(in gp_Pnt P, double S)
    {
        Geom_Geometry G = Copy();
        G.Scale(P, S);
        return G;
    }

    /// <summary>A copy transformed by T.</summary>
    public Geom_Geometry Transformed(in gp_Trsf T)
    {
        Geom_Geometry G = Copy();
        G.Transform(T);
        return G;
    }

    /// <summary>A copy translated by V.</summary>
    public Geom_Geometry Translated(in gp_Vec V)
    {
        Geom_Geometry G = Copy();
        G.Translate(V);
        return G;
    }

    /// <summary>A copy translated from P1 to P2.</summary>
    public Geom_Geometry Translated(in gp_Pnt P1, in gp_Pnt P2)
    {
        Geom_Geometry G = Copy();
        G.Translate(P1, P2);
        return G;
    }

    /// <summary>A new object equal to this one, and not frozen even if this one is.</summary>
    /// <remarks>
    /// Pure virtual in OCCT, which also defines a body that throws <see cref="Standard_ConstructionError"/>;
    /// no class calls that body, so the port leaves it out.
    /// </remarks>
    public abstract Geom_Geometry Copy();
}
