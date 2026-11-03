// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Geometry.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Geometry.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The root of the geometric objects of the Geom2d package (points, vectors, axes, curves in the plane). Only
/// transformations that keep the nature of the geometry apply (<see cref="Transform2D"/>).
/// </summary>
/// <remarks>
/// Geom2d objects are immutable to callers: <see cref="Transformed"/>, <see cref="Mirrored(in Point2D)"/>, ...
/// and the <c>With</c> methods of the derived classes return changed copies. Inside OCCT.NET the in-place
/// mutators remain (internal), and throw <see cref="ImmutableObjectException"/> once the object is frozen.
/// <see cref="Copy"/> always returns an unfrozen object.
/// </remarks>
public abstract class Geom2d_Geometry : Transient
{
    /// <summary>Mirrors this geometry about the point P.</summary>
    internal void Mirror(in Point2D P)
    {
        CheckMutable();
        Transform2D T = new Transform2D();
        T.SetMirror(P);
        Transform(T);
    }

    /// <summary>Mirrors this geometry about the axis A.</summary>
    internal void Mirror(in Axis2D A)
    {
        CheckMutable();
        Transform2D T = new Transform2D();
        T.SetMirror(A);
        Transform(T);
    }

    /// <summary>Rotates this geometry by Ang radians about P.</summary>
    internal void Rotate(in Point2D P, double Ang)
    {
        CheckMutable();
        Transform2D T = new Transform2D();
        T.SetRotation(P, Ang);
        Transform(T);
    }

    /// <summary>Scales this geometry by S about P.</summary>
    internal void Scale(in Point2D P, double S)
    {
        CheckMutable();
        Transform2D T = new Transform2D();
        T.SetScale(P, S);
        Transform(T);
    }

    /// <summary>Translates this geometry by V.</summary>
    internal void Translate(in Vector2D V)
    {
        CheckMutable();
        Transform2D T = new Transform2D();
        T.SetTranslation(V);
        Transform(T);
    }

    /// <summary>Translates this geometry from P1 to P2.</summary>
    internal void Translate(in Point2D P1, in Point2D P2)
    {
        CheckMutable();
        Vector2D V = new Vector2D(P1, P2);
        Translate(V);
    }

    /// <summary>
    /// Transforms this geometry in place. Every concrete class implements it, starting with
    /// <see cref="Transient.CheckMutable"/>; the other transformations go through it.
    /// </summary>
    protected internal abstract void Transform(in Transform2D T);

    /// <summary>A copy mirrored about the point P.</summary>
    public Geom2d_Geometry Mirrored(in Point2D P)
    {
        Geom2d_Geometry G = Copy();
        G.Mirror(P);
        return G;
    }

    /// <summary>A copy mirrored about the axis A.</summary>
    public Geom2d_Geometry Mirrored(in Axis2D A)
    {
        Geom2d_Geometry G = Copy();
        G.Mirror(A);
        return G;
    }

    /// <summary>A copy rotated by Ang radians about P.</summary>
    public Geom2d_Geometry Rotated(in Point2D P, double Ang)
    {
        Geom2d_Geometry G = Copy();
        G.Rotate(P, Ang);
        return G;
    }

    /// <summary>A copy scaled by S about P.</summary>
    public Geom2d_Geometry Scaled(in Point2D P, double S)
    {
        Geom2d_Geometry G = Copy();
        G.Scale(P, S);
        return G;
    }

    /// <summary>A copy transformed by T.</summary>
    public Geom2d_Geometry Transformed(in Transform2D T)
    {
        Geom2d_Geometry G = Copy();
        G.Transform(T);
        return G;
    }

    /// <summary>A copy translated by V.</summary>
    public Geom2d_Geometry Translated(in Vector2D V)
    {
        Geom2d_Geometry G = Copy();
        G.Translate(V);
        return G;
    }

    /// <summary>A copy translated from P1 to P2.</summary>
    public Geom2d_Geometry Translated(in Point2D P1, in Point2D P2)
    {
        Geom2d_Geometry G = Copy();
        G.Translate(P1, P2);
        return G;
    }

    /// <summary>A new object equal to this one, and not frozen even if this one is.</summary>
    public abstract Geom2d_Geometry Copy();
}
