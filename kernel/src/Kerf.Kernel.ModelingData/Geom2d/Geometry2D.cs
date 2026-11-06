// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Geometry.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Geometry.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Geometry;

/// <summary>
/// The root of the geometric objects of the Geom2d package (points, vectors, axes, curves in the plane). Only
/// transformations that keep the nature of the geometry apply (<see cref="Transform2D"/>).
/// </summary>
/// <remarks>
/// Geom2d objects are immutable to callers: <see cref="Transformed"/>, <see cref="Mirrored(in Point2D)"/>, ...
/// and the <c>With</c> methods of the derived classes return changed copies. Inside Kerf the in-place
/// mutators remain (internal), and throw <see cref="ImmutableObjectException"/> once the object is frozen.
/// <see cref="Copy"/> always returns an unfrozen object.
/// </remarks>
public abstract class Geometry2D : Transient
{
    /// <summary>Only the kernel derives from it.</summary>
    private protected Geometry2D()
    {
    }

    /// <summary>Mirrors this geometry about the point P.</summary>
    internal void Mirror(in Point2D p)
    {
        CheckMutable();
        Transform2D t = new Transform2D().WithMirror(p);
        Transform(t);
    }

    /// <summary>Mirrors this geometry about the axis A.</summary>
    internal void Mirror(in Axis2D a)
    {
        CheckMutable();
        Transform2D t = new Transform2D().WithMirror(a);
        Transform(t);
    }

    /// <summary>Rotates this geometry by Ang radians about P.</summary>
    internal void Rotate(in Point2D p, double ang)
    {
        CheckMutable();
        Transform2D t = new Transform2D().WithRotation(p, ang);
        Transform(t);
    }

    /// <summary>Scales this geometry by S about P.</summary>
    internal void Scale(in Point2D p, double s)
    {
        CheckMutable();
        Transform2D t = new Transform2D().WithScale(p, s);
        Transform(t);
    }

    /// <summary>Translates this geometry by V.</summary>
    internal void Translate(in Vector2D v)
    {
        CheckMutable();
        Transform2D t = new Transform2D().WithTranslation(v);
        Transform(t);
    }

    /// <summary>Translates this geometry from P1 to P2.</summary>
    internal void Translate(in Point2D p1, in Point2D p2)
    {
        CheckMutable();
        Vector2D v = new Vector2D(p1, p2);
        Translate(v);
    }

    /// <summary>
    /// Transforms this geometry in place. Every concrete class implements it, starting with
    /// <see cref="Transient.CheckMutable"/>; the other transformations go through it.
    /// </summary>
    protected internal abstract void Transform(in Transform2D transform);

    /// <summary>A copy mirrored about the point P.</summary>
    public Geometry2D Mirrored(in Point2D point)
    {
        Geometry2D g = Copy();
        g.Mirror(point);
        return g;
    }

    /// <summary>A copy mirrored about the axis A.</summary>
    public Geometry2D Mirrored(in Axis2D axis)
    {
        Geometry2D g = Copy();
        g.Mirror(axis);
        return g;
    }

    /// <summary>A copy rotated by Ang radians about P.</summary>
    public Geometry2D Rotated(in Point2D point, double angle)
    {
        Geometry2D g = Copy();
        g.Rotate(point, angle);
        return g;
    }

    /// <summary>A copy scaled by S about P.</summary>
    public Geometry2D Scaled(in Point2D point, double factor)
    {
        Geometry2D g = Copy();
        g.Scale(point, factor);
        return g;
    }

    /// <summary>A copy transformed by T.</summary>
    public Geometry2D Transformed(in Transform2D transform)
    {
        Geometry2D g = Copy();
        g.Transform(transform);
        return g;
    }

    /// <summary>A copy translated by V.</summary>
    public Geometry2D Translated(in Vector2D vector)
    {
        Geometry2D g = Copy();
        g.Translate(vector);
        return g;
    }

    /// <summary>A copy translated from P1 to P2.</summary>
    public Geometry2D Translated(in Point2D point1, in Point2D point2)
    {
        Geometry2D g = Copy();
        g.Translate(point1, point2);
        return g;
    }

    /// <summary>A new object equal to this one, and not frozen even if this one is.</summary>
    public abstract Geometry2D Copy();
}
