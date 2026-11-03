// Port of: src/ModelingData/TKG3d/Geom/Geom_Geometry.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Geometry.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// The root of the geometric objects of the Geom package (points, vectors, axes, curves, surfaces). Only
/// transformations that keep the nature of the geometry apply: translations, rotations, symmetries, scales and
/// their compositions (<see cref="Transform3D"/>).
/// </summary>
/// <remarks>
/// Geom objects are immutable to callers: <see cref="Transformed"/>, <see cref="Mirrored(in Point3D)"/>,
/// <see cref="Rotated"/>, ... and the <c>With</c> methods of the derived classes return changed copies. Inside
/// OCCT.NET the in-place mutators remain (internal), and throw <see cref="ImmutableObjectException"/> once the
/// object is frozen (shared, for instance by topology). <see cref="Copy"/> always returns an unfrozen object.
/// </remarks>
public abstract class Geometry3D : Transient
{
    /// <summary>Mirrors this geometry about the point P.</summary>
    internal void Mirror(in Point3D p)
    {
        CheckMutable();
        Transform3D t = new Transform3D();
        t.SetMirror(p);
        Transform(t);
    }

    /// <summary>Mirrors this geometry about the axis A1.</summary>
    internal void Mirror(in Axis3D a1)
    {
        CheckMutable();
        Transform3D t = new Transform3D();
        t.SetMirror(a1);
        Transform(t);
    }

    /// <summary>Mirrors this geometry about the plane (Location, XDirection, YDirection) of A2.</summary>
    internal void Mirror(in Frame3D a2)
    {
        CheckMutable();
        Transform3D t = new Transform3D();
        t.SetMirror(a2);
        Transform(t);
    }

    /// <summary>Rotates this geometry by Ang radians about the axis A1.</summary>
    internal void Rotate(in Axis3D a1, double ang)
    {
        CheckMutable();
        Transform3D t = new Transform3D();
        t.SetRotation(a1, ang);
        Transform(t);
    }

    /// <summary>Scales this geometry by S about the point P.</summary>
    internal void Scale(in Point3D p, double s)
    {
        CheckMutable();
        Transform3D t = new Transform3D();
        t.SetScale(p, s);
        Transform(t);
    }

    /// <summary>Translates this geometry by V.</summary>
    internal void Translate(in Vector3D v)
    {
        CheckMutable();
        Transform3D t = new Transform3D();
        t.SetTranslation(v);
        Transform(t);
    }

    /// <summary>Translates this geometry from P1 to P2.</summary>
    internal void Translate(in Point3D p1, in Point3D p2)
    {
        CheckMutable();
        Vector3D v = new Vector3D(p1, p2);
        Translate(v);
    }

    /// <summary>
    /// Transforms this geometry in place. Every concrete class implements it, starting with
    /// <see cref="Transient.CheckMutable"/>; the other transformations go through it.
    /// </summary>
    protected internal abstract void Transform(in Transform3D t);

    /// <summary>A copy mirrored about the point P.</summary>
    public Geometry3D Mirrored(in Point3D p)
    {
        Geometry3D g = Copy();
        g.Mirror(p);
        return g;
    }

    /// <summary>A copy mirrored about the axis A1.</summary>
    public Geometry3D Mirrored(in Axis3D a1)
    {
        Geometry3D g = Copy();
        g.Mirror(a1);
        return g;
    }

    /// <summary>A copy mirrored about the plane of A2.</summary>
    public Geometry3D Mirrored(in Frame3D a2)
    {
        Geometry3D g = Copy();
        g.Mirror(a2);
        return g;
    }

    /// <summary>A copy rotated by Ang radians about A1.</summary>
    public Geometry3D Rotated(in Axis3D a1, double ang)
    {
        Geometry3D g = Copy();
        g.Rotate(a1, ang);
        return g;
    }

    /// <summary>A copy scaled by S about P.</summary>
    public Geometry3D Scaled(in Point3D p, double s)
    {
        Geometry3D g = Copy();
        g.Scale(p, s);
        return g;
    }

    /// <summary>A copy transformed by T.</summary>
    public Geometry3D Transformed(in Transform3D t)
    {
        Geometry3D g = Copy();
        g.Transform(t);
        return g;
    }

    /// <summary>A copy translated by V.</summary>
    public Geometry3D Translated(in Vector3D v)
    {
        Geometry3D g = Copy();
        g.Translate(v);
        return g;
    }

    /// <summary>A copy translated from P1 to P2.</summary>
    public Geometry3D Translated(in Point3D p1, in Point3D p2)
    {
        Geometry3D g = Copy();
        g.Translate(p1, p2);
        return g;
    }

    /// <summary>A new object equal to this one, and not frozen even if this one is.</summary>
    /// <remarks>
    /// Pure virtual in OCCT, which also defines a body that throws <see cref="ConstructionErrorException"/>;
    /// no class calls that body, so the port leaves it out.
    /// </remarks>
    public abstract Geometry3D Copy();
}
