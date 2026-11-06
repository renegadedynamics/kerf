// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Transformation.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Transformation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>
/// A shareable (handle) transformation of the plane: translation, rotation, symmetry, scale or a composition of
/// them, the same services as <see cref="Transform2D"/>.
/// </summary>
/// <remarks>
/// <c>new Geom2d_Transformation()</c> is the identity. Immutable to callers and freezable, like the geometry
/// classes: build one with a constructor, a static factory or a <c>With</c> method. OCCT's in-place
/// <c>operator*=</c> is left out.
/// </remarks>
internal class Transformation2 : Transient
{
    private Transform2D _gpTrsf2d;

    /// <summary>The identity transformation.</summary>
    public Transformation2() => _gpTrsf2d = new Transform2D();

    /// <summary>A transformation equal to T.</summary>
    public Transformation2(in Transform2D t) => _gpTrsf2d = t;

    /// <summary>A new, unfrozen transformation equal to this one.</summary>
    public Transformation2 Copy()
    {
        Transformation2 t;
        t = new Transformation2(_gpTrsf2d);
        return t;
    }

    /// <summary>The point symmetry about P.</summary>
    public static Transformation2 Mirror(in Point2D p) => new Transformation2().WithMirror(p);

    /// <summary>The axial symmetry about A.</summary>
    public static Transformation2 Mirror(in Axis2D a) => new Transformation2().WithMirror(a);

    /// <summary>The rotation by Ang radians about P.</summary>
    public static Transformation2 Rotation(in Point2D p, double ang) => new Transformation2().WithRotation(p, ang);

    /// <summary>The scale by S about P.</summary>
    public static Transformation2 Scale(in Point2D p, double s) => new Transformation2().WithScale(p, s);

    /// <summary>The passage from the coordinate system FromSystem1 to ToSystem2.</summary>
    public static Transformation2 Transformation(in Axis2D fromSystem1, in Axis2D toSystem2) =>
        new Transformation2().WithTransformation(fromSystem1, toSystem2);

    /// <summary>The passage from the global coordinate system to ToSystem.</summary>
    public static Transformation2 Transformation(in Axis2D toSystem) =>
        new Transformation2().WithTransformation(toSystem);

    /// <summary>The translation by V.</summary>
    public static Transformation2 Translation(in Vector2D v) => new Transformation2().WithTranslation(v);

    /// <summary>The translation from P1 to P2.</summary>
    public static Transformation2 Translation(in Point2D p1, in Point2D p2) =>
        new Transformation2().WithTranslation(p1, p2);

    /// <summary>The inverse transformation.</summary>
    /// <exception cref="ConstructionErrorException">The transformation is singular.</exception>
    public Transformation2 Inverted() => new Transformation2(_gpTrsf2d.Inverted());

    /// <summary>this * Other.</summary>
    public Transformation2 Multiplied(Transformation2 other) =>
        new Transformation2(_gpTrsf2d.Multiplied(other.Trsf2d()));

    /// <summary>theLeft * Other.</summary>
    public static Transformation2 operator *(Transformation2 left, Transformation2 other) =>
        left.Multiplied(other);

    /// <summary>this * this * ... (N times); the identity for N = 0, the inverse powered for N &lt; 0.</summary>
    public Transformation2 Powered(int n)
    {
        Transform2D temp = _gpTrsf2d;
        temp = temp.Powered(n);
        return new Transformation2(temp);
    }

    /// <summary>Other * this (Kerf: the non-mutating twin of <c>PreMultiply</c>).</summary>
    public Transformation2 PreMultiplied(Transformation2 other)
    {
        Transformation2 copy = Copy();
        copy.PreMultiply(other);
        return copy;
    }

    // Mutators (internal: see docs/translation-rules.md, section 8)

    internal void SetMirror(in Point2D p)
    {
        CheckMutable();
        _gpTrsf2d = _gpTrsf2d.WithMirror(p);
    }

    internal void SetMirror(in Axis2D a)
    {
        CheckMutable();
        _gpTrsf2d = _gpTrsf2d.WithMirror(a);
    }

    internal void SetRotation(in Point2D p, double ang)
    {
        CheckMutable();
        _gpTrsf2d = _gpTrsf2d.WithRotation(p, ang);
    }

    internal void SetScale(in Point2D p, double s)
    {
        CheckMutable();
        _gpTrsf2d = _gpTrsf2d.WithScale(p, s);
    }

    internal void SetTransformation(in Axis2D toAxis)
    {
        CheckMutable();
        _gpTrsf2d = _gpTrsf2d.WithTransformation(toAxis);
    }

    internal void SetTransformation(in Axis2D fromAxis1, in Axis2D toAxis2)
    {
        CheckMutable();
        _gpTrsf2d = _gpTrsf2d.WithTransformation(fromAxis1, toAxis2);
    }

    internal void SetTranslation(in Vector2D v)
    {
        CheckMutable();
        _gpTrsf2d = _gpTrsf2d.WithTranslation(v);
    }

    internal void SetTranslation(in Point2D p1, in Point2D p2)
    {
        CheckMutable();
        _gpTrsf2d = _gpTrsf2d.WithTranslation(p1, p2);
    }

    internal void SetTrsf2d(in Transform2D t)
    {
        CheckMutable();
        _gpTrsf2d = t;
    }

    internal void Invert()
    {
        CheckMutable();
        _gpTrsf2d = _gpTrsf2d.Inverted();
    }

    internal void Multiply(Transformation2 other)
    {
        CheckMutable();
        _gpTrsf2d = _gpTrsf2d.Multiplied(other.Trsf2d());
    }

    internal void Power(int n)
    {
        CheckMutable();
        _gpTrsf2d = _gpTrsf2d.Powered(n);
    }

    internal void PreMultiply(Transformation2 other)
    {
        CheckMutable();
        _gpTrsf2d = _gpTrsf2d.PreMultiplied(other.Trsf2d());
    }

    // Queries

    /// <summary>True if the transformation is indirect (the determinant of its vectorial part is negative).</summary>
    public bool IsNegative => _gpTrsf2d.IsNegative;

    /// <summary>The nature of the transformation.</summary>
    public TransformForm Form() => _gpTrsf2d.Form;

    /// <summary>The scale factor.</summary>
    public double ScaleFactor() => _gpTrsf2d.ScaleFactor;

    /// <summary>The transformation as a <see cref="Transform2D"/>.</summary>
    public Transform2D Trsf2d() => _gpTrsf2d;

    /// <summary>The coefficient (Row, Col) of the 2 × 3 matrix of the transformation.</summary>
    /// <exception cref="OutOfRangeException">Row is not in 1..2 or Col not in 1..3.</exception>
    public double Value(int row, int col) => _gpTrsf2d.Value(row, col);

    /// <summary>Applies the transformation to the pair {X, Y}.</summary>
    public void Transforms(ref double x, ref double y) => _gpTrsf2d.Transforms(ref x, ref y);

    // Non-mutating twins of the Set methods (Kerf)

    public Transformation2 WithMirror(in Point2D p)
    {
        Transformation2 copy = Copy();
        copy.SetMirror(p);
        return copy;
    }

    public Transformation2 WithMirror(in Axis2D a)
    {
        Transformation2 copy = Copy();
        copy.SetMirror(a);
        return copy;
    }

    public Transformation2 WithRotation(in Point2D p, double ang)
    {
        Transformation2 copy = Copy();
        copy.SetRotation(p, ang);
        return copy;
    }

    public Transformation2 WithScale(in Point2D p, double s)
    {
        Transformation2 copy = Copy();
        copy.SetScale(p, s);
        return copy;
    }

    public Transformation2 WithTransformation(in Axis2D toAxis)
    {
        Transformation2 copy = Copy();
        copy.SetTransformation(toAxis);
        return copy;
    }

    public Transformation2 WithTransformation(in Axis2D fromAxis1, in Axis2D toAxis2)
    {
        Transformation2 copy = Copy();
        copy.SetTransformation(fromAxis1, toAxis2);
        return copy;
    }

    public Transformation2 WithTranslation(in Vector2D v)
    {
        Transformation2 copy = Copy();
        copy.SetTranslation(v);
        return copy;
    }

    public Transformation2 WithTranslation(in Point2D p1, in Point2D p2)
    {
        Transformation2 copy = Copy();
        copy.SetTranslation(p1, p2);
        return copy;
    }

    public Transformation2 WithTransform(in Transform2D t)
    {
        Transformation2 copy = Copy();
        copy.SetTrsf2d(t);
        return copy;
    }
}
