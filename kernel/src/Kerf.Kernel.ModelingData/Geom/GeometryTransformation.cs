// Port of: src/ModelingData/TKG3d/Geom/Geom_Transformation.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Transformation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>
/// A shareable (handle) transformation: translation, rotation, symmetry, scale or a composition of them, the
/// same services as <see cref="Transform3D"/>, used where several objects of a data structure share one
/// transformation.
/// </summary>
/// <remarks>
/// <c>new Geom_Transformation()</c> is the identity. Like the geometry classes it is immutable to callers and
/// freezable: build one with a constructor, a static factory (<see cref="Rotation"/>, <see cref="Translation(in Vector3D)"/>,
/// ...) or a <c>With</c> method.
/// </remarks>
internal class GeometryTransformation : Transient
{
    private Transform3D _gpTrsf;

    /// <summary>The identity transformation.</summary>
    public GeometryTransformation() => _gpTrsf = new Transform3D();

    /// <summary>A transformation equal to T.</summary>
    public GeometryTransformation(in Transform3D t) => _gpTrsf = t;

    /// <summary>The point symmetry about P.</summary>
    public static GeometryTransformation Mirror(in Point3D pnt) => new(Transform3D.Mirror(pnt));

    /// <summary>The axial symmetry about A1.</summary>
    public static GeometryTransformation Mirror(in Axis3D a1) => new(Transform3D.Mirror(a1));

    /// <summary>The plane symmetry about the plane (Location, XDirection, YDirection) of A2.</summary>
    public static GeometryTransformation Mirror(in Frame3D a2) => new(Transform3D.Mirror(a2));

    /// <summary>The rotation by theAng radians about theA1.</summary>
    public static GeometryTransformation Rotation(in Axis3D a1, double ang) => new(Transform3D.Rotation(a1, ang));

    /// <summary>The scale by theScale about thePnt.</summary>
    public static GeometryTransformation Scale(in Point3D pnt, double scale) => new(Transform3D.Scale(pnt, scale));

    /// <summary>The passage from the coordinate system theFromSystem1 to theToSystem2.</summary>
    public static GeometryTransformation Transformation(in CoordinateSystem fromSystem1, in CoordinateSystem toSystem2) =>
        new(Transform3D.Transformation(fromSystem1, toSystem2));

    /// <summary>The passage from the global coordinate system to theToSystem.</summary>
    public static GeometryTransformation Transformation(in CoordinateSystem toSystem) => new(Transform3D.Transformation(toSystem));

    /// <summary>The translation by theVec.</summary>
    public static GeometryTransformation Translation(in Vector3D vec) => new(Transform3D.Translation(vec));

    /// <summary>The translation from P1 to P2.</summary>
    public static GeometryTransformation Translation(in Point3D p1, in Point3D p2) => new(Transform3D.Translation(p1, p2));

    // Mutators (internal: see docs/translation-rules.md, section 8)

    internal void SetMirror(in Point3D pnt)
    {
        CheckMutable();
        _gpTrsf = _gpTrsf.WithMirror(pnt);
    }

    internal void SetMirror(in Axis3D a1)
    {
        CheckMutable();
        _gpTrsf = _gpTrsf.WithMirror(a1);
    }

    internal void SetMirror(in Frame3D a2)
    {
        CheckMutable();
        _gpTrsf = _gpTrsf.WithMirror(a2);
    }

    internal void SetRotation(in Axis3D a1, double ang)
    {
        CheckMutable();
        _gpTrsf = _gpTrsf.WithRotation(a1, ang);
    }

    internal void SetScale(in Point3D pnt, double scale)
    {
        CheckMutable();
        _gpTrsf = _gpTrsf.WithScale(pnt, scale);
    }

    internal void SetTransformation(in CoordinateSystem fromSystem1, in CoordinateSystem toSystem2)
    {
        CheckMutable();
        _gpTrsf = _gpTrsf.WithTransformation(fromSystem1, toSystem2);
    }

    internal void SetTransformation(in CoordinateSystem toSystem)
    {
        CheckMutable();
        _gpTrsf = _gpTrsf.WithTransformation(toSystem);
    }

    internal void SetTranslation(in Vector3D vec)
    {
        CheckMutable();
        _gpTrsf = _gpTrsf.WithTranslation(vec);
    }

    internal void SetTranslation(in Point3D p1, in Point3D p2)
    {
        CheckMutable();
        _gpTrsf = _gpTrsf.WithTranslation(p1, p2);
    }

    internal void SetTrsf(in Transform3D trsf)
    {
        CheckMutable();
        _gpTrsf = trsf;
    }

    internal void Invert()
    {
        CheckMutable();
        _gpTrsf = _gpTrsf.Inverted();
    }

    internal void Multiply(GeometryTransformation other)
    {
        CheckMutable();
        _gpTrsf = _gpTrsf.Multiplied(other.Trsf());
    }

    internal void Power(int n)
    {
        CheckMutable();
        _gpTrsf = _gpTrsf.Powered(n);
    }

    internal void PreMultiply(GeometryTransformation other)
    {
        CheckMutable();
        _gpTrsf = _gpTrsf.PreMultiplied(other.Trsf());
    }

    // Queries

    /// <summary>True if the transformation is indirect (the determinant of its vectorial part is negative).</summary>
    public bool IsNegative => _gpTrsf.IsNegative;

    /// <summary>The nature of the transformation.</summary>
    public TransformForm Form() => _gpTrsf.Form;

    /// <summary>The scale factor.</summary>
    public double ScaleFactor() => _gpTrsf.ScaleFactor;

    /// <summary>The transformation as a <see cref="Transform3D"/>.</summary>
    public Transform3D Trsf() => _gpTrsf;

    /// <summary>The coefficient (theRow, theCol) of the 3 × 4 matrix of the transformation.</summary>
    /// <exception cref="OutOfRangeException">Row is not in 1..3 or Col not in 1..4.</exception>
    public double Value(int row, int col) => _gpTrsf.Value(row, col);

    /// <summary>The inverse transformation.</summary>
    /// <exception cref="ConstructionErrorException">The transformation is singular.</exception>
    public GeometryTransformation Inverted() => new GeometryTransformation(_gpTrsf.Inverted());

    /// <summary>this * Other.</summary>
    public GeometryTransformation Multiplied(GeometryTransformation other) =>
        new GeometryTransformation(_gpTrsf.Multiplied(other.Trsf()));

    /// <summary>this * this * ... (N times); the identity for N = 0, the inverse powered for N &lt; 0.</summary>
    public GeometryTransformation Powered(int n)
    {
        Transform3D t = _gpTrsf;
        t = t.Powered(n);
        return new GeometryTransformation(t);
    }

    /// <summary>Other * this (Kerf: the non-mutating twin of <c>PreMultiply</c>).</summary>
    public GeometryTransformation PreMultiplied(GeometryTransformation other)
    {
        GeometryTransformation copy = Copy();
        copy.PreMultiply(other);
        return copy;
    }

    /// <summary>Applies the transformation to the triplet {X, Y, Z}.</summary>
    public void Transforms(ref double x, ref double y, ref double z) => _gpTrsf.Transforms(ref x, ref y, ref z);

    /// <summary>A new, unfrozen transformation equal to this one.</summary>
    public GeometryTransformation Copy()
    {
        GeometryTransformation t;
        t = new GeometryTransformation(_gpTrsf);
        return t;
    }

    // Non-mutating twins of the Set methods (Kerf)

    public GeometryTransformation WithMirror(in Point3D pnt)
    {
        GeometryTransformation copy = Copy();
        copy.SetMirror(pnt);
        return copy;
    }

    public GeometryTransformation WithMirror(in Axis3D a1)
    {
        GeometryTransformation copy = Copy();
        copy.SetMirror(a1);
        return copy;
    }

    public GeometryTransformation WithMirror(in Frame3D a2)
    {
        GeometryTransformation copy = Copy();
        copy.SetMirror(a2);
        return copy;
    }

    public GeometryTransformation WithRotation(in Axis3D a1, double ang)
    {
        GeometryTransformation copy = Copy();
        copy.SetRotation(a1, ang);
        return copy;
    }

    public GeometryTransformation WithScale(in Point3D pnt, double scale)
    {
        GeometryTransformation copy = Copy();
        copy.SetScale(pnt, scale);
        return copy;
    }

    public GeometryTransformation WithTransformation(in CoordinateSystem fromSystem1, in CoordinateSystem toSystem2)
    {
        GeometryTransformation copy = Copy();
        copy.SetTransformation(fromSystem1, toSystem2);
        return copy;
    }

    public GeometryTransformation WithTransformation(in CoordinateSystem toSystem)
    {
        GeometryTransformation copy = Copy();
        copy.SetTransformation(toSystem);
        return copy;
    }

    public GeometryTransformation WithTranslation(in Vector3D vec)
    {
        GeometryTransformation copy = Copy();
        copy.SetTranslation(vec);
        return copy;
    }

    public GeometryTransformation WithTranslation(in Point3D p1, in Point3D p2)
    {
        GeometryTransformation copy = Copy();
        copy.SetTranslation(p1, p2);
        return copy;
    }

    public GeometryTransformation WithTrsf(in Transform3D trsf)
    {
        GeometryTransformation copy = Copy();
        copy.SetTrsf(trsf);
        return copy;
    }
}
