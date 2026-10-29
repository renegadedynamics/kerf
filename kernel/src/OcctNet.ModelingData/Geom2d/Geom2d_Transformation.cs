// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Transformation.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Transformation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A shareable (handle) transformation of the plane: translation, rotation, symmetry, scale or a composition of
/// them, the same services as <see cref="gp_Trsf2d"/>.
/// </summary>
/// <remarks>
/// <c>new Geom2d_Transformation()</c> is the identity. Immutable to callers and freezable, like the geometry
/// classes: build one with a constructor, a static factory or a <c>With</c> method. OCCT's in-place
/// <c>operator*=</c> is left out.
/// </remarks>
public class Geom2d_Transformation : Standard_Transient
{
    private gp_Trsf2d gpTrsf2d;

    /// <summary>The identity transformation.</summary>
    public Geom2d_Transformation() => gpTrsf2d = new gp_Trsf2d();

    /// <summary>A transformation equal to T.</summary>
    public Geom2d_Transformation(in gp_Trsf2d T) => gpTrsf2d = T;

    /// <summary>A new, unfrozen transformation equal to this one.</summary>
    public Geom2d_Transformation Copy()
    {
        Geom2d_Transformation T;
        T = new Geom2d_Transformation(gpTrsf2d);
        return T;
    }

    /// <summary>The point symmetry about P.</summary>
    public static Geom2d_Transformation Mirror(in gp_Pnt2d P) => new Geom2d_Transformation().WithMirror(P);

    /// <summary>The axial symmetry about A.</summary>
    public static Geom2d_Transformation Mirror(in gp_Ax2d A) => new Geom2d_Transformation().WithMirror(A);

    /// <summary>The rotation by Ang radians about P.</summary>
    public static Geom2d_Transformation Rotation(in gp_Pnt2d P, double Ang) => new Geom2d_Transformation().WithRotation(P, Ang);

    /// <summary>The scale by S about P.</summary>
    public static Geom2d_Transformation Scale(in gp_Pnt2d P, double S) => new Geom2d_Transformation().WithScale(P, S);

    /// <summary>The passage from the coordinate system FromSystem1 to ToSystem2.</summary>
    public static Geom2d_Transformation Transformation(in gp_Ax2d FromSystem1, in gp_Ax2d ToSystem2) =>
        new Geom2d_Transformation().WithTransformation(FromSystem1, ToSystem2);

    /// <summary>The passage from the global coordinate system to ToSystem.</summary>
    public static Geom2d_Transformation Transformation(in gp_Ax2d ToSystem) =>
        new Geom2d_Transformation().WithTransformation(ToSystem);

    /// <summary>The translation by V.</summary>
    public static Geom2d_Transformation Translation(in gp_Vec2d V) => new Geom2d_Transformation().WithTranslation(V);

    /// <summary>The translation from P1 to P2.</summary>
    public static Geom2d_Transformation Translation(in gp_Pnt2d P1, in gp_Pnt2d P2) =>
        new Geom2d_Transformation().WithTranslation(P1, P2);

    /// <summary>The inverse transformation.</summary>
    /// <exception cref="Standard_ConstructionError">The transformation is singular.</exception>
    public Geom2d_Transformation Inverted() => new Geom2d_Transformation(gpTrsf2d.Inverted());

    /// <summary>this * Other.</summary>
    public Geom2d_Transformation Multiplied(Geom2d_Transformation Other) =>
        new Geom2d_Transformation(gpTrsf2d.Multiplied(Other.Trsf2d()));

    /// <summary>theLeft * Other.</summary>
    public static Geom2d_Transformation operator *(Geom2d_Transformation theLeft, Geom2d_Transformation Other) =>
        theLeft.Multiplied(Other);

    /// <summary>this * this * ... (N times); the identity for N = 0, the inverse powered for N &lt; 0.</summary>
    public Geom2d_Transformation Powered(int N)
    {
        gp_Trsf2d Temp = gpTrsf2d;
        Temp.Power(N);
        return new Geom2d_Transformation(Temp);
    }

    /// <summary>Other * this (OCCT.NET: the non-mutating twin of <c>PreMultiply</c>).</summary>
    public Geom2d_Transformation PreMultiplied(Geom2d_Transformation Other)
    {
        Geom2d_Transformation aCopy = Copy();
        aCopy.PreMultiply(Other);
        return aCopy;
    }

    // Mutators (internal: see docs/translation-rules.md, section 8)

    internal void SetMirror(in gp_Pnt2d P)
    {
        CheckMutable();
        gpTrsf2d.SetMirror(P);
    }

    internal void SetMirror(in gp_Ax2d A)
    {
        CheckMutable();
        gpTrsf2d.SetMirror(A);
    }

    internal void SetRotation(in gp_Pnt2d P, double Ang)
    {
        CheckMutable();
        gpTrsf2d.SetRotation(P, Ang);
    }

    internal void SetScale(in gp_Pnt2d P, double S)
    {
        CheckMutable();
        gpTrsf2d.SetScale(P, S);
    }

    internal void SetTransformation(in gp_Ax2d ToAxis)
    {
        CheckMutable();
        gpTrsf2d.SetTransformation(ToAxis);
    }

    internal void SetTransformation(in gp_Ax2d FromAxis1, in gp_Ax2d ToAxis2)
    {
        CheckMutable();
        gpTrsf2d.SetTransformation(FromAxis1, ToAxis2);
    }

    internal void SetTranslation(in gp_Vec2d V)
    {
        CheckMutable();
        gpTrsf2d.SetTranslation(V);
    }

    internal void SetTranslation(in gp_Pnt2d P1, in gp_Pnt2d P2)
    {
        CheckMutable();
        gpTrsf2d.SetTranslation(P1, P2);
    }

    internal void SetTrsf2d(in gp_Trsf2d T)
    {
        CheckMutable();
        gpTrsf2d = T;
    }

    internal void Invert()
    {
        CheckMutable();
        gpTrsf2d.Invert();
    }

    internal void Multiply(Geom2d_Transformation Other)
    {
        CheckMutable();
        gpTrsf2d.Multiply(Other.Trsf2d());
    }

    internal void Power(int N)
    {
        CheckMutable();
        gpTrsf2d.Power(N);
    }

    internal void PreMultiply(Geom2d_Transformation Other)
    {
        CheckMutable();
        gpTrsf2d.PreMultiply(Other.Trsf2d());
    }

    // Queries

    /// <summary>True if the transformation is indirect (the determinant of its vectorial part is negative).</summary>
    public bool IsNegative() => gpTrsf2d.IsNegative();

    /// <summary>The nature of the transformation.</summary>
    public gp_TrsfForm Form() => gpTrsf2d.Form();

    /// <summary>The scale factor.</summary>
    public double ScaleFactor() => gpTrsf2d.ScaleFactor();

    /// <summary>The transformation as a <see cref="gp_Trsf2d"/>.</summary>
    public gp_Trsf2d Trsf2d() => gpTrsf2d;

    /// <summary>The coefficient (Row, Col) of the 2 × 3 matrix of the transformation.</summary>
    /// <exception cref="Standard_OutOfRange">Row is not in 1..2 or Col not in 1..3.</exception>
    public double Value(int Row, int Col) => gpTrsf2d.Value(Row, Col);

    /// <summary>Applies the transformation to the pair {X, Y}.</summary>
    public void Transforms(ref double X, ref double Y) => gpTrsf2d.Transforms(ref X, ref Y);

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom2d_Transformation WithMirror(in gp_Pnt2d P)
    {
        Geom2d_Transformation aCopy = Copy();
        aCopy.SetMirror(P);
        return aCopy;
    }

    public Geom2d_Transformation WithMirror(in gp_Ax2d A)
    {
        Geom2d_Transformation aCopy = Copy();
        aCopy.SetMirror(A);
        return aCopy;
    }

    public Geom2d_Transformation WithRotation(in gp_Pnt2d P, double Ang)
    {
        Geom2d_Transformation aCopy = Copy();
        aCopy.SetRotation(P, Ang);
        return aCopy;
    }

    public Geom2d_Transformation WithScale(in gp_Pnt2d P, double S)
    {
        Geom2d_Transformation aCopy = Copy();
        aCopy.SetScale(P, S);
        return aCopy;
    }

    public Geom2d_Transformation WithTransformation(in gp_Ax2d ToAxis)
    {
        Geom2d_Transformation aCopy = Copy();
        aCopy.SetTransformation(ToAxis);
        return aCopy;
    }

    public Geom2d_Transformation WithTransformation(in gp_Ax2d FromAxis1, in gp_Ax2d ToAxis2)
    {
        Geom2d_Transformation aCopy = Copy();
        aCopy.SetTransformation(FromAxis1, ToAxis2);
        return aCopy;
    }

    public Geom2d_Transformation WithTranslation(in gp_Vec2d V)
    {
        Geom2d_Transformation aCopy = Copy();
        aCopy.SetTranslation(V);
        return aCopy;
    }

    public Geom2d_Transformation WithTranslation(in gp_Pnt2d P1, in gp_Pnt2d P2)
    {
        Geom2d_Transformation aCopy = Copy();
        aCopy.SetTranslation(P1, P2);
        return aCopy;
    }

    public Geom2d_Transformation WithTrsf2d(in gp_Trsf2d T)
    {
        Geom2d_Transformation aCopy = Copy();
        aCopy.SetTrsf2d(T);
        return aCopy;
    }
}
