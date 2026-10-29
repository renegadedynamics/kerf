// Port of: src/ModelingData/TKG3d/Geom/Geom_Transformation.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Transformation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A shareable (handle) transformation: translation, rotation, symmetry, scale or a composition of them, the
/// same services as <see cref="gp_Trsf"/>, used where several objects of a data structure share one
/// transformation.
/// </summary>
/// <remarks>
/// <c>new Geom_Transformation()</c> is the identity. Like the geometry classes it is immutable to callers and
/// freezable: build one with a constructor, a static factory (<see cref="Rotation"/>, <see cref="Translation(in gp_Vec)"/>,
/// ...) or a <c>With</c> method.
/// </remarks>
public class Geom_Transformation : Standard_Transient
{
    private gp_Trsf gpTrsf;

    /// <summary>The identity transformation.</summary>
    public Geom_Transformation() => gpTrsf = new gp_Trsf();

    /// <summary>A transformation equal to T.</summary>
    public Geom_Transformation(in gp_Trsf T) => gpTrsf = T;

    /// <summary>The point symmetry about P.</summary>
    public static Geom_Transformation Mirror(in gp_Pnt thePnt) => new(gp_Trsf.Mirror(thePnt));

    /// <summary>The axial symmetry about A1.</summary>
    public static Geom_Transformation Mirror(in gp_Ax1 theA1) => new(gp_Trsf.Mirror(theA1));

    /// <summary>The plane symmetry about the plane (Location, XDirection, YDirection) of A2.</summary>
    public static Geom_Transformation Mirror(in gp_Ax2 theA2) => new(gp_Trsf.Mirror(theA2));

    /// <summary>The rotation by theAng radians about theA1.</summary>
    public static Geom_Transformation Rotation(in gp_Ax1 theA1, double theAng) => new(gp_Trsf.Rotation(theA1, theAng));

    /// <summary>The scale by theScale about thePnt.</summary>
    public static Geom_Transformation Scale(in gp_Pnt thePnt, double theScale) => new(gp_Trsf.Scale(thePnt, theScale));

    /// <summary>The passage from the coordinate system theFromSystem1 to theToSystem2.</summary>
    public static Geom_Transformation Transformation(in gp_Ax3 theFromSystem1, in gp_Ax3 theToSystem2) =>
        new(gp_Trsf.Transformation(theFromSystem1, theToSystem2));

    /// <summary>The passage from the global coordinate system to theToSystem.</summary>
    public static Geom_Transformation Transformation(in gp_Ax3 theToSystem) => new(gp_Trsf.Transformation(theToSystem));

    /// <summary>The translation by theVec.</summary>
    public static Geom_Transformation Translation(in gp_Vec theVec) => new(gp_Trsf.Translation(theVec));

    /// <summary>The translation from P1 to P2.</summary>
    public static Geom_Transformation Translation(in gp_Pnt P1, in gp_Pnt P2) => new(gp_Trsf.Translation(P1, P2));

    // Mutators (internal: see docs/translation-rules.md, section 8)

    internal void SetMirror(in gp_Pnt thePnt)
    {
        CheckMutable();
        gpTrsf.SetMirror(thePnt);
    }

    internal void SetMirror(in gp_Ax1 theA1)
    {
        CheckMutable();
        gpTrsf.SetMirror(theA1);
    }

    internal void SetMirror(in gp_Ax2 theA2)
    {
        CheckMutable();
        gpTrsf.SetMirror(theA2);
    }

    internal void SetRotation(in gp_Ax1 theA1, double theAng)
    {
        CheckMutable();
        gpTrsf.SetRotation(theA1, theAng);
    }

    internal void SetScale(in gp_Pnt thePnt, double theScale)
    {
        CheckMutable();
        gpTrsf.SetScale(thePnt, theScale);
    }

    internal void SetTransformation(in gp_Ax3 theFromSystem1, in gp_Ax3 theToSystem2)
    {
        CheckMutable();
        gpTrsf.SetTransformation(theFromSystem1, theToSystem2);
    }

    internal void SetTransformation(in gp_Ax3 theToSystem)
    {
        CheckMutable();
        gpTrsf.SetTransformation(theToSystem);
    }

    internal void SetTranslation(in gp_Vec theVec)
    {
        CheckMutable();
        gpTrsf.SetTranslation(theVec);
    }

    internal void SetTranslation(in gp_Pnt P1, in gp_Pnt P2)
    {
        CheckMutable();
        gpTrsf.SetTranslation(P1, P2);
    }

    internal void SetTrsf(in gp_Trsf theTrsf)
    {
        CheckMutable();
        gpTrsf = theTrsf;
    }

    internal void Invert()
    {
        CheckMutable();
        gpTrsf.Invert();
    }

    internal void Multiply(Geom_Transformation theOther)
    {
        CheckMutable();
        gpTrsf.Multiply(theOther.Trsf());
    }

    internal void Power(int N)
    {
        CheckMutable();
        gpTrsf.Power(N);
    }

    internal void PreMultiply(Geom_Transformation Other)
    {
        CheckMutable();
        gpTrsf.PreMultiply(Other.Trsf());
    }

    // Queries

    /// <summary>True if the transformation is indirect (the determinant of its vectorial part is negative).</summary>
    public bool IsNegative() => gpTrsf.IsNegative();

    /// <summary>The nature of the transformation.</summary>
    public gp_TrsfForm Form() => gpTrsf.Form();

    /// <summary>The scale factor.</summary>
    public double ScaleFactor() => gpTrsf.ScaleFactor();

    /// <summary>The transformation as a <see cref="gp_Trsf"/>.</summary>
    public gp_Trsf Trsf() => gpTrsf;

    /// <summary>The coefficient (theRow, theCol) of the 3 × 4 matrix of the transformation.</summary>
    /// <exception cref="Standard_OutOfRange">Row is not in 1..3 or Col not in 1..4.</exception>
    public double Value(int theRow, int theCol) => gpTrsf.Value(theRow, theCol);

    /// <summary>The inverse transformation.</summary>
    /// <exception cref="Standard_ConstructionError">The transformation is singular.</exception>
    public Geom_Transformation Inverted() => new Geom_Transformation(gpTrsf.Inverted());

    /// <summary>this * Other.</summary>
    public Geom_Transformation Multiplied(Geom_Transformation Other) =>
        new Geom_Transformation(gpTrsf.Multiplied(Other.Trsf()));

    /// <summary>this * this * ... (N times); the identity for N = 0, the inverse powered for N &lt; 0.</summary>
    public Geom_Transformation Powered(int N)
    {
        gp_Trsf T = gpTrsf;
        T.Power(N);
        return new Geom_Transformation(T);
    }

    /// <summary>Other * this (OCCT.NET: the non-mutating twin of <c>PreMultiply</c>).</summary>
    public Geom_Transformation PreMultiplied(Geom_Transformation Other)
    {
        Geom_Transformation aCopy = Copy();
        aCopy.PreMultiply(Other);
        return aCopy;
    }

    /// <summary>Applies the transformation to the triplet {X, Y, Z}.</summary>
    public void Transforms(ref double theX, ref double theY, ref double theZ) => gpTrsf.Transforms(ref theX, ref theY, ref theZ);

    /// <summary>A new, unfrozen transformation equal to this one.</summary>
    public Geom_Transformation Copy()
    {
        Geom_Transformation T;
        T = new Geom_Transformation(gpTrsf);
        return T;
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom_Transformation WithMirror(in gp_Pnt thePnt)
    {
        Geom_Transformation aCopy = Copy();
        aCopy.SetMirror(thePnt);
        return aCopy;
    }

    public Geom_Transformation WithMirror(in gp_Ax1 theA1)
    {
        Geom_Transformation aCopy = Copy();
        aCopy.SetMirror(theA1);
        return aCopy;
    }

    public Geom_Transformation WithMirror(in gp_Ax2 theA2)
    {
        Geom_Transformation aCopy = Copy();
        aCopy.SetMirror(theA2);
        return aCopy;
    }

    public Geom_Transformation WithRotation(in gp_Ax1 theA1, double theAng)
    {
        Geom_Transformation aCopy = Copy();
        aCopy.SetRotation(theA1, theAng);
        return aCopy;
    }

    public Geom_Transformation WithScale(in gp_Pnt thePnt, double theScale)
    {
        Geom_Transformation aCopy = Copy();
        aCopy.SetScale(thePnt, theScale);
        return aCopy;
    }

    public Geom_Transformation WithTransformation(in gp_Ax3 theFromSystem1, in gp_Ax3 theToSystem2)
    {
        Geom_Transformation aCopy = Copy();
        aCopy.SetTransformation(theFromSystem1, theToSystem2);
        return aCopy;
    }

    public Geom_Transformation WithTransformation(in gp_Ax3 theToSystem)
    {
        Geom_Transformation aCopy = Copy();
        aCopy.SetTransformation(theToSystem);
        return aCopy;
    }

    public Geom_Transformation WithTranslation(in gp_Vec theVec)
    {
        Geom_Transformation aCopy = Copy();
        aCopy.SetTranslation(theVec);
        return aCopy;
    }

    public Geom_Transformation WithTranslation(in gp_Pnt P1, in gp_Pnt P2)
    {
        Geom_Transformation aCopy = Copy();
        aCopy.SetTranslation(P1, P2);
        return aCopy;
    }

    public Geom_Transformation WithTrsf(in gp_Trsf theTrsf)
    {
        Geom_Transformation aCopy = Copy();
        aCopy.SetTrsf(theTrsf);
        return aCopy;
    }
}
