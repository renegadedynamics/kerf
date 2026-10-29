// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_VectorWithMagnitude.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_VectorWithMagnitude.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A vector with a magnitude, possibly zero, in the plane (a shareable <see cref="gp_Vec2d"/>).</summary>
/// <remarks>
/// OCCT's compound operators (<c>+=</c>, <c>-=</c>, <c>*=</c>, <c>/=</c>) mutate in place, so they are left out;
/// ported code calls <c>Add</c>, <c>Subtract</c>, <c>Multiply</c>, <c>Divide</c>. The binary operators return new
/// vectors, as in OCCT.
/// </remarks>
public class Geom2d_VectorWithMagnitude : Geom2d_Vector
{
    /// <summary>A vector equal to V.</summary>
    public Geom2d_VectorWithMagnitude(in gp_Vec2d V) => gpVec2d = V;

    /// <summary>The vector (X, Y).</summary>
    public Geom2d_VectorWithMagnitude(double X, double Y) => gpVec2d = new gp_Vec2d(X, Y);

    /// <summary>The vector from P1 to P2.</summary>
    public Geom2d_VectorWithMagnitude(in gp_Pnt2d P1, in gp_Pnt2d P2) => gpVec2d = new gp_Vec2d(P1, P2);

    /// <summary>A new, unfrozen vector equal to this one.</summary>
    public override Geom2d_Geometry Copy()
    {
        Geom2d_VectorWithMagnitude V;
        V = new Geom2d_VectorWithMagnitude(gpVec2d);
        return V;
    }

    internal void SetCoord(double X, double Y)
    {
        CheckMutable();
        gpVec2d = new gp_Vec2d(X, Y);
    }

    internal void SetVec2d(in gp_Vec2d V)
    {
        CheckMutable();
        gpVec2d = V;
    }

    internal void SetX(double X)
    {
        CheckMutable();
        gpVec2d.SetX(X);
    }

    internal void SetY(double Y)
    {
        CheckMutable();
        gpVec2d.SetY(Y);
    }

    public override double Magnitude() => gpVec2d.Magnitude();

    public override double SquareMagnitude() => gpVec2d.SquareMagnitude();

    internal void Add(Geom2d_Vector Other)
    {
        CheckMutable();
        gpVec2d.Add(Other.Vec2d());
    }

    /// <summary>this + Other.</summary>
    public Geom2d_VectorWithMagnitude Added(Geom2d_Vector Other)
    {
        gp_Vec2d Temp = Other.Vec2d();
        Temp.Add(gpVec2d);
        return new Geom2d_VectorWithMagnitude(Temp);
    }

    public static Geom2d_VectorWithMagnitude operator +(Geom2d_VectorWithMagnitude theLeft, Geom2d_Vector Other) =>
        theLeft.Added(Other);

    /// <summary>The cross product this ^ Other.</summary>
    public override double Crossed(Geom2d_Vector Other) => gpVec2d.Crossed(Other.Vec2d());

    public static double operator ^(Geom2d_VectorWithMagnitude theLeft, Geom2d_Vector Other) => theLeft.Crossed(Other);

    internal void Divide(double Scalar)
    {
        CheckMutable();
        gpVec2d.Divide(Scalar);
    }

    /// <summary>this / Scalar.</summary>
    public Geom2d_VectorWithMagnitude Divided(double Scalar)
    {
        gp_Vec2d V = gpVec2d;
        V.Divide(Scalar);
        return new Geom2d_VectorWithMagnitude(V);
    }

    public static Geom2d_VectorWithMagnitude operator /(Geom2d_VectorWithMagnitude theLeft, double Scalar) =>
        theLeft.Divided(Scalar);

    /// <summary>this * Scalar.</summary>
    public Geom2d_VectorWithMagnitude Multiplied(double Scalar)
    {
        gp_Vec2d V = gpVec2d;
        V.Multiply(Scalar);
        return new Geom2d_VectorWithMagnitude(V);
    }

    internal void Multiply(double Scalar)
    {
        CheckMutable();
        gpVec2d.Multiply(Scalar);
    }

    /// <exception cref="Standard_ConstructionError">The magnitude is at most <c>gp::Resolution()</c>.</exception>
    internal void Normalize()
    {
        CheckMutable();
        gpVec2d.Normalize();
    }

    /// <summary>The unit vector of this one.</summary>
    /// <exception cref="Standard_ConstructionError">The magnitude is at most <c>gp::Resolution()</c>.</exception>
    public Geom2d_VectorWithMagnitude Normalized()
    {
        gp_Vec2d V = gpVec2d.Normalized();
        return new Geom2d_VectorWithMagnitude(V);
    }

    internal void Subtract(Geom2d_Vector Other)
    {
        CheckMutable();
        gpVec2d.Subtract(Other.Vec2d());
    }

    /// <summary>this - Other.</summary>
    public Geom2d_VectorWithMagnitude Subtracted(Geom2d_Vector Other)
    {
        gp_Vec2d V = gpVec2d;
        V.Subtract(Other.Vec2d());
        return new Geom2d_VectorWithMagnitude(V);
    }

    public static Geom2d_VectorWithMagnitude operator -(Geom2d_VectorWithMagnitude theLeft, Geom2d_Vector Other) =>
        theLeft.Subtracted(Other);

    /// <summary>Applies T to this vector.</summary>
    protected internal override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        gpVec2d.Transform(T);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom2d_VectorWithMagnitude WithCoord(double X, double Y)
    {
        Geom2d_VectorWithMagnitude aCopy = (Geom2d_VectorWithMagnitude)Copy();
        aCopy.SetCoord(X, Y);
        return aCopy;
    }

    public Geom2d_VectorWithMagnitude WithVec2d(in gp_Vec2d V)
    {
        Geom2d_VectorWithMagnitude aCopy = (Geom2d_VectorWithMagnitude)Copy();
        aCopy.SetVec2d(V);
        return aCopy;
    }

    public Geom2d_VectorWithMagnitude WithX(double X)
    {
        Geom2d_VectorWithMagnitude aCopy = (Geom2d_VectorWithMagnitude)Copy();
        aCopy.SetX(X);
        return aCopy;
    }

    public Geom2d_VectorWithMagnitude WithY(double Y)
    {
        Geom2d_VectorWithMagnitude aCopy = (Geom2d_VectorWithMagnitude)Copy();
        aCopy.SetY(Y);
        return aCopy;
    }
}
