// Port of: src/ModelingData/TKG3d/Geom/Geom_VectorWithMagnitude.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_VectorWithMagnitude.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A vector with a magnitude, possibly zero, in 3D space (a shareable <see cref="gp_Vec"/>).</summary>
public class Geom_VectorWithMagnitude : Geom_Vector
{
    /// <summary>A vector equal to V.</summary>
    public Geom_VectorWithMagnitude(in gp_Vec V) => gpVec = V;

    /// <summary>The vector (X, Y, Z).</summary>
    public Geom_VectorWithMagnitude(double X, double Y, double Z) => gpVec = new gp_Vec(X, Y, Z);

    /// <summary>The vector from P1 to P2.</summary>
    public Geom_VectorWithMagnitude(in gp_Pnt P1, in gp_Pnt P2) => gpVec = new gp_Vec(P1, P2);

    /// <summary>A new, unfrozen vector equal to this one.</summary>
    public override Geom_Geometry Copy()
    {
        Geom_VectorWithMagnitude V;
        V = new Geom_VectorWithMagnitude(gpVec);
        return V;
    }

    internal void SetCoord(double X, double Y, double Z)
    {
        CheckMutable();
        gpVec = new gp_Vec(X, Y, Z);
    }

    internal void SetVec(in gp_Vec V)
    {
        CheckMutable();
        gpVec = V;
    }

    internal void SetX(double X)
    {
        CheckMutable();
        gpVec.SetX(X);
    }

    internal void SetY(double Y)
    {
        CheckMutable();
        gpVec.SetY(Y);
    }

    internal void SetZ(double Z)
    {
        CheckMutable();
        gpVec.SetZ(Z);
    }

    public override double Magnitude() => gpVec.Magnitude();

    public override double SquareMagnitude() => gpVec.SquareMagnitude();

    internal void Add(Geom_Vector Other)
    {
        CheckMutable();
        gpVec.Add(Other.Vec());
    }

    /// <summary>this + Other.</summary>
    public Geom_VectorWithMagnitude Added(Geom_Vector Other)
    {
        gp_Vec V1 = gpVec;
        V1.Add(Other.Vec());
        return new Geom_VectorWithMagnitude(V1);
    }

    protected internal override void Cross(Geom_Vector Other)
    {
        CheckMutable();
        gpVec.Cross(Other.Vec());
    }

    /// <summary>this ^ Other.</summary>
    public override Geom_Vector Crossed(Geom_Vector Other)
    {
        gp_Vec V = gpVec;
        V.Cross(Other.Vec());
        return new Geom_VectorWithMagnitude(V);
    }

    protected internal override void CrossCross(Geom_Vector V1, Geom_Vector V2)
    {
        CheckMutable();
        gpVec.CrossCross(V1.Vec(), V2.Vec());
    }

    /// <summary>this ^ (V1 ^ V2).</summary>
    public override Geom_Vector CrossCrossed(Geom_Vector V1, Geom_Vector V2)
    {
        gp_Vec V = gpVec;
        V.CrossCross(V1.Vec(), V2.Vec());
        return new Geom_VectorWithMagnitude(V);
    }

    internal void Divide(double Scalar)
    {
        CheckMutable();
        gpVec.Divide(Scalar);
    }

    /// <summary>this / Scalar.</summary>
    public Geom_VectorWithMagnitude Divided(double Scalar)
    {
        gp_Vec V = gpVec;
        V.Divide(Scalar);
        return new Geom_VectorWithMagnitude(V);
    }

    /// <summary>this * Scalar.</summary>
    public Geom_VectorWithMagnitude Multiplied(double Scalar)
    {
        gp_Vec V = gpVec;
        V.Multiply(Scalar);
        return new Geom_VectorWithMagnitude(V);
    }

    internal void Multiply(double Scalar)
    {
        CheckMutable();
        gpVec.Multiply(Scalar);
    }

    /// <exception cref="Standard_ConstructionError">The magnitude is at most <c>gp::Resolution()</c>.</exception>
    internal void Normalize()
    {
        CheckMutable();
        gpVec.Normalize();
    }

    /// <summary>The unit vector of this one.</summary>
    /// <exception cref="Standard_ConstructionError">The magnitude is at most <c>gp::Resolution()</c>.</exception>
    public Geom_VectorWithMagnitude Normalized()
    {
        gp_Vec V = gpVec;
        V.Normalize();
        return new Geom_VectorWithMagnitude(V);
    }

    internal void Subtract(Geom_Vector Other)
    {
        CheckMutable();
        gpVec.Subtract(Other.Vec());
    }

    /// <summary>this - Other.</summary>
    public Geom_VectorWithMagnitude Subtracted(Geom_Vector Other)
    {
        gp_Vec V = gpVec;
        V.Subtract(Other.Vec());
        return new Geom_VectorWithMagnitude(V);
    }

    /// <summary>Applies T to this vector.</summary>
    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        gpVec.Transform(T);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom_VectorWithMagnitude WithCoord(double X, double Y, double Z)
    {
        Geom_VectorWithMagnitude aCopy = (Geom_VectorWithMagnitude)Copy();
        aCopy.SetCoord(X, Y, Z);
        return aCopy;
    }

    public Geom_VectorWithMagnitude WithVec(in gp_Vec V)
    {
        Geom_VectorWithMagnitude aCopy = (Geom_VectorWithMagnitude)Copy();
        aCopy.SetVec(V);
        return aCopy;
    }

    public Geom_VectorWithMagnitude WithX(double X)
    {
        Geom_VectorWithMagnitude aCopy = (Geom_VectorWithMagnitude)Copy();
        aCopy.SetX(X);
        return aCopy;
    }

    public Geom_VectorWithMagnitude WithY(double Y)
    {
        Geom_VectorWithMagnitude aCopy = (Geom_VectorWithMagnitude)Copy();
        aCopy.SetY(Y);
        return aCopy;
    }

    public Geom_VectorWithMagnitude WithZ(double Z)
    {
        Geom_VectorWithMagnitude aCopy = (Geom_VectorWithMagnitude)Copy();
        aCopy.SetZ(Z);
        return aCopy;
    }
}
