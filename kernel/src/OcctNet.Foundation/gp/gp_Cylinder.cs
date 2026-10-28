// Port of: src/FoundationClasses/TKMath/gp/gp_Cylinder.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Cylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An infinite cylindrical surface, defined by its radius and positioned by a coordinate system whose main axis
/// is the cylinder's axis. It is parameterised by <c>P(U, V) = O + R * (cos(U) * XDir + sin(U) * YDir) + V * ZDir</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Cylinder()</c> is OCCT's indefinite cylinder (radius <see cref="Standard.RealLast"/>);
/// <c>default(gp_Cylinder)</c> is not valid.
/// </remarks>
public partial struct gp_Cylinder : IEquatable<gp_Cylinder>
{
    private gp_Ax3 pos;
    private double radius;

    public gp_Cylinder()
    {
        pos = new gp_Ax3();
        radius = RealLast();
    }

    /// <exception cref="Standard_ConstructionError"><paramref name="theRadius"/> is negative.</exception>
    public gp_Cylinder(in gp_Ax3 theA3, double theRadius)
    {
        pos = theA3;
        radius = theRadius;
        Standard_ConstructionError_Raise_if(theRadius < 0.0, "gp_Cylinder() - radius should be positive number");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in gp_Ax1 theA1) => pos.SetAxis(theA1);

    internal void SetLocation(in gp_Pnt theLoc) => pos.SetLocation(theLoc);

    internal void SetPosition(in gp_Ax3 theA3) => pos = theA3;

    internal void SetRadius(double theR)
    {
        Standard_ConstructionError_Raise_if(theR < 0.0, "gp_Cylinder::SetRadius() - radius should be positive number");
        radius = theR;
    }

    /// <summary>Reverses the U parametrisation (the Y direction).</summary>
    internal void UReverse() => pos.YReverse();

    /// <summary>Reverses the V parametrisation (the main direction).</summary>
    internal void VReverse() => pos.ZReverse();

    internal void Mirror(in gp_Pnt P) => pos.Mirror(P);

    internal void Mirror(in gp_Ax1 A1) => pos.Mirror(A1);

    internal void Mirror(in gp_Ax2 A2) => pos.Mirror(A2);

    internal void Rotate(in gp_Ax1 theA1, double theAng) => pos.Rotate(theA1, theAng);

    internal void Scale(in gp_Pnt theP, double theS)
    {
        pos.Scale(theP, theS);
        radius *= theS;
        if (radius < 0)
        {
            radius = -radius;
        }
    }

    internal void Transform(in gp_Trsf theT)
    {
        pos.Transform(theT);
        radius *= theT.ScaleFactor();
        if (radius < 0)
        {
            radius = -radius;
        }
    }

    internal void Translate(in gp_Vec theV) => pos.Translate(theV);

    internal void Translate(in gp_Pnt theP1, in gp_Pnt theP2) => pos.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Cylinder WithAxis(in gp_Ax1 theA1)
    {
        gp_Cylinder aCopy = this;
        aCopy.SetAxis(theA1);
        return aCopy;
    }

    public readonly gp_Cylinder WithLocation(in gp_Pnt theLoc)
    {
        gp_Cylinder aCopy = this;
        aCopy.SetLocation(theLoc);
        return aCopy;
    }

    public readonly gp_Cylinder WithPosition(in gp_Ax3 theA3)
    {
        gp_Cylinder aCopy = this;
        aCopy.SetPosition(theA3);
        return aCopy;
    }

    public readonly gp_Cylinder WithRadius(double theR)
    {
        gp_Cylinder aCopy = this;
        aCopy.SetRadius(theR);
        return aCopy;
    }

    public readonly gp_Cylinder UReversed()
    {
        gp_Cylinder aCopy = this;
        aCopy.UReverse();
        return aCopy;
    }

    public readonly gp_Cylinder VReversed()
    {
        gp_Cylinder aCopy = this;
        aCopy.VReverse();
        return aCopy;
    }

    // Queries

    /// <summary>True when the coordinate system is right-handed.</summary>
    public readonly bool Direct() => pos.Direct();

    /// <summary>The symmetry axis.</summary>
    public readonly gp_Ax1 Axis() => pos.Axis();

    /// <summary>
    /// The coefficients of the implicit equation in the global system:
    /// <c>A1.X^2 + A2.Y^2 + A3.Z^2 + 2.(B1.X.Y + B2.X.Z + B3.Y.Z) + 2.(C1.X + C2.Y + C3.Z) + D = 0</c>.
    /// </summary>
    public readonly void Coefficients(out double A1, out double A2, out double A3,
                                      out double B1, out double B2, out double B3,
                                      out double C1, out double C2, out double C3,
                                      out double D)
    {
        // Dans le repere local du cylindre :
        // X**2 + Y**2 - radius = 0.0
        gp_Trsf T = new();
        T.SetTransformation(pos);
        double T11 = T.Value(1, 1);
        double T12 = T.Value(1, 2);
        double T13 = T.Value(1, 3);
        double T14 = T.Value(1, 4);
        double T21 = T.Value(2, 1);
        double T22 = T.Value(2, 2);
        double T23 = T.Value(2, 3);
        double T24 = T.Value(2, 4);
        A1 = T11 * T11 + T21 * T21;
        A2 = T12 * T12 + T22 * T22;
        A3 = T13 * T13 + T23 * T23;
        B1 = T11 * T12 + T21 * T22;
        B2 = T11 * T13 + T21 * T23;
        B3 = T12 * T13 + T22 * T23;
        C1 = T11 * T14 + T21 * T24;
        C2 = T12 * T14 + T22 * T24;
        C3 = T13 * T14 + T23 * T24;
        D = T14 * T14 + T24 * T24 - radius * radius;
    }

    public readonly gp_Pnt Location() => pos.Location();

    public readonly gp_Ax3 Position() => pos;

    public readonly double Radius() => radius;

    public readonly gp_Ax1 XAxis() => new(pos.Location(), pos.XDirection());

    public readonly gp_Ax1 YAxis() => new(pos.Location(), pos.YDirection());

    public readonly gp_Cylinder Mirrored(in gp_Pnt P)
    {
        gp_Cylinder C = this;
        C.pos.Mirror(P);
        return C;
    }

    public readonly gp_Cylinder Mirrored(in gp_Ax1 A1)
    {
        gp_Cylinder C = this;
        C.pos.Mirror(A1);
        return C;
    }

    public readonly gp_Cylinder Mirrored(in gp_Ax2 A2)
    {
        gp_Cylinder C = this;
        C.pos.Mirror(A2);
        return C;
    }

    public readonly gp_Cylinder Rotated(in gp_Ax1 theA1, double theAng)
    {
        gp_Cylinder aCyl = this;
        aCyl.pos.Rotate(theA1, theAng);
        return aCyl;
    }

    public readonly gp_Cylinder Scaled(in gp_Pnt theP, double theS)
    {
        gp_Cylinder aCyl = this;
        aCyl.pos.Scale(theP, theS);
        aCyl.radius *= theS;
        if (aCyl.radius < 0)
        {
            aCyl.radius = -aCyl.radius;
        }

        return aCyl;
    }

    public readonly gp_Cylinder Transformed(in gp_Trsf theT)
    {
        gp_Cylinder aCyl = this;
        aCyl.pos.Transform(theT);
        aCyl.radius *= theT.ScaleFactor();
        if (aCyl.radius < 0)
        {
            aCyl.radius = -aCyl.radius;
        }

        return aCyl;
    }

    public readonly gp_Cylinder Translated(in gp_Vec theV)
    {
        gp_Cylinder aCyl = this;
        aCyl.pos.Translate(theV);
        return aCyl;
    }

    public readonly gp_Cylinder Translated(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        gp_Cylinder aCyl = this;
        aCyl.pos.Translate(theP1, theP2);
        return aCyl;
    }

    public readonly bool Equals(gp_Cylinder other) => pos.Equals(other.pos) && radius.Equals(other.radius);

    public override readonly bool Equals(object? obj) => obj is gp_Cylinder other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pos, radius);

    public static bool operator ==(in gp_Cylinder theLeft, in gp_Cylinder theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Cylinder theLeft, in gp_Cylinder theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Cylinder({pos}, {radius:R})");
}
