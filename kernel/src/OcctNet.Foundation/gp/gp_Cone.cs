// Port of: src/FoundationClasses/TKMath/gp/gp_Cone.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Cone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An infinite conical surface, defined by its semi-angle and its reference radius (the radius in the plane of
/// its coordinate system), and positioned by a coordinate system whose main axis is the cone's axis. It is
/// parameterised by <c>P(U, V) = O + (R + V * sin(Ang)) * (cos(U) * XDir + sin(U) * YDir) + V * cos(Ang) * ZDir</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Cone()</c> is OCCT's indefinite cone (reference radius <see cref="Standard.RealLast"/>, semi-angle
/// PI/4); <c>default(gp_Cone)</c> is not valid.
/// </remarks>
public partial struct gp_Cone : IEquatable<gp_Cone>
{
    private gp_Ax3 pos;
    private double radius;
    private double semiAngle;

    public gp_Cone()
    {
        pos = new gp_Ax3();
        radius = RealLast();
        semiAngle = M_PI * 0.25;
    }

    /// <exception cref="Standard_ConstructionError">
    /// <paramref name="theRadius"/> is negative, or |<paramref name="theAng"/>| is not in (Resolution, PI/2 - Resolution).
    /// </exception>
    public gp_Cone(in gp_Ax3 theA3, double theAng, double theRadius)
    {
        pos = theA3;
        radius = theRadius;
        semiAngle = theAng;
        double anAbsAng = theAng < 0.0 ? -theAng : theAng;
        Standard_ConstructionError_Raise_if(theRadius < 0.0 || anAbsAng <= gp.Resolution()
                                            || M_PI * 0.5 - anAbsAng <= gp.Resolution(),
                                            "gp_Cone() - invalid construction parameters");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in gp_Ax1 theA1) => pos.SetAxis(theA1);

    internal void SetLocation(in gp_Pnt theLoc) => pos.SetLocation(theLoc);

    internal void SetPosition(in gp_Ax3 theA3) => pos = theA3;

    internal void SetRadius(double theR)
    {
        Standard_ConstructionError_Raise_if(theR < 0.0, "gp_Cone::SetRadius() - radius should be positive number");
        radius = theR;
    }

    internal void SetSemiAngle(double theAng)
    {
        Standard_ConstructionError_Raise_if(Math.Abs(theAng) <= gp.Resolution()
                                            || M_PI * 0.5 - Math.Abs(theAng) <= gp.Resolution(),
                                            "gp_Cone::SetSemiAngle() - invalid angle range");
        semiAngle = theAng;
    }

    /// <summary>Reverses the U parametrisation (the Y direction).</summary>
    internal void UReverse() => pos.YReverse();

    /// <summary>Reverses the V parametrisation (the main direction), which reverses the semi-angle.</summary>
    internal void VReverse()
    {
        pos.ZReverse();
        semiAngle = -semiAngle;
    }

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

    public readonly gp_Cone WithAxis(in gp_Ax1 theA1)
    {
        gp_Cone aCopy = this;
        aCopy.SetAxis(theA1);
        return aCopy;
    }

    public readonly gp_Cone WithLocation(in gp_Pnt theLoc)
    {
        gp_Cone aCopy = this;
        aCopy.SetLocation(theLoc);
        return aCopy;
    }

    public readonly gp_Cone WithPosition(in gp_Ax3 theA3)
    {
        gp_Cone aCopy = this;
        aCopy.SetPosition(theA3);
        return aCopy;
    }

    public readonly gp_Cone WithRadius(double theR)
    {
        gp_Cone aCopy = this;
        aCopy.SetRadius(theR);
        return aCopy;
    }

    public readonly gp_Cone WithSemiAngle(double theAng)
    {
        gp_Cone aCopy = this;
        aCopy.SetSemiAngle(theAng);
        return aCopy;
    }

    public readonly gp_Cone UReversed()
    {
        gp_Cone aCopy = this;
        aCopy.UReverse();
        return aCopy;
    }

    public readonly gp_Cone VReversed()
    {
        gp_Cone aCopy = this;
        aCopy.VReverse();
        return aCopy;
    }

    // Queries

    /// <summary>The apex of the cone, on the negative side of the main axis when the semi-angle is positive.</summary>
    public readonly gp_Pnt Apex()
    {
        gp_XYZ aCoord = pos.Direction().XYZ();
        aCoord.Multiply(-radius / Math.Tan(semiAngle));
        aCoord.Add(pos.Location().XYZ());
        return new gp_Pnt(aCoord);
    }

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
        // Dans le repere du cone :
        // X**2 + Y**2 - (radius + Z * std::tan(semiAngle))**2 = 0.0
        gp_Trsf T = new();
        T.SetTransformation(pos);
        double KAng = Math.Tan(semiAngle);
        double T11 = T.Value(1, 1);
        double T12 = T.Value(1, 2);
        double T13 = T.Value(1, 3);
        double T14 = T.Value(1, 4);
        double T21 = T.Value(2, 1);
        double T22 = T.Value(2, 2);
        double T23 = T.Value(2, 3);
        double T24 = T.Value(2, 4);
        double T31 = T.Value(3, 1) * KAng;
        double T32 = T.Value(3, 2) * KAng;
        double T33 = T.Value(3, 3) * KAng;
        double T34 = T.Value(3, 4) * KAng;
        A1 = T11 * T11 + T21 * T21 - T31 * T31;
        A2 = T12 * T12 + T22 * T22 - T32 * T32;
        A3 = T13 * T13 + T23 * T23 - T33 * T33;
        B1 = T11 * T12 + T21 * T22 - T31 * T32;
        B2 = T11 * T13 + T21 * T23 - T31 * T33;
        B3 = T12 * T13 + T22 * T23 - T32 * T33;
        C1 = T11 * T14 + T21 * T24 - T31 * (radius + T34);
        C2 = T12 * T14 + T22 * T24 - T32 * (radius + T34);
        C3 = T13 * T14 + T23 * T24 - T33 * (radius + T34);
        D = T14 * T14 + T24 * T24 - radius * radius - T34 * T34 - 2.0 * radius * T34;
    }

    public readonly gp_Pnt Location() => pos.Location();

    public readonly gp_Ax3 Position() => pos;

    /// <summary>The radius of the cone in the plane of its coordinate system.</summary>
    public readonly double RefRadius() => radius;

    /// <summary>The half-angle at the apex, in (-PI/2, PI/2), never 0.</summary>
    public readonly double SemiAngle() => semiAngle;

    public readonly gp_Ax1 XAxis() => new(pos.Location(), pos.XDirection());

    public readonly gp_Ax1 YAxis() => new(pos.Location(), pos.YDirection());

    public readonly gp_Cone Mirrored(in gp_Pnt P)
    {
        gp_Cone C = this;
        C.pos.Mirror(P);
        return C;
    }

    public readonly gp_Cone Mirrored(in gp_Ax1 A1)
    {
        gp_Cone C = this;
        C.pos.Mirror(A1);
        return C;
    }

    public readonly gp_Cone Mirrored(in gp_Ax2 A2)
    {
        gp_Cone C = this;
        C.pos.Mirror(A2);
        return C;
    }

    public readonly gp_Cone Rotated(in gp_Ax1 theA1, double theAng)
    {
        gp_Cone aCone = this;
        aCone.pos.Rotate(theA1, theAng);
        return aCone;
    }

    public readonly gp_Cone Scaled(in gp_Pnt theP, double theS)
    {
        gp_Cone aC = this;
        aC.pos.Scale(theP, theS);
        aC.radius *= theS;
        if (aC.radius < 0)
        {
            aC.radius = -aC.radius;
        }

        return aC;
    }

    public readonly gp_Cone Transformed(in gp_Trsf theT)
    {
        gp_Cone aC = this;
        aC.pos.Transform(theT);
        aC.radius *= theT.ScaleFactor();
        if (aC.radius < 0)
        {
            aC.radius = -aC.radius;
        }

        return aC;
    }

    public readonly gp_Cone Translated(in gp_Vec theV)
    {
        gp_Cone aCone = this;
        aCone.pos.Translate(theV);
        return aCone;
    }

    public readonly gp_Cone Translated(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        gp_Cone aCone = this;
        aCone.pos.Translate(theP1, theP2);
        return aCone;
    }

    public readonly bool Equals(gp_Cone other)
        => pos.Equals(other.pos) && radius.Equals(other.radius) && semiAngle.Equals(other.semiAngle);

    public override readonly bool Equals(object? obj) => obj is gp_Cone other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pos, radius, semiAngle);

    public static bool operator ==(in gp_Cone theLeft, in gp_Cone theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Cone theLeft, in gp_Cone theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Cone({pos}, {semiAngle:R}, {radius:R})");
}
