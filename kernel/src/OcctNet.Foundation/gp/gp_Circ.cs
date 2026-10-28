// Port of: src/FoundationClasses/TKMath/gp/gp_Circ.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Circ.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A circle in 3D space, defined by its radius and positioned by a right-handed coordinate system whose origin is
/// the centre and whose main direction is the normal to the circle's plane. The circle is parameterised by the
/// angle from the X direction: <c>P(U) = O + R * (cos(U) * XDir + sin(U) * YDir)</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Circ()</c> is OCCT's indefinite circle (radius <see cref="Standard.RealLast"/>); <c>default(gp_Circ)</c>
/// is not valid.
/// </remarks>
public partial struct gp_Circ : IEquatable<gp_Circ>
{
    private gp_Ax2 pos;
    private double radius;

    public gp_Circ()
    {
        pos = new gp_Ax2();
        radius = RealLast();
    }

    /// <exception cref="Standard_ConstructionError"><paramref name="theRadius"/> is negative.</exception>
    public gp_Circ(in gp_Ax2 theA2, double theRadius)
    {
        pos = theA2;
        radius = theRadius;
        Standard_ConstructionError_Raise_if(theRadius < 0.0, "gp_Circ() - radius should be positive number");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in gp_Ax1 theA1) => pos.SetAxis(theA1);

    internal void SetLocation(in gp_Pnt theP) => pos.SetLocation(theP);

    internal void SetPosition(in gp_Ax2 theA2) => pos = theA2;

    internal void SetRadius(double theRadius)
    {
        Standard_ConstructionError_Raise_if(theRadius < 0.0, "gp_Circ::SetRadius() - radius should be positive number");
        radius = theRadius;
    }

    internal void Mirror(in gp_Pnt P) => pos.Mirror(P);

    internal void Mirror(in gp_Ax1 A1) => pos.Mirror(A1);

    internal void Mirror(in gp_Ax2 A2) => pos.Mirror(A2);

    internal void Rotate(in gp_Ax1 theA1, double theAng) => pos.Rotate(theA1, theAng);

    internal void Scale(in gp_Pnt theP, double theS)
    {
        radius *= theS;
        if (radius < 0)
        {
            radius = -radius;
        }

        pos.Scale(theP, theS);
    }

    internal void Transform(in gp_Trsf theT)
    {
        radius *= theT.ScaleFactor();
        if (radius < 0)
        {
            radius = -radius;
        }

        pos.Transform(theT);
    }

    internal void Translate(in gp_Vec theV) => pos.Translate(theV);

    internal void Translate(in gp_Pnt theP1, in gp_Pnt theP2) => pos.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Circ WithAxis(in gp_Ax1 theA1)
    {
        gp_Circ aCopy = this;
        aCopy.SetAxis(theA1);
        return aCopy;
    }

    public readonly gp_Circ WithLocation(in gp_Pnt theP)
    {
        gp_Circ aCopy = this;
        aCopy.SetLocation(theP);
        return aCopy;
    }

    public readonly gp_Circ WithPosition(in gp_Ax2 theA2)
    {
        gp_Circ aCopy = this;
        aCopy.SetPosition(theA2);
        return aCopy;
    }

    public readonly gp_Circ WithRadius(double theRadius)
    {
        gp_Circ aCopy = this;
        aCopy.SetRadius(theRadius);
        return aCopy;
    }

    // Queries

    public readonly double Area() => M_PI * radius * radius;

    /// <summary>The main axis of the circle: the normal to its plane through its centre.</summary>
    public readonly gp_Ax1 Axis() => pos.Axis();

    /// <summary>The circumference.</summary>
    public readonly double Length() => 2.0 * M_PI * radius;

    /// <summary>The centre of the circle.</summary>
    public readonly gp_Pnt Location() => pos.Location();

    public readonly gp_Ax2 Position() => pos;

    public readonly double Radius() => radius;

    public readonly gp_Ax1 XAxis() => new(pos.Location(), pos.XDirection());

    public readonly gp_Ax1 YAxis() => new(pos.Location(), pos.YDirection());

    public readonly double Distance(in gp_Pnt theP) => Math.Sqrt(SquareDistance(theP));

    public readonly double SquareDistance(in gp_Pnt theP)
    {
        gp_Vec aV = new(Location(), theP);
        double aX = aV.Dot(new gp_Vec(pos.XDirection()));
        double anY = aV.Dot(new gp_Vec(pos.YDirection()));
        double aZ = aV.Dot(new gp_Vec(pos.Direction()));
        double aT = Math.Sqrt(aX * aX + anY * anY) - radius;
        return (aT * aT + aZ * aZ);
    }

    public readonly bool Contains(in gp_Pnt theP, double theLinearTolerance) => Distance(theP) <= theLinearTolerance;

    public readonly gp_Circ Mirrored(in gp_Pnt P)
    {
        gp_Circ C = this;
        C.pos.Mirror(P);
        return C;
    }

    public readonly gp_Circ Mirrored(in gp_Ax1 A1)
    {
        gp_Circ C = this;
        C.pos.Mirror(A1);
        return C;
    }

    public readonly gp_Circ Mirrored(in gp_Ax2 A2)
    {
        gp_Circ C = this;
        C.pos.Mirror(A2);
        return C;
    }

    public readonly gp_Circ Rotated(in gp_Ax1 theA1, double theAng)
    {
        gp_Circ aC = this;
        aC.pos.Rotate(theA1, theAng);
        return aC;
    }

    public readonly gp_Circ Scaled(in gp_Pnt theP, double theS)
    {
        gp_Circ aC = this;
        aC.radius *= theS;
        if (aC.radius < 0)
        {
            aC.radius = -aC.radius;
        }

        aC.pos.Scale(theP, theS);
        return aC;
    }

    public readonly gp_Circ Transformed(in gp_Trsf theT)
    {
        gp_Circ aC = this;
        aC.radius *= theT.ScaleFactor();
        if (aC.radius < 0)
        {
            aC.radius = -aC.radius;
        }

        aC.pos.Transform(theT);
        return aC;
    }

    public readonly gp_Circ Translated(in gp_Vec theV)
    {
        gp_Circ aC = this;
        aC.pos.Translate(theV);
        return aC;
    }

    public readonly gp_Circ Translated(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        gp_Circ aC = this;
        aC.pos.Translate(theP1, theP2);
        return aC;
    }

    public readonly bool Equals(gp_Circ other) => pos.Equals(other.pos) && radius.Equals(other.radius);

    public override readonly bool Equals(object? obj) => obj is gp_Circ other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pos, radius);

    public static bool operator ==(in gp_Circ theLeft, in gp_Circ theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Circ theLeft, in gp_Circ theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Circ({pos}, {radius:R})");
}
