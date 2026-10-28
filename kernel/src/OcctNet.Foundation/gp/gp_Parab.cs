// Port of: src/FoundationClasses/TKMath/gp/gp_Parab.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Parab.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A parabola in 3D space, defined by its focal length and positioned by a right-handed coordinate system: the
/// origin is the apex, the X direction the axis of symmetry and the main direction the normal to its plane. It is
/// parameterised by <c>P(U) = O + U^2 / (4 * F) * XDir + U * YDir</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Parab()</c> is OCCT's indefinite parabola (focal length <see cref="Standard.RealLast"/>);
/// <c>default(gp_Parab)</c> is not valid.
/// </remarks>
public partial struct gp_Parab : IEquatable<gp_Parab>
{
    private gp_Ax2 pos;
    private double focalLength;

    public gp_Parab()
    {
        pos = new gp_Ax2();
        focalLength = RealLast();
    }

    /// <exception cref="Standard_ConstructionError"><paramref name="theFocal"/> is negative.</exception>
    public gp_Parab(in gp_Ax2 theA2, double theFocal)
    {
        pos = theA2;
        focalLength = theFocal;
        Standard_ConstructionError_Raise_if(theFocal < 0.0, "gp_Parab() - focal length should be >= 0");
    }

    /// <summary>The parabola with directrix <paramref name="theD"/> and focus <paramref name="theF"/>.</summary>
    /// <exception cref="Standard_ConstructionError"><paramref name="theF"/> is on <paramref name="theD"/>.</exception>
    public gp_Parab(in gp_Ax1 theD, in gp_Pnt theF)
    {
        gp_Lin aDroite = new(theD);
        focalLength = aDroite.Distance(theF) / 2.0;
        gp_Ax1 anAx = aDroite.Normal(theF).Position();
        gp_Ax1 anAy = aDroite.Position();
        gp_Dir aDD = anAx.Direction();
        pos = new gp_Ax2(new gp_Pnt(theF.X() - focalLength * aDD.X(),
                                    theF.Y() - focalLength * aDD.Y(),
                                    theF.Z() - focalLength * aDD.Z()),
                         anAx.Direction().Crossed(anAy.Direction()),
                         anAx.Direction());
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in gp_Ax1 theA1) => pos.SetAxis(theA1);

    internal void SetFocal(double theFocal)
    {
        Standard_ConstructionError_Raise_if(theFocal < 0.0, "gp_Parab::SetFocal() - focal length should be >= 0");
        focalLength = theFocal;
    }

    internal void SetLocation(in gp_Pnt theP) => pos.SetLocation(theP);

    internal void SetPosition(in gp_Ax2 theA2) => pos = theA2;

    internal void Mirror(in gp_Pnt P) => pos.Mirror(P);

    internal void Mirror(in gp_Ax1 A1) => pos.Mirror(A1);

    internal void Mirror(in gp_Ax2 A2) => pos.Mirror(A2);

    internal void Rotate(in gp_Ax1 theA1, double theAng) => pos.Rotate(theA1, theAng);

    internal void Scale(in gp_Pnt theP, double theS)
    {
        focalLength *= theS;
        if (focalLength < 0)
        {
            focalLength = -focalLength;
        }

        pos.Scale(theP, theS);
    }

    internal void Transform(in gp_Trsf theT)
    {
        focalLength *= theT.ScaleFactor();
        if (focalLength < 0)
        {
            focalLength = -focalLength;
        }

        pos.Transform(theT);
    }

    internal void Translate(in gp_Vec theV) => pos.Translate(theV);

    internal void Translate(in gp_Pnt theP1, in gp_Pnt theP2) => pos.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Parab WithAxis(in gp_Ax1 theA1)
    {
        gp_Parab aCopy = this;
        aCopy.SetAxis(theA1);
        return aCopy;
    }

    public readonly gp_Parab WithFocal(double theFocal)
    {
        gp_Parab aCopy = this;
        aCopy.SetFocal(theFocal);
        return aCopy;
    }

    public readonly gp_Parab WithLocation(in gp_Pnt theP)
    {
        gp_Parab aCopy = this;
        aCopy.SetLocation(theP);
        return aCopy;
    }

    public readonly gp_Parab WithPosition(in gp_Ax2 theA2)
    {
        gp_Parab aCopy = this;
        aCopy.SetPosition(theA2);
        return aCopy;
    }

    // Queries

    /// <summary>The main axis: the normal to the parabola's plane through its apex.</summary>
    public readonly gp_Ax1 Axis() => pos.Axis();

    /// <summary>The directrix: parallel to the Y axis, at the focal length from the apex on the negative X side.</summary>
    public readonly gp_Ax1 Directrix()
    {
        gp_Pnt aPP = pos.Location();
        gp_Dir aDD = pos.XDirection();
        gp_Pnt aP = new(aPP.X() - focalLength * aDD.X(),
                        aPP.Y() - focalLength * aDD.Y(),
                        aPP.Z() - focalLength * aDD.Z());
        return new gp_Ax1(aP, pos.YDirection());
    }

    /// <summary>The distance between the apex and the focus.</summary>
    public readonly double Focal() => focalLength;

    public readonly gp_Pnt Focus()
    {
        gp_Pnt aPP = pos.Location();
        gp_Dir aDD = pos.XDirection();
        return new gp_Pnt(aPP.X() + focalLength * aDD.X(),
                          aPP.Y() + focalLength * aDD.Y(),
                          aPP.Z() + focalLength * aDD.Z());
    }

    /// <summary>The apex.</summary>
    public readonly gp_Pnt Location() => pos.Location();

    /// <summary>The distance between the focus and the directrix: twice the focal length.</summary>
    public readonly double Parameter() => 2.0 * focalLength;

    public readonly gp_Ax2 Position() => pos;

    /// <summary>The axis of symmetry.</summary>
    public readonly gp_Ax1 XAxis() => new(pos.Location(), pos.XDirection());

    public readonly gp_Ax1 YAxis() => new(pos.Location(), pos.YDirection());

    public readonly gp_Parab Mirrored(in gp_Pnt P)
    {
        gp_Parab Prb = this;
        Prb.pos.Mirror(P);
        return Prb;
    }

    public readonly gp_Parab Mirrored(in gp_Ax1 A1)
    {
        gp_Parab Prb = this;
        Prb.pos.Mirror(A1);
        return Prb;
    }

    public readonly gp_Parab Mirrored(in gp_Ax2 A2)
    {
        gp_Parab Prb = this;
        Prb.pos.Mirror(A2);
        return Prb;
    }

    public readonly gp_Parab Rotated(in gp_Ax1 theA1, double theAng)
    {
        gp_Parab aPrb = this;
        aPrb.pos.Rotate(theA1, theAng);
        return aPrb;
    }

    public readonly gp_Parab Scaled(in gp_Pnt theP, double theS)
    {
        gp_Parab aPrb = this;
        aPrb.focalLength *= theS;
        if (aPrb.focalLength < 0)
        {
            aPrb.focalLength = -aPrb.focalLength;
        }

        aPrb.pos.Scale(theP, theS);
        return aPrb;
    }

    public readonly gp_Parab Transformed(in gp_Trsf theT)
    {
        gp_Parab aPrb = this;
        aPrb.focalLength *= theT.ScaleFactor();
        if (aPrb.focalLength < 0)
        {
            aPrb.focalLength = -aPrb.focalLength;
        }

        aPrb.pos.Transform(theT);
        return aPrb;
    }

    public readonly gp_Parab Translated(in gp_Vec theV)
    {
        gp_Parab aPrb = this;
        aPrb.pos.Translate(theV);
        return aPrb;
    }

    public readonly gp_Parab Translated(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        gp_Parab aPrb = this;
        aPrb.pos.Translate(theP1, theP2);
        return aPrb;
    }

    public readonly bool Equals(gp_Parab other) => pos.Equals(other.pos) && focalLength.Equals(other.focalLength);

    public override readonly bool Equals(object? obj) => obj is gp_Parab other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pos, focalLength);

    public static bool operator ==(in gp_Parab theLeft, in gp_Parab theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Parab theLeft, in gp_Parab theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Parab({pos}, {focalLength:R})");
}
