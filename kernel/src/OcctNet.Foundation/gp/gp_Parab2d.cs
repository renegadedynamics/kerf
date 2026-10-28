// Port of: src/FoundationClasses/TKMath/gp/gp_Parab2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Parab2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A parabola in 2D space, positioned by a coordinate system whose origin is the apex and whose X axis is the
/// axis of symmetry. In it the parabola is <c>Y^2 = (2 * P) * X</c>, where P = 2 * focal length.
/// </summary>
/// <remarks>
/// <c>new gp_Parab2d()</c> is OCCT's indefinite parabola (focal length <see cref="Standard.RealLast"/>);
/// <c>default(gp_Parab2d)</c> is not valid.
/// </remarks>
public partial struct gp_Parab2d : IEquatable<gp_Parab2d>
{
    private gp_Ax22d pos;
    private double focalLength;

    public gp_Parab2d()
    {
        pos = new gp_Ax22d();
        focalLength = RealLast();
    }

    /// <summary>A parabola with the given axis of symmetry and focal length, direct when <paramref name="theSense"/> is true.</summary>
    /// <exception cref="Standard_ConstructionError">The focal length is negative.</exception>
    public gp_Parab2d(in gp_Ax2d theMirrorAxis, double theFocalLength, bool theSense = true)
    {
        pos = new gp_Ax22d(theMirrorAxis, theSense);
        focalLength = theFocalLength;
        Standard_ConstructionError_Raise_if(theFocalLength < 0.0, "gp_Parab2d() - focal length should be >= 0");
    }

    /// <exception cref="Standard_ConstructionError">The focal length is negative.</exception>
    public gp_Parab2d(in gp_Ax22d theAxes, double theFocalLength)
    {
        pos = theAxes;
        focalLength = theFocalLength;
        Standard_ConstructionError_Raise_if(theFocalLength < 0.0, "gp_Parab2d() - focal length should be >= 0");
    }

    /// <summary>
    /// The parabola with the given directrix and focus. When the focus is on the directrix, the axis of symmetry is
    /// the directrix turned by -PI/2 (or PI/2 when <paramref name="theSense"/> is false) and the focal length is zero.
    /// </summary>
    public gp_Parab2d(in gp_Ax2d theDirectrix, in gp_Pnt2d theFocus, bool theSense = true)
    {
        gp_Pnt2d aDirLoc = theDirectrix.Location();
        gp_Dir2d aDirVec = theDirectrix.Direction();

        gp_Vec2d aFVec = new(aDirLoc, theFocus);

        gp_Pnt2d anOrigin = new(aDirLoc.XY() + aDirVec.XY() * (aFVec.Dot(new gp_Vec2d(aDirVec))));
        gp_Pnt2d anApex = new(0.5 * (anOrigin.XY() + theFocus.XY()));

        focalLength = 0.5 * anOrigin.Distance(theFocus);

        gp_Dir2d aXDir = (focalLength > 0.0)
                             ? new gp_Dir2d(theFocus.XY() - anOrigin.XY())
                             : theDirectrix.Rotated(aDirLoc, theSense ? -M_PI_2 : M_PI_2).Direction();

        pos = new gp_Ax22d(anApex, aXDir, aDirVec);
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetFocal(double theFocal)
    {
        Standard_ConstructionError_Raise_if(theFocal < 0.0, "gp_Parab2d::SetFocal() - focal length should be >= 0");
        focalLength = theFocal;
    }

    internal void SetLocation(in gp_Pnt2d theP) => pos.SetLocation(theP);

    internal void SetMirrorAxis(in gp_Ax2d theA) => pos.SetXAxis(theA);

    internal void SetAxis(in gp_Ax22d theA) => pos.SetAxis(theA);

    internal void Reverse()
    {
        gp_Dir2d aTemp = pos.YDirection();
        aTemp.Reverse();
        pos.SetAxis(new gp_Ax22d(pos.Location(), pos.XDirection(), aTemp));
    }

    internal void Mirror(in gp_Pnt2d P) => pos.Mirror(P);

    internal void Mirror(in gp_Ax2d A) => pos.Mirror(A);

    internal void Rotate(in gp_Pnt2d theP, double theAng) => pos.Rotate(theP, theAng);

    internal void Scale(in gp_Pnt2d theP, double theS)
    {
        focalLength *= theS;
        if (focalLength < 0)
        {
            focalLength = -focalLength;
        }

        pos.Scale(theP, theS);
    }

    internal void Transform(in gp_Trsf2d theT)
    {
        focalLength *= theT.ScaleFactor();
        if (focalLength < 0)
        {
            focalLength = -focalLength;
        }

        pos.Transform(theT);
    }

    internal void Translate(in gp_Vec2d theV) => pos.Translate(theV);

    internal void Translate(in gp_Pnt2d theP1, in gp_Pnt2d theP2) => pos.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Parab2d WithFocal(double theFocal)
    {
        gp_Parab2d aCopy = this;
        aCopy.SetFocal(theFocal);
        return aCopy;
    }

    public readonly gp_Parab2d WithLocation(in gp_Pnt2d theP)
    {
        gp_Parab2d aCopy = this;
        aCopy.SetLocation(theP);
        return aCopy;
    }

    public readonly gp_Parab2d WithMirrorAxis(in gp_Ax2d theA)
    {
        gp_Parab2d aCopy = this;
        aCopy.SetMirrorAxis(theA);
        return aCopy;
    }

    public readonly gp_Parab2d WithAxis(in gp_Ax22d theA)
    {
        gp_Parab2d aCopy = this;
        aCopy.SetAxis(theA);
        return aCopy;
    }

    // Queries

    /// <summary>
    /// The coefficients of the parabola's implicit equation
    /// <c>A * X^2 + B * Y^2 + 2 * C * X * Y + 2 * D * X + 2 * E * Y + F = 0</c>.
    /// </summary>
    public readonly void Coefficients(out double A, out double B, out double C, out double D, out double E, out double F)
    {
        double P = 2.0 * focalLength;
        gp_Trsf2d T = new();
        T.SetTransformation(pos.XAxis());
        double T11 = T.Value(1, 1);
        double T12 = T.Value(1, 2);
        double T13 = T.Value(1, 3);
        double T21 = T.Value(2, 1);
        double T22 = T.Value(2, 2);
        double T23 = T.Value(2, 3);
        A = T21 * T21;
        B = T22 * T22;
        C = T21 * T22;
        D = (T21 * T23) - (P * T11);
        E = (T22 * T23) - (P * T12);
        F = (T23 * T23) - (2.0 * P * T13);
    }

    /// <summary>The directrix: normal to the axis of symmetry, at the focal length on the side opposite the focus.</summary>
    public readonly gp_Ax2d Directrix()
    {
        gp_Pnt2d aP = new(pos.Location().X() - focalLength * pos.XDirection().X(),
                          pos.Location().Y() - focalLength * pos.XDirection().Y());
        gp_Dir2d aV = pos.YDirection();
        return new gp_Ax2d(aP, aV);
    }

    /// <summary>The distance between the apex and the focus.</summary>
    public readonly double Focal() => focalLength;

    public readonly gp_Pnt2d Focus()
        => new(pos.Location().X() + focalLength * pos.XDirection().X(),
               pos.Location().Y() + focalLength * pos.XDirection().Y());

    public readonly gp_Pnt2d Location() => pos.Location();

    public readonly gp_Ax2d MirrorAxis() => pos.XAxis();

    public readonly gp_Ax22d Axis() => pos;

    /// <summary>The parameter: the distance between the focus and the directrix (2 * focal length).</summary>
    public readonly double Parameter() => 2.0 * focalLength;

    /// <summary>True when the parabola's system is direct (counterclockwise).</summary>
    public readonly bool IsDirect() => (pos.XDirection().Crossed(pos.YDirection())) >= 0.0;

    public readonly gp_Parab2d Reversed()
    {
        gp_Parab2d aP = this;
        gp_Dir2d aTemp = pos.YDirection();
        aTemp.Reverse();
        aP.pos.SetAxis(new gp_Ax22d(pos.Location(), pos.XDirection(), aTemp));
        return aP;
    }

    public readonly gp_Parab2d Mirrored(in gp_Pnt2d P)
    {
        gp_Parab2d Prb = this;
        Prb.pos.Mirror(P);
        return Prb;
    }

    public readonly gp_Parab2d Mirrored(in gp_Ax2d A)
    {
        gp_Parab2d Prb = this;
        Prb.pos.Mirror(A);
        return Prb;
    }

    public readonly gp_Parab2d Rotated(in gp_Pnt2d theP, double theAng)
    {
        gp_Parab2d aPrb = this;
        aPrb.pos.Rotate(theP, theAng);
        return aPrb;
    }

    public readonly gp_Parab2d Scaled(in gp_Pnt2d theP, double theS)
    {
        gp_Parab2d aPrb = this;
        aPrb.focalLength *= theS;
        if (aPrb.focalLength < 0)
        {
            aPrb.focalLength = -aPrb.focalLength;
        }

        aPrb.pos.Scale(theP, theS);
        return aPrb;
    }

    public readonly gp_Parab2d Transformed(in gp_Trsf2d theT)
    {
        gp_Parab2d aPrb = this;
        aPrb.focalLength *= theT.ScaleFactor();
        if (aPrb.focalLength < 0)
        {
            aPrb.focalLength = -aPrb.focalLength;
        }

        aPrb.pos.Transform(theT);
        return aPrb;
    }

    public readonly gp_Parab2d Translated(in gp_Vec2d theV)
    {
        gp_Parab2d aPrb = this;
        aPrb.pos.Translate(theV);
        return aPrb;
    }

    public readonly gp_Parab2d Translated(in gp_Pnt2d theP1, in gp_Pnt2d theP2)
    {
        gp_Parab2d aPrb = this;
        aPrb.pos.Translate(theP1, theP2);
        return aPrb;
    }

    public readonly bool Equals(gp_Parab2d other) => pos.Equals(other.pos) && focalLength.Equals(other.focalLength);

    public override readonly bool Equals(object? obj) => obj is gp_Parab2d other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pos, focalLength);

    public static bool operator ==(in gp_Parab2d theLeft, in gp_Parab2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Parab2d theLeft, in gp_Parab2d theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_Parab2d({pos}, focal {focalLength})";
}
