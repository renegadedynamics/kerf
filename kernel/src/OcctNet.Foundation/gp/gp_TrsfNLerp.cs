// Port of: src/FoundationClasses/TKMath/gp/gp_TrsfNLerp.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Linear interpolation of <see cref="gp_Trsf"/>: the rotation (with <see cref="gp_QuaternionNLerp"/>), the
/// translation and the scale factor are interpolated independently. There is no well-defined interpolation
/// between arbitrary transformations, so this suits simple cases and small intervals.
/// </summary>
/// <remarks>
/// OCCT names this class <c>NCollection_Lerp&lt;gp_Trsf&gt;</c>, a specialisation of the <c>NCollection_Lerp</c>
/// template. The port names it after its header, and writes its translation and scale members (an
/// <c>NCollection_Lerp</c> of <c>gp_XYZ</c> and of <c>double</c>) as start and end fields interpolated with the
/// template's formula <c>(1 - t) * start + t * end</c>.
/// <c>new gp_TrsfNLerp()</c> holds identities; <c>default(gp_TrsfNLerp)</c> is not valid.
/// </remarks>
public partial struct gp_TrsfNLerp : IEquatable<gp_TrsfNLerp>
{
    private gp_XYZ myLocLerpStart;
    private gp_XYZ myLocLerpEnd;
    private double myScaleLerpStart;
    private double myScaleLerpEnd;
    private gp_QuaternionNLerp myRotLerp;
    private gp_Trsf myTrsfStart;
    private gp_Trsf myTrsfEnd;

    /// <summary>The transformation interpolated between two transformations (OCCT's static <c>NCollection_Lerp::Interpolate</c>).</summary>
    public static gp_Trsf Interpolate(in gp_Trsf theStart, in gp_Trsf theEnd, double theT)
    {
        gp_TrsfNLerp aLerp = new(theStart, theEnd);
        aLerp.Interpolate(theT, out gp_Trsf aResult);
        return aResult;
    }

    public gp_TrsfNLerp()
    {
        myLocLerpStart = new gp_XYZ();
        myLocLerpEnd = new gp_XYZ();
        myScaleLerpStart = 0.0;
        myScaleLerpEnd = 0.0;
        myRotLerp = new gp_QuaternionNLerp();
        myTrsfStart = new gp_Trsf();
        myTrsfEnd = new gp_Trsf();
    }

    /// <summary>The tool initialised with two transformations (OCCT's <c>Init</c>).</summary>
    public gp_TrsfNLerp(in gp_Trsf theStart, in gp_Trsf theEnd) : this() => Init(theStart, theEnd);

    // Mutators (internal: see docs/translation-rules.md)

    internal void Init(in gp_Trsf theStart, in gp_Trsf theEnd)
    {
        myTrsfStart = theStart;
        myTrsfEnd = theEnd;
        myLocLerpStart = theStart.TranslationPart();
        myLocLerpEnd = theEnd.TranslationPart();
        myRotLerp.Init(theStart.GetRotation(), theEnd.GetRotation());
        myScaleLerpStart = theStart.ScaleFactor();
        myScaleLerpEnd = theEnd.ScaleFactor();
    }

    // Queries

    /// <summary>The transformation interpolated at <paramref name="theT"/> in [0, 1].</summary>
    public readonly void Interpolate(double theT, out gp_Trsf theResult)
    {
        if (Math.Abs(theT - 0.0) < Precision.Confusion())
        {
            theResult = myTrsfStart;
            return;
        }
        else if (Math.Abs(theT - 1.0) < Precision.Confusion())
        {
            theResult = myTrsfEnd;
            return;
        }

        gp_XYZ aLoc = (1.0 - theT) * myLocLerpStart + theT * myLocLerpEnd; // myLocLerp.Interpolate(theT, aLoc)
        myRotLerp.Interpolate(theT, out gp_Quaternion aRot);
        double aScale = (1.0 - theT) * myScaleLerpStart + theT * myScaleLerpEnd; // myScaleLerp.Interpolate(theT, aScale)
        theResult = new gp_Trsf();
        theResult.SetRotation(aRot);
        theResult.SetTranslationPart(new gp_Vec(aLoc));
        theResult.SetScaleFactor(aScale);
    }

    public readonly bool Equals(gp_TrsfNLerp other)
        => myLocLerpStart.Equals(other.myLocLerpStart) && myLocLerpEnd.Equals(other.myLocLerpEnd)
           && myScaleLerpStart.Equals(other.myScaleLerpStart) && myScaleLerpEnd.Equals(other.myScaleLerpEnd)
           && myRotLerp.Equals(other.myRotLerp) && myTrsfStart.Equals(other.myTrsfStart) && myTrsfEnd.Equals(other.myTrsfEnd);

    public override readonly bool Equals(object? obj) => obj is gp_TrsfNLerp other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(myLocLerpStart, myLocLerpEnd, myScaleLerpStart, myScaleLerpEnd, myRotLerp, myTrsfStart, myTrsfEnd);

    public static bool operator ==(in gp_TrsfNLerp theLeft, in gp_TrsfNLerp theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_TrsfNLerp theLeft, in gp_TrsfNLerp theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_TrsfNLerp({myTrsfStart}, {myTrsfEnd})";
}
