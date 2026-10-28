// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_B2.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// OCCT's template Bnd_B2<RealType> is ported once per instantiation (Bnd_B2d, Bnd_B2f) from the same text, with
// RealType a file-level alias, so C++'s float/double promotions are kept exactly (see docs/decisions.md).

using System.Runtime.CompilerServices;
using static Occt.gp_TrsfForm;
using RealType = double;

namespace Occt;

/// <summary>
/// A 2D axis-aligned bounding box stored as a center and a half-diagonal, in double precision (OCCT's
/// <c>Bnd_B2&lt;double&gt;</c>).
/// </summary>
/// <remarks>
/// OCCT's default (and <c>new Bnd_B2d()</c>) is a void box. <c>default(Bnd_B2d)</c> is not: it is a box reduced to
/// the origin. Ported code must write <c>new Bnd_B2d()</c>.
/// </remarks>
public partial struct Bnd_B2d : IEquatable<Bnd_B2d>
{
    /// <summary>Two reals, as OCCT's <c>std::array&lt;RealType, 2&gt;</c>.</summary>
    [InlineArray(2)]
    public struct Array2
    {
        private RealType myElement0;

        public Array2(RealType the0, RealType the1)
        {
            this[0] = the0;
            this[1] = the1;
        }
    }

    // Constant representing a very large value for void box initialization
    private const RealType THE_RealLast = (RealType)1e30;

    private Array2 myCenter;
    private Array2 myHSize;

    /// <summary>Creates a void box.</summary>
    public Bnd_B2d()
    {
        myCenter = new Array2(THE_RealLast, THE_RealLast);
        myHSize = new Array2(-THE_RealLast, -THE_RealLast);
    }

    public Bnd_B2d(in gp_XY theCenter, in gp_XY theHSize)
    {
        myCenter = new Array2((RealType)theCenter.X(), (RealType)theCenter.Y());
        myHSize = new Array2((RealType)theHSize.X(), (RealType)theHSize.Y());
    }

    public Bnd_B2d(in Array2 theCenter, in Array2 theHSize)
    {
        myCenter = theCenter;
        myHSize = theHSize;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Clear()
    {
        myCenter[0] = THE_RealLast;
        myCenter[1] = THE_RealLast;
        myHSize[0] = -THE_RealLast;
        myHSize[1] = -THE_RealLast;
    }

    internal void Add(in gp_XY thePnt)
    {
        if (IsVoid())
        {
            myCenter[0] = (RealType)thePnt.X();
            myCenter[1] = (RealType)thePnt.Y();
            myHSize[0] = (RealType)0.0;
            myHSize[1] = (RealType)0.0;
        }
        else
        {
            Span<RealType> aDiff = [(RealType)thePnt.X() - myCenter[0], (RealType)thePnt.Y() - myCenter[1]];
            if (aDiff[0] > myHSize[0])
            {
                RealType aShift = (aDiff[0] - myHSize[0]) / 2;
                myCenter[0] += aShift;
                myHSize[0] += aShift;
            }
            else if (aDiff[0] < -myHSize[0])
            {
                RealType aShift = (aDiff[0] + myHSize[0]) / 2;
                myCenter[0] += aShift;
                myHSize[0] -= aShift;
            }

            if (aDiff[1] > myHSize[1])
            {
                RealType aShift = (aDiff[1] - myHSize[1]) / 2;
                myCenter[1] += aShift;
                myHSize[1] += aShift;
            }
            else if (aDiff[1] < -myHSize[1])
            {
                RealType aShift = (aDiff[1] + myHSize[1]) / 2;
                myCenter[1] += aShift;
                myHSize[1] -= aShift;
            }
        }
    }

    internal void Add(in gp_Pnt2d thePnt) => Add(thePnt.XY());

    internal void Add(in Bnd_B2d theBox)
    {
        if (!theBox.IsVoid())
        {
            Add(theBox.CornerMin());
            Add(theBox.CornerMax());
        }
    }

    internal void Enlarge(double aDiff)
    {
        RealType aD = (RealType)Math.Abs(aDiff);
        myHSize[0] += aD;
        myHSize[1] += aD;
    }

    /// <summary>Limits the box by the inside of <paramref name="theBox"/>; false if the boxes do not intersect.</summary>
    internal bool Limit(in Bnd_B2d theBox)
    {
        bool aResult = false;
        Span<RealType> diffC = [theBox.myCenter[0] - myCenter[0], theBox.myCenter[1] - myCenter[1]];
        Span<RealType> sumH = [theBox.myHSize[0] + myHSize[0], theBox.myHSize[1] + myHSize[1]];
        // check the condition IsOut
        if (!compareDist(sumH, diffC))
        {
            Span<RealType> diffH = [theBox.myHSize[0] - myHSize[0], theBox.myHSize[1] - myHSize[1]];
            if (diffC[0] - diffH[0] > 0.0)
            {
                RealType aShift = (diffC[0] - diffH[0]) / 2; // positive
                myCenter[0] += aShift;
                myHSize[0] -= aShift;
            }
            else if (diffC[0] + diffH[0] < 0.0)
            {
                RealType aShift = (diffC[0] + diffH[0]) / 2; // negative
                myCenter[0] += aShift;
                myHSize[0] += aShift;
            }

            if (diffC[1] - diffH[1] > 0.0)
            {
                RealType aShift = (diffC[1] - diffH[1]) / 2; // positive
                myCenter[1] += aShift;
                myHSize[1] -= aShift;
            }
            else if (diffC[1] + diffH[1] < 0.0)
            {
                RealType aShift = (diffC[1] + diffH[1]) / 2; // negative
                myCenter[1] += aShift;
                myHSize[1] += aShift;
            }

            aResult = true;
        }

        return aResult;
    }

    internal void SetCenter(in gp_XY theCenter)
    {
        myCenter[0] = (RealType)theCenter.X();
        myCenter[1] = (RealType)theCenter.Y();
    }

    internal void SetCenter(in Array2 theCenter) => myCenter = theCenter;

    internal void SetHSize(in gp_XY theHSize)
    {
        myHSize[0] = (RealType)theHSize.X();
        myHSize[1] = (RealType)theHSize.Y();
    }

    internal void SetHSize(in Array2 theHSize) => myHSize = theHSize;

    // Copies with a mutator applied (the public way to "set")

    public readonly Bnd_B2d Cleared() => new();

    public readonly Bnd_B2d Added(in gp_XY thePnt)
    {
        Bnd_B2d aCopy = this;
        aCopy.Add(thePnt);
        return aCopy;
    }

    public readonly Bnd_B2d Added(in gp_Pnt2d thePnt)
    {
        Bnd_B2d aCopy = this;
        aCopy.Add(thePnt);
        return aCopy;
    }

    public readonly Bnd_B2d Added(in Bnd_B2d theBox)
    {
        Bnd_B2d aCopy = this;
        aCopy.Add(theBox);
        return aCopy;
    }

    public readonly Bnd_B2d Enlarged(double theDiff)
    {
        Bnd_B2d aCopy = this;
        aCopy.Enlarge(theDiff);
        return aCopy;
    }

    /// <summary>This box limited by the inside of <paramref name="theBox"/>; unchanged if they do not intersect (<see cref="IsOut(in Bnd_B2d)"/>).</summary>
    public readonly Bnd_B2d Limited(in Bnd_B2d theBox)
    {
        Bnd_B2d aCopy = this;
        aCopy.Limit(theBox);
        return aCopy;
    }

    public readonly Bnd_B2d WithCenter(in gp_XY theCenter)
    {
        Bnd_B2d aCopy = this;
        aCopy.SetCenter(theCenter);
        return aCopy;
    }

    public readonly Bnd_B2d WithCenter(in Array2 theCenter)
    {
        Bnd_B2d aCopy = this;
        aCopy.SetCenter(theCenter);
        return aCopy;
    }

    public readonly Bnd_B2d WithHSize(in gp_XY theHSize)
    {
        Bnd_B2d aCopy = this;
        aCopy.SetHSize(theHSize);
        return aCopy;
    }

    public readonly Bnd_B2d WithHSize(in Array2 theHSize)
    {
        Bnd_B2d aCopy = this;
        aCopy.SetHSize(theHSize);
        return aCopy;
    }

    // Queries

    /// <summary>True if the box is void (non-initialized).</summary>
    public readonly bool IsVoid() => (myHSize[0] < -1e-5);

    /// <summary>Center - HSize. Irrelevant for a void box.</summary>
    public readonly gp_XY CornerMin() => new(myCenter[0] - myHSize[0], myCenter[1] - myHSize[1]);

    /// <summary>Center + HSize. Irrelevant for a void box.</summary>
    public readonly gp_XY CornerMax() => new(myCenter[0] + myHSize[0], myCenter[1] + myHSize[1]);

    /// <summary>The square diagonal; very big for a void box.</summary>
    public readonly double SquareExtent() => 4 * (myHSize[0] * myHSize[0] + myHSize[1] * myHSize[1]);

    public readonly Array2 Center() => myCenter;

    public readonly Array2 HSize() => myHSize;

    /// <summary>The box bounding this box transformed by <paramref name="theTrsf"/> (larger if it rotates).</summary>
    public readonly Bnd_B2d Transformed(in gp_Trsf2d theTrsf)
    {
        Bnd_B2d aResult = new();
        gp_TrsfForm aForm = theTrsf.Form();
        double aScale = theTrsf.ScaleFactor();
        double aScaleAbs = Math.Abs(aScale);
        if (aForm == gp_Identity)
        {
            aResult = this;
        }
        else if (aForm == gp_Translation || aForm == gp_PntMirror || aForm == gp_Scale)
        {
            aResult.myCenter[0] = (RealType)(myCenter[0] * aScale + theTrsf.TranslationPart().X());
            aResult.myCenter[1] = (RealType)(myCenter[1] * aScale + theTrsf.TranslationPart().Y());
            aResult.myHSize[0] = (RealType)(myHSize[0] * aScaleAbs);
            aResult.myHSize[1] = (RealType)(myHSize[1] * aScaleAbs);
        }
        else
        {
            gp_XY aCenter = new((double)myCenter[0], (double)myCenter[1]);
            theTrsf.Transforms(ref aCenter);
            aResult.myCenter[0] = (RealType)aCenter.X();
            aResult.myCenter[1] = (RealType)aCenter.Y();
            Span<double> aMat = stackalloc double[4];
            theTrsf.HVectorialPart().CopyTo(aMat);
            aResult.myHSize[0] =
                (RealType)(aScaleAbs * (Math.Abs(aMat[0]) * myHSize[0] + Math.Abs(aMat[1]) * myHSize[1]));
            aResult.myHSize[1] =
                (RealType)(aScaleAbs * (Math.Abs(aMat[2]) * myHSize[0] + Math.Abs(aMat[3]) * myHSize[1]));
        }

        return aResult;
    }

    /// <summary>True if the point is outside the box.</summary>
    public readonly bool IsOut(in gp_XY thePnt)
        => (Math.Abs((RealType)thePnt.X() - myCenter[0]) > myHSize[0]
            || Math.Abs((RealType)thePnt.Y() - myCenter[1]) > myHSize[1]);

    /// <summary>True if the circle (or its outline, if hollow) does not intersect the box.</summary>
    public readonly bool IsOut(in gp_XY theCenter, double theRadius, bool isCircleHollow = false)
    {
        bool aResult = true;
        if (!isCircleHollow)
        {
            // vector from the center of the circle to the nearest box face
            Span<double> aDist =
            [
                Math.Abs(theCenter.X() - (double)myCenter[0]) - (double)myHSize[0],
                Math.Abs(theCenter.Y() - (double)myCenter[1]) - (double)myHSize[1],
            ];
            double aD = 0.0;
            if (aDist[0] > 0.0)
            {
                aD = aDist[0] * aDist[0];
            }

            if (aDist[1] > 0.0)
            {
                aD += aDist[1] * aDist[1];
            }

            aResult = (aD > theRadius * theRadius);
        }
        else
        {
            Span<double> aDistC =
            [
                Math.Abs(theCenter.X() - (double)myCenter[0]),
                Math.Abs(theCenter.Y() - (double)myCenter[1]),
            ];
            // vector from the center of the circle to the nearest box face
            Span<double> aDist = [aDistC[0] - (double)myHSize[0], aDistC[1] - (double)myHSize[1]];
            double aD = 0.0;
            if (aDist[0] > 0.0)
            {
                aD = aDist[0] * aDist[0];
            }

            if (aDist[1] > 0.0)
            {
                aD += aDist[1] * aDist[1];
            }

            if (aD < theRadius * theRadius)
            {
                // the box intersects the solid circle; check if it is completely
                // inside the circle (in such case return isOut==True)
                aDist[0] = aDistC[0] + (double)myHSize[0];
                aDist[1] = aDistC[1] + (double)myHSize[1];
                if (aDist[0] * aDist[0] + aDist[1] * aDist[1] > theRadius * theRadius)
                {
                    aResult = false;
                }
            }
        }

        return aResult;
    }

    /// <summary>True if <paramref name="theBox"/> does not intersect this box.</summary>
    public readonly bool IsOut(in Bnd_B2d theBox)
        => (Math.Abs(theBox.myCenter[0] - myCenter[0]) > theBox.myHSize[0] + myHSize[0]
            || Math.Abs(theBox.myCenter[1] - myCenter[1]) > theBox.myHSize[1] + myHSize[1]);

    /// <summary>True if <paramref name="theBox"/> transformed by <paramref name="theTrsf"/> does not intersect this box.</summary>
    public readonly bool IsOut(in Bnd_B2d theBox, in gp_Trsf2d theTrsf)
    {
        bool aResult = false;
        gp_TrsfForm aForm = theTrsf.Form();
        double aScale = theTrsf.ScaleFactor();
        double aScaleAbs = Math.Abs(aScale);
        if (aForm == gp_Translation || aForm == gp_Identity || aForm == gp_PntMirror || aForm == gp_Scale)
        {
            aResult =
                (Math.Abs((RealType)(theBox.myCenter[0] * aScale + theTrsf.TranslationPart().X()) - myCenter[0])
                     > (RealType)(theBox.myHSize[0] * aScaleAbs) + myHSize[0]
                 || Math.Abs((RealType)(theBox.myCenter[1] * aScale + theTrsf.TranslationPart().Y()) - myCenter[1])
                     > (RealType)(theBox.myHSize[1] * aScaleAbs) + myHSize[1]);
        }
        else
        {
            // theBox is transformed and we check the resulting (enlarged) box against
            // 'this' box.
            Span<double> aMat = stackalloc double[4];
            theTrsf.HVectorialPart().CopyTo(aMat);
            gp_XY aCenter = new((double)theBox.myCenter[0], (double)theBox.myCenter[1]);
            theTrsf.Transforms(ref aCenter);
            Span<double> aDist = [aCenter.X() - (double)myCenter[0], aCenter.Y() - (double)myCenter[1]];
            Span<double> aMatAbs = [Math.Abs(aMat[0]), Math.Abs(aMat[1]), Math.Abs(aMat[2]), Math.Abs(aMat[3])];
            if (Math.Abs(aDist[0])
                    > (aScaleAbs * (aMatAbs[0] * theBox.myHSize[0] + aMatAbs[1] * theBox.myHSize[1])
                       + (double)myHSize[0])
                || Math.Abs(aDist[1])
                    > (aScaleAbs * (aMatAbs[2] * theBox.myHSize[0] + aMatAbs[3] * theBox.myHSize[1])
                       + (double)myHSize[1]))
            {
                aResult = true;
            }
            else
            {
                // theBox is rotated, scaled and translated. We apply the reverse
                // translation and scaling then check against the rotated box 'this'
                if ((Math.Abs(aMat[0] * aDist[0] + aMat[2] * aDist[1])
                     > theBox.myHSize[0] * aScaleAbs + (aMatAbs[0] * myHSize[0] + aMatAbs[2] * myHSize[1]))
                    || (Math.Abs(aMat[1] * aDist[0] + aMat[3] * aDist[1])
                        > theBox.myHSize[1] * aScaleAbs + (aMatAbs[1] * myHSize[0] + aMatAbs[3] * myHSize[1])))
                {
                    aResult = true;
                }
            }
        }

        return aResult;
    }

    /// <summary>True if the line does not intersect the box.</summary>
    public readonly bool IsOut(in gp_Ax2d theLine)
    {
        if (IsVoid())
        {
            return true;
        }

        // Intersect the line containing the segment.
        Span<double> aProd =
        [
            theLine.Direction().XY()
                ^ (new gp_XY(myCenter[0] - theLine.Location().X(), myCenter[1] - theLine.Location().Y())),
            theLine.Direction().X() * (double)myHSize[1],
            theLine.Direction().Y() * (double)myHSize[0],
        ];
        return (Math.Abs(aProd[0]) > (Math.Abs(aProd[1]) + Math.Abs(aProd[2])));
    }

    /// <summary>True if the segment [theP0, theP1] does not intersect the box.</summary>
    public readonly bool IsOut(in gp_XY theP0, in gp_XY theP1)
    {
        bool aResult = true;
        if (!IsVoid())
        {
            // Intersect the line containing the segment.
            gp_XY aSegDelta = theP1 - theP0;
            Span<double> aProd =
            [
                aSegDelta ^ (new gp_XY(myCenter[0], myCenter[1]) - theP0),
                aSegDelta.X() * (double)myHSize[1],
                aSegDelta.Y() * (double)myHSize[0],
            ];
            if (Math.Abs(aProd[0]) < (Math.Abs(aProd[1]) + Math.Abs(aProd[2])))
            {
                // Intersection with line detected; check the segment as bounding box
                gp_XY aHSeg = new(0.5 * aSegDelta.X(), 0.5 * aSegDelta.Y());
                gp_XY aHSegAbs = new(Math.Abs(aHSeg.X()), Math.Abs(aHSeg.Y()));
                aResult = compareDistD(new gp_XY((double)myHSize[0], (double)myHSize[1]) + aHSegAbs,
                                       theP0 + aHSeg - new gp_XY((double)myCenter[0], (double)myCenter[1]));
            }
        }

        return aResult;
    }

    /// <summary>True if this box is fully inside <paramref name="theBox"/>.</summary>
    public readonly bool IsIn(in Bnd_B2d theBox)
        => (Math.Abs(theBox.myCenter[0] - myCenter[0]) < theBox.myHSize[0] - myHSize[0]
            && Math.Abs(theBox.myCenter[1] - myCenter[1]) < theBox.myHSize[1] - myHSize[1]);

    /// <summary>True if this box is fully inside <paramref name="theBox"/> transformed by <paramref name="theTrsf"/>.</summary>
    public readonly bool IsIn(in Bnd_B2d theBox, in gp_Trsf2d theTrsf)
    {
        bool aResult = false;
        gp_TrsfForm aForm = theTrsf.Form();
        double aScale = theTrsf.ScaleFactor();
        double aScaleAbs = Math.Abs(aScale);
        if (aForm == gp_Translation || aForm == gp_Identity || aForm == gp_PntMirror || aForm == gp_Scale)
        {
            aResult =
                (Math.Abs((RealType)(theBox.myCenter[0] * aScale + theTrsf.TranslationPart().X()) - myCenter[0])
                     < (RealType)(theBox.myHSize[0] * aScaleAbs) - myHSize[0]
                 && Math.Abs((RealType)(theBox.myCenter[1] * aScale + theTrsf.TranslationPart().Y()) - myCenter[1])
                     < (RealType)(theBox.myHSize[1] * aScaleAbs) - myHSize[1]);
        }
        else
        {
            // theBox is rotated, scaled and translated. We apply the reverse
            // translation and scaling then check against the rotated box 'this'
            Span<double> aMat = stackalloc double[4];
            theTrsf.HVectorialPart().CopyTo(aMat);
            gp_XY aCenter = new((double)theBox.myCenter[0], (double)theBox.myCenter[1]);
            theTrsf.Transforms(ref aCenter);
            Span<double> aDist = [aCenter.X() - (double)myCenter[0], aCenter.Y() - (double)myCenter[1]];
            if ((Math.Abs(aMat[0] * aDist[0] + aMat[2] * aDist[1])
                 < theBox.myHSize[0] * aScaleAbs - (Math.Abs(aMat[0]) * myHSize[0] + Math.Abs(aMat[2]) * myHSize[1]))
                && (Math.Abs(aMat[1] * aDist[0] + aMat[3] * aDist[1])
                    < theBox.myHSize[1] * aScaleAbs - (Math.Abs(aMat[1]) * myHSize[0] + Math.Abs(aMat[3]) * myHSize[1])))
            {
                aResult = true;
            }
        }

        return aResult;
    }

    private static bool compareDist(ReadOnlySpan<RealType> aHSize, ReadOnlySpan<RealType> aDist)
        => (Math.Abs(aDist[0]) > aHSize[0] || Math.Abs(aDist[1]) > aHSize[1]);

    private static bool compareDistD(in gp_XY aHSize, in gp_XY aDist)
        => (Math.Abs(aDist.X()) > aHSize.X() || Math.Abs(aDist.Y()) > aHSize.Y());

    // Exact equality, for use as a value

    public readonly bool Equals(Bnd_B2d other)
        => myCenter[0].Equals(other.myCenter[0]) && myCenter[1].Equals(other.myCenter[1])
           && myHSize[0].Equals(other.myHSize[0]) && myHSize[1].Equals(other.myHSize[1]);

    public override readonly bool Equals(object? obj) => obj is Bnd_B2d other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myCenter[0], myCenter[1], myHSize[0], myHSize[1]);

    public static bool operator ==(in Bnd_B2d theLeft, in Bnd_B2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Bnd_B2d theLeft, in Bnd_B2d theRight) => !theLeft.Equals(theRight);

    /// <summary>The center and the half-diagonal.</summary>
    public override readonly string ToString() => gp.Format("Bnd_B2d", myCenter[0], myCenter[1], myHSize[0], myHSize[1]);
}
