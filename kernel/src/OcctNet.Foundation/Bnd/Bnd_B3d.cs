// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_B3.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2005-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// OCCT's template Bnd_B3<RealType> is ported once per instantiation (Bnd_B3d, Bnd_B3f) from the same text, with
// RealType a file-level alias, so C++'s float/double promotions are kept exactly (see docs/decisions.md).

using System.Runtime.CompilerServices;
using static Occt.gp_TrsfForm;
using RealType = double;

namespace Occt;

/// <summary>
/// A 3D axis-aligned bounding box stored as a center and a half-diagonal, in double precision (OCCT's
/// <c>Bnd_B3&lt;double&gt;</c>).
/// </summary>
/// <remarks>
/// OCCT's default (and <c>new Bnd_B3d()</c>) is a void box. <c>default(Bnd_B3d)</c> is not: it is a box reduced to
/// the origin. Ported code must write <c>new Bnd_B3d()</c>.
/// </remarks>
public partial struct Bnd_B3d : IEquatable<Bnd_B3d>
{
    /// <summary>Three reals, as OCCT's <c>std::array&lt;RealType, 3&gt;</c>.</summary>
    [InlineArray(3)]
    public struct Array3
    {
        private RealType myElement0;

        public Array3(RealType the0, RealType the1, RealType the2)
        {
            this[0] = the0;
            this[1] = the1;
            this[2] = the2;
        }
    }

    // Constant representing a very large value for void box initialization
    private const RealType THE_RealLast = (RealType)1e30;

    private Array3 myCenter;
    private Array3 myHSize;

    /// <summary>Creates a void box.</summary>
    public Bnd_B3d()
    {
        myCenter = new Array3(THE_RealLast, THE_RealLast, THE_RealLast);
        myHSize = new Array3(-THE_RealLast, -THE_RealLast, -THE_RealLast);
    }

    public Bnd_B3d(in gp_XYZ theCenter, in gp_XYZ theHSize)
    {
        myCenter = new Array3((RealType)theCenter.X(), (RealType)theCenter.Y(), (RealType)theCenter.Z());
        myHSize = new Array3((RealType)theHSize.X(), (RealType)theHSize.Y(), (RealType)theHSize.Z());
    }

    public Bnd_B3d(in Array3 theCenter, in Array3 theHSize)
    {
        myCenter = theCenter;
        myHSize = theHSize;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Clear()
    {
        myCenter[0] = THE_RealLast;
        myCenter[1] = THE_RealLast;
        myCenter[2] = THE_RealLast;
        myHSize[0] = -THE_RealLast;
        myHSize[1] = -THE_RealLast;
        myHSize[2] = -THE_RealLast;
    }

    internal void Add(in gp_XYZ thePnt)
    {
        if (IsVoid())
        {
            myCenter[0] = (RealType)thePnt.X();
            myCenter[1] = (RealType)thePnt.Y();
            myCenter[2] = (RealType)thePnt.Z();
            myHSize[0] = (RealType)0.0;
            myHSize[1] = (RealType)0.0;
            myHSize[2] = (RealType)0.0;
        }
        else
        {
            Span<RealType> aDiff =
            [
                (RealType)thePnt.X() - myCenter[0],
                (RealType)thePnt.Y() - myCenter[1],
                (RealType)thePnt.Z() - myCenter[2],
            ];
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

            if (aDiff[2] > myHSize[2])
            {
                RealType aShift = (aDiff[2] - myHSize[2]) / 2;
                myCenter[2] += aShift;
                myHSize[2] += aShift;
            }
            else if (aDiff[2] < -myHSize[2])
            {
                RealType aShift = (aDiff[2] + myHSize[2]) / 2;
                myCenter[2] += aShift;
                myHSize[2] -= aShift;
            }
        }
    }

    internal void Add(in gp_Pnt thePnt) => Add(thePnt.XYZ());

    internal void Add(in Bnd_B3d theBox)
    {
        if (!theBox.IsVoid())
        {
            Add(theBox.CornerMin());
            Add(theBox.CornerMax());
        }
    }

    internal void Enlarge(double aDiff)
    {
        double aD = Math.Abs(aDiff);
        myHSize[0] += (RealType)aD;
        myHSize[1] += (RealType)aD;
        myHSize[2] += (RealType)aD;
    }

    /// <summary>Limits the box by the inside of <paramref name="theBox"/>; false if the boxes do not intersect.</summary>
    internal bool Limit(in Bnd_B3d theBox)
    {
        bool aResult = false;
        Span<RealType> diffC =
        [
            theBox.myCenter[0] - myCenter[0],
            theBox.myCenter[1] - myCenter[1],
            theBox.myCenter[2] - myCenter[2],
        ];
        Span<RealType> sumH =
        [
            theBox.myHSize[0] + myHSize[0],
            theBox.myHSize[1] + myHSize[1],
            theBox.myHSize[2] + myHSize[2],
        ];
        // check the condition IsOut
        if (!compareDist(sumH, diffC))
        {
            Span<RealType> diffH =
            [
                theBox.myHSize[0] - myHSize[0],
                theBox.myHSize[1] - myHSize[1],
                theBox.myHSize[2] - myHSize[2],
            ];
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

            if (diffC[2] - diffH[2] > 0.0)
            {
                RealType aShift = (diffC[2] - diffH[2]) / 2; // positive
                myCenter[2] += aShift;
                myHSize[2] -= aShift;
            }
            else if (diffC[2] + diffH[2] < 0.0)
            {
                RealType aShift = (diffC[2] + diffH[2]) / 2; // negative
                myCenter[2] += aShift;
                myHSize[2] += aShift;
            }

            aResult = true;
        }

        return aResult;
    }

    internal void SetCenter(in gp_XYZ theCenter)
    {
        myCenter[0] = (RealType)theCenter.X();
        myCenter[1] = (RealType)theCenter.Y();
        myCenter[2] = (RealType)theCenter.Z();
    }

    internal void SetCenter(in Array3 theCenter) => myCenter = theCenter;

    internal void SetHSize(in gp_XYZ theHSize)
    {
        myHSize[0] = (RealType)theHSize.X();
        myHSize[1] = (RealType)theHSize.Y();
        myHSize[2] = (RealType)theHSize.Z();
    }

    internal void SetHSize(in Array3 theHSize) => myHSize = theHSize;

    // Copies with a mutator applied (the public way to "set")

    public readonly Bnd_B3d Cleared() => new();

    public readonly Bnd_B3d Added(in gp_XYZ thePnt)
    {
        Bnd_B3d aCopy = this;
        aCopy.Add(thePnt);
        return aCopy;
    }

    public readonly Bnd_B3d Added(in gp_Pnt thePnt)
    {
        Bnd_B3d aCopy = this;
        aCopy.Add(thePnt);
        return aCopy;
    }

    public readonly Bnd_B3d Added(in Bnd_B3d theBox)
    {
        Bnd_B3d aCopy = this;
        aCopy.Add(theBox);
        return aCopy;
    }

    public readonly Bnd_B3d Enlarged(double theDiff)
    {
        Bnd_B3d aCopy = this;
        aCopy.Enlarge(theDiff);
        return aCopy;
    }

    /// <summary>This box limited by the inside of <paramref name="theBox"/>; unchanged if they do not intersect (<see cref="IsOut(in Bnd_B3d)"/>).</summary>
    public readonly Bnd_B3d Limited(in Bnd_B3d theBox)
    {
        Bnd_B3d aCopy = this;
        aCopy.Limit(theBox);
        return aCopy;
    }

    public readonly Bnd_B3d WithCenter(in gp_XYZ theCenter)
    {
        Bnd_B3d aCopy = this;
        aCopy.SetCenter(theCenter);
        return aCopy;
    }

    public readonly Bnd_B3d WithCenter(in Array3 theCenter)
    {
        Bnd_B3d aCopy = this;
        aCopy.SetCenter(theCenter);
        return aCopy;
    }

    public readonly Bnd_B3d WithHSize(in gp_XYZ theHSize)
    {
        Bnd_B3d aCopy = this;
        aCopy.SetHSize(theHSize);
        return aCopy;
    }

    public readonly Bnd_B3d WithHSize(in Array3 theHSize)
    {
        Bnd_B3d aCopy = this;
        aCopy.SetHSize(theHSize);
        return aCopy;
    }

    // Queries

    /// <summary>True if the box is void (non-initialized).</summary>
    public readonly bool IsVoid() => (myHSize[0] < -1e-5);

    /// <summary>Center - HSize. Irrelevant for a void box.</summary>
    public readonly gp_XYZ CornerMin()
        => new(myCenter[0] - myHSize[0], myCenter[1] - myHSize[1], myCenter[2] - myHSize[2]);

    /// <summary>Center + HSize. Irrelevant for a void box.</summary>
    public readonly gp_XYZ CornerMax()
        => new(myCenter[0] + myHSize[0], myCenter[1] + myHSize[1], myCenter[2] + myHSize[2]);

    /// <summary>The square diagonal; very big for a void box.</summary>
    public readonly double SquareExtent()
        => 4 * (myHSize[0] * myHSize[0] + myHSize[1] * myHSize[1] + myHSize[2] * myHSize[2]);

    public readonly Array3 Center() => myCenter;

    public readonly Array3 HSize() => myHSize;

    /// <summary>The box bounding this box transformed by <paramref name="theTrsf"/> (larger if it rotates).</summary>
    public readonly Bnd_B3d Transformed(in gp_Trsf theTrsf)
    {
        Bnd_B3d aResult = new();
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
            aResult.myCenter[2] = (RealType)(myCenter[2] * aScale + theTrsf.TranslationPart().Z());
            aResult.myHSize[0] = (RealType)(myHSize[0] * aScaleAbs);
            aResult.myHSize[1] = (RealType)(myHSize[1] * aScaleAbs);
            aResult.myHSize[2] = (RealType)(myHSize[2] * aScaleAbs);
        }
        else
        {
            gp_XYZ aCenter = new((double)myCenter[0], (double)myCenter[1], (double)myCenter[2]);
            theTrsf.Transforms(ref aCenter);
            aResult.myCenter[0] = (RealType)aCenter.X();
            aResult.myCenter[1] = (RealType)aCenter.Y();
            aResult.myCenter[2] = (RealType)aCenter.Z();
            Span<double> aMat = stackalloc double[9];
            theTrsf.HVectorialPart().CopyTo(aMat);
            aResult.myHSize[0] =
                (RealType)(aScaleAbs
                           * (Math.Abs(aMat[0]) * myHSize[0] + Math.Abs(aMat[1]) * myHSize[1]
                              + Math.Abs(aMat[2]) * myHSize[2]));
            aResult.myHSize[1] =
                (RealType)(aScaleAbs
                           * (Math.Abs(aMat[3]) * myHSize[0] + Math.Abs(aMat[4]) * myHSize[1]
                              + Math.Abs(aMat[5]) * myHSize[2]));
            aResult.myHSize[2] =
                (RealType)(aScaleAbs
                           * (Math.Abs(aMat[6]) * myHSize[0] + Math.Abs(aMat[7]) * myHSize[1]
                              + Math.Abs(aMat[8]) * myHSize[2]));
        }

        return aResult;
    }

    /// <summary>True if the point is outside the box.</summary>
    public readonly bool IsOut(in gp_XYZ thePnt)
        => (Math.Abs((RealType)thePnt.X() - myCenter[0]) > myHSize[0]
            || Math.Abs((RealType)thePnt.Y() - myCenter[1]) > myHSize[1]
            || Math.Abs((RealType)thePnt.Z() - myCenter[2]) > myHSize[2]);

    /// <summary>True if the sphere (or its surface, if hollow) does not intersect the box.</summary>
    public readonly bool IsOut(in gp_XYZ theCenter, double theRadius, bool isSphereHollow = false)
    {
        bool aResult = true;
        if (!isSphereHollow)
        {
            // vector from the center of the sphere to the nearest box face
            Span<double> aDist =
            [
                Math.Abs(theCenter.X() - (double)myCenter[0]) - (double)myHSize[0],
                Math.Abs(theCenter.Y() - (double)myCenter[1]) - (double)myHSize[1],
                Math.Abs(theCenter.Z() - (double)myCenter[2]) - (double)myHSize[2],
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

            if (aDist[2] > 0.0)
            {
                aD += aDist[2] * aDist[2];
            }

            aResult = (aD > theRadius * theRadius);
        }
        else
        {
            Span<double> aDistC =
            [
                Math.Abs(theCenter.X() - (double)myCenter[0]),
                Math.Abs(theCenter.Y() - (double)myCenter[1]),
                Math.Abs(theCenter.Z() - (double)myCenter[2]),
            ];
            // vector from the center of the sphere to the nearest box face
            Span<double> aDist =
            [
                aDistC[0] - (double)myHSize[0],
                aDistC[1] - (double)myHSize[1],
                aDistC[2] - (double)myHSize[2],
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

            if (aDist[2] > 0.0)
            {
                aD += aDist[2] * aDist[2];
            }

            if (aD < theRadius * theRadius)
            {
                // the box intersects the solid sphere; check if it is completely
                // inside the circle (in such case return isOut==True)
                aDist[0] = aDistC[0] + (double)myHSize[0];
                aDist[1] = aDistC[1] + (double)myHSize[1];
                aDist[2] = aDistC[2] + (double)myHSize[2];
                if (aDist[0] * aDist[0] + aDist[1] * aDist[1] + aDist[2] * aDist[2] > theRadius * theRadius)
                {
                    aResult = false;
                }
            }
        }

        return aResult;
    }

    /// <summary>True if <paramref name="theBox"/> does not intersect this box.</summary>
    public readonly bool IsOut(in Bnd_B3d theBox)
        => (Math.Abs(theBox.myCenter[0] - myCenter[0]) > theBox.myHSize[0] + myHSize[0]
            || Math.Abs(theBox.myCenter[1] - myCenter[1]) > theBox.myHSize[1] + myHSize[1]
            || Math.Abs(theBox.myCenter[2] - myCenter[2]) > theBox.myHSize[2] + myHSize[2]);

    /// <summary>True if <paramref name="theBox"/> transformed by <paramref name="theTrsf"/> does not intersect this box.</summary>
    public readonly bool IsOut(in Bnd_B3d theBox, in gp_Trsf theTrsf)
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
                     > (RealType)(theBox.myHSize[1] * aScaleAbs) + myHSize[1]
                 || Math.Abs((RealType)(theBox.myCenter[2] * aScale + theTrsf.TranslationPart().Z()) - myCenter[2])
                     > (RealType)(theBox.myHSize[2] * aScaleAbs) + myHSize[2]);
        }
        else
        {
            // theBox is transformed and we check the resulting (enlarged) box against
            // 'this' box.
            Span<double> aMat = stackalloc double[9];
            theTrsf.HVectorialPart().CopyTo(aMat);
            gp_XYZ aCenter = new((double)theBox.myCenter[0], (double)theBox.myCenter[1], (double)theBox.myCenter[2]);
            theTrsf.Transforms(ref aCenter);
            Span<double> aDist =
            [
                aCenter.X() - (double)myCenter[0],
                aCenter.Y() - (double)myCenter[1],
                aCenter.Z() - (double)myCenter[2],
            ];
            Span<double> aMatAbs =
            [
                Math.Abs(aMat[0]), Math.Abs(aMat[1]), Math.Abs(aMat[2]),
                Math.Abs(aMat[3]), Math.Abs(aMat[4]), Math.Abs(aMat[5]),
                Math.Abs(aMat[6]), Math.Abs(aMat[7]), Math.Abs(aMat[8]),
            ];
            if (Math.Abs(aDist[0]) > (aScaleAbs
                                          * (aMatAbs[0] * theBox.myHSize[0] + aMatAbs[1] * theBox.myHSize[1]
                                             + aMatAbs[2] * theBox.myHSize[2])
                                      + (double)myHSize[0])
                || Math.Abs(aDist[1]) > (aScaleAbs
                                             * (aMatAbs[3] * theBox.myHSize[0] + aMatAbs[4] * theBox.myHSize[1]
                                                + aMatAbs[5] * theBox.myHSize[2])
                                         + (double)myHSize[1])
                || Math.Abs(aDist[2]) > (aScaleAbs
                                             * (aMatAbs[6] * theBox.myHSize[0] + aMatAbs[7] * theBox.myHSize[1]
                                                + aMatAbs[8] * theBox.myHSize[2])
                                         + (double)myHSize[2]))
            {
                aResult = true;
            }
            else
            {
                // theBox is rotated, scaled and translated. We apply the reverse
                // translation and scaling then check against the rotated box 'this'
                if ((Math.Abs(aMat[0] * aDist[0] + aMat[3] * aDist[1] + aMat[6] * aDist[2])
                     > theBox.myHSize[0] * aScaleAbs
                         + (aMatAbs[0] * myHSize[0] + aMatAbs[3] * myHSize[1] + aMatAbs[6] * myHSize[2]))
                    || (Math.Abs(aMat[1] * aDist[0] + aMat[4] * aDist[1] + aMat[7] * aDist[2])
                        > theBox.myHSize[1] * aScaleAbs
                            + (aMatAbs[1] * myHSize[0] + aMatAbs[4] * myHSize[1] + aMatAbs[7] * myHSize[2]))
                    || (Math.Abs(aMat[2] * aDist[0] + aMat[5] * aDist[1] + aMat[8] * aDist[2])
                        > theBox.myHSize[2] * aScaleAbs
                            + (aMatAbs[2] * myHSize[0] + aMatAbs[5] * myHSize[1] + aMatAbs[8] * myHSize[2])))
                {
                    aResult = true;
                }
            }
        }

        return aResult;
    }

    /// <summary>
    /// True if the line (or the ray, if <paramref name="isRay"/>) does not intersect the box enlarged by
    /// <paramref name="theOverthickness"/>.
    /// </summary>
    public readonly bool IsOut(in gp_Ax1 theLine, bool isRay = false, double theOverthickness = 0.0)
    {
        double aRes = gp.Resolution() * 100.0;
        if (IsVoid())
        {
            return true;
        }

        Span<double> anInter0 = [-RealLast(), RealLast()];
        Span<double> anInter1 = [-RealLast(), RealLast()];
        gp_XYZ aDir = theLine.Direction().XYZ();
        gp_XYZ aDiff = new((double)myCenter[0] - theLine.Location().X(),
                           (double)myCenter[1] - theLine.Location().Y(),
                           (double)myCenter[2] - theLine.Location().Z());
        double aHSize = (double)myHSize[0] + theOverthickness;
        if (aDir.X() > aRes)
        {
            anInter0[0] = (aDiff.X() - aHSize) / aDir.X();
            anInter0[1] = (aDiff.X() + aHSize) / aDir.X();
        }
        else if (aDir.X() < -aRes)
        {
            anInter0[0] = (aDiff.X() + aHSize) / aDir.X();
            anInter0[1] = (aDiff.X() - aHSize) / aDir.X();
        }
        else if (Math.Abs(aDiff.X()) > aHSize)
        {
            return true;
        }

        aHSize = (double)myHSize[1] + theOverthickness;
        if (aDir.Y() > aRes)
        {
            anInter1[0] = (aDiff.Y() - aHSize) / aDir.Y();
            anInter1[1] = (aDiff.Y() + aHSize) / aDir.Y();
        }
        else if (aDir.Y() < -aRes)
        {
            anInter1[0] = (aDiff.Y() + aHSize) / aDir.Y();
            anInter1[1] = (aDiff.Y() - aHSize) / aDir.Y();
        }
        else if (Math.Abs(aDiff.Y()) > aHSize)
        {
            return true;
        }

        if (anInter0[0] > (anInter1[1] + aRes) || anInter0[1] < (anInter1[0] - aRes))
        {
            return true;
        }

        if (anInter1[0] > anInter0[0])
        {
            anInter0[0] = anInter1[0];
        }

        if (anInter1[1] < anInter0[1])
        {
            anInter0[1] = anInter1[1];
        }

        if (isRay && anInter0[1] < -aRes)
        {
            return true;
        }

        aHSize = (double)myHSize[2] + theOverthickness;
        if (aDir.Z() > aRes)
        {
            anInter1[0] = (aDiff.Z() - aHSize) / aDir.Z();
            anInter1[1] = (aDiff.Z() + aHSize) / aDir.Z();
        }
        else if (aDir.Z() < -aRes)
        {
            anInter1[0] = (aDiff.Z() + aHSize) / aDir.Z();
            anInter1[1] = (aDiff.Z() - aHSize) / aDir.Z();
        }
        else
        {
            return (Math.Abs(aDiff.Z()) > aHSize);
        }

        if (isRay && anInter1[1] < -aRes)
        {
            return true;
        }

        return (anInter0[0] > (anInter1[1] + aRes) || anInter0[1] < (anInter1[0] - aRes));
    }

    /// <summary>True if the plane does not intersect the box.</summary>
    public readonly bool IsOut(in gp_Ax3 thePlane)
    {
        if (IsVoid())
        {
            return true;
        }

        gp_XYZ anOrigin = thePlane.Location().XYZ();
        gp_XYZ aDir = thePlane.Direction().XYZ();
        gp_XYZ aBoxCenter = new((double)myCenter[0], (double)myCenter[1], (double)myCenter[2]);
        double aDist0 = (aBoxCenter - anOrigin) * aDir;
        double aDist1 = // proj of HSize on aDir
            (double)myHSize[0] * Math.Abs(aDir.X()) + (double)myHSize[1] * Math.Abs(aDir.Y())
            + (double)myHSize[2] * Math.Abs(aDir.Z());
        return ((aDist0 + aDist1) * (aDist0 - aDist1) > 0.0);
    }

    /// <summary>True if this box is fully inside <paramref name="theBox"/>.</summary>
    public readonly bool IsIn(in Bnd_B3d theBox)
        => (Math.Abs(theBox.myCenter[0] - myCenter[0]) < theBox.myHSize[0] - myHSize[0]
            && Math.Abs(theBox.myCenter[1] - myCenter[1]) < theBox.myHSize[1] - myHSize[1]
            && Math.Abs(theBox.myCenter[2] - myCenter[2]) < theBox.myHSize[2] - myHSize[2]);

    /// <summary>True if this box is fully inside <paramref name="theBox"/> transformed by <paramref name="theTrsf"/>.</summary>
    public readonly bool IsIn(in Bnd_B3d theBox, in gp_Trsf theTrsf)
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
                     < (RealType)(theBox.myHSize[1] * aScaleAbs) - myHSize[1]
                 && Math.Abs((RealType)(theBox.myCenter[2] * aScale + theTrsf.TranslationPart().Z()) - myCenter[2])
                     < (RealType)(theBox.myHSize[2] * aScaleAbs) - myHSize[2]);
        }
        else
        {
            // theBox is rotated, scaled and translated. We apply the reverse
            // translation and scaling then check against the rotated box 'this'
            Span<double> aMat = stackalloc double[9];
            theTrsf.HVectorialPart().CopyTo(aMat);
            gp_XYZ aCenter = new((double)theBox.myCenter[0], (double)theBox.myCenter[1], (double)theBox.myCenter[2]);
            theTrsf.Transforms(ref aCenter);
            Span<double> aDist =
            [
                aCenter.X() - (double)myCenter[0],
                aCenter.Y() - (double)myCenter[1],
                aCenter.Z() - (double)myCenter[2],
            ];
            if ((Math.Abs(aMat[0] * aDist[0] + aMat[3] * aDist[1] + aMat[6] * aDist[2])
                 < theBox.myHSize[0] * aScaleAbs
                     - (Math.Abs(aMat[0]) * myHSize[0] + Math.Abs(aMat[3]) * myHSize[1] + Math.Abs(aMat[6]) * myHSize[2]))
                && (Math.Abs(aMat[1] * aDist[0] + aMat[4] * aDist[1] + aMat[7] * aDist[2])
                    < theBox.myHSize[1] * aScaleAbs
                        - (Math.Abs(aMat[1]) * myHSize[0] + Math.Abs(aMat[4]) * myHSize[1] + Math.Abs(aMat[7]) * myHSize[2]))
                && (Math.Abs(aMat[2] * aDist[0] + aMat[5] * aDist[1] + aMat[8] * aDist[2])
                    < theBox.myHSize[2] * aScaleAbs
                        - (Math.Abs(aMat[2]) * myHSize[0] + Math.Abs(aMat[5]) * myHSize[1] + Math.Abs(aMat[8]) * myHSize[2])))
            {
                aResult = true;
            }
        }

        return aResult;
    }

    private static bool compareDist(ReadOnlySpan<RealType> aHSize, ReadOnlySpan<RealType> aDist)
        => (Math.Abs(aDist[0]) > aHSize[0] || Math.Abs(aDist[1]) > aHSize[1] || Math.Abs(aDist[2]) > aHSize[2]);

    private static bool compareDistD(in gp_XYZ aHSize, in gp_XYZ aDist)
        => (Math.Abs(aDist.X()) > aHSize.X() || Math.Abs(aDist.Y()) > aHSize.Y() || Math.Abs(aDist.Z()) > aHSize.Z());

    // Exact equality, for use as a value

    public readonly bool Equals(Bnd_B3d other)
        => myCenter[0].Equals(other.myCenter[0]) && myCenter[1].Equals(other.myCenter[1])
           && myCenter[2].Equals(other.myCenter[2]) && myHSize[0].Equals(other.myHSize[0])
           && myHSize[1].Equals(other.myHSize[1]) && myHSize[2].Equals(other.myHSize[2]);

    public override readonly bool Equals(object? obj) => obj is Bnd_B3d other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(myCenter[0], myCenter[1], myCenter[2], myHSize[0], myHSize[1], myHSize[2]);

    public static bool operator ==(in Bnd_B3d theLeft, in Bnd_B3d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Bnd_B3d theLeft, in Bnd_B3d theRight) => !theLeft.Equals(theRight);

    /// <summary>The center and the half-diagonal.</summary>
    public override readonly string ToString()
        => gp.Format("Bnd_B3d", myCenter[0], myCenter[1], myCenter[2], myHSize[0], myHSize[1], myHSize[2]);
}
