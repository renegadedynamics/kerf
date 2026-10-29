// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_OBB.hxx
// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_OBB.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// An oriented bounding box (OBB): a centre, three axes and the halves of the three dimensions. Much tighter than an
/// axis-aligned box (AABB), so a more effective rejection test for non-interfering objects.
/// </summary>
/// <remarks>
/// OCCT's default constructor makes a void box (half-sizes -1, axes zero), and so does <c>new Bnd_OBB()</c>;
/// <c>default(Bnd_OBB)</c> has zero half-sizes and is not void. DumpJson is not ported (ToString replaces it).
/// </remarks>
public partial struct Bnd_OBB : IEquatable<Bnd_OBB>
{
    /// <summary>The half-size dimensions, for deconstruction: <c>var (aHX, aHY, aHZ) = anOBB.GetHalfSizes();</c></summary>
    public readonly record struct HalfSizes(double X, double Y, double Z);

    [InlineArray(3)]
    private struct Axes3
    {
        private gp_XYZ myElement0;
    }

    [InlineArray(3)]
    private struct Dims3
    {
        private double myElement0;
    }

    // Precomputed sqrt(3)
    private const double SQRT_3 = 1.7320508075688772935;

    private gp_XYZ myCenter;   //!< Center of the OBB
    private Axes3 myAxes;      //!< Directions of the box's axes (all vectors are already normalized)
    private Dims3 myHDims;     //!< Half-size dimensions of the OBB
    private bool myIsAABox;    //!< To be set if the OBB is axis aligned box

    /// <summary>A void box.</summary>
    public Bnd_OBB()
    {
        myIsAABox = false;
        myHDims[0] = myHDims[1] = myHDims[2] = -1.0;
    }

    /// <summary>A box from all its defining parameters.</summary>
    public Bnd_OBB(in gp_Pnt theCenter,
                   in gp_Dir theXDirection,
                   in gp_Dir theYDirection,
                   in gp_Dir theZDirection,
                   double theHXSize,
                   double theHYSize,
                   double theHZSize)
    {
        myCenter = theCenter.XYZ();
        myIsAABox = false;
        myAxes[0] = theXDirection.XYZ();
        myAxes[1] = theYDirection.XYZ();
        myAxes[2] = theZDirection.XYZ();

        // Standard_ASSERT_VOID on negative sizes: a debug-only report in OCCT, nothing here
        myHDims[0] = theHXSize;
        myHDims[1] = theHYSize;
        myHDims[2] = theHZSize;
    }

    /// <summary>A box from an axis-aligned box (void if that is void).</summary>
    public Bnd_OBB(in Bnd_Box theBox)
    {
        myIsAABox = true;
        if (theBox.IsVoid())
        {
            myHDims[0] = myHDims[1] = myHDims[2] = -1.0;
            myIsAABox = false;
            return;
        }

        theBox.Get(out double aX1, out double aY1, out double aZ1, out double aX2, out double aY2, out double aZ2);

        myAxes[0] = new gp_XYZ(1.0, 0.0, 0.0); // SetCoord (the analyzer treats an inline array element as a copy)
        myAxes[1] = new gp_XYZ(0.0, 1.0, 0.0); // SetCoord (the analyzer treats an inline array element as a copy)
        myAxes[2] = new gp_XYZ(0.0, 0.0, 1.0); // SetCoord (the analyzer treats an inline array element as a copy)

        myHDims[0] = 0.5 * (aX2 - aX1);
        myHDims[1] = 0.5 * (aY2 - aY1);
        myHDims[2] = 0.5 * (aZ2 - aZ1);

        myCenter.SetCoord(0.5 * (aX2 + aX1), 0.5 * (aY2 + aY1), 0.5 * (aZ2 + aZ1));
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>
    /// Rebuilds the box to cover every point of the list. If tolerances are given (one per point), the box is enlarged
    /// by the tolerances of the points on its surface. The optimal mode checks more axes: slower, usually tighter.
    /// </summary>
    internal void ReBuild(NCollection_Array1<gp_Pnt> theListOfPoints,
                          NCollection_Array1<double>? theListOfTolerances = null,
                          bool theIsOptimal = false)
    {
        switch (theListOfPoints.Length())
        {
            case 1:
                ProcessOnePoint(theListOfPoints.First());
                if (theListOfTolerances is not null)
                {
                    Enlarge(theListOfTolerances.First());
                }

                return;
            case 2:
            {
                double aTol1 = (theListOfTolerances is null) ? 0.0 : theListOfTolerances.First();
                double aTol2 = (theListOfTolerances is null) ? 0.0 : theListOfTolerances.Last();
                gp_XYZ aP1 = theListOfPoints.First().XYZ(), aP2 = theListOfPoints.Last().XYZ();
                gp_XYZ aDP = aP2 - aP1;
                double aDPm = aDP.Modulus();
                myIsAABox = false;
                myHDims[1] = myHDims[2] = Max(aTol1, aTol2);

                if (aDPm < Precision.Confusion())
                {
                    ProcessOnePoint(new gp_Pnt(aP1));
                    // OCCT: ProcessOnePoint has just reset myHDims[1] to zero, so the tolerances are dropped
                    Enlarge(myHDims[1] + Precision.Confusion());
                    return;
                }

                myHDims[0] = 0.5 * (aDPm + aTol1 + aTol2);
                myAxes[0] = aDP / aDPm;
                if (Math.Abs(myAxes[0].X()) > Math.Abs(myAxes[0].Y()))
                {
                    // Z-coord. is maximal or X-coord. is maximal
                    myAxes[1] = new gp_XYZ(-myAxes[0].Z(), 0.0, myAxes[0].X()); // SetCoord (the analyzer treats an inline array element as a copy)
                }
                else
                {
                    // Z-coord. is maximal or Y-coord. is maximal
                    myAxes[1] = new gp_XYZ(0.0, -myAxes[0].Z(), myAxes[0].Y()); // SetCoord (the analyzer treats an inline array element as a copy)
                }

                myAxes[2] = myAxes[0].Crossed(myAxes[1]).Normalized();
                myCenter = aP1 + 0.5 * (aDPm - aTol1 + aTol2) * myAxes[0];
                return;
            }
            default:
                break;
        }

        OBBTool aTool = new(theListOfPoints, theListOfTolerances, theIsOptimal);
        aTool.ProcessDiTetrahedron();
        aTool.BuildBox(ref this);
    }

    internal void SetCenter(in gp_Pnt theCenter) => myCenter = theCenter.XYZ();

    /// <summary>Sets the X direction and half-size.</summary>
    internal void SetXComponent(in gp_Dir theXDirection, double theHXSize)
    {
        myAxes[0] = theXDirection.XYZ();
        myHDims[0] = theHXSize;
    }

    /// <summary>Sets the Y direction and half-size.</summary>
    internal void SetYComponent(in gp_Dir theYDirection, double theHYSize)
    {
        myAxes[1] = theYDirection.XYZ();
        myHDims[1] = theHYSize;
    }

    /// <summary>Sets the Z direction and half-size.</summary>
    internal void SetZComponent(in gp_Dir theZDirection, double theHZSize)
    {
        myAxes[2] = theZDirection.XYZ();
        myHDims[2] = theHZSize;
    }

    /// <summary>Clears this box.</summary>
    internal void SetVoid()
    {
        myHDims[0] = myHDims[1] = myHDims[2] = -1.0;
        myCenter = myAxes[0] = myAxes[1] = myAxes[2] = new gp_XYZ();
        myIsAABox = false;
    }

    /// <summary>Sets the flag for an axis-aligned box.</summary>
    internal void SetAABox(bool theFlag) => myIsAABox = theFlag;

    /// <summary>Enlarges the box by the absolute value of the gap.</summary>
    internal void Enlarge(double theGapAdd)
    {
        double aGap = Math.Abs(theGapAdd);
        myHDims[0] += aGap;
        myHDims[1] += aGap;
        myHDims[2] += aGap;
    }

    /// <summary>Rebuilds this box to include all the objects it was made from and the other box.</summary>
    internal void Add(in Bnd_OBB theOther)
    {
        if (!theOther.IsVoid())
        {
            if (IsVoid())
            {
                myCenter = theOther.myCenter;
                myAxes[0] = theOther.myAxes[0];
                myAxes[1] = theOther.myAxes[1];
                myAxes[2] = theOther.myAxes[2];
                myHDims[0] = theOther.myHDims[0];
                myHDims[1] = theOther.myHDims[1];
                myHDims[2] = theOther.myHDims[2];
                myIsAABox = theOther.myIsAABox;
            }
            else
            {
                gp_Pnt[] aList = new gp_Pnt[16];
                GetVertex(aList.AsSpan(0, 8));
                theOther.GetVertex(aList.AsSpan(8, 8));
                ReBuild(new NCollection_Array1<gp_Pnt>(aList, 0, 0, 15));
            }
        }
    }

    /// <summary>Rebuilds this box to include all the objects it was made from and the point.</summary>
    internal void Add(in gp_Pnt theP)
    {
        if (IsVoid())
        {
            myCenter = theP.XYZ();
            myAxes[0] = gp.DX().XYZ();
            myAxes[1] = gp.DY().XYZ();
            myAxes[2] = gp.DZ().XYZ();
            myHDims[0] = 0.0;
            myHDims[1] = 0.0;
            myHDims[2] = 0.0;
            myIsAABox = true;
        }
        else
        {
            gp_Pnt[] aList = new gp_Pnt[9];
            GetVertex(aList.AsSpan(0, 8));
            aList[8] = theP;
            ReBuild(new NCollection_Array1<gp_Pnt>(aList, 0, 0, 8));
        }
    }

    private void ProcessOnePoint(in gp_Pnt theP)
    {
        myIsAABox = true;
        myHDims[0] = myHDims[1] = myHDims[2] = 0.0;
        myAxes[0] = new gp_XYZ(1.0, 0.0, 0.0); // SetCoord (the analyzer treats an inline array element as a copy)
        myAxes[1] = new gp_XYZ(0.0, 1.0, 0.0); // SetCoord (the analyzer treats an inline array element as a copy)
        myAxes[2] = new gp_XYZ(0.0, 0.0, 1.0); // SetCoord (the analyzer treats an inline array element as a copy)
        myCenter = theP.XYZ();
    }

    // Copies with a mutator applied, and factories

    /// <summary>The box covering the points (see <c>ReBuild</c>).</summary>
    public static Bnd_OBB FromPoints(NCollection_Array1<gp_Pnt> theListOfPoints,
                                     NCollection_Array1<double>? theListOfTolerances = null,
                                     bool theIsOptimal = false)
    {
        Bnd_OBB aBox = new();
        aBox.ReBuild(theListOfPoints, theListOfTolerances, theIsOptimal);
        return aBox;
    }

    /// <summary>A void box.</summary>
    public static Bnd_OBB Void() => new();

    /// <summary>This box rebuilt over the points (as <c>ReBuild</c>; the flags of this box can survive in the two-point case).</summary>
    public readonly Bnd_OBB ReBuilt(NCollection_Array1<gp_Pnt> theListOfPoints,
                                    NCollection_Array1<double>? theListOfTolerances = null,
                                    bool theIsOptimal = false)
    {
        Bnd_OBB aCopy = this;
        aCopy.ReBuild(theListOfPoints, theListOfTolerances, theIsOptimal);
        return aCopy;
    }

    public readonly Bnd_OBB WithCenter(in gp_Pnt theCenter)
    {
        Bnd_OBB aCopy = this;
        aCopy.SetCenter(theCenter);
        return aCopy;
    }

    public readonly Bnd_OBB WithXComponent(in gp_Dir theXDirection, double theHXSize)
    {
        Bnd_OBB aCopy = this;
        aCopy.SetXComponent(theXDirection, theHXSize);
        return aCopy;
    }

    public readonly Bnd_OBB WithYComponent(in gp_Dir theYDirection, double theHYSize)
    {
        Bnd_OBB aCopy = this;
        aCopy.SetYComponent(theYDirection, theHYSize);
        return aCopy;
    }

    public readonly Bnd_OBB WithZComponent(in gp_Dir theZDirection, double theHZSize)
    {
        Bnd_OBB aCopy = this;
        aCopy.SetZComponent(theZDirection, theHZSize);
        return aCopy;
    }

    public readonly Bnd_OBB WithVoid() => new();

    public readonly Bnd_OBB WithAABox(bool theFlag)
    {
        Bnd_OBB aCopy = this;
        aCopy.SetAABox(theFlag);
        return aCopy;
    }

    public readonly Bnd_OBB Enlarged(double theGapAdd)
    {
        Bnd_OBB aCopy = this;
        aCopy.Enlarge(theGapAdd);
        return aCopy;
    }

    public readonly Bnd_OBB Added(in Bnd_OBB theOther)
    {
        Bnd_OBB aCopy = this;
        aCopy.Add(theOther);
        return aCopy;
    }

    public readonly Bnd_OBB Added(in gp_Pnt theP)
    {
        Bnd_OBB aCopy = this;
        aCopy.Add(theP);
        return aCopy;
    }

    // Queries

    /// <summary>
    /// The local coordinate system of the box: applied to the axis-aligned box (-XHSize, -YHSize, -ZHSize) to
    /// (XHSize, YHSize, ZHSize), it gives this box.
    /// </summary>
    public readonly gp_Ax3 Position() => new(new gp_Pnt(myCenter), new gp_Dir(ZDirection()), new gp_Dir(XDirection()));

    public readonly gp_XYZ Center() => myCenter;

    public readonly gp_XYZ XDirection() => myAxes[0];

    public readonly gp_XYZ YDirection() => myAxes[1];

    public readonly gp_XYZ ZDirection() => myAxes[2];

    public readonly double XHSize() => myHDims[0];

    public readonly double YHSize() => myHDims[1];

    public readonly double ZHSize() => myHDims[2];

    public readonly HalfSizes GetHalfSizes() => new(myHDims[0], myHDims[1], myHDims[2]);

    /// <summary>True if the box is empty (a negative half-size).</summary>
    public readonly bool IsVoid() => (myHDims[0] < 0.0) || (myHDims[1] < 0.0) || (myHDims[2] < 0.0);

    /// <summary>True if the box is axis-aligned.</summary>
    public readonly bool IsAABox() => myIsAABox;

    /// <summary>
    /// The eight vertices, into a span of at least 8 points; false (and nothing written) if the box is void. Vertex i
    /// has local coordinates (±XHSize, ±YHSize, ±ZHSize), with the sign of X from bit 0 of i, Y bit 1, Z bit 2.
    /// </summary>
    public readonly bool GetVertex(Span<gp_Pnt> theP)
    {
        if (IsVoid())
        {
            return false;
        }

        theP[0].SetXYZ(myCenter - myHDims[0] * myAxes[0] - myHDims[1] * myAxes[1] - myHDims[2] * myAxes[2]);
        theP[1].SetXYZ(myCenter + myHDims[0] * myAxes[0] - myHDims[1] * myAxes[1] - myHDims[2] * myAxes[2]);
        theP[2].SetXYZ(myCenter - myHDims[0] * myAxes[0] + myHDims[1] * myAxes[1] - myHDims[2] * myAxes[2]);
        theP[3].SetXYZ(myCenter + myHDims[0] * myAxes[0] + myHDims[1] * myAxes[1] - myHDims[2] * myAxes[2]);
        theP[4].SetXYZ(myCenter - myHDims[0] * myAxes[0] - myHDims[1] * myAxes[1] + myHDims[2] * myAxes[2]);
        theP[5].SetXYZ(myCenter + myHDims[0] * myAxes[0] - myHDims[1] * myAxes[1] + myHDims[2] * myAxes[2]);
        theP[6].SetXYZ(myCenter - myHDims[0] * myAxes[0] + myHDims[1] * myAxes[1] + myHDims[2] * myAxes[2]);
        theP[7].SetXYZ(myCenter + myHDims[0] * myAxes[0] + myHDims[1] * myAxes[1] + myHDims[2] * myAxes[2]);

        return true;
    }

    /// <summary>The square of the diagonal.</summary>
    public readonly double SquareExtent()
        => 4.0 * (myHDims[0] * myHDims[0] + myHDims[1] * myHDims[1] + myHDims[2] * myHDims[2]);

    /// <summary>True if the boxes do not interfere (separating axis theorem on the 15 axes).</summary>
    public readonly bool IsOut(in Bnd_OBB theOther)
    {
        if (IsVoid() || theOther.IsVoid())
        {
            return true;
        }

        if (myIsAABox && theOther.myIsAABox)
        {
            return (Math.Abs(theOther.myCenter.X() - myCenter.X()) > theOther.myHDims[0] + myHDims[0])
                   || (Math.Abs(theOther.myCenter.Y() - myCenter.Y()) > theOther.myHDims[1] + myHDims[1])
                   || (Math.Abs(theOther.myCenter.Z() - myCenter.Z()) > theOther.myHDims[2] + myHDims[2]);
        }

        // According to the Separating Axis Theorem for Oriented Bounding Boxes it is necessary to check the 15
        // separating axes (Ls): the 6 axes of the boxes and the 9 cross products of their axes. For each, the
        // "length" of box j is L(j) = Sum(myHDims[i] * |myAxes[i].Dot(Ls)|); if |(Center2 - Center1).Dot(Ls)| >
        // L(1) + L(2), the boxes do not interfere.

        // Precomputed difference between centers
        gp_XYZ D = theOther.myCenter - myCenter;

        // Check the axes of the this box, i.e. L is one of myAxes
        for (int i = 0; i < 3; ++i)
        {
            // Length of the second segment
            double aLSegm2 = 0;
            for (int j = 0; j < 3; ++j)
            {
                aLSegm2 += theOther.myHDims[j] * Math.Abs(theOther.myAxes[j].Dot(myAxes[i]));
            }

            // Distance between projected centers
            double aDistCC = Math.Abs(D.Dot(myAxes[i]));

            if (aDistCC > myHDims[i] + aLSegm2)
            {
                return true;
            }
        }

        // Check the axes of the Other box, i.e. L is one of theOther.myAxes
        for (int i = 0; i < 3; ++i)
        {
            // Length of the first segment
            double aLSegm1 = 0.0;
            for (int j = 0; j < 3; ++j)
            {
                aLSegm1 += myHDims[j] * Math.Abs(myAxes[j].Dot(theOther.myAxes[i]));
            }

            // Distance between projected centers
            double aDistCC = Math.Abs(D.Dot(theOther.myAxes[i]));

            if (aDistCC > aLSegm1 + theOther.myHDims[i])
            {
                return true;
            }
        }

        double aTolNull = Epsilon(1.0);

        // Check the axes produced by the cross products
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                // Separating axis
                gp_XYZ aLAxe = myAxes[i].Crossed(theOther.myAxes[j]);

                double aNorm = aLAxe.Modulus();
                if (aNorm < aTolNull)
                {
                    continue;
                }

                aLAxe.Divide(aNorm);

                // Length of the first segment
                double aLSegm1 = 0.0;
                for (int k = 0; k < 3; ++k)
                {
                    aLSegm1 += myHDims[k] * Math.Abs(myAxes[k].Dot(aLAxe));
                }

                // Length of the second segment
                double aLSegm2 = 0.0;
                for (int k = 0; k < 3; ++k)
                {
                    aLSegm2 += theOther.myHDims[k] * Math.Abs(theOther.myAxes[k].Dot(aLAxe));
                }

                // Distance between projected centers
                double aDistCC = Math.Abs(D.Dot(aLAxe));

                if (aDistCC > aLSegm1 + aLSegm2)
                {
                    return true;
                }
            }
        }

        return false;
    }

    /// <summary>True if the point is out of the box.</summary>
    public readonly bool IsOut(in gp_Pnt theP)
    {
        // Project the point to myAxes[i] and compare the parameters with myHDims[i]
        gp_XYZ aRV = theP.XYZ() - myCenter;

        return (Math.Abs(myAxes[0].Dot(aRV)) > myHDims[0]) || (Math.Abs(myAxes[1].Dot(aRV)) > myHDims[1])
               || (Math.Abs(myAxes[2].Dot(aRV)) > myHDims[2]);
    }

    /// <summary>True if the point is inside the box or on its boundary.</summary>
    public readonly bool Contains(in gp_Pnt theP) => !IsOut(theP);

    /// <summary>True if the other box intersects or is inside this one.</summary>
    public readonly bool Intersects(in Bnd_OBB theOther) => !IsOut(theOther);

    /// <summary>True if every vertex of the other box is inside this one.</summary>
    public readonly bool IsCompletelyInside(in Bnd_OBB theOther)
    {
        if (IsVoid() || theOther.IsVoid())
        {
            return false;
        }

        Span<gp_Pnt> aVert = stackalloc gp_Pnt[8];
        theOther.GetVertex(aVert);
        for (int i = 0; i < 8; i++)
        {
            if (IsOut(aVert[i]))
            {
                return false;
            }
        }

        return true;
    }

    // Exact equality, for use as a value

    public readonly bool Equals(Bnd_OBB other)
        => myCenter.Equals(other.myCenter)
           && myAxes[0].Equals(other.myAxes[0]) && myAxes[1].Equals(other.myAxes[1]) && myAxes[2].Equals(other.myAxes[2])
           && myHDims[0].Equals(other.myHDims[0]) && myHDims[1].Equals(other.myHDims[1]) && myHDims[2].Equals(other.myHDims[2])
           && myIsAABox == other.myIsAABox;

    public override readonly bool Equals(object? obj) => obj is Bnd_OBB other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(myCenter, myAxes[0], myAxes[1], myAxes[2], myHDims[0], myHDims[1], myHDims[2], myIsAABox);

    public static bool operator ==(in Bnd_OBB theLeft, in Bnd_OBB theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Bnd_OBB theLeft, in Bnd_OBB theRight) => !theLeft.Equals(theRight);

    /// <summary>Centre, X, Y and Z directions, the three half-sizes, then 1 for an axis-aligned box (0 otherwise).</summary>
    public override readonly string ToString()
        => gp.Format("Bnd_OBB",
                     myCenter.X(), myCenter.Y(), myCenter.Z(),
                     myAxes[0].X(), myAxes[0].Y(), myAxes[0].Z(),
                     myAxes[1].X(), myAxes[1].Y(), myAxes[1].Z(),
                     myAxes[2].X(), myAxes[2].Y(), myAxes[2].Z(),
                     myHDims[0], myHDims[1], myHDims[2],
                     myIsAABox ? 1.0 : 0.0);

    /// <summary>Selects, from the points stored in a BVH tree, the two giving the extreme projections on an axis.</summary>
    private sealed class OBB_ExtremePointsSelector
        : BVH_Traverse<double, NCollection_Vec3<double>, BVH_BoxSet<double, NCollection_Vec3<double>, gp_XYZ>, Bnd_Range>
    {
        private gp_XYZ myAxis;    //!< Axis to project the points to
        private double myPrmMin;  //!< Minimal projection parameter
        private double myPrmMax;  //!< Maximal projection parameter
        private gp_XYZ myPntMin;  //!< Minimal projection point
        private gp_XYZ myPntMax;  //!< Maximal projection point

        public OBB_ExtremePointsSelector()
        {
            myPrmMin = RealLast();
            myPrmMax = RealFirst();
        }

        public void SetAxis(in gp_XYZ theAxis) => myAxis = theAxis;

        public void Clear()
        {
            myPrmMin = RealLast();
            myPrmMax = RealFirst();
        }

        public double MinPrm() => myPrmMin;

        public double MaxPrm() => myPrmMax;

        public gp_XYZ MinPnt() => myPntMin;

        public gp_XYZ MaxPnt() => myPntMax;

        public override bool RejectNode(in NCollection_Vec3<double> theCMin, in NCollection_Vec3<double> theCMax, out Bnd_Range theMetric)
        {
            if (myPrmMin > myPrmMax)
            {
                // No parameters computed yet (C++ leaves the metric as constructed: a void range)
                theMetric = new Bnd_Range();
                return false;
            }

            double aPrmMin = myPrmMin, aPrmMax = myPrmMax;
            bool isToReject = true;

            // Check if the current node is between already found parameters
            for (int i = 0; i < 2; ++i)
            {
                double x = i == 0 ? theCMin.x() : theCMax.x();
                for (int j = 0; j < 2; ++j)
                {
                    double y = j == 0 ? theCMin.y() : theCMax.y();
                    for (int k = 0; k < 2; ++k)
                    {
                        double z = k == 0 ? theCMin.z() : theCMax.z();

                        double aPrm = myAxis.Dot(new gp_XYZ(x, y, z));
                        if (aPrm < aPrmMin)
                        {
                            aPrmMin = aPrm;
                            isToReject = false;
                        }
                        else if (aPrm > aPrmMax)
                        {
                            aPrmMax = aPrm;
                            isToReject = false;
                        }
                    }
                }
            }

            theMetric = new Bnd_Range(aPrmMin, aPrmMax);

            return isToReject;
        }

        public override bool RejectMetric(in Bnd_Range theMetric)
        {
            if (myPrmMin > myPrmMax)
            {
                // no parameters computed
                return false;
            }

            double aMin = 0.0, aMax = 0.0;
            if (!theMetric.GetBounds(ref aMin, ref aMax))
            {
                // void metric
                return false;
            }

            // Check if the box of the branch is inside of the already computed parameters
            return aMin > myPrmMin && aMax < myPrmMax;
        }

        public override bool Accept(int theIndex, in Bnd_Range theMetric)
        {
            gp_XYZ theLeaf = myBVHSet!.Element(theIndex);
            double aPrm = myAxis.Dot(theLeaf);
            if (aPrm < myPrmMin)
            {
                myPrmMin = aPrm;
                myPntMin = theLeaf;
            }

            if (aPrm > myPrmMax)
            {
                myPrmMax = aPrm;
                myPntMax = theLeaf;
            }

            return true;
        }

        public override bool IsMetricBetter(in Bnd_Range theLeft, in Bnd_Range theRight)
        {
            if (myPrmMin > myPrmMax)
            {
                // no parameters computed
                return true;
            }

            Span<double> aMin = stackalloc double[2];
            Span<double> aMax = stackalloc double[2];
            if (!theLeft.GetBounds(ref aMin[0], ref aMax[0]) || !theRight.GetBounds(ref aMin[1], ref aMax[1]))
            {
                // void metrics
                return true;
            }

            // Choose branch with larger extension over computed parameters
            Span<double> anExt = [0.0, 0.0];
            for (int i = 0; i < 2; ++i)
            {
                if (aMin[i] < myPrmMin)
                {
                    anExt[i] += myPrmMin - aMin[i];
                }

                if (aMax[i] > myPrmMax)
                {
                    anExt[i] += aMax[i] - myPrmMax;
                }
            }

            return anExt[0] > anExt[1];
        }
    }

    /// <summary>The tool for OBB construction (DiTO algorithm).</summary>
    private sealed class OBBTool
    {
        // Number of the initial axes.
        private const int myNbInitAxes = 7;

        // Number of extremal points
        private const int myNbExtremalPoints = 2 * myNbInitAxes;

        private readonly NCollection_Array1<gp_Pnt> myPntsList;               //!< The source list of points
        private readonly NCollection_Array1<double>? myListOfTolers;          //!< The array of tolerances
        private readonly int[] myTriIdx = new int[5];                         //!< Points of ditetrahedron (indices in myLExtremalPoints)
        private readonly gp_XYZ[] myLExtremalPoints = new gp_XYZ[myNbExtremalPoints]; //!< List of extremal points
        private readonly gp_XYZ[] myAxes = new gp_XYZ[3];                     //!< The axes of the box (normalized or null)
        private double myQualityCriterion;                                     //!< The surface area of the OBB
        private readonly bool myOptimal;                                       //!< Compute the OBB more tight
        private readonly BVH_BoxSet<double, NCollection_Vec3<double>, gp_XYZ>? myPointBoxSet; //!< Point box set organized with BVH

        // Stored min/max parameters for the axes between extremal points (myParams[i][j] is [i * N + j])
        private readonly double[] myParamMin = new double[myNbExtremalPoints * myNbExtremalPoints];
        private readonly double[] myParamMax = new double[myNbExtremalPoints * myNbExtremalPoints];

        /// <summary>
        /// theL: the points; theLT: their tolerances (null: every tolerance is zero).
        /// </summary>
        public OBBTool(NCollection_Array1<gp_Pnt> theL, NCollection_Array1<double>? theLT = null, bool theIsOptimal = false)
        {
            myPntsList = theL;
            myListOfTolers = theLT;
            myQualityCriterion = RealLast();
            myOptimal = theIsOptimal;

            // Params(): (RealLast, RealFirst)
            Array.Fill(myParamMin, RealLast());
            Array.Fill(myParamMax, RealFirst());

            if (myOptimal)
            {
                // Use linear builder for BVH construction with 30 elements in the leaf
                BVH_LinearBuilder<double, NCollection_Vec3<double>> aLBuilder = new(30);
                myPointBoxSet = new BVH_BoxSet<double, NCollection_Vec3<double>, gp_XYZ>(aLBuilder);
                myPointBoxSet.SetSize(myPntsList.Length());

                // Add the points into Set
                for (int iP = theL.Lower(); iP <= theL.Upper(); ++iP)
                {
                    gp_Pnt aP = theL[iP];
                    double aTol = theLT is not null ? theLT.Value(iP) : Precision.Confusion();
                    BVH_Box<double, NCollection_Vec3<double>> aBox = new(new NCollection_Vec3<double>(aP.X() - aTol, aP.Y() - aTol, aP.Z() - aTol),
                                                                        new NCollection_Vec3<double>(aP.X() + aTol, aP.Y() + aTol, aP.Z() + aTol));
                    myPointBoxSet.Add(aP.XYZ(), aBox);
                }

                myPointBoxSet.Build();
            }

            ComputeExtremePoints();
        }

        // Computes the extreme points on the set of initial axes
        private void ComputeExtremePoints()
        {
            // Six initial axes show great quality on the Optimal OBB, plus the performance is better, but they show
            // worse quality for the not optimal approach; OCCT uses seven.
            ReadOnlySpan<gp_XYZ> anInitialAxesArray =
            [
                new gp_XYZ(1.0, 0.0, 0.0),
                new gp_XYZ(0.0, 1.0, 0.0),
                new gp_XYZ(0.0, 0.0, 1.0),
                new gp_XYZ(1.0, 1.0, 1.0) / SQRT_3,
                new gp_XYZ(1.0, 1.0, -1.0) / SQRT_3,
                new gp_XYZ(1.0, -1.0, 1.0) / SQRT_3,
                new gp_XYZ(1.0, -1.0, -1.0) / SQRT_3,
            ];

            // Min and Max parameter
            Span<double> aParams = stackalloc double[myNbExtremalPoints];

            // Look for the extremal points (myLExtremalPoints)
            for (int anAxeInd = 0, aPrmInd = -1; anAxeInd < myNbInitAxes; ++anAxeInd)
            {
                int aMinInd = ++aPrmInd, aMaxInd = ++aPrmInd;
                aParams[aMinInd] = RealLast();
                aParams[aMaxInd] = -RealLast();
                Project(anInitialAxesArray[anAxeInd], out aParams[aMinInd], out aParams[aMaxInd], aMinInd, aMaxInd);
            }

            // For not optimal box it is necessary to compute the max axis
            // created by the maximally distant extreme points
            if (!myOptimal)
            {
                for (int i = 0; i < 5; i++)
                {
                    myTriIdx[i] = int.MaxValue;
                }

                // Compute myTriIdx[0] and myTriIdx[1].
                double aMaxSqDist = -1.0;
                for (int aPrmInd = 0; aPrmInd < myNbExtremalPoints; aPrmInd += 2)
                {
                    gp_Pnt aP1 = new(myLExtremalPoints[aPrmInd]), aP2 = new(myLExtremalPoints[aPrmInd + 1]);
                    double aSqDist = aP1.SquareDistance(aP2);
                    if (aSqDist > aMaxSqDist)
                    {
                        aMaxSqDist = aSqDist;
                        myTriIdx[0] = aPrmInd;
                        myTriIdx[1] = aPrmInd + 1;
                    }
                }

                // Compute the maximal axis orthogonal to the found one
                FillToTriangle3();
            }
        }

        // Two values of myTriIdx are known; finds myTriIdx[2], the point farthest from the line through them.
        private void FillToTriangle3()
        {
            gp_XYZ aP0 = myLExtremalPoints[myTriIdx[0]];
            gp_XYZ anAxis = myLExtremalPoints[myTriIdx[1]] - aP0;
            double aMaxSqDist = -1.0;
            for (int i = 0; i < myNbExtremalPoints; i++)
            {
                if ((i == myTriIdx[0]) || (i == myTriIdx[1]))
                {
                    continue;
                }

                gp_XYZ aP = myLExtremalPoints[i];
                double aDistToAxe = anAxis.CrossSquareMagnitude(aP - aP0);
                if (aDistToAxe > aMaxSqDist)
                {
                    myTriIdx[2] = i;
                    aMaxSqDist = aDistToAxe;
                }
            }
        }

        // Three values of myTriIdx are known; finds myTriIdx[3] and myTriIdx[4], the points farthest from the
        // triangle's plane on either side of it.
        private void FillToTriangle5(in gp_XYZ theNormal, in gp_XYZ theBarryCenter)
        {
            Span<double> aParams = [0.0, 0.0];
            int id3 = -1, id4 = -1;

            for (int aPtIdx = 0; aPtIdx < myNbExtremalPoints; aPtIdx++)
            {
                if ((aPtIdx == myTriIdx[0]) || (aPtIdx == myTriIdx[1]) || (aPtIdx == myTriIdx[2]))
                {
                    continue;
                }

                gp_XYZ aCurrPoint = myLExtremalPoints[aPtIdx];
                double aParam = theNormal.Dot(aCurrPoint - theBarryCenter);

                if (aParam < aParams[0])
                {
                    id3 = aPtIdx;
                    aParams[0] = aParam;
                }
                else if (aParam > aParams[1])
                {
                    id4 = aPtIdx;
                    aParams[1] = aParam;
                }
            }

            // The points must be in the different sides of the triangle plane.
            if (id3 >= 0 && aParams[0] < -Precision.Confusion())
            {
                myTriIdx[3] = id3;
            }

            if (id4 >= 0 && aParams[1] > Precision.Confusion())
            {
                myTriIdx[4] = id4;
            }
        }

        // Chooses the optimal box with axes containing the normal to the triangle and an edge of it.
        private void ProcessTriangle(int theIdx1, int theIdx2, int theIdx3, bool theIsBuiltTrg)
        {
            const int aNbAxes = 3;

            // All axes must be normalized in order to provide correct area computation
            // (see ComputeQuality(...) method).
            Span<int> ID1 = [theIdx2, theIdx3, theIdx1];
            Span<int> ID2 = [theIdx1, theIdx2, theIdx3];
            Span<gp_XYZ> aYAxis =
            [
                myLExtremalPoints[ID1[0]] - myLExtremalPoints[ID2[0]],
                myLExtremalPoints[ID1[1]] - myLExtremalPoints[ID2[1]],
                myLExtremalPoints[ID1[2]] - myLExtremalPoints[ID2[2]],
            ];

            // Normal to the triangle plane
            gp_XYZ aZAxis = aYAxis[0].Crossed(aYAxis[1]);

            double aSqMod = aZAxis.SquareModulus();

            if (aSqMod < Precision.SquareConfusion())
            {
                return;
            }

            aZAxis.Divide(Math.Sqrt(aSqMod));

            Span<gp_XYZ> aXAxis = stackalloc gp_XYZ[aNbAxes];
            Span<bool> aXAxisValid = stackalloc bool[aNbAxes];
            for (int i = 0; i < aNbAxes; i++)
            {
                gp_XYZ aCross = aYAxis[i].Crossed(aZAxis);
                double aCrossSqMod = aCross.SquareModulus();
                if (aCrossSqMod < Precision.SquareConfusion())
                {
                    aXAxisValid[i] = false;
                    continue;
                }

                aXAxis[i] = aCross / Math.Sqrt(aCrossSqMod);
                aXAxisValid[i] = true;
            }

            if (theIsBuiltTrg)
            {
                FillToTriangle5(aZAxis, myLExtremalPoints[theIdx1]);
            }

            // Min and Max parameter
            const int aNbPoints = 2 * aNbAxes;

            // Compute Min/Max params for ZAxis
            Span<double> aParams = stackalloc double[aNbPoints];
            FindMinMax(aZAxis, out aParams[4], out aParams[5]); // Compute params on ZAxis once

            int aMinIdx = -1;
            for (int anAxeInd = 0; anAxeInd < aNbAxes; anAxeInd++)
            {
                if (!aXAxisValid[anAxeInd])
                {
                    continue;
                }

                gp_XYZ aAX = aXAxis[anAxeInd];

                // Compute params on XAxis
                FindMinMax(aAX, out aParams[0], out aParams[1]);

                // Compute params on YAxis checking for stored values
                ComputeParams(ID1[anAxeInd], ID2[anAxeInd], out aParams[2], out aParams[3]);

                double anArea = ComputeQuality(aParams);
                if (anArea < myQualityCriterion)
                {
                    myQualityCriterion = anArea;
                    aMinIdx = anAxeInd;
                }
            }

            if (aMinIdx < 0)
            {
                return;
            }

            myAxes[0] = aXAxis[aMinIdx];
            myAxes[1] = aYAxis[aMinIdx].Normalized();
            myAxes[2] = aZAxis;
        }

        /// <summary>The DiTO algorithm (http://www.idt.mdh.se/~tla/publ/FastOBBs.pdf).</summary>
        public void ProcessDiTetrahedron()
        {
            // To compute the optimal OBB it is necessary to check all possible axes created by the extremal points,
            // projecting all the points on each axis.
            if (myOptimal)
            {
                for (int i = 0; i < myNbExtremalPoints - 2; i++)
                {
                    for (int j = i + 1; j < myNbExtremalPoints - 1; j++)
                    {
                        for (int k = j + 1; k < myNbExtremalPoints; k++)
                        {
                            ProcessTriangle(i, j, k, false);
                        }
                    }
                }
            }
            else
            {
                // Use the standard DiTo approach
                ProcessTriangle(myTriIdx[0], myTriIdx[1], myTriIdx[2], true);

                if (myTriIdx[3] < myNbExtremalPoints)
                {
                    ProcessTriangle(myTriIdx[0], myTriIdx[1], myTriIdx[3], false);
                    ProcessTriangle(myTriIdx[1], myTriIdx[2], myTriIdx[3], false);
                    ProcessTriangle(myTriIdx[0], myTriIdx[2], myTriIdx[3], false);
                }

                if (myTriIdx[4] < myNbExtremalPoints)
                {
                    ProcessTriangle(myTriIdx[0], myTriIdx[1], myTriIdx[4], false);
                    ProcessTriangle(myTriIdx[1], myTriIdx[2], myTriIdx[4], false);
                    ProcessTriangle(myTriIdx[0], myTriIdx[2], myTriIdx[4], false);
                }
            }
        }

        /// <summary>Makes the box from the computed parameters.</summary>
        public void BuildBox(ref Bnd_OBB theBox)
        {
            theBox.SetVoid();

            // In fact, use Precision::SquareConfusion().
            bool isOBB = myAxes[0].SquareModulus() * myAxes[1].SquareModulus() * myAxes[2].SquareModulus() > 1.0e-14;

            gp_Dir aXDir = isOBB ? new gp_Dir(myAxes[0]) : new gp_Dir(gp_Dir.D.X);
            gp_Dir aYDir = isOBB ? new gp_Dir(myAxes[1]) : new gp_Dir(gp_Dir.D.Y);
            gp_Dir aZDir = isOBB ? new gp_Dir(myAxes[2]) : new gp_Dir(gp_Dir.D.Z);

            const int aNbPoints = 6;
            Span<double> aParams = stackalloc double[aNbPoints];

            gp_XYZ aFCurrPoint = myPntsList.First().XYZ();

            aParams[0] = aParams[1] = aFCurrPoint.Dot(aXDir.XYZ());
            aParams[2] = aParams[3] = aFCurrPoint.Dot(aYDir.XYZ());
            aParams[4] = aParams[5] = aFCurrPoint.Dot(aZDir.XYZ());

            if (myListOfTolers is not null)
            {
                double aTol = myListOfTolers.First();
                aParams[0] -= aTol;
                aParams[1] += aTol;
                aParams[2] -= aTol;
                aParams[3] += aTol;
                aParams[4] -= aTol;
                aParams[5] += aTol;
            }

            for (int i = myPntsList.Lower() + 1; i <= myPntsList.Upper(); i++)
            {
                gp_XYZ aCurrPoint = myPntsList[i].XYZ();
                double aDx = aCurrPoint.Dot(aXDir.XYZ()), aDy = aCurrPoint.Dot(aYDir.XYZ()), aDz = aCurrPoint.Dot(aZDir.XYZ());

                if (myListOfTolers is null)
                {
                    SetMinMax(aParams[0..], aDx);
                    SetMinMax(aParams[2..], aDy);
                    SetMinMax(aParams[4..], aDz);
                }
                else
                {
                    double aTol = myListOfTolers.Value(i);
                    aParams[0] = Min(aParams[0], aDx - aTol);
                    aParams[1] = Max(aParams[1], aDx + aTol);
                    aParams[2] = Min(aParams[2], aDy - aTol);
                    aParams[3] = Max(aParams[3], aDy + aTol);
                    aParams[4] = Min(aParams[4], aDz - aTol);
                    aParams[5] = Max(aParams[5], aDz + aTol);
                }
            }

            // Half-sizes
            double aHX = 0.5 * (aParams[1] - aParams[0]);
            double aHY = 0.5 * (aParams[3] - aParams[2]);
            double aHZ = 0.5 * (aParams[5] - aParams[4]);

            gp_XYZ aCenter = 0.5
                             * ((aParams[1] + aParams[0]) * aXDir.XYZ() + (aParams[3] + aParams[2]) * aYDir.XYZ()
                                + (aParams[5] + aParams[4]) * aZDir.XYZ());

            theBox.SetCenter(new gp_Pnt(aCenter));
            theBox.SetXComponent(aXDir, aHX);
            theBox.SetYComponent(aYDir, aHY);
            theBox.SetZComponent(aZDir, aHZ);
            theBox.SetAABox(!isOBB);
        }

        // Half of the surface area of the box
        private static double ComputeQuality(ReadOnlySpan<double> thePrmArr)
        {
            double aDX = thePrmArr[1] - thePrmArr[0], aDY = thePrmArr[3] - thePrmArr[2], aDZ = thePrmArr[5] - thePrmArr[4];
            return aDX * aDY + aDY * aDZ + aDX * aDZ;
        }

        // The min and max parameters on the axis between myLExtremalPoints[theId1] and [theId2], cached.
        private void ComputeParams(int theId1, int theId2, out double theMin, out double theMax)
        {
            theMin = myParamMin[theId1 * myNbExtremalPoints + theId2];
            theMax = myParamMax[theId1 * myNbExtremalPoints + theId2];

            if (theMin > theMax)
            {
                FindMinMax((myLExtremalPoints[theId1] - myLExtremalPoints[theId2]).Normalized(), out theMin, out theMax);
                myParamMin[theId1 * myNbExtremalPoints + theId2] = myParamMin[theId2 * myNbExtremalPoints + theId1] = theMin;
                myParamMax[theId1 * myNbExtremalPoints + theId2] = myParamMax[theId2 * myNbExtremalPoints + theId1] = theMax;
            }
        }

        // The min and max parameters on the axis: of all the points in the optimal case, of the extreme points otherwise.
        private void FindMinMax(in gp_XYZ theAxis, out double theMin, out double theMax)
        {
            theMin = RealLast();
            theMax = RealFirst();

            if (myOptimal)
            {
                Project(theAxis, out theMin, out theMax);
            }
            else
            {
                for (int i = 0; i < myNbExtremalPoints; ++i)
                {
                    double aPrm = theAxis.Dot(myLExtremalPoints[i]);
                    if (aPrm < theMin)
                    {
                        theMin = aPrm;
                    }

                    if (aPrm > theMax)
                    {
                        theMax = aPrm;
                    }
                }
            }
        }

        // Projects the points on the axis; the extreme points are written into myLExtremalPoints at the given
        // indices (OCCT's optional pointers; -1 for none).
        private void Project(in gp_XYZ theAxis, out double theMin, out double theMax, int thePntMin = -1, int thePntMax = -1)
        {
            theMin = RealLast();
            theMax = RealFirst();

            if (myOptimal)
            {
                // Project BVH
                OBB_ExtremePointsSelector anExtremePointsSelector = new();
                anExtremePointsSelector.SetBVHSet(myPointBoxSet);
                anExtremePointsSelector.SetAxis(theAxis);
                anExtremePointsSelector.Select();
                theMin = anExtremePointsSelector.MinPrm();
                theMax = anExtremePointsSelector.MaxPrm();
                if (thePntMin >= 0)
                {
                    myLExtremalPoints[thePntMin] = anExtremePointsSelector.MinPnt();
                }

                if (thePntMax >= 0)
                {
                    myLExtremalPoints[thePntMax] = anExtremePointsSelector.MaxPnt();
                }
            }
            else
            {
                // Project all points
                for (int iP = myPntsList.Lower(); iP <= myPntsList.Upper(); ++iP)
                {
                    gp_XYZ aPoint = myPntsList[iP].XYZ();
                    double aPrm = theAxis.Dot(aPoint);
                    if (aPrm < theMin)
                    {
                        theMin = aPrm;
                        if (thePntMin >= 0)
                        {
                            myLExtremalPoints[thePntMin] = aPoint;
                        }
                    }

                    if (aPrm > theMax)
                    {
                        theMax = aPrm;
                        if (thePntMax >= 0)
                        {
                            myLExtremalPoints[thePntMax] = aPoint;
                        }
                    }
                }
            }
        }

        // ATTENTION!!! thePrmArr must be initialized before this method calling.
        private static void SetMinMax(Span<double> thePrmArr, double theNewParam)
        {
            if (theNewParam < thePrmArr[0])
            {
                thePrmArr[0] = theNewParam;
            }

            if (theNewParam > thePrmArr[1])
            {
                thePrmArr[1] = theNewParam;
            }
        }
    }
}
