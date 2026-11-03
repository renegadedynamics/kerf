// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Point.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Point.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Occt;

/// <summary>A point on a surface, with both its 3D point and its (U, V) parameters.</summary>
/// <remarks>
/// OCCT's default point is all zeros with <c>PartOfCommon() == 1</c>; <c>default(IntPolyh_Point)</c> has 0 there, so
/// ported code writes <c>new IntPolyh_Point()</c>.
/// </remarks>
public partial struct IntPolyh_Point : IEquatable<IntPolyh_Point>, IntPolyh_IDump
{
    private double myX;
    private double myY;
    private double myZ;
    private double myU;
    private double myV;
    private int myPOC;
    private bool myDegenerated;

    /// <summary>The origin, at parameters (0, 0).</summary>
    public IntPolyh_Point()
    {
        myX = 0.0;
        myY = 0.0;
        myZ = 0.0;
        myU = 0.0;
        myV = 0.0;
        myPOC = 1;
        myDegenerated = false;
    }

    /// <summary>The point (x, y, z) at parameters (u, v).</summary>
    public IntPolyh_Point(double x, double y, double z, double u, double v)
    {
        myX = x;
        myY = y;
        myZ = z;
        myU = u;
        myV = v;
        myPOC = 1;
        myDegenerated = false;
    }

    public readonly double X() => myX;

    public readonly double Y() => myY;

    public readonly double Z() => myZ;

    public readonly double U() => myU;

    public readonly double V() => myV;

    /// <summary>0 if the point is not common with the other surface.</summary>
    public readonly int PartOfCommon() => myPOC;

    /// <summary>The degenerated flag.</summary>
    public readonly bool Degenerated() => myDegenerated;

    // Mutators (internal: see docs/translation-rules.md)

    internal void Set(double x, double y, double z, double u, double v, int II = 1)
    {
        myX = x;
        myY = y;
        myZ = z;
        myU = u;
        myV = v;
        myPOC = II;
    }

    internal void SetX(double x) => myX = x;

    internal void SetY(double y) => myY = y;

    internal void SetZ(double z) => myZ = z;

    internal void SetU(double u) => myU = u;

    internal void SetV(double v) => myV = v;

    internal void SetPartOfCommon(int ii) => myPOC = ii;

    internal void SetDegenerated(bool theFlag) => myDegenerated = theFlag;

    /// <summary>Makes this the middle of P1 and P2 in (U, V), with its 3D point on the surface.</summary>
    internal void Middle(SurfaceAdaptorBase MySurface, in IntPolyh_Point Point1, in IntPolyh_Point Point2)
    {
        myU = (Point1.U() + Point2.U()) * 0.5;
        myV = (Point1.V() + Point2.V()) * 0.5;

        Point3D PtXYZ = MySurface.Value(myU, myV);

        myX = PtXYZ.X();
        myY = PtXYZ.Y();
        myZ = PtXYZ.Z();
    }

    /// <summary>Makes this the cross product of a and b (U, V unchanged).</summary>
    internal void Cross(in IntPolyh_Point a, in IntPolyh_Point b)
    {
        myX = a.myY * b.myZ - a.myZ * b.myY;
        myY = a.myZ * b.myX - a.myX * b.myZ;
        myZ = a.myX * b.myY - a.myY * b.myX;
    }

    // The mutators' twins

    public readonly IntPolyh_Point WithValues(double x, double y, double z, double u, double v, int II = 1)
    {
        IntPolyh_Point aCopy = this;
        aCopy.Set(x, y, z, u, v, II);
        return aCopy;
    }

    public readonly IntPolyh_Point WithX(double x)
    {
        IntPolyh_Point aCopy = this;
        aCopy.SetX(x);
        return aCopy;
    }

    public readonly IntPolyh_Point WithY(double y)
    {
        IntPolyh_Point aCopy = this;
        aCopy.SetY(y);
        return aCopy;
    }

    public readonly IntPolyh_Point WithZ(double z)
    {
        IntPolyh_Point aCopy = this;
        aCopy.SetZ(z);
        return aCopy;
    }

    public readonly IntPolyh_Point WithU(double u)
    {
        IntPolyh_Point aCopy = this;
        aCopy.SetU(u);
        return aCopy;
    }

    public readonly IntPolyh_Point WithV(double v)
    {
        IntPolyh_Point aCopy = this;
        aCopy.SetV(v);
        return aCopy;
    }

    public readonly IntPolyh_Point WithPartOfCommon(int ii)
    {
        IntPolyh_Point aCopy = this;
        aCopy.SetPartOfCommon(ii);
        return aCopy;
    }

    public readonly IntPolyh_Point WithDegenerated(bool theFlag)
    {
        IntPolyh_Point aCopy = this;
        aCopy.SetDegenerated(theFlag);
        return aCopy;
    }

    /// <summary>The middle of P1 and P2 in (U, V), on the surface; the twin of <c>Middle</c>.</summary>
    public readonly IntPolyh_Point Middled(SurfaceAdaptorBase MySurface, in IntPolyh_Point Point1, in IntPolyh_Point Point2)
    {
        IntPolyh_Point aCopy = this;
        aCopy.Middle(MySurface, Point1, Point2);
        return aCopy;
    }

    /// <summary>This point with the cross product of a and b as coordinates; the twin of <c>Cross</c>.</summary>
    public readonly IntPolyh_Point Crossed(in IntPolyh_Point a, in IntPolyh_Point b)
    {
        IntPolyh_Point aCopy = this;
        aCopy.Cross(a, b);
        return aCopy;
    }

    // Arithmetic: all five coordinates; the result has OCCT's default flags

    public readonly IntPolyh_Point Add(in IntPolyh_Point P1)
    {
        IntPolyh_Point res = new();
        res.SetX(myX + P1.X());
        res.SetY(myY + P1.Y());
        res.SetZ(myZ + P1.Z());
        res.SetU(myU + P1.U());
        res.SetV(myV + P1.V());
        return res;
    }

    public readonly IntPolyh_Point Sub(in IntPolyh_Point P1)
    {
        IntPolyh_Point res = new();
        res.SetX(myX - P1.X());
        res.SetY(myY - P1.Y());
        res.SetZ(myZ - P1.Z());
        res.SetU(myU - P1.U());
        res.SetV(myV - P1.V());
        return res;
    }

    /// <summary>The point divided by RR; the default point if |RR| is not above <c>Precision::Computational()</c>.</summary>
    public readonly IntPolyh_Point Divide(double RR)
    {
        IntPolyh_Point res = new();
        if (Math.Abs(RR) > global::Kerf.Engine.Foundation.Precision.Computational())
        {
            res.SetX(myX / RR);
            res.SetY(myY / RR);
            res.SetZ(myZ / RR);
            res.SetU(myU / RR);
            res.SetV(myV / RR);
        }

        return res;
    }

    public readonly IntPolyh_Point Multiplication(double RR)
    {
        IntPolyh_Point res = new();
        res.SetX(myX * RR);
        res.SetY(myY * RR);
        res.SetZ(myZ * RR);
        res.SetU(myU * RR);
        res.SetV(myV * RR);
        return res;
    }

    public static IntPolyh_Point operator +(in IntPolyh_Point theLeft, in IntPolyh_Point theRight) => theLeft.Add(theRight);

    public static IntPolyh_Point operator -(in IntPolyh_Point theLeft, in IntPolyh_Point theRight) => theLeft.Sub(theRight);

    public static IntPolyh_Point operator /(in IntPolyh_Point theLeft, double rr) => theLeft.Divide(rr);

    public static IntPolyh_Point operator *(in IntPolyh_Point theLeft, double rr) => theLeft.Multiplication(rr);

    /// <summary>The square modulus of the 3D point.</summary>
    public readonly double SquareModulus()
    {
        double res = myX * myX + myY * myY + myZ * myZ;
        return res;
    }

    /// <summary>The square distance between the 3D points.</summary>
    public readonly double SquareDistance(in IntPolyh_Point P2)
    {
        double res = (myX - P2.myX) * (myX - P2.myX) + (myY - P2.myY) * (myY - P2.myY)
                     + (myZ - P2.myZ) * (myZ - P2.myZ);
        return res;
    }

    /// <summary>The dot product of the 3D points.</summary>
    public readonly double Dot(in IntPolyh_Point b)
    {
        double t = myX * b.myX + myY * b.myY + myZ * b.myZ;
        return t;
    }

    /// <summary>Writes the point as OCCT's <c>Dump()</c> prints it.</summary>
    public readonly void Dump(TextWriter theStream)
    {
        theStream.Write($"\nPoint : x={IntPolyh_Format.E(myX)}g y={IntPolyh_Format.E(myY)}g z={IntPolyh_Format.E(myZ)}g "
                        + $"u={IntPolyh_Format.E(myU)}g v={IntPolyh_Format.E(myV)}g\n");
    }

    /// <summary>Writes the point with its index, as OCCT's <c>Dump(i)</c> prints it.</summary>
    public readonly void Dump(int i, TextWriter theStream)
    {
        theStream.Write($"\nPoint({IntPolyh_Format.D(i, 3)}) : x={IntPolyh_Format.E(myX)}g y={IntPolyh_Format.E(myY)}g "
                        + $"z={IntPolyh_Format.E(myZ)}g u={IntPolyh_Format.E(myU)}g v={IntPolyh_Format.E(myV)}g "
                        + $"poc={IntPolyh_Format.D(myPOC, 3)}\n");
    }

    public readonly bool Equals(IntPolyh_Point other)
        => myX.Equals(other.myX) && myY.Equals(other.myY) && myZ.Equals(other.myZ) && myU.Equals(other.myU)
           && myV.Equals(other.myV) && myPOC == other.myPOC && myDegenerated == other.myDegenerated;

    public override readonly bool Equals(object? obj) => obj is IntPolyh_Point other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myX, myY, myZ, myU, myV, myPOC, myDegenerated);

    public static bool operator ==(in IntPolyh_Point theLeft, in IntPolyh_Point theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntPolyh_Point theLeft, in IntPolyh_Point theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => Constants.Format("IntPolyh_Point", myX, myY, myZ, myU, myV);
}
