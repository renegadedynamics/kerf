// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_PntFace.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Topology;

namespace Occt;

/// <summary>
/// An intersection point of a curve with a face of a shape: the point, the face, the transition, the parameter on the
/// curve and the parameters on the face.
/// </summary>
/// <remarks><c>default(LocOpe_PntFace)</c> is OCCT's empty point (its orientation is FORWARD).</remarks>
public partial struct LocOpe_PntFace : IEquatable<LocOpe_PntFace>
{
    private Point3D myPnt;
    private Face myFace;
    private Orientation myOri;
    private double myPar;
    private double myUPar;
    private double myVPar;

    /// <summary>Empty constructor. Useful only for the list.</summary>
    public LocOpe_PntFace()
    {
        myPnt = new Point3D();
        myFace = new Face();
        myOri = global::Kerf.Topology.Orientation.Forward; // OCCT: uninitialised
        myPar = 0.0;
        myUPar = 0.0;
        myVPar = 0.0;
    }

    public LocOpe_PntFace(in Point3D P, in Face F, Orientation Or, double Param, double UPar, double VPar)
    {
        myPnt = P;
        myFace = F;
        myOri = Or;
        myPar = Param;
        myUPar = UPar;
        myVPar = VPar;
    }

    public readonly Point3D Pnt() => myPnt;

    public readonly Face Face() => myFace;

    public readonly Orientation Orientation() => myOri;

    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref Orientation ChangeOrientation() => ref myOri;

    public readonly double Parameter() => myPar;

    public readonly double UParameter() => myUPar;

    public readonly double VParameter() => myVPar;

    /// <summary>A copy with the orientation <paramref name="theOri"/>.</summary>
    public readonly LocOpe_PntFace WithOrientation(Orientation theOri)
    {
        LocOpe_PntFace aCopy = this;
        aCopy.myOri = theOri;
        return aCopy;
    }

    public readonly bool Equals(LocOpe_PntFace other) =>
        myPnt == other.myPnt && myFace == other.myFace && myFace.Orientation() == other.myFace.Orientation()
        && myOri == other.myOri && myPar.Equals(other.myPar) && myUPar.Equals(other.myUPar) && myVPar.Equals(other.myVPar);

    public override readonly bool Equals(object? obj) => obj is LocOpe_PntFace other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myPnt, myFace, myOri, myPar, myUPar, myVPar);

    public static bool operator ==(in LocOpe_PntFace theLeft, in LocOpe_PntFace theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in LocOpe_PntFace theLeft, in LocOpe_PntFace theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() =>
        Constants.Format("LocOpe_PntFace", myPnt.X(), myPnt.Y(), myPnt.Z(), (double)myOri, myPar, myUPar, myVPar);
}
