// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Sphere.hxx
// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A bounding sphere of a geometric entity (triangle, line segment, ...), with the entity's U and V indices.</summary>
public partial struct Bnd_Sphere : IEquatable<Bnd_Sphere>
{
    private gp_XYZ myCenter;
    private double myRadius;
    private bool myIsValid;
    private int myU;
    private int myV;

    /// <summary>Creates a sphere of radius 0 at the origin, not valid.</summary>
    public Bnd_Sphere()
    {
        myCenter = new gp_XYZ(0.0, 0.0, 0.0);
        myRadius = 0.0;
        myIsValid = false;
        myU = 0;
        myV = 0;
    }

    /// <summary>Creates a definite sphere (not marked valid).</summary>
    public Bnd_Sphere(in gp_XYZ theCenter, double theRadius, int theU, int theV)
    {
        myCenter = theCenter;
        myRadius = theRadius;
        myIsValid = false;
        myU = theU;
        myV = theV;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValid(bool isValid) => myIsValid = isValid;

    internal void Add(in Bnd_Sphere theOther)
    {
        // OCCT tests a negative radius for "not initialised yet", but the default constructor sets the radius to 0,
        // so adding to a default sphere encloses the origin as well.
        if (myRadius < 0.0)
        {
            // not initialised yet
            this = theOther;
            return;
        }

        double aDist = (myCenter - theOther.myCenter).Modulus();
        if (myRadius + aDist <= theOther.myRadius)
        {
            // the other sphere is larger and encloses this
            this = theOther;
            return;
        }

        if (theOther.myRadius + aDist <= myRadius)
        {
            return; // this sphere encloses other
        }

        // expansion
        double dfR = (aDist + myRadius + theOther.myRadius) * 0.5;
        double aParamOnDiam = (dfR - myRadius) / aDist;
        myCenter = myCenter * (1.0 - aParamOnDiam) + theOther.myCenter * aParamOnDiam;
        myRadius = dfR;
        myIsValid = false;
    }

    // Copies with a mutator applied (the public way to "set")

    public readonly Bnd_Sphere WithValid(bool isValid)
    {
        Bnd_Sphere aCopy = this;
        aCopy.SetValid(isValid);
        return aCopy;
    }

    /// <summary>The smallest sphere enclosing this sphere and <paramref name="theOther"/> (not marked valid).</summary>
    public readonly Bnd_Sphere Added(in Bnd_Sphere theOther)
    {
        Bnd_Sphere aCopy = this;
        aCopy.Add(theOther);
        return aCopy;
    }

    // Queries

    /// <summary>The U parameter on the shape.</summary>
    public readonly int U() => myU;

    /// <summary>The V parameter on the shape.</summary>
    public readonly int V() => myV;

    /// <summary>True if this sphere corresponds to a real entity.</summary>
    public readonly bool IsValid() => myIsValid;

    public readonly gp_XYZ Center() => myCenter;

    public readonly double Radius() => myRadius;

    /// <summary>The minimal and maximal distances from <paramref name="theXYZ"/> to the sphere.</summary>
    public readonly void Distances(in gp_XYZ theXYZ, out double theMin, out double theMax)
    {
        theMax = (theXYZ - myCenter).Modulus();
        theMin = (theMax - myRadius < 0 ? 0.0 : theMax - myRadius);
        theMax += myRadius;
    }

    /// <summary>The minimal and maximal squared distances from <paramref name="theXYZ"/> to the sphere.</summary>
    public readonly void SquareDistances(in gp_XYZ theXYZ, out double theMin, out double theMax)
    {
        theMax = (theXYZ - myCenter).SquareModulus();
        double aRadSq = myRadius * myRadius;
        theMin = (theMax < aRadSq ? 0.0 : theMax - aRadSq);
        theMax += aRadSq;
    }

    /// <summary>Projects a point on the entity (here, its center); always true.</summary>
    public readonly bool Project(in gp_XYZ theNode, out gp_XYZ theProjNode, out double theDist, out bool theInside)
    {
        theProjNode = myCenter;
        theDist = (theNode - theProjNode).Modulus();
        theInside = true;
        return true;
    }

    public readonly double Distance(in gp_XYZ theNode) => (theNode - myCenter).Modulus();

    public readonly double SquareDistance(in gp_XYZ theNode) => (theNode - myCenter).SquareModulus();

    public readonly bool IsOut(in Bnd_Sphere theOther)
        => (myCenter - theOther.myCenter).SquareModulus()
           > (myRadius + theOther.myRadius) * (myRadius + theOther.myRadius);

    /// <summary>
    /// True if the point's minimal distance exceeds <paramref name="theMaxDist"/>; otherwise, for a valid sphere,
    /// lowers <paramref name="theMaxDist"/> to the maximal distance when that is smaller.
    /// </summary>
    public readonly bool IsOut(in gp_XYZ theXYZ, ref double theMaxDist)
    {
        Distances(theXYZ, out double aCurMinDist, out double aCurMaxDist);
        if (aCurMinDist > theMaxDist)
        {
            return true;
        }

        if (myIsValid && aCurMaxDist < theMaxDist)
        {
            theMaxDist = aCurMaxDist;
        }

        return false;
    }

    public readonly double SquareExtent() => 4 * myRadius * myRadius;

    // Exact equality, for use as a value

    public readonly bool Equals(Bnd_Sphere other)
        => myCenter.Equals(other.myCenter) && myRadius.Equals(other.myRadius) && myIsValid == other.myIsValid
           && myU == other.myU && myV == other.myV;

    public override readonly bool Equals(object? obj) => obj is Bnd_Sphere other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myCenter, myRadius, myIsValid, myU, myV);

    public static bool operator ==(in Bnd_Sphere theLeft, in Bnd_Sphere theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Bnd_Sphere theLeft, in Bnd_Sphere theRight) => !theLeft.Equals(theRight);

    /// <summary>The center, radius, validity (1 or 0), U and V.</summary>
    public override readonly string ToString()
        => gp.Format("Bnd_Sphere", myCenter.X(), myCenter.Y(), myCenter.Z(), myRadius, myIsValid ? 1 : 0, myU, myV);
}
