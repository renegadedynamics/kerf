// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_POnSurfParams.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.Extrema_ElementType;

namespace Occt;

/// <summary>
/// A point on a surface with the data Extrema_GenExtPS needs to compute an initial approximation: the square distance
/// to the projected point, the kind of grid element it lies on and that element's indices.
/// </summary>
/// <remarks>
/// OCCT derives this class from Extrema_POnSurf; C# structs can't derive, so it embeds one (<see cref="POnSurf"/>,
/// C++'s slicing conversion) and forwards <see cref="Value"/>, <see cref="Parameter"/> and <c>SetParameters</c>.
/// <c>new Extrema_POnSurfParams()</c> has OCCT's defaults (an indefinite point, distance 0, a node at (0, 0));
/// <c>default</c> is all zeros. The setters are internal, with the twins <see cref="WithSqrDistance"/>,
/// <see cref="WithElementType"/>, <see cref="WithIndices"/> and the constructor.
/// </remarks>
public partial struct Extrema_POnSurfParams : IEquatable<Extrema_POnSurfParams>
{
    private Extrema_POnSurf myPOnSurf;
    private double mySqrDistance;
    private Extrema_ElementType myElementType;
    private int myIndexU;
    private int myIndexV;

    /// <summary>An indefinite point, on a node (0, 0), at distance 0.</summary>
    public Extrema_POnSurfParams()
    {
        myPOnSurf = new Extrema_POnSurf();
        mySqrDistance = 0.0;
        myElementType = Extrema_Node;
        myIndexU = 0;
        myIndexV = 0;
    }

    /// <summary>A point on surface with its parameter values on the surface.</summary>
    public Extrema_POnSurfParams(double theU, double theV, in gp_Pnt thePnt)
    {
        myPOnSurf = new Extrema_POnSurf(theU, theV, thePnt);
        mySqrDistance = 0.0;
        myElementType = Extrema_Node;
        myIndexU = 0;
        myIndexV = 0;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the parameters and the point (Extrema_POnSurf::SetParameters).</summary>
    internal void SetParameters(double theU, double theV, in gp_Pnt thePnt) => myPOnSurf.SetParameters(theU, theV, thePnt);

    /// <summary>Sets the square distance from this point to another one (the point to be projected).</summary>
    internal void SetSqrDistance(double theSqrDistance) => mySqrDistance = theSqrDistance;

    /// <summary>Sets the element type on which this point is situated.</summary>
    internal void SetElementType(Extrema_ElementType theElementType) => myElementType = theElementType;

    /// <summary>Sets the U and V indices of an element that contains this point.</summary>
    internal void SetIndices(int theIndexU, int theIndexV)
    {
        myIndexU = theIndexU;
        myIndexV = theIndexV;
    }

    // Twins

    public readonly Extrema_POnSurfParams WithSqrDistance(double theSqrDistance)
    {
        Extrema_POnSurfParams aCopy = this;
        aCopy.SetSqrDistance(theSqrDistance);
        return aCopy;
    }

    public readonly Extrema_POnSurfParams WithElementType(Extrema_ElementType theElementType)
    {
        Extrema_POnSurfParams aCopy = this;
        aCopy.SetElementType(theElementType);
        return aCopy;
    }

    public readonly Extrema_POnSurfParams WithIndices(int theIndexU, int theIndexV)
    {
        Extrema_POnSurfParams aCopy = this;
        aCopy.SetIndices(theIndexU, theIndexV);
        return aCopy;
    }

    // Queries

    /// <summary>The point on surface alone (C++ converts by slicing).</summary>
    public readonly Extrema_POnSurf POnSurf() => myPOnSurf;

    /// <summary>The 3D point.</summary>
    public readonly gp_Pnt Value() => myPOnSurf.Value();

    /// <summary>The parameter values on the surface.</summary>
    public readonly void Parameter(out double theU, out double theV) => myPOnSurf.Parameter(out theU, out theV);

    /// <summary>The square distance from this point to another one.</summary>
    public readonly double GetSqrDistance() => mySqrDistance;

    /// <summary>The element type on which this point is situated.</summary>
    public readonly Extrema_ElementType GetElementType() => myElementType;

    /// <summary>The U and V indices of an element that contains this point.</summary>
    public readonly void GetIndices(out int theIndexU, out int theIndexV)
    {
        theIndexU = myIndexU;
        theIndexV = myIndexV;
    }

    public readonly bool Equals(Extrema_POnSurfParams other)
        => myPOnSurf.Equals(other.myPOnSurf) && mySqrDistance.Equals(other.mySqrDistance)
           && myElementType == other.myElementType && myIndexU == other.myIndexU && myIndexV == other.myIndexV;

    public override readonly bool Equals(object? obj) => obj is Extrema_POnSurfParams other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myPOnSurf, mySqrDistance, myElementType, myIndexU, myIndexV);

    public static bool operator ==(in Extrema_POnSurfParams theLeft, in Extrema_POnSurfParams theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Extrema_POnSurfParams theLeft, in Extrema_POnSurfParams theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => string.Create(CultureInfo.InvariantCulture,
            $"Extrema_POnSurfParams({myPOnSurf}, {mySqrDistance:R}, {myElementType}, {myIndexU}, {myIndexV})");
}
