// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Surface.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>A surface of the legacy data structure: a surface with a tolerance, and whether it is kept.</summary>
/// <remarks>
/// A C++ value class, so a struct. Unlike points and curves, a new surface is <em>not</em> kept (OCCT's default):
/// <see cref="TopOpeBRepDS_SurfaceExplorer"/> with its default <c>FindOnlyKeep</c> skips it until it is.
/// </remarks>
public partial struct TopOpeBRepDS_Surface : IEquatable<TopOpeBRepDS_Surface>
{
    private Surface? mySurface;
    private double myTolerance;
    private bool myKeep;

    /// <summary>An empty surface, not kept.</summary>
    public TopOpeBRepDS_Surface()
    {
        mySurface = null;
        myTolerance = 0.0;
        myKeep = false;
    }

    /// <summary>The surface <paramref name="theSurface"/> with the tolerance <paramref name="theTolerance"/>, not kept.</summary>
    public TopOpeBRepDS_Surface(Surface? theSurface, double theTolerance)
    {
        mySurface = theSurface;
        myTolerance = theTolerance;
        myKeep = false;
    }

    /// <summary>A copy of <paramref name="theOther"/>.</summary>
    public TopOpeBRepDS_Surface(in TopOpeBRepDS_Surface theOther)
    {
        mySurface = theOther.mySurface;
        myTolerance = theOther.myTolerance;
        myKeep = theOther.myKeep;
    }

    internal void Assign(in TopOpeBRepDS_Surface Other)
    {
        mySurface = Other.mySurface;
        myTolerance = Other.myTolerance;
        myKeep = Other.myKeep;
    }

    /// <summary>The surface.</summary>
    public readonly Surface? Surface() => mySurface;

    /// <summary>The tolerance.</summary>
    public readonly double Tolerance() => myTolerance;

    internal void Tolerance(double theTol)
    {
        myTolerance = theTol;
    }

    /// <summary>Whether the surface is kept.</summary>
    public readonly bool Keep() => myKeep;

    internal void ChangeKeep(bool theToKeep)
    {
        myKeep = theToKeep;
    }

    /// <summary>A copy with the tolerance <paramref name="theTol"/>.</summary>
    public readonly TopOpeBRepDS_Surface WithTolerance(double theTol)
    {
        TopOpeBRepDS_Surface aCopy = this;
        aCopy.Tolerance(theTol);
        return aCopy;
    }

    /// <inheritdoc/>
    public readonly bool Equals(TopOpeBRepDS_Surface other)
        => ReferenceEquals(mySurface, other.mySurface) && myTolerance.Equals(other.myTolerance) && myKeep == other.myKeep;

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is TopOpeBRepDS_Surface other && Equals(other);

    /// <inheritdoc/>
    public override readonly int GetHashCode()
        => HashCode.Combine(mySurface is null ? 0 : System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(mySurface),
                            myTolerance, myKeep);

    public static bool operator ==(in TopOpeBRepDS_Surface theLeft, in TopOpeBRepDS_Surface theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in TopOpeBRepDS_Surface theLeft, in TopOpeBRepDS_Surface theRight) => !theLeft.Equals(theRight);

    /// <inheritdoc/>
    public override readonly string ToString()
        => "TopOpeBRepDS_Surface(" + (mySurface is null ? "null" : global::Kerf.Engine.Foundation.OcctTypeNames.Of(mySurface.GetType())) + ", " + myTolerance.ToString("R", System.Globalization.CultureInfo.InvariantCulture)
           + (myKeep ? ", kept)" : ")");
}
