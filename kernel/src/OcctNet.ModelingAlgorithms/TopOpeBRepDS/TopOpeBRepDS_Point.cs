// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Point.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Point.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>A point of the legacy data structure: a 3D point with a tolerance, and whether it is kept.</summary>
/// <remarks>
/// A C++ value class, so a struct. The data structure keeps its own copy (<see cref="TopOpeBRepDS_DataStructure.AddPoint"/>)
/// and gives it for writing by <c>ref</c> (<see cref="TopOpeBRepDS_DataStructure.ChangePoint"/>).
/// </remarks>
public partial struct TopOpeBRepDS_Point : IEquatable<TopOpeBRepDS_Point>
{
    private Point3D myPoint;
    private double myTolerance;
    private bool myKeep;

    /// <summary>An empty point, kept.</summary>
    /// <remarks>OCCT leaves the tolerance unset; the port's is 0.</remarks>
    public TopOpeBRepDS_Point()
    {
        myPoint = new Point3D();
        myTolerance = 0.0;
        myKeep = true;
    }

    /// <summary>The point <paramref name="P"/> with the tolerance <paramref name="T"/>, kept.</summary>
    public TopOpeBRepDS_Point(in Point3D P, double T)
    {
        myPoint = P;
        myTolerance = T;
        myKeep = true;
    }

    /// <summary>The point and tolerance of the vertex <paramref name="S"/>.</summary>
    /// <exception cref="ProgramErrorException"><paramref name="S"/> is not a vertex.</exception>
    public TopOpeBRepDS_Point(in TopoDS_Shape S)
    {
        myPoint = TopOpeBRepTool_ShapeTool.Pnt(S);
        myTolerance = TopOpeBRepTool_ShapeTool.Tolerance(S);
        // OCCT leaves myKeep unset here (an uninitialised member); the port keeps the point, as the other constructors
        myKeep = true;
    }

    /// <summary>Whether the two points are equal within the larger of their tolerances.</summary>
    public readonly bool IsEqual(in TopOpeBRepDS_Point P)
    {
        double t = Max(myTolerance, P.Tolerance());
        bool b = myPoint.IsEqual(P.Point(), t);
        return b;
    }

    /// <summary>The 3D point.</summary>
    public readonly Point3D Point() => myPoint;

    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref Point3D ChangePoint() => ref myPoint;

    /// <summary>The tolerance.</summary>
    public readonly double Tolerance() => myTolerance;

    internal void Tolerance(double Tol)
    {
        myTolerance = Tol;
    }

    /// <summary>Whether the point is kept (not removed from the data structure).</summary>
    public readonly bool Keep() => myKeep;

    internal void ChangeKeep(bool b)
    {
        myKeep = b;
    }

    /// <summary>A copy with the tolerance <paramref name="Tol"/>.</summary>
    public readonly TopOpeBRepDS_Point WithTolerance(double Tol)
    {
        TopOpeBRepDS_Point aCopy = this;
        aCopy.Tolerance(Tol);
        return aCopy;
    }

    /// <inheritdoc/>
    public readonly bool Equals(TopOpeBRepDS_Point other)
        => myPoint.Equals(other.myPoint) && myTolerance.Equals(other.myTolerance) && myKeep == other.myKeep;

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is TopOpeBRepDS_Point other && Equals(other);

    /// <inheritdoc/>
    public override readonly int GetHashCode() => HashCode.Combine(myPoint, myTolerance, myKeep);

    public static bool operator ==(in TopOpeBRepDS_Point theLeft, in TopOpeBRepDS_Point theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in TopOpeBRepDS_Point theLeft, in TopOpeBRepDS_Point theRight) => !theLeft.Equals(theRight);

    /// <inheritdoc/>
    public override readonly string ToString()
        => "TopOpeBRepDS_Point(" + myPoint.ToString() + ", " + myTolerance.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + (myKeep ? ")" : ", not kept)");
}
