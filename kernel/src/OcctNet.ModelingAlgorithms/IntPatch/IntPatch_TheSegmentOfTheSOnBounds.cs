// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheSegmentOfTheSOnBounds.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheSegmentOfTheSOnBounds_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;

namespace Occt;

/// <summary>
/// A solution segment of the boundary search (<see cref="IntPatch_TheSOnBounds"/>): a part of an arc of the domain
/// on the quadric, with its first and last points when they are known.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct (decision 609); <c>new()</c> and <c>default</c> are the same. The mutators are
/// internal (reached by the boundary search through <see cref="IIntStart_Segment{ThePathPoint}"/>).
/// </remarks>
public partial struct IntPatch_TheSegmentOfTheSOnBounds
    : IIntStart_Segment<IntPatch_ThePathPointOfTheSOnBounds>, IEquatable<IntPatch_TheSegmentOfTheSOnBounds>
{
    private Curve2DAdaptorBase? arc;
    private bool hasfp;
    private IntPatch_ThePathPointOfTheSOnBounds thefp;
    private bool haslp;
    private IntPatch_ThePathPointOfTheSOnBounds thelp;

    /// <summary>Empty constructor.</summary>
    public IntPatch_TheSegmentOfTheSOnBounds()
    {
        arc = null;
        hasfp = false;
        thefp = new IntPatch_ThePathPointOfTheSOnBounds();
        haslp = false;
        thelp = new IntPatch_ThePathPointOfTheSOnBounds();
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the arc (and forgets the limit points).</summary>
    internal void SetValue(Curve2DAdaptorBase A)
    {
        hasfp = false;
        haslp = false;
        arc = A;
    }

    /// <summary>Sets the first point (<paramref name="First"/>) or the last.</summary>
    internal void SetLimitPoint(in IntPatch_ThePathPointOfTheSOnBounds V, bool First)
    {
        if (First)
        {
            hasfp = true;
            thefp = V;
        }
        else
        {
            haslp = true;
            thelp = V;
        }
    }

    // Queries

    /// <summary>The arc of the domain that is a solution.</summary>
    public readonly Curve2DAdaptorBase Curve() => arc!;

    /// <summary>True if a point bounds the segment at its lowest parameter on the arc.</summary>
    public readonly bool HasFirstPoint() => hasfp;

    /// <summary>The first point; throws Standard_DomainError if there is none.</summary>
    public readonly IntPatch_ThePathPointOfTheSOnBounds FirstPoint()
    {
        if (!hasfp)
        {
            throw new DomainErrorException();
        }
        return thefp;
    }

    /// <summary>True if a point bounds the segment at its greatest parameter on the arc.</summary>
    public readonly bool HasLastPoint() => haslp;

    /// <summary>The last point; throws Standard_DomainError if there is none.</summary>
    public readonly IntPatch_ThePathPointOfTheSOnBounds LastPoint()
    {
        if (!haslp)
        {
            throw new DomainErrorException();
        }
        return thelp;
    }

    void IIntStart_Segment<IntPatch_ThePathPointOfTheSOnBounds>.SetValue(Curve2DAdaptorBase A) => SetValue(A);

    void IIntStart_Segment<IntPatch_ThePathPointOfTheSOnBounds>.SetLimitPoint(in IntPatch_ThePathPointOfTheSOnBounds V,
                                                                              bool First) => SetLimitPoint(V, First);

    public readonly bool Equals(IntPatch_TheSegmentOfTheSOnBounds other)
        => ReferenceEquals(arc, other.arc) && hasfp == other.hasfp && thefp.Equals(other.thefp) && haslp == other.haslp
           && thelp.Equals(other.thelp);

    public override readonly bool Equals(object? obj) => obj is IntPatch_TheSegmentOfTheSOnBounds other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(arc, hasfp, thefp, haslp, thelp);

    public static bool operator ==(in IntPatch_TheSegmentOfTheSOnBounds theLeft,
                                   in IntPatch_TheSegmentOfTheSOnBounds theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntPatch_TheSegmentOfTheSOnBounds theLeft,
                                   in IntPatch_TheSegmentOfTheSOnBounds theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"IntPatch_TheSegmentOfTheSOnBounds(first {(hasfp ? thefp.ToString() : "none")}, last {(haslp ? thelp.ToString() : "none")})";
}
