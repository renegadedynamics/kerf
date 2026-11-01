// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_TheSegmentOfTheSearch.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_TheSegmentOfTheSearch_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A solution segment of the boundary search (<see cref="Contap_TheSearch"/>): a part of an arc of the domain
/// on the contour, with its first and last points when they are known.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct (decision 609); <c>new()</c> and <c>default</c> are the same. The mutators are
/// internal (reached by the boundary search through <see cref="IIntStart_Segment{ThePathPoint}"/>).
/// </remarks>
public partial struct Contap_TheSegmentOfTheSearch
    : IIntStart_Segment<Contap_ThePathPointOfTheSearch>, IEquatable<Contap_TheSegmentOfTheSearch>
{
    private Adaptor2d_Curve2d? arc;
    private bool hasfp;
    private Contap_ThePathPointOfTheSearch thefp;
    private bool haslp;
    private Contap_ThePathPointOfTheSearch thelp;

    /// <summary>Empty constructor.</summary>
    public Contap_TheSegmentOfTheSearch()
    {
        arc = null;
        hasfp = false;
        thefp = new Contap_ThePathPointOfTheSearch();
        haslp = false;
        thelp = new Contap_ThePathPointOfTheSearch();
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the arc (and forgets the limit points).</summary>
    internal void SetValue(Adaptor2d_Curve2d A)
    {
        hasfp = false;
        haslp = false;
        arc = A;
    }

    /// <summary>Sets the first point (<paramref name="First"/>) or the last.</summary>
    internal void SetLimitPoint(in Contap_ThePathPointOfTheSearch V, bool First)
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
    public readonly Adaptor2d_Curve2d Curve() => arc!;

    /// <summary>True if a point bounds the segment at its lowest parameter on the arc.</summary>
    public readonly bool HasFirstPoint() => hasfp;

    /// <summary>The first point; throws Standard_DomainError if there is none.</summary>
    public readonly Contap_ThePathPointOfTheSearch FirstPoint()
    {
        if (!hasfp)
        {
            throw new Standard_DomainError();
        }
        return thefp;
    }

    /// <summary>True if a point bounds the segment at its greatest parameter on the arc.</summary>
    public readonly bool HasLastPoint() => haslp;

    /// <summary>The last point; throws Standard_DomainError if there is none.</summary>
    public readonly Contap_ThePathPointOfTheSearch LastPoint()
    {
        if (!haslp)
        {
            throw new Standard_DomainError();
        }
        return thelp;
    }

    void IIntStart_Segment<Contap_ThePathPointOfTheSearch>.SetValue(Adaptor2d_Curve2d A) => SetValue(A);

    void IIntStart_Segment<Contap_ThePathPointOfTheSearch>.SetLimitPoint(in Contap_ThePathPointOfTheSearch V,
                                                                              bool First) => SetLimitPoint(V, First);

    public readonly bool Equals(Contap_TheSegmentOfTheSearch other)
        => ReferenceEquals(arc, other.arc) && hasfp == other.hasfp && thefp.Equals(other.thefp) && haslp == other.haslp
           && thelp.Equals(other.thelp);

    public override readonly bool Equals(object? obj) => obj is Contap_TheSegmentOfTheSearch other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(arc, hasfp, thefp, haslp, thelp);

    public static bool operator ==(in Contap_TheSegmentOfTheSearch theLeft,
                                   in Contap_TheSegmentOfTheSearch theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Contap_TheSegmentOfTheSearch theLeft,
                                   in Contap_TheSegmentOfTheSearch theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"Contap_TheSegmentOfTheSearch(first {(hasfp ? thefp.ToString() : "none")}, last {(haslp ? thelp.ToString() : "none")})";
}
