// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheSegmentOfTheSOnBounds.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_TheSegmentOfTheSOnBounds_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// A solution segment of the boundary search (<see cref="TheSOnBounds"/>): a part of an arc of the domain
/// on the quadric, with its first and last points when they are known.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct (decision 609); <c>new()</c> and <c>default</c> are the same. The mutators are
/// internal (reached by the boundary search through <see cref="ISegment{ThePathPoint}"/>).
/// </remarks>
internal partial struct TheSegmentOfTheSOnBounds
    : ISegment<ThePathPointOfTheSOnBounds>, IEquatable<TheSegmentOfTheSOnBounds>
{
    private Curve2DAdaptorBase? _arc;
    private bool _hasfp;
    private ThePathPointOfTheSOnBounds _thefp;
    private bool _haslp;
    private ThePathPointOfTheSOnBounds _thelp;

    /// <summary>Empty constructor.</summary>
    public TheSegmentOfTheSOnBounds()
    {
        _arc = null;
        _hasfp = false;
        _thefp = new ThePathPointOfTheSOnBounds();
        _haslp = false;
        _thelp = new ThePathPointOfTheSOnBounds();
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the arc (and forgets the limit points).</summary>
    internal void SetValue(Curve2DAdaptorBase a)
    {
        _hasfp = false;
        _haslp = false;
        _arc = a;
    }

    /// <summary>Sets the first point (<paramref name="first"/>) or the last.</summary>
    internal void SetLimitPoint(in ThePathPointOfTheSOnBounds v, bool first)
    {
        if (first)
        {
            _hasfp = true;
            _thefp = v;
        }
        else
        {
            _haslp = true;
            _thelp = v;
        }
    }

    // Queries

    /// <summary>The arc of the domain that is a solution.</summary>
    public readonly Curve2DAdaptorBase Curve() => _arc!;

    /// <summary>True if a point bounds the segment at its lowest parameter on the arc.</summary>
    public readonly bool HasFirstPoint => _hasfp;

    /// <summary>The first point; throws Standard_DomainError if there is none.</summary>
    public readonly ThePathPointOfTheSOnBounds FirstPoint()
    {
        if (!_hasfp)
        {
            throw new DomainErrorException();
        }
        return _thefp;
    }

    /// <summary>True if a point bounds the segment at its greatest parameter on the arc.</summary>
    public readonly bool HasLastPoint => _haslp;

    /// <summary>The last point; throws Standard_DomainError if there is none.</summary>
    public readonly ThePathPointOfTheSOnBounds LastPoint()
    {
        if (!_haslp)
        {
            throw new DomainErrorException();
        }
        return _thelp;
    }

    void ISegment<ThePathPointOfTheSOnBounds>.SetValue(Curve2DAdaptorBase a) => SetValue(a);

    void ISegment<ThePathPointOfTheSOnBounds>.SetLimitPoint(in ThePathPointOfTheSOnBounds v,
                                                                              bool first) => SetLimitPoint(v, first);

    public readonly bool Equals(TheSegmentOfTheSOnBounds other)
        => ReferenceEquals(_arc, other._arc) && _hasfp == other._hasfp && _thefp.Equals(other._thefp) && _haslp == other._haslp
           && _thelp.Equals(other._thelp);

    public override readonly bool Equals(object? obj) => obj is TheSegmentOfTheSOnBounds other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_arc, _hasfp, _thefp, _haslp, _thelp);

    public static bool operator ==(in TheSegmentOfTheSOnBounds left,
                                   in TheSegmentOfTheSOnBounds right) => left.Equals(right);

    public static bool operator !=(in TheSegmentOfTheSOnBounds left,
                                   in TheSegmentOfTheSOnBounds right) => !left.Equals(right);

    public override readonly string ToString()
        => $"IntPatch_TheSegmentOfTheSOnBounds(first {(_hasfp ? _thefp.ToString() : "none")}, last {(_haslp ? _thelp.ToString() : "none")})";
}
