// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_Domain.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_Domain.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_Domain.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;
using System.Globalization;

namespace Kerf.Engine.Classification.Hatching;

/// <summary>
/// A domain of a hatching: the part of its curve between two points on it, either of which may be at infinity.
/// </summary>
/// <remarks>
/// A working value (decision 664): a struct, copied as OCCT copies it, without equality (its points are classes).
/// Each setter stores its own copy of the point, as OCCT's assignment does, so a domain never changes with the
/// hatching it came from; copies of a domain share those points, which nothing changes. The setters are internal,
/// with <c>With</c> twins. <c>default</c> is OCCT's default: an infinite domain.
/// </remarks>
internal struct HatchingDomain
{
    private bool _hasFirstPoint;
    private PointOnHatching? _firstPoint;
    private bool _hasSecondPoint;
    private PointOnHatching? _secondPoint;

    /// <summary>An infinite domain.</summary>
    public HatchingDomain()
    {
        _hasFirstPoint = false;
        _hasSecondPoint = false;
    }

    /// <summary>The domain from <paramref name="p1"/> to <paramref name="p2"/>.</summary>
    public HatchingDomain(PointOnHatching p1, PointOnHatching p2)
    {
        _hasFirstPoint = true;
        _firstPoint = new PointOnHatching(p1);
        _hasSecondPoint = true;
        _secondPoint = new PointOnHatching(p2);
    }

    /// <summary>A semi-infinite domain: from <paramref name="p"/> if <paramref name="first"/>, up to it otherwise.</summary>
    public HatchingDomain(PointOnHatching p, bool first)
    {
        if (first)
        {
            _hasFirstPoint = true;
            _hasSecondPoint = false;
            _firstPoint = new PointOnHatching(p);
        }
        else
        {
            _hasFirstPoint = false;
            _hasSecondPoint = true;
            _secondPoint = new PointOnHatching(p);
        }
    }

    /// <summary>Sets the first and the second points of the domain.</summary>
    internal void SetPoints(PointOnHatching p1, PointOnHatching p2)
    {
        _hasFirstPoint = true;
        _firstPoint = new PointOnHatching(p1);
        _hasSecondPoint = true;
        _secondPoint = new PointOnHatching(p2);
    }

    /// <summary>Sets both points of the domain at infinity.</summary>
    internal void SetPoints()
    {
        _hasFirstPoint = false;
        _hasSecondPoint = false;
    }

    /// <summary>Sets the first point of the domain.</summary>
    internal void SetFirstPoint(PointOnHatching p)
    {
        _hasFirstPoint = true;
        _firstPoint = new PointOnHatching(p);
    }

    /// <summary>Sets the first point of the domain at infinity.</summary>
    internal void SetFirstPoint()
    {
        _hasFirstPoint = false;
    }

    /// <summary>Sets the second point of the domain.</summary>
    internal void SetSecondPoint(PointOnHatching p)
    {
        _hasSecondPoint = true;
        _secondPoint = new PointOnHatching(p);
    }

    /// <summary>Sets the second point of the domain at infinity.</summary>
    internal void SetSecondPoint()
    {
        _hasSecondPoint = false;
    }

    /// <summary>A copy with the points <paramref name="p1"/> and <paramref name="p2"/>.</summary>
    public readonly HatchingDomain WithPoints(PointOnHatching p1, PointOnHatching p2)
    {
        HatchingDomain copy = this;
        copy.SetPoints(p1, p2);
        return copy;
    }

    /// <summary>A copy with both points at infinity.</summary>
    public readonly HatchingDomain WithPoints()
    {
        HatchingDomain copy = this;
        copy.SetPoints();
        return copy;
    }

    /// <summary>A copy with the first point <paramref name="p"/>.</summary>
    public readonly HatchingDomain WithFirstPoint(PointOnHatching p)
    {
        HatchingDomain copy = this;
        copy.SetFirstPoint(p);
        return copy;
    }

    /// <summary>A copy with the first point at infinity.</summary>
    public readonly HatchingDomain WithFirstPoint()
    {
        HatchingDomain copy = this;
        copy.SetFirstPoint();
        return copy;
    }

    /// <summary>A copy with the second point <paramref name="p"/>.</summary>
    public readonly HatchingDomain WithSecondPoint(PointOnHatching p)
    {
        HatchingDomain copy = this;
        copy.SetSecondPoint(p);
        return copy;
    }

    /// <summary>A copy with the second point at infinity.</summary>
    public readonly HatchingDomain WithSecondPoint()
    {
        HatchingDomain copy = this;
        copy.SetSecondPoint();
        return copy;
    }

    /// <summary>True if the domain has a first point.</summary>
    public readonly bool HasFirstPoint => _hasFirstPoint;

    /// <summary>The first point of the domain.</summary>
    /// <exception cref="DomainErrorException">The domain has no first point.</exception>
    public readonly PointOnHatching FirstPoint()
    {
        Standard_DomainError_Raise_if(!_hasFirstPoint, "HatchGen_Domain::FirstPoint");
        return _firstPoint!;
    }

    /// <summary>True if the domain has a second point.</summary>
    public readonly bool HasSecondPoint => _hasSecondPoint;

    /// <summary>The second point of the domain.</summary>
    /// <exception cref="DomainErrorException">The domain has no second point.</exception>
    public readonly PointOnHatching SecondPoint()
    {
        Standard_DomainError_Raise_if(!_hasSecondPoint, "HatchGen_Domain::SecondPoint");
        return _secondPoint!;
    }

    /// <summary>Writes the domain to <paramref name="os"/>, as OCCT writes it to std::cout.</summary>
    public readonly void Dump(TextWriter os, int index = 0)
    {
        os.Write("=== Domain ");
        if (index > 0)
        {
            os.Write($"# {index,3} ");
        }
        else
        {
            os.Write("======");
        }
        os.Write("=============================\n");

        if (_hasFirstPoint)
        {
            _firstPoint!.Dump(os, 1);
        }
        else
        {
            os.Write("    Has not a first point\n");
        }

        if (_hasSecondPoint)
        {
            _secondPoint!.Dump(os, 2);
        }
        else
        {
            os.Write("    Has not a second point\n");
        }

        os.Write("==============================================\n");
    }

    /// <summary>The dump of the domain.</summary>
    public override readonly string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
