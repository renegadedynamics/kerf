// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_Domain.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_Domain.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_Domain.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// A domain of a hatching: the part of its curve between two points on it, either of which may be at infinity.
/// </summary>
/// <remarks>
/// A working value (decision 5802): a struct, copied as OCCT copies it, without equality (its points are classes).
/// Each setter stores its own copy of the point, as OCCT's assignment does, so a domain never changes with the
/// hatching it came from; copies of a domain share those points, which nothing changes. The setters are internal,
/// with <c>With</c> twins. <c>default</c> is OCCT's default: an infinite domain.
/// </remarks>
public struct HatchGen_Domain
{
    private bool myHasFirstPoint;
    private HatchGen_PointOnHatching? myFirstPoint;
    private bool myHasSecondPoint;
    private HatchGen_PointOnHatching? mySecondPoint;

    /// <summary>An infinite domain.</summary>
    public HatchGen_Domain()
    {
        myHasFirstPoint = false;
        myHasSecondPoint = false;
    }

    /// <summary>The domain from <paramref name="P1"/> to <paramref name="P2"/>.</summary>
    public HatchGen_Domain(HatchGen_PointOnHatching P1, HatchGen_PointOnHatching P2)
    {
        myHasFirstPoint = true;
        myFirstPoint = new HatchGen_PointOnHatching(P1);
        myHasSecondPoint = true;
        mySecondPoint = new HatchGen_PointOnHatching(P2);
    }

    /// <summary>A semi-infinite domain: from <paramref name="P"/> if <paramref name="First"/>, up to it otherwise.</summary>
    public HatchGen_Domain(HatchGen_PointOnHatching P, bool First)
    {
        if (First)
        {
            myHasFirstPoint = true;
            myHasSecondPoint = false;
            myFirstPoint = new HatchGen_PointOnHatching(P);
        }
        else
        {
            myHasFirstPoint = false;
            myHasSecondPoint = true;
            mySecondPoint = new HatchGen_PointOnHatching(P);
        }
    }

    /// <summary>Sets the first and the second points of the domain.</summary>
    internal void SetPoints(HatchGen_PointOnHatching P1, HatchGen_PointOnHatching P2)
    {
        myHasFirstPoint = true;
        myFirstPoint = new HatchGen_PointOnHatching(P1);
        myHasSecondPoint = true;
        mySecondPoint = new HatchGen_PointOnHatching(P2);
    }

    /// <summary>Sets both points of the domain at infinity.</summary>
    internal void SetPoints()
    {
        myHasFirstPoint = false;
        myHasSecondPoint = false;
    }

    /// <summary>Sets the first point of the domain.</summary>
    internal void SetFirstPoint(HatchGen_PointOnHatching P)
    {
        myHasFirstPoint = true;
        myFirstPoint = new HatchGen_PointOnHatching(P);
    }

    /// <summary>Sets the first point of the domain at infinity.</summary>
    internal void SetFirstPoint()
    {
        myHasFirstPoint = false;
    }

    /// <summary>Sets the second point of the domain.</summary>
    internal void SetSecondPoint(HatchGen_PointOnHatching P)
    {
        myHasSecondPoint = true;
        mySecondPoint = new HatchGen_PointOnHatching(P);
    }

    /// <summary>Sets the second point of the domain at infinity.</summary>
    internal void SetSecondPoint()
    {
        myHasSecondPoint = false;
    }

    /// <summary>A copy with the points <paramref name="P1"/> and <paramref name="P2"/>.</summary>
    public readonly HatchGen_Domain WithPoints(HatchGen_PointOnHatching P1, HatchGen_PointOnHatching P2)
    {
        HatchGen_Domain aCopy = this;
        aCopy.SetPoints(P1, P2);
        return aCopy;
    }

    /// <summary>A copy with both points at infinity.</summary>
    public readonly HatchGen_Domain WithPoints()
    {
        HatchGen_Domain aCopy = this;
        aCopy.SetPoints();
        return aCopy;
    }

    /// <summary>A copy with the first point <paramref name="P"/>.</summary>
    public readonly HatchGen_Domain WithFirstPoint(HatchGen_PointOnHatching P)
    {
        HatchGen_Domain aCopy = this;
        aCopy.SetFirstPoint(P);
        return aCopy;
    }

    /// <summary>A copy with the first point at infinity.</summary>
    public readonly HatchGen_Domain WithFirstPoint()
    {
        HatchGen_Domain aCopy = this;
        aCopy.SetFirstPoint();
        return aCopy;
    }

    /// <summary>A copy with the second point <paramref name="P"/>.</summary>
    public readonly HatchGen_Domain WithSecondPoint(HatchGen_PointOnHatching P)
    {
        HatchGen_Domain aCopy = this;
        aCopy.SetSecondPoint(P);
        return aCopy;
    }

    /// <summary>A copy with the second point at infinity.</summary>
    public readonly HatchGen_Domain WithSecondPoint()
    {
        HatchGen_Domain aCopy = this;
        aCopy.SetSecondPoint();
        return aCopy;
    }

    /// <summary>True if the domain has a first point.</summary>
    public readonly bool HasFirstPoint() => myHasFirstPoint;

    /// <summary>The first point of the domain.</summary>
    /// <exception cref="Standard_DomainError">The domain has no first point.</exception>
    public readonly HatchGen_PointOnHatching FirstPoint()
    {
        Standard_DomainError_Raise_if(!myHasFirstPoint, "HatchGen_Domain::FirstPoint");
        return myFirstPoint!;
    }

    /// <summary>True if the domain has a second point.</summary>
    public readonly bool HasSecondPoint() => myHasSecondPoint;

    /// <summary>The second point of the domain.</summary>
    /// <exception cref="Standard_DomainError">The domain has no second point.</exception>
    public readonly HatchGen_PointOnHatching SecondPoint()
    {
        Standard_DomainError_Raise_if(!myHasSecondPoint, "HatchGen_Domain::SecondPoint");
        return mySecondPoint!;
    }

    /// <summary>Writes the domain to <paramref name="theOS"/>, as OCCT writes it to std::cout.</summary>
    public readonly void Dump(TextWriter theOS, int Index = 0)
    {
        theOS.Write("=== Domain ");
        if (Index > 0)
        {
            theOS.Write($"# {Index,3} ");
        }
        else
        {
            theOS.Write("======");
        }
        theOS.Write("=============================\n");

        if (myHasFirstPoint)
        {
            myFirstPoint!.Dump(theOS, 1);
        }
        else
        {
            theOS.Write("    Has not a first point\n");
        }

        if (myHasSecondPoint)
        {
            mySecondPoint!.Dump(theOS, 2);
        }
        else
        {
            theOS.Write("    Has not a second point\n");
        }

        theOS.Write("==============================================\n");
    }

    /// <summary>The dump of the domain.</summary>
    public override readonly string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
