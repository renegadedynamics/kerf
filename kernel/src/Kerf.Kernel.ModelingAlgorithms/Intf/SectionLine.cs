// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_SectionLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_SectionLine.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_SectionLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Intersection.Polygons;

/// <summary>A polyline of intersection between two polyhedra, as a sequence of intersection points.</summary>
/// <remarks>
/// A class (it holds a sequence; decision 282): where OCCT copies a section line, ported code writes
/// <c>new Intf_SectionLine(other)</c> (which, as OCCT's copy constructor, doesn't copy the closed flag). Its mutators
/// are internal, so a line returned by <see cref="PolygonInterference.LineValue"/> (the stored one) can't be changed
/// by callers. OCCT's operator== is <see cref="IsEqual"/>; <c>Dump</c> (to std::cout) is left out.
/// </remarks>
internal sealed class SectionLine
{
    private readonly NCollection_Sequence<SectionPoint> _points;
    private bool _closed;

    /// <summary>Constructs an empty section line.</summary>
    public SectionLine()
    {
        _points = new NCollection_Sequence<SectionPoint>();
        _closed = false;
    }

    /// <summary>Copies a section line (its points; the copy is not closed).</summary>
    public SectionLine(SectionLine other)
    {
        _closed = false;
        _points = new NCollection_Sequence<SectionPoint>(other._points);
    }

    /// <summary>C++'s assignment operator: copies the points, not the closed flag.</summary>
    internal void Assign(SectionLine other)
    {
        // closed = theOther.closed; // not copied as in copy constructor
        _points.Assign(other._points);
    }

    /// <summary>The number of points in this section line.</summary>
    public int NumberOfPoints() => _points.Length();

    /// <summary>The intersection point of address <paramref name="index"/> in the section line.</summary>
    public SectionPoint GetPoint(int index) => _points.Value(index);

    /// <summary>True if the section line is closed: its first and last points are logically equal.</summary>
    public bool IsClosed
    {
        get
        {
            // return closed;
            //  On ne peut fermer une ligne de section inseree dans une liste car
            //  la fonction Value() copie l'element avant d'appeler la fonction.
            //  C'est donc la copie qui est modifiee.
            //  Pour la sequence myPoints on a un pointeur donc le pb ne se pose pas.
            return (_points.First().IsEqual(_points.Last()));
        }
    }

    /// <summary>True if <paramref name="thePI"/> is in the section line.</summary>
    public bool Contains(in SectionPoint thePI)
    {
        for (int i = 1; i <= _points.Length(); i++)
        {
            if (thePI.IsEqual(_points[i]))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// 1 if <paramref name="thePI"/> is the first point of the section line, the number of points if it is the
    /// last one, 0 otherwise.
    /// </summary>
    public int IsEnd(in SectionPoint thePI)
    {
        if (_points.First().IsEqual(thePI))
        {
            return 1;
        }

        if (_points.Last().IsEqual(thePI))
        {
            return _points.Length();
        }

        return 0;
    }

    /// <summary>True if both lines have logically equal points (<see cref="SectionPoint.IsEqual"/>) in the same order.</summary>
    public bool IsEqual(SectionLine other)
    {
        if (_points.Length() != other._points.Length())
        {
            return false;
        }

        for (int i = 1; i <= _points.Length(); i++)
        {
            if (!_points[i].IsEqual(other._points[i]))
            {
                return false;
            }
        }

        return true;
    }

    // Mutators (internal: see the remarks)

    /// <summary>Adds a point at the end of the section line.</summary>
    internal void Append(in SectionPoint pi)
    {
        _points.Append(pi);
    }

    /// <summary>Concatenates the section line <paramref name="ls"/> at the end of this one.</summary>
    internal void Append(SectionLine ls)
    {
        _points.Append(ls._points);
    }

    /// <summary>Adds a point at the beginning of the section line.</summary>
    internal void Prepend(in SectionPoint pi)
    {
        _points.Prepend(pi);
    }

    /// <summary>Concatenates the section line <paramref name="ls"/> at the beginning of this one.</summary>
    internal void Prepend(SectionLine ls)
    {
        _points.Prepend(ls._points);
    }

    /// <summary>Reverses the order of the points of the section line.</summary>
    internal void Reverse()
    {
        _points.Reverse();
    }

    /// <summary>Closes the section line (a flag that nothing reads: <see cref="IsClosed"/> compares the ends).</summary>
    internal void Close()
    {
        _closed = true;
    }

    public override string ToString()
    {
        System.Text.StringBuilder builder = new("Intf_SectionLine(");
        builder.Append(_closed ? "closed" : "open");
        for (int p = 1; p <= _points.Length(); p++)
        {
            builder.Append(", ").Append(_points[p].ToString());
        }

        return builder.Append(')').ToString();
    }
}
