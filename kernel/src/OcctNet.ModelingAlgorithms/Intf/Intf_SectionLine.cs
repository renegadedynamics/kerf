// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_SectionLine.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_SectionLine.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_SectionLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A polyline of intersection between two polyhedra, as a sequence of intersection points.</summary>
/// <remarks>
/// A class (it holds a sequence; decision 282): where OCCT copies a section line, ported code writes
/// <c>new Intf_SectionLine(other)</c> (which, as OCCT's copy constructor, doesn't copy the closed flag). Its mutators
/// are internal, so a line returned by <see cref="Intf_Interference.LineValue"/> (the stored one) can't be changed
/// by callers. OCCT's operator== is <see cref="IsEqual"/>; <c>Dump</c> (to std::cout) is left out.
/// </remarks>
public sealed class Intf_SectionLine
{
    private readonly NCollection_Sequence<Intf_SectionPoint> myPoints;
    private bool closed;

    /// <summary>Constructs an empty section line.</summary>
    public Intf_SectionLine()
    {
        myPoints = new NCollection_Sequence<Intf_SectionPoint>();
        closed = false;
    }

    /// <summary>Copies a section line (its points; the copy is not closed).</summary>
    public Intf_SectionLine(Intf_SectionLine Other)
    {
        closed = false;
        myPoints = new NCollection_Sequence<Intf_SectionPoint>(Other.myPoints);
    }

    /// <summary>C++'s assignment operator: copies the points, not the closed flag.</summary>
    internal void Assign(Intf_SectionLine theOther)
    {
        // closed = theOther.closed; // not copied as in copy constructor
        myPoints.Assign(theOther.myPoints);
    }

    /// <summary>The number of points in this section line.</summary>
    public int NumberOfPoints() => myPoints.Length();

    /// <summary>The intersection point of address <paramref name="index"/> in the section line.</summary>
    public Intf_SectionPoint GetPoint(int index) => myPoints.Value(index);

    /// <summary>True if the section line is closed: its first and last points are logically equal.</summary>
    public bool IsClosed()
    {
        // return closed;
        //  On ne peut fermer une ligne de section inseree dans une liste car
        //  la fonction Value() copie l'element avant d'appeler la fonction.
        //  C'est donc la copie qui est modifiee.
        //  Pour la sequence myPoints on a un pointeur donc le pb ne se pose pas.
        return (myPoints.First().IsEqual(myPoints.Last()));
    }

    /// <summary>True if <paramref name="ThePI"/> is in the section line.</summary>
    public bool Contains(in Intf_SectionPoint ThePI)
    {
        for (int i = 1; i <= myPoints.Length(); i++)
        {
            if (ThePI.IsEqual(myPoints[i]))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// 1 if <paramref name="ThePI"/> is the first point of the section line, the number of points if it is the
    /// last one, 0 otherwise.
    /// </summary>
    public int IsEnd(in Intf_SectionPoint ThePI)
    {
        if (myPoints.First().IsEqual(ThePI))
        {
            return 1;
        }

        if (myPoints.Last().IsEqual(ThePI))
        {
            return myPoints.Length();
        }

        return 0;
    }

    /// <summary>True if both lines have logically equal points (<see cref="Intf_SectionPoint.IsEqual"/>) in the same order.</summary>
    public bool IsEqual(Intf_SectionLine Other)
    {
        if (myPoints.Length() != Other.myPoints.Length())
        {
            return false;
        }

        for (int i = 1; i <= myPoints.Length(); i++)
        {
            if (!myPoints[i].IsEqual(Other.myPoints[i]))
            {
                return false;
            }
        }

        return true;
    }

    // Mutators (internal: see the remarks)

    /// <summary>Adds a point at the end of the section line.</summary>
    internal void Append(in Intf_SectionPoint Pi)
    {
        myPoints.Append(Pi);
    }

    /// <summary>Concatenates the section line <paramref name="LS"/> at the end of this one.</summary>
    internal void Append(Intf_SectionLine LS)
    {
        myPoints.Append(LS.myPoints);
    }

    /// <summary>Adds a point at the beginning of the section line.</summary>
    internal void Prepend(in Intf_SectionPoint Pi)
    {
        myPoints.Prepend(Pi);
    }

    /// <summary>Concatenates the section line <paramref name="LS"/> at the beginning of this one.</summary>
    internal void Prepend(Intf_SectionLine LS)
    {
        myPoints.Prepend(LS.myPoints);
    }

    /// <summary>Reverses the order of the points of the section line.</summary>
    internal void Reverse()
    {
        myPoints.Reverse();
    }

    /// <summary>Closes the section line (a flag that nothing reads: <see cref="IsClosed"/> compares the ends).</summary>
    internal void Close()
    {
        closed = true;
    }

    public override string ToString()
    {
        System.Text.StringBuilder aBuilder = new("Intf_SectionLine(");
        aBuilder.Append(closed ? "closed" : "open");
        for (int p = 1; p <= myPoints.Length(); p++)
        {
            aBuilder.Append(", ").Append(myPoints[p].ToString());
        }

        return aBuilder.Append(')').ToString();
    }
}
