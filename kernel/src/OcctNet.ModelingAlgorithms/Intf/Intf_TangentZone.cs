// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_TangentZone.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_TangentZone.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_TangentZone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A zone of tangency between polygons or polyhedra, as a sequence of intersection points.</summary>
/// <remarks>
/// A class (it holds a sequence; decision 282): where OCCT copies a tangent zone, ported code writes
/// <c>new Intf_TangentZone(other)</c>. Its mutators are internal, so a zone returned by
/// <see cref="Intf_Interference.ZoneValue"/> (the stored one, as OCCT's const reference) can't be changed by callers.
/// OCCT's operator== is <see cref="IsEqual"/>; <c>Dump</c> prints only in a debug build and is left out.
/// <para>
/// OCCT's sequence is a linked list, so <see cref="PolygonInsert"/>'s insertions at the front cost nothing; the
/// points are kept here in a <see cref="Points"/> deque for the same reason (unit 9E: with a .NET list, merging the
/// zones of two long coincident polygons was quadratic). The order of the points is OCCT's.
/// </para>
/// </remarks>
public sealed class Intf_TangentZone
{
    private readonly Points Result;
    private double ParamOnFirstMin;
    private double ParamOnFirstMax;
    private double ParamOnSecondMin;
    private double ParamOnSecondMax;

    /// <summary>Builds an empty tangent zone.</summary>
    public Intf_TangentZone()
    {
        Result = new Points();
        ParamOnFirstMin = ParamOnSecondMin = RealLast();
        ParamOnFirstMax = ParamOnSecondMax = RealFirst();
    }

    /// <summary>C++'s copy constructor.</summary>
    public Intf_TangentZone(Intf_TangentZone theOther)
    {
        Result = new Points(theOther.Result);
        ParamOnFirstMin = theOther.ParamOnFirstMin;
        ParamOnFirstMax = theOther.ParamOnFirstMax;
        ParamOnSecondMin = theOther.ParamOnSecondMin;
        ParamOnSecondMax = theOther.ParamOnSecondMax;
    }

    /// <summary>The number of section points in this tangent zone.</summary>
    public int NumberOfPoints() => Result.Length();

    /// <summary>The section point of address <paramref name="Index"/> in the tangent zone.</summary>
    public Intf_SectionPoint GetPoint(int Index) => Result[Index];

    /// <summary>True if both zones have logically equal points (<see cref="Intf_SectionPoint.IsEqual"/>) in the same order.</summary>
    public bool IsEqual(Intf_TangentZone Other)
    {
        if (Result.Length() != Other.Result.Length())
        {
            return false;
        }

        int i;
        for (i = 1; i <= Result.Length(); i++)
        {
            if (!Result[i].IsEqual(Other.Result[i]))
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>True if <paramref name="ThePI"/> is in the tangent zone.</summary>
    public bool Contains(in Intf_SectionPoint ThePI)
    {
        int i;
        for (i = 1; i <= Result.Length(); i++)
        {
            if (ThePI.IsEqual(Result[i]))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>The parameter range of the zone on the first argument of the interference (polygons only).</summary>
    public void ParamOnFirst(out double paraMin, out double paraMax)
    {
        paraMin = ParamOnFirstMin;
        paraMax = ParamOnFirstMax;
    }

    /// <summary>The parameter range of the zone on the second argument of the interference (polygons only).</summary>
    public void ParamOnSecond(out double paraMin, out double paraMax)
    {
        paraMin = ParamOnSecondMin;
        paraMax = ParamOnSecondMax;
    }

    /// <summary>The segments and parameters bounding the zone on the first argument of the interference (polygons only).</summary>
    public void InfoFirst(out int segMin, out double paraMin, out int segMax, out double paraMax)
    {
        ParamOnFirst(out paraMin, out paraMax);
        segMin = (int)(Math.Truncate(paraMin));
        paraMin = paraMin - (double)segMin;
        segMax = (int)(Math.Truncate(paraMax));
        paraMax = paraMax - (double)segMax;
    }

    /// <summary>The segments and parameters bounding the zone on the second argument of the interference (polygons only).</summary>
    public void InfoSecond(out int segMin, out double paraMin, out int segMax, out double paraMax)
    {
        ParamOnSecond(out paraMin, out paraMax);
        segMin = (int)(Math.Truncate(paraMin));
        paraMin = paraMin - (double)segMin;
        segMax = (int)(Math.Truncate(paraMax));
        paraMax = paraMax - (double)segMax;
    }

    /// <summary>True if <paramref name="ThePI"/> is in the parameter range of the tangent zone.</summary>
    public bool RangeContains(in Intf_SectionPoint ThePI)
    {
        double a, b, c, d;
        ParamOnFirst(out a, out b);
        ParamOnSecond(out c, out d);
        return a <= ThePI.ParamOnFirst() && ThePI.ParamOnFirst() <= b && c <= ThePI.ParamOnSecond()
               && ThePI.ParamOnSecond() <= d;
    }

    /// <summary>True if the tangent zone <paramref name="Other"/> has a common part with this one.</summary>
    public bool HasCommonRange(Intf_TangentZone Other)
    {
        double a1, b1, c1, d1;
        double a2, b2, c2, d2;
        ParamOnFirst(out a1, out b1);
        ParamOnSecond(out a2, out b2);
        Other.ParamOnFirst(out c1, out d1);
        Other.ParamOnSecond(out c2, out d2);

        return ((c1 <= a1 && a1 <= d1) || (c1 <= b1 && b1 <= d1) || (a1 <= c1 && c1 <= b1))
               && ((c2 <= a2 && a2 <= d2) || (c2 <= b2 && b2 <= d2) || (a2 <= c2 && c2 <= b2));
    }

    // Mutators (internal: see the remarks)

    /// <summary>Adds a section point to the tangent zone.</summary>
    internal void Append(in Intf_SectionPoint Pi)
    {
        Result.Append(Pi);
        if (ParamOnFirstMin > Pi.ParamOnFirst())
        {
            ParamOnFirstMin = Pi.ParamOnFirst();
        }

        if (ParamOnSecondMin > Pi.ParamOnSecond())
        {
            ParamOnSecondMin = Pi.ParamOnSecond();
        }

        if (ParamOnFirstMax < Pi.ParamOnFirst())
        {
            ParamOnFirstMax = Pi.ParamOnFirst();
        }

        if (ParamOnSecondMax < Pi.ParamOnSecond())
        {
            ParamOnSecondMax = Pi.ParamOnSecond();
        }
    }

    /// <summary>Adds the points of the tangent zone <paramref name="Tzi"/> to this one.</summary>
    internal void Append(Intf_TangentZone Tzi)
    {
        int Tzi_NumberOfPoints = Tzi.NumberOfPoints();
        for (int ipi = 1; ipi <= Tzi_NumberOfPoints; ipi++)
        {
            PolygonInsert(Tzi.GetPoint(ipi));
        }
    }

    /// <summary>Inserts a section point in the tangent zone: OCCT's body is commented out, so nothing is inserted.</summary>
    internal bool Insert(in Intf_SectionPoint Pi)
    {
        _ = Pi;
        bool Inserted = false;
        return Inserted;
    }

    /// <summary>
    /// Inserts a point in the polygonal tangent zone: appended if its parameter on the first argument is the
    /// highest, inserted first if it is within the range, appended otherwise.
    /// </summary>
    internal void PolygonInsert(in Intf_SectionPoint Pi)
    {
        int nbpTz = NumberOfPoints();
        if (nbpTz == 0)
        {
            Append(Pi);
            return;
        }

        if (Pi.ParamOnFirst() >= ParamOnFirstMax)
        {
            Append(Pi);
        }
        else if (Pi.ParamOnFirst() >= ParamOnFirstMin)
        {
            InsertBefore(1, Pi);
        }
        else
        {
            Append(Pi); //-- On met les points sans les classer
                        //-- si on veut on pourra les reclasser
                        //-- ensuite avec un TRI.
        }
    }

    /// <summary>Inserts a section point before <paramref name="Index"/> in the tangent zone.</summary>
    internal void InsertBefore(int Index, in Intf_SectionPoint Pi)
    {
        Result.InsertBefore(Index, Pi);
        if (ParamOnFirstMin > Pi.ParamOnFirst())
        {
            ParamOnFirstMin = Pi.ParamOnFirst();
        }

        if (ParamOnSecondMin > Pi.ParamOnSecond())
        {
            ParamOnSecondMin = Pi.ParamOnSecond();
        }

        if (ParamOnFirstMax < Pi.ParamOnFirst())
        {
            ParamOnFirstMax = Pi.ParamOnFirst();
        }

        if (ParamOnSecondMax < Pi.ParamOnSecond())
        {
            ParamOnSecondMax = Pi.ParamOnSecond();
        }
    }

    /// <summary>Inserts a section point after <paramref name="Index"/> in the tangent zone.</summary>
    internal void InsertAfter(int Index, in Intf_SectionPoint Pi)
    {
        Result.InsertAfter(Index, Pi);
        if (ParamOnFirstMin > Pi.ParamOnFirst())
        {
            ParamOnFirstMin = Pi.ParamOnFirst();
        }

        if (ParamOnSecondMin > Pi.ParamOnSecond())
        {
            ParamOnSecondMin = Pi.ParamOnSecond();
        }

        if (ParamOnFirstMax < Pi.ParamOnFirst())
        {
            ParamOnFirstMax = Pi.ParamOnFirst();
        }

        if (ParamOnSecondMax < Pi.ParamOnSecond())
        {
            ParamOnSecondMax = Pi.ParamOnSecond();
        }
    }

    /// <summary>
    /// OCCT's sequence of section points, indexed from 1, as a deque: the points before the first appended one are
    /// held in reverse in <c>myFront</c>, the others in <c>myBack</c>, so inserting at either end is cheap.
    /// </summary>
    private sealed class Points
    {
        private readonly List<Intf_SectionPoint> myFront;
        private readonly List<Intf_SectionPoint> myBack;

        public Points()
        {
            myFront = [];
            myBack = [];
        }

        public Points(Points theOther)
        {
            myFront = [.. theOther.myFront];
            myBack = [.. theOther.myBack];
        }

        public int Length() => myFront.Count + myBack.Count;

        public Intf_SectionPoint this[int theIndex]
        {
            get
            {
                Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > Length(), "NCollection_Sequence::Value");
                int aNbFront = myFront.Count;
                return theIndex <= aNbFront ? myFront[aNbFront - theIndex] : myBack[theIndex - aNbFront - 1];
            }
        }

        public void Append(in Intf_SectionPoint thePoint) => myBack.Add(thePoint);

        public void InsertBefore(int theIndex, in Intf_SectionPoint thePoint)
        {
            Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > Length() + 1, "NCollection_Sequence::InsertBefore");
            int aPosition = theIndex - 1;
            int aNbFront = myFront.Count;
            if (aPosition <= aNbFront)
            {
                myFront.Insert(aNbFront - aPosition, thePoint);
            }
            else
            {
                myBack.Insert(aPosition - aNbFront, thePoint);
            }
        }

        public void InsertAfter(int theIndex, in Intf_SectionPoint thePoint)
        {
            Standard_OutOfRange_Raise_if(theIndex < 0 || theIndex > Length(), "NCollection_Sequence::InsertAfter");
            InsertBefore(theIndex + 1, thePoint);
        }
    }

    public override string ToString()
    {
        System.Text.StringBuilder aBuilder = new("Intf_TangentZone(");
        aBuilder.Append(gp.Format("first", ParamOnFirstMin, ParamOnFirstMax)).Append(", ");
        aBuilder.Append(gp.Format("second", ParamOnSecondMin, ParamOnSecondMax));
        for (int p = 1; p <= Result.Length(); p++)
        {
            aBuilder.Append(", ").Append(Result[p].ToString());
        }

        return aBuilder.Append(')').ToString();
    }
}
