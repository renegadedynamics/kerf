// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_TangentZone.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_TangentZone.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_TangentZone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Intersection.Polygons;

/// <summary>A zone of tangency between polygons or polyhedra, as a sequence of intersection points.</summary>
/// <remarks>
/// A class (it holds a sequence; decision 282): where OCCT copies a tangent zone, ported code writes
/// <c>new Intf_TangentZone(other)</c>. Its mutators are internal, so a zone returned by
/// <see cref="PolygonInterference.ZoneValue"/> (the stored one, as OCCT's const reference) can't be changed by callers.
/// OCCT's operator== is <see cref="IsEqual"/>; <c>Dump</c> prints only in a debug build and is left out.
/// <para>
/// OCCT's sequence is a linked list, so <see cref="PolygonInsert"/>'s insertions at the front cost nothing; the
/// points are kept here in a <see cref="Points"/> deque for the same reason (unit 9E: with a .NET list, merging the
/// zones of two long coincident polygons was quadratic). The order of the points is OCCT's.
/// </para>
/// </remarks>
internal sealed class TangentZone
{
    private readonly Points _result;
    private double _paramOnFirstMin;
    private double _paramOnFirstMax;
    private double _paramOnSecondMin;
    private double _paramOnSecondMax;

    /// <summary>Builds an empty tangent zone.</summary>
    public TangentZone()
    {
        _result = new Points();
        _paramOnFirstMin = _paramOnSecondMin = RealLast();
        _paramOnFirstMax = _paramOnSecondMax = RealFirst();
    }

    /// <summary>C++'s copy constructor.</summary>
    public TangentZone(TangentZone other)
    {
        _result = new Points(other._result);
        _paramOnFirstMin = other._paramOnFirstMin;
        _paramOnFirstMax = other._paramOnFirstMax;
        _paramOnSecondMin = other._paramOnSecondMin;
        _paramOnSecondMax = other._paramOnSecondMax;
    }

    /// <summary>The number of section points in this tangent zone.</summary>
    public int NumberOfPoints() => _result.Length();

    /// <summary>The section point of address <paramref name="index"/> in the tangent zone.</summary>
    public SectionPoint GetPoint(int index) => _result[index];

    /// <summary>True if both zones have logically equal points (<see cref="SectionPoint.IsEqual"/>) in the same order.</summary>
    public bool IsEqual(TangentZone other)
    {
        if (_result.Length() != other._result.Length())
        {
            return false;
        }

        int i;
        for (i = 1; i <= _result.Length(); i++)
        {
            if (!_result[i].IsEqual(other._result[i]))
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>True if <paramref name="thePI"/> is in the tangent zone.</summary>
    public bool Contains(in SectionPoint thePI)
    {
        int i;
        for (i = 1; i <= _result.Length(); i++)
        {
            if (thePI.IsEqual(_result[i]))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>The parameter range of the zone on the first argument of the interference (polygons only).</summary>
    public void ParamOnFirst(out double paraMin, out double paraMax)
    {
        paraMin = _paramOnFirstMin;
        paraMax = _paramOnFirstMax;
    }

    /// <summary>The parameter range of the zone on the second argument of the interference (polygons only).</summary>
    public void ParamOnSecond(out double paraMin, out double paraMax)
    {
        paraMin = _paramOnSecondMin;
        paraMax = _paramOnSecondMax;
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

    /// <summary>True if <paramref name="thePI"/> is in the parameter range of the tangent zone.</summary>
    public bool RangeContains(in SectionPoint thePI)
    {
        double a, b, c, d;
        ParamOnFirst(out a, out b);
        ParamOnSecond(out c, out d);
        return a <= thePI.ParamOnFirst() && thePI.ParamOnFirst() <= b && c <= thePI.ParamOnSecond()
               && thePI.ParamOnSecond() <= d;
    }

    /// <summary>True if the tangent zone <paramref name="other"/> has a common part with this one.</summary>
    public bool HasCommonRange(TangentZone other)
    {
        double a1, b1, c1, d1;
        double a2, b2, c2, d2;
        ParamOnFirst(out a1, out b1);
        ParamOnSecond(out a2, out b2);
        other.ParamOnFirst(out c1, out d1);
        other.ParamOnSecond(out c2, out d2);

        return ((c1 <= a1 && a1 <= d1) || (c1 <= b1 && b1 <= d1) || (a1 <= c1 && c1 <= b1))
               && ((c2 <= a2 && a2 <= d2) || (c2 <= b2 && b2 <= d2) || (a2 <= c2 && c2 <= b2));
    }

    // Mutators (internal: see the remarks)

    /// <summary>Adds a section point to the tangent zone.</summary>
    internal void Append(in SectionPoint pi)
    {
        _result.Append(pi);
        if (_paramOnFirstMin > pi.ParamOnFirst())
        {
            _paramOnFirstMin = pi.ParamOnFirst();
        }

        if (_paramOnSecondMin > pi.ParamOnSecond())
        {
            _paramOnSecondMin = pi.ParamOnSecond();
        }

        if (_paramOnFirstMax < pi.ParamOnFirst())
        {
            _paramOnFirstMax = pi.ParamOnFirst();
        }

        if (_paramOnSecondMax < pi.ParamOnSecond())
        {
            _paramOnSecondMax = pi.ParamOnSecond();
        }
    }

    /// <summary>Adds the points of the tangent zone <paramref name="tzi"/> to this one.</summary>
    internal void Append(TangentZone tzi)
    {
        int Tzi_NumberOfPoints = tzi.NumberOfPoints();
        for (int ipi = 1; ipi <= Tzi_NumberOfPoints; ipi++)
        {
            PolygonInsert(tzi.GetPoint(ipi));
        }
    }

    /// <summary>Inserts a section point in the tangent zone: OCCT's body is commented out, so nothing is inserted.</summary>
    internal bool Insert(in SectionPoint pi)
    {
        _ = pi;
        bool inserted = false;
        return inserted;
    }

    /// <summary>
    /// Inserts a point in the polygonal tangent zone: appended if its parameter on the first argument is the
    /// highest, inserted first if it is within the range, appended otherwise.
    /// </summary>
    internal void PolygonInsert(in SectionPoint pi)
    {
        int nbpTz = NumberOfPoints();
        if (nbpTz == 0)
        {
            Append(pi);
            return;
        }

        if (pi.ParamOnFirst() >= _paramOnFirstMax)
        {
            Append(pi);
        }
        else if (pi.ParamOnFirst() >= _paramOnFirstMin)
        {
            InsertBefore(1, pi);
        }
        else
        {
            Append(pi); //-- On met les points sans les classer
                        //-- si on veut on pourra les reclasser
                        //-- ensuite avec un TRI.
        }
    }

    /// <summary>Inserts a section point before <paramref name="index"/> in the tangent zone.</summary>
    internal void InsertBefore(int index, in SectionPoint pi)
    {
        _result.InsertBefore(index, pi);
        if (_paramOnFirstMin > pi.ParamOnFirst())
        {
            _paramOnFirstMin = pi.ParamOnFirst();
        }

        if (_paramOnSecondMin > pi.ParamOnSecond())
        {
            _paramOnSecondMin = pi.ParamOnSecond();
        }

        if (_paramOnFirstMax < pi.ParamOnFirst())
        {
            _paramOnFirstMax = pi.ParamOnFirst();
        }

        if (_paramOnSecondMax < pi.ParamOnSecond())
        {
            _paramOnSecondMax = pi.ParamOnSecond();
        }
    }

    /// <summary>Inserts a section point after <paramref name="index"/> in the tangent zone.</summary>
    internal void InsertAfter(int index, in SectionPoint pi)
    {
        _result.InsertAfter(index, pi);
        if (_paramOnFirstMin > pi.ParamOnFirst())
        {
            _paramOnFirstMin = pi.ParamOnFirst();
        }

        if (_paramOnSecondMin > pi.ParamOnSecond())
        {
            _paramOnSecondMin = pi.ParamOnSecond();
        }

        if (_paramOnFirstMax < pi.ParamOnFirst())
        {
            _paramOnFirstMax = pi.ParamOnFirst();
        }

        if (_paramOnSecondMax < pi.ParamOnSecond())
        {
            _paramOnSecondMax = pi.ParamOnSecond();
        }
    }

    /// <summary>
    /// OCCT's sequence of section points, indexed from 1, as a deque: the points before the first appended one are
    /// held in reverse in <c>myFront</c>, the others in <c>myBack</c>, so inserting at either end is cheap.
    /// </summary>
    private sealed class Points
    {
        private readonly List<SectionPoint> _front;
        private readonly List<SectionPoint> _back;

        public Points()
        {
            _front = [];
            _back = [];
        }

        public Points(Points other)
        {
            _front = [.. other._front];
            _back = [.. other._back];
        }

        public int Length() => _front.Count + _back.Count;

        public SectionPoint this[int index]
        {
            get
            {
                Standard_OutOfRange_Raise_if(index < 1 || index > Length(), "NCollection_Sequence::Value");
                int nbFront = _front.Count;
                return index <= nbFront ? _front[nbFront - index] : _back[index - nbFront - 1];
            }
        }

        public void Append(in SectionPoint point) => _back.Add(point);

        public void InsertBefore(int index, in SectionPoint point)
        {
            Standard_OutOfRange_Raise_if(index < 1 || index > Length() + 1, "NCollection_Sequence::InsertBefore");
            int position = index - 1;
            int nbFront = _front.Count;
            if (position <= nbFront)
            {
                _front.Insert(nbFront - position, point);
            }
            else
            {
                _back.Insert(position - nbFront, point);
            }
        }

        public void InsertAfter(int index, in SectionPoint point)
        {
            Standard_OutOfRange_Raise_if(index < 0 || index > Length(), "NCollection_Sequence::InsertAfter");
            InsertBefore(index + 1, point);
        }
    }

    public override string ToString()
    {
        System.Text.StringBuilder builder = new("Intf_TangentZone(");
        builder.Append(Constants.Format("first", _paramOnFirstMin, _paramOnFirstMax)).Append(", ");
        builder.Append(Constants.Format("second", _paramOnSecondMin, _paramOnSecondMax));
        for (int p = 1; p <= _result.Length(); p++)
        {
            builder.Append(", ").Append(_result[p].ToString());
        }

        return builder.Append(')').ToString();
    }
}
