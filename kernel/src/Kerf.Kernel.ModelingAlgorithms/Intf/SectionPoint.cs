// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_SectionPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_SectionPoint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_SectionPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.Polygons.PIType;

namespace Kerf.Engine.Intersection.Polygons;

/// <summary>
/// An intersection point between polygons and polyhedra: its location, and on each argument its type (vertex,
/// edge, face), its addresses in the argument and its parameter.
/// </summary>
/// <remarks>
/// OCCT's default is all zeros (Intf_EXTERNAL), as <c>default(Intf_SectionPoint)</c>. OCCT's <c>operator==</c> is
/// <see cref="IsEqual"/>, which compares the logical information only; <c>==</c>, <c>Equals</c> and
/// <c>GetHashCode</c> compare every field exactly (decision 282). <see cref="Merge"/> is an internal mutator.
/// OCCT's <c>Dump</c> prints only in a debug build and is left out.
/// </remarks>
public partial struct SectionPoint : IEquatable<SectionPoint>
{
    private Point3D _pnt;
    private PIType _dimenObje;
    private int _indexO1;
    private int _indexO2;
    private double _paramObje;
    private PIType _dimenTool;
    private int _indexT1;
    private int _indexT2;
    private double _paramTool;
    private double _incide;

    public SectionPoint()
    {
        _pnt = new Point3D(0.0, 0.0, 0.0);
        _dimenObje = PIType.External;
        _indexO1 = 0;
        _indexO2 = 0;
        _paramObje = 0.0;
        _dimenTool = PIType.External;
        _indexT1 = 0;
        _indexT2 = 0;
        _paramTool = 0.0;
        _incide = 0.0;
    }

    /// <summary>
    /// A section point with the respective dimensions (vertex, edge or face) of the concerned arguments and their
    /// addresses in the topological structure.
    /// </summary>
    public SectionPoint(in Point3D Where,
                             PIType dim1,
                             int addr1,
                             int addr2,
                             double param1,
                             PIType dim2,
                             int addr3,
                             int addr4,
                             double param2,
                             double incid)
    {
        _pnt = Where;
        _dimenObje = dim1;
        _indexO1 = addr1;
        _indexO2 = addr2;
        _paramObje = param1;
        _dimenTool = dim2;
        _indexT1 = addr3;
        _indexT2 = addr4;
        _paramTool = param2;
        _incide = incid;
    }

    /// <summary>
    /// A 2D section point with the respective dimensions (vertex or edge) of the concerned arguments and their
    /// addresses in the topological structure.
    /// </summary>
    public SectionPoint(in Point2D Where,
                             PIType dim1,
                             int addr1,
                             double param1,
                             PIType dim2,
                             int addr2,
                             double param2,
                             double incid)
    {
        _pnt = new Point3D(Where.X, Where.Y, 0.0);
        _dimenObje = dim1;
        _indexO1 = 0;
        _indexO2 = addr1;
        _paramObje = param1;
        _dimenTool = dim2;
        _indexT1 = 0;
        _indexT2 = addr2;
        _paramTool = param2;
        _incide = incid;
    }

    /// <summary>The location of the section point.</summary>
    public readonly Point3D Pnt() => _pnt;

    /// <summary>The cumulated parameter of the section point on the first element.</summary>
    public readonly double ParamOnFirst() => (_indexO2 - 1) + _paramObje;

    /// <summary>The cumulated parameter of the section point on the second element.</summary>
    public readonly double ParamOnSecond() => (_indexT2 - 1) + _paramTool;

    /// <summary>The type of the section point on the first element.</summary>
    public readonly PIType TypeOnFirst() => _dimenObje;

    /// <summary>The type of the section point on the second element.</summary>
    public readonly PIType TypeOnSecond() => _dimenTool;

    public readonly void InfoFirst(out PIType dim, out int add1, out int add2, out double param)
    {
        dim = _dimenObje;
        add1 = _indexO1;
        add2 = _indexO2;
        param = _paramObje;
    }

    /// <summary>The data about the first argument of the interference.</summary>
    public readonly void InfoFirst(out PIType dim, out int add, out double param)
    {
        dim = _dimenObje;
        add = _indexO2;
        param = _paramObje;
    }

    public readonly void InfoSecond(out PIType dim, out int add1, out int add2, out double param)
    {
        dim = _dimenTool;
        add1 = _indexT1;
        add2 = _indexT2;
        param = _paramTool;
    }

    /// <summary>The data about the second argument of the interference.</summary>
    public readonly void InfoSecond(out PIType dim, out int add, out double param)
    {
        dim = _dimenTool;
        add = _indexT2;
        param = _paramTool;
    }

    /// <summary>
    /// The incidence at this section point: the cosine between the two triangles. The best incidence is 0 (PI/2),
    /// the worst 1 (null angle).
    /// </summary>
    public readonly double Incidence() => _incide;

    /// <summary>True if the two section points have the same logical information (types and addresses).</summary>
    public readonly bool IsEqual(in SectionPoint other)
        => (_dimenObje == other._dimenObje && _indexO1 == other._indexO1 && _indexO2 == other._indexO2
            && _dimenTool == other._dimenTool && _indexT1 == other._indexT1 && _indexT2 == other._indexT2);

    /// <summary>True if the two section points are on the same edge of the first or the second element.</summary>
    public readonly bool IsOnSameEdge(in SectionPoint other)
    {
        bool isOn = false;
        if (_dimenObje == PIType.Edge)
        {
            if (other._dimenObje == PIType.Edge)
            {
                isOn = (_indexO1 == other._indexO1 && _indexO2 == other._indexO2);
            }
            else if (other._dimenObje == PIType.Vertex)
            {
                isOn = (_indexO1 == other._indexO1 || _indexO2 == other._indexO1);
            }
        }
        else if (_dimenObje == PIType.Vertex)
        {
            if (other._dimenObje == PIType.Edge)
            {
                isOn = (_indexO1 == other._indexO1 || _indexO1 == other._indexO2);
            }
            else if (other._dimenObje == PIType.Vertex)
            {
                // OCCT: compares the second argument's addresses (IndexT1) on the first argument's vertices
                isOn = (_indexT1 == other._indexT1);
            }
        }

        if (!isOn)
        {
            if (_dimenTool == PIType.Edge)
            {
                if (other._dimenTool == PIType.Edge)
                {
                    isOn = (_indexT1 == other._indexT1 && _indexT2 == other._indexT2);
                }
                else if (other._dimenTool == PIType.Vertex)
                {
                    isOn = (_indexT1 == other._indexT1 || _indexT2 == other._indexT1);
                }
            }
            else if (_dimenTool == PIType.Vertex)
            {
                if (other._dimenTool == PIType.Edge)
                {
                    isOn = (_indexT1 == other._indexT1 || _indexT1 == other._indexT2);
                }
                else if (other._dimenTool == PIType.Vertex)
                {
                    isOn = (_indexT1 == other._indexT1);
                }
            }
        }

        return isOn;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>
    /// Merges two section points: both get this point's location, and on each argument the data of the point of
    /// higher dimension.
    /// </summary>
    internal void Merge(ref SectionPoint other)
    {
        other._pnt = _pnt;
        if (_dimenObje >= other._dimenObje)
        {
            other._dimenObje = _dimenObje;
            other._indexO1 = _indexO1;
            other._indexO2 = _indexO2;
            other._paramObje = _paramObje;
        }
        else
        {
            _dimenObje = other._dimenObje;
            _indexO1 = other._indexO1;
            _indexO2 = other._indexO2;
            _paramObje = other._paramObje;
        }

        if (_dimenTool >= other._dimenTool)
        {
            other._dimenTool = _dimenTool;
            other._indexT1 = _indexT1;
            other._indexT2 = _indexT2;
            other._paramTool = _paramTool;
        }
        else
        {
            _dimenTool = other._dimenTool;
            _indexT1 = other._indexT1;
            _indexT2 = other._indexT2;
            _paramTool = other._paramTool;
        }
    }

    public readonly bool Equals(SectionPoint other)
        => _pnt.Equals(other._pnt) && _dimenObje == other._dimenObje && _indexO1 == other._indexO1
           && _indexO2 == other._indexO2 && _paramObje.Equals(other._paramObje) && _dimenTool == other._dimenTool
           && _indexT1 == other._indexT1 && _indexT2 == other._indexT2 && _paramTool.Equals(other._paramTool)
           && _incide.Equals(other._incide);

    public override readonly bool Equals(object? obj) => obj is SectionPoint other && Equals(other);

    public override readonly int GetHashCode()
    {
        HashCode hash = new();
        hash.Add(_pnt);
        hash.Add(_dimenObje);
        hash.Add(_indexO1);
        hash.Add(_indexO2);
        hash.Add(_paramObje);
        hash.Add(_dimenTool);
        hash.Add(_indexT1);
        hash.Add(_indexT2);
        hash.Add(_paramTool);
        hash.Add(_incide);
        return hash.ToHashCode();
    }

    public static bool operator ==(in SectionPoint left, in SectionPoint right) => left.Equals(right);

    public static bool operator !=(in SectionPoint left, in SectionPoint right) => !left.Equals(right);

    public override readonly string ToString()
        => $"Intf_SectionPoint({_dimenObje} {_indexO1} {_indexO2} {_paramObje.ToString("R", System.Globalization.CultureInfo.InvariantCulture)}, "
           + $"{_dimenTool} {_indexT1} {_indexT2} {_paramTool.ToString("R", System.Globalization.CultureInfo.InvariantCulture)}, "
           + Constants.Format("at", _pnt.X, _pnt.Y, _pnt.Z, _incide) + ")";
}
