// Port of: src/ModelingData/TKGeomBase/LProp/LProp_CurAndInf.hxx
// Port of: src/ModelingData/TKGeomBase/LProp/LProp_CurAndInf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Geometry.Properties.CIType;

namespace Kerf.Engine.Geometry.Properties;

/// <summary>
/// The parameters of a 2D or 3D curve where its curvature is extreme and where it inflects, in increasing order.
/// </summary>
/// <remarks>A working object, filled by <see cref="CurAndInf2d"/>: its methods are public, as in OCCT.</remarks>
internal class CurAndInf
{
    private readonly NCollection_Sequence<double> _params = new();
    private readonly NCollection_Sequence<CIType> _types = new();

    public CurAndInf()
    {
    }

    /// <summary>Adds an inflection point at the parameter, keeping the parameters sorted.</summary>
    public void AddInflection(double param)
    {
        if (_params.IsEmpty())
        {
            _params.Append(param);
            _types.Append(Inflection);
            return;
        }

        if (param > _params.Last())
        {
            _params.Append(param);
            _types.Append(Inflection);
            return;
        }

        for (int i = 1; i <= _params.Length(); i++)
        {
            if (_params.Value(i) > param)
            {
                _params.InsertBefore(i, param);
                _types.InsertBefore(i, Inflection);
                break;
            }
        }
    }

    /// <summary>
    /// Adds an extremum of curvature at the parameter (a minimum of the radius of curvature if
    /// <paramref name="isMin"/>), keeping the parameters sorted.
    /// </summary>
    public void AddExtCur(double param, bool isMin)
    {
        CIType typePoint;
        if (isMin)
        {
            typePoint = MinCur;
        }
        else
        {
            typePoint = MaxCur;
        }

        if (_params.IsEmpty())
        {
            _params.Append(param);
            _types.Append(typePoint);
            return;
        }

        if (param > _params.Last())
        {
            _params.Append(param);
            _types.Append(typePoint);
            return;
        }

        for (int i = 1; i <= _params.Length(); i++)
        {
            if (_params.Value(i) > param)
            {
                _params.InsertBefore(i, param);
                _types.InsertBefore(i, typePoint);
                break;
            }
        }
    }

    public void Clear()
    {
        _params.Clear();
        _types.Clear();
    }

    public bool IsEmpty
    {
        get
        {
            return _params.IsEmpty();
        }
    }

    /// <summary>The number of points, stored by increasing parameter.</summary>
    public int PointCount
    {
        get
        {
            return _params.Length();
        }
    }

    /// <summary>The parameter of the Nth point; throws Standard_OutOfRange if N is not in [1, NbPoints()].</summary>
    public double Parameter(int n)
    {
        if (n < 1 || n > PointCount)
        {
            throw new OutOfRangeException();
        }

        return _params.Value(n);
    }

    /// <summary>
    /// The type of the Nth point: MinCur at a minimum of the radius of curvature, MaxCur at a maximum, Inflection at
    /// an inflection point. Throws Standard_OutOfRange if N is not in [1, NbPoints()].
    /// </summary>
    public CIType Type(int n)
    {
        if (n < 1 || n > PointCount)
        {
            throw new OutOfRangeException();
        }

        return _types.Value(n);
    }
}
