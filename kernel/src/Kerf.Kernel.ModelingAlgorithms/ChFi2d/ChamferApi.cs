// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_ChamferAPI.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_ChamferAPI.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Curves;

using Kerf.Engine.Topology;

using Kerf.Engine.Topology.Brep;

using Kerf.Geometry;

using Kerf.Topology;

namespace Kerf.Engine.Fillets.Planar;

/// <summary>Makes a chamfer between two linear edges.</summary>
/// <remarks>A working object; its results are new, frozen edges. The edges it is given are only read.</remarks>
internal sealed class ChamferApi
{
    private Edge _edge1 = new(), _edge2 = new();
    private Curve3D? _curve1, _curve2;
    private double _start1, _end1, _start2, _end2;
    private bool _commonStart1, _commonStart2;

    // An empty constructor.
    public ChamferApi()
    {
        _start1 = 0.0;
        _end1 = 0.0;
        _start2 = 0.0;
        _end2 = 0.0;
        _commonStart1 = false;
        _commonStart2 = false;
    }

    /// <summary>A chamfer of the first two edges of a wire.</summary>
    public ChamferApi(in Wire wire)
        : this()
    {
        Init(wire);
    }

    /// <summary>A chamfer of two linear edges.</summary>
    public ChamferApi(in Edge edge1, in Edge edge2)
        : this()
    {
        _edge1 = edge1;
        _edge2 = edge2;
    }

    /// <summary>Initializes the class by a wire consisting of two linear edges.</summary>
    public void Init(in Wire wire)
    {
        Edge e1 = new(), e2 = new();
        var itr = new Iterator(wire);
        for (; itr.More(); itr.Next())
        {
            if (e1.IsNull)
            {
                e1 = ShapeCasts.Edge(itr.Value());
            }
            else if (e2.IsNull)
            {
                e2 = ShapeCasts.Edge(itr.Value());
            }
            else
            {
                break;
            }
        }
        Init(e1, e2);
    }

    /// <summary>Initializes the class by two linear edges.</summary>
    public void Init(in Edge edge1, in Edge edge2)
    {
        _edge1 = edge1;
        _edge2 = edge2;
    }

    /// <summary>Prepares the chamfer: finds the common ends of the edges. Returns true.</summary>
    public bool Perform()
    {
        _curve1 = Tool.Curve(_edge1, out _start1, out _end1);
        _curve2 = Tool.Curve(_edge2, out _start2, out _end2);
        // searching for common points
        if (_curve1!.Value(_start1).IsEqual(_curve2!.Value(_end2), global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            _commonStart1 = true;
            _commonStart2 = false;
        }
        else
        {
            if (_curve1.Value(_end1).IsEqual(_curve2.Value(_start2), global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                _commonStart1 = false;
                _commonStart2 = true;
            }
            else
            {
                if (_curve1.Value(_end1).IsEqual(_curve2.Value(_end2), global::Kerf.Engine.Foundation.Precision.Confusion()))
                {
                    _commonStart1 = false;
                    _commonStart2 = false;
                }
                else
                {
                    _commonStart1 = true;
                    _commonStart2 = true;
                }
            }
        }
        return true;
    }

    /// <summary>
    /// The chamfer edge (frozen), at the parametric lengths <paramref name="length1"/> and
    /// <paramref name="length2"/> from the common ends, with the two edges trimmed by it; a null edge if an edge is
    /// shorter than its length.
    /// </summary>
    public Edge Result(ref Edge edge1, ref Edge edge2, double length1, double length2)
    {
        Edge result = new();
        if (Math.Abs(_end1 - _start1) < length1)
        {
            return result;
        }
        if (Math.Abs(_end2 - _start2) < length2)
        {
            return result;
        }

        double common1 = (_commonStart1 ? _start1 : _end1)
                          + (((_start1 > _end1) ^ _commonStart1) ? length1 : -length1);
        double common2 = (_commonStart2 ? _start2 : _end2)
                          + (((_start2 > _end2) ^ _commonStart2) ? length2 : -length2);

        // make chamfer edge
        var ml = new MakeLine(_curve1!.Value(common1), _curve2!.Value(common2));
        var builder = new MakeEdge(ml.Value(),
                                                   _curve1.Value(common1),
                                                   _curve2.Value(common2));
        result = builder.Edge();
        // divide first edge
        var divider1 = new MakeEdge(_curve1, common1, (_commonStart1 ? _end1 : _start1));
        edge1 = divider1.Edge();
        // divide second edge
        var divider2 = new MakeEdge(_curve2, common2, (_commonStart2 ? _end2 : _start2));
        edge2 = divider2.Edge();

        return result;
    }
}
