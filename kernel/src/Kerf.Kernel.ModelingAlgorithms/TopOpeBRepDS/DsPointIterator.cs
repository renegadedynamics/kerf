// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_PointIterator.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_PointIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Booleans.Legacy.Data.DsConfig;
using static Kerf.Engine.Booleans.Legacy.Data.DsKind;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>Iterates over the interferences of a list whose geometry is a point or a vertex: the points on a curve or edge.</summary>
internal sealed class DsPointIterator : DsInterferenceIterator
{
    /// <summary>An iterator over the points of the interferences of <paramref name="l"/>.</summary>
    public DsPointIterator(NCollection_List<DsInterference> l)
        : base(l)
    {
        Match();
    }

    /// <summary>Whether the geometry of <paramref name="i"/> is a POINT or a VERTEX.</summary>
    public override bool MatchInterference(DsInterference i)
    {
        DsKind gt = i.GeometryType();
        bool r = (gt == DsKind.Point) || (gt == DsKind.Vertex);
        return r;
    }

    /// <summary>The index of the current point (or vertex) in the data structure.</summary>
    public int Current() => Value().Geometry();

    /// <summary>The orientation of the current interference's transition for the state <paramref name="s"/>.</summary>
    public Orientation Orientation(Containment s)
    {
        DsInterference i = Value();
        DsTransition t = i.Transition();
        Orientation o = t.Orientation(s);
        return o;
    }

    /// <summary>The parameter of the current point on the curve or edge.</summary>
    /// <exception cref="ProgramErrorException">The interference is neither a curve/point nor an edge/vertex interference (exactly).</exception>
    public double Parameter()
    {
        DsInterference i = Value();
        Type t = i.DynamicType();
        if (t == typeof(DsCurvePointInterference))
        {
            return ((DsCurvePointInterference)i).Parameter();
        }
        else if (t == typeof(DsEdgeVertexInterference))
        {
            return ((DsEdgeVertexInterference)i).Parameter();
        }
        else
        {
            throw new ProgramErrorException("TopOpeBRepDS_PointIterator::Parameter()");
        }
    }

    /// <summary>Whether the current geometry is a vertex.</summary>
    public bool IsVertex => (Value().GeometryType() == DsKind.Vertex);

    /// <summary>Whether the current geometry is a point.</summary>
    public bool IsPoint => (Value().GeometryType() == DsKind.Point);

    /// <summary>Whether the current edge/vertex interference is DIFFORIENTED.</summary>
    /// <exception cref="ProgramErrorException">The interference is not an edge/vertex interference.</exception>
    public bool DiffOriented()
    {
        DsInterference i = Value();
        if (i.DynamicType() == typeof(DsEdgeVertexInterference))
        {
            return ((DsEdgeVertexInterference)i).Config() == DsConfig.Difforiented;
        }
        else
        {
            throw new ProgramErrorException("TopOpeBRepDS_PointIterator::DiffOriented()");
        }
    }

    /// <summary>Whether the current edge/vertex interference is SAMEORIENTED.</summary>
    /// <exception cref="ProgramErrorException">The interference is not an edge/vertex interference.</exception>
    public bool SameOriented()
    {
        DsInterference i = Value();
        if (i.DynamicType() == typeof(DsEdgeVertexInterference))
        {
            return ((DsEdgeVertexInterference)i).Config() == DsConfig.Sameoriented;
        }
        else
        {
            throw new ProgramErrorException("TopOpeBRepDS_PointIterator::SameOriented()");
        }
    }

    /// <summary>The support of the current interference.</summary>
    public int Support() => (Value().Support());
}
