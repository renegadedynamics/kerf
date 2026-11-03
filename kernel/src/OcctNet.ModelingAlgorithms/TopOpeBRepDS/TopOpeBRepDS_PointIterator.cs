// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_PointIterator.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_PointIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.TopOpeBRepDS_Config;
using static Occt.TopOpeBRepDS_Kind;

namespace Occt;

/// <summary>Iterates over the interferences of a list whose geometry is a point or a vertex: the points on a curve or edge.</summary>
public sealed class TopOpeBRepDS_PointIterator : TopOpeBRepDS_InterferenceIterator
{
    /// <summary>An iterator over the points of the interferences of <paramref name="L"/>.</summary>
    public TopOpeBRepDS_PointIterator(NCollection_List<TopOpeBRepDS_Interference> L)
        : base(L)
    {
        Match();
    }

    /// <summary>Whether the geometry of <paramref name="I"/> is a POINT or a VERTEX.</summary>
    public override bool MatchInterference(TopOpeBRepDS_Interference I)
    {
        TopOpeBRepDS_Kind GT = I.GeometryType();
        bool r = (GT == TopOpeBRepDS_POINT) || (GT == TopOpeBRepDS_VERTEX);
        return r;
    }

    /// <summary>The index of the current point (or vertex) in the data structure.</summary>
    public int Current() => Value().Geometry();

    /// <summary>The orientation of the current interference's transition for the state <paramref name="S"/>.</summary>
    public TopAbs_Orientation Orientation(TopAbs_State S)
    {
        TopOpeBRepDS_Interference I = Value();
        TopOpeBRepDS_Transition T = I.Transition();
        TopAbs_Orientation o = T.Orientation(S);
        return o;
    }

    /// <summary>The parameter of the current point on the curve or edge.</summary>
    /// <exception cref="ProgramErrorException">The interference is neither a curve/point nor an edge/vertex interference (exactly).</exception>
    public double Parameter()
    {
        TopOpeBRepDS_Interference I = Value();
        Type T = I.DynamicType();
        if (T == typeof(TopOpeBRepDS_CurvePointInterference))
        {
            return ((TopOpeBRepDS_CurvePointInterference)I).Parameter();
        }
        else if (T == typeof(TopOpeBRepDS_EdgeVertexInterference))
        {
            return ((TopOpeBRepDS_EdgeVertexInterference)I).Parameter();
        }
        else
        {
            throw new ProgramErrorException("TopOpeBRepDS_PointIterator::Parameter()");
        }
    }

    /// <summary>Whether the current geometry is a vertex.</summary>
    public bool IsVertex() => (Value().GeometryType() == TopOpeBRepDS_VERTEX);

    /// <summary>Whether the current geometry is a point.</summary>
    public bool IsPoint() => (Value().GeometryType() == TopOpeBRepDS_POINT);

    /// <summary>Whether the current edge/vertex interference is DIFFORIENTED.</summary>
    /// <exception cref="ProgramErrorException">The interference is not an edge/vertex interference.</exception>
    public bool DiffOriented()
    {
        TopOpeBRepDS_Interference I = Value();
        if (I.DynamicType() == typeof(TopOpeBRepDS_EdgeVertexInterference))
        {
            return ((TopOpeBRepDS_EdgeVertexInterference)I).Config() == TopOpeBRepDS_DIFFORIENTED;
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
        TopOpeBRepDS_Interference I = Value();
        if (I.DynamicType() == typeof(TopOpeBRepDS_EdgeVertexInterference))
        {
            return ((TopOpeBRepDS_EdgeVertexInterference)I).Config() == TopOpeBRepDS_SAMEORIENTED;
        }
        else
        {
            throw new ProgramErrorException("TopOpeBRepDS_PointIterator::SameOriented()");
        }
    }

    /// <summary>The support of the current interference.</summary>
    public int Support() => (Value().Support());
}
