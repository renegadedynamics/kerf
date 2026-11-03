// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_GluedShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_GluedShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features.Local;

/// <summary>A shape glued on another by some of its faces, as a generated shape.</summary>
/// <remarks>It only reads the shape; the shapes it hands out are the shape's own sub-shapes.</remarks>
public sealed class GluedShape : GeneratedShape
{
    private Shape _shape = new();
    // OCCT iterates this map keyed by the faces' addresses; the port in the order the faces were glued
    private readonly NCollection_IndexedMap<Shape> _map = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, Shape> _gShape = new(ShapeHasher.Instance);

    public GluedShape()
    {
    }

    public GluedShape(in Shape s)
    {
        _shape = s;
    }

    public void Init(in Shape s)
    {
        _shape = s;
        _map.Clear();
        _gShape.Clear();
        _list.Clear();
        _gEdges.Clear();
    }

    public void GlueOnFace(in Face f)
    {
        Explorer exp = new(_shape, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(f))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }
        _map.Add(exp.Current()); // bonne orientation
    }

    private void MapEdgeAndVertices()
    {
        if (!_gShape.IsEmpty())
        {
            return;
        }

        // Edges et faces generes

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapEF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(_shape, ShapeType.Edge, ShapeType.Face, theMapEF);

        NCollection_Map<Shape> mapdone = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itl = new();
        Explorer exp = new(), exp2 = new(), exp3 = new();

        for (int itm = 1; itm <= _map.Extent(); itm++)
        {
            Face fac = ShapeCasts.Face(_map[itm]);
            for (exp.Init(fac, ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                if (mapdone.Contains(edg))
                {
                    continue;
                }
                // Est-ce un edge de connexite entre les faces collees
                if (theMapEF.FindFromKey(edg).Extent() != 2)
                {
                    throw new ConstructionErrorException();
                }
                for (itl.Initialize(theMapEF.FindFromKey(edg)); itl.More(); itl.Next())
                {
                    if (!_map.Contains(itl.Value()))
                    {
                        break;
                    }
                }

                if (itl.More())
                {
                    //	myGEdges.Append(edg);
                    _gEdges.Append(edg.Reversed());
                    _gShape.Bind(edg, itl.Value()); // voir orientation,
                                                     // mais devrait etre bon
                }

                mapdone.Add(edg);
            }
        }

        for (itl.Initialize(_gEdges); itl.More(); itl.Next())
        {
            Edge edg = ShapeCasts.Edge(itl.Value());
            for (exp.Init(edg, ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp.Current());
                if (_gShape.IsBound(vtx))
                {
                    continue;
                }
                for (exp2.Init(_gShape[edg], ShapeType.Edge); exp2.More(); exp2.Next())
                {
                    if (exp2.Current().IsSame(edg))
                    {
                        continue;
                    }
                    for (exp3.Init(exp2.Current(), ShapeType.Vertex); exp3.More(); exp3.Next())
                    {
                        if (exp3.Current().IsSame(vtx))
                        {
                            if (_gShape.IsBound(exp2.Current()))
                            {
                                _gShape.Bind(vtx, new Edge());
                            }
                            else
                            {
                                _gShape.Bind(vtx, exp2.Current());
                            }
                            break;
                        }
                    }
                    if (exp3.More())
                    {
                        break;
                    }
                }
            }
        }

        // liste de faces

        for (exp.Init(_shape, ShapeType.Face); exp.More(); exp.Next())
        {
            if (!_map.Contains(exp.Current()))
            {
                _list.Append(exp.Current());
            }
        }
    }

    public override NCollection_List<Shape> GeneratingEdges()
    {
        if (_gShape.IsEmpty())
        {
            MapEdgeAndVertices();
        }
        return _gEdges;
    }

    public override Edge Generated(in Vertex v)
    {
        if (_gShape.IsEmpty())
        {
            MapEdgeAndVertices();
        }
        return ShapeCasts.Edge(_gShape[v]);
    }

    public override Face Generated(in Edge e)
    {
        if (_gShape.IsEmpty())
        {
            MapEdgeAndVertices();
        }
        return ShapeCasts.Face(_gShape[e]);
    }

    public override NCollection_List<Shape> OrientedFaces()
    {
        if (_gShape.IsEmpty())
        {
            MapEdgeAndVertices();
        }
        return _list;
    }
}
