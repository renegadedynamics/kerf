// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_GluedShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_GluedShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>A shape glued on another by some of its faces, as a generated shape.</summary>
/// <remarks>It only reads the shape; the shapes it hands out are the shape's own sub-shapes.</remarks>
public sealed class LocOpe_GluedShape : LocOpe_GeneratedShape
{
    private Shape myShape = new();
    // OCCT iterates this map keyed by the faces' addresses; the port in the order the faces were glued
    private readonly NCollection_IndexedMap<Shape> myMap = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, Shape> myGShape = new(ShapeHasher.Instance);

    public LocOpe_GluedShape()
    {
    }

    public LocOpe_GluedShape(in Shape S)
    {
        myShape = S;
    }

    public void Init(in Shape S)
    {
        myShape = S;
        myMap.Clear();
        myGShape.Clear();
        myList.Clear();
        myGEdges.Clear();
    }

    public void GlueOnFace(in Face F)
    {
        Explorer exp = new(myShape, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(F))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }
        myMap.Add(exp.Current()); // bonne orientation
    }

    private void MapEdgeAndVertices()
    {
        if (!myGShape.IsEmpty())
        {
            return;
        }

        // Edges et faces generes

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMapEF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(myShape, ShapeType.Edge, ShapeType.Face, theMapEF);

        NCollection_Map<Shape> mapdone = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itl = new();
        Explorer exp = new(), exp2 = new(), exp3 = new();

        for (int itm = 1; itm <= myMap.Extent(); itm++)
        {
            Face fac = ShapeCasts.Face(myMap[itm]);
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
                    if (!myMap.Contains(itl.Value()))
                    {
                        break;
                    }
                }

                if (itl.More())
                {
                    //	myGEdges.Append(edg);
                    myGEdges.Append(edg.Reversed());
                    myGShape.Bind(edg, itl.Value()); // voir orientation,
                                                     // mais devrait etre bon
                }

                mapdone.Add(edg);
            }
        }

        for (itl.Initialize(myGEdges); itl.More(); itl.Next())
        {
            Edge edg = ShapeCasts.Edge(itl.Value());
            for (exp.Init(edg, ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp.Current());
                if (myGShape.IsBound(vtx))
                {
                    continue;
                }
                for (exp2.Init(myGShape[edg], ShapeType.Edge); exp2.More(); exp2.Next())
                {
                    if (exp2.Current().IsSame(edg))
                    {
                        continue;
                    }
                    for (exp3.Init(exp2.Current(), ShapeType.Vertex); exp3.More(); exp3.Next())
                    {
                        if (exp3.Current().IsSame(vtx))
                        {
                            if (myGShape.IsBound(exp2.Current()))
                            {
                                myGShape.Bind(vtx, new Edge());
                            }
                            else
                            {
                                myGShape.Bind(vtx, exp2.Current());
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

        for (exp.Init(myShape, ShapeType.Face); exp.More(); exp.Next())
        {
            if (!myMap.Contains(exp.Current()))
            {
                myList.Append(exp.Current());
            }
        }
    }

    public override NCollection_List<Shape> GeneratingEdges()
    {
        if (myGShape.IsEmpty())
        {
            MapEdgeAndVertices();
        }
        return myGEdges;
    }

    public override Edge Generated(in Vertex V)
    {
        if (myGShape.IsEmpty())
        {
            MapEdgeAndVertices();
        }
        return ShapeCasts.Edge(myGShape[V]);
    }

    public override Face Generated(in Edge E)
    {
        if (myGShape.IsEmpty())
        {
            MapEdgeAndVertices();
        }
        return ShapeCasts.Face(myGShape[E]);
    }

    public override NCollection_List<Shape> OrientedFaces()
    {
        if (myGShape.IsEmpty())
        {
            MapEdgeAndVertices();
        }
        return myList;
    }
}
