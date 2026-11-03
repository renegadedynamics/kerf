// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_GluedShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_GluedShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>A shape glued on another by some of its faces, as a generated shape.</summary>
/// <remarks>It only reads the shape; the shapes it hands out are the shape's own sub-shapes.</remarks>
public sealed class LocOpe_GluedShape : LocOpe_GeneratedShape
{
    private TopoDS_Shape myShape = new();
    // OCCT iterates this map keyed by the faces' addresses; the port in the order the faces were glued
    private readonly NCollection_IndexedMap<TopoDS_Shape> myMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myGShape = new(TopTools_ShapeMapHasher.Instance);

    public LocOpe_GluedShape()
    {
    }

    public LocOpe_GluedShape(in TopoDS_Shape S)
    {
        myShape = S;
    }

    public void Init(in TopoDS_Shape S)
    {
        myShape = S;
        myMap.Clear();
        myGShape.Clear();
        myList.Clear();
        myGEdges.Clear();
    }

    public void GlueOnFace(in TopoDS_Face F)
    {
        TopExp_Explorer exp = new(myShape, TopAbs_FACE);
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

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMapEF = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(myShape, TopAbs_EDGE, TopAbs_FACE, theMapEF);

        NCollection_Map<TopoDS_Shape> mapdone = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator itl = new();
        TopExp_Explorer exp = new(), exp2 = new(), exp3 = new();

        for (int itm = 1; itm <= myMap.Extent(); itm++)
        {
            TopoDS_Face fac = TopoDS.Face(myMap[itm]);
            for (exp.Init(fac, TopAbs_EDGE); exp.More(); exp.Next())
            {
                TopoDS_Edge edg = TopoDS.Edge(exp.Current());
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
            TopoDS_Edge edg = TopoDS.Edge(itl.Value());
            for (exp.Init(edg, TopAbs_VERTEX); exp.More(); exp.Next())
            {
                TopoDS_Vertex vtx = TopoDS.Vertex(exp.Current());
                if (myGShape.IsBound(vtx))
                {
                    continue;
                }
                for (exp2.Init(myGShape[edg], TopAbs_EDGE); exp2.More(); exp2.Next())
                {
                    if (exp2.Current().IsSame(edg))
                    {
                        continue;
                    }
                    for (exp3.Init(exp2.Current(), TopAbs_VERTEX); exp3.More(); exp3.Next())
                    {
                        if (exp3.Current().IsSame(vtx))
                        {
                            if (myGShape.IsBound(exp2.Current()))
                            {
                                myGShape.Bind(vtx, new TopoDS_Edge());
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

        for (exp.Init(myShape, TopAbs_FACE); exp.More(); exp.Next())
        {
            if (!myMap.Contains(exp.Current()))
            {
                myList.Append(exp.Current());
            }
        }
    }

    public override NCollection_List<TopoDS_Shape> GeneratingEdges()
    {
        if (myGShape.IsEmpty())
        {
            MapEdgeAndVertices();
        }
        return myGEdges;
    }

    public override TopoDS_Edge Generated(in TopoDS_Vertex V)
    {
        if (myGShape.IsEmpty())
        {
            MapEdgeAndVertices();
        }
        return TopoDS.Edge(myGShape[V]);
    }

    public override TopoDS_Face Generated(in TopoDS_Edge E)
    {
        if (myGShape.IsEmpty())
        {
            MapEdgeAndVertices();
        }
        return TopoDS.Face(myGShape[E]);
    }

    public override NCollection_List<TopoDS_Shape> OrientedFaces()
    {
        if (myGShape.IsEmpty())
        {
            MapEdgeAndVertices();
        }
        return myList;
    }
}
