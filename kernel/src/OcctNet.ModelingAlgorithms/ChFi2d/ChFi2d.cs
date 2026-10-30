// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ChFi2d_ConstructionError;

namespace Occt;

/// <summary>Utilities of the 2D fillet and chamfer algorithms.</summary>
public static class ChFi2d
{
    /// <summary>
    /// True if the edges <paramref name="E1"/> and <paramref name="E2"/> share a vertex, returned in
    /// <paramref name="V"/> (the first vertex of <paramref name="E1"/> if it is shared, else its last). <paramref name="V"/>
    /// is unchanged when they share none.
    /// </summary>
    public static bool CommonVertex(in TopoDS_Edge E1, in TopoDS_Edge E2, ref TopoDS_Vertex V)
    {
        TopExp.Vertices(E1, out TopoDS_Vertex firstVertex1, out TopoDS_Vertex lastVertex1);
        TopExp.Vertices(E2, out TopoDS_Vertex firstVertex2, out TopoDS_Vertex lastVertex2);

        if (firstVertex1.IsSame(firstVertex2) || firstVertex1.IsSame(lastVertex2))
        {
            V = firstVertex1;
            return true;
        }
        if (lastVertex1.IsSame(firstVertex2) || lastVertex1.IsSame(lastVertex2))
        {
            V = lastVertex1;
            return true;
        }
        return false;
    } // CommonVertex

    /// <summary>
    /// Finds the two edges of the face <paramref name="F"/> connected to the vertex <paramref name="V"/>:
    /// <see cref="ChFi2d_IsDone"/>, or <see cref="ChFi2d_ConnexionError"/> if <paramref name="V"/> is not on the face
    /// or doesn't connect exactly two edges (<paramref name="E1"/> and <paramref name="E2"/> are then written as far as
    /// they were found, as OCCT does).
    /// </summary>
    public static ChFi2d_ConstructionError FindConnectedEdges(in TopoDS_Face F,
                                                              in TopoDS_Vertex V,
                                                              ref TopoDS_Edge E1,
                                                              ref TopoDS_Edge E2)
    {
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> vertexMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(F, TopAbs_ShapeEnum.TopAbs_VERTEX, TopAbs_ShapeEnum.TopAbs_EDGE, vertexMap);

        if (vertexMap.Contains(V))
        {
            NCollection_List<TopoDS_Shape>.Iterator iterator = new(vertexMap.FindFromKey(V));
            if (iterator.More())
            {
                E1 = TopoDS.Edge(iterator.Value());
                iterator.Next();
            } // if ...
            else
            {
                return ChFi2d_ConnexionError;
            }
            if (iterator.More())
            {
                E2 = TopoDS.Edge(iterator.Value());
                iterator.Next();
            } // if ...
            else
            {
                return ChFi2d_ConnexionError;
            }

            if (iterator.More())
            {
                return ChFi2d_ConnexionError;
            }
        } // if (isFind)
        else
        {
            return ChFi2d_ConnexionError;
        }
        return ChFi2d_IsDone;
    } // FindConnectedEdges
}
