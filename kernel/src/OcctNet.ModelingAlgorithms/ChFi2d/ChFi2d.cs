// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Topology;
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
    public static bool CommonVertex(in Edge E1, in Edge E2, ref Vertex V)
    {
        Exploration.Vertices(E1, out Vertex firstVertex1, out Vertex lastVertex1);
        Exploration.Vertices(E2, out Vertex firstVertex2, out Vertex lastVertex2);

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
    public static ChFi2d_ConstructionError FindConnectedEdges(in Face F,
                                                              in Vertex V,
                                                              ref Edge E1,
                                                              ref Edge E2)
    {
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> vertexMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(F, ShapeType.Vertex, ShapeType.Edge, vertexMap);

        if (vertexMap.Contains(V))
        {
            NCollection_List<Shape>.Iterator iterator = new(vertexMap.FindFromKey(V));
            if (iterator.More())
            {
                E1 = ShapeCasts.Edge(iterator.Value());
                iterator.Next();
            } // if ...
            else
            {
                return ChFi2d_ConnexionError;
            }
            if (iterator.More())
            {
                E2 = ShapeCasts.Edge(iterator.Value());
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
