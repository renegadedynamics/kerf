// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Planar.ConstructionError;

namespace Kerf.Engine.Fillets.Planar;

/// <summary>Utilities of the 2D fillet and chamfer algorithms.</summary>
internal static class ChFi2d
{
    /// <summary>
    /// True if the edges <paramref name="e1"/> and <paramref name="e2"/> share a vertex, returned in
    /// <paramref name="v"/> (the first vertex of <paramref name="e1"/> if it is shared, else its last). <paramref name="v"/>
    /// is unchanged when they share none.
    /// </summary>
    public static bool CommonVertex(in Edge e1, in Edge e2, ref Vertex v)
    {
        Exploration.Vertices(e1, out Vertex firstVertex1, out Vertex lastVertex1);
        Exploration.Vertices(e2, out Vertex firstVertex2, out Vertex lastVertex2);

        if (firstVertex1.IsSame(firstVertex2) || firstVertex1.IsSame(lastVertex2))
        {
            v = firstVertex1;
            return true;
        }
        if (lastVertex1.IsSame(firstVertex2) || lastVertex1.IsSame(lastVertex2))
        {
            v = lastVertex1;
            return true;
        }
        return false;
    } // CommonVertex

    /// <summary>
    /// Finds the two edges of the face <paramref name="f"/> connected to the vertex <paramref name="v"/>:
    /// <see cref="IsDone"/>, or <see cref="ConnexionError"/> if <paramref name="v"/> is not on the face
    /// or doesn't connect exactly two edges (<paramref name="e1"/> and <paramref name="e2"/> are then written as far as
    /// they were found, as OCCT does).
    /// </summary>
    public static ConstructionError FindConnectedEdges(in Face f,
                                                              in Vertex v,
                                                              ref Edge e1,
                                                              ref Edge e2)
    {
        IndexedMap<Shape, NCollection_List<Shape>> vertexMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(f, ShapeType.Vertex, ShapeType.Edge, vertexMap);

        if (vertexMap.ContainsKey(v))
        {
            NCollection_List<Shape>.Iterator iterator = new(vertexMap.GetValue(v));
            if (iterator.More())
            {
                e1 = ShapeCasts.Edge(iterator.Value());
                iterator.Next();
            } // if ...
            else
            {
                return ConstructionError.ConnexionError;
            }
            if (iterator.More())
            {
                e2 = ShapeCasts.Edge(iterator.Value());
                iterator.Next();
            } // if ...
            else
            {
                return ConstructionError.ConnexionError;
            }

            if (iterator.More())
            {
                return ConstructionError.ConnexionError;
            }
        } // if (isFind)
        else
        {
            return ConstructionError.ConnexionError;
        }
        return ConstructionError.IsDone;
    } // FindConnectedEdges
}
