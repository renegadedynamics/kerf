// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelabellaBaseMeshAlgo.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelabellaBaseMeshAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Meshing.Delaunay;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Meshing.DegreeOfFreedom;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Builds a face's triangulation with the Delabella library (<see cref="IDelaBella"/>), whose result BRepMesh_Delaun
/// then constrains to the face's boundary.
/// </summary>
/// <remarks>
/// Delabella's messages go to <see cref="Message"/>'s process-wide messenger, which sends nothing by default
/// (decision 493): an error as a failure, a warning or a note as a trace.
/// </remarks>
public class DelabellaBaseMeshAlgo : CustomBaseMeshAlgo
{
    /// <summary>An algorithm.</summary>
    public DelabellaBaseMeshAlgo()
    {
    }

    /// <summary>Builds the base triangulation with Delabella.</summary>
    protected override void BuildBaseTriangulation()
    {
        DataStructureOfDelaun structure = GetStructure();

        B2d box = new();
        int nodesNb = structure.NodeCount;
        double[] points = new double[2 * (nodesNb + 4)];
        for (int nodeIt = 0; nodeIt < nodesNb; ++nodeIt)
        {
            MeshVertex vertex = structure.GetNode(nodeIt + 1);

            int baseIdx = 2 * nodeIt;
            points[baseIdx + 0] = vertex.Coord().X();
            points[baseIdx + 1] = vertex.Coord().Y();

            box.Add(new Point2D(vertex.Coord()));
        }

        box.Enlarge(0.1 * (box.CornerMax() - box.CornerMin()).Modulus());
        Xy min = box.CornerMin();
        Xy max = box.CornerMax();

        points[2 * nodesNb + 0] = min.X();
        points[2 * nodesNb + 1] = min.Y();
        structure.AddNode(
            new MeshVertex(points[2 * nodesNb + 0], points[2 * nodesNb + 1], DegreeOfFreedom.Free));

        points[2 * nodesNb + 2] = max.X();
        points[2 * nodesNb + 3] = min.Y();
        structure.AddNode(
            new MeshVertex(points[2 * nodesNb + 2], points[2 * nodesNb + 3], DegreeOfFreedom.Free));

        points[2 * nodesNb + 4] = max.X();
        points[2 * nodesNb + 5] = max.Y();
        structure.AddNode(
            new MeshVertex(points[2 * nodesNb + 4], points[2 * nodesNb + 5], DegreeOfFreedom.Free));

        points[2 * nodesNb + 6] = min.X();
        points[2 * nodesNb + 7] = max.Y();
        structure.AddNode(
            new MeshVertex(points[2 * nodesNb + 6], points[2 * nodesNb + 7], DegreeOfFreedom.Free));

        double diffX = (max.X() - min.X());
        double diffY = (max.Y() - min.Y());
        for (int i = 0; i < points.Length; i += 2)
        {
            points[i + 0] = (points[i + 0] - min.X()) / diffX - 0.5;
            points[i + 1] = (points[i + 1] - min.Y()) / diffY - 0.5;
        }

        global::Kerf.Engine.Meshing.Delaunay.IDelaBella? triangulator = global::Kerf.Engine.Meshing.Delaunay.IDelaBella.Create();
        if (triangulator is null) // should never happen
        {
            throw new ProgramErrorException("BRepMesh_DelabellaBaseMeshAlgo::buildBaseTriangulation: unable "
                                            + "creating a triangulation algorithm");
        }

        triangulator.SetErrLog(LogDelabella2Occ, null);
        try
        {
            int verticesNb = triangulator.Triangulate(points.Length / 2,
                                                        points.AsSpan(0),
                                                        points.AsSpan(1),
                                                        2 * sizeof(double));

            if (verticesNb > 0)
            {
                Span<int> nodes = stackalloc int[3];
                Span<int> edges = stackalloc int[3];
                Span<bool> orientations = stackalloc bool[3];
                DelaunayTriangle? trianglePtr = triangulator.GetFirstDelaunayTriangle();
                while (trianglePtr is not null)
                {
                    nodes[0] = trianglePtr.v[0]!.i + 1;
                    nodes[1] = trianglePtr.v[2]!.i + 1;
                    nodes[2] = trianglePtr.v[1]!.i + 1;

                    for (int k = 0; k < 3; ++k)
                    {
                        MeshEdge link = new(nodes[k], nodes[(k + 1) % 3], DegreeOfFreedom.Free);

                        int linkInfo = structure.AddLink(link);
                        edges[k] = Math.Abs(linkInfo);
                        orientations[k] = linkInfo > 0;
                    }

                    Triangle triangle = new(edges, orientations, DegreeOfFreedom.Free);
                    structure.AddElement(triangle);

                    trianglePtr = trianglePtr.next;
                }
            }

            triangulator.Destroy();
            triangulator = null;
        }
        catch (FailureException theException)
        {
            if (triangulator is not null)
            {
                triangulator.Destroy();
                triangulator = null;
            }

            throw new FailureException(theException.Message);
        }
        catch (Exception exception) when (exception is not (ImmutableObjectException or LockedShapeException))
        {
            if (triangulator is not null)
            {
                triangulator.Destroy();
                triangulator = null;
            }

            throw new FailureException("BRepMesh_DelabellaBaseMeshAlgo::buildBaseTriangulation: exception in "
                                       + "triangulation algorithm");
        }
    }

    // Redirect algorithm messages to OCCT messenger (Delabella gives the formatted text).
    private static int LogDelabella2Occ(object? stream, string message)
    {
        _ = stream;
        Gravity gravity = Warning;
        switch (message.Length > 1 ? message[1] : '\0')
        {
            case 'E':
                gravity = Fail;
                break; // [ERR]
            case 'W':
                gravity = Trace;
                break; // [WRN]
            case 'N':
                gravity = Trace;
                break; // [NFO]
        }
        global::Kerf.Engine.Foundation.Message.Send(message, gravity);
        return 0;
    }
}
