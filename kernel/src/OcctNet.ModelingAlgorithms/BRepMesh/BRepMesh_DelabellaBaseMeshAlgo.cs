// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelabellaBaseMeshAlgo.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelabellaBaseMeshAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2019-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using static Occt.BRepMesh_DegreeOfFreedom;
using static Kerf.Engine.Foundation.Gravity;

namespace Occt;

/// <summary>
/// Builds a face's triangulation with the Delabella library (<see cref="IDelaBella"/>), whose result BRepMesh_Delaun
/// then constrains to the face's boundary.
/// </summary>
/// <remarks>
/// Delabella's messages go to <see cref="Message"/>'s process-wide messenger, which sends nothing by default
/// (decision 493): an error as a failure, a warning or a note as a trace.
/// </remarks>
public class BRepMesh_DelabellaBaseMeshAlgo : BRepMesh_CustomBaseMeshAlgo
{
    /// <summary>An algorithm.</summary>
    public BRepMesh_DelabellaBaseMeshAlgo()
    {
    }

    /// <summary>Builds the base triangulation with Delabella.</summary>
    protected override void buildBaseTriangulation()
    {
        BRepMesh_DataStructureOfDelaun aStructure = getStructure();

        B2d aBox = new();
        int aNodesNb = aStructure.NbNodes();
        double[] aPoints = new double[2 * (aNodesNb + 4)];
        for (int aNodeIt = 0; aNodeIt < aNodesNb; ++aNodeIt)
        {
            BRepMesh_Vertex aVertex = aStructure.GetNode(aNodeIt + 1);

            int aBaseIdx = 2 * aNodeIt;
            aPoints[aBaseIdx + 0] = aVertex.Coord().X();
            aPoints[aBaseIdx + 1] = aVertex.Coord().Y();

            aBox.Add(new Point2D(aVertex.Coord()));
        }

        aBox.Enlarge(0.1 * (aBox.CornerMax() - aBox.CornerMin()).Modulus());
        Xy aMin = aBox.CornerMin();
        Xy aMax = aBox.CornerMax();

        aPoints[2 * aNodesNb + 0] = aMin.X();
        aPoints[2 * aNodesNb + 1] = aMin.Y();
        aStructure.AddNode(
            new BRepMesh_Vertex(aPoints[2 * aNodesNb + 0], aPoints[2 * aNodesNb + 1], BRepMesh_Free));

        aPoints[2 * aNodesNb + 2] = aMax.X();
        aPoints[2 * aNodesNb + 3] = aMin.Y();
        aStructure.AddNode(
            new BRepMesh_Vertex(aPoints[2 * aNodesNb + 2], aPoints[2 * aNodesNb + 3], BRepMesh_Free));

        aPoints[2 * aNodesNb + 4] = aMax.X();
        aPoints[2 * aNodesNb + 5] = aMax.Y();
        aStructure.AddNode(
            new BRepMesh_Vertex(aPoints[2 * aNodesNb + 4], aPoints[2 * aNodesNb + 5], BRepMesh_Free));

        aPoints[2 * aNodesNb + 6] = aMin.X();
        aPoints[2 * aNodesNb + 7] = aMax.Y();
        aStructure.AddNode(
            new BRepMesh_Vertex(aPoints[2 * aNodesNb + 6], aPoints[2 * aNodesNb + 7], BRepMesh_Free));

        double aDiffX = (aMax.X() - aMin.X());
        double aDiffY = (aMax.Y() - aMin.Y());
        for (int i = 0; i < aPoints.Length; i += 2)
        {
            aPoints[i + 0] = (aPoints[i + 0] - aMin.X()) / aDiffX - 0.5;
            aPoints[i + 1] = (aPoints[i + 1] - aMin.Y()) / aDiffY - 0.5;
        }

        IDelaBella? aTriangulator = IDelaBella.Create();
        if (aTriangulator is null) // should never happen
        {
            throw new ProgramErrorException("BRepMesh_DelabellaBaseMeshAlgo::buildBaseTriangulation: unable "
                                            + "creating a triangulation algorithm");
        }

        aTriangulator.SetErrLog(logDelabella2Occ, null);
        try
        {
            int aVerticesNb = aTriangulator.Triangulate(aPoints.Length / 2,
                                                        aPoints.AsSpan(0),
                                                        aPoints.AsSpan(1),
                                                        2 * sizeof(double));

            if (aVerticesNb > 0)
            {
                Span<int> aNodes = stackalloc int[3];
                Span<int> aEdges = stackalloc int[3];
                Span<bool> aOrientations = stackalloc bool[3];
                DelaBella_Triangle? aTrianglePtr = aTriangulator.GetFirstDelaunayTriangle();
                while (aTrianglePtr is not null)
                {
                    aNodes[0] = aTrianglePtr.v[0]!.i + 1;
                    aNodes[1] = aTrianglePtr.v[2]!.i + 1;
                    aNodes[2] = aTrianglePtr.v[1]!.i + 1;

                    for (int k = 0; k < 3; ++k)
                    {
                        BRepMesh_Edge aLink = new(aNodes[k], aNodes[(k + 1) % 3], BRepMesh_Free);

                        int aLinkInfo = aStructure.AddLink(aLink);
                        aEdges[k] = Math.Abs(aLinkInfo);
                        aOrientations[k] = aLinkInfo > 0;
                    }

                    BRepMesh_Triangle aTriangle = new(aEdges, aOrientations, BRepMesh_Free);
                    aStructure.AddElement(aTriangle);

                    aTrianglePtr = aTrianglePtr.next;
                }
            }

            aTriangulator.Destroy();
            aTriangulator = null;
        }
        catch (FailureException theException)
        {
            if (aTriangulator is not null)
            {
                aTriangulator.Destroy();
                aTriangulator = null;
            }

            throw new FailureException(theException.Message);
        }
        catch (Exception anException) when (anException is not (ImmutableObjectException or LockedShapeException))
        {
            if (aTriangulator is not null)
            {
                aTriangulator.Destroy();
                aTriangulator = null;
            }

            throw new FailureException("BRepMesh_DelabellaBaseMeshAlgo::buildBaseTriangulation: exception in "
                                       + "triangulation algorithm");
        }
    }

    // Redirect algorithm messages to OCCT messenger (Delabella gives the formatted text).
    private static int logDelabella2Occ(object? theStream, string theMessage)
    {
        _ = theStream;
        Gravity aGravity = Warning;
        switch (theMessage.Length > 1 ? theMessage[1] : '\0')
        {
            case 'E':
                aGravity = Fail;
                break; // [ERR]
            case 'W':
                aGravity = Trace;
                break; // [WRN]
            case 'N':
                aGravity = Trace;
                break; // [NFO]
        }
        global::Kerf.Engine.Foundation.Message.Send(theMessage, aGravity);
        return 0;
    }
}
