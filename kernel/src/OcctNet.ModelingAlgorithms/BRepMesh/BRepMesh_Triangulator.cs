// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Triangulator.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Triangulator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using static Occt.BRepMesh_DegreeOfFreedom;
using static Kerf.Engine.Foundation.Gravity;

namespace Occt;

/// <summary>
/// Triangulates a planar polygon with holes given as wires of indices into a list of 3D points: the points are
/// projected on the plane of the given normal through the origin and triangulated by <see cref="BRepMesh_Delaun"/>
/// with the wires as frontier links. Wires of three or four points are triangulated directly.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. The triangles' node indices are the points' indices plus 1. Failures are reported to
/// the messenger (<see cref="Message.DefaultMessenger"/> unless one is set), which sends nothing by default
/// (decision 493). <c>MoniTool_TimerSentry</c> (under <c>CHECK_PERF</c>) is left out.
/// </remarks>
public sealed class BRepMesh_Triangulator
{
    private readonly NCollection_DynamicArray<Xyz> myXYZs;
    private readonly NCollection_List<NCollection_Sequence<int>> myWires;
    private readonly Plane3D myPlane;
    private Messenger? myMess;

    private BRepMesh_DataStructureOfDelaun? myMeshStructure;
    private readonly NCollection_DataMap<int, int> myTmpMap = new();
    private NCollection_DynamicArray<int>? myIndices;

    /// <summary>A triangulation of the given triangles over the given nodes.</summary>
    public static Triangulation ToPolyTriangulation(NCollection_Array1<Point3D> theNodes,
                                                         NCollection_List<Triangle1> thePolyTriangles)
    {
        NCollection_Array1<Triangle1> aTriangles = new(1, thePolyTriangles.Extent());
        NCollection_List<Triangle1>.Iterator itT = new(thePolyTriangles);
        for (int i = 1; itT.More(); itT.Next(), i++)
        {
            aTriangles.SetValue(i, itT.Value());
        }

        return new Triangulation(theNodes, aTriangles);
    }

    /// <summary>
    /// A triangulator of the wires (sequences of indices, from 0, into <paramref name="theXYZs"/>) in the plane of the
    /// given normal. The point list and the wires are kept by reference, as OCCT keeps references.
    /// </summary>
    public BRepMesh_Triangulator(NCollection_DynamicArray<Xyz> theXYZs,
                                 NCollection_List<NCollection_Sequence<int>> theWires,
                                 in Direction3D theNorm)
    {
        myXYZs = theXYZs;
        myWires = theWires;
        myPlane = new Plane3D(Constants.Origin(), theNorm);
    }

    /// <summary>Appends the triangles to <paramref name="thePolyTriangles"/>; false on failure.</summary>
    public bool Perform(NCollection_List<Triangle1> thePolyTriangles)
    {
        myMess ??= global::Kerf.Engine.Foundation.Message.DefaultMessenger();

        if (myWires.Extent() == 1)
        {
            NCollection_Sequence<int> aTmpWire = myWires.First();
            if (aTmpWire.Length() < 5)
            {
                // prepare triangles from simple wire (3 or 4 points)
                addTriange34(aTmpWire, thePolyTriangles);
                return true;
            }
        }

        if (!prepareMeshStructure())
        {
            return false;
        }

        if (!triangulate(thePolyTriangles))
        {
            return false;
        }

        return true;
    }

    /// <summary>Sets the messenger that failures are reported to.</summary>
    public void SetMessenger(Messenger? theMess) => myMess = theMess;

    private static int wireNodesNb(NCollection_List<NCollection_Sequence<int>> theWires)
    {
        int nbNodes = 0;
        NCollection_List<NCollection_Sequence<int>>.Iterator itW = new(theWires);
        for (int i = 1; itW.More(); itW.Next(), i++)
        {
            nbNodes += itW.Value().Length();
        }

        return nbNodes;
    }

    private static void appendTriangle(int theNode1,
                                       int theNode2,
                                       int theNode3,
                                       NCollection_Sequence<int> theW,
                                       NCollection_List<Triangle1> thePolyTriangles)
    {
        Triangle1 aT1 = new(theW[theNode1] + 1, theW[theNode2] + 1, theW[theNode3] + 1);
        thePolyTriangles.Append(aT1);
    }

    // auxiliary for makeTrianglesUsingBRepMesh
    private void addTriange34(NCollection_Sequence<int> theW, NCollection_List<Triangle1> thePolyTriangles)
    {
        // prepare triangles from simple wire (3 or 4 points)
        if (theW.Length() == 3)
        {
            Triangle1 aT = new(theW[1] + 1, theW[2] + 1, theW[3] + 1);
            thePolyTriangles.Append(aT);
        }
        else if (theW.Length() == 4)
        {
            double d13 = (myXYZs.Value(theW[1]) - myXYZs.Value(theW[3])).SquareModulus();
            double d24 = (myXYZs.Value(theW[2]) - myXYZs.Value(theW[4])).SquareModulus();
            bool use13 = true;
            if (d24 < d13)
            {
                // additional check for inner corner
                ReadOnlySpan<int> aPivotNodes = [2, 4, 1, 3];
                use13 = checkCondition(aPivotNodes, theW);
            }
            else
            {
                ReadOnlySpan<int> aPivotNodes = [1, 3, 2, 4];
                use13 = !checkCondition(aPivotNodes, theW);
            }

            if (use13)
            {
                appendTriangle(1, 2, 3, theW, thePolyTriangles);
                appendTriangle(3, 4, 1, theW, thePolyTriangles);
            }
            else
            {
                appendTriangle(1, 2, 4, theW, thePolyTriangles);
                appendTriangle(2, 3, 4, theW, thePolyTriangles);
            }
        }
    }

    // auxiliary for addTriange34
    private bool checkCondition(ReadOnlySpan<int> theNodes, NCollection_Sequence<int> theW)
    {
        Xyz aV0 = myXYZs.Value(theW[theNodes[1]]) - myXYZs.Value(theW[theNodes[0]]);
        Xyz aV1 = myXYZs.Value(theW[theNodes[2]]) - myXYZs.Value(theW[theNodes[0]]);
        Xyz aV2 = myXYZs.Value(theW[theNodes[3]]) - myXYZs.Value(theW[theNodes[0]]);

        Xyz aCross1 = aV0.Crossed(aV1);
        Xyz aCross2 = aV0.Crossed(aV2);
        return (aCross1.SquareModulus() < global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                || aCross2.SquareModulus() < global::Kerf.Engine.Foundation.Precision.SquareConfusion()
                || new Direction3D(aCross1).IsEqual(new Direction3D(aCross2), 0.01));
    }

    // performs initialization of mesh data structure.
    private bool prepareMeshStructure()
    {
        myIndices = new NCollection_DynamicArray<int>(wireNodesNb(myWires));
        myMeshStructure = new BRepMesh_DataStructureOfDelaun();

        // fill this structure created BRepMesh_Vertexes using 2d points received
        // by projection initial 3d point on plane.
        try
        {
            int aNumNode = 0;
            NCollection_List<NCollection_Sequence<int>>.Iterator itW = new(myWires);
            for (int i = 1; itW.More(); itW.Next(), i++)
            {
                NCollection_Sequence<int> aW = itW.Value();
                for (int nn = 1; nn <= aW.Length(); ++nn, ++aNumNode)
                {
                    Point2D aP2d = ProjLib.Project(myPlane, new Point3D(myXYZs[aW[nn]]));
                    BRepMesh_Vertex aVertex = new(aP2d.XY(), aNumNode, BRepMesh_Frontier);
                    int nnn = myMeshStructure.AddNode(aVertex);
                    myIndices.SetValue(aNumNode, nnn);
                    myTmpMap.Bind(aNumNode + 1, aW[nn] + 1);
                }

                int aOffset = aNumNode - aW.Length();
                for (int nn = 1; nn <= aW.Length(); ++nn)
                {
                    BRepMesh_Edge anEdge = new(myIndices.Value(aOffset + (nn - 1)),
                                               myIndices.Value(aOffset + (nn % aW.Length())),
                                               BRepMesh_Frontier);
                    myMeshStructure.AddLink(anEdge);
                }
            }
        }
        catch (Exception anException) when (IMeshTools_ModelAlgo.IsCaughtFailure(anException) || anException is FailureException)
        {
            // OCCT catches everything (catch (...)); the port catches what decision 529 maps OCCT's failures to.
            myMess!.Send("makeTrianglesUsingBRepMesh: Exception raised during filling of "
                         + "BRepMesh_DataStructureOfDelaun",
                         Fail);
            return false;
        }

        return true;
    }

    // auxiliary for triangulation
    private bool triangulate(NCollection_List<Triangle1> thePolyTriangles)
    {
        try
        {
            // perform triangulation used created aMeshStructure
            _ = new BRepMesh_Delaun(myMeshStructure!, myIndices!);
            NCollection_PackedMapOfInteger aTriangles = myMeshStructure!.ElementsOfDomain();
            if (aTriangles.Extent() < 1)
            {
                return false;
            }

            // prepare Poly_Triangles from result triangles and add to returned list
            Span<int> aTri2d = stackalloc int[3];
            for (NCollection_PackedMapOfInteger.Iterator aTriIter = new(aTriangles); aTriIter.More(); aTriIter.Next())
            {
                int aTriangleId = aTriIter.Key();
                BRepMesh_Triangle aTriangle = myMeshStructure.GetElement(aTriangleId);
                if (aTriangle.Movability() == BRepMesh_Deleted)
                {
                    continue;
                }

                myMeshStructure.ElementNodes(aTriangle, aTri2d);

                Triangle1 aT = new(myTmpMap.Find(aTri2d[0]),
                                       myTmpMap.Find(aTri2d[1]),
                                       myTmpMap.Find(aTri2d[2]));
                thePolyTriangles.Append(aT);
            }
        }
        catch (FailureException aFailure)
        {
            string aStr = "makeTrianglesUsingBRepMesh: Exception raised during polygon triangulation: ";
            aStr += aFailure.Message;
            myMess!.Send(aStr, Fail);
            return false;
        }

        return true;
    }
}
