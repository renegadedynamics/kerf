// Port of: src/FoundationClasses/TKMath/Poly/Poly_MergeNodesTool.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_MergeNodesTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.InteropServices;

namespace Occt;

/// <summary>
/// Merges the nodes of triangulations, for visualization: coincident nodes (exactly, or within a tolerance) are merged,
/// unless the normals of their elements differ by more than the smooth angle, which keeps sharp edges sharp.
/// Degenerate elements are dropped; equal elements can be merged too.
/// </summary>
/// <remarks>
/// The node map is OCCT's hash map, ported with its buckets, hashes and resizing, so that when several merged nodes
/// could match a new one the same one is chosen as in OCCT. Positions are compared in single precision, as in OCCT.
/// </remarks>
public class Poly_MergeNodesTool : Standard_Transient
{
    /// <summary>
    /// Merges the nodes of a triangulation and returns the new one, or null if there is nothing to merge
    /// (<paramref name="theTris"/> null or too small, or, unless <paramref name="theToForce"/>, nothing merged).
    /// </summary>
    /// <param name="theTris">triangulation to merge</param>
    /// <param name="theTrsf">transformation to apply</param>
    /// <param name="theToReverse">reverse the order of the triangle nodes</param>
    /// <param name="theSmoothAngle">merge angle in radians</param>
    /// <param name="theMergeTolerance">linear merge tolerance</param>
    /// <param name="theToForce">return the merged triangulation even if it has as many nodes and triangles as the input</param>
    public static Poly_Triangulation? MergeNodes(Poly_Triangulation? theTris,
                                                 in gp_Trsf theTrsf,
                                                 bool theToReverse,
                                                 double theSmoothAngle,
                                                 double theMergeTolerance = 0.0,
                                                 bool theToForce = true)
    {
        if (theTris is null || theTris.NbNodes() < 3 || theTris.NbTriangles() < 1)
        {
            return null;
        }

        var aMergeTool = new Poly_MergeNodesTool(theSmoothAngle, theMergeTolerance, theTris.NbTriangles());
        aMergeTool.AddTriangulation(theTris, theTrsf, theToReverse);
        if (!theToForce && aMergeTool.NbNodes() == theTris.NbNodes() && aMergeTool.NbElements() == theTris.NbTriangles())
        {
            return null;
        }

        return aMergeTool.Result();
    }

    // Returns initial number of buckets for the map.
    private static int initialNbBuckets(int theNbFacets)
        => theNbFacets > 0
               ? theNbFacets * 2 // consider ratio 1:2 (NbTriangles:MergedNodes) as expected
               : 995329;         // default initial value for mesh of unknown size

    /// <param name="theSmoothAngle">smooth angle in radians, or 0 to disable merging by angle</param>
    /// <param name="theMergeTolerance">largest distance between merged nodes</param>
    /// <param name="theNbFacets">estimated number of facets, to size the node map</param>
    public Poly_MergeNodesTool(double theSmoothAngle, double theMergeTolerance = 0.0, int theNbFacets = -1)
    {
        myPolyData = new Poly_Triangulation();
        myNodeIndexMap = new MergedNodesMap((theSmoothAngle > 0.0 || theMergeTolerance > 0.0) ? initialNbBuckets(theNbFacets) : 1);
        myElemMap = new NCollection_Map<NCollection_Vec4<int>>();
        myNodeInds = new NCollection_Vec4<int>(0, 0, 0, -1);
        myTriNormal = new NCollection_Vec3<float>(0.0f, 0.0f, 1.0f);
        myUnitFactor = 1.0;
        myNbNodes = 0;
        myNbElems = 0;
        myNbDegenElems = 0;
        myNbMergedElems = 0;
        myToDropDegenerative = true;
        myToMergeElems = false;
        SetMergeAngle(theSmoothAngle);
        SetMergeTolerance(theMergeTolerance);
    }

    /// <summary>The merge tolerance; 0 by default (only nodes with exactly the same coordinates are merged).</summary>
    public double MergeTolerance() => myNodeIndexMap.MergeTolerance();

    public void SetMergeTolerance(double theTolerance) => myNodeIndexMap.SetMergeTolerance(theTolerance);

    /// <summary>The merge angle in radians; 0 by default (nodes are merged only if their normals are exactly the same).</summary>
    public double MergeAngle() => myNodeIndexMap.MergeAngle();

    public void SetMergeAngle(double theAngleRad) => myNodeIndexMap.SetMergeAngle(theAngleRad);

    /// <summary>True if nodes with opposite normals are merged; false by default.</summary>
    public bool ToMergeOpposite() => myNodeIndexMap.ToMergeOpposite();

    public void SetMergeOpposite(bool theToMerge) => myNodeIndexMap.SetMergeOpposite(theToMerge);

    /// <summary>Sets the scale factor applied to the output nodes.</summary>
    public void SetUnitFactor(double theUnitFactor) => myUnitFactor = theUnitFactor;

    /// <summary>True if degenerate elements are dropped; true by default.</summary>
    public bool ToDropDegenerative() => myToDropDegenerative;

    public void SetDropDegenerative(bool theToDrop) => myToDropDegenerative = theToDrop;

    /// <summary>True if equal elements are merged; false by default.</summary>
    public bool ToMergeElems() => myToMergeElems;

    public void SetMergeElems(bool theToMerge) => myToMergeElems = theToMerge;

    /// <summary>The normal of the element being pushed (its first three nodes).</summary>
    public NCollection_Vec3<float> computeTriNormal()
    {
        gp_XYZ aVec01 = myPlaces[1] - myPlaces[0];
        gp_XYZ aVec02 = myPlaces[2] - myPlaces[0];
        gp_XYZ aCross = aVec01 ^ aVec02;
        var aNorm = new NCollection_Vec3<float>((float)aCross.X(), (float)aCross.Y(), (float)aCross.Z());
        return aNorm.Normalized();
    }

    /// <summary>Adds the triangles of a triangulation.</summary>
    public void AddTriangulation(Poly_Triangulation? theTris) => AddTriangulation(theTris, new gp_Trsf(), false);

    /// <summary>Adds the triangles of a triangulation, transformed, with their nodes in reverse order if asked.</summary>
    public virtual void AddTriangulation(Poly_Triangulation? theTris, in gp_Trsf theTrsf, bool theToReverse = false)
    {
        if (theTris is null)
        {
            return;
        }

        if (myPolyData is not null && myPolyData.NbNodes() == 0)
        {
            // preallocate optimistically
            myPolyData.SetDoublePrecision(theTris.IsDoublePrecision());
            myPolyData.ResizeNodes(theTris.NbNodes(), false);
            myPolyData.ResizeTriangles(theTris.NbTriangles(), false);
        }

        for (int anElemIter = 1; anElemIter <= theTris.NbTriangles(); ++anElemIter)
        {
            Poly_Triangle anElem = theTris.Triangle(anElemIter);
            if (theToReverse)
            {
                anElem = new Poly_Triangle(anElem.Value(1), anElem.Value(3), anElem.Value(2));
            }

            for (int aTriNodeIter = 0; aTriNodeIter < 3; ++aTriNodeIter)
            {
                gp_Pnt aNode = theTris.Node(anElem.Value(aTriNodeIter + 1)).Transformed(theTrsf);
                myPlaces[aTriNodeIter] = aNode.XYZ();
            }

            PushLastTriangle();
        }
    }

    /// <summary>The result triangulation, truncated to its size, or null if there is no output triangulation.</summary>
    public Poly_Triangulation? Result()
    {
        if (myPolyData is null)
        {
            return null;
        }

        // compress data
        // OCCT: throws Standard_RangeError when no node was added (an aliased array can't be empty)
        myPolyData.ResizeNodes(myNbNodes, true);
        myPolyData.ResizeTriangles(myNbElems, true);
        return myPolyData;
    }

    /// <summary>Adds a triangle.</summary>
    public void AddTriangle(ReadOnlySpan<gp_XYZ> theElemNodes) => AddElement(theElemNodes, 3);

    /// <summary>Adds a quad.</summary>
    public void AddQuad(ReadOnlySpan<gp_XYZ> theElemNodes) => AddElement(theElemNodes, 4);

    /// <summary>Adds a triangle or a quad: <paramref name="theNbNodes"/> is 3 or 4.</summary>
    public void AddElement(ReadOnlySpan<gp_XYZ> theElemNodes, int theNbNodes)
    {
        if (theNbNodes != 3 && theNbNodes != 4)
        {
            throw new Standard_ProgramError("Poly_MergeNodesTool::AddElement() - Internal error");
        }

        myPlaces[0] = theElemNodes[0];
        myPlaces[1] = theElemNodes[1];
        myPlaces[2] = theElemNodes[2];
        if (theNbNodes == 4)
        {
            myPlaces[3] = theElemNodes[3];
        }

        PushLastElement(theNbNodes);
    }

    /// <summary>The node at a position (0..3) of the element to be pushed, for writing.</summary>
    public ref gp_XYZ ChangeElementNode(int theIndex) => ref myPlaces[theIndex];

    /// <summary>Adds a triangle or a quad whose nodes were set with <see cref="ChangeElementNode"/>.</summary>
    public void PushLastElement(int theNbNodes)
    {
        if (theNbNodes != 3 && theNbNodes != 4)
        {
            throw new Standard_ProgramError("Poly_MergeNodesTool::PushLastElement() - Internal error");
        }

        bool isOpposite = false;
        myNodeInds[3] = -1;
        if (myNodeIndexMap.HasMergeAngle() || myNodeIndexMap.HasMergeTolerance())
        {
            if (!myNodeIndexMap.ToMergeAnyAngle())
            {
                myTriNormal = computeTriNormal();
            }

            pushNodeCheck(ref isOpposite, 0);
            pushNodeCheck(ref isOpposite, 1);
            pushNodeCheck(ref isOpposite, 2);
            if (theNbNodes == 4)
            {
                pushNodeCheck(ref isOpposite, 3);
            }
        }
        else
        {
            pushNodeNoMerge(0);
            pushNodeNoMerge(1);
            pushNodeNoMerge(2);
            if (theNbNodes == 4)
            {
                pushNodeNoMerge(3);
            }
        }

        if (myToDropDegenerative)
        {
            // warning - removing degenerate elements may produce unused nodes
            if (myNodeInds[0] == myNodeInds[1] || myNodeInds[0] == myNodeInds[2] || myNodeInds[1] == myNodeInds[2])
            {
                if (theNbNodes == 4)
                {
                    //
                }
                else
                {
                    ++myNbDegenElems;
                    return;
                }
            }
        }

        if (myToMergeElems)
        {
            NCollection_Vec4<int> aSorted = myNodeInds;
            // theNbNodes is bounded by the Vec4 capacity; clamp defensively.
            int aNbToSort = Min(theNbNodes, 4);
            aSorted.ChangeData()[..aNbToSort].Sort();
            if (!myElemMap.Add(aSorted))
            {
                ++myNbMergedElems;
                return;
            }
        }

        ++myNbElems;
        if (myPolyData is not null)
        {
            if (myPolyData.NbTriangles() < myNbElems)
            {
                myPolyData.ResizeTriangles(myNbElems * 2, true);
            }

            myPolyData.SetTriangle(myNbElems, new Poly_Triangle(myNodeInds[0] + 1, myNodeInds[1] + 1, myNodeInds[2] + 1));
            if (theNbNodes == 4)
            {
                ++myNbElems;
                if (myPolyData.NbTriangles() < myNbElems)
                {
                    myPolyData.ResizeTriangles(myNbElems * 2, true);
                }

                myPolyData.SetTriangle(myNbElems, new Poly_Triangle(myNodeInds[0] + 1, myNodeInds[2] + 1, myNodeInds[3] + 1));
            }
        }
    }

    /// <summary>Adds a triangle whose nodes were set with <see cref="ChangeElementNode"/>.</summary>
    public void PushLastTriangle() => PushLastElement(3);

    /// <summary>Adds a quad whose nodes were set with <see cref="ChangeElementNode"/>.</summary>
    public void PushLastQuad() => PushLastElement(4);

    /// <summary>The output node index (from 0) of a node of the last element pushed.</summary>
    public int ElementNodeIndex(int theIndex) => myNodeInds[theIndex];

    /// <summary>The number of nodes.</summary>
    public int NbNodes() => myNbNodes;

    /// <summary>The number of elements.</summary>
    public int NbElements() => myNbElems;

    /// <summary>The number of degenerate elements dropped.</summary>
    public int NbDegenerativeElems() => myNbDegenElems;

    /// <summary>The number of equal elements merged.</summary>
    public int NbMergedElems() => myNbMergedElems;

    /// <summary>
    /// The output triangulation, for writing. Set to null to use the tool only as a merge map, filling an external
    /// mesh from <see cref="ElementNodeIndex"/>.
    /// </summary>
    public ref Poly_Triangulation? ChangeOutput() => ref myPolyData;

    // Push triangle node with normal angle comparison.
    private void pushNodeCheck(ref bool theIsOpposite, int theTriNode)
    {
        int aNodeIndex = myNbNodes;
        gp_XYZ aPlace = myPlaces[theTriNode];
        var aVec3 = new NCollection_Vec3<float>((float)aPlace.X(), (float)aPlace.Y(), (float)aPlace.Z());
        if (myNodeIndexMap.Bind(ref aNodeIndex, ref theIsOpposite, aVec3, myTriNormal))
        {
            ++myNbNodes;
            if (myPolyData is not null)
            {
                if (myPolyData.NbNodes() < myNbNodes)
                {
                    myPolyData.ResizeNodes(myNbNodes * 2, true);
                }

                myPolyData.SetNode(myNbNodes, new gp_Pnt(aPlace * myUnitFactor));
            }
        }

        myNodeInds[theTriNode] = aNodeIndex;
    }

    // Push triangle node without merging vertices.
    private void pushNodeNoMerge(int theTriNode)
    {
        int aNodeIndex = myNbNodes;
        gp_XYZ aPlace = myPlaces[theTriNode] * myUnitFactor;

        ++myNbNodes;
        if (myPolyData is not null)
        {
            if (myPolyData.NbNodes() < myNbNodes)
            {
                myPolyData.ResizeNodes(myNbNodes * 2, true);
            }

            myPolyData.SetNode(myNbNodes, new gp_Pnt(aPlace));
        }

        myNodeInds[theTriNode] = aNodeIndex;
    }

    // Pair holding Vec3 and Normal to the triangle
    private readonly struct Vec3AndNormal(in NCollection_Vec3<float> thePos, in NCollection_Vec3<float> theNorm)
    {
        public readonly NCollection_Vec3<float> Pos = thePos;   // position
        public readonly NCollection_Vec3<float> Norm = theNorm; // normal to the element
    }

    // Custom map class with key as Node + element normal and value as Node index: OCCT's NCollection_BaseMap,
    // with its buckets.
    private sealed class MergedNodesMap
    {
        // Map node.
        private sealed class DataMapNode(in NCollection_Vec3<float> thePos, in NCollection_Vec3<float> theNorm, int theItem, DataMapNode? theNext)
        {
            private readonly Vec3AndNormal myKey = new(thePos, theNorm);

            public DataMapNode? Next = theNext;

            public int Value = theItem;

            public ref readonly Vec3AndNormal Key() => ref myKey;
        }

        private static readonly NCollection_Vec3<long>[] THE_NEIGHBRS =
        [
            new(-1, 0, 0), new(1, 0, 0), new(0, -1, 0), new(0, 1, 0),
            new(0, 0, -1), new(0, 0, 1), new(-1, -1, 0), new(1, -1, 0),
            new(1, 1, 0), new(-1, 1, 0), new(0, -1, -1), new(0, 1, -1),
            new(0, 1, 1), new(0, -1, 1), new(-1, 0, -1), new(1, 0, -1),
            new(1, 0, 1), new(-1, 0, 1), new(-1, -1, -1), new(1, -1, -1),
            new(-1, 1, -1), new(1, 1, -1), new(-1, -1, 1), new(1, -1, 1),
            new(-1, 1, 1), new(1, 1, 1),
        ];

        // NCollection_BaseMap's state
        private DataMapNode?[]? myData1;
        private long myNbBuckets;
        private int mySize;

        private float myTolerance;       // linear tolerance for comparison
        private float myInvTol;          // inversed linear tolerance for comparison
        private float myAngle;           // angle for comparison
        private float myAngleCos;        // angle cosine for comparison
        private bool myToMergeOpposite;  // merge nodes with opposite normals

        public MergedNodesMap(int theNbBuckets)
        {
            myNbBuckets = theNbBuckets;
            myTolerance = 0.0f;
            myInvTol = 0.0f;
            myAngle = 1.0f;
            myAngleCos = 0.0f;
            myToMergeOpposite = false;
        }

        public double MergeAngle() => myAngle;

        public void SetMergeAngle(double theAngleRad)
        {
            myAngle = (float)theAngleRad;
            myAngleCos = (float)Math.Cos(theAngleRad);
        }

        // 0 angle means angles should much without a tolerance.
        public bool HasMergeAngle() => myAngle > 0.0f;

        // TRUE if merge angle comparison can be skipped (angle is close to 90 degrees).
        public bool ToMergeAnyAngle() => myAngleCos <= 0.01f;

        public bool ToMergeOpposite() => myToMergeOpposite;

        public void SetMergeOpposite(bool theToMerge) => myToMergeOpposite = theToMerge;

        public double MergeTolerance() => myTolerance;

        public void SetMergeTolerance(double theTolerance)
        {
            myTolerance = (float)theTolerance;
            myInvTol = 0.0f;
            if (myTolerance > 0.0f)
            {
                myInvTol = (float)(1.0 / theTolerance);
            }
        }

        public bool HasMergeTolerance() => myTolerance > 0.0f;

        // Bind node to the map or find existing one; TRUE if node was not bound already.
        public bool Bind(ref int theIndex, ref bool theIsOpposite, in NCollection_Vec3<float> thePos, in NCollection_Vec3<float> theNorm)
        {
            if (Resizable())
            {
                ReSize(mySize);
            }

            DataMapNode?[] aData = myData1!;
            long aHash = hashCode(thePos, theNorm, myNbBuckets);
            for (DataMapNode? aNodeIter = aData[aHash]; aNodeIter is not null; aNodeIter = aNodeIter.Next)
            {
                if (isEqual(aNodeIter.Key(), thePos, theNorm, ref theIsOpposite))
                {
                    theIndex = aNodeIter.Value;
                    return false;
                }
            }

            if (myInvTol > 0.0f)
            {
                NCollection_Vec3<long> anIndexCnt = vec3ToCell(thePos);
                for (int aNeigIter = 0; aNeigIter < 26; ++aNeigIter)
                {
                    NCollection_Vec3<long> anIndex = anIndexCnt + THE_NEIGHBRS[aNeigIter];
                    long aHashEx = vec3iHashCode(anIndex, myNbBuckets);
                    for (DataMapNode? aNodeIter = aData[aHashEx]; aNodeIter is not null; aNodeIter = aNodeIter.Next)
                    {
                        if (isEqual(aNodeIter.Key(), thePos, theNorm, ref theIsOpposite))
                        {
                            theIndex = aNodeIter.Value;
                            return false;
                        }
                    }
                }
            }

            // theIsOpposite = false;
            aData[aHash] = new DataMapNode(thePos, theNorm, theIndex, aData[aHash]);
            ++mySize;
            return true;
        }

        // ReSize the map (NCollection_BaseMap::BeginResize, the rehash, EndResize).
        public void ReSize(int theSize)
        {
            long anExtent = theSize < 0 ? 0 : theSize;
            long aNbNewBuck = NCollection_Primes.NextPrimeForMap(anExtent);
            if (aNbNewBuck <= myNbBuckets)
            {
                if (myData1 is not null)
                {
                    return;
                }

                aNbNewBuck = myNbBuckets;
            }

            var aNewData = new DataMapNode?[aNbNewBuck + 1];
            if (myData1 is DataMapNode?[] anOldData)
            {
                for (long anOldBuckIter = 0; anOldBuckIter <= myNbBuckets; ++anOldBuckIter)
                {
                    for (DataMapNode? anOldNodeIter = anOldData[anOldBuckIter]; anOldNodeIter is not null;)
                    {
                        long aNewHash = hashCode(anOldNodeIter.Key().Pos, anOldNodeIter.Key().Norm, aNbNewBuck);
                        DataMapNode? aNextNode = anOldNodeIter.Next;
                        anOldNodeIter.Next = aNewData[aNewHash];
                        aNewData[aNewHash] = anOldNodeIter;
                        anOldNodeIter = aNextNode;
                    }
                }
            }

            myNbBuckets = aNbNewBuck;
            myData1 = aNewData;
        }

        private bool Resizable() => mySize == 0 || (mySize > myNbBuckets);

        // Return cell index for specified 3D point and inverted cell size.
        private NCollection_Vec3<long> vec3ToCell(in NCollection_Vec3<float> thePnt) => NCollection_Vec3<long>.From(thePnt * myInvTol);

        // Hash code for integer vec3 (copied from NCollection_CellFilter).
        private static long vec3iHashCode(in NCollection_Vec3<long> theVec, long theUpper)
        {
            const int aShiftBits = (8 * sizeof(long) - 1) / 3;
            ulong aHashCode = 0;
            aHashCode = (aHashCode << aShiftBits) ^ (ulong)theVec[0];
            aHashCode = (aHashCode << aShiftBits) ^ (ulong)theVec[1];
            aHashCode = (aHashCode << aShiftBits) ^ (ulong)theVec[2];
            return (long)(aHashCode % (ulong)theUpper) + 1;
        }

        // Compute hash code.
        private long hashCode(in NCollection_Vec3<float> thePos, in NCollection_Vec3<float> theNorm, long theUpper)
        {
            _ = theNorm;
            if (myInvTol <= 0.0f)
            {
                // compute DJB2 hash of a string
                // OCCT hashes the bytes as char: signed on MSVC (followed here), unsigned on ARM/GCC
                ReadOnlySpan<byte> aBytes = MemoryMarshal.AsBytes(new ReadOnlySpan<NCollection_Vec3<float>>(in thePos));
                uint aHashCode = 0;
                foreach (byte c in aBytes)
                {
                    aHashCode = ((aHashCode << 5) + aHashCode) ^ (uint)(sbyte)c;
                }

                return (long)(aHashCode % (ulong)theUpper) + 1;
            }

            NCollection_Vec3<long> anIndex = vec3ToCell(thePos);
            return vec3iHashCode(anIndex, theUpper);
        }

        // Compare two vectors with inversed tolerance.
        private bool vec3AreEqual(in NCollection_Vec3<float> theKey1, in NCollection_Vec3<float> theKey2)
        {
            if (myInvTol <= 0.0f)
            {
                return theKey1.IsEqual(theKey2);
            }

            // tolerance should be smaller than triangle size to avoid artifacts
            // const CellVec3i anIndex1 = vec3ToCell (theKey1);
            // const CellVec3i anIndex2 = vec3ToCell (theKey2);
            // return anIndex1.IsEqual (anIndex2);
            if (Math.Abs(theKey1.x() - theKey2.x()) > myTolerance)
            {
                return false;
            }

            if (Math.Abs(theKey1.y() - theKey2.y()) > myTolerance)
            {
                return false;
            }

            if (Math.Abs(theKey1.z() - theKey2.z()) > myTolerance)
            {
                return false;
            }

            return true;
        }

        // Compare two nodes.
        private bool isEqual(in Vec3AndNormal theKey1, in NCollection_Vec3<float> thePos2, in NCollection_Vec3<float> theNorm2, ref bool theIsOpposite)
        {
            if (!vec3AreEqual(theKey1.Pos, thePos2))
            {
                return false;
            }

            float aCosinus = theKey1.Norm.Dot(theNorm2);
            if (aCosinus >= myAngleCos)
            {
                // theIsOpposite = false;
                return true;
            }
            else if (myToMergeOpposite && aCosinus <= -myAngleCos)
            {
                theIsOpposite = true;
                return true;
            }

            return false;
        }
    }

    private Poly_Triangulation? myPolyData;                         // output triangulation
    private readonly MergedNodesMap myNodeIndexMap;                  // map of merged nodes
    private readonly NCollection_Map<NCollection_Vec4<int>> myElemMap; // map of elements
    private NCollection_Vec4<int> myNodeInds;                        // current element indexes
    private NCollection_Vec3<float> myTriNormal;                     // current triangle normal
    private readonly gp_XYZ[] myPlaces = new gp_XYZ[4];              // current triangle/quad coordinates to push
    private double myUnitFactor;                                     // scale factor to apply
    private int myNbNodes;                                           // number of output nodes
    private int myNbElems;                                           // number of output elements
    private int myNbDegenElems;                                      // number of degenerated elements
    private int myNbMergedElems;                                     // number of merged elements
    private bool myToDropDegenerative;                               // flag to filter our degenerate elements
    private bool myToMergeElems;                                     // flag to merge elements
}
