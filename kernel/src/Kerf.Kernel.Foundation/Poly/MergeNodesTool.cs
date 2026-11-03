// Port of: src/FoundationClasses/TKMath/Poly/Poly_MergeNodesTool.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_MergeNodesTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using System.Runtime.InteropServices;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Merges the nodes of triangulations, for visualization: coincident nodes (exactly, or within a tolerance) are merged,
/// unless the normals of their elements differ by more than the smooth angle, which keeps sharp edges sharp.
/// Degenerate elements are dropped; equal elements can be merged too.
/// </summary>
/// <remarks>
/// The node map is OCCT's hash map, ported with its buckets, hashes and resizing, so that when several merged nodes
/// could match a new one the same one is chosen as in OCCT. Positions are compared in single precision, as in OCCT.
/// </remarks>
public class MergeNodesTool : Transient
{
    /// <summary>
    /// Merges the nodes of a triangulation and returns the new one, or null if there is nothing to merge
    /// (<paramref name="tris"/> null or too small, or, unless <paramref name="toForce"/>, nothing merged).
    /// </summary>
    /// <param name="tris">triangulation to merge</param>
    /// <param name="trsf">transformation to apply</param>
    /// <param name="toReverse">reverse the order of the triangle nodes</param>
    /// <param name="smoothAngle">merge angle in radians</param>
    /// <param name="mergeTolerance">linear merge tolerance</param>
    /// <param name="toForce">return the merged triangulation even if it has as many nodes and triangles as the input</param>
    public static Triangulation? MergeNodes(Triangulation? tris,
                                                 in Transform3D trsf,
                                                 bool toReverse,
                                                 double smoothAngle,
                                                 double mergeTolerance = 0.0,
                                                 bool toForce = true)
    {
        if (tris is null || tris.NodeCount < 3 || tris.TriangleCount < 1)
        {
            return null;
        }

        var mergeTool = new MergeNodesTool(smoothAngle, mergeTolerance, tris.TriangleCount);
        mergeTool.AddTriangulation(tris, trsf, toReverse);
        if (!toForce && mergeTool.NodeCount == tris.NodeCount && mergeTool.ElementCount == tris.TriangleCount)
        {
            return null;
        }

        return mergeTool.Result();
    }

    // Returns initial number of buckets for the map.
    private static int InitialNbBuckets(int nbFacets)
        => nbFacets > 0
               ? nbFacets * 2 // consider ratio 1:2 (NbTriangles:MergedNodes) as expected
               : 995329;         // default initial value for mesh of unknown size

    /// <param name="smoothAngle">smooth angle in radians, or 0 to disable merging by angle</param>
    /// <param name="mergeTolerance">largest distance between merged nodes</param>
    /// <param name="nbFacets">estimated number of facets, to size the node map</param>
    public MergeNodesTool(double smoothAngle, double mergeTolerance = 0.0, int nbFacets = -1)
    {
        _polyData = new Triangulation();
        _nodeIndexMap = new MergedNodesMap((smoothAngle > 0.0 || mergeTolerance > 0.0) ? InitialNbBuckets(nbFacets) : 1);
        _elemMap = new HashSet<NCollection_Vec4<int>>();
        _nodeInds = new NCollection_Vec4<int>(0, 0, 0, -1);
        _triNormal = new NCollection_Vec3<float>(0.0f, 0.0f, 1.0f);
        _unitFactor = 1.0;
        _nbNodes = 0;
        _nbElems = 0;
        _nbDegenElems = 0;
        _nbMergedElems = 0;
        _toDropDegenerative = true;
        _toMergeElems = false;
        SetMergeAngle(smoothAngle);
        SetMergeTolerance(mergeTolerance);
    }

    /// <summary>The merge tolerance; 0 by default (only nodes with exactly the same coordinates are merged).</summary>
    public double MergeTolerance() => _nodeIndexMap.MergeTolerance();

    public void SetMergeTolerance(double tolerance) => _nodeIndexMap.SetMergeTolerance(tolerance);

    /// <summary>The merge angle in radians; 0 by default (nodes are merged only if their normals are exactly the same).</summary>
    public double MergeAngle() => _nodeIndexMap.MergeAngle();

    public void SetMergeAngle(double angleRad) => _nodeIndexMap.SetMergeAngle(angleRad);

    /// <summary>True if nodes with opposite normals are merged; false by default.</summary>
    public bool ToMergeOpposite() => _nodeIndexMap.ToMergeOpposite();

    public void SetMergeOpposite(bool toMerge) => _nodeIndexMap.SetMergeOpposite(toMerge);

    /// <summary>Sets the scale factor applied to the output nodes.</summary>
    public void SetUnitFactor(double unitFactor) => _unitFactor = unitFactor;

    /// <summary>True if degenerate elements are dropped; true by default.</summary>
    public bool ToDropDegenerative() => _toDropDegenerative;

    public void SetDropDegenerative(bool toDrop) => _toDropDegenerative = toDrop;

    /// <summary>True if equal elements are merged; false by default.</summary>
    public bool ToMergeElems() => _toMergeElems;

    public void SetMergeElems(bool toMerge) => _toMergeElems = toMerge;

    /// <summary>The normal of the element being pushed (its first three nodes).</summary>
    public NCollection_Vec3<float> ComputeTriNormal()
    {
        Xyz vec01 = _places[1] - _places[0];
        Xyz vec02 = _places[2] - _places[0];
        Xyz cross = vec01 ^ vec02;
        var norm = new NCollection_Vec3<float>((float)cross.X(), (float)cross.Y(), (float)cross.Z());
        return norm.Normalized();
    }

    /// <summary>Adds the triangles of a triangulation.</summary>
    public void AddTriangulation(Triangulation? tris) => AddTriangulation(tris, new Transform3D(), false);

    /// <summary>Adds the triangles of a triangulation, transformed, with their nodes in reverse order if asked.</summary>
    public virtual void AddTriangulation(Triangulation? tris, in Transform3D trsf, bool toReverse = false)
    {
        if (tris is null)
        {
            return;
        }

        if (_polyData is not null && _polyData.NodeCount == 0)
        {
            // preallocate optimistically
            _polyData.SetDoublePrecision(tris.IsDoublePrecision);
            _polyData.ResizeNodes(tris.NodeCount, false);
            _polyData.ResizeTriangles(tris.TriangleCount, false);
        }

        for (int elemIter = 1; elemIter <= tris.TriangleCount; ++elemIter)
        {
            Triangle1 elem = tris.Triangle(elemIter);
            if (toReverse)
            {
                elem = new Triangle1(elem.Value(1), elem.Value(3), elem.Value(2));
            }

            for (int triNodeIter = 0; triNodeIter < 3; ++triNodeIter)
            {
                Point3D node = tris.Node(elem.Value(triNodeIter + 1)).Transformed(trsf);
                _places[triNodeIter] = node.XYZ();
            }

            PushLastTriangle();
        }
    }

    /// <summary>The result triangulation, truncated to its size, or null if there is no output triangulation.</summary>
    public Triangulation? Result()
    {
        if (_polyData is null)
        {
            return null;
        }

        // compress data
        // OCCT: throws Standard_RangeError when no node was added (an aliased array can't be empty)
        _polyData.ResizeNodes(_nbNodes, true);
        _polyData.ResizeTriangles(_nbElems, true);
        return _polyData;
    }

    /// <summary>Adds a triangle.</summary>
    public void AddTriangle(ReadOnlySpan<Xyz> elemNodes) => AddElement(elemNodes, 3);

    /// <summary>Adds a quad.</summary>
    public void AddQuad(ReadOnlySpan<Xyz> elemNodes) => AddElement(elemNodes, 4);

    /// <summary>Adds a triangle or a quad: <paramref name="nbNodes"/> is 3 or 4.</summary>
    public void AddElement(ReadOnlySpan<Xyz> elemNodes, int nbNodes)
    {
        if (nbNodes != 3 && nbNodes != 4)
        {
            throw new ProgramErrorException("Poly_MergeNodesTool::AddElement() - Internal error");
        }

        _places[0] = elemNodes[0];
        _places[1] = elemNodes[1];
        _places[2] = elemNodes[2];
        if (nbNodes == 4)
        {
            _places[3] = elemNodes[3];
        }

        PushLastElement(nbNodes);
    }

    /// <summary>The node at a position (0..3) of the element to be pushed, for writing.</summary>
    public ref Xyz ChangeElementNode(int index) => ref _places[index];

    /// <summary>Adds a triangle or a quad whose nodes were set with <see cref="ChangeElementNode"/>.</summary>
    public void PushLastElement(int nbNodes)
    {
        if (nbNodes != 3 && nbNodes != 4)
        {
            throw new ProgramErrorException("Poly_MergeNodesTool::PushLastElement() - Internal error");
        }

        bool isOpposite = false;
        _nodeInds[3] = -1;
        if (_nodeIndexMap.HasMergeAngle || _nodeIndexMap.HasMergeTolerance)
        {
            if (!_nodeIndexMap.ToMergeAnyAngle())
            {
                _triNormal = ComputeTriNormal();
            }

            PushNodeCheck(ref isOpposite, 0);
            PushNodeCheck(ref isOpposite, 1);
            PushNodeCheck(ref isOpposite, 2);
            if (nbNodes == 4)
            {
                PushNodeCheck(ref isOpposite, 3);
            }
        }
        else
        {
            PushNodeNoMerge(0);
            PushNodeNoMerge(1);
            PushNodeNoMerge(2);
            if (nbNodes == 4)
            {
                PushNodeNoMerge(3);
            }
        }

        if (_toDropDegenerative)
        {
            // warning - removing degenerate elements may produce unused nodes
            if (_nodeInds[0] == _nodeInds[1] || _nodeInds[0] == _nodeInds[2] || _nodeInds[1] == _nodeInds[2])
            {
                if (nbNodes == 4)
                {
                    //
                }
                else
                {
                    ++_nbDegenElems;
                    return;
                }
            }
        }

        if (_toMergeElems)
        {
            NCollection_Vec4<int> sorted = _nodeInds;
            // theNbNodes is bounded by the Vec4 capacity; clamp defensively.
            int nbToSort = Min(nbNodes, 4);
            sorted.ChangeData()[..nbToSort].Sort();
            if (!_elemMap.Add(sorted))
            {
                ++_nbMergedElems;
                return;
            }
        }

        ++_nbElems;
        if (_polyData is not null)
        {
            if (_polyData.TriangleCount < _nbElems)
            {
                _polyData.ResizeTriangles(_nbElems * 2, true);
            }

            _polyData.SetTriangle(_nbElems, new Triangle1(_nodeInds[0] + 1, _nodeInds[1] + 1, _nodeInds[2] + 1));
            if (nbNodes == 4)
            {
                ++_nbElems;
                if (_polyData.TriangleCount < _nbElems)
                {
                    _polyData.ResizeTriangles(_nbElems * 2, true);
                }

                _polyData.SetTriangle(_nbElems, new Triangle1(_nodeInds[0] + 1, _nodeInds[2] + 1, _nodeInds[3] + 1));
            }
        }
    }

    /// <summary>Adds a triangle whose nodes were set with <see cref="ChangeElementNode"/>.</summary>
    public void PushLastTriangle() => PushLastElement(3);

    /// <summary>Adds a quad whose nodes were set with <see cref="ChangeElementNode"/>.</summary>
    public void PushLastQuad() => PushLastElement(4);

    /// <summary>The output node index (from 0) of a node of the last element pushed.</summary>
    public int ElementNodeIndex(int index) => _nodeInds[index];

    /// <summary>The number of nodes.</summary>
    public int NodeCount => _nbNodes;

    /// <summary>The number of elements.</summary>
    public int ElementCount => _nbElems;

    /// <summary>The number of degenerate elements dropped.</summary>
    public int DegenerativeElemCount => _nbDegenElems;

    /// <summary>The number of equal elements merged.</summary>
    public int MergedElemCount => _nbMergedElems;

    /// <summary>
    /// The output triangulation, for writing. Set to null to use the tool only as a merge map, filling an external
    /// mesh from <see cref="ElementNodeIndex"/>.
    /// </summary>
    public ref Triangulation? ChangeOutput() => ref _polyData;

    // Push triangle node with normal angle comparison.
    private void PushNodeCheck(ref bool isOpposite, int triNode)
    {
        int nodeIndex = _nbNodes;
        Xyz place = _places[triNode];
        var vec3 = new NCollection_Vec3<float>((float)place.X(), (float)place.Y(), (float)place.Z());
        if (_nodeIndexMap.Bind(ref nodeIndex, ref isOpposite, vec3, _triNormal))
        {
            ++_nbNodes;
            if (_polyData is not null)
            {
                if (_polyData.NodeCount < _nbNodes)
                {
                    _polyData.ResizeNodes(_nbNodes * 2, true);
                }

                _polyData.SetNode(_nbNodes, new Point3D(place * _unitFactor));
            }
        }

        _nodeInds[triNode] = nodeIndex;
    }

    // Push triangle node without merging vertices.
    private void PushNodeNoMerge(int triNode)
    {
        int nodeIndex = _nbNodes;
        Xyz place = _places[triNode] * _unitFactor;

        ++_nbNodes;
        if (_polyData is not null)
        {
            if (_polyData.NodeCount < _nbNodes)
            {
                _polyData.ResizeNodes(_nbNodes * 2, true);
            }

            _polyData.SetNode(_nbNodes, new Point3D(place));
        }

        _nodeInds[triNode] = nodeIndex;
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
            private readonly Vec3AndNormal _key = new(thePos, theNorm);

            public DataMapNode? Next = theNext;

            public int Value = theItem;

            public ref readonly Vec3AndNormal Key() => ref _key;
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
        private DataMapNode?[]? _data1;
        private long _nbBuckets;
        private int _size;

        private float _tolerance;       // linear tolerance for comparison
        private float _invTol;          // inversed linear tolerance for comparison
        private float _angle;           // angle for comparison
        private float _angleCos;        // angle cosine for comparison
        private bool _toMergeOpposite;  // merge nodes with opposite normals

        public MergedNodesMap(int nbBuckets)
        {
            _nbBuckets = nbBuckets;
            _tolerance = 0.0f;
            _invTol = 0.0f;
            _angle = 1.0f;
            _angleCos = 0.0f;
            _toMergeOpposite = false;
        }

        public double MergeAngle() => _angle;

        public void SetMergeAngle(double angleRad)
        {
            _angle = (float)angleRad;
            _angleCos = (float)Math.Cos(angleRad);
        }

        // 0 angle means angles should much without a tolerance.
        public bool HasMergeAngle => _angle > 0.0f;

        // TRUE if merge angle comparison can be skipped (angle is close to 90 degrees).
        public bool ToMergeAnyAngle() => _angleCos <= 0.01f;

        public bool ToMergeOpposite() => _toMergeOpposite;

        public void SetMergeOpposite(bool toMerge) => _toMergeOpposite = toMerge;

        public double MergeTolerance() => _tolerance;

        public void SetMergeTolerance(double tolerance)
        {
            _tolerance = (float)tolerance;
            _invTol = 0.0f;
            if (_tolerance > 0.0f)
            {
                _invTol = (float)(1.0 / tolerance);
            }
        }

        public bool HasMergeTolerance => _tolerance > 0.0f;

        // Bind node to the map or find existing one; TRUE if node was not bound already.
        public bool Bind(ref int index, ref bool isOpposite, in NCollection_Vec3<float> pos, in NCollection_Vec3<float> norm)
        {
            if (Resizable())
            {
                ReSize(_size);
            }

            DataMapNode?[] data = _data1!;
            long hash = this.HashCode(pos, norm, _nbBuckets);
            for (DataMapNode? nodeIter = data[hash]; nodeIter is not null; nodeIter = nodeIter.Next)
            {
                if (this.IsEqual(nodeIter.Key(), pos, norm, ref isOpposite))
                {
                    index = nodeIter.Value;
                    return false;
                }
            }

            if (_invTol > 0.0f)
            {
                NCollection_Vec3<long> indexCnt = Vec3ToCell(pos);
                for (int neigIter = 0; neigIter < 26; ++neigIter)
                {
                    NCollection_Vec3<long> anIndex = indexCnt + THE_NEIGHBRS[neigIter];
                    long hashEx = Vec3iHashCode(anIndex, _nbBuckets);
                    for (DataMapNode? nodeIter = data[hashEx]; nodeIter is not null; nodeIter = nodeIter.Next)
                    {
                        if (this.IsEqual(nodeIter.Key(), pos, norm, ref isOpposite))
                        {
                            index = nodeIter.Value;
                            return false;
                        }
                    }
                }
            }

            // theIsOpposite = false;
            data[hash] = new DataMapNode(pos, norm, index, data[hash]);
            ++_size;
            return true;
        }

        // ReSize the map (NCollection_BaseMap::BeginResize, the rehash, EndResize).
        public void ReSize(int size)
        {
            long extent = size < 0 ? 0 : size;
            long nbNewBuck = Primes.NextPrimeForMap(extent);
            if (nbNewBuck <= _nbBuckets)
            {
                if (_data1 is not null)
                {
                    return;
                }

                nbNewBuck = _nbBuckets;
            }

            var newData = new DataMapNode?[nbNewBuck + 1];
            if (_data1 is DataMapNode?[] oldData)
            {
                for (long oldBuckIter = 0; oldBuckIter <= _nbBuckets; ++oldBuckIter)
                {
                    for (DataMapNode? oldNodeIter = oldData[oldBuckIter]; oldNodeIter is not null;)
                    {
                        long newHash = this.HashCode(oldNodeIter.Key().Pos, oldNodeIter.Key().Norm, nbNewBuck);
                        DataMapNode? nextNode = oldNodeIter.Next;
                        oldNodeIter.Next = newData[newHash];
                        newData[newHash] = oldNodeIter;
                        oldNodeIter = nextNode;
                    }
                }
            }

            _nbBuckets = nbNewBuck;
            _data1 = newData;
        }

        private bool Resizable() => _size == 0 || (_size > _nbBuckets);

        // Return cell index for specified 3D point and inverted cell size.
        private NCollection_Vec3<long> Vec3ToCell(in NCollection_Vec3<float> pnt) => NCollection_Vec3<long>.From(pnt * _invTol);

        // Hash code for integer vec3 (copied from NCollection_CellFilter).
        private static long Vec3iHashCode(in NCollection_Vec3<long> vec, long upper)
        {
            const int shiftBits = (8 * sizeof(long) - 1) / 3;
            ulong hashCode = 0;
            hashCode = (hashCode << shiftBits) ^ (ulong)vec[0];
            hashCode = (hashCode << shiftBits) ^ (ulong)vec[1];
            hashCode = (hashCode << shiftBits) ^ (ulong)vec[2];
            return (long)(hashCode % (ulong)upper) + 1;
        }

        // Compute hash code.
        private long HashCode(in NCollection_Vec3<float> pos, in NCollection_Vec3<float> norm, long upper)
        {
            _ = norm;
            if (_invTol <= 0.0f)
            {
                // compute DJB2 hash of a string
                // OCCT hashes the bytes as char: signed on MSVC (followed here), unsigned on ARM/GCC
                ReadOnlySpan<byte> bytes = MemoryMarshal.AsBytes(new ReadOnlySpan<NCollection_Vec3<float>>(in pos));
                uint hashCode = 0;
                foreach (byte c in bytes)
                {
                    hashCode = ((hashCode << 5) + hashCode) ^ (uint)(sbyte)c;
                }

                return (long)(hashCode % (ulong)upper) + 1;
            }

            NCollection_Vec3<long> index = Vec3ToCell(pos);
            return Vec3iHashCode(index, upper);
        }

        // Compare two vectors with inversed tolerance.
        private bool Vec3AreEqual(in NCollection_Vec3<float> key1, in NCollection_Vec3<float> key2)
        {
            if (_invTol <= 0.0f)
            {
                return key1.IsEqual(key2);
            }

            // tolerance should be smaller than triangle size to avoid artifacts
            // const CellVec3i anIndex1 = vec3ToCell (theKey1);
            // const CellVec3i anIndex2 = vec3ToCell (theKey2);
            // return anIndex1.IsEqual (anIndex2);
            if (Math.Abs(key1.X() - key2.X()) > _tolerance)
            {
                return false;
            }

            if (Math.Abs(key1.Y() - key2.Y()) > _tolerance)
            {
                return false;
            }

            if (Math.Abs(key1.Z() - key2.Z()) > _tolerance)
            {
                return false;
            }

            return true;
        }

        // Compare two nodes.
        private bool IsEqual(in Vec3AndNormal key1, in NCollection_Vec3<float> pos2, in NCollection_Vec3<float> norm2, ref bool isOpposite)
        {
            if (!Vec3AreEqual(key1.Pos, pos2))
            {
                return false;
            }

            float cosinus = key1.Norm.Dot(norm2);
            if (cosinus >= _angleCos)
            {
                // theIsOpposite = false;
                return true;
            }
            else if (_toMergeOpposite && cosinus <= -_angleCos)
            {
                isOpposite = true;
                return true;
            }

            return false;
        }
    }

    private Triangulation? _polyData;                         // output triangulation
    private readonly MergedNodesMap _nodeIndexMap;                  // map of merged nodes
    private readonly HashSet<NCollection_Vec4<int>> _elemMap; // map of elements
    private NCollection_Vec4<int> _nodeInds;                        // current element indexes
    private NCollection_Vec3<float> _triNormal;                     // current triangle normal
    private readonly Xyz[] _places = new Xyz[4];              // current triangle/quad coordinates to push
    private double _unitFactor;                                     // scale factor to apply
    private int _nbNodes;                                           // number of output nodes
    private int _nbElems;                                           // number of output elements
    private int _nbDegenElems;                                      // number of degenerated elements
    private int _nbMergedElems;                                     // number of merged elements
    private bool _toDropDegenerative;                               // flag to filter our degenerate elements
    private bool _toMergeElems;                                     // flag to merge elements
}
