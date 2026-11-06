// Port of: src/FoundationClasses/TKMath/Poly/Poly_Triangulation.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_Triangulation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Kerf.Engine;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Kerf.Geometry.TransformForm;
using static Kerf.Engine.Meshing.MeshPurpose;

namespace Kerf.Engine.Meshing;

/// <summary>
/// A triangulation of a surface, a set of surfaces or a shape: a table of 3D nodes and a table of triangles
/// (<see cref="Triangle1"/>, triplets of 1-based node indices), with optional tables, parallel to the nodes, of
/// UV parameters on the surface and of normals, and an optional deflection (the largest distance from the surface
/// to the triangulation).
/// </summary>
/// <remarks>
/// <para>
/// The storage follows OCCT: nodes in a <see cref="ArrayOfNodes"/> (double precision by default, single precision
/// after <see cref="SetDoublePrecision"/>(false)), UV nodes in a <see cref="ArrayOfUVNodes"/>, triangles in an
/// <see cref="NCollection_Array1{T}"/> from 1, normals as single-precision vectors in an array from 0. Whole tables can
/// be read without copying through <see cref="NodeCoordinates"/>, <see cref="TriangleIndices"/>,
/// <see cref="UVNodeCoordinates"/> and <see cref="NormalComponents"/>, or copied in either precision with the
/// <c>Copy...To</c> methods.
/// </para>
/// <para>
/// A triangulation is a mutable mesh container, filled by the mesher: its mutators are public, as in OCCT. Once
/// triangulations are attached to shapes they are expected to be frozen like other geometry.
/// </para>
/// <para>
/// Left out: the deferred-loading interface's <c>OSD_FileSystem</c> argument (the file system isn't ported; the
/// methods keep their behaviour without it), <c>DumpJson</c>.
/// </para>
/// </remarks>
internal class Triangulation : Transient
{
    // OCCT keeps the cached box behind an atomic pointer guarded by a shared mutex; here each box is published whole,
    // never changed once published, so readers and writers need no lock (unit 10I)
    private StrongBox<Box>? _cachedMinMax;

    protected double _deflection;
    protected ArrayOfNodes _nodes;
    protected NCollection_Array1<Triangle1> _triangles;
    protected ArrayOfUVNodes _uvNodes;
    protected NCollection_Array1<NCollection_Vec3<float>> _normals;
    protected MeshPurpose _purpose;

    protected TriangulationParameters? _params;

    /// <summary>An empty triangulation.</summary>
    public Triangulation()
    {
        _deflection = 0;
        _nodes = new ArrayOfNodes();
        _triangles = new NCollection_Array1<Triangle1>();
        _uvNodes = new ArrayOfUVNodes();
        _normals = new NCollection_Array1<NCollection_Vec3<float>>();
        _purpose = MeshPurpose_NONE;
    }

    /// <summary>
    /// A triangulation with room for <paramref name="nbNodes"/> nodes (at the origin) and
    /// <paramref name="nbTriangles"/> triangles (all indices 0), and optionally UV nodes and normals.
    /// Throws <see cref="RangeErrorException"/> if <paramref name="nbNodes"/> isn't positive, as OCCT does.
    /// </summary>
    public Triangulation(int nbNodes, int nbTriangles, bool hasUVNodes, bool hasNormals = false)
    {
        _deflection = 0;
        _nodes = new ArrayOfNodes(nbNodes);
        _triangles = new NCollection_Array1<Triangle1>(1, nbTriangles);
        _uvNodes = new ArrayOfUVNodes();
        _normals = new NCollection_Array1<NCollection_Vec3<float>>();
        _purpose = MeshPurpose_NONE;
        if (hasUVNodes)
        {
            _uvNodes.Resize(nbNodes, false);
        }

        if (hasNormals)
        {
            _normals.Resize(0, nbNodes - 1, false);
        }
    }

    /// <summary>A triangulation of copies of the nodes and the triangles.</summary>
    /// <remarks>As in OCCT, the triangle table takes the bounds of <paramref name="triangles"/>, which should start at 1.</remarks>
    public Triangulation(NCollection_Array1<Point3D> nodes, NCollection_Array1<Triangle1> triangles)
    {
        _deflection = 0;
        _nodes = new ArrayOfNodes(nodes.Length());
        _triangles = new NCollection_Array1<Triangle1>(1, triangles.Length());
        _uvNodes = new ArrayOfUVNodes();
        _normals = new NCollection_Array1<NCollection_Vec3<float>>();
        _purpose = MeshPurpose_NONE;

        (Point3D[] nodeData, int nodeOffset) = nodes.Storage();
        var nodeWrapper = new ArrayOfNodes(nodeData, nodeOffset, nodes.Length());
        _nodes.Assign(nodeWrapper);
        _triangles.Assign(triangles); // OCCT: the triangles' bounds are taken too
    }

    /// <summary>
    /// A triangulation of copies of the nodes, their UV parameters (one per node) and the triangles.
    /// </summary>
    /// <remarks>As in OCCT, the triangle table takes the bounds of <paramref name="triangles"/>, which should start at 1.</remarks>
    public Triangulation(NCollection_Array1<Point3D> nodes, NCollection_Array1<Point2D> uvNodes, NCollection_Array1<Triangle1> triangles)
    {
        _deflection = 0;
        _nodes = new ArrayOfNodes(nodes.Length());
        _triangles = new NCollection_Array1<Triangle1>(1, triangles.Length());
        _uvNodes = new ArrayOfUVNodes(nodes.Length());
        _normals = new NCollection_Array1<NCollection_Vec3<float>>();
        _purpose = MeshPurpose_NONE;

        (Point3D[] nodeData, int nodeOffset) = nodes.Storage();
        var nodeWrapper = new ArrayOfNodes(nodeData, nodeOffset, nodes.Length());
        _nodes.Assign(nodeWrapper);
        _triangles.Assign(triangles); // OCCT: the triangles' bounds are taken too
        (Point2D[] uvData, int uvOffset) = uvNodes.Storage();
        var uvNodeWrapper = new ArrayOfUVNodes(uvData, uvOffset, uvNodes.Length());
        _uvNodes.Assign(uvNodeWrapper);
    }

    /// <summary>A full copy of a triangulation (its parameters copied too).</summary>
    public Triangulation(Triangulation triangulation)
    {
        _deflection = triangulation._deflection;
        _nodes = new ArrayOfNodes(triangulation._nodes);
        _triangles = new NCollection_Array1<Triangle1>(triangulation._triangles);
        _uvNodes = new ArrayOfUVNodes(triangulation._uvNodes);
        _normals = new NCollection_Array1<NCollection_Vec3<float>>(triangulation._normals);
        _purpose = triangulation._purpose;
        _params = triangulation._params?.Copy();
        if (triangulation.HasCachedMinMax)
        {
            SetCachedMinMax(triangulation.CachedMinMax());
        }
    }

    /// <summary>A full copy of this triangulation.</summary>
    public virtual Triangulation Copy() => new(this);

    /// <summary>The deflection of this triangulation.</summary>
    public double Deflection() => _deflection;

    /// <summary>Sets the deflection of this triangulation.</summary>
    public void Deflection(double deflection)
    {
        CheckMutable();
        _deflection = deflection;
    }

    /// <summary>The parameters this triangulation was generated with, or null.</summary>
    public TriangulationParameters? Parameters() => _params;

    /// <summary>Sets the parameters this triangulation was generated with.</summary>
    public void Parameters(TriangulationParameters? theParams)
    {
        CheckMutable();
        _params = theParams;
    }

    /// <summary>Clears the nodes, the triangles and all the attributes.</summary>
    public virtual void Clear()
    {
        CheckMutable();
        if (!_nodes.IsEmpty())
        {
            var emptyNodes = new ArrayOfNodes();
            emptyNodes.SetDoublePrecision(_nodes.IsDoublePrecision);
            _nodes.Move(emptyNodes);
        }

        if (!_triangles.IsEmpty())
        {
            var emptyTriangles = new NCollection_Array1<Triangle1>();
            _triangles.Move(emptyTriangles);
        }

        RemoveUVNodes();
        RemoveNormals();
    }

    /// <summary>True if the triangulation has nodes and triangles.</summary>
    public virtual bool HasGeometry => !_nodes.IsEmpty() && !_triangles.IsEmpty();

    public int NodeCount => _nodes.Length();

    public int TriangleCount => _triangles.Length();

    /// <summary>True if UV nodes are defined.</summary>
    public bool HasUVNodes => !_uvNodes.IsEmpty();

    /// <summary>True if nodal normals are defined.</summary>
    public bool HasNormals => !_normals.IsEmpty();

    /// <summary>The node at an index in [1, NbNodes()].</summary>
    public Point3D Node(int index) => _nodes.Value(index - 1);

    /// <summary>Sets the node at an index in [1, NbNodes()].</summary>
    public void SetNode(int index, in Point3D pnt)
    {
        CheckMutable();
        _nodes.SetValue(index - 1, pnt);
    }

    /// <summary>The UV node at an index in [1, NbNodes()].</summary>
    public Point2D UVNode(int index) => _uvNodes.Value(index - 1);

    /// <summary>Sets the UV node at an index in [1, NbNodes()].</summary>
    public void SetUVNode(int index, in Point2D pnt)
    {
        CheckMutable();
        _uvNodes.SetValue(index - 1, pnt);
    }

    /// <summary>The triangle at an index in [1, NbTriangles()].</summary>
    public Triangle1 Triangle(int index) => _triangles.Value(index);

    /// <summary>Sets the triangle at an index in [1, NbTriangles()].</summary>
    public void SetTriangle(int index, in Triangle1 triangle)
    {
        CheckMutable();
        _triangles.SetValue(index, triangle);
    }

    /// <summary>The normal at a node index in [1, NbNodes()].</summary>
    public Direction3D Normal(int index)
    {
        NCollection_Vec3<float> norm = _normals.Value(index - 1);
        return new Direction3D(norm.X(), norm.Y(), norm.Z());
    }

    /// <summary>The normal at a node index in [1, NbNodes()], as stored (single precision).</summary>
    public void Normal(int index, out NCollection_Vec3<float> vec3) => vec3 = _normals.Value(index - 1);

    /// <summary>Sets the normal (a unit vector) at a node index in [1, NbNodes()].</summary>
    public void SetNormal(int index, in NCollection_Vec3<float> normal)
    {
        CheckMutable();
        _normals.SetValue(index - 1, normal);
    }

    /// <summary>Sets the normal at a node index in [1, NbNodes()], rounded to single precision.</summary>
    public void SetNormal(int index, in Direction3D normal)
        => SetNormal(index, new NCollection_Vec3<float>((float)normal.X, (float)normal.Y, (float)normal.Z));

    /// <summary>The mesh purpose bits.</summary>
    public MeshPurpose MeshPurpose() => _purpose;

    /// <summary>Sets the mesh purpose bits.</summary>
    public void SetMeshPurpose(MeshPurpose purpose)
    {
        CheckMutable();
        _purpose = purpose;
    }

    /// <summary>The cached range of the nodes, void if there is none.</summary>
    public Box CachedMinMax()
    {
        StrongBox<Box>? box = Volatile.Read(ref _cachedMinMax);
        return box is null ? new Box() : box.Value;
    }

    /// <summary>
    /// Sets the cached range of the nodes, which must match them exactly, without a gap or transformation. A void box
    /// clears the cache.
    /// </summary>
    public void SetCachedMinMax(in Box box)
    {
        if (box.IsVoid)
        {
            UnsetCachedMinMax();
            return;
        }

        Volatile.Write(ref _cachedMinMax, new StrongBox<Box>(box));
    }

    /// <summary>True if a range of the nodes is cached.</summary>
    public bool HasCachedMinMax => Volatile.Read(ref _cachedMinMax) is not null;

    /// <summary>Caches the range of the nodes.</summary>
    public void UpdateCachedMinMax()
    {
        var box = new Box();
        MinMax(ref box, new Transform3D(), true);
        SetCachedMinMax(box);
    }

    /// <summary>
    /// Extends <paramref name="box"/> by the range of this triangulation, transformed. The cached range is used when
    /// there is one and the transformation has no rotation, or <paramref name="isAccurate"/> is false, or there are
    /// no nodes. False if there is nothing to add (no nodes and no cached range).
    /// </summary>
    public bool MinMax(ref Box box, in Transform3D trsf, bool isAccurate = false)
    {
        var aBox = new Box();
        bool usedCache = false;
        StrongBox<Box>? cachedBox = Volatile.Read(ref _cachedMinMax);
        if (cachedBox is not null
            && (!HasGeometry || !isAccurate || trsf.Form == Identity
                || trsf.Form == Translation || trsf.Form == PointMirror
                || trsf.Form == Scale))
        {
            aBox = cachedBox.Value.Transformed(trsf);
            usedCache = true;
        }

        if (!usedCache)
        {
            aBox = ComputeBoundingBox(trsf);
        }

        if (aBox.IsVoid)
        {
            return false;
        }

        box.Add(aBox);
        return true;
    }

    /// <summary>True if the nodes are in double precision (the default).</summary>
    public bool IsDoublePrecision => _nodes.IsDoublePrecision;

    /// <summary>
    /// Sets whether the 3D and UV nodes are stored in double or single precision. Throws if they are already allocated.
    /// </summary>
    public void SetDoublePrecision(bool isDouble)
    {
        CheckMutable();
        _nodes.SetDoublePrecision(isDouble);
        _uvNodes.SetDoublePrecision(isDouble);
    }

    /// <summary>Resizes the nodes and the UV nodes and normals with them, keeping the old values if asked.</summary>
    public void ResizeNodes(int nbNodes, bool toCopyOld)
    {
        CheckMutable();
        _nodes.Resize(nbNodes, toCopyOld);
        if (!_uvNodes.IsEmpty())
        {
            _uvNodes.Resize(nbNodes, toCopyOld);
        }

        if (!_normals.IsEmpty())
        {
            _normals.Resize(0, nbNodes - 1, toCopyOld);
        }
    }

    /// <summary>Resizes the triangles, keeping the old ones if asked.</summary>
    public void ResizeTriangles(int nbTriangles, bool toCopyOld)
    {
        CheckMutable();
        _triangles.Resize(1, nbTriangles, toCopyOld);
    }

    /// <summary>Allocates the UV nodes, if not yet allocated for every node.</summary>
    public void AddUVNodes()
    {
        CheckMutable();
        if (_uvNodes.IsEmpty() || _uvNodes.Size() != _nodes.Size())
        {
            _uvNodes.Resize(_nodes.Size(), false);
        }
    }

    /// <summary>Deallocates the UV nodes.</summary>
    public void RemoveUVNodes()
    {
        CheckMutable();
        if (!_uvNodes.IsEmpty())
        {
            var empty = new ArrayOfUVNodes();
            empty.SetDoublePrecision(_uvNodes.IsDoublePrecision);
            _uvNodes.Move(empty);
        }
    }

    /// <summary>Allocates the normals, if not yet allocated for every node.</summary>
    public void AddNormals()
    {
        CheckMutable();
        if (_normals.IsEmpty() || _normals.Length() != _nodes.Length())
        {
            _normals.Resize(0, _nodes.Size() - 1, false);
        }
    }

    /// <summary>Deallocates the normals.</summary>
    public void RemoveNormals()
    {
        CheckMutable();
        if (!_normals.IsEmpty())
        {
            var empty = new NCollection_Array1<NCollection_Vec3<float>>();
            _normals.Move(empty);
        }
    }

    /// <summary>Computes smooth normals at the nodes, averaging the normals of the triangles (weighted by area).</summary>
    public void ComputeNormals()
    {
        CheckMutable();
        // zero values
        AddNormals();
        _normals.Init(new NCollection_Vec3<float>(0.0f));

        Span<int> elem = [0, 0, 0];
        foreach (ref readonly Triangle1 tri in _triangles.AsSpan())
        {
            tri.Get(out elem[0], out elem[1], out elem[2]);
            Point3D node0 = _nodes.Value(elem[0] - 1);
            Point3D node1 = _nodes.Value(elem[1] - 1);
            Point3D node2 = _nodes.Value(elem[2] - 1);

            Xyz vec01 = node1.XYZ() - node0.XYZ();
            Xyz vec02 = node2.XYZ() - node0.XYZ();
            Xyz triNorm = vec01 ^ vec02;
            var norm3f = new NCollection_Vec3<float>((float)triNorm.X(), (float)triNorm.Y(), (float)triNorm.Z());
            for (int nodeIter = 0; nodeIter < 3; ++nodeIter)
            {
                _normals.ChangeValue(elem[nodeIter] - 1).Add(norm3f);
            }
        }

        // Normalize all vectors
        foreach (ref NCollection_Vec3<float> norm3f in _normals.AsSpan())
        {
            float mod = norm3f.Modulus();
            norm3f = mod == 0.0f ? new NCollection_Vec3<float>(0.0f, 0.0f, 1.0f) : (norm3f / mod);
        }
    }

    /// <summary>
    /// The nodes as a 1-based array, or null if there are none. In double precision the array shares the nodes'
    /// storage (OCCT: not to be used after the triangulation changes); in single precision it is a copy.
    /// </summary>
    public NCollection_HArray1<Point3D>? MapNodeArray()
    {
        if (_nodes.IsEmpty())
        {
            return null;
        }

        if (_nodes.IsDoublePrecision && _nodes.Storage() is (Point3D[] pntArr, int byteOffset))
        {
            // wrap array
            var hArray = new NCollection_HArray1<Point3D>();
            var array = new NCollection_Array1<Point3D>(pntArr, byteOffset / Unsafe.SizeOf<Point3D>(), 1, NodeCount);
            hArray.Move(array);
            return hArray;
        }

        // deep copy
        var copy = new NCollection_HArray1<Point3D>(1, NodeCount);
        for (int nodeIter = 0; nodeIter < NodeCount; ++nodeIter)
        {
            Point3D pnt = _nodes.Value(nodeIter);
            copy.SetValue(nodeIter + 1, pnt);
        }

        return copy;
    }

    /// <summary>The triangles as a 1-based array sharing their storage, or null if there are none.</summary>
    public NCollection_HArray1<Triangle1>? MapTriangleArray()
    {
        if (_triangles.IsEmpty())
        {
            return null;
        }

        var hArray = new NCollection_HArray1<Triangle1>();
        var array = new NCollection_Array1<Triangle1>(_triangles, _triangles.Lower(), 1, TriangleCount);
        hArray.Move(array);
        return hArray;
    }

    /// <summary>
    /// The UV nodes as a 1-based array, or null if there are none. In double precision the array shares the nodes'
    /// storage; in single precision it is a copy.
    /// </summary>
    public NCollection_HArray1<Point2D>? MapUVNodeArray()
    {
        if (_uvNodes.IsEmpty())
        {
            return null;
        }

        if (_uvNodes.IsDoublePrecision && _uvNodes.Storage() is (Point2D[] pntArr, int byteOffset))
        {
            // wrap array
            var hArray = new NCollection_HArray1<Point2D>();
            var array = new NCollection_Array1<Point2D>(pntArr, byteOffset / Unsafe.SizeOf<Point2D>(), 1, NodeCount);
            hArray.Move(array);
            return hArray;
        }

        // deep copy
        var copy = new NCollection_HArray1<Point2D>(1, NodeCount);
        for (int nodeIter = 0; nodeIter < NodeCount; ++nodeIter)
        {
            Point2D pnt = _uvNodes.Value(nodeIter);
            copy.SetValue(nodeIter + 1, pnt);
        }

        return copy;
    }

    /// <summary>
    /// The normals as a 1-based array of 3 × NbNodes() floats, or null if there are none. OCCT's array shares the
    /// normals' storage; the port's is a copy, since a .NET float array can't alias an array of vectors.
    /// </summary>
    public NCollection_HArray1<float>? MapNormalArray()
    {
        if (_normals.IsEmpty())
        {
            return null;
        }

        var hArray = new NCollection_HArray1<float>(1, 3 * NodeCount);
        NormalComponents().CopyTo(hArray.AsSpan());
        return hArray;
    }

    // Bulk access (Kerf additions)

    /// <summary>
    /// The node coordinates (x, y, z for each node), without copying, when the nodes are in double precision; throws
    /// <see cref="TypeMismatchException"/> in single precision (use <see cref="NodeCoordinatesF"/> or
    /// <see cref="CopyNodeCoordinatesTo(Span{double})"/>). Valid until the nodes are resized.
    /// </summary>
    public ReadOnlySpan<double> NodeCoordinates() => MemoryMarshal.Cast<Point3D, double>(_nodes.AsSpan<Point3D>());

    /// <summary>
    /// The node coordinates (x, y, z for each node), without copying, when the nodes are in single precision; throws
    /// <see cref="TypeMismatchException"/> in double precision. Valid until the nodes are resized.
    /// </summary>
    public ReadOnlySpan<float> NodeCoordinatesF() => MemoryMarshal.Cast<NCollection_Vec3<float>, float>(_nodes.AsSpan<NCollection_Vec3<float>>());

    /// <summary>Copies the node coordinates (x, y, z for each node) in double precision, whatever the storage.</summary>
    public void CopyNodeCoordinatesTo(Span<double> destination)
    {
        if (_nodes.IsDoublePrecision)
        {
            NodeCoordinates().CopyTo(destination);
            return;
        }

        ReadOnlySpan<float> source = NodeCoordinatesF();
        Standard_DimensionMismatch_Raise_if(destination.Length < source.Length, "Poly_Triangulation::CopyNodeCoordinatesTo");
        for (int i = 0; i < source.Length; i++)
        {
            destination[i] = source[i];
        }
    }

    /// <summary>Copies the node coordinates (x, y, z for each node) in single precision, whatever the storage.</summary>
    public void CopyNodeCoordinatesTo(Span<float> destination)
    {
        if (!_nodes.IsDoublePrecision)
        {
            NodeCoordinatesF().CopyTo(destination);
            return;
        }

        ReadOnlySpan<double> source = NodeCoordinates();
        Standard_DimensionMismatch_Raise_if(destination.Length < source.Length, "Poly_Triangulation::CopyNodeCoordinatesTo");
        for (int i = 0; i < source.Length; i++)
        {
            destination[i] = (float)source[i];
        }
    }

    /// <summary>
    /// The triangles' node indices (three 1-based indices for each triangle), without copying. Valid until the
    /// triangles are resized.
    /// </summary>
    public ReadOnlySpan<int> TriangleIndices() => MemoryMarshal.Cast<Triangle1, int>(_triangles.AsSpan());

    /// <summary>The triangles, without copying. Valid until they are resized.</summary>
    public ReadOnlySpan<Triangle1> TriangleSpan() => _triangles.AsSpan();

    /// <summary>
    /// The UV coordinates (u, v for each node), without copying, when they are in double precision; throws
    /// <see cref="TypeMismatchException"/> in single precision (use <see cref="UVNodeCoordinatesF"/>). Empty if there
    /// are no UV nodes.
    /// </summary>
    public ReadOnlySpan<double> UVNodeCoordinates() => MemoryMarshal.Cast<Point2D, double>(_uvNodes.AsSpan<Point2D>());

    /// <summary>The UV coordinates (u, v for each node), without copying, when they are in single precision.</summary>
    public ReadOnlySpan<float> UVNodeCoordinatesF() => MemoryMarshal.Cast<NCollection_Vec2<float>, float>(_uvNodes.AsSpan<NCollection_Vec2<float>>());

    /// <summary>The normals (x, y, z for each node), without copying. Empty if there are no normals.</summary>
    public ReadOnlySpan<float> NormalComponents() => MemoryMarshal.Cast<NCollection_Vec3<float>, float>(_normals.AsSpan());

    // Internal arrays: for the mesher and other ported code

    /// <summary>The internal array of triangles; <see cref="Triangle"/> and <see cref="SetTriangle"/> are preferred.</summary>
    public NCollection_Array1<Triangle1> InternalTriangles() => _triangles;

    /// <summary>The internal array of nodes; <see cref="Node"/> and <see cref="SetNode"/> are preferred.</summary>
    public ArrayOfNodes InternalNodes() => _nodes;

    /// <summary>The internal array of UV nodes; <see cref="UVNode"/> and <see cref="SetUVNode"/> are preferred.</summary>
    public ArrayOfUVNodes InternalUVNodes() => _uvNodes;

    /// <summary>The internal array of normals; <see cref="Normal(int)"/> and <see cref="SetNormal(int, in Direction3D)"/> are preferred.</summary>
    public NCollection_Array1<NCollection_Vec3<float>> InternalNormals() => _normals;
    // Deferred data (OCCT's OSD_FileSystem argument is left out: the file system isn't ported)

    /// <summary>The estimated number of nodes that <see cref="LoadDeferredData()"/> can load.</summary>
    public virtual int DeferredNodeCount => 0;

    /// <summary>The estimated number of triangles that <see cref="LoadDeferredData()"/> can load.</summary>
    public virtual int DeferredTriangleCount => 0;

    /// <summary>True if there is triangulation data that <see cref="LoadDeferredData()"/> can load.</summary>
    public virtual bool HasDeferredData => DeferredTriangleCount > 0;

    /// <summary>Loads the deferred triangulation data into this triangulation.</summary>
    public virtual bool LoadDeferredData()
    {
        CheckMutable();
        if (!HasDeferredData)
        {
            return false;
        }

        if (!this.LoadDeferredData(this))
        {
            return false;
        }

        SetMeshPurpose(_purpose | MeshPurpose_Loaded);
        return true;
    }

    /// <summary>Loads the deferred triangulation data into a new triangulation, or returns null.</summary>
    public virtual Triangulation? DetachedLoadDeferredData()
    {
        if (!HasDeferredData)
        {
            return null;
        }

        Triangulation result = CreateNewEntity();
        if (!this.LoadDeferredData(result))
        {
            return null;
        }

        result.SetMeshPurpose(result.MeshPurpose() | MeshPurpose_Loaded);
        return result;
    }

    /// <summary>Releases the triangulation data if it has deferred storage.</summary>
    public virtual bool UnloadDeferredData()
    {
        CheckMutable();
        if (HasDeferredData)
        {
            Clear();
            SetMeshPurpose(_purpose & ~MeshPurpose_Loaded);
            return true;
        }

        return false;
    }

    /// <summary>Creates a new, empty triangulation of the same kind (a subclass may return its own type).</summary>
    protected virtual Triangulation CreateNewEntity() => new();

    /// <summary>Loads the deferred data into <paramref name="destTriangulation"/>; false by default.</summary>
    protected virtual bool LoadDeferredData(Triangulation destTriangulation) => false;

    /// <summary>Clears the cached range.</summary>
    protected void UnsetCachedMinMax()
    {
        Volatile.Write(ref _cachedMinMax, null);
    }

    /// <summary>The bounding box of the nodes, transformed.</summary>
    protected virtual Box ComputeBoundingBox(in Transform3D trsf)
    {
        var box = new Box();
        if (trsf.Form == Identity)
        {
            for (int nodeIdx = 0; nodeIdx < NodeCount; nodeIdx++)
            {
                box.Add(_nodes.Value(nodeIdx));
            }
        }
        else
        {
            for (int nodeIdx = 0; nodeIdx < NodeCount; nodeIdx++)
            {
                box.Add(_nodes.Value(nodeIdx).Transformed(trsf));
            }
        }

        return box;
    }

    public override string ToString()
        => string.Create(CultureInfo.InvariantCulture,
                         $"Poly_Triangulation({NodeCount} nodes, {TriangleCount} triangles, deflection {_deflection:R})");
}
