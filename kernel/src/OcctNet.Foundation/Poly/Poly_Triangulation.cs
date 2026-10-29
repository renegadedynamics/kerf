// Port of: src/FoundationClasses/TKMath/Poly/Poly_Triangulation.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_Triangulation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using static Occt.gp_TrsfForm;
using static Occt.Poly_MeshPurpose;

namespace Occt;

/// <summary>
/// A triangulation of a surface, a set of surfaces or a shape: a table of 3D nodes and a table of triangles
/// (<see cref="Poly_Triangle"/>, triplets of 1-based node indices), with optional tables, parallel to the nodes, of
/// UV parameters on the surface and of normals, and an optional deflection (the largest distance from the surface
/// to the triangulation).
/// </summary>
/// <remarks>
/// <para>
/// The storage follows OCCT: nodes in a <see cref="Poly_ArrayOfNodes"/> (double precision by default, single precision
/// after <see cref="SetDoublePrecision"/>(false)), UV nodes in a <see cref="Poly_ArrayOfUVNodes"/>, triangles in an
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
public class Poly_Triangulation : Standard_Transient
{
    // OCCT keeps the cached box behind an atomic pointer guarded by a shared mutex
    private readonly Lock myCachedMinMaxMutex = new();
    private StrongBox<Bnd_Box>? myCachedMinMax;

    protected double myDeflection;
    protected Poly_ArrayOfNodes myNodes;
    protected NCollection_Array1<Poly_Triangle> myTriangles;
    protected Poly_ArrayOfUVNodes myUVNodes;
    protected NCollection_Array1<NCollection_Vec3<float>> myNormals;
    protected Poly_MeshPurpose myPurpose;

    protected Poly_TriangulationParameters? myParams;

    /// <summary>An empty triangulation.</summary>
    public Poly_Triangulation()
    {
        myDeflection = 0;
        myNodes = new Poly_ArrayOfNodes();
        myTriangles = new NCollection_Array1<Poly_Triangle>();
        myUVNodes = new Poly_ArrayOfUVNodes();
        myNormals = new NCollection_Array1<NCollection_Vec3<float>>();
        myPurpose = Poly_MeshPurpose_NONE;
    }

    /// <summary>
    /// A triangulation with room for <paramref name="theNbNodes"/> nodes (at the origin) and
    /// <paramref name="theNbTriangles"/> triangles (all indices 0), and optionally UV nodes and normals.
    /// Throws <see cref="Standard_RangeError"/> if <paramref name="theNbNodes"/> isn't positive, as OCCT does.
    /// </summary>
    public Poly_Triangulation(int theNbNodes, int theNbTriangles, bool theHasUVNodes, bool theHasNormals = false)
    {
        myDeflection = 0;
        myNodes = new Poly_ArrayOfNodes(theNbNodes);
        myTriangles = new NCollection_Array1<Poly_Triangle>(1, theNbTriangles);
        myUVNodes = new Poly_ArrayOfUVNodes();
        myNormals = new NCollection_Array1<NCollection_Vec3<float>>();
        myPurpose = Poly_MeshPurpose_NONE;
        if (theHasUVNodes)
        {
            myUVNodes.Resize(theNbNodes, false);
        }

        if (theHasNormals)
        {
            myNormals.Resize(0, theNbNodes - 1, false);
        }
    }

    /// <summary>A triangulation of copies of the nodes and the triangles.</summary>
    /// <remarks>As in OCCT, the triangle table takes the bounds of <paramref name="theTriangles"/>, which should start at 1.</remarks>
    public Poly_Triangulation(NCollection_Array1<gp_Pnt> theNodes, NCollection_Array1<Poly_Triangle> theTriangles)
    {
        myDeflection = 0;
        myNodes = new Poly_ArrayOfNodes(theNodes.Length());
        myTriangles = new NCollection_Array1<Poly_Triangle>(1, theTriangles.Length());
        myUVNodes = new Poly_ArrayOfUVNodes();
        myNormals = new NCollection_Array1<NCollection_Vec3<float>>();
        myPurpose = Poly_MeshPurpose_NONE;

        (gp_Pnt[] aNodeData, int aNodeOffset) = theNodes.Storage();
        var aNodeWrapper = new Poly_ArrayOfNodes(aNodeData, aNodeOffset, theNodes.Length());
        myNodes.Assign(aNodeWrapper);
        myTriangles.Assign(theTriangles); // OCCT: the triangles' bounds are taken too
    }

    /// <summary>
    /// A triangulation of copies of the nodes, their UV parameters (one per node) and the triangles.
    /// </summary>
    /// <remarks>As in OCCT, the triangle table takes the bounds of <paramref name="theTriangles"/>, which should start at 1.</remarks>
    public Poly_Triangulation(NCollection_Array1<gp_Pnt> theNodes, NCollection_Array1<gp_Pnt2d> theUVNodes, NCollection_Array1<Poly_Triangle> theTriangles)
    {
        myDeflection = 0;
        myNodes = new Poly_ArrayOfNodes(theNodes.Length());
        myTriangles = new NCollection_Array1<Poly_Triangle>(1, theTriangles.Length());
        myUVNodes = new Poly_ArrayOfUVNodes(theNodes.Length());
        myNormals = new NCollection_Array1<NCollection_Vec3<float>>();
        myPurpose = Poly_MeshPurpose_NONE;

        (gp_Pnt[] aNodeData, int aNodeOffset) = theNodes.Storage();
        var aNodeWrapper = new Poly_ArrayOfNodes(aNodeData, aNodeOffset, theNodes.Length());
        myNodes.Assign(aNodeWrapper);
        myTriangles.Assign(theTriangles); // OCCT: the triangles' bounds are taken too
        (gp_Pnt2d[] anUVData, int anUVOffset) = theUVNodes.Storage();
        var aUVNodeWrapper = new Poly_ArrayOfUVNodes(anUVData, anUVOffset, theUVNodes.Length());
        myUVNodes.Assign(aUVNodeWrapper);
    }

    /// <summary>A full copy of a triangulation (its parameters copied too).</summary>
    public Poly_Triangulation(Poly_Triangulation theTriangulation)
    {
        myDeflection = theTriangulation.myDeflection;
        myNodes = new Poly_ArrayOfNodes(theTriangulation.myNodes);
        myTriangles = new NCollection_Array1<Poly_Triangle>(theTriangulation.myTriangles);
        myUVNodes = new Poly_ArrayOfUVNodes(theTriangulation.myUVNodes);
        myNormals = new NCollection_Array1<NCollection_Vec3<float>>(theTriangulation.myNormals);
        myPurpose = theTriangulation.myPurpose;
        myParams = theTriangulation.myParams?.Copy();
        if (theTriangulation.HasCachedMinMax())
        {
            SetCachedMinMax(theTriangulation.CachedMinMax());
        }
    }

    /// <summary>A full copy of this triangulation.</summary>
    public virtual Poly_Triangulation Copy() => new(this);

    /// <summary>The deflection of this triangulation.</summary>
    public double Deflection() => myDeflection;

    /// <summary>Sets the deflection of this triangulation.</summary>
    public void Deflection(double theDeflection) => myDeflection = theDeflection;

    /// <summary>The parameters this triangulation was generated with, or null.</summary>
    public Poly_TriangulationParameters? Parameters() => myParams;

    /// <summary>Sets the parameters this triangulation was generated with.</summary>
    public void Parameters(Poly_TriangulationParameters? theParams) => myParams = theParams;

    /// <summary>Clears the nodes, the triangles and all the attributes.</summary>
    public virtual void Clear()
    {
        if (!myNodes.IsEmpty())
        {
            var anEmptyNodes = new Poly_ArrayOfNodes();
            anEmptyNodes.SetDoublePrecision(myNodes.IsDoublePrecision());
            myNodes.Move(anEmptyNodes);
        }

        if (!myTriangles.IsEmpty())
        {
            var anEmptyTriangles = new NCollection_Array1<Poly_Triangle>();
            myTriangles.Move(anEmptyTriangles);
        }

        RemoveUVNodes();
        RemoveNormals();
    }

    /// <summary>True if the triangulation has nodes and triangles.</summary>
    public virtual bool HasGeometry() => !myNodes.IsEmpty() && !myTriangles.IsEmpty();

    public int NbNodes() => myNodes.Length();

    public int NbTriangles() => myTriangles.Length();

    /// <summary>True if UV nodes are defined.</summary>
    public bool HasUVNodes() => !myUVNodes.IsEmpty();

    /// <summary>True if nodal normals are defined.</summary>
    public bool HasNormals() => !myNormals.IsEmpty();

    /// <summary>The node at an index in [1, NbNodes()].</summary>
    public gp_Pnt Node(int theIndex) => myNodes.Value(theIndex - 1);

    /// <summary>Sets the node at an index in [1, NbNodes()].</summary>
    public void SetNode(int theIndex, in gp_Pnt thePnt) => myNodes.SetValue(theIndex - 1, thePnt);

    /// <summary>The UV node at an index in [1, NbNodes()].</summary>
    public gp_Pnt2d UVNode(int theIndex) => myUVNodes.Value(theIndex - 1);

    /// <summary>Sets the UV node at an index in [1, NbNodes()].</summary>
    public void SetUVNode(int theIndex, in gp_Pnt2d thePnt) => myUVNodes.SetValue(theIndex - 1, thePnt);

    /// <summary>The triangle at an index in [1, NbTriangles()].</summary>
    public Poly_Triangle Triangle(int theIndex) => myTriangles.Value(theIndex);

    /// <summary>Sets the triangle at an index in [1, NbTriangles()].</summary>
    public void SetTriangle(int theIndex, in Poly_Triangle theTriangle) => myTriangles.SetValue(theIndex, theTriangle);

    /// <summary>The normal at a node index in [1, NbNodes()].</summary>
    public gp_Dir Normal(int theIndex)
    {
        NCollection_Vec3<float> aNorm = myNormals.Value(theIndex - 1);
        return new gp_Dir(aNorm.x(), aNorm.y(), aNorm.z());
    }

    /// <summary>The normal at a node index in [1, NbNodes()], as stored (single precision).</summary>
    public void Normal(int theIndex, out NCollection_Vec3<float> theVec3) => theVec3 = myNormals.Value(theIndex - 1);

    /// <summary>Sets the normal (a unit vector) at a node index in [1, NbNodes()].</summary>
    public void SetNormal(int theIndex, in NCollection_Vec3<float> theNormal) => myNormals.SetValue(theIndex - 1, theNormal);

    /// <summary>Sets the normal at a node index in [1, NbNodes()], rounded to single precision.</summary>
    public void SetNormal(int theIndex, in gp_Dir theNormal)
        => SetNormal(theIndex, new NCollection_Vec3<float>((float)theNormal.X(), (float)theNormal.Y(), (float)theNormal.Z()));

    /// <summary>The mesh purpose bits.</summary>
    public Poly_MeshPurpose MeshPurpose() => myPurpose;

    /// <summary>Sets the mesh purpose bits.</summary>
    public void SetMeshPurpose(Poly_MeshPurpose thePurpose) => myPurpose = thePurpose;

    /// <summary>The cached range of the nodes, void if there is none.</summary>
    public Bnd_Box CachedMinMax()
    {
        lock (myCachedMinMaxMutex)
        {
            StrongBox<Bnd_Box>? aBox = myCachedMinMax;
            return aBox is null ? new Bnd_Box() : aBox.Value;
        }
    }

    /// <summary>
    /// Sets the cached range of the nodes, which must match them exactly, without a gap or transformation. A void box
    /// clears the cache.
    /// </summary>
    public void SetCachedMinMax(in Bnd_Box theBox)
    {
        if (theBox.IsVoid())
        {
            unsetCachedMinMax();
            return;
        }

        lock (myCachedMinMaxMutex)
        {
            StrongBox<Bnd_Box>? aBox = myCachedMinMax;
            if (aBox is null)
            {
                aBox = new StrongBox<Bnd_Box>(theBox);
                Volatile.Write(ref myCachedMinMax, aBox);
            }
            else
            {
                aBox.Value = theBox;
            }
        }
    }

    /// <summary>True if a range of the nodes is cached.</summary>
    public bool HasCachedMinMax() => Volatile.Read(ref myCachedMinMax) is not null;

    /// <summary>Caches the range of the nodes.</summary>
    public void UpdateCachedMinMax()
    {
        var aBox = new Bnd_Box();
        MinMax(ref aBox, new gp_Trsf(), true);
        SetCachedMinMax(aBox);
    }

    /// <summary>
    /// Extends <paramref name="theBox"/> by the range of this triangulation, transformed. The cached range is used when
    /// there is one and the transformation has no rotation, or <paramref name="theIsAccurate"/> is false, or there are
    /// no nodes. False if there is nothing to add (no nodes and no cached range).
    /// </summary>
    public bool MinMax(ref Bnd_Box theBox, in gp_Trsf theTrsf, bool theIsAccurate = false)
    {
        var aBox = new Bnd_Box();
        bool aUsedCache = false;
        lock (myCachedMinMaxMutex)
        {
            StrongBox<Bnd_Box>? aCachedBox = myCachedMinMax;
            if (aCachedBox is not null
                && (!HasGeometry() || !theIsAccurate || theTrsf.Form() == gp_Identity
                    || theTrsf.Form() == gp_Translation || theTrsf.Form() == gp_PntMirror
                    || theTrsf.Form() == gp_Scale))
            {
                aBox = aCachedBox.Value.Transformed(theTrsf);
                aUsedCache = true;
            }
        }

        if (!aUsedCache)
        {
            aBox = computeBoundingBox(theTrsf);
        }

        if (aBox.IsVoid())
        {
            return false;
        }

        theBox.Add(aBox);
        return true;
    }

    /// <summary>True if the nodes are in double precision (the default).</summary>
    public bool IsDoublePrecision() => myNodes.IsDoublePrecision();

    /// <summary>
    /// Sets whether the 3D and UV nodes are stored in double or single precision. Throws if they are already allocated.
    /// </summary>
    public void SetDoublePrecision(bool theIsDouble)
    {
        myNodes.SetDoublePrecision(theIsDouble);
        myUVNodes.SetDoublePrecision(theIsDouble);
    }

    /// <summary>Resizes the nodes and the UV nodes and normals with them, keeping the old values if asked.</summary>
    public void ResizeNodes(int theNbNodes, bool theToCopyOld)
    {
        myNodes.Resize(theNbNodes, theToCopyOld);
        if (!myUVNodes.IsEmpty())
        {
            myUVNodes.Resize(theNbNodes, theToCopyOld);
        }

        if (!myNormals.IsEmpty())
        {
            myNormals.Resize(0, theNbNodes - 1, theToCopyOld);
        }
    }

    /// <summary>Resizes the triangles, keeping the old ones if asked.</summary>
    public void ResizeTriangles(int theNbTriangles, bool theToCopyOld) => myTriangles.Resize(1, theNbTriangles, theToCopyOld);

    /// <summary>Allocates the UV nodes, if not yet allocated for every node.</summary>
    public void AddUVNodes()
    {
        if (myUVNodes.IsEmpty() || myUVNodes.Size() != myNodes.Size())
        {
            myUVNodes.Resize(myNodes.Size(), false);
        }
    }

    /// <summary>Deallocates the UV nodes.</summary>
    public void RemoveUVNodes()
    {
        if (!myUVNodes.IsEmpty())
        {
            var anEmpty = new Poly_ArrayOfUVNodes();
            anEmpty.SetDoublePrecision(myUVNodes.IsDoublePrecision());
            myUVNodes.Move(anEmpty);
        }
    }

    /// <summary>Allocates the normals, if not yet allocated for every node.</summary>
    public void AddNormals()
    {
        if (myNormals.IsEmpty() || myNormals.Length() != myNodes.Length())
        {
            myNormals.Resize(0, myNodes.Size() - 1, false);
        }
    }

    /// <summary>Deallocates the normals.</summary>
    public void RemoveNormals()
    {
        if (!myNormals.IsEmpty())
        {
            var anEmpty = new NCollection_Array1<NCollection_Vec3<float>>();
            myNormals.Move(anEmpty);
        }
    }

    /// <summary>Computes smooth normals at the nodes, averaging the normals of the triangles (weighted by area).</summary>
    public void ComputeNormals()
    {
        // zero values
        AddNormals();
        myNormals.Init(new NCollection_Vec3<float>(0.0f));

        Span<int> anElem = [0, 0, 0];
        foreach (ref readonly Poly_Triangle aTri in myTriangles.AsSpan())
        {
            aTri.Get(out anElem[0], out anElem[1], out anElem[2]);
            gp_Pnt aNode0 = myNodes.Value(anElem[0] - 1);
            gp_Pnt aNode1 = myNodes.Value(anElem[1] - 1);
            gp_Pnt aNode2 = myNodes.Value(anElem[2] - 1);

            gp_XYZ aVec01 = aNode1.XYZ() - aNode0.XYZ();
            gp_XYZ aVec02 = aNode2.XYZ() - aNode0.XYZ();
            gp_XYZ aTriNorm = aVec01 ^ aVec02;
            var aNorm3f = new NCollection_Vec3<float>((float)aTriNorm.X(), (float)aTriNorm.Y(), (float)aTriNorm.Z());
            for (int aNodeIter = 0; aNodeIter < 3; ++aNodeIter)
            {
                myNormals.ChangeValue(anElem[aNodeIter] - 1).Add(aNorm3f);
            }
        }

        // Normalize all vectors
        foreach (ref NCollection_Vec3<float> aNorm3f in myNormals.AsSpan())
        {
            float aMod = aNorm3f.Modulus();
            aNorm3f = aMod == 0.0f ? new NCollection_Vec3<float>(0.0f, 0.0f, 1.0f) : (aNorm3f / aMod);
        }
    }

    /// <summary>
    /// The nodes as a 1-based array, or null if there are none. In double precision the array shares the nodes'
    /// storage (OCCT: not to be used after the triangulation changes); in single precision it is a copy.
    /// </summary>
    public NCollection_HArray1<gp_Pnt>? MapNodeArray()
    {
        if (myNodes.IsEmpty())
        {
            return null;
        }

        if (myNodes.IsDoublePrecision() && myNodes.Storage() is (gp_Pnt[] aPntArr, int aByteOffset))
        {
            // wrap array
            var anHArray = new NCollection_HArray1<gp_Pnt>();
            var anArray = new NCollection_Array1<gp_Pnt>(aPntArr, aByteOffset / Unsafe.SizeOf<gp_Pnt>(), 1, NbNodes());
            anHArray.Move(anArray);
            return anHArray;
        }

        // deep copy
        var aCopy = new NCollection_HArray1<gp_Pnt>(1, NbNodes());
        for (int aNodeIter = 0; aNodeIter < NbNodes(); ++aNodeIter)
        {
            gp_Pnt aPnt = myNodes.Value(aNodeIter);
            aCopy.SetValue(aNodeIter + 1, aPnt);
        }

        return aCopy;
    }

    /// <summary>The triangles as a 1-based array sharing their storage, or null if there are none.</summary>
    public NCollection_HArray1<Poly_Triangle>? MapTriangleArray()
    {
        if (myTriangles.IsEmpty())
        {
            return null;
        }

        var anHArray = new NCollection_HArray1<Poly_Triangle>();
        var anArray = new NCollection_Array1<Poly_Triangle>(myTriangles, myTriangles.Lower(), 1, NbTriangles());
        anHArray.Move(anArray);
        return anHArray;
    }

    /// <summary>
    /// The UV nodes as a 1-based array, or null if there are none. In double precision the array shares the nodes'
    /// storage; in single precision it is a copy.
    /// </summary>
    public NCollection_HArray1<gp_Pnt2d>? MapUVNodeArray()
    {
        if (myUVNodes.IsEmpty())
        {
            return null;
        }

        if (myUVNodes.IsDoublePrecision() && myUVNodes.Storage() is (gp_Pnt2d[] aPntArr, int aByteOffset))
        {
            // wrap array
            var anHArray = new NCollection_HArray1<gp_Pnt2d>();
            var anArray = new NCollection_Array1<gp_Pnt2d>(aPntArr, aByteOffset / Unsafe.SizeOf<gp_Pnt2d>(), 1, NbNodes());
            anHArray.Move(anArray);
            return anHArray;
        }

        // deep copy
        var aCopy = new NCollection_HArray1<gp_Pnt2d>(1, NbNodes());
        for (int aNodeIter = 0; aNodeIter < NbNodes(); ++aNodeIter)
        {
            gp_Pnt2d aPnt = myUVNodes.Value(aNodeIter);
            aCopy.SetValue(aNodeIter + 1, aPnt);
        }

        return aCopy;
    }

    /// <summary>
    /// The normals as a 1-based array of 3 × NbNodes() floats, or null if there are none. OCCT's array shares the
    /// normals' storage; the port's is a copy, since a .NET float array can't alias an array of vectors.
    /// </summary>
    public NCollection_HArray1<float>? MapNormalArray()
    {
        if (myNormals.IsEmpty())
        {
            return null;
        }

        var anHArray = new NCollection_HArray1<float>(1, 3 * NbNodes());
        NormalComponents().CopyTo(anHArray.AsSpan());
        return anHArray;
    }

    // Bulk access (OCCT.NET additions)

    /// <summary>
    /// The node coordinates (x, y, z for each node), without copying, when the nodes are in double precision; throws
    /// <see cref="Standard_TypeMismatch"/> in single precision (use <see cref="NodeCoordinatesF"/> or
    /// <see cref="CopyNodeCoordinatesTo(Span{double})"/>). Valid until the nodes are resized.
    /// </summary>
    public ReadOnlySpan<double> NodeCoordinates() => MemoryMarshal.Cast<gp_Pnt, double>(myNodes.AsSpan<gp_Pnt>());

    /// <summary>
    /// The node coordinates (x, y, z for each node), without copying, when the nodes are in single precision; throws
    /// <see cref="Standard_TypeMismatch"/> in double precision. Valid until the nodes are resized.
    /// </summary>
    public ReadOnlySpan<float> NodeCoordinatesF() => MemoryMarshal.Cast<NCollection_Vec3<float>, float>(myNodes.AsSpan<NCollection_Vec3<float>>());

    /// <summary>Copies the node coordinates (x, y, z for each node) in double precision, whatever the storage.</summary>
    public void CopyNodeCoordinatesTo(Span<double> theDestination)
    {
        if (myNodes.IsDoublePrecision())
        {
            NodeCoordinates().CopyTo(theDestination);
            return;
        }

        ReadOnlySpan<float> aSource = NodeCoordinatesF();
        Standard_DimensionMismatch_Raise_if(theDestination.Length < aSource.Length, "Poly_Triangulation::CopyNodeCoordinatesTo");
        for (int i = 0; i < aSource.Length; i++)
        {
            theDestination[i] = aSource[i];
        }
    }

    /// <summary>Copies the node coordinates (x, y, z for each node) in single precision, whatever the storage.</summary>
    public void CopyNodeCoordinatesTo(Span<float> theDestination)
    {
        if (!myNodes.IsDoublePrecision())
        {
            NodeCoordinatesF().CopyTo(theDestination);
            return;
        }

        ReadOnlySpan<double> aSource = NodeCoordinates();
        Standard_DimensionMismatch_Raise_if(theDestination.Length < aSource.Length, "Poly_Triangulation::CopyNodeCoordinatesTo");
        for (int i = 0; i < aSource.Length; i++)
        {
            theDestination[i] = (float)aSource[i];
        }
    }

    /// <summary>
    /// The triangles' node indices (three 1-based indices for each triangle), without copying. Valid until the
    /// triangles are resized.
    /// </summary>
    public ReadOnlySpan<int> TriangleIndices() => MemoryMarshal.Cast<Poly_Triangle, int>(myTriangles.AsSpan());

    /// <summary>The triangles, without copying. Valid until they are resized.</summary>
    public ReadOnlySpan<Poly_Triangle> TriangleSpan() => myTriangles.AsSpan();

    /// <summary>
    /// The UV coordinates (u, v for each node), without copying, when they are in double precision; throws
    /// <see cref="Standard_TypeMismatch"/> in single precision (use <see cref="UVNodeCoordinatesF"/>). Empty if there
    /// are no UV nodes.
    /// </summary>
    public ReadOnlySpan<double> UVNodeCoordinates() => MemoryMarshal.Cast<gp_Pnt2d, double>(myUVNodes.AsSpan<gp_Pnt2d>());

    /// <summary>The UV coordinates (u, v for each node), without copying, when they are in single precision.</summary>
    public ReadOnlySpan<float> UVNodeCoordinatesF() => MemoryMarshal.Cast<NCollection_Vec2<float>, float>(myUVNodes.AsSpan<NCollection_Vec2<float>>());

    /// <summary>The normals (x, y, z for each node), without copying. Empty if there are no normals.</summary>
    public ReadOnlySpan<float> NormalComponents() => MemoryMarshal.Cast<NCollection_Vec3<float>, float>(myNormals.AsSpan());

    // Internal arrays: for the mesher and other ported code

    /// <summary>The internal array of triangles; <see cref="Triangle"/> and <see cref="SetTriangle"/> are preferred.</summary>
    public NCollection_Array1<Poly_Triangle> InternalTriangles() => myTriangles;

    /// <summary>The internal array of nodes; <see cref="Node"/> and <see cref="SetNode"/> are preferred.</summary>
    public Poly_ArrayOfNodes InternalNodes() => myNodes;

    /// <summary>The internal array of UV nodes; <see cref="UVNode"/> and <see cref="SetUVNode"/> are preferred.</summary>
    public Poly_ArrayOfUVNodes InternalUVNodes() => myUVNodes;

    /// <summary>The internal array of normals; <see cref="Normal(int)"/> and <see cref="SetNormal(int, in gp_Dir)"/> are preferred.</summary>
    public NCollection_Array1<NCollection_Vec3<float>> InternalNormals() => myNormals;

    /// <summary>Sets the normals from 3 × NbNodes() floats. Deprecated in OCCT: use <see cref="SetNormal(int, in NCollection_Vec3{float})"/>.</summary>
    [Obsolete("Deprecated method, SetNormal() should be used instead")]
    public void SetNormals(NCollection_HArray1<float>? theNormals)
    {
        if (theNormals is null || theNormals.Length() != 3 * NbNodes())
        {
            throw new Standard_DomainError("Poly_Triangulation::SetNormals : wrong length");
        }

        AddNormals();
        int anArrayLower = theNormals.Lower();
        for (int aNodeIter = 1; aNodeIter <= NbNodes(); ++aNodeIter)
        {
            int anArrayInd = anArrayLower + (aNodeIter - 1) * 3;
            var aNorm = new NCollection_Vec3<float>(theNormals.Value(anArrayInd + 0),
                                                    theNormals.Value(anArrayInd + 1),
                                                    theNormals.Value(anArrayInd + 2));
            SetNormal(aNodeIter, aNorm);
        }
    }

    [Obsolete("Deprecated method, Triangle() should be used instead")]
    public NCollection_Array1<Poly_Triangle> Triangles() => myTriangles;

    [Obsolete("Deprecated method, SetTriangle() should be used instead")]
    public NCollection_Array1<Poly_Triangle> ChangeTriangles() => myTriangles;

    [Obsolete("Deprecated method, SetTriangle() should be used instead")]
    public ref Poly_Triangle ChangeTriangle(int theIndex) => ref myTriangles.ChangeValue(theIndex);

    // Deferred data (OCCT's OSD_FileSystem argument is left out: the file system isn't ported)

    /// <summary>The estimated number of nodes that <see cref="LoadDeferredData"/> can load.</summary>
    public virtual int NbDeferredNodes() => 0;

    /// <summary>The estimated number of triangles that <see cref="LoadDeferredData"/> can load.</summary>
    public virtual int NbDeferredTriangles() => 0;

    /// <summary>True if there is triangulation data that <see cref="LoadDeferredData"/> can load.</summary>
    public virtual bool HasDeferredData() => NbDeferredTriangles() > 0;

    /// <summary>Loads the deferred triangulation data into this triangulation.</summary>
    public virtual bool LoadDeferredData()
    {
        if (!HasDeferredData())
        {
            return false;
        }

        if (!loadDeferredData(this))
        {
            return false;
        }

        SetMeshPurpose(myPurpose | Poly_MeshPurpose_Loaded);
        return true;
    }

    /// <summary>Loads the deferred triangulation data into a new triangulation, or returns null.</summary>
    public virtual Poly_Triangulation? DetachedLoadDeferredData()
    {
        if (!HasDeferredData())
        {
            return null;
        }

        Poly_Triangulation aResult = createNewEntity();
        if (!loadDeferredData(aResult))
        {
            return null;
        }

        aResult.SetMeshPurpose(aResult.MeshPurpose() | Poly_MeshPurpose_Loaded);
        return aResult;
    }

    /// <summary>Releases the triangulation data if it has deferred storage.</summary>
    public virtual bool UnloadDeferredData()
    {
        if (HasDeferredData())
        {
            Clear();
            SetMeshPurpose(myPurpose & ~Poly_MeshPurpose_Loaded);
            return true;
        }

        return false;
    }

    /// <summary>Creates a new, empty triangulation of the same kind (a subclass may return its own type).</summary>
    protected virtual Poly_Triangulation createNewEntity() => new();

    /// <summary>Loads the deferred data into <paramref name="theDestTriangulation"/>; false by default.</summary>
    protected virtual bool loadDeferredData(Poly_Triangulation theDestTriangulation) => false;

    /// <summary>Clears the cached range.</summary>
    protected void unsetCachedMinMax()
    {
        lock (myCachedMinMaxMutex)
        {
            Volatile.Write(ref myCachedMinMax, null);
        }
    }

    /// <summary>The bounding box of the nodes, transformed.</summary>
    protected virtual Bnd_Box computeBoundingBox(in gp_Trsf theTrsf)
    {
        var aBox = new Bnd_Box();
        if (theTrsf.Form() == gp_Identity)
        {
            for (int aNodeIdx = 0; aNodeIdx < NbNodes(); aNodeIdx++)
            {
                aBox.Add(myNodes.Value(aNodeIdx));
            }
        }
        else
        {
            for (int aNodeIdx = 0; aNodeIdx < NbNodes(); aNodeIdx++)
            {
                aBox.Add(myNodes.Value(aNodeIdx).Transformed(theTrsf));
            }
        }

        return aBox;
    }

    public override string ToString()
        => string.Create(CultureInfo.InvariantCulture,
                         $"Poly_Triangulation({NbNodes()} nodes, {NbTriangles()} triangles, deflection {myDeflection:R})");
}
