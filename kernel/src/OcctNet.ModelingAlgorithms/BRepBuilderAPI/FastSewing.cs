// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_FastSewing.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_FastSewing.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Fast sewing of surfaces (faces). It supposes that all the surfaces are finite and naturally restricted by their
/// bounds, and that surfaces stitched together have the same parametrisation along their common boundaries, so it
/// doesn't check the SameParameter property of edges.
/// </summary>
/// <remarks>
/// <para>To sew: set the tolerance (default 1e-6), add the surfaces or faces, check the status, call
/// <see cref="Perform"/>, read the error status if any and the result.</para>
/// <para>A working object (decision 21). It only reads its inputs: the faces it builds are empty copies of the
/// caller's, sharing their surfaces. <see cref="GetResult"/> hands out the result frozen.</para>
/// </remarks>
public class FastSewing : Transient
{
    /// <summary>Result statuses (bits of <see cref="GetStatuses"/>).</summary>
    public enum FS_Statuses : uint
    {
        FS_OK = 0x00000000,
        FS_Degenerated = 0x00000001,
        FS_FindVertexError = 0x00000002,
        FS_FindEdgeError = 0x00000004,
        FS_FaceWithNullSurface = 0x00000008,
        FS_NotNaturalBoundsFace = 0x00000010,
        FS_InfiniteSurface = 0x00000020,
        FS_EmptyInput = 0x00000040,
        FS_Exception = 0x00000080
    }

    private Shape _resShape = new();

    // myFaceVec, myVertexVec and myEdgeVec lists are filled only once!!!!!
    // Vector of faces
    private readonly NCollection_DynamicArray<FS_Face> _faceVec = new();
    // Vector of Vertices
    private readonly NCollection_DynamicArray<FS_Vertex> _vertexVec = new();
    // Vector of edges
    private readonly NCollection_DynamicArray<FS_Edge> _edgeVec = new();
    // Tolerance
    private double _tolerance;
    // Bits of computation status
    private uint _statusList;

    //=======================================================================
    // function : IntersetctionOfSets
    // purpose  : Returns minimal value of intersection result
    //=======================================================================
    private static int IntersectionOfSets(NCollection_List<int> set1, NCollection_List<int> set2)
    {
        int intMax = IntegerLast();
        int retVal = intMax;
        for (var it1 = new NCollection_List<int>.Iterator(set1); it1.More(); it1.Next())
        {
            int val1 = it1.Value();
            for (var it2 = new NCollection_List<int>.Iterator(set2); it2.More(); it2.Next())
            {
                int val2 = it2.Value();
                if (val1 == val2)
                {
                    // theIntersectionResult.Append(aVal1);
                    if (val1 < retVal)
                    {
                        retVal = val1;
                    }
                }
            }
        }

        if (retVal == intMax)
        {
            return -1;
        }

        return retVal;
    }

    //=================================================================================================

    private static Curve2D Get2DCurve(int index,
                                           double ufirst,
                                           double ulast,
                                           double vfirst,
                                           double vlast,
                                           bool isReverse = false)
    {
        if ((index < 0) || (index > 3))
        {
            throw new OutOfRangeException("BRepBuilderAPI_FastSewing.cxx, Get2DCurve(): OUT of Range");
        }

        Curve2D? a2dCurv = null;

        if (!isReverse)
        {
            switch (index)
            {
                case 0:
                    a2dCurv = new TrimmedCurve2D(
                        new LineCurve2D(new Point2D(0.0, vfirst), new Direction2D(Direction2D.D.X)),
                        ufirst,
                        ulast);
                    break;
                case 1:
                    a2dCurv = new TrimmedCurve2D(
                        new LineCurve2D(new Point2D(ulast, 0.0), new Direction2D(Direction2D.D.Y)),
                        vfirst,
                        vlast);
                    break;
                case 2:
                    a2dCurv = new TrimmedCurve2D(
                        new LineCurve2D(new Point2D(0.0, vlast), new Direction2D(Direction2D.D.X)),
                        ufirst,
                        ulast);
                    break;
                case 3:
                    a2dCurv = new TrimmedCurve2D(
                        new LineCurve2D(new Point2D(ufirst, 0.0), new Direction2D(Direction2D.D.Y)),
                        vfirst,
                        vlast);
                    break;
                default:
                    break;
            }
        }
        else
        {
            switch (index)
            {
                case 0:
                    a2dCurv = new TrimmedCurve2D(
                        new LineCurve2D(new Point2D(ufirst + ulast, vfirst), new Direction2D(Direction2D.D.NX)),
                        ufirst,
                        ulast);
                    break;
                case 1:
                    a2dCurv = new TrimmedCurve2D(
                        new LineCurve2D(new Point2D(ulast, vfirst + vlast), new Direction2D(Direction2D.D.NY)),
                        vfirst,
                        vlast);
                    break;
                case 2:
                    a2dCurv = new TrimmedCurve2D(
                        new LineCurve2D(new Point2D(ufirst + ulast, vlast), new Direction2D(Direction2D.D.NX)),
                        ufirst,
                        ulast);
                    break;
                case 3:
                    a2dCurv = new TrimmedCurve2D(
                        new LineCurve2D(new Point2D(ufirst, vfirst + vlast), new Direction2D(Direction2D.D.NY)),
                        vfirst,
                        vlast);
                    break;
                default:
                    break;
            }
        }

        return a2dCurv!;
    }

    //=================================================================================================

    /// <summary>Creates an object with the tolerance of connexity <paramref name="tol"/>.</summary>
    public FastSewing(double tol = 1.0e-06)
    {
        _tolerance = tol;
        _statusList = 0;
    }

    /// <summary>Sets the tolerance.</summary>
    public void SetTolerance(double toler) => _tolerance = toler;

    /// <summary>The tolerance.</summary>
    public double GetTolerance() => _tolerance;

    /// <summary>The resulting shape, frozen.</summary>
    public Shape GetResult()
    {
        _resShape.TShape()?.FreezeGraph();
        return _resShape;
    }

    /// <summary>Sets a status; returns its numeric value.</summary>
    protected uint SetStatus(FS_Statuses status)
    {
        uint statusID = (uint)status;
        _statusList |= statusID;
        return statusID;
    }

    //=================================================================================================

    /// <summary>Adds the faces of a shape; false if none was added.</summary>
    public bool Add(in Shape shape)
    {
        bool result = false;
        if (shape.IsNull)
        {
            SetStatus(FS_Statuses.FS_EmptyInput);
            return result;
        }

        var ms = new NCollection_Map<Shape>(ShapeHasher.Instance);
        // aMS.Add(theShape);
        var fExp = new Explorer(shape, ShapeType.Face);
        for (; fExp.More(); fExp.Next())
        {
            Face face = ShapeCasts.Face(fExp.Current());
            if (ms.Add(face))
            {
                Surface? surf = Tool.Surface(face);
                if (surf is null)
                {
                    SetStatus(FS_Statuses.FS_FaceWithNullSurface);
                    continue;
                }

                if (surf is TrimmedSurface)
                {
                    SetStatus(FS_Statuses.FS_NotNaturalBoundsFace);
                    continue;
                }

                surf.Bounds(out double uf, out double ul, out double vf, out double vl);
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(uf) || global::Kerf.Engine.Foundation.Precision.IsInfinite(ul) || global::Kerf.Engine.Foundation.Precision.IsInfinite(vf)
                    || global::Kerf.Engine.Foundation.Precision.IsInfinite(vl))
                {
                    SetStatus(FS_Statuses.FS_InfiniteSurface);
                    continue;
                }

                var fFace = new FS_Face();
                fFace.mySrcFace = face;
                fFace.myID = _faceVec.Length(); // because start index is 0
                _faceVec.Append(fFace);
                result = true;
            }
        }

        return result;
    }

    //=================================================================================================

    /// <summary>Adds a surface; false if it can't be sewn (null, trimmed or infinite).</summary>
    public bool Add(Surface? surface)
    {
        if (surface is null)
        {
            SetStatus(FS_Statuses.FS_FaceWithNullSurface);
            return false;
        }

        if (surface is TrimmedSurface)
        {
            SetStatus(FS_Statuses.FS_NotNaturalBoundsFace);
            return false;
        }

        surface.Bounds(out double uf, out double ul, out double vf, out double vl);
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(uf) || global::Kerf.Engine.Foundation.Precision.IsInfinite(ul) || global::Kerf.Engine.Foundation.Precision.IsInfinite(vf)
            || global::Kerf.Engine.Foundation.Precision.IsInfinite(vl))
        {
            SetStatus(FS_Statuses.FS_InfiniteSurface);
            return false;
        }

        var face = new FS_Face();

        var builder = new BrepBuilder();

        builder.MakeFace(ref face.mySrcFace);
        builder.MakeFace(ref face.mySrcFace, surface, _tolerance);
        builder.NaturalRestriction(face.mySrcFace, true);

        face.myID = _faceVec.Length(); // because start index is 0
        _faceVec.Append(face);

        return true;
    }

    //=================================================================================================

    /// <summary>Computes the resulting shape.</summary>
    public void Perform()
    {
        if (_faceVec.IsEmpty())
        {
            SetStatus(FS_Statuses.FS_EmptyInput);
            return;
        }

        try
        {
            {
                // create vertices having unique coordinates
                double range = Compute3DRange();
                var cells = new CellFilter<NodeInspector, int, Xyz>(Max(_tolerance, range / IntegerLast()));

                for (int i = _faceVec.Lower(); i <= _faceVec.Upper(); i++)
                {
                    FindVertexes(i, cells);
                }
            }

            for (int i = _faceVec.Lower(); i <= _faceVec.Upper(); i++)
            {
                FindEdges(i);
            }

            // Create topological structures

            for (int i = _vertexVec.Lower(); i <= _vertexVec.Upper(); i++)
            {
                _vertexVec.Value(i).CreateTopologicalVertex(_tolerance);
            }

            // Edges
            for (int i = _edgeVec.Lower(); i <= _edgeVec.Upper(); i++)
            {
                _edgeVec.Value(i).CreateTopologicalEdge(_vertexVec, _faceVec, _tolerance);
            }

            // Shell
            var quilt = new Quilt();

            // Faces
            for (int i = _faceVec.Lower(); i <= _faceVec.Upper(); i++)
            {
                FS_Face face = _faceVec.Value(i);
                face.CreateTopologicalWire(_edgeVec, _tolerance);

                face.CreateTopologicalFace();
                quilt.Add(face.myRetFace);
            }

            _resShape = quilt.InternalShells();
        }
        catch (FailureException)
        {
            SetStatus(FS_Statuses.FS_Exception);
            return;
        }
    }

    //=================================================================================================

    /// <summary>Updates the information of an existing edge.</summary>
    protected void UpdateEdgeInfo(int idPrevVertex, int idCurrVertex, int faceID, int idCurvOnFace)
    {
        // Indeed, two vertices combine into one edge only.
        int edgeID = IntersectionOfSets(_vertexVec.Value(idPrevVertex).myEdges,
                                          _vertexVec.Value(idCurrVertex).myEdges);

        // For DEBUG mode only
        Standard_ProgramError_Raise_if(edgeID < 0,
                                       "BRepBuilderAPI_FastSewing::UpdateEdgeInfo: Update not existing edge.");

        FS_Edge edge = _edgeVec.Value(edgeID);
        edge.myFaces.Append(faceID);
        FS_Face face = _faceVec.Value(faceID);
        face.SetEdge(idCurvOnFace, edge.myID);
    }

    //=================================================================================================

    /// <summary>Creates a new edge.</summary>
    protected void CreateNewEdge(int idPrevVertex, int idCurrVertex, int faceID, int idCurvOnFace)
    {
        var edge = new FS_Edge(idPrevVertex, idCurrVertex);
        edge.myID = _edgeVec.Length(); // because start index is 0

        edge.myFaces.Append(faceID);
        FS_Face face = _faceVec.Value(faceID);
        face.SetEdge(idCurvOnFace, edge.myID);

        _vertexVec.Value(idPrevVertex).myEdges.Append(edge.myID);

        if (idPrevVertex == idCurrVertex)
        { // the Edge is degenerated
            SetStatus(FS_Statuses.FS_Degenerated);
        }
        else
        {
            _vertexVec.Value(idCurrVertex).myEdges.Append(edge.myID);
        }

        _edgeVec.Append(edge);
    }

    //=================================================================================================

    /// <summary>Finds or creates the vertices of the corners of a face.</summary>
    protected void FindVertexes(int surfID, CellFilter<NodeInspector, int, Xyz> cells)
    {
        const int nbPoints = 4;
        FS_Face face = _faceVec.Value(surfID);
        Surface surf = Tool.Surface(face.mySrcFace)!;
        surf.Bounds(out double uf, out double ul, out double vf, out double vl);

        Point3D[] pnts = [surf.Value(uf, vf),
                          surf.Value(ul, vf),
                          surf.Value(ul, vl),
                          surf.Value(uf, vl)];

        for (int i = 0; i < nbPoints; i++)
        {
            var vert = new FS_Vertex();

            var inspector = new NodeInspector(_vertexVec, pnts[i], _tolerance);
            var box = new Box();
            box.Add(pnts[i]);
            box.Enlarge(_tolerance);

            cells.Inspect(box.CornerMin().XYZ(), box.CornerMax().XYZ(), ref inspector);
            int resID = inspector.GetResult();

            if (resID < 0)
            {                                     // Add new Vertex
                vert.myID = _vertexVec.Length(); // because start index is 0
                vert.myPnt = pnts[i];
                vert.myFaces.Append(surfID);
                _vertexVec.Append(vert);
                face.SetVertex(i, vert.myID);

                cells.Add(vert.myID, box.CornerMin().XYZ(), box.CornerMax().XYZ());
            }
            else
            { // Change existing vertex
                face.SetVertex(i, resID);
                _vertexVec.Value(resID).myFaces.Append(surfID);
            }
        }
    }

    //=================================================================================================

    /// <summary>Finds or creates the edges of a face.</summary>
    protected void FindEdges(int surfID)
    {
        const int nbPoints = 4;
        FS_Face face = _faceVec.Value(surfID);

        ReadOnlySpan<int> firstInd = [0, 1, 3, 0];
        ReadOnlySpan<int> lastInd = [1, 2, 2, 3];

        for (int i = 0; i < nbPoints; i++)
        {
            int firstVertIndex = firstInd[i], lastVertIndex = lastInd[i];
            int firstVertID = face.myVertices[firstVertIndex],
                lastVertID = face.myVertices[lastVertIndex];

            if (firstVertID == lastVertID)
            { // Edge is degenerated.
                CreateNewEdge(firstVertID, lastVertID, surfID, i);
                continue;
            }

            // Must be minimal element from list
            int intRes = IntersectionOfSets(_vertexVec.Value(firstVertID).myFaces,
                                              _vertexVec.Value(lastVertID).myFaces);

            if ((intRes < 0) || (intRes >= surfID))
            {
                CreateNewEdge(firstVertID, lastVertID, surfID, i);
            }
            else
            { // if(theSurfID > anIntRes) => The edge has been processed earlier
                UpdateEdgeInfo(firstVertID, lastVertID, surfID, i);
            }
        }
    }

    //=================================================================================================

    /// <summary>The bits of the statuses; prints a message for each to <paramref name="os"/> if given.</summary>
    public uint GetStatuses(TextWriter? os = null)
    {
        if (os is null)
        {
            return _statusList;
        }

        if (_statusList == 0)
        {
            os.Write("Fast Sewing OK!\n");
            return _statusList;
        }

        // Number of bits
        const int numMax = 8 * sizeof(uint);
        var ids = (FS_Statuses)0x0001;
        for (int i = 1; i <= numMax; i++, ids = (FS_Statuses)((uint)ids << 1))
        {
            if (((uint)ids & _statusList) == 0)
            {
                continue;
            }

            switch (ids)
            {
                case FS_Statuses.FS_Degenerated:
                    os.Write("Degenerated case. Try to reduce tolerance.\n");
                    break;
                case FS_Statuses.FS_FindVertexError:
                    os.Write("Error while creating list of vertices.\n");
                    break;
                case FS_Statuses.FS_FindEdgeError:
                    os.Write("Error while creating list of edges.\n");
                    break;
                case FS_Statuses.FS_Exception:
                    os.Write("Exception during the operation.\n");
                    break;
                case FS_Statuses.FS_FaceWithNullSurface:
                    os.Write("Source face has null surface.\n");
                    break;
                case FS_Statuses.FS_NotNaturalBoundsFace:
                    os.Write("Source face has trimmed surface.\n");
                    break;
                case FS_Statuses.FS_InfiniteSurface:
                    os.Write("Source face has the surface with infinite boundaries.\n");
                    break;
                case FS_Statuses.FS_EmptyInput:
                    os.Write("Empty source data.\n");
                    break;
                default:
                    return _statusList;
            }
        }

        return _statusList;
    }

    //=================================================================================================

    /// <summary>The largest extent of the box of the faces' corners.</summary>
    protected double Compute3DRange()
    {
        var box = new Box();

        for (int i = _faceVec.Lower(); i <= _faceVec.Upper(); i++)
        {
            FS_Face face = _faceVec.Value(i);
            Surface? surf = Tool.Surface(face.mySrcFace);

            if (surf is null)
            {
                continue;
            }

            surf.Bounds(out double uf, out double ul, out double vf, out double vl);

            box.Add(surf.Value(uf, vf));
            box.Add(surf.Value(ul, vf));
            box.Add(surf.Value(ul, vl));
            box.Add(surf.Value(uf, vl));
        }

        box.Get(out double aXm, out double aYm, out double aZm, out double xm, out double ym, out double zm);
        double delta = xm - aXm;
        delta = Max(delta, ym - aYm);
        delta = Max(delta, zm - aZm);

        return delta;
    }

    // Classes FS_Vertex, FS_Face and FS_Edge keep information about
    // relations between resulted members (e.g. which faces share this vertex? etc.)
    // (Structs in OCCT, held in vectors and changed in place there; classes here, so that the vectors' elements are
    // changed in place too.)

    /// <summary>A vertex of the result.</summary>
    protected sealed class FS_Vertex
    {
        /// <summary>Geometry point of this Vertex</summary>
        public Point3D myPnt = new();
        public Vertex myTopoVert = new();
        /// <summary>List of faces and edges which share this vertex</summary>
        public readonly NCollection_List<int> myFaces = new();
        public readonly NCollection_List<int> myEdges = new();
        /// <summary>Identifies the place of this Vertex in BRepBuilderAPI_FastSewing::myVertexVec list</summary>
        public int myID;

        public FS_Vertex()
        {
            myID = -1;
        }

        /// <summary>Creates topological member (vertex)</summary>
        public void CreateTopologicalVertex(double toler)
        {
            var builder = new BrepBuilder();
            builder.MakeVertex(ref myTopoVert, myPnt, toler);
        }
    }

    /// <summary>A face of the result.</summary>
    protected sealed class FS_Face
    {
        public Face mySrcFace = new();
        public Wire myWire = new();
        public Face myRetFace = new();
        /// <summary>
        /// myEdges[i] number of the edge in myEdgeVec; (i==0) &lt;-&gt; (V=Vf); (i==1) &lt;-&gt; (U=Ul);
        /// (i==2) &lt;-&gt; (V=Vl); (i==3) &lt;-&gt; (U=Uf)
        /// </summary>
        public readonly int[] myEdges = new int[4];
        /// <summary>myVertices[i] is Start point of myEdges[i]</summary>
        public readonly int[] myVertices = new int[4];
        /// <summary>Identifies the place of this Face in BRepBuilderAPI_FastSewing::myFaceVec list</summary>
        public int myID;

        public FS_Face()
        {
            myID = -1;
            for (int i = 0; i < 4; i++)
            {
                myVertices[i] = -1;
                myEdges[i] = -1;
            }
        }

        /// <summary>Sets vertex</summary>
        public void SetVertex(int placeID, int vertID)
        {
            Standard_RangeError_Raise_if((placeID < 0) || (placeID > 3), "FS_Face::SetVertex(): OUT of Range");
            myVertices[placeID] = vertID;
        }

        /// <summary>Sets edge</summary>
        public void SetEdge(int placeID, int edgeID)
        {
            Standard_RangeError_Raise_if((placeID < 0) || (placeID > 3), "FS_Face::SetEdge(): OUT of Range");
            myEdges[placeID] = edgeID;
        }

        //=================================================================================================

        /// <summary>Creates topological members (wire)</summary>
        public void CreateTopologicalWire(NCollection_DynamicArray<FS_Edge> edgeVec, double toler)
        {
            // 3D-curves in 1st and 2nd faces are considered to be in same-range
            Surface surf = Tool.Surface(mySrcFace, out Placement location)!;

            surf.Bounds(out double uf, out double ul, out double vf, out double vl);

            var b = new BrepBuilder();
            b.MakeWire(ref myWire);
            for (int edge = 0; edge < 4; edge++)
            {
                Standard_ProgramError_Raise_if(
                    myEdges[edge] < 0,
                    "BRepBuilderAPI_FastSewing::FS_Face::CreateTopologicalWire: Wire is not closed.");

                FS_Edge fsEdge = edgeVec.Value(myEdges[edge]);
                Orientation ori = edge < 2 ? Forward : Reversed;
                Edge topE = fsEdge.myTopoEdge;

                if (fsEdge.IsDegenerated)
                {
                    topE.Orientation(ori);
                    b.Add(myWire, topE);
                    continue;
                }

                // Check if 3D and 2D-curve have same-orientation.
                // If it is not, 2d-curve will be reversed.
                {
                    Curve3D a3dCurv = Tool.Curve(topE, out double firstPar, out double lastPar)!;
                    Curve2D a2dCurv = Get2DCurve(edge, uf, ul, vf, vl);
                    Point3D pref = a3dCurv.Value(firstPar);
                    Point2D p2df = a2dCurv.Value(firstPar), p2dl = a2dCurv.Value(lastPar);
                    Point3D p3df = surf.Value(p2df.X(), p2df.Y());
                    Point3D p3dl = surf.Value(p2dl.X(), p2dl.Y());

                    p3df.Transform(location.Transformation());
                    p3dl.Transform(location.Transformation());

                    double sqD1 = p3df.SquareDistance(pref);
                    double sqD2 = p3dl.SquareDistance(pref);

                    if (sqD2 < sqD1)
                    {
                        a2dCurv = Get2DCurve(edge, uf, ul, vf, vl, true);
                        ori = global::Kerf.Engine.Topology.TopAbs.Reverse(ori);
                    }

                    b.UpdateEdge(topE, a2dCurv, surf, location, toler);
                }

                topE.Orientation(ori);

                b.Add(myWire, topE);
            }

            myWire.Closed(true);
        }

        //=================================================================================================

        /// <summary>Creates topological members (face)</summary>
        public void CreateTopologicalFace()
        {
            Standard_ProgramError_Raise_if(
                myWire.IsNull,
                "BRepBuilderAPI_FastSewing::FS_Face::CreateTopologicalFace: Cannot create wire.");

            var builder = new BrepBuilder();
            myRetFace = ShapeCasts.Face(mySrcFace.EmptyCopied());
            builder.Add(myRetFace, myWire);
            builder.NaturalRestriction(myRetFace, true);
        }
    }

    /// <summary>An edge of the result.</summary>
    protected sealed class FS_Edge
    {
        /// <summary>List of faces which are shared with this edge; values are indexes in myFaceVec</summary>
        public readonly NCollection_Sequence<int> myFaces = new();
        /// <summary>Identifies the place of this Edge in BRepBuilderAPI_FastSewing::myEdgeVec list</summary>
        public int myID;
        public Edge myTopoEdge = new();
        // Index of the vertex in myVertexVec array
        private readonly int[] _vertices = new int[2];

        public FS_Edge()
        {
            myID = -1;
            _vertices[0] = -1;
            _vertices[1] = -1;
        }

        public FS_Edge(int idVert1, int idVert2)
        {
            myID = -1;
            _vertices[0] = idVert1;
            _vertices[1] = idVert2;
        }

        /// <summary>Sets vertex</summary>
        public void SetVertex(int placeID, int vertID)
        {
            Standard_RangeError_Raise_if((placeID < 0) || (placeID > 1), "FS_Face::SetVertex(): OUT of Range");
            _vertices[placeID] = vertID;
        }

        public bool IsDegenerated => (_vertices[0] == _vertices[1]);

        //=================================================================================================

        /// <summary>Creates topological member (TopoDS_Edge)</summary>
        public void CreateTopologicalEdge(NCollection_DynamicArray<FS_Vertex> vertexVec,
                                          NCollection_DynamicArray<FS_Face> faceVec,
                                          double tol)
        {
            var builder = new BrepBuilder();

            Vertex v1 = vertexVec.Value(_vertices[0]).myTopoVert;
            Vertex v2 = vertexVec.Value(_vertices[1]).myTopoVert;

            v1.Orientation(Forward);
            v2.Orientation(Reversed);

            Curve3D a3dCurv;

            FS_Face face = faceVec.Value(myFaces.Value(NCollection_Sequence<int>.Lower()));

            // 3D-curves in 1st and 2nd faces are considered to be in same-range
            Surface surf = Tool.Surface(face.mySrcFace, out Placement location)!;

            surf.Bounds(out double uf, out double ul, out double vf, out double vl);

            int edgeID = -1;
            for (int ind = 0; ind < 4; ind++)
            {
                if (myID == face.myEdges[ind])
                {
                    edgeID = ind;
                    break;
                }
            }

            // For DEBUG mode only
            Standard_ProgramError_Raise_if(edgeID < 0,
                                           "BRepBuilderAPI_FastSewing::FS_Edge::CreateTopologicalEdge: Single edge.");

            if (IsDegenerated)
            {
                Curve2D a2dCurv = Get2DCurve(edgeID, uf, ul, vf, vl);
                double fPar = a2dCurv.FirstParameter(), lPar = a2dCurv.LastParameter();

                builder.MakeEdge(ref myTopoEdge);
                builder.UpdateEdge(myTopoEdge, a2dCurv, surf, location, tol);
                builder.Add(myTopoEdge, v1);
                builder.Add(myTopoEdge, v2);
                builder.Range(myTopoEdge, fPar, lPar);
                builder.Degenerated(myTopoEdge, true);
                return;
            }

            switch (edgeID)
            {
                case 0:
                    a3dCurv = surf.VIso(vf);
                    break;
                case 1:
                    a3dCurv = surf.UIso(ul);
                    break;
                case 2:
                    a3dCurv = surf.VIso(vl);
                    break;
                case 3:
                    a3dCurv = surf.UIso(uf);
                    break;
                default:
                    throw new OutOfRangeException("FS_Edge::CreateTopologicalEdge()");
            }

            builder.MakeEdge(ref myTopoEdge, a3dCurv, tol);
            builder.Add(myTopoEdge, v1);
            builder.Add(myTopoEdge, v2);
            builder.Range(myTopoEdge, a3dCurv.FirstParameter(), a3dCurv.LastParameter());
        }
    }

    /// <summary>Finds the node nearest to the given point, not farther than the tolerance.</summary>
    protected sealed class NodeInspector : ICellFilterInspector<int, Xyz>
    {
        private readonly NCollection_DynamicArray<FS_Vertex> _vecOfVertexes;
        private readonly Point3D _point;
        private double _sqToler;
        private int _resID;

        public static int Dimension => 3;

        public static double Coord(int i, Xyz pnt) => pnt.Coord(i + 1);

        public static Xyz Shift(in Xyz pnt, double tol) =>
            new(pnt.X() + tol, pnt.Y() + tol, pnt.Z() + tol);

        //=================================================================================================

        public NodeInspector(NCollection_DynamicArray<FS_Vertex> vec, in Point3D pnt, double tol)
        {
            _vecOfVertexes = vec;
            _point = pnt;
            _resID = -1;
            _sqToler = tol * tol;
        }

        //=================================================================================================

        public CellFilterAction Inspect(int id)
        {
            Point3D pt = _vecOfVertexes.Value(id).myPnt;
            double sqDist = pt.SquareDistance(_point);
            if (sqDist < _sqToler)
            {
                _sqToler = sqDist;
                _resID = id;
            }

            return CellFilterAction.CellFilter_Keep;
        }

        public int GetResult() => _resID;
    }
}
