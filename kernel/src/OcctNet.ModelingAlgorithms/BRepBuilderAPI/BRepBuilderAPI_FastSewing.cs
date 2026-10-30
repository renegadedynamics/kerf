// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_FastSewing.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_FastSewing.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

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
public class BRepBuilderAPI_FastSewing : Standard_Transient
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

    private TopoDS_Shape myResShape = new();

    // myFaceVec, myVertexVec and myEdgeVec lists are filled only once!!!!!
    // Vector of faces
    private readonly NCollection_DynamicArray<FS_Face> myFaceVec = new();
    // Vector of Vertices
    private readonly NCollection_DynamicArray<FS_Vertex> myVertexVec = new();
    // Vector of edges
    private readonly NCollection_DynamicArray<FS_Edge> myEdgeVec = new();
    // Tolerance
    private double myTolerance;
    // Bits of computation status
    private uint myStatusList;

    //=======================================================================
    // function : IntersetctionOfSets
    // purpose  : Returns minimal value of intersection result
    //=======================================================================
    private static int IntersectionOfSets(NCollection_List<int> theSet1, NCollection_List<int> theSet2)
    {
        int anIntMax = IntegerLast();
        int aRetVal = anIntMax;
        for (var anIt1 = new NCollection_List<int>.Iterator(theSet1); anIt1.More(); anIt1.Next())
        {
            int aVal1 = anIt1.Value();
            for (var anIt2 = new NCollection_List<int>.Iterator(theSet2); anIt2.More(); anIt2.Next())
            {
                int aVal2 = anIt2.Value();
                if (aVal1 == aVal2)
                {
                    // theIntersectionResult.Append(aVal1);
                    if (aVal1 < aRetVal)
                    {
                        aRetVal = aVal1;
                    }
                }
            }
        }

        if (aRetVal == anIntMax)
        {
            return -1;
        }

        return aRetVal;
    }

    //=================================================================================================

    private static Geom2d_Curve Get2DCurve(int theIndex,
                                           double theUfirst,
                                           double theUlast,
                                           double theVfirst,
                                           double theVlast,
                                           bool theIsReverse = false)
    {
        if ((theIndex < 0) || (theIndex > 3))
        {
            throw new Standard_OutOfRange("BRepBuilderAPI_FastSewing.cxx, Get2DCurve(): OUT of Range");
        }

        Geom2d_Curve? a2dCurv = null;

        if (!theIsReverse)
        {
            switch (theIndex)
            {
                case 0:
                    a2dCurv = new Geom2d_TrimmedCurve(
                        new Geom2d_Line(new gp_Pnt2d(0.0, theVfirst), new gp_Dir2d(gp_Dir2d.D.X)),
                        theUfirst,
                        theUlast);
                    break;
                case 1:
                    a2dCurv = new Geom2d_TrimmedCurve(
                        new Geom2d_Line(new gp_Pnt2d(theUlast, 0.0), new gp_Dir2d(gp_Dir2d.D.Y)),
                        theVfirst,
                        theVlast);
                    break;
                case 2:
                    a2dCurv = new Geom2d_TrimmedCurve(
                        new Geom2d_Line(new gp_Pnt2d(0.0, theVlast), new gp_Dir2d(gp_Dir2d.D.X)),
                        theUfirst,
                        theUlast);
                    break;
                case 3:
                    a2dCurv = new Geom2d_TrimmedCurve(
                        new Geom2d_Line(new gp_Pnt2d(theUfirst, 0.0), new gp_Dir2d(gp_Dir2d.D.Y)),
                        theVfirst,
                        theVlast);
                    break;
                default:
                    break;
            }
        }
        else
        {
            switch (theIndex)
            {
                case 0:
                    a2dCurv = new Geom2d_TrimmedCurve(
                        new Geom2d_Line(new gp_Pnt2d(theUfirst + theUlast, theVfirst), new gp_Dir2d(gp_Dir2d.D.NX)),
                        theUfirst,
                        theUlast);
                    break;
                case 1:
                    a2dCurv = new Geom2d_TrimmedCurve(
                        new Geom2d_Line(new gp_Pnt2d(theUlast, theVfirst + theVlast), new gp_Dir2d(gp_Dir2d.D.NY)),
                        theVfirst,
                        theVlast);
                    break;
                case 2:
                    a2dCurv = new Geom2d_TrimmedCurve(
                        new Geom2d_Line(new gp_Pnt2d(theUfirst + theUlast, theVlast), new gp_Dir2d(gp_Dir2d.D.NX)),
                        theUfirst,
                        theUlast);
                    break;
                case 3:
                    a2dCurv = new Geom2d_TrimmedCurve(
                        new Geom2d_Line(new gp_Pnt2d(theUfirst, theVfirst + theVlast), new gp_Dir2d(gp_Dir2d.D.NY)),
                        theVfirst,
                        theVlast);
                    break;
                default:
                    break;
            }
        }

        return a2dCurv!;
    }

    //=================================================================================================

    /// <summary>Creates an object with the tolerance of connexity <paramref name="theTol"/>.</summary>
    public BRepBuilderAPI_FastSewing(double theTol = 1.0e-06)
    {
        myTolerance = theTol;
        myStatusList = 0;
    }

    /// <summary>Sets the tolerance.</summary>
    public void SetTolerance(double theToler) => myTolerance = theToler;

    /// <summary>The tolerance.</summary>
    public double GetTolerance() => myTolerance;

    /// <summary>The resulting shape, frozen.</summary>
    public TopoDS_Shape GetResult()
    {
        myResShape.TShape()?.FreezeGraph();
        return myResShape;
    }

    /// <summary>Sets a status; returns its numeric value.</summary>
    protected uint SetStatus(FS_Statuses theStatus)
    {
        uint aStatusID = (uint)theStatus;
        myStatusList |= aStatusID;
        return aStatusID;
    }

    //=================================================================================================

    /// <summary>Adds the faces of a shape; false if none was added.</summary>
    public bool Add(in TopoDS_Shape theShape)
    {
        bool aResult = false;
        if (theShape.IsNull())
        {
            SetStatus(FS_Statuses.FS_EmptyInput);
            return aResult;
        }

        var aMS = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        // aMS.Add(theShape);
        var aFExp = new TopExp_Explorer(theShape, TopAbs_FACE);
        for (; aFExp.More(); aFExp.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFExp.Current());
            if (aMS.Add(aFace))
            {
                Geom_Surface? aSurf = BRep_Tool.Surface(aFace);
                if (aSurf is null)
                {
                    SetStatus(FS_Statuses.FS_FaceWithNullSurface);
                    continue;
                }

                if (aSurf is Geom_RectangularTrimmedSurface)
                {
                    SetStatus(FS_Statuses.FS_NotNaturalBoundsFace);
                    continue;
                }

                aSurf.Bounds(out double aUf, out double aUl, out double aVf, out double aVl);
                if (Precision.IsInfinite(aUf) || Precision.IsInfinite(aUl) || Precision.IsInfinite(aVf)
                    || Precision.IsInfinite(aVl))
                {
                    SetStatus(FS_Statuses.FS_InfiniteSurface);
                    continue;
                }

                var aFFace = new FS_Face();
                aFFace.mySrcFace = aFace;
                aFFace.myID = myFaceVec.Length(); // because start index is 0
                myFaceVec.Append(aFFace);
                aResult = true;
            }
        }

        return aResult;
    }

    //=================================================================================================

    /// <summary>Adds a surface; false if it can't be sewn (null, trimmed or infinite).</summary>
    public bool Add(Geom_Surface? theSurface)
    {
        if (theSurface is null)
        {
            SetStatus(FS_Statuses.FS_FaceWithNullSurface);
            return false;
        }

        if (theSurface is Geom_RectangularTrimmedSurface)
        {
            SetStatus(FS_Statuses.FS_NotNaturalBoundsFace);
            return false;
        }

        theSurface.Bounds(out double aUf, out double aUl, out double aVf, out double aVl);
        if (Precision.IsInfinite(aUf) || Precision.IsInfinite(aUl) || Precision.IsInfinite(aVf)
            || Precision.IsInfinite(aVl))
        {
            SetStatus(FS_Statuses.FS_InfiniteSurface);
            return false;
        }

        var aFace = new FS_Face();

        var aBuilder = new BRep_Builder();

        aBuilder.MakeFace(ref aFace.mySrcFace);
        aBuilder.MakeFace(ref aFace.mySrcFace, theSurface, myTolerance);
        aBuilder.NaturalRestriction(aFace.mySrcFace, true);

        aFace.myID = myFaceVec.Length(); // because start index is 0
        myFaceVec.Append(aFace);

        return true;
    }

    //=================================================================================================

    /// <summary>Computes the resulting shape.</summary>
    public void Perform()
    {
        if (myFaceVec.IsEmpty())
        {
            SetStatus(FS_Statuses.FS_EmptyInput);
            return;
        }

        try
        {
            {
                // create vertices having unique coordinates
                double aRange = Compute3DRange();
                var aCells = new NCollection_CellFilter<NodeInspector, int, gp_XYZ>(Max(myTolerance, aRange / IntegerLast()));

                for (int i = myFaceVec.Lower(); i <= myFaceVec.Upper(); i++)
                {
                    FindVertexes(i, aCells);
                }
            }

            for (int i = myFaceVec.Lower(); i <= myFaceVec.Upper(); i++)
            {
                FindEdges(i);
            }

            // Create topological structures

            for (int i = myVertexVec.Lower(); i <= myVertexVec.Upper(); i++)
            {
                myVertexVec.Value(i).CreateTopologicalVertex(myTolerance);
            }

            // Edges
            for (int i = myEdgeVec.Lower(); i <= myEdgeVec.Upper(); i++)
            {
                myEdgeVec.Value(i).CreateTopologicalEdge(myVertexVec, myFaceVec, myTolerance);
            }

            // Shell
            var aQuilt = new BRepTools_Quilt();

            // Faces
            for (int i = myFaceVec.Lower(); i <= myFaceVec.Upper(); i++)
            {
                FS_Face aFace = myFaceVec.Value(i);
                aFace.CreateTopologicalWire(myEdgeVec, myTolerance);

                aFace.CreateTopologicalFace();
                aQuilt.Add(aFace.myRetFace);
            }

            myResShape = aQuilt.InternalShells();
        }
        catch (Standard_Failure)
        {
            SetStatus(FS_Statuses.FS_Exception);
            return;
        }
    }

    //=================================================================================================

    /// <summary>Updates the information of an existing edge.</summary>
    protected void UpdateEdgeInfo(int theIDPrevVertex, int theIDCurrVertex, int theFaceID, int theIDCurvOnFace)
    {
        // Indeed, two vertices combine into one edge only.
        int anEdgeID = IntersectionOfSets(myVertexVec.Value(theIDPrevVertex).myEdges,
                                          myVertexVec.Value(theIDCurrVertex).myEdges);

        // For DEBUG mode only
        Standard_ProgramError_Raise_if(anEdgeID < 0,
                                       "BRepBuilderAPI_FastSewing::UpdateEdgeInfo: Update not existing edge.");

        FS_Edge anEdge = myEdgeVec.Value(anEdgeID);
        anEdge.myFaces.Append(theFaceID);
        FS_Face aFace = myFaceVec.Value(theFaceID);
        aFace.SetEdge(theIDCurvOnFace, anEdge.myID);
    }

    //=================================================================================================

    /// <summary>Creates a new edge.</summary>
    protected void CreateNewEdge(int theIDPrevVertex, int theIDCurrVertex, int theFaceID, int theIDCurvOnFace)
    {
        var anEdge = new FS_Edge(theIDPrevVertex, theIDCurrVertex);
        anEdge.myID = myEdgeVec.Length(); // because start index is 0

        anEdge.myFaces.Append(theFaceID);
        FS_Face aFace = myFaceVec.Value(theFaceID);
        aFace.SetEdge(theIDCurvOnFace, anEdge.myID);

        myVertexVec.Value(theIDPrevVertex).myEdges.Append(anEdge.myID);

        if (theIDPrevVertex == theIDCurrVertex)
        { // the Edge is degenerated
            SetStatus(FS_Statuses.FS_Degenerated);
        }
        else
        {
            myVertexVec.Value(theIDCurrVertex).myEdges.Append(anEdge.myID);
        }

        myEdgeVec.Append(anEdge);
    }

    //=================================================================================================

    /// <summary>Finds or creates the vertices of the corners of a face.</summary>
    protected void FindVertexes(int theSurfID, NCollection_CellFilter<NodeInspector, int, gp_XYZ> theCells)
    {
        const int aNbPoints = 4;
        FS_Face aFace = myFaceVec.Value(theSurfID);
        Geom_Surface aSurf = BRep_Tool.Surface(aFace.mySrcFace)!;
        aSurf.Bounds(out double aUf, out double aUl, out double aVf, out double aVl);

        gp_Pnt[] aPnts = [aSurf.Value(aUf, aVf),
                          aSurf.Value(aUl, aVf),
                          aSurf.Value(aUl, aVl),
                          aSurf.Value(aUf, aVl)];

        for (int i = 0; i < aNbPoints; i++)
        {
            var aVert = new FS_Vertex();

            var anInspector = new NodeInspector(myVertexVec, aPnts[i], myTolerance);
            var aBox = new Bnd_Box();
            aBox.Add(aPnts[i]);
            aBox.Enlarge(myTolerance);

            theCells.Inspect(aBox.CornerMin().XYZ(), aBox.CornerMax().XYZ(), ref anInspector);
            int aResID = anInspector.GetResult();

            if (aResID < 0)
            {                                     // Add new Vertex
                aVert.myID = myVertexVec.Length(); // because start index is 0
                aVert.myPnt = aPnts[i];
                aVert.myFaces.Append(theSurfID);
                myVertexVec.Append(aVert);
                aFace.SetVertex(i, aVert.myID);

                theCells.Add(aVert.myID, aBox.CornerMin().XYZ(), aBox.CornerMax().XYZ());
            }
            else
            { // Change existing vertex
                aFace.SetVertex(i, aResID);
                myVertexVec.Value(aResID).myFaces.Append(theSurfID);
            }
        }
    }

    //=================================================================================================

    /// <summary>Finds or creates the edges of a face.</summary>
    protected void FindEdges(int theSurfID)
    {
        const int aNbPoints = 4;
        FS_Face aFace = myFaceVec.Value(theSurfID);

        ReadOnlySpan<int> aFirstInd = [0, 1, 3, 0];
        ReadOnlySpan<int> aLastInd = [1, 2, 2, 3];

        for (int i = 0; i < aNbPoints; i++)
        {
            int aFirstVertIndex = aFirstInd[i], aLastVertIndex = aLastInd[i];
            int aFirstVertID = aFace.myVertices[aFirstVertIndex],
                aLastVertID = aFace.myVertices[aLastVertIndex];

            if (aFirstVertID == aLastVertID)
            { // Edge is degenerated.
                CreateNewEdge(aFirstVertID, aLastVertID, theSurfID, i);
                continue;
            }

            // Must be minimal element from list
            int anIntRes = IntersectionOfSets(myVertexVec.Value(aFirstVertID).myFaces,
                                              myVertexVec.Value(aLastVertID).myFaces);

            if ((anIntRes < 0) || (anIntRes >= theSurfID))
            {
                CreateNewEdge(aFirstVertID, aLastVertID, theSurfID, i);
            }
            else
            { // if(theSurfID > anIntRes) => The edge has been processed earlier
                UpdateEdgeInfo(aFirstVertID, aLastVertID, theSurfID, i);
            }
        }
    }

    //=================================================================================================

    /// <summary>The bits of the statuses; prints a message for each to <paramref name="theOS"/> if given.</summary>
    public uint GetStatuses(TextWriter? theOS = null)
    {
        if (theOS is null)
        {
            return myStatusList;
        }

        if (myStatusList == 0)
        {
            theOS.Write("Fast Sewing OK!\n");
            return myStatusList;
        }

        // Number of bits
        const int aNumMax = 8 * sizeof(uint);
        var anIDS = (FS_Statuses)0x0001;
        for (int i = 1; i <= aNumMax; i++, anIDS = (FS_Statuses)((uint)anIDS << 1))
        {
            if (((uint)anIDS & myStatusList) == 0)
            {
                continue;
            }

            switch (anIDS)
            {
                case FS_Statuses.FS_Degenerated:
                    theOS.Write("Degenerated case. Try to reduce tolerance.\n");
                    break;
                case FS_Statuses.FS_FindVertexError:
                    theOS.Write("Error while creating list of vertices.\n");
                    break;
                case FS_Statuses.FS_FindEdgeError:
                    theOS.Write("Error while creating list of edges.\n");
                    break;
                case FS_Statuses.FS_Exception:
                    theOS.Write("Exception during the operation.\n");
                    break;
                case FS_Statuses.FS_FaceWithNullSurface:
                    theOS.Write("Source face has null surface.\n");
                    break;
                case FS_Statuses.FS_NotNaturalBoundsFace:
                    theOS.Write("Source face has trimmed surface.\n");
                    break;
                case FS_Statuses.FS_InfiniteSurface:
                    theOS.Write("Source face has the surface with infinite boundaries.\n");
                    break;
                case FS_Statuses.FS_EmptyInput:
                    theOS.Write("Empty source data.\n");
                    break;
                default:
                    return myStatusList;
            }
        }

        return myStatusList;
    }

    //=================================================================================================

    /// <summary>The largest extent of the box of the faces' corners.</summary>
    protected double Compute3DRange()
    {
        var aBox = new Bnd_Box();

        for (int i = myFaceVec.Lower(); i <= myFaceVec.Upper(); i++)
        {
            FS_Face aFace = myFaceVec.Value(i);
            Geom_Surface? aSurf = BRep_Tool.Surface(aFace.mySrcFace);

            if (aSurf is null)
            {
                continue;
            }

            aSurf.Bounds(out double aUf, out double aUl, out double aVf, out double aVl);

            aBox.Add(aSurf.Value(aUf, aVf));
            aBox.Add(aSurf.Value(aUl, aVf));
            aBox.Add(aSurf.Value(aUl, aVl));
            aBox.Add(aSurf.Value(aUf, aVl));
        }

        aBox.Get(out double aXm, out double aYm, out double aZm, out double aXM, out double aYM, out double aZM);
        double aDelta = aXM - aXm;
        aDelta = Max(aDelta, aYM - aYm);
        aDelta = Max(aDelta, aZM - aZm);

        return aDelta;
    }

    // Classes FS_Vertex, FS_Face and FS_Edge keep information about
    // relations between resulted members (e.g. which faces share this vertex? etc.)
    // (Structs in OCCT, held in vectors and changed in place there; classes here, so that the vectors' elements are
    // changed in place too.)

    /// <summary>A vertex of the result.</summary>
    protected sealed class FS_Vertex
    {
        /// <summary>Geometry point of this Vertex</summary>
        public gp_Pnt myPnt = new();
        public TopoDS_Vertex myTopoVert = new();
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
        public void CreateTopologicalVertex(double theToler)
        {
            var aBuilder = new BRep_Builder();
            aBuilder.MakeVertex(ref myTopoVert, myPnt, theToler);
        }
    }

    /// <summary>A face of the result.</summary>
    protected sealed class FS_Face
    {
        public TopoDS_Face mySrcFace = new();
        public TopoDS_Wire myWire = new();
        public TopoDS_Face myRetFace = new();
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
        public void SetVertex(int thePlaceID, int theVertID)
        {
            Standard_RangeError_Raise_if((thePlaceID < 0) || (thePlaceID > 3), "FS_Face::SetVertex(): OUT of Range");
            myVertices[thePlaceID] = theVertID;
        }

        /// <summary>Sets edge</summary>
        public void SetEdge(int thePlaceID, int theEdgeID)
        {
            Standard_RangeError_Raise_if((thePlaceID < 0) || (thePlaceID > 3), "FS_Face::SetEdge(): OUT of Range");
            myEdges[thePlaceID] = theEdgeID;
        }

        //=================================================================================================

        /// <summary>Creates topological members (wire)</summary>
        public void CreateTopologicalWire(NCollection_DynamicArray<FS_Edge> theEdgeVec, double theToler)
        {
            // 3D-curves in 1st and 2nd faces are considered to be in same-range
            Geom_Surface aSurf = BRep_Tool.Surface(mySrcFace, out TopLoc_Location aLocation)!;

            aSurf.Bounds(out double aUf, out double aUl, out double aVf, out double aVl);

            var aB = new BRep_Builder();
            aB.MakeWire(ref myWire);
            for (int anEdge = 0; anEdge < 4; anEdge++)
            {
                Standard_ProgramError_Raise_if(
                    myEdges[anEdge] < 0,
                    "BRepBuilderAPI_FastSewing::FS_Face::CreateTopologicalWire: Wire is not closed.");

                FS_Edge aFSEdge = theEdgeVec.Value(myEdges[anEdge]);
                TopAbs_Orientation anOri = anEdge < 2 ? TopAbs_FORWARD : TopAbs_REVERSED;
                TopoDS_Edge anTopE = aFSEdge.myTopoEdge;

                if (aFSEdge.IsDegenerated())
                {
                    anTopE.Orientation(anOri);
                    aB.Add(myWire, anTopE);
                    continue;
                }

                // Check if 3D and 2D-curve have same-orientation.
                // If it is not, 2d-curve will be reversed.
                {
                    Geom_Curve a3dCurv = BRep_Tool.Curve(anTopE, out double aFirstPar, out double aLastPar)!;
                    Geom2d_Curve a2dCurv = Get2DCurve(anEdge, aUf, aUl, aVf, aVl);
                    gp_Pnt aPref = a3dCurv.Value(aFirstPar);
                    gp_Pnt2d aP2df = a2dCurv.Value(aFirstPar), aP2dl = a2dCurv.Value(aLastPar);
                    gp_Pnt aP3df = aSurf.Value(aP2df.X(), aP2df.Y());
                    gp_Pnt aP3dl = aSurf.Value(aP2dl.X(), aP2dl.Y());

                    aP3df.Transform(aLocation.Transformation());
                    aP3dl.Transform(aLocation.Transformation());

                    double aSqD1 = aP3df.SquareDistance(aPref);
                    double aSqD2 = aP3dl.SquareDistance(aPref);

                    if (aSqD2 < aSqD1)
                    {
                        a2dCurv = Get2DCurve(anEdge, aUf, aUl, aVf, aVl, true);
                        anOri = TopAbs.Reverse(anOri);
                    }

                    aB.UpdateEdge(anTopE, a2dCurv, aSurf, aLocation, theToler);
                }

                anTopE.Orientation(anOri);

                aB.Add(myWire, anTopE);
            }

            myWire.Closed(true);
        }

        //=================================================================================================

        /// <summary>Creates topological members (face)</summary>
        public void CreateTopologicalFace()
        {
            Standard_ProgramError_Raise_if(
                myWire.IsNull(),
                "BRepBuilderAPI_FastSewing::FS_Face::CreateTopologicalFace: Cannot create wire.");

            var aBuilder = new BRep_Builder();
            myRetFace = TopoDS.Face(mySrcFace.EmptyCopied());
            aBuilder.Add(myRetFace, myWire);
            aBuilder.NaturalRestriction(myRetFace, true);
        }
    }

    /// <summary>An edge of the result.</summary>
    protected sealed class FS_Edge
    {
        /// <summary>List of faces which are shared with this edge; values are indexes in myFaceVec</summary>
        public readonly NCollection_Sequence<int> myFaces = new();
        /// <summary>Identifies the place of this Edge in BRepBuilderAPI_FastSewing::myEdgeVec list</summary>
        public int myID;
        public TopoDS_Edge myTopoEdge = new();
        // Index of the vertex in myVertexVec array
        private readonly int[] myVertices = new int[2];

        public FS_Edge()
        {
            myID = -1;
            myVertices[0] = -1;
            myVertices[1] = -1;
        }

        public FS_Edge(int theIDVert1, int theIDVert2)
        {
            myID = -1;
            myVertices[0] = theIDVert1;
            myVertices[1] = theIDVert2;
        }

        /// <summary>Sets vertex</summary>
        public void SetVertex(int thePlaceID, int theVertID)
        {
            Standard_RangeError_Raise_if((thePlaceID < 0) || (thePlaceID > 1), "FS_Face::SetVertex(): OUT of Range");
            myVertices[thePlaceID] = theVertID;
        }

        public bool IsDegenerated() => (myVertices[0] == myVertices[1]);

        //=================================================================================================

        /// <summary>Creates topological member (TopoDS_Edge)</summary>
        public void CreateTopologicalEdge(NCollection_DynamicArray<FS_Vertex> theVertexVec,
                                          NCollection_DynamicArray<FS_Face> theFaceVec,
                                          double theTol)
        {
            var aBuilder = new BRep_Builder();

            TopoDS_Vertex aV1 = theVertexVec.Value(myVertices[0]).myTopoVert;
            TopoDS_Vertex aV2 = theVertexVec.Value(myVertices[1]).myTopoVert;

            aV1.Orientation(TopAbs_FORWARD);
            aV2.Orientation(TopAbs_REVERSED);

            Geom_Curve a3dCurv;

            FS_Face aFace = theFaceVec.Value(myFaces.Value(NCollection_Sequence<int>.Lower()));

            // 3D-curves in 1st and 2nd faces are considered to be in same-range
            Geom_Surface aSurf = BRep_Tool.Surface(aFace.mySrcFace, out TopLoc_Location aLocation)!;

            aSurf.Bounds(out double aUf, out double aUl, out double aVf, out double aVl);

            int anEdgeID = -1;
            for (int anInd = 0; anInd < 4; anInd++)
            {
                if (myID == aFace.myEdges[anInd])
                {
                    anEdgeID = anInd;
                    break;
                }
            }

            // For DEBUG mode only
            Standard_ProgramError_Raise_if(anEdgeID < 0,
                                           "BRepBuilderAPI_FastSewing::FS_Edge::CreateTopologicalEdge: Single edge.");

            if (IsDegenerated())
            {
                Geom2d_Curve a2dCurv = Get2DCurve(anEdgeID, aUf, aUl, aVf, aVl);
                double aFPar = a2dCurv.FirstParameter(), aLPar = a2dCurv.LastParameter();

                aBuilder.MakeEdge(ref myTopoEdge);
                aBuilder.UpdateEdge(myTopoEdge, a2dCurv, aSurf, aLocation, theTol);
                aBuilder.Add(myTopoEdge, aV1);
                aBuilder.Add(myTopoEdge, aV2);
                aBuilder.Range(myTopoEdge, aFPar, aLPar);
                aBuilder.Degenerated(myTopoEdge, true);
                return;
            }

            switch (anEdgeID)
            {
                case 0:
                    a3dCurv = aSurf.VIso(aVf);
                    break;
                case 1:
                    a3dCurv = aSurf.UIso(aUl);
                    break;
                case 2:
                    a3dCurv = aSurf.VIso(aVl);
                    break;
                case 3:
                    a3dCurv = aSurf.UIso(aUf);
                    break;
                default:
                    throw new Standard_OutOfRange("FS_Edge::CreateTopologicalEdge()");
            }

            aBuilder.MakeEdge(ref myTopoEdge, a3dCurv, theTol);
            aBuilder.Add(myTopoEdge, aV1);
            aBuilder.Add(myTopoEdge, aV2);
            aBuilder.Range(myTopoEdge, a3dCurv.FirstParameter(), a3dCurv.LastParameter());
        }
    }

    /// <summary>Finds the node nearest to the given point, not farther than the tolerance.</summary>
    protected sealed class NodeInspector : INCollection_CellFilter_Inspector<int, gp_XYZ>
    {
        private readonly NCollection_DynamicArray<FS_Vertex> myVecOfVertexes;
        private readonly gp_Pnt myPoint;
        private double mySQToler;
        private int myResID;

        public static int Dimension => 3;

        public static double Coord(int i, gp_XYZ thePnt) => thePnt.Coord(i + 1);

        public static gp_XYZ Shift(in gp_XYZ thePnt, double theTol) =>
            new(thePnt.X() + theTol, thePnt.Y() + theTol, thePnt.Z() + theTol);

        //=================================================================================================

        public NodeInspector(NCollection_DynamicArray<FS_Vertex> theVec, in gp_Pnt thePnt, double theTol)
        {
            myVecOfVertexes = theVec;
            myPoint = thePnt;
            myResID = -1;
            mySQToler = theTol * theTol;
        }

        //=================================================================================================

        public NCollection_CellFilter_Action Inspect(int theID)
        {
            gp_Pnt aPt = myVecOfVertexes.Value(theID).myPnt;
            double aSQDist = aPt.SquareDistance(myPoint);
            if (aSQDist < mySQToler)
            {
                mySQToler = aSQDist;
                myResID = theID;
            }

            return NCollection_CellFilter_Action.CellFilter_Keep;
        }

        public int GetResult() => myResID;
    }
}
