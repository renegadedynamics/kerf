// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_SimpleOffset.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_SimpleOffset.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The modification of the simple offset algorithm: the topology-preserving offset, without intersection.
/// </summary>
/// <remarks>
/// <para>The mapping scheme:</para>
/// <list type="bullet">
/// <item><description>each surface is mapped to its geometric offset surface;</description></item>
/// <item><description>for each edge, the pcurves are mapped to the same pcurves on the offset surfaces;</description></item>
/// <item><description>for each edge, the 3D curve is rebuilt from its pcurve on the first offset face;</description></item>
/// <item><description>each vertex is placed at the average of the ends of the offset edges sharing it;</description></item>
/// <item><description>the tolerances are updated to the resulting geometry.</description></item>
/// </list>
/// <para>
/// It reads the input shape only: everything is computed in the constructor, on new edges of its own. The maps are
/// keyed by the shapes with their orientations (OCCT's default hasher), so a shared edge has an image only in the
/// orientation in which it was first met.
/// </para>
/// </remarks>
public sealed class BRepOffset_SimpleOffset : BRepTools_Modification
{
    private struct NewFaceData
    {
        public Geom_Surface? myOffsetS;
        public TopLoc_Location myL;
        public double myTol;
        public bool myRevWires;
        public bool myRevFace;
    }

    private struct NewEdgeData
    {
        public Geom_Curve? myOffsetC; // Resulting curve.
        public TopLoc_Location myL;
        public double myTol;
    }

    private struct NewVertexData
    {
        public gp_Pnt myP;
        public double myTol;
    }

    // Map of faces to new faces information.
    private readonly NCollection_DataMap<TopoDS_Face, NewFaceData> myFaceInfo =
        new(EqualityComparer<TopoDS_Face>.Default); // the default hasher of OCCT (IsEqual)

    // Map of edges to new edges information.
    private readonly NCollection_DataMap<TopoDS_Edge, NewEdgeData> myEdgeInfo =
        new(EqualityComparer<TopoDS_Edge>.Default); // the default hasher of OCCT (IsEqual)

    // Map of vertices to new vertices information.
    private readonly NCollection_DataMap<TopoDS_Vertex, NewVertexData> myVertexInfo =
        new(EqualityComparer<TopoDS_Vertex>.Default); // the default hasher of OCCT (IsEqual)

    // Offset value.
    private readonly double myOffsetValue;

    // Tolerance.
    private readonly double myTolerance;

    //=================================================================================================

    /// <summary>
    /// Computes the offset of <paramref name="theInputShape"/> by the signed distance
    /// <paramref name="theOffsetValue"/>; <paramref name="theTolerance"/> is the tolerance for handling singular
    /// points.
    /// </summary>
    public BRepOffset_SimpleOffset(in TopoDS_Shape theInputShape, double theOffsetValue, double theTolerance)
    {
        myOffsetValue = theOffsetValue;
        myTolerance = theTolerance;
        FillOffsetData(theInputShape);
    }

    //=================================================================================================

    /// <summary>
    /// True if the face has been modified: <paramref name="S"/> is its offset surface, with the identity location.
    /// </summary>
    public override bool NewSurface(in TopoDS_Face F,
                                    out Geom_Surface? S,
                                    out TopLoc_Location L,
                                    out double Tol,
                                    out bool RevWires,
                                    out bool RevFace)
    {
        if (!myFaceInfo.IsBound(F))
        {
            // OCCT.NET: OCCT leaves the outputs unset
            S = null;
            L = new TopLoc_Location();
            Tol = 0.0;
            RevWires = RevFace = false;
            return false;
        }

        NewFaceData aNFD = myFaceInfo.Find(F);

        S = aNFD.myOffsetS;
        L = aNFD.myL;
        Tol = aNFD.myTol;
        RevWires = aNFD.myRevWires;
        RevFace = aNFD.myRevFace;

        return true;
    }

    //=================================================================================================

    /// <summary>True if the edge has been modified: <paramref name="C"/> is its new 3D curve.</summary>
    public override bool NewCurve(in TopoDS_Edge E, out Geom_Curve? C, out TopLoc_Location L, out double Tol)
    {
        if (!myEdgeInfo.IsBound(E))
        {
            // OCCT.NET: OCCT leaves the outputs unset
            C = null;
            L = new TopLoc_Location();
            Tol = 0.0;
            return false;
        }

        NewEdgeData aNED = myEdgeInfo.Find(E);

        C = aNED.myOffsetC;
        L = aNED.myL;
        Tol = aNED.myTol;

        return true;
    }

    //=================================================================================================

    /// <summary>True if the vertex has been modified: <paramref name="P"/> is its new point.</summary>
    public override bool NewPoint(in TopoDS_Vertex V, out gp_Pnt P, out double Tol)
    {
        if (!myVertexInfo.IsBound(V))
        {
            // OCCT.NET: OCCT leaves the outputs unset
            P = new gp_Pnt();
            Tol = 0.0;
            return false;
        }

        NewVertexData aNVD = myVertexInfo.Find(V);

        P = aNVD.myP;
        Tol = aNVD.myTol;

        return true;
    }

    //=================================================================================================

    /// <summary>Always true: the edge keeps its pcurve on the face (on the offset surface).</summary>
    public override bool NewCurve2d(in TopoDS_Edge E,
                                    in TopoDS_Face F,
                                    in TopoDS_Edge NewE,
                                    in TopoDS_Face NewF,
                                    out Geom2d_Curve? C,
                                    out double Tol)
    {
        // Use original pcurve.
        C = BRep_Tool.CurveOnSurface(E, F, out double aF, out double aL);
        Tol = BRep_Tool.Tolerance(E);

        if (myEdgeInfo.IsBound(E))
        {
            Tol = myEdgeInfo.Find(E).myTol;
        }

        return true;
    }

    //=================================================================================================

    /// <summary>Always true: the vertex keeps its parameter on the edge.</summary>
    public override bool NewParameter(in TopoDS_Vertex V, in TopoDS_Edge E, ref double P, ref double Tol)
    {
        // Use original parameter.
        P = BRep_Tool.Parameter(V, E);
        Tol = BRep_Tool.Tolerance(V);

        if (myVertexInfo.IsBound(V))
        {
            Tol = myVertexInfo.Find(V).myTol;
        }

        return true;
    }

    //=================================================================================================

    /// <summary>The continuity of the original edge between the original faces.</summary>
    public override GeomAbs_Shape Continuity(in TopoDS_Edge E,
                                             in TopoDS_Face F1,
                                             in TopoDS_Face F2,
                                             in TopoDS_Edge NewE,
                                             in TopoDS_Face NewF1,
                                             in TopoDS_Face NewF2)
    {
        // Compute result using original continuity.
        return BRep_Tool.Continuity(E, F1, F2);
    }

    //=================================================================================================

    // Fills offset data.
    private void FillOffsetData(in TopoDS_Shape theShape)
    {
        // Clears old data.
        myFaceInfo.Clear();
        myEdgeInfo.Clear();
        myVertexInfo.Clear();

        // Faces loop. Compute offset surface for each face.
        var anExpSF = new TopExp_Explorer(theShape, TopAbs_FACE);
        for (; anExpSF.More(); anExpSF.Next())
        {
            TopoDS_Face aCurrFace = TopoDS.Face(anExpSF.Current());
            FillFaceData(aCurrFace);
        }

        // Iterate over edges to compute 3d curve.
        var aEdgeFaceMap =
            new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(theShape, TopAbs_EDGE, TopAbs_FACE, aEdgeFaceMap);
        for (int anIdx = 1; anIdx <= aEdgeFaceMap.Extent(); ++anIdx)
        {
            TopoDS_Edge aCurrEdge = TopoDS.Edge(aEdgeFaceMap.FindKey(anIdx));
            FillEdgeData(aCurrEdge, aEdgeFaceMap, anIdx);
        }

        // Iterate over vertices to compute new vertex.
        var aVertexEdgeMap =
            new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(theShape, TopAbs_VERTEX, TopAbs_EDGE, aVertexEdgeMap);
        for (int anIdx = 1; anIdx <= aVertexEdgeMap.Extent(); ++anIdx)
        {
            TopoDS_Vertex aCurrVertex = TopoDS.Vertex(aVertexEdgeMap.FindKey(anIdx));
            FillVertexData(aCurrVertex, aVertexEdgeMap, anIdx);
        }
    }

    //=================================================================================================

    // Method to fill new face data for single face.
    private void FillFaceData(in TopoDS_Face theFace)
    {
        NewFaceData aNFD = new();
        aNFD.myRevWires = false;
        aNFD.myRevFace = false;
        aNFD.myTol = BRep_Tool.Tolerance(theFace);

        // Create offset surface.

        // Any existing transformation is applied to the surface.
        // New face will have null transformation.
        Geom_Surface aS = BRep_Tool.Surface(theFace)!;
        aS = BRepOffset.CollapseSingularities(aS, theFace, myTolerance);

        // Take into account face orientation.
        double aMult = 1.0;
        if (theFace.Orientation() == TopAbs_REVERSED)
        {
            aMult = -1.0;
        }

        // aStatus is set by BRepOffset::Surface(), could be used to check result...
        aNFD.myOffsetS = BRepOffset.Surface(aS, aMult * myOffsetValue, out BRepOffset_Status aStatus, true);
        aNFD.myL = new TopLoc_Location(); // Null transformation.

        // Save offset surface in map.
        myFaceInfo.Bind(theFace, aNFD);
    }

    //=================================================================================================

    // Method to fill new edge data for single edge.
    private void FillEdgeData(in TopoDS_Edge theEdge,
                              NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEdgeFaceMap,
                              int theIdx)
    {
        NCollection_List<TopoDS_Shape> aFacesList = theEdgeFaceMap[theIdx];

        if (aFacesList.Size() == 0)
        {
            return; // Free edges are skipped.
        }

        // Get offset surface.
        TopoDS_Face aCurrFace = TopoDS.Face(aFacesList.First());

        if (!myFaceInfo.IsBound(aCurrFace))
        {
            return;
        }

        // No need to deal with transformation - it is applied in fill faces data method.
        NewFaceData aNFD = myFaceInfo.Find(aCurrFace);
        Geom_Surface anOffsetSurf = aNFD.myOffsetS!;

        // Compute offset 3d curve.
        Geom2d_Curve aC2d = BRep_Tool.CurveOnSurface(theEdge, aCurrFace, out double aF, out double aL)!;

        var anEdgeMaker = new BRepBuilderAPI_MakeEdge(aC2d, anOffsetSurf, aF, aL);
        TopoDS_Edge aNewEdge = anEdgeMaker.InternalEdge();

        // Compute max tolerance. Vertex tolerance usage is taken from existing offset computation
        // algorithm. This piece of code significantly influences resulting performance.
        double aTol = BRep_Tool.MaxTolerance(theEdge, TopAbs_VERTEX);
        BRepLib.BuildCurves3d(aNewEdge, aTol);

        NewEdgeData aNED = new();
        aNED.myOffsetC = BRep_Tool.Curve(aNewEdge, out aNED.myL, out aF, out aL);

        // Iterate over adjacent faces for the current edge and compute max deviation.
        double anEdgeTol = 0.0;
        var anIter = new NCollection_List<TopoDS_Shape>.Iterator(aFacesList);
        for (; aNED.myOffsetC is not null && anIter.More(); anIter.Next())
        {
            TopoDS_Face aCurFace = TopoDS.Face(anIter.Value());

            if (!myFaceInfo.IsBound(aCurFace))
            {
                continue;
            }

            // Create offset curve on surface.
            Geom2d_Curve aC2dNew = BRep_Tool.CurveOnSurface(theEdge, aCurFace, out aF, out aL)!;
            Adaptor2d_Curve2d aHCurve2d = new Geom2dAdaptor_Curve(aC2dNew, aF, aL);
            Adaptor3d_Surface aHSurface = new GeomAdaptor_Surface(myFaceInfo.Find(aCurFace).myOffsetS!);
            var aCurveOnSurf = new Adaptor3d_CurveOnSurface(aHCurve2d, aHSurface);

            // Extract 3d-curve (it is not null).
            Adaptor3d_Curve aCurve3d = new GeomAdaptor_Curve(aNED.myOffsetC, aF, aL);

            // It is necessary to compute maximal deviation (tolerance).
            var aValidateEdge = new BRepLib_ValidateEdge(aCurve3d, aCurveOnSurf, true);
            aValidateEdge.Process();
            if (aValidateEdge.IsDone())
            {
                double aMaxTol1 = aValidateEdge.GetMaxDistance();
                anEdgeTol = Max(anEdgeTol, aMaxTol1);
            }
        }
        aNED.myTol = Max(BRep_Tool.Tolerance(aNewEdge), anEdgeTol);

        // Save computed 3d curve in map.
        myEdgeInfo.Bind(theEdge, aNED);
    }

    //=================================================================================================

    // Method to fill new vertex data for single vertex.
    private void FillVertexData(in TopoDS_Vertex theVertex,
                                NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theVertexEdgeMap,
                                int theIdx)
    {
        // Algorithm:
        // Find adjacent edges for the given vertex.
        // Find corresponding end on the each adjacent edge.
        // Get offset points for founded end.
        // Set result vertex position as barycenter of founded points.

        gp_Pnt aCurrPnt = BRep_Tool.Pnt(theVertex);

        NCollection_List<TopoDS_Shape> aEdgesList = theVertexEdgeMap[theIdx];

        if (aEdgesList.Size() == 0)
        {
            return; // Free verices are skipped.
        }

        // Array to store offset points.
        var anOffsetPointVec = new NCollection_DynamicArray<gp_Pnt>();

        double aMaxEdgeTol = 0.0;

        // Iterate over adjacent edges.
        var anIterEdges = new NCollection_List<TopoDS_Shape>.Iterator(aEdgesList);
        for (; anIterEdges.More(); anIterEdges.Next())
        {
            TopoDS_Edge aCurrEdge = TopoDS.Edge(anIterEdges.Value());

            if (!myEdgeInfo.IsBound(aCurrEdge))
            {
                continue; // Skip shared edges with wrong orientation.
            }

            // Find the closest bound.
            Geom_Curve? aC3d = BRep_Tool.Curve(aCurrEdge, out double aF, out double aL);

            // Protection from degenerated edges.
            if (aC3d is null)
            {
                continue;
            }

            gp_Pnt aPntF = aC3d.Value(aF);
            gp_Pnt aPntL = aC3d.Value(aL);

            double aSqDistF = aPntF.SquareDistance(aCurrPnt);
            double aSqDistL = aPntL.SquareDistance(aCurrPnt);

            double aMinParam = aF, aMaxParam = aL;
            if (aSqDistL < aSqDistF)
            {
                // Square distance to last point is closer.
                aMinParam = aL;
                aMaxParam = aF;
            }

            // Compute point on offset edge.
            NewEdgeData aNED = myEdgeInfo.Find(aCurrEdge);
            Geom_Curve anOffsetCurve = aNED.myOffsetC!;
            gp_Pnt anOffsetPoint = anOffsetCurve.Value(aMinParam);
            anOffsetPointVec.Append(anOffsetPoint);

            // Handle situation when edge is closed.
            TopExp.Vertices(aCurrEdge, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
            if (aV1.IsSame(aV2))
            {
                gp_Pnt anOffsetPointLast = anOffsetCurve.Value(aMaxParam);
                anOffsetPointVec.Append(anOffsetPointLast);
            }

            aMaxEdgeTol = Max(aMaxEdgeTol, aNED.myTol);
        }

        // NCollection_DynamicArray starts from 0 by default.
        // It's better to use lower() and upper() in this case instead of direct indexes range.
        gp_Pnt aCenter = new(0.0, 0.0, 0.0);
        for (int i = anOffsetPointVec.Lower(); i <= anOffsetPointVec.Upper(); ++i)
        {
            aCenter.SetXYZ(aCenter.XYZ() + anOffsetPointVec.Value(i).XYZ());
        }
        // OCCT: a vertex none of whose edges has an offset curve (only free or degenerated edges, or shared edges met
        // in the other orientation) divides by zero: its new point is NaN.
        aCenter.SetXYZ(aCenter.XYZ() / anOffsetPointVec.Length());

        // Compute max distance.
        double aSqMaxDist = 0.0;
        for (int i = anOffsetPointVec.Lower(); i <= anOffsetPointVec.Upper(); ++i)
        {
            double aSqDist = aCenter.SquareDistance(anOffsetPointVec.Value(i));
            if (aSqDist > aSqMaxDist)
            {
                aSqMaxDist = aSqDist;
            }
        }

        double aResTol = Max(aMaxEdgeTol, Math.Sqrt(aSqMaxDist));

        const double aMultCoeff = 1.001; // Avoid tolernace problems.
        NewVertexData aNVD = new();
        aNVD.myP = aCenter;
        aNVD.myTol = aResTol * aMultCoeff;

        // Save computed vertex info.
        myVertexInfo.Bind(theVertex, aNVD);
    }
}
