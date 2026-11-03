// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeSimpleOffset.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeSimpleOffset.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepOffsetSimple_Status;
using static Kerf.Geometry.Continuity;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The status of <see cref="BRepOffset_MakeSimpleOffset"/>.</summary>
public enum BRepOffsetSimple_Status
{
    BRepOffsetSimple_OK,
    BRepOffsetSimple_NullInputShape,
    BRepOffsetSimple_ErrorOffsetComputation,
    BRepOffsetSimple_ErrorWallFaceComputation,
    BRepOffsetSimple_ErrorInvalidNbShells,
    BRepOffsetSimple_ErrorNonClosedShell,
}

/// <summary>
/// The simple offset algorithm: builds the offset of a shape without intersection, as a shell, or as a solid closed by
/// wall faces between the free boundaries of the input and their images (<see cref="SetBuildSolidFlag"/>).
/// </summary>
/// <remarks>
/// <para>Algorithm:</para>
/// <list type="number">
/// <item><description>Build source-image maps for vertices, edges and faces (<see cref="BRepOffset_SimpleOffset"/>).
/// The image of a shared edge is built from the corresponding edge of the first face met.</description></item>
/// <item><description>Run <see cref="BRepTools_Modifier"/> to obtain the offset shape.</description></item>
/// <item><description>Ensure the topological integrity of the output shape.</description></item>
/// </list>
/// <para>
/// The result depends on the smoothness of the input: a smooth (G1) input gives a good result. In general the
/// tolerances grow, to cover the gaps between neighbouring faces in the output, with the offset distance and the
/// quality of the joints between the input faces.
/// </para>
/// <para>
/// The modifier, in its immutable-input mode, is the copy of the input (decision 973): the offset shell never writes
/// into the caller's shape. Building the solid does: the walls give the input's free edges pcurves (in place, as
/// OCCT), and <c>ShapeFix_Edge</c> makes them same-parameter. So, with the solid flag, a frozen input is copied at
/// the entry (<see cref="BRepTools_CopyAtEntry"/>, with the healing context); the algorithm runs on the copy, and the
/// copied sub-shapes it left unchanged are the caller's own again in the result. <see cref="Generated"/> and
/// <see cref="Modified"/> answer for the caller's sub-shapes; <see cref="Modified"/> of a sub-shape changed on its copy
/// is that changed copy.
/// </para>
/// <para>
/// <see cref="GetResultShape"/>, <see cref="Generated"/> and <see cref="Modified"/> freeze what they return (a
/// publication point); ported code reads the <c>Internal</c> forms.
/// </para>
/// </remarks>
public sealed class BRepOffset_MakeSimpleOffset
{
    // Input data.
    // Input shape.
    private TopoDS_Shape myInputShape = new();
    // Offset value.
    private double myOffsetValue;
    // Tolerance (for singularities)
    private double myTolerance;
    // Solid building flag. True means solid construction.
    private bool myIsBuildSolid;

    // Internal data.
    // Maximal angle in faces junction. This value helps to estimate result tolerance.
    private double myMaxAngle;
    // Error message.
    private BRepOffsetSimple_Status myError;
    // Done state.
    private bool myIsDone;
    // Map of vertex - wall edge.
    // Used to build shared edge between adjacent wall faces.
    private readonly NCollection_DataMap<TopoDS_Vertex, TopoDS_Edge> myMapVE =
        new(EqualityComparer<TopoDS_Vertex>.Default); // the default hasher of OCCT (IsEqual)
    // Used for histrory support.
    private readonly BRepTools_Modifier myBuilder = new();
    // Used for history support.
    private readonly ShapeBuild_ReShape myReShape;

    // Output data.
    // Result shape.
    private TopoDS_Shape myResShape = new();

    // OCCT.NET: the shape the algorithm works on (the input, or its copy at the entry), and the copy at the entry.
    private TopoDS_Shape myShape = new();
    private BRepTools_CopyAtEntry? myEntry;

    //=================================================================================================

    /// <summary>Constructor. Does nothing.</summary>
    public BRepOffset_MakeSimpleOffset()
    {
        myOffsetValue = 0.0;
        myTolerance = global::Kerf.Engine.Foundation.Precision.Confusion();
        myIsBuildSolid = false;
        myMaxAngle = 0.0;
        myError = BRepOffsetSimple_OK;
        myIsDone = false;
        myReShape = new ShapeBuild_ReShape();
    }

    //=================================================================================================

    /// <summary>Prepares the offset of <paramref name="theInputShape"/> by <paramref name="theOffsetValue"/>.</summary>
    public BRepOffset_MakeSimpleOffset(in TopoDS_Shape theInputShape, double theOffsetValue)
    {
        myInputShape = theInputShape;
        myOffsetValue = theOffsetValue;
        myTolerance = global::Kerf.Engine.Foundation.Precision.Confusion();
        myIsBuildSolid = false;
        myMaxAngle = 0.0;
        myError = BRepOffsetSimple_OK;
        myIsDone = false;
        myReShape = new ShapeBuild_ReShape();
    }

    //=================================================================================================

    /// <summary>Initialises the shape to offset and the offset value.</summary>
    public void Initialize(in TopoDS_Shape theInputShape, double theOffsetValue)
    {
        myInputShape = theInputShape;
        myOffsetValue = theOffsetValue;
        Clear();
    }

    //=================================================================================================

    /// <summary>The error message.</summary>
    public string GetErrorMessage()
    {
        string anError = "";

        if (myError == BRepOffsetSimple_NullInputShape)
        {
            anError = "Null input shape";
            return anError;
        }
        else if (myError == BRepOffsetSimple_ErrorOffsetComputation)
        {
            anError = "Error during offset construction";
            return anError;
        }
        else if (myError == BRepOffsetSimple_ErrorWallFaceComputation)
        {
            anError = "Error during building wall face";
            return anError;
        }
        else if (myError == BRepOffsetSimple_ErrorInvalidNbShells)
        {
            anError = "Result contains two or more shells";
            return anError;
        }
        else if (myError == BRepOffsetSimple_ErrorNonClosedShell)
        {
            anError = "Result shell is not closed";
            return anError;
        }

        return anError;
    }

    /// <summary>The error code.</summary>
    public BRepOffsetSimple_Status GetError() => myError;

    // Inline methods.
    /// <summary>The solid building flag.</summary>
    public bool GetBuildSolidFlag() => myIsBuildSolid;

    /// <summary>Sets the solid building flag (false, the default, builds a shell).</summary>
    public void SetBuildSolidFlag(bool theBuildFlag) => myIsBuildSolid = theBuildFlag;

    /// <summary>The offset value.</summary>
    public double GetOffsetValue() => myOffsetValue;

    /// <summary>Sets the offset value.</summary>
    public void SetOffsetValue(double theOffsetValue) => myOffsetValue = theOffsetValue;

    /// <summary>The tolerance (used for handling singularities).</summary>
    public double GetTolerance() => myTolerance;

    /// <summary>Sets the tolerance (used for handling singularities).</summary>
    public void SetTolerance(double theValue) => myTolerance = theValue;

    /// <summary>True if the offset was computed.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>The result shape (frozen).</summary>
    public TopoDS_Shape GetResultShape()
    {
        myResShape.TShape()?.FreezeGraph();
        return myResShape;
    }

    /// <summary><see cref="GetResultShape"/> without freezing, for ported code.</summary>
    internal TopoDS_Shape InternalGetResultShape() => myResShape;

    //=================================================================================================

    // Clears previous result.
    private void Clear()
    {
        myIsDone = false;
        myError = BRepOffsetSimple_OK;
        myMaxAngle = 0.0;
        myMapVE.Clear();
        myReShape.Clear(); // Clear possible stored modifications.
        // OCCT.NET: the previous copy at the entry
        myEntry = null;
        myShape = myInputShape;
    }

    //=================================================================================================

    /// <summary>The maximal offset value that keeps the result's tolerance below <paramref name="theExpectedToler"/>.</summary>
    public double GetSafeOffset(double theExpectedToler)
    {
        if (myInputShape.IsNull())
        {
            return 0.0; // Input shape is null.
        }

        // Compute max angle in faces junctions.
        if (myMaxAngle == 0.0)
        { // Non-initialized.
            ComputeMaxAngle();
        }

        double aMaxTol = 0.0;
        aMaxTol = BRep_Tool.MaxTolerance(myInputShape, TopAbs_VERTEX);

        // OCCT: a shape without sharp junctions has myMaxAngle 0 (recomputed on every call): the division gives an
        // infinite (or NaN) offset
        double anExpOffset = Max((theExpectedToler - aMaxTol) / (2.0 * myMaxAngle),
                                 0.0); // Minimal distance can't be lower than 0.0.
        return anExpOffset;
    }

    //=================================================================================================

    /// <summary>Computes the offset shape.</summary>
    public void Perform()
    {
        // Clear result of previous computations.
        Clear();

        // Check shape existence.
        if (myInputShape.IsNull())
        {
            myError = BRepOffsetSimple_NullInputShape;
            return;
        }

        if (myMaxAngle == 0.0)
        { // Non-initialized.
            ComputeMaxAngle();
        }

        // OCCT.NET: copy at entry (plan 2.2): the walls of the solid write into the input's free edges
        if (myIsBuildSolid)
        {
            myEntry = new BRepTools_CopyAtEntry(myInputShape, myReShape);
            myShape = myEntry.Shape();
        }

        myBuilder.Init(myShape);
        var aMapper = new BRepOffset_SimpleOffset(myShape, myOffsetValue, myTolerance);
        myBuilder.Perform(aMapper);

        if (!myBuilder.IsDone())
        {
            myError = BRepOffsetSimple_ErrorOffsetComputation;
            finishEntry();
            return;
        }

        myResShape = myBuilder.InternalModifiedShape(myShape);

        // Fix degeneracy. Degenerated edge should be mapped to the degenerated.
        var aBB = new BRep_Builder();
        var anExpSE = new TopExp_Explorer(myShape, TopAbs_EDGE);
        for (; anExpSE.More(); anExpSE.Next())
        {
            TopoDS_Edge aCurrEdge = TopoDS.Edge(anExpSE.Current());

            if (!BRep_Tool.Degenerated(aCurrEdge))
            {
                continue;
            }

            TopoDS_Edge anEdge = TopoDS.Edge(myBuilder.InternalModifiedShape(aCurrEdge));
            aBB.Degenerated(anEdge, true);
        }

        // Restore walls for solid.
        if (myIsBuildSolid && !BuildMissingWalls())
        {
            finishEntry();
            return;
        }

        finishEntry();
        myIsDone = true;
    }

    // OCCT.NET: the end of the copy at the entry: the copies left unchanged are the caller's own again, in the result
    // and in the healing context, which then answers for the caller's sub-shapes.
    private void finishEntry()
    {
        if (myEntry is not null)
        {
            myResShape = myEntry.Finish(myResShape);
        }
    }

    //=============================================================================
    // function : tgtfaces
    // purpose  : check the angle at the border between two squares.
    //           Two shares should have a shared front edge.
    //=============================================================================
    private static void tgtfaces(in TopoDS_Edge Ed, in TopoDS_Face F1, in TopoDS_Face F2, bool couture,
                                 ref double theResAngle)
    {
        // Check that pcurves exist on both faces of edge.
        double aFirst, aLast;
        Geom2d_Curve? aCurve;
        aCurve = BRep_Tool.CurveOnSurface(Ed, F1, out aFirst, out aLast);
        if (aCurve is null)
        {
            return;
        }
        aCurve = BRep_Tool.CurveOnSurface(Ed, F2, out aFirst, out aLast);
        if (aCurve is null)
        {
            return;
        }

        double u;
        TopoDS_Edge E = Ed;
        // OCCT.NET: OCCT makes aBAS1 and aBAS2, then copies them into HS1 and HS2; the copies are made directly
        var HS1 = new BRepAdaptor_Surface(F1, false);
        BRepAdaptor_Surface HS2;
        if (couture)
        {
            HS2 = HS1;
        }
        else
        {
            HS2 = new BRepAdaptor_Surface(F2, false);
        }
        // case when edge lies on the one face

        E.Orientation(TopAbs_FORWARD);
        var C2d1 = new BRepAdaptor_Curve2d(E, F1);
        if (couture)
        {
            E.Orientation(TopAbs_REVERSED);
        }
        var C2d2 = new BRepAdaptor_Curve2d(E, F2);

        bool rev1 = (F1.Orientation() == TopAbs_REVERSED);
        bool rev2 = (F2.Orientation() == TopAbs_REVERSED);
        double f, l, eps;
        BRep_Tool.Range(E, out f, out l);
        // OCCT.NET: OCCT's unused local Extrema_LocateExtPC ext is left out

        eps = (l - f) / 100.0;
        f += eps; // to avoid calculations on
        l -= eps; // points of pointed squares.
        Point2D p;
        Point3D pp1, pp2; //,PP;
        Vector3D du1, dv1;
        Vector3D du2, dv2;
        Vector3D d1, d2;
        double norm;

        const int NBPNT = 23;
        for (int i = 0; i <= NBPNT; i++)
        {
            // First suppose that this is sameParameter
            u = f + (l - f) * i / NBPNT;

            // take derivatives of surfaces at the same u, and compute normals
            C2d1.D0(u, out p);
            HS1.D1(p.X(), p.Y(), out pp1, out du1, out dv1);
            d1 = (du1.Crossed(dv1));
            norm = d1.Magnitude();
            if (norm > 1.0e-12)
            {
                d1 /= norm;
            }
            else
            {
                continue; // skip degenerated point
            }
            if (rev1)
            {
                d1.Reverse();
            }

            C2d2.D0(u, out p);
            HS2.D1(p.X(), p.Y(), out pp2, out du2, out dv2);
            d2 = (du2.Crossed(dv2));
            norm = d2.Magnitude();
            if (norm > 1.0e-12)
            {
                d2 /= norm;
            }
            else
            {
                continue; // skip degenerated point
            }
            if (rev2)
            {
                d2.Reverse();
            }

            // Compute angle.
            double aCurrentAng = d1.Angle(d2);

            theResAngle = Max(theResAngle, aCurrentAng);
        }
    }

    //=============================================================================
    // function : ComputeMaxAngleOnShape
    // purpose  : Code the regularities on all edges of the shape, boundary of
    //            two faces that do not have it.
    //=============================================================================
    private static void ComputeMaxAngleOnShape(in TopoDS_Shape S, ref double theResAngle)
    {
        var M = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(S, TopAbs_EDGE, TopAbs_FACE, M);
        var It = new NCollection_List<TopoDS_Shape>.Iterator();
        var Ex = new TopExp_Explorer();
        TopoDS_Face F1 = new(), F2 = new();
        bool found, couture;
        for (int i = 1; i <= M.Extent(); i++)
        {
            TopoDS_Edge E = TopoDS.Edge(M.FindKey(i));
            found = false;
            couture = false;
            F1.Nullify();
            for (It.Initialize(M.FindFromIndex(i)); It.More() && !found; It.Next())
            {
                if (F1.IsNull())
                {
                    F1 = TopoDS.Face(It.Value());
                }
                else
                {
                    if (!F1.IsSame(TopoDS.Face(It.Value())))
                    {
                        found = true;
                        F2 = TopoDS.Face(It.Value());
                    }
                }
            }
            if (!found && !F1.IsNull())
            { // is it a sewing edge?
                TopAbs_Orientation orE = E.Orientation();
                TopoDS_Edge curE;
                for (Ex.Init(F1, TopAbs_EDGE); Ex.More() && !found; Ex.Next())
                {
                    curE = TopoDS.Edge(Ex.Current());
                    if (E.IsSame(curE) && orE != curE.Orientation())
                    {
                        found = true;
                        couture = true;
                        F2 = F1;
                    }
                }
            }
            if (found)
            {
                if (BRep_Tool.Continuity(E, F1, F2) <= C0)
                {
                    try
                    {
                        tgtfaces(E, F1, F2, couture, ref theResAngle);
                    }
                    catch (FailureException)
                    {
                    }
                }
            }
        }
    }

    //=============================================================================
    // function : ComputeMaxAngle
    // purpose  : Computes max angle in faces junction
    //=============================================================================
    private void ComputeMaxAngle()
    {
        ComputeMaxAngleOnShape(myInputShape, ref myMaxAngle);
    }

    //=============================================================================
    // function : BuildMissingWalls
    // purpose  : Builds walls to the result solid.
    //=============================================================================
    private bool BuildMissingWalls()
    {
        // Internal list of new faces.
        TopoDS_Compound aNewFaces = new();
        var aBB = new BRep_Builder();
        aBB.MakeCompound(ref aNewFaces);

        // Compute outer bounds of original shape.
        var aFB = new ShapeAnalysis_FreeBounds(myShape);
        TopoDS_Compound aFreeWires = aFB.InternalGetClosedWires();

        // Build linear faces on each edge and its image.
        var anExpCW = new TopExp_Explorer(aFreeWires, TopAbs_WIRE);
        for (; anExpCW.More(); anExpCW.Next())
        {
            TopoDS_Wire aCurWire = TopoDS.Wire(anExpCW.Current());

            // Iterate over outer edges in outer wires.
            var anExpWE = new TopExp_Explorer(aCurWire, TopAbs_EDGE);
            for (; anExpWE.More(); anExpWE.Next())
            {
                TopoDS_Edge aCurEdge = TopoDS.Edge(anExpWE.Current());

                TopoDS_Face aNewFace = BuildWallFace(aCurEdge);

                if (aNewFace.IsNull())
                {
                    myError = BRepOffsetSimple_ErrorWallFaceComputation;
                    return false;
                }

                aBB.Add(aNewFaces, aNewFace);
            }
        }

        // Update edges from wall faces.
        var aSFE = new ShapeFix_Edge();
        aSFE.SetContext(myReShape);
        var anExpCE = new TopExp_Explorer(aNewFaces, TopAbs_EDGE);
        for (; anExpCE.More(); anExpCE.Next())
        {
            // Fix same parameter and same range flags.
            TopoDS_Edge aCurrEdge = TopoDS.Edge(anExpCE.Current());
            aSFE.FixSameParameter(aCurrEdge);
        }

        // Update result to be compound.
        TopoDS_Compound aResCompound = new();
        aBB.MakeCompound(ref aResCompound);

        // Add old faces the result.
        var anExpSF = new TopExp_Explorer(myShape, TopAbs_FACE);
        for (; anExpSF.More(); anExpSF.Next())
        {
            aBB.Add(aResCompound, anExpSF.Current());
        }

        // Add new faces the result.
        anExpSF.Init(myResShape, TopAbs_FACE);
        for (; anExpSF.More(); anExpSF.Next())
        {
            aBB.Add(aResCompound, anExpSF.Current());
        }

        // Add wall faces to the result.
        var anExpCF = new TopExp_Explorer(aNewFaces, TopAbs_FACE);
        for (; anExpCF.More(); anExpCF.Next())
        {
            TopoDS_Face aF = TopoDS.Face(anExpCF.Current());
            aBB.Add(aResCompound, aF);
        }

        // Apply stored modifications.
        aResCompound = TopoDS.Compound(myReShape.InternalApply(aResCompound));

        // Create result shell.
        var aQuilt = new BRepTools_Quilt();
        aQuilt.Add(aResCompound);
        TopoDS_Shape aShells = aQuilt.InternalShells();

        var anExpSSh = new TopExp_Explorer(aShells, TopAbs_SHELL);
        TopoDS_Shell aResShell = new();
        for (; anExpSSh.More(); anExpSSh.Next())
        {
            if (!aResShell.IsNull())
            {
                // Shell is not null -> explorer contains two or more shells.
                myError = BRepOffsetSimple_ErrorInvalidNbShells;
                return false;
            }
            aResShell = TopoDS.Shell(anExpSSh.Current());
        }

        // OCCT: with no shell at all (Quilt made none), aResShell is null and IsClosed dereferences it
        if (!BRep_Tool.IsClosed(aResShell))
        {
            myError = BRepOffsetSimple_ErrorNonClosedShell;
            return false;
        }

        // Create result solid.
        TopoDS_Solid aResSolid = new();
        aBB.MakeSolid(ref aResSolid);
        aBB.Add(aResSolid, aResShell);
        myResShape = aResSolid;

        return true;
    }

    //=================================================================================================

    // Builds face on specified wall.
    private TopoDS_Face BuildWallFace(in TopoDS_Edge theOrigEdge)
    {
        TopoDS_Face aResFace = new();

        // Get offset edge. offset edge is reversed to create correct wire.
        TopoDS_Edge aNewEdge = TopoDS.Edge(myBuilder.InternalModifiedShape(theOrigEdge));
        aNewEdge.Orientation(TopAbs_REVERSED);

        TopExp.Vertices(aNewEdge, out TopoDS_Vertex aNewV1, out TopoDS_Vertex aNewV2);

        // Wire contour is:
        // theOrigEdge (forcible forward) -> wall1 -> aNewEdge (forcible reversed) -> wall2
        // Firstly it is necessary to create copy of original shape with forward direction.
        // This simplifies walls creation.
        TopoDS_Edge anOrigCopy = theOrigEdge;
        anOrigCopy.Orientation(TopAbs_FORWARD);
        TopExp.Vertices(anOrigCopy, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);

        // To simplify work with map.
        TopoDS_Vertex aForwardV1 = TopoDS.Vertex(aV1.Oriented(TopAbs_FORWARD));
        TopoDS_Vertex aForwardV2 = TopoDS.Vertex(aV2.Oriented(TopAbs_FORWARD));

        // Check existence of edges in stored map: Edge1
        TopoDS_Edge aWall1;
        if (myMapVE.IsBound(aForwardV2))
        {
            // Edge exists - get it from map.
            aWall1 = myMapVE[aForwardV2];
        }
        else
        {
            // Edge does not exist - create it and add to the map.
            var aME1 = new BRepLib_MakeEdge(TopoDS.Vertex(aV2.Oriented(TopAbs_FORWARD)),
                                            TopoDS.Vertex(aNewV2.Oriented(TopAbs_REVERSED)));
            if (!aME1.IsDone())
            {
                return aResFace;
            }
            aWall1 = aME1.InternalEdge();

            myMapVE.Bind(aForwardV2, aWall1);
        }

        // Check existence of edges in stored map: Edge2
        TopoDS_Edge aWall2;
        if (myMapVE.IsBound(aForwardV1))
        {
            // Edge exists - get it from map.
            aWall2 = TopoDS.Edge(myMapVE[aForwardV1].Oriented(TopAbs_REVERSED));
        }
        else
        {
            // Edge does not exist - create it and add to the map.
            var aME2 = new BRepLib_MakeEdge(TopoDS.Vertex(aV1.Oriented(TopAbs_FORWARD)),
                                            TopoDS.Vertex(aNewV1.Oriented(TopAbs_REVERSED)));
            if (!aME2.IsDone())
            {
                return aResFace;
            }
            aWall2 = aME2.InternalEdge();

            myMapVE.Bind(aForwardV1, aWall2);

            // Orient it in reversed direction.
            aWall2.Orientation(TopAbs_REVERSED);
        }

        var aBB = new BRep_Builder();

        TopoDS_Wire aWire = new();
        aBB.MakeWire(ref aWire);
        aBB.Add(aWire, anOrigCopy);
        aBB.Add(aWire, aWall1);
        aBB.Add(aWire, aNewEdge);
        aBB.Add(aWire, aWall2);

        // Build 3d curves on wire
        BRepLib.BuildCurves3d(aWire);

        // Try to build using simple planar approach.
        TopoDS_Face aF = new();
        try
        {
            // Call of face maker is wrapped by try/catch since it generates exceptions sometimes.
            var aFM = new BRepLib_MakeFace(aWire, true);
            if (aFM.IsDone())
            {
                aF = aFM.InternalFace();
            }
        }
        catch (FailureException)
        {
        }

        if (aF.IsNull()) // Exception in face maker or result is not computed.
        {
            // Build using thrusections.
            bool ToReverse = false;
            double fpar, lpar, fparOE, lparOE;
            Geom_Curve EdgeCurve = BRep_Tool.Curve(theOrigEdge, out fpar, out lpar)!;
            var TrEdgeCurve = new Geom_TrimmedCurve(EdgeCurve, fpar, lpar);
            Geom_Curve OffsetCurve = BRep_Tool.Curve(aNewEdge, out fparOE, out lparOE)!;
            var TrOffsetCurve = new Geom_TrimmedCurve(OffsetCurve, fparOE, lparOE);

            var ThrusecGenerator = new GeomFill_Generator();
            ThrusecGenerator.AddCurve(TrEdgeCurve);
            ThrusecGenerator.AddCurve(TrOffsetCurve);
            ThrusecGenerator.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
            Geom_Surface theSurf = ThrusecGenerator.Surface()!;
            // theSurf = new Geom_SurfaceOfLinearExtrusion( TrOffsetCurve, OffsetDir );
            theSurf.Bounds(out double Uf, out double Ul, out double Vf, out double Vl);
            Placement Loc = new();
            Geom2d_Line EdgeLine2d, OELine2d, aLine2d, aLine2d2;
            EdgeLine2d = new Geom2d_Line(new Point2D(0.0, Vf), new Direction2D(Direction2D.D.X));
            aBB.UpdateEdge(theOrigEdge, EdgeLine2d, theSurf, Loc, global::Kerf.Engine.Foundation.Precision.Confusion());
            OELine2d = new Geom2d_Line(new Point2D(0.0, Vl), new Direction2D(Direction2D.D.X));
            aBB.UpdateEdge(aNewEdge, OELine2d, theSurf, Loc, global::Kerf.Engine.Foundation.Precision.Confusion());
            double UonV1 = (ToReverse) ? Ul : Uf;
            double UonV2 = (ToReverse) ? Uf : Ul;
            aLine2d = new Geom2d_Line(new Point2D(UonV2, 0.0), new Direction2D(Direction2D.D.Y));
            aLine2d2 = new Geom2d_Line(new Point2D(UonV1, 0.0), new Direction2D(Direction2D.D.Y));
            if (aWall1.IsSame(aWall2))
            {
                aBB.UpdateEdge(aWall1, aLine2d, aLine2d2, theSurf, Loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                Geom_Curve BSplC34 = theSurf.UIso(Uf);
                aBB.UpdateEdge(aWall1, BSplC34, global::Kerf.Engine.Foundation.Precision.Confusion());
                aBB.Range(aWall1, Vf, Vl);
            }
            else
            {
                aBB.SameParameter(aWall1, false);
                aBB.SameRange(aWall1, false);
                aBB.SameParameter(aWall2, false);
                aBB.SameRange(aWall2, false);
                aBB.UpdateEdge(aWall1, aLine2d, theSurf, Loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                aBB.Range(aWall1, theSurf, Loc, Vf, Vl);
                aBB.UpdateEdge(aWall2, aLine2d2, theSurf, Loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                aBB.Range(aWall2, theSurf, Loc, Vf, Vl);
                Geom_Curve BSplC3 = theSurf.UIso(UonV2);
                aBB.UpdateEdge(aWall1, BSplC3, global::Kerf.Engine.Foundation.Precision.Confusion());
                aBB.Range(aWall1, Vf, Vl, true); // only for 3d curve
                Geom_Curve BSplC4 = theSurf.UIso(UonV1);
                aBB.UpdateEdge(aWall2, BSplC4, global::Kerf.Engine.Foundation.Precision.Confusion());
                aBB.Range(aWall2, Vf, Vl, true); // only for 3d curve
            }

            aF = new BRepLib_MakeFace(theSurf, aWire).InternalFace();
        }

        return aF;
    }

    //=================================================================================================

    /// <summary>The shape generated from a sub-shape of the input (frozen); null if none.</summary>
    /// <exception cref="NoSuchObjectException">The shape isn't a sub-shape of the input.</exception>
    public TopoDS_Shape Generated(in TopoDS_Shape theShape)
    {
        TopoDS_Shape aRes = InternalGenerated(theShape);
        aRes.TShape()?.FreezeGraph();
        return aRes;
    }

    /// <summary><see cref="Generated"/> without freezing, for ported code.</summary>
    internal TopoDS_Shape InternalGenerated(in TopoDS_Shape theShape)
    {
        // Shape generated by modification.
        TopoDS_Shape aRes;
        // OCCT.NET: the modifier ran on the copy at the entry, if any; what stands for its result after the entry's end
        aRes = myBuilder.InternalModifiedShape(myEntry is null ? theShape : myEntry.Copied(theShape));
        if (myEntry is not null)
        {
            aRes = myEntry.Final(aRes);
        }

        if (aRes.IsNull())
        {
            return aRes;
        }

        // Shape modifications obtained in scope of shape healing.
        aRes = myReShape.InternalApply(aRes);

        return aRes;
    }

    //=================================================================================================

    /// <summary>The shape a sub-shape of the input was modified into by the healing of the walls (frozen); null if
    /// none.</summary>
    public TopoDS_Shape Modified(in TopoDS_Shape theShape)
    {
        TopoDS_Shape aRes = InternalModified(theShape);
        aRes.TShape()?.FreezeGraph();
        return aRes;
    }

    /// <summary><see cref="Modified"/> without freezing, for ported code.</summary>
    internal TopoDS_Shape InternalModified(in TopoDS_Shape theShape)
    {
        TopoDS_Shape anEmptyShape = new();

        // Get modification status and new shape.
        int aModStatus = myReShape.InternalStatus(theShape, out TopoDS_Shape aRes);

        if (aModStatus == 0)
        {
            return anEmptyShape; // No modifications are applied to the shape or its sub-shapes.
        }

        return aRes;
    }
}
