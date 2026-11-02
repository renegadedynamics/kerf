// Port of: src/DataExchange/TKXSBase/XSAlgo/XSAlgo_ShapeProcessor.hxx
// Port of: src/DataExchange/TKXSBase/XSAlgo/XSAlgo_ShapeProcessor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Shape Processing module. Allows to define and apply general Shape Processing as a customizable sequence of
/// operators.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's type aliases are written out: <c>ParameterMap</c> is <c>NCollection_DataMap&lt;string, string&gt;</c>,
/// <c>ProcessingData</c> the tuple (parameters, operations), <c>ProcessingFlags</c> the tuple (operations, set).
/// </para>
/// <para>
/// The data exchange parameters OCCT reads from <c>Interface_Static</c> come from the
/// <see cref="Interface_StaticSet"/> the caller passes (decision 1181), after OCCT's own parameters.
/// <c>PrepareForTransfer</c>, which sets UnitsMethods' process-wide length unit from <c>xstep.cascade.unit</c>, is
/// left out (decision 1203).
/// </para>
/// <para>
/// <see cref="ProcessShape"/> never writes into its input (ShapeProcess's working copy, decision 1022) and returns a
/// frozen shape. <see cref="CheckPCurve(in TopoDS_Edge, in TopoDS_Face, double, bool, Interface_StaticSet)"/> writes into the edge it is given, as OCCT's: it is for a reader's edges
/// under construction, not for published shapes (a frozen edge throws).
/// </para>
/// </remarks>
public class XSAlgo_ShapeProcessor
{
    /// <summary>
    /// Constructor. <paramref name="theParameters"/>: pre-filled parameter map to be used in the processing (copied).
    /// <paramref name="theShapeFixParameters"/>: shape healing parameters to be used in the processing; if
    /// <paramref name="theParameters"/> has some shape healing values, they will override the corresponding values from
    /// <paramref name="theShapeFixParameters"/>.
    /// </summary>
    public XSAlgo_ShapeProcessor(NCollection_DataMap<string, string> theParameters, in DE_ShapeFixParameters theShapeFixParameters)
    {
        myParameters = new NCollection_DataMap<string, string>(theParameters);
        FillParameterMap(theShapeFixParameters, false, myParameters);
    }

    /// <summary>Constructor, with the default shape healing parameters (OCCT's default argument <c>{}</c>).</summary>
    public XSAlgo_ShapeProcessor(NCollection_DataMap<string, string> theParameters)
        : this(theParameters, new DE_ShapeFixParameters())
    {
    }

    /// <summary>Constructor. <paramref name="theParameters"/>: parameters to be used in the processing.</summary>
    public XSAlgo_ShapeProcessor(in DE_ShapeFixParameters theParameters)
    {
        var aMap = new NCollection_DataMap<string, string>();
        FillParameterMap(theParameters, false, aMap);
        myParameters = aMap;
    }

    /// <summary>
    /// Process the shape by applying the specified operations. Returns the processed shape (frozen); may be the same as
    /// the input shape if no modifications were made.
    /// </summary>
    public TopoDS_Shape ProcessShape(in TopoDS_Shape theShape, ShapeProcess.OperationsFlags theOperations,
                                     Message_ProgressRange theProgress = default)
    {
        if (theShape.IsNull())
        {
            return theShape;
        }
        initializeContext(theShape);
        return ShapeProcess.Perform(myContext, theOperations, theProgress) ? myContext.Result() : theShape;
    }

    /// <summary>
    /// As <see cref="ProcessShape"/>, without freezing the result (OCCT.NET, for STEPControl_ActorRead, decision 1273):
    /// a reader's working shapes are processed in place, as OCCT's, so that the items sharing them keep sharing them.
    /// </summary>
    internal TopoDS_Shape InternalProcessShape(in TopoDS_Shape theShape, ShapeProcess.OperationsFlags theOperations,
                                               Message_ProgressRange theProgress = default)
    {
        if (theShape.IsNull())
        {
            return theShape;
        }
        initializeContext(theShape);
        return ShapeProcess.Perform(myContext, theOperations, theProgress) ? myContext.InternalResult() : theShape;
    }

    /// <summary>As <see cref="MergeTransferInfo(Transfer_TransientProcess, int)"/>, without freezing the history's shapes (OCCT.NET, decision 1273).</summary>
    internal void InternalMergeTransferInfo(Transfer_TransientProcess theTransientProcess, int theFirstTPItemIndex)
    {
        if (myContext is null)
        {
            return;
        }
        MergeShapeTransferInfo(theTransientProcess, myContext.InternalMap(), theFirstTPItemIndex, myContext.Messages());
    }

    /// <summary>Get the context of the last processing. Only valid after the ProcessShape() method was called.</summary>
    public ShapeProcess_ShapeContext? GetContext()
    {
        return myContext;
    }

    // Initialize the context with the specified shape.
    private void initializeContext(in TopoDS_Shape theShape)
    {
        myContext = new ShapeProcess_ShapeContext(theShape, null);
        foreach (KeyValuePair<string, string> aParameterIter in myParameters)
        {
            myContext.ResourceManager().SetResource(aParameterIter.Key, aParameterIter.Value);
        }
        // Read and set detalization level.
        if (myParameters.Find("DetalizationLevel", out string aResult) && TCollection_AsciiString.IsIntegerValue(aResult))
        {
            TopAbs_ShapeEnum aDetalizationLevel = (TopAbs_ShapeEnum)TCollection_AsciiString.IntegerValue(aResult);
            myContext.SetDetalisation(aDetalizationLevel);
        }
        // Read and set non-manifold flag.
        if (myParameters.Find("NonManifold", out aResult) && TCollection_AsciiString.IsIntegerValue(aResult))
        {
            bool aNonManifold = TCollection_AsciiString.IntegerValue(aResult) != 0;
            myContext.SetNonManifold(aNonManifold);
        }
    }

    // Add messages from the specified shape to the transfer binder.
    private static void addMessages(ShapeExtend_MsgRegistrator? theMessages, in TopoDS_Shape theShape, Transfer_Binder theBinder)
    {
        if (theMessages is null)
        {
            return;
        }
        if (!theMessages.MapShape().Find(theShape, out NCollection_List<Message_Msg> aShapeMessages))
        {
            return;
        }
        for (var aMsgIter = new NCollection_List<Message_Msg>.Iterator(aShapeMessages); aMsgIter.More(); aMsgIter.Next())
        {
            Message_Msg aMessage = aMsgIter.Value();
            theBinder.AddWarning(aMessage.Value(), aMessage.Original());
        }
    }

    /// <summary>
    /// Merge the results of the shape processing with the transfer process: the shape binders of the process from
    /// <paramref name="theFirstTPItemIndex"/> get the shapes their results became, and the messages attached to them.
    /// </summary>
    public static void MergeShapeTransferInfo(Transfer_TransientProcess theTransientProcess,
                                              NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theModifiedShapesMap,
                                              int theFirstTPItemIndex,
                                              ShapeExtend_MsgRegistrator? theMessages)
    {
        if (theModifiedShapesMap.IsEmpty())
        {
            return;
        }
        bool aToPrint = theMessages is not null && !theMessages.MapShape().IsEmpty();
        for (int i = Math.Max(theFirstTPItemIndex, 1); i <= theTransientProcess.NbMapped(); ++i)
        {
            TransferBRep_ShapeBinder? aShapeBinder = theTransientProcess.MapItem(i) as TransferBRep_ShapeBinder;
            if (aShapeBinder is null || aShapeBinder.Result().IsNull())
            {
                continue;
            }

            TopoDS_Shape anOriginalShape = aShapeBinder.Result();
            if (theModifiedShapesMap.IsBound(anOriginalShape))
            {
                aShapeBinder.SetResult(theModifiedShapesMap.Find(anOriginalShape));
            }
            else if (!anOriginalShape.Location().IsIdentity())
            {
                var aNullLoc = new TopLoc_Location();
                TopoDS_Shape aTemporaryShape = anOriginalShape.Located(aNullLoc);
                if (theModifiedShapesMap.IsBound(aTemporaryShape))
                {
                    aShapeBinder.SetResult(theModifiedShapesMap.Find(aTemporaryShape));
                }
            }
            else
            {
                // Some of edges may be modified.
                var aReShaper = new BRepTools_ReShape();
                bool aHasModifiedEdges = false;
                // Remember modifications.
                for (var anExpSE = new TopExp_Explorer(anOriginalShape, TopAbs_EDGE); anExpSE.More(); anExpSE.Next())
                {
                    if (theModifiedShapesMap.IsBound(anExpSE.Current()))
                    {
                        aHasModifiedEdges = true;
                        TopoDS_Shape aModifiedShape = theModifiedShapesMap.Find(anExpSE.Current());
                        aReShaper.Replace(anExpSE.Current(), aModifiedShape);
                    }
                }
                // Apply modifications and store result in binder.
                if (aHasModifiedEdges)
                {
                    TopoDS_Shape aReshapedShape = aReShaper.Apply(anOriginalShape);
                    aShapeBinder.SetResult(aReshapedShape);
                }
            }

            // update messages
            if (aToPrint)
            {
                addMessages(theMessages, anOriginalShape, aShapeBinder);
            }
        }
    }

    /// <summary>
    /// Merge the results of the shape processing with the transfer process. <paramref name="theFirstTPItemIndex"/>:
    /// index of the first item in the transfer process to merge with.
    /// </summary>
    public void MergeTransferInfo(Transfer_TransientProcess theTransientProcess, int theFirstTPItemIndex)
    {
        if (myContext is null)
        {
            return;
        }
        MergeShapeTransferInfo(theTransientProcess, myContext.Map(), theFirstTPItemIndex, myContext.Messages());
    }

    /// <summary>
    /// Merge the results of the shape processing with the finder process: each modified shape is bound to (or adds a
    /// result to) the binder of what it became.
    /// </summary>
    /// <remarks>
    /// OCCT iterates the map of modified shapes in its hash order (TShape addresses), so the order the new mappers are
    /// bound in the finder process varies natively; the port iterates the map in the order the shape processing recorded
    /// its entries (decision 1204).
    /// </remarks>
    public static void MergeShapeTransferInfo(Transfer_FinderProcess theFinderProcess,
                                              NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theModifiedShapesMap,
                                              ShapeExtend_MsgRegistrator? theMessages)
    {
        if (theModifiedShapesMap.IsEmpty())
        {
            return;
        }
        bool aToPrint = theMessages is not null && !theMessages.MapShape().IsEmpty();
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> ShapeShapeIterator in theModifiedShapesMap)
        {
            TopoDS_Shape anOriginalShape = ShapeShapeIterator.Key;
            TopoDS_Shape aResultShape = ShapeShapeIterator.Value;

            TransferBRep_ShapeMapper aResultMapper = TransferBRep.ShapeMapper(theFinderProcess, aResultShape);
            Transfer_Binder? aResultBinder = theFinderProcess.Find(aResultMapper);

            if (aResultBinder is null)
            {
                aResultBinder = new TransferBRep_ShapeBinder(aResultShape);
                // if <orig> shape was split, put entities corresponding to new shapes
                //  into Transfer_TransientListBinder.
                if (anOriginalShape.ShapeType() > aResultShape.ShapeType())
                {
                    TopoDS_Shape aSubShape = new TopoDS_Shape();
                    var aTransientListBinder = new Transfer_TransientListBinder();
                    for (var aSubShapeIter = new TopoDS_Iterator(aResultShape); aSubShapeIter.More(); aSubShapeIter.Next())
                    {
                        TopoDS_Shape aCurrentSubShape = aSubShapeIter.Value();
                        Transfer_Finder? aSubShapeMapper = TransferBRep.ShapeMapper(theFinderProcess, aCurrentSubShape);
                        if (aSubShapeMapper is null)
                        {
                            continue;
                        }
                        Standard_Transient? aTransientResult = theFinderProcess.FindTransient(aSubShapeMapper);
                        if (aTransientResult is null)
                        {
                            continue;
                        }
                        aTransientListBinder.AddResult(aTransientResult);
                        aSubShape = aCurrentSubShape;
                    }
                    if (aTransientListBinder.NbTransients() == 1)
                    {
                        aResultBinder = new TransferBRep_ShapeBinder(aSubShape);
                    }
                    else if (aTransientListBinder.NbTransients() > 1)
                    {
                        aResultBinder.AddResult(aTransientListBinder);
                    }
                }
            }

            TransferBRep_ShapeMapper anOriginalMapper = TransferBRep.ShapeMapper(theFinderProcess, anOriginalShape);
            Transfer_Binder? anOriginalBinder = theFinderProcess.Find(anOriginalMapper);
            if (anOriginalBinder is null)
            {
                theFinderProcess.Bind(anOriginalMapper, aResultBinder);
            }
            else
            {
                anOriginalBinder.AddResult(aResultBinder);
            }

            // update messages
            if (aToPrint)
            {
                addMessages(theMessages, anOriginalShape, aResultBinder);
            }
        }
    }

    /// <summary>Merge the results of the shape processing with the finder process.</summary>
    public void MergeTransferInfo(Transfer_FinderProcess theFinderProcess)
    {
        if (myContext is null)
        {
            return;
        }
        MergeShapeTransferInfo(theFinderProcess, myContext.Map(), myContext.Messages());
    }

    // Create a new edge with the same geometry as the source edge.
    private static TopoDS_Edge MakeEdgeOnCurve(in TopoDS_Edge aSourceEdge)
    {
        TopoDS_Edge aResult = new TopoDS_Edge();

        var anEdgeAnalyzer = new ShapeAnalysis_Edge();
        if (!anEdgeAnalyzer.Curve3d(aSourceEdge, out Geom_Curve? aSourceGeomCurve, out double aStartParam, out double anEndParam, false))
        {
            return aResult;
        }
        gp_Pnt aCurveStartPt = aSourceGeomCurve.Value(aStartParam);
        gp_Pnt aCurveEndPt = aSourceGeomCurve.Value(anEndParam);
        var anEdgeMaker = new BRepBuilderAPI_MakeEdge(aSourceGeomCurve, aCurveStartPt, aCurveEndPt, aStartParam, anEndParam);
        var SBE = new ShapeBuild_Edge();
        SBE.SetRange3d(anEdgeMaker.InternalEdge(), aStartParam, anEndParam);
        aResult = anEdgeMaker.InternalEdge(); // a working edge, not published
        return aResult;
    }

    /// <summary>
    /// Check quality of pcurve of the edge on the given face, and correct it if necessary. Returns True if the pcurve
    /// was corrected, false if it was dropped. Reads <c>read.stdsameparameter.mode</c> in <paramref name="theStatics"/>.
    /// </summary>
    /// <remarks>Writes into <paramref name="theEdge"/> and its vertices, which must not be frozen.</remarks>
    public static bool CheckPCurve(in TopoDS_Edge theEdge, in TopoDS_Face theFace, double thePrecision, bool theIsSeam,
                                   Interface_StaticSet theStatics)
        => CheckPCurve(theEdge, theFace, thePrecision, theIsSeam, theStatics.IVal("read.stdsameparameter.mode") != 0);

    /// <summary>
    /// As the form with statics, given the value of <c>read.stdsameparameter.mode</c> (OCCT.NET addition: StepToTopoDS
    /// takes it from its model's parameters, decision 1246).
    /// </summary>
    internal static bool CheckPCurve(in TopoDS_Edge theEdge, in TopoDS_Face theFace, double thePrecision, bool theIsSeam,
                                     bool theStdSameParameterMode)
    {
        var anEdgeAnalyzer = new ShapeAnalysis_Edge();

        // Retrieve pcurve and its parameters.
        if (!anEdgeAnalyzer.PCurve(theEdge, theFace, out Geom2d_Curve? aCurve2D, out double aCurve2DParam1, out double aCurve2DParam2, false))
        {
            return false;
        }

        // Check for pcurve longer than surface.
        Geom_Surface aSurface = BRep_Tool.Surface(theFace);
        aSurface.Bounds(out double aFaceSurfaceU1, out double aFaceSurfaceU2, out double aFaceSurfaceV1, out double aFaceSurfaceV2);
        gp_Pnt2d aCurve2DPoint1 = aCurve2D.Value(aCurve2DParam1);
        gp_Pnt2d aCurve2DPoint2 = aCurve2D.Value(aCurve2DParam2);
        // Multi-periodic? Better to discard (beware of infinite values)
        double anEdgeSpanX = Math.Abs(aCurve2DPoint1.X() - aCurve2DPoint2.X());
        double anEdgeSpanY = Math.Abs(aCurve2DPoint1.Y() - aCurve2DPoint2.Y());
        // So if span of pcurve along U or V is longer than 6/8 of the surface span, discard it.
        // Why exactly 6/8? No idea, but it's the same as in the original code.
        if (anEdgeSpanX / 8.0 > (aFaceSurfaceU2 / 6.0 - aFaceSurfaceU1 / 6.0)
            || anEdgeSpanY / 8.0 > (aFaceSurfaceV2 / 6.0 - aFaceSurfaceV1 / 6.0))
        {
            new ShapeBuild_Edge().RemovePCurve(theEdge, theFace);
            return false;
        }

        // Second Check: 2D and 3D consistency (if the Pcurve has not been dropped)
        // We also check that it does not wrap around too much...
        // Example: UVV in DEGREES on a surface in RADIANS, adjusted = 57 turns!
        anEdgeAnalyzer.Curve3d(theEdge, out Geom_Curve? aCurve3D, out double aCurve3DParam1, out double aCurve3DParam2, false);

        gp_Pnt aCurve3DPoint1 = aSurface.Value(aCurve2DPoint1.X(), aCurve2DPoint1.Y());
        gp_Pnt aCurve3DPoint2 = aSurface.Value(aCurve2DPoint2.X(), aCurve2DPoint2.Y());
        TopoDS_Vertex aVertex1 = TopExp.FirstVertex(theEdge);
        TopoDS_Vertex aVertex2 = TopExp.LastVertex(theEdge);
        gp_Pnt aPV1 = aCurve3D is null ? BRep_Tool.Pnt(aVertex1) : aCurve3D.Value(aCurve3DParam1);
        gp_Pnt aPV2 = aCurve3D is null ? BRep_Tool.Pnt(aVertex2) : aCurve3D.Value(aCurve3DParam2);
        double aDist11 = aPV1.Distance(aCurve3DPoint1);
        double aDist22 = aPV2.Distance(aCurve3DPoint2);
        if ((aDist11 > thePrecision) || (aDist22 > thePrecision))
        {
            new ShapeBuild_Edge().RemovePCurve(theEdge, theFace);
            return false;
        }

        // pdn checking deviation between pcurve and 3D curve
        // Make temporary edge for analysis
        if (aCurve3D is null)
        {
            return false;
        }
        TopoDS_Edge aTmpEdge = MakeEdgeOnCurve(theEdge);

        // fill it with pcurve(s)
        var aBuilder = new BRep_Builder();
        Geom2d_Curve? aSeamPCurve;
        if (theIsSeam)
        {
            TopoDS_Edge aReversedEdge = TopoDS.Edge(theEdge.Reversed());
            if (!anEdgeAnalyzer.PCurve(aReversedEdge, theFace, out aSeamPCurve, out double aSeamPCurveParam1, out double aSeamPCurveParam2, false)
                || aSeamPCurve == aCurve2D)
            {
                aSeamPCurve = aCurve2D.Copy() as Geom2d_Curve;
            }
            aBuilder.UpdateEdge(aTmpEdge, aCurve2D, aSeamPCurve, theFace, 0.0);
        }
        else
        {
            aSeamPCurve = null;
            aBuilder.UpdateEdge(aTmpEdge, aCurve2D, theFace, 0.0);
        }
        aBuilder.Range(aTmpEdge, theFace, aCurve2DParam1, aCurve2DParam2);
        aBuilder.SameRange(aTmpEdge, false);
        if (theStdSameParameterMode)
        {
            aBuilder.SameParameter(aTmpEdge, false);
        }

        // call FixSP to see what it will do
        var anEdgeFixer = new ShapeFix_Edge();
        anEdgeFixer.FixSameParameter(aTmpEdge);
        double aTolerance = BRep_Tool.Tolerance(aTmpEdge);
        bool aSameRangeFlag = BRep_Tool.SameRange(aTmpEdge);
        bool aSameParameterFlag = BRep_Tool.SameParameter(aTmpEdge);

        // if result is not nice, try to call projection and take the best
        if (aTolerance > Math.Min(1.0, 2.0 * thePrecision) || !aSameRangeFlag)
        {
            // pdn trying to recompute pcurve
            TopoDS_Edge anEdgePr = MakeEdgeOnCurve(theEdge);
            anEdgeFixer.FixAddPCurve(anEdgePr, theFace, theIsSeam, thePrecision);
            anEdgeFixer.FixSameParameter(anEdgePr);
            double aTolerancePr = BRep_Tool.Tolerance(anEdgePr);
            // pdn choose the best pcurve
            if (aTolerancePr < aTolerance || !aSameRangeFlag)
            {
                aSameRangeFlag = BRep_Tool.SameRange(anEdgePr);
                aSameParameterFlag = BRep_Tool.SameParameter(anEdgePr);
                aTolerance = aTolerancePr;
                aTmpEdge = anEdgePr;
            }
        }

        // get corrected pcurve from the temporary edge, and put to original
        anEdgeAnalyzer.PCurve(aTmpEdge, theFace, out aCurve2D, out aCurve2DParam1, out aCurve2DParam2, false);
        if (theIsSeam)
        {
            TopoDS_Edge aReversedTmpEdge = TopoDS.Edge(aTmpEdge.Reversed());
            anEdgeAnalyzer.PCurve(aReversedTmpEdge, theFace, out aSeamPCurve, out double aReversedTmpEdgeParam1,
                                  out double aReversedTmpEdgeParam2, false);
            if (theEdge.Orientation() == TopAbs_REVERSED) //: abv 14.11.01: coneEl.sat loop
            {
                aBuilder.UpdateEdge(theEdge, aSeamPCurve, aCurve2D, theFace, aTolerance);
            }
            else
            {
                aBuilder.UpdateEdge(theEdge, aCurve2D, aSeamPCurve, theFace, aTolerance);
            }
        }
        else
        {
            aBuilder.UpdateEdge(theEdge, aCurve2D, theFace, aTolerance);
        }

        aBuilder.UpdateVertex(aVertex1, aTolerance);
        aBuilder.UpdateVertex(aVertex2, aTolerance);
        aBuilder.Range(theEdge, theFace, aCurve2DParam1, aCurve2DParam2);
        if (BRep_Tool.SameRange(theEdge))
        {
            aBuilder.SameRange(theEdge, aSameRangeFlag);
        }
        if (BRep_Tool.SameParameter(theEdge))
        {
            aBuilder.SameParameter(theEdge, aSameParameterFlag);
        }

        return true;
    }

    /// <summary>
    /// Reads the parameter map and the operation flags from the resource file named by the static
    /// <paramref name="theFileResourceName"/> (for example "read.step.resource.name", which may hold "STEP"), in the
    /// scope named by the static <paramref name="theScopeResourceName"/> (for example "read.step.sequence", which may
    /// hold "FromSTEP"). If the resource file isn't found, the statics' values are used instead.
    /// </summary>
    public static (NCollection_DataMap<string, string>, ShapeProcess.OperationsFlags) ReadProcessingData(
        string theFileResourceName, string theScopeResourceName, Interface_StaticSet theStatics)
    {
        string aFileName = theStatics.CVal(theFileResourceName);
        var aContext = new ShapeProcess_ShapeContext(new TopoDS_Shape(), aFileName);
        if (!aContext.ResourceManager().IsInitialized())
        {
            // If resource file wasn't found, use static values instead
            theStatics.FillMap(aContext.ResourceManager().GetMap());
        }
        string aScope = theStatics.CVal(theScopeResourceName);

        // Copy parameters to the result.
        var aResultParameters = new NCollection_DataMap<string, string>();
        var aResultFlags = new ShapeProcess.OperationsFlags();
        NCollection_DataMap<string, string> aMap = aContext.ResourceManager().GetMap();
        foreach (KeyValuePair<string, string> anIter in aMap)
        {
            string aKey = anIter.Key;
            if (!aKey.StartsWith(aScope, StringComparison.Ordinal))
            {
                // Ignore all parameters that don't start with the specified scope.
                continue;
            }
            // Remove scope prefix and dot
            // "FromIGES.FixShape.FixFreeFaceMode" -> "FixShape.FixFreeFaceMode"
            aKey = SubString(aKey, aScope.Length + 2, aKey.Length);
            if (aKey == "exec.op")
            {
                // Parse operation flags using Token method
                int aTokenCount = 1;
                string aToken;
                const string aSeparators = " \t,;";
                while ((aToken = TCollection_AsciiString.Token(anIter.Value, aSeparators, aTokenCount)).Length != 0)
                {
                    (ShapeProcess.Operation, bool) anOperationFlag = ShapeProcess.ToOperationFlag(aToken);
                    if (anOperationFlag.Item2)
                    {
                        aResultFlags = aResultFlags.With(anOperationFlag.Item1);
                    }
                    aTokenCount++;
                }
            }
            else
            {
                aResultParameters.Bind(aKey, anIter.Value);
            }
        }
        return (aResultParameters, aResultFlags);
    }

    // TCollection_AsciiString::SubString: the characters FromIndex to ToIndex (1-based)
    private static string SubString(string theString, int theFromIndex, int theToIndex)
    {
        if (theToIndex > theString.Length || theFromIndex <= 0 || theFromIndex > theToIndex)
        {
            throw new Standard_OutOfRange("TCollection_AsciiString::SubString : parameters out of range");
        }
        return theString.Substring(theFromIndex - 1, theToIndex - theFromIndex + 1);
    }

    /// <summary>
    /// Fill the parameter map with the values from the specified parameters. <paramref name="theIsReplace"/>: whether
    /// parameter should be replaced if it already exists in the map.
    /// </summary>
    public static void FillParameterMap(in DE_ShapeFixParameters theParameters, bool theIsReplace,
                                        NCollection_DataMap<string, string> theMap)
    {
        SetParameter("FixShape.Tolerance3d", theParameters.Tolerance3d, theIsReplace, theMap);
        SetParameter("FixShape.MaxTolerance3d", theParameters.MaxTolerance3d, theIsReplace, theMap);
        SetParameter("FixShape.MinTolerance3d", theParameters.MinTolerance3d, theIsReplace, theMap);
        SetParameter("DetalizationLevel", ((int)theParameters.DetalizationLevel).ToString(System.Globalization.CultureInfo.InvariantCulture),
                     theIsReplace, theMap);
        SetParameter("NonManifold", (theParameters.NonManifold ? 1 : 0).ToString(System.Globalization.CultureInfo.InvariantCulture),
                     theIsReplace, theMap);
        SetParameter("FixShape.FixFreeShellMode", theParameters.FixFreeShellMode, theIsReplace, theMap);
        SetParameter("FixShape.FixFreeFaceMode", theParameters.FixFreeFaceMode, theIsReplace, theMap);
        SetParameter("FixShape.FixFreeWireMode", theParameters.FixFreeWireMode, theIsReplace, theMap);
        SetParameter("FixShape.FixSameParameterMode", theParameters.FixSameParameterMode, theIsReplace, theMap);
        SetParameter("FixShape.FixSolidMode", theParameters.FixSolidMode, theIsReplace, theMap);
        SetParameter("FixShape.FixShellOrientationMode", theParameters.FixShellOrientationMode, theIsReplace, theMap);
        SetParameter("FixShape.CreateOpenSolidMode", theParameters.CreateOpenSolidMode, theIsReplace, theMap);
        SetParameter("FixShape.FixShellMode", theParameters.FixShellMode, theIsReplace, theMap);
        SetParameter("FixShape.FixFaceOrientationMode", theParameters.FixFaceOrientationMode, theIsReplace, theMap);
        SetParameter("FixShape.FixFaceMode", theParameters.FixFaceMode, theIsReplace, theMap);
        SetParameter("FixShape.FixWireMode", theParameters.FixWireMode, theIsReplace, theMap);
        SetParameter("FixShape.FixOrientationMode", theParameters.FixOrientationMode, theIsReplace, theMap);
        SetParameter("FixShape.FixAddNaturalBoundMode", theParameters.FixAddNaturalBoundMode, theIsReplace, theMap);
        SetParameter("FixShape.FixMissingSeamMode", theParameters.FixMissingSeamMode, theIsReplace, theMap);
        SetParameter("FixShape.FixSmallAreaWireMode", theParameters.FixSmallAreaWireMode, theIsReplace, theMap);
        SetParameter("FixShape.RemoveSmallAreaFaceMode", theParameters.RemoveSmallAreaFaceMode, theIsReplace, theMap);
        SetParameter("FixShape.FixIntersectingWiresMode", theParameters.FixIntersectingWiresMode, theIsReplace, theMap);
        SetParameter("FixShape.FixLoopWiresMode", theParameters.FixLoopWiresMode, theIsReplace, theMap);
        SetParameter("FixShape.FixSplitFaceMode", theParameters.FixSplitFaceMode, theIsReplace, theMap);
        SetParameter("FixShape.AutoCorrectPrecisionMode", theParameters.AutoCorrectPrecisionMode, theIsReplace, theMap);
        SetParameter("FixShape.ModifyTopologyMode", theParameters.ModifyTopologyMode, theIsReplace, theMap);
        SetParameter("FixShape.ModifyGeometryMode", theParameters.ModifyGeometryMode, theIsReplace, theMap);
        SetParameter("FixShape.ClosedWireMode", theParameters.ClosedWireMode, theIsReplace, theMap);
        SetParameter("FixShape.PreferencePCurveMode", theParameters.PreferencePCurveMode, theIsReplace, theMap);
        SetParameter("FixShape.FixReorderMode", theParameters.FixReorderMode, theIsReplace, theMap);
        SetParameter("FixShape.FixSmallMode", theParameters.FixSmallMode, theIsReplace, theMap);
        SetParameter("FixShape.FixConnectedMode", theParameters.FixConnectedMode, theIsReplace, theMap);
        SetParameter("FixShape.FixEdgeCurvesMode", theParameters.FixEdgeCurvesMode, theIsReplace, theMap);
        SetParameter("FixShape.FixDegeneratedMode", theParameters.FixDegeneratedMode, theIsReplace, theMap);
        SetParameter("FixShape.FixLackingMode", theParameters.FixLackingMode, theIsReplace, theMap);
        SetParameter("FixShape.FixSelfIntersectionMode", theParameters.FixSelfIntersectionMode, theIsReplace, theMap);
        SetParameter("FixShape.RemoveLoopMode", theParameters.RemoveLoopMode, theIsReplace, theMap);
        SetParameter("FixShape.FixReversed2dMode", theParameters.FixReversed2dMode, theIsReplace, theMap);
        SetParameter("FixShape.FixRemovePCurveMode", theParameters.FixRemovePCurveMode, theIsReplace, theMap);
        SetParameter("FixShape.FixRemoveCurve3dMode", theParameters.FixRemoveCurve3dMode, theIsReplace, theMap);
        SetParameter("FixShape.FixAddPCurveMode", theParameters.FixAddPCurveMode, theIsReplace, theMap);
        SetParameter("FixShape.FixAddCurve3dMode", theParameters.FixAddCurve3dMode, theIsReplace, theMap);
        SetParameter("FixShape.FixSeamMode", theParameters.FixSeamMode, theIsReplace, theMap);
        SetParameter("FixShape.FixShiftedMode", theParameters.FixShiftedMode, theIsReplace, theMap);
        SetParameter("FixShape.FixEdgeSameParameterMode", theParameters.FixEdgeSameParameterMode, theIsReplace, theMap);
        SetParameter("FixShape.FixNotchedEdgesMode", theParameters.FixNotchedEdgesMode, theIsReplace, theMap);
        SetParameter("FixShape.FixTailMode", theParameters.FixTailMode, theIsReplace, theMap);
        SetParameter("FixShape.MaxTailAngle", theParameters.MaxTailAngle, theIsReplace, theMap);
        SetParameter("FixShape.MaxTailWidth", theParameters.MaxTailWidth, theIsReplace, theMap);
        SetParameter("FixShape.FixSelfIntersectingEdgeMode", theParameters.FixSelfIntersectingEdgeMode, theIsReplace, theMap);
        SetParameter("FixShape.FixIntersectingEdgesMode", theParameters.FixIntersectingEdgesMode, theIsReplace, theMap);
        SetParameter("FixShape.FixNonAdjacentIntersectingEdgesMode", theParameters.FixNonAdjacentIntersectingEdgesMode,
                     theIsReplace, theMap);
        SetParameter("FixShape.FixVertexPositionMode", theParameters.FixVertexPositionMode, theIsReplace, theMap);
        SetParameter("FixShape.FixVertexToleranceMode", theParameters.FixVertexToleranceMode, theIsReplace, theMap);
    }

    /// <summary>
    /// Sets parameters for shape processing. Parameters from <paramref name="theParameters"/> are copied to the output
    /// map. Parameters from <paramref name="theAdditionalParameters"/> are copied to the output map if they are not
    /// present in <paramref name="theParameters"/>.
    /// </summary>
    public static void SetShapeFixParameters(in DE_ShapeFixParameters theParameters,
                                             NCollection_DataMap<string, string> theAdditionalParameters,
                                             NCollection_DataMap<string, string> theTargetParameterMap)
    {
        FillParameterMap(theParameters, true, theTargetParameterMap);
        foreach (KeyValuePair<string, string> aParamIter in theAdditionalParameters)
        {
            SetParameter(aParamIter.Key, aParamIter.Value, false, theTargetParameterMap);
        }
    }

    /// <summary>Set the parameter in the map (a fix mode, as its integer value).</summary>
    public static void SetParameter(string theKey, DE_ShapeFixParameters.FixMode theValue, bool theIsReplace,
                                    NCollection_DataMap<string, string> theMap)
    {
        SetParameter(theKey, ((int)theValue).ToString(System.Globalization.CultureInfo.InvariantCulture), theIsReplace, theMap);
    }

    /// <summary>Set the parameter in the map (a real, as TCollection_AsciiString writes it: <c>%g</c>).</summary>
    public static void SetParameter(string theKey, double theValue, bool theIsReplace, NCollection_DataMap<string, string> theMap)
    {
        SetParameter(theKey, new TCollection_HAsciiString(theValue).ToCString(), theIsReplace, theMap);
    }

    /// <summary>
    /// Set the parameter in the map. <paramref name="theIsReplace"/>: whether parameter should be replaced if it
    /// already exists in the map.
    /// </summary>
    public static void SetParameter(string theKey, string theValue, bool theIsReplace, NCollection_DataMap<string, string> theMap)
    {
        if (theMap.Find(theKey, out string anExisting))
        {
            if (!theIsReplace || anExisting == theValue)
            {
                return;
            }
        }
        theMap.Bind(theKey, theValue);
    }

    // PrepareForTransfer() (UnitsMethods::SetCasCadeLengthUnit from the static "xstep.cascade.unit", process-wide
    // state) is left out (decision 1203): a reader reads the unit from its static set.

    private readonly NCollection_DataMap<string, string> myParameters; // Parameters to be used in the processing.
    private ShapeProcess_ShapeContext? myContext;                      // Shape context.
}
