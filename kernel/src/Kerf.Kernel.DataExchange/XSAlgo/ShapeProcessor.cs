// Port of: src/DataExchange/TKXSBase/XSAlgo/XSAlgo_ShapeProcessor.hxx
// Port of: src/DataExchange/TKXSBase/XSAlgo/XSAlgo_ShapeProcessor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Healing.Process;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Exchange.Framework;

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
/// <see cref="StaticSet"/> the caller passes (decision 1181), after OCCT's own parameters.
/// <c>PrepareForTransfer</c>, which sets UnitsMethods' process-wide length unit from <c>xstep.cascade.unit</c>, is
/// left out (decision 1203).
/// </para>
/// <para>
/// <see cref="ProcessShape"/> never writes into its input (ShapeProcess's working copy, decision 1022) and returns a
/// frozen shape. <see cref="CheckPCurve(in Edge, in Face, double, bool, StaticSet)"/> writes into the edge it is given, as OCCT's: it is for a reader's edges
/// under construction, not for published shapes (a frozen edge throws).
/// </para>
/// </remarks>
internal class ShapeProcessor
{
    /// <summary>
    /// Constructor. <paramref name="parameters"/>: pre-filled parameter map to be used in the processing (copied).
    /// <paramref name="shapeFixParameters"/>: shape healing parameters to be used in the processing; if
    /// <paramref name="parameters"/> has some shape healing values, they will override the corresponding values from
    /// <paramref name="shapeFixParameters"/>.
    /// </summary>
    public ShapeProcessor(Dictionary<string, string> parameters, in ShapeFixParameters shapeFixParameters)
    {
        _parameters = new Dictionary<string, string>(parameters,parameters.Comparer);
        FillParameterMap(shapeFixParameters, false, _parameters);
    }

    /// <summary>Constructor, with the default shape healing parameters (OCCT's default argument <c>{}</c>).</summary>
    public ShapeProcessor(Dictionary<string, string> parameters)
        : this(parameters, new ShapeFixParameters())
    {
    }

    /// <summary>Constructor. <paramref name="parameters"/>: parameters to be used in the processing.</summary>
    public ShapeProcessor(in ShapeFixParameters parameters)
    {
        var map = new Dictionary<string, string>();
        FillParameterMap(parameters, false, map);
        _parameters = map;
    }

    /// <summary>
    /// Process the shape by applying the specified operations. Returns the processed shape (frozen); may be the same as
    /// the input shape if no modifications were made.
    /// </summary>
    public Shape ProcessShape(in Shape shape, global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags operations,
                                     ProgressRange progress = default)
    {
        if (shape.IsNull)
        {
            return shape;
        }
        InitializeContext(shape);
        return global::Kerf.Engine.Healing.Process.ShapeProcess.Perform(_context, operations, progress) ? _context.Result() : shape;
    }

    /// <summary>
    /// As <see cref="ProcessShape"/>, without freezing the result (Kerf, for STEPControl_ActorRead, decision 1273):
    /// a reader's working shapes are processed in place, as OCCT's, so that the items sharing them keep sharing them.
    /// </summary>
    internal Shape InternalProcessShape(in Shape shape, global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags operations,
                                               ProgressRange progress = default)
    {
        if (shape.IsNull)
        {
            return shape;
        }
        InitializeContext(shape);
        return global::Kerf.Engine.Healing.Process.ShapeProcess.Perform(_context, operations, progress) ? _context.InternalResult() : shape;
    }

    /// <summary>As <see cref="MergeTransferInfo(TransientProcess, int)"/>, without freezing the history's shapes (Kerf, decision 1273).</summary>
    internal void InternalMergeTransferInfo(TransientProcess transientProcess, int firstTPItemIndex)
    {
        if (_context is null)
        {
            return;
        }
        MergeShapeTransferInfo(transientProcess, _context.InternalMap(), firstTPItemIndex, _context.Messages());
    }

    /// <summary>Get the context of the last processing. Only valid after the ProcessShape() method was called.</summary>
    public ShapeContext? GetContext()
    {
        return _context;
    }

    // Initialize the context with the specified shape.
    private void InitializeContext(in Shape shape)
    {
        _context = new ShapeContext(shape, null);
        foreach (KeyValuePair<string, string> parameterIter in _parameters)
        {
            _context.ResourceManager().SetResource(parameterIter.Key, parameterIter.Value);
        }
        // Read and set detalization level.
        if (_parameters.TryGetValue("DetalizationLevel", out string result) && TCollection_AsciiString.IsIntegerValue(result))
        {
            ShapeType detalizationLevel = (ShapeType)TCollection_AsciiString.IntegerValue(result);
            _context.SetDetalisation(detalizationLevel);
        }
        // Read and set non-manifold flag.
        if (_parameters.TryGetValue("NonManifold", out result) && TCollection_AsciiString.IsIntegerValue(result))
        {
            bool nonManifold = TCollection_AsciiString.IntegerValue(result) != 0;
            _context.SetNonManifold(nonManifold);
        }
    }

    // Add messages from the specified shape to the transfer binder.
    private static void AddMessages(MsgRegistrator? messages, in Shape shape, Binder binder)
    {
        if (messages is null)
        {
            return;
        }
        if (!messages.MapShape().TryGetValue(shape, out NCollection_List<Msg> shapeMessages))
        {
            return;
        }
        for (var msgIter = new NCollection_List<Msg>.Iterator(shapeMessages); msgIter.More(); msgIter.Next())
        {
            Msg message = msgIter.Value();
            binder.AddWarning(message.Value(), message.Original());
        }
    }

    /// <summary>
    /// Merge the results of the shape processing with the transfer process: the shape binders of the process from
    /// <paramref name="firstTPItemIndex"/> get the shapes their results became, and the messages attached to them.
    /// </summary>
    public static void MergeShapeTransferInfo(TransientProcess transientProcess,
                                              Dictionary<Shape, Shape> modifiedShapesMap,
                                              int firstTPItemIndex,
                                              MsgRegistrator? messages)
    {
        if (modifiedShapesMap.Count == 0)
        {
            return;
        }
        bool toPrint = messages is not null && messages.MapShape().Count != 0;
        for (int i = Math.Max(firstTPItemIndex, 1); i <= transientProcess.MappedCount; ++i)
        {
            ShapeBinder? shapeBinder = transientProcess.MapItem(i) as ShapeBinder;
            if (shapeBinder is null || shapeBinder.Result().IsNull)
            {
                continue;
            }

            Shape originalShape = shapeBinder.Result();
            if (modifiedShapesMap.ContainsKey(originalShape))
            {
                shapeBinder.SetResult(modifiedShapesMap.GetValue(originalShape));
            }
            else if (!originalShape.Placement.IsIdentity)
            {
                var nullLoc = new Placement();
                Shape temporaryShape = originalShape.Located(nullLoc);
                if (modifiedShapesMap.ContainsKey(temporaryShape))
                {
                    shapeBinder.SetResult(modifiedShapesMap.GetValue(temporaryShape));
                }
            }
            else
            {
                // Some of edges may be modified.
                var reShaper = new ReShape();
                bool hasModifiedEdges = false;
                // Remember modifications.
                for (var expSE = new Explorer(originalShape, ShapeType.Edge); expSE.More(); expSE.Next())
                {
                    if (modifiedShapesMap.ContainsKey(expSE.Current()))
                    {
                        hasModifiedEdges = true;
                        Shape modifiedShape = modifiedShapesMap.GetValue(expSE.Current());
                        reShaper.Replace(expSE.Current(), modifiedShape);
                    }
                }
                // Apply modifications and store result in binder.
                if (hasModifiedEdges)
                {
                    Shape reshapedShape = reShaper.Apply(originalShape);
                    shapeBinder.SetResult(reshapedShape);
                }
            }

            // update messages
            if (toPrint)
            {
                AddMessages(messages, originalShape, shapeBinder);
            }
        }
    }

    /// <summary>
    /// Merge the results of the shape processing with the transfer process. <paramref name="firstTPItemIndex"/>:
    /// index of the first item in the transfer process to merge with.
    /// </summary>
    public void MergeTransferInfo(TransientProcess transientProcess, int firstTPItemIndex)
    {
        if (_context is null)
        {
            return;
        }
        MergeShapeTransferInfo(transientProcess, _context.Map(), firstTPItemIndex, _context.Messages());
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
    public static void MergeShapeTransferInfo(FinderProcess finderProcess,
                                              Dictionary<Shape, Shape> modifiedShapesMap,
                                              MsgRegistrator? messages)
    {
        if (modifiedShapesMap.Count == 0)
        {
            return;
        }
        bool toPrint = messages is not null && messages.MapShape().Count != 0;
        foreach (KeyValuePair<Shape, Shape> shapeShapeIterator in modifiedShapesMap)
        {
            Shape originalShape = shapeShapeIterator.Key;
            Shape resultShape = shapeShapeIterator.Value;

            ShapeMapper resultMapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(finderProcess, resultShape);
            Binder? resultBinder = finderProcess.Find(resultMapper);

            if (resultBinder is null)
            {
                resultBinder = new ShapeBinder(resultShape);
                // if <orig> shape was split, put entities corresponding to new shapes
                //  into Transfer_TransientListBinder.
                if (originalShape.Type > resultShape.Type)
                {
                    Shape subShape = new Shape();
                    var transientListBinder = new TransientListBinder();
                    for (var subShapeIter = new Iterator(resultShape); subShapeIter.More(); subShapeIter.Next())
                    {
                        Shape currentSubShape = subShapeIter.Value();
                        Finder? subShapeMapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(finderProcess, currentSubShape);
                        if (subShapeMapper is null)
                        {
                            continue;
                        }
                        Transient? transientResult = finderProcess.FindTransient(subShapeMapper);
                        if (transientResult is null)
                        {
                            continue;
                        }
                        transientListBinder.AddResult(transientResult);
                        subShape = currentSubShape;
                    }
                    if (transientListBinder.TransientCount == 1)
                    {
                        resultBinder = new ShapeBinder(subShape);
                    }
                    else if (transientListBinder.TransientCount > 1)
                    {
                        resultBinder.AddResult(transientListBinder);
                    }
                }
            }

            ShapeMapper originalMapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(finderProcess, originalShape);
            Binder? originalBinder = finderProcess.Find(originalMapper);
            if (originalBinder is null)
            {
                finderProcess.Bind(originalMapper, resultBinder);
            }
            else
            {
                originalBinder.AddResult(resultBinder);
            }

            // update messages
            if (toPrint)
            {
                AddMessages(messages, originalShape, resultBinder);
            }
        }
    }

    /// <summary>Merge the results of the shape processing with the finder process.</summary>
    public void MergeTransferInfo(FinderProcess finderProcess)
    {
        if (_context is null)
        {
            return;
        }
        MergeShapeTransferInfo(finderProcess, _context.Map(), _context.Messages());
    }

    // Create a new edge with the same geometry as the source edge.
    private static Edge MakeEdgeOnCurve(in Edge aSourceEdge)
    {
        Edge result = new Edge();

        var edgeAnalyzer = new EdgeAnalysis();
        if (!edgeAnalyzer.Curve3d(aSourceEdge, out Curve3D? sourceGeomCurve, out double startParam, out double endParam, false))
        {
            return result;
        }
        Point3D curveStartPt = sourceGeomCurve.Value(startParam);
        Point3D curveEndPt = sourceGeomCurve.Value(endParam);
        var edgeMaker = new MakeEdge(sourceGeomCurve, curveStartPt, curveEndPt, startParam, endParam);
        var sbe = new BuildEdgeBuilder();
        sbe.SetRange3d(edgeMaker.InternalEdge(), startParam, endParam);
        result = edgeMaker.InternalEdge(); // a working edge, not published
        return result;
    }

    /// <summary>
    /// Check quality of pcurve of the edge on the given face, and correct it if necessary. Returns True if the pcurve
    /// was corrected, false if it was dropped. Reads <c>read.stdsameparameter.mode</c> in <paramref name="statics"/>.
    /// </summary>
    /// <remarks>Writes into <paramref name="edge"/> and its vertices, which must not be frozen.</remarks>
    public static bool CheckPCurve(in Edge edge, in Face face, double precision, bool isSeam,
                                   StaticSet statics)
        => CheckPCurve(edge, face, precision, isSeam, statics.IVal("read.stdsameparameter.mode") != 0);

    /// <summary>
    /// As the form with statics, given the value of <c>read.stdsameparameter.mode</c> (Kerf addition: StepToTopoDS
    /// takes it from its model's parameters, decision 1246).
    /// </summary>
    internal static bool CheckPCurve(in Edge edge, in Face face, double precision, bool isSeam,
                                     bool stdSameParameterMode)
    {
        var edgeAnalyzer = new EdgeAnalysis();

        // Retrieve pcurve and its parameters.
        if (!edgeAnalyzer.PCurve(edge, face, out Curve2D? curve2D, out double curve2DParam1, out double curve2DParam2, false))
        {
            return false;
        }

        // Check for pcurve longer than surface.
        Surface surface = Tool.Surface(face);
        surface.ParameterBounds(out double faceSurfaceU1, out double faceSurfaceU2, out double faceSurfaceV1, out double faceSurfaceV2);
        Point2D curve2DPoint1 = curve2D.Value(curve2DParam1);
        Point2D curve2DPoint2 = curve2D.Value(curve2DParam2);
        // Multi-periodic? Better to discard (beware of infinite values)
        double edgeSpanX = Math.Abs(curve2DPoint1.X - curve2DPoint2.X);
        double edgeSpanY = Math.Abs(curve2DPoint1.Y - curve2DPoint2.Y);
        // So if span of pcurve along U or V is longer than 6/8 of the surface span, discard it.
        // Why exactly 6/8? No idea, but it's the same as in the original code.
        if (edgeSpanX / 8.0 > (faceSurfaceU2 / 6.0 - faceSurfaceU1 / 6.0)
            || edgeSpanY / 8.0 > (faceSurfaceV2 / 6.0 - faceSurfaceV1 / 6.0))
        {
            new BuildEdgeBuilder().RemovePCurve(edge, face);
            return false;
        }

        // Second Check: 2D and 3D consistency (if the Pcurve has not been dropped)
        // We also check that it does not wrap around too much...
        // Example: UVV in DEGREES on a surface in RADIANS, adjusted = 57 turns!
        edgeAnalyzer.Curve3d(edge, out Curve3D? curve3D, out double curve3DParam1, out double curve3DParam2, false);

        Point3D curve3DPoint1 = surface.Value(curve2DPoint1.X, curve2DPoint1.Y);
        Point3D curve3DPoint2 = surface.Value(curve2DPoint2.X, curve2DPoint2.Y);
        Vertex vertex1 = Exploration.FirstVertex(edge);
        Vertex vertex2 = Exploration.LastVertex(edge);
        Point3D pv1 = curve3D is null ? Tool.Pnt(vertex1) : curve3D.Value(curve3DParam1);
        Point3D pv2 = curve3D is null ? Tool.Pnt(vertex2) : curve3D.Value(curve3DParam2);
        double dist11 = pv1.DistanceTo(curve3DPoint1);
        double dist22 = pv2.DistanceTo(curve3DPoint2);
        if ((dist11 > precision) || (dist22 > precision))
        {
            new BuildEdgeBuilder().RemovePCurve(edge, face);
            return false;
        }

        // pdn checking deviation between pcurve and 3D curve
        // Make temporary edge for analysis
        if (curve3D is null)
        {
            return false;
        }
        Edge tmpEdge = MakeEdgeOnCurve(edge);

        // fill it with pcurve(s)
        var builder = new BrepBuilder();
        Curve2D? seamPCurve;
        if (isSeam)
        {
            Edge reversedEdge = ShapeCasts.Edge(edge.Reversed());
            if (!edgeAnalyzer.PCurve(reversedEdge, face, out seamPCurve, out double seamPCurveParam1, out double seamPCurveParam2, false)
                || seamPCurve == curve2D)
            {
                seamPCurve = curve2D.Copy() as Curve2D;
            }
            builder.UpdateEdge(tmpEdge, curve2D, seamPCurve, face, 0.0);
        }
        else
        {
            seamPCurve = null;
            builder.UpdateEdge(tmpEdge, curve2D, face, 0.0);
        }
        builder.Range(tmpEdge, face, curve2DParam1, curve2DParam2);
        builder.SameRange(tmpEdge, false);
        if (stdSameParameterMode)
        {
            builder.SameParameter(tmpEdge, false);
        }

        // call FixSP to see what it will do
        var edgeFixer = new EdgeFixer();
        edgeFixer.FixSameParameter(tmpEdge);
        double tolerance = Tool.Tolerance(tmpEdge);
        bool sameRangeFlag = Tool.SameRange(tmpEdge);
        bool sameParameterFlag = Tool.SameParameter(tmpEdge);

        // if result is not nice, try to call projection and take the best
        if (tolerance > Math.Min(1.0, 2.0 * precision) || !sameRangeFlag)
        {
            // pdn trying to recompute pcurve
            Edge edgePr = MakeEdgeOnCurve(edge);
            edgeFixer.FixAddPCurve(edgePr, face, isSeam, precision);
            edgeFixer.FixSameParameter(edgePr);
            double tolerancePr = Tool.Tolerance(edgePr);
            // pdn choose the best pcurve
            if (tolerancePr < tolerance || !sameRangeFlag)
            {
                sameRangeFlag = Tool.SameRange(edgePr);
                sameParameterFlag = Tool.SameParameter(edgePr);
                tolerance = tolerancePr;
                tmpEdge = edgePr;
            }
        }

        // get corrected pcurve from the temporary edge, and put to original
        edgeAnalyzer.PCurve(tmpEdge, face, out curve2D, out curve2DParam1, out curve2DParam2, false);
        if (isSeam)
        {
            Edge reversedTmpEdge = ShapeCasts.Edge(tmpEdge.Reversed());
            edgeAnalyzer.PCurve(reversedTmpEdge, face, out seamPCurve, out double reversedTmpEdgeParam1,
                                  out double reversedTmpEdgeParam2, false);
            if (edge.Orientation == Reversed) //: abv 14.11.01: coneEl.sat loop
            {
                builder.UpdateEdge(edge, seamPCurve, curve2D, face, tolerance);
            }
            else
            {
                builder.UpdateEdge(edge, curve2D, seamPCurve, face, tolerance);
            }
        }
        else
        {
            builder.UpdateEdge(edge, curve2D, face, tolerance);
        }

        builder.UpdateVertex(vertex1, tolerance);
        builder.UpdateVertex(vertex2, tolerance);
        builder.Range(edge, face, curve2DParam1, curve2DParam2);
        if (Tool.SameRange(edge))
        {
            builder.SameRange(edge, sameRangeFlag);
        }
        if (Tool.SameParameter(edge))
        {
            builder.SameParameter(edge, sameParameterFlag);
        }

        return true;
    }

    /// <summary>
    /// Reads the parameter map and the operation flags from the resource file named by the static
    /// <paramref name="fileResourceName"/> (for example "read.step.resource.name", which may hold "STEP"), in the
    /// scope named by the static <paramref name="scopeResourceName"/> (for example "read.step.sequence", which may
    /// hold "FromSTEP"). If the resource file isn't found, the statics' values are used instead.
    /// </summary>
    public static (Dictionary<string, string>, global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags) ReadProcessingData(
        string fileResourceName, string scopeResourceName, StaticSet statics)
    {
        string fileName = statics.CVal(fileResourceName);
        var context = new ShapeContext(new Shape(), fileName);
        if (!context.ResourceManager().IsInitialized)
        {
            // If resource file wasn't found, use static values instead
            statics.FillMap(context.ResourceManager().GetMap());
        }
        string scope = statics.CVal(scopeResourceName);

        // Copy parameters to the result.
        var resultParameters = new Dictionary<string, string>();
        var resultFlags = new global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags();
        Dictionary<string, string> map = context.ResourceManager().GetMap();
        foreach (KeyValuePair<string, string> iter in map)
        {
            string key = iter.Key;
            if (!key.StartsWith(scope, StringComparison.Ordinal))
            {
                // Ignore all parameters that don't start with the specified scope.
                continue;
            }
            // Remove scope prefix and dot
            // "FromIGES.FixShape.FixFreeFaceMode" -> "FixShape.FixFreeFaceMode"
            key = SubString(key, scope.Length + 2, key.Length);
            if (key == "exec.op")
            {
                // Parse operation flags using Token method
                int tokenCount = 1;
                string token;
                const string separators = " \t,;";
                while ((token = TCollection_AsciiString.Token(iter.Value, separators, tokenCount)).Length != 0)
                {
                    (global::Kerf.Engine.Healing.Process.ShapeProcess.Operation, bool) operationFlag = global::Kerf.Engine.Healing.Process.ShapeProcess.ToOperationFlag(token);
                    if (operationFlag.Item2)
                    {
                        resultFlags = resultFlags.With(operationFlag.Item1);
                    }
                    tokenCount++;
                }
            }
            else
            {
                resultParameters[key] = iter.Value;
            }
        }
        return (resultParameters, resultFlags);
    }

    // TCollection_AsciiString::SubString: the characters FromIndex to ToIndex (1-based)
    private static string SubString(string theString, int fromIndex, int toIndex)
    {
        if (toIndex > theString.Length || fromIndex <= 0 || fromIndex > toIndex)
        {
            throw new OutOfRangeException("TCollection_AsciiString::SubString : parameters out of range");
        }
        return theString.Substring(fromIndex - 1, toIndex - fromIndex + 1);
    }

    /// <summary>
    /// Fill the parameter map with the values from the specified parameters. <paramref name="isReplace"/>: whether
    /// parameter should be replaced if it already exists in the map.
    /// </summary>
    public static void FillParameterMap(in ShapeFixParameters parameters, bool isReplace,
                                        Dictionary<string, string> map)
    {
        SetParameter("FixShape.Tolerance3d", parameters.Tolerance3d, isReplace, map);
        SetParameter("FixShape.MaxTolerance3d", parameters.MaxTolerance3d, isReplace, map);
        SetParameter("FixShape.MinTolerance3d", parameters.MinTolerance3d, isReplace, map);
        SetParameter("DetalizationLevel", ((int)parameters.DetalizationLevel).ToString(System.Globalization.CultureInfo.InvariantCulture),
                     isReplace, map);
        SetParameter("NonManifold", (parameters.NonManifold ? 1 : 0).ToString(System.Globalization.CultureInfo.InvariantCulture),
                     isReplace, map);
        SetParameter("FixShape.FixFreeShellMode", parameters.FixFreeShellMode, isReplace, map);
        SetParameter("FixShape.FixFreeFaceMode", parameters.FixFreeFaceMode, isReplace, map);
        SetParameter("FixShape.FixFreeWireMode", parameters.FixFreeWireMode, isReplace, map);
        SetParameter("FixShape.FixSameParameterMode", parameters.FixSameParameterMode, isReplace, map);
        SetParameter("FixShape.FixSolidMode", parameters.FixSolidMode, isReplace, map);
        SetParameter("FixShape.FixShellOrientationMode", parameters.FixShellOrientationMode, isReplace, map);
        SetParameter("FixShape.CreateOpenSolidMode", parameters.CreateOpenSolidMode, isReplace, map);
        SetParameter("FixShape.FixShellMode", parameters.FixShellMode, isReplace, map);
        SetParameter("FixShape.FixFaceOrientationMode", parameters.FixFaceOrientationMode, isReplace, map);
        SetParameter("FixShape.FixFaceMode", parameters.FixFaceMode, isReplace, map);
        SetParameter("FixShape.FixWireMode", parameters.FixWireMode, isReplace, map);
        SetParameter("FixShape.FixOrientationMode", parameters.FixOrientationMode, isReplace, map);
        SetParameter("FixShape.FixAddNaturalBoundMode", parameters.FixAddNaturalBoundMode, isReplace, map);
        SetParameter("FixShape.FixMissingSeamMode", parameters.FixMissingSeamMode, isReplace, map);
        SetParameter("FixShape.FixSmallAreaWireMode", parameters.FixSmallAreaWireMode, isReplace, map);
        SetParameter("FixShape.RemoveSmallAreaFaceMode", parameters.RemoveSmallAreaFaceMode, isReplace, map);
        SetParameter("FixShape.FixIntersectingWiresMode", parameters.FixIntersectingWiresMode, isReplace, map);
        SetParameter("FixShape.FixLoopWiresMode", parameters.FixLoopWiresMode, isReplace, map);
        SetParameter("FixShape.FixSplitFaceMode", parameters.FixSplitFaceMode, isReplace, map);
        SetParameter("FixShape.AutoCorrectPrecisionMode", parameters.AutoCorrectPrecisionMode, isReplace, map);
        SetParameter("FixShape.ModifyTopologyMode", parameters.ModifyTopologyMode, isReplace, map);
        SetParameter("FixShape.ModifyGeometryMode", parameters.ModifyGeometryMode, isReplace, map);
        SetParameter("FixShape.ClosedWireMode", parameters.ClosedWireMode, isReplace, map);
        SetParameter("FixShape.PreferencePCurveMode", parameters.PreferencePCurveMode, isReplace, map);
        SetParameter("FixShape.FixReorderMode", parameters.FixReorderMode, isReplace, map);
        SetParameter("FixShape.FixSmallMode", parameters.FixSmallMode, isReplace, map);
        SetParameter("FixShape.FixConnectedMode", parameters.FixConnectedMode, isReplace, map);
        SetParameter("FixShape.FixEdgeCurvesMode", parameters.FixEdgeCurvesMode, isReplace, map);
        SetParameter("FixShape.FixDegeneratedMode", parameters.FixDegeneratedMode, isReplace, map);
        SetParameter("FixShape.FixLackingMode", parameters.FixLackingMode, isReplace, map);
        SetParameter("FixShape.FixSelfIntersectionMode", parameters.FixSelfIntersectionMode, isReplace, map);
        SetParameter("FixShape.RemoveLoopMode", parameters.RemoveLoopMode, isReplace, map);
        SetParameter("FixShape.FixReversed2dMode", parameters.FixReversed2dMode, isReplace, map);
        SetParameter("FixShape.FixRemovePCurveMode", parameters.FixRemovePCurveMode, isReplace, map);
        SetParameter("FixShape.FixRemoveCurve3dMode", parameters.FixRemoveCurve3dMode, isReplace, map);
        SetParameter("FixShape.FixAddPCurveMode", parameters.FixAddPCurveMode, isReplace, map);
        SetParameter("FixShape.FixAddCurve3dMode", parameters.FixAddCurve3dMode, isReplace, map);
        SetParameter("FixShape.FixSeamMode", parameters.FixSeamMode, isReplace, map);
        SetParameter("FixShape.FixShiftedMode", parameters.FixShiftedMode, isReplace, map);
        SetParameter("FixShape.FixEdgeSameParameterMode", parameters.FixEdgeSameParameterMode, isReplace, map);
        SetParameter("FixShape.FixNotchedEdgesMode", parameters.FixNotchedEdgesMode, isReplace, map);
        SetParameter("FixShape.FixTailMode", parameters.FixTailMode, isReplace, map);
        SetParameter("FixShape.MaxTailAngle", parameters.MaxTailAngle, isReplace, map);
        SetParameter("FixShape.MaxTailWidth", parameters.MaxTailWidth, isReplace, map);
        SetParameter("FixShape.FixSelfIntersectingEdgeMode", parameters.FixSelfIntersectingEdgeMode, isReplace, map);
        SetParameter("FixShape.FixIntersectingEdgesMode", parameters.FixIntersectingEdgesMode, isReplace, map);
        SetParameter("FixShape.FixNonAdjacentIntersectingEdgesMode", parameters.FixNonAdjacentIntersectingEdgesMode,
                     isReplace, map);
        SetParameter("FixShape.FixVertexPositionMode", parameters.FixVertexPositionMode, isReplace, map);
        SetParameter("FixShape.FixVertexToleranceMode", parameters.FixVertexToleranceMode, isReplace, map);
    }

    /// <summary>
    /// Sets parameters for shape processing. Parameters from <paramref name="parameters"/> are copied to the output
    /// map. Parameters from <paramref name="additionalParameters"/> are copied to the output map if they are not
    /// present in <paramref name="parameters"/>.
    /// </summary>
    public static void SetShapeFixParameters(in ShapeFixParameters parameters,
                                             Dictionary<string, string> additionalParameters,
                                             Dictionary<string, string> targetParameterMap)
    {
        FillParameterMap(parameters, true, targetParameterMap);
        foreach (KeyValuePair<string, string> paramIter in additionalParameters)
        {
            SetParameter(paramIter.Key, paramIter.Value, false, targetParameterMap);
        }
    }

    /// <summary>Set the parameter in the map (a fix mode, as its integer value).</summary>
    public static void SetParameter(string key, ShapeFixParameters.FixMode value, bool isReplace,
                                    Dictionary<string, string> map)
    {
        SetParameter(key, ((int)value).ToString(System.Globalization.CultureInfo.InvariantCulture), isReplace, map);
    }

    /// <summary>Set the parameter in the map (a real, as TCollection_AsciiString writes it: <c>%g</c>).</summary>
    public static void SetParameter(string key, double value, bool isReplace, Dictionary<string, string> map)
    {
        SetParameter(key, new TCollection_HAsciiString(value).ToCString(), isReplace, map);
    }

    /// <summary>
    /// Set the parameter in the map. <paramref name="isReplace"/>: whether parameter should be replaced if it
    /// already exists in the map.
    /// </summary>
    public static void SetParameter(string key, string value, bool isReplace, Dictionary<string, string> map)
    {
        if (map.TryGetValue(key, out string existing))
        {
            if (!isReplace || existing == value)
            {
                return;
            }
        }
        map[key] = value;
    }

    // PrepareForTransfer() (UnitsMethods::SetCasCadeLengthUnit from the static "xstep.cascade.unit", process-wide
    // state) is left out (decision 1203): a reader reads the unit from its static set.

    private readonly Dictionary<string, string> _parameters; // Parameters to be used in the processing.
    private ShapeContext? _context;                      // Shape context.
}
