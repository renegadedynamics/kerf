# Diagnostics: the codes

*Unit 10F, 3 November 2026. The types are in [README.md](README.md), section 2; decision 1349 (question 61) made the
codes strings.*

A `Diagnostic` has a `Severity` (warning or error), a `Code`, a plain `Message` and the sub-shapes it concerns
(`diagnostic.Shapes`, from Kerf.Kernel.ModelingData). The code is for programs and stays stable; the message is for
people and may be reworded.

**The form of a code:** lower case, dot-separated words of letters, digits and hyphens, the first word the area:
`boolean.self-interfering-shape`, `construction.disconnected-wire`. `DiagnosticCode` refuses anything else.

**Where codes come from today.** The public operations don't exist yet (unit 10L writes them), so the codes are made at
the engine's edge by four bridges, each in the assembly of what it reads:

| Bridge | Assembly | Reads |
|---|---|---|
| `FailureDiagnostics` | Foundation | An exception that escaped the engine |
| `AlertDiagnostics` | ModelingData | A `Report`'s alerts (the Booleans'), with their shapes |
| `StatusDiagnostics`, `OperationDiagnostics` | ModelingAlgorithms | The modelling algorithms' status enums; a failed operation's exception or report |
| `ExchangeDiagnostics` | DataExchange | The exchange framework's `ReturnStatus` |

10L's operations add their own codes in the same form, as static properties per area where a program is likely to test
for them, and list them here.

## General

| Code | Meaning |
|---|---|
| `input.invalid` | The input is invalid for the operation (`DiagnosticCode.InvalidInput`) |
| `numerics.not-converged` | An iterative computation didn't converge (`DiagnosticCode.NotConverged`) |
| `operation.cancelled` | The operation was cancelled (`DiagnosticCode.Cancelled`; cancellation itself throws `OperationCanceledException`) |
| `result.default` | A `default(Result<T>)`, never given a value or an error (`DiagnosticCode.DefaultResult`) |
| `result.invalid` | What the operation made isn't valid (`DiagnosticCode.InvalidResult`). The failure names the sub-shapes, and usually carries the shape for `TryGetInvalid` (README, section 2.2; decision 1742) |
| `engine.fault` | An exception from inside the engine that isn't one of OCCT's (an index out of range, a null reference), or a status value outside its enum: a fault in the port |

## Exceptions from the engine

An OCCT exception's code is `engine.` and its class name without the package, in lower case with hyphens between
words (`StdFail_NotDone` is `engine.not-done`); the Booleans' own failure is `boolean.failed`. The message is the kind
of failure, then OCCT's message if it has one, without the OCCT names: `the algorithm couldn't complete: command not done`
(decision 1662). A
Boolean's failure (`BRepAlgoAPI_Failure`) carries its report, and `OperationDiagnostics.FromException` gives the
report's alerts in its place (next section). Several of OCCT's classes share a code where they mean the same
(`Geom_UndefinedValue`, `Geom2d_UndefinedValue` and `StdFail_UndefinedValue`).

| Code | OCCT exception | Message (before OCCT's) |
|---|---|---|
| `boolean.failed` | `BRepAlgoAPI_Failure` | the algorithm couldn't complete: The Boolean operation failed: not done |
| `engine.abortive-transaction` | `Standard_AbortiveTransaction` | the operation failed |
| `engine.bad-continuity` | `LProp_BadContinuity` | the operation failed |
| `engine.bad-qualifier` | `GccEnt_BadQualifier` | a value is outside what the operation accepts |
| `engine.check-failure` | `Interface_CheckFailure` | the operation failed |
| `engine.construction-error` | `Standard_ConstructionError` | the geometry can't be constructed |
| `engine.dimension-error` | `Standard_DimensionError` | a value is outside what the operation accepts |
| `engine.dimension-mismatch` | `Standard_DimensionMismatch` | a value is outside what the operation accepts |
| `engine.divide-by-zero` | `Standard_DivideByZero` | a numeric error |
| `engine.domain-error` | `Standard_DomainError` | a value is outside what the operation accepts |
| `engine.failure` | `Standard_Failure` | the operation failed |
| `engine.immutable-object` | `Standard_ImmutableObject` | a value is outside what the operation accepts |
| `engine.infinite-solutions` | `StdFail_InfiniteSolutions` | there are infinitely many solutions |
| `engine.interface-error` | `Interface_InterfaceError` | the operation failed |
| `engine.interface-mismatch` | `Interface_InterfaceMismatch` | the operation failed |
| `engine.is-parallel` | `Geom2dGcc_IsParallel` | a value is outside what the operation accepts |
| `engine.multiply-defined` | `Standard_MultiplyDefined` | a value is outside what the operation accepts |
| `engine.negative-value` | `Standard_NegativeValue` | a value is out of range |
| `engine.no-more-object` | `Standard_NoMoreObject` | something looked for isn't there |
| `engine.no-solution` | `GccAna_NoSolution` | the operation failed |
| `engine.no-such-object` | `Standard_NoSuchObject` | something looked for isn't there |
| `engine.no-such-resource` | `Resource_NoSuchResource` | something looked for isn't there |
| `engine.not-defined` | `LProp_NotDefined` | the operation failed |
| `engine.not-done` | `StdFail_NotDone` | the algorithm couldn't complete |
| `engine.not-implemented` | `Standard_NotImplemented` | not implemented |
| `engine.not-square` | `math_NotSquare` | a value is outside what the operation accepts |
| `engine.null-object` | `Standard_NullObject` | something needed is missing |
| `engine.null-value` | `Standard_NullValue` | a value is out of range |
| `engine.numeric-error` | `Standard_NumericError` | a numeric error |
| `engine.out-of-range` | `Standard_OutOfRange` | a value is out of range |
| `engine.overflow` | `Standard_Overflow` | a numeric error |
| `engine.program-error` | `Standard_ProgramError` | the operation failed |
| `engine.range-error` | `Standard_RangeError` | a value is out of range |
| `engine.singular-matrix` | `math_SingularMatrix` | the operation failed |
| `engine.stream-format-error` | `Storage_StreamFormatError` | the data is malformed |
| `engine.transfer-dead-loop` | `Transfer_TransferDeadLoop` | the operation failed |
| `engine.transfer-failure` | `Transfer_TransferFailure` | the operation failed |
| `engine.type-mismatch` | `Standard_TypeMismatch` | a shape or geometry is of the wrong type |
| `engine.un-compatible-shape` | `BRepExtrema_UnCompatibleShape` | a value is outside what the operation accepts |
| `engine.un-compatible-shapes` | `TopoDS_UnCompatibleShapes` | a value is outside what the operation accepts |
| `engine.undefined` | `StdFail_Undefined` | the operation failed |
| `engine.undefined-axis` | `GProp_UndefinedAxis` | a value is outside what the operation accepts |
| `engine.undefined-derivative` | `StdFail_UndefinedDerivative` | a value is outside what the operation accepts |
| `engine.undefined-derivative` | `Geom2d_UndefinedDerivative` | a value is outside what the operation accepts |
| `engine.undefined-derivative` | `Geom_UndefinedDerivative` | a value is outside what the operation accepts |
| `engine.undefined-value` | `StdFail_UndefinedValue` | a value is outside what the operation accepts |
| `engine.undefined-value` | `Geom2d_UndefinedValue` | a value is outside what the operation accepts |
| `engine.undefined-value` | `Geom_UndefinedValue` | a value is outside what the operation accepts |
| `engine.underflow` | `Standard_Underflow` | a numeric error |
| `engine.vector-with-null-magnitude` | `gp_VectorWithNullMagnitude` | a vector or direction has zero length |

## Alerts

A `Report`'s alerts: its failures become errors, its alarms and warnings warnings (information and traces are left
out). The code is the area (`boolean` for BOPAlgo's; `engine` for any other package's), a dot, and the alert's class
name without the package and the word `Alert`. The message is the alert's text in OCCT's message table where it has
one, else the code's words. An alert that holds a shape names it in `Shapes`.

| Code | OCCT alert | Message |
|---|---|---|
| `boolean.acquired-self-intersection` | `BOPAlgo_AlertAcquiredSelfIntersection` | Some sub-shapes of some of the argument become connected through other shapes and the argument became self-interfered |
| `boolean.bad-positioning` | `BOPAlgo_AlertBadPositioning` | The positioning of the shapes leads to creation of the small edges without valid range |
| `boolean.bop-not-allowed` | `BOPAlgo_AlertBOPNotAllowed` | Boolean operation of the given type is not allowed on the given inputs |
| `boolean.bop-not-set` | `BOPAlgo_AlertBOPNotSet` | Bop not set |
| `boolean.builder-failed` | `BOPAlgo_AlertBuilderFailed` | Building of the result shape has failed |
| `boolean.building-p-curve-failed` | `BOPAlgo_AlertBuildingPCurveFailed` | Building 2D curve of edge on face has failed |
| `boolean.empty-shape` | `BOPAlgo_AlertEmptyShape` | Some of the arguments are empty shapes |
| `boolean.face-builder-unused-edges` | `BOPAlgo_AlertFaceBuilderUnusedEdges` | Some of the edges passed to the Face Builder algorithm have not been classified and not used for faces creation |
| `boolean.intersection-failed` | `BOPAlgo_AlertIntersectionFailed` | The intersection of the arguments has failed |
| `boolean.intersection-of-pair-of-shapes-failed` | `BOPAlgo_AlertIntersectionOfPairOfShapesFailed` | Intersection of pair of shapes has failed |
| `boolean.multi-dimensional-arguments` | `BOPAlgo_AlertMultiDimensionalArguments` | Multi-dimensional arguments |
| `boolean.multiple-arguments` | `BOPAlgo_AlertMultipleArguments` | More than one argument is provided |
| `boolean.no-faces-to-remove` | `BOPAlgo_AlertNoFacesToRemove` | No faces have been found for removal |
| `boolean.no-filler` | `BOPAlgo_AlertNoFiller` | The Pave Filler (the intersection tool) has not been created |
| `boolean.no-periodicity-required` | `BOPAlgo_AlertNoPeriodicityRequired` | No periodicity has been requested for the shape |
| `boolean.not-splittable-edge` | `BOPAlgo_AlertNotSplittableEdge` | Some edges are very small and have such a small valid range, that they cannot be split |
| `boolean.null-input-shapes` | `BOPAlgo_AlertNullInputShapes` | No or null input shapes |
| `boolean.post-treat-ff` | `BOPAlgo_AlertPostTreatFF` | Cannot connect face intersection curves |
| `boolean.removal-of-ib-for-edges-failed` | `BOPAlgo_AlertRemovalOfIBForEdgesFailed` | Removal of internal boundaries among Edges has failed |
| `boolean.removal-of-ib-for-faces-failed` | `BOPAlgo_AlertRemovalOfIBForFacesFailed` | Removal of internal boundaries among Faces has failed |
| `boolean.removal-of-ib-for-m-dim-shapes` | `BOPAlgo_AlertRemovalOfIBForMDimShapes` | Removal of internal boundaries among the multi-dimensional shapes is not supported yet |
| `boolean.removal-of-ib-for-solids-failed` | `BOPAlgo_AlertRemovalOfIBForSolidsFailed` | Removal of internal boundaries among Solids has failed |
| `boolean.remove-features-failed` | `BOPAlgo_AlertRemoveFeaturesFailed` | The Feature Removal algorithm has failed |
| `boolean.self-interfering-shape` | `BOPAlgo_AlertSelfInterferingShape` | Some of the arguments are self-interfering shapes |
| `boolean.shape-is-not-periodic` | `BOPAlgo_AlertShapeIsNotPeriodic` | The shape is not periodic |
| `boolean.shell-splitter-failed` | `BOPAlgo_AlertShellSplitterFailed` | Unable to build loops from the given faces |
| `boolean.solid-builder-failed` | `BOPAlgo_AlertSolidBuilderFailed` | Building Fused solid has failed |
| `boolean.solid-builder-unused-faces` | `BOPAlgo_AlertSolidBuilderUnusedFaces` | Some of the faces passed to the Solid Builder algorithm have not been classified and not used for solids creation |
| `boolean.too-few-arguments` | `BOPAlgo_AlertTooFewArguments` | There are no enough arguments to perform the operation |
| `boolean.too-small-edge` | `BOPAlgo_AlertTooSmallEdge` | Some edges are too small and have no valid range |
| `boolean.unable-to-glue` | `BOPAlgo_AlertUnableToGlue` | Unable to glue the shapes |
| `boolean.unable-to-make-closed-edge-on-face` | `BOPAlgo_AlertUnableToMakeClosedEdgeOnFace` | Unable to make closed edge on face. |
| `boolean.unable-to-make-identical` | `BOPAlgo_AlertUnableToMakeIdentical` | Unable to make the shape to look identical on opposite sides (Splitter fails) |
| `boolean.unable-to-make-periodic` | `BOPAlgo_AlertUnableToMakePeriodic` | Unable to make the shape periodic |
| `boolean.unable-to-orient-the-shape` | `BOPAlgo_AlertUnableToOrientTheShape` | Unable to orient the shape correctly |
| `boolean.unable-to-remove-the-feature` | `BOPAlgo_AlertUnableToRemoveTheFeature` | Unable to remove the feature |
| `boolean.unable-to-repeat` | `BOPAlgo_AlertUnableToRepeat` | Unable to repeat the shape (Gluer fails) |
| `boolean.unable-to-trim` | `BOPAlgo_AlertUnableToTrim` | Unable to trim the shape for making it periodic (BOP Common fails) |
| `boolean.unknown-shape` | `BOPAlgo_AlertUnknownShape` | Shape is unknown for operation |
| `boolean.unsupported-type` | `BOPAlgo_AlertUnsupportedType` | Unsupported type of input shape |
| `boolean.user-break` | `BOPAlgo_AlertUserBreak` | User break |

## Statuses

A status that says the work was done gives no diagnostic; each other value is an error.

### Construction

| Code | OCCT status | Message |
|---|---|---|
| `construction.point-projection-failed` | `BRepBuilderAPI_EdgeError::BRepBuilderAPI_PointProjectionFailed` | An end point couldn't be projected onto the curve. |
| `construction.parameter-out-of-range` | `BRepBuilderAPI_EdgeError::BRepBuilderAPI_ParameterOutOfRange` | An end parameter is outside the curve's range. |
| `construction.different-points-on-closed-curve` | `BRepBuilderAPI_EdgeError::BRepBuilderAPI_DifferentPointsOnClosedCurve` | The end points of an edge on a closed curve differ where they should coincide. |
| `construction.point-with-infinite-parameter` | `BRepBuilderAPI_EdgeError::BRepBuilderAPI_PointWithInfiniteParameter` | An end point was given at an infinite parameter. |
| `construction.point-and-parameter-differ` | `BRepBuilderAPI_EdgeError::BRepBuilderAPI_DifferentsPointAndParameter` | An end point isn't at the curve's point for its parameter. |
| `construction.line-through-identical-points` | `BRepBuilderAPI_EdgeError::BRepBuilderAPI_LineThroughIdenticPoints` | A line can't pass through two identical points. |
| `construction.empty-wire` | `BRepBuilderAPI_WireError::BRepBuilderAPI_EmptyWire` | The wire has no edges. |
| `construction.disconnected-wire` | `BRepBuilderAPI_WireError::BRepBuilderAPI_DisconnectedWire` | An edge isn't connected to the rest of the wire. |
| `construction.non-manifold-wire` | `BRepBuilderAPI_WireError::BRepBuilderAPI_NonManifoldWire` | The edges branch: more than two meet at a vertex. |
| `construction.no-face` | `BRepBuilderAPI_FaceError::BRepBuilderAPI_NoFace` | No face was built: the input gave no surface. |
| `construction.not-planar` | `BRepBuilderAPI_FaceError::BRepBuilderAPI_NotPlanar` | The wire isn't planar, so no plane holds it. |
| `construction.curve-projection-failed` | `BRepBuilderAPI_FaceError::BRepBuilderAPI_CurveProjectionFailed` | An edge's curve couldn't be projected onto the surface. |
| `construction.parameters-out-of-range` | `BRepBuilderAPI_FaceError::BRepBuilderAPI_ParametersOutOfRange` | The parameters are outside the surface's range. |
| `construction.empty-shell` | `BRepBuilderAPI_ShellError::BRepBuilderAPI_EmptyShell` | The shell has no faces. |
| `construction.disconnected-shell` | `BRepBuilderAPI_ShellError::BRepBuilderAPI_DisconnectedShell` | A face isn't connected to the rest of the shell. |
| `construction.shell-parameters-out-of-range` | `BRepBuilderAPI_ShellError::BRepBuilderAPI_ShellParametersOutOfRange` | The parameters are outside the surface's range. |
| `construction.pipe-not-done` | `BRepBuilderAPI_PipeError::BRepBuilderAPI_PipeNotDone` | The sweep couldn't be built. |
| `construction.plane-not-intersecting-guide` | `BRepBuilderAPI_PipeError::BRepBuilderAPI_PlaneNotIntersectGuide` | A section's plane doesn't meet the guide. |
| `construction.impossible-contact` | `BRepBuilderAPI_PipeError::BRepBuilderAPI_ImpossibleContact` | The profile can't keep contact with the guide. |
| `construction.loft-not-done` | `BRepFill_ThruSectionErrorStatus::BRepFill_ThruSectionErrorStatus_NotDone` | The loft couldn't be built. |
| `construction.loft-sections-differ` | `BRepFill_ThruSectionErrorStatus::BRepFill_ThruSectionErrorStatus_NotSameTopology` | The sections don't have the same number of edges, or are mixed open and closed. |
| `construction.loft-sections-inconsistent` | `BRepFill_ThruSectionErrorStatus::BRepFill_ThruSectionErrorStatus_ProfilesInconsistent` | The sections are inconsistent: a vertex is given where a wire is needed, or the reverse. |
| `construction.loft-wrong-usage` | `BRepFill_ThruSectionErrorStatus::BRepFill_ThruSectionErrorStatus_WrongUsage` | A vertex section is allowed only first or last. |
| `construction.loft-null-curve` | `BRepFill_ThruSectionErrorStatus::BRepFill_ThruSectionErrorStatus_Null3DCurve` | An edge of a section has no 3D curve. |
| `construction.loft-failed` | `BRepFill_ThruSectionErrorStatus::BRepFill_ThruSectionErrorStatus_Failed` | The loft's surfaces couldn't be built. |

### Fillets and chamfers

| Code | OCCT status | Message |
|---|---|---|
| `fillet.failed` | `ChFiDS_ErrorStatus::ChFiDS_Error` | The fillet couldn't be built. |
| `fillet.walking-failed` | `ChFiDS_ErrorStatus::ChFiDS_WalkingFailure` | The fillet's surface couldn't be followed along the edges. |
| `fillet.no-starting-solution` | `ChFiDS_ErrorStatus::ChFiDS_StartsolFailure` | No starting point was found for the fillet's surface. |
| `fillet.twisted-surface` | `ChFiDS_ErrorStatus::ChFiDS_TwistedSurface` | The fillet's surface would be twisted. |
| `fillet.radius-changes-at-a-corner` | none (Kerf's `ErrorStatus.EnvelopeCorner`) | The fillet's radius is changing where the path of the rolling ball's centre has a corner (where its edges' curvature jumps, where the ball comes to rest on an edge or leaves it, or where the radius's rate of change jumps): the surface the ball sweeps has a piece of the ball or a crease there, which couldn't be built. Also where a radius changes so sharply that the surface would fold back on itself. |
| `fillet.creased` | none (Kerf's; a warning on a fillet that is returned: decision 2421) | The fillet's surface has a crease of N degrees at (x, y, z): its radius is changing where the path of the rolling ball's centre has a corner, and the surface the ball sweeps isn't smooth there. |

### Offsets

| Code | OCCT status | Message |
|---|---|---|
| `offset.failed` | `BRepOffset_Error::BRepOffset_UnknownError` | The offset couldn't be built. |
| `offset.bad-normals` | `BRepOffset_Error::BRepOffset_BadNormalsOnGeometry` | A face's normal is undefined somewhere. |
| `offset.c0-geometry` | `BRepOffset_Error::BRepOffset_C0Geometry` | A face's surface is only C0 continuous, so it can't be offset. |
| `offset.null-offset` | `BRepOffset_Error::BRepOffset_NullOffset` | The offset distance is zero. |
| `offset.not-connected-shell` | `BRepOffset_Error::BRepOffset_NotConnectedShell` | The shell's faces aren't connected. |
| `offset.cannot-trim-edges` | `BRepOffset_Error::BRepOffset_CannotTrimEdges` | The offset's edges couldn't be trimmed. |
| `offset.cannot-fuse-vertices` | `BRepOffset_Error::BRepOffset_CannotFuseVertices` | The offset's vertices couldn't be fused. |
| `offset.cannot-extend-edge` | `BRepOffset_Error::BRepOffset_CannotExtentEdge` | An offset edge couldn't be extended. |
| `offset.user-break` | `BRepOffset_Error::BRepOffset_UserBreak` | The offset was stopped. |
| `offset.mixed-connectivity` | `BRepOffset_Error::BRepOffset_MixedConnectivity` | Faces meet along an edge partly tangent and partly not. |
| `offset.null-input` | `BRepOffsetSimple_Status::BRepOffsetSimple_NullInputShape` | The shape to offset is null. |
| `offset.computation-failed` | `BRepOffsetSimple_Status::BRepOffsetSimple_ErrorOffsetComputation` | The offset surfaces couldn't be computed. |
| `offset.wall-face-failed` | `BRepOffsetSimple_Status::BRepOffsetSimple_ErrorWallFaceComputation` | The solid's wall faces couldn't be built. |
| `offset.invalid-shell-count` | `BRepOffsetSimple_Status::BRepOffsetSimple_ErrorInvalidNbShells` | The offset doesn't have one shell. |
| `offset.non-closed-shell` | `BRepOffsetSimple_Status::BRepOffsetSimple_ErrorNonClosedShell` | The offset's shell isn't closed. |

### Features

The message is BRepFeat's own description of the status (`BRepFeat::Print`).

| Code | OCCT status | Message |
|---|---|---|
| `feature.bad-direct` | `BRepFeat_StatusError::BRepFeat_BadDirect` | Directions must be opposite. |
| `feature.bad-intersect` | `BRepFeat_StatusError::BRepFeat_BadIntersect` | Intersection failure. |
| `feature.empty-bary-curve` | `BRepFeat_StatusError::BRepFeat_EmptyBaryCurve` | Empty BaryCurve. |
| `feature.empty-cut-result` | `BRepFeat_StatusError::BRepFeat_EmptyCutResult` | Failure in Cut : Empty resulting shape. |
| `feature.false-side` | `BRepFeat_StatusError::BRepFeat_FalseSide` | Verify plane and wire orientation. |
| `feature.inc-direction` | `BRepFeat_StatusError::BRepFeat_IncDirection` | Incoherent Direction for shapes From and Until. |
| `feature.inc-slid-face` | `BRepFeat_StatusError::BRepFeat_IncSlidFace` | Sliding face not in Base shape. |
| `feature.inc-parameter` | `BRepFeat_StatusError::BRepFeat_IncParameter` | Incoherent Parameter : shape Until before shape From. |
| `feature.inc-types` | `BRepFeat_StatusError::BRepFeat_IncTypes` | Invalid option for faces From and Until : 1 Support and 1 not. |
| `feature.interval-overlap` | `BRepFeat_StatusError::BRepFeat_IntervalOverlap` | Shapes From and Until overlap. |
| `feature.inv-first-shape` | `BRepFeat_StatusError::BRepFeat_InvFirstShape` | Invalid First shape : more than 1 face. |
| `feature.inv-option` | `BRepFeat_StatusError::BRepFeat_InvOption` | Invalid option. |
| `feature.inv-shape` | `BRepFeat_StatusError::BRepFeat_InvShape` | Invalid shape. |
| `feature.loc-ope-not-done` | `BRepFeat_StatusError::BRepFeat_LocOpeNotDone` | Local Operation not done. |
| `feature.loc-ope-inv-not-done` | `BRepFeat_StatusError::BRepFeat_LocOpeInvNotDone` | Local Operation : intersection line conflict. |
| `feature.no-ext-face` | `BRepFeat_StatusError::BRepFeat_NoExtFace` | No Extreme faces. |
| `feature.no-face-prof` | `BRepFeat_StatusError::BRepFeat_NoFaceProf` | No Face Profile. |
| `feature.no-gluer` | `BRepFeat_StatusError::BRepFeat_NoGluer` | Gluer Failure. |
| `feature.no-intersect-f` | `BRepFeat_StatusError::BRepFeat_NoIntersectF` | No intersection between Feature and shape From. |
| `feature.no-intersect-u` | `BRepFeat_StatusError::BRepFeat_NoIntersectU` | No intersection between Feature and shape Until. |
| `feature.no-parts` | `BRepFeat_StatusError::BRepFeat_NoParts` | No parts of tool kept. |
| `feature.no-proj-pt` | `BRepFeat_StatusError::BRepFeat_NoProjPt` | No projection points. |
| `feature.not-initialized` | `BRepFeat_StatusError::BRepFeat_NotInitialized` | Fields not initialized. |
| `feature.not-yet-implemented` | `BRepFeat_StatusError::BRepFeat_NotYetImplemented` | Not yet implemented. |
| `feature.null-real-tool` | `BRepFeat_StatusError::BRepFeat_NullRealTool` | Real Tool : Null DPrism. |
| `feature.null-tool-f` | `BRepFeat_StatusError::BRepFeat_NullToolF` | Null Tool : Invalid type for shape Form. |
| `feature.null-tool-u` | `BRepFeat_StatusError::BRepFeat_NullToolU` | Null Tool : Invalid type for shape Until. |

### Exchange

| Code | OCCT status | Message |
|---|---|---|
| `exchange.nothing-done` | `IFSelect_ReturnStatus::IFSelect_RetVoid` | Nothing was read or written. |
| `exchange.invalid-data` | `IFSelect_ReturnStatus::IFSelect_RetError` | The data isn't correct. |
| `exchange.failed` | `IFSelect_ReturnStatus::IFSelect_RetFail` | Reading or writing failed. |
| `exchange.stopped` | `IFSelect_ReturnStatus::IFSelect_RetStop` | Reading or writing was stopped. |

`exchange.entity-skipped` is a warning on a STEP read that succeeded (decision 2308): entities of the file couldn't be
read, and what refers to them is missing from the model. Its message gives how many, the first eight identifiers and
the first failure's text, and ends "Nothing else was read: the model is empty." where nothing is left (decision 2315).

`exchange.invalid-face` is a warning on a STEP read that succeeded (decision 2314): a face the read checked, one the
healing reported on, with a vertex off its edge's curve or bounded by two edges or fewer (decision 2427), is invalid. Its message gives how many faces (each once, however
often the model places it) and the STEP identifiers of the faces they were read from, the first eight. Both codes are
properties of `Step`: `Step.EntitySkipped`, `Step.InvalidFace`.

## The public operations' own codes (unit 10L2)

Beside the bridges above, which the operations use for the engine's statuses and alerts, these are the operations'
own. Those a program is likely to test for are static properties of the operations class (`GeometryOperations.NotConnected`).

| Code | Where | Meaning |
|---|---|---|
| `geometry.construction-failed` | `GeometryOperations` (arcs, segments) | The curve can't be constructed from its data (`gce_ErrorType`, described) |
| `geometry.interpolation-failed` | `Interpolate` | No B-spline passes through the points (coincident points, inconsistent tangents) |
| `geometry.approximation-failed` | `Approximate` | No B-spline fits within the options |
| `geometry.not-connected` | `Join` | A curve doesn't meet the ones before it |
| `geometry.conversion-failed` | `ToBSpline`, `ToCurve2D`, `Join` | The curve can't be converted |
| `geometry.intersection-failed` | `Intersect` | The intersection couldn't be computed |
| `geometry.curve-in-surface`, `geometry.curves-overlap` | `Intersect` (warnings) | A stretch in common, which isn't a point |
| `construction.failed` | `Primitives`, `ShapeFactory` | A primitive or shape the engine couldn't build, with no status |
| `construction.degenerate-polygon` | `Wire.Polygon` | The points coincide |
| `construction.sweep-failed` | `Sweeps` | A sweep failed with no status |
| `construction.not-solid` | `Sweeps.Sweep` | Closed profiles, but the sweep couldn't be closed into a solid |
| `construction.transform-failed` | `ShapeTransforms` | The transformation couldn't be applied |
| `construction.projection-empty` | `ShapeProjections` | The projection doesn't meet the shape |
| `boolean.empty-result` | `BooleanOperations` (warning) | The result is empty |
| `fillet.<status>` | `Fillet2D`, `Chamfer2D` | `ChFi2d_ConstructionError` in kebab case (`fillet.computation-error`), with the corner |
| `offset.not-planar` | `Offset2D` | The planar offset couldn't be built |
| `offset.not-at-distance` | `Offset2D` | The planar offset was made but isn't at its distance from the wire everywhere, so it's refused (decisions 2279, 2281) |
| `offset.crosses-itself` | `Offset2D` | The planar offset was made but crosses itself, or one of its loops crosses another, so it's refused; the message says where (decision 2422) |
| `offset.wrong-side` | `Offset2D` | The planar offset of closed wires was made but a loop of it lies on the other side of the wires than asked (inside for outwards, outside for inwards), so it's refused; the message says where (decision 2443) |
| `offset.join-changed` | `Offset`, `Hollow` (warning) | Round joins failed at an apex, so the result was made with mitred joins, and passed its checks |
| `draft.face-failed` | `FeatureOperations.Draft` | A face can't be drafted, with the face |
| `feature.failed` | `FeatureOperations` | A feature failed with no status |
| `meshing.faces-not-meshed` | `Mesher` (warning) | Some faces couldn't be meshed: they have no triangles |
| `meshing.deflection-exceeded` | `Mesher`, the glTF, 3MF and OBJ writers, `Render`, `ToScene` (warning) | Some sphere or torus faces are further from their surfaces than the deflection and couldn't be refined further, with the faces (decision 2071) |

## The modelling layer

An operation of Kerf.Modeling that fails in the kernel returns a failed `Result<T>` carrying the kernel's diagnostics
(from the bridges above), each error's message prefixed with what was being done, outermost first (unit 10M3; until then a
`KerfException`). Bento shows the first error's message in the cell:

- `box 0 × 1 × 1 failed: The value must be a positive number` (`kernel.invalid-argument`)
- `fuse of a Box with a Cylinder failed: Some of the arguments are self-interfering shapes` (the inputs by class)
- `extrude until / extrude failed: the geometry can't be constructed: BRepSweep_Translation::Constructor`

The operations that return results are `Operations`' (`Fillet`, `Chamfer`, `Extrude`, `Loft`, `Revolve`, `Sweep`,
`Offset`, `Thicken`, `Draft`), the algebra's `Add`, `Subtract` and `Common`, and the importers. The shorthands that can't
return one (the operators `+ - &`, the objects' constructors such as `new Box(0, 1, 1)`, the topological shapes' own
methods) throw `InvalidOperationException` with the same message, the kernel's failure inside (unit 10M3). The
failures the layer detects itself have codes of their own (`ModelingDiagnostics`), the kernel's diagnostics that led to
them following:

| Code | Where |
|---|---|
| `modeling.fillet-failed` | A 3D fillet that fails or is invalid (the message names the edges and why); a 2D fillet's corner |
| `modeling.chamfer-failed` | A chamfer that fails or is invalid |
| `modeling.offset-failed` | A 2D offset, or a face's, shell's or solid's offset (a solid's since unit 10M3) |
| `modeling.loft-failed` | A loft that is invalid and can't be recovered (unit 10M3: it was an `InvalidOperationException`) |
| `modeling.thicken-failed` | A face or shell that can't be thickened (unit 10M3: it was an `InvalidOperationException`) |
| `modeling.draft-failed` | A draft a face can't take (the face is the diagnostic's shape), or a drafted solid that fails |
| `modeling.spline-failed` | A B-spline that can't be interpolated or approximated through the points |
| `kernel.invalid-argument` | An argument the kernel refused, said in context (unit 10M1) |

The layer knows an argument the kernel refused by the exception's type: every argument exception the kernel throws is
one of its own types (`ArgumentException`, `ArgumentOutOfRangeException` or `ArgumentNullException` to a caller,
with the same messages), which `Diagnostic.IsKernelArgumentError` recognises, and KERF0004 makes a kernel source that
throws .NET's own an error. An argument exception of any other kind is the layer's own misuse error, and passes
through. It once went by the assembly of the frame that threw, which the JIT's inlining changes once a call is hot
(decision 2028).

The operations that say what they were doing: the primitives (box, cone, cylinder, sphere, torus, wedge), lofts,
extrusions (plain, with rotation, tapered, until), revolutions, sweeps (single and multi-section), thickening,
triangulation and every Boolean (cut, fuse, intersect, section, split). The importers return the exchange's own
diagnostics (`exchange.failed`, `exchange.malformed`, `exchange.external-references`…); STEP's error says "Could not
import … : it isn't readable STEP data", then the exchange's message (unit 10M3; it was an `ArgumentException`).

## Fonts and text (unit 13D)

| Code | Meaning |
|---|---|
| `font.malformed` | Data that isn't a font, or a font damaged past reading (`Font.Malformed`) |
| `font.limit-exceeded` | A font past a `FontReadLimits` limit (`Font.LimitExceeded`) |
| `font.unsupported` | A font in a form the reader doesn't read: WOFF, CFF2, Type 1, bitmaps only (`Font.Unsupported`) |
| `font.substituted` | A warning: a font was replaced by another, and the message says which (`FontLibrary.Substituted`) |
| `text.missing-glyph` | A character the font has no glyph for (`TextShapes.MissingGlyph`) |
| `text.empty` | Text with nothing to draw (`TextShapes.Empty`) |
