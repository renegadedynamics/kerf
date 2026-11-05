// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Alerts.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// The alerts of the Boolean component, one class per DEFINE_SIMPLE_ALERT (a Message_Alert without data) or
// DEFINE_ALERT_WITH_SHAPE (a TopoDS_AlertWithShape), so callers can check them by type (decision 584):
// algo.HasError(typeof(BOPAlgo_AlertSelfInterferingShape)). Their message texts are in BOPAlgo_BOPAlgo_msg.

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Booleans;

/// <summary>Boolean operation was stopped by user.</summary>
internal sealed class AlertUserBreak : Alert
{
}

/// <summary>Boolean operation of given type is not allowed on the given inputs.</summary>
internal sealed class AlertBopNotAllowed : Alert
{
}

/// <summary>The type of Boolean Operation is not set.</summary>
internal sealed class AlertBopNotSet : Alert
{
}

/// <summary>Building of the result shape has failed.</summary>
internal sealed class AlertBuilderFailed : Alert
{
}

/// <summary>The intersection of the arguments has failed.</summary>
internal sealed class AlertIntersectionFailed : Alert
{
}

/// <summary>More than one argument is provided.</summary>
internal sealed class AlertMultipleArguments : Alert
{
}

/// <summary>The Pave Filler (the intersection tool) has not been created.</summary>
internal sealed class AlertNoFiller : Alert
{
}

/// <summary>Null input shapes.</summary>
internal sealed class AlertNullInputShapes : Alert
{
}

/// <summary>Cannot connect face intersection curves.</summary>
internal sealed class AlertPostTreatFF : Alert
{
}

/// <summary>The BuilderSolid algorithm has failed.</summary>
internal sealed class AlertSolidBuilderFailed : Alert
{
}

/// <summary>There are no enough arguments to perform the operation.</summary>
internal sealed class AlertTooFewArguments : Alert
{
}

/// <summary>The positioning of the shapes leads to creation of the small edges without valid range.</summary>
internal sealed class AlertBadPositioning(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Some of the arguments are empty shapes.</summary>
internal sealed class AlertEmptyShape(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Some edges are very small and have such a small valid range, that they cannot be split.</summary>
internal sealed class AlertNotSplittableEdge(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Removal of internal boundaries among Edges has failed.</summary>
internal sealed class AlertRemovalOfIBForEdgesFailed(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Removal of internal boundaries among Faces has failed.</summary>
internal sealed class AlertRemovalOfIBForFacesFailed(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Removal of internal boundaries among the multi-dimensional shapes is not supported yet.</summary>
internal sealed class AlertRemovalOfIBForMDimShapes(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Removal of internal boundaries among Solids has failed.</summary>
internal sealed class AlertRemovalOfIBForSolidsFailed(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Some of the arguments are self-interfering shapes.</summary>
internal sealed class AlertSelfInterferingShape(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>The positioning of the shapes leads to creation of the small edges without valid range.</summary>
internal sealed class AlertShellSplitterFailed(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Some edges are too small and have no valid range.</summary>
internal sealed class AlertTooSmallEdge(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Intersection of pair of shapes has failed.</summary>
internal sealed class AlertIntersectionOfPairOfShapesFailed(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Building 2D curve of edge on face has failed.</summary>
internal sealed class AlertBuildingPCurveFailed(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Some sub-shapes of some of the argument become connected through other shapes and the argument became self-interfered.</summary>
internal sealed class AlertAcquiredSelfIntersection(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Unsupported type of input shape.</summary>
internal sealed class AlertUnsupportedType(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>No faces have been found for removal.</summary>
internal sealed class AlertNoFacesToRemove : Alert
{
}

/// <summary>Unable to remove the feature.</summary>
internal sealed class AlertUnableToRemoveTheFeature(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>The Feature Removal algorithm has failed.</summary>
internal sealed class AlertRemoveFeaturesFailed : Alert
{
}

/// <summary>Some of the faces passed to the Solid Builder algorithm have not been classified and not used for solids creation.</summary>
internal sealed class AlertSolidBuilderUnusedFaces(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Some of the edges passed to the Face Builder algorithm have not been classified and not used for faces creation.</summary>
internal sealed class AlertFaceBuilderUnusedEdges(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Unable to orient the shape correctly.</summary>
internal sealed class AlertUnableToOrientTheShape(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Shape is unknown for operation.</summary>
internal sealed class AlertUnknownShape(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>No periodicity has been requested for the shape.</summary>
internal sealed class AlertNoPeriodicityRequired : Alert
{
}

/// <summary>Unable to trim the shape for making it periodic (BOP Common fails).</summary>
internal sealed class AlertUnableToTrim(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Unable to make the shape to look identical on opposite sides (Splitter fails).</summary>
internal sealed class AlertUnableToMakeIdentical(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Unable to repeat the shape (Gluer fails).</summary>
internal sealed class AlertUnableToRepeat(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Multi-dimensional arguments.</summary>
internal sealed class AlertMultiDimensionalArguments : Alert
{
}

/// <summary>Unable to make the shape periodic.</summary>
internal sealed class AlertUnableToMakePeriodic(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Unable to glue the shapes.</summary>
internal sealed class AlertUnableToGlue(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>The shape is not periodic.</summary>
internal sealed class AlertShapeIsNotPeriodic(in Shape theShape) : AlertWithShape(theShape)
{
}

/// <summary>Unable to make closed edge on face (to make a seam).</summary>
internal sealed class AlertUnableToMakeClosedEdgeOnFace(in Shape theShape) : AlertWithShape(theShape)
{
}
