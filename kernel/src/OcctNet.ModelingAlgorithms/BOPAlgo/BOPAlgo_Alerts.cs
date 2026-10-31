// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Alerts.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// The alerts of the Boolean component, one class per DEFINE_SIMPLE_ALERT (a Message_Alert without data) or
// DEFINE_ALERT_WITH_SHAPE (a TopoDS_AlertWithShape), so callers can check them by type (decision 584):
// algo.HasError(typeof(BOPAlgo_AlertSelfInterferingShape)). Their message texts are in BOPAlgo_BOPAlgo_msg.

namespace Occt;

/// <summary>Boolean operation was stopped by user.</summary>
public sealed class BOPAlgo_AlertUserBreak : Message_Alert
{
}

/// <summary>Boolean operation of given type is not allowed on the given inputs.</summary>
public sealed class BOPAlgo_AlertBOPNotAllowed : Message_Alert
{
}

/// <summary>The type of Boolean Operation is not set.</summary>
public sealed class BOPAlgo_AlertBOPNotSet : Message_Alert
{
}

/// <summary>Building of the result shape has failed.</summary>
public sealed class BOPAlgo_AlertBuilderFailed : Message_Alert
{
}

/// <summary>The intersection of the arguments has failed.</summary>
public sealed class BOPAlgo_AlertIntersectionFailed : Message_Alert
{
}

/// <summary>More than one argument is provided.</summary>
public sealed class BOPAlgo_AlertMultipleArguments : Message_Alert
{
}

/// <summary>The Pave Filler (the intersection tool) has not been created.</summary>
public sealed class BOPAlgo_AlertNoFiller : Message_Alert
{
}

/// <summary>Null input shapes.</summary>
public sealed class BOPAlgo_AlertNullInputShapes : Message_Alert
{
}

/// <summary>Cannot connect face intersection curves.</summary>
public sealed class BOPAlgo_AlertPostTreatFF : Message_Alert
{
}

/// <summary>The BuilderSolid algorithm has failed.</summary>
public sealed class BOPAlgo_AlertSolidBuilderFailed : Message_Alert
{
}

/// <summary>There are no enough arguments to perform the operation.</summary>
public sealed class BOPAlgo_AlertTooFewArguments : Message_Alert
{
}

/// <summary>The positioning of the shapes leads to creation of the small edges without valid range.</summary>
public sealed class BOPAlgo_AlertBadPositioning(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Some of the arguments are empty shapes.</summary>
public sealed class BOPAlgo_AlertEmptyShape(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Some edges are very small and have such a small valid range, that they cannot be split.</summary>
public sealed class BOPAlgo_AlertNotSplittableEdge(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Removal of internal boundaries among Edges has failed.</summary>
public sealed class BOPAlgo_AlertRemovalOfIBForEdgesFailed(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Removal of internal boundaries among Faces has failed.</summary>
public sealed class BOPAlgo_AlertRemovalOfIBForFacesFailed(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Removal of internal boundaries among the multi-dimensional shapes is not supported yet.</summary>
public sealed class BOPAlgo_AlertRemovalOfIBForMDimShapes(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Removal of internal boundaries among Solids has failed.</summary>
public sealed class BOPAlgo_AlertRemovalOfIBForSolidsFailed(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Some of the arguments are self-interfering shapes.</summary>
public sealed class BOPAlgo_AlertSelfInterferingShape(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>The positioning of the shapes leads to creation of the small edges without valid range.</summary>
public sealed class BOPAlgo_AlertShellSplitterFailed(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Some edges are too small and have no valid range.</summary>
public sealed class BOPAlgo_AlertTooSmallEdge(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Intersection of pair of shapes has failed.</summary>
public sealed class BOPAlgo_AlertIntersectionOfPairOfShapesFailed(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Building 2D curve of edge on face has failed.</summary>
public sealed class BOPAlgo_AlertBuildingPCurveFailed(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Some sub-shapes of some of the argument become connected through other shapes and the argument became self-interfered.</summary>
public sealed class BOPAlgo_AlertAcquiredSelfIntersection(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Unsupported type of input shape.</summary>
public sealed class BOPAlgo_AlertUnsupportedType(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>No faces have been found for removal.</summary>
public sealed class BOPAlgo_AlertNoFacesToRemove : Message_Alert
{
}

/// <summary>Unable to remove the feature.</summary>
public sealed class BOPAlgo_AlertUnableToRemoveTheFeature(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>The Feature Removal algorithm has failed.</summary>
public sealed class BOPAlgo_AlertRemoveFeaturesFailed : Message_Alert
{
}

/// <summary>Some of the faces passed to the Solid Builder algorithm have not been classified and not used for solids creation.</summary>
public sealed class BOPAlgo_AlertSolidBuilderUnusedFaces(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Some of the edges passed to the Face Builder algorithm have not been classified and not used for faces creation.</summary>
public sealed class BOPAlgo_AlertFaceBuilderUnusedEdges(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Unable to orient the shape correctly.</summary>
public sealed class BOPAlgo_AlertUnableToOrientTheShape(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Shape is unknown for operation.</summary>
public sealed class BOPAlgo_AlertUnknownShape(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>No periodicity has been requested for the shape.</summary>
public sealed class BOPAlgo_AlertNoPeriodicityRequired : Message_Alert
{
}

/// <summary>Unable to trim the shape for making it periodic (BOP Common fails).</summary>
public sealed class BOPAlgo_AlertUnableToTrim(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Unable to make the shape to look identical on opposite sides (Splitter fails).</summary>
public sealed class BOPAlgo_AlertUnableToMakeIdentical(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Unable to repeat the shape (Gluer fails).</summary>
public sealed class BOPAlgo_AlertUnableToRepeat(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Multi-dimensional arguments.</summary>
public sealed class BOPAlgo_AlertMultiDimensionalArguments : Message_Alert
{
}

/// <summary>Unable to make the shape periodic.</summary>
public sealed class BOPAlgo_AlertUnableToMakePeriodic(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Unable to glue the shapes.</summary>
public sealed class BOPAlgo_AlertUnableToGlue(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>The shape is not periodic.</summary>
public sealed class BOPAlgo_AlertShapeIsNotPeriodic(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}

/// <summary>Unable to make closed edge on face (to make a seam).</summary>
public sealed class BOPAlgo_AlertUnableToMakeClosedEdgeOnFace(in TopoDS_Shape theShape) : TopoDS_AlertWithShape(theShape)
{
}
