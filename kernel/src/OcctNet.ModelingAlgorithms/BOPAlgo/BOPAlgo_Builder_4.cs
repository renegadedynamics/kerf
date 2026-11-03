// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder_4.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

public partial class BOPAlgo_Builder
{
    /// <summary>
    /// The shapes generated from <paramref name="theS"/> and kept in the result: an edge generates the vertices of its
    /// edge/edge and edge/face intersections, a face those of its edge/face intersections and its section edges and
    /// vertices.
    /// </summary>
    protected virtual NCollection_List<Shape> LocGenerated(in Shape theS)
    {
        // The rules for Generated shapes are these:
        // 1. The EDGE may be generated from the FACES as an intersection edge;
        // 2. The VERTEX may be generated from the EDGES and FACES as an intersection vertex.
        //
        // The list of generated elements will contain only those which are contained
        // in the result of the operation.

        myHistShapes.Clear();

        if (theS.IsNull)
        {
            return myHistShapes;
        }

        // Only EDGES and FACES should be considered
        ShapeType aType = theS.ShapeType();
        if (aType != ShapeType.Edge && aType != ShapeType.Face)
        {
            // Wrong type
            return myHistShapes;
        }

        // Check that DS contains the shape, i.e. it is from the arguments of the operation
        int nS = myDS!.Index(theS);
        if (nS < 0)
        {
            // Unknown shape
            return myHistShapes;
        }

        // Check that the shape has participated in any intersections
        BOPDS_ShapeInfo aSI = myDS.ShapeInfo(nS);
        if (!aSI.HasReference())
        {
            // Untouched shape
            return myHistShapes;
        }

        // Analyze all types of Interferences which can produce
        // new vertices - Edge/Edge and Edge/Face
        NCollection_DynamicArray<BOPDS_InterfEE> aEEs = myDS.InterfEE();
        NCollection_DynamicArray<BOPDS_InterfEF> aEFs = myDS.InterfEF();

        // Fence map to avoid duplicates in the list of Generated;
        NCollection_IntMap aMFence = new();

        // Analyze each interference and find those in which the given shape has participated

        // No need to analyze Edge/Edge interferences for the shapes of type FACE
        bool isFace = aType == ShapeType.Face;

        for (int k = isFace ? 1 : 0; k < 2; ++k)
        {
            int aNbLines = k == 0 ? aEEs.Length() : aEFs.Length();
            for (int i = 0; i < aNbLines; ++i)
            {
                BOPDS_Interf aInt = k == 0 ? aEEs[i] : aEFs[i];
                if (!aInt.HasIndexNew())
                {
                    // No new vertices created
                    continue;
                }

                if (!aInt.Contains(nS))
                {
                    continue;
                }

                int nVNew = aInt.IndexNew();
                myDS.HasShapeSD(nVNew, ref nVNew);
                if (!aMFence.Add(nVNew))
                {
                    continue;
                }

                // Get the new vertex
                Shape aVNew = myDS.Shape(nVNew);

                // Check that the result shape contains vertex
                if (myMapShape.Contains(aVNew))
                {
                    // Save the vertex as generated
                    myHistShapes.Append(aVNew);
                }
            }
        }

        if (!isFace)
        {
            return myHistShapes;
        }

        // For the FACE it is also necessary to collect all
        // section elements created in FACE/FACE interferences.
        // This information is available in the FaceInfo structure.
        BOPDS_FaceInfo aFI = myDS.FaceInfo(nS);

        // Section edges of the face
        NCollection_IndexedMap<BOPDS_PaveBlock> aMPBSc = aFI.PaveBlocksSc();
        // Save section edges contained in the result shape
        int aNb = aMPBSc.Extent();
        for (int i = 1; i <= aNb; ++i)
        {
            Shape aENew = myDS.Shape(aMPBSc[i].Edge());
            if (myMapShape.Contains(aENew))
            {
                myHistShapes.Append(aENew);
            }
        }

        // Section vertices of the face
        NCollection_IntMap aMVSc = aFI.VerticesSc();
        // Save section vertices contained in the result shape (OCCT's order, decision 784)
        foreach (int aItM in aMVSc)
        {
            Shape aVNew = myDS.Shape(aItM);
            if (myMapShape.Contains(aVNew))
            {
                myHistShapes.Append(aVNew);
            }
        }

        return myHistShapes;
    }

    /// <summary>The splits of <paramref name="theS"/> (null if it has no images).</summary>
    protected virtual NCollection_List<Shape>? LocModified(in Shape theS)
        => myImages.Find(theS, out NCollection_List<Shape> aList) ? aList : null;

    /// <summary>OCCT's history of the result: modified, generated and deleted argument sub-shapes.</summary>
    protected void PrepareHistory(ProgressRange theRange)
    {
        if (!HasHistory())
        {
            return;
        }

        // Initializing history tool
        myHistory = new History();

        // Map the result shape
        myMapShape.Clear();
        Exploration.MapShapes(myShape, myMapShape);

        // Among all input shapes find:
        // - Shapes that have been modified (split). Add the splits kept in the result
        //   shape as Modified from the shape;
        // - Shapes that have created new geometries (i.e. generated new shapes). Add
        //   the generated elements kept in the result shape as Generated from the shape;
        // - Shapes that have no trace in the result shape. Add them as Deleted
        //   during the operation.
        int aNbS = myDS!.NbSourceShapes();
        using var aPS = new ProgressScope(theRange, "Preparing history information", aNbS);
        for (int i = 0; i < aNbS; ++i, aPS.Next())
        {
            Shape aS = myDS.Shape(i);

            // Check if History information is available for this kind of shape.
            if (!global::Kerf.Engine.Topology.History.IsSupportedType(aS))
            {
                continue;
            }

            if (UserBreak(aPS))
            {
                return;
            }

            bool isModified = false;

            // Check if the shape has any splits
            NCollection_List<Shape>? pLSp = LocModified(aS);
            if (pLSp is not null)
            {
                // Find all splits of the shape which are kept in the result
                NCollection_List<Shape>.Iterator aIt0 = new(pLSp);
                for (; aIt0.More(); aIt0.Next())
                {
                    Shape aSp = aIt0.Value();
                    // Check if the result shape contains the split
                    if (myMapShape.Contains(aSp))
                    {
                        // Add modified shape with proper orientation
                        ShapeType aType = aSp.ShapeType();
                        if (aType == ShapeType.Vertex || aType == ShapeType.Solid)
                        {
                            aSp.Orientation(aS.Orientation());
                        }
                        else if (BOPTools_AlgoTools.IsSplitToReverse(aSp, aS, myContext))
                        {
                            aSp.Reverse();
                        }

                        myHistory.AddModified(aS, aSp);
                        isModified = true;
                    }
                }
            }

            // Check if the shape has Generated elements
            NCollection_List<Shape> aGenShapes = LocGenerated(aS);
            NCollection_List<Shape>.Iterator aIt = new(aGenShapes);
            for (; aIt.More(); aIt.Next())
            {
                Shape aG = aIt.Value();
                if (myMapShape.Contains(aG))
                {
                    myHistory.AddGenerated(aS, aG);
                }
            }

            // Check if the shape has been deleted, i.e. it is not contained in the result
            // and has no Modified shapes.
            if (!isModified && !myMapShape.Contains(aS))
            {
                myHistory.Remove(aS);
            }
        }
    }
}
