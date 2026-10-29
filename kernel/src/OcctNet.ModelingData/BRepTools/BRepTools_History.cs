// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_History.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_History.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The history of shapes modified, generated or removed by an algorithm: which shapes were made from each input
/// vertex, edge, face or solid. A history merges with the history of a later algorithm (<see cref="Merge(BRepTools_History?)"/>),
/// so the composed history answers for the first algorithm's inputs.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21): the methods that set the history are public. Generated and modified shapes are
/// recorded per input shape in order; a shape can't be both modified and removed, and a shape can't be both generated
/// and modified from the same input (OCCT's assertions are debug-only reports, so a later setting silently wins).
/// </para>
/// <para>
/// <see cref="Generated"/> and <see cref="Modified"/> return read-only lists (OCCT returns a reference to its own list).
/// The order of the lists that <see cref="Merge(BRepTools_History?)"/> builds follows hash maps, as in OCCT, whose
/// order depends on addresses: compare them as sets.
/// </para>
/// </remarks>
public class BRepTools_History : Standard_Transient
{
    /// <summary>
    /// The standard history methods of an algorithm (IsDeleted, Modified and Generated), for building a history from
    /// it (OCCT's template constructor).
    /// </summary>
    public interface IAlgorithm
    {
        /// <summary>True if the algorithm deleted the shape.</summary>
        bool IsDeleted(in TopoDS_Shape theS);

        /// <summary>The shapes modified from the shape.</summary>
        IEnumerable<TopoDS_Shape> Modified(in TopoDS_Shape theS);

        /// <summary>The shapes generated from the shape.</summary>
        IEnumerable<TopoDS_Shape> Generated(in TopoDS_Shape theS);
    }

    /// <summary>The types of the historical relations.</summary>
    public enum TRelationType
    {
        TRelationType_Removed,
        TRelationType_Generated,
        TRelationType_Modified
    }

    // Maps each input shape to all shapes modified from it.
    // If an input shape is not bound to the map then
    // there is no shapes modified from the shape.
    // No any shape should be mapped to an empty list.
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myShapeToModified =
        new(TopTools_ShapeMapHasher.Instance);

    // Maps each input shape to all shapes generated from it.
    // If an input shape is not bound to the map then
    // there is no shapes generated from the shape.
    // No any shape should be mapped to an empty list.
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myShapeToGenerated =
        new(TopTools_ShapeMapHasher.Instance);

    // The removed shapes.
    private readonly NCollection_Map<TopoDS_Shape> myRemoved = new(TopTools_ShapeMapHasher.Instance);

    /// <summary>An empty history.</summary>
    public BRepTools_History()
    {
    }

    /// <summary>
    /// The history of an algorithm with the standard history methods, for the vertices, edges, faces and solids of
    /// its arguments.
    /// </summary>
    public BRepTools_History(NCollection_List<TopoDS_Shape> theArguments, IAlgorithm theAlgo)
    {
        // Map all argument shapes to save them in history
        var anArgsMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var aIt = new NCollection_List<TopoDS_Shape>.Iterator(theArguments);
        for (; aIt.More(); aIt.Next())
        {
            if (!aIt.Value().IsNull())
            {
                TopExp.MapShapes(aIt.Value(), anArgsMap);
            }
        }
        // Copy the history for all supported shapes from the algorithm
        int i, aNb = anArgsMap.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aS = anArgsMap[i];
            if (!IsSupportedType(aS))
            {
                continue;
            }
            if (theAlgo.IsDeleted(aS))
            {
                Remove(aS);
            }
            // Check Modified
            foreach (TopoDS_Shape aModified in theAlgo.Modified(aS))
            {
                AddModified(aS, aModified);
            }
            // Check Generated
            foreach (TopoDS_Shape aGenerated in theAlgo.Generated(aS))
            {
                AddGenerated(aS, aGenerated);
            }
        }
    }

    /// <summary>True if the type of the shape is supported by the history: a vertex, an edge, a face or a solid.</summary>
    public static bool IsSupportedType(in TopoDS_Shape theShape)
    {
        TopAbs_ShapeEnum aType = theShape.ShapeType();
        return aType == TopAbs_VERTEX || aType == TopAbs_EDGE || aType == TopAbs_FACE
               || aType == TopAbs_SOLID;
    }

    /// <summary>Records the second shape as generated from the first.</summary>
    public void AddGenerated(in TopoDS_Shape theInitial, in TopoDS_Shape theGenerated)
    {
        if (!prepareGenerated(theInitial, theGenerated))
        {
            return;
        }

        if (!myShapeToGenerated.Find(theInitial, out NCollection_List<TopoDS_Shape>? aGenerations))
        {
            aGenerations = new NCollection_List<TopoDS_Shape>();
            myShapeToGenerated.Bind(theInitial, aGenerations);
        }

        // Standard_ASSERT_VOID: "Error: a duplicated generation of a shape." (debug-only in OCCT)

        aGenerations.Append(theGenerated);
    }

    /// <summary>Records the second shape as modified from the first.</summary>
    public void AddModified(in TopoDS_Shape theInitial, in TopoDS_Shape theModified)
    {
        if (!prepareModified(theInitial, theModified))
        {
            return;
        }

        if (!myShapeToModified.Find(theInitial, out NCollection_List<TopoDS_Shape>? aModifications))
        {
            aModifications = new NCollection_List<TopoDS_Shape>();
            myShapeToModified.Bind(theInitial, aModifications);
        }

        // Standard_ASSERT_VOID: "Error: a duplicated modification of a shape." (debug-only in OCCT)

        aModifications.Append(theModified);
    }

    /// <summary>Records the shape as removed (and no longer modified).</summary>
    public void Remove(in TopoDS_Shape theRemoved)
    {
        // Apply the limitations.
        if (!IsSupportedType(theRemoved))
        {
            return; // Standard_ASSERT_RETURN: myMsgUnsupportedType
        }

        // Standard_ASSERT_INVOKE_ myMsgModifiedAndRemoved when it was modified: a debug-only report
        myShapeToModified.UnBind(theRemoved);

        myRemoved.Add(theRemoved);
    }

    /// <summary>Records the second shape as the only one generated from the first.</summary>
    public void ReplaceGenerated(in TopoDS_Shape theInitial, in TopoDS_Shape theGenerated)
    {
        if (!prepareGenerated(theInitial, theGenerated))
        {
            return;
        }

        var aGenerations = new NCollection_List<TopoDS_Shape>();
        myShapeToGenerated.Bind(theInitial, aGenerations);
        aGenerations.Append(theGenerated);
    }

    /// <summary>Records the second shape as the only one modified from the first.</summary>
    public void ReplaceModified(in TopoDS_Shape theInitial, in TopoDS_Shape theModified)
    {
        if (!prepareModified(theInitial, theModified))
        {
            return;
        }

        var aModifications = new NCollection_List<TopoDS_Shape>();
        myShapeToModified.Bind(theInitial, aModifications);
        aModifications.Append(theModified);
    }

    /// <summary>Clears the history.</summary>
    public void Clear()
    {
        myShapeToModified.Clear();
        myShapeToGenerated.Clear();
        myRemoved.Clear();
    }

    /// <summary>The shapes generated from the shape (empty for an unsupported type).</summary>
    public IBentoList<TopoDS_Shape> Generated(in TopoDS_Shape theInitial)
    {
        // Apply the limitations.
        if (!(theInitial.IsNull() || IsSupportedType(theInitial)))
        {
            return PackedList<TopoDS_Shape>.Empty; // Standard_ASSERT_RETURN: myMsgUnsupportedType
        }

        //
        return myShapeToGenerated.Find(theInitial, out NCollection_List<TopoDS_Shape>? aGenerations)
                 ? PackedList.Own([.. aGenerations])
                 : PackedList<TopoDS_Shape>.Empty;
    }

    /// <summary>The shapes modified from the shape (empty for an unsupported type).</summary>
    public IBentoList<TopoDS_Shape> Modified(in TopoDS_Shape theInitial)
    {
        // Apply the limitations.
        if (!IsSupportedType(theInitial))
        {
            return PackedList<TopoDS_Shape>.Empty; // Standard_ASSERT_RETURN: myMsgUnsupportedType
        }

        //
        return myShapeToModified.Find(theInitial, out NCollection_List<TopoDS_Shape>? aModifications)
                 ? PackedList.Own([.. aModifications])
                 : PackedList<TopoDS_Shape>.Empty;
    }

    /// <summary>True if the shape is removed (false for an unsupported type).</summary>
    public bool IsRemoved(in TopoDS_Shape theInitial)
    {
        // Apply the limitations.
        if (!IsSupportedType(theInitial))
        {
            return false; // Standard_ASSERT_RETURN: myMsgUnsupportedType
        }

        //
        return myRemoved.Contains(theInitial);
    }

    /// <summary>True if some shape has generated shapes.</summary>
    public bool HasGenerated() => !myShapeToGenerated.IsEmpty();

    /// <summary>True if some shape has modified shapes.</summary>
    public bool HasModified() => !myShapeToModified.IsEmpty();

    /// <summary>True if some shape is removed.</summary>
    public bool HasRemoved() => !myRemoved.IsEmpty();

    /// <summary>
    /// Merges the next history (from this history's results to a later algorithm's) into this one, so that it answers
    /// for this history's inputs. A null history merges nothing.
    /// </summary>
    public void Merge(BRepTools_History? theHistory23)
    {
        if (theHistory23 is null)
        {
            return;
        }

        if (!(theHistory23.HasModified() || theHistory23.HasGenerated() || theHistory23.HasRemoved()))
        {
            // nothing to merge
            return;
        }

        // Propagate R23 directly and M23 and G23 fully to M12 and G12.
        // Remember the propagated shapes.
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>[] aS1ToGAndM =
            [myShapeToGenerated, myShapeToModified];
        var aRPropagated = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        {
            // Propagate R23, M23 and G23 to M12 and G12 directly.
            // Remember the propagated shapes.
            var aMAndGPropagated = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            for (int aI = 0; aI < 2; ++aI)
            {
                // OCCT.NET: the generated map gains keys while the modified map is walked (aI = 1), never the map
                // being walked, so the walk is over the map itself as in OCCT.
                foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMIt1 in aS1ToGAndM[aI])
                {
                    NCollection_List<TopoDS_Shape> aL12 = aMIt1.Value;
                    NCollection_Map<TopoDS_Shape>[] aAdditions =
                    [
                        new(TopTools_ShapeMapHasher.Instance),
                        new(TopTools_ShapeMapHasher.Instance)
                    ]; // The G and M additions.
                    for (var aSIt2 = new NCollection_List<TopoDS_Shape>.Iterator(aL12); aSIt2.More();)
                    {
                        TopoDS_Shape aS2 = aSIt2.Value();
                        if (theHistory23.IsRemoved(aS2))
                        {
                            aRPropagated.Add(aS2);
                            aL12.Remove(aSIt2);
                        }
                        else
                        {
                            if (theHistory23.myShapeToGenerated.IsBound(aS2))
                            {
                                add(aAdditions[0], theHistory23.myShapeToGenerated[aS2]);
                                aMAndGPropagated.Add(aS2);
                            }

                            if (theHistory23.myShapeToModified.IsBound(aS2))
                            {
                                add(aAdditions[aI], theHistory23.myShapeToModified[aS2]);
                                aMAndGPropagated.Add(aS2);

                                aL12.Remove(aSIt2);
                            }
                            else
                            {
                                aSIt2.Next();
                            }
                        }
                    }

                    add(aL12, aAdditions[aI]);
                    if (aI != 0 && !aAdditions[0].IsEmpty())
                    {
                        TopoDS_Shape aS1 = aMIt1.Key;
                        if (!aS1ToGAndM[0].Find(aS1, out NCollection_List<TopoDS_Shape>? aGAndM))
                        {
                            aGAndM = new NCollection_List<TopoDS_Shape>();
                            aS1ToGAndM[0].Bind(aS1, aGAndM);
                        }

                        add(aGAndM, aAdditions[0]);
                    }
                }
            }

            // Propagate M23 and G23 to M12 and G12 sequentially.
            NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>[] aS2ToGAndM =
                [theHistory23.myShapeToGenerated, theHistory23.myShapeToModified];
            for (int aI = 0; aI < 2; ++aI)
            {
                foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMIt2 in aS2ToGAndM[aI])
                {
                    TopoDS_Shape aS2 = aMIt2.Key;
                    if (!aMAndGPropagated.Contains(aS2))
                    {
                        if (!aS1ToGAndM[aI].IsBound(aS2))
                        {
                            aS1ToGAndM[aI].Bind(aS2, new NCollection_List<TopoDS_Shape>());
                        }

                        var aM2 = new NCollection_List<TopoDS_Shape>(aMIt2.Value);
                        aS1ToGAndM[aI][aS2].Append(aM2);
                        myRemoved.Remove(aS2);
                    }
                }
            }
        }

        // Unbound the empty M12 and G12.
        for (int aI = 0; aI < 2; ++aI)
        {
            var anEmpty = new NCollection_List<TopoDS_Shape>();
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMIt1 in aS1ToGAndM[aI])
            {
                if (aMIt1.Value.IsEmpty())
                {
                    anEmpty.Append(aMIt1.Key);
                }
            }
            foreach (TopoDS_Shape aS1 in anEmpty)
            {
                myRemoved.Add(aS1);
                aS1ToGAndM[aI].UnBind(aS1);
            }
        }

        // Propagate R23 to R12 sequentially.
        foreach (TopoDS_Shape aS2 in theHistory23.myRemoved)
        {
            if (!aRPropagated.Contains(aS2) && !myShapeToModified.IsBound(aS2)
                && !myShapeToGenerated.IsBound(aS2))
            {
                myRemoved.Add(aS2);
            }
        }
    }

    /// <summary>
    /// Merges the history of an algorithm with the standard history methods (for the given arguments) into this one.
    /// </summary>
    public void Merge(NCollection_List<TopoDS_Shape> theArguments, IAlgorithm theAlgo)
    {
        // Create new history object from the given algorithm and merge it into this.
        Merge(new BRepTools_History(theArguments, theAlgo));
    }

    /// <summary>Writes the numbers of deleted, modified and generated shapes.</summary>
    public void Dump(TextWriter theS)
    {
        theS.Write("History contains:\n");
        theS.Write(" - " + myRemoved.Extent() + " Deleted shapes;\n");
        theS.Write(" - " + myShapeToModified.Extent() + " Modified shapes;\n");
        theS.Write(" - " + myShapeToGenerated.Extent() + " Generated shapes.\n");
    }

    // Prepares the shapes generated from the first shape to set the second one
    // as generated one from the first one by the addition or the replacement.
    // Returns 'true' on success.
    private bool prepareGenerated(in TopoDS_Shape theInitial, in TopoDS_Shape theGenerated)
    {
        if (!(theInitial.IsNull() || IsSupportedType(theInitial)))
        {
            return false; // Standard_ASSERT_RETURN: myMsgUnsupportedType
        }

        // Standard_ASSERT_INVOKE_ myMsgGeneratedAndModified when removed: a debug-only report
        if (myShapeToModified.Find(theInitial, out NCollection_List<TopoDS_Shape>? aModified))
        {
            aModified.Remove(theGenerated);
        }

        return true;
    }

    // Prepares the shapes modified from the first shape to set the second one
    // as modified one from the first one by the addition or the replacement.
    // Returns 'true' on success.
    private bool prepareModified(in TopoDS_Shape theInitial, in TopoDS_Shape theModified)
    {
        if (!IsSupportedType(theInitial))
        {
            return false; // Standard_ASSERT_RETURN: myMsgUnsupportedType
        }

        // Standard_ASSERT_INVOKE_ myMsgModifiedAndRemoved when removed: a debug-only report
        myRemoved.Remove(theInitial);

        // Standard_ASSERT_INVOKE_ myMsgGeneratedAndModified when removed: a debug-only report
        if (myShapeToGenerated.Find(theInitial, out NCollection_List<TopoDS_Shape>? aGenerated))
        {
            aGenerated.Remove(theModified);
        }

        return true;
    }

    // Adds the elements of the list to the map.
    private static void add(NCollection_Map<TopoDS_Shape> theMap, NCollection_List<TopoDS_Shape> theList)
    {
        for (var aSIt = new NCollection_List<TopoDS_Shape>.Iterator(theList); aSIt.More(); aSIt.Next())
        {
            theMap.Add(aSIt.Value());
        }
    }

    // Adds the elements of the collection to the list.
    private static void add(NCollection_List<TopoDS_Shape> theList, NCollection_Map<TopoDS_Shape> theCollection)
    {
        foreach (TopoDS_Shape aS in theCollection)
        {
            theList.Append(aS);
        }
    }
}
