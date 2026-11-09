// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_History.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_History.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Topology;

namespace Kerf.Engine.Topology;

/// <summary>
/// The history of shapes modified, generated or removed by an algorithm: which shapes were made from each input
/// vertex, edge, face or solid. A history merges with the history of a later algorithm (<see cref="Merge(History?)"/>),
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
/// The order of the lists that <see cref="Merge(History?)"/> builds follows hash maps, as in OCCT, whose
/// order depends on addresses: compare them as sets.
/// </para>
/// </remarks>
internal class History : Transient
{
    /// <summary>
    /// The standard history methods of an algorithm (IsDeleted, Modified and Generated), for building a history from
    /// it (OCCT's template constructor).
    /// </summary>
    public interface IAlgorithm
    {
        /// <summary>True if the algorithm deleted the shape.</summary>
        bool IsDeleted(in Shape s);

        /// <summary>The shapes modified from the shape.</summary>
        IEnumerable<Shape> Modified(in Shape s);

        /// <summary>The shapes generated from the shape.</summary>
        IEnumerable<Shape> Generated(in Shape s);
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
    private readonly Dictionary<Shape, NCollection_List<Shape>> _shapeToModified =
        new(ShapeHasher.Instance);

    // Maps each input shape to all shapes generated from it.
    // If an input shape is not bound to the map then
    // there is no shapes generated from the shape.
    // No any shape should be mapped to an empty list.
    private readonly Dictionary<Shape, NCollection_List<Shape>> _shapeToGenerated =
        new(ShapeHasher.Instance);

    // The removed shapes.
    private readonly HashSet<Shape> _removed = new(ShapeHasher.Instance);

    /// <summary>An empty history.</summary>
    public History()
    {
    }

    /// <summary>
    /// A copy of <paramref name="other"/>: its maps copied as they are (OCCT's implicit copy constructor; decision
    /// 10404). <see cref="Merge(History?)"/> into an empty history isn't a copy: it drops the removal of a
    /// shape that also generated or was modified.
    /// </summary>
    public History(History other)
    {
        KernelArgumentNullException.ThrowIfNull(other);
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> pair in other._shapeToModified)
        {
            _shapeToModified[pair.Key] = new NCollection_List<Shape>(pair.Value);
        }
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> pair in other._shapeToGenerated)
        {
            _shapeToGenerated[pair.Key] = new NCollection_List<Shape>(pair.Value);
        }
        foreach (Shape shape in other._removed)
        {
            _removed.Add(shape);
        }
    }

    /// <summary>
    /// The history of an algorithm with the standard history methods, for the vertices, edges, faces and solids of
    /// its arguments.
    /// </summary>
    public History(NCollection_List<Shape> arguments, IAlgorithm algo)
    {
        // Map all argument shapes to save them in history
        var argsMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        var it = new NCollection_List<Shape>.Iterator(arguments);
        for (; it.More(); it.Next())
        {
            if (!it.Value().IsNull)
            {
                Exploration.MapShapes(it.Value(), argsMap);
            }
        }
        // Copy the history for all supported shapes from the algorithm
        int i, nb = argsMap.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape s = argsMap[i - 1];
            if (!IsSupportedType(s))
            {
                continue;
            }
            if (algo.IsDeleted(s))
            {
                Remove(s);
            }
            // Check Modified
            foreach (Shape modified in algo.Modified(s))
            {
                AddModified(s, modified);
            }
            // Check Generated
            foreach (Shape generated in algo.Generated(s))
            {
                AddGenerated(s, generated);
            }
        }
    }

    /// <summary>True if the type of the shape is supported by the history: a vertex, an edge, a face or a solid.</summary>
    public static bool IsSupportedType(in Shape shape)
    {
        ShapeType type = shape.Type;
        return type == ShapeType.Vertex || type == ShapeType.Edge || type == ShapeType.Face
               || type == ShapeType.Solid;
    }

    /// <summary>Records the second shape as generated from the first.</summary>
    public void AddGenerated(in Shape initial, in Shape generated)
    {
        if (!PrepareGenerated(initial, generated))
        {
            return;
        }

        if (!_shapeToGenerated.TryGetValue(initial, out NCollection_List<Shape>? generations))
        {
            generations = new NCollection_List<Shape>();
            _shapeToGenerated[initial] = generations;
        }

        // Standard_ASSERT_VOID: "Error: a duplicated generation of a shape." (debug-only in OCCT)

        generations.Append(generated);
    }

    /// <summary>Records the second shape as modified from the first.</summary>
    public void AddModified(in Shape initial, in Shape modified)
    {
        if (!PrepareModified(initial, modified))
        {
            return;
        }

        if (!_shapeToModified.TryGetValue(initial, out NCollection_List<Shape>? modifications))
        {
            modifications = new NCollection_List<Shape>();
            _shapeToModified[initial] = modifications;
        }

        // Standard_ASSERT_VOID: "Error: a duplicated modification of a shape." (debug-only in OCCT)

        modifications.Append(modified);
    }

    /// <summary>Records the shape as removed (and no longer modified).</summary>
    public void Remove(in Shape removed)
    {
        // Apply the limitations.
        if (!IsSupportedType(removed))
        {
            return; // Standard_ASSERT_RETURN: myMsgUnsupportedType
        }

        // Standard_ASSERT_INVOKE_ myMsgModifiedAndRemoved when it was modified: a debug-only report
        _shapeToModified.Remove(removed);

        _removed.Add(removed);
    }

    /// <summary>Records the second shape as the only one generated from the first.</summary>
    public void ReplaceGenerated(in Shape initial, in Shape generated)
    {
        if (!PrepareGenerated(initial, generated))
        {
            return;
        }

        var generations = new NCollection_List<Shape>();
        _shapeToGenerated[initial] = generations;
        generations.Append(generated);
    }

    /// <summary>Records the second shape as the only one modified from the first.</summary>
    public void ReplaceModified(in Shape initial, in Shape modified)
    {
        if (!PrepareModified(initial, modified))
        {
            return;
        }

        var modifications = new NCollection_List<Shape>();
        _shapeToModified[initial] = modifications;
        modifications.Append(modified);
    }

    /// <summary>Clears the history.</summary>
    public void Clear()
    {
        _shapeToModified.Clear();
        _shapeToGenerated.Clear();
        _removed.Clear();
    }

    /// <summary>The shapes generated from the shape (empty for an unsupported type).</summary>
    public IReadOnlyList<Shape> Generated(in Shape initial)
    {
        // Apply the limitations.
        if (!(initial.IsNull || IsSupportedType(initial)))
        {
            return Array.Empty<Shape>(); // Standard_ASSERT_RETURN: myMsgUnsupportedType
        }

        //
        return _shapeToGenerated.TryGetValue(initial, out NCollection_List<Shape>? generations)
                 ? ImmutableArrays.Own([.. generations])
                 : Array.Empty<Shape>();
    }

    /// <summary>The shapes modified from the shape (empty for an unsupported type).</summary>
    public IReadOnlyList<Shape> Modified(in Shape initial)
    {
        // Apply the limitations.
        if (!IsSupportedType(initial))
        {
            return Array.Empty<Shape>(); // Standard_ASSERT_RETURN: myMsgUnsupportedType
        }

        //
        return _shapeToModified.TryGetValue(initial, out NCollection_List<Shape>? modifications)
                 ? ImmutableArrays.Own([.. modifications])
                 : Array.Empty<Shape>();
    }

    /// <summary>True if the shape is removed (false for an unsupported type).</summary>
    public bool IsRemoved(in Shape initial)
    {
        // Apply the limitations.
        if (!IsSupportedType(initial))
        {
            return false; // Standard_ASSERT_RETURN: myMsgUnsupportedType
        }

        //
        return _removed.Contains(initial);
    }

    /// <summary>True if some shape has generated shapes.</summary>
    public bool HasGenerated => _shapeToGenerated.Count != 0;

    /// <summary>True if some shape has modified shapes.</summary>
    public bool HasModified => _shapeToModified.Count != 0;

    /// <summary>True if some shape is removed.</summary>
    public bool HasRemoved => _removed.Count != 0;

    /// <summary>
    /// Every relation, for the public <c>ShapeHistory</c> (unit 10L1): each modified shape with its modifications, each
    /// generating shape with its generations, then each removed shape, in the maps' order.
    /// </summary>
    internal IEnumerable<(Shape Initial, TRelationType Kind, IReadOnlyList<Shape> Results)> Relations()
    {
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> pair in _shapeToModified)
        {
            yield return (pair.Key, TRelationType.TRelationType_Modified, [.. pair.Value]);
        }

        foreach (KeyValuePair<Shape, NCollection_List<Shape>> pair in _shapeToGenerated)
        {
            yield return (pair.Key, TRelationType.TRelationType_Generated, [.. pair.Value]);
        }

        foreach (Shape shape in _removed)
        {
            yield return (shape, TRelationType.TRelationType_Removed, []);
        }
    }

    /// <summary>
    /// Merges the next history (from this history's results to a later algorithm's) into this one, so that it answers
    /// for this history's inputs. A null history merges nothing.
    /// </summary>
    public void Merge(History? history23)
    {
        if (history23 is null)
        {
            return;
        }

        if (!(history23.HasModified || history23.HasGenerated || history23.HasRemoved))
        {
            // nothing to merge
            return;
        }

        // Propagate R23 directly and M23 and G23 fully to M12 and G12.
        // Remember the propagated shapes.
        Dictionary<Shape, NCollection_List<Shape>>[] s1ToGAndM =
            [_shapeToGenerated, _shapeToModified];
        var rPropagated = new HashSet<Shape>(ShapeHasher.Instance);
        {
            // Propagate R23, M23 and G23 to M12 and G12 directly.
            // Remember the propagated shapes.
            var mAndGPropagated = new HashSet<Shape>(ShapeHasher.Instance);
            for (int i = 0; i < 2; ++i)
            {
                // Kerf: the generated map gains keys while the modified map is walked (aI = 1), never the map
                // being walked, so the walk is over the map itself as in OCCT.
                foreach (KeyValuePair<Shape, NCollection_List<Shape>> mIt1 in s1ToGAndM[i])
                {
                    NCollection_List<Shape> l12 = mIt1.Value;
                    HashSet<Shape>[] additions =
                    [
                        new(ShapeHasher.Instance),
                        new(ShapeHasher.Instance)
                    ]; // The G and M additions.
                    for (var sIt2 = new NCollection_List<Shape>.Iterator(l12); sIt2.More();)
                    {
                        Shape s2 = sIt2.Value();
                        if (history23.IsRemoved(s2))
                        {
                            rPropagated.Add(s2);
                            l12.Remove(sIt2);
                        }
                        else
                        {
                            if (history23._shapeToGenerated.ContainsKey(s2))
                            {
                                Add(additions[0], history23._shapeToGenerated.GetValue(s2));
                                mAndGPropagated.Add(s2);
                            }

                            if (history23._shapeToModified.ContainsKey(s2))
                            {
                                Add(additions[i], history23._shapeToModified.GetValue(s2));
                                mAndGPropagated.Add(s2);

                                l12.Remove(sIt2);
                            }
                            else
                            {
                                sIt2.Next();
                            }
                        }
                    }

                    Add(l12, additions[i]);
                    if (i != 0 && additions[0].Count != 0)
                    {
                        Shape s1 = mIt1.Key;
                        if (!s1ToGAndM[0].TryGetValue(s1, out NCollection_List<Shape>? gAndM))
                        {
                            gAndM = new NCollection_List<Shape>();
                            s1ToGAndM[0][s1] = gAndM;
                        }

                        Add(gAndM, additions[0]);
                    }
                }
            }

            // Propagate M23 and G23 to M12 and G12 sequentially.
            Dictionary<Shape, NCollection_List<Shape>>[] s2ToGAndM =
                [history23._shapeToGenerated, history23._shapeToModified];
            for (int i = 0; i < 2; ++i)
            {
                foreach (KeyValuePair<Shape, NCollection_List<Shape>> mIt2 in s2ToGAndM[i])
                {
                    Shape s2 = mIt2.Key;
                    if (!mAndGPropagated.Contains(s2))
                    {
                        if (!s1ToGAndM[i].ContainsKey(s2))
                        {
                            s1ToGAndM[i][s2] = new NCollection_List<Shape>();
                        }

                        var m2 = new NCollection_List<Shape>(mIt2.Value);
                        s1ToGAndM[i].GetValue(s2).Append(m2);
                        _removed.Remove(s2);
                    }
                }
            }
        }

        // Unbound the empty M12 and G12.
        for (int i = 0; i < 2; ++i)
        {
            var empty = new NCollection_List<Shape>();
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> mIt1 in s1ToGAndM[i])
            {
                if (mIt1.Value.IsEmpty())
                {
                    empty.Append(mIt1.Key);
                }
            }
            foreach (Shape s1 in empty)
            {
                _removed.Add(s1);
                s1ToGAndM[i].Remove(s1);
            }
        }

        // Propagate R23 to R12 sequentially.
        foreach (Shape s2 in history23._removed)
        {
            if (!rPropagated.Contains(s2) && !_shapeToModified.ContainsKey(s2)
                && !_shapeToGenerated.ContainsKey(s2))
            {
                _removed.Add(s2);
            }
        }
    }

    /// <summary>
    /// Merges the history of an algorithm with the standard history methods (for the given arguments) into this one.
    /// </summary>
    public void Merge(NCollection_List<Shape> arguments, IAlgorithm algo)
    {
        // Create new history object from the given algorithm and merge it into this.
        Merge(new History(arguments, algo));
    }

    /// <summary>Writes the numbers of deleted, modified and generated shapes.</summary>
    public void Dump(TextWriter s)
    {
        s.Write("History contains:\n");
        s.Write(" - " + _removed.Count + " Deleted shapes;\n");
        s.Write(" - " + _shapeToModified.Count + " Modified shapes;\n");
        s.Write(" - " + _shapeToGenerated.Count + " Generated shapes.\n");
    }

    // Prepares the shapes generated from the first shape to set the second one
    // as generated one from the first one by the addition or the replacement.
    // Returns 'true' on success.
    private bool PrepareGenerated(in Shape initial, in Shape generated)
    {
        if (!(initial.IsNull || IsSupportedType(initial)))
        {
            return false; // Standard_ASSERT_RETURN: myMsgUnsupportedType
        }

        // Standard_ASSERT_INVOKE_ myMsgGeneratedAndModified when removed: a debug-only report
        if (_shapeToModified.TryGetValue(initial, out NCollection_List<Shape>? modified))
        {
            modified.Remove(generated);
        }

        return true;
    }

    // Prepares the shapes modified from the first shape to set the second one
    // as modified one from the first one by the addition or the replacement.
    // Returns 'true' on success.
    private bool PrepareModified(in Shape initial, in Shape modified)
    {
        if (!IsSupportedType(initial))
        {
            return false; // Standard_ASSERT_RETURN: myMsgUnsupportedType
        }

        // Standard_ASSERT_INVOKE_ myMsgModifiedAndRemoved when removed: a debug-only report
        _removed.Remove(initial);

        // Standard_ASSERT_INVOKE_ myMsgGeneratedAndModified when removed: a debug-only report
        if (_shapeToGenerated.TryGetValue(initial, out NCollection_List<Shape>? generated))
        {
            generated.Remove(modified);
        }

        return true;
    }

    // Adds the elements of the list to the map.
    private static void Add(HashSet<Shape> map, NCollection_List<Shape> list)
    {
        for (var sIt = new NCollection_List<Shape>.Iterator(list); sIt.More(); sIt.Next())
        {
            map.Add(sIt.Value());
        }
    }

    // Adds the elements of the collection to the list.
    private static void Add(NCollection_List<Shape> list, HashSet<Shape> collection)
    {
        foreach (Shape s in collection)
        {
            list.Append(s);
        }
    }
}
