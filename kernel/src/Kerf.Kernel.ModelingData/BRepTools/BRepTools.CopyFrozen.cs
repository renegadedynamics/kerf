// OCCT.NET: copying the frozen parts of a shape an algorithm works on in place (unit 6G). Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Topology;

using Occt;

namespace Kerf.Engine.Topology.Tools;

public static partial class BrepTools
{
    /// <summary>
    /// Makes an unfrozen shape that an algorithm is about to write into in place (its own intermediate result) free of
    /// frozen TShapes: each frozen sub-graph is replaced, in its unfrozen parents, by an exact copy
    /// (<see cref="ExactCopyForWrite(in Shape, out History)"/>'s), and a frozen shape is replaced as a
    /// whole. The unfrozen TShapes stay the same objects, so the in-place writes still reach every shape that holds
    /// them, as in OCCT, while the frozen originals (a caller's or published shapes) are never written.
    /// </summary>
    /// <param name="shape">The shape (it is returned as is when nothing in it is frozen).</param>
    /// <param name="history">Records each vertex, edge, face and solid of the shape, as placed in it, replaced by its
    /// copy as modified into the copy.</param>
    /// <returns>The shape, or its copy when it is frozen itself.</returns>
    internal static Shape CopyFrozenForWrite(in Shape shape, History history)
    {
        if (shape.IsNull || !HoldsFrozen(shape))
        {
            return shape;
        }

        // the sub-shapes as placed in the shape, before the replacement
        var map = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(shape, map);

        var copies = new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
        var visited = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        Shape result = shape;
        result.TShape(ReplaceFrozen(shape.TShape()!, copies, visited));

        for (int i = 0; i < map.Count; i++)
        {
            Shape s = map[i];
            if (History.IsSupportedType(s) && copies.TryGetValue(s.TShape()!, out TShape? copy)
                && !ReferenceEquals(copy, s.TShape()))
            {
                Shape aNew = s;
                aNew.TShape(copy);
                history.AddModified(s, aNew);
            }
        }
        return result;
    }

    // A frozen TShape is copied; an unfrozen one gets its frozen children replaced in place (each TShape once)
    private static TShape ReplaceFrozen(TShape tShape, Dictionary<TShape, TShape> copies,
                                               HashSet<TShape> visited)
    {
        if (tShape.IsFrozen)
        {
            return ExactCopy(tShape, copies);
        }

        if (!visited.Add(tShape))
        {
            return tShape;
        }

        bool changed = false;
        List<Shape> children = [];
        for (var it = new NCollection_List<Shape>.Iterator(tShape._shapes); it.More(); it.Next())
        {
            Shape child = it.Value();
            TShape childT = child.TShape()!;
            TShape newT = ReplaceFrozen(childT, copies, visited);
            if (!ReferenceEquals(newT, childT))
            {
                newT.Free(false); // as TopoDS_Builder::Add clears it
                child.TShape(newT);
                changed = true;
            }
            children.Add(child);
        }

        if (changed)
        {
            tShape._shapes.Clear();
            foreach (Shape child in children)
            {
                tShape._shapes.Append(child);
            }
        }
        return tShape;
    }
}
