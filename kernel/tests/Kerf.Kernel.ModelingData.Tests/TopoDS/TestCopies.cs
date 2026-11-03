// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Topology.Tools;

/// <summary>
/// An unfrozen exact copy of a shape, for tests that run ported in-place code on a shape it may write into, as OCCT's
/// tests do on the shapes they build (the library's <c>BrepTools.ExactCopyForWrite</c>, kept here for the tests when
/// unit 10H6 removed copy at entry from the library).
/// </summary>
/// <remarks>
/// Each TShape is copied once (<see cref="BrepTools.CopyData"/> and <see cref="BrepTools.CopyFlags"/>: its BRep data in
/// new representations sharing the geometry, its flags), with its children at the same locations and orientations
/// (their Free flag cleared, as adding them clears it). Nothing is recomputed, so in-place code sees on the copy what
/// it would see on the original.
/// </remarks>
internal static class TestCopies
{
    /// <summary>
    /// The exact copy of <paramref name="shape"/>, unfrozen; <paramref name="history"/> records each vertex, edge, face
    /// and solid as modified into its copy.
    /// </summary>
    public static Shape ExactCopyForWrite(in Shape shape, out History history)
    {
        history = new History();
        if (shape.IsNull)
        {
            return new Shape();
        }

        var copies = new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
        Shape copy = shape;
        copy.TShape(ExactCopy(shape.TShape()!, copies));

        var map = new Occt.NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(shape, map);
        for (int i = 1; i <= map.Extent(); i++)
        {
            Shape s = map[i];
            if (History.IsSupportedType(s) && copies.TryGetValue(s.TShape()!, out TShape? image))
            {
                Shape aNew = s;
                aNew.TShape(image);
                history.AddModified(s, aNew);
            }
        }
        return copy;
    }

    /// <summary>True if <paramref name="shape"/>'s graph holds a frozen TShape.</summary>
    public static bool HoldsFrozen(in Shape shape) => !WorkingOverlay.IsWritableInPlace(shape);

    private static TShape ExactCopy(TShape tShape, Dictionary<TShape, TShape> copies)
    {
        if (copies.TryGetValue(tShape, out TShape? done))
        {
            return done;
        }

        RuntimeHelpers.EnsureSufficientExecutionStack();
        TShape copy = BrepTools.CopyData(tShape);
        for (var it = new Occt.NCollection_List<Shape>.Iterator(tShape._shapes); it.More(); it.Next())
        {
            Shape child = it.Value();
            TShape childCopy = ExactCopy(child.TShape()!, copies);
            childCopy.Free(false);
            child.TShape(childCopy);
            copy._shapes.Append(child);
        }

        BrepTools.CopyFlags(tShape, copy);
        copies[tShape] = copy;
        return copy;
    }
}
