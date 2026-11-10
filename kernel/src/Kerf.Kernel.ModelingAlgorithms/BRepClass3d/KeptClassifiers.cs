// Kerf: a frozen shape's classifier, kept with its TShape for the next point asked of it (not a port of an OCCT file; unit 15C).
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Classification;

/// <summary>
/// Classifiers of frozen shapes, kept with their <c>TShape</c>s, so that points asked one at a time of one shape share one
/// classifier: its faces' intersectors and the tree of its edges and vertices are made once, not for every point.
/// </summary>
/// <remarks>
/// <para>
/// A classifier's explorer makes an intersector for each face, the tree of its edges' and vertices' boxes and the faces of each
/// edge, and each intersector its face's box and 2D classifier when first asked: all functions of the shape, its placement and
/// its orientation, which a frozen <c>TShape</c> never changes. What one point leaves for the next is the explorer's choice of
/// face and of where on an edge to aim (OCCT's <c>myFirstFace</c> and <c>myParamOnEdge</c>), which
/// <see cref="SolidClassifier.ForANewPoint"/> sets back to what a new classifier starts from; a classification sets the rest
/// afresh. So a kept classifier gives each point what a new one would. Nothing is kept or read inside an open working overlay,
/// where a frozen <c>TShape</c> can read as its working copy, nor for a shape that isn't frozen.
/// </para>
/// <para>
/// Threads: a classifier is lent to one caller at a time, taken out of its shape's slots under their lock and put back under it,
/// so it's never used on two threads at once, and the lock orders one caller's use before the next's. Two callers at once on one
/// shape get a classifier each, and both are kept, up to <c>PerShape</c>. The table holds nothing alive: an entry goes with
/// its <c>TShape</c>.
/// </para>
/// </remarks>
internal static class KeptClassifiers
{
    private static readonly ConditionalWeakTable<TShape, Slots> Kept = new();

    // At most this many placements, orientations and callers at once of one TShape
    private const int PerShape = 4;

    // The classifiers this thread made afresh and those it was lent, for the tests
    [ThreadStatic]
    internal static int s_made;

    [ThreadStatic]
    internal static int s_lent;

    /// <summary>The state of <paramref name="point"/> in <paramref name="shape"/> with the tolerance <paramref name="tolerance"/>,
    /// as a new classifier would give it (<see cref="SClassifier.State"/>, which is ON exactly when the point is on a face).</summary>
    internal static Containment State(in Shape shape, in Point3D point, double tolerance)
    {
        SolidClassifier classifier = Rent(shape, out Slots? slots);
        classifier.Perform(point, tolerance);
        Containment state = classifier.State();
        slots?.Return(shape, classifier);
        return state;
    }

    private static SolidClassifier Rent(in Shape shape, out Slots? slots)
    {
        slots = null;
        if (shape.TShape() is { IsFrozen: true } tShape && WorkingOverlay.Current is null)
        {
            slots = Kept.GetValue(tShape, static _ => new Slots());
            if (slots.Take(shape) is { } kept)
            {
                s_lent++;
                kept.ForANewPoint();
                return kept;
            }
        }
        s_made++;
        return new SolidClassifier(shape);
    }

    private sealed class Slots
    {
        private readonly List<(Placement Placement, Orientation Orientation, SolidClassifier? Classifier)> _slots = [];

        public SolidClassifier? Take(in Shape shape)
        {
            lock (_slots)
            {
                for (int i = 0; i < _slots.Count; i++)
                {
                    var (placement, orientation, classifier) = _slots[i];
                    if (classifier is not null && orientation == shape.Orientation && placement.IsEqual(shape.Placement))
                    {
                        _slots[i] = (placement, orientation, null);
                        return classifier;
                    }
                }
                return null;
            }
        }

        public void Return(in Shape shape, SolidClassifier classifier)
        {
            lock (_slots)
            {
                for (int i = 0; i < _slots.Count; i++)
                {
                    var (placement, orientation, held) = _slots[i];
                    if (held is null && orientation == shape.Orientation && placement.IsEqual(shape.Placement))
                    {
                        _slots[i] = (placement, orientation, classifier);
                        return;
                    }
                }
                if (_slots.Count < PerShape)
                {
                    _slots.Add((shape.Placement, shape.Orientation, classifier));
                }
            }
        }
    }
}
