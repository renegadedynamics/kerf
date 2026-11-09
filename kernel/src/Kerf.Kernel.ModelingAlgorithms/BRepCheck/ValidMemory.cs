// Kerf: what a check remembers of the faces it found clean (not a port of an OCCT file). docs/performance.md, "What a check can
// remember" (unit 11R) and "The checks' share of a Boolean" (unit 13K).
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;

namespace Kerf.Engine.Healing.Checks;

/// <summary>
/// The frozen faces a whole check found nothing wrong under, each at the placement and orientation it was found at, kept with
/// its <c>TShape</c>: a check that meets one again there skips it (<see cref="Analyzer.Remembering"/>).
/// </summary>
/// <remarks>
/// <para>
/// What a check finds under a face (its vertices and edges in its context, its wires, the face itself) reads only the face's
/// own graph, its placement and its orientation (unit 11R, decision 1799): the shell's and solid's checks read the faces
/// directly, not their results. A frozen <c>TShape</c>, its children and their geometry never change again, so the check made
/// again would find the same. Only a check in which every status of every sub-shape, in every context, is no fault is kept
/// (decision 1800), and nothing is kept or read inside an open working overlay, where a frozen <c>TShape</c> can read as its
/// working copy.
/// </para>
/// <para>
/// A chain of Booleans checks each result, and most of a result's faces are its object's, checked as the last result: 85% of
/// the faces over unit 12Y's eight parts. A Boolean's untouched faces are its inputs' <c>TShape</c>s.
/// </para>
/// <para>
/// Threads: the table is safe between threads, and a face's entry is a list whose nodes never change, read and published
/// with <c>Volatile</c>; two threads keeping one at once can lose one, which is only a face checked again. The table holds
/// nothing alive: an entry goes with its <c>TShape</c>.
/// </para>
/// </remarks>
internal static class ValidMemory
{
    private static readonly ConditionalWeakTable<TShape, Placed> Kept = new();

    // At most this many placements and orientations of one face: a face moved about as a tool has one a move
    private const int KeptPerFace = 4;

    /// <summary>True if a whole check found nothing wrong under this frozen face, as placed and oriented here.</summary>
    internal static bool Knows(in Shape face)
    {
        if (face.TShape() is not { IsFrozen: true } tShape || WorkingOverlay.Current is not null || !Kept.TryGetValue(tShape, out Placed? kept))
        {
            return false;
        }

        for (Placed.Node? node = kept.First; node is not null; node = node.Next)
        {
            if (node.Orientation == face.Orientation && node.Placement.IsEqual(face.Placement))
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>Keeps a whole check's clean result under a frozen face; nothing for one that isn't frozen.</summary>
    internal static void Remember(in Shape face)
    {
        if (face.TShape() is not { IsFrozen: true } tShape || WorkingOverlay.Current is not null || Knows(face))
        {
            return;
        }

        Placed kept = Kept.GetValue(tShape, static _ => new Placed());
        kept.Add(face.Placement, face.Orientation);
    }

    private sealed class Placed
    {
        private Node? _first;

        public Node? First => Volatile.Read(ref _first);

        public void Add(Placement placement, Orientation orientation)
        {
            Node? first = Volatile.Read(ref _first);
            if ((first?.Count ?? 0) < KeptPerFace)
            {
                Volatile.Write(ref _first, new Node(placement, orientation, first));
            }
        }

        public sealed class Node(Placement placement, Orientation orientation, Node? next)
        {
            public readonly Placement Placement = placement;
            public readonly Orientation Orientation = orientation;
            public readonly Node? Next = next;
            public readonly int Count = (next?.Count ?? 0) + 1;
        }
    }
}
