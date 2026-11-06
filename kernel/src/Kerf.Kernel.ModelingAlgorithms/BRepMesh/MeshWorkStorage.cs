// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Meshing;

/// <summary>
/// A thread's lists of triangles and circles for meshing one face, kept for the next face, as OCCT's face algorithm gets its
/// memory from one allocator rather than growing a new array each time. The set is out of the static while a face uses it, so
/// a nested use makes its own and nothing crosses threads. A list is empty when it's rented, and reads go by its count, so a
/// face can't see the last one's values. A thread keeps about 4 MB at most, for its life.
/// </summary>
internal sealed class MeshWorkStorage
{
    private const int LargestKept = 1 << 16;

    [ThreadStatic]
    private static MeshWorkStorage? t_spare;

    /// <summary>The triangles of the face's structure.</summary>
    public readonly List<Triangle> Elements = new(256);

    /// <summary>The circles of the structure's first Delaunay.</summary>
    public readonly List<MeshCircle> Circles = new(256);

    /// <summary>Whether a Delaunay has the circles.</summary>
    public bool CirclesTaken;

    /// <summary>The thread's set, or a new one.</summary>
    public static MeshWorkStorage Rent()
    {
        MeshWorkStorage storage = t_spare ?? new MeshWorkStorage();
        t_spare = null;
        storage.CirclesTaken = false;
        return storage;
    }

    /// <summary>Keeps the set for the thread's next face, unless it has grown too large.</summary>
    public static void Return(MeshWorkStorage storage)
    {
        if (storage.Elements.Capacity <= LargestKept && storage.Circles.Capacity <= LargestKept)
        {
            storage.Elements.Clear();
            storage.Circles.Clear();
            t_spare = storage;
        }
    }
}
