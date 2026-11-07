// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Meshing.Tools;

/// <summary>
/// How many triangles a public mesh may have (decision 1856). A deflection a thousand times too fine asks for a million
/// times the triangles, and takes all the memory there is, so a face stops as soon as its nodes show it can't fit. Null in
/// <see cref="ToolParameters"/> for the engine's own meshes, which OCCT doesn't limit. Faces may mesh in parallel, so the total
/// is counted atomically; it decides only whether to stop, so the order can't change a mesh that finishes.
/// </summary>
internal sealed class MeshBudget(long maxTriangles)
{
    private long _committed;
    private int _refusedFaces;

    /// <summary>Stops if a face of this many nodes, about two triangles each, can't fit in what's left.</summary>
    /// <exception cref="MeshTooLargeException">It can't.</exception>
    public void Check(long nodes)
    {
        if (Volatile.Read(ref _committed) + 2 * nodes > maxTriangles)
        {
            throw new MeshTooLargeException(maxTriangles);
        }
    }

    /// <summary>
    /// How many circles in cells a face may hold. A circle is entered in every cell it covers, and a long thin triangle's
    /// covers thousands, so this is what runs out first: measured at 13 for each triangle on a sphere of 300,000, and about 3
    /// GB of memory at the limit. A mesh of the most triangles allowed fits, as 16 a triangle.
    /// </summary>
    public int MaxCircleEntries => (int)Math.Min(16 * maxTriangles, 1 << 30);

    /// <summary>Stops the mesh.</summary>
    /// <exception cref="MeshTooLargeException">Always.</exception>
    public void Refuse() => throw new MeshTooLargeException(maxTriangles);

    /// <summary>A face's mesh is finished, with this many triangles.</summary>
    public void Commit(int triangles) => Interlocked.Add(ref _committed, triangles);

    /// <summary>Whether a face of this many nodes, about two triangles each, fits in what's left; false for NaN.</summary>
    public bool Fits(double nodes) => Volatile.Read(ref _committed) + 2 * nodes <= maxTriangles;

    /// <summary>A face was refused and left without triangles (decision 22801); the rest of the mesh carries on.</summary>
    public void RefuseFace() => Interlocked.Increment(ref _refusedFaces);

    /// <summary>How many faces were refused.</summary>
    public int RefusedFaces => Volatile.Read(ref _refusedFaces);
}

/// <summary>
/// A face's mesh would take far more memory than its triangles justify, from a surface or range out of proportion to
/// the face, as a damaged number in a file makes (decision 22801). Not a failure of the engine's kind, so no catch of
/// those turns it into a face that fails without saying so: <see cref="BaseMeshAlgo"/> fails the face and counts it.
/// </summary>
internal sealed class MeshFaceRefusedException(string message) : Exception(message);

/// <summary>
/// A mesh would pass <see cref="Kerf.Meshing.MeshOptions.MaxTriangles"/>. Not a failure of the engine's kind: a face's
/// algorithm treats those as that face failing and carries on, and this ends the whole mesh.
/// </summary>
internal sealed class MeshTooLargeException(long maxTriangles)
    : Exception($"The mesh would have more than {maxTriangles} triangles: the deflection is too fine for this shape, or the limit, MeshOptions.MaxTriangles, too low.");
