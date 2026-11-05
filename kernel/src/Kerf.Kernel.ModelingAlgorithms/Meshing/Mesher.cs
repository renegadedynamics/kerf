// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Tools;
using Kerf.Topology;

namespace Kerf.Meshing;

/// <summary>
/// Meshing (docs/api/meshing.md): a shape's faces triangulated and its edges' polylines, as a separate
/// <see cref="ShapeMesh"/>. Also a C# 14 extension member, so <c>Mesher.Mesh(part)</c> and <c>part.Mesh()</c> both work.
/// </summary>
public static class Mesher
{
    /// <summary>The code of a mesh that couldn't be made: <c>meshing.failed</c>.</summary>
    public static DiagnosticCode MeshFailed { get; } = new("meshing.failed");

    /// <summary>The code of a warning that some faces couldn't be meshed: <c>meshing.faces-not-meshed</c>.</summary>
    public static DiagnosticCode FacesNotMeshed { get; } = new("meshing.faces-not-meshed");

    extension<TShape>(TShape shape) where TShape : struct, IShape<TShape>
    {
        /// <summary>
        /// The shape's mesh (OCCT's <c>BRepMesh_IncrementalMesh</c>, read out as build123d.NET's and Bento's viewers read
        /// it). The shape isn't changed: it is meshed in a working overlay that is dropped, so its faces hold no
        /// triangles afterwards and it can be meshed from any thread. A face that already holds a mesh fine enough
        /// (one read from a BREP file) keeps it.
        /// </summary>
        /// <returns>The mesh, with a warning (<see cref="FacesNotMeshed"/>) if some faces couldn't be meshed; or a
        /// failure (<see cref="MeshFailed"/>, or the engine's error).</returns>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
        public Result<ShapeMesh> Mesh(MeshOptions? options = null, IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            Shape input = shape.AsShape();
            if (input.IsNull)
            {
                throw new ArgumentException("The shape is null.", nameof(shape));
            }

            options ??= MeshOptions.Default;
            cancellationToken.ThrowIfCancellationRequested();
            var indicator = new ProgressIndicator(cancellationToken, progress);
            ShapeMesh? mesh;
            int status;
            try
            {
                mesh = IncrementalMesh.ReadMeshed(input, options.ToEngine(), target => Read(input, target, options), indicator.Start(),
                                                  out status);
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
            {
                cancellationToken.ThrowIfCancellationRequested();
                return Result<ShapeMesh>.Failure(OperationDiagnostics.FromException(e));
            }

            cancellationToken.ThrowIfCancellationRequested();
            if (mesh is null)
            {
                return Result<ShapeMesh>.Failure(Diagnostic.Error(MeshFailed, "The shape couldn't be meshed."));
            }

            if ((status & (int)Engine.Meshing.Model.StatusModel.Failure) != 0)
            {
                return Result<ShapeMesh>.Success(mesh, [Diagnostic.Warning(FacesNotMeshed, "Some faces couldn't be meshed: they have no triangles.")]);
            }

            return mesh;
        }
    }

    // Reads the meshes while the mesher's overlay is open: target is the input, or a copy of it of the same structure
    private static ShapeMesh Read(in Shape input, in Shape target, MeshOptions options)
    {
        List<Shape> faces = MeshExtraction.Faces(target), edges = MeshExtraction.Edges(target);
        MeshBuffers buffers = MeshExtraction.Extract(target, faces, edges, options.EdgeDeflection ?? options.LinearDeflection,
                                                     options.AngularDeflection, out Triangulation?[] triangulations, out Placement[] placements);
        if (!ReferenceEquals(input.TShape(), target.TShape()))
        {
            // The sub-shapes the caller knows: the input's, in the same order as the copy's
            faces = MeshExtraction.Faces(input);
            edges = MeshExtraction.Edges(input);
        }

        return new ShapeMesh(input, options, buffers, faces, edges, triangulations, placements);
    }
}
