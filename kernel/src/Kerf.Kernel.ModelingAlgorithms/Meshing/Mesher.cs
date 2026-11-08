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

    /// <summary>The mesh would pass <see cref="MeshOptions.MaxTriangles"/>.</summary>
    public static DiagnosticCode TooLarge { get; } = new("meshing.too-large");

    /// <summary>
    /// Some faces were refused, as what their mesh would take is out of proportion to the face, from a damaged surface or
    /// range (decision 1942): <c>meshing.faces-refused</c>. The failure carries the rest of the mesh.
    /// </summary>
    public static DiagnosticCode FacesRefused { get; } = new("meshing.faces-refused");

    extension<TShape>(TShape shape) where TShape : struct, IShape<TShape>
    {
        /// <summary>
        /// The shape's mesh (OCCT's <c>BRepMesh_IncrementalMesh</c>, read out as Kerf.Modeling's and Bento's viewers read
        /// it). The shape isn't changed: it is meshed in a working overlay that is dropped, so its faces hold no
        /// triangles afterwards and it can be meshed from any thread. A face that already holds a mesh fine enough
        /// (one read from a BREP file) keeps it.
        /// </summary>
        /// <returns>The mesh, with a warning (<see cref="FacesNotMeshed"/>) if some faces couldn't be meshed; or a
        /// failure (<see cref="MeshFailed"/>, <see cref="TooLarge"/>, or the engine's error); or a failure
        /// (<see cref="FacesRefused"/>) carrying the mesh of the other faces; or, if no face could be meshed, a failure
        /// (<see cref="MeshFailed"/>) carrying the mesh, with its edges' polylines and no triangles.</returns>
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
            cancellationToken = Cancellation.Effective(cancellationToken);
            using Cancellation.Scope cancellation = Cancellation.Enter(cancellationToken);
            cancellationToken.ThrowIfCancellationRequested();
            var indicator = new ProgressIndicator(cancellationToken, progress);
            ShapeMesh? mesh;
            int status;
            ToolParameters parameters = options.ToEngine();
            try
            {
                mesh = IncrementalMesh.ReadMeshed(input, parameters, target => Read(input, target, options), indicator.Start(),
                                                  out status);
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (MeshTooLargeException e)
            {
                return Result<ShapeMesh>.Failure(Diagnostic.Error(TooLarge, e.Message));
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

            if (parameters.Budget is { RefusedFaces: > 0 and int refused })
            {
                return Result<ShapeMesh>.Invalid(mesh, Diagnostic.Error(FacesRefused,
                    $"{refused} {(refused == 1 ? "face was" : "faces were")} refused and left without triangles: their mesh would "
                    + "take far more memory than their size and the deflection justify, as a damaged surface or range in a file does."));
            }

            if ((status & (int)Engine.Meshing.Model.StatusModel.Failure) != 0)
            {
                if (mesh.TriangleCount == 0)
                {
                    // Not one face meshed: an invalid result, never a success (decision 1742), carrying the edges' polylines
                    return Result<ShapeMesh>.Invalid(mesh, Diagnostic.Error(MeshFailed, "No face could be meshed: the mesh has no triangles."));
                }

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
