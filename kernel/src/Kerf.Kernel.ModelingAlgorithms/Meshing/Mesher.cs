// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using System.Globalization;
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

    /// <summary>
    /// Some faces' meshes are further from their surfaces than the deflection: <c>meshing.deflection-exceeded</c>, a
    /// warning naming the faces (<c>Shapes</c>) and giving the furthest, as a share of the deflection and in model units
    /// (decision 2071). Each sphere and torus is measured as it's meshed and refined until it keeps to the deflection; a
    /// face it can't be refined on stays as it is, and the mesh says so. Met only at relative deflections, on small fillet
    /// tori, up to 1.14 times out; other kinds of surface aren't measured.
    /// </summary>
    public static DiagnosticCode DeflectionExceeded { get; } = new("meshing.deflection-exceeded");

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
        /// (<see cref="MeshFailed"/>) carrying the mesh, with its edges' polylines and no triangles. A success or a failure
        /// carrying the mesh also warns (<see cref="DeflectionExceeded"/>) of faces further from their surfaces than the
        /// deflection.</returns>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
        public Result<ShapeMesh> Mesh(MeshOptions? options = null, IProgress<double>? progress = null, CancellationToken cancellationToken = default)
        {
            Shape input = shape.AsShape();
            if (input.IsNull)
            {
                throw new KernelArgumentException("The shape is null.", nameof(shape));
            }

            options ??= MeshOptions.Default;
            cancellationToken = Cancellation.Effective(cancellationToken);
            using Cancellation.Scope cancellation = Cancellation.Enter(cancellationToken);
            cancellationToken.ThrowIfCancellationRequested();
            var indicator = new ProgressIndicator(cancellationToken, progress);
            ShapeMesh? mesh;
            int status;
            ImmutableArray<ExceededFace> exceeded;
            ToolParameters parameters = options.ToEngine();
            try
            {
                mesh = IncrementalMesh.ReadMeshed(input, parameters, target => Read(input, target, options), indicator.Start(),
                                                  out status, out exceeded);
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

            ImmutableArray<Diagnostic> warnings = exceeded.IsEmpty ? [] : [DeflectionWarning(exceeded, options.Relative, null)];
            if (parameters.Budget is { RefusedFaces: > 0 and int refused })
            {
                return Result<ShapeMesh>.Invalid(mesh, Diagnostic.Error(FacesRefused,
                    $"{refused} {(refused == 1 ? "face was" : "faces were")} refused and left without triangles: their mesh would "
                    + "take far more memory than their size and the deflection justify, as a damaged surface or range in a file does."), warnings);
            }

            if ((status & (int)Engine.Meshing.Model.StatusModel.Failure) != 0)
            {
                if (mesh.TriangleCount == 0)
                {
                    // Not one face meshed: an invalid result, never a success (decision 1742), carrying the edges' polylines
                    return Result<ShapeMesh>.Invalid(mesh, Diagnostic.Error(MeshFailed, "No face could be meshed: the mesh has no triangles."), warnings);
                }

                return Result<ShapeMesh>.Success(mesh, [Diagnostic.Warning(FacesNotMeshed, "Some faces couldn't be meshed: they have no triangles."), .. warnings]);
            }

            return Result<ShapeMesh>.Success(mesh, warnings);
        }
    }

    /// <summary>
    /// The warning (<see cref="DeflectionExceeded"/>) for faces whose meshes their guards left further from the surface
    /// than the deflection, naming them, of the part named <paramref name="part"/> if given (decision 2071).
    /// </summary>
    internal static Diagnostic DeflectionWarning(ImmutableArray<ExceededFace> faces, bool relative, string? part)
    {
        ExceededFace worst = faces[0];
        foreach (ExceededFace face in faces)
        {
            if (face.Deviation / face.Limit > worst.Deviation / worst.Limit)
            {
                worst = face;
            }
        }

        const int listed = 10;
        string indices = string.Join(", ", faces.Take(listed).Select(f => f.Index.ToString(CultureInfo.InvariantCulture)))
                         + (faces.Length > listed ? $" and {faces.Length - listed} more" : "");
        string of = part is null ? "" : $" of {part}";
        string figures = string.Create(CultureInfo.InvariantCulture,
            $"{worst.Deviation:G4} from it, {worst.Deviation / worst.Limit:F3} times the deflection of {worst.Limit:G4}{(relative ? " (relative mode's for that face)" : "")}");
        string message = faces.Length == 1
            ? $"Face {worst.Index}{of} is further from its surface than the deflection: {figures}. The mesher couldn't refine it further."
            : $"{faces.Length} faces{of} (faces {indices}) are further from their surfaces than the deflection: the furthest, face {worst.Index}, is "
              + $"{figures}. The mesher couldn't refine them further.";
        return Diagnostic.Warning(DeflectionExceeded, message).WithShapes(faces.Select(f => f.Face));
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
