// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using Kerf.Engine.Exchange.Gltf;
using Kerf.Engine.Exchange.Obj;
using Kerf.Engine.Exchange.Stl;
using Kerf.Engine.Exchange.Stl.Api;
using Kerf.Engine.Exchange.ThreeMf;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Meshing;
using Kerf.Topology;
using EngineReadOptions = Kerf.Engine.Exchange.Gltf.ReadOptions;
using EngineTreeReader = Kerf.Engine.Exchange.Gltf.TreeReader;

namespace Kerf.Exchange;

/// <summary>
/// STL: meshes read and written (OCCT's RWStl and StlAPI). A mesh read has no shape:
/// <see cref="ShapeMesh.Shape"/> is null.
/// </summary>
public static class Stl
{
    /// <summary>
    /// The mesh of STL data, binary or ASCII, its coincident nodes merged, scaled from <see cref="StlReadOptions.Unit"/>
    /// to millimetres, within the limits (bytes, triangles).
    /// </summary>
    /// <returns>The mesh, or a failure: <c>exchange.malformed</c> for data holding no triangles or past the limits.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="stream"/> is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result<ShapeMesh> Read(Stream stream, StlReadOptions? options = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        options ??= StlReadOptions.Default;
        cancellationToken.ThrowIfCancellationRequested();
        var limits = new StlReadLimits
        {
            MaxBytes = options.Limits.MaxBytes,
            MaxTriangles = (int)Math.Min(options.Limits.MaxTriangles, int.MaxValue),
        };
        Triangulation? mesh;
        try
        {
            mesh = RWStl.ReadFile(stream, Math.PI / 2.0, new ProgressIndicator(cancellationToken).Start(), limits);
        }
        catch (Exception e) when (e is not OperationCanceledException && ExchangeTree.IsBadData(e))
        {
            cancellationToken.ThrowIfCancellationRequested();
            return Result<ShapeMesh>.Failure(ExchangeTree.BadData(e, "STL"));
        }

        cancellationToken.ThrowIfCancellationRequested();
        if (mesh is null || mesh.TriangleCount == 0)
        {
            return Result<ShapeMesh>.Failure(Diagnostic.Error(ExchangeTree.Malformed, "The STL data holds no triangles, or is larger than the limits."));
        }

        double scale = options.Unit.Millimetres();
        if (scale != 1.0)
        {
            // The reader's triangulations are published: scale a copy
            mesh = mesh.Copy();
            for (int i = 1; i <= mesh.NodeCount; i++)
            {
                Point3D p = mesh.Node(i);
                mesh.SetNode(i, new Point3D(p.X * scale, p.Y * scale, p.Z * scale));
            }
        }

        var face = new Face();
        new BrepBuilder().MakeFace(ref face, mesh);
        face.TShape()!.FreezeGraph();
        return ShapeMesh.FromMeshOnlyShape(face);
    }

    /// <summary>Writes a mesh as STL (OCCT's <c>StlAPI_Writer</c>): binary, or ASCII with LF line ends.</summary>
    /// <returns>Success, or a failure (<c>exchange.write-failed</c>) for a mesh without triangles.</returns>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result Write(ShapeMesh mesh, Stream stream, StlWriteOptions? options = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        ArgumentNullException.ThrowIfNull(stream);
        options ??= StlWriteOptions.Default;
        cancellationToken.ThrowIfCancellationRequested();
        var writer = new ApiWriter();
        writer.SetASCIIMode(options.Ascii);
        bool done = writer.Write(mesh.ToMeshOnlyShape(), stream, new ProgressIndicator(cancellationToken).Start());
        cancellationToken.ThrowIfCancellationRequested();
        return done ? Result.Success() : Result.Failure(Diagnostic.Error(ExchangeTree.WriteFailed, "The mesh has no triangles to write as STL."));
    }
}

/// <summary>glTF 2.0: models read (as meshes) and written (Kerf's RWGltf tree reader and writer).</summary>
public static class Gltf
{
    // The placeholder URI of a buffer embedded afterwards, in a glTF written to one stream
    private const string EmbeddedBuffer = "kerf-embedded-buffer.bin";

    /// <summary>
    /// The model of glTF 2.0 data (<c>.glb</c>, or <c>.gltf</c> with its buffers embedded), Y up turned Z up, in
    /// <see cref="GltfReadOptions.Unit"/>, within the limits (bytes, nodes, depth, triangles): each part a mesh, with
    /// its name and colour; a scene of several parts or a node hierarchy an assembly.
    /// </summary>
    /// <returns>The model, or a failure (<c>exchange.malformed</c>).</returns>
    /// <exception cref="ArgumentNullException"><paramref name="stream"/> is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result<ModelNode> Read(Stream stream, GltfReadOptions? options = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        options ??= GltfReadOptions.Default;
        cancellationToken.ThrowIfCancellationRequested();
        var engineOptions = new EngineReadOptions
        {
            OutputLengthUnit = options.Unit.Millimetres() / 1000.0,
            Limits = GltfReadLimits.Default with
            {
                MaxBytes = options.Limits.MaxBytes,
                MaxNodes = options.Limits.MaxNodes,
                MaxDepth = options.Limits.MaxDepth,
                MaxTriangles = options.Limits.MaxTriangles,
            },
        };
        var reader = new EngineTreeReader(engineOptions);
        Engine.Exchange.Documents.Tree? tree;
        try
        {
            tree = reader.Read(stream, null, new ProgressIndicator(cancellationToken).Start());
        }
        catch (Exception e) when (e is not OperationCanceledException && ExchangeTree.IsBadData(e))
        {
            cancellationToken.ThrowIfCancellationRequested();
            return Result<ModelNode>.Failure(ExchangeTree.BadData(e, "glTF"));
        }

        cancellationToken.ThrowIfCancellationRequested();
        if (tree is null)
        {
            return Result<ModelNode>.Failure(Diagnostic.Error(ExchangeTree.Malformed, $"The glTF data couldn't be read: {reader.Error}"));
        }

        return ExchangeTree.FromTree(tree, productNames: false, meshes: true) is { } model
            ? model
            : Result<ModelNode>.Failure(Diagnostic.Error(ExchangeTree.Malformed, "The glTF data holds no mesh."));
    }

    /// <summary>
    /// Writes a model as glTF 2.0: binary (<c>.glb</c>) or JSON with its buffer embedded; Z up turned Y up, and the unit
    /// into metres, as glTF wants; a material per colour; a node per assembly child, named by its name. The parts'
    /// shapes are meshed with <see cref="GltfWriteOptions.Mesh"/>; meshes are written as they are.
    /// </summary>
    /// <returns>Success, or a failure (<c>exchange.write-failed</c>).</returns>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result Write(ModelNode model, Stream stream, GltfWriteOptions? options = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(model);
        ArgumentNullException.ThrowIfNull(stream);
        options ??= GltfWriteOptions.Default;
        return WriteCore(model, stream, null, null, options, cancellationToken);
    }

    /// <summary>
    /// Writes a model as glTF 2.0 JSON to <paramref name="json"/>, with its buffer in <paramref name="buffer"/>, which
    /// the JSON names <paramref name="bufferUri"/> (a <c>.gltf</c> file and its <c>.bin</c> beside it).
    /// <see cref="GltfWriteOptions.Binary"/> is ignored.
    /// </summary>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    public static Result Write(ModelNode model, Stream json, Stream buffer, string bufferUri, GltfWriteOptions? options = null,
                               CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(model);
        ArgumentNullException.ThrowIfNull(json);
        ArgumentNullException.ThrowIfNull(buffer);
        ArgumentNullException.ThrowIfNull(bufferUri);
        return WriteCore(model, json, buffer, bufferUri, (options ?? GltfWriteOptions.Default) with { Binary = false }, cancellationToken);
    }

    private static Result WriteCore(ModelNode model, Stream stream, Stream? buffer, string? bufferUri, GltfWriteOptions options,
                                CancellationToken cancellationToken)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var writer = new GltfTreeWriter(new GltfWriterOptions { InputLengthUnit = options.Unit.Millimetres() / 1000.0, Generator = options.Generator });
        var tree = ExchangeTree.ToTree(model, options.Mesh);
        var progress = new ProgressIndicator(cancellationToken);
        bool done;
        if (options.Binary)
        {
            done = writer.WriteGlb(tree, stream, progress.Start());
        }
        else if (buffer is not null)
        {
            done = writer.WriteGltf(tree, stream, buffer, bufferUri!, progress.Start());
        }
        else
        {
            using var json = new MemoryStream();
            using var bin = new MemoryStream();
            done = writer.WriteGltf(tree, json, bin, EmbeddedBuffer, progress.Start());
            if (done)
            {
                string text = Encoding.UTF8.GetString(json.ToArray());
                if (bin.Length > 0)
                {
                    text = text.Replace($"\"{EmbeddedBuffer}\"", "\"data:application/octet-stream;base64,"
                                        + Convert.ToBase64String(bin.GetBuffer(), 0, (int)bin.Length) + "\"", StringComparison.Ordinal);
                }

                stream.Write(Encoding.UTF8.GetBytes(text));
            }
        }

        cancellationToken.ThrowIfCancellationRequested();
        return done ? Result.Success() : Result.Failure(Diagnostic.Error(ExchangeTree.WriteFailed, "The model couldn't be written as glTF."));
    }
}

/// <summary>3MF (3MF Core): models written (Kerf's RW3mf tree writer).</summary>
public static class ThreeMf
{
    /// <summary>
    /// Writes a model as 3MF: a mesh object per part, a components object per assembly, a base material per colour, in
    /// <see cref="ThreeMfWriteOptions.Unit"/>. The parts' shapes are meshed with <see cref="ThreeMfWriteOptions.Mesh"/>.
    /// </summary>
    /// <returns>Success, or a failure (<c>exchange.write-failed</c>).</returns>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result Write(ModelNode model, Stream stream, ThreeMfWriteOptions? options = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(model);
        ArgumentNullException.ThrowIfNull(stream);
        options ??= ThreeMfWriteOptions.Default;
        cancellationToken.ThrowIfCancellationRequested();
        var writer = new TreeWriter(new WriteOptions { InputLengthUnit = options.Unit.Millimetres() / 1000.0 });
        bool done = writer.Write(ExchangeTree.ToTree(model, options.Mesh), stream, new ProgressIndicator(cancellationToken).Start());
        cancellationToken.ThrowIfCancellationRequested();
        return done ? Result.Success() : Result.Failure(Diagnostic.Error(ExchangeTree.WriteFailed, "The model couldn't be written as 3MF."));
    }
}

/// <summary>Wavefront OBJ: models written (Kerf's RWObj tree writer), with their materials.</summary>
public static class Obj
{
    /// <summary>
    /// Writes a model as OBJ to <paramref name="obj"/>, and its materials (one per colour) to <paramref name="materials"/>
    /// if given, which the OBJ names <paramref name="materialsName"/>. The parts' shapes are meshed with
    /// <see cref="ObjWriteOptions.Mesh"/>.
    /// </summary>
    /// <returns>Success, or a failure (<c>exchange.write-failed</c>).</returns>
    /// <exception cref="ArgumentNullException"><paramref name="model"/> or <paramref name="obj"/> is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result Write(ModelNode model, Stream obj, Stream? materials = null, string materialsName = "model.mtl",
                               ObjWriteOptions? options = null, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(model);
        ArgumentNullException.ThrowIfNull(obj);
        ArgumentNullException.ThrowIfNull(materialsName);
        options ??= ObjWriteOptions.Default;
        cancellationToken.ThrowIfCancellationRequested();
        var writer = new ObjTreeWriter(new ObjWriterOptions { Scale = options.Scale, ZUpToYUp = options.YUp, WriteNormals = options.Normals });
        bool done = writer.Write(ExchangeTree.ToTree(model, options.Mesh), obj, materials, materialsName,
                                 new ProgressIndicator(cancellationToken).Start());
        cancellationToken.ThrowIfCancellationRequested();
        return done ? Result.Success() : Result.Failure(Diagnostic.Error(ExchangeTree.WriteFailed, "The model couldn't be written as OBJ."));
    }
}
