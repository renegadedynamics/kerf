// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Exchange.Step.Caf;
using Kerf.Engine.Exchange.Step.Construct;
using Kerf.Engine.Exchange.Step.Parser;
using Kerf.Engine.Foundation;
using Kerf.Topology;
using Units = Kerf.Engine.Exchange.Framework.UnitsMethods;

namespace Kerf.Exchange;

/// <summary>
/// STEP (ISO 10303-21), read and written through OCCT's STEPCAFControl over the assembly tree: parts, assemblies, names,
/// colours and placements (docs/api/exchange.md). Data that may be hostile is read within <see cref="ReadLimits"/>.
/// </summary>
public static class Step
{
    /// <summary>
    /// The model of STEP data: the free shapes as roots (one as it is, several in an assembly), assemblies with their
    /// components placed, names and colours. Data that refers to other STEP files is refused: there is no folder to read
    /// them from.
    /// </summary>
    /// <returns>The model, or a failure: <c>exchange.malformed</c>, <c>exchange.limit-exceeded</c>,
    /// <c>exchange.external-references</c>, the reader's status (<c>exchange.invalid-data</c>…), or
    /// <c>exchange.nothing-done</c> for data holding no shape.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="stream"/> is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result<ModelNode> Read(Stream stream, StepReadOptions? options = null, IProgress<double>? progress = null,
                                         CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        return ReadCore("STEP data", stream, options ?? StepReadOptions.Default, null, progress, cancellationToken);
    }

    /// <summary>
    /// The model of a STEP file (see the stream form). With <see cref="StepReadOptions.ExternalReferences"/>, the other
    /// STEP files it refers to are read from its folder, and only from inside it.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="path"/> is null.</exception>
    /// <exception cref="FileNotFoundException">There is no such file.</exception>
    public static Result<ModelNode> Read(string path, StepReadOptions? options = null, IProgress<double>? progress = null,
                                         CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(path);
        options ??= StepReadOptions.Default;
        string full = Path.GetFullPath(path);
        using var file = new FileStream(full, FileMode.Open, FileAccess.Read, FileShare.Read);
        return ReadCore(full, file, options, options.ExternalReferences ? Path.GetDirectoryName(full) : null, progress, cancellationToken);
    }

    /// <summary>
    /// Writes a model as STEP (AP214): a product per part and assembly, the parts of one shape, name and colour written
    /// once and placed by each of their instances. The shapes are written as they are, without OCCT's shape processing.
    /// </summary>
    /// <returns>Success, or a failure (<c>exchange.write-failed</c>) if the model couldn't be translated.</returns>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result Write(ModelNode model, Stream stream, StepWriteOptions? options = null, IProgress<double>? progress = null,
                               CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(model);
        ArgumentNullException.ThrowIfNull(stream);
        options ??= StepWriteOptions.Default;
        cancellationToken.ThrowIfCancellationRequested();

        // The shapes' unit is the file's (XCAF's length unit, in metres)
        double millimetres = options.Unit.Millimetres();
        var unit = Units.GetLengthUnitByFactorValue(millimetres);
        Tree tree = ExchangeTree.ToTree(model, mesh: null)
                                .WithLengthUnit(new DocumentLengthUnit(Units.DumpLengthUnit(unit), millimetres / 1000.0));
        var document = ExchangeDocument.FromTree(tree);
        var writer = new CafWriter();
        writer.SetColorMode(true);
        writer.SetNameMode(true);
        writer.SetLayerMode(true);
        var parameters = writer.Writer().Model().InternalParameters;
        parameters.InitFromStatic(writer.Writer().WS().Statics());
        parameters.WriteUnit = unit;
        parameters.WriteSurfaceCurMode = options.WritePcurves;
        parameters.WritePrecisionMode = (Parameters.WriteMode_PrecisionMode)(int)options.Precision;
        parameters.WriteSchema = options.Schema switch
        {
            StepSchema.Ap203 => Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP203,
            StepSchema.Ap242 => Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP242DIS,
            StepSchema.Ap214 => Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP214IS,
            _ => throw new ArgumentOutOfRangeException(nameof(options), options.Schema, "An unknown STEP schema."),
        };
        bool done;
        try
        {
            done = writer.Transfer(document, in parameters, ModelType.AsIs, null, new ProgressIndicator(cancellationToken, progress).Start());
            cancellationToken.ThrowIfCancellationRequested();
            done = done && writer.WriteStream(stream) == ReturnStatus.RetDone;
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            cancellationToken.ThrowIfCancellationRequested();
            return Result.Failure(Diagnostic.Error(ExchangeTree.WriteFailed, $"The model couldn't be written as STEP ({FailureDiagnostics.PublicText(e)})."));
        }

        return done ? Result.Success() : Result.Failure(Diagnostic.Error(ExchangeTree.WriteFailed, "The model couldn't be written as STEP."));
    }

    /// <summary>Writes a shape as STEP: one part (see the model form).</summary>
    /// <exception cref="ArgumentException">The shape is null.</exception>
    public static Result Write(Shape shape, Stream stream, StepWriteOptions? options = null, IProgress<double>? progress = null,
                               CancellationToken cancellationToken = default)
        => Write(ModelNode.Part(shape), stream, options, progress, cancellationToken);

    private static Result<ModelNode> ReadCore(string name, Stream stream, StepReadOptions options, string? folder, IProgress<double>? progress,
                                              CancellationToken cancellationToken)
    {
        cancellationToken.ThrowIfCancellationRequested();
        ReadLimits limits = options.Limits;
        var stepLimits = new ParserReadLimits { MaxFileSize = limits.MaxBytes, MaxRecords = limits.MaxEntities, MaxDepth = limits.MaxDepth };
        var reader = new CafReader();
        reader.SetColorMode(true);
        reader.SetNameMode(true);
        reader.SetLayerMode(true);
        // No external file is read from a stream unless given a resolver (decision 1285): these are read from the
        // file's folder, and refused outside it
        if (folder is not null)
        {
            reader.SetExternFileResolver(CafReader.FolderResolver(folder));
        }

        var indicator = new ProgressIndicator(cancellationToken, progress);
        try
        {
            var status = reader.ReadStream(name, stream, indicator.Start(), stepLimits);
            cancellationToken.ThrowIfCancellationRequested();
            if (ExchangeDiagnostics.ToDiagnostic(status) is { } error)
            {
                return Result<ModelNode>.Failure(error);
            }

            if (folder is null)
            {
                // OCCT reads a file's external references from the disk while transferring: never for data from a stream
                var references = new ExternRefs(reader.Reader().WS()!);
                if (references.LoadExternRefs() && references.ExternRefCount > 0)
                {
                    return Result<ModelNode>.Failure(Diagnostic.Error(ExchangeTree.ExternalReferences,
                        $"The STEP data refers to {references.ExternRefCount} other STEP file(s), which aren't read from a stream."));
                }
            }

            if (!options.Heal)
            {
                // No shape processing after translation: OCCT's flags set, and empty
                reader.Reader().SetShapeProcessFlags(new global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags());
            }

            var document = new ExchangeDocument();
            reader.Transfer(document, new ProgressIndicator(cancellationToken).Start());
            cancellationToken.ThrowIfCancellationRequested();
            return ExchangeTree.FromTree(document.ToTree(), options.ProductNames, meshes: false) is { } model
                ? model
                : Result<ModelNode>.Failure(ExchangeDiagnostics.ToDiagnostic(ReturnStatus.RetVoid)!);
        }
        catch (Exception e) when (e is not OperationCanceledException && ExchangeTree.IsBadData(e))
        {
            cancellationToken.ThrowIfCancellationRequested();
            return Result<ModelNode>.Failure(ExchangeTree.BadData(e, "STEP"));
        }
    }
}
