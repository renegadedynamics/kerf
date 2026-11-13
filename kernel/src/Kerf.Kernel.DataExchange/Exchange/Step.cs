// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Exchange.Step.Caf;
using Kerf.Engine.Exchange.Step.Construct;
using Kerf.Engine.Exchange.Step.Data;
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
    /// The warning of a read that left entities of the data out, and what refers to them with them:
    /// <c>exchange.entity-skipped</c> (decision 2308).
    /// </summary>
    public static DiagnosticCode EntitySkipped => ExchangeTree.EntitySkipped;

    /// <summary>
    /// The warning of a read whose model holds faces that are invalid: <c>exchange.invalid-face</c> (decision 2314). The
    /// read checks the faces the healing reported on and those with a vertex off its edge's curve, not every face; a
    /// caller who won't take such a model refuses a result with this warning.
    /// </summary>
    public static DiagnosticCode InvalidFace => ExchangeTree.InvalidFace;

    /// <summary>
    /// The model of STEP data: the free shapes as roots (one as it is, several in an assembly), assemblies with their
    /// components placed, names and colours. Data that refers to other STEP files is refused: there is no folder to read
    /// them from.
    /// </summary>
    /// <returns>The model, or a failure: <c>exchange.malformed</c>, <c>exchange.limit-exceeded</c>,
    /// <c>exchange.external-references</c>, the reader's status (<c>exchange.invalid-data</c>…), or
    /// <c>exchange.nothing-done</c> for data holding no shape. A model may come with warnings: <see cref="EntitySkipped"/>
    /// for entities left out, <see cref="InvalidFace"/> for faces the read's check found invalid.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="stream"/> is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result<ModelNode> Read(Stream stream, StepReadOptions? options = null, IProgress<double>? progress = null,
                                         CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(stream);
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
        KernelArgumentNullException.ThrowIfNull(path);
        options ??= StepReadOptions.Default;
        string full = Path.GetFullPath(path);
        using var file = new FileStream(full, FileMode.Open, FileAccess.Read, FileShare.Read);
        return ReadCore(full, file, options, options.ExternalReferences ? Path.GetDirectoryName(full) : null, progress, cancellationToken);
    }

    /// <summary>
    /// <see cref="Read(Stream, StepReadOptions?, IProgress{double}?, CancellationToken)"/> with its input read
    /// asynchronously: the stream's bytes are read into memory without blocking a thread (up to
    /// <see cref="ReadLimits.MaxBytes"/> and one more, so data past the limit is refused as the synchronous form refuses
    /// it), then parsed and translated synchronously, on the thread the read finished on. The same result as the
    /// synchronous form, which parses as it reads: this one holds the file's bytes in memory while it parses.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="stream"/> is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled (the task is cancelled).</exception>
    public static Task<Result<ModelNode>> ReadAsync(Stream stream, StepReadOptions? options = null, IProgress<double>? progress = null,
                                                    CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(stream);
        options ??= StepReadOptions.Default;
        return MemorySpool.ReadThenAsync(stream, options.Limits.MaxBytes,
                                         data => ReadCore("STEP data", data, options, null, progress, cancellationToken), cancellationToken);
    }

    /// <summary>
    /// <see cref="Read(string, StepReadOptions?, IProgress{double}?, CancellationToken)"/> with the file read
    /// asynchronously, as the stream form reads its stream. The file is opened before the task is returned, so a
    /// missing one throws as it does in the synchronous form. The other STEP files it refers to, with
    /// <see cref="StepReadOptions.ExternalReferences"/>, are read synchronously while it is translated.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="path"/> is null.</exception>
    /// <exception cref="FileNotFoundException">There is no such file.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled (the task is cancelled).</exception>
    public static Task<Result<ModelNode>> ReadAsync(string path, StepReadOptions? options = null, IProgress<double>? progress = null,
                                                    CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(path);
        options ??= StepReadOptions.Default;
        string full = Path.GetFullPath(path);
        var file = new FileStream(full, FileMode.Open, FileAccess.Read, FileShare.Read, 4096, FileOptions.Asynchronous | FileOptions.SequentialScan);
        return ReadFile(file, full, options, progress, cancellationToken);

        static async Task<Result<ModelNode>> ReadFile(FileStream file, string full, StepReadOptions options, IProgress<double>? progress,
                                                     CancellationToken cancellationToken)
        {
            MemorySpool data;
            using (file)
            {
                data = await MemorySpool.ReadFromAsync(file, options.Limits.MaxBytes, Cancellation.Effective(cancellationToken)).ConfigureAwait(false);
            }

            using (data)
            {
                return ReadCore(full, data, options, options.ExternalReferences ? Path.GetDirectoryName(full) : null, progress, cancellationToken);
            }
        }
    }

    /// <summary>
    /// <see cref="Write(ModelNode, Stream, StepWriteOptions?, IProgress{double}?, CancellationToken)"/> with its output
    /// written asynchronously: the STEP is built in memory, synchronously, on the caller's thread, then written to the
    /// stream without blocking a thread, and the stream flushed. The same bytes as the synchronous form, but for the
    /// time of writing in the header; nothing is written on a failure.
    /// </summary>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled (the task is cancelled).</exception>
    public static Task<Result> WriteAsync(ModelNode model, Stream stream, StepWriteOptions? options = null, IProgress<double>? progress = null,
                                          CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(model);
        KernelArgumentNullException.ThrowIfNull(stream);
        return MemorySpool.WriteThenAsync(stream, data => Write(model, data, options, progress, cancellationToken), cancellationToken);
    }

    /// <summary>Writes a shape as STEP asynchronously: one part (see the model form).</summary>
    /// <exception cref="ArgumentException">The shape is null.</exception>
    public static Task<Result> WriteAsync(Shape shape, Stream stream, StepWriteOptions? options = null, IProgress<double>? progress = null,
                                          CancellationToken cancellationToken = default)
        => WriteAsync(ModelNode.Part(shape), stream, options, progress, cancellationToken);

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
        KernelArgumentNullException.ThrowIfNull(model);
        KernelArgumentNullException.ThrowIfNull(stream);
        options ??= StepWriteOptions.Default;
        cancellationToken = Cancellation.Effective(cancellationToken);
        using Cancellation.Scope cancellation = Cancellation.Enter(cancellationToken);
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
        writer.SetCleanDuplicates(options.CleanDuplicates);
        var parameters = writer.Writer().Model().InternalParameters;
        parameters.InitFromStatic(writer.Writer().WS().Statics());
        parameters.WriteUnit = unit;
        parameters.WriteSurfaceCurMode = options.WritePcurves;
        parameters.WritePrecisionMode = (Parameters.WriteMode_PrecisionMode)(int)options.Precision;
        parameters.CleanDuplicates = options.CleanDuplicates;
        parameters.WriteSchema = options.Schema switch
        {
            StepSchema.Ap203 => Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP203,
            StepSchema.Ap242 => Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP242DIS,
            StepSchema.Ap214 => Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP214IS,
            _ => throw new KernelArgumentOutOfRangeException(nameof(options), options.Schema, "An unknown STEP schema."),
        };
        bool done;
        try
        {
            done = writer.Transfer(document, in parameters, ModelType.AsIs, null, new ProgressIndicator(cancellationToken, progress).Start());
            cancellationToken.ThrowIfCancellationRequested();
            using OperationContext.Scope texts = OperationContext.Begin(StepWriter.TextsAsDirectives, true);
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

    /// <summary>
    /// The warning for entities the reader couldn't load or translate, so that what refers to them is missing from the
    /// model (decision 2308): those whose data failed its checks as the file was loaded, a reference to an entity that
    /// isn't in the file among them, and those whose translation recorded a failure. Where nothing is left of the model
    /// (no vertex in it), the warning says so (decision 2315). Null where there are none.
    /// </summary>
    private static Diagnostic? SkippedEntities(CafReader reader, ModelNode read)
    {
        if (reader.Reader().WS() is not { } session || session.Model() is not StepModel model)
        {
            return null;
        }

        var ids = new SortedSet<int>();
        string? reason = null;
        void Note(Kerf.Engine.Foundation.Transient? entity, Kerf.Engine.Exchange.Framework.FrameworkCheck? check)
        {
            if (entity is null || !ids.Add(model.IdentLabel(entity)))
            {
                return;
            }
            if (reason is null && check is { FailCount: > 0 })
            {
                reason = check.CFail(1);
            }
        }

        for (int number = 1; number <= model.EntityCount; number++)
        {
            if (model.IsErrorEntity(number))
            {
                Note(model.Value(number), model.ReportEntity(number)?.Check());
            }
        }
        if (session.TransferReader()?.TransientProcess() is { } process)
        {
            for (int item = 1; item <= process.MappedCount; item++)
            {
                if (process.MapItem(item) is { } binder && binder.Check() is { HasFailed: true } check)
                {
                    Note(process.Mapped(item), check);
                }
            }
        }
        if (ids.Count == 0)
        {
            return null;
        }

        string listed = string.Join(", ", ids.Take(8).Select(id => FormattableString.Invariant($"#{id}")));
        string more = ids.Count > 8 ? FormattableString.Invariant($" and {ids.Count - 8} more") : "";
        string why = string.IsNullOrWhiteSpace(reason) ? "" : $" The first says: {reason.Trim()}.";
        string empty = new Kerf.Engine.Topology.Explorer(read.ToShape(), Kerf.Topology.ShapeType.Vertex).More()
            ? ""
            : " Nothing else was read: the model is empty.";
        return Diagnostic.Warning(ExchangeTree.EntitySkipped,
            FormattableString.Invariant($"{ids.Count} entities of the STEP data couldn't be read, and what refers to them is missing from the model: {listed}{more}.") + why + empty);
    }

    /// <summary>
    /// The warning for faces the read's check found invalid after healing (decision 2314): how many, and the STEP
    /// identifiers of the faces they were read from, the first eight. Null where there are none.
    /// </summary>
    private static Diagnostic? InvalidFaces(CafReader reader)
    {
        if (reader.Reader().WS() is not { } session || session.TransferReader()?.TransientProcess() is not { } process
            || process.InvalidFaces.Count == 0)
        {
            return null;
        }

        var ids = new SortedSet<int>();
        if (session.Model() is StepModel model)
        {
            foreach ((_, var entity) in process.InvalidFaces)
            {
                if (entity is not null && model.IdentLabel(entity) is > 0 and var id)
                {
                    ids.Add(id);
                }
            }
        }

        int count = process.InvalidFaces.Count;
        string faces = count == 1 ? "1 face" : FormattableString.Invariant($"{count} faces");
        string which = ids.Count == 0
            ? ""
            : FormattableString.Invariant($" (from {string.Join(", ", ids.Take(8).Select(id => FormattableString.Invariant($"#{id}")))}")
              + (ids.Count > 8 ? FormattableString.Invariant($" and {ids.Count - 8} more") : "") + ")";
        string them = count == 1 ? "it" : "them";
        return Diagnostic.Warning(ExchangeTree.InvalidFace,
            $"{faces} of the STEP data {(count == 1 ? "is" : "are")} invalid as read{which}: the healing reported on {them}, a vertex lies off its edge's curve or {(count == 1 ? "it is" : "they are")} bounded by two edges or fewer, and the check of such faces found {them} wrong. The model holds {them} as read.");
    }

    private static Result<ModelNode> ReadCore(string name, Stream stream, StepReadOptions options, string? folder, IProgress<double>? progress,
                                              CancellationToken cancellationToken)
    {
        cancellationToken = Cancellation.Effective(cancellationToken);
        using Cancellation.Scope cancellation = Cancellation.Enter(cancellationToken);
        cancellationToken.ThrowIfCancellationRequested();
        ReadLimits limits = options.Limits;
        var report = new ParserLimitReport();
        var stepLimits = new ParserReadLimits { MaxFileSize = limits.MaxBytes, MaxRecords = limits.MaxEntities, MaxDepth = limits.MaxDepth, Report = report };
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
            // Kerf: the parser's limits refuse the data as such, not as a failure to read it (decision 1965)
            if (status != ReturnStatus.RetDone && report.Exceeded is { } limit)
            {
                return Result<ModelNode>.Failure(Diagnostic.Error(ExchangeTree.LimitExceeded, $"The STEP data is larger than the limits: its {limit} is above the limit."));
            }
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
            if (ExchangeTree.FromTree(document.ToTree(), options.ProductNames, meshes: false) is not { } model)
            {
                return Result<ModelNode>.Failure(ExchangeDiagnostics.ToDiagnostic(ReturnStatus.RetVoid)!);
            }
            ImmutableArray<Diagnostic> warnings = [.. new[] { SkippedEntities(reader, model), InvalidFaces(reader) }.OfType<Diagnostic>()];
            return warnings.Length == 0 ? model : Result<ModelNode>.Success(model, warnings);
        }
        catch (Exception e) when (e is not OperationCanceledException && ExchangeTree.IsBadData(e))
        {
            cancellationToken.ThrowIfCancellationRequested();
            return Result<ModelNode>.Failure(ExchangeTree.BadData(e, "STEP"));
        }
    }
}
