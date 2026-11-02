namespace Build123d.Occt;

// The stream forms of the STEP reader and writer, and the mesh formats, for the native binding. OCCT's stream forms
// (std::istream, std::ostream) aren't bound, so STEP goes through a temporary file; OCCT's mesh formats (TKDESTL,
// TKRWMesh, and glTF, which needs RapidJSON) aren't bound at all, so they need the OCCT.NET backend. Both backends
// have these members (src/Build123d.OcctNet/Runtime/Exchange.cs), so Build123d compiles against either.

public partial class STEPControl_Reader
{
    /// <summary>
    /// Reads STEP data from a stream. Natively, through a temporary file: only <see cref="ExchangeReadLimits.MaxBytes"/>
    /// applies, and cancellation is checked while copying and after OCCT reads.
    /// </summary>
    public IFSelect_ReturnStatus ReadStream(string name, Stream stream, ExchangeReadLimits limits, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        ArgumentNullException.ThrowIfNull(limits);
        var path = Path.Combine(Path.GetTempPath(), $"b3d-{Guid.NewGuid():N}.step");
        try
        {
            using (var file = new FileStream(path, FileMode.CreateNew, FileAccess.Write, FileShare.None))
                ExchangeIO.CopyWithin(stream, file, limits.MaxBytes, name, cancellationToken);
            var status = ReadFile(path);
            cancellationToken.ThrowIfCancellationRequested();
            return status;
        }
        finally
        {
            File.Delete(path);
        }
    }

    /// <summary>Transfers the roots read. Natively, cancellation is checked before and after.</summary>
    public int TransferRoots(CancellationToken cancellationToken)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var n = TransferRoots();
        cancellationToken.ThrowIfCancellationRequested();
        return n;
    }
}

public partial class STEPControl_Writer
{
    /// <summary>
    /// Translates a shape with writing parameters. Natively, only OCCT's defaults can be given, as OCCT's statics
    /// (<c>Interface_Static</c>) aren't bound, and cancellation is checked before and after.
    /// </summary>
    public IFSelect_ReturnStatus Transfer(TopoDS_Shape sh, STEPControl_StepModelType mode, StepWriteSettings settings,
                                          CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(settings);
        if (settings != StepWriteSettings.Default)
            throw new NotSupportedException("STEP writing parameters other than OCCT's defaults need the OCCT.NET backend: "
                                            + "the native binding doesn't bind Interface_Static");
        cancellationToken.ThrowIfCancellationRequested();
        var status = Transfer(sh, mode);
        cancellationToken.ThrowIfCancellationRequested();
        return status;
    }

    /// <summary>Writes the STEP model to a stream. Natively, through a temporary file.</summary>
    public IFSelect_ReturnStatus WriteStream(Stream stream)
    {
        ArgumentNullException.ThrowIfNull(stream);
        var path = Path.Combine(Path.GetTempPath(), $"b3d-{Guid.NewGuid():N}.step");
        try
        {
            var status = Write(path);
            if (status == IFSelect_ReturnStatus.IFSelect_RetDone)
            {
                using var file = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read);
                file.CopyTo(stream);
            }
            return status;
        }
        finally
        {
            File.Delete(path);
        }
    }
}

public static partial class Extras
{
    private static NotSupportedException MeshFormats() =>
        new("STL, glTF and 3MF need the OCCT.NET backend: the native binding doesn't bind OCCT's mesh formats");

    /// <summary>Writes a meshed shape as STL (OCCT's <c>StlAPI_Writer</c>). Not on the native backend.</summary>
    public static bool WriteStl(TopoDS_Shape meshed, Stream stream, bool ascii, CancellationToken cancellationToken = default)
        => throw MeshFormats();

    /// <summary>A face holding the triangulation read from STL data, its nodes scaled. Not on the native backend.</summary>
    public static TopoDS_Face ReadStl(Stream stream, double scale, ExchangeReadLimits limits, CancellationToken cancellationToken = default)
        => throw MeshFormats();

    /// <summary>
    /// Writes a meshed tree as glTF 2.0: binary (<c>.glb</c>) to <paramref name="stream"/>, or JSON with its buffer in
    /// <paramref name="buffer"/> as <paramref name="bufferUri"/>, or embedded if there's no buffer stream. Not on the
    /// native backend.
    /// </summary>
    public static bool WriteGltf(ExchangeNode root, Stream stream, Stream? buffer, string? bufferUri, bool binary,
                                 double unitInMetres, CancellationToken cancellationToken = default)
        => throw MeshFormats();

    /// <summary>Writes a meshed tree as 3MF. Not on the native backend.</summary>
    public static bool Write3mf(ExchangeNode root, Stream stream, double unitInMetres, CancellationToken cancellationToken = default)
        => throw MeshFormats();

    /// <summary>
    /// The tree read from glTF 2.0 data (<c>.glb</c>, or <c>.gltf</c> with its buffers embedded), in a unit of
    /// <paramref name="unitInMetres"/>. Not on the native backend.
    /// </summary>
    public static ExchangeNode ReadGltf(Stream stream, double unitInMetres, ExchangeReadLimits limits, CancellationToken cancellationToken = default)
        => throw MeshFormats();

    /// <summary>
    /// Whether STEP keeps names, colours and assemblies: not natively, as the native binding doesn't bind OCCT's XCAF
    /// documents or STEPCAFControl, so STEP is read and written as shapes alone (<see cref="STEPControl_Reader"/>,
    /// <see cref="STEPControl_Writer"/>).
    /// </summary>
    public static bool StepKeepsAssemblies => false;

    private static NotSupportedException StepAssemblies() =>
        new("STEP's names, colours and assemblies need the OCCT.NET backend: the native binding doesn't bind OCCT's XCAF documents");

    /// <summary>Reads STEP data into a tree with names, colours and assemblies. Not on the native backend.</summary>
    public static IFSelect_ReturnStatus ReadStep(string name, Stream stream, ExchangeReadLimits limits, bool externalFiles,
                                                 out ExchangeNode? root, CancellationToken cancellationToken = default)
        => throw StepAssemblies();

    /// <summary>Writes a tree as STEP with names, colours and assemblies. Not on the native backend.</summary>
    public static bool WriteStep(ExchangeNode root, Stream stream, StepWriteSettings settings, CancellationToken cancellationToken = default)
        => throw StepAssemblies();
}
