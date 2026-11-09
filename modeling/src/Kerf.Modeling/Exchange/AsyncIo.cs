namespace Kerf.Modeling;

/// <summary>
/// The I/O of the importers' and exporters' async forms (unit 14G): a reader's bytes read asynchronously into memory
/// and then parsed, a writer's output built in memory and then written asynchronously. The parsing and building stay
/// synchronous, CPU work on the thread that runs them: only the I/O is asynchronous.
/// </summary>
internal static class AsyncIo
{
    /// <summary>The stream's bytes from its position, read asynchronously, or null past <paramref name="maxBytes"/>.</summary>
    internal static async Task<byte[]?> ReadAllAsync(Stream stream, long maxBytes, CancellationToken cancellationToken)
    {
        using var memory = new MemoryStream();
        var buffer = new byte[81920];
        long total = 0;
        int n;
        while ((n = await stream.ReadAsync(buffer, cancellationToken).ConfigureAwait(false)) > 0)
        {
            cancellationToken.ThrowIfCancellationRequested();
            total += n;
            if (total > maxBytes)
                return null;
            memory.Write(buffer, 0, n);
        }
        return memory.ToArray();
    }

    /// <summary>
    /// What <paramref name="read"/> makes of a file, opened for asynchronous reading before the task is returned (so a
    /// missing file throws <see cref="FileNotFoundException"/> as the synchronous forms do) and closed when it's done.
    /// </summary>
    internal static Task<T> ReadFileAsync<T>(string fileName, Func<Stream, Task<T>> read)
    {
        ArgumentNullException.ThrowIfNull(fileName);
        if (!File.Exists(fileName))
            throw new FileNotFoundException($"No such file: {fileName}", fileName);
        var file = new FileStream(fileName, FileMode.Open, FileAccess.Read, FileShare.Read, 4096, FileOptions.Asynchronous | FileOptions.SequentialScan);
        Task<T> reading;
        try
        {
            reading = read(file);
        }
        catch
        {
            file.Dispose();
            throw;
        }
        return CloseAfter(file, reading);

        static async Task<T> CloseAfter(FileStream file, Task<T> reading)
        {
            using (file)
                return await reading.ConfigureAwait(false);
        }
    }

    /// <summary>
    /// <paramref name="write"/> run into memory, its output then written to <paramref name="stream"/> asynchronously and
    /// the stream flushed, if it succeeded; a stream that can't be written is <c>exchange.write-failed</c>, as
    /// <c>Exporters3D</c>'s synchronous forms give it. Nothing is written on a failure.
    /// </summary>
    internal static async Task<Result> ToStreamAsync(Stream stream, Func<Stream, Result> write, CancellationToken cancellationToken)
    {
        cancellationToken = Effective(cancellationToken);
        cancellationToken.ThrowIfCancellationRequested();
        using var buffer = new MemoryStream();
        var written = write(buffer);
        if (written.IsFailure)
            return written;
        try
        {
            await stream.WriteAsync(buffer.GetBuffer().AsMemory(0, (int)buffer.Length), cancellationToken).ConfigureAwait(false);
            await stream.FlushAsync(cancellationToken).ConfigureAwait(false);
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException)
        {
            return Result.Failure(Diagnostic.Error(Exporters3D.WriteFailed, $"The data couldn't be written: {e.Message}"));
        }
        return written;
    }

    /// <summary>
    /// The folder of <paramref name="filePath"/>, checked before the task is returned: it must exist, as for
    /// <c>Exporters3D</c>'s synchronous file forms.
    /// </summary>
    internal static void CheckFolder(string filePath)
    {
        ArgumentNullException.ThrowIfNull(filePath);
        var folder = Path.GetDirectoryName(Path.GetFullPath(filePath));
        if (folder is not null && !Directory.Exists(folder))
            throw new DirectoryNotFoundException($"No such folder: {folder}");
    }

    /// <summary>
    /// <c>Exporters3D.ToFile</c>'s async form: the data made in memory and written to the file asynchronously only if
    /// it's whole, so a failure leaves no partial file. The folder is checked before the task is returned.
    /// </summary>
    internal static Task<Result> ToFileAsync(string filePath, Func<Stream, Result> write, CancellationToken cancellationToken)
    {
        CheckFolder(filePath);
        return WriteFile();

        async Task<Result> WriteFile()
        {
            var token = Effective(cancellationToken);
            token.ThrowIfCancellationRequested();
            using var buffer = new MemoryStream();
            var written = write(buffer);
            if (written.IsFailure)
                return written;
            var failed = await WriteFileAsync(filePath, buffer, token).ConfigureAwait(false);
            return failed ?? written;
        }
    }

    /// <summary>Writes a buffer's bytes to a file asynchronously: null, or the failure if it couldn't be written.</summary>
    internal static async Task<Result?> WriteFileAsync(string filePath, MemoryStream buffer, CancellationToken cancellationToken)
    {
        try
        {
            using var file = new FileStream(filePath, FileMode.Create, FileAccess.Write, FileShare.None, 4096, FileOptions.Asynchronous);
            await file.WriteAsync(buffer.GetBuffer().AsMemory(0, (int)buffer.Length), cancellationToken).ConfigureAwait(false);
            await file.FlushAsync(cancellationToken).ConfigureAwait(false);
        }
        catch (Exception e) when (e is IOException or UnauthorizedAccessException)
        {
            return Result.Failure(Diagnostic.Error(Exporters3D.WriteFailed, $"{filePath} couldn't be written: {e.Message}"));
        }
        return null;
    }

    /// <summary>The token an operation obeys: its own if it was given one, else the one in force (<see cref="Cancellation.Current"/>).</summary>
    internal static CancellationToken Effective(CancellationToken own) => own.CanBeCanceled ? own : Cancellation.Current;
}
