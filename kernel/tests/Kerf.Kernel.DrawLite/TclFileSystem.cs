// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// An open channel: a file being read, or written to the in-memory overlay, or stdout/stderr. The file's bytes are a
/// byte string (a character per byte, <see cref="TclEncoding"/>), read and written in the channel's encoding.
/// </summary>
internal sealed class TclChannel(string thePath, string? theRaw, StringBuilder? theWriter)
{
    public string Path { get; } = thePath;

    /// <summary>The channel's encoding (fconfigure -encoding), the system encoding at first.</summary>
    public string Encoding { get; private set; } = TclEncoding.System;

    /// <summary>The text being read (the file's bytes decoded), or null for a channel open only for writing.</summary>
    public string? Content { get; private set; } = theRaw is null ? null : TclEncoding.Decode(TclEncoding.System, theRaw);

    public int Position { get; set; }

    /// <summary>Where writes go (a byte string), or null for a read-only channel.</summary>
    public StringBuilder? Writer { get; } = theWriter;

    public bool AtEnd { get; set; }

    /// <summary>Sets the encoding; the text is decoded again.</summary>
    public void SetEncoding(string theName)
    {
        Encoding = TclEncoding.Check(theName);
        if (theRaw is not null)
        {
            Content = TclEncoding.Decode(Encoding, theRaw);
            Position = Math.Min(Position, Content.Length);
        }
    }

    /// <summary>Writes text in the channel's encoding.</summary>
    public void Write(string theText) => Writer?.Append(TclEncoding.Encode(Encoding, theText));
}

/// <summary>
/// Draw-lite's view of files: reads come from disk, but anything a script writes (open for writing, file mkdir,
/// file delete) goes to an in-memory overlay, so running a test never changes the disk. Paths use / and are
/// resolved against a current directory that cd changes. The overlay keeps a file's bytes as a byte string (a
/// character per byte); Tcl's channels encode and decode them (<see cref="TclEncoding"/>).
/// </summary>
internal sealed class TclFileSystem
{
    private readonly Dictionary<string, StringBuilder> myFiles = new(StringComparer.OrdinalIgnoreCase);
    private readonly HashSet<string> myDirectories = new(StringComparer.OrdinalIgnoreCase);
    private readonly HashSet<string> myDeleted = new(StringComparer.OrdinalIgnoreCase);
    private readonly Dictionary<string, TclChannel> myChannels = new(StringComparer.Ordinal);
    private int myNextChannel = 3;

    public string CurrentDirectory { get; set; } = Slashes(Environment.CurrentDirectory);

    public static string Slashes(string thePath) => thePath.Replace('\\', '/');

    /// <summary>A path made absolute and normalized, with / separators.</summary>
    public string Full(string thePath)
    {
        if (thePath.Length == 0)
        {
            return CurrentDirectory;
        }

        string aPath = thePath.StartsWith('~') ? Environment.GetFolderPath(Environment.SpecialFolder.UserProfile) + thePath[1..] : thePath;
        return Slashes(Path.GetFullPath(aPath, CurrentDirectory.Length > 0 ? CurrentDirectory : Environment.CurrentDirectory)).TrimEnd('/') is { Length: > 0 } aFull
                   ? (aFull.EndsWith(':') ? aFull + "/" : aFull)
                   : "/";
    }

    public bool IsFile(string thePath)
    {
        string aFull = Full(thePath);
        return myFiles.ContainsKey(aFull) || (!myDeleted.Contains(aFull) && File.Exists(aFull));
    }

    public bool IsDirectory(string thePath)
    {
        string aFull = Full(thePath);
        return myDirectories.Contains(aFull) || (!myDeleted.Contains(aFull) && Directory.Exists(aFull));
    }

    public bool Exists(string thePath) => IsFile(thePath) || IsDirectory(thePath);

    /// <summary>True for paths the overlay holds (written by the script).</summary>
    public bool IsVirtual(string thePath)
    {
        string aFull = Full(thePath);
        return myFiles.ContainsKey(aFull) || myDirectories.Contains(aFull);
    }

    /// <summary>A file's text, its bytes decoded in the system encoding (a UTF-8 byte order mark left out).</summary>
    public string ReadAll(string thePath)
    {
        string aRaw = ReadRaw(thePath);
        return TclEncoding.Decode(TclEncoding.System, aRaw.StartsWith("\u00EF\u00BB\u00BF", StringComparison.Ordinal) ? aRaw[3..] : aRaw);
    }

    /// <summary>A file's bytes as a byte string, from the overlay or from disk.</summary>
    public string ReadRaw(string thePath)
    {
        string aFull = Full(thePath);
        if (myFiles.TryGetValue(aFull, out StringBuilder? aText))
        {
            return aText.ToString();
        }

        if (myDeleted.Contains(aFull) || !File.Exists(aFull))
        {
            throw new DrawException($"couldn't read file \"{thePath}\": no such file or directory");
        }

        return TclEncoding.ByteString(File.ReadAllBytes(aFull));
    }

    /// <summary>
    /// A file's bytes, for Draw's file commands (restore, readbrep): from the overlay (whose text holds a byte per
    /// character, as <see cref="WriteBytes"/> stores it) or from disk. False if there is no such file.
    /// </summary>
    public bool TryReadBytes(string thePath, out byte[] theBytes)
    {
        string aFull = Full(thePath);
        if (myFiles.TryGetValue(aFull, out StringBuilder? aText))
        {
            theBytes = Encoding.Latin1.GetBytes(aText.ToString());
            return true;
        }

        theBytes = [];
        if (myDeleted.Contains(aFull) || !File.Exists(aFull))
        {
            return false;
        }

        try
        {
            theBytes = File.ReadAllBytes(aFull);
            return true;
        }
        catch (Exception anException) when (anException is IOException or UnauthorizedAccessException)
        {
            return false;
        }
    }

    /// <summary>Writes a file written by a Draw command (bsave, writebrep, binsave) to the overlay, a character per byte.</summary>
    public void WriteBytes(string thePath, byte[] theBytes)
    {
        string aFull = Full(thePath);
        myFiles[aFull] = new StringBuilder(Encoding.Latin1.GetString(theBytes));
        myDeleted.Remove(aFull);
    }

    public long Size(string thePath)
    {
        string aFull = Full(thePath);
        return myFiles.TryGetValue(aFull, out StringBuilder? aText) ? aText.Length
             : IsFile(aFull) ? new FileInfo(aFull).Length
             : throw new DrawException($"could not read \"{thePath}\": no such file or directory");
    }

    public void MakeDirectory(string thePath)
    {
        string aFull = Full(thePath);
        while (aFull.Length > 0 && !IsDirectory(aFull))
        {
            myDirectories.Add(aFull);
            myDeleted.Remove(aFull);
            int aSlash = aFull.LastIndexOf('/');
            if (aSlash <= 0)
            {
                break;
            }

            aFull = aFull[..aSlash];
        }
    }

    public void Delete(string thePath)
    {
        string aFull = Full(thePath);
        myFiles.Remove(aFull);
        myDirectories.Remove(aFull);
        myDeleted.Add(aFull);
    }

    public void Copy(string theSource, string theTarget)
    {
        string aTarget = IsDirectory(theTarget) ? Full(theTarget) + "/" + Path.GetFileName(Full(theSource)) : Full(theTarget);
        myFiles[aTarget] = new StringBuilder(ReadRaw(theSource));
        myDeleted.Remove(aTarget);
    }

    /// <summary>The names in a directory (on disk and in the overlay).</summary>
    public IEnumerable<(string Name, bool IsDirectory)> List(string theDirectory)
    {
        string aFull = Full(theDirectory);
        var aSeen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        if (Directory.Exists(aFull) && !myDeleted.Contains(aFull))
        {
            // the entries' kinds as the enumeration read them, not a query of each (OCCT's locate_data_file globs the data
            // directories, thousands of files, for each file it looks for: decision 2084)
            foreach (FileSystemInfo anEntry in new DirectoryInfo(aFull).EnumerateFileSystemInfos())
            {
                string aPath = Slashes(Path.Join(aFull, anEntry.Name));
                if (!myDeleted.Contains(aPath) && aSeen.Add(aPath))
                {
                    yield return (Path.GetFileName(aPath), anEntry is DirectoryInfo);
                }
            }
        }

        foreach (string aPath in myFiles.Keys.Concat(myDirectories))
        {
            int aSlash = aPath.LastIndexOf('/');
            if (aSlash > 0 && string.Equals(aPath[..aSlash], aFull, StringComparison.OrdinalIgnoreCase) && aSeen.Add(aPath))
            {
                yield return (aPath[(aSlash + 1)..], myDirectories.Contains(aPath));
            }
        }
    }

    // -------------------------------------------------------------------------------------------------------------
    // Channels

    public string Open(string thePath, string theMode)
    {
        string aFull = Full(thePath);
        bool aWrite = theMode.Contains('w') || theMode.Contains('a') || theMode.Contains('+')
                    || theMode.Contains("WRONLY", StringComparison.Ordinal) || theMode.Contains("RDWR", StringComparison.Ordinal);
        TclChannel aChannel;
        if (!aWrite)
        {
            if (!IsFile(aFull))
            {
                throw new DrawException($"couldn't open \"{thePath}\": no such file or directory") { ErrorCode = "POSIX ENOENT {no such file or directory}" };
            }

            aChannel = new TclChannel(aFull, ReadRaw(aFull), null);
        }
        else
        {
            bool anAppend = theMode.Contains('a') || theMode.Contains("APPEND", StringComparison.Ordinal);
            bool aTruncate = theMode.Contains('w') || theMode.Contains("TRUNC", StringComparison.Ordinal);
            string anExisting = !aTruncate && IsFile(aFull) ? ReadRaw(aFull) : "";
            var aWriter = new StringBuilder(anExisting);
            myFiles[aFull] = aWriter;
            myDeleted.Remove(aFull);
            aChannel = new TclChannel(aFull, anAppend ? null : anExisting, aWriter);
        }

        string anId = $"file{myNextChannel++}";
        myChannels[anId] = aChannel;
        return anId;
    }

    public TclChannel Channel(string theId)
        => myChannels.TryGetValue(theId, out TclChannel? aChannel) ? aChannel : throw new DrawException($"can not find channel named \"{theId}\"");

    public void Close(string theId)
    {
        if (!myChannels.Remove(theId))
        {
            throw new DrawException($"can not find channel named \"{theId}\"");
        }
    }
}
