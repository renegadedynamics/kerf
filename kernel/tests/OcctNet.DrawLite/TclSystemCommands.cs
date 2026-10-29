// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;
using System.Text;

namespace OcctNet.DrawLite;

/// <summary>Tcl's introspection (info), I/O (puts, open, gets, source), file names (file, glob) and clock.</summary>
internal static class TclSystemCommands
{
    private static readonly CultureInfo Invariant = CultureInfo.InvariantCulture;
    private static readonly Stopwatch s_clicks = Stopwatch.StartNew();

    public static void Register(DrawInterpreter theInterpreter)
    {
        theInterpreter.Register("info", Info);
        theInterpreter.Register("puts", Puts);
        theInterpreter.Register("open", Open);
        theInterpreter.Register("close", static (i, a) =>
        {
            if (a.Count < 2)
            {
                throw Usage("close channelId");
            }

            i.Files.Close(a[1]);
            return "";
        });
        theInterpreter.Register("gets", Gets);
        theInterpreter.Register("read", Read);
        theInterpreter.Register("eof", static (i, a) => a.Count == 2 ? (IsStd(a[1]) || i.Files.Channel(a[1]).AtEnd ? "1" : "0") : throw Usage("eof channelId"));
        theInterpreter.Register("flush", static (i, a) => a.Count == 2 ? Flush(i, a[1]) : throw Usage("flush channelId"));
        theInterpreter.Register("fconfigure", static (_, _) => "");
        theInterpreter.Register("chan", Chan);
        theInterpreter.Register("source", Source);
        theInterpreter.Register("file", File);
        theInterpreter.Register("glob", Glob);
        theInterpreter.Register("cd", static (i, a) =>
        {
            string aTarget = a.Count > 1 ? a[1] : Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
            if (!i.Files.IsDirectory(aTarget))
            {
                throw new DrawException($"couldn't change working directory to \"{aTarget}\": no such file or directory");
            }

            i.Files.CurrentDirectory = i.Files.Full(aTarget);
            return "";
        });
        theInterpreter.Register("pwd", static (i, _) => i.Files.CurrentDirectory);
        theInterpreter.Register("clock", Clock);
        theInterpreter.Register("after", static (i, a) => a.Count > 2 && a[1] != "cancel" && a[1] != "info" ? i.EvalScript(TclControlCommands.Concat(a, 2)) : "");
        theInterpreter.Register("update", static (_, _) => "");
        theInterpreter.Register("pid", static (_, _) => Environment.ProcessId.ToString(Invariant));
        theInterpreter.Register("encoding", static (_, a) => a.Count switch
        {
            2 when a[1] == "system" => "utf-8",
            2 when a[1] == "names" => "utf-8 iso8859-1 ascii unicode",
            >= 4 when a[1] is "convertto" or "convertfrom" => a[^1],
            _ => "",
        });
        theInterpreter.Register("package", static (_, a) => a.Count >= 3 && a[1] == "require"
                                                               ? (a[^1] is "Tcl" or "Tclx" ? "8.6" : throw new DrawException($"can't find package {a[^1]}"))
                                                               : "");
    }

    private static DrawException Usage(string theUsage) => TclControlCommands.Usage(theUsage);

    private static bool IsStd(string theChannel) => theChannel is "stdout" or "stderr" or "stdin";

    private static string Flush(DrawInterpreter theInterpreter, string theChannel)
    {
        if (IsStd(theChannel))
        {
            theInterpreter.Output.Flush();
        }
        else
        {
            theInterpreter.Files.Channel(theChannel);
        }

        return "";
    }

    // -------------------------------------------------------------------------------------------------------------
    // info

    private static string Info(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            throw Usage("info subcommand ?arg ...?");
        }

        string? aPattern = a.Count > 2 ? a[2] : null;
        switch (TclControlCommands.Resolve(a[1], "subcommand", ["exists", "commands", "procs", "body", "args", "default", "level", "script", "globals", "vars", "locals", "tclversion", "patchlevel", "hostname", "nameofexecutable", "complete", "cmdcount", "library", "sharedlibextension", "frame", "functions", "loaded", "coroutine", "errorstack", "class", "object"]))
        {
            case "exists" when a.Count == 3:
                return theInterpreter.TryGetVariable(a[2], out _) ? "1" : "0";
            case "commands":
                return Names(theInterpreter.CommandNames, aPattern);
            case "procs":
                return Names(theInterpreter.ProcNames, aPattern);
            case "body" when a.Count == 3:
                return Proc(theInterpreter, a[2]).Body;
            case "args" when a.Count == 3:
                return TclList.Make(Proc(theInterpreter, a[2]).Parameters.Select(p => p.Name).ToArray());
            case "default" when a.Count == 5:
            {
                TclProc aProc = Proc(theInterpreter, a[2]);
                foreach ((string aName, string? aDefault) in aProc.Parameters)
                {
                    if (aName == a[3])
                    {
                        theInterpreter.SetVariable(a[4], aDefault ?? "");
                        return aDefault is null ? "0" : "1";
                    }
                }

                throw new DrawException($"procedure \"{a[2]}\" doesn't have an argument \"{a[3]}\"");
            }

            case "level":
            {
                if (a.Count == 2)
                {
                    return theInterpreter.Level.ToString(Invariant);
                }

                long aLevel = TclNumber.GetInt(a[2]);
                string aFrameLevel = aLevel > 0 ? $"#{aLevel}" : (-aLevel).ToString(Invariant);
                TclFrame aFrame = theInterpreter.FrameAt(aFrameLevel);
                return aFrame.Command is null ? throw new DrawException($"bad level \"{a[2]}\"") : TclList.Format(aFrame.Command);
            }

            case "script":
                if (a.Count == 3)
                {
                    theInterpreter.ScriptFile = a[2];
                }

                return theInterpreter.ScriptFile;
            case "globals":
                return VariableNames(theInterpreter.GlobalFrame, aPattern);
            case "vars" or "locals":
                return VariableNames(theInterpreter.CurrentFrame, aPattern);
            case "tclversion":
                return "8.6";
            case "patchlevel":
                return "8.6.13";
            case "hostname":
                return Environment.MachineName;
            case "nameofexecutable":
                return TclFileSystem.Slashes(Environment.ProcessPath ?? "DRAWEXE");
            case "complete" when a.Count == 3:
                return TclParser.IsComplete(a[2]) ? "1" : "0";
            case "cmdcount":
                return "0";
            case "library":
                return "";
            case "sharedlibextension":
                return OperatingSystem.IsWindows() ? ".dll" : ".so";
            case "frame":
                return theInterpreter.Level.ToString(Invariant);
            default:
                throw new DrawException($"info {a[1]}: unknown or ambiguous subcommand, or wrong # args");
        }
    }

    private static TclProc Proc(DrawInterpreter theInterpreter, string theName)
        => theInterpreter.TryGetProc(theName, out TclProc aProc) ? aProc : throw new DrawException($"\"{theName}\" isn't a procedure");

    private static string Names(IEnumerable<string> theNames, string? thePattern)
    {
        string? aPattern = thePattern?.StartsWith("::", StringComparison.Ordinal) == true ? thePattern.TrimStart(':') : thePattern;
        return TclList.Make(theNames.Where(n => aPattern is null || TclGlob.Match(aPattern, n)).ToArray());
    }

    private static string VariableNames(TclFrame theFrame, string? thePattern)
        => TclList.Make(theFrame.Vars.Where(v => v.Value.Exists && (thePattern is null || TclGlob.Match(thePattern, v.Key))).Select(v => v.Key).ToArray());

    // -------------------------------------------------------------------------------------------------------------
    // Channels

    private static string Puts(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        int i = 1;
        bool aNoNewline = false;
        if (i < a.Count && a[i] == "-nonewline")
        {
            aNoNewline = true;
            i++;
        }

        string aChannel;
        string aText;
        switch (a.Count - i)
        {
            case 1:
                aChannel = "stdout";
                aText = a[i];
                break;
            case 2:
                aChannel = a[i];
                aText = a[i + 1];
                break;
            case 3 when a[i + 2] == "nonewline":
                aChannel = a[i];
                aText = a[i + 1];
                aNoNewline = true;
                break;
            default:
                throw Usage("puts ?-nonewline? ?channelId? string");
        }

        if (aChannel is "stdout" or "stderr")
        {
            if (aNoNewline)
            {
                theInterpreter.Output.Write(aText);
            }
            else
            {
                theInterpreter.Output.WriteLine(aText);
            }

            return "";
        }

        StringBuilder aWriter = theInterpreter.Files.Channel(aChannel).Writer
                                ?? throw new DrawException($"channel \"{aChannel}\" wasn't opened for writing");
        aWriter.Append(aText);
        if (!aNoNewline)
        {
            aWriter.Append('\n');
        }

        return "";
    }

    private static string Open(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count is < 2 or > 4)
        {
            throw Usage("open fileName ?access? ?permissions?");
        }

        if (a[1].StartsWith('|'))
        {
            throw new DrawException("open: command pipelines are not supported by Draw-lite");
        }

        return theInterpreter.Files.Open(a[1], a.Count > 2 ? a[2] : "r");
    }

    private static string Gets(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count is < 2 or > 3)
        {
            throw Usage("gets channelId ?varName?");
        }

        string? aLine = null;
        if (!IsStd(a[1]))
        {
            TclChannel aChannel = theInterpreter.Files.Channel(a[1]);
            string aContent = aChannel.Content ?? throw new DrawException($"channel \"{a[1]}\" wasn't opened for reading");
            if (aChannel.Position < aContent.Length)
            {
                int anEnd = aContent.IndexOf('\n', aChannel.Position);
                if (anEnd < 0)
                {
                    aLine = aContent[aChannel.Position..];
                    aChannel.Position = aContent.Length;
                    aChannel.AtEnd = true;
                }
                else
                {
                    aLine = aContent[aChannel.Position..anEnd].TrimEnd('\r');
                    aChannel.Position = anEnd + 1;
                }
            }
            else
            {
                aChannel.AtEnd = true;
            }
        }

        if (a.Count == 3)
        {
            theInterpreter.SetVariable(a[2], aLine ?? "");
            return (aLine?.Length ?? -1).ToString(Invariant);
        }

        return aLine ?? "";
    }

    private static string Read(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        bool aNoNewline = a.Count > 1 && a[1] == "-nonewline";
        int i = aNoNewline ? 2 : 1;
        if (i >= a.Count)
        {
            throw Usage("read ?-nonewline? channelId");
        }

        if (IsStd(a[i]))
        {
            return "";
        }

        TclChannel aChannel = theInterpreter.Files.Channel(a[i]);
        string aContent = aChannel.Content ?? throw new DrawException($"channel \"{a[i]}\" wasn't opened for reading");
        int aCount = i + 1 < a.Count ? (int)Math.Min(TclNumber.GetInt(a[i + 1]), aContent.Length - aChannel.Position) : aContent.Length - aChannel.Position;
        string aText = aContent.Substring(aChannel.Position, aCount).Replace("\r\n", "\n");
        aChannel.Position += aCount;
        aChannel.AtEnd = aChannel.Position >= aContent.Length;
        return aNoNewline && aText.EndsWith('\n') ? aText[..^1] : aText;
    }

    private static string Chan(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            throw Usage("chan subcommand ?arg ...?");
        }

        var aRest = new List<string> { a[1] };
        aRest.AddRange(a.Skip(2));
        return a[1] switch
        {
            "puts" => Puts(theInterpreter, aRest),
            "gets" => Gets(theInterpreter, aRest),
            "read" => Read(theInterpreter, aRest),
            "close" => theInterpreter.Invoke(["close", .. a.Skip(2)]),
            "flush" => theInterpreter.Invoke(["flush", .. a.Skip(2)]),
            "eof" => theInterpreter.Invoke(["eof", .. a.Skip(2)]),
            "configure" => "",
            _ => throw new DrawException($"chan {a[1]}: not supported by Draw-lite"),
        };
    }

    private static string Source(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        int i = 1;
        if (i + 1 < a.Count && a[i] == "-encoding")
        {
            i += 2;
        }

        if (i != a.Count - 1)
        {
            throw Usage("source ?-encoding name? fileName");
        }

        string aPath = a[i];
        string aScript;
        try
        {
            aScript = theInterpreter.Files.ReadAll(aPath);
        }
        catch (DrawException)
        {
            throw new DrawException($"couldn't read file \"{aPath}\": no such file or directory");
        }

        string aSaved = theInterpreter.ScriptFile;
        theInterpreter.ScriptFile = aPath;
        try
        {
            return theInterpreter.EvalScript(aScript);
        }
        catch (TclReturnException aReturn) when (aReturn.Code == 0)
        {
            return aReturn.Value;
        }
        catch (DrawException anError) when (DrawInterpreter.TagFile(anError, aPath))
        {
            throw;
        }
        finally
        {
            theInterpreter.ScriptFile = aSaved;
        }
    }

    // -------------------------------------------------------------------------------------------------------------
    // file and glob: path operations, queries of the disk, and writes to the in-memory overlay

    private static bool IsAbsolute(string thePath)
        => thePath.StartsWith('/') || thePath.StartsWith('~') || (thePath.Length >= 2 && char.IsAsciiLetter(thePath[0]) && thePath[1] == ':');

    internal static string Join(IEnumerable<string> theParts)
    {
        string aResult = "";
        foreach (string aRaw in theParts)
        {
            string aPart = TclFileSystem.Slashes(aRaw);
            if (aPart.Length == 0)
            {
                continue;
            }

            if (IsAbsolute(aPart) || aResult.Length == 0)
            {
                aResult = aPart;
            }
            else
            {
                aResult = aResult.EndsWith('/') ? aResult + aPart : aResult + "/" + aPart;
            }
        }

        // Tcl's file join drops trailing and doubled separators
        while (aResult.Contains("//", StringComparison.Ordinal))
        {
            aResult = aResult.Replace("//", "/");
        }

        return aResult.Length > 1 && aResult.EndsWith('/') && !aResult.EndsWith(":/") ? aResult.TrimEnd('/') : aResult;
    }

    internal static string[] SplitPath(string thePath)
    {
        string aPath = TclFileSystem.Slashes(thePath);
        var aParts = new List<string>();
        if (aPath.StartsWith('/'))
        {
            aParts.Add("/");
        }
        else if (aPath.Length >= 2 && aPath[1] == ':')
        {
            aParts.Add(aPath[..2] + "/");
            aPath = aPath[2..];
        }

        aParts.AddRange(aPath.Split('/', StringSplitOptions.RemoveEmptyEntries));
        return [.. aParts];
    }

    internal static string Tail(string thePath)
    {
        string[] aParts = SplitPath(thePath);
        return aParts.Length == 0 || (aParts.Length == 1 && IsAbsolute(aParts[0])) ? "" : aParts[^1];
    }

    internal static string DirName(string thePath)
    {
        string[] aParts = SplitPath(thePath);
        if (aParts.Length <= 1)
        {
            return aParts.Length == 1 && IsAbsolute(aParts[0]) ? aParts[0] : ".";
        }

        return Join(aParts[..^1]);
    }

    internal static string Extension(string thePath)
    {
        string aTail = Tail(thePath);
        int aDot = aTail.LastIndexOf('.');
        return aDot < 0 ? "" : aTail[aDot..];
    }

    internal static string RootName(string thePath)
    {
        string anExtension = Extension(thePath);
        return anExtension.Length == 0 ? thePath : thePath[..^anExtension.Length];
    }

    private static string File(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            throw Usage("file subcommand ?arg ...?");
        }

        TclFileSystem aFiles = theInterpreter.Files;
        string Arg(int theIndex) => theIndex < a.Count ? a[theIndex] : throw Usage($"file {a[1]} name");
        switch (TclControlCommands.Resolve(a[1], "subcommand", ["join", "split", "tail", "dirname", "extension", "rootname", "normalize", "nativename", "pathtype", "separator", "exists", "isfile", "isdirectory", "readable", "writable", "executable", "owned", "type", "size", "mtime", "atime", "mkdir", "delete", "copy", "rename", "tempdir", "attributes", "channels", "link", "lstat", "readlink", "stat", "system", "volumes", "tempfile"]))
        {
            case "join":
                return Join(a.Skip(2));
            case "split":
                return TclList.Make(SplitPath(Arg(2)));
            case "tail":
                return Tail(Arg(2));
            case "dirname":
                return DirName(Arg(2));
            case "extension":
                return Extension(Arg(2));
            case "rootname":
                return RootName(Arg(2));
            case "normalize":
                return aFiles.Full(Arg(2));
            case "nativename":
                return OperatingSystem.IsWindows() ? Arg(2).Replace('/', '\\') : Arg(2);
            case "pathtype":
                return Arg(2).StartsWith('/') && !OperatingSystem.IsWindows() || (Arg(2).Length >= 3 && Arg(2)[1] == ':' && Arg(2)[2] is '/' or '\\')
                           ? "absolute"
                           : Arg(2).StartsWith('/') ? "volumerelative" : "relative";
            case "separator":
                return OperatingSystem.IsWindows() ? "\\" : "/";
            case "exists":
                return aFiles.Exists(Arg(2)) ? "1" : "0";
            case "isfile":
                return aFiles.IsFile(Arg(2)) ? "1" : "0";
            case "isdirectory":
                return aFiles.IsDirectory(Arg(2)) ? "1" : "0";
            case "readable":
                return aFiles.Exists(Arg(2)) ? "1" : "0";
            case "writable":
                // Writes go to the in-memory overlay, so anything the script created is writable
                return aFiles.IsVirtual(Arg(2)) ? "1" : "0";
            case "executable" or "owned":
                return "0";
            case "type":
                return aFiles.IsDirectory(Arg(2)) ? "directory" : aFiles.IsFile(Arg(2)) ? "file" : throw new DrawException($"could not read \"{Arg(2)}\": no such file or directory");
            case "size":
                return aFiles.Size(Arg(2)).ToString(Invariant);
            case "mtime" or "atime":
            {
                string aFull = aFiles.Full(Arg(2));
                DateTime aTime = System.IO.File.Exists(aFull) ? System.IO.File.GetLastWriteTimeUtc(aFull) : DateTime.UtcNow;
                return new DateTimeOffset(aTime).ToUnixTimeSeconds().ToString(Invariant);
            }

            case "mkdir":
                foreach (string aPath in a.Skip(2))
                {
                    aFiles.MakeDirectory(aPath);
                }

                return "";
            case "delete":
                foreach (string aPath in a.Skip(2).Where(p => p is not ("-force" or "--")))
                {
                    aFiles.Delete(aPath);
                }

                return "";
            case "copy" or "rename":
            {
                string[] aPaths = a.Skip(2).Where(p => p is not ("-force" or "--")).ToArray();
                if (aPaths.Length < 2)
                {
                    throw Usage($"file {a[1]} ?-force? ?--? source ?source ...? target");
                }

                foreach (string aSource in aPaths[..^1])
                {
                    aFiles.Copy(aSource, aPaths[^1]);
                    if (a[1] == "rename")
                    {
                        aFiles.Delete(aSource);
                    }
                }

                return "";
            }

            case "tempdir":
                return TclFileSystem.Slashes(Path.GetTempPath()).TrimEnd('/');
            default:
                throw new DrawException($"file {a[1]}: not supported by Draw-lite");
        }
    }

    private static string Glob(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        bool aNoComplain = false, aTails = false;
        string? aDirectory = null;
        string? aType = null;
        int i = 1;
        for (; i < a.Count && a[i].StartsWith('-'); i++)
        {
            switch (a[i])
            {
                case "-nocomplain":
                    aNoComplain = true;
                    break;
                case "-tails":
                    aTails = true;
                    break;
                case "-directory" or "-path":
                    aDirectory = a[++i];
                    break;
                case "-types" or "-type":
                    aType = a[++i];
                    break;
                case "-join":
                    break;
                case "--":
                    i++;
                    goto Patterns;
                default:
                    throw new DrawException($"bad option \"{a[i]}\"");
            }
        }

    Patterns:
        var aResults = new List<string>();
        for (; i < a.Count; i++)
        {
            string aPattern = TclFileSystem.Slashes(a[i]);
            string aBase = aDirectory is null ? "" : TclFileSystem.Slashes(aDirectory);
            int aSlash = aPattern.LastIndexOf('/');
            if (aSlash >= 0)
            {
                aBase = Join([aBase, aPattern[..(aSlash + 1)]]);
                aPattern = aPattern[(aSlash + 1)..];
            }

            string aListed = aBase.Length == 0 ? "." : aBase;
            if (!theInterpreter.Files.IsDirectory(aListed))
            {
                continue;
            }

            foreach ((string aName, bool anIsDirectory) in theInterpreter.Files.List(aListed).OrderBy(e => e.Name, StringComparer.Ordinal))
            {
                if (aName.StartsWith('.') && !aPattern.StartsWith('.'))
                {
                    continue;
                }

                if ((aType == "d" && !anIsDirectory) || (aType == "f" && anIsDirectory) || !TclGlob.Match(aPattern, aName, OperatingSystem.IsWindows()))
                {
                    continue;
                }

                aResults.Add(aTails || aBase.Length == 0 ? aName : Join([aBase, aName]));
            }
        }

        if (aResults.Count == 0 && !aNoComplain)
        {
            throw new DrawException($"no files matched glob pattern{(a.Count - i > 1 ? "s" : "")} \"{string.Join(' ', a.Skip(i))}\"");
        }

        return TclList.Make([.. aResults]);
    }

    // -------------------------------------------------------------------------------------------------------------
    // clock

    private static string Clock(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            throw Usage("clock subcommand ?arg ...?");
        }

        switch (TclControlCommands.Resolve(a[1], "subcommand", ["seconds", "milliseconds", "microseconds", "clicks", "format", "add", "scan"]))
        {
            case "seconds":
                return DateTimeOffset.UtcNow.ToUnixTimeSeconds().ToString(Invariant);
            case "milliseconds":
                return DateTimeOffset.UtcNow.ToUnixTimeMilliseconds().ToString(Invariant);
            case "microseconds":
                return ((DateTimeOffset.UtcNow - DateTimeOffset.UnixEpoch).Ticks / 10).ToString(Invariant);
            case "clicks":
                return (a.Count > 2 && a[2] == "-milliseconds" ? s_clicks.ElapsedMilliseconds
                       : a.Count > 2 && a[2] == "-microseconds" ? s_clicks.Elapsed.Ticks / 10
                       : s_clicks.ElapsedTicks).ToString(Invariant);
            case "format" when a.Count >= 3:
            {
                string aFormat = "%a %b %d %H:%M:%S %Z %Y";
                bool aGmt = false;
                for (int i = 3; i + 1 < a.Count; i += 2)
                {
                    if (a[i] == "-format")
                    {
                        aFormat = a[i + 1];
                    }
                    else if (a[i] == "-gmt")
                    {
                        aGmt = TclNumber.GetBoolean(a[i + 1]);
                    }
                }

                DateTimeOffset aTime = DateTimeOffset.FromUnixTimeSeconds(TclNumber.GetInt(a[2]));
                return FormatTime(aGmt ? aTime : aTime.ToLocalTime(), aFormat, aGmt);
            }

            case "add" or "scan":
                throw new DrawException($"clock {a[1]}: not supported by Draw-lite");
            default:
                throw new DrawException($"bad option \"{a[1]}\": must be clicks, format, microseconds, milliseconds, scan, or seconds");
        }
    }

    private static string FormatTime(DateTimeOffset theTime, string theFormat, bool theGmt)
    {
        var aBuilder = new StringBuilder();
        for (int i = 0; i < theFormat.Length; i++)
        {
            if (theFormat[i] != '%' || i + 1 >= theFormat.Length)
            {
                aBuilder.Append(theFormat[i]);
                continue;
            }

            char c = theFormat[++i];
            aBuilder.Append(c switch
            {
                'Y' => theTime.Year.ToString("0000", Invariant),
                'y' => (theTime.Year % 100).ToString("00", Invariant),
                'm' => theTime.Month.ToString("00", Invariant),
                'd' => theTime.Day.ToString("00", Invariant),
                'e' => theTime.Day.ToString(Invariant).PadLeft(2),
                'H' => theTime.Hour.ToString("00", Invariant),
                'I' => ((theTime.Hour + 11) % 12 + 1).ToString("00", Invariant),
                'M' => theTime.Minute.ToString("00", Invariant),
                'S' => theTime.Second.ToString("00", Invariant),
                'p' => theTime.Hour < 12 ? "AM" : "PM",
                'j' => theTime.DayOfYear.ToString("000", Invariant),
                'b' or 'h' => Invariant.DateTimeFormat.GetAbbreviatedMonthName(theTime.Month),
                'B' => Invariant.DateTimeFormat.GetMonthName(theTime.Month),
                'a' => Invariant.DateTimeFormat.GetAbbreviatedDayName(theTime.DayOfWeek),
                'A' => Invariant.DateTimeFormat.GetDayName(theTime.DayOfWeek),
                'u' => (theTime.DayOfWeek == DayOfWeek.Sunday ? 7 : (int)theTime.DayOfWeek).ToString(Invariant),
                'w' => ((int)theTime.DayOfWeek).ToString(Invariant),
                's' => theTime.ToUnixTimeSeconds().ToString(Invariant),
                'Z' => theGmt ? "GMT" : (TimeZoneInfo.Local.IsDaylightSavingTime(theTime) ? TimeZoneInfo.Local.DaylightName : TimeZoneInfo.Local.StandardName),
                'z' => theTime.ToString("zzz", Invariant).Replace(":", ""),
                'T' => theTime.ToString("HH:mm:ss", Invariant),
                'D' => theTime.ToString("MM/dd/yy", Invariant),
                'R' => theTime.ToString("HH:mm", Invariant),
                'c' => theTime.ToString("ddd MMM dd HH:mm:ss yyyy", Invariant),
                '%' => "%",
                _ => "%" + c,
            });
        }

        return aBuilder.ToString();
    }

    /// <summary>The tcl_platform array, as Tcl 8.6 sets it.</summary>
    internal static IEnumerable<(string Key, string Value)> Platform()
    {
        bool aWindows = OperatingSystem.IsWindows();
        yield return ("platform", aWindows ? "windows" : "unix");
        yield return ("os", aWindows ? "Windows NT" : OperatingSystem.IsMacOS() ? "Darwin" : "Linux");
        yield return ("osVersion", Environment.OSVersion.Version.ToString());
        yield return ("machine", RuntimeInformation.OSArchitecture.ToString().ToLowerInvariant());
        yield return ("byteOrder", BitConverter.IsLittleEndian ? "littleEndian" : "bigEndian");
        yield return ("wordSize", "8");
        yield return ("pointerSize", "8");
        yield return ("user", Environment.UserName);
        yield return ("threaded", "1");
        yield return ("pathSeparator", aWindows ? ";" : ":");
    }
}
