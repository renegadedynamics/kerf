// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_Context.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_Context.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Foundation.Resources;
using Occt;

namespace Kerf.Engine.Healing.Process;

/// <summary>
/// The context for shape processing: the resource manager holding the operators' parameters, the current scope (the
/// sequence and operator whose parameters are read), the messenger and the trace level.
/// </summary>
/// <remarks>
/// <para>
/// Parameters are read in the current scope: in scope <c>ToV4.SplitAngle</c>, <c>GetReal("Angle")</c> reads the
/// resource <c>ToV4.SplitAngle.Angle</c>. A real or integer value <c>&amp;name</c> refers to the resource
/// <c>name</c>.
/// </para>
/// <para>
/// Resource files (open question 26, decision 1023): OCCT finds the file <c>name</c> in the directories the environment
/// variables <c>CSF_nameDefaults</c> and <c>CSF_nameUserDefaults</c> name. The port reads no environment variable:
/// without directories it loads OCCT's own resource file of that name, compiled in
/// (<see cref="Manager.EmbeddedDefaults"/>: <c>IGES</c>, <c>STEP</c>) or nothing; files are read only from
/// the directories the caller gives (the optional arguments, which OCCT doesn't have). The process-wide cache of the
/// last file loaded (OCCT's, behind a static mutex) is kept: it is reloaded when the name, the directories or the files'
/// modification times change, and each context gets its own copy.
/// </para>
/// <para>A working object (decision 21).</para>
/// </remarks>
public class ProcessContext : Transient
{
    private Manager? _rc;
    private NCollection_Sequence<string>? _scope;
    private Messenger _messenger;
    private int _traceLev;

    // The process-wide cache of LoadResourceManager (OCCT's statics, behind its static mutex)
    private static readonly Lock THE_SHAPE_PROCESS_MUTEX = new();
    private static Manager? s_sRC;
    private static long s_sMtime, s_sUMtime;
    private static string s_sName = "";
    private static string s_sPath = "", s_sUPath = ""; // OCCT.NET: the files, part of the name (decision 1023)

    /// <summary>Creates an empty context (no resource manager), trace level 1.</summary>
    public ProcessContext()
    {
        _messenger = global::Kerf.Engine.Foundation.Message.DefaultMessenger();
        _traceLev = 1;
    }

    /// <summary>
    /// Creates a context with the resource file <paramref name="file"/> (see <see cref="Init"/>) and the scope
    /// <paramref name="scope"/>.
    /// </summary>
    public ProcessContext(string file, string scope = "", string? defaultsDirectory = null,
                                string? userDefaultsDirectory = null)
    {
        Init(file, scope, defaultsDirectory, userDefaultsDirectory);
        _messenger = global::Kerf.Engine.Foundation.Message.DefaultMessenger();
        _traceLev = 1;
    }

    /// <summary>
    /// Loads the resource file <paramref name="file"/> (an empty name: an empty resource manager) and sets the scope
    /// <paramref name="scope"/>. The file is OCCT's own of that name, compiled in, unless directories are given (an
    /// addition of the port: the file of that name in <paramref name="defaultsDirectory"/>, then the one in
    /// <paramref name="userDefaultsDirectory"/> over it). Always true.
    /// </summary>
    public bool Init(string file, string scope = "", string? defaultsDirectory = null, string? userDefaultsDirectory = null)
    {
        _scope = null;
        if (!string.IsNullOrEmpty(file))
        {
            _rc = LoadResourceManager(file, defaultsDirectory, userDefaultsDirectory);
        }
        else
        {
            _rc = new Manager();
        }
        if (!string.IsNullOrEmpty(scope))
        {
            SetScope(scope);
        }
        return true; // myRC->Length() >0; NOT IMPLEMENTED
    }

    /// <summary>
    /// A new resource manager for the resource file <paramref name="name"/> (see <see cref="Init"/>): a copy of the
    /// process-wide cached one, loaded again only if the name, the directories or the files' modification times
    /// changed.
    /// </summary>
    public Manager LoadResourceManager(string name, string? defaultsDirectory = null,
                                                string? userDefaultsDirectory = null)
    {
        // Mutex is needed because we are initializing and changing static variables here, so
        // without mutex it leads to race condition.
        lock (THE_SHAPE_PROCESS_MUTEX)
        {
            // Optimisation of loading resource file: file is load only once
            // and reloaded only if file date has changed
            long mtime = 0, uMtime = 0;
            string path = string.IsNullOrEmpty(defaultsDirectory) ? "" : Manager.ResourcePath(defaultsDirectory, name);
            string userPath = string.IsNullOrEmpty(userDefaultsDirectory) ? "" : Manager.ResourcePath(userDefaultsDirectory, name);
            if (path.Length != 0)
            {
                mtime = ModificationTime(path);
            }
            if (userPath.Length != 0)
            {
                uMtime = ModificationTime(userPath);
            }

            bool isFileModified = false;
            if (s_sRC is not null)
            {
                if (s_sName == name && s_sPath == path && s_sUPath == userPath)
                {
                    if (s_sMtime != mtime)
                    {
                        s_sMtime = mtime;
                        isFileModified = true;
                    }
                    if (s_sUMtime != uMtime)
                    {
                        s_sUMtime = uMtime;
                        isFileModified = true;
                    }
                    if (isFileModified)
                    {
                        s_sRC = null;
                    }
                }
                else
                {
                    s_sRC = null;
                }
            }
            if (s_sRC is null)
            {
                // OCCT_DEBUG: an info message on reloading, left out
                s_sRC = path.Length == 0 && userPath.Length == 0
                    ? new Manager(name)
                    : new Manager(name, path.Length == 0 ? null : defaultsDirectory, userPath.Length == 0 ? null : userDefaultsDirectory);
                if (!isFileModified)
                {
                    s_sName = name;
                    s_sPath = path;
                    s_sUPath = userPath;
                    s_sMtime = mtime;
                    s_sUMtime = uMtime;
                }
            }
            // Creating copy of sRC for thread safety of Resource_Manager variables
            // We should return copy because calling of Resource_Manager::SetResource() for one object
            // in multiple threads causes race condition
            return new Manager(s_sRC);
        }
    }

    // stat's st_mtime: the file's modification time; 0 when the file can't be read (OCCT leaves stat's buffer as is)
    private static long ModificationTime(string path)
    {
        try
        {
            return File.Exists(path) ? File.GetLastWriteTimeUtc(path).Ticks : 0;
        }
        catch (Exception exception) when (exception is IOException or UnauthorizedAccessException or ArgumentException
                                                or NotSupportedException)
        {
            return 0;
        }
    }

    /// <summary>
    /// Forgets the process-wide cached resource manager (an addition of the port, for test harnesses that run cases
    /// one after another: the next <see cref="LoadResourceManager"/> loads the file again).
    /// </summary>
    internal static void ClearResourceCache()
    {
        lock (THE_SHAPE_PROCESS_MUTEX)
        {
            s_sRC = null;
            s_sName = "";
            s_sPath = "";
            s_sUPath = "";
            s_sMtime = 0;
            s_sUMtime = 0;
        }
    }

    /// <summary>The resource manager (null for a context made empty).</summary>
    public Manager? ResourceManager() => _rc;

    /// <summary>Enters the scope <paramref name="scope"/>, nested in the current one.</summary>
    public void SetScope(string scope)
    {
        _scope ??= new NCollection_Sequence<string>();
        string str;
        if (_scope.Length() > 0)
        {
            str = _scope.Value(_scope.Length()) + "." + scope;
        }
        else
        {
            str = scope;
        }
        _scope.Append(str);
    }

    /// <summary>Leaves the current scope.</summary>
    public void UnSetScope()
    {
        if (_scope is not null && _scope.Length() > 0)
        {
            _scope.Remove(_scope.Length());
        }
    }

    private static string MakeName(NCollection_Sequence<string>? scope, string param)
    {
        if (scope is not null && scope.Length() > 0)
        {
            return scope.Value(scope.Length()) + "." + param;
        }
        return param;
    }

    /// <summary>True if the parameter is defined in the current scope.</summary>
    public bool IsParamSet(string param) => _rc is not null && _rc.Find(MakeName(_scope, param));

    /// <summary>The value of the parameter as a string; false (<paramref name="str"/> unchanged) if it isn't defined.</summary>
    public bool GetString(string param, ref string str)
    {
        if (_rc is null)
        {
            return false;
        }
        string pname = MakeName(_scope, param);
        if (!_rc.Find(pname))
        {
            // OCCT_DEBUG: a warning that the parameter is not defined, left out
            return false;
        }
        str = _rc.Value(pname);
        return true;
    }

    /// <summary>
    /// The value of a real parameter, or of the resource a value <c>&amp;name</c> refers to; false
    /// (<paramref name="val"/> unchanged) if it isn't defined or isn't a real.
    /// </summary>
    public bool GetReal(string param, ref double val)
    {
        if (_rc is null)
        {
            return false;
        }

        string str = "";
        if (!GetString(param, ref str))
        {
            return false;
        }

        if (TCollection_AsciiString.IsRealValue(str))
        {
            val = TCollection_AsciiString.RealValue(str);
            return true;
        }

        // if not real, try to treat as alias "&param"
        str = str.TrimStart(TrimmedSpaces);
        if (Value1(str) == '&')
        {
            string @ref = str.Substring(1).Trim(TrimmedSpaces);
            if (!_rc.Find(@ref))
            {
                // OCCT_DEBUG: a warning that the parameter is not defined, left out
                return false;
            }
            str = _rc.Value(@ref);
            if (TCollection_AsciiString.IsRealValue(str))
            {
                val = TCollection_AsciiString.RealValue(str);
                return true;
            }
        }
        // OCCT_DEBUG: a warning that the parameter is neither a real nor a reference to one, left out
        return false;
    }

    /// <summary>
    /// The value of an integer parameter, or of the resource a value <c>&amp;name</c> refers to; false
    /// (<paramref name="val"/> unchanged) if it isn't defined or isn't an integer.
    /// </summary>
    public bool GetInteger(string param, ref int val)
    {
        if (_rc is null)
        {
            return false;
        }

        string str = "";
        if (!GetString(param, ref str))
        {
            return false;
        }

        if (TCollection_AsciiString.IsIntegerValue(str))
        {
            val = TCollection_AsciiString.IntegerValue(str);
            return true;
        }

        // if not integer, try to treat as alias "&param"
        str = str.TrimStart(TrimmedSpaces);
        if (Value1(str) == '&')
        {
            string @ref = str.Substring(1).Trim(TrimmedSpaces);
            if (!_rc.Find(@ref))
            {
                // OCCT_DEBUG: a warning that the parameter is not defined, left out
                return false;
            }
            str = _rc.Value(@ref);
            if (TCollection_AsciiString.IsIntegerValue(str))
            {
                val = TCollection_AsciiString.IntegerValue(str);
                return true;
            }
        }
        // OCCT_DEBUG: a warning that the parameter is neither an integer nor a reference to one, left out
        return false;
    }

    // TCollection_AsciiString's LeftAdjust and RightAdjust remove what IsSpace (C's isspace) accepts
    private static readonly char[] TrimmedSpaces = [' ', '\t', '\n', '\v', '\f', '\r'];

    // TCollection_AsciiString::Value(1), which throws Standard_OutOfRange on an empty string
    private static char Value1(string theString)
    {
        if (theString.Length == 0)
        {
            throw new OutOfRangeException("TCollection_AsciiString::Value : parameter theWhere");
        }
        return theString[0];
    }

    /// <summary>
    /// The value of a boolean parameter (an integer, true if not 0); false (<paramref name="val"/> unchanged) if it
    /// isn't defined or isn't an integer.
    /// </summary>
    public bool GetBoolean(string param, ref bool val)
    {
        if (_rc is null)
        {
            return false;
        }
        // OCCT reads Integer() and catches its Standard_Failure when the parameter is missing or not an integer, which
        // is usual: the operators ask for every option they have. The status form says the same without the exception
        // (unit 10F)
        if (!_rc.TryGetInteger(MakeName(_scope, param), out int value))
        {
            return false;
        }
        val = value != 0;
        return true;
    }

    /// <summary>The value of a real parameter, or <paramref name="def"/>.</summary>
    public double RealVal(string param, double def)
    {
        double val = 0;
        return GetReal(param, ref val) ? val : def;
    }

    /// <summary>The value of a boolean parameter, or <paramref name="def"/>.</summary>
    public bool BooleanVal(string param, bool def)
    {
        bool val = false;
        return GetBoolean(param, ref val) ? val : def;
    }

    /// <summary>The value of an integer parameter, or <paramref name="def"/>.</summary>
    public int IntegerVal(string param, int def)
    {
        int val = 0;
        return GetInteger(param, ref val) ? val : def;
    }

    /// <summary>The value of a parameter as a string, or <paramref name="def"/>.</summary>
    public string? StringVal(string param, string? def)
    {
        if (_rc is null)
        {
            return def;
        }
        // OCCT catches Value()'s Standard_NoSuchObject for a missing parameter; Find says the same (unit 10F)
        return _rc.Find(MakeName(_scope, param), out string value) ? value : def;
    }

    /// <summary>Sets the messenger for the messages (null: the default messenger).</summary>
    public void SetMessenger(Messenger? messenger)
    {
        if (messenger is null)
        {
            _messenger = global::Kerf.Engine.Foundation.Message.DefaultMessenger();
        }
        else
        {
            _messenger = messenger;
        }
    }

    /// <summary>The messenger used for the messages.</summary>
    public Messenger Messenger() => _messenger;

    /// <summary>
    /// Sets the trace level for the messages: 0 none, 1 errors (the default), 2 errors and warnings, 3 all.
    /// </summary>
    public void SetTraceLevel(int tracelev)
    {
        _traceLev = tracelev;
    }

    /// <summary>The trace level for the messages.</summary>
    public int TraceLevel() => _traceLev;
}
