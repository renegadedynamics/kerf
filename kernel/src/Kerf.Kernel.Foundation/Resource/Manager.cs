// Port of: src/FoundationClasses/TKernel/Resource/Resource_Manager.hxx
// Port of: src/FoundationClasses/TKernel/Resource/Resource_Manager.cxx
// Port of: src/FoundationClasses/TKernel/Resource/Resource_NoSuchResource.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Occt;
using System.Text;

namespace Kerf.Engine.Foundation.Resources;

/// <summary>Raised by <see cref="Manager.Value"/> for a resource that isn't defined.</summary>
public class NoSuchResourceException : NoSuchObjectException
{
    public NoSuchResourceException() { }

    public NoSuchResourceException(string? message) : base(message) { }
}

/// <summary>
/// Defines a resource structure and its management methods: resources are <c>name : value</c> pairs read from resource
/// files, which a program can query and change.
/// </summary>
/// <remarks>
/// <para>
/// A resource file has one resource per line, <c>name : value</c>; white space around the name and before the value
/// is ignored (a value starting with white space or a backslash escapes it with a backslash), lines starting with
/// <c>!</c> are comments.
/// </para>
/// <para>
/// OCCT.NET reads no environment variable (open question 26): OCCT's <c>Resource_Manager(aName)</c> loads the files
/// that <c>CSF_&lt;aName&gt;Defaults</c> and <c>CSF_&lt;aName&gt;UserDefaults</c> name; the port's loads the resource
/// file OCCT ships under that name, compiled in (<see cref="EmbeddedDefaults"/>: OCCT's <c>IGES</c> and <c>STEP</c>
/// files of resources/XSTEPResource, which hold ShapeProcess's sequences for data exchange), and nothing for another
/// name. Files are read only where the caller names their directories (the constructor that takes them, as OCCT's).
/// <c>GetResourcePath</c> (an environment lookup) is left out; <see cref="ResourcePath"/> composes the path in a
/// given directory. <c>Save</c> writes into a given directory instead of <c>CSF_&lt;aName&gt;UserDefaults</c>.
/// The <c>ResourceDebug</c> and <c>CSF_ResourceVerbose</c> switches are left out (debug output only), and so are
/// <c>ExtValue</c> and <c>SetResource</c> of an extended string, which go through Resource_Unicode's process-wide
/// format (not ported).
/// </para>
/// <para>A working object (decision 21): its resources change in place.</para>
/// </remarks>
public class Manager : Transient
{
    private readonly string _name;
    private readonly Dictionary<string, string> _refMap;
    private readonly Dictionary<string, string> _userMap;
    private readonly bool _verbose;
    private bool _initialized;

    /// <summary>
    /// Creates a resource manager for the resource file <paramref name="name"/> in
    /// <paramref name="defaultsDirectory"/>, then the one in <paramref name="userDefaultsDirectory"/> (each
    /// skipped when its directory is empty or null), the second overriding the first.
    /// </summary>
    public Manager(string name, string? defaultsDirectory, string? userDefaultsDirectory,
                            bool isVerbose = false)
    {
        _name = name;
        _verbose = isVerbose;
        _initialized = false;
        _refMap = new Dictionary<string, string>(StringComparer.Ordinal);
        _userMap = new Dictionary<string, string>(StringComparer.Ordinal);
        if (!string.IsNullOrEmpty(defaultsDirectory))
        {
            Load(ResourcePath(defaultsDirectory, name), _refMap);
        }
        else if (_verbose)
        {
            Console.Out.Write("Resource Manager Warning: aDefaultsDirectory is empty." + '\n');
        }

        if (!string.IsNullOrEmpty(userDefaultsDirectory))
        {
            // OCCT: the user file goes into the reference map too
            Load(ResourcePath(userDefaultsDirectory, name), _refMap);
        }
        else if (_verbose)
        {
            Console.Out.Write("Resource Manager Warning: anUserDefaultsDirectory is empty." + '\n');
        }
    }

    /// <summary>
    /// Creates a resource manager for the resources named <paramref name="aName"/>: OCCT's own resource file of that
    /// name, compiled in (<see cref="EmbeddedDefaults"/>), or none. OCCT reads the files the environment variables
    /// <c>CSF_aNameDefaults</c> and <c>CSF_aNameUserDefaults</c> name; the port reads no environment variable.
    /// </summary>
    public Manager(string aName, bool verbose = false)
    {
        _name = aName;
        _verbose = verbose;
        _initialized = false;
        _refMap = new Dictionary<string, string>(StringComparer.Ordinal);
        _userMap = new Dictionary<string, string>(StringComparer.Ordinal);

        string? text = EmbeddedDefaults(aName);
        if (text is not null)
        {
            LoadText(text, aName, _refMap);
        }
        else if (_verbose)
        {
            Console.Out.Write("Resource Manager Warning: no resource file \"" + aName + "\" compiled in." + '\n');
        }
    }

    /// <summary>Creates an empty resource manager.</summary>
    public Manager()
    {
        _name = "";
        _verbose = false;
        _initialized = false;
        _refMap = new Dictionary<string, string>(StringComparer.Ordinal);
        _userMap = new Dictionary<string, string>(StringComparer.Ordinal);
    }

    /// <summary>A copy of <paramref name="other"/>, with copies of its resources (OCCT's implicit copy constructor).</summary>
    public Manager(Manager other)
    {
        _name = other._name;
        _verbose = other._verbose;
        _initialized = other._initialized;
        _refMap = new Dictionary<string, string>(other._refMap,other._refMap.Comparer);
        _userMap = new Dictionary<string, string>(other._userMap,other._userMap.Comparer);
    }

    /// <summary>
    /// The text of OCCT's resource file <paramref name="name"/> as compiled into the port, or null: <c>IGES</c> and
    /// <c>STEP</c> (resources/XSTEPResource).
    /// </summary>
    public static string? EmbeddedDefaults(string name) => name switch
    {
        "IGES" => IgesResources.Text,
        "STEP" => StepResources.Text,
        _ => null,
    };

    /// <summary>
    /// The path of the resource file <paramref name="name"/> in <paramref name="directory"/> (OCCT's
    /// <c>GetResourcePath</c> composes it so, in the directory an environment variable names).
    /// </summary>
    public static string ResourcePath(string directory, string name) => Path.Combine(directory, name);

    private void Load(string path, Dictionary<string, string> aMap)
    {
        string fileName = Path.GetFileName(path);
        byte[] bytes;
        try
        {
            bytes = File.ReadAllBytes(path);
        }
        catch (Exception exception) when (exception is IOException or UnauthorizedAccessException or ArgumentException
                                                or NotSupportedException)
        {
            if (_verbose)
            {
                Console.Out.Write("Resource Manager Warning: Cannot read file \"" + fileName
                                  + "\". File not found or permission denied." + '\n');
            }
            return;
        }

        // OCCT reads bytes: one character per byte
        LoadText(Encoding.Latin1.GetString(bytes), fileName, aMap);
    }

    private void LoadText(string text, string fileName, Dictionary<string, string> aMap)
    {
        _initialized = true;
        int lineNumber = 1;
        int position = 0;
        Resource_KindOfLine kind;
        while ((kind = WhatKindOfLine(text, ref position, out string token1, out string token2)) != Resource_KindOfLine.KOL_End)
        {
            switch (kind)
            {
                case Resource_KindOfLine.KOL_End:
                case Resource_KindOfLine.KOL_Comment:
                case Resource_KindOfLine.KOL_Empty:
                    break;
                case Resource_KindOfLine.KOL_Resource:
                    aMap[token1] = token2;
                    break;
                case Resource_KindOfLine.KOL_Error:
                    if (_verbose)
                    {
                        Console.Out.Write("Resource Manager: Syntax error at line " + lineNumber + " in file : " + fileName + '\n');
                    }
                    break;
            }
            lineNumber++;
        }
        if (_verbose)
        {
            Console.Out.Write("Resource Manager: " + (ReferenceEquals(aMap, _userMap) ? "User" : "Reference") + " file \""
                              + fileName + "\" loaded" + '\n');
        }
    }

    // Auxiliary enumeration for function WhatKindOfLine().
    private enum Resource_KindOfLine
    {
        KOL_End,
        KOL_Empty,
        KOL_Comment,
        KOL_Resource,
        KOL_Error,
    }

    // The next line of the text from thePosition, always ending with '\n' (GetLine); null at the end.
    private static string? GetLine(string text, ref int position)
    {
        if (position >= text.Length)
        {
            return null;
        }

        int end = text.IndexOf('\n', position);
        string line;
        if (end < 0)
        {
            line = text.Substring(position) + "\n";
            position = text.Length;
        }
        else
        {
            line = text.Substring(position, end + 1 - position);
            position = end + 1;
        }
        return line;
    }

    // OCCT's WhatKindOfLine on a line that ends with '\n' (1-based positions as TCollection_AsciiString's).
    private static Resource_KindOfLine WhatKindOfLine(string text, ref int position, out string aToken1, out string aToken2)
    {
        aToken1 = "";
        aToken2 = "";
        string? line = GetLine(text, ref position);
        if (line is null)
        {
            return Resource_KindOfLine.KOL_End;
        }

        char Value(int index) => line[index - 1];
        int FirstLocationNotInSet(int theFrom)
        {
            for (int i = theFrom; i <= line.Length; i++)
            {
                if (line[i - 1] != ' ' && line[i - 1] != '\t')
                {
                    return i;
                }
            }
            return 0;
        }

        if (Value(1) == '!')
        {
            return Resource_KindOfLine.KOL_Comment;
        }

        int pos1 = FirstLocationNotInSet(1);
        if (Value(pos1) == '\n')
        {
            return Resource_KindOfLine.KOL_Empty;
        }

        int pos2 = line.IndexOf(':', pos1 - 1) + 1;
        if (pos2 == 0 || pos1 == pos2)
        {
            return Resource_KindOfLine.KOL_Error;
        }

        int pos;
        for (pos = pos2 - 1; Value(pos) == '\t' || Value(pos) == ' '; pos--)
        {
        }
        aToken1 = line.Substring(pos1 - 1, pos - pos1 + 1);

        pos = FirstLocationNotInSet(pos2 + 1);
        if (pos != 0)
        {
            if (Value(pos) == '\\')
            {
                switch (Value(pos + 1))
                {
                    case '\\':
                    case ' ':
                    case '\t':
                        pos++;
                        break;
                }
            }
        }
        if (pos == line.Length)
        {
            aToken2 = "";
        }
        else
        {
            line = line.Remove(0, pos - 1);
            int lineLength = line.Length;
            if (lineLength >= 2)
            {
                if (line[lineLength - 2] == '\r')
                {
                    line = line.Remove(lineLength - 2, 1);
                }
            }
            line = line.Remove(line.Length - 1);
            aToken2 = line;
        }
        return Resource_KindOfLine.KOL_Resource;
    }

    /// <summary>
    /// Sorts the user resources (those set with <see cref="SetResource(string, string)"/>) and saves them in the file
    /// named after the manager in <paramref name="userDefaultsDirectory"/>, creating the directory if needed. False
    /// if the directory is empty or the file can't be written. OCCT saves into the directory
    /// <c>CSF_&lt;name&gt;UserDefaults</c> names.
    /// </summary>
    public bool Save(string? userDefaultsDirectory)
    {
        if (string.IsNullOrEmpty(userDefaultsDirectory))
        {
            if (_verbose)
            {
                Console.Out.Write("Resource Manager Warning: no directory given.  Cannot save resources." + '\n');
            }
            return false;
        }

        string filePath = ResourcePath(userDefaultsDirectory, _name);
        try
        {
            Directory.CreateDirectory(userDefaultsDirectory);
        }
        catch (Exception exception) when (exception is IOException or UnauthorizedAccessException or ArgumentException
                                                or NotSupportedException)
        {
            if (_verbose)
            {
                Console.Out.Write("Resource Manager: Error opening or creating directory \"" + userDefaultsDirectory
                                  + "\". Permission denied. Cannot save resources." + '\n');
            }
            return false;
        }

        var text = new StringBuilder();
        int nbKey = _userMap.Count;
        if (nbKey != 0)
        {
            var keyArray = new string[nbKey];
            int index = 0;
            foreach (KeyValuePair<string, string> iter in _userMap)
            {
                keyArray[index++] = iter.Key;
            }

            // std::sort of TCollection_AsciiString: byte order
            Array.Sort(keyArray, StringComparer.Ordinal);

            for (index = 0; index < nbKey; index++)
            {
                string value = _userMap.GetValue(keyArray[index]);
                if (value.Length != 0)
                {
                    switch (value[0])
                    {
                        case '\\':
                        case ' ':
                        case '\t':
                            value = value.Insert(0, "\\");
                            break;
                    }
                }
                text.Append(keyArray[index]).Append(":\t").Append(value).Append('\n');
            }
        }

        try
        {
            File.WriteAllBytes(filePath, Encoding.Latin1.GetBytes(text.ToString()));
        }
        catch (Exception exception) when (exception is IOException or UnauthorizedAccessException or ArgumentException
                                                or NotSupportedException)
        {
            if (_verbose)
            {
                Console.Out.Write("Resource Manager: Error opening or creating file \"" + filePath
                                  + "\". Permission denied. Cannot save resources." + '\n');
            }
            return false;
        }
        if (_verbose && nbKey != 0)
        {
            Console.Out.Write("Resource Manager: Resources saved in file " + filePath + '\n');
        }
        return true;
    }

    /// <summary>True if the resource is defined.</summary>
    public bool Find(string aResource) => _userMap.ContainsKey(aResource) || _refMap.ContainsKey(aResource);

    /// <summary>True if the resource is defined, and its value.</summary>
    public bool Find(string resource, out string value)
    {
        if (_userMap.TryGetValue(resource, out value!))
        {
            return true;
        }
        return _refMap.TryGetValue(resource, out value!);
    }

    /// <summary>
    /// The value of an integer resource, if it is defined and an integer: <see cref="Integer"/> without its exceptions
    /// (Standard_NoSuchObject, Standard_TypeMismatch), for callers that only want to know (unit 10F).
    /// </summary>
    public bool TryGetInteger(string resourceName, out int value)
    {
        if (Find(resourceName, out string text) && TCollection_AsciiString.IsIntegerValue(text))
        {
            value = TCollection_AsciiString.IntegerValue(text);
            return true;
        }
        value = 0;
        return false;
    }

    /// <summary>The value of an integer resource; Standard_TypeMismatch if it isn't an integer.</summary>
    public virtual int Integer(string aResourceName)
    {
        string result = Value(aResourceName);
        if (!TCollection_AsciiString.IsIntegerValue(result))
        {
            throw new TypeMismatchException("Value of resource `" + aResourceName + "` is not an integer");
        }
        return TCollection_AsciiString.IntegerValue(result);
    }

    /// <summary>The value of a real resource; Standard_TypeMismatch if it isn't a real.</summary>
    public virtual double Real(string aResourceName)
    {
        string result = Value(aResourceName);
        if (!TCollection_AsciiString.IsRealValue(result))
        {
            throw new TypeMismatchException("Value of resource `" + aResourceName + "` is not a real");
        }
        return TCollection_AsciiString.RealValue(result);
    }

    /// <summary>The value of a resource (the user's value first); Resource_NoSuchResource if it isn't defined.</summary>
    public virtual string Value(string aResource)
    {
        if (_userMap.ContainsKey(aResource))
        {
            return _userMap.GetValue(aResource);
        }
        if (_refMap.ContainsKey(aResource))
        {
            return _refMap.GetValue(aResource);
        }
        throw new NoSuchResourceException(aResource);
    }

    /// <summary>Sets the value of an integer resource, creating it if needed.</summary>
    public virtual void SetResource(string aResourceName, int aValue)
    {
        SetResource(aResourceName, aValue.ToString(System.Globalization.CultureInfo.InvariantCulture));
    }

    /// <summary>Sets the value of a real resource (as C's <c>%g</c> writes it), creating it if needed.</summary>
    public virtual void SetResource(string aResourceName, double aValue)
    {
        SetResource(aResourceName, StreamUtilities.FormatReal(aValue, 6));
    }

    /// <summary>Sets the value of a resource, creating it if needed.</summary>
    public virtual void SetResource(string aResource, string aValue)
    {
        _userMap[aResource] = aValue;
    }

    /// <summary>
    /// The reference resources (read from the files), or with <paramref name="refMap"/> false the user's (set with
    /// SetResource); the map itself, which the caller may change.
    /// </summary>
    public Dictionary<string, string> GetMap(bool refMap = true) => refMap ? _refMap : _userMap;

    /// <summary>True if a resource file was read.</summary>
    public bool IsInitialized => _initialized;
}
