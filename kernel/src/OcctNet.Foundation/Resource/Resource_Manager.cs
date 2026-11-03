// Port of: src/FoundationClasses/TKernel/Resource/Resource_Manager.hxx
// Port of: src/FoundationClasses/TKernel/Resource/Resource_Manager.cxx
// Port of: src/FoundationClasses/TKernel/Resource/Resource_NoSuchResource.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;

namespace Occt;

/// <summary>Raised by <see cref="Resource_Manager.Value"/> for a resource that isn't defined.</summary>
[OcctName("Resource_NoSuchResource")]
public class Resource_NoSuchResource : Standard_NoSuchObject
{
    public Resource_NoSuchResource() { }

    public Resource_NoSuchResource(string? message) : base(message) { }
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
public class Resource_Manager : Standard_Transient
{
    private readonly string myName;
    private readonly NCollection_DataMap<string, string> myRefMap;
    private readonly NCollection_DataMap<string, string> myUserMap;
    private readonly bool myVerbose;
    private bool myInitialized;

    /// <summary>
    /// Creates a resource manager for the resource file <paramref name="theName"/> in
    /// <paramref name="theDefaultsDirectory"/>, then the one in <paramref name="theUserDefaultsDirectory"/> (each
    /// skipped when its directory is empty or null), the second overriding the first.
    /// </summary>
    public Resource_Manager(string theName, string? theDefaultsDirectory, string? theUserDefaultsDirectory,
                            bool theIsVerbose = false)
    {
        myName = theName;
        myVerbose = theIsVerbose;
        myInitialized = false;
        myRefMap = new NCollection_DataMap<string, string>(StringComparer.Ordinal);
        myUserMap = new NCollection_DataMap<string, string>(StringComparer.Ordinal);
        if (!string.IsNullOrEmpty(theDefaultsDirectory))
        {
            Load(ResourcePath(theDefaultsDirectory, theName), myRefMap);
        }
        else if (myVerbose)
        {
            Console.Out.Write("Resource Manager Warning: aDefaultsDirectory is empty." + '\n');
        }

        if (!string.IsNullOrEmpty(theUserDefaultsDirectory))
        {
            // OCCT: the user file goes into the reference map too
            Load(ResourcePath(theUserDefaultsDirectory, theName), myRefMap);
        }
        else if (myVerbose)
        {
            Console.Out.Write("Resource Manager Warning: anUserDefaultsDirectory is empty." + '\n');
        }
    }

    /// <summary>
    /// Creates a resource manager for the resources named <paramref name="aName"/>: OCCT's own resource file of that
    /// name, compiled in (<see cref="EmbeddedDefaults"/>), or none. OCCT reads the files the environment variables
    /// <c>CSF_aNameDefaults</c> and <c>CSF_aNameUserDefaults</c> name; the port reads no environment variable.
    /// </summary>
    public Resource_Manager(string aName, bool Verbose = false)
    {
        myName = aName;
        myVerbose = Verbose;
        myInitialized = false;
        myRefMap = new NCollection_DataMap<string, string>(StringComparer.Ordinal);
        myUserMap = new NCollection_DataMap<string, string>(StringComparer.Ordinal);

        string? aText = EmbeddedDefaults(aName);
        if (aText is not null)
        {
            LoadText(aText, aName, myRefMap);
        }
        else if (myVerbose)
        {
            Console.Out.Write("Resource Manager Warning: no resource file \"" + aName + "\" compiled in." + '\n');
        }
    }

    /// <summary>Creates an empty resource manager.</summary>
    public Resource_Manager()
    {
        myName = "";
        myVerbose = false;
        myInitialized = false;
        myRefMap = new NCollection_DataMap<string, string>(StringComparer.Ordinal);
        myUserMap = new NCollection_DataMap<string, string>(StringComparer.Ordinal);
    }

    /// <summary>A copy of <paramref name="theOther"/>, with copies of its resources (OCCT's implicit copy constructor).</summary>
    public Resource_Manager(Resource_Manager theOther)
    {
        myName = theOther.myName;
        myVerbose = theOther.myVerbose;
        myInitialized = theOther.myInitialized;
        myRefMap = new NCollection_DataMap<string, string>(theOther.myRefMap);
        myUserMap = new NCollection_DataMap<string, string>(theOther.myUserMap);
    }

    /// <summary>
    /// The text of OCCT's resource file <paramref name="theName"/> as compiled into the port, or null: <c>IGES</c> and
    /// <c>STEP</c> (resources/XSTEPResource).
    /// </summary>
    public static string? EmbeddedDefaults(string theName) => theName switch
    {
        "IGES" => XSTEPResource_IGES.Text,
        "STEP" => XSTEPResource_STEP.Text,
        _ => null,
    };

    /// <summary>
    /// The path of the resource file <paramref name="theName"/> in <paramref name="theDirectory"/> (OCCT's
    /// <c>GetResourcePath</c> composes it so, in the directory an environment variable names).
    /// </summary>
    public static string ResourcePath(string theDirectory, string theName) => Path.Combine(theDirectory, theName);

    private void Load(string thePath, NCollection_DataMap<string, string> aMap)
    {
        string FileName = Path.GetFileName(thePath);
        byte[] aBytes;
        try
        {
            aBytes = File.ReadAllBytes(thePath);
        }
        catch (Exception anException) when (anException is IOException or UnauthorizedAccessException or ArgumentException
                                                or NotSupportedException)
        {
            if (myVerbose)
            {
                Console.Out.Write("Resource Manager Warning: Cannot read file \"" + FileName
                                  + "\". File not found or permission denied." + '\n');
            }
            return;
        }

        // OCCT reads bytes: one character per byte
        LoadText(Encoding.Latin1.GetString(aBytes), FileName, aMap);
    }

    private void LoadText(string theText, string FileName, NCollection_DataMap<string, string> aMap)
    {
        myInitialized = true;
        int LineNumber = 1;
        int aPosition = 0;
        Resource_KindOfLine aKind;
        while ((aKind = WhatKindOfLine(theText, ref aPosition, out string Token1, out string Token2)) != Resource_KindOfLine.Resource_KOL_End)
        {
            switch (aKind)
            {
                case Resource_KindOfLine.Resource_KOL_End:
                case Resource_KindOfLine.Resource_KOL_Comment:
                case Resource_KindOfLine.Resource_KOL_Empty:
                    break;
                case Resource_KindOfLine.Resource_KOL_Resource:
                    aMap.Bind(Token1, Token2);
                    break;
                case Resource_KindOfLine.Resource_KOL_Error:
                    if (myVerbose)
                    {
                        Console.Out.Write("Resource Manager: Syntax error at line " + LineNumber + " in file : " + FileName + '\n');
                    }
                    break;
            }
            LineNumber++;
        }
        if (myVerbose)
        {
            Console.Out.Write("Resource Manager: " + (ReferenceEquals(aMap, myUserMap) ? "User" : "Reference") + " file \""
                              + FileName + "\" loaded" + '\n');
        }
    }

    // Auxiliary enumeration for function WhatKindOfLine().
    private enum Resource_KindOfLine
    {
        Resource_KOL_End,
        Resource_KOL_Empty,
        Resource_KOL_Comment,
        Resource_KOL_Resource,
        Resource_KOL_Error,
    }

    // The next line of the text from thePosition, always ending with '\n' (GetLine); null at the end.
    private static string? GetLine(string theText, ref int thePosition)
    {
        if (thePosition >= theText.Length)
        {
            return null;
        }

        int anEnd = theText.IndexOf('\n', thePosition);
        string aLine;
        if (anEnd < 0)
        {
            aLine = theText.Substring(thePosition) + "\n";
            thePosition = theText.Length;
        }
        else
        {
            aLine = theText.Substring(thePosition, anEnd + 1 - thePosition);
            thePosition = anEnd + 1;
        }
        return aLine;
    }

    // OCCT's WhatKindOfLine on a line that ends with '\n' (1-based positions as TCollection_AsciiString's).
    private static Resource_KindOfLine WhatKindOfLine(string theText, ref int thePosition, out string aToken1, out string aToken2)
    {
        aToken1 = "";
        aToken2 = "";
        string? Line = GetLine(theText, ref thePosition);
        if (Line is null)
        {
            return Resource_KindOfLine.Resource_KOL_End;
        }

        char Value(int theIndex) => Line[theIndex - 1];
        int FirstLocationNotInSet(int theFrom)
        {
            for (int i = theFrom; i <= Line.Length; i++)
            {
                if (Line[i - 1] != ' ' && Line[i - 1] != '\t')
                {
                    return i;
                }
            }
            return 0;
        }

        if (Value(1) == '!')
        {
            return Resource_KindOfLine.Resource_KOL_Comment;
        }

        int Pos1 = FirstLocationNotInSet(1);
        if (Value(Pos1) == '\n')
        {
            return Resource_KindOfLine.Resource_KOL_Empty;
        }

        int Pos2 = Line.IndexOf(':', Pos1 - 1) + 1;
        if (Pos2 == 0 || Pos1 == Pos2)
        {
            return Resource_KindOfLine.Resource_KOL_Error;
        }

        int Pos;
        for (Pos = Pos2 - 1; Value(Pos) == '\t' || Value(Pos) == ' '; Pos--)
        {
        }
        aToken1 = Line.Substring(Pos1 - 1, Pos - Pos1 + 1);

        Pos = FirstLocationNotInSet(Pos2 + 1);
        if (Pos != 0)
        {
            if (Value(Pos) == '\\')
            {
                switch (Value(Pos + 1))
                {
                    case '\\':
                    case ' ':
                    case '\t':
                        Pos++;
                        break;
                }
            }
        }
        if (Pos == Line.Length)
        {
            aToken2 = "";
        }
        else
        {
            Line = Line.Remove(0, Pos - 1);
            int aLineLength = Line.Length;
            if (aLineLength >= 2)
            {
                if (Line[aLineLength - 2] == '\r')
                {
                    Line = Line.Remove(aLineLength - 2, 1);
                }
            }
            Line = Line.Remove(Line.Length - 1);
            aToken2 = Line;
        }
        return Resource_KindOfLine.Resource_KOL_Resource;
    }

    /// <summary>
    /// Sorts the user resources (those set with <see cref="SetResource(string, string)"/>) and saves them in the file
    /// named after the manager in <paramref name="theUserDefaultsDirectory"/>, creating the directory if needed. False
    /// if the directory is empty or the file can't be written. OCCT saves into the directory
    /// <c>CSF_&lt;name&gt;UserDefaults</c> names.
    /// </summary>
    public bool Save(string? theUserDefaultsDirectory)
    {
        if (string.IsNullOrEmpty(theUserDefaultsDirectory))
        {
            if (myVerbose)
            {
                Console.Out.Write("Resource Manager Warning: no directory given.  Cannot save resources." + '\n');
            }
            return false;
        }

        string aFilePath = ResourcePath(theUserDefaultsDirectory, myName);
        try
        {
            Directory.CreateDirectory(theUserDefaultsDirectory);
        }
        catch (Exception anException) when (anException is IOException or UnauthorizedAccessException or ArgumentException
                                                or NotSupportedException)
        {
            if (myVerbose)
            {
                Console.Out.Write("Resource Manager: Error opening or creating directory \"" + theUserDefaultsDirectory
                                  + "\". Permission denied. Cannot save resources." + '\n');
            }
            return false;
        }

        var aText = new StringBuilder();
        int NbKey = myUserMap.Extent();
        if (NbKey != 0)
        {
            var KeyArray = new string[NbKey];
            int Index = 0;
            foreach (KeyValuePair<string, string> Iter in myUserMap)
            {
                KeyArray[Index++] = Iter.Key;
            }

            // std::sort of TCollection_AsciiString: byte order
            Array.Sort(KeyArray, StringComparer.Ordinal);

            for (Index = 0; Index < NbKey; Index++)
            {
                string Value = myUserMap.Find(KeyArray[Index]);
                if (Value.Length != 0)
                {
                    switch (Value[0])
                    {
                        case '\\':
                        case ' ':
                        case '\t':
                            Value = Value.Insert(0, "\\");
                            break;
                    }
                }
                aText.Append(KeyArray[Index]).Append(":\t").Append(Value).Append('\n');
            }
        }

        try
        {
            File.WriteAllBytes(aFilePath, Encoding.Latin1.GetBytes(aText.ToString()));
        }
        catch (Exception anException) when (anException is IOException or UnauthorizedAccessException or ArgumentException
                                                or NotSupportedException)
        {
            if (myVerbose)
            {
                Console.Out.Write("Resource Manager: Error opening or creating file \"" + aFilePath
                                  + "\". Permission denied. Cannot save resources." + '\n');
            }
            return false;
        }
        if (myVerbose && NbKey != 0)
        {
            Console.Out.Write("Resource Manager: Resources saved in file " + aFilePath + '\n');
        }
        return true;
    }

    /// <summary>True if the resource is defined.</summary>
    public bool Find(string aResource) => myUserMap.IsBound(aResource) || myRefMap.IsBound(aResource);

    /// <summary>True if the resource is defined, and its value.</summary>
    public bool Find(string theResource, out string theValue)
    {
        if (myUserMap.Find(theResource, out theValue!))
        {
            return true;
        }
        return myRefMap.Find(theResource, out theValue!);
    }

    /// <summary>The value of an integer resource; Standard_TypeMismatch if it isn't an integer.</summary>
    public virtual int Integer(string aResourceName)
    {
        string Result = Value(aResourceName);
        if (!TCollection_AsciiString.IsIntegerValue(Result))
        {
            throw new Standard_TypeMismatch("Value of resource `" + aResourceName + "` is not an integer");
        }
        return TCollection_AsciiString.IntegerValue(Result);
    }

    /// <summary>The value of a real resource; Standard_TypeMismatch if it isn't a real.</summary>
    public virtual double Real(string aResourceName)
    {
        string Result = Value(aResourceName);
        if (!TCollection_AsciiString.IsRealValue(Result))
        {
            throw new Standard_TypeMismatch("Value of resource `" + aResourceName + "` is not a real");
        }
        return TCollection_AsciiString.RealValue(Result);
    }

    /// <summary>The value of a resource (the user's value first); Resource_NoSuchResource if it isn't defined.</summary>
    public virtual string Value(string aResource)
    {
        if (myUserMap.IsBound(aResource))
        {
            return myUserMap.Find(aResource);
        }
        if (myRefMap.IsBound(aResource))
        {
            return myRefMap.Find(aResource);
        }
        throw new Resource_NoSuchResource(aResource);
    }

    /// <summary>Sets the value of an integer resource, creating it if needed.</summary>
    public virtual void SetResource(string aResourceName, int aValue)
    {
        SetResource(aResourceName, aValue.ToString(System.Globalization.CultureInfo.InvariantCulture));
    }

    /// <summary>Sets the value of a real resource (as C's <c>%g</c> writes it), creating it if needed.</summary>
    public virtual void SetResource(string aResourceName, double aValue)
    {
        SetResource(aResourceName, Standard_Stream.FormatReal(aValue, 6));
    }

    /// <summary>Sets the value of a resource, creating it if needed.</summary>
    public virtual void SetResource(string aResource, string aValue)
    {
        myUserMap.Bind(aResource, aValue);
    }

    /// <summary>
    /// The reference resources (read from the files), or with <paramref name="theRefMap"/> false the user's (set with
    /// SetResource); the map itself, which the caller may change.
    /// </summary>
    public NCollection_DataMap<string, string> GetMap(bool theRefMap = true) => theRefMap ? myRefMap : myUserMap;

    /// <summary>True if a resource file was read.</summary>
    public bool IsInitialized() => myInitialized;
}
