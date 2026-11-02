// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_FileName.hxx
// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_FileName.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The FILE_NAME entity of a STEP file's header.</summary>
public class HeaderSection_FileName : Standard_Transient
{
    private TCollection_HAsciiString name;
    private TCollection_HAsciiString timeStamp;
    private NCollection_HArray1<TCollection_HAsciiString> author;
    private NCollection_HArray1<TCollection_HAsciiString> organization;
    private TCollection_HAsciiString preprocessorVersion;
    private TCollection_HAsciiString originatingSystem;
    private TCollection_HAsciiString authorisation;

    /// <summary>Returns a FileName.</summary>
    public HeaderSection_FileName()
    {
    }

    public void Init(TCollection_HAsciiString? aName, TCollection_HAsciiString? aTimeStamp, NCollection_HArray1<TCollection_HAsciiString>? aAuthor,
                     NCollection_HArray1<TCollection_HAsciiString>? aOrganization, TCollection_HAsciiString? aPreprocessorVersion,
                     TCollection_HAsciiString? aOriginatingSystem, TCollection_HAsciiString? aAuthorisation)
    {
        // --- class own fields ---
        name = aName;
        timeStamp = aTimeStamp;
        author = aAuthor;
        organization = aOrganization;
        preprocessorVersion = aPreprocessorVersion;
        originatingSystem = aOriginatingSystem;
        authorisation = aAuthorisation;
    }

    public void SetName(TCollection_HAsciiString? aName)
    {
        name = aName;
    }

    public TCollection_HAsciiString? Name()
    {
        return name;
    }

    public void SetTimeStamp(TCollection_HAsciiString? aTimeStamp)
    {
        timeStamp = aTimeStamp;
    }

    public TCollection_HAsciiString? TimeStamp()
    {
        return timeStamp;
    }

    public void SetAuthor(NCollection_HArray1<TCollection_HAsciiString>? aAuthor)
    {
        author = aAuthor;
    }

    public NCollection_HArray1<TCollection_HAsciiString>? Author()
    {
        return author;
    }

    public TCollection_HAsciiString? AuthorValue(int num)
    {
        return author.Value(num);
    }

    public int NbAuthor()
    {
        if (author is null)
        {
            return 0;
        }
        return author.Length();
    }

    public void SetOrganization(NCollection_HArray1<TCollection_HAsciiString>? aOrganization)
    {
        organization = aOrganization;
    }

    public NCollection_HArray1<TCollection_HAsciiString>? Organization()
    {
        return organization;
    }

    public TCollection_HAsciiString? OrganizationValue(int num)
    {
        return organization.Value(num);
    }

    public int NbOrganization()
    {
        if (organization is null)
        {
            return 0;
        }
        return organization.Length();
    }

    public void SetPreprocessorVersion(TCollection_HAsciiString? aPreprocessorVersion)
    {
        preprocessorVersion = aPreprocessorVersion;
    }

    public TCollection_HAsciiString? PreprocessorVersion()
    {
        return preprocessorVersion;
    }

    public void SetOriginatingSystem(TCollection_HAsciiString? aOriginatingSystem)
    {
        originatingSystem = aOriginatingSystem;
    }

    public TCollection_HAsciiString? OriginatingSystem()
    {
        return originatingSystem;
    }

    public void SetAuthorisation(TCollection_HAsciiString? aAuthorisation)
    {
        authorisation = aAuthorisation;
    }

    public TCollection_HAsciiString? Authorisation()
    {
        return authorisation;
    }
}
