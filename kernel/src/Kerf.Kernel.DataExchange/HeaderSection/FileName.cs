// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_FileName.hxx
// Port of: src/DataExchange/TKDESTEP/HeaderSection/HeaderSection_FileName.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Header;

/// <summary>The FILE_NAME entity of a STEP file's header.</summary>
internal class FileName : Transient
{
    private TCollection_HAsciiString _name;
    private TCollection_HAsciiString _timeStamp;
    private NCollection_HArray1<TCollection_HAsciiString> _author;
    private NCollection_HArray1<TCollection_HAsciiString> _organization;
    private TCollection_HAsciiString _preprocessorVersion;
    private TCollection_HAsciiString _originatingSystem;
    private TCollection_HAsciiString _authorisation;

    /// <summary>Returns a FileName.</summary>
    public FileName()
    {
    }

    public void Init(TCollection_HAsciiString? aName, TCollection_HAsciiString? aTimeStamp, NCollection_HArray1<TCollection_HAsciiString>? aAuthor,
                     NCollection_HArray1<TCollection_HAsciiString>? aOrganization, TCollection_HAsciiString? aPreprocessorVersion,
                     TCollection_HAsciiString? aOriginatingSystem, TCollection_HAsciiString? aAuthorisation)
    {
        // --- class own fields ---
        _name = aName;
        _timeStamp = aTimeStamp;
        _author = aAuthor;
        _organization = aOrganization;
        _preprocessorVersion = aPreprocessorVersion;
        _originatingSystem = aOriginatingSystem;
        _authorisation = aAuthorisation;
    }

    public void SetName(TCollection_HAsciiString? aName)
    {
        _name = aName;
    }

    public TCollection_HAsciiString? Name()
    {
        return _name;
    }

    public void SetTimeStamp(TCollection_HAsciiString? aTimeStamp)
    {
        _timeStamp = aTimeStamp;
    }

    public TCollection_HAsciiString? TimeStamp()
    {
        return _timeStamp;
    }

    public void SetAuthor(NCollection_HArray1<TCollection_HAsciiString>? aAuthor)
    {
        _author = aAuthor;
    }

    public NCollection_HArray1<TCollection_HAsciiString>? Author()
    {
        return _author;
    }

    public TCollection_HAsciiString? AuthorValue(int num)
    {
        return _author.Value(num);
    }

    public int NbAuthor()
    {
        if (_author is null)
        {
            return 0;
        }
        return _author.Length();
    }

    public void SetOrganization(NCollection_HArray1<TCollection_HAsciiString>? aOrganization)
    {
        _organization = aOrganization;
    }

    public NCollection_HArray1<TCollection_HAsciiString>? Organization()
    {
        return _organization;
    }

    public TCollection_HAsciiString? OrganizationValue(int num)
    {
        return _organization.Value(num);
    }

    public int NbOrganization()
    {
        if (_organization is null)
        {
            return 0;
        }
        return _organization.Length();
    }

    public void SetPreprocessorVersion(TCollection_HAsciiString? aPreprocessorVersion)
    {
        _preprocessorVersion = aPreprocessorVersion;
    }

    public TCollection_HAsciiString? PreprocessorVersion()
    {
        return _preprocessorVersion;
    }

    public void SetOriginatingSystem(TCollection_HAsciiString? aOriginatingSystem)
    {
        _originatingSystem = aOriginatingSystem;
    }

    public TCollection_HAsciiString? OriginatingSystem()
    {
        return _originatingSystem;
    }

    public void SetAuthorisation(TCollection_HAsciiString? aAuthorisation)
    {
        _authorisation = aAuthorisation;
    }

    public TCollection_HAsciiString? Authorisation()
    {
        return _authorisation;
    }
}
