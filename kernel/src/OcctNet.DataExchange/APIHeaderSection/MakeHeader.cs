// Port of: src/DataExchange/TKDESTEP/APIHeaderSection/APIHeaderSection_MakeHeader.hxx
// Port of: src/DataExchange/TKDESTEP/APIHeaderSection/APIHeaderSection_MakeHeader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;

using Kerf.Engine.Exchange.Step.Data;

using Occt;

namespace Kerf.Engine.Exchange.Step.Header;

/// <summary>This class allows to consult and prepare/edit data stored in a Step Model Header.</summary>
/// <remarks>A C++ value class, a C# class here (decision 1185).</remarks>
public class MakeHeader
{
    // #58 rln 28.12.98 STEP header fields (NOTE: more parameterization is necessary)
    // This is a generic header for any STEP scheme

    private bool _done;
    private FileName _fn;
    private FileSchema _fs;
    private FileDescription _fd;

    /// <summary>Prepares a MakeHeader from the content of a StepModel. See IsDone to know if the Header is well defined.</summary>
    public MakeHeader(StepModel model)
    {
        _done = true;
        if (model.HasHeaderEntity(typeof(FileName)))
        {
            _fn = model.HeaderEntity(typeof(FileName)) as FileName;
        }
        else
        {
            _done = false;
        }
        if (model.HasHeaderEntity(typeof(FileSchema)))
        {
            _fs = model.HeaderEntity(typeof(FileSchema)) as FileSchema;
        }
        else
        {
            _done = false;
        }
        if (model.HasHeaderEntity(typeof(FileDescription)))
        {
            _fd = model.HeaderEntity(typeof(FileDescription)) as FileDescription;
        }
        else
        {
            _done = false;
        }
    }

    /// <summary>Prepares a new MakeHeader from scratch.</summary>
    public MakeHeader(int shapetype = 0)
    {
        switch (shapetype)
        {
            case 1:
                Init("Open CASCADE Facetted BRep Model");
                break;
            case 2:
                Init("Open CASCADE Face Based Surface Model");
                break;
            case 3:
                Init("Open CASCADE Shell Based Surface Model");
                break;
            case 4:
                Init("Open CASCADE Manifold Solid Brep Model");
                break;
            default:
                Init("Open CASCADE Shape Model");
                break;
        }
    }

    /// <summary>Cancels the former definition and gives a FileName. To be used when a Model has no well defined Header.</summary>
    public void Init(string nameval)
    {
        _done = true;

        // - File Name
        if (_fn is null)
        {
            _fn = new FileName();
        }
        TCollection_HAsciiString name = new TCollection_HAsciiString(nameval);
        _fn.SetName(name);
        FrameworkMsg.TDate(out string timestamp, 0, 0, 0, 0, 0, 1, "C:%4.4d-%2.2d-%2.2dT%2.2d:%2.2d:%2.2d"); // actually
        TCollection_HAsciiString tst = new TCollection_HAsciiString(timestamp);
        _fn.SetTimeStamp(tst);
        NCollection_HArray1<TCollection_HAsciiString> authors = new NCollection_HArray1<TCollection_HAsciiString>(1, 1);
        TCollection_HAsciiString a1 = new TCollection_HAsciiString("Author");
        authors.SetValue(1, a1);
        _fn.SetAuthor(authors);
        NCollection_HArray1<TCollection_HAsciiString> org = new NCollection_HArray1<TCollection_HAsciiString>(1, 1);
        TCollection_HAsciiString org1 = new TCollection_HAsciiString("Open CASCADE");
        org.SetValue(1, org1);
        _fn.SetOrganization(org);

        string procver = global::Kerf.Engine.Foundation.Standard.Sprintf(InterfaceVersion.XSTEP_PROCESSOR_VERSION, "STEP");
        TCollection_HAsciiString pv = new TCollection_HAsciiString(procver);
        _fn.SetPreprocessorVersion(pv);

        TCollection_HAsciiString sys = new TCollection_HAsciiString(InterfaceVersion.XSTEP_SYSTEM_VERSION); // #58 rln
        _fn.SetOriginatingSystem(sys);
        TCollection_HAsciiString auth = new TCollection_HAsciiString("Unknown");
        _fn.SetAuthorisation(auth);

        // - File Description
        if (_fd is null)
        {
            _fd = new FileDescription();
        }
        NCollection_HArray1<TCollection_HAsciiString> descr = new NCollection_HArray1<TCollection_HAsciiString>(1, 1);
        TCollection_HAsciiString descr1 = new TCollection_HAsciiString("Open CASCADE Model");
        descr.SetValue(1, descr1);
        _fd.SetDescription(descr);
        TCollection_HAsciiString il = new TCollection_HAsciiString("2;1");
        _fd.SetImplementationLevel(il);

        // - File Schema
        if (_fs is null)
        {
            _fs = new FileSchema();
        }
        NCollection_HArray1<TCollection_HAsciiString> schid = new NCollection_HArray1<TCollection_HAsciiString>(1, 1);
        TCollection_HAsciiString schid1 = new TCollection_HAsciiString("");
        schid.SetValue(1, schid1);
        _fs.SetSchemaIdentifiers(schid);
    }

    /// <summary>Returns True if all data have been defined (see also HasFn, HasFs, HasFd).</summary>
    public bool IsDone
    {
        get
        {
            return _done;
        }
    }

    /// <summary>Creates an empty header for a new STEP model and allows the header fields to be completed.</summary>
    public void Apply(StepModel model)
    {
        EntityIterator header = model.Header();
        if (HasFd && !model.HasHeaderEntity(typeof(FileDescription)))
        {
            header.AddItem(_fd);
        }
        if (HasFn && !model.HasHeaderEntity(typeof(FileName)))
        {
            header.AddItem(_fn);
        }
        if (HasFs && !model.HasHeaderEntity(typeof(FileSchema)))
        {
            // Schema defined? If not take it from the protocole
            TCollection_HAsciiString sch = null;
            NCollection_HArray1<TCollection_HAsciiString> schid = _fs.SchemaIdentifiers();
            if (schid is not null)
            {
                sch = schid.Value(1);
            }
            else
            {
                schid = new NCollection_HArray1<TCollection_HAsciiString>(1, 1);
                _fs.SetSchemaIdentifiers(schid);
            }
            if (sch is not null)
            {
                if (sch.Length() < 2)
                {
                    sch = null;
                }
            } // not defined
            if (sch is null)
            {
                if (model.Protocol() is StepProtocol stepro)
                {
                    sch = new TCollection_HAsciiString(stepro.SchemaName(model));
                }
                if (sch is not null)
                {
                    schid.SetValue(1, sch);
                }
            }
            header.AddItem(_fs);
        }
        model.ClearHeader();
        for (header.Start(); header.More(); header.Next())
        {
            model.AddHeaderEntity(header.Value());
        }
    }

    /// <summary>
    /// Builds a Header, creates a new StepModel, then applies the Header to the StepModel. The Schema Name is taken
    /// from the Protocol (if it inherits from StepData, else it is left in blanks).
    /// </summary>
    public StepModel NewModel(Protocol? protocol)
    {
        StepModel stepmodel = new StepModel();
        stepmodel.SetProtocol(protocol);

        // - Make Header information
        Apply(stepmodel);
        return stepmodel;
    }

    //   ########        Individual Queries / Actions        ########

    // ========
    // FileName
    // ========

    /// <summary>Checks whether there is a file_name entity. Returns True if there is one.</summary>
    public bool HasFn
    {
        get
        {
            return (_fn is not null);
        }
    }

    /// <summary>Returns the file_name entity. Returns an empty entity if the file_name entity is not initialized.</summary>
    public FileName? FnValue()
    {
        return _fn;
    }

    public void SetName(TCollection_HAsciiString? aName)
    {
        if (_fn is not null)
        {
            _fn.SetName(aName);
        }
    }

    /// <summary>Returns the name attribute for the file_name entity.</summary>
    public TCollection_HAsciiString? Name()
    {
        return (_fn is null ? null : _fn.Name());
    }

    public void SetTimeStamp(TCollection_HAsciiString? aTimeStamp)
    {
        if (_fn is not null)
        {
            _fn.SetTimeStamp(aTimeStamp);
        }
    }

    /// <summary>Returns the value of the time_stamp attribute for the file_name entity.</summary>
    public TCollection_HAsciiString? TimeStamp()
    {
        return (_fn is null ? null : _fn.TimeStamp());
    }

    public void SetAuthor(NCollection_HArray1<TCollection_HAsciiString>? aAuthor)
    {
        if (_fn is not null)
        {
            _fn.SetAuthor(aAuthor);
        }
    }

    public void SetAuthorValue(int num, TCollection_HAsciiString? aAuthor)
    {
        if (_fn is null)
        {
            return;
        }
        NCollection_HArray1<TCollection_HAsciiString> li = _fn.Author();
        if (num >= li.Lower() && num <= li.Upper())
        {
            li.SetValue(num, aAuthor);
        }
    }

    public NCollection_HArray1<TCollection_HAsciiString>? Author()
    {
        return (_fn is null ? null : _fn.Author());
    }

    /// <summary>Returns the value of the name attribute for the file_name entity.</summary>
    public TCollection_HAsciiString? AuthorValue(int num)
    {
        return (_fn is null ? null : _fn.AuthorValue(num));
    }

    /// <summary>Returns the number of values for the author attribute in the file_name entity.</summary>
    public int AuthorCount
    {
        get
        {
            return (_fn is null ? 0 : _fn.NbAuthor());
        }
    }

    public void SetOrganization(NCollection_HArray1<TCollection_HAsciiString>? aOrganization)
    {
        if (_fn is not null)
        {
            _fn.SetOrganization(aOrganization);
        }
    }

    public void SetOrganizationValue(int num, TCollection_HAsciiString? aOrgan)
    {
        if (_fn is null)
        {
            return;
        }
        NCollection_HArray1<TCollection_HAsciiString> li = _fn.Organization();
        if (num >= li.Lower() && num <= li.Upper())
        {
            li.SetValue(num, aOrgan);
        }
    }

    public NCollection_HArray1<TCollection_HAsciiString>? Organization()
    {
        return (_fn is null ? null : _fn.Organization());
    }

    /// <summary>Returns the value of attribute organization for the file_name entity.</summary>
    public TCollection_HAsciiString? OrganizationValue(int num)
    {
        return (_fn is null ? null : _fn.OrganizationValue(num));
    }

    /// <summary>Returns the number of values for the organization attribute in the file_name entity.</summary>
    public int OrganizationCount
    {
        get
        {
            return (_fn is null ? 0 : _fn.NbOrganization());
        }
    }

    public void SetPreprocessorVersion(TCollection_HAsciiString? aPreprocessorVersion)
    {
        if (_fn is not null)
        {
            _fn.SetPreprocessorVersion(aPreprocessorVersion);
        }
    }

    /// <summary>Returns the name of the preprocessor_version for the file_name entity.</summary>
    public TCollection_HAsciiString? PreprocessorVersion()
    {
        return (_fn is null ? null : _fn.PreprocessorVersion());
    }

    public void SetOriginatingSystem(TCollection_HAsciiString? aOriginatingSystem)
    {
        if (_fn is not null)
        {
            _fn.SetOriginatingSystem(aOriginatingSystem);
        }
    }

    public TCollection_HAsciiString? OriginatingSystem()
    {
        return (_fn is null ? null : _fn.OriginatingSystem());
    }

    public void SetAuthorisation(TCollection_HAsciiString? aAuthorisation)
    {
        if (_fn is not null)
        {
            _fn.SetAuthorisation(aAuthorisation);
        }
    }

    /// <summary>Returns the value of the authorization attribute for the file_name entity.</summary>
    public TCollection_HAsciiString? Authorisation()
    {
        return (_fn is null ? null : _fn.Authorisation());
    }

    // ===========
    // File Schema
    // ===========

    /// <summary>Checks whether there is a file_schema entity. Returns True if there is one.</summary>
    public bool HasFs
    {
        get
        {
            return (_fs is not null);
        }
    }

    /// <summary>Returns the file_schema entity. Returns an empty entity if the file_schema entity is not initialized.</summary>
    public FileSchema? FsValue()
    {
        return _fs;
    }

    public void SetSchemaIdentifiers(NCollection_HArray1<TCollection_HAsciiString>? aSchemaIdentifiers)
    {
        if (_fs is not null)
        {
            _fs.SetSchemaIdentifiers(aSchemaIdentifiers);
        }
    }

    public void SetSchemaIdentifiersValue(int num, TCollection_HAsciiString? aSchem)
    {
        if (_fs is null)
        {
            return;
        }
        NCollection_HArray1<TCollection_HAsciiString> li = _fs.SchemaIdentifiers();
        if (num >= li.Lower() && num <= li.Upper())
        {
            li.SetValue(num, aSchem);
        }
    }

    public NCollection_HArray1<TCollection_HAsciiString>? SchemaIdentifiers()
    {
        return (_fs is null ? null : _fs.SchemaIdentifiers());
    }

    /// <summary>Returns the value of the schema_identifier attribute for the file_schema entity.</summary>
    public TCollection_HAsciiString? SchemaIdentifiersValue(int num)
    {
        return (_fs is null ? null : _fs.SchemaIdentifiersValue(num));
    }

    /// <summary>Returns the number of values for the schema_identifier attribute in the file_schema entity.</summary>
    public int SchemaIdentifierCount
    {
        get
        {
            return (_fs is null ? 0 : _fs.NbSchemaIdentifiers());
        }
    }

    /// <summary>Add a subname of schema (if not yet in the list).</summary>
    public void AddSchemaIdentifier(TCollection_HAsciiString aSchem)
    {
        if (_fs is null)
        {
            _fs = new FileSchema();
        }
        NCollection_HArray1<TCollection_HAsciiString> idents = _fs.SchemaIdentifiers();

        // check that requested subschema is already in the list
        int i;
        for (i = 1; idents is not null && i <= idents.Length(); i++)
        {
            if (aSchem.IsSameString(idents.Value(i)))
            {
                return;
            }
        }

        // add a subshema
        NCollection_HArray1<TCollection_HAsciiString> ids = new NCollection_HArray1<TCollection_HAsciiString>(1, (idents is null ? 1 : idents.Length() + 1));
        for (i = 1; idents is not null && i <= idents.Length(); i++)
        {
            ids.SetValue(i, idents.Value(i));
        }
        ids.SetValue(i, aSchem);
        _fs.SetSchemaIdentifiers(ids);
    }

    // ================
    // File Description
    // ================

    /// <summary>Checks whether there is a file_description entity. Returns True if there is one.</summary>
    public bool HasFd
    {
        get
        {
            return (_fd is not null);
        }
    }

    /// <summary>Returns the file_description entity. Returns an empty entity if the file_description entity is not initialized.</summary>
    public FileDescription? FdValue()
    {
        return _fd;
    }

    public void SetDescription(NCollection_HArray1<TCollection_HAsciiString>? aDescription)
    {
        if (_fs is not null) // OCCT: tests fs, then sets fd
        {
            _fd.SetDescription(aDescription);
        }
    }

    public void SetDescriptionValue(int num, TCollection_HAsciiString? aDescr)
    {
        if (_fd is null)
        {
            return;
        }
        NCollection_HArray1<TCollection_HAsciiString> li = _fd.Description();
        if (num >= li.Lower() && num <= li.Upper())
        {
            li.SetValue(num, aDescr);
        }
    }

    public NCollection_HArray1<TCollection_HAsciiString>? Description()
    {
        return (_fd is null ? null : _fd.Description());
    }

    /// <summary>Returns the value of the description attribute for the file_description entity.</summary>
    public TCollection_HAsciiString? DescriptionValue(int num)
    {
        return (_fd is null ? null : _fd.DescriptionValue(num));
    }

    /// <summary>Returns the number of values for the file_description entity in the STEP file header.</summary>
    public int DescriptionCount
    {
        get
        {
            return (_fd is null ? 0 : _fd.NbDescription());
        }
    }

    public void SetImplementationLevel(TCollection_HAsciiString? aImplementationLevel)
    {
        if (_fd is not null)
        {
            _fd.SetImplementationLevel(aImplementationLevel);
        }
    }

    /// <summary>Returns the value of the implementation_level attribute for the file_description entity.</summary>
    public TCollection_HAsciiString? ImplementationLevel()
    {
        return (_fd is null ? null : _fd.ImplementationLevel());
    }
}
