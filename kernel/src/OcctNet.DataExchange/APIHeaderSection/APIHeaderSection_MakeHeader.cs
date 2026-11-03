// Port of: src/DataExchange/TKDESTEP/APIHeaderSection/APIHeaderSection_MakeHeader.hxx
// Port of: src/DataExchange/TKDESTEP/APIHeaderSection/APIHeaderSection_MakeHeader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>This class allows to consult and prepare/edit data stored in a Step Model Header.</summary>
/// <remarks>A C++ value class, a C# class here (decision 1185).</remarks>
public class APIHeaderSection_MakeHeader
{
    // #58 rln 28.12.98 STEP header fields (NOTE: more parameterization is necessary)
    // This is a generic header for any STEP scheme

    private bool done;
    private HeaderSection_FileName fn;
    private HeaderSection_FileSchema fs;
    private HeaderSection_FileDescription fd;

    /// <summary>Prepares a MakeHeader from the content of a StepModel. See IsDone to know if the Header is well defined.</summary>
    public APIHeaderSection_MakeHeader(StepData_StepModel model)
    {
        done = true;
        if (model.HasHeaderEntity(typeof(HeaderSection_FileName)))
        {
            fn = model.HeaderEntity(typeof(HeaderSection_FileName)) as HeaderSection_FileName;
        }
        else
        {
            done = false;
        }
        if (model.HasHeaderEntity(typeof(HeaderSection_FileSchema)))
        {
            fs = model.HeaderEntity(typeof(HeaderSection_FileSchema)) as HeaderSection_FileSchema;
        }
        else
        {
            done = false;
        }
        if (model.HasHeaderEntity(typeof(HeaderSection_FileDescription)))
        {
            fd = model.HeaderEntity(typeof(HeaderSection_FileDescription)) as HeaderSection_FileDescription;
        }
        else
        {
            done = false;
        }
    }

    /// <summary>Prepares a new MakeHeader from scratch.</summary>
    public APIHeaderSection_MakeHeader(int shapetype = 0)
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
        done = true;

        // - File Name
        if (fn is null)
        {
            fn = new HeaderSection_FileName();
        }
        TCollection_HAsciiString name = new TCollection_HAsciiString(nameval);
        fn.SetName(name);
        Interface_MSG.TDate(out string timestamp, 0, 0, 0, 0, 0, 1, "C:%4.4d-%2.2d-%2.2dT%2.2d:%2.2d:%2.2d"); // actually
        TCollection_HAsciiString tst = new TCollection_HAsciiString(timestamp);
        fn.SetTimeStamp(tst);
        NCollection_HArray1<TCollection_HAsciiString> authors = new NCollection_HArray1<TCollection_HAsciiString>(1, 1);
        TCollection_HAsciiString a1 = new TCollection_HAsciiString("Author");
        authors.SetValue(1, a1);
        fn.SetAuthor(authors);
        NCollection_HArray1<TCollection_HAsciiString> org = new NCollection_HArray1<TCollection_HAsciiString>(1, 1);
        TCollection_HAsciiString org1 = new TCollection_HAsciiString("Open CASCADE");
        org.SetValue(1, org1);
        fn.SetOrganization(org);

        string procver = global::Kerf.Engine.Foundation.Standard.Sprintf(Interface_Version.XSTEP_PROCESSOR_VERSION, "STEP");
        TCollection_HAsciiString pv = new TCollection_HAsciiString(procver);
        fn.SetPreprocessorVersion(pv);

        TCollection_HAsciiString sys = new TCollection_HAsciiString(Interface_Version.XSTEP_SYSTEM_VERSION); // #58 rln
        fn.SetOriginatingSystem(sys);
        TCollection_HAsciiString auth = new TCollection_HAsciiString("Unknown");
        fn.SetAuthorisation(auth);

        // - File Description
        if (fd is null)
        {
            fd = new HeaderSection_FileDescription();
        }
        NCollection_HArray1<TCollection_HAsciiString> descr = new NCollection_HArray1<TCollection_HAsciiString>(1, 1);
        TCollection_HAsciiString descr1 = new TCollection_HAsciiString("Open CASCADE Model");
        descr.SetValue(1, descr1);
        fd.SetDescription(descr);
        TCollection_HAsciiString il = new TCollection_HAsciiString("2;1");
        fd.SetImplementationLevel(il);

        // - File Schema
        if (fs is null)
        {
            fs = new HeaderSection_FileSchema();
        }
        NCollection_HArray1<TCollection_HAsciiString> schid = new NCollection_HArray1<TCollection_HAsciiString>(1, 1);
        TCollection_HAsciiString schid1 = new TCollection_HAsciiString("");
        schid.SetValue(1, schid1);
        fs.SetSchemaIdentifiers(schid);
    }

    /// <summary>Returns True if all data have been defined (see also HasFn, HasFs, HasFd).</summary>
    public bool IsDone()
    {
        return done;
    }

    /// <summary>Creates an empty header for a new STEP model and allows the header fields to be completed.</summary>
    public void Apply(StepData_StepModel model)
    {
        Interface_EntityIterator header = model.Header();
        if (HasFd() && !model.HasHeaderEntity(typeof(HeaderSection_FileDescription)))
        {
            header.AddItem(fd);
        }
        if (HasFn() && !model.HasHeaderEntity(typeof(HeaderSection_FileName)))
        {
            header.AddItem(fn);
        }
        if (HasFs() && !model.HasHeaderEntity(typeof(HeaderSection_FileSchema)))
        {
            // Schema defined? If not take it from the protocole
            TCollection_HAsciiString sch = null;
            NCollection_HArray1<TCollection_HAsciiString> schid = fs.SchemaIdentifiers();
            if (schid is not null)
            {
                sch = schid.Value(1);
            }
            else
            {
                schid = new NCollection_HArray1<TCollection_HAsciiString>(1, 1);
                fs.SetSchemaIdentifiers(schid);
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
                if (model.Protocol() is StepData_Protocol stepro)
                {
                    sch = new TCollection_HAsciiString(stepro.SchemaName(model));
                }
                if (sch is not null)
                {
                    schid.SetValue(1, sch);
                }
            }
            header.AddItem(fs);
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
    public StepData_StepModel NewModel(Interface_Protocol? protocol)
    {
        StepData_StepModel stepmodel = new StepData_StepModel();
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
    public bool HasFn()
    {
        return (fn is not null);
    }

    /// <summary>Returns the file_name entity. Returns an empty entity if the file_name entity is not initialized.</summary>
    public HeaderSection_FileName? FnValue()
    {
        return fn;
    }

    public void SetName(TCollection_HAsciiString? aName)
    {
        if (fn is not null)
        {
            fn.SetName(aName);
        }
    }

    /// <summary>Returns the name attribute for the file_name entity.</summary>
    public TCollection_HAsciiString? Name()
    {
        return (fn is null ? null : fn.Name());
    }

    public void SetTimeStamp(TCollection_HAsciiString? aTimeStamp)
    {
        if (fn is not null)
        {
            fn.SetTimeStamp(aTimeStamp);
        }
    }

    /// <summary>Returns the value of the time_stamp attribute for the file_name entity.</summary>
    public TCollection_HAsciiString? TimeStamp()
    {
        return (fn is null ? null : fn.TimeStamp());
    }

    public void SetAuthor(NCollection_HArray1<TCollection_HAsciiString>? aAuthor)
    {
        if (fn is not null)
        {
            fn.SetAuthor(aAuthor);
        }
    }

    public void SetAuthorValue(int num, TCollection_HAsciiString? aAuthor)
    {
        if (fn is null)
        {
            return;
        }
        NCollection_HArray1<TCollection_HAsciiString> li = fn.Author();
        if (num >= li.Lower() && num <= li.Upper())
        {
            li.SetValue(num, aAuthor);
        }
    }

    public NCollection_HArray1<TCollection_HAsciiString>? Author()
    {
        return (fn is null ? null : fn.Author());
    }

    /// <summary>Returns the value of the name attribute for the file_name entity.</summary>
    public TCollection_HAsciiString? AuthorValue(int num)
    {
        return (fn is null ? null : fn.AuthorValue(num));
    }

    /// <summary>Returns the number of values for the author attribute in the file_name entity.</summary>
    public int NbAuthor()
    {
        return (fn is null ? 0 : fn.NbAuthor());
    }

    public void SetOrganization(NCollection_HArray1<TCollection_HAsciiString>? aOrganization)
    {
        if (fn is not null)
        {
            fn.SetOrganization(aOrganization);
        }
    }

    public void SetOrganizationValue(int num, TCollection_HAsciiString? aOrgan)
    {
        if (fn is null)
        {
            return;
        }
        NCollection_HArray1<TCollection_HAsciiString> li = fn.Organization();
        if (num >= li.Lower() && num <= li.Upper())
        {
            li.SetValue(num, aOrgan);
        }
    }

    public NCollection_HArray1<TCollection_HAsciiString>? Organization()
    {
        return (fn is null ? null : fn.Organization());
    }

    /// <summary>Returns the value of attribute organization for the file_name entity.</summary>
    public TCollection_HAsciiString? OrganizationValue(int num)
    {
        return (fn is null ? null : fn.OrganizationValue(num));
    }

    /// <summary>Returns the number of values for the organization attribute in the file_name entity.</summary>
    public int NbOrganization()
    {
        return (fn is null ? 0 : fn.NbOrganization());
    }

    public void SetPreprocessorVersion(TCollection_HAsciiString? aPreprocessorVersion)
    {
        if (fn is not null)
        {
            fn.SetPreprocessorVersion(aPreprocessorVersion);
        }
    }

    /// <summary>Returns the name of the preprocessor_version for the file_name entity.</summary>
    public TCollection_HAsciiString? PreprocessorVersion()
    {
        return (fn is null ? null : fn.PreprocessorVersion());
    }

    public void SetOriginatingSystem(TCollection_HAsciiString? aOriginatingSystem)
    {
        if (fn is not null)
        {
            fn.SetOriginatingSystem(aOriginatingSystem);
        }
    }

    public TCollection_HAsciiString? OriginatingSystem()
    {
        return (fn is null ? null : fn.OriginatingSystem());
    }

    public void SetAuthorisation(TCollection_HAsciiString? aAuthorisation)
    {
        if (fn is not null)
        {
            fn.SetAuthorisation(aAuthorisation);
        }
    }

    /// <summary>Returns the value of the authorization attribute for the file_name entity.</summary>
    public TCollection_HAsciiString? Authorisation()
    {
        return (fn is null ? null : fn.Authorisation());
    }

    // ===========
    // File Schema
    // ===========

    /// <summary>Checks whether there is a file_schema entity. Returns True if there is one.</summary>
    public bool HasFs()
    {
        return (fs is not null);
    }

    /// <summary>Returns the file_schema entity. Returns an empty entity if the file_schema entity is not initialized.</summary>
    public HeaderSection_FileSchema? FsValue()
    {
        return fs;
    }

    public void SetSchemaIdentifiers(NCollection_HArray1<TCollection_HAsciiString>? aSchemaIdentifiers)
    {
        if (fs is not null)
        {
            fs.SetSchemaIdentifiers(aSchemaIdentifiers);
        }
    }

    public void SetSchemaIdentifiersValue(int num, TCollection_HAsciiString? aSchem)
    {
        if (fs is null)
        {
            return;
        }
        NCollection_HArray1<TCollection_HAsciiString> li = fs.SchemaIdentifiers();
        if (num >= li.Lower() && num <= li.Upper())
        {
            li.SetValue(num, aSchem);
        }
    }

    public NCollection_HArray1<TCollection_HAsciiString>? SchemaIdentifiers()
    {
        return (fs is null ? null : fs.SchemaIdentifiers());
    }

    /// <summary>Returns the value of the schema_identifier attribute for the file_schema entity.</summary>
    public TCollection_HAsciiString? SchemaIdentifiersValue(int num)
    {
        return (fs is null ? null : fs.SchemaIdentifiersValue(num));
    }

    /// <summary>Returns the number of values for the schema_identifier attribute in the file_schema entity.</summary>
    public int NbSchemaIdentifiers()
    {
        return (fs is null ? 0 : fs.NbSchemaIdentifiers());
    }

    /// <summary>Add a subname of schema (if not yet in the list).</summary>
    public void AddSchemaIdentifier(TCollection_HAsciiString aSchem)
    {
        if (fs is null)
        {
            fs = new HeaderSection_FileSchema();
        }
        NCollection_HArray1<TCollection_HAsciiString> idents = fs.SchemaIdentifiers();

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
        fs.SetSchemaIdentifiers(ids);
    }

    // ================
    // File Description
    // ================

    /// <summary>Checks whether there is a file_description entity. Returns True if there is one.</summary>
    public bool HasFd()
    {
        return (fd is not null);
    }

    /// <summary>Returns the file_description entity. Returns an empty entity if the file_description entity is not initialized.</summary>
    public HeaderSection_FileDescription? FdValue()
    {
        return fd;
    }

    public void SetDescription(NCollection_HArray1<TCollection_HAsciiString>? aDescription)
    {
        if (fs is not null) // OCCT: tests fs, then sets fd
        {
            fd.SetDescription(aDescription);
        }
    }

    public void SetDescriptionValue(int num, TCollection_HAsciiString? aDescr)
    {
        if (fd is null)
        {
            return;
        }
        NCollection_HArray1<TCollection_HAsciiString> li = fd.Description();
        if (num >= li.Lower() && num <= li.Upper())
        {
            li.SetValue(num, aDescr);
        }
    }

    public NCollection_HArray1<TCollection_HAsciiString>? Description()
    {
        return (fd is null ? null : fd.Description());
    }

    /// <summary>Returns the value of the description attribute for the file_description entity.</summary>
    public TCollection_HAsciiString? DescriptionValue(int num)
    {
        return (fd is null ? null : fd.DescriptionValue(num));
    }

    /// <summary>Returns the number of values for the file_description entity in the STEP file header.</summary>
    public int NbDescription()
    {
        return (fd is null ? 0 : fd.NbDescription());
    }

    public void SetImplementationLevel(TCollection_HAsciiString? aImplementationLevel)
    {
        if (fd is not null)
        {
            fd.SetImplementationLevel(aImplementationLevel);
        }
    }

    /// <summary>Returns the value of the implementation_level attribute for the file_description entity.</summary>
    public TCollection_HAsciiString? ImplementationLevel()
    {
        return (fd is null ? null : fd.ImplementationLevel());
    }
}
