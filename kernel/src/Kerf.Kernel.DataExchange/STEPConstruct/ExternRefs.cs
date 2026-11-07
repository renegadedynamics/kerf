// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_ExternRefs.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_ExternRefs.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Ap203;
using Kerf.Engine.Exchange.Step.Ap214;
using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Construct;

/// <summary>
/// Provides a tool for analyzing (reading) and creating (writing) references to external files in STEP It maintains
/// a data structure in the form of sequences of relevant STEP entities (roots), allowing either to create them by
/// convenient API, or load from existing model and investigate
/// </summary>
/// <remarks>
/// A C++ value class, a C# class here (decision 1185). Kerf: <see cref="FileName"/> looks for the external file
/// on the disk, beside the session's loaded file, as OCCT does (<c>OSD_File::Exists</c>), with
/// <see cref="Path.GetFullPath(string, string)"/> in place of <c>OSD_Path::AbsolutePath</c> (decision 1277).
/// </remarks>
internal class ExternRefs : ConstructTool
{
    /// <summary>Creates an empty tool</summary>
    public ExternRefs()
    {
    }

    /// <summary>Creates a tool and initializes it</summary>
    public ExternRefs(ExchangeWorkSession ws) : base(ws)
    {
    }

    /// <summary>Initializes tool; returns True if succeeded</summary>
    public bool Init(ExchangeWorkSession ws)
    {
        Clear();
        return SetWS(ws);
    }

    /// <summary>Clears internal fields (list of defined extern refs)</summary>
    public void Clear()
    {
        _aeiAs.Clear();
        _roles.Clear();
        _formats.Clear();
        _shapes.Clear();
        _types.Clear();
        _isAP214.Clear();
        // PTV 30.01.2003 TRJ11
        _docFiles.Clear();
        _sharedPRPC = null;
        _sharedDocType = null;
        _sharedPDC = null;
        _sharedPC = null;
        _apd = null;
    }

    private static bool FindPDWADandExcludeExcess(AppliedDocumentReference adr,
                                                  NCollection_Sequence<Transient> aSeqOfPDWAD,
                                                  EntityGraph graph,
                                                  ref ProductDefinitionWithAssociatedDocuments aPDWAD)
    {
        // WARNING! do not add check for aSeqOfPDWAD.Length() and exit if it < 1,
        // because this methods invokes with an empty sequence too to find PDWAD by ADR
        EntityIterator subsADR = graph.Shareds(adr);
        for (subsADR.Start(); subsADR.More(); subsADR.Next())
        {
            if (!subsADR.Value().IsKind(typeof(Document)))
            {
                continue;
            }
            Document doc = subsADR.Value() as Document;
            // looking for Document Product Equivalence
            EntityIterator subsD = graph.Sharings(doc);
            for (subsD.Start(); subsD.More(); subsD.Next())
            {
                if (!subsD.Value().IsKind(typeof(DocumentProductEquivalence)))
                {
                    continue;
                }
                DocumentProductEquivalence dpe = subsD.Value() as DocumentProductEquivalence;
                // take PDF and search the same PDF by PDWAD chain
                EntityIterator subsDPE = graph.Shareds(dpe);
                for (subsDPE.Start(); subsDPE.More(); subsDPE.Next())
                {
                    if (!subsDPE.Value().IsKind(typeof(ProductDefinitionFormation)))
                    {
                        continue;
                    }
                    ProductDefinitionFormation pdf = subsDPE.Value() as ProductDefinitionFormation;
                    EntityIterator subs = graph.Sharings(pdf);
                    for (subs.Start(); subs.More(); subs.Next())
                    {
                        if (!subs.Value().IsKind(typeof(ProductDefinitionWithAssociatedDocuments)))
                        {
                            continue;
                        }
                        aPDWAD = subs.Value() as ProductDefinitionWithAssociatedDocuments;
                    }
                    // now searching for PDWAD that refer to the same PDF
                    for (int pdwadi = 1; pdwadi <= aSeqOfPDWAD.Length(); pdwadi++)
                    {
                        ProductDefinitionWithAssociatedDocuments curPDWAD =
                            aSeqOfPDWAD.Value(pdwadi) as ProductDefinitionWithAssociatedDocuments;
                        if (curPDWAD is not null && aPDWAD == curPDWAD)
                        {
                            // found the same Product Definition Formation
                            aSeqOfPDWAD.Remove(pdwadi);
                            return true;
                        }
                    }
                } // end of looking for PDF by ADR chain
            } // end of looking for DPE
        } // end iterations on Shareds(ADR)
        return false;
    }

    /// <summary>
    /// Searches current STEP model for external references and loads them to the internal data structures NOTE: does
    /// not clear data structures before loading
    /// </summary>
    public bool LoadExternRefs()
    {
        // iterate on entities in the model and find AEIAs
        // or PDWADs (for AP203)
        InterfaceModel model = Model();
        Type tADR = typeof(AppliedDocumentReference);
        Type tPDWAD = typeof(ProductDefinitionWithAssociatedDocuments);
        int nb = model.EntityCount;

        // PTV 28.01.2003 CAX-IF TRJ11, file ext_ref_master.stp
        // search all ADR and PDWAD and exclude excess PDWADs
        NCollection_Sequence<Transient> seqOfADR = new NCollection_Sequence<Transient>();
        NCollection_Sequence<Transient> seqOfPDWAD = new NCollection_Sequence<Transient>();
        for (int ient = 1; ient <= nb; ient++)
        {
            Transient enti = model.Value(ient);
            if (enti.DynamicType() == tPDWAD)
            {
                seqOfPDWAD.Append(enti);
            }
            else if (enti.DynamicType() == tADR)
            {
                seqOfADR.Append(enti);
            }
        }
        int isAP214 = 0;
        // run on sequence aSeqOfADR of ADR and remove excess PDWAD from aSeqOfPDWAD
        for (int adri = 1; adri <= seqOfADR.Length(); adri++)
        {
            AppliedDocumentReference adr = seqOfADR.Value(adri) as AppliedDocumentReference;
            // looking for Product Definition Formation and exclude excess PDWAD from aSeqOfPDWAD
            ProductDefinitionWithAssociatedDocuments pdwad = null;
            FindPDWADandExcludeExcess(adr, seqOfPDWAD, Graph(), ref pdwad);

            // now add all necessary information as original implementation.
            isAP214 = 1;
            RoleAssociation role = null;
            ProductDefinition shape = null;
            PropertyDefinitionRepresentation format = null;
            DocumentRepresentationType type = null;
            // AppliedDocumentReference with RoleAssociation...
            EntityIterator subs4 = Graph().Sharings(adr);
            for (subs4.Start(); subs4.More(); subs4.Next())
            {
                if (subs4.Value().IsKind(typeof(RoleAssociation)))
                {
                    role = subs4.Value() as RoleAssociation;
                }
            }

            subs4 = Graph().Shareds(adr);
            for (subs4.Start(); subs4.More(); subs4.Next())
            {
                if (subs4.Value().IsKind(typeof(ProductDefinition)))
                {
                    shape = subs4.Value() as ProductDefinition;
                }
            }
            // search for Document file
            DocumentFile docFile = null;
            if (pdwad is null)
            { // shouldn't begin from TRJ11
                // lookinf from ADR
                subs4 = Graph().Shareds(adr);
            }
            else
            {
                // looking from PDWAD
                subs4 = Graph().Shareds(pdwad);
            }
            for (subs4.Start(); subs4.More(); subs4.Next())
            {
                if (!subs4.Value().IsKind(typeof(DocumentFile)))
                {
                    continue;
                }
                docFile = subs4.Value() as DocumentFile;
                if (docFile is null)
                {
                    continue;
                }
                // for each DocumentFile, find associated with it data:
                EntityIterator subs = Graph().Sharings(docFile);
                for (subs.Start(); subs.More(); subs.Next())
                {
                    Transient sub = subs.Value();

                    // FORMAT - ???????
                    //
                    // PDRs of a shape and of a file format
                    if (sub.IsKind(typeof(PropertyDefinition)))
                    {
                        PropertyDefinition pd = sub as PropertyDefinition;
                        EntityIterator subs2 = Graph().Sharings(pd);
                        for (subs2.Start(); subs2.More(); subs2.Next())
                        {
                            PropertyDefinitionRepresentation pdr = subs2.Value() as PropertyDefinitionRepresentation;
                            if (pdr is null)
                            {
                                continue;
                            }
                            if (pdr.UsedRepresentation().IsKind(typeof(StepShapeRepresentation)))
                            {
                                format = pdr;
                            }
                        }
                    }
                    // DocumentRepresentationType
                    if (sub.IsKind(typeof(DocumentRepresentationType)))
                    {
                        type = sub as DocumentRepresentationType;
                    }
                    if (type is not null && format is not null)
                    {
                        break;
                    }
                }
                if (type is not null && format is not null)
                {
                    break;
                }
            }
            if (docFile is null)
            {
                continue;
            }
            _aeiAs.Append(adr);
            _roles.Append(role);
            _formats.Append(format);
            _shapes.Append(shape);
            _types.Append(type);
            _isAP214.Append(isAP214);
            _docFiles.Append(docFile);
        } // end iterations on aSeqOfADR

        // now iterates on sequence aSeqOfPDWAD of Product Definition With Associated Documents
        for (int pdwadi = 1; pdwadi <= seqOfPDWAD.Length(); pdwadi++)
        {
            isAP214 = 0;
            ProductDefinitionWithAssociatedDocuments pdwad =
                seqOfPDWAD.Value(pdwadi) as ProductDefinitionWithAssociatedDocuments;
            _shapes.Append(pdwad);
            _isAP214.Append(isAP214);
            _aeiAs.Append((Transient)null);
            _roles.Append((Transient)null);
            _formats.Append((Transient)null);
            _types.Append((Transient)null);
            _docFiles.Append((Transient)null);
        }

        return _shapes.Length() > 0;
    }

    /// <summary>Returns number of defined extern references</summary>
    public int ExternRefCount
    {
        get
        {
            return _shapes.Length();
        }
    }

    /// <summary>
    /// Kerf (unit 8Z): when set, tells <see cref="FileName"/> whether an external file exists (its name as the
    /// file gives it, the session's loaded file), in place of OCCT's look on the disk beside the loaded file.
    /// STEPCAFControl_Reader sets it from its resolver, or to false when it reads no external files.
    /// </summary>
    internal Func<string, string, bool>? InternalFileExists { get; set; }

    private bool Probe(string name, string fullName)
    {
        if (InternalFileExists is null)
        {
            return FileExists(fullName);
        }
        return InternalFileExists(name, WS().LoadedFile() ?? "");
    }

    // OSD_Path's folder of the session's loaded file, and OSD_Path::AbsolutePath
    private static string DirectoryOf(string file)
    {
        if (string.IsNullOrEmpty(file))
        {
            return "";
        }
        string dir = Path.GetDirectoryName(file);
        return dir is null ? "" : dir;
    }

    internal static string AbsolutePath(string dirPath, string relFilePath)
    {
        try
        {
            if (Path.IsPathRooted(relFilePath))
            {
                return relFilePath;
            }
            if (string.IsNullOrEmpty(dirPath))
            {
                return "";
            }
            return Path.GetFullPath(relFilePath, Path.GetFullPath(dirPath));
        }
        catch (Exception e) when (e is not OutOfMemoryException) // Kerf: decision 22802
        {
            return "";
        }
    }

    private static bool FileExists(string path)
    {
        try
        {
            return File.Exists(path);
        }
        catch (Exception e) when (e is not OutOfMemoryException) // Kerf: decision 22802
        {
            return false;
        }
    }

    /// <summary>Returns filename for numth extern reference Returns Null if FileName is not defined or bad</summary>
    public string FileName(int num)
    {
        DocumentFile docFile = null;
        AppliedExternalIdentificationAssignment aeia = null;
        string cStringFileName = null;
        if (_docFiles.Length() >= num && _docFiles.Value(num) is not null)
        {
            docFile = _docFiles.Value(num) as DocumentFile;
        }
        else if (_isAP214.Value(num) == 1)
        {
            AppliedDocumentReference adr = _aeiAs.Value(num) as AppliedDocumentReference;

            // PTV 28.01.2003 CAX-IF TRJ11, file ext_ref_master.stp
            // search document file name by long chain ADR->D<-DPE->PDF<-PDWAD->DF
            ProductDefinitionWithAssociatedDocuments pdwad = null;
            // create an empty aSeqOfPDWAD
            NCollection_Sequence<Transient> seqOfPDWAD = new NCollection_Sequence<Transient>();
            // we do not need to exclude, just find PDWAD
            FindPDWADandExcludeExcess(adr, seqOfPDWAD, Graph(), ref pdwad);

            // search for Document file
            EntityIterator subs4;
            if (pdwad is null)
            { // shouldn't begin from TRJ11
                // lookinf from ADR
                subs4 = Graph().Shareds(adr);
            }
            else
            {
                // looking from PDWAD
                subs4 = Graph().Shareds(pdwad);
            }
            for (subs4.Start(); subs4.More(); subs4.Next())
            {
                if (!subs4.Value().IsKind(typeof(DocumentFile)))
                {
                    continue;
                }
                docFile = subs4.Value() as DocumentFile;
                if (docFile is null)
                {
                    continue;
                }
            }
        }
        else
        {
            ProductDefinitionWithAssociatedDocuments pdwad =
                _shapes.Value(num) as ProductDefinitionWithAssociatedDocuments;
            if (pdwad is null || pdwad.DocIds() is null)
            {
                return "";
            }
            for (int i = 1; i <= pdwad.DocIdCount; i++)
            {
                Document doc = pdwad.DocIdsValue(i);
                TCollection_HAsciiString filename = doc.Name();
                if (filename is not null && !filename.IsEmpty())
                {
                    return filename.ToCString();
                }
            }
            return "";
        }
        // take name from AEIA and from DF
        if (docFile is not null)
        {
            EntityIterator subs3 = Graph().Sharings(docFile);
            for (subs3.Start(); subs3.More(); subs3.Next())
            {
                if (subs3.Value().IsKind(typeof(AppliedExternalIdentificationAssignment)))
                {
                    aeia = subs3.Value() as AppliedExternalIdentificationAssignment;
                    if (aeia is not null)
                    {
                        break;
                    }
                }
            }
        }
        if (aeia is not null)
        {
            TCollection_HAsciiString filename = aeia.AssignedId();
            if (filename is not null && !filename.IsEmpty())
            {
                cStringFileName = filename.ToCString();
                // ptv 29.01.2003 file trj4_xr1-tc-214.stp entity #71 have id "#71"
                if (cStringFileName.Length > 0 && cStringFileName[0] == '#')
                {
                    cStringFileName = null;
                }
            }
            if (string.IsNullOrEmpty(cStringFileName))
            {
                // try to take name from external source
                ExternalSource theSource = aeia.Source();
                if (theSource is not null)
                {
                    SourceItem theSourceId = theSource.SourceId();
                    if (theSourceId is not null && !theSourceId.IsNull)
                    {
                        SelectNamed theFileName = theSourceId.Value() as SelectNamed;
                        if (theFileName is null || theFileName.Kind() != 6)
                        {
                            // nothing to do, hope could take name later.
                        }
                        else
                        {
                            cStringFileName = theFileName.String();
                        }
                    }
                }
            }
        }
        string oldFileName = null;
        // compute true path to the extern file
        string dpath = DirectoryOf(WS().LoadedFile());
        if (!string.IsNullOrEmpty(cStringFileName))
        {
            string fullname = AbsolutePath(dpath, cStringFileName);
            if (fullname.Length <= 0)
            {
                fullname = cStringFileName;
            }
            if (!Probe(cStringFileName, fullname))
            {
                oldFileName = cStringFileName;
                cStringFileName = null;
            }
        }
        if (string.IsNullOrEmpty(cStringFileName))
        {
            // try to find name of the directory from DocFile
            if (docFile is not null)
            {
                TCollection_HAsciiString filename = docFile.Id();
                if (filename is not null && !filename.IsEmpty())
                {
                    cStringFileName = filename.ToCString();
                }
                if (string.IsNullOrEmpty(cStringFileName))
                {
                    filename = docFile.Name();
                    if (filename is not null && !filename.IsEmpty())
                    {
                        cStringFileName = filename.ToCString();
                    }
                }
                if (string.IsNullOrEmpty(cStringFileName))
                {
                    if (oldFileName is not null)
                    {
                        cStringFileName = oldFileName;
                    }
                    else
                    {
                        return "";
                    }
                }
            }
        }
        if (cStringFileName is null)
        {
            // OCCT: a null name (no document file) reaches OSD_Path::AbsolutePath and AssignCat; nothing is found
            cStringFileName = oldFileName ?? "";
        }
        {
            string fullname = AbsolutePath(dpath, cStringFileName);
            if (fullname.Length <= 0)
            {
                fullname = cStringFileName;
            }
            if (!Probe(cStringFileName, fullname))
            {
                if (oldFileName is not null)
                {
                    cStringFileName = oldFileName;
                }
                TransientProcess tp = WS().TransferReader().TransientProcess();
                string mess = "Can not read external file " + cStringFileName;
                tp.AddFail(docFile, mess);
            }
            else
            {
                if (oldFileName is not null && oldFileName != cStringFileName)
                {
                    TransientProcess tp = WS().TransferReader().TransientProcess();
                    string mess = "External file with name from entity AEIA (" + oldFileName
                                   + ") not existed => use file name from DocumentFile entity - " + cStringFileName;
                    tp.AddWarning(docFile, mess);
                }
            }
        }
        return cStringFileName;
    }

    /// <summary>
    /// Returns ProductDefinition to which numth extern reference is associated. Returns Null if cannot be detected or
    /// if extern reference is not associated to SDR in a proper way.
    /// </summary>
    public ProductDefinition ProdDef(int num)
    {
        return _shapes.Value(num) as ProductDefinition;
    }

    /// <summary>Returns DocumentFile to which numth extern reference is associated. Returns Null if cannot be detected.</summary>
    public DocumentFile DocFile(int num)
    {
        return _docFiles.Value(num) as DocumentFile;
    }

    /// <summary>Returns format identification string for the extern document Returns Null handle if format is not defined</summary>
    public TCollection_HAsciiString Format(int num)
    {
        TCollection_HAsciiString format = null;
        if (_isAP214.Value(num) == 0)
        {
            return format;
        }
        PropertyDefinitionRepresentation pdr = _formats.Value(num) as PropertyDefinitionRepresentation;

        if (pdr is null)
        {
            return format;
        }

        StepRepresentation rep = pdr.UsedRepresentation();
        for (int i = 1; i <= rep.NbItems(); i++)
        {
            if (rep.ItemsValue(i).IsKind(typeof(DescriptiveRepresentationItem)))
            {
                DescriptiveRepresentationItem dri = rep.ItemsValue(i) as DescriptiveRepresentationItem;
                format = dri.Description();
                break;
            }
        }

        return format;
    }

    /// <summary>
    /// Create a new external reference with specified attributes attached to a given SDR &lt;format&gt; can be Null
    /// string, in that case this information is not written. Else, it can be "STEP AP214" or "STEP AP203" Returns
    /// index of a new extern ref
    /// </summary>
    public int AddExternRef(string filename, ProductDefinition pd, string format)
    {
        TCollection_HAsciiString emptyString = new TCollection_HAsciiString("");
        TCollection_HAsciiString fmt = new TCollection_HAsciiString(format);
        TCollection_HAsciiString tmp = new TCollection_HAsciiString("203");
        int np = fmt.Location(tmp, 1, fmt.Length());

        //  if( !(fmt==tmp) ) {
        if (!(np > 0))
        {
            // create core entity DocumentFile
            DocumentType DT = new DocumentType();
            DT.Init(emptyString);
            TCollection_HAsciiString dFid = new TCollection_HAsciiString(filename);
            // PTV 30.01.2003 TRJ11 -  copy external filename as is
            //     DFid->AssignCat ( " file id" );
            DocumentFile df = new DocumentFile();
            df.Init(dFid, emptyString, false, emptyString, DT, emptyString, false, emptyString);

            // create AppliedExternalIdentificationAssignment et al
            IdentificationRole ir = new IdentificationRole();
            // PTV 30.01.2003 TRJ11
            //    - set the ("external document id and location", $) without unmeaning description
            TCollection_HAsciiString name = new TCollection_HAsciiString("external document id and location");
            ir.SetName(name);
            //     occ::handle<TCollection_HAsciiString> aIRdescr = new TCollection_HAsciiString("source
            //     system"); IR->Init(aName, true, aIRdescr);

            SelectNamed sds = new SelectNamed();
            sds.SetString(filename);
            sds.SetName("IDENTIFIER");
            SourceItem sid = new SourceItem();
            sid.SetValue(sds);
            ExternalSource es = new ExternalSource();
            es.Init(sid);

            ExternalIdentificationItem item = new ExternalIdentificationItem();
            item.SetValue(df);
            NCollection_HArray1<ExternalIdentificationItem> items =
                new NCollection_HArray1<ExternalIdentificationItem>(1, 1);
            items.SetValue(1, item);

            AppliedExternalIdentificationAssignment extIdent = new AppliedExternalIdentificationAssignment();
            //     ExtIdent->Init(EmptyString, IR, ES, Items);
            // PTV 30.01.2003 TRJ11 - store filename in AEIA
            TCollection_HAsciiString fName = new TCollection_HAsciiString(filename);
            extIdent.Init(fName, ir, es, items);

            // create DocumentRepresentationType
            TCollection_HAsciiString dig = new TCollection_HAsciiString("digital");
            DocumentRepresentationType type = new DocumentRepresentationType();
            type.Init(dig, df);

            // create AppliedDocumentReference,
            AppliedDocumentReference adr = new AppliedDocumentReference();
            // PTV 30.01.2003 TRJ11 - create additional entities for AP214
            AddAP214ExterRef(adr, pd, df, filename);

            // create RoleAssociation etc.
            ObjectRole or = new ObjectRole();
            TCollection_HAsciiString mandatory = new TCollection_HAsciiString("mandatory");
            or.Init(mandatory, false, emptyString);
            RoleSelect rs = new RoleSelect();
            rs.SetValue(adr);
            RoleAssociation role = new RoleAssociation();
            role.Init(or, rs);

            // create PDR for association with SR
            CharacterizedDefinition cd = new CharacterizedDefinition();
            cd.SetValue(df);
            TCollection_HAsciiString pDname = new TCollection_HAsciiString("external definition");
            PropertyDefinition propD = new PropertyDefinition();
            propD.Init(pDname, true, emptyString, cd);
            RepresentedDefinition rd = new RepresentedDefinition();
            rd.SetValue(propD);
            //    occ::handle<StepRepr_PropertyDefinitionRepresentation> PDRshape = new
            //    StepRepr_PropertyDefinitionRepresentation; PDRshape->Init ( RD, SDR->UsedRepresentation()
            //    );

            // create PDR for definition of document format (if defined)
            PropertyDefinitionRepresentation pdRformat = null;
            if (!string.IsNullOrEmpty(format))
            {
                TCollection_HAsciiString rCftype = new TCollection_HAsciiString("document parameters");
                RepresentationContext rCf = new RepresentationContext();
                rCf.Init(emptyString, rCftype);

                TCollection_HAsciiString drIname = new TCollection_HAsciiString("data format");
                TCollection_HAsciiString drIdscr = new TCollection_HAsciiString(format);
                DescriptiveRepresentationItem dri = new DescriptiveRepresentationItem();
                dri.Init(drIname, drIdscr);
                NCollection_HArray1<RepresentationItem> fItems = new NCollection_HArray1<RepresentationItem>(1, 1);
                fItems.SetValue(1, dri);

                TCollection_HAsciiString sRfname = new TCollection_HAsciiString("document format");
                StepRepresentation sRformat = new StepRepresentation();
                sRformat.Init(sRfname, fItems, rCf);

                CharacterizedDefinition cDf = new CharacterizedDefinition();
                cDf.SetValue(df);
                TCollection_HAsciiString pDfname = new TCollection_HAsciiString("document property");
                PropertyDefinition pDf = new PropertyDefinition();
                pDf.Init(pDfname, true, emptyString, cDf);
                RepresentedDefinition rDf = new RepresentedDefinition();
                rDf.SetValue(pDf);

                pdRformat = new PropertyDefinitionRepresentation();
                pdRformat.Init(rDf, sRformat);
            }

            // add all the created root entities to sequences
            _aeiAs.Append(extIdent);    // StepAP214_AppliedExternalIdentificationAssignment
            _roles.Append(role);        // StepBasic_RoleAssociation
            _formats.Append(pdRformat); // StepRepr_PropertyDefinitionRepresentation
            //    myShapes.Append ( PDRshape );    //StepRepr_PropertyDefinitionRepresentation
            _shapes.Append(pd);  // StepBasic_ProductDefinition
            _types.Append(type); // StepBasic_DocumentRepresentationType
        }
        else
        { // format=="AP203"
            ProductDefinitionFormation prodDefForm = pd.Formation();
            ProductDefinitionContext prodDefCont = pd.FrameOfReference();

            // create document
            TCollection_HAsciiString fname = new TCollection_HAsciiString(filename);
            DocumentType docType = new DocumentType();
            TCollection_HAsciiString dt = new TCollection_HAsciiString("cad_filename");
            docType.Init(dt);
            Document doc = new Document();
            TCollection_HAsciiString description = new TCollection_HAsciiString("CAD Model associated to the part");
            doc.Init(emptyString, fname, true, description, docType);
            NCollection_HArray1<Document> docIds = new NCollection_HArray1<Document>(1, 1);
            docIds.SetValue(1, doc);

            // create ProductDefinitionWithAssociatedDocuments
            description = pd.Description();
            ProductDefinitionWithAssociatedDocuments pdwad = new ProductDefinitionWithAssociatedDocuments();
            pdwad.Init(emptyString, description, prodDefForm, prodDefCont, docIds);

            // search in graph for replace
            EntityIterator subs = Graph().Sharings(pd);
            for (subs.Start(); subs.More(); subs.Next())
            {
                Transient sub = subs.Value();
                if (!sub.IsKind(typeof(ProductDefinitionShape)))
                {
                    continue;
                }
                ProductDefinitionShape prodDefSh = sub as ProductDefinitionShape;
                if (prodDefSh is null)
                {
                    continue;
                }
                CharacterizedDefinition cDf = new CharacterizedDefinition();
                cDf.SetValue(pdwad);
                prodDefSh.SetDefinition(cDf);
            }

            int numProdDef = Model().Number(pd);
            EntityIterator subs2 = Graph().Sharings(pd);
            for (subs2.Start(); subs2.More(); subs2.Next())
            {
                Transient sub2 = subs2.Value();

                if (sub2.IsKind(typeof(NextAssemblyUsageOccurrence)))
                {
                    NextAssemblyUsageOccurrence nauo = sub2 as NextAssemblyUsageOccurrence;
                    nauo.SetRelatedProductDefinition(pdwad);
                }

                if (sub2.IsKind(typeof(CcDesignPersonAndOrganizationAssignment)))
                {
                    CcDesignPersonAndOrganizationAssignment cdpaoa = sub2 as CcDesignPersonAndOrganizationAssignment;
                    NCollection_HArray1<PersonOrganizationItem> hapoi = cdpaoa.Items();
                    for (int i = 1; i <= hapoi.Length(); i++)
                    {
                        PersonOrganizationItem poi = hapoi.Value(i);
                        ProductDefinition pDtmp = poi.ProductDefinition();
                        int numPDtmp = Model().Number(pDtmp);
                        if (numProdDef == numPDtmp)
                        {
                            // OCCT sets a copy and stores it back
                            PersonOrganizationItem aNew = new PersonOrganizationItem();
                            aNew.SetValue(pdwad);
                            hapoi.SetValue(i, aNew);
                        }
                    }
                }
                if (sub2.IsKind(typeof(CcDesignDateAndTimeAssignment)))
                {
                    CcDesignDateAndTimeAssignment cddata = sub2 as CcDesignDateAndTimeAssignment;
                    NCollection_HArray1<DateTimeItem> hadti = cddata.Items();
                    for (int i = 1; i <= hadti.Length(); i++)
                    {
                        DateTimeItem dti = hadti.Value(i);
                        ProductDefinition pDtmp = dti.ProductDefinition();
                        int numPDtmp = Model().Number(pDtmp);
                        if (numProdDef == numPDtmp)
                        {
                            DateTimeItem aNew = new DateTimeItem();
                            aNew.SetValue(pdwad);
                            hadti.SetValue(i, aNew);
                        }
                    }
                }
                if (sub2.IsKind(typeof(CcDesignApproval)))
                {
                    CcDesignApproval cda = sub2 as CcDesignApproval;
                    NCollection_HArray1<ApprovedItem> haai = cda.Items();
                    for (int i = 1; i <= haai.Length(); i++)
                    {
                        ApprovedItem ai = haai.Value(i);
                        ProductDefinition pDtmp = ai.ProductDefinition();
                        int numPDtmp = Model().Number(pDtmp);
                        if (numProdDef == numPDtmp)
                        {
                            ApprovedItem aNew = new ApprovedItem();
                            aNew.SetValue(pdwad);
                            haai.SetValue(i, aNew);
                        }
                    }
                }
            }

            _aeiAs.Append(pdwad);
            _replaceNum.Append(numProdDef);
            _roles.Append(doc);
            _types.Append(docType);
        }
        return _aeiAs.Length();
    }

    /// <summary>Check (create if it is null) all shared entities for the model</summary>
    public void CheckAP214Shared()
    {
        TCollection_HAsciiString emptyString = new TCollection_HAsciiString("");
        if (_sharedPRPC is null)
        {
            // create new ProductRelatedProductCategory for all extern files.
            TCollection_HAsciiString prpCname = new TCollection_HAsciiString("document");
            _sharedPRPC = new ProductRelatedProductCategory();
            _sharedPRPC.Init(prpCname, false, emptyString, null);
        }
        if (_sharedDocType is null)
        {
            // create new shared Document Type
            _sharedDocType = new DocumentType();
            TCollection_HAsciiString prod_dat_type = new TCollection_HAsciiString("configuration controlled document version");
            _sharedDocType.Init(prod_dat_type);
        }
        if (_sharedPDC is null)
        {
            // create new shared Product Definition Context
            _sharedPDC = new ProductDefinitionContext();
            TCollection_HAsciiString pdCname = new TCollection_HAsciiString("digital document definition");
            ApplicationContext appContext = GetAP214APD().Application();
            _sharedPDC.Init(pdCname, appContext, emptyString);
        }
        if (_sharedPC is null)
        {
            // create new shared ProductContext
            _sharedPC = new ProductContext();
            ApplicationContext appContext = GetAP214APD().Application();
            _sharedPC.Init(emptyString, appContext, emptyString);
        }
    }

    /// <summary>Adds all the currently defined external refs to the model Returns number of written extern refs</summary>
    public int WriteExternRefs(int num)
    {
        if (num == 3)
        {
            for (int i = 1; i <= _aeiAs.Length(); i++)
            {
                Model().ReplaceEntity(_replaceNum.Value(i), _aeiAs.Value(i));
                if (_roles.Value(i) is not null)
                {
                    Model().AddWithRefs(_roles.Value(i));
                }
                if (_types.Value(i) is not null)
                {
                    Model().AddWithRefs(_types.Value(i));
                }
            }
        }
        else
        {
            for (int i = 1; i <= _aeiAs.Length(); i++)
            {
                Model().AddWithRefs(_aeiAs.Value(i));
                if (_roles.Value(i) is not null)
                {
                    Model().AddWithRefs(_roles.Value(i));
                }
                if (_formats.Value(i) is not null)
                {
                    Model().AddWithRefs(_formats.Value(i));
                }
                if (_shapes.Value(i) is not null)
                {
                    Model().AddWithRefs(_shapes.Value(i));
                }
                if (_types.Value(i) is not null)
                {
                    Model().AddWithRefs(_types.Value(i));
                }
            }
        }
        // PTV 30.01.2003 TRJ11
        if (_apd is not null)
        {
            Model().AddWithRefs(_apd);
        }
        if (_sharedPRPC is not null)
        {
            Model().AddWithRefs(_sharedPRPC);
        }

        return _aeiAs.Length();
    }

    /// <summary>Set the ApplicationProtocolDefinition of the PDM schema</summary>
    public void SetAP214APD(ApplicationProtocolDefinition apd)
    {
        _apd = apd;
    }

    /// <summary>
    /// Returns the ApplicationProtocolDefinition of the PDM schema NOTE: if not defined then create new APD with new
    /// Application Context
    /// </summary>
    public ApplicationProtocolDefinition GetAP214APD()
    {
        if (_apd is null)
        {
            // create new APD with new Application Context
            _apd = new ApplicationProtocolDefinition();
            // examples of the values taken from ext_ref_master.stp
            TCollection_HAsciiString status = new TCollection_HAsciiString("version 1.1");
            TCollection_HAsciiString appSchemaName = new TCollection_HAsciiString("pdm_schema");
            int intProtocolYear = 1999;
            ApplicationContext application = new ApplicationContext();
            TCollection_HAsciiString emptyString = new TCollection_HAsciiString("");
            application.Init(emptyString);
            _apd.Init(status, appSchemaName, intProtocolYear, application);
        }
        return _apd;
    }

    /// <summary>
    /// Create a new additional structure entities and add necessary references Note: do not refer from ADR to DF
    /// directly in AP214 (TRJ11).
    /// </summary>
    protected bool AddAP214ExterRef(AppliedDocumentReference adr, ProductDefinition pd,
                                    DocumentFile df, string filename)
    {
        NCollection_HArray1<DocumentReferenceItem> drIs = new NCollection_HArray1<DocumentReferenceItem>(1, 1);
        DocumentReferenceItem dri = new DocumentReferenceItem();
        dri.SetValue(pd);
        drIs.SetValue(1, dri);
        TCollection_HAsciiString emptyString = new TCollection_HAsciiString("");

        // create/get created shared entities:
        // DocumentType, ProductDefinitionContext, ProductRelatedProductCategory, ProductContext
        CheckAP214Shared();

        // create document
        Document document = new Document();
        document.Init(emptyString, emptyString, false, emptyString, _sharedDocType);
        adr.Init(document, emptyString, drIs);

        // create new product
        Product product = new Product();
        NCollection_HArray1<Product> hProducts = _sharedPRPC.Products();
        int nbProducts = 0;
        if (hProducts is not null)
        {
            nbProducts = hProducts.Length();
        }
        int intProdId = 20001 + nbProducts;
        TCollection_HAsciiString productID = new TCollection_HAsciiString(intProdId);
        TCollection_HAsciiString productName = new TCollection_HAsciiString(filename + "-Doc");
        NCollection_HArray1<ProductContext> hProdContext = new NCollection_HArray1<ProductContext>(1, 1);
        hProdContext.SetValue(1, _sharedPC);
        product.Init(productID, productName, emptyString, hProdContext);

        // create new product definition formation
        ProductDefinitionFormation pdf = new ProductDefinitionFormation();
        // name id taken from example Standard_ExtString_ref_master.stp
        TCollection_HAsciiString PDF_ID = new TCollection_HAsciiString("1");
        pdf.Init(PDF_ID, emptyString, product);

        DocumentProductEquivalence dpe = new DocumentProductEquivalence();
        TCollection_HAsciiString dpEname = new TCollection_HAsciiString("equivalence");
        ProductOrFormationOrDefinition pofod = new ProductOrFormationOrDefinition();
        pofod.SetValue(pdf);
        dpe.Init(dpEname, false, emptyString, document, pofod);
        // add to the model with references
        Model().AddWithRefs(dpe);

        // add products to shared PRPC
        NCollection_HArray1<Product> newHProducts = new NCollection_HArray1<Product>(1, nbProducts + 1);
        for (int pi = 1; pi <= nbProducts; pi++)
        {
            newHProducts.SetValue(pi, hProducts.Value(pi));
        }
        newHProducts.SetValue(nbProducts + 1, product);
        // set the hArray to the PRPC
        _sharedPRPC.SetProducts(newHProducts);

        // create new PDWAD
        ProductDefinitionWithAssociatedDocuments pdwad = new ProductDefinitionWithAssociatedDocuments();
        NCollection_HArray1<Document> docIds = new NCollection_HArray1<Document>(1, 1);
        docIds.SetValue(1, df);
        TCollection_HAsciiString PDWAD_ID = new TCollection_HAsciiString("1");
        pdwad.Init(PDWAD_ID, emptyString, pdf, _sharedPDC, docIds);
        // add to the model with references
        Model().AddWithRefs(pdwad);

        return true;
    }

    private readonly NCollection_Sequence<Transient> _aeiAs = new NCollection_Sequence<Transient>();
    private readonly NCollection_Sequence<Transient> _roles = new NCollection_Sequence<Transient>();
    private readonly NCollection_Sequence<Transient> _formats = new NCollection_Sequence<Transient>();
    private readonly NCollection_Sequence<Transient> _shapes = new NCollection_Sequence<Transient>();
    private readonly NCollection_Sequence<Transient> _types = new NCollection_Sequence<Transient>();
    private readonly NCollection_Sequence<int> _isAP214 = new NCollection_Sequence<int>();
    private readonly NCollection_Sequence<int> _replaceNum = new NCollection_Sequence<int>();
    private readonly NCollection_Sequence<Transient> _docFiles = new NCollection_Sequence<Transient>();
    private ProductRelatedProductCategory _sharedPRPC;
    private DocumentType _sharedDocType;
    private ProductDefinitionContext _sharedPDC;
    private ProductContext _sharedPC;
    private ApplicationProtocolDefinition _apd;
}
