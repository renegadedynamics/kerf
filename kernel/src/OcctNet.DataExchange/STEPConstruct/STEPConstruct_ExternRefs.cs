// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_ExternRefs.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_ExternRefs.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// Provides a tool for analyzing (reading) and creating (writing) references to external files in STEP It maintains
/// a data structure in the form of sequences of relevant STEP entities (roots), allowing either to create them by
/// convenient API, or load from existing model and investigate
/// </summary>
/// <remarks>
/// A C++ value class, a C# class here (decision 1185). OCCT.NET: <see cref="FileName"/> looks for the external file
/// on the disk, beside the session's loaded file, as OCCT does (<c>OSD_File::Exists</c>), with
/// <see cref="Path.GetFullPath(string, string)"/> in place of <c>OSD_Path::AbsolutePath</c> (decision 1277).
/// </remarks>
public class STEPConstruct_ExternRefs : STEPConstruct_Tool
{
    /// <summary>Creates an empty tool</summary>
    public STEPConstruct_ExternRefs()
    {
    }

    /// <summary>Creates a tool and initializes it</summary>
    public STEPConstruct_ExternRefs(XSControl_WorkSession WS) : base(WS)
    {
    }

    /// <summary>Initializes tool; returns True if succeeded</summary>
    public bool Init(XSControl_WorkSession WS)
    {
        Clear();
        return SetWS(WS);
    }

    /// <summary>Clears internal fields (list of defined extern refs)</summary>
    public void Clear()
    {
        myAEIAs.Clear();
        myRoles.Clear();
        myFormats.Clear();
        myShapes.Clear();
        myTypes.Clear();
        myIsAP214.Clear();
        // PTV 30.01.2003 TRJ11
        myDocFiles.Clear();
        mySharedPRPC = null;
        mySharedDocType = null;
        mySharedPDC = null;
        mySharedPC = null;
        myAPD = null;
    }

    private static bool findPDWADandExcludeExcess(StepAP214_AppliedDocumentReference ADR,
                                                  NCollection_Sequence<Standard_Transient> aSeqOfPDWAD,
                                                  Interface_Graph Graph,
                                                  ref StepBasic_ProductDefinitionWithAssociatedDocuments aPDWAD)
    {
        // WARNING! do not add check for aSeqOfPDWAD.Length() and exit if it < 1,
        // because this methods invokes with an empty sequence too to find PDWAD by ADR
        Interface_EntityIterator subsADR = Graph.Shareds(ADR);
        for (subsADR.Start(); subsADR.More(); subsADR.Next())
        {
            if (!subsADR.Value().IsKind(typeof(StepBasic_Document)))
            {
                continue;
            }
            StepBasic_Document aDoc = subsADR.Value() as StepBasic_Document;
            // looking for Document Product Equivalence
            Interface_EntityIterator subsD = Graph.Sharings(aDoc);
            for (subsD.Start(); subsD.More(); subsD.Next())
            {
                if (!subsD.Value().IsKind(typeof(StepBasic_DocumentProductEquivalence)))
                {
                    continue;
                }
                StepBasic_DocumentProductEquivalence aDPE = subsD.Value() as StepBasic_DocumentProductEquivalence;
                // take PDF and search the same PDF by PDWAD chain
                Interface_EntityIterator subsDPE = Graph.Shareds(aDPE);
                for (subsDPE.Start(); subsDPE.More(); subsDPE.Next())
                {
                    if (!subsDPE.Value().IsKind(typeof(StepBasic_ProductDefinitionFormation)))
                    {
                        continue;
                    }
                    StepBasic_ProductDefinitionFormation aPDF = subsDPE.Value() as StepBasic_ProductDefinitionFormation;
                    Interface_EntityIterator subs = Graph.Sharings(aPDF);
                    for (subs.Start(); subs.More(); subs.Next())
                    {
                        if (!subs.Value().IsKind(typeof(StepBasic_ProductDefinitionWithAssociatedDocuments)))
                        {
                            continue;
                        }
                        aPDWAD = subs.Value() as StepBasic_ProductDefinitionWithAssociatedDocuments;
                    }
                    // now searching for PDWAD that refer to the same PDF
                    for (int pdwadi = 1; pdwadi <= aSeqOfPDWAD.Length(); pdwadi++)
                    {
                        StepBasic_ProductDefinitionWithAssociatedDocuments aCurPDWAD =
                            aSeqOfPDWAD.Value(pdwadi) as StepBasic_ProductDefinitionWithAssociatedDocuments;
                        if (aCurPDWAD is not null && aPDWAD == aCurPDWAD)
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
        Interface_InterfaceModel model = Model();
        Type tADR = typeof(StepAP214_AppliedDocumentReference);
        Type tPDWAD = typeof(StepBasic_ProductDefinitionWithAssociatedDocuments);
        int nb = model.NbEntities();

        // PTV 28.01.2003 CAX-IF TRJ11, file ext_ref_master.stp
        // search all ADR and PDWAD and exclude excess PDWADs
        NCollection_Sequence<Standard_Transient> aSeqOfADR = new NCollection_Sequence<Standard_Transient>();
        NCollection_Sequence<Standard_Transient> aSeqOfPDWAD = new NCollection_Sequence<Standard_Transient>();
        for (int ient = 1; ient <= nb; ient++)
        {
            Standard_Transient enti = model.Value(ient);
            if (enti.DynamicType() == tPDWAD)
            {
                aSeqOfPDWAD.Append(enti);
            }
            else if (enti.DynamicType() == tADR)
            {
                aSeqOfADR.Append(enti);
            }
        }
        int IsAP214 = 0;
        // run on sequence aSeqOfADR of ADR and remove excess PDWAD from aSeqOfPDWAD
        for (int adri = 1; adri <= aSeqOfADR.Length(); adri++)
        {
            StepAP214_AppliedDocumentReference ADR = aSeqOfADR.Value(adri) as StepAP214_AppliedDocumentReference;
            // looking for Product Definition Formation and exclude excess PDWAD from aSeqOfPDWAD
            StepBasic_ProductDefinitionWithAssociatedDocuments aPDWAD = null;
            findPDWADandExcludeExcess(ADR, aSeqOfPDWAD, Graph(), ref aPDWAD);

            // now add all necessary information as original implementation.
            IsAP214 = 1;
            StepBasic_RoleAssociation Role = null;
            StepBasic_ProductDefinition Shape = null;
            StepRepr_PropertyDefinitionRepresentation Format = null;
            StepBasic_DocumentRepresentationType Type = null;
            // AppliedDocumentReference with RoleAssociation...
            Interface_EntityIterator subs4 = Graph().Sharings(ADR);
            for (subs4.Start(); subs4.More(); subs4.Next())
            {
                if (subs4.Value().IsKind(typeof(StepBasic_RoleAssociation)))
                {
                    Role = subs4.Value() as StepBasic_RoleAssociation;
                }
            }

            subs4 = Graph().Shareds(ADR);
            for (subs4.Start(); subs4.More(); subs4.Next())
            {
                if (subs4.Value().IsKind(typeof(StepBasic_ProductDefinition)))
                {
                    Shape = subs4.Value() as StepBasic_ProductDefinition;
                }
            }
            // search for Document file
            StepBasic_DocumentFile DocFile = null;
            if (aPDWAD is null)
            { // shouldn't begin from TRJ11
                // lookinf from ADR
                subs4 = Graph().Shareds(ADR);
            }
            else
            {
                // looking from PDWAD
                subs4 = Graph().Shareds(aPDWAD);
            }
            for (subs4.Start(); subs4.More(); subs4.Next())
            {
                if (!subs4.Value().IsKind(typeof(StepBasic_DocumentFile)))
                {
                    continue;
                }
                DocFile = subs4.Value() as StepBasic_DocumentFile;
                if (DocFile is null)
                {
                    continue;
                }
                // for each DocumentFile, find associated with it data:
                Interface_EntityIterator subs = Graph().Sharings(DocFile);
                for (subs.Start(); subs.More(); subs.Next())
                {
                    Standard_Transient sub = subs.Value();

                    // FORMAT - ???????
                    //
                    // PDRs of a shape and of a file format
                    if (sub.IsKind(typeof(StepRepr_PropertyDefinition)))
                    {
                        StepRepr_PropertyDefinition PD = sub as StepRepr_PropertyDefinition;
                        Interface_EntityIterator subs2 = Graph().Sharings(PD);
                        for (subs2.Start(); subs2.More(); subs2.Next())
                        {
                            StepRepr_PropertyDefinitionRepresentation PDR = subs2.Value() as StepRepr_PropertyDefinitionRepresentation;
                            if (PDR is null)
                            {
                                continue;
                            }
                            if (PDR.UsedRepresentation().IsKind(typeof(StepShape_ShapeRepresentation)))
                            {
                                Format = PDR;
                            }
                        }
                    }
                    // DocumentRepresentationType
                    if (sub.IsKind(typeof(StepBasic_DocumentRepresentationType)))
                    {
                        Type = sub as StepBasic_DocumentRepresentationType;
                    }
                    if (Type is not null && Format is not null)
                    {
                        break;
                    }
                }
                if (Type is not null && Format is not null)
                {
                    break;
                }
            }
            if (DocFile is null)
            {
                continue;
            }
            myAEIAs.Append(ADR);
            myRoles.Append(Role);
            myFormats.Append(Format);
            myShapes.Append(Shape);
            myTypes.Append(Type);
            myIsAP214.Append(IsAP214);
            myDocFiles.Append(DocFile);
        } // end iterations on aSeqOfADR

        // now iterates on sequence aSeqOfPDWAD of Product Definition With Associated Documents
        for (int pdwadi = 1; pdwadi <= aSeqOfPDWAD.Length(); pdwadi++)
        {
            IsAP214 = 0;
            StepBasic_ProductDefinitionWithAssociatedDocuments aPDWAD =
                aSeqOfPDWAD.Value(pdwadi) as StepBasic_ProductDefinitionWithAssociatedDocuments;
            myShapes.Append(aPDWAD);
            myIsAP214.Append(IsAP214);
            myAEIAs.Append((Standard_Transient)null);
            myRoles.Append((Standard_Transient)null);
            myFormats.Append((Standard_Transient)null);
            myTypes.Append((Standard_Transient)null);
            myDocFiles.Append((Standard_Transient)null);
        }

        return myShapes.Length() > 0;
    }

    /// <summary>Returns number of defined extern references</summary>
    public int NbExternRefs()
    {
        return myShapes.Length();
    }

    /// <summary>
    /// OCCT.NET (unit 8Z): when set, tells <see cref="FileName"/> whether an external file exists (its name as the
    /// file gives it, the session's loaded file), in place of OCCT's look on the disk beside the loaded file.
    /// STEPCAFControl_Reader sets it from its resolver, or to false when it reads no external files.
    /// </summary>
    internal Func<string, string, bool>? InternalFileExists { get; set; }

    private bool probe(string theName, string theFullName)
    {
        if (InternalFileExists is null)
        {
            return fileExists(theFullName);
        }
        return InternalFileExists(theName, WS().LoadedFile() ?? "");
    }

    // OSD_Path's folder of the session's loaded file, and OSD_Path::AbsolutePath
    private static string directoryOf(string theFile)
    {
        if (string.IsNullOrEmpty(theFile))
        {
            return "";
        }
        string aDir = Path.GetDirectoryName(theFile);
        return aDir is null ? "" : aDir;
    }

    internal static string AbsolutePath(string theDirPath, string theRelFilePath)
    {
        try
        {
            if (Path.IsPathRooted(theRelFilePath))
            {
                return theRelFilePath;
            }
            if (string.IsNullOrEmpty(theDirPath))
            {
                return "";
            }
            return Path.GetFullPath(theRelFilePath, Path.GetFullPath(theDirPath));
        }
        catch (Exception)
        {
            return "";
        }
    }

    private static bool fileExists(string thePath)
    {
        try
        {
            return File.Exists(thePath);
        }
        catch (Exception)
        {
            return false;
        }
    }

    /// <summary>Returns filename for numth extern reference Returns Null if FileName is not defined or bad</summary>
    public string FileName(int num)
    {
        StepBasic_DocumentFile DocFile = null;
        StepAP214_AppliedExternalIdentificationAssignment AEIA = null;
        string aCStringFileName = null;
        if (myDocFiles.Length() >= num && myDocFiles.Value(num) is not null)
        {
            DocFile = myDocFiles.Value(num) as StepBasic_DocumentFile;
        }
        else if (myIsAP214.Value(num) == 1)
        {
            StepAP214_AppliedDocumentReference ADR = myAEIAs.Value(num) as StepAP214_AppliedDocumentReference;

            // PTV 28.01.2003 CAX-IF TRJ11, file ext_ref_master.stp
            // search document file name by long chain ADR->D<-DPE->PDF<-PDWAD->DF
            StepBasic_ProductDefinitionWithAssociatedDocuments aPDWAD = null;
            // create an empty aSeqOfPDWAD
            NCollection_Sequence<Standard_Transient> aSeqOfPDWAD = new NCollection_Sequence<Standard_Transient>();
            // we do not need to exclude, just find PDWAD
            findPDWADandExcludeExcess(ADR, aSeqOfPDWAD, Graph(), ref aPDWAD);

            // search for Document file
            Interface_EntityIterator subs4;
            if (aPDWAD is null)
            { // shouldn't begin from TRJ11
                // lookinf from ADR
                subs4 = Graph().Shareds(ADR);
            }
            else
            {
                // looking from PDWAD
                subs4 = Graph().Shareds(aPDWAD);
            }
            for (subs4.Start(); subs4.More(); subs4.Next())
            {
                if (!subs4.Value().IsKind(typeof(StepBasic_DocumentFile)))
                {
                    continue;
                }
                DocFile = subs4.Value() as StepBasic_DocumentFile;
                if (DocFile is null)
                {
                    continue;
                }
            }
        }
        else
        {
            StepBasic_ProductDefinitionWithAssociatedDocuments aPDWAD =
                myShapes.Value(num) as StepBasic_ProductDefinitionWithAssociatedDocuments;
            if (aPDWAD is null || aPDWAD.DocIds() is null)
            {
                return "";
            }
            for (int i = 1; i <= aPDWAD.NbDocIds(); i++)
            {
                StepBasic_Document Doc = aPDWAD.DocIdsValue(i);
                TCollection_HAsciiString aFilename = Doc.Name();
                if (aFilename is not null && !aFilename.IsEmpty())
                {
                    return aFilename.ToCString();
                }
            }
            return "";
        }
        // take name from AEIA and from DF
        if (DocFile is not null)
        {
            Interface_EntityIterator subs3 = Graph().Sharings(DocFile);
            for (subs3.Start(); subs3.More(); subs3.Next())
            {
                if (subs3.Value().IsKind(typeof(StepAP214_AppliedExternalIdentificationAssignment)))
                {
                    AEIA = subs3.Value() as StepAP214_AppliedExternalIdentificationAssignment;
                    if (AEIA is not null)
                    {
                        break;
                    }
                }
            }
        }
        if (AEIA is not null)
        {
            TCollection_HAsciiString aFilename = AEIA.AssignedId();
            if (aFilename is not null && !aFilename.IsEmpty())
            {
                aCStringFileName = aFilename.ToCString();
                // ptv 29.01.2003 file trj4_xr1-tc-214.stp entity #71 have id "#71"
                if (aCStringFileName.Length > 0 && aCStringFileName[0] == '#')
                {
                    aCStringFileName = null;
                }
            }
            if (string.IsNullOrEmpty(aCStringFileName))
            {
                // try to take name from external source
                StepBasic_ExternalSource theSource = AEIA.Source();
                if (theSource is not null)
                {
                    StepBasic_SourceItem theSourceId = theSource.SourceId();
                    if (theSourceId is not null && !theSourceId.IsNull())
                    {
                        StepData_SelectNamed theFileName = theSourceId.Value() as StepData_SelectNamed;
                        if (theFileName is null || theFileName.Kind() != 6)
                        {
                            // nothing to do, hope could take name later.
                        }
                        else
                        {
                            aCStringFileName = theFileName.String();
                        }
                    }
                }
            }
        }
        string oldFileName = null;
        // compute true path to the extern file
        string dpath = directoryOf(WS().LoadedFile());
        if (!string.IsNullOrEmpty(aCStringFileName))
        {
            string fullname = AbsolutePath(dpath, aCStringFileName);
            if (fullname.Length <= 0)
            {
                fullname = aCStringFileName;
            }
            if (!probe(aCStringFileName, fullname))
            {
                oldFileName = aCStringFileName;
                aCStringFileName = null;
            }
        }
        if (string.IsNullOrEmpty(aCStringFileName))
        {
            // try to find name of the directory from DocFile
            if (DocFile is not null)
            {
                TCollection_HAsciiString aFilename = DocFile.Id();
                if (aFilename is not null && !aFilename.IsEmpty())
                {
                    aCStringFileName = aFilename.ToCString();
                }
                if (string.IsNullOrEmpty(aCStringFileName))
                {
                    aFilename = DocFile.Name();
                    if (aFilename is not null && !aFilename.IsEmpty())
                    {
                        aCStringFileName = aFilename.ToCString();
                    }
                }
                if (string.IsNullOrEmpty(aCStringFileName))
                {
                    if (oldFileName is not null)
                    {
                        aCStringFileName = oldFileName;
                    }
                    else
                    {
                        return "";
                    }
                }
            }
        }
        if (aCStringFileName is null)
        {
            // OCCT: a null name (no document file) reaches OSD_Path::AbsolutePath and AssignCat; nothing is found
            aCStringFileName = oldFileName ?? "";
        }
        {
            string fullname = AbsolutePath(dpath, aCStringFileName);
            if (fullname.Length <= 0)
            {
                fullname = aCStringFileName;
            }
            if (!probe(aCStringFileName, fullname))
            {
                if (oldFileName is not null)
                {
                    aCStringFileName = oldFileName;
                }
                Transfer_TransientProcess aTP = WS().TransferReader().TransientProcess();
                string aMess = "Can not read external file " + aCStringFileName;
                aTP.AddFail(DocFile, aMess);
            }
            else
            {
                if (oldFileName is not null && oldFileName != aCStringFileName)
                {
                    Transfer_TransientProcess aTP = WS().TransferReader().TransientProcess();
                    string aMess = "External file with name from entity AEIA (" + oldFileName
                                   + ") not existed => use file name from DocumentFile entity - " + aCStringFileName;
                    aTP.AddWarning(DocFile, aMess);
                }
            }
        }
        return aCStringFileName;
    }

    /// <summary>
    /// Returns ProductDefinition to which numth extern reference is associated. Returns Null if cannot be detected or
    /// if extern reference is not associated to SDR in a proper way.
    /// </summary>
    public StepBasic_ProductDefinition ProdDef(int num)
    {
        return myShapes.Value(num) as StepBasic_ProductDefinition;
    }

    /// <summary>Returns DocumentFile to which numth extern reference is associated. Returns Null if cannot be detected.</summary>
    public StepBasic_DocumentFile DocFile(int num)
    {
        return myDocFiles.Value(num) as StepBasic_DocumentFile;
    }

    /// <summary>Returns format identification string for the extern document Returns Null handle if format is not defined</summary>
    public TCollection_HAsciiString Format(int num)
    {
        TCollection_HAsciiString Format = null;
        if (myIsAP214.Value(num) == 0)
        {
            return Format;
        }
        StepRepr_PropertyDefinitionRepresentation PDR = myFormats.Value(num) as StepRepr_PropertyDefinitionRepresentation;

        if (PDR is null)
        {
            return Format;
        }

        StepRepr_Representation rep = PDR.UsedRepresentation();
        for (int i = 1; i <= rep.NbItems(); i++)
        {
            if (rep.ItemsValue(i).IsKind(typeof(StepRepr_DescriptiveRepresentationItem)))
            {
                StepRepr_DescriptiveRepresentationItem DRI = rep.ItemsValue(i) as StepRepr_DescriptiveRepresentationItem;
                Format = DRI.Description();
                break;
            }
        }

        return Format;
    }

    /// <summary>
    /// Create a new external reference with specified attributes attached to a given SDR &lt;format&gt; can be Null
    /// string, in that case this information is not written. Else, it can be "STEP AP214" or "STEP AP203" Returns
    /// index of a new extern ref
    /// </summary>
    public int AddExternRef(string filename, StepBasic_ProductDefinition PD, string format)
    {
        TCollection_HAsciiString EmptyString = new TCollection_HAsciiString("");
        TCollection_HAsciiString fmt = new TCollection_HAsciiString(format);
        TCollection_HAsciiString tmp = new TCollection_HAsciiString("203");
        int np = fmt.Location(tmp, 1, fmt.Length());

        //  if( !(fmt==tmp) ) {
        if (!(np > 0))
        {
            // create core entity DocumentFile
            StepBasic_DocumentType DT = new StepBasic_DocumentType();
            DT.Init(EmptyString);
            TCollection_HAsciiString DFid = new TCollection_HAsciiString(filename);
            // PTV 30.01.2003 TRJ11 -  copy external filename as is
            //     DFid->AssignCat ( " file id" );
            StepBasic_DocumentFile DF = new StepBasic_DocumentFile();
            DF.Init(DFid, EmptyString, false, EmptyString, DT, EmptyString, false, EmptyString);

            // create AppliedExternalIdentificationAssignment et al
            StepBasic_IdentificationRole IR = new StepBasic_IdentificationRole();
            // PTV 30.01.2003 TRJ11
            //    - set the ("external document id and location", $) without unmeaning description
            TCollection_HAsciiString aName = new TCollection_HAsciiString("external document id and location");
            IR.SetName(aName);
            //     occ::handle<TCollection_HAsciiString> aIRdescr = new TCollection_HAsciiString("source
            //     system"); IR->Init(aName, true, aIRdescr);

            StepData_SelectNamed SDS = new StepData_SelectNamed();
            SDS.SetString(filename);
            SDS.SetName("IDENTIFIER");
            StepBasic_SourceItem SID = new StepBasic_SourceItem();
            SID.SetValue(SDS);
            StepBasic_ExternalSource ES = new StepBasic_ExternalSource();
            ES.Init(SID);

            StepAP214_ExternalIdentificationItem Item = new StepAP214_ExternalIdentificationItem();
            Item.SetValue(DF);
            NCollection_HArray1<StepAP214_ExternalIdentificationItem> Items =
                new NCollection_HArray1<StepAP214_ExternalIdentificationItem>(1, 1);
            Items.SetValue(1, Item);

            StepAP214_AppliedExternalIdentificationAssignment ExtIdent = new StepAP214_AppliedExternalIdentificationAssignment();
            //     ExtIdent->Init(EmptyString, IR, ES, Items);
            // PTV 30.01.2003 TRJ11 - store filename in AEIA
            TCollection_HAsciiString aFName = new TCollection_HAsciiString(filename);
            ExtIdent.Init(aFName, IR, ES, Items);

            // create DocumentRepresentationType
            TCollection_HAsciiString Dig = new TCollection_HAsciiString("digital");
            StepBasic_DocumentRepresentationType Type = new StepBasic_DocumentRepresentationType();
            Type.Init(Dig, DF);

            // create AppliedDocumentReference,
            StepAP214_AppliedDocumentReference ADR = new StepAP214_AppliedDocumentReference();
            // PTV 30.01.2003 TRJ11 - create additional entities for AP214
            addAP214ExterRef(ADR, PD, DF, filename);

            // create RoleAssociation etc.
            StepBasic_ObjectRole OR = new StepBasic_ObjectRole();
            TCollection_HAsciiString mandatory = new TCollection_HAsciiString("mandatory");
            OR.Init(mandatory, false, EmptyString);
            StepBasic_RoleSelect RS = new StepBasic_RoleSelect();
            RS.SetValue(ADR);
            StepBasic_RoleAssociation Role = new StepBasic_RoleAssociation();
            Role.Init(OR, RS);

            // create PDR for association with SR
            StepRepr_CharacterizedDefinition CD = new StepRepr_CharacterizedDefinition();
            CD.SetValue(DF);
            TCollection_HAsciiString PDname = new TCollection_HAsciiString("external definition");
            StepRepr_PropertyDefinition PropD = new StepRepr_PropertyDefinition();
            PropD.Init(PDname, true, EmptyString, CD);
            StepRepr_RepresentedDefinition RD = new StepRepr_RepresentedDefinition();
            RD.SetValue(PropD);
            //    occ::handle<StepRepr_PropertyDefinitionRepresentation> PDRshape = new
            //    StepRepr_PropertyDefinitionRepresentation; PDRshape->Init ( RD, SDR->UsedRepresentation()
            //    );

            // create PDR for definition of document format (if defined)
            StepRepr_PropertyDefinitionRepresentation PDRformat = null;
            if (!string.IsNullOrEmpty(format))
            {
                TCollection_HAsciiString RCftype = new TCollection_HAsciiString("document parameters");
                StepRepr_RepresentationContext RCf = new StepRepr_RepresentationContext();
                RCf.Init(EmptyString, RCftype);

                TCollection_HAsciiString DRIname = new TCollection_HAsciiString("data format");
                TCollection_HAsciiString DRIdscr = new TCollection_HAsciiString(format);
                StepRepr_DescriptiveRepresentationItem DRI = new StepRepr_DescriptiveRepresentationItem();
                DRI.Init(DRIname, DRIdscr);
                NCollection_HArray1<StepRepr_RepresentationItem> fItems = new NCollection_HArray1<StepRepr_RepresentationItem>(1, 1);
                fItems.SetValue(1, DRI);

                TCollection_HAsciiString SRfname = new TCollection_HAsciiString("document format");
                StepRepr_Representation SRformat = new StepRepr_Representation();
                SRformat.Init(SRfname, fItems, RCf);

                StepRepr_CharacterizedDefinition CDf = new StepRepr_CharacterizedDefinition();
                CDf.SetValue(DF);
                TCollection_HAsciiString PDfname = new TCollection_HAsciiString("document property");
                StepRepr_PropertyDefinition PDf = new StepRepr_PropertyDefinition();
                PDf.Init(PDfname, true, EmptyString, CDf);
                StepRepr_RepresentedDefinition RDf = new StepRepr_RepresentedDefinition();
                RDf.SetValue(PDf);

                PDRformat = new StepRepr_PropertyDefinitionRepresentation();
                PDRformat.Init(RDf, SRformat);
            }

            // add all the created root entities to sequences
            myAEIAs.Append(ExtIdent);    // StepAP214_AppliedExternalIdentificationAssignment
            myRoles.Append(Role);        // StepBasic_RoleAssociation
            myFormats.Append(PDRformat); // StepRepr_PropertyDefinitionRepresentation
            //    myShapes.Append ( PDRshape );    //StepRepr_PropertyDefinitionRepresentation
            myShapes.Append(PD);  // StepBasic_ProductDefinition
            myTypes.Append(Type); // StepBasic_DocumentRepresentationType
        }
        else
        { // format=="AP203"
            StepBasic_ProductDefinitionFormation ProdDefForm = PD.Formation();
            StepBasic_ProductDefinitionContext ProdDefCont = PD.FrameOfReference();

            // create document
            TCollection_HAsciiString fname = new TCollection_HAsciiString(filename);
            StepBasic_DocumentType aDocType = new StepBasic_DocumentType();
            TCollection_HAsciiString aDT = new TCollection_HAsciiString("cad_filename");
            aDocType.Init(aDT);
            StepBasic_Document aDoc = new StepBasic_Document();
            TCollection_HAsciiString aDescription = new TCollection_HAsciiString("CAD Model associated to the part");
            aDoc.Init(EmptyString, fname, true, aDescription, aDocType);
            NCollection_HArray1<StepBasic_Document> aDocIds = new NCollection_HArray1<StepBasic_Document>(1, 1);
            aDocIds.SetValue(1, aDoc);

            // create ProductDefinitionWithAssociatedDocuments
            aDescription = PD.Description();
            StepBasic_ProductDefinitionWithAssociatedDocuments PDWAD = new StepBasic_ProductDefinitionWithAssociatedDocuments();
            PDWAD.Init(EmptyString, aDescription, ProdDefForm, ProdDefCont, aDocIds);

            // search in graph for replace
            Interface_EntityIterator subs = Graph().Sharings(PD);
            for (subs.Start(); subs.More(); subs.Next())
            {
                Standard_Transient sub = subs.Value();
                if (!sub.IsKind(typeof(StepRepr_ProductDefinitionShape)))
                {
                    continue;
                }
                StepRepr_ProductDefinitionShape ProdDefSh = sub as StepRepr_ProductDefinitionShape;
                if (ProdDefSh is null)
                {
                    continue;
                }
                StepRepr_CharacterizedDefinition CDf = new StepRepr_CharacterizedDefinition();
                CDf.SetValue(PDWAD);
                ProdDefSh.SetDefinition(CDf);
            }

            int numProdDef = Model().Number(PD);
            Interface_EntityIterator subs2 = Graph().Sharings(PD);
            for (subs2.Start(); subs2.More(); subs2.Next())
            {
                Standard_Transient sub2 = subs2.Value();

                if (sub2.IsKind(typeof(StepRepr_NextAssemblyUsageOccurrence)))
                {
                    StepRepr_NextAssemblyUsageOccurrence NAUO = sub2 as StepRepr_NextAssemblyUsageOccurrence;
                    NAUO.SetRelatedProductDefinition(PDWAD);
                }

                if (sub2.IsKind(typeof(StepAP203_CcDesignPersonAndOrganizationAssignment)))
                {
                    StepAP203_CcDesignPersonAndOrganizationAssignment CDPAOA = sub2 as StepAP203_CcDesignPersonAndOrganizationAssignment;
                    NCollection_HArray1<StepAP203_PersonOrganizationItem> HAPOI = CDPAOA.Items();
                    for (int i = 1; i <= HAPOI.Length(); i++)
                    {
                        StepAP203_PersonOrganizationItem POI = HAPOI.Value(i);
                        StepBasic_ProductDefinition PDtmp = POI.ProductDefinition();
                        int numPDtmp = Model().Number(PDtmp);
                        if (numProdDef == numPDtmp)
                        {
                            // OCCT sets a copy and stores it back
                            StepAP203_PersonOrganizationItem aNew = new StepAP203_PersonOrganizationItem();
                            aNew.SetValue(PDWAD);
                            HAPOI.SetValue(i, aNew);
                        }
                    }
                }
                if (sub2.IsKind(typeof(StepAP203_CcDesignDateAndTimeAssignment)))
                {
                    StepAP203_CcDesignDateAndTimeAssignment CDDATA = sub2 as StepAP203_CcDesignDateAndTimeAssignment;
                    NCollection_HArray1<StepAP203_DateTimeItem> HADTI = CDDATA.Items();
                    for (int i = 1; i <= HADTI.Length(); i++)
                    {
                        StepAP203_DateTimeItem DTI = HADTI.Value(i);
                        StepBasic_ProductDefinition PDtmp = DTI.ProductDefinition();
                        int numPDtmp = Model().Number(PDtmp);
                        if (numProdDef == numPDtmp)
                        {
                            StepAP203_DateTimeItem aNew = new StepAP203_DateTimeItem();
                            aNew.SetValue(PDWAD);
                            HADTI.SetValue(i, aNew);
                        }
                    }
                }
                if (sub2.IsKind(typeof(StepAP203_CcDesignApproval)))
                {
                    StepAP203_CcDesignApproval CDA = sub2 as StepAP203_CcDesignApproval;
                    NCollection_HArray1<StepAP203_ApprovedItem> HAAI = CDA.Items();
                    for (int i = 1; i <= HAAI.Length(); i++)
                    {
                        StepAP203_ApprovedItem AI = HAAI.Value(i);
                        StepBasic_ProductDefinition PDtmp = AI.ProductDefinition();
                        int numPDtmp = Model().Number(PDtmp);
                        if (numProdDef == numPDtmp)
                        {
                            StepAP203_ApprovedItem aNew = new StepAP203_ApprovedItem();
                            aNew.SetValue(PDWAD);
                            HAAI.SetValue(i, aNew);
                        }
                    }
                }
            }

            myAEIAs.Append(PDWAD);
            myReplaceNum.Append(numProdDef);
            myRoles.Append(aDoc);
            myTypes.Append(aDocType);
        }
        return myAEIAs.Length();
    }

    /// <summary>Check (create if it is null) all shared entities for the model</summary>
    public void checkAP214Shared()
    {
        TCollection_HAsciiString EmptyString = new TCollection_HAsciiString("");
        if (mySharedPRPC is null)
        {
            // create new ProductRelatedProductCategory for all extern files.
            TCollection_HAsciiString PRPCname = new TCollection_HAsciiString("document");
            mySharedPRPC = new StepBasic_ProductRelatedProductCategory();
            mySharedPRPC.Init(PRPCname, false, EmptyString, null);
        }
        if (mySharedDocType is null)
        {
            // create new shared Document Type
            mySharedDocType = new StepBasic_DocumentType();
            TCollection_HAsciiString prod_dat_type = new TCollection_HAsciiString("configuration controlled document version");
            mySharedDocType.Init(prod_dat_type);
        }
        if (mySharedPDC is null)
        {
            // create new shared Product Definition Context
            mySharedPDC = new StepBasic_ProductDefinitionContext();
            TCollection_HAsciiString aPDCname = new TCollection_HAsciiString("digital document definition");
            StepBasic_ApplicationContext anAppContext = GetAP214APD().Application();
            mySharedPDC.Init(aPDCname, anAppContext, EmptyString);
        }
        if (mySharedPC is null)
        {
            // create new shared ProductContext
            mySharedPC = new StepBasic_ProductContext();
            StepBasic_ApplicationContext anAppContext = GetAP214APD().Application();
            mySharedPC.Init(EmptyString, anAppContext, EmptyString);
        }
    }

    /// <summary>Adds all the currently defined external refs to the model Returns number of written extern refs</summary>
    public int WriteExternRefs(int num)
    {
        if (num == 3)
        {
            for (int i = 1; i <= myAEIAs.Length(); i++)
            {
                Model().ReplaceEntity(myReplaceNum.Value(i), myAEIAs.Value(i));
                if (myRoles.Value(i) is not null)
                {
                    Model().AddWithRefs(myRoles.Value(i));
                }
                if (myTypes.Value(i) is not null)
                {
                    Model().AddWithRefs(myTypes.Value(i));
                }
            }
        }
        else
        {
            for (int i = 1; i <= myAEIAs.Length(); i++)
            {
                Model().AddWithRefs(myAEIAs.Value(i));
                if (myRoles.Value(i) is not null)
                {
                    Model().AddWithRefs(myRoles.Value(i));
                }
                if (myFormats.Value(i) is not null)
                {
                    Model().AddWithRefs(myFormats.Value(i));
                }
                if (myShapes.Value(i) is not null)
                {
                    Model().AddWithRefs(myShapes.Value(i));
                }
                if (myTypes.Value(i) is not null)
                {
                    Model().AddWithRefs(myTypes.Value(i));
                }
            }
        }
        // PTV 30.01.2003 TRJ11
        if (myAPD is not null)
        {
            Model().AddWithRefs(myAPD);
        }
        if (mySharedPRPC is not null)
        {
            Model().AddWithRefs(mySharedPRPC);
        }

        return myAEIAs.Length();
    }

    /// <summary>Set the ApplicationProtocolDefinition of the PDM schema</summary>
    public void SetAP214APD(StepBasic_ApplicationProtocolDefinition APD)
    {
        myAPD = APD;
    }

    /// <summary>
    /// Returns the ApplicationProtocolDefinition of the PDM schema NOTE: if not defined then create new APD with new
    /// Application Context
    /// </summary>
    public StepBasic_ApplicationProtocolDefinition GetAP214APD()
    {
        if (myAPD is null)
        {
            // create new APD with new Application Context
            myAPD = new StepBasic_ApplicationProtocolDefinition();
            // examples of the values taken from ext_ref_master.stp
            TCollection_HAsciiString status = new TCollection_HAsciiString("version 1.1");
            TCollection_HAsciiString appSchemaName = new TCollection_HAsciiString("pdm_schema");
            int intProtocolYear = 1999;
            StepBasic_ApplicationContext aApplication = new StepBasic_ApplicationContext();
            TCollection_HAsciiString EmptyString = new TCollection_HAsciiString("");
            aApplication.Init(EmptyString);
            myAPD.Init(status, appSchemaName, intProtocolYear, aApplication);
        }
        return myAPD;
    }

    /// <summary>
    /// Create a new additional structure entities and add necessary references Note: do not refer from ADR to DF
    /// directly in AP214 (TRJ11).
    /// </summary>
    protected bool addAP214ExterRef(StepAP214_AppliedDocumentReference ADR, StepBasic_ProductDefinition PD,
                                    StepBasic_DocumentFile DF, string filename)
    {
        NCollection_HArray1<StepAP214_DocumentReferenceItem> DRIs = new NCollection_HArray1<StepAP214_DocumentReferenceItem>(1, 1);
        StepAP214_DocumentReferenceItem aDRI = new StepAP214_DocumentReferenceItem();
        aDRI.SetValue(PD);
        DRIs.SetValue(1, aDRI);
        TCollection_HAsciiString EmptyString = new TCollection_HAsciiString("");

        // create/get created shared entities:
        // DocumentType, ProductDefinitionContext, ProductRelatedProductCategory, ProductContext
        checkAP214Shared();

        // create document
        StepBasic_Document aDocument = new StepBasic_Document();
        aDocument.Init(EmptyString, EmptyString, false, EmptyString, mySharedDocType);
        ADR.Init(aDocument, EmptyString, DRIs);

        // create new product
        StepBasic_Product Product = new StepBasic_Product();
        NCollection_HArray1<StepBasic_Product> HProducts = mySharedPRPC.Products();
        int nbProducts = 0;
        if (HProducts is not null)
        {
            nbProducts = HProducts.Length();
        }
        int intProdId = 20001 + nbProducts;
        TCollection_HAsciiString ProductID = new TCollection_HAsciiString(intProdId);
        TCollection_HAsciiString ProductName = new TCollection_HAsciiString(filename + "-Doc");
        NCollection_HArray1<StepBasic_ProductContext> aHProdContext = new NCollection_HArray1<StepBasic_ProductContext>(1, 1);
        aHProdContext.SetValue(1, mySharedPC);
        Product.Init(ProductID, ProductName, EmptyString, aHProdContext);

        // create new product definition formation
        StepBasic_ProductDefinitionFormation PDF = new StepBasic_ProductDefinitionFormation();
        // name id taken from example Standard_ExtString_ref_master.stp
        TCollection_HAsciiString PDF_ID = new TCollection_HAsciiString("1");
        PDF.Init(PDF_ID, EmptyString, Product);

        StepBasic_DocumentProductEquivalence DPE = new StepBasic_DocumentProductEquivalence();
        TCollection_HAsciiString DPEname = new TCollection_HAsciiString("equivalence");
        StepBasic_ProductOrFormationOrDefinition aPOFOD = new StepBasic_ProductOrFormationOrDefinition();
        aPOFOD.SetValue(PDF);
        DPE.Init(DPEname, false, EmptyString, aDocument, aPOFOD);
        // add to the model with references
        Model().AddWithRefs(DPE);

        // add products to shared PRPC
        NCollection_HArray1<StepBasic_Product> newHProducts = new NCollection_HArray1<StepBasic_Product>(1, nbProducts + 1);
        for (int pi = 1; pi <= nbProducts; pi++)
        {
            newHProducts.SetValue(pi, HProducts.Value(pi));
        }
        newHProducts.SetValue(nbProducts + 1, Product);
        // set the hArray to the PRPC
        mySharedPRPC.SetProducts(newHProducts);

        // create new PDWAD
        StepBasic_ProductDefinitionWithAssociatedDocuments PDWAD = new StepBasic_ProductDefinitionWithAssociatedDocuments();
        NCollection_HArray1<StepBasic_Document> aDocIds = new NCollection_HArray1<StepBasic_Document>(1, 1);
        aDocIds.SetValue(1, DF);
        TCollection_HAsciiString PDWAD_ID = new TCollection_HAsciiString("1");
        PDWAD.Init(PDWAD_ID, EmptyString, PDF, mySharedPDC, aDocIds);
        // add to the model with references
        Model().AddWithRefs(PDWAD);

        return true;
    }

    private readonly NCollection_Sequence<Standard_Transient> myAEIAs = new NCollection_Sequence<Standard_Transient>();
    private readonly NCollection_Sequence<Standard_Transient> myRoles = new NCollection_Sequence<Standard_Transient>();
    private readonly NCollection_Sequence<Standard_Transient> myFormats = new NCollection_Sequence<Standard_Transient>();
    private readonly NCollection_Sequence<Standard_Transient> myShapes = new NCollection_Sequence<Standard_Transient>();
    private readonly NCollection_Sequence<Standard_Transient> myTypes = new NCollection_Sequence<Standard_Transient>();
    private readonly NCollection_Sequence<int> myIsAP214 = new NCollection_Sequence<int>();
    private readonly NCollection_Sequence<int> myReplaceNum = new NCollection_Sequence<int>();
    private readonly NCollection_Sequence<Standard_Transient> myDocFiles = new NCollection_Sequence<Standard_Transient>();
    private StepBasic_ProductRelatedProductCategory mySharedPRPC;
    private StepBasic_DocumentType mySharedDocType;
    private StepBasic_ProductDefinitionContext mySharedPDC;
    private StepBasic_ProductContext mySharedPC;
    private StepBasic_ApplicationProtocolDefinition myAPD;
}
