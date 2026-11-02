// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_Reader.hxx
// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_Reader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.IFSelect_ReturnStatus;
using static Occt.TopAbs_ShapeEnum;
using static Occt.XdeDoc_ColorType;

namespace Occt;

/// <summary>
/// Provides a tool to read STEP file and put it into DECAF document. Besides transfer of shapes (including
/// assemblies) provided by STEPControl, supports also colors and part names
/// </summary>
/// <remarks>
/// <para>
/// This reader supports reading files with external references i.e. multifile reading It behaves as usual Reader
/// (from STEPControl) for the main file (e.g. if it is single file) Results of reading other files can be accessed
/// by name of the file or by iterating on a readers
/// </para>
/// <para>
/// OCCT.NET: the document is an <see cref="XdeDoc_Document"/> (decision 1166, 1231), whose labels and methods stand in
/// for OCAF's <c>TDF_Label</c>s and XCAF's tools; the reader fills it as OCCT fills an XCAF document, in OCCT's order.
/// Shapes, assemblies, external references, names, colours (surface, curve, per sub-shape, visibility), layers,
/// validation properties, the length unit and, if the session's <c>read.stepcaf.subshapes.name</c> asks, the
/// sub-shapes' names are read. SHUOs, GD&amp;T, materials, views and metadata are not (decision 13504): their modes are
/// kept, and their methods read nothing and return false. A colour of an instance of a sub-assembly's component (an
/// XCAF SHUO) is not kept (decision 13503). The session gets a STEPCAFControl_Controller of its own (decision 13201).
/// The document's shapes are those the reader publishes, frozen.
/// </para>
/// </remarks>
public class STEPCAFControl_Reader
{
    /// <summary>Creates a reader with an empty STEP model and sets ColorMode, LayerMode, NameMode and PropsMode to true.</summary>
    public STEPCAFControl_Reader()
    {
        myColorMode = true;
        myNameMode = true;
        myLayerMode = true;
        myPropsMode = true;
        myMetaMode = true;
        myProductMetaMode = false;
        mySHUOMode = false;
        myGDTMode = true;
        myMatMode = true;
        myViewMode = true;
        STEPCAFControl_Controller.Init();
        myReader = new STEPControl_Reader();
        if (myReader.WS() is not null)
        {
            Init(myReader.WS());
        }
    }

    /// <summary>
    /// Creates a reader tool and attaches it to an already existing Session Clears the session if it was not yet set
    /// for STEP
    /// </summary>
    public STEPCAFControl_Reader(XSControl_WorkSession WS, bool scratch = true)
    {
        myColorMode = true;
        myNameMode = true;
        myLayerMode = true;
        myPropsMode = true;
        myMetaMode = true;
        myProductMetaMode = false;
        mySHUOMode = false;
        myGDTMode = true;
        myMatMode = true;
        myViewMode = true;
        STEPCAFControl_Controller.Init();
        myReader = new STEPControl_Reader();
        Init(WS, scratch);
    }

    /// <summary>Clears the internal data structures and attaches to a new session Clears the session if it was not yet set for STEP</summary>
    public void Init(XSControl_WorkSession WS, bool scratch = true)
    {
        // necessary only in Writer, to set good actor:  WS->SelectNorm ( "STEP" );
        // OCCT.NET: the session's controller is a STEPCAFControl_Controller of its own (decision 13201)
        STEPCAFControl_Controller.SelectFor(WS);
        myReader.SetWS(WS, scratch);
        myFiles.Clear();
        myMap.Clear();
    }

    /// <summary>
    /// Converts a name read from the file: OCCT makes a TCollection_ExtendedString of it, treating it as UTF-8 if the
    /// model's source code page isn't NoConversion. The port's strings are decoded by the parser already.
    /// </summary>
    protected virtual string convertName(string theName)
    {
        return theName;
    }

    /// <summary>Loads a file and returns the read status Provided for use like single-file reader.</summary>
    public IFSelect_ReturnStatus ReadFile(string theFileName)
    {
        return myReader.ReadFile(theFileName);
    }

    /// <summary>Loads a file with the given parameters and returns the read status</summary>
    public IFSelect_ReturnStatus ReadFile(string theFileName, in DESTEP_Parameters theParams)
    {
        return myReader.ReadFile(theFileName, theParams);
    }

    /// <summary>Loads a file from stream and returns the read status.</summary>
    public IFSelect_ReturnStatus ReadStream(string theName, Stream theIStream)
    {
        return myReader.ReadStream(theName, theIStream);
    }

    /// <summary>Loads a file from stream with limits for hostile input (OCCT.NET, plan 2.7).</summary>
    public IFSelect_ReturnStatus ReadStream(string theName, Stream theIStream, Message_ProgressRange theProgress,
                                            StepFile_ReadLimits theLimits)
    {
        return myReader.ReadStream(theName, theIStream, theProgress, theLimits);
    }

    /// <summary>Returns number of roots recognized for transfer Shortcut for Reader().NbRootsForTransfer()</summary>
    public int NbRootsForTransfer()
    {
        return myReader.NbRootsForTransfer();
    }

    /// <summary>
    /// Translates currently loaded STEP file into the document Returns True if succeeded, and False in case of fail
    /// Provided for use like single-file reader
    /// </summary>
    public bool TransferOneRoot(int num, XdeDoc_Document doc, Message_ProgressRange theProgress = default)
    {
        NCollection_Sequence<XdeDoc_Label> Lseq = new NCollection_Sequence<XdeDoc_Label>();
        return Transfer(myReader, num, doc, Lseq, false, theProgress);
    }

    /// <summary>
    /// Translates currently loaded STEP file into the document Returns True if succeeded, and False in case of fail
    /// Provided for use like single-file reader
    /// </summary>
    public bool Transfer(XdeDoc_Document doc, Message_ProgressRange theProgress = default)
    {
        NCollection_Sequence<XdeDoc_Label> Lseq = new NCollection_Sequence<XdeDoc_Label>();
        return Transfer(myReader, 0, doc, Lseq, false, theProgress);
    }

    public bool Perform(string filename, XdeDoc_Document doc, Message_ProgressRange theProgress = default)
    {
        if (ReadFile(filename) != IFSelect_RetDone)
        {
            return false;
        }
        return Transfer(doc, theProgress);
    }

    public bool Perform(string filename, XdeDoc_Document doc, in DESTEP_Parameters theParams,
                        Message_ProgressRange theProgress = default)
    {
        if (ReadFile(filename, theParams) != IFSelect_RetDone)
        {
            return false;
        }
        return Transfer(doc, theProgress);
    }

    /// <summary>Returns data on external files Returns Null handle if no external files are read</summary>
    public NCollection_DataMap<string, STEPCAFControl_ExternFile> ExternFiles()
    {
        return myFiles;
    }

    /// <summary>Returns data on external file by its name Returns False if no external file with given name is read</summary>
    public bool ExternFile(string name, out STEPCAFControl_ExternFile ef)
    {
        ef = null;
        if (myFiles.IsEmpty() || !myFiles.IsBound(name))
        {
            return false;
        }
        ef = myFiles.Find(name);
        return true;
    }

    /// <summary>Returns basic reader</summary>
    public STEPControl_Reader ChangeReader()
    {
        return myReader;
    }

    /// <summary>Returns basic reader as const</summary>
    public STEPControl_Reader Reader()
    {
        return myReader;
    }

    /// <summary>The map of the shapes read to their labels</summary>
    public NCollection_DataMap<TopoDS_Shape, XdeDoc_Label> GetShapeLabelMap() => myMap;

    //=======================================================================
    // function : FillShapesMap
    // purpose  : auxiliary: fill a map by all compounds and their components
    //=======================================================================
    private static void FillShapesMap(TopoDS_Shape S, NCollection_Map<TopoDS_Shape> map)
    {
        TopoDS_Shape S0 = S.Located(new TopLoc_Location());
        map.Add(S0);
        if (S.ShapeType() != TopAbs_COMPOUND)
        {
            return;
        }
        for (TopoDS_Iterator it = new TopoDS_Iterator(S); it.More(); it.Next())
        {
            FillShapesMap(it.Value(), map);
        }
    }

    private void prepareUnits(StepData_StepModel theModel, XdeDoc_Document theDoc, ref StepData_Factors theLocalFactors)
    {
        double aScaleFactorMM = 1.0;
        if (theDoc.GetLengthUnit(out double aUnitM))
        {
            aScaleFactorMM = aUnitM * UnitsMethods.GetLengthUnitScale(UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Meter,
                                                                     UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Millimeter);
        }
        else
        {
            // OCCT: XSAlgo_ShapeProcessor::PrepareForTransfer() then the process-wide unit; here the session's
            // (decision 13202)
            aScaleFactorMM = UnitsMethods.GetCasCadeLengthUnit(myReader.WS().Statics());
            // Sets length unit to the document
            double aUnitValue = aScaleFactorMM * UnitsMethods.GetLengthUnitScale(UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Millimeter,
                                                                                UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Meter);
            theDoc.SetLengthUnit(aUnitValue, UnitsMethods.DumpLengthUnit(aScaleFactorMM, UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Millimeter));
        }
        theLocalFactors.SetCascadeUnit(aScaleFactorMM);
        theModel.SetLocalLengthUnit(aScaleFactorMM);
    }

    /// <summary>
    /// Translates STEP file already loaded into the reader into the document If num==0, translates all roots, else
    /// only root number num Returns True if succeeded, and False in case of fail If asOne is True, in case of multiple
    /// results composes them into assembly. Fills sequence of produced labels
    /// </summary>
    protected bool Transfer(STEPControl_Reader reader, int nroot, XdeDoc_Document doc,
                            NCollection_Sequence<XdeDoc_Label> Lseq, bool asOne = false,
                            Message_ProgressRange theProgress = default)
    {
        reader.ClearShapes();
        StepData_StepModel aModel = reader.Model() as StepData_StepModel;
        StepData_Factors aLocalFactors = new StepData_Factors();
        prepareUnits(aModel, doc, ref aLocalFactors);
        int i;

        // Read all shapes
        int num = reader.NbRootsForTransfer();
        if (num <= 0)
        {
            return false;
        }

        using Message_ProgressScope aPSRoot = new Message_ProgressScope(theProgress, null, 2);

        if (nroot != 0)
        {
            if (nroot > num)
            {
                return false;
            }
            reader.TransferOneRoot(nroot, aPSRoot.Next());
        }
        else
        {
            using Message_ProgressScope aPS = new Message_ProgressScope(aPSRoot.Next(), null, num);
            for (i = 1; i <= num && aPS.More(); i++)
            {
                reader.TransferOneRoot(i, aPS.Next());
            }
        }
        if (aPSRoot.UserBreak())
        {
            return false;
        }

        num = reader.NbShapes();
        if (num <= 0)
        {
            return false;
        }

        // Fill a map of (top-level) shapes resulting from that transfer
        // Only these shapes will be considered further
        NCollection_Map<TopoDS_Shape> ShapesMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        NCollection_Map<TopoDS_Shape> NewShapesMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        for (i = 1; i <= num; i++)
        {
            FillShapesMap(reader.Shape(i), ShapesMap);
        }

        // Collect information on shapes originating from SDRs
        // this will be used to distinguish compounds representing assemblies
        // from the ones representing hybrid models and shape sets
        // OCCT.NET: the map iterates in insertion order (ExpandSubShapes iterates it; decision 13505)
        NCollection_DataMap<TopoDS_Shape, StepBasic_ProductDefinition> ShapePDMap =
            new NCollection_DataMap<TopoDS_Shape, StepBasic_ProductDefinition>(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<StepBasic_ProductDefinition, STEPCAFControl_ExternFile> PDFileMap =
            new NCollection_DataMap<StepBasic_ProductDefinition, STEPCAFControl_ExternFile>(ReferenceEqualityComparer.Instance);
        Transfer_TransientProcess TP = reader.WS().TransferReader().TransientProcess();
        int nb = aModel.NbEntities();

        NCollection_HSequence<Standard_Transient> SeqPDS = new NCollection_HSequence<Standard_Transient>();

        for (i = 1; i <= nb; i++)
        {
            Standard_Transient enti = aModel.Value(i);
            if (enti.IsKind(typeof(StepRepr_ProductDefinitionShape)))
            {
                // sequence for acceleration ReadMaterials
                SeqPDS.Append(enti);
            }
            if (enti.IsKind(typeof(StepBasic_ProductDefinition)))
            {
                StepBasic_ProductDefinition PD = enti as StepBasic_ProductDefinition;
                int index = TP.MapIndex(PD);
                if (index > 0)
                {
                    Transfer_Binder binder = TP.MapItem(index);
                    TopoDS_Shape S = TransferBRep.ShapeResult(binder);
                    if (!S.IsNull() && ShapesMap.Contains(S))
                    {
                        NewShapesMap.Add(S);
                        ShapePDMap.Bind(S, PD);
                        PDFileMap.Bind(PD, null);
                    }
                }
            }
            if (enti.IsKind(typeof(StepShape_ShapeRepresentation)))
            {
                int index = TP.MapIndex(enti);
                if (index > 0)
                {
                    Transfer_Binder binder = TP.MapItem(index);
                    TopoDS_Shape S = TransferBRep.ShapeResult(binder);
                    if (!S.IsNull() && ShapesMap.Contains(S))
                    {
                        NewShapesMap.Add(S);
                    }
                }
            }
        }

        // get file name and directory name of the main file
        string aLoadedFile = reader.WS().LoadedFile() ?? "";
        string aMainName = Path.GetFileName(aLoadedFile);
        string dpath = Path.GetDirectoryName(aLoadedFile) ?? "";

        // Load external references (only for relevant SDRs)
        // and fill map SDR -> extern file
        STEPConstruct_ExternRefs ExtRefs = new STEPConstruct_ExternRefs(reader.WS());
        ExtRefs.LoadExternRefs();
        using Message_ProgressScope aPSE = new Message_ProgressScope(aPSRoot.Next(), null, ExtRefs.NbExternRefs());
        for (i = 1; i <= ExtRefs.NbExternRefs() && aPSE.More(); i++)
        {
            Message_ProgressRange aRange = aPSE.Next();
            // check extern ref format
            TCollection_HAsciiString format = ExtRefs.Format(i);
            if (format is not null)
            {
                TCollection_HAsciiString ap203 = new TCollection_HAsciiString("STEP AP203");
                TCollection_HAsciiString ap214 = new TCollection_HAsciiString("STEP AP214");
                if (!format.IsSameString(ap203, false) && !format.IsSameString(ap214, false))
                {
                    continue;
                }
            }

            // get and check filename of the current extern ref
            string filename = ExtRefs.FileName(i);

            if (string.IsNullOrEmpty(filename))
            {
                continue; // not a valid extern ref
            }

            // compute true path to the extern file
            string fullname = STEPConstruct_ExternRefs.AbsolutePath(dpath, filename);
            if (fullname.Length <= 0)
            {
                fullname = filename;
            }

            // check for not the same file
            string aMainFullName = STEPConstruct_ExternRefs.AbsolutePath(dpath, aMainName);
            if (string.Equals(aMainFullName, fullname, StringComparison.OrdinalIgnoreCase))
            {
                TP.AddWarning(ExtRefs.DocFile(i), "External reference file is the same main file");
                continue; // not a valid extern ref
            }

            // get and check PD associated with the current extern ref
            StepBasic_ProductDefinition PD = ExtRefs.ProdDef(i);
            if (PD is null)
            {
                continue; // not a valid extern ref
            }
            if (!PDFileMap.IsBound(PD))
            {
                continue; // this PD is not concerned by current transfer
            }

            // read extern file (or use existing data) and record its data
            STEPCAFControl_ExternFile EF = ReadExternFile(filename, fullname, doc, aRange);
            PDFileMap.Bind(PD, EF);
        }

        // and insert them to the document
        if (asOne)
        {
            Lseq.Append(AddShape(reader.OneShape(), doc, NewShapesMap, ShapePDMap, PDFileMap));
        }
        else
        {
            for (i = 1; i <= num; i++)
            {
                Lseq.Append(AddShape(reader.Shape(i), doc, NewShapesMap, ShapePDMap, PDFileMap));
            }
        }

        // read colors
        if (GetColorMode())
        {
            ReadColors(reader.WS(), doc, aLocalFactors);
        }

        // read names
        if (GetNameMode())
        {
            ReadNames(reader.WS(), doc, PDFileMap);
        }

        // read validation props
        if (GetPropsMode())
        {
            ReadValProps(reader.WS(), doc, PDFileMap, aLocalFactors);
        }

        // read layers
        if (GetLayerMode())
        {
            ReadLayers(reader.WS(), doc);
        }

        // read SHUO entities from STEP model
        if (GetSHUOMode())
        {
            ReadSHUOs(reader.WS(), doc, PDFileMap);
        }

        // read GDT entities from STEP model
        if (GetGDTMode())
        {
            ReadGDTs(reader.WS(), doc, aLocalFactors);
        }

        // read Material entities from STEP model
        if (GetMatMode())
        {
            ReadMaterials(reader.WS(), doc, SeqPDS, aLocalFactors);
        }

        // read View entities from STEP model
        if (GetViewMode())
        {
            ReadViews(reader.WS(), doc, aLocalFactors);
        }

        // read metadata
        if (GetMetaMode())
        {
            ReadMetadata(reader.WS(), doc, aLocalFactors);
        }

        // read product metadata
        if (GetProductMetaMode())
        {
            ReadProductMetadata(reader.WS(), doc);
        }

        // Expand resulting CAF structure for sub-shapes (optionally with their
        // names) if requested
        ExpandSubShapes(doc, ShapePDMap);

        // Update assembly compounds
        doc.UpdateAssemblies();
        return true;
    }

    /// <summary>
    /// Add a shape to a document Depending on a case, this shape can be added as one, or as assembly, or (in case if
    /// it is associated with external reference) taken as that referred shape
    /// </summary>
    protected XdeDoc_Label AddShape(TopoDS_Shape S, XdeDoc_Document STool, NCollection_Map<TopoDS_Shape> NewShapesMap,
                                    NCollection_DataMap<TopoDS_Shape, StepBasic_ProductDefinition> ShapePDMap,
                                    NCollection_DataMap<StepBasic_ProductDefinition, STEPCAFControl_ExternFile> PDFileMap)
    {
        // if shape has already been mapped, just return corresponding label
        if (myMap.IsBound(S))
        {
            return myMap.Find(S);
        }

        // if shape is located, create instance
        if (!S.Location().IsIdentity())
        {
            TopoDS_Shape S0 = S.Located(new TopLoc_Location());
            AddShape(S0, STool, NewShapesMap, ShapePDMap, PDFileMap);
            XdeDoc_Label L = STool.AddShape(S, false); // should create reference
            myMap.Bind(S, L);
            return L;
        }

        // if shape is not compound, simple add it
        if (S.ShapeType() != TopAbs_COMPOUND)
        {
            XdeDoc_Label L = STool.AddShape(S, false);
            myMap.Bind(S, L);
            return L;
        }

        // for compounds, compute number of subshapes and check whether this is assembly
        bool isAssembly = false;
        int nbComponents = 0;
        TopoDS_Iterator it = new TopoDS_Iterator();
        for (it.Initialize(S); it.More() && !isAssembly; it.Next(), nbComponents++)
        {
            TopoDS_Shape Sub0 = it.Value().Located(new TopLoc_Location());
            if (NewShapesMap.Contains(Sub0))
            {
                isAssembly = true;
            }
        }

        //  if(nbComponents>0) isAssembly = true;

        // check whether it has associated external ref
        NCollection_Sequence<string> SHAS = new NCollection_Sequence<string>();
        if (ShapePDMap.IsBound(S) && PDFileMap.IsBound(ShapePDMap.Find(S)))
        {
            STEPCAFControl_ExternFile EF = PDFileMap.Find(ShapePDMap.Find(S));
            if (EF is not null)
            {
                // (store information on extern refs in the document)
                SHAS.Append(EF.GetName().ToCString());
                // if yes, just return corresponding label
                if (!EF.GetLabel().IsNull())
                {
                    // but if components >0, ignore extern ref!
                    if (nbComponents <= 0)
                    {
                        myMap.Bind(S, EF.GetLabel());
                        STool.SetExternRefs(EF.GetLabel(), SHAS);
                        return EF.GetLabel();
                    }
                }
                // OCCT_DEBUG output left out
            }
        }

        // add compound either as a whole,
        if (!isAssembly)
        {
            XdeDoc_Label L = STool.AddShape(S, false);
            if (SHAS.Length() > 0)
            {
                STool.SetExternRefs(L, SHAS);
            }
            myMap.Bind(S, L);
            return L;
        }

        // or as assembly, component-by-component
        XdeDoc_Label LA = STool.NewShape();
        nbComponents = 0;
        for (it.Initialize(S); it.More(); it.Next(), nbComponents++)
        {
            TopoDS_Shape Sub0 = it.Value().Located(new TopLoc_Location());
            XdeDoc_Label subL = AddShape(Sub0, STool, NewShapesMap, ShapePDMap, PDFileMap);
            if (!subL.IsNull())
            {
                XdeDoc_Label instL = STool.AddComponent(LA, subL, it.Value().Location());
                if (!myMap.IsBound(it.Value()))
                {
                    myMap.Bind(it.Value(), instL);
                }
            }
        }
        if (SHAS.Length() > 0)
        {
            STool.SetExternRefs(LA, SHAS);
        }
        myMap.Bind(S, LA);
        // STool->SetShape ( L, S ); // it is necessary for assemblies OCC1747 // commemted by skl for
        // OCC2941

        return LA;
    }

    /// <summary>Reads (or if returns already read) extern file with given name</summary>
    /// <remarks>
    /// OCCT.NET: the external file is read from the disk (<paramref name="fullname"/>) by a session on the main
    /// session's parameter set (OCCT's statics are process-wide).
    /// </remarks>
    protected STEPCAFControl_ExternFile ReadExternFile(string file, string fullname, XdeDoc_Document doc,
                                                       Message_ProgressRange theProgress = default)
    {
        // if the file is already read, associate it with SDR
        if (myFiles.IsBound(file))
        {
            return myFiles.ChangeFind(file);
        }

        // create new WorkSession and Reader
        XSControl_WorkSession newWS = new XSControl_WorkSession(myReader.WS().Statics());
        STEPCAFControl_Controller.SelectFor(newWS);
        STEPControl_Reader sr = new STEPControl_Reader(newWS, false);

        // start to fill the resulting ExternFile structure
        STEPCAFControl_ExternFile EF = new STEPCAFControl_ExternFile();
        EF.SetWS(newWS);
        EF.SetName(new TCollection_HAsciiString(file));

        // read file
        EF.SetLoadStatus(sr.ReadFile(fullname));

        // transfer in single-result mode
        if (EF.GetLoadStatus() == IFSelect_RetDone)
        {
            NCollection_Sequence<XdeDoc_Label> labels = new NCollection_Sequence<XdeDoc_Label>();
            EF.SetTransferStatus(Transfer(sr, 0, doc, labels, false, theProgress));
            if (labels.Length() > 0)
            {
                EF.SetLabel(labels.Value(1));
            }
        }

        // add read file to dictionary
        myFiles.Bind(file, EF);

        return EF;
    }

    private static void findStyledSR(StepVisual_StyledItem style, ref StepShape_ShapeRepresentation aSR)
    {
        // search Shape Representation for component styled item
        for (int j = 1; j <= style.NbStyles(); j++)
        {
            StepVisual_PresentationStyleByContext PSA = style.StylesValue(j) as StepVisual_PresentationStyleByContext;
            if (PSA is null)
            {
                continue;
            }
            StepVisual_StyleContextSelect aStyleCntxSlct = PSA.StyleContext();
            StepShape_ShapeRepresentation aCurrentSR = aStyleCntxSlct?.Representation() as StepShape_ShapeRepresentation;
            if (aCurrentSR is null)
            {
                continue;
            }
            aSR = aCurrentSR;
            break;
        }
    }

    private static readonly XdeDoc_ColorType[] theColorTypes = [XdeDoc_ColorGen, XdeDoc_ColorSurf, XdeDoc_ColorCurv];

    //=======================================================================
    // function : propagateColorToParts
    // purpose  : auxiliary, propagate color styles from assemblies to parts
    //=======================================================================
    private static void propagateColorToParts(XdeDoc_Document theSTool, in XdeDoc_Label theRoot)
    {
        // collect components to propagate
        NCollection_Sequence<XdeDoc_Label> aComponents = new NCollection_Sequence<XdeDoc_Label>();
        if (theRoot.IsEqual(theSTool.ShapesLabel()))
        {
            theSTool.GetFreeShapes(aComponents);
        }
        else
        {
            theSTool.GetComponents(theRoot, aComponents);
        }

        // iterate each component
        foreach (XdeDoc_Label anIt in aComponents)
        {
            // get original label
            XdeDoc_Label anOriginalL = anIt;
            if (theSTool.GetReferredShape(anOriginalL, out XdeDoc_Label aReferred))
            {
                anOriginalL = aReferred;
            }

            // propagate to components without own colors
            foreach (XdeDoc_ColorType aType in theColorTypes)
            {
                if (theSTool.GetColor(theRoot, aType, out XdeDoc_Label aColorL)
                    && !theSTool.GetColor(anOriginalL, aType, out XdeDoc_Label _))
                {
                    theSTool.SetColor(anOriginalL, aColorL, aType);
                }
            }
            if (!theSTool.IsVisible(theRoot))
            {
                theSTool.SetVisibility(anOriginalL, false);
            }

            // propagate to next level children
            if (theSTool.IsAssembly(anOriginalL))
            {
                propagateColorToParts(theSTool, anOriginalL);
            }
        }
    }

    // the colours decoded as SetStyle and SetAssemblyComponentStyle decode them
    private static void decodeColors(StepVisual_Colour aSurfCol, StepVisual_Colour aBoundCol, StepVisual_Colour aCurveCol,
                                     STEPConstruct_RenderingProperties aRenderProps, out Quantity_ColorRGBA aFullSCol,
                                     out Quantity_Color aBCol, out Quantity_Color aCCol)
    {
        Quantity_Color aSCol = new Quantity_Color();
        aBCol = new Quantity_Color();
        aCCol = new Quantity_Color();
        aFullSCol = new Quantity_ColorRGBA();
        if (aSurfCol is not null)
        {
            STEPConstruct_Styles.DecodeColor(aSurfCol, ref aSCol);
            aFullSCol = new Quantity_ColorRGBA(aSCol);
        }
        if (aBoundCol is not null)
        {
            STEPConstruct_Styles.DecodeColor(aBoundCol, ref aBCol);
        }
        if (aCurveCol is not null)
        {
            STEPConstruct_Styles.DecodeColor(aCurveCol, ref aCCol);
        }
        if (aRenderProps.IsDefined())
        {
            aFullSCol = aRenderProps.GetRGBAColor();
        }
    }

    //=======================================================================
    // function : SetAssemblyComponentStyle
    // purpose  : auxiliary: set override style for assembly components
    //=======================================================================
    private static void SetAssemblyComponentStyle(Transfer_TransientProcess theTP, XdeDoc_Document theCTool,
                                                  STEPConstruct_Styles theStyles,
                                                  StepVisual_ContextDependentOverRidingStyledItem theStyle,
                                                  in StepData_Factors theLocalFactors)
    {
        if (theStyle is null)
        {
            return;
        }

        // check if it is component style
        bool anIsComponent = false;
        if (!theStyles.GetColors(theStyle, out StepVisual_Colour aSurfCol, out StepVisual_Colour aBoundCol,
                                 out StepVisual_Colour aCurveCol, out STEPConstruct_RenderingProperties aRenderProps,
                                 ref anIsComponent))
        {
            return;
        }

        Interface_Graph aGraph = theTP.Graph();
        TopLoc_Location aLoc = new TopLoc_Location(); // init;
        // find shape
        TopoDS_Shape aShape = new TopoDS_Shape();
        Transfer_Binder aBinder;
        StepShape_ShapeRepresentation aRepr = theStyle.ItemAP242()?.Value() as StepShape_ShapeRepresentation;
        if (aRepr is null)
        {
            return;
        }
        StepRepr_ShapeRepresentationRelationship aSRR = null;
        Interface_EntityIterator aSubs = theTP.Graph().Sharings(aRepr);
        for (aSubs.Start(); aSubs.More(); aSubs.Next())
        {
            Standard_Transient aSubsVal = aSubs.Value();
            if (aSubsVal.IsKind(typeof(StepRepr_ShapeRepresentationRelationship)))
            {
                aSRR = aSubsVal as StepRepr_ShapeRepresentationRelationship;
            }
        }

        aBinder = theTP.Find(aSRR);
        if (aBinder is not null)
        {
            aShape = TransferBRep.ShapeResult(aBinder);
        }
        if (aShape.IsNull())
        {
            return;
        }

        // get transformation
        aSubs = aGraph.Shareds(theStyle);
        aSubs.Start();
        for (; aSubs.More(); aSubs.Next())
        {
            StepRepr_ShapeRepresentationRelationshipWithTransformation aRelation =
                aSubs.Value() as StepRepr_ShapeRepresentationRelationshipWithTransformation;
            if (aRelation is null)
            {
                continue;
            }

            StepRepr_Transformation aTransf = aRelation.TransformationOperator();
            StepRepr_ItemDefinedTransformation anItemTransf = aTransf?.ItemDefinedTransformation();
            if (anItemTransf is not null)
            {
                StepGeom_Axis2Placement3d anAxp1 = anItemTransf.TransformItem1() as StepGeom_Axis2Placement3d;
                StepGeom_Axis2Placement3d anAxp2 = anItemTransf.TransformItem2() as StepGeom_Axis2Placement3d;

                if (anAxp1 is not null && anAxp2 is not null)
                {
                    Geom_Axis2Placement anOrig = StepToGeom.MakeAxis2Placement(anAxp1, theLocalFactors);
                    Geom_Axis2Placement aTarg = StepToGeom.MakeAxis2Placement(anAxp2, theLocalFactors);
                    gp_Ax3 anAx3Orig = new gp_Ax3(anOrig.Ax2());
                    gp_Ax3 anAx3Targ = new gp_Ax3(aTarg.Ax2());

                    gp_Trsf aTr1 = new gp_Trsf();
                    aTr1.SetTransformation(anAx3Targ, anAx3Orig);
                    TopLoc_Location aLoc1 = new TopLoc_Location(aTr1);
                    aLoc = aLoc.Multiplied(aLoc1);
                }
            }
        }

        aShape = aShape.Located(aLoc);

        if (aSurfCol is not null || aBoundCol is not null || aCurveCol is not null || aRenderProps.IsDefined())
        {
            decodeColors(aSurfCol, aBoundCol, aCurveCol, aRenderProps, out Quantity_ColorRGBA aFullSCol,
                         out Quantity_Color aBCol, out Quantity_Color aCCol);

            if (aSurfCol is not null || aRenderProps.IsDefined())
            {
                theCTool.SetInstanceColor(aShape, XdeDoc_ColorSurf, aFullSCol);
            }
            if (aBoundCol is not null)
            {
                theCTool.SetInstanceColor(aShape, XdeDoc_ColorCurv, aBCol);
            }
            if (aCurveCol is not null)
            {
                theCTool.SetInstanceColor(aShape, XdeDoc_ColorCurv, aCCol);
            }
        }
    }

    //=======================================================================
    // function : SetStyle
    // purpose  : auxiliary: set style for parts and instances
    //=======================================================================
    private static void SetStyle(XSControl_WorkSession theWS, NCollection_DataMap<TopoDS_Shape, XdeDoc_Label> theMap,
                                 XdeDoc_Document theSTool, STEPConstruct_Styles theStyles,
                                 NCollection_HSequence<Standard_Transient> theHSeqOfInvisStyle,
                                 StepVisual_StyledItem theStyle, in StepData_Factors theLocalFactors)
    {
        if (theStyle is null)
        {
            return;
        }

        Transfer_TransientProcess aTP = theWS.TransferReader().TransientProcess();
        if (theStyle is StepVisual_OverRidingStyledItem anOverridingStyle)
        {
            SetStyle(theWS, theMap, theSTool, theStyles, theHSeqOfInvisStyle, anOverridingStyle.OverRiddenStyle(),
                     theLocalFactors);
            if (theStyle is StepVisual_ContextDependentOverRidingStyledItem anAssemblyComponentStyle)
            {
                SetAssemblyComponentStyle(aTP, theSTool, theStyles, anAssemblyComponentStyle, theLocalFactors);
                return;
            }
        }

        bool anIsVisible = true;
        // check the visibility of styled item.
        for (int si = 1; si <= theHSeqOfInvisStyle.Length(); si++)
        {
            if (theStyle != theHSeqOfInvisStyle.Value(si))
            {
                continue;
            }
            // found that current style is invisible.
            anIsVisible = false;
            break;
        }

        // check if it is component style
        bool anIsComponent = false;
        if (!theStyles.GetColors(theStyle, out StepVisual_Colour aSurfCol, out StepVisual_Colour aBoundCol,
                                 out StepVisual_Colour aCurveCol, out STEPConstruct_RenderingProperties aRenderProps,
                                 ref anIsComponent)
            && anIsVisible)
        {
            return;
        }

        // collect styled items
        List<StepVisual_StyledItemTarget> anItems = new List<StepVisual_StyledItemTarget>();
        if (theStyle.ItemAP242() is not null && !theStyle.ItemAP242().IsNull())
        {
            anItems.Add(theStyle.ItemAP242());
        }

        for (int itemIt = 0; itemIt < anItems.Count; itemIt++)
        {
            int anIndex = aTP.MapIndex(anItems[itemIt].Value());
            TopoDS_Shape aS = new TopoDS_Shape();
            if (anIndex > 0)
            {
                Transfer_Binder aBinder = aTP.MapItem(anIndex);
                aS = TransferBRep.ShapeResult(aBinder);
            }
            bool isSkipSHUOstyle = false;
            // take shape with real location.
            while (anIsComponent)
            {
                // take SR of NAUO
                StepShape_ShapeRepresentation aSR = null;
                findStyledSR(theStyle, ref aSR);
                // search for SR along model
                if (aSR is null)
                {
                    break;
                }
                Interface_EntityIterator aSubs = theWS.HGraph().Graph().Sharings(aSR);
                StepShape_ShapeDefinitionRepresentation aSDR;
                for (aSubs.Start(); aSubs.More(); aSubs.Next())
                {
                    aSDR = aSubs.Value() as StepShape_ShapeDefinitionRepresentation;
                    if (aSDR is null)
                    {
                        continue;
                    }
                    StepRepr_RepresentedDefinition aPDSselect = aSDR.Definition();
                    StepRepr_ProductDefinitionShape PDS = aPDSselect.PropertyDefinition() as StepRepr_ProductDefinitionShape;
                    if (PDS is null)
                    {
                        continue;
                    }
                    StepRepr_CharacterizedDefinition aCharDef = PDS.Definition();

                    StepRepr_AssemblyComponentUsage ACU = aCharDef.ProductDefinitionRelationship() as StepRepr_AssemblyComponentUsage;
                    if (ACU is null)
                    {
                        continue;
                    }
                    // PTV 10.02.2003 skip styled item that refer to SHUO
                    if (ACU.IsKind(typeof(StepRepr_SpecifiedHigherUsageOccurrence)))
                    {
                        isSkipSHUOstyle = true;
                        break;
                    }
                    StepRepr_NextAssemblyUsageOccurrence NAUO = ACU as StepRepr_NextAssemblyUsageOccurrence;
                    if (NAUO is null)
                    {
                        continue;
                    }

                    // PTV 10.02.2003 to find component of assembly CORRECTLY
                    STEPConstruct_Tool aTool = new STEPConstruct_Tool(theWS);
                    XdeDoc_Label aShLab = FindInstance(NAUO, theSTool, aTool, theMap);
                    TopoDS_Shape aSh = aShLab.IsNull() ? new TopoDS_Shape() : theSTool.GetShape(aShLab);
                    if (!aSh.IsNull())
                    {
                        aS = aSh;
                        break;
                    }
                }
                break;
            }
            if (isSkipSHUOstyle)
            {
                continue; // skip styled item which refer to SHUO
            }

            if (aS.IsNull())
            {
                continue;
            }

            if (aSurfCol is not null || aBoundCol is not null || aCurveCol is not null || aRenderProps.IsDefined()
                || !anIsVisible)
            {
                bool isFound = theSTool.SearchUsingMap(aS, out XdeDoc_Label aL, false, true);
                if (aSurfCol is not null || aBoundCol is not null || aCurveCol is not null || aRenderProps.IsDefined())
                {
                    decodeColors(aSurfCol, aBoundCol, aCurveCol, aRenderProps, out Quantity_ColorRGBA aFullSCol,
                                 out Quantity_Color aBCol, out Quantity_Color aCCol);
                    if (isFound)
                    {
                        if (aSurfCol is not null || aRenderProps.IsDefined())
                        {
                            theSTool.SetColor(aL, aFullSCol, XdeDoc_ColorSurf);
                        }
                        if (aBoundCol is not null)
                        {
                            theSTool.SetColor(aL, aBCol, XdeDoc_ColorCurv);
                        }
                        if (aCurveCol is not null)
                        {
                            theSTool.SetColor(aL, aCCol, XdeDoc_ColorCurv);
                        }
                    }
                    else
                    {
                        for (TopoDS_Iterator it = new TopoDS_Iterator(aS); it.More(); it.Next())
                        {
                            if (theSTool.SearchUsingMap(it.Value(), out XdeDoc_Label aL1, false, true))
                            {
                                if (aSurfCol is not null || aRenderProps.IsDefined())
                                {
                                    theSTool.SetColor(aL1, aFullSCol, XdeDoc_ColorSurf);
                                }
                                if (aBoundCol is not null)
                                {
                                    theSTool.SetColor(aL1, aBCol, XdeDoc_ColorCurv);
                                }
                                if (aCurveCol is not null)
                                {
                                    theSTool.SetColor(aL1, aCCol, XdeDoc_ColorCurv);
                                }
                            }
                        }
                    }
                }
                if (!anIsVisible)
                {
                    // sets the invisibility for shape.
                    if (isFound)
                    {
                        theSTool.SetVisibility(aL, false);
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : IsOverriden
    // purpose  : auxiliary: check that style is overridden
    //=======================================================================
    private static bool IsOverriden(Interface_Graph theGraph, StepVisual_StyledItem theStyle, bool theIsRoot)
    {
        Interface_EntityIterator aSubs = theGraph.Sharings(theStyle);
        aSubs.Start();
        for (; aSubs.More(); aSubs.Next())
        {
            StepVisual_OverRidingStyledItem anOverRidingStyle = aSubs.Value() as StepVisual_OverRidingStyledItem;
            if (anOverRidingStyle is not null)
            {
                if (!theIsRoot)
                {
                    return true;
                }
                // for root style returns true only if it is overridden by other root style
                Standard_Transient anItem = anOverRidingStyle.ItemAP242()?.Value();
                if (anItem is not null && anItem.IsKind(typeof(StepShape_ShapeRepresentation)))
                {
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>
    /// Reads style assignments from STEP model and sets corresponding color assignments in the DECAF document
    /// </summary>
    protected bool ReadColors(XSControl_WorkSession WS, XdeDoc_Document Doc, in StepData_Factors theLocalFactors)
    {
        STEPConstruct_Styles Styles = new STEPConstruct_Styles(WS);
        if (!Styles.LoadStyles())
        {
            // OCCT_DEBUG output left out
            return false;
        }
        // searching for invisible items in the model
        NCollection_HSequence<Standard_Transient> aHSeqOfInvisStyle = new NCollection_HSequence<Standard_Transient>();
        Styles.LoadInvisStyles(ref aHSeqOfInvisStyle);

        Interface_Graph aGraph = Styles.Graph();

        // parse and search for color attributes
        int nb = Styles.NbRootStyles();
        // apply root styles earlier, as they can be overridden
        // function IsOverriden for root style returns true only if it is overridden by other root style
        bool anIsRootStyle = true;
        for (int i = 1; i <= nb; i++)
        {
            StepVisual_StyledItem Style = Styles.RootStyle(i);
            // check that style is overridden by other root style
            if (!IsOverriden(aGraph, Style, anIsRootStyle))
            {
                SetStyle(WS, myMap, Doc, Styles, aHSeqOfInvisStyle, Style, theLocalFactors);
            }
        }

        nb = Styles.NbStyles();
        // apply leaf styles, they can override root styles
        anIsRootStyle = false;
        for (int i = 1; i <= nb; i++)
        {
            StepVisual_StyledItem Style = Styles.Style(i);
            // check that style is overridden
            if (!IsOverriden(aGraph, Style, anIsRootStyle))
            {
                SetStyle(WS, myMap, Doc, Styles, aHSeqOfInvisStyle, Style, theLocalFactors);
            }
        }

        // OCCT: CTool->ReverseChainsOfTreeNodes(), the order of the shapes linked to a colour label, which the
        // document doesn't keep (decision 13503)

        // some colors can be attached to assemblies, propagate them to components
        propagateColorToParts(Doc, Doc.ShapesLabel());
        return true;
    }

    private static XdeDoc_Label GetLabelFromPD(StepBasic_ProductDefinition PD, XdeDoc_Document STool,
                                               Transfer_TransientProcess TP,
                                               NCollection_DataMap<StepBasic_ProductDefinition, STEPCAFControl_ExternFile> PDFileMap,
                                               NCollection_DataMap<TopoDS_Shape, XdeDoc_Label> ShapeLabelMap)
    {
        XdeDoc_Label L = default;
        if (PDFileMap.IsBound(PD))
        {
            STEPCAFControl_ExternFile EF = PDFileMap.Find(PD);
            if (EF is not null)
            {
                L = EF.GetLabel();
                if (!L.IsNull())
                {
                    return L;
                }
            }
        }

        TopoDS_Shape S;
        Transfer_Binder binder = TP.Find(PD);
        if (binder is null || !binder.HasResult())
        {
            return L;
        }
        S = TransferBRep.ShapeResult(TP, binder);
        if (S.IsNull())
        {
            return L;
        }

        if (ShapeLabelMap.IsBound(S))
        {
            L = ShapeLabelMap.Find(S);
        }
        if (L.IsNull())
        {
            STool.Search(S, out L, true, true, false);
        }
        return L;
    }

    /// <summary>
    /// Find a label of the instance of a component (NAUO) in the document, through the shape it was read into.
    /// </summary>
    public static XdeDoc_Label FindInstance(StepRepr_NextAssemblyUsageOccurrence NAUO, XdeDoc_Document STool,
                                            STEPConstruct_Tool Tool, NCollection_DataMap<TopoDS_Shape, XdeDoc_Label> ShapeLabelMap)
    {
        XdeDoc_Label L = default;

        // get shape resulting from CDSR (in fact, only location is interesting)
        Transfer_TransientProcess TP = Tool.TransientProcess();
        Transfer_Binder binder = TP.Find(NAUO);
        if (binder is null || !binder.HasResult())
        {
            // OCCT_DEBUG output left out
            return L;
        }

        TopoDS_Shape S = TransferBRep.ShapeResult(TP, binder);
        if (S.IsNull())
        {
            // OCCT_DEBUG output left out
            return L;
        }

        if (ShapeLabelMap.IsBound(S))
        {
            L = ShapeLabelMap.Find(S);
        }
        else
        {
            STool.Search(S, out L, true, true, false);
        }

        return L;
    }

    /// <summary>
    /// Reads names of parts defined in the STEP model and assigns them to corresponding labels in the DECAF document
    /// </summary>
    protected bool ReadNames(XSControl_WorkSession WS, XdeDoc_Document Doc,
                             NCollection_DataMap<StepBasic_ProductDefinition, STEPCAFControl_ExternFile> PDFileMap)
    {
        // get starting data
        Interface_InterfaceModel Model = WS.Model();
        XSControl_TransferReader TR = WS.TransferReader();
        Transfer_TransientProcess TP = TR.TransientProcess();
        STEPConstruct_Tool Tool = new STEPConstruct_Tool(WS);

        // iterate on model to find all SDRs and CDSRs
        int nb = Model.NbEntities();
        Type tNAUO = typeof(StepRepr_NextAssemblyUsageOccurrence);
        Type tPD = typeof(StepBasic_ProductDefinition);
        Type tPDWAD = typeof(StepBasic_ProductDefinitionWithAssociatedDocuments);
        TCollection_HAsciiString name = null;
        XdeDoc_Label L;
        Interface_Graph aGraph = null;
        for (int i = 1; i <= nb; i++)
        {
            Standard_Transient enti = Model.Value(i);

            // get description of NAUO
            if (enti.DynamicType() == tNAUO)
            {
                L = default;
                StepRepr_NextAssemblyUsageOccurrence NAUO = enti as StepRepr_NextAssemblyUsageOccurrence;
                if (NAUO is null)
                {
                    continue;
                }
                // OCCT: WS->Graph(), which recomputes the graph only if the model changed (it doesn't here)
                aGraph ??= WS.Graph();
                Interface_EntityIterator subs = aGraph.Sharings(NAUO);
                for (subs.Start(); subs.More(); subs.Next())
                {
                    StepRepr_ProductDefinitionShape PDS = subs.Value() as StepRepr_ProductDefinitionShape;
                    if (PDS is null)
                    {
                        continue;
                    }
                    StepBasic_ProductDefinitionRelationship PDR = PDS.Definition().ProductDefinitionRelationship();
                    if (PDR is null)
                    {
                        continue;
                    }
                    if (PDR.HasDescription() && PDR.Description().UsefullLength() > 0)
                    {
                        name = PDR.Description();
                    }
                    else if (PDR.Name() is not null && PDR.Name().UsefullLength() > 0)
                    {
                        name = PDR.Name();
                    }
                    else if (PDR.Id() is not null)
                    {
                        name = PDR.Id();
                    }
                    else
                    {
                        name = new TCollection_HAsciiString();
                    }
                }
                // find proper label
                L = FindInstance(NAUO, Doc, Tool, myMap);
                if (L.IsNull())
                {
                    continue;
                }

                // OCCT: a NAUO no PDS refers to keeps the name found before (null at the first: OCCT would crash)
                string str = convertName(name is null ? "" : name.ToCString());
                Doc.SetName(L, str);
            }

            // for PD get name of associated product
            if (enti.DynamicType() == tPD || enti.DynamicType() == tPDWAD)
            {
                L = default;
                StepBasic_ProductDefinition PD = enti as StepBasic_ProductDefinition;
                if (PD is null)
                {
                    continue;
                }
                StepBasic_Product Prod = PD.Formation() is not null ? PD.Formation().OfProduct() : null;
                if (Prod is null)
                {
                    name = new TCollection_HAsciiString();
                }
                else if (Prod.Name() is not null && Prod.Name().UsefullLength() > 0)
                {
                    name = Prod.Name();
                }
                else if (Prod.Id() is not null)
                {
                    name = Prod.Id();
                }
                else
                {
                    name = new TCollection_HAsciiString();
                }
                L = GetLabelFromPD(PD, Doc, TP, PDFileMap, myMap);
                if (L.IsNull())
                {
                    continue;
                }
                string str = convertName(name.ToCString());
                Doc.SetName(L, str);
            }
            // set a name to the document
            // TCollection_ExtendedString str = convertName (name->String());
            // TDataStd_Name::Set ( L, str );
        }

        return true;
    }

    private static XdeDoc_Label GetLabelFromPD(StepBasic_ProductDefinition PD, XdeDoc_Document STool,
                                               STEPConstruct_ValidationProps Props,
                                               NCollection_DataMap<StepBasic_ProductDefinition, STEPCAFControl_ExternFile> PDFileMap,
                                               NCollection_DataMap<TopoDS_Shape, XdeDoc_Label> ShapeLabelMap)
    {
        XdeDoc_Label L = default;
        if (PDFileMap.IsBound(PD))
        {
            STEPCAFControl_ExternFile EF = PDFileMap.Find(PD);
            if (EF is not null)
            {
                L = EF.GetLabel();
                if (!L.IsNull())
                {
                    return L;
                }
            }
        }
        TopoDS_Shape S = Props.GetPropShape(PD);
        if (S.IsNull())
        {
            return L;
        }
        if (ShapeLabelMap.IsBound(S))
        {
            L = ShapeLabelMap.Find(S);
        }
        if (L.IsNull())
        {
            STool.Search(S, out L, true, true, false);
        }
        return L;
    }

    /// <summary>
    /// Reads validation properties assigned to shapes in the STEP model and assigns them to corresponding labels in
    /// the DECAF document
    /// </summary>
    protected bool ReadValProps(XSControl_WorkSession WS, XdeDoc_Document Doc,
                                NCollection_DataMap<StepBasic_ProductDefinition, STEPCAFControl_ExternFile> PDFileMap,
                                in StepData_Factors theLocalFactors)
    {
        // get starting data
        XSControl_TransferReader TR = WS.TransferReader();
        Transfer_TransientProcess TP = TR.TransientProcess();

        // load props from the STEP model
        NCollection_Sequence<Standard_Transient> props = new NCollection_Sequence<Standard_Transient>();
        STEPConstruct_ValidationProps Props = new STEPConstruct_ValidationProps(WS);
        if (!Props.LoadProps(props))
        {
            // OCCT_DEBUG output left out
            return false;
        }

        // interpret props one by one
        for (int i = 1; i <= props.Length(); i++)
        {
            StepRepr_PropertyDefinitionRepresentation PDR = props.Value(i) as StepRepr_PropertyDefinitionRepresentation;
            if (PDR is null)
            {
                continue;
            }

            XdeDoc_Label L = default;

            StepRepr_PropertyDefinition PD = PDR.Definition().PropertyDefinition();
            Interface_EntityIterator subs = Props.Graph().Shareds(PD);
            for (subs.Start(); L.IsNull() && subs.More(); subs.Next())
            {
                if (subs.Value().IsKind(typeof(StepRepr_ProductDefinitionShape)))
                {
                    StepRepr_ProductDefinitionShape PDS = subs.Value() as StepRepr_ProductDefinitionShape;
                    if (PDS is null)
                    {
                        continue;
                    }
                    // find corresponding NAUO
                    StepRepr_NextAssemblyUsageOccurrence NAUO = null;
                    Interface_EntityIterator subs1 = Props.Graph().Shareds(PDS);
                    for (subs1.Start(); NAUO is null && subs1.More(); subs1.Next())
                    {
                        if (subs1.Value().IsKind(typeof(StepRepr_NextAssemblyUsageOccurrence)))
                        {
                            NAUO = subs1.Value() as StepRepr_NextAssemblyUsageOccurrence;
                        }
                    }
                    if (NAUO is not null)
                    {
                        L = FindInstance(NAUO, Doc, new STEPConstruct_Tool(WS), myMap);
                        if (L.IsNull())
                        {
                            continue;
                        }
                    }
                    else
                    {
                        // find corresponding ProductDefinition:
                        StepBasic_ProductDefinition ProdDef = null;
                        Interface_EntityIterator subsPDS = Props.Graph().Shareds(PDS);
                        for (subsPDS.Start(); ProdDef is null && subsPDS.More(); subsPDS.Next())
                        {
                            if (subsPDS.Value().IsKind(typeof(StepBasic_ProductDefinition)))
                            {
                                ProdDef = subsPDS.Value() as StepBasic_ProductDefinition;
                            }
                        }
                        if (ProdDef is null)
                        {
                            continue;
                        }
                        L = GetLabelFromPD(ProdDef, Doc, Props, PDFileMap, myMap);
                    }
                }

                if (subs.Value().IsKind(typeof(StepRepr_ShapeAspect)))
                {
                    StepRepr_ShapeAspect SA = subs.Value() as StepRepr_ShapeAspect;
                    if (SA is null)
                    {
                        continue;
                    }
                    // find ShapeRepresentation
                    StepShape_ShapeRepresentation SR = null;
                    Interface_EntityIterator subs1 = Props.Graph().Sharings(SA);
                    for (subs1.Start(); subs1.More() && SR is null; subs1.Next())
                    {
                        StepRepr_PropertyDefinition PropD1 = subs1.Value() as StepRepr_PropertyDefinition;
                        if (PropD1 is null)
                        {
                            continue;
                        }
                        Interface_EntityIterator subs2 = Props.Graph().Sharings(PropD1);
                        for (subs2.Start(); subs2.More() && SR is null; subs2.Next())
                        {
                            StepShape_ShapeDefinitionRepresentation SDR = subs2.Value() as StepShape_ShapeDefinitionRepresentation;
                            if (SDR is null)
                            {
                                continue;
                            }
                            SR = SDR.UsedRepresentation() as StepShape_ShapeRepresentation;
                        }
                    }
                    if (SR is null)
                    {
                        continue;
                    }
                    Transfer_Binder binder = null;
                    for (int ir = 1; ir <= SR.NbItems() && binder is null; ir++)
                    {
                        if (SR.ItemsValue(ir).IsKind(typeof(StepShape_SolidModel)))
                        {
                            binder = TP.Find(SR.ItemsValue(ir));
                        }
                        else if (SR.ItemsValue(ir).IsKind(typeof(StepShape_ShellBasedSurfaceModel)))
                        {
                            binder = TP.Find(SR.ItemsValue(ir));
                        }
                        else if (SR.ItemsValue(ir).IsKind(typeof(StepShape_GeometricSet)))
                        {
                            binder = TP.Find(SR.ItemsValue(ir));
                        }
                    }
                    if (binder is null || !binder.HasResult())
                    {
                        continue;
                    }
                    TopoDS_Shape S = TransferBRep.ShapeResult(TP, binder);
                    if (S.IsNull())
                    {
                        continue;
                    }
                    if (myMap.IsBound(S))
                    {
                        L = myMap.Find(S);
                    }
                    if (L.IsNull())
                    {
                        Doc.Search(S, out L, true, true, true);
                    }
                }
            }

            if (L.IsNull())
            {
                continue;
            }

            // decode validation properties
            StepRepr_Representation rep = PDR.UsedRepresentation();
            for (int j = 1; j <= rep.NbItems(); j++)
            {
                StepRepr_RepresentationItem ent = rep.ItemsValue(j);
                if (Props.GetPropReal(ent, out double val, out bool isArea, theLocalFactors))
                {
                    if (isArea)
                    {
                        Doc.SetArea(L, val);
                    }
                    else
                    {
                        Doc.SetVolume(L, val);
                    }
                }
                else if (Props.GetPropPnt(ent, rep.ContextOfItems(), out gp_Pnt pos, theLocalFactors))
                {
                    Doc.SetCentroid(L, pos);
                }
            }
        }
        return true;
    }

    /// <summary>
    /// Reads layers of parts defined in the STEP model and set reference between shape and layers in the DECAF
    /// document
    /// </summary>
    protected bool ReadLayers(XSControl_WorkSession WS, XdeDoc_Document Doc)
    {
        Interface_InterfaceModel Model = WS.Model();
        XSControl_TransferReader TR = WS.TransferReader();
        Transfer_TransientProcess TP = TR.TransientProcess();

        Type tSVPLA = typeof(StepVisual_PresentationLayerAssignment);
        int nb = Model.NbEntities();
        Interface_Graph aGraph = null;

        for (int i = 1; i <= nb; i++)
        {
            Standard_Transient enti = Model.Value(i);
            if (!enti.IsKind(tSVPLA))
            {
                continue;
            }
            StepVisual_PresentationLayerAssignment SVPLA = enti as StepVisual_PresentationLayerAssignment;
            if (SVPLA.AssignedItems() is null)
            {
                continue;
            }

            TCollection_HAsciiString hName = SVPLA.Name();
            string aLayerName = hName.ToCString();
            XdeDoc_Label aLayerLabel = default;

            // check invisibility
            bool isVisible = true;
            aGraph ??= WS.Graph();
            Interface_EntityIterator subs = aGraph.Sharings(SVPLA);
            for (subs.Start(); subs.More() && isVisible; subs.Next())
            {
                if (!subs.Value().IsKind(typeof(StepVisual_Invisibility)))
                {
                    continue;
                }
                isVisible = false;
            }

            // find a target shape and its label in the document
            for (int j = 1; j <= SVPLA.NbAssignedItems(); j++)
            {
                StepVisual_LayeredItem LI = SVPLA.AssignedItemsValue(j);
                Transfer_Binder binder = LI is null ? null : TP.Find(LI.Value());
                if (binder is null || !binder.HasResult())
                {
                    continue;
                }

                TopoDS_Shape S = TransferBRep.ShapeResult(TP, binder);
                if (S.IsNull())
                {
                    continue;
                }

                if (!Doc.Search(S, out XdeDoc_Label shL, true, true, true))
                {
                    continue;
                }
                if (aLayerLabel.IsNull())
                {
                    aLayerLabel = Doc.AddLayer(aLayerName, isVisible);
                }
                Doc.SetLayer(shL, aLayerLabel);
            }

            if (!aLayerLabel.IsNull())
            {
                Doc.SetVisibility(aLayerLabel, isVisible);
            }
        }
        return true;
    }

    /// <summary>Reads SHUO for instances defined in the STEP model</summary>
    /// <remarks>OCCT.NET: not read (the document has no SHUOs, decision 13504).</remarks>
    protected bool ReadSHUOs(XSControl_WorkSession WS, XdeDoc_Document Doc,
                             NCollection_DataMap<StepBasic_ProductDefinition, STEPCAFControl_ExternFile> PDFileMap)
    {
        _ = WS;
        _ = Doc;
        _ = PDFileMap;
        return false;
    }

    /// <summary>Reads D&amp;GT for instances defined in the STEP model</summary>
    /// <remarks>OCCT.NET: not read (decision 13504).</remarks>
    protected bool ReadGDTs(XSControl_WorkSession WS, XdeDoc_Document Doc, in StepData_Factors theLocalFactors)
    {
        _ = WS;
        _ = Doc;
        return false;
    }

    /// <summary>Reads materials for instances defined in the STEP model</summary>
    /// <remarks>OCCT.NET: not read (decision 13504).</remarks>
    protected bool ReadMaterials(XSControl_WorkSession WS, XdeDoc_Document doc, NCollection_HSequence<Standard_Transient> SeqPDS,
                                 in StepData_Factors theLocalFactors)
    {
        _ = WS;
        _ = doc;
        _ = SeqPDS;
        return false;
    }

    /// <summary>Reads Views for instances defined in the STEP model</summary>
    /// <remarks>OCCT.NET: not read (decision 13504).</remarks>
    protected bool ReadViews(XSControl_WorkSession theWS, XdeDoc_Document theDoc, in StepData_Factors theLocalFactors)
    {
        _ = theWS;
        _ = theDoc;
        return false;
    }

    /// <summary>Reads metadata assigned to shapes in the STEP model</summary>
    /// <remarks>OCCT.NET: not read (decision 13504).</remarks>
    protected bool ReadMetadata(XSControl_WorkSession theWS, XdeDoc_Document theDoc, in StepData_Factors theLocalFactors)
    {
        _ = theWS;
        _ = theDoc;
        return false;
    }

    /// <summary>Reads product metadata assigned to shapes in the STEP model</summary>
    /// <remarks>OCCT.NET: not read (decision 13504).</remarks>
    protected bool ReadProductMetadata(XSControl_WorkSession theWS, XdeDoc_Document theDoc)
    {
        _ = theWS;
        _ = theDoc;
        return false;
    }

    /// <summary>
    /// Populates the sub-Label of the passed TDF Label with shape data associated with the given STEP Representation
    /// Item, including naming and topological information.
    /// </summary>
    public XdeDoc_Label SettleShapeData(StepRepr_RepresentationItem theItem, in XdeDoc_Label theLab,
                                        XdeDoc_Document theShapeTool, Transfer_TransientProcess TP)
    {
        XdeDoc_Label aResult = theLab;
        if (theItem is null)
        {
            return aResult;
        }

        TCollection_HAsciiString hName = theItem.Name();
        if (hName is null || hName.IsEmpty())
        {
            return aResult;
        }

        Transfer_Binder aBinder = TP.Find(theItem);
        if (aBinder is null)
        {
            return aResult;
        }

        TopoDS_Shape aShape = TransferBRep.ShapeResult(aBinder);
        if (aShape.IsNull())
        {
            return aResult;
        }

        // Allocate sub-Label
        aResult = theShapeTool.AddSubShape(theLab, aShape);
        if (aResult.IsNull())
        {
            return aResult;
        }

        string aName = hName.ToCString();
        theShapeTool.SetName(aResult, aName);
        theShapeTool.SetShape(aResult, aShape);

        return aResult;
    }

    //=======================================================================
    // function : collectRepresentationItems
    // purpose  : recursive collection of representation items for given representation
    //           with all representations, related to it.
    //=======================================================================
    private static void collectRepresentationItems(Interface_Graph theGraph, StepShape_ShapeRepresentation theRepresentation,
                                                   NCollection_Sequence<StepRepr_RepresentationItem> theItems, int theDepth = 0)
    {
        NCollection_HArray1<StepRepr_RepresentationItem> anItems = theRepresentation.Items();
        if (anItems is not null)
        {
            for (int i = anItems.Lower(); i <= anItems.Upper(); i++)
            {
                StepRepr_RepresentationItem anReprItem = anItems.Value(i);
                if (anReprItem is null)
                {
                    continue;
                }
                theItems.Append(anReprItem);
            }
        }

        // OCCT.NET: a cycle of representation relationships recurses without end in OCCT; stopped (decision 13505)
        if (theDepth > 1000)
        {
            return;
        }
        Interface_EntityIterator entIt = theGraph.TypedSharings(theRepresentation, typeof(StepRepr_RepresentationRelationship));
        for (entIt.Start(); entIt.More(); entIt.Next())
        {
            StepRepr_RepresentationRelationship aRelationship = entIt.Value() as StepRepr_RepresentationRelationship;
            if (aRelationship.Rep1() == theRepresentation)
            {
                StepShape_ShapeRepresentation aRepr = aRelationship.Rep2() as StepShape_ShapeRepresentation;
                if (aRepr is not null)
                {
                    collectRepresentationItems(theGraph, aRepr, theItems, theDepth + 1);
                }
            }
        }
    }

    /// <summary>
    /// Expands the topological structure of Manifold Solid BRep STEP entity to OCAF sub-tree. Entities having no
    /// names associated via their StepRepr_RepresentationItem base class are skipped.
    /// </summary>
    protected void ExpandSubShapes(XdeDoc_Document ShapeTool, NCollection_DataMap<TopoDS_Shape, StepBasic_ProductDefinition> ShapePDMap)
    {
        Transfer_TransientProcess TP = Reader().WS().TransferReader().TransientProcess();
        StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;
        if (!aStepModel.InternalParameters.ReadSubshapeNames)
        {
            return;
        }

        Interface_Graph Graph = Reader().WS().Graph();

        foreach (KeyValuePair<TopoDS_Shape, StepBasic_ProductDefinition> it in ShapePDMap)
        {
            TopoDS_Shape aRootShape = it.Key;
            StepBasic_ProductDefinition aPDef = it.Value;
            if (aPDef is null)
            {
                continue;
            }

            // Find SDR by Product
            StepShape_ShapeDefinitionRepresentation aSDR = null;
            Interface_EntityIterator entIt = Graph.TypedSharings(aPDef, typeof(StepShape_ShapeDefinitionRepresentation));
            for (entIt.Start(); entIt.More(); entIt.Next())
            {
                Standard_Transient aReferer = entIt.Value();
                aSDR = aReferer as StepShape_ShapeDefinitionRepresentation;
                if (aSDR is not null)
                {
                    break;
                }
            }

            if (aSDR is null)
            {
                continue;
            }

            // Access shape representation
            StepShape_ShapeRepresentation aShapeRepr = aSDR.UsedRepresentation() as StepShape_ShapeRepresentation;

            if (aShapeRepr is null)
            {
                continue;
            }

            // Access representation items
            NCollection_Sequence<StepRepr_RepresentationItem> aReprItems = new NCollection_Sequence<StepRepr_RepresentationItem>();
            collectRepresentationItems(Graph, aShapeRepr, aReprItems);

            if (aReprItems.Length() == 0)
            {
                continue;
            }

            if (!myMap.IsBound(aRootShape))
            {
                continue;
            }

            XdeDoc_Label aRootLab = myMap.Find(aRootShape);
            // Do not add subshapes to assembly,
            // they will be processed with corresponding Shape_Product_Definition of necessary part.
            if (ShapeTool.IsAssembly(aRootLab))
            {
                continue;
            }

            NCollection_Sequence<StepRepr_RepresentationItem> aMSBSeq = new NCollection_Sequence<StepRepr_RepresentationItem>();
            NCollection_Sequence<StepRepr_RepresentationItem> aSBSMSeq = new NCollection_Sequence<StepRepr_RepresentationItem>();

            // Iterate over the top level representation items collecting the
            // topological containers to expand
            for (int i = 1; i <= aReprItems.Length(); ++i)
            {
                StepRepr_RepresentationItem aTRepr = aReprItems.Value(i);
                if (aTRepr.IsKind(typeof(StepShape_ManifoldSolidBrep)))
                {
                    aMSBSeq.Append(aTRepr);
                }
                else if (aTRepr.IsKind(typeof(StepShape_ShellBasedSurfaceModel)))
                {
                    aSBSMSeq.Append(aTRepr);
                }
            }

            // Insert intermediate OCAF Labels for SOLIDs in case there are more
            // than one Manifold Solid BRep in the Shape Representation
            bool doInsertSolidLab = aMSBSeq.Length() > 1;

            // Expand Manifold Solid BReps
            for (int i = 1; i <= aMSBSeq.Length(); ++i)
            {
                // Put additional Label for SOLID
                if (doInsertSolidLab)
                {
                    SettleShapeData(aMSBSeq.Value(i), aRootLab, ShapeTool, TP);
                }

                ExpandManifoldSolidBrep(aRootLab, aMSBSeq.Value(i), TP, ShapeTool);
            }

            // Expand Shell-Based Surface Models
            for (int i = 1; i <= aSBSMSeq.Length(); ++i)
            {
                ExpandSBSM(aRootLab, aSBSMSeq.Value(i), TP, ShapeTool);
            }
        }
    }

    /// <summary>
    /// Expands the topological structure of Manifold Solid BRep STEP entity to OCAF sub-tree. Entities having no
    /// names associated via their StepRepr_RepresentationItem base class are skipped.
    /// </summary>
    protected void ExpandManifoldSolidBrep(XdeDoc_Label ShapeLab, StepRepr_RepresentationItem Repr,
                                           Transfer_TransientProcess TP, XdeDoc_Document ShapeTool)
    {
        // Access outer shell
        StepShape_ManifoldSolidBrep aMSB = Repr as StepShape_ManifoldSolidBrep;
        StepShape_ConnectedFaceSet aShell = aMSB.Outer();

        // Expand shell contents to CAF tree
        ExpandShell(aShell, ShapeLab, TP, ShapeTool);
    }

    /// <summary>
    /// Expands the topological structure of Shell-Based Surface Model STEP entity to OCAF sub-tree. Entities having
    /// no names associated via their StepRepr_RepresentationItem base class are skipped.
    /// </summary>
    protected void ExpandSBSM(XdeDoc_Label ShapeLab, StepRepr_RepresentationItem Repr, Transfer_TransientProcess TP,
                              XdeDoc_Document ShapeTool)
    {
        StepShape_ShellBasedSurfaceModel aSBSM = Repr as StepShape_ShellBasedSurfaceModel;

        // Access boundary shells
        NCollection_HArray1<StepShape_Shell> aShells = aSBSM.SbsmBoundary();
        for (int s = aShells.Lower(); s <= aShells.Upper(); ++s)
        {
            StepShape_Shell aShell = aShells.Value(s);
            StepShape_ConnectedFaceSet aCFS;
            StepShape_OpenShell anOpenShell = aShell?.OpenShell();
            StepShape_ClosedShell aClosedShell = aShell?.ClosedShell();

            if (anOpenShell is not null)
            {
                aCFS = anOpenShell;
            }
            else
            {
                aCFS = aClosedShell;
            }

            ExpandShell(aCFS, ShapeLab, TP, ShapeTool);
        }
    }

    /// <summary>
    /// Expands STEP Shell structure to OCAF sub-tree. Entities having no names associated via their
    /// StepRepr_RepresentationItem base class are skipped.
    /// </summary>
    protected void ExpandShell(StepShape_ConnectedFaceSet Shell, XdeDoc_Label RootLab, Transfer_TransientProcess TP,
                               XdeDoc_Document ShapeTool)
    {
        // Record CAF data
        SettleShapeData(Shell, RootLab, ShapeTool, TP);

        // Access faces
        NCollection_HArray1<StepShape_Face> aFaces = Shell.CfsFaces();
        for (int f = aFaces.Lower(); f <= aFaces.Upper(); ++f)
        {
            StepShape_Face aFace = aFaces.Value(f);
            if (aFace is null)
            {
                continue;
            }

            // Record CAF data
            SettleShapeData(aFace, RootLab, ShapeTool, TP);

            // Access face bounds
            NCollection_HArray1<StepShape_FaceBound> aWires = aFace.Bounds();
            if (aWires is null)
            {
                continue;
            }
            for (int w = aWires.Lower(); w <= aWires.Upper(); ++w)
            {
                StepShape_Loop aWire = aWires.Value(w).Bound();

                // Record CAF data
                SettleShapeData(aWire, RootLab, ShapeTool, TP);

                // Access wire edges
                // Currently only EDGE LOOPs are considered (!)
                if (!aWire.IsInstance(typeof(StepShape_EdgeLoop)))
                {
                    continue;
                }

                // Access edges
                StepShape_EdgeLoop anEdgeLoop = aWire as StepShape_EdgeLoop;
                NCollection_HArray1<StepShape_OrientedEdge> anEdges = anEdgeLoop.EdgeList();
                for (int e = anEdges.Lower(); e <= anEdges.Upper(); ++e)
                {
                    StepShape_OrientedEdge anOrientedEdge = anEdges.Value(e);
                    if (anOrientedEdge is null)
                    {
                        continue;
                    }
                    StepShape_Edge anEdge = anOrientedEdge.EdgeElement();
                    if (anEdge is null)
                    {
                        continue;
                    }

                    // Record CAF data
                    SettleShapeData(anEdge, RootLab, ShapeTool, TP);

                    // Access vertices
                    StepShape_Vertex aV1 = anEdge.EdgeStart();
                    StepShape_Vertex aV2 = anEdge.EdgeEnd();

                    // Record CAF data
                    SettleShapeData(aV1, RootLab, ShapeTool, TP);
                    SettleShapeData(aV2, RootLab, ShapeTool, TP);
                }
            }
        }
    }

    /// <summary>Set ColorMode for indicate read Colors or not.</summary>
    public void SetColorMode(bool colormode) => myColorMode = colormode;

    public bool GetColorMode() => myColorMode;

    /// <summary>Set NameMode for indicate read Name or not.</summary>
    public void SetNameMode(bool namemode) => myNameMode = namemode;

    public bool GetNameMode() => myNameMode;

    /// <summary>Set LayerMode for indicate read Layers or not.</summary>
    public void SetLayerMode(bool layermode) => myLayerMode = layermode;

    public bool GetLayerMode() => myLayerMode;

    /// <summary>PropsMode for indicate read Validation properties or not.</summary>
    public void SetPropsMode(bool propsmode) => myPropsMode = propsmode;

    public bool GetPropsMode() => myPropsMode;

    /// <summary>MetaMode for indication to read Metadata for Part/Product or not.</summary>
    public void SetMetaMode(bool theMetaMode) => myMetaMode = theMetaMode;

    public bool GetMetaMode() => myMetaMode;

    /// <summary>MetaMode for indication to read metadata from product of a part.</summary>
    public void SetProductMetaMode(bool theProductMetaMode) => myProductMetaMode = theProductMetaMode;

    public bool GetProductMetaMode() => myProductMetaMode;

    /// <summary>Set SHUO mode for indicate write SHUO or not.</summary>
    public void SetSHUOMode(bool shuomode) => mySHUOMode = shuomode;

    public bool GetSHUOMode() => mySHUOMode;

    /// <summary>Set GDT mode for indicate write GDT or not.</summary>
    public void SetGDTMode(bool gdtmode) => myGDTMode = gdtmode;

    public bool GetGDTMode() => myGDTMode;

    /// <summary>Set Material mode</summary>
    public void SetMatMode(bool matmode) => myMatMode = matmode;

    public bool GetMatMode() => myMatMode;

    /// <summary>Set View mode</summary>
    public void SetViewMode(bool viewmode) => myViewMode = viewmode;

    /// <summary>Get View mode</summary>
    public bool GetViewMode() => myViewMode;

    /// <summary>Sets parameters for shape processing.</summary>
    public void SetShapeFixParameters(NCollection_DataMap<string, string> theParameters)
    {
        myReader.SetShapeFixParameters(theParameters);
    }

    /// <summary>Sets parameters for shape processing.</summary>
    public void SetShapeFixParameters(in DE_ShapeFixParameters theParameters, NCollection_DataMap<string, string> theAdditionalParameters)
    {
        myReader.SetShapeFixParameters(theParameters, theAdditionalParameters);
    }

    /// <summary>Returns parameters for shape processing that was set by SetParameters() method.</summary>
    public NCollection_DataMap<string, string> GetShapeFixParameters()
    {
        return myReader.GetShapeFixParameters();
    }

    /// <summary>Sets flags defining operations to be performed on shapes.</summary>
    public void SetShapeProcessFlags(ShapeProcess.OperationsFlags theFlags)
    {
        myReader.SetShapeProcessFlags(theFlags);
    }

    /// <summary>Returns flags defining operations to be performed on shapes.</summary>
    public (ShapeProcess.OperationsFlags, bool) GetShapeProcessFlags()
    {
        return myReader.GetShapeProcessFlags();
    }

    private readonly STEPControl_Reader myReader;
    private readonly NCollection_DataMap<string, STEPCAFControl_ExternFile> myFiles = new NCollection_DataMap<string, STEPCAFControl_ExternFile>();
    private readonly NCollection_DataMap<TopoDS_Shape, XdeDoc_Label> myMap =
        new NCollection_DataMap<TopoDS_Shape, XdeDoc_Label>(TopTools_ShapeMapHasher.Instance);
    private bool myColorMode;
    private bool myNameMode;
    private bool myLayerMode;
    private bool myPropsMode;
    private bool myMetaMode;
    private bool myProductMetaMode;
    private bool mySHUOMode;
    private bool myGDTMode;
    private bool myMatMode;
    private bool myViewMode;
}
