// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_Writer.hxx
// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_Writer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.IFSelect_ReturnStatus;
using static Occt.Quantity_NameOfColor;
using static Occt.STEPControl_StepModelType;
using static Occt.TopAbs_ShapeEnum;
using static Occt.XdeDoc_ColorType;

namespace Occt;

/// <summary>
/// Provides a tool to write DECAF document to the STEP file. Besides transfer of shapes (including assemblies)
/// provided by STEPControl, supports also colors and part names
/// </summary>
/// <remarks>
/// <para>
/// Also supports multifile writing
/// </para>
/// <para>
/// OCCT.NET: the document is an <see cref="XdeDoc_Document"/> (decisions 1166, 1231); the writer reads its labels as
/// OCCT reads XCAF's, in OCCT's order, so the STEP text is native's. Shapes, assemblies, external references (the
/// multifile mode), names, colours, layers, validation properties, the length unit and, if
/// <c>write.stepcaf.subshapes.name</c> asks, the sub-shapes' names are written. SHUOs, GD&amp;T, materials, visual
/// materials and metadata are not (decision 1279): the document has none, and OCCT writes nothing for a document
/// without them. The writer never changes the document. The session gets a STEPCAFControl_Controller of its own
/// (decision 1264); the header's FILE_NAME is made as STEPControl_Writer makes it (decision 1267).
/// </para>
/// </remarks>
public class STEPCAFControl_Writer
{
    // XCAFPrs_Style as MakeSTEPStyles uses it: the colours and the visibility (OCCT.NET, decision 1279: no materials)
    private sealed class Style
    {
        public Quantity_ColorRGBA? ColorSurf;
        public Quantity_Color? ColorCurv;
        public bool IsVisible = true;

        public Style Copy() => (Style)MemberwiseClone();
    }

    //=======================================================================
    // function : GetLabelName
    // purpose  : auxiliary function: take name of label and append it to str
    //=======================================================================
    private static bool GetLabelName(in XdeDoc_Label theL, TCollection_HAsciiString theStr)
    {
        if (theL.IsNull() || !theL.Document.GetName(theL, out string aName))
        {
            return false;
        }
        if (string.IsNullOrEmpty(aName))
        {
            return false;
        }

        // set name, removing spaces around it
        // OCCT: TCollection_AsciiString's LeftAdjust and RightAdjust (IsSpace: ' ', '\t', '\n', '\v', '\f', '\r')
        string aBuffer = aName.Trim(' ', '\t', '\n', '\v', '\f', '\r');
        theStr.AssignCat(aBuffer);
        return true;
    }

    /// <summary>Creates a writer with an empty STEP model and sets ColorMode, LayerMode, NameMode and PropsMode to true.</summary>
    public STEPCAFControl_Writer()
    {
        STEPCAFControl_Controller.Init();
        XSControl_WorkSession aWS = new XSControl_WorkSession();
        Init(aWS);
    }

    /// <summary>
    /// Creates a reader tool and attaches it to an already existing Session Clears the session if it was not yet set
    /// for STEP Clears the internal data structures
    /// </summary>
    public STEPCAFControl_Writer(XSControl_WorkSession theWS, bool theScratch = true)
    {
        STEPCAFControl_Controller.Init();
        Init(theWS, theScratch);
    }

    /// <summary>Clears the internal data structures and attaches to a new session Clears the session if it was not yet set for STEP</summary>
    public void Init(XSControl_WorkSession theWS, bool theScratch = true)
    {
        // OCCT: theWS->SelectNorm("STEP"), the STEPCAFControl_Controller recorded process-wide; here one of the
        // session's own (decision 1264)
        STEPCAFControl_Controller.SelectFor(theWS);
        if (myWriter is null)
        {
            myWriter = new STEPControl_Writer(theWS, theScratch);
        }
        else
        {
            myWriter.SetWS(theWS, theScratch);
        }
        myFiles.Clear();
        myLabEF.Clear();
        myLabels.Clear();
        myPureRefLabels.Clear();
        myRootLabels.Clear();
    }

    /// <summary>Writes all the produced models into file In case of multimodel with extern references, filename will be a name of root file, all other files have names of corresponding parts Provided for use like single-file writer</summary>
    public IFSelect_ReturnStatus Write(string theFileName)
    {
        if (myIsCleanDuplicates)
        {
            // remove duplicates: StepTidy is not ported (decision 1271)
        }

        IFSelect_ReturnStatus aStatus = myWriter.Write(theFileName);
        if (aStatus != IFSelect_RetDone)
        {
            return aStatus;
        }

        // get directory name of the main file
        string aDirPath = Path.GetDirectoryName(theFileName) ?? "";

        foreach (KeyValuePair<string, STEPCAFControl_ExternFile> anExtFileIter in myFiles)
        {
            STEPCAFControl_ExternFile anExtFile = anExtFileIter.Value;
            if (anExtFile.GetWriteStatus() != IFSelect_RetVoid)
            {
                continue;
            }

            // construct extern file name
            string aFileName = STEPConstruct_ExternRefs.AbsolutePath(aDirPath, anExtFile.GetName().ToCString());
            if (aFileName.Length <= 0)
            {
                aFileName = anExtFile.GetName().ToCString();
            }
            Message.SendTrace("Writing external file: " + aFileName + "\n");

            IFSelect_ReturnStatus anExtStatus = anExtFile.GetWS().SendAll(aFileName);
            anExtFile.SetWriteStatus(anExtStatus);
            if (anExtStatus != IFSelect_RetDone)
            {
                aStatus = anExtStatus;
            }
        }

        return aStatus;
    }

    /// <summary>Writes all the produced models into the stream. Provided for use like single-file writer</summary>
    public IFSelect_ReturnStatus WriteStream(Stream theStream)
    {
        if (!myFiles.IsEmpty())
        {
            // writing external files is unsupported via stream interface
            return IFSelect_RetError;
        }

        return myWriter.WriteStream(theStream);
    }

    /// <summary>
    /// OCCT.NET (unit 8Z): writes the main model into <paramref name="theStream"/> and, in the multifile mode, each
    /// external file into the stream <paramref name="theExternStreams"/> opens for its name (which it may refuse with
    /// null: that file's write status is then <c>IFSelect_RetError</c>, and so is the result). The writer opens no
    /// file itself: the names come from the document's labels and may hold separators or <c>..</c>, so the caller
    /// decides where each goes (a folder, an archive, memory). The streams are disposed after writing. As
    /// <see cref="Write"/>, the files already written are not written again.
    /// </summary>
    public IFSelect_ReturnStatus WriteStream(Stream theStream, STEPCAFControl_ExternFileWriter theExternStreams)
    {
        ArgumentNullException.ThrowIfNull(theExternStreams);
        IFSelect_ReturnStatus aStatus = myWriter.WriteStream(theStream);
        if (aStatus != IFSelect_RetDone)
        {
            return aStatus;
        }
        foreach (KeyValuePair<string, STEPCAFControl_ExternFile> anExtFileIter in myFiles)
        {
            STEPCAFControl_ExternFile anExtFile = anExtFileIter.Value;
            if (anExtFile.GetWriteStatus() != IFSelect_RetVoid)
            {
                continue;
            }
            string aFileName = anExtFile.GetName().ToCString();
            Stream anExtStream = theExternStreams(aFileName);
            IFSelect_ReturnStatus anExtStatus;
            if (anExtStream is null || !anExtStream.CanWrite)
            {
                anExtStream?.Dispose();
                anExtStatus = IFSelect_RetError;
            }
            else
            {
                using (anExtStream)
                {
                    anExtStatus = anExtFile.GetWS().SendAll(aFileName, false, anExtStream);
                }
            }
            anExtFile.SetWriteStatus(anExtStatus);
            if (anExtStatus != IFSelect_RetDone)
            {
                aStatus = anExtStatus;
            }
        }
        return aStatus;
    }

    private void prepareUnit(XdeDoc_Document theDoc, StepData_StepModel theModel, ref StepData_Factors theLocalFactors)
    {
        if (theDoc is not null && theDoc.GetLengthUnit(out double aUnitValue))
        {
            theModel.SetLocalLengthUnit(aUnitValue * 1000); // convert to mm
            theLocalFactors.SetCascadeUnit(aUnitValue * 1000);
        }
        else
        {
            // OCCT: XSAlgo_ShapeProcessor::PrepareForTransfer() then the process-wide unit; here the session's
            // (decision 1265)
            double aUnit = UnitsMethods.GetCasCadeLengthUnit(myWriter.WS().Statics());
            theModel.SetLocalLengthUnit(aUnit);
            theLocalFactors.SetCascadeUnit(aUnit);
        }
    }

    /// <summary>
    /// Transfers a document (or single label) to a STEP model The mode of translation of shape is AsIs If multi is
    /// not null pointer, it switches to multifile mode (with external refs), and string pointed by &lt;multi&gt;
    /// gives prefix for names of extern files (can be empty string) Returns True if translation is OK
    /// </summary>
    public bool Transfer(XdeDoc_Document theDoc, STEPControl_StepModelType theMode = STEPControl_AsIs,
                         string? theMulti = null, Message_ProgressRange theProgress = default)
    {
        StepData_StepModel aModel = myWriter.WS().Model() as StepData_StepModel;
        // OCCT: InitFromStatic(), the process-wide statics; here the session's (decision 1181)
        aModel.InternalParameters.InitFromStatic(myWriter.WS().Statics());
        return Transfer(theDoc, aModel.InternalParameters, theMode, theMulti, theProgress);
    }

    /// <summary>Transfers a document (or single label) to a STEP model with the given parameters</summary>
    public bool Transfer(XdeDoc_Document theDoc, in DESTEP_Parameters theParams, STEPControl_StepModelType theMode = STEPControl_AsIs,
                         string? theMulti = null, Message_ProgressRange theProgress = default)
    {
        myRootLabels.Clear();
        myRootLabels.Add(theDoc);
        NCollection_Sequence<XdeDoc_Label> aLabels = new NCollection_Sequence<XdeDoc_Label>();
        theDoc.GetFreeShapes(aLabels);
        StepData_StepModel aModel = myWriter.WS().Model() as StepData_StepModel;
        aModel.InternalParameters = theParams;
        return transfer(myWriter, aLabels, theMode, theMulti, false, theProgress);
    }

    /// <summary>Method to transfer part of the document specified by label</summary>
    public bool Transfer(in XdeDoc_Label theLabel, STEPControl_StepModelType theMode = STEPControl_AsIs,
                         string? theIsMulti = null, Message_ProgressRange theProgress = default)
    {
        StepData_StepModel aModel = myWriter.WS().Model() as StepData_StepModel;
        aModel.InternalParameters.InitFromStatic(myWriter.WS().Statics());
        return Transfer(theLabel, aModel.InternalParameters, theMode, theIsMulti, theProgress);
    }

    /// <summary>Method to transfer part of the document specified by label with the given parameters</summary>
    public bool Transfer(in XdeDoc_Label theLabel, in DESTEP_Parameters theParams,
                         STEPControl_StepModelType theMode = STEPControl_AsIs, string? theIsMulti = null,
                         Message_ProgressRange theProgress = default)
    {
        if (theLabel.IsNull())
        {
            return false;
        }
        NCollection_Sequence<XdeDoc_Label> aLabels = new NCollection_Sequence<XdeDoc_Label>();
        aLabels.Append(theLabel);
        myRootLabels.Clear();
        myRootLabels.Add(theLabel.Document);
        StepData_StepModel aModel = myWriter.WS().Model() as StepData_StepModel;
        aModel.InternalParameters = theParams;
        return transfer(myWriter, aLabels, theMode, theIsMulti, false, theProgress);
    }

    /// <summary>Method to writing sequence of root assemblies or part of the file specified by use by one label</summary>
    public bool Transfer(NCollection_Sequence<XdeDoc_Label> theLabels, STEPControl_StepModelType theMode = STEPControl_AsIs,
                         string? theIsMulti = null, Message_ProgressRange theProgress = default)
    {
        StepData_StepModel aModel = myWriter.WS().Model() as StepData_StepModel;
        aModel.InternalParameters.InitFromStatic(myWriter.WS().Statics());
        return Transfer(theLabels, aModel.InternalParameters, theMode, theIsMulti, theProgress);
    }

    /// <summary>Method to writing sequence of root assemblies or part of the file specified by use by one label</summary>
    public bool Transfer(NCollection_Sequence<XdeDoc_Label> theLabels, in DESTEP_Parameters theParams,
                         STEPControl_StepModelType theMode = STEPControl_AsIs, string? theIsMulti = null,
                         Message_ProgressRange theProgress = default)
    {
        myRootLabels.Clear();
        foreach (XdeDoc_Label aLabel in theLabels)
        {
            if (!aLabel.IsNull() && !myRootLabels.Contains(aLabel.Document))
            {
                myRootLabels.Add(aLabel.Document);
            }
        }
        StepData_StepModel aModel = myWriter.WS().Model() as StepData_StepModel;
        aModel.InternalParameters = theParams;
        return transfer(myWriter, theLabels, theMode, theIsMulti, false, theProgress);
    }

    public bool Perform(XdeDoc_Document theDoc, string theFileName, Message_ProgressRange theProgress = default)
    {
        if (!Transfer(theDoc, STEPControl_AsIs, null, theProgress))
        {
            return false;
        }
        return Write(theFileName) == IFSelect_RetDone;
    }

    public bool Perform(XdeDoc_Document theDoc, string theFileName, in DESTEP_Parameters theParams,
                        Message_ProgressRange theProgress = default)
    {
        if (!Transfer(theDoc, theParams, STEPControl_AsIs, null, theProgress))
        {
            return false;
        }
        return Write(theFileName) == IFSelect_RetDone;
    }

    /// <summary>Returns data on external files Returns Null handle if no external files are read</summary>
    public NCollection_DataMap<string, STEPCAFControl_ExternFile> ExternFiles() => myFiles;

    /// <summary>Returns data on external file by its original label Returns False if no external file with given name is read</summary>
    public bool ExternFile(in XdeDoc_Label theLabel, out STEPCAFControl_ExternFile theExtFile)
    {
        theExtFile = null;
        if (!myLabEF.IsBound(theLabel))
        {
            return false;
        }
        theExtFile = myLabEF.Find(theLabel);
        return true;
    }

    /// <summary>Returns data on external file by its name Returns False if no external file with given name is read</summary>
    public bool ExternFile(string theName, out STEPCAFControl_ExternFile theExtFile)
    {
        theExtFile = null;
        // OCCT: "!myFiles.IsEmpty() ||", which makes the method fail whenever there are files
        if (!myFiles.IsEmpty() || !myFiles.IsBound(theName))
        {
            return false;
        }
        theExtFile = myFiles.Find(theName);
        return true;
    }

    /// <summary>Returns basic reader for root file</summary>
    public STEPControl_Writer ChangeWriter() => myWriter;

    /// <summary>Returns basic reader as const</summary>
    public STEPControl_Writer Writer() => myWriter;

    /// <summary>Set ColorMode for indicate write Colors or not.</summary>
    public void SetColorMode(bool theColorMode) => myColorMode = theColorMode;

    public bool GetColorMode() => myColorMode;

    /// <summary>Set NameMode for indicate write Name or not.</summary>
    public void SetNameMode(bool theNameMode) => myNameMode = theNameMode;

    public bool GetNameMode() => myNameMode;

    /// <summary>Set LayerMode for indicate write Layers or not.</summary>
    public void SetLayerMode(bool theLayerMode) => myLayerMode = theLayerMode;

    public bool GetLayerMode() => myLayerMode;

    /// <summary>PropsMode for indicate write Validation properties or not.</summary>
    public void SetPropsMode(bool thePropsMode) => myPropsMode = thePropsMode;

    public bool GetPropsMode() => myPropsMode;

    /// <summary>Set SHUO mode for indicate write SHUO or not.</summary>
    public void SetSHUOMode(bool theSHUOMode) => mySHUOMode = theSHUOMode;

    public bool GetSHUOMode() => mySHUOMode;

    /// <summary>Set dimtolmode for indicate write D&amp;GTs or not.</summary>
    public void SetDimTolMode(bool theDimTolMode) => myGDTMode = theDimTolMode;

    public bool GetDimTolMode() => myGDTMode;

    /// <summary>Set dimtolmode for indicate write D&amp;GTs or not.</summary>
    public void SetMaterialMode(bool theMaterialMode) => myMatMode = theMaterialMode;

    public bool GetMaterialMode() => myMatMode;

    /// <summary>Set the visual materials mode</summary>
    public void SetVisualMaterialMode(bool theVisMaterialMode) => myVisMatMode = theVisMaterialMode;

    public bool GetVisualMaterialMode() => myVisMatMode;

    /// <summary>Set metadata mode</summary>
    public void SetMetadataMode(bool theMetadataMode) => myMetadataMode = theMetadataMode;

    public bool GetMetadataMode() => myMetadataMode;

    /// <summary>Sets the clean duplicates mode (StepTidy, which isn't ported: decision 1271).</summary>
    public void SetCleanDuplicates(bool theCleanDuplicates) => myIsCleanDuplicates = theCleanDuplicates;

    public bool GetCleanDuplicates() => myIsCleanDuplicates;

    /// <summary>Sets parameters for shape processing.</summary>
    public void SetShapeFixParameters(NCollection_DataMap<string, string> theParameters)
    {
        myWriter.SetShapeFixParameters(theParameters);
    }

    /// <summary>Sets parameters for shape processing.</summary>
    public void SetShapeFixParameters(in DE_ShapeFixParameters theParameters, NCollection_DataMap<string, string> theAdditionalParameters)
    {
        myWriter.SetShapeFixParameters(theParameters, theAdditionalParameters);
    }

    /// <summary>Returns parameters for shape processing that was set by SetParameters() method.</summary>
    public NCollection_DataMap<string, string> GetShapeFixParameters() => myWriter.GetShapeFixParameters();

    /// <summary>Sets flags defining operations to be performed on shapes.</summary>
    public void SetShapeProcessFlags(ShapeProcess.OperationsFlags theFlags) => myWriter.SetShapeProcessFlags(theFlags);

    /// <summary>Returns flags defining operations to be performed on shapes.</summary>
    public (ShapeProcess.OperationsFlags, bool) GetShapeProcessFlags() => myWriter.GetShapeProcessFlags();

    /// <summary>
    /// Transfers labels to a STEP model Returns True if translation is OK isExternFile setting from transferExternFiles
    /// method
    /// </summary>
    protected bool transfer(STEPControl_Writer theWriter, NCollection_Sequence<XdeDoc_Label> theLabels,
                            STEPControl_StepModelType theMode, string? theIsMulti, bool theIsExternFile,
                            Message_ProgressRange theProgress)
    {
        if (theLabels.IsEmpty())
        {
            return false;
        }

        STEPCAFControl_ActorWrite anActor = theWriter.WS().NormAdaptor().ActorWrite() as STEPCAFControl_ActorWrite;

        StepData_Factors aLocalFactors = new StepData_Factors();
        StepData_StepModel aModel = theWriter.WS().Model() as StepData_StepModel;
        prepareUnit(theLabels.First().Document, aModel, ref aLocalFactors); // set local length unit to the model
        // translate free top-level shapes of the DECAF document
        NCollection_Sequence<XdeDoc_Label> aSubLabels = new NCollection_Sequence<XdeDoc_Label>();
        using Message_ProgressScope aPS = new Message_ProgressScope(theProgress, "Labels", theLabels.Length());
        // Iterate on requested shapes
        for (int aLabelIndex = 1; aLabelIndex <= theLabels.Length() && aPS.More(); aLabelIndex++)
        {
            Message_ProgressRange aRange = aPS.Next();
            XdeDoc_Label aCurL = theLabels.Value(aLabelIndex);
            if (myLabels.IsBound(aCurL))
            {
                continue; // already processed
            }

            XdeDoc_Document aDoc = aCurL.Document;
            TopoDS_Shape aCurShape = aDoc.GetShape(aCurL);
            if (aCurShape.IsNull())
            {
                continue;
            }
            TopoDS_Shape aShapeForBind = aCurShape;
            // write shape either as a whole, or as multifile (with extern refs)
            if (theIsMulti is null)
            {
                anActor.SetStdMode(false);

                NCollection_Sequence<XdeDoc_Label> aCompLabels = new NCollection_Sequence<XdeDoc_Label>();

                // For case when only part of assembly structure should be written in the document
                // if specified label is component of the assembly then
                // in order to save location of this component in the high-level assembly
                // and save name of high-level assembly it is necessary to represent structure of high-level
                // assembly as assembly with one component specified by current label. For that compound
                // containing only specified component is binded to the label of the high-level assembly. The
                // such way full structure of high-level assembly was replaced on the assembly containing one
                // component. For case when free shape reference is (located root) also create an auxiliary
                // assembly.
                if (aDoc.IsReference(aCurL))
                {
                    TopoDS_Compound aComp = new TopoDS_Compound();
                    BRep_Builder aB = new BRep_Builder();
                    aB.MakeCompound(ref aComp);
                    aB.Add(aComp, aCurShape);
                    aCurShape = aComp;
                    myPureRefLabels.Bind(aCurL, aComp);
                    aCompLabels.Append(aCurL);
                    if (aDoc.GetReferredShape(aCurL, out XdeDoc_Label aRefL))
                    {
                        if (aDoc.IsAssembly(aRefL))
                        {
                            aDoc.GetComponents(aRefL, aCompLabels, true);
                        }
                    }
                    if (!aDoc.IsFree(aCurL))
                    {
                        aCurL = aCurL.Father();
                    }
                }
                else
                {
                    // fill sequence of (sub) shapes for which attributes should be written
                    // and set actor to handle assemblies in a proper way
                    if (aDoc.IsAssembly(aCurL))
                    {
                        aDoc.GetComponents(aCurL, aCompLabels, true);
                    }
                }

                foreach (XdeDoc_Label aCurSubShL in aCompLabels)
                {
                    if (myLabels.IsBound(aCurSubShL))
                    {
                        continue;
                    }
                    TopoDS_Shape aCurSh = aDoc.GetShape(aCurSubShL);
                    if (aCurSh.IsNull())
                    {
                        continue;
                    }
                    myLabels.Bind(aCurSubShL, aCurSh);
                    aSubLabels.Append(aCurSubShL);
                    if (!aDoc.GetReferredShape(aCurSubShL, out XdeDoc_Label aRefL))
                    {
                        continue;
                    }
                    if (!myLabels.IsBound(aRefL))
                    {
                        TopoDS_Shape aRefSh = aDoc.GetShape(aRefL);
                        myLabels.Bind(aRefL, aRefSh);
                        aSubLabels.Append(aRefL);
                        if (aDoc.IsAssembly(aRefL))
                        {
                            anActor.RegisterAssembly(aRefSh);
                        }
                    }
                }
                myLabels.Bind(aCurL, aShapeForBind);
                aSubLabels.Append(aCurL);

                if (aDoc.IsAssembly(aCurL) || aDoc.IsReference(aCurL))
                {
                    anActor.RegisterAssembly(aCurShape);
                }

                theWriter.Transfer(aCurShape, theMode, aModel.InternalParameters, false, aRange);
                anActor.SetStdMode(true); // restore default behaviour
            }
            else
            {
                // translate final solids
                using Message_ProgressScope aPS1 = new Message_ProgressScope(aRange, null, 2);
                TopoDS_Shape aSass = transferExternFiles(aCurL, theMode, aSubLabels, aLocalFactors, theIsMulti, aPS1.Next());
                if (aPS1.UserBreak())
                {
                    return false;
                }

                DESTEP_Parameters.WriteMode_Assembly assemblymode = aModel.InternalParameters.WriteAssembly;
                aModel.InternalParameters.WriteAssembly = DESTEP_Parameters.WriteMode_Assembly.WriteMode_Assembly_On;
                theWriter.Transfer(aSass, STEPControl_AsIs, aModel.InternalParameters, true, aPS1.Next());
                aModel.InternalParameters.WriteAssembly = assemblymode;
            }
        }
        if (aPS.UserBreak())
        {
            return false;
        }

        theWriter.WS().ComputeGraph(true); // Setting to clear and regenerate graph

        // write names
        if (GetNameMode())
        {
            writeNames(theWriter.WS(), aSubLabels);
        }

        if (theIsMulti is null)
        {
            // write colors
            if (GetColorMode())
            {
                writeColors(theWriter.WS(), aSubLabels);
            }

            // write layers
            if (GetLayerMode())
            {
                writeLayers(theWriter.WS(), aSubLabels);
            }

            // write SHUO entities
            if (GetSHUOMode() && !theIsExternFile)
            {
                // do not store SHUO for extern reference for the moment
                writeSHUOs(theWriter.WS(), aSubLabels);
            }

            // write G&DTs
            if (GetDimTolMode())
            {
                if (aModel.InternalParameters.WriteSchema == DESTEP_Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP242DIS)
                {
                    writeDGTsAP242(theWriter.WS(), aSubLabels, aLocalFactors);
                }
                else
                {
                    writeDGTs(theWriter.WS(), aSubLabels);
                }
            }

            // write Materials
            if (GetMaterialMode())
            {
                writeMaterials(theWriter.WS(), aSubLabels);
            }

            // register all MDGPRs in model
            // OCCT.NET: OCCT iterates a map keyed by shapes (by address); here in the order the MDGPRs were made
            // (decision 1280)
            foreach (KeyValuePair<TopoDS_Shape, Standard_Transient> anItr in myMapCompMDGPR)
            {
                aModel.AddWithRefs(anItr.Value);
            }
        }

        if (theIsMulti is not null)
        { // external refs
            writeExternRefs(theWriter.WS(), aSubLabels);
        }

        // write validation props
        if (GetPropsMode())
        {
            writeValProps(theWriter.WS(), aSubLabels, theIsMulti);
        }

        // write metadata
        if (GetMetadataMode())
        {
            writeMetadata(theWriter.WS(), aSubLabels);
        }

        // refresh graph
        theWriter.WS().ComputeGraph(true);

        // Write names for the sub-shapes
        if (aModel.InternalParameters.WriteSubshapeNames)
        {
            XSControl_TransferWriter TW = ChangeWriter().WS().TransferWriter();
            Transfer_FinderProcess aFP = TW.FinderProcess();

            // Iterate on requested sub shapes
            foreach (XdeDoc_Label aCurL in aSubLabels)
            {
                foreach (XdeDoc_Label aSubL in aCurL.Document.Descendants(aCurL))
                {
                    // Access name recorded in OCAF TDataStd_Name attribute
                    TCollection_HAsciiString aHSubName = new TCollection_HAsciiString();
                    if (!GetLabelName(aSubL, aHSubName))
                    {
                        continue;
                    }

                    // Access topological data
                    TopoDS_Shape aSubS = aSubL.Document.GetShape(aSubL);
                    if (aSubS.IsNull())
                    {
                        continue;
                    }

                    // Access the correspondent STEP Representation Item
                    StepRepr_RepresentationItem aRI = null;
                    TransferBRep_ShapeMapper aShMapper = TransferBRep.ShapeMapper(aFP, aSubS);
                    if (!aFP.FindTypedTransient(aShMapper, typeof(StepRepr_RepresentationItem), ref aRI))
                    {
                        continue;
                    }

                    // Record the name
                    aRI.SetName(aHSubName);
                }
            }
        }

        return true;
    }

    /// <summary>
    /// Parses assembly structure of label L, writes all the simple shapes each to its own file named by name of its
    /// label plus prefix Returns shape representing that assembly structure in the form of nested empty compounds
    /// (and a sequence of labels which are newly written nodes of this assembly)
    /// </summary>
    protected TopoDS_Shape transferExternFiles(in XdeDoc_Label theLabel, STEPControl_StepModelType theMode,
                                               NCollection_Sequence<XdeDoc_Label> theLabels, in StepData_Factors theLocalFactors,
                                               string thePrefix, Message_ProgressRange theProgress)
    {
        // if label already translated, just return the shape
        if (myLabels.IsBound(theLabel))
        {
            return myLabels.Find(theLabel);
        }

        XdeDoc_Document aDoc = theLabel.Document;
        TopoDS_Compound aComp = new TopoDS_Compound();
        BRep_Builder aBuilder = new BRep_Builder();
        aBuilder.MakeCompound(ref aComp);
        // if not assembly, write to separate file
        if (!aDoc.IsAssembly(theLabel) && !aDoc.IsComponent(theLabel))
        {
            theLabels.Append(theLabel);
            // prepare for transfer
            XSControl_WorkSession aNewWS = new XSControl_WorkSession(myWriter.WS().Statics());
            STEPCAFControl_Controller.SelectFor(aNewWS);
            STEPControl_Writer aStepWriter = new STEPControl_Writer(aNewWS, true);
            NCollection_Sequence<XdeDoc_Label> aLabelSeq = new NCollection_Sequence<XdeDoc_Label>();
            aLabelSeq.Append(theLabel);

            // construct the name for extern file
            TCollection_HAsciiString aBaseName = new TCollection_HAsciiString();
            if (!string.IsNullOrEmpty(thePrefix))
            {
                aBaseName.AssignCat(thePrefix);
            }
            GetLabelName(theLabel, aBaseName);
            TCollection_HAsciiString aNewName = new TCollection_HAsciiString(aBaseName);
            aNewName.AssignCat(".stp");
            if (myFiles.IsBound(aNewName.ToCString()))
            { // avoid confusions
                for (int k = 1; k < 32000; k++)
                {
                    aNewName = new TCollection_HAsciiString(aBaseName);
                    aNewName.AssignCat("_");
                    aNewName.AssignCat(k.ToString(System.Globalization.CultureInfo.InvariantCulture));
                    aNewName.AssignCat(".stp");
                    if (!myFiles.IsBound(aNewName.ToCString()))
                    {
                        break;
                    }
                }
            }

            // translate and record extern file
            STEPCAFControl_ExternFile anExtFile = new STEPCAFControl_ExternFile();
            anExtFile.SetWS(aNewWS);
            anExtFile.SetName(aNewName);
            anExtFile.SetLabel(theLabel);
            DESTEP_Parameters.WriteMode_Assembly anAssemblymode = aStepWriter.Model().InternalParameters.WriteAssembly;
            aStepWriter.Model().InternalParameters.WriteAssembly = DESTEP_Parameters.WriteMode_Assembly.WriteMode_Assembly_Off;
            string anIsMulti = null;
            anExtFile.SetTransferStatus(transfer(aStepWriter, aLabelSeq, theMode, anIsMulti, true, theProgress));
            aStepWriter.Model().InternalParameters.WriteAssembly = anAssemblymode;
            myLabEF.Bind(theLabel, anExtFile);
            myFiles.Bind(aNewName.ToCString(), anExtFile);

            // return empty compound as replacement for the shape
            myLabels.Bind(theLabel, aComp);
            return aComp;
        }
        NCollection_Sequence<XdeDoc_Label> aCompLabels = new NCollection_Sequence<XdeDoc_Label>();
        XdeDoc_Label aLabel = theLabel;
        // if specified shape is component then high-level assembly is considered
        // to get valid structure with location
        if (aDoc.IsComponent(theLabel))
        {
            aCompLabels.Append(theLabel);
            aLabel = theLabel.Father();
        }
        // else iterate on components add create structure of empty compounds
        // representing the assembly
        else if (aDoc.IsAssembly(theLabel))
        {
            aDoc.GetComponents(theLabel, aCompLabels, false);
        }

        theLabels.Append(aLabel);
        using Message_ProgressScope aPS = new Message_ProgressScope(theProgress, null, aCompLabels.Length());
        // Iterate on requested shapes
        for (int i = 1; i <= aCompLabels.Length() && aPS.More(); i++)
        {
            XdeDoc_Label aCurL = aCompLabels.Value(i);
            if (!aDoc.GetReferredShape(aCurL, out XdeDoc_Label aRefL))
            {
                continue;
            }
            TopoDS_Shape aShComp = transferExternFiles(aRefL, theMode, theLabels, theLocalFactors, thePrefix, aPS.Next());
            aShComp = aShComp.Located(aDoc.GetLocation(aCurL));
            aBuilder.Add(aComp, aShComp);
        }
        myLabels.Bind(aLabel, aComp);
        return aComp;
    }

    /// <summary>Write external references to STEP</summary>
    protected bool writeExternRefs(XSControl_WorkSession theWS, NCollection_Sequence<XdeDoc_Label> theLabels)
    {
        if (theLabels.IsEmpty())
        {
            return false;
        }

        XSControl_TransferWriter aTW = theWS.TransferWriter();
        Transfer_FinderProcess aFP = aTW.FinderProcess();
        STEPConstruct_ExternRefs anEFTool = new STEPConstruct_ExternRefs(theWS);
        StepData_StepModel aStepModel = theWS.Model() as StepData_StepModel;
        int aStepSchema = (int)aStepModel.InternalParameters.WriteSchema;
        // Iterate on requested shapes
        foreach (XdeDoc_Label aLab in theLabels)
        {
            if (aLab.Document.IsAssembly(aLab))
            {
                continue; // skip assemblies
            }

            // get extern file
            if (!ExternFile(aLab, out STEPCAFControl_ExternFile anExtFile))
            {
                continue; // should never be
            }

            // find SDR
            if (!myLabels.IsBound(aLab))
            {
                continue; // not recorded as translated, skip
            }
            TopoDS_Shape aShape = myLabels.Find(aLab);

            StepShape_ShapeDefinitionRepresentation aSDR = null;
            TransferBRep_ShapeMapper mapper = TransferBRep.ShapeMapper(aFP, aShape);
            if (!aFP.FindTypedTransient(mapper, typeof(StepShape_ShapeDefinitionRepresentation), ref aSDR))
            {
                Message.SendTrace("Warning: Cannot find SDR for " + aShape.TShape().GetType().Name + "\n");
                continue;
            }

            // add extern ref
            string aStepFormat = aStepSchema == 3 ? "STEP AP203" : "STEP AP214";
            // try to get PD from SDR
            StepRepr_RepresentedDefinition aRD = aSDR.Definition();
            StepRepr_PropertyDefinition aPropDef = aRD.PropertyDefinition();
            if (aPropDef is null)
            {
                Message.SendTrace("Warning: STEPCAFControl_Writer::writeExternRefs StepRepr_PropertyDefinition is null for "
                                  + aShape.TShape().GetType().Name + "\n");
                continue;
            }
            StepRepr_CharacterizedDefinition aCharDef = aPropDef.Definition();
            StepBasic_ProductDefinition aPD = aCharDef.ProductDefinition();
            if (aPD is null)
            {
                Message.SendTrace("Warning: STEPCAFControl_Writer::writeExternRefs StepBasic_ProductDefinition is null for "
                                  + aShape.TShape().GetType().Name + "\n");
                continue;
            }
            anEFTool.AddExternRef(anExtFile.GetName().ToCString(), aPD, aStepFormat);
        }
        anEFTool.WriteExternRefs(aStepSchema);
        return true;
    }

    private static int FindEntities(Transfer_FinderProcess theFP, TopoDS_Shape theShape, ref TopLoc_Location theLocation,
                                    NCollection_Sequence<Standard_Transient> theSeqRI)
    {
        StepRepr_RepresentationItem anItem = STEPConstruct.FindEntity(theFP, theShape, ref theLocation);

        if (anItem is not null)
        {
            theSeqRI.Append(anItem);
            return 1;
        }

        // may be S was split during shape processing
        TransferBRep_ShapeMapper aMapper = TransferBRep.ShapeMapper(theFP, theShape);
        Transfer_Binder aBinder = theFP.Find(aMapper);
        if (aBinder is null)
        {
            return 0;
        }

        Transfer_TransientListBinder aTransientListBinder = aBinder as Transfer_TransientListBinder;
        int aResCount = 0;
        if (aTransientListBinder is null)
        {
            for (TopoDS_Iterator anIter = new TopoDS_Iterator(theShape); anIter.More(); anIter.Next())
            {
                StepRepr_RepresentationItem aLocalItem = STEPConstruct.FindEntity(theFP, anIter.Value(), ref theLocation);
                if (aLocalItem is null)
                {
                    continue;
                }
                aResCount++;
                theSeqRI.Append(aLocalItem);
            }
        }
        else
        {
            int aNbTransient = aTransientListBinder.NbTransients();
            for (int anInd = 1; anInd <= aNbTransient; anInd++)
            {
                Standard_Transient anEntity = aTransientListBinder.Transient(anInd);
                anItem = anEntity as StepRepr_RepresentationItem;
                if (anItem is null)
                {
                    continue;
                }
                aResCount++;
                theSeqRI.Append(anItem);
            }
        }
        return aResCount;
    }

    private static bool getStyledItem(TopoDS_Shape theShape, XdeDoc_Document theShapeTool, STEPConstruct_Styles theStyles,
                                      ref StepVisual_StyledItem theResSelItem,
                                      NCollection_DataMap<TopoDS_Shape, Standard_Transient> theMapCompMDGPR)
    {
        XdeDoc_Label aTopShL = theShapeTool.FindShape(theShape, false);
        TopoDS_Shape aTopLevSh = aTopShL.IsNull() ? new TopoDS_Shape() : theShapeTool.GetShape(aTopShL);
        bool anIsFound = false;
        if (aTopLevSh.IsNull() || !theMapCompMDGPR.IsBound(aTopLevSh))
        {
            return false;
        }
        StepVisual_PresentationRepresentation aMDGPR = theMapCompMDGPR.Find(aTopLevSh) as StepVisual_PresentationRepresentation;
        NCollection_HArray1<StepRepr_RepresentationItem> anSelItmHArr = aMDGPR.Items();
        if (anSelItmHArr is null)
        {
            return false;
        }
        // Search for PSA of Manifold solid
        NCollection_Sequence<Standard_Transient> aNewSeqRI = new NCollection_Sequence<Standard_Transient>();
        bool isFilled = false;
        for (int anIter = anSelItmHArr.Lower(); anIter <= anSelItmHArr.Upper() && !anIsFound; anIter++)
        {
            StepVisual_StyledItem aSelItm = anSelItmHArr.Value(anIter) as StepVisual_StyledItem;

            if (aSelItm is null)
            {
                continue;
            }
            // Check that it is a styled item for manifold solid brep
            if (!isFilled)
            {
                TopLoc_Location aLoc = new TopLoc_Location();
                FindEntities(theStyles.FinderProcess(), aTopLevSh, ref aLoc, aNewSeqRI);
                isFilled = true;
            }
            if (!aNewSeqRI.IsEmpty())
            {
                StepRepr_RepresentationItem anItem = aSelItm.Item();
                bool isSameMonSolBR = false;
                foreach (Standard_Transient aIterRI in aNewSeqRI)
                {
                    if (anItem is not null && anItem == aIterRI)
                    {
                        isSameMonSolBR = true;
                        break;
                    }
                }
                if (!isSameMonSolBR)
                {
                    continue;
                }
            }
            NCollection_HArray1<StepVisual_PresentationStyleAssignment> aStyles = aSelItm.Styles();
            for (int aStyleIter = aStyles.Lower(); aStyleIter <= aStyles.Upper() && !anIsFound; aStyleIter++)
            {
                StepVisual_PresentationStyleAssignment aFatherPSA = aStyles.Value(aStyleIter);
                // check for PSA for top-level (not Presentation style by context for NAUO)
                if (aFatherPSA is null || aFatherPSA.IsKind(typeof(StepVisual_PresentationStyleByContext)))
                {
                    continue;
                }
                theResSelItem = aSelItm;
                anIsFound = true;
            }
        }
        return anIsFound;
    }

    private static bool setDefaultInstanceColor(StepVisual_StyledItem theStyleItem, StepVisual_PresentationStyleAssignment thePSA)
    {
        bool anIsFound = false;
        NCollection_HArray1<StepVisual_PresentationStyleAssignment> aStyles = theStyleItem.Styles();
        for (int aStyleIter = aStyles.Lower(); aStyleIter <= aStyles.Upper() && !anIsFound; aStyleIter++)
        {
            StepVisual_PresentationStyleAssignment aFatherPSA = aStyles.Value(aStyleIter);
            // check for PSA for top-level (not Presentation style by context for NAUO)
            if (aFatherPSA is null || aFatherPSA.IsKind(typeof(StepVisual_PresentationStyleByContext)))
            {
                return false;
            }

            // get style select from father PSA
            if (aFatherPSA.NbStyles() > 0)
            {
                NCollection_HArray1<StepVisual_PresentationStyleSelect> aFatherStyles =
                    new NCollection_HArray1<StepVisual_PresentationStyleSelect>(1, aFatherPSA.NbStyles());
                int aSettingInd = 1;
                NCollection_HArray1<StepVisual_PresentationStyleSelect> anOldStyles = aFatherPSA.Styles();
                for (int aFatherStyleIter = anOldStyles.Lower(); aFatherStyleIter <= anOldStyles.Upper(); aFatherStyleIter++)
                {
                    StepVisual_PresentationStyleSelect aPSS = new StepVisual_PresentationStyleSelect();
                    StepVisual_PresentationStyleSelect anOlDPSS = anOldStyles.Value(aFatherStyleIter);
                    if (anOlDPSS.PointStyle() is not null)
                    {
                        aPSS.SetValue(anOlDPSS.PointStyle());
                    }
                    else if (anOlDPSS.CurveStyle() is not null)
                    {
                        aPSS.SetValue(anOlDPSS.CurveStyle());
                    }
                    else if (anOlDPSS.SurfaceStyleUsage() is not null)
                    {
                        aPSS.SetValue(anOlDPSS.SurfaceStyleUsage());
                    }
                    else
                    {
                        anIsFound = false;
                        break;
                    }
                    // OCCT stores a copy of the old select
                    aFatherStyles.SetValue(aSettingInd++, new StepVisual_PresentationStyleSelect(anOlDPSS));
                    anIsFound = true;
                }
                // init PSA of NAUO
                if (anIsFound)
                {
                    thePSA.Init(aFatherStyles);
                }
            }
        }
        return anIsFound;
    }

    private static void MakeSTEPStyles(STEPConstruct_Styles theStyles, TopoDS_Shape theShape,
                                       NCollection_IndexedDataMap<TopoDS_Shape, Style> theSettings,
                                       ref StepVisual_StyledItem theOverride, NCollection_Map<TopoDS_Shape> theMap,
                                       NCollection_DataMap<TopoDS_Shape, Standard_Transient> theMapCompMDGPR,
                                       NCollection_DataMap<string, Standard_Transient> theDPDCs,
                                       NCollection_DataMap<gp_Pnt, Standard_Transient> theColRGBs, XdeDoc_Document theShTool,
                                       Style theInherit, bool theIsComponent, bool theVisMaterialMode)
    {
        // skip already processed shapes
        if (!theMap.Add(theShape))
        {
            return;
        }

        // check if shape has its own style (r inherits from ancestor)
        Style aStyle = new Style();
        if (theInherit is not null)
        {
            aStyle = theInherit.Copy();
        }
        if (theSettings.Contains(theShape))
        {
            Style anOwnStyle = theSettings.FindFromKey(theShape);
            if (!anOwnStyle.IsVisible)
            {
                aStyle.IsVisible = false;
            }
            if (anOwnStyle.ColorCurv is not null)
            {
                aStyle.ColorCurv = anOwnStyle.ColorCurv;
            }
            if (anOwnStyle.ColorSurf is not null)
            {
                aStyle.ColorSurf = anOwnStyle.ColorSurf;
            }
            // OCCT: the visual material (none here, decision 1279)
        }

        // translate colors to STEP
        StepVisual_Colour aSurfColor = null;
        StepVisual_Colour aCurvColor = null;
        double aRenderTransp = 0.0;
        if (aStyle.ColorSurf is Quantity_ColorRGBA aSurfCol)
        {
            aRenderTransp = 1.0 - aSurfCol.Alpha();
            aSurfColor = STEPConstruct_Styles.EncodeColor(aSurfCol.GetRGB(), theDPDCs, theColRGBs);
        }
        if (aStyle.ColorCurv is Quantity_Color aCurvCol)
        {
            aCurvColor = STEPConstruct_Styles.EncodeColor(aCurvCol, theDPDCs, theColRGBs);
        }

        bool aHasOwn = aSurfColor is not null || aCurvColor is not null || !aStyle.IsVisible;

        // find target item and assign style to it
        StepVisual_StyledItem aSTEPstyle = theOverride;
        if (aHasOwn)
        {
            if (theShape.ShapeType() != TopAbs_COMPOUND || theIsComponent)
            { // skip compounds, let subshapes inherit its colors
                TopLoc_Location aLocation = new TopLoc_Location();
                NCollection_Sequence<Standard_Transient> aSeqRI = new NCollection_Sequence<Standard_Transient>();
                int aNbEntities = FindEntities(theStyles.FinderProcess(), theShape, ref aLocation, aSeqRI);
                if (aNbEntities <= 0)
                {
                    Message.SendTrace("Warning: Cannot find RI for " + theShape.TShape().GetType().Name + "\n");
                }
                // Get overridden style gka 10.06.03
                if (theIsComponent && aNbEntities > 0)
                {
                    getStyledItem(theShape, theShTool, theStyles, ref theOverride, theMapCompMDGPR);
                }

                foreach (Standard_Transient anEntIter in aSeqRI)
                {
                    StepRepr_RepresentationItem anItem = anEntIter as StepRepr_RepresentationItem;
                    StepVisual_PresentationStyleAssignment aPSA;
                    if (aStyle.IsVisible || aSurfColor is not null || aCurvColor is not null)
                    {
                        STEPConstruct_RenderingProperties aRenderProps = new STEPConstruct_RenderingProperties();
                        // OCCT: first the visual material, if theVisMaterialMode (none here)
                        _ = theVisMaterialMode;
                        if (aRenderTransp > 0.0)
                        {
                            aRenderProps.Init(aStyle.ColorSurf.Value);
                        }
                        aPSA = theStyles.MakeColorPSA(anItem, aSurfColor, aCurvColor, aRenderProps, theIsComponent);
                    }
                    else
                    {
                        // default white color
                        aSurfColor = STEPConstruct_Styles.EncodeColor(new Quantity_Color(Quantity_NOC_WHITE), theDPDCs, theColRGBs);
                        aPSA = theStyles.MakeColorPSA(anItem, aSurfColor, aCurvColor, new STEPConstruct_RenderingProperties(),
                                                      theIsComponent);
                        if (theIsComponent)
                        {
                            setDefaultInstanceColor(theOverride, aPSA);
                        }
                    } // end of component case

                    aSTEPstyle = theStyles.AddStyle(anItem, aPSA, theOverride);
                    aHasOwn = false;
                }
            }
        }

        // iterate on subshapes (except vertices :)
        if (theShape.ShapeType() == TopAbs_EDGE)
        {
            return;
        }
        if (theIsComponent)
        {
            return;
        }
        for (TopoDS_Iterator anIter = new TopoDS_Iterator(theShape); anIter.More(); anIter.Next())
        {
            MakeSTEPStyles(theStyles, anIter.Value(), theSettings, ref aSTEPstyle, theMap, theMapCompMDGPR, theDPDCs,
                           theColRGBs, theShTool, aHasOwn ? aStyle : null, false, theVisMaterialMode);
        }
    }

    /// <summary>
    /// Write colors assigned to specified labels, to STEP model
    /// </summary>
    protected bool writeColors(XSControl_WorkSession theWS, NCollection_Sequence<XdeDoc_Label> theLabels)
    {
        if (theLabels.IsEmpty())
        {
            return false;
        }

        STEPConstruct_Styles Styles = new STEPConstruct_Styles(theWS);
        NCollection_DataMap<string, Standard_Transient> DPDCs = new NCollection_DataMap<string, Standard_Transient>();
        NCollection_DataMap<gp_Pnt, Standard_Transient> ColRGBs = new NCollection_DataMap<gp_Pnt, Standard_Transient>();
        // Iterate on requested shapes
        foreach (XdeDoc_Label aLabel in theLabels)
        {
            // Iterate on shapes in the document
            XdeDoc_Document aSTool = aLabel.Document;
            // Skip assemblies: colors assigned to assemblies and their instances
            // are not supported (it is not clear how to encode that in STEP)
            if (aSTool.IsAssembly(aLabel))
            {
                Message.SendTrace("Warning: Cannot write color  for Assembly" + "\n");
                Message.SendTrace("Info: Check for colors assigned to components in assembly" + "\n");
                // PTV 22.01.2003 Write color for instances.
                NCollection_Sequence<XdeDoc_Label> compLabels = new NCollection_Sequence<XdeDoc_Label>();
                if (!aSTool.GetComponents(aLabel, compLabels))
                {
                    continue;
                }
                writeColors(theWS, compLabels);
                continue;
            }
            Styles.ClearStyles();

            // get a target shape and try to find corresponding context
            // (all the colors set under that label will be put into that context)
            if (!aSTool.GetShape(aLabel, out TopoDS_Shape aShape))
            {
                continue;
            }
            bool anIsComponent = aSTool.IsComponent(aLabel) || myPureRefLabels.IsBound(aLabel);
            TopoDS_Shape aTopSh = aShape;
            StepRepr_RepresentationContext aContext = Styles.FindContext(aShape);
            if (anIsComponent)
            {
                XdeDoc_Label aTopShL = aSTool.FindShape(aShape, false);
                if (aTopShL.IsNull())
                {
                    continue;
                }
                aTopSh = aSTool.GetShape(aTopShL);
                aContext = Styles.FindContext(aTopSh);
            }
            if (aContext is null)
            {
                continue;
            }

            // collect settings set on that label
            NCollection_IndexedDataMap<TopoDS_Shape, Style> aSettings =
                new NCollection_IndexedDataMap<TopoDS_Shape, Style>(TopTools_ShapeMapHasher.Instance);
            NCollection_Sequence<XdeDoc_Label> aSeq = new NCollection_Sequence<XdeDoc_Label>();
            aSeq.Append(aLabel);
            aSTool.GetSubShapes(aLabel, aSeq);
            bool anIsVisible = true;
            foreach (XdeDoc_Label aSeqValue in aSeq)
            {
                Style aStyle = new Style();
                if (aSeqValue == aLabel)
                {
                    // check for invisible status of object on label
                    if (!aSTool.IsVisible(aSeqValue))
                    {
                        anIsVisible = false;
                        aStyle.IsVisible = false;
                    }
                }
                if (aSTool.GetColor(aSeqValue, XdeDoc_ColorGen, out Quantity_ColorRGBA aColor))
                {
                    aStyle.ColorCurv = aColor.GetRGB();
                    aStyle.ColorSurf = aColor;
                }
                if (aSTool.GetColor(aSeqValue, XdeDoc_ColorSurf, out aColor))
                {
                    aStyle.ColorSurf = aColor;
                }
                if (aSTool.GetColor(aSeqValue, XdeDoc_ColorCurv, out aColor))
                {
                    aStyle.ColorCurv = aColor.GetRGB();
                }
                // OCCT: without a surface colour, the visual material's base colour (none here, decision 1279)
                if (aStyle.ColorCurv is null && aStyle.ColorSurf is null && anIsVisible)
                {
                    continue;
                }

                TopoDS_Shape aSub = aSTool.GetShape(aSeqValue);
                if (aSettings.Contains(aSub))
                {
                    aSettings.ChangeFromKey(aSub) = aStyle;
                }
                else
                {
                    aSettings.Add(aSub, aStyle);
                }
            }

            if (aSettings.Extent() <= 0)
            {
                continue;
            }

            // iterate on subshapes and create STEP styles
            StepVisual_StyledItem anOverride = null;
            NCollection_Map<TopoDS_Shape> aMap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

            MakeSTEPStyles(Styles, aShape, aSettings, ref anOverride, aMap, myMapCompMDGPR, DPDCs, ColRGBs, aSTool, null,
                           anIsComponent, GetVisualMaterialMode());

            XSControl_TransferWriter aTW = theWS.TransferWriter();
            Transfer_FinderProcess aFP = aTW.FinderProcess();
            StepData_StepModel aStepModel = aFP.Model() as StepData_StepModel;

            // create MDGPR and record it in model
            StepVisual_MechanicalDesignGeometricPresentationRepresentation aMDGPR;

            if (!anIsComponent)
            {
                if (myMapCompMDGPR.IsBound(aTopSh))
                {
                    Message.SendTrace("Error: Current Top-Level shape have MDGPR already " + "\n");
                }
                Styles.CreateMDGPR(aContext, out aMDGPR, aStepModel);
                if (aMDGPR is not null)
                {
                    myMapCompMDGPR.Bind(aTopSh, aMDGPR);
                }
            }
            else
            {
                // create SDR and add to model.
                TransferBRep_ShapeMapper aMapper = TransferBRep.ShapeMapper(aFP, aShape);
                StepShape_ContextDependentShapeRepresentation aCDSR = null;
                if (aFP.FindTypedTransient(aMapper, typeof(StepShape_ContextDependentShapeRepresentation), ref aCDSR))
                {
                    // create SDR for NAUO
                    StepRepr_ProductDefinitionShape aNullPDS = null; // important to be NULL
                    Styles.CreateNAUOSRD(aContext, aCDSR, aNullPDS);

                    // search for MDGPR of the component top-level shape
                    if (myMapCompMDGPR.IsBound(aTopSh))
                    {
                        aMDGPR = myMapCompMDGPR.Find(aTopSh) as StepVisual_MechanicalDesignGeometricPresentationRepresentation;
                    }
                    else
                    {
                        aMDGPR = new StepVisual_MechanicalDesignGeometricPresentationRepresentation();
                        TCollection_HAsciiString aReprName = new TCollection_HAsciiString("");
                        aMDGPR.SetName(aReprName);
                        aMDGPR.SetContextOfItems(aContext);
                        myMapCompMDGPR.Bind(aTopSh, aMDGPR);
                    }
                    NCollection_HArray1<StepRepr_RepresentationItem> anOldItems = aMDGPR.Items();
                    int oldLengthlen = 0;
                    if (anOldItems is not null)
                    {
                        oldLengthlen = anOldItems.Length();
                    }
                    int aNbIt = oldLengthlen + Styles.NbStyles();
                    if (aNbIt == 0)
                    {
                        continue;
                    }
                    NCollection_HArray1<StepRepr_RepresentationItem> aNewItems = new NCollection_HArray1<StepRepr_RepresentationItem>(1, aNbIt);
                    int anElemInd = 1;
                    for (int aStyleInd = 1; aStyleInd <= oldLengthlen; aStyleInd++)
                    {
                        aNewItems.SetValue(anElemInd++, anOldItems.Value(aStyleInd));
                    }
                    for (int aStyleInd = 1; aStyleInd <= Styles.NbStyles(); aStyleInd++)
                    {
                        aNewItems.SetValue(anElemInd++, Styles.Style(aStyleInd));
                    }

                    if (aNewItems.Length() > 0)
                    {
                        aMDGPR.SetItems(aNewItems);
                    }
                } // end of work with CDSR
            }
            if (!anIsVisible)
            {
                // create invisibility item and refer for styledItem
                StepVisual_Invisibility anInvisibility = new StepVisual_Invisibility();
                NCollection_HArray1<StepVisual_InvisibleItem> anInvisibilitySeq =
                    new NCollection_HArray1<StepVisual_InvisibleItem>(1, Styles.NbStyles());
                // put all style item into the harray
                for (int aStyleInd = 1; aStyleInd <= Styles.NbStyles(); aStyleInd++)
                {
                    StepRepr_RepresentationItem aStyledItm = Styles.Style(aStyleInd);
                    StepVisual_InvisibleItem anInvItem = new StepVisual_InvisibleItem();
                    anInvItem.SetValue(aStyledItm);
                    anInvisibilitySeq.SetValue(aStyleInd, anInvItem);
                }
                // set the invisibility of items
                anInvisibility.Init(anInvisibilitySeq);
                theWS.Model().AddWithRefs(anInvisibility);
            }
        }
        return true;
    }

    /// <summary>Write names assigned to specified labels, to STEP model</summary>
    protected bool writeNames(XSControl_WorkSession theWS, NCollection_Sequence<XdeDoc_Label> theLabels)
    {
        if (theLabels.IsEmpty())
        {
            return false;
        }

        // get working data
        XSControl_TransferWriter aTW = theWS.TransferWriter();
        Transfer_FinderProcess aFP = aTW.FinderProcess();

        // Iterate on requested shapes
        foreach (XdeDoc_Label aLabel in theLabels)
        {
            // find target STEP entity for the current shape
            if (!myLabels.IsBound(aLabel))
            {
                continue; // not recorded as translated, skip
            }
            // get name
            TCollection_HAsciiString aHName = new TCollection_HAsciiString();
            if (!GetLabelName(aLabel, aHName))
            {
                continue;
            }
            TopoDS_Shape aShape = myLabels.Find(aLabel);
            StepShape_ShapeDefinitionRepresentation aSDR = null;
            StepShape_ContextDependentShapeRepresentation aCDSR = null;
            bool isComponent = aLabel.Document.IsComponent(aLabel) || myPureRefLabels.IsBound(aLabel);
            TransferBRep_ShapeMapper aMapper = TransferBRep.ShapeMapper(aFP, aShape);
            if (isComponent && aFP.FindTypedTransient(aMapper, typeof(StepShape_ContextDependentShapeRepresentation), ref aCDSR))
            {
                StepRepr_ProductDefinitionShape aPDS = aCDSR.RepresentedProductRelation();
                StepBasic_ProductDefinitionRelationship aNAUO = aPDS.Definition().ProductDefinitionRelationship();
                if (aNAUO is not null)
                {
                    aNAUO.SetName(aHName);
                }
                if (myPureRefLabels.Find(aLabel, out TopoDS_Shape anInternalAssembly))
                {
                    TransferBRep_ShapeMapper aMapperOfInternalShape = TransferBRep.ShapeMapper(aFP, anInternalAssembly);
                    aFP.FindTypedTransient(aMapperOfInternalShape, typeof(StepShape_ShapeDefinitionRepresentation), ref aSDR);
                }
            }
            if (aSDR is not null
                || (aCDSR is null && aFP.FindTypedTransient(aMapper, typeof(StepShape_ShapeDefinitionRepresentation), ref aSDR)))
            {
                // set the name to the PRODUCT
                StepRepr_PropertyDefinition aPropD = aSDR.Definition().PropertyDefinition();
                if (aPropD is null)
                {
                    continue;
                }
                StepBasic_ProductDefinition aPD = aPropD.Definition().ProductDefinition();
                if (aPD is null)
                {
                    continue;
                }
                StepBasic_Product aProd = aPD.Formation().OfProduct();

                aProd.SetId(aHName);
                aProd.SetName(aHName);
            }
            else
            {
                Message.SendTrace("Warning: Cannot find RI for " + aShape.TShape().GetType().Name + "\n");
                continue;
            }
        }
        return true;
    }

    /// <summary>Write metadata assigned to specified labels, to STEP model</summary>
    /// <remarks>OCCT.NET: the document has no metadata (TDataStd_NamedData); nothing is written (decision 1279).</remarks>
    protected bool writeMetadata(XSControl_WorkSession theWS, NCollection_Sequence<XdeDoc_Label> theLabels)
    {
        _ = theWS;
        return !theLabels.IsEmpty();
    }

    private static bool WritePropsForLabel(XSControl_WorkSession theWS, NCollection_DataMap<XdeDoc_Label, TopoDS_Shape> theLabels,
                                           in XdeDoc_Label theLabel, string? theIsMulti)
    {
        _ = theLabels;
        if (theLabel.IsNull())
        {
            return false;
        }

        STEPConstruct_ValidationProps aProps = new STEPConstruct_ValidationProps(theWS);

        XdeDoc_Document aDoc = theLabel.Document;
        TopoDS_Shape aShape = aDoc.GetShape(theLabel);
        if (aShape.IsNull())
        {
            return false;
        }

        XdeDoc_ValidationProps aValProps = aDoc.GetValidationProps(theLabel);
        if (theIsMulti is null || aDoc.IsAssembly(theLabel))
        {
            // write area
            if (aValProps?.Area is double anArea)
            {
                aProps.AddArea(aShape, anArea);
            }
            // write volume
            if (aValProps?.Volume is double aVolume)
            {
                aProps.AddVolume(aShape, aVolume);
            }
        }
        // write centroid
        if (aValProps?.Centroid is gp_Pnt aCentroid)
        {
            aProps.AddCentroid(aShape, aCentroid);
        }

        if (aDoc.IsCompound(theLabel) || aDoc.IsAssembly(theLabel))
        {
            if (aDoc.HasChild(theLabel))
            {
                for (int aChildInd = 1; aChildInd <= aDoc.NbChildren(theLabel); aChildInd++)
                {
                    WritePropsForLabel(theWS, theLabels, aDoc.FindChild(theLabel, aChildInd), theIsMulti);
                }
            }
        }

        return true;
    }

    /// <summary>Write validation properties assigned to specified labels, to STEP model</summary>
    protected bool writeValProps(XSControl_WorkSession theWS, NCollection_Sequence<XdeDoc_Label> theLabels, string? theIsMulti)
    {
        if (theLabels.IsEmpty())
        {
            return false;
        }

        // Iterate on requested shapes
        foreach (XdeDoc_Label aLabel in theLabels)
        {
            WritePropsForLabel(theWS, myLabels, aLabel, theIsMulti);
        }

        return true;
    }

    /// <summary>Write layers assigned to specified labels, to STEP model</summary>
    protected bool writeLayers(XSControl_WorkSession theWS, NCollection_Sequence<XdeDoc_Label> theLabels)
    {
        if (theLabels.IsEmpty())
        {
            return false;
        }

        // get working data
        Interface_InterfaceModel aModel = theWS.Model();
        XSControl_TransferWriter aTW = theWS.TransferWriter();
        Transfer_FinderProcess aFP = aTW.FinderProcess();
        NCollection_Sequence<XdeDoc_Label> aGlobalLayerLS = new NCollection_Sequence<XdeDoc_Label>();
        // Iterate on requested shapes collect Tools
        foreach (XdeDoc_Document aLTool in myRootLabels)
        {
            NCollection_Sequence<XdeDoc_Label> aLayerLS = new NCollection_Sequence<XdeDoc_Label>();
            aLTool.GetLayerLabels(aLayerLS);
            foreach (XdeDoc_Label aLayer in aLayerLS)
            {
                aGlobalLayerLS.Append(aLayer);
            }
        }
        foreach (XdeDoc_Label aLayerL in aGlobalLayerLS)
        {
            XdeDoc_Document aDoc = aLayerL.Document;
            // get labels of shapes in that layer
            NCollection_Sequence<XdeDoc_Label> aShapeLabels = new NCollection_Sequence<XdeDoc_Label>();
            aDoc.GetShapesOfLayer(aLayerL, aShapeLabels);
            if (aShapeLabels.IsEmpty())
            {
                continue;
            }

            // name of layer: if not set, is considered as being empty
            TCollection_HAsciiString aHName = new TCollection_HAsciiString();
            GetLabelName(aLayerL, aHName);

            // Find target STEP entity for each shape and add to StepVisual_PresentationLayerAssignment
            // items.
            NCollection_Sequence<Standard_Transient> aSeqRI = new NCollection_Sequence<Standard_Transient>();
            foreach (XdeDoc_Label aShLabel in aShapeLabels)
            {
                if (aShLabel.IsNull())
                {
                    continue;
                }

                // there is no way to assign layer to instance in STEP
                if (aDoc.IsAssembly(aShLabel) || aDoc.IsReference(aShLabel))
                {
                    continue;
                }
                // check that the shape is one of (uub)labels written during current transfer
                bool anIsWritten = false;
                foreach (XdeDoc_Label aLabelIter in theLabels)
                {
                    if (aDoc.IsDescendant(aShLabel, aLabelIter))
                    {
                        anIsWritten = true;
                        break;
                    }
                }
                if (!anIsWritten)
                {
                    continue;
                }

                // get target STEP entity
                TopoDS_Shape anOneShape = aDoc.GetShape(aShLabel);

                TopLoc_Location aLoc = new TopLoc_Location();
                int aNb = FindEntities(aFP, anOneShape, ref aLoc, aSeqRI);
                if (aNb <= 0)
                {
                    Message.SendTrace("Warning: Cannot find RI for " + anOneShape.TShape()?.GetType().Name + "\n");
                }
            }
            if (aSeqRI.IsEmpty())
            {
                continue;
            }

            // analyze visibility
            StepVisual_PresentationLayerAssignment aStepLayerAs = new StepVisual_PresentationLayerAssignment();
            TCollection_HAsciiString aDescr;
            bool isLinv = false;
            if (!aDoc.IsVisible(aLayerL))
            {
                aDescr = new TCollection_HAsciiString("invisible");
                Message.SendTrace("\tLayer \"" + aHName.ToCString() + "\" is invisible" + "\n");
                isLinv = true;
            }
            else
            {
                aDescr = new TCollection_HAsciiString("visible");
            }

            // create layer entity
            int anSetStyleInd = 1;
            NCollection_HArray1<StepVisual_LayeredItem> aHArrayOfLItem = new NCollection_HArray1<StepVisual_LayeredItem>(1, aSeqRI.Length());
            foreach (Standard_Transient aRIIter in aSeqRI)
            {
                StepVisual_LayeredItem aLI = new StepVisual_LayeredItem();
                aLI.SetValue(aRIIter);
                aHArrayOfLItem.SetValue(anSetStyleInd++, aLI);
            }
            aStepLayerAs.Init(aHName, aDescr, aHArrayOfLItem);
            aModel.AddWithRefs(aStepLayerAs);
            // PTV 23.01.2003 add the invisibility AFTER adding layer into the model.
            // add the invisibility for the layer
            if (isLinv)
            {
                // Invisibility Item for containing invisible layers.
                NCollection_HArray1<StepVisual_InvisibleItem> aHInvsblItm = new NCollection_HArray1<StepVisual_InvisibleItem>(1, 1);
                StepVisual_InvisibleItem aInvIt = new StepVisual_InvisibleItem();
                aInvIt.SetValue(aStepLayerAs);
                aHInvsblItm.SetValue(1, aInvIt);

                StepVisual_Invisibility aInvsblt = new StepVisual_Invisibility();
                aInvsblt.Init(aHInvsblItm);
                aModel.AddWithRefs(aInvsblt);
            }
        }
        return true;
    }

    /// <summary>Write SHUO assigned to specified component, to STEP model</summary>
    /// <remarks>OCCT.NET: the document has no SHUOs; nothing is written (decision 1279).</remarks>
    protected bool writeSHUOs(XSControl_WorkSession theWS, NCollection_Sequence<XdeDoc_Label> theLabels)
    {
        _ = theWS;
        return !theLabels.IsEmpty();
    }

    /// <summary>Write D&amp;GTs assigned to specified labels, to STEP model</summary>
    /// <remarks>OCCT.NET: the document has no GD&amp;T; nothing is written (decision 1279).</remarks>
    protected bool writeDGTs(XSControl_WorkSession theWS, NCollection_Sequence<XdeDoc_Label> theLabels)
    {
        _ = theWS;
        _ = theLabels;
        return false;
    }

    /// <summary>Write D&amp;GTs assigned to specified labels, to STEP model, according AP242</summary>
    /// <remarks>OCCT.NET: the document has no GD&amp;T; nothing is written (decision 1279).</remarks>
    protected bool writeDGTsAP242(XSControl_WorkSession theWS, NCollection_Sequence<XdeDoc_Label> theLabels,
                                  in StepData_Factors theLocalFactors)
    {
        _ = theWS;
        _ = theLabels;
        return true;
    }

    /// <summary>Write materials assigned to specified labels, to STEP model</summary>
    /// <remarks>OCCT.NET: the document has no materials; nothing is written (decision 1279).</remarks>
    protected bool writeMaterials(XSControl_WorkSession theWS, NCollection_Sequence<XdeDoc_Label> theLabels)
    {
        _ = theWS;
        return !theLabels.IsEmpty();
    }

    private STEPControl_Writer myWriter;
    private readonly NCollection_DataMap<string, STEPCAFControl_ExternFile> myFiles = new NCollection_DataMap<string, STEPCAFControl_ExternFile>();
    private readonly NCollection_DataMap<XdeDoc_Label, STEPCAFControl_ExternFile> myLabEF = new NCollection_DataMap<XdeDoc_Label, STEPCAFControl_ExternFile>();
    private readonly NCollection_DataMap<XdeDoc_Label, TopoDS_Shape> myLabels = new NCollection_DataMap<XdeDoc_Label, TopoDS_Shape>();
    private readonly NCollection_DataMap<XdeDoc_Label, TopoDS_Shape> myPureRefLabels = new NCollection_DataMap<XdeDoc_Label, TopoDS_Shape>();
    // OCCT: the documents' root labels (a map); here the documents, in the order given
    private readonly List<XdeDoc_Document> myRootLabels = new List<XdeDoc_Document>();
    private readonly NCollection_DataMap<TopoDS_Shape, Standard_Transient> myMapCompMDGPR =
        new NCollection_DataMap<TopoDS_Shape, Standard_Transient>(TopTools_ShapeMapHasher.Instance);
    private bool myColorMode = true;
    private bool myNameMode = true;
    private bool myLayerMode = true;
    private bool myPropsMode = true;
    private bool myMetadataMode = true;
    private bool mySHUOMode = true;
    private bool myGDTMode = true;
    private bool myMatMode = true;
    private bool myVisMatMode;
    private bool myIsCleanDuplicates;
}
