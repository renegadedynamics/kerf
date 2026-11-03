// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_Writer.hxx
// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_Writer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Construct;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Documents;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Exchange.Framework.Selection.ReturnStatus;
using static Kerf.Engine.Foundation.NameOfColor;
using static Kerf.Engine.Exchange.Step.ModelType;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Exchange.Documents.ColorType;

namespace Kerf.Engine.Exchange.Step.Caf;

/// <summary>
/// Provides a tool to write DECAF document to the STEP file. Besides transfer of shapes (including assemblies)
/// provided by STEPControl, supports also colors and part names
/// </summary>
/// <remarks>
/// <para>
/// Also supports multifile writing
/// </para>
/// <para>
/// OCCT.NET: the document is an <see cref="ExchangeDocument"/> (decisions 1166, 1231); the writer reads its labels as
/// OCCT reads XCAF's, in OCCT's order, so the STEP text is native's. Shapes, assemblies, external references (the
/// multifile mode), names, colours, layers, validation properties, the length unit and, if
/// <c>write.stepcaf.subshapes.name</c> asks, the sub-shapes' names are written. SHUOs, GD&amp;T, materials, visual
/// materials and metadata are not (decision 1279): the document has none, and OCCT writes nothing for a document
/// without them. The writer never changes the document. The session gets a STEPCAFControl_Controller of its own
/// (decision 1264); the header's FILE_NAME is made as STEPControl_Writer makes it (decision 1267).
/// </para>
/// </remarks>
public class CafWriter
{
    // XCAFPrs_Style as MakeSTEPStyles uses it: the colours and the visibility (OCCT.NET, decision 1279: no materials)
    private sealed class Style
    {
        public ColorWithAlpha? ColorSurf;
        public Color? ColorCurv;
        public bool IsVisible = true;

        public Style Copy() => (Style)MemberwiseClone();
    }

    //=======================================================================
    // function : GetLabelName
    // purpose  : auxiliary function: take name of label and append it to str
    //=======================================================================
    private static bool GetLabelName(in Label l, TCollection_HAsciiString str)
    {
        if (l.IsNull || !l.Document.GetName(l, out string name))
        {
            return false;
        }
        if (string.IsNullOrEmpty(name))
        {
            return false;
        }

        // set name, removing spaces around it
        // OCCT: TCollection_AsciiString's LeftAdjust and RightAdjust (IsSpace: ' ', '\t', '\n', '\v', '\f', '\r')
        string buffer = name.Trim(' ', '\t', '\n', '\v', '\f', '\r');
        str.AssignCat(buffer);
        return true;
    }

    /// <summary>Creates a writer with an empty STEP model and sets ColorMode, LayerMode, NameMode and PropsMode to true.</summary>
    public CafWriter()
    {
        CafController.Init();
        ExchangeWorkSession ws = new ExchangeWorkSession();
        Init(ws);
    }

    /// <summary>
    /// Creates a reader tool and attaches it to an already existing Session Clears the session if it was not yet set
    /// for STEP Clears the internal data structures
    /// </summary>
    public CafWriter(ExchangeWorkSession ws, bool scratch = true)
    {
        CafController.Init();
        Init(ws, scratch);
    }

    /// <summary>Clears the internal data structures and attaches to a new session Clears the session if it was not yet set for STEP</summary>
    public void Init(ExchangeWorkSession ws, bool scratch = true)
    {
        // OCCT: theWS->SelectNorm("STEP"), the STEPCAFControl_Controller recorded process-wide; here one of the
        // session's own (decision 1264)
        CafController.SelectFor(ws);
        if (_writer is null)
        {
            _writer = new Writer(ws, scratch);
        }
        else
        {
            _writer.SetWS(ws, scratch);
        }
        _files.Clear();
        _labEF.Clear();
        _labels.Clear();
        _pureRefLabels.Clear();
        _rootLabels.Clear();
    }

    /// <summary>Writes all the produced models into file In case of multimodel with extern references, filename will be a name of root file, all other files have names of corresponding parts Provided for use like single-file writer</summary>
    public ReturnStatus Write(string fileName)
    {
        if (_isCleanDuplicates)
        {
            // remove duplicates: StepTidy is not ported (decision 1271)
        }

        ReturnStatus status = _writer.Write(fileName);
        if (status != ReturnStatus.RetDone)
        {
            return status;
        }

        // get directory name of the main file
        string dirPath = Path.GetDirectoryName(fileName) ?? "";

        foreach (KeyValuePair<string, ExternFile> extFileIter in _files)
        {
            ExternFile extFile = extFileIter.Value;
            if (extFile.GetWriteStatus() != ReturnStatus.RetVoid)
            {
                continue;
            }

            // construct extern file name
            string aFileName = ExternRefs.AbsolutePath(dirPath, extFile.GetName().ToCString());
            if (aFileName.Length <= 0)
            {
                aFileName = extFile.GetName().ToCString();
            }
            global::Kerf.Engine.Foundation.Message.SendTrace("Writing external file: " + aFileName + "\n");

            ReturnStatus extStatus = extFile.GetWS().SendAll(aFileName);
            extFile.SetWriteStatus(extStatus);
            if (extStatus != ReturnStatus.RetDone)
            {
                status = extStatus;
            }
        }

        return status;
    }

    /// <summary>Writes all the produced models into the stream. Provided for use like single-file writer</summary>
    public ReturnStatus WriteStream(Stream stream)
    {
        if (!_files.IsEmpty())
        {
            // writing external files is unsupported via stream interface
            return ReturnStatus.RetError;
        }

        return _writer.WriteStream(stream);
    }

    /// <summary>
    /// OCCT.NET (unit 8Z): writes the main model into <paramref name="stream"/> and, in the multifile mode, each
    /// external file into the stream <paramref name="externStreams"/> opens for its name (which it may refuse with
    /// null: that file's write status is then <c>IFSelect_RetError</c>, and so is the result). The writer opens no
    /// file itself: the names come from the document's labels and may hold separators or <c>..</c>, so the caller
    /// decides where each goes (a folder, an archive, memory). The streams are disposed after writing. As
    /// <see cref="Write"/>, the files already written are not written again.
    /// </summary>
    public ReturnStatus WriteStream(Stream stream, ExternFileWriter externStreams)
    {
        ArgumentNullException.ThrowIfNull(externStreams);
        ReturnStatus status = _writer.WriteStream(stream);
        if (status != ReturnStatus.RetDone)
        {
            return status;
        }
        foreach (KeyValuePair<string, ExternFile> extFileIter in _files)
        {
            ExternFile extFile = extFileIter.Value;
            if (extFile.GetWriteStatus() != ReturnStatus.RetVoid)
            {
                continue;
            }
            string fileName = extFile.GetName().ToCString();
            Stream extStream = externStreams(fileName);
            ReturnStatus extStatus;
            if (extStream is null || !extStream.CanWrite)
            {
                extStream?.Dispose();
                extStatus = ReturnStatus.RetError;
            }
            else
            {
                using (extStream)
                {
                    extStatus = extFile.GetWS().SendAll(fileName, false, extStream);
                }
            }
            extFile.SetWriteStatus(extStatus);
            if (extStatus != ReturnStatus.RetDone)
            {
                status = extStatus;
            }
        }
        return status;
    }

    private void PrepareUnit(ExchangeDocument doc, StepModel model, ref Factors localFactors)
    {
        if (doc is not null && doc.GetLengthUnit(out double unitValue))
        {
            model.SetLocalLengthUnit(unitValue * 1000); // convert to mm
            localFactors.SetCascadeUnit(unitValue * 1000);
        }
        else
        {
            // OCCT: XSAlgo_ShapeProcessor::PrepareForTransfer() then the process-wide unit; here the session's
            // (decision 1265)
            double unit = global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetCasCadeLengthUnit(_writer.WS().Statics());
            model.SetLocalLengthUnit(unit);
            localFactors.SetCascadeUnit(unit);
        }
    }

    /// <summary>
    /// Transfers a document (or single label) to a STEP model The mode of translation of shape is AsIs If multi is
    /// not null pointer, it switches to multifile mode (with external refs), and string pointed by &lt;multi&gt;
    /// gives prefix for names of extern files (can be empty string) Returns True if translation is OK
    /// </summary>
    public bool Transfer(ExchangeDocument doc, ModelType mode = ModelType.AsIs,
                         string? multi = null, ProgressRange progress = default)
    {
        StepModel model = _writer.WS().Model() as StepModel;
        // OCCT: InitFromStatic(), the process-wide statics; here the session's (decision 1181)
        model.InternalParameters.InitFromStatic(_writer.WS().Statics());
        return Transfer(doc, model.InternalParameters, mode, multi, progress);
    }

    /// <summary>Transfers a document (or single label) to a STEP model with the given parameters</summary>
    public bool Transfer(ExchangeDocument doc, in Parameters theParams, ModelType mode = ModelType.AsIs,
                         string? multi = null, ProgressRange progress = default)
    {
        _rootLabels.Clear();
        _rootLabels.Add(doc);
        NCollection_Sequence<Label> labels = new NCollection_Sequence<Label>();
        doc.GetFreeShapes(labels);
        StepModel model = _writer.WS().Model() as StepModel;
        model.InternalParameters = theParams;
        return TransferCore(_writer, labels, mode, multi, false, progress);
    }

    /// <summary>Method to transfer part of the document specified by label</summary>
    public bool Transfer(in Label label, ModelType mode = ModelType.AsIs,
                         string? isMulti = null, ProgressRange progress = default)
    {
        StepModel model = _writer.WS().Model() as StepModel;
        model.InternalParameters.InitFromStatic(_writer.WS().Statics());
        return Transfer(label, model.InternalParameters, mode, isMulti, progress);
    }

    /// <summary>Method to transfer part of the document specified by label with the given parameters</summary>
    public bool Transfer(in Label label, in Parameters theParams,
                         ModelType mode = ModelType.AsIs, string? isMulti = null,
                         ProgressRange progress = default)
    {
        if (label.IsNull)
        {
            return false;
        }
        NCollection_Sequence<Label> labels = new NCollection_Sequence<Label>();
        labels.Append(label);
        _rootLabels.Clear();
        _rootLabels.Add(label.Document);
        StepModel model = _writer.WS().Model() as StepModel;
        model.InternalParameters = theParams;
        return TransferCore(_writer, labels, mode, isMulti, false, progress);
    }

    /// <summary>Method to writing sequence of root assemblies or part of the file specified by use by one label</summary>
    public bool Transfer(NCollection_Sequence<Label> labels, ModelType mode = ModelType.AsIs,
                         string? isMulti = null, ProgressRange progress = default)
    {
        StepModel model = _writer.WS().Model() as StepModel;
        model.InternalParameters.InitFromStatic(_writer.WS().Statics());
        return Transfer(labels, model.InternalParameters, mode, isMulti, progress);
    }

    /// <summary>Method to writing sequence of root assemblies or part of the file specified by use by one label</summary>
    public bool Transfer(NCollection_Sequence<Label> labels, in Parameters theParams,
                         ModelType mode = ModelType.AsIs, string? isMulti = null,
                         ProgressRange progress = default)
    {
        _rootLabels.Clear();
        foreach (Label label in labels)
        {
            if (!label.IsNull && !_rootLabels.Contains(label.Document))
            {
                _rootLabels.Add(label.Document);
            }
        }
        StepModel model = _writer.WS().Model() as StepModel;
        model.InternalParameters = theParams;
        return TransferCore(_writer, labels, mode, isMulti, false, progress);
    }

    public bool Perform(ExchangeDocument doc, string fileName, ProgressRange progress = default)
    {
        if (!Transfer(doc, ModelType.AsIs, null, progress))
        {
            return false;
        }
        return Write(fileName) == ReturnStatus.RetDone;
    }

    public bool Perform(ExchangeDocument doc, string fileName, in Parameters theParams,
                        ProgressRange progress = default)
    {
        if (!Transfer(doc, theParams, ModelType.AsIs, null, progress))
        {
            return false;
        }
        return Write(fileName) == ReturnStatus.RetDone;
    }

    /// <summary>Returns data on external files Returns Null handle if no external files are read</summary>
    public NCollection_DataMap<string, ExternFile> ExternFiles() => _files;

    /// <summary>Returns data on external file by its original label Returns False if no external file with given name is read</summary>
    public bool ExternFile(in Label label, out ExternFile extFile)
    {
        extFile = null;
        if (!_labEF.IsBound(label))
        {
            return false;
        }
        extFile = _labEF.Find(label);
        return true;
    }

    /// <summary>Returns data on external file by its name Returns False if no external file with given name is read</summary>
    public bool ExternFile(string name, out ExternFile extFile)
    {
        extFile = null;
        // OCCT: "!myFiles.IsEmpty() ||", which makes the method fail whenever there are files
        if (!_files.IsEmpty() || !_files.IsBound(name))
        {
            return false;
        }
        extFile = _files.Find(name);
        return true;
    }

    /// <summary>Returns basic reader for root file</summary>
    public Writer ChangeWriter() => _writer;

    /// <summary>Returns basic reader as const</summary>
    public Writer Writer() => _writer;

    /// <summary>Set ColorMode for indicate write Colors or not.</summary>
    public void SetColorMode(bool colorMode) => _colorMode = colorMode;

    public bool GetColorMode() => _colorMode;

    /// <summary>Set NameMode for indicate write Name or not.</summary>
    public void SetNameMode(bool nameMode) => _nameMode = nameMode;

    public bool GetNameMode() => _nameMode;

    /// <summary>Set LayerMode for indicate write Layers or not.</summary>
    public void SetLayerMode(bool layerMode) => _layerMode = layerMode;

    public bool GetLayerMode() => _layerMode;

    /// <summary>PropsMode for indicate write Validation properties or not.</summary>
    public void SetPropsMode(bool propsMode) => _propsMode = propsMode;

    public bool GetPropsMode() => _propsMode;

    /// <summary>Set SHUO mode for indicate write SHUO or not.</summary>
    public void SetSHUOMode(bool shuoMode) => _shuoMode = shuoMode;

    public bool GetSHUOMode() => _shuoMode;

    /// <summary>Set dimtolmode for indicate write D&amp;GTs or not.</summary>
    public void SetDimTolMode(bool dimTolMode) => _gdtMode = dimTolMode;

    public bool GetDimTolMode() => _gdtMode;

    /// <summary>Set dimtolmode for indicate write D&amp;GTs or not.</summary>
    public void SetMaterialMode(bool materialMode) => _matMode = materialMode;

    public bool GetMaterialMode() => _matMode;

    /// <summary>Set the visual materials mode</summary>
    public void SetVisualMaterialMode(bool visMaterialMode) => _visMatMode = visMaterialMode;

    public bool GetVisualMaterialMode() => _visMatMode;

    /// <summary>Set metadata mode</summary>
    public void SetMetadataMode(bool metadataMode) => _metadataMode = metadataMode;

    public bool GetMetadataMode() => _metadataMode;

    /// <summary>Sets the clean duplicates mode (StepTidy, which isn't ported: decision 1271).</summary>
    public void SetCleanDuplicates(bool cleanDuplicates) => _isCleanDuplicates = cleanDuplicates;

    public bool GetCleanDuplicates() => _isCleanDuplicates;

    /// <summary>Sets parameters for shape processing.</summary>
    public void SetShapeFixParameters(NCollection_DataMap<string, string> parameters)
    {
        _writer.SetShapeFixParameters(parameters);
    }

    /// <summary>Sets parameters for shape processing.</summary>
    public void SetShapeFixParameters(in ShapeFixParameters parameters, NCollection_DataMap<string, string> additionalParameters)
    {
        _writer.SetShapeFixParameters(parameters, additionalParameters);
    }

    /// <summary>Returns parameters for shape processing that was set by SetParameters() method.</summary>
    public NCollection_DataMap<string, string> GetShapeFixParameters() => _writer.GetShapeFixParameters();

    /// <summary>Sets flags defining operations to be performed on shapes.</summary>
    public void SetShapeProcessFlags(global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags flags) => _writer.SetShapeProcessFlags(flags);

    /// <summary>Returns flags defining operations to be performed on shapes.</summary>
    public (global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags, bool) GetShapeProcessFlags() => _writer.GetShapeProcessFlags();

    /// <summary>
    /// Transfers labels to a STEP model Returns True if translation is OK isExternFile setting from transferExternFiles
    /// method
    /// </summary>
    protected bool TransferCore(Writer writer, NCollection_Sequence<Label> labels,
                            ModelType mode, string? isMulti, bool isExternFile,
                            ProgressRange progress)
    {
        if (labels.IsEmpty())
        {
            return false;
        }

        CafActorWrite actor = writer.WS().NormAdaptor().ActorWrite() as CafActorWrite;

        Factors localFactors = new Factors();
        StepModel model = writer.WS().Model() as StepModel;
        PrepareUnit(labels.First().Document, model, ref localFactors); // set local length unit to the model
        // translate free top-level shapes of the DECAF document
        NCollection_Sequence<Label> subLabels = new NCollection_Sequence<Label>();
        using ProgressScope ps = new ProgressScope(progress, "Labels", labels.Length());
        // Iterate on requested shapes
        for (int labelIndex = 1; labelIndex <= labels.Length() && ps.More(); labelIndex++)
        {
            ProgressRange range = ps.Next();
            Label curL = labels.Value(labelIndex);
            if (_labels.IsBound(curL))
            {
                continue; // already processed
            }

            ExchangeDocument doc = curL.Document;
            Shape curShape = doc.GetShape(curL);
            if (curShape.IsNull)
            {
                continue;
            }
            Shape shapeForBind = curShape;
            // write shape either as a whole, or as multifile (with extern refs)
            if (isMulti is null)
            {
                actor.SetStdMode(false);

                NCollection_Sequence<Label> compLabels = new NCollection_Sequence<Label>();

                // For case when only part of assembly structure should be written in the document
                // if specified label is component of the assembly then
                // in order to save location of this component in the high-level assembly
                // and save name of high-level assembly it is necessary to represent structure of high-level
                // assembly as assembly with one component specified by current label. For that compound
                // containing only specified component is binded to the label of the high-level assembly. The
                // such way full structure of high-level assembly was replaced on the assembly containing one
                // component. For case when free shape reference is (located root) also create an auxiliary
                // assembly.
                if (doc.IsReference(curL))
                {
                    Compound comp = new Compound();
                    BrepBuilder b = new BrepBuilder();
                    b.MakeCompound(ref comp);
                    b.Add(comp, curShape);
                    curShape = comp;
                    _pureRefLabels.Bind(curL, comp);
                    compLabels.Append(curL);
                    if (doc.GetReferredShape(curL, out Label refL))
                    {
                        if (doc.IsAssembly(refL))
                        {
                            doc.GetComponents(refL, compLabels, true);
                        }
                    }
                    if (!doc.IsFree(curL))
                    {
                        curL = curL.Father();
                    }
                }
                else
                {
                    // fill sequence of (sub) shapes for which attributes should be written
                    // and set actor to handle assemblies in a proper way
                    if (doc.IsAssembly(curL))
                    {
                        doc.GetComponents(curL, compLabels, true);
                    }
                }

                foreach (Label curSubShL in compLabels)
                {
                    if (_labels.IsBound(curSubShL))
                    {
                        continue;
                    }
                    Shape curSh = doc.GetShape(curSubShL);
                    if (curSh.IsNull)
                    {
                        continue;
                    }
                    _labels.Bind(curSubShL, curSh);
                    subLabels.Append(curSubShL);
                    if (!doc.GetReferredShape(curSubShL, out Label refL))
                    {
                        continue;
                    }
                    if (!_labels.IsBound(refL))
                    {
                        Shape refSh = doc.GetShape(refL);
                        _labels.Bind(refL, refSh);
                        subLabels.Append(refL);
                        if (doc.IsAssembly(refL))
                        {
                            actor.RegisterAssembly(refSh);
                        }
                    }
                }
                _labels.Bind(curL, shapeForBind);
                subLabels.Append(curL);

                if (doc.IsAssembly(curL) || doc.IsReference(curL))
                {
                    actor.RegisterAssembly(curShape);
                }

                writer.Transfer(curShape, mode, model.InternalParameters, false, range);
                actor.SetStdMode(true); // restore default behaviour
            }
            else
            {
                // translate final solids
                using ProgressScope ps1 = new ProgressScope(range, null, 2);
                Shape sass = TransferExternFiles(curL, mode, subLabels, localFactors, isMulti, ps1.Next());
                if (ps1.UserBreak())
                {
                    return false;
                }

                Parameters.WriteMode_Assembly assemblymode = model.InternalParameters.WriteAssembly;
                model.InternalParameters.WriteAssembly = Parameters.WriteMode_Assembly.WriteMode_Assembly_On;
                writer.Transfer(sass, ModelType.AsIs, model.InternalParameters, true, ps1.Next());
                model.InternalParameters.WriteAssembly = assemblymode;
            }
        }
        if (ps.UserBreak())
        {
            return false;
        }

        writer.WS().ComputeGraph(true); // Setting to clear and regenerate graph

        // write names
        if (GetNameMode())
        {
            WriteNames(writer.WS(), subLabels);
        }

        if (isMulti is null)
        {
            // write colors
            if (GetColorMode())
            {
                WriteColors(writer.WS(), subLabels);
            }

            // write layers
            if (GetLayerMode())
            {
                WriteLayers(writer.WS(), subLabels);
            }

            // write SHUO entities
            if (GetSHUOMode() && !isExternFile)
            {
                // do not store SHUO for extern reference for the moment
                WriteSHUOs(writer.WS(), subLabels);
            }

            // write G&DTs
            if (GetDimTolMode())
            {
                if (model.InternalParameters.WriteSchema == Parameters.WriteMode_StepSchema.WriteMode_StepSchema_AP242DIS)
                {
                    WriteDGTsAP242(writer.WS(), subLabels, localFactors);
                }
                else
                {
                    WriteDGTs(writer.WS(), subLabels);
                }
            }

            // write Materials
            if (GetMaterialMode())
            {
                WriteMaterials(writer.WS(), subLabels);
            }

            // register all MDGPRs in model
            // OCCT.NET: OCCT iterates a map keyed by shapes (by address); here in the order the MDGPRs were made
            // (decision 1280)
            foreach (KeyValuePair<Shape, Transient> itr in _mapCompMDGPR)
            {
                model.AddWithRefs(itr.Value);
            }
        }

        if (isMulti is not null)
        { // external refs
            WriteExternRefs(writer.WS(), subLabels);
        }

        // write validation props
        if (GetPropsMode())
        {
            WriteValProps(writer.WS(), subLabels, isMulti);
        }

        // write metadata
        if (GetMetadataMode())
        {
            WriteMetadata(writer.WS(), subLabels);
        }

        // refresh graph
        writer.WS().ComputeGraph(true);

        // Write names for the sub-shapes
        if (model.InternalParameters.WriteSubshapeNames)
        {
            TransferWriter tw = ChangeWriter().WS().TransferWriter();
            FinderProcess fp = tw.FinderProcess();

            // Iterate on requested sub shapes
            foreach (Label curL in subLabels)
            {
                foreach (Label subL in curL.Document.Descendants(curL))
                {
                    // Access name recorded in OCAF TDataStd_Name attribute
                    TCollection_HAsciiString hSubName = new TCollection_HAsciiString();
                    if (!GetLabelName(subL, hSubName))
                    {
                        continue;
                    }

                    // Access topological data
                    Shape subS = subL.Document.GetShape(subL);
                    if (subS.IsNull)
                    {
                        continue;
                    }

                    // Access the correspondent STEP Representation Item
                    RepresentationItem ri = null;
                    ShapeMapper shMapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, subS);
                    if (!fp.FindTypedTransient(shMapper, typeof(RepresentationItem), ref ri))
                    {
                        continue;
                    }

                    // Record the name
                    ri.SetName(hSubName);
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
    protected Shape TransferExternFiles(in Label label, ModelType mode,
                                               NCollection_Sequence<Label> labels, in Factors localFactors,
                                               string prefix, ProgressRange progress)
    {
        // if label already translated, just return the shape
        if (_labels.IsBound(label))
        {
            return _labels.Find(label);
        }

        ExchangeDocument doc = label.Document;
        Compound comp = new Compound();
        BrepBuilder builder = new BrepBuilder();
        builder.MakeCompound(ref comp);
        // if not assembly, write to separate file
        if (!doc.IsAssembly(label) && !doc.IsComponent(label))
        {
            labels.Append(label);
            // prepare for transfer
            ExchangeWorkSession newWS = new ExchangeWorkSession(_writer.WS().Statics());
            CafController.SelectFor(newWS);
            Writer stepWriter = new Writer(newWS, true);
            NCollection_Sequence<Label> labelSeq = new NCollection_Sequence<Label>();
            labelSeq.Append(label);

            // construct the name for extern file
            TCollection_HAsciiString baseName = new TCollection_HAsciiString();
            if (!string.IsNullOrEmpty(prefix))
            {
                baseName.AssignCat(prefix);
            }
            GetLabelName(label, baseName);
            TCollection_HAsciiString newName = new TCollection_HAsciiString(baseName);
            newName.AssignCat(".stp");
            if (_files.IsBound(newName.ToCString()))
            { // avoid confusions
                for (int k = 1; k < 32000; k++)
                {
                    newName = new TCollection_HAsciiString(baseName);
                    newName.AssignCat("_");
                    newName.AssignCat(k.ToString(System.Globalization.CultureInfo.InvariantCulture));
                    newName.AssignCat(".stp");
                    if (!_files.IsBound(newName.ToCString()))
                    {
                        break;
                    }
                }
            }

            // translate and record extern file
            ExternFile extFile = new ExternFile();
            extFile.SetWS(newWS);
            extFile.SetName(newName);
            extFile.SetLabel(label);
            Parameters.WriteMode_Assembly assemblymode = stepWriter.Model().InternalParameters.WriteAssembly;
            stepWriter.Model().InternalParameters.WriteAssembly = Parameters.WriteMode_Assembly.WriteMode_Assembly_Off;
            string isMulti = null;
            extFile.SetTransferStatus(TransferCore(stepWriter, labelSeq, mode, isMulti, true, progress));
            stepWriter.Model().InternalParameters.WriteAssembly = assemblymode;
            _labEF.Bind(label, extFile);
            _files.Bind(newName.ToCString(), extFile);

            // return empty compound as replacement for the shape
            _labels.Bind(label, comp);
            return comp;
        }
        NCollection_Sequence<Label> compLabels = new NCollection_Sequence<Label>();
        Label aLabel = label;
        // if specified shape is component then high-level assembly is considered
        // to get valid structure with location
        if (doc.IsComponent(label))
        {
            compLabels.Append(label);
            aLabel = label.Father();
        }
        // else iterate on components add create structure of empty compounds
        // representing the assembly
        else if (doc.IsAssembly(label))
        {
            doc.GetComponents(label, compLabels, false);
        }

        labels.Append(aLabel);
        using ProgressScope ps = new ProgressScope(progress, null, compLabels.Length());
        // Iterate on requested shapes
        for (int i = 1; i <= compLabels.Length() && ps.More(); i++)
        {
            Label curL = compLabels.Value(i);
            if (!doc.GetReferredShape(curL, out Label refL))
            {
                continue;
            }
            Shape shComp = TransferExternFiles(refL, mode, labels, localFactors, prefix, ps.Next());
            shComp = shComp.Located(doc.GetLocation(curL));
            builder.Add(comp, shComp);
        }
        _labels.Bind(aLabel, comp);
        return comp;
    }

    /// <summary>Write external references to STEP</summary>
    protected bool WriteExternRefs(ExchangeWorkSession ws, NCollection_Sequence<Label> labels)
    {
        if (labels.IsEmpty())
        {
            return false;
        }

        TransferWriter tw = ws.TransferWriter();
        FinderProcess fp = tw.FinderProcess();
        ExternRefs efTool = new ExternRefs(ws);
        StepModel stepModel = ws.Model() as StepModel;
        int stepSchema = (int)stepModel.InternalParameters.WriteSchema;
        // Iterate on requested shapes
        foreach (Label lab in labels)
        {
            if (lab.Document.IsAssembly(lab))
            {
                continue; // skip assemblies
            }

            // get extern file
            if (!ExternFile(lab, out ExternFile extFile))
            {
                continue; // should never be
            }

            // find SDR
            if (!_labels.IsBound(lab))
            {
                continue; // not recorded as translated, skip
            }
            Shape shape = _labels.Find(lab);

            StepShapeDefinitionRepresentation sdr = null;
            ShapeMapper mapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, shape);
            if (!fp.FindTypedTransient(mapper, typeof(StepShapeDefinitionRepresentation), ref sdr))
            {
                global::Kerf.Engine.Foundation.Message.SendTrace("Warning: Cannot find SDR for " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(shape.TShape().GetType()) + "\n");
                continue;
            }

            // add extern ref
            string stepFormat = stepSchema == 3 ? "STEP AP203" : "STEP AP214";
            // try to get PD from SDR
            RepresentedDefinition rd = sdr.Definition();
            PropertyDefinition propDef = rd.PropertyDefinition();
            if (propDef is null)
            {
                global::Kerf.Engine.Foundation.Message.SendTrace("Warning: STEPCAFControl_Writer::writeExternRefs StepRepr_PropertyDefinition is null for "
                                  + global::Kerf.Engine.Foundation.OcctTypeNames.Of(shape.TShape().GetType()) + "\n");
                continue;
            }
            CharacterizedDefinition charDef = propDef.Definition();
            ProductDefinition pd = charDef.ProductDefinition();
            if (pd is null)
            {
                global::Kerf.Engine.Foundation.Message.SendTrace("Warning: STEPCAFControl_Writer::writeExternRefs StepBasic_ProductDefinition is null for "
                                  + global::Kerf.Engine.Foundation.OcctTypeNames.Of(shape.TShape().GetType()) + "\n");
                continue;
            }
            efTool.AddExternRef(extFile.GetName().ToCString(), pd, stepFormat);
        }
        efTool.WriteExternRefs(stepSchema);
        return true;
    }

    private static int FindEntities(FinderProcess fp, Shape shape, ref Placement location,
                                    NCollection_Sequence<Transient> seqRI)
    {
        RepresentationItem item = StepConstruct.FindEntity(fp, shape, ref location);

        if (item is not null)
        {
            seqRI.Append(item);
            return 1;
        }

        // may be S was split during shape processing
        ShapeMapper mapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, shape);
        Binder binder = fp.Find(mapper);
        if (binder is null)
        {
            return 0;
        }

        TransientListBinder transientListBinder = binder as TransientListBinder;
        int resCount = 0;
        if (transientListBinder is null)
        {
            for (Iterator iter = new Iterator(shape); iter.More(); iter.Next())
            {
                RepresentationItem localItem = StepConstruct.FindEntity(fp, iter.Value(), ref location);
                if (localItem is null)
                {
                    continue;
                }
                resCount++;
                seqRI.Append(localItem);
            }
        }
        else
        {
            int nbTransient = transientListBinder.TransientCount;
            for (int ind = 1; ind <= nbTransient; ind++)
            {
                Transient entity = transientListBinder.Transient(ind);
                item = entity as RepresentationItem;
                if (item is null)
                {
                    continue;
                }
                resCount++;
                seqRI.Append(item);
            }
        }
        return resCount;
    }

    private static bool GetStyledItem(Shape shape, ExchangeDocument shapeTool, Styles styles,
                                      ref StyledItem resSelItem,
                                      NCollection_DataMap<Shape, Transient> mapCompMDGPR)
    {
        Label topShL = shapeTool.FindShape(shape, false);
        Shape topLevSh = topShL.IsNull ? new Shape() : shapeTool.GetShape(topShL);
        bool isFound = false;
        if (topLevSh.IsNull || !mapCompMDGPR.IsBound(topLevSh))
        {
            return false;
        }
        PresentationRepresentation mdgpr = mapCompMDGPR.Find(topLevSh) as PresentationRepresentation;
        NCollection_HArray1<RepresentationItem> selItmHArr = mdgpr.Items();
        if (selItmHArr is null)
        {
            return false;
        }
        // Search for PSA of Manifold solid
        NCollection_Sequence<Transient> newSeqRI = new NCollection_Sequence<Transient>();
        bool isFilled = false;
        for (int iter = selItmHArr.Lower(); iter <= selItmHArr.Upper() && !isFound; iter++)
        {
            StyledItem selItm = selItmHArr.Value(iter) as StyledItem;

            if (selItm is null)
            {
                continue;
            }
            // Check that it is a styled item for manifold solid brep
            if (!isFilled)
            {
                Placement loc = new Placement();
                FindEntities(styles.FinderProcess(), topLevSh, ref loc, newSeqRI);
                isFilled = true;
            }
            if (!newSeqRI.IsEmpty())
            {
                RepresentationItem item = selItm.Item();
                bool isSameMonSolBR = false;
                foreach (Transient iterRI in newSeqRI)
                {
                    if (item is not null && item == iterRI)
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
            NCollection_HArray1<PresentationStyleAssignment> aStyles = selItm.Styles();
            for (int styleIter = aStyles.Lower(); styleIter <= aStyles.Upper() && !isFound; styleIter++)
            {
                PresentationStyleAssignment fatherPSA = aStyles.Value(styleIter);
                // check for PSA for top-level (not Presentation style by context for NAUO)
                if (fatherPSA is null || fatherPSA.IsKind(typeof(PresentationStyleByContext)))
                {
                    continue;
                }
                resSelItem = selItm;
                isFound = true;
            }
        }
        return isFound;
    }

    private static bool SetDefaultInstanceColor(StyledItem styleItem, PresentationStyleAssignment psa)
    {
        bool isFound = false;
        NCollection_HArray1<PresentationStyleAssignment> styles = styleItem.Styles();
        for (int styleIter = styles.Lower(); styleIter <= styles.Upper() && !isFound; styleIter++)
        {
            PresentationStyleAssignment fatherPSA = styles.Value(styleIter);
            // check for PSA for top-level (not Presentation style by context for NAUO)
            if (fatherPSA is null || fatherPSA.IsKind(typeof(PresentationStyleByContext)))
            {
                return false;
            }

            // get style select from father PSA
            if (fatherPSA.StyleCount > 0)
            {
                NCollection_HArray1<PresentationStyleSelect> fatherStyles =
                    new NCollection_HArray1<PresentationStyleSelect>(1, fatherPSA.StyleCount);
                int settingInd = 1;
                NCollection_HArray1<PresentationStyleSelect> oldStyles = fatherPSA.Styles();
                for (int fatherStyleIter = oldStyles.Lower(); fatherStyleIter <= oldStyles.Upper(); fatherStyleIter++)
                {
                    PresentationStyleSelect pss = new PresentationStyleSelect();
                    PresentationStyleSelect olDPSS = oldStyles.Value(fatherStyleIter);
                    if (olDPSS.PointStyle() is not null)
                    {
                        pss.SetValue(olDPSS.PointStyle());
                    }
                    else if (olDPSS.CurveStyle() is not null)
                    {
                        pss.SetValue(olDPSS.CurveStyle());
                    }
                    else if (olDPSS.SurfaceStyleUsage() is not null)
                    {
                        pss.SetValue(olDPSS.SurfaceStyleUsage());
                    }
                    else
                    {
                        isFound = false;
                        break;
                    }
                    // OCCT stores a copy of the old select
                    fatherStyles.SetValue(settingInd++, new PresentationStyleSelect(olDPSS));
                    isFound = true;
                }
                // init PSA of NAUO
                if (isFound)
                {
                    psa.Init(fatherStyles);
                }
            }
        }
        return isFound;
    }

    private static void MakeSTEPStyles(Styles styles, Shape shape,
                                       NCollection_IndexedDataMap<Shape, Style> settings,
                                       ref StyledItem theOverride, NCollection_Map<Shape> map,
                                       NCollection_DataMap<Shape, Transient> mapCompMDGPR,
                                       NCollection_DataMap<string, Transient> dpdCs,
                                       NCollection_DataMap<Point3D, Transient> colRGBs, ExchangeDocument shTool,
                                       Style inherit, bool isComponent, bool visMaterialMode)
    {
        // skip already processed shapes
        if (!map.Add(shape))
        {
            return;
        }

        // check if shape has its own style (r inherits from ancestor)
        Style style = new Style();
        if (inherit is not null)
        {
            style = inherit.Copy();
        }
        if (settings.Contains(shape))
        {
            Style ownStyle = settings.FindFromKey(shape);
            if (!ownStyle.IsVisible)
            {
                style.IsVisible = false;
            }
            if (ownStyle.ColorCurv is not null)
            {
                style.ColorCurv = ownStyle.ColorCurv;
            }
            if (ownStyle.ColorSurf is not null)
            {
                style.ColorSurf = ownStyle.ColorSurf;
            }
            // OCCT: the visual material (none here, decision 1279)
        }

        // translate colors to STEP
        Colour surfColor = null;
        Colour curvColor = null;
        double renderTransp = 0.0;
        if (style.ColorSurf is ColorWithAlpha surfCol)
        {
            renderTransp = 1.0 - surfCol.Alpha();
            surfColor = Styles.EncodeColor(surfCol.GetRGB(), dpdCs, colRGBs);
        }
        if (style.ColorCurv is Color curvCol)
        {
            curvColor = Styles.EncodeColor(curvCol, dpdCs, colRGBs);
        }

        bool hasOwn = surfColor is not null || curvColor is not null || !style.IsVisible;

        // find target item and assign style to it
        StyledItem stePstyle = theOverride;
        if (hasOwn)
        {
            if (shape.ShapeType() != ShapeType.Compound || isComponent)
            { // skip compounds, let subshapes inherit its colors
                Placement location = new Placement();
                NCollection_Sequence<Transient> seqRI = new NCollection_Sequence<Transient>();
                int nbEntities = FindEntities(styles.FinderProcess(), shape, ref location, seqRI);
                if (nbEntities <= 0)
                {
                    global::Kerf.Engine.Foundation.Message.SendTrace("Warning: Cannot find RI for " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(shape.TShape().GetType()) + "\n");
                }
                // Get overridden style gka 10.06.03
                if (isComponent && nbEntities > 0)
                {
                    GetStyledItem(shape, shTool, styles, ref theOverride, mapCompMDGPR);
                }

                foreach (Transient entIter in seqRI)
                {
                    RepresentationItem item = entIter as RepresentationItem;
                    PresentationStyleAssignment psa;
                    if (style.IsVisible || surfColor is not null || curvColor is not null)
                    {
                        RenderingProperties renderProps = new RenderingProperties();
                        // OCCT: first the visual material, if theVisMaterialMode (none here)
                        _ = visMaterialMode;
                        if (renderTransp > 0.0)
                        {
                            renderProps.Init(style.ColorSurf.Value);
                        }
                        psa = styles.MakeColorPSA(item, surfColor, curvColor, renderProps, isComponent);
                    }
                    else
                    {
                        // default white color
                        surfColor = Styles.EncodeColor(new Color(NocWhite), dpdCs, colRGBs);
                        psa = styles.MakeColorPSA(item, surfColor, curvColor, new RenderingProperties(),
                                                      isComponent);
                        if (isComponent)
                        {
                            SetDefaultInstanceColor(theOverride, psa);
                        }
                    } // end of component case

                    stePstyle = styles.AddStyle(item, psa, theOverride);
                    hasOwn = false;
                }
            }
        }

        // iterate on subshapes (except vertices :)
        if (shape.ShapeType() == ShapeType.Edge)
        {
            return;
        }
        if (isComponent)
        {
            return;
        }
        for (Iterator iter = new Iterator(shape); iter.More(); iter.Next())
        {
            MakeSTEPStyles(styles, iter.Value(), settings, ref stePstyle, map, mapCompMDGPR, dpdCs,
                           colRGBs, shTool, hasOwn ? style : null, false, visMaterialMode);
        }
    }

    /// <summary>
    /// Write colors assigned to specified labels, to STEP model
    /// </summary>
    protected bool WriteColors(ExchangeWorkSession ws, NCollection_Sequence<Label> labels)
    {
        if (labels.IsEmpty())
        {
            return false;
        }

        Styles styles = new Styles(ws);
        NCollection_DataMap<string, Transient> dpdCs = new NCollection_DataMap<string, Transient>();
        NCollection_DataMap<Point3D, Transient> colRGBs = new NCollection_DataMap<Point3D, Transient>();
        // Iterate on requested shapes
        foreach (Label label in labels)
        {
            // Iterate on shapes in the document
            ExchangeDocument sTool = label.Document;
            // Skip assemblies: colors assigned to assemblies and their instances
            // are not supported (it is not clear how to encode that in STEP)
            if (sTool.IsAssembly(label))
            {
                global::Kerf.Engine.Foundation.Message.SendTrace("Warning: Cannot write color  for Assembly" + "\n");
                global::Kerf.Engine.Foundation.Message.SendTrace("Info: Check for colors assigned to components in assembly" + "\n");
                // PTV 22.01.2003 Write color for instances.
                NCollection_Sequence<Label> compLabels = new NCollection_Sequence<Label>();
                if (!sTool.GetComponents(label, compLabels))
                {
                    continue;
                }
                WriteColors(ws, compLabels);
                continue;
            }
            styles.ClearStyles();

            // get a target shape and try to find corresponding context
            // (all the colors set under that label will be put into that context)
            if (!sTool.GetShape(label, out Shape shape))
            {
                continue;
            }
            bool isComponent = sTool.IsComponent(label) || _pureRefLabels.IsBound(label);
            Shape topSh = shape;
            RepresentationContext context = styles.FindContext(shape);
            if (isComponent)
            {
                Label topShL = sTool.FindShape(shape, false);
                if (topShL.IsNull)
                {
                    continue;
                }
                topSh = sTool.GetShape(topShL);
                context = styles.FindContext(topSh);
            }
            if (context is null)
            {
                continue;
            }

            // collect settings set on that label
            NCollection_IndexedDataMap<Shape, Style> settings =
                new NCollection_IndexedDataMap<Shape, Style>(ShapeHasher.Instance);
            NCollection_Sequence<Label> seq = new NCollection_Sequence<Label>();
            seq.Append(label);
            sTool.GetSubShapes(label, seq);
            bool isVisible = true;
            foreach (Label seqValue in seq)
            {
                Style style = new Style();
                if (seqValue == label)
                {
                    // check for invisible status of object on label
                    if (!sTool.IsVisible(seqValue))
                    {
                        isVisible = false;
                        style.IsVisible = false;
                    }
                }
                if (sTool.GetColor(seqValue, ColorType.ColorGen, out ColorWithAlpha color))
                {
                    style.ColorCurv = color.GetRGB();
                    style.ColorSurf = color;
                }
                if (sTool.GetColor(seqValue, ColorType.ColorSurf, out color))
                {
                    style.ColorSurf = color;
                }
                if (sTool.GetColor(seqValue, ColorType.ColorCurv, out color))
                {
                    style.ColorCurv = color.GetRGB();
                }
                // OCCT: without a surface colour, the visual material's base colour (none here, decision 1279)
                if (style.ColorCurv is null && style.ColorSurf is null && isVisible)
                {
                    continue;
                }

                Shape sub = sTool.GetShape(seqValue);
                if (settings.Contains(sub))
                {
                    settings.ChangeFromKey(sub) = style;
                }
                else
                {
                    settings.Add(sub, style);
                }
            }

            if (settings.Extent() <= 0)
            {
                continue;
            }

            // iterate on subshapes and create STEP styles
            StyledItem anOverride = null;
            NCollection_Map<Shape> map = new NCollection_Map<Shape>(ShapeHasher.Instance);

            MakeSTEPStyles(styles, shape, settings, ref anOverride, map, _mapCompMDGPR, dpdCs, colRGBs, sTool, null,
                           isComponent, GetVisualMaterialMode());

            TransferWriter tw = ws.TransferWriter();
            FinderProcess fp = tw.FinderProcess();
            StepModel stepModel = fp.Model() as StepModel;

            // create MDGPR and record it in model
            MechanicalDesignGeometricPresentationRepresentation mdgpr;

            if (!isComponent)
            {
                if (_mapCompMDGPR.IsBound(topSh))
                {
                    global::Kerf.Engine.Foundation.Message.SendTrace("Error: Current Top-Level shape have MDGPR already " + "\n");
                }
                styles.CreateMDGPR(context, out mdgpr, stepModel);
                if (mdgpr is not null)
                {
                    _mapCompMDGPR.Bind(topSh, mdgpr);
                }
            }
            else
            {
                // create SDR and add to model.
                ShapeMapper mapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, shape);
                StepContextDependentShapeRepresentation cdsr = null;
                if (fp.FindTypedTransient(mapper, typeof(StepContextDependentShapeRepresentation), ref cdsr))
                {
                    // create SDR for NAUO
                    ProductDefinitionShape nullPDS = null; // important to be NULL
                    styles.CreateNAUOSRD(context, cdsr, nullPDS);

                    // search for MDGPR of the component top-level shape
                    if (_mapCompMDGPR.IsBound(topSh))
                    {
                        mdgpr = _mapCompMDGPR.Find(topSh) as MechanicalDesignGeometricPresentationRepresentation;
                    }
                    else
                    {
                        mdgpr = new MechanicalDesignGeometricPresentationRepresentation();
                        TCollection_HAsciiString reprName = new TCollection_HAsciiString("");
                        mdgpr.SetName(reprName);
                        mdgpr.SetContextOfItems(context);
                        _mapCompMDGPR.Bind(topSh, mdgpr);
                    }
                    NCollection_HArray1<RepresentationItem> oldItems = mdgpr.Items();
                    int oldLengthlen = 0;
                    if (oldItems is not null)
                    {
                        oldLengthlen = oldItems.Length();
                    }
                    int nbIt = oldLengthlen + styles.StyleCount;
                    if (nbIt == 0)
                    {
                        continue;
                    }
                    NCollection_HArray1<RepresentationItem> newItems = new NCollection_HArray1<RepresentationItem>(1, nbIt);
                    int elemInd = 1;
                    for (int styleInd = 1; styleInd <= oldLengthlen; styleInd++)
                    {
                        newItems.SetValue(elemInd++, oldItems.Value(styleInd));
                    }
                    for (int styleInd = 1; styleInd <= styles.StyleCount; styleInd++)
                    {
                        newItems.SetValue(elemInd++, styles.Style(styleInd));
                    }

                    if (newItems.Length() > 0)
                    {
                        mdgpr.SetItems(newItems);
                    }
                } // end of work with CDSR
            }
            if (!isVisible)
            {
                // create invisibility item and refer for styledItem
                Invisibility invisibility = new Invisibility();
                NCollection_HArray1<InvisibleItem> invisibilitySeq =
                    new NCollection_HArray1<InvisibleItem>(1, styles.StyleCount);
                // put all style item into the harray
                for (int styleInd = 1; styleInd <= styles.StyleCount; styleInd++)
                {
                    RepresentationItem styledItm = styles.Style(styleInd);
                    InvisibleItem invItem = new InvisibleItem();
                    invItem.SetValue(styledItm);
                    invisibilitySeq.SetValue(styleInd, invItem);
                }
                // set the invisibility of items
                invisibility.Init(invisibilitySeq);
                ws.Model().AddWithRefs(invisibility);
            }
        }
        return true;
    }

    /// <summary>Write names assigned to specified labels, to STEP model</summary>
    protected bool WriteNames(ExchangeWorkSession ws, NCollection_Sequence<Label> labels)
    {
        if (labels.IsEmpty())
        {
            return false;
        }

        // get working data
        TransferWriter tw = ws.TransferWriter();
        FinderProcess fp = tw.FinderProcess();

        // Iterate on requested shapes
        foreach (Label label in labels)
        {
            // find target STEP entity for the current shape
            if (!_labels.IsBound(label))
            {
                continue; // not recorded as translated, skip
            }
            // get name
            TCollection_HAsciiString hName = new TCollection_HAsciiString();
            if (!GetLabelName(label, hName))
            {
                continue;
            }
            Shape shape = _labels.Find(label);
            StepShapeDefinitionRepresentation sdr = null;
            StepContextDependentShapeRepresentation cdsr = null;
            bool isComponent = label.Document.IsComponent(label) || _pureRefLabels.IsBound(label);
            ShapeMapper mapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, shape);
            if (isComponent && fp.FindTypedTransient(mapper, typeof(StepContextDependentShapeRepresentation), ref cdsr))
            {
                ProductDefinitionShape pds = cdsr.RepresentedProductRelation();
                ProductDefinitionRelationship nauo = pds.Definition().ProductDefinitionRelationship();
                if (nauo is not null)
                {
                    nauo.SetName(hName);
                }
                if (_pureRefLabels.Find(label, out Shape internalAssembly))
                {
                    ShapeMapper mapperOfInternalShape = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, internalAssembly);
                    fp.FindTypedTransient(mapperOfInternalShape, typeof(StepShapeDefinitionRepresentation), ref sdr);
                }
            }
            if (sdr is not null
                || (cdsr is null && fp.FindTypedTransient(mapper, typeof(StepShapeDefinitionRepresentation), ref sdr)))
            {
                // set the name to the PRODUCT
                PropertyDefinition propD = sdr.Definition().PropertyDefinition();
                if (propD is null)
                {
                    continue;
                }
                ProductDefinition pd = propD.Definition().ProductDefinition();
                if (pd is null)
                {
                    continue;
                }
                Product prod = pd.Formation().OfProduct();

                prod.SetId(hName);
                prod.SetName(hName);
            }
            else
            {
                global::Kerf.Engine.Foundation.Message.SendTrace("Warning: Cannot find RI for " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(shape.TShape().GetType()) + "\n");
                continue;
            }
        }
        return true;
    }

    /// <summary>Write metadata assigned to specified labels, to STEP model</summary>
    /// <remarks>OCCT.NET: the document has no metadata (TDataStd_NamedData); nothing is written (decision 1279).</remarks>
    protected bool WriteMetadata(ExchangeWorkSession ws, NCollection_Sequence<Label> labels)
    {
        _ = ws;
        return !labels.IsEmpty();
    }

    private static bool WritePropsForLabel(ExchangeWorkSession ws, NCollection_DataMap<Label, Shape> labels,
                                           in Label label, string? isMulti)
    {
        _ = labels;
        if (label.IsNull)
        {
            return false;
        }

        ValidationProps props = new ValidationProps(ws);

        ExchangeDocument doc = label.Document;
        Shape shape = doc.GetShape(label);
        if (shape.IsNull)
        {
            return false;
        }

        DocumentValidationProps valProps = doc.GetValidationProps(label);
        if (isMulti is null || doc.IsAssembly(label))
        {
            // write area
            if (valProps?.Area is double area)
            {
                props.AddArea(shape, area);
            }
            // write volume
            if (valProps?.Volume is double volume)
            {
                props.AddVolume(shape, volume);
            }
        }
        // write centroid
        if (valProps?.Centroid is Point3D centroid)
        {
            props.AddCentroid(shape, centroid);
        }

        if (doc.IsCompound(label) || doc.IsAssembly(label))
        {
            if (doc.HasChild(label))
            {
                for (int childInd = 1; childInd <= doc.NbChildren(label); childInd++)
                {
                    WritePropsForLabel(ws, labels, doc.FindChild(label, childInd), isMulti);
                }
            }
        }

        return true;
    }

    /// <summary>Write validation properties assigned to specified labels, to STEP model</summary>
    protected bool WriteValProps(ExchangeWorkSession ws, NCollection_Sequence<Label> labels, string? isMulti)
    {
        if (labels.IsEmpty())
        {
            return false;
        }

        // Iterate on requested shapes
        foreach (Label label in labels)
        {
            WritePropsForLabel(ws, _labels, label, isMulti);
        }

        return true;
    }

    /// <summary>Write layers assigned to specified labels, to STEP model</summary>
    protected bool WriteLayers(ExchangeWorkSession ws, NCollection_Sequence<Label> labels)
    {
        if (labels.IsEmpty())
        {
            return false;
        }

        // get working data
        InterfaceModel model = ws.Model();
        TransferWriter tw = ws.TransferWriter();
        FinderProcess fp = tw.FinderProcess();
        NCollection_Sequence<Label> globalLayerLS = new NCollection_Sequence<Label>();
        // Iterate on requested shapes collect Tools
        foreach (ExchangeDocument lTool in _rootLabels)
        {
            NCollection_Sequence<Label> layerLS = new NCollection_Sequence<Label>();
            lTool.GetLayerLabels(layerLS);
            foreach (Label layer in layerLS)
            {
                globalLayerLS.Append(layer);
            }
        }
        foreach (Label layerL in globalLayerLS)
        {
            ExchangeDocument doc = layerL.Document;
            // get labels of shapes in that layer
            NCollection_Sequence<Label> shapeLabels = new NCollection_Sequence<Label>();
            doc.GetShapesOfLayer(layerL, shapeLabels);
            if (shapeLabels.IsEmpty())
            {
                continue;
            }

            // name of layer: if not set, is considered as being empty
            TCollection_HAsciiString hName = new TCollection_HAsciiString();
            GetLabelName(layerL, hName);

            // Find target STEP entity for each shape and add to StepVisual_PresentationLayerAssignment
            // items.
            NCollection_Sequence<Transient> seqRI = new NCollection_Sequence<Transient>();
            foreach (Label shLabel in shapeLabels)
            {
                if (shLabel.IsNull)
                {
                    continue;
                }

                // there is no way to assign layer to instance in STEP
                if (doc.IsAssembly(shLabel) || doc.IsReference(shLabel))
                {
                    continue;
                }
                // check that the shape is one of (uub)labels written during current transfer
                bool isWritten = false;
                foreach (Label labelIter in labels)
                {
                    if (doc.IsDescendant(shLabel, labelIter))
                    {
                        isWritten = true;
                        break;
                    }
                }
                if (!isWritten)
                {
                    continue;
                }

                // get target STEP entity
                Shape oneShape = doc.GetShape(shLabel);

                Placement loc = new Placement();
                int nb = FindEntities(fp, oneShape, ref loc, seqRI);
                if (nb <= 0)
                {
                    global::Kerf.Engine.Foundation.Message.SendTrace("Warning: Cannot find RI for " + (oneShape.TShape() is { } tShape ? global::Kerf.Engine.Foundation.OcctTypeNames.Of(tShape.GetType()) : null) + "\n");
                }
            }
            if (seqRI.IsEmpty())
            {
                continue;
            }

            // analyze visibility
            PresentationLayerAssignment stepLayerAs = new PresentationLayerAssignment();
            TCollection_HAsciiString descr;
            bool isLinv = false;
            if (!doc.IsVisible(layerL))
            {
                descr = new TCollection_HAsciiString("invisible");
                global::Kerf.Engine.Foundation.Message.SendTrace("\tLayer \"" + hName.ToCString() + "\" is invisible" + "\n");
                isLinv = true;
            }
            else
            {
                descr = new TCollection_HAsciiString("visible");
            }

            // create layer entity
            int setStyleInd = 1;
            NCollection_HArray1<LayeredItem> hArrayOfLItem = new NCollection_HArray1<LayeredItem>(1, seqRI.Length());
            foreach (Transient riIter in seqRI)
            {
                LayeredItem li = new LayeredItem();
                li.SetValue(riIter);
                hArrayOfLItem.SetValue(setStyleInd++, li);
            }
            stepLayerAs.Init(hName, descr, hArrayOfLItem);
            model.AddWithRefs(stepLayerAs);
            // PTV 23.01.2003 add the invisibility AFTER adding layer into the model.
            // add the invisibility for the layer
            if (isLinv)
            {
                // Invisibility Item for containing invisible layers.
                NCollection_HArray1<InvisibleItem> hInvsblItm = new NCollection_HArray1<InvisibleItem>(1, 1);
                InvisibleItem invIt = new InvisibleItem();
                invIt.SetValue(stepLayerAs);
                hInvsblItm.SetValue(1, invIt);

                Invisibility invsblt = new Invisibility();
                invsblt.Init(hInvsblItm);
                model.AddWithRefs(invsblt);
            }
        }
        return true;
    }

    /// <summary>Write SHUO assigned to specified component, to STEP model</summary>
    /// <remarks>OCCT.NET: the document has no SHUOs; nothing is written (decision 1279).</remarks>
    protected bool WriteSHUOs(ExchangeWorkSession ws, NCollection_Sequence<Label> labels)
    {
        _ = ws;
        return !labels.IsEmpty();
    }

    /// <summary>Write D&amp;GTs assigned to specified labels, to STEP model</summary>
    /// <remarks>OCCT.NET: the document has no GD&amp;T; nothing is written (decision 1279).</remarks>
    protected bool WriteDGTs(ExchangeWorkSession ws, NCollection_Sequence<Label> labels)
    {
        _ = ws;
        _ = labels;
        return false;
    }

    /// <summary>Write D&amp;GTs assigned to specified labels, to STEP model, according AP242</summary>
    /// <remarks>OCCT.NET: the document has no GD&amp;T; nothing is written (decision 1279).</remarks>
    protected bool WriteDGTsAP242(ExchangeWorkSession ws, NCollection_Sequence<Label> labels,
                                  in Factors localFactors)
    {
        _ = ws;
        _ = labels;
        return true;
    }

    /// <summary>Write materials assigned to specified labels, to STEP model</summary>
    /// <remarks>OCCT.NET: the document has no materials; nothing is written (decision 1279).</remarks>
    protected bool WriteMaterials(ExchangeWorkSession ws, NCollection_Sequence<Label> labels)
    {
        _ = ws;
        return !labels.IsEmpty();
    }

    private Writer _writer;
    private readonly NCollection_DataMap<string, ExternFile> _files = new NCollection_DataMap<string, ExternFile>();
    private readonly NCollection_DataMap<Label, ExternFile> _labEF = new NCollection_DataMap<Label, ExternFile>();
    private readonly NCollection_DataMap<Label, Shape> _labels = new NCollection_DataMap<Label, Shape>();
    private readonly NCollection_DataMap<Label, Shape> _pureRefLabels = new NCollection_DataMap<Label, Shape>();
    // OCCT: the documents' root labels (a map); here the documents, in the order given
    private readonly List<ExchangeDocument> _rootLabels = new List<ExchangeDocument>();
    private readonly NCollection_DataMap<Shape, Transient> _mapCompMDGPR =
        new NCollection_DataMap<Shape, Transient>(ShapeHasher.Instance);
    private bool _colorMode = true;
    private bool _nameMode = true;
    private bool _layerMode = true;
    private bool _propsMode = true;
    private bool _metadataMode = true;
    private bool _shuoMode = true;
    private bool _gdtMode = true;
    private bool _matMode = true;
    private bool _visMatMode;
    private bool _isCleanDuplicates;
}
