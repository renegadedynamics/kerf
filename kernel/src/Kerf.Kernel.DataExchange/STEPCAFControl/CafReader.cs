// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_Reader.hxx
// Port of: src/DataExchange/TKDESTEP/STEPCAFControl/STEPCAFControl_Reader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Construct;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Documents;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Parser;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Exchange.Framework.Selection.ReturnStatus;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Exchange.Documents.ColorType;

namespace Kerf.Engine.Exchange.Step.Caf;

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
/// Kerf: the document is an <see cref="ExchangeDocument"/> (decision 1166, 1231), whose labels and methods stand in
/// for OCAF's <c>TDF_Label</c>s and XCAF's tools; the reader fills it as OCCT fills an XCAF document, in OCCT's order.
/// Shapes, assemblies, external references, names, colours (surface, curve, per sub-shape, visibility), layers,
/// validation properties, the length unit and, if the session's <c>read.stepcaf.subshapes.name</c> asks, the
/// sub-shapes' names are read. SHUOs, GD&amp;T, materials, views and metadata are not (decision 1279): their modes are
/// kept, and their methods read nothing and return false. A colour of an instance of a sub-assembly's component (an
/// XCAF SHUO) is not kept (decision 1278). The session gets a STEPCAFControl_Controller of its own (decision 1264).
/// The document's shapes are those the reader publishes, frozen.
/// </para>
/// <para>
/// Kerf (unit 8Z): external files are opened by a resolver (<see cref="SetExternFileResolver"/>): from the disk as
/// OCCT for a file loaded by ReadFile, none for one loaded by ReadStream unless a resolver is given, none at all with
/// <see cref="SetExternFilesMode"/> false; each is read with the caller's limits and cancellation, within a budget
/// across them (<see cref="SetExternFileLimits"/>), and a cycle of references stops.
/// </para>
/// </remarks>
internal class CafReader
{
    /// <summary>Creates a reader with an empty STEP model and sets ColorMode, LayerMode, NameMode and PropsMode to true.</summary>
    public CafReader()
    {
        _colorMode = true;
        _nameMode = true;
        _layerMode = true;
        _propsMode = true;
        _metaMode = true;
        _productMetaMode = false;
        _shuoMode = false;
        _gdtMode = true;
        _matMode = true;
        _viewMode = true;
        CafController.Init();
        _reader = new StepReader();
        if (_reader.WS() is not null)
        {
            Init(_reader.WS());
        }
    }

    /// <summary>
    /// Creates a reader tool and attaches it to an already existing Session Clears the session if it was not yet set
    /// for STEP
    /// </summary>
    public CafReader(ExchangeWorkSession ws, bool scratch = true)
    {
        _colorMode = true;
        _nameMode = true;
        _layerMode = true;
        _propsMode = true;
        _metaMode = true;
        _productMetaMode = false;
        _shuoMode = false;
        _gdtMode = true;
        _matMode = true;
        _viewMode = true;
        CafController.Init();
        _reader = new StepReader();
        Init(ws, scratch);
    }

    /// <summary>Clears the internal data structures and attaches to a new session Clears the session if it was not yet set for STEP</summary>
    public void Init(ExchangeWorkSession ws, bool scratch = true)
    {
        // necessary only in Writer, to set good actor:  WS->SelectNorm ( "STEP" );
        // Kerf: the session's controller is a STEPCAFControl_Controller of its own (decision 1264)
        CafController.SelectFor(ws);
        _reader.SetWS(ws, scratch);
        _files.Clear();
        _map.Clear();
        ResetExternBudget(false, null);
    }

    /// <summary>
    /// Converts a name read from the file: OCCT makes a TCollection_ExtendedString of it, treating it as UTF-8 if the
    /// model's source code page isn't NoConversion. The port's strings are decoded by the parser already.
    /// </summary>
    protected virtual string ConvertName(string name)
    {
        return name;
    }

    /// <summary>Loads a file and returns the read status Provided for use like single-file reader.</summary>
    public ReturnStatus ReadFile(string fileName)
    {
        ResetExternBudget(false, null);
        return _reader.ReadFile(fileName);
    }

    /// <summary>Loads a file with the given parameters and returns the read status</summary>
    public ReturnStatus ReadFile(string fileName, in Parameters theParams)
    {
        ResetExternBudget(false, null);
        return _reader.ReadFile(fileName, theParams);
    }

    /// <summary>
    /// Loads a file with limits for hostile input (Kerf, plan 2.7). The limits apply to the external files too
    /// (unit 8Z).
    /// </summary>
    public ReturnStatus ReadFile(string fileName, ProgressRange progress, ParserReadLimits limits)
    {
        ResetExternBudget(false, limits);
        return _reader.ReadFile(fileName, progress, limits);
    }

    /// <summary>Loads a file from stream and returns the read status.</summary>
    /// <remarks>
    /// Kerf (unit 8Z): external references are read only through a resolver given by
    /// <see cref="SetExternFileResolver"/>; without one, none is read (OCCT reads them from the disk).
    /// </remarks>
    public ReturnStatus ReadStream(string name, Stream iStream)
    {
        ResetExternBudget(true, null);
        return _reader.ReadStream(name, iStream);
    }

    /// <summary>
    /// Loads a file from stream with limits for hostile input (Kerf, plan 2.7). The limits apply to the external
    /// files too, which are read only through a resolver (unit 8Z).
    /// </summary>
    public ReturnStatus ReadStream(string name, Stream iStream, ProgressRange progress,
                                            ParserReadLimits limits)
    {
        ResetExternBudget(true, limits);
        return _reader.ReadStream(name, iStream, progress, limits);
    }

    // ---- Kerf (unit 8Z): how external files are read ------------------------------------------------------

    /// <summary>
    /// Kerf (unit 8Z): whether external references are read at all (default true). When false, none is opened and
    /// every reference stays unresolved as OCCT leaves a reference to a missing file: the transient process records
    /// "Can not read external file", the shape's label records the file's name (<c>SetExternRefs</c>), and the
    /// <see cref="ExternFiles"/> entry has the load status <c>IFSelect_RetError</c>.
    /// </summary>
    public void SetExternFilesMode(bool mode) => _externFilesMode = mode;

    /// <summary>Kerf (unit 8Z): see <see cref="SetExternFilesMode"/>.</summary>
    public bool GetExternFilesMode() => _externFilesMode;

    /// <summary>
    /// Kerf (unit 8Z): opens the external files. When null (the default), a file loaded by ReadFile reads them from
    /// the disk as OCCT does (<see cref="FileSystemResolver"/>), and a file loaded by ReadStream reads none. A resolver
    /// other than <see cref="FileSystemResolver"/> names the files it is given lexically: an external file's name is
    /// its reference combined with the referencing file's folder, <c>.</c> and <c>..</c> folded, without the disk or
    /// the current directory (OCCT: <c>OSD_Path::AbsolutePath</c>); that name is what its own references are resolved
    /// against. The resolver is also asked whether a file exists (the stream it returns is then disposed unread), where
    /// OCCT looks on the disk to choose between the names a reference gives.
    /// </summary>
    public void SetExternFileResolver(ExternFileResolver? resolver) => _externResolver = resolver;

    /// <summary>Kerf (unit 8Z): see <see cref="SetExternFileResolver"/>.</summary>
    public ExternFileResolver? GetExternFileResolver() => _externResolver;

    /// <summary>
    /// Kerf (unit 8Z): the budget of the external reads for one main file (files, bytes, depth). A reference past
    /// the budget, or to a file being read already (a cycle), stays unresolved with a warning in the transient process.
    /// </summary>
    public void SetExternFileLimits(ExternFileLimits limits)
    {
        ArgumentNullException.ThrowIfNull(limits);
        _externLimits = limits;
    }

    /// <summary>Kerf (unit 8Z): see <see cref="SetExternFileLimits"/>.</summary>
    public ExternFileLimits GetExternFileLimits() => _externLimits;

    /// <summary>
    /// Kerf (unit 8Z): OCCT's way, the file a reference names read from the disk: an absolute name as it is, a
    /// relative one beside the referencing file (<c>OSD_Path::AbsolutePath</c>; as it is if the referencing name has
    /// no folder, that is from the current directory). The default of the file API.
    /// </summary>
    public static ExternFileResolver FileSystemResolver { get; } = (fileName, referencingName) =>
    {
        string fullName = ExternRefs.AbsolutePath(DirectoryOf(referencingName), fileName);
        if (fullName.Length <= 0)
        {
            fullName = fileName;
        }
        return OpenFile(fullName);
    };

    /// <summary>
    /// Kerf (unit 8Z): a resolver confined to a folder: a reference is resolved beside the referencing file (its
    /// folder taken inside <paramref name="root"/>; a referencing name outside the folder, or absolute, stands for the
    /// folder itself) and read only if it lies inside the folder. Absolute names, drive-relative names, UNC and device
    /// paths, names with a colon (alternate data streams) and names leaving the folder by <c>..</c> are refused, as are
    /// files reached through a link (a reparse point) below the folder.
    /// </summary>
    public static ExternFileResolver FolderResolver(string root)
    {
        ArgumentException.ThrowIfNullOrEmpty(root);
        string aRoot = Path.TrimEndingDirectorySeparator(Path.GetFullPath(root)) + Path.DirectorySeparatorChar;
        StringComparison cmp = OperatingSystem.IsWindows() ? StringComparison.OrdinalIgnoreCase : StringComparison.Ordinal;
        return (fileName, referencingName) =>
        {
            if (string.IsNullOrEmpty(fileName) || fileName.Contains('\0') || fileName.Contains(':')
                || Path.IsPathRooted(fileName) || fileName.StartsWith('/') || fileName.StartsWith('\\'))
            {
                return null;
            }
            string dir = aRoot;
            string refDir = DirectoryOf(referencingName ?? "");
            if (refDir.Length > 0 && !refDir.Contains('\0') && !refDir.Contains(':') && !Path.IsPathRooted(refDir)
                && !refDir.StartsWith('/') && !refDir.StartsWith('\\'))
            {
                string candidate = Path.TrimEndingDirectorySeparator(Path.GetFullPath(Path.Combine(aRoot, refDir)))
                                    + Path.DirectorySeparatorChar;
                if (candidate.StartsWith(aRoot, cmp))
                {
                    dir = candidate;
                }
            }
            string path;
            try
            {
                path = Path.GetFullPath(Path.Combine(dir, fileName));
            }
            catch (Exception exc) when (exc is ArgumentException or NotSupportedException or PathTooLongException)
            {
                return null;
            }
            if (!path.StartsWith(aRoot, cmp) || path.Length <= aRoot.Length)
            {
                return null;
            }
            // no link below the folder (a file or folder reparse point could lead out of it)
            for (string? step = path; step is not null && step.Length > aRoot.Length; step = Path.GetDirectoryName(step))
            {
                try
                {
                    if ((File.GetAttributes(step) & FileAttributes.ReparsePoint) != 0)
                    {
                        return null;
                    }
                }
                catch (Exception exc) when (exc is IOException or UnauthorizedAccessException)
                {
                    return null;
                }
            }
            return OpenFile(path);
        };
    }

    private static string DirectoryOf(string file)
    {
        if (string.IsNullOrEmpty(file))
        {
            return "";
        }
        try
        {
            return Path.GetDirectoryName(file) ?? "";
        }
        catch (ArgumentException)
        {
            return "";
        }
    }

    private static Stream OpenFile(string path)
    {
        try
        {
            return new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read);
        }
        catch (Exception exc) when (exc is IOException or UnauthorizedAccessException or ArgumentException or NotSupportedException)
        {
            return null;
        }
    }

    // A name combined lexically (a resolver's naming): the referencing file's folder and the reference, with '.' and
    // '..' folded and '/' as the separator; an absolute reference as it is
    internal static string CombineName(string referencingName, string fileName)
    {
        if (string.IsNullOrEmpty(fileName))
        {
            return "";
        }
        bool isRooted;
        try
        {
            isRooted = Path.IsPathRooted(fileName);
        }
        catch (ArgumentException)
        {
            isRooted = true;
        }
        if (isRooted || fileName.StartsWith('/') || fileName.StartsWith('\\'))
        {
            return fileName;
        }
        string dir = DirectoryOf(referencingName);
        return NormalizeName(dir.Length > 0 ? dir + "/" + fileName : fileName);
    }

    internal static string NormalizeName(string name)
    {
        if (string.IsNullOrEmpty(name))
        {
            return "";
        }
        string root;
        try
        {
            root = Path.GetPathRoot(name) ?? "";
        }
        catch (ArgumentException)
        {
            root = "";
        }
        var parts = new List<string>();
        foreach (string part in name.Substring(root.Length).Split('/', '\\'))
        {
            if (part.Length == 0 || part == ".")
            {
                continue;
            }
            if (part == ".." && parts.Count > 0 && parts[^1] != "..")
            {
                parts.RemoveAt(parts.Count - 1);
                continue;
            }
            if (part == ".." && root.Length > 0)
            {
                continue; // above a root: the root
            }
            parts.Add(part);
        }
        return root.Replace('\\', '/') + string.Join("/", parts);
    }

    private ExternFileResolver EffectiveResolver()
    {
        if (!_externFilesMode)
        {
            return null;
        }
        if (_externResolver is not null)
        {
            return _externResolver;
        }
        return _isStreamLoaded ? null : FileSystemResolver;
    }

    private void ResetExternBudget(bool isStream, ParserReadLimits limits)
    {
        _isStreamLoaded = isStream;
        _readLimits = limits;
        _externCount = 0;
        _externBytes = 0;
        _externStack.Clear();
    }

    private static bool ProbeResolver(ExternFileResolver resolver, string fileName, string referencingName)
    {
        Stream stream = CallResolver(resolver, fileName, referencingName);
        if (stream is null)
        {
            return false;
        }
        stream.Dispose();
        return true;
    }

    private static Stream CallResolver(ExternFileResolver resolver, string fileName, string referencingName)
    {
        try
        {
            Stream stream = resolver(fileName, referencingName);
            if (stream is not null && !stream.CanRead)
            {
                stream.Dispose();
                return null;
            }
            return stream;
        }
        catch (Exception exc) when (exc is IOException or UnauthorizedAccessException or ArgumentException or NotSupportedException)
        {
            return null;
        }
    }

    // Why a reference isn't read (a cycle, past the budget), or null
    private string ExternRefusal(string fullName)
    {
        foreach (string name in _externStack)
        {
            if (string.Equals(name, fullName, StringComparison.OrdinalIgnoreCase))
            {
                return "External reference file is being read already (a cycle of references): " + fullName;
            }
        }
        if (_externStack.Count > _externLimits.MaxDepth)
        {
            return "External reference file not read: deeper than " + _externLimits.MaxDepth + " files: " + fullName;
        }
        if (_externCount >= _externLimits.MaxFiles)
        {
            return "External reference file not read: more than " + _externLimits.MaxFiles + " files: " + fullName;
        }
        if (_externBytes >= _externLimits.MaxTotalSize)
        {
            return "External reference file not read: the external files' " + _externLimits.MaxTotalSize
                   + " bytes are read: " + fullName;
        }
        return null;
    }

    /// <summary>Returns number of roots recognized for transfer Shortcut for Reader().NbRootsForTransfer()</summary>
    public int RootsForTransferCount
    {
        get
        {
            return _reader.NbRootsForTransfer();
        }
    }

    /// <summary>
    /// Translates currently loaded STEP file into the document Returns True if succeeded, and False in case of fail
    /// Provided for use like single-file reader
    /// </summary>
    public bool TransferOneRoot(int num, ExchangeDocument doc, ProgressRange progress = default)
    {
        NCollection_Sequence<Label> lseq = new NCollection_Sequence<Label>();
        return Transfer(_reader, num, doc, lseq, false, progress);
    }

    /// <summary>
    /// Translates currently loaded STEP file into the document Returns True if succeeded, and False in case of fail
    /// Provided for use like single-file reader
    /// </summary>
    public bool Transfer(ExchangeDocument doc, ProgressRange progress = default)
    {
        NCollection_Sequence<Label> lseq = new NCollection_Sequence<Label>();
        return Transfer(_reader, 0, doc, lseq, false, progress);
    }

    public bool Perform(string filename, ExchangeDocument doc, ProgressRange progress = default)
    {
        if (ReadFile(filename) != ReturnStatus.RetDone)
        {
            return false;
        }
        return Transfer(doc, progress);
    }

    public bool Perform(string filename, ExchangeDocument doc, in Parameters theParams,
                        ProgressRange progress = default)
    {
        if (ReadFile(filename, theParams) != ReturnStatus.RetDone)
        {
            return false;
        }
        return Transfer(doc, progress);
    }

    /// <summary>Returns data on external files Returns Null handle if no external files are read</summary>
    public Dictionary<string, ExternFile> ExternFiles()
    {
        return _files;
    }

    /// <summary>Returns data on external file by its name Returns False if no external file with given name is read</summary>
    public bool ExternFile(string name, out ExternFile ef)
    {
        ef = null;
        if (_files.Count == 0 || !_files.ContainsKey(name))
        {
            return false;
        }
        ef = _files.GetValue(name);
        return true;
    }

    /// <summary>Returns basic reader</summary>
    public StepReader ChangeReader()
    {
        return _reader;
    }

    /// <summary>Returns basic reader as const</summary>
    public StepReader Reader()
    {
        return _reader;
    }

    /// <summary>The map of the shapes read to their labels</summary>
    public Dictionary<Shape, Label> GetShapeLabelMap() => _map;

    //=======================================================================
    // function : FillShapesMap
    // purpose  : auxiliary: fill a map by all compounds and their components
    //=======================================================================
    private static void FillShapesMap(Shape s, HashSet<Shape> map)
    {
        Shape s0 = s.Located(new Placement());
        map.Add(s0);
        if (s.Type != ShapeType.Compound)
        {
            return;
        }
        for (Iterator it = new Iterator(s); it.More(); it.Next())
        {
            FillShapesMap(it.Value(), map);
        }
    }

    private void PrepareUnits(StepModel model, ExchangeDocument doc, ref Factors localFactors)
    {
        double scaleFactorMM = 1.0;
        if (doc.GetLengthUnit(out double unitM))
        {
            scaleFactorMM = unitM * global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetLengthUnitScale(FrameworkLengthUnit.LengthUnit_Meter,
                                                                     FrameworkLengthUnit.LengthUnit_Millimeter);
        }
        else
        {
            // OCCT: XSAlgo_ShapeProcessor::PrepareForTransfer() then the process-wide unit; here the session's
            // (decision 1265)
            scaleFactorMM = global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetCasCadeLengthUnit(_reader.WS().Statics());
            // Sets length unit to the document
            double unitValue = scaleFactorMM * global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetLengthUnitScale(FrameworkLengthUnit.LengthUnit_Millimeter,
                                                                                FrameworkLengthUnit.LengthUnit_Meter);
            doc.SetLengthUnit(unitValue, global::Kerf.Engine.Exchange.Framework.UnitsMethods.DumpLengthUnit(scaleFactorMM, FrameworkLengthUnit.LengthUnit_Millimeter));
        }
        localFactors.SetCascadeUnit(scaleFactorMM);
        model.SetLocalLengthUnit(scaleFactorMM);
    }

    /// <summary>
    /// Translates STEP file already loaded into the reader into the document If num==0, translates all roots, else
    /// only root number num Returns True if succeeded, and False in case of fail If asOne is True, in case of multiple
    /// results composes them into assembly. Fills sequence of produced labels
    /// </summary>
    protected bool Transfer(StepReader reader, int nroot, ExchangeDocument doc,
                            NCollection_Sequence<Label> lseq, bool asOne = false,
                            ProgressRange progress = default)
    {
        reader.ClearShapes();
        StepModel model = reader.Model() as StepModel;
        Factors localFactors = new Factors();
        PrepareUnits(model, doc, ref localFactors);
        int i;

        // Read all shapes
        int num = reader.NbRootsForTransfer();
        if (num <= 0)
        {
            return false;
        }

        using ProgressScope psRoot = new ProgressScope(progress, null, 2);

        if (nroot != 0)
        {
            if (nroot > num)
            {
                return false;
            }
            reader.TransferOneRoot(nroot, psRoot.Next());
        }
        else
        {
            using ProgressScope ps = new ProgressScope(psRoot.Next(), null, num);
            for (i = 1; i <= num && ps.More(); i++)
            {
                reader.TransferOneRoot(i, ps.Next());
            }
        }
        if (psRoot.UserBreak())
        {
            return false;
        }

        num = reader.ShapeCount;
        if (num <= 0)
        {
            return false;
        }

        // Fill a map of (top-level) shapes resulting from that transfer
        // Only these shapes will be considered further
        HashSet<Shape> shapesMap = new HashSet<Shape>(ShapeHasher.Instance);
        HashSet<Shape> newShapesMap = new HashSet<Shape>(ShapeHasher.Instance);
        for (i = 1; i <= num; i++)
        {
            FillShapesMap(reader.Shape(i), shapesMap);
        }

        // Collect information on shapes originating from SDRs
        // this will be used to distinguish compounds representing assemblies
        // from the ones representing hybrid models and shape sets
        // Kerf: the map iterates in insertion order (ExpandSubShapes iterates it; decision 1280)
        Dictionary<Shape, ProductDefinition> shapePDMap =
            new Dictionary<Shape, ProductDefinition>(ShapeHasher.Instance);
        Dictionary<ProductDefinition, ExternFile> pdFileMap =
            new Dictionary<ProductDefinition, ExternFile>(ReferenceEqualityComparer.Instance);
        TransientProcess tp = reader.WS().TransferReader().TransientProcess();
        int nb = model.EntityCount;

        NCollection_HSequence<Transient> seqPDS = new NCollection_HSequence<Transient>();

        for (i = 1; i <= nb; i++)
        {
            Transient enti = model.Value(i);
            if (enti.IsKind(typeof(ProductDefinitionShape)))
            {
                // sequence for acceleration ReadMaterials
                seqPDS.Append(enti);
            }
            if (enti.IsKind(typeof(ProductDefinition)))
            {
                ProductDefinition pd = enti as ProductDefinition;
                int index = tp.MapIndex(pd);
                if (index > 0)
                {
                    Binder binder = tp.MapItem(index);
                    Shape s = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
                    if (!s.IsNull && shapesMap.Contains(s))
                    {
                        newShapesMap.Add(s);
                        shapePDMap[s] = pd;
                        pdFileMap[pd] = null;
                    }
                }
            }
            if (enti.IsKind(typeof(StepShapeRepresentation)))
            {
                int index = tp.MapIndex(enti);
                if (index > 0)
                {
                    Binder binder = tp.MapItem(index);
                    Shape s = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
                    if (!s.IsNull && shapesMap.Contains(s))
                    {
                        newShapesMap.Add(s);
                    }
                }
            }
        }

        // get file name and directory name of the main file
        string loadedFile = reader.WS().LoadedFile() ?? "";
        string mainName = Path.GetFileName(loadedFile);
        string dpath = Path.GetDirectoryName(loadedFile) ?? "";

        // Kerf (unit 8Z): the resolver opens the external files; OCCT's naming unless it is another than the disk's
        ExternFileResolver resolver = EffectiveResolver();
        bool isOcctNaming = resolver is null || ReferenceEquals(resolver, FileSystemResolver);

        // Load external references (only for relevant SDRs)
        // and fill map SDR -> extern file
        ExternRefs extRefs = new ExternRefs(reader.WS());
        if (resolver is null)
        {
            extRefs.InternalFileExists = (_, _) => false;
        }
        else if (!isOcctNaming)
        {
            extRefs.InternalFileExists = (name, referencing) => ProbeResolver(resolver, name, referencing);
        }
        extRefs.LoadExternRefs();
        using ProgressScope pse = new ProgressScope(psRoot.Next(), null, extRefs.ExternRefCount);
        // Kerf: the files being read, the main one first, against cycles (OCCT recurses without end)
        bool isTopFile = _externStack.Count == 0;
        if (isTopFile)
        {
            string topName = isOcctNaming ? ExternRefs.AbsolutePath(dpath, mainName) : NormalizeName(loadedFile);
            _externStack.Add(topName.Length > 0 ? topName : loadedFile);
        }
        try
        {
            for (i = 1; i <= extRefs.ExternRefCount && pse.More(); i++)
            {
                ProgressRange range = pse.Next();
                // check extern ref format
                TCollection_HAsciiString format = extRefs.Format(i);
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
                string filename = extRefs.FileName(i);

                if (string.IsNullOrEmpty(filename))
                {
                    continue; // not a valid extern ref
                }

                // compute true path to the extern file
                // Kerf: a resolver's name is lexical (unit 8Z)
                string fullname = isOcctNaming ? ExternRefs.AbsolutePath(dpath, filename) : CombineName(loadedFile, filename);
                if (fullname.Length <= 0)
                {
                    fullname = filename;
                }

                // check for not the same file
                string mainFullName = isOcctNaming ? ExternRefs.AbsolutePath(dpath, mainName) : NormalizeName(loadedFile);
                if (string.Equals(mainFullName, fullname, StringComparison.OrdinalIgnoreCase))
                {
                    tp.AddWarning(extRefs.DocFile(i), "External reference file is the same main file");
                    continue; // not a valid extern ref
                }

                // get and check PD associated with the current extern ref
                ProductDefinition pd = extRefs.ProdDef(i);
                if (pd is null)
                {
                    continue; // not a valid extern ref
                }
                if (!pdFileMap.ContainsKey(pd))
                {
                    continue; // this PD is not concerned by current transfer
                }

                // Kerf (unit 8Z): a cycle or a reference past the budget stays unresolved, with a warning
                if (resolver is not null && !_files.ContainsKey(filename))
                {
                    string refusal = ExternRefusal(fullname);
                    if (refusal is not null)
                    {
                        tp.AddWarning(extRefs.DocFile(i) ?? (Transient)pd, refusal);
                        ExternFile unread = new ExternFile();
                        unread.SetName(new TCollection_HAsciiString(filename));
                        unread.SetLoadStatus(ReturnStatus.RetError);
                        pdFileMap[pd] = unread;
                        range.Close();
                        continue;
                    }
                }

                // read extern file (or use existing data) and record its data
                ExternFile ef = ReadExternFile(filename, fullname, loadedFile, doc, range);
                pdFileMap[pd] = ef;
            }
        }
        finally
        {
            if (isTopFile)
            {
                _externStack.Clear();
            }
        }

        // and insert them to the document
        if (asOne)
        {
            lseq.Append(AddShape(reader.OneShape(), doc, newShapesMap, shapePDMap, pdFileMap));
        }
        else
        {
            for (i = 1; i <= num; i++)
            {
                lseq.Append(AddShape(reader.Shape(i), doc, newShapesMap, shapePDMap, pdFileMap));
            }
        }

        // read colors
        if (GetColorMode())
        {
            ReadColors(reader.WS(), doc, localFactors);
        }

        // read names
        if (GetNameMode())
        {
            ReadNames(reader.WS(), doc, pdFileMap);
        }

        // read validation props
        if (GetPropsMode())
        {
            ReadValProps(reader.WS(), doc, pdFileMap, localFactors);
        }

        // read layers
        if (GetLayerMode())
        {
            ReadLayers(reader.WS(), doc);
        }

        // read SHUO entities from STEP model
        if (GetSHUOMode())
        {
            ReadSHUOs(reader.WS(), doc, pdFileMap);
        }

        // read GDT entities from STEP model
        if (GetGDTMode())
        {
            ReadGDTs(reader.WS(), doc, localFactors);
        }

        // read Material entities from STEP model
        if (GetMatMode())
        {
            ReadMaterials(reader.WS(), doc, seqPDS, localFactors);
        }

        // read View entities from STEP model
        if (GetViewMode())
        {
            ReadViews(reader.WS(), doc, localFactors);
        }

        // read metadata
        if (GetMetaMode())
        {
            ReadMetadata(reader.WS(), doc, localFactors);
        }

        // read product metadata
        if (GetProductMetaMode())
        {
            ReadProductMetadata(reader.WS(), doc);
        }

        // Expand resulting CAF structure for sub-shapes (optionally with their
        // names) if requested
        ExpandSubShapes(doc, shapePDMap);

        // Update assembly compounds
        doc.UpdateAssemblies();
        return true;
    }

    /// <summary>
    /// Add a shape to a document Depending on a case, this shape can be added as one, or as assembly, or (in case if
    /// it is associated with external reference) taken as that referred shape
    /// </summary>
    protected Label AddShape(Shape s, ExchangeDocument sTool, HashSet<Shape> newShapesMap,
                                    Dictionary<Shape, ProductDefinition> shapePDMap,
                                    Dictionary<ProductDefinition, ExternFile> pdFileMap)
    {
        // if shape has already been mapped, just return corresponding label
        if (_map.ContainsKey(s))
        {
            return _map.GetValue(s);
        }

        // if shape is located, create instance
        if (!s.Placement.IsIdentity)
        {
            Shape s0 = s.Located(new Placement());
            AddShape(s0, sTool, newShapesMap, shapePDMap, pdFileMap);
            Label l = sTool.AddShape(s, false); // should create reference
            _map[s] = l;
            return l;
        }

        // if shape is not compound, simple add it
        if (s.Type != ShapeType.Compound)
        {
            Label l = sTool.AddShape(s, false);
            _map[s] = l;
            return l;
        }

        // for compounds, compute number of subshapes and check whether this is assembly
        bool isAssembly = false;
        int nbComponents = 0;
        Iterator it = new Iterator();
        for (it.Initialize(s); it.More() && !isAssembly; it.Next(), nbComponents++)
        {
            Shape sub0 = it.Value().Located(new Placement());
            if (newShapesMap.Contains(sub0))
            {
                isAssembly = true;
            }
        }

        //  if(nbComponents>0) isAssembly = true;

        // check whether it has associated external ref
        NCollection_Sequence<string> shas = new NCollection_Sequence<string>();
        if (shapePDMap.ContainsKey(s) && pdFileMap.ContainsKey(shapePDMap.GetValue(s)))
        {
            ExternFile ef = pdFileMap.GetValue(shapePDMap.GetValue(s));
            if (ef is not null)
            {
                // (store information on extern refs in the document)
                shas.Append(ef.GetName().ToCString());
                // if yes, just return corresponding label
                if (!ef.GetLabel().IsNull)
                {
                    // but if components >0, ignore extern ref!
                    if (nbComponents <= 0)
                    {
                        _map[s] = ef.GetLabel();
                        sTool.SetExternRefs(ef.GetLabel(), shas);
                        return ef.GetLabel();
                    }
                }
                // OCCT_DEBUG output left out
            }
        }

        // add compound either as a whole,
        if (!isAssembly)
        {
            Label l = sTool.AddShape(s, false);
            if (shas.Length() > 0)
            {
                sTool.SetExternRefs(l, shas);
            }
            _map[s] = l;
            return l;
        }

        // or as assembly, component-by-component
        Label la = sTool.NewShape();
        nbComponents = 0;
        for (it.Initialize(s); it.More(); it.Next(), nbComponents++)
        {
            Shape sub0 = it.Value().Located(new Placement());
            Label subL = AddShape(sub0, sTool, newShapesMap, shapePDMap, pdFileMap);
            if (!subL.IsNull)
            {
                Label instL = sTool.AddComponent(la, subL, it.Value().Placement);
                if (!_map.ContainsKey(it.Value()))
                {
                    _map[it.Value()] = instL;
                }
            }
        }
        if (shas.Length() > 0)
        {
            sTool.SetExternRefs(la, shas);
        }
        _map[s] = la;
        // STool->SetShape ( L, S ); // it is necessary for assemblies OCC1747 // commemted by skl for
        // OCC2941

        return la;
    }

    /// <summary>Reads (or if returns already read) extern file with given name</summary>
    /// <remarks>
    /// Kerf: the external file is read by a session on the main session's parameter set (OCCT's statics are
    /// process-wide), through the reader's resolver (unit 8Z: OCCT reads <paramref name="fullname"/> from the disk),
    /// with the caller's limits and within the external files' budget; it is read under the name
    /// <paramref name="fullname"/>, against which its own references are resolved.
    /// </remarks>
    protected ExternFile ReadExternFile(string file, string fullname, ExchangeDocument doc,
                                                       ProgressRange progress = default)
    {
        return ReadExternFile(file, fullname, _reader.WS().LoadedFile() ?? "", doc, progress);
    }

    private ExternFile ReadExternFile(string file, string fullname, string referencingName,
                                                     ExchangeDocument doc, ProgressRange progress)
    {
        // if the file is already read, associate it with SDR
        if (_files.ContainsKey(file))
        {
            return _files.GetValue(file);
        }

        // create new WorkSession and Reader
        ExchangeWorkSession newWS = new ExchangeWorkSession(_reader.WS().Statics());
        CafController.SelectFor(newWS);
        StepReader sr = new StepReader(newWS, false);

        // start to fill the resulting ExternFile structure
        ExternFile ef = new ExternFile();
        ef.SetWS(newWS);
        ef.SetName(new TCollection_HAsciiString(file));

        // read file
        // Kerf: through the resolver, with the limits and progress (unit 8Z)
        using ProgressScope ps = new ProgressScope(progress, null, 2);
        ef.SetLoadStatus(ReadExternStream(sr, file, fullname, referencingName, ps.Next()));

        // transfer in single-result mode
        if (ef.GetLoadStatus() == ReturnStatus.RetDone)
        {
            NCollection_Sequence<Label> labels = new NCollection_Sequence<Label>();
            _externStack.Add(fullname);
            try
            {
                ef.SetTransferStatus(Transfer(sr, 0, doc, labels, false, ps.Next()));
            }
            finally
            {
                _externStack.RemoveAt(_externStack.Count - 1);
            }
            if (labels.Length() > 0)
            {
                ef.SetLabel(labels.Value(1));
            }
        }

        // add read file to dictionary
        _files[file] = ef;

        return ef;
    }

    // OCCT: sr.ReadFile(fullname); a file not opened is IFSelect_RetError as StepFile_Read's -1
    private ReturnStatus ReadExternStream(StepReader sr, string file, string fullname,
                                                   string referencingName, ProgressRange progress)
    {
        ExternFileResolver resolver = EffectiveResolver();
        if (resolver is null || progress.UserBreak())
        {
            return ReturnStatus.RetError;
        }
        Stream stream = CallResolver(resolver, file, referencingName);
        if (stream is null)
        {
            return ReturnStatus.RetError;
        }
        _externCount++;
        using (stream)
        {
            ParserReadLimits limits = _readLimits ?? ParserReadLimits.Default;
            long left = Math.Max(0, _externLimits.MaxTotalSize - _externBytes);
            limits = limits with { MaxFileSize = Math.Min(limits.MaxFileSize, left) };
            CountingStream counted = new CountingStream(stream, limits.MaxFileSize);
            try
            {
                return sr.ReadStream(fullname, counted, progress, limits);
            }
            finally
            {
                _externBytes += counted.BytesRead;
            }
        }
    }

    private static void FindStyledSR(StyledItem style, ref StepShapeRepresentation aSR)
    {
        // search Shape Representation for component styled item
        for (int j = 1; j <= style.StyleCount; j++)
        {
            PresentationStyleByContext psa = style.StylesValue(j) as PresentationStyleByContext;
            if (psa is null)
            {
                continue;
            }
            StyleContextSelect styleCntxSlct = psa.StyleContext();
            StepShapeRepresentation currentSR = styleCntxSlct?.Representation() as StepShapeRepresentation;
            if (currentSR is null)
            {
                continue;
            }
            aSR = currentSR;
            break;
        }
    }

    private static readonly ColorType[] ColorTypes = [ColorType.ColorGen, ColorType.ColorSurf, ColorType.ColorCurv];

    //=======================================================================
    // function : propagateColorToParts
    // purpose  : auxiliary, propagate color styles from assemblies to parts
    //=======================================================================
    private static void PropagateColorToParts(ExchangeDocument sTool, in Label root)
    {
        // collect components to propagate
        NCollection_Sequence<Label> components = new NCollection_Sequence<Label>();
        if (root.IsEqual(sTool.ShapesLabel()))
        {
            sTool.GetFreeShapes(components);
        }
        else
        {
            sTool.GetComponents(root, components);
        }

        // iterate each component
        foreach (Label it in components)
        {
            // get original label
            Label originalL = it;
            if (sTool.GetReferredShape(originalL, out Label referred))
            {
                originalL = referred;
            }

            // propagate to components without own colors
            foreach (ColorType type in ColorTypes)
            {
                if (sTool.GetColor(root, type, out Label colorL)
                    && !sTool.GetColor(originalL, type, out Label _))
                {
                    sTool.SetColor(originalL, colorL, type);
                }
            }
            if (!sTool.IsVisible(root))
            {
                sTool.SetVisibility(originalL, false);
            }

            // propagate to next level children
            if (sTool.IsAssembly(originalL))
            {
                PropagateColorToParts(sTool, originalL);
            }
        }
    }

    // the colours decoded as SetStyle and SetAssemblyComponentStyle decode them
    private static void DecodeColors(Colour aSurfCol, Colour aBoundCol, Colour aCurveCol,
                                     RenderingProperties aRenderProps, out ColorWithAlpha aFullSCol,
                                     out LinearColor aBCol, out LinearColor aCCol)
    {
        LinearColor sCol = new LinearColor();
        aBCol = new LinearColor();
        aCCol = new LinearColor();
        aFullSCol = new ColorWithAlpha();
        if (aSurfCol is not null)
        {
            Styles.DecodeColor(aSurfCol, ref sCol);
            aFullSCol = new ColorWithAlpha(sCol);
        }
        if (aBoundCol is not null)
        {
            Styles.DecodeColor(aBoundCol, ref aBCol);
        }
        if (aCurveCol is not null)
        {
            Styles.DecodeColor(aCurveCol, ref aCCol);
        }
        if (aRenderProps.IsDefined)
        {
            aFullSCol = aRenderProps.GetRGBAColor();
        }
    }

    //=======================================================================
    // function : SetAssemblyComponentStyle
    // purpose  : auxiliary: set override style for assembly components
    //=======================================================================
    private static void SetAssemblyComponentStyle(TransientProcess tp, ExchangeDocument cTool,
                                                  Styles styles,
                                                  ContextDependentOverRidingStyledItem style,
                                                  in Factors localFactors)
    {
        if (style is null)
        {
            return;
        }

        // check if it is component style
        bool isComponent = false;
        if (!styles.GetColors(style, out Colour surfCol, out Colour boundCol,
                                 out Colour curveCol, out RenderingProperties renderProps,
                                 ref isComponent))
        {
            return;
        }

        EntityGraph graph = tp.Graph();
        Placement loc = new Placement(); // init;
        // find shape
        Shape shape = new Shape();
        Binder binder;
        StepShapeRepresentation repr = style.ItemAP242()?.Value() as StepShapeRepresentation;
        if (repr is null)
        {
            return;
        }
        ShapeRepresentationRelationship srr = null;
        EntityIterator subs = tp.Graph().Sharings(repr);
        for (subs.Start(); subs.More(); subs.Next())
        {
            Transient subsVal = subs.Value();
            if (subsVal.IsKind(typeof(ShapeRepresentationRelationship)))
            {
                srr = subsVal as ShapeRepresentationRelationship;
            }
        }

        binder = tp.Find(srr);
        if (binder is not null)
        {
            shape = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
        }
        if (shape.IsNull)
        {
            return;
        }

        // get transformation
        subs = graph.Shareds(style);
        subs.Start();
        for (; subs.More(); subs.Next())
        {
            ShapeRepresentationRelationshipWithTransformation relation =
                subs.Value() as ShapeRepresentationRelationshipWithTransformation;
            if (relation is null)
            {
                continue;
            }

            Transformation transf = relation.TransformationOperator();
            ItemDefinedTransformation itemTransf = transf?.ItemDefinedTransformation();
            if (itemTransf is not null)
            {
                StepAxis2Placement3d axp1 = itemTransf.TransformItem1() as StepAxis2Placement3d;
                StepAxis2Placement3d axp2 = itemTransf.TransformItem2() as StepAxis2Placement3d;

                if (axp1 is not null && axp2 is not null)
                {
                    Axis2Placement orig = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(axp1, localFactors);
                    Axis2Placement targ = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeAxis2Placement(axp2, localFactors);
                    CoordinateSystem ax3Orig = new CoordinateSystem(orig.Ax2());
                    CoordinateSystem ax3Targ = new CoordinateSystem(targ.Ax2());

                    Transform3D tr1 = new Transform3D().WithTransformation(ax3Targ, ax3Orig);
                    Placement loc1 = new Placement(tr1);
                    loc = loc.Multiplied(loc1);
                }
            }
        }

        shape = shape.Located(loc);

        if (surfCol is not null || boundCol is not null || curveCol is not null || renderProps.IsDefined)
        {
            DecodeColors(surfCol, boundCol, curveCol, renderProps, out ColorWithAlpha fullSCol,
                         out LinearColor bCol, out LinearColor cCol);

            if (surfCol is not null || renderProps.IsDefined)
            {
                cTool.SetInstanceColor(shape, ColorType.ColorSurf, fullSCol);
            }
            if (boundCol is not null)
            {
                cTool.SetInstanceColor(shape, ColorType.ColorCurv, bCol);
            }
            if (curveCol is not null)
            {
                cTool.SetInstanceColor(shape, ColorType.ColorCurv, cCol);
            }
        }
    }

    //=======================================================================
    // function : SetStyle
    // purpose  : auxiliary: set style for parts and instances
    //=======================================================================
    private static void SetStyle(ExchangeWorkSession ws, Dictionary<Shape, Label> map,
                                 ExchangeDocument sTool, Styles styles,
                                 NCollection_HSequence<Transient> hSeqOfInvisStyle,
                                 StyledItem style, in Factors localFactors)
    {
        if (style is null)
        {
            return;
        }

        TransientProcess tp = ws.TransferReader().TransientProcess();
        if (style is OverRidingStyledItem overridingStyle)
        {
            SetStyle(ws, map, sTool, styles, hSeqOfInvisStyle, overridingStyle.OverRiddenStyle(),
                     localFactors);
            if (style is ContextDependentOverRidingStyledItem assemblyComponentStyle)
            {
                SetAssemblyComponentStyle(tp, sTool, styles, assemblyComponentStyle, localFactors);
                return;
            }
        }

        bool isVisible = true;
        // check the visibility of styled item.
        for (int si = 1; si <= hSeqOfInvisStyle.Length(); si++)
        {
            if (style != hSeqOfInvisStyle.Value(si))
            {
                continue;
            }
            // found that current style is invisible.
            isVisible = false;
            break;
        }

        // check if it is component style
        bool isComponent = false;
        if (!styles.GetColors(style, out Colour surfCol, out Colour boundCol,
                                 out Colour curveCol, out RenderingProperties renderProps,
                                 ref isComponent)
            && isVisible)
        {
            return;
        }

        // collect styled items
        List<StyledItemTarget> items = new List<StyledItemTarget>();
        if (style.ItemAP242() is not null && !style.ItemAP242().IsNull)
        {
            items.Add(style.ItemAP242());
        }

        for (int itemIt = 0; itemIt < items.Count; itemIt++)
        {
            int index = tp.MapIndex(items[itemIt].Value());
            Shape s = new Shape();
            if (index > 0)
            {
                Binder binder = tp.MapItem(index);
                s = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
            }
            bool isSkipSHUOstyle = false;
            // take shape with real location.
            while (isComponent)
            {
                // take SR of NAUO
                StepShapeRepresentation sr = null;
                FindStyledSR(style, ref sr);
                // search for SR along model
                if (sr is null)
                {
                    break;
                }
                EntityIterator subs = ws.HGraph().Graph().Sharings(sr);
                StepShapeDefinitionRepresentation sdr;
                for (subs.Start(); subs.More(); subs.Next())
                {
                    sdr = subs.Value() as StepShapeDefinitionRepresentation;
                    if (sdr is null)
                    {
                        continue;
                    }
                    RepresentedDefinition pdSselect = sdr.Definition();
                    ProductDefinitionShape pds = pdSselect.PropertyDefinition() as ProductDefinitionShape;
                    if (pds is null)
                    {
                        continue;
                    }
                    CharacterizedDefinition charDef = pds.Definition();

                    AssemblyComponentUsage acu = charDef.ProductDefinitionRelationship() as AssemblyComponentUsage;
                    if (acu is null)
                    {
                        continue;
                    }
                    // PTV 10.02.2003 skip styled item that refer to SHUO
                    if (acu.IsKind(typeof(SpecifiedHigherUsageOccurrence)))
                    {
                        isSkipSHUOstyle = true;
                        break;
                    }
                    NextAssemblyUsageOccurrence nauo = acu as NextAssemblyUsageOccurrence;
                    if (nauo is null)
                    {
                        continue;
                    }

                    // PTV 10.02.2003 to find component of assembly CORRECTLY
                    ConstructTool tool = new ConstructTool(ws);
                    Label shLab = FindInstance(nauo, sTool, tool, map);
                    Shape sh = shLab.IsNull ? new Shape() : sTool.GetShape(shLab);
                    if (!sh.IsNull)
                    {
                        s = sh;
                        break;
                    }
                }
                break;
            }
            if (isSkipSHUOstyle)
            {
                continue; // skip styled item which refer to SHUO
            }

            if (s.IsNull)
            {
                continue;
            }

            if (surfCol is not null || boundCol is not null || curveCol is not null || renderProps.IsDefined
                || !isVisible)
            {
                bool isFound = sTool.SearchUsingMap(s, out Label l, false, true);
                if (surfCol is not null || boundCol is not null || curveCol is not null || renderProps.IsDefined)
                {
                    DecodeColors(surfCol, boundCol, curveCol, renderProps, out ColorWithAlpha fullSCol,
                                 out LinearColor bCol, out LinearColor cCol);
                    if (isFound)
                    {
                        if (surfCol is not null || renderProps.IsDefined)
                        {
                            sTool.SetColor(l, fullSCol, ColorType.ColorSurf);
                        }
                        if (boundCol is not null)
                        {
                            sTool.SetColor(l, bCol, ColorType.ColorCurv);
                        }
                        if (curveCol is not null)
                        {
                            sTool.SetColor(l, cCol, ColorType.ColorCurv);
                        }
                    }
                    else
                    {
                        for (Iterator it = new Iterator(s); it.More(); it.Next())
                        {
                            if (sTool.SearchUsingMap(it.Value(), out Label l1, false, true))
                            {
                                if (surfCol is not null || renderProps.IsDefined)
                                {
                                    sTool.SetColor(l1, fullSCol, ColorType.ColorSurf);
                                }
                                if (boundCol is not null)
                                {
                                    sTool.SetColor(l1, bCol, ColorType.ColorCurv);
                                }
                                if (curveCol is not null)
                                {
                                    sTool.SetColor(l1, cCol, ColorType.ColorCurv);
                                }
                            }
                        }
                    }
                }
                if (!isVisible)
                {
                    // sets the invisibility for shape.
                    if (isFound)
                    {
                        sTool.SetVisibility(l, false);
                    }
                }
            }
        }
    }

    //=======================================================================
    // function : IsOverriden
    // purpose  : auxiliary: check that style is overridden
    //=======================================================================
    private static bool IsOverriden(EntityGraph graph, StyledItem style, bool isRoot)
    {
        EntityIterator subs = graph.Sharings(style);
        subs.Start();
        for (; subs.More(); subs.Next())
        {
            OverRidingStyledItem overRidingStyle = subs.Value() as OverRidingStyledItem;
            if (overRidingStyle is not null)
            {
                if (!isRoot)
                {
                    return true;
                }
                // for root style returns true only if it is overridden by other root style
                Transient item = overRidingStyle.ItemAP242()?.Value();
                if (item is not null && item.IsKind(typeof(StepShapeRepresentation)))
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
    protected bool ReadColors(ExchangeWorkSession ws, ExchangeDocument doc, in Factors localFactors)
    {
        Styles styles = new Styles(ws);
        if (!styles.LoadStyles())
        {
            // OCCT_DEBUG output left out
            return false;
        }
        // searching for invisible items in the model
        NCollection_HSequence<Transient> hSeqOfInvisStyle = new NCollection_HSequence<Transient>();
        styles.LoadInvisStyles(ref hSeqOfInvisStyle);

        EntityGraph graph = styles.Graph();

        // parse and search for color attributes
        int nb = styles.RootStyleCount;
        // apply root styles earlier, as they can be overridden
        // function IsOverriden for root style returns true only if it is overridden by other root style
        bool isRootStyle = true;
        for (int i = 1; i <= nb; i++)
        {
            StyledItem style = styles.RootStyle(i);
            // check that style is overridden by other root style
            if (!IsOverriden(graph, style, isRootStyle))
            {
                SetStyle(ws, _map, doc, styles, hSeqOfInvisStyle, style, localFactors);
            }
        }

        nb = styles.StyleCount;
        // apply leaf styles, they can override root styles
        isRootStyle = false;
        for (int i = 1; i <= nb; i++)
        {
            StyledItem style = styles.Style(i);
            // check that style is overridden
            if (!IsOverriden(graph, style, isRootStyle))
            {
                SetStyle(ws, _map, doc, styles, hSeqOfInvisStyle, style, localFactors);
            }
        }

        // OCCT: CTool->ReverseChainsOfTreeNodes(), the order of the shapes linked to a colour label, which the
        // document doesn't keep (decision 1278)

        // some colors can be attached to assemblies, propagate them to components
        PropagateColorToParts(doc, doc.ShapesLabel());
        return true;
    }

    private static Label GetLabelFromPD(ProductDefinition pd, ExchangeDocument sTool,
                                               TransientProcess tp,
                                               Dictionary<ProductDefinition, ExternFile> pdFileMap,
                                               Dictionary<Shape, Label> shapeLabelMap)
    {
        Label l = default;
        if (pdFileMap.ContainsKey(pd))
        {
            ExternFile ef = pdFileMap.GetValue(pd);
            if (ef is not null)
            {
                l = ef.GetLabel();
                if (!l.IsNull)
                {
                    return l;
                }
            }
        }

        Shape s;
        Binder binder = tp.Find(pd);
        if (binder is null || !binder.HasResult)
        {
            return l;
        }
        s = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(tp, binder);
        if (s.IsNull)
        {
            return l;
        }

        if (shapeLabelMap.ContainsKey(s))
        {
            l = shapeLabelMap.GetValue(s);
        }
        if (l.IsNull)
        {
            sTool.Search(s, out l, true, true, false);
        }
        return l;
    }

    /// <summary>
    /// Find a label of the instance of a component (NAUO) in the document, through the shape it was read into.
    /// </summary>
    public static Label FindInstance(NextAssemblyUsageOccurrence nauo, ExchangeDocument sTool,
                                            ConstructTool tool, Dictionary<Shape, Label> shapeLabelMap)
    {
        Label l = default;

        // get shape resulting from CDSR (in fact, only location is interesting)
        TransientProcess tp = tool.TransientProcess();
        Binder binder = tp.Find(nauo);
        if (binder is null || !binder.HasResult)
        {
            // OCCT_DEBUG output left out
            return l;
        }

        Shape s = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(tp, binder);
        if (s.IsNull)
        {
            // OCCT_DEBUG output left out
            return l;
        }

        if (shapeLabelMap.ContainsKey(s))
        {
            l = shapeLabelMap.GetValue(s);
        }
        else
        {
            sTool.Search(s, out l, true, true, false);
        }

        return l;
    }

    /// <summary>
    /// Reads names of parts defined in the STEP model and assigns them to corresponding labels in the DECAF document
    /// </summary>
    protected bool ReadNames(ExchangeWorkSession ws, ExchangeDocument doc,
                             Dictionary<ProductDefinition, ExternFile> pdFileMap)
    {
        // get starting data
        InterfaceModel model = ws.Model();
        TransferReader tr = ws.TransferReader();
        TransientProcess tp = tr.TransientProcess();
        ConstructTool tool = new ConstructTool(ws);

        // iterate on model to find all SDRs and CDSRs
        int nb = model.EntityCount;
        Type tNAUO = typeof(NextAssemblyUsageOccurrence);
        Type tPD = typeof(ProductDefinition);
        Type tPDWAD = typeof(ProductDefinitionWithAssociatedDocuments);
        TCollection_HAsciiString name = null;
        Label l;
        EntityGraph graph = null;
        for (int i = 1; i <= nb; i++)
        {
            Transient enti = model.Value(i);

            // get description of NAUO
            if (enti.DynamicType() == tNAUO)
            {
                l = default;
                NextAssemblyUsageOccurrence nauo = enti as NextAssemblyUsageOccurrence;
                if (nauo is null)
                {
                    continue;
                }
                // OCCT: WS->Graph(), which recomputes the graph only if the model changed (it doesn't here)
                graph ??= ws.Graph();
                EntityIterator subs = graph.Sharings(nauo);
                for (subs.Start(); subs.More(); subs.Next())
                {
                    ProductDefinitionShape pds = subs.Value() as ProductDefinitionShape;
                    if (pds is null)
                    {
                        continue;
                    }
                    ProductDefinitionRelationship pdr = pds.Definition().ProductDefinitionRelationship();
                    if (pdr is null)
                    {
                        continue;
                    }
                    if (pdr.HasDescription && pdr.Description().UsefullLength() > 0)
                    {
                        name = pdr.Description();
                    }
                    else if (pdr.Name() is not null && pdr.Name().UsefullLength() > 0)
                    {
                        name = pdr.Name();
                    }
                    else if (pdr.Id() is not null)
                    {
                        name = pdr.Id();
                    }
                    else
                    {
                        name = new TCollection_HAsciiString();
                    }
                }
                // find proper label
                l = FindInstance(nauo, doc, tool, _map);
                if (l.IsNull)
                {
                    continue;
                }

                // OCCT: a NAUO no PDS refers to keeps the name found before (null at the first: OCCT would crash)
                string str = ConvertName(name is null ? "" : name.ToCString());
                doc.SetName(l, str);
            }

            // for PD get name of associated product
            if (enti.DynamicType() == tPD || enti.DynamicType() == tPDWAD)
            {
                l = default;
                ProductDefinition pd = enti as ProductDefinition;
                if (pd is null)
                {
                    continue;
                }
                Product prod = pd.Formation() is not null ? pd.Formation().OfProduct() : null;
                if (prod is null)
                {
                    name = new TCollection_HAsciiString();
                }
                else if (prod.Name() is not null && prod.Name().UsefullLength() > 0)
                {
                    name = prod.Name();
                }
                else if (prod.Id() is not null)
                {
                    name = prod.Id();
                }
                else
                {
                    name = new TCollection_HAsciiString();
                }
                l = GetLabelFromPD(pd, doc, tp, pdFileMap, _map);
                if (l.IsNull)
                {
                    continue;
                }
                string str = ConvertName(name.ToCString());
                doc.SetName(l, str);
            }
            // set a name to the document
            // TCollection_ExtendedString str = convertName (name->String());
            // TDataStd_Name::Set ( L, str );
        }

        return true;
    }

    private static Label GetLabelFromPD(ProductDefinition pd, ExchangeDocument sTool,
                                               ValidationProps props,
                                               Dictionary<ProductDefinition, ExternFile> pdFileMap,
                                               Dictionary<Shape, Label> shapeLabelMap)
    {
        Label l = default;
        if (pdFileMap.ContainsKey(pd))
        {
            ExternFile ef = pdFileMap.GetValue(pd);
            if (ef is not null)
            {
                l = ef.GetLabel();
                if (!l.IsNull)
                {
                    return l;
                }
            }
        }
        Shape s = props.GetPropShape(pd);
        if (s.IsNull)
        {
            return l;
        }
        if (shapeLabelMap.ContainsKey(s))
        {
            l = shapeLabelMap.GetValue(s);
        }
        if (l.IsNull)
        {
            sTool.Search(s, out l, true, true, false);
        }
        return l;
    }

    /// <summary>
    /// Reads validation properties assigned to shapes in the STEP model and assigns them to corresponding labels in
    /// the DECAF document
    /// </summary>
    protected bool ReadValProps(ExchangeWorkSession ws, ExchangeDocument doc,
                                Dictionary<ProductDefinition, ExternFile> pdFileMap,
                                in Factors localFactors)
    {
        // get starting data
        TransferReader tr = ws.TransferReader();
        TransientProcess tp = tr.TransientProcess();

        // load props from the STEP model
        NCollection_Sequence<Transient> props = new NCollection_Sequence<Transient>();
        ValidationProps Props = new ValidationProps(ws);
        if (!Props.LoadProps(props))
        {
            // OCCT_DEBUG output left out
            return false;
        }

        // interpret props one by one
        for (int i = 1; i <= props.Length(); i++)
        {
            PropertyDefinitionRepresentation pdr = props.Value(i) as PropertyDefinitionRepresentation;
            if (pdr is null)
            {
                continue;
            }

            Label l = default;

            PropertyDefinition pd = pdr.Definition().PropertyDefinition();
            EntityIterator subs = Props.Graph().Shareds(pd);
            for (subs.Start(); l.IsNull && subs.More(); subs.Next())
            {
                if (subs.Value().IsKind(typeof(ProductDefinitionShape)))
                {
                    ProductDefinitionShape pds = subs.Value() as ProductDefinitionShape;
                    if (pds is null)
                    {
                        continue;
                    }
                    // find corresponding NAUO
                    NextAssemblyUsageOccurrence nauo = null;
                    EntityIterator subs1 = Props.Graph().Shareds(pds);
                    for (subs1.Start(); nauo is null && subs1.More(); subs1.Next())
                    {
                        if (subs1.Value().IsKind(typeof(NextAssemblyUsageOccurrence)))
                        {
                            nauo = subs1.Value() as NextAssemblyUsageOccurrence;
                        }
                    }
                    if (nauo is not null)
                    {
                        l = FindInstance(nauo, doc, new ConstructTool(ws), _map);
                        if (l.IsNull)
                        {
                            continue;
                        }
                    }
                    else
                    {
                        // find corresponding ProductDefinition:
                        ProductDefinition prodDef = null;
                        EntityIterator subsPDS = Props.Graph().Shareds(pds);
                        for (subsPDS.Start(); prodDef is null && subsPDS.More(); subsPDS.Next())
                        {
                            if (subsPDS.Value().IsKind(typeof(ProductDefinition)))
                            {
                                prodDef = subsPDS.Value() as ProductDefinition;
                            }
                        }
                        if (prodDef is null)
                        {
                            continue;
                        }
                        l = GetLabelFromPD(prodDef, doc, Props, pdFileMap, _map);
                    }
                }

                if (subs.Value().IsKind(typeof(ShapeAspect)))
                {
                    ShapeAspect sa = subs.Value() as ShapeAspect;
                    if (sa is null)
                    {
                        continue;
                    }
                    // find ShapeRepresentation
                    StepShapeRepresentation sr = null;
                    EntityIterator subs1 = Props.Graph().Sharings(sa);
                    for (subs1.Start(); subs1.More() && sr is null; subs1.Next())
                    {
                        PropertyDefinition propD1 = subs1.Value() as PropertyDefinition;
                        if (propD1 is null)
                        {
                            continue;
                        }
                        EntityIterator subs2 = Props.Graph().Sharings(propD1);
                        for (subs2.Start(); subs2.More() && sr is null; subs2.Next())
                        {
                            StepShapeDefinitionRepresentation sdr = subs2.Value() as StepShapeDefinitionRepresentation;
                            if (sdr is null)
                            {
                                continue;
                            }
                            sr = sdr.UsedRepresentation() as StepShapeRepresentation;
                        }
                    }
                    if (sr is null)
                    {
                        continue;
                    }
                    Binder binder = null;
                    for (int ir = 1; ir <= sr.NbItems() && binder is null; ir++)
                    {
                        if (sr.ItemsValue(ir).IsKind(typeof(StepSolidModel)))
                        {
                            binder = tp.Find(sr.ItemsValue(ir));
                        }
                        else if (sr.ItemsValue(ir).IsKind(typeof(StepShellBasedSurfaceModel)))
                        {
                            binder = tp.Find(sr.ItemsValue(ir));
                        }
                        else if (sr.ItemsValue(ir).IsKind(typeof(StepGeometricSet)))
                        {
                            binder = tp.Find(sr.ItemsValue(ir));
                        }
                    }
                    if (binder is null || !binder.HasResult)
                    {
                        continue;
                    }
                    Shape s = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(tp, binder);
                    if (s.IsNull)
                    {
                        continue;
                    }
                    if (_map.ContainsKey(s))
                    {
                        l = _map.GetValue(s);
                    }
                    if (l.IsNull)
                    {
                        doc.Search(s, out l, true, true, true);
                    }
                }
            }

            if (l.IsNull)
            {
                continue;
            }

            // decode validation properties
            StepRepresentation rep = pdr.UsedRepresentation();
            for (int j = 1; j <= rep.NbItems(); j++)
            {
                RepresentationItem ent = rep.ItemsValue(j);
                if (Props.GetPropReal(ent, out double val, out bool isArea, localFactors))
                {
                    if (isArea)
                    {
                        doc.SetArea(l, val);
                    }
                    else
                    {
                        doc.SetVolume(l, val);
                    }
                }
                else if (Props.GetPropPnt(ent, rep.ContextOfItems(), out Point3D pos, localFactors))
                {
                    doc.SetCentroid(l, pos);
                }
            }
        }
        return true;
    }

    /// <summary>
    /// Reads layers of parts defined in the STEP model and set reference between shape and layers in the DECAF
    /// document
    /// </summary>
    protected bool ReadLayers(ExchangeWorkSession ws, ExchangeDocument doc)
    {
        InterfaceModel model = ws.Model();
        TransferReader tr = ws.TransferReader();
        TransientProcess tp = tr.TransientProcess();

        Type tSVPLA = typeof(PresentationLayerAssignment);
        int nb = model.EntityCount;
        EntityGraph graph = null;

        for (int i = 1; i <= nb; i++)
        {
            Transient enti = model.Value(i);
            if (!enti.IsKind(tSVPLA))
            {
                continue;
            }
            PresentationLayerAssignment svpla = enti as PresentationLayerAssignment;
            if (svpla.AssignedItems() is null)
            {
                continue;
            }

            TCollection_HAsciiString hName = svpla.Name();
            string layerName = hName.ToCString();
            Label layerLabel = default;

            // check invisibility
            bool isVisible = true;
            graph ??= ws.Graph();
            EntityIterator subs = graph.Sharings(svpla);
            for (subs.Start(); subs.More() && isVisible; subs.Next())
            {
                if (!subs.Value().IsKind(typeof(Invisibility)))
                {
                    continue;
                }
                isVisible = false;
            }

            // find a target shape and its label in the document
            for (int j = 1; j <= svpla.AssignedItemCount; j++)
            {
                LayeredItem li = svpla.AssignedItemsValue(j);
                Binder binder = li is null ? null : tp.Find(li.Value());
                if (binder is null || !binder.HasResult)
                {
                    continue;
                }

                Shape s = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(tp, binder);
                if (s.IsNull)
                {
                    continue;
                }

                if (!doc.Search(s, out Label shL, true, true, true))
                {
                    continue;
                }
                if (layerLabel.IsNull)
                {
                    layerLabel = doc.AddLayer(layerName, isVisible);
                }
                doc.SetLayer(shL, layerLabel);
            }

            if (!layerLabel.IsNull)
            {
                doc.SetVisibility(layerLabel, isVisible);
            }
        }
        return true;
    }

    /// <summary>Reads SHUO for instances defined in the STEP model</summary>
    /// <remarks>Kerf: not read (the document has no SHUOs, decision 1279).</remarks>
    protected bool ReadSHUOs(ExchangeWorkSession ws, ExchangeDocument doc,
                             Dictionary<ProductDefinition, ExternFile> pdFileMap)
    {
        _ = ws;
        _ = doc;
        _ = pdFileMap;
        return false;
    }

    /// <summary>Reads D&amp;GT for instances defined in the STEP model</summary>
    /// <remarks>Kerf: not read (decision 1279).</remarks>
    protected bool ReadGDTs(ExchangeWorkSession ws, ExchangeDocument doc, in Factors localFactors)
    {
        _ = ws;
        _ = doc;
        return false;
    }

    /// <summary>Reads materials for instances defined in the STEP model</summary>
    /// <remarks>Kerf: not read (decision 1279).</remarks>
    protected bool ReadMaterials(ExchangeWorkSession ws, ExchangeDocument doc, NCollection_HSequence<Transient> seqPDS,
                                 in Factors localFactors)
    {
        _ = ws;
        _ = doc;
        _ = seqPDS;
        return false;
    }

    /// <summary>Reads Views for instances defined in the STEP model</summary>
    /// <remarks>Kerf: not read (decision 1279).</remarks>
    protected bool ReadViews(ExchangeWorkSession ws, ExchangeDocument doc, in Factors localFactors)
    {
        _ = ws;
        _ = doc;
        return false;
    }

    /// <summary>Reads metadata assigned to shapes in the STEP model</summary>
    /// <remarks>Kerf: not read (decision 1279).</remarks>
    protected bool ReadMetadata(ExchangeWorkSession ws, ExchangeDocument doc, in Factors localFactors)
    {
        _ = ws;
        _ = doc;
        return false;
    }

    /// <summary>Reads product metadata assigned to shapes in the STEP model</summary>
    /// <remarks>Kerf: not read (decision 1279).</remarks>
    protected bool ReadProductMetadata(ExchangeWorkSession ws, ExchangeDocument doc)
    {
        _ = ws;
        _ = doc;
        return false;
    }

    /// <summary>
    /// Populates the sub-Label of the passed TDF Label with shape data associated with the given STEP Representation
    /// Item, including naming and topological information.
    /// </summary>
    public Label SettleShapeData(RepresentationItem item, in Label lab,
                                        ExchangeDocument shapeTool, TransientProcess tp)
    {
        Label result = lab;
        if (item is null)
        {
            return result;
        }

        TCollection_HAsciiString hName = item.Name();
        if (hName is null || hName.IsEmpty())
        {
            return result;
        }

        Binder binder = tp.Find(item);
        if (binder is null)
        {
            return result;
        }

        Shape shape = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
        if (shape.IsNull)
        {
            return result;
        }

        // Allocate sub-Label
        result = shapeTool.AddSubShape(lab, shape);
        if (result.IsNull)
        {
            return result;
        }

        string name = hName.ToCString();
        shapeTool.SetName(result, name);
        shapeTool.SetShape(result, shape);

        return result;
    }

    //=======================================================================
    // function : collectRepresentationItems
    // purpose  : recursive collection of representation items for given representation
    //           with all representations, related to it.
    //=======================================================================
    private static void CollectRepresentationItems(EntityGraph graph, StepShapeRepresentation representation,
                                                   NCollection_Sequence<RepresentationItem> items, int depth = 0)
    {
        NCollection_HArray1<RepresentationItem> anItems = representation.Items();
        if (anItems is not null)
        {
            for (int i = anItems.Lower(); i <= anItems.Upper(); i++)
            {
                RepresentationItem reprItem = anItems.Value(i);
                if (reprItem is null)
                {
                    continue;
                }
                items.Append(reprItem);
            }
        }

        // Kerf: a cycle of representation relationships recurses without end in OCCT; stopped (decision 1280)
        if (depth > 1000)
        {
            return;
        }
        EntityIterator entIt = graph.TypedSharings(representation, typeof(RepresentationRelationship));
        for (entIt.Start(); entIt.More(); entIt.Next())
        {
            RepresentationRelationship relationship = entIt.Value() as RepresentationRelationship;
            if (relationship.Rep1() == representation)
            {
                StepShapeRepresentation repr = relationship.Rep2() as StepShapeRepresentation;
                if (repr is not null)
                {
                    CollectRepresentationItems(graph, repr, items, depth + 1);
                }
            }
        }
    }

    /// <summary>
    /// Expands the topological structure of Manifold Solid BRep STEP entity to OCAF sub-tree. Entities having no
    /// names associated via their StepRepr_RepresentationItem base class are skipped.
    /// </summary>
    protected void ExpandSubShapes(ExchangeDocument shapeTool, Dictionary<Shape, ProductDefinition> shapePDMap)
    {
        TransientProcess tp = Reader().WS().TransferReader().TransientProcess();
        StepModel stepModel = tp.Model() as StepModel;
        if (!stepModel.InternalParameters.ReadSubshapeNames)
        {
            return;
        }

        EntityGraph graph = Reader().WS().Graph();

        foreach (KeyValuePair<Shape, ProductDefinition> it in shapePDMap)
        {
            Shape rootShape = it.Key;
            ProductDefinition pDef = it.Value;
            if (pDef is null)
            {
                continue;
            }

            // Find SDR by Product
            StepShapeDefinitionRepresentation sdr = null;
            EntityIterator entIt = graph.TypedSharings(pDef, typeof(StepShapeDefinitionRepresentation));
            for (entIt.Start(); entIt.More(); entIt.Next())
            {
                Transient referer = entIt.Value();
                sdr = referer as StepShapeDefinitionRepresentation;
                if (sdr is not null)
                {
                    break;
                }
            }

            if (sdr is null)
            {
                continue;
            }

            // Access shape representation
            StepShapeRepresentation shapeRepr = sdr.UsedRepresentation() as StepShapeRepresentation;

            if (shapeRepr is null)
            {
                continue;
            }

            // Access representation items
            NCollection_Sequence<RepresentationItem> reprItems = new NCollection_Sequence<RepresentationItem>();
            CollectRepresentationItems(graph, shapeRepr, reprItems);

            if (reprItems.Length() == 0)
            {
                continue;
            }

            if (!_map.ContainsKey(rootShape))
            {
                continue;
            }

            Label rootLab = _map.GetValue(rootShape);
            // Do not add subshapes to assembly,
            // they will be processed with corresponding Shape_Product_Definition of necessary part.
            if (shapeTool.IsAssembly(rootLab))
            {
                continue;
            }

            NCollection_Sequence<RepresentationItem> msbSeq = new NCollection_Sequence<RepresentationItem>();
            NCollection_Sequence<RepresentationItem> sbsmSeq = new NCollection_Sequence<RepresentationItem>();

            // Iterate over the top level representation items collecting the
            // topological containers to expand
            for (int i = 1; i <= reprItems.Length(); ++i)
            {
                RepresentationItem tRepr = reprItems.Value(i);
                if (tRepr.IsKind(typeof(StepManifoldSolidBrep)))
                {
                    msbSeq.Append(tRepr);
                }
                else if (tRepr.IsKind(typeof(StepShellBasedSurfaceModel)))
                {
                    sbsmSeq.Append(tRepr);
                }
            }

            // Insert intermediate OCAF Labels for SOLIDs in case there are more
            // than one Manifold Solid BRep in the Shape Representation
            bool doInsertSolidLab = msbSeq.Length() > 1;

            // Expand Manifold Solid BReps
            for (int i = 1; i <= msbSeq.Length(); ++i)
            {
                // Put additional Label for SOLID
                if (doInsertSolidLab)
                {
                    SettleShapeData(msbSeq.Value(i), rootLab, shapeTool, tp);
                }

                ExpandManifoldSolidBrep(rootLab, msbSeq.Value(i), tp, shapeTool);
            }

            // Expand Shell-Based Surface Models
            for (int i = 1; i <= sbsmSeq.Length(); ++i)
            {
                ExpandSBSM(rootLab, sbsmSeq.Value(i), tp, shapeTool);
            }
        }
    }

    /// <summary>
    /// Expands the topological structure of Manifold Solid BRep STEP entity to OCAF sub-tree. Entities having no
    /// names associated via their StepRepr_RepresentationItem base class are skipped.
    /// </summary>
    protected void ExpandManifoldSolidBrep(Label shapeLab, RepresentationItem repr,
                                           TransientProcess tp, ExchangeDocument shapeTool)
    {
        // Access outer shell
        StepManifoldSolidBrep msb = repr as StepManifoldSolidBrep;
        StepConnectedFaceSet shell = msb.Outer();

        // Expand shell contents to CAF tree
        ExpandShell(shell, shapeLab, tp, shapeTool);
    }

    /// <summary>
    /// Expands the topological structure of Shell-Based Surface Model STEP entity to OCAF sub-tree. Entities having
    /// no names associated via their StepRepr_RepresentationItem base class are skipped.
    /// </summary>
    protected void ExpandSBSM(Label shapeLab, RepresentationItem repr, TransientProcess tp,
                              ExchangeDocument shapeTool)
    {
        StepShellBasedSurfaceModel sbsm = repr as StepShellBasedSurfaceModel;

        // Access boundary shells
        NCollection_HArray1<StepShell> shells = sbsm.SbsmBoundary();
        for (int s = shells.Lower(); s <= shells.Upper(); ++s)
        {
            StepShell shell = shells.Value(s);
            StepConnectedFaceSet cfs;
            StepOpenShell openShell = shell?.OpenShell();
            StepClosedShell closedShell = shell?.ClosedShell();

            if (openShell is not null)
            {
                cfs = openShell;
            }
            else
            {
                cfs = closedShell;
            }

            ExpandShell(cfs, shapeLab, tp, shapeTool);
        }
    }

    /// <summary>
    /// Expands STEP Shell structure to OCAF sub-tree. Entities having no names associated via their
    /// StepRepr_RepresentationItem base class are skipped.
    /// </summary>
    protected void ExpandShell(StepConnectedFaceSet shell, Label rootLab, TransientProcess tp,
                               ExchangeDocument shapeTool)
    {
        // Record CAF data
        SettleShapeData(shell, rootLab, shapeTool, tp);

        // Access faces
        NCollection_HArray1<StepFace> faces = shell.CfsFaces();
        for (int f = faces.Lower(); f <= faces.Upper(); ++f)
        {
            StepFace face = faces.Value(f);
            if (face is null)
            {
                continue;
            }

            // Record CAF data
            SettleShapeData(face, rootLab, shapeTool, tp);

            // Access face bounds
            NCollection_HArray1<StepFaceBound> wires = face.Bounds();
            if (wires is null)
            {
                continue;
            }
            for (int w = wires.Lower(); w <= wires.Upper(); ++w)
            {
                StepLoop wire = wires.Value(w).Bound();

                // Record CAF data
                SettleShapeData(wire, rootLab, shapeTool, tp);

                // Access wire edges
                // Currently only EDGE LOOPs are considered (!)
                if (!wire.IsInstance(typeof(StepEdgeLoop)))
                {
                    continue;
                }

                // Access edges
                StepEdgeLoop edgeLoop = wire as StepEdgeLoop;
                NCollection_HArray1<StepOrientedEdge> edges = edgeLoop.EdgeList();
                for (int e = edges.Lower(); e <= edges.Upper(); ++e)
                {
                    StepOrientedEdge orientedEdge = edges.Value(e);
                    if (orientedEdge is null)
                    {
                        continue;
                    }
                    StepEdge edge = orientedEdge.EdgeElement();
                    if (edge is null)
                    {
                        continue;
                    }

                    // Record CAF data
                    SettleShapeData(edge, rootLab, shapeTool, tp);

                    // Access vertices
                    StepVertex v1 = edge.EdgeStart();
                    StepVertex v2 = edge.EdgeEnd();

                    // Record CAF data
                    SettleShapeData(v1, rootLab, shapeTool, tp);
                    SettleShapeData(v2, rootLab, shapeTool, tp);
                }
            }
        }
    }

    /// <summary>Set ColorMode for indicate read Colors or not.</summary>
    public void SetColorMode(bool colormode) => _colorMode = colormode;

    public bool GetColorMode() => _colorMode;

    /// <summary>Set NameMode for indicate read Name or not.</summary>
    public void SetNameMode(bool namemode) => _nameMode = namemode;

    public bool GetNameMode() => _nameMode;

    /// <summary>Set LayerMode for indicate read Layers or not.</summary>
    public void SetLayerMode(bool layermode) => _layerMode = layermode;

    public bool GetLayerMode() => _layerMode;

    /// <summary>PropsMode for indicate read Validation properties or not.</summary>
    public void SetPropsMode(bool propsmode) => _propsMode = propsmode;

    public bool GetPropsMode() => _propsMode;

    /// <summary>MetaMode for indication to read Metadata for Part/Product or not.</summary>
    public void SetMetaMode(bool metaMode) => _metaMode = metaMode;

    public bool GetMetaMode() => _metaMode;

    /// <summary>MetaMode for indication to read metadata from product of a part.</summary>
    public void SetProductMetaMode(bool productMetaMode) => _productMetaMode = productMetaMode;

    public bool GetProductMetaMode() => _productMetaMode;

    /// <summary>Set SHUO mode for indicate write SHUO or not.</summary>
    public void SetSHUOMode(bool shuomode) => _shuoMode = shuomode;

    public bool GetSHUOMode() => _shuoMode;

    /// <summary>Set GDT mode for indicate write GDT or not.</summary>
    public void SetGDTMode(bool gdtmode) => _gdtMode = gdtmode;

    public bool GetGDTMode() => _gdtMode;

    /// <summary>Set Material mode</summary>
    public void SetMatMode(bool matmode) => _matMode = matmode;

    public bool GetMatMode() => _matMode;

    /// <summary>Set View mode</summary>
    public void SetViewMode(bool viewmode) => _viewMode = viewmode;

    /// <summary>Get View mode</summary>
    public bool GetViewMode() => _viewMode;

    /// <summary>Sets parameters for shape processing.</summary>
    public void SetShapeFixParameters(Dictionary<string, string> parameters)
    {
        _reader.SetShapeFixParameters(parameters);
    }

    /// <summary>Sets parameters for shape processing.</summary>
    public void SetShapeFixParameters(in ShapeFixParameters parameters, Dictionary<string, string> additionalParameters)
    {
        _reader.SetShapeFixParameters(parameters, additionalParameters);
    }

    /// <summary>Returns parameters for shape processing that was set by SetParameters() method.</summary>
    public Dictionary<string, string> GetShapeFixParameters()
    {
        return _reader.GetShapeFixParameters();
    }

    /// <summary>Sets flags defining operations to be performed on shapes.</summary>
    public void SetShapeProcessFlags(global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags flags)
    {
        _reader.SetShapeProcessFlags(flags);
    }

    /// <summary>Returns flags defining operations to be performed on shapes.</summary>
    public (global::Kerf.Engine.Healing.Process.ShapeProcess.OperationsFlags, bool) GetShapeProcessFlags()
    {
        return _reader.GetShapeProcessFlags();
    }

    private readonly StepReader _reader;
    private readonly Dictionary<string, ExternFile> _files = new Dictionary<string, ExternFile>();
    private readonly Dictionary<Shape, Label> _map =
        new Dictionary<Shape, Label>(ShapeHasher.Instance);
    private bool _colorMode;
    private bool _nameMode;
    private bool _layerMode;
    private bool _propsMode;
    private bool _metaMode;
    private bool _productMetaMode;
    private bool _shuoMode;
    private bool _gdtMode;
    private bool _matMode;
    private bool _viewMode;
    // Kerf (unit 8Z): the external reads
    private bool _externFilesMode = true;
    private ExternFileResolver? _externResolver;
    private ExternFileLimits _externLimits = ExternFileLimits.Default;
    private bool _isStreamLoaded;
    private ParserReadLimits _readLimits;
    private int _externCount;
    private long _externBytes;
    private readonly List<string> _externStack = new List<string>();
}
