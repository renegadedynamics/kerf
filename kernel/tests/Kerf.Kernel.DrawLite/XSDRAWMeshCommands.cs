// Port of: src/Draw/TKXSDRAWGLTF/XSDRAWGLTF/XSDRAWGLTF.cxx (ReadGltf, readgltf, WriteGltf, writegltf),
//          src/Draw/TKXSDRAWOBJ/XSDRAWOBJ/XSDRAWOBJ.cxx (WriteObj, writeobj)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2023-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Gltf;
using Kerf.Engine.Exchange.Obj;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// The glTF and OBJ commands (unit 8G1) over Draw-lite's documents (the session's unit, xstep.cascade.unit, for a
/// document without one, since unit 8W), with unit 8T's fresh reader and writers in place
/// of RWGltf_CafReader, RWGltf_CafWriter and RWObj_CafWriter (decisions 1233, 1234, 1236). Files are Draw-lite's
/// overlay (TclFileSystem): written files stay in memory, read files come from it or the disk.
/// </summary>
/// <remarks>
/// The options the fresh writers have no counterpart for are accepted and have no effect (decision 1260): glTF's
/// <c>-trsfFormat</c> (matrices are written), <c>-mergeFaces</c>, <c>-splitIndices16</c>, <c>-forceUVExport</c>,
/// <c>-texturesSeparate</c>, Draco and <c>-parallel</c>; <c>-comments</c> and <c>-author</c>; the name formats other
/// than <c>product</c> (instance names dropped) and <c>instOrProd</c> (the writer's own rule). Reading:
/// <c>-noCreateDoc</c>, <c>-listExternalFiles</c>, <c>-assetInfo</c>, late loading, <c>-allScenes</c> and the
/// precision options are not supported. <c>ReadObj</c> and <c>readobj</c> are not registered: there is no OBJ reader.
/// </remarks>
internal static class XSDRAWMeshCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "ReadGltf", ReadGltf);
        DrawCxx.Add(theInterpreter, "readgltf", ReadGltf);
        DrawCxx.Add(theInterpreter, "WriteGltf", WriteGltf);
        DrawCxx.Add(theInterpreter, "writegltf", WriteGltf);
        DrawCxx.Add(theInterpreter, "WriteObj", WriteObj);
        DrawCxx.Add(theInterpreter, "writeobj", WriteObj);
    }

    // Message::SendFail (decision 405)
    private static int fail(DrawInterpreter theDI, string theMessage)
    {
        theDI.Output.WriteLine(theMessage);
        return 1;
    }

    // XSDRAW::GetLengthUnit: the document's unit in millimetres, else the session's (xstep.cascade.unit)
    private static double lengthUnitMM(DrawInterpreter theDI, ExchangeDocument? theDoc)
        => theDoc is not null && theDoc.GetLengthUnit(out double aUnit) ? aUnit * 1000.0 : XSDRAWCommands.GetLengthUnit(theDI);

    /// <summary>
    /// A glTF file of Draw-lite's files read by unit 8T's reader into the system unit (metres per unit), its external
    /// buffers resolved beside it; null, with the message, if it isn't found or can't be read.
    /// </summary>
    internal static Tree? ReadGltfTree(DrawInterpreter theDI, string theFilePath, double theScaleFactorM)
    {
        var aReader = new TreeReader(new ReadOptions { OutputLengthUnit = theScaleFactorM });
        if (!theDI.Files.TryReadBytes(theFilePath, out byte[] aBytes))
        {
            fail(theDI, "File '" + theFilePath + "' is not found");
            return null;
        }
        string aFolder = theDI.Files.Full(theFilePath);
        aFolder = aFolder[..(aFolder.LastIndexOf('/') + 1)];
        Stream? resolve(string theUri)
        {
            if (Path.IsPathRooted(theUri) || theUri.Contains(':', StringComparison.Ordinal))
            {
                return null;
            }
            return theDI.Files.TryReadBytes(aFolder + theUri, out byte[] aBuffer) ? new MemoryStream(aBuffer) : null;
        }
        Tree? aTree = aReader.Read(new MemoryStream(aBytes), resolve);
        if (aTree is null)
        {
            fail(theDI, "Error: file '" + theFilePath + "' can not be read: " + aReader.Error);
        }
        return aTree;
    }

    /// <summary>A tree written by unit 8T's glTF writer to Draw-lite's files (a .glb, or a .gltf with its .bin beside it).</summary>
    internal static void WriteGltfTree(DrawInterpreter theDI, string theFilePath, Tree theTree, double theScaleFactorM, bool theIsZUp, bool theToMergeFaces)
    {
        var aWriter = new GltfTreeWriter(new GltfWriterOptions { InputLengthUnit = theScaleFactorM, ZUpToYUp = theIsZUp, MergeFaces = theToMergeFaces });
        if (theFilePath.EndsWith(".glb", StringComparison.OrdinalIgnoreCase))
        {
            var aGlb = new MemoryStream();
            aWriter.WriteGlb(theTree, aGlb);
            theDI.Files.WriteBytes(theFilePath, aGlb.ToArray());
        }
        else
        {
            string aBinPath = Path.ChangeExtension(theFilePath, ".bin");
            var aJson = new MemoryStream();
            var aBin = new MemoryStream();
            aWriter.WriteGltf(theTree, aJson, aBin, Uri.EscapeDataString(Path.GetFileName(aBinPath)));
            theDI.Files.WriteBytes(theFilePath, aJson.ToArray());
            theDI.Files.WriteBytes(aBinPath, aBin.ToArray());
        }
    }

    private static int ReadGltf(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVec)
    {
        string aDestName = "", aFilePath = "";
        bool toUseExistingDoc = false;
        bool toListExternalFiles = false, toPrintAssetInfo = false;
        bool isNoDoc = theArgVec[0] == "readgltf";
        for (int anArgIter = 1; anArgIter < theArgVec.Count; ++anArgIter)
        {
            string anArgCase = theArgVec[anArgIter].ToLowerInvariant();
            if (!isNoDoc && anArgCase is "-nocreate" or "-nocreatedoc")
            {
                toUseExistingDoc = DBRepCommands.ParseOnOffIterator(theArgVec, ref anArgIter);
            }
            else if (anArgCase is "-parallel" or "-doubleprec" or "-doubleprecision" or "-singleprec" or "-singleprecision"
                     or "-skiplateloading" or "-keeplate" or "-allscenes" or "-toprintinfo" or "-toprintdebuginfo" or "-applyscale")
            {
                DBRepCommands.ParseOnOffIterator(theArgVec, ref anArgIter);
            }
            else if (anArgCase is "-listexternalfiles" or "-listexternals" or "-listexternal" or "-external" or "-externalfiles")
            {
                toListExternalFiles = DBRepCommands.ParseOnOffIterator(theArgVec, ref anArgIter);
            }
            else if (anArgCase is "-assetinfo" or "-metadata")
            {
                toPrintAssetInfo = DBRepCommands.ParseOnOffIterator(theArgVec, ref anArgIter);
            }
            else if (aDestName.Length == 0)
            {
                aDestName = theArgVec[anArgIter];
            }
            else if (aFilePath.Length == 0)
            {
                aFilePath = theArgVec[anArgIter];
            }
            else
            {
                return fail(theDI, "Syntax error at '" + theArgVec[anArgIter] + "'");
            }
        }
        if (aFilePath.Length == 0 && aDestName.Length != 0 && (toListExternalFiles || toPrintAssetInfo))
        {
            (aFilePath, aDestName) = (aDestName, aFilePath);
        }
        if (aFilePath.Length == 0)
        {
            return fail(theDI, "Syntax error: wrong number of arguments");
        }
        if (toUseExistingDoc || toListExternalFiles || toPrintAssetInfo || aDestName.Length == 0)
        {
            // decision 1260
            return fail(theDI, "Error: " + theArgVec[0] + " of Draw-lite reads into a new document or shape only");
        }

        ExchangeDocument? anOld = isNoDoc ? null : DDocStdCommands.GetDocument(theDI, aDestName, false);
        if (anOld is not null)
        {
            return fail(theDI, "Error: document with name " + aDestName + " already exists");
        }

        // a new document has no unit: the session's
        double aScaleFactorM = lengthUnitMM(theDI, null) / 1000.0;
        Tree aTree = ReadGltfTree(theDI, aFilePath, aScaleFactorM) ?? Tree.Empty;

        if (isNoDoc)
        {
            // RWMesh_CafReader::SingleShape
            ExchangeDocument aTmp = ExchangeDocument.FromTree(aTree);
            theDI.SetShape(aDestName, XDEDRAWCommands.GetOneShape(aTmp));
            return 0;
        }

        ExchangeDocument aDoc = ExchangeDocument.FromTree(aTree);
        NameInstances(aDoc);
        // RWMesh_CafReader::fillDocument: the system unit when the document has none
        if (!aDoc.GetLengthUnit(out _))
        {
            aDoc.SetLengthUnit(aScaleFactorM, global::Kerf.Engine.Exchange.Framework.UnitsMethods.DumpLengthUnit(aScaleFactorM, global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit.LengthUnit_Meter));
        }
        DDocStdCommands.NewDocument(theDI, aDestName, aDoc);
        return 0;
    }

    /// <summary>
    /// RWMesh_CafReader's names (addShapeIntoDoc, setShapeName): an instance without a name of its own takes its
    /// product's, as the readers of OCCT's mesh formats name every instance (unit 8T's trees name an instance only by
    /// a name differing from its product's: decision 1236).
    /// </summary>
    internal static void NameInstances(ExchangeDocument theDoc)
    {
        var aStack = new Stack<Label>(theDoc.Children(theDoc.ShapesLabel()).Reverse());
        while (aStack.Count > 0)
        {
            Label aLabel = aStack.Pop();
            if (theDoc.GetReferredShape(aLabel, out Label aReferred)
                && !theDoc.GetName(aLabel, out _) && theDoc.GetName(aReferred, out string? aName) && !string.IsNullOrEmpty(aName))
            {
                theDoc.SetName(aLabel, aName);
            }
            if (theDoc.IsAssembly(aLabel))
            {
                foreach (Label aChild in theDoc.Children(aLabel).Reverse())
                {
                    aStack.Push(aChild);
                }
            }
        }
    }

    // the document of the argument, or a new one holding the shape it names; null with the message otherwise
    private static ExchangeDocument? docOrShape(DrawInterpreter theDI, string theName, out bool theIsShape)
    {
        theIsShape = false;
        ExchangeDocument? aDoc = DDocStdCommands.GetDocument(theDI, theName, false);
        if (aDoc is not null)
        {
            return aDoc;
        }
        Shape aShape = theDI.GetShape(theName);
        if (aShape.IsNull)
        {
            fail(theDI, "Syntax error: '" + theName + "' is not a shape nor document");
            return null;
        }
        theIsShape = true;
        aDoc = new ExchangeDocument { AutoNaming = DDocStdCommands.AutoNaming(theDI) };
        aDoc.AddShape(aShape);
        return aDoc;
    }

    // RWMesh_NameFormat_Product: the nodes named by their products only
    private static Tree productNames(Tree theTree)
    {
        var aDone = new Dictionary<DocumentProduct, DocumentProduct>(ReferenceEqualityComparer.Instance);
        DocumentProduct product(DocumentProduct theProduct)
        {
            if (aDone.TryGetValue(theProduct, out DocumentProduct? aKnown))
            {
                return aKnown;
            }
            DocumentProduct aNew = theProduct.IsAssembly ? theProduct.WithComponents(theProduct.Components.Select(instance)) : theProduct;
            aDone[theProduct] = aNew;
            return aNew;
        }
        Instance instance(Instance theInstance) => new(product(theInstance.Product), theInstance.Location, theInstance.Attributes.WithName(null));
        return theTree.WithRoots(theTree.Roots.Select(instance));
    }

    private static int WriteGltf(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVec)
    {
        string aGltfFilePath = "";
        ExchangeDocument? aDoc = null;
        bool isProductNames = false;
        bool isZUp = true;
        bool toMergeFaces = false;
        int theNbArgs = theArgVec.Count;
        for (int anArgIter = 1; anArgIter < theNbArgs; ++anArgIter)
        {
            string anArgCase = theArgVec[anArgIter].ToLowerInvariant();
            if (anArgCase is "-comments" or "-author" && anArgIter + 1 < theNbArgs)
            {
                ++anArgIter;
            }
            else if (anArgCase == "-mergefaces")
            {
                toMergeFaces = true;
                if (anArgIter + 1 < theNbArgs && DBRepCommands.ParseOnOff(theArgVec[anArgIter + 1], ref toMergeFaces))
                {
                    ++anArgIter;
                }
            }
            else if (anArgCase is "-forceuvexport" or "-forceuv"
                     or "-splitindices16" or "-splitindexes16" or "-splitindices" or "-splitindexes" or "-splitind")
            {
                bool anIgnored = true;
                if (anArgIter + 1 < theNbArgs && DBRepCommands.ParseOnOff(theArgVec[anArgIter + 1], ref anIgnored))
                {
                    ++anArgIter;
                }
            }
            else if (anArgIter + 1 < theNbArgs
                     && anArgCase is "-systemcoordinatesystem" or "-systemcoordsystem" or "-systemcoordsys" or "-syscoordsys")
            {
                string aCS = theArgVec[++anArgIter].ToLowerInvariant();
                if (aCS is not ("zup" or "yup"))
                {
                    return fail(theDI, "Syntax error: unknown coordinate system '" + theArgVec[anArgIter] + "'");
                }
                isZUp = aCS == "zup";
            }
            else if (anArgCase == "-trsfformat" && anArgIter + 1 < theNbArgs)
            {
                if (theArgVec[++anArgIter].ToLowerInvariant() is not ("compact" or "mat4" or "trs"))
                {
                    return fail(theDI, "Syntax error at '" + anArgCase + "'");
                }
            }
            else if (anArgCase is "-nodenameformat" or "-nodename" or "-meshnameformat" or "-meshname")
            {
                ++anArgIter;
                string? aFormat = anArgIter < theNbArgs ? parseNameFormat(theArgVec[anArgIter]) : null;
                if (aFormat is null)
                {
                    return fail(theDI, "Syntax error at '" + anArgCase + "'");
                }
                if (anArgCase.StartsWith("-node", StringComparison.Ordinal))
                {
                    isProductNames = aFormat == "product";
                }
            }
            else if (aDoc is null)
            {
                aDoc = docOrShape(theDI, theArgVec[anArgIter], out bool isShape);
                if (aDoc is null)
                {
                    return 1;
                }
                // auto-naming doesn't generate meaningful instance names
                isProductNames |= isShape;
            }
            else if (aGltfFilePath.Length == 0)
            {
                aGltfFilePath = theArgVec[anArgIter];
            }
            else if (anArgCase is "-texturesseparate")
            {
            }
            else if (anArgCase is "-draco" or "-unifiedquantization" or "-parallel")
            {
                DBRepCommands.ParseOnOffIterator(theArgVec, ref anArgIter);
            }
            else if (anArgCase is "-compressionlevel" or "-quantizepositionbits" or "-quantizenormalbits" or "-quantizetexcoordbits"
                     or "-quantizecolorbits" or "-quantizegenericbits" && anArgIter + 1 < theNbArgs
                     && DDocStdCommands.ParseInteger(theDI, theArgVec[anArgIter + 1], out _))
            {
                ++anArgIter;
            }
            else
            {
                return fail(theDI, "Syntax error at '" + theArgVec[anArgIter] + "'");
            }
        }
        if (aGltfFilePath.Length == 0 || aDoc is null)
        {
            return fail(theDI, "Syntax error: wrong number of arguments");
        }

        double aScaleFactorM = lengthUnitMM(theDI, aDoc) / 1000.0;
        Tree aTree = aDoc.ToTree();
        if (isProductNames)
        {
            aTree = productNames(aTree);
        }
        WriteGltfTree(theDI, aGltfFilePath, aTree, aScaleFactorM, isZUp, toMergeFaces);
        return 0;
    }

    // parseNameFormat: the formats Draw accepts, normalised; null if unknown
    private static string? parseNameFormat(string theArg) => theArg.ToLowerInvariant() switch
    {
        "empty" => "empty",
        "product" or "prod" => "product",
        "instance" or "inst" => "instance",
        "instanceorproduct" or "instance||product" or "instance|product" or "instorprod" or "inst||prod" or "inst|prod" => "instorprod",
        "productorinstance" or "product||instance" or "product|instance" or "prodorinst" or "prod||inst" or "prod|inst" => "prodorinst",
        "productandinstance" or "prodandinst" or "product&instance" or "prod&inst" => "prodandinst",
        "productandinstanceandocaf" or "verbose" or "debug" => "verbose",
        _ => null,
    };

    private static int WriteObj(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVec)
    {
        string anObjFilePath = "";
        ExchangeDocument? aDoc = null;
        double aFileUnitFactor = -1.0;
        bool isSystemZUp = true, isFileZUp = false;
        int theNbArgs = theArgVec.Count;
        for (int anArgIter = 1; anArgIter < theNbArgs; ++anArgIter)
        {
            string anArgCase = theArgVec[anArgIter].ToLowerInvariant();
            if (anArgIter + 1 < theNbArgs && anArgCase is "-unit" or "-units" or "-fileunit" or "-fileunits")
            {
                // UnitsAPI::AnyToSI (1.0, unit): the length units UnitsMethods knows (decision 1260)
                string aUnitStr = theArgVec[++anArgIter];
                global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit aUnit = global::Kerf.Engine.Exchange.Framework.UnitsMethods.LengthUnitFromString(aUnitStr, false);
                aFileUnitFactor = aUnit == global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit.LengthUnit_Undefined
                                    ? -1.0
                                    : global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetLengthFactorValue((int)aUnit) * 0.001;
                if (aFileUnitFactor <= 0.0)
                {
                    return fail(theDI, "Syntax error: wrong length unit '" + aUnitStr + "'");
                }
            }
            else if (anArgIter + 1 < theNbArgs && anArgCase is "-filecoordinatesystem" or "-filecoordsystem" or "-filecoordsys"
                     or "-systemcoordinatesystem" or "-systemcoordsystem" or "-systemcoordsys" or "-syscoordsys")
            {
                string aCS = theArgVec[++anArgIter].ToLowerInvariant();
                if (aCS is not ("zup" or "yup"))
                {
                    return fail(theDI, "Syntax error: unknown coordinate system '" + theArgVec[anArgIter] + "'");
                }
                if (anArgCase.StartsWith("-file", StringComparison.Ordinal))
                {
                    isFileZUp = aCS == "zup";
                }
                else
                {
                    isSystemZUp = aCS == "zup";
                }
            }
            else if (anArgCase is "-comments" or "-author" && anArgIter + 1 < theNbArgs)
            {
                ++anArgIter;
            }
            else if (aDoc is null)
            {
                aDoc = docOrShape(theDI, theArgVec[anArgIter], out _);
                if (aDoc is null)
                {
                    return 1;
                }
            }
            else if (anObjFilePath.Length == 0)
            {
                anObjFilePath = theArgVec[anArgIter];
            }
            else
            {
                return fail(theDI, "Syntax error at '" + theArgVec[anArgIter] + "'");
            }
        }
        if (anObjFilePath.Length == 0 || aDoc is null)
        {
            return fail(theDI, "Syntax error: wrong number of arguments");
        }

        // the session's unit (XSDRAW::GetLengthUnit () without a document) to the file's; the axes Z-up to Y-up when
        // the system is Z-up and the file Y-up (RWMesh_CoordinateSystemConverter)
        double aSystemUnitFactor = XSDRAWCommands.GetLengthUnit(theDI) * 0.001;
        double aScale = aFileUnitFactor > 0.0 ? aSystemUnitFactor / aFileUnitFactor : 1.0;
        // (a Y-up system written to a Z-up file is written unchanged: decision 1260)
        var aWriter = new ObjTreeWriter(new ObjWriterOptions { Scale = aScale, ZUpToYUp = isSystemZUp && !isFileZUp });
        string aMtlPath = Path.ChangeExtension(anObjFilePath, ".mtl");
        var anObj = new MemoryStream();
        var aMtl = new MemoryStream();
        aWriter.Write(aDoc.ToTree(), anObj, aMtl, Path.GetFileName(aMtlPath));
        theDI.Files.WriteBytes(anObjFilePath, anObj.ToArray());
        theDI.Files.WriteBytes(aMtlPath, aMtl.ToArray());
        return 0;
    }
}
