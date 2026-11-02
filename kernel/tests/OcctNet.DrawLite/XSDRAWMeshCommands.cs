// Port of: src/Draw/TKXSDRAWGLTF/XSDRAWGLTF/XSDRAWGLTF.cxx (ReadGltf, readgltf, WriteGltf, writegltf),
//          src/Draw/TKXSDRAWOBJ/XSDRAWOBJ/XSDRAWOBJ.cxx (WriteObj, writeobj)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2023-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The glTF and OBJ commands (unit 8G1) over Draw-lite's documents, with unit 8T's fresh reader and writers in place
/// of RWGltf_CafReader, RWGltf_CafWriter and RWObj_CafWriter (decisions 13105, 13106, 13108). Files are Draw-lite's
/// overlay (TclFileSystem): written files stay in memory, read files come from it or the disk.
/// </summary>
/// <remarks>
/// The options the fresh writers have no counterpart for are accepted and have no effect (decision 13305): glTF's
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

    // XSDRAW::GetLengthUnit: the document's unit in millimetres, else the session's (millimetres until unit 8F)
    private static double lengthUnitMM(XdeDoc_Document? theDoc)
        => theDoc is not null && theDoc.GetLengthUnit(out double aUnit) ? aUnit * 1000.0 : UnitsMethods.GetCasCadeLengthUnit(null);

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
            // decision 13305
            return fail(theDI, "Error: " + theArgVec[0] + " of Draw-lite reads into a new document or shape only");
        }

        XdeDoc_Document? anOld = isNoDoc ? null : DDocStdCommands.GetDocument(theDI, aDestName, false);
        if (anOld is not null)
        {
            return fail(theDI, "Error: document with name " + aDestName + " already exists");
        }

        // a new document has no unit: the session's
        double aScaleFactorM = lengthUnitMM(null) / 1000.0;
        var aReader = new RWGltf_TreeReader(new RWGltf_ReadOptions { OutputLengthUnit = aScaleFactorM });
        XdeDoc_Tree? aTree = null;
        if (theDI.Files.TryReadBytes(aFilePath, out byte[] aBytes))
        {
            string aFolder = theDI.Files.Full(aFilePath);
            aFolder = aFolder[..(aFolder.LastIndexOf('/') + 1)];
            Stream? resolve(string theUri)
            {
                if (Path.IsPathRooted(theUri) || theUri.Contains(':', StringComparison.Ordinal))
                {
                    return null;
                }
                return theDI.Files.TryReadBytes(aFolder + theUri, out byte[] aBuffer) ? new MemoryStream(aBuffer) : null;
            }
            aTree = aReader.Read(new MemoryStream(aBytes), resolve);
            if (aTree is null)
            {
                fail(theDI, "Error: file '" + aFilePath + "' can not be read: " + aReader.Error);
            }
        }
        else
        {
            fail(theDI, "File '" + aFilePath + "' is not found");
        }
        aTree ??= XdeDoc_Tree.Empty;

        if (isNoDoc)
        {
            // RWMesh_CafReader::SingleShape
            XdeDoc_Document aTmp = XdeDoc_Document.FromTree(aTree);
            theDI.SetShape(aDestName, XDEDRAWCommands.GetOneShape(aTmp));
            return 0;
        }

        XdeDoc_Document aDoc = XdeDoc_Document.FromTree(aTree);
        NameInstances(aDoc);
        // RWMesh_CafReader::fillDocument: the system unit when the document has none
        if (!aDoc.GetLengthUnit(out _))
        {
            aDoc.SetLengthUnit(aScaleFactorM, UnitsMethods.DumpLengthUnit(aScaleFactorM, UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Meter));
        }
        DDocStdCommands.NewDocument(theDI, aDestName, aDoc);
        return 0;
    }

    /// <summary>
    /// RWMesh_CafReader's names (addShapeIntoDoc, setShapeName): an instance without a name of its own takes its
    /// product's, as the readers of OCCT's mesh formats name every instance (unit 8T's trees name an instance only by
    /// a name differing from its product's: decision 13108).
    /// </summary>
    internal static void NameInstances(XdeDoc_Document theDoc)
    {
        var aStack = new Stack<XdeDoc_Label>(theDoc.Children(theDoc.ShapesLabel()).Reverse());
        while (aStack.Count > 0)
        {
            XdeDoc_Label aLabel = aStack.Pop();
            if (theDoc.GetReferredShape(aLabel, out XdeDoc_Label aReferred)
                && !theDoc.GetName(aLabel, out _) && theDoc.GetName(aReferred, out string? aName) && !string.IsNullOrEmpty(aName))
            {
                theDoc.SetName(aLabel, aName);
            }
            if (theDoc.IsAssembly(aLabel))
            {
                foreach (XdeDoc_Label aChild in theDoc.Children(aLabel).Reverse())
                {
                    aStack.Push(aChild);
                }
            }
        }
    }

    // the document of the argument, or a new one holding the shape it names; null with the message otherwise
    private static XdeDoc_Document? docOrShape(DrawInterpreter theDI, string theName, out bool theIsShape)
    {
        theIsShape = false;
        XdeDoc_Document? aDoc = DDocStdCommands.GetDocument(theDI, theName, false);
        if (aDoc is not null)
        {
            return aDoc;
        }
        TopoDS_Shape aShape = theDI.GetShape(theName);
        if (aShape.IsNull())
        {
            fail(theDI, "Syntax error: '" + theName + "' is not a shape nor document");
            return null;
        }
        theIsShape = true;
        aDoc = new XdeDoc_Document { AutoNaming = DDocStdCommands.AutoNaming(theDI) };
        aDoc.AddShape(aShape);
        return aDoc;
    }

    // RWMesh_NameFormat_Product: the nodes named by their products only
    private static XdeDoc_Tree productNames(XdeDoc_Tree theTree)
    {
        var aDone = new Dictionary<XdeDoc_Product, XdeDoc_Product>(ReferenceEqualityComparer.Instance);
        XdeDoc_Product product(XdeDoc_Product theProduct)
        {
            if (aDone.TryGetValue(theProduct, out XdeDoc_Product? aKnown))
            {
                return aKnown;
            }
            XdeDoc_Product aNew = theProduct.IsAssembly ? theProduct.WithComponents(theProduct.Components.Select(instance)) : theProduct;
            aDone[theProduct] = aNew;
            return aNew;
        }
        XdeDoc_Instance instance(XdeDoc_Instance theInstance) => new(product(theInstance.Product), theInstance.Location, theInstance.Attributes.WithName(null));
        return theTree.WithRoots(theTree.Roots.Select(instance));
    }

    private static int WriteGltf(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVec)
    {
        string aGltfFilePath = "";
        XdeDoc_Document? aDoc = null;
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

        double aScaleFactorM = lengthUnitMM(aDoc) / 1000.0;
        XdeDoc_Tree aTree = aDoc.ToTree();
        if (isProductNames)
        {
            aTree = productNames(aTree);
        }
        var aWriter = new RWGltf_TreeWriter(new RWGltf_WriteOptions { InputLengthUnit = aScaleFactorM, ZUpToYUp = isZUp, MergeFaces = toMergeFaces });
        if (aGltfFilePath.EndsWith(".glb", StringComparison.OrdinalIgnoreCase))
        {
            var aGlb = new MemoryStream();
            aWriter.WriteGlb(aTree, aGlb);
            theDI.Files.WriteBytes(aGltfFilePath, aGlb.ToArray());
        }
        else
        {
            string aBinPath = Path.ChangeExtension(aGltfFilePath, ".bin");
            var aJson = new MemoryStream();
            var aBin = new MemoryStream();
            aWriter.WriteGltf(aTree, aJson, aBin, Uri.EscapeDataString(Path.GetFileName(aBinPath)));
            theDI.Files.WriteBytes(aGltfFilePath, aJson.ToArray());
            theDI.Files.WriteBytes(aBinPath, aBin.ToArray());
        }
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
        XdeDoc_Document? aDoc = null;
        double aFileUnitFactor = -1.0;
        bool isSystemZUp = true, isFileZUp = false;
        int theNbArgs = theArgVec.Count;
        for (int anArgIter = 1; anArgIter < theNbArgs; ++anArgIter)
        {
            string anArgCase = theArgVec[anArgIter].ToLowerInvariant();
            if (anArgIter + 1 < theNbArgs && anArgCase is "-unit" or "-units" or "-fileunit" or "-fileunits")
            {
                // UnitsAPI::AnyToSI (1.0, unit): the length units UnitsMethods knows (decision 13305)
                string aUnitStr = theArgVec[++anArgIter];
                UnitsMethods_LengthUnit aUnit = UnitsMethods.LengthUnitFromString(aUnitStr, false);
                aFileUnitFactor = aUnit == UnitsMethods_LengthUnit.UnitsMethods_LengthUnit_Undefined
                                    ? -1.0
                                    : UnitsMethods.GetLengthFactorValue((int)aUnit) * 0.001;
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
        double aSystemUnitFactor = UnitsMethods.GetCasCadeLengthUnit(null) * 0.001;
        double aScale = aFileUnitFactor > 0.0 ? aSystemUnitFactor / aFileUnitFactor : 1.0;
        // (a Y-up system written to a Z-up file is written unchanged: decision 13305)
        var aWriter = new RWObj_TreeWriter(new RWObj_WriteOptions { Scale = aScale, ZUpToYUp = isSystemZUp && !isFileZUp });
        string aMtlPath = Path.ChangeExtension(anObjFilePath, ".mtl");
        var anObj = new MemoryStream();
        var aMtl = new MemoryStream();
        aWriter.Write(aDoc.ToTree(), anObj, aMtl, Path.GetFileName(aMtlPath));
        theDI.Files.WriteBytes(anObjFilePath, anObj.ToArray());
        theDI.Files.WriteBytes(aMtlPath, aMtl.ToArray());
        return 0;
    }
}
