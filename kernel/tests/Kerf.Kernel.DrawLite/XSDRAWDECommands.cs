// Port of: src/Draw/TKXSDRAWDE/XSDRAWDE/XSDRAWDE.cxx (ReadFile, readfile, WriteFile, writefile),
//          src/DataExchange/TKDE/DE/DE_Wrapper.cxx (FindReadProvider, FindWriteProvider, Load),
//          src/DataExchange/TKDE/DE/DE_ConfigurationContext.cxx (the configuration's text),
//          src/DataExchange/TKDESTEP/DESTEP/DESTEP_Provider.cxx, DESTEP_ConfigurationNode.cxx (Load),
//          src/DataExchange/TKDESTL/DESTL/DESTL_Provider.cxx, DESTL_ConfigurationNode.cxx,
//          src/DataExchange/TKDEGLTF/DEGLTF/DEGLTF_Provider.cxx, DEGLTF_ConfigurationNode.cxx,
//          src/DataExchange/TKDECascade/DEBREP/DEBREP_Provider.cxx, DEBREP_ConfigurationNode.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2023-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange;
using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Exchange.Documents;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Exchange.Step.Caf;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Stl.Api;
using Kerf.Engine.Foundation.Resources;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Globalization;
using System.Text;
using Kerf.Engine;
using static Kerf.Engine.Exchange.Framework.Selection.ReturnStatus;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// XSDRAWDE's <c>ReadFile</c>, <c>readfile</c>, <c>WriteFile</c> and <c>writefile</c> (unit 8W): a file read into a
/// document or a shape, or written from one, by the format its name (or, reading, its content) gives, with the
/// configuration's parameters (<c>-conf</c>).
/// </summary>
/// <remarks>
/// TKDE's framework (DE_Wrapper, the configuration nodes and the providers) isn't ported (decision 1271): this is
/// Draw-lite's stand-in for <c>DE_Wrapper::GlobalWrapper()->Copy()</c> with the providers of the formats the port reads
/// and writes, STEP (DESTEP_Provider), STL (DESTL_Provider), glTF (DEGLTF_Provider, over unit 8T's reader and writer)
/// and BRep (DEBREP_Provider), whose code is ported over Draw-lite's documents and files. The configuration's text is
/// read as DE_ConfigurationContext reads it (a file if the argument names one, else lines of
/// <c>scope.key : value</c>); the global units and each of these providers' parameters are taken from it. The formats
/// OCCT finds a provider for that the port has none of (IGES, OBJ reading, PLY, VRML, OCAF's XBF) fail with a message.
/// The commands run as XSDRAW's (decision 1275): the messages of the providers and the libraries (Message::SendFail,
/// SendWarning, SendInfo) go to the command's result, as DRAWEXE's Draw_Printer gives them.
/// XSDRAW::CollectActiveWorkSessions is left out.
/// </remarks>
internal static class XSDRAWDECommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        XSDRAWCommands.Pilot(theInterpreter, "ReadFile", ReadFile);
        XSDRAWCommands.Pilot(theInterpreter, "readfile", ReadFile);
        XSDRAWCommands.Pilot(theInterpreter, "WriteFile", WriteFile);
        XSDRAWCommands.Pilot(theInterpreter, "writefile", WriteFile);
    }

    // Message::SendFail and SendWarning: to the command's result while it runs (XSDRAWCommands.Pilot, decision 1275)
    private static bool fail(DrawInterpreter theDI, string theMessage)
    {
        _ = theDI;
        global::Kerf.Engine.Foundation.Message.SendFail(theMessage);
        return false;
    }

    private static void warn(DrawInterpreter theDI, string theMessage)
    {
        _ = theDI;
        global::Kerf.Engine.Foundation.Message.SendWarning(theMessage);
    }

    // -------------------------------------------------------------------------------------------------------------
    // The commands

    private static int ReadFile(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVec)
    {
        int theNbArgs = theArgVec.Count;
        if (theNbArgs > 6)
        {
            theResult.Append("Use: ").Append(theArgVec[0]).Append(" name filePath [-conf <value|path>]\n");
            return 1;
        }
        string aDocShapeName = "", aFilePath = "", aConfString = "";
        ExchangeDocument? aDoc = null;
        bool isNoDoc = theArgVec[0] == "readfile";
        for (int anArgIter = 1; anArgIter < theNbArgs; ++anArgIter)
        {
            string anArg = theArgVec[anArgIter].ToLowerInvariant();
            if (anArg == "-conf" && anArgIter + 1 < theNbArgs)
            {
                ++anArgIter;
                aConfString = theArgVec[anArgIter];
            }
            else if (aDocShapeName.Length == 0)
            {
                aDocShapeName = theArgVec[anArgIter];
                if (!isNoDoc)
                {
                    aDoc = DDocStdCommands.GetDocument(theDI, aDocShapeName, false);
                }
            }
            else if (aFilePath.Length == 0)
            {
                aFilePath = theArgVec[anArgIter];
            }
            else
            {
                theResult.Append("Syntax error at argument '").Append(theArgVec[anArgIter]).Append("'\n");
                return 1;
            }
        }
        if (aDocShapeName.Length == 0 || aFilePath.Length == 0)
        {
            theResult.Append("Syntax error: wrong number of arguments\n");
            return 1;
        }
        if (aDoc is null && !isNoDoc)
        {
            aDoc = DDocStdCommands.NewDocument(theDI, theArgVec[1]);
        }

        var aConf = new Wrapper();
        bool aStat = true;
        if (aConfString.Length > 0)
        {
            aStat = aConf.Load(theDI, aConfString);
        }
        if (aStat)
        {
            if (isNoDoc)
            {
                aStat = aConf.Read(theDI, aFilePath, out Shape aShape);
                if (aStat)
                {
                    theDI.SetShape(aDocShapeName, aShape);
                }
            }
            else
            {
                aStat = aConf.Read(theDI, aFilePath, aDocShapeName, aDoc!);
            }
        }
        return aStat ? 0 : 1;
    }

    private static int WriteFile(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgVec)
    {
        int theNbArgs = theArgVec.Count;
        if (theNbArgs > 6)
        {
            theResult.Append("Use: ").Append(theArgVec[0]).Append(" name filePath [-conf <value|path>]\n");
            return 1;
        }
        string aDocShapeName = "", aFilePath = "", aConfString = "";
        ExchangeDocument? aDoc = null;
        bool isNoDoc = theArgVec[0] == "writefile";
        for (int anArgIter = 1; anArgIter < theNbArgs; ++anArgIter)
        {
            string anArg = theArgVec[anArgIter].ToLowerInvariant();
            if (anArg == "-conf" && anArgIter + 1 < theNbArgs)
            {
                ++anArgIter;
                aConfString = theArgVec[anArgIter];
            }
            else if (aDocShapeName.Length == 0)
            {
                aDocShapeName = theArgVec[anArgIter];
                if (!isNoDoc)
                {
                    aDoc = DDocStdCommands.GetDocument(theDI, aDocShapeName, false);
                }
            }
            else if (aFilePath.Length == 0)
            {
                aFilePath = theArgVec[anArgIter];
            }
            else
            {
                theResult.Append("Syntax error at argument '").Append(theArgVec[anArgIter]).Append("'\n");
                return 1;
            }
        }
        if (aDocShapeName.Length == 0 || aFilePath.Length == 0)
        {
            theResult.Append("Syntax error: wrong number of arguments\n");
            return 1;
        }
        if (aDoc is null && !isNoDoc)
        {
            theResult.Append("Error: incorrect document\n");
            return 1;
        }
        var aConf = new Wrapper();
        bool aStat = true;
        if (aConfString.Length > 0)
        {
            aStat = aConf.Load(theDI, aConfString);
        }
        if (aStat)
        {
            if (isNoDoc)
            {
                Shape aShape = theDI.GetShape(aDocShapeName);
                if (aShape.IsNull)
                {
                    theResult.Append("Error: incorrect shape ").Append(aDocShapeName).Append('\n');
                    return 1;
                }
                aStat = aConf.Write(theDI, aFilePath, aShape);
            }
            else
            {
                aStat = aConf.Write(theDI, aFilePath, aDoc!);
            }
        }
        return aStat ? 0 : 1;
    }

    // -------------------------------------------------------------------------------------------------------------
    // DE_ConfigurationContext: the configuration's resources, "scope.key : value"

    private sealed class Configuration
    {
        private readonly Dictionary<string, string> myResources = new(StringComparer.Ordinal);

        // DE_ConfigurationContext::Load: a file if one exists by that name, else the text itself
        public bool Load(DrawInterpreter theDI, string theConfiguration)
        {
            bool isFile;
            try
            {
                isFile = !theConfiguration.Contains('\n', StringComparison.Ordinal) && theDI.Files.IsFile(theConfiguration);
            }
            catch (Exception anException) when (anException is ArgumentException or NotSupportedException or IOException)
            {
                isFile = false;
            }
            return LoadStr(isFile ? theDI.Files.ReadAll(theConfiguration) : theConfiguration);
        }

        // DE_ConfigurationContext::LoadStr: line by line, comments (!) and empty lines skipped; an error stops it
        private bool LoadStr(string theText)
        {
            foreach (string aLine0 in theText.Split('\n'))
            {
                string aLine = aLine0.Trim(' ', '\t', '\r');
                if (aLine.Length == 0 || aLine[0] == '!')
                {
                    continue;
                }
                int aColon = aLine.IndexOf(':', StringComparison.Ordinal);
                if (aColon <= 0)
                {
                    return false; // DE_ConfigurationContext_KindOfLine_Error
                }
                myResources[aLine[..aColon].TrimEnd(' ', '\t')] = aLine[(aColon + 1)..].Trim(' ', '\t');
            }
            return true;
        }

        private bool Value(string theParam, string theScope, out string theValue)
            => myResources.TryGetValue(theScope.Length == 0 ? theParam : theScope + "." + theParam, out theValue!);

        public double RealVal(string theParam, double theDefault, string theScope)
            => Value(theParam, theScope, out string aValue) && double.TryParse(aValue, NumberStyles.Float, CultureInfo.InvariantCulture, out double aReal)
                   ? aReal : theDefault;

        public int IntegerVal(string theParam, int theDefault, string theScope)
            => Value(theParam, theScope, out string aValue) && int.TryParse(aValue, NumberStyles.Integer, CultureInfo.InvariantCulture, out int anInt)
                   ? anInt : theDefault;

        // DE_ConfigurationContext::BooleanVal: an integer, non-zero for true
        public bool BooleanVal(string theParam, bool theDefault, string theScope) => IntegerVal(theParam, theDefault ? 1 : 0, theScope) != 0;

        public string StringVal(string theParam, string theDefault, string theScope)
            => Value(theParam, theScope, out string aValue) ? aValue : theDefault;
    }

    // -------------------------------------------------------------------------------------------------------------
    // DE_Wrapper with the configuration nodes of the formats the port has providers for

    private enum Format
    {
        None,
        Step,
        Stl,
        Gltf,
        Brep,
        Unsupported,
    }

    private sealed class Wrapper
    {
        // DE_ConfigurationNode::GlobalParameters
        private double myLengthUnit = 1.0; // the target unit, scaled to millimetres
        private double mySystemUnit = 1.0; // the system unit, scaled to millimetres

        // DESTEP_ConfigurationNode
        private Parameters myStep = new();
        private readonly ShapeFixParameters myStepShapeFix = Parameters.GetDefaultShapeFixParameters();

        // DESTL_ConfigurationNode
        private double myStlMergeAngle = 90.0;
        private bool myStlReadBRep;
        private bool myStlWriteAscii = true;

        // DEGLTF_ConfigurationNode (the parameters unit 8T's reader and writer have)
        private bool myGltfMergeFaces;

        // DEBREP_ConfigurationNode
        private bool myBrepWriteBinary = true;
        private global::Kerf.Engine.Exchange.Brep.Binary.BinaryFormatVersion myBrepVersionBin = global::Kerf.Engine.Exchange.Brep.Binary.BinaryFormatVersion.FormatVersion_CURRENT;
        private FormatVersion myBrepVersionAscii = FormatVersion.FormatVersion_CURRENT;
        private bool myBrepWriteTriangles = true;
        private bool myBrepWriteNormals = true;

        // DE_Wrapper::Load(string, recursive): the global parameters, then each node's
        public bool Load(DrawInterpreter theDI, string theResource)
        {
            var aResource = new Configuration();
            if (!aResource.Load(theDI, theResource))
            {
                return true; // OCCT ignores DE_ConfigurationContext::Load's result
            }
            const string aGlobal = "global.general";
            myLengthUnit = aResource.RealVal("length.unit", myLengthUnit, aGlobal);
            mySystemUnit = aResource.RealVal("system.unit", mySystemUnit, aGlobal);
            loadStep(aResource);
            const string aStl = "provider.STL.OCC";
            myStlMergeAngle = aResource.RealVal("read.merge.angle", myStlMergeAngle, aStl);
            myStlReadBRep = aResource.BooleanVal("read.brep", myStlReadBRep, aStl);
            myStlWriteAscii = aResource.BooleanVal("write.ascii", myStlWriteAscii, aStl);
            myGltfMergeFaces = aResource.BooleanVal("write.merge.faces", myGltfMergeFaces, "provider.GLTF.OCC");
            const string aBrep = "provider.BREP.OCC";
            myBrepWriteBinary = aResource.BooleanVal("write.binary", myBrepWriteBinary, aBrep);
            myBrepVersionBin = (global::Kerf.Engine.Exchange.Brep.Binary.BinaryFormatVersion)aResource.IntegerVal("write.version.binary", (int)myBrepVersionBin, aBrep);
            myBrepVersionAscii = (FormatVersion)aResource.IntegerVal("write.version.ascii", (int)myBrepVersionAscii, aBrep);
            myBrepWriteTriangles = aResource.BooleanVal("write.triangles", myBrepWriteTriangles, aBrep);
            myBrepWriteNormals = aResource.BooleanVal("write.normals", myBrepWriteNormals, aBrep);
            return true;
        }

        // DESTEP_ConfigurationNode::Load (its shape healing parameters, DE_ShapeFixConfigurationNode's, aren't read)
        private void loadStep(Configuration theResource)
        {
            const string aScope = "provider.STEP.OCC";
            ref Parameters p = ref myStep;
            int i(string theKey, int theValue) => theResource.IntegerVal(theKey, theValue, aScope);
            bool b(string theKey, bool theValue) => theResource.BooleanVal(theKey, theValue, aScope);
            double r(string theKey, double theValue) => theResource.RealVal(theKey, theValue, aScope);
            p.ReadBSplineContinuity = (Parameters.ReadMode_BSplineContinuity)i("read.iges.bspline.continuity", (int)p.ReadBSplineContinuity);
            p.ReadPrecisionMode = (Parameters.ReadMode_Precision)i("read.precision.mode", (int)p.ReadPrecisionMode);
            p.ReadPrecisionVal = r("read.precision.val", p.ReadPrecisionVal);
            p.ReadMaxPrecisionMode = (Parameters.ReadMode_MaxPrecision)i("read.maxprecision.mode", (int)p.ReadMaxPrecisionMode);
            p.ReadMaxPrecisionVal = r("read.maxprecision.val", p.ReadMaxPrecisionVal);
            p.ReadSameParamMode = b("read.stdsameparameter.mode", p.ReadSameParamMode);
            p.ReadSurfaceCurveMode = (Parameters.ReadMode_SurfaceCurve)i("read.surfacecurve.mode", (int)p.ReadSurfaceCurveMode);
            p.EncodeRegAngle = r("read.encoderegularity.angle", p.EncodeRegAngle);
            p.AngleUnit = (Parameters.AngleUnitMode)i("angleunit.mode", (int)p.AngleUnit);
            p.ReadProductMode = b("read.product.mode", p.ReadProductMode);
            p.ReadProductContext = (Parameters.ReadMode_ProductContext)i("read.product.context", (int)p.ReadProductContext);
            p.ReadShapeRepr = (Parameters.ReadMode_ShapeRepr)i("read.shape.repr", (int)p.ReadShapeRepr);
            p.ReadTessellated = (Parameters.RWMode_Tessellated)i("read.tessellated", (int)p.ReadTessellated);
            p.ReadAssemblyLevel = (Parameters.ReadMode_AssemblyLevel)i("read.assembly.level", (int)p.ReadAssemblyLevel);
            p.ReadRelationship = b("read.shape.relationship", p.ReadRelationship);
            p.ReadShapeAspect = b("read.shape.aspect", p.ReadShapeAspect);
            p.ReadConstrRelation = b("read.constructivegeom.relationship", p.ReadConstrRelation);
            p.ReadSubshapeNames = b("read.stepcaf.subshapes.name", p.ReadSubshapeNames);
            p.ReadCodePage = (FormatType)i("read.codepage", (int)p.ReadCodePage);
            p.ReadNonmanifold = b("read.nonmanifold", p.ReadNonmanifold);
            p.ReadIdeas = b("read.ideas", p.ReadIdeas);
            p.ReadAllShapes = b("read.all.shapes", p.ReadAllShapes);
            p.ReadRootTransformation = b("read.root.transformation", p.ReadRootTransformation);
            p.ReadColor = b("read.color", p.ReadColor);
            p.ReadName = b("read.name", p.ReadName);
            p.ReadLayer = b("read.layer", p.ReadLayer);
            p.ReadProps = b("read.props", p.ReadProps);
            p.ReadMetadata = b("read.metadata", p.ReadMetadata);
            p.ReadProductMetadata = b("read.productmetadata", p.ReadProductMetadata);
            p.WritePrecisionMode = (Parameters.WriteMode_PrecisionMode)i("write.precision.mode", (int)p.WritePrecisionMode);
            p.WritePrecisionVal = r("write.precision.val", p.WritePrecisionVal);
            p.WriteAssembly = (Parameters.WriteMode_Assembly)i("write.assembly", (int)p.WriteAssembly);
            p.WriteSchema = (Parameters.WriteMode_StepSchema)i("write.schema", (int)p.WriteSchema);
            p.WriteTessellated = (Parameters.RWMode_Tessellated)i("write.tessellated", (int)p.WriteTessellated);
            p.WriteProductName = theResource.StringVal("write.product.name", p.WriteProductName, aScope);
            p.WriteSurfaceCurMode = b("write.surfacecurve.mode", p.WriteSurfaceCurMode);
            p.WriteUnit = (global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit)i("write.unit", (int)p.WriteUnit);
            p.WriteVertexMode = (Parameters.WriteMode_VertexMode)i("write.vertex.mode", (int)p.WriteVertexMode);
            p.WriteSubshapeNames = b("write.stepcaf.subshapes.name", p.WriteSubshapeNames);
            p.WriteColor = b("write.color", p.WriteColor);
            p.WriteNonmanifold = b("write.nonmanifold", p.WriteNonmanifold);
            p.WriteName = b("write.name", p.WriteName);
            p.WriteLayer = b("write.layer", p.WriteLayer);
            p.WriteProps = b("write.props", p.WriteProps);
            p.WriteMetadata = b("write.metadata", p.WriteMetadata);
            p.WriteMaterial = b("write.material", p.WriteMaterial);
            p.WriteVisMaterial = b("write.vismaterial", p.WriteVisMaterial);
            p.WriteModelType = (ModelType)i("write.model.type", (int)p.WriteModelType);
            p.CleanDuplicates = b("write.cleanduplicates", p.CleanDuplicates);
            p.WriteScalingTrsf = b("write.scaling.trsf", p.WriteScalingTrsf);
        }

        // DE_Wrapper::FindReadProvider (the content checked too) and FindWriteProvider (the extension only), over the
        // nodes OCCT registers in DRAWEXE
        private static Format find(DrawInterpreter theDI, string thePath, bool theToImport)
        {
            string anExt = Path.GetExtension(thePath).TrimStart('.').ToLowerInvariant();
            Format aByExt = anExt switch
            {
                "stp" or "step" or "stpz" => Format.Step,
                "stl" => Format.Stl,
                "gltf" or "glb" => Format.Gltf,
                "brep" => Format.Brep,
                "igs" or "iges" or "ply" or "vrml" or "wrl" or "xbf" => Format.Unsupported,
                "obj" => Format.Unsupported, // OCCT's OBJ reader isn't ported; Draw-lite's WriteObj has no provider here
                _ => Format.None,
            };
            if (!theToImport)
            {
                return aByExt;
            }
            // DE_ValidationUtils::CreateContentBuffer: the file must be readable
            if (!theDI.Files.TryReadBytes(thePath, out byte[] aBytes))
            {
                return Format.None;
            }
            if (aByExt != Format.None)
            {
                return aByExt;
            }
            string aHead = Encoding.Latin1.GetString(aBytes, 0, Math.Min(aBytes.Length, 2048));
            if (aBytes.Length >= 100 && !aHead.Contains("IFC", StringComparison.Ordinal)
                && aHead.Contains("ISO-10303-21", StringComparison.Ordinal) && aHead.Contains("FILE_SCHEMA", StringComparison.Ordinal))
            {
                return Format.Step;
            }
            if (aBytes.Length >= 20 && (aHead.Contains("DBRep_DrawableShape", StringComparison.Ordinal)
                                        || aHead.Contains("CASCADE Topology V1", StringComparison.Ordinal)
                                        || aHead.Contains("CASCADE Topology V3", StringComparison.Ordinal)))
            {
                return Format.Brep;
            }
            if (aBytes.Length >= 7 && (aHead.StartsWith("solid", StringComparison.Ordinal) || aHead.StartsWith("SOLID", StringComparison.Ordinal))
                && char.IsWhiteSpace(aHead[5]))
            {
                return Format.Stl;
            }
            return Format.None;
        }

        private static bool unsupported(DrawInterpreter theDI, string thePath, Format theFormat)
            => fail(theDI, theFormat == Format.None
                               ? "Error: no provider is registered for the file " + thePath
                               : "Error: Draw-lite has no provider for the format of the file " + thePath);

        // DE_Wrapper::Read(path, document, session)
        public bool Read(DrawInterpreter theDI, string thePath, string theName, ExchangeDocument theDocument)
            => find(theDI, thePath, true) switch
            {
                Format.Step => readStep(theDI, thePath, theDocument),
                Format.Stl => readStl(theDI, thePath, out Shape aShape) && addShape(theDocument, aShape),
                Format.Gltf => readGltf(theDI, thePath, theName, theDocument),
                Format.Brep => readBrep(theDI, thePath, out Shape aShape) && addShape(theDocument, aShape),
                Format aFormat => unsupported(theDI, thePath, aFormat),
            };

        // DE_Wrapper::Read(path, shape, session)
        public bool Read(DrawInterpreter theDI, string thePath, out Shape theShape)
        {
            theShape = new Shape();
            switch (find(theDI, thePath, true))
            {
                case Format.Step:
                    return readStep(theDI, thePath, out theShape);
                case Format.Stl:
                    return readStl(theDI, thePath, out theShape);
                case Format.Gltf:
                    return readGltf(theDI, thePath, out theShape);
                case Format.Brep:
                    return readBrep(theDI, thePath, out theShape);
                case var aFormat:
                    return unsupported(theDI, thePath, aFormat);
            }
        }

        // DE_Wrapper::Write(path, document, session)
        public bool Write(DrawInterpreter theDI, string thePath, ExchangeDocument theDocument)
            => find(theDI, thePath, false) switch
            {
                Format.Step => writeStep(theDI, thePath, theDocument),
                Format.Stl => freeShape(theDI, thePath, theDocument, "DESTL_Provider", out Shape aShape) && writeStl(theDI, thePath, aShape),
                Format.Gltf => writeGltf(theDI, thePath, theDocument),
                Format.Brep => freeShape(theDI, thePath, theDocument, "DEBREP_Provider", out Shape aShape) && writeBrep(theDI, thePath, aShape),
                Format aFormat => unsupported(theDI, thePath, aFormat),
            };

        // DE_Wrapper::Write(path, shape, session)
        public bool Write(DrawInterpreter theDI, string thePath, Shape theShape)
        {
            switch (find(theDI, thePath, false))
            {
                case Format.Step:
                    return writeStep(theDI, thePath, theShape);
                case Format.Stl:
                    return writeStl(theDI, thePath, theShape);
                case Format.Gltf:
                {
                    // DEGLTF_Provider::Write(shape): a document holding the shape
                    var aDoc = new ExchangeDocument();
                    aDoc.AddShape(theShape);
                    return writeGltf(theDI, thePath, aDoc);
                }
                case Format.Brep:
                    return writeBrep(theDI, thePath, theShape);
                case var aFormat:
                    return unsupported(theDI, thePath, aFormat);
            }
        }

        private static bool addShape(ExchangeDocument theDocument, Shape theShape)
        {
            theDocument.AddShape(theShape);
            return true;
        }

        // DESTL_Provider and DEBREP_Provider::Write(document): the free shape, or a compound of the free shapes
        private static bool freeShape(DrawInterpreter theDI, string thePath, ExchangeDocument theDocument, string theProvider, out Shape theShape)
        {
            theShape = new Shape();
            var aLabels = new NCollection_Sequence<Label>();
            theDocument.GetFreeShapes(aLabels);
            if (aLabels.Length() <= 0)
            {
                return fail(theDI, "Error in the " + theProvider + " during writing the file " + thePath + "\t: Document contain no shapes");
            }
            if (aLabels.Length() == 1)
            {
                theShape = theDocument.GetShape(aLabels.Value(1));
            }
            else
            {
                Compound aComp = new();
                var aBuilder = new BrepBuilder();
                aBuilder.MakeCompound(ref aComp);
                for (int anIndex = 1; anIndex <= aLabels.Length(); anIndex++)
                {
                    aBuilder.Add(aComp, theDocument.GetShape(aLabels.Value(anIndex)));
                }
                theShape = aComp;
            }
            return true;
        }

        // ---------------------------------------------------------------------------------------------------------
        // DESTEP_Provider, on the interpreter's session (personizeWS: the readers and writers give it STEPCAFControl's
        // controller, decision 1277)

        private bool readStep(DrawInterpreter theDI, string thePath, ExchangeDocument theDocument)
        {
            global::Kerf.Engine.Exchange.Framework.ExchangeWorkSession aWS = XSDRAWCommands.Session(theDI);
            var aReader = new CafReader(aWS, true);
            // configureSTEPCAFReader
            aReader.SetColorMode(myStep.ReadColor);
            aReader.SetNameMode(myStep.ReadName);
            aReader.SetLayerMode(myStep.ReadLayer);
            aReader.SetPropsMode(myStep.ReadProps);
            aReader.SetMetaMode(myStep.ReadMetadata);
            aReader.SetProductMetaMode(myStep.ReadProductMetadata);
            aReader.SetShapeFixParameters(myStepShapeFix, new Dictionary<string, string>());
            theDocument.SetLengthUnit(myLengthUnit / 1000.0, global::Kerf.Engine.Exchange.Framework.UnitsMethods.DumpLengthUnit(myLengthUnit, global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit.LengthUnit_Millimeter));

            ReturnStatus aReadStat = theDI.Files.TryReadBytes(thePath, out byte[] aBytes)
                                                  ? aReader.ChangeReader().ReadStream(thePath, myStep, new MemoryStream(aBytes))
                                                  : ReturnStatus.RetError;
            if (aReadStat != ReturnStatus.RetDone)
            {
                return fail(theDI, "Error in the DESTEP_Provider during reading the file " + thePath + "\t: abandon");
            }
            if (!aReader.Transfer(theDocument))
            {
                return fail(theDI, "Error in the DESTEP_Provider during reading the file " + thePath + "\t: Cannot read any relevant data from the STEP file");
            }
            return true;
        }

        private bool readStep(DrawInterpreter theDI, string thePath, out Shape theShape)
        {
            theShape = new Shape();
            global::Kerf.Engine.Exchange.Framework.ExchangeWorkSession aWS = XSDRAWCommands.Session(theDI);
            StepController.SelectFor(aWS);
            var aReader = new StepReader();
            aReader.SetWS(aWS);
            aReader.SetShapeFixParameters(myStepShapeFix);
            ReturnStatus aReadstat = theDI.Files.TryReadBytes(thePath, out byte[] aBytes)
                                                  ? aReader.ReadStream(thePath, myStep, new MemoryStream(aBytes))
                                                  : ReturnStatus.RetError;
            if (aReadstat != ReturnStatus.RetDone)
            {
                return fail(theDI, "Error in the DESTEP_Provider during reading the file " + thePath + "\t: abandon, no model loaded");
            }
            aReader.StepModel().SetLocalLengthUnit(myLengthUnit);
            if (aReader.TransferRoots() <= 0)
            {
                return fail(theDI, "Error in the DESTEP_Provider during reading the file " + thePath + "\t:Cannot read any relevant data from the STEP file");
            }
            theShape = aReader.OneShape();
            return true;
        }

        private bool writeStep(DrawInterpreter theDI, string thePath, ExchangeDocument theDocument)
        {
            global::Kerf.Engine.Exchange.Framework.ExchangeWorkSession aWS = XSDRAWCommands.Session(theDI);
            var aWriter = new CafWriter(aWS, true);
            // configureSTEPCAFWriter
            aWriter.SetColorMode(myStep.WriteColor);
            aWriter.SetNameMode(myStep.WriteName);
            aWriter.SetLayerMode(myStep.WriteLayer);
            aWriter.SetPropsMode(myStep.WriteProps);
            aWriter.SetMaterialMode(myStep.WriteMaterial);
            aWriter.SetVisualMaterialMode(myStep.WriteVisMaterial);
            aWriter.SetCleanDuplicates(myStep.CleanDuplicates);
            aWriter.SetShapeFixParameters(myStepShapeFix, new Dictionary<string, string>());
            var aModel = (StepModel)aWriter.Writer().WS()!.Model()!;
            if (theDocument.GetLengthUnit(out double aUnitM))
            {
                aModel.SetLocalLengthUnit(aUnitM * 1000.0);
            }
            else
            {
                aModel.SetLocalLengthUnit(mySystemUnit);
                warn(theDI, "Warning in the DESTEP_Provider during writing\t: The document has no information on Units. Using global parameter as initial Unit.");
            }

            ModelType aMode = myStep.WriteModelType;
            Parameters aParams = myStep;
            aParams.WriteUnit = global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetLengthUnitByFactorValue(myLengthUnit, global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit.LengthUnit_Millimeter);
            aModel.SetWriteLengthUnit(myLengthUnit);
            if (!aWriter.Transfer(theDocument, aParams, aMode))
            {
                return fail(theDI, "Error in the DESTEP_Provider during writing the file " + thePath + "\t: The document cannot be translated or gives no result");
            }
            ReturnStatus aStatus = XSDRAWSTEPDocCommands.Write(theDI, aWriter, thePath);
            return aStatus switch
            {
                ReturnStatus.RetVoid => fail(theDI, "Error in the DESTEP_Provider during writing the file " + thePath + "\t: No file written"),
                ReturnStatus.RetDone => true,
                _ => fail(theDI, "Error in the DESTEP_Provider during writing the file " + thePath + "\t: Error on writing file"),
            };
        }

        private bool writeStep(DrawInterpreter theDI, string thePath, Shape theShape)
        {
            global::Kerf.Engine.Exchange.Framework.ExchangeWorkSession aWS = XSDRAWCommands.Session(theDI);
            StepController.SelectFor(aWS);
            var aWriter = new Writer();
            aWriter.SetWS(aWS);
            StepModel aModel = aWriter.Model()!;
            Parameters aParams = myStep;
            aModel.SetLocalLengthUnit(mySystemUnit);
            global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit aTargetUnit = global::Kerf.Engine.Exchange.Framework.UnitsMethods.GetLengthUnitByFactorValue(myLengthUnit, global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit.LengthUnit_Millimeter);
            aParams.WriteUnit = aTargetUnit;
            if (aTargetUnit == global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit.LengthUnit_Undefined)
            {
                aModel.SetWriteLengthUnit(1.0);
                warn(theDI, "Custom units are not supported by STEP format, but LengthUnit global parameter doesn't fit any predefined unit. Units will be scaled to Millimeters");
            }
            else
            {
                aModel.SetWriteLengthUnit(myLengthUnit);
            }
            aWriter.SetShapeFixParameters(myStepShapeFix);
            if (aWriter.Transfer(theShape, myStep.WriteModelType, aParams, true) != ReturnStatus.RetDone)
            {
                return fail(theDI, "Error in the DESTEP_Provider during reading the file " + thePath + "\t: abandon, no model loaded");
            }
            if (myStep.CleanDuplicates)
            {
                aWriter.CleanDuplicateEntities();
            }
            if (XSDRAWSTEPCommands.Write(theDI, aWriter, thePath) != ReturnStatus.RetDone)
            {
                return fail(theDI, "DESTEP_Provider: Error on writing file");
            }
            return true;
        }

        // ---------------------------------------------------------------------------------------------------------
        // DESTL_Provider

        private bool readStl(DrawInterpreter theDI, string thePath, out Shape theShape)
        {
            theShape = new Shape();
            warn(theDI, "OCCT Stl reader does not support model scaling according to custom length unit");
            double aMergeAngle = myStlMergeAngle * Math.PI / 180.0;
            if (aMergeAngle != Math.PI / 2 && (aMergeAngle < 0.0 || aMergeAngle > Math.PI / 2))
            {
                return fail(theDI, "Error in the DESTL_Provider during reading the file " + thePath + "\t: The merge angle is out of the valid range");
            }
            if (!theDI.Files.TryReadBytes(thePath, out byte[] aBytes))
            {
                // RWStl::ReadFile gives no triangulation, StlAPI::Read fails
                if (myStlReadBRep)
                {
                    return fail(theDI, "Error in the DESTL_Provider during reading the file " + thePath);
                }
                aBytes = [];
            }
            if (!myStlReadBRep)
            {
                Triangulation? aTriangulation = aBytes.Length > 0 ? global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(aBytes), aMergeAngle) : null;
                Face aFace = new();
                var aB = new BrepBuilder();
                aB.MakeFace(ref aFace);
                aB.UpdateFace(aFace, aTriangulation);
                theShape = aFace;
            }
            else if (!new StlApiReader().Read(ref theShape, new MemoryStream(aBytes))) // StlAPI::Read
            {
                return fail(theDI, "Error in the DESTL_Provider during reading the file " + thePath);
            }
            return true;
        }

        private bool writeStl(DrawInterpreter theDI, string thePath, Shape theShape)
        {
            warn(theDI, "OCCT Stl writer does not support model scaling according to custom length unit");
            if (myLengthUnit != 1.0)
            {
                // DE_ValidationUtils::WarnLengthUnitNotSupported
                warn(theDI, "Warning during writing the file " + thePath + ": Format doesn't support custom units. Model won't be scaled");
            }
            var aWriter = new ApiWriter();
            aWriter.SetASCIIMode(myStlWriteAscii);
            var aStream = new MemoryStream();
            bool isOK = aWriter.Write(theShape, aStream);
            theDI.Files.WriteBytes(thePath, aStream.ToArray());
            return isOK || fail(theDI, "Error in the DESTL_Provider during writing the file " + thePath + "\t: Mesh writing has been failed");
        }

        // ---------------------------------------------------------------------------------------------------------
        // DEGLTF_Provider, over unit 8T's reader and writer (the system unit in metres: the global length unit / 1000;
        // Z-up system, Y-up file, as the node's defaults)

        private bool readGltf(DrawInterpreter theDI, string thePath, string theName, ExchangeDocument theDocument)
        {
            if (theDocument.Children(theDocument.ShapesLabel()).Count > 0)
            {
                return fail(theDI, "Error: Draw-lite reads a glTF file into a new or empty document only");
            }
            double aScaleFactorM = myLengthUnit / 1000.0;
            Tree? aTree = XSDRAWMeshCommands.ReadGltfTree(theDI, thePath, aScaleFactorM);
            if (aTree is null)
            {
                return fail(theDI, "Error in the DEGLTF_Provider during reading the file " + thePath);
            }
            ExchangeDocument aDoc = ExchangeDocument.FromTree(aTree);
            XSDRAWMeshCommands.NameInstances(aDoc);
            // XCAFDoc_DocumentTool::SetLengthUnit before RWMesh_CafReader::Perform
            aDoc.SetLengthUnit(aScaleFactorM, global::Kerf.Engine.Exchange.Framework.UnitsMethods.DumpLengthUnit(myLengthUnit, global::Kerf.Engine.Exchange.Framework.FrameworkLengthUnit.LengthUnit_Millimeter));
            DDocStdCommands.NewDocument(theDI, theName, aDoc);
            return true;
        }

        private bool readGltf(DrawInterpreter theDI, string thePath, out Shape theShape)
        {
            theShape = new Shape();
            Tree? aTree = XSDRAWMeshCommands.ReadGltfTree(theDI, thePath, myLengthUnit / 1000.0);
            if (aTree is null)
            {
                return fail(theDI, "Error in the DEGLTF_Provider during reading the file " + thePath);
            }
            theShape = XDEDRAWCommands.GetOneShape(ExchangeDocument.FromTree(aTree)); // RWMesh_CafReader::SingleShape
            return true;
        }

        private bool writeGltf(DrawInterpreter theDI, string thePath, ExchangeDocument theDocument)
        {
            // RWGltf_CafWriter takes the document's unit; DEGLTF_Provider gives the system unit to a document without one
            double aScaleFactorM;
            if (!theDocument.GetLengthUnit(out aScaleFactorM))
            {
                aScaleFactorM = mySystemUnit / 1000.0;
                warn(theDI, "Warning in the DEGLTF_Provider during writing the file " + thePath
                            + "\t: The document has no information on Units. Using global parameter as initial Unit.");
            }
            if (myLengthUnit != 1000.0)
            {
                warn(theDI, "Warning during writing the file " + thePath + ": Target format doesn't support custom units. Model will be scaled to Meters (unit: "
                            + myLengthUnit.ToString(CultureInfo.InvariantCulture) + ")");
            }
            XSDRAWMeshCommands.WriteGltfTree(theDI, thePath, theDocument.ToTree(), aScaleFactorM, true, myGltfMergeFaces);
            return true;
        }

        // ---------------------------------------------------------------------------------------------------------
        // DEBREP_Provider

        private static bool readBrep(DrawInterpreter theDI, string thePath, out Shape theShape)
        {
            theShape = new Shape();
            if (!theDI.Files.TryReadBytes(thePath, out byte[] aBytes) || aBytes.Length < 255)
            {
                // the 255 bytes read to recognise the format
                return fail(theDI, "Error in the DEBREP_Provider during reading the file " + thePath + "\t: Cannot read the file");
            }
            bool isBinaryFormat = !aBytes.AsSpan().StartsWith("DBRep_DrawableShape"u8);
            if (isBinaryFormat)
            {
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Read(out theShape, new MemoryStream(aBytes, false));
                return true;
            }
            // BRepTools::Read(shape, file): the shape set, then the shape
            string aText = Encoding.Latin1.GetString(aBytes);
            var aReader = new StringReader(aText);
            global::Kerf.Engine.Foundation.StreamUtilities.SetInputLength(aReader, aText.Length);
            var aShapeSet = new BrepShapeSet();
            aShapeSet.Read(aReader);
            if (aShapeSet.ShapeCount == 0)
            {
                return fail(theDI, "Error in the DEBREP_Provider during reading the file " + thePath + "\t: Cannot read from the file");
            }
            aShapeSet.Read(out theShape, aReader);
            return true;
        }

        private bool writeBrep(DrawInterpreter theDI, string thePath, Shape theShape)
        {
            if (myLengthUnit != 1.0)
            {
                warn(theDI, "Warning in the DEBREP_Provider during writing the file " + thePath
                            + "\t: Target Units for writing were changed, but current format doesn't support scaling");
            }
            if (myBrepWriteBinary)
            {
                if (myBrepVersionBin > (global::Kerf.Engine.Exchange.Brep.Binary.BinaryFormatVersion)global::Kerf.Engine.Exchange.Brep.Binary.BinaryFormatVersionBounds.BinTools_FormatVersion_UPPER
                    || myBrepVersionBin < (global::Kerf.Engine.Exchange.Brep.Binary.BinaryFormatVersion)global::Kerf.Engine.Exchange.Brep.Binary.BinaryFormatVersionBounds.BinTools_FormatVersion_LOWER)
                {
                    return fail(theDI, "Error in the DEBREP_Provider during writing the file " + thePath + "\t: Unknown format version");
                }
                if (myBrepWriteNormals && myBrepVersionBin < global::Kerf.Engine.Exchange.Brep.Binary.BinaryFormatVersion.FormatVersion_VERSION_4)
                {
                    return fail(theDI, "Error in the DEBREP_Provider during writing the file " + thePath
                                       + "\t: Vertex normals require binary format version 4 or later");
                }
                var aStream = new MemoryStream();
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Write(theShape, aStream, myBrepWriteTriangles, myBrepWriteNormals, myBrepVersionBin);
                theDI.Files.WriteBytes(thePath, aStream.ToArray());
            }
            else
            {
                if (myBrepVersionAscii > (FormatVersion)FormatVersionBounds.TopTools_FormatVersion_UPPER
                    || myBrepVersionAscii < (FormatVersion)FormatVersionBounds.TopTools_FormatVersion_LOWER)
                {
                    return fail(theDI, "Error in the DEBREP_Provider during writing the file " + thePath + "\t: Unknown format version");
                }
                if (myBrepWriteNormals && myBrepVersionAscii < FormatVersion.FormatVersion_VERSION_3)
                {
                    return fail(theDI, "Error in the DEBREP_Provider during writing the file " + thePath
                                       + "\t: Error: vertex normals require ascii format version 3 or later");
                }
                // BRepTools::Write(shape, file): without Draw's "DBRep_DrawableShape" line
                var aStream = new StringWriter { NewLine = "\n" };
                BrepTools.Write(theShape, aStream, myBrepWriteTriangles, myBrepWriteNormals, myBrepVersionAscii);
                theDI.Files.WriteBytes(thePath, Encoding.Latin1.GetBytes(aStream.ToString()));
            }
            return true;
        }
    }
}
