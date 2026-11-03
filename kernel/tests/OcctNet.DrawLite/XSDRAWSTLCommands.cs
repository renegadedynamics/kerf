// Port of: src/Draw/TKXSDRAWSTL/XSDRAWSTL/XSDRAWSTL.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2023-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// XSDRAWSTL's STL commands (unit 8H1): <c>writestl</c> (StlAPI_Writer) and <c>readstl</c> (RWStl, or StlAPI with
/// <c>-brep</c>). The MeshVS commands of the file (<c>meshfromstl</c> and the others) need the viewer and are left
/// out, as is the DE plugin registration (DESTL needs TKDE's framework).
/// </summary>
/// <remarks>
/// Draw-lite's files are its in-memory overlay (TclFileSystem): <c>writestl</c> writes the bytes the stream writer
/// makes there (LF line ends, where DRAWEXE on Windows writes ASCII STL in text mode, CR LF: decision 1168), and
/// <c>readstl</c> reads through the library's stream forms of <c>RWStl::ReadFile</c>, which read as the file forms do.
/// The messages the library sends go to the command's result while it runs, as DRAWEXE prints them (unit 8W: before
/// it, to the process-wide messenger, which Draw-lite leaves empty); the one about a missing file, which the stream
/// forms don't send, is written to the interpreter's output here (decision 405). The progress indicators are left out.
/// </remarks>
internal static class XSDRAWSTLCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        // the library's messages to the command's result, as DRAWEXE's Draw_Printer (decision 1275; unit 8W)
        XSDRAWCommands.Pilot(theInterpreter, "writestl", writestl);
        XSDRAWCommands.Pilot(theInterpreter, "readstl", readstl);
    }

    // writestl shape file [ascii/binary (0/1) : 1 by default]
    private static int writestl(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 3 || argc > 4)
        {
            di.Append("Use: ").Append(argv[0]).Append(" shape file [ascii/binary (0/1) : 1 by default]\n");
        }
        else
        {
            Shape aShape = theDI.GetShape(argv[1]);
            bool isASCIIMode = false;
            if (argc == 4)
            {
                isASCIIMode = theDI.Atoi(argv[3]) == 0;
            }
            var aWriter = new StlAPI_Writer();
            aWriter.SetASCIIMode(isASCIIMode);
            var aStream = new MemoryStream();
            bool isOK = aWriter.Write(aShape, aStream);
            // std::ofstream creates the file before writing anything
            theDI.Files.WriteBytes(argv[2], aStream.ToArray());
            if (!isOK)
            {
                di.Append("** Error **: Mesh writing has been failed.\n");
            }
        }
        return 0;
    }

    // readstl shape file [-brep] [-mergeAngle Angle] [-multi]
    private static int readstl(DrawInterpreter theDI, DrawResult theResult, IReadOnlyList<string> theArgv)
    {
        int theArgc = theArgv.Count;
        string aShapeName = "", aFilePath = "";
        bool toCreateCompOfTris = false;
        bool anIsMulti = false;
        double aMergeAngle = Math.PI / 2.0;
        for (int anArgIter = 1; anArgIter < theArgc; ++anArgIter)
        {
            string anArg = theArgv[anArgIter].ToLowerInvariant();
            if (aShapeName.Length == 0)
            {
                aShapeName = theArgv[anArgIter];
            }
            else if (aFilePath.Length == 0)
            {
                aFilePath = theArgv[anArgIter];
            }
            else if (anArg == "-brep")
            {
                toCreateCompOfTris = true;
                if (anArgIter + 1 < theArgc && DBRepCommands.ParseOnOff(theArgv[anArgIter + 1], ref toCreateCompOfTris))
                {
                    ++anArgIter;
                }
            }
            else if (anArg == "-multi")
            {
                anIsMulti = true;
                if (anArgIter + 1 < theArgc && DBRepCommands.ParseOnOff(theArgv[anArgIter + 1], ref anIsMulti))
                {
                    ++anArgIter;
                }
            }
            else if (anArg is "-mergeangle" or "-smoothangle" or "-nomergeangle" or "-nosmoothangle")
            {
                if (anArg.StartsWith("-no", StringComparison.Ordinal))
                {
                    aMergeAngle = Math.PI / 2.0;
                }
                else
                {
                    aMergeAngle = Math.PI / 4.0;
                    if (anArgIter + 1 < theArgc && theDI.ParseReal(theArgv[anArgIter + 1], out double anAngle))
                    {
                        aMergeAngle = anAngle;
                        if (aMergeAngle < 0.0 || aMergeAngle > 90.0)
                        {
                            theResult.Append("Syntax error: angle should be within [0,90] range");
                            return 1;
                        }

                        ++anArgIter;
                        aMergeAngle = aMergeAngle * Math.PI / 180.0;
                    }
                }
            }
            else
            {
                theDI.Output.WriteLine("Syntax error: unknown argument '" + theArgv[anArgIter] + "'");
                return 1;
            }
        }
        if (aFilePath.Length == 0)
        {
            theDI.Output.WriteLine("Syntax error: not enough arguments");
            return 1;
        }

        // RWStl_Reader::Read's message for a file that can't be opened (the library's goes to the empty messenger)
        bool isFile = theDI.Files.TryReadBytes(aFilePath, out byte[] aBytes);
        if (!isFile)
        {
            theDI.Output.WriteLine("Error: file '" + aFilePath + "' is not found");
        }

        Shape aShape = new();
        if (!toCreateCompOfTris)
        {
            if (anIsMulti)
            {
                var aTriangList = new NCollection_Sequence<Triangulation>();
                // Read STL file to the triangulation list.
                if (isFile)
                {
                    RWStl.ReadFile(new MemoryStream(aBytes), aMergeAngle, aTriangList);
                }
                var aB = new BrepBuilder();
                Face aFace = new();
                if (aTriangList.Size() == 1)
                {
                    aB.MakeFace(ref aFace);
                    aB.UpdateFace(aFace, aTriangList.First());
                    aShape = aFace;
                }
                else
                {
                    Compound aCmp = new();
                    aB.MakeCompound(ref aCmp);

                    for (int i = 1; i <= aTriangList.Length(); ++i)
                    {
                        aFace = new Face();
                        aB.MakeFace(ref aFace);
                        aB.UpdateFace(aFace, aTriangList.Value(i));
                        aB.Add(aCmp, aFace);
                    }
                    aShape = aCmp;
                }
            }
            else
            {
                // Read STL file to the triangulation.
                Triangulation? aTriangulation = isFile ? RWStl.ReadFile(new MemoryStream(aBytes), aMergeAngle) : null;

                Face aFace = new();
                var aB = new BrepBuilder();
                aB.MakeFace(ref aFace);
                aB.UpdateFace(aFace, aTriangulation);
                aShape = aFace;
            }
        }
        else if (isFile)
        {
            // StlAPI::Read
            Triangulation? aMesh = RWStl.ReadFile(new MemoryStream(aBytes));
            if (aMesh is not null)
            {
                var aConverter = new BRepBuilderAPI_MakeShapeOnMesh(aMesh);
                aConverter.Build();
                if (aConverter.IsDone() && !aConverter.Shape().IsNull)
                {
                    aShape = aConverter.Shape();
                }
            }
        }
        theDI.SetShape(aShapeName, aShape);
        return 0;
    }
}
