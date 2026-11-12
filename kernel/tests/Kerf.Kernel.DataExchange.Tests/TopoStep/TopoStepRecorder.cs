// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Export;
using Kerf.Engine.Exchange.Step.Import;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Exchange.Step.Visual;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Globalization;
using System.Security.Cryptography;
using System.Text;
using Kerf.Kernel.DataExchange.Tests.StepGen;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Exchange.Step.Export.FacetedError;

namespace Kerf.Kernel.DataExchange.Tests.TopoStep;

/// <summary>
/// The port's twin of tools/toporec (unit 8E): the same lines, for StepToTopoDS_Builder on a model's shape
/// representation items ("s2t") and for TopoDSToStep's makers on a shape ("t2s").
/// </summary>
internal static class TopoStepRecorder
{
    public static readonly Factors Millimetres = new();
    public static readonly Factors Inches = new Factors().InitializedFactors(25.4, M_PI / 180.0, 1.0);

    /// <summary>StepAP214's protocol (unit 8C2), as native's recorder reads and writes with.</summary>
    internal static StepProtocol Protocol() => StepRoundTripTests.Protocol();

    internal static string Sha256(string theText) => System.Convert.ToHexStringLower(SHA256.HashData(Encoding.Latin1.GetBytes(theText)));

    // ---------------------------------------------------------------------------------------------------------
    // StepToTopoDS

    private static string Checks(TransientProcess theTP)
    {
        int aNbW = 0, aNbF = 0;
        CheckIterator aList = theTP.CheckList(false);
        for (aList.Start(); aList.More(); aList.Next())
        {
            aNbW += aList.Value()!.WarningCount;
            aNbF += aList.Value()!.FailCount;
        }

        return " W" + aNbW + " F" + aNbF;
    }

    // The item's translation: "NOTDONE", "EXC" or the BRep text's hash and length (null: not an item translated)
    private static string? Translate(Transient theEnt, TransientProcess theTP, out string? theText)
    {
        theText = null;
        ImportBuilder? aBuilder = Build(theEnt, theTP);
        if (aBuilder is null)
        {
            return null;
        }

        if (!aBuilder.IsDone)
        {
            return "NOTDONE";
        }

        var aS = new StringWriter { NewLine = "\n" };
        BrepTools.Write(aBuilder.Value(), aS);
        theText = aS.ToString();
        return Sha256(theText) + " " + theText.Length;
    }

    /// <summary>
    /// The hash and length of the item's BRep text, <paramref name="theText"/>, with the triangles of its complex
    /// triangulated faces and surface sets turned the other way, their second and third nodes exchanged: how OCCT 8.0.1
    /// winds a strip's and a fan's triangles, against their lists (decision 33105). Null where the item has none, or its
    /// triangulations can't be told apart.
    /// </summary>
    public static string? WoundAsNative(Transient theEnt, string theText)
    {
        List<bool>? aComplex = theEnt switch
        {
            ComplexTriangulatedFace or ComplexTriangulatedSurfaceSet => [true],
            TessellatedShell aShell => ComplexFaces(aShell.Items()),
            TessellatedSolid aSolid => ComplexFaces(aSolid.Items()),
            _ => null,
        };
        if (aComplex is null || !aComplex.Contains(true))
        {
            return null;
        }

        // The text's triangulations, one for each face in the faces' order: a line of the counts of nodes and
        // triangles, whether there are UV nodes and normals, and the deflection; then a line of the nodes, the UV
        // nodes, the triangles and the normals
        string[] aLines = theText.Split('\n');
        int aHead = Array.FindIndex(aLines, l => l.StartsWith("Triangulations ", StringComparison.Ordinal));
        if (aHead < 0 || int.Parse(aLines[aHead].Split(' ')[1], CultureInfo.InvariantCulture) != aComplex.Count)
        {
            return null;
        }

        for (int k = 0; k < aComplex.Count; k++)
        {
            string[] aCounts = aLines[aHead + 1 + 2 * k].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            string[] aData = aLines[aHead + 2 + 2 * k].Split(' ');
            int aNbNodes = int.Parse(aCounts[0], CultureInfo.InvariantCulture), aNbTriangles = int.Parse(aCounts[1], CultureInfo.InvariantCulture);
            int aFirst = aNbNodes * (aCounts[2] == "1" ? 5 : 3);
            if (aCounts.Length != 5 || aData.Count(d => d.Length > 0) != aFirst + 3 * aNbTriangles + (aCounts[3] == "1" ? 3 * aNbNodes : 0))
            {
                return null;
            }

            if (!aComplex[k])
            {
                continue;
            }

            for (int aTriangle = 0; aTriangle < aNbTriangles; aTriangle++)
            {
                (aData[aFirst + 3 * aTriangle + 1], aData[aFirst + 3 * aTriangle + 2]) = (aData[aFirst + 3 * aTriangle + 2], aData[aFirst + 3 * aTriangle + 1]);
            }

            aLines[aHead + 2 + 2 * k] = string.Join(' ', aData);
        }

        string aText = string.Join('\n', aLines);
        return Sha256(aText) + " " + aText.Length;
    }

    /// <summary>
    /// How many triangles of the model's complex triangulated faces and surface sets that carry normals turn the way their
    /// nodes' normals point, and how many turn against them.
    /// </summary>
    public static (int With, int Against) Windings(StepModel theModel)
    {
        theModel.InternalParameters = new Parameters();
        int aWith = 0, anAgainst = 0;
        for (int i = 1; i <= theModel.EntityCount; i++)
        {
            Transient anEnt = theModel.Value(i)!;
            if (anEnt is not (ComplexTriangulatedFace or ComplexTriangulatedSurfaceSet))
            {
                continue;
            }

            var aTP = new TransientProcess(theModel.EntityCount);
            aTP.SetModel(theModel);
            ImportBuilder? aBuilder = Build(anEnt, aTP);
            if (aBuilder is null || !aBuilder.IsDone)
            {
                continue;
            }

            for (var anExp = new Explorer(aBuilder.Value(), ShapeType.Face); anExp.More(); anExp.Next())
            {
                if (Kerf.Engine.Topology.Brep.Tool.Triangulation(ShapeCasts.Face(anExp.Current()), out _) is not { HasNormals: true } aMesh)
                {
                    continue;
                }

                for (int aTriangle = 1; aTriangle <= aMesh.TriangleCount; aTriangle++)
                {
                    aMesh.Triangle(aTriangle).Get(out int aN1, out int aN2, out int aN3);
                    var (aP1, aP2, aP3) = (aMesh.Node(aN1), aMesh.Node(aN2), aMesh.Node(aN3));
                    double aUx = aP2.X - aP1.X, aUy = aP2.Y - aP1.Y, aUz = aP2.Z - aP1.Z;
                    double aVx = aP3.X - aP1.X, aVy = aP3.Y - aP1.Y, aVz = aP3.Z - aP1.Z;
                    double aNx = aUy * aVz - aUz * aVy, aNy = aUz * aVx - aUx * aVz, aNz = aUx * aVy - aUy * aVx;
                    double aDot = 0;
                    foreach (int aNode in (int[])[aN1, aN2, aN3])
                    {
                        var aNormal = aMesh.Normal(aNode);
                        aDot += aNx * aNormal.X + aNy * aNormal.Y + aNz * aNormal.Z;
                    }

                    if (aDot > 0)
                    {
                        aWith++;
                    }
                    else if (aDot < 0)
                    {
                        anAgainst++;
                    }
                }
            }
        }

        return (aWith, anAgainst);
    }

    // Which of a tessellated shell's or solid's faces, in their order, are complex triangulated ones
    private static List<bool> ComplexFaces(NCollection_HArray1<TessellatedStructuredItem> theItems)
    {
        var aRes = new List<bool>();
        for (int i = theItems.Lower(); i <= theItems.Upper(); i++)
        {
            if (theItems.Value(i) is TessellatedFace aFace)
            {
                aRes.Add(aFace is ComplexTriangulatedFace);
            }
        }

        return aRes;
    }

    // The builder an item is translated with, initialised with it (null: not an item translated)
    private static ImportBuilder? Build(Transient theEnt, TransientProcess theTP)
    {
        var aBuilder = new ImportBuilder();
        var aFactors = new Factors();
        bool aHasGeom = false;
        switch (theEnt)
        {
            case StepFacetedBrepAndBrepWithVoids e:
                aBuilder.Init(e, theTP, aFactors);
                break;
            case StepBrepWithVoids e:
                aBuilder.Init(e, theTP, aFactors);
                break;
            case StepFacetedBrep e:
                aBuilder.Init(e, theTP, aFactors);
                break;
            case StepManifoldSolidBrep e:
                aBuilder.Init(e, theTP, aFactors);
                break;
            case StepShellBasedSurfaceModel e:
                aBuilder.Init(e, theTP, new NMTool(), aFactors);
                break;
            case StepEdgeBasedWireframeModel e:
                aBuilder.Init(e, theTP, aFactors);
                break;
            case StepFaceBasedSurfaceModel e:
                aBuilder.Init(e, theTP, aFactors);
                break;
            case StepGeometricSet e:
                aBuilder.Init(e, theTP, aFactors);
                break;
            case TessellatedSolid e:
                aBuilder.Init(e, theTP, false, ref aHasGeom, aFactors);
                break;
            case TessellatedShell e:
                aBuilder.Init(e, theTP, false, ref aHasGeom, aFactors);
                break;
            case TessellatedFace e:
                aBuilder.Init(e, theTP, false, ref aHasGeom, aFactors);
                break;
            case TessellatedSurfaceSet e:
                aBuilder.Init(e, theTP, ref aHasGeom, aFactors);
                break;
            default:
                return null;
        }

        return aBuilder;
    }

    /// <summary>The lines of a model's items ("I ..."), and the BRep texts by entity number.</summary>
    public static List<string> ModelLines(StepModel theModel, Dictionary<int, string>? theTexts = null)
    {
        theModel.InternalParameters = new Parameters();
        var aLines = new List<string>();
        for (int i = 1; i <= theModel.EntityCount; i++)
        {
            Transient anEnt = theModel.Value(i)!;
            var aTP = new TransientProcess(theModel.EntityCount);
            aTP.SetModel(theModel);
            string? aRes, aText = null;
            try
            {
                aRes = Translate(anEnt, aTP, out aText);
            }
            catch (Exception anException) when (anException is not OutOfMemoryException)
            {
                aRes = "EXC";
            }

            if (aRes is null)
            {
                continue;
            }

            aLines.Add($"I {i} {Kerf.Engine.Foundation.OcctTypeNames.Of(anEnt.GetType())} {aRes}{Checks(aTP)}");
            if (theTexts is not null && aText is not null)
            {
                theTexts[i] = aText;
            }
        }

        return aLines;
    }

    // ---------------------------------------------------------------------------------------------------------
    // TopoDSToStep

    public static List<string> Written(Transient theEnt)
    {
        StepProtocol aProtocol = Protocol();
        var aModel = new StepModel();
        aModel.SetProtocol(aProtocol);
        aModel.AddWithRefs(theEnt, aProtocol);
        var aWriter = new StepWriter(aModel);
        var aLib = new WriterLib(aProtocol);
        for (int i = 1; i <= aModel.EntityCount; i++)
        {
            aWriter.SendEntity(i, aLib);
        }

        var aRes = new List<string>();
        for (int i = 1; i <= aWriter.LineCount; i++)
        {
            aRes.Add(aWriter.Line(i).ToCString());
        }

        return aRes;
    }

    private static FinderProcess NewFP()
    {
        var aModel = new StepModel();
        aModel.SetProtocol(Protocol());
        aModel.InternalParameters = new Parameters();
        var aFP = new FinderProcess();
        aFP.SetModel(aModel);
        return aFP;
    }

    private static void Convert(List<string> theLines, List<string>? theDump, string theOp, int theIndex, string theTag,
                                Func<FinderProcess, (bool Done, Transient? Value)> theMaker)
    {
        string aRes;
        List<string> aWritten = [];
        try
        {
            var (isDone, aValue) = theMaker(NewFP());
            if (!isDone || aValue is null)
            {
                aRes = "NOTDONE";
            }
            else
            {
                aWritten = Written(aValue);
                string aFirst = aWritten.Count == 0 ? "" : aWritten[0];
                aRes = aWritten.Count + " " + StepFileDump.Hash(aWritten) + " " + (aFirst.Length > 120 ? aFirst[..120] : aFirst);
            }
        }
        catch (Exception anException) when (anException is not OutOfMemoryException)
        {
            aRes = "EXC";
        }

        theLines.Add($"C {theOp} {theIndex} {theTag} {aRes}");
        if (theDump is not null)
        {
            theDump.Add($"C {theOp} {theIndex} {theTag}");
            theDump.AddRange(aWritten);
        }
    }

    /// <summary>The conversions of a shape ("C ..."), with both factor sets; the written lines in the dump.</summary>
    public static List<string> ShapeLines(in Shape theShape, List<string>? theDump = null)
    {
        var aLines = new List<string>();
        Shape aShape = theShape;
        foreach (var (aTag, F) in new[] { ("mm", Millimetres), ("in", Inches) })
        {
            int i = 0;
            for (var anExp = new Explorer(aShape, ShapeType.Solid); anExp.More(); anExp.Next())
            {
                Solid aSolid = ShapeCasts.Solid(anExp.Current());
                ++i;
                int aNbShells = 0;
                for (var anIt = new Iterator(aSolid); anIt.More(); anIt.Next())
                {
                    aNbShells += anIt.Value().Type == ShapeType.Shell ? 1 : 0;
                }

                Convert(aLines, theDump, "msb-solid", i, aTag, FP =>
                {
                    var M = new ManifoldSolidBrepMaker(aSolid, FP, F);
                    return (M.IsDone, M.IsDone ? M.Value() : null);
                });
                if (aNbShells > 1)
                {
                    Convert(aLines, theDump, "bwv-solid", i, aTag, FP =>
                    {
                        var M = new MakeBrepWithVoids(aSolid, FP, F);
                        return (M.IsDone, M.IsDone ? M.Value() : null);
                    });
                }

                Convert(aLines, theDump, "sbsm-solid", i, aTag, FP =>
                {
                    var M = new MakeShellBasedSurfaceModel(aSolid, FP, F);
                    return (M.IsDone, M.IsDone ? M.Value() : null);
                });
                if (FacetedTool.CheckTopoDSShape(aSolid) == FacetedError.FacetedDone)
                {
                    Convert(aLines, theDump, "fb-solid", i, aTag, FP =>
                    {
                        var M = new MakeFacetedBrep(aSolid, FP, F);
                        return (M.IsDone, M.IsDone ? M.Value() : null);
                    });
                }
            }

            i = 0;
            for (var anExp = new Explorer(aShape, ShapeType.Shell); anExp.More(); anExp.Next())
            {
                Shell aShell = ShapeCasts.Shell(anExp.Current());
                ++i;
                Convert(aLines, theDump, "msb-shell", i, aTag, FP =>
                {
                    var M = new ManifoldSolidBrepMaker(aShell, FP, F);
                    return (M.IsDone, M.IsDone ? M.Value() : null);
                });
                Convert(aLines, theDump, "sbsm-shell", i, aTag, FP =>
                {
                    var M = new MakeShellBasedSurfaceModel(aShell, FP, F);
                    return (M.IsDone, M.IsDone ? M.Value() : null);
                });
                if (aShell.ClosedFlag && FacetedTool.CheckTopoDSShape(aShell) == FacetedError.FacetedDone)
                {
                    Convert(aLines, theDump, "fb-shell", i, aTag, FP =>
                    {
                        var M = new MakeFacetedBrep(aShell, FP, F);
                        return (M.IsDone, M.IsDone ? M.Value() : null);
                    });
                }
            }

            i = 0;
            for (var anExp = new Explorer(aShape, ShapeType.Face); anExp.More() && i < 20; anExp.Next())
            {
                ++i;
                Face aFace = ShapeCasts.Face(anExp.Current());
                Convert(aLines, theDump, "sbsm-face", i, aTag, FP =>
                {
                    var M = new MakeShellBasedSurfaceModel(aFace, FP, F);
                    return (M.IsDone, M.IsDone ? M.Value() : null);
                });
            }

            Convert(aLines, theDump, "gcs", 1, aTag, FP =>
            {
                var M = new MakeGeometricCurveSet(aShape, FP, F);
                return (M.IsDone, M.IsDone ? M.Value() : null);
            });
        }

        return aLines;
    }
}
