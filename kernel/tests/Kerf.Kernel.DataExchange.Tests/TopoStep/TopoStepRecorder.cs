// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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

        if (!aBuilder.IsDone)
        {
            return "NOTDONE";
        }

        var aS = new StringWriter { NewLine = "\n" };
        BrepTools.Write(aBuilder.Value(), aS);
        theText = aS.ToString();
        return Sha256(theText) + " " + theText.Length;
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
