// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Security.Cryptography;
using System.Text;
using OcctNet.DataExchange.Tests.StepGen;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopoDSToStep_FacetedError;

namespace OcctNet.DataExchange.Tests.TopoStep;

/// <summary>
/// The port's twin of tools/toporec (unit 8E): the same lines, for StepToTopoDS_Builder on a model's shape
/// representation items ("s2t") and for TopoDSToStep's makers on a shape ("t2s").
/// </summary>
internal static class TopoStepRecorder
{
    public static readonly StepData_Factors Millimetres = new();
    public static readonly StepData_Factors Inches = new StepData_Factors().InitializedFactors(25.4, M_PI / 180.0, 1.0);

    /// <summary>StepAP214's protocol (unit 8C2), as native's recorder reads and writes with.</summary>
    internal static StepData_Protocol Protocol() => StepRoundTripTests.Protocol();

    internal static string Sha256(string theText) => System.Convert.ToHexStringLower(SHA256.HashData(Encoding.Latin1.GetBytes(theText)));

    // ---------------------------------------------------------------------------------------------------------
    // StepToTopoDS

    private static string Checks(Transfer_TransientProcess theTP)
    {
        int aNbW = 0, aNbF = 0;
        Interface_CheckIterator aList = theTP.CheckList(false);
        for (aList.Start(); aList.More(); aList.Next())
        {
            aNbW += aList.Value()!.NbWarnings();
            aNbF += aList.Value()!.NbFails();
        }

        return " W" + aNbW + " F" + aNbF;
    }

    // The item's translation: "NOTDONE", "EXC" or the BRep text's hash and length (null: not an item translated)
    private static string? Translate(Transient theEnt, Transfer_TransientProcess theTP, out string? theText)
    {
        theText = null;
        var aBuilder = new StepToTopoDS_Builder();
        var aFactors = new StepData_Factors();
        bool aHasGeom = false;
        switch (theEnt)
        {
            case StepShape_FacetedBrepAndBrepWithVoids e:
                aBuilder.Init(e, theTP, aFactors);
                break;
            case StepShape_BrepWithVoids e:
                aBuilder.Init(e, theTP, aFactors);
                break;
            case StepShape_FacetedBrep e:
                aBuilder.Init(e, theTP, aFactors);
                break;
            case StepShape_ManifoldSolidBrep e:
                aBuilder.Init(e, theTP, aFactors);
                break;
            case StepShape_ShellBasedSurfaceModel e:
                aBuilder.Init(e, theTP, new StepToTopoDS_NMTool(), aFactors);
                break;
            case StepShape_EdgeBasedWireframeModel e:
                aBuilder.Init(e, theTP, aFactors);
                break;
            case StepShape_FaceBasedSurfaceModel e:
                aBuilder.Init(e, theTP, aFactors);
                break;
            case StepShape_GeometricSet e:
                aBuilder.Init(e, theTP, aFactors);
                break;
            case StepVisual_TessellatedSolid e:
                aBuilder.Init(e, theTP, false, ref aHasGeom, aFactors);
                break;
            case StepVisual_TessellatedShell e:
                aBuilder.Init(e, theTP, false, ref aHasGeom, aFactors);
                break;
            case StepVisual_TessellatedFace e:
                aBuilder.Init(e, theTP, false, ref aHasGeom, aFactors);
                break;
            case StepVisual_TessellatedSurfaceSet e:
                aBuilder.Init(e, theTP, ref aHasGeom, aFactors);
                break;
            default:
                return null;
        }

        if (!aBuilder.IsDone())
        {
            return "NOTDONE";
        }

        var aS = new StringWriter { NewLine = "\n" };
        BRepTools.Write(aBuilder.Value(), aS);
        theText = aS.ToString();
        return Sha256(theText) + " " + theText.Length;
    }

    /// <summary>The lines of a model's items ("I ..."), and the BRep texts by entity number.</summary>
    public static List<string> ModelLines(StepData_StepModel theModel, Dictionary<int, string>? theTexts = null)
    {
        theModel.InternalParameters = new DESTEP_Parameters();
        var aLines = new List<string>();
        for (int i = 1; i <= theModel.NbEntities(); i++)
        {
            Transient anEnt = theModel.Value(i)!;
            var aTP = new Transfer_TransientProcess(theModel.NbEntities());
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
        StepData_Protocol aProtocol = Protocol();
        var aModel = new StepData_StepModel();
        aModel.SetProtocol(aProtocol);
        aModel.AddWithRefs(theEnt, aProtocol);
        var aWriter = new StepData_StepWriter(aModel);
        var aLib = new StepData_WriterLib(aProtocol);
        for (int i = 1; i <= aModel.NbEntities(); i++)
        {
            aWriter.SendEntity(i, aLib);
        }

        var aRes = new List<string>();
        for (int i = 1; i <= aWriter.NbLines(); i++)
        {
            aRes.Add(aWriter.Line(i).ToCString());
        }

        return aRes;
    }

    private static Transfer_FinderProcess NewFP()
    {
        var aModel = new StepData_StepModel();
        aModel.SetProtocol(Protocol());
        aModel.InternalParameters = new DESTEP_Parameters();
        var aFP = new Transfer_FinderProcess();
        aFP.SetModel(aModel);
        return aFP;
    }

    private static void Convert(List<string> theLines, List<string>? theDump, string theOp, int theIndex, string theTag,
                                Func<Transfer_FinderProcess, (bool Done, Transient? Value)> theMaker)
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
    public static List<string> ShapeLines(in TopoDS_Shape theShape, List<string>? theDump = null)
    {
        var aLines = new List<string>();
        TopoDS_Shape aShape = theShape;
        foreach (var (aTag, F) in new[] { ("mm", Millimetres), ("in", Inches) })
        {
            int i = 0;
            for (var anExp = new TopExp_Explorer(aShape, TopAbs_SOLID); anExp.More(); anExp.Next())
            {
                TopoDS_Solid aSolid = TopoDS.Solid(anExp.Current());
                ++i;
                int aNbShells = 0;
                for (var anIt = new TopoDS_Iterator(aSolid); anIt.More(); anIt.Next())
                {
                    aNbShells += anIt.Value().ShapeType() == TopAbs_SHELL ? 1 : 0;
                }

                Convert(aLines, theDump, "msb-solid", i, aTag, FP =>
                {
                    var M = new TopoDSToStep_MakeManifoldSolidBrep(aSolid, FP, F);
                    return (M.IsDone(), M.IsDone() ? M.Value() : null);
                });
                if (aNbShells > 1)
                {
                    Convert(aLines, theDump, "bwv-solid", i, aTag, FP =>
                    {
                        var M = new TopoDSToStep_MakeBrepWithVoids(aSolid, FP, F);
                        return (M.IsDone(), M.IsDone() ? M.Value() : null);
                    });
                }

                Convert(aLines, theDump, "sbsm-solid", i, aTag, FP =>
                {
                    var M = new TopoDSToStep_MakeShellBasedSurfaceModel(aSolid, FP, F);
                    return (M.IsDone(), M.IsDone() ? M.Value() : null);
                });
                if (TopoDSToStep_FacetedTool.CheckTopoDSShape(aSolid) == TopoDSToStep_FacetedDone)
                {
                    Convert(aLines, theDump, "fb-solid", i, aTag, FP =>
                    {
                        var M = new TopoDSToStep_MakeFacetedBrep(aSolid, FP, F);
                        return (M.IsDone(), M.IsDone() ? M.Value() : null);
                    });
                }
            }

            i = 0;
            for (var anExp = new TopExp_Explorer(aShape, TopAbs_SHELL); anExp.More(); anExp.Next())
            {
                TopoDS_Shell aShell = TopoDS.Shell(anExp.Current());
                ++i;
                Convert(aLines, theDump, "msb-shell", i, aTag, FP =>
                {
                    var M = new TopoDSToStep_MakeManifoldSolidBrep(aShell, FP, F);
                    return (M.IsDone(), M.IsDone() ? M.Value() : null);
                });
                Convert(aLines, theDump, "sbsm-shell", i, aTag, FP =>
                {
                    var M = new TopoDSToStep_MakeShellBasedSurfaceModel(aShell, FP, F);
                    return (M.IsDone(), M.IsDone() ? M.Value() : null);
                });
                if (aShell.Closed() && TopoDSToStep_FacetedTool.CheckTopoDSShape(aShell) == TopoDSToStep_FacetedDone)
                {
                    Convert(aLines, theDump, "fb-shell", i, aTag, FP =>
                    {
                        var M = new TopoDSToStep_MakeFacetedBrep(aShell, FP, F);
                        return (M.IsDone(), M.IsDone() ? M.Value() : null);
                    });
                }
            }

            i = 0;
            for (var anExp = new TopExp_Explorer(aShape, TopAbs_FACE); anExp.More() && i < 20; anExp.Next())
            {
                ++i;
                TopoDS_Face aFace = TopoDS.Face(anExp.Current());
                Convert(aLines, theDump, "sbsm-face", i, aTag, FP =>
                {
                    var M = new TopoDSToStep_MakeShellBasedSurfaceModel(aFace, FP, F);
                    return (M.IsDone(), M.IsDone() ? M.Value() : null);
                });
            }

            Convert(aLines, theDump, "gcs", 1, aTag, FP =>
            {
                var M = new TopoDSToStep_MakeGeometricCurveSet(aShape, FP, F);
                return (M.IsDone(), M.IsDone() ? M.Value() : null);
            });
        }

        return aLines;
    }
}
