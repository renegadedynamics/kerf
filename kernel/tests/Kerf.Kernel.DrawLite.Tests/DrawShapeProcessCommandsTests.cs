// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 7C's ShapeProcess commands, <c>DT_ApplySeq</c> and <c>SPApply</c>: the resource file found through the
/// interpreter's <c>env(CSF_nameDefaults)</c>, OCCT's compiled-in defaults, the statistics printed to the interpreter's
/// output, the argument errors, and the input left as it was.
/// </summary>
public class DrawShapeProcessCommandsTests
{
    private static (DrawInterpreter, StringWriter) Load()
    {
        var anOutput = new StringWriter { NewLine = "\n" };
        var anInterpreter = new DrawInterpreter(anOutput);
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return (anInterpreter, anOutput);
    }

    private static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    private static string Error(DrawInterpreter theDI, string theScript) =>
        Assert.ThrowsAny<DrawException>(() => theDI.Eval(theScript)).Reason;

    [Fact]
    public void DT_ApplySeq_reads_the_resource_file_the_environment_names()
    {
        (DrawInterpreter i, StringWriter anOutput) = Load();
        string aDir = Path.Combine(Path.GetTempPath(), "occtnet_dtapplyseq_" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(aDir);
        File.WriteAllText(Path.Combine(aDir, "mine"), "split.exec.op : SplitAngle\nsplit.SplitAngle.Angle : 1.6\n");

        i.Eval("pcylinder c 10 20");
        Shape aCylinder = i.GetShape("c");
        int aFaces = Count(aCylinder, ShapeType.Face);

        // without the environment variable, no file: nothing done
        i.Eval("DT_ApplySeq r c mine split");
        Assert.Equal(aFaces, Count(i.GetShape("r"), ShapeType.Face));

        i.Eval($"set env(CSF_mineDefaults) {{{aDir.Replace('\\', '/')}}}");
        anOutput.GetStringBuilder().Clear();
        Assert.Equal("", i.Eval("DT_ApplySeq r c mine split"));
        Assert.Equal(aFaces + 3, Count(i.GetShape("r"), ShapeType.Face));
        // PrintPreparationResult (the message file's texts, their leading blanks removed as OCCT's loader removes them)
        Assert.StartsWith("Mapping:\nShells:\nResult is Shell                 : 1\nNo Result                       : 0\nFaces:\n",
                          anOutput.ToString(), StringComparison.Ordinal);
        Assert.EndsWith("Preparation ratio:\nShells: 100 per cent\nFaces : 100 per cent\n", anOutput.ToString(), StringComparison.Ordinal);
        // in place, as DRAWEXE: the input keeps its faces
        Assert.Equal(aFaces, Count(i.GetShape("c"), ShapeType.Face));
        Assert.Equal(aFaces, Count(aCylinder, ShapeType.Face));

        // SPApply: the same sequence through a context; PrintStatistics prints the same statistics
        anOutput.GetStringBuilder().Clear();
        Assert.Equal("", i.Eval("SPApply r c mine split"));
        Assert.Equal(aFaces + 3, Count(i.GetShape("r"), ShapeType.Face));
        Assert.StartsWith("Mapping:\n", anOutput.ToString(), StringComparison.Ordinal);
        Assert.Equal("Shape not modified\n", i.Eval("SPApply r c mine nothing"));
    }

    [Fact]
    public void The_compiled_in_defaults_and_the_argument_errors()
    {
        (DrawInterpreter i, StringWriter anOutput) = Load();
        i.Eval("box b 10 20 30");
        // OCCT's STEP file: ToSTEP is SplitCommonVertex and DirectFaces, which a box doesn't need
        i.Eval("unset -nocomplain env(CSF_STEPDefaults)");
        Assert.Equal("Shape not modified\n", i.Eval("SPApply r b STEP ToSTEP"));
        anOutput.GetStringBuilder().Clear();
        Assert.Equal("Shape not modified\n", i.Eval("SPApply r b STEP NoSuchSequence"));
        Assert.StartsWith("Warning: Shape Processing: Sequence not defined for NoSuchSequence, nothing to do\n", anOutput.ToString(),
                          StringComparison.Ordinal);

        Assert.Contains("Specify result, shape, resource name and prefix (optional)", Error(i, "DT_ApplySeq r b"), StringComparison.Ordinal);
        Assert.Contains("Specify result, shape, resource name and prefix (optional)", Error(i, "SPApply r b"), StringComparison.Ordinal);
        Assert.Contains("Shape unknown : nothing", Error(i, "DT_ApplySeq r nothing STEP"), StringComparison.Ordinal);
        Assert.Contains("Shape unknown : nothing", Error(i, "SPApply r nothing STEP"), StringComparison.Ordinal);
    }
}
