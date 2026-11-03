// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Reflection;
using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// OCCT's own Tcl procedures (resources/DrawResources): they load, and the check procedures run on top of stand-ins
/// for the C++ commands they call (nbshapes, tolerance, sprops, ...) that come with the geometry ports.
/// </summary>
public class DrawResourcesTests(ITestOutputHelper output)
{
    internal static string OcctSourceDirectory
        => typeof(DrawResourcesTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == "OcctSourceDir").Value!;

    internal static string OcctTestDataDirectory
        => typeof(DrawResourcesTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == "OcctTestDataDir").Value!;

    internal static string ResourcesDirectory => Path.Combine(OcctSourceDirectory, "resources", "DrawResources");

    private static (DrawInterpreter Interpreter, StringWriter Output) Load()
    {
        Assert.True(Directory.Exists(ResourcesDirectory), $"No OCCT source at {OcctSourceDirectory}: run tools/get-occt.ps1");
        var anOutput = new StringWriter { NewLine = "\n" };
        var anInterpreter = new DrawInterpreter(anOutput);
        anInterpreter.LoadDrawResources(ResourcesDirectory);
        return (anInterpreter, anOutput);
    }

    [Fact]
    public void CheckCommands_tcl_sources_on_its_own_after_StandardCommands_tcl()
    {
        var anInterpreter = new DrawInterpreter();
        // CheckCommands.tcl calls help, which StandardCommands.tcl defines
        anInterpreter.Source(Path.Combine(ResourcesDirectory, "StandardCommands.tcl").Replace('\\', '/'));
        anInterpreter.Source(Path.Combine(ResourcesDirectory, "CheckCommands.tcl").Replace('\\', '/'));

        foreach (string aProc in new[] { "checkprops", "checknbshapes", "checkreal", "checkmaxtol", "checkfreebounds", "checkfaults", "checktrinfo", "checkdump", "checklength", "checkplatform", "checkgravitycenter", "checkpoint", "checkMultilineStrings", "_check_arg", "_check_args" })
        {
            Assert.NotNull(anInterpreter.ProcBody(aProc));
        }

        Assert.Contains("checkprops", anInterpreter.Eval("array names Draw_Helps"));
    }

    [Fact]
    public void All_draw_resources_load()
    {
        (DrawInterpreter anInterpreter, _) = Load();
        foreach (string aProc in new[] { "help", "restore", "datadir", "locate_data_file", "testgrid", "checkprops", "lvarpop", "lmatch" })
        {
            Assert.True(anInterpreter.HasCommand(aProc), aProc);
        }

        // checkview drives OCCT's viewers; Draw-lite keeps it a no-op
        Assert.Null(anInterpreter.ProcBody("checkview"));
        Assert.Equal("", anInterpreter.Eval("checkview -display result -3d -path x.png"));
    }

    [Fact]
    public void Checkreal_and_checkpoint_compare_values()
    {
        (DrawInterpreter anInterpreter, StringWriter anOutput) = Load();
        anInterpreter.Eval("""
            checkreal "Max tolerance" 0.001 0.001 1e-6 1e-6
            checkreal "Area" 10.5 10 0.1 0.01
            checkpoint p {1 2 3} {1 2 3.00001} 1e-4
            """);
        Assert.Equal("""
            Check of Max tolerance OK: value = 0.001, expected = 0.001
            Error: Area = 10.5 is not equal to expected 10
            Check of p OK: value = (1 2 3), expected = (1 2 3.00001)

            """.ReplaceLineEndings("\n"), anOutput.ToString());
    }

    [Fact]
    public void Checknbshapes_parses_nbshapes_output()
    {
        (DrawInterpreter anInterpreter, StringWriter anOutput) = Load();
        anInterpreter.Register("nbshapes", static (_, a) => $"""
            Number of shapes in {a[1]}
             VERTEX    : 8
             EDGE      : 12
             WIRE      : 6
             FACE      : 6
             SHELL     : 1
             SOLID     : 1
             COMPSOLID : 0
             COMPOUND  : 0
             SHAPE     : 34

            """);
        anInterpreter.Eval("checknbshapes result -vertex 8 -edge 12 -face 5 -m Box");
        string aLog = anOutput.ToString();
        Assert.Contains("checknbshapes result -vertex 8 -edge 12 -face 5 -m Box", aLog);
        Assert.Contains("OK : Box is GOOD because number of VERTEX entities is equal to number of expected VERTEX entities", aLog);
        Assert.Contains("OK : Box is GOOD because number of EDGE entities is equal to number of expected EDGE entities", aLog);
        Assert.Contains("Error : Box is WRONG because number of FACE entities in shape \"result\" is 6", aLog);
        Assert.Equal("Error: unsupported option \"-bogus\"", Assert.ThrowsAny<DrawException>(() => anInterpreter.Eval("checknbshapes result -bogus 1")).Reason);
    }

    [Fact]
    public void Checkprops_compares_mass_properties()
    {
        (DrawInterpreter anInterpreter, StringWriter anOutput) = Load();
        // Stand-ins for the Draw commands checkprops uses: isdraw, dtyp (through whatis), and sprops/vprops
        anInterpreter.Register("isdraw", static (_, _) => "1");
        anInterpreter.Register("dtyp", static (_, a) => $"shape {a[1]}");
        anInterpreter.Register("sprops", static (_, _) => "Mass :  600\n\nCenter of gravity : \nX = 5\nY = 5\nZ = 5\n");
        anInterpreter.Register("vprops", static (_, _) => "Mass :  1000.5\n");
        anInterpreter.Eval("checkprops result -s 600 -v 1000");
        string aLog = anOutput.ToString();
        Assert.Contains("checkprops result -s 600 -v 1000", aLog);
        Assert.DoesNotContain("Error", aLog);

        anOutput.GetStringBuilder().Clear();
        anInterpreter.Eval("checkprops result -s 700");
        Assert.Contains("Error : The area of result shape is 600, expected 700", anOutput.ToString());

        anOutput.GetStringBuilder().Clear();
        anInterpreter.Eval("checkgravitycenter result -s 5 5 5 1e-7");
        Assert.Contains("Check of center of gravity is OK: value = (5, 5, 5), expected = (5, 5, 5)", anOutput.ToString());
    }

    [Fact]
    public void Checkmaxtol_reads_tolerance_output()
    {
        (DrawInterpreter anInterpreter, StringWriter anOutput) = Load();
        anInterpreter.Register("tolerance", static (_, _) => "Tolerance MAX=0.00031 AVG=0.0001 MIN=1e-07\nFACE\t: MAX=1e-07\n");
        Assert.Equal("0.00031", anInterpreter.Eval("checkmaxtol result -ref 0.00031"));
        Assert.Contains("Check of Max tolerance OK: value = 0.00031, expected = 0.00031", anOutput.ToString());
        Assert.Equal("0.00031", anInterpreter.Eval("checkmaxtol result -min_tol 0.001"));
        anInterpreter.Eval("checkmaxtol result -min_tol 0.0001");
        Assert.Contains("Error: tolerance of \"result\" (0.00031) is greater than checking tolerance (0.0001)", anOutput.ToString());
    }

    [Fact]
    public void Checkplatform_and_help_work()
    {
        (DrawInterpreter anInterpreter, StringWriter anOutput) = Load();
        string aPlatform = anInterpreter.Eval("checkplatform");
        Assert.Equal(OperatingSystem.IsWindows() ? "Windows" : OperatingSystem.IsMacOS() ? "MacOS" : "Linux", aPlatform);
        Assert.Equal(OperatingSystem.IsWindows() ? "1" : "0", anInterpreter.Eval("checkplatform -windows"));
        anInterpreter.Eval("help checkreal");
        Assert.Contains("checkreal", anOutput.ToString());
        Assert.Contains("Use: checkreal name value expected tol_abs tol_rel", anOutput.ToString());
    }

    [Fact]
    public void Locate_data_file_searches_the_data_path()
    {
        (DrawInterpreter anInterpreter, _) = Load();
        string aDirectory = Directory.CreateTempSubdirectory("drawlite-data").FullName;
        try
        {
            Directory.CreateDirectory(Path.Combine(aDirectory, "brep"));
            File.WriteAllText(Path.Combine(aDirectory, "brep", "shape.brep"), "");
            string aTcl = aDirectory.Replace('\\', '/');
            anInterpreter.Eval($"set env(CSF_TestDataPath) {{{aTcl}}}");
            Assert.Equal($"{aTcl}/brep/shape.brep".ToLowerInvariant(), anInterpreter.Eval("locate_data_file shape.brep").ToLowerInvariant());
            Assert.StartsWith("File nosuch.brep could not be found", Assert.ThrowsAny<DrawException>(() => anInterpreter.Eval("locate_data_file nosuch.brep")).Reason);
        }
        finally
        {
            Directory.Delete(aDirectory, recursive: true);
        }
    }

    [Fact]
    public void Reports_the_primitives_the_check_procedures_need()
    {
        (DrawInterpreter anInterpreter, _) = Load();
        var aCorpus = new DrawCorpus(Path.Combine(OcctSourceDirectory, "tests"));
        string aCheckCommands = File.ReadAllText(Path.Combine(ResourcesDirectory, "CheckCommands.tcl"));
        List<string> aProcs = anInterpreter.ProcNames.Where(p => aCheckCommands.Contains($"proc {p} ", StringComparison.Ordinal))
                                           .Order(StringComparer.Ordinal).ToList();

        var aMissing = new SortedDictionary<string, SortedSet<string>>(StringComparer.Ordinal);
        foreach (string aProc in aProcs)
        {
            foreach (string aCall in Closure(aCorpus, anInterpreter, aProc).Where(c => !anInterpreter.HasCommand(c)))
            {
                if (!aMissing.TryGetValue(aCall, out SortedSet<string>? aUsers))
                {
                    aUsers = new SortedSet<string>(StringComparer.Ordinal);
                    aMissing[aCall] = aUsers;
                }

                aUsers.Add(aProc);
            }
        }

        output.WriteLine($"CheckCommands.tcl procedures: {aProcs.Count}; C++ commands they call that Draw-lite lacks:");
        foreach ((string aCall, SortedSet<string> aUsers) in aMissing)
        {
            output.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {aCall,-14} used by {string.Join(", ", aUsers)}"));
        }

        // unit 3L1's, 3L2a's and 4L1's commands are there (checktrinfo's trinfo), and 6L's freebounds (checkfreebounds)
        foreach (string aPrimitive in new[] { "nbshapes", "tolerance", "isdraw", "dtyp", "sprops", "vprops", "lprops", "explode", "dump", "checkshape", "trinfo", "freebounds" })
        {
            Assert.DoesNotContain(aPrimitive, aMissing.Keys);
        }

        // Tcl itself is complete for these procedures
        Assert.DoesNotContain(aMissing.Keys, k => k is "regexp" or "lsort" or "upvar" or "switch" or "string" or "dict" or "expr");
    }

    private static HashSet<string> Closure(DrawCorpus theCorpus, DrawInterpreter theInterpreter, string theProc)
    {
        var aSeen = new HashSet<string>(StringComparer.Ordinal);
        var aPending = new Stack<string>([theProc]);
        while (aPending.TryPop(out string? aName))
        {
            foreach (string aCall in theCorpus.ProcCalls(theInterpreter, aName) ?? new HashSet<string>())
            {
                if (aSeen.Add(aCall))
                {
                    aPending.Push(aCall);
                }
            }
        }

        return aSeen;
    }
}
