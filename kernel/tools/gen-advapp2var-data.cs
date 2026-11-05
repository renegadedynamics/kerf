// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Generates src/Kerf.Kernel.ModelingData/AdvApp2Var/AdvApp2Var_Data_*.cs from OCCT's AdvApp2Var_Data_*.pxx: the constant
// tables of AdvApp2Var's numeric core (decisions 77 and 902). Every literal is copied as written, except that C's
// "1." and "1.e5" become "1.0" and "1.0e5", which C# requires.
//
//   dotnet run tools/gen-advapp2var-data.cs [OCCT source root, default .deps/occt-src]

using System.Globalization;
using System.Text;
using System.Text.RegularExpressions;

string aRoot = FindRoot(Directory.GetCurrentDirectory())
               ?? throw new InvalidOperationException("Run from inside the Kerf repository.");
string anOcctSrc = args.Length > 0 ? args[0] : Path.Combine(aRoot, ".deps", "occt-src");
string aSrcDir = Path.Combine(anOcctSrc, "src", "ModelingData", "TKGeomBase", "AdvApp2Var");
string anOutDir = Path.Combine(aRoot, "src", "Kerf.Kernel.ModelingData", "AdvApp2Var");
if (!Directory.Exists(aSrcDir))
{
    Console.Error.WriteLine($"No OCCT source at {aSrcDir}: run tools/get-occt.ps1");
    return 1;
}

// One flat table per file, cast by OCCT to its struct: (struct, C++ array name, size, fields as (name, length)).
var aTables = new (string File, string Struct, string Getter, int Size, (string Name, int Length, string Comment)[] Fields)[]
{
    ("Mlgdrtl", "mlgdrtl_1_", "Getmlgdrtl", 1891,
     [("rootab", 930, "was [465][2]"), ("hiltab", 930, "was [465][2]"), ("hi0tab", 31, "")]),
    ("Mmjcobi", "mmjcobi_1_", "Getmmjcobi", 7936,
     [("plgcan", 3968, "was [496][2][4]"), ("canjac", 3968, "was [496][2][4]")]),
    ("Mmcmcnp", "mmcmcnp_1_", "Getmmcmcnp", 3721, [("cnp", 3721, "was [61][61]")]),
    ("Mmapgss", "mmapgss_1_", "Getmmapgss", 5069, [("gslxjs", 5017, ""), ("gsl0js", 52, "")]),
    ("Mmapgs0", "mmapgs0_1_", "Getmmapgs0", 4810, [("gslxj0", 4761, ""), ("gsl0j0", 49, "")]),
    ("Mmapgs1", "mmapgs1_1_", "Getmmapgs1", 4551, [("gslxj1", 4505, ""), ("gsl0j1", 46, "")]),
    ("Mmapgs2", "mmapgs2_1_", "Getmmapgs2", 4292, [("gslxj2", 4249, ""), ("gsl0j2", 43, "")]),
};

foreach (var aTable in aTables)
{
    string aText = File.ReadAllText(Path.Combine(aSrcDir, $"AdvApp2Var_Data_{aTable.File}.pxx"));
    Match anArray = Regex.Match(aText, @"static double (\w+)\[(\d+)\] = \{(.*?)\};", RegexOptions.Singleline);
    if (!anArray.Success || int.Parse(anArray.Groups[2].Value, CultureInfo.InvariantCulture) != aTable.Size)
    {
        throw new InvalidOperationException($"{aTable.File}: table not found or not of size {aTable.Size}");
    }
    List<string> aValues = Literals(anArray.Groups[3].Value);
    if (aValues.Count != aTable.Size)
    {
        throw new InvalidOperationException($"{aTable.File}: {aValues.Count} literals for {aTable.Size} elements");
    }

    var anOut = new StringBuilder();
    Header(anOut, aTable.File);
    anOut.Append($$"""
        /// <summary>The <c>{{aTable.Struct}}</c> common block: views of one constant table (decision 902).</summary>
        internal readonly ref struct {{aTable.Struct}}
        {

        """);
    int aStart = 0;
    foreach (var aField in aTable.Fields)
    {
        string aComment = aField.Comment.Length > 0 ? $" ({aField.Comment})" : "";
        anOut.Append($"    /// <summary><c>double {aField.Name}[{aField.Length}]</c>{aComment}, from element {aStart}.</summary>\n");
        anOut.Append($"    public readonly F2cConstPtr<double> {aField.Name};\n\n");
        aStart += aField.Length;
    }
    anOut.Append($"    internal {aTable.Struct}(ReadOnlySpan<double> theTable)\n    {{\n");
    aStart = 0;
    foreach (var aField in aTable.Fields)
    {
        anOut.Append($"        {aField.Name} = new F2cConstPtr<double>(theTable, {aStart});\n");
        aStart += aField.Length;
    }
    anOut.Append("    }\n}\n\n");
    anOut.Append($$"""
        internal static partial class AdvApp2Var_Data
        {
            public static {{aTable.Struct}} {{aTable.Getter}}() => new(s_e_{{aTable.File}});

            // C++: static double e_1[{{aTable.Size}}], cast to {{aTable.Struct}}.
            private static ReadOnlySpan<double> s_e_{{aTable.File}} => [

        """);
    AppendValues(anOut, aValues, 2);
    anOut.Append("    ];\n}\n");
    Write($"AdvApp2Var_Data_{aTable.File}.cs", anOut);
}

// AdvApp2Var_Data_Main.pxx: four structs initialised member by member.
{
    string aText = File.ReadAllText(Path.Combine(aSrcDir, "AdvApp2Var_Data_Main.pxx"));
    List<string> aNombr = Literals(Regex.Match(aText, @"s_mdnombr = \{(.*?)\};", RegexOptions.Singleline).Groups[1].Value);
    List<string> aNbr = Literals(Regex.Match(aText, @"s_minombr_1 = \{\s*\{(.*?)\}\};", RegexOptions.Singleline).Groups[1].Value);
    List<string> aPar = Literals(Regex.Match(aText, @"s_maovpar = \{(.*?)\};", RegexOptions.Singleline).Groups[1].Value);
    MatchCollection aChars = Regex.Matches(
        Regex.Match(aText, @"s_maovpch_1_ = \{(.*?)\};", RegexOptions.Singleline).Groups[1].Value, @"\{([^}]*)\}");
    if (aNombr.Count != 10 || aNbr.Count != 1001 || aPar.Count != 20 || aChars.Count != 4)
    {
        throw new InvalidOperationException("AdvApp2Var_Data_Main.pxx: unexpected initialisers");
    }
    string[] aPch = aChars.Select(theMatch => string.Concat(
        Regex.Matches(theMatch.Groups[1].Value, @"'(.)'").Select(theChar => theChar.Groups[1].Value))).ToArray();

    var anOut = new StringBuilder();
    Header(anOut, "Main");
    anOut.Append($$"""
        internal static partial class AdvApp2Var_Data
        {
            private static readonly mdnombr_1_ s_mdnombr = new()
            {
                pi = {{aNombr[0]}},
                deuxpi = {{aNombr[1]}},
                pisur2 = {{aNombr[2]}},
                pis180 = {{aNombr[3]}},
                c180pi = {{aNombr[4]}},
                zero = {{aNombr[5]}},
                one = {{aNombr[6]}},
                a180 = {{aNombr[7]}},
                a360 = {{aNombr[8]}},
                a90 = {{aNombr[9]}},
            };

            public static ref readonly mdnombr_1_ Getmdnombr() => ref s_mdnombr;

            public static minombr_1_ Getminombr() => new(s_nbr);

            // C++: minombr_1_::nbr[1001].
            private static ReadOnlySpan<int> s_nbr => [

        """);
    AppendValues(anOut, aNbr, 20);
    anOut.Append($$"""
            ];

            private static readonly maovpar_1_ s_maovpar = new()
            {
                r8und = {{aPar[0]}},
                r8ovr = {{aPar[1]}},
                x4und = {{aPar[2]}},
                x4ovr = {{aPar[3]}},
                r4und = {{aPar[4]}},
                r4ovr = {{aPar[5]}},
                r4nbe = {{aPar[6]}},
                r8nbm = {{aPar[7]}},
                r8nbe = {{aPar[8]}},
                i4ovr = {{aPar[9]}},
                i4ovn = {{aPar[10]}},
                r4exp = {{aPar[11]}},
                r8exp = {{aPar[12]}},
                r4exn = {{aPar[13]}},
                r8exn = {{aPar[14]}},
                r4ncs = {{aPar[15]}},
                r8ncs = {{aPar[16]}},
                r4nbm = {{aPar[17]}},
                i2ovr = {{aPar[18]}},
                i2ovn = {{aPar[19]}},
            };

            public static ref readonly maovpar_1_ Getmaovpar() => ref s_maovpar;

            private static readonly maovpch_1_ s_maovpch = new()
            {
                cnmmac = "{{aPch[0]}}",
                frmr4 = "{{aPch[1]}}",
                frmr8 = "{{aPch[2]}}",
                cdcode = "{{aPch[3]}}",
            };

            public static ref readonly maovpch_1_ Getmaovpch() => ref s_maovpch;
        }

        """);
    Write("AdvApp2Var_Data_Main.cs", anOut);
}
return 0;

// The comma-separated literals of a C initialiser list, in C# syntax.
static List<string> Literals(string theList)
{
    var aResult = new List<string>();
    foreach (string aRaw in theList.Split(','))
    {
        string aLiteral = Regex.Replace(aRaw.Trim(), @"\s+", "");
        if (aLiteral.Length == 0)
        {
            continue;
        }
        // "1." -> "1.0", "1.e5" -> "1.0e5"; C# needs a digit after the point.
        aLiteral = Regex.Replace(aLiteral, @"(\d)\.(?=$|[eE])", "$1.0");
        aResult.Add(aLiteral);
    }
    return aResult;
}

static void AppendValues(StringBuilder theOut, List<string> theValues, int thePerLine)
{
    for (int i = 0; i < theValues.Count; i += thePerLine)
    {
        theOut.Append("        ");
        theOut.AppendJoin(", ", theValues.Skip(i).Take(thePerLine));
        theOut.Append(",\n");
    }
}

static void Header(StringBuilder theOut, string theFile)
{
    theOut.Append($"""
        // Port of: src/ModelingData/TKGeomBase/AdvApp2Var/AdvApp2Var_Data_{theFile}.pxx
        // Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
        // Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

        // Generated by tools/gen-advapp2var-data.cs from OCCT's AdvApp2Var_Data_{theFile}.pxx, with every literal copied
        // as written (decisions 77 and 902). Properties returning a ReadOnlySpan over a constant array are stored in the
        // assembly's data section: reading them allocates nothing. Don't edit: regenerate.

        namespace Occt;


        """);
}

void Write(string theName, StringBuilder theText)
{
    File.WriteAllText(Path.Combine(anOutDir, theName), theText.ToString().Replace("\r\n", "\n"));
    Console.WriteLine($"Wrote {theName}");
}

static string? FindRoot(string theStart)
{
    for (var aDir = new DirectoryInfo(theStart); aDir != null; aDir = aDir.Parent)
    {
        if (File.Exists(Path.Combine(aDir.FullName, "occt-version.json")))
        {
            return aDir.FullName;
        }
    }
    return null;
}
