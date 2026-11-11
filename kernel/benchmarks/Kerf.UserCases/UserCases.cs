// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Kerf.Modeling;
using static Kerf.Modeling.Operations;

namespace Kerf.UserCases;

/// <summary>One shape a case made, as the case's result reads it.</summary>
public sealed record Measure(string Label, double Volume, bool IsValid, int Faces)
{
    public static Measure Of(string label, Part part) => new(label, part.Volume, part.IsValid, part.Faces().Count);

    /// <summary>Whether two measures agree: the same label, validity and faces, and volumes within a relative tolerance.</summary>
    public bool Matches(Measure other, double relativeVolume) =>
        Label == other.Label && IsValid == other.IsValid && Faces == other.Faces &&
        Math.Abs(Volume - other.Volume) <= relativeVolume * Math.Max(Math.Abs(Volume), Math.Abs(other.Volume));
}

/// <summary>
/// What a case gave: its shapes' measures, or the failure an operation returned, or nothing where this version of Kerf
/// can't run the case (it uses an API that came later).
/// </summary>
public sealed record CaseResult(IReadOnlyList<Measure> Measures, string? Failure, bool IsAbsent = false)
{
    public static readonly CaseResult Absent = new([], null, IsAbsent: true);

    public static CaseResult Failed(string failure) => new([], failure);

    /// <summary>Whether this result agrees with another to <paramref name="relativeVolume"/> on each volume.</summary>
    public bool Matches(CaseResult other, double relativeVolume = UserCases.VolumeTolerance) =>
        IsAbsent == other.IsAbsent && (Failure is null) == (other.Failure is null) && Measures.Count == other.Measures.Count &&
        Measures.Zip(other.Measures).All(p => p.First.Matches(p.Second, relativeVolume));

    /// <summary>
    /// The result on one line, for the release check: <c>absent</c>, <c>failed: why</c>, or each measure as
    /// <c>label=volume,valid,faces</c> joined by <c>;</c>, the volume to the last bit.
    /// </summary>
    public string Exact() => IsAbsent ? "absent" : Failure is not null ? $"failed: {OneLine(Failure)}"
        : string.Join(';', Measures.Select(m => $"{m.Label}={m.Volume.ToString("R", CultureInfo.InvariantCulture)},{(m.IsValid ? "valid" : "invalid")},{m.Faces}"));

    /// <summary>The result as the rig keeps it, compared exactly between runs: the total volume to 0.001, and counts.</summary>
    public string Summary() => IsAbsent ? "absent" : Failure is not null ? "failed"
        : string.Create(CultureInfo.InvariantCulture,
            $"{Measures.Count} shapes, volume {Measures.Sum(m => m.Volume):F3}, {Measures.Count(m => !m.IsValid)} invalid, {Measures.Sum(m => m.Faces)} faces");

    private static string OneLine(string text) => text.Split('\n')[0].Trim().Replace('\t', ' ');
}

/// <summary>
/// A user's case: what it builds, from which files, and the result it has to give. <see cref="Prepare"/> reads the
/// operands (not timed) and returns the operation a user waits for, which is timed; <see cref="Read"/> measures what it
/// made (not timed).
/// </summary>
public sealed record UserCase(string Name, string Description, Func<string, Func<object>?> Prepare, Func<object, CaseResult> Read,
    CaseResult Expected);

/// <summary>
/// Users' cases (unit 13C, decision 2015): parts shaped like users', each with a fixed result, which the release check
/// (tools/release-check.ps1) runs against the last release and the rig (tools/bench.ps1) runs on every timed run.
/// README.md says where each came from.
/// </summary>
public static class UserCases
{
    /// <summary>How far a volume may move, relative, and still be the same result.</summary>
    public const double VolumeTolerance = 1e-6;

    /// <summary>The cases, in the order they run.</summary>
    public static IReadOnlyList<UserCase> All { get; } =
    [
        Fuse("fuse-boss", "a screw boss (a cylinder, r 3, 5.5 long) fused onto an enclosure's door", "door-before-boss.step", "boss.step"),
        Fuse("fuse-detent", "a 1.5 x 0.5 x 1.5 detent bump fused onto a slanted facet of the door", "door-before-detent.step", "detent-bump.step"),
        Chamfer("chamfer-guide-box", "the guide's box (40 x 30 x 30, its vertical edges filleted 1.7), its top's 8 edges chamfered 1.4",
            GuideBox, part => part.Edges().GroupBy(Axis.Z).Last(), 1.4, collapse: false),
        Chamfer("collapse-guide-box", "the same 8 edges chamfered 3, past the corners' radius: TightCorners.Collapse",
            GuideBox, part => part.Edges().GroupBy(Axis.Z).Last(), 3, collapse: true),
        Chamfer("chamfer-housing", "a coil housing's section extruded, the 20 edges round its end chamfered 1.4",
            HousingPrism, part => part.Edges().GroupBy(Axis.X).Last(), 1.4, collapse: false),
        Chamfer("collapse-housing", "the same 20 edges chamfered 3, past the 1.5 corners: TightCorners.Collapse",
            HousingPrism, part => part.Edges().GroupBy(Axis.X).Last(), 3, collapse: true),
        Enclosure(),
        EnclosureV016(),
        ShaftGauge(),
    ];

    /// <summary>The case of this name.</summary>
    public static UserCase Named(string name) =>
        All.FirstOrDefault(c => c.Name == name) ?? throw new ArgumentException($"no user case is called {name}", nameof(name));

    /// <summary>Runs a case once, untimed: what the test and the expected values use.</summary>
    public static CaseResult RunOnce(UserCase userCase, string dataFolder)
    {
        var operation = userCase.Prepare(dataFolder);
        return operation is null ? CaseResult.Absent : userCase.Read(operation());
    }

    private static Part Load(string dataFolder, string file)
    {
        var imported = Importers.ImportStep(Path.Combine(dataFolder, file));
        return imported.TryGetValue(out var shape) ? new Part(shape) : throw new InvalidOperationException($"{file}: {imported.Error}");
    }

    private static CaseResult ReadPart(object outcome, string label) => outcome switch
    {
        Part part => new CaseResult([Measure.Of(label, part)], null),
        string failure => CaseResult.Failed(failure),
        _ => throw new InvalidOperationException($"a case made a {outcome.GetType()}"),
    };

    // A small part fused onto a large one: the user timed the fuse and the volume (unit 13A's two cases)
    private static UserCase Fuse(string name, string description, string large, string small) => new(name, description,
        data =>
        {
            Part a = Load(data, large), b = Load(data, small);
            return () =>
            {
                Part fused = a + b;
                _ = fused.Volume;
                return fused;
            };
        },
        outcome => ReadPart(outcome, "fused"), Expected.Of(name));

    // The guide's example of a collapsing chamfer: a box, its vertical edges rounded 1.7
    private static Part GuideBox(string data)
    {
        Part block = new Box(40, 30, 30);
        return Fillet(block, block.Edges().ParallelTo(Axis.Z), 1.7).Value;
    }

    // The blade bumper's coil housing section, extruded along x: lines and arcs, with 1.5 corners
    private static Part HousingPrism(string data) => Load(data, "housing-prism.step");

    // A chamfer, collapsing the corners or not (unit 13B's cases, and the same edges inside the corners' radius beside
    // them). Before 0.4.0 there is no TightCorners, so a collapsing case runs the chamfer without it, which is refused
    private static UserCase Chamfer(string name, string description, Func<string, Part> make, Func<Part, IEnumerable<Kerf.Topology.Edge>> edges,
        double length, bool collapse) => new(name, description,
        data =>
        {
            var part = make(data);
            var chosen = edges(part).ToList();
            return () =>
            {
#if KERF_0_4_OR_LATER
                var chamfered = Operations.Chamfer(part, chosen, length,
                    options: collapse ? new ChamferOptions { TightCorners = TightCorners.Collapse } : null);
#else
                var chamfered = Operations.Chamfer(part, chosen, length);
#endif
                return chamfered.TryGetValue(out var result) ? result : (object)(chamfered.Error?.ToString() ?? "failed");
            };
        },
        outcome => ReadPart(outcome, "chamfered"), Expected.Of(name));

    // The blade bumper's enclosure, built whole: every printed part, the print aid and the bought parts placed in it
    // (renegadedynamics/hardware, enclosure/README.md). It needs Kerf 0.4.0's TightCorners and ConvexPolyhedron
    private static UserCase Enclosure() => new("enclosure", "the blade bumper's enclosure, built whole: 13 printed parts, a print aid and 34 components (bought parts, the shaft, screws, nuts and plugs) placed in it",
        data =>
        {
#if KERF_0_4_OR_LATER
            return () =>
            {
                Renegade.BladeBumper.Rebuild.Forget();
                var shapes = new List<(string, Part)>();
                shapes.AddRange(Renegade.BladeBumper.Bumper.Parts.Select(kv => ("part " + kv.Key, kv.Value)));
                shapes.AddRange(Renegade.BladeBumper.Bumper.PrintAids.Select(kv => ("aid " + kv.Key, kv.Value)));
                shapes.AddRange(Renegade.BladeBumper.Bumper.Components.Select(kv => ("component " + kv.Key, kv.Value)));
                _ = Renegade.BladeBumper.Bumper.Layout;
                return shapes;
            };
#else
            return null;
#endif
        },
        outcome => new CaseResult([.. ((List<(string Label, Part Part)>)outcome).Select(s => Measure.Of(s.Label, s.Part))], null),
        Expected.Of("enclosure"));

    // The same enclosure at v0.16 (unit 14D; renegadedynamics/hardware 9ab5fb0, enclosure-v0.16/README.md): laid out for three
    // shaft sizes, built here at 35 mm, the size the script builds by default. It needs Kerf 0.4.0's API, as v0.11 does
    private static UserCase EnclosureV016() => new("enclosure-v0.16", "the blade bumper's enclosure at v0.16, built whole for a 35 mm shaft: 13 printed parts, a print aid and 34 components placed in it",
        data =>
        {
#if KERF_0_4_OR_LATER
            return EnclosureV016Model.Build;
#else
            return null;
#endif
        },
        outcome => new CaseResult([.. ((List<(string Label, Part Part)>)outcome).Select(s => Measure.Of(s.Label, s.Part))], null),
        Expected.Of("enclosure-v0.16"));

    // Renegade's rowing blade shaft gauges (unit 15D; renegadedynamics/hardware 031bd55, shaft-gauge/README.md): every size,
    // each one's blank and its engraved gauge. It engraves text, which came in 0.5.0
    private static UserCase ShaftGauge() => new("shaft-gauge", "the rowing blade shaft gauges, all three sizes: a rounded hexagon with six U-shaped cutouts, their mouths filleted, every rim chamfered, the sizes engraved",
        data =>
        {
#if KERF_0_5_OR_LATER
            return ShaftGaugeModel.Build;
#else
            return null;
#endif
        },
        outcome => new CaseResult([.. ((List<(string Label, Part Part)>)outcome).Select(s => Measure.Of(s.Label, s.Part))], null),
        Expected.Of("shaft-gauge"));
}
