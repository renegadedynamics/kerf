// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// kerf-rename: renames to .NET conventions from a reviewed name map. See README.md.
//
//   kerf-rename draft --solution Kerf.slnx --assembly Kerf.Kernel.Foundation [--package gp ...] --seed seed.map --out draft.map
//                     [--engine-namespace Kerf.Engine]
//   kerf-rename apply --solution Kerf.slnx --assembly Kerf.Kernel.Foundation [--package gp ...] --map reviewed.map
//                     [--report report.txt] [--applied applied.map] [--rename-files] [--dry-run] [-p:Name=Value ...]

using System.Diagnostics;
using System.Text;
using Microsoft.CodeAnalysis;

namespace Kerf.Tools.Rename;

internal static class Program
{
    public static async Task<int> Main(string[] args)
    {
        if (args.Length == 0 || args[0] is not ("draft" or "apply"))
        {
            Console.Error.WriteLine("usage: kerf-rename draft|apply --solution <sln> | --project <csproj>... --assembly <name>... [--package <prefix>...] ...");
            return 2;
        }

        var options = new Dictionary<string, List<string>>(StringComparer.Ordinal);
        var properties = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase) { ["Configuration"] = "Release" };
        for (int i = 1; i < args.Length; i++)
        {
            if (args[i].StartsWith("-p:", StringComparison.Ordinal))
            {
                string[] pair = args[i][3..].Split('=', 2);
                properties[pair[0]] = pair[1];
            }
            else if (args[i] is "--dry-run" or "--rename-files")
            {
                options[args[i]] = [];
            }
            else if (args[i].StartsWith("--", StringComparison.Ordinal) && i + 1 < args.Length)
            {
                if (!options.TryGetValue(args[i], out List<string>? values))
                {
                    options[args[i]] = values = [];
                }

                values.Add(args[++i]);
            }
            else
            {
                Console.Error.WriteLine("unexpected argument: " + args[i]);
                return 2;
            }
        }

        List<string> Get(string name) => options.GetValueOrDefault(name) ?? [];
        string? One(string name) => Get(name).LastOrDefault();

        List<string> inputs = [.. Get("--solution"), .. Get("--project")];
        if (inputs.Count == 0 || Get("--assembly").Count == 0)
        {
            Console.Error.WriteLine("--solution (or --project) and --assembly are required");
            return 2;
        }

        var scope = new Scope(Get("--assembly"), Get("--package"));
        TextWriter log = Console.Out;
        var clock = Stopwatch.StartNew();
        Loaded loaded = await Loaded.OpenAsync([.. inputs.Select(Path.GetFullPath)], properties, log);

        if (args[0] == "draft")
        {
            NameMap seed = One("--seed") is { } seedPath ? NameMap.Load(seedPath) : new NameMap();
            Drafter.Draft(loaded, scope, seed, One("--out") ?? "draft.map", One("--engine-namespace"), log);
            return 0;
        }

        string mapPath = One("--map") ?? throw new ArgumentException("--map is required");
        NameMap map = NameMap.Load(mapPath);
        return await ApplyAsync(loaded, map, scope, mapPath, One("--report"), One("--applied"), options.ContainsKey("--dry-run"),
            options.ContainsKey("--rename-files"), inputs, clock, log);
    }

    private static async Task<int> ApplyAsync(Loaded loaded, NameMap map, Scope scope, string mapPath, string? reportPath, string? appliedPath,
        bool dryRun, bool renameFiles, List<string> inputs, Stopwatch clock, TextWriter log)
    {
        Plan plan = Planner.Build(loaded, map, scope, log);
        var scanner = new Scanner(plan, map, scope);
        var scans = new DocumentScan[loaded.Documents.Count];
        TimeSpan planned = clock.Elapsed;
        await Parallel.ForEachAsync(Enumerable.Range(0, loaded.Documents.Count), async (i, ct) =>
        {
            scans[i] = await scanner.ScanAsync(loaded.Documents[i], ct);
        });
        log.WriteLine($"Scanned {scans.Length} documents in {(clock.Elapsed - planned).TotalSeconds:F1} s");

        // Methods that can't become properties after all: used as delegates, or declared in a way the tool can't convert
        foreach (Occurrence occurrence in scans.SelectMany(s => s.Occurrences))
        {
            if (occurrence.Blocker is { } blocker && plan.Converts(occurrence.Key))
            {
                plan.Cancel(occurrence.Key, blocker);
            }
        }

        // Edits
        var editor = new Editor(plan);
        int files = 0, changes = 0;
        var renamedFiles = new List<(string From, string To)>();
        var generated = new SortedDictionary<string, int>(StringComparer.Ordinal);
        foreach (DocumentScan scan in scans)
        {
            (string? text, int count) = editor.Edit(scan);
            if (text is null)
            {
                continue;
            }

            files++;
            changes += count;
            if (scan.Path.EndsWith(".g.cs", StringComparison.OrdinalIgnoreCase) || scan.Text.ToString(new Microsoft.CodeAnalysis.Text.TextSpan(0, Math.Min(400, scan.Text.Length))).Contains("<auto-generated", StringComparison.Ordinal))
            {
                generated[Path.GetDirectoryName(scan.Path)!] = generated.GetValueOrDefault(Path.GetDirectoryName(scan.Path)!) + 1;
            }
            if (!dryRun)
            {
                File.WriteAllText(scan.Path, text, EncodingOf(scan.Path));
            }

            if (renameFiles && NewFileName(scan, plan) is { } newPath)
            {
                renamedFiles.Add((scan.Path, newPath));
            }
        }

        foreach ((string folder, int count) in generated)
        {
            plan.Notices.Add($"{folder}: {count} generated files edited; their generator still writes the old names");
        }

        // MSBuild's Using items
        var fullNames = new Dictionary<string, string>(StringComparer.Ordinal);
        foreach ((string key, Category category) in plan.Categories)
        {
            if (category == Category.Type && plan.NameOf(key) is { } newName && loaded.Find(key) is INamedTypeSymbol type)
            {
                string @namespace = plan.Namespaces.GetValueOrDefault(key) ?? type.ContainingNamespace.ToDisplayString();
                fullNames[type.ContainingNamespace.ToDisplayString() + "." + type.Name] = @namespace + "." + newName;
            }
        }

        var aliases = map.Aliases.Where(a => plan.NameOf("A:" + a.Key) is not null).ToDictionary(a => a.Key, a => a.Value, StringComparer.Ordinal);
        int msbuild = 0;
        foreach (string root in inputs.Select(i => Path.GetDirectoryName(Path.GetFullPath(i))!).Distinct())
        {
            foreach (string path in Directory.EnumerateFiles(root, "*.*", SearchOption.AllDirectories)
                         .Where(p => (p.EndsWith(".props", StringComparison.OrdinalIgnoreCase) || p.EndsWith(".targets", StringComparison.OrdinalIgnoreCase)
                                      || p.EndsWith(".csproj", StringComparison.OrdinalIgnoreCase)) && !Loaded.IsGenerated(p)
                                     && !p.Contains(Path.DirectorySeparatorChar + "bin" + Path.DirectorySeparatorChar, StringComparison.OrdinalIgnoreCase)))
            {
                msbuild += editor.EditMsBuild(path, fullNames, aliases, dryRun);
            }
        }

        if (!dryRun)
        {
            foreach ((string from, string to) in renamedFiles)
            {
                File.Move(from, to);
            }
        }

        log.WriteLine($"{(dryRun ? "Would change" : "Changed")} {files} files ({changes} edits, {msbuild} MSBuild Using items, {renamedFiles.Count} files renamed) in {clock.Elapsed.TotalSeconds:F1} s");
        WriteReport(reportPath, plan, scans, map, mapPath, scope, files, changes, msbuild, renamedFiles, clock.Elapsed, inputs[0], log);

        if (appliedPath is not null)
        {
            foreach ((string key, string reason) in plan.Cancelled)
            {
                if (!key.StartsWith("L:", StringComparison.Ordinal) && !key.StartsWith("A:", StringComparison.Ordinal))
                {
                    map.Keep.Add(key);
                    map.Notes["keep " + key] = reason;
                }
            }

            map.Save(appliedPath,
            [
                $"kerf-rename map as applied to {string.Join(", ", scope.Assemblies)}"
                    + (scope.Packages.Count > 0 ? " (packages " + string.Join(", ", scope.Packages) + ")" : "") + $", from {Path.GetFileName(mapPath)}",
                "The keep entries are the renames the tool dropped, with the reason: apply this map, not the reviewed one, to other repositories.",
            ]);
        }

        return 0;
    }

    private static string? NewFileName(DocumentScan scan, Plan plan)
    {
        string stem = Path.GetFileNameWithoutExtension(scan.Path);
        foreach (Occurrence occurrence in scan.Occurrences)
        {
            if (occurrence.Kind == OccurrenceKind.Name && plan.Categories.GetValueOrDefault(occurrence.Key) == Category.Type
                && scan.Text.ToString(occurrence.Span) == stem && plan.NameOf(occurrence.Key) is { } newName
                && occurrence.Key.Split('`')[0].EndsWith("." + stem, StringComparison.Ordinal))
            {
                string target = Path.Combine(Path.GetDirectoryName(scan.Path)!, newName + ".cs");
                return File.Exists(target) ? null : target;
            }
        }

        return null;
    }

    private static void WriteReport(string? path, Plan plan, DocumentScan[] scans, NameMap map, string mapPath, Scope scope, int files, int changes,
        int msbuild, List<(string From, string To)> renamedFiles, TimeSpan elapsed, string inputRoot, TextWriter log)
    {
        var report = new StringBuilder();
        report.AppendLine($"kerf-rename apply: {string.Join(", ", scope.Assemblies)}"
                          + (scope.Packages.Count > 0 ? " (packages " + string.Join(", ", scope.Packages) + ")" : "") + $" with {mapPath}");
        report.AppendLine($"{files} files changed, {changes} edits, {msbuild} MSBuild Using items, {renamedFiles.Count} files renamed, {elapsed.TotalSeconds:F1} s");
        report.AppendLine();
        report.AppendLine("Renamed, by kind (symbols, then edits):");
        var edits = scans.SelectMany(s => s.Occurrences).Where(o => plan.Names.ContainsKey(o.Key))
            .GroupBy(o => plan.Categories.GetValueOrDefault(o.Key)).ToDictionary(g => g.Key, g => g.Count());
        foreach (var group in plan.Names.Keys.GroupBy(k => plan.Categories.GetValueOrDefault(k)).OrderBy(g => g.Key))
        {
            report.AppendLine($"  {group.Key,-10} {group.Count(),7} {edits.GetValueOrDefault(group.Key),9}");
        }

        report.AppendLine($"  Converted to properties: {plan.Conversions.Keys.Count(plan.Converts)}");
        report.AppendLine();
        report.AppendLine($"Not renamed ({plan.Cancelled.Count}): collisions and blockers, not guessed");
        foreach (var group in plan.Cancelled.GroupBy(c => Kind(c.Key)).OrderBy(g => g.Key, StringComparer.Ordinal))
        {
            report.AppendLine($"  {group.Key}: {group.Count()}");
        }

        foreach ((string key, string reason) in plan.Cancelled.OrderBy(c => c.Key, StringComparer.Ordinal))
        {
            report.AppendLine($"  {key}: {reason}");
        }

        report.AppendLine();
        report.AppendLine($"Notices ({plan.Notices.Count}): seen, not changed");
        foreach (string notice in plan.Notices.Order(StringComparer.Ordinal))
        {
            report.AppendLine("  " + notice);
        }

        if (renamedFiles.Count > 0)
        {
            report.AppendLine();
            report.AppendLine("Files renamed:");
            foreach ((string from, string to) in renamedFiles)
            {
                report.AppendLine($"  {from} -> {Path.GetFileName(to)}");
            }
        }

        // Paths relative to the solution's folder
        string root = Path.GetDirectoryName(Path.GetFullPath(inputRoot))! + Path.DirectorySeparatorChar;
        report.Replace(root, "");
        if (path is not null)
        {
            File.WriteAllText(path, report.ToString());
            log.WriteLine("Report: " + path);
        }
        else
        {
            log.Write(report.ToString());
        }
    }

    private static string Kind(string key) => key.StartsWith("L:", StringComparison.Ordinal) ? "locals and lambda parameters"
        : key.Contains('#') ? "parameters"
        : key[..1] switch { "T" => "types", "F" => "fields", "M" => "methods", "P" => "properties", _ => "other" };

    /// <summary>A file's encoding as found: UTF-8 with a byte order mark only if it had one.</summary>
    public static Encoding EncodingOf(string path)
    {
        Span<byte> head = stackalloc byte[3];
        using FileStream stream = File.OpenRead(path);
        int read = stream.Read(head);
        return read == 3 && head[0] == 0xEF && head[1] == 0xBB && head[2] == 0xBF ? new UTF8Encoding(true) : new UTF8Encoding(false);
    }
}
