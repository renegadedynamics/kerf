// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text.RegularExpressions;
using System.Xml.Linq;
using Kerf.Tests.Shared;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// The kernel never builds on the modelling layer (unit 15J): no project under kernel/src or kernel/tests references
/// Kerf.Modeling, directly or through another project, so a change to the modelling layer alone can't break a kernel
/// suite, and the merge gate (kernel/tools/merge/choose-suites.pl) may leave them out for one. The references are read
/// from the .csproj files and every Directory.Build.props above them, as MSBuild would find them.
/// </summary>
public partial class ProjectGraphTests
{
    private static readonly string Root = ApiSurface.RepositoryRoot();

    private static string Modeling => Path.GetFullPath(Path.Combine(Root, "modeling", "src", "Kerf.Modeling", "Kerf.Modeling.csproj"));

    private static IEnumerable<string> KernelProjects()
        => new[] { "src", "tests" }
            .SelectMany(d => Directory.EnumerateFiles(Path.Combine(Root, "kernel", d), "*.csproj", SearchOption.AllDirectories))
            .Where(p => !p.Contains($"{Path.DirectorySeparatorChar}bin{Path.DirectorySeparatorChar}", StringComparison.Ordinal)
                && !p.Contains($"{Path.DirectorySeparatorChar}obj{Path.DirectorySeparatorChar}", StringComparison.Ordinal))
            .Select(Path.GetFullPath)
            .Order(StringComparer.Ordinal);

    [GeneratedRegex(@"\$\((\w+)\)")]
    private static partial Regex Property();

    // The project's own references and those of every Directory.Build.props from its folder up to the repository's root
    // (more than MSBuild imports, never fewer), each resolved to a full path
    private static IEnumerable<string> References(string project)
    {
        var files = new List<string> { project };
        for (string? dir = Path.GetDirectoryName(project); dir is not null && dir.Length >= Root.Length; dir = Path.GetDirectoryName(dir))
        {
            string props = Path.Combine(dir, "Directory.Build.props");
            if (File.Exists(props))
            {
                files.Add(props);
            }
        }

        foreach (string file in files)
        {
            string fileDir = Path.GetDirectoryName(file)! + Path.DirectorySeparatorChar;
            foreach (XElement reference in XDocument.Load(file).Descendants("ProjectReference"))
            {
                string include = (string?)reference.Attribute("Include") ?? "";
                string resolved = Property().Replace(include, m => m.Groups[1].Value switch
                {
                    "MSBuildThisFileDirectory" => fileDir,
                    "KerfModelingRoot" => Path.Combine(Root, "modeling") + Path.DirectorySeparatorChar,
                    "KerfKernelRoot" => Path.Combine(Root, "kernel") + Path.DirectorySeparatorChar,
                    _ => throw new InvalidOperationException($"{file}: {include} names a property this test doesn't know"),
                });
                string full = Path.GetFullPath(Path.Combine(fileDir, resolved.Replace('\\', Path.DirectorySeparatorChar)));
                if (full != project)
                {
                    yield return full;
                }
            }
        }
    }

    // Every project the given one builds on, through references, with the path of references that reached each
    private static Dictionary<string, string> Reached(string project)
    {
        var via = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase) { [project] = Path.GetFileName(project) };
        var todo = new Queue<string>([project]);
        while (todo.TryDequeue(out string? next))
        {
            foreach (string reference in References(next))
            {
                Assert.True(File.Exists(reference), $"{next} references {reference}, which doesn't exist");
                if (via.TryAdd(reference, via[next] + " -> " + Path.GetFileName(reference)))
                {
                    todo.Enqueue(reference);
                }
            }
        }

        return via;
    }

    [Fact]
    public void No_kernel_project_reaches_the_modelling_layer()
    {
        var reaching = KernelProjects()
            .Select(Reached)
            .Where(r => r.ContainsKey(Modeling))
            .Select(r => r[Modeling])
            .ToList();
        Assert.True(reaching.Count == 0, "The kernel builds on Kerf.Modeling: " + string.Join("; ", reaching));
    }

    [Fact]
    public void The_walk_finds_what_it_looks_for()
    {
        // the check above would pass on a walk that found nothing: the users' cases do reach Kerf.Modeling, written with
        // a property ($(KerfModelingRoot))
        Assert.True(KernelProjects().Count() >= 10);
        var userCases = Reached(Path.GetFullPath(Path.Combine(Root, "kernel", "benchmarks", "Kerf.UserCases", "Kerf.UserCases.csproj")));
        Assert.Contains(Modeling, userCases.Keys);
        // through another project: the kernel's DataExchange, which Kerf.Modeling references
        Assert.Contains(userCases.Keys, k => k.EndsWith("Kerf.Kernel.Foundation.csproj", StringComparison.Ordinal));
        // and through a reference made in a Directory.Build.props: the analyzers, from kernel/src's
        Assert.Contains(userCases.Keys, k => k.EndsWith("Kerf.Analyzers.csproj", StringComparison.Ordinal));
    }
}
