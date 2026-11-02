// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using System.Diagnostics;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.MSBuild;

namespace Kerf.Tools.Rename;

/// <summary>A loaded solution, compiled, with the documents worth scanning.</summary>
internal sealed class Loaded
{
    public required Solution Solution { get; init; }
    public required IReadOnlyDictionary<ProjectId, Compilation> Compilations { get; init; }

    /// <summary>One document per file: a file compiled into several projects is scanned once.</summary>
    public required IReadOnlyList<Document> Documents { get; init; }

    public required MSBuildWorkspace Workspace { get; init; }

    public static async Task<Loaded> OpenAsync(IReadOnlyList<string> paths, IReadOnlyDictionary<string, string> properties, TextWriter log)
    {
        var clock = Stopwatch.StartNew();
        var workspace = MSBuildWorkspace.Create(properties.ToDictionary());
        workspace.RegisterWorkspaceFailedHandler(e =>
        {
            if (e.Diagnostic.Kind == WorkspaceDiagnosticKind.Failure)
            {
                log.WriteLine("workspace: " + e.Diagnostic.Message);
            }
        });

        Solution solution;
        if (paths.Count == 1 && (paths[0].EndsWith(".sln", StringComparison.OrdinalIgnoreCase) || paths[0].EndsWith(".slnx", StringComparison.OrdinalIgnoreCase)))
        {
            solution = await workspace.OpenSolutionAsync(paths[0]);
        }
        else
        {
            foreach (string project in paths)
            {
                await workspace.OpenProjectAsync(project);
            }

            solution = workspace.CurrentSolution;
        }

        // Parse documentation comments everywhere, so crefs and parameter names in them bind
        foreach (Project project in solution.Projects)
        {
            if (project.ParseOptions is CSharpParseOptions options && options.DocumentationMode < DocumentationMode.Parse)
            {
                solution = solution.WithProjectParseOptions(project.Id, options.WithDocumentationMode(DocumentationMode.Parse));
            }
        }

        log.WriteLine($"Loaded {solution.Projects.Count()} projects in {clock.Elapsed.TotalSeconds:F1} s");

        var compilations = new ConcurrentDictionary<ProjectId, Compilation>();
        await Parallel.ForEachAsync(solution.Projects, async (project, ct) =>
        {
            Compilation compilation = (await project.GetCompilationAsync(ct))!;
            int errors = compilation.GetDiagnostics(ct).Count(d => d.Severity == DiagnosticSeverity.Error);
            if (errors > 0)
            {
                log.WriteLine($"warning: {project.Name} has {errors} compile errors before renaming; references in broken code may be missed");
            }

            compilations[project.Id] = compilation;
        });
        log.WriteLine($"Compiled in {clock.Elapsed.TotalSeconds:F1} s");

        var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        var documents = new List<Document>();
        foreach (Project project in solution.Projects.OrderBy(p => p.Name, StringComparer.Ordinal))
        {
            foreach (Document document in project.Documents)
            {
                if (document.FilePath is { } path && path.EndsWith(".cs", StringComparison.OrdinalIgnoreCase) && !IsGenerated(path) && seen.Add(path))
                {
                    documents.Add(document);
                }
            }
        }

        return new Loaded { Solution = solution, Compilations = compilations, Documents = documents, Workspace = workspace };
    }

    /// <summary>
    /// Files under obj (global usings, assembly attributes) come from MSBuild: they're regenerated, not edited. Generated
    /// files that are committed (a generator's output) are edited like any other, and reported.
    /// </summary>
    public static bool IsGenerated(string path)
        => path.Contains(Path.DirectorySeparatorChar + "obj" + Path.DirectorySeparatorChar, StringComparison.OrdinalIgnoreCase);

    /// <summary>Every type declared in source, nested ones included.</summary>
    public IEnumerable<INamedTypeSymbol> SourceTypes()
    {
        var seen = new HashSet<ISymbol>(SymbolEqualityComparer.Default);
        foreach (Compilation compilation in Compilations.Values)
        {
            foreach (INamedTypeSymbol type in AllTypes(compilation.Assembly.GlobalNamespace))
            {
                if (type.Locations.Any(l => l.IsInSource) && seen.Add(type))
                {
                    yield return type;
                }
            }
        }
    }

    private static IEnumerable<INamedTypeSymbol> AllTypes(INamespaceOrTypeSymbol container)
    {
        foreach (ISymbol member in container.GetMembers())
        {
            if (member is INamespaceOrTypeSymbol inner)
            {
                if (inner is INamedTypeSymbol type)
                {
                    yield return type;
                }

                foreach (INamedTypeSymbol nested in AllTypes(inner))
                {
                    yield return nested;
                }
            }
        }
    }

    /// <summary>The symbol a map id names, in source or in a referenced assembly.</summary>
    public ISymbol? Find(string id)
    {
        foreach (Compilation compilation in Compilations.Values)
        {
            if (DocumentationCommentId.GetFirstSymbolForDeclarationId(id, compilation) is { } symbol)
            {
                return symbol;
            }
        }

        return null;
    }
}
