// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.Diagnostics;
using Kerf.Analyzers;

namespace Kerf.Kernel.Foundation.Tests.Analyzers;

// Roslyn's, not Kerf's (namespace Kerf's types come before the file's usings)
using Diagnostic = Microsoft.CodeAnalysis.Diagnostic;

public class PublicImmutabilityAnalyzerTests
{
    [Theory]
    [InlineData("public sealed class C { public int X { get; set; } }")]
    [InlineData("public sealed class C { public int X { get; protected set; } }")]
    [InlineData("public sealed class C { public int X; }")]
    [InlineData("public struct S { private int _x; public void Bump() => _x++; }")]
    [InlineData("public sealed class C { private int _x; public void SetX(int x) => _x = x; }")]
    [InlineData("public sealed class C { public void Transform(int t) { } }")]
    [InlineData("public sealed class Outer { public sealed class C { public int X { get; set; } } }")]
    public void Reports_a_way_to_change_a_public_type(string type)
        => Assert.Single(Analyze("Kerf.Geometry", type));

    [Theory]
    [InlineData("public sealed class C { public int X { get; init; } }")]
    [InlineData("public sealed class C { public int X { get; private set; } }")]
    [InlineData("public sealed class C { public readonly int X; public const int Y = 1; }")]
    [InlineData("public readonly struct S { private readonly int _x; public S Bumped() => new(); }")]
    [InlineData("public struct S { private int _x; public readonly int X() => _x; }")]
    [InlineData("public sealed class C { internal void SetX(int x) { } public C Transformed(int t) => this; }")]
    [InlineData("public sealed class C { public void SetValues(out int x) => x = 0; }")]
    [InlineData("internal sealed class C { public int X { get; set; } }")]
    public void Allows_immutable_and_internal_members(string type)
        => Assert.Empty(Analyze("Kerf.Geometry", type));

    [Theory]
    [InlineData("Kerf.Engine.Booleans")]
    [InlineData("Occt")]
    public void Leaves_the_engine_and_other_namespaces_alone(string ns)
        => Assert.Empty(Analyze(ns, "public sealed class C { public int X { get; set; } }"));

    [Fact]
    public void Covers_the_modelling_layer()
        => Assert.Single(Analyze("Kerf.Modeling", "public sealed class C { public int X { get; set; } }"));

    [Fact]
    public void Covers_the_root_namespace()
        => Assert.Single(Analyze("Kerf", "public sealed class C { public int X { get; set; } }"));

    private static ImmutableArray<Diagnostic> Analyze(string ns, string type)
    {
        string source = $"namespace {ns} {{ {type} }}";
        var compilation = CSharpCompilation.Create(
            "Probe",
            [CSharpSyntaxTree.ParseText(source, new CSharpParseOptions(LanguageVersion.Preview))],
            [MetadataReference.CreateFromFile(typeof(object).Assembly.Location),
             MetadataReference.CreateFromFile(typeof(System.Runtime.CompilerServices.IsExternalInit).Assembly.Location)],
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));

        var compileErrors = compilation.GetDiagnostics().Where(d => d.Severity == DiagnosticSeverity.Error).ToList();
        Assert.True(compileErrors.Count == 0, string.Join("\n", compileErrors));

        return compilation.WithAnalyzers([new PublicImmutabilityAnalyzer()])
                          .GetAnalyzerDiagnosticsAsync().GetAwaiter().GetResult()
                          .Where(d => d.Id == PublicImmutabilityAnalyzer.DiagnosticId)
                          .ToImmutableArray();
    }
}
