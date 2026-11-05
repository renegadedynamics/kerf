// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.Diagnostics;
using Kerf.Analyzers;

namespace Kerf.Kernel.Foundation.Tests.Analyzers;

// Roslyn's, not Kerf's (namespace Kerf's types come before the file's usings)
using Diagnostic = Microsoft.CodeAnalysis.Diagnostic;

public class ShapeMapComparerAnalyzerTests
{
    // Stand-ins shaped like Kerf's: shape structs in Kerf.Topology, indexed collections with an optional comparer
    private const string Prelude = """
        using System.Collections.Generic;
        namespace Kerf.Topology
        {
            public struct Shape { }
            public struct Edge { }
        }
        namespace Kerf.Geometry
        {
            public struct Point3D { }
        }
        namespace Kerf.Engine.Collections
        {
            using Kerf.Topology;
            public sealed class ShapeHasher : IEqualityComparer<Shape>
            {
                public static ShapeHasher Instance { get; } = new();
                public bool Equals(Shape a, Shape b) => true;
                public int GetHashCode(Shape s) => 0;
            }
            public class IndexedSet<TKey>
            {
                public IndexedSet(IEqualityComparer<TKey>? comparer = null) { }
                public IndexedSet(IndexedSet<TKey> other) { }
            }
            public class IndexedMap<TKey, TValue>
            {
                public IndexedMap(IEqualityComparer<TKey>? comparer = null) { }
                public IndexedMap(IndexedMap<TKey, TValue> other) { }
            }
            public class ShapeSet : IndexedSet<Shape>
            {
                public ShapeSet() { }
                public ShapeSet(IEqualityComparer<Shape> h) : base(h) { }
            }
            public sealed class OccShape { }
        }
        """;

    [Theory]
    [InlineData("var m = new HashSet<Shape>(16);")]
    [InlineData("var m = new IndexedSet<Edge>();")]
    [InlineData("Dictionary<Edge, int> m = new();")]
    [InlineData("var m = new IndexedMap<Shape, int>();")]
    [InlineData("IndexedSet<Shape> m = new();")]
    [InlineData("var m = new IndexedSet<Shape>((IEqualityComparer<Shape>?)null);")]
    [InlineData("var m = new HashSet<Shape>();")]
    [InlineData("var m = new HashSet<Edge>(new List<Edge>());")]
    [InlineData("var m = new Dictionary<Shape, int>();")]
    [InlineData("var m = new Dictionary<Edge, int>(16);")]
    [InlineData("HashSet<Shape> m = [];")]
    [InlineData("var m = new ShapeSet();")]
    public void Reports_a_map_of_shapes_without_a_comparer(string statement)
        => Assert.Single(Analyze(statement));

    [Theory]
    [InlineData("var m = new IndexedSet<Shape>(ShapeHasher.Instance);")]
    [InlineData("var m = new IndexedSet<Shape>(comparer: ShapeHasher.Instance);")]
    [InlineData("var m = new IndexedMap<Shape, int>(EqualityComparer<Shape>.Default);")]
    [InlineData("var m = new HashSet<Shape>(ShapeHasher.Instance);")]
    [InlineData("var m = new Dictionary<Shape, int>(4, ShapeHasher.Instance);")]
    [InlineData("var a = new IndexedSet<Shape>(ShapeHasher.Instance); var b = new IndexedSet<Shape>(a);")]
    [InlineData("var a = new IndexedMap<Shape, int>(ShapeHasher.Instance); var b = new IndexedMap<Shape, int>(a);")]
    [InlineData("var m = new ShapeSet(ShapeHasher.Instance);")]
    [InlineData("var m = new IndexedSet<Point3D>();")]
    [InlineData("var m = new HashSet<int>();")]
    [InlineData("var m = new Dictionary<int, Shape>();")]
    [InlineData("var m = new List<Shape>();")]
    [InlineData("var m = new HashSet<OccShape>();")]
    public void Allows_an_explicit_comparer_or_another_key(string statement)
        => Assert.Empty(Analyze(statement));

    private static ImmutableArray<Diagnostic> Analyze(string statement)
    {
        string source = Prelude + "\nnamespace Kerf.Engine.Collections { using Kerf.Topology; using Kerf.Geometry; public static class Test { static void M() { " + statement + " } } }";
        var compilation = CSharpCompilation.Create(
            "Probe",
            [CSharpSyntaxTree.ParseText(source)],
            [
                MetadataReference.CreateFromFile(typeof(object).Assembly.Location),
                MetadataReference.CreateFromFile(typeof(HashSet<>).Assembly.Location),
                MetadataReference.CreateFromFile(Path.Combine(Path.GetDirectoryName(typeof(object).Assembly.Location)!, "System.Runtime.dll")),
                MetadataReference.CreateFromFile(Path.Combine(Path.GetDirectoryName(typeof(object).Assembly.Location)!, "System.Collections.dll")),
            ],
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary, nullableContextOptions: NullableContextOptions.Enable));

        var compileErrors = compilation.GetDiagnostics().Where(d => d.Severity == DiagnosticSeverity.Error).ToList();
        Assert.True(compileErrors.Count == 0, string.Join("\n", compileErrors));

        return compilation.WithAnalyzers([new ShapeMapComparerAnalyzer()])
                          .GetAnalyzerDiagnosticsAsync().GetAwaiter().GetResult()
                          .Where(d => d.Id == ShapeMapComparerAnalyzer.DiagnosticId)
                          .ToImmutableArray();
    }
}
