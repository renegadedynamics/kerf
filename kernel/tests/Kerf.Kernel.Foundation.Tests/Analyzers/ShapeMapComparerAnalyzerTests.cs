// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
    // Stand-ins shaped like the port's: shape structs in Occt, NCollection maps with an optional comparer
    private const string Prelude = """
        using System.Collections.Generic;
        namespace Occt
        {
            public struct TopoDS_Shape { }
            public struct TopoDS_Edge { }
            public struct gp_Pnt { }
            public sealed class TopTools_ShapeMapHasher : IEqualityComparer<TopoDS_Shape>
            {
                public static TopTools_ShapeMapHasher Instance { get; } = new();
                public bool Equals(TopoDS_Shape a, TopoDS_Shape b) => true;
                public int GetHashCode(TopoDS_Shape s) => 0;
            }
            public class NCollection_Map<TKey>
            {
                public NCollection_Map(IEqualityComparer<TKey>? theHasher = null) { }
                public NCollection_Map(NCollection_Map<TKey> theOther) { }
            }
            public class NCollection_IndexedMap<TKey>
            {
                public NCollection_IndexedMap(IEqualityComparer<TKey>? theHasher = null) { }
            }
            public class NCollection_DataMap<TKey, TValue>
            {
                public NCollection_DataMap(IEqualityComparer<TKey>? theHasher = null) { }
            }
            public class NCollection_IndexedDataMap<TKey, TValue>
            {
                public NCollection_IndexedDataMap(IEqualityComparer<TKey>? theHasher = null) { }
                public NCollection_IndexedDataMap(NCollection_IndexedDataMap<TKey, TValue> theOther) { }
            }
            public class ShapeSet : NCollection_IndexedMap<TopoDS_Shape>
            {
                public ShapeSet() { }
                public ShapeSet(IEqualityComparer<TopoDS_Shape> h) : base(h) { }
            }
        }
        """;

    [Theory]
    [InlineData("var m = new NCollection_Map<TopoDS_Shape>();")]
    [InlineData("var m = new NCollection_IndexedMap<TopoDS_Edge>();")]
    [InlineData("var m = new NCollection_DataMap<TopoDS_Shape, int>();")]
    [InlineData("var m = new NCollection_IndexedDataMap<TopoDS_Shape, int>();")]
    [InlineData("NCollection_Map<TopoDS_Shape> m = new();")]
    [InlineData("var m = new NCollection_IndexedMap<TopoDS_Shape>(null);")]
    [InlineData("var m = new HashSet<TopoDS_Shape>();")]
    [InlineData("var m = new HashSet<TopoDS_Edge>(new List<TopoDS_Edge>());")]
    [InlineData("var m = new Dictionary<TopoDS_Shape, int>();")]
    [InlineData("var m = new Dictionary<TopoDS_Edge, int>(16);")]
    [InlineData("HashSet<TopoDS_Shape> m = [];")]
    [InlineData("var m = new ShapeSet();")]
    public void Reports_a_map_of_shapes_without_a_comparer(string statement)
        => Assert.Single(Analyze(statement));

    [Theory]
    [InlineData("var m = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);")]
    [InlineData("var m = new NCollection_IndexedMap<TopoDS_Shape>(theHasher: TopTools_ShapeMapHasher.Instance);")]
    [InlineData("var m = new NCollection_DataMap<TopoDS_Shape, int>(EqualityComparer<TopoDS_Shape>.Default);")]
    [InlineData("var m = new HashSet<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);")]
    [InlineData("var m = new Dictionary<TopoDS_Shape, int>(4, TopTools_ShapeMapHasher.Instance);")]
    [InlineData("var a = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance); var b = new NCollection_Map<TopoDS_Shape>(a);")]
    [InlineData("var a = new NCollection_IndexedDataMap<TopoDS_Shape, int>(TopTools_ShapeMapHasher.Instance); var b = new NCollection_IndexedDataMap<TopoDS_Shape, int>(a);")]
    [InlineData("var m = new ShapeSet(TopTools_ShapeMapHasher.Instance);")]
    [InlineData("var m = new NCollection_Map<gp_Pnt>();")]
    [InlineData("var m = new HashSet<int>();")]
    [InlineData("var m = new Dictionary<int, TopoDS_Shape>();")]
    [InlineData("var m = new List<TopoDS_Shape>();")]
    public void Allows_an_explicit_comparer_or_another_key(string statement)
        => Assert.Empty(Analyze(statement));

    private static ImmutableArray<Diagnostic> Analyze(string statement)
    {
        string source = Prelude + "\nnamespace Occt { public static class Test { static void M() { " + statement + " } } }";
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
