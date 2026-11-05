// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.Diagnostics;
using Kerf.Analyzers;

namespace Kerf.Kernel.Foundation.Tests.Analyzers;

// Roslyn's, not Kerf's (namespace Kerf's types come before the file's usings)
using Diagnostic = Microsoft.CodeAnalysis.Diagnostic;

public class DefensiveCopyAnalyzerTests
{
    // A struct shaped like the gp types: a readonly query and an internal mutator
    private const string Prelude = """
        public struct P
        {
            private double x;
            public readonly double X() => x;
            internal void Move(double d) => x += d;
            public readonly P Moved(double d) { P p = this; p.Move(d); return p; }
        }
        public readonly struct R { public void Touch() { } }
        public sealed class Holder
        {
            public P Mutable;
            public readonly P Fixed;
            public P Prop { get; set; }
            public P Get() => Mutable;
            public ref P GetRef() => ref Mutable;
        }
        """;

    [Theory]
    [InlineData("void M(Holder h) { h.Fixed.Move(1); }")]
    [InlineData("void M(in P p) { p.Move(1); }")]
    [InlineData("void M(Holder h) { h.Prop.Move(1); }")]
    [InlineData("void M(Holder h) { h.Get().Move(1); }")]
    [InlineData("void M(P[] ps) { foreach (var p in ps) { p.Move(1); } }")]
    [InlineData("void M(ref readonly P p) { p.Move(1); }")]
    public void Reports_mutation_of_a_read_only_value(string method)
        => Assert.Single(Analyze(method));

    [Theory]
    [InlineData("void M(P p) { p.Move(1); }")]
    [InlineData("void M(ref P p) { p.Move(1); }")]
    [InlineData("void M(Holder h) { h.Mutable.Move(1); }")]
    [InlineData("void M(Holder h) { h.GetRef().Move(1); }")]
    [InlineData("void M(P[] ps) { ps[0].Move(1); }")]
    [InlineData("void M(in P p) { double x = p.X(); P q = p.Moved(x); }")]
    [InlineData("void M(R r) { r.Touch(); }")]
    public void Allows_mutation_of_a_variable(string method)
        => Assert.Empty(Analyze(method));

    [Fact]
    public void Reports_a_mutator_called_from_a_readonly_member()
        => Assert.Single(Analyze("", extraStruct: "public struct Q { private P p; public readonly void M() { p.Move(1); } }"));

    [Fact]
    public void Treats_inline_array_elements_as_part_of_their_struct()
    {
        const string Buffer = "[System.Runtime.CompilerServices.InlineArray(3)] public struct Three { private P e; }";
        Assert.Empty(Analyze("", extraStruct: Buffer + " public struct S { private Three b; public void M() { b[1].Move(1); } }"));
        Assert.Single(Analyze("", extraStruct: Buffer + " public struct S { private Three b; public readonly void M() { b[1].Move(1); } }"));
    }

    [Fact]
    public void Allows_a_readonly_field_in_its_own_constructor()
        => Assert.Empty(Analyze("", extraStruct: "public sealed class C { private readonly P p; public C() { p.Move(1); } }"));

    [Fact]
    public void Allows_a_field_of_a_readonly_struct_in_its_constructor()
    {
        Assert.Empty(Analyze("", extraStruct: "public readonly struct T { private readonly P p; public T(double d) { p = default; p.Move(d); } }"));
        Assert.Single(Analyze("", extraStruct: "public readonly struct T { private readonly P p; public T Moved() { p.Move(1); return this; } }"));
    }

    private static ImmutableArray<Diagnostic> Analyze(string method, string extraStruct = "")
    {
        string source = Prelude + extraStruct + (method.Length > 0 ? "\npublic static class Test { static " + method + " }" : "");
        var compilation = CSharpCompilation.Create(
            "Probe",
            [CSharpSyntaxTree.ParseText(source)],
            [MetadataReference.CreateFromFile(typeof(object).Assembly.Location)],
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));

        var compileErrors = compilation.GetDiagnostics().Where(d => d.Severity == DiagnosticSeverity.Error).ToList();
        Assert.True(compileErrors.Count == 0, string.Join("\n", compileErrors));

        return compilation.WithAnalyzers([new DefensiveCopyAnalyzer()])
                          .GetAnalyzerDiagnosticsAsync().GetAwaiter().GetResult()
                          .Where(d => d.Id == DefensiveCopyAnalyzer.DiagnosticId)
                          .ToImmutableArray();
    }
}
