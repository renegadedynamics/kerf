// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.Diagnostics;
using Kerf.Analyzers;

namespace Kerf.Kernel.Foundation.Tests.Analyzers;

// Roslyn's, not Kerf's (namespace Kerf's types come before the file's usings)
using Diagnostic = Microsoft.CodeAnalysis.Diagnostic;

public class KernelArgumentErrorAnalyzerTests
{
    // Stand-ins shaped like the kernel's argument exceptions (Standard/KernelArgumentErrors.cs)
    private const string Prelude = """
        using System;
        namespace Kerf.Engine.Foundation
        {
            internal interface IKernelArgumentError { }
            internal sealed class KernelArgumentException : ArgumentException, IKernelArgumentError
            {
                public KernelArgumentException(string message, string name) : base(message, name) { }
                // The kernel's own type copies .NET's messages: what it makes inside isn't reported
                public static KernelArgumentException Copy(string message) => throw new ArgumentException(message);
            }
            internal sealed class KernelArgumentNullException : ArgumentNullException, IKernelArgumentError
            {
                public KernelArgumentNullException(string name) : base(name) { }
                public static new void ThrowIfNull(object argument, string name = null) { if (argument is null) throw new KernelArgumentNullException(name); }
            }
            internal sealed class KernelArgumentOutOfRangeException : ArgumentOutOfRangeException, IKernelArgumentError
            {
                public KernelArgumentOutOfRangeException(string name, object value, string message) : base(name, value, message) { }
            }
        }
        """;

    [Theory]
    [InlineData("throw new ArgumentException(\"no\", \"x\");")]
    [InlineData("throw new ArgumentOutOfRangeException(\"x\", 1.0, \"no\");")]
    [InlineData("throw new ArgumentNullException(\"x\");")]
    [InlineData("var e = new ArgumentException(\"no\"); throw e;")]
    [InlineData("ArgumentException e = new(\"no\"); throw e;")]
    [InlineData("throw new System.ComponentModel.InvalidEnumArgumentException(\"x\", 1, typeof(int));")]
    [InlineData("ArgumentNullException.ThrowIfNull(o);")]
    [InlineData("ArgumentOutOfRangeException.ThrowIfNegative(d);")]
    [InlineData("ArgumentException.ThrowIfNullOrEmpty(\"\");")]
    // The kernel's type without a throw helper of that name: .NET's, inherited
    [InlineData("KernelArgumentOutOfRangeException.ThrowIfNegative(d);")]
    public void Reports_dotnets_own_argument_exception(string statement)
        => Assert.Single(Analyze(statement));

    [Theory]
    [InlineData("throw new KernelArgumentException(\"no\", \"x\");")]
    [InlineData("throw new KernelArgumentOutOfRangeException(\"x\", 1.0, \"no\");")]
    [InlineData("KernelArgumentNullException.ThrowIfNull(o);")]
    [InlineData("throw new InvalidOperationException(\"no\");")]
    [InlineData("try { } catch (ArgumentException) { throw; }")]
    public void Allows_the_kernel_s_own(string statement)
        => Assert.Empty(Analyze(statement));

    [Fact]
    public void Leaves_other_assemblies_alone()
    {
        // The modelling layer's argument exceptions are its misuse errors, and the kernel's tests throw what they like
        Assert.Empty(Analyze("throw new ArgumentException(\"no\");", "Kerf.Modeling"));
        Assert.Empty(Analyze("throw new ArgumentException(\"no\");", "Kerf.Kernel.Foundation.Tests"));
    }

    private static ImmutableArray<Diagnostic> Analyze(string statement, string assembly = "Kerf.Kernel.Probe")
    {
        string source = Prelude + "\nnamespace Kerf.Engine.Foundation { public static class Test { static void M(object o, double d) { " + statement + " } } }";
        string runtime = Path.GetDirectoryName(typeof(object).Assembly.Location)!;
        var compilation = CSharpCompilation.Create(
            assembly,
            [CSharpSyntaxTree.ParseText(source)],
            [
                MetadataReference.CreateFromFile(typeof(object).Assembly.Location),
                MetadataReference.CreateFromFile(Path.Combine(runtime, "System.Runtime.dll")),
                MetadataReference.CreateFromFile(Path.Combine(runtime, "System.ComponentModel.Primitives.dll")),
            ],
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));

        var compileErrors = compilation.GetDiagnostics().Where(d => d.Severity == DiagnosticSeverity.Error).ToList();
        Assert.True(compileErrors.Count == 0, string.Join("\n", compileErrors));

        return compilation.WithAnalyzers([new KernelArgumentErrorAnalyzer()])
                          .GetAnalyzerDiagnosticsAsync().GetAwaiter().GetResult()
                          .Where(d => d.Id == KernelArgumentErrorAnalyzer.DiagnosticId)
                          .ToImmutableArray();
    }
}
