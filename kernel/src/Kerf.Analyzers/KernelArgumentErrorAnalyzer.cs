// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.Diagnostics;
using Microsoft.CodeAnalysis.Operations;

namespace Kerf.Analyzers;

/// <summary>
/// Reports an argument exception the kernel makes as .NET's own type: a <c>new</c> of <c>ArgumentException</c> or a type
/// derived from it that isn't the kernel's (<c>Kerf.Engine.Foundation.IKernelArgumentError</c>), and a call of .NET's
/// throw helpers on those types (<c>ArgumentNullException.ThrowIfNull</c>, <c>ArgumentOutOfRangeException.ThrowIfNegative</c>…).
/// </summary>
/// <remarks>
/// The modelling layer tells the kernel's refusal of a value from its own misuse by the exception's type (decision 25801).
/// It once did by the assembly of the frame that threw, which the JIT's inlining changes once a call is hot, so the same
/// input gave a different exception: a kernel argument exception of .NET's type would bring that back. The kernel's own
/// types (<c>KernelArgumentException</c>, <c>KernelArgumentOutOfRangeException</c>, <c>KernelArgumentNullException</c>)
/// have the same constructors and throw helpers, and are exempt, as is the code inside them, which copies .NET's messages.
/// </remarks>
[DiagnosticAnalyzer(LanguageNames.CSharp)]
public sealed class KernelArgumentErrorAnalyzer : DiagnosticAnalyzer
{
    public const string DiagnosticId = "KERF0004";

    private static readonly DiagnosticDescriptor Rule = new(
        DiagnosticId,
        "Kernel argument exception of .NET's type",
        "'{0}' is .NET's argument exception; the kernel throws its own, KernelArgument{1}, so the modelling layer can tell the kernel's refusal by its type",
        "Correctness",
        DiagnosticSeverity.Error,
        isEnabledByDefault: true,
        description: "Every argument exception the kernel makes is one of its own three types, which the modelling layer tells by type wherever the JIT compiled the check (decision 25801).");

    private const string MarkerName = "Kerf.Engine.Foundation.IKernelArgumentError";

    public override ImmutableArray<DiagnosticDescriptor> SupportedDiagnostics => ImmutableArray.Create(Rule);

    public override void Initialize(AnalysisContext context)
    {
        context.ConfigureGeneratedCodeAnalysis(GeneratedCodeAnalysisFlags.None);
        context.EnableConcurrentExecution();
        context.RegisterCompilationStartAction(start =>
        {
            INamedTypeSymbol? argument = start.Compilation.GetTypeByMetadataName("System.ArgumentException");
            if (argument is null || !IsKernelAssembly(start.Compilation.AssemblyName))
            {
                return;
            }

            start.RegisterOperationAction(c => AnalyzeCreation(c, argument), OperationKind.ObjectCreation);
            start.RegisterOperationAction(c => AnalyzeInvocation(c, argument), OperationKind.Invocation);
        });
    }

    private static void AnalyzeCreation(OperationAnalysisContext context, INamedTypeSymbol argument)
    {
        var creation = (IObjectCreationOperation)context.Operation;
        if (creation.Type is INamedTypeSymbol type && IsForeignArgumentError(type, argument) && !InsideKernelType(context))
        {
            context.ReportDiagnostic(Diagnostic.Create(Rule, creation.Syntax.GetLocation(), type.Name, Suffix(type)));
        }
    }

    private static void AnalyzeInvocation(OperationAnalysisContext context, INamedTypeSymbol argument)
    {
        var invocation = (IInvocationOperation)context.Operation;
        INamedTypeSymbol type = invocation.TargetMethod.ContainingType;
        if (invocation.TargetMethod.IsStatic && invocation.TargetMethod.Name.StartsWith("ThrowIf", System.StringComparison.Ordinal)
            && IsForeignArgumentError(type, argument) && !InsideKernelType(context))
        {
            context.ReportDiagnostic(Diagnostic.Create(Rule, invocation.Syntax.GetLocation(), type.Name + "." + invocation.TargetMethod.Name, Suffix(type)));
        }
    }

    // The kernel's own assemblies (Kerf.Kernel.Foundation and the rest): not the modelling layer, whose argument
    // exceptions are its misuse errors, nor the kernel's tests, tools and benchmarks, which the analyzer reaches too
    private static bool IsKernelAssembly(string? name)
        => name is not null && name.StartsWith("Kerf.Kernel.", System.StringComparison.Ordinal)
           && !name.EndsWith(".Tests", System.StringComparison.Ordinal)
           && name is not ("Kerf.Kernel.DrawLite" or "Kerf.Kernel.Benchmarks");

    // ArgumentException or a type derived from it, not marked as the kernel's
    private static bool IsForeignArgumentError(INamedTypeSymbol type, INamedTypeSymbol argument)
    {
        bool derived = false;
        for (INamedTypeSymbol? current = type; current is not null; current = current.BaseType)
        {
            if (SymbolEqualityComparer.Default.Equals(current, argument))
            {
                derived = true;
                break;
            }
        }

        return derived && !IsKernelType(type);
    }

    private static bool IsKernelType(INamedTypeSymbol type)
    {
        foreach (INamedTypeSymbol implemented in type.AllInterfaces)
        {
            if (implemented.ToDisplayString() == MarkerName)
            {
                return true;
            }
        }

        return false;
    }

    // The kernel's types make .NET's exceptions to copy their messages
    private static bool InsideKernelType(OperationAnalysisContext context)
    {
        for (INamedTypeSymbol? type = context.ContainingSymbol.ContainingType; type is not null; type = type.ContainingType)
        {
            if (IsKernelType(type))
            {
                return true;
            }
        }

        return false;
    }

    // The kernel's type to use instead: KernelArgumentException for ArgumentException, and so on
    private static string Suffix(INamedTypeSymbol type) => type.Name switch
    {
        "ArgumentOutOfRangeException" => "OutOfRangeException",
        "ArgumentNullException" => "NullException",
        _ => "Exception",
    };
}
