// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.Diagnostics;
using Microsoft.CodeAnalysis.Operations;

namespace Kerf.Analyzers;

/// <summary>
/// Reports a call to a mutating struct method on a value that can't be written: a readonly field, an
/// <c>in</c> parameter, <c>this</c> in a readonly member, or a temporary such as a method's result.
/// </summary>
/// <remarks>
/// The gp types keep OCCT's in-place mutators as internal methods, so ported code can call them as the C++
/// does. C# runs such a call on a hidden copy when the value is read-only, and the change is silently lost.
/// That is a compile error here instead.
/// </remarks>
[DiagnosticAnalyzer(LanguageNames.CSharp)]
public sealed class DefensiveCopyAnalyzer : DiagnosticAnalyzer
{
    public const string DiagnosticId = "KERF0001";

    private static readonly DiagnosticDescriptor Rule = new(
        DiagnosticId,
        "Mutating call on a read-only value",
        "'{0}' changes its struct, but '{1}' is read-only here, so the call would change a copy and the change would be lost",
        "Correctness",
        DiagnosticSeverity.Error,
        isEnabledByDefault: true,
        description: "Copy the value into a local, mutate the local, and assign it back; or use the method's non-mutating form ('Transformed' for 'Transform').");

    public override ImmutableArray<DiagnosticDescriptor> SupportedDiagnostics => ImmutableArray.Create(Rule);

    public override void Initialize(AnalysisContext context)
    {
        context.ConfigureGeneratedCodeAnalysis(GeneratedCodeAnalysisFlags.None);
        context.EnableConcurrentExecution();
        context.RegisterOperationAction(AnalyzeInvocation, OperationKind.Invocation);
    }

    private static void AnalyzeInvocation(OperationAnalysisContext context)
    {
        var invocation = (IInvocationOperation)context.Operation;
        var method = invocation.TargetMethod;
        var instance = invocation.Instance;
        if (method.IsStatic || instance is null || method.IsReadOnly)
        {
            return;
        }

        var type = method.ContainingType;
        // A tuple has no method that changes it, though ValueTuple's aren't marked readonly
        if (type is null || !type.IsValueType || type.IsReadOnly || type.IsRefLikeType || type.IsTupleType)
        {
            return;
        }

        if (!IsWritable(instance, context.ContainingSymbol))
        {
            context.ReportDiagnostic(Diagnostic.Create(Rule, invocation.Syntax.GetLocation(), method.Name, instance.Syntax.ToString()));
        }
    }

    /// <summary>Whether the operation denotes a variable that a mutating call can change in place.</summary>
    internal static bool IsWritable(IOperation operation, ISymbol containingSymbol)
    {
        switch (operation)
        {
            case ILocalReferenceOperation local:
                return local.Local.RefKind != RefKind.RefReadOnly && !local.Local.IsForEach && !local.Local.IsUsing;

            case IParameterReferenceOperation parameter:
                return parameter.Parameter.RefKind is RefKind.None or RefKind.Ref or RefKind.Out;

            case IInstanceReferenceOperation:
                // `this` of a struct is writable unless the member (or the whole struct) is readonly; a constructor's
                // `this` is writable even in a readonly struct
                return containingSymbol is IMethodSymbol { MethodKind: MethodKind.Constructor }
                       || containingSymbol is not IMethodSymbol { IsReadOnly: true }
                       && containingSymbol.ContainingType is not { IsValueType: true, IsReadOnly: true };

            case IFieldReferenceOperation field:
                if (field.Field.IsReadOnly && !IsInOwnConstructor(field, containingSymbol))
                {
                    return false;
                }

                // A field of a class instance is a location of its own; a field of a struct is as writable as the struct
                return field.Instance is null || !field.Instance.Type!.IsValueType || IsWritable(field.Instance, containingSymbol);

            case IArrayElementReferenceOperation:
                return true;

            case IInlineArrayAccessOperation inlineArray:
                // An element of an [InlineArray] buffer is part of its struct: as writable as the buffer
                return IsWritable(inlineArray.Instance, containingSymbol);

            case IPropertyReferenceOperation property:
                return property.Property.ReturnsByRef;

            case IInvocationOperation call:
                return call.TargetMethod.ReturnsByRef;

            default:
                return false;
        }
    }

    private static bool IsInOwnConstructor(IFieldReferenceOperation field, ISymbol containingSymbol)
        => containingSymbol is IMethodSymbol { MethodKind: MethodKind.Constructor or MethodKind.StaticConstructor } constructor
           && SymbolEqualityComparer.Default.Equals(constructor.ContainingType, field.Field.ContainingType)
           && (field.Instance is null or IInstanceReferenceOperation);
}
