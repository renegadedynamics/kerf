// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using System.Linq;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.Diagnostics;

namespace Kerf.Analyzers;

/// <summary>
/// Reports a way to change a public type of a public namespace (<c>Kerf</c>, <c>Kerf.Geometry</c>, <c>Kerf.Topology</c>
/// and the other areas; not <c>Kerf.Engine.*</c>) from outside the kernel: a public or protected <c>set</c> accessor,
/// a public field that isn't <c>readonly</c> or <c>const</c>, a public member of a struct that isn't <c>readonly</c>,
/// or a public method shaped like an in-place mutator (a <c>void</c> method named <c>Set…</c>, or one of OCCT's
/// in-place operations: <c>Transform</c>, <c>Translate</c>, <c>Rotate</c>…).
/// </summary>
/// <remarks>
/// Unit 10G (phase10-plan.md 2.2 and 2.4; design-rules.md section 3, rules KERF0101 to KERF0103 there): published
/// values and geometry are immutable, and their changed copies are the returning forms (<c>Transformed</c>,
/// <c>WithPole</c>). <c>init</c> accessors are allowed. Internal members aren't reported: the engine, inside the
/// kernel, may still build an object in place before publishing it.
/// </remarks>
[DiagnosticAnalyzer(LanguageNames.CSharp)]
public sealed class PublicImmutabilityAnalyzer : DiagnosticAnalyzer
{
    public const string DiagnosticId = "KERF0003";

    private static readonly DiagnosticDescriptor Rule = new(
        DiagnosticId,
        "A public type of a public namespace can be changed",
        "'{0}' {1}; public types of Kerf's public namespaces are immutable",
        "Design",
        DiagnosticSeverity.Error,
        isEnabledByDefault: true,
        description: "Give the type a returning form (Transformed, WithX) or an init accessor in place of the mutator, or make the member internal.");

    // OCCT's in-place operations, whose returning forms are the "-ed" names
    private static readonly ImmutableHashSet<string> InPlaceNames = ImmutableHashSet.Create(
        "Transform", "Translate", "Rotate", "Scale", "Mirror", "Reverse", "UReverse", "VReverse", "Normalize",
        "Invert", "Multiply", "PreMultiply", "Add", "Subtract", "Divide", "Cross", "CrossCross", "Power", "Transpose",
        "Nullify", "Move", "Complement", "Compose", "Segment", "IncreaseDegree", "InsertKnot", "InsertKnots",
        "RemoveKnot", "RemovePole", "MovePoint", "ExchangeUV");

    public override ImmutableArray<DiagnosticDescriptor> SupportedDiagnostics => ImmutableArray.Create(Rule);

    public override void Initialize(AnalysisContext context)
    {
        context.ConfigureGeneratedCodeAnalysis(GeneratedCodeAnalysisFlags.None);
        context.EnableConcurrentExecution();
        context.RegisterSymbolAction(AnalyzeType, SymbolKind.NamedType);
    }

    /// <summary>Whether a namespace is one of Kerf's public ones: <c>Kerf</c> or <c>Kerf.X…</c>, X not <c>Engine</c>.</summary>
    internal static bool IsPublicNamespace(INamespaceSymbol? ns)
    {
        string name = ns?.ToDisplayString() ?? "";
        return name == "Kerf" || name.StartsWith("Kerf.", System.StringComparison.Ordinal)
               && !name.StartsWith("Kerf.Engine", System.StringComparison.Ordinal)
               && !name.StartsWith("Kerf.Modeling", System.StringComparison.Ordinal)
               && !name.StartsWith("Kerf.Kernel", System.StringComparison.Ordinal);
    }

    private static bool IsVisible(ISymbol symbol)
    {
        for (ISymbol? s = symbol; s is INamedTypeSymbol or IMethodSymbol or IPropertySymbol or IFieldSymbol; s = s.ContainingType)
        {
            if (s.DeclaredAccessibility is not (Accessibility.Public or Accessibility.Protected or Accessibility.ProtectedOrInternal))
            {
                return false;
            }

            if (s.ContainingType is null)
            {
                break;
            }
        }

        return true;
    }

    private static void AnalyzeType(SymbolAnalysisContext context)
    {
        var type = (INamedTypeSymbol)context.Symbol;
        if (type.TypeKind is not (TypeKind.Class or TypeKind.Struct) || !IsVisible(type) || !IsPublicNamespace(type.ContainingNamespace))
        {
            return;
        }

        foreach (ISymbol member in type.GetMembers())
        {
            if (member.IsImplicitlyDeclared || member.DeclaredAccessibility != Accessibility.Public && !(member is IPropertySymbol))
            {
                continue;
            }

            string? why = member switch
            {
                IPropertySymbol { SetMethod: { IsInitOnly: false } set } when set.DeclaredAccessibility is Accessibility.Public or Accessibility.Protected or Accessibility.ProtectedOrInternal
                    => "has a set accessor",
                IFieldSymbol { IsReadOnly: false, IsConst: false } => "is a field that isn't readonly",
                IMethodSymbol { MethodKind: MethodKind.Ordinary, IsStatic: false, IsReadOnly: false } method when type.IsValueType && !type.IsReadOnly
                    => "is a member of a struct that isn't readonly" + (method.ReturnsVoid ? ", and returns nothing" : ""),
                IMethodSymbol { MethodKind: MethodKind.Ordinary, IsStatic: false, ReturnsVoid: true } method when !type.IsValueType
                                                                                                              && method.Parameters.All(p => p.RefKind != RefKind.Out)
                                                                                                              && (method.Name.Length > 3 && method.Name.StartsWith("Set", System.StringComparison.Ordinal) && char.IsUpper(method.Name[3])
                                                                                                                  || InPlaceNames.Contains(method.Name))
                    => "changes its object in place",
                _ => null,
            };
            if (why is not null)
            {
                foreach (Location location in member.Locations)
                {
                    if (location.IsInSource)
                    {
                        context.ReportDiagnostic(Diagnostic.Create(Rule, location, type.Name + "." + member.Name, why));
                    }
                }
            }
        }
    }
}
