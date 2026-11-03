// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.Diagnostics;
using Microsoft.CodeAnalysis.Operations;

namespace Kerf.Analyzers;

/// <summary>
/// Reports a map or set keyed by a shape (TopoDS_Shape or one of its eight subtypes) created without an explicit
/// comparer: <c>NCollection_Map</c>, <c>NCollection_IndexedMap</c>, <c>NCollection_DataMap</c>,
/// <c>NCollection_IndexedDataMap</c>, <c>HashSet</c> and <c>Dictionary</c>, and classes derived from them.
/// </summary>
/// <remarks>
/// OCCT names its hasher in every map of shapes, usually <c>TopTools_ShapeMapHasher</c>, which compares shapes with
/// IsSame (the orientation is ignored). The port's maps take the hasher as an optional comparer, and the default one
/// is the shapes' own equality, IsEqual: a forgotten comparer silently changes which shapes are the same key
/// (docs/phase3-design.md, section 3). So the comparer must be written out: <c>TopTools_ShapeMapHasher.Instance</c>,
/// or <c>EqualityComparer&lt;TopoDS_Shape&gt;.Default</c> where OCCT uses <c>std::hash</c>. The NCollection maps' copy
/// constructors take the comparer of the map they copy, so they are allowed. A collection expression can't take a
/// comparer, so it is reported too.
/// </remarks>
[DiagnosticAnalyzer(LanguageNames.CSharp)]
public sealed class ShapeMapComparerAnalyzer : DiagnosticAnalyzer
{
    public const string DiagnosticId = "KERF0002";

    private static readonly DiagnosticDescriptor Rule = new(
        DiagnosticId,
        "Map of shapes without an explicit comparer",
        "'{0}' is keyed by '{1}' but is created without a comparer, so it compares keys with IsEqual; pass TopTools_ShapeMapHasher.Instance (IsSame, OCCT's TopTools maps) or EqualityComparer<{1}>.Default (IsEqual)",
        "Correctness",
        DiagnosticSeverity.Error,
        isEnabledByDefault: true,
        description: "OCCT names the hasher of every map of shapes, usually TopTools_ShapeMapHasher (IsSame). The default comparer of the port's maps is IsEqual, so the comparer must be written out.");

    // The generic collections whose first type argument is the key, by their metadata names
    private static readonly ImmutableHashSet<string> MapTypes = ImmutableHashSet.Create(
        "Occt.NCollection_Map`1",
        "Occt.NCollection_IndexedMap`1",
        "Occt.NCollection_DataMap`2",
        "Occt.NCollection_IndexedDataMap`2",
        "System.Collections.Generic.HashSet`1",
        "System.Collections.Generic.Dictionary`2");

    private static readonly ImmutableHashSet<string> ShapeTypes = ImmutableHashSet.Create(
        "TopoDS_Shape",
        "TopoDS_Vertex",
        "TopoDS_Edge",
        "TopoDS_Wire",
        "TopoDS_Face",
        "TopoDS_Shell",
        "TopoDS_Solid",
        "TopoDS_CompSolid",
        "TopoDS_Compound");

    public override ImmutableArray<DiagnosticDescriptor> SupportedDiagnostics => ImmutableArray.Create(Rule);

    public override void Initialize(AnalysisContext context)
    {
        context.ConfigureGeneratedCodeAnalysis(GeneratedCodeAnalysisFlags.None);
        context.EnableConcurrentExecution();
        context.RegisterOperationAction(AnalyzeCreation, OperationKind.ObjectCreation);
        context.RegisterOperationAction(AnalyzeCollectionExpression, OperationKind.CollectionExpression);
    }

    private static void AnalyzeCreation(OperationAnalysisContext context)
    {
        var creation = (IObjectCreationOperation)context.Operation;
        if (creation.Type is not INamedTypeSymbol type || !IsShapeMap(type, out INamedTypeSymbol? map, out ITypeSymbol? key))
        {
            return;
        }

        foreach (IArgumentOperation argument in creation.Arguments)
        {
            if (argument.ArgumentKind == ArgumentKind.DefaultValue || argument.Parameter is null)
            {
                continue;
            }

            ITypeSymbol parameterType = argument.Parameter.Type;
            if (IsEqualityComparer(parameterType))
            {
                // an explicit null is the default comparer
                if (argument.Value.ConstantValue is { HasValue: true, Value: null })
                {
                    continue;
                }

                return;
            }

            // The NCollection maps' copy constructors keep the comparer of the map they copy
            if (map!.ContainingNamespace?.ToDisplayString() == "Occt" && creation.Arguments.Length == 1
                && parameterType is INamedTypeSymbol named
                && SymbolEqualityComparer.Default.Equals(named.OriginalDefinition, map.OriginalDefinition))
            {
                return;
            }
        }

        context.ReportDiagnostic(Diagnostic.Create(Rule, creation.Syntax.GetLocation(), type.Name, key!.Name));
    }

    private static void AnalyzeCollectionExpression(OperationAnalysisContext context)
    {
        var expression = (ICollectionExpressionOperation)context.Operation;
        if (expression.Type is INamedTypeSymbol type && IsShapeMap(type, out _, out ITypeSymbol? key))
        {
            context.ReportDiagnostic(Diagnostic.Create(Rule, expression.Syntax.GetLocation(), type.Name, key!.Name));
        }
    }

    /// <summary>Whether the type is, or derives from, one of the maps keyed by a shape type.</summary>
    internal static bool IsShapeMap(INamedTypeSymbol type, out INamedTypeSymbol? map, out ITypeSymbol? key)
    {
        for (INamedTypeSymbol? current = type; current is not null; current = current.BaseType)
        {
            if (current.IsGenericType && MapTypes.Contains(MetadataName(current.OriginalDefinition))
                && IsShapeType(current.TypeArguments[0]))
            {
                map = current;
                key = current.TypeArguments[0];
                return true;
            }
        }

        map = null;
        key = null;
        return false;
    }

    private static bool IsShapeType(ITypeSymbol type)
        => type.IsValueType && ShapeTypes.Contains(type.Name) && type.ContainingNamespace?.ToDisplayString() == "Occt";

    private static bool IsEqualityComparer(ITypeSymbol type)
        => type is INamedTypeSymbol { IsGenericType: true } named
           && MetadataName(named.OriginalDefinition) == "System.Collections.Generic.IEqualityComparer`1";

    private static string MetadataName(INamedTypeSymbol type)
    {
        string ns = type.ContainingNamespace is { IsGlobalNamespace: false } containing ? containing.ToDisplayString() + "." : "";
        return ns + type.MetadataName;
    }
}
