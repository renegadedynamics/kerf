// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Microsoft.CodeAnalysis;

namespace Kerf.Tools.Rename;

/// <summary>Stable keys for symbols: the map's ids, the same in source and in metadata.</summary>
internal static class Keys
{
    /// <summary>The symbol a reference stands for: the type of a constructor, the definition of a generic use.</summary>
    public static ISymbol Normalize(ISymbol symbol)
    {
        switch (symbol)
        {
            case IMethodSymbol { MethodKind: MethodKind.Constructor or MethodKind.StaticConstructor or MethodKind.Destructor } constructor:
                return constructor.ContainingType.OriginalDefinition;
            case IMethodSymbol { ReducedFrom: { } reduced }:
                return reduced.OriginalDefinition;
            case IParameterSymbol { ContainingSymbol: IMethodSymbol { ReducedFrom: { } reducedOwner } } parameter:
                return reducedOwner.OriginalDefinition.Parameters[parameter.Ordinal + 1];
            case IParameterSymbol { ContainingSymbol: IMethodSymbol { AssociatedSymbol: IPropertySymbol indexer } } parameter
                when parameter.Ordinal < indexer.Parameters.Length:
                // An indexer's accessors have parameters of their own: they are the indexer's
                return indexer.OriginalDefinition.Parameters[parameter.Ordinal];
            case ITypeSymbol type:
                return type.OriginalDefinition;
            default:
                return symbol.OriginalDefinition;
        }
    }

    /// <summary>The root of an override chain: overrides share their root's key, so they rename together.</summary>
    public static ISymbol Root(ISymbol symbol)
    {
        switch (symbol)
        {
            case IMethodSymbol method:
                while (method.OverriddenMethod is { } overridden)
                {
                    method = overridden.OriginalDefinition;
                }

                return method;
            case IPropertySymbol property:
                while (property.OverriddenProperty is { } overridden)
                {
                    property = overridden.OriginalDefinition;
                }

                return property;
            case IEventSymbol @event:
                while (@event.OverriddenEvent is { } overridden)
                {
                    @event = overridden.OriginalDefinition;
                }

                return @event;
            default:
                return symbol;
        }
    }

    /// <summary>The map's id of a symbol, or null for symbols the map can't name (namespaces, type parameters).</summary>
    public static string? Of(ISymbol symbol)
    {
        symbol = Root(Normalize(symbol));
        switch (symbol)
        {
            case INamespaceSymbol or ITypeParameterSymbol or IAliasSymbol or IDiscardSymbol:
                return null;
            case IParameterSymbol parameter:
                return parameter.ContainingSymbol is IMethodSymbol { MethodKind: MethodKind.AnonymousFunction or MethodKind.LocalFunction }
                    ? Local(parameter)
                    // A parameter belongs to its own method, not to the root of its override chain: an override's
                    // parameter is checked for collisions in the override's own body
                    : DocumentationCommentId.CreateDeclarationId(parameter.ContainingSymbol) is { } owner ? owner + "#" + parameter.Name : null;
            case ILocalSymbol or IRangeVariableSymbol or IMethodSymbol { MethodKind: MethodKind.LocalFunction or MethodKind.AnonymousFunction }:
                return Local(symbol);
            default:
                return DocumentationCommentId.CreateDeclarationId(symbol);
        }
    }

    private static string? Local(ISymbol symbol)
        => symbol.Locations.FirstOrDefault(l => l.IsInSource) is { } location
            ? "L:" + location.SourceTree!.FilePath + ":" + location.SourceSpan.Start
            : null;
}
