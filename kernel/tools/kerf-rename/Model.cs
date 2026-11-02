// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.Text;

namespace Kerf.Tools.Rename;

/// <summary>Which declarations a run renames: those of some assemblies, optionally only some of their packages.</summary>
internal sealed class Scope(IEnumerable<string> assemblies, IEnumerable<string> packages)
{
    private readonly HashSet<string> _assemblies = new(assemblies, StringComparer.OrdinalIgnoreCase);
    private readonly string[] _packages = [.. packages];

    public IReadOnlyCollection<string> Assemblies => _assemblies;

    public IReadOnlyList<string> Packages => _packages;

    public bool Contains(ISymbol? symbol)
    {
        if (symbol is null || symbol.ContainingAssembly is null || !_assemblies.Contains(symbol.ContainingAssembly.Name))
        {
            return false;
        }

        if (_packages.Length == 0)
        {
            return true;
        }

        INamedTypeSymbol? type = symbol as INamedTypeSymbol ?? symbol.ContainingType;
        while (type?.ContainingType is { } outer)
        {
            type = outer;
        }

        return type is not null && _packages.Any(p => type.Name == p || type.Name.StartsWith(p + "_", StringComparison.Ordinal));
    }
}

internal enum Category
{
    Type,
    Member,
    Property,
    Field,
    Static,
    Parameter,
    Local,
    Alias,
}

/// <summary>What the run will rename: every key's new simple name, and what was cancelled and why.</summary>
internal sealed class Plan
{
    public ConcurrentDictionary<string, string> Names { get; } = new(StringComparer.Ordinal);
    public ConcurrentDictionary<string, Category> Categories { get; } = new(StringComparer.Ordinal);

    /// <summary>Type key to its new namespace, for types that move.</summary>
    public ConcurrentDictionary<string, string> Namespaces { get; } = new(StringComparer.Ordinal);

    /// <summary>Type key to its old namespace, for types that move.</summary>
    public ConcurrentDictionary<string, string> OldNamespaces { get; } = new(StringComparer.Ordinal);

    /// <summary>Methods that become properties (their keys).</summary>
    public ConcurrentDictionary<string, byte> Conversions { get; } = new(StringComparer.Ordinal);

    /// <summary>Key to the reason a rename was dropped, for the report and for the applied map's keep entries.</summary>
    public ConcurrentDictionary<string, string> Cancelled { get; } = new(StringComparer.Ordinal);

    /// <summary>Old simple names of everything planned: the tokens worth binding.</summary>
    public ConcurrentDictionary<string, byte> OldNames { get; } = new(StringComparer.Ordinal);

    /// <summary>New simple names of the types that move to another namespace.</summary>
    public ConcurrentDictionary<string, byte> MovedTypeNames { get; } = new(StringComparer.Ordinal);

    /// <summary>Old simple names of the types planned, for noticing strings that name them.</summary>
    public ConcurrentDictionary<string, byte> TypeOldNames { get; } = new(StringComparer.Ordinal);

    /// <summary>Things the tool saw but doesn't change: strings naming renamed types, string type names, and so on.</summary>
    public ConcurrentBag<string> Notices { get; } = [];

    public void Add(string key, string oldName, string newName, Category category)
    {
        if (oldName == newName)
        {
            return;
        }

        Names[key] = newName;
        Categories[key] = category;
        OldNames[oldName] = 0;
    }

    public void Cancel(string key, string reason)
    {
        if (Names.TryRemove(key, out _))
        {
            Cancelled[key] = reason;
        }

        Conversions.TryRemove(key, out _);
        Namespaces.TryRemove(key, out _);
    }

    public string? NameOf(string key) => Names.TryGetValue(key, out string? name) ? name : null;

    public bool Converts(string key) => Conversions.ContainsKey(key) && Names.ContainsKey(key);
}

internal enum OccurrenceKind
{
    /// <summary>A name to replace.</summary>
    Name,

    /// <summary>A call of a method that becomes a property: <see cref="Occurrence.Extra"/> is the argument list.</summary>
    Invoked,

    /// <summary>A method used as a delegate: the method can't become a property.</summary>
    MethodGroup,

    /// <summary>A cref to a method: <see cref="Occurrence.Extra"/> is its parameter list, if any.</summary>
    Cref,

    /// <summary>A method declared with <c>=&gt;</c>: <see cref="Occurrence.Extra"/> is its parameter list.</summary>
    DeclaredExpression,

    /// <summary>A method declared without a body: <see cref="Occurrence.Extra"/> runs from its parameter list to its semicolon.</summary>
    DeclaredAbstract,

    /// <summary>A method declared with a block: <see cref="Occurrence.Extra"/> is its parameter list, <see cref="Occurrence.Body"/> its block.</summary>
    DeclaredBlock,

    /// <summary>A type named after a namespace: <see cref="Occurrence.Extra"/> is the namespace's part.</summary>
    TypeQualified,

    /// <summary>A type named on its own, in code whose namespace is <see cref="Occurrence.Namespace"/>.</summary>
    TypeUnqualified,

    /// <summary>A string naming a member for <c>UnsafeAccessor</c>: the span is the literal's.</summary>
    AccessorName,
}

internal sealed record Occurrence(TextSpan Span, string Key, OccurrenceKind Kind)
{
    public TextSpan Extra { get; init; }
    public TextSpan Body { get; init; }

    /// <summary>For a type named on its own: the enclosing namespace declaration's span start (-1 for none) and full name.</summary>
    public int NamespaceDeclaration { get; init; } = -1;

    public string Namespace { get; init; } = "";

    /// <summary>The new simple name would mean something else here: qualify it.</summary>
    public bool Ambiguous { get; init; }

    /// <summary>For an ambiguous simple name: what to write instead (<c>this.X</c>, or the member qualified in full).</summary>
    public string? Qualified { get; init; }

    /// <summary>Inside an interpolated string's hole, where <c>global::</c> would read as a format.</summary>
    public bool InInterpolation { get; init; }

    /// <summary>The qualifier was written <c>global::</c> or through an alias: write the new one in full.</summary>
    public bool GlobalQualifier { get; init; }

    /// <summary>Why a method declared here can't become a property, if it can't.</summary>
    public string? Blocker { get; init; }
}

/// <summary>What a scan found in one document.</summary>
internal sealed class DocumentScan(string path, SourceText text)
{
    public string Path { get; } = path;
    public SourceText Text { get; } = text;
    public List<Occurrence> Occurrences { get; } = [];

    /// <summary>
    /// Names of types the document reaches through a using directive that share a new name: a new using directive
    /// would make them ambiguous.
    /// </summary>
    public HashSet<string> TypeNames { get; } = new(StringComparer.Ordinal);

    /// <summary>Namespace declarations: span start, name span, full name, and the keys of the types declared directly in it.</summary>
    public List<(int Start, TextSpan NameSpan, string Name, List<string> Types)> NamespaceDeclarations { get; } = [];

    /// <summary>Compilation-unit using directives: span, the name for a plain using, null for static and alias usings.</summary>
    public List<(TextSpan Span, string? Name)> Usings { get; } = [];

    /// <summary>Where a new using directive goes when there are none: the first token's start.</summary>
    public int FirstTokenStart { get; set; }
}
