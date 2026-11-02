// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.CSharp.Syntax;

namespace Kerf.Tools.Rename;

/// <summary>
/// Drafts the systematic part of a map from the code, for review: each type under its package's namespace without the
/// package prefix, enum members without it, and the parameterless <c>NbXxx</c>, <c>IsXxx</c> and <c>HasXxx</c>
/// methods that are cheap getters as properties. Others are written commented out, with the reason.
/// </summary>
internal static class Drafter
{
    public static void Draft(Loaded loaded, Scope scope, NameMap seed, string output, string? engineNamespace, TextWriter log)
    {
        // Every package without a namespace of its own goes under the engine's: Kerf.Engine.BSplCLib
        if (engineNamespace is not null)
        {
            foreach (INamedTypeSymbol type in loaded.SourceTypes().Where(t => t.ContainingType is null && scope.Contains(t)))
            {
                int underscore = type.Name.IndexOf('_');
                string package = underscore > 0 ? type.Name[..underscore] : type.Name;
                if (!seed.Packages.ContainsKey(package) && !type.Name.StartsWith('_'))
                {
                    seed.Packages[package] = engineNamespace + "." + Naming.Pascal(package);
                }
            }
        }

        var map = new NameMap();
        foreach (string rule in NameMap.Rules.All)
        {
            map.RuleSwitches[rule] = seed.RuleSwitches.GetValueOrDefault(rule, true);
        }

        foreach ((string package, string @namespace) in seed.Packages)
        {
            map.Packages[package] = @namespace;
        }

        var rejected = new List<string>();
        foreach (INamedTypeSymbol type in loaded.SourceTypes().Where(scope.Contains).OrderBy(t => Keys.Of(t), StringComparer.Ordinal))
        {
            string key = Keys.Of(type)!;
            if (seed.Types.TryGetValue(key, out string? reviewed))
            {
                map.Types[key] = reviewed;
                map.Notes["type " + key] = seed.Notes.GetValueOrDefault("type " + key) ?? "";
            }
            else if (type.ContainingType is null && Package(type.Name, seed) is { } package)
            {
                string stem = type.Name == package ? Naming.Pascal(type.Name) : type.Name[(package.Length + 1)..];
                map.Types[key] = seed.Packages[package] + "." + Naming.Pascal(stem);
                map.Notes["type " + key] = "draft" + (type.Name == package ? ": the package's own class, named like its namespace" : "");
            }

            if (type.TypeKind == TypeKind.Enum)
            {
                foreach (IFieldSymbol member in type.GetMembers().OfType<IFieldSymbol>())
                {
                    if (Package(member.Name, seed) is { } prefix && member.Name.Length > prefix.Length + 1)
                    {
                        map.Members[Keys.Of(member)!] = Naming.Pascal(member.Name[(prefix.Length + 1)..]);
                    }
                }
            }

            foreach (IMethodSymbol method in type.GetMembers().OfType<IMethodSymbol>())
            {
                // An override follows its root's entry
                if (method.MethodKind != MethodKind.Ordinary || method.OverriddenMethod is not null || Naming.PropertyFor(method.Name) is not { } property)
                {
                    continue;
                }

                string methodKey = Keys.Of(method)!;
                string? reason = method.Parameters.Length > 0 ? null : Planner.ConversionBlocker(method) ?? NotAGetter(method);
                if (method.Parameters.Length > 0)
                {
                    continue;
                }

                if (reason is null)
                {
                    map.Properties[methodKey] = property;
                }
                else
                {
                    rejected.Add($"# property {methodKey} {property}  # stays a method: {reason}");
                }
            }
        }

        foreach ((string id, string name) in seed.Members)
        {
            map.Members[id] = name;
        }

        foreach ((string id, string name) in seed.Aliases)
        {
            map.Aliases[id] = name;
        }

        foreach (string id in seed.Keep)
        {
            map.Keep.Add(id);
        }

        map.Save(output,
        [
            "kerf-rename map, drafted by `kerf-rename draft` for " + string.Join(", ", scope.Assemblies)
                + (scope.Packages.Count > 0 ? " (packages " + string.Join(", ", scope.Packages) + ")" : ""),
            "Review every entry marked 'draft': the names are proposals for unit 10D's API review.",
            "Ids are Roslyn's documentation comment ids. Format: kerf-rename/README.md.",
        ]);
        if (rejected.Count > 0)
        {
            File.AppendAllLines(output, ["", "# Methods the drafter left as methods:", .. rejected.Order(StringComparer.Ordinal)]);
        }

        log.WriteLine($"Drafted {map.Types.Count} types, {map.Members.Count} members, {map.Properties.Count} properties ({rejected.Count} left as methods) into {output}");
    }

    private static string? Package(string name, NameMap seed)
        => seed.Packages.Keys.Where(p => name == p || name.StartsWith(p + "_", StringComparison.Ordinal)).MaxBy(p => p.Length);

    /// <summary>
    /// Why a method isn't a cheap getter, or null if it is. A getter is abstract (the name decides), or an expression, or a
    /// block of at most three statements whose last returns and whose others only guard; and it assigns nothing but locals.
    /// </summary>
    private static string? NotAGetter(IMethodSymbol method)
    {
        if (method.IsAbstract)
        {
            return null;
        }

        foreach (SyntaxReference reference in method.DeclaringSyntaxReferences)
        {
            if (reference.GetSyntax() is not MethodDeclarationSyntax declaration)
            {
                return "not declared as a method";
            }

            SyntaxNode? body = (SyntaxNode?)declaration.ExpressionBody ?? declaration.Body;
            if (body is null)
            {
                return "has no body";
            }

            if (body.DescendantNodes().Any(n => n is AssignmentExpressionSyntax { Left: not IdentifierNameSyntax } or PostfixUnaryExpressionSyntax
                    { RawKind: (int)SyntaxKind.PostIncrementExpression or (int)SyntaxKind.PostDecrementExpression }
                    or PrefixUnaryExpressionSyntax { RawKind: (int)SyntaxKind.PreIncrementExpression or (int)SyntaxKind.PreDecrementExpression }))
            {
                return "changes state";
            }

            if (declaration.Body is { } block)
            {
                if (block.Statements.Count > 3 || block.Statements.LastOrDefault() is not ReturnStatementSyntax
                    || block.Statements.SkipLast(1).Any(s => !IsGuard(s)))
                {
                    return "computes";
                }
            }
        }

        return null;
    }

    private static bool IsGuard(StatementSyntax statement) => statement switch
    {
        IfStatementSyntax { Else: null, Statement: ThrowStatementSyntax or BlockSyntax { Statements: [ThrowStatementSyntax] } } => true,
        ExpressionStatementSyntax { Expression: InvocationExpressionSyntax invocation }
            => invocation.Expression.ToString().EndsWith("Raise_if", StringComparison.Ordinal),
        _ => false,
    };
}
