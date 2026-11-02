// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp.Syntax;

namespace Kerf.Tools.Rename;

/// <summary>
/// The plan before scanning: the map's types, members and properties, and the field rules, checked for collisions in
/// each type's hierarchy. Parameters and locals are planned during the scan, body by body.
/// </summary>
internal static class Planner
{
    public static Plan Build(Loaded loaded, NameMap map, Scope scope, TextWriter log)
    {
        var plan = new Plan();
        List<INamedTypeSymbol> sourceTypes = [.. loaded.SourceTypes()];

        // Types
        foreach ((string id, string fullName) in map.Types)
        {
            if (loaded.Find(id) is not INamedTypeSymbol type || !scope.Contains(type))
            {
                continue;
            }

            int dot = fullName.LastIndexOf('.');
            string newName = fullName[(dot + 1)..];
            string newNamespace = dot < 0 ? "" : fullName[..dot];
            string oldNamespace = type.ContainingNamespace.ToDisplayString();
            plan.Add(id, type.Name, newName, Category.Type);
            plan.OldNames[type.Name] = 0;
            plan.TypeOldNames[type.Name] = 0;
            if (type.ContainingType is null && newNamespace != oldNamespace)
            {
                // A type that only moves keeps its key in the plan, as a property that keeps its name does
                plan.Names[id] = newName;
                plan.Categories[id] = Category.Type;
                plan.Namespaces[id] = newNamespace;
                plan.OldNamespaces[id] = oldNamespace;
            }
        }

        CheckTypes(loaded, plan, sourceTypes);
        foreach (string key in plan.Namespaces.Keys)
        {
            plan.MovedTypeNames[plan.NameOf(key)!] = 0;
        }

        foreach ((string alias, string newAlias) in map.Aliases)
        {
            plan.Add("A:" + alias, alias, newAlias, Category.Alias);
        }

        // Members and properties named in the map
        foreach ((string id, string newName) in map.Members)
        {
            if (loaded.Find(id) is { } member && scope.Contains(member))
            {
                plan.Add(id, member.Name, newName, Category.Member);
            }
        }

        foreach ((string id, string newName) in map.Properties)
        {
            if (loaded.Find(id) is not IMethodSymbol method || !scope.Contains(method))
            {
                continue;
            }

            if (ConversionBlocker(method) is { } blocker)
            {
                plan.Cancelled[id] = blocker;
                continue;
            }

            // A property keeps its key in the plan even when its name doesn't change (IsDone)
            plan.Names[id] = newName;
            plan.Categories[id] = Category.Property;
            plan.OldNames[method.Name] = 0;
            plan.Conversions[id] = 0;
        }

        // Fields by rule
        bool instances = map.Rule(NameMap.Rules.Fields);
        bool statics = map.Rule(NameMap.Rules.Statics);
        if (instances || statics)
        {
            foreach (INamedTypeSymbol type in sourceTypes.Where(scope.Contains))
            {
                foreach (IFieldSymbol field in type.GetMembers().OfType<IFieldSymbol>())
                {
                    string? key = Keys.Of(field);
                    if (key is null || plan.Names.ContainsKey(key) || map.Keep.Contains(key) || map.Members.ContainsKey(key))
                    {
                        continue;
                    }

                    if (Naming.Field(field, instances, statics) is { } newName)
                    {
                        plan.Add(key, field.Name, newName, field.IsStatic || field.IsConst ? Category.Static : Category.Field);
                    }
                }
            }
        }

        if (map.Rule(NameMap.Rules.Methods))
        {
            foreach (INamedTypeSymbol type in sourceTypes.Where(scope.Contains))
            {
                foreach (IMethodSymbol method in type.GetMembers().OfType<IMethodSymbol>())
                {
                    string? key = Keys.Of(method);
                    if (key is not null && method.OverriddenMethod is null && !plan.Names.ContainsKey(key) && !map.Keep.Contains(key)
                        && !map.Members.ContainsKey(key) && Naming.Method(method) is { } newName)
                    {
                        plan.Add(key, method.Name, newName, Category.Member);
                    }
                }
            }
        }

        CheckMembers(plan, sourceTypes, scope);
        log.WriteLine($"Planned {plan.Names.Count} renames of types, members and fields; {plan.Cancelled.Count} cancelled");
        return plan;
    }

    /// <summary>Why a method can't become a get-only property, or null if it can.</summary>
    public static string? ConversionBlocker(IMethodSymbol method)
    {
        if (method.Parameters.Length > 0 || method.IsGenericMethod || method.ReturnsVoid || method.ReturnsByRef || method.ReturnsByRefReadonly)
        {
            return "not a parameterless method returning a value";
        }

        if (method.MethodKind != MethodKind.Ordinary || method.IsExtern || method.IsPartialDefinition || method.PartialImplementationPart is not null)
        {
            return "not an ordinary method";
        }

        if (method.ContainingType.TypeKind == TypeKind.Interface || method.ExplicitInterfaceImplementations.Length > 0)
        {
            return "an interface member";
        }

        foreach (INamedTypeSymbol @interface in method.ContainingType.AllInterfaces)
        {
            foreach (ISymbol member in @interface.GetMembers())
            {
                if (SymbolEqualityComparer.Default.Equals(method.ContainingType.FindImplementationForInterfaceMember(member), method))
                {
                    return "implements " + member.ToDisplayString();
                }
            }
        }

        if (method.OverriddenMethod is not null)
        {
            return "overrides a method the map doesn't convert";
        }

        if (method.GetAttributes().Length > 0)
        {
            return "has attributes";
        }

        return null;
    }

    /// <summary>Types: no two to one name; no new name taken by a type that stays; a type's members can't take its name.</summary>
    private static void CheckTypes(Loaded loaded, Plan plan, List<INamedTypeSymbol> sourceTypes)
    {
        var finalNames = new Dictionary<string, List<string>>(StringComparer.Ordinal);
        foreach (INamedTypeSymbol type in sourceTypes)
        {
            string key = Keys.Of(type)!;
            string name = plan.NameOf(key) ?? type.Name;
            string container = type.ContainingType is { } outer
                ? Keys.Of(outer)!
                : plan.Namespaces.TryGetValue(key, out string? moved) ? moved : type.ContainingNamespace.ToDisplayString();
            string full = container + "." + name + "`" + type.Arity;
            if (!finalNames.TryGetValue(full, out List<string>? keys))
            {
                finalNames[full] = keys = [];
            }

            keys.Add(key);
        }

        foreach ((string full, List<string> keys) in finalNames)
        {
            if (keys.Count > 1)
            {
                foreach (string key in keys)
                {
                    plan.Cancel(key, $"another type would be called {full}");
                }
            }
        }

        // A namespace declaration can move only if every type in it moves to the same place
        bool changed = true;
        while (changed)
        {
            changed = false;
            foreach (string key in plan.Namespaces.Keys.ToList())
            {
                if (loaded.Find(key) is not INamedTypeSymbol type)
                {
                    continue;
                }

                foreach (SyntaxReference reference in type.DeclaringSyntaxReferences)
                {
                    SyntaxNode node = reference.GetSyntax();
                    if (node.Parent is not BaseNamespaceDeclarationSyntax declaration || declaration.Parent is not CompilationUnitSyntax)
                    {
                        plan.Namespaces.TryRemove(key, out _);
                        plan.Notices.Add($"{key}: not moved to its new namespace: declared outside a single top-level namespace declaration");
                        changed = true;
                        break;
                    }

                    string target = plan.Namespaces[key];
                    foreach (MemberDeclarationSyntax sibling in declaration.Members)
                    {
                        string? siblingKey = sibling is BaseTypeDeclarationSyntax or DelegateDeclarationSyntax
                            ? Keys.Of(GetDeclared(loaded, node.SyntaxTree, sibling))
                            : null;
                        string? siblingTarget = siblingKey is null ? null : plan.Namespaces.GetValueOrDefault(siblingKey);
                        if (siblingTarget != target)
                        {
                            plan.Namespaces.TryRemove(key, out _);
                            plan.Notices.Add($"{key}: not moved to {target}: its file declares {siblingKey ?? sibling.Kind().ToString()}, which goes elsewhere");
                            changed = true;
                            break;
                        }
                    }

                    if (!plan.Namespaces.ContainsKey(key))
                    {
                        break;
                    }
                }
            }
        }

        foreach (INamedTypeSymbol type in sourceTypes)
        {
            string key = Keys.Of(type)!;
            if (plan.NameOf(key) is { } newName && type.GetMembers().Any(m => m.Name == newName || (m.CanBeReferencedByName && plan.NameOf(Keys.Of(m) ?? "") == newName)))
            {
                plan.Cancel(key, $"a member of the type is called {newName}");
            }
        }
    }

    private static ISymbol GetDeclared(Loaded loaded, SyntaxTree tree, SyntaxNode node)
    {
        foreach (Compilation compilation in loaded.Compilations.Values)
        {
            if (compilation.ContainsSyntaxTree(tree))
            {
                return compilation.GetSemanticModel(tree).GetDeclaredSymbol(node)!;
            }
        }

        throw new InvalidOperationException("No compilation holds " + tree.FilePath);
    }

    /// <summary>
    /// Members: after renaming, no two members visible in one type may share a name unless they all are methods (overloads)
    /// or they already shared it. Checked in every type of the scope and every type derived from one.
    /// </summary>
    private static void CheckMembers(Plan plan, List<INamedTypeSymbol> sourceTypes, Scope scope)
    {
        bool InHierarchyOfScope(INamedTypeSymbol type)
        {
            for (INamedTypeSymbol? current = type; current is not null; current = current.BaseType)
            {
                if (scope.Contains(current))
                {
                    return true;
                }
            }

            return false;
        }

        bool changed = true;
        while (changed)
        {
            changed = false;
            foreach (INamedTypeSymbol type in sourceTypes.Where(InHierarchyOfScope))
            {
                var groups = new Dictionary<string, Dictionary<string, ISymbol>>(StringComparer.Ordinal);
                for (INamedTypeSymbol? current = type; current is not null; current = current.BaseType)
                {
                    foreach (ISymbol member in current.GetMembers())
                    {
                        if (!member.CanBeReferencedByName || member is IMethodSymbol { MethodKind: not MethodKind.Ordinary }
                            || (member.DeclaredAccessibility == Accessibility.Private && !SymbolEqualityComparer.Default.Equals(current, type)))
                        {
                            continue;
                        }

                        string? key = Keys.Of(member);
                        if (key is null)
                        {
                            continue;
                        }

                        string name = plan.NameOf(key) ?? member.Name;
                        if (!groups.TryGetValue(name, out Dictionary<string, ISymbol>? group))
                        {
                            groups[name] = group = new(StringComparer.Ordinal);
                        }

                        group.TryAdd(key, member);
                    }
                }

                // A method that becomes a property can't implement an interface's method any more
                foreach (INamedTypeSymbol @interface in type.AllInterfaces)
                {
                    foreach (ISymbol member in @interface.GetMembers())
                    {
                        if (type.FindImplementationForInterfaceMember(member) is IMethodSymbol implementation
                            && Keys.Of(implementation) is { } implementationKey && plan.Converts(implementationKey))
                        {
                            plan.Cancel(implementationKey, $"implements {member.ToDisplayString()} in {type.ToDisplayString()}");
                            changed = true;
                        }
                    }
                }

                foreach ((string name, Dictionary<string, ISymbol> group) in groups)
                {
                    // Overloads, and names that already coexisted, stay legal unless a method becomes a property
                    bool converting = group.Keys.Any(plan.Converts);
                    if (group.Count < 2 || (!converting && (group.Values.All(m => m is IMethodSymbol)
                                                            || group.Values.Select(m => m.Name).Distinct().Count() < 2)))
                    {
                        continue;
                    }

                    foreach ((string key, ISymbol member) in group)
                    {
                        if (plan.Names.ContainsKey(key) && (plan.NameOf(key) != member.Name || plan.Converts(key)))
                        {
                            plan.Cancel(key, $"{type.Name} would have two members called {name}: "
                                             + string.Join(", ", group.Values.Select(m => m.ToDisplayString())));
                            changed = true;
                        }
                    }
                }
            }
        }
    }
}
