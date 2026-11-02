// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using Microsoft.CodeAnalysis.Text;

namespace Kerf.Tools.Rename;

/// <summary>
/// One pass over a document: plans the parameters and locals of the scope's bodies, then finds every token that names
/// something planned.
/// </summary>
internal sealed class Scanner(Plan plan, NameMap map, Scope scope)
{
    private readonly bool _parameters = map.Rule(NameMap.Rules.Parameters);
    private readonly bool _locals = map.Rule(NameMap.Rules.Locals);

    public async Task<DocumentScan> ScanAsync(Document document, CancellationToken ct)
    {
        SyntaxNode root = (await document.GetSyntaxRootAsync(ct))!;
        SemanticModel model = (await document.GetSemanticModelAsync(ct))!;
        var scan = new DocumentScan(document.FilePath!, await document.GetTextAsync(ct));

        var unit = (CompilationUnitSyntax)root;
        scan.FirstTokenStart = unit.GetFirstToken(includeZeroWidth: true).SpanStart;
        foreach (UsingDirectiveSyntax @using in unit.Usings)
        {
            scan.Usings.Add((@using.Span, @using.Alias is null && @using.StaticKeyword == default && @using.GlobalKeyword == default
                ? @using.NamespaceOrType.ToString()
                : null));
        }

        foreach (BaseNamespaceDeclarationSyntax declaration in root.DescendantNodes(n => n is CompilationUnitSyntax or BaseNamespaceDeclarationSyntax)
                     .OfType<BaseNamespaceDeclarationSyntax>())
        {
            List<string> types = [];
            foreach (MemberDeclarationSyntax member in declaration.Members)
            {
                if (member is BaseTypeDeclarationSyntax or DelegateDeclarationSyntax && model.GetDeclaredSymbol(member, ct) is { } type
                    && Keys.Of(type) is { } key)
                {
                    types.Add(key);
                }
            }

            scan.NamespaceDeclarations.Add((declaration.SpanStart, declaration.Name.Span, FullName(declaration), types));
        }

        // Bodies first: they plan the parameters and locals that the tokens below then find
        bool inScope = false;
        foreach (BaseTypeDeclarationSyntax typeDeclaration in root.DescendantNodes().OfType<BaseTypeDeclarationSyntax>())
        {
            if (model.GetDeclaredSymbol(typeDeclaration, ct) is not { } type || !scope.Contains(type))
            {
                continue;
            }

            inScope = true;
            if (typeDeclaration is TypeDeclarationSyntax withMembers)
            {
                foreach (MemberDeclarationSyntax member in withMembers.Members)
                {
                    if (member is not BaseTypeDeclarationSyntax)
                    {
                        PlanBody(model, member, ct);
                    }
                }
            }
        }

        foreach (DelegateDeclarationSyntax @delegate in root.DescendantNodes().OfType<DelegateDeclarationSyntax>())
        {
            if (model.GetDeclaredSymbol(@delegate, ct) is { } type && scope.Contains(type))
            {
                PlanBody(model, @delegate, ct);
            }
        }

        foreach (SyntaxToken token in root.DescendantTokens(descendIntoTrivia: true))
        {
            if (!token.IsKind(SyntaxKind.IdentifierToken))
            {
                if (token.IsKind(SyntaxKind.StringLiteralToken) || token.IsKind(SyntaxKind.InterpolatedStringTextToken))
                {
                    NoticeString(scan, token);
                }

                continue;
            }

            string text = token.ValueText;
            if (!token.Parent!.IsPartOfStructuredTrivia())
            {
                scan.Identifiers.Add(text);
            }

            if (text == "UnsafeAccessor" && token.Parent is IdentifierNameSyntax { Parent: AttributeSyntax attribute })
            {
                ScanAccessor(scan, model, attribute, ct);
                continue;
            }

            if (plan.OldNames.ContainsKey(text) || (inScope && _locals && Naming.Local(text) is not null))
            {
                ScanToken(scan, model, token, ct);
            }
        }

        return scan;
    }

    private static string FullName(BaseNamespaceDeclarationSyntax declaration)
    {
        string name = declaration.Name.ToString();
        return declaration.Parent is BaseNamespaceDeclarationSyntax outer ? FullName(outer) + "." + name : name;
    }

    /// <summary>
    /// Plans the renames of a member's parameters and locals, dropping any that would collide: two names in the body that
    /// were different becoming the same, or a reserved word.
    /// </summary>
    private void PlanBody(SemanticModel model, MemberDeclarationSyntax member, CancellationToken ct)
    {
        if (member is MethodDeclarationSyntax method && method.Modifiers.Any(SyntaxKind.PartialKeyword))
        {
            return;
        }

        if (member is TypeDeclarationSyntax { ParameterList: not null })
        {
            return;
        }

        // Everything named in the body: the candidates and everything they could collide with
        var candidates = new Dictionary<string, (ISymbol Symbol, string NewName, Category Category)>(StringComparer.Ordinal);
        var symbols = new Dictionary<string, ISymbol>(StringComparer.Ordinal);
        bool inAccessor = false;
        foreach (SyntaxNode node in member.DescendantNodesAndSelf(n => n is not BaseTypeDeclarationSyntax))
        {
            if (node is AccessorDeclarationSyntax)
            {
                inAccessor = true;
            }

            ISymbol? declared = node switch
            {
                ParameterSyntax or VariableDeclaratorSyntax { Parent.Parent: not BaseFieldDeclarationSyntax } or SingleVariableDesignationSyntax
                    or ForEachStatementSyntax or CatchDeclarationSyntax or LocalFunctionStatementSyntax or FromClauseSyntax
                    or LetClauseSyntax or JoinClauseSyntax or QueryContinuationSyntax or TypeParameterSyntax
                    => model.GetDeclaredSymbol(node, ct),
                _ => null,
            };

            if (declared is not null && Keys.Of(declared) is { } declaredKey)
            {
                symbols[declaredKey] = declared;
                if (map.Keep.Contains(declaredKey))
                {
                    continue;
                }

                if (declared is IParameterSymbol parameter && _parameters)
                {
                    string? newName = map.Params.GetValueOrDefault(declaredKey) ?? Naming.Parameter(parameter.Name);
                    if (newName is not null && !IsPositional(parameter))
                    {
                        candidates[declaredKey] = (declared, newName, Category.Parameter);
                    }
                }
                else if (declared is ILocalSymbol or IRangeVariableSymbol && _locals && Naming.Local(declared.Name) is { } newName)
                {
                    candidates[declaredKey] = (declared, newName, Category.Local);
                }
            }
            else if (node is SimpleNameSyntax name && IsScopeBound(name))
            {
                SymbolInfo info = model.GetSymbolInfo(name, ct);
                if ((info.Symbol ?? info.CandidateSymbols.FirstOrDefault()) is { } referenced && Keys.Of(referenced) is { } referencedKey)
                {
                    symbols.TryAdd(referencedKey, referenced);
                }
                else if (info.Symbol is INamespaceSymbol @namespace)
                {
                    symbols.TryAdd("N:" + @namespace.ToDisplayString(), @namespace);
                }
            }
        }

        if (candidates.Count == 0)
        {
            return;
        }

        // Parameters of the member itself, which its body may not mention
        var accepted = new Dictionary<string, (ISymbol Symbol, string NewName, Category Category)>(candidates, StringComparer.Ordinal);
        foreach ((string key, (ISymbol symbol, string newName, _)) in candidates)
        {
            if (Naming.IsReserved(newName) && !(newName is "value" or "field" && !inAccessor) || newName == "args")
            {
                accepted.Remove(key);
                Cancel(key, symbol, $"'{newName}' is reserved");
            }
        }

        // Drop renames until no final name stands for two symbols that had different names; locals give way first
        bool changed = true;
        while (changed)
        {
            changed = false;
            var groups = symbols.GroupBy(pair => accepted.TryGetValue(pair.Key, out var candidate) ? candidate.NewName : FinalName(pair.Key, pair.Value));
            foreach (var group in groups)
            {
                if (group.Select(pair => pair.Value.Name).Distinct().Count() < 2)
                {
                    continue;
                }

                string[] renamed = [.. group.Where(pair => accepted.ContainsKey(pair.Key))
                    .OrderBy(pair => accepted[pair.Key].Category == Category.Local ? 0 : 1).Select(pair => pair.Key)];
                if (renamed.Length == 0)
                {
                    continue;
                }

                string loser = renamed[0];
                Cancel(loser, symbols[loser], $"'{group.Key}' would also name {string.Join(", ", group.Where(p => p.Key != loser).Select(p => Describe(p.Value)))}");
                accepted.Remove(loser);
                changed = true;
                break;
            }
        }

        foreach ((string key, (ISymbol symbol, string newName, Category category)) in accepted)
        {
            plan.Add(key, symbol.Name, newName, category);
        }
    }

    private string FinalName(string key, ISymbol symbol) => plan.NameOf(key) ?? symbol.Name;

    private void Cancel(string key, ISymbol symbol, string reason)
    {
        plan.Cancelled[key] = reason;
        plan.Names.TryRemove(key, out _);
    }

    private static string Describe(ISymbol symbol) => symbol.Kind.ToString().ToLowerInvariant() + " " + symbol.Name;

    /// <summary>A record's or a primary constructor's parameter is also a property: the map names it.</summary>
    private static bool IsPositional(IParameterSymbol parameter)
        => parameter.ContainingSymbol is IMethodSymbol { MethodKind: MethodKind.Constructor } constructor
           && constructor.DeclaringSyntaxReferences.Any(r => r.GetSyntax() is TypeDeclarationSyntax);

    /// <summary>Whether a simple name is looked up in the enclosing scopes, as opposed to a member of something.</summary>
    private static bool IsScopeBound(SimpleNameSyntax name) => name.Parent switch
    {
        MemberAccessExpressionSyntax access => access.Expression == name,
        QualifiedNameSyntax qualified => qualified.Left == name,
        MemberBindingExpressionSyntax or NameColonSyntax or NameEqualsSyntax or AliasQualifiedNameSyntax or XmlNameAttributeSyntax => false,
        AssignmentExpressionSyntax { Parent: InitializerExpressionSyntax } assignment => assignment.Right == name,
        _ => !name.IsPartOfStructuredTrivia(),
    };

    private void ScanToken(DocumentScan scan, SemanticModel model, SyntaxToken token, CancellationToken ct)
    {
        SyntaxNode parent = token.Parent!;
        if (parent is SimpleNameSyntax name)
        {
            ScanReference(scan, model, name, ct);
            return;
        }

        ISymbol? declared = parent switch
        {
            BaseTypeDeclarationSyntax or DelegateDeclarationSyntax or MethodDeclarationSyntax or PropertyDeclarationSyntax
                or EventDeclarationSyntax or VariableDeclaratorSyntax or EnumMemberDeclarationSyntax or ParameterSyntax
                or SingleVariableDesignationSyntax or ForEachStatementSyntax or CatchDeclarationSyntax or LocalFunctionStatementSyntax
                or FromClauseSyntax or LetClauseSyntax or JoinClauseSyntax or QueryContinuationSyntax
                => model.GetDeclaredSymbol(parent, ct),
            ConstructorDeclarationSyntax or DestructorDeclarationSyntax => model.GetDeclaredSymbol(parent, ct)?.ContainingType,
            _ => null,
        };

        if (declared is null || Keys.Of(declared) is not { } key || !plan.Names.ContainsKey(key))
        {
            return;
        }

        if (parent is MethodDeclarationSyntax method && plan.Converts(key))
        {
            string? blocker = method.AttributeLists.Count > 0 ? "a declaration has attributes"
                : method.ExplicitInterfaceSpecifier is not null ? "a declaration implements an interface explicitly"
                : method.Body is { } block && block.DescendantTokens().Any(t => t.Span.Length > 0 && t.ToString().Contains('\n')) ? "a declaration's body has a multi-line string"
                : null;
            Occurrence occurrence = method switch
            {
                { ExpressionBody: not null } => new(token.Span, key, OccurrenceKind.DeclaredExpression) { Extra = method.ParameterList.Span },
                { Body: { } body } => new(token.Span, key, OccurrenceKind.DeclaredBlock) { Extra = method.ParameterList.Span, Body = body.Span },
                _ => new(token.Span, key, OccurrenceKind.DeclaredAbstract) { Extra = TextSpan.FromBounds(method.ParameterList.SpanStart, method.SemicolonToken.Span.End) },
            };
            scan.Occurrences.Add(occurrence with { Blocker = blocker });
            return;
        }

        scan.Occurrences.Add(new(token.Span, key, OccurrenceKind.Name));
    }

    private void ScanReference(DocumentScan scan, SemanticModel model, SimpleNameSyntax name, CancellationToken ct)
    {
        // <param name="theX"/>, <paramref name="theX"/>: the parameter of the documented member
        if (name.Parent is XmlNameAttributeSyntax)
        {
            if (name.FirstAncestorOrSelf<DocumentationCommentTriviaSyntax>()?.ParentTrivia.Token.Parent?.FirstAncestorOrSelf<MemberDeclarationSyntax>() is { } documented)
            {
                foreach (ParameterSyntax parameterSyntax in documented.DescendantNodes(n => n is not BlockSyntax and not ArrowExpressionClauseSyntax).OfType<ParameterSyntax>())
                {
                    if (parameterSyntax.Identifier.ValueText == name.Identifier.ValueText
                        && model.GetDeclaredSymbol(parameterSyntax, ct) is { } parameter && Keys.Of(parameter) is { } parameterKey
                        && plan.Names.ContainsKey(parameterKey))
                    {
                        scan.Occurrences.Add(new(name.Identifier.Span, parameterKey, OccurrenceKind.Name));
                        return;
                    }
                }
            }

            return;
        }

        // A using alias (math_Vector)
        if (model.GetAliasInfo(name, ct) is { } alias)
        {
            if (map.Aliases.ContainsKey(alias.Name))
            {
                scan.Occurrences.Add(new(name.Identifier.Span, "A:" + alias.Name, OccurrenceKind.Name));
            }

            return;
        }

        SyntaxNode bound = name.Parent is NameMemberCrefSyntax member
            ? member.Parent is QualifiedCrefSyntax qualifiedCref && qualifiedCref.Member == member ? qualifiedCref : member
            : name;
        SymbolInfo info = model.GetSymbolInfo(bound, ct);
        ISymbol? symbol = info.Symbol ?? info.CandidateSymbols.FirstOrDefault();
        if (symbol is null || Keys.Of(symbol) is not { } key)
        {
            return;
        }

        if (!plan.Names.ContainsKey(key))
        {
            // A parameter of a method compiled elsewhere (another repository applying the map): the rule, unless kept
            if (symbol is IParameterSymbol parameter && _parameters && !parameter.Locations.Any(l => l.IsInSource)
                && scope.Contains(parameter.ContainingSymbol) && !map.Keep.Contains(key)
                && (map.Params.GetValueOrDefault(key) ?? Naming.Parameter(parameter.Name)) is { } ruleName)
            {
                plan.Add(key, parameter.Name, ruleName, Category.Parameter);
            }
            else
            {
                return;
            }
        }

        if (Keys.Normalize(symbol) is INamedTypeSymbol)
        {
            ScanTypeReference(scan, model, name, key, ct);
            return;
        }

        if (name.Parent is NameMemberCrefSyntax cref)
        {
            scan.Occurrences.Add(new(name.Identifier.Span, key, OccurrenceKind.Cref) { Extra = cref.Parameters?.Span ?? default });
            return;
        }

        if (symbol is IMethodSymbol && plan.Converts(key))
        {
            ExpressionSyntax expression = name.Parent is MemberAccessExpressionSyntax access && access.Name == name ? access
                : name.Parent is MemberBindingExpressionSyntax binding ? binding
                : name;
            if (expression.Parent is InvocationExpressionSyntax invocation && invocation.Expression == expression)
            {
                scan.Occurrences.Add(new(name.Identifier.Span, key, OccurrenceKind.Invoked) { Extra = invocation.ArgumentList.Span });
            }
            else if (IsInNameof(name))
            {
                scan.Occurrences.Add(new(name.Identifier.Span, key, OccurrenceKind.Name));
            }
            else
            {
                scan.Occurrences.Add(new(name.Identifier.Span, key, OccurrenceKind.MethodGroup)
                {
                    Blocker = $"used as a method group at {scan.Path}({scan.Text.Lines.GetLinePosition(name.SpanStart).Line + 1})",
                });
            }

            return;
        }

        // A member named on its own (an enum member through using static, a field in its type): would its new name
        // mean something else here?
        bool simple = name.Parent is not (MemberAccessExpressionSyntax { Name: var right } and var _) || right != name;
        if (simple && name.Parent is not MemberBindingExpressionSyntax && !name.IsPartOfStructuredTrivia()
            && symbol is IFieldSymbol or IPropertySymbol or IMethodSymbol or IEventSymbol && plan.NameOf(key) is { } newMemberName
            && model.LookupSymbols(name.SpanStart, name: newMemberName).Any(s => Keys.Of(s) != key))
        {
            scan.Occurrences.Add(new(name.Identifier.Span, key, OccurrenceKind.Name)
            {
                Ambiguous = true,
                Qualified = symbol.IsStatic || symbol is IFieldSymbol { IsConst: true }
                    ? "global::" + QualifiedTypeName(symbol.ContainingType) + "." + newMemberName
                    : "this." + newMemberName,
            });
            return;
        }

        scan.Occurrences.Add(new(name.Identifier.Span, key, OccurrenceKind.Name));
    }

    /// <summary>A type's name after the plan, in full.</summary>
    private string QualifiedTypeName(INamedTypeSymbol type)
    {
        string key = Keys.Of(type)!;
        string name = plan.NameOf(key) ?? type.Name;
        return type.ContainingType is { } outer
            ? QualifiedTypeName(outer) + "." + name
            : (plan.Namespaces.GetValueOrDefault(key) ?? type.ContainingNamespace.ToDisplayString()) + "." + name;
    }

    private void ScanTypeReference(DocumentScan scan, SemanticModel model, SimpleNameSyntax name, string key, CancellationToken ct)
    {
        if (!plan.Namespaces.TryGetValue(key, out string? newNamespace))
        {
            scan.Occurrences.Add(new(name.Identifier.Span, key, OccurrenceKind.Name));
            return;
        }

        // Qualified by a namespace: Occt.gp_Pnt, global::Occt.gp_Pnt, O.gp_Pnt through an alias
        SyntaxNode? left = name.Parent switch
        {
            QualifiedNameSyntax qualified when qualified.Right == name => qualified.Left,
            MemberAccessExpressionSyntax access when access.Name == name => access.Expression,
            _ => null,
        };
        if (left is not null)
        {
            if (model.GetSymbolInfo(left, ct).Symbol is INamespaceSymbol)
            {
                bool global = left.ToString().StartsWith("global::", StringComparison.Ordinal)
                              || (left is IdentifierNameSyntax leftName && model.GetAliasInfo(leftName, ct) is not null);
                scan.Occurrences.Add(new(name.Identifier.Span, key, OccurrenceKind.TypeQualified) { Extra = left.Span, GlobalQualifier = global });
            }
            else
            {
                scan.Occurrences.Add(new(name.Identifier.Span, key, OccurrenceKind.Name));
            }

            return;
        }

        if (name.Parent is AliasQualifiedNameSyntax)
        {
            plan.Notices.Add($"{scan.Path}: {name.Parent} names a type that moves; not changed");
            return;
        }

        // On its own: would the new name mean something else here?
        string newName = plan.NameOf(key)!;
        bool ambiguous = !name.IsPartOfStructuredTrivia() && model.LookupSymbols(name.SpanStart, name: newName).Length > 0;
        BaseNamespaceDeclarationSyntax? enclosing = name.FirstAncestorOrSelf<BaseNamespaceDeclarationSyntax>();
        scan.Occurrences.Add(new(name.Identifier.Span, key, OccurrenceKind.TypeUnqualified)
        {
            NamespaceDeclaration = enclosing?.SpanStart ?? -1,
            Namespace = enclosing is null ? "" : FullName(enclosing),
            Ambiguous = ambiguous,
        });
    }

    private static bool IsInNameof(SyntaxNode node)
        => node.Ancestors().OfType<InvocationExpressionSyntax>().Any(i => i.Expression is IdentifierNameSyntax { Identifier.ValueText: "nameof" });

    /// <summary>[UnsafeAccessor(Kind, Name = "member")] on an extern method: the member is looked up in its first parameter's type.</summary>
    private void ScanAccessor(DocumentScan scan, SemanticModel model, AttributeSyntax attribute, CancellationToken ct)
    {
        if (attribute.FirstAncestorOrSelf<MethodDeclarationSyntax>() is not { } method || model.GetDeclaredSymbol(method, ct) is not { } accessor)
        {
            return;
        }

        foreach (AttributeData data in accessor.GetReturnTypeAttributes().Concat(accessor.Parameters.SelectMany(p => p.GetAttributes())))
        {
            if (data.AttributeClass?.Name == "UnsafeAccessorTypeAttribute")
            {
                plan.Notices.Add($"{scan.Path}: {accessor.Name} names a type in a string ({data.ConstructorArguments.FirstOrDefault().Value}): not checked");
            }
        }

        AttributeArgumentSyntax? nameArgument = attribute.ArgumentList?.Arguments.FirstOrDefault(a => a.NameEquals?.Name.Identifier.ValueText == "Name");
        if (nameArgument?.Expression is not LiteralExpressionSyntax literal || accessor.Parameters.Length == 0)
        {
            return;
        }

        string memberName = literal.Token.ValueText;
        for (ITypeSymbol? type = accessor.Parameters[0].Type; type is not null; type = type.BaseType)
        {
            foreach (ISymbol member in type.GetMembers(memberName))
            {
                if (Keys.Of(member) is { } key && plan.Names.ContainsKey(key))
                {
                    scan.Occurrences.Add(new(literal.Token.Span, key, OccurrenceKind.AccessorName));
                    return;
                }
            }
        }
    }

    /// <summary>A string that names a renamed type: reported, never changed (messages and reflection need judgement).</summary>
    private void NoticeString(DocumentScan scan, SyntaxToken token)
    {
        string value = token.ValueText;
        if (value.Length < 3 || !value.Contains('_'))
        {
            return;
        }

        foreach (string word in value.Split((char[])[' ', '.', '(', ')', ',', ':', '"', '\'', '`', '<', '>', '[', ']', '\n', '\t'], StringSplitOptions.RemoveEmptyEntries))
        {
            if (plan.OldNames.ContainsKey(word) && plan.TypeOldNames.ContainsKey(word))
            {
                plan.Notices.Add($"{scan.Path}({scan.Text.Lines.GetLinePosition(token.SpanStart).Line + 1}): string names {word}");
                return;
            }
        }
    }
}
