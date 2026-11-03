// collections-rewrite (units 10J, 10J2): replaces an NCollection wrapper by the .NET collection it wraps, from the compiler's symbols.
// Usage: collections-rewrite <mode> <project or solution>... [-p:Name=Value]... [--dry]
// Modes: datamap (NCollection_DataMap), dynarray (NCollection_DynamicArray/Vector), indexed (the indexed maps)
using System.Collections.Concurrent;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using Microsoft.CodeAnalysis.MSBuild;
using static Microsoft.CodeAnalysis.CSharp.SyntaxFactory;

string mode = args[0];
bool dry = args.Contains("--dry");
var paths = args.Skip(1).Where(a => !a.StartsWith("-p:") && !a.StartsWith("--")).ToList();
var props = args.Where(a => a.StartsWith("-p:")).Select(a => a[3..].Split('=', 2)).ToDictionary(a => a[0], a => a[1]);
props["Configuration"] = "Release";

string[] wrapperNames = mode switch
{
    "datamap" => ["Occt.NCollection_DataMap`2"],
    "dynarray" => ["Occt.NCollection_DynamicArray`1", "Occt.NCollection_Vector`1"],
    "indexed" => ["Occt.NCollection_IndexedMap`1", "Occt.NCollection_IndexedDataMap`2"],
    _ => throw new ArgumentException(mode),
};

var ws = MSBuildWorkspace.Create(props);
ws.RegisterWorkspaceFailedHandler(e => { if (e.Diagnostic.Kind == WorkspaceDiagnosticKind.Failure) Console.WriteLine("ws: " + e.Diagnostic.Message); });
foreach (var p in paths)
{
    if (p.EndsWith(".slnx") || p.EndsWith(".sln")) await ws.OpenSolutionAsync(p);
    else if (!ws.CurrentSolution.Projects.Any(x => string.Equals(x.FilePath, Path.GetFullPath(p), StringComparison.OrdinalIgnoreCase))) await ws.OpenProjectAsync(p);
}
var solution = ws.CurrentSolution;
foreach (var project in solution.Projects)
    if (project.ParseOptions is CSharpParseOptions o && o.DocumentationMode < DocumentationMode.Parse)
        solution = solution.WithProjectParseOptions(project.Id, o.WithDocumentationMode(DocumentationMode.Parse));
Console.WriteLine($"{solution.Projects.Count()} projects");

var seen = new ConcurrentDictionary<string, bool>(StringComparer.OrdinalIgnoreCase);
var report = new ConcurrentBag<string>();
var census = new ConcurrentDictionary<string, int>();
int changedFiles = 0, edits = 0;
foreach (var project in solution.Projects)
{
    var compilation = await project.GetCompilationAsync();
    var wrappers = wrapperNames.Select(n => compilation!.GetTypeByMetadataName(n)).ToArray();
    if (wrappers[0] is not { } wrapper) { continue; }
    foreach (var doc in project.Documents)
    {
        if (doc.FilePath is not { } path || path.Contains(Path.DirectorySeparatorChar + "obj" + Path.DirectorySeparatorChar) || !seen.TryAdd(path, true)) continue;
        if (wrapper.Locations.Any(l => string.Equals(l.SourceTree?.FilePath, path, StringComparison.OrdinalIgnoreCase))) continue;
        var tree = (await doc.GetSyntaxTreeAsync())!;
        var root = await tree.GetRootAsync();
        var model = compilation.GetSemanticModel(tree);
        WrapperRewriter rw = mode switch
        {
            "datamap" => new DataMapRewriter(model, wrappers!, path, report, census),
            "indexed" => new IndexedRewriter(model, wrappers!, path, report, census),
            _ => new DynArrayRewriter(model, wrappers!, path, report, census),
        };
        var newRoot = rw.Visit(root);
        if (rw.Edits > 0)
        {
            foreach (var ns in rw.NeedsUsings)
                if (newRoot is CompilationUnitSyntax cu && !cu.Usings.Any(u => u.Name?.ToString() == ns) && !IsInNamespace(cu, ns))
                    newRoot = AddUsing(cu, ns);
            if (!dry)
                File.WriteAllText(path, newRoot.ToFullString(), new System.Text.UTF8Encoding(encoderShouldEmitUTF8Identifier: File.ReadAllBytes(path) is [0xEF, 0xBB, 0xBF, ..]));
            changedFiles++; edits += rw.Edits;
        }
    }
}
Console.WriteLine($"{changedFiles} files, {edits} edits");
foreach (var kv in census.OrderBy(k => k.Key)) Console.WriteLine($"  {kv.Key}: {kv.Value}");
foreach (var r in report.Distinct().OrderBy(x => x)) Console.WriteLine(r);

static bool IsInNamespace(CompilationUnitSyntax cu, string ns)
    => cu.Members.OfType<BaseNamespaceDeclarationSyntax>().Any(n => n.Name.ToString() == ns);

static CompilationUnitSyntax AddUsing(CompilationUnitSyntax cu, string ns)
{
    var directive = UsingDirective(ParseName(ns)).WithUsingKeyword(Token(SyntaxKind.UsingKeyword).WithTrailingTrivia(Space))
        .WithTrailingTrivia(cu.Usings.Count > 0 ? cu.Usings[0].GetTrailingTrivia() : TriviaList(CarriageReturnLineFeed));
    if (cu.Usings.Count == 0)
    {
        // Before the first member, after the header comments
        var first = cu.Members.FirstOrDefault();
        if (first is null) return cu;
        var lead = first.GetLeadingTrivia();
        var newFirst = first.WithLeadingTrivia(TriviaList(CarriageReturnLineFeed));
        return cu.ReplaceNode(first, newFirst).WithUsings(SingletonList(directive.WithLeadingTrivia(lead)));
    }
    // Sorted among the plain usings: Kerf.* after Kerf.Engine.* ... insert before the first that sorts after it
    var usings = cu.Usings;
    int at = usings.Count;
    for (int i = 0; i < usings.Count; i++)
    {
        var u = usings[i];
        if (u.Alias is null && u.StaticKeyword.IsKind(SyntaxKind.None) && u.GlobalKeyword.IsKind(SyntaxKind.None)
            && string.CompareOrdinal(u.Name!.ToString(), ns) > 0 && !u.Name.ToString().StartsWith("System"))
        {
            at = i;
            break;
        }
        if (u.Alias is not null || !u.StaticKeyword.IsKind(SyntaxKind.None)) { at = i; break; }
    }
    if (at < usings.Count)
    {
        // Take over the leading trivia of the one it precedes (header comments stay first)
        var next = usings[at];
        var d = directive.WithLeadingTrivia(next.GetLeadingTrivia()).WithTrailingTrivia(next.GetTrailingTrivia().Where(t => t.IsKind(SyntaxKind.EndOfLineTrivia)).Take(1));
        if (!d.GetTrailingTrivia().Any()) d = d.WithTrailingTrivia(CarriageReturnLineFeed);
        usings = usings.Replace(next, next.WithLeadingTrivia()).Insert(at, d);
    }
    else
    {
        var last = usings[^1];
        var d = directive.WithLeadingTrivia().WithTrailingTrivia(last.GetTrailingTrivia());
        usings = usings.Replace(last, last.WithTrailingTrivia(CarriageReturnLineFeed)).Add(d);
    }
    return cu.WithUsings(usings);
}

/// <summary>NCollection_DataMap to Dictionary.</summary>
abstract class WrapperRewriter(SemanticModel model, INamedTypeSymbol?[] wrappers, string path, ConcurrentBag<string> report,
                             ConcurrentDictionary<string, int> census) : CSharpSyntaxRewriter(visitIntoStructuredTrivia: true)
{
    public int Edits;
    public readonly SortedSet<string> NeedsUsings = new(StringComparer.Ordinal);
    protected readonly SemanticModel model = model;
    protected readonly ConcurrentBag<string> report = report;
    protected abstract string NewName { get; }
    protected virtual string NewNameFor(INamedTypeSymbol t) => NewName;
    protected virtual string NewNamespace => "System.Collections.Generic";
    protected virtual void OnTypeRenamed() { }
    protected INamedTypeSymbol wrapper = wrappers[0]!;

    protected bool IsWrapper(ITypeSymbol? t) => t is INamedTypeSymbol n && wrappers.Any(w => SymbolEqualityComparer.Default.Equals(n.OriginalDefinition, w));

    protected string Where(SyntaxNode n) => $"{Path.GetFileName(path)}:{n.GetLocation().GetLineSpan().StartLinePosition.Line + 1}";

    protected void Count(string what) => census.AddOrUpdate(what, 1, (_, c) => c + 1);

    public override SyntaxNode? VisitQualifiedName(QualifiedNameSyntax node)
    {
        // Occt.NCollection_DataMap<K, V> -> System.Collections.Generic.Dictionary<K, V>
        if (node.Right is GenericNameSyntax g && model.GetSymbolInfo(node).Symbol is INamedTypeSymbol t && IsWrapper(t))
        {
            var visited = (GenericNameSyntax)VisitGenericName(g)!;
            Edits++;
            return QualifiedName(ParseName(NewNamespace), visited.WithIdentifier(Identifier(NewNameFor(t)).WithTriviaFrom(visited.Identifier)))
                .WithTriviaFrom(node);
        }
        return base.VisitQualifiedName(node);
    }

    public override SyntaxNode? VisitGenericName(GenericNameSyntax node)
    {
        var visited = (GenericNameSyntax)base.VisitGenericName(node)!;
        var symbol = model.GetSymbolInfo(node).Symbol;
        if (wrappers.Any(w => w?.Name == node.Identifier.ValueText)
            && (symbol is INamedTypeSymbol t && IsWrapper(t) || symbol is IMethodSymbol { MethodKind: MethodKind.Constructor } c && IsWrapper(c.ContainingType)
                || symbol is null && model.GetSymbolInfo(node).CandidateSymbols.Any(s => s is INamedTypeSymbol t2 && IsWrapper(t2))))
        {
            Edits++;
            Count("type");
            OnTypeRenamed();
            var named = symbol as INamedTypeSymbol ?? (symbol as IMethodSymbol)?.ContainingType ?? model.GetSymbolInfo(node).CandidateSymbols.OfType<INamedTypeSymbol>().First();
            return visited.WithIdentifier(Identifier(NewNameFor(named)).WithTriviaFrom(visited.Identifier));
        }
        return visited;
    }

    public override SyntaxNode? VisitIdentifierName(IdentifierNameSyntax node)
    {
        var symbol = model.GetSymbolInfo(node).Symbol;
        if (symbol is not null && IsWrapper(symbol.ContainingType) && symbol is IMethodSymbol { MethodKind: not MethodKind.Constructor }
            && node.Parent is not MemberAccessExpressionSyntax { Parent: InvocationExpressionSyntax })
            report.Add($"method group {symbol.Name} at {Where(node)}");
        if (symbol is not null && IsWrapper(symbol.ContainingType) && symbol is IPropertySymbol { Name: not "Keys" })
            report.Add($"property {symbol.Name} at {Where(node)}");
        return base.VisitIdentifierName(node);
    }

    // The rest is per wrapper

    // Whether the value at expr is written through (so a copy from Dictionary's indexer would lose the write)
    protected bool WrittenThrough(ExpressionSyntax expr, ITypeSymbol type)
    {
        var parent = expr.Parent;
        while (parent is ParenthesizedExpressionSyntax) { expr = (ExpressionSyntax)parent; parent = parent.Parent; }
        if (parent is RefExpressionSyntax) return true;
        if (parent is ArgumentSyntax a && (a.RefKindKeyword.IsKind(SyntaxKind.RefKeyword) || a.RefKindKeyword.IsKind(SyntaxKind.OutKeyword))) return true;
        if (!type.IsValueType) return false;
        if (type is INamedTypeSymbol { IsReadOnly: true }) return false;
        if (parent is MemberAccessExpressionSyntax ma && ma.Expression == expr)
        {
            var member = model.GetSymbolInfo(ma.Name).Symbol;
            switch (member)
            {
                case IMethodSymbol m:
                    return !m.IsReadOnly && !m.IsStatic && !m.IsExtensionMethod && !(m.ContainingType.IsReadOnly);
                case IFieldSymbol f:
                    return IsAssigned(ma) || WrittenThrough(ma, f.Type);
                case IPropertySymbol p:
                    return IsAssigned(ma) && p.SetMethod is { IsReadOnly: false } || (p.GetMethod is { IsReadOnly: false } && !p.ContainingType.IsReadOnly && p.ContainingType.TypeKind == TypeKind.Struct && !IsAutoProp(p));
                default:
                    return true;
            }
        }
        if (parent is ElementAccessExpressionSyntax ea && ea.Expression == expr)
        {
            // an indexer of a struct: inline arrays, spans; treat as written when assigned
            return IsAssigned(ea) || model.GetTypeInfo(ea).Type is { IsValueType: true } t && WrittenThrough(ea, t);
        }
        return false;
    }

    protected static bool IsAutoProp(IPropertySymbol p) => p.DeclaringSyntaxReferences.Any(r => r.GetSyntax() is PropertyDeclarationSyntax { AccessorList: { } al } && al.Accessors.All(x => x.Body is null && x.ExpressionBody is null));

    protected static bool IsAssigned(ExpressionSyntax e)
    {
        var parent = e.Parent;
        while (parent is ParenthesizedExpressionSyntax) { e = (ExpressionSyntax)parent; parent = parent.Parent; }
        return parent is AssignmentExpressionSyntax asg && asg.Left == e
            || parent is PrefixUnaryExpressionSyntax pu && (pu.IsKind(SyntaxKind.PreIncrementExpression) || pu.IsKind(SyntaxKind.PreDecrementExpression))
            || parent is PostfixUnaryExpressionSyntax po && (po.IsKind(SyntaxKind.PostIncrementExpression) || po.IsKind(SyntaxKind.PostDecrementExpression))
            || parent is RefExpressionSyntax
            || parent is ArgumentSyntax a && !a.RefKindKeyword.IsKind(SyntaxKind.None) && !a.RefKindKeyword.IsKind(SyntaxKind.InKeyword);
    }

    protected static bool ResultUnused(InvocationExpressionSyntax node) => node.Parent is ExpressionStatementSyntax
        || node.Parent is ArrowExpressionClauseSyntax { Parent: MethodDeclarationSyntax { ReturnType: PredefinedTypeSyntax { Keyword.ValueText: "void" } } }
        || node.Parent is ArrowExpressionClauseSyntax { Parent: LocalFunctionStatementSyntax { ReturnType: PredefinedTypeSyntax { Keyword.ValueText: "void" } } };

    protected static ExpressionSyntax Call(ExpressionSyntax receiver, SyntaxToken dot, string name, ArgumentListSyntax args)
        => InvocationExpression(MemberAccessExpression(SyntaxKind.SimpleMemberAccessExpression, receiver, dot, IdentifierName(name)), args);

    protected static ElementAccessExpressionSyntax Index(ExpressionSyntax receiver, ArgumentSyntax key)
        => ElementAccessExpression(receiver, BracketedArgumentList(SingletonSeparatedList(key.WithoutTrivia())));

    protected static ElementAccessExpressionSyntax Index(ExpressionSyntax receiver, ExpressionSyntax key)
        => ElementAccessExpression(receiver, BracketedArgumentList(SingletonSeparatedList(Argument(key))));

    protected ExpressionSyntax CountOf(ExpressionSyntax receiver) => MemberAccessExpression(SyntaxKind.SimpleMemberAccessExpression, receiver.WithoutTrailingTrivia(), IdentifierName("Count"));

    protected static bool SafeForComparison(SyntaxNode? parent) => parent is ArgumentSyntax or EqualsValueClauseSyntax or ReturnStatementSyntax
        or IfStatementSyntax or WhileStatementSyntax or DoStatementSyntax or ArrowExpressionClauseSyntax or ParenthesizedExpressionSyntax
        or AssignmentExpressionSyntax or BinaryExpressionSyntax { RawKind: (int)SyntaxKind.LogicalAndExpression or (int)SyntaxKind.LogicalOrExpression }
        or ConditionalExpressionSyntax;

    public override SyntaxNode? VisitPrefixUnaryExpression(PrefixUnaryExpressionSyntax node)
    {
        if (node.IsKind(SyntaxKind.LogicalNotExpression) && node.Operand is InvocationExpressionSyntax inv
            && inv.ArgumentList.Arguments.Count == 0 && model.GetSymbolInfo(inv).Symbol is IMethodSymbol { Name: "IsEmpty" } m && IsWrapper(m.ContainingType)
            && inv.Expression is MemberAccessExpressionSyntax ma)
        {
            var receiver = (ExpressionSyntax)Visit(ma.Expression)!;
            Edits++;
            Count("!IsEmpty");
            ExpressionSyntax result = BinaryExpression(SyntaxKind.NotEqualsExpression,
                MemberAccessExpression(SyntaxKind.SimpleMemberAccessExpression, receiver.WithoutTrailingTrivia(), IdentifierName("Count")),
                LiteralExpression(SyntaxKind.NumericLiteralExpression, Literal(0)));
            if (!SafeForComparison(node.Parent)) result = ParenthesizedExpression(result);
            return result.WithTriviaFrom(node).NormalizeWhitespaceKeepTrivia();
        }
        return base.VisitPrefixUnaryExpression(node);
    }


    protected virtual SyntaxNode? CopyCtor(BaseObjectCreationExpressionSyntax node, BaseObjectCreationExpressionSyntax visited)
    {
        if (model.GetSymbolInfo(node).Symbol is IMethodSymbol { MethodKind: MethodKind.Constructor } c && IsWrapper(c.ContainingType)
            && c.Parameters.Length == 1 && IsWrapper(c.Parameters[0].Type))
        {
            var arg = visited.ArgumentList!.Arguments[0].Expression;
            if (arg is not (IdentifierNameSyntax or MemberAccessExpressionSyntax { Expression: IdentifierNameSyntax or ThisExpressionSyntax }))
            {
                report.Add($"copy of a complex expression at {Where(node)}");
                return visited;
            }
            Edits++;
            Count("copy");
            var comparer = Argument(MemberAccessExpression(SyntaxKind.SimpleMemberAccessExpression, arg.WithoutTrivia(), IdentifierName("Comparer")));
            return visited.WithArgumentList(visited.ArgumentList.AddArguments(comparer));
        }
        return visited;
    }

    public override SyntaxNode? VisitObjectCreationExpression(ObjectCreationExpressionSyntax node)
        => CopyCtor(node, (ObjectCreationExpressionSyntax)base.VisitObjectCreationExpression(node)!);

    public override SyntaxNode? VisitImplicitObjectCreationExpression(ImplicitObjectCreationExpressionSyntax node)
        => CopyCtor(node, (ImplicitObjectCreationExpressionSyntax)base.VisitImplicitObjectCreationExpression(node)!);
}

sealed class DataMapRewriter(SemanticModel model, INamedTypeSymbol?[] wrappers, string path, ConcurrentBag<string> report,
                             ConcurrentDictionary<string, int> census) : WrapperRewriter(model, wrappers, path, report, census)
{
    protected override string NewName => "Dictionary";

    public override SyntaxNode? VisitElementAccessExpression(ElementAccessExpressionSyntax node)
    {
        if (model.GetSymbolInfo(node).Symbol is IPropertySymbol { IsIndexer: true } p && IsWrapper(p.ContainingType))
        {
            var valueType = p.Type;
            var receiver = (ExpressionSyntax)Visit(node.Expression)!;
            var args = (BracketedArgumentListSyntax)Visit(node.ArgumentList)!;
            Edits++; NeedsUsings.Add("Kerf.Engine.Collections");
            if (WrittenThrough(node, valueType) || IsAssigned(node))
            {
                Count("this[] by ref");
                return Call(receiver, Token(SyntaxKind.DotToken), "GetValueRef", ArgumentList(args.Arguments)).WithTriviaFrom(node);
            }
            Count("this[]");
            return Call(receiver, Token(SyntaxKind.DotToken), "GetValue", ArgumentList(args.Arguments)).WithTriviaFrom(node);
        }
        return base.VisitElementAccessExpression(node);
    }

    public override SyntaxNode? VisitInvocationExpression(InvocationExpressionSyntax node)
    {
        if (model.GetSymbolInfo(node).Symbol is not IMethodSymbol m || !IsWrapper(m.ContainingType))
            return base.VisitInvocationExpression(node);
        if (node.Expression is not MemberAccessExpressionSyntax ma)
        {
            report.Add($"unqualified {m.Name} at {Where(node)}");
            return base.VisitInvocationExpression(node);
        }
        var receiver = (ExpressionSyntax)Visit(ma.Expression)!;
        var args = (ArgumentListSyntax)Visit(node.ArgumentList)!;
        var dot = ma.OperatorToken;
        int n = args.Arguments.Count;
        var valueType = ((INamedTypeSymbol)m.ContainingType).TypeArguments[1];
        ExpressionSyntax? result = null;
        string tag = m.Name;
        switch (m.Name, n)
        {
            case ("Extent" or "Size", 0):
                result = MemberAccessExpression(SyntaxKind.SimpleMemberAccessExpression, receiver, dot, IdentifierName("Count"));
                if (ResultUnused(node)) report.Add($"statement {m.Name} at {Where(node)}");
                break;
            case ("IsEmpty", 0):
                result = BinaryExpression(SyntaxKind.EqualsExpression,
                    MemberAccessExpression(SyntaxKind.SimpleMemberAccessExpression, receiver.WithoutTrailingTrivia(), IdentifierName("Count")),
                    LiteralExpression(SyntaxKind.NumericLiteralExpression, Literal(0)));
                if (!SafeForComparison(node.Parent)) result = ParenthesizedExpression(result);
                return result.WithTriviaFrom(node).NormalizeWhitespaceKeepTrivia().Also(() => { Edits++; Count(tag); });
            case ("Clear", 0) or ("GetEnumerator", 0):
                Count(tag);
                return node.WithExpression(ma.WithExpression(receiver)).WithArgumentList(args);
            case ("ReSize", 1):
                result = Call(receiver, dot, "EnsureCapacity", args);
                break;
            case ("IsBound" or "Contains", 1):
                result = Call(receiver, dot, "ContainsKey", args);
                break;
            case ("UnBind", 1):
                result = Call(receiver, dot, "Remove", args);
                break;
            case ("TryBind", 2):
                result = Call(receiver, dot, "TryAdd", args);
                break;
            case ("Find", 2):
                result = Call(receiver, dot, "TryGetValue", args);
                break;
            case ("Find", 1):
                NeedsUsings.Add("Kerf.Engine.Collections");
                result = Call(receiver, dot, "GetValue", args);
                break;
            case ("ChangeFind", 1):
                if (WrittenThrough(node, valueType) || IsAssigned(node))
                {
                    tag = "ChangeFind by ref";
                    NeedsUsings.Add("Kerf.Engine.Collections");
                    result = Call(receiver, dot, "GetValueRef", args);
                }
                else
                {
                    NeedsUsings.Add("Kerf.Engine.Collections");
                    result = Call(receiver, dot, "GetValue", args);
                }
                break;
            case ("Bind", 2):
                if (ResultUnused(node))
                {
                    result = AssignmentExpression(SyntaxKind.SimpleAssignmentExpression, Index(receiver, args.Arguments[0]),
                        Token(SyntaxKind.EqualsToken).WithLeadingTrivia(Space).WithTrailingTrivia(Space),
                        args.Arguments[1].Expression.WithoutTrivia());
                }
                else
                {
                    tag = "Bind used";
                    report.Add($"Bind's result used at {Where(node)}");
                    result = null;
                }
                break;
            case ("Bound", 2):
                tag = ResultUnused(node) ? "Bound unused" : "Bound";
                NeedsUsings.Add("Kerf.Engine.Collections");
                result = Call(receiver, dot, "SetValueRef", args);
                break;
            case ("TryBound", 2):
                if (ResultUnused(node))
                {
                    tag = "TryBound unused";
                    result = Call(receiver, dot, "TryAdd", args);
                }
                else
                {
                    NeedsUsings.Add("Kerf.Engine.Collections");
                    result = Call(receiver, dot, "GetOrAddValueRef", args);
                }
                break;
            default:
                report.Add($"member {m.Name}/{n} at {Where(node)}");
                break;
        }
        Count(tag);
        if (result is null) return node.WithExpression(ma.WithExpression(receiver)).WithArgumentList(args);
        Edits++;
        if (receiver.GetTrailingTrivia().Any(t => t.IsKind(SyntaxKind.EndOfLineTrivia)) || dot.LeadingTrivia.Any())
            report.Add($"multi-line receiver ({m.Name}) at {Where(node)}");
        return result.WithTriviaFrom(node);
    }
}

static class Ext
{
    public static ExpressionSyntax NormalizeWhitespaceKeepTrivia(this ExpressionSyntax e)
    {
        var lead = e.GetLeadingTrivia();
        var trail = e.GetTrailingTrivia();
        return e.NormalizeWhitespace().WithLeadingTrivia(lead).WithTrailingTrivia(trail);
    }

    public static T Also<T>(this T t, Action a) { a(); return t; }
}
