using System.Collections.Concurrent;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using static Microsoft.CodeAnalysis.CSharp.SyntaxFactory;

/// <summary>
/// NCollection_IndexedMap to IndexedSet and NCollection_IndexedDataMap to IndexedMap, 1-based to 0-based. Every index
/// that crosses the boundary moves by one at the call (i - 1 in, IndexOf + 1 out), so the code's own numbering is
/// unchanged; loops from 1 to Extent() whose variable only indexes the map become loops from 0.
/// </summary>
sealed class IndexedRewriter(SemanticModel model, INamedTypeSymbol?[] wrappers, string path, ConcurrentBag<string> report,
                             ConcurrentDictionary<string, int> census) : WrapperRewriter(model, wrappers, path, report, census)
{
    protected override string NewName => "IndexedSet";
    protected override string NewNamespace => "Kerf.Engine.Collections";
    protected override string NewNameFor(INamedTypeSymbol t) => t.TypeArguments.Length == 2 ? "IndexedMap" : "IndexedSet";
    protected override void OnTypeRenamed() => NeedsUsings.Add("Kerf.Engine.Collections");

    // Loop variables that now run from 0 and only index the map
    readonly HashSet<string> zeroBased = new(StringComparer.Ordinal);

    bool IsDataMap(ITypeSymbol t) => ((INamedTypeSymbol)t).TypeArguments.Length == 2;

    static ExpressionSyntax Lit(int n) => LiteralExpression(SyntaxKind.NumericLiteralExpression, Literal(n));

    static BinaryExpressionSyntax Spaced(SyntaxKind kind, ExpressionSyntax l, SyntaxKind op, ExpressionSyntax r)
        => BinaryExpression(kind, l, Token(op).WithLeadingTrivia(Space).WithTrailingTrivia(Space), r);

    static bool LowerThanAdditive(ExpressionSyntax e) => e is ConditionalExpressionSyntax or AssignmentExpressionSyntax or LambdaExpressionSyntax
        or BinaryExpressionSyntax { RawKind: not ((int)SyntaxKind.AddExpression or (int)SyntaxKind.SubtractExpression or (int)SyntaxKind.MultiplyExpression
            or (int)SyntaxKind.DivideExpression or (int)SyntaxKind.ModuloExpression) };

    /// <summary>The 0-based index for a 1-based one.</summary>
    ExpressionSyntax Minus1(ExpressionSyntax e)
    {
        e = e.WithoutTrivia();
        if (e is IdentifierNameSyntax idn && zeroBased.Contains(idn.Identifier.ValueText))
            return e;
        if (e is LiteralExpressionSyntax { Token.Value: int n })
            return Lit(n - 1);
        if (e is BinaryExpressionSyntax { RawKind: (int)SyntaxKind.AddExpression, Right: LiteralExpressionSyntax { Token.Value: 1 } } add)
            return add.Left.WithoutTrivia();
        if (e is BinaryExpressionSyntax { RawKind: (int)SyntaxKind.SubtractExpression, Right: LiteralExpressionSyntax { Token.Value: int k } } sub)
            return sub.WithRight(Lit(k + 1));
        return Spaced(SyntaxKind.SubtractExpression, LowerThanAdditive(e) ? ParenthesizedExpression(e) : e, SyntaxKind.MinusToken, Lit(1));
    }

    ArgumentListSyntax Minus1(ArgumentListSyntax args, params int[] which)
    {
        var list = args.Arguments;
        foreach (int i in which)
            list = list.Replace(list[i], list[i].WithExpression(Minus1(list[i].Expression)));
        return args.WithArguments(list);
    }

    /// <summary>x + 1, parenthesised where the parent binds tighter or subtracts it.</summary>
    ExpressionSyntax Plus1(ExpressionSyntax e, SyntaxNode? parent, SyntaxNode original)
    {
        // a string concatenation would append the digit
        if (parent is BinaryExpressionSyntax { RawKind: (int)SyntaxKind.AddExpression } concat
            && model.GetTypeInfo(concat).Type?.SpecialType == SpecialType.System_String)
            return ParenthesizedExpression(Spaced(SyntaxKind.AddExpression, e, SyntaxKind.PlusToken, Lit(1)));
        ExpressionSyntax r = Spaced(SyntaxKind.AddExpression, e, SyntaxKind.PlusToken, Lit(1));
        bool safe = parent is ArgumentSyntax or EqualsValueClauseSyntax or ReturnStatementSyntax or ParenthesizedExpressionSyntax or ArrowExpressionClauseSyntax
            or ExpressionStatementSyntax
            || parent is AssignmentExpressionSyntax a && a.Right == original
            || parent is BinaryExpressionSyntax b && (b.RawKind is (int)SyntaxKind.EqualsExpression or (int)SyntaxKind.NotEqualsExpression
                or (int)SyntaxKind.LessThanExpression or (int)SyntaxKind.LessThanOrEqualExpression or (int)SyntaxKind.GreaterThanExpression
                or (int)SyntaxKind.GreaterThanOrEqualExpression or (int)SyntaxKind.LogicalAndExpression or (int)SyntaxKind.LogicalOrExpression
                || b.IsKind(SyntaxKind.AddExpression) || b.IsKind(SyntaxKind.SubtractExpression) && b.Left == original);
        return safe ? r : ParenthesizedExpression(r);
    }

    IMethodSymbol? WrapperMethod(ExpressionSyntax e)
        => e is InvocationExpressionSyntax inv && model.GetSymbolInfo(inv).Symbol is IMethodSymbol m && IsWrapper(m.ContainingType) ? m : null;

    // FindIndex(k) compared with 0 or 1: Contains
    public override SyntaxNode? VisitBinaryExpression(BinaryExpressionSyntax node)
    {
        ExpressionSyntax? call = null; int lit = -1; bool callLeft = true;
        if (node.Right is LiteralExpressionSyntax { Token.Value: int r } && WrapperMethod(node.Left) is { Name: "FindIndex" }) { call = node.Left; lit = r; }
        else if (node.Left is LiteralExpressionSyntax { Token.Value: int l } && WrapperMethod(node.Right) is { Name: "FindIndex" }) { call = node.Right; lit = l; callLeft = false; }
        if (call is InvocationExpressionSyntax { Expression: MemberAccessExpressionSyntax ma } inv)
        {
            var kind = (SyntaxKind)node.RawKind;
            if (!callLeft)
                kind = kind switch
                {
                    SyntaxKind.LessThanExpression => SyntaxKind.GreaterThanExpression,
                    SyntaxKind.GreaterThanExpression => SyntaxKind.LessThanExpression,
                    SyntaxKind.LessThanOrEqualExpression => SyntaxKind.GreaterThanOrEqualExpression,
                    SyntaxKind.GreaterThanOrEqualExpression => SyntaxKind.LessThanOrEqualExpression,
                    _ => kind,
                };
            bool? contains = (kind, lit) switch
            {
                (SyntaxKind.EqualsExpression, 0) or (SyntaxKind.LessThanExpression, 1) or (SyntaxKind.LessThanOrEqualExpression, 0) => false,
                (SyntaxKind.NotEqualsExpression, 0) or (SyntaxKind.GreaterThanExpression, 0) or (SyntaxKind.GreaterThanOrEqualExpression, 1) => true,
                _ => null,
            };
            if (contains is { } c)
            {
                var m = WrapperMethod(inv)!;
                var receiver = (ExpressionSyntax)Visit(ma.Expression)!;
                var args = (ArgumentListSyntax)Visit(inv.ArgumentList)!;
                ExpressionSyntax result = Call(receiver, ma.OperatorToken, IsDataMap(m.ContainingType) ? "ContainsKey" : "Contains", args);
                if (!c) result = PrefixUnaryExpression(SyntaxKind.LogicalNotExpression, result);
                Edits++; Count("FindIndex compared");
                return result.WithTriviaFrom(node);
            }
        }
        return base.VisitBinaryExpression(node);
    }

    // for (int i = 1; i <= m.Extent(); i++) whose i only indexes m: for (int i = 0; i < m.Count; i++)
    public override SyntaxNode? VisitForStatement(ForStatementSyntax node)
    {
        if (node.Declaration is { Variables: [{ Initializer.Value: LiteralExpressionSyntax { Token.Value: 1 } } v] } decl
            && node.Condition is BinaryExpressionSyntax { RawKind: (int)SyntaxKind.LessThanOrEqualExpression, Left: IdentifierNameSyntax iv } cond
            && iv.Identifier.ValueText == v.Identifier.ValueText
            && node.Incrementors is [var inc] && IsIncrement(inc, v.Identifier.ValueText)
            && model.GetDeclaredSymbol(v) is ILocalSymbol loopVar
            && BoundReceiver(cond.Right) is { } bound)
        {
            var refs = node.Statement.DescendantNodes().OfType<IdentifierNameSyntax>()
                .Where(id => id.Identifier.ValueText == loopVar.Name && SymbolEqualityComparer.Default.Equals(model.GetSymbolInfo(id).Symbol, loopVar)).ToList();
            if (refs.Count > 0 && refs.All(id => IsIndexOf(id, bound)))
            {
                zeroBased.Add(loopVar.Name);
                var newDecl = decl.WithVariables(SingletonSeparatedList(v.WithInitializer(v.Initializer!.WithValue(Lit(0).WithTriviaFrom(v.Initializer.Value)))));
                var newCond = cond.WithOperatorToken(Token(SyntaxKind.LessThanToken).WithTriviaFrom(cond.OperatorToken)).WithRight((ExpressionSyntax)Visit(cond.Right)!);
                var body = (StatementSyntax)Visit(node.Statement)!;
                zeroBased.Remove(loopVar.Name);
                Edits++; Count("loop from 0");
                return node.WithDeclaration(newDecl).WithCondition(newCond).WithStatement(body);
            }
        }
        return base.VisitForStatement(node);
    }

    static bool IsIncrement(ExpressionSyntax inc, string name)
        => inc is PostfixUnaryExpressionSyntax { RawKind: (int)SyntaxKind.PostIncrementExpression, Operand: IdentifierNameSyntax a } && a.Identifier.ValueText == name
        || inc is PrefixUnaryExpressionSyntax { RawKind: (int)SyntaxKind.PreIncrementExpression, Operand: IdentifierNameSyntax b } && b.Identifier.ValueText == name;

    // The map whose size bounds the loop: m.Extent()/m.Size(), or a local initialised from it and never written again
    ISymbol? BoundReceiver(ExpressionSyntax bound)
    {
        if (bound is InvocationExpressionSyntax { Expression: MemberAccessExpressionSyntax ma, ArgumentList.Arguments.Count: 0 } inv
            && WrapperMethod(inv) is { Name: "Extent" or "Size" })
            return SimpleReceiver(ma.Expression);
        if (bound is IdentifierNameSyntax id && model.GetSymbolInfo(id).Symbol is ILocalSymbol local
            && local.DeclaringSyntaxReferences is [var r] && r.GetSyntax() is VariableDeclaratorSyntax { Initializer.Value: var init }
            && !IsWrittenAfterDeclaration(local, r.GetSyntax()))
            return BoundReceiver(init);
        return null;
    }

    bool IsWrittenAfterDeclaration(ILocalSymbol local, SyntaxNode declarator)
    {
        var scope = declarator.Ancestors().OfType<BlockSyntax>().LastOrDefault() ?? declarator.Parent!;
        return scope.DescendantNodes().OfType<IdentifierNameSyntax>()
            .Where(id => id.Identifier.ValueText == local.Name && SymbolEqualityComparer.Default.Equals(model.GetSymbolInfo(id).Symbol, local))
            .Any(id => IsAssigned(id));
    }

    ISymbol? SimpleReceiver(ExpressionSyntax e)
        => e is IdentifierNameSyntax or MemberAccessExpressionSyntax { Expression: ThisExpressionSyntax } ? model.GetSymbolInfo(e).Symbol : null;

    // id is the index argument of an access to the bound map: m.FindKey(i), m.FindFromIndex(i), m.ChangeFromIndex(i), m[i]
    bool IsIndexOf(IdentifierNameSyntax id, ISymbol map)
    {
        if (id.Parent is not ArgumentSyntax arg) return false;
        if (arg.Parent is ArgumentListSyntax { Arguments.Count: 1, Parent: InvocationExpressionSyntax { Expression: MemberAccessExpressionSyntax ma } inv }
            && WrapperMethod(inv) is { Name: "FindKey" or "FindFromIndex" or "ChangeFromIndex" })
            return SymbolEqualityComparer.Default.Equals(SimpleReceiver(ma.Expression), map);
        if (arg.Parent is BracketedArgumentListSyntax { Arguments.Count: 1, Parent: ElementAccessExpressionSyntax ea }
            && model.GetSymbolInfo(ea).Symbol is IPropertySymbol { IsIndexer: true } p && IsWrapper(p.ContainingType))
            return SymbolEqualityComparer.Default.Equals(SimpleReceiver(ea.Expression), map);
        return false;
    }

    public override SyntaxNode? VisitElementAccessExpression(ElementAccessExpressionSyntax node)
    {
        if (model.GetSymbolInfo(node).Symbol is IPropertySymbol { IsIndexer: true } p && IsWrapper(p.ContainingType))
        {
            var receiver = (ExpressionSyntax)Visit(node.Expression)!;
            var args = (BracketedArgumentListSyntax)Visit(node.ArgumentList)!;
            var index = Minus1(args.Arguments[0].Expression);
            Edits++;
            if (IsDataMap(p.ContainingType))
            {
                Count("data this[]");
                return Call(receiver, Token(SyntaxKind.DotToken), "ValueAt", ArgumentList(SingletonSeparatedList(Argument(index)))).WithTriviaFrom(node);
            }
            Count("set this[]");
            return node.WithExpression(receiver).WithArgumentList(args.WithArguments(SingletonSeparatedList(args.Arguments[0].WithExpression(index))));
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
        bool data = IsDataMap(m.ContainingType);
        ExpressionSyntax? result = null;
        string tag = (data ? "data " : "set ") + m.Name;
        bool unused = ResultUnused(node);
        switch (m.Name, n)
        {
            case ("Extent" or "Size", 0):
                result = CountOf(receiver);
                break;
            case ("IsEmpty", 0):
                result = Spaced(SyntaxKind.EqualsExpression, CountOf(receiver), SyntaxKind.EqualsEqualsToken, Lit(0));
                if (!SafeForComparison(node.Parent)) result = ParenthesizedExpression(result);
                break;
            case ("Clear", 0) or ("RemoveLast", 0) or ("GetEnumerator", 0):
                Count(tag);
                return node.WithExpression(ma.WithExpression(receiver)).WithArgumentList(args);
            case ("Add", 1) or ("Add", 2):
                if (unused) { Count(tag); return node.WithExpression(ma.WithExpression(receiver)).WithArgumentList(args); }
                tag += " used";
                result = Plus1(Call(receiver, dot, "Add", args), node.Parent, node);
                break;
            case ("Contains", 1):
                if (!data) { Count(tag); return node.WithExpression(ma.WithExpression(receiver)).WithArgumentList(args); }
                result = Call(receiver, dot, "ContainsKey", args);
                break;
            case ("FindIndex", 1):
                result = Plus1(Call(receiver, dot, "IndexOf", args), node.Parent, node);
                break;
            case ("FindKey", 1):
                result = data ? Call(receiver, dot, "KeyAt", Minus1(args, 0))
                              : ElementAccessExpression(receiver, BracketedArgumentList(SingletonSeparatedList(Argument(Minus1(args.Arguments[0].Expression)))));
                break;
            case ("FindFromIndex" or "ChangeFromIndex", 1):
                result = Call(receiver, dot, "ValueAt", Minus1(args, 0));
                break;
            case ("FindFromKey", 1):
                result = Call(receiver, dot, "GetValue", args);
                break;
            case ("FindFromKey", 2):
                result = Call(receiver, dot, "TryGetValue", args);
                break;
            case ("ChangeFromKey", 1):
                var valueType = ((INamedTypeSymbol)m.ContainingType).TypeArguments[1];
                result = Call(receiver, dot, WrittenThrough(node, valueType) || IsAssigned(node) ? "GetValueRef" : "GetValue", args);
                break;
            case ("RemoveFromIndex", 1):
                result = Call(receiver, dot, "RemoveAtSwap", Minus1(args, 0));
                break;
            case ("RemoveKey", 1):
                result = Call(receiver, dot, "Remove", args);
                break;
            case ("Substitute", 2):
                result = Call(receiver, dot, "Substitute", Minus1(args, 0));
                break;
            case ("Swap", 2):
                result = Call(receiver, dot, "Swap", Minus1(args, 0, 1));
                break;
            default:
                tag += " (by hand)";
                report.Add($"member {m.Name}/{n}{(unused ? "" : " used")} at {Where(node)}");
                break;
        }
        Count(tag);
        if (result is null) return node.WithExpression(ma.WithExpression(receiver)).WithArgumentList(args);
        Edits++;
        if (receiver.GetTrailingTrivia().Any(t => t.IsKind(SyntaxKind.EndOfLineTrivia)) || dot.LeadingTrivia.Any())
            report.Add($"multi-line receiver ({m.Name}) at {Where(node)}");
        return result.WithTriviaFrom(node);
    }

    protected override SyntaxNode? CopyCtor(BaseObjectCreationExpressionSyntax node, BaseObjectCreationExpressionSyntax visited) => visited;
}
