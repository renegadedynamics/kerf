using System.Collections.Concurrent;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using static Microsoft.CodeAnalysis.CSharp.SyntaxFactory;

/// <summary>NCollection_DynamicArray (and NCollection_Vector) to List.</summary>
sealed class DynArrayRewriter(SemanticModel model, INamedTypeSymbol?[] wrappers, string path, ConcurrentBag<string> report,
                              ConcurrentDictionary<string, int> census) : WrapperRewriter(model, wrappers, path, report, census)
{
    protected override string NewName => "List";

    ExpressionSyntax SpanOf(ExpressionSyntax receiver)
    {
        NeedsUsings.Add("System.Runtime.InteropServices");
        return InvocationExpression(MemberAccessExpression(SyntaxKind.SimpleMemberAccessExpression, IdentifierName("CollectionsMarshal"), IdentifierName("AsSpan")),
            ArgumentList(SingletonSeparatedList(Argument(receiver.WithoutTrivia()))));
    }

    static ExpressionSyntax FromEnd(int n) => PrefixUnaryExpression(SyntaxKind.IndexExpression, LiteralExpression(SyntaxKind.NumericLiteralExpression, Literal(n)));

    static ExpressionSyntax Lit(int n) => LiteralExpression(SyntaxKind.NumericLiteralExpression, Literal(n));

    static ExpressionSyntax Spaced(SyntaxKind kind, ExpressionSyntax l, SyntaxKind op, ExpressionSyntax r)
        => BinaryExpression(kind, l, Token(op).WithLeadingTrivia(Space).WithTrailingTrivia(Space), r);

    // r.Append(v) whose result is used: T x = r.Append(v); -> T x = v; r.Add(x);  ref T x = ref r.Append(v); -> r.Add(v); ref T x = ref AsSpan(r)[^1];
    // p = r.Append(v); -> p = v; r.Add(p);
    InvocationExpressionSyntax? AppendCall(ExpressionSyntax? e)
    {
        if (e is RefExpressionSyntax re) e = re.Expression;
        return e is InvocationExpressionSyntax { Expression: MemberAccessExpressionSyntax { Name.Identifier.ValueText: "Append" } ma, ArgumentList.Arguments.Count: 1 } inv
            && model.GetSymbolInfo(inv).Symbol is IMethodSymbol m && IsWrapper(m.ContainingType)
            && ma.Expression is IdentifierNameSyntax or MemberAccessExpressionSyntax { Expression: IdentifierNameSyntax or ThisExpressionSyntax } ? inv : null;
    }

    static ExpressionStatementSyntax AddStatement(ExpressionSyntax receiver, ExpressionSyntax value)
        => ExpressionStatement(InvocationExpression(MemberAccessExpression(SyntaxKind.SimpleMemberAccessExpression, receiver.WithoutTrivia(), IdentifierName("Add")),
            ArgumentList(SingletonSeparatedList(Argument(value.WithoutTrivia())))));

    public override SyntaxNode? VisitBlock(BlockSyntax node)
    {
        var list = new List<StatementSyntax>();
        bool changed = false;
        foreach (var st in node.Statements)
        {
            if (st is LocalDeclarationStatementSyntax { Declaration.Variables: [{ Initializer.Value: var init } decl] } ld && AppendCall(init) is { } inv)
            {
                var ma = (MemberAccessExpressionSyntax)inv.Expression;
                var receiver = (ExpressionSyntax)Visit(ma.Expression)!;
                var value = (ExpressionSyntax)Visit(inv.ArgumentList.Arguments[0].Expression)!;
                var lead = st.GetLeadingTrivia();
                var indent = lead.Where(t => t.IsKind(SyntaxKind.WhitespaceTrivia)).LastOrDefault();
                var eol = st.GetTrailingTrivia();
                if (init is RefExpressionSyntax)
                {
                    var add = AddStatement(receiver, value).WithLeadingTrivia(lead).WithTrailingTrivia(eol);
                    var newDecl = ld.WithDeclaration(ld.Declaration.WithVariables(SingletonSeparatedList(decl.WithInitializer(decl.Initializer!.WithValue(
                        RefExpression(Token(SyntaxKind.RefKeyword).WithTrailingTrivia(Space), Index(SpanOf(receiver), FromEnd(1))).WithLeadingTrivia(Space)))))).WithLeadingTrivia(indent).WithTrailingTrivia(eol);
                    list.Add(add); list.Add(newDecl);
                    Count("Append ref declaration");
                }
                else
                {
                    var newDecl = ld.WithDeclaration(ld.Declaration.WithVariables(SingletonSeparatedList(decl.WithInitializer(decl.Initializer!.WithValue(value.WithLeadingTrivia(Space).WithoutTrailingTrivia())))));
                    var add = AddStatement(receiver, IdentifierName(decl.Identifier.ValueText)).WithLeadingTrivia(indent).WithTrailingTrivia(eol);
                    list.Add(newDecl); list.Add(add);
                    Count("Append declaration");
                }
                Edits++; changed = true;
                continue;
            }
            if (st is ExpressionStatementSyntax { Expression: AssignmentExpressionSyntax { RawKind: (int)SyntaxKind.SimpleAssignmentExpression, Left: IdentifierNameSyntax target, Right: var right } asg } es
                && AppendCall(right) is { } inv2 && right is not RefExpressionSyntax)
            {
                var ma = (MemberAccessExpressionSyntax)inv2.Expression;
                var receiver = (ExpressionSyntax)Visit(ma.Expression)!;
                var value = (ExpressionSyntax)Visit(inv2.ArgumentList.Arguments[0].Expression)!;
                var lead = st.GetLeadingTrivia();
                var indent = lead.Where(t => t.IsKind(SyntaxKind.WhitespaceTrivia)).LastOrDefault();
                var eol = st.GetTrailingTrivia();
                list.Add(es.WithExpression(asg.WithRight(value.WithLeadingTrivia(Space).WithoutTrailingTrivia())));
                list.Add(AddStatement(receiver, target).WithLeadingTrivia(indent).WithTrailingTrivia(eol));
                Count("Append assignment");
                Edits++; changed = true;
                continue;
            }
            list.Add((StatementSyntax)Visit(st)!);
        }
        var visited = node.WithOpenBraceToken(VisitToken(node.OpenBraceToken)).WithCloseBraceToken(VisitToken(node.CloseBraceToken));
        return changed || true ? visited.WithStatements(List(list)) : visited;
    }

    public override SyntaxNode? VisitElementAccessExpression(ElementAccessExpressionSyntax node)
    {
        if (model.GetSymbolInfo(node).Symbol is IPropertySymbol { IsIndexer: true } p && IsWrapper(p.ContainingType))
        {
            var receiver = (ExpressionSyntax)Visit(node.Expression)!;
            var args = (BracketedArgumentListSyntax)Visit(node.ArgumentList)!;
            if (WrittenThrough(node, p.Type))
            {
                Edits++; Count("this[] by ref");
                return ElementAccessExpression(SpanOf(receiver), args).WithTriviaFrom(node);
            }
            Count("this[]");
            return node.WithExpression(receiver).WithArgumentList(args);
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
        var elementType = ((INamedTypeSymbol)m.ContainingType).TypeArguments[0];
        ExpressionSyntax? result = null;
        string tag = m.Name;
        bool unused = ResultUnused(node);
        switch (m.Name, n)
        {
            case ("Size" or "Length", 0):
                result = CountOf(receiver);
                break;
            case ("Lower", 0):
                result = Lit(0);
                if (receiver is not (IdentifierNameSyntax or MemberAccessExpressionSyntax { Expression: IdentifierNameSyntax or ThisExpressionSyntax }))
                    report.Add($"Lower of a complex receiver at {Where(node)}");
                break;
            case ("Upper", 0):
                result = Spaced(SyntaxKind.SubtractExpression, CountOf(receiver), SyntaxKind.MinusToken, Lit(1));
                if (!SafeForComparison(node.Parent) || node.Parent is BinaryExpressionSyntax) result = ParenthesizedExpression(result);
                break;
            case ("IsEmpty", 0):
                result = Spaced(SyntaxKind.EqualsExpression, CountOf(receiver), SyntaxKind.EqualsEqualsToken, Lit(0));
                if (!SafeForComparison(node.Parent)) result = ParenthesizedExpression(result);
                break;
            case ("Clear", 0) or ("GetEnumerator", 0):
                Count(tag);
                return node.WithExpression(ma.WithExpression(receiver)).WithArgumentList(args);
            case ("Append", 1):
                if (unused) result = Call(receiver, dot, "Add", args);
                else { tag = "Append used"; report.Add($"Append's result used at {Where(node)}"); }
                break;
            case ("InsertBefore", 2):
                if (unused) result = Call(receiver, dot, "Insert", args);
                else { tag = "InsertBefore used"; report.Add($"InsertBefore's result used at {Where(node)}"); }
                break;
            case ("InsertAfter", 2):
                if (unused)
                {
                    var i = args.Arguments[0];
                    var i1 = Spaced(SyntaxKind.AddExpression, i.Expression.WithoutTrivia() is BinaryExpressionSyntax or ConditionalExpressionSyntax ? ParenthesizedExpression(i.Expression.WithoutTrivia()) : i.Expression.WithoutTrivia(), SyntaxKind.PlusToken, Lit(1));
                    result = Call(receiver, dot, "Insert", args.WithArguments(args.Arguments.Replace(i, i.WithExpression(i1))));
                }
                else { tag = "InsertAfter used"; report.Add($"InsertAfter's result used at {Where(node)}"); }
                break;
            case ("Value", 1):
                result = Index(receiver, args.Arguments[0]);
                break;
            case ("ChangeValue", 1):
                if (WrittenThrough(node, elementType)) { tag = "ChangeValue by ref"; result = Index(SpanOf(receiver), args.Arguments[0]); }
                else result = Index(receiver, args.Arguments[0]);
                break;
            case ("First", 0):
                result = Index(receiver, Lit(0));
                break;
            case ("Last", 0):
                result = Index(receiver, FromEnd(1));
                break;
            case ("ChangeFirst", 0):
                if (WrittenThrough(node, elementType)) { tag = "ChangeFirst by ref"; result = Index(SpanOf(receiver), Lit(0)); }
                else result = Index(receiver, Lit(0));
                break;
            case ("ChangeLast", 0):
                if (WrittenThrough(node, elementType)) { tag = "ChangeLast by ref"; result = Index(SpanOf(receiver), FromEnd(1)); }
                else result = Index(receiver, FromEnd(1));
                break;
            case ("SetValue", 2) when unused:
                NeedsUsings.Add("Kerf.Engine.Collections");
                result = Call(receiver, dot, "SetAt", args);
                break;
            case ("AsSpan", 0):
                result = SpanOf(receiver);
                break;
            default:
                tag = m.Name + " (by hand)";
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

    protected override SyntaxNode? CopyCtor(BaseObjectCreationExpressionSyntax node, BaseObjectCreationExpressionSyntax visited)
    {
        if (model.GetSymbolInfo(node).Symbol is IMethodSymbol { MethodKind: MethodKind.Constructor } c && IsWrapper(c.ContainingType)
            && c.Parameters.Length == 1 && c.Parameters[0].Type.SpecialType == SpecialType.System_Int32 && visited.ArgumentList is { Arguments.Count: 1 })
        {
            var arg = visited.ArgumentList!.Arguments[0].Expression;
            // OCCT's block size: 256 (the default) was taken lazily as the first capacity (9E); List grows from 4.
            if (arg is LiteralExpressionSyntax { Token.Value: 256 or 0 })
            {
                Edits++; Count("ctor 256/0 dropped");
                return visited.WithArgumentList(visited.ArgumentList.WithArguments(default));
            }
            Count("ctor capacity");
            report.Add($"capacity {arg} at {Where(node)}");
        }
        return visited;
    }
}
