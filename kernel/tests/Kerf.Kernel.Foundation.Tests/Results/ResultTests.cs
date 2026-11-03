// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Tests of Result<T>, Result, Diagnostic, DiagnosticCode and KerfException (unit 10F; docs/api/README.md, section 2).

using System.Collections.Immutable;

namespace Kerf.Kernel.Foundation.Tests.Results;

public class DiagnosticCodeTests
{
    [Theory]
    [InlineData("input.invalid")]
    [InlineData("boolean.self-interfering-shape")]
    [InlineData("engine.not-done")]
    [InlineData("offset.c0-geometry")]
    [InlineData("fault")]
    public void WellFormedCodesAreAccepted(string text)
    {
        var code = new DiagnosticCode(text);
        Assert.Equal(text, code.Value);
        Assert.Equal(text, code.ToString());
        Assert.Equal(new DiagnosticCode(text), code);
    }

    [Theory]
    [InlineData("")]
    [InlineData("Input.Invalid")]
    [InlineData("input..invalid")]
    [InlineData(".input")]
    [InlineData("input.")]
    [InlineData("input invalid")]
    [InlineData("input.-invalid")]
    [InlineData("input_invalid")]
    public void MalformedCodesThrow(string text)
    {
        Assert.Throws<ArgumentException>(() => new DiagnosticCode(text));
    }

    [Fact]
    public void ANullCodeThrows()
    {
        Assert.Throws<ArgumentNullException>(() => new DiagnosticCode(null!));
    }

    [Fact]
    public void TheWellKnownCodes()
    {
        Assert.Equal("input.invalid", DiagnosticCode.InvalidInput.Value);
        Assert.Equal("numerics.not-converged", DiagnosticCode.NotConverged.Value);
        Assert.Equal("operation.cancelled", DiagnosticCode.Cancelled.Value);
        Assert.Equal("result.default", DiagnosticCode.DefaultResult.Value);
    }

    [Fact]
    public void TheDefaultCodeHasNoText()
    {
        Assert.Null(default(DiagnosticCode).Value);
        Assert.Equal("", default(DiagnosticCode).ToString());
    }
}

public class DiagnosticTests
{
    [Fact]
    public void ErrorsAndWarnings()
    {
        var error = Diagnostic.Error(DiagnosticCode.InvalidInput, "The radius is too large.");
        Assert.Equal(Severity.Error, error.Severity);
        Assert.True(error.IsError);
        Assert.Equal(DiagnosticCode.InvalidInput, error.Code);
        Assert.Equal("The radius is too large.", error.Message);
        Assert.Equal("error input.invalid: The radius is too large.", error.ToString());

        var warning = Diagnostic.Warning(DiagnosticCode.NotConverged, "Close enough.");
        Assert.False(warning.IsError);
        Assert.Equal("warning numerics.not-converged: Close enough.", warning.ToString());
    }

    [Fact]
    public void DiagnosticsCompareByValue()
    {
        Assert.Equal(Diagnostic.Error(DiagnosticCode.InvalidInput, "a"), Diagnostic.Error(DiagnosticCode.InvalidInput, "a"));
        Assert.NotEqual(Diagnostic.Error(DiagnosticCode.InvalidInput, "a"), Diagnostic.Warning(DiagnosticCode.InvalidInput, "a"));
        Assert.NotEqual(Diagnostic.Error(DiagnosticCode.InvalidInput, "a"), Diagnostic.Error(DiagnosticCode.InvalidInput, "b"));
    }

    [Fact]
    public void ADiagnosticNeedsACodeAndAMessage()
    {
        Assert.Throws<ArgumentException>(() => Diagnostic.Error(default, "a"));
        Assert.Throws<ArgumentNullException>(() => Diagnostic.Error(DiagnosticCode.InvalidInput, null!));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Diagnostic { Severity = (Severity)7, Code = DiagnosticCode.InvalidInput, Message = "a" });
    }

    [Fact]
    public void WithKeepsTheChecks()
    {
        var error = Diagnostic.Error(DiagnosticCode.InvalidInput, "a");
        Assert.Equal("b", (error with { Message = "b" }).Message);
        Assert.Throws<ArgumentException>(() => error with { Code = default });
    }
}

public class ResultTests
{
    private static readonly Diagnostic Bad = Diagnostic.Error(DiagnosticCode.InvalidInput, "Bad input.");
    private static readonly Diagnostic Careful = Diagnostic.Warning(DiagnosticCode.NotConverged, "Careful.");

    [Fact]
    public void ASuccessHasItsValue()
    {
        var result = Result<int>.Success(42);
        Assert.True(result.IsSuccess);
        Assert.False(result.IsFailure);
        Assert.Equal(42, result.Value);
        Assert.Equal(42, result.GetValueOrThrow());
        Assert.Equal(42, result.GetValueOrDefault(7));
        Assert.True(result.TryGetValue(out var value));
        Assert.Equal(42, value);
        Assert.Null(result.Error);
        Assert.Empty(result.Diagnostics);
        Assert.Equal("Success: 42", result.ToString());
    }

    [Fact]
    public void AValueConvertsToASuccess()
    {
        Result<string> result = "made";
        Assert.True(result.IsSuccess);
        Assert.Equal("made", result.Value);
    }

    [Fact]
    public void ASuccessKeepsItsWarnings()
    {
        var result = Result<int>.Success(1, [Careful]);
        Assert.True(result.IsSuccess);
        Assert.Equal<Diagnostic>([Careful], result.Diagnostics);
        Assert.Null(result.Error);
    }

    [Fact]
    public void ASuccessCantCarryAnError()
    {
        Assert.Throws<ArgumentException>(() => Result<int>.Success(1, [Bad]));
        Assert.Throws<ArgumentException>(() => Result<int>.Success(1, [null!]));
    }

    [Fact]
    public void AFailureHasItsError()
    {
        var result = Result<int>.Failure(Bad, [Careful]);
        Assert.False(result.IsSuccess);
        Assert.True(result.IsFailure);
        Assert.Same(Bad, result.Error);
        Assert.Equal<Diagnostic>([Careful, Bad], result.Diagnostics);
        Assert.False(result.TryGetValue(out var value));
        Assert.Equal(0, value);
        Assert.Equal(7, result.GetValueOrDefault(7));
        Assert.Equal("Failure: error input.invalid: Bad input.", result.ToString());
    }

    [Fact]
    public void AFailedValueThrowsWithTheDiagnostics()
    {
        var result = Result<int>.Failure(Bad, [Careful]);
        var e = Assert.Throws<KerfException>(() => result.Value);
        Assert.Equal("Bad input.", e.Message);
        Assert.Equal<Diagnostic>([Careful, Bad], e.Diagnostics);
        Assert.Same(Bad, e.Error);
        Assert.Throws<KerfException>(() => result.GetValueOrThrow());
    }

    [Fact]
    public void AFailureNeedsAnError()
    {
        Assert.Throws<ArgumentException>(() => Result<int>.Failure(Careful));
        Assert.Throws<ArgumentNullException>(() => Result<int>.Failure((Diagnostic)null!));
        Assert.Throws<ArgumentException>(() => Result<int>.Failure([Careful]));
        Assert.Throws<ArgumentException>(() => Result<int>.Failure(ImmutableArray<Diagnostic>.Empty));
        Assert.Throws<ArgumentException>(() => Result<int>.Failure(Bad, [Bad]));
    }

    [Fact]
    public void AFailureMayHaveSeveralErrors()
    {
        var other = Diagnostic.Error(new DiagnosticCode("boolean.self-interfering-shape"), "Self-interfering.");
        var result = Result<int>.Failure([Careful, Bad, other]);
        Assert.Same(Bad, result.Error);
        var e = Assert.Throws<KerfException>(() => result.Value);
        Assert.Equal("Bad input. (and 1 more error)", e.Message);
    }

    [Fact]
    public void ADefaultResultIsAFailure()
    {
        var result = default(Result<int>);
        Assert.False(result.IsSuccess);
        Assert.Equal(DiagnosticCode.DefaultResult, result.Error!.Code);
        Assert.Single(result.Diagnostics);
        Assert.Throws<KerfException>(() => result.Value);
    }

    [Fact]
    public void MapKeepsTheWarnings()
    {
        var mapped = Result<int>.Success(2, [Careful]).Map(x => x * 1.5);
        Assert.Equal(3.0, mapped.Value);
        Assert.Equal<Diagnostic>([Careful], mapped.Diagnostics);
    }

    [Fact]
    public void MapKeepsAFailure()
    {
        bool called = false;
        var mapped = Result<int>.Failure(Bad, [Careful]).Map(x => { called = true; return x.ToString(); });
        Assert.False(called);
        Assert.False(mapped.IsSuccess);
        Assert.Equal<Diagnostic>([Careful, Bad], mapped.Diagnostics);
    }

    [Fact]
    public void BindPutsEarlierWarningsFirst()
    {
        var other = Diagnostic.Warning(new DiagnosticCode("other.warning"), "Other.");
        var bound = Result<int>.Success(2, [Careful]).Bind(x => Result<string>.Success(x.ToString(), [other]));
        Assert.Equal("2", bound.Value);
        Assert.Equal<Diagnostic>([Careful, other], bound.Diagnostics);

        var failed = Result<int>.Success(2, [Careful]).Bind(_ => Result<string>.Failure(Bad));
        Assert.False(failed.IsSuccess);
        Assert.Equal<Diagnostic>([Careful, Bad], failed.Diagnostics);
        Assert.Same(Bad, failed.Error);
    }

    [Fact]
    public void BindKeepsAFailure()
    {
        bool called = false;
        var bound = Result<int>.Failure(Bad).Bind(x => { called = true; return Result<int>.Success(x); });
        Assert.False(called);
        Assert.Same(Bad, bound.Error);
    }

    [Fact]
    public void MapAndBindRejectNull()
    {
        Assert.Throws<ArgumentNullException>(() => Result<int>.Success(1).Map<int>(null!));
        Assert.Throws<ArgumentNullException>(() => Result<int>.Success(1).Bind<int>(null!));
    }

    [Fact]
    public void AReferenceValueMayBeNull()
    {
        var result = Result<string?>.Success(null);
        Assert.True(result.IsSuccess);
        Assert.Null(result.Value);
    }
}

public class ResultWithoutValueTests
{
    private static readonly Diagnostic Bad = Diagnostic.Error(DiagnosticCode.InvalidInput, "Bad input.");
    private static readonly Diagnostic Careful = Diagnostic.Warning(DiagnosticCode.NotConverged, "Careful.");

    [Fact]
    public void SuccessAndFailure()
    {
        var success = Result.Success([Careful]);
        Assert.True(success.IsSuccess);
        Assert.Equal<Diagnostic>([Careful], success.Diagnostics);
        Assert.Null(success.Error);
        success.ThrowIfFailed();
        Assert.Equal("Success", success.ToString());

        var failure = Result.Failure(Bad, [Careful]);
        Assert.True(failure.IsFailure);
        Assert.Same(Bad, failure.Error);
        Assert.Equal<Diagnostic>([Careful, Bad], failure.Diagnostics);
        Assert.Equal("Bad input.", Assert.Throws<KerfException>(failure.ThrowIfFailed).Message);
    }

    [Fact]
    public void TheChecks()
    {
        Assert.Throws<ArgumentException>(() => Result.Success([Bad]));
        Assert.Throws<ArgumentException>(() => Result.Failure(Careful));
        Assert.Throws<ArgumentException>(() => Result.Failure([Careful]));
    }

    [Fact]
    public void ADefaultResultIsAFailure()
    {
        Assert.Equal(DiagnosticCode.DefaultResult, default(Result).Error!.Code);
        Assert.Throws<KerfException>(default(Result).ThrowIfFailed);
    }

    [Fact]
    public void MapAndBind()
    {
        var mapped = Result.Success([Careful]).Map(() => 5);
        Assert.Equal(5, mapped.Value);
        Assert.Equal<Diagnostic>([Careful], mapped.Diagnostics);
        Assert.Same(Bad, Result.Failure(Bad).Map(() => 5).Error);

        var other = Diagnostic.Warning(new DiagnosticCode("other.warning"), "Other.");
        var bound = Result.Success([Careful]).Bind(() => Result.Success([other]));
        Assert.True(bound.IsSuccess);
        Assert.Equal<Diagnostic>([Careful, other], bound.Diagnostics);
        var failed = Result.Success([Careful]).Bind(() => Result.Failure(Bad));
        Assert.Equal<Diagnostic>([Careful, Bad], failed.Diagnostics);
        Assert.Same(Bad, Result.Failure(Bad).Bind(() => Result.Success()).Error);
    }
}

public class KerfExceptionTests
{
    [Fact]
    public void ItCarriesTheDiagnosticsAndTheCause()
    {
        var cause = new InvalidOperationException("inner");
        var error = Diagnostic.Error(DiagnosticCode.InvalidInput, "Bad input.");
        var e = new KerfException(error, cause);
        Assert.Equal("Bad input.", e.Message);
        Assert.Same(cause, e.InnerException);
        Assert.Same(error, e.Error);
        Assert.Equal<Diagnostic>([error], e.Diagnostics);
    }

    [Fact]
    public void ItNeedsAnError()
    {
        Assert.Throws<ArgumentException>(() => new KerfException([Diagnostic.Warning(DiagnosticCode.NotConverged, "a")]));
        Assert.Throws<ArgumentNullException>(() => new KerfException((Diagnostic)null!));
    }
}
