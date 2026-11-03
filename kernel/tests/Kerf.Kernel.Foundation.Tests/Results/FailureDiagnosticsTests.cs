// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Tests of the bridge from the engine's exceptions to diagnostics (unit 10F).

using Kerf.Engine.Foundation;

namespace Kerf.Kernel.Foundation.Tests.Results;

public class FailureDiagnosticsTests
{
    [Theory]
    [InlineData(typeof(NotDoneException), "engine.not-done")]
    [InlineData(typeof(ConstructionErrorException), "engine.construction-error")]
    [InlineData(typeof(DomainErrorException), "engine.domain-error")]
    [InlineData(typeof(FailureException), "engine.failure")]
    [InlineData(typeof(NullVectorException), "engine.vector-with-null-magnitude")]
    [InlineData(typeof(StreamFormatErrorException), "engine.stream-format-error")]
    [InlineData(typeof(OutOfRangeException), "engine.out-of-range")]
    [InlineData(typeof(IndexOutOfRangeException), "engine.fault")]
    [InlineData(typeof(NullReferenceException), "engine.fault")]
    public void CodesComeFromOcctsNames(Type type, string code)
    {
        Assert.Equal(code, FailureDiagnostics.CodeOf(type).Value);
    }

    [Theory]
    [InlineData("NotDone", "not-done")]
    [InlineData("VectorWithNullMagnitude", "vector-with-null-magnitude")]
    [InlineData("BOPNotAllowed", "bop-not-allowed")]
    [InlineData("RemovalOfIBForEdgesFailed", "removal-of-ib-for-edges-failed")]
    [InlineData("PostTreatFF", "post-treat-ff")]
    [InlineData("C0Geometry", "c0-geometry")]
    [InlineData("Null3DCurve", "null3d-curve")]
    [InlineData("LocOpeNotDone", "loc-ope-not-done")]
    public void KebabCase(string name, string kebab)
    {
        Assert.Equal(kebab, FailureDiagnostics.Kebab(name));
    }

    [Fact]
    public void EveryOcctExceptionHasAWellFormedCode()
    {
        foreach (var type in typeof(FailureException).Assembly.GetTypes().Where(t => typeof(FailureException).IsAssignableFrom(t)))
            Assert.StartsWith("engine.", FailureDiagnostics.CodeOf(type).Value);
    }

    [Fact]
    public void TheMessageSaysTheKindThenOcctsMessage()
    {
        var diagnostic = FailureDiagnostics.ToDiagnostic(new NotDoneException("BRepAlgoAPI_BuilderAlgo::Shape"));
        Assert.True(diagnostic.IsError);
        Assert.Equal("engine.not-done", diagnostic.Code.Value);
        Assert.Equal("the algorithm couldn't complete: BRepAlgoAPI_BuilderAlgo::Shape", diagnostic.Message);
    }

    [Fact]
    public void AFailureWithoutAMessageSaysOnlyItsKind()
    {
        Assert.Equal("a value is outside what the operation accepts", FailureDiagnostics.ToDiagnostic(new DomainErrorException()).Message);
        Assert.Equal("the geometry can't be constructed", FailureDiagnostics.ToDiagnostic(new ConstructionErrorException("")).Message);
    }

    [Fact]
    public void AnotherExceptionIsAFault()
    {
        var diagnostic = FailureDiagnostics.ToDiagnostic(new IndexOutOfRangeException("Index was outside the bounds of the array."));
        Assert.Equal(FailureDiagnostics.Fault, diagnostic.Code);
        Assert.Equal("an internal fault: Index was outside the bounds of the array.", diagnostic.Message);
    }
}
