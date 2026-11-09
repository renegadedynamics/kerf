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

    // Each form the engine's messages take; none of them may reach a user as it is (unit 11L)
    [Theory]
    [InlineData("gp_Dir() - input vector has zero norm", "input vector has zero norm")]
    [InlineData("gp_Circ::SetRadius() - radius should be positive number", "radius should be positive number")]
    [InlineData("gp_GTrsf::SetForm, null determinant", "null determinant")]
    [InlineData(" Geom_OffsetSurface : UIso", "UIso")]
    [InlineData("BRepTools::Read: bad header", "bad header")]
    [InlineData("TopoDS_Builder::Add", "")]
    [InlineData("math_Gauss ", "")]
    [InlineData("BRepAlgoAPI_Failure", "")]
    [InlineData(" Make Geom_Curve (3D) failed", "Make Curve (3D) failed")]
    [InlineData("list(TopoDS_Shape)", "list(Shape)")]
    [InlineData("called from BRepFill_Sweep::BuildEdge too early", "called from BuildEdge too early")]
    [InlineData("BSpline curve: invalid degree", "BSpline curve: invalid degree")]
    [InlineData("No CARTESIAN_POINT in entity 12", "No CARTESIAN_POINT in entity 12")]
    [InlineData("radius should be >= 0", "radius should be >= 0")]
    public void PublicTextIsInKerfsTerms(string engine, string expected)
    {
        Assert.Equal(expected, FailureDiagnostics.PublicText(engine));
    }

    [Fact]
    public void PublicTextLeavesAnyOtherExceptionsMessageAlone()
    {
        const string message = "Access to C:/My_Files/a_B.step is denied";
        Assert.Equal(message, FailureDiagnostics.PublicText(new IOException(message)));
    }

    [Fact]
    public void ARefusalAndADiagnosticNameNoOcctType()
    {
        var refusal = KernelAssert.Throws<ArgumentException>(
            () => Refusal.ArgumentIf(true, FailureKind.ConstructionError, "gp_Circ() - radius should be positive number"));
        Assert.Equal("The geometry can't be constructed: radius should be positive number.", refusal.Message.Split(" (Parameter")[0]);
        Assert.Equal("the geometry can't be constructed: radius should be positive number", FailureDiagnostics.ToDiagnostic(refusal).Message);
        Assert.Equal("the algorithm couldn't complete: command not done",
                     FailureDiagnostics.ToDiagnostic(new NotDoneException("BRep_API: command not done")).Message);
        // The engine's own failure keeps OCCT's words: only what leaves changes
        Assert.Contains("gp_Circ()", ((FailureException)refusal.InnerException!).GetMessageString());
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
        var diagnostic = FailureDiagnostics.ToDiagnostic(new NotDoneException("BRepAlgoAPI_BuilderAlgo::Shape: not built"));
        Assert.True(diagnostic.IsError);
        Assert.Equal("engine.not-done", diagnostic.Code.Value);
        Assert.Equal("the algorithm couldn't complete: not built", diagnostic.Message);
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
