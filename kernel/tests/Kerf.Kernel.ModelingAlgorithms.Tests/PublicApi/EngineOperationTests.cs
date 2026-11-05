// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using EngineBox = Kerf.Engine.Construction.Api.MakeBox;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.PublicApi;

/// <summary>The one way public operations run engine algorithms (decision 1369, unit 10L1).</summary>
public class EngineOperationTests
{
    private static readonly Diagnostic Failed = Diagnostic.Error(new DiagnosticCode("construction.failed"), "The box failed.");

    private static EngineOutcome Box(double size)
    {
        var box = new EngineBox(size, size, size);
        box.Build();
        return new EngineOutcome(box.IsDone, () => box.Shape());
    }

    [Fact]
    public void A_done_algorithm_gives_its_shape_and_history()
    {
        Result<Built<Solid>> result = EngineOperation.Run<Solid>(_ => Box(2), Failed);
        Assert.True(result.IsSuccess);
        Assert.Equal(6, result.Value.Shape.AllFaces().Length);
        Assert.False(result.Value.History.HasModified);
        Result<Built<Shape>> asShape = EngineOperation.Run<Shape>(_ => Box(2), Failed);
        Assert.Equal(ShapeType.Solid, asShape.Value.Shape.Type);
    }

    [Fact]
    public void An_engine_failure_is_a_failed_result()
    {
        Result<Built<Solid>> result = EngineOperation.Run<Solid>(_ => Box(0), Failed);
        Assert.False(result.IsSuccess);
        Assert.NotNull(result.Error);
        Assert.StartsWith("engine.", result.Error!.Code.Value, StringComparison.Ordinal);
    }

    [Fact]
    public void An_algorithm_not_done_gives_the_operations_own_error()
    {
        Result<Built<Solid>> result = EngineOperation.Run<Solid>(_ => new EngineOutcome(false, () => throw new InvalidOperationException()), Failed);
        Assert.False(result.IsSuccess);
        Assert.Equal(Failed, result.Error);
    }

    [Fact]
    public void A_result_of_another_type_is_a_failure()
    {
        Result<Built<Face>> result = EngineOperation.Run<Face>(_ => Box(1), Failed);
        Assert.False(result.IsSuccess);
        Assert.Contains("Solid", result.Error!.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void Cancellation_throws_rather_than_failing()
    {
        using var source = new CancellationTokenSource();
        source.Cancel();
        Assert.Throws<OperationCanceledException>(() => EngineOperation.Run<Solid>(_ => Box(1), Failed, cancellationToken: source.Token));
        using var later = new CancellationTokenSource();
        Assert.Throws<OperationCanceledException>(() => EngineOperation.Run<Solid>(range =>
        {
            later.Cancel();
            Assert.True(range.UserBreak());
            return Box(1);
        }, Failed, cancellationToken: later.Token));
    }
}
