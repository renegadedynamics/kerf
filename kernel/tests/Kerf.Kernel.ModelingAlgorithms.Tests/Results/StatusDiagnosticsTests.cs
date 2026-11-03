// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Tests of the bridges from the modelling algorithms' statuses and the Booleans' alerts to diagnostics (unit 10F).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Features;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Offsets;
using Kerf.Engine.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.Results;

public class StatusDiagnosticsTests
{
    // Every value of each status enum: the first (done) gives null, the others an error with a code of their own
    private static void CheckAll<TEnum>(Func<TEnum, Diagnostic?> bridge, string area) where TEnum : struct, Enum
    {
        var values = Enum.GetValues<TEnum>();
        Assert.Null(bridge(values[0]));
        var codes = new HashSet<string>();
        foreach (var value in values.Skip(1))
        {
            var diagnostic = bridge(value);
            Assert.NotNull(diagnostic);
            Assert.True(diagnostic.IsError);
            Assert.StartsWith(area + ".", diagnostic.Code.Value);
            Assert.True(codes.Add(diagnostic.Code.Value), $"{value}: {diagnostic.Code} is another status's code");
            Assert.EndsWith(".", diagnostic.Message);
        }
        var unknown = bridge((TEnum)(object)999);
        Assert.Equal(FailureDiagnostics.Fault, unknown!.Code);
    }

    [Fact]
    public void Construction()
    {
        CheckAll<WireError>(StatusDiagnostics.ToDiagnostic, "construction");
        CheckAll<EdgeError>(StatusDiagnostics.ToDiagnostic, "construction");
        CheckAll<FaceError>(StatusDiagnostics.ToDiagnostic, "construction");
        CheckAll<ShellError>(StatusDiagnostics.ToDiagnostic, "construction");
        CheckAll<PipeError>(StatusDiagnostics.ToDiagnostic, "construction");
        CheckAll<ThruSectionErrorStatus>(StatusDiagnostics.ToDiagnostic, "construction");
    }

    [Fact]
    public void FilletsOffsetsAndFeatures()
    {
        CheckAll<ErrorStatus>(StatusDiagnostics.ToDiagnostic, "fillet");
        CheckAll<Error>(StatusDiagnostics.ToDiagnostic, "offset");
        CheckAll<SimpleOffsetStatus>(StatusDiagnostics.ToDiagnostic, "offset");
        CheckAll<StatusError>(StatusDiagnostics.ToDiagnostic, "feature");
    }

    [Fact]
    public void SomeCodes()
    {
        Assert.Equal("construction.disconnected-wire", StatusDiagnostics.ToDiagnostic(WireError.DisconnectedWire)!.Code.Value);
        Assert.Equal("fillet.walking-failed", StatusDiagnostics.ToDiagnostic(ErrorStatus.WalkingFailure)!.Code.Value);
        Assert.Equal("offset.c0-geometry", StatusDiagnostics.ToDiagnostic(Error.C0Geometry)!.Code.Value);
        var feature = StatusDiagnostics.ToDiagnostic(StatusError.LocOpeNotDone)!;
        Assert.Equal("feature.loc-ope-not-done", feature.Code.Value);
        Assert.Equal("Local Operation not done.", feature.Message);
    }

    [Fact]
    public void ABooleansAlertsBecomeDiagnostics()
    {
        var fuse = new Fuse();
        fuse.Build();
        Assert.True(fuse.HasErrors);
        var error = Assert.Single(AlertDiagnostics.ToDiagnostics(fuse.GetReport()));
        Assert.Equal(Kerf.Severity.Error, error.Severity);
        Assert.Equal("boolean.too-few-arguments", error.Code.Value);
        Assert.Equal("There are no enough arguments to perform the operation", error.Message);
    }

    [Fact]
    public void BooleanAlertCodes()
    {
        Assert.Equal("boolean.self-interfering-shape", AlertDiagnostics.CodeOf(new AlertSelfInterferingShape(new Kerf.Topology.Shape())).Value);
        Assert.Equal("boolean.bop-not-allowed", AlertDiagnostics.CodeOf(new AlertBopNotAllowed()).Value);
        Assert.Equal("boolean.removal-of-ib-for-edges-failed", AlertDiagnostics.CodeOf(new AlertRemovalOfIBForEdgesFailed(new Kerf.Topology.Shape())).Value);
    }
}
