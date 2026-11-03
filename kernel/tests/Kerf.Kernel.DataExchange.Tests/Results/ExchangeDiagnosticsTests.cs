// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Tests of the bridge from the exchange framework's statuses to diagnostics (unit 10F).

using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Foundation;

namespace Kerf.Kernel.DataExchange.Tests.Results;

public class ExchangeDiagnosticsTests
{
    [Theory]
    [InlineData(ReturnStatus.RetVoid, "exchange.nothing-done", "Nothing was read or written.")]
    [InlineData(ReturnStatus.RetError, "exchange.invalid-data", "The data isn't correct.")]
    [InlineData(ReturnStatus.RetFail, "exchange.failed", "Reading or writing failed.")]
    [InlineData(ReturnStatus.RetStop, "exchange.stopped", "Reading or writing was stopped.")]
    public void FailingStatusesAreErrors(ReturnStatus status, string code, string message)
    {
        var diagnostic = ExchangeDiagnostics.ToDiagnostic(status)!;
        Assert.True(diagnostic.IsError);
        Assert.Equal(code, diagnostic.Code.Value);
        Assert.Equal(message, diagnostic.Message);
    }

    [Fact]
    public void DoneIsNoError()
    {
        Assert.Null(ExchangeDiagnostics.ToDiagnostic(ReturnStatus.RetDone));
        Assert.Equal(FailureDiagnostics.Fault, ExchangeDiagnostics.ToDiagnostic((ReturnStatus)99)!.Code);
    }
}
