// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Kernel.Foundation.Tests;

public class StreamFormatErrorExceptionTests
{
    [Fact]
    public void It_carries_its_message_and_cause()
    {
        var cause = new InvalidOperationException("cause");
        var error = new StreamFormatErrorException("message", cause);
        Assert.Equal("message", error.Message);
        Assert.Same(cause, error.InnerException);
        Assert.Null(new StreamFormatErrorException().InnerException);
        Assert.IsAssignableFrom<FailureException>(new StreamFormatErrorException("message"));
    }
}
