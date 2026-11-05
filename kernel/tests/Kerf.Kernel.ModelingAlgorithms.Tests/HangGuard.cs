// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Runs a call that OCCT could hang in, failing the test if it doesn't return within a limit (Kerf unit 10K1's tests of
/// fixed hangs). The call runs on a background thread, so a hang can't keep the test host from exiting.
/// </summary>
internal static class HangGuard
{
    public static T Within<T>(TimeSpan theLimit, Func<T> theCall)
    {
        T aResult = default!;
        Exception? anError = null;
        Thread aThread = new(() =>
        {
            try
            {
                aResult = theCall();
            }
            catch (Exception anException)
            {
                anError = anException;
            }
        }, 16 << 20)
        { IsBackground = true };
        aThread.Start();
        Assert.True(aThread.Join(theLimit), $"the call didn't return within {theLimit.TotalSeconds} s (a hang)");
        if (anError is not null)
        {
            System.Runtime.ExceptionServices.ExceptionDispatchInfo.Capture(anError).Throw();
        }
        return aResult;
    }

    public static void Within(TimeSpan theLimit, Action theCall) => Within(theLimit, () =>
    {
        theCall();
        return 0;
    });
}
