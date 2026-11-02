namespace Build123d.Occt;

/// <summary>Information about the OCCT build: here, OCCT.NET's port.</summary>
public static partial class Occt
{
    /// <summary>The OCCT version OCCT.NET ports (its occt-version.json): V8_0_1.</summary>
    public static string Version => "8.0.1";

    /// <summary>
    /// Under the native backend, makes the shim read <paramref name="address"/> to test fault handling. OCCT.NET runs no
    /// native code, so there is no fault to make.
    /// </summary>
    public static int FaultForTesting(IntPtr address) =>
        throw new NotSupportedException("OCCT.NET runs no native code, so it has no native faults to test");
}
