namespace Build123d.Occt;

/// <summary>
/// An exception thrown by OCCT, such as Standard_ConstructionError or StdFail_NotDone. Its message starts with what
/// was being done when OCCT failed (see <see cref="OcctContext.Doing"/>), e.g. "fuse failed: StdFail_NotDone: …".
/// Under the OCCT.NET backend it carries OCCT.NET's exception (a Standard_Failure) as its inner exception.
/// </summary>
public sealed class OcctException : Exception
{
    public OcctException(string occtType, string message)
        : this(occtType, message, OcctContext.Current, null)
    {
    }

    private OcctException(string occtType, string message, string? operation, Exception? inner)
        : base(Compose(occtType, message, operation), inner)
    {
        OcctType = occtType;
        OcctMessage = message;
        Operation = operation;
    }

    /// <summary>The OCCT exception class name.</summary>
    public string OcctType { get; }

    /// <summary>OCCT's own message, which may be empty.</summary>
    public string OcctMessage { get; }

    /// <summary>What was being done, outermost first ("extrude / fuse"), or null if nothing said.</summary>
    public string? Operation { get; }

    /// <summary>Whether OCCT reported a fault (an access violation, say) rather than a failure of its own.</summary>
    public bool IsFault => OcctType.StartsWith("OSD_Exception", StringComparison.Ordinal) || OcctType == "unknown";

    private static string Compose(string occtType, string message, string? operation)
    {
        var what = message.Length > 0 ? $"{occtType}: {message}" : occtType;
        return operation is null ? what : $"{operation} failed: {what}";
    }

    /// <summary>The exception for OCCT.NET's <paramref name="failure"/>, as the native shim reports OCCT's.</summary>
    internal static OcctException From(global::Occt.Standard_Failure failure)
    {
        var message = failure.GetMessageString();
        // A failure raised without a message has .NET's default one, where OCCT's would be empty
        if (message.StartsWith("Exception of type '", StringComparison.Ordinal))
            message = "";
        return new OcctException(failure.OcctType, message, OcctContext.Current, failure);
    }
}
