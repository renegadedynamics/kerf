namespace Kerf.Modeling;

/// <summary>
/// The modelling layer's own diagnostic codes, for the failures a caller should expect that the layer itself detects
/// (a fillet whose result is invalid, a draft a face can't take). The kernel's failures keep the kernel's codes
/// (<c>boolean.self-interfering-shape</c>, <c>fillet.walking-failed</c>), with the operation said in the message.
/// </summary>
/// <remarks>
/// A failed operation (<see cref="Operations.Fillet(IEnumerable{Edge}, double)"/>, <see cref="Part.Subtract"/>,
/// <see cref="Importers.ImportStep(Stream, ImportLimits?, CancellationToken, string)"/>…) returns a failed
/// <see cref="Result{T}"/> carrying these diagnostics. Misuse (a null argument, an index out of range)
/// throws .NET's argument exceptions.
/// </remarks>
public static class ModelingDiagnostics
{
    /// <summary>A 3D or 2D fillet couldn't be built, or was invalid: <c>modeling.fillet-failed</c>.</summary>
    public static DiagnosticCode FilletFailed { get; } = new("modeling.fillet-failed");

    /// <summary>A chamfer couldn't be built, or was invalid: <c>modeling.chamfer-failed</c>.</summary>
    public static DiagnosticCode ChamferFailed { get; } = new("modeling.chamfer-failed");

    /// <summary>A warning from a sweep, such as which frame it fell back on: <c>modeling.sweep-warning</c>.</summary>
    public static DiagnosticCode SweepWarning { get; } = new("modeling.sweep-warning");

    /// <summary>An offset couldn't be built: <c>modeling.offset-failed</c>.</summary>
    public static DiagnosticCode OffsetFailed { get; } = new("modeling.offset-failed");

    /// <summary>A surface couldn't be thickened into a solid: <c>modeling.thicken-failed</c>.</summary>
    public static DiagnosticCode ThickenFailed { get; } = new("modeling.thicken-failed");

    /// <summary>A loft was invalid and couldn't be recovered: <c>modeling.loft-failed</c>.</summary>
    public static DiagnosticCode LoftFailed { get; } = new("modeling.loft-failed");

    /// <summary>A draft couldn't be added to a face, or the drafted solid failed; the face is the diagnostic's shape: <c>modeling.draft-failed</c>.</summary>
    public static DiagnosticCode DraftFailed { get; } = new("modeling.draft-failed");

    /// <summary>A spline couldn't be interpolated or approximated through the points: <c>modeling.spline-failed</c>.</summary>
    public static DiagnosticCode SplineFailed { get; } = new("modeling.spline-failed");

    /// <summary>
    /// A convex polyhedron couldn't be made: the points' hull has no volume (fewer than four distinct points, or all on
    /// one line or in one plane), a coordinate isn't finite, or the solid wasn't valid: <c>modeling.hull-failed</c>.
    /// </summary>
    public static DiagnosticCode HullFailed { get; } = new("modeling.hull-failed");

    /// <summary>
    /// No shape fits the values an object was given, each valid on its own: no arc of that radius touches both targets, no
    /// triangle has those sides, a rounded rectangle's radius is too large for it: <c>modeling.no-solution</c>.
    /// </summary>
    public static DiagnosticCode NoSolution { get; } = new("modeling.no-solution");

    /// <summary>An arc or line meant to end at a limit, or at another curve, never meets it: <c>modeling.limit-not-reached</c>.</summary>
    public static DiagnosticCode LimitNotReached { get; } = new("modeling.limit-not-reached");

    /// <summary>
    /// An object couldn't be built from the values it was given, for a reason found while building it (two points of a
    /// polyline the same, a slot too short to have a side): <c>modeling.object-failed</c>. A failure of the kernel's keeps
    /// the kernel's code.
    /// </summary>
    public static DiagnosticCode ObjectFailed { get; } = new("modeling.object-failed");

    /// <summary>An argument the kernel refused, said in context: <c>kernel.invalid-argument</c>.</summary>
    public static DiagnosticCode KernelInvalidArgument { get; } = new("kernel.invalid-argument");

    /// <summary>A warning from <see cref="ExportDxf"/>: edges off the XY plane were projected onto it:
    /// <c>modeling.dxf-not-planar</c>.</summary>
    public static DiagnosticCode DxfNotPlanar { get; } = new("modeling.dxf-not-planar");

    /// <summary>
    /// What an operation made that isn't valid, from the exception a shorthand threw for it (<c>result.invalid</c>): the
    /// operators and constructors can't return a <see cref="Result{T}"/>, whose <c>TryGetInvalid</c> gives the same. To look at
    /// or to draw, not to build on (decision 1742). <see cref="ModelingException.TryGetInvalid"/> is the same.
    /// </summary>
    public static bool TryGetInvalid(Exception failure, out Shape invalid)
    {
        ArgumentNullException.ThrowIfNull(failure);
        invalid = Failures.InvalidOf(failure) ?? default;
        return Failures.InvalidOf(failure) is not null;
    }
}
