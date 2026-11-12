// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Features;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Offsets;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The bridge from the modelling algorithms' status enums to diagnostics (phase10-plan.md 2.5): each failing status is
/// an error with a stable code and a plain message; a status that says the work was done is null. docs/api/diagnostics.md
/// lists the codes.
/// </summary>
internal static class StatusDiagnostics
{
    /// <summary>The error a wire builder's status reports, or null if it built the wire.</summary>
    public static Diagnostic? ToDiagnostic(WireError status) => status switch
    {
        WireError.WireDone => null,
        WireError.EmptyWire => Error("construction.empty-wire", "The wire has no edges."),
        WireError.DisconnectedWire => Error("construction.disconnected-wire", "An edge isn't connected to the rest of the wire."),
        WireError.NonManifoldWire => Error("construction.non-manifold-wire", "The edges branch: more than two meet at a vertex."),
        _ => Unknown(status),
    };

    /// <summary>The error an edge builder's status reports, or null if it built the edge.</summary>
    public static Diagnostic? ToDiagnostic(EdgeError status) => status switch
    {
        EdgeError.EdgeDone => null,
        EdgeError.PointProjectionFailed => Error("construction.point-projection-failed", "An end point couldn't be projected onto the curve."),
        EdgeError.ParameterOutOfRange => Error("construction.parameter-out-of-range", "An end parameter is outside the curve's range."),
        EdgeError.DifferentPointsOnClosedCurve => Error("construction.different-points-on-closed-curve", "The end points of an edge on a closed curve differ where they should coincide."),
        EdgeError.PointWithInfiniteParameter => Error("construction.point-with-infinite-parameter", "An end point was given at an infinite parameter."),
        EdgeError.DifferentsPointAndParameter => Error("construction.point-and-parameter-differ", "An end point isn't at the curve's point for its parameter."),
        EdgeError.LineThroughIdenticPoints => Error("construction.line-through-identical-points", "A line can't pass through two identical points."),
        _ => Unknown(status),
    };

    /// <summary>The error a face builder's status reports, or null if it built the face.</summary>
    public static Diagnostic? ToDiagnostic(FaceError status) => status switch
    {
        FaceError.FaceDone => null,
        FaceError.NoFace => Error("construction.no-face", "No face was built: the input gave no surface."),
        FaceError.NotPlanar => Error("construction.not-planar", "The wire isn't planar, so no plane holds it."),
        FaceError.CurveProjectionFailed => Error("construction.curve-projection-failed", "An edge's curve couldn't be projected onto the surface."),
        FaceError.ParametersOutOfRange => Error("construction.parameters-out-of-range", "The parameters are outside the surface's range."),
        _ => Unknown(status),
    };

    /// <summary>The error a shell builder's status reports, or null if it built the shell.</summary>
    public static Diagnostic? ToDiagnostic(ShellError status) => status switch
    {
        ShellError.ShellDone => null,
        ShellError.EmptyShell => Error("construction.empty-shell", "The shell has no faces."),
        ShellError.DisconnectedShell => Error("construction.disconnected-shell", "A face isn't connected to the rest of the shell."),
        ShellError.ShellParametersOutOfRange => Error("construction.shell-parameters-out-of-range", "The parameters are outside the surface's range."),
        _ => Unknown(status),
    };

    /// <summary>The error a pipe's status reports, or null if it built the pipe.</summary>
    public static Diagnostic? ToDiagnostic(PipeError status) => status switch
    {
        PipeError.PipeDone => null,
        PipeError.PipeNotDone => Error("construction.pipe-not-done", "The sweep couldn't be built."),
        PipeError.PlaneNotIntersectGuide => Error("construction.plane-not-intersecting-guide", "A section's plane doesn't meet the guide."),
        PipeError.ImpossibleContact => Error("construction.impossible-contact", "The profile can't keep contact with the guide."),
        _ => Unknown(status),
    };

    /// <summary>The error a loft's status reports, or null if it built the loft.</summary>
    public static Diagnostic? ToDiagnostic(ThruSectionErrorStatus status) => status switch
    {
        ThruSectionErrorStatus.ThruSectionErrorStatus_Done => null,
        ThruSectionErrorStatus.ThruSectionErrorStatus_NotDone => Error("construction.loft-not-done", "The loft couldn't be built."),
        ThruSectionErrorStatus.ThruSectionErrorStatus_NotSameTopology => Error("construction.loft-sections-differ", "The sections don't have the same number of edges, or are mixed open and closed."),
        ThruSectionErrorStatus.ThruSectionErrorStatus_ProfilesInconsistent => Error("construction.loft-sections-inconsistent", "The sections are inconsistent: a vertex is given where a wire is needed, or the reverse."),
        ThruSectionErrorStatus.ThruSectionErrorStatus_WrongUsage => Error("construction.loft-wrong-usage", "A vertex section is allowed only first or last."),
        ThruSectionErrorStatus.ThruSectionErrorStatus_Null3DCurve => Error("construction.loft-null-curve", "An edge of a section has no 3D curve."),
        ThruSectionErrorStatus.ThruSectionErrorStatus_Failed => Error("construction.loft-failed", "The loft's surfaces couldn't be built."),
        _ => Unknown(status),
    };

    /// <summary>The error a fillet or chamfer's status reports, or null if it built them.</summary>
    public static Diagnostic? ToDiagnostic(ErrorStatus status) => status switch
    {
        ErrorStatus.Ok => null,
        ErrorStatus.Error => Error("fillet.failed", "The fillet couldn't be built."),
        ErrorStatus.WalkingFailure => Error("fillet.walking-failed", "The fillet's surface couldn't be followed along the edges."),
        ErrorStatus.StartsolFailure => Error("fillet.no-starting-solution", "No starting point was found for the fillet's surface."),
        ErrorStatus.TwistedSurface => Error("fillet.twisted-surface", "The fillet's surface would be twisted."),
        ErrorStatus.EnvelopeCorner => Error("fillet.radius-changes-at-a-corner", "The fillet's radius is changing where its edges' curvature jumps, or its rate of change jumps: the rolling ball's surface has a corner there that can't be built."),
        _ => Unknown(status),
    };

    /// <summary>The error a 3D offset's status reports, or null if it built the offset.</summary>
    public static Diagnostic? ToDiagnostic(Offsets.Error status) => status switch
    {
        Offsets.Error.NoError => null,
        Offsets.Error.UnknownError => Error("offset.failed", "The offset couldn't be built."),
        Offsets.Error.BadNormalsOnGeometry => Error("offset.bad-normals", "A face's normal is undefined somewhere."),
        Offsets.Error.C0Geometry => Error("offset.c0-geometry", "A face's surface is only C0 continuous, so it can't be offset."),
        Offsets.Error.NullOffset => Error("offset.null-offset", "The offset distance is zero."),
        Offsets.Error.NotConnectedShell => Error("offset.not-connected-shell", "The shell's faces aren't connected."),
        Offsets.Error.CannotTrimEdges => Error("offset.cannot-trim-edges", "The offset's edges couldn't be trimmed."),
        Offsets.Error.CannotFuseVertices => Error("offset.cannot-fuse-vertices", "The offset's vertices couldn't be fused."),
        Offsets.Error.CannotExtentEdge => Error("offset.cannot-extend-edge", "An offset edge couldn't be extended."),
        Offsets.Error.UserBreak => Error("offset.user-break", "The offset was stopped."),
        Offsets.Error.MixedConnectivity => Error("offset.mixed-connectivity", "Faces meet along an edge partly tangent and partly not."),
        _ => Unknown(status),
    };

    /// <summary>The error a simple offset's status reports, or null if it built the offset.</summary>
    public static Diagnostic? ToDiagnostic(SimpleOffsetStatus status) => status switch
    {
        SimpleOffsetStatus.OK => null,
        SimpleOffsetStatus.NullInputShape => Error("offset.null-input", "The shape to offset is null."),
        SimpleOffsetStatus.ErrorOffsetComputation => Error("offset.computation-failed", "The offset surfaces couldn't be computed."),
        SimpleOffsetStatus.ErrorWallFaceComputation => Error("offset.wall-face-failed", "The solid's wall faces couldn't be built."),
        SimpleOffsetStatus.ErrorInvalidNbShells => Error("offset.invalid-shell-count", "The offset doesn't have one shell."),
        SimpleOffsetStatus.ErrorNonClosedShell => Error("offset.non-closed-shell", "The offset's shell isn't closed."),
        _ => Unknown(status),
    };

    /// <summary>The error a feature's status reports (BRepFeat's own description as the message), or null if it is OK.</summary>
    public static Diagnostic? ToDiagnostic(StatusError status) =>
        status == StatusError.OK
            ? null
            : Enum.IsDefined(status)
                ? Error("feature." + FailureDiagnostics.Kebab(status.ToString()), BRepFeat.Description(status) + ".")
                : Unknown(status);

    private static Diagnostic Error(string code, string message) => Diagnostic.Error(new DiagnosticCode(code), message);

    // A value outside the enum: a fault, not a status OCCT defines
    private static Diagnostic Unknown<TEnum>(TEnum status) where TEnum : struct, Enum =>
        Diagnostic.Error(FailureDiagnostics.Fault, $"An unknown {typeof(TEnum).Name} status ({status}).");
}
