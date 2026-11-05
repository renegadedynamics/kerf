// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// The Booleans and clean-ups build123d.NET's own tests make (unit 6M, the phase 6 milestone), rebuilt with build123d.NET's
/// public API as open question 20's provisional answer has it (decision 947): each scenario transcribes one test's
/// construction, and the capture records every <c>BRepAlgoAPI_*</c> call build123d.NET's Boolean path makes on the way
/// (its arguments and tools as the native shapes it hands OCCT, its options) and every <c>ShapeUpgrade_UnifySameDomain</c>
/// it runs (<c>Clean()</c>, and the clean after each Boolean). The scenario goes on with build123d.NET's own result.
/// </summary>
/// <remarks>
/// The capture mirrors build123d.NET's dispatch (Shape.Booleans.cs, Compound.cs, Shape2D.cs, Shape3D.cs, Shape1D.cs):
/// <list type="bullet">
/// <item><c>a - b</c> (Subtract): a cut of <c>[a]</c> by the top-level shapes of <c>b</c>;</item>
/// <item><c>a + b</c> (Add): for a Part, Sketch or plain compound, a fuse of its first top-level shape with the rest of its
/// own and <c>b</c>'s, fuzzy at <c>Constants.Tolerance</c>; for a solid or face, a fuse of <c>[a]</c> with <c>b</c>'s
/// top-level shapes; joining 1D shapes (wires) and sewing shells make no Boolean;</item>
/// <item><c>a &amp; b</c> and <c>Intersect</c>: per element, a common for a solid, a common and a section for two faces or
/// shells, a section for a face and an edge, a common and a section for two edges; then a clean of the result;</item>
/// <item><c>Fuse</c> and <c>Cut</c>: <c>[a]</c> against the shapes given, with glue (shift) and the fuzzy value;</item>
/// <item><c>Split</c>: a splitter of <c>[a]</c> by the plane's face or the tool, not cleaned.</item>
/// </list>
/// Every Boolean runs in parallel and non-destructively, as build123d.NET's does. Operations that clean or fuse inside
/// (Extrude both ways, Revolve, Loft, Section, Thicken, Trace) are transcribed from their source with their public building
/// blocks (<c>Solid.Extrude</c>, <c>clean: false</c>...). The touches <c>includeTouched</c> adds are left out: they repeat
/// the same Booleans.
/// </remarks>
internal sealed partial class Build123dBooleanCapture
{
    internal enum Kind { Fuse, Cut, Common, Section, Split, Clean }

    /// <summary>
    /// One native call: a Boolean (its arguments, tools and options, and whether build123d.NET cleans its result) or a
    /// clean (its shape in <see cref="Arguments"/>). <see cref="Result"/> is build123d.NET's own result of an operator or
    /// method that makes just this Boolean (and its clean), which checks the transcription.
    /// </summary>
    internal sealed record Call(string Name, Kind Kind, N.TopoDS_Shape[] Arguments, N.TopoDS_Shape[] Tools, double Fuzzy, bool Glue,
                                bool Clean, N.TopoDS_Shape? Result = null);

    internal readonly List<Call> Calls = [];

    /// <summary>
    /// The calls, as recorded (unit 10A) by running build123d.NET's scenarios on native OCCT, which went in unit 10B: the
    /// calls' shapes are native's, through the recorded facade, and the scenarios' failures are kept with them.
    /// </summary>
    internal static List<Call> Recorded(List<string> theFailures)
    {
        (List<Call> aCalls, List<string> aFailures) = Tape.Value("build123d.NET's Booleans", static r =>
        {
            List<string> aFailed = [.. ((object?[])r.Value()!).Cast<string>()];
            var aCalls = new List<Call>();
            for (int i = r.Int(); i > 0; i--)
            {
                aCalls.Add(new Call(r.String()!, (Kind)r.Int(), Tape.Array<N.TopoDS_Shape>(r.Value()), Tape.Array<N.TopoDS_Shape>(r.Value()),
                                    (double)r.Value()!, (bool)r.Value()!, (bool)r.Value()!, r.Object<N.TopoDS_Shape>()));
            }
            return (aCalls, aFailed);
        });
        theFailures.AddRange(aFailures);
        return aCalls;
    }
}
