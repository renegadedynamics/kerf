// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Blend;
using Kerf.Geometry;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>
/// A blend function of evolving radius, whose sections are the circles of the envelope its ball sweeps (unit 15AK,
/// <see cref="EnvelopeCircle"/>). The circle takes the derivative of the centre's path, which jumps where the law's
/// slope does, or the guide's curvature while the radius is changing: the path has a corner there, the true envelope
/// a piece of the ball between two circles, and the sections on the two sides don't meet.
/// </summary>
internal interface IEnvelopeSections
{
    /// <summary>The intervals on which the sections are of the class asked; at C0, those between the jumps.</summary>
    int NbIntervals(Continuity s);

    /// <summary>The intervals' bounds.</summary>
    void Intervals(NCollection_Array1<double> t, Continuity s);

    /// <summary>
    /// How far apart the middles of the two sections' arcs are at <paramref name="prm"/>, the one made with the guide
    /// and the law as they are from <paramref name="before"/> to prm and the other as from prm to
    /// <paramref name="after"/>, on the walk's solution there (found from the point <paramref name="near"/> it), and
    /// the ball's radius there. False where the solution or a circle can't be had.
    /// </summary>
    bool SectionGap(in BlendPoint near, double prm, double before, double after, out double gap, out double radius);

    /// <summary>
    /// The section at the walk's point <paramref name="at"/>, with the path of the ball's centre there. False where
    /// the circle can't be had.
    /// </summary>
    bool SectionEnd(in BlendPoint at, out EnvelopeEnd end);

    /// <summary>
    /// <see cref="EnvelopeCircle.LeastAdvance"/> of the section at <paramref name="prm"/>, on the walk's solution there
    /// (found from the point <paramref name="near"/> it). False where the solution or the circle can't be had.
    /// </summary>
    bool LeastAdvance(in BlendPoint near, double prm, out double advance);
}
