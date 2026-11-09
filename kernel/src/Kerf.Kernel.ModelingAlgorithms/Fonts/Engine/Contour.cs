// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Fonts;

namespace Kerf.Engine.Fonts;

/// <summary>
/// A contour as a decomposition gives it: its segments, and whether the last is the line its close drew back to the
/// start, which a single-stroke font (whose strokes are open) doesn't draw, as OCCT's <c>StdPrs_BRepFont</c> leaves it.
/// </summary>
internal readonly record struct Contour(ImmutableArray<OutlineSegment> Segments, bool ClosedByLine);
