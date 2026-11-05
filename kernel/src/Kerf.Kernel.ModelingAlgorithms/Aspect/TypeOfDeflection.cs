// Port of: src/Visualization/TKService/Aspect/Aspect_TypeOfDeflection.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.HiddenLines.Presentation;

/// <summary>
/// Defines if the maximal chordial deflection used when drawing an object is absolute or relative to the size of the
/// object.
/// </summary>
internal enum TypeOfDeflection
{
    TodRelative,
    TodAbsolute,
}
