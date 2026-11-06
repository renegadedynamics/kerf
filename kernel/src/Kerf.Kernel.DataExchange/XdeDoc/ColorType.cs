// New in Kerf (unit 8T): the immutable assembly tree in place of XCAF (decision 1229). Not a port.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Exchange.Documents;

/// <summary>
/// The kinds of colour a shape carries, as XCAF's <c>XCAFDoc_ColorType</c> (same names and values): a generic colour
/// (for both surfaces and curves), a surface colour and a curve colour.
/// </summary>
internal enum ColorType
{
    /// <summary>The generic colour: the surfaces' and the curves' where they have none of their own.</summary>
    ColorGen = 0,

    /// <summary>The surfaces' colour.</summary>
    ColorSurf = 1,

    /// <summary>The curves' colour.</summary>
    ColorCurv = 2,
}
