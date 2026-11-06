// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_AbscissaType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Measurement;

/// <summary>How a curve is parameterised, for the abscissa algorithms.</summary>
internal enum AbscissaType
{
    LengthParametrized,
    Parametrized,
    AbsComposite,
}
