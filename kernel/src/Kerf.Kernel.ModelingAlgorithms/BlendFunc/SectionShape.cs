// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_SectionShape.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>The shape of a blend's sections.</summary>
internal enum SectionShape
{
    Rational,
    QuasiAngular,
    Polynomial,
    Linear,
}
