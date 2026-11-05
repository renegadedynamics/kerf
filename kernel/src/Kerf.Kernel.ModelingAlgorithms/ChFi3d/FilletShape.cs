// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_FilletShape.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets;
using Kerf.Engine;

namespace Kerf.Fillets;

/// <summary>The shape of a fillet's sections: rational (exact circular arcs), quasi-angular or polynomial.</summary>
public enum FilletShape
{
    Rational,
    QuasiAngular,
    Polynomial,
}
