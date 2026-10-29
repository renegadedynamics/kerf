// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_RealFunction.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A function for <see cref="CPnts_MyGaussFunction"/>: its value at X for the client data D. OCCT's
/// <c>double (*)(const double, void* const)</c>; the client data is an object.
/// </summary>
public delegate double CPnts_RealFunction(double X, object? D);
