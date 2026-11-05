// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_ChAsym.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_ChAsymInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_ChamfInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Chamfer.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_ConstRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_ConstRadInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_ConstThroat.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_ConstThroatInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_ConstThroatWithPenetration.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_ConstThroatWithPenetrationInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_CSCircular.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_CSConstRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_EvolRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_EvolRadInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Ruled.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_RuledInv.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// BRepBlend's typedef headers name BlendFunc's functions (BRepBlend_ConstRad is BlendFunc_ConstRad, BRepBlend_CSCircular
// is BlendFunc_CSCircular, ...). C# cannot publish a type alias, so ported code writes the BlendFunc names out
// (decision 1059); their provenance lines are here.

namespace Kerf.Engine;
