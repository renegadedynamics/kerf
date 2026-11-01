// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Config.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// How a shape lies with respect to the reference of its same-domain group: not sharing its geometry, or sharing it
/// with the same or the opposite orientation.
/// </summary>
public enum TopOpeBRepDS_Config
{
    TopOpeBRepDS_UNSHGEOMETRY,
    TopOpeBRepDS_SAMEORIENTED,
    TopOpeBRepDS_DIFFORIENTED,
}
