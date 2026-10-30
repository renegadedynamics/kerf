// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_TrihedronWithGuide.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_TrihedronWithGuide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Defines a trihedron along a curve with a guide curve.</summary>
public abstract class GeomFill_TrihedronWithGuide : GeomFill_TrihedronLaw
{
    protected Adaptor3d_Curve? myGuide;
    protected Adaptor3d_Curve? myTrimG;
    protected gp_Pnt myCurPointOnGuide;

    public abstract Adaptor3d_Curve Guide();

    public abstract void Origine(double Param1, double Param2);

    /// <summary>The current point on the guide found by D0, D1 or D2.</summary>
    public gp_Pnt CurrentPointOnGuide() => myCurPointOnGuide;
}
