// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_HVertex.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_HVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A vertex of the parametric domain of a surface, as Adaptor3d_TopolTool sees it: a 2D point, an orientation
/// and a tolerance. BRepTopAdaptor_HVertex redefines it for topological vertices.
/// </summary>
public class Adaptor3d_HVertex : Standard_Transient
{
    private gp_Pnt2d myPnt;
    private double myTol;
    private TopAbs_Orientation myOri;

    /// <summary>A vertex at the origin with a zero tolerance.</summary>
    /// <remarks>OCCT leaves the orientation uninitialised; here it is FORWARD.</remarks>
    public Adaptor3d_HVertex()
    {
        // OCCT: myOri is left uninitialised (docs/occt-issues.md).
        myTol = 0.0;
    }

    public Adaptor3d_HVertex(in gp_Pnt2d P, TopAbs_Orientation Or, double Resolution)
    {
        myPnt = P;
        myTol = Resolution;
        myOri = Or;
    }

    public virtual gp_Pnt2d Value() => myPnt;

    /// <summary>The parameter of the vertex on C, which must be a line.</summary>
    public virtual double Parameter(Adaptor2d_Curve2d C) => ElCLib.Parameter(C.Line(), myPnt);

    /// <summary>The parametric (2D) resolution.</summary>
    public virtual double Resolution(Adaptor2d_Curve2d C) => myTol;

    public virtual TopAbs_Orientation Orientation() => myOri;

    /// <summary>True if the two vertices are within <c>Precision::Confusion()</c>.</summary>
    public virtual bool IsSame(Adaptor3d_HVertex Other) => (myPnt.Distance(Other.Value()) <= Precision.Confusion());
}
