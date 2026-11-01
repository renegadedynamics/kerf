// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_CurveOnVertex.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_CurveOnVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// The degenerate curve of the rolling ball's centre on a vertex: the vertex's point over the range of the edge EonF.
/// Only the parameters, D0 (<see cref="EvalD0"/>) and the type are defined; the other members throw
/// <see cref="Standard_NotImplemented"/> (the base class's), and <see cref="Circle"/> Standard_NoSuchObject, as in OCCT.
/// </summary>
public sealed class BiTgte_CurveOnVertex : Adaptor3d_Curve
{
    private double myFirst;
    private double myLast;
    private gp_Pnt myPnt = new();

    public BiTgte_CurveOnVertex()
    {
        myFirst = 0.0;
        myLast = 0.0;
    }

    public BiTgte_CurveOnVertex(in TopoDS_Edge theEonF, in TopoDS_Vertex theVertex)
    {
        myFirst = 0.0;
        myLast = 0.0;
        Init(theEonF, theVertex);
    }

    public void Init(in TopoDS_Edge EonF, in TopoDS_Vertex V)
    {
        BRep_Tool.Range(EonF, out myFirst, out myLast);
        myPnt = BRep_Tool.Pnt(V);
    }

    public override double FirstParameter() => myFirst;

    public override double LastParameter() => myLast;

    public override gp_Pnt EvalD0(double theU) => myPnt;

    public override GeomAbs_CurveType GetType() => GeomAbs_OtherCurve;

    /// <exception cref="Standard_NoSuchObject">Always.</exception>
    public override gp_Circ Circle() => throw new Standard_NoSuchObject("BiTgte_CurveOnVertex::Circle");
}
