// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_CurveOnEdge.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_CurveOnEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// The curve of the rolling ball's centre on an edge: a point of the edge EonF of the offset face projected on the
/// original edge. Only the parameters, D0 (<see cref="EvalD0"/>), the type and a zero-radius circle are defined; the
/// other members throw <see cref="Standard_NotImplemented"/>, as in OCCT (the base class's).
/// </summary>
public sealed class BiTgte_CurveOnEdge : Adaptor3d_Curve
{
    private TopoDS_Edge myEdge = new();
    private TopoDS_Edge myEonF = new();
    private Geom_Curve? myCurv;
    private Geom_Curve? myConF;
    private GeomAbs_CurveType myType;
    private gp_Circ myCirc = new();

    public BiTgte_CurveOnEdge()
    {
        myType = GeomAbs_OtherCurve;
    }

    public BiTgte_CurveOnEdge(in TopoDS_Edge theEonF, in TopoDS_Edge theEdge)
    {
        myEdge = theEdge;
        myEonF = theEonF;
        myType = GeomAbs_OtherCurve;
        Init(theEonF, theEdge);
    }

    public override Adaptor3d_Curve ShallowCopy()
    {
        BiTgte_CurveOnEdge aCopy = new()
        {
            myEdge = myEdge,
            myEonF = myEonF,
            myCurv = myCurv,
            myConF = myConF,
            myType = myType,
            myCirc = myCirc
        };

        return aCopy;
    }

    public void Init(in TopoDS_Edge EonF, in TopoDS_Edge Edge)
    {
        double f, l;

        myEdge = Edge;
        myCurv = BRep_Tool.Curve(myEdge, out f, out l);
        myCurv = new Geom_TrimmedCurve(myCurv!, f, l);

        myEonF = EonF;
        myConF = BRep_Tool.Curve(myEonF, out f, out l);
        myConF = new Geom_TrimmedCurve(myConF!, f, l);

        // peut on generer un cercle de rayon nul
        GeomAdaptor_Curve Curv = new(myCurv);
        GeomAdaptor_Curve ConF = new(myConF);

        myType = GeomAbs_OtherCurve;
        if (Curv.GetType() == GeomAbs_Line && ConF.GetType() == GeomAbs_Circle)
        {
            gp_Ax1 a1 = Curv.Line().Position();
            gp_Ax1 a2 = ConF.Circle().Axis();
            if (a1.IsCoaxial(a2, Precision.Angular(), Precision.Confusion()))
            {
                myType = GeomAbs_Circle;
                myCirc = new gp_Circ(ConF.Circle().Position(), 0.0);
            }
        }
    }

    public override double FirstParameter() => myConF!.FirstParameter();

    public override double LastParameter() => myConF!.LastParameter();

    public override gp_Pnt EvalD0(double theU)
    {
        GeomAPI_ProjectPointOnCurve aProjector = new();
        gp_Pnt aP = myConF!.Value(theU);
        aProjector.Init(aP, myCurv!);
        return aProjector.NearestPoint();
    }

    public override GeomAbs_CurveType GetType() => myType;

    /// <exception cref="Standard_NoSuchObject">The curve isn't a circle.</exception>
    public override gp_Circ Circle()
    {
        if (myType != GeomAbs_Circle)
        {
            throw new Standard_NoSuchObject("BiTgte_CurveOnEdge::Circle");
        }

        return myCirc;
    }
}
