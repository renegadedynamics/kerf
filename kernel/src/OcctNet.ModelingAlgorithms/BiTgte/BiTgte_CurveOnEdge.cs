// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_CurveOnEdge.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_CurveOnEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// The curve of the rolling ball's centre on an edge: a point of the edge EonF of the offset face projected on the
/// original edge. Only the parameters, D0 (<see cref="EvalD0"/>), the type and a zero-radius circle are defined; the
/// other members throw <see cref="MethodNotImplementedException"/>, as in OCCT (the base class's).
/// </summary>
public sealed class BiTgte_CurveOnEdge : Adaptor3d_Curve
{
    private TopoDS_Edge myEdge = new();
    private TopoDS_Edge myEonF = new();
    private Geom_Curve? myCurv;
    private Geom_Curve? myConF;
    private CurveKind myType;
    private Circle3D myCirc = new();

    public BiTgte_CurveOnEdge()
    {
        myType = OtherCurve;
    }

    public BiTgte_CurveOnEdge(in TopoDS_Edge theEonF, in TopoDS_Edge theEdge)
    {
        myEdge = theEdge;
        myEonF = theEonF;
        myType = OtherCurve;
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

        myType = OtherCurve;
        if (Curv.GetType() == global::Kerf.Geometry.CurveKind.Line && ConF.GetType() == global::Kerf.Geometry.CurveKind.Circle)
        {
            Axis3D a1 = Curv.Line().Position();
            Axis3D a2 = ConF.Circle().Axis();
            if (a1.IsCoaxial(a2, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                myType = global::Kerf.Geometry.CurveKind.Circle;
                myCirc = new Circle3D(ConF.Circle().Position(), 0.0);
            }
        }
    }

    public override double FirstParameter() => myConF!.FirstParameter();

    public override double LastParameter() => myConF!.LastParameter();

    public override Point3D EvalD0(double theU)
    {
        GeomAPI_ProjectPointOnCurve aProjector = new();
        Point3D aP = myConF!.Value(theU);
        aProjector.Init(aP, myCurv!);
        return aProjector.NearestPoint();
    }

    public override CurveKind GetType() => myType;

    /// <exception cref="NoSuchObjectException">The curve isn't a circle.</exception>
    public override Circle3D Circle()
    {
        if (myType != global::Kerf.Geometry.CurveKind.Circle)
        {
            throw new NoSuchObjectException("BiTgte_CurveOnEdge::Circle");
        }

        return myCirc;
    }
}
