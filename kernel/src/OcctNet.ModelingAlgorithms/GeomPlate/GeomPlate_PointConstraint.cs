// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_PointConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_PointConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A point constraint of a plate surface: a point to pass through (order 0, or -1 for a point that is only
/// approached), or a point on a surface whose tangent plane (order 1) or curvature (order 2) the plate must follow.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's: its setters are public, and GeomPlate_BuildPlateSurface sets the
/// order and the 2D point it finds. It keeps the caller's surface, read only.
/// </remarks>
public class GeomPlate_PointConstraint : Transient
{
    protected int myOrder;
    private readonly GeomLProp_SLProps myLProp;
    private readonly Point3D myPoint;
    private Point2D myPt2d;
    private readonly Geom_Surface? mySurf;
    private readonly Vector3D myD11;
    private readonly Vector3D myD12;
    private readonly Vector3D myD21;
    private readonly Vector3D myD22;
    private readonly Vector3D myD23;
    private readonly double myU;
    private readonly double myV;
    private double myTolDist;
    private double myTolAng;
    private double myTolCurv;
    private bool hasPnt2dOnSurf;

    /// <summary>A point the plate passes through (order 0) or approaches (order -1).</summary>
    public GeomPlate_PointConstraint(in Point3D Pt, int Order, double TolDist = 0.0001)
    {
        myOrder = Order;
        myLProp = new GeomLProp_SLProps(2, TolDist);
        myPoint = Pt;
        myPt2d = new Point2D();
        myU = 0.0;
        myV = 0.0;
        myTolDist = TolDist;
        myTolAng = 0.0;
        myTolCurv = 0.0;
        hasPnt2dOnSurf = false;
        if ((myOrder > 1) || (myOrder < -1))
        {
            throw new FailureException("GeomPlate_PointConstraint : the constraint must 0 or -1 with a point");
        }
    }

    /// <summary>The point of <paramref name="Surf"/> at (U, V), with the surface's derivatives there.</summary>
    public GeomPlate_PointConstraint(double U, double V, Geom_Surface Surf, int Order, double TolDist = 0.0001,
                                     double TolAng = 0.01, double TolCurv = 0.1)
    {
        myOrder = Order;
        myLProp = new GeomLProp_SLProps(2, TolDist);
        mySurf = Surf;
        myPt2d = new Point2D();
        myU = U;
        myV = V;
        myTolDist = TolDist;
        myTolAng = TolAng;
        myTolCurv = TolCurv;
        hasPnt2dOnSurf = false;
        Surf.D2(myU, myV, out myPoint, out myD11, out myD12, out myD21, out myD22, out myD23);
        myLProp.SetSurface(Surf);
    }

    public void D0(out Point3D P)
    {
        P = myPoint;
    }

    public void D1(out Point3D P, out Vector3D V1, out Vector3D V2)
    {
        P = myPoint;
        V1 = myD11;
        V2 = myD12;
    }

    public void D2(out Point3D P, out Vector3D V1, out Vector3D V2, out Vector3D V3, out Vector3D V4, out Vector3D V5)
    {
        P = myPoint;
        V1 = myD11;
        V2 = myD12;
        V3 = myD21;
        V4 = myD22;
        V5 = myD23;
    }

    public void SetG0Criterion(double TolDist)
    {
        myTolDist = TolDist;
    }

    public void SetG1Criterion(double TolAng)
    {
        myTolAng = TolAng;
    }

    public void SetG2Criterion(double TolCurv)
    {
        myTolCurv = TolCurv;
    }

    public double G0Criterion() => myTolDist;

    public double G1Criterion() => myTolAng;

    public double G2Criterion() => myTolCurv;

    /// <summary>The local properties of the surface at the constraint's parameters (the object the constraint keeps).</summary>
    public GeomLProp_SLProps LPropSurf()
    {
        myLProp.SetParameters(myU, myV);
        return myLProp;
    }

    public int Order() => myOrder;

    public void SetOrder(int Order)
    {
        myOrder = Order;
    }

    public bool HasPnt2dOnSurf() => hasPnt2dOnSurf;

    public void SetPnt2dOnSurf(in Point2D Pnt2d)
    {
        myPt2d = Pnt2d;
        hasPnt2dOnSurf = true;
    }

    public Point2D Pnt2dOnSurf() => myPt2d;
}
