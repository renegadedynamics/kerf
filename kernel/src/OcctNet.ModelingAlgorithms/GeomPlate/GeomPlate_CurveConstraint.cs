// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_CurveConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_CurveConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A curve constraint of a plate surface: a 3D curve the plate passes through (order 0, or -1 for a curve only
/// approached), or a curve on a surface (an <see cref="Adaptor3d_CurveOnSurface"/> over a
/// <see cref="GeomAdaptor_Surface"/>) whose tangent plane (order 1) or curvature (order 2) the plate must follow.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's: its setters are public, and GeomPlate_BuildPlateSurface sets the
/// projected curve, the 2D curve and the order. It keeps the caller's adaptors and laws, read only.
/// </remarks>
public class GeomPlate_CurveConstraint : Standard_Transient
{
    protected Adaptor3d_CurveOnSurface? myFrontiere;
    protected int myNbPoints;
    protected int myOrder;
    protected Adaptor3d_Curve? my3dCurve;
    protected int myTang;
    protected Geom2d_Curve? my2dCurve;
    protected Adaptor2d_Curve2d? myHCurve2d;
    protected Law_Function? myG0Crit;
    protected Law_Function? myG1Crit;
    protected Law_Function? myG2Crit;
    protected bool myConstG0;
    protected bool myConstG1;
    protected bool myConstG2;
    protected GeomLProp_SLProps myLProp;
    protected double myTolDist;
    protected double myTolAng;
    protected double myTolCurv;
    protected double myTolU;
    protected double myTolV;

    public GeomPlate_CurveConstraint()
    {
        myNbPoints = 0;
        myOrder = 0;
        myTang = 0;
        myConstG0 = false;
        myConstG1 = false;
        myConstG2 = false;
        myLProp = new GeomLProp_SLProps(2, 1.0e-4);
        myTolDist = 0.0;
        myTolAng = 0.0;
        myTolCurv = 0.0;
        myTolU = 0.0;
        myTolV = 0.0;
    }

    /// <summary>
    /// A constraint on <paramref name="Boundary"/> of order <paramref name="Tang"/> (-1 to 2), checked at
    /// <paramref name="NPt"/> points.
    /// </summary>
    /// <exception cref="Standard_Failure">The order isn't -1 to 2, or a curve on a surface isn't on a
    /// GeomAdaptor_Surface.</exception>
    public GeomPlate_CurveConstraint(Adaptor3d_Curve Boundary, int Tang, int NPt = 10, double TolDist = 0.0001,
                                     double TolAng = 0.01, double TolCurv = 0.1)
    {
        myLProp = new GeomLProp_SLProps(2, TolDist);
        myTolDist = TolDist;
        myTolAng = TolAng;
        myTolCurv = TolCurv;
        myOrder = Tang;
        if ((Tang < -1) || (Tang > 2))
        {
            throw new Standard_Failure("GeomPlate : The continuity is not G0 G1 or G2");
        }
        myNbPoints = NPt;
        myConstG0 = true;
        myConstG1 = true;
        myConstG2 = true;

        myFrontiere = Boundary as Adaptor3d_CurveOnSurface;

        if (myFrontiere is null)
        {
            my3dCurve = Boundary;
        }
        else
        {
            Geom_Surface? Surf;
            GeomAdaptor_Surface? GS1 = myFrontiere.GetSurface() as GeomAdaptor_Surface;

            if (GS1 is not null)
            {
                Surf = GS1.Surface();
            }
            else
            {
                throw new Standard_Failure("GeomPlate_CurveConstraint : Surface must be GeomAdaptor_Surface");
            }

            myLProp.SetSurface(Surf!);
        }

        my2dCurve = null;
        myHCurve2d = null;
        myTolU = 0.0;
        myTolV = 0.0;
        myG0Crit = null;
        myG1Crit = null;
        myG2Crit = null;
    }

    public double FirstParameter()
    {
        if (myHCurve2d is not null)
        {
            return myHCurve2d.FirstParameter();
        }
        else if (my3dCurve is null)
        {
            return myFrontiere!.FirstParameter();
        }
        else
        {
            return my3dCurve.FirstParameter();
        }
    }

    public double LastParameter()
    {
        if (myHCurve2d is not null)
        {
            return myHCurve2d.LastParameter();
        }
        else if (my3dCurve is null)
        {
            return myFrontiere!.LastParameter();
        }
        else
        {
            return my3dCurve.LastParameter();
        }
    }

    public double Length()
    {
        if (my3dCurve is null)
        {
            return GCPnts_AbscissaPoint.Length(myFrontiere!);
        }
        else
        {
            return GCPnts_AbscissaPoint.Length(my3dCurve);
        }
    }

    public void D0(double U, out gp_Pnt P)
    {
        gp_Pnt2d P2d;

        if (my3dCurve is null)
        {
            P2d = myFrontiere!.GetCurve()!.Value(U);
            myFrontiere.GetSurface()!.D0(P2d.Coord(1), P2d.Coord(2), out P);
        }
        else
        {
            my3dCurve.D0(U, out P);
        }
    }

    /// <exception cref="Standard_Failure">The constraint isn't on a surface.</exception>
    public void D1(double U, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2)
    {
        gp_Pnt2d P2d;
        if (my3dCurve is not null)
        {
            throw new Standard_Failure("GeomPlate_CurveConstraint.cxx : Curve must be on a Surface");
        }

        P2d = myFrontiere!.GetCurve()!.Value(U);
        myFrontiere.GetSurface()!.D1(P2d.Coord(1), P2d.Coord(2), out P, out V1, out V2);
    }

    /// <exception cref="Standard_Failure">The constraint isn't on a surface.</exception>
    public void D2(double U, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2, out gp_Vec V3, out gp_Vec V4, out gp_Vec V5)
    {
        gp_Pnt2d P2d;
        if (my3dCurve is not null)
        {
            throw new Standard_Failure("GeomPlate_CurveConstraint.cxx : Curve must be on a Surface");
        }

        P2d = myFrontiere!.GetCurve()!.Value(U);
        myFrontiere.GetSurface()!.D2(P2d.Coord(1), P2d.Coord(2), out P, out V1, out V2, out V3, out V4, out V5);
    }

    public void SetG0Criterion(Law_Function G0Crit)
    {
        myG0Crit = G0Crit;
        myConstG0 = false;
    }

    public void SetG1Criterion(Law_Function G1Crit)
    {
        if (my3dCurve is not null)
        {
            throw new Standard_Failure("GeomPlate_CurveConstraint.cxx : Curve must be on a Surface");
        }
        myG1Crit = G1Crit;
        myConstG1 = false;
    }

    public void SetG2Criterion(Law_Function G2Crit)
    {
        if (my3dCurve is not null)
        {
            throw new Standard_Failure("GeomPlate_CurveConstraint.cxx : Curve must be on a Surface");
        }
        myG2Crit = G2Crit;
        myConstG2 = false;
    }

    public double G0Criterion(double U)
    {
        if (myConstG0)
        {
            return myTolDist;
        }
        else
        {
            return myG0Crit!.Value(U);
        }
    }

    public double G1Criterion(double U)
    {
        if (my3dCurve is not null)
        {
            throw new Standard_Failure("GeomPlate_CurveConstraint.cxx : Curve must be on a Surface");
        }
        if (myConstG1)
        {
            return myTolAng;
        }
        else
        {
            return myG1Crit!.Value(U);
        }
    }

    public double G2Criterion(double U)
    {
        if (my3dCurve is not null)
        {
            throw new Standard_Failure("GeomPlate_CurveConstraint.cxx : Curve must be on a Surface");
        }
        if (myConstG2)
        {
            return myTolCurv;
        }
        else
        {
            return myG2Crit!.Value(U);
        }
    }

    /// <summary>
    /// The 2D curve on the surface: the one set, else an approximation of the projected curve (a new curve on each
    /// call), else null.
    /// </summary>
    public Geom2d_Curve? Curve2dOnSurf()
    {
        if (my2dCurve is null && myHCurve2d is not null)
        {
            Geom2d_Curve? C2d;
            GeomAbs_Shape Continuity = GeomAbs_Shape.GeomAbs_C1;
            int MaxDegree = 10;
            int MaxSeg = 20 + myHCurve2d.NbIntervals(GeomAbs_Shape.GeomAbs_C3);
            Approx_Curve2d appr = new Approx_Curve2d(myHCurve2d, myHCurve2d.FirstParameter(), myHCurve2d.LastParameter(),
                                                     myTolU, myTolV, Continuity, MaxDegree, MaxSeg);
            C2d = appr.Curve();
            return C2d;
        }
        else
        {
            return my2dCurve;
        }
    }

    public void SetCurve2dOnSurf(Geom2d_Curve? Curve)
    {
        my2dCurve = Curve;
    }

    public Adaptor2d_Curve2d? ProjectedCurve() => myHCurve2d;

    public void SetProjectedCurve(Adaptor2d_Curve2d? Curve, double TolU, double TolV)
    {
        myHCurve2d = Curve;
        myTolU = TolU;
        myTolV = TolV;
    }

    public Adaptor3d_Curve Curve3d()
    {
        if (my3dCurve is null)
        {
            return myFrontiere!;
        }
        else
        {
            return my3dCurve;
        }
    }

    public int NbPoints() => myNbPoints;

    public int Order() => myOrder;

    public void SetNbPoints(int NewNb)
    {
        myNbPoints = NewNb;
    }

    public void SetOrder(int Order)
    {
        myOrder = Order;
    }

    /// <summary>The local properties of the support surface at the constraint's parameter U.</summary>
    /// <exception cref="Standard_Failure">The constraint isn't on a surface.</exception>
    public GeomLProp_SLProps LPropSurf(double U)
    {
        if (myFrontiere is null)
        {
            throw new Standard_Failure("GeomPlate_CurveConstraint.cxx : Curve must be on a Surface");
        }
        gp_Pnt2d P2d = myFrontiere.GetCurve()!.Value(U);
        myLProp.SetParameters(P2d.X(), P2d.Y());
        return myLProp;
    }
}
