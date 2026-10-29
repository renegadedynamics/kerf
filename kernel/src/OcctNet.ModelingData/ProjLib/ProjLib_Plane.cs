// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Plane.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Plane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>Projects elementary curves on a plane.</summary>
public class ProjLib_Plane : ProjLib_Projector
{
    private gp_Pln myPlane = new();

    /// <summary>Undefined projection.</summary>
    public ProjLib_Plane()
    {
    }

    /// <summary>Projection on the plane Pl.</summary>
    public ProjLib_Plane(in gp_Pln Pl)
    {
        Init(Pl);
    }

    /// <summary>Projection of the line L on the plane Pl.</summary>
    public ProjLib_Plane(in gp_Pln Pl, in gp_Lin L)
    {
        Init(Pl);
        Project(L);
    }

    /// <summary>Projection of the circle C on the plane Pl.</summary>
    public ProjLib_Plane(in gp_Pln Pl, in gp_Circ C)
    {
        Init(Pl);
        Project(C);
    }

    /// <summary>Projection of the ellipse E on the plane Pl.</summary>
    public ProjLib_Plane(in gp_Pln Pl, in gp_Elips E)
    {
        Init(Pl);
        Project(E);
    }

    /// <summary>Projection of the parabola P on the plane Pl.</summary>
    public ProjLib_Plane(in gp_Pln Pl, in gp_Parab P)
    {
        Init(Pl);
        Project(P);
    }

    /// <summary>Projection of the hyperbola H on the plane Pl.</summary>
    public ProjLib_Plane(in gp_Pln Pl, in gp_Hypr H)
    {
        Init(Pl);
        Project(H);
    }

    public void Init(in gp_Pln Pl)
    {
        myType = GeomAbs_OtherCurve;
        isDone = false;
        myIsPeriodic = false;
        myPlane = Pl;
    }

    private static gp_Pnt2d EvalPnt2d(in gp_Pnt P, in gp_Pln Pl)
    {
        gp_Vec OP = new(Pl.Location(), P);
        return new gp_Pnt2d(OP.Dot(new gp_Vec(Pl.Position().XDirection())),
                            OP.Dot(new gp_Vec(Pl.Position().YDirection())));
    }

    private static gp_Dir2d EvalDir2d(in gp_Dir D, in gp_Pln Pl)
    {
        return new gp_Dir2d(D.Dot(Pl.Position().XDirection()), D.Dot(Pl.Position().YDirection()));
    }

    public override void Project(in gp_Lin L)
    {
        myType = GeomAbs_Line;
        myLin = new gp_Lin2d(EvalPnt2d(L.Location(), myPlane), EvalDir2d(L.Direction(), myPlane));
        isDone = true;
    }

    public override void Project(in gp_Circ C)
    {
        myType = GeomAbs_Circle;

        gp_Pnt2d P2d = EvalPnt2d(C.Location(), myPlane);
        gp_Dir2d X2d = EvalDir2d(C.Position().XDirection(), myPlane);
        gp_Dir2d Y2d = EvalDir2d(C.Position().YDirection(), myPlane);
        gp_Ax22d Ax = new(P2d, X2d, Y2d);

        myCirc = new gp_Circ2d(Ax, C.Radius());
        myIsPeriodic = true;
        isDone = true;
    }

    public override void Project(in gp_Elips E)
    {
        myType = GeomAbs_Ellipse;

        gp_Pnt2d P2d = EvalPnt2d(E.Location(), myPlane);
        gp_Dir2d X2d = EvalDir2d(E.Position().XDirection(), myPlane);
        gp_Dir2d Y2d = EvalDir2d(E.Position().YDirection(), myPlane);
        gp_Ax22d Ax = new(P2d, X2d, Y2d);

        myElips = new gp_Elips2d(Ax, E.MajorRadius(), E.MinorRadius());
        myIsPeriodic = true;
        isDone = true;
    }

    public override void Project(in gp_Parab P)
    {
        myType = GeomAbs_Parabola;

        gp_Pnt2d P2d = EvalPnt2d(P.Location(), myPlane);
        gp_Dir2d X2d = EvalDir2d(P.Position().XDirection(), myPlane);
        gp_Dir2d Y2d = EvalDir2d(P.Position().YDirection(), myPlane);
        gp_Ax22d Ax = new(P2d, X2d, Y2d);

        myParab = new gp_Parab2d(Ax, P.Focal());
        isDone = true;
    }

    public override void Project(in gp_Hypr H)
    {
        myType = GeomAbs_Hyperbola;

        gp_Pnt2d P2d = EvalPnt2d(H.Location(), myPlane);
        gp_Dir2d X2d = EvalDir2d(H.Position().XDirection(), myPlane);
        gp_Dir2d Y2d = EvalDir2d(H.Position().YDirection(), myPlane);
        gp_Ax22d Ax = new(P2d, X2d, Y2d);

        myHypr = new gp_Hypr2d(Ax, H.MajorRadius(), H.MinorRadius());
        isDone = true;
    }
}
