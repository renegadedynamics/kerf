// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Plane.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Plane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>Projects elementary curves on a plane.</summary>
public class ProjLib_Plane : ProjLib_Projector
{
    private Plane3D myPlane = new();

    /// <summary>Undefined projection.</summary>
    public ProjLib_Plane()
    {
    }

    /// <summary>Projection on the plane Pl.</summary>
    public ProjLib_Plane(in Plane3D Pl)
    {
        Init(Pl);
    }

    /// <summary>Projection of the line L on the plane Pl.</summary>
    public ProjLib_Plane(in Plane3D Pl, in Line3D L)
    {
        Init(Pl);
        Project(L);
    }

    /// <summary>Projection of the circle C on the plane Pl.</summary>
    public ProjLib_Plane(in Plane3D Pl, in Circle3D C)
    {
        Init(Pl);
        Project(C);
    }

    /// <summary>Projection of the ellipse E on the plane Pl.</summary>
    public ProjLib_Plane(in Plane3D Pl, in Ellipse3D E)
    {
        Init(Pl);
        Project(E);
    }

    /// <summary>Projection of the parabola P on the plane Pl.</summary>
    public ProjLib_Plane(in Plane3D Pl, in Parabola3D P)
    {
        Init(Pl);
        Project(P);
    }

    /// <summary>Projection of the hyperbola H on the plane Pl.</summary>
    public ProjLib_Plane(in Plane3D Pl, in Hyperbola3D H)
    {
        Init(Pl);
        Project(H);
    }

    public void Init(in Plane3D Pl)
    {
        myType = OtherCurve;
        isDone = false;
        myIsPeriodic = false;
        myPlane = Pl;
    }

    private static Point2D EvalPnt2d(in Point3D P, in Plane3D Pl)
    {
        Vector3D OP = new(Pl.Location(), P);
        return new Point2D(OP.Dot(new Vector3D(Pl.Position().XDirection())),
                            OP.Dot(new Vector3D(Pl.Position().YDirection())));
    }

    private static Direction2D EvalDir2d(in Direction3D D, in Plane3D Pl)
    {
        return new Direction2D(D.Dot(Pl.Position().XDirection()), D.Dot(Pl.Position().YDirection()));
    }

    public override void Project(in Line3D L)
    {
        myType = global::Kerf.Geometry.CurveKind.Line;
        myLin = new Line2D(EvalPnt2d(L.Location(), myPlane), EvalDir2d(L.Direction(), myPlane));
        isDone = true;
    }

    public override void Project(in Circle3D C)
    {
        myType = global::Kerf.Geometry.CurveKind.Circle;

        Point2D P2d = EvalPnt2d(C.Location(), myPlane);
        Direction2D X2d = EvalDir2d(C.Position().XDirection(), myPlane);
        Direction2D Y2d = EvalDir2d(C.Position().YDirection(), myPlane);
        Frame2D Ax = new(P2d, X2d, Y2d);

        myCirc = new Circle2D(Ax, C.Radius());
        myIsPeriodic = true;
        isDone = true;
    }

    public override void Project(in Ellipse3D E)
    {
        myType = global::Kerf.Geometry.CurveKind.Ellipse;

        Point2D P2d = EvalPnt2d(E.Location(), myPlane);
        Direction2D X2d = EvalDir2d(E.Position().XDirection(), myPlane);
        Direction2D Y2d = EvalDir2d(E.Position().YDirection(), myPlane);
        Frame2D Ax = new(P2d, X2d, Y2d);

        myElips = new Ellipse2D(Ax, E.MajorRadius(), E.MinorRadius());
        myIsPeriodic = true;
        isDone = true;
    }

    public override void Project(in Parabola3D P)
    {
        myType = global::Kerf.Geometry.CurveKind.Parabola;

        Point2D P2d = EvalPnt2d(P.Location(), myPlane);
        Direction2D X2d = EvalDir2d(P.Position().XDirection(), myPlane);
        Direction2D Y2d = EvalDir2d(P.Position().YDirection(), myPlane);
        Frame2D Ax = new(P2d, X2d, Y2d);

        myParab = new Parabola2D(Ax, P.Focal());
        isDone = true;
    }

    public override void Project(in Hyperbola3D H)
    {
        myType = global::Kerf.Geometry.CurveKind.Hyperbola;

        Point2D P2d = EvalPnt2d(H.Location(), myPlane);
        Direction2D X2d = EvalDir2d(H.Position().XDirection(), myPlane);
        Direction2D Y2d = EvalDir2d(H.Position().YDirection(), myPlane);
        Frame2D Ax = new(P2d, X2d, Y2d);

        myHypr = new Hyperbola2D(Ax, H.MajorRadius(), H.MinorRadius());
        isDone = true;
    }
}
