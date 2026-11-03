// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Cylinder.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Cylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>Projects elementary curves on a cylinder: lines parallel to the axis and circles normal to it.</summary>
public class ProjLib_Cylinder : ProjLib_Projector
{
    private Cylinder myCylinder = new();

    /// <summary>Undefined projection.</summary>
    public ProjLib_Cylinder()
    {
    }

    /// <summary>Projection on the cylinder Cyl.</summary>
    public ProjLib_Cylinder(in Cylinder Cyl)
    {
        Init(Cyl);
    }

    /// <summary>Projection of the line L on the cylinder Cyl.</summary>
    public ProjLib_Cylinder(in Cylinder Cyl, in Line3D L)
    {
        Init(Cyl);
        Project(L);
    }

    /// <summary>Projection of the circle C on the cylinder Cyl.</summary>
    public ProjLib_Cylinder(in Cylinder Cyl, in Circle3D C)
    {
        Init(Cyl);
        Project(C);
    }

    /// <summary>Projection of the ellipse E on the cylinder Cyl (left to approximation: never done).</summary>
    public ProjLib_Cylinder(in Cylinder Cyl, in Ellipse3D E)
    {
        Init(Cyl);
        Project(E);
    }

    public void Init(in Cylinder Cyl)
    {
        myType = OtherCurve;
        myCylinder = Cyl;
        myIsPeriodic = false;
        isDone = false;
    }

    private static Point2D EvalPnt2d(in Point3D P, in Cylinder Cy)
    {
        Vector3D OP = new(Cy.Location(), P);
        double X = OP.Dot(new Vector3D(Cy.Position().XDirection()));
        double Y = OP.Dot(new Vector3D(Cy.Position().YDirection()));
        double Z = OP.Dot(new Vector3D(Cy.Position().Direction()));
        double U;

        if (Math.Abs(X) > global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(Y) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            U = Math.Atan2(Y, X);
        }
        else
        {
            U = 0.0;
        }
        return new Point2D(U, Z);
    }

    public override void Project(in Line3D L)
    {
        // Check the line is parallel to the axis of cylinder.
        // In other cases, the projection is wrong.
        if (L.Direction().XYZ().CrossSquareMagnitude(myCylinder.Position().Direction().XYZ())
            > global::Kerf.Engine.Foundation.Precision.Angular() * global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return;
        }

        myType = global::Kerf.Geometry.CurveKind.Line;

        Point2D P2d = EvalPnt2d(L.Location(), myCylinder);
        if (P2d.X() < 0.0)
        {
            P2d.SetX(P2d.X() + 2 * M_PI);
        }
        double Signe = L.Direction().Dot(myCylinder.Position().Direction());
        Signe = (Signe > 0.0) ? 1.0 : -1.0;
        Direction2D D2d = new(0.0, Signe);

        myLin = new Line2D(P2d, D2d);
        isDone = true;
    }

    public override void Project(in Circle3D C)
    {
        // Check the circle's normal is parallel to the axis of cylinder.
        // In other cases, the projection is wrong.
        CoordinateSystem aCylPos = myCylinder.Position();
        Frame3D aCircPos = C.Position();
        if (aCylPos.Direction().XYZ().CrossSquareMagnitude(aCircPos.Direction().XYZ())
            > global::Kerf.Engine.Foundation.Precision.Angular() * global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return;
        }

        myType = global::Kerf.Geometry.CurveKind.Line;

        Direction3D ZCyl = aCylPos.XDirection().Crossed(aCylPos.YDirection());

        double U = aCylPos.XDirection().AngleWithRef(aCircPos.XDirection(), ZCyl);

        Vector3D OP = new(myCylinder.Location(), C.Location());
        double V = OP.Dot(new Vector3D(aCylPos.Direction()));

        Point2D P2d1 = new(U, V);
        Direction2D D2d = new();
        if (ZCyl.Dot(aCircPos.Direction()) > 0.0)
        {
            D2d.SetCoord(1.0, 0.0);
        }
        else
        {
            D2d.SetCoord(-1.0, 0.0);
        }

        myLin = new Line2D(P2d1, D2d);
        isDone = true;
    }

    // void  ProjLib_Cylinder::Project(const gp_Elips& E)
    public override void Project(in Ellipse3D E)
    {
        // Due to widespread issues with poorly handled periodicity,
        // the projection of an ellipse onto a cylinder is delegated to approximation.
    }

    public override void Project(in Parabola3D P)
    {
        base.Project(P);
    }

    public override void Project(in Hyperbola3D H)
    {
        base.Project(H);
    }
}
