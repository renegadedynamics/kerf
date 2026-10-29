// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Cylinder.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Cylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>Projects elementary curves on a cylinder: lines parallel to the axis and circles normal to it.</summary>
public class ProjLib_Cylinder : ProjLib_Projector
{
    private gp_Cylinder myCylinder = new();

    /// <summary>Undefined projection.</summary>
    public ProjLib_Cylinder()
    {
    }

    /// <summary>Projection on the cylinder Cyl.</summary>
    public ProjLib_Cylinder(in gp_Cylinder Cyl)
    {
        Init(Cyl);
    }

    /// <summary>Projection of the line L on the cylinder Cyl.</summary>
    public ProjLib_Cylinder(in gp_Cylinder Cyl, in gp_Lin L)
    {
        Init(Cyl);
        Project(L);
    }

    /// <summary>Projection of the circle C on the cylinder Cyl.</summary>
    public ProjLib_Cylinder(in gp_Cylinder Cyl, in gp_Circ C)
    {
        Init(Cyl);
        Project(C);
    }

    /// <summary>Projection of the ellipse E on the cylinder Cyl (left to approximation: never done).</summary>
    public ProjLib_Cylinder(in gp_Cylinder Cyl, in gp_Elips E)
    {
        Init(Cyl);
        Project(E);
    }

    public void Init(in gp_Cylinder Cyl)
    {
        myType = GeomAbs_OtherCurve;
        myCylinder = Cyl;
        myIsPeriodic = false;
        isDone = false;
    }

    private static gp_Pnt2d EvalPnt2d(in gp_Pnt P, in gp_Cylinder Cy)
    {
        gp_Vec OP = new(Cy.Location(), P);
        double X = OP.Dot(new gp_Vec(Cy.Position().XDirection()));
        double Y = OP.Dot(new gp_Vec(Cy.Position().YDirection()));
        double Z = OP.Dot(new gp_Vec(Cy.Position().Direction()));
        double U;

        if (Math.Abs(X) > Precision.PConfusion() || Math.Abs(Y) > Precision.PConfusion())
        {
            U = Math.Atan2(Y, X);
        }
        else
        {
            U = 0.0;
        }
        return new gp_Pnt2d(U, Z);
    }

    public override void Project(in gp_Lin L)
    {
        // Check the line is parallel to the axis of cylinder.
        // In other cases, the projection is wrong.
        if (L.Direction().XYZ().CrossSquareMagnitude(myCylinder.Position().Direction().XYZ())
            > Precision.Angular() * Precision.Angular())
        {
            return;
        }

        myType = GeomAbs_Line;

        gp_Pnt2d P2d = EvalPnt2d(L.Location(), myCylinder);
        if (P2d.X() < 0.0)
        {
            P2d.SetX(P2d.X() + 2 * M_PI);
        }
        double Signe = L.Direction().Dot(myCylinder.Position().Direction());
        Signe = (Signe > 0.0) ? 1.0 : -1.0;
        gp_Dir2d D2d = new(0.0, Signe);

        myLin = new gp_Lin2d(P2d, D2d);
        isDone = true;
    }

    public override void Project(in gp_Circ C)
    {
        // Check the circle's normal is parallel to the axis of cylinder.
        // In other cases, the projection is wrong.
        gp_Ax3 aCylPos = myCylinder.Position();
        gp_Ax2 aCircPos = C.Position();
        if (aCylPos.Direction().XYZ().CrossSquareMagnitude(aCircPos.Direction().XYZ())
            > Precision.Angular() * Precision.Angular())
        {
            return;
        }

        myType = GeomAbs_Line;

        gp_Dir ZCyl = aCylPos.XDirection().Crossed(aCylPos.YDirection());

        double U = aCylPos.XDirection().AngleWithRef(aCircPos.XDirection(), ZCyl);

        gp_Vec OP = new(myCylinder.Location(), C.Location());
        double V = OP.Dot(new gp_Vec(aCylPos.Direction()));

        gp_Pnt2d P2d1 = new(U, V);
        gp_Dir2d D2d = new();
        if (ZCyl.Dot(aCircPos.Direction()) > 0.0)
        {
            D2d.SetCoord(1.0, 0.0);
        }
        else
        {
            D2d.SetCoord(-1.0, 0.0);
        }

        myLin = new gp_Lin2d(P2d1, D2d);
        isDone = true;
    }

    // void  ProjLib_Cylinder::Project(const gp_Elips& E)
    public override void Project(in gp_Elips E)
    {
        // Due to widespread issues with poorly handled periodicity,
        // the projection of an ellipse onto a cylinder is delegated to approximation.
    }

    public override void Project(in gp_Parab P)
    {
        base.Project(P);
    }

    public override void Project(in gp_Hypr H)
    {
        base.Project(H);
    }
}
