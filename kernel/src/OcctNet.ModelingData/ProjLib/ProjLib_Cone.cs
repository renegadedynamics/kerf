// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Cone.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Cone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>Projects elementary curves on a cone: lines along a generatrix and circles normal to the axis.</summary>
public class ProjLib_Cone : ProjLib_Projector
{
    private gp_Cone myCone = new();

    /// <summary>Undefined projection.</summary>
    public ProjLib_Cone()
    {
    }

    /// <summary>Projection on the cone Co.</summary>
    public ProjLib_Cone(in gp_Cone Co)
    {
        Init(Co);
    }

    /// <summary>Projection of the line L on the cone Co.</summary>
    public ProjLib_Cone(in gp_Cone Co, in gp_Lin L)
    {
        Init(Co);
        Project(L);
    }

    /// <summary>Projection of the circle C on the cone Co.</summary>
    public ProjLib_Cone(in gp_Cone Co, in gp_Circ C)
    {
        Init(Co);
        Project(C);
    }

    public void Init(in gp_Cone Co)
    {
        myType = GeomAbs_OtherCurve;
        myCone = Co;
        myIsPeriodic = false;
        isDone = false;
    }

    public override void Project(in gp_Lin L)
    {
        gp_Pnt aPnt = L.Location(), anApex = myCone.Apex();

        double aDeltaV = 0.0;

        double U, V;
        if (aPnt.IsEqual(anApex, Precision.Confusion()))
        {
            // Take another point in the line L, which does not coincide with the cone apex.
            aPnt.Translate(new gp_Vec(L.Direction().XYZ()));
            aDeltaV = 1.0; // == ||L.Direction()|| == 1.0
        }

        ElSLib.ConeParameters(myCone.Position(), myCone.RefRadius(), myCone.SemiAngle(), aPnt, out U, out V);
        //
        gp_Pnt P;
        gp_Vec Vu, Vv;

        ElSLib.ConeD1(U, V, myCone.Position(), myCone.RefRadius(), myCone.SemiAngle(), out P, out Vu, out Vv);

        gp_Dir Dv = new(Vv);
        if (Dv.IsParallel(L.Direction(), Precision.Angular()))
        {
            // L is parallel to U-isoline of the cone.
            myType = GeomAbs_Line;

            double aSign = Math.CopySign(1.0, L.Direction().Dot(Dv));
            gp_Pnt2d P2d = new(U, V - aDeltaV * aSign);
            gp_Dir2d D2d = new(0.0, aSign);

            myLin = new gp_Lin2d(P2d, D2d);

            isDone = true;
        }
    }

    public override void Project(in gp_Circ C)
    {
        myType = GeomAbs_Line;

        gp_Ax3 ConePos = myCone.Position();
        gp_Ax3 CircPos = new(C.Position());
        //
        if (!ConePos.Direction().IsParallel(CircPos.Direction(), Precision.Angular()))
        {
            isDone = false;
            return;
        }
        //
        gp_Dir ZCone = ConePos.XDirection().Crossed(ConePos.YDirection());
        gp_Dir ZCir = CircPos.XDirection().Crossed(CircPos.YDirection());

        double U, V;
        double x = ConePos.XDirection().Dot(CircPos.XDirection());
        double y = ConePos.YDirection().Dot(CircPos.XDirection());
        double z = new gp_Vec(myCone.Location(), C.Location()).Dot(new gp_Vec(ConePos.Direction()));

        // to find point U V, we use the code from ElSLib
        // without applying the Trsf to the point (unnecessary round trip).
        // x/y are direction cosines (dimensionless); use angular tolerance
        // to avoid unstable atan2() around zero.
        if (Math.Abs(x) <= Precision.Angular() && Math.Abs(y) <= Precision.Angular())
        {
            U = 0.0;
        }
        else if (-myCone.RefRadius() > z * Math.Tan(myCone.SemiAngle()))
        {
            U = Math.Atan2(-y, -x);
        }
        else
        {
            U = Math.Atan2(y, x);
        }
        if (U < 0.0)
        {
            U += 2 * M_PI;
        }

        V = z / Math.Cos(myCone.SemiAngle());

        gp_Pnt2d P2d1 = new(U, V);
        gp_Dir2d D2d = new();
        if (ZCone.Dot(ZCir) > 0.0)
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

    public override void Project(in gp_Elips E)
    {
        base.Project(E);
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
