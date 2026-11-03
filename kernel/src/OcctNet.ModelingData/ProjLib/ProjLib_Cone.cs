// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Cone.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Cone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>Projects elementary curves on a cone: lines along a generatrix and circles normal to the axis.</summary>
public class ProjLib_Cone : ProjLib_Projector
{
    private Cone myCone = new();

    /// <summary>Undefined projection.</summary>
    public ProjLib_Cone()
    {
    }

    /// <summary>Projection on the cone Co.</summary>
    public ProjLib_Cone(in Cone Co)
    {
        Init(Co);
    }

    /// <summary>Projection of the line L on the cone Co.</summary>
    public ProjLib_Cone(in Cone Co, in Line3D L)
    {
        Init(Co);
        Project(L);
    }

    /// <summary>Projection of the circle C on the cone Co.</summary>
    public ProjLib_Cone(in Cone Co, in Circle3D C)
    {
        Init(Co);
        Project(C);
    }

    public void Init(in Cone Co)
    {
        myType = OtherCurve;
        myCone = Co;
        myIsPeriodic = false;
        isDone = false;
    }

    public override void Project(in Line3D L)
    {
        Point3D aPnt = L.Location(), anApex = myCone.Apex();

        double aDeltaV = 0.0;

        double U, V;
        if (aPnt.IsEqual(anApex, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            // Take another point in the line L, which does not coincide with the cone apex.
            aPnt.Translate(new Vector3D(L.Direction().XYZ()));
            aDeltaV = 1.0; // == ||L.Direction()|| == 1.0
        }

        global::Kerf.Engine.Geometry.ElSLib.ConeParameters(myCone.Position(), myCone.RefRadius(), myCone.SemiAngle(), aPnt, out U, out V);
        //
        Point3D P;
        Vector3D Vu, Vv;

        global::Kerf.Engine.Geometry.ElSLib.ConeD1(U, V, myCone.Position(), myCone.RefRadius(), myCone.SemiAngle(), out P, out Vu, out Vv);

        Direction3D Dv = new(Vv);
        if (Dv.IsParallel(L.Direction(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            // L is parallel to U-isoline of the cone.
            myType = global::Kerf.Geometry.CurveKind.Line;

            double aSign = Math.CopySign(1.0, L.Direction().Dot(Dv));
            Point2D P2d = new(U, V - aDeltaV * aSign);
            Direction2D D2d = new(0.0, aSign);

            myLin = new Line2D(P2d, D2d);

            isDone = true;
        }
    }

    public override void Project(in Circle3D C)
    {
        // OCCT: the type is set before the check below, so a circle not normal to the axis leaves a "Line" result
        // that is not done, and Line() returns the default line instead of throwing
        myType = global::Kerf.Geometry.CurveKind.Line;

        CoordinateSystem ConePos = myCone.Position();
        CoordinateSystem CircPos = new(C.Position());
        //
        if (!ConePos.Direction().IsParallel(CircPos.Direction(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            isDone = false;
            return;
        }
        //
        Direction3D ZCone = ConePos.XDirection().Crossed(ConePos.YDirection());
        Direction3D ZCir = CircPos.XDirection().Crossed(CircPos.YDirection());

        double U, V;
        double x = ConePos.XDirection().Dot(CircPos.XDirection());
        double y = ConePos.YDirection().Dot(CircPos.XDirection());
        double z = new Vector3D(myCone.Location(), C.Location()).Dot(new Vector3D(ConePos.Direction()));

        // to find point U V, we use the code from ElSLib
        // without applying the Trsf to the point (unnecessary round trip).
        // x/y are direction cosines (dimensionless); use angular tolerance
        // to avoid unstable atan2() around zero.
        if (Math.Abs(x) <= global::Kerf.Engine.Foundation.Precision.Angular() && Math.Abs(y) <= global::Kerf.Engine.Foundation.Precision.Angular())
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

        Point2D P2d1 = new(U, V);
        Direction2D D2d = new();
        if (ZCone.Dot(ZCir) > 0.0)
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

    public override void Project(in Ellipse3D E)
    {
        base.Project(E);
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
