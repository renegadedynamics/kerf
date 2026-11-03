// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Torus.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Torus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>Projects elementary curves on a torus: circles that are U or V isolines.</summary>
public class ProjLib_Torus : ProjLib_Projector
{
    private Torus myTorus = new();

    /// <summary>Undefined projection.</summary>
    public ProjLib_Torus()
    {
    }

    /// <summary>Projection on the torus To.</summary>
    public ProjLib_Torus(in Torus To)
    {
        Init(To);
    }

    /// <summary>Projection of the circle C on the torus To.</summary>
    public ProjLib_Torus(in Torus To, in Circle3D C)
    {
        Init(To);
        Project(C);
    }

    public void Init(in Torus To)
    {
        myType = OtherCurve;
        myTorus = To;
        myIsPeriodic = false;
        isDone = false;
    }

    private static Point2D EvalPnt2d(in Vector3D Ve, in Torus To)
    {
        double X = Ve.Dot(new Vector3D(To.Position().XDirection()));
        double Y = Ve.Dot(new Vector3D(To.Position().YDirection()));
        double U, V;

        if (Math.Abs(X) > global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(Y) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            U = Math.Atan2(Y, X);
        }
        else
        {
            U = 0.0;
        }

        V = 0.0;
        return new Point2D(U, V);
    }

    public override void Project(in Circle3D C)
    {
        myType = global::Kerf.Geometry.CurveKind.Line;

        Vector3D Xc = new(C.Position().XDirection());
        Vector3D Yc = new(C.Position().YDirection());
        Vector3D Xt = new(myTorus.Position().XDirection());
        Vector3D Yt = new(myTorus.Position().YDirection());
        Vector3D Zt = new(myTorus.Position().Direction());
        Vector3D OC = new(myTorus.Location(), C.Location());

        //  if (OC.Magnitude() < Precision::Confusion()      ||
        //      OC.IsParallel(myTorus.Position().Direction(),
        //		    Precision::Angular())) {

        if (OC.Magnitude() < global::Kerf.Engine.Foundation.Precision.Confusion()
            || C.Position().Direction().IsParallel(myTorus.Position().Direction(), global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            // Iso V
            Point2D P1 = EvalPnt2d(Xc, myTorus); // evaluate U1
            Point2D P2 = EvalPnt2d(Yc, myTorus); // evaluate U2
            double Z = OC.Dot(new Vector3D(myTorus.Position().Direction()));
            Z /= myTorus.MinorRadius();

            double V;

            if (Z > 1.0)
            {
                V = M_PI / 2.0; // simple protection
            } // against calculation errors
            else if (Z < -1.0)
            {              // it happens that Z is slightly
                V = -M_PI / 2; // greater than 1.
            }
            else
            {
                V = Math.Asin(Z);
            }

            if (C.Radius() < myTorus.MajorRadius())
            {
                V = M_PI - V;
            }
            else if (V < 0.0)
            {
                V += 2 * M_PI;
            }
            P1.SetY(V);
            P2.SetY(V);
            Vector2D V2d = new(P1, P2);
            // Normally std::abs( P1.X() - P2.X()) = PI/2
            // If != PI/2, we crossed the period => Reverse the Direction
            if (Math.Abs(P1.X() - P2.X()) > M_PI)
            {
                V2d.Reverse();
            }

            Direction2D D2 = new(V2d);
            if (P1.X() < 0)
            {
                P1.SetX(2 * M_PI + P1.X());
            }
            myLin = new Line2D(P1, D2);
        }
        else
        {
            // Iso U  -> U = angle( Xt, OC)
            double U = Xt.AngleWithRef(OC, Xt ^ Yt);
            if (U < 0.0)
            {
                U += 2 * M_PI;
            }

            // Origin of the line
            double V1 = OC.AngleWithRef(Xc, OC ^ Zt);
            if (V1 < 0.0)
            {
                V1 += 2 * M_PI;
            }

            Point2D P1 = new(U, V1);

            // Direction of the line
            Direction2D D2 = Constants.DY2d();
            if (((OC ^ Zt) * (Xc ^ Yc)) < 0.0)
            {
                D2.Reverse();
            }

            myLin = new Line2D(P1, D2);
        }
        isDone = true;
    }

    public override void Project(in Line3D L)
    {
        base.Project(L);
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
