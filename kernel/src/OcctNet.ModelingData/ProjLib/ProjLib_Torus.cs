// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Torus.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Torus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>Projects elementary curves on a torus: circles that are U or V isolines.</summary>
public class ProjLib_Torus : ProjLib_Projector
{
    private gp_Torus myTorus = new();

    /// <summary>Undefined projection.</summary>
    public ProjLib_Torus()
    {
    }

    /// <summary>Projection on the torus To.</summary>
    public ProjLib_Torus(in gp_Torus To)
    {
        Init(To);
    }

    /// <summary>Projection of the circle C on the torus To.</summary>
    public ProjLib_Torus(in gp_Torus To, in gp_Circ C)
    {
        Init(To);
        Project(C);
    }

    public void Init(in gp_Torus To)
    {
        myType = GeomAbs_OtherCurve;
        myTorus = To;
        myIsPeriodic = false;
        isDone = false;
    }

    private static gp_Pnt2d EvalPnt2d(in gp_Vec Ve, in gp_Torus To)
    {
        double X = Ve.Dot(new gp_Vec(To.Position().XDirection()));
        double Y = Ve.Dot(new gp_Vec(To.Position().YDirection()));
        double U, V;

        if (Math.Abs(X) > Precision.PConfusion() || Math.Abs(Y) > Precision.PConfusion())
        {
            U = Math.Atan2(Y, X);
        }
        else
        {
            U = 0.0;
        }

        V = 0.0;
        return new gp_Pnt2d(U, V);
    }

    public override void Project(in gp_Circ C)
    {
        myType = GeomAbs_Line;

        gp_Vec Xc = new(C.Position().XDirection());
        gp_Vec Yc = new(C.Position().YDirection());
        gp_Vec Xt = new(myTorus.Position().XDirection());
        gp_Vec Yt = new(myTorus.Position().YDirection());
        gp_Vec Zt = new(myTorus.Position().Direction());
        gp_Vec OC = new(myTorus.Location(), C.Location());

        //  if (OC.Magnitude() < Precision::Confusion()      ||
        //      OC.IsParallel(myTorus.Position().Direction(),
        //		    Precision::Angular())) {

        if (OC.Magnitude() < Precision.Confusion()
            || C.Position().Direction().IsParallel(myTorus.Position().Direction(), Precision.Angular()))
        {
            // Iso V
            gp_Pnt2d P1 = EvalPnt2d(Xc, myTorus); // evaluate U1
            gp_Pnt2d P2 = EvalPnt2d(Yc, myTorus); // evaluate U2
            double Z = OC.Dot(new gp_Vec(myTorus.Position().Direction()));
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
            gp_Vec2d V2d = new(P1, P2);
            // Normally std::abs( P1.X() - P2.X()) = PI/2
            // If != PI/2, we crossed the period => Reverse the Direction
            if (Math.Abs(P1.X() - P2.X()) > M_PI)
            {
                V2d.Reverse();
            }

            gp_Dir2d D2 = new(V2d);
            if (P1.X() < 0)
            {
                P1.SetX(2 * M_PI + P1.X());
            }
            myLin = new gp_Lin2d(P1, D2);
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

            gp_Pnt2d P1 = new(U, V1);

            // Direction of the line
            gp_Dir2d D2 = gp.DY2d();
            if (((OC ^ Zt) * (Xc ^ Yc)) < 0.0)
            {
                D2.Reverse();
            }

            myLin = new gp_Lin2d(P1, D2);
        }
        isDone = true;
    }

    public override void Project(in gp_Lin L)
    {
        base.Project(L);
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
