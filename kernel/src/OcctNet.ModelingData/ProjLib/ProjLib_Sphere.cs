// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Sphere.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>Projects elementary curves on a sphere: circles that are U or V isolines.</summary>
public class ProjLib_Sphere : ProjLib_Projector
{
    private gp_Sphere mySphere = new();

    /// <summary>Undefined projection.</summary>
    public ProjLib_Sphere()
    {
    }

    /// <summary>Projection on the sphere Sp.</summary>
    public ProjLib_Sphere(in gp_Sphere Sp)
    {
        Init(Sp);
    }

    /// <summary>Projection of the circle C on the sphere Sp.</summary>
    public ProjLib_Sphere(in gp_Sphere Sp, in gp_Circ C)
    {
        Init(Sp);
        Project(C);
    }

    public void Init(in gp_Sphere Sp)
    {
        myType = GeomAbs_OtherCurve;
        mySphere = Sp;
        myIsPeriodic = false;
        isDone = false;
    }

    private static gp_Pnt2d EvalPnt2d(in gp_Vec P, in gp_Sphere Sp)
    {
        double X = P.Dot(new gp_Vec(Sp.Position().XDirection()));
        double Y = P.Dot(new gp_Vec(Sp.Position().YDirection()));
        double Z = P.Dot(new gp_Vec(Sp.Position().Direction()));
        double U, V;

        if (Math.Abs(X) > Precision.PConfusion() || Math.Abs(Y) > Precision.PConfusion())
        {
            double UU = Math.Atan2(Y, X);
            U = ElCLib.InPeriod(UU, 0.0, 2 * M_PI);
        }
        else
        {
            U = 0.0;
        }

        if (Z > 1.0)
        {
            Z = 1.0;
        }
        else if (Z < -1.0)
        {
            Z = -1.0;
        }
        V = Math.Asin(Z);

        return new gp_Pnt2d(U, V);
    }

    public override void Project(in gp_Circ C)
    {
        gp_Pnt O;          // O Location of Sp;
        gp_Dir Xc, Yc, Zc; // X Y Z Direction of C;
        gp_Dir Xs, Ys, Zs; // X Y Z Direction of Sp;

        // Check the validity :
        //                      Xc & Yc must be perpendicular to Zs ->IsoV;
        //                      O,Zs is in the Plane O,Xc,Yc;       ->IsoU;

        O = mySphere.Location();
        Xc = C.Position().XDirection();
        Yc = C.Position().YDirection();
        Zc = Xc ^ Yc;
        Xs = mySphere.Position().XDirection();
        Ys = mySphere.Position().YDirection();
        Zs = mySphere.Position().Direction();

        bool isIsoU, isIsoV;
        double Tol = Precision.Confusion();

        isIsoU = Zc.IsNormal(Zs, Tol) && O.IsEqual(C.Location(), Tol);
        isIsoV = Xc.IsNormal(Zs, Tol) && Yc.IsNormal(Zs, Tol);

        gp_Pnt2d P2d1 = new(), P2d2;
        gp_Dir2d D2d = new();

        if (isIsoU)
        {
            myType = GeomAbs_Line;

            P2d1 = EvalPnt2d(new gp_Vec(Xc), mySphere);
            P2d2 = EvalPnt2d(new gp_Vec(Yc), mySphere);

            if (isIsoU
                && (Math.Abs(P2d1.Y() - M_PI / 2.0) < Precision.PConfusion()
                    || Math.Abs(P2d1.Y() + M_PI / 2.0) < Precision.PConfusion()))
            {
                // then P1 is on the apex of the sphere and U is undefined
                // The value of U is given by P2d2.Y() .
                P2d1.SetX(P2d2.X());
            }
            else if (Math.Abs(Math.Abs(P2d1.X() - P2d2.X()) - M_PI) < Precision.PConfusion())
            {
                // then we have U2 = U1 + PI; V2;
                // we have to assume that U1 = U2
                //   so V2 = PI - V2;
                P2d2.SetX(P2d1.X());
                if (P2d2.Y() < 0.0)
                {
                    P2d2.SetY(-M_PI - P2d2.Y());
                }
                else
                {
                    P2d2.SetY(M_PI - P2d2.Y());
                }
            }
            else
            {
                P2d2.SetX(P2d1.X());
            }

            D2d = new gp_Dir2d(new gp_Vec2d(P2d1, P2d2));
            isDone = true;
        }
        else if (isIsoV)
        {
            myType = GeomAbs_Line;

            // P2d(U,V) :first point of the PCurve.
            double U = Xs.AngleWithRef(Xc, Xs ^ Ys);
            if (U < 0)
            {
                U += 2 * M_PI;
            }
            double Z = new gp_Vec(O, C.Location()).Dot(new gp_Vec(Zs));
            double V = Math.Asin(Z / mySphere.Radius());
            P2d1 = new gp_Pnt2d(U, V);
            D2d = new gp_Dir2d((Xc ^ Yc).Dot(Xs ^ Ys), 0.0);
            isDone = true;
        }
        myLin = new gp_Lin2d(P2d1, D2d);
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

    /// <summary>Sets the point of parameter U on C in the natural restrictions of the sphere.</summary>
    /// <exception cref="StdFail_NotDone">The projection is not done.</exception>
    public void SetInBounds(double U)
    {
        StdFail_NotDone_Raise_if(!isDone, "ProjLib_Sphere:SetInBounds");

        // first set the y of the first point in -pi/2 pi/2
        double newY, Y = ElCLib.Value(U, myLin).Y();
        newY = ElCLib.InPeriod(Y, -M_PI, M_PI);

        myLin.Translate(new gp_Vec2d(0.0, newY - Y));

        gp_Pnt2d P = ElCLib.Value(U, myLin);
        gp_Trsf2d Trsf = new();
        gp_Ax2d Axis;
        double Tol = 1.0e-7;
        gp_Dir2d D2d = myLin.Direction();
        //  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116 Begin
        //   if ((P.Y() > M_PI/2) ||
        if ((P.Y() - M_PI / 2 > Tol) ||
            //  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116 End
            (Math.Abs(P.Y() - M_PI / 2) < Tol && D2d.IsEqual(gp.DY2d(), Tol)))
        {
            Axis = new gp_Ax2d(new gp_Pnt2d(0.0, M_PI / 2.0), gp.DX2d());
        }
        //  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116 Begin
        //   else if ((P.Y() < -M_PI/2) ||
        else if ((P.Y() + M_PI / 2 < -Tol) ||
                 //  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116 End
                 (Math.Abs(P.Y() + M_PI / 2) < Tol && D2d.IsOpposite(gp.DY2d(), Tol)))
        {
            Axis = new gp_Ax2d(new gp_Pnt2d(0.0, -M_PI / 2.0), gp.DX2d());
        }
        else
        {
            return;
        }
        Trsf.SetMirror(Axis);
        myLin.Transform(Trsf);

        myLin.Translate(new gp_Vec2d(M_PI, 0.0));

        // Now we need to adjust the U parameter
        double newX, X = ElCLib.Value(U, myLin).X();
        newX = ElCLib.InPeriod(X, 0.0, 2.0 * M_PI);
        myLin.Translate(new gp_Vec2d(newX - X, 0.0));
    }
}
