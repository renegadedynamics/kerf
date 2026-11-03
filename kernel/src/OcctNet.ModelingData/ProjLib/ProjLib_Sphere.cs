// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Sphere.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>Projects elementary curves on a sphere: circles that are U or V isolines.</summary>
public class ProjLib_Sphere : ProjLib_Projector
{
    private Sphere mySphere = new();

    /// <summary>Undefined projection.</summary>
    public ProjLib_Sphere()
    {
    }

    /// <summary>Projection on the sphere Sp.</summary>
    public ProjLib_Sphere(in Sphere Sp)
    {
        Init(Sp);
    }

    /// <summary>Projection of the circle C on the sphere Sp.</summary>
    public ProjLib_Sphere(in Sphere Sp, in Circle3D C)
    {
        Init(Sp);
        Project(C);
    }

    public void Init(in Sphere Sp)
    {
        myType = OtherCurve;
        mySphere = Sp;
        myIsPeriodic = false;
        isDone = false;
    }

    private static Point2D EvalPnt2d(in Vector3D P, in Sphere Sp)
    {
        double X = P.Dot(new Vector3D(Sp.Position().XDirection()));
        double Y = P.Dot(new Vector3D(Sp.Position().YDirection()));
        double Z = P.Dot(new Vector3D(Sp.Position().Direction()));
        double U, V;

        if (Math.Abs(X) > global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(Y) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            double UU = Math.Atan2(Y, X);
            U = global::Kerf.Engine.Geometry.ElCLib.InPeriod(UU, 0.0, 2 * M_PI);
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

        return new Point2D(U, V);
    }

    public override void Project(in Circle3D C)
    {
        Point3D O;          // O Location of Sp;
        Direction3D Xc, Yc, Zc; // X Y Z Direction of C;
        Direction3D Xs, Ys, Zs; // X Y Z Direction of Sp;

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
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        isIsoU = Zc.IsNormal(Zs, Tol) && O.IsEqual(C.Location(), Tol);
        isIsoV = Xc.IsNormal(Zs, Tol) && Yc.IsNormal(Zs, Tol);

        Point2D P2d1 = new(), P2d2;
        Direction2D D2d = new();

        if (isIsoU)
        {
            myType = global::Kerf.Geometry.CurveKind.Line;

            P2d1 = EvalPnt2d(new Vector3D(Xc), mySphere);
            P2d2 = EvalPnt2d(new Vector3D(Yc), mySphere);

            if (isIsoU
                && (Math.Abs(P2d1.Y() - M_PI / 2.0) < global::Kerf.Engine.Foundation.Precision.PConfusion()
                    || Math.Abs(P2d1.Y() + M_PI / 2.0) < global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                // then P1 is on the apex of the sphere and U is undefined
                // The value of U is given by P2d2.Y() .
                P2d1.SetX(P2d2.X());
            }
            else if (Math.Abs(Math.Abs(P2d1.X() - P2d2.X()) - M_PI) < global::Kerf.Engine.Foundation.Precision.PConfusion())
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

            D2d = new Direction2D(new Vector2D(P2d1, P2d2));
            isDone = true;
        }
        else if (isIsoV)
        {
            myType = global::Kerf.Geometry.CurveKind.Line;

            // P2d(U,V) :first point of the PCurve.
            double U = Xs.AngleWithRef(Xc, Xs ^ Ys);
            if (U < 0)
            {
                U += 2 * M_PI;
            }
            double Z = new Vector3D(O, C.Location()).Dot(new Vector3D(Zs));
            double V = Math.Asin(Z / mySphere.Radius());
            P2d1 = new Point2D(U, V);
            D2d = new Direction2D((Xc ^ Yc).Dot(Xs ^ Ys), 0.0);
            isDone = true;
        }
        myLin = new Line2D(P2d1, D2d);
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

    /// <summary>Sets the point of parameter U on C in the natural restrictions of the sphere.</summary>
    /// <exception cref="NotDoneException">The projection is not done.</exception>
    public void SetInBounds(double U)
    {
        StdFail_NotDone_Raise_if(!isDone, "ProjLib_Sphere:SetInBounds");

        // first set the y of the first point in -pi/2 pi/2
        double newY, Y = global::Kerf.Engine.Geometry.ElCLib.Value(U, myLin).Y();
        newY = global::Kerf.Engine.Geometry.ElCLib.InPeriod(Y, -M_PI, M_PI);

        myLin.Translate(new Vector2D(0.0, newY - Y));

        Point2D P = global::Kerf.Engine.Geometry.ElCLib.Value(U, myLin);
        Transform2D Trsf = new();
        Axis2D Axis;
        double Tol = 1.0e-7;
        Direction2D D2d = myLin.Direction();
        //  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116 Begin
        //   if ((P.Y() > M_PI/2) ||
        if ((P.Y() - M_PI / 2 > Tol) ||
            //  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116 End
            (Math.Abs(P.Y() - M_PI / 2) < Tol && D2d.IsEqual(Constants.DY2d(), Tol)))
        {
            Axis = new Axis2D(new Point2D(0.0, M_PI / 2.0), Constants.DX2d());
        }
        //  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116 Begin
        //   else if ((P.Y() < -M_PI/2) ||
        else if ((P.Y() + M_PI / 2 < -Tol) ||
                 //  Modified by skv - Tue Aug  1 16:29:59 2006 OCC13116 End
                 (Math.Abs(P.Y() + M_PI / 2) < Tol && D2d.IsOpposite(Constants.DY2d(), Tol)))
        {
            Axis = new Axis2D(new Point2D(0.0, -M_PI / 2.0), Constants.DX2d());
        }
        else
        {
            return;
        }
        Trsf.SetMirror(Axis);
        myLin.Transform(Trsf);

        myLin.Translate(new Vector2D(M_PI, 0.0));

        // Now we need to adjust the U parameter
        double newX, X = global::Kerf.Engine.Geometry.ElCLib.Value(U, myLin).X();
        newX = global::Kerf.Engine.Geometry.ElCLib.InPeriod(X, 0.0, 2.0 * M_PI);
        myLin.Translate(new Vector2D(newX - X, 0.0));
    }
}
