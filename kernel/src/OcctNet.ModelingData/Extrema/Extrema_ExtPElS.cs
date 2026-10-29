// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPElS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPElS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// All the extremum distances (minimum or maximum) between a point and an elementary surface (plane, cylinder,
/// cone, sphere or torus).
/// </summary>
/// <remarks>An algorithm object: construct it with the point and the surface, or empty and call <c>Perform</c>.</remarks>
public class Extrema_ExtPElS
{
    private static readonly double ExtPElS_MyEps = Epsilon(2.0 * M_PI);

    private bool myDone;
    private int myNbExt;
    private readonly double[] mySqDist = new double[4];
    private readonly Extrema_POnSurf[] myPoint = [new(), new(), new(), new()];

    public Extrema_ExtPElS()
    {
        myDone = false;
        myNbExt = 0;
        for (int i = 0; i < 4; i++)
        {
            mySqDist[i] = RealLast();
        }
    }

    /// <summary>The distances between a point and a cylinder. Tol is used to test if the point is on the axis.</summary>
    public Extrema_ExtPElS(in gp_Pnt P, in gp_Cylinder S, double Tol) => Perform(P, S, Tol);

    /*-----------------------------------------------------------------------------
    Function:
    Find 2 extreme distances between point P and cylinder S.

    Method:
      Let Pp be the projection of P in plane XOY of the cylinder;
      2 cases are considered:
      1- distance(Pp,O) < Tol:
         There are infinite solutions; IsDone() = false.
      2- distance(Pp,O) > Tol:
         let V = OP.OZ,
              U1 = angle(OX,OPp) with 0 < U1 < 2.*M_PI
          U2 = U1 + M_PI with 0 < U2 < 2.*M_PI;
         then (U1,V) corresponds to the min distance.
         and  (U2,V) corresponds to the max distance.
    -----------------------------------------------------------------------------*/
    public void Perform(in gp_Pnt P, in gp_Cylinder S, double Tol)
    {
        myDone = false;
        myNbExt = 0;

        // Projection of point P in plane XOY of the cylinder ...
        gp_Ax3 Pos = S.Position();
        gp_Pnt O = Pos.Location();
        gp_Vec OZ = new(Pos.Direction());
        double V = new gp_Vec(O, P).Dot(OZ);
        gp_Pnt Pp = P.Translated(OZ.Multiplied(-V));

        // Calculation of extrema
        gp_Vec OPp = new(O, Pp);
        if (OPp.Magnitude() < Tol)
        {
            return;
        }
        gp_Vec myZ = new(Pos.XDirection() ^ Pos.YDirection());
        double U1 = new gp_Vec(Pos.XDirection()).AngleWithRef(OPp, myZ); //-M_PI<U1<M_PI
        if (U1 > -ExtPElS_MyEps && U1 < ExtPElS_MyEps)
        {
            U1 = 0.0;
        }
        double U2 = U1 + M_PI;
        if (U1 < 0.0)
        {
            U1 += 2.0 * M_PI;
        }

        gp_Pnt Ps;
        Ps = ElSLib.Value(U1, V, S);
        mySqDist[0] = Ps.SquareDistance(P);
        myPoint[0] = new Extrema_POnSurf(U1, V, Ps);
        Ps = ElSLib.Value(U2, V, S);
        mySqDist[1] = Ps.SquareDistance(P);
        myPoint[1] = new Extrema_POnSurf(U2, V, Ps);

        myNbExt = 2;
        myDone = true;
    }

    /// <summary>
    /// The distances between a point and a cone. Tol is used to test if the point is at the apex or on the axis.
    /// </summary>
    public Extrema_ExtPElS(in gp_Pnt P, in gp_Cone S, double Tol) => Perform(P, S, Tol);

    /*-----------------------------------------------------------------------------
    Function:
       Find 2 extreme distances between point P and cone S.

    Method:
      Let M the top of the cone.
      2 cases are considered:
      1- distance(P,M) < Tol:
         there is a minimum in M.
      2- distance(P,M) > Tol:
         Let Pp the projection of P in the plane XOY of the cone;
         2 cases are considered:
         1- distance(Pp,O) < Tol:
         There is an infinite number of solutions; IsDone() = false.
         2- distance(Pp,O) > Tol:
            There exist 2 extrema:
            Let Vm = value of v for point M,
                 Vp = value of v for point P,
                 U1 = angle(OX,OPp) if Vp > Vm )
                 -angle(OX,OPp) otherwise      ) with 0. < U1 < 2*M_PI,
                 U2 = U1 + M_PI with 0. < U2 < 2*M_PI;
            We are in plane PpOZ.
           Let A the angle of the cone,
                 B = angle(MP,MO) with 0. < B < M_PI,
             L = longueur(MP),
             V1 = (L * cos(B-A)) + Vm,
             V2 = (L * cos(B+A)) + Vm;
           then (U1,V1) and (U2,V2) correspond to min distances.
    -----------------------------------------------------------------------------*/
    public void Perform(in gp_Pnt P, in gp_Cone S, double Tol)
    {
        myDone = false;
        myNbExt = 0;

        gp_Pnt M = S.Apex();
        gp_Ax3 Pos = S.Position();
        gp_Pnt O = Pos.Location();
        double A = S.SemiAngle();
        gp_Vec OZ = new(Pos.Direction());
        gp_Vec myZ = new(Pos.XDirection() ^ Pos.YDirection());
        gp_Vec MP = new(M, P);

        double L2 = MP.SquareMagnitude();
        double Vm = -(S.RefRadius() / Math.Sin(A));

        // Case when P is mixed with S ...
        if (L2 < Tol * Tol)
        {
            mySqDist[0] = L2;
            myPoint[0] = new Extrema_POnSurf(0.0, Vm, M);
            myNbExt = 1;
            myDone = true;
            return;
        }
        gp_Vec DirZ;
        if (M.SquareDistance(O) < Tol * Tol)
        {
            DirZ = (A < 0 ? -OZ : OZ);
        }
        else
        {
            DirZ = new gp_Vec(M, O);
        }

        // Projection of P in the reference plane of the cone ...
        double Zp = new gp_Vec(O, P).Dot(OZ);

        gp_Pnt Pp = P.Translated(OZ.Multiplied(-Zp));
        gp_Vec OPp = new(O, Pp);
        if (OPp.SquareMagnitude() < Tol * Tol)
        {
            return;
        }
        double U1, V1, U2, V2;
        bool Same = DirZ.Dot(MP) >= 0.0;
        U1 = new gp_Vec(Pos.XDirection()).AngleWithRef(OPp, myZ); //-M_PI<U1<M_PI
        if (U1 > -ExtPElS_MyEps && U1 < ExtPElS_MyEps)
        {
            U1 = 0.0;
        }
        if (!Same)
        {
            U1 += M_PI;
        }
        U2 = U1 + M_PI;
        if (U1 < 0.0)
        {
            U1 += 2.0 * M_PI;
        }
        if (U2 > 2.0 * M_PI)
        {
            U2 -= 2.0 * M_PI;
        }
        double B = MP.Angle(DirZ);
        A = Math.Abs(A);
        double L = Math.Sqrt(L2);
        if (!Same)
        {
            B = M_PI - B;
            V1 = -L * Math.Cos(B - A);
            V2 = -L * Math.Cos(B + A);
        }
        else
        {
            V1 = L * Math.Cos(B - A);
            V2 = L * Math.Cos(B + A);
        }
        double Sense = OZ.Dot(new gp_Vec(new gp_Dir(DirZ)));
        V1 *= Sense;
        V2 *= Sense;
        V1 += Vm;
        V2 += Vm;

        gp_Pnt Ps;
        Ps = ElSLib.Value(U1, V1, S);
        mySqDist[0] = Ps.SquareDistance(P);
        myPoint[0] = new Extrema_POnSurf(U1, V1, Ps);
        Ps = ElSLib.Value(U2, V2, S);
        mySqDist[1] = Ps.SquareDistance(P);
        myPoint[1] = new Extrema_POnSurf(U2, V2, Ps);

        myNbExt = 2;
        myDone = true;
    }

    /// <summary>The distances between a point and a sphere. Tol is used to test if the point is at the center.</summary>
    public Extrema_ExtPElS(in gp_Pnt P, in gp_Sphere S, double Tol) => Perform(P, S, Tol);

    /*-----------------------------------------------------------------------------
    Function:
      Find 2 extreme distances between point P and sphere S.

    Method:
       Let O be the origin of the sphere.
      2 cases are considered:
      1- distance(P,O) < Tol:
         There is an infinite number of solutions; IsDone() = false
      2- distance(P,O) > Tol:
         Let Pp be the projection of point P in the plane XOY of the sphere;
         2 cases are considered:
         1- distance(Pp,O) < Tol:
            2 solutions are: (0,-M_PI/2.) and (0.,M_PI/2.)
         2- distance(Pp,O) > Tol:
            Let U1 = angle(OX,OPp) with 0. < U1 < 2.*M_PI,
             U2 = U1 + M_PI avec 0. < U2 < 2*M_PI,
             V1 = angle(OPp,OP) with -M_PI/2. < V1 < M_PI/2. ,
        then (U1, V1) corresponds to the min distance
        and  (U2,-V1) corresponds to the max distance.
    -----------------------------------------------------------------------------*/
    public void Perform(in gp_Pnt P, in gp_Sphere S, double Tol)
    {
        myDone = false;
        myNbExt = 0;

        gp_Ax3 Pos = S.Position();
        gp_Vec OP = new(Pos.Location(), P);

        // Case when P is mixed with O ...
        if (OP.SquareMagnitude() < Tol * Tol)
        {
            return;
        }

        // Projection if P in plane XOY of the sphere ...
        gp_Pnt O = Pos.Location();
        gp_Vec OZ = new(Pos.Direction());
        double Zp = OP.Dot(OZ);
        gp_Pnt Pp = P.Translated(OZ.Multiplied(-Zp));

        // Calculation of extrema ...
        gp_Vec OPp = new(O, Pp);
        double U1, U2, V;
        if (OPp.SquareMagnitude() < Tol * Tol)
        {
            U1 = 0.0;
            U2 = 0.0;
            if (Zp < 0.0)
            {
                V = -M_PI / 2.0;
            }
            else
            {
                V = M_PI / 2.0;
            }
        }
        else
        {
            gp_Vec myZ = new(Pos.XDirection() ^ Pos.YDirection());
            U1 = new gp_Vec(Pos.XDirection()).AngleWithRef(OPp, myZ);
            if (U1 > -ExtPElS_MyEps && U1 < ExtPElS_MyEps)
            {
                U1 = 0.0;
            }
            U2 = U1 + M_PI;
            if (U1 < 0.0)
            {
                U1 += 2.0 * M_PI;
            }
            V = OP.Angle(OPp);
            if (Zp < 0.0)
            {
                V = -V;
            }
        }

        gp_Pnt Ps;
        Ps = ElSLib.Value(U1, V, S);
        mySqDist[0] = Ps.SquareDistance(P);
        myPoint[0] = new Extrema_POnSurf(U1, V, Ps);
        Ps = ElSLib.Value(U2, -V, S);
        mySqDist[1] = Ps.SquareDistance(P);
        myPoint[1] = new Extrema_POnSurf(U2, -V, Ps);

        myNbExt = 2;
        myDone = true;
    }

    /// <summary>The distances between a point and a torus. Tol is used to test if the point is on the axis.</summary>
    public Extrema_ExtPElS(in gp_Pnt P, in gp_Torus S, double Tol) => Perform(P, S, Tol);

    /*-----------------------------------------------------------------------------
    Function:
      Find 2 extreme distances between point P and torus S.

      Method:
      Let Pp be the projection of point P in plane XOY of the torus;
      2 cases are considered:
      1- distance(Pp,O) < Tol:
         There is an infinite number of solutions; IsDone() = false.
      2- distance(Pp,O) > Tol:
         One is located in plane PpOZ;
         Let V1 = angle(OX,OPp) with 0. < V1 < 2.*M_PI,
         V2 = V1 + M_PI with 0. < V2 < 2.*M_PI,
         O1 and O2 centers of circles (O1 on coord. posit.)
             U1 = angle(OPp,O1P),
         U2 = angle(OPp,PO2);
         then (U1,V1) corresponds to the min distance
         and  (U2,V2) corresponds to the max distance.
    -----------------------------------------------------------------------------*/
    public void Perform(in gp_Pnt P, in gp_Torus S, double Tol)
    {
        double tol2 = Tol * Tol;
        myDone = false;
        myNbExt = 0;

        // Projection of P in plane XOY ...
        gp_Ax3 Pos = S.Position();
        gp_Pnt O = Pos.Location();
        gp_Vec OZ = new(Pos.Direction());
        gp_Pnt Pp = P.Translated(OZ.Multiplied(-(new gp_Vec(O, P).Dot(new gp_Vec(Pos.Direction())))));

        // Calculation of extrema ...
        gp_Vec OPp = new(O, Pp);
        double R2 = OPp.SquareMagnitude();
        if (R2 < tol2)
        {
            return;
        }

        gp_Vec myZ = new(Pos.XDirection() ^ Pos.YDirection());
        double U1 = new gp_Vec(Pos.XDirection()).AngleWithRef(OPp, myZ);
        if (U1 > -ExtPElS_MyEps && U1 < ExtPElS_MyEps)
        {
            U1 = 0.0;
        }
        double U2 = U1 + M_PI;
        if (U1 < 0.0)
        {
            U1 += 2.0 * M_PI;
        }
        double R = Math.Sqrt(R2);
        gp_Vec OO1 = OPp.Divided(R).Multiplied(S.MajorRadius());
        gp_Vec OO2 = OO1.Multiplied(-1.0);
        gp_Pnt O1 = O.Translated(OO1);
        gp_Pnt O2 = O.Translated(OO2);

        if (O1.SquareDistance(P) < tol2)
        {
            return;
        }
        if (O2.SquareDistance(P) < tol2)
        {
            return;
        }

        double V1 = OPp.AngleWithRef(new gp_Vec(O1, P), OPp.Crossed(OZ));
        if (V1 > -ExtPElS_MyEps && V1 < ExtPElS_MyEps)
        {
            V1 = 0.0;
        }
        OPp.Reverse();
        double V2 = OPp.AngleWithRef(new gp_Vec(P, O2), OPp.Crossed(OZ));
        if (V2 > -ExtPElS_MyEps && V2 < ExtPElS_MyEps)
        {
            V2 = 0.0;
        }

        if (V1 < 0.0)
        {
            V1 += 2.0 * M_PI;
        }
        if (V2 < 0.0)
        {
            V2 += 2.0 * M_PI;
        }

        gp_Pnt Ps;
        Ps = ElSLib.Value(U1, V1, S);
        mySqDist[0] = Ps.SquareDistance(P);
        myPoint[0] = new Extrema_POnSurf(U1, V1, Ps);

        Ps = ElSLib.Value(U1, V1 + M_PI, S);
        mySqDist[1] = Ps.SquareDistance(P);
        myPoint[1] = new Extrema_POnSurf(U1, V1 + M_PI, Ps);

        Ps = ElSLib.Value(U2, V2, S);
        mySqDist[2] = Ps.SquareDistance(P);
        myPoint[2] = new Extrema_POnSurf(U2, V2, Ps);

        Ps = ElSLib.Value(U2, V2 + M_PI, S);
        mySqDist[3] = Ps.SquareDistance(P);
        myPoint[3] = new Extrema_POnSurf(U2, V2 + M_PI, Ps);

        myNbExt = 4;
        myDone = true;
    }

    /// <summary>The distance between a point and a plane (its projection). Tol is not used.</summary>
    public Extrema_ExtPElS(in gp_Pnt P, in gp_Pln S, double Tol) => Perform(P, S, Tol);

    public void Perform(in gp_Pnt P, in gp_Pln S, double Tol)
    {
        _ = Tol; // OCCT: the tolerance is not used
        myDone = false;
        myNbExt = 0;

        // Projection of point P in plane XOY of the cylinder ...
        gp_Pnt O = S.Location();
        gp_Vec OZ = new(S.Axis().Direction());
        double U, V = new gp_Vec(O, P).Dot(OZ);
        gp_Pnt Pp = P.Translated(OZ.Multiplied(-V));

        ElSLib.Parameters(S, P, out U, out V);
        mySqDist[0] = Pp.SquareDistance(P);
        myPoint[0] = new Extrema_POnSurf(U, V, Pp);
        myNbExt = 1;
        myDone = true;
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>The number of extremum distances.</summary>
    public int NbExt()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return myNbExt;
    }

    /// <summary>The value of the Nth resulting square distance.</summary>
    public double SquareDistance(int N)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new Standard_OutOfRange();
        }
        return mySqDist[N - 1];
    }

    /// <summary>The point of the Nth resulting distance.</summary>
    public Extrema_POnSurf Point(int N)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new Standard_OutOfRange();
        }
        return myPoint[N - 1];
    }
}
