// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtElC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// All the extremum distances (minimum or maximum) between two elementary curves: a line and an elementary curve,
/// or two circles.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (OCCT gives it no Perform).</remarks>
public class Extrema_ExtElC
{
    private bool myDone;
    private bool myIsPar;
    private int myNbExt;
    private readonly double[] mySqDist = new double[6];

    // Extrema_POnCurv's default is all zeros, so the array needs no filling
    private readonly Extrema_POnCurv[,] myPoint = new Extrema_POnCurv[6, 2];

    public Extrema_ExtElC()
    {
        myDone = false;
        myIsPar = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }
    }

    // Function:
    //   Find min distance between 2 straight lines.

    // Method:
    //   Let D1 and D2, be 2 directions of straight lines C1 and C2.
    //   2 cases are considered:
    //   1- if Angle(D1,D2) < AngTol, straight lines are parallel.
    //      The distance is the distance between a point of C1 and the straight line C2.
    //   2- if Angle(D1,D2) > AngTol:
    //      Let P1=C1(u1) and P2=C2(u2).
    //      Then we must find u1 and u2 such as the distance P1P2 is minimal.
    //      Target function is:
    //        F(u1, u2) = ((L1x+D1x*u1)-(L2x+D2x*u2))^2 +
    //                    ((L1y+D1y*u1)-(L2y+D2y*u2))^2 +
    //                    ((L1z+D1z*u1)-(L2z+D2z*u2))^2 --> min,
    //      where L1 and L2 are lines locations, D1 and D2 are lines directions.
    //    Let simplify the function F

    //      F(u1, u2) = (D1x*u1-D2x*u2-Lx)^2 + (D1y*u1-D2y*u2-Ly)^2 + (D1z*u1-D2z*u2-Lz)^2,
    //    where L is a vector L1L2.

    //    In point of minimum, the condition
    //      {dF/du1 = 0
    //      {dF/du2 = 0

    //    must be satisfied.

    //      dF/du1 = 2*D1x*(D1x*u1-D2x*u2-Lx) +
    //               2*D1y*(D1y*u1-D2y*u2-Ly) +
    //               2*D1z*(D1z*u1-D2z*u2-Lz) =
    //             = 2*((D1^2)*u1-(D1.D2)*u2-L.D1) =
    //             = 2*(u1-(D1.D2)*u2-L.D1)
    //      dF/du2 = -2*D2x*(D1x*u1-D2x*u2-Lx) -
    //                2*D2y*(D1y*u1-D2y*u2-Ly) -
    //                2*D2z*(D1z*u1-D2z*u2-Lz)=
    //             = -2*((D2.D1)*u1-(D2^2)*u2-(D2.L)) =
    //             = -2*((D2.D1)*u1-u2-(D2.L))

    //   Consequently, we have two-equation system with two variables:

    //     {u1 - (D1.D2)*u2 = L.D1
    //     {(D1.D2)*u1 - u2 = L.D2

    //   This system has one solution if (D1.D2)^2 != 1
    //   (if straight lines are not parallel).

    /// <summary>
    /// The distance between two lines. AngTol is not used: the lines are parallel when their angle is below
    /// <see cref="Precision.Angular"/>.
    /// </summary>
    public Extrema_ExtElC(in Line3D theC1, in Line3D theC2, double AngTol)
    {
        _ = AngTol; // OCCT: the tolerance is not used
        myDone = false;
        myNbExt = 0;
        myIsPar = false;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }

        Direction3D aD1 = theC1.Position().Direction(), aD2 = theC2.Position().Direction();
        double aCosA = aD1.Dot(aD2);
        double aSqSinA = 1.0 - aCosA * aCosA;
        double aU1 = 0.0, aU2 = 0.0;
        if (aSqSinA < Constants.Resolution() || aD1.IsParallel(aD2, global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            myIsPar = true;
        }
        else
        {
            Xyz aL1L2 = theC2.Location().XYZ() - theC1.Location().XYZ();
            double aD1L = aD1.XYZ().Dot(aL1L2), aD2L = aD2.XYZ().Dot(aL1L2);
            aU1 = (aD1L - aCosA * aD2L) / aSqSinA;
            aU2 = (aCosA * aD1L - aD2L) / aSqSinA;

            myIsPar = global::Kerf.Engine.Foundation.Precision.IsInfinite(aU1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aU2);
        }

        if (myIsPar)
        {
            mySqDist[0] = theC2.SquareDistance(theC1.Location());
            myNbExt = 1;
            myDone = true;
            return;
        }

        // Here myIsPar == false;

        Point3D aP1 = global::Kerf.Engine.Geometry.ElCLib.Value(aU1, theC1), aP2 = global::Kerf.Engine.Geometry.ElCLib.Value(aU2, theC2);
        mySqDist[myNbExt] = aP1.SquareDistance(aP2);
        myPoint[myNbExt, 0] = new Extrema_POnCurv(aU1, aP1);
        myPoint[myNbExt, 1] = new Extrema_POnCurv(aU2, aP2);
        myNbExt = 1;
        myDone = true;
    }

    /// <summary>Computes the extrema when the line and the circle are in one plane.</summary>
    protected bool PlanarLineCircleExtrema(in Line3D theLin, in Circle3D theCirc)
    {
        Direction3D aDirC = theCirc.Axis().Direction(), aDirL = theLin.Direction();

        if (Math.Abs(aDirC.Dot(aDirL)) > global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return false;
        }

        // The line is in the circle-plane completely
        //(or parallel to the circle-plane).
        // Therefore, we are looking for extremas and
        // intersections in 2D-space.

        Xyz aCLoc = theCirc.Location().XYZ();
        Xyz aDCx = theCirc.Position().XDirection().XYZ(),
               aDCy = theCirc.Position().YDirection().XYZ();

        Xyz aLLoc = theLin.Location().XYZ();
        Xyz aLDir = theLin.Direction().XYZ();

        Xyz aVecCL = aLLoc - aCLoc;

        // Center of 2D-circle
        Point2D aPC = new(0.0, 0.0);

        Frame2D aCircAxis = new(aPC, new Direction2D(Direction2D.D.X), new Direction2D(Direction2D.D.Y));
        Circle2D aCirc2d = new(aCircAxis, theCirc.Radius());

        Point2D aPL = new(aVecCL.Dot(aDCx), aVecCL.Dot(aDCy));
        Direction2D aDL = new(aLDir.Dot(aDCx), aLDir.Dot(aDCy));
        Line2D aLin2d = new(aPL, aDL);

        // Extremas
        Extrema_ExtElC2d anExt2d = new(aLin2d, aCirc2d, global::Kerf.Engine.Foundation.Precision.Confusion());
        // Intersections
        IntAna2d_AnaIntersection anInters = new(aLin2d, aCirc2d);

        myDone = anExt2d.IsDone() || anInters.IsDone();

        if (!myDone)
        {
            return true;
        }

        int aNbExtr = anExt2d.NbExt();
        int aNbSol = anInters.NbPoints();

        int aNbSum = aNbExtr + aNbSol;
        for (int anExtrID = 1; anExtrID <= aNbSum; anExtrID++)
        {
            int aDelta = anExtrID - aNbExtr;

            double aLinPar = 0.0, aCircPar = 0.0;

            if (aDelta < 1)
            {
                Extrema_POnCurv2d aPLin2d, aPCirc2d;
                anExt2d.Points(anExtrID, out aPLin2d, out aPCirc2d);
                aLinPar = aPLin2d.Parameter();
                aCircPar = aPCirc2d.Parameter();
            }
            else
            {
                aLinPar = anInters.Point(aDelta).ParamOnFirst();
                aCircPar = anInters.Point(aDelta).ParamOnSecond();
            }

            Point3D aPOnL = global::Kerf.Engine.Geometry.ElCLib.LineValue(aLinPar, theLin.Position()),
                   aPOnC = global::Kerf.Engine.Geometry.ElCLib.CircleValue(aCircPar, theCirc.Position(), theCirc.Radius());

            mySqDist[myNbExt] = aPOnL.SquareDistance(aPOnC);
            myPoint[myNbExt, 0].SetValues(aLinPar, aPOnL);
            myPoint[myNbExt, 1].SetValues(aCircPar, aPOnC);
            myNbExt++;
        }

        return true;
    }

    //=================================================================================================
    // Find extreme distances between straight line C1 and circle C2.
    //
    // Method:
    //   Let P1=C1(u1) and P2=C2(u2) be two solution points
    //        D the direction of straight line C1
    //        T tangent at point P2;
    //  Then, ( P1P2.D = 0. (1)
    //         ( P1P2.T = 0. (2)
    //  Let O1 and O2 be the origins of C1 and C2;
    //  Then, (1) <=> (O1P2-u1*D).D = 0.         as O1P1 = u1*D
    //             <=> u1 = O1P2.D                as D.D = 1.
    //         (2) <=> P1O2.T = 0.                as O2P2.T = 0.
    //             <=> ((P2O1.D)D+O1O2).T = 0.    as P1O1 = -u1*D = (P2O1.D)D
    //             <=> (((P2O2+O2O1).D)D+O1O2).T = 0.
    //             <=> ((P2O2.D)(D.T)+((O2O1.D)D-O2O1).T = 0.
    //  We are in the reference frame of the circle; let:
    //         Cos = std::cos(u2) and Sin = std::sin(u2),
    //         P2 (R*Cos,R*Sin,0.),
    //         T (-R*Sin,R*Cos,0.),
    //         D (Dx,Dy,Dz),
    //         V (Vx,Vy,Vz) = (O2O1.D)D-O2O1;
    //  Then, the equation by Cos and Sin is as follows:
    //    -(2*R*R*Dx*Dy)   * Cos**2  +       A1
    //   R*R*(Dx**2-Dy**2) * Cos*Sin +    2* A2
    //         R*Vy        * Cos     +       A3
    //        -R*Vx        * Sin     +       A4
    //      R*R*Dx*Dy                = 0.    A5
    // Use the algorithm math_TrigonometricFunctionRoots to solve this equation.
    //=================================================================================================

    /// <summary>The distances between a line and a circle. Tol is not used.</summary>
    public Extrema_ExtElC(in Line3D C1, in Circle3D C2, double Tol)
    {
        _ = Tol; // OCCT: the tolerance is not used
        double Dx, Dy, Dz, aRO2O1, aTolRO2O1;
        double R, A1, A2, A3, A4, A5, aTol;
        Direction3D x2, y2, z2, D, D1;
        //
        myIsPar = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }

        if (PlanarLineCircleExtrema(C1, C2))
        {
            return;
        }

        // Calculate T1 in the reference of the circle ...
        D = C1.Direction();
        D1 = D;
        x2 = C2.XAxis().Direction();
        y2 = C2.YAxis().Direction();
        z2 = C2.Axis().Direction();
        Dx = D.Dot(x2);
        Dy = D.Dot(y2);
        Dz = D.Dot(z2);
        //
        D.SetCoord(Dx, Dy, Dz);
        RefineDir(ref D);
        D.Coord(out Dx, out Dy, out Dz);
        //
        // Calculate V in the reference frame of the circle:
        Point3D O1 = C1.Location();
        Point3D O2 = C2.Location();
        Vector3D O2O1 = new(O2, O1);
        //
        aTolRO2O1 = Constants.Resolution();
        aRO2O1 = O2O1.Magnitude();
        if (aRO2O1 > aTolRO2O1)
        {
            Direction3D aDO2O1 = new();
            //
            O2O1.Multiply(1.0 / aRO2O1);
            aDO2O1.SetCoord(O2O1.Dot(new Vector3D(x2)), O2O1.Dot(new Vector3D(y2)), O2O1.Dot(new Vector3D(z2)));
            RefineDir(ref aDO2O1);
            O2O1.SetXYZ(aRO2O1 * aDO2O1.XYZ());
        }
        else
        {
            O2O1.SetCoord(O2O1.Dot(new Vector3D(x2)), O2O1.Dot(new Vector3D(y2)), O2O1.Dot(new Vector3D(z2)));
        }
        //
        Xyz Vxyz = (D.XYZ() * (O2O1.Dot(new Vector3D(D)))) - O2O1.XYZ();
        //
        aTol = 1.0e-12;
        // Calculate the coefficients of the equation by Cos and Sin ...
        // [divided by R]
        R = C2.Radius();
        A5 = R * Dx * Dy;
        A1 = -2.0 * A5;
        A2 = 0.5 * R * (Dx * Dx - Dy * Dy); // /2.;
        A3 = Vxyz.Y();
        A4 = -Vxyz.X();
        //
        if (A1 >= -aTol && A1 <= aTol)
        {
            A1 = 0.0;
        }
        if (A2 >= -aTol && A2 <= aTol)
        {
            A2 = 0.0;
        }
        if (A3 >= -aTol && A3 <= aTol)
        {
            A3 = 0.0;
        }
        if (A4 >= -aTol && A4 <= aTol)
        {
            A4 = 0.0;
        }
        if (A5 >= -aTol && A5 <= aTol)
        {
            A5 = 0.0;
        }
        //
        ExtremaExtElC_TrigonometricRoots Sol = new(A1, A2, A3, A4, A5, 0.0, M_PI + M_PI);
        if (!Sol.IsDone())
        {
            return;
        }
        if (Sol.InfiniteRoots())
        {
            myIsPar = true;
            mySqDist[0] = R * R;
            myNbExt = 1;
            myDone = true;
            return;
        }
        // Storage of solutions ...
        int NoSol, NbSol;
        double U1, U2;
        Point3D P1, P2;
        //
        NbSol = Sol.NbSolutions();
        for (NoSol = 1; NoSol <= NbSol; ++NoSol)
        {
            U2 = Sol.Value(NoSol);
            P2 = global::Kerf.Engine.Geometry.ElCLib.Value(U2, C2);
            U1 = (new Vector3D(O1, P2)).Dot(new Vector3D(D1));
            P1 = global::Kerf.Engine.Geometry.ElCLib.Value(U1, C1);
            mySqDist[myNbExt] = P1.SquareDistance(P2);
            myPoint[myNbExt, 0].SetValues(U1, P1);
            myPoint[myNbExt, 1].SetValues(U2, P2);
            myNbExt++;
        }
        myDone = true;
    }

    /// <summary>The distances between a line and an ellipse.</summary>
    public Extrema_ExtElC(in Line3D C1, in Ellipse3D C2)
    {
        /*-----------------------------------------------------------------------------
        Function:
          Find extreme distances between straight line C1 and ellipse C2.

        Method:
          Let P1=C1(u1) and P2=C2(u2) two solution points
                D the direction of straight line C1
            T the tangent to point P2;
          Then, ( P1P2.D = 0. (1)
                 ( P1P2.T = 0. (2)
          Let O1 and O2 be the origins of C1 and C2;
          Then, (1) <=> (O1P2-u1*D).D = 0.        as O1P1 = u1*D
                 <=> u1 = O1P2.D              as D.D = 1.
                 (2) <=> P1O2.T = 0.              as O2P2.T = 0.
                     <=> ((P2O1.D)D+O1O2).T = 0.  as P1O1 = -u1*D = (P2O1.D)D
                 <=> (((P2O2+O2O1).D)D+O1O2).T = 0.
                 <=> ((P2O2.D)(D.T)+((O2O1.D)D-O2O1).T = 0.
          We are in the reference of the ellipse; let:
                 Cos = std::cos(u2) and Sin = std::sin(u2),
                 P2 (MajR*Cos,MinR*Sin,0.),
                 T (-MajR*Sin,MinR*Cos,0.),
             D (Dx,Dy,Dz),
             V (Vx,Vy,Vz) = (O2O1.D)D-O2O1;
          Then,  get the following equation by Cos and Sin:
            -(2*MajR*MinR*Dx*Dy)             * Cos**2  +
           (MajR*MajR*Dx**2-MinR*MinR*Dy**2) * Cos*Sin +
                 MinR*Vy                     * Cos     +
               - MajR*Vx                     * Sin     +
              MinR*MajR*Dx*Dy                = 0.
          Use algorithm math_TrigonometricFunctionRoots to solve this equation.
        -----------------------------------------------------------------------------*/
        myIsPar = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }

        // Calculate T1 the reference of the ellipse ...
        Direction3D D = C1.Direction();
        Direction3D D1 = D;
        Direction3D x2, y2, z2;
        x2 = C2.XAxis().Direction();
        y2 = C2.YAxis().Direction();
        z2 = C2.Axis().Direction();
        double Dx = D.Dot(x2);
        double Dy = D.Dot(y2);
        double Dz = D.Dot(z2);
        D.SetCoord(Dx, Dy, Dz);

        // Calculate V ...
        Point3D O1 = C1.Location();
        Point3D O2 = C2.Location();
        Vector3D O2O1 = new(O2, O1);
        O2O1.SetCoord(O2O1.Dot(new Vector3D(x2)), O2O1.Dot(new Vector3D(y2)), O2O1.Dot(new Vector3D(z2)));
        Xyz Vxyz = (D.XYZ() * (O2O1.Dot(new Vector3D(D)))) - O2O1.XYZ();

        // Calculate the coefficients of the equation by Cos and Sin ...
        double MajR = C2.MajorRadius();
        double MinR = C2.MinorRadius();
        double A5 = MajR * MinR * Dx * Dy;
        double A1 = -2.0 * A5;
        double R2 = MajR * MajR;
        double r2 = MinR * MinR;
        double A2 = (R2 * Dx * Dx - r2 * Dy * Dy - R2 + r2) / 2.0;
        double A3 = MinR * Vxyz.Y();
        double A4 = -MajR * Vxyz.X();
        //
        double aEps = 1.0e-12;
        //
        if (Math.Abs(A5) <= aEps)
        {
            A5 = 0.0;
        }
        if (Math.Abs(A1) <= aEps)
        {
            A1 = 0.0;
        }
        if (Math.Abs(A2) <= aEps)
        {
            A2 = 0.0;
        }
        if (Math.Abs(A3) <= aEps)
        {
            A3 = 0.0;
        }
        if (Math.Abs(A4) <= aEps)
        {
            A4 = 0.0;
        }
        //
        ExtremaExtElC_TrigonometricRoots Sol = new(A1, A2, A3, A4, A5, 0.0, M_PI + M_PI);
        if (!Sol.IsDone())
        {
            return;
        }
        //
        if (Sol.InfiniteRoots())
        {
            myIsPar = true;
            Point3D aP = global::Kerf.Engine.Geometry.ElCLib.EllipseValue(0.0, C2.Position(), C2.MajorRadius(), C2.MinorRadius());
            mySqDist[0] = C1.SquareDistance(aP);
            myNbExt = 1;
            myDone = true;
            return;
        }

        // Storage of solutions ...
        Point3D P1, P2;
        double U1, U2;
        int NbSol = Sol.NbSolutions();
        for (int NoSol = 1; NoSol <= NbSol; NoSol++)
        {
            U2 = Sol.Value(NoSol);
            P2 = global::Kerf.Engine.Geometry.ElCLib.Value(U2, C2);
            U1 = (new Vector3D(O1, P2)).Dot(new Vector3D(D1));
            P1 = global::Kerf.Engine.Geometry.ElCLib.Value(U1, C1);
            mySqDist[myNbExt] = P1.SquareDistance(P2);
            myPoint[myNbExt, 0] = new Extrema_POnCurv(U1, P1);
            myPoint[myNbExt, 1] = new Extrema_POnCurv(U2, P2);
            myNbExt++;
        }
        myDone = true;
    }

    /// <summary>The distances between a line and a hyperbola.</summary>
    public Extrema_ExtElC(in Line3D C1, in Hyperbola3D C2)
    {
        /*-----------------------------------------------------------------------------
        Function:
          Find extrema between straight line C1 and hyperbola C2.

        Method:
          Let P1=C1(u1) and P2=C2(u2) be two solution points
                D the direction of straight line C1
            T the tangent at point P2;
          Then, ( P1P2.D = 0. (1)
                ( P1P2.T = 0. (2)
          Let O1 and O2 be the origins of C1 and C2;
          Then, (1) <=> (O1P2-u1*D).D = 0.         as O1P1 = u1*D
                 <=> u1 = O1P2.D               as D.D = 1.
                 (2) <=> (P1O2 + O2P2).T= 0.
                     <=> ((P2O1.D)D+O1O2 + O2P2).T = 0.  as P1O1 = -u1*D = (P2O1.D)D
                 <=> (((P2O2+O2O1).D)D+O1O2 + O2P2).T = 0.
                 <=> (P2O2.D)(D.T)+((O2O1.D)D-O2O1).T + O2P2.T= 0.
          We are in the reference of the hyperbola; let:
                 by writing P (R* Chu, r* Shu, 0.0)
             and Chu = (v**2 + 1)/(2*v) ,
                 Shu = (V**2 - 1)/(2*v)

             T(R*Shu, r*Chu)
             D (Dx,Dy,Dz),
             V (Vx,Vy,Vz) = (O2O1.D)D-O2O1;

          Then we obtain the following equation by v:
                 (-2*R*r*Dx*Dy - R*R*Dx*Dx-r*r*Dy*Dy + R*R + r*r)     * v**4  +
             (2*R*Vx + 2*r*Vy)                                    * v**3  +
             (-2*R*Vx + 2*r*Vy)                                   * v     +
             (-2*R*r*Dx*Dy - (R*R*Dx*Dx-r*r*Dy*Dy + R*R + r*r))  = 0

          Use the algorithm math_DirectPolynomialRoots to solve this equation.
        -----------------------------------------------------------------------------*/
        myIsPar = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }

        // Calculate T1 in the reference of the hyperbola...
        Direction3D D = C1.Direction();
        Direction3D D1 = D;
        Direction3D x2, y2, z2;
        x2 = C2.XAxis().Direction();
        y2 = C2.YAxis().Direction();
        z2 = C2.Axis().Direction();
        double Dx = D.Dot(x2);
        double Dy = D.Dot(y2);
        double Dz = D.Dot(z2);
        D.SetCoord(Dx, Dy, Dz);

        // Calculate V ...
        Point3D O1 = C1.Location();
        Point3D O2 = C2.Location();
        Vector3D O2O1 = new(O2, O1);
        O2O1.SetCoord(O2O1.Dot(new Vector3D(x2)), O2O1.Dot(new Vector3D(y2)), O2O1.Dot(new Vector3D(z2)));
        Xyz Vxyz = (D.XYZ() * (O2O1.Dot(new Vector3D(D)))) - O2O1.XYZ();
        double Vx = Vxyz.X();
        double Vy = Vxyz.Y();

        // Calculate coefficients of the equation by v
        double R = C2.MajorRadius();
        double r = C2.MinorRadius();
        double a = -2 * R * r * Dx * Dy;
        double b = -R * R * Dx * Dx - r * r * Dy * Dy + R * R + r * r;
        double A1 = a + b;
        double A2 = 2 * R * Vx + 2 * r * Vy;
        double A4 = -2 * R * Vx + 2 * r * Vy;
        double A5 = a - b;

        DirectPolynomialRoots Sol = new(A1, A2, 0.0, A4, A5);
        if (!Sol.IsDone)
        {
            return;
        }

        // Store solutions ...
        Point3D P1, P2;
        double U1, U2, v;
        int NbSol = Sol.SolutionCount;
        for (int NoSol = 1; NoSol <= NbSol; NoSol++)
        {
            v = Sol.Value(NoSol);
            if (v > 0.0)
            {
                U2 = Math.Log(v);
                P2 = global::Kerf.Engine.Geometry.ElCLib.Value(U2, C2);
                U1 = (new Vector3D(O1, P2)).Dot(new Vector3D(D1));
                P1 = global::Kerf.Engine.Geometry.ElCLib.Value(U1, C1);
                mySqDist[myNbExt] = P1.SquareDistance(P2);
                myPoint[myNbExt, 0] = new Extrema_POnCurv(U1, P1);
                myPoint[myNbExt, 1] = new Extrema_POnCurv(U2, P2);
                myNbExt++;
            }
        }
        myDone = true;
    }

    /// <summary>The distances between a line and a parabola.</summary>
    public Extrema_ExtElC(in Line3D C1, in Parabola3D C2)
    {
        /*-----------------------------------------------------------------------------
        Function:
          Find extreme distances between straight line C1 and parabole C2.

        Method:
           Let P1=C1(u1) and P2=C2(u2) be two solution points
                D the direction of straight line C1
            T the tangent to point P2;
          Then, ( P1P2.D = 0. (1)
                ( P1P2.T = 0. (2)
          Let O1 and O2 be the origins of C1 and C2;
          Then, (1) <=> (O1P2-u1*D).D = 0.         as O1P1 = u1*D
                 <=> u1 = O1P2.D               as D.D = 1.
                 (2) <=> (P1O2 + O2P2).T= 0.
                     <=> ((P2O1.D)D+O1O2 + O2P2).T = 0.  as P1O1 = -u1*D = (P2O1.D)D
                 <=> (((P2O2+O2O1).D)D+O1O2 + O2P2).T = 0.
                 <=> (P2O2.D)(D.T)+((O2O1.D)D-O2O1).T + O2P2.T = 0.
          We are in the reference of the parabola; let:
                 P2 (y*y/(2*p), y, 0)
                 T (y/p, 1, 0)
             D (Dx,Dy,Dz),
             V (Vx,Vy,Vz) = (O2O1.D)D-O2O1;

          Then, get the following equation by y:
             ((1-Dx*Dx)/(2*p*p))            *  y*y*y  +        A1
             (-3*Dx*Dy/(2*p))               *  y*y    +        A2
             (1-Dy*Dy + Vx/p)               *  y      +        A3
                Vy                          = 0.               A4

          Use the algorithm math_DirectPolynomialRoots to solve this equation.
        -----------------------------------------------------------------------------*/
        myIsPar = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }

        // Calculate T1 in the reference of the parabola...
        Direction3D D = C1.Direction();
        Direction3D D1 = D;
        Direction3D x2, y2, z2;
        x2 = C2.XAxis().Direction();
        y2 = C2.YAxis().Direction();
        z2 = C2.Axis().Direction();
        double Dx = D.Dot(x2);
        double Dy = D.Dot(y2);
        double Dz = D.Dot(z2);
        D.SetCoord(Dx, Dy, Dz);

        // Calculate V ...
        Point3D O1 = C1.Location();
        Point3D O2 = C2.Location();
        Vector3D O2O1 = new(O2, O1);
        O2O1.SetCoord(O2O1.Dot(new Vector3D(x2)), O2O1.Dot(new Vector3D(y2)), O2O1.Dot(new Vector3D(z2)));
        Xyz Vxyz = (D.XYZ() * (O2O1.Dot(new Vector3D(D)))) - O2O1.XYZ();

        // Calculate coefficients of the equation by y
        double P = C2.Parameter();
        double A1 = (1 - Dx * Dx) / (2.0 * P * P);
        double A2 = (-3.0 * Dx * Dy / (2.0 * P));
        double A3 = (1 - Dy * Dy + Vxyz.X() / P);
        double A4 = Vxyz.Y();

        DirectPolynomialRoots Sol = new(A1, A2, A3, A4);
        if (!Sol.IsDone)
        {
            return;
        }

        // Storage of solutions ...
        Point3D P1, P2;
        double U1, U2;
        int NbSol = Sol.SolutionCount;
        for (int NoSol = 1; NoSol <= NbSol; NoSol++)
        {
            U2 = Sol.Value(NoSol);
            P2 = global::Kerf.Engine.Geometry.ElCLib.Value(U2, C2);
            U1 = (new Vector3D(O1, P2)).Dot(new Vector3D(D1));
            P1 = global::Kerf.Engine.Geometry.ElCLib.Value(U1, C1);
            mySqDist[myNbExt] = P1.SquareDistance(P2);
            myPoint[myNbExt, 0] = new Extrema_POnCurv(U1, P1);
            myPoint[myNbExt, 1] = new Extrema_POnCurv(U2, P2);
            myNbExt++;
        }
        myDone = true;
    }

    /// <summary>
    /// The distances between two circles. Only coplanar circles give a result (IsDone is false otherwise); the
    /// circles can be parallel (concentric) or identical.
    /// </summary>
    public Extrema_ExtElC(in Circle3D C1, in Circle3D C2)
    {
        bool bIsSamePlane, bIsSameAxe;
        double aTolD, aTolD2, aTolA, aD2, aDC2;
        Point3D aPc1, aPc2;
        Direction3D aDc1, aDc2;
        //
        myIsPar = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }
        //
        aTolA = global::Kerf.Engine.Foundation.Precision.Angular();
        aTolD = global::Kerf.Engine.Foundation.Precision.Confusion();
        aTolD2 = aTolD * aTolD;
        //
        aPc1 = C1.Location();
        aDc1 = C1.Axis().Direction();

        aPc2 = C2.Location();
        aDc2 = C2.Axis().Direction();
        Plane3D aPlc1 = new(aPc1, aDc1);
        //
        aD2 = aPlc1.SquareDistance(aPc2);
        bIsSamePlane = aDc1.IsParallel(aDc2, aTolA) && aD2 < aTolD2;
        if (!bIsSamePlane)
        {
            return;
        }

        // Here, both circles are in the same plane.

        //
        aDC2 = aPc1.SquareDistance(aPc2);
        bIsSameAxe = aDC2 < aTolD2;
        //
        if (bIsSameAxe)
        {
            myIsPar = true;
            myNbExt = 1;
            myDone = true;
            double aDR = C1.Radius() - C2.Radius();
            mySqDist[0] = aDR * aDR;
            return;
        }

        bool bIn, bOut;
        int j1, j2;
        double aR1, aR2, aD12, aT11, aT12, aT21, aT22;
        Circle3D aC1, aC2;
        Point3D aP11, aP12, aP21, aP22;
        //
        myDone = true;
        //
        aR1 = C1.Radius();
        aR2 = C2.Radius();
        //
        j1 = 0;
        j2 = 1;
        aC1 = C1;
        aC2 = C2;
        if (aR2 > aR1)
        {
            j1 = 1;
            j2 = 0;
            aC1 = C2;
            aC2 = C1;
        }
        //
        aR1 = aC1.Radius(); // max radius
        aR2 = aC2.Radius(); // min radius
        //
        aPc1 = aC1.Location();
        aPc2 = aC2.Location();
        //
        aD12 = aPc1.Distance(aPc2);
        Vector3D aVec12 = new(aPc1, aPc2);
        Direction3D aDir12 = new(aVec12);
        //
        // 1. Four common solutions
        myNbExt = 4;
        //
        aP11 = new Point3D(aPc1.XYZ() - aR1 * aDir12.XYZ());
        aP12 = new Point3D(aPc1.XYZ() + aR1 * aDir12.XYZ());
        aP21 = new Point3D(aPc2.XYZ() - aR2 * aDir12.XYZ());
        aP22 = new Point3D(aPc2.XYZ() + aR2 * aDir12.XYZ());
        //
        aT11 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC1, aP11);
        aT12 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC1, aP12);
        aT21 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC2, aP21);
        aT22 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC2, aP22);
        //
        // P11, P21
        myPoint[0, j1].SetValues(aT11, aP11);
        myPoint[0, j2].SetValues(aT21, aP21);
        mySqDist[0] = aP11.SquareDistance(aP21);
        // P11, P22
        myPoint[1, j1].SetValues(aT11, aP11);
        myPoint[1, j2].SetValues(aT22, aP22);
        mySqDist[1] = aP11.SquareDistance(aP22);
        //
        // P12, P21
        myPoint[2, j1].SetValues(aT12, aP12);
        myPoint[2, j2].SetValues(aT21, aP21);
        mySqDist[2] = aP12.SquareDistance(aP21);
        //
        // P12, P22
        myPoint[3, j1].SetValues(aT12, aP12);
        myPoint[3, j2].SetValues(aT22, aP22);
        mySqDist[3] = aP12.SquareDistance(aP22);
        //
        // 2. Check for intersections
        bOut = aD12 > (aR1 + aR2 + aTolD);
        bIn = aD12 < (aR1 - aR2 - aTolD);
        if (!bOut && !bIn)
        {
            bool bNbExt6;
            double aAlpha, aBeta, aVal, aDist2;
            Span<double> aT = stackalloc double[2];
            Point3D aPt, aPL1, aPL2;
            Direction3D aDLt;
            //
            aAlpha = 0.5 * (aR1 * aR1 - aR2 * aR2 + aD12 * aD12) / aD12;
            aVal = aR1 * aR1 - aAlpha * aAlpha;
            if (aVal < 0.0)
            { // see pkv/900/L4 for details
                aVal = -aVal;
            }
            aBeta = Math.Sqrt(aVal);
            aPt = new Point3D(aPc1.XYZ() + aAlpha * aDir12.XYZ());
            //
            aDLt = aDc1 ^ aDir12;
            aPL1 = new Point3D(aPt.XYZ() + aBeta * aDLt.XYZ());
            aPL2 = new Point3D(aPt.XYZ() - aBeta * aDLt.XYZ());
            //
            aDist2 = aPL1.SquareDistance(aPL2);
            bNbExt6 = aDist2 > aTolD2;
            //
            myNbExt = 5; // just in case. see pkv/900/L4 for details
            aT[j1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC1, aPL1);
            aT[j2] = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC2, aPL1);
            myPoint[4, j1].SetValues(aT[j1], aPL1);
            myPoint[4, j2].SetValues(aT[j2], aPL1);
            mySqDist[4] = 0.0;
            //
            if (bNbExt6)
            {
                myNbExt = 6;
                aT[j1] = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC1, aPL2);
                aT[j2] = global::Kerf.Engine.Geometry.ElCLib.Parameter(aC2, aPL2);
                myPoint[5, j1].SetValues(aT[j1], aPL2);
                myPoint[5, j2].SetValues(aT[j2], aPL2);
                mySqDist[5] = 0.0;
            }
            //
        } // if (!bOut || !bIn) {
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>True if the two curves are parallel.</summary>
    public bool IsParallel()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return myIsPar;
    }

    /// <summary>The number of extremum distances.</summary>
    public int NbExt()
    {
        if (!IsDone())
        {
            throw new NotDoneException();
        }
        return myNbExt;
    }

    /// <summary>The value of the Nth extremum square distance.</summary>
    public double SquareDistance(int N = 1)
    {
        if (N < 1 || N > NbExt())
        {
            throw new OutOfRangeException();
        }

        return mySqDist[N - 1];
    }

    /// <summary>The points of the Nth extremum distance: P1 on the first curve, P2 on the second.</summary>
    public void Points(int N, out Extrema_POnCurv P1, out Extrema_POnCurv P2)
    {
        if (N < 1 || N > NbExt())
        {
            throw new OutOfRangeException();
        }

        P1 = myPoint[N - 1, 0];
        P2 = myPoint[N - 1, 1];
    }

    private static void RefineDir(ref Direction3D aDir)
    {
        int i, j, k, iK;
        double aEps, aX1, aX2, aOne;
        Span<double> aCx = stackalloc double[3];
        //
        iK = 3;
        aEps = RealEpsilon();
        aDir.Coord(out aCx[0], out aCx[1], out aCx[2]);
        //
        for (i = 0; i < iK; ++i)
        {
            aOne = (aCx[i] > 0.0) ? 1.0 : -1.0;
            aX1 = aOne - aEps;
            aX2 = aOne + aEps;
            //
            if (aCx[i] > aX1 && aCx[i] < aX2)
            {
                j = (i + 1) % iK;
                k = (i + 2) % iK;
                aCx[i] = aOne;
                aCx[j] = 0.0;
                aCx[k] = 0.0;
                aDir.SetCoord(aCx[0], aCx[1], aCx[2]);
                return;
            }
        }
    }

    //=================================================================================================
    // class    : ExtremaExtElC_TrigonometricRoots
    // purpose  : Internal class (provides sorted roots of a trigonometric polynomial).
    //            Code duplicated from IntAna_IntQuadQuad.cxx.
    //            Reliable solution for near-zero coefficient problems
    //            with recovery attempt if coeff < 1.e-10.
    //=================================================================================================
    private sealed class ExtremaExtElC_TrigonometricRoots
    {
        private readonly double[] Roots = new double[4];
        private readonly bool done;
        private readonly int NbRoots;
        private readonly bool infinite_roots;

        public ExtremaExtElC_TrigonometricRoots(double CC, double SC, double C, double S, double Cte, double Binf, double Bsup)
        {
            NbRoots = 0;
            infinite_roots = false;
            int i, nbessai;
            double cc, sc, c, s, cte;
            //
            nbessai = 1;
            cc = CC;
            sc = SC;
            c = C;
            s = S;
            cte = Cte;
            done = false;
            while (nbessai <= 2 && !done)
            {
                //-- F= AA*CN*CN+2*BB*CN*SN+CC*CN+DD*SN+EE;
                TrigonometricFunctionRoots MTFR = new(cc, sc, c, s, cte, Binf, Bsup);
                //
                if (MTFR.IsDone)
                {
                    done = true;
                    if (MTFR.InfiniteRoots())
                    {
                        infinite_roots = true;
                    }
                    else
                    { // else #1
                        bool Triee;
                        int j, SvNbRoots;
                        double aTwoPI, aMaxCoef, aPrecision;
                        //
                        aTwoPI = M_PI + M_PI;
                        NbRoots = MTFR.SolutionCount;
                        for (i = 0; i < NbRoots; ++i)
                        {
                            Roots[i] = MTFR.Value(i + 1);
                            if (Roots[i] < 0.0)
                            {
                                Roots[i] = Roots[i] + aTwoPI;
                            }
                            if (Roots[i] > aTwoPI)
                            {
                                Roots[i] = Roots[i] - aTwoPI;
                            }
                        }
                        //
                        //-- The direct search gives unreliable results.
                        aMaxCoef = Max(CC, SC);
                        aMaxCoef = Max(aMaxCoef, C);
                        aMaxCoef = Max(aMaxCoef, S);
                        aMaxCoef = Max(aMaxCoef, Cte);
                        aPrecision = Max(1.0e-8, 1.0e-12 * aMaxCoef);

                        SvNbRoots = NbRoots;
                        for (i = 0; i < SvNbRoots; ++i)
                        {
                            double y;
                            double co = Math.Cos(Roots[i]);
                            double si = Math.Sin(Roots[i]);
                            y = co * (CC * co + (SC + SC) * si + C) + S * si + Cte;
                            // modified by OCC  Tue Oct  3 18:43:00 2006
                            if (Math.Abs(y) > aPrecision)
                            {
                                NbRoots--;
                                Roots[i] = 1000.0;
                            }
                        }
                        //
                        do
                        {
                            double t;
                            //
                            Triee = true;
                            for (i = 1, j = 0; i < SvNbRoots; ++i, ++j)
                            {
                                if (Roots[i] < Roots[j])
                                {
                                    Triee = false;
                                    t = Roots[i];
                                    Roots[i] = Roots[j];
                                    Roots[j] = t;
                                }
                            }
                        } while (!Triee);
                        //
                        infinite_roots = false;
                        if (NbRoots == 0)
                        { //--!!!!! Detect case Pol = Cte ( 1e-50 ) !!!!
                            if ((Math.Abs(CC) + Math.Abs(SC) + Math.Abs(C) + Math.Abs(S)) < 1e-10)
                            {
                                if (Math.Abs(Cte) < 1e-10)
                                {
                                    infinite_roots = true;
                                }
                            }
                        }
                    } // else #1
                } // if(MTFR.IsDone()) {
                else
                {
                    // try to set very small coefficients to ZERO
                    if (Math.Abs(CC) < 1e-10)
                    {
                        cc = 0.0;
                    }
                    if (Math.Abs(SC) < 1e-10)
                    {
                        sc = 0.0;
                    }
                    if (Math.Abs(C) < 1e-10)
                    {
                        c = 0.0;
                    }
                    if (Math.Abs(S) < 1e-10)
                    {
                        s = 0.0;
                    }
                    if (Math.Abs(Cte) < 1e-10)
                    {
                        cte = 0.0;
                    }
                    nbessai++;
                }
            } // while (nbessai<=2 && !done) {
        }

        public bool IsDone() => done;

        public bool IsARoot(double u)
        {
            double PIpPI, aEps;
            //
            aEps = RealEpsilon();
            PIpPI = M_PI + M_PI;
            for (int i = 0; i < NbRoots; i++)
            {
                if (Math.Abs(u - Roots[i]) <= aEps)
                {
                    return true;
                }
                if (Math.Abs(u - Roots[i] - PIpPI) <= aEps)
                {
                    return true;
                }
            }
            return false;
        }

        public int NbSolutions()
        {
            if (!done)
            {
                throw new NotDoneException();
            }
            return NbRoots;
        }

        public bool InfiniteRoots()
        {
            if (!done)
            {
                throw new NotDoneException();
            }
            return infinite_roots;
        }

        public double Value(int n)
        {
            if ((!done) || (n > NbRoots))
            {
                throw new NotDoneException();
            }
            return Roots[n - 1];
        }
    }
}
