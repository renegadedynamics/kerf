// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntConicQuad.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntConicQuad.lxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_IntConicQuad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.IntAna_ResultType;

namespace Occt;

/// <summary>
/// The analytic intersection between a conic (line, circle, ellipse, parabola or hyperbola of gp) and a quadric
/// (<see cref="IntAna_Quadric"/>). The intersection between a conic and a plane is treated as a special case. The
/// results are points, with their parameters on the conic.
/// </summary>
/// <remarks>An algorithm object: construct it with the arguments, or empty and call <c>Perform</c>.</remarks>
public class IntAna_IntConicQuad
{
    private static readonly double PIpPI = M_PI + M_PI;

    private bool done;
    private bool parallel;
    private bool inquadric;
    private int nbpts;
    private readonly Point3D[] pnts = new Point3D[4];
    private readonly double[] paramonc = new double[4];

    //==                                          E m p t y   C o n s t r u c t o r
    public IntAna_IntConicQuad()
    {
        done = false;
        parallel = false;
        inquadric = false;
        nbpts = 0;
    }

    //==                                                 L i n e  -   Q u a d r i c
    public IntAna_IntConicQuad(in Line3D L, IntAna_Quadric Quad)
    {
        Perform(L, Quad);
    }

    public void Perform(in Line3D L, IntAna_Quadric Quad)
    {
        double Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, QCte;
        done = inquadric = parallel = false;

        //----------------------------------------------------------------------
        //-- Substitution of x=t Lx + Lx0       ( expressed in                 )
        //--                  y=t Ly + Ly0       (  the canonical coordinate   )
        //--                  z=t Lz + Lz0       (  system                     )
        //--
        //-- Into     Qxx x**2 + Qyy y**2 + Qzz z**2
        //--          + 2 ( Qxy x y  + Qxz x z  + Qyz y z  )
        //--          + 2 ( Qx x + Qy y + Qz z )
        //--          + QCte
        //--
        //-- Gives a polynomial in t: A2 t**2 + A1 t + A0 with:
        //----------------------------------------------------------------------

        double Lx0, Ly0, Lz0, Lx, Ly, Lz;

        nbpts = 0;

        L.Direction().Coord(out Lx, out Ly, out Lz);
        L.Location().Coord(out Lx0, out Ly0, out Lz0);

        Quad.Coefficients(out Qxx, out Qyy, out Qzz, out Qxy, out Qxz, out Qyz, out Qx, out Qy, out Qz, out QCte);

        double A0 = (QCte + Qxx * Lx0 * Lx0 + Qyy * Ly0 * Ly0 + Qzz * Lz0 * Lz0
                     + 2.0 * (Lx0 * (Qx + Qxy * Ly0 + Qxz * Lz0) + Ly0 * (Qy + Qyz * Lz0) + Qz * Lz0));

        double A1 =
            2.0
            * (Lx * (Qx + Qxx * Lx0 + Qxy * Ly0 + Qxz * Lz0) + Ly * (Qy + Qxy * Lx0 + Qyy * Ly0 + Qyz * Lz0)
               + Lz * (Qz + Qxz * Lx0 + Qyz * Ly0 + Qzz * Lz0));

        double A2 = (Qxx * Lx * Lx + Qyy * Ly * Ly + Qzz * Lz * Lz
                     + 2.0 * (Lx * (Qxy * Ly + Qxz * Lz) + Qyz * Ly * Lz));

        DirectPolynomialRoots LinQuadPol = new(A2, A1, A0);

        if (LinQuadPol.IsDone)
        {
            done = true;
            if (LinQuadPol.InfiniteRoots())
            {
                inquadric = true;
            }
            else
            {
                nbpts = LinQuadPol.SolutionCount;

                for (int i = 1; i <= nbpts; i++)
                {
                    double t = LinQuadPol.Value(i);
                    paramonc[i - 1] = t;
                    pnts[i - 1] = new Point3D(Lx0 + Lx * t, Ly0 + Ly * t, Lz0 + Lz * t);
                }
            }
        }
    }

    //==                                            C i r c l e   -   Q u a d r i c
    public IntAna_IntConicQuad(in Circle3D C, IntAna_Quadric Quad)
    {
        Perform(C, Quad);
    }

    public void Perform(in Circle3D C, IntAna_Quadric Quad)
    {
        double Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, QCte;

        //----------------------------------------------------------------------
        //-- In the reference frame associated with C.Position():
        //-- xC = R * Cos[t]
        //-- yC = R * Sin[t]
        //-- zC = 0
        //--
        //-- We express the quadric in this frame and substitute
        //-- xC, yC and zC for x, y and z
        //--
        //-- We obtain a polynomial in Cos[t] and Sin[t] of degree 2
        //----------------------------------------------------------------------
        done = inquadric = parallel = false;

        Quad.Coefficients(out Qxx, out Qyy, out Qzz, out Qxy, out Qxz, out Qyz, out Qx, out Qy, out Qz, out QCte);
        Quad.NewCoefficients(ref Qxx, ref Qyy, ref Qzz, ref Qxy, ref Qxz, ref Qyz, ref Qx, ref Qy, ref Qz, ref QCte,
                             new CoordinateSystem(C.Position()));

        double R = C.Radius();
        double RR = R * R;

        double P_CosCos = RR * Qxx; //-- Cos Cos
        double P_SinSin = RR * Qyy; //-- Sin Sin
        double P_Sin = R * Qy;      //-- 2 Sin
        double P_Cos = R * Qx;      //-- 2 Cos
        double P_CosSin = RR * Qxy; //-- 2 Cos Sin
        double P_Cte = QCte;        //-- 1

        TrigonometricFunctionRoots CircQuadPol = new(P_CosCos - P_SinSin,
                                                          P_CosSin,
                                                          P_Cos + P_Cos,
                                                          P_Sin + P_Sin,
                                                          P_Cte + P_SinSin,
                                                          0.0,
                                                          PIpPI);

        if (CircQuadPol.IsDone)
        {
            done = true;
            if (CircQuadPol.InfiniteRoots())
            {
                inquadric = true;
            }
            else
            {
                nbpts = CircQuadPol.SolutionCount;

                for (int i = 1; i <= nbpts; i++)
                {
                    double t = CircQuadPol.Value(i);
                    paramonc[i - 1] = t;
                    pnts[i - 1] = global::Kerf.Engine.Geometry.ElCLib.CircleValue(t, C.Position(), R);
                }
            }
        }
    }

    //==                                                  E l i p s - Q u a d r i c
    public IntAna_IntConicQuad(in Ellipse3D E, IntAna_Quadric Quad)
    {
        Perform(E, Quad);
    }

    public void Perform(in Ellipse3D E, IntAna_Quadric Quad)
    {
        double Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, QCte;

        done = inquadric = parallel = false;

        //----------------------------------------------------------------------
        //-- In the reference frame associated with E.Position():
        //-- xE = R * Cos[t]
        //-- yE = r * Sin[t]
        //-- zE = 0
        //--
        //-- We express the quadric in this frame and substitute
        //-- xE, yE and zE for x, y and z
        //--
        //-- We obtain a polynomial in Cos[t] and Sin[t] of degree 2
        //----------------------------------------------------------------------

        Quad.Coefficients(out Qxx, out Qyy, out Qzz, out Qxy, out Qxz, out Qyz, out Qx, out Qy, out Qz, out QCte);
        Quad.NewCoefficients(ref Qxx, ref Qyy, ref Qzz, ref Qxy, ref Qxz, ref Qyz, ref Qx, ref Qy, ref Qz, ref QCte,
                             new CoordinateSystem(E.Position()));

        double R = E.MajorRadius();
        double r = E.MinorRadius();

        double P_CosCos = R * R * Qxx; //-- Cos Cos
        double P_SinSin = r * r * Qyy; //-- Sin Sin
        double P_Sin = r * Qy;         //-- 2 Sin
        double P_Cos = R * Qx;         //-- 2 Cos
        double P_CosSin = R * r * Qxy; //-- 2 Cos Sin
        double P_Cte = QCte;           //-- 1

        TrigonometricFunctionRoots ElipsQuadPol = new(P_CosCos - P_SinSin,
                                                           P_CosSin,
                                                           P_Cos + P_Cos,
                                                           P_Sin + P_Sin,
                                                           P_Cte + P_SinSin,
                                                           0.0,
                                                           PIpPI);

        if (ElipsQuadPol.IsDone)
        {
            done = true;
            if (ElipsQuadPol.InfiniteRoots())
            {
                inquadric = true;
            }
            else
            {
                nbpts = ElipsQuadPol.SolutionCount;
                for (int i = 1; i <= nbpts; i++)
                {
                    double t = ElipsQuadPol.Value(i);
                    paramonc[i - 1] = t;
                    pnts[i - 1] = global::Kerf.Engine.Geometry.ElCLib.EllipseValue(t, E.Position(), R, r);
                }
            }
        }
    }

    //==                                                  P a r a b - Q u a d r i c
    public IntAna_IntConicQuad(in Parabola3D P, IntAna_Quadric Quad)
    {
        Perform(P, Quad);
    }

    public void Perform(in Parabola3D P, IntAna_Quadric Quad)
    {
        double Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, QCte;

        done = inquadric = parallel = false;

        //----------------------------------------------------------------------
        //-- In the reference frame associated with P.Position():
        //-- xP = y*y / (2 p)
        //-- yP = y
        //-- zP = 0
        //--
        //-- We express the quadric in this frame and substitute
        //-- xP, yP and zP for x, y and z
        //--
        //-- We obtain a polynomial in y of degree 4
        //----------------------------------------------------------------------

        Quad.Coefficients(out Qxx, out Qyy, out Qzz, out Qxy, out Qxz, out Qyz, out Qx, out Qy, out Qz, out QCte);
        Quad.NewCoefficients(ref Qxx, ref Qyy, ref Qzz, ref Qxy, ref Qxz, ref Qyz, ref Qx, ref Qy, ref Qz, ref QCte,
                             new CoordinateSystem(P.Position()));

        double f = P.Focal();
        double Un_Sur_2p = 0.25 / f;

        double A4 = Qxx * Un_Sur_2p * Un_Sur_2p;
        double A3 = (Qxy + Qxy) * Un_Sur_2p;
        double A2 = Qyy + (Qx + Qx) * Un_Sur_2p;
        double A1 = Qy + Qy;
        double A0 = QCte;

        DirectPolynomialRoots ParabQuadPol = new(A4, A3, A2, A1, A0);

        if (ParabQuadPol.IsDone)
        {
            done = true;
            if (ParabQuadPol.InfiniteRoots())
            {
                inquadric = true;
            }
            else
            {
                nbpts = ParabQuadPol.SolutionCount;

                for (int i = 1; i <= nbpts; i++)
                {
                    double t = ParabQuadPol.Value(i);
                    paramonc[i - 1] = t;
                    pnts[i - 1] = global::Kerf.Engine.Geometry.ElCLib.ParabolaValue(t, P.Position(), f);
                }
            }
        }
    }

    //==                                                    H y p r - Q u a d r i c
    public IntAna_IntConicQuad(in Hyperbola3D H, IntAna_Quadric Quad)
    {
        Perform(H, Quad);
    }

    public void Perform(in Hyperbola3D H, IntAna_Quadric Quad)
    {
        double Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, QCte;

        done = inquadric = parallel = false;

        //----------------------------------------------------------------------
        //-- In the reference frame associated with H.Position():
        //-- xH = R Ch[t]
        //-- yH = r Sh[t]
        //-- zH = 0
        //--
        //-- We express the quadric in this frame and substitute
        //-- xH, yH and zH for x, y and z
        //--
        //-- We obtain a polynomial in Exp[t] of degree 4
        //----------------------------------------------------------------------

        Quad.Coefficients(out Qxx, out Qyy, out Qzz, out Qxy, out Qxz, out Qyz, out Qx, out Qy, out Qz, out QCte);
        Quad.NewCoefficients(ref Qxx, ref Qyy, ref Qzz, ref Qxy, ref Qxz, ref Qyz, ref Qx, ref Qy, ref Qz, ref QCte,
                             new CoordinateSystem(H.Position()));

        double R = H.MajorRadius();
        double r = H.MinorRadius();

        double RR = R * R;
        double rr = r * r;
        double Rr = R * r;

        double A4 = RR * Qxx + Rr * (Qxy + Qxy) + rr * Qyy;
        double A3 = 4.0 * (R * Qx + r * Qy);
        double A2 = 2.0 * ((QCte + QCte) + Qxx * RR - Qyy * rr);
        double A1 = 4.0 * (R * Qx - r * Qy);
        double A0 = Qxx * RR - Rr * (Qxy + Qxy) + Qyy * rr;

        DirectPolynomialRoots HyperQuadPol = new(A4, A3, A2, A1, A0);

        if (HyperQuadPol.IsDone)
        {
            done = true;
            if (HyperQuadPol.InfiniteRoots())
            {
                inquadric = true;
            }
            else
            {
                nbpts = HyperQuadPol.SolutionCount;
                int bonnessolutions = 0;
                for (int i = 1; i <= nbpts; i++)
                {
                    double t = HyperQuadPol.Value(i);
                    if (t >= RealEpsilon())
                    {
                        double Lnt = Math.Log(t);
                        paramonc[bonnessolutions] = Lnt;
                        pnts[bonnessolutions] = global::Kerf.Engine.Geometry.ElCLib.HyperbolaValue(Lnt, H.Position(), R, r);
                        bonnessolutions++;
                    }
                }
                nbpts = bonnessolutions;
            }
        }
    }

    /// <summary>
    /// The intersection between a line and a plane. Tolang is the angular tolerance below which the line is
    /// parallel to the plane. When Len (the length of a bounded line) is not 0, a nearly parallel line is only
    /// parallel if its end point at Len is within Tol of the plane.
    /// </summary>
    public IntAna_IntConicQuad(in Line3D L, in Plane3D P, double Tolang, double Tol = 0, double Len = 0)
    {
        Perform(L, P, Tolang, Tol, Len);
    }

    /// <summary>The intersection between a circle and a plane.</summary>
    public IntAna_IntConicQuad(in Circle3D C, in Plane3D P, double Tolang, double Tol)
    {
        Perform(C, P, Tolang, Tol);
    }

    /// <summary>The intersection between an ellipse and a plane (the tolerances are not used).</summary>
    public IntAna_IntConicQuad(in Ellipse3D E, in Plane3D P, double Tolang, double Tol)
    {
        Perform(E, P, Tolang, Tol);
    }

    /// <summary>The intersection between a parabola and a plane (the tolerance is not used).</summary>
    public IntAna_IntConicQuad(in Parabola3D Pb, in Plane3D P, double Tolang)
    {
        Perform(Pb, P, Tolang);
    }

    /// <summary>The intersection between a hyperbola and a plane (the tolerance is not used).</summary>
    public IntAna_IntConicQuad(in Hyperbola3D H, in Plane3D P, double Tolang)
    {
        Perform(H, P, Tolang);
    }

    public void Perform(in Line3D L, in Plane3D P, double Tolang, double Tol = 0, double Len = 0)
    {
        // Tolang represents the angular tolerance from which we consider
        // that the angle between 2 vectors is zero. We will reason about the cosine of this
        // angle, (we have std::cos(t) equivalent to t near Pi/2).

        done = false;

        double A, B, C, D;
        double Al, Bl, Cl;
        double Dis, Direc;

        P.Coefficients(out A, out B, out C, out D);
        Point3D Orig = L.Location();
        L.Direction().Coord(out Al, out Bl, out Cl);

        Direc = A * Al + B * Bl + C * Cl;
        Dis = A * Orig.X() + B * Orig.Y() + C * Orig.Z() + D;
        //
        parallel = false;
        if (Math.Abs(Direc) < Tolang)
        {
            parallel = true;
            if (Len != 0 && Direc != 0)
            {
                // check the distance from bounding point of the line to the plane
                Point3D aP1 = new(), aP2 = new();
                //
                aP1.SetCoord(Orig.X() - Dis * A, Orig.Y() - Dis * B, Orig.Z() - Dis * C);
                aP2.SetCoord(aP1.X() + Len * Al, aP1.Y() + Len * Bl, aP1.Z() + Len * Cl);
                if (P.Distance(aP2) > Tol)
                {
                    parallel = false;
                }
            }
        }
        if (parallel)
        {
            inquadric = Math.Abs(Dis) < Tolang;
        }
        else
        {
            parallel = false;
            inquadric = false;
            nbpts = 1;
            paramonc[0] = -Dis / Direc;
            pnts[0].SetCoord(Orig.X() + paramonc[0] * Al,
                             Orig.Y() + paramonc[0] * Bl,
                             Orig.Z() + paramonc[0] * Cl);
        }
        done = true;
    }

    public void Perform(in Circle3D C, in Plane3D P, double Tolang, double Tol)
    {
        done = false;

        Plane3D Plconic = new(new CoordinateSystem(C.Position()));
        IntAna_QuadQuadGeo IntP = new(Plconic, P, Tolang, Tol);
        if (!IntP.IsDone())
        {
            return;
        }
        if (IntP.TypeInter() == IntAna_Empty)
        {
            parallel = true;
            double distmax = P.Distance(C.Location()) + C.Radius() * Tolang;
            inquadric = distmax < Tol;
            done = true;
        }
        else if (IntP.TypeInter() == IntAna_Same)
        {
            inquadric = true;
            done = true;
        }
        else
        {
            inquadric = false;
            parallel = false;
            Line3D Ligsol = IntP.Line(1);

            Vector3D V0 = new(Plconic.Location(), Ligsol.Location());
            Vector3D Axex = new(Plconic.Position().XDirection());
            Vector3D Axey = new(Plconic.Position().YDirection());

            Point2D Orig = new(Axex.Dot(V0), Axey.Dot(V0));
            Vector2D Dire = new(Axex.Dot(new Vector3D(Ligsol.Direction())), Axey.Dot(new Vector3D(Ligsol.Direction())));

            Line2D Ligs = new(Orig, new Direction2D(Dire));
            Point2D Pnt2dBid = new(0.0, 0.0);
            Direction2D Dir2dBid = new(Direction2D.D.X);
            Axis2D Ax2dBid = new(Pnt2dBid, Dir2dBid);
            Circle2D Cir = new(Ax2dBid, C.Radius());

            IntAna2d_AnaIntersection Int2d = new(Ligs, Cir);

            if (!Int2d.IsDone())
            {
                return;
            }

            nbpts = Int2d.NbPoints();
            for (int i = 1; i <= nbpts; i++)
            {
                Point2D resul = Int2d.Point(i).Value();
                double X = resul.X();
                double Y = resul.Y();
                pnts[i - 1].SetCoord(Plconic.Location().X() + X * Axex.X() + Y * Axey.X(),
                                     Plconic.Location().Y() + X * Axex.Y() + Y * Axey.Y(),
                                     Plconic.Location().Z() + X * Axex.Z() + Y * Axey.Z());
                paramonc[i - 1] = Int2d.Point(i).ParamOnSecond();
            }
            done = true;
        }
    }

    public void Perform(in Ellipse3D E, in Plane3D Pln, double Tolang, double Tol)
    {
        _ = Tolang;
        _ = Tol;
        // OCCT converts the plane implicitly through IntAna_Quadric's constructor
        Perform(E, new IntAna_Quadric(Pln));
    }

    public void Perform(in Parabola3D P, in Plane3D Pln, double Tolang)
    {
        _ = Tolang;
        Perform(P, new IntAna_Quadric(Pln));
    }

    public void Perform(in Hyperbola3D H, in Plane3D Pln, double Tolang)
    {
        _ = Tolang;
        Perform(H, new IntAna_Quadric(Pln));
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone() => done;

    /// <summary>True if the conic lies in the quadric.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public bool IsInQuadric()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return inquadric;
    }

    /// <summary>True if the line is parallel to the plane, or the circle's plane parallel to the plane.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public bool IsParallel()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return parallel;
    }

    /// <summary>The number of intersection points.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">The conic is parallel or in the quadric.</exception>
    public int NbPoints()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        if (parallel || inquadric)
        {
            throw new DomainErrorException();
        }
        return nbpts;
    }

    /// <summary>The intersection point of index i (from 1).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">The conic is parallel or in the quadric.</exception>
    /// <exception cref="OutOfRangeException">i is not in [1, NbPoints].</exception>
    public Point3D Point(int i)
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        if (parallel || inquadric)
        {
            throw new DomainErrorException();
        }
        if ((i > nbpts) || (i <= 0))
        {
            throw new OutOfRangeException();
        }
        return pnts[i - 1];
    }

    /// <summary>The parameter on the conic of the intersection point of index i (from 1).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    /// <exception cref="DomainErrorException">The conic is parallel or in the quadric.</exception>
    /// <exception cref="OutOfRangeException">i is not in [1, NbPoints].</exception>
    public double ParamOnConic(int i)
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        if (parallel || inquadric)
        {
            throw new DomainErrorException();
        }
        if ((i > nbpts) || (i <= 0))
        {
            throw new OutOfRangeException();
        }
        return paramonc[i - 1];
    }
}
