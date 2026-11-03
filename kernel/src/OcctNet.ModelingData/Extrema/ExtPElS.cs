// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPElS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPElS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Extrema;

/// <summary>
/// All the extremum distances (minimum or maximum) between a point and an elementary surface (plane, cylinder,
/// cone, sphere or torus).
/// </summary>
/// <remarks>An algorithm object: construct it with the point and the surface, or empty and call <c>Perform</c>.</remarks>
public class ExtPElS
{
    private static readonly double ExtPElS_MyEps = Epsilon(2.0 * M_PI);

    private bool _done;
    private int _nbExt;
    private readonly double[] _sqDist = new double[4];
    private readonly POnSurf[] _point = [new(), new(), new(), new()];

    public ExtPElS()
    {
        _done = false;
        _nbExt = 0;
        for (int i = 0; i < 4; i++)
        {
            _sqDist[i] = RealLast();
        }
    }

    /// <summary>The distances between a point and a cylinder. Tol is used to test if the point is on the axis.</summary>
    public ExtPElS(in Point3D p, in Cylinder s, double tol) => Perform(p, s, tol);

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
    public void Perform(in Point3D p, in Cylinder s, double tol)
    {
        _done = false;
        _nbExt = 0;

        // Projection of point P in plane XOY of the cylinder ...
        CoordinateSystem pos = s.Position();
        Point3D o = pos.Location();
        Vector3D oz = new(pos.Direction());
        double v = new Vector3D(o, p).Dot(oz);
        Point3D pp = p.Translated(oz.Multiplied(-v));

        // Calculation of extrema
        Vector3D oPp = new(o, pp);
        if (oPp.Magnitude() < tol)
        {
            return;
        }
        Vector3D myZ = new(pos.XDirection() ^ pos.YDirection());
        double u1 = new Vector3D(pos.XDirection()).AngleWithRef(oPp, myZ); //-M_PI<U1<M_PI
        if (u1 > -ExtPElS_MyEps && u1 < ExtPElS_MyEps)
        {
            u1 = 0.0;
        }
        double u2 = u1 + M_PI;
        if (u1 < 0.0)
        {
            u1 += 2.0 * M_PI;
        }

        Point3D ps;
        ps = global::Kerf.Engine.Geometry.ElSLib.Value(u1, v, s);
        _sqDist[0] = ps.SquareDistance(p);
        _point[0] = new POnSurf(u1, v, ps);
        ps = global::Kerf.Engine.Geometry.ElSLib.Value(u2, v, s);
        _sqDist[1] = ps.SquareDistance(p);
        _point[1] = new POnSurf(u2, v, ps);

        _nbExt = 2;
        _done = true;
    }

    /// <summary>
    /// The distances between a point and a cone. Tol is used to test if the point is at the apex or on the axis.
    /// </summary>
    public ExtPElS(in Point3D p, in Cone s, double tol) => Perform(p, s, tol);

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
    public void Perform(in Point3D p, in Cone s, double tol)
    {
        _done = false;
        _nbExt = 0;

        Point3D m = s.Apex();
        CoordinateSystem pos = s.Position();
        Point3D o = pos.Location();
        double a = s.SemiAngle();
        Vector3D oz = new(pos.Direction());
        Vector3D myZ = new(pos.XDirection() ^ pos.YDirection());
        Vector3D mp = new(m, p);

        double l2 = mp.SquareMagnitude();
        double vm = -(s.RefRadius() / Math.Sin(a));

        // Case when P is mixed with S ...
        if (l2 < tol * tol)
        {
            _sqDist[0] = l2;
            _point[0] = new POnSurf(0.0, vm, m);
            _nbExt = 1;
            _done = true;
            return;
        }
        Vector3D dirZ;
        if (m.SquareDistance(o) < tol * tol)
        {
            dirZ = (a < 0 ? -oz : oz);
        }
        else
        {
            dirZ = new Vector3D(m, o);
        }

        // Projection of P in the reference plane of the cone ...
        double zp = new Vector3D(o, p).Dot(oz);

        Point3D pp = p.Translated(oz.Multiplied(-zp));
        Vector3D oPp = new(o, pp);
        if (oPp.SquareMagnitude() < tol * tol)
        {
            return;
        }
        double u1, v1, u2, v2;
        bool same = dirZ.Dot(mp) >= 0.0;
        u1 = new Vector3D(pos.XDirection()).AngleWithRef(oPp, myZ); //-M_PI<U1<M_PI
        if (u1 > -ExtPElS_MyEps && u1 < ExtPElS_MyEps)
        {
            u1 = 0.0;
        }
        if (!same)
        {
            u1 += M_PI;
        }
        u2 = u1 + M_PI;
        if (u1 < 0.0)
        {
            u1 += 2.0 * M_PI;
        }
        if (u2 > 2.0 * M_PI)
        {
            u2 -= 2.0 * M_PI;
        }
        double b = mp.Angle(dirZ);
        a = Math.Abs(a);
        double l = Math.Sqrt(l2);
        if (!same)
        {
            b = M_PI - b;
            v1 = -l * Math.Cos(b - a);
            v2 = -l * Math.Cos(b + a);
        }
        else
        {
            v1 = l * Math.Cos(b - a);
            v2 = l * Math.Cos(b + a);
        }
        double sense = oz.Dot(new Vector3D(new Direction3D(dirZ)));
        v1 *= sense;
        v2 *= sense;
        v1 += vm;
        v2 += vm;

        Point3D ps;
        ps = global::Kerf.Engine.Geometry.ElSLib.Value(u1, v1, s);
        _sqDist[0] = ps.SquareDistance(p);
        _point[0] = new POnSurf(u1, v1, ps);
        ps = global::Kerf.Engine.Geometry.ElSLib.Value(u2, v2, s);
        _sqDist[1] = ps.SquareDistance(p);
        _point[1] = new POnSurf(u2, v2, ps);

        _nbExt = 2;
        _done = true;
    }

    /// <summary>The distances between a point and a sphere. Tol is used to test if the point is at the center.</summary>
    public ExtPElS(in Point3D p, in Sphere s, double tol) => Perform(p, s, tol);

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
    public void Perform(in Point3D p, in Sphere s, double tol)
    {
        _done = false;
        _nbExt = 0;

        CoordinateSystem pos = s.Position();
        Vector3D op = new(pos.Location(), p);

        // Case when P is mixed with O ...
        if (op.SquareMagnitude() < tol * tol)
        {
            return;
        }

        // Projection if P in plane XOY of the sphere ...
        Point3D o = pos.Location();
        Vector3D oz = new(pos.Direction());
        double zp = op.Dot(oz);
        Point3D pp = p.Translated(oz.Multiplied(-zp));

        // Calculation of extrema ...
        Vector3D oPp = new(o, pp);
        double u1, u2, v;
        if (oPp.SquareMagnitude() < tol * tol)
        {
            u1 = 0.0;
            u2 = 0.0;
            if (zp < 0.0)
            {
                v = -M_PI / 2.0;
            }
            else
            {
                v = M_PI / 2.0;
            }
        }
        else
        {
            Vector3D myZ = new(pos.XDirection() ^ pos.YDirection());
            u1 = new Vector3D(pos.XDirection()).AngleWithRef(oPp, myZ);
            if (u1 > -ExtPElS_MyEps && u1 < ExtPElS_MyEps)
            {
                u1 = 0.0;
            }
            u2 = u1 + M_PI;
            if (u1 < 0.0)
            {
                u1 += 2.0 * M_PI;
            }
            v = op.Angle(oPp);
            if (zp < 0.0)
            {
                v = -v;
            }
        }

        Point3D ps;
        ps = global::Kerf.Engine.Geometry.ElSLib.Value(u1, v, s);
        _sqDist[0] = ps.SquareDistance(p);
        _point[0] = new POnSurf(u1, v, ps);
        ps = global::Kerf.Engine.Geometry.ElSLib.Value(u2, -v, s);
        _sqDist[1] = ps.SquareDistance(p);
        _point[1] = new POnSurf(u2, -v, ps);

        _nbExt = 2;
        _done = true;
    }

    /// <summary>The distances between a point and a torus. Tol is used to test if the point is on the axis.</summary>
    public ExtPElS(in Point3D p, in Torus s, double tol) => Perform(p, s, tol);

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
    public void Perform(in Point3D p, in Torus s, double tol)
    {
        double tol2 = tol * tol;
        _done = false;
        _nbExt = 0;

        // Projection of P in plane XOY ...
        CoordinateSystem pos = s.Position();
        Point3D o = pos.Location();
        Vector3D oz = new(pos.Direction());
        Point3D pp = p.Translated(oz.Multiplied(-(new Vector3D(o, p).Dot(new Vector3D(pos.Direction())))));

        // Calculation of extrema ...
        Vector3D oPp = new(o, pp);
        double r2 = oPp.SquareMagnitude();
        if (r2 < tol2)
        {
            return;
        }

        Vector3D myZ = new(pos.XDirection() ^ pos.YDirection());
        double u1 = new Vector3D(pos.XDirection()).AngleWithRef(oPp, myZ);
        if (u1 > -ExtPElS_MyEps && u1 < ExtPElS_MyEps)
        {
            u1 = 0.0;
        }
        double u2 = u1 + M_PI;
        if (u1 < 0.0)
        {
            u1 += 2.0 * M_PI;
        }
        double r = Math.Sqrt(r2);
        Vector3D oo1 = oPp.Divided(r).Multiplied(s.MajorRadius());
        Vector3D oo2 = oo1.Multiplied(-1.0);
        Point3D o1 = o.Translated(oo1);
        Point3D o2 = o.Translated(oo2);

        if (o1.SquareDistance(p) < tol2)
        {
            return;
        }
        if (o2.SquareDistance(p) < tol2)
        {
            return;
        }

        double v1 = oPp.AngleWithRef(new Vector3D(o1, p), oPp.Crossed(oz));
        if (v1 > -ExtPElS_MyEps && v1 < ExtPElS_MyEps)
        {
            v1 = 0.0;
        }
        oPp.Reverse();
        double v2 = oPp.AngleWithRef(new Vector3D(p, o2), oPp.Crossed(oz));
        if (v2 > -ExtPElS_MyEps && v2 < ExtPElS_MyEps)
        {
            v2 = 0.0;
        }

        if (v1 < 0.0)
        {
            v1 += 2.0 * M_PI;
        }
        if (v2 < 0.0)
        {
            v2 += 2.0 * M_PI;
        }

        Point3D ps;
        ps = global::Kerf.Engine.Geometry.ElSLib.Value(u1, v1, s);
        _sqDist[0] = ps.SquareDistance(p);
        _point[0] = new POnSurf(u1, v1, ps);

        ps = global::Kerf.Engine.Geometry.ElSLib.Value(u1, v1 + M_PI, s);
        _sqDist[1] = ps.SquareDistance(p);
        _point[1] = new POnSurf(u1, v1 + M_PI, ps);

        ps = global::Kerf.Engine.Geometry.ElSLib.Value(u2, v2, s);
        _sqDist[2] = ps.SquareDistance(p);
        _point[2] = new POnSurf(u2, v2, ps);

        ps = global::Kerf.Engine.Geometry.ElSLib.Value(u2, v2 + M_PI, s);
        _sqDist[3] = ps.SquareDistance(p);
        _point[3] = new POnSurf(u2, v2 + M_PI, ps);

        _nbExt = 4;
        _done = true;
    }

    /// <summary>The distance between a point and a plane (its projection). Tol is not used.</summary>
    public ExtPElS(in Point3D p, in Plane3D s, double tol) => Perform(p, s, tol);

    public void Perform(in Point3D p, in Plane3D s, double tol)
    {
        _ = tol; // OCCT: the tolerance is not used
        _done = false;
        _nbExt = 0;

        // Projection of point P in plane XOY of the cylinder ...
        Point3D o = s.Location();
        Vector3D oz = new(s.Axis().Direction());
        double u, v = new Vector3D(o, p).Dot(oz);
        Point3D pp = p.Translated(oz.Multiplied(-v));

        global::Kerf.Engine.Geometry.ElSLib.Parameters(s, p, out u, out v);
        _sqDist[0] = pp.SquareDistance(p);
        _point[0] = new POnSurf(u, v, pp);
        _nbExt = 1;
        _done = true;
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone => _done;

    /// <summary>The number of extremum distances.</summary>
    public int ExtCount
    {
        get
        {
            if (!IsDone)
            {
                throw new NotDoneException();
            }
            return _nbExt;
        }
    }

    /// <summary>The value of the Nth resulting square distance.</summary>
    public double SquareDistance(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _sqDist[n - 1];
    }

    /// <summary>The point of the Nth resulting distance.</summary>
    public POnSurf Point(int n)
    {
        if ((n < 1) || (n > ExtCount))
        {
            throw new OutOfRangeException();
        }
        return _point[n - 1];
    }
}
