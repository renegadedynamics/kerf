// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Surface.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Surface.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeCustom/ShapeCustom_Surface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Healing.Analysis;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Healing.Custom;

/// <summary>
/// Converts a B-spline or Bezier surface to an analytical surface (plane, cylinder, cone, sphere, torus) where one
/// fits within a tolerance, or a closed B-spline surface to its periodic form.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
internal sealed class SurfaceCustom
{
    private Surface? _surf;
    private double _gap;

    /// <summary>An empty tool.</summary>
    public SurfaceCustom()
    {
        _gap = 0;
    }

    /// <summary>A tool on the surface.</summary>
    public SurfaceCustom(Surface? s)
    {
        _gap = 0;
        Init(s);
    }

    /// <summary>Sets the surface.</summary>
    public void Init(Surface? s)
    {
        _surf = s;
    }

    /// <summary>The largest deviation of the converted surface from the original found by the last <see cref="ConvertToAnalytical"/>.</summary>
    public double Gap()
    {
        return _gap;
    }

    /// <summary>
    /// Tries to convert the surface (a B-spline or Bezier) to an analytic one within <paramref name="tol"/>, and
    /// returns it, or null. With <paramref name="substitute"/>, the result replaces the tool's surface. The
    /// parametrisation isn't kept, so pcurves must be redone.
    /// </summary>
    public Surface? ConvertToAnalytical(double tol, bool substitute)
    {
        Surface? newSurf = null;

        int nUP, nVP, nCP, i, j, uDeg, vDeg;
        double u1, u2, v1, v2, c1, c2, du, dv, u = 0, v = 0;
        Curve3D? iso = null;
        bool uClosed = true;

        // seuls cas traites : BSpline et Bezier
        global::Kerf.Geometry.BSplineSurface? theBSplneS = _surf as global::Kerf.Geometry.BSplineSurface;
        if (theBSplneS is null)
        {
            global::Kerf.Geometry.BezierSurface? theBezierS = _surf as global::Kerf.Geometry.BezierSurface;
            if (theBezierS is not null)
            { // Bezier :
                nUP = theBezierS.UPoleCount;
                nVP = theBezierS.VPoleCount;
                uDeg = theBezierS.UDegree;
                vDeg = theBezierS.VDegree;
            }
            else
            {
                return newSurf; // non reconnu : terminus
            }
        }
        else
        { // BSpline :
            nUP = theBSplneS.UPoleCount;
            nVP = theBSplneS.VPoleCount;
            uDeg = theBSplneS.UDegree;
            vDeg = theBSplneS.VDegree;
        }

        _surf!.ParameterBounds(out u1, out u2, out v1, out v2);
        NCollection_Array1<Point3D> p1 = new(1, 3), p2 = new(1, 3), p3 = new(1, 3);
        NCollection_Array1<double> r = new(1, 3);
        Point3D origPnt, resPnt;
        Vector3D origD1U = new Vector3D(), resD1U, resD1V;

        bool cySpCo = false;
        bool toroid = false;
        bool planar = false;

        if (nUP == 2 && nVP == 2)
        {
            if (uDeg == 1 && vDeg == 1)
            {
                planar = true;
            }
        }
        else if (_surf.IsUClosed)
        { // VRAI IsUClosed
            if (_surf.IsVClosed)
            {
                toroid = true;
            }
            else
            {
                cySpCo = true;
            }
        }
        else
        {
            if (_surf.IsVClosed)
            { // VRAI IsVClosed
                cySpCo = true;
                uClosed = false;
            }
        }

        if (planar)
        {
            //    NearestPlane ...
            NCollection_Array1<Point3D> pnts = new(1, 4);
            pnts.SetValue(1, _surf.Value(u1, v1));
            pnts.SetValue(2, _surf.Value(u2, v1));
            pnts.SetValue(3, _surf.Value(u1, v2));
            pnts.SetValue(4, _surf.Value(u2, v2));
            Plane3D pln = new Plane3D(); // double Dmax;
            bool it = GeomAnalysis.NearestPlane(pnts, ref pln, out _gap /*Dmax*/);

            //  ICI, on fabrique le plan, et zou
            if (!it || _gap /*Dmax*/ > tol)
            {
                return newSurf; //  pas un plan
            }

            //    IL RESTE a verifier l orientation ...
            //    On regarde sur chaque surface les vecteurs P(U0->U1),P(V0->V1)
            //    On prend la normale : les deux normales doivent etre dans le meme sens
            //    Sinon, inverser la normale (pas le Pln entier !) et refaire la Plane
            newSurf = new PlaneSurface(pln);
            Vector3D uold = new Vector3D(pnts[1], pnts[2]);
            Vector3D vold = new Vector3D(pnts[1], pnts[3]);
            Vector3D nold = uold.Crossed(vold);
            Vector3D unew = new Vector3D(newSurf.Value(u1, v1), newSurf.Value(u2, v1));
            Vector3D vnew = new Vector3D(newSurf.Value(u1, v1), newSurf.Value(u1, v2));
            Vector3D nnew = unew.Crossed(vnew);
            if (nold.Dot(nnew) < 0.0)
            {
                CoordinateSystem ax3 = pln.Position;
                ax3 = ax3.ZReversed();
                ax3 = ax3.XReversed();
                pln = new Plane3D(ax3);
                newSurf = new PlaneSurface(pln);
            }

            if (substitute)
            {
                Init(newSurf);
            }
            return newSurf;
        }
        else if (cySpCo)
        {
            if (!uClosed)
            {
                c1 = u1;
                c2 = u2;
                u1 = v1;
                u2 = v2;
                v1 = c1;
                v2 = c2;
                nCP = nUP;
                nUP = nVP;
                nVP = nCP;
            }

            for (i = 1; i <= 3; i++)
            {
                if (i == 1)
                {
                    v = v1;
                }
                else if (i == 2)
                {
                    v = v2;
                }
                else if (i == 3)
                {
                    v = 0.5 * (v1 + v2);
                }

                if (uClosed)
                {
                    iso = _surf.VIso(v);
                }
                else
                {
                    iso = _surf.UIso(v);
                }

                iso.D0(u1, out p1[i]);
                iso.D0(0.5 * (u1 + u2), out p2[i]);
                p3[i] = p3[i].WithCoord(0.5 * (p1[i].X + p2[i].X),
                               0.5 * (p1[i].Y + p2[i].Y),
                               0.5 * (p1[i].Z + p2[i].Z));
                r[i] = p3[i].DistanceTo(p1[i]);
            }

            // OCCT: the last iso built, at parameter 0 rather than U1 (occt-issues.md)
            iso!.D1(0.0, out origPnt, out origD1U);
            Vector3D xVec = new Vector3D(p3[3], p1[3]);
            Vector3D vec = new Vector3D(p3[1], p3[2]);
            //      gp_Dir xDir(xVec);  ne sert pas. Null si R3 = 0
            Direction3D dir = new Direction3D(vec);
            CoordinateSystem aAx3 = new CoordinateSystem(p3[1], dir, new Direction3D(xVec));
            //  CKY  3-FEV-1997 : verification du sens de description
            // C++ converts origD1U to a gp_Dir implicitly
            if (aAx3.YDirection.Dot(new Direction3D(origD1U)) < 0)
            {
                aAx3 = aAx3.YReversed(); //  mais X reste !
            }

            if (nVP > 2)
            {
                if ((Math.Abs(r[1]) < tol) && (Math.Abs(r[2]) < tol) && (Math.Abs(r[3]) > tol))
                {
                    SphericalSurface anObject = new SphericalSurface(aAx3, r[3]);
                    if (!uClosed)
                    {
                        anObject.UReverse();
                    }
                    newSurf = anObject;
                }
            }
            else if (nVP == 2)
            {
                if (Math.Abs(r[2] - r[1]) < tol)
                {
                    CylindricalSurface anObject = new CylindricalSurface(aAx3, r[1]);
                    if (!uClosed)
                    {
                        anObject.UReverse();
                    }
                    newSurf = anObject;
                }
                else
                {
                    Vector3D vec2 = new Vector3D(p1[1], p1[2]);
                    double angle = vec.AngleTo(vec2);
                    if (r[1] < r[2])
                    {
                        ConicalSurface anObject = new ConicalSurface(aAx3, angle, r[1]);
                        // if (!uClosed) anObject->UReverse();
                        anObject.UReverse();
                        newSurf = anObject;
                    }
                    else
                    {
                        dir = dir.Reversed();
                        Vector3D anotherXVec = new Vector3D(p3[2], p1[2]);
                        Direction3D anotherXDir = new Direction3D(anotherXVec);
                        CoordinateSystem anotherAx3 = new CoordinateSystem(p3[2], dir, anotherXDir);
                        ConicalSurface anObject = new ConicalSurface(anotherAx3, angle, r[2]);
                        // if (!uClosed) anObject->UReverse();
                        anObject.UReverse();
                        newSurf = anObject;
                    }
                }
            }
        }
        else if (toroid)
        {
            // test by iso U and isoV
            bool isFound = false;
            for (j = 1; (j <= 2) && !isFound; j++)
            {
                if (j == 2)
                {
                    c1 = u1;
                    c2 = u2;
                    u1 = v1;
                    u2 = v2;
                    v1 = c1;
                    v2 = c2;
                }
                for (i = 1; i <= 3; i++)
                {
                    if (i == 1)
                    {
                        u = u1;
                    }
                    else if (i == 2)
                    {
                        u = 0.5 * (u1 + u2);
                    }
                    else if (i == 3)
                    {
                        u = 0.25 * (u1 + u2);
                    }

                    iso = _surf.UIso(u);

                    iso.D0(v1, out p1[i]);
                    iso.D0(0.5 * (v1 + v2), out p2[i]);
                    p3[i] = p3[i].WithCoord(0.5 * (p1[i].X + p2[i].X),
                                   0.5 * (p1[i].Y + p2[i].Y),
                                   0.5 * (p1[i].Z + p2[i].Z));
                    r[i] = p3[i].DistanceTo(p1[i]);
                }
                if ((Math.Abs(r[1] - r[2]) < tol) && (Math.Abs(r[1] - r[3]) < tol))
                {
                    Point3D p10 = new Point3D(0.5 * (p3[1].X + p3[2].X),
                                            0.5 * (p3[1].Y + p3[2].Y),
                                            0.5 * (p3[1].Z + p3[2].Z));
                    Vector3D vec = new Vector3D(p10, p3[1]);
                    Vector3D vec2 = new Vector3D(p10, p3[3]);
                    double rr1 = r[1], rr2 = r[2], rr3;
                    vec = vec.Crossed(vec2);

                    if (vec.Magnitude <= Constants.Resolution())
                    {
                        vec = vec.WithCoord(0.0, 0.0, 1.0);
                    }

                    Direction3D dir = new Direction3D(vec);

                    CoordinateSystem aAx3 = new CoordinateSystem(p10, dir);
                    rr1 = p10.DistanceTo(p3[1]);
                    //          modif empirique (pourtant NON DEMONTREE) : inverser roles RR1,RR2
                    //          CKY, 24-JAN-1997
                    if (rr1 < rr2)
                    {
                        rr3 = rr1;
                        rr1 = rr2;
                        rr2 = rr3;
                    }
                    ToroidalSurface anObject = new ToroidalSurface(aAx3, rr1, rr2);
                    if (j == 2)
                    {
                        anObject.UReverse();
                    }
                    anObject.D1(0.0, 0.0, out resPnt, out resD1U, out resD1V);
                    newSurf = anObject;
                    isFound = true;
                }
            }
        }
        if (newSurf is null)
        {
            return newSurf;
        }

        //---------------------------------------------------------------------
        //                 verification
        //---------------------------------------------------------------------

        SurfaceAdaptor nhs = new SurfaceAdaptor(newSurf);
        SurfaceAdaptor surfAdapt = nhs;

        const int np = 21;
        double s = 0.0, t = 0.0; // U,V deja fait
        Point3D p3d, p3d2;
        bool onSurface = true;

        double dis;
        _gap = 0.0;

        du = (u2 - u1) / (np - 1);
        dv = (v2 - v1) / (np - 1);
        for (j = 1; (j <= np) && onSurface; j++)
        {
            v = v1 + dv * (j - 1);

            if (uClosed)
            {
                iso = _surf.VIso(v);
            }
            else
            {
                iso = _surf.UIso(v);
            }

            for (i = 1; i <= np; i++)
            {
                u = u1 + du * (i - 1);
                iso.D0(u, out p3d);
                switch (surfAdapt.GetType())
                {
                    case SurfaceKind.Cylinder:
                    {
                        Cylinder cylinder = surfAdapt.Cylinder();
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cylinder, p3d, out s, out t);
                        break;
                    }
                    case SurfaceKind.Cone:
                    {
                        Cone cone = surfAdapt.Cone();
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cone, p3d, out s, out t);
                        break;
                    }
                    case SurfaceKind.Sphere:
                    {
                        Sphere sphere = surfAdapt.Sphere();
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(sphere, p3d, out s, out t);
                        break;
                    }
                    case SurfaceKind.Torus:
                    {
                        Torus torus = surfAdapt.Torus();
                        global::Kerf.Engine.Geometry.ElSLib.Parameters(torus, p3d, out s, out t);
                        break;
                    }
                    default:
                        break;
                }

                newSurf!.D0(s, t, out p3d2);

                dis = p3d.DistanceTo(p3d2);
                if (dis > _gap)
                {
                    _gap = dis;
                }

                if (dis > tol)
                {
                    onSurface = false;
                    newSurf = null;
                    // The presumption is rejected
                    break;
                }
            }
        }
        // OCCT: the adaptor is never null, so a rejected surface (null) replaces the tool's surface with substitute
        // (occt-issues.md)
        if (substitute && nhs is not null)
        {
            Init(newSurf);
        }
        return newSurf;
    }

    /// <summary>
    /// Tries to convert the surface (a B-spline closed in U or V within <paramref name="preci"/>) to its periodic
    /// form, and returns the result, or null if nothing was done. With <paramref name="substitute"/>, the result
    /// replaces the tool's surface.
    /// </summary>
    /// <remarks>
    /// OCCT makes the surface itself periodic when its knots needn't be rearranged; the port makes a copy of a frozen
    /// surface periodic instead (decision 974), so the caller's surface never changes.
    /// </remarks>
    public Surface? ConvertToPeriodic(bool substitute, double preci = -1)
    {
        Surface? newSurf = null;
        global::Kerf.Geometry.BSplineSurface? bSpl = _surf as global::Kerf.Geometry.BSplineSurface;
        if (bSpl is null)
        {
            return newSurf;
        }

        SurfaceAnalysis sas = new SurfaceAnalysis(_surf);
        bool uclosed = sas.IsUClosed(preci);
        bool vclosed = sas.IsVClosed(preci);

        if (!uclosed && !vclosed)
        {
            return newSurf;
        }

        bool converted = false; //: p6

        if (uclosed && !bSpl.IsUPeriodic && bSpl.UPoleCount > 3)
        {
            bool set = true;
            // if degree+1 at ends, first change it to 1 by rearranging knots
            if (bSpl.UMultiplicity(1) == bSpl.UDegree + 1
                && bSpl.UMultiplicity(bSpl.UKnotCount) == bSpl.UDegree + 1)
            {
                NCollection_Array2<Point3D> oldPoles = bSpl.InternalPoles();
                NCollection_Array2<double> oldWeights = bSpl.InternalWeightsArray();
                int nbUKnots = bSpl.UKnotCount;
                NCollection_Array1<double> oldUKnots = bSpl.InternalUKnots();
                NCollection_Array1<double> oldVKnots = bSpl.InternalVKnots();
                NCollection_Array1<int> oldUMults = bSpl.InternalUMultiplicities();
                NCollection_Array1<int> oldVMults = bSpl.InternalVMultiplicities();

                NCollection_Array1<double> newUKnots = new(1, nbUKnots + 2);
                NCollection_Array1<int> newUMults = new(1, nbUKnots + 2);
                double a =
                    0.5 * (bSpl.UKnot(2) - bSpl.UKnot(1) + bSpl.UKnot(nbUKnots) - bSpl.UKnot(nbUKnots - 1));

                newUKnots[1] = oldUKnots[1] - a;
                newUKnots[nbUKnots + 2] = oldUKnots[nbUKnots] + a;
                newUMults[1] = newUMults[nbUKnots + 2] = 1;
                for (int i = 2; i <= nbUKnots + 1; i++)
                {
                    newUKnots[i] = oldUKnots[i - 1];
                    newUMults[i] = oldUMults[i - 1];
                }
                newUMults[2] = newUMults[nbUKnots + 1] = bSpl.UDegree;
                global::Kerf.Geometry.BSplineSurface res = new global::Kerf.Geometry.BSplineSurface(oldPoles,
                                                                  oldWeights,
                                                                  newUKnots,
                                                                  oldVKnots,
                                                                  newUMults,
                                                                  oldVMults,
                                                                  bSpl.UDegree,
                                                                  bSpl.VDegree,
                                                                  bSpl.IsUPeriodic,
                                                                  bSpl.IsVPeriodic);
                bSpl = res;
            }
            else if (bSpl.UMultiplicity(1) > bSpl.UDegree
                     || bSpl.UMultiplicity(bSpl.UKnotCount) > bSpl.UDegree + 1)
            {
                set = false;
            }
            if (set)
            {
                // OCCT.NET: OCCT makes the caller's surface periodic in place; a frozen surface is copied first
                // (decision 974)
                if (bSpl.IsFrozen)
                {
                    bSpl = (global::Kerf.Geometry.BSplineSurface)bSpl.Copy();
                }
                bSpl.SetUPeriodic(); // make periodic
                converted = true;
            }
        }

        if (vclosed && !bSpl.IsVPeriodic && bSpl.VPoleCount > 3)
        {
            bool set = true;
            // if degree+1 at ends, first change it to 1 by rearranging knots
            if (bSpl.VMultiplicity(1) == bSpl.VDegree + 1
                && bSpl.VMultiplicity(bSpl.VKnotCount) == bSpl.VDegree + 1)
            {
                NCollection_Array2<Point3D> oldPoles = bSpl.InternalPoles();
                NCollection_Array2<double> oldWeights = bSpl.InternalWeightsArray();
                int nbVKnots = bSpl.VKnotCount;
                NCollection_Array1<double> oldUKnots = bSpl.InternalUKnots();
                NCollection_Array1<double> oldVKnots = bSpl.InternalVKnots();
                NCollection_Array1<int> oldUMults = bSpl.InternalUMultiplicities();
                NCollection_Array1<int> oldVMults = bSpl.InternalVMultiplicities();

                NCollection_Array1<double> newVKnots = new(1, nbVKnots + 2);
                NCollection_Array1<int> newVMults = new(1, nbVKnots + 2);
                double a =
                    0.5 * (bSpl.VKnot(2) - bSpl.VKnot(1) + bSpl.VKnot(nbVKnots) - bSpl.VKnot(nbVKnots - 1));

                newVKnots[1] = oldVKnots[1] - a;
                newVKnots[nbVKnots + 2] = oldVKnots[nbVKnots] + a;
                newVMults[1] = newVMults[nbVKnots + 2] = 1;
                for (int i = 2; i <= nbVKnots + 1; i++)
                {
                    newVKnots[i] = oldVKnots[i - 1];
                    newVMults[i] = oldVMults[i - 1];
                }
                newVMults[2] = newVMults[nbVKnots + 1] = bSpl.VDegree;
                global::Kerf.Geometry.BSplineSurface res = new global::Kerf.Geometry.BSplineSurface(oldPoles,
                                                                  oldWeights,
                                                                  oldUKnots,
                                                                  newVKnots,
                                                                  oldUMults,
                                                                  newVMults,
                                                                  bSpl.UDegree,
                                                                  bSpl.VDegree,
                                                                  bSpl.IsUPeriodic,
                                                                  bSpl.IsVPeriodic);
                bSpl = res;
            }
            else if (bSpl.VMultiplicity(1) > bSpl.VDegree
                     || bSpl.VMultiplicity(bSpl.VKnotCount) > bSpl.VDegree + 1)
            {
                set = false;
            }
            if (set)
            {
                // OCCT.NET: as above (decision 974)
                if (bSpl.IsFrozen)
                {
                    bSpl = (global::Kerf.Geometry.BSplineSurface)bSpl.Copy();
                }
                bSpl.SetVPeriodic(); // make periodic
                converted = true;
            }
        }

        if (!converted)
        {
            return newSurf;
        }
        newSurf = bSpl;
        if (substitute)
        {
            _surf = newSurf;
        }
        return newSurf;
    }
}
