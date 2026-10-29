// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_InterUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.IntCurveSurface_TransitionOnCurve;
using static Occt.Intf_PIType;

namespace Occt;

/// <summary>
/// The helpers of the generic curve/surface intersection (OCCT's namespace IntCurveSurface_InterUtils): bounds of
/// infinite surfaces, parameters of section points, transitions, the analytic cases and the refinement of the
/// interference's points.
/// </summary>
/// <remarks>
/// OCCT's template functions over the curve, the surface and their tools are generic methods constrained on the
/// tool interfaces (decision 3601). Points are returned through an <c>NCollection_DynamicArray</c> filled by the
/// caller's array, as in OCCT.
/// </remarks>
internal static class IntCurveSurface_InterUtils
{
    public const double THE_TOLERANCE_ANGULAIRE = 1.0e-12;
    public const double THE_TOLTANGENCY = 0.00000001;

    /// <summary>
    /// Projects <paramref name="theLine"/> and its symmetric line about the X axis on <paramref name="thePln"/>,
    /// intersects them with the projected basis curve, and widens [<paramref name="theVmin"/>,
    /// <paramref name="theVmax"/>] with the parameters found on it.
    /// </summary>
    public static void ProjectIntersectAndEstLim(in gp_Lin theLine, in gp_Pln thePln, ProjLib_Plane theBasCurvProj,
                                                 ref double theVmin, ref double theVmax, ref bool theNoIntersection)
    {
        ProjLib_Plane aLineProj = new(thePln, theLine);
        if (!aLineProj.IsDone())
        {
            return;
        }
        gp_Lin2d aLin2d = aLineProj.Line();

        // make a second line X-axe symmetric to the first one
        gp_Pnt2d aP1 = aLin2d.Location();
        gp_Pnt2d aP2 = new(aP1.XY() + aLin2d.Direction().XY());
        gp_Pnt2d aP1sym = new(aP1.X(), -aP1.Y());
        gp_Pnt2d aP2sym = new(aP2.X(), -aP2.Y());
        gp_Lin2d aLin2dsym = new(aP1sym, new gp_Dir2d(new gp_Vec2d(aP1sym, aP2sym)));

        // intersect projections
        IntAna2d_Conic aCon = new(aLin2d);
        IntAna2d_Conic aConSym = new(aLin2dsym);
        IntAna2d_AnaIntersection anIntersect = new();
        IntAna2d_AnaIntersection anIntersectSym = new();

        switch (theBasCurvProj.GetType())
        {
            case GeomAbs_Line:
                anIntersectSym.Perform(theBasCurvProj.Line(), aConSym);
                anIntersect.Perform(theBasCurvProj.Line(), aCon);
                break;
            case GeomAbs_Hyperbola:
                anIntersectSym.Perform(theBasCurvProj.Hyperbola(), aConSym);
                anIntersect.Perform(theBasCurvProj.Hyperbola(), aCon);
                break;
            case GeomAbs_Parabola:
                anIntersectSym.Perform(theBasCurvProj.Parabola(), aConSym);
                anIntersect.Perform(theBasCurvProj.Parabola(), aCon);
                break;
            default:
                return; // not infinite curve
        }

        // retrieve params of intersections
        int aNbIntPnt = anIntersect.IsDone() ? anIntersect.NbPoints() : 0;
        int aNbIntPntSym = anIntersectSym.IsDone() ? anIntersectSym.NbPoints() : 0;
        int iPnt, aNbPnt = Max(aNbIntPnt, aNbIntPntSym);

        if (aNbPnt == 0)
        {
            theNoIntersection = true;
            return;
        }
        double aParam;
        for (iPnt = 1; iPnt <= aNbPnt; iPnt++)
        {
            if (iPnt <= aNbIntPnt)
            {
                IntAna2d_IntPoint aIntPnt = anIntersect.Point(iPnt);
                aParam = aIntPnt.ParamOnFirst();
                theVmin = Min(theVmin, aParam);
                theVmax = Max(theVmax, aParam);
            }
            if (iPnt <= aNbIntPntSym)
            {
                IntAna2d_IntPoint aIntPnt = anIntersectSym.Point(iPnt);
                aParam = aIntPnt.ParamOnFirst();
                theVmin = Min(theVmin, aParam);
                theVmax = Max(theVmax, aParam);
            }
        }
    }

    /// <summary>Bounds the parameters of an infinite surface to ±1e10 (the generic fallback).</summary>
    public static void EstLimForInfSurf(ref double U1new, ref double U2new, ref double V1new, ref double V2new)
    {
        U1new = Max(U1new, -1.0e10);
        U2new = Min(U2new, 1.0e10);
        V1new = Max(V1new, -1.0e10);
        V2new = Min(V2new, 1.0e10);
    }

    /// <summary>Estimates the bounds of an infinite surface of extrusion (or offset of one) around the line.</summary>
    public static void EstLimForInfExtr<SurfaceType, SurfaceTool>(in gp_Lin Line, in SurfaceType surface,
                                                                  bool IsOffSurf, int nbsu, bool U1inf, bool U2inf,
                                                                  bool V1inf, bool V2inf, ref double U1new,
                                                                  ref double U2new, ref double V1new,
                                                                  ref double V2new, out bool NoIntersection)
        where SurfaceTool : IIntCurveSurface_SurfaceTool<SurfaceType>
    {
        NoIntersection = false;

        Adaptor3d_Surface? aBasSurf = null;

        if (IsOffSurf)
        {
            aBasSurf = SurfaceTool.BasisSurface(surface);
        }

        gp_Dir aDirOfExt;

        if (IsOffSurf)
        {
            aDirOfExt = aBasSurf!.Direction();
        }
        else
        {
            aDirOfExt = SurfaceTool.Direction(surface);
        }

        double tolang = THE_TOLERANCE_ANGULAIRE;

        if (aDirOfExt.IsParallel(Line.Direction(), tolang))
        {
            NoIntersection = true;
            return;
        }

        if ((V1inf || V2inf) && !(U1inf || U2inf))
        {
            double vmin = RealLast(), vmax = -vmin;
            gp_Lin aL = new();
            double step = (U2new - U1new) / nbsu;
            double u = U1new, v;
            gp_Pnt aP;
            int i;

            for (i = 0; i <= nbsu; i++)
            {
                SurfaceTool.D0(surface, u, 0.0, out aP);
                aL.SetLocation(aP);
                aL.SetDirection(aDirOfExt);

                Extrema_ExtElC aExtr = new(aL, Line, tolang);

                if (!aExtr.IsDone())
                {
                    return;
                }

                if (aExtr.IsParallel())
                {
                    NoIntersection = true;
                    return;
                }

                aExtr.Points(1, out Extrema_POnCurv aP1, out _);
                v = aP1.Parameter();
                vmin = Min(vmin, v);
                vmax = Max(vmax, v);

                u += step;
            }

            vmin = vmin - Math.Abs(vmin) - 10.0;
            vmax = vmax + Math.Abs(vmax) + 10.0;

            V1new = Max(V1new, vmin);
            V2new = Min(V2new, vmax);
        }
        else if (U1inf || U2inf)
        {
            double umin = RealLast(), umax = -umin;
            double u0 = Min(Max(0.0, U1new), U2new);
            double v0 = Min(Max(0.0, V1new), V2new);
            SurfaceTool.D0(surface, u0, v0, out gp_Pnt aP);
            gp_Pln aRefPln = new(aP, aDirOfExt);

            Adaptor3d_Curve aBasCurv;

            if (IsOffSurf)
            {
                aBasCurv = aBasSurf!.BasisCurve();
            }
            else
            {
                aBasCurv = SurfaceTool.BasisCurve(surface);
            }

            ProjLib_Plane Projector = new(aRefPln);

            Projector.Project(Line);

            if (!Projector.IsDone())
            {
                return;
            }

            gp_Lin2d Line2d = Projector.Line();

            GeomAbs_CurveType aCurvTyp = aBasCurv.GetType();

            if (aCurvTyp == GeomAbs_Line)
            {
                Projector.Project(aBasCurv.Line());

                if (!Projector.IsDone())
                {
                    return;
                }

                gp_Lin2d aL2d = Projector.Line();

                IntAna2d_AnaIntersection anInter = new(Line2d, aL2d);

                if (!anInter.IsDone())
                {
                    return;
                }

                if (anInter.IsEmpty() || anInter.IdenticalElements() || anInter.ParallelElements())
                {
                    NoIntersection = true;
                    return;
                }

                IntAna2d_IntPoint anIntPnt = anInter.Point(1);
                umin = umax = anIntPnt.ParamOnSecond();
            }
            else if (aCurvTyp == GeomAbs_Parabola || aCurvTyp == GeomAbs_Hyperbola)
            {
                IntAna2d_Conic aCon = new(Line2d);
                IntAna2d_AnaIntersection anInter = new();

                if (aCurvTyp == GeomAbs_Parabola)
                {
                    Projector.Project(aBasCurv.Parabola());
                    if (!Projector.IsDone())
                    {
                        return;
                    }

                    gp_Parab2d aP2d = Projector.Parabola();

                    anInter.Perform(aP2d, aCon);
                }
                else
                {
                    Projector.Project(aBasCurv.Hyperbola());
                    if (!Projector.IsDone())
                    {
                        return;
                    }

                    gp_Hypr2d aH2d = Projector.Hyperbola();
                    anInter.Perform(aH2d, aCon);
                }

                if (!anInter.IsDone())
                {
                    return;
                }

                if (anInter.IsEmpty())
                {
                    NoIntersection = true;
                    return;
                }

                int i, nbint = anInter.NbPoints();
                for (i = 1; i <= nbint; i++)
                {
                    IntAna2d_IntPoint anIntPnt = anInter.Point(i);

                    umin = Min(anIntPnt.ParamOnFirst(), umin);
                    umax = Max(anIntPnt.ParamOnFirst(), umax);
                }
            }
            else
            {
                return;
            }

            umin = umin - Math.Abs(umin) - 10;
            umax = umax + Math.Abs(umax) + 10;

            U1new = Max(U1new, umin);
            U2new = Min(U2new, umax);

            if (V1inf || V2inf)
            {
                EstLimForInfExtr<SurfaceType, SurfaceTool>(Line, surface, IsOffSurf, nbsu, false, false, V1inf, V2inf,
                                                           ref U1new, ref U2new, ref V1new, ref V2new,
                                                           out NoIntersection);
            }
        }
    }

    /// <summary>Estimates the V bounds of an infinite surface of revolution around the line.</summary>
    public static void EstLimForInfRevl<SurfaceType, SurfaceTool>(in gp_Lin Line, in SurfaceType surface, bool U1inf,
                                                                  bool U2inf, bool V1inf, bool V2inf,
                                                                  ref double U1new, ref double U2new,
                                                                  ref double V1new, ref double V2new,
                                                                  out bool NoIntersection)
        where SurfaceTool : IIntCurveSurface_SurfaceTool<SurfaceType>
    {
        NoIntersection = false;

        if (U1inf || U2inf)
        {
            if (U1inf)
            {
                U1new = Max(0.0, U1new);
            }
            else
            {
                U2new = Min(2 * M_PI, U2new);
            }
            if (!V1inf && !V2inf)
            {
                return;
            }
        }

        Adaptor3d_Curve aBasisCurve = SurfaceTool.BasisCurve(surface);
        gp_Ax1 aRevAx = SurfaceTool.AxeOfRevolution(surface);
        gp_Vec aXVec = new(aRevAx.Direction());
        double aTolAng = Precision.Angular();

        // make plane to project a basis curve
        gp_Pnt O = aRevAx.Location();
        double aU = 0.0;
        gp_Pnt P = aBasisCurve.Value(aU);
        while (O.SquareDistance(P) <= Precision.PConfusion() || aXVec.IsParallel(new gp_Vec(O, P), aTolAng))
        {
            aU += 1.0;
            P = aBasisCurve.Value(aU);
            if (aU > 3)
            {
                // basis curve is a line coinciding with aXVec, P is any not on aXVec
                P = new gp_Pnt(aU, aU + 1, aU + 2);
            }
        }
        gp_Vec aNVec = aXVec ^ new gp_Vec(O, P);
        gp_Pln aPln = new(new gp_Ax3(O, new gp_Dir(aNVec), new gp_Dir(aXVec)));

        // project basic curve
        ProjLib_Plane aBasCurvProj = new(aPln);
        switch (aBasisCurve.GetType())
        {
            case GeomAbs_Line:
                aBasCurvProj.Project(aBasisCurve.Line());
                break;
            case GeomAbs_Hyperbola:
                aBasCurvProj.Project(aBasisCurve.Hyperbola());
                break;
            case GeomAbs_Parabola:
                aBasCurvProj.Project(aBasisCurve.Parabola());
                break;
            default:
                return; // not infinite curve
        }
        if (!aBasCurvProj.IsDone())
        {
            return;
        }
        // make plane to project Line
        if (aXVec.IsParallel(new gp_Vec(Line.Direction()), aTolAng))
        {
            P = Line.Location();
            while (O.SquareDistance(P) <= Precision.PConfusion())
            {
                aU += 1.0;
                P = new gp_Pnt(aU, aU + 1, aU + 2); // any not on aXVec
            }
            aNVec = aXVec ^ new gp_Vec(O, P);
        }
        else
        {
            aNVec = aXVec.Crossed(new gp_Vec(Line.Direction()));
        }

        aPln = new gp_Pln(new gp_Ax3(O, new gp_Dir(aNVec), new gp_Dir(aXVec)));

        // make a second plane perpendicular to the first one, rotated around aXVec
        gp_Pln aPlnPrp = aPln.Rotated(new gp_Ax1(O, new gp_Dir(aXVec)), M_PI / 2.0);

        // project Line and its X-axe symmetric one to plane and intersect
        // resulting curve with projection of Basic Curve
        double aVmin = RealLast(), aVmax = -aVmin;
        bool aNoInt1 = false, aNoInt2 = false;
        ProjectIntersectAndEstLim(Line, aPln, aBasCurvProj, ref aVmin, ref aVmax, ref aNoInt1);
        ProjectIntersectAndEstLim(Line, aPlnPrp, aBasCurvProj, ref aVmin, ref aVmax, ref aNoInt2);

        if (aNoInt1 && aNoInt2)
        {
            NoIntersection = true;
            return;
        }

        aVmin = aVmin - Math.Abs(aVmin) - 10;
        aVmax = aVmax + Math.Abs(aVmax) + 10;

        if (V1inf)
        {
            V1new = aVmin;
        }
        if (V2inf)
        {
            V2new = aVmax;
        }
    }

    /// <summary>Estimates the bounds of an infinite offset surface around the line.</summary>
    public static void EstLimForInfOffs<SurfaceType, SurfaceTool>(in gp_Lin Line, in SurfaceType surface, int nbsu,
                                                                  bool U1inf, bool U2inf, bool V1inf, bool V2inf,
                                                                  ref double U1new, ref double U2new,
                                                                  ref double V1new, ref double V2new,
                                                                  out bool NoIntersection)
        where SurfaceTool : IIntCurveSurface_SurfaceTool<SurfaceType>
    {
        NoIntersection = false;

        Adaptor3d_Surface aBasSurf = SurfaceTool.BasisSurface(surface);
        double anOffVal = SurfaceTool.OffsetValue(surface);

        GeomAbs_SurfaceType aTypeOfBasSurf = aBasSurf.GetType();

        //  case for plane, cylinder and cone - make equivalent surface;
        if (aTypeOfBasSurf == GeomAbs_Plane)
        {
            gp_Pln aPln = aBasSurf.Plane();
            // gp_Dir ^ gp_Dir is a gp_Dir (normalized), converted to gp_Vec
            gp_Vec aT = new(aPln.Position().XDirection() ^ aPln.Position().YDirection());
            aT *= anOffVal;
            aPln.Translate(aT);
            IntAna_IntConicQuad LinPlane = new(Line, aPln, THE_TOLERANCE_ANGULAIRE);

            if (!LinPlane.IsDone())
            {
                return;
            }

            if (LinPlane.IsParallel() || LinPlane.IsInQuadric())
            {
                NoIntersection = true;
                return;
            }

            ElSLib.Parameters(aPln, LinPlane.Point(1), out double u, out double v);
            U1new = Max(U1new, u - 10.0);
            U2new = Min(U2new, u + 10.0);
            V1new = Max(V1new, v - 10.0);
            V2new = Min(V2new, v + 10.0);
        }
        else if (aTypeOfBasSurf == GeomAbs_Cylinder)
        {
            gp_Cylinder aCyl = aBasSurf.Cylinder();

            double aR = aCyl.Radius();
            gp_Ax3 anA = aCyl.Position();

            if (anA.Direct())
            {
                aR += anOffVal;
            }
            else
            {
                aR -= anOffVal;
            }

            if (aR >= THE_TOLTANGENCY)
            {
                aCyl.SetRadius(aR);
            }
            else if (aR <= -THE_TOLTANGENCY)
            {
                anA.Rotate(new gp_Ax1(anA.Location(), anA.Direction()), M_PI);
                aCyl.SetPosition(anA);
                aCyl.SetRadius(-aR);
            }
            else
            {
                NoIntersection = true;
                return;
            }

            IntAna_IntConicQuad LinCylinder = new(Line, new IntAna_Quadric(aCyl));

            if (!LinCylinder.IsDone())
            {
                return;
            }

            if (LinCylinder.IsParallel() || LinCylinder.IsInQuadric())
            {
                NoIntersection = true;
                return;
            }

            int i, nbp = LinCylinder.NbPoints();
            double vmin = RealLast(), vmax = -vmin;

            for (i = 1; i <= nbp; i++)
            {
                ElSLib.Parameters(aCyl, LinCylinder.Point(i), out _, out double v);
                vmin = Min(vmin, v);
                vmax = Max(vmax, v);
            }

            V1new = Max(V1new, vmin - Math.Abs(vmin) - 10.0);
            V2new = Min(V2new, vmax + Math.Abs(vmax) + 10.0);
        }
        else if (aTypeOfBasSurf == GeomAbs_Cone)
        {
            gp_Cone aCon = aBasSurf.Cone();
            double anAng = aCon.SemiAngle();
            double aR = aCon.RefRadius() + anOffVal * Math.Cos(anAng);
            gp_Ax3 anA = aCon.Position();
            if (aR >= 0.0)
            {
                gp_Vec aZ = new(anA.Direction());
                aZ *= -anOffVal * Math.Sin(anAng);
                anA.Translate(aZ);
                aCon.SetPosition(anA);
                aCon.SetRadius(aR);
                aCon.SetSemiAngle(anAng);
            }
            else
            {
                return;
            }

            IntAna_IntConicQuad LinCone = new(Line, new IntAna_Quadric(aCon));

            if (!LinCone.IsDone())
            {
                return;
            }

            if (LinCone.IsParallel() || LinCone.IsInQuadric())
            {
                NoIntersection = true;
                return;
            }

            int i, nbp = LinCone.NbPoints();
            double vmin = RealLast(), vmax = -vmin;

            for (i = 1; i <= nbp; i++)
            {
                ElSLib.Parameters(aCon, LinCone.Point(i), out _, out double v);
                vmin = Min(vmin, v);
                vmax = Max(vmax, v);
            }

            V1new = Max(V1new, vmin - Math.Abs(vmin) - 10.0);
            V2new = Min(V2new, vmax + Math.Abs(vmax) + 10.0);
        }
        else if (aTypeOfBasSurf == GeomAbs_SurfaceOfExtrusion)
        {
            double anU1 = U1new, anU2 = U2new;

            EstLimForInfExtr<SurfaceType, SurfaceTool>(Line, surface, true, nbsu, U1inf, U2inf, V1inf, V2inf,
                                                       ref U1new, ref U2new, ref V1new, ref V2new,
                                                       out NoIntersection);

            if (NoIntersection)
            {
                return;
            }

            if (U1inf || U2inf)
            {
                GeomAbs_CurveType aBasCurvType = aBasSurf.BasisCurve().GetType();
                if (aBasCurvType == GeomAbs_Line)
                {
                    U1new = Max(anU1, -1.0e10);
                    U2new = Min(anU2, 1.0e10);
                }
                else if (aBasCurvType == GeomAbs_Parabola)
                {
                    gp_Parab aPrb = aBasSurf.BasisCurve().Parabola();
                    double aF = aPrb.Focal();
                    double dU = 2.0e5 * Math.Sqrt(aF);
                    U1new = Max(anU1, -dU);
                    U2new = Min(anU2, dU);
                }
                else if (aBasCurvType == GeomAbs_Hyperbola)
                {
                    U1new = Max(anU1, -30.0);
                    U2new = Min(anU2, 30.0);
                }
                else
                {
                    U1new = Max(anU1, -1.0e10);
                    U2new = Min(anU2, 1.0e10);
                }
            }
        }
        else if (aTypeOfBasSurf == GeomAbs_SurfaceOfRevolution)
        {
            GeomAbs_CurveType aBasCurvType = aBasSurf.BasisCurve().GetType();
            if (aBasCurvType == GeomAbs_Line)
            {
                V1new = Max(V1new, -1.0e10);
                V2new = Min(V2new, 1.0e10);
            }
            else if (aBasCurvType == GeomAbs_Parabola)
            {
                gp_Parab aPrb = aBasSurf.BasisCurve().Parabola();
                double aF = aPrb.Focal();
                double dV = 2.0e5 * Math.Sqrt(aF);
                V1new = Max(V1new, -dV);
                V2new = Min(V2new, dV);
            }
            else if (aBasCurvType == GeomAbs_Hyperbola)
            {
                V1new = Max(V1new, -30.0);
                V2new = Min(V2new, 30.0);
            }
            else
            {
                V1new = Max(V1new, -1.0e10);
                V2new = Min(V2new, 1.0e10);
            }
        }
        else
        {
            V1new = Max(V1new, -1.0e10);
            V2new = Min(V2new, 1.0e10);
        }
    }

    /// <summary>
    /// The approximate parameters on the surface (from the polyhedron's vertex, edge or triangle) and on the curve
    /// (from the polygon's segment) of a section point of the interference.
    /// </summary>
    public static void SectionPointToParameters<PolyhedronType, PolygonType>(in Intf_SectionPoint Sp,
                                                                             PolyhedronType Polyhedron,
                                                                             PolygonType Polygon, out double U,
                                                                             out double V, out double W)
        where PolyhedronType : IIntCurveSurface_Polyhedron
        where PolygonType : IIntCurveSurface_Polygon
    {
        Intf_PIType typ;
        int Adr1, Adr2;
        double Param, u, v;
        gp_Pnt P = Sp.Pnt();

        int Pt1, Pt2, Pt3;
        double u1 = 0.0, v1 = 0.0, param;
        //----------------------------------------------------------------------
        //--          Approximate parameter calculation on surface            --
        //----------------------------------------------------------------------

        Sp.InfoSecond(out typ, out Adr1, out Adr2, out Param);
        switch (typ)
        {
            case Intf_VERTEX: //-- Adr1 is the vertex number
            {
                Polyhedron.Parameters(Adr1, out u1, out v1);
                break;
            }
            case Intf_EDGE:
            {
                Polyhedron.Parameters(Adr1, out u1, out v1);
                Polyhedron.Parameters(Adr2, out u, out v);
                u1 += Param * (u - u1);
                v1 += Param * (v - v1);
                break;
            }
            case Intf_FACE:
            {
                double ua, va, ub, vb, uc, vc, ca, cb, cc, cabc;
                Polyhedron.Triangle(Adr1, out Pt1, out Pt2, out Pt3);
                gp_Pnt PA = Polyhedron.Point(Pt1);
                gp_Pnt PB = Polyhedron.Point(Pt2);
                gp_Pnt PC = Polyhedron.Point(Pt3);
                Polyhedron.Parameters(Pt1, out ua, out va);
                Polyhedron.Parameters(Pt2, out ub, out vb);
                Polyhedron.Parameters(Pt3, out uc, out vc);
                gp_Vec Normale = new gp_Vec(PA, PB).Crossed(new gp_Vec(PA, PC));
                cc = (new gp_Vec(PA, PB).Crossed(new gp_Vec(PA, P))).Dot(Normale);
                ca = (new gp_Vec(PB, PC).Crossed(new gp_Vec(PB, P))).Dot(Normale);
                cb = (new gp_Vec(PC, PA).Crossed(new gp_Vec(PC, P))).Dot(Normale);
                cabc = ca + cb + cc;

                if (Math.Abs(cabc) > Precision.Computational())
                {
                    ca /= cabc;
                    cb /= cabc;
                    cc /= cabc;

                    u1 = ca * ua + cb * ub + cc * uc;
                    v1 = ca * va + cb * vb + cc * vc;
                }
                else
                {
                    gp_Vec aAB = new(PA, PB);
                    gp_Vec aBC = new(PB, PC);
                    gp_Vec aCA = new(PC, PA);
                    double aSqAB = aAB.SquareMagnitude();
                    double aSqBC = aBC.SquareMagnitude();
                    double aSqCA = aCA.SquareMagnitude();

                    if (aSqAB >= aSqBC && aSqAB >= aSqCA && aSqAB > Precision.SquareConfusion())
                    {
                        double aT = new gp_Vec(PA, P).Dot(aAB) / aSqAB;
                        aT = Min(1.0, Max(0.0, aT));
                        u1 = ua + aT * (ub - ua);
                        v1 = va + aT * (vb - va);
                    }
                    else if (aSqBC >= aSqCA && aSqBC > Precision.SquareConfusion())
                    {
                        double aT = new gp_Vec(PB, P).Dot(aBC) / aSqBC;
                        aT = Min(1.0, Max(0.0, aT));
                        u1 = ub + aT * (uc - ub);
                        v1 = vb + aT * (vc - vb);
                    }
                    else if (aSqCA > Precision.SquareConfusion())
                    {
                        double aT = new gp_Vec(PC, P).Dot(aCA) / aSqCA;
                        aT = Min(1.0, Max(0.0, aT));
                        u1 = uc + aT * (ua - uc);
                        v1 = vc + aT * (va - vc);
                    }
                    else
                    {
                        u1 = ua;
                        v1 = va;
                    }
                }
                break;
            }
            default:
            {
                break;
            }
        }
        //----------------------------------------------------------------------
        //--              Approximate point calculation on Curve              --
        //----------------------------------------------------------------------
        Sp.InfoFirst(out typ, out int SegIndex, out param);
        W = Polygon.ApproxParamOnCurve(SegIndex, param);
        U = u1;
        V = v1;
    }

    /// <summary>The transition of the curve through the surface at the point of parameters (u, v) and w.</summary>
    public static void ComputeTransitions<CurveType, CurveTool, SurfaceType, SurfaceTool>(
        in CurveType curve, double w, out IntCurveSurface_TransitionOnCurve TransOnCurve, in SurfaceType surface,
        double u, double v)
        where CurveTool : IIntCurveSurface_CurveTool<CurveType>
        where SurfaceTool : IIntCurveSurface_SurfaceTool<SurfaceType>
    {
        gp_Vec NSurf, D1U, D1V;
        gp_Pnt Psurf;
        double CosDir;

        SurfaceTool.D1(surface, u, v, out Psurf, out D1U, out D1V);
        NSurf = D1U.Crossed(D1V);
        CurveTool.D1(curve, w, out Psurf, out D1U);
        double Norm = NSurf.Magnitude();
        if (Norm > THE_TOLERANCE_ANGULAIRE && D1U.SquareMagnitude() > THE_TOLERANCE_ANGULAIRE)
        {
            D1U.Normalize();
            CosDir = NSurf.Dot(D1U);
            CosDir /= Norm;
            if (-CosDir > THE_TOLERANCE_ANGULAIRE)
            {
                //--  --Curve--->    <----Surface----
                TransOnCurve = IntCurveSurface_In;
            }
            else if (CosDir > THE_TOLERANCE_ANGULAIRE)
            {
                //--  --Curve--->  ----Surface-->
                TransOnCurve = IntCurveSurface_Out;
            }
            else
            {
                TransOnCurve = IntCurveSurface_Tangent;
            }
        }
        else
        {
            TransOnCurve = IntCurveSurface_Tangent;
        }
    }

    /// <summary>
    /// The parameters of <paramref name="P"/> on the quadric surface (ElSLib); unchanged for other surfaces, as in
    /// OCCT.
    /// </summary>
    public static void ComputeParamsOnQuadric<SurfaceType, SurfaceTool>(in SurfaceType surface, in gp_Pnt P,
                                                                        ref double u, ref double v)
        where SurfaceTool : IIntCurveSurface_SurfaceTool<SurfaceType>
    {
        GeomAbs_SurfaceType SurfaceType_ = SurfaceTool.GetType(surface);
        switch (SurfaceType_)
        {
            case GeomAbs_Plane:
            {
                ElSLib.Parameters(SurfaceTool.Plane(surface), P, out u, out v);
                break;
            }
            case GeomAbs_Cylinder:
            {
                ElSLib.Parameters(SurfaceTool.Cylinder(surface), P, out u, out v);
                break;
            }
            case GeomAbs_Cone:
            {
                ElSLib.Parameters(SurfaceTool.Cone(surface), P, out u, out v);
                break;
            }
            case GeomAbs_Sphere:
            {
                ElSLib.Parameters(SurfaceTool.Sphere(surface), P, out u, out v);
                break;
            }
            default:
                break;
        }
    }

    /// <summary>Samples the surface on a 50 × 50 grid into <paramref name="thePntsOnSurface"/> and the box.</summary>
    public static void DoSurface<SurfaceType, SurfaceTool>(in SurfaceType theSurface, double theU0, double theU1,
                                                           double theV0, double theV1,
                                                           NCollection_Array2<gp_Pnt> thePntsOnSurface,
                                                           ref Bnd_Box theBoxSurface, out double theGap)
        where SurfaceTool : IIntCurveSurface_SurfaceTool<SurfaceType>
    {
        int iU = 0, iV = 0;
        double U = 0.0, V = 0.0;
        double dU = (theU1 - theU0) / 50.0, dV = (theV1 - theV0) / 50.0;
        gp_Pnt aPnt;

        for (iU = 0; iU < 50; iU++)
        {
            if (iU == 0)
            {
                U = theU0;
            }
            else if (iU == 49)
            {
                U = theU1;
            }
            else
            {
                U = theU0 + dU * ((double)iU);
            }

            for (iV = 0; iV < 50; iV++)
            {
                if (iV == 0)
                {
                    V = theV0;
                }
                else if (iV == 49)
                {
                    V = theV1;
                }
                else
                {
                    V = theV0 + dV * ((double)iV);
                }

                SurfaceTool.D0(theSurface, U, V, out aPnt);
                theBoxSurface.Add(aPnt);
                thePntsOnSurface.SetValue(iU + 1, iV + 1, aPnt);
            }
        }
        double Ures = SurfaceTool.UResolution(theSurface, dU);
        double Vres = SurfaceTool.VResolution(theSurface, dV);
        theGap = Max(Ures, Vres);
    }

    /// <summary>
    /// New parameter bounds of the surface around the grid points nearest to the corners of a box
    /// (<paramref name="theX"/>, <paramref name="theY"/>, <paramref name="theZ"/>), in the directions that are not
    /// closed.
    /// </summary>
    public static void DoNewBounds<SurfaceType, SurfaceTool>(in SurfaceType theSurface, double theU0, double theU1,
                                                             double theV0, double theV1,
                                                             NCollection_Array2<gp_Pnt> thePntsOnSurface,
                                                             NCollection_Array1<double> theX,
                                                             NCollection_Array1<double> theY,
                                                             NCollection_Array1<double> theZ,
                                                             NCollection_Array1<double> theBounds)
        where SurfaceTool : IIntCurveSurface_SurfaceTool<SurfaceType>
    {
        theBounds.SetValue(1, theU0);
        theBounds.SetValue(2, theU1);
        theBounds.SetValue(3, theV0);
        theBounds.SetValue(4, theV1);

        bool isUClosed = (SurfaceTool.IsUClosed(theSurface) || SurfaceTool.IsUPeriodic(theSurface));
        bool isVClosed = (SurfaceTool.IsVClosed(theSurface) || SurfaceTool.IsVPeriodic(theSurface));
        bool checkU = (isUClosed) ? false : true;
        bool checkV = (isVClosed) ? false : true;

        int i = 0, j = 0, k = 0, iU = 0, iV = 0;
        int iUmin = 50, iVmin = 50, iUmax = 1, iVmax = 1;

        for (i = 1; i <= 2; i++)
        {
            for (j = 1; j <= 2; j++)
            {
                for (k = 1; k <= 2; k++)
                {
                    gp_Pnt aPoint = new(theX[i], theY[j], theZ[k]);
                    double DistMin = 1.0e+100;
                    int diU = 0, diV = 0;
                    for (iU = 1; iU <= 50; iU++)
                    {
                        for (iV = 1; iV <= 50; iV++)
                        {
                            gp_Pnt aP = thePntsOnSurface.Value(iU, iV);
                            double dist = aP.SquareDistance(aPoint);
                            if (dist < DistMin)
                            {
                                DistMin = dist;
                                diU = iU;
                                diV = iV;
                            }
                        }
                    }
                    if (diU > 0 && diU < iUmin)
                    {
                        iUmin = diU;
                    }
                    if (diU > 0 && diU > iUmax)
                    {
                        iUmax = diU;
                    }
                    if (diV > 0 && diV < iVmin)
                    {
                        iVmin = diV;
                    }
                    if (diV > 0 && diV > iVmax)
                    {
                        iVmax = diV;
                    }
                }
            }
        }

        double dU = (theU1 - theU0) / 50.0, dV = (theV1 - theV0) / 50.0;

        double USmin = theU0 + dU * ((double)(iUmin - 1));
        double USmax = theU0 + dU * ((double)(iUmax - 1));
        double VSmin = theV0 + dV * ((double)(iVmin - 1));
        double VSmax = theV0 + dV * ((double)(iVmax - 1));

        if (USmin > USmax)
        {
            double tmp = USmax;
            USmax = USmin;
            USmin = tmp;
        }
        if (VSmin > VSmax)
        {
            double tmp = VSmax;
            VSmax = VSmin;
            VSmin = tmp;
        }

        USmin -= 1.5 * dU;
        if (USmin < theU0)
        {
            USmin = theU0;
        }
        USmax += 1.5 * dU;
        if (USmax > theU1)
        {
            USmax = theU1;
        }
        VSmin -= 1.5 * dV;
        if (VSmin < theV0)
        {
            VSmin = theV0;
        }
        VSmax += 1.5 * dV;
        if (VSmax > theV1)
        {
            VSmax = theV1;
        }

        if (checkU)
        {
            theBounds.SetValue(1, USmin);
            theBounds.SetValue(2, USmax);
        }
        if (checkV)
        {
            theBounds.SetValue(3, VSmin);
            theBounds.SetValue(4, VSmax);
        }
    }

    /// <summary>
    /// The intersection point at the parameters, put in the periods of the curve and the surface, with its
    /// transition; false if it is outside their bounds (by more than the tangency tolerance).
    /// </summary>
    public static bool ComputeAppendPoint<CurveType, CurveTool, SurfaceType, SurfaceTool>(
        in CurveType theCurve, double theLw, in SurfaceType theSurface, double theSu, double theSv,
        out IntCurveSurface_IntersectionPoint thePoint)
        where CurveTool : IIntCurveSurface_CurveTool<CurveType>
        where SurfaceTool : IIntCurveSurface_SurfaceTool<SurfaceType>
    {
        thePoint = new IntCurveSurface_IntersectionPoint();
        double W0 = CurveTool.FirstParameter(theCurve);
        double W1 = CurveTool.LastParameter(theCurve);
        double U0 = SurfaceTool.FirstUParameter(theSurface);
        double U1 = SurfaceTool.LastUParameter(theSurface);
        double V0 = SurfaceTool.FirstVParameter(theSurface);
        double V1 = SurfaceTool.LastVParameter(theSurface);

        double w = theLw, u = theSu, v = theSv;

        GeomAbs_CurveType aCType = CurveTool.GetType(theCurve);

        if (CurveTool.IsPeriodic(theCurve) || aCType == GeomAbs_Circle || aCType == GeomAbs_Ellipse)
        {
            w = ElCLib.InPeriod(w, W0, W0 + CurveTool.Period(theCurve));
        }

        if ((W0 - w) >= THE_TOLTANGENCY || (w - W1) >= THE_TOLTANGENCY)
        {
            return false;
        }

        GeomAbs_SurfaceType aSType = SurfaceTool.GetType(theSurface);
        if (SurfaceTool.IsUPeriodic(theSurface) || aSType == GeomAbs_Cylinder || aSType == GeomAbs_Cone
            || aSType == GeomAbs_Sphere)
        {
            u = ElCLib.InPeriod(u, U0, U0 + SurfaceTool.UPeriod(theSurface));
        }

        if (SurfaceTool.IsVPeriodic(theSurface))
        {
            v = ElCLib.InPeriod(v, V0, V0 + SurfaceTool.VPeriod(theSurface));
        }

        if ((U0 - u) >= THE_TOLTANGENCY || (u - U1) >= THE_TOLTANGENCY)
        {
            return false;
        }
        if ((V0 - v) >= THE_TOLTANGENCY || (v - V1) >= THE_TOLTANGENCY)
        {
            return false;
        }

        ComputeTransitions<CurveType, CurveTool, SurfaceType, SurfaceTool>(theCurve, w,
                                                                           out IntCurveSurface_TransitionOnCurve TransOnCurve,
                                                                           theSurface, u, v);
        gp_Pnt P = CurveTool.Value(theCurve, w);
        thePoint = new IntCurveSurface_IntersectionPoint(P, u, v, w, TransOnCurve);
        return true;
    }

    /// <summary>
    /// The points of an analytic conic/quadric intersection that are inside the bounds; false if it was not done,
    /// and <paramref name="theIsParallel"/> if the conic is parallel to or on the quadric.
    /// </summary>
    public static bool ProcessIntAna<CurveType, CurveTool, SurfaceType, SurfaceTool>(
        in CurveType theCurve, in SurfaceType theSurface, IntAna_IntConicQuad theIntAna, out bool theIsParallel,
        NCollection_DynamicArray<IntCurveSurface_IntersectionPoint> thePoints)
        where CurveTool : IIntCurveSurface_CurveTool<CurveType>
        where SurfaceTool : IIntCurveSurface_SurfaceTool<SurfaceType>
    {
        theIsParallel = false;
        thePoints.Clear();

        if (!theIntAna.IsDone())
        {
            return false;
        }

        if (theIntAna.IsInQuadric() || theIntAna.IsParallel())
        {
            theIsParallel = true;
            return true;
        }

        int nbp = theIntAna.NbPoints();
        double u = 0.0, v = 0.0, w;
        for (int i = 1; i <= nbp; i++)
        {
            gp_Pnt P = theIntAna.Point(i);
            w = theIntAna.ParamOnConic(i);
            ComputeParamsOnQuadric<SurfaceType, SurfaceTool>(theSurface, P, ref u, ref v);

            if (ComputeAppendPoint<CurveType, CurveTool, SurfaceType, SurfaceTool>(theCurve, w, theSurface, u, v,
                                                                                   out IntCurveSurface_IntersectionPoint aPoint))
            {
                thePoints.Append(aPoint);
            }
        }
        return true;
    }

    /// <summary>The intersection of the curve with a quadric surface by the exact quadric/curve algorithm.</summary>
    public static void PerformCurveQuadric<QuadCurvExactType, CurveType, CurveTool, SurfaceType, SurfaceTool>(
        in CurveType theCurve, in SurfaceType theSurface,
        NCollection_DynamicArray<IntCurveSurface_IntersectionPoint> thePoints)
        where QuadCurvExactType : IIntCurveSurface_TheQuadCurvExactInter<QuadCurvExactType, SurfaceType, CurveType>
        where CurveTool : IIntCurveSurface_CurveTool<CurveType>
        where SurfaceTool : IIntCurveSurface_SurfaceTool<SurfaceType>
    {
        thePoints.Clear();

        QuadCurvExactType QuadCurv = QuadCurvExactType.Create(theSurface, theCurve);
        if (QuadCurv.IsDone())
        {
            int NbRoots = QuadCurv.NbRoots();
            double u = 0.0, v = 0.0, w;
            for (int i = 1; i <= NbRoots; i++)
            {
                w = QuadCurv.Root(i);
                ComputeParamsOnQuadric<SurfaceType, SurfaceTool>(theSurface, CurveTool.Value(theCurve, w), ref u,
                                                                 ref v);

                if (ComputeAppendPoint<CurveType, CurveTool, SurfaceType, SurfaceTool>(theCurve, w, theSurface, u, v,
                                                                                       out IntCurveSurface_IntersectionPoint aPoint))
                {
                    thePoints.Append(aPoint);
                }
            }
        }
    }

    /// <summary>The intersection of a line and a torus (IntAna_IntLinTorus); false if it failed.</summary>
    public static bool ProcessLinTorus<CurveType, CurveTool, SurfaceType, SurfaceTool>(
        in gp_Lin theLine, in CurveType theCurve, in SurfaceType theSurface,
        NCollection_DynamicArray<IntCurveSurface_IntersectionPoint> thePoints)
        where CurveTool : IIntCurveSurface_CurveTool<CurveType>
        where SurfaceTool : IIntCurveSurface_SurfaceTool<SurfaceType>
    {
        thePoints.Clear();

        IntAna_IntLinTorus intlintorus = new(theLine, SurfaceTool.Torus(theSurface));
        if (!intlintorus.IsDone())
        {
            return false;
        }

        int nbp = intlintorus.NbPoints();
        double w;
        for (int i = 1; i <= nbp; i++)
        {
            w = intlintorus.ParamOnLine(i);
            intlintorus.ParamOnTorus(i, out double fi, out double theta);

            if (ComputeAppendPoint<CurveType, CurveTool, SurfaceType, SurfaceTool>(theCurve, w, theSurface, fi, theta,
                                                                                   out IntCurveSurface_IntersectionPoint aPoint))
            {
                thePoints.Append(aPoint);
            }
        }
        return true;
    }

    /// <summary>The start points of the exact intersection: their (U, V, W) parameters.</summary>
    public sealed class SortedStartPoints
    {
        public readonly NCollection_DynamicArray<double> TabU = new();
        public readonly NCollection_DynamicArray<double> TabV = new();
        public readonly NCollection_DynamicArray<double> TabW = new();

        public void Clear()
        {
            TabU.Clear();
            TabV.Clear();
            TabW.Clear();
        }

        public int Size() => TabU.Length();

        public void Append(double theU, double theV, double theW)
        {
            TabU.Append(theU);
            TabV.Append(theV);
            TabW.Append(theW);
        }
    }

    /// <summary>The parameters of the section points of the interference, and of the points of its tangent zones.</summary>
    public static void CollectInterferencePoints<InterferenceType, PolyhedronType, PolygonType>(
        InterferenceType theInterference, PolyhedronType thePolyhedron, PolygonType thePolygon,
        SortedStartPoints thePoints)
        where InterferenceType : Intf_Interference
        where PolyhedronType : IIntCurveSurface_Polyhedron
        where PolygonType : IIntCurveSurface_Polygon
    {
        thePoints.Clear();

        int NbSectionPoints = theInterference.NbSectionPoints();
        int NbTangentZones = theInterference.NbTangentZones();

        double u, v, w;

        for (int i = 1; i <= NbSectionPoints; i++)
        {
            Intf_SectionPoint SP = theInterference.PntValue(i);
            SectionPointToParameters(SP, thePolyhedron, thePolygon, out u, out v, out w);
            thePoints.Append(u, v, w);
        }

        for (int i = 1; i <= NbTangentZones; i++)
        {
            Intf_TangentZone TZ = theInterference.ZoneValue(i);
            int nbpnts = TZ.NumberOfPoints();
            for (int j = 1; j <= nbpnts; j++)
            {
                Intf_SectionPoint SP = TZ.GetPoint(j);
                SectionPointToParameters(SP, thePolyhedron, thePolygon, out u, out v, out w);
                thePoints.Append(u, v, w);
            }
        }
    }

    /// <summary>Sorts the start points by W, then U, then V (bubble sorts, merging near-equal values).</summary>
    public static void SortStartPoints(SortedStartPoints thePoints)
    {
        int NbStartPoints = thePoints.Size();
        if (NbStartPoints == 0)
        {
            return;
        }

        double ptol = 10 * Precision.PConfusion();

        // Sort by W
        bool Triok;
        do
        {
            Triok = true;
            for (int i = 1; i < NbStartPoints; i++)
            {
                int im1 = i - 1;
                if (thePoints.TabW[i] < thePoints.TabW[im1])
                {
                    (thePoints.TabW[i], thePoints.TabW[im1]) = (thePoints.TabW[im1], thePoints.TabW[i]);
                    (thePoints.TabU[i], thePoints.TabU[im1]) = (thePoints.TabU[im1], thePoints.TabU[i]);
                    (thePoints.TabV[i], thePoints.TabV[im1]) = (thePoints.TabV[im1], thePoints.TabV[i]);
                    Triok = false;
                }
            }
        } while (!Triok);

        // Sort by U for same W
        do
        {
            Triok = true;
            for (int i = 1; i < NbStartPoints; i++)
            {
                int im1 = i - 1;
                if ((thePoints.TabW[i] - thePoints.TabW[im1]) < ptol)
                {
                    thePoints.TabW[i] = thePoints.TabW[im1];
                    if (thePoints.TabU[i] < thePoints.TabU[im1])
                    {
                        (thePoints.TabU[i], thePoints.TabU[im1]) = (thePoints.TabU[im1], thePoints.TabU[i]);
                        (thePoints.TabV[i], thePoints.TabV[im1]) = (thePoints.TabV[im1], thePoints.TabV[i]);
                        Triok = false;
                    }
                }
            }
        } while (!Triok);

        // Sort by V for same W and U
        do
        {
            Triok = true;
            for (int i = 1; i < NbStartPoints; i++)
            {
                int im1 = i - 1;
                if (((thePoints.TabW[i] - thePoints.TabW[im1]) < ptol)
                    && ((thePoints.TabU[i] - thePoints.TabU[im1]) < ptol))
                {
                    thePoints.TabU[i] = thePoints.TabU[im1];
                    if (thePoints.TabV[i] < thePoints.TabV[im1])
                    {
                        (thePoints.TabV[i], thePoints.TabV[im1]) = (thePoints.TabV[im1], thePoints.TabV[i]);
                        Triok = false;
                    }
                }
            }
        } while (!Triok);
    }

    /// <summary>
    /// Refines each distinct start point by the exact intersection, and keeps the points inside the curve's and the
    /// surface's bounds.
    /// </summary>
    public static void ProcessSortedPoints<ExactInterType, CSFunctionType, CurveType, CurveTool, SurfaceType, SurfaceTool>(
        ExactInterType theExactInter, math_FunctionSetRoot theRsnld, SortedStartPoints thePoints, double theU0,
        double theU1, double theV0, double theV1, double theWinf, double theWsup, in CurveType theCurve,
        in SurfaceType theSurface, NCollection_DynamicArray<IntCurveSurface_IntersectionPoint> theResult)
        where ExactInterType : IntImp_IntCS<CSFunctionType, SurfaceType, SurfaceTool, CurveType, CurveTool>
        where CSFunctionType : math_FunctionSetWithDerivatives, IIntImp_CSFunction<SurfaceType, CurveType>
        where CurveTool : IIntCurveSurface_CurveTool<CurveType>
        where SurfaceTool : IIntCurveSurface_SurfaceTool<SurfaceType>
    {
        theResult.Clear();

        int NbStartPoints = thePoints.Size();
        if (NbStartPoints == 0)
        {
            return;
        }

        double ptol = 10 * Precision.PConfusion();
        double su = 0, sv = 0, sw = 0;

        for (int i = 0; i < NbStartPoints; i++)
        {
            double u = thePoints.TabU[i];
            double v = thePoints.TabV[i];
            double w = thePoints.TabW[i];

            if (i == 0)
            {
                su = u - 1;
            }

            if (Math.Abs(u - su) > ptol || Math.Abs(v - sv) > ptol || Math.Abs(w - sw) > ptol)
            {
                theExactInter.Perform(u, v, w, theRsnld, theU0, theU1, theV0, theV1, theWinf, theWsup);
                if (theExactInter.IsDone() && !theExactInter.IsEmpty())
                {
                    w = theExactInter.ParameterOnCurve();
                    theExactInter.ParameterOnSurface(out u, out v);

                    if (ComputeAppendPoint<CurveType, CurveTool, SurfaceType, SurfaceTool>(theCurve, w, theSurface, u,
                                                                                           v, out IntCurveSurface_IntersectionPoint aPoint))
                    {
                        theResult.Append(aPoint);
                    }
                }
            }
            su = thePoints.TabU[i];
            sv = thePoints.TabV[i];
            sw = thePoints.TabW[i];
        }
    }

    /// <summary>A rectangle of parameters on a surface.</summary>
    public readonly struct UVBounds(double theU0, double theU1, double theV0, double theV1)
    {
        public readonly double U0 = theU0;
        public readonly double U1 = theU1;
        public readonly double V0 = theV0;
        public readonly double V1 = theV1;
    }

    /// <summary>The rectangles of the C2 intervals of the surface in U and V.</summary>
    public static void DecomposeSurfaceIntervals<SurfaceType, SurfaceTool>(in SurfaceType theSurface,
                                                                           NCollection_DynamicArray<UVBounds> theIntervals)
        where SurfaceTool : IIntCurveSurface_SurfaceTool<SurfaceType>
    {
        theIntervals.Clear();

        int NbUOnS = SurfaceTool.NbUIntervals(theSurface, GeomAbs_C2);
        int NbVOnS = SurfaceTool.NbVIntervals(theSurface, GeomAbs_C2);

        if (NbUOnS > 1)
        {
            NCollection_Array1<double> TabU = new(1, NbUOnS + 1);
            SurfaceTool.UIntervals(theSurface, TabU, GeomAbs_C2);

            for (int iu = 1; iu <= NbUOnS; iu++)
            {
                double U0 = TabU.Value(iu);
                double U1 = TabU.Value(iu + 1);

                if (NbVOnS > 1)
                {
                    NCollection_Array1<double> TabV = new(1, NbVOnS + 1);
                    SurfaceTool.VIntervals(theSurface, TabV, GeomAbs_C2);
                    for (int iv = 1; iv <= NbVOnS; iv++)
                    {
                        double V0 = TabV.Value(iv);
                        double V1 = TabV.Value(iv + 1);
                        theIntervals.Append(new UVBounds(U0, U1, V0, V1));
                    }
                }
                else
                {
                    double V0 = SurfaceTool.FirstVParameter(theSurface);
                    double V1 = SurfaceTool.LastVParameter(theSurface);
                    theIntervals.Append(new UVBounds(U0, U1, V0, V1));
                }
            }
        }
        else if (NbVOnS > 1)
        {
            double U0 = SurfaceTool.FirstUParameter(theSurface);
            double U1 = SurfaceTool.LastUParameter(theSurface);

            NCollection_Array1<double> TabV = new(1, NbVOnS + 1);
            SurfaceTool.VIntervals(theSurface, TabV, GeomAbs_C2);

            for (int iv = 1; iv <= NbVOnS; iv++)
            {
                double V0 = TabV.Value(iv);
                double V1 = TabV.Value(iv + 1);
                theIntervals.Append(new UVBounds(U0, U1, V0, V1));
            }
        }
        else
        {
            double U0 = SurfaceTool.FirstUParameter(theSurface);
            double U1 = SurfaceTool.LastUParameter(theSurface);
            double V0 = SurfaceTool.FirstVParameter(theSurface);
            double V1 = SurfaceTool.LastVParameter(theSurface);
            theIntervals.Append(new UVBounds(U0, U1, V0, V1));
        }
    }

    /// <summary>Clamps the parameters to ±1e50 (protection from double overflow in square magnitudes, bug26525).</summary>
    public static void ClampUVParameters(ref double theU1, ref double theU2, ref double theV1, ref double theV2)
    {
        const double THE_PARAM_LIMIT = 1.0e50;
        if (theU1 < -THE_PARAM_LIMIT)
        {
            theU1 = -THE_PARAM_LIMIT;
        }
        if (theU2 > THE_PARAM_LIMIT)
        {
            theU2 = THE_PARAM_LIMIT;
        }
        if (theV1 < -THE_PARAM_LIMIT)
        {
            theV1 = -THE_PARAM_LIMIT;
        }
        if (theV2 > THE_PARAM_LIMIT)
        {
            theV2 = THE_PARAM_LIMIT;
        }
    }
}
