// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_Intersector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_Intersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification.Topological;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Classification;

/// <summary>
/// Intersects an edge (its pcurve on its face) with a segment of a 2D line: the intersector of the face classifier.
/// </summary>
/// <remarks>
/// A working object (decision 21): an <see cref="Intersection2D"/> whose results are those of the last
/// <see cref="Perform(in Line2D, double, double, in ClassifierEdge)"/>.
/// </remarks>
internal class Intersector : IntConicCurveOfGInter, IIntersector<ClassifierEdge>
{
    /// <summary>An empty intersector.</summary>
    public Intersector()
    {
    }

    private static double MaxTol2DCurEdge(in Vertex v1,
                                          in Vertex v2,
                                          in Face f,
                                          double tol)
    {
        double tolV3D1, tolV3D2;
        if (v1.IsNull)
        {
            tolV3D1 = 0.0;
        }
        else
        {
            tolV3D1 = Tool.Tolerance(v1);
        }

        if (v2.IsNull)
        {
            tolV3D2 = 0.0;
        }
        else
        {
            tolV3D2 = Tool.Tolerance(v2);
        }

        double tol2D, ur, vr;

        double tolV3D = Max(tolV3D1, tolV3D2);
        var s = new FaceSurfaceAdaptor(f, false);

        ur = s.UResolution(tolV3D);
        vr = s.VResolution(tolV3D);
        tol2D = Max(ur, vr);
        //
        tol2D = Max(tol2D, tol);
        return tol2D;
    }

    private static bool IsInter(in Box2d box, in Line2D l, double p)
    {
        bool statusInter = true;
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(p))
        {
            statusInter = box.IsOut(l);
        }
        else
        {
            Point2D pntF = l.Location;
            Point2D pntL = global::Kerf.Engine.Geometry.ElCLib.Value(p, l);
            statusInter = box.IsOut(pntF, pntL);
        }

        return !statusInter;
    }

    private static bool CheckOn(ref IntersectionPoint2D pntInter,
                                in Face f,
                                in Line2D l,
                                CurveAdaptor2D cur,
                                ref double tolZ,
                                double fin,
                                double deb)
    {
        var extPC2d = new ExtPC2d(l.Location, cur);
        double minDist = RealLast();
        int minInd = 0;
        if (extPC2d.IsDone)
        {
            int nbPnts = extPC2d.ExtCount;
            for (int i = 1; i <= nbPnts; ++i)
            {
                double dist = extPC2d.SquareDistance(i);

                if (dist < minDist)
                {
                    minDist = dist;
                    minInd = i;
                }
            }
        }

        if (minInd != 0)
        {
            minDist = Math.Sqrt(minDist);
        }

        if (minDist <= tolZ)
        {
            Point2D pntExact = (extPC2d.Point(minInd)).Value();
            double par = (extPC2d.Point(minInd)).Parameter();
            //
            RefineTolerance(f, cur, par, ref tolZ);
            //
            if (minDist <= tolZ)
            {
                var trOnLin = new Transition(Position2D.Head);
                Position2D posOnCurve = Position2D.Middle;
                if ((Math.Abs(par - deb) <= global::Kerf.Engine.Foundation.Precision.Confusion()) || (par < deb))
                {
                    posOnCurve = Position2D.Head;
                }
                else if ((Math.Abs(par - fin) <= global::Kerf.Engine.Foundation.Precision.Confusion()) || (par > fin))
                {
                    posOnCurve = Position2D.End;
                }

                //
                var trOnCurve = new Transition(posOnCurve);
                pntInter = new IntersectionPoint2D(pntExact, 0.0, par, trOnLin, trOnCurve, false);
                //
                return true;
            }
        }

        return false;
    }

    private static void CheckSkip(ref GInter inter,
                                  in Line2D l,
                                  in ClassifierEdge e,
                                  Curve2D c2D,
                                  in TwoDDomain dl,
                                  CurveAdaptor2D cur,
                                  CurveAdaptor2D cga,
                                  ref double fin,
                                  ref double deb,
                                  double maxTol,
                                  ref Point2D pdeb,
                                  ref Point2D pfin)
    {
        if (e.Edge().IsNull || e.Face().IsNull)
        {
            return;
        }

        bool isLSkip = false;
        Vertex vl; // the last vertex of current edge

        Curve2D? skipC2D;

        vl = Exploration.LastVertex(e.Edge(), true);
        if (vl.IsNull)
        {
            return;
        }

        if (!(Tool.Tolerance(vl) > maxTol) || e.NextEdge().IsNull)
        {
            return;
        }

        double ldeb = 0.0, lfin = 0.0;
        Curve2D? lc2D; // the next curve

        lc2D = Tool.CurveOnSurface(e.NextEdge(), e.Face(), out ldeb, out lfin);
        if (lc2D is null)
        {
            return;
        }

        double a, b, c; // coefficients of the straight line
        double x1, y1, x2, y2; // coordinates of the ends of edges
        Point2D p1, p2; // the ends of edges

        l.Coefficients(out a, out b, out c);

        double at1 = fin;
        if (e.Edge().Orientation != Forward)
        {
            at1 = deb;
        }

        double at2 = ldeb;
        if (e.NextEdge().Orientation != Forward)
        {
            at2 = lfin;
        }

        p1 = c2D.Value(at1);
        p2 = lc2D.Value(at2);

        // Check if points belong to DL domain
        double par1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, p1);
        double par2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, p2);

        if (par1 <= dl.FirstParameter() || par1 >= dl.LastParameter()
            || par2 <= dl.FirstParameter() || par2 >= dl.LastParameter())
        {
            return;
        }

        x1 = p1.X;
        y1 = p1.Y;
        x2 = p2.X;
        y2 = p2.Y;
        double fv = a * x1 + b * y1 + c;
        double sv = a * x2 + b * y2 + c;

        // Check for getting into vertex with high tolerance
        if ((fv * sv) >= 0)
        {
            isLSkip = false;
        }
        else
        {
            isLSkip = true;
            var mkSeg = new MakeSegment2d(p1, p2);
            if (!mkSeg.IsDone)
            {
                return;
            }

            skipC2D = mkSeg.Value();

            if (skipC2D is null || !isLSkip)
            {
                return;
            }

            // if we got
            cur.Load(skipC2D);
            if (cur.Curve() is null)
            {
                return;
            }

            double atoldeb = 1.0e-5, atolfin = 1.0e-5;

            deb = cur.FirstParameter();
            fin = cur.LastParameter();
            cur.D0(deb, out pdeb);
            cur.D0(fin, out pfin);

            var de = new TwoDDomain(pdeb, deb, atoldeb, pfin, fin, atolfin);
            // temporary periodic domain
            if (cur.Curve()!.IsPeriodic)
            {
                de.SetEquivalentParameters(cur.FirstParameter(),
                                            cur.FirstParameter() + cur.Curve()!.LastParameter
                                            - cur.Curve()!.FirstParameter);
            }

            inter = new GInter(cga,
                                            dl,
                                            cur,
                                            de,
                                            global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                            global::Kerf.Engine.Foundation.Precision.PIntersection());
        }
    }

    /// <summary>Intersects the segment of <paramref name="l"/> from its origin to the parameter <paramref name="p"/> with the edge.</summary>
    public void Perform(in Line2D l, double p, double tol, in ClassifierEdge e)
    {
        double deb = 0.0, fin = 0.0, tolZ = tol;
        Curve2D? c2D;
        //
        Edge ee = e.Edge();
        Face f = e.Face();

        //
        c2D = Tool.CurveOnSurface(ee, f, out deb, out fin);
        if (c2D is null)
        {
            _done = false; // !IsDone()
            return;
        }

        //
        Box2d bond = new();
        Point2D pntF = new();
        bool useBndBox = e.UseBndBox();
        if (useBndBox)
        {
            Add2dCurve.Add(c2D, deb, fin, 0.0, ref bond);
            bond.SetGap(tolZ);
            pntF = l.Location;
        }

        //
        var c = new CurveAdaptor2D(c2D, deb, fin);
        //
        // Case of "ON": direct check of belonging to edge
        // taking into account the tolerance
        if (!useBndBox || (useBndBox && !bond.IsOut(pntF)))
        {
            bool statusOn = false;
            IntersectionPoint2D pntInter = new();
            double debTol = deb;
            double finTol = fin;
            if (tolZ > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                debTol = deb - tolZ;
                finTol = fin + tolZ;
            }

            var curAdaptor = new CurveAdaptor2D(c2D, debTol, finTol);

            statusOn = CheckOn(ref pntInter, f, l, curAdaptor, ref tolZ, fin, deb);
            if (statusOn)
            {
                Append(pntInter);
                _done = true;
                return;
            }
        }

        //
        if (useBndBox)
        {
            Exploration.Vertices(ee, out Vertex vf, out Vertex vl);

            tolZ = MaxTol2DCurEdge(vf, vl, f, tol);
            bond.SetGap(tolZ);

            if (!IsInter(bond, l, p))
            {
                _done = false;
                return;
            }
        }

        c.D0(deb, out Point2D pdeb);
        c.D0(fin, out Point2D pfin);
        double toldeb = 1.0e-5, tolfin = 1.0e-5;

        TwoDDomain dl = new();
        //
        if (p != RealLast())
        {
            dl.SetValues(l.Location,
                         0.0,
                         global::Kerf.Engine.Foundation.Precision.PConfusion(),
                         global::Kerf.Engine.Geometry.ElCLib.Value(p, l),
                         p,
                         global::Kerf.Engine.Foundation.Precision.PConfusion());
        }
        else
        {
            dl.SetValues(l.Location, 0.0, global::Kerf.Engine.Foundation.Precision.PConfusion(), true);
        }

        var de = new TwoDDomain(pdeb, deb, toldeb, pfin, fin, tolfin);
        // temporary periodic domain
        if (c.Curve()!.IsPeriodic)
        {
            de.SetEquivalentParameters(c.FirstParameter(),
                                       c.FirstParameter() + c.Curve()!.LastParameter
                                       - c.Curve()!.FirstParameter);
        }

        var gl = new LineCurve2D(l);
        var cga = new CurveAdaptor2D(gl);
        var inter = new GInter(cga, dl, c, de, global::Kerf.Engine.Foundation.Precision.PConfusion(), global::Kerf.Engine.Foundation.Precision.PIntersection());
        //
        // The check is for hitting the intersector to
        // a vertex with high tolerance
        if (inter.IsEmpty)
        {
            CheckSkip(ref inter, l, e, c2D, dl, c, cga, ref fin, ref deb, e.MaxTolerance(), ref pdeb, ref pfin);
        }

        //
        SetValues(inter);
    }

    /// <summary>
    /// The tangent <paramref name="tang"/>, normal <paramref name="norm"/> and curvature <paramref name="c"/> of the
    /// edge's pcurve at the parameter <paramref name="u"/>.
    /// </summary>
    /// <remarks>
    /// Where OCCT leaves the tangent as the caller had it (no tangent and a chord too short), the port gives OCCT's
    /// callers' default, (1, 0).
    /// </remarks>
    public void LocalGeometry(in ClassifierEdge e, double u, out Direction2D tang, out Direction2D norm, out double c)
    {
        tang = new Direction2D();
        norm = new Direction2D();
        Curve2D pCurve = Tool.CurveOnSurface(e.Edge(), e.Face(), out double fpar, out double lpar)!;
        var prop = new CLProps2d(pCurve, u, 2, global::Kerf.Engine.Foundation.Precision.PConfusion());

        c = 0.0;
        if (prop.IsTangentDefined())
        {
            prop.Tangent(out tang);
            c = prop.Curvature();
        }
        else
        {
            GetTangentAsChord(pCurve, ref tang, u, fpar, lpar);
        }

        if (c > global::Kerf.Engine.Foundation.Precision.PConfusion() && !global::Kerf.Engine.Foundation.Precision.IsInfinite(c))
        {
            prop.Normal(out norm);
        }
        else
        {
            norm = norm.WithCoord(tang.Y, -tang.X);
        }
    }

    private static void RefineTolerance(in Face aF,
                                        CurveAdaptor2D aC,
                                        double aT,
                                        ref double aTolZ)
    {
        SurfaceKind typeS;
        //
        var bas = new FaceSurfaceAdaptor(aF, false);
        //
        typeS = bas.GetType();
        if (typeS == SurfaceKind.Cylinder)
        {
            double uRes, vRes, tolX;
            //
            uRes = bas.UResolution(aTolZ);
            vRes = bas.VResolution(aTolZ);
            //
            aC.D1(aT, out Point2D _, out Vector2D v2D);
            Direction2D d2D = new(v2D);
            //
            tolX = uRes * d2D.Y + vRes * d2D.X;
            if (tolX < 0.0)
            {
                tolX = -tolX;
            }

            //
            if (tolX < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                tolX = global::Kerf.Engine.Foundation.Precision.Confusion();
            }

            //
            if (tolX < aTolZ)
            {
                aTolZ = tolX;
            }
        }
    }

    private static void GetTangentAsChord(Curve2D pCurve,
                                          ref Direction2D tangent,
                                          double param,
                                          double first,
                                          double last)
    {
        double offset = 0.1 * (last - first);

        if (last - param < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            // theParam == theLast
            offset *= -1;
        }
        else if (param + offset > last)
        {
            //<theParam> is close to <theLast>
            offset = 0.5 * (last - param);
        }

        Point2D pnt2d = pCurve.Value(param);
        Point2D offsetPnt2d = pCurve.Value(param + offset);

        Vector2D chord = new(pnt2d, offsetPnt2d);
        if (offset < 0.0)
        {
            chord = chord.Reversed();
        }

        double sqLength = chord.SquareMagnitude;
        if (sqLength > global::Kerf.Engine.Foundation.Precision.SquarePConfusion())
        {
            tangent = new Direction2D(chord);
        }
    }
}
