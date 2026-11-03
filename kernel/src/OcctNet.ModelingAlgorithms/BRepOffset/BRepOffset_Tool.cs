// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Tool.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ChFiDS_TypeOfConcavity;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>The offset algorithms' topological and geometric tools.</summary>
/// <remarks>
/// <para>
/// Tools of the offset algorithms, which, as in OCCT, write into the shapes they are given: the section tools
/// (<see cref="Inter3D"/>, <see cref="TryProject"/>, <see cref="PipeInter"/>, <see cref="InterOrExtent"/>,
/// <see cref="Inter2d"/>) build 3D curves and pcurves on the faces' and edges' own edges, raise their vertices' and
/// edges' tolerances, and (<see cref="Inter2d"/>, through BRep_Builder.UpdateVertex) record the new vertices'
/// parameters on the edges; <see cref="EnLargeFace"/> with <c>UpDatePCurve</c> moves the pcurves of the face's edges
/// onto the enlarged surface and gives the face that surface; <see cref="ExtentFace"/> puts the face's edges' pcurves
/// on the enlarged face, sets their ranges, and projects their vertices on the new edges;
/// <see cref="CorrectOrientation"/> reverses edges in the descendants' lists of the AsDes it is given. The shapes written are the offset's own working shapes (the faces and edges the offset
/// builds, and the offset's own copy of the caller's shape, unit 7E): they must be unfrozen; a frozen one throws at
/// the first write. The shapes handed out are not frozen.
/// </para>
/// <para>
/// Geometry stored in shapes is frozen, so where OCCT changes it in place the port changes a copy: the 2D curves
/// <c>PutInBounds</c> translates, and the offset surface whose basis <c>EnlargeGeometry</c> replaces (OCCT changes the
/// face's own surface there).
/// </para>
/// </remarks>
public static class BRepOffset_Tool
{
    // The constant defines the maximal value to enlarge surfaces.
    // It is limited to 1.e+7. This limitation is justified by the
    // floating point format. As we can have only 15
    // valuable decimal numbers, then during intersection of surfaces with
    // bounds of 1.e+8 the possible inaccuracy might appear already in seventh
    // decimal place which will be more than Precision::Confusion value -
    // 1.e-7, default tolerance value for the section curves.
    // By decreasing the max enlarge value to 1.e+7 the inaccuracy will be
    // shifted to eighth decimal place, i.e. the inaccuracy will be
    // decreased to values less than 1.e-7.
    private const double TheInfini = 1.0e+7;

    // OCCT_DEBUG: AffichExtent (debug output) is left out.

    //=================================================================================================

    /// <summary>
    /// <paramref name="V1"/> is the first vertex and <paramref name="V2"/> the last vertex of <paramref name="E"/>,
    /// taking account of the edge's orientation.
    /// </summary>
    public static void EdgeVertices(in Edge E, out Vertex V1, out Vertex V2)
    {
        if (E.Orientation() == Reversed)
        {
            Exploration.Vertices(E, out V2, out V1);
        }
        else
        {
            Exploration.Vertices(E, out V1, out V2);
        }
    }

    //=================================================================================================

    private static void FindPeriod(in Face F, ref double umin, ref double umax, ref double vmin, ref double vmax)
    {
        Box2d B = new();
        Explorer exp = new();
        for (exp.Init(F, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge E = ShapeCasts.Edge(exp.Current());

            Curve2D? C = Tool.CurveOnSurface(E, F, out double pf, out double pl);
            if (C is null)
            {
                return;
            }
            CurveAdaptor2D PC = new(C, pf, pl);
            double i, nbp = 20;
            if (PC.GetType() == Line)
            {
                nbp = 2;
            }
            double step = (pl - pf) / nbp;
            PC.D0(pf, out Point2D P);
            B.Add(P);
            for (i = 2; i < nbp; i++)
            {
                pf += step;
                PC.D0(pf, out P);
                B.Add(P);
            }
            PC.D0(pl, out P);
            B.Add(P);
            B.Get(out umin, out vmin, out umax, out vmax);
        }
    }

    //=======================================================================
    // function : PutInBounds
    // purpose  : Recadre la courbe 2d dans les bounds de la face
    //=======================================================================

    private static void PutInBounds(in Face F, in Edge E, ref Curve2D C2d)
    {
        // OCCT leaves these uninitialized (FindPeriod may not write them): the port starts them at 0.
        double umin = 0.0, umax = 0.0, vmin = 0.0, vmax = 0.0;
        Tool.Range(E, out double f, out double l);

        // Recup S avec la location pour eviter la copie.
        Surface S = Tool.Surface(F, out Placement L)!;

        if (S.GetType() == typeof(TrimmedSurface))
        {
            S = ((TrimmedSurface)S).BasisSurface();
        }
        //---------------
        // Recadre en U.
        //---------------
        if (!S.IsUPeriodic && !S.IsVPeriodic)
        {
            return;
        }

        FindPeriod(F, ref umin, ref umax, ref vmin, ref vmax);

        if (S.IsUPeriodic)
        {
            double period = S.UPeriod();
            double eps = period * 1.0e-6;
            Point2D Pf = C2d.Value(f);
            Point2D Pl = C2d.Value(l);
            Point2D Pm = C2d.Value(0.34 * f + 0.66 * l);
            double minC = Min(Pf.X(), Pl.X());
            minC = Min(minC, Pm.X());
            double maxC = Max(Pf.X(), Pl.X());
            maxC = Max(maxC, Pm.X());
            double du = 0.0;
            if (minC < umin - eps)
            {
                du = ((int)((umin - minC) / period) + 1) * period;
            }
            if (minC > umax + eps)
            {
                du = -((int)((minC - umax) / period) + 1) * period;
            }
            if (du != 0)
            {
                Vector2D T1 = new(du, 0.0);
                ownCurve(ref C2d);
                C2d.Translate(T1);
                minC += du;
                maxC += du;
            }
            // Ajuste au mieux la courbe dans le domaine.
            if (maxC > umax + 100 * eps)
            {
                double d1 = maxC - umax;
                double d2 = umin - minC + period;
                if (d2 < d1)
                {
                    du = -period;
                }
                if (du != 0.0)
                {
                    Vector2D T2 = new(du, 0.0);
                    ownCurve(ref C2d);
                    C2d.Translate(T2);
                }
            }
        }
        //------------------
        // Recadre en V.
        //------------------
        if (S.IsVPeriodic)
        {
            double period = S.VPeriod();
            double eps = period * 1.0e-6;
            Point2D Pf = C2d.Value(f);
            Point2D Pl = C2d.Value(l);
            Point2D Pm = C2d.Value(0.34 * f + 0.66 * l);
            double minC = Min(Pf.Y(), Pl.Y());
            minC = Min(minC, Pm.Y());
            double maxC = Max(Pf.Y(), Pl.Y());
            maxC = Max(maxC, Pm.Y());
            double dv = 0.0;
            if (minC < vmin - eps)
            {
                dv = ((int)((vmin - minC) / period) + 1) * period;
            }
            if (minC > vmax + eps)
            {
                dv = -((int)((minC - vmax) / period) + 1) * period;
            }
            if (dv != 0)
            {
                Vector2D T1 = new(0.0, dv);
                ownCurve(ref C2d);
                C2d.Translate(T1);
                minC += dv;
                maxC += dv;
            }
            // Ajuste au mieux la courbe dans le domaine.
            if (maxC > vmax + 100 * eps)
            {
                double d1 = maxC - vmax;
                double d2 = vmin - minC + period;
                if (d2 < d1)
                {
                    dv = -period;
                }
                if (dv != 0.0)
                {
                    Vector2D T2 = new(0.0, dv);
                    ownCurve(ref C2d);
                    C2d.Translate(T2);
                }
            }
        }
    }

    // OCCT.NET: OCCT translates the 2D curve it is given in place; it may be frozen (an intersector's result, or a
    // pcurve of a shape), so the port translates a copy, which the caller receives as OCCT's caller receives its own.
    private static void ownCurve(ref Curve2D C2d)
    {
        if (C2d.IsFrozen)
        {
            C2d = (Curve2D)C2d.Copy();
        }
    }

    //=================================================================================================

    /// <summary>The largest side of the curve's bounding box.</summary>
    public static double Gabarit(Curve3D aCurve)
    {
        CurveAdaptor GC = new(aCurve);
        Box aBox = new();
        Add3dCurve.Add(GC, global::Kerf.Engine.Foundation.Precision.Confusion(), ref aBox);
        double dist;
        aBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);
        dist = Max((aXmax - aXmin), (aYmax - aYmin));
        dist = Max(dist, (aZmax - aZmin));
        return dist;
    }

    //=================================================================================================

    private static void BuildPCurves(in Edge E, in Face F)
    {
        Curve2D? C2d = Tool.CurveOnSurface(E, F, out double ff, out double ll);
        if (C2d is not null)
        {
            return;
        }

        // double Tolerance = std::max(Precision::Confusion(),BRep_Tool::Tolerance(E));
        double Tolerance = global::Kerf.Engine.Foundation.Precision.Confusion();

        FaceSurfaceAdaptor AS = new(F, false);
        EdgeCurveAdaptor AC = new(E);

        // Try to find pcurve on a bound of BSpline or Bezier surface
        Surface theSurf = Tool.Surface(F)!;
        Type typS = theSurf.GetType();
        if (typS == typeof(global::Kerf.Geometry.OffsetSurface))
        {
            typS = ((global::Kerf.Geometry.OffsetSurface)theSurf).BasisSurface().GetType();
        }
        if (typS == typeof(global::Kerf.Geometry.BezierSurface) || typS == typeof(global::Kerf.Geometry.BSplineSurface))
        {
            Point3D fpoint = AC.Value(AC.FirstParameter());
            Point3D lpoint = AC.Value(AC.LastParameter());
            Face theFace = new BRepLib_MakeFace(theSurf, global::Kerf.Engine.Foundation.Precision.Confusion()).InternalFace();
            double U1 = 0.0, U2 = 0.0, TolProj = 1.0e-4; // 1.e-5;
            Edge theEdge = new();
            Explorer Explo = new();
            Explo.Init(theFace, ShapeType.Edge);
            for (; Explo.More(); Explo.Next())
            {
                Edge anEdge = ShapeCasts.Edge(Explo.Current());
                EdgeCurveAdaptor aCurve = new(anEdge);
                CurvePointExtrema fextr = new(fpoint, aCurve);
                if (!fextr.IsDone || fextr.ExtCount < 1)
                {
                    continue;
                }
                double dist2, dist2min = RealLast();
                int i;
                for (i = 1; i <= fextr.ExtCount; i++)
                {
                    dist2 = fextr.SquareDistance(i);
                    if (dist2 < dist2min)
                    {
                        dist2min = dist2;
                        U1 = fextr.Point(i).Parameter();
                    }
                }
                if (dist2min > TolProj * TolProj)
                {
                    continue;
                }
                CurvePointExtrema lextr = new(lpoint, aCurve);
                if (!lextr.IsDone || lextr.ExtCount < 1)
                {
                    continue;
                }
                dist2min = RealLast();
                for (i = 1; i <= lextr.ExtCount; i++)
                {
                    dist2 = lextr.SquareDistance(i);
                    if (dist2 < dist2min)
                    {
                        dist2min = dist2;
                        U2 = lextr.Point(i).Parameter();
                    }
                }
                if (dist2min <= TolProj * TolProj)
                {
                    theEdge = anEdge;
                    break;
                }
            } // for (; Explo.More(); Explo.Current())

            if (!theEdge.IsNull)
            {
                // Construction of pcurve
                if (U2 < U1)
                {
                    double temp = U1;
                    U1 = U2;
                    U2 = temp;
                }
                C2d = Tool.CurveOnSurface(theEdge, theFace, out double f, out double l);
                C2d = new TrimmedCurve2D(C2d!, U1, U2);

                if (theSurf.IsUPeriodic || theSurf.IsVPeriodic)
                {
                    PutInBounds(F, E, ref C2d);
                }

                BrepBuilder B = new();
                B.UpdateEdge(E, C2d, F, Tool.Tolerance(E));
                BRepLib.SameRange(E);

                return;
            }
        } // if (typS == ...

        FaceSurfaceAdaptor HS = (FaceSurfaceAdaptor)AS.ShallowCopy();
        EdgeCurveAdaptor HC = (EdgeCurveAdaptor)AC.ShallowCopy();

        ProjectedCurve Proj = new(HS, HC, Tolerance);

        switch (Proj.GetType())
        {
            case Line:
                C2d = new LineCurve2D(Proj.Line());
                break;

            case Circle:
                C2d = new CircleCurve2D(Proj.Circle());
                break;

            case Ellipse:
                C2d = new EllipseCurve2D(Proj.Ellipse());
                break;

            case Parabola:
                C2d = new ParabolaCurve2D(Proj.Parabola());
                break;

            case Hyperbola:
                C2d = new HyperbolaCurve2D(Proj.Hyperbola());
                break;

            case BezierCurve:
                C2d = Proj.Bezier();
                break;

            case BSplineCurve:
                C2d = Proj.BSpline();
                break;
            default:
                break;
        }

        if (AS.IsUPeriodic || AS.IsVPeriodic)
        {
            // OCCT: C2d may still be null here (another type of projection), and PutInBounds dereferences it.
            PutInBounds(F, E, ref C2d!);
        }
        if (C2d is not null)
        {
            BrepBuilder B = new();
            B.UpdateEdge(E, C2d, F, Tool.Tolerance(E));
        }
        else
        {
            throw new ConstructionErrorException("BRepOffset_Tool::BuildPCurves");
        }
    }

    //=================================================================================================

    /// <summary>
    /// <paramref name="E"/> is a section between <paramref name="F1"/> and <paramref name="F2"/>: computes
    /// <paramref name="O1"/>, the orientation of E in F1 influenced by F2, and <paramref name="O2"/> likewise.
    /// </summary>
    public static void OrientSection(in Edge E, in Face F1, in Face F2,
                                     out Orientation O1, out Orientation O2)
    {
        double f, l;

        Surface S1 = Tool.Surface(F1)!;
        Surface S2 = Tool.Surface(F2)!;
        Curve2D C1 = Tool.CurveOnSurface(E, F1, out f, out l)!;
        Curve2D C2 = Tool.CurveOnSurface(E, F2, out f, out l)!;
        Curve3D C = Tool.Curve(E, out Placement L, out f, out l)!;

        EdgeCurveAdaptor BAcurve = new(E);

        AbscissaPoint AP = new(BAcurve, AbscissaPoint.Length(BAcurve) / 2.0, f);
        double ParOnC;

        if (AP.IsDone)
        {
            ParOnC = AP.Parameter();
        }
        else
        {
            ParOnC = BOPTools_AlgoTools2D.IntermediatePoint(f, l);
        }

        Vector3D T1 = C.DN(ParOnC, 1).Transformed(L.Transformation());
        if (T1.SquareMagnitude() > Constants.Resolution())
        {
            T1.Normalize();
        }

        Point2D P = C1.Value(ParOnC);

        S1.D1(P.X(), P.Y(), out Point3D P3, out Vector3D D1U, out Vector3D D1V);
        Vector3D DN1 = D1U ^ D1V;
        if (F1.Orientation() == Reversed)
        {
            DN1.Reverse();
        }

        P = C2.Value(ParOnC);
        S2.D1(P.X(), P.Y(), out P3, out D1U, out D1V);
        Vector3D DN2 = D1U ^ D1V;
        if (F2.Orientation() == Reversed)
        {
            DN2.Reverse();
        }

        Vector3D ProVec = DN2 ^ T1;
        double Prod = DN1.Dot(ProVec);
        if (Prod < 0.0)
        {
            O1 = Forward;
        }
        else
        {
            O1 = Reversed;
        }
        ProVec = DN1 ^ T1;
        Prod = DN2.Dot(ProVec);
        if (Prod < 0.0)
        {
            O2 = Forward;
        }
        else
        {
            O2 = Reversed;
        }
        if (F1.Orientation() == Reversed)
        {
            O1 = global::Kerf.Engine.Topology.TopAbs.Reverse(O1);
        }
        if (F2.Orientation() == Reversed)
        {
            O2 = global::Kerf.Engine.Topology.TopAbs.Reverse(O2);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Looks for the common vertices and edges of the faces <paramref name="theF1"/> and <paramref name="theF2"/>:
    /// <paramref name="theLE"/> receives the common edges, <paramref name="theLV"/> the common vertices. True if any
    /// was found.
    /// </summary>
    public static bool FindCommonShapes(in Face theF1, in Face theF2, NCollection_List<Shape> theLE,
                                        NCollection_List<Shape> theLV)
    {
        bool bFoundEdges = FindCommonShapes(theF1, theF2, ShapeType.Edge, theLE);
        bool bFoundVerts = FindCommonShapes(theF1, theF2, ShapeType.Vertex, theLV);
        return bFoundEdges || bFoundVerts;
    }

    //=================================================================================================

    /// <summary>
    /// Looks for the common sub-shapes of type <paramref name="theType"/> of <paramref name="theS1"/> and
    /// <paramref name="theS2"/>, which <paramref name="theLSC"/> receives. True if any was found.
    /// </summary>
    public static bool FindCommonShapes(in Shape theS1, in Shape theS2, ShapeType theType,
                                        NCollection_List<Shape> theLSC)
    {
        theLSC.Clear();
        //
        NCollection_Map<Shape> aMS = new(ShapeHasher.Instance);
        Explorer aExp = new(theS1, theType);
        for (; aExp.More(); aExp.Next())
        {
            aMS.Add(aExp.Current());
        }
        //
        if (aMS.IsEmpty())
        {
            return false;
        }
        //
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
        aExp.Init(theS2, theType);
        for (; aExp.More(); aExp.Next())
        {
            Shape aS2 = aExp.Current();
            if (aMS.Contains(aS2))
            {
                if (aMFence.Add(aS2))
                {
                    theLSC.Append(aS2);
                }
            }
        }
        //
        return !theLSC.IsEmpty();
    }

    //=================================================================================================

    private static bool ToSmall(Curve3D C)
    {
        double Tol = 10 * global::Kerf.Engine.Foundation.Precision.Confusion();
        double m = (C.FirstParameter() * 0.668 + C.LastParameter() * 0.332);
        Point3D P1 = C.Value(C.FirstParameter());
        Point3D P2 = C.Value(C.LastParameter());
        Point3D P3 = C.Value(m);
        if (P1.Distance(P2) > Tol)
        {
            return false;
        }
        if (P2.Distance(P3) > Tol)
        {
            return false;
        }
        return true;
    }

    //=================================================================================================

    private static bool IsOnSurface(Curve3D C, Surface S, double TolConf, out double TolReached)
    {
        double f = C.FirstParameter();
        double l = C.LastParameter();
        int n = 5;
        double du = (f - l) / (n - 1); // OCCT: (f - l), so the points sampled run from f away from l
        TolReached = 0.0;

        Point3D P;
        double U, V;

        SurfaceAdaptor AS = new(S);

        switch (AS.GetType())
        {
            case Plane:
            {
                CoordinateSystem Ax = AS.Plane().Position();
                for (int i = 0; i < n; i++)
                {
                    P = C.Value(f + i * du);
                    global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(Ax, P, out U, out V);
                    TolReached = P.Distance(global::Kerf.Engine.Geometry.ElSLib.PlaneValue(U, V, Ax));
                    if (TolReached > TolConf)
                    {
                        return false;
                    }
                }
                break;
            }
            case SurfaceKind.Cylinder:
            {
                CoordinateSystem Ax = AS.Cylinder().Position();
                double Rad = AS.Cylinder().Radius();
                for (int i = 0; i < n; i++)
                {
                    P = C.Value(f + i * du);
                    global::Kerf.Engine.Geometry.ElSLib.CylinderParameters(Ax, Rad, P, out U, out V);
                    TolReached = P.Distance(global::Kerf.Engine.Geometry.ElSLib.CylinderValue(U, V, Ax, Rad));
                    if (TolReached > TolConf)
                    {
                        return false;
                    }
                }
                break;
            }
            case SurfaceKind.Cone:
            {
                CoordinateSystem Ax = AS.Cone().Position();
                double Rad = AS.Cone().RefRadius();
                double Alp = AS.Cone().SemiAngle();
                for (int i = 0; i < n; i++)
                {
                    P = C.Value(f + i * du);
                    global::Kerf.Engine.Geometry.ElSLib.ConeParameters(Ax, Rad, Alp, P, out U, out V);
                    TolReached = P.Distance(global::Kerf.Engine.Geometry.ElSLib.ConeValue(U, V, Ax, Rad, Alp));
                    if (TolReached > TolConf)
                    {
                        return false;
                    }
                }
                break;
            }
            case SurfaceKind.Sphere:
            {
                CoordinateSystem Ax = AS.Sphere().Position();
                double Rad = AS.Sphere().Radius();
                for (int i = 0; i < n; i++)
                {
                    P = C.Value(f + i * du);
                    global::Kerf.Engine.Geometry.ElSLib.SphereParameters(Ax, Rad, P, out U, out V);
                    TolReached = P.Distance(global::Kerf.Engine.Geometry.ElSLib.SphereValue(U, V, Ax, Rad));
                    if (TolReached > TolConf)
                    {
                        return false;
                    }
                }
                break;
            }
            case SurfaceKind.Torus:
            {
                CoordinateSystem Ax = AS.Torus().Position();
                double R1 = AS.Torus().MajorRadius();
                double R2 = AS.Torus().MinorRadius();
                for (int i = 0; i < n; i++)
                {
                    P = C.Value(f + i * du);
                    global::Kerf.Engine.Geometry.ElSLib.TorusParameters(Ax, R1, R2, P, out U, out V);
                    TolReached = P.Distance(global::Kerf.Engine.Geometry.ElSLib.TorusValue(U, V, Ax, R1, R2));
                    if (TolReached > TolConf)
                    {
                        return false;
                    }
                }
                break;
            }

            default:
            {
                return false;
            }
        }

        return true;
    }

    //=================================================================================================

    /// <summary>
    /// The sections of the surfaces of <paramref name="F1"/> and <paramref name="F2"/> as new edges with pcurves on
    /// both faces: <paramref name="L1"/> receives them oriented on F1, <paramref name="L2"/> on F2.
    /// </summary>
    public static void PipeInter(in Face F1, in Face F2, NCollection_List<Shape> L1,
                                 NCollection_List<Shape> L2, Containment Side)
    {
        Curve3D CI;
        L1.Clear();
        L2.Clear();
        BrepBuilder B = new();
        Surface S1 = Tool.Surface(F1)!;
        Surface S2 = Tool.Surface(F2)!;

        GeomInt_IntSS Inter = new(S1, S2, global::Kerf.Engine.Foundation.Precision.Confusion(), true, true, true);

        if (Inter.IsDone())
        {
            for (int i = 1; i <= Inter.NbLines(); i++)
            {
                CI = Inter.Line(i);
                if (ToSmall(CI))
                {
                    continue;
                }
                Edge E = new BRepLib_MakeEdge(CI).InternalEdge();
                if (Inter.HasLineOnS1(i))
                {
                    Curve2D C2 = Inter.LineOnS1(i)!;
                    PutInBounds(F1, E, ref C2);
                    B.UpdateEdge(E, C2, F1, Tool.Tolerance(E));
                }
                else
                {
                    BuildPCurves(E, F1);
                }
                if (Inter.HasLineOnS2(i))
                {
                    Curve2D C2 = Inter.LineOnS2(i)!;
                    PutInBounds(F2, E, ref C2);
                    B.UpdateEdge(E, C2, F2, Tool.Tolerance(E));
                }
                else
                {
                    BuildPCurves(E, F2);
                }
                OrientSection(E, F1, F2, out Orientation O1, out Orientation O2);
                if (Side == Out)
                {
                    O1 = global::Kerf.Engine.Topology.TopAbs.Reverse(O1);
                    O2 = global::Kerf.Engine.Topology.TopAbs.Reverse(O2);
                }
                L1.Append(E.Oriented(O1));
                L2.Append(E.Oriented(O2));
            }
        }
    }

    //=======================================================================
    // function : IsAutonomVertex
    // purpose  : Checks whether a vertex is "autonom" or not
    //=======================================================================

    private static bool IsAutonomVertex(in Shape theVertex, BOPDS_DS thePDS, in Face theFace1,
                                        in Face theFace2)
    {
        int nV = thePDS.Index(theVertex);
        Span<int> nF = stackalloc int[2];
        nF[0] = thePDS.Index(theFace1);
        nF[1] = thePDS.Index(theFace2);

        for (int i = 0; i < 2; i++)
        {
            BOPDS_FaceInfo aFaceInfo = thePDS.FaceInfo(nF[i]);
            NCollection_IntMap IndMap = aFaceInfo.VerticesOn();
            if (IndMap.Contains(nV))
            {
                return false;
            }
        }

        return true;
    }

    //=======================================================================
    // function : IsAutonomVertex
    // purpose  : Checks whether a vertex is "autonom" or not
    //=======================================================================

    private static bool IsAutonomVertex(in Shape aVertex, BOPDS_DS pDS)
    {
        int index;
        int aNbVVs, aNbEEs, aNbEFs, aInt;
        //
        index = pDS.Index(aVertex);
        if (index == -1)
        {
            int i, i1, i2;
            i1 = pDS.NbSourceShapes();
            i2 = pDS.NbShapes();
            for (i = i1; i < i2; ++i)
            {
                Shape aSx = pDS.Shape(i);
                if (aSx.IsSame(aVertex))
                {
                    index = i;
                    break;
                }
            }
        }
        //
        if (!pDS.IsNewShape(index))
        {
            return false;
        }
        // check if vertex with index "index" is not created in VV or EE or EF interference
        // VV
        NCollection_DynamicArray<BOPDS_InterfVV> aVVs = pDS.InterfVV();
        aNbVVs = aVVs.Length();
        for (aInt = 0; aInt < aNbVVs; aInt++)
        {
            BOPDS_InterfVV aVV = aVVs[aInt];
            if (aVV.HasIndexNew())
            {
                if (aVV.IndexNew() == index)
                {
                    return false;
                }
            }
        }
        // EE
        NCollection_DynamicArray<BOPDS_InterfEE> aEEs = pDS.InterfEE();
        aNbEEs = aEEs.Length();
        for (aInt = 0; aInt < aNbEEs; aInt++)
        {
            BOPDS_InterfEE aEE = aEEs[aInt];
            IntTools_CommonPrt aCP = aEE.CommonPart();
            if (aCP.Type() == ShapeType.Vertex)
            {
                if (aEE.IndexNew() == index)
                {
                    return false;
                }
            }
        }
        // EF
        NCollection_DynamicArray<BOPDS_InterfEF> aEFs = pDS.InterfEF();
        aNbEFs = aEFs.Length();
        for (aInt = 0; aInt < aNbEFs; aInt++)
        {
            BOPDS_InterfEF aEF = aEFs[aInt];
            IntTools_CommonPrt aCP = aEF.CommonPart();
            if (aCP.Type() == ShapeType.Vertex)
            {
                if (aEF.IndexNew() == index)
                {
                    return false;
                }
            }
        }
        return true;
    }

    //=======================================================================
    // function : AreConnex
    // purpose  : define if two shapes are connex by a vertex (vertices)
    //=======================================================================

    private static bool AreConnex(in Wire W1, in Wire W2)
    {
        Exploration.Vertices(W1, out Vertex V11, out Vertex V12);
        Exploration.Vertices(W2, out Vertex V21, out Vertex V22);

        return V11.IsSame(V21) || V11.IsSame(V22) || V12.IsSame(V21) || V12.IsSame(V22);
    }

    //=======================================================================
    // function : AreClosed
    // purpose  : define if two edges are connex by two vertices
    //=======================================================================

    private static bool AreClosed(in Edge E1, in Edge E2)
    {
        Exploration.Vertices(E1, out Vertex V11, out Vertex V12);
        Exploration.Vertices(E2, out Vertex V21, out Vertex V22);

        return (V11.IsSame(V21) && V12.IsSame(V22)) || (V11.IsSame(V22) && V12.IsSame(V21));
    }

    //=================================================================================================

    private static bool BSplineEdges(in Edge E1, in Edge E2, int par1, int par2, ref double angle)
    {
        double Param1, Param2;

        Curve3D C1 = Tool.Curve(E1, out double first1, out double last1)!;
        if (C1.GetType() == typeof(TrimmedCurve3D))
        {
            C1 = ((TrimmedCurve3D)C1).BasisCurve();
        }

        Curve3D C2 = Tool.Curve(E2, out double first2, out double last2)!;
        if (C2.GetType() == typeof(TrimmedCurve3D))
        {
            C2 = ((TrimmedCurve3D)C2).BasisCurve();
        }

        if (C1.GetType() != typeof(BSplineCurve3D) || C2.GetType() != typeof(BSplineCurve3D))
        {
            return false;
        }

        Param1 = (par1 == 0) ? first1 : last1;
        Param2 = (par2 == 0) ? first2 : last2;

        C1.D1(Param1, out Point3D Pnt1, out Vector3D Der1);
        C2.D1(Param2, out Point3D Pnt2, out Vector3D Der2);

        if (Der1.Magnitude() <= Constants.Resolution() || Der2.Magnitude() <= Constants.Resolution())
        {
            angle = M_PI / 2.0;
        }
        else
        {
            angle = Der1.Angle(Der2);
        }

        return true;
    }

    //=================================================================================================

    private static double AngleWireEdge(in Wire aWire, in Edge anEdge)
    {
        Vertex CV;
        Exploration.Vertices(aWire, out Vertex V11, out Vertex V12);
        Exploration.Vertices(anEdge, out Vertex V21, out Vertex V22);
        CV = (V11.IsSame(V21) || V11.IsSame(V22)) ? V11 : V12;
        Edge FirstEdge = new();
        Iterator itw = new(aWire);
        for (; itw.More(); itw.Next())
        {
            FirstEdge = ShapeCasts.Edge(itw.Value());
            Exploration.Vertices(FirstEdge, out Vertex v1, out Vertex v2);
            if (v1.IsSame(CV) || v2.IsSame(CV))
            {
                V11 = v1;
                V12 = v2;
                break;
            }
        }
        // OCCT leaves Angle uninitialized, and BSplineEdges doesn't set it unless both edges are B-splines: the port
        // starts it at 0.
        double Angle = 0.0;
        if (V11.IsSame(CV) && V21.IsSame(CV))
        {
            BSplineEdges(FirstEdge, anEdge, 0, 0, ref Angle);
            Angle = M_PI - Angle;
        }
        else if (V11.IsSame(CV) && V22.IsSame(CV))
        {
            BSplineEdges(FirstEdge, anEdge, 0, 1, ref Angle);
        }
        else if (V12.IsSame(CV) && V21.IsSame(CV))
        {
            BSplineEdges(FirstEdge, anEdge, 1, 0, ref Angle);
        }
        else
        {
            BSplineEdges(FirstEdge, anEdge, 1, 1, ref Angle);
            Angle = M_PI - Angle;
        }
        return Angle;
    }

    //=================================================================================================

    private static void ReconstructPCurves(in Edge anEdge)
    {
        Curve3D C3d = Tool.Curve(anEdge, out double f, out double l)!;

        NCollection_List<CurveRepresentation>.Iterator itcr = new(((BrepTEdge)anEdge.TShape()!).ChangeCurves());
        for (; itcr.More(); itcr.Next())
        {
            CurveRepresentation CurveRep = itcr.Value();
            if (CurveRep.IsCurveOnSurface())
            {
                Surface theSurf = CurveRep.Surface()!;
                Placement theLoc = CurveRep.Location();
                theLoc = anEdge.Location() * theLoc;
                theSurf = (Surface)theSurf.Transformed(theLoc.Transformation());
                Curve2D? ProjPCurve = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(C3d, f, l, theSurf);
                if (ProjPCurve is not null)
                {
                    CurveRep.PCurve(ProjPCurve);
                }
            }
        }
    }

    //=================================================================================================

    private static Curve2D ConcatPCurves(in Edge E1, in Edge E2, in Face F, bool After,
                                              out double newFirst, out double newLast)
    {
        double Tol = 1.0e-7;
        Continuity Continuity = C1;
        int MaxDeg = 14;
        int MaxSeg = 16;

        Curve2D PCurve1, PCurve2, newPCurve;

        PCurve1 = Tool.CurveOnSurface(E1, F, out double first1, out double last1)!;
        if (PCurve1.GetType() == typeof(TrimmedCurve2D))
        {
            PCurve1 = ((TrimmedCurve2D)PCurve1).BasisCurve();
        }

        PCurve2 = Tool.CurveOnSurface(E2, F, out double first2, out double last2)!;
        if (PCurve2.GetType() == typeof(TrimmedCurve2D))
        {
            PCurve2 = ((TrimmedCurve2D)PCurve2).BasisCurve();
        }

        if (PCurve1 == PCurve2)
        {
            newPCurve = PCurve1;
            newFirst = Min(first1, first2);
            newLast = Max(last1, last2);
        }
        else if (PCurve1.GetType() == PCurve2.GetType()
                 && (PCurve1.GetType() == typeof(LineCurve2D) || PCurve1 is ConicCurve2D))
        {
            newPCurve = PCurve1;
            Point2D P1, P2;
            P1 = PCurve2.Value(first2);
            P2 = PCurve2.Value(last2);
            if (PCurve1.GetType() == typeof(LineCurve2D))
            {
                LineCurve2D Lin1 = (LineCurve2D)PCurve1;
                Line2D theLin = Lin1.Lin2d();
                first2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin, P1);
                last2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theLin, P2);
            }
            else if (PCurve1.GetType() == typeof(CircleCurve2D))
            {
                CircleCurve2D Circ1 = (CircleCurve2D)PCurve1;
                Circle2D theCirc = Circ1.Circ2d();
                first2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theCirc, P1);
                last2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theCirc, P2);
            }
            else if (PCurve1.GetType() == typeof(EllipseCurve2D))
            {
                EllipseCurve2D Ell1 = (EllipseCurve2D)PCurve1;
                Ellipse2D theElips = Ell1.Elips2d();
                first2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theElips, P1);
                last2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theElips, P2);
            }
            else if (PCurve1.GetType() == typeof(ParabolaCurve2D))
            {
                ParabolaCurve2D Parab1 = (ParabolaCurve2D)PCurve1;
                Parabola2D theParab = Parab1.Parab2d();
                first2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theParab, P1);
                last2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theParab, P2);
            }
            else if (PCurve1.GetType() == typeof(HyperbolaCurve2D))
            {
                HyperbolaCurve2D Hypr1 = (HyperbolaCurve2D)PCurve1;
                Hyperbola2D theHypr = Hypr1.Hypr2d();
                first2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theHypr, P1);
                last2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(theHypr, P2);
            }
            newFirst = Min(first1, first2);
            newLast = Max(last1, last2);
        }
        else
        {
            TrimmedCurve2D TC1 = new(PCurve1, first1, last1);
            TrimmedCurve2D TC2 = new(PCurve2, first2, last2);
            CompCurveToBSplineCurve2D Concat2d = new(TC1);
            Concat2d.Add(TC2, global::Kerf.Engine.Foundation.Precision.Confusion(), After);
            newPCurve = Concat2d.BSplineCurve()!;
            if (newPCurve.Continuity() < C1)
            {
                ApproxCurve2D Approx2d = new(newPCurve, Tol, Continuity, MaxSeg, MaxDeg);
                if (Approx2d.HasResult)
                {
                    newPCurve = Approx2d.Curve()!;
                }
            }
            newFirst = newPCurve.FirstParameter();
            newLast = newPCurve.LastParameter();
        }

        return newPCurve;
    }

    //=================================================================================================

    private static Edge Glue(in Edge E1, in Edge E2, in Vertex Vfirst, in Vertex Vlast,
                                    bool After, in Face F1, bool addPCurve1, in Face F2, bool addPCurve2,
                                    double theGlueTol)
    {
        Edge newEdge = new();

        double Tol = 1.0e-7;
        Continuity Continuity = global::Kerf.Geometry.Continuity.C1;
        int MaxDeg = 14;
        int MaxSeg = 16;

        Curve3D C1, C2, newCurve;
        Curve2D newPCurve;
        double fparam = 0.0, lparam = 0.0;
        bool IsCanonic = false;

        C1 = Tool.Curve(E1, out double first1, out double last1)!;
        if (C1.GetType() == typeof(TrimmedCurve3D))
        {
            C1 = ((TrimmedCurve3D)C1).BasisCurve();
        }

        C2 = Tool.Curve(E2, out double first2, out double last2)!;
        if (C2.GetType() == typeof(TrimmedCurve3D))
        {
            C2 = ((TrimmedCurve3D)C2).BasisCurve();
        }

        if (C1 == C2)
        {
            newCurve = C1;
            fparam = Min(first1, first2);
            lparam = Max(last1, last2);
        }
        else if (C1.GetType() == C2.GetType() && (C1.GetType() == typeof(LineCurve3D) || C1 is ConicCurve3D))
        {
            IsCanonic = true;
            newCurve = C1;
        }
        else
        {
            TrimmedCurve3D TC1 = new(C1, first1, last1);
            TrimmedCurve3D TC2 = new(C2, first2, last2);
            CompCurveToBSplineCurve Concat = new(TC1);
            if (!Concat.Add(TC2, theGlueTol, After))
            {
                return newEdge;
            }
            newCurve = Concat.BSplineCurve()!;
            if (newCurve.Continuity() < global::Kerf.Geometry.Continuity.C1)
            {
                ApproxCurve Approx3d = new(newCurve, Tol, Continuity, MaxSeg, MaxDeg);
                if (Approx3d.HasResult)
                {
                    newCurve = Approx3d.Curve()!;
                }
            }
            fparam = newCurve.FirstParameter();
            lparam = newCurve.LastParameter();
        }

        BrepBuilder BB = new();

        if (IsCanonic)
        {
            newEdge = new BRepLib_MakeEdge(newCurve, Vfirst, Vlast).InternalEdge();
        }
        else
        {
            newEdge = new BRepLib_MakeEdge(newCurve, Vfirst, Vlast, fparam, lparam).InternalEdge();
        }

        double newFirst, newLast;
        if (addPCurve1)
        {
            newPCurve = ConcatPCurves(E1, E2, F1, After, out newFirst, out newLast);
            BB.UpdateEdge(newEdge, newPCurve, F1, 0.0);
            BB.Range(newEdge, F1, newFirst, newLast);
        }
        if (addPCurve2)
        {
            newPCurve = ConcatPCurves(E1, E2, F2, After, out newFirst, out newLast);
            BB.UpdateEdge(newEdge, newPCurve, F2, 0.0);
            BB.Range(newEdge, F2, newFirst, newLast);
        }

        return newEdge;
    }

    //=================================================================================================

    private static void CheckIntersFF(BOPDS_DS pDS, in Edge RefEdge, NCollection_IndexedMap<Shape> TrueEdges)
    {
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = pDS.InterfFF();
        int aNb = aFFs.Length();
        int i, j, nbe = 0;

        Compound Edges = new();
        BrepBuilder BB = new();
        BB.MakeCompound(ref Edges);

        for (i = 0; i < aNb; ++i)
        {
            BOPDS_InterfFF aFFi = aFFs[i];
            NCollection_DynamicArray<BOPDS_Curve> aBCurves = aFFi.Curves();
            int aNbCurves = aBCurves.Length();

            for (j = 0; j < aNbCurves; ++j)
            {
                BOPDS_Curve aBC = aBCurves[j];
                NCollection_List<BOPDS_PaveBlock> aSectEdges = aBC.PaveBlocks();

                NCollection_List<BOPDS_PaveBlock>.Iterator aPBIt = new();
                aPBIt.Initialize(aSectEdges);

                for (; aPBIt.More(); aPBIt.Next())
                {
                    BOPDS_PaveBlock aPB = aPBIt.Value();
                    int nSect = aPB.Edge();
                    Edge anEdge = ShapeCasts.Edge(pDS.Shape(nSect));
                    BB.Add(Edges, anEdge);
                    nbe++;
                }
            }
        }

        if (nbe == 0)
        {
            return;
        }

        NCollection_List<Shape> CompList = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(Edges, ShapeType.Vertex, ShapeType.Edge, CompList);

        Shape NearestCompound = new();
        if (CompList.Extent() == 1)
        {
            NearestCompound = CompList.First();
        }
        else
        {
            EdgeCurveAdaptor BAcurve = new(RefEdge);
            Point3D Pref = BAcurve.Value((BAcurve.FirstParameter() + BAcurve.LastParameter()) / 2);
            Vertex Vref = new BRepLib_MakeVertex(Pref).InternalVertex();
            double MinDist = RealLast();
            NCollection_List<Shape>.Iterator itl = new(CompList);
            for (; itl.More(); itl.Next())
            {
                Shape aCompound = itl.Value();

                BRepExtrema_DistShapeShape Projector = new(Vref, aCompound);
                if (!Projector.IsDone() || Projector.NbSolution() == 0)
                {
                    continue;
                }

                double aDist = Projector.Value();
                if (aDist < MinDist)
                {
                    MinDist = aDist;
                    NearestCompound = aCompound;
                }
            }
        }

        Exploration.MapShapes(NearestCompound, ShapeType.Edge, TrueEdges);
    }

    //=================================================================================================

    private static Edge AssembleEdge(BOPDS_DS pDS, in Face F1, in Face F2, bool addPCurve1,
                                            bool addPCurve2, NCollection_Sequence<Shape> EdgesForConcat)
    {
        Edge NullEdge = new();
        Edge CurEdge = ShapeCasts.Edge(EdgesForConcat[1]);
        double aGlueTol = global::Kerf.Engine.Foundation.Precision.Confusion();

        for (int j = 2; j <= EdgesForConcat.Length(); j++)
        {
            Edge anEdge = ShapeCasts.Edge(EdgesForConcat[j]);
            bool After = false;
            Vertex Vfirst = new(), Vlast = new();
            bool AreClosedWire = AreClosed(CurEdge, anEdge);
            if (AreClosedWire)
            {
                Exploration.Vertices(CurEdge, out Vertex V1, out Vertex V2);
                bool IsAutonomV1 = IsAutonomVertex(V1, pDS, F1, F2);
                bool IsAutonomV2 = IsAutonomVertex(V2, pDS, F1, F2);
                if (IsAutonomV1)
                {
                    After = false;
                    Vfirst = Vlast = V2;
                }
                else if (IsAutonomV2)
                {
                    After = true;
                    Vfirst = Vlast = V1;
                }
                else
                {
                    return NullEdge;
                }
            }
            else
            {
                Vertex CV = new();
                Exploration.CommonVertex(CurEdge, anEdge, ref CV);
                bool IsAutonomCV = false;
                if (!CV.IsNull)
                {
                    IsAutonomCV = IsAutonomVertex(CV, pDS, F1, F2);
                }
                if (IsAutonomCV)
                {
                    aGlueTol = Tool.Tolerance(CV);
                    Exploration.Vertices(CurEdge, out Vertex V11, out Vertex V12);
                    Exploration.Vertices(anEdge, out Vertex V21, out Vertex V22);
                    if (V11.IsSame(CV) && V21.IsSame(CV))
                    {
                        Vfirst = V22;
                        Vlast = V12;
                    }
                    else if (V11.IsSame(CV) && V22.IsSame(CV))
                    {
                        Vfirst = V21;
                        Vlast = V12;
                    }
                    else if (V12.IsSame(CV) && V21.IsSame(CV))
                    {
                        Vfirst = V11;
                        Vlast = V22;
                    }
                    else
                    {
                        Vfirst = V11;
                        Vlast = V21;
                    }
                }
                else
                {
                    return NullEdge;
                }
            } // end of else (open wire)

            Edge NewEdge = Glue(CurEdge, anEdge, Vfirst, Vlast, After, F1, addPCurve1, F2, addPCurve2, aGlueTol);
            if (NewEdge.IsNull)
            {
                return NullEdge;
            }
            else
            {
                CurEdge = NewEdge;
            }
        } // end of for (int j = 2; j <= EdgesForConcat.Length(); j++)

        return CurEdge;
    }

    //=================================================================================================

    /// <summary>
    /// The section of <paramref name="F1"/> and <paramref name="F2"/>: <paramref name="LInt1"/> receives its edges
    /// oriented on F1, <paramref name="LInt2"/> the same edges oriented on F2. With a reference edge, the section
    /// edges nearest to it are kept and glued where they can be.
    /// </summary>
    public static void Inter3D(in Face F1, in Face F2, NCollection_List<Shape> LInt1,
                               NCollection_List<Shape> LInt2, Containment Side, in Edge RefEdge,
                               in Face RefFace1, in Face RefFace2)
    {
        NCollection_List<Shape> L1 = LInt1;
        NCollection_List<Shape> L2 = LInt2;

        // Check if the faces are planar and not trimmed - in this case
        // the IntTools_FaceFace intersection algorithm will be used directly.
        FaceSurfaceAdaptor aBAS1 = new(F1, false), aBAS2 = new(F2, false);
        if (aBAS1.GetType() == Plane && aBAS2.GetType() == Plane)
        {
            aBAS1.Initialize(F1, true);
            if (IsInf(aBAS1.LastUParameter()) && IsInf(aBAS1.LastVParameter()))
            {
                aBAS2.Initialize(F2, true);
                if (IsInf(aBAS2.LastUParameter()) && IsInf(aBAS2.LastVParameter()))
                {
                    // Intersect the planes without pave filler
                    PerformPlanes(F1, F2, Side, L1, L2);
                    return;
                }
            }
        }

        // create 3D curves on faces
        BRepLib.BuildCurves3d(F1);
        BRepLib.BuildCurves3d(F2);
        UpdateVertexTolerances(F1);
        UpdateVertexTolerances(F2);

        BOPAlgo_PaveFiller aPF = new();
        NCollection_List<Shape> aLS = new();
        aLS.Append(F1);
        aLS.Append(F2);
        aPF.SetArguments(aLS);
        //
        aPF.Perform();

        NCollection_IndexedMap<Shape> TrueEdges = new(ShapeHasher.Instance);
        if (!RefEdge.IsNull)
        {
            CheckIntersFF(aPF.PDS()!, RefEdge, TrueEdges);
        }

        bool addPCurve1 = true;
        bool addPCurve2 = true;

        BOPDS_DS pDS = aPF.PDS()!;
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = pDS.InterfFF();
        int aNb = aFFs.Length();
        int i = 0, j = 0, k;
        // Store Result
        L1.Clear();
        L2.Clear();
        Orientation O1, O2;
        BrepBuilder BB = new();
        //
        IntTools_Context? aContext = aPF.Context();
        //
        for (i = 0; i < aNb; i++)
        {
            BOPDS_InterfFF aFFi = aFFs[i];
            NCollection_DynamicArray<BOPDS_Curve> aBCurves = aFFi.Curves();

            int aNbCurves = aBCurves.Length();

            for (j = 0; j < aNbCurves; j++)
            {
                BOPDS_Curve aBC = aBCurves[j];
                NCollection_List<BOPDS_PaveBlock> aSectEdges = aBC.PaveBlocks();

                NCollection_List<BOPDS_PaveBlock>.Iterator aPBIt = new();
                aPBIt.Initialize(aSectEdges);

                for (; aPBIt.More(); aPBIt.Next())
                {
                    BOPDS_PaveBlock aPB = aPBIt.Value();
                    int nSect = aPB.Edge();
                    Edge anEdge = ShapeCasts.Edge(pDS.Shape(nSect));
                    if (!TrueEdges.IsEmpty() && !TrueEdges.Contains(anEdge))
                    {
                        continue;
                    }

                    Curve3D? aC3DE = Tool.Curve(anEdge, out double f, out double l);
                    TrimmedCurve3D? aC3DETrim = null;

                    if (aC3DE is not null)
                    {
                        aC3DETrim = new TrimmedCurve3D(aC3DE, f, l);
                    }

                    double aTolEdge = Tool.Tolerance(anEdge);

                    if (!BOPTools_AlgoTools2D.HasCurveOnSurface(anEdge, F1))
                    {
                        Curve2D? aC2d = aBC.Curve().FirstCurve2d();
                        if (aC3DETrim is not null)
                        {
                            Curve2D? aC2dNew;

                            if (aC3DE!.IsPeriodic)
                            {
                                BOPTools_AlgoTools2D.AdjustPCurveOnFace(F1, f, l, aC2d!, out aC2dNew, aContext);
                            }
                            else
                            {
                                BOPTools_AlgoTools2D.AdjustPCurveOnFace(F1, aC3DETrim, aC2d!, out aC2dNew, aContext);
                            }
                            aC2d = aC2dNew;
                        }
                        BB.UpdateEdge(anEdge, aC2d, F1, aTolEdge);
                    }

                    if (!BOPTools_AlgoTools2D.HasCurveOnSurface(anEdge, F2))
                    {
                        Curve2D? aC2d = aBC.Curve().SecondCurve2d();
                        if (aC3DETrim is not null)
                        {
                            Curve2D? aC2dNew;

                            if (aC3DE!.IsPeriodic)
                            {
                                BOPTools_AlgoTools2D.AdjustPCurveOnFace(F2, f, l, aC2d!, out aC2dNew, aContext);
                            }
                            else
                            {
                                BOPTools_AlgoTools2D.AdjustPCurveOnFace(F2, aC3DETrim, aC2d!, out aC2dNew, aContext);
                            }
                            aC2d = aC2dNew;
                        }
                        BB.UpdateEdge(anEdge, aC2d, F2, aTolEdge);
                    }

                    OrientSection(anEdge, F1, F2, out O1, out O2);
                    if (Side == Out)
                    {
                        O1 = global::Kerf.Engine.Topology.TopAbs.Reverse(O1);
                        O2 = global::Kerf.Engine.Topology.TopAbs.Reverse(O2);
                    }

                    L1.Append(anEdge.Oriented(O1));
                    L2.Append(anEdge.Oriented(O2));
                }
            }
        }

        double aSameParTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        bool isEl1 = false, isEl2 = false;

        Surface aSurf = Tool.Surface(F1)!;
        if (aSurf.GetType() == typeof(TrimmedSurface))
        {
            aSurf = ((TrimmedSurface)aSurf).BasisSurface();
        }
        if (aSurf.GetType() == typeof(PlaneSurface))
        {
            addPCurve1 = false;
        }
        else if (aSurf is ElementarySurface)
        {
            isEl1 = true;
        }

        aSurf = Tool.Surface(F2)!;
        if (aSurf.GetType() == typeof(TrimmedSurface))
        {
            aSurf = ((TrimmedSurface)aSurf).BasisSurface();
        }
        if (aSurf.GetType() == typeof(PlaneSurface))
        {
            addPCurve2 = false;
        }
        else if (aSurf is ElementarySurface)
        {
            isEl2 = true;
        }

        if (L1.Extent() > 1 && (!isEl1 || !isEl2) && !RefFace1.IsNull)
        {
            // remove excess edges that are out of range
            Exploration.Vertices(RefEdge, out Vertex aV1, out Vertex aV2);
            if (!aV1.IsSame(aV2)) // only if RefEdge is open
            {
                Surface aRefSurf1 = Tool.Surface(RefFace1)!;
                Surface aRefSurf2 = Tool.Surface(RefFace2)!;
                if (aRefSurf1.IsUClosed || aRefSurf1.IsVClosed || aRefSurf2.IsUClosed || aRefSurf2.IsVClosed)
                {
                    Edge MinAngleEdge = new();
                    double MinAngle = global::Kerf.Engine.Foundation.Precision.Infinite();
                    EdgeCurveAdaptor aRefBAcurve = new(RefEdge);
                    Point3D aRefPnt = aRefBAcurve.Value((aRefBAcurve.FirstParameter() + aRefBAcurve.LastParameter()) / 2);

                    NCollection_List<Shape>.Iterator itl = new(L1);
                    for (; itl.More(); itl.Next())
                    {
                        Edge anEdge = ShapeCasts.Edge(itl.Value());

                        EdgeCurveAdaptor aBAcurve = new(anEdge);
                        Point3D aMidPntOnEdge = aBAcurve.Value((aBAcurve.FirstParameter() + aBAcurve.LastParameter()) / 2);
                        Vector3D RefToMid = new(aRefPnt, aMidPntOnEdge);

                        CurvePointExtrema aProjector = new(aRefPnt, aBAcurve);
                        if (aProjector.IsDone)
                        {
                            int imin = 0;
                            double MinSqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
                            for (int ind = 1; ind <= aProjector.ExtCount; ind++)
                            {
                                double aSqDist = aProjector.SquareDistance(ind);
                                if (aSqDist < MinSqDist)
                                {
                                    MinSqDist = aSqDist;
                                    imin = ind;
                                }
                            }
                            if (imin != 0)
                            {
                                Point3D aProjectionOnEdge = aProjector.Point(imin).Value();
                                Vector3D RefToProj = new(aRefPnt, aProjectionOnEdge);
                                double anAngle = RefToProj.Angle(RefToMid);
                                if (anAngle < MinAngle)
                                {
                                    MinAngle = anAngle;
                                    MinAngleEdge = anEdge;
                                }
                            }
                        }
                    }

                    if (!MinAngleEdge.IsNull)
                    {
                        NCollection_List<Shape>.Iterator itlist1 = new(L1);
                        NCollection_List<Shape>.Iterator itlist2 = new(L2);

                        while (itlist1.More())
                        {
                            Shape anEdge = itlist1.Value();
                            if (anEdge.IsSame(MinAngleEdge))
                            {
                                itlist1.Next();
                                itlist2.Next();
                            }
                            else
                            {
                                L1.Remove(itlist1);
                                L2.Remove(itlist2);
                            }
                        }
                    }
                } // if closed
            } // if (!aV1.IsSame(aV2))
        } // if (L1.Extent() > 1 && (!isEl1 || !isEl2) && !theRefFace1.IsNull())

        if (L1.Extent() > 1 && (!isEl1 || !isEl2))
        {
            NCollection_Sequence<Shape> eseq = new();
            NCollection_Sequence<Shape> EdgesForConcat = new();

            if (!TrueEdges.IsEmpty())
            {
                for (i = TrueEdges.Extent(); i >= 1; i--)
                {
                    EdgesForConcat.Append(TrueEdges[i]);
                }
                Edge AssembledEdge = AssembleEdge(pDS, F1, F2, addPCurve1, addPCurve2, EdgesForConcat);
                if (AssembledEdge.IsNull)
                {
                    for (i = TrueEdges.Extent(); i >= 1; i--)
                    {
                        eseq.Append(TrueEdges[i]);
                    }
                }
                else
                {
                    eseq.Append(AssembledEdge);
                }
            }
            else
            {
                NCollection_Sequence<Shape> wseq = new();
                NCollection_Sequence<Shape> edges = new();
                NCollection_List<Shape>.Iterator itl = new(L1);
                for (; itl.More(); itl.Next())
                {
                    edges.Append(itl.Value());
                }
                while (!edges.IsEmpty())
                {
                    Edge anEdge = ShapeCasts.Edge(edges.First());
                    Wire aWire = new(), resWire = new();
                    BB.MakeWire(ref aWire);
                    BB.Add(aWire, anEdge);
                    NCollection_Sequence<int> Candidates = new();
                    for (k = 1; k <= wseq.Length(); k++)
                    {
                        resWire = ShapeCasts.Wire(wseq[k]);
                        if (AreConnex(resWire, aWire))
                        {
                            Candidates.Append(1);
                            break;
                        }
                    }
                    if (Candidates.IsEmpty())
                    {
                        wseq.Append(aWire);
                        edges.Remove(1);
                    }
                    else
                    {
                        for (j = 2; j <= edges.Length(); j++)
                        {
                            anEdge = ShapeCasts.Edge(edges[j]);
                            aWire.Nullify();
                            BB.MakeWire(ref aWire);
                            BB.Add(aWire, anEdge);
                            if (AreConnex(resWire, aWire))
                            {
                                Candidates.Append(j);
                            }
                        }
                        int minind = 1;
                        if (Candidates.Length() > 1)
                        {
                            double MinAngle = RealLast();
                            for (j = 1; j <= Candidates.Length(); j++)
                            {
                                anEdge = ShapeCasts.Edge(edges[Candidates[j]]);
                                double anAngle = AngleWireEdge(resWire, anEdge);
                                if (anAngle < MinAngle)
                                {
                                    MinAngle = anAngle;
                                    minind = j;
                                }
                            }
                        }
                        BB.Add(resWire, ShapeCasts.Edge(edges[Candidates[minind]]));
                        wseq[k] = resWire;
                        edges.Remove(Candidates[minind]);
                    }
                } // end of while (!edges.IsEmpty())

                for (i = 1; i <= wseq.Length(); i++)
                {
                    Wire aWire = ShapeCasts.Wire(wseq[i]);
                    NCollection_Sequence<Shape> aLocalEdgesForConcat = new();
                    if (aWire.Closed())
                    {
                        Vertex StartVertex = new();
                        Edge StartEdge = new();
                        bool StartFound = false;
                        NCollection_List<Shape> Elist = new();

                        Iterator itw = new(aWire);
                        for (; itw.More(); itw.Next())
                        {
                            Edge anEdge = ShapeCasts.Edge(itw.Value());
                            if (StartFound)
                            {
                                Elist.Append(anEdge);
                            }
                            else
                            {
                                Exploration.Vertices(anEdge, out Vertex V1, out Vertex V2);
                                if (!IsAutonomVertex(V1, pDS))
                                {
                                    StartVertex = V2;
                                    StartEdge = anEdge;
                                    StartFound = true;
                                }
                                else if (!IsAutonomVertex(V2, pDS))
                                {
                                    StartVertex = V1;
                                    StartEdge = anEdge;
                                    StartFound = true;
                                }
                                else
                                {
                                    Elist.Append(anEdge);
                                }
                            }
                        } // end of for (; itw.More(); itw.Next())
                        if (!StartFound)
                        {
                            itl.Initialize(Elist);
                            StartEdge = ShapeCasts.Edge(itl.Value());
                            Elist.Remove(itl);
                            Exploration.Vertices(StartEdge, out Vertex V1, out _);
                            StartVertex = V1;
                        }
                        aLocalEdgesForConcat.Append(StartEdge);
                        while (!Elist.IsEmpty())
                        {
                            for (itl.Initialize(Elist); itl.More(); itl.Next())
                            {
                                Edge anEdge = ShapeCasts.Edge(itl.Value());
                                Exploration.Vertices(anEdge, out Vertex V1, out Vertex V2);
                                if (V1.IsSame(StartVertex))
                                {
                                    StartVertex = V2;
                                    aLocalEdgesForConcat.Append(anEdge);
                                    Elist.Remove(itl);
                                    break;
                                }
                                else if (V2.IsSame(StartVertex))
                                {
                                    StartVertex = V1;
                                    aLocalEdgesForConcat.Append(anEdge);
                                    Elist.Remove(itl);
                                    break;
                                }
                            }
                        } // end of while (!Elist.IsEmpty())
                    } // end of if (aWire.Closed())
                    else
                    {
                        WireExplorer Wexp = new(aWire);
                        for (; Wexp.More(); Wexp.Next())
                        {
                            aLocalEdgesForConcat.Append(Wexp.Current());
                        }
                    }

                    Edge AssembledEdge = AssembleEdge(pDS, F1, F2, addPCurve1, addPCurve2, aLocalEdgesForConcat);
                    if (AssembledEdge.IsNull)
                    {
                        for (j = aLocalEdgesForConcat.Length(); j >= 1; j--)
                        {
                            eseq.Append(aLocalEdgesForConcat[j]);
                        }
                    }
                    else
                    {
                        eseq.Append(AssembledEdge);
                    }
                } // for (i = 1; i <= wseq.Length(); i++)
            } // end of else (when TrueEdges is empty)

            if (eseq.Length() < L1.Extent())
            {
                L1.Clear();
                L2.Clear();
                for (i = 1; i <= eseq.Length(); i++)
                {
                    Shape aShape = eseq[i];
                    Edge anEdge = ShapeCasts.Edge(eseq[i]);
                    BRepLib.SameParameter(anEdge, aSameParTol, true);
                    double EdgeTol = Tool.Tolerance(anEdge);
                    // OCCT_DEBUG output left out.
                    if (EdgeTol > 1.0e-2)
                    {
                        continue;
                    }

                    if (EdgeTol >= 1.0e-4)
                    {
                        ReconstructPCurves(anEdge);
                        BRepLib.SameParameter(anEdge, aSameParTol, true);
                        // OCCT_DEBUG output left out.
                    }

                    OrientSection(anEdge, F1, F2, out O1, out O2);
                    if (Side == Out)
                    {
                        O1 = global::Kerf.Engine.Topology.TopAbs.Reverse(O1);
                        O2 = global::Kerf.Engine.Topology.TopAbs.Reverse(O2);
                    }

                    L1.Append(anEdge.Oriented(O1));
                    L2.Append(anEdge.Oriented(O2));
                }
            }
        } // end of if (L1.Extent() > 1)

        else
        {
            NCollection_List<Shape>.Iterator itl = new(L1);
            for (; itl.More(); itl.Next())
            {
                Edge anEdge = ShapeCasts.Edge(itl.Value());
                BRepLib.SameParameter(anEdge, aSameParTol, true);
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Finds whether the edges <paramref name="Edges"/> (of the face <paramref name="F2"/>) lie on the face
    /// <paramref name="F1"/>: <paramref name="LInt1"/> and <paramref name="LInt2"/> receive the edges that do, updated
    /// (tolerance, pcurve on F1). True if all of them do.
    /// </summary>
    public static bool TryProject(in Face F1, in Face F2, NCollection_List<Shape> Edges,
                                  NCollection_List<Shape> LInt1, NCollection_List<Shape> LInt2,
                                  Containment Side, double TolConf)
    {
        // try to find if the edges <Edges> are laying on the face F1.
        LInt1.Clear();
        LInt2.Clear();
        NCollection_List<Shape>.Iterator it = new(Edges);
        bool isOk = true;
        bool Ok = true;
        Surface Bouchon = Tool.Surface(F1)!;
        BrepBuilder B = new();

        for (; it.More(); it.Next())
        {
            Edge CurE = ShapeCasts.Edge(it.Value());
            Curve3D? C = Tool.Curve(CurE, out Placement L, out double f, out double l);
            if (C is null)
            {
                BRepLib.BuildCurve3d(CurE, Tool.Tolerance(CurE));
                C = Tool.Curve(CurE, out L, out f, out l);
                if (C is null) // not 3d curve, can be degenerated, need to skip
                {
                    continue;
                }
            }
            C = new TrimmedCurve3D(C, f, l);
            if (!L.IsIdentity)
            {
                C.Transform(L.Transformation());
            }
            isOk = IsOnSurface(C, Bouchon, TolConf, out double TolReached);

            if (isOk)
            {
                B.UpdateEdge(CurE, TolReached);
                BuildPCurves(CurE, F1);
                OrientSection(CurE, F1, F2, out Orientation O1, out Orientation O2);
                if (Side == Out)
                {
                    O1 = global::Kerf.Engine.Topology.TopAbs.Reverse(O1);
                    O2 = global::Kerf.Engine.Topology.TopAbs.Reverse(O2);
                }
                LInt1.Append(CurE.Oriented(O1));
                LInt2.Append(CurE.Oriented(O2));
            }
            else
            {
                Ok = false;
            }
        }
        return Ok;
    }

    //=================================================================================================

    /// <summary>
    /// The sections of the surfaces of <paramref name="F1"/> and <paramref name="F2"/> (a trimmed plane taken
    /// untrimmed) as new edges: <paramref name="L1"/> receives them oriented on F1, <paramref name="L2"/> on F2.
    /// </summary>
    public static void InterOrExtent(in Face F1, in Face F2, NCollection_List<Shape> L1,
                                     NCollection_List<Shape> L2, Containment Side)
    {
        Curve3D CI;
        L1.Clear();
        L2.Clear();
        Surface S1 = Tool.Surface(F1)!;
        Surface S2 = Tool.Surface(F2)!;

        if (S1.GetType() == typeof(TrimmedSurface))
        {
            TrimmedSurface RTS;
            RTS = (TrimmedSurface)S1;
            if (RTS.BasisSurface().GetType() == typeof(PlaneSurface))
            {
                S1 = RTS.BasisSurface();
            }
        }
        if (S2.GetType() == typeof(TrimmedSurface))
        {
            TrimmedSurface RTS;
            RTS = (TrimmedSurface)S2;
            if (RTS.BasisSurface().GetType() == typeof(PlaneSurface))
            {
                S2 = RTS.BasisSurface();
            }
        }

        GeomInt_IntSS Inter = new(S1, S2, global::Kerf.Engine.Foundation.Precision.Confusion());

        if (Inter.IsDone())
        {
            for (int i = 1; i <= Inter.NbLines(); i++)
            {
                CI = Inter.Line(i);

                if (ToSmall(CI))
                {
                    continue;
                }
                Edge E = new BRepLib_MakeEdge(CI).InternalEdge();
                BuildPCurves(E, F1);
                BuildPCurves(E, F2);
                OrientSection(E, F1, F2, out Orientation O1, out Orientation O2);
                if (Side == Out)
                {
                    O1 = global::Kerf.Engine.Topology.TopAbs.Reverse(O1);
                    O2 = global::Kerf.Engine.Topology.TopAbs.Reverse(O2);
                }
                L1.Append(E.Oriented(O1));
                L2.Append(E.Oriented(O2));
            }
        }
    }

    //=================================================================================================

    private static void ExtentEdge(in Face F, in Face EF, in Edge E, out Edge NE)
    {
        EdgeCurveAdaptor CE = new(E);
        CurveKind Type = CE.GetType();
        Shape aLocalEdge = E.EmptyCopied();
        NE = ShapeCasts.Edge(aLocalEdge);
        //  NE = TopoDS::Edge(E.EmptyCopied());

        if (Type == Line || Type == Circle || Type == Ellipse || Type == Hyperbola
            || Type == Parabola)
        {
            return;
        }
        // Extension en tangence jusqu'au bord de la surface.
        double PMax = 1.0e2;
        Surface S = Tool.Surface(F, out Placement L)!;

        S.Bounds(out double umin, out double umax, out double vmin, out double vmax);
        umin = Max(umin, -PMax);
        vmin = Max(vmin, -PMax);
        umax = Min(umax, PMax);
        vmax = Min(vmax, PMax);

        Curve2D C2d = Tool.CurveOnSurface(E, F, out double f, out double l)!;

        // calcul point cible. ie point d'intersection du prolongement tangent et des bords.
        C2d.D1(CE.FirstParameter(), out Point2D P, out Vector2D Tang);
        double tx, ty, tmin;
        tx = ty = global::Kerf.Engine.Foundation.Precision.Infinite();
        if (Math.Abs(Tang.X()) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            tx = Min(Math.Abs((umax - P.X()) / Tang.X()), Math.Abs((umin - P.X()) / Tang.X()));
        }
        if (Math.Abs(Tang.Y()) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            ty = Min(Math.Abs((vmax - P.Y()) / Tang.Y()), Math.Abs((vmin - P.Y()) / Tang.Y()));
        }
        tmin = Min(tx, ty);
        Tang = tmin * Tang;
        Point2D PF2d = new(P.X() - Tang.X(), P.Y() - Tang.Y());

        C2d.D1(CE.LastParameter(), out P, out Tang);
        tx = ty = global::Kerf.Engine.Foundation.Precision.Infinite();
        if (Math.Abs(Tang.X()) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            tx = Min(Math.Abs((umax - P.X()) / Tang.X()), Math.Abs((umin - P.X()) / Tang.X()));
        }
        if (Math.Abs(Tang.Y()) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            ty = Min(Math.Abs((vmax - P.Y()) / Tang.Y()), Math.Abs((vmin - P.Y()) / Tang.Y()));
        }
        tmin = Min(tx, ty);
        Tang = tmin * Tang;
        Point2D PL2d = new(P.X() + Tang.X(), P.Y() + Tang.Y());

        Curve3D CC = GeomAPI.To3d(C2d, new Plane3D(new CoordinateSystem(Constants.XOY())));
        Point3D PF = new(PF2d.X(), PF2d.Y(), 0.0);
        Point3D PL = new(PL2d.X(), PL2d.Y(), 0.0);

        BoundedCurve3D? ExtC = CC as BoundedCurve3D;
        if (ExtC is null)
        {
            return;
        }

        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref ExtC, PF, 1, false);
        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref ExtC, PL, 1, true);

        Curve2D CNE2d = GeomAPI.To2d(ExtC, new Plane3D(new CoordinateSystem(Constants.XOY())))!;

        // Construction de la nouvelle arrete;
        BrepBuilder B = new();
        B.MakeEdge(ref NE);
        //  B.UpdateEdge (NE,CNE2d,F,BRep_Tool::Tolerance(E));
        B.UpdateEdge(NE, CNE2d, EF, Tool.Tolerance(E));
        B.Range(NE, CNE2d.FirstParameter(), CNE2d.LastParameter());
        NE.Orientation(E.Orientation());
    }

    //=================================================================================================

    private static bool ProjectVertexOnEdge(in Vertex V, in Edge E, double TolConf)
    {
        BrepBuilder B = new();
        double f, l;
        double U = 0.0;
        bool found = false;

        Point3D P = Tool.Pnt(V);
        EdgeCurveAdaptor C = new(E);
        f = C.FirstParameter();
        l = C.LastParameter();

        if (V.Orientation() == Forward)
        {
            if (Math.Abs(f) < global::Kerf.Engine.Foundation.Precision.Infinite())
            {
                Point3D PF = C.Value(f);
                if (PF.IsEqual(P, TolConf))
                {
                    U = f;
                    found = true;
                }
            }
        }
        if (V.Orientation() == Reversed)
        {
            if (!found && Math.Abs(l) < global::Kerf.Engine.Foundation.Precision.Infinite())
            {
                Point3D PL = C.Value(l);
                if (PL.IsEqual(P, TolConf))
                {
                    U = l;
                    found = true;
                }
            }
        }
        if (!found)
        {
            CurvePointExtrema Proj = new(P, C);
            if (Proj.IsDone && Proj.ExtCount > 0)
            {
                double Dist2, Dist2Min = Proj.SquareDistance(1);
                U = Proj.Point(1).Parameter();
                for (int i = 2; i <= Proj.ExtCount; i++)
                {
                    Dist2 = Proj.SquareDistance(i);
                    if (Dist2 < Dist2Min)
                    {
                        Dist2Min = Dist2;
                        U = Proj.Point(i).Parameter();
                    }
                }
                found = true;
            }
        }

        // OCCT_DEBUG (AffichExtent) checks left out.
        if (found)
        {
            Shape aLocalShape = E.Oriented(Forward);
            Edge EE = ShapeCasts.Edge(aLocalShape);
            aLocalShape = V.Oriented(Internal);
            //    TopoDS_Edge EE = TopoDS::Edge(E.Oriented(TopAbs_FORWARD));
            B.UpdateVertex(ShapeCasts.Vertex(aLocalShape), U, EE, Tool.Tolerance(E));
        }
        return found;
    }

    //=================================================================================================

    /// <summary>
    /// The intersection vertices of the edges <paramref name="E1"/> and <paramref name="E2"/> on the face
    /// <paramref name="F"/> (new vertices with parameters on both edges), which <paramref name="LV"/> receives: only
    /// the nearest to E1's start and end where there are several.
    /// </summary>
    public static void Inter2d(in Face F, in Edge E1, in Edge E2, NCollection_List<Shape> LV,
                               double Tol)
    {
        double TolConf = Tol;
        BrepBuilder B = new();
        double[] fl1 = new double[2], fl2 = new double[2];
        LV.Clear();

        // Si l edge a ete etendu les pcurves ne sont pas forcement
        // a jour.
        BuildPCurves(E1, F);
        BuildPCurves(E2, F);

        // Construction des curves 3d si elles n existent pas
        // utile pour coder correctement les parametres des vertex
        // d intersection sur les edges.
        // TopLoc_Location L;
        // double   f,l;
        // occ::handle<Geom_Curve> C3d1 = BRep_Tool::Curve(E1,L,f,l);
        // if (C3d1.IsNull()) {
        //  BRepLib::BuildCurve3d(E1,BRep_Tool::Tolerance(E1));
        //}
        // occ::handle<Geom_Curve> C3d2 = BRep_Tool::Curve(E2,L,f,l);
        // if (C3d2.IsNull()) {
        //  BRepLib::BuildCurve3d(E2,BRep_Tool::Tolerance(E2));
        //}

        int NbPC1 = 1, NbPC2 = 1;
        if (Tool.IsClosed(E1, F))
        {
            NbPC1++;
        }
        if (Tool.IsClosed(E2, F))
        {
            NbPC2++;
        }

        Surface S = Tool.Surface(F)!;
        Curve2D? C1, C2;
        bool YaSol = false;
        int itry = 0;

        while (!YaSol && itry < 2)
        {
            for (int i = 1; i <= NbPC1; i++)
            {
                Shape aLocalEdgeReversedE1 = E1.Reversed();
                if (i == 1)
                {
                    C1 = Tool.CurveOnSurface(E1, F, out fl1[0], out fl1[1]);
                }
                else
                {
                    C1 = Tool.CurveOnSurface(ShapeCasts.Edge(aLocalEdgeReversedE1), F, out fl1[0], out fl1[1]);
                }
                for (int j = 1; j <= NbPC2; j++)
                {
                    Shape aLocalEdge = E2.Reversed();
                    if (j == 1)
                    {
                        C2 = Tool.CurveOnSurface(E2, F, out fl2[0], out fl2[1]);
                    }
                    else
                    {
                        C2 = Tool.CurveOnSurface(ShapeCasts.Edge(aLocalEdge), F, out fl2[0], out fl2[1]);
                    }
                    // OCCT_DEBUG: the check for missing pcurves (which returns) is left out.
                    double U1 = 0.0, U2 = 0.0;
                    Point2D P2d = new();
                    bool aCurrentFind = false;
                    if (itry == 1)
                    {
                        fl1[0] = C1!.FirstParameter();
                        fl1[1] = C1.LastParameter();
                        fl2[0] = C2!.FirstParameter();
                        fl2[1] = C2.LastParameter();
                    }
                    CurveAdaptor2D AC1 = new(C1!, fl1[0], fl1[1]);
                    CurveAdaptor2D AC2 = new(C2!, fl2[0], fl2[1]);

                    if (itry == 0)
                    {
                        Point2D[] P1 = new Point2D[2], P2 = new Point2D[2];
                        P1[0] = C1!.Value(fl1[0]);
                        P1[1] = C1.Value(fl1[1]);
                        P2[0] = C2!.Value(fl2[0]);
                        P2[1] = C2.Value(fl2[1]);

                        int i1;
                        for (i1 = 0; i1 < 2; i1++)
                        {
                            for (int i2 = 0; i2 < 2; i2++)
                            {
                                if (Math.Abs(fl1[i1]) < global::Kerf.Engine.Foundation.Precision.Infinite() && Math.Abs(fl2[i2]) < global::Kerf.Engine.Foundation.Precision.Infinite())
                                {
                                    if (P1[i1].IsEqual(P2[i2], TolConf))
                                    {
                                        YaSol = true;
                                        aCurrentFind = true;
                                        U1 = fl1[i1];
                                        U2 = fl2[i2];
                                        P2d = C1.Value(U1);
                                    }
                                }
                            }
                        }
                        if (!YaSol)
                        {
                            for (i1 = 0; i1 < 2; i1++)
                            {
                                ExtPC2d extr = new(P1[i1], AC2);
                                if (extr.IsDone && extr.ExtCount > 0)
                                {
                                    double Dist2, Dist2Min = extr.SquareDistance(1);
                                    int IndexMin = 1;
                                    for (int ind = 2; ind <= extr.ExtCount; ind++)
                                    {
                                        Dist2 = extr.SquareDistance(ind);
                                        if (Dist2 < Dist2Min)
                                        {
                                            Dist2Min = Dist2;
                                            IndexMin = ind;
                                        }
                                    }
                                    if (Dist2Min <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                                    {
                                        YaSol = true;
                                        aCurrentFind = true;
                                        P2d = P1[i1];
                                        U1 = fl1[i1];
                                        U2 = (extr.Point(IndexMin)).Parameter();
                                        break;
                                    }
                                }
                            }
                        }
                        if (!YaSol)
                        {
                            for (int i2 = 0; i2 < 2; i2++)
                            {
                                ExtPC2d extr = new(P2[i2], AC1);
                                if (extr.IsDone && extr.ExtCount > 0)
                                {
                                    double Dist2, Dist2Min = extr.SquareDistance(1);
                                    int IndexMin = 1;
                                    for (int ind = 2; ind <= extr.ExtCount; ind++)
                                    {
                                        Dist2 = extr.SquareDistance(ind);
                                        if (Dist2 < Dist2Min)
                                        {
                                            Dist2Min = Dist2;
                                            IndexMin = ind;
                                        }
                                    }
                                    if (Dist2Min <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                                    {
                                        YaSol = true;
                                        aCurrentFind = true;
                                        P2d = P2[i2];
                                        U2 = fl2[i2];
                                        U1 = (extr.Point(IndexMin)).Parameter();
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    if (!YaSol)
                    {
                        Geom2dInt_GInter Inter = new(AC1, AC2, TolConf, TolConf);

                        if (!Inter.IsEmpty() && Inter.NbPoints() > 0)
                        {
                            YaSol = true;
                            aCurrentFind = true;
                            U1 = Inter.Point(1).ParamOnFirst();
                            U2 = Inter.Point(1).ParamOnSecond();
                            P2d = Inter.Point(1).Value();
                        }
                        else if (!Inter.IsEmpty() && Inter.NbSegments() > 0)
                        {
                            YaSol = true;
                            aCurrentFind = true;
                            IntRes2d_IntersectionSegment Seg = Inter.Segment(1);
                            IntRes2d_IntersectionPoint IntP1 = Seg.FirstPoint();
                            IntRes2d_IntersectionPoint IntP2 = Seg.LastPoint();
                            double U1on1 = IntP1.ParamOnFirst();
                            double U1on2 = IntP2.ParamOnFirst();
                            double U2on1 = IntP1.ParamOnSecond();
                            double U2on2 = IntP2.ParamOnSecond();
                            // OCCT_DEBUG output left out.
                            U1 = (U1on1 + U1on2) / 2.0;
                            U2 = (U2on1 + U2on2) / 2.0;
                            Point2D P2d1 = C1!.Value(U1);
                            Point2D P2d2 = C2!.Value(U2);
                            P2d.SetX((P2d1.X() + P2d2.X()) / 2.0);
                            P2d.SetY((P2d1.Y() + P2d2.Y()) / 2.0);
                        }
                    }
                    if (aCurrentFind)
                    {
                        Point3D P = S.Value(P2d.X(), P2d.Y());
                        Vertex V = new BRepLib_MakeVertex(P).InternalVertex();
                        V.Orientation(Internal);
                        Shape aLocalEdgeOrientedE1 = E1.Oriented(Forward);
                        B.UpdateVertex(V, U1, ShapeCasts.Edge(aLocalEdgeOrientedE1), TolConf);
                        aLocalEdgeOrientedE1 = E2.Oriented(Forward);
                        B.UpdateVertex(V, U2, ShapeCasts.Edge(aLocalEdgeOrientedE1), TolConf);
                        //	  B.UpdateVertex(V,U1,TopoDS::Edge(E1.Oriented(TopAbs_FORWARD)),TolConf);
                        //	  B.UpdateVertex(V,U2,TopoDS::Edge(E2.Oriented(TopAbs_FORWARD)),TolConf);
                        LV.Append(V);
                    }
                }
            }
            itry++;
        }

        if (LV.Extent() > 1)
        {
            //------------------------------------------------
            // garde seulement les vertex les plus proches du
            // debut et de la fin.
            //------------------------------------------------
            NCollection_List<Shape>.Iterator it = new(LV);
            Vertex VF = new(), VL = new();
            double UMin = global::Kerf.Engine.Foundation.Precision.Infinite();
            double UMax = -global::Kerf.Engine.Foundation.Precision.Infinite();
            double U;

            for (; it.More(); it.Next())
            {
                Vertex CV = ShapeCasts.Vertex(it.Value());
                Shape aLocalEdge = E1.Oriented(Forward);
                U = Tool.Parameter(CV, ShapeCasts.Edge(aLocalEdge));
                //      U = BRep_Tool::Parameter(CV,TopoDS::Edge(E1.Oriented(TopAbs_FORWARD)));
                if (U < UMin)
                {
                    VF = CV;
                    UMin = U;
                }
                if (U > UMax)
                {
                    VL = CV;
                    UMax = U;
                }
            }
            LV.Clear();
            LV.Append(VF);
            LV.Append(VL);
        }

        // OCCT_DEBUG output left out.
    }

    //=================================================================================================

    private static void SelectEdge(in Face F, in Face EF, in Edge E,
                                   NCollection_List<Shape> LInt)
    {
        //------------------------------------------------------------
        // detrompeur sur les intersections sur les faces periodiques
        //------------------------------------------------------------
        NCollection_List<Shape>.Iterator it = new(LInt);
        double dU = 1.0e100;
        Edge GE = new();

        double tmp;
        Tool.Range(E, out double Fst, out double Lst);
        EdgeCurveAdaptor Ad1 = new(E);

        Point3D PFirst = Ad1.Value(Fst);
        Point3D PLast = Ad1.Value(Lst);

        //----------------------------------------------------------------------
        // Selection de l edge qui couvre le plus le domaine de l edge initiale.
        //----------------------------------------------------------------------
        for (; it.More(); it.Next())
        {
            Edge EI = ShapeCasts.Edge(it.Value());

            Tool.Range(EI, out Fst, out Lst);
            EdgeCurveAdaptor Ad2 = new(EI);
            Point3D P1 = Ad2.Value(Fst);
            Point3D P2 = Ad2.Value(Lst);

            tmp = P1.Distance(PFirst) + P2.Distance(PLast);
            if (tmp <= dU)
            {
                dU = tmp;
                GE = EI;
            }
        }
        LInt.Clear();
        LInt.Append(GE);
    }

    //=================================================================================================

    private static void MakeFace(Surface S, double Um, double UM, double Vm, double VM, bool uclosed, bool vclosed,
                                 bool isVminDegen, bool isVmaxDegen, ref Face F)
    {
        double UMin = Um;
        double UMax = UM;
        double VMin = Vm;
        double VMax = VM;

        // compute infinite flags
        bool umininf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(UMin);
        bool umaxinf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(UMax);
        bool vmininf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMin);
        bool vmaxinf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMax);

        // degenerated flags (for cones)
        bool vmindegen = isVminDegen, vmaxdegen = isVmaxDegen;
        Surface theSurf = S;
        if (S.GetType() == typeof(TrimmedSurface))
        {
            theSurf = ((TrimmedSurface)S).BasisSurface();
        }
        if (theSurf.GetType() == typeof(ConicalSurface))
        {
            ConicalSurface ConicalS = (ConicalSurface)theSurf;
            Cone theCone = ConicalS.Cone();
            Point3D theApex = theCone.Apex();
            global::Kerf.Engine.Geometry.ElSLib.Parameters(theCone, theApex, out double Uapex, out double Vapex);
            if (Math.Abs(VMin - Vapex) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                vmindegen = true;
            }
            if (Math.Abs(VMax - Vapex) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                vmaxdegen = true;
            }
        }

        // compute vertices
        BrepBuilder B = new();
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        Vertex V00 = new(), V10 = new(), V11 = new(), V01 = new();

        if (!umininf)
        {
            if (!vmininf)
            {
                B.MakeVertex(ref V00, S.Value(UMin, VMin), tol);
            }
            if (!vmaxinf)
            {
                B.MakeVertex(ref V01, S.Value(UMin, VMax), tol);
            }
        }
        if (!umaxinf)
        {
            if (!vmininf)
            {
                B.MakeVertex(ref V10, S.Value(UMax, VMin), tol);
            }
            if (!vmaxinf)
            {
                B.MakeVertex(ref V11, S.Value(UMax, VMax), tol);
            }
        }

        if (uclosed)
        {
            V10 = V00;
            V11 = V01;
        }

        if (vclosed)
        {
            V01 = V00;
            V11 = V10;
        }

        if (vmindegen)
        {
            V10 = V00;
        }
        if (vmaxdegen)
        {
            V11 = V01;
        }

        // make the lines
        LineCurve2D? Lumin = null, Lumax = null, Lvmin = null, Lvmax = null;
        if (!umininf)
        {
            Lumin = new LineCurve2D(new Point2D(UMin, 0), new Direction2D(Direction2D.D.Y));
        }
        if (!umaxinf)
        {
            Lumax = new LineCurve2D(new Point2D(UMax, 0), new Direction2D(Direction2D.D.Y));
        }
        if (!vmininf)
        {
            Lvmin = new LineCurve2D(new Point2D(0, VMin), new Direction2D(Direction2D.D.X));
        }
        if (!vmaxinf)
        {
            Lvmax = new LineCurve2D(new Point2D(0, VMax), new Direction2D(Direction2D.D.X));
        }

        Curve3D? Cumin = null, Cumax = null, Cvmin = null, Cvmax = null;
        double TolApex = 1.0e-5;
        // bool hasiso = ! S->IsKind(STANDARD_TYPE(Geom_OffsetSurface));
        bool hasiso = S is ElementarySurface;
        if (hasiso)
        {
            if (!umininf)
            {
                Cumin = S.UIso(UMin);
            }
            if (!umaxinf)
            {
                Cumax = S.UIso(UMax);
            }
            if (!vmininf)
            {
                Cvmin = S.VIso(VMin);
                if (Gabarit(Cvmin) <= TolApex)
                {
                    vmindegen = true;
                }
            }
            if (!vmaxinf)
            {
                Cvmax = S.VIso(VMax);
                if (Gabarit(Cvmax) <= TolApex)
                {
                    vmaxdegen = true;
                }
            }
        }

        // make the face
        B.MakeFace(ref F, S, tol);

        // make the edges
        Edge eumin = new(), eumax = new(), evmin = new(), evmax = new();

        if (!umininf)
        {
            if (hasiso)
            {
                B.MakeEdge(ref eumin, Cumin, tol);
            }
            else
            {
                B.MakeEdge(ref eumin);
            }
            if (uclosed)
            {
                B.UpdateEdge(eumin, Lumax, Lumin, F, tol);
            }
            else
            {
                B.UpdateEdge(eumin, Lumin, F, tol);
            }
            if (!vmininf)
            {
                V00.Orientation(Forward);
                B.Add(eumin, V00);
            }
            if (!vmaxinf)
            {
                V01.Orientation(Reversed);
                B.Add(eumin, V01);
            }
            B.Range(eumin, VMin, VMax);
        }

        if (!umaxinf)
        {
            if (uclosed)
            {
                eumax = eumin;
            }
            else
            {
                if (hasiso)
                {
                    B.MakeEdge(ref eumax, Cumax, tol);
                }
                else
                {
                    B.MakeEdge(ref eumax);
                }
                B.UpdateEdge(eumax, Lumax, F, tol);
                if (!vmininf)
                {
                    V10.Orientation(Forward);
                    B.Add(eumax, V10);
                }
                if (!vmaxinf)
                {
                    V11.Orientation(Reversed);
                    B.Add(eumax, V11);
                }
                B.Range(eumax, VMin, VMax);
            }
        }

        if (!vmininf)
        {
            if (hasiso && !vmindegen)
            {
                B.MakeEdge(ref evmin, Cvmin, tol);
            }
            else
            {
                B.MakeEdge(ref evmin);
            }
            if (vclosed)
            {
                B.UpdateEdge(evmin, Lvmin, Lvmax, F, tol);
            }
            else
            {
                B.UpdateEdge(evmin, Lvmin, F, tol);
            }
            if (!umininf)
            {
                V00.Orientation(Forward);
                B.Add(evmin, V00);
            }
            if (!umaxinf)
            {
                V10.Orientation(Reversed);
                B.Add(evmin, V10);
            }
            B.Range(evmin, UMin, UMax);
            if (vmindegen)
            {
                B.Degenerated(evmin, true);
            }
        }

        if (!vmaxinf)
        {
            if (vclosed)
            {
                evmax = evmin;
            }
            else
            {
                if (hasiso && !vmaxdegen)
                {
                    B.MakeEdge(ref evmax, Cvmax, tol);
                }
                else
                {
                    B.MakeEdge(ref evmax);
                }
                B.UpdateEdge(evmax, Lvmax, F, tol);
                if (!umininf)
                {
                    V01.Orientation(Forward);
                    B.Add(evmax, V01);
                }
                if (!umaxinf)
                {
                    V11.Orientation(Reversed);
                    B.Add(evmax, V11);
                }
                B.Range(evmax, UMin, UMax);
                if (vmaxdegen)
                {
                    B.Degenerated(evmax, true);
                }
            }
        }

        // make the wires and add them to the face
        eumin.Orientation(Reversed);
        evmax.Orientation(Reversed);

        Wire W = new();

        if (!umininf && !umaxinf && vmininf && vmaxinf)
        {
            // two wires in u
            B.MakeWire(ref W);
            B.Add(W, eumin);
            B.Add(F, W);
            B.MakeWire(ref W);
            B.Add(W, eumax);
            B.Add(F, W);
            F.Closed(uclosed);
        }

        else if (umininf && umaxinf && !vmininf && !vmaxinf)
        {
            // two wires in v
            B.MakeWire(ref W);
            B.Add(W, evmin);
            B.Add(F, W);
            B.MakeWire(ref W);
            B.Add(W, evmax);
            B.Add(F, W);
            F.Closed(vclosed);
        }

        else if (!umininf || !umaxinf || !vmininf || !vmaxinf)
        {
            // one wire
            B.MakeWire(ref W);
            if (!umininf)
            {
                B.Add(W, eumin);
            }
            if (!vmininf)
            {
                B.Add(W, evmin);
            }
            if (!umaxinf)
            {
                B.Add(W, eumax);
            }
            if (!vmaxinf)
            {
                B.Add(W, evmax);
            }
            B.Add(F, W);
            W.Closed(!umininf && !umaxinf && !vmininf && !vmaxinf);
            F.Closed(uclosed && vclosed);
        }
    }

    //=================================================================================================

    private static bool EnlargeGeometry(ref Surface S, ref double U1, ref double U2, ref double V1, ref double V2,
                                        ref bool IsV1degen, ref bool IsV2degen, double uf1, double uf2, double vf1,
                                        double vf2, double coeff, bool theGlobalEnlargeU, bool theGlobalEnlargeVfirst,
                                        bool theGlobalEnlargeVlast, double theLenBeforeUfirst, double theLenAfterUlast,
                                        double theLenBeforeVfirst, double theLenAfterVlast)
    {
        const double TolApex = 1.0e-5;

        bool SurfaceChange = false;
        if (S.GetType() == typeof(TrimmedSurface))
        {
            Surface BS = ((TrimmedSurface)S).BasisSurface();
            EnlargeGeometry(ref BS,
                            ref U1,
                            ref U2,
                            ref V1,
                            ref V2,
                            ref IsV1degen,
                            ref IsV2degen,
                            uf1,
                            uf2,
                            vf1,
                            vf2,
                            coeff,
                            theGlobalEnlargeU,
                            theGlobalEnlargeVfirst,
                            theGlobalEnlargeVlast,
                            theLenBeforeUfirst,
                            theLenAfterUlast,
                            theLenBeforeVfirst,
                            theLenAfterVlast);
            if (!theGlobalEnlargeVfirst)
            {
                V1 = vf1;
            }
            if (!theGlobalEnlargeVlast)
            {
                V2 = vf2;
            }
            if (!theGlobalEnlargeVfirst || !theGlobalEnlargeVlast)
            {
                // Handle(Geom_RectangularTrimmedSurface)::DownCast (S)->SetTrim( U1, U2, V1, V2 );
                S = new TrimmedSurface(BS, U1, U2, V1, V2);
            }
            else
            {
                S = BS;
            }
            SurfaceChange = true;
        }
        else if (S.GetType() == typeof(global::Kerf.Geometry.OffsetSurface))
        {
            Surface Surf = ((global::Kerf.Geometry.OffsetSurface)S).BasisSurface();
            SurfaceChange = EnlargeGeometry(ref Surf,
                                            ref U1,
                                            ref U2,
                                            ref V1,
                                            ref V2,
                                            ref IsV1degen,
                                            ref IsV2degen,
                                            uf1,
                                            uf2,
                                            vf1,
                                            vf2,
                                            coeff,
                                            theGlobalEnlargeU,
                                            theGlobalEnlargeVfirst,
                                            theGlobalEnlargeVlast,
                                            theLenBeforeUfirst,
                                            theLenAfterUlast,
                                            theLenBeforeVfirst,
                                            theLenAfterVlast);
            // OCCT.NET: OCCT sets the basis surface of S in place, and S is the face's own (frozen) surface: the port
            // sets it on a copy, which S becomes (the face keeps its surface). Where the basis surface is unchanged,
            // OCCT's call changes nothing, and the port keeps S.
            if (!ReferenceEquals(Surf, ((global::Kerf.Geometry.OffsetSurface)S).BasisSurface()))
            {
                S = ((global::Kerf.Geometry.OffsetSurface)S).WithBasisSurface(Surf);
            }
        }
        else if (S.GetType() == typeof(ExtrusionSurface) || S.GetType() == typeof(RevolutionSurface))
        {
            double du_first = 0.0, du_last = 0.0, dv_first = 0.0, dv_last = 0.0;
            Curve3D uiso, viso, uiso1, uiso2, viso1, viso2;
            bool enlargeU = theGlobalEnlargeU, enlargeV = true;
            bool enlargeUfirst = enlargeU, enlargeUlast = enlargeU;
            bool enlargeVfirst = theGlobalEnlargeVfirst, enlargeVlast = theGlobalEnlargeVlast;
            S.Bounds(out double u1, out double u2, out double v1, out double v2);
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u2))
            {
                du_first = du_last = uf2 - uf1;
                u1 = uf1 - du_first;
                u2 = uf2 + du_last;
                enlargeU = false;
            }
            else if (S.IsUClosed)
            {
                enlargeU = false;
            }
            else
            {
                viso = S.VIso(vf1);
                CurveAdaptor gac = new(viso);
                double du_default = AbscissaPoint.Length(gac) * coeff;
                du_first = (theLenBeforeUfirst == -1) ? du_default : theLenBeforeUfirst;
                du_last = (theLenAfterUlast == -1) ? du_default : theLenAfterUlast;
                uiso1 = S.UIso(uf1);
                uiso2 = S.UIso(uf2);
                if (Gabarit(uiso1) <= TolApex)
                {
                    enlargeUfirst = false;
                }
                if (Gabarit(uiso2) <= TolApex)
                {
                    enlargeUlast = false;
                }
            }
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(v1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(v2))
            {
                dv_first = dv_last = vf2 - vf1;
                v1 = vf1 - dv_first;
                v2 = vf2 + dv_last;
                enlargeV = false;
            }
            else if (S.IsVClosed)
            {
                enlargeV = false;
            }
            else
            {
                uiso = S.UIso(uf1);
                CurveAdaptor gac = new(uiso);
                double dv_default = AbscissaPoint.Length(gac) * coeff;
                dv_first = (theLenBeforeVfirst == -1) ? dv_default : theLenBeforeVfirst;
                dv_last = (theLenAfterVlast == -1) ? dv_default : theLenAfterVlast;
                viso1 = S.VIso(vf1);
                viso2 = S.VIso(vf2);
                if (Gabarit(viso1) <= TolApex)
                {
                    enlargeVfirst = false;
                    IsV1degen = true;
                }
                if (Gabarit(viso2) <= TolApex)
                {
                    enlargeVlast = false;
                    IsV2degen = true;
                }
            }
            BoundedSurface aSurf = new TrimmedSurface(S, u1, u2, v1, v2);
            if (enlargeU)
            {
                if (enlargeUfirst && du_first != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSurf, du_first, 1, true, false);
                }
                if (enlargeUlast && du_last != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSurf, du_last, 1, true, true);
                }
            }
            if (enlargeV)
            {
                if (enlargeVfirst && dv_first != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSurf, dv_first, 1, false, false);
                }
                if (enlargeVlast && dv_last != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSurf, dv_last, 1, false, true);
                }
            }
            S = aSurf;
            S.Bounds(out U1, out U2, out V1, out V2);
            SurfaceChange = true;
        }
        else if (S.GetType() == typeof(global::Kerf.Geometry.BezierSurface) || S.GetType() == typeof(global::Kerf.Geometry.BSplineSurface))
        {
            bool enlargeU = theGlobalEnlargeU, enlargeV = true;
            bool enlargeUfirst = enlargeU, enlargeUlast = enlargeU;
            bool enlargeVfirst = theGlobalEnlargeVfirst, enlargeVlast = theGlobalEnlargeVlast;
            if (S.IsUClosed)
            {
                enlargeU = false;
            }
            if (S.IsVClosed)
            {
                enlargeV = false;
            }

            double duf = uf2 - uf1, dvf = vf2 - vf1;
            S.Bounds(out double u1, out double u2, out double v1, out double v2);

            double du_first = 0.0, du_last = 0.0, dv_first = 0.0, dv_last = 0.0;
            Curve3D uiso1, uiso2, viso1, viso2;
            double gabarit_uiso1, gabarit_uiso2, gabarit_viso1, gabarit_viso2;

            uiso1 = S.UIso(u1);
            uiso2 = S.UIso(u2);
            viso1 = S.VIso(v1);
            viso2 = S.VIso(v2);
            gabarit_uiso1 = Gabarit(uiso1);
            gabarit_uiso2 = Gabarit(uiso2);
            gabarit_viso1 = Gabarit(viso1);
            gabarit_viso2 = Gabarit(viso2);
            if (gabarit_viso1 <= TolApex || gabarit_viso2 <= TolApex)
            {
                enlargeU = false;
            }
            if (gabarit_uiso1 <= TolApex || gabarit_uiso2 <= TolApex)
            {
                enlargeV = false;
            }

            CurveAdaptor gac = new();
            if (enlargeU)
            {
                gac.Load(viso1);
                double du_default = AbscissaPoint.Length(gac) * coeff;
                du_first = (theLenBeforeUfirst == -1) ? du_default : theLenBeforeUfirst;
                du_last = (theLenAfterUlast == -1) ? du_default : theLenAfterUlast;
                if (gabarit_uiso1 <= TolApex)
                {
                    enlargeUfirst = false;
                }
                if (gabarit_uiso2 <= TolApex)
                {
                    enlargeUlast = false;
                }
            }
            if (enlargeV)
            {
                gac.Load(uiso1);
                double dv_default = AbscissaPoint.Length(gac) * coeff;
                dv_first = (theLenBeforeVfirst == -1) ? dv_default : theLenBeforeVfirst;
                dv_last = (theLenAfterVlast == -1) ? dv_default : theLenAfterVlast;
                if (gabarit_viso1 <= TolApex)
                {
                    enlargeVfirst = false;
                    IsV1degen = true;
                }
                if (gabarit_viso2 <= TolApex)
                {
                    enlargeVlast = false;
                    IsV2degen = true;
                }
            }

            // OCCT.NET: OCCT's ExtendSurfByLength changes a B-spline surface (here the face's own) in place; the
            // port's changes a copy (decision 245), so the face keeps its surface.
            BoundedSurface aSurf = (BoundedSurface)S;
            if (enlargeU)
            {
                if (enlargeUfirst && uf1 - u1 < duf && du_first != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSurf, du_first, 1, true, false);
                }
                if (enlargeUlast && u2 - uf2 < duf && du_last != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSurf, du_last, 1, true, true);
                }
            }
            if (enlargeV)
            {
                if (enlargeVfirst && vf1 - v1 < dvf && dv_first != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSurf, dv_first, 1, false, false);
                }
                if (enlargeVlast && v2 - vf2 < dvf && dv_last != 0.0)
                {
                    global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSurf, dv_last, 1, false, true);
                }
            }
            S = aSurf;

            S.Bounds(out U1, out U2, out V1, out V2);
            SurfaceChange = true;
        }
        else
        {
            S.Bounds(out double UU1, out double UU2, out double VV1, out double VV2);
            // Pas d extension au dela des bornes de la surface.
            U1 = Max(UU1, U1);
            V1 = Max(VV1, V1);
            U2 = Min(UU2, U2);
            V2 = Min(VV2, V2);
        }
        return SurfaceChange;
    }

    //=======================================================================
    // function : UpDatePCurve
    // purpose  :  Mise a jour des pcurves de F sur la surface de de BF.
    //            F and BF has to be FORWARD,
    //=======================================================================

    private static void UpdatePCurves(in Face F, in Face BF)
    {
        int i;
        BrepBuilder B = new();
        NCollection_IndexedMap<Shape> Emap = new(ShapeHasher.Instance);
        Curve2D? NullPCurve = null;

        Exploration.MapShapes(F, ShapeType.Edge, Emap);

        for (i = 1; i <= Emap.Extent(); i++)
        {
            Edge CE = ShapeCasts.Edge(Emap[i]);
            CE.Orientation(Forward);
            Curve2D? C2 = Tool.CurveOnSurface(CE, F, out double f, out double l);
            if (C2 is not null)
            {
                if (Tool.IsClosed(CE, F))
                {
                    CE.Reverse();
                    Curve2D? C2R = Tool.CurveOnSurface(CE, F, out f, out l);
                    B.UpdateEdge(CE, NullPCurve, NullPCurve, F, Tool.Tolerance(CE));
                    B.UpdateEdge(CE, C2, C2R, BF, Tool.Tolerance(CE));
                }
                else
                {
                    B.UpdateEdge(CE, NullPCurve, F, Tool.Tolerance(CE));
                    B.UpdateEdge(CE, C2, BF, Tool.Tolerance(CE));
                }

                B.Range(CE, f, l);
            }
        }
    }

    //=================================================================================================

    private static void CompactUVBounds(in Face F, out double UMin, out double UMax, out double VMin,
                                        out double VMax)
    {
        // Calcul serre pour que les bornes ne couvrent pas plus d une periode
        double N = 33;
        Box2d B = new();

        Explorer exp = new();
        for (exp.Init(F, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge E = ShapeCasts.Edge(exp.Current());
            EdgeCurve2DAdaptor C = new(E, F);
            Tool.Range(E, out double U1, out double U2);
            Point2D P;
            double U = U1;
            double DU = (U2 - U1) / (N - 1);
            for (int j = 1; j < N; j++)
            {
                C.D0(U, out P);
                U += DU;
                B.Add(P);
            }
            C.D0(U2, out P);
            B.Add(P);
        }

        if (!B.IsVoid)
        {
            B.Get(out UMin, out VMin, out UMax, out VMax);
        }
        else
        {
            Tool.Surface(F)!.Bounds(out UMin, out UMax, out VMin, out VMax);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Whether the surface of <paramref name="F"/> may be enlarged in U and at either end in V: not across the
    /// tangent (or degenerated) iso-line edges that bound the face.
    /// </summary>
    public static void CheckBounds(in Face F, BRepOffset_Analyse Analyse, out bool enlargeU,
                                   out bool enlargeVfirst, out bool enlargeVlast)
    {
        enlargeU = true;
        enlargeVfirst = true;
        enlargeVlast = true;

        int Ubound = 0, Vbound = 0;
        double Ufirst = RealLast(), Ulast = RealFirst();
        double Vfirst = RealLast(), Vlast = RealFirst();

        CompactUVBounds(F, out double UF1, out double UF2, out double VF1, out double VF2);

        Surface theSurf = Tool.Surface(F)!;
        if (theSurf.GetType() == typeof(TrimmedSurface))
        {
            theSurf = ((TrimmedSurface)theSurf).BasisSurface();
        }

        if (theSurf.GetType() == typeof(ExtrusionSurface)
            || theSurf.GetType() == typeof(RevolutionSurface)
            || theSurf.GetType() == typeof(global::Kerf.Geometry.BezierSurface) || theSurf.GetType() == typeof(global::Kerf.Geometry.BSplineSurface))
        {
            Explorer Explo = new(F, ShapeType.Edge);
            for (; Explo.More(); Explo.Next())
            {
                Edge anEdge = ShapeCasts.Edge(Explo.Current());
                NCollection_List<BRepOffset_Interval> L = Analyse.Type(anEdge);
                if (!L.IsEmpty() || Tool.Degenerated(anEdge))
                {
                    // OCCT: L.First() on an empty list (a degenerated edge without intervals) is undefined in OCCT's
                    // release builds; the port throws Standard_NoSuchObject.
                    ChFiDS_TypeOfConcavity OT = L.First().Type();
                    if (OT == ChFiDS_Tangential || Tool.Degenerated(anEdge))
                    {
                        Curve2D aCurve = Tool.CurveOnSurface(anEdge, F, out double fpar, out double lpar)!;
                        if (aCurve.GetType() == typeof(TrimmedCurve2D))
                        {
                            aCurve = ((TrimmedCurve2D)aCurve).BasisCurve();
                        }

                        LineCurve2D? theLine = null;
                        if (aCurve.GetType() == typeof(LineCurve2D))
                        {
                            theLine = (LineCurve2D)aCurve;
                        }
                        else if (aCurve.GetType() == typeof(BezierCurve2D)
                                 || aCurve.GetType() == typeof(BSplineCurve2D))
                        {
                            // OCCT leaves these uninitialized (they are not read).
                            double newFpar = 0.0, newLpar = 0.0, deviation = 0.0;
                            theLine = ShapeCustom_Curve2d.ConvertToLine2d(aCurve,
                                                                          fpar,
                                                                          lpar,
                                                                          global::Kerf.Engine.Foundation.Precision.Confusion(),
                                                                          ref newFpar,
                                                                          ref newLpar,
                                                                          ref deviation);
                        }

                        if (theLine is not null)
                        {
                            Direction2D theDir = theLine.Direction();
                            if (theDir.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                Vbound++;
                                if (Tool.Degenerated(anEdge))
                                {
                                    if (Math.Abs(theLine.Location().Y() - VF1) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                                    {
                                        enlargeVfirst = false;
                                    }
                                    else
                                    { // theLine->Location().Y() is near VF2
                                        enlargeVlast = false;
                                    }
                                }
                                else
                                {
                                    if (theLine.Location().Y() < Vfirst)
                                    {
                                        Vfirst = theLine.Location().Y();
                                    }
                                    if (theLine.Location().Y() > Vlast)
                                    {
                                        Vlast = theLine.Location().Y();
                                    }
                                }
                            }
                            else if (theDir.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                Ubound++;
                                if (theLine.Location().X() < Ufirst)
                                {
                                    Ufirst = theLine.Location().X();
                                }
                                if (theLine.Location().X() > Ulast)
                                {
                                    Ulast = theLine.Location().X();
                                }
                            }
                        }
                    }
                }
            }
        }

        if (Ubound >= 2 || Vbound >= 2)
        {
            if (Ubound >= 2 && Math.Abs(UF1 - Ufirst) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                && Math.Abs(UF2 - Ulast) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                enlargeU = false;
            }
            if (Vbound >= 2 && Math.Abs(VF1 - Vfirst) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                && Math.Abs(VF2 - Vlast) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                enlargeVfirst = false;
                enlargeVlast = false;
            }
        }
    }

    //=================================================================================================

    // OCCT.NET: stands for OCCT's change in place of the surface theOld of the face F into theNew (see EnLargeFace):
    // F and the pcurves of its edges on theOld are put on theNew. Nothing is done where F or one of its edges is frozen
    // (a caller's face keeps its surface, decision 1101).
    private static void PutSurfaceInPlace(in Face F, Surface theOld, Surface theNew, in Placement L)
    {
        if (F.TShape()!.Locked())
        {
            return;
        }
        NCollection_IndexedMap<Shape> anEdges = new(ShapeHasher.Instance);
        Exploration.MapShapes(F, ShapeType.Edge, anEdges);
        for (int i = 1; i <= anEdges.Extent(); i++)
        {
            if (anEdges[i].TShape()!.Locked())
            {
                return;
            }
        }
        BrepBuilder B = new();
        B.UpdateFace(F, theNew, L, Tool.Tolerance(F));
        for (int i = 1; i <= anEdges.Extent(); i++)
        {
            Edge E = ShapeCasts.Edge(anEdges[i]);
            Placement aLoc = L.Predivided(E.Location());
            BrepTEdge TE = (BrepTEdge)E.TShape()!;
            for (NCollection_List<CurveRepresentation>.Iterator it = new(TE.ChangeCurves()); it.More(); it.Next())
            {
                if (it.Value() is CurveOnSurfaceRepresentation aCOS && aCOS.IsCurveOnSurface(theOld, aLoc))
                {
                    aCOS.ReplaceSurface(theNew);
                }
            }
            TE.Modified(true);
        }
    }

    /// <summary>
    /// Makes in <paramref name="NF"/> a face on the surface of <paramref name="F"/> enlarged (or, if
    /// <paramref name="ChangeGeom"/> is false, on the surface itself, within its bounds). True if the surface was
    /// changed; then with <paramref name="UpDatePCurve"/> the pcurves of F's edges move onto the new surface, which F
    /// receives. <paramref name="enlargeU"/>, <paramref name="enlargeVfirst"/> and <paramref name="enlargeVlast"/>
    /// allow or forbid the enlargement in U and at either end in V. With <paramref name="theExtensionMode"/> 1,
    /// potentially infinite surfaces are extended by the largest value and bounded ones by 25%; with 2, infinite
    /// surfaces by 10 times the face's size, bounded ones by 100%. The lengths, where not -1, set the enlargement at
    /// each side.
    /// </summary>
    public static bool EnLargeFace(in Face F, ref Face NF, bool ChangeGeom, bool UpDatePCurve = false,
                                   bool enlargeU = true, bool enlargeVfirst = true, bool enlargeVlast = true,
                                   int theExtensionMode = 1, double theLenBeforeUfirst = -1.0,
                                   double theLenAfterUlast = -1.0, double theLenBeforeVfirst = -1.0,
                                   double theLenAfterVlast = -1.0)
    {
        bool CanExtentSurface = ChangeGeom;
        bool UpdatePCurve = UpDatePCurve;
        bool theEnlargeU = enlargeU;
        bool theEnlargeVfirst = enlargeVfirst;
        bool theEnlargeVlast = enlargeVlast;

        //---------------------------
        // extension de la geometrie.
        //---------------------------
        Surface S = Tool.Surface(F, out Placement L)!;
        double UU1, VV1, UU2, VV2;
        bool uperiodic = false, vperiodic = false;
        bool isVV1degen = false, isVV2degen = false;
        double UF1, VF1, UF2, VF2;
        bool SurfaceChange = false;

        if (S.IsUPeriodic || S.IsVPeriodic)
        {
            // Calcul serre pour que les bornes ne couvre pas plus d une periode
            CompactUVBounds(F, out UF1, out UF2, out VF1, out VF2);
        }
        else
        {
            BrepTools.UVBounds(F, out UF1, out UF2, out VF1, out VF2);
        }

        S.Bounds(out double US1, out double US2, out double VS1, out double VS2);
        double coeff;
        if (theExtensionMode == 1)
        {
            UU1 = VV1 = -TheInfini;
            UU2 = VV2 = TheInfini;
            coeff = 0.25;
        }
        else
        {
            double FaceDU = UF2 - UF1;
            double FaceDV = VF2 - VF1;
            UU1 = UF1 - 10 * FaceDU;
            UU2 = UF2 + 10 * FaceDU;
            VV1 = VF1 - 10 * FaceDV;
            VV2 = VF2 + 10 * FaceDV;
            coeff = 1.0;
        }

        Surface aFaceSurface = S;
        if (CanExtentSurface)
        {
            SurfaceChange = EnlargeGeometry(ref S,
                                            ref UU1,
                                            ref UU2,
                                            ref VV1,
                                            ref VV2,
                                            ref isVV1degen,
                                            ref isVV2degen,
                                            UF1,
                                            UF2,
                                            VF1,
                                            VF2,
                                            coeff,
                                            theEnlargeU,
                                            theEnlargeVfirst,
                                            theEnlargeVlast,
                                            theLenBeforeUfirst,
                                            theLenAfterUlast,
                                            theLenBeforeVfirst,
                                            theLenAfterVlast);
            // OCCT.NET: on an offset surface OCCT sets the enlarged basis on the face's own surface in place, so F lies
            // on the enlarged surface from then on (its bounds grow; ExtentEdge and the callers read them). The port's
            // EnlargeGeometry gives a copy: on a working face it is put on F and its pcurves (decision 1101).
            if (aFaceSurface is global::Kerf.Geometry.OffsetSurface && S is global::Kerf.Geometry.OffsetSurface && !ReferenceEquals(S, aFaceSurface))
            {
                PutSurfaceInPlace(F, aFaceSurface, S, L);
            }
        }
        else
        {
            UU1 = Max(US1, UU1);
            UU2 = Min(UU2, US2);
            VV1 = Max(VS1, VV1);
            VV2 = Min(VS2, VV2);
        }

        if (S.IsUPeriodic)
        {
            uperiodic = true;
            double Period = S.UPeriod();
            double Delta = Period - (UF2 - UF1);
            double alpha = 0.1;
            UU1 = UF1 - alpha * Delta;
            UU2 = UF2 + alpha * Delta;
            if ((UU2 - UU1) > Period)
            {
                UU2 = UU1 + Period;
            }
        }
        if (S.IsVPeriodic)
        {
            vperiodic = true;
            double Period = S.VPeriod();
            double Delta = Period - (VF2 - VF1);
            double alpha = 0.1;
            VV1 = VF1 - alpha * Delta;
            VV2 = VF2 + alpha * Delta;
            if ((VV2 - VV1) > Period)
            {
                VV2 = VV1 + Period;
            }
        }

        // Special treatment for conical surfaces
        Surface theSurf = S;
        if (S.GetType() == typeof(TrimmedSurface))
        {
            theSurf = ((TrimmedSurface)S).BasisSurface();
        }
        if (theSurf.GetType() == typeof(ConicalSurface))
        {
            ConicalSurface ConicalS = (ConicalSurface)theSurf;
            Cone theCone = ConicalS.Cone();
            Point3D theApex = theCone.Apex();
            global::Kerf.Engine.Geometry.ElSLib.Parameters(theCone, theApex, out double Uapex, out double Vapex);
            if (VV1 < Vapex && Vapex < VV2)
            {
                // consider that VF1 and VF2 are on the same side from apex
                double TolApex = 1.0e-5;
                if (Vapex - VF1 >= TolApex || Vapex - VF2 >= TolApex)
                { // if (VF1 < Vapex || VF2 < Vapex)
                    VV2 = Vapex;
                }
                else
                {
                    VV1 = Vapex;
                }
            }
        }

        if (!theEnlargeU)
        {
            UU1 = UF1;
            UU2 = UF2;
        }
        if (!theEnlargeVfirst)
        {
            VV1 = VF1;
        }
        if (!theEnlargeVlast)
        {
            VV2 = VF2;
        }

        // Detect closedness in U and V directions
        BrepTools.DetectClosedness(F, out bool uclosed, out bool vclosed);
        if (uclosed && !uperiodic && (theLenBeforeUfirst != 0.0 || theLenAfterUlast != 0.0))
        {
            uclosed = false;
        }
        if (vclosed && !vperiodic && (theLenBeforeVfirst != 0.0 && theLenAfterVlast != 0.0))
        {
            vclosed = false;
        }

        MakeFace(S, UU1, UU2, VV1, VV2, uclosed, vclosed, isVV1degen, isVV2degen, ref NF);
        NF.Location(L);
        /*
          if (S->DynamicType() == STANDARD_TYPE(Geom_RectangularTrimmedSurface)) {
            BRep_Builder B;
            //----------------------------------------------------------------
            // utile pour les bouchons on ne doit pas changer leur geometrie.
            // (Ce que fait BRepLib_MakeFace si S est restreinte).
            // On remet S et on update les pcurves.
            //----------------------------------------------------------------
            ...
          }
        */
        if (SurfaceChange && UpdatePCurve)
        {
            Shape aLocalFace = F.Oriented(Forward);
            UpdatePCurves(ShapeCasts.Face(aLocalFace), NF);
            // UpdatePCurves(TopoDS::Face(F.Oriented(TopAbs_FORWARD)),BF);
            BrepBuilder BB = new();
            BB.UpdateFace(F, S, L, Tool.Tolerance(F));
        }

        NF.Orientation(F.Orientation());
        return SurfaceChange;
    }

    //=================================================================================================

    private static bool TryParameter(in Edge OE, in Vertex V, in Edge NE, double TolConf)
    {
        EdgeCurveAdaptor OC = new(OE);
        EdgeCurveAdaptor NC = new(NE);
        double Of = OC.FirstParameter();
        double Ol = OC.LastParameter();
        double Nf = NC.FirstParameter();
        double Nl = NC.LastParameter();
        double U = 0.0;
        Point3D P = Tool.Pnt(V);
        bool OK = false;

        if (P.Distance(OC.Value(Of)) < TolConf)
        {
            if (Of > Nf && Of < Nl && P.Distance(NC.Value(Of)) < TolConf)
            {
                OK = true;
                U = Of;
            }
        }
        if (P.Distance(OC.Value(Ol)) < TolConf)
        {
            if (Ol > Nf && Ol < Nl && P.Distance(NC.Value(Ol)) < TolConf)
            {
                OK = true;
                U = Ol;
            }
        }
        if (OK)
        {
            BrepBuilder B = new();
            Shape aLocalShape = NE.Oriented(Forward);
            Edge EE = ShapeCasts.Edge(aLocalShape);
            //    TopoDS_Edge EE = TopoDS::Edge(NE.Oriented(TopAbs_FORWARD));
            aLocalShape = V.Oriented(Internal);
            B.UpdateVertex(ShapeCasts.Vertex(aLocalShape), U, NE, Tool.Tolerance(NE));
            //    B.UpdateVertex(TopoDS::Vertex(V.Oriented(TopAbs_INTERNAL)),
            //		   U,NE,BRep_Tool::Tolerance(NE));
        }
        return OK;
    }

    //=================================================================================================

    /// <summary>
    /// Stores in <paramref name="MVE"/>, for each vertex of <paramref name="S"/>, the edges of S incident to it (each
    /// edge once per vertex).
    /// </summary>
    public static void MapVertexEdges(in Shape S,
                                      NCollection_DataMap<Shape, NCollection_List<Shape>> MVE)
    {
        Explorer exp = new();
        exp.Init(S.Oriented(Forward), ShapeType.Edge);
        NCollection_Map<Shape> DejaVu = new(ShapeHasher.Instance);
        for (; exp.More(); exp.Next())
        {
            Edge E = ShapeCasts.Edge(exp.Current());
            if (DejaVu.Add(E))
            {
                Exploration.Vertices(E, out Vertex V1, out Vertex V2);
                if (!MVE.IsBound(V1))
                {
                    NCollection_List<Shape> empty = new();
                    MVE.Bind(V1, empty);
                }
                MVE[V1].Append(E);
                if (!V1.IsSame(V2))
                {
                    if (!MVE.IsBound(V2))
                    {
                        NCollection_List<Shape> empty = new();
                        MVE.Bind(V2, empty);
                    }
                    MVE[V2].Append(E);
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Through the wire explorer, stores in <paramref name="NOnV1"/>, for an edge of <paramref name="W"/>, its
    /// neighbouring edge at its first vertex, and in <paramref name="NOnV2"/> its neighbour at its last vertex.
    /// </summary>
    public static void BuildNeighbour(in Wire W, in Face F,
                                      NCollection_DataMap<Shape, Shape> NOnV1,
                                      NCollection_DataMap<Shape, Shape> NOnV2)
    {
        Vertex V1, V2, VP1, VP2, FV1, FV2;
        Edge CurE, FirstE, PrecE;
        WireExplorer wexp = new();

        Shape aLocalFace = F.Oriented(Forward);
        Shape aLocalWire = W.Oriented(Forward);
        wexp.Init(ShapeCasts.Wire(aLocalWire), ShapeCasts.Face(aLocalFace));
        //  wexp.Init(TopoDS::Wire(W.Oriented(TopAbs_FORWARD)),
        //	    TopoDS::Face(F.Oriented(TopAbs_FORWARD)));
        CurE = FirstE = PrecE = wexp.Current();
        Exploration.Vertices(CurE, out V1, out V2);
        FV1 = VP1 = V1;
        FV2 = VP2 = V2;
        wexp.Next();
        while (wexp.More())
        {
            CurE = wexp.Current();
            Exploration.Vertices(CurE, out V1, out V2);
            if (V1.IsSame(VP1))
            {
                NOnV1.Bind(PrecE, CurE);
                NOnV1.Bind(CurE, PrecE);
            }
            if (V1.IsSame(VP2))
            {
                NOnV2.Bind(PrecE, CurE);
                NOnV1.Bind(CurE, PrecE);
            }
            if (V2.IsSame(VP1))
            {
                NOnV1.Bind(PrecE, CurE);
                NOnV2.Bind(CurE, PrecE);
            }
            if (V2.IsSame(VP2))
            {
                NOnV2.Bind(PrecE, CurE);
                NOnV2.Bind(CurE, PrecE);
            }
            PrecE = CurE;
            VP1 = V1;
            VP2 = V2;
            wexp.Next();
        }
        if (V1.IsSame(FV1))
        {
            NOnV1.Bind(FirstE, CurE);
            NOnV1.Bind(CurE, FirstE);
        }
        if (V1.IsSame(FV2))
        {
            NOnV2.Bind(FirstE, CurE);
            NOnV1.Bind(CurE, FirstE);
        }
        if (V2.IsSame(FV1))
        {
            NOnV1.Bind(FirstE, CurE);
            NOnV2.Bind(CurE, FirstE);
        }
        if (V2.IsSame(FV2))
        {
            NOnV2.Bind(FirstE, CurE);
            NOnV2.Bind(CurE, FirstE);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Makes in <paramref name="NF"/> the face <paramref name="F"/> extended: its edges bound in
    /// <paramref name="ToBuild"/> to a face are replaced by their sections with that face, their neighbours extended;
    /// <paramref name="ConstShapes"/> gives the edges and vertices to keep, and receives the new ones.
    /// </summary>
    public static void ExtentFace(in Face F, NCollection_DataMap<Shape, Shape> ConstShapes,
                                  NCollection_DataMap<Shape, Shape> ToBuild, Containment Side,
                                  double TolConf, ref Face NF)
    {
        Explorer exp = new(), exp2 = new();
        NCollection_DataMap<Shape, Shape> Build = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, Shape> Extent = new(ShapeHasher.Instance);
        Edge NE = new();
        BrepBuilder B = new();
        Face EF = new();

        // Construction de la boite englobante de la face a etendre et des bouchons pour
        // limiter les extensions.
        // Bnd_Box ContextBox;
        // BRepBndLib::Add(F,B);
        // TopTools_DataMapIteratorOfDataMapOfShape itTB(ToBuild);
        // for (; itTB.More(); itTB.Next()) {
        // BRepBndLib::Add(TopBuild.Value(), ContextBox);
        //}

        bool SurfaceChange;
        SurfaceChange = EnLargeFace(F, ref EF, true);

        Shape aLocalShape = EF.EmptyCopied();
        NF = ShapeCasts.Face(aLocalShape);
        //  NF = TopoDS::Face(EF.EmptyCopied());
        NF.Orientation(Forward);

        if (SurfaceChange)
        {
            //------------------------------------------------
            // Mise a jour des pcurves sur la surface de base.
            //------------------------------------------------
            Face Fforward = F;
            Fforward.Orientation(Forward);
            NCollection_IndexedMap<Shape> Emap = new(ShapeHasher.Instance);
            Exploration.MapShapes(Fforward, ShapeType.Edge, Emap);
            double f, l;
            for (int i = 1; i <= Emap.Extent(); i++)
            {
                Edge CE = ShapeCasts.Edge(Emap[i]);
                CE.Orientation(Forward);
                Edge Ecs = new(); // patch
                Curve2D? C2 = Tool.CurveOnSurface(CE, Fforward, out f, out l);
                if (C2 is not null)
                {
                    if (ConstShapes.IsBound(CE))
                    {
                        Ecs = ShapeCasts.Edge(ConstShapes[CE]);
                        Tool.Range(Ecs, out f, out l);
                    }
                    if (Tool.IsClosed(CE, Fforward))
                    {
                        Shape aLocalShapeReversedCE = CE.Reversed();
                        Curve2D? C2R = Tool.CurveOnSurface(ShapeCasts.Edge(aLocalShapeReversedCE), Fforward,
                                                                     out f, out l);
                        //	  occ::handle<Geom2d_Curve> C2R =
                        //	    BRep_Tool::CurveOnSurface(TopoDS::Edge(CE.Reversed()),F,f,l);
                        B.UpdateEdge(CE, C2, C2R, EF, Tool.Tolerance(CE));
                        if (!Ecs.IsNull)
                        {
                            B.UpdateEdge(Ecs, C2, C2R, EF, Tool.Tolerance(CE));
                        }
                    }
                    else
                    {
                        B.UpdateEdge(CE, C2, EF, Tool.Tolerance(CE));
                        if (!Ecs.IsNull)
                        {
                            B.UpdateEdge(Ecs, C2, EF, Tool.Tolerance(CE));
                        }
                    }
                    B.Range(CE, f, l);
                    if (!Ecs.IsNull)
                    {
                        B.Range(Ecs, f, l);
                    }
                }
            }
        }

        for (exp.Init(F.Oriented(Forward), ShapeType.Wire); exp.More(); exp.Next())
        {
            Wire W = ShapeCasts.Wire(exp.Current());
            NCollection_DataMap<Shape, NCollection_List<Shape>> MVE = new(ShapeHasher.Instance); // Vertex -> Edges incidentes.
            NCollection_DataMap<Shape, Shape> NOnV1 = new(ShapeHasher.Instance);
            NCollection_DataMap<Shape, Shape> NOnV2 = new(ShapeHasher.Instance);

            MapVertexEdges(W, MVE);
            BuildNeighbour(W, F, NOnV1, NOnV2);

            NCollection_List<Shape> LInt1 = new(), LInt2 = new();
            Face StopFace = new();
            //------------------------------------------------
            // Construction edges
            //------------------------------------------------
            for (exp2.Init(W.Oriented(Forward), ShapeType.Edge); exp2.More(); exp2.Next())
            {
                Edge E = ShapeCasts.Edge(exp2.Current());
                if (ConstShapes.IsBound(E))
                {
                    ToBuild.UnBind(E);
                }
                if (ToBuild.IsBound(E))
                {
                    NCollection_List<Shape> LOE = new();
                    LOE.Append(E);
                    if (TryProject(ShapeCasts.Face(ToBuild[E]), EF, LOE, LInt2, LInt1, Side, TolConf) && !LInt1.IsEmpty())
                    {
                        ToBuild.UnBind(E);
                    }
                }
            }

            for (exp2.Init(W.Oriented(Forward), ShapeType.Edge); exp2.More(); exp2.Next())
            {
                Edge E = ShapeCasts.Edge(exp2.Current());
                if (ConstShapes.IsBound(E))
                {
                    ToBuild.UnBind(E);
                }
                if (ToBuild.IsBound(E))
                {
                    EnLargeFace(ShapeCasts.Face(ToBuild[E]), ref StopFace, false);
                    Face NullFace = new();
                    Inter3D(EF, StopFace, LInt1, LInt2, Side, E, NullFace, NullFace);
                    // No intersection, it may happen for example for a chosen (non-offsetted) planar face and
                    // its neighbour offsetted cylindrical face, if the offset is directed so that
                    // the radius of the cylinder becomes smaller.
                    if (LInt1.IsEmpty())
                    {
                        continue;
                    }
                    if (LInt1.Extent() > 1)
                    {
                        // l intersection est en plusieurs edges (franchissement de couture)
                        SelectEdge(F, EF, E, LInt1);
                    }
                    NE = ShapeCasts.Edge(LInt1.First());
                    BrepTEdge TE = (BrepTEdge)NE.TShape()!;
                    TE.Tolerance(TE.Tolerance() * 10.0); //????
                    if (NE.Orientation() == E.Orientation())
                    {
                        Build.Bind(E, NE.Oriented(Forward));
                    }
                    else
                    {
                        Build.Bind(E, NE.Oriented(Reversed));
                    }
                    Edge EOnV1 = ShapeCasts.Edge(NOnV1[E]);
                    if (!ToBuild.IsBound(EOnV1) && !ConstShapes.IsBound(EOnV1) && !Build.IsBound(EOnV1))
                    {
                        ExtentEdge(F, EF, EOnV1, out NE);
                        Build.Bind(EOnV1, NE.Oriented(Forward));
                    }
                    Edge EOnV2 = ShapeCasts.Edge(NOnV2[E]);
                    if (!ToBuild.IsBound(EOnV2) && !ConstShapes.IsBound(EOnV2) && !Build.IsBound(EOnV2))
                    {
                        ExtentEdge(F, EF, EOnV2, out NE);
                        Build.Bind(EOnV2, NE.Oriented(Forward));
                    }
                }
            }

            //------------------------------------------------
            // Construction Vertex.
            //------------------------------------------------
            NCollection_List<Shape> LV = new();
            double f, l;
            Vertex V1, V2;

            for (exp2.Init(W.Oriented(Forward), ShapeType.Edge); exp2.More(); exp2.Next())
            {
                Edge E = ShapeCasts.Edge(exp2.Current());
                Exploration.Vertices(E, out V1, out V2);
                Tool.Range(E, out f, out l);
                Vertex V = new();
                if (Build.IsBound(E))
                {
                    Edge NEOnV1 = ShapeCasts.Edge(NOnV1[E]);
                    if (Build.IsBound(NEOnV1) && (ToBuild.IsBound(E) || ToBuild.IsBound(NEOnV1)))
                    {
                        if (E.IsSame(NEOnV1))
                        {
                            V = Exploration.FirstVertex(ShapeCasts.Edge(Build[E]));
                        }
                        else
                        {
                            //---------------
                            // intersection.
                            //---------------
                            if (!Build.IsBound(V1))
                            {
                                Inter2d(EF, ShapeCasts.Edge(Build[E]), ShapeCasts.Edge(Build[NEOnV1]), LV,
                                        /*TolConf*/ global::Kerf.Engine.Foundation.Precision.Confusion());

                                if (!LV.IsEmpty())
                                {
                                    if (Build[E].Orientation() == Forward)
                                    {
                                        V = ShapeCasts.Vertex(LV.First());
                                    }
                                    else
                                    {
                                        V = ShapeCasts.Vertex(LV.Last());
                                    }
                                }
                                else
                                {
                                    return;
                                }
                            }
                            else
                            {
                                V = ShapeCasts.Vertex(Build[V1]);
                                if (MVE[V1].Extent() > 2)
                                {
                                    V.Orientation(Forward);
                                    if (Build[E].Orientation() == Reversed)
                                    {
                                        V.Orientation(Reversed);
                                    }

                                    ProjectVertexOnEdge(V, ShapeCasts.Edge(Build[E]), TolConf);
                                }
                            }
                        }
                    }
                    else
                    {
                        //------------
                        // projection
                        //------------
                        V = V1;
                        if (ConstShapes.IsBound(V1))
                        {
                            V = ShapeCasts.Vertex(ConstShapes[V1]);
                        }
                        V.Orientation(Forward);
                        if (Build[E].Orientation() == Reversed)
                        {
                            V.Orientation(Reversed);
                        }
                        if (!TryParameter(E, V, ShapeCasts.Edge(Build[E]), TolConf))
                        {
                            ProjectVertexOnEdge(V, ShapeCasts.Edge(Build[E]), TolConf);
                        }
                    }

                    ConstShapes.Bind(V1, V);
                    Build.Bind(V1, V);
                    Edge NEOnV2 = ShapeCasts.Edge(NOnV2[E]);
                    if (Build.IsBound(NEOnV2) && (ToBuild.IsBound(E) || ToBuild.IsBound(NEOnV2)))
                    {
                        if (E.IsSame(NEOnV2))
                        {
                            V = Exploration.LastVertex(ShapeCasts.Edge(Build[E]));
                        }
                        else
                        {
                            //--------------
                            // intersection.
                            //---------------

                            if (!Build.IsBound(V2))
                            {
                                Inter2d(EF, ShapeCasts.Edge(Build[E]), ShapeCasts.Edge(Build[NEOnV2]), LV,
                                        /*TolConf*/ global::Kerf.Engine.Foundation.Precision.Confusion());

                                if (!LV.IsEmpty())
                                {
                                    if (Build[E].Orientation() == Forward)
                                    {
                                        V = ShapeCasts.Vertex(LV.Last());
                                    }
                                    else
                                    {
                                        V = ShapeCasts.Vertex(LV.First());
                                    }
                                }
                                else
                                {
                                    return;
                                }
                            }
                            else
                            {
                                V = ShapeCasts.Vertex(Build[V2]);
                                if (MVE[V2].Extent() > 2)
                                {
                                    V.Orientation(Reversed);
                                    if (Build[E].Orientation() == Reversed)
                                    {
                                        V.Orientation(Forward);
                                    }

                                    ProjectVertexOnEdge(V, ShapeCasts.Edge(Build[E]), TolConf);
                                }
                            }
                        }
                    }
                    else
                    {
                        //------------
                        // projection
                        //------------
                        V = V2;
                        if (ConstShapes.IsBound(V2))
                        {
                            V = ShapeCasts.Vertex(ConstShapes[V2]);
                        }
                        V.Orientation(Reversed);
                        if (Build[E].Orientation() == Reversed)
                        {
                            V.Orientation(Forward);
                        }
                        if (!TryParameter(E, V, ShapeCasts.Edge(Build[E]), TolConf))
                        {
                            ProjectVertexOnEdge(V, ShapeCasts.Edge(Build[E]), TolConf);
                        }
                    }
                    ConstShapes.Bind(V2, V);
                    Build.Bind(V2, V);
                }
            }

            Wire NW = new();
            Vertex NV1, NV2;
            Orientation Or;
            double U1, U2;
            double eps = global::Kerf.Engine.Foundation.Precision.Confusion();

            // OCCT_DEBUG: an unused location is left out.
            B.MakeWire(ref NW);

            //-----------------
            // Reconstruction.
            //-----------------
            for (exp2.Init(W.Oriented(Forward), ShapeType.Edge); exp2.More(); exp2.Next())
            {
                Edge E = ShapeCasts.Edge(exp2.Current());
                Exploration.Vertices(E, out V1, out V2);
                if (Build.IsBound(E))
                {
                    NE = ShapeCasts.Edge(Build[E]);
                    Tool.Range(NE, out f, out l);
                    Or = NE.Orientation();
                    //-----------------------------------------------------
                    // Copy pour virer les vertex deja sur la nouvelle edge.
                    //-----------------------------------------------------
                    NV1 = ShapeCasts.Vertex(ConstShapes[V1]);
                    NV2 = ShapeCasts.Vertex(ConstShapes[V2]);

                    Shape aLocalVertexOrientedNV1 = NV1.Oriented(Internal);
                    Shape aLocalEdge = NE.Oriented(Internal);

                    U1 = Tool.Parameter(ShapeCasts.Vertex(aLocalVertexOrientedNV1), ShapeCasts.Edge(aLocalEdge));
                    aLocalVertexOrientedNV1 = NV2.Oriented(Internal);
                    aLocalEdge = NE.Oriented(Forward);
                    U2 = Tool.Parameter(ShapeCasts.Vertex(aLocalVertexOrientedNV1), ShapeCasts.Edge(aLocalEdge));
                    //	U1 = BRep_Tool::Parameter
                    //	  (TopoDS::Vertex(NV1.Oriented(TopAbs_INTERNAL)),
                    //	   TopoDS::Edge  (NE .Oriented(TopAbs_FORWARD)));
                    //	U2 = BRep_Tool::Parameter
                    //	  (TopoDS::Vertex(NV2.Oriented(TopAbs_INTERNAL)),
                    //	   TopoDS::Edge  (NE.Oriented(TopAbs_FORWARD)));
                    aLocalEdge = NE.EmptyCopied();
                    NE = ShapeCasts.Edge(aLocalEdge);
                    NE.Orientation(Forward);
                    if (NV1.IsSame(NV2))
                    {
                        //--------------
                        // edge ferme.
                        //--------------
                        if (Or == Forward)
                        {
                            U1 = f;
                            U2 = l;
                        }
                        else
                        {
                            U1 = l;
                            U2 = f;
                        }
                        if (Or == Forward)
                        {
                            if (U1 > U2)
                            {
                                if (Math.Abs(U1 - l) < eps)
                                {
                                    U1 = f;
                                }
                                if (Math.Abs(U2 - f) < eps)
                                {
                                    U2 = l;
                                }
                            }
                            Shape aLocalVertex = NV1.Oriented(Forward);
                            B.Add(NE, ShapeCasts.Vertex(aLocalVertex));
                            aLocalVertex = NV2.Oriented(Reversed);
                            B.Add(NE, ShapeCasts.Vertex(aLocalVertex));
                            //		B.Add (NE,TopoDS::Vertex(NV1.Oriented(TopAbs_FORWARD )));
                            //		B.Add (NE,TopoDS::Vertex(NV2.Oriented(TopAbs_REVERSED)));
                            B.Range(NE, U1, U2);
                            ConstShapes.Bind(E, NE);
                            NE.Orientation(E.Orientation());
                        }
                        else
                        {
                            if (U2 > U1)
                            {
                                if (Math.Abs(U2 - l) < eps)
                                {
                                    U2 = f;
                                }
                                if (Math.Abs(U1 - f) < eps)
                                {
                                    U1 = l;
                                }
                            }
                            Shape aLocalVertex = NV2.Oriented(Forward);
                            B.Add(NE, ShapeCasts.Vertex(aLocalVertex));
                            aLocalVertex = NV1.Oriented(Reversed);
                            B.Add(NE, ShapeCasts.Vertex(aLocalVertex));
                            //		B.Add (NE,TopoDS::Vertex(NV2.Oriented(TopAbs_FORWARD )));
                            //		B.Add (NE,TopoDS::Vertex(NV1.Oriented(TopAbs_REVERSED)));
                            B.Range(NE, U2, U1);
                            ConstShapes.Bind(E, NE.Oriented(Reversed));
                            NE.Orientation(global::Kerf.Engine.Topology.TopAbs.Reverse(E.Orientation()));
                        }
                    }
                    else
                    {
                        //-------------------
                        // edge is not ferme.
                        //-------------------
                        if (Or == Forward)
                        {
                            if (U1 > U2)
                            {
                                Shape aLocalVertex = NV2.Oriented(Forward);
                                B.Add(NE, ShapeCasts.Vertex(aLocalVertex));
                                aLocalVertex = NV1.Oriented(Reversed);
                                B.Add(NE, ShapeCasts.Vertex(aLocalVertex));
                                //		B.Add (NE,TopoDS::Vertex(NV2.Oriented(TopAbs_FORWARD )));
                                //		B.Add (NE,TopoDS::Vertex(NV1.Oriented(TopAbs_REVERSED)));
                                B.Range(NE, U2, U1);
                            }
                            else
                            {
                                Shape aLocalVertex = NV1.Oriented(Forward);
                                B.Add(NE, ShapeCasts.Vertex(aLocalVertex));
                                aLocalVertex = NV2.Oriented(Reversed);
                                B.Add(NE, ShapeCasts.Vertex(aLocalVertex));
                                //		  B.Add (NE,TopoDS::Vertex(NV1.Oriented(TopAbs_FORWARD )));
                                //		  B.Add (NE,TopoDS::Vertex(NV2.Oriented(TopAbs_REVERSED)));
                                B.Range(NE, U1, U2);
                            }
                            ConstShapes.Bind(E, NE);
                            NE.Orientation(E.Orientation());
                        }
                        else
                        {
                            if (U2 > U1)
                            {
                                Shape aLocalVertex = NV1.Oriented(Forward);
                                B.Add(NE, ShapeCasts.Vertex(aLocalVertex));
                                aLocalVertex = NV2.Oriented(Reversed);
                                B.Add(NE, ShapeCasts.Vertex(aLocalVertex));
                                //		B.Add (NE,TopoDS::Vertex(NV1.Oriented(TopAbs_FORWARD )));
                                //		B.Add (NE,TopoDS::Vertex(NV2.Oriented(TopAbs_REVERSED)));
                                B.Range(NE, U1, U2);
                                ConstShapes.Bind(E, NE);
                                NE.Orientation(E.Orientation());
                            }
                            else
                            {
                                Shape aLocalVertex = NV2.Oriented(Forward);
                                B.Add(NE, ShapeCasts.Vertex(aLocalVertex));
                                aLocalVertex = NV1.Oriented(Reversed);
                                B.Add(NE, ShapeCasts.Vertex(aLocalVertex));
                                //		  B.Add (NE,TopoDS::Vertex(NV2.Oriented(TopAbs_FORWARD )));
                                //		  B.Add (NE,TopoDS::Vertex(NV1.Oriented(TopAbs_REVERSED)));
                                B.Range(NE, U2, U1);
                                ConstShapes.Bind(E, NE.Oriented(Reversed));
                                NE.Orientation(global::Kerf.Engine.Topology.TopAbs.Reverse(E.Orientation()));
                            }
                        }
                    }
                    Build.UnBind(E);
                } // Build.IsBound(E)
                else if (ConstShapes.IsBound(E))
                { // !Build.IsBound(E)
                    NE = ShapeCasts.Edge(ConstShapes[E]);
                    BuildPCurves(NE, NF);
                    Or = NE.Orientation();
                    if (Or == Reversed)
                    {
                        NE.Orientation(global::Kerf.Engine.Topology.TopAbs.Reverse(E.Orientation()));
                    }
                    else
                    {
                        NE.Orientation(E.Orientation());
                    }
                }
                else
                {
                    NE = E;
                    ConstShapes.Bind(E, NE.Oriented(Forward));
                }
                B.Add(NW, NE);
            }
            B.Add(NF, NW.Oriented(W.Orientation()));
        }
        NF.Orientation(F.Orientation());
        BrepTools.Update(NF); // Maj des UVPoints
    }

    //=================================================================================================

    /// <summary>
    /// Removes the invalid parts of an offset shape: the shells with free boundaries other than
    /// <paramref name="Boundary"/>'s edges (the caps' free boundaries) are removed. Null if nothing is left.
    /// </summary>
    public static Shape Deboucle3D(in Shape S, NCollection_Map<Shape> Boundary)
    {
        Shape SS = new();
        switch (S.ShapeType())
        {
            case ShapeType.Shell:
            {
                // if the shell contains free borders that do not belong to the
                // free borders of caps (Boundary) it is removed.
                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> Map =
                    new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(S, ShapeType.Edge, ShapeType.Face, Map);

                bool JeGarde = true;
                for (int i = 1; i <= Map.Extent() && JeGarde; i++)
                {
                    NCollection_List<Shape> aLF = Map[i];
                    if (aLF.Extent() < 2)
                    {
                        Edge anEdge = ShapeCasts.Edge(Map.FindKey(i));
                        if (anEdge.Orientation() == Internal)
                        {
                            Face aFace = ShapeCasts.Face(aLF.First());
                            if (aFace.Orientation() != Internal)
                            {
                                continue;
                            }
                        }
                        if (!Boundary.Contains(anEdge) && !Tool.Degenerated(anEdge))
                        {
                            JeGarde = false;
                        }
                    }
                }
                if (JeGarde)
                {
                    SS = S;
                }
            }
            break;

            case ShapeType.Compound:
            case ShapeType.Solid:
            {
                // iterate on sub-shapes and add non-empty.
                Iterator it = new(S);
                Shape SubShape;
                int NbSub = 0;
                BrepBuilder B = new();
                if (S.ShapeType() == ShapeType.Compound)
                {
                    Compound aC = new();
                    B.MakeCompound(ref aC);
                    SS = aC;
                }
                else
                {
                    Solid aSo = new();
                    B.MakeSolid(ref aSo);
                    SS = aSo;
                }
                for (; it.More(); it.Next())
                {
                    Shape CurS = it.Value();
                    SubShape = Deboucle3D(CurS, Boundary);
                    if (!SubShape.IsNull)
                    {
                        B.Add(SS, SubShape);
                        NbSub++;
                    }
                }
                if (NbSub == 0)
                {
                    SS = new Shape();
                }
            }
            break;

            default:
                break;
        }

        return SS;
    }

    //=================================================================================================

    private static bool IsInOut(BRepTopAdaptor_FClass2d FC, CurveAdaptor2D AC, Containment S)
    {
        double Def = 100 * global::Kerf.Engine.Foundation.Precision.Confusion();
        QuasiUniformDeflection QU = new(AC, Def);

        for (int i = 1; i <= QU.PointCount; i++)
        {
            Point2D P = AC.Value(QU.Parameter(i));
            if (FC.Perform(P) != S)
            {
                return false;
            }
        }
        return true;
    }

    //=================================================================================================

    /// <summary>
    /// For a positive offset, reverses in <paramref name="AsDes"/>'s descendants of the offset faces of
    /// <paramref name="SI"/>'s faces the new edges (<paramref name="NewEdges"/>) whose pcurves lie outside the
    /// initial face.
    /// </summary>
    public static void CorrectOrientation(in Shape SI, NCollection_IndexedMap<Shape> NewEdges,
                                          BRepAlgo_AsDes AsDes, BRepAlgo_Image InitOffset, double Offset)
    {
        Explorer exp = new();
        exp.Init(SI, ShapeType.Face);
        double f = 0.0, l = 0.0;

        for (; exp.More(); exp.Next())
        {
            Face FI = ShapeCasts.Face(exp.Current());
            NCollection_List<Shape> LOF = InitOffset.Image(FI);
            NCollection_List<Shape>.Iterator it = new(LOF);
            for (; it.More(); it.Next())
            {
                Face OF = ShapeCasts.Face(it.Value());
                NCollection_List<Shape> LOE = AsDes.ChangeDescendant(OF);
                NCollection_List<Shape>.Iterator itE = new(LOE);

                bool YaInt = false;
                for (; itE.More(); itE.Next())
                {
                    Edge OE = ShapeCasts.Edge(itE.Value());
                    if (NewEdges.Contains(OE))
                    {
                        YaInt = true;
                        break;
                    }
                }
                if (YaInt)
                {
                    Shape aLocalFace = FI.Oriented(Forward);
                    BRepTopAdaptor_FClass2d FC = new(ShapeCasts.Face(aLocalFace), global::Kerf.Engine.Foundation.Precision.Confusion());
                    //	BRepTopAdaptor_FClass2d FC (TopoDS::Face(FI.Oriented(TopAbs_FORWARD)),
                    //				    Precision::Confusion());
                    for (itE.Initialize(LOE); itE.More(); itE.Next())
                    {
                        ref Shape OE = ref itE.ChangeValue();
                        if (NewEdges.Contains(OE))
                        {
                            Curve2D? CO2d = Tool.CurveOnSurface(ShapeCasts.Edge(OE), OF, out f, out l);
                            CurveAdaptor2D AC = new(CO2d!, f, l);

                            if (Offset > 0)
                            {
                                if (IsInOut(FC, AC, Out))
                                {
                                    OE.Reverse();
                                }
                            }
                            //	    else {
                            //	      if (IsInOut(FC,AC,TopAbs_IN)) OE.Reverse();
                            //	    }
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Compares the normals of the planar faces <paramref name="theFace1"/> and <paramref name="theFace2"/>: true if
    /// both are planar and their normals agree within <paramref name="theTolAng"/>.
    /// </summary>
    public static bool CheckPlanesNormals(in Face theFace1, in Face theFace2, double theTolAng = 1.0e-8)
    {
        FaceSurfaceAdaptor aBAS1 = new(theFace1, false), aBAS2 = new(theFace2, false);
        if (aBAS1.GetType() != Plane || aBAS2.GetType() != Plane)
        {
            return false;
        }
        //
        Direction3D aDN1 = aBAS1.Plane().Position().Direction();
        if (theFace1.Orientation() == Reversed)
        {
            aDN1.Reverse();
        }
        //
        Direction3D aDN2 = aBAS2.Plane().Position().Direction();
        if (theFace2.Orientation() == Reversed)
        {
            aDN2.Reverse();
        }
        //
        double anAngle = aDN1.Angle(aDN2);
        return (anAngle < theTolAng);
    }

    //=================================================================================================

    private static void PerformPlanes(in Face theFace1, in Face theFace2, Containment theSide,
                                      NCollection_List<Shape> theL1, NCollection_List<Shape> theL2)
    {
        theL1.Clear();
        theL2.Clear();
        // Intersect the planes using IntTools_FaceFace directly
        IntTools_FaceFace aFF = new();
        aFF.SetParameters(true, true, true, global::Kerf.Engine.Foundation.Precision.Confusion());
        aFF.Perform(theFace1, theFace2);
        //
        if (!aFF.IsDone())
        {
            return;
        }
        //
        NCollection_Sequence<IntTools_Curve> aSC = aFF.InternalLines();
        if (aSC.IsEmpty())
        {
            return;
        }
        //
        // In Plane/Plane intersection only one curve is always produced.
        // Make the edge from this section curve.
        Edge aE = new();
        {
            BrepBuilder aBB = new();
            IntTools_Curve aIC = aSC[1];
            Curve3D? aC3D = aIC.Curve();
            aBB.MakeEdge(ref aE, aC3D, aIC.Tolerance());
            // Get bounds of the curve
            double aTF = 0.0, aTL = 0.0;
            Point3D aPF = new(), aPL = new();
            aIC.Bounds(ref aTF, ref aTL, ref aPF, ref aPL);
            // Make the bounding vertices
            Vertex aVF = new(), aVL = new();
            aBB.MakeVertex(ref aVF, aPF, aIC.Tolerance());
            aBB.MakeVertex(ref aVL, aPL, aIC.Tolerance());
            aVL.Orientation(Reversed);
            // Add vertices to the edge
            aBB.Add(aE, aVF);
            aBB.Add(aE, aVL);
            // Add 2D curves to the edge
            aBB.UpdateEdge(aE, aIC.FirstCurve2d(), theFace1, aIC.Tolerance());
            aBB.UpdateEdge(aE, aIC.SecondCurve2d(), theFace2, aIC.Tolerance());
            // Update range of the new edge
            aBB.Range(aE, aTF, aTL);
        }
        //
        // Orient section
        OrientSection(aE, theFace1, theFace2, out Orientation O1, out Orientation O2);
        if (theSide == Out)
        {
            O1 = global::Kerf.Engine.Topology.TopAbs.Reverse(O1);
            O2 = global::Kerf.Engine.Topology.TopAbs.Reverse(O2);
        }
        //
        BRepLib.SameParameter(aE, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
        //
        // Add edge to result
        theL1.Append(aE.Oriented(O1));
        theL2.Append(aE.Oriented(O2));
    }

    //=======================================================================
    // function : IsInf
    // purpose  : Checks if the given value is close to infinite (TheInfini)
    //=======================================================================
    private static bool IsInf(double theVal)
    {
        return (theVal > TheInfini * 0.9);
    }

    private static void UpdateVertexTolerances(in Face theFace)
    {
        BrepBuilder BB = new();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> VEmap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theFace, ShapeType.Vertex, ShapeType.Edge, VEmap);

        for (int i = 1; i <= VEmap.Extent(); i++)
        {
            Vertex aVertex = ShapeCasts.Vertex(VEmap.FindKey(i));
            NCollection_List<Shape> Elist = VEmap[i];
            Point3D PntVtx = Tool.Pnt(aVertex);
            NCollection_List<Shape>.Iterator itl = new(Elist);
            for (; itl.More(); itl.Next())
            {
                Edge anEdge = ShapeCasts.Edge(itl.Value());
                Exploration.Vertices(anEdge, out Vertex V1, out Vertex V2);
                Tool.Range(anEdge, out double fpar, out double lpar);
                double aParam = (V1.IsSame(aVertex)) ? fpar : lpar;
                if (!Tool.Degenerated(anEdge))
                {
                    EdgeCurveAdaptor BAcurve = new(anEdge);
                    Point3D aPnt = BAcurve.Value(aParam);
                    double aDist = PntVtx.Distance(aPnt);
                    BB.UpdateVertex(aVertex, aDist);
                    if (V1.IsSame(V2))
                    {
                        aPnt = BAcurve.Value(lpar);
                        aDist = PntVtx.Distance(aPnt);
                        BB.UpdateVertex(aVertex, aDist);
                    }
                }
                EdgeCurveAdaptor BAcurveonsurf = new(anEdge, theFace);
                {
                    Point3D aPnt = BAcurveonsurf.Value(aParam);
                    double aDist = PntVtx.Distance(aPnt);
                    BB.UpdateVertex(aVertex, aDist);
                    if (V1.IsSame(V2))
                    {
                        aPnt = BAcurveonsurf.Value(lpar);
                        aDist = PntVtx.Distance(aPnt);
                        BB.UpdateVertex(aVertex, aDist);
                    }
                }
            }
        }
    }
}
