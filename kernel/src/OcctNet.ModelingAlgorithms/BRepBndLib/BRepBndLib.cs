// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBndLib/BRepBndLib.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBndLib/BRepBndLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The bounding boxes of shapes: axis-aligned (<see cref="Add"/>, <see cref="AddClose"/>,
/// <see cref="AddOptimal"/>) and oriented (<see cref="AddOBB"/>).
/// </summary>
/// <remarks>
/// The <c>Add</c> methods enlarge a caller's box passed by <c>ref</c>, as BndLib's do (decision 263); the
/// <c>Box</c> methods return a new box. They only read the shape: a shape carrying meshes gives the same box whoever
/// asks (design 5).
/// </remarks>
public static partial class BRepBndLib
{
    /// <summary>
    /// Adds the shape <paramref name="S"/> to the box <paramref name="B"/>: each face (its triangulation if it has one
    /// and <paramref name="useTriangulation"/>), then each edge outside the faces (its polygon if it has one), then
    /// each vertex outside the edges. After each one the box is enlarged by its tolerance (plus the deflection of a
    /// mesh). The box may be somewhat larger than the shape.
    /// </summary>
    public static void Add(in TopoDS_Shape S, ref Bnd_Box B, bool useTriangulation = true)
    {
        var ex = new TopExp_Explorer();

        // Add the faces
        var BS = new BRepAdaptor_Surface();
        TopLoc_Location l, aDummyLoc;
        int i, nbNodes;
        var BC = new BRepAdaptor_Curve();

        for (ex.Init(S, TopAbs_FACE); ex.More(); ex.Next())
        {
            TopoDS_Face F = TopoDS.Face(ex.Current());
            Poly_Triangulation? T = BRep_Tool.Triangulation(F, out l);
            Geom_Surface? GS = BRep_Tool.Surface(F, out aDummyLoc);
            if ((useTriangulation || GS is null) && T is not null && T.MinMax(ref B, l.Transformation()))
            {
                //       B.Enlarge(T->Deflection());
                B.Enlarge(T.Deflection() + BRep_Tool.Tolerance(F));
            }
            else
            {
                if (GS is not null)
                {
                    BS.Initialize(F, false);
                    if (BS.GetType() != GeomAbs_Plane)
                    {
                        BS.Initialize(F);
                        BndLib_AddSurface.Add(BS, BRep_Tool.Tolerance(F), ref B);
                    }
                    else
                    {
                        // on travaille directement sur les courbes 3d.
                        var ex2 = new TopExp_Explorer(F, TopAbs_EDGE);
                        if (!ex2.More())
                        {
                            BS.Initialize(F);
                            BndLib_AddSurface.Add(BS, BRep_Tool.Tolerance(F), ref B);
                        }
                        else
                        {
                            for (; ex2.More(); ex2.Next())
                            {
                                TopoDS_Edge anEdge = TopoDS.Edge(ex2.Current());
                                if (BRep_Tool.IsGeometric(anEdge))
                                {
                                    BC.Initialize(anEdge);
                                    BndLib_Add3dCurve.Add(BC, BRep_Tool.Tolerance(anEdge), ref B);
                                }
                            }

                            B.Enlarge(BRep_Tool.Tolerance(F));
                        }
                    }
                }
            }
        }

        // Add the edges not in faces
        for (ex.Init(S, TopAbs_EDGE, TopAbs_FACE); ex.More(); ex.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(ex.Current());
            if (!useTriangulation && BRep_Tool.IsGeometric(E))
            {
                BC.Initialize(E);
                BndLib_Add3dCurve.Add(BC, BRep_Tool.Tolerance(E), ref B);
                continue;
            }

            Poly_Polygon3D? P3d = BRep_Tool.Polygon3D(E, out l);
            if (P3d is not null && P3d.NbNodes() > 0)
            {
                NCollection_Array1<gp_Pnt> Nodes = P3d.Nodes();
                nbNodes = P3d.NbNodes();
                for (i = 1; i <= nbNodes; i++)
                {
                    if (l.IsIdentity())
                    {
                        B.Add(Nodes[i]);
                    }
                    else
                    {
                        B.Add(Nodes[i].Transformed(l.Transformation()));
                    }
                }

                //       B.Enlarge(P3d->Deflection());
                B.Enlarge(P3d.Deflection() + BRep_Tool.Tolerance(E));
            }
            else
            {
                BRep_Tool.PolygonOnTriangulation(E, out Poly_PolygonOnTriangulation? Poly, out Poly_Triangulation? T, out l);
                if (Poly is not null && T is not null && T.NbNodes() > 0)
                {
                    NCollection_Array1<int> Indices = Poly.Nodes();
                    nbNodes = Indices.Length();
                    if (l.IsIdentity())
                    {
                        for (i = 1; i <= nbNodes; i++)
                        {
                            B.Add(T.Node(Indices[i]));
                        }
                    }
                    else
                    {
                        for (i = 1; i <= nbNodes; i++)
                        {
                            B.Add(T.Node(Indices[i]).Transformed(l.Transformation()));
                        }
                    }

                    // 	B.Enlarge(T->Deflection());
                    B.Enlarge(Poly.Deflection() + BRep_Tool.Tolerance(E));
                }
                else
                {
                    if (BRep_Tool.IsGeometric(E))
                    {
                        BC.Initialize(E);
                        BndLib_Add3dCurve.Add(BC, BRep_Tool.Tolerance(E), ref B);
                    }
                }
            }
        }

        // Add the vertices not in edges

        for (ex.Init(S, TopAbs_VERTEX, TopAbs_EDGE); ex.More(); ex.Next())
        {
            B.Add(BRep_Tool.Pnt(TopoDS.Vertex(ex.Current())));
            B.Enlarge(BRep_Tool.Tolerance(TopoDS.Vertex(ex.Current())));
        }
    }

    /// <summary>
    /// Adds the shape <paramref name="S"/> to the box <paramref name="B"/> through its edges' 3D curves and its
    /// vertices outside the edges, without enlarging by tolerances. Quick, but right only for shapes of polygonal
    /// planar faces (not checked).
    /// </summary>
    public static void AddClose(in TopoDS_Shape S, ref Bnd_Box B)
    {
        var ex = new TopExp_Explorer();

        // No faces

        // Add the edges

        var BC = new BRepAdaptor_Curve();

        for (ex.Init(S, TopAbs_EDGE); ex.More(); ex.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(ex.Current());
            if (BRep_Tool.IsGeometric(anEdge))
            {
                BC.Initialize(anEdge);
                BndLib_Add3dCurve.Add(BC, 0.0, ref B);
            }
        }

        // Add the vertices not in edges

        for (ex.Init(S, TopAbs_VERTEX, TopAbs_EDGE); ex.More(); ex.Next())
        {
            B.Add(BRep_Tool.Pnt(TopoDS.Vertex(ex.Current())));
        }
    }

    /// <summary>
    /// Adds the shape <paramref name="S"/> to the box <paramref name="B"/> precisely: as <see cref="Add"/>, with the
    /// optimal boxes of the geometry. With <paramref name="useShapeTolerance"/> the box is enlarged by the shapes'
    /// tolerances, which also drive the numerical methods; otherwise it bounds the geometry, computed with
    /// Precision::Confusion.
    /// </summary>
    public static void AddOptimal(in TopoDS_Shape S,
                                  ref Bnd_Box B,
                                  bool useTriangulation = true,
                                  bool useShapeTolerance = false)
    {
        var ex = new TopExp_Explorer();

        // Add the faces
        var BS = new BRepAdaptor_Surface();
        Poly_Triangulation? T;
        TopLoc_Location l;
        int i, nbNodes;
        var BC = new BRepAdaptor_Curve();

        for (ex.Init(S, TopAbs_FACE); ex.More(); ex.Next())
        {
            TopoDS_Face F = TopoDS.Face(ex.Current());
            T = BRep_Tool.Triangulation(F, out l);
            Bnd_Box aLocBox = new();
            if (useTriangulation && T is not null && T.MinMax(ref aLocBox, l.Transformation()))
            {
                //       B.Enlarge(T->Deflection());
                aLocBox.Enlarge(T.Deflection() + BRep_Tool.Tolerance(F));
                aLocBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
                B.Update(xmin, ymin, zmin, xmax, ymax, zmax);
            }
            else
            {
                Geom_Surface? GS = BRep_Tool.Surface(F, out l);
                if (GS is not null)
                {
                    BS.Initialize(F, false);
                    if (CanUseEdges(BS))
                    {
                        var ex2 = new TopExp_Explorer(F, TopAbs_EDGE);
                        if (!ex2.More())
                        {
                            BS.Initialize(F);
                            double Tol = useShapeTolerance ? BRep_Tool.Tolerance(F) : 0.0;
                            BndLib_AddSurface.AddOptimal(BS, Tol, ref aLocBox);
                        }
                        else
                        {
                            double Tol;
                            for (; ex2.More(); ex2.Next())
                            {
                                Bnd_Box anEBox = new();
                                TopoDS_Edge anE = TopoDS.Edge(ex2.Current());
                                if (BRep_Tool.Degenerated(anE) || !BRep_Tool.IsGeometric(anE))
                                {
                                    continue;
                                }

                                BC.Initialize(anE);
                                Tol = useShapeTolerance ? BRep_Tool.Tolerance(anE) : 0.0;
                                BndLib_Add3dCurve.AddOptimal(BC, Tol, ref anEBox);
                                aLocBox.Add(anEBox);
                            }
                        }
                    }
                    else
                    {
                        double umin, umax, vmin, vmax;
                        bool isNaturalRestriction = false;
                        double Tol = useShapeTolerance ? BRep_Tool.Tolerance(F) : 0.0;
                        FindExactUVBounds(F, out umin, out umax, out vmin, out vmax, Tol, out isNaturalRestriction);
                        BndLib_AddSurface.AddOptimal(BS, umin, umax, vmin, vmax, Tol, ref aLocBox);
                        //
                        if (!isNaturalRestriction)
                        {
                            var ex2 = new TopExp_Explorer(F, TopAbs_EDGE);
                            Bnd_Box EBox = new();
                            for (; ex2.More(); ex2.Next())
                            {
                                Bnd_Box anEBox = new();
                                TopoDS_Edge anE = TopoDS.Edge(ex2.Current());
                                if (BRep_Tool.Degenerated(anE) || !BRep_Tool.IsGeometric(anE))
                                {
                                    continue;
                                }

                                BC.Initialize(anE);
                                Tol = useShapeTolerance ? BRep_Tool.Tolerance(anE) : 0.0;
                                BndLib_Add3dCurve.AddOptimal(BC, Tol, ref anEBox);
                                EBox.Add(anEBox);
                            }

                            Tol = useShapeTolerance ? BRep_Tool.Tolerance(F) : 0.0;
                            AdjustFaceBox(BS, umin, umax, vmin, vmax, ref aLocBox, EBox, Tol);
                        }
                    }

                    if (!aLocBox.IsVoid())
                    {
                        aLocBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
                        B.Update(xmin, ymin, zmin, xmax, ymax, zmax);
                    }
                }
            }
        }

        // Add the edges not in faces
        for (ex.Init(S, TopAbs_EDGE, TopAbs_FACE); ex.More(); ex.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(ex.Current());
            Bnd_Box aLocBox = new();
            Poly_Polygon3D? P3d = BRep_Tool.Polygon3D(E, out l);
            if (useTriangulation && P3d is not null && P3d.NbNodes() > 0)
            {
                NCollection_Array1<gp_Pnt> Nodes = P3d.Nodes();
                nbNodes = P3d.NbNodes();
                for (i = 1; i <= nbNodes; i++)
                {
                    if (l.IsIdentity())
                    {
                        aLocBox.Add(Nodes[i]);
                    }
                    else
                    {
                        aLocBox.Add(Nodes[i].Transformed(l.Transformation()));
                    }
                }

                double Tol = useShapeTolerance ? BRep_Tool.Tolerance(E) : 0.0;
                aLocBox.Enlarge(P3d.Deflection() + Tol);
            }
            else
            {
                BRep_Tool.PolygonOnTriangulation(E, out Poly_PolygonOnTriangulation? Poly, out T, out l);
                if (useTriangulation && Poly is not null && T is not null && T.NbNodes() > 0)
                {
                    NCollection_Array1<int> Indices = Poly.Nodes();
                    nbNodes = Indices.Length();
                    for (i = 1; i <= nbNodes; i++)
                    {
                        if (l.IsIdentity())
                        {
                            aLocBox.Add(T.Node(Indices[i]));
                        }
                        else
                        {
                            aLocBox.Add(T.Node(Indices[i]).Transformed(l.Transformation()));
                        }
                    }

                    double Tol = useShapeTolerance ? BRep_Tool.Tolerance(E) : 0.0;
                    aLocBox.Enlarge(Poly.Deflection() + Tol);
                }
                else
                {
                    if (BRep_Tool.IsGeometric(E))
                    {
                        BC.Initialize(E);
                        double Tol = useShapeTolerance ? BRep_Tool.Tolerance(E) : 0.0;
                        BndLib_Add3dCurve.AddOptimal(BC, Tol, ref aLocBox);
                    }
                }
            }

            if (!aLocBox.IsVoid())
            {
                aLocBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
                B.Update(xmin, ymin, zmin, xmax, ymax, zmax);
            }
        }

        // Add the vertices not in edges

        for (ex.Init(S, TopAbs_VERTEX, TopAbs_EDGE); ex.More(); ex.Next())
        {
            Bnd_Box aLocBox = new();
            TopoDS_Vertex aV = TopoDS.Vertex(ex.Current());
            aLocBox.Add(BRep_Tool.Pnt(aV));
            double Tol = useShapeTolerance ? BRep_Tool.Tolerance(aV) : 0.0;
            aLocBox.Enlarge(Tol);
            aLocBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
            B.Update(xmin, ymin, zmin, xmax, ymax, zmax);
        }
    }

    // Defines whether the box of the face can be found from the boxes of its edges only
    private static bool CanUseEdges(Adaptor3d_Surface BS)
    {
        GeomAbs_SurfaceType aST = BS.GetType();
        if (aST == GeomAbs_Plane || aST == GeomAbs_Cylinder || aST == GeomAbs_Cone
            || aST == GeomAbs_SurfaceOfExtrusion)
        {
            return true;
        }
        else if (aST == GeomAbs_SurfaceOfRevolution)
        {
            Adaptor3d_Curve aBC = BS.BasisCurve();
            return aBC.GetType() == GeomAbs_Line;
        }
        else if (aST == GeomAbs_OffsetSurface)
        {
            Adaptor3d_Surface aS = BS.BasisSurface();
            return CanUseEdges(aS);
        }
        else if (aST == GeomAbs_BSplineSurface)
        {
            Geom_BSplineSurface aBSpl = BS.BSpline();
            return (aBSpl.UDegree() == 1 && aBSpl.NbUKnots() == 2)
                   || (aBSpl.VDegree() == 1 && aBSpl.NbVKnots() == 2);
        }
        else if (aST == GeomAbs_BezierSurface)
        {
            Geom_BezierSurface aBz = BS.Bezier();
            return (aBz.UDegree() == 1) || (aBz.VDegree() == 1);
        }

        return false;
    }

    private static void FindExactUVBounds(in TopoDS_Face FF,
                                          out double umin,
                                          out double umax,
                                          out double vmin,
                                          out double vmax,
                                          double Tol,
                                          out bool isNaturalRestriction)
    {
        // OCCT leaves umin..vmax unset on a face that isn't a natural restriction and has no pcurve; the port
        // starts them at 0 (decision 32)
        umin = umax = vmin = vmax = 0.0;
        TopoDS_Face F = FF;
        F.Orientation(TopAbs_FORWARD);
        var ex = new TopExp_Explorer(F, TopAbs_EDGE);
        //
        // Check Natural restriction
        isNaturalRestriction = BRep_Tool.NaturalRestriction(F); // Can we trust this flag?
        var aBAS = new BRepAdaptor_Surface(F, false);
        if (!isNaturalRestriction)
        {
            // Check by comparing pcurves and surface boundaries
            umin = aBAS.FirstUParameter();
            umax = aBAS.LastUParameter();
            vmin = aBAS.FirstVParameter();
            vmax = aBAS.LastVParameter();
            bool isUperiodic = aBAS.IsUPeriodic(), isVperiodic = aBAS.IsVPeriodic();
            double aT1, aT2;
            double TolU = Max(aBAS.UResolution(Tol), Precision.PConfusion());
            double TolV = Max(aBAS.VResolution(Tol), Precision.PConfusion());
            int Nu = 0, Nv = 0, NbEdges = 0;
            gp_Vec2d Du = new(1, 0), Dv = new(0, 1);
            for (; ex.More(); ex.Next())
            {
                NbEdges++;
                if (NbEdges > 4)
                {
                    break;
                }

                TopoDS_Edge aE = TopoDS.Edge(ex.Current());
                Geom2d_Curve? aC2D = BRep_Tool.CurveOnSurface(aE, F, out aT1, out aT2);
                if (aC2D is null)
                {
                    break;
                }

                //
                aC2D.D1((aT1 + aT2) / 2.0, out gp_Pnt2d aP, out gp_Vec2d aV);
                double magn = aV.SquareMagnitude();
                if (magn < gp.Resolution())
                {
                    break;
                }
                else
                {
                    aV /= Math.Sqrt(magn);
                }

                double u = aP.X(), v = aP.Y();
                if (isUperiodic)
                {
                    // OCCT discards the result of ElCLib::InPeriod (it returns the value), so u is unchanged
                    _ = ElCLib.InPeriod(u, umin, umax);
                }

                if (isVperiodic)
                {
                    // OCCT discards the result of ElCLib::InPeriod (it returns the value), so v is unchanged
                    _ = ElCLib.InPeriod(v, vmin, vmax);
                }

                //
                if (Math.Abs(u - umin) <= TolU || Math.Abs(u - umax) <= TolU)
                {
                    double d = Dv * aV;
                    if (1.0 - Math.Abs(d) <= Precision.PConfusion())
                    {
                        Nu++;
                        if (Nu > 2)
                        {
                            break;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                else if (Math.Abs(v - vmin) <= TolV || Math.Abs(v - vmax) <= TolV)
                {
                    double d = Du * aV;
                    if (1.0 - Math.Abs(d) <= Precision.PConfusion())
                    {
                        Nv++;
                        if (Nv > 2)
                        {
                            break;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    break;
                }
            }

            if (Nu == 2 && Nv == 2)
            {
                isNaturalRestriction = true;
            }
        }

        //
        if (isNaturalRestriction)
        {
            umin = aBAS.FirstUParameter();
            umax = aBAS.LastUParameter();
            vmin = aBAS.FirstVParameter();
            vmax = aBAS.LastVParameter();
            return;
        }

        // fill box for the given face
        {
            double aT1, aT2;
            double TolU = Max(aBAS.UResolution(Tol), Precision.PConfusion());
            double TolV = Max(aBAS.VResolution(Tol), Precision.PConfusion());
            double TolUV = Max(TolU, TolV);
            Bnd_Box2d aBox = new();
            ex.Init(F, TopAbs_EDGE);
            for (; ex.More(); ex.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(ex.Current());
                Geom2d_Curve? aC2D = BRep_Tool.CurveOnSurface(aE, F, out aT1, out aT2);
                if (aC2D is null)
                {
                    continue;
                }

                //
                BndLib_Add2dCurve.AddOptimal(aC2D, aT1, aT2, TolUV, ref aBox);
                //
            }

            // In some cases no edges are found
            if (!aBox.IsVoid())
            {
                aBox.Get(out umin, out vmin, out umax, out vmax);
            }
        }

        //
        Geom_Surface aS = BRep_Tool.Surface(FF, out TopLoc_Location _)!;
        aS.Bounds(out double aUmin, out double aUmax, out double aVmin, out double aVmax);
        if (!aS.IsUPeriodic())
        {
            umin = Max(aUmin, umin);
            umax = Min(aUmax, umax);
        }
        else
        {
            if (umax - umin > aS.UPeriod())
            {
                double delta = umax - umin - aS.UPeriod();
                umin += delta / 2.0;
                umax -= delta / 2;
            }
        }

        //
        if (!aS.IsVPeriodic())
        {
            vmin = Max(aVmin, vmin);
            vmax = Min(aVmax, vmax);
        }
        else
        {
            if (vmax - vmin > aS.VPeriod())
            {
                double delta = vmax - vmin - aS.VPeriod();
                vmin += delta / 2.0;
                vmax -= delta / 2;
            }
        }
    }

    private static void Reorder(ref double a, ref double b)
    {
        if (a > b)
        {
            double t = a;
            a = b;
            b = t;
        }
    }

    private static bool IsModifySize(BRepAdaptor_Surface theBS,
                                     in gp_Pln thePln,
                                     in gp_Pnt theP,
                                     double umin,
                                     double umax,
                                     double vmin,
                                     double vmax,
                                     BRepTopAdaptor_FClass2d theFClass,
                                     double theTolU,
                                     double theTolV)
    {
        double pu1 = 0, pu2, pv1 = 0, pv2;
        ElSLib.PlaneParameters(thePln.Position(), theP, out pu2, out pv2);
        Reorder(ref pu1, ref pu2);
        Reorder(ref pv1, ref pv2);
        var aPlane = new Geom_Plane(thePln);
        var aGAPln = new GeomAdaptor_Surface(aPlane, pu1, pu2, pv1, pv2);
        var anExtr = new Extrema_ExtSS(aGAPln, theBS, pu1, pu2, pv1, pv2, umin, umax, vmin, vmax, theTolU, theTolV);
        if (anExtr.IsDone())
        {
            if (anExtr.NbExt() > 0)
            {
                int i, imin = 0;
                double dmin = RealLast();
                double uextr = 0.0, vextr = 0.0;
                for (i = 1; i <= anExtr.NbExt(); ++i)
                {
                    double d = anExtr.SquareDistance(i);
                    if (d < dmin)
                    {
                        imin = i;
                        dmin = d;
                    }
                }

                if (imin > 0)
                {
                    anExtr.Points(imin, out Extrema_POnSurf _, out Extrema_POnSurf P2);
                    P2.Parameter(out uextr, out vextr);
                }
                else
                {
                    return false;
                }

                //
                gp_Pnt2d aP2d = new(uextr, vextr);
                TopAbs_State aSt = theFClass.Perform(aP2d);
                if (aSt != TopAbs_IN)
                {
                    return true;
                }
            }
            else
            {
                return true; // extrema point seems to be out of face UV bounds
            }
        }

        //
        return false;
    }

    private static void AdjustFaceBox(BRepAdaptor_Surface BS,
                                      double umin,
                                      double umax,
                                      double vmin,
                                      double vmax,
                                      ref Bnd_Box FaceBox,
                                      in Bnd_Box EdgeBox,
                                      double Tol)
    {
        if (EdgeBox.IsVoid())
        {
            return;
        }

        if (FaceBox.IsVoid())
        {
            FaceBox = EdgeBox;
            return;
        }

        FaceBox.Get(out double fxmin, out double fymin, out double fzmin, out double fxmax, out double fymax, out double fzmax);
        EdgeBox.Get(out double exmin, out double eymin, out double ezmin, out double exmax, out double eymax, out double ezmax);
        //
        double TolU = Max(BS.UResolution(Tol), Precision.PConfusion());
        double TolV = Max(BS.VResolution(Tol), Precision.PConfusion());
        var FClass = new BRepTopAdaptor_FClass2d(BS.Face(), Max(TolU, TolV));
        //
        bool isModified = false;
        if (exmin > fxmin)
        {
            //
            var pl = new gp_Pln(new gp_Ax3(new gp_Pnt(fxmin, fymin, fzmin), gp.DX()));
            gp_Pnt aP = new(fxmin, fymax, fzmax);
            if (IsModifySize(BS, pl, aP, umin, umax, vmin, vmax, FClass, TolU, TolV))
            {
                fxmin = exmin;
                isModified = true;
            }
        }

        if (exmax < fxmax)
        {
            //
            var pl = new gp_Pln(new gp_Ax3(new gp_Pnt(fxmax, fymax, fzmax), gp.DX()));
            gp_Pnt aP = new(fxmax, fymin, fzmin);
            if (IsModifySize(BS, pl, aP, umin, umax, vmin, vmax, FClass, TolU, TolV))
            {
                fxmax = exmax;
                isModified = true;
            }
        }

        //
        if (eymin > fymin)
        {
            //
            var pl = new gp_Pln(new gp_Ax3(new gp_Pnt(fxmin, fymin, fzmin), gp.DY()));
            gp_Pnt aP = new(fxmax, fymin, fzmax);
            if (IsModifySize(BS, pl, aP, umin, umax, vmin, vmax, FClass, TolU, TolV))
            {
                fymin = eymin;
                isModified = true;
            }
        }

        if (eymax < fymax)
        {
            //
            var pl = new gp_Pln(new gp_Ax3(new gp_Pnt(fxmax, fymax, fzmax), gp.DY()));
            gp_Pnt aP = new(fxmin, fymax, fzmin);
            if (IsModifySize(BS, pl, aP, umin, umax, vmin, vmax, FClass, TolU, TolV))
            {
                fymax = eymax;
                isModified = true;
            }
        }

        //
        if (ezmin > fzmin)
        {
            //
            var pl = new gp_Pln(new gp_Ax3(new gp_Pnt(fxmin, fymin, fzmin), gp.DZ()));
            gp_Pnt aP = new(fxmax, fymax, fzmin);
            if (IsModifySize(BS, pl, aP, umin, umax, vmin, vmax, FClass, TolU, TolV))
            {
                fzmin = ezmin;
                isModified = true;
            }
        }

        if (ezmax < fzmax)
        {
            //
            var pl = new gp_Pln(new gp_Ax3(new gp_Pnt(fxmax, fymax, fzmax), gp.DZ()));
            gp_Pnt aP = new(fxmin, fymin, fzmax);
            if (IsModifySize(BS, pl, aP, umin, umax, vmin, vmax, FClass, TolU, TolV))
            {
                fzmax = ezmax;
                isModified = true;
            }
        }

        //
        if (isModified)
        {
            FaceBox.SetVoid();
            FaceBox.Update(fxmin, fymin, fzmin, fxmax, fymax, fzmax);
        }
    }

    // Functional forms (not in OCCT)

    /// <summary>The box of the shape: a new box given to <see cref="Add"/>.</summary>
    public static Bnd_Box Box(in TopoDS_Shape S, bool useTriangulation = true)
    {
        Bnd_Box aBox = new();
        Add(S, ref aBox, useTriangulation);
        return aBox;
    }

    /// <summary>The close box of the shape: a new box given to <see cref="AddClose"/>.</summary>
    public static Bnd_Box BoxClose(in TopoDS_Shape S)
    {
        Bnd_Box aBox = new();
        AddClose(S, ref aBox);
        return aBox;
    }

    /// <summary>The optimal box of the shape: a new box given to <see cref="AddOptimal"/>.</summary>
    public static Bnd_Box BoxOptimal(in TopoDS_Shape S, bool useTriangulation = true, bool useShapeTolerance = false)
    {
        Bnd_Box aBox = new();
        AddOptimal(S, ref aBox, useTriangulation, useShapeTolerance);
        return aBox;
    }
}
