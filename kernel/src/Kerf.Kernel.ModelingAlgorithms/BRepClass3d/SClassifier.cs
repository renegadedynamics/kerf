// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SClassifier.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Numerics;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Classification;

/// <summary>Classifies a point in a solid explored by a <see cref="SolidExplorer"/>.</summary>
/// <remarks>A working object (decision 21). It only reads the shape.</remarks>
internal class SClassifier
{
    private Face _face = new();

    //! This variable stores information about algorithm internal state.
    //! Type of this variable differs from TopAbs_State since it contains
    //! additional information about error status.
    //! 1 - Error inside of the algorithm.
    //! 2 - ON.
    //! 3 - IN.
    //! 4 - OUT.
    private int _state;

    /// <summary>An empty classifier.</summary>
    public SClassifier()
    {
        _state = 0;
    }

    /// <summary>Classifies the point <paramref name="p"/> with the tolerance <paramref name="tol"/> in the solid of <paramref name="s"/>.</summary>
    public SClassifier(SolidExplorer s, in Point3D p, double tol)
    {
        if (s.Reject(p))
        {
            _state = 3; //-- in ds solid case without face
        }
        else
        {
            Perform(s, p, tol);
        }
    }

    /// <summary>
    /// Classifies a point at infinity in the solid of <paramref name="aSE"/> (OUT for a finite solid, IN for a hole in
    /// space): the reversed normal at random points of the faces is intersected with the solid, until the nearest
    /// intersection is inside a face and not tangent. The tolerance is unused.
    /// </summary>
    public void PerformInfinitePoint(SolidExplorer aSE, double tol)
    {
        // Take a normal to the first extracted face in its random inner point
        // and intersect this reversed normal with the faces of the solid.
        // If the min.par.-intersection point is
        //  a) inner point of a face
        //  b) transition is not TANGENT
        //     (the line does not touch the face but pierces it)
        // then set <myState> to IN or OUT according to transition
        // else take the next random point inside the min.par.-intersected face
        // and continue

        if (aSE.Reject(new Point3D(0, 0, 0)))
        {
            _state = 3; //-- in ds solid case without face
            return;
        }

        //
        //------------------------------------------------------------
        // 1
        bool bFound;
        double param, u = 0.0, v = 0.0;
        Point3D point = new();
        Direction3D dn = new();

        var randomGenerator = new BullardGenerator();
        _face.Nullify();
        _state = 2;

        // Collect faces in sequence to iterate
        var faces = new List<Face>();
        for (aSE.InitShell(); aSE.MoreShell(); aSE.NextShell())
        {
            for (aSE.InitFace(); aSE.MoreFace(); aSE.NextFace())
            {
                faces.Add(aSE.CurrentFace());
            }
        }

        // iteratively try up to 10 probing points from each face
        const int NB_MAX_POINTS_PER_FACE = 10;
        for (int itry = 0; itry < NB_MAX_POINTS_PER_FACE; itry++)
        {
            for (int iFace = 0; iFace < faces.Count; ++iFace)
            {
                Face f = faces[iFace];

                Containment state = global::Kerf.Topology.Containment.Out;
                TransitionOnCurve transition = TransitionOnCurve.Tangent;

                param = 0.1 + 0.8 * randomGenerator.NextReal(); // random number in range [0.1, 0.9]
                bFound = SolidExplorer.FindAPointInTheFace(f, ref point, ref u, ref v, ref param);
                if (!bFound || !FaceNormal(f, u, v, ref dn))
                {
                    continue;
                }

                var lin = new Line3D(point, -dn);
                double parmin = RealLast();
                for (aSE.InitShell(); aSE.MoreShell(); aSE.NextShell())
                {
                    if (!aSE.RejectShell(lin))
                    {
                        for (aSE.InitFace(); aSE.MoreFace(); aSE.NextFace())
                        {
                            if (!aSE.RejectFace(lin))
                            {
                                Shape localShape = aSE.CurrentFace();
                                Face curFace = ShapeCasts.Face(localShape);
                                CurveSurfaceIntersector intersector3d = aSE.Intersector(curFace);
                                intersector3d.Perform(lin, -RealLast(), parmin);

                                if (intersector3d.IsDone)
                                {
                                    if (intersector3d.PntCount != 0)
                                    {
                                        int imin = 1;
                                        for (int i = 2; i <= intersector3d.PntCount; i++)
                                        {
                                            if (intersector3d.WParameter(i) < intersector3d.WParameter(imin))
                                            {
                                                imin = i;
                                            }
                                        }

                                        parmin = intersector3d.WParameter(imin);
                                        state = intersector3d.State(imin);
                                        transition = intersector3d.Transition(imin);
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        _state = 1;
                    }
                } // end of loop on the whole solid

                if (state == global::Kerf.Topology.Containment.In)
                {
                    if (transition == TransitionOnCurve.Out)
                    {
                        //-- The line is going from inside the solid to outside
                        //-- the solid.
                        _state = 3; //-- IN --
                        return;
                    }
                    else if (transition == TransitionOnCurve.In)
                    {
                        _state = 4; //-- OUT --
                        return;
                    }
                }
            } // iteration by faces
        } // iteration by points
    }

    /// <summary>Classifies the point <paramref name="p"/> with the tolerance <paramref name="tol"/> in the solid of <paramref name="solidExplorer"/>.</summary>
    public void Perform(SolidExplorer solidExplorer, in Point3D p, double tol)
    {
        // Kerf (unit 15E17, decision 2452): a shape placed with a scale is classified in its own frame, the point taken
        // there by the placement's inverse and the tolerance with it; the deciding face is placed back
        SolidExplorer? unscaled = solidExplorer.Unscaled(out Placement placement);
        if (unscaled is not null && !global::Kerf.Engine.Healing.Fixes.UndoneFixes.AreUndone)
        {
            Transform3D trsf = placement.Transformation;
            Perform(unscaled, trsf.Inverted() * p, tol / Math.Abs(trsf.ScaleFactor));
            if (!_face.IsNull)
            {
                _face.Move(placement);
            }

            return;
        }

        PerformByRay(solidExplorer, p, tol);

        // Kerf (unit 15E17, decision 2447): a point outside a finite solid's box is outside it, whatever the ray found. The
        // ray reads IN when the face intersectors miss the crossing where it enters (OCCT #1700, #1703), and that put a point
        // outside the box IN, so a distance between disjoint solids was 0, an inner solution and a success.
        if (_state == 3 && solidExplorer.IsOutsideFiniteBounds(p, tol))
        {
            _state = 4;
            _face = new Face();
        }
    }

    private void PerformByRay(SolidExplorer solidExplorer, in Point3D p, double tol)
    {
        // Kerf (unit 15C2): what the last point left in the explorer, its choice of face, where on an edge to aim, the faces'
        // intersectors it made unrestricted and each intersector's flag of a line found parallel, set back to a new explorer's,
        // so every point is classified as by a new classifier. OCCT carries them over, and a stale flag can call a point ON
        // (occt-issues.md, "Found by unit 15C").
        solidExplorer.ForANewPoint();
        if (solidExplorer.Reject(p))
        {
            // Solid without faces => the whole space. Always in.
            _state = 3; // IN
            return;
        }

        SpatialTree<int, Box> tree = solidExplorer.GetTree();
        IndexedSet<Shape> mapEV = solidExplorer.GetMapEV();

        // Vertices/Edges vs Point.
        var selectorPoint = new BndBoxTreeSelectorPoint(mapEV);
        selectorPoint.SetCurrentPoint(p);

        int selsVE = 0;
        selsVE = tree.Select(selectorPoint);

        if (selsVE > 0)
        {
            // The point P lays inside the tolerance area of vertices/edges => return ON state.
            _state = 2; // ON.
            return;
        }

        // Kerf: kept by the explorer for a frozen shape (unit 12Y)
        IndexedMap<Shape, NCollection_List<Shape>> mapEF = solidExplorer.GetMapEF();

        var selectorLine = new BndBoxTreeSelectorLine(mapEV);

        _face.Nullify();
        _state = 0;

        Line3D l = new();
        double par = 0.0;
        // We compute the intersection between the line built in the Solid Explorer and the shape.
        //-- --------------------------------------------------------------------------------
        // Calculate intersection with the face closest to the direction of bounding boxes
        // by priority so that to have the smallest possible parmin.
        // Optimization to produce as much as possible rejections with other faces.
        int iFlag;

        // If found line passes through a bound of any face, it means that the line
        // is not found properly and it is necessary to repeat whole procedure.
        // That's why the main loop while is added.
        bool isFaultyLine = true;
        int indFace = 0;
        double parmin = 0.0;
        while (isFaultyLine)
        {
            if (indFace == 0)
            {
                iFlag = solidExplorer.Segment(p, ref l, ref par);
            }
            else
            {
                iFlag = solidExplorer.OtherSegment(p, ref l, ref par);
            }

            int curInd = solidExplorer.GetFaceSegmentIndex();

            if (curInd > indFace)
            {
                indFace = curInd;
            }
            else
            {
                _state = 1; // Faulty.
                return;
            }

            if (iFlag == 1)
            {
                // IsOnFace iFlag==1 i.e face is Infinite
                _state = 2; // ON.
                return;
            }

            if (iFlag == 2)
            {
                _state = 4; // OUT.
                return;
            }

            // Check if the point is ON surface but OUT of the face.
            // Just skip this face because it is bad for classification.
            if (iFlag == 3)
            {
                continue;
            }

            isFaultyLine = false;
            parmin = RealLast();

            double nearFaultPar = RealLast(); // Parameter on line.
            selectorLine.ClearResults();
            selectorLine.SetCurrentLine(l, par);
            int selsEVL = 0;
            selsEVL = tree.Select(selectorLine); // SelsEE > 0 => Line/Edges & Line/Vertex intersection
            if (selsEVL > 0)
            {
                // Line and edges / vertices interference.
                int vlInterNb = selectorLine.GetNbVertParam();
                var lvInts = new HashSet<Shape>(ShapeHasher.Instance);
                for (int i = 1; i <= vlInterNb; i++)
                {
                    // Line and vertex.
                    selectorLine.GetVertParam(i, out Vertex V, out double lp);

                    lvInts.Add(V);
                    if (Math.Abs(lp) < Math.Abs(nearFaultPar))
                    {
                        nearFaultPar = lp;
                    }
                }

                double param = 0.0;
                Edge ee;
                double lpar = RealLast();
                for (int i = 1; i <= selectorLine.GetNbEdgeParam(); i++)
                {
                    // Line and edge.
                    selectorLine.GetEdgeParam(i, out ee, out param, out lpar);
                    NCollection_List<Shape> ffs = mapEF.GetValue(ee); // ffs size == 2
                    if (ffs.Extent() != 2)
                    {
                        continue;
                    }

                    Face f1 = ShapeCasts.Face(ffs.First());
                    Face f2 = ShapeCasts.Face(ffs.Last());
                    Exploration.Vertices(ee, out Vertex v1, out Vertex v2);
                    if (lvInts.Contains(v1) || lvInts.Contains(v2))
                    {
                        continue;
                    }

                    TransitionOnCurve tran = TransitionOnCurve.Tangent;
                    int tst = GetTransi(f1, f2, ee, param, l, ref tran);
                    if (tst == 1 && Math.Abs(lpar) < Math.Abs(parmin))
                    {
                        parmin = lpar;
                        Trans(parmin, ref tran, ref _state);
                    }
                    else if (Math.Abs(lpar) < Math.Abs(nearFaultPar))
                    {
                        nearFaultPar = lpar;
                    }
                }
            }

            for (solidExplorer.InitShell(); solidExplorer.MoreShell() && !isFaultyLine;
                 solidExplorer.NextShell())
            {
                if (!solidExplorer.RejectShell(l))
                {
                    for (solidExplorer.InitFace(); solidExplorer.MoreFace() && !isFaultyLine;
                         solidExplorer.NextFace())
                    {
                        if (!solidExplorer.RejectFace(l))
                        {
                            Shape localShape = solidExplorer.CurrentFace();
                            Face f = ShapeCasts.Face(localShape);
                            CurveSurfaceIntersector intersector3d = solidExplorer.Intersector(f);

                            // Prolong segment, since there are cases when
                            // the intersector does not find intersection points with the original
                            // segment due to rough triangulation of a parameterized surface.
                            double addW = Max(10 * tol, 0.01 * par);
                            double AddW = addW;

                            // Kerf (unit 15D): not the polyhedron's box, building the polyhedron, where the line is surely clear of
                            // it: Perform then finds nothing over any part of the line, so how far it's prolonged doesn't matter
                            Box boxF = intersector3d.SurelyMissesPolyhedron(l) ? new Box() : intersector3d.Bounding();

                            // The box must be finite in order to correctly prolong the segment to its bounds.
                            if (!boxF.IsVoid && !boxF.IsWhole)
                            {
                                boxF.Get(out double _, out double _, out double _, out double _, out double _, out double _);

                                double boxaddW = GetAddToParam(l, par, boxF);
                                addW = Max(addW, boxaddW);
                            }

                            double minW = -AddW;
                            double maxW = Min(par * 10, par + addW);
                            intersector3d.Perform(l, minW, maxW);
                            if (intersector3d.IsDone)
                            {
                                if (intersector3d.PntCount == 0)
                                {
                                    if (intersector3d.IsParallel)
                                    {
                                        // Check distance between surface and point
                                        var bas = new FaceSurfaceAdaptor(f, false);
                                        var proj = new ExtPS(p,
                                                                      bas,
                                                                      global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                                                      global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                                                      ExtFlag_MIN);
                                        if (proj.IsDone && proj.ExtCount > 0)
                                        {
                                            int i, indmin = 0;
                                            double d = RealLast();
                                            for (i = 1; i <= proj.ExtCount; ++i)
                                            {
                                                if (proj.SquareDistance(i) < d)
                                                {
                                                    d = proj.SquareDistance(i);
                                                    indmin = i;
                                                }
                                            }

                                            if (indmin > 0)
                                            {
                                                if (d <= tol * tol)
                                                {
                                                    POnSurf ponS = proj.Point(indmin);
                                                    ponS.Parameter(out double u, out double v);
                                                    Point2D p2d = new(u, v);
                                                    Containment st = intersector3d.ClassifyUVPoint(p2d);
                                                    if (st == global::Kerf.Topology.Containment.In || st == On)
                                                    {
                                                        _state = 2;
                                                        _face = f;
                                                        parmin = 0.0;
                                                        break;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }

                                for (int i = 1; i <= intersector3d.PntCount; i++)
                                {
                                    if (Math.Abs(intersector3d.WParameter(i))
                                        < Math.Abs(parmin) - global::Kerf.Engine.Foundation.Precision.PConfusion())
                                    {
                                        parmin = intersector3d.WParameter(i);
                                        Containment state = intersector3d.State(i);
                                        if (Math.Abs(parmin) <= tol)
                                        {
                                            _state = 2;
                                            _face = f;
                                            break;
                                        }
                                        //  Treatment of case TopAbs_ON separately.
                                        else if (state == global::Kerf.Topology.Containment.In)
                                        {
                                            //-- The intersection point between the line and a face F
                                            // -- of the solid is in the face F

                                            TransitionOnCurve tran = intersector3d.Transition(i);
                                            if (tran == TransitionOnCurve.Tangent)
                                            {
                                                continue; // ignore this point
                                            }

                                            Trans(parmin, ref tran, ref _state);
                                            _face = f;
                                        }
                                        // If the state is TopAbs_ON, it is necessary to chose
                                        // another line and to repeat the whole procedure.
                                        else if (state == On)
                                        {
                                            isFaultyLine = true;
                                            break;
                                        }
                                    }
                                    else
                                    {
                                        //-- No point has been found by the Intersector3d.
                                        //-- Or a Point has been found with a greater parameter.
                                    }
                                } //-- loop by intersection points

                                if (_state == 2)
                                {
                                    break;
                                }
                            } //-- Face has not been rejected
                            else
                            {
                                _state = 1;
                            }
                        }
                    } //-- Exploration of the faces

                    if (_state == 2)
                    {
                        break;
                    }
                } //-- Shell has not been rejected
                else
                {
                    _state = 1;
                }
            } //-- Exploration of the shells

            if (nearFaultPar != RealLast()
                && Math.Abs(parmin) >= Math.Abs(nearFaultPar) - global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                isFaultyLine = true;
            }
        }
    }

    /// <summary>The result of the classification (OUT also when the algorithm failed).</summary>
    public Containment State()
    {
        if (_state == 2)
        {
            return (On);
        }
        else if (_state == 3)
        {
            return (global::Kerf.Topology.Containment.In);
        }
        else if (_state == 4)
        {
            return (global::Kerf.Topology.Containment.Out);
        }

        // return OUT state when there is an error during execution.
        return (global::Kerf.Topology.Containment.Out);
    }

    /// <summary>The face that decided the classification; when the state is ON, the face holding the point.</summary>
    public Face Face() => (_face);

    /// <summary>True if the classification failed (OCCT's name: the state is then OUT).</summary>
    public bool Rejected() => (_state == 1);

    /// <summary>True when the point is on a face.</summary>
    public bool IsOnAFace => (_state == 2);

    /// <summary>Sets the state to IN.</summary>
    protected void ForceIn() => _state = 3;

    /// <summary>Sets the state to OUT.</summary>
    protected void ForceOut() => _state = 4;

    private static double GetAddToParam(in Line3D l, double p, in Box b)
    {
        b.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
        Span<double> x = [xmin, xmax];
        Span<double> y = [ymin, ymax];
        Span<double> z = [zmin, zmax];
        int i = 0, j = 0, k = 0;
        double Par = p;
        for (i = 0; i < 2; i++)
        {
            for (j = 0; j < 2; j++)
            {
                for (k = 0; k < 2; k++)
                {
                    double X = Math.Abs(x[i] - l.Location.X);
                    double Y = Math.Abs(y[j] - l.Location.Y);
                    double Z = Math.Abs(z[k] - l.Location.Z);
                    if (X < 1.0e+20 && Y < 1.0e+20 && Z < 1.0e+20)
                    {
                        Point3D aP = new(x[i], y[j], z[k]);
                        double par = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, aP);
                        if (par > Par)
                        {
                            Par = par;
                        }
                    }
                    else
                    {
                        return 1.0e+20;
                    }
                }
            }
        }

        return Par - p;
    }

    // modified by NIZHNY-MKK  Mon Jun 21 15:13:40 2004
    private static bool FaceNormal(in Face aF, double u, double v, ref Direction3D aDN)
    {
        Surface s = Tool.Surface(aF)!;
        s.D1(u, v, out Point3D _, out Vector3D d1U, out Vector3D d1V);
        Vector3D n = d1U.Crossed(d1V);
        if (n.Magnitude <= Constants.Resolution())
        {
            return false;
        }

        n = n.Normalized();
        aDN = aDN.WithXYZ(n.XYZ());
        if (aF.Orientation == Reversed)
        {
            aDN = aDN.Reversed();
        }

        return true;
    }

    private static bool GetNormalOnFaceBound(in Edge e, in Face F, double param, ref Direction3D outDir)
    {
        Curve2D? c2d = Tool.CurveOnSurface(e, F, out double f, out double l);
        if (c2d is null)
        {
            return false;
        }

        if (param < f || param > l)
        {
            return false;
        }

        c2d.D0(param, out Point2D p2d);
        return FaceNormal(F, p2d.X, p2d.Y, ref outDir);
    }

    // gets transition of line <L> passing through/near the edge <e> of faces <f1>, <f2>. <param> is
    //  a parameter on the edge where the minimum distance between <l> and <e> was found
    private static int GetTransi(in Face f1,
                                 in Face f2,
                                 in Edge e,
                                 double param,
                                 in Line3D l,
                                 ref TransitionOnCurve trans)
    {
        // return statuses:
        // 1 => OK
        // 0 => skip
        //-1 => probably a faulty line
        Direction3D nf1 = new(), nf2 = new();
        if (!GetNormalOnFaceBound(e, f1, param, ref nf1))
        {
            return -1;
        }

        if (!GetNormalOnFaceBound(e, f2, param, ref nf2))
        {
            return -1;
        }

        Direction3D lDir = l.Direction;

        if (Math.Abs(lDir.Dot(nf1)) < global::Kerf.Engine.Foundation.Precision.Angular()
            || Math.Abs(lDir.Dot(nf2)) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            // line is orthogonal to normal(s)
            // trans = IntCurveSurface_Tangent;
            return -1;
        }

        if (nf1.IsParallel(nf2, global::Kerf.Engine.Foundation.Precision.Angular()))
        {
            double angD = nf1.Dot(lDir);
            if (Math.Abs(angD) < global::Kerf.Engine.Foundation.Precision.Angular())
            {
                return -1;
            }
            else if (angD > 0)
            {
                trans = TransitionOnCurve.Out;
            }
            else
            {
                // angD < -Precision::Angular())
                trans = TransitionOnCurve.In;
            }

            return 1;
        }

        Vector3D n = new(nf1 ^ nf2);

        //proj LDir on the plane defined by nf1/nf2 directions
        Direction3D projL = new((n.XYZ() ^ lDir.XYZ()) ^ n.XYZ());

        double fAD = nf1.Dot(projL);
        double sAD = nf2.Dot(projL);

        if (fAD < -global::Kerf.Engine.Foundation.Precision.Angular() && sAD < -global::Kerf.Engine.Foundation.Precision.Angular())
        {
            trans = TransitionOnCurve.In;
        }
        else if (fAD > global::Kerf.Engine.Foundation.Precision.Angular() && sAD > global::Kerf.Engine.Foundation.Precision.Angular())
        {
            trans = TransitionOnCurve.Out;
        }
        else
        {
            return 0;
        }

        return 1;
    }

    private static void Trans(double parmin, ref TransitionOnCurve tran, ref int state)
    {
        // if parmin is negative we should reverse transition
        if (parmin < 0)
        {
            tran = (tran == TransitionOnCurve.Out ? TransitionOnCurve.In : TransitionOnCurve.Out);
        }

        if (tran == TransitionOnCurve.Out)
        {
            //-- The line is going from inside the solid to outside
            //-- the solid.
            state = 3; // IN
        }
        else
        {
            state = 4; // OUT
        }
    }
}
