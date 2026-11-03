// Port of: src/ModelingAlgorithms/TKTopAlgo/IntCurvesFace/IntCurvesFace_Intersector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/IntCurvesFace/IntCurvesFace_Intersector.lxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/IntCurvesFace/IntCurvesFace_Intersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Occt.IntCurveSurface_TransitionOnCurve;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The intersection of a line or a curve with a face: the points of the curve on the face's surface that are IN or
/// ON the face, sorted along the curve.
/// </summary>
/// <remarks>
/// A working object (decision 21): load a face once, then intersect any number of curves with
/// <see cref="Perform(in Line3D, double, double)"/>. On surfaces other than the elementary ones it keeps a polyhedron
/// of the surface, and the sorted boxes of its triangles, for all the curves. It only reads the face.
/// </remarks>
public class IntCurvesFace_Intersector : Transient
{
    private readonly BRepTopAdaptor_TopolTool myTopolTool;
    private readonly BRepAdaptor_Surface Hsurface;
    private readonly double Tol;
    private readonly NCollection_Sequence<IntCurveSurface_IntersectionPoint> SeqPnt = new();
    private readonly NCollection_Sequence<int> mySeqState = new();
    private bool done;
    private readonly bool myReady;
    private int nbpnt;
    private readonly TopoDS_Face face;
    private readonly IntCurveSurface_ThePolyhedronOfHInter? myPolyhedron;
    private BoundSortBox? myBndBounding;
    private bool myUseBoundTol;
    private bool myIsParallel; // Curve is "parallel" face surface
                               // This case is recognized only for some pairs
                               // of analytical curves and surfaces (plane - line, ...)

    private static void ComputeSamplePars(Adaptor3d_Surface Hsurface,
                                          int nbsu,
                                          int nbsv,
                                          out NCollection_HArray1<double> UPars,
                                          out NCollection_HArray1<double> VPars)
    {
        int NbUInts = Hsurface.NbUIntervals(C2);
        int NbVInts = Hsurface.NbVIntervals(C2);
        var UInts = new NCollection_Array1<double>(1, NbUInts + 1);
        var VInts = new NCollection_Array1<double>(1, NbVInts + 1);
        Hsurface.UIntervals(UInts, C2);
        Hsurface.VIntervals(VInts, C2);
        //
        var NbUSubInts = new NCollection_Array1<int>(1, NbUInts);
        var NbVSubInts = new NCollection_Array1<int>(1, NbVInts);
        //
        int i, j, ind, NbU, NbV;
        double t, dt;
        t = UInts[NbUInts + 1] - UInts[1];
        t = 1.0 / t;
        NbU = 0;
        for (i = 1; i <= NbUInts; ++i)
        {
            dt = (UInts[i + 1] - UInts[i]);
            NbUSubInts[i] = RealToInt(nbsu * dt * t) + 1;
            NbU += NbUSubInts[i];
        }

        t = VInts[NbVInts + 1] - VInts[1];
        t = 1.0 / t;
        NbV = 0;
        for (i = 1; i <= NbVInts; ++i)
        {
            dt = (VInts[i + 1] - VInts[i]);
            NbVSubInts[i] = RealToInt(nbsv * dt * t) + 1;
            NbV += NbVSubInts[i];
        }

        UPars = new NCollection_HArray1<double>(1, NbU + 1);
        VPars = new NCollection_HArray1<double>(1, NbV + 1);
        //
        ind = 1;
        for (i = 1; i <= NbUInts; ++i)
        {
            UPars.SetValue(ind++, UInts[i]);
            dt = (UInts[i + 1] - UInts[i]) / NbUSubInts[i];
            t = UInts[i];
            for (j = 1; j < NbUSubInts[i]; ++j)
            {
                t += dt;
                UPars.SetValue(ind++, t);
            }
        }

        UPars.SetValue(ind, UInts[NbUInts + 1]);
        //
        ind = 1;
        for (i = 1; i <= NbVInts; ++i)
        {
            VPars.SetValue(ind++, VInts[i]);
            dt = (VInts[i + 1] - VInts[i]) / NbVSubInts[i];
            t = VInts[i];
            for (j = 1; j < NbVSubInts[i]; ++j)
            {
                t += dt;
                VPars.SetValue(ind++, t);
            }
        }

        VPars.SetValue(ind, VInts[NbVInts + 1]);
    }

    /// <summary>The type of the face's surface.</summary>
    public SurfaceKind SurfaceType() => (Adaptor3d_HSurfaceTool.GetType(Hsurface));

    /// <summary>
    /// Loads the face <paramref name="Face"/>. <paramref name="aTol"/> decides whether the first point of a segment
    /// is near the face (the parameter of an intersection may then be negative, above -aTol). With
    /// <paramref name="aRestr"/> the surface is restricted to the face's UV box. With <paramref name="UseBToler"/> the
    /// intersection points are classified with the larger of aTol and the tolerances of the face's edges, otherwise
    /// with a null tolerance.
    /// </summary>
    public IntCurvesFace_Intersector(in TopoDS_Face Face, double aTol, bool aRestr = true, bool UseBToler = true)
    {
        Tol = aTol;
        done = false;
        myReady = false;
        nbpnt = 0;
        myUseBoundTol = UseBToler;
        myIsParallel = false;

        // C++ initialises a local adaptor and keeps a copy of it; the port keeps the adaptor itself
        var surface = new BRepAdaptor_Surface();
        face = Face;
        surface.Initialize(Face, aRestr);
        Hsurface = surface;
        myTopolTool = new BRepTopAdaptor_TopolTool(Hsurface);

        SurfaceKind SurfaceType = Adaptor3d_HSurfaceTool.GetType(Hsurface);
        if ((SurfaceType != Plane) && (SurfaceType != SurfaceKind.Cylinder)
            && (SurfaceType != SurfaceKind.Cone) && (SurfaceType != SurfaceKind.Sphere)
            && (SurfaceType != SurfaceKind.Torus))
        {
            int nbsu, nbsv;
            double U0, V0, U1, V1;
            U0 = Hsurface.FirstUParameter();
            U1 = Hsurface.LastUParameter();
            V0 = Hsurface.FirstVParameter();
            V1 = Hsurface.LastVParameter();
            //
            nbsu = myTopolTool.NbSamplesU();
            nbsv = myTopolTool.NbSamplesV();
            //
            double aURes = Hsurface.UResolution(1.0);
            double aVRes = Hsurface.VResolution(1.0);

            // Checking correlation between number of samples and length of the face along each axis
            const double aTresh = 100.0;
            int aMinSamples = 20;
            const int aMaxSamples = 40;
            const int aMaxSamples2 = aMaxSamples * aMaxSamples;
            double dU = (U1 - U0) / aURes;
            double dV = (V1 - V0) / aVRes;
            if (nbsu < aMinSamples)
            {
                nbsu = aMinSamples;
            }

            if (nbsv < aMinSamples)
            {
                nbsv = aMinSamples;
            }

            if (nbsu > aMaxSamples)
            {
                nbsu = aMaxSamples;
            }

            if (nbsv > aMaxSamples)
            {
                nbsv = aMaxSamples;
            }

            if (dU > global::Kerf.Engine.Foundation.Precision.Confusion() && dV > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                if (Max(dU, dV) > Min(dU, dV) * aTresh)
                {
                    aMinSamples = 10;
                    nbsu = (int)(Math.Sqrt(dU / dV) * aMaxSamples);
                    if (nbsu < aMinSamples)
                    {
                        nbsu = aMinSamples;
                    }

                    nbsv = aMaxSamples2 / nbsu;
                    if (nbsv < aMinSamples)
                    {
                        nbsv = aMinSamples;
                        nbsu = aMaxSamples2 / aMinSamples;
                    }
                }
            }
            else
            {
                return; // surface has no extension along one of directions
            }

            int NbUOnS = Hsurface.NbUIntervals(C2);
            int NbVOnS = Hsurface.NbVIntervals(C2);

            if (NbUOnS > 1 || NbVOnS > 1)
            {
                ComputeSamplePars(Hsurface, nbsu, nbsv, out NCollection_HArray1<double> UPars, out NCollection_HArray1<double> VPars);
                myPolyhedron = new IntCurveSurface_ThePolyhedronOfHInter(Hsurface,
                                                                         UPars.ChangeArray1(),
                                                                         VPars.ChangeArray1());
            }
            else
            {
                myPolyhedron = new IntCurveSurface_ThePolyhedronOfHInter(Hsurface, nbsu, nbsv, U0, V0, U1, V1);
            }
        }

        myReady = true;
    }

    private void InternalCall(IntCurveSurface_HInter HICS, double parinf, double parsup)
    {
        if (HICS.IsDone() && HICS.NbPoints() > 0)
        {
            // Calculate tolerance for 2d classifier
            double mintol3d = BRep_Tool.Tolerance(face);
            double maxtol3d = mintol3d;
            double mintol2d = Tol, maxtol2d = Tol;
            var anExp = new TopExp_Explorer(face, TopAbs_EDGE);
            for (; anExp.More(); anExp.Next())
            {
                double curtol = BRep_Tool.Tolerance(TopoDS.Edge(anExp.Current()));
                mintol3d = Min(mintol3d, curtol);
                maxtol3d = Max(maxtol3d, curtol);
            }

            double minres = Max(Hsurface.UResolution(mintol3d), Hsurface.VResolution(mintol3d));
            double maxres = Max(Hsurface.UResolution(maxtol3d), Hsurface.VResolution(maxtol3d));
            mintol2d = Max(minres, Tol);
            maxtol2d = Max(maxres, Tol);
            //
            BRepTopAdaptor_TopolTool? anAdditionalTool = null;
            for (int index = HICS.NbPoints(); index >= 1; index--)
            {
                IntCurveSurface_IntersectionPoint HICSPointindex = HICS.Point(index);
                Point2D Puv = new(HICSPointindex.U(), HICSPointindex.V());

                // TopAbs_State currentstate = myTopolTool->Classify(Puv,Tol);
                TopAbs_State currentstate = myTopolTool.Classify(Puv, !myUseBoundTol ? 0 : mintol2d);
                if (myUseBoundTol && currentstate == TopAbs_OUT && maxtol2d > mintol2d)
                {
                    anAdditionalTool ??= new BRepTopAdaptor_TopolTool(Hsurface);

                    currentstate = anAdditionalTool.Classify(Puv, maxtol2d);
                    if (currentstate == TopAbs_ON)
                    {
                        currentstate = TopAbs_OUT;
                        // Find out nearest edge and it's tolerance
                        anExp.Init(face, TopAbs_EDGE);
                        for (; anExp.More(); anExp.Next())
                        {
                            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
                            // OCCT: an edge without a 3D curve (a degenerated edge) gives a null curve to the projection, which
                            // throws (occt-issues.md)
                            Geom_Curve aPC = BRep_Tool.Curve(anE, out double f, out double l)!;
                            var aProj = new GeomAPI_ProjectPointOnCurve(HICSPointindex.Pnt(), aPC, f, l);
                            if (aProj.NbPoints() > 0)
                            {
                                if (aProj.LowerDistance() <= maxtol3d)
                                {
                                    // Nearest edge is found, state is really ON
                                    currentstate = TopAbs_ON;
                                    break;
                                }
                            }
                        }
                    }
                }

                if (currentstate == TopAbs_IN || currentstate == TopAbs_ON)
                {
                    double HICSW = HICSPointindex.W();
                    if (HICSW >= parinf && HICSW <= parsup)
                    {
                        double U = HICSPointindex.U();
                        double V = HICSPointindex.V();
                        double W = HICSW;
                        IntCurveSurface_TransitionOnCurve transition = HICSPointindex.Transition();
                        Point3D pnt = HICSPointindex.Pnt();
                        //  Modified by skv - Wed Sep  3 16:14:10 2003 OCC578 Begin
                        int anIntState = (currentstate == TopAbs_IN) ? 0 : 1;
                        //  Modified by skv - Wed Sep  3 16:14:11 2003 OCC578 End

                        if (transition != IntCurveSurface_Tangent && face.Orientation() == TopAbs_REVERSED)
                        {
                            if (transition == IntCurveSurface_In)
                            {
                                transition = IntCurveSurface_Out;
                            }
                            else
                            {
                                transition = IntCurveSurface_In;
                            }
                        }

                        //----- Insertion du point
                        if (nbpnt == 0)
                        {
                            var PPP = new IntCurveSurface_IntersectionPoint(pnt, U, V, W, transition);
                            SeqPnt.Append(PPP);
                            //  Modified by skv - Wed Sep  3 16:14:10 2003 OCC578 Begin
                            mySeqState.Append(anIntState);
                            //  Modified by skv - Wed Sep  3 16:14:11 2003 OCC578 End
                        }
                        else
                        {
                            int i = 1;
                            int b = nbpnt + 1;
                            while (i <= nbpnt)
                            {
                                IntCurveSurface_IntersectionPoint Pnti = SeqPnt.Value(i);
                                double wi = Pnti.W();
                                if (wi >= W)
                                {
                                    b = i;
                                    i = nbpnt;
                                }

                                i++;
                            }

                            var PPP = new IntCurveSurface_IntersectionPoint(pnt, U, V, W, transition);
                            //  Modified by skv - Wed Sep  3 16:14:10 2003 OCC578 Begin
                            if (b > nbpnt)
                            {
                                SeqPnt.Append(PPP);
                                mySeqState.Append(anIntState);
                            }
                            else if (b > 0)
                            {
                                SeqPnt.InsertBefore(b, PPP);
                                mySeqState.InsertBefore(b, anIntState);
                            }
                            //  Modified by skv - Wed Sep  3 16:14:11 2003 OCC578 End
                        }

                        nbpnt++;
                    }
                } //-- classifier state is IN or ON
            } //-- Loop on Intersection points.
        } //-- HICS.IsDone()
        else if (HICS.IsDone())
        {
            myIsParallel = HICS.IsParallel();
        }
    }

    /// <summary>
    /// Intersects the segment [<paramref name="ParMin"/>, <paramref name="ParMax"/>] of the line
    /// <paramref name="L"/> (they may be ∓RealLast) with the face.
    /// </summary>
    public void Perform(in Line3D L, double ParMin, double ParMax)
    {
        done = false;
        if (!myReady)
        {
            return;
        }

        done = true;
        SeqPnt.Clear();
        mySeqState.Clear();
        nbpnt = 0;

        var HICS = new IntCurveSurface_HInter();
        var geomline = new Geom_Line(L);
        // C++ builds an adaptor and copies it into a handle: one adaptor here
        var HLL = new GeomAdaptor_Curve(geomline);
        double parinf = ParMin;
        double parsup = ParMax;
        //
        if (myPolyhedron is null)
        {
            HICS.Perform(HLL, Hsurface);
        }
        else
        {
            var bndTool = new Intf_Tool();
            Box boxLine = new();
            bndTool.LinBox(L, myPolyhedron.Bounding(), ref boxLine);
            if (bndTool.NbSegments() == 0)
            {
                return;
            }

            for (int nbseg = 1; nbseg <= bndTool.NbSegments(); nbseg++)
            {
                double pinf = bndTool.BeginParam(nbseg);
                double psup = bndTool.EndParam(nbseg);
                double pppp = 0.05 * (psup - pinf);
                pinf -= pppp;
                psup += pppp;
                if ((psup - pinf) < 1e-10)
                {
                    pinf -= 1e-10;
                    psup += 1e-10;
                }

                if (nbseg == 1)
                {
                    parinf = pinf;
                    parsup = psup;
                }
                else
                {
                    if (parinf > pinf)
                    {
                        parinf = pinf;
                    }

                    if (parsup < psup)
                    {
                        parsup = psup;
                    }
                }
            }

            if (parinf > ParMax)
            {
                return;
            }

            if (parsup < ParMin)
            {
                return;
            }

            if (parinf < ParMin)
            {
                parinf = ParMin;
            }

            if (parsup > ParMax)
            {
                parsup = ParMax;
            }

            if (parinf > (parsup - 1e-9))
            {
                return;
            }

            var polygon = new IntCurveSurface_ThePolygonOfHInter(HLL, parinf, parsup, 2);
            // OPTIMISATION is 1 in OCCT: the boxes of the triangles are sorted once for all the curves
            if (myBndBounding is null)
            {
                myBndBounding = new BoundSortBox();
                myBndBounding.Initialize(IntCurveSurface_ThePolyhedronToolOfHInter.Bounding(myPolyhedron),
                                         IntCurveSurface_ThePolyhedronToolOfHInter.ComponentsBounding(myPolyhedron));
            }

            HICS.Perform(HLL, polygon, Hsurface, myPolyhedron, myBndBounding);
        }

        InternalCall(HICS, parinf, parsup);
    }

    /// <summary>
    /// Intersects the part [<paramref name="ParMin"/>, <paramref name="ParMax"/>] of the curve
    /// <paramref name="HCu"/> (they may be ∓infinite) with the face.
    /// </summary>
    public void Perform(Adaptor3d_Curve HCu, double ParMin, double ParMax)
    {
        done = false;
        if (!myReady)
        {
            return;
        }

        done = true;
        SeqPnt.Clear();
        //  Modified by skv - Wed Sep  3 16:14:10 2003 OCC578 Begin
        mySeqState.Clear();
        //  Modified by skv - Wed Sep  3 16:14:11 2003 OCC578 End
        nbpnt = 0;
        var HICS = new IntCurveSurface_HInter();

        //--
        double parinf = ParMin;
        double parsup = ParMax;

        if (myPolyhedron is null)
        {
            HICS.Perform(HCu, Hsurface);
        }
        else
        {
            parinf = IntCurveSurface_TheHCurveTool.FirstParameter(HCu);
            parsup = IntCurveSurface_TheHCurveTool.LastParameter(HCu);
            if (parinf < ParMin)
            {
                parinf = ParMin;
            }

            if (parsup > ParMax)
            {
                parsup = ParMax;
            }

            if (parinf > (parsup - 1e-9))
            {
                return;
            }

            int nbs;
            nbs = IntCurveSurface_TheHCurveTool.NbSamples(HCu, parinf, parsup);

            var polygon = new IntCurveSurface_ThePolygonOfHInter(HCu, parinf, parsup, nbs);
            // OPTIMISATION is 1 in OCCT: the boxes of the triangles are sorted once for all the curves
            if (myBndBounding is null)
            {
                myBndBounding = new BoundSortBox();
                myBndBounding.Initialize(IntCurveSurface_ThePolyhedronToolOfHInter.Bounding(myPolyhedron),
                                         IntCurveSurface_ThePolyhedronToolOfHInter.ComponentsBounding(myPolyhedron));
            }

            HICS.Perform(HCu, polygon, Hsurface, myPolyhedron, myBndBounding);
        }

        InternalCall(HICS, parinf, parsup);
    }

    /// <summary>True when the intersection has been computed.</summary>
    public bool IsDone() => (done);

    /// <summary>The number of intersection points.</summary>
    public int NbPnt() => (nbpnt);

    /// <summary>The U parameter of the <paramref name="i"/>-th point on the surface.</summary>
    public double UParameter(int i) => (SeqPnt.Value(i).U());

    /// <summary>The V parameter of the <paramref name="i"/>-th point on the surface.</summary>
    public double VParameter(int i) => (SeqPnt.Value(i).V());

    /// <summary>The parameter of the <paramref name="i"/>-th point on the line or curve.</summary>
    public double WParameter(int i) => (SeqPnt.Value(i).W());

    /// <summary>The <paramref name="i"/>-th point.</summary>
    public Point3D Pnt(int i) => (SeqPnt.Value(i).Pnt());

    /// <summary>The transition of the line or curve on the surface at the <paramref name="i"/>-th point (taking the face's orientation into account).</summary>
    public IntCurveSurface_TransitionOnCurve Transition(int i) => (SeqPnt.Value(i).Transition());

    /// <summary>The state of the <paramref name="i"/>-th point in the face: IN, or ON its boundary.</summary>
    //  Modified by skv - Wed Sep  3 15:34:18 2003 OCC578 Begin
    public TopAbs_State State(int i) => (mySeqState.Value(i) == 0) ? TopAbs_IN : TopAbs_ON;

    /// <summary>
    /// True if the curve is parallel to the surface or lies on it (only recognised for some analytic pairs, a line and
    /// a plane for one).
    /// </summary>
    public bool IsParallel() => myIsParallel;

    /// <summary>The face.</summary>
    public TopoDS_Face Face() => (face);

    /// <summary>The state of the point <paramref name="Puv"/> of the face's parameter space (tolerance 1e-7).</summary>
    public TopAbs_State ClassifyUVPoint(in Point2D Puv)
    {
        TopAbs_State state = myTopolTool.Classify(Puv, 1e-7);
        return state;
    }

    /// <summary>The box of the polyhedron of the surface; a void box for the elementary surfaces.</summary>
    public Box Bounding()
    {
        if (myPolyhedron is not null)
        {
            return myPolyhedron.Bounding();
        }
        else
        {
            Box B = new();
            return B;
        }
    }

    /// <summary>Sets whether the points are classified with the tolerances of the face's edges.</summary>
    public void SetUseBoundToler(bool UseBToler) => myUseBoundTol = UseBToler;

    /// <summary>True if the points are classified with the tolerances of the face's edges.</summary>
    public bool GetUseBoundToler() => myUseBoundTol;
}
