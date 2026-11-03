// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_Modification_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Intersection;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Occt.Draft_ErrorStatus;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.ResultType;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

// OCCT_DEBUG blocks (the traces of the vertex recomputation and of NewSurface's failures) are left out.

public sealed partial class Draft_Modification
{
    //=================================================================================================

    private bool InternalAdd(in Face F, in Direction3D Direction, double Angle, in Plane3D NeutralPlane, bool Flag = true)
    {
        if (myFMap.Contains(F))
        {
            return (badShape.IsNull);
        }

        Orientation oris = Orientation(myShape, F);
        Placement Lo;
        // gp_Dir NewDirection = Direction;
        // double NewAngle = Angle;
        Surface S = Tool.Surface(F, out Lo)!;
        S = (Surface)S.Transformed(Lo.Transformation());
        if (S.GetType() == typeof(TrimmedSurface))
        {
            S = ((TrimmedSurface)S).BasisSurface();
        }
        Surface? NewS = null;
        CircleCurve3D? theCircle = null;

        bool postponed = (!Flag);
        if (postponed)
        {
            Type typS = S.GetType();
            if (typS == typeof(CylindricalSurface) || typS == typeof(ExtrusionSurface))
            {
                Circle3D Cir;
                if (typS == typeof(CylindricalSurface))
                {
                    Cylinder cyl = ((CylindricalSurface)S).Cylinder();
                    Axis3D axcyl = cyl.Axis();
                    Cir = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(cyl.Position(), cyl.Radius(), 0.0);
                    Vector3D VV = new(cyl.Location(), NeutralPlane.Location());
                    Cir.Translate(VV.Dot(new Vector3D(axcyl.Direction())) * new Vector3D(axcyl.Direction()));
                }
                else
                {
                    Curve3D Cbas = ((ExtrusionSurface)S).BasisCurve();
                    Direction3D theDirextr = ((ExtrusionSurface)S).Direction();

                    if (Cbas is TrimmedCurve3D aTrimmed)
                    {
                        Cbas = aTrimmed.BasisCurve();
                    }
                    if (Cbas is CircleCurve3D aCircle)
                    {
                        Cir = aCircle.Circ();
                        Direction3D dircir = Cir.Axis().Direction();
                        if (!Direction.IsParallel(dircir, global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            badShape = F;
                            errStat = Draft_FaceRecomputation;
                            return false;
                        }
                    }
                    else
                    {
                        badShape = F;
                        errStat = Draft_FaceRecomputation;
                        return false;
                    }

                    CoordinateSystem Axis = NeutralPlane.Position();
                    double L = new Vector3D(Cir.Location(), Axis.Location()).Dot(new Vector3D(Axis.Direction()));
                    double Cos = theDirextr.Dot(Axis.Direction());
                    Vector3D VV = (L / Cos) * new Vector3D(theDirextr);
                    Cir.Translate(VV);
                }

                theCircle = new CircleCurve3D(Cir);
            }
            else
            {
                postponed = false;
            }
        }

        if (!postponed)
        {
            NewS = NewSurface(S, oris, Direction, Angle, NeutralPlane);
            if (NewS is null)
            {
                badShape = F;
                errStat = Draft_FaceRecomputation;
                return false;
            }
            // To avoid some problems with infinite restrictions
            Type typs = NewS.GetType();
            if (typs == typeof(CylindricalSurface) || typs == typeof(ConicalSurface))
            {
                BrepTools.UVBounds(F, out double umin, out double umax, out double vmin, out double vmax);
                if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vmin) && !global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vmax))
                {
                    double deltav = 10.0 * (vmax - vmin);
                    if (typs == typeof(CylindricalSurface))
                    {
                        vmin = vmin - deltav;
                        vmax = vmax + deltav;
                    }
                    else
                    {
                        Cone Co = ((ConicalSurface)NewS).Cone();
                        double Vapex = -Co.RefRadius() / Math.Sin(Co.SemiAngle());
                        if (vmin < Vapex)
                        { // vmax should not exceed Vapex
                            if (vmax + deltav > Vapex)
                            {
                                vmax = Vapex;
                                vmin = vmin - 10.0 * (vmax - vmin);
                                // JAG debug to avoid apex
                                vmax = vmax - global::Kerf.Engine.Foundation.Precision.Confusion();
                            }
                            else
                            {
                                vmin = vmin - deltav;
                                vmax = vmax + deltav;
                            }
                        }
                        else
                        { // Vapex <= vmin < vmax
                            if (vmin - deltav < Vapex)
                            {
                                vmin = Vapex;
                                vmax = vmax + 10.0 * (vmax - vmin);
                                // JAG debug to avoid apex
                                vmin = vmin + global::Kerf.Engine.Foundation.Precision.Confusion();
                            }
                            else
                            {
                                vmin = vmin - deltav;
                                vmax = vmax + deltav;
                            }
                        }
                    }
                    NewS = new TrimmedSurface(NewS, 0.0, 2.0 * M_PI, vmin, vmax);
                }
            }
        }

        if (postponed || S != NewS)
        {
            Draft_FaceInfo FI = new(NewS, true);
            FI.RootFace(curFace);
            myFMap.Add(F, FI);
            if (postponed)
            {
                myFMap.ChangeFromKey(F).ChangeCurve() = theCircle;
            }
        }

        Explorer aExp = new(F, ShapeType.Edge);
        NCollection_Map<Shape> MapOfE = new(ShapeHasher.Instance);
        while (aExp.More() && badShape.IsNull)
        {
            Edge edg = ShapeCasts.Edge(aExp.Current());
            if (!myEMap.Contains(edg))
            {
                bool addedg = false;
                bool addface = false;
                Face OtherF = new();
                // if (BRep_Tool::IsClosed(edg,F)) {
                if (BrepTools.IsReallyClosed(edg, F))
                {
                    addedg = true;
                    addface = false;
                }
                else
                {
                    // Find the other face containing the edge.
                    int nbother = 0;
                    for (NCollection_List<Shape>.Iterator it = new(myEFMap.FindFromKey(edg)); it.More(); it.Next())
                    {
                        if (!it.Value().IsSame(F))
                        {
                            if (OtherF.IsNull)
                            {
                                OtherF = ShapeCasts.Face(it.Value());
                            }
                            nbother++;
                        }
                    }
                    if (nbother >= 2)
                    {
                        badShape = edg;
                        errStat = Draft_EdgeRecomputation;
                    }
                    else if (!OtherF.IsNull && Tool.Continuity(edg, F, OtherF) >= G1)
                    {
                        addface = true;
                        addedg = true;
                    }
                    else if (nbother == 0)
                    {
                        //	    badShape = F;
                    }
                }
                if (addedg)
                {
                    if (postponed)
                    {
                        myFMap.ChangeFromKey(F).Add(OtherF);
                    }
                    Curve3D C = Tool.Curve(edg, out Placement L, out double f, out double l)!;
                    C = (Curve3D)C.Transformed(L.Transformation());
                    if (C.GetType() == typeof(TrimmedCurve3D))
                    {
                        C = ((TrimmedCurve3D)C).BasisCurve();
                    }
                    Curve3D? NewC = null;
                    Draft_EdgeInfo EInf = new(true);
                    if (postponed)
                    {
                        EInf.Add(F);
                        EInf.Add(OtherF);

                        // find fixed point
                        if (C is not LineCurve3D aLocalGeom)
                        {
                            badShape = edg;
                            errStat = Draft_EdgeRecomputation;
                        }
                        else
                        {
                            Line3D lin = aLocalGeom.Lin();
                            IntConicQuad ilipl = new(lin, NeutralPlane, global::Kerf.Engine.Foundation.Precision.Angular());
                            if (ilipl.IsDone && ilipl.PointCount != 0)
                            {
                                EInf.Tangent(ilipl.Point(1));
                            }
                            else
                            {
                                badShape = edg;
                                errStat = Draft_EdgeRecomputation;
                            }
                        }
                    }
                    else
                    {
                        NewC = NewCurve(C, S, oris, Direction, Angle, NeutralPlane, Flag);
                        if (NewC is null)
                        {
                            badShape = edg;
                            errStat = Draft_EdgeRecomputation;
                        }
                    }

                    if (NewC is TrimmedCurve3D T)
                    {
                        NewC = T.BasisCurve();
                    }
                    EInf.ChangeGeometry() = NewC;

                    EInf.RootFace(curFace);
                    myEMap.Add(edg, EInf);
                    MapOfE.Add(edg);
                    if (addface)
                    {
                        bool Fl = Flag;
                        Surface alocalSurface = Tool.Surface(OtherF, out Lo)!;
                        if (alocalSurface.GetType() == typeof(TrimmedSurface))
                        {
                            alocalSurface = ((TrimmedSurface)alocalSurface).BasisSurface();
                        }
                        Type typS = alocalSurface.GetType();
                        if (typS == typeof(CylindricalSurface) || typS == typeof(ExtrusionSurface))
                        {
                            if (myFMap.Contains(F))
                            {
                                if (!Flag && !postponed)
                                {
                                    myFMap.RemoveKey(F);
                                    foreach (Shape aKey in MapOfE)
                                    {
                                        myEMap.RemoveKey(ShapeCasts.Edge(aKey));
                                    }
                                }
                            }
                        }
                        InternalAdd(OtherF, Direction, Angle, NeutralPlane, Fl);
                    }
                }
            }
            aExp.Next();
        }
        return (badShape.IsNull);
    }

    //=================================================================================================

    private bool Propagate()
    {
        if (!badShape.IsNull)
        {
            return false;
        }

        // Set all edges and vertices of modified faces
        Face F;
        Edge E;
        Vertex V;
        Explorer editer = new();
        Explorer vtiter = new();

        for (int i = 1; i <= myFMap.Extent(); i++)
        {
            Face Fc = ShapeCasts.Face(myFMap.FindKey(i));

            // Exploration of the edges of the face
            editer.Init(Fc, ShapeType.Edge);
            while (editer.More())
            {
                E = ShapeCasts.Edge(editer.Current());

                if (!myEMap.Contains(E))
                {
                    Draft_EdgeInfo EInf = new(true);
                    myEMap.Add(E, EInf);
                }
                myEMap.ChangeFromKey(E).Add(Fc);

                // Exploration of the vertices of the edge
                vtiter.Init(E, ShapeType.Vertex);
                while (vtiter.More())
                {
                    V = ShapeCasts.Vertex(vtiter.Current());
                    if (!myVMap.Contains(V))
                    {
                        Draft_VertexInfo VInf = new();
                        myVMap.Add(V, VInf);
                    }

                    myVMap.ChangeFromKey(V).Add(E);
                    myVMap.ChangeFromKey(V).ChangeParameter(E) = Tool.Parameter(V, E);
                    vtiter.Next();
                }
                editer.Next();
            }
        }

        Explorer anc = new();
        bool found;

        // Set edges containing modified vertices.

        for (int i = 1; i <= myVMap.Extent(); i++)
        {
            Vertex Vt = ShapeCasts.Vertex(myVMap.FindKey(i));

            // Exploration of the ancestors of the vertex
            anc.Init(myShape, ShapeType.Edge);

            while (anc.More())
            {
                E = ShapeCasts.Edge(anc.Current());
                vtiter.Init(E, ShapeType.Vertex);
                found = false;
                while (vtiter.More())
                {
                    if (Vt.IsSame(ShapeCasts.Vertex(vtiter.Current())))
                    {
                        found = true;
                        break;
                    }
                    vtiter.Next();
                }
                if (found)
                {
                    if (!myEMap.Contains(E))
                    {
                        Draft_EdgeInfo EInf = new(false);
                        myEMap.Add(E, EInf);
                    }
                    myVMap.ChangeFromKey(Vt).Add(E);
                    myVMap.ChangeFromKey(Vt).ChangeParameter(E) = Tool.Parameter(Vt, E);
                }
                anc.Next();
            }
        }

        // Set faces containing modified edges
        for (int i = 1; i <= myEMap.Extent(); i++)
        {
            Edge Ed = ShapeCasts.Edge(myEMap.FindKey(i));
            for (NCollection_List<Shape>.Iterator it = new(myEFMap.FindFromKey(Ed)); it.More(); it.Next())
            {
                F = ShapeCasts.Face(it.Value());
                if (!myFMap.Contains(F))
                {
                    Surface S = Tool.Surface(F, out Placement L)!;
                    Surface NewS = (Surface)S.Transformed(L.Transformation());

                    Type typs = S.GetType();
                    if (typs == typeof(CylindricalSurface) || typs == typeof(ConicalSurface))
                    {
                        BrepTools.UVBounds(F, out double umin, out double umax, out double vmin, out double vmax);
                        if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vmin) && !global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vmax))
                        {
                            double deltav = 10.0 * (vmax - vmin);
                            vmin = vmin - deltav;
                            vmax = vmax + deltav;
                            NewS = new TrimmedSurface(NewS, 0.0, 2.0 * M_PI, vmin, vmax);
                        }
                    }

                    Draft_FaceInfo FInf = new(NewS, false);
                    myFMap.Add(F, FInf);
                }
                myEMap.ChangeFromKey(Ed).Add(F);
            }
        }

        //  Try to add faces for free borders...
        // JAG 09.11.95
        for (int i = 1; i <= myEMap.Extent(); i++)
        {
            Draft_EdgeInfo Einf = myEMap.ChangeFromIndex(i);
            if (Einf.NewGeometry() && Einf.Geometry() is null && Einf.SecondFace().IsNull)
            {
                Placement Loc;
                Surface S1 = Tool.Surface(Einf.FirstFace(), out Loc)!;
                S1 = (Surface)S1.Transformed(Loc.Transformation());
                Surface? S2 = null;

                double f, l;
                Edge EK = ShapeCasts.Edge(myEMap.FindKey(i));
                Curve3D C = Tool.Curve(EK, out Loc, out f, out l)!;
                C = (Curve3D)C.Transformed(Loc.Transformation());
                if (C.GetType() == typeof(TrimmedCurve3D))
                {
                    C = ((TrimmedCurve3D)C).BasisCurve();
                }
                if (S1 is not PlaneSurface)
                {
                    if (C is ConicCurve3D aConic)
                    {
                        CoordinateSystem thePl = new(aConic.Position());
                        S2 = new PlaneSurface(thePl);
                    }
                    else if (C.GetType() == typeof(LineCurve3D))
                    {
                        Axis3D axis;
                        if (S1.GetType() == typeof(TrimmedSurface))
                        {
                            axis = ((ElementarySurface)((TrimmedSurface)S1).BasisSurface()).Axis();
                        }
                        else
                        {
                            axis = ((ElementarySurface)S1).Axis();
                        }
                        Vector3D they = new(axis.Location(), C.Value(0.0));
                        Direction3D axz = new(new Vector3D(axis.Direction()).Crossed(they));
                        S2 = new PlaneSurface(new CoordinateSystem(axis.Location(), axz, axis.Direction()));
                    }
                    else
                    {
                        badShape = EK;
                        errStat = Draft_EdgeRecomputation;
                        break; // leave from for
                    }
                }
                else
                { // on the plane
                    for (int j = 1; j <= myVMap.Extent(); j++)
                    {
                        Draft_VertexInfo Vinf = myVMap.ChangeFromIndex(j);
                        for (Vinf.InitEdgeIterator(); Vinf.MoreEdge(); Vinf.NextEdge())
                        {
                            if (Vinf.Edge().IsSame(EK))
                            {
                                break;
                            }
                        }
                        if (Vinf.MoreEdge())
                        {
                            for (Vinf.InitEdgeIterator(); Vinf.MoreEdge(); Vinf.NextEdge())
                            {
                                Edge edg = Vinf.Edge();
                                if (!edg.IsSame(EK))
                                {
                                    Draft_EdgeInfo EI = myEMap.FindFromKey(edg);
                                    if (!EI.FirstFace().IsSame(Einf.FirstFace())
                                        && (EI.SecondFace().IsNull || !EI.SecondFace().IsSame(Einf.FirstFace())))
                                    {
                                        break;
                                    }
                                }
                            }
                            if (Vinf.MoreEdge())
                            {
                                Curve3D C2 = Tool.Curve(Vinf.Edge(), out Loc, out f, out l)!;
                                CurveAdaptor HCur;
                                Vector3D Direc;
                                C2 = (Curve3D)C2.Transformed(Loc.Transformation());
                                if (C2.GetType() == typeof(TrimmedCurve3D))
                                {
                                    C2 = ((TrimmedCurve3D)C2).BasisCurve();
                                }
                                if (C.GetType() == typeof(LineCurve3D))
                                {
                                    Direc = new Vector3D(((LineCurve3D)C).Lin().Direction());
                                    HCur = new CurveAdaptor(C2);
                                }
                                else if (C2.GetType() == typeof(LineCurve3D))
                                {
                                    Direc = new Vector3D(((LineCurve3D)C2).Lin().Direction());
                                    HCur = new CurveAdaptor(C);
                                }
                                else
                                {
                                    badShape = EK;
                                    errStat = Draft_EdgeRecomputation;
                                    break; // leave from while
                                }
                                SurfaceOfLinearExtrusion SLE = new(HCur, new Direction3D(Direc));
                                switch (SLE.GetType())
                                {
                                    case Plane:
                                        S2 = new PlaneSurface(SLE.Plane());
                                        break;
                                    case SurfaceKind.Cylinder:
                                        S2 = new CylindricalSurface(SLE.Cylinder());
                                        break;
                                    default:
                                        S2 = new ExtrusionSurface(HCur.Curve()!, new Direction3D(Direc));
                                        break;
                                }
                            }
                            else
                            {
                                badShape = EK;
                                errStat = Draft_EdgeRecomputation;
                                break; // leave from while
                            }
                            break;
                        }
                        // j++;
                    }
                }

                if (badShape.IsNull)
                {
                    BrepBuilder B = new();
                    Face TheNewFace = new();
                    B.MakeFace(ref TheNewFace, S2, global::Kerf.Engine.Foundation.Precision.Confusion());
                    Einf.Add(TheNewFace);
                    Draft_FaceInfo FI = new(S2, false);
                    myFMap.Add(TheNewFace, FI);
                }
                else
                {
                    break; // leave from for
                }
                // Fin JAG 09.11.95
            }
        }
        return (badShape.IsNull);
    }

    //=================================================================================================

    /// <summary>
    /// Performs the draft: computes the new faces, edges and vertices. On an error <see cref="IsDone"/> is false,
    /// <see cref="Error"/> tells what failed and <see cref="ProblematicShape"/> on which shape.
    /// </summary>
    /// <exception cref="ConstructionErrorException">An Add failed and the bad face wasn't removed.</exception>
    public void Perform()
    {
        if (!badShape.IsNull)
        {
            throw new ConstructionErrorException();
        }

        if (!myComp)
        {
            myComp = true;
            if (!Propagate())
            {
                return;
            }

            // Calculate eventual faces

            for (int i = 1; i <= myFMap.Extent(); i++)
            {
                Face FK = ShapeCasts.Face(myFMap.FindKey(i));
                Draft_FaceInfo Finf = myFMap.ChangeFromIndex(i);
                if (Finf.NewGeometry() && Finf.Geometry() is null)
                {
                    Face F1 = Finf.FirstFace();
                    Face F2 = Finf.SecondFace();

                    if (F1.IsNull || F2.IsNull)
                    {
                        errStat = Draft_FaceRecomputation;
                        badShape = FK;
                        return;
                    }
                    Surface? S1 = myFMap.FindFromKey(F1).Geometry();
                    Surface? S2 = myFMap.FindFromKey(F2).Geometry();
                    if (S1 is null || S2 is null)
                    {
                        errStat = Draft_FaceRecomputation;
                        badShape = FK;
                        return;
                    }
                    if (S1.GetType() == typeof(TrimmedSurface))
                    {
                        S1 = ((TrimmedSurface)S1).BasisSurface();
                    }
                    if (S2.GetType() == typeof(TrimmedSurface))
                    {
                        S2 = ((TrimmedSurface)S2).BasisSurface();
                    }
                    PlaneSurface? P1 = S1 as PlaneSurface;
                    PlaneSurface? P2 = S2 as PlaneSurface;
                    if (P1 is null || P2 is null)
                    {
                        errStat = Draft_FaceRecomputation;
                        badShape = FK;
                        return;
                    }
                    Plane3D pp1 = P1.Pln();
                    Plane3D pp2 = P2.Pln();
                    QuadQuadGeo i2p = new(pp1, pp2, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
                    if (!i2p.IsDone || i2p.TypeInter() != Line)
                    {
                        errStat = Draft_FaceRecomputation;
                        badShape = FK;
                        return;
                    }

                    Direction3D extrdir = i2p.Line(1).Direction();

                    // Preserve the same direction as the base face
                    Surface RefSurf = Tool.Surface(FK)!;
                    if (RefSurf.GetType() == typeof(TrimmedSurface))
                    {
                        RefSurf = ((TrimmedSurface)RefSurf).BasisSurface();
                    }
                    Direction3D DirRef = new();

                    if (RefSurf.GetType() == typeof(CylindricalSurface))
                    {
                        CoordinateSystem AxeRef = ((CylindricalSurface)RefSurf).Cylinder().Position();
                        DirRef = AxeRef.Direction();
                    }
                    else if (RefSurf.GetType() == typeof(ExtrusionSurface))
                    {
                        DirRef = ((ExtrusionSurface)RefSurf).Direction();
                    }

                    if (extrdir.Dot(DirRef) < 0.0)
                    {
                        extrdir.Reverse();
                    }

                    // it is possible to accelerate speed by storing the info during
                    // InternalAdd --> modification of FaceInfo to preserve the circle

                    // OCCT down-casts the curve without a check; a null circle throws in the extrusion's constructor.
                    CircleCurve3D CCir = (CircleCurve3D)Finf.Curve()!;
                    Surface NewS = new ExtrusionSurface(CCir, extrdir);

                    BrepTools.UVBounds(FK, out double umin, out double umax, out double vmin, out double vmax);
                    if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vmin) && !global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vmax))
                    {
                        double deltav = 2.0 * (vmax - vmin);
                        vmin = vmin - deltav;
                        vmax = vmax + deltav;
                    }

                    // very temporary
                    else
                    {
                        vmax = 300;
                        vmin = -300;
                    }

                    NewS = new TrimmedSurface(NewS, 0.0, 1.9 * M_PI, vmin, vmax);
                    Finf.ChangeGeometry() = NewS;
                }
            }

            // Calculate new edges.
            for (int ii = 1; ii <= myEMap.Extent(); ii++)
            {
                Draft_EdgeInfo Einf = myEMap.ChangeFromIndex(ii);

                Edge theEdge = ShapeCasts.Edge(myEMap.FindKey(ii));

                Surface? S1, S2;
                Curve3D C;
                Curve3D? newC = null;
                C = Tool.Curve(theEdge, out Placement L, out double f, out double l)!;
                C = (Curve3D)C.Transformed(L.Transformation());

                if (Einf.NewGeometry() && Einf.Geometry() is null)
                {
                    if (!Einf.IsTangent(out Point3D ptfixe))
                    {
                        Face FirstFace = Einf.FirstFace();
                        Face SecondFace = Einf.SecondFace();

                        S1 = myFMap.FindFromKey(FirstFace).Geometry()!;
                        S2 = myFMap.FindFromKey(SecondFace).Geometry()!;

                        int detrompeur = 0;

                        // Return FirstVertex and the tangent at this point.
                        Vertex FV = Exploration.FirstVertex(theEdge);
                        Vertex LV = Exploration.LastVertex(theEdge);
                        double pmin = 0.0;
                        double prmfv = Tool.Parameter(FV, theEdge);
                        double prmlv = Tool.Parameter(LV, theEdge);
                        Point3D pfv, plv;
                        Vector3D d1fv, d1lv, newd1;
                        C.D1(prmfv, out pfv, out d1fv);
                        C.D1(prmlv, out plv, out d1lv);

                        double TolF1 = Tool.Tolerance(FirstFace);
                        double TolF2 = Tool.Tolerance(SecondFace);

                        // Pass the tolerance of the face to project
                        GeomAPI_ProjectPointOnSurf proj1 = new(pfv, S1, TolF1);
                        GeomAPI_ProjectPointOnSurf proj2 = new(plv, S1, TolF1);
                        GeomAPI_ProjectPointOnSurf proj3 = new(pfv, S2, TolF2);
                        GeomAPI_ProjectPointOnSurf proj4 = new(plv, S2, TolF2);

                        if (proj1.IsDone() && proj2.IsDone())
                        {
                            if (proj1.LowerDistance() <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                && proj2.LowerDistance() <= global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                detrompeur = 1;
                            }
                        }

                        if (proj3.IsDone() && proj4.IsDone())
                        {
                            if (proj3.LowerDistance() <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                && proj4.LowerDistance() <= global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                detrompeur = 2;
                            }
                        }

                        Direction3D TheDirExtr = new();
                        CoordinateSystem Axis = new();
                        Curve3D? TheNewCurve = null;
                        bool KPart = false;

                        if (S1.GetType() == typeof(TrimmedSurface))
                        {
                            S1 = ((TrimmedSurface)S1).BasisSurface();
                        }
                        if (S2.GetType() == typeof(TrimmedSurface))
                        {
                            S2 = ((TrimmedSurface)S2).BasisSurface();
                        }

                        bool PC1 = true; // KPart on S1
                        if (S1.GetType() == typeof(ExtrusionSurface) && S2.GetType() == typeof(PlaneSurface))
                        {
                            KPart = true;
                            Axis = ((PlaneSurface)S2).Position();
                            TheNewCurve = ((ExtrusionSurface)S1).BasisCurve();
                            TheDirExtr = ((ExtrusionSurface)S1).Direction();
                        }
                        else if (S2.GetType() == typeof(ExtrusionSurface)
                                 && S1.GetType() == typeof(PlaneSurface))
                        {
                            KPart = true;
                            PC1 = false;
                            Axis = ((PlaneSurface)S1).Position();
                            TheNewCurve = ((ExtrusionSurface)S2).BasisCurve();
                            TheDirExtr = ((ExtrusionSurface)S2).Direction();
                        }
                        CircleCurve3D? aCirc = null;
                        if (KPart)
                        { // very temporary on circles !!!
                            aCirc = TheNewCurve as CircleCurve3D;
                            if (aCirc is null)
                            {
                                KPart = false;
                            }
                            else
                            {
                                Direction3D AxofCirc = aCirc.Position().Direction();
                                KPart = AxofCirc.IsParallel(Axis.Direction(), global::Kerf.Engine.Foundation.Precision.Angular());
                            }
                        }

                        int imin;
                        GeomInt_IntSS i2s = new();
                        if (KPart)
                        {
                            // direct calculation of NewC
                            double aLocalReal = new Vector3D(aCirc!.Circ().Location(), Axis.Location())
                                                    .Dot(new Vector3D(Axis.Direction()));
                            double Cos = TheDirExtr.Dot(Axis.Direction());
                            Vector3D VV = (aLocalReal / Cos) * new Vector3D(TheDirExtr);
                            newC = (Curve3D)TheNewCurve!.Translated(VV);
                            // it is possible to calculate PCurve
                            LineCurve2D L2d = new(new Point2D(0.0, aLocalReal / Cos), Constants.DX2d());

                            if (PC1)
                            {
                                Einf.ChangeFirstPC() = L2d;
                            }
                            else
                            {
                                Einf.ChangeSecondPC() = L2d;
                            }
                        }
                        else
                        {
                            S1 = myFMap.FindFromKey(Einf.FirstFace()).Geometry()!;
                            S2 = myFMap.FindFromKey(Einf.SecondFace()).Geometry()!;

                            // PCurves are not calculated immediately for 2 reasons:
                            // 1 - If ProjLib should make an Approx, it is stupid to approximate the
                            //     entire intersection curve.
                            // 2 - Additionally, if YaRev, there is a risk to not be SameRange.
                            i2s.Perform(S1, S2, global::Kerf.Engine.Foundation.Precision.Confusion(), true, false, false);

                            if (!i2s.IsDone() || i2s.NbLines() <= 0)
                            {
                                errStat = Draft_EdgeRecomputation;
                                badShape = theEdge;
                                return;
                            }

                            double Glob2Min = RealLast();
                            CurveAdaptor TheCurve = new();

                            int i, j; //,jmin;

                            if (i2s.Line(1).GetType() != typeof(BSplineCurve3D))
                            {
                                double Dist2Min = RealLast();
                                imin = 0;
                                for (i = 1; i <= i2s.NbLines(); i++)
                                {
                                    TheCurve.Load(i2s.Line(i));
                                    CurvePointExtrema myExtPC = new(pfv, TheCurve);

                                    double locpmin = 0.0;
                                    if (myExtPC.IsDone)
                                    {
                                        if (myExtPC.ExtCount >= 1)
                                        {
                                            Dist2Min = myExtPC.SquareDistance(1);
                                            locpmin = myExtPC.Point(1).Parameter();
                                        }
                                        if (myExtPC.ExtCount == 2 && Dist2Min > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                                        {
                                            // to avoid incorrectly choosing the image
                                            // of the first vertex of the initial edge
                                            double d1_2 = myExtPC.SquareDistance(1);
                                            double d2_2 = myExtPC.SquareDistance(2);
                                            if (d1_2 > 1.21 * d2_2)
                                            {
                                                Dist2Min = myExtPC.SquareDistance(2);
                                                locpmin = myExtPC.Point(2).Parameter();
                                            }
                                            else if (d2_2 > 1.21 * d1_2)
                                            {
                                                Dist2Min = myExtPC.SquareDistance(1);
                                                locpmin = myExtPC.Point(1).Parameter();
                                            }
                                            else
                                            {
                                                double pfvpar = myExtPC.Point(1).Parameter();
                                                double plvpar = myExtPC.Point(2).Parameter();
                                                newC = i2s.Line(i);

                                                newC.D0(pfvpar, out Point3D pfvprim);
                                                newC.D0(plvpar, out Point3D plvprim);

                                                Surface? theSurf = null;
                                                if (detrompeur == 1)
                                                {
                                                    if (S1.GetType() == typeof(TrimmedSurface))
                                                    {
                                                        S1 = ((TrimmedSurface)S1).BasisSurface();
                                                    }
                                                    theSurf = S1;
                                                }
                                                else if (detrompeur == 2)
                                                {
                                                    if (S2.GetType() == typeof(TrimmedSurface))
                                                    {
                                                        S2 = ((TrimmedSurface)S2).BasisSurface();
                                                    }
                                                    theSurf = S2;
                                                }
                                                if (detrompeur != 0 && detrompeur != 4)
                                                {
                                                    double ul = 0.0, vl = 0.0, uf = 0.0, vf = 0.0;
                                                    double ufprim = 0.0, ulprim = 0.0, vfprim = 0.0, vlprim = 0.0;

                                                    if (theSurf!.GetType() == typeof(PlaneSurface))
                                                    {
                                                        // OCCT reads S2 whatever theSurf is (a null down-cast throws).
                                                        Plane3D pl = downCast<PlaneSurface>(S2).Pln();
                                                        global::Kerf.Engine.Geometry.ElSLib.Parameters(pl, plv, out ul, out vl);
                                                        global::Kerf.Engine.Geometry.ElSLib.Parameters(pl, pfv, out uf, out vf);
                                                        global::Kerf.Engine.Geometry.ElSLib.Parameters(pl, plvprim, out ulprim, out vlprim);
                                                        global::Kerf.Engine.Geometry.ElSLib.Parameters(pl, pfvprim, out ufprim, out vfprim);
                                                    }
                                                    else if (theSurf.GetType() == typeof(CylindricalSurface))
                                                    {
                                                        Cylinder cy = downCast<CylindricalSurface>(S2).Cylinder();
                                                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cy, plv, out ul, out vl);
                                                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cy, pfv, out uf, out vf);
                                                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cy, plvprim, out ulprim, out vlprim);
                                                        global::Kerf.Engine.Geometry.ElSLib.Parameters(cy, pfvprim, out ufprim, out vfprim);
                                                    }
                                                    else
                                                    {
                                                        detrompeur = 4;
                                                    }

                                                    if (detrompeur == 1 || detrompeur == 2)
                                                    {
                                                        Vector2D v1 = new((ul - ufprim), (vl - vfprim));
                                                        Vector2D norm = new((vf - vfprim), (ufprim - uf));
                                                        Vector2D v2 = new((ulprim - ufprim), (vlprim - vfprim));
                                                        if ((v1.Dot(norm)) * (v2.Dot(norm)) < 0)
                                                        {
                                                            Dist2Min = myExtPC.SquareDistance(2);
                                                            locpmin = myExtPC.Point(2).Parameter();
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                        if (myExtPC.ExtCount == 1 || myExtPC.ExtCount > 2 || detrompeur == 4)
                                        {
                                            Dist2Min = myExtPC.SquareDistance(1);
                                            locpmin = myExtPC.Point(1).Parameter();
                                            for (j = 2; j <= myExtPC.ExtCount; j++)
                                            {
                                                double Dist2 = myExtPC.SquareDistance(j);
                                                if (Dist2 < Dist2Min)
                                                {
                                                    Dist2Min = Dist2;
                                                    locpmin = myExtPC.Point(j).Parameter();
                                                }
                                            }
                                        }
                                        else if (myExtPC.ExtCount < 1)
                                        {
                                            myExtPC.TrimmedSquareDistances(out double dist1_2, out double dist2_2,
                                                                           out Point3D _, out Point3D _);
                                            if (dist1_2 < dist2_2)
                                            {
                                                Dist2Min = dist1_2;
                                                locpmin = TheCurve.FirstParameter();
                                            }
                                            else
                                            {
                                                Dist2Min = dist2_2;
                                                locpmin = TheCurve.LastParameter();
                                            }
                                        }

                                        if (Dist2Min < Glob2Min)
                                        {
                                            Glob2Min = Dist2Min;
                                            imin = i;
                                            pmin = locpmin;
                                        }
                                    }
                                }
                                if (imin == 0)
                                {
                                    errStat = Draft_EdgeRecomputation;
                                    badShape = theEdge;
                                    return;
                                }

                                newC = i2s.Line(imin);

                                newC.D1(pmin, out pfv, out newd1);
                                bool YaRev = d1fv.Dot(newd1) < 0.0;

                                if (YaRev)
                                {
                                    reverse(ref newC);
                                }

                                if (i2s.HasLineOnS1(imin))
                                {
                                    Einf.ChangeFirstPC() = i2s.LineOnS1(imin);
                                    if (YaRev)
                                    {
                                        reverse(ref Einf.ChangeFirstPC());
                                    }
                                }

                                if (i2s.HasLineOnS2(imin))
                                {
                                    Einf.ChangeSecondPC() = i2s.LineOnS2(imin);
                                    if (YaRev)
                                    {
                                        reverse(ref Einf.ChangeSecondPC());
                                    }
                                }
                            } // if (i2s.Line(1)->DynamicType() != STANDARD_TYPE(Geom_BSplineCurve))
                            else // i2s.Line(1) is BSplineCurve
                            {
                                // OCCT.NET: OCCT reverses the intersection's own curves in place and compares them
                                // by handle; the port's are frozen, so this branch works on unfrozen copies of them,
                                // made once (the same handles throughout).
                                NCollection_Array1<Curve3D> aLines = new(1, i2s.NbLines());
                                for (i = 1; i <= i2s.NbLines(); i++)
                                {
                                    aLines[i] = (Curve3D)i2s.Line(i).Copy();
                                }

                                // Find the first curve to glue
                                NCollection_Sequence<Curve3D> Candidates = new();
                                if (S1.GetType() == typeof(CylindricalSurface)
                                    || S1.GetType() == typeof(ConicalSurface))
                                {
                                    for (i = 1; i <= i2s.NbLines(); i++)
                                    {
                                        Curve3D aCurve = aLines[i];
                                        Point3D Pnt = aCurve.Value(aCurve.FirstParameter());
                                        GeomAPI_ProjectPointOnSurf projector = new(Pnt, S1, global::Kerf.Engine.Foundation.Precision.Confusion());
                                        projector.LowerDistanceParameters(out double U, out double V);
                                        if (Math.Abs(U) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                            || Math.Abs(U - 2.0 * M_PI) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                                        {
                                            Candidates.Append(aCurve);
                                        }
                                        else
                                        {
                                            Pnt = aCurve.Value(aCurve.LastParameter());
                                            projector.Init(Pnt, S1, global::Kerf.Engine.Foundation.Precision.Confusion());
                                            projector.LowerDistanceParameters(out U, out V);
                                            if (Math.Abs(U) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                                || Math.Abs(U - 2.0 * M_PI) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                                            {
                                                aCurve.Reverse();
                                                Candidates.Append(aCurve);
                                            }
                                        }
                                    }

                                    if (Candidates.Length() == 0)
                                    {
                                        // errStat = Draft_EdgeRecomputation;
                                        // badShape = TopoDS::Edge(ite.Key());
                                        // return;
                                        for (i = 1; i <= i2s.NbLines(); i++)
                                        {
                                            Candidates.Append(aLines[i]);
                                        }
                                    }
                                }
                                else
                                {
                                    for (i = 1; i <= i2s.NbLines(); i++)
                                    {
                                        Candidates.Append(aLines[i]);
                                    }
                                }

                                Curve3D? FirstCurve = null;
                                if (Candidates.Length() > 1)
                                {
                                    double DistMin = global::Kerf.Engine.Foundation.Precision.Infinite();
                                    for (i = 1; i <= Candidates.Length(); i++)
                                    {
                                        Curve3D aCurve = Candidates[i];
                                        Point3D Pnt = aCurve.Value(aCurve.FirstParameter());
                                        double Dist = Pnt.Distance(pfv);
                                        if (Dist - DistMin < -global::Kerf.Engine.Foundation.Precision.Confusion())
                                        {
                                            DistMin = Dist;
                                            FirstCurve = aCurve;
                                        }
                                    }
                                }
                                else
                                {
                                    FirstCurve = Candidates[1];
                                }

                                // Glueing
                                NCollection_Sequence<Curve3D> Curves = new();
                                for (i = 1; i <= i2s.NbLines(); i++)
                                {
                                    if (FirstCurve != aLines[i])
                                    {
                                        Curves.Append(aLines[i]);
                                    }
                                }

                                NCollection_Sequence<Curve3D> ToGlue = new();
                                // OCCT reads FirstCurve before its null check below.
                                Point3D EndPoint = FirstCurve!.Value(FirstCurve.LastParameter());
                                bool added = true;
                                while (added)
                                {
                                    added = false;
                                    for (i = 1; i <= Curves.Length(); i++)
                                    {
                                        Curve3D aCurve = Curves[i];
                                        Point3D pfirst, plast;
                                        pfirst = aCurve.Value(aCurve.FirstParameter());
                                        plast = aCurve.Value(aCurve.LastParameter());
                                        if (pfirst.Distance(EndPoint) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                                        {
                                            ToGlue.Append(aCurve);
                                            EndPoint = plast;
                                            Curves.Remove(i);
                                            added = true;
                                            break;
                                        }
                                        if (plast.Distance(EndPoint) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                                        {
                                            aCurve.Reverse();
                                            ToGlue.Append(aCurve);
                                            EndPoint = pfirst;
                                            Curves.Remove(i);
                                            added = true;
                                            break;
                                        }
                                    }
                                }

                                if (FirstCurve is null)
                                {
                                    errStat = Draft_EdgeRecomputation;
                                    badShape = theEdge;
                                    return;
                                }

                                CompCurveToBSplineCurve Concat = new(downCast<BSplineCurve3D>(FirstCurve));
                                for (i = 1; i <= ToGlue.Length(); i++)
                                {
                                    Concat.Add(downCast<BSplineCurve3D>(ToGlue[i]), global::Kerf.Engine.Foundation.Precision.Confusion(), true);
                                }

                                newC = Concat.BSplineCurve()!;

                                TheCurve.Load(newC);
                                CurvePointExtrema myExtPC = new(pfv, TheCurve);
                                double Dist2Min = RealLast();
                                for (i = 1; i <= myExtPC.ExtCount; i++)
                                {
                                    if (myExtPC.IsMin(i))
                                    {
                                        double Dist2 = myExtPC.SquareDistance(i);
                                        if (Dist2 < Dist2Min)
                                        {
                                            Dist2Min = Dist2;
                                            pmin = myExtPC.Point(i).Parameter();
                                        }
                                    }
                                }
                                newC.D1(pmin, out pfv, out newd1);
                                bool YaRev = d1fv.Dot(newd1) < 0.0;

                                if (YaRev)
                                {
                                    reverse(ref newC);
                                }
                            } // else: i2s.NbLines() > 2 && S1 is Cylinder or Cone

                            Einf.Tolerance(Max(Einf.Tolerance(), i2s.TolReached3d()));
                        } // End step KPart
                    }
                    else
                    { // case of tangency
                        Face F1 = Einf.FirstFace();
                        Face F2 = Einf.SecondFace();

                        Surface? aLocalS1 = myFMap.FindFromKey(F1).Geometry();
                        Surface? aLocalS2 = myFMap.FindFromKey(F2).Geometry();
                        if (aLocalS1 is null || aLocalS2 is null)
                        {
                            errStat = Draft_EdgeRecomputation;
                            badShape = theEdge;
                            return;
                        }
                        if (aLocalS1.GetType() == typeof(TrimmedSurface))
                        {
                            aLocalS1 = ((TrimmedSurface)aLocalS1).BasisSurface();
                        }
                        if (aLocalS2.GetType() == typeof(TrimmedSurface))
                        {
                            aLocalS2 = ((TrimmedSurface)aLocalS2).BasisSurface();
                        }

                        Direction3D dirextr = new();
                        // bool dirfound = false;
                        if (aLocalS1.GetType() == typeof(CylindricalSurface))
                        {
                            Cylinder cyl = ((CylindricalSurface)aLocalS1).Cylinder();
                            dirextr = cyl.Axis().Direction();
                            // dirfound = true;
                            //  see direction...
                        }
                        else if (aLocalS1.GetType() == typeof(ExtrusionSurface))
                        {
                            dirextr = ((ExtrusionSurface)aLocalS1).Direction();
                            // dirfound = true;
                            //  see direction...

                            // Here it is possible to calculate PCurve.
                            ExtrusionSurface SEL = (ExtrusionSurface)aLocalS1;
                            if (SEL.BasisCurve() is CircleCurve3D GCir)
                            {
                                double U = global::Kerf.Engine.Geometry.ElCLib.Parameter(GCir.Circ(), ptfixe);
                                LineCurve2D PC1 = new(new Point2D(U, 0.0), Constants.DY2d());
                                Einf.ChangeFirstPC() = PC1;
                            }
                        }

                        else if (aLocalS2.GetType() == typeof(CylindricalSurface))
                        {
                            Cylinder cyl = ((CylindricalSurface)aLocalS2).Cylinder();
                            dirextr = cyl.Axis().Direction();
                            // dirfound = true;
                            // see direction...
                        }
                        else if (aLocalS2.GetType() == typeof(ExtrusionSurface))
                        {
                            dirextr = ((ExtrusionSurface)aLocalS2).Direction();
                            // dirfound = true;
                            // see direction...

                            // Here it is possible to calculate PCurve.
                            ExtrusionSurface SEL = (ExtrusionSurface)aLocalS2;
                            if (SEL.BasisCurve() is CircleCurve3D GCir)
                            {
                                double U = global::Kerf.Engine.Geometry.ElCLib.Parameter(GCir.Circ(), ptfixe);
                                LineCurve2D PC2 = new(new Point2D(U, 0.0), Constants.DY2d());
                                Einf.ChangeSecondPC() = PC2;
                            }
                        }
                        newC = new LineCurve3D(ptfixe, dirextr);

                        C.D1(0.0, out Point3D pfv, out Vector3D d1fv);
                        newC.D1(0.0, out pfv, out Vector3D newd1);
                        bool YaRev = d1fv.Dot(newd1) < 0.0;
                        if (YaRev)
                        {
                            reverse(ref newC);
                            if (Einf.FirstPC() is not null)
                            {
                                reverse(ref Einf.ChangeFirstPC());
                            }
                            if (Einf.SecondPC() is not null)
                            {
                                reverse(ref Einf.ChangeSecondPC());
                            }
                        }
                    }

                    if (newC is TrimmedCurve3D T)
                    {
                        newC = T.BasisCurve();
                    }
                    Einf.ChangeGeometry() = newC;
                }
                else if (!Einf.NewGeometry())
                {
                    // set existing curve 3D
                    if (C is TrimmedCurve3D T)
                    {
                        C = T.BasisCurve();
                    }
                    Einf.ChangeGeometry() = C;
                }
            }

            // Calculate new vertices.

            CurveAdaptor HAC = new();
            SurfaceAdaptor HAS = new();

            for (int ii = 1; ii <= myVMap.Extent(); ii++)
            {
                CurveAdaptor AC = HAC;
                SurfaceAdaptor AS = HAS;

                Vertex TVV = ShapeCasts.Vertex(myVMap.FindKey(ii));
                Draft_VertexInfo Vinf = myVMap.ChangeFromIndex(ii);
                if (!Choose(myFMap, myEMap, TVV, Vinf, AC, AS))
                {
                    // no concerted edge => alignment of two consecutive edges.
                    Point3D pvt = new();
                    Vinf.ChangeGeometry() = pvt;
                    Vinf.InitEdgeIterator();
                    if (Vinf.MoreEdge())
                    {
                        Edge Edg1 = Vinf.Edge();
                        // const Draft_EdgeInfo& Einf1 = myEMap(Edg1);
                        Draft_EdgeInfo Einf1 = myEMap.ChangeFromKey(Edg1);
                        Point3D vtori = Tool.Pnt(TVV);
                        // Einf1.Geometry()->D0(Vinf.Parameter(Edg1), pvt);
                        GeomAPI_ProjectPointOnCurve Projector = new(vtori, Einf1.Geometry()!); // patch
                        pvt = Projector.NearestPoint();

                        double dion = pvt.SquareDistance(vtori);
                        Vinf.NextEdge();
                        if (Vinf.MoreEdge())
                        {
                            Edge Edg2 = Vinf.Edge();
                            // const Draft_EdgeInfo& Einf2 = myEMap(Edg2);
                            Draft_EdgeInfo Einf2 = myEMap.ChangeFromKey(Edg2);
                            //	    double f;
                            Einf2.Geometry()!.D0(Vinf.Parameter(Edg2), out Point3D opvt);

                            if (opvt.SquareDistance(vtori) < dion)
                            {
                                pvt = opvt;
                            }
                            // Vinf.ChangeParameter(Edg2) = Parameter(Einf2.Geometry(), pvt);
                            double param2 = Parameter(Einf2.Geometry()!, pvt, out int done2);
                            if (done2 != 0)
                            {
                                Surface S1 = myFMap.FindFromKey(Einf2.FirstFace()).Geometry()!;
                                Surface S2 = myFMap.FindFromKey(Einf2.SecondFace()).Geometry()!;
                                Vinf.ChangeParameter(Edg2) =
                                    SmartParameter(Einf2, Tool.Tolerance(Edg2), pvt, done2, S1, S2);
                            }
                            else
                            {
                                Vinf.ChangeParameter(Edg2) = param2;
                            }
                        }

                        Vinf.ChangeGeometry() = pvt;
                        // Vinf.ChangeParameter(Edg1) = Parameter(Einf1.Geometry(), pvt);
                        double param = Parameter(Einf1.Geometry()!, pvt, out int done);
                        if (done != 0)
                        {
                            Surface S1 = myFMap.FindFromKey(Einf1.FirstFace()).Geometry()!;
                            Surface S2 = myFMap.FindFromKey(Einf1.SecondFace()).Geometry()!;
                            Vinf.ChangeParameter(Edg1) =
                                SmartParameter(Einf1, Tool.Tolerance(Edg1), pvt, done, S1, S2);
                        }
                        else
                        {
                            Vinf.ChangeParameter(Edg1) = param;
                        }
                        continue;
                    }

                    errStat = Draft_VertexRecomputation;
                    badShape = TVV;
                    return;
                }

                IntCurveSurface_HInter myintcs = new();
                myintcs.Perform(HAC, HAS);
                if (!myintcs.IsDone())
                {
                    errStat = Draft_VertexRecomputation;
                    badShape = TVV;
                    return;
                }

                {
                    Point3D vtori = Tool.Pnt(TVV);
                    Point3D pvt;

                    int nbsol = myintcs.NbPoints();
                    if (nbsol <= 0)
                    {
                        ExtCS extr = new(AC, AS, global::Kerf.Engine.Foundation.Precision.PConfusion(), global::Kerf.Engine.Foundation.Precision.PConfusion());

                        if (!extr.IsDone || extr.ExtCount == 0)
                        {
                            errStat = Draft_VertexRecomputation;
                            badShape = TVV;
                            return;
                        }

                        double disref = RealLast();
                        int iref = 0;
                        POnCurv Pc;
                        POnSurf Ps;
                        for (int i = 1; i <= extr.ExtCount; i++)
                        {
                            extr.Points(i, out Pc, out Ps);
                            double distemp = Pc.Value().SquareDistance(vtori);
                            if (distemp < disref)
                            {
                                disref = distemp;
                                iref = i;
                            }
                        }
                        extr.Points(iref, out Pc, out Ps);
                        pvt = Pc.Value();
                    }
                    else
                    {
                        double disref = RealLast();
                        int iref = 0;
                        for (int i = 1; i <= nbsol; i++)
                        {
                            double distemp = myintcs.Point(i).Pnt().SquareDistance(vtori);
                            if (distemp < disref)
                            {
                                disref = distemp;
                                iref = i;
                            }
                        }
                        pvt = myintcs.Point(iref).Pnt();
                    }

                    Vinf.ChangeGeometry() = pvt;

                    for (Vinf.InitEdgeIterator(); Vinf.MoreEdge(); Vinf.NextEdge())
                    {
                        Edge Edg = Vinf.Edge();
                        double initpar = Vinf.Parameter(Edg);
                        // const Draft_EdgeInfo& Einf = myEMap(Edg);
                        Draft_EdgeInfo Einf = myEMap.ChangeFromKey(Edg);
                        // Vinf.ChangeParameter(Edg) = Parameter(Einf.Geometry(),pvt);
                        double param = Parameter(Einf.Geometry()!, pvt, out int done);
                        if (done != 0)
                        {
                            Surface S1 = myFMap.FindFromKey(Einf.FirstFace()).Geometry()!;
                            Surface S2 = myFMap.FindFromKey(Einf.SecondFace()).Geometry()!;
                            Vinf.ChangeParameter(Edg) = SmartParameter(Einf, Tool.Tolerance(Edg), pvt, done, S1, S2);
                        }
                        else
                        {
                            if (Math.Abs(initpar - param) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                Curve3D? aC = Tool.Curve(Edg, out Placement Loc, out double f, out double l);
                                if (aC!.GetType() == typeof(TrimmedCurve3D))
                                {
                                    Einf.SetNewGeometry(true);
                                }
                            }
                            Vinf.ChangeParameter(Edg) = param;
                        }
                    }
                }
            }
        }

        // small loop of validation/protection

        for (int i = 1; i <= myEMap.Extent(); i++)
        {
            Edge edg = ShapeCasts.Edge(myEMap.FindKey(i));

            Exploration.Vertices(edg, out Vertex Vf, out Vertex Vl);
            if (edg.Orientation() == Reversed)
            {
                Vf.Reverse();
                Vl.Reverse();
            }

            if (myVMap.Contains(Vf) && myVMap.Contains(Vl))
            {
                // Here, we compare directions of the source edge (from input shape)
                // and corresponding selected part of the intersection edge.
                // If these directions are opposite then we reverse intersection edge
                // and recompute corresponding vertex-parameters.

                double aParF = myVMap.ChangeFromKey(Vf).Parameter(edg);
                double aParL = myVMap.ChangeFromKey(Vl).Parameter(edg);

                if (aParL < aParF)
                {
                    Draft_EdgeInfo aEinf = myEMap.ChangeFromKey(edg);
                    Curve3D aSCurve = Tool.Curve(edg, out double aF, out double aL)!;
                    Curve3D? anIntCurv = aEinf.Geometry()!;
                    Point3D aPf, aPl;
                    Vector3D aDirNF, aDirNL, aDirOF, aDirOL;
                    aSCurve.D1(Tool.Parameter(Vf, edg), out aPf, out aDirOF);
                    aSCurve.D1(Tool.Parameter(Vl, edg), out aPl, out aDirOL);
                    anIntCurv.D1(aParF, out aPf, out aDirNF);
                    anIntCurv.D1(aParL, out aPl, out aDirNL);

                    double aSqMagn = aDirNF.SquareMagnitude();

                    if (aSqMagn > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        aDirNF.Divide(Math.Sqrt(aSqMagn));
                    }

                    aSqMagn = aDirNL.SquareMagnitude();
                    if (aSqMagn > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        aDirNL.Divide(Math.Sqrt(aSqMagn));
                    }

                    aSqMagn = aDirOF.SquareMagnitude();
                    if (aSqMagn > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        aDirOF.Divide(Math.Sqrt(aSqMagn));
                    }

                    aSqMagn = aDirOL.SquareMagnitude();
                    if (aSqMagn > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        aDirOL.Divide(Math.Sqrt(aSqMagn));
                    }

                    double aCosF = aDirNF.Dot(aDirOF), aCosL = aDirNL.Dot(aDirOL);
                    double aCosMax = Math.Abs(aCosF) > Math.Abs(aCosL) ? aCosF : aCosL;

                    if (aCosMax < 0.0)
                    {
                        reverse(ref anIntCurv);
                        aEinf.ChangeGeometry() = anIntCurv;
                        double aPar = Parameter(aEinf.Geometry()!, aPf, out int anErr);
                        if (anErr == 0)
                        {
                            myVMap.ChangeFromKey(Vf).ChangeParameter(edg) = aPar;
                        }
                        aPar = Parameter(aEinf.Geometry()!, aPl, out anErr);
                        if (anErr == 0)
                        {
                            myVMap.ChangeFromKey(Vl).ChangeParameter(edg) = aPar;
                        }
                    }
                }
            }

            double pf = 0.0, pl = 0.0, tolerance = 0.0;
            if (!NewParameter(Vf, edg, ref pf, ref tolerance))
            {
                pf = Tool.Parameter(Vf, edg);
            }
            if (!NewParameter(Vl, edg, ref pl, ref tolerance))
            {
                pl = Tool.Parameter(Vl, edg);
            }
            if (pl <= pf)
            {
                //      const occ::handle<Geom_Curve> gc=ite.Value().Geometry();
                //      if (!gc.IsNull()) {
                //	pl = gc->LastParameter();
                //	pf = gc->FirstParameter();
                //      }
                Curve3D theCurve = myEMap.FindFromKey(edg).Geometry()!;
                if (theCurve.IsClosed)
                {
                    // pf >= pl
                    double FirstPar = theCurve.FirstParameter(), LastPar = theCurve.LastParameter();
                    double pconf = global::Kerf.Engine.Foundation.Precision.PConfusion();
                    if (Math.Abs(pf - LastPar) <= pconf)
                    {
                        pf = FirstPar;
                    }
                    else if (Math.Abs(pl - FirstPar) <= pconf)
                    {
                        pl = LastPar;
                    }

                    if (pl <= pf)
                    {
                        pl += (LastPar - FirstPar);
                    }
                }
                if (pl <= pf)
                {
                    errStat = Draft_EdgeRecomputation;
                    badShape = edg;
                    return;
                }
            }
            if (myVMap.Contains(Vf))
            {
                myVMap.ChangeFromKey(Vf).ChangeParameter(edg) = pf;
            }
            if (myVMap.Contains(Vl))
            {
                myVMap.ChangeFromKey(Vl).ChangeParameter(edg) = pl;
            }
        }
    }

    //=================================================================================================

    private Surface? NewSurface(Surface S,
                                     Orientation Oris,
                                     in Direction3D Direction,
                                     double Angle,
                                     in Plane3D NeutralPlane)
    {
        Surface? NewS = null;

        Type TypeS = S.GetType();

        if (TypeS == typeof(PlaneSurface))
        {
            Plane3D Pl = ((PlaneSurface)S).Pln();
            if (FindRotation(Pl, Oris, Direction, Angle, NeutralPlane, out Axis3D Axe, out double Theta))
            {
                if (Math.Abs(Theta) > global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    NewS = (Surface)S.Rotated(Axe, Theta);
                }
                else
                {
                    NewS = S;
                }
            }
        }
        else if (TypeS == typeof(CylindricalSurface))
        {
            double testdir = Direction.Dot(NeutralPlane.Axis().Direction());
            if (Math.Abs(testdir) <= 1.0 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                return NewS;
            }
            Cylinder Cy = ((CylindricalSurface)S).Cylinder();
            testdir = Direction.Dot(Cy.Axis().Direction());
            if (Math.Abs(testdir) <= 1.0 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                return NewS;
            }
            if (Math.Abs(Angle) > global::Kerf.Engine.Foundation.Precision.Angular())
            {
                QuadQuadGeo i2s = new();
                i2s.Perform(NeutralPlane, Cy, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
                bool isIntDone = i2s.IsDone;

                if (i2s.TypeInter() == Ellipse)
                {
                    Ellipse3D anEl = i2s.Ellipse(1);
                    double aMajorR = anEl.MajorRadius();
                    double aMinorR = anEl.MinorRadius();
                    isIntDone = (aMajorR < 100000.0 * aMinorR);
                }

                if (!isIntDone || i2s.TypeInter() != Circle)
                {
                    return NewS;
                }
                CoordinateSystem axcone = Cy.Position();
                // Pb : Where is the material???
                double alpha = Angle;
                bool direct = axcone.Direct();
                if ((direct && Oris == Reversed) || (!direct && Oris == Forward))
                {
                    alpha = -alpha;
                }

                Point3D Center = i2s.Circle(1).Location();
                if (testdir < 0.0)
                {
                    alpha = -alpha;
                }
                double Z = global::Kerf.Engine.Geometry.ElCLib.LineParameter(Cy.Axis(), Center);
                double Rad = Cy.Radius() + Z * Math.Tan(alpha);
                if (Rad < 0.0)
                {
                    Rad = -Rad;
                }
                else
                {
                    alpha = -alpha;
                }
                Cone co = new(axcone, alpha, Rad);
                NewS = new ConicalSurface(co);
            }
            else
            {
                NewS = S;
            }
        }
        else if (TypeS == typeof(ConicalSurface))
        {
            double testdir = Direction.Dot(NeutralPlane.Axis().Direction());
            if (Math.Abs(testdir) <= 1.0 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                return NewS;
            }

            Cone Co1 = ((ConicalSurface)S).Cone();

            testdir = Direction.Dot(Co1.Axis().Direction());
            if (Math.Abs(testdir) <= 1.0 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                return NewS;
            }

            QuadQuadGeo i2s = new();
            i2s.Perform(NeutralPlane, Co1, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
            if (!i2s.IsDone || i2s.TypeInter() != Circle)
            {
                return NewS;
            }
            CoordinateSystem axcone = Co1.Position();
            // Pb : Where is the material???
            double alpha = Angle;
            bool direct = axcone.Direct();
            if ((direct && Oris == Reversed) || (!direct && Oris == Forward))
            {
                alpha = -alpha;
            }

            Point3D Center = i2s.Circle(1).Location();
            if (Math.Abs(Angle) > global::Kerf.Engine.Foundation.Precision.Angular())
            {
                if (testdir < 0.0)
                {
                    alpha = -alpha;
                }
                double Z = global::Kerf.Engine.Geometry.ElCLib.LineParameter(Co1.Axis(), Center);
                double Rad = i2s.Circle(1).Radius() + Z * Math.Tan(alpha);
                if (Rad < 0.0)
                {
                    Rad = -Rad;
                }
                else
                {
                    alpha = -alpha;
                }
                if (Math.Abs(alpha - Co1.SemiAngle()) < global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    NewS = S;
                }
                else
                {
                    Cone co = new(axcone, alpha, Rad);
                    NewS = new ConicalSurface(co);
                }
            }
            else
            {
                NewS = new CylindricalSurface(new Cylinder(axcone, i2s.Circle(1).Radius()));
            }
        }
        return NewS;
    }

    //=================================================================================================

    private Curve3D? NewCurve(Curve3D C,
                                 Surface S,
                                 Orientation Oris,
                                 in Direction3D Direction,
                                 double Angle,
                                 in Plane3D NeutralPlane,
                                 bool Flag = true)
    {
        Curve3D? NewC = null;

        Type TypeS = S.GetType();

        if (TypeS == typeof(PlaneSurface))
        {
            Plane3D Pl = ((PlaneSurface)S).Pln();
            if (FindRotation(Pl, Oris, Direction, Angle, NeutralPlane, out Axis3D Axe, out double Theta))
            {
                if (Math.Abs(Theta) > global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    NewC = (Curve3D)C.Rotated(Axe, Theta);
                }
                else
                {
                    NewC = C;
                }
            }
            return NewC;
        }

        if (C.GetType() != typeof(LineCurve3D))
        {
            return NewC;
        }

        Line3D lin = ((LineCurve3D)C).Lin();
        //  double testdir = Direction.Dot(lin.Direction());
        //  if (std::abs(testdir) <= 1.-Precision::Angular()) {
        //    return NewC;
        //  }
        Direction3D Norm = new();
        if (TypeS == typeof(CylindricalSurface))
        {
            Cylinder Cy = ((CylindricalSurface)S).Cylinder();
            global::Kerf.Engine.Geometry.ElSLib.Parameters(Cy, lin.Location(), out double U, out double V);
            global::Kerf.Engine.Geometry.ElSLib.D1(U, V, Cy, out Point3D pbid, out Vector3D d1u, out Vector3D d1v);
            Norm = new Direction3D(d1u.Crossed(d1v));
        }
        else if (TypeS == typeof(ConicalSurface))
        {
            Cone Co = ((ConicalSurface)S).Cone();
            global::Kerf.Engine.Geometry.ElSLib.Parameters(Co, lin.Location(), out double U, out double V);
            global::Kerf.Engine.Geometry.ElSLib.D1(U, V, Co, out Point3D pbid, out Vector3D d1u, out Vector3D d1v);
            Norm = new Direction3D(d1u.Crossed(d1v));
        }

        IntConicQuad ilipl = new(lin, NeutralPlane, global::Kerf.Engine.Foundation.Precision.Angular());
        if (ilipl.IsDone && ilipl.PointCount != 0)
        {
            if (Oris == Reversed)
            {
                Norm.Reverse();
            }
            Axis3D axrot = new(ilipl.Point(1), Norm.Crossed(Direction));
            Line3D lires = new Line3D(new Axis3D(ilipl.Point(1), Direction)).Rotated(axrot, Angle);
            if (lires.Direction().Dot(lin.Direction()) < 0.0)
            {
                lires.Reverse();
            }
            NewC = new LineCurve3D(lires);
        }
        return NewC;
    }

    //=================================================================================================

    private static bool Choose(NCollection_IndexedDataMap<Shape, Draft_FaceInfo> theFMap,
                               NCollection_IndexedDataMap<Shape, Draft_EdgeInfo> theEMap,
                               in Vertex Vtx,
                               Draft_VertexInfo Vinf,
                               CurveAdaptor AC,
                               SurfaceAdaptor AS)
    {
        Vector3D tgref;
        Vinf.InitEdgeIterator();

        // Find a regular edge with null SecondFace
        while (Vinf.MoreEdge())
        {
            Edge E1 = Vinf.Edge();
            Draft_EdgeInfo Einf1 = theEMap.FindFromKey(E1);
            if (Einf1.SecondFace().IsNull)
            {
                break;
            }
            else
            {
                Continuity te = Tool.Continuity(E1, Einf1.FirstFace(), Einf1.SecondFace());
                if (te >= G1)
                {
                    break;
                }
            }
            Vinf.NextEdge();
        }
        if (!Vinf.MoreEdge())
        { // take the first edge
            Vinf.InitEdgeIterator();
        }

        Edge Eref = Vinf.Edge();
        // const Draft_EdgeInfo& Einf = theEMap(Eref);
        Draft_EdgeInfo Einf = theEMap.ChangeFromKey(Eref);

        AC.Load(Einf.Geometry()!);

        double prm;
        Curve3D C = Tool.Curve(Eref, out Placement Loc, out double f, out double l)!;
        C = (Curve3D)C.Transformed(Loc.Transformation());
        // prm = Parameter(C,BRep_Tool::Pnt(Vtx));
        double param = Parameter(C, Tool.Pnt(Vtx), out int done);
        if (done != 0)
        {
            Surface S1 = theFMap.FindFromKey(Einf.FirstFace()).Geometry()!;
            Surface S2 = theFMap.FindFromKey(Einf.SecondFace()).Geometry()!;
            prm = SmartParameter(Einf, Tool.Tolerance(Eref), Tool.Pnt(Vtx), done, S1, S2);
        }
        else
        {
            prm = param;
        }
        C.D1(prm, out Point3D ptbid, out tgref);

        Vinf.InitEdgeIterator();
        while (Vinf.MoreEdge())
        {
            // Find a non tangent edge
            Edge Edg = Vinf.Edge();
            if (!Edg.IsSame(Eref))
            {
                // const Draft_EdgeInfo& Einfo = theEMap(Edg);
                Draft_EdgeInfo Einfo = theEMap.ChangeFromKey(Edg);
                if (!Einfo.SecondFace().IsNull
                    && Tool.Continuity(Edg, Einfo.FirstFace(), Einfo.SecondFace()) <= C0)
                {
                    C = Tool.Curve(Edg, out Loc, out f, out l)!;
                    C = (Curve3D)C.Transformed(Loc.Transformation());
                    // prm = Parameter(C,BRep_Tool::Pnt(Vtx));
                    double anewparam = Parameter(C, Tool.Pnt(Vtx), out int anewdone);
                    if (anewdone != 0)
                    {
                        Surface S1 = theFMap.FindFromKey(Einfo.FirstFace()).Geometry()!;
                        Surface S2 = theFMap.FindFromKey(Einfo.SecondFace()).Geometry()!;
                        prm = SmartParameter(Einfo, Tool.Tolerance(Edg), Tool.Pnt(Vtx), anewdone, S1, S2);
                    }
                    else
                    {
                        prm = anewparam;
                    }
                    C.D1(prm, out ptbid, out Vector3D tg);
                    if (tg.CrossMagnitude(tgref) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        break;
                    }
                }
            }
            Vinf.NextEdge();
        }
        if (!Vinf.MoreEdge())
        {
            return false;
        }

        Draft_EdgeInfo Einf2 = theEMap.FindFromKey(Vinf.Edge());
        if (!Einf.SecondFace().IsNull)
        {
            if (Einf2.FirstFace().IsSame(Einf.FirstFace()) || Einf2.FirstFace().IsSame(Einf.SecondFace()))
            {
                AS.Load(theFMap.FindFromKey(Einf2.SecondFace()).Geometry()!);
            }
            else
            {
                AS.Load(theFMap.FindFromKey(Einf2.FirstFace()).Geometry()!);
            }
        }
        else
        {
            if (Einf2.FirstFace().IsSame(Einf.FirstFace()))
            {
                AS.Load(theFMap.FindFromKey(Einf2.SecondFace()).Geometry()!);
            }
            else
            {
                AS.Load(theFMap.FindFromKey(Einf2.FirstFace()).Geometry()!);
            }
        }
        return true;
    }

    //=================================================================================================

    private static double Parameter(Curve3D C, in Point3D P, out int done)
    {
        done = 0;
        Curve3D cbase = C;
        Type ctyp = C.GetType();
        if (ctyp == typeof(TrimmedCurve3D))
        {
            cbase = ((TrimmedCurve3D)C).BasisCurve();
            ctyp = cbase.GetType();
        }
        double param;
        if (ctyp == typeof(LineCurve3D))
        {
            param = global::Kerf.Engine.Geometry.ElCLib.Parameter(((LineCurve3D)cbase).Lin(), P);
        }
        else if (ctyp == typeof(CircleCurve3D))
        {
            param = global::Kerf.Engine.Geometry.ElCLib.Parameter(((CircleCurve3D)cbase).Circ(), P);
            if (Math.Abs(2.0 * M_PI - param) <= Epsilon(2.0 * M_PI))
            {
                param = 0.0;
            }
        }
        else if (ctyp == typeof(EllipseCurve3D))
        {
            param = global::Kerf.Engine.Geometry.ElCLib.Parameter(((EllipseCurve3D)cbase).Elips(), P);
            if (Math.Abs(2.0 * M_PI - param) <= Epsilon(2.0 * M_PI))
            {
                param = 0.0;
            }
        }
        else if (ctyp == typeof(ParabolaCurve3D))
        {
            param = global::Kerf.Engine.Geometry.ElCLib.Parameter(((ParabolaCurve3D)cbase).Parab(), P);
        }
        else if (ctyp == typeof(HyperbolaCurve3D))
        {
            param = global::Kerf.Engine.Geometry.ElCLib.Parameter(((HyperbolaCurve3D)cbase).Hypr(), P);
        }
        else
        {
            CurveAdaptor TheCurve = new(C);
            CurvePointExtrema myExtPC = new(P, TheCurve);
            if (!myExtPC.IsDone)
            {
                throw new FailureException("Draft_Modification_1::Parameter: ExtremaPC not done.");
            }
            if (myExtPC.ExtCount >= 1)
            {
                double Dist2, Dist2Min = myExtPC.SquareDistance(1);
                int j, jmin = 1;
                for (j = 2; j <= myExtPC.ExtCount; j++)
                {
                    Dist2 = myExtPC.SquareDistance(j);
                    if (Dist2 < Dist2Min)
                    {
                        Dist2Min = Dist2;
                        jmin = j;
                    }
                }
                param = myExtPC.Point(jmin).Parameter();
            }
            else
            {
                myExtPC.TrimmedSquareDistances(out double dist1_2, out double dist2_2, out Point3D _, out Point3D _);
                if (dist1_2 < dist2_2)
                {
                    done = -1;
                    param = TheCurve.FirstParameter();
                }
                else
                {
                    done = 1;
                    param = TheCurve.LastParameter();
                }
            }

            if (cbase.IsPeriodic)
            {
                double Per = cbase.Period();
                double Tolp = global::Kerf.Engine.Foundation.Precision.Parametric(global::Kerf.Engine.Foundation.Precision.Confusion());
                if (Math.Abs(Per - param) <= Tolp)
                {
                    param = 0.0;
                }
            }
        }
        return param;
    }

    //=================================================================================================

    private static double SmartParameter(Draft_EdgeInfo Einf,
                                         double EdgeTol,
                                         in Point3D Pnt,
                                         int sign,
                                         Surface S1,
                                         Surface S2)
    {
        Curve2D NewC2d;
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        double Etol = EdgeTol;

        Curve2D? pcu1 = Einf.FirstPC();
        Curve2D? pcu2 = Einf.SecondPC();

        if (pcu1 is null)
        {
            Curve3D theCurve = Einf.Geometry()!;
            pcu1 = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(theCurve, theCurve.FirstParameter(), theCurve.LastParameter(), S1, ref Etol);
            Einf.ChangeFirstPC() = pcu1;
        }
        if (pcu2 is null)
        {
            Curve3D theCurve = Einf.Geometry()!;
            pcu2 = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(theCurve, theCurve.FirstParameter(), theCurve.LastParameter(), S2, ref Etol);
            Einf.ChangeSecondPC() = pcu2;
        }

        GeomAPI_ProjectPointOnSurf Projector = new(Pnt, S1);
        Projector.LowerDistanceParameters(out double U, out double V);

        // OCCT dereferences the projection without a check (null if it failed).
        NewC2d = Einf.FirstPC()!;
        if (NewC2d.GetType() == typeof(TrimmedCurve2D))
        {
            NewC2d = ((TrimmedCurve2D)NewC2d).BasisCurve();
        }

        Point2D P2d = new(U, V);
        Geom2dAPI_ProjectPointOnCurve Projector2d = new(P2d, NewC2d);
        if (Projector2d.NbPoints() == 0 || Projector2d.LowerDistance() > Tol)
        {
            BSplineCurve2D BCurve;
            if (NewC2d.GetType() != typeof(BSplineCurve2D))
            {
                BCurve = Geom2dConvert2D.CurveToBSplineCurve(NewC2d);
            }
            else
            {
                BCurve = (BSplineCurve2D)NewC2d;
            }
            if (sign == -1)
            {
                NCollection_Array1<Point2D> PntArray = new(1, 2);
                PntArray[1] = P2d;
                PntArray[2] = BCurve.Pole(1);
                BezierCurve2D Patch = new(PntArray);
                CompCurveToBSplineCurve2D Concat = new(BCurve, QuasiAngular);
                Concat.Add(Patch, Tol, false);
                BCurve = Concat.BSplineCurve()!;
            }
            else
            {
                NCollection_Array1<Point2D> PntArray = new(1, 2);
                PntArray[1] = BCurve.Pole(BCurve.PoleCount);
                PntArray[2] = P2d;
                BezierCurve2D Patch = new(PntArray);
                CompCurveToBSplineCurve2D Concat = new(BCurve, QuasiAngular);
                Concat.Add(Patch, Tol, true);
                BCurve = Concat.BSplineCurve()!;
            }
            NewC2d = BCurve;
        }
        Einf.ChangeFirstPC() = NewC2d;
        CurveAdaptor2D hcur = new(NewC2d);
        SurfaceAdaptor hsur = new(S1);
        CurveOnSurface cons = new(hcur, hsur);
        CurveOnSurface hcons = cons; // OCCT copies the adaptor into a handle: one object here
        SurfaceAdaptor hsur2 = new(S2);
        CompProjectedCurve HProjector = new(hsur2, hcons, Tol, Tol);
        HProjector.Bounds(1, out double Udeb, out double Ufin);
        int MaxSeg = 20 + HProjector.NbIntervals(C3);
        CurveOnSurfaceApproximation appr = new(HProjector, hsur2, Udeb, Ufin, Tol);
        appr.Perform(MaxSeg, 10, C1, false, false);
        Einf.ChangeSecondPC() = appr.Curve2d();
        Einf.ChangeGeometry() = appr.Curve3d();
        Einf.SetNewGeometry(true);

        if (sign == -1)
        {
            return Einf.Geometry()!.FirstParameter();
        }
        else
        {
            return Einf.Geometry()!.LastParameter();
        }
    }

    //=================================================================================================

    private static Orientation Orientation(in Shape S, in Face F)
    {
        //
        // change porting NT
        //
        Explorer expl = new();
        expl.Init(S, ShapeType.Face);
        while (expl.More())
        {
            if (ShapeCasts.Face(expl.Current()).IsSame(F))
            {
                return expl.Current().Orientation();
            }
            expl.Next();
        }
        return Forward;
    }

    //=================================================================================================

    private static bool FindRotation(in Plane3D Pl,
                                     Orientation Oris,
                                     in Direction3D Direction,
                                     double Angle,
                                     in Plane3D NeutralPlane,
                                     out Axis3D Axe,
                                     out double theta)
    {
        Axe = new Axis3D();
        theta = 0.0;
        QuadQuadGeo i2pl = new(Pl, NeutralPlane, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());

        if (i2pl.IsDone && i2pl.TypeInter() == Line)
        {
            Line3D li = i2pl.Line(1);
            // Try to turn around this line
            Direction3D nx = li.Direction();
            Direction3D ny = Pl.Axis().Direction().Crossed(nx);
            double a = Direction.Dot(nx);
            if (Math.Abs(a) <= 1 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                double b = Direction.Dot(ny);
                double c = Direction.Dot(Pl.Axis().Direction());
                bool direct = Pl.Position().Direct();
                if ((direct && Oris == Reversed) || (!direct && Oris == Forward))
                {
                    b = -b;
                    c = -c;
                }
                double denom = Math.Sqrt(1 - a * a);
                double Sina = Math.Sin(Angle);
                if (denom > Math.Abs(Sina))
                {
                    double phi = Math.Atan2(b / denom, c / denom);
                    double theta0 = Math.Acos(Sina / denom);
                    theta = theta0 - phi;
                    if (Math.Cos(theta) < 0.0)
                    {
                        theta = -theta0 - phi;
                    }
                    //  modified by NIZHNY-EAP Tue Nov 16 15:51:38 1999 ___BEGIN___
                    while (Math.Abs(theta) > M_PI)
                    {
                        theta = theta + M_PI * (theta < 0 ? 1 : -1);
                    }
                    //  modified by NIZHNY-EAP Tue Nov 16 15:53:32 1999 ___END___
                    Axe = li.Position();
                    return true;
                }
            }
        }
        return false;
    }

    //=================================================================================================

    // OCCT.NET: OCCT reverses curves in place (the intersection's curves, its pcurves, the curves kept for an edge);
    // a frozen one (the intersection's are) is reversed on a copy, which takes its place.
    private static void reverse(ref Curve3D? theCurve)
    {
        if (theCurve!.IsFrozen)
        {
            theCurve = (Curve3D)theCurve.Copy();
        }
        theCurve.Reverse();
    }

    private static void reverse(ref Curve2D? theCurve)
    {
        if (theCurve!.IsFrozen)
        {
            theCurve = (Curve2D)theCurve.Copy();
        }
        theCurve.Reverse();
    }

    // OCCT's occ::down_cast followed by a dereference: a null handle there is an access violation, a
    // NullReferenceException here.
    private static T downCast<T>(Geometry3D? theGeom) where T : Geometry3D
    {
        return theGeom as T ?? throw new NullReferenceException();
    }
}
