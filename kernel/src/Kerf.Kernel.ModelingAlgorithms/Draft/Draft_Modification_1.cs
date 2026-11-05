// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_Modification_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Intersection;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Surfaces;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Engine.Features.Draft.DraftErrorStatus;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.ResultType;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features.Draft;

// OCCT_DEBUG blocks (the traces of the vertex recomputation and of NewSurface's failures) are left out.

internal sealed partial class DraftModification
{
    //=================================================================================================

    private bool InternalAdd(in Face F, in Direction3D direction, double angle, in Plane3D neutralPlane, bool flag = true)
    {
        if (_fMap.ContainsKey(F))
        {
            return (_badShape.IsNull);
        }

        Orientation oris = Orientation(_shape, F);
        Placement lo;
        // gp_Dir NewDirection = Direction;
        // double NewAngle = Angle;
        Surface s = Tool.Surface(F, out lo)!;
        s = (Surface)s.Transformed(lo.Transformation);
        if (s.GetType() == typeof(TrimmedSurface))
        {
            s = ((TrimmedSurface)s).BasisSurface;
        }
        Surface? newS = null;
        CircleCurve3D? theCircle = null;

        bool postponed = (!flag);
        if (postponed)
        {
            Type typS = s.GetType();
            if (typS == typeof(CylindricalSurface) || typS == typeof(ExtrusionSurface))
            {
                Circle3D cir;
                if (typS == typeof(CylindricalSurface))
                {
                    Cylinder3D cyl = ((CylindricalSurface)s).Cylinder;
                    Axis3D axcyl = cyl.Axis;
                    cir = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(cyl.Position, cyl.Radius, 0.0);
                    Vector3D vv = new(cyl.Location, neutralPlane.Location);
                    cir = cir.Translated(vv.Dot(new Vector3D(axcyl.Direction)) * new Vector3D(axcyl.Direction));
                }
                else
                {
                    Curve3D cbas = ((ExtrusionSurface)s).BasisCurve;
                    Direction3D theDirextr = ((ExtrusionSurface)s).Direction;

                    if (cbas is TrimmedCurve3D trimmed)
                    {
                        cbas = trimmed.BasisCurve;
                    }
                    if (cbas is CircleCurve3D circle)
                    {
                        cir = circle.Circle;
                        Direction3D dircir = cir.Axis.Direction;
                        if (!direction.IsParallel(dircir, global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            _badShape = F;
                            _errStat = DraftErrorStatus.FaceRecomputation;
                            return false;
                        }
                    }
                    else
                    {
                        _badShape = F;
                        _errStat = DraftErrorStatus.FaceRecomputation;
                        return false;
                    }

                    CoordinateSystem axis = neutralPlane.Position;
                    double L = new Vector3D(cir.Location, axis.Location).Dot(new Vector3D(axis.Direction));
                    double cos = theDirextr.Dot(axis.Direction);
                    Vector3D vv = (L / cos) * new Vector3D(theDirextr);
                    cir = cir.Translated(vv);
                }

                theCircle = new CircleCurve3D(cir);
            }
            else
            {
                postponed = false;
            }
        }

        if (!postponed)
        {
            newS = NewSurface(s, oris, direction, angle, neutralPlane);
            if (newS is null)
            {
                _badShape = F;
                _errStat = DraftErrorStatus.FaceRecomputation;
                return false;
            }
            // To avoid some problems with infinite restrictions
            Type typs = newS.GetType();
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
                        Cone3D co = ((ConicalSurface)newS).Cone;
                        double vapex = -co.RefRadius / Math.Sin(co.SemiAngle);
                        if (vmin < vapex)
                        { // vmax should not exceed Vapex
                            if (vmax + deltav > vapex)
                            {
                                vmax = vapex;
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
                            if (vmin - deltav < vapex)
                            {
                                vmin = vapex;
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
                    newS = new TrimmedSurface(newS, 0.0, 2.0 * M_PI, vmin, vmax);
                }
            }
        }

        if (postponed || s != newS)
        {
            DraftFaceInfo fi = new(newS, true);
            fi.RootFace(_curFace);
            _fMap.Add(F, fi);
            if (postponed)
            {
                _fMap.GetValue(F).ChangeCurve() = theCircle;
            }
        }

        Explorer exp = new(F, ShapeType.Edge);
        HashSet<Shape> mapOfE = new(ShapeHasher.Instance);
        while (exp.More() && _badShape.IsNull)
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            if (!_eMap.ContainsKey(edg))
            {
                bool addedg = false;
                bool addface = false;
                Face otherF = new();
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
                    for (NCollection_List<Shape>.Iterator it = new(_efMap.GetValue(edg)); it.More(); it.Next())
                    {
                        if (!it.Value().IsSame(F))
                        {
                            if (otherF.IsNull)
                            {
                                otherF = ShapeCasts.Face(it.Value());
                            }
                            nbother++;
                        }
                    }
                    if (nbother >= 2)
                    {
                        _badShape = edg;
                        _errStat = DraftErrorStatus.EdgeRecomputation;
                    }
                    else if (!otherF.IsNull && Tool.Continuity(edg, F, otherF) >= G1)
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
                        _fMap.GetValue(F).Add(otherF);
                    }
                    Curve3D c = Tool.Curve(edg, out Placement L, out double f, out double l)!;
                    c = (Curve3D)c.Transformed(L.Transformation);
                    if (c.GetType() == typeof(TrimmedCurve3D))
                    {
                        c = ((TrimmedCurve3D)c).BasisCurve;
                    }
                    Curve3D? newC = null;
                    DraftEdgeInfo eInf = new(true);
                    if (postponed)
                    {
                        eInf.Add(F);
                        eInf.Add(otherF);

                        // find fixed point
                        if (c is not LineCurve3D localGeom)
                        {
                            _badShape = edg;
                            _errStat = DraftErrorStatus.EdgeRecomputation;
                        }
                        else
                        {
                            Line3D lin = localGeom.Line;
                            IntConicQuad ilipl = new(lin, neutralPlane, global::Kerf.Engine.Foundation.Precision.Angular());
                            if (ilipl.IsDone && ilipl.PointCount != 0)
                            {
                                eInf.Tangent(ilipl.Point(1));
                            }
                            else
                            {
                                _badShape = edg;
                                _errStat = DraftErrorStatus.EdgeRecomputation;
                            }
                        }
                    }
                    else
                    {
                        newC = NewCurve(c, s, oris, direction, angle, neutralPlane, flag);
                        if (newC is null)
                        {
                            _badShape = edg;
                            _errStat = DraftErrorStatus.EdgeRecomputation;
                        }
                    }

                    if (newC is TrimmedCurve3D t)
                    {
                        newC = t.BasisCurve;
                    }
                    eInf.ChangeGeometry() = newC;

                    eInf.RootFace(_curFace);
                    _eMap.Add(edg, eInf);
                    mapOfE.Add(edg);
                    if (addface)
                    {
                        bool fl = flag;
                        Surface alocalSurface = Tool.Surface(otherF, out lo)!;
                        if (alocalSurface.GetType() == typeof(TrimmedSurface))
                        {
                            alocalSurface = ((TrimmedSurface)alocalSurface).BasisSurface;
                        }
                        Type typS = alocalSurface.GetType();
                        if (typS == typeof(CylindricalSurface) || typS == typeof(ExtrusionSurface))
                        {
                            if (_fMap.ContainsKey(F))
                            {
                                if (!flag && !postponed)
                                {
                                    _fMap.Remove(F);
                                    foreach (Shape key in mapOfE)
                                    {
                                        _eMap.Remove(ShapeCasts.Edge(key));
                                    }
                                }
                            }
                        }
                        InternalAdd(otherF, direction, angle, neutralPlane, fl);
                    }
                }
            }
            exp.Next();
        }
        return (_badShape.IsNull);
    }

    //=================================================================================================

    private bool Propagate()
    {
        if (!_badShape.IsNull)
        {
            return false;
        }

        // Set all edges and vertices of modified faces
        Face F;
        Edge e;
        Vertex v;
        Explorer editer = new();
        Explorer vtiter = new();

        for (int i = 0; i < _fMap.Count; i++)
        {
            Face fc = ShapeCasts.Face(_fMap.KeyAt(i));

            // Exploration of the edges of the face
            editer.Init(fc, ShapeType.Edge);
            while (editer.More())
            {
                e = ShapeCasts.Edge(editer.Current());

                if (!_eMap.ContainsKey(e))
                {
                    DraftEdgeInfo eInf = new(true);
                    _eMap.Add(e, eInf);
                }
                _eMap.GetValue(e).Add(fc);

                // Exploration of the vertices of the edge
                vtiter.Init(e, ShapeType.Vertex);
                while (vtiter.More())
                {
                    v = ShapeCasts.Vertex(vtiter.Current());
                    if (!_vMap.ContainsKey(v))
                    {
                        VertexInfo vInf = new();
                        _vMap.Add(v, vInf);
                    }

                    _vMap.GetValue(v).Add(e);
                    _vMap.GetValue(v).ChangeParameter(e) = Tool.Parameter(v, e);
                    vtiter.Next();
                }
                editer.Next();
            }
        }

        Explorer anc = new();
        bool found;

        // Set edges containing modified vertices.

        for (int i = 0; i < _vMap.Count; i++)
        {
            Vertex vt = ShapeCasts.Vertex(_vMap.KeyAt(i));

            // Exploration of the ancestors of the vertex
            anc.Init(_shape, ShapeType.Edge);

            while (anc.More())
            {
                e = ShapeCasts.Edge(anc.Current());
                vtiter.Init(e, ShapeType.Vertex);
                found = false;
                while (vtiter.More())
                {
                    if (vt.IsSame(ShapeCasts.Vertex(vtiter.Current())))
                    {
                        found = true;
                        break;
                    }
                    vtiter.Next();
                }
                if (found)
                {
                    if (!_eMap.ContainsKey(e))
                    {
                        DraftEdgeInfo eInf = new(false);
                        _eMap.Add(e, eInf);
                    }
                    _vMap.GetValue(vt).Add(e);
                    _vMap.GetValue(vt).ChangeParameter(e) = Tool.Parameter(vt, e);
                }
                anc.Next();
            }
        }

        // Set faces containing modified edges
        for (int i = 0; i < _eMap.Count; i++)
        {
            Edge ed = ShapeCasts.Edge(_eMap.KeyAt(i));
            for (NCollection_List<Shape>.Iterator it = new(_efMap.GetValue(ed)); it.More(); it.Next())
            {
                F = ShapeCasts.Face(it.Value());
                if (!_fMap.ContainsKey(F))
                {
                    Surface s = Tool.Surface(F, out Placement L)!;
                    Surface newS = (Surface)s.Transformed(L.Transformation);

                    Type typs = s.GetType();
                    if (typs == typeof(CylindricalSurface) || typs == typeof(ConicalSurface))
                    {
                        BrepTools.UVBounds(F, out double umin, out double umax, out double vmin, out double vmax);
                        if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vmin) && !global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vmax))
                        {
                            double deltav = 10.0 * (vmax - vmin);
                            vmin = vmin - deltav;
                            vmax = vmax + deltav;
                            newS = new TrimmedSurface(newS, 0.0, 2.0 * M_PI, vmin, vmax);
                        }
                    }

                    DraftFaceInfo fInf = new(newS, false);
                    _fMap.Add(F, fInf);
                }
                _eMap.GetValue(ed).Add(F);
            }
        }

        //  Try to add faces for free borders...
        // JAG 09.11.95
        for (int i = 0; i < _eMap.Count; i++)
        {
            DraftEdgeInfo einf = _eMap.ValueAt(i);
            if (einf.NewGeometry() && einf.Geometry() is null && einf.SecondFace().IsNull)
            {
                Placement loc;
                Surface s1 = Tool.Surface(einf.FirstFace(), out loc)!;
                s1 = (Surface)s1.Transformed(loc.Transformation);
                Surface? s2 = null;

                double f, l;
                Edge ek = ShapeCasts.Edge(_eMap.KeyAt(i));
                Curve3D c = Tool.Curve(ek, out loc, out f, out l)!;
                c = (Curve3D)c.Transformed(loc.Transformation);
                if (c.GetType() == typeof(TrimmedCurve3D))
                {
                    c = ((TrimmedCurve3D)c).BasisCurve;
                }
                if (s1 is not PlaneSurface)
                {
                    if (c is ConicCurve3D conic)
                    {
                        CoordinateSystem thePl = new(conic.Position);
                        s2 = new PlaneSurface(thePl);
                    }
                    else if (c.GetType() == typeof(LineCurve3D))
                    {
                        Axis3D axis;
                        if (s1.GetType() == typeof(TrimmedSurface))
                        {
                            axis = ((ElementarySurface)((TrimmedSurface)s1).BasisSurface).Axis;
                        }
                        else
                        {
                            axis = ((ElementarySurface)s1).Axis;
                        }
                        Vector3D they = new(axis.Location, c.Value(0.0));
                        Direction3D axz = new(new Vector3D(axis.Direction).Crossed(they));
                        s2 = new PlaneSurface(new CoordinateSystem(axis.Location, axz, axis.Direction));
                    }
                    else
                    {
                        _badShape = ek;
                        _errStat = DraftErrorStatus.EdgeRecomputation;
                        break; // leave from for
                    }
                }
                else
                { // on the plane
                    for (int j = 0; j < _vMap.Count; j++)
                    {
                        VertexInfo vinf = _vMap.ValueAt(j);
                        for (vinf.InitEdgeIterator(); vinf.MoreEdge(); vinf.NextEdge())
                        {
                            if (vinf.Edge().IsSame(ek))
                            {
                                break;
                            }
                        }
                        if (vinf.MoreEdge())
                        {
                            for (vinf.InitEdgeIterator(); vinf.MoreEdge(); vinf.NextEdge())
                            {
                                Edge edg = vinf.Edge();
                                if (!edg.IsSame(ek))
                                {
                                    DraftEdgeInfo ei = _eMap.GetValue(edg);
                                    if (!ei.FirstFace().IsSame(einf.FirstFace())
                                        && (ei.SecondFace().IsNull || !ei.SecondFace().IsSame(einf.FirstFace())))
                                    {
                                        break;
                                    }
                                }
                            }
                            if (vinf.MoreEdge())
                            {
                                Curve3D c2 = Tool.Curve(vinf.Edge(), out loc, out f, out l)!;
                                CurveAdaptor hCur;
                                Vector3D direc;
                                c2 = (Curve3D)c2.Transformed(loc.Transformation);
                                if (c2.GetType() == typeof(TrimmedCurve3D))
                                {
                                    c2 = ((TrimmedCurve3D)c2).BasisCurve;
                                }
                                if (c.GetType() == typeof(LineCurve3D))
                                {
                                    direc = new Vector3D(((LineCurve3D)c).Line.Direction);
                                    hCur = new CurveAdaptor(c2);
                                }
                                else if (c2.GetType() == typeof(LineCurve3D))
                                {
                                    direc = new Vector3D(((LineCurve3D)c2).Line.Direction);
                                    hCur = new CurveAdaptor(c);
                                }
                                else
                                {
                                    _badShape = ek;
                                    _errStat = DraftErrorStatus.EdgeRecomputation;
                                    break; // leave from while
                                }
                                SurfaceOfLinearExtrusion sle = new(hCur, new Direction3D(direc));
                                switch (sle.GetType())
                                {
                                    case Plane:
                                        s2 = new PlaneSurface(sle.Plane());
                                        break;
                                    case SurfaceKind.Cylinder:
                                        s2 = new CylindricalSurface(sle.Cylinder());
                                        break;
                                    default:
                                        s2 = new ExtrusionSurface(hCur.Curve()!, new Direction3D(direc));
                                        break;
                                }
                            }
                            else
                            {
                                _badShape = ek;
                                _errStat = DraftErrorStatus.EdgeRecomputation;
                                break; // leave from while
                            }
                            break;
                        }
                        // j++;
                    }
                }

                if (_badShape.IsNull)
                {
                    BrepBuilder b = new();
                    Face theNewFace = new();
                    b.MakeFace(ref theNewFace, s2, global::Kerf.Engine.Foundation.Precision.Confusion());
                    einf.Add(theNewFace);
                    DraftFaceInfo fi = new(s2, false);
                    _fMap.Add(theNewFace, fi);
                }
                else
                {
                    break; // leave from for
                }
                // Fin JAG 09.11.95
            }
        }
        return (_badShape.IsNull);
    }

    //=================================================================================================

    /// <summary>
    /// Performs the draft: computes the new faces, edges and vertices. On an error <see cref="IsDone"/> is false,
    /// <see cref="Error"/> tells what failed and <see cref="ProblematicShape"/> on which shape.
    /// </summary>
    /// <exception cref="ConstructionErrorException">An Add failed and the bad face wasn't removed.</exception>
    public void Perform()
    {
        if (!_badShape.IsNull)
        {
            throw new ConstructionErrorException();
        }

        if (!_comp)
        {
            _comp = true;
            if (!Propagate())
            {
                return;
            }

            // Calculate eventual faces

            for (int i = 0; i < _fMap.Count; i++)
            {
                Face fk = ShapeCasts.Face(_fMap.KeyAt(i));
                DraftFaceInfo finf = _fMap.ValueAt(i);
                if (finf.NewGeometry() && finf.Geometry() is null)
                {
                    Face f1 = finf.FirstFace();
                    Face f2 = finf.SecondFace();

                    if (f1.IsNull || f2.IsNull)
                    {
                        _errStat = DraftErrorStatus.FaceRecomputation;
                        _badShape = fk;
                        return;
                    }
                    Surface? s1 = _fMap.GetValue(f1).Geometry();
                    Surface? s2 = _fMap.GetValue(f2).Geometry();
                    if (s1 is null || s2 is null)
                    {
                        _errStat = DraftErrorStatus.FaceRecomputation;
                        _badShape = fk;
                        return;
                    }
                    if (s1.GetType() == typeof(TrimmedSurface))
                    {
                        s1 = ((TrimmedSurface)s1).BasisSurface;
                    }
                    if (s2.GetType() == typeof(TrimmedSurface))
                    {
                        s2 = ((TrimmedSurface)s2).BasisSurface;
                    }
                    PlaneSurface? p1 = s1 as PlaneSurface;
                    PlaneSurface? p2 = s2 as PlaneSurface;
                    if (p1 is null || p2 is null)
                    {
                        _errStat = DraftErrorStatus.FaceRecomputation;
                        _badShape = fk;
                        return;
                    }
                    Plane3D pp1 = p1.Plane;
                    Plane3D pp2 = p2.Plane;
                    QuadQuadGeo i2p = new(pp1, pp2, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
                    if (!i2p.IsDone || i2p.TypeInter() != Line)
                    {
                        _errStat = DraftErrorStatus.FaceRecomputation;
                        _badShape = fk;
                        return;
                    }

                    Direction3D extrdir = i2p.Line(1).Direction;

                    // Preserve the same direction as the base face
                    Surface refSurf = Tool.Surface(fk)!;
                    if (refSurf.GetType() == typeof(TrimmedSurface))
                    {
                        refSurf = ((TrimmedSurface)refSurf).BasisSurface;
                    }
                    Direction3D dirRef = new();

                    if (refSurf.GetType() == typeof(CylindricalSurface))
                    {
                        CoordinateSystem axeRef = ((CylindricalSurface)refSurf).Cylinder.Position;
                        dirRef = axeRef.Direction;
                    }
                    else if (refSurf.GetType() == typeof(ExtrusionSurface))
                    {
                        dirRef = ((ExtrusionSurface)refSurf).Direction;
                    }

                    if (extrdir.Dot(dirRef) < 0.0)
                    {
                        extrdir = extrdir.Reversed();
                    }

                    // it is possible to accelerate speed by storing the info during
                    // InternalAdd --> modification of FaceInfo to preserve the circle

                    // OCCT down-casts the curve without a check; a null circle throws in the extrusion's constructor.
                    CircleCurve3D cCir = (CircleCurve3D)finf.Curve()!;
                    Surface newS = new ExtrusionSurface(cCir, extrdir);

                    BrepTools.UVBounds(fk, out double umin, out double umax, out double vmin, out double vmax);
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

                    newS = new TrimmedSurface(newS, 0.0, 1.9 * M_PI, vmin, vmax);
                    finf.ChangeGeometry() = newS;
                }
            }

            // Calculate new edges.
            for (int ii = 0; ii < _eMap.Count; ii++)
            {
                DraftEdgeInfo Einf = _eMap.ValueAt(ii);

                Edge theEdge = ShapeCasts.Edge(_eMap.KeyAt(ii));

                Surface? s1, s2;
                Curve3D C;
                Curve3D? newC = null;
                C = Tool.Curve(theEdge, out Placement L, out double f, out double l)!;
                C = (Curve3D)C.Transformed(L.Transformation);

                if (Einf.NewGeometry() && Einf.Geometry() is null)
                {
                    if (!Einf.IsTangent(out Point3D ptfixe))
                    {
                        Face firstFace = Einf.FirstFace();
                        Face secondFace = Einf.SecondFace();

                        s1 = _fMap.GetValue(firstFace).Geometry()!;
                        s2 = _fMap.GetValue(secondFace).Geometry()!;

                        int detrompeur = 0;

                        // Return FirstVertex and the tangent at this point.
                        Vertex fv = Exploration.FirstVertex(theEdge);
                        Vertex lv = Exploration.LastVertex(theEdge);
                        double pmin = 0.0;
                        double prmfv = Tool.Parameter(fv, theEdge);
                        double prmlv = Tool.Parameter(lv, theEdge);
                        Point3D pfv, plv;
                        Vector3D d1fv, d1lv, newd1;
                        C.D1(prmfv, out pfv, out d1fv);
                        C.D1(prmlv, out plv, out d1lv);

                        double tolF1 = Tool.Tolerance(firstFace);
                        double tolF2 = Tool.Tolerance(secondFace);

                        // Pass the tolerance of the face to project
                        ProjectPointOnSurface proj1 = new(pfv, s1, tolF1);
                        ProjectPointOnSurface proj2 = new(plv, s1, tolF1);
                        ProjectPointOnSurface proj3 = new(pfv, s2, tolF2);
                        ProjectPointOnSurface proj4 = new(plv, s2, tolF2);

                        if (proj1.IsDone && proj2.IsDone)
                        {
                            if (proj1.LowerDistance() <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                && proj2.LowerDistance() <= global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                detrompeur = 1;
                            }
                        }

                        if (proj3.IsDone && proj4.IsDone)
                        {
                            if (proj3.LowerDistance() <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                && proj4.LowerDistance() <= global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                detrompeur = 2;
                            }
                        }

                        Direction3D theDirExtr = new();
                        CoordinateSystem axis = new();
                        Curve3D? theNewCurve = null;
                        bool kPart = false;

                        if (s1.GetType() == typeof(TrimmedSurface))
                        {
                            s1 = ((TrimmedSurface)s1).BasisSurface;
                        }
                        if (s2.GetType() == typeof(TrimmedSurface))
                        {
                            s2 = ((TrimmedSurface)s2).BasisSurface;
                        }

                        bool pc1 = true; // KPart on S1
                        if (s1.GetType() == typeof(ExtrusionSurface) && s2.GetType() == typeof(PlaneSurface))
                        {
                            kPart = true;
                            axis = ((PlaneSurface)s2).Position;
                            theNewCurve = ((ExtrusionSurface)s1).BasisCurve;
                            theDirExtr = ((ExtrusionSurface)s1).Direction;
                        }
                        else if (s2.GetType() == typeof(ExtrusionSurface)
                                 && s1.GetType() == typeof(PlaneSurface))
                        {
                            kPart = true;
                            pc1 = false;
                            axis = ((PlaneSurface)s1).Position;
                            theNewCurve = ((ExtrusionSurface)s2).BasisCurve;
                            theDirExtr = ((ExtrusionSurface)s2).Direction;
                        }
                        CircleCurve3D? circ = null;
                        if (kPart)
                        { // very temporary on circles !!!
                            circ = theNewCurve as CircleCurve3D;
                            if (circ is null)
                            {
                                kPart = false;
                            }
                            else
                            {
                                Direction3D axofCirc = circ.Position.Direction;
                                kPart = axofCirc.IsParallel(axis.Direction, global::Kerf.Engine.Foundation.Precision.Angular());
                            }
                        }

                        int imin;
                        IntSS i2s = new();
                        if (kPart)
                        {
                            // direct calculation of NewC
                            double localReal = new Vector3D(circ!.Circle.Location, axis.Location)
                                                    .Dot(new Vector3D(axis.Direction));
                            double cos = theDirExtr.Dot(axis.Direction);
                            Vector3D vv = (localReal / cos) * new Vector3D(theDirExtr);
                            newC = (Curve3D)theNewCurve!.Translated(vv);
                            // it is possible to calculate PCurve
                            LineCurve2D l2d = new(new Point2D(0.0, localReal / cos), Constants.DX2d());

                            if (pc1)
                            {
                                Einf.ChangeFirstPC() = l2d;
                            }
                            else
                            {
                                Einf.ChangeSecondPC() = l2d;
                            }
                        }
                        else
                        {
                            s1 = _fMap.GetValue(Einf.FirstFace()).Geometry()!;
                            s2 = _fMap.GetValue(Einf.SecondFace()).Geometry()!;

                            // PCurves are not calculated immediately for 2 reasons:
                            // 1 - If ProjLib should make an Approx, it is stupid to approximate the
                            //     entire intersection curve.
                            // 2 - Additionally, if YaRev, there is a risk to not be SameRange.
                            i2s.Perform(s1, s2, global::Kerf.Engine.Foundation.Precision.Confusion(), true, false, false);

                            if (!i2s.IsDone || i2s.LineCount <= 0)
                            {
                                _errStat = DraftErrorStatus.EdgeRecomputation;
                                _badShape = theEdge;
                                return;
                            }

                            double glob2Min = RealLast();
                            CurveAdaptor TheCurve = new();

                            int i, j; //,jmin;

                            if (i2s.Line(1).GetType() != typeof(BSplineCurve3D))
                            {
                                double dist2Min = RealLast();
                                imin = 0;
                                for (i = 1; i <= i2s.LineCount; i++)
                                {
                                    TheCurve.Load(i2s.Line(i));
                                    CurvePointExtrema myExtPC = new(pfv, TheCurve);

                                    double locpmin = 0.0;
                                    if (myExtPC.IsDone)
                                    {
                                        if (myExtPC.ExtCount >= 1)
                                        {
                                            dist2Min = myExtPC.SquareDistance(1);
                                            locpmin = myExtPC.Point(1).Parameter();
                                        }
                                        if (myExtPC.ExtCount == 2 && dist2Min > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                                        {
                                            // to avoid incorrectly choosing the image
                                            // of the first vertex of the initial edge
                                            double d1_2 = myExtPC.SquareDistance(1);
                                            double d2_2 = myExtPC.SquareDistance(2);
                                            if (d1_2 > 1.21 * d2_2)
                                            {
                                                dist2Min = myExtPC.SquareDistance(2);
                                                locpmin = myExtPC.Point(2).Parameter();
                                            }
                                            else if (d2_2 > 1.21 * d1_2)
                                            {
                                                dist2Min = myExtPC.SquareDistance(1);
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
                                                    if (s1.GetType() == typeof(TrimmedSurface))
                                                    {
                                                        s1 = ((TrimmedSurface)s1).BasisSurface;
                                                    }
                                                    theSurf = s1;
                                                }
                                                else if (detrompeur == 2)
                                                {
                                                    if (s2.GetType() == typeof(TrimmedSurface))
                                                    {
                                                        s2 = ((TrimmedSurface)s2).BasisSurface;
                                                    }
                                                    theSurf = s2;
                                                }
                                                if (detrompeur != 0 && detrompeur != 4)
                                                {
                                                    double ul = 0.0, vl = 0.0, uf = 0.0, vf = 0.0;
                                                    double ufprim = 0.0, ulprim = 0.0, vfprim = 0.0, vlprim = 0.0;

                                                    if (theSurf!.GetType() == typeof(PlaneSurface))
                                                    {
                                                        // OCCT reads S2 whatever theSurf is (a null down-cast throws).
                                                        Plane3D pl = DownCast<PlaneSurface>(s2).Plane;
                                                        global::Kerf.Engine.Geometry.ElSLib.Parameters(pl, plv, out ul, out vl);
                                                        global::Kerf.Engine.Geometry.ElSLib.Parameters(pl, pfv, out uf, out vf);
                                                        global::Kerf.Engine.Geometry.ElSLib.Parameters(pl, plvprim, out ulprim, out vlprim);
                                                        global::Kerf.Engine.Geometry.ElSLib.Parameters(pl, pfvprim, out ufprim, out vfprim);
                                                    }
                                                    else if (theSurf.GetType() == typeof(CylindricalSurface))
                                                    {
                                                        Cylinder3D cy = DownCast<CylindricalSurface>(s2).Cylinder;
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
                                                            dist2Min = myExtPC.SquareDistance(2);
                                                            locpmin = myExtPC.Point(2).Parameter();
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                        if (myExtPC.ExtCount == 1 || myExtPC.ExtCount > 2 || detrompeur == 4)
                                        {
                                            dist2Min = myExtPC.SquareDistance(1);
                                            locpmin = myExtPC.Point(1).Parameter();
                                            for (j = 2; j <= myExtPC.ExtCount; j++)
                                            {
                                                double dist2 = myExtPC.SquareDistance(j);
                                                if (dist2 < dist2Min)
                                                {
                                                    dist2Min = dist2;
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
                                                dist2Min = dist1_2;
                                                locpmin = TheCurve.FirstParameter();
                                            }
                                            else
                                            {
                                                dist2Min = dist2_2;
                                                locpmin = TheCurve.LastParameter();
                                            }
                                        }

                                        if (dist2Min < glob2Min)
                                        {
                                            glob2Min = dist2Min;
                                            imin = i;
                                            pmin = locpmin;
                                        }
                                    }
                                }
                                if (imin == 0)
                                {
                                    _errStat = DraftErrorStatus.EdgeRecomputation;
                                    _badShape = theEdge;
                                    return;
                                }

                                newC = i2s.Line(imin);

                                newC.D1(pmin, out pfv, out newd1);
                                bool yaRev = d1fv.Dot(newd1) < 0.0;

                                if (yaRev)
                                {
                                    Reverse(ref newC);
                                }

                                if (i2s.HasLineOnS1(imin))
                                {
                                    Einf.ChangeFirstPC() = i2s.LineOnS1(imin);
                                    if (yaRev)
                                    {
                                        Reverse(ref Einf.ChangeFirstPC());
                                    }
                                }

                                if (i2s.HasLineOnS2(imin))
                                {
                                    Einf.ChangeSecondPC() = i2s.LineOnS2(imin);
                                    if (yaRev)
                                    {
                                        Reverse(ref Einf.ChangeSecondPC());
                                    }
                                }
                            } // if (i2s.Line(1)->DynamicType() != STANDARD_TYPE(Geom_BSplineCurve))
                            else // i2s.Line(1) is BSplineCurve
                            {
                                // OCCT.NET: OCCT reverses the intersection's own curves in place and compares them
                                // by handle; the port's are frozen, so this branch works on unfrozen copies of them,
                                // made once (the same handles throughout).
                                NCollection_Array1<Curve3D> lines = new(1, i2s.LineCount);
                                for (i = 1; i <= i2s.LineCount; i++)
                                {
                                    lines[i] = (Curve3D)i2s.Line(i).Copy();
                                }

                                // Find the first curve to glue
                                NCollection_Sequence<Curve3D> candidates = new();
                                if (s1.GetType() == typeof(CylindricalSurface)
                                    || s1.GetType() == typeof(ConicalSurface))
                                {
                                    for (i = 1; i <= i2s.LineCount; i++)
                                    {
                                        Curve3D curve = lines[i];
                                        Point3D pnt = curve.Value(curve.FirstParameter);
                                        ProjectPointOnSurface projector = new(pnt, s1, global::Kerf.Engine.Foundation.Precision.Confusion());
                                        projector.LowerDistanceParameters(out double u, out double v);
                                        if (Math.Abs(u) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                            || Math.Abs(u - 2.0 * M_PI) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                                        {
                                            candidates.Append(curve);
                                        }
                                        else
                                        {
                                            pnt = curve.Value(curve.LastParameter);
                                            projector.Init(pnt, s1, global::Kerf.Engine.Foundation.Precision.Confusion());
                                            projector.LowerDistanceParameters(out u, out v);
                                            if (Math.Abs(u) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                                || Math.Abs(u - 2.0 * M_PI) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                                            {
                                                curve.Reverse();
                                                candidates.Append(curve);
                                            }
                                        }
                                    }

                                    if (candidates.Length() == 0)
                                    {
                                        // errStat = Draft_EdgeRecomputation;
                                        // badShape = TopoDS::Edge(ite.Key());
                                        // return;
                                        for (i = 1; i <= i2s.LineCount; i++)
                                        {
                                            candidates.Append(lines[i]);
                                        }
                                    }
                                }
                                else
                                {
                                    for (i = 1; i <= i2s.LineCount; i++)
                                    {
                                        candidates.Append(lines[i]);
                                    }
                                }

                                Curve3D? firstCurve = null;
                                if (candidates.Length() > 1)
                                {
                                    double distMin = global::Kerf.Engine.Foundation.Precision.Infinite();
                                    for (i = 1; i <= candidates.Length(); i++)
                                    {
                                        Curve3D curve = candidates[i];
                                        Point3D pnt = curve.Value(curve.FirstParameter);
                                        double dist = pnt.DistanceTo(pfv);
                                        if (dist - distMin < -global::Kerf.Engine.Foundation.Precision.Confusion())
                                        {
                                            distMin = dist;
                                            firstCurve = curve;
                                        }
                                    }
                                }
                                else
                                {
                                    firstCurve = candidates[1];
                                }

                                // Glueing
                                NCollection_Sequence<Curve3D> curves = new();
                                for (i = 1; i <= i2s.LineCount; i++)
                                {
                                    if (firstCurve != lines[i])
                                    {
                                        curves.Append(lines[i]);
                                    }
                                }

                                NCollection_Sequence<Curve3D> toGlue = new();
                                // OCCT reads FirstCurve before its null check below.
                                Point3D endPoint = firstCurve!.Value(firstCurve.LastParameter);
                                bool added = true;
                                while (added)
                                {
                                    added = false;
                                    for (i = 1; i <= curves.Length(); i++)
                                    {
                                        Curve3D curve = curves[i];
                                        Point3D pfirst, plast;
                                        pfirst = curve.Value(curve.FirstParameter);
                                        plast = curve.Value(curve.LastParameter);
                                        if (pfirst.DistanceTo(endPoint) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                                        {
                                            toGlue.Append(curve);
                                            endPoint = plast;
                                            curves.Remove(i);
                                            added = true;
                                            break;
                                        }
                                        if (plast.DistanceTo(endPoint) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                                        {
                                            curve.Reverse();
                                            toGlue.Append(curve);
                                            endPoint = pfirst;
                                            curves.Remove(i);
                                            added = true;
                                            break;
                                        }
                                    }
                                }

                                if (firstCurve is null)
                                {
                                    _errStat = DraftErrorStatus.EdgeRecomputation;
                                    _badShape = theEdge;
                                    return;
                                }

                                CompCurveToBSplineCurve concat = new(DownCast<BSplineCurve3D>(firstCurve));
                                for (i = 1; i <= toGlue.Length(); i++)
                                {
                                    concat.Add(DownCast<BSplineCurve3D>(toGlue[i]), global::Kerf.Engine.Foundation.Precision.Confusion(), true);
                                }

                                newC = concat.BSplineCurve()!;

                                TheCurve.Load(newC);
                                CurvePointExtrema myExtPC = new(pfv, TheCurve);
                                double dist2Min = RealLast();
                                for (i = 1; i <= myExtPC.ExtCount; i++)
                                {
                                    if (myExtPC.IsMin(i))
                                    {
                                        double dist2 = myExtPC.SquareDistance(i);
                                        if (dist2 < dist2Min)
                                        {
                                            dist2Min = dist2;
                                            pmin = myExtPC.Point(i).Parameter();
                                        }
                                    }
                                }
                                newC.D1(pmin, out pfv, out newd1);
                                bool yaRev = d1fv.Dot(newd1) < 0.0;

                                if (yaRev)
                                {
                                    Reverse(ref newC);
                                }
                            } // else: i2s.NbLines() > 2 && S1 is Cylinder or Cone

                            Einf.Tolerance(Max(Einf.Tolerance(), i2s.TolReached3d()));
                        } // End step KPart
                    }
                    else
                    { // case of tangency
                        Face f1 = Einf.FirstFace();
                        Face f2 = Einf.SecondFace();

                        Surface? localS1 = _fMap.GetValue(f1).Geometry();
                        Surface? localS2 = _fMap.GetValue(f2).Geometry();
                        if (localS1 is null || localS2 is null)
                        {
                            _errStat = DraftErrorStatus.EdgeRecomputation;
                            _badShape = theEdge;
                            return;
                        }
                        if (localS1.GetType() == typeof(TrimmedSurface))
                        {
                            localS1 = ((TrimmedSurface)localS1).BasisSurface;
                        }
                        if (localS2.GetType() == typeof(TrimmedSurface))
                        {
                            localS2 = ((TrimmedSurface)localS2).BasisSurface;
                        }

                        Direction3D dirextr = new();
                        // bool dirfound = false;
                        if (localS1.GetType() == typeof(CylindricalSurface))
                        {
                            Cylinder3D cyl = ((CylindricalSurface)localS1).Cylinder;
                            dirextr = cyl.Axis.Direction;
                            // dirfound = true;
                            //  see direction...
                        }
                        else if (localS1.GetType() == typeof(ExtrusionSurface))
                        {
                            dirextr = ((ExtrusionSurface)localS1).Direction;
                            // dirfound = true;
                            //  see direction...

                            // Here it is possible to calculate PCurve.
                            ExtrusionSurface sel = (ExtrusionSurface)localS1;
                            if (sel.BasisCurve is CircleCurve3D gCir)
                            {
                                double u = global::Kerf.Engine.Geometry.ElCLib.Parameter(gCir.Circle, ptfixe);
                                LineCurve2D pc1 = new(new Point2D(u, 0.0), Constants.DY2d());
                                Einf.ChangeFirstPC() = pc1;
                            }
                        }

                        else if (localS2.GetType() == typeof(CylindricalSurface))
                        {
                            Cylinder3D cyl = ((CylindricalSurface)localS2).Cylinder;
                            dirextr = cyl.Axis.Direction;
                            // dirfound = true;
                            // see direction...
                        }
                        else if (localS2.GetType() == typeof(ExtrusionSurface))
                        {
                            dirextr = ((ExtrusionSurface)localS2).Direction;
                            // dirfound = true;
                            // see direction...

                            // Here it is possible to calculate PCurve.
                            ExtrusionSurface sel = (ExtrusionSurface)localS2;
                            if (sel.BasisCurve is CircleCurve3D gCir)
                            {
                                double u = global::Kerf.Engine.Geometry.ElCLib.Parameter(gCir.Circle, ptfixe);
                                LineCurve2D pc2 = new(new Point2D(u, 0.0), Constants.DY2d());
                                Einf.ChangeSecondPC() = pc2;
                            }
                        }
                        newC = new LineCurve3D(ptfixe, dirextr);

                        C.D1(0.0, out Point3D pfv, out Vector3D d1fv);
                        newC.D1(0.0, out pfv, out Vector3D newd1);
                        bool yaRev = d1fv.Dot(newd1) < 0.0;
                        if (yaRev)
                        {
                            Reverse(ref newC);
                            if (Einf.FirstPC() is not null)
                            {
                                Reverse(ref Einf.ChangeFirstPC());
                            }
                            if (Einf.SecondPC() is not null)
                            {
                                Reverse(ref Einf.ChangeSecondPC());
                            }
                        }
                    }

                    if (newC is TrimmedCurve3D t)
                    {
                        newC = t.BasisCurve;
                    }
                    Einf.ChangeGeometry() = newC;
                }
                else if (!Einf.NewGeometry())
                {
                    // set existing curve 3D
                    if (C is TrimmedCurve3D t)
                    {
                        C = t.BasisCurve;
                    }
                    Einf.ChangeGeometry() = C;
                }
            }

            // Calculate new vertices.

            CurveAdaptor hac = new();
            SurfaceAdaptor has = new();

            for (int ii = 0; ii < _vMap.Count; ii++)
            {
                CurveAdaptor ac = hac;
                SurfaceAdaptor AS = has;

                Vertex tvv = ShapeCasts.Vertex(_vMap.KeyAt(ii));
                VertexInfo vinf = _vMap.ValueAt(ii);
                if (!Choose(_fMap, _eMap, tvv, vinf, ac, AS))
                {
                    // no concerted edge => alignment of two consecutive edges.
                    Point3D pvt = new();
                    vinf.ChangeGeometry() = pvt;
                    vinf.InitEdgeIterator();
                    if (vinf.MoreEdge())
                    {
                        Edge edg1 = vinf.Edge();
                        // const Draft_EdgeInfo& Einf1 = myEMap(Edg1);
                        DraftEdgeInfo einf1 = _eMap.GetValue(edg1);
                        Point3D vtori = Tool.Pnt(tvv);
                        // Einf1.Geometry()->D0(Vinf.Parameter(Edg1), pvt);
                        ProjectPointOnCurve Projector = new(vtori, einf1.Geometry()!); // patch
                        pvt = Projector.NearestPoint();

                        double dion = pvt.SquareDistanceTo(vtori);
                        vinf.NextEdge();
                        if (vinf.MoreEdge())
                        {
                            Edge edg2 = vinf.Edge();
                            // const Draft_EdgeInfo& Einf2 = myEMap(Edg2);
                            DraftEdgeInfo einf2 = _eMap.GetValue(edg2);
                            //	    double f;
                            einf2.Geometry()!.D0(vinf.Parameter(edg2), out Point3D opvt);

                            if (opvt.SquareDistanceTo(vtori) < dion)
                            {
                                pvt = opvt;
                            }
                            // Vinf.ChangeParameter(Edg2) = Parameter(Einf2.Geometry(), pvt);
                            double param2 = Parameter(einf2.Geometry()!, pvt, out int done2);
                            if (done2 != 0)
                            {
                                Surface s1 = _fMap.GetValue(einf2.FirstFace()).Geometry()!;
                                Surface s2 = _fMap.GetValue(einf2.SecondFace()).Geometry()!;
                                vinf.ChangeParameter(edg2) =
                                    SmartParameter(einf2, Tool.Tolerance(edg2), pvt, done2, s1, s2);
                            }
                            else
                            {
                                vinf.ChangeParameter(edg2) = param2;
                            }
                        }

                        vinf.ChangeGeometry() = pvt;
                        // Vinf.ChangeParameter(Edg1) = Parameter(Einf1.Geometry(), pvt);
                        double param = Parameter(einf1.Geometry()!, pvt, out int done);
                        if (done != 0)
                        {
                            Surface s1 = _fMap.GetValue(einf1.FirstFace()).Geometry()!;
                            Surface s2 = _fMap.GetValue(einf1.SecondFace()).Geometry()!;
                            vinf.ChangeParameter(edg1) =
                                SmartParameter(einf1, Tool.Tolerance(edg1), pvt, done, s1, s2);
                        }
                        else
                        {
                            vinf.ChangeParameter(edg1) = param;
                        }
                        continue;
                    }

                    _errStat = DraftErrorStatus.VertexRecomputation;
                    _badShape = tvv;
                    return;
                }

                HInter myintcs = new();
                myintcs.Perform(hac, has);
                if (!myintcs.IsDone)
                {
                    _errStat = DraftErrorStatus.VertexRecomputation;
                    _badShape = tvv;
                    return;
                }

                {
                    Point3D vtori = Tool.Pnt(tvv);
                    Point3D pvt;

                    int nbsol = myintcs.PointCount;
                    if (nbsol <= 0)
                    {
                        ExtCS extr = new(ac, AS, global::Kerf.Engine.Foundation.Precision.PConfusion(), global::Kerf.Engine.Foundation.Precision.PConfusion());

                        if (!extr.IsDone || extr.ExtCount == 0)
                        {
                            _errStat = DraftErrorStatus.VertexRecomputation;
                            _badShape = tvv;
                            return;
                        }

                        double disref = RealLast();
                        int iref = 0;
                        POnCurv pc;
                        POnSurf ps;
                        for (int i = 1; i <= extr.ExtCount; i++)
                        {
                            extr.Points(i, out pc, out ps);
                            double distemp = pc.Value().SquareDistanceTo(vtori);
                            if (distemp < disref)
                            {
                                disref = distemp;
                                iref = i;
                            }
                        }
                        extr.Points(iref, out pc, out ps);
                        pvt = pc.Value();
                    }
                    else
                    {
                        double disref = RealLast();
                        int iref = 0;
                        for (int i = 1; i <= nbsol; i++)
                        {
                            double distemp = myintcs.Point(i).Pnt().SquareDistanceTo(vtori);
                            if (distemp < disref)
                            {
                                disref = distemp;
                                iref = i;
                            }
                        }
                        pvt = myintcs.Point(iref).Pnt();
                    }

                    vinf.ChangeGeometry() = pvt;

                    for (vinf.InitEdgeIterator(); vinf.MoreEdge(); vinf.NextEdge())
                    {
                        Edge Edg = vinf.Edge();
                        double initpar = vinf.Parameter(Edg);
                        // const Draft_EdgeInfo& Einf = myEMap(Edg);
                        DraftEdgeInfo Einf = _eMap.GetValue(Edg);
                        // Vinf.ChangeParameter(Edg) = Parameter(Einf.Geometry(),pvt);
                        double param = Parameter(Einf.Geometry()!, pvt, out int done);
                        if (done != 0)
                        {
                            Surface s1 = _fMap.GetValue(Einf.FirstFace()).Geometry()!;
                            Surface s2 = _fMap.GetValue(Einf.SecondFace()).Geometry()!;
                            vinf.ChangeParameter(Edg) = SmartParameter(Einf, Tool.Tolerance(Edg), pvt, done, s1, s2);
                        }
                        else
                        {
                            if (Math.Abs(initpar - param) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                Curve3D? c = Tool.Curve(Edg, out Placement loc, out double f, out double l);
                                if (c!.GetType() == typeof(TrimmedCurve3D))
                                {
                                    Einf.SetNewGeometry(true);
                                }
                            }
                            vinf.ChangeParameter(Edg) = param;
                        }
                    }
                }
            }
        }

        // small loop of validation/protection

        for (int i = 0; i < _eMap.Count; i++)
        {
            Edge edg = ShapeCasts.Edge(_eMap.KeyAt(i));

            Exploration.Vertices(edg, out Vertex Vf, out Vertex Vl);
            if (edg.Orientation == Reversed)
            {
                Vf.Reverse();
                Vl.Reverse();
            }

            if (_vMap.ContainsKey(Vf) && _vMap.ContainsKey(Vl))
            {
                // Here, we compare directions of the source edge (from input shape)
                // and corresponding selected part of the intersection edge.
                // If these directions are opposite then we reverse intersection edge
                // and recompute corresponding vertex-parameters.

                double parF = _vMap.GetValue(Vf).Parameter(edg);
                double parL = _vMap.GetValue(Vl).Parameter(edg);

                if (parL < parF)
                {
                    DraftEdgeInfo einf = _eMap.GetValue(edg);
                    Curve3D sCurve = Tool.Curve(edg, out double aF, out double aL)!;
                    Curve3D? intCurv = einf.Geometry()!;
                    Point3D aPf, aPl;
                    Vector3D dirNF, dirNL, dirOF, dirOL;
                    sCurve.D1(Tool.Parameter(Vf, edg), out aPf, out dirOF);
                    sCurve.D1(Tool.Parameter(Vl, edg), out aPl, out dirOL);
                    intCurv.D1(parF, out aPf, out dirNF);
                    intCurv.D1(parL, out aPl, out dirNL);

                    double sqMagn = dirNF.SquareMagnitude;

                    if (sqMagn > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        dirNF = dirNF.Divided(Math.Sqrt(sqMagn));
                    }

                    sqMagn = dirNL.SquareMagnitude;
                    if (sqMagn > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        dirNL = dirNL.Divided(Math.Sqrt(sqMagn));
                    }

                    sqMagn = dirOF.SquareMagnitude;
                    if (sqMagn > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        dirOF = dirOF.Divided(Math.Sqrt(sqMagn));
                    }

                    sqMagn = dirOL.SquareMagnitude;
                    if (sqMagn > global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        dirOL = dirOL.Divided(Math.Sqrt(sqMagn));
                    }

                    double cosF = dirNF.Dot(dirOF), cosL = dirNL.Dot(dirOL);
                    double cosMax = Math.Abs(cosF) > Math.Abs(cosL) ? cosF : cosL;

                    if (cosMax < 0.0)
                    {
                        Reverse(ref intCurv);
                        einf.ChangeGeometry() = intCurv;
                        double par = Parameter(einf.Geometry()!, aPf, out int err);
                        if (err == 0)
                        {
                            _vMap.GetValue(Vf).ChangeParameter(edg) = par;
                        }
                        par = Parameter(einf.Geometry()!, aPl, out err);
                        if (err == 0)
                        {
                            _vMap.GetValue(Vl).ChangeParameter(edg) = par;
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
                Curve3D theCurve = _eMap.GetValue(edg).Geometry()!;
                if (theCurve.IsClosed)
                {
                    // pf >= pl
                    double firstPar = theCurve.FirstParameter, lastPar = theCurve.LastParameter;
                    double pconf = global::Kerf.Engine.Foundation.Precision.PConfusion();
                    if (Math.Abs(pf - lastPar) <= pconf)
                    {
                        pf = firstPar;
                    }
                    else if (Math.Abs(pl - firstPar) <= pconf)
                    {
                        pl = lastPar;
                    }

                    if (pl <= pf)
                    {
                        pl += (lastPar - firstPar);
                    }
                }
                if (pl <= pf)
                {
                    _errStat = DraftErrorStatus.EdgeRecomputation;
                    _badShape = edg;
                    return;
                }
            }
            if (_vMap.ContainsKey(Vf))
            {
                _vMap.GetValue(Vf).ChangeParameter(edg) = pf;
            }
            if (_vMap.ContainsKey(Vl))
            {
                _vMap.GetValue(Vl).ChangeParameter(edg) = pl;
            }
        }
    }

    //=================================================================================================

    private Surface? NewSurface(Surface s,
                                     Orientation oris,
                                     in Direction3D direction,
                                     double angle,
                                     in Plane3D neutralPlane)
    {
        Surface? newS = null;

        Type typeS = s.GetType();

        if (typeS == typeof(PlaneSurface))
        {
            Plane3D pl = ((PlaneSurface)s).Plane;
            if (FindRotation(pl, oris, direction, angle, neutralPlane, out Axis3D axe, out double theta))
            {
                if (Math.Abs(theta) > global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    newS = (Surface)s.Rotated(axe, theta);
                }
                else
                {
                    newS = s;
                }
            }
        }
        else if (typeS == typeof(CylindricalSurface))
        {
            double testdir = direction.Dot(neutralPlane.Axis.Direction);
            if (Math.Abs(testdir) <= 1.0 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                return newS;
            }
            Cylinder3D cy = ((CylindricalSurface)s).Cylinder;
            testdir = direction.Dot(cy.Axis.Direction);
            if (Math.Abs(testdir) <= 1.0 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                return newS;
            }
            if (Math.Abs(angle) > global::Kerf.Engine.Foundation.Precision.Angular())
            {
                QuadQuadGeo i2s = new();
                i2s.Perform(neutralPlane, cy, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
                bool isIntDone = i2s.IsDone;

                if (i2s.TypeInter() == Ellipse)
                {
                    Ellipse3D el = i2s.Ellipse(1);
                    double majorR = el.MajorRadius;
                    double minorR = el.MinorRadius;
                    isIntDone = (majorR < 100000.0 * minorR);
                }

                if (!isIntDone || i2s.TypeInter() != Circle)
                {
                    return newS;
                }
                CoordinateSystem axcone = cy.Position;
                // Pb : Where is the material???
                double alpha = angle;
                bool direct = axcone.IsDirect;
                if ((direct && oris == Reversed) || (!direct && oris == Forward))
                {
                    alpha = -alpha;
                }

                Point3D center = i2s.Circle(1).Location;
                if (testdir < 0.0)
                {
                    alpha = -alpha;
                }
                double z = global::Kerf.Engine.Geometry.ElCLib.LineParameter(cy.Axis, center);
                double rad = cy.Radius + z * Math.Tan(alpha);
                if (rad < 0.0)
                {
                    rad = -rad;
                }
                else
                {
                    alpha = -alpha;
                }
                Cone3D co = new(axcone, alpha, rad);
                newS = new ConicalSurface(co);
            }
            else
            {
                newS = s;
            }
        }
        else if (typeS == typeof(ConicalSurface))
        {
            double testdir = direction.Dot(neutralPlane.Axis.Direction);
            if (Math.Abs(testdir) <= 1.0 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                return newS;
            }

            Cone3D co1 = ((ConicalSurface)s).Cone;

            testdir = direction.Dot(co1.Axis.Direction);
            if (Math.Abs(testdir) <= 1.0 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                return newS;
            }

            QuadQuadGeo i2s = new();
            i2s.Perform(neutralPlane, co1, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
            if (!i2s.IsDone || i2s.TypeInter() != Circle)
            {
                return newS;
            }
            CoordinateSystem axcone = co1.Position;
            // Pb : Where is the material???
            double alpha = angle;
            bool direct = axcone.IsDirect;
            if ((direct && oris == Reversed) || (!direct && oris == Forward))
            {
                alpha = -alpha;
            }

            Point3D center = i2s.Circle(1).Location;
            if (Math.Abs(angle) > global::Kerf.Engine.Foundation.Precision.Angular())
            {
                if (testdir < 0.0)
                {
                    alpha = -alpha;
                }
                double z = global::Kerf.Engine.Geometry.ElCLib.LineParameter(co1.Axis, center);
                double rad = i2s.Circle(1).Radius + z * Math.Tan(alpha);
                if (rad < 0.0)
                {
                    rad = -rad;
                }
                else
                {
                    alpha = -alpha;
                }
                if (Math.Abs(alpha - co1.SemiAngle) < global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    newS = s;
                }
                else
                {
                    Cone3D co = new(axcone, alpha, rad);
                    newS = new ConicalSurface(co);
                }
            }
            else
            {
                newS = new CylindricalSurface(new Cylinder3D(axcone, i2s.Circle(1).Radius));
            }
        }
        return newS;
    }

    //=================================================================================================

    private Curve3D? NewCurve(Curve3D c,
                                 Surface s,
                                 Orientation oris,
                                 in Direction3D direction,
                                 double angle,
                                 in Plane3D neutralPlane,
                                 bool flag = true)
    {
        Curve3D? newC = null;

        Type typeS = s.GetType();

        if (typeS == typeof(PlaneSurface))
        {
            Plane3D pl = ((PlaneSurface)s).Plane;
            if (FindRotation(pl, oris, direction, angle, neutralPlane, out Axis3D axe, out double theta))
            {
                if (Math.Abs(theta) > global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    newC = (Curve3D)c.Rotated(axe, theta);
                }
                else
                {
                    newC = c;
                }
            }
            return newC;
        }

        if (c.GetType() != typeof(LineCurve3D))
        {
            return newC;
        }

        Line3D lin = ((LineCurve3D)c).Line;
        //  double testdir = Direction.Dot(lin.Direction());
        //  if (std::abs(testdir) <= 1.-Precision::Angular()) {
        //    return NewC;
        //  }
        Direction3D norm = new();
        if (typeS == typeof(CylindricalSurface))
        {
            Cylinder3D cy = ((CylindricalSurface)s).Cylinder;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(cy, lin.Location, out double u, out double v);
            global::Kerf.Engine.Geometry.ElSLib.D1(u, v, cy, out Point3D pbid, out Vector3D d1u, out Vector3D d1v);
            norm = new Direction3D(d1u.Crossed(d1v));
        }
        else if (typeS == typeof(ConicalSurface))
        {
            Cone3D co = ((ConicalSurface)s).Cone;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(co, lin.Location, out double u, out double v);
            global::Kerf.Engine.Geometry.ElSLib.D1(u, v, co, out Point3D pbid, out Vector3D d1u, out Vector3D d1v);
            norm = new Direction3D(d1u.Crossed(d1v));
        }

        IntConicQuad ilipl = new(lin, neutralPlane, global::Kerf.Engine.Foundation.Precision.Angular());
        if (ilipl.IsDone && ilipl.PointCount != 0)
        {
            if (oris == Reversed)
            {
                norm = norm.Reversed();
            }
            Axis3D axrot = new(ilipl.Point(1), norm.Crossed(direction));
            Line3D lires = new Line3D(new Axis3D(ilipl.Point(1), direction)).Rotated(axrot, angle);
            if (lires.Direction.Dot(lin.Direction) < 0.0)
            {
                lires = lires.Reversed();
            }
            newC = new LineCurve3D(lires);
        }
        return newC;
    }

    //=================================================================================================

    private static bool Choose(IndexedMap<Shape, DraftFaceInfo> fMap,
                               IndexedMap<Shape, DraftEdgeInfo> eMap,
                               in Vertex vtx,
                               VertexInfo vinf,
                               CurveAdaptor ac,
                               SurfaceAdaptor AS)
    {
        Vector3D tgref;
        vinf.InitEdgeIterator();

        // Find a regular edge with null SecondFace
        while (vinf.MoreEdge())
        {
            Edge e1 = vinf.Edge();
            DraftEdgeInfo einf1 = eMap.GetValue(e1);
            if (einf1.SecondFace().IsNull)
            {
                break;
            }
            else
            {
                Continuity te = Tool.Continuity(e1, einf1.FirstFace(), einf1.SecondFace());
                if (te >= G1)
                {
                    break;
                }
            }
            vinf.NextEdge();
        }
        if (!vinf.MoreEdge())
        { // take the first edge
            vinf.InitEdgeIterator();
        }

        Edge eref = vinf.Edge();
        // const Draft_EdgeInfo& Einf = theEMap(Eref);
        DraftEdgeInfo einf = eMap.GetValue(eref);

        ac.Load(einf.Geometry()!);

        double prm;
        Curve3D c = Tool.Curve(eref, out Placement loc, out double f, out double l)!;
        c = (Curve3D)c.Transformed(loc.Transformation);
        // prm = Parameter(C,BRep_Tool::Pnt(Vtx));
        double param = Parameter(c, Tool.Pnt(vtx), out int done);
        if (done != 0)
        {
            Surface s1 = fMap.GetValue(einf.FirstFace()).Geometry()!;
            Surface s2 = fMap.GetValue(einf.SecondFace()).Geometry()!;
            prm = SmartParameter(einf, Tool.Tolerance(eref), Tool.Pnt(vtx), done, s1, s2);
        }
        else
        {
            prm = param;
        }
        c.D1(prm, out Point3D ptbid, out tgref);

        vinf.InitEdgeIterator();
        while (vinf.MoreEdge())
        {
            // Find a non tangent edge
            Edge edg = vinf.Edge();
            if (!edg.IsSame(eref))
            {
                // const Draft_EdgeInfo& Einfo = theEMap(Edg);
                DraftEdgeInfo einfo = eMap.GetValue(edg);
                if (!einfo.SecondFace().IsNull
                    && Tool.Continuity(edg, einfo.FirstFace(), einfo.SecondFace()) <= C0)
                {
                    c = Tool.Curve(edg, out loc, out f, out l)!;
                    c = (Curve3D)c.Transformed(loc.Transformation);
                    // prm = Parameter(C,BRep_Tool::Pnt(Vtx));
                    double anewparam = Parameter(c, Tool.Pnt(vtx), out int anewdone);
                    if (anewdone != 0)
                    {
                        Surface s1 = fMap.GetValue(einfo.FirstFace()).Geometry()!;
                        Surface s2 = fMap.GetValue(einfo.SecondFace()).Geometry()!;
                        prm = SmartParameter(einfo, Tool.Tolerance(edg), Tool.Pnt(vtx), anewdone, s1, s2);
                    }
                    else
                    {
                        prm = anewparam;
                    }
                    c.D1(prm, out ptbid, out Vector3D tg);
                    if (tg.CrossMagnitude(tgref) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        break;
                    }
                }
            }
            vinf.NextEdge();
        }
        if (!vinf.MoreEdge())
        {
            return false;
        }

        DraftEdgeInfo einf2 = eMap.GetValue(vinf.Edge());
        if (!einf.SecondFace().IsNull)
        {
            if (einf2.FirstFace().IsSame(einf.FirstFace()) || einf2.FirstFace().IsSame(einf.SecondFace()))
            {
                AS.Load(fMap.GetValue(einf2.SecondFace()).Geometry()!);
            }
            else
            {
                AS.Load(fMap.GetValue(einf2.FirstFace()).Geometry()!);
            }
        }
        else
        {
            if (einf2.FirstFace().IsSame(einf.FirstFace()))
            {
                AS.Load(fMap.GetValue(einf2.SecondFace()).Geometry()!);
            }
            else
            {
                AS.Load(fMap.GetValue(einf2.FirstFace()).Geometry()!);
            }
        }
        return true;
    }

    //=================================================================================================

    private static double Parameter(Curve3D c, in Point3D p, out int done)
    {
        done = 0;
        Curve3D cbase = c;
        Type ctyp = c.GetType();
        if (ctyp == typeof(TrimmedCurve3D))
        {
            cbase = ((TrimmedCurve3D)c).BasisCurve;
            ctyp = cbase.GetType();
        }
        double param;
        if (ctyp == typeof(LineCurve3D))
        {
            param = global::Kerf.Engine.Geometry.ElCLib.Parameter(((LineCurve3D)cbase).Line, p);
        }
        else if (ctyp == typeof(CircleCurve3D))
        {
            param = global::Kerf.Engine.Geometry.ElCLib.Parameter(((CircleCurve3D)cbase).Circle, p);
            if (Math.Abs(2.0 * M_PI - param) <= Epsilon(2.0 * M_PI))
            {
                param = 0.0;
            }
        }
        else if (ctyp == typeof(EllipseCurve3D))
        {
            param = global::Kerf.Engine.Geometry.ElCLib.Parameter(((EllipseCurve3D)cbase).Ellipse, p);
            if (Math.Abs(2.0 * M_PI - param) <= Epsilon(2.0 * M_PI))
            {
                param = 0.0;
            }
        }
        else if (ctyp == typeof(ParabolaCurve3D))
        {
            param = global::Kerf.Engine.Geometry.ElCLib.Parameter(((ParabolaCurve3D)cbase).Parabola, p);
        }
        else if (ctyp == typeof(HyperbolaCurve3D))
        {
            param = global::Kerf.Engine.Geometry.ElCLib.Parameter(((HyperbolaCurve3D)cbase).Hyperbola, p);
        }
        else
        {
            CurveAdaptor theCurve = new(c);
            CurvePointExtrema myExtPC = new(p, theCurve);
            if (!myExtPC.IsDone)
            {
                throw new FailureException("Draft_Modification_1::Parameter: ExtremaPC not done.");
            }
            if (myExtPC.ExtCount >= 1)
            {
                double dist2, dist2Min = myExtPC.SquareDistance(1);
                int j, jmin = 1;
                for (j = 2; j <= myExtPC.ExtCount; j++)
                {
                    dist2 = myExtPC.SquareDistance(j);
                    if (dist2 < dist2Min)
                    {
                        dist2Min = dist2;
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
                    param = theCurve.FirstParameter();
                }
                else
                {
                    done = 1;
                    param = theCurve.LastParameter();
                }
            }

            if (cbase.IsPeriodic)
            {
                double per = cbase.Period;
                double tolp = global::Kerf.Engine.Foundation.Precision.Parametric(global::Kerf.Engine.Foundation.Precision.Confusion());
                if (Math.Abs(per - param) <= tolp)
                {
                    param = 0.0;
                }
            }
        }
        return param;
    }

    //=================================================================================================

    private static double SmartParameter(DraftEdgeInfo einf,
                                         double edgeTol,
                                         in Point3D pnt,
                                         int sign,
                                         Surface s1,
                                         Surface s2)
    {
        Curve2D newC2d;
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        double etol = edgeTol;

        Curve2D? pcu1 = einf.FirstPC();
        Curve2D? pcu2 = einf.SecondPC();

        if (pcu1 is null)
        {
            Curve3D theCurve = einf.Geometry()!;
            pcu1 = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(theCurve, theCurve.FirstParameter, theCurve.LastParameter, s1, ref etol);
            einf.ChangeFirstPC() = pcu1;
        }
        if (pcu2 is null)
        {
            Curve3D theCurve = einf.Geometry()!;
            pcu2 = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(theCurve, theCurve.FirstParameter, theCurve.LastParameter, s2, ref etol);
            einf.ChangeSecondPC() = pcu2;
        }

        ProjectPointOnSurface projector = new(pnt, s1);
        projector.LowerDistanceParameters(out double u, out double v);

        // OCCT dereferences the projection without a check (null if it failed).
        newC2d = einf.FirstPC()!;
        if (newC2d.GetType() == typeof(TrimmedCurve2D))
        {
            newC2d = ((TrimmedCurve2D)newC2d).BasisCurve;
        }

        Point2D p2d = new(u, v);
        ProjectPointOnCurve2D projector2d = new(p2d, newC2d);
        if (projector2d.NbPoints() == 0 || projector2d.LowerDistance() > tol)
        {
            BSplineCurve2D bCurve;
            if (newC2d.GetType() != typeof(BSplineCurve2D))
            {
                bCurve = Geom2dConvert2D.CurveToBSplineCurve(newC2d);
            }
            else
            {
                bCurve = (BSplineCurve2D)newC2d;
            }
            if (sign == -1)
            {
                NCollection_Array1<Point2D> pntArray = new(1, 2);
                pntArray[1] = p2d;
                pntArray[2] = bCurve.Pole(1);
                BezierCurve2D patch = new(pntArray);
                CompCurveToBSplineCurve2D concat = new(bCurve, QuasiAngular);
                concat.Add(patch, tol, false);
                bCurve = concat.BSplineCurve()!;
            }
            else
            {
                NCollection_Array1<Point2D> pntArray = new(1, 2);
                pntArray[1] = bCurve.Pole(bCurve.PoleCount);
                pntArray[2] = p2d;
                BezierCurve2D patch = new(pntArray);
                CompCurveToBSplineCurve2D concat = new(bCurve, QuasiAngular);
                concat.Add(patch, tol, true);
                bCurve = concat.BSplineCurve()!;
            }
            newC2d = bCurve;
        }
        einf.ChangeFirstPC() = newC2d;
        CurveAdaptor2D hcur = new(newC2d);
        SurfaceAdaptor hsur = new(s1);
        CurveOnSurface cons = new(hcur, hsur);
        CurveOnSurface hcons = cons; // OCCT copies the adaptor into a handle: one object here
        SurfaceAdaptor hsur2 = new(s2);
        CompProjectedCurve hProjector = new(hsur2, hcons, tol, tol);
        hProjector.Bounds(1, out double udeb, out double ufin);
        int maxSeg = 20 + hProjector.NbIntervals(C3);
        CurveOnSurfaceApproximation appr = new(hProjector, hsur2, udeb, ufin, tol);
        appr.Perform(maxSeg, 10, C1, false, false);
        einf.ChangeSecondPC() = appr.Curve2d();
        einf.ChangeGeometry() = appr.Curve3d();
        einf.SetNewGeometry(true);

        if (sign == -1)
        {
            return einf.Geometry()!.FirstParameter;
        }
        else
        {
            return einf.Geometry()!.LastParameter;
        }
    }

    //=================================================================================================

    private static Orientation Orientation(in Shape s, in Face f)
    {
        //
        // change porting NT
        //
        Explorer expl = new();
        expl.Init(s, ShapeType.Face);
        while (expl.More())
        {
            if (ShapeCasts.Face(expl.Current()).IsSame(f))
            {
                return expl.Current().Orientation;
            }
            expl.Next();
        }
        return Forward;
    }

    //=================================================================================================

    private static bool FindRotation(in Plane3D pl,
                                     Orientation oris,
                                     in Direction3D direction,
                                     double angle,
                                     in Plane3D neutralPlane,
                                     out Axis3D axe,
                                     out double theta)
    {
        axe = new Axis3D();
        theta = 0.0;
        QuadQuadGeo i2pl = new(pl, neutralPlane, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());

        if (i2pl.IsDone && i2pl.TypeInter() == Line)
        {
            Line3D li = i2pl.Line(1);
            // Try to turn around this line
            Direction3D nx = li.Direction;
            Direction3D ny = pl.Axis.Direction.Crossed(nx);
            double a = direction.Dot(nx);
            if (Math.Abs(a) <= 1 - global::Kerf.Engine.Foundation.Precision.Angular())
            {
                double b = direction.Dot(ny);
                double c = direction.Dot(pl.Axis.Direction);
                bool direct = pl.Position.IsDirect;
                if ((direct && oris == Reversed) || (!direct && oris == Forward))
                {
                    b = -b;
                    c = -c;
                }
                double denom = Math.Sqrt(1 - a * a);
                double sina = Math.Sin(angle);
                if (denom > Math.Abs(sina))
                {
                    double phi = Math.Atan2(b / denom, c / denom);
                    double theta0 = Math.Acos(sina / denom);
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
                    axe = li.Position;
                    return true;
                }
            }
        }
        return false;
    }

    //=================================================================================================

    // OCCT.NET: OCCT reverses curves in place (the intersection's curves, its pcurves, the curves kept for an edge);
    // a frozen one (the intersection's are) is reversed on a copy, which takes its place.
    private static void Reverse(ref Curve3D? curve)
    {
        if (curve!.IsFrozen)
        {
            curve = (Curve3D)curve.Copy();
        }
        curve.Reverse();
    }

    private static void Reverse(ref Curve2D? curve)
    {
        if (curve!.IsFrozen)
        {
            curve = (Curve2D)curve.Copy();
        }
        curve.Reverse();
    }

    // OCCT's occ::down_cast followed by a dereference: a null handle there is an access violation, a
    // NullReferenceException here.
    private static T DownCast<T>(Geometry3D? geom) where T : Geometry3D
    {
        return geom as T ?? throw new NullReferenceException();
    }
}
