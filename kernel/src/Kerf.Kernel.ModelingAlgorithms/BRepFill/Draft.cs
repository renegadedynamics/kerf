// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Draft.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Draft.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Classification;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Features.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Healing;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Engine.Booleans.GlueMode;
using static Kerf.Engine.Booleans.Operation;
using static Kerf.Engine.Construction.Filling.TransitionStyle;
using static Kerf.Engine.Construction.Library.LibraryWireError;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Construction.Filling;

// OCCT_DEBUG blocks (traces) are left out.

/// <summary>
/// A draft surface: a ruled skin swept from a wire (or a face's outer wire, or a shell's free boundary) along a
/// direction at an angle, up to a length, a surface or a shape, sewn to the input face or shell.
/// </summary>
/// <remarks>
/// A working object (decision 21), the engine of <see cref="MakeDraft"/>. It reads its input: the
/// sweep, the Booleans and the sewing build new shapes. OCCT marks the wire closed in place when its ends are the same
/// vertex; the port marks a new wire with the same edges when the caller's is frozen (decision 1045).
/// <see cref="Shape"/>, <see cref="Shell"/> and <see cref="Generated"/> freeze what they return; the internal forms
/// don't.
/// </remarks>
public sealed class Draft
{
    private Direction3D _dir;
    private double _angle;
    private double _angmin;
    private double _angmax;
    private double _tol;
    private DraftLaw? _loc;
    private SectionLaw? _sec;
    private NCollection_HArray2<Shape>? _sections;
    private NCollection_HArray2<Shape>? _faces;
    private readonly NCollection_List<Shape> _generated = new();
    private Shape _shape = new();
    private readonly Shape _top;
    private Shell _shell = new();
    private Wire _wire = new();
    private readonly Continuity _cont;
    private TransitionStyle _style;
    private bool _isInternal;
    private bool _done;

    //=================================================================================================

    private static void ComputeTrsf(in Wire w, in Direction3D d, ref Box box, ref Transform3D tf)
    {
        // Calculate approximate barycenter
        WireExplorer exp = new(w);
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool BT;
        Xyz bary = new(0.0, 0.0, 0.0);
        int nb;

        for (nb = 0; exp.More(); exp.Next())
        {
            //    Bary += BT.Pnt(Exp.CurrentVertex()).XYZ();
            bary += Tool.Pnt(exp.CurrentVertex()).XYZ();
            nb++;
        }
        bary /= nb;

        // Calculate the Transformation
        CoordinateSystem n = new(new Point3D(bary), d);
        tf = tf.WithTransformation(n);
        EdgeCurveAdaptor ac = new();
        //  BndLib_Add3dCurve BC;

        // transformation to the wire
        Wire theW = w;
        Placement loc = new(tf);
        theW.Location(loc);

        // Calculate the box
        box.SetVoid();
        for (exp.Init(theW); exp.More(); exp.Next())
        {
            ac.Initialize(exp.Current());
            //    BC.Add(AC, 0.1, Box);
            Add3dCurve.Add(ac, 0.1, ref box);
        }
    }

    //=================================================================================================

    private static double Longueur(in Box wBox, in Box sBox, ref Direction3D d, ref Point3D p)
    {
        // face of the box most remoted from the face input in
        // the direction of skin
        double xmin, ymin, zmin, xmax, ymax, zmax, wZmin, wZmax, l;

        //"coord" of the box
        wBox.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);
        wZmin = zmin;
        wZmax = zmax;

        sBox.Get(out xmin, out ymin, out zmin, out xmax, out ymax, out zmax);
        p = p.WithCoord((xmin + xmax) / 2, (ymin + ymax) / 2, zmax);

        if (zmax < wZmin)
        {
            // Skin in the wrong direction. Invert...
            d = d.Reversed();
            l = wZmax - zmin;
            p = p.WithZ(zmin);
        }
        else
        {
            l = zmax - wZmin;
        }
        return l;
    }

    //=======================================================================
    // function : GoodOrientation
    // purpose  : Check if the law is oriented to have an exterior skin
    //======================================================================
    private static bool GoodOrientation(in Box b, LocationLaw law, in Direction3D d)
    {
        double f, l, r, t = 0.0;

        b.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
        Point3D p1 = new(xmin, ymin, zmin), p2 = new(xmax, ymax, zmax);
        Vector3D v = new(p1, p2);

        law.CurvilinearBounds(law.LawCount, out f, out l);
        r = v.Magnitude() / l;

        int ii, ind;
        int nb = (int)(4 + (10 * r));
        r = l / nb;

        nb++; // Number of points

        NCollection_Array1<Point3D> pnts = new(1, nb);
        CurveAdaptorBase ac;
        Xyz bary = new(0.0, 0.0, 0.0);

        for (ii = 1; ii <= nb; ii++)
        {
            law.Parameter((ii - 1) * r, out ind, ref t);
            ac = law.Law(ind).GetCurve()!;
            ac.D0(t, out pnts[ii]);
            bary += pnts[ii].XYZ();
        }

        bary /= nb;
        Point3D centre = new(bary);
        Vector3D normal = new(d.XYZ());
        double angle = 0;
        Vector3D Ref = new(centre, pnts[1]);

        for (ii = 2; ii <= nb; ii++)
        {
            Vector3D R = new(centre, pnts[ii]);
            angle += Ref.AngleWithRef(R, normal);
            Ref = R;
        }

        return (angle >= 0);
    }

    //=================================================================================================

    /// <summary>
    /// The draft of the wire, face or shell <paramref name="s"/> in the direction <paramref name="dir"/> at the angle
    /// <paramref name="angle"/>.
    /// </summary>
    /// <exception cref="ConstructionErrorException">Another shape type, or a shell without a free boundary wire.</exception>
    public Draft(in Shape s, in Direction3D dir, double angle)
    {
        _loc = null;
        _sec = null;
        _faces = null;
        _sections = null;

        switch (s.ShapeType())
        {
            case ShapeType.Wire:
            {
                _wire = ShapeCasts.Wire(s);
                break;
            }
            case ShapeType.Face:
            {
                Iterator exp = new(s);
                _wire = ShapeCasts.Wire(exp.Value());
                break;
            }
            case global::Kerf.Topology.ShapeType.Shell:
            {
                NCollection_List<Shape> list = new();
                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> edgemap =
                    new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(s, ShapeType.Edge, ShapeType.Face, edgemap);
                int iedge, nbf;
                for (iedge = 1; iedge <= edgemap.Extent(); iedge++)
                {
                    Edge theEdge = ShapeCasts.Edge(edgemap.FindKey(iedge));
                    // skip degenerated edges
                    if (!Tool.Degenerated(theEdge))
                    {
                        nbf = edgemap[iedge].Extent();
                        if (nbf == 1)
                        {
                            list.Append(theEdge);
                        }
                    }
                }

                if (list.Extent() > 0)
                {
                    LibraryMakeWire mw = new();
                    mw.Add(list);
                    LibraryWireError err = mw.Error();
                    if (err == LibraryWireError.WireDone)
                    {
                        _wire = mw.InternalWire();
                    }
                    else
                    {
                        throw new ConstructionErrorException("BRepFill_Draft");
                    }
                }
                else
                {
                    throw new ConstructionErrorException("BRepFill_Draft");
                }
                break;
            }
            default:
                throw new ConstructionErrorException("BRepFill_Draft");
        }

        // Attention to closed non declared wires !
        if (!_wire.Closed())
        {
            Exploration.Vertices(_wire, out Vertex vf, out Vertex vl);
            if (vf.IsSame(vl))
            {
                // OCCT.NET: OCCT sets the flag on the caller's wire; a frozen one is replaced by a new wire with the same
                // edges (decision 1045).
                if (_wire.TShape()!.IsFrozen)
                {
                    Wire newWire = ShapeCasts.Wire(_wire.EmptyCopied());
                    BrepBuilder bb = new();
                    for (Iterator it = new(_wire, false, false); it.More(); it.Next())
                    {
                        bb.Add(newWire, it.Value());
                    }
                    _wire = newWire;
                }
                _wire.Closed(true);
            }
        }

        _angle = Math.Abs(angle);
        _dir = dir;
        _top = s;
        _done = false;
        _tol = 1.0e-4;
        _cont = C1;
        SetOptions();
        SetDraft();
    }

    //=================================================================================================

    /// <summary>Sets the transition style at the wire's corners and the angular control of the sweep.</summary>
    public void SetOptions(TransitionStyle style = TransitionStyle.Right, double min = 0.01, double max = 3.0)
    {
        _style = style;
        _angmin = min;
        _angmax = max;
    }

    //=================================================================================================

    /// <summary>True for an internal draft (the skin inside the wire).</summary>
    public void SetDraft(bool Internal = false)
    {
        _isInternal = Internal;
    }

    //=======================================================================
    // function :Perform
    // purpose  : calculate a surface of skinning
    //======================================================================
    /// <summary>Builds the draft up to the length <paramref name="lengthMax"/>.</summary>
    public void Perform(double lengthMax)
    {
        Surface? s = null;
        Box wBox = new(); //, SBox;
        Transform3D trsf = new();

        ComputeTrsf(_wire, _dir, ref wBox, ref trsf);
        Init(s, lengthMax, wBox);
        BuildShell(s);
        Sewing();
    }

    //=======================================================================
    // function :Perform
    // purpose  : calculate a surface of skinning
    //======================================================================
    /// <summary>Builds the draft up to the surface <paramref name="surface"/>.</summary>
    public void Perform(Surface surface, bool keepInsideSurface = true)
    {
        Box wBox = new(), sBox = new();
        Transform3D trsf = new();
        Point3D pt = new();
        double l;

        ComputeTrsf(_wire, _dir, ref wBox, ref trsf);

        // box with bounds of the stop surface
        Surface surf;
        surf = (Surface)surface.Transformed(trsf);
        SurfaceAdaptor s1 = new(surf);
        //  BndLib_AddSurface AS;
        //  AS.Add(S1, 0.1, SBox);
        AddSurface.Add(s1, 0.1, ref sBox);

        // calculate the maximum length of the rule.
        l = Longueur(wBox, sBox, ref _dir, ref pt);
        l /= Math.Abs(Math.Cos(_angle));

        // Construction
        Init(surface, l, wBox);
        BuildShell(surface, !keepInsideSurface);
        Sewing();
    }

    //================================================================
    // function :Perform
    // purpose  : calculate the surface of skinning, stopped by a shape
    //================================================================
    /// <summary>Builds the draft up to the shape <paramref name="stopShape"/>.</summary>
    public void Perform(in Shape stopShape, bool keepOutSide = true)
    {
        Box wBox = new(), sBox = new();
        Transform3D trsf = new();
        Point3D pt = new();
        double l;

        ComputeTrsf(_wire, _dir, ref wBox, ref trsf);

        // bounding box of the stop shape
        Box bSurf = new(); //, TheBox;
        double umin, umax, vmin, vmax;
        //  BRepTools B;
        //  BRep_Tool BT;
        Surface surf;

        //  BndLib_AddSurface AS;

        Explorer ex = new(stopShape, ShapeType.Face);

        sBox.SetVoid();
        while (ex.More())
        { // parse faces of the stop shape
            //    B.UVBounds(TopoDS::Face(Ex.Current()), Umin,Umax,Vmin,Vmax);
            BrepTools.UVBounds(ShapeCasts.Face(ex.Current()), out umin, out umax, out vmin, out vmax);
            surf = (Surface)Tool.Surface(ShapeCasts.Face(ex.Current()))!.Transformed(trsf);
            SurfaceAdaptor s1 = new(surf);
            // bounding box of the current face
            //    AS.Add(S1, Umin, Umax, Vmin, Vmax, 0.1, BSurf);
            AddSurface.Add(s1, umin, umax, vmin, vmax, 0.1, ref bSurf);
            sBox.Add(bSurf); // group boxes
            ex.Next();
        } // while_Ex

        // calculate the maximum length of the rule.
        l = Longueur(wBox, sBox, ref _dir, ref pt);
        l /= Math.Abs(Math.Cos(_angle));

        // surface of stop
        Transform3D inv;
        inv = trsf.Inverted(); // inverted transformation
        pt = pt.Transformed(inv); // coordinate in the absolute reference
        PlaneSurface plan = new(pt, _dir);
        surf = new TrimmedSurface(plan, -l, l, -l, l);

        // Sweeping and restriction
        Init(plan, l * 1.01, wBox);
        BuildShell(surf, false);
        Fuse(stopShape, keepOutSide);
        Sewing();
    }

    //=================================================================================================

    private void Init(Surface? surf, double length, in Box box)
    {
        bool b;

        // law of positioning
        LocationDraft loc = new(_dir, _angle);
        _loc = new DraftLaw(_wire, loc);

        b = GoodOrientation(box, _loc, _dir);

        if (_isInternal ^ (!b))
        {
            _angle = -_angle;
            loc.SetAngle(_angle);
            _loc = new DraftLaw(_wire, loc);
        }

        _loc.CleanLaw(_angmin); // Clean small discontinuities.

        // law of section
        // generating line is straight and parallel to binormal.
        Point3D p = new(0, 0, 0);
        Vector3D d = new(0.0, 1.0, 0.0);

        // Control of the orientation
        _loc.Law(1).GetDomain(out double f, out double l);
        Matrix3x3 m = new();

        Vector3D bid = new();
        _loc.Law(1).D0((f + l) / 2, ref m, ref bid);
        Direction3D bn = new(m.Column(2));

        double ang = _dir.Angle(bn);
        if (ang > M_PI / 2)
        {
            d = d.Reversed();
        }
        LineCurve3D L = new(p, new Direction3D(d));

        Curve3D tc = new TrimmedCurve3D(L, 0, length);

        LibraryMakeEdge me = new(tc);
        Edge eg = me.InternalEdge();

        LibraryMakeWire mw = new(eg);
        Wire g = mw.InternalWire();

        _sec = new ShapeLaw(g, true);
    }

    //=======================================================================
    // function : BuildShell
    // purpose  : Construction of the skinning surface
    //======================================================================
    private void BuildShell(Surface? surf, bool keepOutSide = false)
    {
        // construction of the surface
        Sweep sweep = new(_sec!, _loc!, true);
        sweep.SetTolerance(_tol);
        sweep.SetAngularControl(_angmin, _angmax);
        HashSet<Shape> dummy = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, NCollection_HArray2<Shape>> dummy2 = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, NCollection_HArray2<Shape>> dummy3 = new(ShapeHasher.Instance);
        sweep.Build(dummy, dummy2, dummy3, _style, _cont);
        if (sweep.IsDone)
        {
            _shape = sweep.Shape();
            _shell = ShapeCasts.Shell(_shape);
            _faces = sweep.SubShape();
            _sections = sweep.Sections();
            _done = true;
            // Control of the orientation
            bool @out = true;
            Explorer ex = new(_shell, ShapeType.Face);
            Face f;
            f = ShapeCasts.Face(ex.Current());
            FaceSurfaceAdaptor sf = new(f);
            double u, v;
            Vector3D V;
            u = sf.FirstUParameter();
            v = sf.FirstVParameter();
            sf.D1(u, v, out Point3D p, out Vector3D v1, out Vector3D v2);
            V = v1.Crossed(v2);
            if (f.Orientation() == Reversed)
            {
                V = V.Reversed();
            }
            if (V.Magnitude() > 1.0e-10)
            {
                @out = _dir.Angle(new Direction3D(V)) > M_PI / 2;
            }
            if (@out == _isInternal)
            {
                _shell.Reverse();
                _shape.Reverse();
            }
        }
        else
        {
            _done = false;
            return;
        }

        if (surf is not null)
        { // Add the face at end

            // Waiting the use of traces & restriction in BRepFill_Sweep
            // Make Fuse.
            LibraryMakeFace mkF = new();
            mkF.Init(surf, true, global::Kerf.Engine.Foundation.Precision.Confusion());
            Fuse(mkF.InternalFace(), keepOutSide);
        }
    }

    //=======================================================================
    // function : Fuse
    // purpose  : Boolean operation between the skin and the
    //           stop shape
    //======================================================================
    private bool Fuse(in Shape stopShape, bool keepOutSide)
    {
        BrepBuilder b = new();
        bool issolid = false;
        Solid sol1 = new(), sol2 = new();
        Containment state1 = Out, state2 = Out;

        if (_shape.ShapeType() == ShapeType.Solid)
        {
            sol1 = ShapeCasts.Solid(_shape);
            issolid = true;
        }
        else
        {
            b.MakeSolid(ref sol1);
            b.Add(sol1, _shape); // shell => solid (for fusion)
        }

        switch (stopShape.ShapeType())
        {
            case ShapeType.Compound:
            {
                Iterator it = new(stopShape);
                return Fuse(it.Value(), keepOutSide);
            }
            case ShapeType.Solid:
            {
                sol2 = ShapeCasts.Solid(stopShape);
                break;
            }
            case global::Kerf.Topology.ShapeType.Shell:
            {
                b.MakeSolid(ref sol2);
                b.Add(sol2, stopShape); // shell => solid (for fusion)
                break;
            }

            case ShapeType.Face:
            {
                Shell s = new();
                b.MakeShell(ref s);
                b.Add(s, stopShape);
                s.Closed(Tool.IsClosed(s));
                b.MakeSolid(ref sol2);
                b.Add(sol2, s); // shell => solid (for fusion)
                break;
            }

            default:
            {
                return false; // Impossible to do
            }
        }

        // Perform intersection of solids
        PaveFiller pf = new();
        NCollection_List<Shape> anArgs = new();
        anArgs.Append(sol1);
        anArgs.Append(sol2);
        pf.SetArguments(anArgs);
        pf.Perform();
        if (pf.HasErrors)
        {
            return false;
        }

        SectionOperation sec = new(sol1, sol2, pf);
        Shape section = sec.InternalShape();

        Explorer exp = new(section, ShapeType.Edge);
        if (!exp.More())
        {
            // No section edges produced
            return false;
        }

        if (stopShape.ShapeType() != ShapeType.Solid)
        {
            // It is required to choose the state by the geometry

            // We need to find the section edge, closest to myWire
            Edge seMin = new();
            double dmin = global::Kerf.Engine.Foundation.Precision.Infinite();
            DistanceShapeShape distTool = new();
            distTool.LoadS1(_wire);

            for (; exp.More(); exp.Next())
            {
                Shape se = exp.Current();
                distTool.LoadS2(se);
                distTool.Perform();
                if (distTool.IsDone)
                {
                    double d = distTool.Value();
                    if (d < dmin)
                    {
                        dmin = d;
                        seMin = ShapeCasts.Edge(se);
                        if (dmin < global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            break;
                        }
                    }
                }
            }

            if (!seMin.IsNull)
            {
                // Get geometry of StopShape
                Surface? s = null;
                Curve2D? c2d = null;
                double f = 0.0, l = 0.0;
                Placement L = new();
                Tool.CurveOnSurface(seMin, ref c2d, ref s, ref L, ref f, ref l, 2);

                // Find a normal.
                // OCCT dereferences the second pcurve without a check.
                c2d!.D0((f + l) / 2, out Point2D p2d);
                SLProps sp = new(s!, p2d.X(), p2d.Y(), 1, 1.0e-12);
                if (!sp.IsNormalDefined)
                {
                    c2d.D0((3 * f + l) / 4, out p2d);
                    sp.SetParameters(p2d.X(), p2d.Y());
                    if (!sp.IsNormalDefined)
                    {
                        c2d.D0((f + 3 * l) / 4, out p2d);
                        sp.SetParameters(p2d.X(), p2d.Y());
                    }
                }

                if (sp.IsNormalDefined)
                {
                    // Subtract State1
                    if (_dir.Angle(sp.Normal()) < M_PI / 2)
                    {
                        state1 = In;
                    }
                    else
                    {
                        state1 = Out;
                    }
                }
            }
        }

        if (!keepOutSide)
        { // Invert State2;
            if (state2 == In)
            {
                state2 = Out;
            }
            else
            {
                state2 = In;
            }
        }

        // Perform Boolean operation
        BooleanBuilder builder = new();
        builder.AddArgument(sol1);
        builder.AddArgument(sol2);
        builder.PerformWithFiller(pf);
        if (builder.HasErrors)
        {
            return false;
        }

        Shape result = new();
        History history = new();

        bool isSingleOpNeeded = true;
        // To get rid of the unnecessary parts of first solid make the cutting first
        if (state1 == Out)
        {
            NCollection_List<Shape> lo = new(), lt = new();
            lo.Append(sol1);
            lt.Append(sol2);
            builder.BuildBOP(lo, lt, Operation.Cut, new ProgressRange());
            if (!builder.HasErrors)
            {
                Solid cutMin = new();
                Explorer expS = new(builder.InternalShape(), ShapeType.Solid);
                if (expS.More())
                {
                    cutMin = ShapeCasts.Solid(expS.Current());
                    expS.Next();
                    if (expS.More())
                    {
                        double dMin = global::Kerf.Engine.Foundation.Precision.Infinite();
                        DistanceShapeShape distTool = new();
                        distTool.LoadS1(_wire);

                        for (expS.ReInit(); expS.More(); expS.Next())
                        {
                            Shape cut = expS.Current();
                            distTool.LoadS2(cut);
                            distTool.Perform();
                            if (distTool.IsDone)
                            {
                                double d = distTool.Value();
                                if (d < dMin)
                                {
                                    dMin = d;
                                    cutMin = ShapeCasts.Solid(cut);
                                }
                            }
                        }
                    }
                }

                if (!cutMin.IsNull)
                {
                    // Save history for first argument only
                    history.Merge(lo, new BuilderHistory(builder));

                    // Perform needed operation with result of Cut
                    BooleanBuilder gluer = new();
                    gluer.AddArgument(cutMin);
                    gluer.AddArgument(sol2);
                    gluer.SetGlue(GlueMode.GlueShift);
                    gluer.Perform();

                    lo.Clear();
                    lo.Append(cutMin);
                    gluer.BuildBOP(lo, state1, lt, state2, new ProgressRange());

                    if (!gluer.HasErrors)
                    {
                        history.Merge(gluer.InternalHistory());

                        result = gluer.InternalShape();
                        expS.Init(result, ShapeType.Solid);
                        isSingleOpNeeded = !expS.More();
                    }
                }
            }
        }

        if (isSingleOpNeeded)
        {
            history.Clear();

            NCollection_List<Shape> lo = new(), lt = new();
            lo.Append(sol1);
            lt.Append(sol2);

            builder.BuildBOP(lo, state1, lt, state2, new ProgressRange());
            if (builder.HasErrors)
            {
                return false;
            }

            history.Merge(builder.InternalHistory());
            result = builder.InternalShape();
        }

        if (issolid)
        {
            _shape = result;
        }
        else
        {
            Explorer Exp = new();
            Exp.Init(result, global::Kerf.Topology.ShapeType.Shell);
            if (Exp.More())
            {
                _shape = Exp.Current();
            }
        }

        // Update the History
        int ii;
        for (ii = 1; ii <= _loc!.LawCount; ii++)
        {
            var L = history.Modified(_faces!.Value(1, ii));
            if (L.Count > 0)
            {
                _faces.SetValue(1, ii, L[0]);
            }
        }
        for (ii = 1; ii <= _loc.LawCount + 1; ii++)
        {
            var L = history.Modified(_sections!.Value(1, ii));
            if (L.Count > 0)
            {
                _sections.SetValue(1, ii, L[0]);
            }
        }

        return true;
    }

    // OCCT builds a BRepTools_History from any algorithm with Modified, Generated and IsDeleted (a template); the
    // port's history takes the interface, which the Boolean builder doesn't implement.
    private sealed class BuilderHistory(BuilderShape theAlgo) : History.IAlgorithm
    {
        public bool IsDeleted(in Shape s) => theAlgo.IsDeleted(s);

        public IEnumerable<Shape> Modified(in Shape s) => theAlgo.Modified(s);

        public IEnumerable<Shape> Generated(in Shape s) => theAlgo.Generated(s);
    }

    //=======================================================================
    // function : Sewing
    // purpose  : Assemble the skin with the above face
    //======================================================================
    private bool Sewing()
    {
        bool toAss;
        bool ok = false;
        toAss = (_top.ShapeType() != ShapeType.Wire);

        if ((!toAss) || (!_done))
        {
            return false;
        }

        // Assembly make a shell from the faces of the shape + the input shape
        Sewing ass = new(5 * _tol, true, true, false);
        ass.Add(_shape);
        ass.Add(_top);
        toAss = true;

        int nbCE;

        ass.Perform();
        // Check if the assembly is real.
        nbCE = ass.ContigousEdgeCount;

        if (nbCE > 0)
        {
            Shape res;
            res = ass.InternalSewedShape();
            if ((res.ShapeType() == global::Kerf.Topology.ShapeType.Shell) || (res.ShapeType() == ShapeType.Solid))
            {
                _shape = res;
                ok = true;
            }
            else if (res.ShapeType() == ShapeType.Compound)
            {
                Iterator it = new(res);
                res = it.Value();
                it.Next();
                if (!it.More())
                { // Only one part => this is correct
                    _shape = res;
                    ok = true;
                }
            }
        }

        if (ok)
        {
            // Update the History
            int ii;
            for (ii = 1; ii <= _loc!.LawCount; ii++)
            {
                if (ass.IsModified(_faces!.Value(1, ii)))
                {
                    _faces.SetValue(1, ii, ass.InternalModified(_faces.Value(1, ii)));
                }
            }
            for (ii = 1; ii <= _loc.LawCount + 1; ii++)
            {
                if (ass.IsModified(_sections!.Value(1, ii)))
                {
                    _sections.SetValue(1, ii, ass.InternalModified(_sections.Value(1, ii)));
                }
            }

            if (_shape.Closed())
            { // Make a Solid
                Solid solid = new();
                BrepBuilder bs = new();
                bs.MakeSolid(ref solid);
                bs.Add(solid, ShapeCasts.Shell(_shape));

                SolidClassifier sc = new(solid);
                sc.PerformInfinitePoint(global::Kerf.Engine.Foundation.Precision.Confusion());
                if (sc.State() == In)
                {
                    bs.MakeSolid(ref solid);
                    _shape.Reverse();
                    bs.Add(solid, ShapeCasts.Shell(_shape));
                }
                _shape = solid;
            }
        }
        return ok;
    }

    //=======================================================================
    // function : Generated
    // purpose  : return a sub-part generated by sweeping
    //======================================================================
    /// <summary>The face generated from an edge of the wire (frozen).</summary>
    public ISchematiqList<Shape> Generated(in Shape s)
    {
        NCollection_List<Shape> list = InternalGenerated(s);
        var shapes = new List<Shape>();
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            it.Value().TShape()?.FreezeGraph();
            shapes.Add(it.Value());
        }
        return PackedList.Own([.. shapes]);
    }

    /// <summary><see cref="Generated"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalGenerated(in Shape s)
    {
        _generated.Clear();
        Edge e;
        int ii;
        e = ShapeCasts.Edge(s);
        if (e.IsNull)
        {
            // OCCT: E is null here, so no vertex matches (a vertex's S throws in TopoDS::Edge above).
            for (ii = 0; ii <= _loc!.LawCount; ii++)
            {
                if (e.IsSame(_loc.Vertex(ii)))
                {
                    _generated.Append(_sections!.Value(1, ii + 1));
                    break;
                }
            }
        }
        else
        {
            for (ii = 1; ii <= _loc!.LawCount; ii++)
            {
                if (e.IsSame(_loc.Edge(ii)))
                {
                    _generated.Append(_faces!.Value(1, ii));
                    break;
                }
            }
        }

        return _generated;
    }

    //=================================================================================================

    /// <summary>The result (frozen): the skin sewn to the input face or shell, as a shell or solid.</summary>
    public Shape Shape()
    {
        _shape.TShape()?.FreezeGraph();
        return _shape;
    }

    internal Shape InternalShape() => _shape;

    //=====================================================================
    // function : Shell
    // purpose  : surface of skinning with the input face (=>shell)
    //=====================================================================
    /// <summary>The draft skin alone (frozen).</summary>
    public Shell Shell()
    {
        _shell.TShape()?.FreezeGraph();
        return _shell;
    }

    internal Shell InternalShell() => _shell;

    //=================================================================================================

    /// <summary>True if the draft was built.</summary>
    public bool IsDone => _done;
}
