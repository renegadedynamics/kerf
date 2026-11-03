// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeDPrism.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeDPrism.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Engine.Features.PerfSelection;
using static Kerf.Engine.Features.StatusError;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features;

/// <summary>
/// Describes functions to build draft prism topologies from basis shape surfaces: a planar face swept along its normal
/// with a taper angle, as a depression (a cut) or a protrusion (a fusion).
/// </summary>
/// <remarks>
/// <para>The mode is 0 for a depression, 1 for a protrusion, 2 for the feature alone. The draft prism goes for a
/// height, up to a face, from a face up to another, through all, up to the end of the shape, from the end up to a face,
/// or up to a face at a height.</para>
/// <para>The basis shape and the profile are read, never written: the draft prism evolves a copy of a frozen profile
/// (LocOpe_DPrism, decision 1014).</para>
/// </remarks>
public sealed class MakeDraftPrism : Form
{
    private Face _pbase = new();
    private readonly Dictionary<Shape, NCollection_List<Shape>> _slface = new(ShapeHasher.Instance);
    private double _angle;
    private readonly NCollection_Sequence<Curve3D> _curves = new();
    private Curve3D? _bCurve;
    private NCollection_List<Shape> _topEdges = new();
    private NCollection_List<Shape> _latEdges = new();
    // OCCT: the class declares its own myStatusError, hiding BRepFeat_Form's; CurrentStatusError reads the form's
#pragma warning disable CS0414 // written, never read: as in OCCT
    private StatusError _statusError;
#pragma warning restore CS0414

    /// <summary>
    /// A draft prism of the planar face <paramref name="pbase"/> on <paramref name="sbase"/>, with the taper
    /// <paramref name="angle"/> (radians), from the sketch face <paramref name="skface"/>; <paramref name="fuse"/> is
    /// the mode (0 cut, 1 fuse, 2 the feature alone).
    /// </summary>
    public MakeDraftPrism(in Shape sbase, in Face pbase, in Face skface, double angle, int fuse,
                               bool modify)
    {
        Init(sbase, pbase, skface, angle, fuse, modify);
    }

    /// <summary>An empty draft prism.</summary>
    public MakeDraftPrism()
    {
        _angle = RealLast();
        _statusError = StatusError.OK;
    }

    /// <summary>Initializes the draft prism (see the constructor).</summary>
    public void Init(in Shape sbase, in Face pbase, in Face skface, double angle, int mode,
                     bool modify)
    {
        // OCCT.NET: the working copies of frozen inputs (decision 1052)
        StartWorkingShapes();
        _skface = Working(skface);
        SketchFaceValid();
        _sbase = Working(sbase);
        BasisShapeValid();
        _pbase = Working(pbase);
        _slface.Clear();
        if (mode == 0)
        {
            _fuse = false;
            _justFeat = false;
        }
        else if (mode == 1)
        {
            _fuse = true;
            _justFeat = false;
        }
        else if (mode == 2)
        {
            _fuse = true;
            _justFeat = true;
        }
        _modify = modify;
        _justGluer = false;

        _shape.Nullify();
        _map.Clear();
        _fShape.Nullify();
        _lShape.Nullify();
        _topEdges.Clear();
        _latEdges.Clear();
        Explorer exp = new();
        for (exp.Init(_sbase, ShapeType.Face); exp.More(); exp.Next())
        {
            NCollection_List<Shape> thelist = new();
            _map[exp.Current()] = thelist;
            _map.GetValue(exp.Current()).Append(exp.Current());
        }
        _angle = angle;
        // OCCT_DEBUG trace left out
    }

    /// <summary>Adds an element of sliding: the edge <paramref name="e"/> of the profile slides on the face <paramref name="f"/>.</summary>
    /// <exception cref="ConstructionErrorException">F isn't a face of the basis shape, or E an edge of the profile.</exception>
    public void Add(in Edge e, in Face f)
    {
        Explorer exp = new();
        for (exp.Init(_sbase, ShapeType.Face); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(Working(f)))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }

        for (exp.Init(_pbase, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(Working(e)))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }

        if (!_slface.ContainsKey(f))
        {
            NCollection_List<Shape> thelist = new();
            _slface[f] = thelist;
        }
        NCollection_List<Shape>.Iterator itl = new(_slface.GetValue(f));
        for (; itl.More(); itl.Next())
        {
            if (itl.Value().IsSame(e))
            {
                break;
            }
        }
        if (!itl.More())
        {
            _slface.GetValue(f).Append(e);
        }
    }

    /// <summary>Builds a draft prism of the height <paramref name="height"/>, and the topological reconstruction.</summary>
    public void Perform(double height)
    {
        _sFrom.Nullify();
        ShapeFromValid();
        _sUntil.Nullify();
        ShapeUntilValid();
        _gluedF.Clear();
        _perfSelection = PerfSelection.NoSelection;
        PerfSelectionValid();

        double theheight = height / Math.Cos(_angle);
        DPrism theDPrism = new(_pbase, theheight, _angle);
        Shape vraiDPrism = theDPrism.InternalShape();

        MajMap(_pbase, theDPrism, _map, ref _fShape, ref _lShape);

        _gShape = vraiDPrism;
        GeneratedShapeValid();

        Shape Base = theDPrism.InternalFirstShape();
        Explorer exp = new(Base, ShapeType.Face);
        exp.Next();
        if (exp.More())
        {
            NotDone();
            _statusError = StatusError.InvFirstShape;
            return;
        }

        // management of gluing faces

        GluedFacesValid();

        if (_gluedF.Count != 0)
        { // case gluing
            _justGluer = true;
            theDPrism.Curves(_curves);
            _bCurve = theDPrism.BarycCurve();
            GlobalPerform();
        }

        // if there is no gluing -> call topological operations
        if (!_justGluer)
        {
            if (_fuse)
            {
                Fuse f = new(_sbase, _gShape);
                _shape = f.InternalShape();
                UpdateDescendants(f, _shape, false);
                Done();
            }
            else
            {
                Cut c = new(_sbase, _gShape);
                _shape = c.InternalShape();
                UpdateDescendants(c, _shape, false);
                Done();
            }
        }
    }

    /// <summary>Builds a draft prism up to the shape <paramref name="until"/>.</summary>
    /// <exception cref="ConstructionErrorException">Until is null or has no face.</exception>
    public void Perform(in Shape until)
    {
        if (until.IsNull)
        {
            throw new ConstructionErrorException();
        }
        Explorer exp = new(until, ShapeType.Face);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }

        _gluedF.Clear();
        _perfSelection = PerfSelection.SelectionU;
        PerfSelectionValid();
        _sFrom.Nullify();
        ShapeFromValid();
        _sUntil = Working(until);
        bool trf = TransformShapeFU(1);
        ShapeUntilValid();
        Curve3D? C = TestCurve(_pbase);
        int sens = SensOfPrism(C, _sUntil);

        BrepBuilder bB = new();
        double height = sens * HeightMax(_sbase, _skface, _sFrom, _sUntil);
        DPrism theDPrism = new(_pbase, height, _angle);
        Shape vraiDPrism = theDPrism.InternalShape();
        if (!trf)
        {
            MajMap(_pbase, theDPrism, _map, ref _fShape, ref _lShape);
            _gShape = vraiDPrism;
            GeneratedShapeValid();
            Shape Base = theDPrism.InternalFirstShape();
            exp.Init(Base, ShapeType.Face);
            exp.Next();
            if (exp.More())
            {
                NotDone();
                _statusError = StatusError.InvFirstShape;
                return;
            }

            GluedFacesValid();
            theDPrism.Curves(_curves);
            _bCurve = theDPrism.BarycCurve();
            GlobalPerform();
        }
        else
        {
            MajMap(_pbase, theDPrism, _map, ref _fShape, ref _lShape);
            Curve3D? c1;
            if (sens == -1)
            {
                c1 = C!.Reversed();
            }
            else
            {
                c1 = C;
            }
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(c1);
            CSIntersector asi = new(_sUntil);
            asi.Perform(scur);
            Orientation or;
            if (asi.IsDone && asi.NbPoints(1) >= 1)
            {
                if (_fuse)
                {
                    or = asi.Point(1, 1).Orientation();
                }
                else
                {
                    or = asi.Point(1, asi.NbPoints(1)).Orientation();
                }
                Face fUntil = asi.Point(1, 1).Face();
                Compound comp = new();
                bB.MakeCompound(ref comp);
                Solid s = global::Kerf.Engine.Features.BRepFeat.Tool(_sUntil, fUntil, or);
                if (!s.IsNull)
                {
                    bB.Add(comp, s);
                }

                Cut trP = new(vraiDPrism, comp);
                UpdateDescendants(trP, trP.InternalShape(), false);
                Explorer ex = new(trP.InternalShape(), ShapeType.Solid);
                Shape cutsh = ex.Current();
                if (_fuse)
                {
                    Fuse f = new(_sbase, cutsh);
                    _shape = f.InternalShape();
                    UpdateDescendants(f, _shape, false);
                    Done();
                }
                else
                {
                    Cut c = new(_sbase, cutsh);
                    _shape = c.InternalShape();
                    UpdateDescendants(c, _shape, false);
                    Done();
                }
            }
        }
        // The new edges are the gluing's own edges of the result (unfrozen), written in place as in OCCT
        NCollection_List<Shape>.Iterator ited = new(_newEdges);
        for (; ited.More(); ited.Next())
        {
            Edge ledg = ShapeCasts.Edge(ited.Value());
            if (!global::Kerf.Engine.Booleans.Algo.BRepAlgo.IsValid(ledg))
            {
                bB.SameRange(ledg, false);
                bB.SameParameter(ledg, false);
                BrepLibrary.SameParameter(ledg, Tool.Tolerance(ledg));
            }
        }
    }

    /// <summary>Builds a draft prism from the shape <paramref name="From"/> up to the shape <paramref name="until"/>.</summary>
    /// <exception cref="ConstructionErrorException">From or Until is null or has no face.</exception>
    public void Perform(in Shape From, in Shape until)
    {
        if (From.IsNull || until.IsNull)
        {
            throw new ConstructionErrorException();
        }

        if (!_skface.IsNull)
        {
            if (Working(From).IsSame(_skface))
            {
                _justGluer = true;
                Perform(until);
                if (_justGluer)
                {
                    return;
                }
            }
            else if (Working(until).IsSame(_skface))
            {
                _justGluer = true;
                Perform(From);
                if (_justGluer)
                {
                    return;
                }
            }
        }

        _gluedF.Clear();
        _perfSelection = PerfSelection.SelectionFU;
        PerfSelectionValid();

        Explorer exp = new(From, ShapeType.Face);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }
        exp.Init(until, ShapeType.Face);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }
        _sFrom = Working(From);
        bool trff = TransformShapeFU(0);
        ShapeFromValid();
        _sUntil = Working(until);
        bool trfu = TransformShapeFU(1);
        ShapeUntilValid();
        if (trfu != trff)
        {
            NotDone();
            _statusError = StatusError.IncTypes;
            return;
        }
        Curve3D? C = TestCurve(_pbase);
        int sens;
        if (From.IsSame(until))
        {
            sens = 1;
        }
        else
        {
            sens = SensOfPrism(C, _sUntil);
        }

        double height = sens * HeightMax(_sbase, _pbase, _sFrom, _sUntil);
        DPrism theDPrism = new(_pbase, height, height, _angle);
        Shape vraiDPrism = theDPrism.InternalShape();
        if (!trff)
        {
            MajMap(_pbase, theDPrism, _map, ref _fShape, ref _lShape);

            // Make systematically dprism
            _gShape = vraiDPrism;
            GeneratedShapeValid();

            // management of gluing faces
            GluedFacesValid();
            theDPrism.Curves(_curves);
            _bCurve = theDPrism.BarycCurve();
            // topologic reconstruction
            GlobalPerform();
        }
        else
        {
            // management of descendants
            MajMap(_pbase, theDPrism, _map, ref _fShape, ref _lShape);
            Curve3D? c1;
            if (sens == -1)
            {
                c1 = C!.Reversed();
            }
            else
            {
                c1 = C;
            }
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(c1);
            CSIntersector asi1 = new(_sUntil);
            CSIntersector asi2 = new(_sFrom);
            asi1.Perform(scur);
            asi2.Perform(scur);
            Orientation orU, orF;
            Face fFrom, fUntil;
            // direction of dprism
            if (asi1.IsDone && asi1.NbPoints(1) >= 1)
            {
                if (_fuse)
                {
                    orU = asi1.Point(1, 1).Orientation();
                }
                else
                {
                    orU = asi1.Point(1, asi1.NbPoints(1)).Orientation();
                }
                fUntil = asi1.Point(1, 1).Face();
            }
            else
            {
                NotDone();
                _statusError = StatusError.NoIntersectU;
                return;
            }
            if (asi2.IsDone && asi2.NbPoints(1) >= 1)
            {
                orF = asi2.Point(1, 1).Orientation();
                orF = global::Kerf.Engine.Topology.TopAbs.Reverse(orF);
                fFrom = asi2.Point(1, 1).Face();
            }
            else
            {
                NotDone();
                _statusError = StatusError.NoIntersectF;
                return;
            }
            Compound comp = new();
            BrepBuilder b = new();
            b.MakeCompound(ref comp);
            Solid s = global::Kerf.Engine.Features.BRepFeat.Tool(_sUntil, fUntil, orU);
            if (!s.IsNull)
            {
                b.Add(comp, s);
            }
            else
            {
                NotDone();
                _statusError = StatusError.NullToolU;
                return;
            }
            Solid ss = global::Kerf.Engine.Features.BRepFeat.Tool(_sFrom, fFrom, orF);
            if (!ss.IsNull)
            {
                b.Add(comp, ss);
            }
            else
            {
                NotDone();
                _statusError = StatusError.NullToolF;
                return;
            }

            Cut trP = new(vraiDPrism, comp);
            if (_fuse)
            {
                Fuse f = new(_sbase, trP.InternalShape());
                _shape = f.InternalShape();
                UpdateDescendants(f, _shape, false);
                Done();
            }
            else
            {
                Cut c = new(_sbase, trP.InternalShape());
                _shape = c.InternalShape();
                UpdateDescendants(c, _shape, false);
                Done();
            }
        }
    }

    /// <summary>Builds a draft prism up to the end of the basis shape.</summary>
    public void PerformUntilEnd()
    {
        _perfSelection = PerfSelection.SelectionSh;
        PerfSelectionValid();
        _gluedF.Clear();
        _sUntil.Nullify();
        ShapeUntilValid();
        _sFrom.Nullify();
        ShapeFromValid();
        double height = HeightMax(_sbase, _skface, _sFrom, _sUntil);
        DPrism theDPrism = new(_pbase, height, _angle);
        Shape vraiDPrism = theDPrism.InternalShape();

        MajMap(_pbase, theDPrism, _map, ref _fShape, ref _lShape);
        _gShape = vraiDPrism;
        GeneratedShapeValid();
        GluedFacesValid();

        theDPrism.Curves(_curves);
        _bCurve = theDPrism.BarycCurve();
        GlobalPerform();
    }

    /// <summary>Builds a draft prism from the end of the basis shape up to the shape <paramref name="until"/>.</summary>
    /// <exception cref="ConstructionErrorException">Until is null or has no face.</exception>
    public void PerformFromEnd(in Shape until)
    {
        if (until.IsNull)
        {
            throw new ConstructionErrorException();
        }
        if (!_skface.IsNull && Working(until).IsSame(_skface))
        {
            PerformUntilEnd();
            return;
        }
        Explorer exp = new();
        exp.Init(until, ShapeType.Face);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }
        _perfSelection = PerfSelection.SelectionShU;
        PerfSelectionValid();
        _sFrom.Nullify();
        ShapeFromValid();
        _sUntil = Working(until);
        bool trf = TransformShapeFU(1);
        ShapeUntilValid();
        Curve3D? C = TestCurve(_pbase);
        int sens = SensOfPrism(C, _sUntil);
        double height = sens * HeightMax(_sbase, _skface, _sFrom, _sUntil);
        DPrism theDPrism = new(_pbase, height, height, _angle);
        Shape vraiDPrism = theDPrism.InternalShape();
        if (vraiDPrism.IsNull)
        {
            NotDone();
            _statusError = StatusError.NullRealTool;
            return;
        }

        if (!trf)
        { // case finite face
            MajMap(_pbase, theDPrism, _map, ref _fShape, ref _lShape);
            _gShape = vraiDPrism;
            GeneratedShapeValid();
            _gluedF.Clear();
            GluedFacesValid();
            theDPrism.Curves(_curves);
            _bCurve = theDPrism.BarycCurve();
            GlobalPerform();
        }
        else
        { // case support
            MajMap(_pbase, theDPrism, _map, ref _fShape, ref _lShape);
            Curve3D? c2;
            if (sens == -1)
            {
                c2 = C!.Reversed();
            }
            else
            {
                c2 = C;
            }
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(c2);
            CSIntersector asi1 = new(_sUntil);
            CSIntersector asi2 = new(_sbase);
            asi1.Perform(scur);
            asi2.Perform(scur);
            Orientation orU = Forward, orF = Forward;
            Face fUntil = new(), fFrom = new();
            if (asi1.IsDone && asi1.NbPoints(1) >= 1)
            {
                orU = asi1.Point(1, 1).Orientation();
                double prm = asi1.Point(1, 1).Parameter();
                if (prm < 0)
                {
                    orU = global::Kerf.Engine.Topology.TopAbs.Reverse(orU);
                }
                fUntil = asi1.Point(1, 1).Face();
            }

            if (asi2.IsDone && asi2.NbPoints(1) >= 1)
            {
                int jj = asi2.NbPoints(1);
                double prm = asi2.Point(1, 1).Parameter();
                fFrom = asi2.Point(1, 1).Face();
                orF = asi2.Point(1, 1).Orientation();
                orF = global::Kerf.Engine.Topology.TopAbs.Reverse(orF);
                for (int iii = 1; iii <= jj; iii++)
                {
                    if (asi2.Point(1, iii).Parameter() < prm)
                    {
                        prm = asi2.Point(1, iii).Parameter();
                        fFrom = asi2.Point(1, iii).Face();
                        orF = asi2.Point(1, iii).Orientation();
                        orF = global::Kerf.Engine.Topology.TopAbs.Reverse(orF);
                    }
                }
                Surface s = Tool.Surface(fFrom)!;
                if (s.GetType() == typeof(TrimmedSurface))
                {
                    s = ((TrimmedSurface)s).BasisSurface();
                }
                LibraryMakeFace fac = new(s, global::Kerf.Engine.Foundation.Precision.Confusion());
                _sFrom = fac.InternalFace();
                trf = TransformShapeFU(0);
            }

            Compound comp = new();
            BrepBuilder b = new();
            b.MakeCompound(ref comp);
            Solid sol = global::Kerf.Engine.Features.BRepFeat.Tool(_sUntil, fUntil, orU);
            if (!sol.IsNull)
            {
                b.Add(comp, sol);
            }
            else
            {
                NotDone();
                _statusError = StatusError.NullToolU;
                return;
            }

            Solid sol1 = global::Kerf.Engine.Features.BRepFeat.Tool(_sFrom, fFrom, orF);
            if (!sol1.IsNull)
            {
                b.Add(comp, sol1);
            }
            else
            {
                NotDone();
                _statusError = StatusError.NullToolF;
                return;
            }

            Cut trP = new(vraiDPrism, comp);
            if (_fuse)
            {
                Fuse f = new(_sbase, trP.InternalShape());
                _shape = f.InternalShape();
                UpdateDescendants(f, _shape, false);
                Done();
            }
            else
            {
                Cut c = new(_sbase, trP.InternalShape());
                _shape = c.InternalShape();
                UpdateDescendants(c, _shape, false);
                Done();
            }
        }
    }

    /// <summary>Builds a draft prism through all the basis shape.</summary>
    public void PerformThruAll()
    {
        _sUntil.Nullify();
        ShapeUntilValid();
        _sFrom.Nullify();
        ShapeFromValid();
        if (!_fuse)
        {
            _perfSelection = PerfSelection.NoSelection;
        }
        else
        {
            _perfSelection = PerfSelection.SelectionSh;
        }

        PerfSelectionValid();
        _gluedF.Clear();
        GluedFacesValid();

        double height = HeightMax(_sbase, _skface, _sFrom, _sUntil);
        DPrism theDPrism = new(_pbase, height, height, _angle);
        Shape vraiDPrism = theDPrism.InternalShape();
        MajMap(_pbase, theDPrism, _map, ref _fShape, ref _lShape);

        _gShape = vraiDPrism;
        GeneratedShapeValid();

        if (!_fuse)
        {
            Cut c = new(_sbase, _gShape);
            if (c.IsDone)
            {
                _shape = c.InternalShape();
                UpdateDescendants(c, _shape, false);
                Done();
            }
        }
        else
        {
            theDPrism.Curves(_curves);
            _bCurve = theDPrism.BarycCurve();
            GlobalPerform();
        }
    }

    /// <summary>Builds a draft prism up to the shape <paramref name="until"/>, at the height <paramref name="height"/>.</summary>
    /// <exception cref="ConstructionErrorException">Until has no face (a null Until included, see below).</exception>
    public void PerformUntilHeight(in Shape until, double height)
    {
        // OCCT: neither of the two first calls returns: a null Until goes on and throws below, a zero height goes on
        // and builds the draft prism again with the height 0
        if (until.IsNull)
        {
            Perform(height);
        }
        if (height == 0)
        {
            Perform(until);
        }
        Explorer exp = new(until, ShapeType.Face);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }
        _gluedF.Clear();
        _perfSelection = PerfSelection.NoSelection;
        PerfSelectionValid();
        _sFrom.Nullify();
        ShapeFromValid();
        _sUntil = Working(until);
        bool trf = TransformShapeFU(1);
        ShapeUntilValid();
        Curve3D? C = TestCurve(_pbase);
        int sens = SensOfPrism(C, _sUntil);
        DPrism theDPrism = new(_pbase, sens * height, _angle);
        Shape vraiDPrism = theDPrism.InternalShape();

        if (!trf)
        { // case face finished
            MajMap(_pbase, theDPrism, _map, ref _fShape, ref _lShape);
            _gShape = vraiDPrism;
            GeneratedShapeValid();
            Shape Base = theDPrism.InternalFirstShape();
            exp.Init(Base, ShapeType.Face);
            exp.Next();
            if (exp.More())
            {
                NotDone();
                _statusError = StatusError.InvFirstShape;
                return;
            }

            GluedFacesValid();
            theDPrism.Curves(_curves);
            _bCurve = theDPrism.BarycCurve();
            GlobalPerform();
        }
        else
        { // case support
            MajMap(_pbase, theDPrism, _map, ref _fShape, ref _lShape);
            Curve3D? c1;
            if (sens == -1)
            {
                c1 = C!.Reversed();
            }
            else
            {
                c1 = C;
            }
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(c1);
            CSIntersector asi = new(_sUntil);
            asi.Perform(scur);
            Orientation or;
            if (asi.IsDone && asi.NbPoints(1) >= 1)
            {
                if (_fuse)
                {
                    or = asi.Point(1, 1).Orientation();
                }
                else
                {
                    or = asi.Point(1, asi.NbPoints(1)).Orientation();
                }
                Face fUntil = asi.Point(1, 1).Face();
                Compound comp = new();
                BrepBuilder b = new();
                b.MakeCompound(ref comp);
                Solid s = global::Kerf.Engine.Features.BRepFeat.Tool(_sUntil, fUntil, or);
                if (!s.IsNull)
                {
                    b.Add(comp, s);
                }
                Cut trP = new(vraiDPrism, comp);
                if (_fuse)
                {
                    Fuse f = new(_sbase, trP.InternalShape());
                    _shape = f.InternalShape();
                    UpdateDescendants(f, _shape, false);
                    Done();
                }
                else
                {
                    Cut c = new(_sbase, trP.InternalShape());
                    _shape = c.InternalShape();
                    UpdateDescendants(c, _shape, false);
                    Done();
                }
            }
        }
    }

    /// <summary>The curves along the draft prism.</summary>
    public override void Curves(NCollection_Sequence<Curve3D> scur)
    {
        scur.Assign(_curves);
    }

    /// <summary>
    /// Determines the top and lateral edges of the boss: <paramref name="signature"/> 1 or -1 takes the first shape of
    /// the draft prism as its top, 2 or -2 its last shape; a negative signature takes the new edges as lateral edges.
    /// </summary>
    public void BossEdges(int signature)
    {
        NCollection_List<Shape> theLastShape = new();
        theLastShape.Clear();
        if (signature == 1 || signature == -1)
        {
            theLastShape = new NCollection_List<Shape>(InternalFirstShape());
        }
        else if (signature == 2 || signature == -2)
        {
            theLastShape = new NCollection_List<Shape>(InternalLastShape());
        }
        else
        {
            return;
        }

        // Edges Top
        NCollection_List<Shape>.Iterator itLS = new();
        Explorer expE = new();
        for (itLS.Initialize(theLastShape); itLS.More(); itLS.Next())
        {
            Face ff = ShapeCasts.Face(itLS.Value());
            for (expE.Init(ff, ShapeType.Edge); expE.More(); expE.Next())
            {
                Edge ee = ShapeCasts.Edge(expE.Current());
                _topEdges.Append(ee);
            }
        }

        // Edges Bottom
        if (signature < 0)
        {
            // Attention check if TgtEdges is important
            _latEdges = new NCollection_List<Shape>(InternalNewEdges());
        }
        else if (signature > 0)
        {
            if (!_shape.IsNull)
            {
                // OCCT iterates this address-keyed map into the lateral edges: the port in the order of its additions and
                // removals (decision 1054)
                HashSet<Shape> mapE = new(ShapeHasher.Instance);
                bool found;

                Explorer expF = new();
                for (expF.Init(_shape, ShapeType.Face); expF.More(); expF.Next())
                {
                    found = false;
                    Face ff = ShapeCasts.Face(expF.Current());
                    for (itLS.Initialize(theLastShape); itLS.More(); itLS.Next())
                    {
                        Face topFace = ShapeCasts.Face(itLS.Value());
                        if (!ff.IsSame(topFace))
                        {
                            for (expE.Init(ff, ShapeType.Edge); expE.More() && !found; expE.Next())
                            {
                                Edge e1 = ShapeCasts.Edge(expE.Current());
                                Exploration.Vertices(e1, out Vertex v1, out Vertex v2);
                                NCollection_List<Shape>.Iterator it = new(_topEdges);
                                for (; it.More() && !found; it.Next())
                                {
                                    Edge e2 = ShapeCasts.Edge(it.Value());
                                    Exploration.Vertices(e2, out Vertex vt1, out Vertex vt2);

                                    if (v1.IsSame(vt1) || v1.IsSame(vt2) || v2.IsSame(vt1) || v2.IsSame(vt2))
                                    {
                                        found = true;
                                        Explorer expE2 = new();
                                        for (expE2.Init(ff, ShapeType.Edge); expE2.More(); expE2.Next())
                                        {
                                            Edge e3 = ShapeCasts.Edge(expE2.Current());
                                            if (mapE.Contains(e3))
                                            {
                                                mapE.Remove(e3);
                                            }
                                            else
                                            {
                                                mapE.Add(e3);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                NCollection_List<Shape>.Iterator it2 = new(_topEdges);
                for (; it2.More(); it2.Next())
                {
                    if (mapE.Contains(it2.Value()))
                    {
                        mapE.Remove(it2.Value());
                    }
                }

                foreach (Shape itMap in mapE)
                {
                    if (!Tool.Degenerated(ShapeCasts.Edge(itMap)))
                    {
                        _latEdges.Append(itMap);
                    }
                }
            }
        }
    }

    /// <summary>The edges of the top of the boss (frozen).</summary>
    public ISchematiqList<Shape> TopEdges()
    {
        BeforePublishingTopology();
        return PublishedList(_topEdges);
    }

    /// <summary><see cref="TopEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalTopEdges() => _topEdges;

    /// <summary>The lateral edges of the boss (frozen).</summary>
    public ISchematiqList<Shape> LatEdges()
    {
        BeforePublishingTopology();
        return PublishedList(_latEdges);
    }

    /// <summary><see cref="LatEdges"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalLatEdges() => _latEdges;

    private void BeforePublishingTopology() => _shape.TShape()?.FreezeGraph();

    /// <summary>The curve along the draft prism through the barycentre of its profile.</summary>
    public override Curve3D? BarycCurve()
    {
        return _bCurve;
    }

    private static double HeightMax(in Shape sbase,  // shape initial
                                    in Face skface,  // face de sketch
                                    in Shape sFrom,  // shape from
                                    in Shape sUntil) // shape until
    {
        Box box = new();
        Bounds.Add(sbase, ref box);
        Bounds.Add(skface, ref box);
        if (!sFrom.IsNull)
        {
            Bounds.Add(sFrom, ref box);
        }
        if (!sUntil.IsNull)
        {
            Bounds.Add(sUntil, ref box);
        }

        double[] c = new double[6];

        box.Get(out c[0], out c[2], out c[4], out c[1], out c[3], out c[5]);
        double par = Max(Max(Math.Abs(c[1] - c[0]), Math.Abs(c[3] - c[2])), Math.Abs(c[5] - c[4]));
        return par;
    }

    private static int SensOfPrism(Curve3D? c, in Shape until)
    {
        CSIntersector asi1 = new(until);
        NCollection_Sequence<Curve3D?> scur = new();
        scur.Append(c);
        asi1.Perform(scur);
        int sens = 1;
        if (asi1.IsDone && asi1.NbPoints(1) >= 1)
        {
            int nb = asi1.NbPoints(1);
            double prm1 = asi1.Point(1, 1).Parameter();
            double prm2 = asi1.Point(1, nb).Parameter();
            if (prm1 < 0.0 && prm2 < 0.0)
            {
                sens = -1;
            }
        }
        else if (global::Kerf.Engine.Features.BRepFeat.ParametricBarycenter(until, c!) < 0)
        {
            sens = -1;
        }
        return sens;
    }

    private static void MajMap(in Shape b, DPrism p,
                               Dictionary<Shape, NCollection_List<Shape>> map,
                               ref Shape fShape, ref Shape lShape)
    {
        Explorer exp = new();
        if (!p.InternalFirstShape().IsNull)
        {
            exp.Init(p.InternalFirstShape(), ShapeType.Wire);
            if (exp.More())
            {
                fShape = exp.Current();
                NCollection_List<Shape> thelist = new();
                map[fShape] = thelist;
                for (exp.Init(p.InternalFirstShape(), ShapeType.Face); exp.More(); exp.Next())
                {
                    map.GetValue(fShape).Append(exp.Current());
                }
            }
        }

        if (!p.InternalLastShape().IsNull)
        {
            exp.Init(p.InternalLastShape(), ShapeType.Wire);
            if (exp.More())
            {
                lShape = exp.Current();
                NCollection_List<Shape> thelist1 = new();
                map[lShape] = thelist1;
                for (exp.Init(p.InternalLastShape(), ShapeType.Face); exp.More(); exp.Next())
                {
                    map.GetValue(lShape).Append(exp.Current());
                }
            }
        }

        for (exp.Init(b, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (!map.ContainsKey(exp.Current()))
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                map[edg] = new NCollection_List<Shape>(p.InternalShapes(edg));
            }
        }
    }

    private static Curve3D? TestCurve(in Face Base)
    {
        Point3D bar = new(0.0, 0.0, 0.0);
        NCollection_Sequence<Point3D> spt = new();
        global::Kerf.Engine.Features.Local.LocOpe.SampleEdges(Base, spt);
        for (int jj = 1; jj <= spt.Length(); jj++)
        {
            Point3D pvt = spt[jj];
            bar = new Point3D(bar.XYZ() + pvt.XYZ());
        }
        bar = new Point3D(bar.XYZ().Divided(spt.Length()));
        Surface s = Tool.Surface(Base)!;
        if (s.GetType() == typeof(TrimmedSurface))
        {
            s = ((TrimmedSurface)s).BasisSurface();
        }
        if (s is not PlaneSurface p)
        {
            return null;
        }
        Plane3D pp = p.Pln();
        Direction3D normale = pp.Position().XDirection() ^ pp.Position().YDirection();
        Axis3D theAx = new(bar, normale);
        LineCurve3D theLin = new(theAx);
        return theLin;
    }
}
