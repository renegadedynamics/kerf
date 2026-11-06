// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePrism.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePrism.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePrism.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Features.PerfSelection;
using static Kerf.Engine.Features.StatusError;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features;

/// <summary>
/// Describes functions to build prism features: a depression (a cut) or a protrusion (a fusion) along a direction, from
/// a basis face of a sketch face.
/// </summary>
/// <remarks>
/// <para>The mode is 0 for a depression, 1 for a protrusion, 2 for the feature alone (a protrusion not fused). The
/// prism goes for a length, up to a face, from a face up to another, through all, up to the end of the shape, from the
/// end up to a face, or up to a face at a height.</para>
/// <para>The basis shape and the profile are read, never written: the prism sweeps a copy of a frozen profile
/// (LocOpe_Prism, decision 1014).</para>
/// </remarks>
internal sealed class MakePrism : Form
{
    private Shape _pbase = new();
    private readonly Dictionary<Shape, NCollection_List<Shape>> _slface = new(ShapeHasher.Instance);
    private Direction3D _dir = new();
    private readonly NCollection_Sequence<Curve3D> _curves = new();
    private Curve3D? _bCurve;
    // OCCT: the class declares its own myStatusError, hiding BRepFeat_Form's; CurrentStatusError reads the form's

    /// <summary>Builds a prism by projecting a wire along the face of a shape.</summary>
    public MakePrism()
    {
    }

    /// <summary>
    /// Builds a prism of the profile <paramref name="pbase"/> on the basis shape <paramref name="sbase"/>, along
    /// <paramref name="direction"/>, from the sketch face <paramref name="skface"/>; <paramref name="fuse"/> is the mode
    /// (0 cut, 1 fuse, 2 the feature alone).
    /// </summary>
    public MakePrism(in Shape sbase, in Shape pbase, in Face skface, in Direction3D direction,
                              int fuse, bool modify)
    {
        Init(sbase, pbase, skface, direction, fuse, modify);
    }

    /// <summary>Initializes the prism (see the constructor).</summary>
    public void Init(in Shape sbase, in Shape pbase, in Face skface, in Direction3D direc, int mode,
                     bool modify)
    {
        // Kerf: the working overlay the feature writes into its inputs in (decision 1052, unit 10H5)
        StartWorkingShapes();
        using WorkingOverlay.Scope scope = EnterWork();
        _skface = skface;
        SketchFaceValid();
        _sbase = sbase;
        BasisShapeValid();
        _pbase = pbase;
        _slface.Clear();
        _dir = direc;
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
        _newEdges.Clear();
        _tgtEdges.Clear();
        _map.Clear();
        _fShape.Nullify();
        _lShape.Nullify();
        Explorer exp = new();
        for (exp.Init(_sbase, ShapeType.Face); exp.More(); exp.Next())
        {
            NCollection_List<Shape> thelist = new();
            _map[exp.Current()] = thelist;
            _map.GetValue(exp.Current()).Append(exp.Current());
        }
        // OCCT_DEBUG trace left out
    }

    /// <summary>Adds an element of sliding: the edge <paramref name="e"/> of the profile slides on the face <paramref name="f"/>.</summary>
    /// <exception cref="ConstructionErrorException">F isn't a face of the basis shape, or E an edge of the profile.</exception>
    public void Add(in Edge e, in Face f)
    {
        using WorkingOverlay.Scope scope = EnterWork(); // Kerf: the working overlay (unit 10H5)
        Explorer exp = new();
        for (exp.Init(_sbase, ShapeType.Face); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(f))
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
            if (exp.Current().IsSame(e))
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
            NCollection_List<Shape> thelist1 = new();
            _slface[f] = thelist1;
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

    /// <summary>Builds a prism of length <paramref name="length"/>, and the topological reconstruction.</summary>
    public void Perform(double length)
    {
        using WorkingOverlay.Scope scope = EnterWork(); // Kerf: the working overlay (unit 10H5)
        _sFrom.Nullify();
        ShapeFromValid();
        _sUntil.Nullify();
        ShapeUntilValid();
        _gluedF.Clear();
        _perfSelection = PerfSelection.NoSelection;
        PerfSelectionValid();
        Vector3D v = length * new Vector3D(_dir);

        // construction of prism of height Length

        LocalPrism thePrism = new(_pbase, v);
        Shape vraiPrism = thePrism.InternalShape();

        // management of descendants
        MajMap(_pbase, thePrism, _map, ref _fShape, ref _lShape);

        _gShape = vraiPrism; // the primitive
        GeneratedShapeValid();

        Face fFace = new();

        bool found = false;

        // try to detect the faces of gluing
        // in case if the top of the prism is tangent to the initial shape

        if (!_skface.IsNull || _slface.Count != 0)
        {
            if (_lShape.Type == ShapeType.Wire)
            {
                Explorer ex1 = new(vraiPrism, ShapeType.Face);
                for (; ex1.More(); ex1.Next())
                {
                    Explorer ex2 = new(ex1.Current(), ShapeType.Wire);
                    for (; ex2.More(); ex2.Next())
                    {
                        if (ex2.Current().IsSame(_lShape))
                        {
                            fFace = ShapeCasts.Face(ex1.Current());
                            found = true;
                            break;
                        }
                    }
                    if (found)
                    {
                        break;
                    }
                }
            }

            Explorer exp = new(_sbase, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                Face ff = ShapeCasts.Face(exp.Current());
                if (ToFuse(ff, fFace))
                {
                    if (!fFace.IsSame(_pbase) && global::Kerf.Engine.Features.BRepFeat.IsInside(ff, fFace))
                    {
                        break;
                    }
                }
            }
        }

        // management of faces of gluing given by the user

        GluedFacesValid();

        if (_gluedF.Count != 0)
        { // case gluing
            _justGluer = true;
            thePrism.Curves(_curves);
            _bCurve = thePrism.BarycCurve();
            GlobalPerform(); // topological reconstruction
        }

        // if there is no gluing  -> call of ope topo
        if (!_justGluer)
        {
            if (_fuse && !_justFeat)
            {
                Fuse f = new(_sbase, _gShape);
                _shape = f.InternalShape();
                UpdateDescendants(f, _shape, false);
                Done();
            }
            else if (!_fuse)
            {
                Cut c = new(_sbase, _gShape);
                _shape = c.InternalShape();
                UpdateDescendants(c, _shape, false);
                Done();
            }
            else
            {
                _shape = _gShape;
                Done();
            }
        }
    }

    /// <summary>Builds a prism up to the shape <paramref name="until"/>, and the topological reconstruction.</summary>
    /// <exception cref="ConstructionErrorException">Until is null or has no face.</exception>
    public void Perform(in Shape until)
    {
        using WorkingOverlay.Scope scope = EnterWork(); // Kerf: the working overlay (unit 10H5)
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
        _sUntil = until;
        bool trf = TransformShapeFU(1);
        ShapeUntilValid();
        Curve3D C = TestCurve(_pbase, new Vector3D(_dir));
        int sens = SensOfPrism(C, _sUntil);
        double height = HeightMax(_sbase, _skface, _sFrom, _sUntil);
        Vector3D v = 2 * sens * height * new Vector3D(_dir);

        // construction of long prism
        LocalPrism thePrism = new(_pbase, v);
        Shape vraiPrism = thePrism.InternalShape();

        // in case of support of face Until
        if (!trf)
        {
            MajMap(_pbase, thePrism, _map, ref _fShape, ref _lShape);
            _gShape = vraiPrism;
            GeneratedShapeValid();
            GluedFacesValid();
            thePrism.Curves(_curves);
            _bCurve = thePrism.BarycCurve();
            GlobalPerform();
        }
        else
        { // until support -> passage to topological operations
            MajMap(_pbase, thePrism, _map, ref _fShape, ref _lShape);
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(C);

            // direction of the prism depending on Until

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
                if (sens == -1)
                {
                    or = global::Kerf.Engine.Topology.TopAbs.Reverse(or);
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
                Cut trP = new(vraiPrism, comp);
                UpdateDescendants(trP, trP.InternalShape(), false);
                //
                Explorer ex = new(trP.InternalShape(), ShapeType.Solid);
                Shape cutsh = ex.Current();
                if (_fuse && !_justFeat)
                {
                    Fuse f = new(_sbase, cutsh);
                    _shape = f.InternalShape();
                    UpdateDescendants(f, _shape, false);
                    Done();
                }
                else if (!_fuse)
                {
                    Cut c = new(_sbase, cutsh);
                    _shape = c.InternalShape();
                    UpdateDescendants(c, _shape, false);
                    Done();
                }
                else
                {
                    _shape = cutsh;
                    Done();
                }
            }
        }
    }

    /// <summary>Builds a prism from the shape <paramref name="From"/> up to the shape <paramref name="until"/>.</summary>
    /// <exception cref="ConstructionErrorException">From or Until is null or has no face.</exception>
    public void Perform(in Shape From, in Shape until)
    {
        using WorkingOverlay.Scope scope = EnterWork(); // Kerf: the working overlay (unit 10H5)
        if (From.IsNull || until.IsNull)
        {
            throw new ConstructionErrorException();
        }

        if (!_skface.IsNull)
        {
            if (From.IsSame(_skface))
            {
                _justGluer = true;
                Perform(until);
                if (_justGluer)
                {
                    return;
                }
            }
            else if (until.IsSame(_skface))
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
        _sFrom = From;
        bool trff = TransformShapeFU(0);
        ShapeFromValid();
        _sUntil = until;
        bool trfu = TransformShapeFU(1);
        ShapeUntilValid();
        if (trfu != trff)
        {
            NotDone();
            return;
        }

        // length depending on bounding boxes

        double height = HeightMax(_sbase, _skface, _sFrom, _sUntil);
        Curve3D C = TestCurve(_pbase, new Vector3D(_dir));
        int sens; // direction of prism
        int tran; // transfer of prism
        if (From.IsSame(until))
        {
            sens = 1;
            tran = -1;
        }
        else
        {
            sens = SensOfPrism(C, _sUntil);
            tran = sens * SensOfPrism(C, _sFrom);
        }
        LocalPrism thePrism = new();
        if (tran < 0)
        {
            Vector3D vtra = -3 * height * sens / 2.0 * new Vector3D(_dir);
            thePrism.Perform(_pbase, 3 * sens * height * new Vector3D(_dir), vtra);
        }
        else
        {
            thePrism.Perform(_pbase, 2 * sens * height * new Vector3D(_dir));
        }
        Shape vraiPrism = thePrism.InternalShape();

        if (!trff)
        {
            MajMap(_pbase, thePrism, _map, ref _fShape, ref _lShape);

            _gShape = vraiPrism;
            GeneratedShapeValid();
            GluedFacesValid();
            thePrism.Curves(_curves);
            _bCurve = thePrism.BarycCurve();
            GlobalPerform();
        }
        else
        { // case until support -> topological operation
            MajMap(_pbase, thePrism, _map, ref _fShape, ref _lShape);
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(C);
            CSIntersector asi1 = new(_sUntil);
            CSIntersector asi2 = new(_sFrom);
            asi1.Perform(scur);
            asi2.Perform(scur);
            Orientation orU, orF;
            Face fFrom, fUntil;
            double parF, parU;
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
                if (sens == -1)
                {
                    orU = global::Kerf.Engine.Topology.TopAbs.Reverse(orU);
                }
                fUntil = asi1.Point(1, 1).Face();
                parU = asi1.Point(1, 1).Parameter();
            }
            else
            {
                NotDone();
                return;
            }
            if (asi2.IsDone && asi2.NbPoints(1) >= 1)
            {
                orF = asi2.Point(1, 1).Orientation();
                if (sens == 1)
                {
                    orF = global::Kerf.Engine.Topology.TopAbs.Reverse(orF);
                }
                fFrom = asi2.Point(1, 1).Face();
                parF = asi2.Point(1, 1).Parameter();
            }
            else
            {
                NotDone();
                return;
            }
            if (tran > 0 && (Math.Abs(parU) < Math.Abs(parF)))
            {
                Orientation or;
                or = orU;
                orU = orF;
                orF = or;
            }
            //
            NCollection_List<Shape> lTools = new();
            Solid s = global::Kerf.Engine.Features.BRepFeat.Tool(_sUntil, fUntil, orU);
            if (!s.IsNull)
            {
                lTools.Append(s);
            }
            else
            {
                NotDone();
                return;
            }
            Solid ss = global::Kerf.Engine.Features.BRepFeat.Tool(_sFrom, fFrom, orF);
            if (!ss.IsNull)
            {
                lTools.Append(ss);
            }
            else
            {
                NotDone();
                return;
            }
            //
            NCollection_List<Shape> lObj = new();
            lObj.Append(vraiPrism);
            //
            Cut trP = new();
            trP.SetArguments(lObj);
            trP.SetTools(lTools);
            trP.Build();
            UpdateDescendants(trP, trP.InternalShape(), false);
            if (_fuse && !_justFeat)
            {
                Fuse f = new(_sbase, trP.InternalShape());
                _shape = f.InternalShape();
                UpdateDescendants(f, _shape, false);
                Done();
            }
            else if (!_fuse)
            {
                Cut c = new(_sbase, trP.InternalShape());
                _shape = c.InternalShape();
                UpdateDescendants(c, _shape, false);
                Done();
            }
            else
            {
                _shape = trP.InternalShape();
                Done();
            }
        }
    }

    /// <summary>Builds a prism up to the end of the basis shape.</summary>
    public void PerformUntilEnd()
    {
        using WorkingOverlay.Scope scope = EnterWork(); // Kerf: the working overlay (unit 10H5)
        _perfSelection = PerfSelection.SelectionSh;
        PerfSelectionValid();
        _gluedF.Clear();
        _sUntil.Nullify();
        ShapeUntilValid();
        _sFrom.Nullify();
        ShapeFromValid();
        double height = HeightMax(_sbase, _skface, _sFrom, _sUntil);
        Vector3D v = 2 * height * new Vector3D(_dir);

        LocalPrism thePrism = new(_pbase, v);
        Shape vraiPrism = thePrism.InternalShape();

        MajMap(_pbase, thePrism, _map, ref _fShape, ref _lShape);

        _gShape = vraiPrism;
        GeneratedShapeValid();
        GluedFacesValid();

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
            thePrism.Curves(_curves);
            _bCurve = thePrism.BarycCurve();
            GlobalPerform();
        }
    }

    /// <summary>Builds a prism from the end of the basis shape up to the shape <paramref name="until"/>.</summary>
    /// <exception cref="ConstructionErrorException">Until is null or has no face.</exception>
    public void PerformFromEnd(in Shape until)
    {
        using WorkingOverlay.Scope scope = EnterWork(); // Kerf: the working overlay (unit 10H5)
        if (until.IsNull)
        {
            throw new ConstructionErrorException();
        }
        if (!_skface.IsNull && until.IsSame(_skface))
        {
            _dir = _dir.Reversed();
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
        _sUntil = until;
        bool trf = TransformShapeFU(1);
        ShapeUntilValid();
        Curve3D C = TestCurve(_pbase, new Vector3D(_dir));
        int sens = SensOfPrism(C, _sUntil);
        double height = HeightMax(_sbase, _skface, _sFrom, _sUntil);
        Vector3D vtra = -3 * height * sens / 2.0 * new Vector3D(_dir);
        Vector3D vect = 3 * sens * height * new Vector3D(_dir);
        LocalPrism thePrism = new(_pbase, vect, vtra);
        Shape vraiPrism = thePrism.InternalShape();

        if (!trf)
        { // case face until
            MajMap(_pbase, thePrism, _map, ref _fShape, ref _lShape);
            _gShape = vraiPrism;
            GeneratedShapeValid();
            _gluedF.Clear();
            GluedFacesValid();
            thePrism.Curves(_curves);
            _bCurve = thePrism.BarycCurve();
            GlobalPerform();
        }
        else
        { // case support
            MajMap(_pbase, thePrism, _map, ref _fShape, ref _lShape);
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(C);
            CSIntersector asi1 = new(_sUntil);
            CSIntersector asi2 = new(_sbase);
            asi1.Perform(scur);
            asi2.Perform(scur);
            Orientation orU = Forward, orF = Forward;
            Face fUntil = new(), fFrom = new();
            if (asi1.IsDone && asi1.NbPoints(1) >= 1)
            {
                orU = asi1.Point(1, 1).Orientation();
                if (sens == -1)
                {
                    orU = global::Kerf.Engine.Topology.TopAbs.Reverse(orU);
                }
                fUntil = asi1.Point(1, 1).Face();
            }
            if (asi2.IsDone && asi2.NbPoints(1) >= 1)
            {
                orF = asi2.Point(1, 1).Orientation();
                fFrom = asi2.Point(1, 1).Face();
                Surface s = Tool.Surface(fFrom)!;
                if (s.GetType() == typeof(TrimmedSurface))
                {
                    s = ((TrimmedSurface)s).BasisSurface;
                }
                LibraryMakeFace fac = new(s, global::Kerf.Engine.Foundation.Precision.Confusion());
                _sFrom = fac.InternalFace();
                trf = TransformShapeFU(0);
                fFrom = ShapeCasts.Face(_sFrom);
            }

            NCollection_List<Shape> lTools = new();
            Solid sol = global::Kerf.Engine.Features.BRepFeat.Tool(_sUntil, fUntil, orU);
            if (!sol.IsNull)
            {
                lTools.Append(sol);
            }
            else
            {
                NotDone();
                return;
            }

            Solid sol1 = global::Kerf.Engine.Features.BRepFeat.Tool(_sFrom, fFrom, orF);
            if (!sol1.IsNull)
            {
                lTools.Append(sol1);
            }
            else
            {
                NotDone();
                return;
            }
            //
            NCollection_List<Shape> lObj = new();
            lObj.Append(vraiPrism);
            //
            Cut trP = new();
            trP.SetArguments(lObj);
            trP.SetTools(lTools);
            trP.Build();
            //
            UpdateDescendants(trP, trP.InternalShape(), false);
            if (_fuse && !_justFeat)
            {
                Fuse f = new(_sbase, trP.InternalShape());
                _shape = f.InternalShape();
                UpdateDescendants(f, _shape, false);
                Done();
            }
            else if (!_fuse)
            {
                Cut c = new(_sbase, trP.InternalShape());
                _shape = c.InternalShape();
                UpdateDescendants(c, _shape, false);
                Done();
            }
            else
            {
                _shape = trP.InternalShape();
                Done();
            }
        }
    }

    /// <summary>Builds a prism through all the basis shape.</summary>
    public void PerformThruAll()
    {
        using WorkingOverlay.Scope scope = EnterWork(); // Kerf: the working overlay (unit 10H5)
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
        Vector3D v = 3 * height * new Vector3D(_dir);
        Vector3D vtra = -3 * height / 2.0 * new Vector3D(_dir);
        LocalPrism thePrism = new(_pbase, v, vtra);
        Shape vraiPrism = thePrism.InternalShape();
        MajMap(_pbase, thePrism, _map, ref _fShape, ref _lShape);

        _gShape = vraiPrism;
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
            thePrism.Curves(_curves);
            _bCurve = thePrism.BarycCurve();
            GlobalPerform();
        }
    }

    /// <summary>Builds a prism up to the shape <paramref name="until"/>, at the height <paramref name="length"/>.</summary>
    /// <exception cref="ConstructionErrorException">Until has no face (a null Until included, see below).</exception>
    public void PerformUntilHeight(in Shape until, double length)
    {
        using WorkingOverlay.Scope scope = EnterWork(); // Kerf: the working overlay (unit 10H5)
        // OCCT: neither of the two first calls returns: a null Until goes on and throws below, a zero length goes on
        // and builds the prism again with the length 0
        if (until.IsNull)
        {
            Perform(length);
        }
        if (length == 0)
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
        _sUntil = until;
        bool trf = TransformShapeFU(1);
        ShapeUntilValid();
        Curve3D C = TestCurve(_pbase, new Vector3D(_dir));
        int sens = SensOfPrism(C, _sUntil);
        Vector3D v = sens * length * new Vector3D(_dir);
        LocalPrism thePrism = new(_pbase, v);
        Shape vraiPrism = thePrism.InternalShape();

        if (!trf)
        {
            MajMap(_pbase, thePrism, _map, ref _fShape, ref _lShape);

            _gShape = vraiPrism;
            GeneratedShapeValid();
            GluedFacesValid();
            thePrism.Curves(_curves);
            _bCurve = thePrism.BarycCurve();
            GlobalPerform();
        }
        else
        {
            MajMap(_pbase, thePrism, _map, ref _fShape, ref _lShape);
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(C);
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
                if (sens == -1)
                {
                    or = global::Kerf.Engine.Topology.TopAbs.Reverse(or);
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

                Cut trP = new(vraiPrism, comp);
                UpdateDescendants(trP, trP.InternalShape(), false);
                if (_fuse && !_justFeat)
                {
                    Fuse f = new(_sbase, trP.InternalShape());
                    _shape = f.InternalShape();
                    UpdateDescendants(f, _shape, false);
                    Done();
                }
                else if (!_fuse)
                {
                    Cut c = new(_sbase, trP.InternalShape());
                    _shape = c.InternalShape();
                    UpdateDescendants(c, _shape, false);
                    Done();
                }
                else
                {
                    _shape = trP.InternalShape();
                    Done();
                }
            }
        }
    }

    /// <summary>The curves parallel to the axis of the prism.</summary>
    public override void Curves(NCollection_Sequence<Curve3D> scur)
    {
        scur.Assign(_curves);
    }

    /// <summary>The curve parallel to the axis of the prism through the centre of masses.</summary>
    public override Curve3D? BarycCurve()
    {
        return _bCurve;
    }

    // Calculate the height of the prism following the parameters of bounding box
    private static double HeightMax(in Shape sbase, in Face skface, in Shape sFrom,
                                    in Shape sUntil)
    {
        Box box = new();
        Bounds.Add(sbase, ref box);
        Bounds.Add(skface, ref box);
        if (!sFrom.IsNull)
        {
            bool facRevolInfini = false;
            Explorer exp = new();
            exp.Init(sFrom, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Explorer exp1 = new();
                exp1.Init(exp.Current(), ShapeType.Vertex);
                if (!exp1.More())
                {
                    facRevolInfini = true;
                    break;
                }
            }
            if (!facRevolInfini)
            {
                Bounds.Add(sFrom, ref box);
            }
        }
        if (!sUntil.IsNull)
        {
            bool facRevolInfini = false;
            Explorer exp = new();
            exp.Init(sUntil, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Explorer exp1 = new();
                exp1.Init(exp.Current(), ShapeType.Vertex);
                if (!exp1.More())
                {
                    facRevolInfini = true;
                    break;
                }
            }
            if (!facRevolInfini)
            {
                Bounds.Add(sUntil, ref box);
            }
        }

        double[] c = new double[6];

        box.Get(out c[0], out c[2], out c[4], out c[1], out c[3], out c[5]);
        double parmin = c[0], parmax = c[0];
        for (int i = 0; i < 6; i++)
        {
            if (c[i] > parmax)
            {
                parmax = c[i];
            }
            if (c[i] < parmin)
            {
                parmin = c[i];
            }
        }
        double height = Math.Abs(2.0 * (parmax - parmin));
        return height;
    }

    // Direction of the prism depending on the shape Until
    private static int SensOfPrism(Curve3D c, in Shape until)
    {
        CSIntersector asi1 = new(until);
        NCollection_Sequence<Curve3D?> scur = new();
        scur.Append(c);
        asi1.Perform(scur);
        int sens = 1;
        if (asi1.IsDone && asi1.NbPoints(1) >= 1)
        {
            if (asi1.Point(1, 1).Parameter() + global::Kerf.Engine.Foundation.Precision.Confusion() < 0.0
                && asi1.Point(1, asi1.NbPoints(1)).Parameter() + global::Kerf.Engine.Foundation.Precision.Confusion() < 0.0)
            {
                sens = -1;
            }
        }
        else if (global::Kerf.Engine.Features.BRepFeat.ParametricBarycenter(until, c) < 0)
        {
            sens = -1;
        }
        return sens;
    }

    private static void MajMap(in Shape b, LocalPrism p,
                               Dictionary<Shape, NCollection_List<Shape>> map,
                               ref Shape fShape, ref Shape lShape)
    {
        Explorer exp = new(p.InternalFirstShape(), ShapeType.Wire);
        if (exp.More())
        {
            fShape = exp.Current();
            NCollection_List<Shape> thelist2 = new();
            map[fShape] = thelist2;
            for (exp.Init(p.InternalFirstShape(), ShapeType.Face); exp.More(); exp.Next())
            {
                map.GetValue(fShape).Append(exp.Current());
            }
        }

        exp.Init(p.InternalLastShape(), ShapeType.Wire);
        if (exp.More())
        {
            lShape = exp.Current();
            NCollection_List<Shape> thelist3 = new();
            map[lShape] = thelist3;
            for (exp.Init(p.InternalLastShape(), ShapeType.Face); exp.More(); exp.Next())
            {
                map.GetValue(lShape).Append(exp.Current());
            }
        }

        for (exp.Init(b, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (!map.ContainsKey(exp.Current()))
            {
                map[exp.Current()] = new NCollection_List<Shape>(p.InternalShapes(exp.Current()));
            }
        }
    }

    private static Curve3D TestCurve(in Shape Base, in Vector3D v)
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
        Axis3D newAx = new(bar, new Direction3D(v));
        LineCurve3D theLin = new(newAx);
        return theLin;
    }

    private static bool ToFuse(in Face f1, in Face f2)
    {
        if (f1.IsNull || f2.IsNull)
        {
            return false;
        }

        Surface s1, s2;
        double tollin = global::Kerf.Engine.Foundation.Precision.Confusion();
        double tolang = global::Kerf.Engine.Foundation.Precision.Angular();

        s1 = Tool.Surface(f1, out Placement loc1)!;
        s2 = Tool.Surface(f2, out Placement loc2)!;

        if (s1.GetType() == typeof(TrimmedSurface))
        {
            s1 = ((TrimmedSurface)s1).BasisSurface;
        }

        if (s2.GetType() == typeof(TrimmedSurface))
        {
            s2 = ((TrimmedSurface)s2).BasisSurface;
        }

        if (s1.GetType() != s2.GetType())
        {
            return false;
        }

        bool valRet = false;
        if (s1.GetType() == typeof(PlaneSurface))
        {
            Plane3D pl1 = ((PlaneSurface)s1).Plane;
            Plane3D pl2 = ((PlaneSurface)s2).Plane;

            // apply locations
            if (!loc1.IsIdentity)
            {
                pl1 = pl1.Transformed(loc1.Transformation);
            }
            if (!loc2.IsIdentity)
            {
                pl2 = pl2.Transformed(loc2.Transformation);
            }

            if (pl1.Position.IsCoplanar(pl2.Position, tollin, tolang))
            {
                valRet = true;
            }
        }

        return valRet;
    }
}
