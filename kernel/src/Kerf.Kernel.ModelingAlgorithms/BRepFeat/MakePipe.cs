// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePipe.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePipe.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakePipe.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Features.PerfSelection;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features;

/// <summary>
/// Constructs compound shapes with pipe features: a profile swept along a spine, as a depression (a cut) or a
/// protrusion (a fusion).
/// </summary>
/// <remarks>
/// <para>The mode is 0 for a depression, 1 for a protrusion, 2 for the feature alone. The pipe goes along the whole
/// spine, up to a face, or from a face up to another.</para>
/// <para>The basis shape, profile and spine are read, never written: the pipe sweeps a copy of a frozen profile
/// (LocOpe_Pipe, decision 1014).</para>
/// </remarks>
public sealed class MakePipe : Form
{
    private Shape _pbase = new();
    // OCCT: the class declares its own mySkface, hiding BRepFeat_Form's, which stays null
    private new Face _skface = new();
    private readonly Dictionary<Shape, NCollection_List<Shape>> _slface = new(ShapeHasher.Instance);
    private Wire _spine = new();
    private NCollection_Sequence<Curve3D> _curves = new();
    private Curve3D? _bCurve;

    /// <summary>An empty pipe feature.</summary>
    public MakePipe()
    {
    }

    /// <summary>
    /// A pipe feature of the profile <paramref name="pbase"/> along <paramref name="spine"/> on
    /// <paramref name="sbase"/>, from the sketch face <paramref name="skface"/>; <paramref name="fuse"/> is the mode
    /// (0 cut, 1 fuse, 2 the feature alone).
    /// </summary>
    public MakePipe(in Shape sbase, in Shape pbase, in Face skface, in Wire spine,
                             int fuse, bool modify)
    {
        Init(sbase, pbase, skface, spine, fuse, modify);
    }

    /// <summary>Initializes the pipe feature (see the constructor).</summary>
    public void Init(in Shape sbase, in Shape pbase, in Face skface, in Wire spine, int mode,
                     bool modify)
    {
        // Kerf: the working overlay the feature writes into its inputs in (decision 1052, unit 10H5)
        StartWorkingShapes();
        using WorkingOverlay.Scope scope = EnterWork();
        _sbase = sbase;
        BasisShapeValid();
        _skface = skface;
        SketchFaceValid();
        _pbase = pbase;
        _slface.Clear();
        _spine = ShapeCasts.Wire(spine);
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

    /// <summary>Builds the pipe along the whole spine.</summary>
    public void Perform()
    {
        using WorkingOverlay.Scope scope = EnterWork(); // Kerf: the working overlay (unit 10H5)
        _sFrom.Nullify();
        ShapeFromValid();
        _sUntil.Nullify();
        ShapeUntilValid();
        _gluedF.Clear();
        _perfSelection = PerfSelection.NoSelection;
        PerfSelectionValid();
        Shape theBase = _pbase;
        LocalPipe thePipe = new(_spine, theBase);
        Shape vraiPipe = thePipe.InternalShape();
        MajMap(_pbase, thePipe, _map, ref _fShape, ref _lShape);
        _gShape = vraiPipe;
        GeneratedShapeValid();

        GluedFacesValid();

        if (_gluedF.Count == 0)
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
        else
        {
            _fShape = thePipe.InternalFirstShape();
            NCollection_Sequence<Point3D> spt = new();
            global::Kerf.Engine.Features.Local.LocOpe.SampleEdges(_fShape, spt);
            _curves = new NCollection_Sequence<Curve3D>(thePipe.Curves(spt)!);
            _bCurve = thePipe.BarycCurve();
            GlobalPerform();
        }
    }

    /// <summary>Builds the pipe up to the shape <paramref name="until"/>.</summary>
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
        TransformShapeFU(1);
        ShapeUntilValid();
        LocalPipe thePipe = new(_spine, _pbase);
        Shape vraiTuyau = thePipe.InternalShape();
        MajMap(_pbase, thePipe, _map, ref _fShape, ref _lShape);
        _gShape = vraiTuyau;
        GeneratedShapeValid();
        GluedFacesValid();
        // OCCT: myFShape becomes the pipe's first shape, which the map doesn't hold (FirstShape then throws)
        _fShape = thePipe.InternalFirstShape();
        NCollection_Sequence<Point3D> spt = new();
        global::Kerf.Engine.Features.Local.LocOpe.SampleEdges(_fShape, spt);
        _curves = new NCollection_Sequence<Curve3D>(thePipe.Curves(spt)!);
        _bCurve = thePipe.BarycCurve();
        GlobalPerform();
    }

    /// <summary>Builds the pipe from the shape <paramref name="From"/> up to the shape <paramref name="until"/>.</summary>
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
                Perform(until);
                return;
            }
            else if (until.IsSame(_skface))
            {
                Perform(From);
                return;
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
        TransformShapeFU(0);
        ShapeFromValid();
        _sUntil = until;
        TransformShapeFU(1);
        ShapeUntilValid();
        LocalPipe thePipe = new(_spine, _pbase);
        Shape vraiTuyau = thePipe.InternalShape();
        MajMap(_pbase, thePipe, _map, ref _fShape, ref _lShape);
        _gShape = vraiTuyau;
        GeneratedShapeValid();
        GluedFacesValid();
        _fShape = thePipe.InternalFirstShape();
        NCollection_Sequence<Point3D> spt = new();
        global::Kerf.Engine.Features.Local.LocOpe.SampleEdges(_fShape, spt);
        _curves = new NCollection_Sequence<Curve3D>(thePipe.Curves(spt)!);
        _bCurve = thePipe.BarycCurve();
        GlobalPerform();
    }

    /// <summary>The curves of the pipe through the sample points of its first shape.</summary>
    public override void Curves(NCollection_Sequence<Curve3D> scur)
    {
        scur.Assign(_curves);
    }

    /// <summary>The curve of the pipe through the barycentre of its first shape.</summary>
    public override Curve3D? BarycCurve()
    {
        return _bCurve;
    }

    private static void MajMap(in Shape b, LocalPipe p,
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
}
