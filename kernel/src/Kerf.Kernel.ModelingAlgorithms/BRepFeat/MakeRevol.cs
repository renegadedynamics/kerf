// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevol.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevol.lxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_MakeRevol.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Features.PerfSelection;
using static Kerf.Engine.Features.StatusError;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features;

/// <summary>
/// Describes functions to build revolved features: a profile revolved around an axis, as a depression (a cut) or a
/// protrusion (a fusion).
/// </summary>
/// <remarks>
/// <para>The mode is 0 for a depression, 1 for a protrusion, 2 for the feature alone. The revolution goes for an angle,
/// up to a face, from a face up to another, all around, or up to a face at an angle.</para>
/// <para>The basis shape and the profile are read, never written: the revolution sweeps a copy of a frozen profile
/// (LocOpe_Revol, decision 1014).</para>
/// </remarks>
public sealed class MakeRevol : Form
{
    private Shape _pbase = new();
    private readonly Dictionary<Shape, NCollection_List<Shape>> _slface = new(ShapeHasher.Instance);
    private Axis3D _axis = new();
    private readonly NCollection_Sequence<Curve3D> _curves = new();
    private Curve3D? _bCurve;
    // OCCT: the class declares its own myStatusError, hiding BRepFeat_Form's; CurrentStatusError reads the form's
#pragma warning disable CS0414 // written, never read: as in OCCT
    private StatusError _statusError;
#pragma warning restore CS0414

    /// <summary>An empty revolved feature.</summary>
    public MakeRevol()
    {
        _statusError = StatusError.OK;
    }

    /// <summary>
    /// A revolved feature of the profile <paramref name="pbase"/> on <paramref name="sbase"/> around
    /// <paramref name="axis"/>, from the sketch face <paramref name="skface"/>; <paramref name="fuse"/> is the mode
    /// (0 cut, 1 fuse, 2 the feature alone).
    /// </summary>
    public MakeRevol(in Shape sbase, in Shape pbase, in Face skface, in Axis3D axis,
                              int fuse, bool modify)
    {
        Init(sbase, pbase, skface, axis, fuse, modify);
    }

    /// <summary>Initializes the revolved feature (see the constructor).</summary>
    public void Init(in Shape sbase, in Shape pbase, in Face skface, in Axis3D axis, int mode,
                     bool modify)
    {
        // OCCT.NET: the working copies of frozen inputs (decision 1052)
        StartWorkingShapes();
        _axis = axis;
        _pbase = Working(pbase);
        _sbase = Working(sbase);
        BasisShapeValid();
        _skface = Working(skface);
        SketchFaceValid();
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

    /// <summary>Builds a revolved feature of the angle <paramref name="angle"/> (radians).</summary>
    public void Perform(double angle)
    {
        _sFrom.Nullify();
        ShapeFromValid();
        _sUntil.Nullify();
        ShapeUntilValid();
        _gluedF.Clear();
        _perfSelection = PerfSelection.NoSelection;
        PerfSelectionValid();
        bool revolComp = 2 * M_PI - Math.Abs(angle) <= global::Kerf.Engine.Foundation.Precision.Angular();
        LocalRevol theRevol = new();
        double angledec = 0.0;
        Explorer exp = new();
        if (revolComp)
        {
            _skface.Nullify();
        }
        if (angledec == 0.0)
        {
            theRevol.Perform(_pbase, _axis, angle);
        }
        else
        {
            theRevol.Perform(_pbase, _axis, angle, angledec);
        }
        Shape vraiRevol = theRevol.InternalShape();

        MajMap(_pbase, theRevol, _map, ref _fShape, ref _lShape);

        _gShape = vraiRevol;
        GeneratedShapeValid();
        Shape Base = theRevol.InternalFirstShape();
        exp.Init(Base, ShapeType.Face);
        Face theBase = ShapeCasts.Face(exp.Current());
        exp.Next();
        if (exp.More())
        {
            NotDone();
            _statusError = StatusError.InvFirstShape;
            return;
        }

        Face fFace = new();

        bool found = false;

        if (!_skface.IsNull || _slface.Count != 0)
        {
            if (_lShape.ShapeType() == ShapeType.Wire)
            {
                Explorer ex1 = new(vraiRevol, ShapeType.Face);
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

            Explorer anExp = new(_sbase, ShapeType.Face);
            for (; anExp.More(); anExp.Next())
            {
                Face ff = ShapeCasts.Face(anExp.Current());
                if (ToFuse(ff, fFace))
                {
                    if (!fFace.IsSame(_pbase) && global::Kerf.Engine.Features.BRepFeat.IsInside(ff, fFace))
                    {
                        break;
                    }
                }
            }
        }

        GluedFacesValid();
        if (!_skface.IsNull)
        {
            VerifGluedFaces(_skface, theBase, ref _bCurve, _curves, theRevol, _gluedF);
        }

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
            theRevol.Curves(_curves);
            _bCurve = theRevol.BarycCurve();
            GlobalPerform();
        }
    }

    /// <summary>Builds a revolved feature up to the shape <paramref name="until"/>.</summary>
    /// <exception cref="ConstructionErrorException">Until is null or has no face.</exception>
    public void Perform(in Shape until)
    {
        double angle = 0.0;
        bool tourComplet = false;

        if (until.IsNull)
        {
            throw new ConstructionErrorException();
        }
        Explorer exp = new(until, ShapeType.Face);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }
        if (!_skface.IsNull && Working(until).IsSame(_skface))
        {
            angle = 2 * M_PI;
            tourComplet = true;
        }
        _gluedF.Clear();
        _perfSelection = PerfSelection.SelectionU;
        PerfSelectionValid();
        _sFrom.Nullify();
        ShapeFromValid();
        _sUntil = Working(until);
        bool trf = TransformShapeFU(1);
        ShapeUntilValid();

        // Do systematically almost complete revolution
        LocalRevol theRevol = new();
        if (!tourComplet)
        {
            angle = 2.0 * M_PI - 3 * M_PI / 180.0;
        }
        theRevol.Perform(_pbase, _axis, angle);
        Shape vraiRevol = theRevol.InternalShape();
        MajMap(_pbase, theRevol, _map, ref _fShape, ref _lShape);

        if (!trf)
        {
            _gShape = vraiRevol;
            GeneratedShapeValid();

            Shape Base = theRevol.InternalFirstShape();
            exp.Init(Base, ShapeType.Face);
            exp.Next();
            if (exp.More())
            {
                NotDone();
                _statusError = StatusError.InvFirstShape;
                return;
            }

            GluedFacesValid();
            theRevol.Curves(_curves);
            _bCurve = theRevol.BarycCurve();
            GlobalPerform();
        }
        else
        {
            NCollection_Sequence<Curve3D?> scur = new();
            theRevol.Curves(_curves);
            _bCurve = theRevol.BarycCurve();
            scur.Clear();
            scur.Append(_bCurve);
            CSIntersector asi = new(_sUntil);
            asi.Perform(scur);
            if (asi.IsDone && asi.NbPoints(1) >= 1)
            {
                Orientation or = asi.Point(1, 1).Orientation();
                Face fUntil = asi.Point(1, 1).Face();
                Compound comp = new();
                BrepBuilder b = new();
                b.MakeCompound(ref comp);
                Solid s = global::Kerf.Engine.Features.BRepFeat.Tool(_sUntil, fUntil, or);
                if (!s.IsNull)
                {
                    b.Add(comp, s);
                }
                Cut trP = new(vraiRevol, comp);
                Shape cutsh = trP.InternalShape();
                Explorer ex = new(cutsh, ShapeType.Solid);
                for (; ex.More(); ex.Next())
                {
                    Explorer ex1 = new(ex.Current(), ShapeType.Face);
                    for (; ex1.More(); ex1.Next())
                    {
                        Face fac = ShapeCasts.Face(ex1.Current());
                        if (fac.IsSame(_pbase))
                        {
                            vraiRevol = ex.Current();
                            break;
                        }
                    }
                }
                if (_fuse)
                {
                    Fuse f = new(_sbase, vraiRevol);
                    _shape = f.InternalShape();
                    UpdateDescendants(f, _shape, false);
                    Done();
                }
                else
                {
                    Cut c = new(_sbase, vraiRevol);
                    _shape = c.InternalShape();
                    UpdateDescendants(c, _shape, false);
                    Done();
                }
            }
        }
    }

    /// <summary>Builds a revolved feature from the shape <paramref name="From"/> up to the shape <paramref name="until"/>.</summary>
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
                _axis = _axis.Reversed();
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

        LocalRevol theRevol = new();
        theRevol.Perform(_pbase, _axis, 2 * M_PI);
        Shape vraiRevol = theRevol.InternalShape();

        MajMap(_pbase, theRevol, _map, ref _fShape, ref _lShape);

        if (!trff)
        {
            _gShape = vraiRevol;
            GeneratedShapeValid();
            GluedFacesValid();
            theRevol.Curves(_curves);
            _bCurve = theRevol.BarycCurve();
            GlobalPerform();
        }
        else
        {
            theRevol.Curves(_curves);
            _bCurve = theRevol.BarycCurve();
            NCollection_Sequence<Curve3D?> scur = new();
            scur.Clear();
            scur.Append(_bCurve);
            CSIntersector asi1 = new(_sUntil);
            CSIntersector asi2 = new(_sFrom);
            asi1.Perform(scur);
            asi2.Perform(scur);
            Orientation orU, orF;
            Face fFrom, fUntil;
            double prF, prU;
            if (asi1.IsDone && asi1.NbPoints(1) >= 1)
            {
                orU = asi1.Point(1, 1).Orientation();
                fUntil = asi1.Point(1, 1).Face();
                prU = asi1.Point(1, 1).Parameter();
            }
            else
            {
                NotDone();
                _statusError = StatusError.NoIntersectU;
                return;
            }
            if (asi2.IsDone && asi2.NbPoints(1) >= 1)
            {
                double pr1 = asi2.Point(1, 1).Parameter();
                pr1 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(pr1, prU - 2 * M_PI, prU);
                double pr2 = asi2.Point(1, asi2.NbPoints(1)).Parameter();
                pr2 = global::Kerf.Engine.Geometry.ElCLib.InPeriod(pr2, prU - 2 * M_PI, prU);
                orF = global::Kerf.Engine.Topology.TopAbs.Reverse(orU);
                fFrom = asi2.Point(1, 1).Face();
                prF = Max(pr1, pr2);
            }
            else
            {
                NotDone();
                _statusError = StatusError.NoIntersectF;
                return;
            }
            if (!(prU > prF))
            {
                NotDone();
                _statusError = StatusError.IncParameter;
                return;
            }

            Compound comp = new();
            BrepBuilder b = new();
            b.MakeCompound(ref comp);
            Solid sf = global::Kerf.Engine.Features.BRepFeat.Tool(_sFrom, fFrom, orF);
            if (!sf.IsNull)
            {
                b.Add(comp, sf);
            }
            Solid su = global::Kerf.Engine.Features.BRepFeat.Tool(_sUntil, fUntil, orU);
            if (!su.IsNull)
            {
                b.Add(comp, su);
            }
            Cut trP = new(vraiRevol, comp);
            Shape cutsh = trP.InternalShape();
            Explorer ex = new(cutsh, ShapeType.Solid);
            vraiRevol = ex.Current();
            for (; ex.More(); ex.Next())
            {
                double prCur = global::Kerf.Engine.Features.BRepFeat.ParametricBarycenter(ex.Current(), _bCurve!);
                if (prF <= prCur && prU >= prCur)
                {
                    vraiRevol = ex.Current();
                    break;
                }
            }
            if (_fuse && !_justFeat)
            {
                Fuse f = new(_sbase, vraiRevol);
                _shape = f.InternalShape();
                UpdateDescendants(f, _shape, false);
                Done();
            }
            else if (!_fuse && !_justFeat)
            {
                Cut c = new(_sbase, vraiRevol);
                _shape = c.InternalShape();
                UpdateDescendants(c, _shape, false);
                Done();
            }
            else
            {
                _shape = vraiRevol;
                Done();
            }
        }
    }

    /// <summary>Builds a revolved feature all around the axis.</summary>
    public void PerformThruAll()
    {
        Perform(2.0 * M_PI);
    }

    /// <summary>Builds a revolved feature up to the shape <paramref name="until"/>, at the angle <paramref name="angle"/>.</summary>
    /// <exception cref="ConstructionErrorException">Until has no face (a null Until included, see below).</exception>
    public void PerformUntilAngle(in Shape until, double angle)
    {
        // OCCT: neither of the two first calls returns: a null Until goes on and throws below, a zero angle goes on and
        // builds the revolution again with the angle 0
        if (until.IsNull)
        {
            Perform(angle);
        }
        if (angle == 0)
        {
            Perform(until);
        }
        Explorer exp = new(until, ShapeType.Face);
        if (!exp.More())
        {
            throw new ConstructionErrorException();
        }

        if (!_skface.IsNull && Working(until).IsSame(_skface))
        {
            Perform(angle);
            return;
        }

        _gluedF.Clear();
        _perfSelection = PerfSelection.NoSelection;
        PerfSelectionValid();
        _sFrom.Nullify();
        ShapeFromValid();
        _sUntil = Working(until);
        bool trf = TransformShapeFU(1);
        ShapeUntilValid();

        // Produce systematicallt an almost complete revolution
        LocalRevol theRevol = new();
        theRevol.Perform(_pbase, _axis, angle);
        Shape vraiRevol = theRevol.InternalShape();

        MajMap(_pbase, theRevol, _map, ref _fShape, ref _lShape);

        // OCCT: the branches are the other way round from Perform(Until): a face until is cut, a support glued
        if (trf)
        {
            _gShape = vraiRevol;
            GeneratedShapeValid();

            Shape Base = theRevol.InternalFirstShape();
            exp.Init(Base, ShapeType.Face);
            exp.Next();
            if (exp.More())
            {
                NotDone();
                _statusError = StatusError.InvFirstShape;
                return;
            }

            GluedFacesValid();
            theRevol.Curves(_curves);
            _bCurve = theRevol.BarycCurve();
            GlobalPerform();
        }
        else
        {
            NCollection_Sequence<Curve3D?> scur = new();
            theRevol.Curves(_curves);
            _bCurve = theRevol.BarycCurve();
            scur.Clear();
            scur.Append(_bCurve);
            CSIntersector asi = new(_sUntil);
            asi.Perform(scur);
            if (asi.IsDone && asi.NbPoints(1) >= 1)
            {
                Orientation or = asi.Point(1, 1).Orientation();
                Face fUntil = asi.Point(1, 1).Face();
                Compound comp = new();
                BrepBuilder b = new();
                b.MakeCompound(ref comp);
                Solid s = global::Kerf.Engine.Features.BRepFeat.Tool(_sUntil, fUntil, or);
                if (!s.IsNull)
                {
                    b.Add(comp, s);
                }
                Cut trP = new(vraiRevol, comp);
                Shape cutsh = trP.InternalShape();
                Explorer ex = new(cutsh, ShapeType.Solid);
                for (; ex.More(); ex.Next())
                {
                    Explorer ex1 = new(ex.Current(), ShapeType.Face);
                    for (; ex1.More(); ex1.Next())
                    {
                        Face fac = ShapeCasts.Face(ex1.Current());
                        if (fac.IsSame(_pbase))
                        {
                            vraiRevol = ex.Current();
                            break;
                        }
                    }
                }
                if (_fuse)
                {
                    Fuse f = new(_sbase, vraiRevol);
                    _shape = f.InternalShape();
                    UpdateDescendants(f, _shape, false);
                    Done();
                }
                else
                {
                    Cut c = new(_sbase, vraiRevol);
                    _shape = c.InternalShape();
                    UpdateDescendants(c, _shape, false);
                    Done();
                }
            }
        }
    }

    /// <summary>The curves of the revolution.</summary>
    public override void Curves(NCollection_Sequence<Curve3D> scur)
    {
        scur.Assign(_curves);
    }

    /// <summary>The circle of the revolution through the barycentre of the profile.</summary>
    public override Curve3D? BarycCurve()
    {
        return _bCurve;
    }

    // Checks whether the gluing faces are valid: if not, the case without gluing
    private static void VerifGluedFaces(in Face skface, in Shape pbase, ref Curve3D? bCurve,
                                        NCollection_Sequence<Curve3D> curves, LocalRevol revol,
                                        Dictionary<Shape, Shape> map)
    {
        bool gluedFaces = true;
        Shape vraiRevol = revol.InternalShape();

        NCollection_Sequence<Curve3D?> scur = new();
        revol.Curves(curves);
        bCurve = revol.BarycCurve();
        scur.Clear();
        scur.Append(bCurve);
        CSIntersector asi = new(skface);
        asi.Perform(scur);
        if (asi.IsDone && asi.NbPoints(1) >= 1)
        {
            Orientation or = asi.Point(1, 1).Orientation();
            Face fSk = asi.Point(1, 1).Face();
            Compound comp = new();
            BrepBuilder b = new();
            b.MakeCompound(ref comp);
            Solid s = global::Kerf.Engine.Features.BRepFeat.Tool(skface, fSk, or);
            if (!s.IsNull)
            {
                b.Add(comp, s);
            }
            Cut trP = new(vraiRevol, comp);
            Shape cutsh = trP.InternalShape();
            Explorer ex = new(cutsh, ShapeType.Solid);
            for (; ex.More(); ex.Next())
            {
                Explorer ex1 = new(ex.Current(), ShapeType.Face);
                for (; ex1.More(); ex1.Next())
                {
                    Face fac1 = ShapeCasts.Face(ex1.Current());
                    Explorer ex2 = new(pbase, ShapeType.Face);
                    for (; ex2.More(); ex2.Next())
                    {
                        Face fac2 = ShapeCasts.Face(ex2.Current());
                        if (fac1.IsSame(fac2))
                        {
                            break;
                        }
                    }
                    if (ex2.More())
                    {
                        break;
                    }
                }
                if (ex1.More())
                {
                    continue;
                }
                gluedFaces = false;
                break;
            }
            if (!gluedFaces)
            {
                map.Clear();
            }
        }
    }

    private static void MajMap(in Shape b, LocalRevol p,
                               Dictionary<Shape, NCollection_List<Shape>> map,
                               ref Shape fShape, ref Shape lShape)
    {
        Explorer exp = new(p.InternalFirstShape(), ShapeType.Wire);
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

        for (exp.Init(b, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (!map.ContainsKey(exp.Current()))
            {
                map[exp.Current()] = new NCollection_List<Shape>(p.InternalShapes(exp.Current()));
            }
        }
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

        s1 = Tool.Surface(f1, out _)!;
        s2 = Tool.Surface(f2, out _)!;

        if (s1.GetType() == typeof(TrimmedSurface))
        {
            s1 = ((TrimmedSurface)s1).BasisSurface();
        }

        if (s2.GetType() == typeof(TrimmedSurface))
        {
            s2 = ((TrimmedSurface)s2).BasisSurface();
        }

        if (s1.GetType() != s2.GetType())
        {
            return false;
        }

        bool valRet = false;
        if (s1.GetType() == typeof(PlaneSurface))
        {
            s1 = Tool.Surface(f1)!; // to apply the location.
            s2 = Tool.Surface(f2)!;
            // OCCT: a trimmed plane gives a null plane here (an access violation natively)
            Plane3D pl1 = (s1 as PlaneSurface)!.Pln();
            Plane3D pl2 = (s2 as PlaneSurface)!.Pln();

            if (pl1.Position().IsCoplanar(pl2.Position(), tollin, tolang))
            {
                valRet = true;
            }
        }

        return valRet;
    }
}
